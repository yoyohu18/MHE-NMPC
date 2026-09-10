#!/usr/bin/env python3
# MHE 在线诊断节点(开环,只估计、不反馈)。订阅跟 acados_nmpc_node 同一份里程计,
# 以及它新增发布的实际控制量 /acados_nmpc/u_opt,维护滑动窗口实时跑 MHE,
# 把质量估计发布到 /acados_nmpc/mhe_mass_estimate,纯诊断——不修改任何发给
# PX4 的指令,不碰已经验证过的 NMPC 控制逻辑。
#
# 已知简化:u_opt 里的力矩 tau 是 NMPC 算出来的"意图值",不是 PX4 内部姿态
# 速率环实际施加的力矩(我们对 PX4 内部那一层没有可见性)——质量估计主要靠
# vel_dot 里的 (1/m)*T 项,T(总推力)电机响应快、基本等于指令值,这一部分
# 应该是可信的;tau 的不精确主要影响窗口内姿态/角速度过程残差的拟合质量,
# 是质量估计的次要误差来源,不是主要的。

import math
import time

import numpy as np
import rclpy
from scipy.linalg import block_diag
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, Float64MultiArray
from actuator_msgs.msg import Actuators

from offboard_test.nmpc_node import quat_to_rotmat

from .mhe_params import p as mhe_p
from .mhe_solver_builder import ensure_mhe_ocp_solver
from .mhe_weight_learning import M0_THETA, ParametricWeightSchedule
from .payload_estimate import (
    PayloadEstimate,
    RELEASE_RATIO_THR,
    empty_evidence_scores,
    inertia_from_mass_moment,
    release_decision,
    release_evidence,
    no_payload_confidence,
    relative_error_percent,
)
from .residual_logger import ResidualLogger


# Gazebo MulticopterMotorModel 推力系数(x500_base/model.sdf 里每个电机的
# <motorConstant>,4 个电机相同)。单电机推力 = MOTOR_CONSTANT * ω²,总推力
# T_phys = MOTOR_CONSTANT * Σω²。ω 取 /x500_payload_0/command/motor_speed 的
# velocity 字段(rad/s)——实测确认是真实转速、未被 rotorVelocitySlowdownSim=10
# 缩放:drop 后物理 2.0kg 时实测 ω≈765 → 4*MOTOR_CONSTANT*765²≈20.0N≈2.04kg*g,
# 与真值吻合,所以直接用、不做任何缩放。
#
# 为什么这才是 MHE 该用的推力(关键):/acados_nmpc/u_opt[0] 是 NMPC 基于自己
# 的质量估计 m_est 算出来的"标量意图推力"(被 cost 钉在 m_est*g 附近),m_est
# 一偏离真值(典型就是 drop 之后),这个意图推力就 ≠ 飞机真实受力,MHE 拿它当
# 已知输入会陷入"NMPC的T → MHE自洽回m_est → NMPC的T"的盲区,估不出质量阶跃。
# 改用电机转速反算的真实物理推力(经 PX4/电机真实非线性映射、与 m_est 完全
# 解耦),MHE 看到的才是"真实力 vs 实测加速度",质量始终可观测。
# (真机迁移:换成 ESC 转速遥测 / 推力台 RPM→推力曲线作数据源。)
MOTOR_CONSTANT = 8.54858e-06

# 标定增益 = 1.0:SDF 名义 motorConstant 就是对的,不需要任何修正(2026-07-02
# 钉死)。曾经在这里放过 1.2134——那是按"飞机满载 2.5kg"这个错误前提反标出来的
# 幽灵增益:mass_changer 插件在 Configure 阶段 SetInertial(2.5) 和运行时一样,
# 也从未进到 DART 物理引擎(gz-sim #2733 同一机制,组件写了、gz model 读得到,
# 但刚体按 SDF 原值 2.064kg 建),飞机全程其实是 x500_base 的 2.064kg。四组独立
# 数据零自由参数互证:满载悬停原始反算 20.19N=2.064g✓、wrench drop 后 15.25N=
# 2.064g-4.9✓、gripper 空机 20.22N✓、MHE 读数 2.50/1.89=真值×1.2134✓。
# 真机标定时该增益由推力台 RPM→推力曲线直接给出。
THRUST_CAL_GAIN = 1.0

# 电机转速反算**体力矩**(B.3 Phase 0,2026-07-13):与 T_phys=k_f·Σω² 同源思路,
# 但用各电机推力 F_i=k_f·ω_i² 乘力臂叉出 roll/pitch 力矩。这是强闭环 Δr 在线估计的
# 观测量——模型无关、与 MHE/NMPC 状态完全解耦,规避 c_xy 进 model.p 后"估计↔模型"
# 自举耦合(mhe 0.15kg 吃过的那类正反馈)。x500_base/model.sdf 几何:臂 0.174m,
# velocity[i] 对应 motorNumber i(rotor_i)。FLU 体系(x前 y左 z上,与 odom_cb 同):
#   τ_roll(绕x)= Σ y_i·F_i ,  τ_pitch(绕y)= -Σ x_i·F_i
# 稳态偏心悬停时 τ_roll≈ m_p·g·ry、τ_pitch≈ -m_p·g·rx(07-03 实测 roll −0.35Nm 与
# 质心模型精确吻合)。整体符号在 Phase 0 用已知 attach_offset 标定(见验证脚本)。
ROTOR_X = np.array([0.174, -0.174,  0.174, -0.174])  # motorNumber 0..3 的机体 x
ROTOR_Y = np.array([-0.174, 0.174,  0.174, -0.174])  # 同上 y
TORQUE_SIGN = 1.0  # Phase 0 标定后固定的整体符号(默认+1,验证对表后确认/翻转)

# --- yaw 反扭矩(2026-08-25,MHE↔NMPC 完全解耦的最后一块拼图)---
# 背景:u_known 四个分量里,推力和 roll/pitch 已经能从电机转速反算
# (与 NMPC 无关),唯独 **yaw 力矩一直沿用 NMPC 的意图值** —— 因为这里原本
# 只填 0 占位(当时只需要 roll/pitch 算 c_xy)。补上它,MHE 就完全只吼
# "rate control + allocation 之后真实发生了什么",不再知道控制器在想什么。
#
# 为什么这不是可有可无的潤色(2026-08-25 拿 20 批残差诊断 CSV 对过表):
#   通道   相关    斜率(实际执行/NMPC 意图)
#   roll  +0.933   1.083
#   pitch +0.698   1.101
#   yaw   +0.911   **0.320**   <-- 差 3 倍
# roll/pitch 的意图值跟实际基本一致,唯独 yaw 严重脱节:四旋翼的 yaw 权限
# 本来就比 roll/pitch 小一个量级(只能靠反扭矩),NMPC 要 0.2Nm、PX4 实际只
# 给得出 0.065Nm。也就是说 MHE 一直把一个**比真实大 3 倍的假力矩**当"已知
# 输入"喂进转动方程。这跟 geom_release_mode='self' 那次坠机是同一个病的
# 两个症状——那次是"NMPC 幽灵配平力矩被 MHE 当已知真值收下→转动残差
# 对任意 m 都自洽"。
#
# 物理:gz MulticopterMotorModel 给机体的反扭矩 tau_z_i = -dir_i*k_m*F_i
# (螺旋桨 ccw 旋转 => 机体收到 cw 反作用)。x500 model.sdf:
# momentConstant=0.016;motorNumber 0,1=ccw(dir=+1), 2,3=cw(dir=-1)。
# 符号已实数据标定:-dir 那个候选 20/20 批与 NMPC 意图值正相关(中位
# +0.911),+dir 候选全负相关 => 排除。标定脚本见仓库 test 目录。
# ⚠️ 近似:这是**稳态反扭矩**,不含转子角加速度项 ΣJ_r·dω_i/dt。yaw 快速
#    机动时转子加减速的贡献会漏掉;悬停/温和机动下可忽。
# (真机迁移:k_m 由推力台测反扭矩→RPM 曲线给出,同 MOTOR_CONSTANT。)
MOMENT_CONSTANT = 0.016
ROTOR_DIR = np.array([+1.0, +1.0, -1.0, -1.0])   # ccw=+1, cw=-1 (motorNumber 0..3)
YAW_DIR = -ROTOR_DIR                              # 机体收到的反扭矩方向(已标定)
YAW_TORQUE_SIGN = 1.0  # 整体符号备用旋钮(同 TORQUE_SIGN;标定已确认 +1)


class MHENode(Node):
    def __init__(self):
        super().__init__('mhe_node')
        self.get_logger().info(
            'MHE node starting, building/loading solver... '
            f'(geom_coupled={mhe_p.geom_coupled}, N={mhe_p.N}, '
            f'm_min={mhe_p.m_min:.3f}, m_B={mhe_p.m_B:.4f})')
        self.solver = ensure_mhe_ocp_solver()

        # 默认部署链路只允许 motor speed + odometry 进入估计器。旧的 attach/drop
        # 订阅保留为显式消融开关,但不再是主线的一部分。
        self.declare_parameter('external_event_inputs', False)
        self.external_event_inputs = bool(
            self.get_parameter('external_event_inputs').value)

        self.x_meas = None
        self.u_known = None
        self.thrust_phys = None  # 电机转速反算的真实总推力(见 MOTOR_CONSTANT 注释)
        self.tau_phys = None     # 电机转速反算的真实体力矩 [roll,pitch,yaw](B.3 Phase0)
        # --- 抗混叠累加器(2026-08-25)---
        # motor_speed 实测 **250Hz**、MHE 才 10Hz —— 25:1 降采样。取“区间最后
        # 一帧瞬时值”再当成 0.1s 内的常量喂给模型 = 典型混叠。这正是 2026-08-24
        # 那条待验假说说的“不准但光滑(NMPC 20Hz 分段常值) 反而比 准但混叠
        # (tau_phys 瞬时值) 好”的机制。零阶保持的正确离散化是**区间平均**,
        # 不是端点采样 —— 所以这里按电机推力 F_i 累加,timer_cb 每拍 flush。
        # 在 F_i 上平均而不是在 ω 上平均:推力≈k_fω² 是非线性的,先平方再平均
        # 才是真实冲量;先平均再平方会系统性低估(Jensen)。
        self._motor_f_acc = np.zeros(4)
        self._motor_n = 0
        # Payload-frame health must describe estimator data age, not publisher
        # heartbeat age.  A failed solve still publishes held last-good values,
        # while stopped odometry/motor streams otherwise leave cached inputs in
        # place indefinitely.
        self.declare_parameter('payload_input_fresh_sec', 0.30)
        self.declare_parameter('payload_solution_fresh_sec', 0.30)
        self.payload_input_fresh_sec = max(
            float(self.get_parameter('payload_input_fresh_sec').value), mhe_p.dt)
        self.payload_solution_fresh_sec = max(
            float(self.get_parameter('payload_solution_fresh_sec').value), mhe_p.dt)
        self._last_odom_rx_sec = None
        self._last_motor_rx_sec = None
        self._last_u_rx_sec = None
        self._last_solve_success_sec = None
        self._last_solve_ok = False
        # re-anchor 把 s_est 清成 0 之后、下一次成功解之前,那个 0 不是证据。
        self._s_reanchored = False
        self._payload_inputs_were_fresh = False
        # attach 实测几何 [rx,ry,rz](box-drone,机体系,rz<0),来自
        # /gripper/attach_offset;喂 _payload_geometry 算 dJ/c_xy 给 MHE 自己的
        # om_dot 用(2026-07-07 坏几何复测发现的修复,见 mhe_params.py n_geom
        # 注释)。wrench(mass_changer)场景永远是 None,dJ/c_xy 保持 0,不影响。
        self.attach_offset = None
        # m_p 的棘轮估计(只增不减),供 _payload_geometry 算 dJ/c_xy 用。第一版
        # 直接拿瞬时 self.m_est-m_nominal 算 m_p_hat 有个自反馈缺陷:LIFT 暂态
        # 期间 solve 失败时 m_est 可能被暂时拖到低于 m_nominal,max(...,0.0) 会
        # 让修正瞬间归零——恰好在最需要修正姿态失配的时刻自己失效,形成新的
        # 自洽错误状态(2026-07-07 坏几何复测第二轮实测:pos_err 卡在 0.8m+,
        # 载荷是否在机上(物理状态,2026-07-15 加)。**不靠 attach_offset is None
        # 判断**:最后一次 attach 几何要留给日志/下一轮规划,物理状态必须独立表达。
        # 背景:棘轮/地板本为 attach-only 场景设计(防 attach 暂态估计坍缩),
        # B.3/B.5 加 drop 后没同步释放——drop 后棘轮仍卡在带载值、attach_offset
        # 仍在,_payload_geometry 继续按"幽灵载荷"给 om_dot 算 dJ/c_xy,姿态/力矩
        # 残差被错误归因 → 质量估计系统性偏低(实测 drop 后 m_est -4.1%、z 下沉
        # 5.8cm;同一轮带载段却只 -0.4%,这个不对称就是它的指纹)。这是 **drop
        # 状态同步缺失的模型状态机 bug**,不是 figure8 机动下的固有估计偏置。
        # ⚠️ drop 时只释放几何,**绝不硬重置质量状态**——让 MHE 从当前 m_est
        # 连续跑、自行收敛回空机质量,才能证明"模型正确后估计器自己能回归"。
        self._payload_attached = False

        # --- 载荷几何的质量标度 = **包线上界**(2026-08-26 去先验改造)---
        # 本项目的前提是**不能知道载荷质量**,否则在线质量估计失去意义。原先
        # 这里有三个通往 m_p_hat 的载荷质量入口,全部已删:
        #   grip_true_payload_mass —— 真值直灌(诊断专用,真机拿不到);
        #   grip_geom_mp_prior     —— "包裹标称质量"弱先验(**任务信息**);
        #   grip_geom_mp_floor     —— 永久下界地板 0.15kg(也是任务信息:它断言
        #                             "这次的载荷至少有 0.15kg")。
        # 三者都要求部署时有人告诉系统这一趟吊的是多重的东西。
        #
        # 留下的这一个是**机架规格**("这架飞机最多吊得动多少"),写在整定表里、
        # 与飞哪一趟无关 —— 拿它做几何标度不算"知道载荷质量"。
        #
        # ⚠️ 为什么不能直接删成"m_est 反推+棘轮":那正是 07-09 坐实的自举正反馈
        #    根因 —— m_est 要收敛需 dJ/c_xy 够大,dJ/c_xy 够大需 m_est 先上去,
        #    轻载信号弱冲不破(0.15kg 结构性失败)。常数包线**完全不读 m_est**,
        #    一次切断 m_est→dJ 和 m_est→c_xy 两条自举链(共用 m_p_hat 瓶颈),
        #    且不像 floor 那样只护低估方向,向上超调被棘轮永久锁死的对称风险也
        #    一并消除(棘轮在这条路径上根本不参与)。
        # ⚠️ 代价:几何按包线过估。07-19 实测先验故意错 -33%(0.2 对真实 0.3)、
        #    floor 关闭、n=8 → 下垂塌陷 0/8,m_est 仍收敛到 2.357(真值 2.364,
        #    误差 0.3%)。即**几何只需量级大致对以避免模型结构性失配,精确质量
        #    由 MHE 独立负责** —— 这条实测结论正是本次改造敢用常数包线的依据。
        self.declare_parameter('grip_payload_envelope', 0.3)
        self.grip_payload_envelope = float(
            self.get_parameter('grip_payload_envelope').value)

        # --- 评估专用真值(2026-08-24)---
        # 载荷真实质量,**只**用于往日志/internal 流里写 m_true / c_true / J_true
        # 这几列做 estimate-vs-truth 对比。关键性质:**它在代码里没有任何通往
        # 模型的路径** —— 不进 _payload_geometry、不进 geom、不进 m_est、不进
        # 任何 solver.set,纯评估。2026-08-26 去先验改造删掉了那个会把真值灌进
        # 模型的 grip_true_payload_mass 诊断开关后,本参数是节点里**仅存**的真值
        # 入口,且只写日志。0.0 = 没有真值,真值列写 nan。
        self.declare_parameter('eval_true_payload_mass', 0.0)
        self.eval_true_payload_mass = float(
            self.get_parameter('eval_true_payload_mass').value)

        # --- 几何释放方式(2026-08-24)---
        # 'event'(默认,历史行为):收到 drop 事件就把几何槽清零 → **模型被告知
        #   "载荷没了"**。这是一条外部信息,严格说不属于估计器该有的输入。
        # 'self':模型**永不被告知 drop**。几何槽里的 r_p 保持最后一次 attach 测到
        #   的杆臂不变,期望载荷贡献由 c(m)=(m_P/m_T)·r_xy 和 ΔJ(m)=μ(m)(|r|²I−rrᵀ)
        #   里的 m_P=(m−m_B)⁺ 随质量估计自行熄灭。
        # ⚠️⚠️ **2026-08-24 实测:这个期望在闭环下不成立,不加下面那条前提会坠机。**
        #   我原先在这里写"这是负反馈自校正,与 legacy 的正反馈自举不同"——**错了**,
        #   已撤回。实测(gviz_*_20260824_154827):drop 被残差正确自检出来,但随后
        #   m_est 冲到 5.0kg 撞 m_max、MHE 74 次求解失败/6 次重锚、NMPC 278 次失败,
        #   z 掉到 −0.48m 坠机。
        #   机理:MHE 的已知力矩 u_known[1:4] 长期沿用 **NMPC 的意图值**(见 u_opt_cb),
        #   而 'self' 模式下 NMPC 自己也还持有幽灵杆臂 → 它指令的幽灵配平力矩
        #   c(m̂)·T 被 MHE 当"已知真实力矩"收下 → MHE 模型用同一个 c(m) 复现它 →
        #   **转动残差对任意 m 都自洽**,质量在占 94% 信息量的通道上失去可观测性,
        #   自由漂高 → NMPC 推得更猛 → 重锚种子 T_phys/g 又"确认"更重 → 发散。
        #   这与 2026-07 在推力通道上识破的自洽盲区(见 MOTOR_CONSTANT 注释)是同一个
        #   病,只是长在力矩通道上,且只有在转动通路变成主力之后才暴露。
        #   **前提:'self' 必须配合 tau_source='phys'/'phys_full'**(把 u_known 的力矩也换成电机
        #   转速反算值),否则环断不掉。下面已强制该组合。
        # ⚠️ 只有 coupled 档才成立:legacy 档的几何幅值来自外部先验/棘轮,不随
        #   质量熄灭,'self' 会让 drop 后永远挂着幽灵载荷 → 直接拒绝该组合。
        # 载荷质量信息的**生效状态**必须在日志里讲清楚,否则后来人看到启动
        # 参数里有个 0.5 会误以为"方法需要预先知道载荷多重"。
        if mhe_p.geom_coupled:
            self.get_logger().info(
                '[prior] geom_coupled=True → 载荷质量信息**完全不生效**'
                f'(grip_payload_envelope={self.grip_payload_envelope} 是死代码:'
                f'coupled 档下 _payload_geometry 根本不被调用);'
                f'模型只吃可测杆臂 r_p + 被估质量,m 的先验仅有 Q0 质量维'
                f'={float(mhe_p.Q0[mhe_p.nx, mhe_p.nx]):.3g} '
                f'(σ={1.0/float(mhe_p.Q0[mhe_p.nx, mhe_p.nx])**0.5:.2f}kg,实质无先验)')
        else:
            self.get_logger().info(
                '[prior] geom_coupled=False → 几何幅值由**载荷包线上界**标度'
                f'(envelope={self.grip_payload_envelope}kg,机架规格非任务信息);'
                '任务信息型先验(true/prior/floor)已于 2026-08-26 全部删除')

        # --- MHE 已知力矩的来源(2026-08-24)---
        # 'command'(默认,历史行为):u_known[1:4] 用 NMPC 的意图力矩。legacy 档下
        #   无害——那时转动通路对质量的 Jacobian 恒零,力矩只影响姿态过程残差。
        # 'phys':roll/pitch 换成电机转速反算的 tau_phys(motor_speed_cb 早就在算,
        #   B.3 用它做 c_xy_est),与 MHE/NMPC 状态完全解耦。**耦合档必须用它**:
        #   转动通路一旦承担 94% 的质量信息,拿控制器的意图值当"已知真实力矩"就会
        #   让残差对任意 m 自洽(实测坠机,见 geom_release_mode 注释)。这与 2026-07
        #   把 u[0] 换成 thrust_phys 是同一个修法、同一个理由。
        # ⚠️ tau_phys 的 yaw 分量目前恒为 0 占位(c_xy 只需 roll/pitch),所以 yaw
        #   仍沿用 NMPC 意图值——yaw 不参与 c,对质量辨识影响小,但要记着这是近似。
        # ⚠️ 默认恒为 'command'。我 2026-08-24 一度把耦合档默认设成 'phys',
        # **实测证明有害并已回退**:coupled+self+phys 在 attach/LIFT 段就发散坠机
        # (gviz_*_20260824_155451,比 coupled+self+command 的 drop 段发散更早),
        # 而 coupled+event+command 是**已验证可用**的配置。默认设成 'phys' 会连带
        # 破坏那个可用配置,所以回退。'phys' 保留为显式 opt-in 旋钮待离线查清。
        # 待验假说:tau_phys 是 250Hz 的快变真实力矩,MHE 按 10Hz 采样并在 0.1s
        # 内当常数用 → 混叠;而 NMPC 意图力矩是 20Hz 分段常值、与射击区间匹配,
        # 于是"不准但光滑"反而比"准但混叠"好。判别法见离线 standalone 测试。
        # mhe_tau_source: u_known 的力矩分量从哪来。
        #   'command'  (默认,历史行为): roll/pitch/yaw 全用 NMPC 意图值
        #   'phys'     : roll/pitch 换成电机转速反算,yaw 仍是 NMPC 意图值
        #   'phys_full': **三轴全部**电机转速反算 —— MHE 与 NMPC 完全解耦,
        #                只吃 rate control + allocation 之后真实发生的量。
        # 'phys' 留着不动是为了让既有实验(geom_release_mode='self' 等)可复现;
        # 新工作应该用 'phys_full',理由见 MOMENT_CONSTANT 处那张斜率对照表
        # (yaw 意图值比实际大 3 倍)。
        # 抗混叠开关:**默认关**。理论上区间平均才是 250Hz->10Hz 的正确离散化
        # (见 _motor_f_acc 注释),但打开它会改掉**每一个**档位的 u[0](推力那
        # 一维从 2026-07 起就取电机反算值,'command' 档也不例外)—— 那是全部
        # 既有实验的输入,属于重大静默行为改变。本仓惯例:没有 n>=8 的实测
        # 不改默认值(2026-08-25 dJ 先验去除刚栽在这条上)。先当显式 opt-in,
        # 用来验 2026-08-24 那条"准但混叠"假说;验过再谈默认。
        # 【2026-09-10 翻回 False】上面这段注释("默认关"、"没有 n>=8 的实测不改
        # 默认")说的才是本仓的规矩,但代码在 2026-08-25 前后被改成了 True 而注释
        # 没同步 —— 那次改动没有实测背书。09-05 的 n=8 配对把这一档判掉了
        # (见下面 mhe_tau_source 处),现在把这一对默认一起翻回已验证的组合。
        self.declare_parameter('motor_window_avg', False)
        self.motor_window_avg = bool(
            self.get_parameter('motor_window_avg').value)
        # 【2026-09-10 翻回 'command'】上面"⚠️ 默认恒为 'command'"那段是原始规矩,
        # 代码却在 2026-08-25 前后被改成 'phys_full'(注释没同步,也没有实测背书)。
        # 2026-09-05 的 n=8 配对(run_tau_source_ab.sh,4m/s 主线档)判定留在 command:
        #   · phys_full+avg 臂 **1/8 轮在 drop 之前发散**(peak 40.6m、attach 后 4.3s
        #     起 solve failed、推力塌到下界 0.50N),command 臂 0/8、同底座历史再 0/12;
        #     即 2026-08-24 那个失效模式的复现 —— 配 motor_window_avg=1 也没消掉,
        #     所以上面那条"准但混叠"假说**不足以**为 phys_full 翻案。
        #   · 质量精度**未检出差异**(带载稳态偏差配对差 +0.21pp,95% CI [−0.05,+0.47],
        #     5/7 同向 p=.45;跑前功效可检出 0.23pp)—— 不能反过来说两者等价。
        #   · phys_full 唯一显著的好处是 MHE 求解 9.09→7.41ms(−17.5%,7/7 同向
        #     p=.0067),但 10Hz MHE 预算 100ms,占比 9.1%→7.4%,没有实际意义。
        #   判负的力量来自预注册的一票否决 + 08-24 先验 + viz 历史旁证,**不是**本批
        #   的统计功效(单看计数 1/8 vs 0/8 Fisher p=1.0)。记忆 tau-source-viz-headless-ab。
        # ⚠️ 'phys'/'phys_full' 两档保留为显式 opt-in(既有实验靠它复现),只是不再是默认。
        # ⚠️ 连带影响:src/scripts/masschanger/ 的三个脚本不传本参数,会跟着默认走 ——
        #    它们是 07 月阶段 A 的设施,当时默认就是 'command',所以这次是**恢复**其历史口径。
        self.declare_parameter('mhe_tau_source', 'command')
        self.tau_source = str(
            self.get_parameter('mhe_tau_source').value).lower()
        if self.tau_source not in ('command', 'phys', 'phys_full'):
            self.tau_source = 'command'

        # --- pre-offboard 冷启动估计(2026-08-25,"去先验"下半程第 #5 项)---
        # 目的:在 NMPC 接管**之前**(PX4 posctl 悬停段)就把质量估出来,收敛后
        # 再喂给 NMPC —— 这样 NMPC 从接管第一帧起用的就是**估出来的**质量,
        # 而不是硬编码的空机标称值 p.m。配合 MHE_NO_MASS_PRIOR=1 才是完整的
        # "零先验":那个开关卸掉 Q0 软锚 + m_min 硬下界(见 mhe_params),这个
        # 开关负责让估计有机会在接管前跑完。
        #
        # 为什么 pre-offboard 段跑得起来:timer_cb 原本卡在 u_known is None,
        # 而 u_known 在 tau_source='phys' 下**四个分量里三个来自电机转速**
        # (见 u_opt_cb),跟 NMPC 完全无关;motor_speed_cb 从 gz 桥一起来就在
        # 跑。所以这里直接用电机反算值合成 u_known,yaw 力矩填 0(反扭没建模,
        # posctl 悬停下实测 roll/pitch 才 ±0.002Nm,yaw 同量级)。
        # ⚠️ 这条路上的 u_known **不含任何 NMPC 意图值**,环是断开的 ——
        #    比常规 command 模式还干净,不会重演 geom_release_mode='self' 那个
        #    "NMPC 幽灵配平力矩被 MHE 当已知真值收下"的自洽盲区。
        self.declare_parameter('pre_offboard_estimate', False)
        self.pre_offboard_estimate = bool(
            self.get_parameter('pre_offboard_estimate').value)
        # 起飞门控:地面支持力会让 T_phys 与 m·g 脱钩(全仓反复踩过的坑),
        # 必须确认真的离地了才让这条路生效。
        self.declare_parameter('pre_offboard_min_z', 0.5)
        self.pre_offboard_min_z = float(
            self.get_parameter('pre_offboard_min_z').value)
        # 收敛判据:最近 conv_frames 帧 m_est 的样本标准差 < conv_std [kg]。
        # 收敛**之前不发布** mass_pub —— 未收敛的冷启动值直接灌进 NMPC 的
        # 动力学模型是危险的(m 偏小 => 悬停推力算低 => 掉高度)。
        self.declare_parameter('pre_offboard_conv_std', 0.02)
        self.declare_parameter('pre_offboard_conv_frames', 20)
        self.declare_parameter('pre_offboard_timeout_sec', 30.0)
        self.pre_off_conv_std = float(
            self.get_parameter('pre_offboard_conv_std').value)
        self.pre_off_conv_frames = max(
            2, int(self.get_parameter('pre_offboard_conv_frames').value))
        self.pre_off_timeout_frames = int(round(
            float(self.get_parameter('pre_offboard_timeout_sec').value)
            / mhe_p.dt))
        self._u_from_nmpc = False      # u_known 是否已由 NMPC 的 u_opt 接手
        self._pre_off_converged = False
        self._pre_off_m_hist = []
        self._pre_off_first_frame = None
        if self.pre_offboard_estimate and self.tau_source not in ('phys', 'phys_full'):
            # 不强制改:tau_source 是别的实验的自变量,这里只把后果讲明白。
            self.get_logger().warn(
                "pre_offboard_estimate=True 但 mhe_tau_source='command':"
                "pre-offboard 段的力矩必然是电机反算值(NMPC 还没输出),接管"
                "瞬间会切回 NMPC 意图值 —— u_known 的语义在那一帧跳变。"
                "想避免请一并设 mhe_tau_source:=phys。")
        if self.pre_offboard_estimate:
            self.get_logger().info(
                f'[cold-start] pre-offboard 估计已开启 (min_z='
                f'{self.pre_offboard_min_z}m, 收敛判据: {self.pre_off_conv_frames}'
                f'帧 std<{self.pre_off_conv_std}kg, 超时'
                f'{self.pre_off_timeout_frames * mhe_p.dt:.0f}s)'
                f' | no_mass_prior={mhe_p.no_mass_prior}')

        # 遗忘因子与事件触发互斥(见 _solve_window 注释)
        self._forget_active = mhe_p.forgetting_lambda < 1.0
        self._forget_set = False
        self.declare_parameter('geom_release_mode', 'event')
        self.geom_release_mode = str(
            self.get_parameter('geom_release_mode').value).lower()
        if self.geom_release_mode not in ('event', 'self'):
            self.get_logger().warn(
                f"unknown geom_release_mode '{self.geom_release_mode}', "
                "falling back to 'event'")
            self.geom_release_mode = 'event'
        # ⚠️ 这里原本强制 'self' 必须配 'phys'(我基于"幽灵配平力矩致转动残差自洽"
        # 的推断)。**该推断未被实测支持**:改成 'phys' 后反而在 attach 段就发散,
        # 比原来更早。强制关系已撤销,'self' 目前是**已知不可用**的实验档。
        if self.geom_release_mode == 'self':
            self.get_logger().warn(
                "geom_release_mode='self':估计器不被告知 drop。⚠️ 2026-08-24 实测,"
                "**NMPC 侧也设成 'self' 会必然坠机**——drop 后控制器继续按幽灵偏心"
                "载荷配平,指令 0.424N·m(tau_max 的 85%)作用在空机 J_xx=0.0142 上 = "
                "29.8 rad/s²,而 MHE 10Hz/2s 窗口要 1~2s 才纠得过来,飞机先翻。"
                "**正确组合 = MHE 'self' + NMPC 'event'**(控制器是发释放指令的一方,"
                "它知道不构成认知作弊)。")
        if self.geom_release_mode == 'self' and not mhe_p.geom_coupled:
            self.get_logger().error(
                "geom_release_mode='self' 需要 MHE_GEOM_COUPLED=1 —— legacy 档的"
                "几何幅值来自外部先验、不随质量熄灭,drop 后会永远挂幽灵载荷。"
                "已强制退回 'event'。")
            self.geom_release_mode = 'event'
        # 最后一次 attach 测到的杆臂,**drop 事件不清除它**(清除就等于告知模型)。
        self._r_p_last = None

        self.y_buf = []  # 测量缓冲区,最多 N+1 帧
        self.u_buf = []  # 已知输入缓冲区,最多 N 帧

        self.x0_bar = None
        self.x_guess = None
        self.m_est = mhe_p.m_nominal
        # 一阶质量矩估计 s=m_P·r_xy [kg·m](estimate_moment 档才有;否则恒为空数组)
        self.s_est = np.zeros(mhe_p.ns)
        # 对外发布的是连续、物理可消费的载荷状态。m_payload 与 s 经同一个一阶
        # 平滑器后再派生 c/J,因此 drop 时不会出现任何事件清零阶跃。
        self.declare_parameter('payload_output_tau_sec', 0.25)
        self.payload_output_tau = max(
            float(self.get_parameter('payload_output_tau_sec').value), 1e-3)
        self.declare_parameter('no_payload_mass_full', 0.015)
        self.declare_parameter('no_payload_mass_zero', 0.060)
        self.declare_parameter('no_payload_moment_full', 0.0015)
        self.declare_parameter('no_payload_moment_zero', 0.0060)
        self.declare_parameter('no_payload_inertia_full', 0.0020)
        self.declare_parameter('no_payload_inertia_zero', 0.0100)
        self._payload_conf_args = {
            'mass_full': float(self.get_parameter('no_payload_mass_full').value),
            'mass_zero': float(self.get_parameter('no_payload_mass_zero').value),
            'moment_full': float(self.get_parameter('no_payload_moment_full').value),
            'moment_zero': float(self.get_parameter('no_payload_moment_zero').value),
            'inertia_full': float(self.get_parameter('no_payload_inertia_full').value),
            'inertia_zero': float(self.get_parameter('no_payload_inertia_zero').value),
        }
        self._m_payload_out = 0.0
        self._s_out = np.zeros(2)
        self._no_payload_confidence = 1.0
        # 连续 solve 失败计数。失败时 x0_bar/x_guess 不更新而 y_buf/u_buf 照常
        # 滑动,先验会越滞后越矛盾 → status=2 自锁死循环(2026-07-06 吊挂
        # LIFT 暂态实测:一次硬失败后 2000+ 窗口连败、m_est 冻结)。连败达
        # 阈值就把先验重锚到当前窗口(质量保留最后估计),给 solver 台阶下。
        self._fail_streak = 0
        self.counter = 0
        self.frames = 0  # 已入缓冲的总帧数(事件调度器的全局帧序号基准)
        # 论文 timing 表的原始样本:整轮飞行的 solve 耗时都留着,收尾时一次性
        # 出分位数(轮次才 ~100s、10Hz,总量千级,不必限长)。
        self._solve_ms = []

        # 事件触发权重调度(M0,规则版):抓/放是已知事件,收到事件后把窗口内
        # 事件前 stage 降权,让质量估计在一两帧内跳到新值而不是等 2s 窗口
        # 滑完。机制/理由见 mhe_event_weights.py 文件头。
        #
        # 两段式触发(2026-07-03 SITL 实测后加):ROS 事件只"武装"检测器,
        # T_phys 相对武装时刻基线突变超过 confirm_thresh 才真正开始降权——
        # 实测 drop 的 gz CLI 冷启动 discovery 让物理生效滞后事件通知 ~1.1s,
        # 若按事件时刻降权,中间十来帧"名义权重+旧物理"会以全权重把质量拖回
        # 旧值,红利被吃掉大半。真机上夹爪指令→机械释放→载荷转移同样有延迟,
        # 这个结构本来就更对;它也是 M1"从残差检测突变"的雏形(事件把检测器
        # 的误报率降到零,检测器把触发时刻对齐到物理)。confirm_timeout 秒内
        # 没确认则按事件时刻兜底触发。
        # 权重时间表 theta(4维,见 mhe_weight_learning.ParametricWeightSchedule
        # 注释):默认 M0_THETA=[-4,0,0,0](二值降权规则);M1 学出的 θ* 通过参数
        # 传入。M0_THETA 下 ParametricWeightSchedule 与旧 EventWeightScheduler
        # 严格等价(过渡期事件后 stage 缩放全为 1)。
        self.declare_parameter('event_trigger_enable', True)
        self.declare_parameter('schedule_theta', [float(v) for v in M0_THETA])
        self.declare_parameter('event_confirm_thresh_n', 1.5)
        self.declare_parameter('event_confirm_timeout_sec', 3.0)
        # 无信号消融(2026-07-13,长期计划 A.2 / C.3):事件触发的调度机制不变,
        # 但**去掉外部事件信号这一路**——检测器不再由 /acados_nmpc/mass_event 或
        # /gripper/attach_offset 武装,改成纯从 T_phys 相对慢基线的残差自触发
        # (低通基线 + 持续超阈值确认,同一个 confirm_thresh),量化"事件信号"
        # 相对"仅靠残差检测"到底值多少。这是论文"学习/规则版对规则版增量"的
        # 核心对照。'external'=默认(现有有信号行为,逐字节不变),'residual'=无信号。
        # 持续帧数 resid_persist 是抗阵风脉冲的第二段(阵风残差是脉冲/宽带,质量
        # 突变是台阶,持续超阈值才确认——正是 C.3 风扰消融要用的判据,提前上)。
        self.declare_parameter('event_signal_mode', 'external')
        self.declare_parameter('resid_baseline_tau_sec', 3.0)
        self.declare_parameter('resid_persist_frames', 2)
        # 预热门控(2026-07-13 冒烟实测加):起飞/NMPC 接管姿态的暂态期 T_phys
        # 本身大幅摆动(frame 5 摆 1.66N),此时没有可信静基线,残差检测器会把
        # 暂态摆动误当成质量突变(实测无信号版在 frame 5 假触发)。要求检测器先
        # 自主确认基线已静(连续 resid_warmup_frames 帧 T_phys 贴基线,偏差
        # <resid_settle_tol_n)才开放检测——完全不用外部时刻信息,与"无信号"
        # 语义一致。暂态摆动会不断打断预热计数,预热自然推迟到真正悬停稳。
        self.declare_parameter('resid_warmup_frames', 20)
        self.declare_parameter('resid_settle_tol_n', 0.8)
        # 无真值确认阈值(B.4 学习+强闭环合流,2026-07-15):CEM 学出的 θ 第 5 维
        # 是无量纲 α,实际确认阈值 = α·g·mass。训练/评估在外部用花名册**真值** mass
        # 算好传 event_confirm_thresh_n;部署(无真值)必须换成一个不含任务信息的
        # 质量标度。confirm_thresh_alpha>=0 时:confirm_thresh = α·g·
        # grip_payload_envelope(**载荷包线上界,机架规格**);<0=禁用,退回固定
        # event_confirm_thresh_n(现有行为)。
        # 【2026-08-26 去先验改造】原先这里用的是 confirm_payload_prior=0.3,
        # 即"这次要抓的盒子大概多重"的操作先验 —— 任务信息,已删。
        # ⚠️ 包线上界 > 实际载荷时阈值偏高 → 小载荷 T_phys 达不到阈值 → 超时
        #    兜底退回事件帧(graceful,不是硬失败),代价是确认延迟。
        self.declare_parameter('confirm_thresh_alpha', -1.0)
        self.event_enabled = bool(
            self.get_parameter('event_trigger_enable').value)
        # 遗忘因子与事件触发互斥(见 _solve_window 注释)。必须放在 event_enabled
        # 赋值**之后**——之前放前面引用了还不存在的属性。
        if self._forget_active and self.event_enabled:
            self.get_logger().error(
                f'MHE_LAMBDA={mhe_p.forgetting_lambda} 与 event_trigger_enable=true '
                '互斥(两套 stage 缩放会相乘,归因说不清)。遗忘因子是**替代**事件'
                '触发的方案,已自动停用事件触发。')
            self.event_enabled = False
        c_alpha = float(self.get_parameter('confirm_thresh_alpha').value)
        if c_alpha >= 0.0:
            self.confirm_thresh = (
                c_alpha * mhe_p.g * self.grip_payload_envelope)
        else:
            self.confirm_thresh = float(
                self.get_parameter('event_confirm_thresh_n').value)
        self.confirm_timeout_frames = int(round(
            float(self.get_parameter('event_confirm_timeout_sec').value)
            / mhe_p.dt))
        self.signal_mode = str(
            self.get_parameter('event_signal_mode').value).lower()
        if self.signal_mode not in ('external', 'residual'):
            self.get_logger().warn(
                f"unknown event_signal_mode '{self.signal_mode}', "
                "falling back to 'external'")
            self.signal_mode = 'external'
        # 慢基线 EMA 系数 alpha=dt/tau:tau 取几秒,足够慢不追台阶(冻结在过渡期
        # 又进一步保证武装基线是阶跃前的真值),又能跟掉长期缓漂。
        tau = max(float(self.get_parameter('resid_baseline_tau_sec').value),
                  mhe_p.dt)
        self.resid_ema_alpha = mhe_p.dt / tau
        self.resid_persist = max(
            1, int(self.get_parameter('resid_persist_frames').value))
        self.resid_warmup = max(
            1, int(self.get_parameter('resid_warmup_frames').value))
        self.resid_settle_tol = float(
            self.get_parameter('resid_settle_tol_n').value)
        # 自检测到"推力下降型"事件时,是否自行释放载荷几何(见 _residual_detect)。
        # 默认 True:它只在 event_signal_mode='residual' 下才有机会执行,而那一档
        # 的全部意义就是"不依赖外部信号",几何释放没有理由留个外部依赖的尾巴。
        self.declare_parameter('resid_release_geom', True)
        self.resid_release_geom = bool(
            self.get_parameter('resid_release_geom').value)

        # ---- 并行阶跃判据(2026-08-26,默认关)----
        # 上面那条判据看的是"T_phys 偏离**慢基线**",需要一条可信基线,因此带一道
        # 预热门控。这条并行判据换一个完全不同的量:**相邻两个短窗的均值差**,
        # 不需要基线、因此没有预热这个失效面。两者取或,原判据一行不改 ——
        # 悬停工况下已验证的行为(阶段 A.2)因此原样保留。
        #
        # 为什么窗口要**短**(离线回放实测,test_step_detector_replay.py):
        #   半窗 0.3s → 机动段最大 |Δ|=0.766N,drop 处 2.99N,区分度 3.9×
        #   半窗 0.5s → 2.3×
        #   半窗 0.8s → 1.0×(完全不可分)
        # 窗口越长,figure-8 的缓变在窗内积累越多,而阶跃幅度被稀释 —— 与"越长
        # 越抗噪"的直觉相反。
        #
        # persist 不只是抗噪:drop 的推力阶跃是**永久**的(质量真的变了),而轨迹
        # 切换引起的推力变化是**暂态**的,所以持续帧数本身带区分力。
        # persist=2/thresh=1.0 实测:drop 延迟 0.30s、幅度 -2.37N(余量 2.4×)、
        # 稳态段零误触发。
        #
        # ⚠️ 固有限制:"质量变了"与"要飞的轨迹变了"在推力通道上**不可分** ——
        #    figure-8 切入那一下(实测 -1.16N / +1.25N)也会被检出。C.3 风扰消融
        #    得过同类结论(持续垂直力与质量突变不可分)。所以**释放几何**用一条
        #    单独的、更高的门槛 resid_step_release_thresh(默认 2.0N):
        #    误触发降权只是慢一点,误释放几何却会让模型丢掉还挂着的载荷。
        self.declare_parameter('resid_step_enable', False)
        self.declare_parameter('resid_step_half', 3)          # 半窗帧数(0.3s@10Hz)
        self.declare_parameter('resid_step_thresh', 1.0)      # 触发降权 [N]
        self.declare_parameter('resid_step_persist', 2)
        self.declare_parameter('resid_step_release_thresh', 2.0)  # 释放几何 [N]
        self.resid_step_enable = bool(
            self.get_parameter('resid_step_enable').value)
        self.resid_step_half = max(1, int(self.get_parameter('resid_step_half').value))
        self.resid_step_thresh = float(self.get_parameter('resid_step_thresh').value)
        self.resid_step_persist = max(
            1, int(self.get_parameter('resid_step_persist').value))
        self.resid_step_release_thresh = float(
            self.get_parameter('resid_step_release_thresh').value)
        self._tphys_hist = []      # 最近 2*half 帧 T_phys(阶跃判据用)
        self._step_pending = 0
        if self.resid_step_enable:
            self.get_logger().info(
                f'[step-detect] 并行阶跃判据已开启: 半窗 '
                f'{self.resid_step_half * mhe_p.dt:.1f}s, 阈值 '
                f'{self.resid_step_thresh}N (释放几何门槛 '
                f'{self.resid_step_release_thresh}N), persist='
                f'{self.resid_step_persist}')
        self._resid_baseline = None   # T_phys 慢基线(残差自检测用)
        self._resid_pending = 0       # 已连续超阈值的帧数(持续确认计数)
        self._resid_cross_frame = None  # 首次越阈那一帧的全局序号
        self._resid_settled = False   # 基线是否已预热到"可信静基线"(开放检测)
        self._resid_stable = 0        # 连续贴基线的帧数(预热计数)

        # ---- 强闭环 c_xy 在线估计(B.3 Phase1,2026-07-14)----
        # 从 tau_phys/thrust_phys 直接反解复合质心水平偏移 c_xy=[cx,cy]:
        # τ_roll=cy·T → cy=τ_roll/T,τ_pitch=-cx·T → cx=-τ_pitch/T(Phase0 已实测
        # 验证 <2%,且这两个量都来自电机转速反算,与 MHE 的 m_est 完全解耦——从根
        # 上避开 c_xy↔model 自举耦合)。**Phase1 只记录不闭环**:估计值发话题+对真值
        # 打日志,不喂任何模型。稳态门控:只在悬停稳(|ω|/|v_xy| 小)时更新 EMA,
        # 避开 descend/LIFT/drop 暂态的力矩污染(思路同无信号消融的预热门控)。
        # 不进 MHE 状态(窗外慢滤波),符合长期计划"两个新变量不同时上线"。
        # IMPORTANT: despite its historical name, this is currently also the
        # master gate for all online release decisions because _update_c_xy_est()
        # contains the mass-domain path and the sole release_decision() call.
        # Release-validation launchers must explicitly enable it; a bare launch
        # leaves release unclassified until the NMPC unresolved timeout.
        self.declare_parameter('c_xy_est_enable', False)   # legacy-safe default
        self.declare_parameter('c_xy_est_tau_sec', 2.0)    # EMA 时间常数
        self.declare_parameter('c_xy_steady_omega', 0.15)  # 稳态门控角速度阈 rad/s
        self.declare_parameter('c_xy_steady_vel', 0.20)    # 稳态门控水平速度阈 m/s
        self.c_xy_est_enable = bool(
            self.get_parameter('c_xy_est_enable').value)
        ctau = max(float(self.get_parameter('c_xy_est_tau_sec').value), mhe_p.dt)
        self.c_xy_ema_alpha = mhe_p.dt / ctau
        self.c_xy_steady_omega = float(
            self.get_parameter('c_xy_steady_omega').value)
        self.c_xy_steady_vel = float(
            self.get_parameter('c_xy_steady_vel').value)
        # ---- 基于质量的载荷卸载判据(2026-09-01)----
        # 动机:geom_release_mode='self' 下 MHE 从不被告知 drop,而 c_xy_est 这条
        # **发布支路**自己不会熄灭 —— 它的更新入口有稳态门控(|ω|、|v_xy|),
        # figure8 中一次都不满足,EMA 停在带载值(实测 drop 后 60s 仍报 cy=-4.8mm)。
        # 模型内部的 c(m) 在 coupled 档下随 m 正常熄灭,脏的只有这条对外话题。
        # 判据放在**质量域**而非推力残差域:实测 0.15kg 工况带载段 m_p 最低
        # 0.0785kg、drop 后最高 0.007kg,间隔 11x;同工况推力残差侧真信号 1.47N
        # vs LIFT 段噪声 1.23N 只有 1.2x(阈值 1.5N 漏检、1.177N 被 LIFT 诈出
        # 提前释放,两头堵)。而且这是**状态判据**不是事件判据,不必抓准时刻,
        # 可以加滞回慢慢确认 —— c_xy_est 在机动中本来就不更新,延迟无代价。
        # ⚠️ 只在 'self' 档启用:'event' 档的 _model_r_p() 读 attach_offset,而
        #    _release_payload 会清它 -> 等于让 m_est 关掉模型几何 = 自举耦合
        #    (2026-07 踩过);'self' 档模型走 _r_p_last,不受此影响。
        self.declare_parameter('c_xy_mass_release_mp', 0.0)      # kg,0=关
        self.declare_parameter('c_xy_mass_release_persist', 20)  # 帧,10Hz->2s
        # 武装门控(2026-09-01 第一版实测补):**必须先看见载荷,才允许宣布它消失**。
        # 第一版漏了这条,判据只问"m_p 小不小" —— 而 attach 后 m_est 要从空机值
        # 往上爬,收敛期的 m_p 本来就小于阈值,于是在 LIFT 刚开始(drop 前 74s)
        # 就把释放点着了;真 drop 到来时 _payload_attached 早已 False、判据失效,
        # c_xy_est 反而 29s 都没熄。武装水位 = arm_ratio × 释放阈值。
        self.declare_parameter('c_xy_mass_arm_ratio', 3.0)
        # 武装**也要持续**(2026-09-01 配对批次 1/8 轮实测补):只看单点冲高不够 ——
        # attach 后 m̂ 的收敛振荡会同时满足"冲高"和"回落":实测 t=9.9s 单点冲到
        # m_p=0.232 把武装点着,t=12.0s 就回落到 0.017 且持续 2s → 误释放
        # (drop−71.5s),而 t=14.0s 才真正收敛。要求连续若干帧高于水位后,
        # 单点冲高武装不了,武装推迟到收敛之后,那以后不再有低于阈值的低谷。
        self.declare_parameter('c_xy_mass_arm_persist', 20)  # 帧,10Hz->2s
        self.c_xy_mass_release_mp = float(
            self.get_parameter('c_xy_mass_release_mp').value)
        self.c_xy_mass_release_persist = max(
            1, int(self.get_parameter('c_xy_mass_release_persist').value))
        self.c_xy_mass_arm_ratio = float(
            self.get_parameter('c_xy_mass_arm_ratio').value)
        self.c_xy_mass_arm_persist = max(
            1, int(self.get_parameter('c_xy_mass_arm_persist').value))
        self._c_xy_mass_low = 0
        self._c_xy_mass_high = 0
        self._c_xy_mass_armed = False

        # ---- 一阶质量矩的释放(2026-09-04)----
        # 96 架次主线批次坐实的阻塞缺陷:_release_payload 只归零 c_xy_est,
        # **不动 s**,而 _publish_payload_estimate 的 s_target 直接取 self.s_est
        # —— drop 后 m 已正确回到空机、s 却停在 8~13mm 的幽灵偏心上,
        # no_payload_confidence 的乘积判据被 s 项一票否决(conf 恒 0.000),
        # NMPC 的 drop 因此永远 complete 不了(7/48 轮,全在机动工况)。
        # 修法是把**目标**切零而不是放宽阈值:放宽只是绕过幽灵 s,模型照样
        # 吃着残余偏心飞。对外的 _s_out 仍走原来的 LPF 连续衰减到零,接口上
        # 没有阶跃。
        self._s_release_latched = False

        # ---- 载荷存在状态机(2026-09-04)----
        # EMPTY ──自主检测 ATTACH / 持续载荷证据──> LOADED
        # LOADED ──自主检测 DROP  / 持续空载证据──> EMPTY
        #
        # 为什么必须独立于 _payload_attached:后者的语义是"收到过外部 attach
        # 通知",而连续主线的前提正是**不发**这个通知 —— 三条自主释放路径
        # (step-detect / no-signal 残差 / 质量域)全都写着 and self._payload_attached,
        # 于是在主线档下一条都进不去,_release_payload 从未被调用过(2026-09-04
        # 第二轮批次第 2 架次坐实:no-signal 明明检出了 DROP,却被这个门挡下)。
        # 这里给出估计器**自己**判定的存在状态,两个状态不混用:_payload_attached
        # 继续只表示外部事件,_payload_present 才是释放路径的门控。外部通知不再是
        # 进入这些路径的必要条件 —— 也不作为充分条件,免得又把两者绑在一起。
        self.declare_parameter('payload_present_enter_mp', 0.09)   # kg
        self.declare_parameter('payload_present_enter_persist', 20)  # 帧,10Hz->2s
        # 退出是**慢兜底**,不是主力:主力是残差 DROP 快速通道。0.02kg/5s 比进入
        # 严得多,且只在低机动窗口累计(见 _mass_observable)——4m/s figure8 段 m_p
        # 会周期性探底到 −0.10kg(机动低估,见记忆 mest-maneuver-underestimate),
        # 拿绝对阈值在机动中判空载必然误判。09-04 首次 smoke 实测:2s 窗口下
        # figure8 段误退 3 次,EMPTY 窗口占比 9.7%。
        self.declare_parameter('payload_present_exit_mp', 0.02)    # kg
        self.declare_parameter('payload_present_exit_persist', 50)  # 帧,10Hz->5s
        # 质量可观测窗口的门槛(与 c_xy_steady_* 同量纲但**独立**:那两个管的是
        # "力矩采样干不干净",这两个管的是"质量估计可不可信",不该一起调)。
        self.declare_parameter('payload_exit_steady_omega', 0.15)  # rad/s
        self.declare_parameter('payload_exit_steady_vel', 0.20)    # m/s
        self.payload_present_enter_mp = float(
            self.get_parameter('payload_present_enter_mp').value)
        self.payload_present_enter_persist = max(
            1, int(self.get_parameter('payload_present_enter_persist').value))
        self.payload_present_exit_mp = float(
            self.get_parameter('payload_present_exit_mp').value)
        self.payload_present_exit_persist = max(
            1, int(self.get_parameter('payload_present_exit_persist').value))
        self.payload_exit_steady_omega = float(
            self.get_parameter('payload_exit_steady_omega').value)
        self.payload_exit_steady_vel = float(
            self.get_parameter('payload_exit_steady_vel').value)
        self._payload_present = False      # False=EMPTY, True=LOADED(瞬时判定)
        # ★ "本轮曾可靠进入过 LOADED" 的 latch。释放路径的门控用**它**,不用瞬时
        # _payload_present:机动中质量探底会让后者临时翻回 EMPTY,若拿它当门控,
        # 真 drop 恰好落在那个窗口时,残差明明检出了 DROP 也会被自己的门挡住
        # (09-04 smoke 实测该窗口占 figure8 段的 9.7%)。armed 只在**真实释放
        # 完成**时清除,所以临时 EMPTY 不会挡住随后的真 drop。
        self._load_armed = False
        # [s-decay] 日志:释放后逐帧记 s 的衰减,用来回答"对外是否连续、有没有阶跃"
        # (_s_out 原先只发布不落日志,SITL 里看不到时序)。10Hz × 100 帧 = 10s,
        # 足够覆盖 payload_output_tau=0.3s 的 LPF 全过程。
        self.declare_parameter('s_decay_log_frames', 100)
        self.s_decay_log_frames = max(
            0, int(self.get_parameter('s_decay_log_frames').value))
        self._s_decay_n = 0
        self._s_out_prev = np.zeros(2)
        self._present_hi = 0
        self._present_lo = 0
        # --- drop 侧释放判据:一阶矩**相对**塌陷 + 质量/惯量佐证(2026-09-04 改)---
        # 武装期间记 |s| 峰值,|s|/peak 跌破 ratio 且质量与惯量通道也报"空"、
        # 持续若干帧 ⇒ 释放。
        #
        # ★ 为什么 ratio 从 0.10 提到 0.20:09-04 纯默认档实测(232444),物理卸载后
        #   |s| 从峰值 0.0229 塌到 0.0032 = 塌掉 86%,但 0.10 要求塌掉 >90% —— 那等于
        #   要求估计器残噪低于一个**没有物理必要性**的水平。三条释放路径因此全断,
        #   conf 卡在 0.694,DROP 永远 complete 不了(死锁细节见 docs/mjc_lifecycle_viz.md)。
        #   0.20 相对实测的 0.14 有裕量,又不是一下子放到很松。
        #
        # ★ 为什么持续帧从 20(2s) 缩到 5(0.5s):加了质量/惯量两条佐证之后,单靠
        #   长时间等待来防误触发的必要性下降;而 drop 后每多等一帧,NMPC 的
        #   grip_drop_pending 就多挂一帧。
        #
        # ★ 为什么佐证用 mass/inertia 而**不用** moment 通道:moment 通道是绝对阈值
        #   (moment_full=0.0015),而 s 的残噪本底就在 0.003 量级 —— 用它当触发条件
        #   正是死锁的成因。这里只借 payload_estimate 的同一套标定算 mass/inertia
        #   两个分数,与 conf 同源但不共用那条卡死的通道。
        #   ⚠️ 单独的 mass 佐证会误报(机动中 m_p 周期性探底到负值,这正是
        #   _mass_observable 门控存在的理由),但它与"相对塌陷"合取后是安全的:
        #   载荷还在机上时 |s| 不会塌到自身峰值的 20% 以下。所以这条路**故意不加**
        #   机动门控 —— 加了就退回质量域判据在 figure8 中永远不累计的老问题。
        # ratio<=0 关闭。
        # ★ 0.20 -> 0.30(2026-09-05):59 轮 valid 历史日志的**留一交叉验证**
        #   59/59 折一致选中 0.30(留出轮 90% 成功),而 0.20 只有 48/59。
        #   误释放在两个阈值下都是 0 —— 因为释放要求 ratio 与**质量通道**同时
        #   成立,带载段 ratio 掉到 0.000 的 8 轮全部被质量通道否决。
        #   这不是"把阈值往上调到能过",是合取结构下 ratio 单通道判别力本来就
        #   不该被当成唯一闸门(它与带载分布重叠)。
        self.declare_parameter('s_release_ratio', RELEASE_RATIO_THR)
        self.declare_parameter('s_release_persist', 5)    # 帧,10Hz->0.5s
        self.declare_parameter('s_release_mass_score_min', 0.90)
        self.declare_parameter('s_release_inertia_score_min', 0.90)
        self.s_release_ratio = float(self.get_parameter('s_release_ratio').value)
        self.s_release_persist = max(
            1, int(self.get_parameter('s_release_persist').value))
        self.s_release_mass_score_min = float(
            self.get_parameter('s_release_mass_score_min').value)
        self.s_release_inertia_score_min = float(
            self.get_parameter('s_release_inertia_score_min').value)
        # |s| 的绝对上限:**不是判别器**,只挡"peak 异常大导致 ratio 假性偏小"。
        # 09-04/05 实测两个分布是重叠的(带载 |s| 最小 0.0039 反而低于卸载后
        # 最大 0.0049),所以绝对量给得宽松,判别力全靠 ratio + 质量通道的合取。
        self.declare_parameter('s_release_abs_max', 0.008)
        self.declare_parameter('s_release_fast_persist', 2)
        self.declare_parameter('s_release_strong_persist', 3)
        self.s_release_abs_max = float(
            self.get_parameter('s_release_abs_max').value)
        self.s_release_fast_persist = max(
            1, int(self.get_parameter('s_release_fast_persist').value))
        self.s_release_strong_persist = max(
            1, int(self.get_parameter('s_release_strong_persist').value))
        # --- 独立的 release-residual 判据(2026-09-05)---
        # 与 confirm_thresh **完全解耦**:后者还管事件权重调度与 attach/drop 的
        # 质量突变检测,降它会改变 MHE 收敛行为、混淆实验结论。这条只投"释放票"。
        # 形式:短窗方向阶跃 ΔT = mean(post half) - mean(pre half),不复用慢基线。
        #
        # 阈值按**系统最低支持载荷 0.15kg** 的检测规格定(m_P·g=1.47N),不是用
        # 本轮载荷真值。0.2kg 高频回放(fixtures/tphys_replay_20260826_165455)标定:
        #   figure-8 稳定段 ΔT 最负 -1.69N —— **比 0.15kg 的真信号 1.47N 还大**,
        #   所以幅度上不可分,任何能检出 0.15kg 的绝对阈值都会被机动瞬态碰到。
        #   可用的区分维度只有**持续性**(drop 是永久台阶,机动是暂态):
        #     T_floor=0.9N -> fig8 最长连续 3 帧, drop 连续 4 帧
        #     T_floor=1.2N -> fig8 最长连续 1 帧, drop 连续 3 帧
        # 于是:普通票门槛低(0.9N/2帧),只允许与**质量通道**配对(载荷还在时
        # quantity_empty 恒假,机动误报无害);强票靠更长的持续(4帧)把 fig8 的
        # 3 帧甩开,才允许与 moment 塌陷配对。
        # ⚠️ z 归一化实测**没有区分力**:σ(MAD)≈0.13N 太小,越过 T_floor 的样本
        #    自动 z>6,z=3/4/6 的结果逐帧相同。保留参数但默认 0(关),别拿它当
        #    第二道闸门 —— 那会是虚假的严格。
        self.declare_parameter('release_resid_enable', True)
        self.declare_parameter('release_resid_half', 3)          # 半窗帧数 0.3s
        self.declare_parameter('release_resid_floor_n', 0.9)     # 普通票 [N]
        self.declare_parameter('release_resid_persist', 2)
        self.declare_parameter('release_resid_strong_floor_n', 0.9)
        self.declare_parameter('release_resid_strong_persist', 4)
        self.declare_parameter('release_resid_z', 0.0)           # 0=不用 z
        self.release_resid_enable = bool(
            self.get_parameter('release_resid_enable').value)
        self.release_resid_half = max(
            1, int(self.get_parameter('release_resid_half').value))
        self.release_resid_floor = float(
            self.get_parameter('release_resid_floor_n').value)
        self.release_resid_persist = max(
            1, int(self.get_parameter('release_resid_persist').value))
        self.release_resid_strong_floor = float(
            self.get_parameter('release_resid_strong_floor_n').value)
        self.release_resid_strong_persist = max(
            1, int(self.get_parameter('release_resid_strong_persist').value))
        self.release_resid_z = float(
            self.get_parameter('release_resid_z').value)
        self._rr_hist = []          # T_phys 短历史
        self._rr_d_hist = []        # ΔT 历史(算滚动 sigma)
        self._rr_n = 0              # 普通票连续帧
        self._rr_strong_n = 0       # 强票连续帧
        # 残差 DROP 证据的时效 [s]。残差是**边沿**事件,过期不该再算数;
        # 给 3s 是为了覆盖"残差先看见、质量域随后确认"的正常先后关系。
        self.declare_parameter('residual_evidence_hold_sec', 3.0)
        self.residual_evidence_hold_sec = float(
            self.get_parameter('residual_evidence_hold_sec').value)
        self._residual_drop_evidence_until = -1.0
        self._residual_strong_evidence_until = -1.0
        # --- moment reference(2026-09-05):取代 running max 作为 ratio 的分母 ---
        # 物理上限 = 载荷包线 x attach 偏心上限。越界样本是动态失真,不是几何:
        # 15 轮实测 7.6%~18.3% 的带载帧越界(max 0.052,反解 r_xy=0.35m)。
        # ⚠️ 该缺陷在 ddfe9d2 之前就存在;A 修复只是让 detector 更早开始跑,把越界率
        #    从 7.6% 抬到 18.3%,不是根因。
        # 由**共享的平台包线**派生,不写死(2026-09-05):两个节点从同一份
        # config/gripper/gripper_params.yaml 的 /** 段读这两个值,避免
        # 0.039 / 0.30 / 0.13 三处独立漂移。这是机架规格,不是任务信息,
        # 也不是 attach/drop 事件信号 —— 它不进估计器的观测通路。
        self.declare_parameter('payload_mass_envelope', 0.30)  # kg
        self.declare_parameter('payload_rxy_envelope', 0.13)   # m
        # override 仅供测试:>0 时覆盖派生值并打印显式警告。
        self.declare_parameter('moment_abs_max', 0.0)
        self.declare_parameter('moment_ref_window', 20)      # 帧,10Hz -> 2s
        self.declare_parameter('moment_ref_omega_max', 0.15) # rad/s
        self.declare_parameter('moment_ref_vel_max', 0.20)   # m/s
        self.declare_parameter('moment_ref_max_cv', 0.25)    # 窗口离散度上限
        self.declare_parameter('moment_ref_dir_min', 0.90)   # 方向一致度下限
        _m_env = float(self.get_parameter('payload_mass_envelope').value)
        _r_env = float(self.get_parameter('payload_rxy_envelope').value)
        _derived = max(_m_env * _r_env, 1e-6)
        _override = float(self.get_parameter('moment_abs_max').value)
        if _override > 0.0:
            self.moment_abs_max = _override
            self.get_logger().warn(
                f'⚠️ moment_abs_max 被**手动覆盖**为 {_override:.4f} kg·m '
                f'(派生值 {_derived:.4f} = {_m_env:.2f}kg x {_r_env:.2f}m)。'
                f'override 仅供测试,正式批次应让它随平台包线联动。')
        else:
            self.moment_abs_max = _derived
            self.get_logger().info(
                f'[moment-ref] 物理上限由平台包线派生: {_derived:.4f} kg·m '
                f'= {_m_env:.2f}kg x {_r_env:.2f}m')
        # 越界剔除开关(2026-09-10)。默认开 = 不再把物理不可能的 s 喂给 NMPC。
        # 置 false 退回旧行为,仅用于复现 2026-09-10 之前的批次。
        self.declare_parameter('c_xy_clamp_enable', True)
        self.c_xy_clamp_enable = bool(
            self.get_parameter('c_xy_clamp_enable').value)
        self._c_xy_reject = 0     # 因越界被剔除的帧数
        self._c_xy_total = 0      # 走过 c_xy 一阶矩路径的总帧数
        if not self.c_xy_clamp_enable:
            self.get_logger().warn(
                '⚠️ c_xy_clamp_enable=false:|s|>moment_abs_max 的帧会原样进 '
                'NMPC 的 c_xy —— 这是 2026-09-10 之前的旧行为,仅供复现历史批次。')
        self.moment_ref_window = max(
            3, int(self.get_parameter('moment_ref_window').value))
        self.moment_ref_omega_max = float(
            self.get_parameter('moment_ref_omega_max').value)
        self.moment_ref_vel_max = float(
            self.get_parameter('moment_ref_vel_max').value)
        self.moment_ref_max_cv = float(
            self.get_parameter('moment_ref_max_cv').value)
        self.moment_ref_dir_min = float(
            self.get_parameter('moment_ref_dir_min').value)
        self._moment_ref_ready = False
        self._s_ref_loaded = 0.0
        self._moment_ref_buf = []
        self._moment_ref_reject = 0       # estimator-quality 指标
        self._dyn_overrange = 0           # 动态段越界帧计数(同上)
        self._s_peak = 0.0
        self._s_low = 0
        self._s_fast = 0
        self._s_strong = 0
        self._s_ratio_log_n = 0
        # 释放检测器自检：一阶矩统一判决应在载荷武装后每拍
        # 执行。既往批次三次出现“UNRESOLVED 兜底接住了检测器根本
        # 没运行”，因此这里不仅打启动配置，还看实际执行心跳。
        self.declare_parameter('release_detector_watchdog_sec', 3.0)
        self.declare_parameter('release_detector_warn_repeat_sec', 10.0)
        self.release_detector_watchdog_sec = max(
            mhe_p.dt, float(self.get_parameter(
                'release_detector_watchdog_sec').value))
        self.release_detector_warn_repeat_sec = max(
            self.release_detector_watchdog_sec, float(self.get_parameter(
                'release_detector_warn_repeat_sec').value))
        self._release_detector_eval_frame = None
        self._release_detector_armed_frame = None
        self._release_detector_last_warn_frame = None
        if self.c_xy_mass_release_mp > 0.0 and self.geom_release_mode != 'self':
            self.get_logger().error(
                "c_xy_mass_release_mp 只在 geom_release_mode='self' 下可用"
                "(event 档 _release_payload 会清 attach_offset,而该档模型正读它"
                " -> m_est 关掉模型几何 = 自举耦合)。已停用。")
            self.c_xy_mass_release_mp = 0.0
        elif self.c_xy_mass_release_mp > 0.0:
            self.get_logger().info(
                f'[c_xy_est] 质量域卸载判据已开:先武装(m_p>'
                f'{self.c_xy_mass_release_mp * self.c_xy_mass_arm_ratio:.3f}kg)'
                f',再判 m_p<{self.c_xy_mass_release_mp}kg 持续 '
                f'{self.c_xy_mass_release_persist} 帧 -> 释放偏心估计')

        # --- c_xy 由一阶质量矩 s 直接给出(2026-08-26)---
        # 现状的问题(实测 gviz_20260826_194614):发给 NMPC 的 c_xy 走的是窗外
        # EMA + **稳态门控**(|ω|>0.15 或 |v_xy|>0.2 就直接 return)。figure-8 的
        # 线速度是 r·w=2.83 m/s,比门限大 14 倍 —— **机动中它一次都不更新**。
        # 于是 drop 之后 c_xy 冻结在带载值(实测 cy=-0.0084 一直不动),NMPC 只能
        # 靠外部 drop 事件把几何清零;一旦让 NMPC 也"自己看"(geom_release_mode
        # ='self'),它就会拿着这个幽灵偏心配平:0.0084×20.25N=0.17N·m 作用在空机
        # J_xx=0.0142 上 = 12 rad/s²,半秒滚到 370°/s —— 这正是 2026-08-24 那两轮
        # NMPC 'self' 坠机的机制。
        #
        # 修法:estimate_moment 档下 s=m_P·r_xy 是**窗口内被估的状态**,有独立观测
        # (τ_phys/T),不需要稳态门控;c_xy = s/m_T 直接由它给出。实测 s 在 drop 后
        # 2s 内从 0.0187 降到 0.0027(-86%)、4s 到 0.0010(-95%),而带载段它估的
        # 0.0187 与真值 m_P·r_y=0.2×0.093=0.0186 几乎重合。
        # 这条路打通后 NMPC 才谈得上"自己发现载荷没了" —— 在此之前它手里的偏心
        # 是个冻结的常数,谈不上估计。
        # ⚠️ 只在 ns>0(MHE_ESTIMATE_MOMENT=1)时可用;默认关,保持既有行为。
        self.declare_parameter('c_xy_from_moment', bool(mhe_p.ns))
        self.c_xy_from_moment = bool(
            self.get_parameter('c_xy_from_moment').value)
        if self.c_xy_from_moment and not mhe_p.ns:
            self.get_logger().error(
                'c_xy_from_moment=True 需要 MHE_ESTIMATE_MOMENT=1(s 是那一档才有的'
                '状态),已退回窗外 EMA + 稳态门控。')
            self.c_xy_from_moment = False
        elif self.c_xy_from_moment:
            self.get_logger().info(
                '[c_xy] 来源 = 一阶质量矩 s/m_T(窗口内估计,**无稳态门控**),'
                '替代窗外 EMA —— 机动中也会更新')
        self.c_xy_est = np.zeros(2)   # [cx, cy] 估计
        self._c_xy_inited = False

        # ---- 载荷**意外**脱落看门狗(2026-08-30)----
        # 动机:内环增益在 attach 时按包线放大到 5×,复位的唯一触发是 nmpc_node
        # 的 grip_dropped —— 而那条只在"夹爪是它自己松的"时置位。真机上夹爪失效
        # /载荷被扯掉时**没有任何复位路径**,5× 过增益留在空机上 = 高频振荡→姿态
        # 发散→掉高(2026-07-14 实测过这个崩法)。这里补一条 MHE→NMPC 的安全网。
        # 与 _residual_detect(A.2 无信号消融)**刻意分开实现**:那条的语义和已发表
        # 结果绑在 event_signal_mode='residual' 上,不能为了安全网去动它的触发条件。
        # 两层判据(见 payload_lost_watch_enable 的取值理由):
        #   快层 = T_phys 双窗阶跃(反应快,但灵敏度随载荷质量缩放,轻载会漏)
        #   慢层 = 倾角修正后的推力隐含质量持续贴近空机(与载荷质量无关,但要等)
        self.declare_parameter('payload_lost_watch_enable', False)
        self.declare_parameter('payload_lost_step_thresh', 2.0)   # N,快层
        self.declare_parameter('payload_lost_step_half', 3)       # 半窗帧数
        self.declare_parameter('payload_lost_step_persist', 2)
        self.declare_parameter('payload_lost_mass_margin', 0.10)  # kg,慢层
        self.declare_parameter('payload_lost_hold_sec', 3.0)
        self.declare_parameter('payload_lost_vz_gate', 0.30)      # m/s
        self.payload_lost_watch = bool(
            self.get_parameter('payload_lost_watch_enable').value)
        self.pl_step_thresh = float(
            self.get_parameter('payload_lost_step_thresh').value)
        self.pl_step_half = int(self.get_parameter('payload_lost_step_half').value)
        self.pl_step_persist = int(
            self.get_parameter('payload_lost_step_persist').value)
        self.pl_mass_margin = float(
            self.get_parameter('payload_lost_mass_margin').value)
        self.pl_hold_frames = int(round(
            float(self.get_parameter('payload_lost_hold_sec').value) / mhe_p.dt))
        self.pl_vz_gate = float(self.get_parameter('payload_lost_vz_gate').value)
        self._pl_hist = []
        self._pl_step_pending = 0
        self._pl_hold = 0
        if self.payload_lost_watch:
            self.get_logger().info(
                f'[payload-lost] 看门狗 on: 快层 |ΔT|≥{self.pl_step_thresh}N '
                f'(双窗 {self.pl_step_half}帧, persist {self.pl_step_persist}) | '
                f'慢层 T·cosθ/g < m_B+{self.pl_mass_margin}kg 持续 '
                f'{self.pl_hold_frames * mhe_p.dt:.1f}s (|vz|<{self.pl_vz_gate})')

        # ---- 机动门控:降低机动期质量估计的权重(2026-08-25)----
        # 动机:m_est 静态准(-0.17%)但**机动中系统性低估 8%**(记忆
        # new-4ms-workpoint-validation / high-maneuver-ablation 实测)。机动期
        # 的残差里混着未建模气动、转子角加速度、内环跟踪滞后,这些都被归因到
        # 质量维;而质量本身在机动期并没有变。把这些帧的估计权重降下来,既减少
        # 污染,也切细自举耦合(m_est→dJ/c→NMPC→飞行→残差→m_est)在机动期的增益
        # —— 耦合最危险的正是这一段。
        #
        # 旋钮选 Q0 的质量维(到达代价锚),不是 R/Q:
        #   Q0[m] 大 = 强锚定到上一窗口的估计 = "粘",当前窗口推不动它;
        #   Q0[m] 小 = 自由跟随当前窗口数据。
        # ⚠️ 这与 mhe_event_weights 里"不调 Q0"那条注释不矛盾:那里的瓶颈是
        #    "窗口内跨阶跃的旧测量 + m_dot=0 刚性",Q0 确实不是瓶颈;这里要压的
        #    恰恰是"当前窗口数据对质量的推动力",Q0 正是它的正确表达。
        # ⚠️ 与事件过渡**互斥**:过渡期让位不干预(两套 stage-0 W 会互相覆盖,
        #    而且同时上线会让归因说不清 —— 本仓一贯做法)。
        # 门限沿用 c_xy_est 那套稳态判据的同一口径。默认关。
        self.declare_parameter('maneuver_gate_enable', False)
        self.declare_parameter('maneuver_omega_thresh', 0.15)   # rad/s
        self.declare_parameter('maneuver_vel_thresh', 0.20)     # m/s
        self.declare_parameter('maneuver_q0_cap', 1.0e4)        # 最大锚紧倍数
        self.declare_parameter('maneuver_exponent', 2.0)
        self.maneuver_gate_enable = bool(
            self.get_parameter('maneuver_gate_enable').value)
        self.maneuver_omega_thresh = float(
            self.get_parameter('maneuver_omega_thresh').value)
        self.maneuver_vel_thresh = float(
            self.get_parameter('maneuver_vel_thresh').value)
        self.maneuver_q0_cap = float(self.get_parameter('maneuver_q0_cap').value)
        self.maneuver_exponent = float(
            self.get_parameter('maneuver_exponent').value)
        self._mg_gain_applied = None    # 上次实际写进 solver 的锚紧倍数
        self._mg_frames_gated = 0       # 被降权的帧数(统计用)
        self._mg_frames_total = 0
        if self.maneuver_gate_enable:
            self.get_logger().info(
                f'[maneuver-gate] 已开启: |ω|>{self.maneuver_omega_thresh} rad/s '
                f'或 |v|>{self.maneuver_vel_thresh} m/s 时按 lvl^'
                f'{self.maneuver_exponent} 锚紧 Q0[m](cap {self.maneuver_q0_cap:.0e})')

        theta = np.array(self.get_parameter('schedule_theta').value,
                         dtype=float)
        self.scheduler = ParametricWeightSchedule(theta)
        self.get_logger().info(
            f'weight schedule theta = [{", ".join(f"{v:.2f}" for v in theta)}]'
            f', signal mode = {self.signal_mode}'
            + (f' (residual self-trigger: thresh={self.confirm_thresh:.1f}N '
               f'persist={self.resid_persist} baseline_tau='
               f'{mhe_p.dt / self.resid_ema_alpha:.1f}s)'
               if self.signal_mode == 'residual' else ''))
        self._in_transition = False  # 上一窗口是否处于事件过渡期(日志用)
        self._armed_frame = None     # 检测器武装时刻的帧序号(None=未武装)
        self._armed_baseline = None  # 武装时刻的 T_phys 基线
        self._log_until_frame = -1   # 事件后逐帧日志窗口(与调度解耦,对照组也打)

        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_cb, mavros_sensor_qos)
        # sensor-only 主线不读取 NMPC 意图输入;motor_speed_cb 直接提供完整的
        # [T,tau]。打开 external_event_inputs 才恢复旧的 command/event 接线。
        if self.external_event_inputs:
            self.u_opt_sub = self.create_subscription(
                Float64MultiArray, '/acados_nmpc/u_opt', self.u_opt_cb, 10)
        # PX4 发给 Gazebo 电机模型的转速指令(经 ros_gz_bridge 桥接,跟
        # prop_joint_state_publisher 用的是同一个话题/同一套 QoS=depth10)。
        # 话题名做成参数:mass_changer 场景是 x500_payload_0,夹爪场景是空机
        # x500_0。默认保持原值,不影响现有 run_sitl_acados.sh。
        self.declare_parameter('motor_speed_topic',
                               '/x500_payload_0/command/motor_speed')
        motor_topic = self.get_parameter('motor_speed_topic').value
        self.motor_speed_sub = self.create_subscription(
            Actuators, motor_topic, self.motor_speed_cb, 10)

        # 质量突变事件源(两个场景,同一处理):
        # - wrench baseline:acados_nmpc_node 触发 drop 的同时发 /acados_nmpc/
        #   mass_event(它就是事件的发起者,不用去桥接 gz topic);
        # - gripper 场景:proximity 节点 attach 瞬间发 /gripper/attach_offset。
        #   注意发布方是 TRANSIENT_LOCAL latched,这里故意用默认 VOLATILE 订阅
        #   ——只收活消息,MHE 重启时不会把旧的 latched attach 当成新事件。
        if self.external_event_inputs:
            self.mass_event_sub = self.create_subscription(
                Empty, '/acados_nmpc/mass_event', self.mass_event_cb, 10)
            self.attach_event_sub = self.create_subscription(
                Float64MultiArray, '/gripper/attach_offset',
                self.attach_event_cb, 10)

        self.mass_pub = self.create_publisher(
            Float64, '/acados_nmpc/mhe_mass_estimate', 10)
        # 体力矩反算发布(B.3 Phase0):[roll,pitch,yaw] Nm,供强闭环 Δr 估计器订阅
        self.tau_phys_pub = self.create_publisher(
            Float64MultiArray, '/acados_nmpc/tau_phys', 10)
        # 在线 c_xy 估计发布(B.3 Phase1):[cx,cy] m,Phase2 合闸时 NMPC 订阅它替代
        # attach 真值反推的几何(Phase1 只发+记录,NMPC 暂不吃)
        self.c_xy_est_pub = self.create_publisher(
            Float64MultiArray, '/acados_nmpc/c_xy_est', 10)
        # 原子化连续估计帧。字段顺序由 payload_estimate.FIELDS 唯一定义;
        # NMPC 主线只订阅这一条,不会混用不同 MHE 周期的 m/s/J。
        self.payload_estimate_pub = self.create_publisher(
            Float64MultiArray, '/mhe/payload_estimate', 10)
        # 载荷意外脱落告警(见 payload_lost_watch_enable):MHE→NMPC 的唯一一条
        # 反向通路,收方复位内环增益 + 归零几何。RELIABLE 默认 QoS 即可(单次事件)。
        self.payload_lost_pub = self.create_publisher(
            Empty, '/mhe/payload_lost', 10)

        # ---- 残差可辨识性诊断采集(2026-07-31,默认关) ----
        # residual_log_dir 为空 = 完全关闭,回调里只多一次 `if not enabled: return`。
        # 开启时四条异步原始流各写各的 CSV,**不在线拼表**(三个话题时钟域/频率
        # 不同,在线拼行会把时间错位烙进数据,而那正是要诊断的东西)。
        self.declare_parameter('residual_log_dir', '')
        _rl_dir = str(self.get_parameter('residual_log_dir').value)
        _stamp = time.strftime('%Y%m%d_%H%M%S')
        self.resid_log = ResidualLogger(_rl_dir, _stamp, meta={
            # 标定常数:tau_phys 的标签质量完全由这几个数决定
            'MOTOR_CONSTANT': MOTOR_CONSTANT,
            'THRUST_CAL_GAIN': THRUST_CAL_GAIN,
            'ROTOR_X': list(ROTOR_X), 'ROTOR_Y': list(ROTOR_Y),
            'TORQUE_SIGN': TORQUE_SIGN,
            'MOMENT_CONSTANT': MOMENT_CONSTANT,
            'ROTOR_DIR': list(ROTOR_DIR), 'YAW_TORQUE_SIGN': YAW_TORQUE_SIGN,
            'tau_source': self.tau_source,
            'motor_window_avg': int(bool(self.motor_window_avg)),
            # 刚体常数:算 J·omega_dot + omega x J·omega 要用
            'Jxx': mhe_p.Jxx, 'Jyy': mhe_p.Jyy, 'Jzz': mhe_p.Jzz,
            'm_nominal': mhe_p.m_nominal, 'g': mhe_p.g,
            'mhe_N': mhe_p.N, 'mhe_dt': mhe_p.dt,
            'grip_payload_envelope': self.grip_payload_envelope,
            'geom_coupled': int(bool(mhe_p.geom_coupled)),
            'eval_true_payload_mass': self.eval_true_payload_mass,
            'motor_speed_topic': motor_topic,
            # 语义声明:后处理不能靠猜
            'odom_frame': 'twist.linear/angular = body FLU (原始值,未转 world)',
            'tau_phys_def': ('tau_xy=TORQUE_SIGN*sum(ROTOR_{Y,-X}*k_f*w^2); '
                             'tau_z=YAW_TORQUE_SIGN*k_m*sum(YAW_DIR*k_f*w^2) '
                             '(2026-08-25 起不再是 0 占位). '
                             '执行器输出力矩,不含 J*omega_dot,也不含转子角加速度项'),
            'command_semantics': 'u_opt=NMPC理想输出[T,tx,ty,tz],非执行器输入(下游转 body-rate setpoint 给PX4)',
            'command_has_header': 'False (Float64MultiArray 无时间戳,只有接收时刻)',
            'clock_note': 'use_sim_time 未设;motor 经 ros_gz_bridge(Gazebo), odom 经 mavros(PX4) — 时钟域须用数据验证',
        })
        if self.resid_log.enabled:
            # 仅诊断采集时才订阅 IMU(MHE 本身不用它),避免平时多一路无谓回调。
            # 动机:odom 实测只有 ~31Hz,用它差分算 omega_dot 会被微分噪声主导;
            # IMU 的 angular_velocity 频率高得多,是 omega_dot 的更好来源。
            from sensor_msgs.msg import Imu
            self.imu_sub = self.create_subscription(
                Imu, '/mavros/imu/data',
                lambda m: self.resid_log.log_imu(m), mavros_sensor_qos)
            self.get_logger().info(f'[resid] 诊断采集已开启 -> {_rl_dir}'
                                   ' (含 IMU 流)')

        self._report_release_detector_startup()
        self.timer = self.create_timer(mhe_p.dt, self.timer_cb)
        self.get_logger().info(
            f'MHE node initialized! tau_source={self.tau_source} '
            f'(roll/pitch {"电机转速反算" if self.tau_source in ("phys", "phys_full") else "NMPC意图值"}'
            f', yaw {"电机转速反算(完全解耦)" if self.tau_source == "phys_full" else "NMPC意图值"})'
            f', motor_window_avg={self.motor_window_avg}'
            f', geom_release_mode={self.geom_release_mode}. '
            'Waiting for odometry + control data...')

    def _release_detector_config(self):
        """返回统一 moment 判决与 legacy 质量判决的实际可用性。"""
        moment = bool(
            self.c_xy_est_enable and self.s_release_ratio > 0.0 and mhe_p.ns)
        mass = bool(
            self.c_xy_est_enable and self.c_xy_mass_release_mp > 0.0
            and self.geom_release_mode == 'self')
        return moment, mass

    def _report_release_detector_startup(self):
        """启动时明示检测器是 READY 还是被哪个总开关关住。"""
        moment, mass = self._release_detector_config()
        if moment:
            self.get_logger().info(
                f'[release-selfcheck] READY: c_xy_est_enable=1, '
                f'moment-state ns={mhe_p.ns}, ratio={self.s_release_ratio:.2f}; '
                f'legacy_mass={int(mass)}, watchdog='
                f'{self.release_detector_watchdog_sec:.1f}s')
            return

        blockers = []
        if not self.c_xy_est_enable:
            blockers.append('c_xy_est_enable=false(master gate)')
        if not mhe_p.ns:
            blockers.append('MHE_ESTIMATE_MOMENT=0(ns=0)')
        if self.s_release_ratio <= 0.0:
            blockers.append('s_release_ratio<=0')
        self.get_logger().warn(
            '[release-selfcheck] UNIFIED DETECTOR OFF: '
            + ', '.join(blockers)
            + f'; legacy_mass={int(mass)}. 带载后若无其他可靠释放路径，'
              '系统将保持载荷模型并进入 UNRESOLVED。')

    def _check_release_detector_watchdog(self):
        """已武装后长时间无统一判决心跳时节流报警。"""
        if not self._load_armed:
            self._release_detector_armed_frame = None
            self._release_detector_last_warn_frame = None
            return
        if self._release_detector_armed_frame is None:
            self._release_detector_armed_frame = self.frames

        last_eval = self._release_detector_eval_frame
        baseline = (self._release_detector_armed_frame
                    if last_eval is None else last_eval)
        silent_sec = max(0.0, (self.frames - baseline) * mhe_p.dt)
        if silent_sec < self.release_detector_watchdog_sec:
            return

        last_warn = self._release_detector_last_warn_frame
        if (last_warn is not None and
                (self.frames - last_warn) * mhe_p.dt
                < self.release_detector_warn_repeat_sec):
            return
        moment, mass = self._release_detector_config()
        self.get_logger().warn(
            f'[release-selfcheck] ARMED BUT NO UNIFIED DETECTOR EVALUATION '
            f'for {silent_sec:.1f}s: c_xy_est_enable='
            f'{int(self.c_xy_est_enable)}, ns={mhe_p.ns}, '
            f's_release_ratio={self.s_release_ratio:.2f}, '
            f'unified_ready={int(moment)}, legacy_mass={int(mass)}. '
            f'UNRESOLVED may be masking a disabled/misconfigured detector.')
        self._release_detector_last_warn_frame = self.frames

    def odom_cb(self, msg):
        self.resid_log.log_odom(msg)   # 记原始 body 值,在任何变换之前
        # 跟 acados_nmpc_node.odom_cb 完全一样的 body(FLU)->world(ENU) 速度转换,
        # 两边必须用同一套约定,否则喂给 MHE 的"测量"跟它的动力学模型对不上。
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        om = msg.twist.twist.angular
        qn = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if qn < 1e-6:
            return
        qw, qx, qy, qz = q.w/qn, q.x/qn, q.y/qn, q.z/qn
        R_wb = quat_to_rotmat(qw, qx, qy, qz)
        vel_world = R_wb @ np.array([vel.x, vel.y, vel.z])
        x = np.array([
            pos.x, pos.y, pos.z,
            vel_world[0], vel_world[1], vel_world[2],
            qw, qx, qy, qz,
            om.x, om.y, om.z
        ])
        if not np.all(np.isfinite(x)):
            return
        self.x_meas = x
        self._last_odom_rx_sec = self.get_clock().now().nanoseconds * 1e-9

    def motor_speed_cb(self, msg):
        if len(msg.velocity) >= 4:
            w = np.array(msg.velocity[:4])
            if np.all(np.isfinite(w)):
                w2 = w * w
                self.thrust_phys = float(
                    THRUST_CAL_GAIN * MOTOR_CONSTANT * np.sum(w2))
                # 体力矩反算(B.3 Phase0,见文件头 ROTOR_X/Y 注释)。F_i=k_f·ω_i²,
                # τ_roll=Σy_i·F_i, τ_pitch=-Σx_i·F_i(FLU)。yaw 反扭这里不需要
                # (c_xy 只靠 roll/pitch),留 0 占位保持三维接口。
                f = THRUST_CAL_GAIN * MOTOR_CONSTANT * w2
                self._motor_f_acc += f
                self._motor_n += 1
                tau_roll = TORQUE_SIGN * float(np.sum(ROTOR_Y * f))
                tau_pitch = TORQUE_SIGN * float(-np.sum(ROTOR_X * f))
                # yaw 反扭矩(见 MOMENT_CONSTANT 注释):不再是 0 占位。
                tau_yaw = YAW_TORQUE_SIGN * MOMENT_CONSTANT * float(
                    np.sum(YAW_DIR * f))
                self.tau_phys = np.array([tau_roll, tau_pitch, tau_yaw])
                _now = self.get_clock().now().nanoseconds * 1e-9
                self._last_motor_rx_sec = _now
                self.resid_log.log_motor(msg, w, self.tau_phys,
                                         self.thrust_phys)
                # pre-offboard 冷启动:NMPC 还没接管时,用纯电机反算值合成
                # u_known,让 timer_cb 的 `u_known is None` 那道门放行。
                # 一旦 u_opt 到达(_u_from_nmpc=True)就永久让位给 u_opt_cb。
                if ((not self.external_event_inputs
                     or (self.pre_offboard_estimate and not self._u_from_nmpc))
                        and self._pre_offboard_gate_ok()):
                    self.u_known = np.array(
                        [self.thrust_phys, tau_roll, tau_pitch, tau_yaw])
                    self._last_u_rx_sec = _now

    def _mass_observable(self):
        """质量估计当前是否可信到能用来判"空载"(低机动窗口)。

        机动中 m_p 周期性探底到负值,任何正阈值都拦不住,所以 figure8 这类高动态
        段**不累计**退出计数(而不是放宽阈值——放宽只是把误判推迟)。进入 LOADED
        不受此门控:m_p 高于入阈是正证据,机动只会让它偏低、不会凭空造出载荷。
        """
        if self.x_meas is None:
            return False
        om = float(np.linalg.norm(self.x_meas[10:13]))
        v_xy = float(np.linalg.norm(self.x_meas[3:5]))
        return (om <= self.payload_exit_steady_omega
                and v_xy <= self.payload_exit_steady_vel)

    def _update_payload_presence(self, event=None):
        """更新自主载荷存在状态(见 __init__ 里 _payload_present)。

        两条入口:
          event='attach'/'drop' —— 残差检测器自己看出来的质量突变,快速通道;
          event=None            —— 逐帧的持续证据,用 m_p 的双阈值+持续帧数。
        双阈值(进 0.09kg / 出 0.03kg)是迟滞,不是随手取的两个数:attach 收敛期
        m_p 天然偏小,单阈值会在 LIFT 刚开始就来回翻(质量域判据第一版就是这么
        栽的,见 c_xy_mass_arm_ratio 的注释)。

        进入 LOADED 时解除 s 的释放闩:重新带载后 s 的发布目标必须回到估计值。
        """
        prev = self._payload_present
        if event == 'attach':
            self._payload_present = True
            self._present_hi = self._present_lo = 0
        elif event == 'drop':
            self._payload_present = False
            self._present_hi = self._present_lo = 0
        else:
            m_p = float(self.m_est) - mhe_p.m_B
            if not self._payload_present:
                if m_p > self.payload_present_enter_mp:
                    self._present_hi += 1
                    if self._present_hi >= self.payload_present_enter_persist:
                        self._payload_present = True
                        self._present_hi = 0
                else:
                    self._present_hi = 0
            else:
                if not self._mass_observable():
                    self._present_lo = 0      # 高动态:清除退出计数,不冻结
                elif m_p < self.payload_present_exit_mp:
                    self._present_lo += 1
                    if self._present_lo >= self.payload_present_exit_persist:
                        self._payload_present = False
                        self._present_lo = 0
                else:
                    self._present_lo = 0
        if self._payload_present != prev:
            src = event if event else 'sustained m_p evidence'
            self.get_logger().info(
                f'[payload-state] {"EMPTY->LOADED" if self._payload_present else "LOADED->EMPTY"}'
                f' ({src}, m_p={float(self.m_est) - mhe_p.m_B:+.3f}kg)')
            if self._payload_present:
                self._load_armed = True       # 本轮确实带上过载荷
                self._s_release_latched = False

    def _release_payload(self, src):
        """载荷卸载时的**对称释放**:三条触发路径(外部 mass_event / 残差慢基线
        自检测 / 残差阶跃自检测)共用这一处,免得再漏项。**只释放几何/偏心,
        不动 m_est** —— 质量状态要让估计器自己连续收敛回空机,否则验证不了
        "模型正确后估计器能回归"(理由同 mass_event_cb 原注释)。

        2026-08-30 加 c_xy_est 归零:那条支路**自己没有卸载检测**,更新入口
        _update_c_xy_est 的稳态门控(|ω|<c_xy_steady_omega、|v_xy|<c_xy_steady_vel)
        在机动中永远不满足,EMA 也就不会自行衰减 —— 实测 drop 后 138s 仍停在
        带载值 cy=-10.4mm(真值 0)。模型侧没吃到脏值是 nmpc_node 的 grip_dropped
        分支兜住的,但话题 /acados_nmpc/c_xy_est 上跑的仍是幽灵偏心,任何别的
        订阅者都会读到。与 2026-07-15 那个 "nmpc 侧归零了、mhe 侧漏了对称处理"
        是同构缺陷,只是漏在这条支路上。
        清 _c_xy_inited 而不只是清值:下次进稳态窗口时用 c_inst 直接重播种,
        不必从带载旧值慢慢 EMA 爬回来。"""
        self._payload_attached = False
        self.attach_offset = None
        # s 的目标切零(见 __init__ 里 _s_release_latched)。清的是**发布目标**,
        # 不是 self.s_est ——后者是窗口解出来的状态,下一帧照样会算出残值来;
        # 真正要断的是它到 _s_out 的那条链。_s_out 自己按 LPF 连续衰减。
        self._s_release_latched = True
        # 释放真的发生了,才清"本轮曾加载"的 latch。清早了(比如临时 EMPTY 就清)
        # 就会把随后的真 drop 挡在门外。
        self._load_armed = False
        self._s_decay_n = 0            # 开始记 [s-decay]
        self._payload_present = False
        self._present_hi = 0
        self._present_lo = 0
        self._s_peak = 0.0
        # moment 基准随载荷一起作废:下一次任务必须重新建立自己的基准,
        # 否则会拿上一轮的分母去判这一轮(2026-09-05)。
        self._moment_ref_ready = False
        self._s_ref_loaded = 0.0
        self._moment_ref_buf = []
        self._moment_ref_reject = 0
        self._dyn_overrange = 0
        self._s_low = 0
        if self._c_xy_inited or bool(np.any(self.c_xy_est)):
            self.c_xy_est = np.zeros(2)
            self._c_xy_inited = False
            self.c_xy_est_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            self.get_logger().info(
                f'[c_xy_est] 载荷释放({src}) → 在线偏心估计归零(重新播种)')

    def _payload_lost_watch(self):
        """载荷**意外**脱落看门狗(2026-08-30)。两层并行判据,任一命中就发
        /mhe/payload_lost 并本地释放几何。只在"MHE 认为载荷还在机上"时工作;
        命中后 _payload_attached 已置 False,自然不会重复触发。

        与 _residual_detect(A.2 无信号消融)**刻意分开实现**:那条的语义和已
        发表结果绑在 event_signal_mode='residual' 上,不该为了安全网去改它的
        触发条件;这条只服务"复位内环增益",判据可以按自己的风险取舍来定。

        ⚠️ 误触发的代价 = 载荷还在却把增益复位 → 内环带宽掉回 1/4 → 正是
        2026-07-02 ulog 实测的 ~1Hz 增幅振荡崩法。所以两层都取保守方向:
          快层门槛(默认 2.0N)高于 A.2 降权用的 1.0N;实测 0.3kg 掉包阶跃
            2.53N、检测器口径(0.3s 双窗)噪声 σ≈0.30N ⇒ 8σ 余量。
          慢层用 T·cosθ/g,机动中该量只会**偏大**(要多给推力),方向上只会
            漏检不会误报;再加 |vz| 门,垂直加速段推力与质量不对应时不判。
        两层的分工:快层反应快但灵敏度随载荷质量缩放(0.2kg 阶跃只有 ~1.7N,
        会被 2.0N 门漏掉);慢层与载荷质量无关但要等 hold_sec。而增益是按
        **包线**放大的(掉多轻的载荷都留着同样的 5× 过增益),所以轻载必须靠
        慢层兜住。"""
        if not self._payload_attached or self.thrust_phys is None:
            return
        T = float(self.thrust_phys)

        # ★★ 2026-08-31:|vz| 门**提到两层之前**(原来只有慢层有,快层没有)。
        # 根因实测(看门狗 on/off 批次 rep3_on,grip_nmpc_20260830_235339):
        # LIFT 抬升坡道 vz≈0.65m/s,爬升后推力回落被快层读成 ΔT=−2.40N(门槛 2.0N)
        # → **载荷还挂着就把内环增益从 5.0 打回 1.0** → 复现 07-02 的 ~1Hz 增幅
        # 振荡 → 发散 15.69m。真正的脱落在那之后 **35 秒**才发生。
        # 即"推力与质量不对应的时段两层都不判",慢层本来就是这个语义,快层漏了。
        # ⚠️ 代价:抬升途中真脱落会漏检(慢层同样被门住)。接受它 —— 那一段飞机
        # 低、慢,而误触发的代价是直接炸机;n=4 实测误触发率 1/4。
        # ⚠️ 被门住时必须**清空快层缓冲**:否则窗口会横跨门的两侧,拿爬升段的均值
        # 和平飞段的均值作差,那个差本身就是伪阶跃。
        if self.x_meas is None:
            return
        if abs(float(self.x_meas[5])) > self.pl_vz_gate:
            self._pl_hist.clear()
            self._pl_step_pending = 0
            self._pl_hold = 0
            return

        # --- 快层:T_phys 双窗阶跃(与 resid_step 同式,独立缓冲与阈值)---
        self._pl_hist.append(T)
        n2 = 2 * self.pl_step_half
        if len(self._pl_hist) > n2:
            self._pl_hist.pop(0)
        if len(self._pl_hist) == n2:
            h = self.pl_step_half
            d = (sum(self._pl_hist[h:]) - sum(self._pl_hist[:h])) / h
            if d <= -self.pl_step_thresh:
                self._pl_step_pending += 1
                if self._pl_step_pending >= self.pl_step_persist:
                    self._fire_payload_lost(
                        f'快层 ΔT={d:+.2f}N over {n2 * mhe_p.dt:.1f}s '
                        f'(门槛 {self.pl_step_thresh}N, persist '
                        f'{self.pl_step_persist})')
                    return
            else:
                self._pl_step_pending = 0

        # --- 慢层:倾角修正后的推力隐含质量持续贴近空机 ---
        # (|vz| 门已在函数开头统一施加,这里不再重复)
        _q = self.x_meas[6:10]
        _n = float(np.linalg.norm(_q))
        if _n < 1e-6:
            return
        _qw, _qx, _qy, _qz = (_q / _n)
        _c = float(np.clip(1.0 - 2.0 * (_qx * _qx + _qy * _qy), 1e-3, 1.0))
        m_implied = T * _c / mhe_p.g
        if m_implied < mhe_p.m_B + self.pl_mass_margin:
            self._pl_hold += 1
            if self._pl_hold >= self.pl_hold_frames:
                self._fire_payload_lost(
                    f'慢层 T·cosθ/g={m_implied:.3f}kg < '
                    f'{mhe_p.m_B + self.pl_mass_margin:.3f}kg 持续 '
                    f'{self._pl_hold * mhe_p.dt:.1f}s')
        else:
            self._pl_hold = 0

    def _fire_payload_lost(self, why):
        """命中后的统一动作:告警话题 + 本地释放几何/偏心。**不动 m_est**
        (同 _release_payload 的理由)。增益复位在 nmpc_node 侧——那边才有
        mavros 参数客户端,且它自己发起的 drop 已有同一条复位路径。"""
        self.get_logger().warn(f'[payload-lost] 检出载荷意外脱落: {why} '
                               '→ 发 /mhe/payload_lost(NMPC 侧复位内环增益)')
        self.payload_lost_pub.publish(Empty())
        self._release_payload('payload-lost watchdog')
        self._pl_step_pending = 0
        self._pl_hold = 0
        self._pl_hist.clear()

    def mass_event_cb(self, msg):
        """质量突变事件 = **卸载**(wrench drop / gripper drop——nmpc_node 释放
        夹爪时同帧发这条)。真实载荷已不存在,必须立刻释放"幽灵几何",否则
        _payload_geometry 会继续按带载棘轮给 om_dot 算 dJ/c_xy(见 __init__ 里
        _payload_attached 注释)。**只释放几何,不动 m_est**——质量状态由 MHE
        自己从当前值连续收敛回空机,这样才验证得了"模型正确后估计器能回归"。
        wrench 场景本来就没几何(attach_offset 恒 None),这里是无害的 no-op。"""
        self._release_payload('external mass_event')
        self._on_mass_event('drop')

    def attach_event_cb(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.shape[0] == 3 and np.all(np.isfinite(data)):
            self.attach_offset = data
            self._r_p_last = data.copy()   # 'self' 释放模式下模型只认它
            self._payload_attached = True   # 载荷上机(**外部**通知,legacy 档用)
            # s 的释放闩由自主状态机负责解除(见 _update_payload_presence),
            # 这里不碰 —— 外部事件状态与估计器自主状态不混用。
        self._on_mass_event('attach (gripper)')

    def _payload_geometry(self, r_p):
        """由载荷相对机体的几何偏移 r_p=[rx,ry,rz](rz<0)算 (dJ, c_xy)。跟
        acados_nmpc_node.py 的同名方法完全一致(平行轴定理+复合质心),区别
        是这里没有 m_p/m_t 真值。2026-08-26 去先验改造后 m_p_hat 只有**一条**
        来源:grip_payload_envelope(载荷包线上界,机架规格)。原先的三条来源
        (真值直灌 / 操作先验 / m_est 反推+棘轮+floor)全部删除——前两条是任务
        信息(要求部署时知道这趟吊多重),第三条是 07-09 坐实的自举正反馈根因
        ("m_est 反推 m_p_hat 再算 dJ/c_xy、dJ/c_xy 又反过来影响 m_est")。

        (历史:棘轮 + 永久地板曾是打破自举耦合的过渡方案,只护住"低估坍缩到
        0"一个方向,对称的"暂态向上超调被永久锁死"治不了;常数包线两个方向
        一起消除,棘轮已随之删除。)

        ⚠️ 物理状态门控(2026-07-15):载荷不在机上就直接返回零几何,**不看棘轮
        也不看 attach_offset**——棘轮只增不减、attach_offset 保留最后一次几何,
        两者都不能表达"货已经卸了"。漏了这道门 = drop 后按幽灵载荷算 dJ/c_xy,
        质量估计被系统性拖低(见 __init__ 里 _payload_attached 注释)。
        """
        if not self._payload_attached:
            return 0.0, np.zeros(2)
        # 几何标度 = 载荷包线上界(机架规格)。常数、**完全不读 m_est**,
        # 自举耦合在结构上不存在。见 __init__ 里该参数注释。
        m_p_hat = self.grip_payload_envelope
        m_t = mhe_p.m_nominal + m_p_hat
        mu = mhe_p.m_nominal * m_p_hat / m_t if m_t > 0.0 else 0.0
        dJ = mu * (r_p[2] ** 2 + 0.5 * (r_p[0] ** 2 + r_p[1] ** 2))
        c = (m_p_hat / m_t) * np.asarray(r_p[0:2], dtype=float) if m_t > 0.0 \
            else np.zeros(2)
        return float(dJ), c

    def _pre_offboard_gate_ok(self):
        """pre-offboard 段是否允许合成 u_known(起飞门控)。

        两条都必须过:①推力高于 seed_thrust_min(没在飞就免谈);②真实高度
        高于 pre_offboard_min_z。第二条是关键 —— **pre-offboard 段真的会在
        地上**,这跟 offboard 之后不一样(NMPC 在 1.5m 空中才接管,那个坑够
        不着)。地面支持力会让 T_phys 与 m·g 脱钩,此时估出来的质量没有意义。
        """
        if self.thrust_phys is None or not math.isfinite(self.thrust_phys) \
                or self.thrust_phys < mhe_p.seed_thrust_min:
            return False
        if self.x_meas is None or self.x_meas[2] < self.pre_offboard_min_z:
            return False
        return True

    def _update_pre_offboard_convergence(self):
        """冷启动收敛判定:最近 N 帧 m_est 的样本标准差落到阈值以下就算收敛。

        只在 pre-offboard 段(NMPC 未接管)跑;一旦判定收敛就锁存 True,不再
        因为后续抖动翻回未收敛 —— 收敛是个一次性的"闸门打开"事件,不是需要
        逐帧维持的状态(否则 NMPC 会收到断续的质量更新)。
        """
        if not self.pre_offboard_estimate or self._u_from_nmpc \
                or self._pre_off_converged:
            return
        if self._pre_off_first_frame is None:
            self._pre_off_first_frame = self.frames
        self._pre_off_m_hist.append(self.m_est)
        if len(self._pre_off_m_hist) > self.pre_off_conv_frames:
            self._pre_off_m_hist.pop(0)
        if len(self._pre_off_m_hist) < self.pre_off_conv_frames:
            return
        sd = float(np.std(self._pre_off_m_hist, ddof=1))
        if sd < self.pre_off_conv_std:
            self._pre_off_converged = True
            self.get_logger().info(
                f'[cold-start] 收敛! m_est={self.m_est:.4f} kg '
                f'(最近 {self.pre_off_conv_frames} 帧 std={sd:.4f} < '
                f'{self.pre_off_conv_std}), 用时 '
                f'{(self.frames - self._pre_off_first_frame) * mhe_p.dt:.1f}s '
                f'| 开始向 NMPC 发布')

    def _mass_publish_allowed(self):
        """是否允许把 m_est 发给 NMPC。

        pre_offboard_estimate 关闭时恒为 True(历史行为逐字节不变)。开启时,
        pre-offboard 段必须等收敛才放行 —— 未收敛的冷启动值灌进 NMPC 的动力
        学模型是危险的(m 偏小 => 悬停推力算低 => 掉高度)。超时兜底:等太久
        还不收敛就放行并 WARN,避免退化成"NMPC 永远收不到估计"(那等于悄悄
        变成 use_mhe=False,比发个粗糙值更糟,而且不容易在日志里看出来)。
        """
        if not self.pre_offboard_estimate or self._u_from_nmpc \
                or self._pre_off_converged:
            return True
        if self._pre_off_first_frame is not None and (
                self.frames - self._pre_off_first_frame
                > self.pre_off_timeout_frames):
            self._pre_off_converged = True   # 锁存,避免每帧刷 WARN
            self.get_logger().warn(
                f'[cold-start] 超时未收敛 ('
                f'{self.pre_off_timeout_frames * mhe_p.dt:.0f}s), 放行当前 '
                f'm_est={self.m_est:.4f} kg 给 NMPC')
            return True
        return False

    def _seed_mass_from_thrust(self, why: str):
        """零先验的质量种子:悬停近似 m ≈ T_phys/g。

        T_phys=k_f·Σω²(电机转速反算)与质量先验、与 MHE 自身状态都完全解耦
        —— 这是整条链上唯一不含先验的质量观测量,理由见 mhe_params.seed_from_thrust。
        拿不到电机数据或推力过低(没在飞)时回退 m_nominal 并 WARN:宁可退回先验,
        也不用一个不成立的近似。

        返回 (m_seed, from_thrust)。"""
        # no_mass_prior(冷启动)档下**绝不回退 m_nominal** —— 那恰恰是要去掉的
        # 那个先验。兜底改用箱约束中点(不含任何 m_B 信息),并且忽略
        # seed_from_thrust 开关:冷启动档下 T_phys/g 是唯一可用的零先验起点,
        # 而 T_phys 是**测量**不是先验。
        # ⚠️ 语义:这一档里种子只进 x_guess(SQP 起点)和 x0_bar,而 x0_bar 的
        #    质量维权重已降到 1e-6 —— 它不进代价函数,**不影响最优解**,只影响
        #    收敛迭代数。initial guess ≠ prior,论文措辞必须分清这两个词。
        if mhe_p.no_mass_prior:
            fallback = 0.5 * (mhe_p.m_min + mhe_p.m_max)
            fallback_why = f'箱约束中点 {fallback:.3f}'
            use_thrust = True
        else:
            fallback = float(mhe_p.m_nominal)
            fallback_why = f'm_nominal={fallback:.3f}'
            use_thrust = mhe_p.seed_from_thrust

        if not use_thrust:
            return fallback, False
        T = self.thrust_phys
        if T is None or not math.isfinite(T) or T < mhe_p.seed_thrust_min:
            self.get_logger().warn(
                f'mass seed ({why}): thrust_phys unavailable or too low '
                f'({"None" if T is None else f"{T:.2f}N"} < '
                f'{mhe_p.seed_thrust_min:.2f}N), falling back to '
                f'{fallback_why} kg')
            return fallback, False
        return float(np.clip(T / mhe_p.g, mhe_p.m_min, mhe_p.m_max)), True

    @staticmethod
    def _aug(y13, m_seed):
        """把 13 维量测拼成 MHE 的增广状态初值。s/dJ 一律从 0 起——
        它是"载荷的转动特征",没证据之前默认没有载荷。"""
        return np.concatenate([y13, [m_seed], np.zeros(mhe_p.ns)])

    @staticmethod
    def _geometry_from_m(m_t, r_p, m_b=None):
        """由**总质量** m_t 和载荷几何偏移 r_p 算 (dJ_rp均值标量, c_xy)——与
        mhe_model.py 耦合档内部那套代数完全一致的 numpy 复制品,只用于打日志
        和真值对比(模型自己在符号层算,不吃这里的返回值)。"""
        m_b = mhe_p.m_B if m_b is None else m_b
        r_p = np.asarray(r_p, dtype=float)
        m_p = max(float(m_t) - m_b, 0.0)
        if m_t <= 0.0 or m_p <= 0.0:
            return 0.0, np.zeros(2)
        mu = m_b * m_p / m_t
        dJ = mu * (r_p[2] ** 2 + 0.5 * (r_p[0] ** 2 + r_p[1] ** 2))
        c = (m_p / m_t) * r_p[0:2]
        return float(dJ), c

    @staticmethod
    def _inertia_from_m(m_t, r_p, m_b=None):
        """完整 3x3 惯量矩阵 J(m_t) = J_B + mu*(|r|^2 I - r r^T)(含非对角项),
        与耦合档模型内部一致。用于 J_true vs J_hat 对比。"""
        m_b = mhe_p.m_B if m_b is None else m_b
        r_p = np.asarray(r_p, dtype=float)
        J = np.diag([mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz])
        m_p = max(float(m_t) - m_b, 0.0)
        if m_p <= 0.0 or m_t <= 0.0:
            return J
        mu = m_b * m_p / m_t
        return J + mu * (float(r_p @ r_p) * np.eye(3) - np.outer(r_p, r_p))

    def _payload_inputs_fresh(self, now=None):
        """Whether all physical/known-input streams are recent enough."""
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        stamps = (self._last_odom_rx_sec, self._last_motor_rx_sec,
                  self._last_u_rx_sec)
        return all(
            stamp is not None and 0.0 <= now - stamp <= self.payload_input_fresh_sec
            for stamp in stamps)

    def _payload_frame_health(self):
        """Return ``(healthy, solution_age)`` for the atomic output frame."""
        now = self.get_clock().now().nanoseconds * 1e-9
        inputs_fresh = self._payload_inputs_fresh(now)
        solution_age = (1.0e6 if self._last_solve_success_sec is None else
                        max(0.0, now - self._last_solve_success_sec))
        healthy = (self._last_solve_ok and inputs_fresh
                   and solution_age <= self.payload_solution_fresh_sec)
        return bool(healthy), float(solution_age)

    def _publish_payload_estimate(self):
        """Publish one coherent, continuous m/s/c/J/confidence estimate frame."""
        healthy, solution_age = self._payload_frame_health()
        if healthy:
            alpha = 1.0 - math.exp(-mhe_p.dt / self.payload_output_tau)
            mp_target = max(float(self.m_est) - mhe_p.m_B, 0.0)
            if self._s_release_latched:
                # 载荷已释放:目标零。_s_out 的 LPF 负责把残值连续带下去。
                s_target = np.zeros(2)
            elif mhe_p.ns:
                s_target = np.asarray(self.s_est, dtype=float)
            else:
                # Legacy fallback only; the default moment model always takes
                # this first branch.
                s_target = max(float(self.m_est), 1e-6) * np.asarray(
                    self.c_xy_est, dtype=float)
            self._m_payload_out += alpha * (mp_target - self._m_payload_out)
            self._s_out += alpha * (s_target - self._s_out)
        m_out = mhe_p.m_B + self._m_payload_out
        c_xy, dJ_diag, J_diag = inertia_from_mass_moment(
            m_out, self._s_out, mhe_p.m_B,
            [mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz], mhe_p.rz_prior,
            payload_ki=mhe_p.payload_ki)
        if healthy:
            conf_target = no_payload_confidence(
                self._m_payload_out, self._s_out, dJ_diag,
                **self._payload_conf_args)
            self._no_payload_confidence += alpha * (
                conf_target - self._no_payload_confidence)
        estimate = PayloadEstimate(
            m_total=m_out, s_xy=self._s_out.copy(), c_xy=c_xy,
            dJ_diag=dJ_diag, J_diag=J_diag,
            no_payload_confidence=self._no_payload_confidence,
            healthy=healthy, solution_age_sec=solution_age)
        self.payload_estimate_pub.publish(
            Float64MultiArray(data=estimate.as_array().tolist()))
        if self._s_release_latched and self._s_decay_n < self.s_decay_log_frames:
            self._s_decay_n += 1
            s_est_n = (float(np.linalg.norm(self.s_est)) if mhe_p.ns else
                       float(np.linalg.norm(np.asarray(self.c_xy_est)
                                            * max(float(self.m_est), 1e-6))))
            s_out_n = float(np.linalg.norm(self._s_out))
            self.get_logger().info(
                f'[s-decay] n={self._s_decay_n} |s_est|={s_est_n:.5f} '
                f'|s_target|=0.00000 |s_out|={s_out_n:.5f} '
                f'd_s_out={s_out_n - float(np.linalg.norm(self._s_out_prev)):+.5f} '
                f'conf={self._no_payload_confidence:.3f} '
                f'present={int(self._payload_present)} '
                f'armed={int(self._load_armed)} '
                f'latched={int(self._s_release_latched)} '
                f'health={int(bool(healthy))}')
        self._s_out_prev = self._s_out.copy()

    def _apply_forgetting(self):
        """逐 stage 按年龄给 R/Q 打折(见 mhe_params.forgetting_lambda)。
        stage j 的年龄 = N-1-j(j=0 最老),权重乘 λ^年龄。stage 0 额外带到达
        代价块,**该块不打折**——它锚的是物理状态,不是要遗忘的老量测。
        权重只在 λ 变化时重设一次(_forget_set 标志),不必每帧 cost_set。"""
        if self._forget_set:
            return
        lam = mhe_p.forgetting_lambda
        N = mhe_p.N
        w_nom = block_diag(mhe_p.R, mhe_p.Q)
        self.solver.cost_set(
            0, 'W', block_diag(lam ** (N - 1) * mhe_p.R,
                               lam ** (N - 1) * mhe_p.Q, mhe_p.Q0))
        for j in range(1, N):
            self.solver.cost_set(j, 'W', (lam ** (N - 1 - j)) * w_nom)
        self._forget_set = True
        self.get_logger().info(
            f'[forget] 遗忘因子 λ={lam:.3f} 已应用:stage 权重 λ^(N-1-j),'
            f'最老 stage 缩放 {lam ** (N - 1):.2e},有效记忆 ≈ '
            f'{1.0 / (1.0 - lam):.1f} 帧 ({1.0 / (1.0 - lam) * mhe_p.dt:.2f}s);'
            f'到达代价块不打折。事件触发已停用。')

    def _model_r_p(self):
        """模型几何槽里该放的 r_p —— **这是模型能看到的全部载荷信息**。
        'event':drop 事件一到就清零(模型被告知载荷没了,历史行为)。
        'self' :永不被告知 drop,保持最后一次 attach 的杆臂;载荷贡献靠
                m_P=(m−m_B)⁺ 随质量估计自行熄灭(见 __init__ 里该参数注释)。
        两种模式在 attach **之前**都是零(那时确实还没有任何杆臂信息)。"""
        if not self.external_event_inputs and mhe_p.estimate_moment:
            # s_xy carries all lateral geometry.  Only the rack's weak vertical
            # arm prior is needed for inertia, and it is present continuously;
            # neither attach nor drop changes this parameter slot.
            return np.array([0.0, 0.0, mhe_p.rz_prior])
        if self.geom_release_mode == 'self':
            return (np.zeros(3) if self._r_p_last is None
                    else np.asarray(self._r_p_last, dtype=float))
        if self._payload_attached and self.attach_offset is not None:
            return np.asarray(self.attach_offset, dtype=float)
        return np.zeros(3)

    def _truth_row(self):
        """estimate vs ground truth 的一行(纯评估,见 eval_true_payload_mass 注释)。
        m_true(t) 是分段常数:载荷在机上 = m_B+m_P_true,否则 = m_B。c_true/J_true
        用**同一个** attach 几何 r_p 和真值质量算,所以 c/J 的差异纯粹来自质量估计
        误差,不掺几何误差——要单独看几何误差就对比 c_xy_est(τ_phys 反算)那一路。"""
        mp_true = self.eval_true_payload_mass
        if mp_true <= 0.0:
            return None
        attached = bool(self._payload_attached)
        m_true = mhe_p.m_B + (mp_true if attached else 0.0)
        r_p = (self.attach_offset if (attached and self.attach_offset is not None)
               else np.zeros(3))
        dJ_h, c_h = self._geometry_from_m(self.m_est, r_p)
        dJ_t, c_t = self._geometry_from_m(m_true, r_p)
        J_h = self._inertia_from_m(self.m_est, r_p)
        J_t = self._inertia_from_m(m_true, r_p)
        return {
            'm_true': m_true,
            'cx_hat': c_h[0], 'cy_hat': c_h[1],
            'cx_true': c_t[0], 'cy_true': c_t[1],
            'dJ_hat': dJ_h, 'dJ_true': dJ_t,
            'Jxx_hat': J_h[0, 0], 'Jxx_true': J_t[0, 0],
            'Jyz_hat': J_h[1, 2], 'Jyz_true': J_t[1, 2],
        }

    def _on_mass_event(self, source: str):
        # 逐帧日志窗口不看开关:对照组(event_trigger_enable=false)也要有
        # 10Hz 分辨率的 m_est 记录,否则量不出收敛时间、没法对比
        self._log_until_frame = self.frames + 2 * mhe_p.N + \
            self.confirm_timeout_frames
        if not self.event_enabled:
            self.get_logger().info(
                f'mass event [{source}] received (trigger disabled, '
                'logging only)')
            return
        if self.signal_mode == 'residual':
            # 无信号消融:外部事件只用来对齐诊断日志窗口(为了有 10Hz 数据可
            # 解析对比),**绝不拿它武装检测器**——武装/确认全交给 _residual_detect
            # 从 T_phys 残差自触发,这才是"无信号"的语义。
            self.get_logger().info(
                f'mass event [{source}] received (no-signal ablation: '
                'external signal ignored for triggering, logging only; '
                'detection deferred to T_phys residual)')
            return
        # 两段式触发第一段:武装 T_phys 突变检测器,记下当前基线
        self._armed_frame = self.frames
        self._armed_baseline = self.thrust_phys
        self.get_logger().info(
            f'mass event [{source}] received at frame {self.frames}: '
            f'armed, waiting for T_phys confirmation '
            f'(baseline={self._armed_baseline if self._armed_baseline is not None else float("nan"):.2f}N, '
            f'thresh={self.confirm_thresh:.1f}N)')

    def _check_confirmation(self):
        """两段式触发第二段:武装状态下检查 T_phys 是否已偏离基线超阈值
        (物理真正生效的时刻),确认后才通知调度器开始降权;超时兜底触发。
        每帧入缓冲后调用(self.frames 已递增,最新帧序号 = frames-1)。"""
        if self._armed_frame is None:
            return
        if self._armed_baseline is None:
            # 武装时 T_phys 还没来过,拿到第一帧就补基线
            self._armed_baseline = self.thrust_phys
            return
        confirmed = (self.thrust_phys is not None and
                     abs(self.thrust_phys - self._armed_baseline)
                     > self.confirm_thresh)
        timed_out = (self.frames - self._armed_frame
                     > self.confirm_timeout_frames)
        if not (confirmed or timed_out):
            return
        # 刚入缓冲的这一帧已带新物理(confirmed)或按事件时刻兜底(timed_out)
        first_post = self.frames - 1 if confirmed else self._armed_frame
        self.scheduler.notify_event(first_post)
        self.get_logger().info(
            'mass event {} at frame {}: de-weighting pre-event stages'.format(
                'confirmed by T_phys '
                f'({self.thrust_phys:.2f}N vs baseline '
                f'{self._armed_baseline:.2f}N)' if confirmed
                else 'confirmation timed out, falling back to event frame',
                first_post))
        self._armed_frame = None
        self._armed_baseline = None

    def _update_moment_reference(self, s_vec, frame_ok):
        """建立并**冻结**带载一阶矩基准 s_ref_loaded(2026-09-05)。

        为什么不能再用 running max:ratio 的分母原来是 max_history(|s|),而
        figure-8 段的 s 估计会**系统性越过物理上限** —— 15 轮实测 7.6%~18.3% 的
        带载帧 |s| > m_P,max·r_xy,max = 0.30x0.13 = 0.039 kg·m(最大 0.052,
        反解 r_xy 达 0.35m,远超 attach 上限 0.13m)。那些不是几何,是动态失真。
        一旦进了分母,带载稳态的 ratio 就被压到阈值以下,moment_collapsed 会在
        载荷还挂着时成立 —— 与 fastA 栽掉的结构同型。
        (⚠️ 这个缺陷在 ddfe9d2 之前就存在;A 修复只是让 detector 更早开始跑,
         把越界率从 7.6% 抬到 18.3%,并非根因。)

        状态机:
            LOADED(自主) -> healthy/fresh -> **自主识别**的低动态窗口
            -> 剔除物理越界样本 -> 稳健统计 -> 冻结 -> 进入机动后不再更新

        低动态窗口是**在线判据**(角速度/平移速度 + 估计健康度),不是
        "attach 后等 N 秒" —— 后者只是当前脚本时序下的巧合区间。
        """
        if self._moment_ref_ready or not frame_ok:
            return
        s_norm = float(np.linalg.norm(s_vec))
        # ① 物理越界样本:直接**剔除**,不裁剪后使用(裁剪仍是拿错误样本构造分母)
        if s_norm > self.moment_abs_max:
            self._moment_ref_reject += 1
            self._moment_ref_buf.clear()
            return
        # ② 低动态窗口:自主判据,与 _mass_observable 同源但阈值独立
        if self.x_meas is None:
            return
        om = float(np.linalg.norm(self.x_meas[10:13]))
        v_xy = float(np.linalg.norm(self.x_meas[3:5]))
        if om > self.moment_ref_omega_max or v_xy > self.moment_ref_vel_max:
            self._moment_ref_buf.clear()      # 机动打断窗口,重新攒
            return
        self._moment_ref_buf.append(np.asarray(s_vec, dtype=float).copy())
        if len(self._moment_ref_buf) < self.moment_ref_window:
            return
        buf = np.asarray(self._moment_ref_buf[-self.moment_ref_window:])
        norms = np.linalg.norm(buf, axis=1)
        med = float(np.median(norms))
        if med <= 1e-6:
            self._moment_ref_buf.clear()
            return
        # ③ 离散度:窗口内幅值要稳
        mad = float(np.median(np.abs(norms - med)))
        if (1.4826 * mad / med) > self.moment_ref_max_cv:
            self._moment_ref_buf.clear()
            return
        # ④ 方向一致性:防"大小正常但方向乱跳"
        unit = buf / np.maximum(norms[:, None], 1e-9)
        mean_dir = unit.mean(axis=0)
        if float(np.linalg.norm(mean_dir)) < self.moment_ref_dir_min:
            self._moment_ref_buf.clear()
            return
        # ⑤ 稳健统计建立并**冻结**(截尾均值:去掉两端各 20% 后取均值)
        k = max(1, int(round(0.2 * len(norms))))
        trimmed = float(np.mean(np.sort(norms)[k:len(norms) - k])) if len(norms) > 2 * k else med
        self._s_ref_loaded = trimmed
        self._moment_ref_ready = True
        self.get_logger().info(
            f'[moment-ref] 基准已建立并冻结: |s_ref|={trimmed:.5f} kg·m '
            f'(窗口 {self.moment_ref_window} 帧, 中位 {med:.5f}, '
            f'离散 {1.4826*mad/max(med,1e-9):.2f}, 方向一致度 '
            f'{float(np.linalg.norm(mean_dir)):.2f}, 越界剔除 '
            f'{self._moment_ref_reject} 帧) — 此后不再更新')

    def _update_release_residual(self):
        """独立的 release-residual 投票(见 __init__ 里 release_resid_* 注释)。

        每拍调用。只产出**带时效的票**,从不直接释放 —— 释放一律由
        payload_estimate.release_decision 汇总三个信息源后决定。
        """
        if not self.release_resid_enable or self.thrust_phys is None:
            return
        h = self.release_resid_half
        self._rr_hist.append(float(self.thrust_phys))
        if len(self._rr_hist) > 2 * h:
            self._rr_hist.pop(0)
        if len(self._rr_hist) < 2 * h:
            return
        pre = sum(self._rr_hist[:h]) / h
        post = sum(self._rr_hist[h:]) / h
        d = post - pre
        # 滚动 sigma 只用**过去**的 ΔT(不含当前帧),否则阶跃自己会把 sigma 抬起来
        sigma = None
        if len(self._rr_d_hist) >= 8:
            med = float(np.median(self._rr_d_hist))
            mad = float(np.median([abs(x - med) for x in self._rr_d_hist]))
            sigma = max(1.4826 * mad, 0.05)
        self._rr_d_hist.append(d)
        if len(self._rr_d_hist) > 30:
            self._rr_d_hist.pop(0)
        z_ok = True
        if self.release_resid_z > 0.0 and sigma is not None:
            z_ok = (-d / sigma) > self.release_resid_z
        now = self.get_clock().now().nanoseconds * 1e-9
        if d < -self.release_resid_floor and z_ok:
            self._rr_n += 1
            if self._rr_n >= self.release_resid_persist:
                self._residual_drop_evidence_until = (
                    now + self.residual_evidence_hold_sec)
        else:
            self._rr_n = 0
        if d < -self.release_resid_strong_floor and z_ok:
            self._rr_strong_n += 1
            if self._rr_strong_n >= self.release_resid_strong_persist:
                if now >= self._residual_strong_evidence_until:
                    self.get_logger().info(
                        f'[release-resid] STRONG 票: ΔT={d:+.2f}N '
                        f'x{self._rr_strong_n}帧 (floor '
                        f'{self.release_resid_strong_floor}N)')
                self._residual_strong_evidence_until = (
                    now + self.residual_evidence_hold_sec)
        else:
            self._rr_strong_n = 0

    def _residual_detect(self):
        """无信号消融的自触发检测器(event_signal_mode='residual')。
        不使用任何外部事件武装,纯从 T_phys 相对慢基线(EMA 低通)的残差做
        两段式确认:①瞬时残差越过 confirm_thresh = 越阈(第一段),②连续
        resid_persist 帧仍越阈 = 确认(第二段,抗阵风脉冲——阵风残差是脉冲/
        宽带,质量突变是台阶)。确认后用"首次越阈帧"作为第一个事件后测量帧
        通知调度器,与外部有信号版走同一条降权链路。每帧入缓冲后调用。"""
        T = self.thrust_phys
        if T is None:
            return
        a = self.resid_ema_alpha
        if self.scheduler.event_frame is not None:
            # 已在事件过渡期内,调度器接管;基线/预热全复位,过渡结束后由下面的
            # None 分支重新播种到**新稳态** T_phys 并重新预热——否则过渡结束基线
            # 还停在阶跃前旧值,会把已经永久的台阶反复当成新事件无限重触发。
            self._resid_baseline = None
            self._resid_pending = 0
            self._resid_settled = False
            self._resid_stable = 0
            # 阶跃判据同样复位:跨事件比较两个短窗没有意义,而且过渡期的 T_phys
            # 本来就在阶跃中间,留着会在过渡结束那一帧诈出一个假事件。
            self._step_pending = 0
            self._tphys_hist.clear()
            return

        # ===== 并行阶跃判据(见 __init__ 里 resid_step_enable 注释)=====
        # 放在慢基线判据**之前**:后者在未预热时会直接 return,阶跃判据必须先跑,
        # 否则一旦预热受阻就永远轮不到它 —— 而"预热受阻时仍能检出"正是它的用途。
        self._tphys_hist.append(T)
        _n2 = 2 * self.resid_step_half
        if len(self._tphys_hist) > _n2:
            self._tphys_hist.pop(0)
        if self.resid_step_enable and len(self._tphys_hist) == _n2:
            h = self.resid_step_half
            _pre = sum(self._tphys_hist[:h]) / h
            _post = sum(self._tphys_hist[h:]) / h
            _d = _post - _pre
            if abs(_d) > self.resid_step_thresh:
                self._step_pending += 1
                if self._step_pending >= self.resid_step_persist:
                    # 事件帧取两窗交界(阶跃真正发生的位置),而不是当前帧
                    _f = self.frames - 1 - h
                    self.scheduler.notify_event(_f)
                    _dropped = _d < 0.0
                    self.get_logger().info(
                        f'[step-detect] 阶跃自检测 at frame {_f}: '
                        f'Δ={_d:+.2f}N over {2*h*mhe_p.dt:.1f}s '
                        f'(阈值 {self.resid_step_thresh}N, persist='
                        f'{self.resid_step_persist}), direction='
                        f'{"DROP" if _dropped else "ATTACH"}; de-weighting')
                    # 释放几何用**更高**的门槛:轨迹切换与质量突变在推力通道上
                    # 不可分,误降权只是慢一点,误释放几何会让模型丢掉还挂着的载荷。
                    if (_dropped and self.resid_release_geom
                            and self._load_armed):
                        # 2026-09-05:并行阶跃判据同样只投票,不再自己释放。
                        # 它原来绕过了统一的 release_decision —— 即便默认关闭,
                        # 留着就是"禁止单通道释放"规则上的一个缺口。
                        _now = self.get_clock().now().nanoseconds * 1e-9
                        self._residual_drop_evidence_until = (
                            _now + self.residual_evidence_hold_sec)
                        self.get_logger().info(
                            '[step-detect] → 投出释放票(不直接释放)')
                        self.get_logger().info(
                            f'[step-detect] → 自主释放载荷几何 '
                            f'(|Δ|={-_d:.2f}N ≥ '
                            f'{self.resid_step_release_thresh}N)')
                    elif _dropped:
                        self.get_logger().info(
                            f'[step-detect] 下降型事件但 |Δ|={-_d:.2f}N < '
                            f'{self.resid_step_release_thresh}N,只降权不释放几何'
                            '(可能是轨迹切换而非卸载)')
                    self._step_pending = 0
                    self._tphys_hist.clear()
                    return
            else:
                self._step_pending = 0

        if self._resid_baseline is None:
            self._resid_baseline = T
            self._resid_stable = 0
            return
        dev = abs(T - self._resid_baseline)
        if not self._resid_settled:
            # 预热门控:连续 resid_warmup 帧 T_phys 贴基线(<settle_tol)才判定
            # 基线可信、开放检测。暂态摆动(>settle_tol)会清零计数、推迟预热。
            self._resid_baseline = (1.0 - a) * self._resid_baseline + a * T
            if dev < self.resid_settle_tol:
                self._resid_stable += 1
                if self._resid_stable >= self.resid_warmup:
                    self._resid_settled = True
                    self.get_logger().info(
                        '[no-signal] baseline settled at '
                        f'{self._resid_baseline:.2f}N '
                        f'(quiet {self.resid_warmup} frames); detector armed')
            else:
                self._resid_stable = 0
            return
        # 已预热:两段式确认
        if dev > self.confirm_thresh:
            if self._resid_pending == 0:
                self._resid_cross_frame = self.frames - 1  # 首次越阈帧序号
            self._resid_pending += 1
            if self._resid_pending >= self.resid_persist:
                self.scheduler.notify_event(self._resid_cross_frame)
                # 事件方向:T_phys 掉下去 = 载荷没了(drop);涨上来 = 上货(attach)。
                # dev 本身是 abs(),方向要单独取。
                dropped = (T < self._resid_baseline)
                self.get_logger().info(
                    '[no-signal] mass mutation self-detected at frame '
                    f'{self._resid_cross_frame}: T_phys={T:.2f}N vs baseline '
                    f'{self._resid_baseline:.2f}N (dev {dev:.2f}N > '
                    f'{self.confirm_thresh:.1f}N over {self.resid_persist} '
                    f'frames, direction={"DROP" if dropped else "ATTACH"}); '
                    'de-weighting pre-event stages')
                # --- 自主释放几何(2026-08-25)---
                # 原本只有 mass_event_cb(外部信号)会释放"幽灵几何";残差自检测
                # 这条路只降权、不释放,于是一旦把 drop 事件关掉,MHE 就会永远
                # 挂着已经不存在的载荷几何,继续用它给 om_dot 算 dJ/c_xy。
                # ⚠️ geom_release_mode='self'(靠 m_est 让几何自行熄灭)在 legacy
                #    档是被明确拒绝的(几何幅值来自先验/棘轮,不随质量熄灭),所以
                #    legacy 档下要去掉外部 drop 信号,**只能**走这条"自检测到
                #    推力下降 → 自己释放几何"的路。
                # 释放动作与 mass_event_cb 完全一致(2026-08-30 起三条路径共用
                # _release_payload:丢 attach 几何 + c_xy_est 归零),差别只在触发源:
                # 那边是别人告诉它,这边是自己看出来的。**不动 m_est** —— 同
                # mass_event_cb 的理由:让估计器自己连续收敛回空机,才验证得了
                # "模型正确后它能回归"。
                if (dropped and self.resid_release_geom
                        and self._load_armed):
                    # 2026-09-05:残差不再**单独**释放,而是作为三信息源里的
                    # "动态证据"投进 release_decision(见 payload_estimate 的
                    # 模块注释)。单通道裸奔正是上一版的问题:残差档下它一个人
                    # 说了算,默认档下它又完全失声,两种极端都不好。
                    # 证据带时效:残差是**边沿**事件,过期就不该再算数。
                    self._residual_drop_evidence_until = (
                        self.get_clock().now().nanoseconds * 1e-9
                        + self.residual_evidence_hold_sec)
                    self.get_logger().info(
                        f'[no-signal] 残差检出 DROP 方向 → 作为释放证据保持 '
                        f'{self.residual_evidence_hold_sec:.1f}s')
                elif not dropped:
                    # 检出的是 ATTACH 方向:快速通道进 LOADED,不必等 m_p 攒够
                    # 持续帧 —— 残差看见的承重比质量收敛早得多。
                    self._update_payload_presence('attach')
                self._resid_pending = 0
            return
        # 未越阈(或阵风脉冲已回落):清持续计数,慢基线继续低通跟踪长期缓漂
        self._resid_pending = 0
        self._resid_baseline = (1.0 - a) * self._resid_baseline + a * T

    def u_opt_cb(self, msg):
        u = np.array(msg.data)
        if u.shape[0] == mhe_p.nu_known and np.all(np.isfinite(u)):
            if not self._u_from_nmpc:
                self._u_from_nmpc = True
                if self.pre_offboard_estimate:
                    # 接管时刻:把冷启动估计的成绩单打出来,便于事后对齐
                    # "NMPC 拿到的初始质量"与"它自己硬编码的 p.m"。
                    self.get_logger().info(
                        f'[cold-start] NMPC 接管 (u_opt 首帧): '
                        f'm_est={self.m_est:.4f} kg, '
                        f'converged={self._pre_off_converged}')
            # 记 NMPC **原始**理想输出:必须在下面把 u[0] 换成 thrust_phys 之前,
            # 否则 command 流的推力列会变成 motor 流的量,两流不再独立。
            self.resid_log.log_command(u)
            # 推力分量(u[0])换成电机转速反算的真实物理推力(见 MOTOR_CONSTANT
            # 注释)。力矩:tau_source='command' 时沿用 NMPC 意图值(legacy 行为,
            # 那时转动通路不携带质量信息);'phys' 时 roll/pitch 也换成电机转速
            # 反算的 tau_phys —— 耦合档下这是必需的,理由见该参数注释。
            # 任一物理量还没到时退回 NMPC 值,避免丢帧。
            if self.thrust_phys is not None:
                u = u.copy()
                u[0] = self.thrust_phys
            if self.tau_source in ('phys', 'phys_full') \
                    and self.tau_phys is not None:
                u = u.copy()
                u[1] = float(self.tau_phys[0])   # roll
                u[2] = float(self.tau_phys[1])   # pitch
                if self.tau_source == 'phys_full':
                    # 完全解耦:yaw 也用电机反算的真实反扭矩。至此 u_known 四个
                    # 分量**没有一个**来自 NMPC —— MHE 只吃 allocation 的输出。
                    u[3] = float(self.tau_phys[2])
                # 'phys' 档:u[3] 仍是 NMPC 意图值(legacy,实测比真实大 3 倍)
            self.u_known = u
            self._last_u_rx_sec = self.get_clock().now().nanoseconds * 1e-9

    def _drain_motor_avg(self):
        """取出上一个 MHE 采样区间内电机推力的**平均**,并清空累加器。

        返回 (T_avg, tau_avg) 或 (None, None)(区间内没收到电机数据)。
        见 _motor_f_acc 处注释:250Hz -> 10Hz 必须用区间平均而非端点采样,
        否则就是 2026-08-24 记下的那个"准但混叠"。
        """
        if self._motor_n == 0:
            return None, None
        f = self._motor_f_acc / self._motor_n
        self._motor_f_acc = np.zeros(4)
        self._motor_n = 0
        T = float(np.sum(f))
        tau = np.array([
            TORQUE_SIGN * float(np.sum(ROTOR_Y * f)),
            TORQUE_SIGN * float(-np.sum(ROTOR_X * f)),
            YAW_TORQUE_SIGN * MOMENT_CONSTANT * float(np.sum(YAW_DIR * f)),
        ])
        return T, tau

    def timer_cb(self):
        # NMPC 接管姿态控制之前(还在用位置 setpoint 飞向起点)没有 u_opt,
        # 这段时间没有意义的输入,直接跳过,不往缓冲区塞假数据。
        if self.x_meas is None or self.u_known is None:
            return
        # Do not duplicate cached odometry/control into the MHE window.  Keep a
        # heartbeat alive so NMPC immediately sees ``healthy=0`` and holds the
        # last safe model instead of interpreting publication as freshness.
        now = self.get_clock().now().nanoseconds * 1e-9
        inputs_fresh = self._payload_inputs_fresh(now)
        if not inputs_fresh:
            if self._payload_inputs_were_fresh:
                # A temporal hole invalidates the fixed-dt window.  Rebuild it
                # entirely from fresh samples instead of solving across the gap.
                self.y_buf.clear()
                self.u_buf.clear()
                self.x0_bar = None
                self.x_guess = None
                self._last_solve_ok = False
                self.get_logger().warn(
                    '[payload-health] input stream stale; MHE window cleared, '
                    'publishing held estimate as unhealthy')
            self._payload_inputs_were_fresh = False
            if self._mass_publish_allowed():
                self._publish_payload_estimate()
            return
        self._payload_inputs_were_fresh = True
        # internal 流:MHE 自身状态,供离线诊断做分桶(空载/带载)与几何真值对照
        self.resid_log.log_internal(self.m_est, self._payload_attached,
                                    self.attach_offset, self.thrust_phys,
                                    truth=self._truth_row())

        # 抗混叠:把这一拍要入缓冲的 u_known 里"来自电机"的分量换成上一个
        # MHE 区间的平均值(而不是回调里最后一帧的瞬时值)。只动本来就取自
        # 电机的分量 —— NMPC 意图值那些原样保留,不改既有档位的语义。
        T_avg, tau_avg = self._drain_motor_avg()
        u_row = self.u_known.copy()
        if self.motor_window_avg and T_avg is not None:
            u_row[0] = T_avg                      # u[0] 恒为物理推力
            if self.tau_source in ('phys', 'phys_full'):
                u_row[1] = float(tau_avg[0])
                u_row[2] = float(tau_avg[1])
            if self.tau_source == 'phys_full':
                u_row[3] = float(tau_avg[2])

        self.y_buf.append(self.x_meas.copy())
        if len(self.y_buf) > mhe_p.N + 1:
            self.y_buf.pop(0)
        self.u_buf.append(u_row)
        if len(self.u_buf) > mhe_p.N:
            self.u_buf.pop(0)
        self.frames += 1
        # 先更新自主存在状态:下面的检测器和质量域判据都以它为门控,必须先于
        # 它们跑,否则本帧的释放会用上一帧的状态。
        self._update_payload_presence()
        if self.payload_lost_watch:
            self._payload_lost_watch()
        if self.event_enabled and self.signal_mode == 'residual':
            self._update_release_residual()
            self._residual_detect()
        else:
            self._check_confirmation()

        if self.c_xy_est_enable:
            self._update_c_xy_est()
        self._check_release_detector_watchdog()

        if len(self.y_buf) < mhe_p.N + 1:
            if self._mass_publish_allowed():
                self._publish_payload_estimate()
            return  # 窗口还没攒满

        self._solve_window()
        # Publish every estimator tick, including a held last-good state after a
        # failed solve.  Consumers can therefore use freshness independently of
        # solver status and never need an event fallback.
        if self._mass_publish_allowed():
            self._publish_payload_estimate()

    def _update_c_xy_est(self):
        """强闭环 c_xy 在线估计(B.3 Phase1,只记录不闭环)。稳态悬停时从电机
        转速反算的体力矩直接反解复合质心水平偏移,慢 EMA 滤噪。见 __init__ 里
        c_xy_est 的注释。c_xy=[cx,cy]=[-τ_pitch/T, τ_roll/T]。"""
        # 质量域卸载判据(见 __init__ 里 c_xy_mass_release_mp)。**必须放在最前**:
        # 下面的稳态门控在机动中直接 return,放它后面就永远轮不到 —— 而"机动中
        # 也要能熄灭"正是它存在的理由。
        if self.c_xy_mass_release_mp > 0.0 and self._load_armed:
            _m_p = float(self.m_est) - mhe_p.m_B
            if not self._c_xy_mass_armed:
                # 未武装:只等 m_p 爬过高水位,期间**不判释放**(attach 收敛期
                # m_p 天然偏小,判了必误触发 —— 第一版就是这么栽的)。
                if _m_p > self.c_xy_mass_release_mp * self.c_xy_mass_arm_ratio:
                    self._c_xy_mass_high += 1
                    if self._c_xy_mass_high >= self.c_xy_mass_arm_persist:
                        self._c_xy_mass_armed = True
                        self.get_logger().info(
                            f'[c_xy_est] 质量域判据已武装(m_p={_m_p:.3f}kg '
                            f'连续{self.c_xy_mass_arm_persist}帧高于水位)')
                else:
                    self._c_xy_mass_high = 0
                self._c_xy_mass_low = 0
            elif not self._mass_observable():
                self._c_xy_mass_low = 0   # 高动态:质量不可观测,不累计释放计数
            elif _m_p < self.c_xy_mass_release_mp:
                self._c_xy_mass_low += 1
                if self._c_xy_mass_low >= self.c_xy_mass_release_persist:
                    self._c_xy_mass_low = 0
                    self._c_xy_mass_high = 0
                    self._c_xy_mass_armed = False
                    self._update_payload_presence('drop')
                    self._release_payload(
                        f'mass-based m_p<{self.c_xy_mass_release_mp:.3f}kg'
                        f' x{self.c_xy_mass_release_persist}帧')
                    return
            else:
                self._c_xy_mass_low = 0
        # --- 统一释放判据(2026-09-05 三信息源)---
        # ★ 外层门控**去掉 _c_xy_mass_armed**,并整段移出质量域判据的 if 块:原来两个
        # 武装状态被串在了一起 —— 自主状态机的 _load_armed(残差 attach 或持续 m_p 证据)
        # 已经置位,却还要等旧的"质量持续过 0.09kg"路径再武装一次。20260905_162459 那轮
        # m_est 全程偏低(进 LOADED 时 m_p=-0.103,靠残差事件进的),_c_xy_mass_armed 从未
        # 置位 => [s-collapse] 一行都没有,统一 detector 整段没运行(被 unresolved 兜住)。
        # 旧的质量域释放分支保留它**自己的** _c_xy_mass_armed(那条路本就该等质量爬起来),
        # 但它不该继续当 moment/residual detector 的总开关 —— 两条路的证据来源不同。
        # 决策逻辑放在 payload_estimate.release_decision:在线节点与离线
        # 回放(scripts/gripper/replay_release_detector.py)共用同一份实现,
        # 免得"回放通过、上线不通过"。三个信息源与"禁止单通道释放"的理由
        # 见该模块注释。
        if (self.s_release_ratio > 0.0 and mhe_p.ns
                and self._load_armed):
            # 心跳记的是“统一判决代码确实进入”，不是最终是否放行。
            # 不健康/基准未就绪/越界都是正常的“运行中但拒绝判决”，
            # 不应被 watchdog 误报成 detector 没运行。
            self._release_detector_eval_frame = self.frames
            # ★★ health/freshness 门控(2026-09-05):估计不可信时**不判决**。
            # 根因链(20260905_152924):连续 5 次 solve failure -> re-anchor
            # 把 s_est 清成 0 -> 释放判据在下一次成功解之前照常运行 ->
            # ratio=0.000 被当成 moment_collapsed -> 恰好 m_p 也探负 ->
            # slow 误释放。而这条判据跑在 _solve_window **之前**,却完全没有
            # 用上节点已有的 health/freshness 状态。
            # 不健康时:立即清持续计数(不是暂停——跨盲区拼出来的持续是假的),
            # 且不允许释放;等下一次 fresh 且成功的解才恢复判决。
            # ⚠️ _s_peak **不清**:短暂求解失败不该丢掉本轮的带载基准,
            #    否则重新立峰会把判据的分母搞错。
            frame_ok, _sol_age = self._payload_frame_health()
            frame_ok = frame_ok and not self._s_reanchored
            if not frame_ok:
                self._s_low = 0
                self._s_fast = 0
                self._s_strong = 0
                if self._s_ratio_log_n % 10 == 1:
                    self.get_logger().warn(
                        f'[s-collapse] 估计不健康,释放判决暂停 '
                        f'(solve_ok={int(self._last_solve_ok)} '
                        f'reanchored={int(self._s_reanchored)} '
                        f'age={_sol_age:.2f}s) — 持续计数已清零')
                self._s_ratio_log_n += 1
                return
            s_norm = float(np.linalg.norm(self.s_est))
            self._s_peak = max(self._s_peak, s_norm)   # 仅诊断,不再进 ratio
            self._update_moment_reference(self.s_est, frame_ok)
            if s_norm > self.moment_abs_max:
                # 物理不可能值:不投 moment 票、清持续计数、只累计质量指标
                self._dyn_overrange += 1
                self._s_low = self._s_fast = self._s_strong = 0
                if self._s_ratio_log_n % 10 == 1:
                    self.get_logger().warn(
                        f'[s-collapse] |s|={s_norm:.5f} > 物理上限 '
                        f'{self.moment_abs_max:.3f} — 动态失真,不参与 moment '
                        f'投票(本轮累计 {self._dyn_overrange} 帧)')
                self._s_ratio_log_n += 1
                return
            if not self._moment_ref_ready:
                # 基准未建立 ⇒ 不判决(ratio 没有可信分母)。持续下去最终走
                # unresolved,**不**降低释放门槛。
                self._s_low = self._s_fast = self._s_strong = 0
                if self._s_ratio_log_n % 10 == 1:
                    self.get_logger().info(
                        f'[s-collapse] moment 基准未就绪(窗口 '
                        f'{len(self._moment_ref_buf)}/{self.moment_ref_window}, '
                        f'越界剔除 {self._moment_ref_reject}) — 暂不判决')
                self._s_ratio_log_n += 1
                return
            m_p_now = max(float(self.m_est) - mhe_p.m_B, 0.0)
            resid_ev = (self.get_clock().now().nanoseconds * 1e-9
                        < self._residual_drop_evidence_until)
            _now = self.get_clock().now().nanoseconds * 1e-9
            strong_ev = _now < self._residual_strong_evidence_until
            ev = release_evidence(
                m_p_now, s_norm, self._s_ref_loaded, resid_ev,
                ratio_thr=self.s_release_ratio,
                s_abs_max=self.s_release_abs_max,
                mass_release_mp=self.c_xy_mass_release_mp,
                residual_strong=strong_ev)
            (fire, why, self._s_low, self._s_fast,
             self._s_strong) = release_decision(
                ev, self._load_armed, self._s_low, self._s_fast,
                slow_persist=self.s_release_persist,
                fast_persist=self.s_release_fast_persist,
                strong_frames=self._s_strong,
                strong_persist=self.s_release_strong_persist)
            # 诊断:带载段的 ratio_min 与卸载后的 ratio 是给阈值定分界用的
            # 分布数据(2026-09-04 起逐轮记录)。
            self._s_ratio_log_n += 1
            if self._s_ratio_log_n % 10 == 1:      # 10Hz -> 每 1s 一行
                self.get_logger().info(
                    f'[s-collapse] |s|={s_norm:.5f} ref={self._s_ref_loaded:.5f} '
                    f'peak={self._s_peak:.5f} ovr={self._dyn_overrange} '
                    f'ratio={ev["ratio"]:.3f} (thr {self.s_release_ratio:.2f}) '
                    f'moment={int(ev["moment_collapsed"])} '
                    f'quantity={int(ev["quantity_empty"])} '
                    f'resid={int(ev["residual_drop"])}'
                    f'{"S" if ev["residual_strong"] else ""} '
                    f'health=1 age={_sol_age:.2f} '
                    f'solve_ok={int(self._last_solve_ok)} '
                    f'reanch={int(self._s_reanchored)} '
                    f'm_p={float(self.m_est) - mhe_p.m_B:+.4f} '
                    f'slow={self._s_low}/{self.s_release_persist}')
            if fire:
                self._update_payload_presence('drop')
                self._release_payload(why)
                return
        # --- 一阶质量矩驱动(见 __init__ 里 c_xy_from_moment 注释)---
        # 放在最前:这条路不依赖 tau_phys/稳态,s 本身就是窗口解算出来的。
        if self.c_xy_from_moment and mhe_p.ns:
            m_t = max(float(self.m_est), mhe_p.m_min)
            # ★ 2026-09-10:同一个 s_est,两条消费路径待遇必须一致。
            # 释放判据(_check_s_collapse)明确剔除 |s|>moment_abs_max 的帧,理由写着
            # "物理不可能值、动态失真,不参与 moment 投票",figure-8 下实测 7.6%~18.3%
            # 的帧越界。而这条路**原本零门控**:同样那批帧,除以 m_t 之后原样发给
            # NMPC 当 model.p 的 c_xy —— 于是 1/6 左右的帧在给控制器编造物理上不可能
            # 的偏心量,生成假的 τ=c×T 补偿力矩。而 roll/pitch 力矩本来就有 21%/47%
            # 的帧打在约束上(见 nmpc 的 [flight-diag] u_sat),没有余量消化这种噪声。
            # (讽刺的是被它取代的 tau_phys 路径反而有稳态门控+EMA,见下面 else 分支。)
            #
            # 处理:越界帧**保持上一拍**并照常 publish —— 与释放判据"不投票"同语义,
            # 且维持话题速率不变(下游不会因为掉帧触发任何超时/freshness 逻辑)。
            # 上限用 moment_abs_max(=payload_mass_envelope × payload_rxy_envelope,
            # 平台包线派生),是机架规格不是任务信息,不泄露载荷真值。
            s_vec = np.asarray(self.s_est, dtype=float)
            s_norm = float(np.linalg.norm(s_vec))
            if self.c_xy_clamp_enable and s_norm > self.moment_abs_max:
                self._c_xy_reject += 1
                if not self._c_xy_inited:
                    # 还没有任何合法值可保持 —— 不发,等第一个物理可能的帧。
                    if self._c_xy_reject % 20 == 1:
                        self.get_logger().warn(
                            f'[c_xy-clamp] |s|={s_norm:.5f} > 上限 '
                            f'{self.moment_abs_max:.4f} 且尚无合法初值 — 暂不发布 '
                            f'(累计剔除 {self._c_xy_reject} 帧)')
                    return
                if self._c_xy_reject % 20 == 1:
                    self.get_logger().warn(
                        f'[c_xy-clamp] |s|={s_norm:.5f} > 上限 '
                        f'{self.moment_abs_max:.4f} — 动态失真,保持上一拍 '
                        f'c_xy=[{self.c_xy_est[0]*100:+.2f},'
                        f'{self.c_xy_est[1]*100:+.2f}]cm '
                        f'(累计剔除 {self._c_xy_reject}/{self._c_xy_total} 帧)')
            else:
                self.c_xy_est = s_vec / m_t
                self._c_xy_inited = True
            self._c_xy_total += 1
            # 低频汇总:剔除率为 0 的轮次也要留痕,否则 grep 不到就分不清
            # "这轮很干净"和"这轮根本没跑到这条路"。300 帧 @10Hz = 每 30s 一行。
            if self._c_xy_total % 300 == 0:
                self.get_logger().info(
                    f'[c_xy-clamp] 越界剔除 {self._c_xy_reject}/{self._c_xy_total} '
                    f'({100.0 * self._c_xy_reject / self._c_xy_total:.1f}%) '
                    f'上限 {self.moment_abs_max:.4f} kg·m '
                    f'clamp={int(self.c_xy_clamp_enable)}')
            self.c_xy_est_pub.publish(
                Float64MultiArray(data=[float(self.c_xy_est[0]),
                                        float(self.c_xy_est[1])]))
            return
        if self.tau_phys is None or self.thrust_phys is None \
                or self.x_meas is None:
            return
        T = self.thrust_phys
        if T < 1.0:
            return  # 没有有效推力(未起飞/异常)
        om = self.x_meas[10:13]
        vel_xy = self.x_meas[3:5]
        # 稳态门控:只在悬停稳时采样,避开 descend/LIFT/drop 暂态力矩污染
        if (float(np.linalg.norm(om)) > self.c_xy_steady_omega or
                float(np.linalg.norm(vel_xy)) > self.c_xy_steady_vel):
            return
        c_inst = np.array([-self.tau_phys[1] / T, self.tau_phys[0] / T])
        if not self._c_xy_inited:
            self.c_xy_est = c_inst
            self._c_xy_inited = True
        else:
            a = self.c_xy_ema_alpha
            self.c_xy_est = (1.0 - a) * self.c_xy_est + a * c_inst
        self.c_xy_est_pub.publish(
            Float64MultiArray(data=[float(self.c_xy_est[0]),
                                    float(self.c_xy_est[1])]))

    def _maneuver_level(self):
        """当前机动度:角速度与水平速度各自相对阈值的倍数,取大者。
        <=1 = 悬停/准静态;>1 = 机动中。口径与 _update_c_xy_est 的稳态门控一致。"""
        if self.x_meas is None:
            return 0.0
        om = float(np.linalg.norm(self.x_meas[10:13]))
        v = float(np.linalg.norm(self.x_meas[3:6]))
        return max(om / max(self.maneuver_omega_thresh, 1e-6),
                   v / max(self.maneuver_vel_thresh, 1e-6))

    def _apply_maneuver_gate(self, in_transition: bool):
        """机动期把 Q0 的质量维锚紧,悬停期恢复名义。

        gain = clip(lvl^exponent, 1, cap):悬停(lvl<=1)时恒为 1(名义权重,
        行为与关闭开关时逐位相同);机动越猛锚得越紧,封顶 cap。用连续函数而不是
        二值开关 —— 二值会在阈值附近抖动时反复切换权重,把一个数值问题变成一个
        控制问题。写入前量化到 1.25 倍的对数格,避免每帧都 cost_set。
        """
        self._mg_frames_total += 1
        if in_transition:
            return          # 事件过渡期让位(见 __init__ 注释)
        lvl = self._maneuver_level()
        gain = 1.0 if lvl <= 1.0 else min(lvl ** self.maneuver_exponent,
                                          self.maneuver_q0_cap)
        if gain > 1.0:
            self._mg_frames_gated += 1
        # 量化:落到 1.25^k 的格子上,减少无谓的 cost_set
        q = 1.0 if gain <= 1.0 else 1.25 ** round(math.log(gain, 1.25))
        if self._mg_gain_applied is not None and abs(q - self._mg_gain_applied) < 1e-9:
            return
        W0 = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0).copy()
        idx = mhe_p.nx + mhe_p.nw + mhe_p.nx        # Q0 块里质量那一维的全局下标
        W0[idx, idx] *= q
        self.solver.cost_set(0, 'W', W0)
        self._mg_gain_applied = q
        # 每次实际改档都留一行:没有这行就完全无法事后判断门控到底有没有起作用
        # (2026-08-25 首轮 SITL 就栽在这:统计变量加了却没打印,两轮飞完拿不出
        # 任何门控生效的证据)。改档次数被量化压得很少,不会刷屏。
        self.get_logger().info(
            f'[maneuver-gate] lvl={lvl:.2f} → Q0[m] ×{q:.3g} '
            f'(gated {self._mg_frames_gated}/{self._mg_frames_total} 帧, '
            f'{100.0 * self._mg_frames_gated / max(self._mg_frames_total, 1):.0f}%)')

    def _solve_window(self):
        N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw
        y_win = self.y_buf
        u_win = self.u_buf

        if self.x0_bar is None:
            # 到达代价先验均值的质量维:用零先验的 T_phys/g,不用空机标称值。
            m_seed, from_thrust = self._seed_mass_from_thrust('initial window')
            self.x0_bar = self._aug(y_win[0], m_seed)
            self.x_guess = [self._aug(y_win[min(i, N)], m_seed)
                             for i in range(N + 1)]
            self.m_est = m_seed
            if mhe_p.no_mass_prior:
                # 冷启动档:这不是 "prior",Q0 质量维已经是 1e-6。
                self.get_logger().info(
                    f'[cold-start] initial guess (NOT a prior): '
                    f'm={m_seed:.3f} kg '
                    f'({"T_phys/g (measurement)" if from_thrust else "box midpoint"})'
                    f' | Q0_m={mhe_p.Q0[mhe_p.nx, mhe_p.nx]:.1e}, '
                    f'lbx=[{mhe_p.m_min:.3f}, {mhe_p.m_max:.3f}]')
            else:
                self.get_logger().info(
                    f'arrival prior seeded: m={m_seed:.3f} kg '
                    f'({"T_phys/g, no prior" if from_thrust else "m_nominal prior"})')

        # 已知几何 [dJ, cx, cy]:用当前 m_est 和 attach 实测偏移算(见
        # _payload_geometry),窗口内所有 stage 共用同一个当前值——geometry 是
        # 常量、只有 m_p_hat 随 m_est 缓慢变,不需要按帧存历史,跟 T/tau 的
        # 逐帧历史值语义不同。没有 attach(wrench 场景/attach 前)则为全零。
        # 门控用物理状态 _payload_attached(不是 attach_offset is None——后者
        # 保留最后几何、表达不了"货已卸";见 __init__ 注释)。
        if mhe_p.geom_coupled or mhe_p.estimate_moment:
            # 耦合档:geom 槛位装**载荷几何偏移 r_p**,dJ/c 由模型内部按被估质量
            # 现算(见 mhe_model.py)。这里不再需要任何 m_p_hat ——
            # grip_payload_envelope 在这一档**不参与**(历史上的先验/地板/棘轮
            # 本来就是"缺了 ∂(J,c)/∂m 这条导数通路"的补丁,用来稳住那条外层
            # 不动点迭代;它们已于 2026-08-26 全部删除)。
            geom = self._model_r_p()
            r_p_log = geom            # 日志/几何存在性判据一律看原始杆臂
            # ⚠️ 仅供日志/诊断,**不是模型用的值**:模型内部按每次求解的被估质量
            # 现算 J(m)/c(m);这里是拿"上一次的 m_est"代入同一套代数,好让日志有个
            # 可读的当量。attach 那一帧 m_est 还≈m_B,所以会显示 dJ=0/c=0——那不
            # 代表几何没生效,只代表此刻的质量估计还没涨上来。
            dJ, c_xy = self._geometry_from_m(self.m_est, geom)
            if mhe_p.estimate_moment and mhe_p.moment_a_mode == 'frozen':
                # A=μr_z² 用**上一窗口的 m̂** 现算(见 mhe_params.moment_a_mode)。
                # geom[0] 在 moment 档是空闲槽:模型丢弃了 O(r_xy²) 项,用不到 r_x。
                # 借它传 m̂ → 窗口内 ∂A/∂m≡0(优化器不能拿 m 当惯量旋钮),但 A 的
                # 数值每帧跟着质量刷新,不必被包线钉死。
                geom = np.asarray(geom, dtype=float).copy()
                geom[0] = float(self.m_est)
                # ⚠️ 借槽之后 geom[0] 恒非零,不能再拿它判"几何在不在"(会恒 True),
                # 日志与 _geom_on 都改看原始杆臂。
                r_p_log = self._model_r_p()
        else:
            if self._payload_attached and self.attach_offset is not None:
                dJ, c_xy = self._payload_geometry(self.attach_offset)
            else:
                dJ, c_xy = 0.0, np.zeros(2)
            geom = np.array([dJ, c_xy[0], c_xy[1]])
            r_p_log = geom
        # 几何归零可验证性(B.5 验收要"drop 后一个 MHE 周期内 dJ/c_xy 归零"):
        # 状态翻转时打一条,便于从日志直接判定释放时刻。
        _geom_on = (bool(np.any(np.asarray(r_p_log) != 0.0)) if mhe_p.geom_coupled
                    else self._payload_attached)
        if getattr(self, '_geom_active_prev', None) != _geom_on:
            self.get_logger().info(
                f'[geom] model_geom_on={_geom_on} '
                f'(release_mode={self.geom_release_mode}, '
                f'truth_attached={self._payload_attached}) '
                f'mode={"coupled(r_p)" if mhe_p.geom_coupled else "legacy(dJ,c)"} '
                f'-> geom={np.array2string(np.asarray(r_p_log), precision=4)} '
                f'dJ={dJ:.4f} c_xy=[{c_xy[0]:+.4f},{c_xy[1]:+.4f}]'
                + (' (dJ/c 为按当前 m_est 折算的诊断当量,模型内部随 m 实时生成)'
                   if mhe_p.geom_coupled else ''))
            self._geom_active_prev = _geom_on

        yref_0 = np.concatenate([y_win[0], np.zeros(nw), self.x0_bar])
        self.solver.set(0, 'yref', yref_0)
        self.solver.set(0, 'p', np.concatenate([u_win[0], geom]))
        self.solver.set(0, 'x', self.x_guess[0])

        for j in range(1, N):
            yref = np.concatenate([y_win[j], np.zeros(nw)])
            self.solver.set(j, 'yref', yref)
            self.solver.set(j, 'p', np.concatenate([u_win[j], geom]))
            self.solver.set(j, 'x', self.x_guess[j])

        self.solver.set(N, 'x', self.x_guess[N])

        # 权重调度:两条互斥的路径
        #  ①事件触发降权(M0,默认):事件过渡期内把事件前 stage 降权,过渡期结束
        #    由调度器自己恢复名义权重(机制见 mhe_event_weights.py)
        #  ②遗忘因子(λ<1):逐 stage 按年龄打折,**无事件**,老数据永远算得轻
        # 二者不叠加——叠加的话过渡期里两套缩放会相乘,归因说不清(见记忆里
        # 反复出现的"混算两种效应压低统计功效"教训)。互斥在 __init__ 里强制。
        if self._forget_active:
            self._apply_forgetting()
            in_transition = False
        else:
            in_transition = self.scheduler.apply(self.solver, self.frames - 1)
        if in_transition != self._in_transition:
            self.get_logger().info(
                'event transition {}'.format(
                    'started: pre-event stages de-weighted'
                    if in_transition else 'ended: nominal weights restored'))
            self._in_transition = in_transition

        # 机动门控(见 __init__ 注释):事件过渡期让位,避免两套 stage-0 W 互相覆盖
        if self.maneuver_gate_enable:
            self._apply_maneuver_gate(in_transition)

        # 实时性仪表:只测 solver.solve() 本身的墙钟耗时,用于论文 timing 表。
        # 与 acados_nmpc_node 里的 solve_time 同口径(墙钟、单位 ms),便于对表。
        _t_solve0 = time.time()
        status = self.solver.solve()
        self._solve_ms.append((time.time() - _t_solve0) * 1000.0)
        if status != 0:
            self._last_solve_ok = False
            self._fail_streak += 1
            self.get_logger().warn(
                f'MHE solve failed [status={status}], skipping this window '
                f'(streak {self._fail_streak})')
            if self._fail_streak >= 5:
                # 重锚:先验/初值全部对齐到当前窗口的量测。质量维种子曾经直接
                # 重用 self.m_est——但 2026-07-07 坏几何复测发现这会把问题锁死:
                # 暴烈暂态(LIFT/attach)期间,失败streak开始前的最后一次"成功"
                # 解本身就可能已经错(status=0 不代表暂态下的解可信,实测冻结在
                # 1.638kg,比空机 2.06kg 还轻),重锚把这个错值当种子喂回去,
                # solver 在错误先验附近找到一个自洽但错误的局部解,此后 solve
                # 不再失败(看起来"已恢复")但 m_est 再也不动——错误被永久锁定。
                # 改用电机转速反算的 thrust_phys(独立于 MHE 自身状态,不会被
                # 同一次错误污染)做悬停近似 m≈T/g 当种子,给 solver 一个物理
                # 站得住脚的重新出发点,而不是延续可能已经错的旧估计。
                m_seed, from_thrust = self._seed_mass_from_thrust('re-anchor')
                if not from_thrust:
                    # 拿不到 thrust_phys 时,这里比初始化更保守:延续当前估计
                    # 而不是跳回 m_nominal —— 连败往往发生在飞行中段,那时
                    # m_est 即使可疑也比空机标称值离真值近。
                    m_seed = self.m_est
                # 重锚时 s 也归零:连败往往伴随几何/姿态失配,把上一次可能已错的
                # 质量矩一起带过去只会延续错误(与 m 用 thrust_phys 重种同理)。
                self.x0_bar = self._aug(y_win[0], m_seed)
                self.x_guess = [
                    self._aug(y_win[min(i, N)], m_seed)
                    for i in range(N + 1)]
                self.s_est = np.zeros(mhe_p.ns)
                # ★ 2026-09-05:这个 0 是**初始化值**,不是"一阶矩塌了"。
                # 它曾经直接喂进释放判据的 moment 通道(ratio=0.000),再撞上
                # m_p 探负 => slow 误释放。标记它,由 health 门控挡住,直到下一次
                # 成功解产生真实的 s。
                self._s_reanchored = True
                self.get_logger().warn(
                    f're-anchored arrival prior to current window after '
                    f'{self._fail_streak} consecutive failures '
                    f'(m seeded from thrust_phys: {self.m_est:.3f} -> '
                    f'{m_seed:.3f} kg)')
                self.m_est = m_seed
                self._fail_streak = 0
            return
        x_sol = [self.solver.get(i, 'x') for i in range(N + 1)]
        if not all(np.all(np.isfinite(x)) for x in x_sol):
            self._last_solve_ok = False
            self._fail_streak += 1
            self.get_logger().warn(
                'MHE returned non-finite state with success status; '
                f'holding last payload estimate (streak {self._fail_streak})')
            return
        self.m_est = float(x_sol[N][nx])
        if mhe_p.ns:
            self.s_est = np.array(x_sol[N][nx + mhe_p.nm:nx + mhe_p.nm + mhe_p.ns],
                                  dtype=float)
        self._last_solve_ok = True
        self._last_solve_success_sec = (
            self.get_clock().now().nanoseconds * 1e-9)
        self._fail_streak = 0
        self._s_reanchored = False      # 拿到真实 s 了,释放判决可以恢复

        # 事件后逐帧打日志(平时 2s 一条太粗,量不出亚秒级收敛)。窗口由
        # _on_mass_event 设定、与调度开关解耦——对照组(触发关)也照打,
        # 否则没有 10Hz 分辨率数据可对比。SITL 对比实验就靠这段日志。
        if self.frames <= self._log_until_frame:
            t_phys = (self.thrust_phys
                      if self.thrust_phys is not None else float('nan'))
            tag = 'transition' if self._in_transition else 'post-event'
            self.get_logger().info(
                f'[{tag}] m_est={self.m_est:.3f} kg '
                f'(T_phys={t_phys:.2f}N)')

        # 滑动窗口 shift:下一窗口的到达代价先验取这一次窗口里 x[1] 的估计值,
        # 跟 acados_nmpc_node 里 NMPC 的 warm-start shift 是同一套思路。
        self.x0_bar = x_sol[1].copy()
        self.x_guess = [x_sol[min(i + 1, N)].copy() for i in range(N + 1)]

        self._update_pre_offboard_convergence()
        if self._mass_publish_allowed():
            self.mass_pub.publish(Float64(data=self.m_est))
        if self.tau_phys is not None:
            self.tau_phys_pub.publish(
                Float64MultiArray(data=[float(v) for v in self.tau_phys]))

        self.counter += 1
        if self.counter % 20 == 0:
            t_phys = self.thrust_phys if self.thrust_phys is not None else float('nan')
            # 与 NMPC 那条 solve=X.Xms 同格式,batch 收尾直接 grep 出 timing 表。
            _s = self._solve_ms[-20:]
            self.get_logger().info(
                f'MHE mass estimate: {self.m_est:.3f} kg (T_phys={t_phys:.2f}N) '
                f'| solve={sum(_s)/len(_s):.1f}ms (max{max(_s):.1f})')
            # estimate vs ground truth 对照行(评估专用真值,见 _truth_row)。
            # grep '[truth]' 就能直接出 m/c/J 三组的时间序列。
            tr = self._truth_row()
            if tr is not None:
                mp_t = (self.eval_true_payload_mass
                        if self._payload_attached else 0.0)
                r_ev = (self.attach_offset if self.attach_offset is not None
                        else np.zeros(3))
                self.get_logger().info(
                    f"[truth] m_hat={self.m_est:.4f} m_true={tr['m_true']:.4f} "
                    f"err={100*(self.m_est-tr['m_true'])/tr['m_true']:+.2f}% | "
                    f"c_hat=[{tr['cx_hat']:+.4f},{tr['cy_hat']:+.4f}] "
                    f"c_true=[{tr['cx_true']:+.4f},{tr['cy_true']:+.4f}] | "
                    f"Jxx_hat={tr['Jxx_hat']:.5f} Jxx_true={tr['Jxx_true']:.5f} "
                    f"Jyz_hat={tr['Jyz_hat']:+.5f} Jyz_true={tr['Jyz_true']:+.5f}"
                    + (f" | s_hat=[{self.s_est[0]:+.4f},{self.s_est[1]:+.4f}] "
                       f"s_true=[{mp_t*r_ev[0]:+.4f},{mp_t*r_ev[1]:+.4f}] kg·m"
                       if mhe_p.estimate_moment else ''))
            # B.3 Phase0 验证行:反算力矩 vs 质心模型预测(需 attach 真值对表)。
            # 稳态偏心悬停 τ_roll 应≈ m_p·g·ry、τ_pitch≈ -m_p·g·rx(复现 07-03)。
            if self.tau_phys is not None:
                tr, tp = self.tau_phys[0], self.tau_phys[1]
                pred = ''
                if self.attach_offset is not None:
                    m_p = max(self.m_est - mhe_p.m_nominal, 0.0)
                    ry, rx = self.attach_offset[1], self.attach_offset[0]
                    pred = (f' | model pred: roll={m_p*mhe_p.g*ry:+.3f} '
                            f'pitch={-m_p*mhe_p.g*rx:+.3f} '
                            f'(m_p={m_p:.3f} r=[{rx:+.3f},{ry:+.3f}])')
                self.get_logger().info(
                    f'[tau_phys] roll={tr:+.3f} pitch={tp:+.3f} Nm{pred}')
            # ---- 机动低估诊断(2026-08-28)----
            # 08-28 三轮实测把问题收窄到:T_phys/g 精确等于真质量(0.04%),而 m_est
            # 低 1.4~2.7% —— 推力测量没问题,是 MHE 拿着对的推力估低了。
            # 悬停/准稳态下垂直平衡给出 m = T·cosθ/g(θ=推力轴倾角)。打出这三个量
            # 就能判:m_est ≈ T·cosθ/g ⇒ MHE 内部自洽,偏差来自 T_phys 的乘性标定;
            # m_est <  T·cosθ/g ⇒ MHE 里还有别的项在吃掉它(kd/滞后/权重)。
            # cosθ 由体 z 轴在世界 z 上的投影给出 = R33 = 1-2(qx²+qy²)。
            if self.thrust_phys is not None and self.x_meas is not None:
                _q = self.x_meas[6:10]
                _n = float(np.linalg.norm(_q))
                if _n > 1e-6:
                    _qw, _qx, _qy, _qz = (_q / _n)
                    _c = 1.0 - 2.0 * (_qx * _qx + _qy * _qy)   # R33 = cos(tilt)
                    _c = float(np.clip(_c, 1e-3, 1.0))
                    _mq = self.thrust_phys * _c / mhe_p.g       # 准静态质量
                    _vz = float(self.x_meas[5])
                    # Ground/pre-arm diagnostics can legitimately have T=0.
                    # Never let a log-only percentage divide by zero take down
                    # the estimator node.
                    _bias_pct = relative_error_percent(self.m_est, _mq)
                    self.get_logger().info(
                        f'[mass-diag] m_est={self.m_est:.4f} T={self.thrust_phys:.3f}N '
                        f'tilt={math.degrees(math.acos(_c)):.2f}deg cos={_c:.5f} '
                        f'| T/g={self.thrust_phys / mhe_p.g:.4f} '
                        f'T*cos/g={_mq:.4f} '
                        f'| m_est-T*cos/g={self.m_est - _mq:+.4f}kg '
                        f'({_bias_pct:+.2f}%) vz={_vz:+.3f}')
            # B.3 Phase1 验证行:在线 c_xy 估计 vs attach 真值反推的 c_xy。质量
            # 因子用 eval_true_payload_mass(**纯评估**真值,不通往模型),否则退回
            # m_est 反推 —— 后者只是个标签近似,不是真值(见记忆 cxy-truth-label-defect:
            # 论文图 4 与演示视频都已按真值重算)。2026-08-26 前这里优先读的是
            # grip_true_payload_mass,那个开关会把真值灌进模型,已随去先验改造删除。
            if self.c_xy_est_enable and self._c_xy_inited:
                ref = ''
                if self.attach_offset is not None:
                    m_p = (self.eval_true_payload_mass
                           if self.eval_true_payload_mass > 0.0
                           else max(self.m_est - mhe_p.m_nominal, 0.0))
                    m_t = mhe_p.m_nominal + m_p
                    if m_t > 0.0:
                        c_true = (m_p / m_t) * self.attach_offset[0:2]
                        ref = (f' | truth c=[{c_true[0]:+.4f},{c_true[1]:+.4f}] '
                               f'(m_p={m_p:.3f})')
                self.get_logger().info(
                    f'[c_xy_est] cx={self.c_xy_est[0]:+.4f} '
                    f'cy={self.c_xy_est[1]:+.4f} m{ref}')


def main():
    rclpy.init()
    node = MHENode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 批次脚本每轮结束用 kill -9,SIGKILL 捕不到 → 采集器已按 200 行分批落盘,
        # 最坏只丢尾部不足一批的行(诊断可接受)。正常退出路径这里补全。
        node.resid_log.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
