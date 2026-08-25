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


class MHENode(Node):
    def __init__(self):
        super().__init__('mhe_node')
        self.get_logger().info(
            'MHE node starting, building/loading solver... '
            f'(geom_coupled={mhe_p.geom_coupled}, N={mhe_p.N}, '
            f'm_min={mhe_p.m_min:.3f}, m_B={mhe_p.m_B:.4f})')
        self.solver = ensure_mhe_ocp_solver()

        self.x_meas = None
        self.u_known = None
        self.thrust_phys = None  # 电机转速反算的真实总推力(见 MOTOR_CONSTANT 注释)
        self.tau_phys = None     # 电机转速反算的真实体力矩 [roll,pitch,yaw](B.3 Phase0)
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
        # m_est 卡在 1.2-1.4kg,比修复前更差)。改成棘轮:只记录 attach 以来
        # 见过的最高 m_p_hat,某次暂态读数偏低不会让已经建立的几何修正撤销。
        self._m_p_hat_ratchet = 0.0
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

        # 诊断参数(2026-07-08,吊挂 CEM 首轮 0.15kg 结构性失败排查——两个明显
        # 不同的 θ 都 4/4 失败,怀疑是"用 self.m_est 反推 m_p_hat 再算 dJ/c_xy,
        # dJ/c_xy 又反过来影响 self.m_est"这个自举耦合在低质量(dJ/c_xy 信号
        # 本就比 0.3kg 弱)下失稳)。默认 0.0 = 原行为(棘轮反推);>0 时
        # _payload_geometry 直接拿这个真值算 dJ/c_xy,完全脱钩 self.m_est——
        # 只留"估质量",摘掉"用质量反推几何缩放"这一环。若固定后 0.15kg 能
        # 稳定收敛,坐实自举耦合是根因;若依旧失败,根因在别处(比如 T_phys
        # 信噪比本身在轻载下就不够)。仅用于诊断,不是"已知几何"阶段的最终
        # 设计——质量仍应由 MHE 估,不该整体从外部喂真值。
        self.declare_parameter('grip_true_payload_mass', 0.0)
        self.grip_true_payload_mass = float(
            self.get_parameter('grip_true_payload_mass').value)

        # --- 完全解耦(2026-07-19):载荷几何大小由**独立操作先验**决定 ---
        # >0 时 m_p_hat 直接取该值,**完全不读 m_est**(绕过反推/棘轮/floor),
        # 于是 dJ 和 c_xy 同时脱离自举耦合——两者都只由 m_p_hat 驱动,这是唯一瓶颈。
        # 与 grip_true_payload_mass 机制相同但**语义不同**:那个是真值(诊断专用,
        # 真机拿不到),这个是"包裹标称质量"这类真机可用的弱先验。
        # 证据(实验见记忆 mhe-geom-mest-decoupling):先验故意错 -33%(0.2 对真实
        # 0.3)、floor 关闭、n=8 → 下垂塌陷 0/8,m_est 仍收敛到 2.357(真值 2.364,
        # 误差 0.3%)。即**几何只需量级大致对以避免模型结构性失配,精确质量由 MHE
        # 独立负责**;合并统计几何解耦 15/15 vs 耦合 10/15,Fisher p=0.021。
        # 默认 0.0=关闭(退回 floor 路径,保持既有默认行为);MHE 是通用节点、
        # 不同场景载荷不同,不硬编码先验,由启动脚本按场景标称值传入。
        self.declare_parameter('grip_geom_mp_prior', 0.0)
        self.grip_geom_mp_prior = float(
            self.get_parameter('grip_geom_mp_prior').value)

        # 方案C 永久地板(2026-07-09,打破 m_est<->dJ/c_xy 自举耦合但**不喂
        # 真值**):grip_true_payload_mass 诊断坐实根因是自举正反馈——0.15kg
        # 陷入鸡生蛋(m_est 要收敛需 dJ/c_xy 够大,dJ/c_xy 够大需 m_est 先
        # 上去,轻载信号弱冲不破)。修法:给反推的 m_p_hat 一个**永久下界地板**
        # ——m_p_hat = max(棘轮自适应值, grip_geom_mp_floor)。物理正当:地板取
        # "包裹质量操作下界"这个真机可用的弱先验(不是精确真值),对 <=下界 的
        # 载荷几何恒按下界算(≈诊断的恒定正确几何),对 >下界 的载荷自适应爬过
        # 地板用真实值。**关键:地板不衰减、不交回**——2026-07-09 实测衰减版
        # (线性 w:0->1 撤除)在 ecc=0.10 脆:mp=0.3 过度补偿崩到 m_min,mp=0.15
        # 撤太早趴空机;换永久地板后 ecc=0.05/0.10 均干净收敛到真值(<0.3%)。
        # 2026-07-17 默认从 0.0(关闭)改为 0.15(开启)。原先关着是为了"保已验证
        # 结果 + 留 A/B 对照",但 A/B 已经做完,而且证明**关着它有 25% 的任务失败
        # 率**:gripper attach 0.3kg/ecc0.05 跑 n=16(on 8 + off 8),4 轮里 m_est
        # 在 de-weight 后没能冲过 m_nominal(2.064)→ m_p_hat 恒为 0 → dJ/c_xy 永不
        # 上线 → 锁死在 1.44~2.03kg 不自纠(T_phys=23.1N 明示 2.36kg 也没用),NMPC
        # 消费错值后稳态下垂 0.24~0.61m、抬不到目标高度(不坠机,但任务失败)。
        # 开 floor=0.15 后 n=8 全部精确收敛 2.360、塌陷 0/8(机制上死锁路径被物理
        # 排除;统计上 0/8 vs 3/8 的 Fisher p=0.10 尚未达显著,要论文级需每组 n≈25)。
        # 原以为这只是 0.15kg 轻载的鸡生蛋,实测 0.3kg 一样中,只是变成概率事件。
        self.declare_parameter('grip_geom_mp_floor', 0.15)
        self.grip_geom_mp_floor = float(
            self.get_parameter('grip_geom_mp_floor').value)

        # --- 评估专用真值(2026-08-24)---
        # 载荷真实质量,**只**用于往日志/internal 流里写 m_true / c_true / J_true
        # 这几列做 estimate-vs-truth 对比。与 grip_true_payload_mass 的区别是**它
        # 在代码里没有任何通往模型的路径**:不进 _payload_geometry、不进 geom、
        # 不进 m_est、不进任何 solver.set。grip_true_payload_mass 那个是诊断用的
        # "把真值灌进模型"开关(记忆 cxy-truth-label-defect 明确要求生产实验别开),
        # 两者必须分开命名,否则将来很容易误用。0.0 = 没有真值,真值列写 nan。
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
        #   **前提:'self' 必须配合 tau_source='phys'**(把 u_known 的力矩也换成电机
        #   转速反算值),否则环断不掉。下面已强制该组合。
        # ⚠️ 只有 coupled 档才成立:legacy 档的几何幅值来自外部先验/棘轮,不随
        #   质量熄灭,'self' 会让 drop 后永远挂着幽灵载荷 → 直接拒绝该组合。
        # 载荷质量先验的**生效状态**必须在日志里讲清楚:脚本里照样会传
        # grip_geom_mp_prior/floor(为了 legacy 批次能复现),但 coupled 档下
        # _payload_geometry 根本不被调用 → 它们是死代码。不打这条,后来人看到
        # 启动参数里有 0.3 会误以为"方法需要预先知道载荷多重"。
        if mhe_p.geom_coupled:
            self.get_logger().info(
                '[prior] geom_coupled=True → 载荷质量先验全部**不生效**'
                f'(grip_geom_mp_prior={self.grip_geom_mp_prior}, '
                f'floor={self.grip_geom_mp_floor}, '
                f'true={self.grip_true_payload_mass} 均为死代码);'
                f'模型只吃可测杆臂 r_p + 被估质量,m 的先验仅有 Q0 质量维'
                f'={float(mhe_p.Q0[mhe_p.nx, mhe_p.nx]):.3g} '
                f'(σ={1.0/float(mhe_p.Q0[mhe_p.nx, mhe_p.nx])**0.5:.2f}kg,实质无先验)')
        else:
            self.get_logger().warn(
                '[prior] geom_coupled=False → 几何幅值来自**载荷质量先验**'
                f'(prior={self.grip_geom_mp_prior}, floor={self.grip_geom_mp_floor})')

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
        self.declare_parameter('mhe_tau_source', 'command')
        self.tau_source = str(
            self.get_parameter('mhe_tau_source').value).lower()
        if self.tau_source not in ('command', 'phys'):
            self.tau_source = 'command'

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
        # 算好传 event_confirm_thresh_n;部署(无真值)必须换成**操作先验** mass。
        # confirm_thresh_alpha>=0 时:confirm_thresh = α·g·confirm_payload_prior
        # (与 nmpc_node grip_payload_prior 同类的操作先验,非测量真值);<0=禁用,
        # 退回固定 event_confirm_thresh_n(现有行为)。⚠️先验≠实际质量时阈值会偏
        # (先验偏大→小载荷 T_phys 达不到阈值→超时兜底退回事件帧,graceful),
        # 单载荷部署 prior=预期载荷时无偏。
        self.declare_parameter('confirm_thresh_alpha', -1.0)
        self.declare_parameter('confirm_payload_prior', 0.3)
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
            m_prior = float(self.get_parameter('confirm_payload_prior').value)
            self.confirm_thresh = c_alpha * mhe_p.g * m_prior
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
        self.declare_parameter('c_xy_est_enable', False)   # 默认关(不扰其它实验);B.3 实验显式开
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
        self.c_xy_est = np.zeros(2)   # [cx, cy] 估计
        self._c_xy_inited = False

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
            # 刚体常数:算 J·omega_dot + omega x J·omega 要用
            'Jxx': mhe_p.Jxx, 'Jyy': mhe_p.Jyy, 'Jzz': mhe_p.Jzz,
            'm_nominal': mhe_p.m_nominal, 'g': mhe_p.g,
            'mhe_N': mhe_p.N, 'mhe_dt': mhe_p.dt,
            # 真值条件(诊断分桶/留出全靠它)
            'grip_true_payload_mass': self.grip_true_payload_mass,
            'geom_coupled': int(bool(mhe_p.geom_coupled)),
            'eval_true_payload_mass': self.eval_true_payload_mass,
            'motor_speed_topic': motor_topic,
            # 语义声明:后处理不能靠猜
            'odom_frame': 'twist.linear/angular = body FLU (原始值,未转 world)',
            'tau_phys_def': 'tau=TORQUE_SIGN*sum(ROTOR_{Y,-X}*k_f*w^2), 执行器输出力矩,不含 J*omega_dot',
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

        self.timer = self.create_timer(mhe_p.dt, self.timer_cb)
        self.get_logger().info(
            f'MHE node initialized! tau_source={self.tau_source} '
            f'(roll/pitch {"电机转速反算" if self.tau_source == "phys" else "NMPC意图值"}, '
            f'yaw 恒为 NMPC 意图值), geom_release_mode={self.geom_release_mode}. '
            'Waiting for odometry + control data...')

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
                tau_roll = TORQUE_SIGN * float(np.sum(ROTOR_Y * f))
                tau_pitch = TORQUE_SIGN * float(-np.sum(ROTOR_X * f))
                self.tau_phys = np.array([tau_roll, tau_pitch, 0.0])
                self.resid_log.log_motor(msg, w, self.tau_phys,
                                         self.thrust_phys)

    def mass_event_cb(self, msg):
        """质量突变事件 = **卸载**(wrench drop / gripper drop——nmpc_node 释放
        夹爪时同帧发这条)。真实载荷已不存在,必须立刻释放"幽灵几何",否则
        _payload_geometry 会继续按带载棘轮给 om_dot 算 dJ/c_xy(见 __init__ 里
        _payload_attached 注释)。**只释放几何,不动 m_est**——质量状态由 MHE
        自己从当前值连续收敛回空机,这样才验证得了"模型正确后估计器能回归"。
        wrench 场景本来就没几何(attach_offset 恒 None),这里是无害的 no-op。"""
        self._payload_attached = False
        self._m_p_hat_ratchet = 0.0
        self.attach_offset = None
        self._on_mass_event('drop')

    def attach_event_cb(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.shape[0] == 3 and np.all(np.isfinite(data)):
            self.attach_offset = data
            self._r_p_last = data.copy()   # 'self' 释放模式下模型只认它
            self._m_p_hat_ratchet = 0.0  # 新 attach:棘轮重新从 0 起(见 __init__ 注释)
            self._payload_attached = True   # 载荷上机,开放几何修正
        self._on_mass_event('attach (gripper)')

    def _payload_geometry(self, r_p):
        """由载荷相对机体的几何偏移 r_p=[rx,ry,rz](rz<0)算 (dJ, c_xy)。跟
        acados_nmpc_node.py 的同名方法完全一致(平行轴定理+复合质心),区别
        是这里没有 m_p/m_t 真值。m_p_hat 有三条来源,优先级从高到低:
          ①grip_true_payload_mass>0:真值(诊断专用,真机拿不到)
          ②grip_geom_mp_prior>0:**独立操作先验,完全不读 m_est(推荐的生产路径,
            2026-07-19 完全解耦改造)**
          ③否则:m_est 反推+棘轮+floor(旧的耦合路径,floor 只护低估方向)
        ③之所以危险:它构成了"self.m_est 反推 m_p_hat 再算 dJ/c_xy、dJ/c_xy 又反过来影响 self.m_est"
        直接拿瞬时 self.m_est,棘轮(只增不减)的理由见 __init__ 里
        _m_p_hat_ratchet 的注释:瞬时值在 LIFT 暂态失败期间可能跌破
        m_nominal,把修正读成 0,恰好在最需要它的时候自己失效。

        ⚠️ 已知局限(2026-07-07 讨论,未实测但架构上成立的对称风险):棘轮
        只解决"低估坍缩到 0"这一个方向,治不了对称的另一个方向——如果某次
        暂态 m_est 向上超调,棘轮会把这个虚高值永久锁住,之后 dJ/c_xy 就按
        虚高质量算,长期偏大。6 次 ry=0.05 实测只出现过低估,从未出现过超调,
        所以暂不上更重的架构(低通/迟滞/attach 时锁定几何配置独立于 m_est)
        ——证据不对称,不该为没观测到的故障方向预先做重设计。这里只加一个
        廉价兜底:棘轮上限钳在 m_max-m_nominal(物理上"载荷不可能比这更重"
        的边界),防止极端超调把 dJ/c_xy 锁到离谱的值,不改变已验证的行为。

        诊断分支(grip_true_payload_mass>0):跳过棘轮,直接用真值——彻底摘掉
        "self.m_est 反推 m_p_hat 再算 dJ/c_xy、dJ/c_xy 又反过来影响 self.m_est"
        这个自举耦合,只诊断用,见 __init__ 里该参数的注释。

        ⚠️ 物理状态门控(2026-07-15):载荷不在机上就直接返回零几何,**不看棘轮
        也不看 attach_offset**——棘轮只增不减、attach_offset 保留最后一次几何,
        两者都不能表达"货已经卸了"。漏了这道门 = drop 后按幽灵载荷算 dJ/c_xy,
        质量估计被系统性拖低(见 __init__ 里 _payload_attached 注释)。
        """
        if not self._payload_attached:
            return 0.0, np.zeros(2)
        if self.grip_true_payload_mass > 0.0:
            m_p_hat = self.grip_true_payload_mass
        elif self.grip_geom_mp_prior > 0.0:
            # 完全解耦路径:几何大小 = 独立操作先验,**不读 m_est**。
            # 这一支同时切断 m_est→dJ 和 m_est→c_xy 两条自举链(共用 m_p_hat 瓶颈),
            # 且不像 floor 那样只护住低估方向——向上超调被棘轮永久锁死的对称风险
            # 也一并消除(棘轮在这条路径上根本不参与)。见 __init__ 里该参数注释。
            m_p_hat = self.grip_geom_mp_prior
        else:
            m_p_hat = max(self.m_est - mhe_p.m_nominal, 0.0)
            self._m_p_hat_ratchet = max(self._m_p_hat_ratchet, m_p_hat)
            self._m_p_hat_ratchet = min(
                self._m_p_hat_ratchet, mhe_p.m_max - mhe_p.m_nominal)
            m_p_hat = self._m_p_hat_ratchet
            # 方案C:永久下界地板(见 __init__ 注释),打破 m_est<->dJ/c_xy
            # 自举鸡生蛋。不衰减、不交回。
            if self.grip_geom_mp_floor > 0.0:
                m_p_hat = max(m_p_hat, self.grip_geom_mp_floor)
        m_t = mhe_p.m_nominal + m_p_hat
        mu = mhe_p.m_nominal * m_p_hat / m_t if m_t > 0.0 else 0.0
        dJ = mu * (r_p[2] ** 2 + 0.5 * (r_p[0] ** 2 + r_p[1] ** 2))
        c = (m_p_hat / m_t) * np.asarray(r_p[0:2], dtype=float) if m_t > 0.0 \
            else np.zeros(2)
        return float(dJ), c

    def _seed_mass_from_thrust(self, why: str):
        """零先验的质量种子:悬停近似 m ≈ T_phys/g。

        T_phys=k_f·Σω²(电机转速反算)与质量先验、与 MHE 自身状态都完全解耦
        —— 这是整条链上唯一不含先验的质量观测量,理由见 mhe_params.seed_from_thrust。
        拿不到电机数据或推力过低(没在飞)时回退 m_nominal 并 WARN:宁可退回先验,
        也不用一个不成立的近似。

        返回 (m_seed, from_thrust)。"""
        if not mhe_p.seed_from_thrust:
            return float(mhe_p.m_nominal), False
        T = self.thrust_phys
        if T is None or not math.isfinite(T) or T < mhe_p.seed_thrust_min:
            self.get_logger().warn(
                f'mass seed ({why}): thrust_phys unavailable or too low '
                f'({"None" if T is None else f"{T:.2f}N"} < '
                f'{mhe_p.seed_thrust_min:.2f}N), falling back to '
                f'm_nominal={mhe_p.m_nominal:.3f} kg')
            return float(mhe_p.m_nominal), False
        return float(np.clip(T / mhe_p.g, mhe_p.m_min, mhe_p.m_max)), True

    @staticmethod
    def _aug(y13, m_seed):
        """把 13 维量测拼成 MHE 的增广状态初值(14 或 16 维)。s 一律从 0 起——
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
            return
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
                self.get_logger().info(
                    '[no-signal] mass mutation self-detected at frame '
                    f'{self._resid_cross_frame}: T_phys={T:.2f}N vs baseline '
                    f'{self._resid_baseline:.2f}N (dev {dev:.2f}N > '
                    f'{self.confirm_thresh:.1f}N over {self.resid_persist} '
                    'frames); de-weighting pre-event stages')
                self._resid_pending = 0
            return
        # 未越阈(或阵风脉冲已回落):清持续计数,慢基线继续低通跟踪长期缓漂
        self._resid_pending = 0
        self._resid_baseline = (1.0 - a) * self._resid_baseline + a * T

    def u_opt_cb(self, msg):
        u = np.array(msg.data)
        if u.shape[0] == mhe_p.nu_known and np.all(np.isfinite(u)):
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
            if self.tau_source == 'phys' and self.tau_phys is not None:
                u = u.copy()
                u[1] = float(self.tau_phys[0])   # roll
                u[2] = float(self.tau_phys[1])   # pitch
                # u[3](yaw)保持 NMPC 意图值:tau_phys 的 yaw 是 0 占位,不是实测
            self.u_known = u

    def timer_cb(self):
        # NMPC 接管姿态控制之前(还在用位置 setpoint 飞向起点)没有 u_opt,
        # 这段时间没有意义的输入,直接跳过,不往缓冲区塞假数据。
        if self.x_meas is None or self.u_known is None:
            return
        # internal 流:MHE 自身状态,供离线诊断做分桶(空载/带载)与几何真值对照
        self.resid_log.log_internal(self.m_est, self._payload_attached,
                                    self.attach_offset, self.thrust_phys,
                                    truth=self._truth_row())

        self.y_buf.append(self.x_meas.copy())
        if len(self.y_buf) > mhe_p.N + 1:
            self.y_buf.pop(0)
        self.u_buf.append(self.u_known.copy())
        if len(self.u_buf) > mhe_p.N:
            self.u_buf.pop(0)
        self.frames += 1
        if self.event_enabled and self.signal_mode == 'residual':
            self._residual_detect()
        else:
            self._check_confirmation()

        if self.c_xy_est_enable:
            self._update_c_xy_est()

        if len(self.y_buf) < mhe_p.N + 1:
            return  # 窗口还没攒满

        self._solve_window()

    def _update_c_xy_est(self):
        """强闭环 c_xy 在线估计(B.3 Phase1,只记录不闭环)。稳态悬停时从电机
        转速反算的体力矩直接反解复合质心水平偏移,慢 EMA 滤噪。见 __init__ 里
        c_xy_est 的注释。c_xy=[cx,cy]=[-τ_pitch/T, τ_roll/T]。"""
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
            self.get_logger().info(
                f'arrival prior seeded: m={m_seed:.3f} kg '
                f'({"T_phys/g, no prior" if from_thrust else "m_nominal prior"})')

        # 已知几何 [dJ, cx, cy]:用当前 m_est 和 attach 实测偏移算(见
        # _payload_geometry),窗口内所有 stage 共用同一个当前值——geometry 是
        # 常量、只有 m_p_hat 随 m_est 缓慢变,不需要按帧存历史,跟 T/tau 的
        # 逐帧历史值语义不同。没有 attach(wrench 场景/attach 前)则为全零。
        # 门控用物理状态 _payload_attached(不是 attach_offset is None——后者
        # 保留最后几何、表达不了"货已卸";见 __init__ 注释)。
        if mhe_p.geom_coupled:
            # 耦合档:geom 槛位装**载荷几何偏移 r_p**,dJ/c 由模型内部按被估质量
            # 现算(见 mhe_model.py)。这里不再需要任何 m_p_hat ——
            # grip_geom_mp_prior / grip_geom_mp_floor / 棘轮 / grip_true_payload_mass
            # 在这一档**全部不参与**(它们本来就是"缺了 ∂(J,c)/∂m 这条导数通路"
            # 的补丁:先验、地板、棘轮都是为了稳住那条外层不动点迭代)。
            geom = self._model_r_p()
            # ⚠️ 仅供日志/诊断,**不是模型用的值**:模型内部按每次求解的被估质量
            # 现算 J(m)/c(m);这里是拿"上一次的 m_est"代入同一套代数,好让日志有个
            # 可读的当量。attach 那一帧 m_est 还≈m_B,所以会显示 dJ=0/c=0——那不
            # 代表几何没生效,只代表此刻的质量估计还没涨上来。
            dJ, c_xy = self._geometry_from_m(self.m_est, geom)
        else:
            if self._payload_attached and self.attach_offset is not None:
                dJ, c_xy = self._payload_geometry(self.attach_offset)
            else:
                dJ, c_xy = 0.0, np.zeros(2)
            geom = np.array([dJ, c_xy[0], c_xy[1]])
        # 几何归零可验证性(B.5 验收要"drop 后一个 MHE 周期内 dJ/c_xy 归零"):
        # 状态翻转时打一条,便于从日志直接判定释放时刻。
        _geom_on = (bool(np.any(geom != 0.0)) if mhe_p.geom_coupled
                    else self._payload_attached)
        if getattr(self, '_geom_active_prev', None) != _geom_on:
            self.get_logger().info(
                f'[geom] model_geom_on={_geom_on} '
                f'(release_mode={self.geom_release_mode}, '
                f'truth_attached={self._payload_attached}) '
                f'mode={"coupled(r_p)" if mhe_p.geom_coupled else "legacy(dJ,c)"} '
                f'-> geom={np.array2string(geom, precision=4)} '
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

        # 实时性仪表:只测 solver.solve() 本身的墙钟耗时,用于论文 timing 表。
        # 与 acados_nmpc_node 里的 solve_time 同口径(墙钟、单位 ms),便于对表。
        _t_solve0 = time.time()
        status = self.solver.solve()
        self._solve_ms.append((time.time() - _t_solve0) * 1000.0)
        if status != 0:
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
                self.get_logger().warn(
                    f're-anchored arrival prior to current window after '
                    f'{self._fail_streak} consecutive failures '
                    f'(m seeded from thrust_phys: {self.m_est:.3f} -> '
                    f'{m_seed:.3f} kg)')
                self.m_est = m_seed
                self._fail_streak = 0
            return
        self._fail_streak = 0

        x_sol = [self.solver.get(i, 'x') for i in range(N + 1)]
        self.m_est = float(x_sol[N][nx])
        if mhe_p.ns:
            self.s_est = np.array(x_sol[N][nx + mhe_p.nm:nx + mhe_p.nm + mhe_p.ns],
                                  dtype=float)

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
            # B.3 Phase1 验证行:在线 c_xy 估计 vs attach 真值反推的 c_xy。真值
            # 用 grip_true_payload_mass(诊断真值)优先,否则退回 m_est 反推 m_p。
            if self.c_xy_est_enable and self._c_xy_inited:
                ref = ''
                if self.attach_offset is not None:
                    m_p = (self.grip_true_payload_mass
                           if self.grip_true_payload_mass > 0.0
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