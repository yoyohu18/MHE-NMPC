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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, Float64MultiArray
from actuator_msgs.msg import Actuators

from offboard_test.nmpc_node import quat_to_rotmat

from .mhe_params import p as mhe_p
from .mhe_solver_builder import ensure_mhe_ocp_solver
from .mhe_weight_learning import M0_THETA, ParametricWeightSchedule


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
        self.get_logger().info('MHE node starting, building/loading solver...')
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
        # 默认 0.0 = 关闭(退回原棘轮行为,保 0.3kg 已验证结果 + A/B 对照)。
        self.declare_parameter('grip_geom_mp_floor', 0.0)
        self.grip_geom_mp_floor = float(
            self.get_parameter('grip_geom_mp_floor').value)

        self.y_buf = []  # 测量缓冲区,最多 N+1 帧
        self.u_buf = []  # 已知输入缓冲区,最多 N 帧

        self.x0_bar = None
        self.x_guess = None
        self.m_est = mhe_p.m_nominal
        # 连续 solve 失败计数。失败时 x0_bar/x_guess 不更新而 y_buf/u_buf 照常
        # 滑动,先验会越滞后越矛盾 → status=2 自锁死循环(2026-07-06 吊挂
        # LIFT 暂态实测:一次硬失败后 2000+ 窗口连败、m_est 冻结)。连败达
        # 阈值就把先验重锚到当前窗口(质量保留最后估计),给 solver 台阶下。
        self._fail_streak = 0
        self.counter = 0
        self.frames = 0  # 已入缓冲的总帧数(事件调度器的全局帧序号基准)

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
        self.event_enabled = bool(
            self.get_parameter('event_trigger_enable').value)
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

        self.timer = self.create_timer(mhe_p.dt, self.timer_cb)
        self.get_logger().info(
            'MHE node initialized! Waiting for odometry + control data...')

    def odom_cb(self, msg):
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

    def mass_event_cb(self, msg):
        self._on_mass_event('drop (wrench)')

    def attach_event_cb(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.shape[0] == 3 and np.all(np.isfinite(data)):
            self.attach_offset = data
            self._m_p_hat_ratchet = 0.0  # 新 attach:棘轮重新从 0 起(见 __init__ 注释)
        self._on_mass_event('attach (gripper)')

    def _payload_geometry(self, r_p):
        """由载荷相对机体的几何偏移 r_p=[rx,ry,rz](rz<0)算 (dJ, c_xy)。跟
        acados_nmpc_node.py 的同名方法完全一致(平行轴定理+复合质心),区别
        是这里没有独立的 m_p/m_t 真值,需要用质量估计反推 m_p_hat——但不能
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
        """
        if self.grip_true_payload_mass > 0.0:
            m_p_hat = self.grip_true_payload_mass
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
            # 推力分量(u[0])换成电机转速反算的真实物理推力(见 MOTOR_CONSTANT
            # 注释),力矩 tau(u[1:4])仍沿用 NMPC 意图值(对质量估计是次要项)。
            # motor_speed 还没到时退回 NMPC 的推力,避免丢帧。
            if self.thrust_phys is not None:
                u = u.copy()
                u[0] = self.thrust_phys
            self.u_known = u

    def timer_cb(self):
        # NMPC 接管姿态控制之前(还在用位置 setpoint 飞向起点)没有 u_opt,
        # 这段时间没有意义的输入,直接跳过,不往缓冲区塞假数据。
        if self.x_meas is None or self.u_known is None:
            return

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

        if len(self.y_buf) < mhe_p.N + 1:
            return  # 窗口还没攒满

        self._solve_window()

    def _solve_window(self):
        N, nx, nw = mhe_p.N, mhe_p.nx, mhe_p.nw
        y_win = self.y_buf
        u_win = self.u_buf

        if self.x0_bar is None:
            self.x0_bar = np.concatenate([y_win[0], [mhe_p.m_nominal]])
            self.x_guess = [np.concatenate([y_win[min(i, N)], [mhe_p.m_nominal]])
                             for i in range(N + 1)]

        # 已知几何 [dJ, cx, cy]:用当前 m_est 和 attach 实测偏移算(见
        # _payload_geometry),窗口内所有 stage 共用同一个当前值——geometry 是
        # 常量、只有 m_p_hat 随 m_est 缓慢变,不需要按帧存历史,跟 T/tau 的
        # 逐帧历史值语义不同。没有 attach(wrench 场景/attach 前)则为全零。
        if self.attach_offset is not None:
            dJ, c_xy = self._payload_geometry(self.attach_offset)
        else:
            dJ, c_xy = 0.0, np.zeros(2)
        geom = np.array([dJ, c_xy[0], c_xy[1]])

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

        # 事件触发权重调度:事件过渡期内把事件前 stage 降权(机制见
        # mhe_event_weights.py),过渡期结束由调度器自己恢复名义权重
        in_transition = self.scheduler.apply(self.solver, self.frames - 1)
        if in_transition != self._in_transition:
            self.get_logger().info(
                'event transition {}'.format(
                    'started: pre-event stages de-weighted'
                    if in_transition else 'ended: nominal weights restored'))
            self._in_transition = in_transition

        status = self.solver.solve()
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
                if self.thrust_phys is not None:
                    m_seed = float(np.clip(
                        self.thrust_phys / mhe_p.g, mhe_p.m_min, mhe_p.m_max))
                else:
                    m_seed = self.m_est  # 没有电机数据时没有更好的退路
                self.x0_bar = np.concatenate([y_win[0], [m_seed]])
                self.x_guess = [
                    np.concatenate([y_win[min(i, N)], [m_seed]])
                    for i in range(N + 1)]
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
            self.get_logger().info(
                f'MHE mass estimate: {self.m_est:.3f} kg (T_phys={t_phys:.2f}N)')
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


def main():
    rclpy.init()
    node = MHENode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()