#!/usr/bin/env python3
# acados 版 NMPC 节点。状态机(预热→等EKF→切OFFBOARD→解锁→飞到起点→NMPC接管)
# 跟 offboard_test/nmpc_node.py 的 NMPCNode 完全一致,直接照搬,只有 solve_nmpc()
# 内部换成调 acados 求解器,其它行为(包括坐标系、限幅、推力归一化、body_rate
# 斜坡)都保持不变,方便跟 CasADi/IPOPT 版本直接对比。

import functools
import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSetV2, SetMode
from rcl_interfaces.msg import ParameterType, ParameterValue
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray

from offboard_test.nmpc_node import (
    build_reference,
    build_reference_window,
    quat_to_euler,
    quat_to_rotmat,
)
from .figure8_reference import (
    auto_ramp_time,
    build_reference_figure8,
    build_reference_window_figure8,
)
from .straight_reference import (
    build_reference_straight,
    build_reference_window_straight,
)

from .acados_params import YAW_RATE_K, p, scaled_stage_terminal_W
from .acados_solver_builder import ensure_acados_ocp_solver
from .l1_adaptive import L1Augmentation

# 转子标定常数。**必须与 mhe_node.py 顶部的同名常数保持一致**——两处各自硬编码
# 是沿用既有风格(避免 nmpc_node import mhe_node 拖进整个节点类),改一处必须改另
# 一处。用途:从电机转速反算实测推力 T_phys 与实测体力矩 tau_phys。
MOTOR_CONSTANT = 8.54858e-06
ROTOR_X = np.array([0.174, -0.174,  0.174, -0.174])   # motorNumber 0..3 机体 x
ROTOR_Y = np.array([-0.174, 0.174,  0.174, -0.174])   # 同上 y
TORQUE_SIGN = 1.0
from .mhe_params import p as mhe_p


class AcadosNMPCNode(Node):
    def __init__(self):
        super().__init__('acados_nmpc_node')
        self.get_logger().info('acados NMPC node starting, building/loading solver...')

        self.solver = ensure_acados_ocp_solver()

        self.state      = State()
        self.x_cur      = None
        self.counter    = 0
        self.start_time = None
        self.armed_and_flying  = False
        self.last_mode_req_time = 0.0
        self.last_arm_req_time  = 0.0
        self.ekf_wait_sec = 10.0
        self.nmpc_started = False
        self.nmpc_start_time = None
        self.z_hover = 3.0
        self.start_xy_threshold = 0.20
        self.start_z_threshold  = 0.15
        # 归一化油门 <-> 真实推力 的物理映射(2026-07-02 查 PX4 源码确认,取代早先
        # 的线性启发式 norm=hover_thrust_pct*T/(p.m*g)——那条是过原点直线,只在满载
        # 工作点跟真实推力曲线相切,drop 到空载后失配,实测稳态偏低~12cm)。链路:
        #   1) offboard AttitudeTarget 的 thrust 走姿态速率环、直接当集合推力设定,
        #      位置控制器被绕过,MPC_THR_HOVER(PX4 位置控制器用来估算悬停油门的
        #      参数,默认 0.6)不参与;
        #   2) THR_MDL_FAC=0(airframe 默认)→ PX4 不做推力曲线线性化,电机控制信号
        #      = 归一化推力,线性透传;
        #   3) GZMixingInterfaceESC(仿真里的电机接口)把归一化[0,1]缩放成电机角速度:
        #      ω = OMEGA_MIN + OMEGA_SPAN*norm  (SIM_GZ_EC_MIN/MAX = 150/1000 rad/s);
        #   4) gz MulticopterMotorModel(电机物理模型)出力 T = THRUST_K * ω²(4 电机合计)。
        # 合成正向: T(norm) = THRUST_K*(OMEGA_MIN + OMEGA_SPAN*norm)²;反解在
        # publish_attitude 里。THRUST_K 就是 4×SDF motorConstant,不乘任何标定
        # 增益——这里曾短暂放过 ×1.2134,那是按"满载 2.5kg"错误前提标出的幽灵
        # 增益(mass_changer Configure 时刻的 SetInertial 其实也进不了 DART,
        # 飞机全程 2.064kg,见 mhe_node.py THRUST_CAL_GAIN 注释的四组互证),
        # 已回退。与 mhe_node 的 MOTOR_CONSTANT×THRUST_CAL_GAIN 同源,两边必须
        # 一致,否则 MHE 的质量读数和 NMPC 的推力反解会互相打架。
        self.OMEGA_MIN  = 150.0    # SIM_GZ_EC_MIN
        self.OMEGA_SPAN = 850.0    # SIM_GZ_EC_MAX(1000) - SIM_GZ_EC_MIN(150)
        self.THRUST_K   = 4.0 * 8.54858e-06   # T=THRUST_K*ω² [N/(rad/s)²], 4 电机合计
        self.omega_cmd_max = 2.0
        self.bodyrate_ramp_time = 0.8
        # 先跑悬停测试,跟当年 CasADi 节点的验证顺序一样:先确认能稳定悬停,
        # 再切 False 去追轨迹
        self.hover_test_mode = False

        # 'circle' / 'figure8' / 'straight' 三种参考轨迹都保留、互不影响,改这
        # 一个字符串就能切换。straight 是往返直线(yaw 固定不变,详见
        # straight_reference.py),用来直观验证机头朝向是否跟随飞行方向——比
        # 圆形/8字更容易在 RViz 里一眼看出对不对。
        self.trajectory_shape = 'figure8'

        # ---- figure8 轨迹尺度(高机动消融,2026-07-29)----
        # 原来 r/w 是 figure8_reference 里写死的默认值(r=1.0/w=0.3),整条裸机
        # 路径没有任何旋钮能调,提难度必须改源码。这里接成 ROS 参数,默认值跟
        # 原来完全一致,所以不传参时历史批次逐字节复现。
        #   包络 = 2r × r,峰值速度 v_peak = √2·r·w,峰值水平加速度 = v_peak²/r。
        #   例:r=5.0/w=0.707 → 10m×5m 包络、5 m/s 峰值、5 m/s² (27° 倾角)。
        # fig8_ramp: 0=沿用写死的 4.0(默认,不变);<0=按 auto_ramp_time(w) 自适应
        # (高机动必须用,否则振幅渐增本身的额外速度会造成起步冲击);>0=显式指定。
        self.declare_parameter('fig8_r', 1.0)
        self.declare_parameter('fig8_w', 0.3)
        self.declare_parameter('fig8_ramp', 0.0)
        # 轨迹尺度联动 Bryson 权重(默认关,开了才动 Q/P)。L_vel/L_omega 在
        # acados_params 里是按 r=1.0/w=0.3 写死的,高机动下跟实际信号量级差一个
        # 数量级。开启后按最终生效的 (r,w) 重算并 cost_set 进求解器。见
        # scaled_stage_terminal_W 的推导;它对低速工况取 max() 后返回原值,
        # 所以即使误开也不会改变低速批次的结果。
        self.declare_parameter('traj_scale_weights', False)
        self.fig8_r = float(self.get_parameter('fig8_r').value)
        self.fig8_w = float(self.get_parameter('fig8_w').value)
        _fr = float(self.get_parameter('fig8_ramp').value)
        self.fig8_ramp = (auto_ramp_time(self.fig8_w) if _fr < 0.0
                          else (4.0 if _fr == 0.0 else _fr))

        if self.trajectory_shape == 'figure8':
            # partial 绑住尺度参数,这样下游所有 ref_fn(t,...)/ref_window_fn(
            # t,N,dt,...) 调用点一处都不用改,也不会漏掉某个调用点用回默认 r/w
            # ——_build_ref_path_msg 那条注释记的就是这类"两处参数没对齐"的 bug。
            self.ref_fn = functools.partial(
                build_reference_figure8, r=self.fig8_r, w=self.fig8_w,
                ramp_time=self.fig8_ramp)
            self.ref_window_fn = functools.partial(
                build_reference_window_figure8, r=self.fig8_r, w=self.fig8_w,
                ramp_time=self.fig8_ramp)
        elif self.trajectory_shape == 'straight':
            self.ref_fn = build_reference_straight
            self.ref_window_fn = build_reference_window_straight
        else:
            self.ref_fn = build_reference
            self.ref_window_fn = build_reference_window

        # 求解失败处理用:偶尔失败一次不去扰动求解器内部的 warm-start(让它
        # 保留当前、哪怕没收敛的迭代值当下一次起点),只有连续失败太多次
        # (大概率已经飞出去了)才强制拉回安全悬停状态。
        self.solve_fail_count = 0
        # 按 wall-clock ~2s 折算(不再硬编码 20@dt=0.1s),避免改 p.dt 时这个
        # 容忍窗口跟着悄悄缩短/拉长——2026-07-08 把 dt 从 0.1 改 0.05 时发现的。
        self.max_consecutive_fail = max(1, round(2.0 / p.dt))
        self.last_u_opt = p.u_hover.copy()
        self.last_omega_cmd = np.zeros(3)

        # --- 解耦发布(路 A):NMPC 以 1/dt Hz 求解整条预测轨迹,由一个更高频的
        # 定时器沿轨迹插值前推发 body_rate + thrust 到 publish_hz。
        # 目的:减小低频零阶保持带来的相位滞后,并让 attach/lift
        # /drop 这类快暂态的指令在两次求解之间平滑演进而不是阶梯保持。
        # _traj_omega: (3, N+1) 未 ramp 的角速度轨迹(网格 t=0,dt,..,N·dt);
        # _traj_thrust: (N,) 推力轨迹(网格 t=0,..,(N-1)·dt);None=退化零阶保持。
        self._traj_omega = None
        self._traj_thrust = None
        # 上次求解起点的 ROS 时刻(get_clock().now(),受 use_sim_time 控制)。
        # 必须用 ROS clock 而非 time.time():插值网格刻度是 p.dt(仿真时间),
        # 若用墙钟算 tau,RTF≠1 时会系统性错配(RTF<1 前推不足、RTF>1 撞 clamp),
        # 悄悄削掉甚至反转解耦发布的收益。None=还没有第一条可信轨迹。
        self._traj_stamp = None
        self._omega_grid = np.arange(p.N + 1) * p.dt
        self._thrust_grid = np.arange(p.N) * p.dt

        self.max_res_stat = 0.0
        self.last_res_stat = 0.0
        self.last_sqp_iter = 0

        # 闭环自适应质量:默认是 acados_params.py 里的标定常数 p.m,一旦
        # mhe_node 发来新估计就实时更新——下一次 solve_nmpc 调用就会用这个新
        # 值算动力学(见 acados_model.py 的 m_sym),不需要重新生成/编译求解器。
        # 用 mhe_params 里同一套边界做夹紧,防止 MHE 估计器偶尔抽风给出离谱值
        # (比如窗口还没收敛时)直接污染 NMPC 的动力学模型。
        #
        # "投放"场景(方案A,见 model.sdf 里 mass_changer 插件的注释):原本按
        # "飞机从起飞就带 payload"把初值设成 p.m+payload=2.564。但 2026-07-03
        # 接管窗口逐帧实测坐实:mass_changer 的满载 2.5kg 从没进 DART,飞机全程
        # 真实质量 ≈2.05kg(维持高度时 T 稳定在 ~20N=2.05×g;闭环 MHE 也在
        # t≈2.1s 独立收敛到 2.06)。用 2.564 初始化 => 接管头 2s NMPC 悬停推力
        # 高估 ~25%(25.15 vs 真实 20.1N)=> 接管瞬间强烈上冲(vz 峰 +0.7m/s、
        # 过冲 +0.27m,就是观察到的"起飞"),直到 MHE 收敛才回落。故初值改回
        # 真实空机 p.m,消掉这段过推。若将来修好 mass_changer 让满载真进 DART,
        # 这里需把 m_est 初值改回 p.m+payload。
        self.m_est = p.m
        # 吊挂载荷惯量增量(model.p 的第 15 维,见 acados_model.py dJ_sym 注释)。
        # mass_changer 场景是 wrench 模拟的纯平动质量变化、无惯量变化,恒 0;
        # 只有 gripper 场景 attach 后才会被 _grip_mass_step 阶跃。
        self.dJ_est = 0.0
        # 复合质心水平偏移 c=[cx,cy](model.p 第 16-17 维,见 acados_model.py
        # c_sym 注释)。同样只在 gripper attach 后由 _grip_mass_step 赋值。
        self.c_est = np.zeros(2)
        # 平动 lumped 扰动比力 d(model.p 第 18-20 维,C.1 L1 增广,见
        # acados_model.py d_sym 注释)。truth/online 模式恒零(行为与 17 维版
        # 逐位一致);L1 模式(control_mode='l1',下一步接线)由 L1Augmentation
        # 在线赋值。
        self.d_lumped = np.zeros(3)
        # attach 瞬间的真实几何偏移(box-机体,proximity 节点发布),None=还没
        # 收到。有它就用真实力臂/偏心算 dJ 和 c_est,没有才退回 grip_arm_d 参数。
        self.attach_offset = None

        # ---- 强闭环几何来源 A/B 开关(B.3 Phase2,2026-07-14)----
        # geom_source='truth'(默认,现有行为):dJ/c_est 从 attach_offset 真值算;
        # 'online':c_est 吃 mhe_node 发的 /acados_nmpc/c_xy_est(电机力矩反算,
        # 与真值无关),dJ 由 m_est+rz 几何先验(grip_arm_d)在线推——消掉对 attach
        # 真值的依赖。attach_offset 仍收(用于 [attach-window] 评估对表),但 online
        # 模式下不喂几何。Phase1 已验证 c_xy_est 追真值<5%。⚠️前提:m_est 须健康
        # (信号质量依赖健康飞行,见 memory b3-strong-closed-loop-dr Phase1 教训),
        # gripper 场景配 GRIP_GEOM_MP_FLOOR。
        # 几何释放方式(2026-08-24,与 mhe_node 的同名参数同一语义):
        # 'event' = drop 后把几何槽清零(控制器知道自己松了夹爪,历史行为);
        # 'self'  = 几何槽保持最后一次 attach 的杆臂,载荷贡献由 c(m)/J(m) 里的
        #           m_P=(m−m_B)⁺ 随 m_est 自行熄灭 —— 控制器模型也只通过质量
        #           估计感知掉包,不用释放指令这条旁路。仅 NMPC_GEOM_COUPLED=1 有意义。
        self.declare_parameter('geom_release_mode', 'event')
        self.geom_release_mode = str(
            self.get_parameter('geom_release_mode').value).lower()
        if self.geom_release_mode not in ('event', 'self'):
            self.geom_release_mode = 'event'
        if self.geom_release_mode == 'self' and not p.geom_coupled:
            self.get_logger().error(
                "geom_release_mode='self' 需要 NMPC_GEOM_COUPLED=1,已退回 'event'")
            self.geom_release_mode = 'event'
        if self.geom_release_mode == 'self':
            # 2026-08-24 实测:控制器侧用 'self' 必然坠机(见 mhe_node 同名参数注释
            # 里的力矩量级)。这里只告警不强制——留给对照实验,但默认脚本不会开。
            self.get_logger().warn(
                "⚠️ NMPC geom_release_mode='self':drop 后控制器仍按幽灵偏心载荷"
                "配平,实测 2/2 坠机。除非在做对照实验,否则请用 'event'。")

        self.declare_parameter('geom_source', 'truth')
        self.geom_source = str(self.get_parameter('geom_source').value).lower()
        if self.geom_source not in ('truth', 'online'):
            self.geom_source = 'truth'
        self.c_xy_online = np.zeros(2)     # 最新在线 c_xy 估计(订阅)
        self.geom_online_active = False    # attach 后 online 几何是否已接管

        # ---- C.1 控制模式开关(2026-07-23,实施计划_C1 §8 D3 拍板:独立参数,
        # 不复用 geom_source——geom_source 管"几何从哪来",control_mode 管
        # "估质量(流派A) vs 补 lumped 扰动(流派B)"两条路线)----
        # 'mhe'(默认):现有行为,m_est/dJ/c_est 喂模型,d_lumped 恒零。
        # 'l1'  :L1-NMPC baseline——m_est 固定标称 p.m(mhe_mass_cb 旁路),
        #         dJ/c_est 恒零,d_lumped 由 L1Augmentation 在 odom_cb 高频
        #         更新;**不做 (J+dJ)/J 内环增益缩放**(对照设计:L1 只在外环
        #         补力、不动内环,见实施计划 §4)。
        # 三处门控已接线(2026-07-23,同日):①_grip_mass_step attach 旁路
        # (不做 m_est 阶跃/几何/增益);②_grip_drop_phase 里 l1.reset()(掉包
        # 残留 d̂ 会拽偏推力,同 rate 增益复位教训);③attach_offset_cb 的
        # _scale_px4_rate_gains 门控。
        self.declare_parameter('control_mode', 'mhe')
        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        if self.control_mode not in ('mhe', 'l1'):
            self.control_mode = 'mhe'
        self.declare_parameter('l1_a_gain', 10.0)    # 预测器带宽(估计极点-a/2)
        self.declare_parameter('l1_omega_c', 0.5)    # 补偿低通截止(Phase1 实测:0.5 稳、1.0/2.0 与外环带宽耦合致 z 振荡炸机,07-23 扫参定)
        self.l1 = L1Augmentation(
            a_gain=float(self.get_parameter('l1_a_gain').value),
            omega_c=float(self.get_parameter('l1_omega_c').value))
        self._l1_last_stamp = None      # odom 时间戳(算实际 dt,防墙钟/仿真钟混用)
        self._l1_log_count = 0

        # ---- 转动 lumped 扰动通道 xi(2026-08-25)----
        # 补齐 Hanover RA-L 2021 的 matched uncertainty σ_m=[ς_z, ξ_x, ξ_y, ξ_z]:
        # 07-23 只做了平动那一半(d_lumped),转动这三维一直缺着,om_dot 里的模型
        # 误差(dJ 先验错、c_xy 先验错、未建模气动力矩)因此无处可去,只能靠先验准。
        # **独立于 control_mode**:主线 mhe 模式下同样可开——这正是目的,让 NMPC
        # 的 dJ 先验降级成"可以给错的值"。默认 False → model.p 的 xi 槽恒零,
        # 逐位退回历史行为。
        self.declare_parameter('tau_lumped_enable', False)
        self.tau_lumped_enable = bool(
            self.get_parameter('tau_lumped_enable').value)
        # xi 限幅 [rad/s²]。**40 不是拍脑袋**:0.3kg@0.47m 的 dJ=0.0663 是空机
        # Jxx=0.0142 的 4.7 倍,若 dJ 先验完全不给,补偿量 |xi| = τ·(1/J_true −
        # 1/J_nom) 随力矩线性涨——test_tau_lumped_standalone.py 实测 τ=0.35Nm
        # (70% tau_max)就要 20.3、τ=tau_max 要 29.0。首版设 20 会在大机动时
        # **静默截断**(限幅了也不报错,只表现为补偿不足),故放到 40 留余量。
        self.declare_parameter('xi_max', 40.0)
        # 转动通道的补偿低通截止,**与平动的 l1_omega_c 分开**:平动那个 0.5 是
        # C.1 扫参定的,机制是"omega_c 高 → 与外环位置带宽耦合 → z 振荡炸机";
        # 转动 xi 进的是力矩通道、直接面对 PX4 速率内环(带宽高得多),约束不同源,
        # 不该被平动的结论绑住。默认仍取 0.5 保守起步(实测收敛 5.06s);attach
        # 瞬态若嫌慢就扫这个参数(1.0→2.77s、2.0→1.71s,见 standalone 测试)。
        self.declare_parameter('xi_omega_c', 0.5)
        self.xi_lumped = np.zeros(3)
        # dim=2:只估 roll/pitch。dJ 只进 Jxx/Jyy(见 acados_model 的 J_vec),
        # yaw 通道不受 dJ 先验影响;且 tau_phys 的 yaw 分量是 0 占位,若把 yaw
        # 也交给 L1,真实 yaw 力矩会被整个当成扰动积到限幅。
        self.l1_rot = L1Augmentation(
            a_gain=float(self.get_parameter('l1_a_gain').value),
            omega_c=float(self.get_parameter('xi_omega_c').value),
            dim=2, d_max=float(self.get_parameter('xi_max').value))
        self._l1_rot_last_stamp = None
        self._l1_rot_log_count = 0
        # a_nom 的推力项用**电机转速反算的实际推力**,绝不用 last_u_opt(NMPC
        # 指令)——首版实测(2026-07-23,grip_nmpc_162421)用指令推力在 lift 段
        # 自激炸机:T 快变时 10Hz 零阶保持的指令与实际差一大截 → a_nom 错 →
        # 垃圾进 d̂ → d̂ 喂回模型 → 推力更错 → bang-bang(40.5↔0.5N)。这正是
        # 07-13 拍板"观测量用电机转速反算、不用 NMPC 指令"要避开的"估计↔模型"
        # 自举,在 L1 上重演了一次。口径与 mhe_node 完全同源:
        # T_phys = MOTOR_CONSTANT·Σω²(k 两侧抵消性质同样成立)。
        # ---- ω_cmd 缩放:增益调度的等效实现(2026-08-25)----
        # 发给 PX4 的是 body_rate setpoint 不是力矩(见 publish_attitude),PX4 速率环
        # 产生 τ = K(ω_cmd − ω)。真实惯量 J_t 下角加速度 = K(ω_cmd−ω)/J_t,比期望
        # 小 J_t/J_a 倍。把 **setpoint 误差**放大同样倍数:
        #     ω_cmd' = ω + (Ĵ_t/J_a)·(ω_cmd − ω)
        # 数学上等价于把速率环增益乘 Ĵ_t/J_a,但**完全在本进程内、每帧可变**,
        # 不碰 PX4 参数 —— 绕开了 [drop-disturbance-fix] 证伪过的那条路的三个硬伤:
        # ParamSetV2 异步往返延迟、参数在 PX4 侧阶跃生效、写入触发落盘 I/O。
        # 同源做法见 FlyAware(RA-L 2026, arXiv:2601.22686)的 IAGS:
        # K_k = J_a^{-1}·Ĵ_t 乘在 PID 之外,使开环传函与 J_t 无关。
        #
        # ⚠️ **与 scale_px4_rate_gains 互斥**:两者补的是同一件事,同时开 = 双重
        #    补偿,循环增益翻倍。2026-08-25 的 xi 通道就是栽在双重补偿上(标称模型
        #    用空机 J 而 NMPC 模型已含 dJ,当场坠机 70m)。这里直接在代码里互锁,
        #    不靠使用者记得。
        #
        # ⚠️ **不是完全等价**:PX4 速率环的 D 项是对**测量**微分(不是误差微分,
        #    避免 setpoint 跳变),所以放大 setpoint 误差只等效放大了 P 和 I,
        #    D 不跟着涨 → 比例越大相对阻尼越低。5× 时这个差异不小,必要时把
        #    MC_*RATE_D 一次性(不是连续)设到匹配值。
        self.declare_parameter('omega_scale_enable', False)
        # 缩放因子来源:'thrust' = T_phys/g 反推有效承重(用户要的"跟着承重连续爬",
        # attach 渐进承重期间飞机近悬停,此时 T_phys/g ≈ 有效总质量);
        # 'mest' = 用 MHE 的 m_est。两者都再经同一套几何代数换成惯量比。
        self.declare_parameter('omega_scale_source', 'thrust')
        # 一阶低通时间常数 [s]。T_phys 有噪声、机动时还含向心分量,直接喂会让
        # setpoint 抖。0.5s 起步(远慢于速率环、快于 attach 承重过程)。
        self.declare_parameter('omega_scale_tau', 0.5)
        # 比例上限,与 _scale_px4_rate_gains 的 cap 同值同理由(防惯量估计异常时
        # 把内环推成高频震荡)。
        self.declare_parameter('omega_scale_cap', 5.0)
        self.omega_scale_enable = bool(
            self.get_parameter('omega_scale_enable').value)
        self.omega_scale_source = str(
            self.get_parameter('omega_scale_source').value).lower()
        self.omega_scale_tau = float(self.get_parameter('omega_scale_tau').value)
        self.omega_scale_cap = float(self.get_parameter('omega_scale_cap').value)
        self.omega_scale = 1.0              # 当前(已低通)比例,1.0=不缩放
        self._omega_scale_stamp = None
        self._omega_scale_clip_warned = False
        if self.omega_scale_enable:
            self.get_logger().info(
                f'OMEGA SCALE: on (source={self.omega_scale_source}, '
                f'tau={self.omega_scale_tau:.2f}s, cap={self.omega_scale_cap:.1f}) '
                '— 增益调度走 setpoint 侧,PX4 参数不动')

        _need_motor = (self.control_mode == 'l1' or self.tau_lumped_enable
                       or (self.omega_scale_enable
                           and self.omega_scale_source == 'thrust'))
        if _need_motor:
            from actuator_msgs.msg import Actuators
            self._l1_T_phys = None      # None=还没收到转速,L1 不推进(起飞前安全)
            self._l1_tau_phys = None    # 同上,转动通道用
            self.declare_parameter('motor_speed_topic',
                                   '/x500_0/command/motor_speed')
            self.create_subscription(
                Actuators, self.get_parameter('motor_speed_topic').value,
                self._l1_motor_cb, 10)
        if self.control_mode == 'l1':
            self.get_logger().info(
                f'CONTROL MODE: l1 (a={self.l1.a:.1f}, omega_c={self.l1.omega_c:.1f}'
                f') — m_est 固定 {p.m}kg,几何/增益缩放停用,平动扰动走 d_lumped')
        if self.tau_lumped_enable and p.geom_coupled:
            self.tau_lumped_enable = False
            self.get_logger().warn(
                'TAU LUMPED 与 geom_coupled 档不兼容(coupled 档的 J/c 由模型内部'
                '按 m_est 现算,标称侧要复刻那套代数会引入第二份实现)——已自动关闭 xi。')
        if self.tau_lumped_enable:
            self.get_logger().info(
                f'TAU LUMPED: on (a={self.l1_rot.a:.1f}, '
                f'omega_c={self.l1_rot.omega_c:.1f}, xi_max='
                f'{self.l1_rot.d_max:.1f}rad/s^2, roll/pitch only) — '
                'om_dot 的模型误差交给 xi 吸收')

        # "投放包裹"场景,第三版实现(前两版分别撞上了"独立 dynamic 刚体致命
        # 飞不起来"和"DetachableJoint 在同模型内 self-reference 不生效"两个
        # 坑,具体见 model.sdf 注释):质量真正的切换由 x500_payload 模型上的
        # 自定义 gz-sim C++ 插件(gz_plugins/mass_changer/)负责——它在
        # Configure 阶段(仿真刚启动、飞机还没解锁)就把 base_link 设成满载
        # 2.5kg,这里完全不需要发任何 attach 信号,飞机从起飞那一刻物理上就是
        # 满载的。飞到轨迹起点、开始 NMPC 追踪、追踪满 drop_after_track_sec
        # 秒(给 MHE 足够时间先收敛到带载真值,顺便验证带载情况下追踪本身是
        # 稳的)之后,这边只需要往 /payload/drop_mass 发一条消息,插件就会把
        # base_link 切回空载 2.0kg——质量做减法,而不是当年"抓取"设计里的
        # 加法,对 MHE 在线收敛能力的验证价值是等价的,阶跃方向不影响测试
        # 目的。设 payload_enabled=False 即可完全跳过 drop,退回"全程满载"
        # 的普通飞行(质量本身仍由插件管理,不会变成空机)。
        #
        # 2026-07-29 接成 ROS 参数(默认 True,历史行为不变):纯速度阶梯诊断需要
        # 关掉 drop。理由是 drop 会同时改三件事——有效质量阶跃、撞 MHE 质量下界、
        # 引入一个与速度无关的大扰动,把"速度效应"彻底混掉。关掉后 MassChanger
        # 不施加任何 wrench(drop 前本就没有),全程真实质量恒为裸机 2.0643kg,
        # m_est 不会碰下界,e vs v 的关系才测得干净。
        self.declare_parameter('payload_enabled', True)
        self.payload_enabled = bool(
            self.get_parameter('payload_enabled').value)
        self.drop_after_track_sec = 30.0
        self.drop_settle_sec = 2.0
        self.drop_triggered = False
        self.drop_trigger_time = None
        self.drop_done = False

        # ============ 夹爪吊挂实验模式(默认关;关闭时下面全部不生效,
        # 现有 mass_changer/figure8 行为逐字节不变)============
        # 用 DetachableJoint 磁吸夹爪代替 mass_changer:空机起飞 -> 低空悬停到
        # box 正上方 -> proximity 节点触发 attach 把真实独立 box 焊上来 -> 定时
        # 抬升把 box 吊离地面 -> MHE 检测质量阶跃、闭环喂回 NMPC 补偿。用来验证
        # acados NMPC + MHE 能否扛住裸 PID 扛不住的吊挂载荷突变。
        self.declare_parameter('gripper_mode', False)
        self.declare_parameter('grip_x', 1.0)
        self.declare_parameter('grip_y', 0.0)
        self.declare_parameter('grip_z_low', 0.7)   # 吸附时低空悬停高度
        self.declare_parameter('grip_z_high', 2.0)  # 抬升后悬停高度
        self.declare_parameter('grip_lift_after_sec', 12.0)
        self.declare_parameter('grip_lift_dur', 5.0)

        # ===== LIFT 中途 hold + dJ 跟随 m_est(2026-08-25,默认全关)=====
        # 动机(gviz_20260825_201344 逐帧实测):
        #   t=7.70 attach,box 仍在地上 → T_phys 恒为 20.1N(=空机重量,载荷重量
        #     由地面承担)→ m_est 不但不涨,反而 2.064→2.023 一路下漂。
        #     **所以"延长 attach 到离地的窗口给 MHE 时间"是无效的**:那段时间
        #     MHE 没有任何载荷信息可估,拉长只会让它漂得更低。
        #   t=10.70 box 离地 → T 跳到 24.4N,m_est 开始爬;
        #   t=11.30(离地后仅 0.6s)m_est=2.319,真值 2.364,误差 -1.9% → **信息这时
        #     就齐了**;
        #   t=13.19 发散。中间 1.9s 白白浪费,因为 dJ_est 在 attach 瞬间按
        #     grip_payload_envelope 定死后**再也不更新**(见 _update_online_geometry)。
        # 结论:要给 MHE 的时间应该加在**离地之后**,而不是 attach 之后;并且
        # 估出来的质量必须真的送到 dJ 和内环增益上,否则给再多时间也没用。
        #
        # (a) LIFT 中途 hold:抬升到 grip_lift_hold_dz 时把斜坡**冻结**
        #     grip_lift_hold_sec 秒。此时 box 已离地、载荷全额加载,MHE 有信号
        #     且飞机还没进入大机动,是估计的最佳窗口。
        self.declare_parameter('grip_lift_hold_enable', False)
        self.declare_parameter('grip_lift_hold_dz', 0.35)   # 相对 grip_z_low
        self.declare_parameter('grip_lift_hold_sec', 3.0)
        # (b) dJ 跟随 m_est(棘轮 + 上界):把 m_est 的增长送进模型 dJ 和内环增益。
        # ⚠️ 这条路 2026-07-28 撤销过一次,撤销理由必须原样搬到这里:
        #    旧实现**每帧双向重推** dJ,两个方向都不安全 ——
        #      · m_est 高估时 dJ 无界跟涨(实测 m_est=3.734 → dJ=0.204,真值 0.058
        #        的 3.5 倍),NMPC 这一侧没有 MHE 的棘轮/地板/m_min 保护;
        #      · m_est 低于空机时 m_p<=1e-3 的门把几何整个冻住(LIFT 段常态,
        #        上面那张表里 t=8.9~10.1 就是)。
        #    本实现用**棘轮 + 硬上界**同时堵住这两个方向:只增不减(治第二条,
        #    LIFT 段的下漂再也冻不住它),且封顶 grip_mp_cap(治第一条,高估时
        #    dJ 不会无界跟涨)。这与 MHE 侧 grip_geom_mp_floor + m_max 是同一套
        #    哲学的镜像。棘轮语义正是"不断增加的质量估计"。
        # drop 时是否给 MHE 发 mass_event(见 _grip_drop_phase 里的详细注释)。
        # 默认 True 保持历史行为;设 False 时**必须**同时给 mhe_node 配
        # event_signal_mode:=residual,否则 MHE 既收不到信号也不会自检测。
        self.declare_parameter('drop_publish_mass_event', True)
        self.declare_parameter('dj_track_mest', False)
        self.declare_parameter('grip_mp_cap', 0.6)       # 载荷质量操作包线上界 kg
        # 增益重缩放的滞回:棘轮相对上次缩放至少涨 dmp[kg],或涨够 (ratio-1)
        # 的相对量,取两者较大。⚠️ 纯比例滞回不行 —— 从 m_p≈0 起步时基数太小,
        # 0→0.3kg 会触发十几次(单测实测 14 次);绝对增量才是这里的主约束。
        self.declare_parameter('dj_gain_rescale_dmp', 0.05)
        self.declare_parameter('dj_gain_rescale_ratio', 1.25)
        # gripper drop 全流程(B.3,2026-07-14):lift 完成后悬停 grip_drop_after_sec
        # 秒再释放磁吸夹爪(发 /gripper/enable=false,proximity 节点收到即 detach),
        # 让 box 掉落,m_est 回空机、c_xy 归零——验证强闭环走完 attach→稳飞→drop。
        # 0=禁用(保持现有 attach-only 行为)。
        self.declare_parameter('grip_drop_after_sec', 0.0)
        # B.5 动态轨迹(2026-07-15):lift 完成+稳定后,从悬停切到 figure8 跟踪,
        # 带偏心载荷飞机动、drop 落在 figure8 中途——验证"机动中突变"。figure8
        # 偏移到切换瞬间 drone 的 xy(平滑过渡),z=grip_z_high。默认关。
        self.declare_parameter('grip_dynamic_after_lift', False)
        self.declare_parameter('grip_dyn_r', 0.8)          # figure8 半径(载荷,收小)
        self.declare_parameter('grip_dyn_w', 0.25)         # figure8 角速率(收慢)
        # 带载 figure8 的 ramp_time,语义同 fig8_ramp:0=写死的 4.0(默认,历史
        # 批次不变);<0=auto_ramp_time(grip_dyn_w) 自适应;>0=显式指定。
        self.declare_parameter('grip_dyn_ramp', 0.0)
        # 立体 8 字的高度起伏 dz[m](2026-07-30):z=grip_z_high+dz·sin(a),与 x
        # 同相,所以右叶(x>0)整体抬高 dz、左叶压低 dz,两叶在原点交叉处等高——
        # 裸机 fig8 一直是 dz=0.5 的立体 8 字,带载这条却写死 dz=0.0(平面),
        # 现在参数化。**默认仍是 0.0**,阶段 B 的历史批次逐字节复现。
        # 两处耦合,改 dz 前先过一遍:
        #  ① 最低点 = grip_z_high - dz,box 还挂在下方 grip_arm_d(默认 0.47m)
        #     处,要留离地余量:dz <= grip_z_high - grip_arm_d - 0.3 才稳妥。
        #  ② grip_drop_at_fig8_tip 丢在 a=3π/2,那里 sin=-1 = **轨迹最低点**,
        #     所以 dz 越大 drop 高度越低(落地冲击越小),不是等高丢。
        # 对 MHE 事件触发器的影响可忽略:z 起伏的推力扰动 ΔT=m·dz·w²,再经
        # 残差基线 EMA(τ=3s)高通打折,r=5/w=0.283/dz=0.8 只有 0.10N,相对
        # confirm_thresh(θ* 档 ≈2.9N)是 3%——远够不着误触发带。真正会撞阈值的
        # 是"小半径追高速"(ΔT=m·dz·v²/2r²),本场景 r=5.0 不在那个区。
        self.declare_parameter('grip_dyn_dz', 0.0)
        self.declare_parameter('grip_dyn_settle_sec', 3.0)  # lift 完成后多久切动态
        # drop 落点(动态模式):true=drop 精确落在 figure8 最左端(x=r·sin(a) 最小,
        # a=3π/2),grip_drop_after_sec 退化为"最早哪一圈可 drop"的最小门;
        # false=按 grip_drop_after_sec 固定时刻 drop。默认 false 保持现有行为。
        self.declare_parameter('grip_drop_at_fig8_tip', False)
        # 定时质量阶跃(和 MHE 解耦的诊断):吸附后把 m_est 从空机手动抬到
        # p.m+grip_payload_mass,给 NMPC "正确的带载质量认知",用来区分发散到底
        # 是"NMPC 不知道质量变了"还是"吊挂物理(CoM/摆动/拴系)本身补不了"。
        # grip_payload_mass<=0 则不做阶跃。
        self.declare_parameter('grip_payload_mass', 0.0)
        self.declare_parameter('grip_mass_step_sec', 4.0)
        # 吊挂力臂 d[m]:box 焊接点到机体的距离,用来算惯量增量 dJ=m_p*d^2
        # (见 acados_model.py dJ_sym)。跟 attach 时的 dz(proximity 日志里)
        # 对齐:当前窗口 h_min/h_max=[0.35,0.60],短臂稳态 dz≈0.47m。
        self.declare_parameter('grip_arm_d', 0.47)
        # 载荷**包线上界** [kg] —— 本节点唯一的载荷质量信息来源。
        #
        # 【2026-08-26 去先验改造】原先这里有三个参数:grip_payload_prior(模型侧
        # dJ 初值)、grip_gain_prior(内环增益整定)、grip_gain_envelope(包线)。
        # 前两个是**任务信息**("这次要抓的盒子大概 0.3kg"),部署时得有人告诉
        # 系统 —— 本项目的前提是**不能知道载荷质量**,否则在线质量估计失去意义,
        # 所以两者已删除。留下的这一个是**机架规格**("这架飞机最多吊得动多少"),
        # 写在整定表里、与飞哪一趟无关,拿它做整定不算"知道载荷质量"。
        #
        # 为什么不是直接删成 0(两条实测记录):
        #   ① 模型侧 dJ=0 → NMPC 按空机惯量规划角加速度、真实惯量 5 倍,力矩
        #      饱和后级联发散。08-25 run_prior_ab_4ms 首批 rep5 实测 LIFT 段末尾
        #      DIVERGED(peak_pos_err 15.1m,NMPC solve failed 1277 次,而 MHE 完全
        #      正常 m_est 误差 0.06%)—— 不是估计不准,是模型惯量为零。
        #   ② 增益侧归零 → 内环带宽被载荷惯量压塌,08-24 实测 MHE 62 次失败。
        # 包线上界同时治这两条:量级够大,且不含任务信息。
        #
        # 为什么在主线工作点上是**免费**的(08-25 解析核算):_scale_px4_rate_gains
        # 的 ratio=(Jxx+dJ)/Jxx 有 cap 5.0,而 dJ(m_p)=[m_B·m_p/(m_B+m_p)]·d²,
        # m_B=2.0643, d=0.47, Jxx=0.0142 → 撞 cap 的临界载荷是 m_p=0.2937kg。
        # 主线工况 0.3kg 已在 cap 里(裸 ratio 5.075),任何 >=0.2937 的包线给出
        # **逐位相同**的 MC_*RATE_K —— 那个"先验"的数值精度从来没被用到,只有
        # "够不够大"被用到,而"够大"正是包线的语义。
        # ⚠️ 轻载格子(0.15/0.2kg)裸 ratio 只有 3.18/3.84,**不在 cap 里**:换成
        #    包线会把增益一路提到 5.0,那是真的改了内环整定,需实测复核。
        # ⚠️ 模型侧 dJ 初值由包线算出会**过估**(0.5 包线 vs 0.3 真值 → dJ 高
        #    1.54×)。过估比低估安全(规划更保守,不会力矩饱和),且 dj_track_mest
        #    打开时 _update_online_geometry 会用在线 m_est 棘轮精修。
        self.declare_parameter('grip_payload_envelope', 0.5)
        # box 正上方的安全接近高度:两段式接近的第一段目标高度。先在这个高度
        # 把水平位置对齐、悬停稳,再垂直下降到 grip_z_low——避免在 grip_z_low
        # 这种低空做水平平移时高度下冲、起落架把 box 顶出吸附窗口(attach 竞态)。
        self.declare_parameter('grip_approach_z', 1.5)
        # 07-07 新增:NMPC 现在改成在 grip_approach_z(安全高度,离 box 远)就
        # 接管,而不是像旧版那样下降到 grip_z_low 才接管——根因是 MHE 的事件
        # 触发降权机制依赖 self.frames(只有 NMPC 发布 u_opt 后才计数),旧版
        # "NMPC 接管"和"物理 attach"焊在同一帧,MHE 窗口从来没机会攒够"事件前"
        # 数据,降权逻辑结构性没生效(2026-07-07 headless 验证发现,at frame 0
        # 就确认坐实)。grip_settle_sec:NMPC 接管后在安全高度稳定悬停这么久
        # (给 MHE 窗口热身,须 > N*dt=2.0s 留余量);grip_descend_dur:稳定期满
        # 后平滑下降到 grip_z_low 触发 attach 的过渡时长。
        self.declare_parameter('grip_settle_sec', 3.0)
        self.declare_parameter('grip_descend_dur', 2.0)
        # descend 斜坡走完只代表指令到达低位,不代表无人机真实高度已跟上。
        # attach/enable 必须再等真实 z 和速度收敛,否则会在系统性高度滞后时
        # 提前吸附,把 payload 挂得比 grip_arm_d 设计假设更深。
        self.declare_parameter('grip_descend_z_tol', 0.02)
        self.declare_parameter('grip_descend_v_tol', 0.15)
        # _grip_mass_step 原来纯按 nmpc_time 阈值触发,现在 NMPC 接管时刻和真实
        # attach 已经解耦,不能再假设阈值到时 attach 已经发生——必须等
        # attach_offset 真的到达;这个超时是它的兜底(万一 attach 真的没发生,
        # 别永远卡住,超时后退回旧的 grip_arm_d 兜底几何路径)。
        self.declare_parameter('grip_mass_step_timeout_sec', 5.0)
        # [attach-window] 逐帧诊断日志(供 CEM 学习脚本解析)的窗口时长——
        # gripper 瞬态比 wrench 的 [drop-window](硬编码 8.0s)长得多,同一份
        # 后续实测过 20-60s+ 才收敛的工况都有,给足余量。
        self.declare_parameter('attach_window_sec', 40.0)
        # attach 后把 PX4 内环速率 PID 的总增益 MC_ROLLRATE_K/MC_PITCHRATE_K
        # 按惯量比 (J+dJ)/J 放大。根因(2026-07-02 20:46 ulog 实测):挂 0.3kg
        # 后真实 roll 惯量是空机的 ~4.4 倍,但 PX4 速率环增益是按空机整定的,
        # 内环带宽掉到 1/4;NMPC 的 omega 指令方向每帧都对(正误差给负速率),
        # 但实际角速率响应滞后 0.3-0.5s,外环(Q_att=400 按空机敏捷内环调的)
        # 遇上迟钝内环 → 相位滞后 → roll 以 ~1Hz 增幅振荡(+3°→-13°→+50°→
        # 倾覆),跟"力矩饱和"是同一现象的两个面。给 NMPC 建再准的 dJ/质心
        # 模型都救不了这个:失配在"NMPC 规划的是力矩、执行的是 PX4 速率环"
        # 这个接口上,必须让内环自己知道惯量变了。设 False 可做 A/B 对照。
        self.declare_parameter('scale_px4_rate_gains', True)
        # use_mhe=False:MHE 估计不喂回 NMPC(m_est 固定),用于隔离验证控制器。
        # 默认 True,保持现有闭环行为不变。
        self.declare_parameter('use_mhe', True)
        self.use_mhe = bool(self.get_parameter('use_mhe').value)
        self.gripper_mode = bool(self.get_parameter('gripper_mode').value)
        if self.gripper_mode:
            self.grip_x = float(self.get_parameter('grip_x').value)
            self.grip_y = float(self.get_parameter('grip_y').value)
            self.grip_z_low = float(self.get_parameter('grip_z_low').value)
            self.grip_z_high = float(self.get_parameter('grip_z_high').value)
            self.grip_lift_after_sec = float(
                self.get_parameter('grip_lift_after_sec').value)
            self.grip_lift_dur = float(self.get_parameter('grip_lift_dur').value)
            self.grip_drop_after_sec = float(
                self.get_parameter('grip_drop_after_sec').value)
            self.grip_dynamic_after_lift = bool(
                self.get_parameter('grip_dynamic_after_lift').value)
            self.grip_dyn_r = float(self.get_parameter('grip_dyn_r').value)
            self.grip_dyn_w = float(self.get_parameter('grip_dyn_w').value)
            _gr = float(self.get_parameter('grip_dyn_ramp').value)
            self.grip_dyn_ramp = (auto_ramp_time(self.grip_dyn_w) if _gr < 0.0
                                  else (4.0 if _gr == 0.0 else _gr))
            self.grip_dyn_dz = float(self.get_parameter('grip_dyn_dz').value)
            self.grip_dyn_settle_sec = float(
                self.get_parameter('grip_dyn_settle_sec').value)
            self.grip_lift_hold_enable = bool(
                self.get_parameter('grip_lift_hold_enable').value)
            self.grip_lift_hold_dz = float(
                self.get_parameter('grip_lift_hold_dz').value)
            self.grip_lift_hold_sec = float(
                self.get_parameter('grip_lift_hold_sec').value)
            self.dj_track_mest = bool(self.get_parameter('dj_track_mest').value)
            self.drop_publish_mass_event = bool(
                self.get_parameter('drop_publish_mass_event').value)
            self.grip_mp_cap = float(self.get_parameter('grip_mp_cap').value)
            self.dj_gain_rescale_dmp = float(
                self.get_parameter('dj_gain_rescale_dmp').value)
            self.dj_gain_rescale_ratio = float(
                self.get_parameter('dj_gain_rescale_ratio').value)
            # LIFT hold 状态
            self._lift_hold_start = None   # 进入 hold 的时刻(None=还没进)
            self._lift_frozen_sec = 0.0    # 已冻结的总时长(从斜坡时间里扣掉)
            self._lift_hold_done = False
            # dJ 棘轮状态
            self._mp_ratchet = 0.0         # 见过的最大载荷质量(只增不减)
            self._mp_gain_applied = 0.0    # 上次用来缩增益的 m_p(滞回基准)
            self.grip_drop_at_fig8_tip = bool(
                self.get_parameter('grip_drop_at_fig8_tip').value)
            self.grip_payload_mass = float(
                self.get_parameter('grip_payload_mass').value)
            self.grip_mass_step_sec = float(
                self.get_parameter('grip_mass_step_sec').value)
            self.grip_arm_d = float(self.get_parameter('grip_arm_d').value)
            self.grip_payload_envelope = float(
                self.get_parameter('grip_payload_envelope').value)
            self.grip_approach_z = float(
                self.get_parameter('grip_approach_z').value)
            self.grip_settle_sec = float(
                self.get_parameter('grip_settle_sec').value)
            self.grip_descend_dur = float(
                self.get_parameter('grip_descend_dur').value)
            self.grip_descend_z_tol = float(
                self.get_parameter('grip_descend_z_tol').value)
            self.grip_descend_v_tol = float(
                self.get_parameter('grip_descend_v_tol').value)
            self.grip_mass_step_timeout_sec = float(
                self.get_parameter('grip_mass_step_timeout_sec').value)
            self.attach_window_sec = float(
                self.get_parameter('attach_window_sec').value)
            self.grip_climbed = False        # 接近第0段:先在起飞点原地爬到安全高度
            self.grip_high_aligned = False   # 两段式接近:第一段(高空对齐)是否完成
            self.grip_descend_started = False
            self.grip_descend_done = False
            self.grip_descend_done_time = None  # descend 完成(发 enable)的 nmpc_time
            self.grip_enable_sent = False    # 是否已主动发过 /gripper/enable
            self.attach_time = None  # _grip_mass_step 真正触发时的 nmpc_time
            self.grip_mass_stepped = False
            # 方案(a):attach 从"proximity 几何被动触发"改成"NMPC 主动门控"。
            # proximity 默认 disabled,收到 /gripper/enable=true 才判定几何。
            # NMPC 走完 接管->settle->descend 到位后由 _descend_phase 主动发一次
            # enable,attach 成为受控、时刻明确的事件(RELIABLE QoS,proximity
            # 早在线保证送达)。启动脚本里持续发 enable 的行已删。
            self.enable_pub = self.create_publisher(Bool, '/gripper/enable', 10)
            # PX4 内环增益同步(见 scale_px4_rate_gains 参数声明处的根因注释)
            self.scale_px4_rate_gains = bool(
                self.get_parameter('scale_px4_rate_gains').value)
            # ⚠️ 互锁必须在这里(而不是随参数一起提前):它依赖
            # scale_px4_rate_gains,那个参数只在 gripper_mode 分支里读。
            if self.omega_scale_enable and self.scale_px4_rate_gains:
                # 互锁:两者补同一件事,同开=双重补偿(见参数声明处注释)。
                self.scale_px4_rate_gains = False
                self.get_logger().warn(
                    'omega_scale_enable=true → 自动关闭 scale_px4_rate_gains'
                    '(两者补同一件事,同时开是双重补偿)')
            self.px4_gains_scaled = False
            self.px4_base_req_sent = False
            self.px4_rate_k_base = {}
            self.param_get_client = self.create_client(
                ParamGet, '/mavros/param/get')
            self.param_set_client = self.create_client(
                ParamSetV2, '/mavros/param/set')
            self.hover_test_mode = True     # 固定悬停,不追 figure8
            self.payload_enabled = False    # 不用 mass_changer 的 drop
            self.m_est = p.m                 # 空机起飞(不是 p.m+payload)
            # 07-07 改:NMPC 先在安全高度 grip_approach_z 接管(不是 grip_z_low),
            # 见 grip_settle_sec 参数声明处注释——给 MHE 窗口热身时间。
            self.z_hover = self.grip_approach_z
            self.grip_lift_started = False
            self.grip_drop_done = False   # gripper drop 是否已触发(一次)
            self.grip_dropped = False     # box 已释放(online 几何归零标志)
            self.grip_dynamic_active = False  # 是否已切到 figure8 动态跟踪(B.5)
            self.grip_dyn_t0 = None       # figure8 起始 nmpc_time
            self.grip_dyn_cx = 0.0        # figure8 xy 偏移(切换瞬间 drone 位置)
            self.grip_dyn_cy = 0.0
            self.ref_fn = self._grip_ref
            self.ref_window_fn = self._grip_ref_window
            # 放宽"到达悬停点"判据:box 在 move-to-start 阶段就被吸上,0.3kg
            # 载荷 + 低空地效让 PX4 位置控制稳态下垂约 0.2m,用原 0.15m 阈值会
            # 永远切不进 NMPC。放宽后让 acados 姿态控制接管,由它把高度顶上去。
            self.start_xy_threshold = 0.30
            self.start_z_threshold = 0.35
            self.get_logger().info(
                f'GRIPPER MODE: hover over box ({self.grip_x},{self.grip_y}) '
                f'z {self.grip_z_low}->{self.grip_z_high}m, lift after '
                f'{self.grip_lift_after_sec}s, empty m_est={self.m_est:.2f}kg')

        # ---- 轨迹尺度联动权重(必须放在 gripper 分支之后)----
        # 实际要飞的 8 字是哪一组 (r,w) 取决于模式:gripper 动态用 grip_dyn_*,
        # 裸机用 fig8_*。两个分支的参数都读完了才能选对,所以放这里而不是上面
        # 声明处。整条 horizon(0..N-1 用 stage W,N 用终端 W_e)一次性设完,
        # 之后不再改——与 MHE 侧 mhe_event_weights 的逐帧 cost_set 不是一回事。
        if bool(self.get_parameter('traj_scale_weights').value):
            _tr = self.grip_dyn_r if self.gripper_mode else self.fig8_r
            _tw = self.grip_dyn_w if self.gripper_mode else self.fig8_w
            W_stage, W_e = scaled_stage_terminal_W(_tr, _tw)
            for _i in range(p.N):
                self.solver.cost_set(_i, 'W', W_stage)
            self.solver.cost_set(p.N, 'W', W_e)
            self.get_logger().info(
                f'TRAJ-SCALED WEIGHTS: (r={_tr}, w={_tw}) -> '
                f'L_vel={max(p.L_vel, _tr*_tw):.3f} '
                f'L_omega={max(p.L_omega, YAW_RATE_K*_tw):.3f}')

        xref0 = self.ref_fn(0.0)
        # 预热求解器:acados 第一次 solve 之外,SQP 在远离收敛点时可能需要几次
        # 迭代才能收敛(实测验证过,见 test_acados_ocp_standalone.py),这里在
        # spin 之前先从悬停状态/悬停参考多解几次,把这部分代价提前消化掉,
        # 避免发生在 NMPC 接管那一帧阻塞 body_rate ramp。
        self.get_logger().info('Warming up acados solver...')
        t_warm = time.time()
        self._seed_initial_guess(xref0, np.tile(xref0.reshape(-1, 1), (1, p.N + 1)))
        for _ in range(10):
            u_opt, omega_cmd, _ = self.solve_nmpc(xref0, 0.0)
        self.get_logger().info(
            f'Solver warmup complete, took {(time.time()-t_warm)*1000:.1f} ms')

        mavros_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, mavros_state_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_cb, mavros_sensor_qos)

        # mhe_node 的质量估计,闭环喂回 NMPC 动力学(见 self.m_est 用法)
        self.mhe_mass_sub = self.create_subscription(
            Float64, '/acados_nmpc/mhe_mass_estimate', self.mhe_mass_cb, 10)
        # 在线 c_xy 估计(B.3 Phase2,geom_source='online' 时喂 model.p 的 c_est)
        self.c_xy_online_sub = self.create_subscription(
            Float64MultiArray, '/acados_nmpc/c_xy_est', self.c_xy_online_cb, 10)

        # proximity 节点在 attach 瞬间发布的真实几何偏移(TRANSIENT_LOCAL,
        # 订阅方必须同 QoS 才能收到 latched 消息)
        self.attach_offset_sub = self.create_subscription(
            Float64MultiArray, '/gripper/attach_offset',
            self.attach_offset_cb,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude', 10)

        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local', 10)

        # 话题名加 acados_ 前缀,跟 offboard_test 的 plot_logger.py 区分开,
        # 避免两个节点同时跑起来互相串话题
        self.ref_path_pub = self.create_publisher(
            Path, '/acados_nmpc/reference_path', 10)
        self.actual_path_pub = self.create_publisher(
            Path, '/acados_nmpc/actual_path', 10)
        self.nmpc_traj_pub = self.create_publisher(
            Path, '/acados_nmpc_traj', 10)
        self.tracking_err_pub = self.create_publisher(
            Float64, '/acados_nmpc/tracking_error', 10)
        # 纯诊断用:把 solve_nmpc 实际算出/采用的 [T,taux,tauy,tauz] 发出去,
        # 给 mhe_node(开环质量估计)当"已知输入"用。只在 NMPC 真正接管姿态控制
        # 后才有意义的数据(在那之前走位置 setpoint,没有 u_opt),不影响任何
        # 现有控制行为——纯增量。
        self.u_opt_pub = self.create_publisher(
            Float64MultiArray, '/acados_nmpc/u_opt', 10)

        # 质量突变事件通知(给 mhe_node 的事件触发权重调度用,见 mhe_event_
        # weights.py):drop 是本节点自己发起的"已知事件",触发 gz 侧质量切换
        # 的同一帧把事件广播出去,MHE 收到后立刻降权窗口内的旧数据。纯增量,
        # 不影响任何控制行为。
        self.mass_event_pub = self.create_publisher(
            Empty, '/acados_nmpc/mass_event', 10)

        # 传 fig8 尺度:period=2π/w 决定采样跨度、ramp_time 决定采样起点,两者
        # 不传就会用方法签名里的 w=0.3/ramp=4.0,fig8_w 一改 RViz 参考线就画不满
        # (或多画)一圈——正是方法注释里记的那类"两处参数没对齐"。gripper 模式
        # 下此刻 ref_fn 还是固定悬停点(退化线),传了也无副作用。
        self.ref_path_msg = self._build_ref_path_msg(
            r=self.fig8_r, w=self.fig8_w, ramp_time=self.fig8_ramp)
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = 'map'
        self.nmpc_traj_msg = Path()
        self.nmpc_traj_msg.header.frame_id = 'map'
        self.actual_path_max_len = 2000

        self.viz_timer = self.create_timer(0.1, self._publish_paths)

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(p.dt, self.timer_cb)

        # 解耦发布定时器(路 A)。decouple_publish=False 时完全退回旧行为
        # (timer_cb 里以 1/dt Hz 直接发 body_rate),便于 A/B 对照。
        # publish_hz<=1/dt 时提频无意义但也不会出错(τ 恒接近 0,退化成直接发布)。
        self.declare_parameter('decouple_publish', True)
        self.declare_parameter('publish_hz', 50.0)
        self.decouple_publish = bool(self.get_parameter('decouple_publish').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if self.decouple_publish:
            self.pub_timer = self.create_timer(
                1.0 / publish_hz, self._publish_from_traj)
            self.get_logger().info(
                f'Decoupled publish enabled: solve @{1.0/p.dt:.0f}Hz, '
                f'body_rate/thrust @{publish_hz:.0f}Hz (trajectory interpolation)')
        self.get_logger().info('acados NMPC node initialized! Waiting for EKF2 convergence...')

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        q   = msg.pose.pose.orientation
        om  = msg.twist.twist.angular
        qn = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if qn < 1e-6:
            return
        qw, qx, qy, qz = q.w/qn, q.x/qn, q.y/qn, q.z/qn
        # body(FLU) -> world(ENU),跟 offboard_test 的 odom_cb 一样的转换,
        # 不转的话飞机一倾斜速度就对不上,几秒内发散
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
        self.x_cur = x
        if self.control_mode == 'l1':
            self._l1_update(msg, R_wb, vel_world)
        if self.tau_lumped_enable:
            self._l1_rot_update(msg, x[10:13])      # x[10:13]=机体角速度(FLU)
        self._append_actual_path(msg)

    def _l1_motor_cb(self, msg):
        # 口径与 mhe_node.motor_speed_cb 同源:T_phys = k_f·Σω²(4 电机)。
        if len(msg.velocity) >= 4:
            w = np.asarray(msg.velocity[:4], dtype=float)
            if np.all(np.isfinite(w)):
                w2 = w * w
                self._l1_T_phys = float(MOTOR_CONSTANT * np.sum(w2))
                # 体力矩反算,与 mhe_node 逐字同源(B.3 Phase0):F_i=k_f·ω_i²,
                # τ_roll=Σy_i·F_i、τ_pitch=−Σx_i·F_i (FLU)。yaw 留 0 占位。
                f = MOTOR_CONSTANT * w2
                self._l1_tau_phys = np.array([
                    TORQUE_SIGN * float(np.sum(ROTOR_Y * f)),
                    TORQUE_SIGN * float(-np.sum(ROTOR_X * f)),
                    0.0])

    def _l1_update(self, odom_msg, R_wb, vel_world):
        """L1 增广的高频更新(odom 频率,比 NMPC 求解频率快——估计要快、补偿被
        omega_c 低通,时间尺度分离)。a_nom 按 acados_model 的 vel_dot 用**标称
        质量 p.m、d=0** 算(流派 B 设定:控制器全程不知道真实质量);推力用
        电机转速反算的 T_phys(实测量,见 __init__ 里 _l1_T_phys 注释——用
        NMPC 指令推力会自举炸机,首版实测教训)。dt 用 odom 消息时间戳差
        (仿真里是 sim time,墙钟会错拍)。"""
        if self._l1_T_phys is None:     # 还没收到转速:不推进(起飞前/话题未通)
            return
        # 地面门控(2026-07-23 第二轮实测教训,grip_nmpc_163705):飞机在地面时
        # T_phys≈0 但 v̇=0(地面支持力),L1 会把支持力"如实"估成 d_z=+g——数学
        # 上对、控制上致命(NMPC 带着"有向上的力"起飞→推力全错)。与 MHE 无信号
        # 版的预热门控是同款问题(nosignal-ablation 起飞暂态假触发)。低推力
        # (=地面/怠速/坠地)时清零并挂起,离地重新从零估计(收敛只要几秒)。
        if self._l1_T_phys < 0.4 * p.m * p.g:
            if np.any(self.d_lumped != 0.0):
                self.get_logger().info('[l1] low-thrust gate: reset (on ground)')
            self.l1.reset()
            self.d_lumped = np.zeros(3)
            self._l1_last_stamp = None
            return
        t = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        if self._l1_last_stamp is None:
            self._l1_last_stamp = t
            return
        dt = t - self._l1_last_stamp
        self._l1_last_stamp = t
        if not (0.0 < dt < 0.5):        # 时基跳变/暂停帧:跳过不推进
            return
        a_nom = (R_wb @ np.array([0.0, 0.0, self._l1_T_phys])
                 - p.kd * vel_world) / p.m - np.array([0.0, 0.0, p.g])
        self.d_lumped = self.l1.update(vel_world, a_nom, dt)
        self._l1_log_count += 1
        if self._l1_log_count % 100 == 0:   # ~每几秒一行,格式仿 [c_xy_est]
            d = self.d_lumped
            self.get_logger().info(
                f'[l1] d_hat_f=[{d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f}]m/s^2 '
                f'|equiv_dm={-d[2] * p.m / p.g:+.3f}kg')

    def _l1_rot_update(self, odom_msg, om_body):
        """转动 lumped 扰动 xi 的高频更新。与 _l1_update 同构,量纲是**角加速度**
        [rad/s²] 而不是力矩——换算成力矩要乘 J,而"J 是多少"正是要消掉的先验;
        输出角加速度,消费侧 om_dot += xi 不需要任何惯量假设。

        两条纪律:
        1) ω̇_nom 必须用 **NMPC 模型此刻真正在用的 J 和 c**(即 _geom_slot() 给
           出的 dJ/c_xy),不是空机值。L1 的定义是"残差 = 实测 − **控制器所用
           模型**的预测",标称模型与控制器不一致时 xi 会去补一个模型已经补过的
           量 → 双重补偿。
           ⚠️ 2026-08-25 首版就是写成空机 J 而**当场坠机**(grip_nmpc_003811,
           pos_err 峰值 70m、1094 次 solve failed)。机制:先验给对时 NMPC 的
           J_vec 已含 dJ=0.0663,xi 却按空机 J_nom=0.0142 去估 → xi ∝ tau·
           (1/J_true − 1/J_nom),NMPC 为抵消 xi 加大力矩 → tau_phys 变大 → xi
           更大,**循环增益 |J_model/J_nom − 1| = 4.7 >> 1 必然发散**。日志里
           悬停段 xi≈0(tau≈0 时暴露不出来),一进机动 4 秒内就从 0.03 冲到 19.3、
           tau_phys 打到 2.9Nm(tau_max 的 6 倍)。修正后循环增益变成
           |J_model/J_true − 1|,先验给对≈0、给错 20% 才 0.2,稳定。
        2) 力矩用电机转速反算的 tau_phys(实测),**不用 NMPC 指令力矩**。PX4 速率
           内环夹在中间,指令≠实际;用指令会把内环传递误差烙进 xi——与 07-23
           "用指令推力算 a_nom 自激炸机"是同一条教训。
        """
        if self._l1_tau_phys is None or self._l1_T_phys is None:
            return                      # 还没收到转速:不推进(起飞前/话题未通)
        # 地面门控:与平动通道同一条件同一理由(地面支持力会被"如实"估成扰动)。
        if self._l1_T_phys < 0.4 * p.m * p.g:
            if np.any(self.xi_lumped != 0.0):
                self.get_logger().info('[xi] low-thrust gate: reset (on ground)')
            self.l1_rot.reset()
            self.xi_lumped = np.zeros(3)
            self._l1_rot_last_stamp = None
            return
        t = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        if self._l1_rot_last_stamp is None:
            self._l1_rot_last_stamp = t
            return
        dt = t - self._l1_rot_last_stamp
        self._l1_rot_last_stamp = t
        if not (0.0 < dt < 0.5):        # 时基跳变/暂停帧:不推进
            return
        # 与 NMPC 模型逐项对齐(见 acados_model.py 的 legacy 分支 om_dot):
        # J_vec = [Jxx+dJ, Jyy+dJ, Jzz]、tau_thrust_com = [-cy*T, +cx*T, 0]。
        geom = self._geom_slot()                # [dJ, cx, cy](或 coupled 档的 r_p)
        if p.geom_coupled:
            # coupled 档 geom 装的是 r_p 几何偏移,J/c 由模型内部按 m_est 现算;
            # 这里不复刻那套代数(会引入第二份实现),该档暂不支持 xi。
            return
        dJ, cx, cy = float(geom[0]), float(geom[1]), float(geom[2])
        J = np.array([p.Jxx + dJ, p.Jyy + dJ, p.Jzz])
        tau_com = np.array([-cy * self._l1_T_phys, cx * self._l1_T_phys, 0.0])
        om_dot_nom = ((self._l1_tau_phys + tau_com
                       - np.cross(om_body, J * om_body)) / J)
        xi2 = self.l1_rot.update(om_body[:2], om_dot_nom[:2], dt)
        self.xi_lumped = np.array([xi2[0], xi2[1], 0.0])
        self._l1_rot_log_count += 1
        if self._l1_rot_log_count % 100 == 0:
            x_ = self.xi_lumped
            # equiv_dJ:把 xi 折算成"等效惯量误差"只在准静态、力矩主导时近似成立
            # (xi ≈ −τ·dJ/(J·(J+dJ))),仅供读日志时有个量级感,不进任何控制路径。
            self.get_logger().info(
                f'[xi] xi_hat_f=[{x_[0]:+.3f},{x_[1]:+.3f}]rad/s^2 '
                f'|tau_phys=[{self._l1_tau_phys[0]:+.3f},'
                f'{self._l1_tau_phys[1]:+.3f}]Nm')

    def attach_offset_cb(self, msg):
        data = np.asarray(msg.data, dtype=float)
        if data.shape == (3,) and np.all(np.isfinite(data)):
            self.attach_offset = data
            self.get_logger().info(
                f'attach offset received: box-drone = [{data[0]:+.3f}, '
                f'{data[1]:+.3f}, {data[2]:+.3f}] m')
            # attach 发生在 NMPC 接管前(posctl 低空悬停阶段),真实惯量从这一
            # 刻起就已经变大,立刻放大内环增益、不等 NMPC 的质量阶跃——posctl
            # 阶段的姿态保持同样受益,也避免接管瞬间"模型阶跃+增益阶跃"叠加。
            # online 模式不用 attach 真值算增益(消依赖),改由 _update_online_geometry
            # 在 dJ_online 起来后触发(见该方法);attach_offset 仅留作评估对表。
            # control_mode='l1' 同样不缩增益(流派 B 对照:不动内环,见 §4)。
            if (self.gripper_mode and self.grip_payload_mass > 0.0
                    and self.geom_source == 'truth'
                    and self.control_mode != 'l1'):
                dJ, _ = self._payload_geometry(data)
                self._scale_px4_rate_gains(dJ)

    def c_xy_online_cb(self, msg):
        d = np.asarray(msg.data, dtype=float)
        if d.shape == (2,) and np.all(np.isfinite(d)):
            self.c_xy_online = d

    def _update_online_geometry(self):
        """B.3 Phase2:online 模式下 c_est 吃 mhe_node 发的在线 c_xy(τ_phys 反算,
        与 m_est 无关);**dJ 不在这里更新**——它在 attach 瞬间由操作先验
        (grip_payload_envelope + grip_arm_d)一次算定,之后保持不变。每次 solve 前调。

        2026-07-28 改:原实现每帧用 m_p=m_est-p.m 重推 dJ,等于把 07-19 在 MHE 侧
        切断的 m_est→dJ 链在 NMPC 侧接了回来。两个方向都不安全:m_est 高估时 dJ
        无界跟涨(实测见过 m_est=3.734 → dJ=0.204,真值 0.058 的 3.5 倍),而这一侧
        没有 MHE 的棘轮/地板/m_min 保护;m_est 低于空机时旧的 `m_p<=1e-3` 门又把
        几何整个冻住(LIFT 段常态)。代价只有 dJ 的水平项 ½(rx²+ry²)——占 dJ 2.2%
        (rz² 主导 97.8%),而 dJ 只需量级对(几何解耦实验:先验错 33% 时 m_est 仍准
        0.3%)。按可辨识性分配:c_xy 有独立观测(τ_phys)→在线估;rz 在力矩通道上
        不可辨识(叉乘消掉)→弱先验;两者都不吃 m_est。

        rate 增益已在 attach 瞬间用先验缩放过(见 _grip_mass_step online 分支),
        这里只精修 model.p 的 c_est,不再动内环增益(避免随估计抖动反复改)。"""
        if self.grip_dropped:
            # box 已释放(drop),载荷没了:几何归零(不再随残噪更新)
            self.dJ_est = 0.0
            self.c_est = np.zeros(2)
            self._mp_ratchet = 0.0        # 棘轮随载荷一起卸掉
            self._mp_gain_applied = 0.0
            return
        self.c_est = self.c_xy_online.copy()
        # dJ 跟随 m_est:棘轮(只增不减)+ 硬上界。两个方向的保护都不能少,
        # 理由见 dj_track_mest 参数声明处搬过来的 07-28 撤销记录。
        if self.dj_track_mest and self.grip_mass_stepped:
            m_p_now = max(float(self.m_est) - p.m, 0.0)
            m_p_now = min(m_p_now, self.grip_mp_cap)        # 治"高估无界跟涨"
            if m_p_now > self._mp_ratchet:                  # 治"LIFT 段冻住"
                self._mp_ratchet = m_p_now
                self.dJ_est = self._dJ_from_mp(self._mp_ratchet)
                # 内环增益跟进,但要滞回:估计每涨一点就重设一次 PX4 参数既没
                # 意义又会刷服务调用(而且 PX4 会把它当真机参数落盘,见 headless
                # 脚本的持久化护栏)。只在棘轮相对上次缩放涨够 ratio 才动。
                _need = max(self.dj_gain_rescale_dmp,
                            self._mp_gain_applied * (self.dj_gain_rescale_ratio - 1.0))
                if self._mp_ratchet - self._mp_gain_applied >= _need:
                    self._mp_gain_applied = self._mp_ratchet
                    self._scale_px4_rate_gains(self.dJ_est, allow_rescale=True)

    def mhe_mass_cb(self, msg):
        # use_mhe=False 时:MHE 只当诊断,不把估计喂回 NMPC(m_est 保持固定)。
        # 用来隔离验证控制器本身能否扛住载荷,与 MHE 估计质量解耦。
        # control_mode='l1' 时同样旁路:流派 B 全程不消费质量估计(m_est 固定
        # 标称),变载荷全靠 d_lumped 补——这是 C.1 对照的定义本身。
        if not self.use_mhe or self.control_mode == 'l1':
            return
        m = float(msg.data)
        if math.isfinite(m):
            self.m_est = float(np.clip(m, mhe_p.m_min, mhe_p.m_max))

    def _build_ref_path_msg(self, r=1.0, w=0.3, z_hover=3.0,
                            n_samples=400, hover_time=2.0, ramp_time=4.0,
                            t_offset=0.0):
        # 从 hover_time+ramp_time 之后开始采样(振幅已经渐变完、alpha=1 的稳态
        # 轨迹),否则采样区间会覆盖 ramp-in 过程,画出的参考线会带渐变螺旋的
        # 痕迹,不是完整对称的形状——纯可视化 bug,不影响 NMPC 实际跟踪的参考。
        # 形状本身始终由 ref_fn 决定(不在这里传 r/dz),这样可视化跟 solve_nmpc
        # 实际用的参考永远一致,不用在两处分别维护——之前 straight 用这个方法
        # 自己的 r=1.0 默认值会把可视化画成 ±1m,跟实际飞的 ±2.0m 不一致,就是
        # 这类参数没对齐导致的。(形参 r 因此是留着的,方法体并不使用。)
        # 但 w 和 ramp_time 这里要用:w 定采样跨度 period=2π/w、ramp_time 定采样
        # 起点,调用方必须传实际生效的那一组,否则 fig8_w/grip_dyn_w 一改就画不满
        # 一圈。
        # t_offset:_grip_dyn_ref 内部按 t - self.grip_dyn_t0 算相位,t=0 是切
        # 到动态跟踪那一刻的 nmpc_time,不是节点启动时的 0——不传的话采样区间
        # 落在 grip_dyn_t0 之前,build_reference_figure8 会一直落在 t<hover_time
        # 分支,画出来的"8"字会退化成一个点。
        msg = Path()
        msg.header.frame_id = 'map'
        period = 2.0 * math.pi / w
        t0 = t_offset + hover_time + ramp_time
        for i in range(n_samples + 1):
            t = t0 + i * period / n_samples
            xref = self.ref_fn(t, z_hover=z_hover)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(xref[0])
            ps.pose.position.y = float(xref[1])
            ps.pose.position.z = float(xref[2])
            ps.pose.orientation.w = float(xref[6])
            ps.pose.orientation.x = float(xref[7])
            ps.pose.orientation.y = float(xref[8])
            ps.pose.orientation.z = float(xref[9])
            msg.poses.append(ps)
        return msg

    def _publish_paths(self):
        now = self.get_clock().now().to_msg()
        self.ref_path_msg.header.stamp = now
        self.ref_path_pub.publish(self.ref_path_msg)
        self.actual_path_msg.header.stamp = now
        self.actual_path_pub.publish(self.actual_path_msg)
        self.nmpc_traj_msg.header.stamp = now
        self.nmpc_traj_pub.publish(self.nmpc_traj_msg)

    def _append_actual_path(self, odom_msg):
        ps = PoseStamped()
        ps.header.stamp = odom_msg.header.stamp
        ps.header.frame_id = 'map'
        ps.pose = odom_msg.pose.pose
        self.actual_path_msg.poses.append(ps)
        if len(self.actual_path_msg.poses) > self.actual_path_max_len:
            self.actual_path_msg.poses.pop(0)

    def _set_nmpc_traj_msg(self, X_sol):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for i in range(X_sol.shape[1]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(X_sol[0, i])
            ps.pose.position.y = float(X_sol[1, i])
            ps.pose.position.z = float(X_sol[2, i])
            ps.pose.orientation.w = float(X_sol[6, i])
            ps.pose.orientation.x = float(X_sol[7, i])
            ps.pose.orientation.y = float(X_sol[8, i])
            ps.pose.orientation.z = float(X_sol[9, i])
            msg.poses.append(ps)
        self.nmpc_traj_msg = msg

    def pub_hover_pos(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.z_hover
        pose.pose.orientation.w = 1.0
        self.pos_pub.publish(pose)

    def pub_position_ref(self, xref):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(xref[0])
        pose.pose.position.y = float(xref[1])
        pose.pose.position.z = float(xref[2])
        pose.pose.orientation.w = float(xref[6])
        pose.pose.orientation.x = float(xref[7])
        pose.pose.orientation.y = float(xref[8])
        pose.pose.orientation.z = float(xref[9])
        self.pos_pub.publish(pose)

    def _seed_initial_guess(self, x_init, Xref_win):
        # 跟 acados_model.py 里 cost 用的 u_hover_dyn 同一个修复:这里也不能
        # 再用 p.u_hover(基于固定标定质量 p.m≈2.06kg 算出的 ~20.25N 常量)去
        # 初始化 warm-start 猜测。实测踩过的坑:质量从一开始就满载(3.5643kg,
        # 真实悬停推力~35N)起飞时,种子值跟真实解相差约 75%,SQP 经常在
        # 100 次迭代内卡进病态(alpha 长期很小、res_stat 不再下降、最终
        # solve failed),求解失败后代码 fallback 到"延用上一次的 T",导致
        # 观测到的推力长期停留在偏低的旧值附近,跟质量阶跃大小无关都会复现。
        u_hover_now = np.array([self.m_est * p.g, 0.0, 0.0, 0.0])
        for i in range(p.N):
            self.solver.set(i, 'x', Xref_win[:, i])
            self.solver.set(i, 'u', u_hover_now)
        self.solver.set(p.N, 'x', Xref_win[:, p.N])

    def solve_nmpc(self, x_cur, t_ref):
        Xref_win = self.ref_window_fn(t_ref, p.N, p.dt, z_hover=self.z_hover)

        # B.3 Phase2:online 几何接管后,每次 solve 前用最新 m_est+在线 c_xy
        # 刷新 dJ_est/c_est(替代 attach 真值一次性赋值)
        if self.geom_online_active and self.geom_source == 'online':
            self._update_online_geometry()

        self.solver.set(0, 'lbx', x_cur)
        self.solver.set(0, 'ubx', x_cur)
        geom_slot = self._geom_slot()
        for i in range(p.N + 1):
            self.solver.set(i, 'p', np.concatenate(
                [Xref_win[:, i], [self.m_est], geom_slot, self.d_lumped,
                 self.xi_lumped]))

        # t_start_ros:轨迹 t=0 对应的采样时刻(ROS clock),给高频回调算 τ 用。
        # t_start:墙钟,仅用于测 solve 耗时(solve_time),两者别混。
        t_start_ros = self.get_clock().now()
        t_start = time.time()
        status = self.solver.solve()

        # 诊断:KKT 残差和 SQP 迭代次数。假说是代价函数在某些状态下病态
        # (R 对力矩的惩罚远小于 Q 对位置的惩罚,Hessian 可能接近奇异),
        # 数值上应该表现为 res_stat(平稳性残差)在发散前就开始恶化、
        # sqp_iter 顶到 max_iter。不管成功失败都记录,方便看趋势。
        res_stat, res_eq, res_ineq, res_comp = self.solver.get_stats('residuals')
        sqp_iter = self.solver.get_stats('sqp_iter')
        self.max_res_stat = max(self.max_res_stat, float(res_stat))
        self.last_res_stat = float(res_stat)
        self.last_sqp_iter = int(sqp_iter)

        if res_stat > 10.0 or sqp_iter >= 95:
            self.get_logger().warn(
                f't={t_ref:.2f}s | KKT residual abnormal! res_stat={res_stat:.3e} '
                f'res_eq={res_eq:.3e} res_ineq={res_ineq:.3e} '
                f'res_comp={res_comp:.3e} | sqp_iter={sqp_iter} | '
                f'peak res_stat={self.max_res_stat:.3e}')

        if status == 0:
            self.solve_fail_count = 0
            u_opt = self.solver.get(0, 'u')
            # 取"提前一步的预测状态"里的角速度,不是 u_opt 里的力矩——跟
            # offboard_test/nmpc_node.py 里 omega_cmd = X_sol[10:13, 1] 的语义
            # 完全一致,不要"顺手"改成更直接的 u_opt 角速度通道。
            omega_cmd = self.solver.get(1, 'x')[10:13]
            X_sol = np.array(
                [self.solver.get(i, 'x') for i in range(p.N + 1)]).T
            U_sol = np.array(
                [self.solver.get(i, 'u') for i in range(p.N)]).T
            self._set_nmpc_traj_msg(X_sol)
            # 存整条未 ramp 的角速度/推力轨迹给高频发布定时器插值前推(路 A)。
            # 角速度取 X 的 [10:13] 分量(与 omega_cmd=X_sol[10:13,1] 同源),
            # 推力取每步 u 的第 0 分量。copy 防被后面 warm-start shift 改到。
            self._traj_omega = X_sol[10:13, :].copy()
            self._traj_thrust = U_sol[0, :].copy()

            # warm-start 向前滚动一步、末端复制,跟 offboard_test/nmpc_node.py
            # 里 X_init/U_init 的 shift 逻辑完全对应——这一步原来漏掉了:
            # acados 自己只会把"上一次第 i 阶段的解"留在第 i 阶段,不会自动按
            # 时间往前挪,放着不管的话每次给的初始猜测都系统性慢一拍,在持续
            # 移动的参考(圆形轨迹)上这个偏差会被放大,导致 SQP 收敛不了。
            for i in range(p.N):
                self.solver.set(i, 'x', X_sol[:, min(i + 1, p.N)])
                self.solver.set(i, 'u', U_sol[:, min(i + 1, p.N - 1)])
            self.solver.set(p.N, 'x', X_sol[:, p.N])

            self.last_u_opt = u_opt
            self.last_omega_cmd = omega_cmd
        else:
            # 求解失败没有可信轨迹,让高频发布退化成零阶保持(沿用下面算出的
            # last_u_opt/last_omega_cmd),不插值一条错的轨迹前推。
            self._traj_omega = None
            self._traj_thrust = None
            self.solve_fail_count += 1
            self.get_logger().warn(
                f'acados solve failed [status={status}, '
                f'{self.solve_fail_count} consecutive] | '
                f'x_cur pos={x_cur[0:3]} vel={x_cur[3:6]} '
                f'|q|={np.linalg.norm(x_cur[6:10]):.4f}')

            if self.solve_fail_count > self.max_consecutive_fail:
                # 连续失败太久,大概率已经飞出去了,求解器内部状态不可信,
                # 强制拉回安全的悬停猜测
                self.get_logger().warn(
                    f'Exceeded {self.max_consecutive_fail} consecutive failures, '
                    f'resetting warm-start to safe hover state')
                self._seed_initial_guess(x_cur, Xref_win)
                u_opt = p.u_hover.copy()
                omega_cmd = np.zeros(3)
            elif self.solve_fail_count == 1:
                # 只失败这一帧:不去扰动求解器内部状态,让它保留当前(哪怕没
                # 收敛)的迭代值当下一次起点,通常比强行假设"静止悬停"更接近
                # 真实解;指令延用上一次成功的结果,给它一帧机会自己恢复
                u_opt = self.last_u_opt.copy()
                omega_cmd = self.last_omega_cmd.copy()
            else:
                # 连续失败第 2 次起:推力可以继续延用上一次的值(安全,不会让
                # 飞机产生持续的姿态变化),但角速度必须归零——之前在这里也
                # 延用上一次的 omega_cmd,结果连续失败约 0.5~1 秒时飞机会
                # 带着同一个非零角速度持续旋转、完全没有
                # 刹车,几秒内就倾覆自由下坠摔机。归零角速度只是"停止主动
                # 旋转",不会像整体重置成悬停那样粗暴。
                u_opt = self.last_u_opt.copy()
                omega_cmd = np.zeros(3)

        # 轨迹的 t=0 对应 x_cur 采样时刻(≈t_start_ros),高频回调据此算经过时间 τ。
        # 用 ROS clock(t_start_ros)而非墙钟,与插值网格的仿真时间刻度对齐。
        self._traj_stamp = t_start_ros
        solve_time = (time.time() - t_start) * 1000
        return u_opt, omega_cmd, solve_time

    def _update_omega_scale(self):
        """更新 ω_cmd 缩放比例 Ĵ_t/J_a(每帧,一阶低通)。

        为什么低通:T_phys 本身有噪声,且**机动时 T_phys 含向心分量**(T_phys/g
        只在近悬停时才等于有效质量)。attach 的渐进承重过程是秒级的,而速率环是
        百 Hz 级——0.5s 的时间常数远慢于内环、又足够跟上承重,两边都不打架。
        没有低通的话,这就退化成"高频抖动的增益",正是要避免的那件事。

        只缩放 roll/pitch:dJ 只进 Jxx/Jyy(载荷挂在下方,绕 z 的惯量几乎不变),
        与 _scale_px4_rate_gains 只改 MC_ROLLRATE_K/MC_PITCHRATE_K 同理。
        """
        if not self.omega_scale_enable:
            return
        if self.omega_scale_source == 'thrust':
            if self._l1_T_phys is None:
                return                      # 还没收到转速:保持 1.0
            # 地面门控:与 L1 同一条件同一理由(地面支持力会让 T_phys 失去意义)
            if self._l1_T_phys < 0.4 * p.m * p.g:
                self.omega_scale = 1.0
                self._omega_scale_stamp = None
                return
            m_eff = self._l1_T_phys / p.g
        else:
            m_eff = self.m_est
        # 有效载荷 → 同一套几何代数 → 惯量比。空载时 m_p_eff=0 ⇒ target=1.0,
        # 自然退化不影响空机(FlyAware 的 K_k≈I3 是同一个边界条件)。
        m_p_eff = max(0.0, m_eff - p.m)
        dJ_eff = self._dJ_from_mp(m_p_eff)
        target = float(np.clip((p.Jxx + dJ_eff) / p.Jxx, 1.0, self.omega_scale_cap))

        t = self.get_clock().now().nanoseconds * 1e-9   # ROS clock,不用墙钟
        if self._omega_scale_stamp is None:
            self._omega_scale_stamp = t
            self.omega_scale = target       # 首帧直接对齐,不从 1.0 慢慢爬
            return
        dt = t - self._omega_scale_stamp
        self._omega_scale_stamp = t
        if not (0.0 < dt < 0.5):            # 时基跳变帧:不推进
            return
        alpha = 1.0 - np.exp(-dt / max(self.omega_scale_tau, 1e-3))
        self.omega_scale += alpha * (target - self.omega_scale)

    def publish_attitude(self, u_opt, omega_cmd):
        T = u_opt[0]
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 把 NMPC 求得的牛顿推力 T 反解成 PX4 归一化油门 setpoint,走上面那条物理
        # 映射的逆: ω_req = sqrt(T/THRUST_K), norm = (ω_req - OMEGA_MIN)/OMEGA_SPAN。
        # 任意质量工作点都对,不再像线性启发式只在满载点相切、空载漂低。T<=0 兜底。
        if T > 0.0:
            omega_req = float(np.sqrt(T / self.THRUST_K))
            norm = (omega_req - self.OMEGA_MIN) / self.OMEGA_SPAN
        else:
            norm = 0.0
        msg.thrust = float(np.clip(norm, 0.05, 0.95))
        wmax = self.omega_cmd_max
        w_cmd = np.asarray(omega_cmd, dtype=float).copy()
        if self.omega_scale_enable:
            self._update_omega_scale()
            if self.omega_scale > 1.0 + 1e-6:
                # ω_cmd' = ω + s·(ω_cmd − ω),只作用 roll/pitch
                w_now = (self.x_cur[10:13] if self.x_cur is not None
                         else np.zeros(3))
                w_cmd[0:2] = (w_now[0:2]
                              + self.omega_scale * (w_cmd[0:2] - w_now[0:2]))
                # 撞限幅 = 补偿被静默截断(与 xi_max 同一类坑,08-25 踩过)。
                # 只警告一次避免刷屏;真要用大比例得同步放宽 omega_cmd_max。
                if ((np.abs(w_cmd[0:2]) > wmax).any()
                        and not self._omega_scale_clip_warned):
                    self._omega_scale_clip_warned = True
                    self.get_logger().warn(
                        f'[omega_scale] 缩放后 body_rate 撞限幅 ±{wmax:.1f}rad/s '
                        f'(s={self.omega_scale:.2f}) — 补偿被截断,'
                        '考虑放宽 omega_cmd_max')
        # 诊断日志(2026-08-25):量 ω_cmd 的实际分布,给 omega_cmd_max 该放宽到
        # 多少提供依据。同时补上 omega_scale 缺的 s 周期日志。只打印不改逻辑;
        # 50Hz 下每 10 帧一行 ≈ 5Hz,飞 93s 约 465 行,grep 完能直接算分位数。
        self._wlog_count = getattr(self, '_wlog_count', 0) + 1
        if self._wlog_count % 10 == 0:
            _wn = (self.x_cur[10:13] if self.x_cur is not None else np.zeros(3))
            _raw = np.asarray(omega_cmd, dtype=float)
            self.get_logger().info(
                f'[wcmd] raw=[{_raw[0]:+.3f},{_raw[1]:+.3f},{_raw[2]:+.3f}] '
                f'w=[{_wn[0]:+.3f},{_wn[1]:+.3f},{_wn[2]:+.3f}] '
                f'err=[{_raw[0]-_wn[0]:+.3f},{_raw[1]-_wn[1]:+.3f}] '
                f'scaled=[{w_cmd[0]:+.3f},{w_cmd[1]:+.3f}] '
                f's={getattr(self, "omega_scale", 1.0):.2f}')
        msg.body_rate.x = float(np.clip(w_cmd[0], -wmax, wmax))
        msg.body_rate.y = float(np.clip(w_cmd[1], -wmax, wmax))
        msg.body_rate.z = float(np.clip(w_cmd[2], -wmax, wmax))
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        self.att_pub.publish(msg)

    def _publish_from_traj(self):
        """高频发布定时器(路 A):沿最近一次求解的预测轨迹插值前推,以 publish_hz
        发 body_rate + thrust。只在 NMPC 接管后生效——接管前的 position setpoint
        仍由 1/dt Hz 的 timer_cb 发。
        单线程 executor 下本回调与 timer_cb 串行执行,读 _traj_* 无并发,无需加锁。"""
        if not self.nmpc_started or self.nmpc_start_time is None:
            return
        nmpc_time = (self.get_clock().now()
                     - self.nmpc_start_time).nanoseconds / 1e9

        if self._traj_omega is not None:
            # 有可信轨迹:按自求解起点经过的时间 τ 沿轨迹插值。角速度沿用原来
            # "看前一步(lookahead=dt,omega_cmd 原取 X_sol[:,1])"的语义,叠加 τ
            # 平滑前推;推力沿用原来取当前步(lookahead=0,U_sol[:,0])的语义。
            # τ 正常在 [0, dt] 内,clamp 到网格末端防两次求解间隔异常拉长时越界。
            tau = (self.get_clock().now() - self._traj_stamp).nanoseconds / 1e9
            t_om = min(max(p.dt + tau, 0.0), self._omega_grid[-1])
            t_th = min(max(tau, 0.0), self._thrust_grid[-1])
            omega_cmd = np.array([
                np.interp(t_om, self._omega_grid, self._traj_omega[k])
                for k in range(3)])
            T = float(np.interp(t_th, self._thrust_grid, self._traj_thrust))
        else:
            # 求解失败(或还没有第一条轨迹):零阶保持最近一次成功的指令,与
            # solve_nmpc 失败分支延用 last_* 的语义一致。
            omega_cmd = self.last_omega_cmd.copy()
            T = float(self.last_u_opt[0])

        # 与 timer_cb 完全一致的 body_rate 接管斜坡(smoothstep),用实时 nmpc_time
        # 重算,让高频每一发都拿到当下正确的 ramp 系数。
        if nmpc_time < self.bodyrate_ramp_time:
            s = nmpc_time / self.bodyrate_ramp_time
            omega_cmd = omega_cmd * (10*s**3 - 15*s**4 + 6*s**5)

        # publish_attitude 只用 u_opt[0]=T 与 omega_cmd(body_rate 模式,力矩分量
        # 不进 setpoint),故只需填 T。
        self.publish_attitude(np.array([T, 0.0, 0.0, 0.0]), omega_cmd)

    def _trigger_payload_detach(self):
        # 发给 mass_changer 插件(gz_plugins/mass_changer/)的 /payload/drop_mass
        # topic——插件收到后会在物理引擎层面把 base_link 从 2.5kg 切回
        # 2.0kg,只切一次。用 Popen 而不是 run/check_call:这是在 NMPC 定时器
        # 回调(timer_cb)里发起的调用,回调线程绝不能被一次子进程同步阻塞——
        # gz CLI 首次冷启动的 gz-transport discovery 实测可能要将近 1 秒,
        # 真要等它跑完,这一帧就直接卡成了"飞机瞬间失去所有姿态指令"。这次
        # 不需要像旧的 attach 设计那样卡时机精度(质量切换是瞬时的组件覆写,
        # 不存在"discovery 慢了几百毫秒、被焊在错误位置"这类问题),所以可以
        # 放心用 Popen 发出去就不等。
        try:
            subprocess.Popen(
                ['gz', 'topic', '-t', '/payload/drop_mass',
                 '-m', 'gz.msgs.Empty', '-p', 'unused: true'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f'Failed to trigger payload drop: {e}')
        # 同帧广播质量突变事件给 mhe_node(事件触发权重调度)。注意 gz CLI 冷
        # 启动 discovery 可能比这条 ROS 消息慢几百毫秒——事件先到、物理后变,
        # MHE 侧多降权一两帧旧数据,无害(方向是保守的)。
        self.mass_event_pub.publish(Empty())

    def _drop_phase(self, nmpc_time):
        """NMPC 已经接管追踪之后,每帧都会被调用。追踪满 drop_after_track_sec
        秒后触发一次质量切换(往 /payload/drop_mass 发一条消息,mass_changer
        插件收到后把 base_link 从 2.5kg 切回 2.0kg),再等 drop_settle_sec 秒
        纯粹是为了在日志里标记一下这个阶段——质量切换是物理引擎里一次性的
        组件覆写,飞行中随时触发都没问题,不存在"卡时机"的顾虑。"""
        if self.drop_done or not self.payload_enabled:
            return
        if not self.drop_triggered:
            if nmpc_time < self.drop_after_track_sec:
                return
            self._trigger_payload_detach()
            self.drop_triggered = True
            self.drop_trigger_time = self.get_clock().now()
            self.get_logger().info(
                f't={nmpc_time:.1f}s | Drop triggered (mass switched to empty).')
            return

        settle_time = (self.get_clock().now() -
                        self.drop_trigger_time).nanoseconds / 1e9
        if settle_time < self.drop_settle_sec:
            return
        self.drop_done = True
        self.get_logger().info('Payload drop settled.')

    # ---------- 夹爪吊挂模式的固定悬停参考 + 抬升阶段 ----------
    def _grip_ref(self, t=0.0, **kw):
        """gripper 模式的参考:固定悬停在 (grip_x, grip_y, self.z_hover)、
        姿态水平、速度/角速度为零。self.z_hover 由 _lift_phase 随时间抬升。"""
        return np.concatenate([
            np.array([self.grip_x, self.grip_y, self.z_hover]),
            np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3)])

    def _grip_ref_window(self, t_start, N, dt, z_hover=None):
        return np.tile(self._grip_ref().reshape(-1, 1), (1, N + 1))

    # ---------- B.5 动态轨迹:lift 后带载荷飞 figure8 ----------
    def _grip_dyn_ref(self, t=0.0, **kw):
        xr = build_reference_figure8(
            t - self.grip_dyn_t0, r=self.grip_dyn_r, w=self.grip_dyn_w,
            z_hover=self.grip_z_high, dz=self.grip_dyn_dz,
            ramp_time=self.grip_dyn_ramp)
        xr[0] += self.grip_dyn_cx
        xr[1] += self.grip_dyn_cy
        return xr

    def _grip_dyn_ref_window(self, t_start, N, dt, z_hover=None):
        xref = build_reference_window_figure8(
            t_start - self.grip_dyn_t0, N, dt, r=self.grip_dyn_r,
            w=self.grip_dyn_w, z_hover=self.grip_z_high, dz=self.grip_dyn_dz,
            ramp_time=self.grip_dyn_ramp)
        xref[0, :] += self.grip_dyn_cx
        xref[1, :] += self.grip_dyn_cy
        return xref

    def _grip_dynamic_phase(self, nmpc_time):
        """B.5:lift 完成 + grip_dyn_settle_sec 秒后,从悬停切到 figure8 跟踪
        (偏移到切换瞬间 drone 的 xy,平滑过渡),带偏心载荷飞机动。之后 drop 落在
        figure8 中途。只切一次;grip_dynamic_after_lift=false 时禁用。"""
        if (not self.gripper_mode or not self.grip_dynamic_after_lift
                or self.grip_dynamic_active or not self.grip_lift_started
                or self.attach_time is None or self.x_cur is None):
            return
        lift_done_t = (self.attach_time + self.grip_lift_after_sec
                       + self.grip_lift_dur)
        if nmpc_time < lift_done_t + self.grip_dyn_settle_sec:
            return
        self.grip_dynamic_active = True
        self.grip_dyn_t0 = nmpc_time
        self.grip_dyn_cx = float(self.x_cur[0])
        self.grip_dyn_cy = float(self.x_cur[1])
        self.hover_test_mode = False   # 放开 t_ref,让 figure8 随时间推进
        self.ref_fn = self._grip_dyn_ref
        self.ref_window_fn = self._grip_dyn_ref_window
        # RViz 的 Reference Path 是 init 时用彼时的 ref_fn(_grip_ref,固定悬停点)
        # 建的一条退化短线,切到动态跟踪后必须用新 ref_fn 重建,否则 RViz 里一直
        # 显示旧的悬停参考,看不出真实在飞的 8 字(t_offset=grip_dyn_t0,见
        # _build_ref_path_msg 里的说明)。
        self.ref_path_msg = self._build_ref_path_msg(
            r=self.grip_dyn_r, w=self.grip_dyn_w, z_hover=self.grip_z_high,
            ramp_time=self.grip_dyn_ramp, t_offset=self.grip_dyn_t0)
        self.get_logger().info(
            f't={nmpc_time:.1f}s | DYNAMIC: switch to figure8 (r={self.grip_dyn_r} '
            f'w={self.grip_dyn_w}) centered at [{self.grip_dyn_cx:.2f},'
            f'{self.grip_dyn_cy:.2f}] z={self.grip_z_high}, tracking with payload')

    def _grip_mass_step(self, nmpc_time):
        """诊断:吸附后(grip_mass_step_sec)把 m_est 从空机手动阶跃到带载真值,
        并同时阶跃吊挂惯量增量 dJ_est 和复合质心水平偏移 c_est——给 NMPC 正确的
        "质量+惯量+质心"认知(和 MHE 解耦),用来隔离验证"建模是否足够",而不是
        "估计是否收敛"。几何优先用 proximity 发来的 attach 实测偏移 r_p(两次
        实测 dz=0.593/0.393 差异很大,写死参数不可靠),没收到才退回 grip_arm_d
        参数、且此时偏心只能按 0 算。grip_payload_mass<=0 则不做。

        物理:机体 m_b 与载荷 m_p 两点刚体系,复合质心 r_c=(m_p/m_t)*r_p;
        对复合质心的滚转/俯仰惯量增量按平行轴定理是约化质量 μ=m_b*m_p/m_t
        乘力臂平方(dJxx=μ*(ry²+rz²), dJyy=μ*(rx²+rz²),模型里是单标量 dJ,
        取两者均值);推力不过质心产生的常值力矩交给模型里的 c_sym 项。"""
        if not self.gripper_mode or self.grip_mass_stepped:
            return
        if self.grip_payload_mass <= 0.0:
            return
        # 方案(a):质量阶跃严格绑定真实 attach 事件,不再靠 nmpc_time 阈值猜测
        # (grip_mass_step_sec 在受控 attach 下已无意义)。必须先 descend 到位、
        # NMPC 主动发过 enable(见 _descend_phase),attach 才可能发生;然后等真实
        # attach_offset 到达才阶跃。超时兜底:发 enable 后 grip_mass_step_timeout_sec
        # 秒仍没收到 attach_offset(万一 attach 没成功),才退回 grip_arm_d 兜底几何。
        if not self.grip_descend_done or self.grip_descend_done_time is None:
            return
        if (self.attach_offset is None and nmpc_time <
                self.grip_descend_done_time + self.grip_mass_step_timeout_sec):
            return
        self.grip_mass_stepped = True
        self.attach_time = nmpc_time  # _lift_phase 从这个时刻起算,而非 NMPC 接管时刻
        # C.1 L1 模式(流派 B):attach 不做模型侧前馈——不阶跃 m_est、不给
        # dJ/c_est,平动变载荷全靠 L1 在 d_lumped 上在线补(对照定义,实施计划
        # §4)。attach_time/grip_mass_stepped 照常置位:lift 时序与
        # [attach-window] 日志是评估口径,与控制路线无关,两个流派必须一致。
        # **D2(b) 拍板(2026-07-23,用户)**:允许用**档位操作先验**缩内环增益
        # ——纯平动版实测 0.3kg/ecc0.10 姿态失稳(x/y d̂ 大幅乱跳=07-02 内环
        # 带宽问题裸奔,grip_nmpc_163705);增益缩放用与主方法 online 模式同款
        # 的载荷包线上界(机架规格,非测量真值),不破坏"不知精确质量"设定,对照
        # 更公平(否则是绑着 baseline 的手打,审稿必疑)。drop 复位走公共路径
        # (_reset_px4_rate_gains 已在 _grip_drop_phase)。
        if self.control_mode == 'l1':
            mp0, src0 = self._envelope_mp()  # l1 分支本来就只做增益、不做模型前馈
            dJ0 = self._dJ_from_mp(mp0)
            self._scale_px4_rate_gains(dJ0)
            self.get_logger().info(
                f't={nmpc_time:.1f}s | ATTACH (control_mode=l1): rate gains from '
                f'{src0} m_p={mp0:.2f}kg (dJ={dJ0:.4f}); no model-side feedforward,'
                f' translational payload handled by L1 d_lumped only')
            return
        m_p = self.grip_payload_mass
        m_t = p.m + m_p
        # 弱闭环(use_mhe=True)时 m 全程只来自 MHE,这里不许塞真值——哪怕
        # 一个周期也会污染"MHE 收敛前暂态扛不扛得住"的归因;dJ/c/gain 照常
        # 前馈(它们本来就按已知 payload 走几何路径,与 MHE 无关)。
        if not self.use_mhe:
            self.m_est = m_t
        # online 模式(B.3 Phase2):不从 attach 真值算几何,交给
        # _update_online_geometry 用 m_est+在线 c_xy+rz 先验在线推;attach_offset
        # 仅留作评估。dJ/c_est/gain 都在那里随 m_est 收敛起来。
        if self.geom_source == 'online':
            self.geom_online_active = True
            # attach 瞬间用**包线上界**(grip_payload_envelope + grip_arm_d)立刻
            # 初始化 dJ 与 rate 增益(给内环即时鲁棒性,不等估计收敛——否则实测
            # 炸机);c_est 先 0,随后 _update_online_geometry 用在线 m_est/c_xy 精修。
            mp0, _ = self._envelope_mp()
            self.dJ_est = self._dJ_from_mp(mp0)      # 模型侧:可以是 0/给错
            self.c_est = np.zeros(2)
            mp_gain, src_gain = self._envelope_mp()      # 执行器侧:必须够大
            self._scale_px4_rate_gains(self._dJ_from_mp(mp_gain))
            self.get_logger().info(
                f't={nmpc_time:.1f}s | ATTACH (geom_source=online): model dJ from '
                f'envelope m_p={mp0:.2f}kg arm={self.grip_arm_d:.2f}m '
                f'(dJ={self.dJ_est:.4f}); rate gains from {src_gain} '
                f'm_p={mp_gain:.2f}kg; c_xy refine online, attach truth eval-only')
            return
        if self.attach_offset is not None:
            r_p = self.attach_offset
        else:
            r_p = np.array([0.0, 0.0, -self.grip_arm_d])
            self.get_logger().warn(
                'no attach offset received, falling back to grip_arm_d '
                f'{self.grip_arm_d:.2f}m with zero lateral offset')
        self.dJ_est, self.c_est = self._payload_geometry(r_p)
        self._scale_px4_rate_gains(self.dJ_est)  # 兜底:没收到 attach offset 时这里补
        tau_hover_roll = abs(self.c_est[1]) * m_t * p.g
        self.get_logger().info(
            f't={nmpc_time:.1f}s | MASS+INERTIA+COM STEP: m_est {p.m:.2f}->'
            f'{m_t:.2f}kg, dJ 0->{self.dJ_est:.4f} kg·m², '
            f'c_xy=[{self.c_est[0]*100:+.2f},{self.c_est[1]*100:+.2f}]cm '
            f'(payload {m_p:.2f}kg @ r_p=[{r_p[0]:+.3f},{r_p[1]:+.3f},'
            f'{r_p[2]:+.3f}]m, hover roll tau≈{tau_hover_roll:.3f}Nm '
            f'of tau_max {p.tau_max}, '
            f'{"m from MHE (no truth inject)" if self.use_mhe else "MHE bypass"})')

    def _geom_slot(self):
        """model.p 里几何那 3 个槛位该装什么(见 acados_params.geom_coupled)。
        legacy: [dJ_est, cx_est, cy_est](窗外算好的常量,与 m_est 无函数关系)。
        coupled: [rx, ry, rz] 载荷几何偏移,J/c 由模型内部按 m_est 现算——这一档
        NMPC 与 MHE 用**同一套**代数,两个求解器对同一质量给出同一姿态动力学。
        载荷不在机上(未 attach / 已 drop)时装全零,退化回空机 J、c=0。"""
        if not p.geom_coupled:
            return np.concatenate([[self.dJ_est], self.c_est])
        if not self.gripper_mode or not self.grip_mass_stepped:
            return np.zeros(3)
        # 'self' 释放模式:**不因 drop 清零**,让 c(m)/J(m) 随 m_est 自行熄灭
        if self.grip_dropped and self.geom_release_mode != 'self':
            return np.zeros(3)
        if self.attach_offset is not None:
            return np.asarray(self.attach_offset, dtype=float)
        # 没收到 attach 几何:退回力臂参数 + 零偏心(与 legacy 兜底同一口径)
        return np.array([0.0, 0.0, -self.grip_arm_d])

    def _payload_geometry(self, r_p):
        """由载荷相对机体的几何偏移 r_p=[rx,ry,rz](rz<0)算 (dJ, c_xy)。
        机体 m_b 与载荷 m_p 两点刚体系:复合质心 r_c=(m_p/m_t)*r_p;对复合
        质心的滚转/俯仰惯量增量按平行轴定理是约化质量 μ=m_b*m_p/m_t 乘力臂
        平方(dJxx=μ*(ry²+rz²), dJyy=μ*(rx²+rz²),模型里是单标量 dJ,取均值)。"""
        m_p = self.grip_payload_mass
        m_t = p.m + m_p
        mu = p.m * m_p / m_t
        dJ = mu * (r_p[2] ** 2 + 0.5 * (r_p[0] ** 2 + r_p[1] ** 2))
        c = (m_p / m_t) * np.asarray(r_p[0:2], dtype=float)
        return float(dJ), c

    # ---------- PX4 内环速率增益随惯量同步 ----------
    def _request_rate_gain_bases(self):
        """异步读一次 MC_*RATE_K 的基准值(机架配置可能不是默认 1.0),attach
        时在基准上乘惯量比。服务没就绪前每帧重试,发出后不再重复。"""
        if self.px4_base_req_sent or not self.param_get_client.service_is_ready():
            return
        self.px4_base_req_sent = True
        for pid in ('MC_ROLLRATE_K', 'MC_PITCHRATE_K'):
            fut = self.param_get_client.call_async(
                ParamGet.Request(param_id=pid))
            fut.add_done_callback(
                lambda f, pid=pid: self._store_rate_gain_base(pid, f))

    def _store_rate_gain_base(self, pid, fut):
        try:
            res = fut.result()
        except Exception as e:
            self.get_logger().warn(f'ParamGet {pid} failed: {e}')
            return
        if res.success:
            self.px4_rate_k_base[pid] = float(res.value.real)
            self.get_logger().info(
                f'PX4 {pid} base = {self.px4_rate_k_base[pid]:.3f}')
        else:
            self.get_logger().warn(f'ParamGet {pid} unsuccessful, '
                                   f'will fall back to base 1.0')

    def _dJ_from_mp(self, m_p):
        """由载荷质量先验算吊挂惯量增量 dJ = μ·d²(μ=约化质量 m_b·m_p/m_t)。
        原先这段代数在 _grip_mass_step 的两个分支里各抄了一遍,拆 prior 时提出来
        —— 模型侧和增益侧现在要用**不同的** m_p 各算一次。"""
        m_t = p.m + m_p
        mu = p.m * m_p / m_t if m_t > 0 else 0.0
        return float(mu * self.grip_arm_d ** 2)

    def _envelope_mp(self):
        """内环增益缩放 / 模型侧 dJ 初值该用的载荷质量 = **包线上界**(机架规格)。
        2026-08-26 去先验改造后这是唯一来源:原先的三级优先(gain_envelope →
        gain_prior → payload_prior)里后两级都是任务信息,已删。
        返回 (m_p, 来源标签),标签只用于日志/论文溯源。"""
        return self.grip_payload_envelope, 'envelope(rack spec)'

    def _scale_px4_rate_gains(self, dJ, allow_rescale=False):
        """把 PX4 速率环总增益按惯量比 (J+dJ)/J 放大,恢复被载荷惯量压塌的
        内环带宽(根因见 scale_px4_rate_gains 参数声明处注释)。只做一次;
        ratio 上限 5 防止 dJ 异常大时把内环推成高频震荡。"""
        if not self.scale_px4_rate_gains:
            return
        # allow_rescale=True 是 dj_track_mest 那条路专用:棘轮涨上去时要能把
        # 增益跟着调大。默认 False 保持"只做一次"的历史行为。
        if self.px4_gains_scaled and not allow_rescale:
            return
        self.px4_gains_scaled = True
        for pid, J0 in (('MC_ROLLRATE_K', p.Jxx), ('MC_PITCHRATE_K', p.Jyy)):
            ratio = min((J0 + dJ) / J0, 5.0)
            base = self.px4_rate_k_base.get(pid, 1.0)
            req = ParamSetV2.Request()
            req.force_set = False
            req.param_id = pid
            req.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(base * ratio))
            self.param_set_client.call_async(req)
            self.get_logger().info(
                f'PX4 {pid}: {base:.3f} -> {base * ratio:.3f} '
                f'(inertia ratio {(J0 + dJ) / J0:.2f}, cap 5.0)')

    def _reset_px4_rate_gains(self):
        """把 MC_*RATE_K 复位到 base(drop 释放载荷后调):载荷卸掉、真实惯量
        回到空机名义值,若增益还停在放大过的值(5x)就是严重过增益,空机内环
        会高频振荡→姿态发散→掉高炸机(2026-07-14 drop 全流程实测)。复位并把
        px4_gains_scaled 清回 False(万一后续再 attach 可重新放大)。"""
        if not self.scale_px4_rate_gains or not self.px4_gains_scaled:
            return
        self.px4_gains_scaled = False
        for pid in ('MC_ROLLRATE_K', 'MC_PITCHRATE_K'):
            base = self.px4_rate_k_base.get(pid, 1.0)
            req = ParamSetV2.Request()
            req.force_set = False
            req.param_id = pid
            req.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(base))
            self.param_set_client.call_async(req)
            self.get_logger().info(f'PX4 {pid}: reset -> {base:.3f} (payload dropped)')

    def _descend_phase(self, nmpc_time):
        """07-07 新增:NMPC 在安全高度 grip_approach_z 接管后,先稳定悬停
        grip_settle_sec 秒(给 MHE 滑动窗口热身——见 grip_settle_sec 参数声明
        处注释),稳定期满再平滑下降到 grip_z_low。斜坡完成后继续保持低位,
        等真实高度和速度也收敛后才触发 attach;避免"指令到位但机体滞后"
        时过早吸附,造成系统性深挂。"""
        if not self.gripper_mode or self.grip_descend_done:
            return
        if nmpc_time < self.grip_settle_sec:
            return
        s = min(max((nmpc_time - self.grip_settle_sec) /
                    self.grip_descend_dur, 0.0), 1.0)
        smooth = 10*s**3 - 15*s**4 + 6*s**5
        self.z_hover = self.grip_approach_z + \
            (self.grip_z_low - self.grip_approach_z) * smooth
        if not self.grip_descend_started:
            self.grip_descend_started = True
            self.get_logger().info(
                f't={nmpc_time:.1f}s | DESCEND: settle done, lowering '
                f'{self.grip_approach_z}->{self.grip_z_low}m, will enable '
                f'gripper at bottom')
        z_err = float(abs(self.x_cur[2] - self.grip_z_low))
        v_norm = float(np.linalg.norm(self.x_cur[3:6]))
        if (s >= 1.0 and not self.grip_enable_sent
                and z_err < self.grip_descend_z_tol
                and v_norm < self.grip_descend_v_tol):
            # 下降到位:主动发 enable,proximity 现在才判定几何 -> attach 在此刻
            # 发生(而非爬升途中)。这是方案(a)的核心受控事件时刻。
            self.grip_descend_done = True
            self.grip_descend_done_time = nmpc_time
            self.grip_enable_sent = True
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info(
                f't={nmpc_time:.1f}s | DESCEND done at '
                f'z={self.x_cur[2]:.3f}m (target {self.grip_z_low:.3f}m, '
                f'z_err={z_err:.3f}m, v={v_norm:.2f}m/s), gripper ENABLED '
                f'(attach will trigger now)')

    def _lift_phase(self, nmpc_time):
        """attach 真正发生(_grip_mass_step 记下 self.attach_time)满
        grip_lift_after_sec 秒后,把 z_hover 从 grip_z_low 平滑抬到
        grip_z_high,把 box 吊离地面,让 MHE 看到全额 +载荷质量。07-07 改:
        计时基准从"NMPC 接管时刻(nmpc_time=0)"改成"attach 真正发生的时刻
        (self.attach_time)"——NMPC 现在提前在安全高度接管,nmpc_time=0 早于
        attach 好几秒,不能再假设两者同时。attach_time 为 None(还没 attach)
        时不触发,不会提前把 box 抬空。"""
        if (not self.gripper_mode or self.attach_time is None
                or nmpc_time < self.attach_time + self.grip_lift_after_sec):
            return
        # LIFT 中途 hold(见 grip_lift_hold_enable 注释):抬到 hold_dz 就把斜坡
        # 时间冻结 hold_sec 秒。冻结用"从经过时间里扣掉已冻结时长"实现,而不是
        # 记住 s 再恢复 —— 后者在 hold 结束时会让 smoothstep 的导数跳变。
        t_eff = (nmpc_time - self.attach_time - self.grip_lift_after_sec
                 - self._lift_frozen_sec)
        if self.grip_lift_hold_enable and not self._lift_hold_done:
            if self._lift_hold_start is not None:
                # 冻结中:把新流逝的时间全部记进 frozen,t_eff 原地不动
                held = nmpc_time - self._lift_hold_start
                if held >= self.grip_lift_hold_sec:
                    self._lift_hold_done = True
                    self._lift_frozen_sec += held
                    self.get_logger().info(
                        f't={nmpc_time:.1f}s | LIFT hold done ({held:.1f}s), '
                        f'resuming climb | m_est={self.m_est:.3f} '
                        f'dJ={self.dJ_est:.4f} mp_ratchet={self._mp_ratchet:.3f}')
                else:
                    t_eff = (self._lift_hold_start - self.attach_time
                             - self.grip_lift_after_sec - self._lift_frozen_sec)
            else:
                s_now = min(max(t_eff / self.grip_lift_dur, 0.0), 1.0)
                sm_now = 10*s_now**3 - 15*s_now**4 + 6*s_now**5
                dz_now = (self.grip_z_high - self.grip_z_low) * sm_now
                if dz_now >= self.grip_lift_hold_dz:
                    self._lift_hold_start = nmpc_time
                    self.get_logger().info(
                        f't={nmpc_time:.1f}s | LIFT hold START at '
                        f'dz={dz_now:.3f}m (z={self.z_hover:.2f}m) for '
                        f'{self.grip_lift_hold_sec:.1f}s — payload is airborne, '
                        f'letting MHE converge | m_est={self.m_est:.3f}')
        s = min(max(t_eff / self.grip_lift_dur, 0.0), 1.0)
        smooth = 10*s**3 - 15*s**4 + 6*s**5
        self.z_hover = self.grip_z_low + \
            (self.grip_z_high - self.grip_z_low) * smooth
        if not self.grip_lift_started:
            self.grip_lift_started = True
            self.get_logger().info(
                f't={nmpc_time:.1f}s | LIFT: raising hover '
                f'{self.grip_z_low}->{self.grip_z_high}m to lift payload')

    def _grip_drop_phase(self, nmpc_time):
        """B.3 drop 全流程(2026-07-14):lift 完成后悬停 grip_drop_after_sec 秒,
        释放磁吸夹爪(发 /gripper/enable=false,proximity 节点检到 enable 拉低即
        detach box),同帧发 mass_event 通知 mhe_node 质量突变。online 模式下标记
        grip_dropped,_update_online_geometry 随即把 dJ/c_est 归零(载荷已卸)。
        只触发一次;grip_drop_after_sec<=0 禁用。"""
        if (not self.gripper_mode or self.grip_drop_done
                or self.grip_drop_after_sec <= 0.0 or self.attach_time is None):
            return
        t_drop = (self.attach_time + self.grip_lift_after_sec
                  + self.grip_lift_dur + self.grip_drop_after_sec)
        if nmpc_time < t_drop:
            return
        # 动态模式 + drop_at_fig8_tip:t_drop 只作"最早可 drop"的门,真正 drop
        # 精确落在 figure8 最左端(x=r·sin(a) 最小 → a=3π/2 mod 2π)。
        # 窗口宽度必须跟着 w 走:相位每帧进 w·dt,窗口窄于一帧就会整圈整圈地
        # 漏过尖点、drop 永不触发。原来写死的 0.12rad 是按 w=0.25(每帧 0.025rad,
        # ~5 帧余量)标的,w 一提到 0.7 每帧就进 0.07rad、只剩 1 帧余量,再快直接
        # 漏。取 3 帧宽度保底(3·w·dt),低速时 0.12 仍占优、行为不变。
        if self.grip_drop_at_fig8_tip and self.grip_dynamic_active:
            tc = nmpc_time - self.grip_dyn_t0 - 2.0  # 2.0 = figure8 hover_time
            if tc <= 0.0:
                return
            a_mod = (self.grip_dyn_w * tc) % (2.0 * np.pi)
            tip_win = max(0.12, 3.0 * self.grip_dyn_w * p.dt)
            if not (1.5 * np.pi <= a_mod < 1.5 * np.pi + tip_win):
                return
        self.grip_drop_done = True
        self.grip_dropped = True
        self.enable_pub.publish(Bool(data=False))  # 拉低 → proximity 释放 box
        # --- 是否把 drop 告诉 MHE(2026-08-25)---
        # False = **不发**:MHE 必须自己从 T_phys 残差看出载荷没了。drop 是外部
        # 事件信号,与 MHE 先验/几何先验同属"不该给估计器的信息",去掉它才和
        # 全仓"传感最小化"的主线一致(阶段 A.2 已证明事件收益不依赖外部信号)。
        # ⚠️ 关掉它必须同时满足两个前提,否则 MHE 会永远挂着幽灵载荷几何:
        #    ① mhe_node 的 event_signal_mode='residual'(自检测代替外部信号);
        #    ② mhe_node 的 resid_release_geom=True(自检测到"推力下降型"事件时
        #       自行释放几何)—— 2026-08-25 新增,在那之前残差这条路只降权、
        #       不释放几何,关掉信号就会留下幽灵载荷。
        # ⚠️ 注意 NMPC **自己**清几何/复位增益是合法的:夹爪是它松开的,它当然
        #    知道货没了。不合法的是把这个消息塞给估计器。两者别混为一谈。
        # 默认 True = 历史行为逐字节不变。
        if self.drop_publish_mass_event:
            self.mass_event_pub.publish(Empty())   # 通知 mhe_node 质量突变
        else:
            self.get_logger().info(
                't={:.1f}s | DROP: mass_event **未发布**(drop_publish_mass_event'
                '=false) — MHE 须自行从 T_phys 残差检测'.format(nmpc_time))
        self._reset_px4_rate_gains()               # 空机复位内环增益(否则过增益炸机)
        if self.control_mode == 'l1':
            # 掉包后真实 lumped 扰动阶跃回零,残留 d̂_f 会把推力拽偏(等价于
            # 带着幽灵载荷飞)——与 rate 增益复位同一类必需处理。复位后 L1 从
            # 零重新估计(若有残余未建模效应会自行爬回)。
            self.l1.reset()
            self.d_lumped = np.zeros(3)
        if self.tau_lumped_enable:
            # 同理:掉包后真实转动扰动阶跃回零,残留 xi 会持续给一个幽灵角加速度
            # 偏置,NMPC 会拿姿态去抵消它。
            self.l1_rot.reset()
            self.xi_lumped = np.zeros(3)
            self._l1_rot_last_stamp = None
        self.get_logger().info(
            f't={nmpc_time:.1f}s | DROP: released gripper (enable=false), '
            'payload detached; geometry -> empty, rate gains reset')

    def timer_cb(self):
        if self.counter < 100:
            self.pub_hover_pos()
            self.counter += 1
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        t_elapsed = (self.get_clock().now() -
                     self.start_time).nanoseconds / 1e9

        if not self.state.connected:
            self.pub_hover_pos()
            return

        if self.gripper_mode:
            self._request_rate_gain_bases()

        if t_elapsed < self.ekf_wait_sec:
            self.pub_hover_pos()
            self.counter += 1
            if self.counter % 100 == 0:
                self.get_logger().info(
                    f'Waiting for EKF2 convergence... '
                    f'{t_elapsed:.0f}/{self.ekf_wait_sec:.0f} s')
            return

        now = time.time()

        if self.state.mode != 'OFFBOARD':
            self.pub_hover_pos()
            if (self.mode_client.service_is_ready()
                    and now - self.last_mode_req_time > 1.0):
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.mode_client.call_async(req)
                self.last_mode_req_time = now
                self.get_logger().info('Switching to OFFBOARD mode')
            return

        if not self.state.armed:
            self.armed_and_flying = False
            self.nmpc_started = False
            self.nmpc_start_time = None
            self.pub_hover_pos()
            if (self.arming_client.service_is_ready()
                    and now - self.last_arm_req_time > 1.0):
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.last_arm_req_time = now
                self.get_logger().info('Sending arm command')
            return

        if not self.armed_and_flying:
            self.armed_and_flying = True
            self.get_logger().info('Armed, preparing to move to trajectory start...')

        if self.x_cur is None:
            self.pub_hover_pos()
            return

        start_ref = self.ref_fn(0.0)
        err_xy = float(np.linalg.norm(self.x_cur[0:2] - start_ref[0:2]))
        err_z  = float(abs(self.x_cur[2] - start_ref[2]))
        v_norm = float(np.linalg.norm(self.x_cur[3:6]))
        # gripper 接近第0段(2026-07-08 加):先在起飞点(spawn≈局部原点)原地
        # 垂直爬到 grip_approach_z 安全高度,再进入第一段水平平移。原来直接给
        # 一个 3D setpoint (grip_x,grip_y,grip_approach_z),PX4 会同时水平+垂直
        # 运动,低空平移那段会撞上地面 box——以前靠 proximity 在撞点把 box 吸走
        # 才没事;方案(a)不提前吸,box 成了实体障碍,空载无人机低空平移到 box
        # 正上方时卡在它上面爬不起来(实测世界系 z≈0)。拆成"先垂直爬升、再水平
        # 平移"就避开了,这也正是下面第一段注释本来声称的意图。
        if (self.gripper_mode and not self.nmpc_started
                and not self.grip_climbed):
            climb_ref = start_ref.copy()
            climb_ref[0] = 0.0   # spawn xy(远离 box),原地垂直爬升
            climb_ref[1] = 0.0
            climb_ref[2] = self.grip_approach_z
            self.pub_position_ref(climb_ref)
            self.counter += 1
            if self.counter % 50 == 0:
                self.get_logger().info(
                    f'Gripper approach (climb): rising at spawn to '
                    f'z={self.grip_approach_z:.2f}m | z={self.x_cur[2]:.2f}m '
                    f'v={v_norm:.2f}m/s')
            if (abs(self.x_cur[2] - self.grip_approach_z) < 0.20
                    and v_norm < 0.30):
                self.grip_climbed = True
                self.get_logger().info(
                    'Gripper approach: climbed to safe altitude at spawn, '
                    'now translating over box')
            return

        # gripper 两段式接近第一段:先飞到 box 正上方 grip_approach_z 的安全高度
        # 并把水平位置对齐、悬停稳,再让下面的常规逻辑把目标切到 grip_z_low、
        # 垂直下降过去。这样低空水平平移(会高度下冲)发生在远高于 box 的高度,
        # 起落架不会在下冲时顶到 box;垂直下降段没有水平速度,也不会撞。
        # 水平对齐要收得比 proximity 的 r_xy(0.15)更紧,保证垂直下降真的落在
        # 吸附窗口内。
        if (self.gripper_mode and not self.nmpc_started
                and not self.grip_high_aligned):
            high_ref = start_ref.copy()
            high_ref[2] = self.grip_approach_z
            self.pub_position_ref(high_ref)
            hi_xy = float(np.linalg.norm(self.x_cur[0:2] - high_ref[0:2]))
            hi_z = float(abs(self.x_cur[2] - high_ref[2]))
            self.counter += 1
            if self.counter % 50 == 0:
                self.get_logger().info(
                    f'Gripper approach (high): aligning over box at '
                    f'z={self.grip_approach_z:.2f}m | xy={hi_xy:.3f}m '
                    f'z_err={hi_z:.3f}m v={v_norm:.2f}m/s')
            if hi_xy < 0.10 and hi_z < 0.20 and v_norm < 0.25:
                self.grip_high_aligned = True
                self.get_logger().info(
                    'Gripper approach: aligned above box, NMPC will take '
                    f'over here (z={self.grip_approach_z:.2f}m), settle '
                    f'{self.grip_settle_sec:.1f}s then descend to '
                    f'z={self.grip_z_low:.2f}m to trigger attach')
            return

        if not self.nmpc_started:
            self.pub_position_ref(start_ref)
            self.counter += 1
            if self.counter % 50 == 0:
                self.get_logger().info(
                    f'Moving to trajectory start... xy={err_xy:.3f}m '
                    f'z={err_z:.3f}m v={v_norm:.2f}m/s')
            if (err_xy > self.start_xy_threshold
                    or err_z > self.start_z_threshold
                    or v_norm > 0.25):
                return

            self.nmpc_started = True
            self.nmpc_start_time = self.get_clock().now()
            self._seed_initial_guess(
                start_ref, np.tile(start_ref.reshape(-1, 1), (1, p.N + 1)))
            self.get_logger().info('Reached trajectory start, NMPC tracking begins!')

        nmpc_time = (self.get_clock().now() -
                     self.nmpc_start_time).nanoseconds / 1e9
        self._drop_phase(nmpc_time)
        self._descend_phase(nmpc_time)
        self._grip_mass_step(nmpc_time)
        self._lift_phase(nmpc_time)
        self._grip_dynamic_phase(nmpc_time)
        self._grip_drop_phase(nmpc_time)
        t_ref = 0.0 if self.hover_test_mode else nmpc_time

        u_opt, omega_cmd, solve_time = self.solve_nmpc(self.x_cur, t_ref)
        if nmpc_time < self.bodyrate_ramp_time:
            s = nmpc_time / self.bodyrate_ramp_time
            ramp = 10*s**3 - 15*s**4 + 6*s**5
            omega_cmd = omega_cmd * ramp
        # decouple 模式下 body_rate/thrust 由高频 _publish_from_traj 定时器发,这里
        # 不再由 NMPC 定时器直接发布。
        if not self.decouple_publish:
            self.publish_attitude(u_opt, omega_cmd)
        # u_opt 以 NMPC 求解频率发布；MHE 保持自己的 10Hz 定时器并消费最新值。
        # decouple 下高频实际施加的推力是沿预测轨迹的插值值,稳态与此相等。
        # 2026-07-16/17 SITL A/B(attach+lift,0.3kg/ecc0.05,on n=8 vs off n=8)实测:
        # 稳态 rms 中位数 on 0.017m vs off 0.016m——**两者无差别,上面这句"稳态相等"
        # 是对的**。塌陷率 on 12%(1/8) vs off 38%(3/8) 方向上 on 略优,但 Fisher
        # p=0.285 **不显著**,且 on 自己也塌——即"解耦发布有效"目前**没有证据支持**,
        # 别拿它当卖点。当时观察到的"off 稳态时好时坏(0.014~0.59m)"根因不在发布
        # 频率,而是 MHE 的 m_est 在 attach 后有概率锁死在错值(见 mhe_node.py 的
        # grip_geom_mp_floor 注释),已由该 floor 修掉(修后 0/8)。
        # 教训:该 A/B 前三次判读(n=1/n=5)全部被小样本误导且每次反转,SITL 单工况
        # 方差大,n<8 不要下结论。
        self.u_opt_pub.publish(Float64MultiArray(data=[float(v) for v in u_opt]))

        xref_now = self.ref_fn(t_ref)
        pos_err = np.linalg.norm(self.x_cur[0:3] - xref_now[0:3])
        self.tracking_err_pub.publish(Float64(data=float(pos_err)))

        # drop 事件段逐帧记录(平时 50 帧一条太粗,量不出暂态峰——2026-07-03
        # 三方对比曾因 5s 采样的相位伪影误报控制层收益,坐实必须逐帧)
        if self.drop_trigger_time is not None:
            _dt_ev = (self.get_clock().now()
                      - self.drop_trigger_time).nanoseconds / 1e9
            if _dt_ev < 8.0:
                self.get_logger().info(
                    f'[drop-window] t={nmpc_time:.2f}s pos_err={pos_err:.3f}m '
                    f'T={u_opt[0]:.2f}N z={self.x_cur[2]:.3f} '
                    f'm_est={self.m_est:.3f}')

        # gripper 版事件段逐帧记录:门槛用 self.attach_time(_grip_mass_step 记
        # 下的 nmpc_time 浮点秒,不是 ROS Time,不用转纳秒)。字段与 [drop-window]
        # 完全一致,parse_dropwindow_logs.py 用同一份正则解析两种 tag。窗口时长
        # 用 attach_window_sec 参数(默认 40s),不复用 [drop-window] 的 8.0s——
        # gripper attach 瞬态更长。供离线分析脚本(aggregate_b4_matrix.py /
        # aggregate_alpha_only.py)的控制层指标解析用。
        # (原文写"供 CEM 学习脚本 grip_cem_optimize.py 用",该脚本已于 2026-07-31
        #  随 CEM 删除;这段 [attach-window] 日志本身仍是所有 A/B 分析的数据源。)
        if self.gripper_mode and self.attach_time is not None:
            _dt_ev = nmpc_time - self.attach_time
            if 0.0 <= _dt_ev < self.attach_window_sec:
                self.get_logger().info(
                    f'[attach-window] t={nmpc_time:.2f}s pos_err={pos_err:.3f}m '
                    f'T={u_opt[0]:.2f}N z={self.x_cur[2]:.3f} '
                    f'm_est={self.m_est:.3f}')

        self.counter += 1
        # 保持约每 5 秒一条摘要日志，不让频率变化悄悄改变日志密度。
        if self.counter % max(1, round(5.0 / p.dt)) == 0:
            self.get_logger().info(
                f't={nmpc_time:.1f}s | pos_err={pos_err:.3f}m | '
                f'T={u_opt[0]:.2f}N | tau=[r{u_opt[1]:.3f} p{u_opt[2]:.3f} '
                f'y{u_opt[3]:.3f}]Nm | '
                f'res_stat={self.last_res_stat:.3e}(peak {self.max_res_stat:.3e}) '
                f'sqp_iter={self.last_sqp_iter} | solve={solve_time:.1f}ms')


def main():
    rclpy.init()
    node = AcadosNMPCNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
