#!/usr/bin/env python3
# acados 版 NMPC 节点。状态机(预热→等EKF→切OFFBOARD→解锁→飞到起点→NMPC接管)
# 跟 offboard_test/nmpc_node.py 的 NMPCNode 完全一致,直接照搬,只有 solve_nmpc()
# 内部换成调 acados 求解器,其它行为(包括坐标系、限幅、推力归一化、body_rate
# 斜坡)都保持不变,方便跟 CasADi/IPOPT 版本直接对比。

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
from std_msgs.msg import Float64, Float64MultiArray

from offboard_test.nmpc_node import (
    build_reference,
    build_reference_window,
    quat_to_euler,
    quat_to_rotmat,
)
from .figure8_reference import (
    build_reference_figure8,
    build_reference_window_figure8,
)
from .straight_reference import (
    build_reference_straight,
    build_reference_window_straight,
)

from .acados_params import p
from .acados_solver_builder import ensure_acados_ocp_solver
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
        #      位置控制器被绕过,MPC_THR_HOVER 不参与(所以那个 0.60 是巧合、非标定点);
        #   2) THR_MDL_FAC=0(airframe 默认)→ PX4 不做推力曲线线性化,电机控制信号
        #      = 归一化推力,线性透传;
        #   3) GZMixingInterfaceESC 把归一化[0,1]缩放成电机角速度:
        #      ω = OMEGA_MIN + OMEGA_SPAN*norm  (SIM_GZ_EC_MIN/MAX = 150/1000 rad/s);
        #   4) gz MulticopterMotorModel 出力 T = THRUST_K * ω²(4 电机合计)。
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

        # 证伪实验开关:True(默认)=正常 warm-start(shift 上一次解当初值);
        # False=每一步都用参考轨迹重新 seed(cold start,丢掉历史路径依赖)。
        # 假说是:warm-start 在 8 字交叉点附近的正反馈(线性化点偏了->tau 推得
        # 更偏->下一步起点更差)是雪崩根因。如果 cold start 下发散消失或推迟,
        # 就证实是这个机制;如果照样发散,说明跟 warm-start 无关。
        self.warm_start_enabled = True

        # 'circle' / 'figure8' / 'straight' 三种参考轨迹都保留、互不影响,改这
        # 一个字符串就能切换。straight 是往返直线(yaw 固定不变,详见
        # straight_reference.py),用来直观验证机头朝向是否跟随飞行方向——比
        # 圆形/8字更容易在 RViz 里一眼看出对不对。
        self.trajectory_shape = 'figure8'
        if self.trajectory_shape == 'figure8':
            self.ref_fn = build_reference_figure8
            self.ref_window_fn = build_reference_window_figure8
        elif self.trajectory_shape == 'straight':
            self.ref_fn = build_reference_straight
            self.ref_window_fn = build_reference_window_straight
        else:
            self.ref_fn = build_reference
            self.ref_window_fn = build_reference_window

        self.rate_test_mode = False
        self.rate_test_duration = 0.5
        self.rate_test_cmd = np.array([0.2, 0.0, 0.0])

        # 求解失败处理用:偶尔失败一次不去扰动求解器内部的 warm-start(让它
        # 保留当前、哪怕没收敛的迭代值当下一次起点),只有连续失败太多次
        # (大概率已经飞出去了)才强制拉回安全悬停状态。
        self.solve_fail_count = 0
        self.max_consecutive_fail = 20  # @dt=0.1s 约 2 秒
        self.last_u_opt = p.u_hover.copy()
        self.last_omega_cmd = np.zeros(3)

        # 诊断实验(tau_max 临时放宽到 2.0):记录力矩绝对值的峰值,看求解器
        # 在不被约束卡住的情况下自己想要多大力矩。
        self.max_roll_torque  = 0.0
        self.max_pitch_torque = 0.0
        self.max_yaw_torque   = 0.0
        self.max_res_stat = 0.0
        self.last_res_stat = 0.0
        self.last_sqp_iter = 0
        self.sqp_trace_dumped = False

        # 闭环自适应质量:默认是 acados_params.py 里的标定常数 p.m,一旦
        # mhe_node 发来新估计就实时更新——下一次 solve_nmpc 调用就会用这个新
        # 值算动力学(见 acados_model.py 的 m_sym),不需要重新生成/编译求解器。
        # 用 mhe_params 里同一套边界做夹紧,防止 MHE 估计器偶尔抽风给出离谱值
        # (比如窗口还没收敛时)直接污染 NMPC 的动力学模型。
        #
        # "投放"场景(方案A,见 model.sdf 里 mass_changer 插件的注释):飞机从
        # 起飞那一刻就带着 payload(质量是空机+包裹),所以初始猜测值也要从
        # 空机改成带载——不然 NMPC 刚接管的头几步会用错误偏小的质量算动力学,
        # 直到 MHE 自己收敛回来,平白多一段瞬态误差。
        self.payload_mass = 0.5
        self.m_est = p.m + self.payload_mass
        # 吊挂载荷惯量增量(model.p 的第 15 维,见 acados_model.py dJ_sym 注释)。
        # mass_changer 场景是 wrench 模拟的纯平动质量变化、无惯量变化,恒 0;
        # 只有 gripper 场景 attach 后才会被 _grip_mass_step 阶跃。
        self.dJ_est = 0.0
        # 复合质心水平偏移 c=[cx,cy](model.p 第 16-17 维,见 acados_model.py
        # c_sym 注释)。同样只在 gripper attach 后由 _grip_mass_step 赋值。
        self.c_est = np.zeros(2)
        # attach 瞬间的真实几何偏移(box-机体,proximity 节点发布),None=还没
        # 收到。有它就用真实力臂/偏心算 dJ 和 c_est,没有才退回 grip_arm_d 参数。
        self.attach_offset = None

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
        self.payload_enabled = True
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
        # box 正上方的安全接近高度:两段式接近的第一段目标高度。先在这个高度
        # 把水平位置对齐、悬停稳,再垂直下降到 grip_z_low——避免在 grip_z_low
        # 这种低空做水平平移时高度下冲、起落架把 box 顶出吸附窗口(attach 竞态)。
        self.declare_parameter('grip_approach_z', 1.5)
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
            self.grip_payload_mass = float(
                self.get_parameter('grip_payload_mass').value)
            self.grip_mass_step_sec = float(
                self.get_parameter('grip_mass_step_sec').value)
            self.grip_arm_d = float(self.get_parameter('grip_arm_d').value)
            self.grip_approach_z = float(
                self.get_parameter('grip_approach_z').value)
            self.grip_high_aligned = False   # 两段式接近:第一段(高空对齐)是否完成
            self.grip_mass_stepped = False
            # PX4 内环增益同步(见 scale_px4_rate_gains 参数声明处的根因注释)
            self.scale_px4_rate_gains = bool(
                self.get_parameter('scale_px4_rate_gains').value)
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
            self.z_hover = self.grip_z_low   # 先低空悬停到 box 上方
            self.grip_lift_started = False
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

        self.ref_path_msg = self._build_ref_path_msg()
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
        self._append_actual_path(msg)

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
            if self.gripper_mode and self.grip_payload_mass > 0.0:
                dJ, _ = self._payload_geometry(data)
                self._scale_px4_rate_gains(dJ)

    def mhe_mass_cb(self, msg):
        # use_mhe=False 时:MHE 只当诊断,不把估计喂回 NMPC(m_est 保持固定)。
        # 用来隔离验证控制器本身能否扛住载荷,与 MHE 估计质量解耦。
        if not self.use_mhe:
            return
        m = float(msg.data)
        if math.isfinite(m):
            self.m_est = float(np.clip(m, mhe_p.m_min, mhe_p.m_max))

    def _build_ref_path_msg(self, r=1.0, w=0.3, z_hover=3.0,
                            n_samples=400, hover_time=2.0, ramp_time=4.0):
        # 从 hover_time+ramp_time 之后开始采样(振幅已经渐变完、alpha=1 的稳态
        # 轨迹),否则采样区间会覆盖 ramp-in 过程,画出的参考线会带渐变螺旋的
        # 痕迹,不是完整对称的形状——纯可视化 bug,不影响 NMPC 实际跟踪的参考。
        # 不传 r/w/dz,让 ref_fn 用各自的默认值(circle=半径1.0平面,
        # figure8=半径1.0+0.5立体,straight=±2.0往返),这样可视化跟 solve_nmpc
        # 实际用的参考永远一致,不用在两处分别维护——之前 straight 用这个方法
        # 自己的 r=1.0 默认值会把可视化画成 ±1m,跟实际飞的 ±2.0m 不一致,就是
        # 这类参数没对齐导致的。
        msg = Path()
        msg.header.frame_id = 'map'
        period = 2.0 * math.pi / w
        t0 = hover_time + ramp_time
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

        self.solver.set(0, 'lbx', x_cur)
        self.solver.set(0, 'ubx', x_cur)
        for i in range(p.N + 1):
            self.solver.set(i, 'p', np.concatenate(
                [Xref_win[:, i], [self.m_est], [self.dJ_est], self.c_est]))

        if not self.warm_start_enabled:
            # cold start:无论上一步成功与否,都丢掉历史 warm-start,每一步
            # 重新从参考轨迹/悬停猜测出发,切断"上一步解→这一步初值"的路径依赖。
            self._seed_initial_guess(x_cur, Xref_win)

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

        # 一次性 dump:第一次顶满 max_iter 时,把完整的逐次迭代 res_stat/alpha
        # 轨迹打出来,用来区分"震荡型"(非光滑代价,alpha 反复缩步长)还是
        # "平台型"(参考轨迹本身不可行,alpha 接近 1 但 res_stat 单调躺平)。
        # statistics 矩阵行定义(SQP): 0=iter,1=res_stat,2=res_eq,3=res_ineq,
        # 4=res_comp,5=qp_status,6=qp_iter,7=alpha。
        if sqp_iter >= 95 and not self.sqp_trace_dumped:
            self.sqp_trace_dumped = True
            stats = self.solver.get_stats('statistics')
            n_iter = stats.shape[1]
            self.get_logger().warn(
                f't={t_ref:.2f}s | sqp_iter hit max ({sqp_iter}) for the first time, '
                f'dumping full iteration trace ({n_iter} rows, format: it res_stat alpha):')
            lines = [f'it={i:3d} res_stat={stats[1, i]:.4e} alpha={stats[7, i]:.4f}'
                     for i in range(n_iter)]
            self.get_logger().warn('\n'.join(lines))
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
                # 延用上一次的 omega_cmd,结果连续失败 5~10 帧(@10Hz 只有
                # 0.5~1 秒)时飞机会带着同一个非零角速度持续旋转、完全没有
                # 刹车,几秒内就倾覆自由下坠摔机。归零角速度只是"停止主动
                # 旋转",不会像整体重置成悬停那样粗暴。
                u_opt = self.last_u_opt.copy()
                omega_cmd = np.zeros(3)

        solve_time = (time.time() - t_start) * 1000
        return u_opt, omega_cmd, solve_time

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
        msg.body_rate.x = float(np.clip(omega_cmd[0], -wmax, wmax))
        msg.body_rate.y = float(np.clip(omega_cmd[1], -wmax, wmax))
        msg.body_rate.z = float(np.clip(omega_cmd[2], -wmax, wmax))
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        self.att_pub.publish(msg)

    def _trigger_payload_detach(self):
        # 发给 mass_changer 插件(gz_plugins/mass_changer/)的 /payload/drop_mass
        # topic——插件收到后会在物理引擎层面把 base_link 从 2.5kg 切回
        # 2.0kg,只切一次。用 Popen 而不是 run/check_call:这是在 10Hz 定时器
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
        if self.grip_payload_mass <= 0.0 or nmpc_time < self.grip_mass_step_sec:
            return
        self.grip_mass_stepped = True
        m_p = self.grip_payload_mass
        m_t = p.m + m_p
        self.m_est = m_t
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
            f'of tau_max {p.tau_max}, MHE bypass)')

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

    def _scale_px4_rate_gains(self, dJ):
        """把 PX4 速率环总增益按惯量比 (J+dJ)/J 放大,恢复被载荷惯量压塌的
        内环带宽(根因见 scale_px4_rate_gains 参数声明处注释)。只做一次;
        ratio 上限 5 防止 dJ 异常大时把内环推成高频震荡。"""
        if self.px4_gains_scaled or not self.scale_px4_rate_gains:
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

    def _lift_phase(self, nmpc_time):
        """NMPC 接管(低空悬停 + proximity 完成 attach)满 grip_lift_after_sec
        秒后,把 z_hover 从 grip_z_low 平滑抬到 grip_z_high,把 box 吊离地面,
        让 MHE 看到全额 +载荷质量。"""
        if not self.gripper_mode or nmpc_time < self.grip_lift_after_sec:
            return
        s = min(max((nmpc_time - self.grip_lift_after_sec) /
                    self.grip_lift_dur, 0.0), 1.0)
        smooth = 10*s**3 - 15*s**4 + 6*s**5
        self.z_hover = self.grip_z_low + \
            (self.grip_z_high - self.grip_z_low) * smooth
        if not self.grip_lift_started:
            self.grip_lift_started = True
            self.get_logger().info(
                f't={nmpc_time:.1f}s | LIFT: raising hover '
                f'{self.grip_z_low}->{self.grip_z_high}m to lift payload')

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
                    'Gripper approach: aligned above box, descending '
                    f'vertically to grip z={self.grip_z_low:.2f}m')
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
        self._grip_mass_step(nmpc_time)
        self._lift_phase(nmpc_time)
        t_ref = 0.0 if self.hover_test_mode else nmpc_time

        if self.rate_test_mode:
            u_opt = p.u_hover.copy()
            solve_time = 0.0
            if nmpc_time < self.rate_test_duration:
                omega_cmd = self.rate_test_cmd.copy()
            else:
                omega_cmd = np.zeros(3)
            roll, pitch, yaw = quat_to_euler(*self.x_cur[6:10])
            self.get_logger().info(
                f't={nmpc_time:.3f}s | om_meas=[{self.x_cur[10]:+.3f} '
                f'{self.x_cur[11]:+.3f} {self.x_cur[12]:+.3f}] | '
                f'rpy=[{math.degrees(roll):+6.2f} {math.degrees(pitch):+6.2f} '
                f'{math.degrees(yaw):+6.2f}]deg | cmd={omega_cmd}')
        else:
            u_opt, omega_cmd, solve_time = self.solve_nmpc(self.x_cur, t_ref)
            if nmpc_time < self.bodyrate_ramp_time:
                s = nmpc_time / self.bodyrate_ramp_time
                ramp = 10*s**3 - 15*s**4 + 6*s**5
                omega_cmd = omega_cmd * ramp
        self.publish_attitude(u_opt, omega_cmd)
        self.u_opt_pub.publish(Float64MultiArray(data=[float(v) for v in u_opt]))

        xref_now = self.ref_fn(t_ref)
        pos_err = np.linalg.norm(self.x_cur[0:3] - xref_now[0:3])
        self.tracking_err_pub.publish(Float64(data=float(pos_err)))

        # 诊断:力矩输出占约束上限的比例,以及实际绝对值峰值(tau_max 现在临时
        # 放宽到 2.0,看求解器在不被约束卡住的情况下自己想要多大力矩)。
        roll_pct  = abs(u_opt[1]) / p.tau_max * 100.0
        pitch_pct = abs(u_opt[2]) / p.tau_max * 100.0
        yaw_pct   = abs(u_opt[3]) / p.tau_psi * 100.0
        self.max_roll_torque  = max(self.max_roll_torque, abs(u_opt[1]))
        self.max_pitch_torque = max(self.max_pitch_torque, abs(u_opt[2]))
        self.max_yaw_torque   = max(self.max_yaw_torque, abs(u_opt[3]))
        max_pct   = max(roll_pct, pitch_pct, yaw_pct)
        if max_pct > 85.0:
            self.get_logger().warn(
                f't={nmpc_time:.2f}s | Torque near constraint limit! roll={roll_pct:.0f}% '
                f'pitch={pitch_pct:.0f}% yaw={yaw_pct:.0f}% | pos_err={pos_err:.3f}m | '
                f'peak roll={self.max_roll_torque:.3f} pitch={self.max_pitch_torque:.3f} '
                f'yaw={self.max_yaw_torque:.3f} Nm')

        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(
                f't={nmpc_time:.1f}s | pos_err={pos_err:.3f}m | '
                f'T={u_opt[0]:.2f}N | tau=[r{u_opt[1]:.3f} p{u_opt[2]:.3f} '
                f'y{u_opt[3]:.3f}]Nm | peak=[r{self.max_roll_torque:.3f} '
                f'p{self.max_pitch_torque:.3f} y{self.max_yaw_torque:.3f}]Nm | '
                f'res_stat={self.last_res_stat:.3e}(peak {self.max_res_stat:.3e}) '
                f'sqp_iter={self.last_sqp_iter} | solve={solve_time:.1f}ms')


def main():
    rclpy.init()
    node = AcadosNMPCNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
