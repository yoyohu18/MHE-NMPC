#!/usr/bin/env python3
# =========================================================
#  NMPC 无人机 — Python/CasADi + ROS2 版本 v2
#  修复：加入 EKF2 等待逻辑，避免过早解锁
# =========================================================

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
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64

import numpy as np
import casadi as cs
import math
import time


# =========================================================
#  1. 物理参数
# =========================================================
class Params:
    m   = 2.0643
    g   = 9.81
    Jxx = 0.0142
    Jyy = 0.0142
    Jzz = 0.0210
    kd  = 0.05
    N   = 10
    dt  = 0.1
    nx  = 13
    nu  = 4
    Tmin    = 0.5
    Tmax    = 2 * m * g
    tau_max = 0.5
    tau_psi = 0.2

    # --- Bryson's rule 无量纲化: Q_ii = 1/(典型尺度)^2,R_jj 同理 ---
    # 这套权重是在 acados 移植版上诊断出来的:原来的 Q_pos=4 对 R_torque=0.1
    # 量纲失衡,Hessian 各方向曲率差几十倍,在 8 字轨迹上会让 Gauss-Newton
    # 收敛率退化到 ~0.9(慢线性收敛),顶满 SQP max_iter 后偶发雪崩失控
    # (res_stat 峰值能到 6.26e5)。这套规则把每个误差分量在各自"典型尺度"
    # 下对 cost 的贡献都归一化到 1,数值上跟 acados_params.py 完全一致,
    # 拿去给 acados 用四轮+独立验证全部稳定(8字轨迹最长 422 秒零失败)。
    # CasADi/IPOPT 本身在圆形轨迹上从没出现过这个问题,这次同步过来是为了
    # 让两边用同一套"讲得通道理"的权重,不是因为 CasADi 这边出过故障。
    L_pos, L_vel, L_att, L_omega = 0.1, 0.3, 0.05, 0.3
    L_T, L_tau_rp, L_tau_yaw = 2.0, tau_max, tau_psi

    Qp = np.diag([10.0, 10.0, 10.0])
    Qv = 1  * np.eye(3)
    Qq = 1  * np.eye(3)
    Qw = 0.1 * np.eye(3)
    Q  = np.block([
        [(1/L_pos**2)*np.eye(3),   np.zeros((3,3)),         np.zeros((3,3)),          np.zeros((3,3))],
        [np.zeros((3,3)),          (1/L_vel**2)*np.eye(3),  np.zeros((3,3)),          np.zeros((3,3))],
        [np.zeros((3,3)),          np.zeros((3,3)),         (1/L_att**2)*np.eye(3),   np.zeros((3,3))],
        [np.zeros((3,3)),          np.zeros((3,3)),         np.zeros((3,3)),          (1/L_omega**2)*np.eye(3)]
    ])
    R  = np.diag([1/L_T**2, 1/L_tau_rp**2, 1/L_tau_rp**2, 1/L_tau_yaw**2])
    P  = np.block([
        [20*(1/L_pos**2)*np.eye(3), np.zeros((3,3)),           np.zeros((3,3)),            np.zeros((3,3))],
        [np.zeros((3,3)),           (1/L_vel**2)*np.eye(3),    np.zeros((3,3)),            np.zeros((3,3))],
        [np.zeros((3,3)),           np.zeros((3,3)),           2*(1/L_att**2)*np.eye(3),   np.zeros((3,3))],
        [np.zeros((3,3)),           np.zeros((3,3)),           np.zeros((3,3)),            0.2*(1/L_omega**2)*np.eye(3)]
    ])
    u_hover = np.array([m * g, 0.0, 0.0, 0.0])


p = Params()


# =========================================================
#  2. 动力学
# =========================================================
def build_dynamics():
    x_sym = cs.MX.sym('x', p.nx)
    u_sym = cs.MX.sym('u', p.nu)

    vel = x_sym[3:6]
    q_  = x_sym[6:10]
    om  = x_sym[10:13]
    T_   = u_sym[0]
    tau_ = u_sym[1:4]

    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3]
    R_q = cs.vertcat(
        cs.horzcat(qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),        2*(qx*qz+qw*qy)),
        cs.horzcat(2*(qx*qy+qw*qz),          qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)),
        cs.horzcat(2*(qx*qz-qw*qy),          2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2)
    )

    g_vec = cs.MX([0.0, 0.0, p.g])
    vel_dot = (1.0/p.m) * (cs.mtimes(R_q, cs.vertcat(0.0, 0.0, T_)) - p.kd * vel) - g_vec

    Xi_q = cs.vertcat(
        cs.horzcat(-qx, -qy, -qz),
        cs.horzcat( qw, -qz,  qy),
        cs.horzcat( qz,  qw, -qx),
        cs.horzcat(-qy,  qx,  qw)
    )
    quat_dot = 0.5 * cs.mtimes(Xi_q, om)

    J_vec = cs.MX([p.Jxx, p.Jyy, p.Jzz])
    Jom   = J_vec * om
    om_dot = (tau_ - cs.cross(om, Jom)) / J_vec

    xdot = cs.vertcat(vel, vel_dot, quat_dot, om_dot)
    f_ode = cs.Function('f_ode', [x_sym, u_sym], [xdot])
    return f_ode, x_sym, u_sym


# =========================================================
#  3. RK4
# =========================================================
def build_rk4(f_ode, x_sym, u_sym):
    dt = p.dt
    k1 = f_ode(x_sym, u_sym)
    k2 = f_ode(x_sym + dt/2 * k1, u_sym)
    k3 = f_ode(x_sym + dt/2 * k2, u_sym)
    k4 = f_ode(x_sym + dt * k3,   u_sym)
    x_next = x_sym + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    q_next = x_next[6:10]
    x_next[6:10] = q_next / cs.norm_2(q_next)
    F_rk4 = cs.Function('F_rk4', [x_sym, u_sym], [x_next])
    return F_rk4


# =========================================================
#  追踪误差
# =========================================================
def tracking_error_sym(x, xr):
    ep = x[0:3] - xr[0:3]
    ev = x[3:6] - xr[3:6]
    q  = x[6:10]
    qr = xr[6:10]
    qrc = cs.vertcat(qr[0], -qr[1], -qr[2], -qr[3])
    pw=qrc[0]; px=qrc[1]; py=qrc[2]; pz=qrc[3]
    qw=q[0];  qx=q[1];  qy=q[2];  qz=q[3]
    prod_q = cs.vertcat(
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw
    )
    eq_vec = prod_q[1:4]
    eomega = x[10:13] - xr[10:13]
    return cs.vertcat(ep, ev, eq_vec, eomega)


# =========================================================
#  4. NLP
# =========================================================
def build_nlp():
    print("构建符号动力学...")
    f_ode, x_sym, u_sym = build_dynamics()
    print("构建 RK4...")
    F_rk4 = build_rk4(f_ode, x_sym, u_sym)
    print("构建 NLP...")
    opti = cs.Opti()

    X = opti.variable(p.nx, p.N+1)
    U = opti.variable(p.nu, p.N)
    X0_par   = opti.parameter(p.nx, 1)
    Xref_par = opti.parameter(p.nx, p.N+1)
    Uref_par = opti.parameter(p.nu, p.N)

    J_total = 0
    for t in range(p.N):
        e_t  = tracking_error_sym(X[:, t], Xref_par[:, t])
        eu_t = U[:, t] - Uref_par[:, t]
        J_total += 0.5 * cs.mtimes([e_t.T,  cs.MX(p.Q), e_t])
        J_total += 0.5 * cs.mtimes([eu_t.T, cs.MX(p.R), eu_t])

    eN = tracking_error_sym(X[:, p.N], Xref_par[:, p.N])
    J_total += 0.5 * cs.mtimes([eN.T, cs.MX(p.P), eN])
    opti.minimize(J_total)

    opti.subject_to(X[:, 0] == X0_par)
    for t in range(p.N):
        opti.subject_to(X[:, t+1] == F_rk4(X[:, t], U[:, t]))

    opti.subject_to(opti.bounded(p.Tmin,     U[0, :], p.Tmax))
    opti.subject_to(opti.bounded(-p.tau_max, U[1, :], p.tau_max))
    opti.subject_to(opti.bounded(-p.tau_max, U[2, :], p.tau_max))
    opti.subject_to(opti.bounded(-p.tau_psi, U[3, :], p.tau_psi))

    # 注意:四元数 + 非线性推力旋转动力学,exact Hessian 比 L-BFGS 稳得多.
    # 速度需要进一步压榨时再考虑 fatrop/acados,不要回 limited-memory.
    opts = {
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'ipopt.max_iter': 100,
        'ipopt.tol': 1e-3,
        'ipopt.acceptable_tol': 1e-2,
        'ipopt.acceptable_iter': 5,
        # warm-start 留着但不激进,首次冷启动也能正常进入
        'ipopt.warm_start_init_point': 'yes',
        'ipopt.warm_start_bound_push': 1e-4,
        'ipopt.warm_start_mult_bound_push': 1e-4,
        'ipopt.mu_init': 1e-2,
        'print_time': 0,
    }
    opti.solver('ipopt', opts)
    print("NLP 构建完成！")
    return opti, X, U, X0_par, Xref_par, Uref_par


# =========================================================
#  四元数 -> 旋转矩阵 (body -> world，与 build_dynamics 中 R_q 一致)
# =========================================================
def quat_to_rotmat(qw, qx, qy, qz):
    return np.array([
        [qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),        2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),         qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2]
    ])


def quat_to_euler(qw, qx, qy, qz):
    roll  = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    pitch = math.asin(np.clip(2*(qw*qy-qz*qx), -1.0, 1.0))
    yaw   = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return roll, pitch, yaw


# =========================================================
#  参考轨迹
# =========================================================
def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll/2);  sr = math.sin(roll/2)
    cp = math.cos(pitch/2); sp = math.sin(pitch/2)
    cy = math.cos(yaw/2);   sy = math.sin(yaw/2)
    q = np.array([
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        sy*cp*sr + cy*sp*cr,
        sy*cp*cr - cy*sp*sr
    ])
    return q / np.linalg.norm(q)


def build_reference(t, r=1.0, w=0.3, z_hover=3.0, dz=0.0,
                    hover_time=2.0, ramp_time=4.0):
    # 圆形轨迹: x = r*cos(a), y = r*sin(a),恒定角速度 w,恒定速度 r*w,
    # 不像 8 字在交叉点附近有速度方向反转/yaw 奇异性,更容易稳定跟踪。
    # 默认 dz=0,z 保持恒定 z_hover,避免 z 大幅振荡
    # t < hover_time   : 在起点 (r,0,z_hover+dz) 悬停
    # ramp_time 内     : 用 5 次 smooth-step 把振幅从 0 渐增到目标,避免速度/yaw 突变
    if t < hover_time:
        pos = np.array([r, 0.0, z_hover + dz])
        vel = np.zeros(3)
        q   = np.array([1.0, 0.0, 0.0, 0.0])
        om  = np.zeros(3)
        return np.concatenate([pos, vel, q, om])

    tc = t - hover_time

    if tc < ramp_time:
        s = tc / ramp_time
        alpha     = 10*s**3 - 15*s**4 + 6*s**5
        alpha_dot = (30*s**2 - 60*s**3 + 30*s**4) / ramp_time
    else:
        alpha     = 1.0
        alpha_dot = 0.0

    a  = w * tc
    ca, sa = math.cos(a), math.sin(a)

    # 完整圆形轨迹
    x_f  = r * ca
    y_f  = r * sa
    z_f  = z_hover + dz * ca
    vx_f = -r * w * sa
    vy_f = r * w * ca
    vz_f = -dz * w * sa
    ax_f = -r * w * w * ca
    ay_f = -r * w * w * sa

    # 起点 (a=0): (r, 0, z_hover+dz),从起点 alpha 插值到圆形轨迹
    x0, y0, z0 = r, 0.0, z_hover + dz

    x = x0 + alpha * (x_f - x0)
    y = y0 + alpha * (y_f - y0)
    z = z0 + alpha * (z_f - z0)
    vx = alpha_dot * (x_f - x0) + alpha * vx_f
    vy = alpha_dot * (y_f - y0) + alpha * vy_f
    vz = alpha_dot * (z_f - z0) + alpha * vz_f
    ax = alpha_dot * vx_f + alpha * ax_f
    ay = alpha_dot * vy_f + alpha * ay_f

    denom = vx*vx + vy*vy
    if denom > 1e-4:
        yaw      = math.atan2(vy, vx)
        yaw_rate = (vx*ay - vy*ax) / denom
    else:
        yaw      = 0.0
        yaw_rate = 0.0

    pos = np.array([x, y, z])
    vel = np.array([vx, vy, vz])
    q   = euler_to_quat(0.0, 0.0, yaw)
    om  = np.array([0.0, 0.0, yaw_rate])
    return np.concatenate([pos, vel, q, om])


def build_reference_window(t_start, N, dt, r=1.0, w=0.3, z_hover=3.0, dz=0.0):
    xref = np.zeros((p.nx, N+1))
    for i in range(N+1):
        xref[:, i] = build_reference(t_start + i*dt, r, w, z_hover, dz)
    return xref


def build_reference_figure8(t, r=1.0, w=0.3, z_hover=3.0, dz=0.5,
                             hover_time=2.0, ramp_time=4.0):
    # 8 字轨迹(Gerono lemniscate),跟 offboard_test_acados/figure8_reference.py
    # 完全同一套数学,同步过来给 CasADi/IPOPT 版本做对照测试。
    # 参数化: x=r*sin(a), y=r*sin(a)*cos(a)=r/2*sin(2a), a=w*tc 匀角速度转动。
    if t < hover_time:
        pos = np.array([0.0, 0.0, z_hover])
        vel = np.zeros(3)
        q   = np.array([1.0, 0.0, 0.0, 0.0])
        om  = np.zeros(3)
        return np.concatenate([pos, vel, q, om])

    tc = t - hover_time

    if tc < ramp_time:
        s = tc / ramp_time
        alpha     = 10*s**3 - 15*s**4 + 6*s**5
        alpha_dot = (30*s**2 - 60*s**3 + 30*s**4) / ramp_time
    else:
        alpha     = 1.0
        alpha_dot = 0.0

    a = w * tc
    sa, ca   = math.sin(a), math.cos(a)
    s2a, c2a = math.sin(2*a), math.cos(2*a)

    # z 用 sin(a) 而不是 cos(a):跟 x 同符号同步变化,右侧叶子(x>0)整体抬高、
    # 左侧叶子(x<0)整体压低,在原点交叉处两叶在同一高度 z_hover 相接。
    x_f  = r * sa
    y_f  = 0.5 * r * s2a
    z_f  = z_hover + dz * sa
    vx_f = r * w * ca
    vy_f = r * w * c2a
    vz_f = dz * w * ca
    ax_f = -r * w * w * sa
    ay_f = -2.0 * r * w * w * s2a

    x0, y0, z0 = 0.0, 0.0, z_hover

    x = x0 + alpha * (x_f - x0)
    y = y0 + alpha * (y_f - y0)
    z = z0 + alpha * (z_f - z0)
    vx = alpha_dot * (x_f - x0) + alpha * vx_f
    vy = alpha_dot * (y_f - y0) + alpha * vy_f
    vz = alpha_dot * (z_f - z0) + alpha * vz_f
    ax = alpha_dot * vx_f + alpha * ax_f
    ay = alpha_dot * vy_f + alpha * ay_f

    denom = vx*vx + vy*vy
    if denom > 1e-4:
        yaw      = math.atan2(vy, vx)
        yaw_rate = (vx*ay - vy*ax) / denom
    else:
        yaw      = 0.0
        yaw_rate = 0.0

    pos = np.array([x, y, z])
    vel = np.array([vx, vy, vz])
    q   = euler_to_quat(0.0, 0.0, yaw)
    om  = np.array([0.0, 0.0, yaw_rate])
    return np.concatenate([pos, vel, q, om])


def build_reference_window_figure8(t_start, N, dt, r=1.0, w=0.3, z_hover=3.0, dz=0.5):
    xref = np.zeros((p.nx, N+1))
    for i in range(N+1):
        xref[:, i] = build_reference_figure8(t_start + i*dt, r, w, z_hover, dz)
    return xref


# =========================================================
#  ROS2 节点
# =========================================================
class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')
        self.get_logger().info('NMPC 节点启动，正在构建 NLP...')

        self.opti, self.X, self.U, \
            self.X0_par, self.Xref_par, self.Uref_par = build_nlp()

        self.state     = State()
        self.x_cur     = None
        self.counter   = 0
        self.start_time = None
        self.armed_and_flying = False
        self.last_mode_req_time = 0.0
        self.last_arm_req_time  = 0.0
        self.ekf_wait_sec = 10.0
        self.nmpc_started = False
        self.nmpc_start_time = None
        # 圆形轨迹默认 dz=0,起点 z = z_hover,所有阶段高度一致
        self.z_hover = 3.0
        self.start_xy_threshold = 0.20
        self.start_z_threshold  = 0.15
        # 推力归一化:T = m*g 对应该值。实测值 0.729 (来自 vehicle_thrust_setpoint
        # 在 gz_x500 稳定悬停时的 Z 分量),MPC_THR_HOVER=0.6 不反映该机型真实情况
        self.hover_thrust_pct = 0.729
        # body_rate 安全上限,防止 NMPC 极端预测把飞机扯翻
        self.omega_cmd_max = 2.0
        # NMPC 接管后头几百毫秒把 body_rate 从 0 平滑斜坡到 NMPC 输出
        self.bodyrate_ramp_time = 0.8
        # 调试用:固定 t_ref=0,让 NMPC 只悬停在起点,不追八字轨迹
        self.hover_test_mode = False

        # 圆形轨迹已经验证过,现在切到 8 字轨迹跟 acados 版做对照测试。
        # 改回 'circle' 即可切回圆形,两套 reference 函数都保留、互不影响。
        self.trajectory_shape = 'figure8'
        if self.trajectory_shape == 'figure8':
            self.ref_fn = build_reference_figure8
            self.ref_window_fn = build_reference_window_figure8
        else:
            self.ref_fn = build_reference
            self.ref_window_fn = build_reference_window

        xref0 = self.ref_fn(0.0)
        self.X_init = np.tile(xref0.reshape(-1,1), (1, p.N+1))
        self.U_init = np.tile(p.u_hover.reshape(-1,1), (1, p.N))

        # 预热 IPOPT:首次 solve 是冷启动(~350ms),若发生在 NMPC 接管那一帧会
        # 阻塞整个单线程 executor,导致 body_rate ramp 在前两帧之间跳变。
        # 这里在节点初始化阶段(尚未 spin)用悬停状态先 solve 一次,
        # 把符号函数 JIT/线性求解器初始化都提前做完,后续每次 solve 只需 ~10ms。
        self.get_logger().info('预热 IPOPT 求解器...')
        t_warm = time.time()
        self.solve_nmpc(xref0, 0.0)
        self.get_logger().info(
            f'求解器预热完成，耗时 {(time.time()-t_warm)*1000:.1f} ms')
        # 预热 solve 的 warm-start 结果重置回悬停初值,避免污染首次真实求解
        self.X_init = np.tile(xref0.reshape(-1,1), (1, p.N+1))
        self.U_init = np.tile(p.u_hover.reshape(-1,1), (1, p.N))

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

        self.att_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude', 10)

        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local', 10)

        self.ref_path_pub = self.create_publisher(
            Path, '/nmpc/reference_path', 10)
        self.actual_path_pub = self.create_publisher(
            Path, '/nmpc/actual_path', 10)
        self.nmpc_traj_pub = self.create_publisher(
            Path, '/nmpc_traj', 10)
        self.tracking_err_pub = self.create_publisher(
            Float64, '/nmpc/tracking_error', 10)

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
        self.get_logger().info('NMPC 节点初始化完成！等待 EKF2 收敛...')

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        q   = msg.pose.pose.orientation
        om  = msg.twist.twist.angular
        # 四元数归一化,避免 EKF 数值漂移让 NMPC 初始点不在单位流形上
        qn = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if qn < 1e-6:
            return  # 异常,丢弃
        qw, qx, qy, qz = q.w/qn, q.x/qn, q.y/qn, q.z/qn
        # mavros local_position/odom 的 twist.linear 是 body frame(FLU)速度,
        # 而 NMPC 动力学模型里 pos_dot = vel 要求 vel 是 world frame(ENU),
        # 这里用当前姿态转到 world frame,否则水平时(vel误差小)看不出问题,
        # 一旦开始倾斜 body/world 速度就对不上,几秒内发散摔机。
        R_wb = quat_to_rotmat(qw, qx, qy, qz)
        vel_world = R_wb @ np.array([vel.x, vel.y, vel.z])
        x = np.array([
            pos.x, pos.y, pos.z,
            vel_world[0], vel_world[1], vel_world[2],
            qw, qx, qy, qz,
            om.x, om.y, om.z
        ])
        # 任何一个 NaN/Inf 都直接丢弃,否则会喂坏 NMPC warm-start
        if not np.all(np.isfinite(x)):
            return
        self.x_cur = x
        self._append_actual_path(msg)

    def _build_ref_path_msg(self, r=1.0, w=0.3, z_hover=3.0,
                            n_samples=400, hover_time=2.0, ramp_time=4.0):
        # 从 hover_time+ramp_time 之后开始采样(振幅已经渐变完、alpha=1 的稳态
        # 轨迹),避免采样区间覆盖 ramp-in 过程画出带渐变痕迹的参考线。
        # 不传 dz,让 ref_fn 用各自默认值(circle=0.0 平面,figure8=0.5 立体),
        # 这样可视化跟 solve_nmpc 实际用的参考永远一致。
        msg = Path()
        msg.header.frame_id = 'map'
        period = 2.0 * math.pi / w
        t0 = hover_time + ramp_time
        for i in range(n_samples + 1):
            t = t0 + i * period / n_samples
            xref = self.ref_fn(t, r, w, z_hover)
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
        # 起飞/等待阶段悬停在 (0,0,z_hover),与圆形轨迹起点同高度,
        # 切到 NMPC 时只需 xy 移动,无 z 跳变
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

    def solve_nmpc(self, x_cur, t_ref):
        # 跟踪模式：参考轨迹从当前 NMPC 时间 t_ref 开始,沿圆形轨迹向前取 N+1 个点
        Xref_win = self.ref_window_fn(t_ref, p.N, p.dt, z_hover=self.z_hover)
        Uref_win = np.tile(p.u_hover.reshape(-1,1), (1, p.N))

        # X_init[:,0] 与 x_cur 不一致会让初始猜测不满足等式约束 X[:,0]==X0_par,
        # 强制对齐能显著降低 IPOPT 首次启动失败率
        self.X_init[:, 0] = x_cur

        self.opti.set_value(self.X0_par,   x_cur)
        self.opti.set_value(self.Xref_par, Xref_win)
        self.opti.set_value(self.Uref_par, Uref_win)
        self.opti.set_initial(self.X, self.X_init)
        self.opti.set_initial(self.U, self.U_init)

        t_start = time.time()
        try:
            sol = self.opti.solve()
            X_sol = sol.value(self.X)
            U_sol = sol.value(self.U)
            # warm-start 向前滚动一步,末端复制
            self.X_init = np.hstack([X_sol[:, 1:], X_sol[:, -1:]])
            self.U_init = np.hstack([U_sol[:, 1:], U_sol[:, -1:]])
            u_opt = U_sol[:, 0]
            omega_cmd = X_sol[10:13, 1]
            self._set_nmpc_traj_msg(X_sol)
        except Exception as e:
            # 取 IPOPT 返回的具体状态 (Restoration_Failed / Infeasible / Max_Iter 等)
            try:
                status = self.opti.return_status()
            except Exception:
                status = 'unknown'
            self.get_logger().warn(
                f'IPOPT 失败 [{status}]: {e} | '
                f'x_cur pos={x_cur[0:3]} vel={x_cur[3:6]} '
                f'|q|={np.linalg.norm(x_cur[6:10]):.4f}')
            # 失败时丢掉脏 warm-start,用当前参考hover 重置,
            # 否则坏初值会持续喂给下一次求解,造成"一直失败"
            self.X_init = np.tile(Xref_win[:, 0:1], (1, p.N+1))
            self.X_init[:, 0] = x_cur
            self.U_init = np.tile(p.u_hover.reshape(-1,1), (1, p.N))
            u_opt = p.u_hover.copy()
            omega_cmd = np.zeros(3)

        solve_time = (time.time() - t_start) * 1000
        return u_opt, omega_cmd, solve_time

    def publish_attitude(self, u_opt, omega_cmd):
        T = u_opt[0]  # 牛顿
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 线性归一化:T = m*g 对应 hover_thrust_pct,假设推力线性
        norm = self.hover_thrust_pct * T / (p.m * p.g)
        msg.thrust = float(np.clip(norm, 0.05, 0.95))
        # body_rate 限幅,防止极端预测翻机
        wmax = self.omega_cmd_max
        msg.body_rate.x = float(np.clip(omega_cmd[0], -wmax, wmax))
        msg.body_rate.y = float(np.clip(omega_cmd[1], -wmax, wmax))
        msg.body_rate.z = float(np.clip(omega_cmd[2], -wmax, wmax))
        # IGNORE_ATTITUDE 时姿态字段应被忽略,但填单位四元数防止部分 PX4 实现 bug
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        self.att_pub.publish(msg)

    def timer_cb(self):
        # 阶段0：预热，发100帧位置指令
        if self.counter < 100:
            self.pub_hover_pos()
            self.counter += 1
            return

        # 记录起始时间
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        t_elapsed = (self.get_clock().now() -
                     self.start_time).nanoseconds / 1e9

        if not self.state.connected:
            self.pub_hover_pos()
            return

        # 阶段1：等待EKF2收敛，一直发位置指令
        if t_elapsed < self.ekf_wait_sec:
            self.pub_hover_pos()
            self.counter += 1
            if self.counter % 100 == 0:
                self.get_logger().info(
                    f'等待 EKF2 收敛... '
                    f'{t_elapsed:.0f}/{self.ekf_wait_sec:.0f} 秒')
            return

        now = time.time()

        # 阶段2：切换 OFFBOARD 模式（限频 1Hz）
        if self.state.mode != 'OFFBOARD':
            self.pub_hover_pos()
            if (self.mode_client.service_is_ready()
                    and now - self.last_mode_req_time > 1.0):
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.mode_client.call_async(req)
                self.last_mode_req_time = now
                self.get_logger().info('切换 OFFBOARD 模式')
            return

        # 阶段3：解锁（限频 1Hz）
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
                self.get_logger().info('发送解锁指令')
            return

        # 阶段4：准备轨迹跟踪
        if not self.armed_and_flying:
            self.armed_and_flying = True
            self.get_logger().info('已解锁，准备移动到轨迹起点...')

        if self.x_cur is None:
            self.pub_hover_pos()
            return

        # 阶段4：先到轨迹起点，再把 NMPC 时间置零，避免追击已经跑远的参考点
        start_ref = self.ref_fn(0.0)
        err_xy = float(np.linalg.norm(self.x_cur[0:2] - start_ref[0:2]))
        err_z  = float(abs(self.x_cur[2] - start_ref[2]))
        v_norm = float(np.linalg.norm(self.x_cur[3:6]))
        if not self.nmpc_started:
            self.pub_position_ref(start_ref)
            self.counter += 1
            if self.counter % 50 == 0:
                self.get_logger().info(
                    f'移动到轨迹起点... xy={err_xy:.3f}m '
                    f'z={err_z:.3f}m v={v_norm:.2f}m/s')
            # 必须 xy / z 都到位且速度近 0,姿态接管才平稳
            if (err_xy > self.start_xy_threshold
                    or err_z > self.start_z_threshold
                    or v_norm > 0.25):
                return

            self.nmpc_started = True
            self.nmpc_start_time = self.get_clock().now()
            self.X_init = np.tile(start_ref.reshape(-1,1), (1, p.N+1))
            self.U_init = np.tile(p.u_hover.reshape(-1,1), (1, p.N))
            self.get_logger().info('已到轨迹起点，开始 NMPC 轨迹跟踪！')

        # 求解 NMPC
        nmpc_time = (self.get_clock().now() -
                     self.nmpc_start_time).nanoseconds / 1e9
        t_ref = 0.0 if self.hover_test_mode else nmpc_time

        u_opt, omega_cmd, solve_time = self.solve_nmpc(self.x_cur, t_ref)
        # 接管头 bodyrate_ramp_time 秒平滑斜坡 body_rate,从 0 缓增到 NMPC 输出
        # 避免位置环切姿态/速率环时 1kHz 内环看到突变给大转矩
        if nmpc_time < self.bodyrate_ramp_time:
            s = nmpc_time / self.bodyrate_ramp_time
            ramp = 10*s**3 - 15*s**4 + 6*s**5  # smooth-step
            omega_cmd = omega_cmd * ramp
        self.publish_attitude(u_opt, omega_cmd)

        # 轨迹误差:每帧都发布到 /nmpc/tracking_error,方便 rqt_plot/PlotJuggler 订阅
        xref_now = self.ref_fn(t_ref)
        pos_err = np.linalg.norm(self.x_cur[0:3] - xref_now[0:3])
        self.tracking_err_pub.publish(Float64(data=float(pos_err)))

        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(
                f't={nmpc_time:.1f}s | 位置误差={pos_err:.3f}m | '
                f'T={u_opt[0]:.2f}N | 求解={solve_time:.1f}ms')


def main():
    rclpy.init()
    node = NMPCNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
