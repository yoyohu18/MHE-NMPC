#!/usr/bin/env python3
# 物理/安全常数(质量、惯量、推力/力矩边界)直接复用 offboard_test.nmpc_node.Params,
# 避免两边各存一份、以后改了机架参数却忘了同步。
# Q/R/P/N/dt 是 NMPC 自己的代价函数和时域设置,故意跟 offboard_test 独立、各自能调,
# 调一个控制器的参数不该悄悄影响另一个。

import numpy as np
from offboard_test.nmpc_node import Params as _BaseParams

_base = _BaseParams()


class AcadosParams:
    # --- 物理常数,跟 offboard_test 共享(数值定义在 offboard_test/nmpc_node.py
    #     的 Params 类里,这里只是引用,改机架参数去那边改,两边自动同步) ---
    # Physical constants shared with offboard_test (values live in the Params
    # class in offboard_test/nmpc_node.py; these are just references — edit
    # the airframe params there and both packages stay in sync).
    m   = _base.m    # 机体质量(kg)。出现在推力方程 vel_dot=(1/m)(R@[0,0,T]-kd*v)-g 里
                      # Mass (kg). Appears in vel_dot = (1/m)(R@[0,0,T] - kd*v) - g
    g   = _base.g    # 重力加速度(m/s^2),ENU 坐标系(Z 朝上),方程里是 -g_vec(沿 -Z)
                      # Gravitational acceleration (m/s^2), ENU frame (Z up); enters as -g_vec
    Jxx = _base.Jxx  # 绕机体 x 轴(roll)转动惯量(kg*m^2),用在角速度动力学 om_dot=(tau-omx(J*om))/J
                      # Moment of inertia about body x-axis (roll, kg*m^2), used in om_dot=(tau-omx(J*om))/J
    Jyy = _base.Jyy  # 绕机体 y 轴(pitch)转动惯量(kg*m^2)
                      # Moment of inertia about body y-axis (pitch, kg*m^2)
    Jzz = _base.Jzz  # 绕机体 z 轴(yaw)转动惯量(kg*m^2),比 Jxx/Jyy 大,yaw 方向惯性更强
                      # Moment of inertia about body z-axis (yaw, kg*m^2); larger than Jxx/Jyy
    kd  = _base.kd   # 线性空气阻力系数,vel_dot 里的 -kd*vel 项,模拟桨叶/机身阻力
                      # Linear drag coefficient, the -kd*vel term in vel_dot (models prop/airframe drag)
    Tmin    = _base.Tmin    # 总推力下限(N),四个电机合力最小值,留给 acados 的输入约束 lbu[0]
                              # Min total thrust (N), feeds acados input constraint lbu[0]
    Tmax    = _base.Tmax    # 总推力上限(N) = 2*m*g,即最大能输出悬停推力的 2 倍(留够机动余量)
                              # Max total thrust (N) = 2*m*g, i.e. 2x hover thrust for maneuvering margin
    tau_max = _base.tau_max # roll/pitch 力矩约束上限(Nm),输入约束 ubu[1],ubu[2]
                              # Roll/pitch torque limit (Nm), input constraint ubu[1], ubu[2]
    tau_psi = _base.tau_psi # yaw 力矩约束上限(Nm),比 tau_max 小很多(0.2 vs 0.5)——
                             # 实测电机能输出的偏航力矩本来就比横滚/俯仰小
                             # Yaw torque limit (Nm), much smaller than tau_max (0.2 vs 0.5) —
                             # motors simply can't produce as much yaw torque as roll/pitch
    nx = 13  # 状态维度: pos(3)+vel(3)+quat(4)+omega(3) = 13
              # State dimension: pos(3)+vel(3)+quat(4)+omega(3) = 13
    nu = 4   # 控制维度: 总推力 T(1) + 力矩 tau_x,tau_y,tau_z(3) = 4
              # Control dimension: total thrust T(1) + torques tau_x,tau_y,tau_z(3) = 4

    # --- acados 控制器自己的时域/代价权重,独立调(不影响 offboard_test 那边) ---
    # acados controller's own horizon/cost weights, tuned independently
    # (changing these never affects the offboard_test CasADi/IPOPT side).
    N  = 10   # 预测时域步数(horizon steps)。2026-07-08 复测过 dt=0.05/N=20
              # (同样 1s 时域但步数翻倍,20Hz 控制环)——即使叠加了 dJ/c_xy
              # 建模、MERIT_BACKTRACKING、方案(a)受控attach、descend 收敛判据
              # 这些后续修复,同一个 ry=0.05 工况下 attach 瞬态 pos_err 峰值仍
              # 从 <0.05m 恶化到 1.16m(虽然这次没像最早那次一样发散),solve
              # 耗时也从 1-5ms 涨到 6-9ms。跟当年结论一致,改回这组固定值。
              # Horizon length (steps). 2026-07-08 retested dt=0.05/N=20 (same 1s
              # horizon, double the steps, 20Hz control loop) — even with the
              # later dJ/c_xy modeling, MERIT_BACKTRACKING, controlled attach
              # (方案a), and descend convergence-gate fixes, the same ry=0.05
              # case still got a worse attach transient (pos_err peak 0.05m ->
              # 1.16m, though it no longer diverged outright) and slower solves
              # (1-5ms -> 6-9ms). Confirms the original finding; reverted.
    dt = 0.1  # 每步时长(s),N*dt=1.0s 是 NMPC 往前看的预测时域长度;dt 同时也是
              # ROS2 控制循环周期(acados_nmpc_node.py 的 self.timer 用的就是这个值)
              # Step length (s). N*dt=1.0s is the NMPC look-ahead horizon; dt also
              # doubles as the ROS2 control-loop period (self.timer in acados_nmpc_node.py)

    # --- Bryson's rule 无量纲化: Q_ii = 1/(典型尺度)^2,R_jj 同理 ---
    # Bryson's rule non-dimensionalization: Q_ii = 1/(typical scale)^2, same for R_jj.
    # 诊断发现 res_stat 起点 2.16、收敛率卡在 ~0.9(干净环境复测下确认是真实
    # 问题,不是环境干扰:res_stat 峰值能飙到 6.26e5,彻底数值崩溃)。根因之一
    # 是 Q_pos=4 对 R_torque=0.1 的量纲失衡,Hessian 在不同方向曲率差几十倍。
    # 每个分量在各自"典型尺度"下对 cost 的贡献都归一化到 1,让 Gauss-Newton
    # 的 Hessian 各方向曲率均衡。下面这组 L_* 就是各状态/控制误差分量的
    # "容许误差量级"(单位跟对应物理量一致),取得越小代表对应方向罚得越重。
    # Diagnosis found res_stat starting at 2.16 with convergence rate stuck at
    # ~0.9 (confirmed real, not environment noise, via a clean-env retest —
    # res_stat could spike to 6.26e5, a full numerical blow-up). One root cause:
    # Q_pos=4 vs R_torque=0.1 is a unit mismatch, making the Hessian's curvature
    # differ by tens of times across directions. Normalizing each component's
    # cost contribution to 1 at its own "typical scale" balances the Gauss-Newton
    # Hessian's curvature across directions. The L_* values below are each
    # state/control error's "tolerable error magnitude" (same units as the
    # physical quantity) — smaller L means that direction is penalized harder.
    L_pos   = 0.1       # 位置误差容许量级(m):0.1m 的跟踪误差算"正常"
                          # Tolerable position error (m): 0.1m tracking error is "normal"
    L_vel   = 0.3       # 速度误差容许量级(m/s) = 轨迹特征速度 r*w(半径1m*角速度0.3rad/s)
                          # Tolerable velocity error (m/s) = trajectory's characteristic speed r*w
    L_att   = 0.05      # 姿态误差(四元数虚部)容许量级,约对应 5.7° 的姿态偏差
                          # Tolerable attitude error (quaternion vector part), ~5.7 degrees
    L_omega = 0.3       # 角速度误差容许量级(rad/s),跟圆形轨迹的恒定 yaw_rate 同量级
                          # Tolerable angular-rate error (rad/s), same order as the circle's constant yaw_rate
    L_T     = 2.0       # 推力误差容许量级(N),相对悬停推力(~20N)的合理波动范围
                          # Tolerable thrust error (N), a reasonable swing around hover thrust (~20N)
    L_tau_rp  = tau_max  # roll/pitch 力矩误差容许量级,直接取约束值本身(控制量天然不会超约束)
                           # Tolerable roll/pitch torque error — just use the constraint itself
                           # (the control input can never exceed it anyway)
    L_tau_yaw = tau_psi  # yaw 力矩误差容许量级,同理取 tau_psi
                           # Tolerable yaw torque error, likewise taken as tau_psi

    # cost_y_expr = [tracking_error_sym(x,xr)(12维); u - u_hover(4维)] -> 16维
    # cost_y_expr = [tracking_error_sym(x,xr) (12-dim); u - u_hover (4-dim)] -> 16-dim
    # Q 作用在 tracking_error_sym 的 12 维输出上,按 4 个 3x3 块对应:
    # Q acts on the 12-dim output of tracking_error_sym, as 4 blocks of 3x3:
    #   block 1 = 位置误差(ep)权重 = 1/L_pos^2  = 100
    #             position error (ep) weight = 1/L_pos^2 = 100
    #   block 2 = 速度误差(ev)权重 = 1/L_vel^2  ≈ 11.1
    #             velocity error (ev) weight = 1/L_vel^2 ≈ 11.1
    #   block 3 = 姿态误差(eq_vec,四元数虚部)权重 = 1/L_att^2 = 400
    #             attitude error (eq_vec, quaternion vector part) weight = 1/L_att^2 = 400
    #   block 4 = 角速度误差(eomega)权重 = 1/L_omega^2 ≈ 11.1
    #             angular-rate error (eomega) weight = 1/L_omega^2 ≈ 11.1
    Q = np.block([
        [(1/L_pos**2)*np.eye(3),   np.zeros((3,3)),         np.zeros((3,3)),          np.zeros((3,3))],
        [np.zeros((3,3)),          (1/L_vel**2)*np.eye(3),  np.zeros((3,3)),          np.zeros((3,3))],
        [np.zeros((3,3)),          np.zeros((3,3)),         (1/L_att**2)*np.eye(3),   np.zeros((3,3))],
        [np.zeros((3,3)),          np.zeros((3,3)),         np.zeros((3,3)),          (1/L_omega**2)*np.eye(3)]
    ])
    # R 作用在 u-u_hover 的 4 维残差上,对角线依次是:
    # R acts on the 4-dim residual u-u_hover; the diagonal entries are:
    #   [0] 推力误差权重    = 1/L_T**2      = 0.25
    #       thrust error weight = 1/L_T**2 = 0.25
    #   [1] roll 力矩误差权重 = 1/L_tau_rp**2  = 4.0
    #       roll torque error weight = 1/L_tau_rp**2 = 4.0
    #   [2] pitch 力矩误差权重 = 1/L_tau_rp**2 = 4.0(跟 roll 共用同一个典型尺度)
    #       pitch torque error weight = 1/L_tau_rp**2 = 4.0 (shares the same scale as roll)
    #   [3] yaw 力矩误差权重  = 1/L_tau_yaw**2 = 25.0(yaw 力矩约束更紧,所以罚得更重)
    #       yaw torque error weight = 1/L_tau_yaw**2 = 25.0 (tighter constraint -> heavier penalty)
    R = np.diag([1/L_T**2, 1/L_tau_rp**2, 1/L_tau_rp**2, 1/L_tau_yaw**2])

    # 终端代价(cost_y_expr_e,只有 12 维 tracking_error_sym,没有控制项)跟
    # stage 的 Q 保持相同的相对比例放大,逼着预测轨迹末端更精确地落在参考点上:
    #   位置 20 倍(2000)、速度 1 倍(11.1,不额外加重)、姿态 2 倍(800)、
    #   角速度 0.2 倍(2.22,末端不需要严格限制角速度,放松反而利于求解)
    # Terminal cost (cost_y_expr_e, just the 12-dim tracking_error_sym, no control
    # term) scales the stage Q by the same relative factors, forcing the predicted
    # trajectory's endpoint closer to the reference:
    #   position x20 (2000), velocity x1 (11.1, no extra weight), attitude x2 (800),
    #   angular rate x0.2 (2.22 — no need to tightly constrain terminal angular
    #   rate; relaxing it actually helps the solver)
    P = np.block([
        [20*(1/L_pos**2)*np.eye(3), np.zeros((3,3)),           np.zeros((3,3)),            np.zeros((3,3))],
        [np.zeros((3,3)),           (1/L_vel**2)*np.eye(3),    np.zeros((3,3)),            np.zeros((3,3))],
        [np.zeros((3,3)),           np.zeros((3,3)),           2*(1/L_att**2)*np.eye(3),   np.zeros((3,3))],
        [np.zeros((3,3)),           np.zeros((3,3)),           np.zeros((3,3)),            0.2*(1/L_omega**2)*np.eye(3)]
    ])

    # 悬停时的参考控制量:推力=重力(T=m*g,刚好抵消重力),三个力矩都是 0
    # (悬停不需要任何转动)。cost 里 u-u_hover 算的就是相对这个基准的偏差,
    # 而不是相对绝对零的偏差——这样悬停时残差天然是 0,不会被无谓惩罚。
    # Reference control at hover: thrust = gravity (T=m*g, exactly cancels
    # gravity), all three torques are 0 (no rotation needed at hover). The cost
    # computes u-u_hover relative to this baseline, not relative to absolute
    # zero — so the residual is naturally 0 at hover and isn't penalized for no reason.
    u_hover = np.array([m * g, 0.0, 0.0, 0.0])


p = AcadosParams()
