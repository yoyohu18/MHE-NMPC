#!/usr/bin/env python3
# acados 版 NMPC 节点。状态机(预热→等EKF→切OFFBOARD→解锁→飞到起点→NMPC接管)
# 跟 offboard_test/nmpc_node.py 的 NMPCNode 完全一致,直接照搬,只有 solve_nmpc()
# 内部换成调 acados 求解器,其它行为(包括坐标系、限幅、推力归一化、body_rate
# 斜坡)都保持不变,方便跟 CasADi/IPOPT 版本直接对比。

import math
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
from mavros_msgs.srv import CommandBool, SetMode
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
        # 跟 offboard_test 用同一个实测标定值,两边可比
        self.hover_thrust_pct = 0.729
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
        for i in range(p.N):
            self.solver.set(i, 'x', Xref_win[:, i])
            self.solver.set(i, 'u', p.u_hover)
        self.solver.set(p.N, 'x', Xref_win[:, p.N])

    def solve_nmpc(self, x_cur, t_ref):
        Xref_win = self.ref_window_fn(t_ref, p.N, p.dt, z_hover=self.z_hover)

        self.solver.set(0, 'lbx', x_cur)
        self.solver.set(0, 'ubx', x_cur)
        for i in range(p.N + 1):
            self.solver.set(i, 'p', Xref_win[:, i])

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
        norm = self.hover_thrust_pct * T / (p.m * p.g)
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
