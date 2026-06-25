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
from std_msgs.msg import Float64, Float64MultiArray

from offboard_test.nmpc_node import quat_to_rotmat

from .mhe_params import p as mhe_p
from .mhe_solver_builder import ensure_mhe_ocp_solver


class MHENode(Node):
    def __init__(self):
        super().__init__('mhe_node')
        self.get_logger().info('MHE node starting, building/loading solver...')
        self.solver = ensure_mhe_ocp_solver()

        self.x_meas = None
        self.u_known = None

        self.y_buf = []  # 测量缓冲区,最多 N+1 帧
        self.u_buf = []  # 已知输入缓冲区,最多 N 帧

        self.x0_bar = None
        self.x_guess = None
        self.m_est = mhe_p.m_nominal
        self.counter = 0

        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_cb, mavros_sensor_qos)
        self.u_opt_sub = self.create_subscription(
            Float64MultiArray, '/acados_nmpc/u_opt', self.u_opt_cb, 10)

        self.mass_pub = self.create_publisher(
            Float64, '/acados_nmpc/mhe_mass_estimate', 10)

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

    def u_opt_cb(self, msg):
        u = np.array(msg.data)
        if u.shape[0] == mhe_p.nu_known and np.all(np.isfinite(u)):
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

        yref_0 = np.concatenate([y_win[0], np.zeros(nw), self.x0_bar])
        self.solver.set(0, 'yref', yref_0)
        self.solver.set(0, 'p', u_win[0])
        self.solver.set(0, 'x', self.x_guess[0])

        for j in range(1, N):
            yref = np.concatenate([y_win[j], np.zeros(nw)])
            self.solver.set(j, 'yref', yref)
            self.solver.set(j, 'p', u_win[j])
            self.solver.set(j, 'x', self.x_guess[j])

        self.solver.set(N, 'x', self.x_guess[N])

        status = self.solver.solve()
        if status != 0:
            self.get_logger().warn(
                f'MHE solve failed [status={status}], skipping this window')
            return

        x_sol = [self.solver.get(i, 'x') for i in range(N + 1)]
        self.m_est = float(x_sol[N][nx])

        # 滑动窗口 shift:下一窗口的到达代价先验取这一次窗口里 x[1] 的估计值,
        # 跟 acados_nmpc_node 里 NMPC 的 warm-start shift 是同一套思路。
        self.x0_bar = x_sol[1].copy()
        self.x_guess = [x_sol[min(i + 1, N)].copy() for i in range(N + 1)]

        self.mass_pub.publish(Float64(data=self.m_est))

        self.counter += 1
        if self.counter % 20 == 0:
            self.get_logger().info(f'MHE mass estimate: {self.m_est:.3f} kg')


def main():
    rclpy.init()
    node = MHENode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()