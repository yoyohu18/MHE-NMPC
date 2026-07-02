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
from actuator_msgs.msg import Actuators

from offboard_test.nmpc_node import quat_to_rotmat

from .mhe_params import p as mhe_p
from .mhe_solver_builder import ensure_mhe_ocp_solver


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


class MHENode(Node):
    def __init__(self):
        super().__init__('mhe_node')
        self.get_logger().info('MHE node starting, building/loading solver...')
        self.solver = ensure_mhe_ocp_solver()

        self.x_meas = None
        self.u_known = None
        self.thrust_phys = None  # 电机转速反算的真实总推力(见 MOTOR_CONSTANT 注释)

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
        # PX4 发给 Gazebo 电机模型的转速指令(经 ros_gz_bridge 桥接,跟
        # prop_joint_state_publisher 用的是同一个话题/同一套 QoS=depth10)。
        # 话题名做成参数:mass_changer 场景是 x500_payload_0,夹爪场景是空机
        # x500_0。默认保持原值,不影响现有 run_sitl_acados.sh。
        self.declare_parameter('motor_speed_topic',
                               '/x500_payload_0/command/motor_speed')
        motor_topic = self.get_parameter('motor_speed_topic').value
        self.motor_speed_sub = self.create_subscription(
            Actuators, motor_topic, self.motor_speed_cb, 10)

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

    def motor_speed_cb(self, msg):
        if len(msg.velocity) >= 4:
            w = np.array(msg.velocity[:4])
            if np.all(np.isfinite(w)):
                self.thrust_phys = float(
                    THRUST_CAL_GAIN * MOTOR_CONSTANT * np.sum(w * w))

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
            t_phys = self.thrust_phys if self.thrust_phys is not None else float('nan')
            self.get_logger().info(
                f'MHE mass estimate: {self.m_est:.3f} kg (T_phys={t_phys:.2f}N)')


def main():
    rclpy.init()
    node = MHENode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()