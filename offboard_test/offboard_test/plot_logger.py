#!/usr/bin/env python3
# 订阅 nmpc_node 发布的参考/实际轨迹和跟踪误差,每 SAVE_PERIOD 秒自动把当前
# 累积数据画图保存到 nmpc_test_latest.png(覆盖),不需要等进程退出才能看到图。
# 退出(Ctrl+C / 节点关闭)时再额外存一份带时间戳的最终版本。

import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float64

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RESULTS_DIR = os.path.expanduser('~/ros2_ws_HJH/nmpc_test_results')
SAVE_PERIOD = 5.0  # 秒,定期把当前数据画图覆盖保存,不必等进程退出


class PlotLogger(Node):
    def __init__(self):
        super().__init__('plot_logger')

        self.ref_path = None
        self.actual_xyz = []   # [(t, x, y, z)]
        self.err_t = []
        self.err_v = []
        self.t0 = time.time()

        self.create_subscription(
            Path, '/nmpc/reference_path', self.ref_cb, 10)
        self.create_subscription(
            Path, '/nmpc/actual_path', self.actual_cb, 10)
        self.create_subscription(
            Float64, '/nmpc/tracking_error', self.err_cb, 10)

        self.create_timer(SAVE_PERIOD, self.periodic_save)

        self.get_logger().info(
            f'plot_logger 已启动,记录中,每 {SAVE_PERIOD:.0f}s '
            f'自动更新 nmpc_test_latest.png')

    def ref_cb(self, msg):
        self.ref_path = msg

    def actual_cb(self, msg):
        if not msg.poses:
            return
        ps = msg.poses[-1]
        t = time.time() - self.t0
        p = ps.pose.position
        self.actual_xyz.append((t, p.x, p.y, p.z))

    def err_cb(self, msg):
        self.err_t.append(time.time() - self.t0)
        self.err_v.append(msg.data)

    def periodic_save(self):
        self.save_plot(os.path.join(RESULTS_DIR, 'nmpc_test_latest.png'))

    def save_plot(self, out_path=None):
        if not self.actual_xyz and self.ref_path is None:
            self.get_logger().warn('没有收到任何数据,跳过画图')
            return

        os.makedirs(RESULTS_DIR, exist_ok=True)
        if out_path is None:
            ts = time.strftime('%Y%m%d_%H%M%S')
            out_path = os.path.join(RESULTS_DIR, f'nmpc_test_{ts}.png')

        fig, axes = plt.subplots(2, 2, figsize=(12, 9))

        # XY 轨迹对比
        ax = axes[0, 0]
        if self.ref_path is not None and self.ref_path.poses:
            rx = [p.pose.position.x for p in self.ref_path.poses]
            ry = [p.pose.position.y for p in self.ref_path.poses]
            ax.plot(rx, ry, 'r--', label='reference')
        if self.actual_xyz:
            ax_x = [v[1] for v in self.actual_xyz]
            ax_y = [v[2] for v in self.actual_xyz]
            ax.plot(ax_x, ax_y, 'b-', label='actual')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.set_title('XY trajectory'); ax.axis('equal'); ax.legend(); ax.grid(True)

        # Z vs time
        ax = axes[0, 1]
        if self.actual_xyz:
            t_z = [v[0] for v in self.actual_xyz]
            z_v = [v[3] for v in self.actual_xyz]
            ax.plot(t_z, z_v, 'b-', label='actual z')
        ax.set_xlabel('t [s]'); ax.set_ylabel('z [m]')
        ax.set_title('Altitude vs time'); ax.legend(); ax.grid(True)

        # 跟踪误差 vs time
        ax = axes[1, 0]
        if self.err_t:
            ax.plot(self.err_t, self.err_v, 'g-')
        ax.set_xlabel('t [s]'); ax.set_ylabel('position error [m]')
        ax.set_title('Tracking error vs time'); ax.grid(True)

        # 3D 轨迹
        ax3d = fig.add_subplot(2, 2, 4, projection='3d')
        if self.ref_path is not None and self.ref_path.poses:
            rx = [p.pose.position.x for p in self.ref_path.poses]
            ry = [p.pose.position.y for p in self.ref_path.poses]
            rz = [p.pose.position.z for p in self.ref_path.poses]
            ax3d.plot(rx, ry, rz, 'r--', label='reference')
        if self.actual_xyz:
            ax_x = [v[1] for v in self.actual_xyz]
            ax_y = [v[2] for v in self.actual_xyz]
            ax_z = [v[3] for v in self.actual_xyz]
            ax3d.plot(ax_x, ax_y, ax_z, 'b-', label='actual')
        ax3d.set_xlabel('x'); ax3d.set_ylabel('y'); ax3d.set_zlabel('z')
        ax3d.set_title('3D trajectory'); ax3d.legend()

        fig.tight_layout()
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'图片已保存: {out_path}')


def main():
    rclpy.init()
    node = PlotLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
