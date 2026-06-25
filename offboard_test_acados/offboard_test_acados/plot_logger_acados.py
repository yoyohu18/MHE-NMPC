#!/usr/bin/env python3
# offboard_test/plot_logger.py 的拷贝,话题名换成 acados_nmpc_node 发布的
# /acados_nmpc/... 系列——原版 plot_logger 是硬编码 /nmpc/... 的,对这个节点会
# 一声不响地什么都画不出来,所以需要单独一份,不能直接复用。

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
SAVE_PERIOD = 5.0


class PlotLoggerAcados(Node):
    def __init__(self):
        super().__init__('plot_logger_acados')

        self.ref_path = None
        self.actual_xyz = []
        self.err_t = []
        self.err_v = []
        self.t0 = time.time()

        self.create_subscription(
            Path, '/acados_nmpc/reference_path', self.ref_cb, 10)
        self.create_subscription(
            Path, '/acados_nmpc/actual_path', self.actual_cb, 10)
        self.create_subscription(
            Float64, '/acados_nmpc/tracking_error', self.err_cb, 10)

        self.create_timer(SAVE_PERIOD, self.periodic_save)

        self.get_logger().info(
            f'plot_logger_acados started, recording, auto-updating '
            f'acados_nmpc_test_latest.png every {SAVE_PERIOD:.0f}s')

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
        self.save_plot(os.path.join(RESULTS_DIR, 'acados_nmpc_test_latest.png'))

    def save_plot(self, out_path=None):
        if not self.actual_xyz and self.ref_path is None:
            self.get_logger().warn('No data received yet, skipping plot')
            return

        os.makedirs(RESULTS_DIR, exist_ok=True)
        if out_path is None:
            ts = time.strftime('%Y%m%d_%H%M%S')
            out_path = os.path.join(RESULTS_DIR, f'acados_nmpc_test_{ts}.png')

        fig, axes = plt.subplots(2, 2, figsize=(12, 9))

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
        ax.set_title('XY trajectory (acados)'); ax.axis('equal'); ax.legend(); ax.grid(True)

        ax = axes[0, 1]
        if self.actual_xyz:
            t_z = [v[0] for v in self.actual_xyz]
            z_v = [v[3] for v in self.actual_xyz]
            ax.plot(t_z, z_v, 'b-', label='actual z')
        ax.set_xlabel('t [s]'); ax.set_ylabel('z [m]')
        ax.set_title('Altitude vs time (acados)'); ax.legend(); ax.grid(True)

        ax = axes[1, 0]
        if self.err_t:
            ax.plot(self.err_t, self.err_v, 'g-')
        ax.set_xlabel('t [s]'); ax.set_ylabel('position error [m]')
        ax.set_title('Tracking error vs time (acados)'); ax.grid(True)

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
        ax3d.set_title('3D trajectory (acados)'); ax3d.legend()

        fig.tight_layout()
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Plot saved: {out_path}')


def main():
    rclpy.init()
    node = PlotLoggerAcados()
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
