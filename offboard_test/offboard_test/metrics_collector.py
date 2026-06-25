#!/usr/bin/env python3
# 自动调参用:订阅 /nmpc/tracking_error 和 /mavros/local_position/odom,
# 跑满 RUN_DURATION 秒后计算稳态跟踪误差均值,并检测过程中是否摔机(z 跌破阈值),
# 把结果写成一行 JSON 到 OUTPUT_FILE,然后自己退出。给 tune_nmpc.py 当裁判用。

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

RUN_DURATION  = float(os.environ.get('TUNE_RUN_DURATION', '90'))
STEADY_WINDOW = float(os.environ.get('TUNE_STEADY_WINDOW', '20'))
CRASH_Z       = float(os.environ.get('TUNE_CRASH_Z', '-0.3'))
OUTPUT_FILE   = os.environ.get('TUNE_OUTPUT_FILE', '/tmp/nmpc_tune_metrics.json')


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.err_t = []
        self.err_v = []
        self.min_z = None
        self.t0 = time.time()

        # /mavros/local_position/odom 是 MAVROS 用 BEST_EFFORT 发的,订阅方 QoS
        # 必须匹配,否则收不到任何消息(之前就是因为漏了这个,导致 0 样本)
        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Float64, '/nmpc/tracking_error', self.err_cb, 10)
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_cb,
            mavros_sensor_qos)

        self.create_timer(RUN_DURATION, self.finish)
        self.get_logger().info(
            f'metrics_collector 运行 {RUN_DURATION:.0f}s 后写出结果到 {OUTPUT_FILE}')

    def err_cb(self, msg):
        self.err_t.append(time.time() - self.t0)
        self.err_v.append(msg.data)

    def odom_cb(self, msg):
        z = msg.pose.pose.position.z
        if self.min_z is None or z < self.min_z:
            self.min_z = z

    def finish(self):
        steady = [v for t, v in zip(self.err_t, self.err_v)
                  if t >= RUN_DURATION - STEADY_WINDOW]
        mean_error = float(sum(steady) / len(steady)) if steady else None
        crashed = bool(self.min_z is not None and self.min_z < CRASH_Z)

        result = {
            'mean_error': mean_error,
            'min_z': self.min_z,
            'crashed': crashed,
            'n_samples': len(steady),
        }
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(result, f)
        self.get_logger().info(f'结果已写出: {result}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
