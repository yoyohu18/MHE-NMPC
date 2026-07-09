#!/usr/bin/env python3
# 把实时里程计广播成 map->base_link 的 TF,配合 urdf/masschanger/x500.urdf + robot_state_publisher
# 在 RViz 的 RobotModel 显示项里渲染出完整机身(机架+4个电机/桨叶)。取代之前
# 单一 mesh 的 Marker 方案(drone_marker_publisher.py)——同一份 odom,换成更
# "标准"的 URDF+TF 渲染路径,能看到电机/桨叶而不仅是中央机身,且姿态補偿直接
# 写在 URDF 的 visual origin 里,不需要在这里手算四元数 hack。

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DroneTFBroadcaster(Node):
    def __init__(self):
        super().__init__('drone_tf_broadcaster')
        self.br = TransformBroadcaster(self)

        mavros_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_cb, mavros_sensor_qos)

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = DroneTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
