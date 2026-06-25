#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard_node')
        self.state = State()
        self.counter = 0
        self.start_time = None

        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_cb, 10)

        self.pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info('节点启动')

    def state_cb(self, msg):
        self.state = msg

    def get_circle_setpoint(self, t):
        # 先悬停3秒
        if t < 3.0:
            x = 0.0
            y = 0.0
            z = 2.0
        else:
            # 圆形轨迹
            # 半径 1m，角速度 0.5 rad/s
            tc = t - 3.0
            r = 1.0
            w = 0.5
            x = r * math.cos(w * tc)
            y = r * math.sin(w * tc)
            z = 2.0
        return x, y, z

    def timer_cb(self):
        # 先发100帧预热
        if self.counter < 100:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.z = 2.0
            self.pos_pub.publish(pose)
            self.counter += 1
            return

        # 记录起始时间
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # 计算当前时间
        elapsed = (self.get_clock().now() - 
                   self.start_time).nanoseconds / 1e9

        # 计算圆形轨迹目标点
        x, y, z = self.get_circle_setpoint(elapsed)

        # 发布目标位置
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pos_pub.publish(pose)

        if not self.state.connected:
            return

        # 切换 OFFBOARD
        if self.state.mode != 'OFFBOARD':
            if self.mode_client.service_is_ready():
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.mode_client.call_async(req)
                self.get_logger().info('切换 OFFBOARD 模式')
            return

        # 解锁
        if not self.state.armed:
            if self.arming_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info('发送解锁指令')
            return

        # 打印当前位置（每50帧打印一次）
        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(
                f't={elapsed:.1f}s 目标: x={x:.2f} y={y:.2f} z={z:.2f}')

def main():
    rclpy.init()
    node = OffboardNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

