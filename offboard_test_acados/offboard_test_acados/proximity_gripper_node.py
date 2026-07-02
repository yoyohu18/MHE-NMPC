#!/usr/bin/env python3
"""接近触发夹爪节点 (ROS 2 + gz-transport)。

监听无人机和目标 box 的位姿,当**全部**条件满足时,往夹爪插件发 attach
命令,把 box 焊到机体上:
  - 水平距离 < r_xy
  - 相对高度 (无人机.z - 目标.z) ∈ [h_min, h_max]  (无人机在目标上方)
  - 机体与目标的相对速度模长 < v_rel_max  (避免瞬间刚性约束 + 速度失配
    产生冲击 jerk 把仿真打崩)
  - 收到外部"夹取使能"信号 /gripper/enable (std_msgs/Bool = true)

detach:收到 /gripper/release (std_msgs/Bool = true),或 enable 拉低时释放。

位姿来源 / attach 命令出口都走 **gz-transport 直连**,不经 ros_gz_bridge:
  - 订阅 gz 的 /world/<world>/pose/info(gz.msgs.Pose_V),里面每个 Pose 自带
    name(顶层 model 即 model 名),所以能可靠区分无人机和 box。
    (ros_gz_bridge 把 Pose_V 转 TFMessage 时 child_frame_id 会丢成空串,
     用不了 —— 这是当初踩的坑,所以这里直接读 gz。)
  - attach/detach 用 gz.msgs.StringMsg 直接 publish 到插件订阅的 topic。
无人机和 box 同在 gz ENU 世界系,速度用位姿有限差分得到。

只有"夹取使能/释放"这两个外部信号走 ROS(方便飞行节点/人工触发)。
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import gz.transport13 as gztransport
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg


class TrackedPose:
    """一个实体的最近一帧位姿 + 估计速度。"""

    def __init__(self):
        self.t = None
        self.pos = None
        self.vel = (0.0, 0.0, 0.0)

    def update(self, t, pos):
        if self.t is not None and t > self.t:
            dt = t - self.t
            if dt > 1e-4:
                self.vel = tuple((pos[i] - self.pos[i]) / dt for i in range(3))
        self.t = t
        self.pos = pos


class ProximityGripperNode(Node):
    def __init__(self):
        super().__init__('proximity_gripper_node')

        self.declare_parameter('world_name', 'gripper_test')
        self.declare_parameter('drone_model', 'x500_0')
        self.declare_parameter('parent_link', 'base_link')
        self.declare_parameter('targets', ['box'])
        self.declare_parameter('child_link', 'box_link')
        self.declare_parameter('r_xy', 0.15)
        self.declare_parameter('h_min', 0.3)
        self.declare_parameter('h_max', 1.5)
        self.declare_parameter('v_rel_max', 0.2)
        self.declare_parameter('attach_topic', '/gripper/attach')
        self.declare_parameter('detach_topic', '/gripper/detach')
        self.declare_parameter('enable_topic', '/gripper/enable')
        self.declare_parameter('release_topic', '/gripper/release')

        g = self.get_parameter
        world = g('world_name').value
        self.drone_model = g('drone_model').value
        self.parent_link = g('parent_link').value
        self.targets = list(g('targets').value)
        self.child_link = g('child_link').value
        self.r_xy = float(g('r_xy').value)
        self.h_min = float(g('h_min').value)
        self.h_max = float(g('h_max').value)
        self.v_rel_max = float(g('v_rel_max').value)

        self.enabled = False
        self.poses = {self.drone_model: TrackedPose()}
        for t in self.targets:
            self.poses[t] = TrackedPose()
        self.attached = set()

        # gz-transport:订阅 pose/info,广告 attach/detach。
        self.gz = gztransport.Node()
        self.pose_topic = f'/world/{world}/pose/info'
        self.gz.subscribe(Pose_V, self.pose_topic, self.on_pose_gz)
        self.attach_pub = self.gz.advertise(
            g('attach_topic').value, StringMsg)
        self.detach_pub = self.gz.advertise(
            g('detach_topic').value, StringMsg)

        # ROS:只收使能/释放。
        self.create_subscription(Bool, g('enable_topic').value,
                                 self.on_enable, 10)
        self.create_subscription(Bool, g('release_topic').value,
                                 self.on_release, 10)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f'proximity_gripper ready (gz-transport): drone={self.drone_model} '
            f'targets={self.targets} pose_topic={self.pose_topic} '
            f'r_xy={self.r_xy} h=[{self.h_min},{self.h_max}] '
            f'v_rel_max={self.v_rel_max}')

    # --- gz-transport 回调(gz 线程)---
    def on_pose_gz(self, msg: Pose_V):
        for p in msg.pose:
            if p.name in self.poses:
                t = p.header.stamp.sec + p.header.stamp.nsec * 1e-9
                pos = (p.position.x, p.position.y, p.position.z)
                self.poses[p.name].update(t, pos)

    # --- ROS 回调 ---
    def on_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.get_logger().info('gripper ENABLED')
        elif not msg.data and self.enabled:
            self.get_logger().info('gripper DISABLED -> releasing all')
        self.enabled = msg.data
        if not self.enabled:
            for tgt in list(self.attached):
                self.send_detach(tgt)

    def on_release(self, msg: Bool):
        if msg.data:
            for tgt in list(self.attached):
                self.send_detach(tgt)

    # --- 主判定 ---
    def tick(self):
        if not self.enabled:
            return
        drone = self.poses.get(self.drone_model)
        if drone is None or drone.pos is None:
            return
        for tgt in self.targets:
            if tgt in self.attached:
                continue
            tp = self.poses.get(tgt)
            if tp is None or tp.pos is None:
                continue
            if self.should_attach(drone, tp):
                self.send_attach(tgt)

    def should_attach(self, drone, tgt):
        dx = drone.pos[0] - tgt.pos[0]
        dy = drone.pos[1] - tgt.pos[1]
        dz = drone.pos[2] - tgt.pos[2]
        d_xy = math.hypot(dx, dy)
        if d_xy >= self.r_xy:
            return False
        if not (self.h_min <= dz <= self.h_max):
            return False
        v_rel = math.sqrt(sum(
            (drone.vel[i] - tgt.vel[i]) ** 2 for i in range(3)))
        if v_rel >= self.v_rel_max:
            return False
        self.get_logger().info(
            f'attach condition met: d_xy={d_xy:.3f} dz={dz:.3f} '
            f'v_rel={v_rel:.3f}')
        return True

    # --- 发命令(gz-transport)---
    def send_attach(self, tgt):
        payload = (f'{self.drone_model} {self.parent_link} '
                   f'{tgt} {self.child_link}')
        msg = StringMsg()
        msg.data = payload
        self.attach_pub.publish(msg)
        self.attached.add(tgt)
        self.get_logger().info(f'-> ATTACH "{payload}"')

    def send_detach(self, tgt):
        payload = (f'{self.drone_model} {self.parent_link} '
                   f'{tgt} {self.child_link}')
        msg = StringMsg()
        msg.data = payload
        self.detach_pub.publish(msg)
        self.attached.discard(tgt)
        self.get_logger().info(f'-> DETACH "{payload}"')


def main(args=None):
    rclpy.init(args=args)
    node = ProximityGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
