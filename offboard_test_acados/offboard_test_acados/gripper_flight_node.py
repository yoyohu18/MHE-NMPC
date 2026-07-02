#!/usr/bin/env python3
"""磁吸夹爪演示飞行节点 (MAVROS offboard, ENU 位置 setpoint)。

飞行剖面(全程位置 setpoint 控制,不涉及 NMPC):
  1. WARMUP   : 起飞前预热 setpoint 流(OFFBOARD 要求先有 setpoint),解锁 + 切 OFFBOARD
  2. TAKEOFF  : 在 spawn 点正上方 takeoff_z 悬停稳一会儿
  3. GOTO_BOX : 平移到 box 正上方 (box_x, box_y, over_z) 并悬停;同时持续发
                /gripper/enable=true。无人机一旦在 box 上方、相对速度近零,
                proximity_gripper_node 就会触发 attach 把 box 焊上来。
  4. LIFT     : 带着 box 爬升到 hover_z 并一直悬停。

MAVROS 本地系是 ENU,且原点在 EKF/spawn 点(= gz 世界原点),所以 setpoint
的 (x,y,z) 直接就是 gz ENU 坐标:box 在 gz (1,0) → setpoint x=1,y=0。

所有航点/高度/驻留时间都是 ROS 参数。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# MAVROS 发 /mavros/state 和 /mavros/local_position/pose 用的是 BEST_EFFORT
# sensor QoS;默认 RELIABLE 订阅会 QoS 不兼容、收不到任何消息。
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)


WARMUP, TAKEOFF, GOTO_BOX, LIFT, DROP = range(5)


class GripperFlightNode(Node):
    def __init__(self):
        super().__init__('gripper_flight_node')

        self.declare_parameter('takeoff_z', 1.0)
        self.declare_parameter('box_x', 1.0)
        self.declare_parameter('box_y', 0.0)
        self.declare_parameter('over_z', 0.6)      # 悬停在 box 上方的高度
        self.declare_parameter('hover_z', 1.3)     # 吸附后爬升悬停高度
        self.declare_parameter('climb_rate', 0.4)  # LIFT 段 setpoint 爬升速率 [m/s]
        self.declare_parameter('takeoff_dwell', 5.0)
        self.declare_parameter('over_dwell', 12.0)  # 在 box 上方悬停多久(等 attach)
        self.declare_parameter('lift_dwell', 10.0)  # 带载悬停多久后投放
        self.declare_parameter('pos_tol', 0.25)
        self.declare_parameter('enable_topic', '/gripper/enable')

        g = self.get_parameter
        self.takeoff_z = float(g('takeoff_z').value)
        self.box_x = float(g('box_x').value)
        self.box_y = float(g('box_y').value)
        self.over_z = float(g('over_z').value)
        self.hover_z = float(g('hover_z').value)
        self.climb_rate = float(g('climb_rate').value)
        self.takeoff_dwell = float(g('takeoff_dwell').value)
        self.over_dwell = float(g('over_dwell').value)
        self.lift_dwell = float(g('lift_dwell').value)
        self.pos_tol = float(g('pos_tol').value)

        self.state = State()
        self.pose = None
        self.counter = 0
        self.phase = WARMUP
        self.phase_t0 = None
        self.lift_z = self.over_z

        self.create_subscription(
            State, '/mavros/state', self.on_state, SENSOR_QOS)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            SENSOR_QOS)
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.enable_pub = self.create_publisher(
            Bool, g('enable_topic').value, 10)

        self.arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')

        self.create_timer(0.02, self.tick)   # 50 Hz setpoint stream
        self.get_logger().info(
            f'gripper_flight ready: box=({self.box_x},{self.box_y}) '
            f'over_z={self.over_z} hover_z={self.hover_z}')

    def on_state(self, msg):
        self.state = msg

    def on_pose(self, msg):
        self.pose = msg.pose.position

    # --- helpers ---
    def publish_sp(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        # 必须给合法四元数(单位阵 -> yaw=0);留默认 (0,0,0,0) 会让 MAVROS
        # 从零四元数解出非法/NaN 的 yaw setpoint,PX4 位置控制可能因此发散飞走。
        sp.pose.orientation.w = 1.0
        self.sp_pub.publish(sp)

    def publish_enable(self, on):
        self.enable_pub.publish(Bool(data=bool(on)))

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def at(self, x, y, z):
        if self.pose is None:
            return False
        return math.sqrt((self.pose.x - x) ** 2 +
                         (self.pose.y - y) ** 2 +
                         (self.pose.z - z) ** 2) < self.pos_tol

    def enter(self, phase, name):
        self.phase = phase
        self.phase_t0 = self.now_s()
        self.get_logger().info(f'--> phase {name}')

    # --- main loop ---
    def tick(self):
        # WARMUP: 先灌 setpoint,再解锁 + 切 OFFBOARD。
        if self.phase == WARMUP:
            self.publish_sp(0.0, 0.0, self.takeoff_z)
            self.counter += 1
            if self.counter < 100:
                return
            if not self.state.connected:
                return
            if self.state.mode != 'OFFBOARD':
                if self.set_mode.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = 'OFFBOARD'
                    self.set_mode.call_async(req)
                return
            if not self.state.armed:
                if self.arming.service_is_ready():
                    req = CommandBool.Request()
                    req.value = True
                    self.arming.call_async(req)
                return
            self.enter(TAKEOFF, 'TAKEOFF')
            return

        # 任何时候掉出 OFFBOARD / 解锁就重试,保持 setpoint 流不断。
        if self.state.mode != 'OFFBOARD' and self.set_mode.service_is_ready():
            req = SetMode.Request(); req.custom_mode = 'OFFBOARD'
            self.set_mode.call_async(req)
        if not self.state.armed and self.arming.service_is_ready():
            req = CommandBool.Request(); req.value = True
            self.arming.call_async(req)

        if self.phase == TAKEOFF:
            self.publish_sp(0.0, 0.0, self.takeoff_z)
            if self.at(0.0, 0.0, self.takeoff_z) and \
                    self.now_s() - self.phase_t0 > self.takeoff_dwell:
                self.enter(GOTO_BOX, 'GOTO_BOX')

        elif self.phase == GOTO_BOX:
            self.publish_sp(self.box_x, self.box_y, self.over_z)
            self.publish_enable(True)   # 使能夹取(条件满足时 proximity 会 attach)
            # 只按"在 box 上方驻留够久"推进,不强求收敛到 0.25m 容差:一旦吸上
            # 0.8kg 载荷,位置环要克服多出的重力+力臂,稳态可能差几十厘米,卡在
            # at() 上就永远不上升了。attach 由 proximity 节点独立按几何条件触发,
            # 这里只要给足时间让无人机飞到 box 上方、proximity 完成 attach 即可。
            if self.now_s() - self.phase_t0 > self.over_dwell:
                self.get_logger().info(
                    'loiter over box done -> lifting (payload attached by '
                    'proximity node if geometry met)')
                self.lift_z = self.over_z   # 从当前高度开始平滑爬升
                self.enter(LIFT, 'LIFT')

        elif self.phase == LIFT:
            # 平滑地把 z setpoint 以 climb_rate 抬到 hover_z,避免一步 0.7m 的
            # 位置跳变叠加载荷突变把姿态打崩。
            self.lift_z = min(self.hover_z, self.lift_z + self.climb_rate * 0.02)
            self.publish_sp(self.box_x, self.box_y, self.lift_z)
            self.publish_enable(True)   # 保持使能,避免 enable 拉低触发 detach
            # 爬到 hover_z 并带载悬停 lift_dwell 秒后投放。
            if self.lift_z >= self.hover_z - 1e-3 and \
                    self.now_s() - self.phase_t0 > self.lift_dwell:
                self.enter(DROP, 'DROP')

        elif self.phase == DROP:
            # 停发 enable=True、改发 enable=False:接近节点 on_enable(False) 会
            # 立刻 detach 且不再重抓,盒子自由落体、无人机瞬间失重上冲(预期物理,
            # 不平滑)。无人机继续在 hover_z 悬停。
            self.publish_sp(self.box_x, self.box_y, self.hover_z)
            self.publish_enable(False)
            if not getattr(self, '_drop_logged', False):
                self.get_logger().info('DROP: released payload (enable=False)')
                self._drop_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = GripperFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
