#!/usr/bin/env python3
"""Unloaded PX4 cascade baseline; publishes ENU position/velocity references."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .figure8_reference import auto_ramp_time, build_reference_figure8


def reference_clock(elapsed, duration, stop_time):
    """C1 phase clock: smoothly reduce reference velocity to zero at the end."""
    if elapsed <= duration:
        return max(0.0, elapsed), 1.0
    s = min((elapsed - duration) / stop_time, 1.0)
    return (duration + stop_time * (s / 2 + math.sin(math.pi * s) / (2 * math.pi)),
            (1 + math.cos(math.pi * s)) / 2)


class PX4PIDBenchmark(Node):
    def __init__(self):
        super().__init__('px4_pid_benchmark')
        defaults = dict(fig8_r=1.0, fig8_w=0.3, fig8_ramp=0.0,
                        z_hover=3.0, dz=0.5, hover_time=2.0,
                        duration=60.0, stop_time=4.0, takeoff_time=8.0,
                        settle_time=2.0, position_tolerance=0.2,
                        warmup_time=2.0, telemetry_timeout=1.0,
                        rate_hz=50.0, velocity_feedforward=True, yaw_ramp=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.cfg.items():
            if not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('fig8_r', 'fig8_w', 'z_hover', 'duration', 'stop_time',
                     'takeoff_time', 'settle_time', 'position_tolerance',
                     'warmup_time', 'telemetry_timeout', 'rate_hz'):
            if self.cfg[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if self.cfg['rate_hz'] < 10 or self.cfg['warmup_time'] < 2:
            raise ValueError('Require rate_hz >= 10 and warmup_time >= 2')
        if self.cfg['hover_time'] < 0 or abs(self.cfg['dz']) >= self.cfg['z_hover']:
            raise ValueError('Require hover_time >= 0 and abs(dz) < z_hover')
        ramp = self.cfg['fig8_ramp']
        self.ramp = auto_ramp_time(self.cfg['fig8_w']) if ramp < 0 else (ramp or 4.0)
        self.state = None
        self.pose = None
        self.state_at = self.pose_at = None
        self.phase = 'WAIT'
        self.started = self.last_tick = self.stable_since = None
        self.origin = None
        self.active_once = False
        self.pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '~/reference', 10)
        self.phase_pub = self.create_publisher(String, '~/phase', 10)
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.create_timer(1.0 / self.cfg['rate_hz'], self.tick)
        self.get_logger().info('PX4 baseline ready; manual OFFBOARD/arming required.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.state, self.state_at = msg, self.now()

    def on_pose(self, msg):
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.pose, self.pose_at = p, self.now()

    def enter(self, phase, now):
        self.phase, self.started = phase, now
        self.get_logger().info(f'phase -> {phase}')

    def tick(self):
        now = self.now()
        self.phase_pub.publish(String(data=self.phase))
        if self.phase == 'ABORT':
            return
        if self.last_tick is not None and (now < self.last_tick or
                now - self.last_tick > self.cfg['telemetry_timeout']):
            if self.active_once:
                self.enter('ABORT', now)
                return
            self.enter('WAIT', now)
        self.last_tick = now
        fresh = (self.state_at is not None and self.pose_at is not None and
                 0 <= now - self.state_at <= self.cfg['telemetry_timeout'] and
                 0 <= now - self.pose_at <= self.cfg['telemetry_timeout'] and
                 self.state.connected)
        if not fresh:
            if self.phase != 'WAIT':
                self.enter('ABORT' if self.active_once else 'WAIT', now)
            return
        active = self.state.armed and self.state.mode == 'OFFBOARD'
        if self.active_once and not active:
            self.enter('ABORT', now)
            return
        if self.phase == 'WAIT':
            self.origin = (self.pose.x, self.pose.y, self.pose.z)
            self.enter('WARMUP', now)
        pos, vel, yaw = self.origin, (0.0, 0.0, 0.0), 0.0
        if self.phase == 'WARMUP':
            if now - self.started >= self.cfg['warmup_time'] and active:
                self.active_once = True
                self.enter('TAKEOFF', now)
        if self.phase == 'TAKEOFF':
            s = min((now - self.started) / self.cfg['takeoff_time'], 1.0)
            blend = 10*s**3 - 15*s**4 + 6*s**5
            speed = (30*s**2 - 60*s**3 + 30*s**4) / self.cfg['takeoff_time']
            height = self.cfg['z_hover'] - self.origin[2]
            pos = (self.origin[0], self.origin[1], self.origin[2] + height * blend)
            vel = (0.0, 0.0, height * speed)
            error = math.dist((self.pose.x, self.pose.y, self.pose.z), pos)
            if s == 1 and error < self.cfg['position_tolerance']:
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= self.cfg['settle_time']:
                    self.enter('TRACK', now)
            else:
                self.stable_since = None
        if self.phase in ('TRACK', 'HOLD'):
            elapsed = now - self.started
            t, scale = reference_clock(elapsed, self.cfg['duration'], self.cfg['stop_time'])
            ref = build_reference_figure8(
                t, r=self.cfg['fig8_r'], w=self.cfg['fig8_w'],
                z_hover=self.cfg['z_hover'], dz=self.cfg['dz'],
                hover_time=self.cfg['hover_time'], ramp_time=self.ramp,
                yaw_ramp=self.cfg['yaw_ramp'])
            pos = (ref[0] + self.origin[0], ref[1] + self.origin[1], ref[2])
            vel = tuple(ref[3:6] * scale)
            yaw = 2 * math.atan2(ref[9], ref[6])
            if elapsed >= self.cfg['duration'] + self.cfg['stop_time'] and self.phase != 'HOLD':
                self.phase = 'HOLD'  # Preserve phase-clock epoch so endpoint stays fixed.
                self.get_logger().info('phase -> HOLD (manual landing required)')
        self.publish(pos, vel, yaw)

    def publish(self, pos, vel, yaw):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # MAVROS converts these ROS ENU values to MAVLink LOCAL_NED internally.
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                         PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
        if not self.cfg['velocity_feedforward']:
            msg.type_mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        msg.position.x, msg.position.y, msg.position.z = map(float, pos)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, vel)
        msg.yaw = float(yaw)
        self.pub.publish(msg)
        ref = PoseStamped()
        ref.header = msg.header
        ref.pose.position = msg.position
        ref.pose.orientation.w = math.cos(yaw / 2)
        ref.pose.orientation.z = math.sin(yaw / 2)
        self.ref_pub.publish(ref)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PX4PIDBenchmark()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
