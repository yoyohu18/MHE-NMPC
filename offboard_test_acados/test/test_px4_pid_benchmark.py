"""Offline tests: no ROS node, DDS connection or simulator is started."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from mavros_msgs.msg import PositionTarget
from builtin_interfaces.msg import Time
from offboard_test_acados.px4_pid_benchmark import PX4PIDBenchmark, reference_clock


def test_stop_clock_derivative_and_endpoint():
    for t in (0.1, 59.9, 60.0, 61.0, 63.0, 64.0, 70.0):
        phase, speed = reference_clock(t, 60.0, 4.0)
        eps = 1e-5
        derivative = (reference_clock(t + eps, 60., 4.)[0] -
                      reference_clock(t - eps, 60., 4.)[0]) / (2 * eps)
        assert derivative == pytest.approx(speed, abs=1e-7)
    assert reference_clock(64., 60., 4.) == (62., 0.)
    assert reference_clock(100., 60., 4.) == (62., 0.)


@pytest.mark.parametrize('feedforward', [True, False])
def test_message_mask_enu_and_yaw(feedforward):
    node = SimpleNamespace(cfg={'velocity_feedforward': feedforward},
                           pub=Mock(), ref_pub=Mock(),
                           get_clock=lambda: SimpleNamespace(
                               now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    PX4PIDBenchmark.publish(node, (1, 2, 3), (4, 5, 6), math.pi / 2)
    msg = node.pub.publish.call_args.args[0]
    assert msg.coordinate_frame == PositionTarget.FRAME_LOCAL_NED
    assert (msg.position.x, msg.position.y, msg.position.z) == (1, 2, 3)
    assert msg.type_mask == (2496 if feedforward else 2552)
    assert msg.yaw == pytest.approx(math.pi / 2)
    ref = node.ref_pub.publish.call_args.args[0]
    assert ref.pose.orientation.z == pytest.approx(math.sqrt(0.5))


def fake_node():
    node = SimpleNamespace(
        cfg=dict(telemetry_timeout=1., warmup_time=2., takeoff_time=8.,
                 z_hover=3., position_tolerance=.2, settle_time=2.,
                 duration=60., stop_time=4., fig8_r=1., fig8_w=.3,
                 dz=.5, hover_time=2., yaw_ramp=False),
        phase='WAIT', state_at=0., pose_at=0., last_tick=None,
        state=SimpleNamespace(connected=True, armed=False, mode='POSCTL'),
        pose=SimpleNamespace(x=0., y=0., z=0.), active_once=False,
        stable_since=None, started=None, origin=None, ramp=4.,
        phase_pub=Mock(), publish=Mock(), get_logger=lambda: Mock())
    node.enter = lambda phase, now: PX4PIDBenchmark.enter(node, phase, now)
    return node


def step(node, t):
    node.now = lambda: t
    node.state_at = node.pose_at = t
    PX4PIDBenchmark.tick(node)


def test_manual_activation_tracking_hold_and_abort():
    node = fake_node()
    step(node, 0.)
    assert node.phase == 'WARMUP'
    # Frequent ticks avoid deliberately triggering the scheduler-gap watchdog.
    node.state.armed, node.state.mode = True, 'OFFBOARD'
    for i in range(1, 13):
        if i >= 10:
            node.pose.z = 3.
        step(node, float(i))
    assert node.phase == 'TRACK'
    for i in range(13, 78):
        step(node, float(i))
    assert node.phase == 'HOLD'
    end = node.publish.call_args
    step(node, 78.)
    assert node.publish.call_args == end
    node.state.mode = 'POSCTL'
    node.publish.reset_mock()
    step(node, 79.)
    assert node.phase == 'ABORT'
    node.state.mode = 'OFFBOARD'
    step(node, 80.)
    node.publish.assert_not_called()


def test_stale_pose_aborts_without_publishing():
    node = fake_node()
    node.phase, node.active_once = 'TRACK', True
    node.now = lambda: 2.
    PX4PIDBenchmark.tick(node)
    assert node.phase == 'ABORT'
    node.publish.assert_not_called()
