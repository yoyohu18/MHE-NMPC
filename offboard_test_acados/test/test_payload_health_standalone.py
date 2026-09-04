#!/usr/bin/env python3
"""Health/freshness checks for MHE payload frames without starting ROS."""

from offboard_test_acados import mhe_node as mn


class _Clock:
    def __init__(self, owner):
        self.owner = owner

    def now(self):
        class _Now:
            pass
        value = _Now()
        value.nanoseconds = int(self.owner.now_sec * 1e9)
        return value


class _Stub:
    _payload_inputs_fresh = mn.MHENode._payload_inputs_fresh
    _payload_frame_health = mn.MHENode._payload_frame_health

    def __init__(self):
        self.now_sec = 10.0
        self.payload_input_fresh_sec = 0.30
        self.payload_solution_fresh_sec = 0.30
        self._last_odom_rx_sec = 9.9
        self._last_motor_rx_sec = 9.9
        self._last_u_rx_sec = 9.9
        self._last_solve_success_sec = 9.9
        self._last_solve_ok = True

    def get_clock(self):
        return _Clock(self)


def test_health_requires_last_solve_and_all_fresh_inputs():
    state = _Stub()
    healthy, age = state._payload_frame_health()
    assert healthy and abs(age - 0.1) < 1e-9

    state._last_solve_ok = False
    assert not state._payload_frame_health()[0]
    state._last_solve_ok = True
    state._last_motor_rx_sec = 9.0
    assert not state._payload_frame_health()[0]


def test_old_solution_is_unhealthy_even_with_fresh_transport_inputs():
    state = _Stub()
    state._last_solve_success_sec = 9.0
    healthy, age = state._payload_frame_health()
    assert not healthy and abs(age - 1.0) < 1e-9


if __name__ == '__main__':
    test_health_requires_last_solve_and_all_fresh_inputs()
    test_old_solution_is_unhealthy_even_with_fresh_transport_inputs()
    print('all passed')
