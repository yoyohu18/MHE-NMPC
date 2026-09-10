#!/usr/bin/env python3
"""释放检测器启动自检与运行时心跳看门狗回归测试（不起 ROS）。"""

from offboard_test_acados import mhe_node as mn


class _Logger:
    def __init__(self):
        self.infos = []
        self.warns = []

    def info(self, msg):
        self.infos.append(msg)

    def warn(self, msg):
        self.warns.append(msg)


class _Stub:
    _release_detector_config = mn.MHENode._release_detector_config
    _report_release_detector_startup = (
        mn.MHENode._report_release_detector_startup)
    _check_release_detector_watchdog = (
        mn.MHENode._check_release_detector_watchdog)

    def __init__(self, enabled=True):
        self.c_xy_est_enable = enabled
        self.s_release_ratio = 0.30
        self.c_xy_mass_release_mp = 0.03
        self.geom_release_mode = 'self'
        self.release_detector_watchdog_sec = 3.0
        self.release_detector_warn_repeat_sec = 10.0
        self._load_armed = False
        self.frames = 0
        self._release_detector_eval_frame = None
        self._release_detector_armed_frame = None
        self._release_detector_last_warn_frame = None
        self._log = _Logger()

    def get_logger(self):
        return self._log


def test_startup_reports_ready_configuration():
    st = _Stub(enabled=True)
    st._report_release_detector_startup()
    assert any('READY' in msg for msg in st._log.infos)
    assert not st._log.warns


def test_startup_warns_when_master_gate_is_off():
    st = _Stub(enabled=False)
    st._report_release_detector_startup()
    assert any('UNIFIED DETECTOR OFF' in msg for msg in st._log.warns)
    assert any('c_xy_est_enable=false' in msg for msg in st._log.warns)


def test_armed_silence_warns_and_is_rate_limited():
    st = _Stub(enabled=False)
    st._load_armed = True
    st._check_release_detector_watchdog()       # 建立武装时刻
    st.frames = 29
    st._check_release_detector_watchdog()
    assert not st._log.warns
    st.frames = 30
    st._check_release_detector_watchdog()
    assert len(st._log.warns) == 1
    assert 'ARMED BUT NO UNIFIED DETECTOR EVALUATION' in st._log.warns[0]

    st.frames = 129                            # 距上次警告 9.9 s
    st._check_release_detector_watchdog()
    assert len(st._log.warns) == 1
    st.frames = 130
    st._check_release_detector_watchdog()
    assert len(st._log.warns) == 2


def test_detector_heartbeat_suppresses_warning_and_disarm_resets():
    st = _Stub(enabled=True)
    st._load_armed = True
    st._check_release_detector_watchdog()
    st.frames = 30
    st._release_detector_eval_frame = st.frames
    st._check_release_detector_watchdog()
    assert not st._log.warns

    st._load_armed = False
    st._check_release_detector_watchdog()
    assert st._release_detector_armed_frame is None
    assert st._release_detector_last_warn_frame is None
