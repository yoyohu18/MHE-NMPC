#!/usr/bin/env python3
"""实验计划 §4 清除逻辑 baseline 开关(release_baseline)的决策点测试(不起 ROS)。

只测开关改动的分支本身:放行/抑制、释放指令边沿、真值分离、NMPC accept 门控。
检测器的证据与持续逻辑由 release_decision 自己的测试负责,这里用 monkeypatch 固定。
"""

import numpy as np
import pytest

from offboard_test_acados import mhe_node as mn
from offboard_test_acados import acados_nmpc_node as mn_nmpc
from offboard_test_acados.mhe_params import p as mhe_p
from offboard_test_acados.payload_estimate import RELEASE_BASELINES

from test_drop_confirm_standalone import _Clock, _Logger, _NMPCStub


class _Msg:
    def __init__(self, data=None):
        self.data = data


class _MHEStub:
    _update_c_xy_est = mn.MHENode._update_c_xy_est
    _eventless_release_suppressed = mn.MHENode._eventless_release_suppressed
    _suppressed_release_tag = mn.MHENode._suppressed_release_tag
    _release_cmd_enable_cb = mn.MHENode._release_cmd_enable_cb
    _release_oracle_sep_cb = mn.MHENode._release_oracle_sep_cb

    def __init__(self, baseline='P', path='mass'):
        self.now_sec = 100.0
        self._log = _Logger()
        self.release_baseline = baseline
        self.release_arm_on_command = False
        self._release_cmd_seen = False
        self._release_cmd_armed = False
        self._cmd_enable_prev = None
        self._cmd_hold = 0
        self._last_solve_ok = True
        self._s_reanchored = False
        self.releases = []
        self.m_est = mhe_p.m_B              # m_p = 0:质量域"空"
        self._load_armed = True
        # 质量域判据
        self.c_xy_mass_release_mp = 0.03 if path == 'mass' else 0.0
        self.c_xy_mass_release_persist = 3
        self.c_xy_mass_arm_ratio = 3.0
        self.c_xy_mass_arm_persist = 3
        self._c_xy_mass_armed = True
        self._c_xy_mass_high = 0
        self._c_xy_mass_low = 0
        # 统一 moment 判据
        self.s_release_ratio = 0.30 if path == 'moment' else 0.0
        self.s_release_abs_max = 0.008
        self.s_release_persist = 3
        self.s_release_fast_persist = 2
        self.s_release_strong_persist = 3
        self._s_low = self._s_fast = self._s_strong = 0
        self._s_peak = 0.0
        self._s_ref_loaded = 0.02
        self._moment_ref_ready = True
        self.moment_abs_max = 0.039
        self._dyn_overrange = 0
        self._residual_drop_evidence_until = -1.0
        self._residual_strong_evidence_until = -1.0
        self._s_ratio_log_n = 0
        self._release_detector_eval_frame = None
        self.frames = 0
        self.s_est = np.array([0.0, 0.001])
        # 之后的 c_xy 分支全部短路
        self.c_xy_from_moment = False
        self.tau_phys = None

    def get_clock(self):
        return _Clock(self)

    def get_logger(self):
        return self._log

    def _mass_observable(self):
        return True

    def _payload_frame_health(self):
        return True, 0.0

    def _update_moment_reference(self, s, ok):
        pass

    def _update_payload_presence(self, event=None):
        pass

    def _release_payload(self, src):
        self.releases.append(src)
        self._load_armed = False


@pytest.fixture
def forced_moment_fire(monkeypatch):
    """统一 moment 判据每帧都判"放行"。"""
    monkeypatch.setattr(mn, 'release_evidence', lambda *a, **k: {
        'ratio': 0.0, 'moment_collapsed': True, 'quantity_empty': True,
        'residual_drop': False, 'residual_strong': False})
    monkeypatch.setattr(mn, 'release_decision',
                        lambda *a, **k: (True, 'forced', 0, 0, 0))


def _tick(st, n):
    for _ in range(n):
        st._update_c_xy_est()


def test_baseline_table_matches_script():
    assert set(RELEASE_BASELINES) == {'P', 'A_PRIME', 'C_SAME', 'P_NOMOMENT', 'ORACLE'}


# ------------------------------------------------------------ 抑制真值表
@pytest.mark.parametrize('baseline,cmd_seen,moment_path,expected', [
    ('P', False, True, False), ('P', False, False, False),
    ('A_PRIME', False, True, True), ('A_PRIME', True, False, True),
    ('ORACLE', False, True, True), ('ORACLE', True, False, True),
    ('C_SAME', False, True, True), ('C_SAME', False, False, True),
    ('C_SAME', True, True, False), ('C_SAME', True, False, False),
    ('P_NOMOMENT', False, True, True), ('P_NOMOMENT', False, False, False),
])
def test_suppression_table(baseline, cmd_seen, moment_path, expected):
    st = _MHEStub(baseline)
    st._release_cmd_seen = cmd_seen
    assert st._eventless_release_suppressed(moment_path) is expected


def test_c_thrust_still_suppresses_both_paths():
    st = _MHEStub('P')
    st.release_arm_on_command = True
    assert st._eventless_release_suppressed(True)
    assert st._eventless_release_suppressed(False)


# ------------------------------------------------------------ 质量域路径
@pytest.mark.parametrize('baseline,released', [
    ('P', True), ('P_NOMOMENT', True),
    ('A_PRIME', False), ('ORACLE', False), ('C_SAME', False)])
def test_mass_path_release_by_baseline(baseline, released):
    st = _MHEStub(baseline, path='mass')
    _tick(st, 10)
    assert bool(st.releases) is released


# ------------------------------------------------------------ moment 路径
@pytest.mark.parametrize('baseline,released', [
    ('P', True), ('P_NOMOMENT', False),
    ('A_PRIME', False), ('ORACLE', False), ('C_SAME', False)])
def test_moment_path_release_by_baseline(forced_moment_fire, baseline, released):
    st = _MHEStub(baseline, path='moment')
    _tick(st, 3)
    assert bool(st.releases) is released
    assert st._release_detector_eval_frame is not None   # 照算:心跳不变


# ------------------------------------------------------------ 指令边沿
def test_a_prime_releases_on_command_edge_only_once():
    st = _MHEStub('A_PRIME')
    st._release_cmd_enable_cb(_Msg(False))      # 无 True 前沿:不是边沿
    st._release_cmd_enable_cb(_Msg(True))
    assert st.releases == []
    st._release_cmd_enable_cb(_Msg(False))
    assert st.releases == ['command immediate']
    st._release_cmd_enable_cb(_Msg(True))
    st._release_cmd_enable_cb(_Msg(False))
    assert st.releases == ['command immediate']


def test_c_same_opens_detector_and_restarts_persistence():
    st = _MHEStub('C_SAME', path='mass')
    _tick(st, 10)                                 # 指令前:照算不放行,计数照攒
    assert st.releases == [] and st._c_xy_mass_low >= st.c_xy_mass_release_persist
    st._release_cmd_enable_cb(_Msg(True))
    st._release_cmd_enable_cb(_Msg(False))
    assert st._release_cmd_seen and st.releases == []
    assert st._c_xy_mass_low == 0                 # 持续帧从指令起重新计
    _tick(st, st.c_xy_mass_release_persist - 1)
    assert st.releases == []
    _tick(st, 1)
    assert len(st.releases) == 1


@pytest.mark.parametrize('baseline', ['P', 'P_NOMOMENT', 'ORACLE'])
def test_command_edge_is_inert_for_other_baselines(baseline):
    st = _MHEStub(baseline)
    st._release_cmd_enable_cb(_Msg(True))
    st._release_cmd_enable_cb(_Msg(False))
    assert st.releases == [] and not st._release_cmd_seen


def test_oracle_releases_on_true_separation():
    st = _MHEStub('ORACLE')
    st._release_oracle_sep_cb(_Msg())
    assert st.releases == ['oracle true separation']


# ------------------------------------------------------------ NMPC accept
def _nmpc(baseline):
    node = _NMPCStub()
    node.release_baseline = baseline
    node.attach_time = 0.0
    return node


def _drop_phase(node):
    node._grip_drop_phase = mn_nmpc.AcadosNMPCNode._grip_drop_phase.__get__(node)
    node.continuous_payload_estimates = True
    node.grip_drop_after_sec = 1.0
    node.grip_lift_after_sec = node.grip_lift_dur = 0.0
    node.grip_drop_at_fig8_tip = False
    node.grip_dynamic_active = False
    node.drop_time = None
    node.enable_pub = type('P', (), {'publish': lambda self, m: None})()
    node._grip_drop_phase(5.0)


def test_nmpc_a_prime_accepts_at_command():
    node = _nmpc('A_PRIME')
    _drop_phase(node)
    assert node.grip_dropped and not node.grip_drop_pending
    assert any('src=command' in m for m in node._log.msgs)
    assert node._no_payload_latched


@pytest.mark.parametrize('baseline', ['P', 'C_SAME', 'P_NOMOMENT', 'ORACLE'])
def test_nmpc_other_baselines_wait_after_command(baseline):
    node = _nmpc(baseline)
    _drop_phase(node)
    assert node.grip_drop_pending and not node.grip_dropped


@pytest.mark.parametrize('baseline,pending,allowed', [
    ('P', False, True), ('P', True, True),
    ('P_NOMOMENT', False, True),
    ('C_SAME', False, False), ('C_SAME', True, True),
    ('A_PRIME', True, False), ('ORACLE', True, False)])
def test_nmpc_confidence_accept_gate(baseline, pending, allowed):
    node = _nmpc(baseline)
    node.grip_drop_pending = pending
    if pending:
        node._drop_cmd_wall = node.now_sec
    for _ in range(node.no_payload_conf_hold_frames + 2):   # conf=1.0 持续
        node._confirm_no_payload_if_persistent()
    assert node._no_payload_latched is allowed


def test_nmpc_oracle_accepts_on_true_separation():
    node = _nmpc('ORACLE')
    node.grip_drop_pending = True
    mn_nmpc.AcadosNMPCNode._release_oracle_sep_cb(node, _Msg())
    assert node.grip_dropped and not node.grip_drop_pending
    assert any('src=oracle' in m for m in node._log.msgs)


def test_nmpc_confidence_accept_log_unchanged():
    """P 的 DROP complete 文本逐字不变(旧日志解析器依赖它)。"""
    node = _nmpc('P')
    node.grip_drop_pending = True
    node._drop_cmd_wall = node.now_sec
    for _ in range(node.no_payload_conf_hold_frames):
        node._confirm_no_payload_if_persistent()
    assert any(m.startswith('DROP complete: MHE no-payload confidence 1.000 '
                            'persisted for ') for m in node._log.msgs)
