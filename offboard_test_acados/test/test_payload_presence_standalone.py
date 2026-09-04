#!/usr/bin/env python3
"""MHE 自主载荷存在状态机的回归测试(2026-09-04,不起 ROS)。

背景:三条自主释放路径原先都以 `_payload_attached`(收到过**外部** attach 通知)
为门控,而连续主线的前提正是不发这个通知 —— 于是主线档下 `_release_payload`
从未被调用过。状态机 `_payload_present` 由估计器自己判定,替下那个门控。

这里测的就是"没有任何外部事件"时,attach→drop 全程能不能自己走通。
"""

import numpy as np

from offboard_test_acados import mhe_node as mn
from offboard_test_acados.mhe_params import p as mhe_p


class _Logger:
    def __init__(self):
        self.msgs = []

    def info(self, m):
        self.msgs.append(m)

    warn = error = info


class _Stub:
    _update_payload_presence = mn.MHENode._update_payload_presence
    _release_payload = mn.MHENode._release_payload
    _mass_observable = mn.MHENode._mass_observable

    def __init__(self):
        self.m_est = mhe_p.m_B
        self.payload_present_enter_mp = 0.09
        self.payload_present_enter_persist = 20
        self.payload_present_exit_mp = 0.02
        self.payload_present_exit_persist = 50
        self.payload_exit_steady_omega = 0.15
        self.payload_exit_steady_vel = 0.20
        # 默认静止(低机动窗口内);_maneuver() 切到 figure8 那种高动态
        self.x_meas = np.zeros(13)
        self._payload_present = False
        self._load_armed = False
        self._s_decay_n = 0
        self._s_out_prev = np.zeros(2)
        self._present_hi = 0
        self._present_lo = 0
        self._s_release_latched = False
        self._s_peak = 0.0
        self._s_low = 0
        # _release_payload 要动的东西
        self._payload_attached = False
        self.attach_offset = None
        self.c_xy_est = np.zeros(2)
        self._c_xy_inited = False
        self._log = _Logger()

    def get_logger(self):
        return self._log

    @property
    def c_xy_est_pub(self):
        class _P:
            def publish(self, msg):
                pass
        return _P()

    def _tick(self, m_p, n=1):
        self.m_est = mhe_p.m_B + m_p
        for _ in range(n):
            self._update_payload_presence()

    def _maneuver(self, on=True):
        """切换到 figure8 那种高动态(|v_xy| 远超门槛)。"""
        self.x_meas[3] = 4.0 if on else 0.0
        self.x_meas[10] = 0.5 if on else 0.0


def test_enters_loaded_only_after_persistent_evidence():
    """单帧冲高不算数 —— attach 收敛期的振荡会同时满足冲高和回落。"""
    st = _Stub()
    st._tick(0.15, n=st.payload_present_enter_persist - 1)
    assert not st._payload_present
    st._tick(0.00, n=1)                     # 回落一帧,计数清零
    st._tick(0.15, n=st.payload_present_enter_persist - 1)
    assert not st._payload_present, '持续计数没有被回落打断'
    st._tick(0.15, n=1)
    assert st._payload_present


def test_hysteresis_holds_loaded_through_convergence_dip():
    """进 0.09 / 出 0.03 的迟滞:LOADED 期间 m_p 掉到 0.05 不该翻回 EMPTY。"""
    st = _Stub()
    st._tick(0.15, n=25)
    assert st._payload_present
    st._tick(0.05, n=120)                   # 低于入阈但高于出阈
    assert st._payload_present, '迟滞没起作用,LOADED 被中途踢回 EMPTY'


def test_exits_empty_after_persistent_unloaded_evidence():
    st = _Stub()
    st._tick(0.15, n=25)
    st._tick(0.00, n=st.payload_present_exit_persist - 1)
    assert st._payload_present
    st._tick(0.00, n=1)
    assert not st._payload_present


def test_residual_events_are_fast_paths():
    """残差检测器看见的承重比质量收敛早得多,快速通道不等持续帧。"""
    st = _Stub()
    st._update_payload_presence('attach')
    assert st._payload_present
    st._update_payload_presence('drop')
    assert not st._payload_present


def test_autonomous_attach_then_drop_without_any_external_event():
    """全程无外部事件:自主进 LOADED → 自主释放 → s 闩置位。

    这是主线档的完整通路,也是第二轮批次卡住的那一条。
    """
    st = _Stub()
    assert not st._payload_attached          # 从头到尾没有外部通知

    st._tick(0.15, n=25)                     # 载荷上机,估计器自己看出来
    assert st._payload_present
    assert not st._s_release_latched         # 带载期间 s 正常发布

    # 载荷离机。慢兜底要 exit_persist 帧(5s)且须在低机动窗口内。
    st._tick(0.00, n=st.payload_present_exit_persist + 1)
    assert not st._payload_present
    st._release_payload('autonomous')        # 门控放行后释放
    assert st._s_release_latched, 's 的发布目标没有切零'
    assert not st._payload_attached, '自主通路不该动外部事件状态'


def test_reentering_loaded_clears_s_latch():
    """重新带载必须解除 s 闩,否则下一次任务 s 恒为零。"""
    st = _Stub()
    st._tick(0.15, n=25)
    st._tick(0.00, n=25)
    st._release_payload('autonomous')
    assert st._s_release_latched
    st._tick(0.15, n=25)                     # 再次 attach
    assert st._payload_present
    assert not st._s_release_latched


def test_maneuver_mass_dip_does_not_unload():
    """带载机动中 m_p 探负持续 3s,不得 LOADED->EMPTY、不得释放。

    这是 09-04 首次 smoke 的实测故障:4m/s figure8 段 m_p 震荡到 -0.10kg,
    2s 窗口下误退了 3 次。
    """
    st = _Stub()
    st._tick(0.15, n=25)
    assert st._payload_present and st._load_armed
    st._maneuver(True)
    st._tick(-0.10, n=30)                    # 10Hz x 30 = 3s 的负 m_p
    assert st._payload_present, '机动质量探底把 LOADED 踢掉了'
    assert st._load_armed, 'armed latch 不该被机动探底清掉'
    assert not st._s_release_latched, '不该误释放 s'


def test_reliable_drop_releases_even_while_transiently_empty():
    """_payload_present 暂时为 False 时收到可靠 DROP,仍必须能释放。

    门控用的是 armed latch 而不是瞬时状态,正是为了这个场景。
    """
    st = _Stub()
    st._tick(0.15, n=25)
    assert st._load_armed
    st._payload_present = False              # 临时 EMPTY(机动误判残留)
    assert st._load_armed, '临时 EMPTY 不该清 armed'
    st._release_payload('no-signal residual')   # 门控放行 -> 真释放
    assert st._s_release_latched
    assert not st._load_armed, '释放完成后才清 armed'


def test_slow_fallback_releases_without_residual_signal():
    """无残差信号、低机动、空载持续 5s -> 慢兜底自己完成退出。"""
    st = _Stub()
    st._tick(0.15, n=25)
    st._maneuver(False)                      # 低机动窗口
    st._tick(0.00, n=st.payload_present_exit_persist - 1)
    assert st._payload_present
    st._tick(0.00, n=1)
    assert not st._payload_present, '慢兜底没有触发'


def test_slow_fallback_frozen_during_maneuver():
    """同样的空载读数,在高动态里不该累计出退出。"""
    st = _Stub()
    st._tick(0.15, n=25)
    st._maneuver(True)
    st._tick(0.00, n=st.payload_present_exit_persist * 3)
    assert st._payload_present, '高动态窗口没有清除退出计数'


def test_presence_is_independent_of_external_attach_flag():
    """两个状态不混用:外部通知不是进入自主状态的必要条件,也不是充分条件。"""
    st = _Stub()
    st._payload_attached = True              # 外部说有载荷
    st._tick(0.00, n=40)                     # 但估计量说没有
    assert not st._payload_present, '自主状态被外部事件状态污染了'


if __name__ == '__main__':
    import sys
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'  PASS  {name}')
            except AssertionError as e:
                fails += 1
                print(f'  FAIL  {name}: {e}')
    print(f'\n{fails} failed')
    sys.exit(1 if fails else 0)
