#!/usr/bin/env python3
"""drop 侧"一阶矩相对塌陷"释放判据的回归测试(2026-09-04,不起 ROS)。

背景:纯默认档实测(gviz_*_20260904_232444)出现 **DROP 确认死锁** ——
物理卸载后 |s| 从峰值 0.0229 塌到 0.0032(塌掉 86%),但旧判据要求跌破峰值的
10%(即塌掉 >90%),等于要求估计器残噪低于一个没有物理必要性的水平;而质量域
判据被 _mass_observable 的机动门控挡死(drop 后仍在飞 figure8)、残差路被
4.4N 阈值堵死 —— 三条路全断,_release_payload 从不被调用,发布目标 s 不切零,
no_payload_confidence 卡在 0.694 上不去,NMPC 的 DROP 永远 complete 不了。

修法(用户 2026-09-04 拍板):把 drop 侧塌陷比例放宽到 0.20 并加质量/惯量佐证,
**不动** moment_full(那是 confidence 标定,与状态转移解耦),也**不**要求回低机动。

这里锁住三件事:
  1. 载荷在机上时,即便机动让 m_p 探到负值(mass 佐证满分),也不得释放;
  2. 实测那组数字(ratio=0.14)在新判据下能释放、在旧的 0.10 下不能;
  3. 释放必须把发布目标 s 切零(_s_release_latched)——那正是 conf 能涨回去的前提。
"""

import numpy as np

from offboard_test_acados import mhe_node as mn
from offboard_test_acados.mhe_params import p as mhe_p
from offboard_test_acados.payload_estimate import (
    empty_evidence_scores,
    inertia_from_mass_moment,
    no_payload_confidence,
)


# 09-04 232444 轮的实测值
S_PEAK_LOADED = 0.0229      # 带载 |s| 峰值 [kg·m]
S_RESID_UNLOADED = 0.0032   # 物理卸载后的 |s| 残噪本底 [kg·m]


class _Logger:
    def __init__(self):
        self.msgs = []

    def info(self, m):
        self.msgs.append(m)

    warn = error = info


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Stub:
    """只装 _update_c_xy_est 需要的那部分状态。"""

    _update_c_xy_est = mn.MHENode._update_c_xy_est
    _release_payload = mn.MHENode._release_payload
    _update_payload_presence = mn.MHENode._update_payload_presence
    _mass_observable = mn.MHENode._mass_observable

    def __init__(self, ratio=0.20, persist=5):
        self.m_est = mhe_p.m_B + 0.15
        self.s_est = np.array([0.0, -S_PEAK_LOADED])
        # 质量域判据(主力):这里让它保持"已武装但不触发释放"
        self.c_xy_mass_release_mp = 0.03
        self.c_xy_mass_arm_ratio = 3.0
        self.c_xy_mass_arm_persist = 20
        self.c_xy_mass_release_persist = 20
        self._c_xy_mass_armed = True
        self._c_xy_mass_high = 0
        self._c_xy_mass_low = 0
        # drop 侧塌陷判据(被测对象)
        self.s_release_ratio = ratio
        self.s_release_persist = persist
        self.s_release_mass_score_min = 0.90
        self.s_release_inertia_score_min = 0.90
        self._s_peak = 0.0
        self._s_low = 0
        self._s_ratio_log_n = 0
        self._payload_conf_args = {
            'mass_full': 0.015, 'mass_zero': 0.060,
            'moment_full': 0.0015, 'moment_zero': 0.0060,
            'inertia_full': 0.0020, 'inertia_zero': 0.0100,
        }
        # 状态机 / 释放要动的东西
        self._payload_present = True
        self._load_armed = True
        self._s_release_latched = False
        self._payload_attached = False
        self.attach_offset = None
        self._s_decay_n = 0
        self._present_hi = 0
        self._present_lo = 0
        self.payload_present_enter_mp = 0.09
        self.payload_present_enter_persist = 20
        self.payload_present_exit_mp = 0.02
        self.payload_present_exit_persist = 50
        self.payload_exit_steady_omega = 0.15
        self.payload_exit_steady_vel = 0.20
        # c_xy 发布路径
        self.c_xy_from_moment = True
        self.c_xy_est = np.zeros(2)
        self._c_xy_inited = False
        self.c_xy_est_pub = _Pub()
        # figure8 高动态:|ω|、|v_xy| 都远超稳态门限 —— 质量域判据在这一档
        # 永远不累计,正是死锁现场
        self.x_meas = np.zeros(13)
        self.x_meas[3] = 4.0      # v_x
        self.x_meas[12] = 1.04    # yaw rate
        self.tau_phys = None
        self.thrust_phys = None
        self._log = _Logger()

    def get_logger(self):
        return self._log

    def _tick(self, s_xy, m_p, n=1):
        self.s_est = np.asarray(s_xy, dtype=float)
        self.m_est = mhe_p.m_B + m_p
        for _ in range(n):
            self._update_c_xy_est()


def _released(st):
    return st._s_release_latched


def test_loaded_maneuver_dip_does_not_release():
    """带载 figure8:m_p 探到负值(mass 佐证满分),但 |s| 没塌 ⇒ 不得释放。

    这是合取判据的安全性核心。单看质量通道会误报,正是 _mass_observable 门控
    存在的理由;这里靠"相对塌陷"把它挡住,所以才敢不加那道机动门控。
    """
    st = _Stub()
    st._tick([0.0, -S_PEAK_LOADED], 0.15, n=5)       # 先立峰值
    assert st._s_peak > 0.02
    # m_p 探负 30 帧(3s),|s| 仍在峰值附近
    st._tick([0.0, -0.0210], -0.10, n=30)
    assert not _released(st), '带载机动中 m_p 探负就释放 = 误触发'
    mass_sc, _, _ = empty_evidence_scores(0.0, [0.0, -0.0210],
                                          [0.0, 0.0, 0.0],
                                          **st._payload_conf_args)
    assert mass_sc == 1.0, '这一格的前提是质量通道确实满分(否则测不到合取)'


def test_measured_collapse_releases_within_persist():
    """实测那组数字:ratio=0.14 ⇒ 新判据在 persist 帧内释放。"""
    st = _Stub()
    st._tick([0.0, -S_PEAK_LOADED], 0.15, n=5)
    peak = st._s_peak
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=st.s_release_persist - 1)
    assert not _released(st), '不到 persist 帧就释放 = 少了持续确认'
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=1)
    assert _released(st), f'ratio={S_RESID_UNLOADED/peak:.3f} 应当释放'
    assert abs(S_RESID_UNLOADED / peak - 0.14) < 0.02, '实测比值应在 0.14 附近'


def test_old_ratio_would_deadlock():
    """同一组数据在旧的 0.10 下释放不了 —— 死锁的直接复现。"""
    st = _Stub(ratio=0.10)
    st._tick([0.0, -S_PEAK_LOADED], 0.15, n=5)
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=40)
    assert not _released(st), '旧阈值本就该卡住(这一格失败说明复现不到死锁)'


def test_release_unblocks_confidence():
    """释放 ⇒ 发布目标切零 ⇒ conf 才可能过 0.90。

    锁住的是那条因果:conf 是 release 之后的状态质量指标,不是 release 的前置
    条件。用残噪本底直接算 conf,应当够不到阈值;切零后才够得到。
    """
    m_out = mhe_p.m_B
    _, dJ, _ = inertia_from_mass_moment(
        m_out, [S_RESID_UNLOADED, 0.0], mhe_p.m_B,
        [mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz], mhe_p.rz_prior)
    conf_residual = no_payload_confidence(0.0, [S_RESID_UNLOADED, 0.0], dJ)
    assert conf_residual < 0.90, (
        f'残噪本底下 conf={conf_residual:.3f} 竟已达标,死锁前提不成立')
    assert 0.6 < conf_residual < 0.8, (
        f'应复现实测的 0.694 量级,实得 {conf_residual:.3f}')
    conf_latched = no_payload_confidence(0.0, [0.0, 0.0], [0.0, 0.0, 0.0])
    assert conf_latched >= 0.90, '切零后 conf 必须能达标'

    st = _Stub()
    st._tick([0.0, -S_PEAK_LOADED], 0.15, n=5)
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=st.s_release_persist)
    assert _released(st) and not st._payload_present


def _run():
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_') or not callable(fn):
            continue
        try:
            fn()
            print(f'  PASS  {name}')
        except AssertionError as e:
            fails += 1
            print(f'  FAIL  {name}: {e}')
    print(f'\n{fails} failed')
    return fails


if __name__ == '__main__':
    raise SystemExit(1 if _run() else 0)
