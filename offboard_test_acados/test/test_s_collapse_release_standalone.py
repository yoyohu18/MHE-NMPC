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
    _update_moment_reference = mn.MHENode._update_moment_reference
    _release_payload = mn.MHENode._release_payload
    _update_payload_presence = mn.MHENode._update_payload_presence
    _mass_observable = mn.MHENode._mass_observable
    _eventless_release_suppressed = mn.MHENode._eventless_release_suppressed
    _suppressed_release_tag = mn.MHENode._suppressed_release_tag

    def __init__(self, ratio=0.30, persist=5):
        self.m_est = mhe_p.m_B + 0.15
        self.s_est = np.array([0.0, -S_PEAK_LOADED])
        # 质量域判据(主力):这里让它保持"已武装但不触发释放"
        self.c_xy_mass_release_mp = 0.03
        self.c_xy_mass_arm_ratio = 3.0
        self.c_xy_mass_arm_persist = 20
        self.c_xy_mass_release_persist = 20
        # 旧质量域释放分支保持**未武装**:本文件测的是统一 detector,
        # 而 stub 默认低动态(建立 moment 基准的前提)会让旧分支的释放计数开始累计,
        # 20 帧后它会自己释放,把 detector 的行为盖掉(2026-09-05 踩到)。
        # 未武装时旧分支只走武装逻辑、不释放;统一 detector 自 ddfe9d2 起也不再
        # 依赖这个位,所以这正是"目标轮"的真实形态。
        self._c_xy_mass_armed = False
        self._c_xy_mass_high = 0
        self._c_xy_mass_low = 0
        # drop 侧塌陷判据(被测对象)
        self.s_release_ratio = ratio
        self.s_release_persist = persist
        self.s_release_mass_score_min = 0.90
        self.s_release_inertia_score_min = 0.90
        self.s_release_abs_max = 0.008
        self.s_release_fast_persist = 2
        self.s_release_strong_persist = 3
        self._s_fast = 0
        self._s_strong = 0
        self._residual_drop_evidence_until = -1.0
        self._residual_strong_evidence_until = -1.0
        # moment reference(2026-09-05)
        self.moment_abs_max = 0.039
        self.moment_ref_window = 20
        self.moment_ref_omega_max = 0.15
        self.moment_ref_vel_max = 0.20
        self.moment_ref_max_cv = 0.25
        self.moment_ref_dir_min = 0.90
        self._moment_ref_ready = False
        self._s_ref_loaded = 0.0
        self._moment_ref_buf = []
        self._moment_ref_reject = 0
        self._dyn_overrange = 0
        # health 门控(2026-09-05)
        self._last_solve_ok = True
        self._last_solve_success_sec = 0.0
        self._s_reanchored = False
        self._frame_healthy = True
        self._t = 0.0
        self._s_peak = 0.0
        self._s_low = 0
        self._s_ratio_log_n = 0
        self.frames = 0
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
        self.c_xy_clamp_enable = True
        self._c_xy_reject = 0
        self._c_xy_total = 0
        self.c_xy_est = np.zeros(2)
        self._c_xy_inited = False
        self.c_xy_est_pub = _Pub()
        # 默认**低动态**:moment 基准只在低动态窗口建立(2026-09-05)。
        # 需要 figure-8 高动态语义的用例显式调 _low_dynamic(False)。
        self.x_meas = np.zeros(13)
        self.tau_phys = None
        self.thrust_phys = None
        self._log = _Logger()

    def get_logger(self):
        return self._log

    def _payload_frame_health(self):
        return bool(self._frame_healthy), 0.0

    def get_clock(self):
        stub = self

        class _C:
            @staticmethod
            def now():
                class _N:
                    nanoseconds = stub._t * 1e9
                return _N()
        return _C()

    def _seed_reference(self, s_y, n=None):
        """在低动态下用稳定的 |s| 建立并冻结 moment 基准(测试前置)。"""
        self._low_dynamic(True)
        self._tick([0.0, -abs(s_y)], 0.15, n=n or (self.moment_ref_window + 2))
        assert self._moment_ref_ready, '前置:基准应已建立'
        # 前置用的高 m_p 会把**旧质量域分支**一并武装(它的水位是 0.09kg),
        # 之后 m_p 一探负那条路就会自己释放,把统一 detector 的行为盖掉。
        # 本文件只测统一 detector,所以前置后复位旧分支的状态。
        self._c_xy_mass_armed = False
        self._c_xy_mass_high = 0
        self._c_xy_mass_low = 0

    def _low_dynamic(self, on=True):
        """切到低动态(建立 moment 基准的前提)/回到 figure-8 高动态。"""
        self.x_meas = np.zeros(13)
        if not on:
            self.x_meas[3] = 4.0      # v_x
            self.x_meas[12] = 1.04    # yaw rate

    def _arm_residual(self, hold=3.0, strong=False):
        self._residual_drop_evidence_until = self._t + hold
        if strong:
            self._residual_strong_evidence_until = self._t + hold

    def _tick(self, s_xy, m_p, n=1):
        self.s_est = np.asarray(s_xy, dtype=float)
        self.m_est = mhe_p.m_B + m_p
        for _ in range(n):
            self._t += 0.1
            self.frames += 1
            self._update_c_xy_est()


def _released(st):
    return st._s_release_latched


def test_loaded_maneuver_dip_does_not_release():
    """带载 figure8:m_p 探到负值(mass 佐证满分),但 |s| 没塌 ⇒ 不得释放。

    这是合取判据的安全性核心。单看质量通道会误报,正是 _mass_observable 门控
    存在的理由;这里靠"相对塌陷"把它挡住,所以才敢不加那道机动门控。
    """
    st = _Stub()
    st._seed_reference(S_PEAK_LOADED)       # 先立峰值
    assert st._moment_ref_ready and st._s_ref_loaded > 0.02
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
    st._seed_reference(S_PEAK_LOADED)
    peak = st._s_ref_loaded
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=st.s_release_persist - 1)
    assert not _released(st), '不到 persist 帧就释放 = 少了持续确认'
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=1)
    assert _released(st), f'ratio={S_RESID_UNLOADED/peak:.3f} 应当释放'
    assert abs(S_RESID_UNLOADED / peak - 0.14) < 0.02, '实测比值应在 0.14 附近'


def test_old_ratio_would_deadlock():
    """同一组数据在旧的 0.10 下释放不了 —— 死锁的直接复现。"""
    st = _Stub(ratio=0.10)
    st._seed_reference(S_PEAK_LOADED)
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
    st._seed_reference(S_PEAK_LOADED)
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=st.s_release_persist)
    assert _released(st) and not st._payload_present


def test_single_channel_never_releases():
    """禁止单通道:光有 ratio 塌陷(载荷还在)不释放;光有质量为空也不释放。

    mass 与 inertia 在连续接口里不是两个独立证据(dJ 由 m_est 与固定 r_z 派生),
    所以它们合并成一条通道,更不能靠"两票"过关。
    """
    st = _Stub()
    st._seed_reference(S_PEAK_LOADED)
    # ① |s| 塌到 0,但载荷量证据说还挂着 -> 不释放
    st._tick([0.0002, 0.0], 0.15, n=40)
    assert not _released(st), 'ratio 单通道不得释放'
    # ② 质量为空,但 |s| 还在峰值附近 -> 不释放
    st2 = _Stub()
    st2._seed_reference(S_PEAK_LOADED)
    st2._tick([0.0, -S_PEAK_LOADED], -0.005, n=40)
    assert not _released(st2), '质量(含惯量)单通道不得释放'


def test_no_release_without_moment_evidence():
    """★ 任何成功释放都必须带 moment 证据(2026-09-05 用户拍板)。

    原 fastA(普通残差 + 质量为空,无 moment)已删除:9 轮 SITL 里 3 轮提前释放,
    最早提前 51.1s。根因是"载荷还在时 quantity_empty 恒假"这个假设不成立 ——
    4m/s figure-8 中 m_est 长时间贴在下界 m_min,m_p 持续为负(带载段
    m_p<0.03 的连续时长中位 14s、最坏 48s),幅度与持续性都分不开。
    于是那条路径是两个都不可靠的通道相与,没有任何东西能否决它。

    代价是"ratio 不塌的 drop"检不出 —— 那是**有意接受的漏检**,交给控制器的
    unresolved 路径,而不是靠更弱的证据去释放。
    """
    worst_peak, worst_resid = 0.0140, 0.0049      # ratio=0.35,moment 不成立
    st = _Stub()
    st._seed_reference(worst_peak)
    st._arm_residual()                            # 普通票在场
    st._tick([worst_resid, 0.0], -0.005, n=30)    # 质量也报空
    assert not _released(st), '缺 moment 证据时不得释放(原 fastA 已删除)'
    st2 = _Stub()
    st2._seed_reference(worst_peak)
    st2._arm_residual(strong=True)                # 强票 + moment 才行
    st2._tick([0.0003, 0.0], -0.005, n=st2.s_release_strong_persist)
    assert _released(st2), '强票 + moment 塌陷应当释放'


def test_maneuver_mass_dip_with_residual_does_not_release():
    """复现 20260905_152924 的误释放现场:m_p 探负 + 普通残差票,但 |s| 没塌。

    实测那一帧:ratio=0.694 moment=0 quantity=1 resid=1 m_p=-0.0624。
    删掉 fastA 之后这组证据必须**不**释放。
    """
    st = _Stub()
    st._seed_reference(0.03283)          # peak=0.03283(实测)
    st._arm_residual()
    # |s|=0.02278 -> ratio=0.694,远高于阈值;m_p=-0.0624 让质量通道报空
    st._tick([0.0, -0.02278], -0.0624, n=40)
    assert not _released(st), '这正是 152924 轮 drop 前 51s 的误释放现场'


def test_plain_residual_must_not_pair_with_collapse():
    """普通残差票 + moment 塌陷**不得**释放,只有强票才行。

    理由是幅度不可分:0.2kg 高频回放里 figure-8 稳定段 ΔT 最负到 -1.69N,
    比 0.15kg 卸载的真信号 -1.47N 还大。普通票必然会在机动中偶发,若允许它与
    "偶发 ratio 塌陷"组合,就是一次等着发生的误释放。
    """
    st = _Stub()
    st._seed_reference(S_PEAK_LOADED)
    st._arm_residual(strong=False)
    # |s| 塌了,但载荷量说还挂着(机动中 m_p 没探空) -> 只有普通票,不该释放
    st._tick([0.0005, 0.0], 0.15, n=20)
    assert not _released(st), '普通残差票不得与 moment 塌陷直接释放'
    # 换成强票 -> 允许
    st2 = _Stub()
    st2._seed_reference(S_PEAK_LOADED)
    st2._arm_residual(strong=True)
    st2._tick([0.0005, 0.0], 0.15, n=st2.s_release_strong_persist)
    assert _released(st2), '强票 + moment 塌陷应当释放'


def test_unhealthy_frame_blocks_release():
    """★ 20260905_152924 的根因回归:re-anchor 的 s_est=0 不得当成 moment 证据。

    根因链:连续 5 次 solve failure -> re-anchor 把 s_est 清成 0 -> 释放判据
    (跑在 _solve_window 之前)照常运行 -> ratio=0.000 被读成 moment_collapsed
    -> 恰好 m_p 也探负 -> slow 误释放。
    这一格锁的是:health=false 时,即使 ratio=0 且 quantity empty,也必须零释放。
    """
    st = _Stub()
    st._seed_reference(0.03283)          # 立带载峰值
    peak = st._s_ref_loaded
    # re-anchor:s 被清零、标志置位,解还没成功
    st._s_reanchored = True
    st._frame_healthy = False
    st._tick([0.0, 0.0], -0.0624, n=60)           # ratio=0 且 m_p 探负
    assert not _released(st), 'health=false 时不得释放(这正是 152924 的现场)'
    assert st._s_low == 0 and st._s_strong == 0, '不健康时持续计数必须清零'
    assert abs(st._s_ref_loaded - peak) < 1e-9, '基准不该被短暂求解失败清掉'


def test_release_resumes_after_health_returns():
    """健康恢复后判决要能正常继续 —— 门控不是永久封锁。"""
    st = _Stub()
    st._seed_reference(0.0229)
    st._s_reanchored = True; st._frame_healthy = False
    st._tick([0.0, 0.0], -0.05, n=20)
    assert not _released(st)
    # 下一次成功解:标志清掉、健康恢复,且 s 是真实的塌陷值
    st._s_reanchored = False; st._frame_healthy = True
    st._tick([0.0032, 0.0], -0.005, n=st.s_release_persist)
    assert _released(st), '健康恢复且证据充分时应当释放'


def test_counters_do_not_span_unhealthy_gap():
    """不健康帧要**打断**持续,不能跨盲区把两段拼成一次达标。"""
    st = _Stub()
    st._seed_reference(0.0229)
    st._tick([0.0032, 0.0], -0.005, n=st.s_release_persist - 1)   # 差一帧
    assert not _released(st)
    st._frame_healthy = False
    st._tick([0.0032, 0.0], -0.005, n=1)                          # 盲区一帧
    assert st._s_low == 0
    st._frame_healthy = True
    st._tick([0.0032, 0.0], -0.005, n=st.s_release_persist - 1)   # 又差一帧
    assert not _released(st), '跨盲区拼出来的持续不算数'
    st._tick([0.0032, 0.0], -0.005, n=1)
    assert _released(st)


def test_detector_runs_without_mass_domain_arming():
    """★ 统一 detector 不得再被旧的质量域武装位 _c_xy_mass_armed 挡住。

    20260905_162459:自主状态机已 EMPTY->LOADED(_load_armed=True,靠残差 attach
    事件),但那轮 m_est 全程偏低,旧路径的 _c_xy_mass_armed 从未置位 =>
    [s-collapse] 一行都没有,统一 detector 整段没运行。
    两个武装状态的证据来源不同,不该串在一起当同一个总开关。
    """
    st = _Stub()
    assert not st._c_xy_mass_armed       # 旧质量域路径从未武装(stub 默认)
    assert st._load_armed                # 但自主状态机认为载荷在机上
    st._seed_reference(S_PEAK_LOADED)
    assert st._moment_ref_ready and st._s_ref_loaded > 0.02, '未武装时 detector 仍应在跑(峰值要立起来)'
    st._tick([S_RESID_UNLOADED, 0.0002], -0.005, n=st.s_release_persist)
    assert _released(st), '统一 detector 不该被 _c_xy_mass_armed 挡住'


def test_moment_ref_rejects_attach_spike():
    """★ reference 构造:注入越界尖峰,基准必须落在稳定段,且尖峰期间不得释放。

    2026-09-05 实测:figure-8 段 7.6%~18.3% 的带载帧 |s| 越过物理上限
    0.30kg x 0.13m = 0.039 kg·m(最大 0.052,反解 r_xy=0.35m —— 那不是几何,
    是动态失真)。旧实现拿 running max 当分母,这些样本一旦进去,带载稳态的 ratio
    就被压到阈值以下,moment_collapsed 会在载荷还挂着时成立。
    """
    st = _Stub()
    st._low_dynamic(True)
    # ① 先灌越界尖峰(含用户点名的 0.0769):必须被剔除,基准不得建立
    for v in (0.0769, 0.0743, 0.0470):
        st._tick([0.0, -v], 0.15, n=5)
        assert not st._moment_ref_ready, f'越界样本 {v} 不该进入基准'
    assert st._moment_ref_reject > 0, '越界样本应计入 estimator-quality 指标'
    assert not _released(st), '尖峰期间不得释放'
    # ② 稳定段:基准建立并冻结,取值应接近稳定值而非尖峰
    st._tick([0.0, -0.0180], 0.15, n=st.moment_ref_window + 2)
    assert st._moment_ref_ready, '低动态 + 物理可行的稳定窗口应能建立基准'
    assert abs(st._s_ref_loaded - 0.0180) < 0.002, (
        f'基准 {st._s_ref_loaded:.4f} 应接近稳定段 0.0180,而不是尖峰量级')
    # ③ 冻结:回到高动态再出现大值,基准不得被抬高
    ref = st._s_ref_loaded
    st._low_dynamic(False)
    st._tick([0.0, -0.0520], 0.15, n=20)
    assert st._s_ref_loaded == ref, 'figure-8 中的越界值不得抬高已冻结的基准'
    assert not _released(st), '越界期间不得释放'


def test_no_release_before_reference_ready():
    """基准未建立 ⇒ 不判决(ratio 没有可信分母),最终交 unresolved。"""
    st = _Stub()
    st._low_dynamic(False)          # 全程高动态:窗口永远攒不满
    st._tick([0.0, -0.0180], 0.15, n=30)
    assert not st._moment_ref_ready
    st._tick([0.0032, 0.0], -0.005, n=40)   # 即使证据"充分"
    assert not _released(st), '基准未就绪时不得释放'


def test_overrange_clears_release_counters():
    """越界帧必须清 slow/fastB 持续计数,不能跨越界帧拼接。"""
    st = _Stub()
    st._low_dynamic(True)
    st._tick([0.0, -0.0180], 0.15, n=st.moment_ref_window + 2)
    assert st._moment_ref_ready
    st._tick([0.0030, 0.0], -0.005, n=st.s_release_persist - 1)   # 差一帧
    assert not _released(st)
    st._tick([0.0, -0.0500], -0.005, n=1)                          # 越界一帧
    assert st._s_low == 0, '越界帧必须清持续计数'
    assert st._dyn_overrange > 0


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
