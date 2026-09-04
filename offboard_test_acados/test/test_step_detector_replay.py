#!/usr/bin/env python3
# 机动中的质量突变检测:候选判据的离线回放验证(2026-08-26)。不需 acados/ROS。
#
# 要解决的问题
# ------------
# 2026-08-26 关掉外部 drop 信号那一轮(gviz_*_20260826_160719),drop 来了检测器
# 根本没看见,5 秒后发散;日志显示 drop 之后 16.6 秒才出现 "detector armed"。
#
# ⚠️ 撤回一个当时的误判:我曾断言根因是"figure-8 带载段的推力波动远超
#    settle_tol=0.5N,预热计数反复清零"。用本模块这份干净的 10Hz 回放数据核对,
#    **带载机动段 std 只有 0.261N,偏离中位 >0.5N 的帧仅占 4.5%** —— 波动根本
#    没有超门槛。原先那个 "92%" 的统计取自稀疏采样(2s 一条)且混入了发散段数据,
#    不成立。真实根因尚未定论,本模块**不对现有判据下结论**(见 legacy_detect 的
#    忠实性说明)。
#
# 无论根因为何,现有判据都依赖一条慢基线 + 一道预热门控;候选判据的价值在于
# **完全不需要基线,因此没有预热这个失效面**。这一点可以独立验证,与根因无关。
#
# 本模块**不改源文件**:候选判据以独立函数实现在这里,用真实回放数据验证通过之后,
# 再决定要不要移进 mhe_node。
#
# 回放数据
# --------
# fixtures/tphys_replay_20260826_165455.csv —— resid_internal 流导出的 T_phys 序列。
# 这一路**就是 _residual_detect 每拍拿到的那个量**(同一个 self.thrust_phys),10.0Hz,
# 所以回放等价于把检测器接到真实飞行上。该轮全程 solve failed=0(正常飞行,不是发散
# 数据),attach t=6.50s,figure-8 切入 t=19.5s,drop t=71.50s。
#
# 跑法:PYTHONPATH=<ws>/src/offboard_test_acados:<ws>/src/offboard_test:. python3 本文件

import os
import numpy as np

FIX = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   'fixtures', 'tphys_replay_20260826_165455.csv')
DT = 0.1
T_ATTACH, T_DROP, T_FIG8 = 6.50, 71.50, 19.5


def _load():
    t, T, att = [], [], []
    for line in open(FIX):
        if line.startswith('#') or line.startswith('t_s'):
            continue
        a, b, c = line.strip().split(',')
        t.append(float(a)); T.append(float(b)); att.append(int(c))
    return np.array(t), np.array(T), np.array(att)


# ======================= 现有判据(复刻,用于证明问题存在) =======================
def legacy_detect(T, dt=DT, baseline_tau=3.0, settle_tol=0.5,
                  warmup=20, thresh=1.5, persist=2):
    """复刻 mhe_node._residual_detect 的核心:慢基线 EMA + 预热门控 + 两段式确认。

    ⚠️⚠️ **这个复刻不忠实,不能用它给真实判据定罪。** 它省掉了真实代码里
    "事件过渡期由 scheduler 接管"那一段:真实实现在过渡期内每帧把 baseline 置
    None,直到过渡结束才重新播种 —— 那时 T_phys 已经稳定在**新稳态**上。而这个
    复刻在事件当帧就地播种,基线落在阶跃**中间**的瞬时值(实测 18.3N,而后续稳态
    21.9N),于是 dev 永久 >settle_tol、预热计数永远清零。两者行为完全不同。
    保留它只是为了给候选判据提供一个"有基线 + 有预热门控"的结构对照,
    **它的输出不构成对 mhe_node 的任何结论**。

    返回 (armed_idx, events):armed_idx = 首次武装的帧号(None=从未武装)。
    """
    a = dt / max(baseline_tau, dt)
    base = None
    settled = False
    stable = 0
    pending = 0
    armed_idx = None
    events = []
    for i, Ti in enumerate(T):
        if base is None:
            base = Ti; stable = 0; continue
        dev = abs(Ti - base)
        if not settled:
            if dev < settle_tol:
                stable += 1
                if stable >= warmup:
                    settled = True
                    if armed_idx is None:
                        armed_idx = i
            else:
                stable = 0
            continue
        if dev > thresh:
            pending += 1
            if pending >= persist:
                events.append(i); pending = 0
                base = None; settled = False; stable = 0   # 事件后重新播种
            continue
        pending = 0
        base = (1.0 - a) * base + a * Ti
    return armed_idx, events


# ============================ 候选判据:短窗阶跃检测 ============================
def step_detect(T, half=3, thresh=1.0, persist=2):
    """短窗前后均值差的阶跃检测器。

    思路:drop/attach 是**阶跃**,机动引起的推力起伏是**缓变**(figure-8 周期
    ~22s,z 起伏同量级)。取两段相邻的短窗求均值差:
      · 阶跃 → 幅度全额体现在差值里;
      · 缓变 → 窗口越短,窗内积累的变化越小。
    所以窗口要**短**。实测(见本文件末尾的可分性表):半窗 0.3s 区分度 3.9×,
    0.5s 降到 1.9×,0.8s 就完全不可分了 —— 与"窗口越长越抗噪"的直觉相反。

    与现有判据的本质差别:**不需要慢基线,因此没有"预热门控"这个失效面**。
    (不声称现有判据一定败在预热上 —— 那个根因尚未定论,见文件头的撤回说明。)

    persist(连续超阈帧数)不只是抗噪:drop 的推力阶跃是**永久**的(质量真的变了),
    轨迹切换引起的推力变化是**暂态**的(很快稳到新工作点),所以持续帧数本身带
    区分力。实测(半窗 0.3s):
        persist=1 thresh=1.0 → drop 延迟 0.20s 幅度 -1.07N(余量仅 1.1×),切换误检 2
        persist=2 thresh=1.0 → drop 延迟 0.30s 幅度 -2.37N(余量 2.4×),切换误检 2   ← 默认
        persist=3 thresh=1.2 → 切换误检 0,但 drop 余量掉到 1.3×
    取 persist=2:延迟 0.3s 可接受,余量 2.4× 才经得起换工况。

    返回 [(帧号, 有符号差值)]:差值 <0 = 掉推力(drop),>0 = 加推力(attach)。
    """
    ev = []
    pending = 0
    last = -10 ** 9
    for i in range(2 * half, len(T)):
        pre = T[i - 2 * half:i - half].mean()
        post = T[i - half:i].mean()
        d = post - pre
        if abs(d) > thresh:
            pending += 1
            if pending >= persist and i - last > 2 * half:
                ev.append((i, d)); last = i; pending = 0
        else:
            pending = 0
    return ev


# ================================== 测试 ==================================
def test_fixture_sane():
    t, T, att = _load()
    assert len(t) > 1000, f'回放数据太短: {len(t)}'
    assert abs(np.median(np.diff(t)) - DT) < 1e-6, '不是 10Hz'
    i_att = np.where(np.diff(att) > 0)[0]
    i_drp = np.where(np.diff(att) < 0)[0]
    assert abs(t[i_att[0] + 1] - T_ATTACH) < 0.15
    assert abs(t[i_drp[0] + 1] - T_DROP) < 0.15
    print(f'[0] fixture OK (n={len(t)}, {t[-1]:.0f}s, attach {T_ATTACH}s, drop {T_DROP}s)')


def test_maneuver_noise_is_below_settle_tol():
    """把"机动波动超过 settle_tol"这个**错误假设**钉死为不成立。

    这条断言存在的意义是防止它被重新捡起来:带载机动段的推力波动其实很小,
    预热门控的失效(如果确实失效)不能归因于波动幅度。
    """
    t, T, _ = _load()
    seg = T[(t > 25) & (t < T_DROP - 1)]
    over = float(np.mean(np.abs(seg - np.median(seg)) > 0.5))
    assert seg.std() < 0.4, f'带载机动段 std={seg.std():.3f}N 与记录不符'
    assert over < 0.15, f'超 settle_tol 的帧占比 {100*over:.0f}%,与记录不符'
    print(f'[1] 机动波动 < settle_tol OK '
          f'(std={seg.std():.3f}N, 仅 {100*over:.1f}% 的帧偏离 >0.5N '
          f'→ "波动过大导致预热失败"不成立)')


def test_legacy_replica_loses_baseline_after_event():
    """记录复刻版的失效模式,并**明确它只属于复刻**。

    复刻在事件当帧就地播种基线,落在阶跃中间 → 之后永远预热不完。真实实现靠
    scheduler 过渡期反复置 None 规避了这一点。这条测试是为了让这个区别留在
    代码里,而不是留在某次对话里。
    """
    t, T, _ = _load()
    armed, events = legacy_detect(T)
    assert armed is not None and t[armed] < 5.0, '复刻应在起飞段武装一次'
    assert len(events) == 1, f'复刻应只触发 1 次,实得 {len(events)}'
    got_drop = any(abs(t[i] - T_DROP) < 2.0 for i in events)
    assert not got_drop, '复刻不应抓到 drop(它在第一次事件后就失去基线了)'
    print(f'[2] 复刻失效模式 OK (t={t[armed]:.1f}s 武装 → t={t[events[0]]:.1f}s '
          f'触发 → 此后基线丢失,抓不到 t={T_DROP}s 的 drop;**仅复刻如此**)')


def test_step_detects_drop():
    """候选判据必须抓到 drop,且延迟在半个窗口量级。"""
    t, T, _ = _load()
    ev = step_detect(T)
    drops = [(t[i], d) for i, d in ev if d < 0 and abs(t[i] - T_DROP) < 2.0]
    assert drops, f'未检出 drop;检出的事件在 t={[round(t[i],1) for i,_ in ev]}'
    t_hit, amp = drops[0]
    lag = t_hit - T_DROP
    assert 0 <= lag < 0.5, f'检出延迟 {lag:.2f}s 过大'
    assert amp < -2.0, f'drop 幅度应显著为负且有余量,实得 {amp:.2f}N'
    assert abs(amp) / 1.0 > 2.0, f'检出余量仅 {abs(amp):.2f}×阈值,换个工况就会漏'
    print(f'[3] 抓到 drop OK (t={t_hit:.2f}s, 延迟 {lag:.2f}s, Δ={amp:+.2f}N)')


def test_step_no_false_alarm_in_figure8():
    """候选判据在纯机动段(figure-8 带载稳态)必须零误触发。"""
    t, T, _ = _load()
    ev = step_detect(T)
    false = [t[i] for i, _ in ev if 25.0 < t[i] < T_DROP - 1.0]
    assert not false, f'figure-8 段误触发 {len(false)} 次 @ {[round(x,1) for x in false]}'
    post = [t[i] for i, _ in ev if T_DROP + 3.0 < t[i] < t[-1] - 1.0]
    assert not post, f'drop 后空机机动段误触发 {len(post)} 次'
    print('[4] figure-8 带载稳态段 + 空机段 零误触发 OK')


def test_step_detects_attach_direction():
    """attach(推力上升)也应被抓到,且方向为正 —— 方向决定要不要释放几何。"""
    t, T, _ = _load()
    ev = step_detect(T)
    ups = [(t[i], d) for i, d in ev if d > 0 and abs(t[i] - T_ATTACH) < 6.0]
    assert ups, 'attach 段未检出上升型事件'
    assert all(d > 0 for _, d in ups)
    print(f'[5] attach 方向判别 OK (t={ups[0][0]:.2f}s, Δ={ups[0][1]:+.2f}N)')


def test_window_length_tradeoff():
    """窗口越长越好是错的:量化区分度随半窗的变化,锁住这个反直觉的结论。"""
    t, T, _ = _load()
    rows = []
    for half in (3, 5, 8):
        d = np.array([T[i-half:i].mean() - T[i-2*half:i-half].mean()
                      for i in range(2*half, len(T))])
        tt = t[2*half:]
        man = np.abs(d[(tt > 25) & (tt < 70)]).max()
        drp = np.abs(d[(tt > T_DROP) & (tt < T_DROP + 1.5)]).max()
        rows.append((half, man, drp, drp / man))
    for half, man, drp, r in rows:
        print(f'      半窗 {half*DT:.1f}s: 机动段max={man:.3f}N drop={drp:.3f}N 区分度={r:.1f}×')
    assert rows[0][3] > rows[1][3] > rows[2][3], '区分度应随窗口变长而单调下降'
    assert rows[0][3] > 3.0, f'0.3s 半窗区分度 {rows[0][3]:.1f}× 不足'
    assert rows[2][3] < 1.5, '0.8s 半窗应已不可分'
    print(f'[6] 窗口长度权衡 OK (短窗更好,与"越长越抗噪"的直觉相反)')


def test_threshold_margin():
    """阈值 1.0N 两侧都要有余量,否则换个工况就失效。"""
    t, T, _ = _load()
    d = np.array([T[i-3:i].mean() - T[i-6:i-3].mean() for i in range(6, len(T))])
    tt = t[6:]
    man = np.abs(d[(tt > 25) & (tt < 70)]).max()
    # 用**实际检出那一帧**的幅度,不是窗口内的峰值 —— 后者会高估余量
    ev = step_detect(T)
    amp = abs([d for i, d in ev if d < 0 and abs(t[i] - T_DROP) < 2.5][0])
    assert man < 1.0 * 0.85, f'误触发余量不足: 机动段 max {man:.3f}N vs 阈值 1.0N'
    assert amp > 1.0 * 2.0, f'检出余量不足: 实际检出 {amp:.3f}N vs 阈值 1.0N'
    print(f'[7] 阈值 1.0N 余量 OK (下方留 {100*(1-man/1.0):.0f}%, '
          f'上方留 {amp/1.0:.1f}× — 按实际检出帧算,非窗口峰值)')


def test_known_limitation_trajectory_switch():
    """**已知限制**:轨迹切换(figure-8 切入)本身就是一次真实的推力阶跃,
    候选判据会把它一并检出 —— 实测 t≈22.7s(−1.69N)与 t≈24.5s(+1.18N)。

    这不是判据的 bug,是推力通道的**固有可辨识性边界**:"质量变了"和"要飞的
    轨迹变了"都表现为 T_phys 阶跃,单看推力无法区分。记忆里 C.3 风扰消融得到过
    同类结论(持续垂直力与质量突变在推力通道上不可分)。

    要消掉它需要额外信息,可选:
      · 用 T/a 残差代替 T 本身(轨迹切换时 a 也同步阶跃,质量突变时不会);
      · 或由控制器在自己切换轨迹后短暂抑制检测 —— 这属于"控制器知道自己干了
        什么",与"外部告诉估计器载荷没了"不是一回事,语义上可接受。
    在选定方案之前,这条测试把现状钉住,防止它被当成回归。
    """
    t, T, _ = _load()
    ev = step_detect(T)
    sw = [(round(t[i], 1), round(d, 2)) for i, d in ev if T_FIG8 < t[i] < 25.0]
    assert sw, '预期 figure-8 切入段有检出(已知限制),实际没有 —— 数据或判据变了'
    print(f'[8] 已知限制:轨迹切换被一并检出 {sw} '
          f'(推力通道固有,不是 bug)')


# ==================== 接进源码后:回放真实的 _residual_detect ====================
# 上面测的是独立函数;下面把同一份数据喂给 mhe_node 里**真实的**实现,验证接线。

class _Sched:
    """模拟 EventWeightScheduler:事件后 hold 帧内 event_frame 非 None
    (真实实现里事件要滑过整个窗口才清),期间 _residual_detect 走过渡期分支。"""
    def __init__(self, hold=20):
        self.event_frame = None; self.notified = []; self._left = 0; self.hold = hold
    def notify_event(self, f):
        self.event_frame = f; self.notified.append(f); self._left = self.hold
    def tick(self):
        if self._left > 0:
            self._left -= 1
            if self._left == 0:
                self.event_frame = None


def _replay_real(step_on, T, att=None, release_thresh=2.0):
    """把 T 序列逐帧喂给真实的 MHENode._residual_detect。

    att(fixture 的 payload_attached 列)用来忠实地驱动 _payload_attached:
    起飞段它是 False,attach 上升沿才置 True(模拟 attach_event_cb)。**不能**
    一上来就设 True —— 那样起飞段的推力阶跃(t≈3.8s,幅度够大)会被当成 drop
    误释放几何,而真实流程里那时钩子上根本没有东西。
    """
    from offboard_test_acados import mhe_node as mn
    from offboard_test_acados.mhe_params import p as mhe_p

    class _S:
        pass
    s = _S()
    s.resid_ema_alpha = mhe_p.dt / 3.0
    s.confirm_thresh = 1.5
    s.resid_persist = 2
    s.resid_warmup = 20
    s.resid_settle_tol = 0.5
    s._resid_baseline = None; s._resid_pending = 0; s._resid_cross_frame = None
    s._resid_settled = False; s._resid_stable = 0
    s.frames = 0
    s.scheduler = _Sched()
    s._payload_attached = bool(att[0]) if att is not None else True
    # 2026-09-04:释放门控换成估计器自主判定的 _payload_present。回放里
    # att[] 描述的就是"载荷physically在不在",所以自主状态跟它同步起步。
    s._payload_present = s._payload_attached
    s._load_armed = s._payload_attached
    s.x_meas = np.zeros(13)
    s.payload_exit_steady_omega = 0.15
    s.payload_exit_steady_vel = 0.20
    s.s_decay_log_frames = 0
    s._s_decay_n = 0
    s._s_out_prev = np.zeros(2)
    s.m_est = mhe_p.m_B + 0.30
    s.payload_present_enter_mp = 0.09
    s.payload_present_enter_persist = 20
    s.payload_present_exit_mp = 0.03
    s.payload_present_exit_persist = 20
    s._present_hi = 0; s._present_lo = 0
    s._s_release_latched = False; s._s_peak = 0.0; s._s_low = 0
    s.c_xy_est = np.zeros(2); s._c_xy_inited = True
    s.c_xy_est_pub = type('_P', (), {'publish': lambda self, m: None})()
    s._release_payload = mn.MHENode._release_payload.__get__(s)
    s._update_payload_presence = mn.MHENode._update_payload_presence.__get__(s)
    s._mass_observable = mn.MHENode._mass_observable.__get__(s)
    s.attach_offset = np.array([0.006, -0.093, -0.516])
    s.resid_release_geom = True
    s.resid_step_enable = step_on
    s.resid_step_half = 3
    s.resid_step_thresh = 1.0
    s.resid_step_persist = 2
    s.resid_step_release_thresh = release_thresh
    s._tphys_hist = []; s._step_pending = 0
    s.logs = []
    class _L:
        def __init__(self, o): self.o = o
        def info(self, m): self.o.logs.append(m)
        def warn(self, m): self.o.logs.append(m)
    s.get_logger = lambda: _L(s)
    released_at = []
    for i, Ti in enumerate(T):
        # attach 上升沿:模拟 attach_event_cb 把物理状态置真
        if att is not None and i > 0 and att[i] and not att[i-1]:
            s._payload_attached = True
            s._payload_present = True
            s._load_armed = True
            s.attach_offset = np.array([0.006, -0.093, -0.516])
        s.thrust_phys = float(Ti); s.frames += 1
        was = s._payload_attached
        mn.MHENode._residual_detect(s)
        if was and not s._payload_attached:
            released_at.append(i)
        s.scheduler.tick()
    return s, released_at


def test_real_detector_step_off_is_legacy():
    """默认(关)时,新代码路径完全不参与 —— 历史行为不变。"""
    t, T, att = _load()
    s, rel = _replay_real(False, T, att)
    assert not any('step-detect' in m for m in s.logs), '关闭时不该有阶跃判据日志'
    assert s._tphys_hist == [] or len(s._tphys_hist) <= 6, '历史缓冲不该无界增长'
    print(f'[9] 源码默认档(关)= legacy OK (阶跃判据零参与, '
          f'事件 {len(s.scheduler.notified)} 次)')


def test_real_detector_step_on_catches_drop():
    """开启后:真实实现能抓到 drop 并自主释放几何。"""
    t, T, att = _load()
    s, rel = _replay_real(True, T, att)
    steps = [m for m in s.logs if '阶跃自检测' in m]
    assert steps, '阶跃判据未触发'
    assert rel, '未释放载荷几何'
    t_rel = t[rel[0]]
    assert abs(t_rel - T_DROP) < 0.8, f'释放时刻 t={t_rel:.2f}s 偏离 drop({T_DROP}s)'
    assert not s._payload_attached
    assert s.attach_offset is None
    # 注:MHE 侧棘轮 _m_p_hat_ratchet 已随 2026-08-26 去先验改造移除
    # (几何标度改用常数包线,不再读 m_est),释放路径不再有它要清。
    print(f'[10] 源码开启档抓到 drop OK (释放于 t={t_rel:.2f}s, '
          f'延迟 {t_rel-T_DROP:+.2f}s, 阶跃事件 {len(steps)} 次)')


def test_real_detector_release_threshold_blocks_switch():
    """分级门槛必须挡住轨迹切换:figure-8 切入那次 |Δ|≈1.16N < 2.0N,
    应当只降权、不释放几何 —— 否则模型会丢掉还挂在钩子上的载荷。"""
    t, T, att = _load()
    s, rel = _replay_real(True, T, att)
    early = [i for i in rel if t[i] < T_DROP - 2.0]
    assert not early, (
        f'在 drop 之前就释放了几何 @ t={[round(t[i],1) for i in early]} —— '
        '分级门槛失效')
    blocked = [m for m in s.logs if '只降权不释放几何' in m]
    print(f'[11] 分级门槛挡住轨迹切换 OK '
          f'(drop 前零释放; 被拦下的下降型事件 {len(blocked)} 次)')


if __name__ == '__main__':
    test_fixture_sane()
    test_maneuver_noise_is_below_settle_tol()
    test_legacy_replica_loses_baseline_after_event()
    test_step_detects_drop()
    test_step_no_false_alarm_in_figure8()
    test_step_detects_attach_direction()
    test_window_length_tradeoff()
    test_threshold_margin()
    test_known_limitation_trajectory_switch()
    test_real_detector_step_off_is_legacy()
    test_real_detector_step_on_catches_drop()
    test_real_detector_release_threshold_blocks_switch()
    print('\nall passed')
