#!/usr/bin/env python3
"""成果演示视频 —— 数据动画层(叠在 Gazebo/RViz 录屏之上的底部条带)。

从一次 run_sitl_gripper_viz.sh 完整任务的 gviz_nmpc_/gviz_mhe_ 日志渲染
四联滚动曲线(位置误差 / MHE 质量 / 一阶质量矩 |s_xy| / MHE 惯量 Jxx),
带阶段中文标注和当前时刻游标,输出**带 alpha 通道**的 webm,便于
compose_demo.sh 直接 overlay 到录屏画面底部。

当前主线不消费 attach/drop 通知。因此本脚本不能再依赖旧的
``attach offset received`` 或 ``[dJ_online]`` 日志：一阶质量矩从 MHE 的
``s_hat``/``s-decay`` 读取，惯量按 payload_estimate 接口的同一代数由质量
重建。载荷真值从该轮 ``world -> box mass=... ecc=...`` 启动日志自动读取；
找不到时直接报错，禁止静默回退到历史默认 0.30 kg。

关键:时间轴用**任务时间 t**(NMPC 日志里的 t=…s),并把墙钟→任务时间的
偏移 off 写进 meta.json —— 录屏第 k 秒对应任务时间 = rec_start + k - off,
所以画面和曲线是按时间戳硬对齐的,不靠目测对齐 drop 那一帧。

用法:
  python3 make_overlay.py [--stamp 20260715_204725] [--fps 25] [--t-end 115]
输出:
  nmpc_test_results/demo_video/overlay_<stamp>.webm
  nmpc_test_results/demo_video/overlay_<stamp>.meta.json
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

RES = os.path.expanduser('~/ros2_ws_HJH/nmpc_test_results')
OUT = os.path.join(RES, 'demo_video')

# 复用论文图脚本里已经调通的日志正则(单一事实来源,别再抄一份)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import make_paper_figs as mpf   # noqa: E402

M_DRY = 2.064            # 裸机质量 [kg]
J_DRY = 0.0142           # 裸机 Jxx [kg·m²](acados_params;文档里那个 0.0217 是错的)
RZ_PRIOR = 0.47          # |mhe_params.rz_prior| [m]，payload_estimate: dJxy=μ·rz²

# 深色底上的配色:主曲线亮蓝/亮绿,真值中性灰虚线
C_EST, C_TRUTH, C_ERR = '#5aa9ff', '#9be27a', '#ffd166'
FG, FG2 = '#f2f1ee', '#a9a7a2'
PANEL_RGBA = (0.06, 0.06, 0.08, 0.72)   # 半透明深色条带

RUN_CFG = re.compile(
    r'world -> box mass=([-+0-9.eE]+).*?\becc=([-+0-9.eE]+)')
S_HAT = re.compile(
    r'\[INFO\] \[(\d+\.\d+)\].*\[truth\].*'
    r's_hat=\[([+-][\d.]+),([+-][\d.]+)\]')
S_DECAY = re.compile(
    r'\[INFO\] \[(\d+\.\d+)\].*\[s-decay\].*'
    r'\|s_out\|=([\d.]+).*conf=([\d.]+).*health=([01])')
FLIGHT_CONF = re.compile(
    r'\[flight-diag\] t=([\d.]+)s.*health=([01]) '
    r'age=([\d.]+)s conf=([\d.]+)')


def resolve_run_config(stamp, payload=None, eccentricity=None):
    """Return (payload, eccentricity, source), refusing stale truth defaults."""
    if payload is not None and eccentricity is not None:
        return float(payload), abs(float(eccentricity)), 'command line'

    candidates = sorted(glob.glob(os.path.join(RES, f'*{stamp}*.log')))
    latest = os.path.join(OUT, 'viz_launch.log')
    if os.path.exists(latest):
        candidates.append(latest)
    seen = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        try:
            text = open(path, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        # viz_launch.log is overwritten every run; accept it only when it names
        # this stamp.  A stamp-bearing filename is already unambiguous.
        if stamp not in os.path.basename(path) and stamp not in text:
            continue
        match = RUN_CFG.search(text)
        if not match:
            continue
        p_log, e_log = map(float, match.groups())
        return (float(payload) if payload is not None else p_log,
                abs(float(eccentricity)) if eccentricity is not None
                else abs(e_log), path)

    missing = []
    if payload is None:
        missing.append('--payload')
    if eccentricity is None:
        missing.append('--ecc')
    raise SystemExit(
        f'找不到 {stamp} 的 world -> box 配置；请显式给出 '
        + ' 和 '.join(missing) + '，避免画出错误真值')


def _cjk_font():
    for name in ('Noto Sans CJK SC', 'Noto Sans CJK TC', 'AR PL UMing CN'):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            return name
        except Exception:
            continue
    return None


def load_mission(stamp, t_end):
    """解析一次完整任务,返回按任务时间对齐的各条曲线 + 阶段时刻 + 墙钟偏移。"""
    nmpc = os.path.join(RES, f'gviz_nmpc_{stamp}.log')
    mhe = os.path.join(RES, f'gviz_mhe_{stamp}.log')
    for p in (nmpc, mhe):
        if not os.path.exists(p):
            sys.exit(f'找不到日志: {p}')

    tp, ep, tw, ew = [], [], [], []
    c_zero = [None]        # 自主释放的墙钟时刻，用于衔接 s_hat -> s_out
    tj, jv = [], []        # legacy NMPC [dJ_online]
    tconf, vconf, hconf = [], [], []
    phases, offsets = {}, []
    r_att = [None]
    for line in open(nmpc):
        mw = mpf.NMPC_WIN.search(line)
        if mw:
            tw.append(float(mw.group(3))); ew.append(float(mw.group(4)))
            offsets.append(mpf._wall(mw) - float(mw.group(3)))
            continue
        mp_ = mpf.NMPC_PERIODIC.search(line)
        if mp_:
            tp.append(float(mp_.group(3))); ep.append(float(mp_.group(4)))
            offsets.append(mpf._wall(mp_) - float(mp_.group(3)))
        mf = mpf.PHASE.search(line)
        if mf and mf.group(4) not in phases:
            phases[mf.group(4)] = float(mf.group(3))
        # 2026-08-21:抓取几何真值(Gazebo 给的 box 相对机体偏移),给 c_y 真值用
        ma = re.search(r'attach offset received: box-drone = '
                       r'\[\s*([-+0-9.]+),\s*([-+0-9.]+),', line)
        if ma and r_att[0] is None:
            r_att[0] = (float(ma.group(1)), float(ma.group(2)))
        # 模型侧 dJ 的在线轨迹(dj_track_mest=true 才有)。⚠️ 这条日志在 **NMPC**
        # 日志里,别放到下面的 MHE 循环去(2026-08-30 第一版就放错了,解析恒为空)。
        # off 要等这个循环跑完才算得出来,所以先存墙钟,循环后统一换算。
        mj = re.search(r'\[INFO\] \[(\d+\.\d+)\].*\[dJ_online\].*'
                       r'dJ=([\d.]+)', line)
        if mj:
            tj.append(float(mj.group(1))); jv.append(float(mj.group(2)))
        mcf = FLIGHT_CONF.search(line)
        if mcf:
            tconf.append(float(mcf.group(1)))
            hconf.append(float(mcf.group(2)))
            vconf.append(float(mcf.group(4)))
    if not offsets:
        sys.exit(f'{nmpc}: 没解析到任何 NMPC 时间行')
    off = float(np.median(offsets))
    tj = [t - off for t in tj]      # 墙钟 -> 任务时间(见上面存墙钟的理由)

    tm, mm, tc, cxv, cyv, cy_tr = [], [], [], [], [], []
    ts_raw, sv_raw = [], []
    ts_decay, sv_decay, conf_decay, health_decay = [], [], [], []
    for line in open(mhe):
        ml = mpf.MASS_LN.search(line) or mpf.EV_LINE.search(line)
        if ml:
            tm.append(mpf._wall(ml) - off); mm.append(float(ml.group(3)))
            continue
        mc = mpf.CXY.search(line)
        if mc:
            tc.append(mpf._wall(mc) - off)
            cxv.append(float(mc.group(3))); cyv.append(float(mc.group(4)))
            cy_tr.append(float(mc.group(6)) if mc.group(6) else np.nan)
        ms = S_HAT.search(line)
        if ms:
            ts_raw.append(float(ms.group(1)) - off)
            sv_raw.append(float(np.hypot(float(ms.group(2)),
                                         float(ms.group(3)))))
        msd = S_DECAY.search(line)
        if msd:
            ts_decay.append(float(msd.group(1)) - off)
            sv_decay.append(float(msd.group(2)))
            conf_decay.append(float(msd.group(3)))
            health_decay.append(float(msd.group(4)))
        # 2026-08-30 起 MHE 在载荷离机时把 c_xy_est **清零并发一次 0**(见
        # mhe_node._release_payload)。这一下不走 [c_xy_est] 周期日志(那条被
        # _c_xy_inited 门着),所以单独抓,否则曲线停在最后一个带载值、角落读数
        # 也一直显示那个陈旧值 —— 而演示要说明的恰恰是"看门狗把它归零了"。
        mz = re.search(r'\[c_xy_est\] 载荷释放\(', line)
        if mz:
            mw2 = mpf.WALL_RE.search(line) if hasattr(mpf, 'WALL_RE') else None
            _w = None
            if mw2:
                _w = float(mw2.group(1))
            else:
                _m = re.search(r'\[INFO\] \[(\d+\.\d+)\]', line)
                if _m:
                    _w = float(_m.group(1))
            if _w is not None:
                c_zero[0] = _w - off

    def _clip_sort(t, *ys):
        t = np.asarray(t, float)
        keep = (t >= 0) & (t <= t_end)
        t = t[keep]; ys = [np.asarray(y, float)[keep] for y in ys]
        i = np.argsort(t)
        return (t[i],) + tuple(y[i] for y in ys)

    t_err, v_err = _clip_sort(np.concatenate([tp, tw]),
                              np.concatenate([ep, ew]))
    t_m, v_m = _clip_sort(tm, mm)
    t_c, v_cx, v_c, v_ct = _clip_sort(tc, cxv, cyv, cy_tr)
    # c_xy 只在挂载期间有物理意义(drop 后发布端冻结的 EMA 是残留,画了误导)
    # 载荷离机的时刻 = 计划投放 DROP 或意外脱落 LOST,取先发生的那个
    t_off = min(phases.get('DROP', t_end), phases.get('LOST', t_end))
    keep_c = t_c <= t_off
    t_c, v_cx, v_c, v_ct = (t_c[keep_c], v_cx[keep_c],
                              v_c[keep_c], v_ct[keep_c])
    # 补一个**真实发生过**的零采样:估计器在载荷离机那一刻确实清零并发布了 0。
    # 只有日志里出现过归零行才补 —— 2026-08-30 之前的旧日志没有这条(那时的
    # 行为是冻结残留值),补了就是伪造。
    if c_zero[0] is not None and 0 <= c_zero[0] <= t_end:
        t_c = np.append(t_c, c_zero[0])
        v_cx = np.append(v_cx, 0.0)
        v_c = np.append(v_c, 0.0)
        v_ct = np.append(v_ct, 0.0)   # 载荷没了,真值偏心也是 0

    t_j, v_j = _clip_sort(tj, jv)
    # 载荷离机时模型 dJ 确实被归零(LOST 走 payload_lost_cb、计划 DROP 走
    # _grip_drop_phase,两条都显式清 dJ_est)。而 [dJ_online] 日志在
    # _update_online_geometry 的 grip_dropped 分支里提前 return,所以**不会**
    # 打出那个 0 —— 曲线会停在最后一个带载值。这里按已发生的事补上。
    if len(t_j) and ('LOST' in phases or 'DROP' in phases) and t_off <= t_end:
        t_j = np.append(t_j[t_j <= t_off], t_off)
        v_j = np.append(v_j[:len(t_j) - 1], 0.0)

    # 无事件主线的权威量是一阶质量矩 s，不是 c_xy。释放前的
    # s_hat 来自 MHE 窗口解；释放后优先用 10 Hz [s-decay] 的真正
    # 发布端 s_out。旧日志没有 s_hat 时，才用 m*c 做兼容回退。
    t_s0, v_s0 = _clip_sort(ts_raw, sv_raw)
    t_sd, v_sd, v_cd, v_hd = _clip_sort(
        ts_decay, sv_decay, conf_decay, health_decay)
    if not len(t_s0) and len(t_c) and len(t_m):
        m_at_c = np.interp(t_c, t_m, v_m)
        t_s0 = t_c.copy()
        v_s0 = m_at_c * np.hypot(v_cx, v_c)
    if len(t_sd):
        release_t = float(t_sd[0])
        keep = t_s0 < release_t
        t_s = np.concatenate([t_s0[keep], t_sd])
        v_s = np.concatenate([v_s0[keep], v_sd])
    else:
        t_s, v_s = t_s0, v_s0

    # 新主线不再产生 [dJ_online]。用 payload_estimate 的同一代数
    # dJ_xy=μ*r_z² 从 MHE 质量重建；旧日志有 [dJ_online] 时仍优先
    # 画真实的 NMPC 模型值。payload_output_tau 仅 0.25 s，对 2 Hz 取样
    # 的可视化误差远小于旧脚本的“整条线缺失”。
    if not len(t_j) and len(t_m):
        mp = np.maximum(v_m - M_DRY, 0.0)
        mu = M_DRY * mp / np.maximum(v_m, 1e-6)
        t_j = t_m.copy()
        v_j = mu * RZ_PRIOR ** 2

    # confidence 在常规飞行诊断里是 5 s 一点，drop 后由 s-decay
    # 补到 10 Hz。将它放在顶部读数，不与惯量共用小坐标轴。
    if len(t_sd):
        t_conf = np.concatenate([np.asarray(tconf), t_sd])
        v_conf = np.concatenate([np.asarray(vconf), v_cd])
        h_conf = np.concatenate([np.asarray(hconf), v_hd])
    else:
        t_conf = np.asarray(tconf, float)
        v_conf = np.asarray(vconf, float)
        h_conf = np.asarray(hconf, float)
    if len(t_conf):
        order = np.argsort(t_conf)
        keep = (t_conf[order] >= 0) & (t_conf[order] <= t_end)
        order = order[keep]
        t_conf, v_conf, h_conf = (t_conf[order], v_conf[order], h_conf[order])

    return dict(off=off, phases=phases, t_err=t_err, v_err=v_err,
                t_m=t_m, v_m=v_m, t_c=t_c, v_c=v_c, v_ct=v_ct,
                t_s=t_s, v_s=v_s, t_j=t_j, v_j=v_j,
                t_conf=t_conf, v_conf=v_conf, h_conf=h_conf,
                r_att=r_att[0])


PHASE_CN = {'ATTACH': '接近并抓取载荷', 'LIFT': '抬升(有效质量阶跃)',
            'DYNAMIC': '8 字动态轨迹跟踪', 'DROP': '投放载荷(突卸扰动)',
            'LOST': '载荷意外脱落(看门狗复位内环增益)'}
PHASE_EN = {'ATTACH': 'approach & grasp payload', 'LIFT': 'lift (effective mass step)',
            'DYNAMIC': 'figure-8 trajectory tracking', 'DROP': 'release (sudden unloading)',
            'LOST': 'unplanned payload loss (watchdog resets inner-loop gains)'}


def build(d, payload, eccentricity, t_end, fps, width, height, out_path,
          lang='zh'):
    cjk = _cjk_font()
    use_cn = (lang == 'zh') and (cjk is not None)
    plt.rcParams.update({
        'font.size': 11, 'axes.labelsize': 11, 'legend.fontsize': 10,
        'xtick.labelsize': 9.5, 'ytick.labelsize': 9.5,
        'text.color': FG, 'axes.labelcolor': FG,
        'xtick.color': FG2, 'ytick.color': FG2,
        'axes.edgecolor': FG2, 'axes.linewidth': 0.8,
        'axes.grid': True, 'grid.color': '#3a3a40', 'grid.linewidth': 0.6,
        'axes.spines.top': False, 'axes.spines.right': False,
        'legend.frameon': False, 'figure.dpi': 100,
    })
    if cjk:
        plt.rcParams['font.sans-serif'] = [cjk] + \
            plt.rcParams.get('font.sans-serif', [])
        plt.rcParams['axes.unicode_minus'] = False

    fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
    fig.patch.set_facecolor(PANEL_RGBA[:3])
    fig.patch.set_alpha(PANEL_RGBA[3])
    # 2026-08-30 由三栏改四栏(加惯量 J)。栏宽/间距同步收窄,末栏右边缘
    # 0.045+3*0.238+0.185=0.944,仍留得下读数文字。
    axes = [fig.add_axes([0.045 + i * 0.238, 0.20, 0.185, 0.60])
            for i in range(4)]
    for ax in axes:
        ax.set_facecolor((0, 0, 0, 0))
        ax.set_xlim(0, t_end)
        ax.set_xlabel('任务时间 [s]' if use_cn else 'mission time [s]')

    ph = d['phases']
    t_att = ph.get('ATTACH', 7.0)
    # 夹爪刚闭合时箱子仍由地面承重；对“电机转速+里程计”的 MHE，
    # 有效质量/质量矩/惯量真值应从 LIFT 起算，否则会误导观众认为
    # attach 指令后估计器应立即跳变。
    t_load = ph.get('LIFT', t_att)
    t_drop = min(ph.get('DROP', t_end), ph.get('LOST', t_end))

    # 1) 位置误差
    axes[0].set_ylabel('位置误差 [m]' if use_cn else 'position error [m]')
    axes[0].set_ylim(0, max(0.35, float(np.nanmax(d['v_err'])) * 1.1))
    # 2) 质量:真值阶跃 + MHE 估计
    axes[1].set_ylabel('质量 [kg]' if use_cn else 'mass [kg]')
    mass_all = np.concatenate([
        np.asarray(d['v_m'], float), [M_DRY, M_DRY + payload]])
    mass_pad = max(0.035, 0.12 * (float(np.nanmax(mass_all))
                                 - float(np.nanmin(mass_all))))
    axes[1].set_ylim(float(np.nanmin(mass_all)) - mass_pad,
                     float(np.nanmax(mass_all)) + mass_pad)
    axes[1].plot([0, t_load, t_load, t_drop, t_drop, t_end],
                 [M_DRY, M_DRY, M_DRY + payload, M_DRY + payload, M_DRY, M_DRY],
                 color=C_TRUTH, lw=1.4, ls='--',
                 label='有效真值' if use_cn else 'effective truth')
    # 3) 一阶质量矩 |s_xy|。用模长可在不消费 attach 方向通知的前提下
    # 展示主线的权威输出，并能直接看出 drop 后 s_out 的连续衰减。
    axes[2].set_ylabel(r'一阶质量矩 $|s_{xy}|$ [kg·m]' if use_cn
                       else r'first mass moment $|s_{xy}|$ [kg m]')
    s_true = payload * abs(float(eccentricity))
    axes[2].plot([0, t_load, t_load, t_drop, t_drop, t_end],
                 [0.0, 0.0, s_true, s_true, 0.0, 0.0],
                 color=C_TRUTH, lw=1.4, ls='--',
                 label='有效真值' if use_cn else 'effective truth')
    s_max = max(s_true, float(np.nanmax(d['v_s'])) if len(d['v_s']) else 0.0,
                0.006)
    axes[2].set_ylim(0.0, 1.25 * s_max)

    # 4) 惯量 Jxx:MHE payload_estimate 的 J_dry+dJ vs 同口径真值。
    #    这里展示的是连续估计量，不把 attach 初始的 0.30 kg 安全
    #    bootstrap 误标成物理真值。
    _mu = M_DRY * payload / (M_DRY + payload)
    _dJ_true = _mu * RZ_PRIOR ** 2
    axes[3].set_ylabel('惯量 $J_{xx}$ [kg·m$^2$]' if use_cn
                       else 'inertia $J_{xx}$ [kg m$^2$]')
    j_all = J_DRY + np.asarray(d['v_j'], float)
    j_hi = max(J_DRY + _dJ_true,
               float(np.nanmax(j_all)) if len(j_all) else J_DRY)
    j_pad = max(0.003, 0.12 * (j_hi - J_DRY))
    axes[3].set_ylim(max(0.0, J_DRY - j_pad), j_hi + j_pad)
    axes[3].plot([0, t_load, t_load, t_drop, t_drop, t_end],
                 [J_DRY, J_DRY, J_DRY + _dJ_true, J_DRY + _dJ_true,
                  J_DRY, J_DRY],
                 color=C_TRUTH, lw=1.4, ls='--',
                 label='有效真值' if use_cn else 'effective truth')

    # 阶段竖线(全程静态,避免逐帧闪烁)
    for ax in axes:
        for k, tv in ph.items():
            ax.axvline(tv, color=FG2, lw=0.7, ls=':', alpha=0.7)

    # 动态元素
    ln_err, = axes[0].plot([], [], color=C_ERR, lw=1.8)
    ln_m, = axes[1].plot([], [], color=C_EST, lw=1.8,
                         label='MHE 估计' if use_cn else 'MHE estimate')
    ln_s, = axes[2].plot([], [], color=C_EST, lw=1.8,
                         label='MHE 估计' if use_cn else 'MHE estimate')
    ln_j, = axes[3].plot([], [], color=C_EST, lw=1.8,
                         label='MHE 估计' if use_cn else 'MHE estimate')
    cursors = [ax.axvline(0, color=FG, lw=1.0, alpha=0.85) for ax in axes]
    readouts = [ax.text(0.98, 1.06, '', transform=ax.transAxes, ha='right',
                        va='bottom', fontsize=13, color=FG) for ax in axes]
    axes[1].legend(loc='upper left', handlelength=1.5)
    axes[2].legend(loc='upper left', handlelength=1.5)
    axes[3].legend(loc='upper left', handlelength=1.5)
    phase_txt = fig.text(0.045, 0.94, '', fontsize=17, color=FG, va='top')
    conf_txt = fig.text(0.78, 0.94, '', fontsize=13, color=FG, va='top',
                        ha='right')
    clock_txt = fig.text(0.955, 0.94, '', fontsize=14, color=FG2, va='top',
                         ha='right')

    def _cur(t, ts, vs, fmt, scale=1.0):
        sel = ts <= t
        if not np.any(sel):
            return np.array([]), np.array([]), ''
        return ts[sel], vs[sel], fmt.format(vs[sel][-1] * scale)

    n_frames = int(t_end * fps) + 1
    cmd = ['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
           '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{width}x{height}',
           '-r', str(fps), '-i', '-',
           '-c:v', 'libvpx-vp9', '-pix_fmt', 'yuva420p', '-b:v', '0',
           '-crf', '28', '-row-mt', '1', out_path]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    for i in range(n_frames):
        t = i / fps
        x, y, s = _cur(t, d['t_err'], d['v_err'], '{:.2f} m')
        ln_err.set_data(x, y); readouts[0].set_text(s)
        x, y, s = _cur(t, d['t_m'], d['v_m'], '{:.3f} kg')
        ln_m.set_data(x, y); readouts[1].set_text(s)
        x, y, s = _cur(t, d['t_s'], d['v_s'], '{:.4f}')
        ln_s.set_data(x, y); readouts[2].set_text(s)
        x, y, s = _cur(t, d['t_j'], d['v_j'], '{:.4f}')
        # 画的是总惯量 J_dry+dJ(相对变化看得出来),读数同口径
        ln_j.set_data(x, J_DRY + y)
        readouts[3].set_text('' if not len(y) else f'{J_DRY + y[-1]:.4f}')
        _, _, sc = _cur(t, d['t_conf'], d['v_conf'], '{:.3f}')
        if sc:
            healthy = d['h_conf'][d['t_conf'] <= t][-1] >= 0.5
            conf_txt.set_text(
                ('无载荷置信度 ' if use_cn else 'empty confidence ')
                + sc + ('' if healthy else '  STALE'))
        else:
            conf_txt.set_text('')
        for c in cursors:
            c.set_xdata([t, t])
        cur_ph = ''
        for k, tv in sorted(ph.items(), key=lambda kv: kv[1]):
            if t >= tv:
                cur_ph = (PHASE_CN if use_cn else PHASE_EN).get(k, k)
        phase_txt.set_text(cur_ph)
        clock_txt.set_text(f't = {t:5.1f} s')

        fig.canvas.draw()
        proc.stdin.write(np.asarray(fig.canvas.buffer_rgba()).tobytes())
        if i % (fps * 10) == 0:
            print(f'  frame {i}/{n_frames}  (t={t:.0f}s)', flush=True)

    proc.stdin.close()
    if proc.wait() != 0:
        sys.exit('ffmpeg 编码失败')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stamp', default=mpf.STAMP5,
                    help='gviz_{nmpc,mhe}_<stamp>.log 的时间戳')
    ap.add_argument('--payload', type=float, default=None,
                    help='载荷真值 [kg]；默认从本轮启动日志自动读取')
    ap.add_argument('--ecc', type=float, default=None,
                    help='水平偏心距离 [m]；默认从本轮启动日志自动读取')
    ap.add_argument('--t-end', type=float, default=mpf.T_END)
    ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--lang', choices=('zh', 'en'), default='zh',
                    help='曲线轴标签与阶段标注的语言')
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--height', type=int, default=340)
    a = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    payload, eccentricity, cfg_source = resolve_run_config(
        a.stamp, a.payload, a.ecc)
    d = load_mission(a.stamp, a.t_end)
    print(f'阶段时刻: ' + ', '.join(f'{k}={v:.1f}s' for k, v in
                                 sorted(d['phases'].items(), key=lambda kv: kv[1])))
    print(f'墙钟→任务时间偏移 off={d["off"]:.3f}')
    print(f'本轮真值: payload={payload:.3f}kg, ecc={eccentricity:.3f}m '
          f'(来源: {cfg_source})')
    sfx = '' if a.lang == 'zh' else f'_{a.lang}'
    out = os.path.join(OUT, f'overlay_{a.stamp}{sfx}.webm')
    build(d, payload, eccentricity, a.t_end, a.fps, a.width, a.height,
          out, a.lang)

    meta = dict(stamp=a.stamp, lang=a.lang, off=d['off'], fps=a.fps, t_end=a.t_end,
                width=a.width, height=a.height, payload=payload,
                eccentricity=eccentricity, config_source=cfg_source,
                phases=d['phases'], overlay=out)
    with open(os.path.join(OUT, f'overlay_{a.stamp}{sfx}.meta.json'), 'w') as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f'\n已生成 {out}')
    print(f'      {OUT}/overlay_{a.stamp}{sfx}.meta.json')


if __name__ == '__main__':
    main()
