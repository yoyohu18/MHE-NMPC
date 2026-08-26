#!/usr/bin/env python3
"""成果演示视频 —— 数据动画层(叠在 Gazebo/RViz 录屏之上的底部条带)。

从一次 run_sitl_gripper_viz.sh 完整任务的 gviz_nmpc_/gviz_mhe_ 日志渲染
三联滚动曲线(位置误差 / MHE 质量估计 vs 真值 / 在线偏心 c_y vs 真值),
带阶段中文标注和当前时刻游标,输出**带 alpha 通道**的 webm,便于
compose_demo.sh 直接 overlay 到录屏画面底部。

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

# 深色底上的配色:主曲线亮蓝/亮绿,真值中性灰虚线
C_EST, C_TRUTH, C_ERR = '#5aa9ff', '#9be27a', '#ffd166'
FG, FG2 = '#f2f1ee', '#a9a7a2'
PANEL_RGBA = (0.06, 0.06, 0.08, 0.72)   # 半透明深色条带


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
    if not offsets:
        sys.exit(f'{nmpc}: 没解析到任何 NMPC 时间行')
    off = float(np.median(offsets))

    tm, mm, tc, cyv, cy_tr = [], [], [], [], []
    for line in open(mhe):
        ml = mpf.MASS_LN.search(line) or mpf.EV_LINE.search(line)
        if ml:
            tm.append(mpf._wall(ml) - off); mm.append(float(ml.group(3)))
            continue
        mc = mpf.CXY.search(line)
        if mc:
            tc.append(mpf._wall(mc) - off); cyv.append(float(mc.group(4)))
            cy_tr.append(float(mc.group(6)) if mc.group(6) else np.nan)

    def _clip_sort(t, *ys):
        t = np.asarray(t, float)
        keep = (t >= 0) & (t <= t_end)
        t = t[keep]; ys = [np.asarray(y, float)[keep] for y in ys]
        i = np.argsort(t)
        return (t[i],) + tuple(y[i] for y in ys)

    t_err, v_err = _clip_sort(np.concatenate([tp, tw]),
                              np.concatenate([ep, ew]))
    t_m, v_m = _clip_sort(tm, mm)
    t_c, v_c, v_ct = _clip_sort(tc, cyv, cy_tr)
    # c_xy 只在挂载期间有物理意义(drop 后发布端冻结的 EMA 是残留,画了误导)
    keep_c = t_c <= phases.get('DROP', t_end)
    t_c, v_c, v_ct = t_c[keep_c], v_c[keep_c], v_ct[keep_c]

    return dict(off=off, phases=phases, t_err=t_err, v_err=v_err,
                t_m=t_m, v_m=v_m, t_c=t_c, v_c=v_c, v_ct=v_ct,
                r_att=r_att[0])


PHASE_CN = {'ATTACH': '接近并抓取载荷', 'LIFT': '抬升(有效质量阶跃)',
            'DYNAMIC': '8 字动态轨迹跟踪', 'DROP': '投放载荷(突卸扰动)'}
PHASE_EN = {'ATTACH': 'approach & grasp payload', 'LIFT': 'lift (effective mass step)',
            'DYNAMIC': 'figure-8 trajectory tracking', 'DROP': 'release (sudden unloading)'}


def build(d, payload, t_end, fps, width, height, out_path, lang='zh'):
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
    axes = [fig.add_axes([0.045 + i * 0.322, 0.20, 0.245, 0.60])
            for i in range(3)]
    for ax in axes:
        ax.set_facecolor((0, 0, 0, 0))
        ax.set_xlim(0, t_end)
        ax.set_xlabel('任务时间 [s]' if use_cn else 'mission time [s]')

    ph = d['phases']
    t_att = ph.get('ATTACH', 7.0)
    t_drop = ph.get('DROP', t_end)

    # 1) 位置误差
    axes[0].set_ylabel('位置误差 [m]' if use_cn else 'position error [m]')
    axes[0].set_ylim(0, max(0.35, float(np.nanmax(d['v_err'])) * 1.1))
    # 2) 质量:真值阶跃 + MHE 估计
    axes[1].set_ylabel('质量 [kg]' if use_cn else 'mass [kg]')
    axes[1].set_ylim(M_DRY - 0.25, M_DRY + payload + 0.25)
    axes[1].plot([0, t_att, t_att, t_drop, t_drop, t_end],
                 [M_DRY, M_DRY, M_DRY + payload, M_DRY + payload, M_DRY, M_DRY],
                 color=C_TRUTH, lw=1.4, ls='--',
                 label='真值' if use_cn else 'truth')
    # 3) 偏心 c_y
    axes[2].set_ylabel('质心偏心 $c_y$ [cm]' if use_cn else 'CoM offset $c_y$ [cm]')
    if len(d['t_c']):
        # ⚠️2026-08-21:**08-26 之前的** MHE 日志里那个 `truth c=` 只有几何因子是
        # 真值,质量因子在节点没收到 grip_true_payload_mass 时会退回 m_est 反推的
        # m_p,于是"真值"会跟着 m_est 一起抖(实测印出 m_p=0.263/0.218/0.256,真值
        # 明明是 0.300)。08-26 去先验改造删掉了 grip_true_payload_mass,质量因子
        # 改读 eval_true_payload_mass(纯评估真值),新日志里 `truth c=` 就是真值。
        # 这里仍用真实载荷质量 × attach 几何真值自己算,对新日志是恒等变换,
        # 对旧日志则是必要的修正。
        v_ct = d['v_ct']
        if d.get('r_att') is not None:
            v_ct = np.full_like(d['t_c'],
                                payload / (M_DRY + payload) * d['r_att'][1])
        axes[2].plot(d['t_c'], 1e2 * v_ct, color=C_TRUTH, lw=1.4, ls='--',
                     label='真值' if use_cn else 'truth')
        lo = np.nanmin(1e2 * np.concatenate([d['v_c'], v_ct]))
        hi = np.nanmax(1e2 * np.concatenate([d['v_c'], v_ct]))
        pad = max(1.0, 0.2 * (hi - lo))
        axes[2].set_ylim(lo - pad, hi + pad)

    # 阶段竖线(全程静态,避免逐帧闪烁)
    for ax in axes:
        for k, tv in ph.items():
            ax.axvline(tv, color=FG2, lw=0.7, ls=':', alpha=0.7)

    # 动态元素
    ln_err, = axes[0].plot([], [], color=C_ERR, lw=1.8)
    ln_m, = axes[1].plot([], [], color=C_EST, lw=1.8,
                         label='MHE 估计' if use_cn else 'MHE estimate')
    ln_c, = axes[2].plot([], [], color=C_EST, lw=1.8,
                         label='在线估计' if use_cn else 'online estimate')
    cursors = [ax.axvline(0, color=FG, lw=1.0, alpha=0.85) for ax in axes]
    readouts = [ax.text(0.98, 1.06, '', transform=ax.transAxes, ha='right',
                        va='bottom', fontsize=13, color=FG) for ax in axes]
    axes[1].legend(loc='upper left', handlelength=1.5)
    axes[2].legend(loc='upper left', handlelength=1.5)
    phase_txt = fig.text(0.045, 0.94, '', fontsize=17, color=FG, va='top')
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
        x, y, s = _cur(t, d['t_c'], d['v_c'], '{:.1f} cm', 1e2)
        ln_c.set_data(x, 1e2 * y); readouts[2].set_text(s)
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
    ap.add_argument('--payload', type=float, default=0.3, help='载荷真值 [kg]')
    ap.add_argument('--t-end', type=float, default=mpf.T_END)
    ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--lang', choices=('zh', 'en'), default='zh',
                    help='曲线轴标签与阶段标注的语言')
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--height', type=int, default=340)
    a = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    d = load_mission(a.stamp, a.t_end)
    print(f'阶段时刻: ' + ', '.join(f'{k}={v:.1f}s' for k, v in
                                 sorted(d['phases'].items(), key=lambda kv: kv[1])))
    print(f'墙钟→任务时间偏移 off={d["off"]:.3f}')
    sfx = '' if a.lang == 'zh' else f'_{a.lang}'
    out = os.path.join(OUT, f'overlay_{a.stamp}{sfx}.webm')
    build(d, a.payload, a.t_end, a.fps, a.width, a.height, out, a.lang)

    meta = dict(stamp=a.stamp, lang=a.lang, off=d['off'], fps=a.fps, t_end=a.t_end,
                width=a.width, height=a.height, payload=a.payload,
                phases=d['phases'], overlay=out)
    with open(os.path.join(OUT, f'overlay_{a.stamp}{sfx}.meta.json'), 'w') as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f'\n已生成 {out}')
    print(f'      {OUT}/overlay_{a.stamp}{sfx}.meta.json')


if __name__ == '__main__':
    main()
