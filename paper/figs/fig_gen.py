#!/usr/bin/env python3
"""C.3 论文图 1-5 生成(2026-07-27)。输出矢量 PDF 到 paper/figs/。
配色=dataviz 参考调色板已验证前三槽(blue/orange/aqua,固定顺序,CVD ΔE9.1≥8/常视19.6≥15)。
数据源:图3=冻结事件时间线解析器(031851事件 vs 162003固定);图4=冻结偏心扫格;
图5=grip B.5 全流程 attach-window 逐帧。图1/2=示意图(无数据)。
IEEE 风格:Times-compatible serif、最终版面不小于约 7pt、细线、recessive grid、
≥2 series 必有图例；PDF 使用可搜索/可复制的 TrueType(Type 42)字体，禁用 Type 3。"""
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
BUNDLED_RUN = os.path.abspath(os.path.join(HERE, '..', 'data'))
LEGACY_RUN = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'nmpc_test_results'))
RUN = os.environ.get('NMPC_RESULTS_DIR')
if not RUN:
    RUN = BUNDLED_RUN if os.path.isfile(
        os.path.join(BUNDLED_RUN, 'mainline_ab2_manifest.csv')) else LEGACY_RUN

# ---- dataviz 参考调色板(light,已验证) ----
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e6e6e3', '#fcfcfb'

plt.rcParams.update({
    # Nimbus Roman 与正文使用的 Times 风格一致；后备字体保证精简环境仍可复现。
    'font.family': 'serif',
    'font.serif': ['Nimbus Roman', 'Tinos', 'Times New Roman', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size': 8.5, 'axes.titlesize': 8.5, 'axes.labelsize': 8.5,
    'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'legend.fontsize': 7.5,
    'axes.edgecolor': INK2, 'axes.linewidth': 0.7,
    'xtick.color': INK2, 'ytick.color': INK2,
    'text.color': INK, 'axes.labelcolor': INK,
    'grid.color': GRID, 'grid.linewidth': 0.55,
    'lines.linewidth': 1.25,
    'pdf.fonttype': 42, 'ps.fonttype': 42,
    'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
})
COL1, COL2 = 3.45, 7.0   # IEEE 单/双栏宽 [in]


def _style(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True, axis='both', alpha=0.7, zorder=0)
    ax.set_axisbelow(True)


def save(fig, name):
    p = os.path.join(HERE, name)
    fig.savefig(p)
    plt.close(fig)
    print(f'  wrote {name}')


# ========================= 图 1: 系统框图 =========================
def fig1():
    fig, ax = plt.subplots(figsize=(COL2, 2.5))
    # 左侧留出 1.7 单位给泳道标签,标签放在画布外侧而不是压在带内——带内四个角
    # 都被功能框或数据流注记占着,任何"塞进带里"的摆法都会重叠。
    ax.set_xlim(-1.75, 10); ax.set_ylim(0, 6); ax.axis('off')

    def box(x, y, w, h, text, fc, ec):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc=fc, ec=ec, lw=1.0, zorder=2))
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=8.0, zorder=3)

    def arrow(x1, y1, x2, y2, label='', col=INK2, off=0.12):
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle='-|>',
                     mutation_scale=9, lw=1.0, color=col, zorder=1))
        if label:
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + off, label, ha='center', va='bottom',
                    fontsize=7.0, color=INK2, style='italic')

    # 三层背景带
    for y, lab, c in [(4.2, 'ROS 2 outer loop', '#eef4fb'),
                      (2.3, 'PX4 inner loop', '#fdf1ea'),
                      (0.4, 'Gazebo physics', '#eaf7f1')]:
        ax.add_patch(plt.Rectangle((0.1, y), 9.8, 1.55, fc=c, ec='none', zorder=0))
        ax.text(-0.05, y + 0.775, lab, fontsize=7.0, color=INK2, style='italic',
                va='center', ha='right')

    # 频率标注:每个环标出实际运行速率(实测见 Table VI)。审稿人和读者判断实时性
    # 可行性时,第一件想知道的就是"谁跑多快",放进架构图省一次来回。
    box(0.5, 4.35, 2.7, 1.1,
        'MHE  @10 Hz\n($m$, $s$, $J$, confidence;\nhealth/freshness)', '#fff', BLUE)
    box(6.8, 4.35, 2.6, 1.1, 'NMPC  @20 Hz\n(acados SQP-RTI)\nN=20, 1.0 s', '#fff', BLUE)
    box(3.7, 2.45, 2.6, 1.1, 'PX4 rate loop\n(fixed parameters)  @1 kHz', '#fff', ORANGE)
    box(3.7, 0.55, 2.6, 1.1, 'Gazebo x500\n+ gripper (DART)', '#fff', AQUA)

    arrow(3.2, 4.9, 6.8, 4.9, '/mhe/payload_estimate: $m,s,J,\\gamma$  @10 Hz', BLUE)
    arrow(8.1, 4.35, 5.6, 3.55, 'T, scaled $\\omega_{cmd}$  @50 Hz', ORANGE, 0.0)
    arrow(5.0, 2.45, 5.0, 1.65, 'motor cmd', ORANGE)
    arrow(3.7, 0.95, 1.15, 4.35, '', AQUA)   # Gazebo -> MHE, label placed clear below
    # 贴在 Gazebo→MHE 那条反馈箭头左侧(箭头在 y=2.55 处约 x=2.5),右对齐避让
    ax.text(2.02, 2.55, 'motor speeds\n+ odometry', ha='right', fontsize=7.0, color=AQUA,
            style='italic')
    ax.text(9.7, 0.75, 'sensing-minimal:\nonly motor speeds + odometry', ha='right',
            fontsize=7.0, color=AQUA, style='italic')
    save(fig, 'fig1_architecture.pdf')


# ==================== 图 2: 滑窗降权示意 ====================
def fig2():
    fig, ax = plt.subplots(figsize=(COL1, 2.0))
    N = 20; ev = 12  # 事件落在窗内第 12 stage
    stages = np.arange(N)
    w = np.where(stages < ev, 0.18, 1.0)   # 事件前降权(M0: 1e-4 级,示意用 0.18)
    colors = np.where(stages < ev, ORANGE, BLUE)
    ax.bar(stages, w, width=0.8, color=colors, edgecolor=SURF, linewidth=0.5, zorder=2)
    ax.axvline(ev - 0.5, color=INK, lw=1.2, ls='--', zorder=3)
    ax.text(ev - 0.5, 1.12, 'payload event\n(T$_{phys}$ > $\\alpha g m$)', ha='center',
            fontsize=7.0, color=INK)
    ax.text(ev / 2 - 0.5, 0.30, 'pre-event\nstages\ndeweighted', ha='center', fontsize=7.0,
            color=ORANGE)
    ax.text((ev + N) / 2 - 0.5, 0.5, 'post-event\nnominal', ha='center', va='center',
            fontsize=7.0, color='white')
    ax.set_xlabel('MHE window stage (oldest $\\rightarrow$ newest)')
    ax.set_ylabel('measurement weight')
    ax.set_ylim(0, 1.35); ax.set_xlim(-0.7, N - 0.3)
    ax.set_yticks([0, 0.5, 1.0])
    _style(ax)
    save(fig, 'fig2_window_deweight.pdf')


# ============= 图 3: 事件触发 vs 固定 m_est 时间线 =============
def fig3():
    import frozen_m0_log_parser as P
    P.M_TRUE, P.BAND = 1.564, 0.08
    fixed = P.parse(f'{RUN}/mhe_node_20260703_162003.log')
    event = P.parse(f'{RUN}/mhe_node_20260703_031851.log')
    fig, ax = plt.subplots(figsize=(COL1, 2.3))
    ax.axhspan(P.M_TRUE - P.BAND, P.M_TRUE + P.BAND, color=AQUA, alpha=0.15, zorder=0)
    ax.axhline(P.M_TRUE, color=AQUA, lw=0.8, ls=':', zorder=1)
    ax.text(ax.get_xlim()[1] if False else 3.2, P.M_TRUE + 0.012, 'truth $\\pm$0.08 kg band',
            fontsize=7.0, color='#0f7a55', ha='right')
    for r, c, lab in [(fixed, ORANGE, 'fixed weights'), (event, BLUE, 'event-triggered')]:
        m = (r['t'] >= -0.5)
        ax.plot(r['t'][m], r['m'][m], color=c, lw=1.4, label=lab, zorder=3)
    ax.axvline(0, color=INK2, lw=0.8, ls='--', zorder=1)
    ax.text(0.05, 1.30, 't=0: event\nphysically effective', fontsize=7.0, color=INK2)
    ax.set_xlabel('time relative to physical effectiveness [s]')
    ax.set_ylabel('mass estimate [kg]')
    ax.set_xlim(-0.5, 3.5); ax.set_ylim(1.28, 2.15)
    ax.legend(loc='lower right', frameon=False)
    _style(ax)
    save(fig, 'fig3_mest_timeline.pdf')


# ============= 图 4: c_xy est vs truth 偏心扫格 =============
def fig4():
    # Phase1 偏心扫格。数值必须从冻结汇总文件解析，不在绘图代码里
    # 再维护一份“真值”；这样 REPRODUCE.md 声明的数据链才是可执行的。
    src = f'{RUN}/cxy_ecc_sweep_20260714_143710.txt'
    pat = re.compile(
        r'^(\d+\.\d+)\s+ok(?:\([^)]*\))?.*?cy=([+-]?[\d.]+)\s+m\s*\|\s*'
        r'truth c=\[[^,]+,([+-]?[\d.]+)\]')
    rows = []
    with open(src) as f:
        for line in f:
            m = pat.search(line)
            if m:
                rows.append(tuple(map(float, m.groups())))
    if len(rows) != 4:
        raise RuntimeError(f'Fig.4 expected 4 frozen sweep rows, got {len(rows)} from {src}')
    ecc = [r[0] for r in rows]
    est = [r[1] for r in rows]
    truth = [r[2] for r in rows]
    fig, ax = plt.subplots(figsize=(COL1, 2.4))
    lim = [-0.019, 0.002]
    ax.plot(lim, lim, color=INK2, lw=0.8, ls='--', zorder=1)
    ax.text(-0.0035, -0.0060, 'y = x', fontsize=7.0, color=INK2, rotation=38)
    ax.scatter(truth, est, s=42, color=BLUE, edgecolor=SURF, lw=0.6, zorder=3)
    # 点标签统一放右下,唯独最密的 0.10m(左下角)放左上避让 y=x 线
    for e, t, s in zip(ecc, truth, est):
        dx, dy = (5, -8) if e != 0.10 else (-24, 6)
        ax.annotate(f'{e:.2f} m', (t, s), textcoords='offset points', xytext=(dx, dy),
                    fontsize=7.0, color=INK2)
    ax.set_xlabel('true CoM offset $c_y$ [m]')
    ax.set_ylabel('online estimate $\\hat{c}_y$ [m]')
    ax.set_xlim(lim); ax.set_ylim(lim)
    ax.set_aspect('equal')
    ax.text(0.05, 0.93, 'dominant component tracks truth < 5%', transform=ax.transAxes,
            fontsize=7.0, color=BLUE, va='top')
    _style(ax)
    save(fig, 'fig4_cxy_vs_truth.pdf')


# ===== 图 5: 完整 grasp -> carry -> release 周期(dense 全程 attach-window) =====
def fig5():
    # grip_nmpc_20260715_165245: attach-window 10Hz 稠密覆盖全程 6.7-116.5s,
    # 含 grasp(~7)/lift(~8.6)/loaded hover/DROP(85.4)/recovery——唯一带稠密 release
    # 暂态的 B.5 run(0720 的 figure-8 run drop 在窗外)。release 峰 pos_err 0.214m。
    pat = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m T=([\d.]+)N '
                     r'z=([\d.-]+) m_est=([\d.]+)')
    t, pe, z, me = [], [], [], []
    for line in open(f'{RUN}/grip_nmpc_20260715_165245.log'):
        m = pat.search(line)
        if m:
            t.append(float(m.group(1))); pe.append(float(m.group(2)))
            z.append(float(m.group(4))); me.append(float(m.group(5)))
    t, pe, z, me = map(np.array, (t, pe, z, me))
    keep = t <= 100
    t, pe, z, me = t[keep], pe[keep], z[keep], me[keep]
    fig, axs = plt.subplots(3, 1, figsize=(COL1, 3.7), sharex=True)
    axs[0].plot(t, pe, color=BLUE, lw=0.9); axs[0].set_ylabel('pos err [m]')
    axs[1].plot(t, me, color=ORANGE, lw=0.9); axs[1].set_ylabel('$\\hat{m}$ [kg]')
    for yv, lab in [(2.364, 'loaded 2.364'), (2.064, 'empty 2.064')]:
        axs[1].axhline(yv, color=AQUA, lw=0.6, ls=':')
        axs[1].text(2, yv, lab, fontsize=7.0, color='#0f7a55', va='bottom')
    axs[2].plot(t, z, color=AQUA, lw=0.9); axs[2].set_ylabel('height z [m]')
    axs[2].set_xlabel('time since NMPC handoff [s]')
    for x, lab in [(8.6, 'grasp/lift'), (85.4, 'release')]:
        for ax in axs:
            ax.axvline(x, color=INK2, lw=0.7, ls='--', alpha=0.6, zorder=1)
        axs[0].text(x + 1.2, axs[0].get_ylim()[1] * 0.80, lab, fontsize=7.0, color=INK2)
    axs[0].annotate('release transient\n0.21 m, ${\\sim}$2.5 s', xy=(85.4, 0.214),
                    xytext=(60, 0.17), fontsize=7.0, color=INK2,
                    arrowprops=dict(arrowstyle='->', lw=0.6, color=INK2))
    for ax in axs:
        _style(ax)
    fig.align_ylabels(axs)
    fig.subplots_adjust(hspace=0.13)
    save(fig, 'fig5_b5_fullflow.pdf')


if __name__ == '__main__':
    for fn in (fig1, fig2, fig3, fig4, fig5):
        try:
            fn()
        except Exception as e:
            print(f'  FAIL {fn.__name__}: {e}')
    print('done.')
