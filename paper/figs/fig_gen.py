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
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
from paper_style import apply_style, FONT, BLUE, ORANGE, AQUA, INK, INK2, GRID, SURF

HERE = os.path.dirname(os.path.abspath(__file__))
BUNDLED_RUN = os.path.abspath(os.path.join(HERE, '..', 'data'))
LEGACY_RUN = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'nmpc_test_results'))
RUN = os.environ.get('NMPC_RESULTS_DIR')
if not RUN:
    RUN = BUNDLED_RUN if os.path.isfile(
        os.path.join(BUNDLED_RUN, 'mainline_ab2_manifest.csv')) else LEGACY_RUN

COL1, COL2 = 3.45, 7.0   # IEEE 单/双栏宽 [in]
apply_style()


def _style(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True, axis='both', alpha=0.7, zorder=0)
    ax.set_axisbelow(True)


def save(fig, name):
    p = os.path.join(HERE, name)
    fig.savefig(p)
    if name.endswith('.pdf'):
        fig.savefig(p[:-4] + '.png', dpi=240)
    plt.close(fig)
    print(f'  wrote {name}')


# ========================= 图 1: 系统框图 =========================
def fig1():
    """System architecture with the information contract as the visual focal point."""
    # 2.02in tall packed title and body lines ~5.8pt apart under 6-7pt fonts, so they
    # overlapped; 2.4in (with the empty bottom margin cropped) keeps in-box row gaps near 8pt.
    fig, ax = plt.subplots(figsize=(COL2, 2.4), facecolor='white')
    ax.set_xlim(0, 100); ax.set_ylim(2.2, 40); ax.axis('off')

    NAVY = '#18324A'
    BLUE_D, BLUE_M, BLUE_L = BLUE, '#8FB5D5', '#EAF3FA'
    TEAL_D, TEAL_L = AQUA, '#EDF8F5'
    ORANGE_D, ORANGE_L = ORANGE, '#FFF3EA'
    RED_D = '#B74343'
    GREY, LINE, PALE = '#627487', '#C9D4DE', '#F8FAFC'
    # Tinos is a Times-metric *serif* face; the name is legacy. Fall back within the
    # serif family so the figure matches the manuscript text, not a sans default.
    SANS = [FONT, 'Nimbus Roman', 'Liberation Serif', 'DejaVu Serif']
    boxed_text = []

    def txt(x, y, s, size=7.0, color=NAVY, weight='normal', ha='left', va='center',
            style='normal', z=5):
        return ax.text(x, y, s, fontsize=size, color=color, fontweight=weight,
                       fontfamily=SANS, ha=ha, va=va, style=style, zorder=z)

    def arrow(p1, p2, color=NAVY, lw=1.1, rad=0.0, z=3):
        a = FancyArrowPatch(p1, p2, arrowstyle='-|>', mutation_scale=8.5,
                            connectionstyle=f'arc3,rad={rad}', color=color, lw=lw,
                            shrinkA=1.2, shrinkB=1.2, zorder=z)
        ax.add_patch(a)
        return a

    def block(x, y, w, h, title, lines, edge=BLUE_D, fill='white', title_size=7.0,
              strong=False):
        body = FancyBboxPatch((x, y), w, h,
                              boxstyle='round,pad=0.10,rounding_size=0.45',
                              fc=fill, ec=edge, lw=1.35 if strong else 0.9, zorder=2)
        ax.add_patch(body)
        ax.add_patch(plt.Rectangle((x, y), 0.65, h, fc=edge, ec='none', zorder=2.2))
        title_y = y + h - 1.9
        title_artist = txt(x + w / 2 + 0.25, title_y, title, title_size, edge,
                           'bold', ha='center')
        boxed_text.append((title_artist, body, 1.1))
        if lines:
            if len(lines) == 1:
                ys = [y + h * 0.32]
            elif len(lines) == 2:
                ys = [y + h * 0.46, y + h * 0.20]
            else:
                ys = np.linspace(y + h * 0.55, y + h * 0.18, len(lines))
            for yy, (value, size, color, weight) in zip(ys, lines):
                artist = txt(x + w / 2 + 0.25, yy, value, size, color, weight, ha='center')
                boxed_text.append((artist, body, 1.0))
        return body

    # Deployment boundary: present, but visually recessive.
    outer = FancyBboxPatch((3.2, 8.1), 74.3, 29.5,
                           boxstyle='round,pad=0.12,rounding_size=0.75',
                           fc=PALE, ec=BLUE_M, lw=0.8, ls=(0, (3.0, 2.4)), zorder=0)
    ax.add_patch(outer)
    txt(46.5, 36.45, 'ROS 2 OUTER LOOP', 6.0, BLUE_D, 'bold', ha='center',
        z=1).set_bbox(dict(fc=PALE, ec='none', pad=0.7))

    # Estimation side.
    recon = block(6.0, 11.6, 20.0, 6.2, 'ROTOR WRENCH',
                  [(r'$(T_{phys},\;\tau_{phys})\leftarrow\omega_i$', 6.0, GREY, 'normal')],
                  edge=TEAL_D, fill='white', title_size=6.35)
    mhe = block(6.0, 23.0, 20.0, 9.0, 'ONLINE MHE',
                [('continuous estimation', 5.9, GREY, 'normal'),
                 (r'estimate $m,\;s=m_P r_{xy}\;\rightarrow\;\Delta J$', 6.15, NAVY, 'normal')],
                edge=BLUE_D, fill='white', title_size=6.15)

    # The paper's contribution is the contract, so it is the only strongly filled block.
    frame = block(33.0, 22.0, 19.0, 11.0, 'ATOMIC FRAME',
                  [(r'$\mathcal{P}=\{m,s,\Delta J,\gamma,h,a\}$', 6.55, NAVY, 'bold'),
                   ('atomic timestamp', 5.75, GREY, 'normal')],
                  edge=BLUE_D, fill=BLUE_L, title_size=5.9, strong=True)

    # Two downstream consumers make the contract's role explicit.
    nmpc = block(59.0, 28.2, 15.5, 8.2, 'NMPC',
                 [(r'$p=[x_{ref},m,c_{xy},J]$', 5.65, NAVY, 'normal'),
                  (r'state $x\leftarrow$ odometry', 5.45, GREY, 'normal')],
                 edge=BLUE_D, fill='white', title_size=6.45)
    scheduler = block(59.0, 18.6, 15.5, 7.3, 'RATE SCHEDULER',
                      [(r'$\Delta J\;\rightarrow\;$ rate scale', 5.75, NAVY, 'normal')],
                      edge=BLUE_D, fill='white', title_size=5.85)

    # Safety semantics are connected to confidence/health/age rather than floating as a slogan.
    safety = FancyBboxPatch((33.0, 10.5), 41.5, 4.7,
                            boxstyle='round,pad=0.10,rounding_size=0.45',
                            fc='white', ec='#718096', lw=0.9, zorder=2)
    ax.add_patch(safety)
    ax.add_patch(plt.Rectangle((33.0, 10.5), 0.65, 4.7,
                               fc='#718096', ec='none', zorder=2.2))
    safety_title = txt(53.9, 13.65, 'UNRESOLVED', 6.15, NAVY, 'bold', ha='center')
    safety_detail = txt(53.9, 11.85,
                        r'no valid evidence $\;\rightarrow\;$ smooth brake-to-hover',
                        5.65, GREY, 'normal', ha='center')
    boxed_text.extend([(safety_title, safety, 1.0), (safety_detail, safety, 1.0)])

    # Execution lies outside ROS 2.
    px4 = block(82.0, 24.0, 15.0, 8.5, 'PX4 RATE LOOP',
                [('fixed gains · 1 kHz', 5.9, GREY, 'normal')],
                edge=ORANGE_D, fill=ORANGE_L, title_size=5.9)
    plant = block(82.0, 7.0, 15.0, 10.0, 'QUADROTOR',
                  [('rigid payload', 6.1, NAVY, 'bold'),
                   ('Gazebo · DART', 5.8, GREY, 'normal')],
                  edge=TEAL_D, fill=TEAL_L, title_size=6.8)

    # Physical estimates feed the MHE; the MHE publishes through a qualified atomic contract.
    arrow((16.0, 17.8), (16.0, 23.0), TEAL_D, 1.0)
    txt(18.0, 20.3, r'$T_{phys},\tau_{phys}$', 5.8, TEAL_D, style='italic')
    arrow((26.0, 27.5), (33.0, 27.5), BLUE_D, 1.35)
    txt(29.5, 29.0, 'qualified', 5.45, BLUE_D, 'bold', ha='center').set_bbox(
        dict(fc=PALE, ec='none', pad=0.25))

    # One frame, two consumers.
    arrow((52.0, 29.4), (59.0, 32.0), BLUE_D, 1.1)
    txt(58.2, 34.4, 'model update', 5.25, BLUE_D, ha='right', style='italic')
    arrow((52.0, 25.4), (59.0, 22.25), BLUE_D, 1.1)
    txt(55.2, 21.5, 'rate scale', 5.25, BLUE_D, ha='center', style='italic')
    arrow((42.5, 22.0), (42.5, 15.2), '#718096', 0.95)
    txt(44.0, 18.8, r'$\gamma,h,a$', 5.55, GREY, style='italic')

    # Commands converge at the inner loop, then actuate the physical vehicle.
    arrow((74.5, 32.2), (82.0, 29.8), ORANGE_D, 1.05)
    txt(79.9, 32.3, '$T$', 6.0, ORANGE_D, ha='center')
    arrow((74.5, 22.25), (82.0, 26.6), ORANGE_D, 1.05)
    txt(79.6, 22.4, r'$\omega^\star$', 6.0, ORANGE_D, ha='center')
    arrow((89.5, 24.0), (89.5, 17.0), ORANGE_D, 1.05)
    txt(91.0, 20.5, 'motor cmd', 5.6, ORANGE_D, style='italic')

    # Proprioceptive feedback closes the actual loop along a clear, unoccupied baseline.
    ax.plot([89.5, 89.5, 16.0], [7.0, 5.1, 5.1], color=TEAL_D, lw=0.9, zorder=1)
    arrow((16.0, 5.1), (16.0, 11.6), TEAL_D, 0.9, z=2)
    txt(51.5, 3.65, r'only motor speeds $\omega_i$ + odometry $x$', 6.0, TEAL_D,
        'bold', ha='center')

    # The forbidden input terminates before reaching the deployed graph.
    txt(6.2, 39.0, 'attach / drop notification', 5.9, GREY)
    ax.plot([23.9, 23.9], [38.7, 35.2], color=RED_D, lw=0.8,
            ls=(0, (2.2, 2.2)), zorder=2)
    ax.add_patch(Circle((23.9, 34.35), 0.78, fc='white', ec=RED_D, lw=1.0, zorder=4))
    ax.plot([23.40, 24.40], [33.85, 34.85], color=RED_D, lw=1.2, zorder=5)
    txt(25.3, 34.35, 'NOT CONSUMED', 5.85, RED_D, 'bold')

    # Generation-time guard: a future wording edit cannot silently escape a module again.
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    for artist, patch, margin_px in boxed_text:
        tb = artist.get_window_extent(renderer=renderer)
        pb = patch.get_window_extent(renderer=renderer)
        if (tb.x0 < pb.x0 + margin_px or tb.x1 > pb.x1 - margin_px or
                tb.y0 < pb.y0 + margin_px or tb.y1 > pb.y1 - margin_px):
            raise RuntimeError(f'Fig.1 text escaped module: {artist.get_text()!r}')

    png = os.path.join(HERE, 'fig1_architecture.png')
    fig.savefig(png, dpi=240, facecolor='white')
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
    """fixed vs self-triggered (no external signal), delta m = -0.3 kg, n = 6 each.

    Uses only the `nosignal` arm of the ablation: its detector is armed purely by
    the T_phys residual and consumes no attach/drop notification, matching the
    eventless claim of the paper. The `signal` arm stays a control in Table I.
    """
    import frozen_m0_log_parser as P
    DELTA, M_EMPTY = -0.3, 2.064
    P.M_TRUE, P.BAND = M_EMPTY + DELTA, 0.08

    runs = {'fixed': [], 'nosignal': []}
    man = os.path.join(RUN, 'nosignal_ablation_20260713_182612.txt')
    for line in open(man):
        if line.startswith('#'):
            continue
        cfg, d, _rep, mhe_stamp, _ns, status = line.split()
        if float(d) == DELTA and cfg in runs and status == 'ok':
            runs[cfg].append(P.parse(os.path.join(RUN, f'mhe_node_{mhe_stamp}.log')))
    for cfg, rs in runs.items():
        if len(rs) != 6:
            raise RuntimeError(f'Fig.3 expected 6 {cfg} runs, got {len(rs)}')

    grid = np.arange(-0.5, 3.5 + 1e-9, 0.02)

    def band(rs):
        """Per-run mass resampled on the common t_phys-aligned grid."""
        M = np.full((len(rs), grid.size), np.nan)
        for i, r in enumerate(rs):
            inside = (grid >= r['t'][0]) & (grid <= r['t'][-1])
            M[i, inside] = np.interp(grid[inside], r['t'], r['m'])
        return np.nanmedian(M, 0), np.nanmin(M, 0), np.nanmax(M, 0)

    fig, ax = plt.subplots(figsize=(COL1, 1.86))
    ax.axhspan(P.M_TRUE - P.BAND, P.M_TRUE + P.BAND, color=AQUA, alpha=0.15, zorder=0)
    ax.axhline(P.M_TRUE, color=AQUA, lw=0.8, ls=':', zorder=1)
    ax.text(3.3, P.M_TRUE + 0.014, 'truth $\\pm$0.08 kg band',
            fontsize=7.0, color='#0f7a55', ha='right')

    for cfg, c, lab in [('fixed', ORANGE, 'fixed weights'),
                        ('nosignal', BLUE, 'self-triggered (no signal)')]:
        med, lo, hi = band(runs[cfg])
        ax.fill_between(grid, lo, hi, color=c, alpha=0.18, lw=0, zorder=2)
        ax.plot(grid, med, color=c, lw=1.4, label=lab, zorder=3)

    ax.axvline(0, color=INK2, lw=0.8, ls='--', zorder=1)
    ax.text(0.06, 1.845, 't=0: event\nphysically effective', fontsize=7.0, color=INK2)
    ax.set_xlabel('time relative to physical effectiveness [s]')
    ax.set_ylabel('mass estimate [kg]')
    ax.set_xlim(-0.5, 3.5)
    ax.set_ylim(1.60, 2.13)
    ax.legend(loc='upper right', frameon=False)
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
