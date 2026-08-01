#!/usr/bin/env python3
"""B.5 drop 几何释放修复验收(2026-07-15)。

背景:drop 后 MHE 仍按"幽灵载荷"(棘轮卡在带载值 + attach_offset 未清)给自己的
om_dot 算 dJ/c_xy,质量估计被系统性拖低(实测 m_est -4.1%、z 下沉 5.8cm;同一轮
带载段却只 -0.4%,这个不对称是它的指纹)。定性:**drop 状态同步缺失的模型状态机
bug**,不是 figure8 机动下的固有质量估计偏置。修法:drop 事件释放几何
(_payload_attached=False + 棘轮归零 + attach_offset=None),**不硬重置质量状态**
——让 MHE 从当前 m_est 连续跑、自行收敛回空机质量。

每轮记录:drop 时刻 / drop 前 2s 平均 m_est / drop 后最低 m_est / 回到 2.064±0.03
的时间 / drop 后 5-10s 的 z 稳态误差 / dJ,c_xy 是否在 drop 后一个 MHE 周期内归零 /
MHE solve failure 数。通过标准:3/3 轮几何立即归零、m_est 回到空机带内、原 5-8cm
持续高度偏差消失,且带载段质量误差与轨迹性能不退化。

用法: python3 verify_drop_geom_release.py <gviz_nmpc_日志...>(每轮一个)
"""
import re
import sys

import numpy as np

M_EMPTY, Z_REF, BAND = 2.064, 2.5, 0.03
AW = re.compile(r'\[attach-window\] t=([\d.]+)s pos_err=([\d.]+)m T=([\d.]+)N '
                r'z=(-?[\d.]+) m_est=([\d.]+)')
DROP = re.compile(r't=([\d.]+)s \| DROP: released')
# DROP 行的 ROS 墙钟(与 mhe 日志跨文件对齐用)
DROP_WALL = re.compile(r'\[(\d+)\.(\d+)\].*DROP: released')
# ⚠️ [geom] 由 mhe_node 打,在 grip_mhe_*.log 里(不在 nmpc 日志)——首版脚本在
# nmpc 日志里找导致误判"几何未归零",实为脚本 bug(2026-07-15)。
# c_xy 打印带正负号(c_xy=[+0.0000,+0.0000]),字符类必须含 '+'
GEOM = re.compile(r'\[(\d+)\.(\d+)\].*\[geom\] payload_attached=(\w+) -> '
                  r'dJ=([\d.]+) c_xy=\[([-+\d.]+),([-+\d.]+)\]')


def analyze(nmpc_log):
    txt = open(nmpc_log, errors='ignore').read()
    mhe_log = nmpc_log.replace('gviz_nmpc', 'gviz_mhe').replace(
        'grip_nmpc', 'grip_mhe')
    rows = np.array([[float(g) for g in m.groups()]
                     for m in AW.finditer(txt)])
    d = DROP.search(txt)
    if not len(rows) or not d:
        return None
    t, pe, T, z, me = rows.T
    t_drop = float(d.group(1))
    m_load_true = None

    # drop 前 2s 平均 m_est(带载段)
    pre = (t >= t_drop - 2.0) & (t < t_drop)
    m_pre = float(me[pre].mean()) if pre.any() else float('nan')

    # drop 后最低 m_est
    post = t > t_drop
    m_min = float(me[post].min()) if post.any() else float('nan')
    t_min = float(t[post][int(np.argmin(me[post]))] - t_drop) if post.any() else float('nan')

    # 回到 2.064±0.03 并保持的时间
    inb = post & (np.abs(me - M_EMPTY) <= BAND)
    t_back = float('nan')
    if inb.any():
        # 找第一个之后再没出过带的时刻
        idx = np.where(post)[0]
        for i in idx:
            if abs(me[i] - M_EMPTY) <= BAND and np.all(
                    np.abs(me[i:] - M_EMPTY) <= BAND):
                t_back = float(t[i] - t_drop); break

    # drop 后 5-10s 的 z 稳态误差
    win = (t >= t_drop + 5.0) & (t <= t_drop + 10.0)
    z_err = float((z[win] - Z_REF).mean()) if win.any() else float('nan')
    # 带载段(drop 前 2s)z 误差,作对照(不该退化)
    z_pre = float((z[pre] - Z_REF).mean()) if pre.any() else float('nan')

    # 几何归零:[geom] 翻转日志在 **mhe 日志**里;用 ROS 墙钟与 nmpc 的 DROP 行
    # 跨文件对齐,算"drop → 几何归零"的延迟(验收要求 ≤ 一个 MHE 周期 100ms)。
    geom_zero_dt, geom_ok = float('nan'), False
    dw = DROP_WALL.search(txt)
    try:
        mtxt_g = open(mhe_log, errors='ignore').read()
    except FileNotFoundError:
        mtxt_g = ''
    if dw and mtxt_g:
        t_drop_wall = float(dw.group(1)) + float(dw.group(2)) * 1e-9
        for m in GEOM.finditer(mtxt_g):
            if m.group(3) == 'False':
                w = float(m.group(1)) + float(m.group(2)) * 1e-9
                if w < t_drop_wall - 1.0:
                    continue  # drop 之前的翻转(起飞前初始态),跳过
                dJ, cx, cy = (float(m.group(4)), float(m.group(5)),
                              float(m.group(6)))
                geom_ok = (dJ == 0.0 and cx == 0.0 and cy == 0.0)
                geom_zero_dt = w - t_drop_wall
                break

    # MHE solve failure
    try:
        mtxt = open(mhe_log, errors='ignore').read()
        n_fail = mtxt.count('MHE solve failed')
    except FileNotFoundError:
        n_fail = -1
    return dict(t_drop=t_drop, m_pre=m_pre, m_min=m_min, t_min=t_min,
                t_back=t_back, z_err=z_err, z_pre=z_pre,
                geom_zero_dt=geom_zero_dt, geom_ok=geom_ok, n_fail=n_fail)


def main():
    logs = sys.argv[1:]
    if not logs:
        print(__doc__); return
    res = []
    for lg in logs:
        r = analyze(lg)
        if r is None:
            print(f'{lg}: 解析失败(无 drop / 无逐帧)'); continue
        res.append(r)
        print(f'\n=== {lg.split("/")[-1]} ===')
        print(f'  drop 时刻          : t={r["t_drop"]:.2f}s')
        print(f'  drop 前2s m_est    : {r["m_pre"]:.3f} kg  (带载真值 2.364, 误差 {r["m_pre"]-2.364:+.3f})')
        print(f'  drop 后最低 m_est  : {r["m_min"]:.3f} kg @ +{r["t_min"]:.2f}s')
        print(f'  回到 2.064±0.03    : {r["t_back"]:.2f}s' if np.isfinite(r["t_back"])
              else '  回到 2.064±0.03    : 未回到 ✗')
        print(f'  drop后5-10s z 误差 : {r["z_err"]*100:+.1f} cm   (带载段 {r["z_pre"]*100:+.1f} cm)')
        print(f'  几何归零           : {"✓" if r["geom_ok"] else "✗"} dJ=c_xy=0, '
              f'距 drop {r["geom_zero_dt"]*1000:.0f} ms (MHE周期 100ms)')
        print(f'  MHE solve failure  : {r["n_fail"]}')
    if len(res) >= 2:
        print('\n===== 汇总 =====')
        zb = np.array([r['z_err'] for r in res]) * 100
        mb = np.array([r['t_back'] for r in res])
        ok = sum(1 for r in res if r['geom_ok'] and np.isfinite(r['t_back'])
                 and abs(r['z_err']) < 0.03)
        print(f'  z 稳态误差: {zb.mean():+.1f}±{zb.std():.1f} cm (n={len(res)}) '
              f'[修前基线 -5.8cm]')
        print(f'  m_est 回带时间: {np.nanmean(mb):.2f}s')
        print(f'  通过(几何归零+回带+|z|<3cm): {ok}/{len(res)}')


if __name__ == '__main__':
    main()
