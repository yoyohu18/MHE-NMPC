#!/usr/bin/env python3
"""速度阶梯聚合(高机动诊断,2026-07-29)。

用法: aggregate_speed_ladder.py <manifest>

回答三个问题:
  1) 跟踪误差是否满足 e ≈ v·τ(固定执行时延)?  -> 过原点线性回归,看 R² 与 τ
  2) m_est 是否随速度系统性下降(质量吸收未建模机动误差)? -> m_est vs v
  3) NMPC 指令推力与物理推力(转速反算)的偏差是否随速度增大?
     -> 高 body-rate 下四电机差分的凸性效应 Σωᵢ² > 4ω̄²

判据提醒: 这些是 n=1/档的探边界数据,不是定案(SITL 单工况 n<8 不下结论)。
"""
import re
import sys
from pathlib import Path

import numpy as np

RUNDIR = Path('/home/clear/ros2_ws_HJH/nmpc_test_results')
M_FLOOR = 1.961   # MHE 质量下界 = 0.95*m_b

nmpc_pat = re.compile(
    r't=([\d.]+)s \| pos_err=([\d.]+)m \| T=([\d.-]+)N .*?'
    r'sqp_iter=(\d+) \| solve=([\d.]+)ms')
mhe_pat = re.compile(r'estimate: ([\d.]+) kg \(T_phys=([\d.]+)N\)')

rows = []
for line in open(sys.argv[1]):
    if line.startswith('#'):
        continue
    f = line.split()
    if len(f) < 6 or f[5] != 'ok':
        if len(f) >= 6:
            print(f'跳过 v={f[0]}: status={f[5]}')
        continue
    v, w, ramp, need, stamp = float(f[0]), float(f[1]), float(f[2]), float(f[3]), f[4]
    # 第 7 列 settle 是 07-30 加的(ramp 固定后新增的起振残差衰减余量)。
    # 旧 manifest 没有这列 -> 0.0,于是 t_ss 退回 2.0+ramp,**历史批次逐字节复现**。
    settle = float(f[6]) if len(f) >= 7 else 0.0

    ntxt = (RUNDIR / f'acados_nmpc_node_{stamp}.log').read_text(errors='replace')
    mtxt = (RUNDIR / f'mhe_node_{stamp}.log').read_text(errors='replace')

    A = np.array([[float(g) for g in m.groups()] for m in nmpc_pat.finditer(ntxt)])
    M = np.array([[float(g) for g in m.groups()] for m in mhe_pat.finditer(mtxt)])
    if len(A) == 0:
        print(f'跳过 v={v}: 无跟踪采样')
        continue

    # 只取 ramp 结束后的稳态段(hover_time=2.0 + ramp + settle),否则起振段污染统计。
    # settle 是必要的:ramp 出口虽然 C² 连续(5 次 smooth-step 在 s=1 处 alpha_dot 与
    # alpha_ddot 都归零,接缝上没有加速度台阶),但 ramp **中段**(s≈0.21 峰值处)的
    # 位置残差要按闭环时间常数衰减,紧贴 ramp 结束切窗口会把这段瞬态算进稳态统计。
    # ramp 长时这问题被自然掩盖,固定到 3.0s 后必须显式留余量。
    t_ss = 2.0 + ramp + settle
    sel = A[:, 0] > t_ss
    if sel.sum() < 3:
        sel = A[:, 0] > 0   # 兜底:采样太少就全用
    rows.append(dict(
        v=v, w=w, n=int(sel.sum()),
        pe_med=float(np.median(A[sel, 1])), pe_p95=float(np.percentile(A[sel, 1], 95)),
        T_cmd=float(np.median(A[sel, 2])),
        it_max=int(A[sel, 3].max()), sv_p95=float(np.percentile(A[sel, 4], 95)),
        m_med=float(np.median(M[:, 0])) if len(M) else np.nan,
        m_min=float(M[:, 0].min()) if len(M) else np.nan,
        T_phys=float(np.median(M[:, 1])) if len(M) else np.nan,
        failed=ntxt.count('solve failed')))

if not rows:
    print('没有可用数据')
    sys.exit(2)

print(f'\n{"v_peak":>7} {"n":>3} {"pos_err中位":>11} {"p95":>7} {"T_cmd":>7} '
      f'{"T_phys":>7} {"Δ推力":>7} {"m_est中位":>9} {"m_min":>7} {"裕度":>6} '
      f'{"iter":>5} {"solve":>6} {"fail":>5}')
print('-' * 112)
for r in rows:
    print(f'{r["v"]:7.2f} {r["n"]:3d} {r["pe_med"]:10.3f}m {r["pe_p95"]:6.3f}m '
          f'{r["T_cmd"]:6.2f}N {r["T_phys"]:6.2f}N {r["T_phys"]-r["T_cmd"]:+6.2f}N '
          f'{r["m_med"]:9.3f} {r["m_min"]:7.3f} {r["m_min"]-M_FLOOR:+6.3f} '
          f'{r["it_max"]:5d} {r["sv_p95"]:5.2f}ms {r["failed"]:5d}')

v = np.array([r['v'] for r in rows])
pe = np.array([r['pe_med'] for r in rows])

print('\n=== 问题1: e ≈ v·τ ? ===')
if len(v) >= 2:
    # 过原点: 最小化 Σ(pe - τ v)²  -> τ = Σ(v·pe)/Σv²
    tau0 = float(v @ pe / (v @ v))
    ss_res0 = float(((pe - tau0 * v) ** 2).sum())
    ss_tot = float(((pe - pe.mean()) ** 2).sum())
    r2_0 = 1 - ss_res0 / ss_tot if ss_tot > 0 else float('nan')
    # 带截距
    slope, icept = np.polyfit(v, pe, 1)
    pred = slope * v + icept
    r2_1 = 1 - ((pe - pred) ** 2).sum() / ss_tot if ss_tot > 0 else float('nan')
    print(f'过原点  : τ = {tau0*1000:.0f} ms,  R² = {r2_0:.4f}')
    print(f'带截距  : 斜率 {slope*1000:.0f} ms, 截距 {icept*1000:+.0f} mm, R² = {r2_1:.4f}')
    print(f'  逐点检验 (e_pred = τ·v, τ={tau0*1000:.0f}ms):')
    for r in rows:
        p = tau0 * r['v']
        print(f'    v={r["v"]:.2f}  实测 {r["pe_med"]:.3f}m  预测 {p:.3f}m  '
              f'残差 {r["pe_med"]-p:+.3f}m ({100*(r["pe_med"]/p-1) if p>0 else 0:+.0f}%)')
    print('  判读: R²>0.95 且截距≈0 -> 误差主要是固定执行时延×速度,')
    print('        不该先修质量估计器;截距显著>0 -> 另有与速度无关的偏差项。')

print('\n=== 问题2: m_est vs 速度 ===')
mm = np.array([r['m_med'] for r in rows])
print(f'm_est 中位跨档范围: {mm.min():.3f} ~ {mm.max():.3f} kg  '
      f'(极差 {1000*(mm.max()-mm.min()):.0f} g)')
print(f'最低触及: {min(r["m_min"] for r in rows):.3f} kg  '
      f'(下界 {M_FLOOR}, 裕度 {min(r["m_min"] for r in rows)-M_FLOOR:+.3f})')
if len(v) >= 3:
    print(f'corr(m_est中位, v) = {np.corrcoef(v, mm)[0,1]:+.3f}   '
          f'(强负相关 -> 质量在吸收机动误差)')
print('  真实质量恒为 2.0643kg(drop 关闭、无 wrench),偏离即估计偏差。')

print('\n=== 问题3: 推力两层一致性 ===')
d = np.array([r['T_phys'] - r['T_cmd'] for r in rows])
print(f'Δ推力(T_phys - T_cmd) 跨档: {d.min():+.2f} ~ {d.max():+.2f} N')
if len(v) >= 3:
    print(f'corr(Δ推力, v) = {np.corrcoef(v, d)[0,1]:+.3f}')
    print('  07-29 首批实测: 纯跟踪下 |Δ| ≤ 0.12N(两层一致,映射链无问题),')
    print('  相关性为负 —— "高 body-rate 电机差分凸性效应致 T_phys>T_cmd" 的')
    print('  猜想已被否证。drop 场景那 +1.9N 是 wrench 与 m_est 撞下界的交互,')
    print('  不是通用映射偏差,别把两者混为一谈。')
