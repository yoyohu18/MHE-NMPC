#!/usr/bin/env python3
"""移植诊断的配对复现(2026-07-09):单次跑下两个留出点结论矛盾
(ecc=0.075 缩放θ4无效、ecc=0.05 缩放θ4反超M0),必须多次配对才能分辨
"θ4量级错配"vs"节奏过拟合"vs"纯单次噪声"。

三方 × 两留出点,每格补到 n=3(含之前 holdout/transplant 各已有的 1 次,
这里再各补 2 次)。交错跑(rep 外层、config 内层),减少时间漂移把某一方
系统性拉偏。结果连同已有的单次一起汇总算 mean±std。
"""
import sys

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/gripper')
import grip_cem_optimize as g  # noqa: E402

THETA_STAR = np.load(
    '/home/clear/ros2_ws_HJH/nmpc_test_results/theta_grip_cem.npy')
MASS = 0.225
ANCHOR = 0.30
theta_scaled = THETA_STAR.copy()
theta_scaled[4] = THETA_STAR[4] * (MASS / ANCHOR)

# (label, theta) 三方
ARMS = (
    ('M0', g.M0_THETA),
    ('star', THETA_STAR),
    ('scaled', theta_scaled),
)
ECCS = (0.075, 0.05)
N_EXTRA = 2   # 每格再补 2 次(已有 1 次 -> n=3)


def run_one(label, theta, ecc):
    m = g.run_sitl_once(theta, MASS, ecc)
    if m is None:
        m = g.run_sitl_once(theta, MASS, ecc)
    loss = g.loss_of(m) if m is not None else g.PENALTY
    ent = m['enter'] if m else float('nan')
    print(f'REP [{label}] ecc={ecc} -> loss={loss:.3f} enter={ent:.3f} '
          f'{"" if m else "FAILED"}', flush=True)
    return loss


def main():
    print(f'theta_scaled θ4 = {theta_scaled[4]:.4f}N (star '
          f'{THETA_STAR[4]:.4f} * {MASS}/{ANCHOR})', flush=True)
    for rep in range(N_EXTRA):
        for ecc in ECCS:
            for label, theta in ARMS:
                run_one(label, theta, ecc)
    print('=== 补充复现结束,连同已有单次一起在外部汇总 ===', flush=True)


if __name__ == '__main__':
    main()
