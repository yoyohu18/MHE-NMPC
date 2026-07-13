#!/usr/bin/env python3
"""吊挂 CEM 学出的 θ* 留出泛化验证(2026-07-09)。

训练花名册是质量{0.15,0.30}kg × 偏心{0.05,0.10}m 的 2x2 网格(见
grip_cem_optimize.ROSTER)。这里用两个轴都不在网格上的插值点,复用
grip_cem_optimize 里已经调好的 run_sitl_once/loss_of,对比 M0 规则版 vs
学出的 θ*,判断收益是不是只在训练点上成立(过拟合花名册),还是在没见过
的工况上依然保持。

用法: python3 grip_cem_validate_holdout.py
"""
import sys

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/gripper')
import grip_cem_optimize as g  # noqa: E402

THETA_STAR = np.load(
    '/home/clear/ros2_ws_HJH/nmpc_test_results/theta_grip_cem.npy')
HOLDOUT = (
    (0.225, 0.075),  # 两个轴都插值:质量、偏心都是训练花名册没见过的值
    (0.225, 0.05),   # 只换质量轴,偏心沿用训练值,方便定位问题出在哪个轴
)


def run_one(label, theta, mass, ecc):
    m = g.run_sitl_once(theta, mass, ecc)
    if m is None:
        m = g.run_sitl_once(theta, mass, ecc)  # 重试一次,跟 eval_theta 一致
    loss = g.loss_of(m) if m is not None else g.PENALTY
    print(f'[{label}] mass={mass} ecc={ecc} theta5={theta[4]:.3f} '
          f'-> loss={loss:.3f} {m if m else "FAILED"}', flush=True)
    return loss


def main():
    print(f'θ* = {THETA_STAR}', flush=True)
    print(f'M0 = {g.M0_THETA}', flush=True)
    results = []
    for mass, ecc in HOLDOUT:
        l_m0 = run_one('M0', g.M0_THETA, mass, ecc)
        l_star = run_one('θ*', THETA_STAR, mass, ecc)
        results.append((mass, ecc, l_m0, l_star))
    print('\n=== 汇总 ===')
    for mass, ecc, l_m0, l_star in results:
        win = 'θ* 赢' if l_star < l_m0 else 'M0 赢'
        print(f'mass={mass} ecc={ecc}: M0={l_m0:.3f} θ*={l_star:.3f} '
              f'({win}, 差={l_m0-l_star:+.3f})')


if __name__ == '__main__':
    main()
