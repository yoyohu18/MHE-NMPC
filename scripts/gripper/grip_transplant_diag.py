#!/usr/bin/env python3
"""移植诊断(2026-07-09):区分留出点上 θ* 输给 M0 是"θ4 单力量纲没归一化"
(一阶量级错配)还是"θ0/θ1/θ3 节奏组合过拟合两个训练点"(多自由度)。

做法:只把 θ* 的 θ4(唯一力量纲维,直接跟 |T_phys-baseline|≈ΔT=g·m_p 比较)
按静态 ΔT 比例缩放到留出质量(0.30kg 为锚:θ4' = 1.4847×0.225/0.30 ≈ 1.114N),
其余四维(节奏:θ0/θ1 无量纲权重缩放、θ2 时间帧、θ3 无量纲质量锚)原样不动,
只重跑两个留出点。判决:
  - 缩放版追平/反超 M0 → 纯 θ4 量级错配,节奏四维没动就够 → 处方=无量纲
    重参数化(阈值搜 α=θ/ΔT_static),原两点花名册重跑 CEM。
  - 缩放后仍输 → 时间结构本身也随质量变 → 才需加密花名册 + θ 条件化。

M0 / 原 θ* 的留出点单次结果本会话已有(holdout_validate),这里只补缩放 θ*。
"""
import sys

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/gripper')
import grip_cem_optimize as g  # noqa: E402

THETA_STAR = np.load(
    '/home/clear/ros2_ws_HJH/nmpc_test_results/theta_grip_cem.npy')
HOLDOUT_MASS = 0.225
ANCHOR_MASS = 0.30          # θ4 缩放的参考锚(留出质量更靠近它)
HOLDOUT_ECCS = (0.075, 0.05)

# 只缩放 θ4,其余四维原样。θ4' = θ4 * (holdout_mass / anchor_mass)。
theta_scaled = THETA_STAR.copy()
theta_scaled[4] = THETA_STAR[4] * (HOLDOUT_MASS / ANCHOR_MASS)


def run_one(label, theta, ecc):
    m = g.run_sitl_once(theta, HOLDOUT_MASS, ecc)
    if m is None:
        m = g.run_sitl_once(theta, HOLDOUT_MASS, ecc)
    loss = g.loss_of(m) if m is not None else g.PENALTY
    print(f'[{label}] mass={HOLDOUT_MASS} ecc={ecc} theta4={theta[4]:.3f} '
          f'-> loss={loss:.3f} {m if m else "FAILED"}', flush=True)
    return loss


def main():
    print(f'θ*          = {THETA_STAR}', flush=True)
    print(f'θ*_scaled   = {theta_scaled}  (只 θ4: '
          f'{THETA_STAR[4]:.3f} -> {theta_scaled[4]:.3f} N)', flush=True)
    for ecc in HOLDOUT_ECCS:
        run_one('θ*_scaled', theta_scaled, ecc)


if __name__ == '__main__':
    main()
