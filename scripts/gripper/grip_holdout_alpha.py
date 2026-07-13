#!/usr/bin/env python3
"""α版 θ* 的留出泛化验证(2026-07-10)。

用无量纲重参数化后重跑 CEM 学出的 α版 θ*(theta_grip_cem_alpha.npy),在两个
留出点(mass=0.225kg,两个轴都不在训练网格上)对比规则版 M0。判断:归一化 θ4
后,学出的调度在没见过的中间质量上能否稳定不输 M0(用户步骤5:能则可写论文)。

关键:两臂都走 run_sitl_once 的 α 语义(实际阈值=α·9.81·mass),M0 基线用
α=0.8/(9.81·0.225)=0.362——在 0.225kg 下精确复现规则版的固定 0.8N,诚实对照
(holdout 质量固定,固定 N 和固定 α 等价)。n=3 配对,交错跑减时间漂移。
"""
import sys

import numpy as np

sys.path.insert(0, '/home/clear/ros2_ws_HJH/src/scripts/gripper')
import grip_cem_optimize as g  # noqa: E402

MASS = 0.225
ECCS = (0.075, 0.05)
N_REP = 3

THETA_ALPHA = np.load(
    '/home/clear/ros2_ws_HJH/nmpc_test_results/theta_grip_cem_alpha.npy')
# M0 规则版:固定 0.8N。在 α 框架下、mass=0.225 时等价 α=0.8/(9.81*0.225)。
M0_ALPHA = np.array([-4.0, 0.0, 0.0, 0.0, 0.8 / (g.GRAVITY * MASS)])

ARMS = (('M0', M0_ALPHA), ('alpha*', THETA_ALPHA))


def run_one(label, theta, ecc):
    m = g.run_sitl_once(theta, MASS, ecc)
    if m is None:
        m = g.run_sitl_once(theta, MASS, ecc)
    loss = g.loss_of(m) if m is not None else g.PENALTY
    ent = m['enter'] if m else float('nan')
    thr = theta[4] * g.GRAVITY * MASS
    print(f'HOLD [{label}] ecc={ecc} thr={thr:.3f}N -> loss={loss:.3f} '
          f'enter={ent:.3f} {"" if m else "FAILED"}', flush=True)
    return loss


def main():
    print(f'α* = {THETA_ALPHA} (thr@{MASS}kg = '
          f'{THETA_ALPHA[4]*g.GRAVITY*MASS:.3f}N)', flush=True)
    print(f'M0 = {M0_ALPHA} (thr@{MASS}kg = '
          f'{M0_ALPHA[4]*g.GRAVITY*MASS:.3f}N = 固定0.8N规则版)', flush=True)
    for rep in range(N_REP):
        for ecc in ECCS:
            for label, theta in ARMS:
                run_one(label, theta, ecc)
    print('=== holdout 结束,外部汇总 ===', flush=True)


if __name__ == '__main__':
    main()
