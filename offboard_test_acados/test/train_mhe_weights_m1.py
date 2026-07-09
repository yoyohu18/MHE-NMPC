#!/usr/bin/env python3
# M1 阶段一训练脚本:在闭环训练场里用无梯度优化(差分进化)搜参数化权重时间表,
# 对比三条基线:固定权重 / M0 二值规则 / 学出的时间表。目的只有一个:证明
# "权重时间表可学出超过手调规则的收益"(尤其是压过冲的同时保住入带速度)。
# 运行:
#   export ACADOS_SOURCE_DIR=/home/clear/acados
#   export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
#   python3 train_mhe_weights_m1.py [--quick]

import argparse
import os
import time

import numpy as np

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.mhe_solver_builder import ensure_mhe_ocp_solver  # noqa: E402
from offboard_test_acados.mhe_weight_learning import (  # noqa: E402
    DEFAULT_SCENARIOS, M0_THETA, evaluate)


def _report(name, mean_loss, results):
    print(f'--- {name}: loss={mean_loss:.3f} ---')
    for sc, r in zip(DEFAULT_SCENARIOS, results):
        print(f'  dm={sc.dm:+.1f} lag={sc.lag_sec}s: '
              f'enter={r["t_enter"]:.2f}s settle={r["t_settle"]:.2f}s '
              f'overshoot={r["overshoot"]:.3f}kg jitter={r["jitter"]:.4f} '
              f'z_peak={r["z_peak"]:.3f}m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true',
                    help='只跑基线,不优化(验证训练场)')
    ap.add_argument('--maxiter', type=int, default=12)
    args = ap.parse_args()

    solver = ensure_mhe_ocp_solver()

    t0 = time.time()
    loss_fixed, res_fixed = evaluate(solver, None)
    t_ep = (time.time() - t0) / len(DEFAULT_SCENARIOS)
    print(f'(每集耗时 ~{t_ep:.2f}s)')
    _report('固定权重(基线)', loss_fixed, res_fixed)

    loss_m0, res_m0 = evaluate(solver, M0_THETA)
    _report('M0 二值规则 theta=[-4,0,0,0]', loss_m0, res_m0)

    if args.quick:
        return

    # 差分进化:4 维、紧预算。theta 边界:
    #   [0] 事件前降权 log10 ∈ [-6, -0.5]
    #   [1] 事件后 R_vel 缩放 log10 ∈ [-3, 1]
    #   [2] 过渡延长帧数 ∈ [0, 20]
    #   [3] Q0 质量锚缩放 log10 ∈ [-2, 3]
    from scipy.optimize import differential_evolution
    # 第一轮 θ1 顶到上界 1.0,放宽到 2.0
    bounds = [(-6.0, -0.5), (-3.0, 2.0), (0.0, 20.0), (-2.0, 3.0)]
    n_eval = [0]

    def f(theta):
        n_eval[0] += 1
        loss, _ = evaluate(solver, theta)
        return loss

    t0 = time.time()
    out = differential_evolution(
        f, bounds, maxiter=args.maxiter, popsize=6, tol=1e-3,
        seed=7, polish=False, init='sobol', workers=1,
        x0=M0_THETA)  # 从 M0 规则出发
    print(f'\n优化完成: {n_eval[0]} 次评估, {time.time()-t0:.0f}s')

    theta_star = out.x
    loss_star, res_star = evaluate(solver, theta_star)
    print(f'theta* = [{", ".join(f"{v:.2f}" for v in theta_star)}]')
    _report('学出的时间表 theta*', loss_star, res_star)

    print('\n===== 汇总 =====')
    print(f'固定权重: {loss_fixed:.3f}')
    print(f'M0 规则 : {loss_m0:.3f}')
    print(f'学出 θ* : {loss_star:.3f} '
          f'(vs M0 {100*(loss_m0-loss_star)/loss_m0:+.1f}%)')

    # 泛化检查:留出工况(训练没见过的幅度/延迟)
    from offboard_test_acados.mhe_weight_learning import Scenario
    holdout = [Scenario(dm=-0.9, lag_sec=0.7, seed=11),
               Scenario(dm=+0.75, lag_sec=1.3, seed=12)]  # 可飞范围内插值
    for name, th in [('M0 规则', M0_THETA), ('θ*', theta_star)]:
        loss_h, res_h = evaluate(solver, th, scenarios=holdout)
        print(f'留出工况 {name}: loss={loss_h:.3f} '
              + ' | '.join(f'dm={sc.dm:+.2f}:enter={r["t_enter"]:.2f}s '
                           f'os={r["overshoot"]:.3f}'
                           for sc, r in zip(holdout, res_h)))

    np.save(os.path.join(os.path.dirname(__file__), 'theta_star_m1.npy'),
            theta_star)

    # 对比图:SITL 同款工况(dm=-0.5, lag=1.0)三方轨迹
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        sc = DEFAULT_SCENARIOS[0]
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for th, lab, c in [(None, 'fixed', 'tab:blue'),
                           (M0_THETA, 'M0 rule', 'tab:orange'),
                           (theta_star, 'learned θ*', 'tab:green')]:
            _, res = evaluate(solver, th, scenarios=[sc])
            r = res[0]['rec']
            axes[0].plot(r['t'], r['m_est'], color=c, label=lab)
            axes[1].plot(r['t'], r['z'], color=c, label=lab)
        axes[0].plot(r['t'], r['m_true'], 'r--', lw=1, label='true')
        axes[0].set_ylabel('m_est [kg]'); axes[0].legend(); axes[0].grid(True)
        axes[1].axhline(3.0, color='r', ls='--', lw=1)
        axes[1].set_ylabel('z [m]'); axes[1].set_xlabel('t [s]')
        axes[1].legend(); axes[1].grid(True)
        axes[0].set_title('M1 phase-1: learned weight schedule vs baselines '
                          f'(dm={sc.dm}, lag={sc.lag_sec}s)')
        out_png = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', '..',
            'nmpc_test_results', 'mhe_m1_learned_schedule.png'))
        fig.savefig(out_png, dpi=120, bbox_inches='tight')
        print(f'saved plot: {out_png}')
    except ImportError:
        pass


if __name__ == '__main__':
    main()
