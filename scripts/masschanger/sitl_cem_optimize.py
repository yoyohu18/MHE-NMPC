#!/usr/bin/env python3
"""通宵 SITL 在环权重时间表优化(M1 岔路①,2026-07-06)。

背景:廉价训练场学出的 θ* 两次在 SITL 判定实验中不可迁移(细粒度排序是训练场
伪影,见 技术方案_MHE学习模块_20260703.md M1 节)——所以直接在真 SITL 闭环里优化。

方法:交叉熵法(CEM,纯 numpy 无新依赖),5 维 θ = 4 维权重时间表(语义见
mhe_weight_learning.ParametricWeightSchedule)+ 1 维事件确认阈值 [N](2026-07-07
质量板块升级:固定 1.5N 对 ≤0.15kg 阶跃是盲区)。每候选评 |DELTAS|=4 轮 SITL
(±0.15 小幅度 + ±0.8 大幅度,同一花名册 = common random numbers),按平均 loss
排序,精英重拟合高斯。第 0 代注入 M0 规则与 θ*₂ 作参照(阈值维=现行 1.5)。
每轮 SITL ≈2.5-3min → 每代 6 候选×4 轮 ≈ 70min → 通宵 9h ≈ 7-8 代。
留出验证(训练后手动):±0.3 / ±0.5 / +1.5 上比对 θ* vs M0。

鲁棒性:逐轮有效性判定(drop 未生效/解析失败 → 重试一次,再失败记罚分);
teardown 走 SIGTERM(kill -9 会留 DDS 尸体,2026-07-03 教训);全程 JSONL 落盘
可断点恢复检查;墙钟死线到点收尾并写出 best_theta。

用法: python3 sitl_cem_optimize.py [--hours 9] [--gens 14] [--pop 6]
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

WS = '/home/clear/ros2_ws_HJH'
RESULTS = f'{WS}/nmpc_test_results'
LAUNCHER = f'{WS}/src/scripts/masschanger/run_sitl_headless.sh'
sys.path.insert(0, f'{WS}/src/scripts/masschanger')
import parse_m0_logs as est      # noqa: E402  (M_TRUE/BAND 运行时 setattr)
import parse_dropwindow_logs as ctl  # noqa: E402

M_BASE = 2.064
# θ 第 5 维 = 事件确认阈值 event_confirm_thresh_n [N](2026-07-07 加入)。
# 动机:固定 1.5N 对轻载荷是盲的——0.15kg 阶跃 ΔT=1.47N 恰在阈值之下,
# 触发永不确认;阈值过低又会在真实质量变化物理生效前就被噪声确认,去权
# 时机错位。让 CEM 在小/大幅度混合花名册上自己权衡。
BOUNDS = np.array([[-6.0, -0.5], [-3.0, 2.0], [0.0, 20.0], [-2.0, 3.0],
                   [0.2, 3.0]])
M0_THETA = np.array([-4.0, 0.0, 0.0, 0.0, 1.5])   # 第 5 维=现行规则值
TH2 = np.array([-5.9829, 1.9870, 3.7753, -1.6466, 1.5])
# 评估花名册:小幅度(现行阈值盲区)+ 大幅度(饱和风险)双方向,所有候选
# 用同一花名册(common random numbers,工况差异不淹没 θ 差异)。
# 留出验证集(训练后手动跑):±0.3 / ±0.5 / +1.5。
DELTAS = (-0.15, +0.15, -0.8, +0.8)
PENALTY = 20.0   # 单工况失效(重试仍失败)的罚分 loss
SIGMA0 = np.array([1.5, 1.2, 5.0, 1.2, 0.6])   # 初始搜索宽度(逐维下限基准)


def teardown():
    for pat, sig in [
            ('offboard_test_acados/lib|parameter_bridge|gcs_heartbeat', '-TERM'),
            ('bin/px4|gz sim|mavros', '-TERM')]:
        subprocess.run(
            f"ps -eo pid,cmd | grep -E '{pat}' | grep -v grep | "
            f"awk '{{print $1}}' | xargs -r kill {sig} 2>/dev/null",
            shell=True)
        time.sleep(2)
    time.sleep(2)
    subprocess.run(
        "ps -eo pid,cmd | grep -E 'bin/px4|gz sim|mavros|offboard_test_acados/"
        "lib|gcs_heartbeat|parameter_bridge' | grep -v grep | awk '{print $1}'"
        " | xargs -r kill -9 2>/dev/null", shell=True)
    time.sleep(2)


def run_sitl_once(theta, delta):
    """一轮无头 SITL,返回指标 dict 或 None(失效)。"""
    t0 = time.time()
    env = dict(os.environ,
               MASS_CHANGER_DELTA_KG=f'{delta}',
               MHE_EVENT_TRIGGER='true',
               MHE_SCHEDULE_THETA='[' + ','.join(
                   f'{v:.4f}' for v in theta[:4]) + ']',
               MHE_CONFIRM_THRESH=f'{theta[4]:.4f}')
    subprocess.run([LAUNCHER], env=env, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    # 等新的非空 mhe 日志
    mhe = None
    for _ in range(180):
        cand = subprocess.run(
            f'ls -t {RESULTS}/mhe_node_*.log | head -1',
            shell=True, capture_output=True, text=True).stdout.strip()
        if cand and os.path.getmtime(cand) >= t0 and os.path.getsize(cand) > 100:
            mhe = cand
            break
        time.sleep(1)
    if mhe is None:
        teardown()
        return None
    # 等 drop 事件
    ok = False
    for _ in range(240):
        if 'mass event' in open(mhe, errors='ignore').read():
            ok = True
            break
        time.sleep(1)
    if not ok:
        teardown()
        return None
    time.sleep(20)   # 日志窗口收尾
    nmpc = subprocess.run(
        f'ls -t {RESULTS}/acados_nmpc_node_*.log | head -1',
        shell=True, capture_output=True, text=True).stdout.strip()
    teardown()
    # 解析(est 的 M_TRUE/BAND 按本轮 delta 动态设)。BAND 设下限:小幅度下
    # 0.15*|delta| 会窄过 MHE 稳态噪声底(~0.03kg,2026-07-07 吊挂run实测
    # 终值误差 1.3% ≈ 0.03),入带判定永不满足 → 有效工况被误判失效。
    est.M_TRUE = M_BASE + delta
    est.BAND = max(0.15 * abs(delta), 0.03)
    # t_phys 检测阈值也须随幅度缩:静态 ΔT=9.81*|delta|,dm=0.15 时仅 1.47N,
    # 固定 1.5N 全靠暂态过冲才踩得到(2026-07-07 冒烟实测 21.67 vs 基线 20.09,
    # 险过)——检测不到 → raise → 有效工况被记罚分。取静态 ΔT 的 70% 且不高于
    # 1.5(T_phys 悬停噪声 ~0.05N,0.2N 下限余量充足)。
    est.THRESH = min(1.5, max(0.7 * 9.81 * abs(delta), 0.2))
    try:
        r = est.parse(mhe)
        c = ctl.parse(nmpc)
    except (RuntimeError, Exception):
        return None
    inband = (r['t'] >= 0) & (np.abs(r['m'] - est.M_TRUE) <= est.BAND)
    if not np.any(inband):
        return None
    # 过冲 = 首次入带之后的最大绝对偏离(方向无关——m_min 基的定义在加载方向
    # +delta 下失义:m 从下方逼近,m_min 恒等于起始值,2026-07-06 探针发现)
    k_in = int(np.argmax(inband))
    overshoot = float(np.max(np.abs(r['m'][k_in:] - est.M_TRUE)))
    return dict(
        enter=float(r['t'][inband][0]), settle=float(r['t_settle']),
        overshoot=overshoot,
        pe_peak=float(c['pe_peak']), t_rec=float(c['t_rec']),
        mhe_log=os.path.basename(mhe))


def loss_of(m):
    return (m['enter'] + m['settle'] + 8.0 * m['overshoot']
            + 3.0 * m['pe_peak'] + 0.5 * m['t_rec'])


def eval_theta(theta, logf):
    losses = []
    for delta in DELTAS:
        m = run_sitl_once(theta, delta)
        if m is None:                      # 重试一次
            m = run_sitl_once(theta, delta)
        L = loss_of(m) if m is not None else PENALTY
        losses.append(L)
        rec = dict(ts=datetime.now().isoformat(timespec='seconds'),
                   theta=[round(float(v), 4) for v in theta],
                   delta=delta, loss=round(L, 3),
                   metrics=m if m else 'FAILED')
        logf.write(json.dumps(rec, ensure_ascii=False) + '\n')
        logf.flush()
    return float(np.mean(losses))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hours', type=float, default=9.0)
    ap.add_argument('--gens', type=int, default=14)
    ap.add_argument('--pop', type=int, default=6)
    args = ap.parse_args()

    deadline = time.time() + args.hours * 3600
    rng = np.random.default_rng(7)
    mu = M0_THETA.copy().astype(float)
    sigma = SIGMA0.copy()
    n_elite = max(2, args.pop // 3)

    jsonl = f'{RESULTS}/cem_sitl_{datetime.now():%Y%m%d_%H%M%S}.jsonl'
    print(f'CEM SITL 在环优化: pop={args.pop} gens<={args.gens} '
          f'deadline={args.hours}h  log={jsonl}', flush=True)
    best = (None, np.inf)

    with open(jsonl, 'w') as logf:
        for gen in range(args.gens):
            cands = [mu + sigma * rng.standard_normal(len(mu))
                     for _ in range(args.pop)]
            if gen == 0:                     # 注入参照点
                cands[0] = M0_THETA.copy()
                cands[1] = TH2.copy()
            cands = [np.clip(c, BOUNDS[:, 0], BOUNDS[:, 1]) for c in cands]

            scores = []
            for i, c in enumerate(cands):
                if time.time() > deadline:
                    break
                L = eval_theta(c, logf)
                scores.append(L)
                print(f'gen{gen} cand{i} theta='
                      f'[{", ".join(f"{v:.2f}" for v in c)}] loss={L:.3f}',
                      flush=True)
                if L < best[1]:
                    best = (c.copy(), L)
            if len(scores) < n_elite or time.time() > deadline:
                print('deadline/数据不足,收尾', flush=True)
                break
            elite = [cands[j] for j in np.argsort(scores)[:n_elite]]
            mu = np.mean(elite, axis=0)
            # 防塌缩:除加性 0.05 外再设逐维下限=初始 sigma 的 20%。教训
            # (2026-07-07 run 005927 gen0):n_elite=2 时两精英某维碰巧接近
            # (阈值维 1.48/1.59),该维 sigma 一代就塌到 0.12,探索冻结——
            # 加性 0.05 对量纲大的维形同虚设。
            sigma = np.maximum(np.std(elite, axis=0) * 1.2 + 0.05,
                               0.2 * SIGMA0)
            print(f'== gen{gen} done: best_gen={min(scores):.3f} '
                  f'best_all={best[1]:.3f} mu=[{", ".join(f"{v:.2f}" for v in mu)}] '
                  f'sigma=[{", ".join(f"{v:.2f}" for v in sigma)}]', flush=True)

    teardown()
    if best[0] is not None:
        np.save(f'{RESULTS}/theta_sitl_cem.npy', best[0])
        print(f'FINAL best theta=[{", ".join(f"{v:.4f}" for v in best[0])}] '
              f'loss={best[1]:.3f} -> theta_sitl_cem.npy', flush=True)


if __name__ == '__main__':
    main()
