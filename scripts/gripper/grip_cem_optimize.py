#!/usr/bin/env python3
"""吊挂 attach 场景 SITL 在环权重时间表优化(长期计划 §阶段B步骤2,2026-07-08)。

从 masschanger/sitl_cem_optimize.py fork 而来。CEM 机制、5 维 θ 语义、日志解析、
teardown 护栏都与 wrench 版一致——差异全在"场景"层:载具换成真刚体 gripper
attach(不是 wrench 恒力),花名册换成质量×偏心网格,真值质量恒为加载(空机
+ payload,不像 wrench 的 delta 可正可负),确认阈值按 gripper 物理瞬时生效重标
(没有 wrench 的 gz CLI ~1.1s 冷启动延迟)。

5 维 θ = 4 维权重时间表(mhe_weight_learning.ParametricWeightSchedule)+ 1 维
**无量纲确认阈值 α**(2026-07-09 重参数化,原为绝对值 [N])。实际阈值
= α·GRAVITY·mass,见 BOUNDS/M0_THETA 处注释。首轮 N-绝对值版已证"能学出收益"
的存在性,但留出中间质量外推失败(θ4 力量纲没归一化),移植诊断坐实根因,本版
改无量纲 α 让阈值随 ΔT 自适应。节奏维 θ0/θ1/θ3 是有迁移价值的无量纲量,不动。

用法: python3 grip_cem_optimize.py [--hours 4] [--gens 6] [--pop 4]
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
LAUNCHER = f'{WS}/src/scripts/gripper/run_gripper_headless.sh'
# 解析模块仍复用 masschanger 里那份(scenario-agnostic,靠运行时 setattr
# M_TRUE/BAND/THRESH 适配场景,parse_dropwindow_logs.py 的正则已支持
# [attach-window] tag)。
sys.path.insert(0, f'{WS}/src/scripts/masschanger')
import parse_m0_logs as est      # noqa: E402  (M_TRUE/BAND/THRESH 运行时 setattr)
import parse_dropwindow_logs as ctl  # noqa: E402

# 空机质量常数(acados_params.py p.m / offboard_test.nmpc_node.Params.m)。
# gripper 真值质量 = 空机 + payload,恒为加载。
M_EMPTY = 2.0643
GRAVITY = 9.81   # 静态 ΔT = GRAVITY*mass(载荷重量),θ4=α 的归一化基准
# θ 第 5 维 = **无量纲确认阈值 α**(2026-07-09 重参数化,原来是绝对值 [N])。
# 动机:θ4 是唯一力量纲维,直接跟 |T_phys-baseline|≈ΔT=g·m_p 比较,固定绝对值
# 在不同质量上呈现的"有效相对门槛"差一倍(θ*=1.485N 在 0.15kg 是 α=1.01、在
# 0.30kg 是 α=0.50),CEM 学出的折中在留出中间质量上外推失败。移植诊断坐实:
# 只把 θ4 按 ΔT 归一化(节奏维 θ0/θ1/θ3 原样)就能在留出点稳定反超 M0
# (Δloss 0.56>4σ,Δenter 0.16s>3σ)。改成搜 α,run_sitl_once 里实际阈值
# = α·GRAVITY·mass,结构上就能外推到任意质量。区间 [0.1,1.0]:下界 0.1 保证
# 0.1·ΔT 仍高于 T_phys 悬停噪声(~0.05N),上界 1.0=门槛等于满 ΔT(再高=等完全
# 稳态才确认,无意义)。详见记忆 mhe-learning-m0-event-trigger。
BOUNDS = np.array([[-6.0, -0.5], [-3.0, 2.0], [0.0, 20.0], [-2.0, 3.0],
                   [0.1, 1.0]])
# 种子:前 4 维 = M0 规则版节奏(scenario-agnostic,无量纲、直接沿用);第 5 维
# = α 种子 0.5(移植诊断验证过的好值,scaled 成功 α≈0.505)。
M0_THETA = np.array([-4.0, 0.0, 0.0, 0.0, 0.5])
# 评估花名册:质量(0.15/0.30kg)× 偏心(0.05/0.10m)网格,精简 2×2 起步
# (gripper 单集比 wrench 慢且方差大)。0.05/0.10 是今天(07-08)descend 修复后
# 已验证稳定的两个偏心值,不引入未验证的新值。所有候选用同一花名册
# (common random numbers,工况差异不淹没 θ 差异)。
ROSTER = ((0.30, 0.05), (0.30, 0.10), (0.15, 0.05), (0.15, 0.10))
# 方案C 永久地板(2026-07-09,见 mhe_node.py `_payload_geometry`/记忆
# mhe-learning-m0-event-trigger):打破 m_est<->dJ/c_xy 自举耦合,0.15kg 两个
# 偏心点没有这个 CEM 全灭(07-08 首轮 smoke test 4/4 FAILED)。地板取花名册
# 最小质量档(操作下界弱先验,不是逐工况真值)。忘加这个等于让 0.15kg 退回
# 原棘轮又全灭——CEM 会把两个工况的 PENALTY=20 当常数吃掉,浪费一半算力
# 还测不出真实收益。
GEOM_MP_FLOOR = min(m for m, _ in ROSTER)
PENALTY = 20.0   # 单工况失效(重试仍失败)的罚分 loss
SIGMA0 = np.array([1.5, 1.2, 5.0, 1.2, 0.3])   # 初始搜索宽度(第5维=α尺度,收窄)

# 单集收尾等待:gripper attach 瞬态比 wrench 长(实测 20-60s+),不照抄 wrench 的
# 固定 sleep(20)。设成略宽于 acados_nmpc_node 的 attach_window_sec(默认 40s),
# 保证 [attach-window] 日志窗口写完再解析。先用简单固定等待,不做"提前判定收敛"
# 的智能退出(等跑出数据证明确实需要再加)。
ATTACH_TAIL_MAX_SEC = 45
# 等新日志出现 / 等 mass event 出现的超时:gripper 启动链路更长(脚本自带
# ~28s 固定 sleep + ~10s 爬升对齐 + ~5-7s 下降到 attach,~45-50s 才可能有事件),
# 比 wrench 宽松。
LOG_APPEAR_TRIES = 240   # 等 grip_mhe 日志出现(每次 1s)
EVENT_TRIES = 180        # 等 mass event 出现(attach ~50-60s 内确定性发生)


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


def run_sitl_once(theta, mass, ecc):
    """一轮无头 gripper SITL,返回指标 dict 或 None(失效)。"""
    t0 = time.time()
    # θ4=α 无量纲 → 实际确认阈值 [N] = α·ΔT_static = α·GRAVITY·mass。
    # 这样同一个 α 在不同质量上对应"相同的相对门槛",结构上可外推。
    confirm_thresh_n = theta[4] * GRAVITY * mass
    env = dict(os.environ,
               GRIP_PAYLOAD_KG=f'{mass}',
               GRIP_ECC_Y=f'{ecc}',
               MHE_EVENT_TRIGGER='true',
               MHE_SCHEDULE_THETA='[' + ','.join(
                   f'{v:.4f}' for v in theta[:4]) + ']',
               MHE_CONFIRM_THRESH=f'{confirm_thresh_n:.4f}',
               GRIP_GEOM_MP_FLOOR=f'{GEOM_MP_FLOOR}')
    subprocess.run([LAUNCHER], env=env, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    # 等新的非空 grip_mhe 日志
    mhe = None
    for _ in range(LOG_APPEAR_TRIES):
        cand = subprocess.run(
            f'ls -t {RESULTS}/grip_mhe_*.log | head -1',
            shell=True, capture_output=True, text=True).stdout.strip()
        if cand and os.path.getmtime(cand) >= t0 and os.path.getsize(cand) > 100:
            mhe = cand
            break
        time.sleep(1)
    if mhe is None:
        teardown()
        return None
    # 等 attach 事件
    ok = False
    for _ in range(EVENT_TRIES):
        if 'mass event' in open(mhe, errors='ignore').read():
            ok = True
            break
        time.sleep(1)
    if not ok:
        teardown()
        return None
    time.sleep(ATTACH_TAIL_MAX_SEC)   # [attach-window] 日志窗口收尾
    nmpc = subprocess.run(
        f'ls -t {RESULTS}/grip_nmpc_*.log | head -1',
        shell=True, capture_output=True, text=True).stdout.strip()
    teardown()
    # 真值质量恒为加载(空机 + payload),无正负号分支。BAND 设 0.03 下限:
    # 0.15*mass 在 0.15kg 档(0.0225)会窄过 MHE 稳态噪声底(~0.03kg)。
    est.M_TRUE = M_EMPTY + mass
    est.BAND = max(0.15 * mass, 0.03)
    # 物理生效检测阈值随质量缩:静态 ΔT=9.81*mass,0.15kg 仅 1.47N,固定 1.5N
    # 是盲区。取静态 ΔT 的 70% 且不高于 1.5(T_phys 悬停噪声 ~0.05N,0.2N 下限
    # 余量充足)。控制层解析(ctl)也用同一阈值——wrench 版没做这步是因为没触发
    # 这个必要性,gripper 小质量档正好是盲区。
    est.THRESH = min(1.5, max(0.7 * 9.81 * mass, 0.2))
    ctl.THRESH = est.THRESH
    try:
        r = est.parse(mhe)
        c = ctl.parse(nmpc)
    except (RuntimeError, Exception):
        return None
    inband = (r['t'] >= 0) & (np.abs(r['m'] - est.M_TRUE) <= est.BAND)
    if not np.any(inband):
        return None
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
    for mass, ecc in ROSTER:
        m = run_sitl_once(theta, mass, ecc)
        if m is None:                      # 重试一次
            m = run_sitl_once(theta, mass, ecc)
        L = loss_of(m) if m is not None else PENALTY
        losses.append(L)
        rec = dict(ts=datetime.now().isoformat(timespec='seconds'),
                   theta=[round(float(v), 4) for v in theta],
                   mass=mass, ecc=ecc, loss=round(L, 3),
                   metrics=m if m else 'FAILED')
        logf.write(json.dumps(rec, ensure_ascii=False) + '\n')
        logf.flush()
    return float(np.mean(losses))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hours', type=float, default=4.0)
    ap.add_argument('--gens', type=int, default=6)
    ap.add_argument('--pop', type=int, default=4)
    args = ap.parse_args()

    deadline = time.time() + args.hours * 3600
    rng = np.random.default_rng(7)
    mu = M0_THETA.copy().astype(float)
    sigma = SIGMA0.copy()
    n_elite = max(2, args.pop // 3)

    jsonl = f'{RESULTS}/grip_cem_{datetime.now():%Y%m%d_%H%M%S}.jsonl'
    print(f'gripper CEM SITL 在环优化: pop={args.pop} gens<={args.gens} '
          f'deadline={args.hours}h  log={jsonl}', flush=True)
    best = (None, np.inf)

    with open(jsonl, 'w') as logf:
        for gen in range(args.gens):
            cands = [mu + sigma * rng.standard_normal(len(mu))
                     for _ in range(args.pop)]
            if gen == 0:                     # 只注入 M0 规则参照(不注 wrench TH2:
                cands[0] = M0_THETA.copy()   # 不同物理体制,无迁移依据)
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
            # 防塌缩:除加性 0.05 外再设逐维下限=初始 sigma 的 20%。
            sigma = np.maximum(np.std(elite, axis=0) * 1.2 + 0.05,
                               0.2 * SIGMA0)
            print(f'== gen{gen} done: best_gen={min(scores):.3f} '
                  f'best_all={best[1]:.3f} mu=[{", ".join(f"{v:.2f}" for v in mu)}] '
                  f'sigma=[{", ".join(f"{v:.2f}" for v in sigma)}]', flush=True)

    teardown()
    if best[0] is not None:
        # 无量纲 α 版单独存,不覆盖旧 N-绝对值版 theta_grip_cem.npy(对照要用)。
        np.save(f'{RESULTS}/theta_grip_cem_alpha.npy', best[0])
        print(f'FINAL best theta(α版)=[{", ".join(f"{v:.4f}" for v in best[0])}] '
              f'loss={best[1]:.3f} -> theta_grip_cem_alpha.npy', flush=True)


if __name__ == '__main__':
    main()
