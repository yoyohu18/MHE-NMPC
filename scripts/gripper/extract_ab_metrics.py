#!/usr/bin/env python3
"""从一轮 SITL 的 NMPC/MHE 日志里抽 A/B 需要的指标,输出一行空格分隔的值。

用法: extract_ab_metrics.py <nmpc_log> <mhe_log>
输出: status peak_pos_err nmpc_failed attach_ecc m_err_p50 m_err_p90 solve_med traj n_samples

为什么要采这么多列(2026-08-31 的教训,见记忆 mhe-coupled-realtime-fix):
  · **坠机会把一切下游指标染色** —— solve 失败数、solve 耗时、估计误差都会因为
    状态发散而恶化。所以 status 必须由 NMPC 侧的 solve failed + pos_err 峰值判定,
    并且在报告里先看它,再看别的列;否则会拿坠毁过程中的数字下结论(我连续错了两次)。
  · **attach 偏心是被控但仍有残余变异的混杂变量**(见 grip-attach-eccentricity-
    roll-saturation):它同时影响估计难度和坠机率,必须逐轮记录以便事后查两臂平衡。
  · m 的误差只取 **DROP 前 STEADY_WIN 秒**这个稳态带载窗口。
    2026-08-31 第一版按 "DYNAMIC..DROP" 取,而 run_gripper_headless.sh 的
    GRIP_DYNAMIC 默认 false —— 没有 figure8 的轮次里找不到 DYNAMIC,窗口**静默
    退化成整段日志**,把 LIFT 段探底(17.05% = 撞下界 1.961)混进了"稳态精度"。
    改成锚在 DROP 上:悬停批和 figure8 批都落在带载稳态里(figure8 从 t≈14s 起、
    drop 在 t≈57s,DROP 前 20s 全在机动段内),不依赖任何可选的阶段行;
    **拿不到 DROP 就报 nan,绝不退化成全段**。
"""
import math
import re
import sys

TS = re.compile(r'\[(\d{10}\.\d+)\]')
STEADY_WIN = 20.0   # 稳态带载窗口长度 [s],锚在 DROP 之前


def _stamp(line):
    m = TS.search(line)
    return float(m.group(1)) if m else None


def main():
    nmpc_log, mhe_log = sys.argv[1], sys.argv[2]
    ntxt = open(nmpc_log, errors='ignore').read().splitlines()

    peak = 0.0
    nfail = 0
    ecc = float('nan')
    t_dyn = t_drop = None
    for line in ntxt:
        if 'solve failed' in line:
            nfail += 1
        for v in re.findall(r'pos_err=([0-9.]+)', line):
            peak = max(peak, float(v))
        m = re.search(r'attach offset received: box-drone = '
                      r'\[([-+0-9.]+), ([-+0-9.]+), ([-+0-9.]+)\]', line)
        if m and math.isnan(ecc):
            ecc = math.hypot(float(m.group(1)), float(m.group(2)))
        if 'DYNAMIC: switch to figure8' in line and t_dyn is None:
            t_dyn = _stamp(line)
        # ⚠️ 2026-09-05 修:连续主线(continuous_payload_estimates)下 NMPC 打的是
        #    "DROP command issued"(物理释放指令)+"DROP complete"(确认),legacy
        #    事件档才打 "DROP: released gripper"。只认旧串会把**成功的轮次**
        #    判成 NO_DROP,并让稳态窗口拿不到 → m_err 全 nan(实测 20260905_204345)。
        #    锚点取**指令时刻**而非 complete:那 3s 确认期载荷已经脱手,
        #    算进"带载稳态"会污染。legacy 日志没有新串,行为不变。
        if ('DROP: released gripper' in line or 'DROP command issued' in line
                or 'PAYLOAD LOST' in line) and t_drop is None:
            t_drop = _stamp(line)

    # attach 偏心:主线档(无 attach 通知订阅)的 NMPC 日志不再打 "attach offset
    # received",回退到同 stamp 的 proximity 节点日志(它打 "offset (box - drone)")。
    # 这一列是混杂变量的记录,必须拿得到,否则事后查不了两臂平不平衡
    # (见记忆 grip-attach-eccentricity-roll-saturation)。
    if math.isnan(ecc):
        prox = nmpc_log.replace('grip_nmpc_', 'grip_proximity_')
        try:
            for line in open(prox, errors='ignore'):
                m = re.search(r'offset \(box - drone\) = '
                              r'\[([-+0-9.]+), ([-+0-9.]+), ([-+0-9.]+)\]', line)
                if m:
                    ecc = math.hypot(float(m.group(1)), float(m.group(2)))
                    break
        except OSError:
            pass

    # 结局判定优先于一切:先确认飞机还在天上,别的列才有意义。
    if nfail > 100 or peak > 5.0:
        status = 'CRASH'
    elif peak > 2.0:
        status = 'DIVERGED'
    elif t_drop is None:
        status = 'NO_DROP'
    else:
        status = 'ok'

    errs, solves = [], []
    # 窗口锚在 DROP 上,见模块 docstring。t_dyn 只用来在输出里标注这轮有没有机动。
    hi = t_drop if t_drop else None
    lo = (hi - STEADY_WIN) if hi else None
    for line in open(mhe_log, errors='ignore'):
        t = _stamp(line)
        if t is None:
            continue
        ms = re.search(r'solve=([0-9.]+)ms', line)
        if ms:
            solves.append(float(ms.group(1)))
        if lo is None or not (lo <= t <= hi):
            continue
        m = re.search(r'\[truth\] m_hat=[0-9.]+ m_true=[0-9.]+ err=([-+0-9.]+)%',
                      line)
        if m:
            errs.append(abs(float(m.group(1))))

    def med(v):
        if not v:
            return float('nan')
        v = sorted(v)
        return v[len(v) // 2]

    def p90(v):
        # 用 p90 而不是 max:max 会被单帧尖峰主导(第一版就被 LIFT 探底占满)。
        if not v:
            return float('nan')
        v = sorted(v)
        return v[int(len(v) * 0.9)]

    print(f'{status} {peak:.3f} {nfail} {ecc:.4f} '
          f'{med(errs):.3f} {p90(errs):.3f} {med(solves):.2f} '
          f'{"fig8" if t_dyn else "hover"} {len(errs)}')


if __name__ == '__main__':
    main()
