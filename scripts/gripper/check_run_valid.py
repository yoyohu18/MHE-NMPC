#!/usr/bin/env python3
"""判定一个架次对主线命题是否**有效**(预注册,2026-09-04)。

无效 = 故障发生在 drop 之前,与"NMPC 是否消费自身 drop 状态"无关:
  pre-drop-crash : 位置误差在 drop 指令之前就发散(LIFT 段或 figure8 早期)
  attach-fail    : 载荷压根没上机(m_est 全程贴空机值)
这两类按预注册**作废该配对并补跑**,同时单独统计发生率。

stdout 打一行标签;exit 0=有效,1=无效,2=日志不可读。
用法: python3 check_run_valid.py <grip_nmpc_*.log>
"""
import re
import sys

AW = re.compile(r'\[attach-window\] t=([\d.-]+)s pos_err=([\d.]+)m '
                r'T=[\d.-]+N z=([\d.-]+) m_est=([\d.-]+)')
EMPTY = re.compile(r'empty m_est=([\d.]+)kg')
TCMD = re.compile(r't=([\d.]+)s \| DROP (?:command issued|: released)')

# 判据阈值。pos_err 用 2m(远超正常暂态峰 0.3m,又不会把 drop 后的正常恢复算进来);
# attach 判据 0.08kg 是最小载荷 0.15kg 的一半,估计器再保守也不至于只认到一半。
CRASH_POS_ERR = 2.0
ATTACH_MIN_MP = 0.08


def main(path):
    m_empty = 2.06
    t_cmd = None
    frames = []
    try:
        with open(path, errors='ignore') as f:
            for line in f:
                if (mm := EMPTY.search(line)):
                    m_empty = float(mm.group(1))
                if t_cmd is None and (mm := TCMD.search(line)):
                    t_cmd = float(mm.group(1))
                if (mm := AW.search(line)):
                    frames.append((float(mm.group(1)), float(mm.group(2)),
                                   float(mm.group(4))))
    except OSError as e:
        print(f'unreadable: {e}')
        return 2
    if not frames:
        print('invalid:no-frames')
        return 1

    t_crash = next((f[0] for f in frames if f[1] > CRASH_POS_ERR), None)
    if t_crash is not None and (t_cmd is None or t_crash < t_cmd):
        print(f'invalid:pre-drop-crash (t_crash={t_crash:.1f}s, '
              f't_drop_cmd={"none" if t_cmd is None else f"{t_cmd:.1f}s"})')
        return 1

    # 用 90 分位数而不是峰值:m̂ 的单帧噪声尖点能到 +0.11kg,拿峰值判会让
    # 真正没抓上的轮次漏网(#19 全程贴 2.064,却有个 2.173 的尖点),同时把
    # 正常轮误伤。p90 要求"载荷在机上是持续状态",这正是 attach 的定义。
    ms = sorted(f[2] for f in frames)
    m_p90 = ms[int(0.90 * (len(ms) - 1))]
    if m_p90 - m_empty < ATTACH_MIN_MP:
        print(f'invalid:attach-fail (m_p90-m_empty={m_p90 - m_empty:.3f}kg)')
        return 1

    print('valid')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: check_run_valid.py <grip_nmpc_*.log>')
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
