#!/usr/bin/env python3
"""重建并冻结 Table release replay 的 73 轮 cohort(2026-09-10 恢复)。

背景:2026-09-05 的审计用探索模式入口跑出 73 轮,当时没留清单。REPRODUCE.md
一度把它记成"无界 glob",但无界 glob 在同一截止时刻给出的是 249 轮 —— 记账错了。
真实 cohort = **按 mtime 排序后、截至审计提交 078b054(2026-09-05 14:37:33)
的最后 73 个可回放轮次**(即当时 `--limit` 后缀的自然结果)。

该定义已用七项独立指纹逐位核验(见 paper/manifests/release_replay_20260905.md):
73 轮 / 72 有 drop / 58 检出 / 14 漏检 / 带载 2614 帧零误释放 /
延迟中位 2.1 s / peak_min 0.0134 / 残噪 max 0.0203 / 对抗 ratio 1.520 /
check_run_valid 判定 valid 且有 drop = 59。

validity 列来自预注册脚本 `check_run_valid.py`(自 6419cbe 起未改动),不是事后
人工挑选。

用法:
  python3 freeze_release_cohort.py > ../../paper/manifests/release_replay_20260905.csv
"""

import csv
import glob
import hashlib
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from replay_release_detector import RES, load_round  # noqa: E402

# 审计提交 078b054 的 committer 时间。cohort 的截止边界。
CUTOFF = '2026-09-05 14:37:33'
COHORT_SIZE = 73
CHECK_VALID = os.path.join(HERE, 'check_run_valid.py')


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    cut = time.mktime(time.strptime(CUTOFF, '%Y-%m-%d %H:%M:%S'))
    paths = [p for p in sorted(glob.glob(os.path.join(RES, '*mhe_*.log')),
                               key=os.path.getmtime)
             if os.path.getmtime(p) < cut]
    cohort = [(p, r) for p in paths
              for r in (load_round(p),) if r is not None][-COHORT_SIZE:]
    if len(cohort) != COHORT_SIZE:
        raise SystemExit(f'只重建出 {len(cohort)} 轮,期望 {COHORT_SIZE}:'
                         '历史日志可能已被删除,cohort 无法恢复')

    w = csv.writer(sys.stdout)
    w.writerow(['order', 'stamp', 'mhe_log', 'nmpc_log', 'validity',
                'mhe_sha256', 'nmpc_sha256'])
    for i, (mhe_path, rnd) in enumerate(cohort, 1):
        nmpc_path = mhe_path.replace('mhe_', 'nmpc_')
        cp = subprocess.run([sys.executable, CHECK_VALID, nmpc_path],
                            capture_output=True, text=True)
        if cp.returncode not in (0, 1):
            raise SystemExit(f'{nmpc_path}: check_run_valid 不可判定 '
                             f'({cp.stdout.strip()})')
        w.writerow([i, rnd.stamp, os.path.basename(mhe_path),
                    os.path.basename(nmpc_path),
                    'valid' if cp.returncode == 0 else 'invalid',
                    _sha256(mhe_path), _sha256(nmpc_path)])


if __name__ == '__main__':
    main()
