#!/usr/bin/env python3
"""把 manifest 按"物理 attach"重分类后放进暂存目录,供不支持该口径的汇总脚本原样使用(2026-09-15)。

背景: check_run_valid.py 用 m_est 判断载荷是否挂上,估计器锁死会被误分为 attach-fail。
本工具不改原 manifest:对每个 manifest 生成副本,若某行 validity 含 attach-fail 且
grip_proximity_<stamp>.log 记录了 "attach offset",则改为 valid;并把该 manifest 引用的
grip_{nmpc,mhe,proximity} 日志符号链接到暂存目录。
用法: python3 stage_physical_validity.py OUT_DIR MANIFEST.csv [...]
然后: python3 aggregate_cmdarm_ab.py OUT_DIR/<manifest>.csv ...
"""
import csv
import os
import sys


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for man in sys.argv[2:]:
        src = os.path.dirname(os.path.abspath(man))
        rows = list(csv.DictReader(open(man)))
        changed = 0
        for r in rows:
            st = r.get('stamp', 'NA')
            if st == 'NA':
                continue
            for k in ('nmpc', 'mhe', 'proximity'):
                f = f'grip_{k}_{st}.log'
                s, d = os.path.join(src, f), os.path.join(out, f)
                if os.path.exists(s) and not os.path.lexists(d):
                    os.symlink(s, d)
            prox = os.path.join(src, f'grip_proximity_{st}.log')
            if ('attach-fail' in r['validity'] and os.path.exists(prox)
                    and 'attach offset' in open(prox, errors='ignore').read()):
                print(f'  {os.path.basename(man)} {r.get("arm", "?")} {st}: {r["validity"]} -> valid')
                r['validity'] = 'valid'
                changed += 1
        dst = os.path.join(out, os.path.basename(man))
        if os.path.lexists(dst):
            os.unlink(dst)
        with open(dst, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(rows)
        print(f'{os.path.basename(man)}: {len(rows)} rows, reclassified {changed}')


if __name__ == '__main__':
    main()
