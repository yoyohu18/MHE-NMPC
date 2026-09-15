#!/usr/bin/env python3
"""Aggregate the final-interface residual-step on/off ablation.

The manifest records eight paired W3 flights.  Step-on enables the optional
0.6 N residual-step MHE window reweighting; step-off is the deployed setting.
Release-residual geometry clearing was disabled in both arms.

Usage:
  python3 scripts/gripper/aggregate_step_ab.py \
    ../nmpc_test_results/stepab_20260912_234849.txt
"""
import pathlib
import re
import statistics
import sys

from scipy.stats import wilcoxon


TS_LINE = re.compile(r"^\[[A-Z]+\] \[(\d+\.\d+)\].*", re.M)


def first_timestamp(text, phrase):
    match = re.search(r"^\[[A-Z]+\] \[(\d+\.\d+)\].*" + re.escape(phrase), text, re.M)
    return float(match.group(1)) if match else None


def analyze(root, arm, pair, stamp, status):
    nmpc = (root / f"grip_nmpc_{stamp}.log").read_text(errors="ignore")
    mhe = (root / f"grip_mhe_{stamp}.log").read_text(errors="ignore")
    attach = first_timestamp(nmpc, "ATTACH command issued")
    drop = first_timestamp(nmpc, "DROP command issued")
    complete = first_timestamp(nmpc, "DROP complete")
    edges = [
        (float(m.group(1)), m.group(2))
        for m in re.finditer(
            r"^\[INFO\] \[(\d+\.\d+)\].*\[step-detect\] "
            r"阶跃自检测.*direction=(\w+)",
            mhe,
            re.M,
        )
    ]
    expected_attach = next(
        (edge for edge in edges if edge[1] == "ATTACH" and attach <= edge[0] <= attach + 10.0),
        None,
    ) if attach is not None else None
    expected_release = next(
        (edge for edge in edges if edge[1] == "DROP" and drop <= edge[0] <= drop + 10.0),
        None,
    ) if drop is not None else None
    expected = {expected_attach, expected_release} - {None}
    return {
        "arm": arm,
        "pair": pair,
        "stamp": stamp,
        "status": status,
        "confirmed": complete is not None,
        "delay": complete - drop if complete is not None and drop is not None else None,
        "edges": len(edges),
        "expected_edges": len(expected),
        "unintended_edges": len(edges) - len(expected),
    }


def main():
    manifest = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(
        "../nmpc_test_results/stepab_20260912_234849.txt"
    )
    root = manifest.parent
    rows = []
    for line in manifest.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] in {"A", "B"}:
            rows.append(analyze(root, fields[0], int(fields[1]), fields[2], fields[3]))

    by_key = {(row["arm"], row["pair"]): row for row in rows}
    print("Final-interface residual-step reweighting ablation")
    for arm, label in (("A", "step-on"), ("B", "step-off")):
        selected = [row for row in rows if row["arm"] == arm]
        print(
            f"  {label}: confirmed {sum(row['confirmed'] for row in selected)}/{len(selected)}, "
            f"edges {sum(row['edges'] for row in selected)}, "
            f"unintended {sum(row['unintended_edges'] for row in selected)}"
        )

    complete_pairs = []
    for pair in range(1, 9):
        on = by_key[("A", pair)]
        off = by_key[("B", pair)]
        if on["delay"] is not None and off["delay"] is not None:
            complete_pairs.append(on["delay"] - off["delay"])
    test = wilcoxon(complete_pairs, alternative="two-sided", method="exact")
    print(
        f"  complete pairs {len(complete_pairs)}/8, median(on-off) "
        f"{statistics.median(complete_pairs):+.2f} s, exact Wilcoxon p={test.pvalue:.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
