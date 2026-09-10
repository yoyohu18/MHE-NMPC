#!/bin/bash
# 论文全部图表的一键复现 —— 从冻结的实验日志重建 paper/figs/*.pdf 与 main.pdf。
#
# 设计前提:论文数据**不重跑仿真**。每个数字都来自 paper/data/ 下带时间戳的
# 冻结原始日志,本脚本只做"日志 → 图 → PDF"这一段,因此结果逐字节可复现。数据源清单
# 与"论文数字 ← 哪个文件"的逐条映射见 REPRODUCE.md。
#
# 用法:  bash paper/reproduce.sh          # 校验数据源 + 出图 + 编译
#        bash paper/reproduce.sh --check  # 只校验数据源是否齐全,不出图
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd "$REPO/.." && pwd)"
BUNDLED_RUN="$REPO/paper/data"
LEGACY_RUN="$WORKSPACE/nmpc_test_results"
if [ -f "$BUNDLED_RUN/mainline_ab2_manifest.csv" ]; then
  RUN="$BUNDLED_RUN"
  echo "数据源: $RUN (仓库内冻结数据)"
elif [ -d "$LEGACY_RUN" ]; then
  RUN="$LEGACY_RUN"
  echo "数据源: $RUN (兼容旧外部目录)"
else
  RUN="$BUNDLED_RUN"
  echo "数据源: $RUN (仓库内冻结数据)"
fi
# replay_release_detector.py 也使用同一数据根目录。
export NMPC_RESULTS_DIR="$RUN"
# 固定 PDF 内部的 CreationDate/ModDate，否则内容相同的重建仍会产生不同字节。
# 时间锚定为 2026-09-10 数据—代码审计日 00:00:00 UTC。
export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-1788998400}"
export FORCE_SOURCE_DATE=1
export TZ=UTC
FIGS="$REPO/paper/figs"
CHECK_ONLY="${1:-}"

# ---- 冻结数据源清单(改这里 = 改论文数据,慎动) ----
SOURCES=(
  "nosignal_ablation_20260713_182612.txt|Table 1 事件触发/无信号消融 n=6"
  "mhe_node_20260703_162003.log|Fig.3 fixed-weights 时线"
  "mhe_node_20260703_031851.log|Fig.3 event-triggered 时线"
  "alpha_only_20260730_214611.txt|Table 2 权重调度 2x2 析因 @20Hz n=8/格 (64 runs)"
  "alpha_only_10hz_20260731_150621.txt|Table 2 的 10Hz 复现 n=8 (32 runs)"
  "geom_grid_20260723_171711.txt|Table 4 L1-NMPC 三方矩阵 {truth,online,l1} 60 runs"
  "alpha_only_20260803_155130.txt|§IV-C 基线阈值口径消解三臂 n=8 (24 runs)"
  "wind_fpr_20260727_150703.txt|Table 5 风扰自触发 FPR n=3/格"
  "cxy_ecc_sweep_20260714_143710.txt|Fig.4 偏心扫格 est-vs-truth(绘图直接解析)"
  "grip_nmpc_20260715_165245.log|Fig.5 B.5 全流程(绘图实际读取源)"
  "grip_mhe_20260803_144036.log|Table 6 timing:MHE solve 耗时(仪表化轮次)"
  "mainline_ab2_manifest.csv|autonomous-release Tables:76 flights/38 attempted pairs"
)

echo "=== 1/3 校验冻结数据源 ==="
missing=0
if [ -f "$RUN/SHA256SUMS" ]; then
  if (cd "$RUN" && sha256sum --quiet -c SHA256SUMS); then
    echo "  OK    SHA256SUMS (711 files)                  冻结数据字节级完整性"
  else
    echo "!! 冻结数据 SHA-256 校验失败"
    missing=$((missing + 1))
  fi
elif [ "$RUN" = "$BUNDLED_RUN" ]; then
  echo "  MISS  SHA256SUMS                              仓库内冻结数据缺少完整性清单"
  missing=$((missing + 1))
fi
for entry in "${SOURCES[@]}"; do
  f="${entry%%|*}"; desc="${entry##*|}"
  if [ -e "$RUN/$f" ]; then
    printf '  OK    %-46s %s\n' "$f" "$desc"
  else
    printf '  MISS  %-46s %s\n' "$f" "$desc"
    missing=$((missing + 1))
  fi
done

check_pair() {
  local left="$1" right="$2" desc="$3"
  for f in "$RUN/$left" "$RUN/$right"; do
    if [ ! -e "$f" ]; then
      echo "  MISS  $(basename "$f")  $desc referenced"
      missing=$((missing + 1))
    fi
  done
}

# 聚合 manifest 只是索引；重算还需要它指向的每轮原始日志。
while read -r _ _ _ mhe_stamp nmpc_stamp status _; do
  [ -z "${mhe_stamp:-}" ] && continue
  check_pair "mhe_node_${mhe_stamp}.log" "acados_nmpc_node_${nmpc_stamp}.log" "nosignal manifest"
done < <(awk '!/^#/ && NF>=6 && $6=="ok"' "$RUN/nosignal_ablation_20260713_182612.txt")

for manifest in alpha_only_20260730_214611.txt alpha_only_10hz_20260731_150621.txt alpha_only_20260803_155130.txt; do
  while read -r _ _ _ _ stamp status _; do
    [ -z "${stamp:-}" ] && continue
    check_pair "grip_mhe_${stamp}.log" "grip_nmpc_${stamp}.log" "$manifest"
  done < <(awk '!/^#/ && NF>=6 && $6=="ok"' "$RUN/$manifest")
done

while read -r _ _ _ _ stamp status _; do
  [ -z "${stamp:-}" ] && continue
  check_pair "grip_mhe_${stamp}.log" "grip_nmpc_${stamp}.log" "geom-grid manifest"
done < <(awk '!/^#/ && NF>=6 && $6=="ok"' "$RUN/geom_grid_20260723_171711.txt")

while read -r _ _ _ stamp _ status _; do
  [ -z "${stamp:-}" ] && continue
  check_pair "mhe_node_${stamp}.log" "acados_nmpc_node_${stamp}.log" "wind-FPR manifest"
done < <(awk '!/^#/ && NF>=6 && $6=="ok"' "$RUN/wind_fpr_20260727_150703.txt")

# Timing 表的 NMPC 数据是明确的历史文件集，不是“当前随便找一批日志”。
# 该日期段已冻结：141 个文件中 139 个含 solve 记录，共 1009 样本。
timing_files=("$RUN"/grip_nmpc_2026073*.log)
timing_n=0
for f in "${timing_files[@]}"; do
  # awk 在 0 命中时仍返回成功，避免 set -e/pipefail 把合法的零样本日志当错误。
  n=$(awk '{ while (match($0, /solve=[0-9.]+/)) { n++; $0=substr($0, RSTART+RLENGTH) } } END { print n+0 }' "$f")
  timing_n=$((timing_n + n))
done
if [ "${#timing_files[@]}" -ne 141 ] || [ "$timing_n" -ne 1009 ]; then
  echo "!! Table timing NMPC cohort changed: files=${#timing_files[@]} (expected 141), samples=$timing_n (expected 1009)"
  missing=$((missing + 1))
else
  echo "  OK    grip_nmpc_2026073*.log (141 files)       Table timing NMPC 1009 samples"
fi

# Table release replay 的 cohort 已冻结为有序清单(73 轮 / 146 个文件 + SHA-256)。
# 这里只校验身份,完整回放命令见 REPRODUCE.md(带 --loocv,约半分钟)。
cohort_csv="$REPO/paper/manifests/release_replay_20260905.csv"
if python3 - "$REPO" "$cohort_csv" <<'PYEOF'
import sys
sys.path.insert(0, sys.argv[1] + '/scripts/gripper')
from replay_release_detector import load_manifest
rows = load_manifest(sys.argv[2])
if len(rows) != 73:
    raise SystemExit(f'cohort 行数 {len(rows)} != 73')
PYEOF
then
  echo "  OK    release_replay_20260905.csv (73 轮)      Table release replay cohort + SHA-256"
else
  echo "!! Table release replay cohort 校验失败(行数/缺文件/SHA-256 不匹配)"
  missing=$((missing + 1))
fi

# paired-release manifest 中每个 stamp 必须同时有 MHE/NMPC 原始日志。
while IFS=, read -r _ _ _ _ stamp _; do
  [ "$stamp" = "stamp" ] && continue
  [ "$stamp" = "NA" ] && continue
  for prefix in grip_mhe grip_nmpc; do
    if [ ! -e "$RUN/${prefix}_${stamp}.log" ]; then
      echo "  MISS  ${prefix}_${stamp}.log  mainline_ab2_manifest.csv referenced"
      missing=$((missing + 1))
    fi
  done
done < "$RUN/mainline_ab2_manifest.csv"
if [ "$missing" -gt 0 ]; then
  echo "!! 缺 $missing 个数据源,无法复现。这些是冻结日志,不能靠重跑仿真再生"
  echo "   (SITL 无可注入随机种子,重跑得到的是另一次实现,不是同一组数字)。"
  exit 1
fi
[ "$CHECK_ONLY" = "--check" ] && { echo "数据源齐全。"; exit 0; }

echo
echo "=== 2/3 重建论文图 (paper/figs/*.pdf) ==="
(cd "$FIGS" && python3 fig_gen.py)

echo
echo "=== 3/3 编译 main.pdf ==="
cd "$REPO/paper"
pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null
bibtex main >/dev/null 2>&1 || true
pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null
pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null

if grep -q "undefined references" main.log; then
  echo "!! 仍有未解析引用,检查 main.log"
  grep -n "Warning.*undefined" main.log | head
fi
echo "done: $(cd "$REPO/paper" && ls -la main.pdf | awk '{print $5" bytes  "$NF}')"
echo "     页数: $(pdfinfo main.pdf 2>/dev/null | awk '/^Pages/{print $2}')"
