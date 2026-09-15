#!/bin/bash
# 2026-09-15 06:1x 补样本(用户已授权夜间串联):不 kill 任何进程,只排队。
#   1 四箱部分丢失 4 m/s 补 3 对(作废率 ~30%,目标凑满 5 对有效)
#   2 意外脱落 W5 补 2 对(目标凑满 5 对有效)
set -u
WS=/home/clear/ros2_ws_HJH
G=$WS/src/scripts/gripper
LOG=$WS/nmpc_test_results/chain_topup_20260915.log
step() {
  local name="$1"; shift
  echo "[chain] $(date +%T) START $name" >> "$LOG"
  env "$@" >> "$LOG" 2>&1 9>&-
  echo "[chain] $(date +%T) END $name exit=$?" >> "$LOG"
  sleep 30
}
step partial_S4_topup PAIRS=3 SPEED=4 bash "$G/run_partial_loss.sh"
step uncmd_W5_topup2  PAIRS=2 FAULT=uncommanded bash "$G/run_matched_cmd_armed_ab.sh"
echo "[chain] $(date +%T) ALL DONE" >> "$LOG"
