#!/bin/bash
# 2026-09-15 夜间串联(用户授权):依次跑完剩余批次。本脚本不 kill 任何进程 ——
# 每个批次脚本在每轮开跑前自带 cleanup,结束时也自带 cleanup;这里只负责排队。
# 顺序(按对论文的价值/耗时):
#   0 等当前 4 m/s 四箱部分丢失批次(run_partial_loss.sh)结束
#   1 延迟脱离 matched 基线补 1 对(W5)
#   2 意外脱落 W5 补 2 对
#   3 意外脱落 W3(0.15 kg, 4 m/s)5 对
#   4 估计器侧推力映射失配(W5, 0.95/1.00/1.05 x 4 组)
#   5 意外脱落 W4(0.30 kg, 4 m/s)5 对
#   6 窗口力平衡种子 A/B(W4, 8 有效对, 上限 20 次)
# ⚠️ 运行期间不要编辑任何被调用的脚本(bash 增量读脚本,会读错位)。
set -u
WS=/home/clear/ros2_ws_HJH
G=$WS/src/scripts/gripper
LOG=$WS/nmpc_test_results/chain_overnight_20260915.log

step() {
  local name="$1"; shift
  echo "[chain] $(date +%T) START $name" >> "$LOG"
  env "$@" >> "$LOG" 2>&1 9>&-
  echo "[chain] $(date +%T) END $name exit=$?" >> "$LOG"
  sleep 30
}

echo "[chain] $(date +%T) waiting for run_partial_loss.sh" >> "$LOG"
while pgrep -f "run_partial_loss.sh" >/dev/null; do sleep 20; done
sleep 30

step delayed_makeup   PAIRS=1 FAULT=delayed bash "$G/run_matched_cmd_armed_ab.sh"
step uncmd_W5_makeup  PAIRS=2 FAULT=uncommanded bash "$G/run_matched_cmd_armed_ab.sh"
step uncmd_W3         PAIRS=5 FAULT=uncommanded WORKPOINT=W3 bash "$G/run_matched_cmd_armed_ab_v2.sh"
step thrustmap        PAIRS=4 FAULT=thrustmap ARMS="G0.95 G1.00 G1.05" POST_SEC=20 bash "$G/run_matched_cmd_armed_ab.sh"
step uncmd_W4         PAIRS=5 FAULT=uncommanded WORKPOINT=W4 bash "$G/run_matched_cmd_armed_ab_v2.sh"
step seed_window      VALID_PAIRS=8 MAX_TRIES=20 ONLY_W=W4 bash "$G/run_seed_window_ab.sh"
echo "[chain] $(date +%T) ALL DONE" >> "$LOG"
