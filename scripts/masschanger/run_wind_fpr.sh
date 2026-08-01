#!/bin/bash
# C.3 风扰消融 —— 误触发率(FPR)扫格(2026-07-27)。
#
# 【问什么】残差自触发检测器(event_signal_mode=residual)在**纯风扰、无质量突变**
# 下会不会误触发?误触发率随风幅怎么变?对水平风(只压姿态、不显著改集合推力)
# 是否天然免疫?—— 这就是长期计划/论文草案 §VI-E C.3 要的"误触发率 vs 风扰幅度"。
#
# 【设计】masschanger 场景 DELTA_KG=0(全程无质量突变,任何 self-detected 都是误触发)。
# 每轮:起栈→等 detector armed→gz topic 施持续风(给定轴/幅度)→稳飞观测窗→
# 数 'self-detected' 条数(>0=该轮误触发)→收栈。轴 z=垂直(升/沉气流,预期>阈值
# 才误触发)、x=水平(预期几乎不误触发,亮点对照)。
#
# 用法:  AXES="z x" MAGS_Z="0.5 1.0 1.5 2.0 2.5 3.0" MAGS_X="1.0 2.0 3.0" REPS=3 \
#         [OBS_SEC=35] [MANIFEST=续跑] bash src/scripts/masschanger/run_wind_fpr.sh
# 长跑须 setsid。不用 set -e/-u(要逐轮兜错)。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
AXES="${AXES:-z x}"
MAGS_Z="${MAGS_Z:-0.5 1.0 1.5 2.0 2.5 3.0}"
MAGS_X="${MAGS_X:-1.0 2.0 3.0}"
REPS="${REPS:-3}"
OBS_SEC="${OBS_SEC:-35}"          # 施风后稳飞观测窗
ARM_TIMEOUT="${ARM_TIMEOUT:-180}" # 等 detector armed 上限
MODEL="x500_payload_0::base_link"
WORLD="default"
STAMP=$(date +%Y%m%d_%H%M%S)

source /opt/ros/jazzy/setup.bash 2>/dev/null

# --- 跨工作区守卫(2026-07-27,memory gripper-slung-load-vs-mhe/c3-wind-ablation:
#     共享 px4_sitl_default,他人栈会 kill 我的,反之亦然。flock 只防自己)---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[wind] 拒启:检测到非本工作区的 px4/gz/make 在跑(共享构建会互杀)。"
  echo "$foreign" | cut -c1-100
  exit 2
fi
exec 8>"/tmp/wind_fpr.lock"
flock -n 8 || { echo "[wind] 已有本脚本实例在跑,退出。"; exit 1; }

MANIFEST="${MANIFEST:-$RUNDIR/wind_fpr_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  {
    echo "# C.3 wind FPR $STAMP : axis mag_N rep mhe_stamp n_selfdetect status"
    echo "# 配置: DELTA_KG=0(无质量突变) signal=residual thresh=1.5N persist=2 warmup=20"
    echo "# 配置: axes=${AXES} mags_z=${MAGS_Z} mags_x=${MAGS_X} reps=${REPS} obs=${OBS_SEC}s"
    echo "# 判据: n_selfdetect>0 = 该轮误触发(FPR 分子); status=ok/ARM_TIMEOUT/CRASH"
  } > "$MANIFEST"
fi
echo "[wind] manifest: $MANIFEST"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "ros_gz_bridge" "parameter_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(ps -eo pid,cmd | grep "$pat" | grep -v grep | grep -v ugrep | grep "$WS" | awk '{print $1}')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  # gz/px4/mavros 不带 $WS 标识,按名清(此时已守卫过无他人栈)
  for pat in "px4_sitl_default/bin/px4" "gz sim" "mavros_node" "gcs_heartbeat.py"; do
    pids=$(ps -eo pid,cmd | grep "$pat" | grep -v grep | grep -v ugrep | awk '{print $1}')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[wind] interrupted"; cleanup; exit 130' INT TERM

run_one() {
  local axis="$1" mag="$2" rep="$3"
  if grep -qE "^$axis $mag $rep .* (ok|CRASH)\$" "$MANIFEST" 2>/dev/null; then
    echo "[wind] skip $axis $mag $rep (done)"; return
  fi
  echo "[wind] === axis=$axis mag=${mag}N rep=$rep ==="
  local LAUNCH="$RUNDIR/wind_launch_${STAMP}_${axis}_${mag}_${rep}.log"
  MHE_SIGNAL_MODE=residual MHE_EVENT_TRIGGER=true MASS_CHANGER_DELTA_KG=0 \
    bash "$WS/src/scripts/masschanger/run_sitl_headless.sh" > "$LAUNCH" 2>&1
  local M=$(grep -oE "$RUNDIR/mhe_node_[0-9_]+\.log" "$LAUNCH" | head -1)
  local N=$(grep -oE "$RUNDIR/acados_nmpc_node_[0-9_]+\.log" "$LAUNCH" | head -1)
  local stamp=$(echo "$M" | grep -oE "[0-9]{8}_[0-9]{6}")
  # 等 detector armed
  local waited=0 armed=0
  while [ "$waited" -lt "$ARM_TIMEOUT" ]; do
    grep -q "detector armed" "$M" 2>/dev/null && { armed=1; break; }
    sleep 4; waited=$((waited+4))
  done
  if [ "$armed" != 1 ]; then
    echo "$axis $mag $rep $stamp 0 ARM_TIMEOUT" >> "$MANIFEST"
    echo "[wind] $axis $mag $rep -> ARM_TIMEOUT"; cleanup; return
  fi
  sleep 3   # armed 后稳一拍再施风
  # 施持续风
  local force="force: {${axis}: -${mag}}"
  [ "$axis" = x ] && force="force: {x: ${mag}}"   # 水平方向符号无所谓,取正
  gz topic -t "/world/$WORLD/wrench/persistent" -m gz.msgs.EntityWrench \
    -p "entity: {name: \"$MODEL\", type: LINK}, wrench: {${force}}" >/dev/null 2>&1
  # 观测窗
  sleep "$OBS_SEC"
  # 数误触发 + 崩溃判据(pos_err 峰>2m)
  local ndet=$(grep -c "self-detected" "$M" 2>/dev/null); ndet=${ndet:-0}
  local peak=$(grep -oE "pos_err=[0-9.]+" "$N" 2>/dev/null | grep -oE "[0-9.]+" | sort -rn | head -1)
  local status=ok
  [ -n "$peak" ] && awk "BEGIN{exit !($peak>2.0)}" && status=CRASH
  echo "$axis $mag $rep $stamp $ndet $status" >> "$MANIFEST"
  echo "[wind] $axis $mag $rep stamp=$stamp n_selfdetect=$ndet peak=${peak:-NA} -> $status"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  for mag in $MAGS_Z; do [[ " $AXES " == *" z "* ]] && run_one z "$mag" "$rep"; done
  for mag in $MAGS_X; do [[ " $AXES " == *" x "* ]] && run_one x "$mag" "$rep"; done
done
echo "[wind] done. manifest: $MANIFEST"
echo "[wind] 聚合: 按 (axis,mag) 数 n_selfdetect>0 的比例 = FPR"
