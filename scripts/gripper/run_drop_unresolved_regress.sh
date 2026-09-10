#!/bin/bash
# UNRESOLVED 兜底路径的**构造回归**(2026-09-07)。
#
# 【历史缺陷】`_check_drop_unresolved()` 曾只用 `grip_dynamic_active=False` 表达
# "退出 figure-8",而该标志同时是 `_grip_dynamic_phase()` 的"只切一次"哨兵 →
# 清掉它的下一拍(50ms 后)figure-8 被**原地重启**:圆心重置到当前位置、相位 t0
# 归零、ramp 从头,而 build_reference_figure8 的 hover_time=2.0 让重启后头 2s
# 的速度参考 = 0 —— 要求一架 4m/s 的飞机瞬间刹停。实测 5/5 失败轮时序一致:
#   UNRESOLVED +12.00s → figure-8 重启 +12.05s → 首次 solve failed +12.75s
#   → 1.25s 内 z 6.14→0.13(vz −1.75→−10.5 m/s)穿地。
#
# 【为什么构造而不是等自然发生】自然工况下 UNRESOLVED 的触发率约 50%
# (drop 后 |s| 塌不下去才超时),触发后约 50% 穿地 → 总发生率仅 25%,
# n=16 也只能排除到约 20% 上界。把 drop_unresolved_timeout_sec 调到 2s
# 让**每轮**都走该路径,发生率提到接近 100%,n=6 即有决定性功效。
#
# 【预注册判据】
#  ① 机制判据(确定性,n=1 即可判):UNRESOLVED 日志必须明确写出“平滑制动后在
#     停止点悬停”,且其后**不应**再跟一条 "DYNAMIC: switch to figure8"。
#     前者验证 ref_fn/ref_window_fn 的语义切换版本在场,后者验证不会重入机动。
#  ② 结局判据(计数):drop 后穿地率(peak_post>5m)。修复前预期 ≥5/6,修复后 0/6。
#  ③ 前置:drop **之前**必须正常(peak_pre<2m),否则该轮作废 —— 本回归只问
#     drop 后的事,drop 前的发散(如 08-24 那类)是另一个问题。
#
# 用法(两组之间只差那一处代码改动,**别在同一组里改代码**):
#   TAG=before REPS=6 bash src/scripts/gripper/run_drop_unresolved_regress.sh
#   ...改代码 + colcon build...
#   TAG=after  REPS=6 bash src/scripts/gripper/run_drop_unresolved_regress.sh
# 聚合: python3 src/scripts/gripper/aggregate_drop_unresolved.py <before.txt> <after.txt>
set -u

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

exec 9>"/tmp/drop_unresolved_regress.lock"
if ! flock -n 9; then
  echo "[reg] 已有实例在跑,退出。"; exit 1
fi

TAG="${TAG:-before}"
REPS="${REPS:-6}"
UNRES_TIMEOUT="${UNRES_TIMEOUT:-2.0}"   # ★ 构造:让每轮都走 UNRESOLVED
UNRES_STOP_SEC="${UNRES_STOP_SEC:-3.0}"
RECOVER_SEC="${RECOVER_SEC:-45}"
TIMEOUT="${TIMEOUT:-300}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="$RUNDIR/drop_unresolved_${TAG}_${STAMP}.txt"
{
  echo "# UNRESOLVED 构造回归  tag=$TAG  $STAMP"
  echo "# 列: tag rep nmpc_stamp status peak_pos_err nmpc_failed attach_ecc m_err_p50 m_err_p90 solve_med traj n_samples"
  echo "# 构造: drop_unresolved_timeout_sec=$UNRES_TIMEOUT (默认 12.0) → 每轮强制走 UNRESOLVED 路径"
  echo "# 制动: drop_unresolved_stop_sec=$UNRES_STOP_SEC"
  echo "# 底座: 4m/s 主线档,与 run_cxy_mass_repeat.sh 逐字对齐"
  echo "# 判据①机制: 日志声明平滑制动到停止点,且 UNRESOLVED 后不再进入 figure8"
  echo "# 判据②结局: drop 后 peak_post>5m 的轮次数"
} > "$MANIFEST"
echo "[reg] tag=$TAG manifest: $MANIFEST"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "transport13/gz-transport-topic" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(pgrep -f "$pat" 2>/dev/null); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[reg] interrupted"; cleanup; exit 130' INT TERM

run_one() {
  local rep="$1"
  local launch="$RUNDIR/unres_launch_${TAG}_${STAMP}_${rep}.log"
  echo "[reg] === $TAG rep=$rep $(date +%T) ==="
  cleanup
  DROP_UNRES_TIMEOUT=$UNRES_TIMEOUT \
  DROP_UNRES_STOP_SEC=$UNRES_STOP_SEC \
  GRIP_PAYLOAD_KG=0.15 GRIP_PAYLOAD_ENVELOPE=0.3 GRIP_ECC_Y=0.10 \
    GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
    GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 \
    GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
    DJ_TRACK_MEST=true GRIP_DJ_FLOOR_MP=0.05 MHE_C_XY_EST=true \
    DJ_RATCHET=false \
    DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
    MHE_CONFIRM_ALPHA=1.5 MHE_RESID_STEP=0 \
    MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self NMPC_GEOM_RELEASE_MODE=event \
    MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
    MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
    EVAL_TRUE_PAYLOAD_MASS=0.15 \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 12); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "$TAG $rep - LAUNCH_FAIL - - - - - - - 0" >> "$MANIFEST"; cleanup; return
  fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")
  local mhe="${nmpc/grip_nmpc_/grip_mhe_}"

  # 核对构造真的生效(超时值打进了 UNRESOLVED 那行文案)
  local waited=0 dropped=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    if grep -qE "DROP complete|DROP: released" "$nmpc" 2>/dev/null; then dropped=1; break; fi
    sleep 6; waited=$((waited+6))
  done
  [ "$dropped" -eq 1 ] && sleep "$RECOVER_SEC"

  local metrics
  metrics=$(python3 "$WS/src/scripts/gripper/extract_ab_metrics.py" "$nmpc" "$mhe" 2>/dev/null)
  [ -z "$metrics" ] && metrics="EXTRACT_FAIL 99 99 nan nan nan nan - 0"
  echo "$TAG $rep $nstamp $metrics" >> "$MANIFEST"
  echo "[reg] $TAG rep$rep $nstamp -> $metrics"
  cleanup
}

for rep in $(seq 1 "$REPS"); do run_one "$rep"; done
echo "[reg] done. manifest: $MANIFEST"
