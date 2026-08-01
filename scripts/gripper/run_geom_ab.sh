#!/bin/bash
# B.3 Phase2 online-vs-truth A/B(2026-07-14):同工况配对比"NMPC 吃在线 c_xy 估计"
# (geom_source=online)vs"吃 attach 真值几何"(truth)的控制层暂态。证在线估计版
# ≈真值版(消 attach 真值依赖不掉性能)。复用 run_gripper_headless.sh(cleanup+构建+
# 起栈),从其 launch 输出锁定每轮确切日志 stamp(避开 ls -t 抓旧日志的坑)。
# 每轮跑到 lift 完成+悬停稳,记 nmpc stamp+mode 进 manifest;控制层指标离线用
# parse_dropwindow_logs(aggregate_geom_ab.py)解析。
#
# 用法:  MODES="truth online" REPS=3 GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 \
#         bash src/scripts/gripper/run_geom_ab.sh
# 长跑须 setsid 脱离会话(见记忆 nosignal-ablation)。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
MODES="${MODES:-truth online}"
REPS="${REPS:-3}"
PAYLOAD="${GRIP_PAYLOAD_KG:-0.3}"
ECC="${GRIP_ECC_Y:-0.10}"
NEED_AW="${NEED_AW:-220}"    # 等够多少条 [attach-window](lift 完成+悬停,~11s@20Hz)
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="$RUNDIR/geom_ab_${STAMP}.txt"
echo "# B.3 Phase2 geom A/B $STAMP payload=${PAYLOAD}kg ecc=${ECC}m : mode rep nmpc_stamp status" > "$MANIFEST"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(pgrep -f "$pat" 2>/dev/null); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[ab] interrupted"; cleanup; exit 130' INT TERM

for mode in $MODES; do
  for rep in $(seq 1 "$REPS"); do
    echo "[ab] === mode=$mode rep=$rep ==="
    LAUNCH="$RUNDIR/geom_ab_launch_${STAMP}_${mode}_${rep}.log"
    GRIP_PAYLOAD_KG=$PAYLOAD GRIP_ECC_Y=$ECC USE_MHE=true MHE_C_XY_EST=true \
      GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=$mode \
      bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1
    # 从 launch 输出拿本轮确切 nmpc 日志(run_gripper_headless 打印 nmpc=...)
    NMPC=""; for k in $(seq 1 12); do
      NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
      [ -n "$NMPC" ] && break; sleep 5
    done
    nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
    # 等 lift 完成
    waited=0; aw=0
    while [ "$waited" -lt "$TIMEOUT" ]; do
      aw=$(grep -c "attach-window" "$NMPC" 2>/dev/null)
      [ -n "$aw" ] && [ "$aw" -ge "$NEED_AW" ] && break
      sleep 6; waited=$((waited+6))
    done
    # 发散判据:峰值 pos_err
    peak=$(grep -oE "pos_err=[0-9.]+" "$NMPC" 2>/dev/null | grep -oE "[0-9.]+" | sort -rn | head -1)
    status=ok
    awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
    [ "$aw" -lt "$NEED_AW" ] && status=TIMEOUT_${aw}
    echo "$mode $rep $nstamp $status" >> "$MANIFEST"
    echo "[ab] mode=$mode rep=$rep stamp=$nstamp aw=$aw peak=$peak -> $status"
    cleanup
  done
done
echo "[ab] done. manifest: $MANIFEST"
cat "$MANIFEST"
