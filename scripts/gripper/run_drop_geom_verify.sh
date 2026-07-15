#!/bin/bash
# B.5 drop 几何释放修复验收批(2026-07-15):同配置跑 N 轮 attach→figure8(2.75圈)
# →第三左端 drop→恢复,逐帧窗口覆盖全程,供 verify_drop_geom_release.py 出 7 项指标。
# 通过标准:3/3 轮 drop 后几何立即归零、m_est 回到 2.064±0.03、原 5-8cm 持续
# 高度偏差消失,且带载段质量误差/轨迹性能不退化。
# 用法: REPS=3 bash src/scripts/gripper/run_drop_geom_verify.sh

WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"
REPS="${REPS:-3}"; STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/drop_geom_verify_${STAMP}.txt"
echo "# drop 几何释放验收 $STAMP : rep nmpc_stamp status" > "$MAN"
echo "[verify] manifest: $MAN"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(pgrep -f "$pat" 2>/dev/null); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 3
}
trap 'echo "[verify] interrupted"; cleanup; exit 130' INT TERM

for rep in $(seq 1 "$REPS"); do
  echo "[verify] === rep=$rep ==="
  LAUNCH="$RUNDIR/drop_geom_launch_${STAMP}_${rep}.log"
  GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=online \
    MHE_SCHEDULE_THETA="[-4.8038,-1.2080,0.4930,-0.9602,0.9875]" \
    MHE_CONFIRM_ALPHA=0.9875 MHE_CONFIRM_PRIOR=0.3 \
    GRIP_DYNAMIC=true GRIP_DROP_AT_TIP=true GRIP_DROP_AFTER=60.0 \
    ATTACH_WINDOW_SEC=130.0 \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1
  NMPC=""; for k in $(seq 1 12); do
    NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
    [ -n "$NMPC" ] && break; sleep 5
  done
  nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
  # 等 drop + 恢复(drop@~86s,再留 25s 看 m_est 自行收敛与 z 回归)
  waited=0; ok=0
  while [ "$waited" -lt 220 ]; do
    grep -aq "DROP: released" "$NMPC" 2>/dev/null && { sleep 28; ok=1; break; }
    sleep 6; waited=$((waited+6))
  done
  [ "$ok" -eq 1 ] && st=ok || st=NO_DROP
  echo "$rep $nstamp $st" >> "$MAN"
  echo "[verify] rep=$rep stamp=$nstamp -> $st"
  cleanup
done
echo "[verify] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/verify_drop_geom_release.py \\"
grep -vE '^#' "$MAN" | awk -v d="$RUNDIR" '$3=="ok"{printf "  %s/grip_nmpc_%s.log \\\n", d, $2}'
