#!/bin/bash
# B.3 主实验网格 n≥5(2026-07-14):方法{truth,online}×质量×偏心×n 全格配对,量化
# "NMPC 吃在线 c_xy 估计"vs"吃 attach 真值几何"的控制层暂态(pos_err 峰/恢复),
# 证在线估计版≈真值版、消 attach 真值依赖不掉性能。attach→lift→稳飞(不含 drop,
# drop 全流程另已单验;这里聚焦带载几何 A/B 的统计)。复用 run_gripper_headless.sh,
# 从其 launch 输出锁定每轮确切 nmpc 日志 stamp(避 ls -t 抓旧日志)。可断点续:
# manifest 已有的 (mass,ecc,mode,rep) 跳过。
#
# 用法:  MASSES="0.2 0.3" ECCS="0.05 0.10" MODES="truth online" REPS=5 \
#         [MANIFEST=已有manifest续跑] bash src/scripts/gripper/run_geom_grid.sh
# 长跑须 setsid 脱离会话。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.05 0.10}"
MODES="${MODES:-truth online}"
REPS="${REPS:-5}"
NEED_AW="${NEED_AW:-110}"
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="${MANIFEST:-$RUNDIR/geom_grid_${STAMP}.txt}"
[ -f "$MANIFEST" ] || echo "# B.3 geom grid $STAMP : mass ecc mode rep nmpc_stamp status" > "$MANIFEST"
echo "[grid] manifest: $MANIFEST"

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
trap 'echo "[grid] interrupted"; cleanup; exit 130' INT TERM

done_cnt=0; total=0
for mass in $MASSES; do for ecc in $ECCS; do for mode in $MODES; do for rep in $(seq 1 "$REPS"); do
  total=$((total+1))
  # 断点续:已有该 (mass,ecc,mode,rep) 的 ok 行则跳过
  if grep -qE "^$mass $ecc $mode $rep .* ok\$" "$MANIFEST" 2>/dev/null; then
    echo "[grid] skip $mass $ecc $mode $rep (already ok)"; done_cnt=$((done_cnt+1)); continue
  fi
  echo "[grid] === mass=$mass ecc=$ecc mode=$mode rep=$rep ==="
  LAUNCH="$RUNDIR/geom_grid_launch_${STAMP}_${mass}_${ecc}_${mode}_${rep}.log"
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=$mode \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1
  NMPC=""; for k in $(seq 1 12); do
    NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
    [ -n "$NMPC" ] && break; sleep 5
  done
  nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
  waited=0; aw=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    aw=$(grep -c "attach-window" "$NMPC" 2>/dev/null)
    [ -n "$aw" ] && [ "$aw" -ge "$NEED_AW" ] && break
    sleep 6; waited=$((waited+6))
  done
  peak=$(grep -oE "pos_err=[0-9.]+" "$NMPC" 2>/dev/null | grep -oE "[0-9.]+" | sort -rn | head -1)
  status=ok
  [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && status=TIMEOUT_${aw}
  echo "$mass $ecc $mode $rep $nstamp $status" >> "$MANIFEST"
  echo "[grid] $mass $ecc $mode $rep stamp=$nstamp aw=$aw peak=$peak -> $status ($done_cnt/$total done)"
  [ "$status" = ok ] && done_cnt=$((done_cnt+1))
  cleanup
done; done; done; done
echo "[grid] done: $done_cnt ok / $total cells. manifest: $MANIFEST"
