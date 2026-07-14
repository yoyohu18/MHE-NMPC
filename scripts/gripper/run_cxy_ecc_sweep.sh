#!/bin/bash
# B.3 Phase1 偏心扫格(2026-07-14):固定载荷、扫多个横向偏心,每点跑 gripper attach
# 到 c_xy 收敛,记录在线 c_xy 估计 vs attach 真值,出误差表钉死 Phase1。
# 复用 run_gripper_headless.sh(它自带 cleanup 前导 + 构建 + 起栈,MHE_C_XY_EST 透传)。
# 每点:起栈 → 等 grip_mhe 里 [c_xy_est] 攒够(attach+收敛)→ 记最后一条 → 下一点
# (下一次 run_gripper_headless 的 cleanup 会收上一栈)。最后统一收栈。
#
# 用法:  ECCS="0.05 0.08 0.10 0.12" GRIP_PAYLOAD_KG=0.3 \
#         bash src/scripts/gripper/run_cxy_ecc_sweep.sh
# 长跑须 setsid 脱离会话(见记忆 nosignal-ablation 双批碰撞教训)。
# 不用 set -e/-u(ROS setup 不 -u 安全;靠显式判断处理单点失败)。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
ECCS="${ECCS:-0.05 0.08 0.10 0.12}"
PAYLOAD="${GRIP_PAYLOAD_KG:-0.3}"
NEED="${NEED:-20}"          # 每点等够多少条 [c_xy_est](attach 后 EMA 收敛)
TIMEOUT="${TIMEOUT:-220}"   # 单点等收敛上限秒
STAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY="$RUNDIR/cxy_ecc_sweep_${STAMP}.txt"
echo "# B.3 Phase1 c_xy 偏心扫格 $STAMP  payload=${PAYLOAD}kg" > "$SUMMARY"
echo "# ecc_y  <最后一条 [c_xy_est] 行(含 est 与 truth)>" >> "$SUMMARY"

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

trap 'echo "[sweep] interrupted"; cleanup; exit 130' INT TERM

for ecc in $ECCS; do
  echo "[sweep] === ecc_y=$ecc payload=$PAYLOAD ==="
  # run_gripper_headless.sh 自带 cleanup 前导 + 构建 + 起栈;返回后栈在后台跑
  GRIP_PAYLOAD_KG=$PAYLOAD GRIP_ECC_Y=$ecc MHE_C_XY_EST=true USE_MHE=true \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > "$RUNDIR/cxy_sweep_launch_${STAMP}_${ecc}.log" 2>&1
  # 等本点 grip_mhe 收敛
  waited=0; got=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    MHE=$(ls -t "$RUNDIR"/grip_mhe_2026*.log 2>/dev/null | head -1)
    if [ -n "$MHE" ]; then
      got=$(grep -c "\[c_xy_est\]" "$MHE" 2>/dev/null)
      [ "$got" -ge "$NEED" ] && break
    fi
    sleep 6; waited=$((waited+6))
  done
  MHE=$(ls -t "$RUNDIR"/grip_mhe_2026*.log 2>/dev/null | head -1)
  last=$(grep "\[c_xy_est\]" "$MHE" 2>/dev/null | tail -1 | sed 's/.*\[c_xy_est\]/[c_xy_est]/')
  if [ "$got" -ge "$NEED" ]; then
    echo "$ecc  ok  $last" >> "$SUMMARY"
    echo "[sweep] ecc=$ecc ok ($got lines): $last"
  else
    echo "$ecc  TIMEOUT  got=$got  $last" >> "$SUMMARY"
    echo "[sweep] !! ecc=$ecc timeout (got=$got)"
  fi
done

cleanup
echo "[sweep] done. summary: $SUMMARY"
cat "$SUMMARY"
