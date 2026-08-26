#!/bin/bash
# 【2026-08-26 去先验改造】本脚本原先顺带设置的载荷质量先验环境变量
# (GRIP_GEOM_MP_PRIOR / GRIP_GEOM_MP_FLOOR / MHE_CONFIRM_PRIOR 等)已删除:
# 对应的节点参数不复存在,留着只会静默失效并误导读者。载荷质量信息现在只有
# grip_payload_envelope 一条(机架规格包线上界,run_gripper_headless.sh 默认 0.5)。
# 本脚本自身的研究主题不受影响。
# B.4 学习+强闭环合流矩阵(2026-07-15):{M0规则, θ*学习}×质量×偏心×n,全部在
# **online 无真值几何**配置下(NMPC_GEOM_SOURCE=online + floor=0.15),问"θ* 在 B.2
# 训练(有真值几何)之外、放到强闭环无真值配置里,对 M0 的收益是否保持"。
# M0:schedule=[-4,0,0,0] + 固定确认阈值(alpha=-1→1.5N,M0 规则)。
# θ*:schedule=α版θ* + α 无真值确认阈值(confirm_thresh_alpha=θ*[4],prior=测试质量,
#     即"操作员知道本次预期载荷",无真值 mass)。聚焦 attach→lift(不含 drop)。
# 复用 run_gripper_headless.sh,从 launch 输出锁定确切日志 stamp,支持断点续。
#
# 用法:  MASSES="0.2 0.3" ECCS="0.10" REPS=5 [MANIFEST=续跑] \
#         bash src/scripts/gripper/run_b4_matrix.sh
# 长跑须 setsid。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.10}"
METHODS="${METHODS:-M0 thetastar}"
REPS="${REPS:-5}"
THETA_STAR="-4.8038,-1.2080,0.4930,-0.9602,0.9875"
ALPHA_STAR="0.9875"
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="${MANIFEST:-$RUNDIR/b4_matrix_${STAMP}.txt}"
[ -f "$MANIFEST" ] || echo "# B.4 matrix $STAMP : method mass ecc rep stamp status" > "$MANIFEST"
echo "[b4] manifest: $MANIFEST"

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
trap 'echo "[b4] interrupted"; cleanup; exit 130' INT TERM

for method in $METHODS; do for mass in $MASSES; do for ecc in $ECCS; do for rep in $(seq 1 "$REPS"); do
  if grep -qE "^$method $mass $ecc $rep .* ok\$" "$MANIFEST" 2>/dev/null; then
    echo "[b4] skip $method $mass $ecc $rep (ok)"; continue
  fi
  # 方法→θ + 确认阈值配置
  if [ "$method" = thetastar ]; then
    TH="[$THETA_STAR]"; ALPHA="$ALPHA_STAR"; PRIOR="$mass"
  else
    TH="[-4.0,0.0,0.0,0.0]"; ALPHA="-1.0"; PRIOR="0.3"
  fi
  echo "[b4] === method=$method mass=$mass ecc=$ecc rep=$rep ==="
  LAUNCH="$RUNDIR/b4_launch_${STAMP}_${method}_${mass}_${ecc}_${rep}.log"
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    NMPC_GEOM_SOURCE=online \
    MHE_SCHEDULE_THETA="$TH" MHE_CONFIRM_ALPHA="$ALPHA" \
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
  [ -z "$peak" ] && peak=99
  status=ok
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && status=TIMEOUT_${aw}
  echo "$method $mass $ecc $rep $nstamp $status" >> "$MANIFEST"
  echo "[b4] $method $mass $ecc $rep stamp=$nstamp aw=$aw peak=$peak -> $status"
  cleanup
done; done; done; done
echo "[b4] done. manifest: $MANIFEST"
