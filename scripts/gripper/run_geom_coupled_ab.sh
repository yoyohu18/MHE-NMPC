#!/bin/bash
# 【2026-08-26 去先验改造】本脚本原先顺带设置的载荷质量先验环境变量
# (GRIP_GEOM_MP_PRIOR / GRIP_GEOM_MP_FLOOR / MHE_CONFIRM_PRIOR 等)已删除:
# 对应的节点参数不复存在,留着只会静默失效并误导读者。载荷质量信息现在只有
# grip_payload_envelope 一条(机架规格包线上界,run_gripper_headless.sh 默认 0.5)。
# 本脚本自身的研究主题不受影响。
# 几何-质量耦合 A/B(2026-08-24)。唯一差异 = MHE_GEOM_COUPLED(0=legacy:dJ/c_xy
# 是窗外算好的常参数,∂(J,c)/∂m≡0;1=coupled:J(m)/c(m) 在模型内由被估质量现算)。
# 其余全部对齐:同载荷/偏心/几何先验/θ/确认阈值/轨迹/drop 时刻。
#
# 每轮是完整的 attach → 飞行 → drop → 回收,配对交错(奇偶轮换先后顺序,抵消
# 机器热身/漂移),每轮独立 RESID 子目录,便于按目录聚合。
#
# 用法: REPS=3 bash src/scripts/gripper/run_geom_coupled_ab.sh
#       (长跑用 setsid 脱离会话:setsid bash ... > log 2>&1 < /dev/null &)
# 聚合: python3 src/scripts/gripper/aggregate_mass_truth.py <resid 子目录>

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

# 单实例锁(记忆 sitl-batch-pitfalls:双批碰撞会互相 kill、两批数据全废)。
# 9>&- 关掉子进程继承的 fd,否则孤儿持锁、ps 里看不见却抢不到锁。
exec 9>"/tmp/geom_coupled_ab.lock"
if ! flock -n 9; then
  echo "[ab] 已有实例在跑(/tmp/geom_coupled_ab.lock 被持有),退出。"; exit 1
fi

REPS="${REPS:-3}"
MASS="${MASS:-0.3}"
ECC="${ECC:-0.05}"
DROP_AFTER="${DROP_AFTER:-45.0}"     # lift 完成后再悬停这么久才 drop
RECOVER_SEC="${RECOVER_SEC:-25}"     # drop 之后再录这么久,用来量恢复
TIMEOUT="${TIMEOUT:-260}"            # 单轮等待上限(秒)
STAMP=$(date +%Y%m%d_%H%M%S)
RESID_ROOT="$RUNDIR/geom_ab_$STAMP"
MANIFEST="$RUNDIR/geom_coupled_ab_${STAMP}.txt"
mkdir -p "$RESID_ROOT"
{
  echo "# 几何-质量耦合 A/B  $STAMP"
  echo "# 列: mode rep nmpc_stamp status peak_pos_err resid_dir"
  echo "# 配置: mass=$MASS ecc=$ECC drop_after=$DROP_AFTER recover=$RECOVER_SEC reps=$REPS"
  echo "# 对齐: geom_source=online, geom_mp_prior=$MASS, theta=M0, c_xy_est=on"
  echo "# 唯一差异: MHE_GEOM_COUPLED"
} > "$MANIFEST"
echo "[ab] manifest: $MANIFEST"
echo "[ab] resid root: $RESID_ROOT"

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

run_one() {
  local mode="$1" rep="$2" cpl
  [ "$mode" = coupled ] && cpl=1 || cpl=0
  local rdir="$RESID_ROOT/${mode}_rep${rep}"
  mkdir -p "$rdir"
  local launch="$RUNDIR/geomab_launch_${STAMP}_${mode}_${rep}.log"
  echo "[ab] === mode=$mode rep=$rep (MHE_GEOM_COUPLED=$cpl) ==="
  MHE_GEOM_COUPLED=$cpl \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC USE_MHE=true MHE_C_XY_EST=true \
    NMPC_GEOM_SOURCE=online \
    GRIP_DROP_AFTER=$DROP_AFTER RESID_LOG_DIR="$rdir" \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1

  local nmpc="" k
  for k in $(seq 1 12); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "$mode $rep - LAUNCH_FAIL - $rdir" >> "$MANIFEST"
    echo "[ab] $mode rep$rep LAUNCH_FAIL"; cleanup; return
  fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  # 等 drop 发生(NMPC 日志里的 'DROP: released gripper'),再多录 RECOVER_SEC 秒
  local waited=0 dropped=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    if grep -q "DROP: released gripper" "$nmpc" 2>/dev/null; then dropped=1; break; fi
    sleep 6; waited=$((waited+6))
  done
  local status=ok
  if [ "$dropped" -eq 1 ]; then
    sleep "$RECOVER_SEC"
  else
    status=NO_DROP
  fi
  local peak
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$mode $rep $nstamp $status $peak $rdir" >> "$MANIFEST"
  echo "[ab] $mode rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  # 配对交错:奇数轮 legacy 先,偶数轮 coupled 先
  if [ $((rep % 2)) -eq 1 ]; then order="legacy coupled"; else order="coupled legacy"; fi
  for mode in $order; do run_one "$mode" "$rep"; done
done
echo "[ab] done. manifest: $MANIFEST"
