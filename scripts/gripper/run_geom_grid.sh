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
# mode 维(C.1 Phase 2 扩,2026-07-23):truth/online=几何来源 A/B(原 B.3);
# **l1**=NMPC_CONTROL_MODE=l1(流派 B 对照:不估质量,L1 补 d_lumped+档位先验
# 缩增益,ω_c 默认 0.5,见 memory c1-l1-nmpc-baseline)。
MODES="${MODES:-truth online l1}"
REPS="${REPS:-5}"
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)

# 单实例锁(2026-07-23,同 run_b4_decoupled_verify.sh:双批并发会经 headless
# 前置清理互杀节点,当天实测两批全作废,教训第二次出现即上机制护栏)。
exec 9>"/tmp/geom_grid.lock"
if ! flock -n 9; then
  echo "[grid] 已有另一个实例在跑(锁 /tmp/geom_grid.lock 被持有),退出。"
  exit 1
fi

MANIFEST="${MANIFEST:-$RUNDIR/geom_grid_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  # 分组元数据落盘(承 07-20 发散 sweep 未落盘教训)
  {
    echo "# B.3/C.1 geom grid $STAMP : mass ecc mode rep nmpc_stamp status"
    echo "# 配置: modes=${MODES} masses=${MASSES} eccs=${ECCS} reps=${REPS}"
    echo "# 配置: geom_prior=headless默认(=GRIP_PAYLOAD_KG 档位值) floor=0.15 m_min=默认1.961"
    echo "# 配置: l1: omega_c=headless默认0.5 a=10 D2(b)档位先验缩增益"
    echo "# 判据: DIVERGED=pos_err峰>2.0m ; NEED_AW=${NEED_AW}"
  } > "$MANIFEST"
fi
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
  # mode → (几何来源, 控制模式) 映射:l1 走 control_mode,geom_source 无关
  # (l1 模式下 nmpc 侧几何前馈全被旁路,attach 真值仅评估)。
  GS=$mode; CM=mhe
  [ "$mode" = l1 ] && { GS=truth; CM=l1; }
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=$GS NMPC_CONTROL_MODE=$CM \
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
