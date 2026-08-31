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
# 【2026-08-31 更新到新条件】
#   · 工作点从 ecc=0.05 改到 **ecc=0.10**(主线 viz/headless 默认),这批要回答的是
#     "主体能不能往 coupled 切",不是复现 08-24 那个已中断的批次(它 reps=7 只留下
#     一行表头)。两批不可比,这是有意的。
#   · attach 偏心现在**有上限**:proximity 的 r_xy = ECC + GRIP_ATTACH_TOL(0.03)。
#     08-31 定位到坠机根因是偏心过大致配平力矩吃满 roll 权限(记忆
#     grip-attach-eccentricity-roll-saturation),那是 legacy/coupled 共有的失效模式;
#     卡住它之后两臂才可比 —— 这也正是现在值得跑这个 A/B 的原因。
#   · 指标从"只记 peak_pos_err"扩到 7 列(见 extract_ab_metrics.py 里的理由):
#     **status 必须先看**,坠机会把 solve 耗时/失败数/估计误差全部染色。
#   · NMPC 侧**不开** coupled:NMPC_GEOM_COUPLED 的可用边界是 r_y≲0.05m,
#     而本批 ecc=0.10 远在禁区外。推荐组合始终是 MHE coupled + NMPC 几何走 online。
#
# 用法: REPS=8 bash src/scripts/gripper/run_geom_coupled_ab.sh
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

REPS="${REPS:-8}"
MASS="${MASS:-0.3}"
ECC="${ECC:-0.10}"                   # 主线工作点(08-31 起,见头部)
ATTACH_TOL="${ATTACH_TOL:-0.03}"     # attach 偏心上限 = ECC + 本值
# 【2026-08-31 第二批】figure8 机动。DYNAMIC=false 时是悬停带载(第一批就是,
# 因为 run_gripper_headless.sh 的 GRIP_DYNAMIC 默认 false 而这里没传)。
# ⚠️ headless 的 8 字默认是 r=0.8/w=0.25 —— 峰值速度只有 0.28m/s、包络 1.6×0.8m,
# 那不算机动。这里显式取 **viz 主线那一套**(r=5.0/w=0.283/ramp=3.0/dz=0.8,
# 峰值速度 2.0m/s),否则两个脚本"跑的 figure8"根本不是一回事。
DYNAMIC="${DYNAMIC:-true}"
DYN_R="${DYN_R:-5.0}"; DYN_W="${DYN_W:-0.283}"
DYN_RAMP="${DYN_RAMP:-3.0}"; DYN_DZ="${DYN_DZ:-0.8}"
DROP_AFTER="${DROP_AFTER:-45.0}"     # lift 完成后再悬停这么久才 drop
RECOVER_SEC="${RECOVER_SEC:-25}"     # drop 之后再录这么久,用来量恢复
TIMEOUT="${TIMEOUT:-260}"            # 单轮等待上限(秒)
STAMP=$(date +%Y%m%d_%H%M%S)
RESID_ROOT="$RUNDIR/geom_ab_$STAMP"
MANIFEST="$RUNDIR/geom_coupled_ab_${STAMP}.txt"
mkdir -p "$RESID_ROOT"
{
  echo "# 几何-质量耦合 A/B  $STAMP"
  echo "# 列: mode rep nmpc_stamp status peak_pos_err nmpc_failed attach_ecc m_err_p50 m_err_p90 solve_med traj n_samples resid_dir"
  echo "# 配置: mass=$MASS ecc=$ECC attach_tol=$ATTACH_TOL (r_xy 上限 $(python3 -c "print(f'{$ECC+$ATTACH_TOL:.3f}')")) drop_after=$DROP_AFTER recover=$RECOVER_SEC reps=$REPS"
  echo "# 轨迹: dynamic=$DYNAMIC r=$DYN_R w=$DYN_W ramp=$DYN_RAMP dz=$DYN_DZ (v_peak=$(python3 -c "import math;print(f'{math.sqrt(2)*$DYN_R*$DYN_W:.2f}')")m/s)"
  echo "# 对齐: geom_source=online(NMPC 侧不开 coupled), theta=M0, c_xy_est=on, envelope=0.5(headless 默认)"
  echo "# 唯一差异: MHE_GEOM_COUPLED"
  echo "# m_err_* = |误差|%,取 **DROP 前 20s** 稳态带载窗口(见 extract_ab_metrics.py);"
  echo "# traj 列标出该轮是 fig8 还是 hover —— 两种批次的数字不可混读;status=CRASH 行其余列不可用"
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
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC GRIP_ATTACH_TOL=$ATTACH_TOL \
    GRIP_DYNAMIC=$DYNAMIC GRIP_DYN_R=$DYN_R GRIP_DYN_W=$DYN_W \
    GRIP_DYN_RAMP=$DYN_RAMP GRIP_DYN_DZ=$DYN_DZ \
    USE_MHE=true MHE_C_XY_EST=true \
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
  [ "$dropped" -eq 1 ] && sleep "$RECOVER_SEC"
  # 指标一律由 extract_ab_metrics.py 统一口径地抽(status 也在里面判),
  # 避免这里和聚合脚本各写一套 grep 而口径悄悄分叉。
  local mhe="${nmpc/grip_nmpc_/grip_mhe_}"
  local metrics
  metrics=$(python3 "$WS/src/scripts/gripper/extract_ab_metrics.py" \
            "$nmpc" "$mhe" 2>/dev/null)
  [ -z "$metrics" ] && metrics="EXTRACT_FAIL 99 99 nan nan nan nan - 0"
  echo "$mode $rep $nstamp $metrics $rdir" >> "$MANIFEST"
  echo "[ab] $mode rep$rep stamp=$nstamp -> $metrics"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  # 配对交错:奇数轮 legacy 先,偶数轮 coupled 先
  if [ $((rep % 2)) -eq 1 ]; then order="legacy coupled"; else order="coupled legacy"; fi
  for mode in $order; do run_one "$mode" "$rep"; done
done
echo "[ab] done. manifest: $MANIFEST"
