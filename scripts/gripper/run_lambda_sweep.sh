#!/bin/bash
# 【2026-08-26 去先验改造】本脚本原先顺带设置的载荷质量先验环境变量
# (GRIP_GEOM_MP_PRIOR / GRIP_GEOM_MP_FLOOR / MHE_CONFIRM_PRIOR 等)已删除:
# 对应的节点参数不复存在,留着只会静默失效并误导读者。载荷质量信息现在只有
# grip_payload_envelope 一条(机架规格包线上界,run_gripper_headless.sh 默认 0.5)。
# 本脚本自身的研究主题不受影响。
# 遗忘因子 λ 的 SITL 扫描(2026-08-24)。四臂,唯一差异 = 权重调度方式:
#   event   : 事件触发降权(M0),λ=1        —— 需要外部信号
#   fixed   : 无事件、无遗忘,λ=1           —— 真正的"什么都不做"基线
#   lam0.8  : 无事件,遗忘 λ=0.8            —— 零信号
#   lam0.7  : 无事件,遗忘 λ=0.7            —— 零信号
# 其余全部对齐已验证配置:MHE 几何耦合、NMPC 几何 legacy、先验 0.3、event 释放。
# λ 只改运行时权重(cost_set),**不改模型** → 四臂共用同一份 codegen,不必重编。
#
# 用法: REPS=2 bash src/scripts/gripper/run_lambda_sweep.sh
#      (长跑 setsid 脱离会话)
# 聚合: python3 src/scripts/gripper/aggregate_mass_truth.py <resid 子目录>

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

exec 9>"/tmp/lambda_sweep.lock"
if ! flock -n 9; then echo "[lam] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-2}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.05}"
DROP_AFTER="${DROP_AFTER:-45.0}"; RECOVER_SEC="${RECOVER_SEC:-25}"
TIMEOUT="${TIMEOUT:-260}"
ARMS="${ARMS:-event fixed lam0.8 lam0.7}"
STAMP=$(date +%Y%m%d_%H%M%S)
RESID_ROOT="$RUNDIR/lambda_$STAMP"
MANIFEST="$RUNDIR/lambda_sweep_${STAMP}.txt"
mkdir -p "$RESID_ROOT"
{
  echo "# 遗忘因子 λ SITL 扫描 $STAMP"
  echo "# 列: arm rep nmpc_stamp status peak_pos_err resid_dir"
  echo "# 配置: mass=$MASS ecc=$ECC drop_after=$DROP_AFTER reps=$REPS"
  echo "# 对齐: MHE_GEOM_COUPLED=1, NMPC几何=legacy, prior=$MASS, geom_release=event"
  echo "# 唯一差异: 权重调度(event / fixed / 遗忘λ)"
} > "$MANIFEST"
echo "[lam] manifest: $MANIFEST"

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
trap 'echo "[lam] interrupted"; cleanup; exit 130' INT TERM

run_one() {
  local arm="$1" rep="$2" trig lam
  case "$arm" in
    event)  trig=true;  lam=1.0 ;;
    fixed)  trig=false; lam=1.0 ;;
    lam*)   trig=false; lam="${arm#lam}" ;;
  esac
  local rdir="$RESID_ROOT/${arm}_rep${rep}"; mkdir -p "$rdir"
  local launch="$RUNDIR/lam_launch_${STAMP}_${arm}_${rep}.log"
  echo "[lam] === arm=$arm rep=$rep (event_trigger=$trig, lambda=$lam) ==="
  MHE_GEOM_COUPLED=1 MHE_LAMBDA=$lam MHE_EVENT_TRIGGER=$trig \
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
    echo "$arm $rep - LAUNCH_FAIL - $rdir" >> "$MANIFEST"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")
  local waited=0 dropped=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    if grep -q "DROP: released gripper" "$nmpc" 2>/dev/null; then dropped=1; break; fi
    sleep 6; waited=$((waited+6))
  done
  local status=ok
  if [ "$dropped" -eq 1 ]; then sleep "$RECOVER_SEC"; else status=NO_DROP; fi
  local peak
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $nstamp $status $peak $rdir" >> "$MANIFEST"
  echo "[lam] $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

# 每轮把臂的顺序轮转一格(抵消机器热身/漂移)。用 python 生成,别在 shell 里
# 手搓位置参数轮转——那种写法极易漏臂或重复,且不易自测。
for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
import sys
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[lam] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[lam] done. manifest: $MANIFEST"
