#!/bin/bash
# 冒烟:phys_full 力矩 + 越界触发重锚,看离地后 m 还贴不贴下界 (2026-09-17)
#
# 背景见记忆 liftoff-lb-stuck-mechanisms:离地后 m 贴 m_min 有两种机理——
#   A 持续振荡 + command 力矩失配(开环回放:phys_full 治好 4/5)
#   B attach 撞击后落进 |s|>包线 的错误盆地(新增 reanchor_on_moment_overrange)
# 本脚本只看**吊起阶段 + 进 figure-8 后一圈**,不投放,飞满一圈即收栈。
#
# ⚠️ 这是冒烟不是 A/B:主线 stuck 基线率约 7%,n=6 时即使完全没修也期望只有
#    ~0.4 轮 stuck,"0/6 stuck" 不能当修复证据。要下结论须另立配对批次。
#
# 工况 = run_e1_dev.sh 的 COMMON_ENV(stuck 轮的出处),改动只有:
#   + MHE_TAU_SOURCE=phys_full MHE_MOTOR_AVG=1 MHE_REANCHOR_OVR=1
#   + GRIP_DROP_AFTER=0(不投放)、RELEASE_BASELINE=P、原始流落盘(便于回放)
#
# 用法: REPS=6 setsid bash src/scripts/gripper/run_pf_ovr_smoke.sh > /tmp/pf_ovr.log 2>&1 &
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[pf] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 9>"/tmp/pf_ovr_smoke.lock"
if ! flock -n 9; then echo "[pf] 已有实例在跑,退出。"; exit 1; fi

busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[pf] SITL 栈被占用,放弃(先确认是谁在用)"; exit 2; }

REPS="${REPS:-6}"
W="${W:-0.1415}"
# 一圈 = 2π/w;ramp 段也算进去再留 5s 余量
LAP=$(python3 -c "import math; print(int(2*math.pi/$W + 5))")
STAMP=$(date +%Y%m%d_%H%M%S)
BDIR="$RUNDIR/pf_ovr_smoke_$STAMP"; mkdir -p "$BDIR/resid"
MAN="$BDIR/manifest.txt"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DROP_AFTER=0.0 GRIP_DYN_W=$W \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=0.15 MHE_RESID_CONFIRM=true \
CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 \
GRIP_UNCOMMANDED_LOSS=false RELEASE_BASELINE=P \
MHE_TAU_SOURCE=phys_full MHE_MOTOR_AVG=1 MHE_REANCHOR_OVR=1"

{
echo "# phys_full + 越界重锚 冒烟  $STAMP  (REPS=$REPS, 每轮 figure-8 飞 ${LAP}s)"
echo "# env: $COMMON_ENV"
echo "# 列: rep nmpc_stamp status peak_pos_err"
} > "$MAN"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "transport13/gz-transport-topic" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[pf] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local rep="$1" rdir="$BDIR/resid/rep$1" launch="$BDIR/launch_rep$1.log"
  mkdir -p "$rdir"
  echo "[pf] === rep=$rep ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿
  env $COMMON_ENV RESID_LOG_DIR="$rdir" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[pf] rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$rep - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 300 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    grep -aq "Traceback" "$nmpc" 2>/dev/null && break
    sleep 5; waited=$((waited+5))
  done
  local status=ok
  if [ "$got" -eq 0 ]; then
    status=NODYN
    echo "[pf]   rep$rep 300s 内没进 figure-8"
  else
    echo "[pf]   已进 figure-8,再飞 ${LAP}s"
    sleep "$LAP"
  fi
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  local peak
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && [ "$status" = ok ] && status=DIVERGED
  echo "$rep $nstamp $status $peak" >> "$MAN"
  echo "[pf]   rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

for rep in $(seq 1 "$REPS"); do run_one "$rep"; done
echo "[pf] 批次完成 -> $MAN"
