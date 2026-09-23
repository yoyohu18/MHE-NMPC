#!/bin/bash
# Matched command-armed confirmation baseline (2026-09-14).
#
# Arm B = final eventless interface (identical to run_delayed_detach_ab.sh arm B).
# Arm C = same MHE/NMPC config, plus MHE release_arm_on_command=true: eventless
#         release paths are suppressed; the release command (/gripper/enable
#         True->False) opens a window and a thrust-proxy quantity must persist
#         (payload_lost watchdog slow-layer constants, not tuned here).
# FAULT=delayed      : command issued, physical joint held DETACH_DELAY s.
# FAULT=uncommanded  : payload physically released at the same trajectory phase,
#                      no release command reaches controller or estimator.
#
# Smoke: PAIRS=1 FAULT=delayed bash src/scripts/gripper/run_matched_cmd_armed_ab.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
STAMP=$(date +%Y%m%d_%H%M%S)
FAULT=${FAULT:-delayed}
MAN=${MAN_FILE:-$RES/cmdarm_ab_${FAULT}_${STAMP}.csv}
LOCK=/tmp/mainline_ab.lock
PAIRS=${PAIRS:-5}
DETACH_DELAY=${DETACH_DELAY:-4.0}
[ "$FAULT" != delayed ] && DETACH_DELAY=0.0
ARMS=${ARMS:-B C}
POST_SEC=${POST_SEC:-30}
WORKPOINT=${WORKPOINT:-W5}
ONLY_ARM=${ONLY_ARM:-}
CHECK=$WS/src/scripts/gripper/check_run_valid.py

case "$WORKPOINT" in
  W3) WORK_ENV="GRIP_DROP_AFTER=55.0 GRIP_DYN_W=0.283"; SPEED=4 ;;
  W5) WORK_ENV="GRIP_DROP_AFTER=30.0 GRIP_DYN_W=0.1415"; SPEED=2 ;;
  *) echo "[cmdarm-ab] WORKPOINT must be W3 or W5"; exit 2 ;;
esac

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[cmdarm-ab] refuse: non-workspace PX4/Gazebo process detected"
  echo "$foreign" | cut -c1-100
  exit 2
fi

exec 9>"$LOCK"
flock -n 9 || { echo "[cmdarm-ab] another batch holds $LOCK"; exit 1; }

cleanup() {
  for pat in "run_gripper[_]headless.sh" "px4_sitl_default/bin/px4" \
             "make px4_sitl" "gz sim" "gz_bridge" \
             "transport13/gz-transport-topic" "ros_gz_bridge" \
             "mavros/mavros_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "MicroXRCEAgent" "gcs_heartbeat"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do
      kill -9 "$pp" 2>/dev/null || true
    done
  done
  sleep 8
}
trap 'echo "[cmdarm-ab] interrupted"; cleanup; exit 130' INT TERM

echo "idx,pair,arm,stamp,delay_sec,started,finished,command_seen,detach_seen,completion_seen,validity" > "$MAN"
PROV="${MAN%.csv}.provenance.txt"
{
  echo "batch_started=$(date +%FT%T%z)"
  echo "purpose=matched command-armed baseline, fault=$FAULT"
  echo "condition=$WORKPOINT: payload=0.15kg ecc=0.10m peak_speed=${SPEED}m/s"
  echo "detach_delay_sec=$DETACH_DELAY"
  echo "arms=$ARMS motor_noise=${MOTOR_NOISE:-0.0}"
  echo "git_head=$(cd "$WS/src" && git rev-parse HEAD 2>/dev/null)"
  echo "dirty_list<<EOF"
  (cd "$WS/src" && git status --porcelain 2>/dev/null)
  echo "EOF"
  sha256sum \
    "$WS/src/offboard_test_acados/offboard_test_acados/gripper/proximity_gripper_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/acados_nmpc_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/mhe_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/payload_estimate.py"
} > "$PROV"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 $WORK_ENV \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=0.15 GRIP_DETACH_DELAY_SEC=$DETACH_DELAY \
MHE_RESID_CONFIRM=${MHE_RESID_CONFIRM:-true} \
GRIP_UNCOMMANDED_LOSS=$([ "$FAULT" = uncommanded ] && echo true || echo false)"

arm_env() {
  # FAULT=thrustmap: arms are estimator-side thrust-map gains (B interface).
  case "$1" in
    C) echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 MHE_RELEASE_ARM_ON_COMMAND=true" ;;
    B) echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 MHE_RELEASE_ARM_ON_COMMAND=false" ;;
    G*) echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 MHE_RELEASE_ARM_ON_COMMAND=false MHE_THRUST_FAULT_GAIN=${1#G} MHE_MOTOR_NOISE=${MOTOR_NOISE:-0.0}" ;;
  esac
}

run_one() {
  local pair="$1" arm="$2" idx="$3" t0 N stamp P S validity
  t0=$(date +%FT%T)
  echo "[cmdarm-ab] === pair=$pair arm=$arm idx=$idx ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.cmdarm_ab_before

  setsid nohup env $COMMON_ENV $(arm_env "$arm") \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/cmdarm_ab_current.log 2>&1 9>&- &

  N=""
  for _ in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.cmdarm_ab_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "$idx,$pair,$arm,NA,$DETACH_DELAY,$t0,$(date +%FT%T),0,0,0,invalid:no-log" >> "$MAN"
    return
  fi
  stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  P="$RES/grip_proximity_${stamp}.log"
  S="$RES/grip_state_${stamp}.log"

  for _ in $(seq 1 90); do
    grep -qE "DROP command issued|DROP: released gripper|UNCOMMANDED LOSS injected" "$N" 2>/dev/null && break
    sleep 5
  done
  local cmd=0 det=0 done=0
  grep -qE "DROP command issued|DROP: released gripper|UNCOMMANDED LOSS injected" "$N" 2>/dev/null && cmd=1

  if [ "$cmd" -eq 1 ]; then
    for _ in $(seq 1 20); do
      grep -q -- "-> DETACH" "$P" 2>/dev/null && break
      sleep 1
    done
    grep -q -- "-> DETACH" "$P" 2>/dev/null && det=1
    for _ in $(seq 1 24); do
      grep -qE "DROP complete|DROP: released gripper" "$N" 2>/dev/null && break
      sleep 1
    done
    grep -qE "DROP complete|DROP: released gripper" "$N" 2>/dev/null && done=1
    sleep "$POST_SEC"
  fi

  validity=$(python3 "$CHECK" "$N" 2>&1 || true)
  [ -s "$S" ] || det=0
  echo "$idx,$pair,$arm,$stamp,$DETACH_DELAY,$t0,$(date +%FT%T),$cmd,$det,$done,\"$validity\"" >> "$MAN"
  echo "[cmdarm-ab] stamp=$stamp command=$cmd physical_detach=$det complete=$done validity=$validity"
}

idx=0
for pair in $(seq 1 "$PAIRS"); do
  if [ -n "$ONLY_ARM" ]; then
    order="$ONLY_ARM"
  elif [ $((pair % 2)) -eq 1 ]; then
    order="$ARMS"
  else
    order=$(echo $ARMS | tr ' ' '\n' | tac | tr '\n' ' ')
  fi
  for arm in $order; do
    idx=$((idx + 1))
    run_one "$pair" "$arm" "$idx"
  done
done

cleanup
echo "[cmdarm-ab] complete: $MAN"
