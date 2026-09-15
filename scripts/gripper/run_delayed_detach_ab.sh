#!/bin/bash
# Command/detachment mismatch fault injection.
#
# The controller issues its normal release command, while proximity_gripper_node
# holds the physical DetachableJoint for DETACH_DELAY seconds. Arm A consumes the
# command as detachment evidence; arm B keeps the final eventless interface.
#
# Default experiment: five randomized A/B pairs at W5 (0.15 kg, 0.10 m,
# 2 m/s), with a 4 s physical-detachment delay. W5 remains a dynamic
# figure-eight but avoids confounding the injected release fault with W3's
# known pre-release tracking boundary. Every attempt is retained;
# pre-command validity is recorded rather than silently replaced.
#
# Smoke example:
#   PAIRS=1 bash src/scripts/gripper/run_delayed_detach_ab.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
STAMP=$(date +%Y%m%d_%H%M%S)
MAN=${MAN_FILE:-$RES/delayed_detach_ab_${STAMP}.csv}
LOCK=/tmp/mainline_ab.lock
PAIRS=${PAIRS:-5}
DETACH_DELAY=${DETACH_DELAY:-4.0}
POST_SEC=${POST_SEC:-12}
WORKPOINT=${WORKPOINT:-W5}
ONLY_ARM=${ONLY_ARM:-}
CHECK=$WS/src/scripts/gripper/check_run_valid.py

case "$WORKPOINT" in
  W3) WORK_ENV="GRIP_DROP_AFTER=55.0 GRIP_DYN_W=0.283"; SPEED=4 ;;
  W5) WORK_ENV="GRIP_DROP_AFTER=30.0 GRIP_DYN_W=0.1415"; SPEED=2 ;;
  *) echo "[delay-ab] WORKPOINT must be W3 or W5"; exit 2 ;;
esac

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[delay-ab] refuse: non-workspace PX4/Gazebo process detected"
  echo "$foreign" | cut -c1-100
  exit 2
fi

exec 9>"$LOCK"
flock -n 9 || { echo "[delay-ab] another batch holds $LOCK"; exit 1; }

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
trap 'echo "[delay-ab] interrupted"; cleanup; exit 130' INT TERM

echo "idx,pair,arm,stamp,delay_sec,started,finished,command_seen,detach_seen,completion_seen,validity" > "$MAN"
PROV="${MAN%.csv}.provenance.txt"
{
  echo "batch_started=$(date +%FT%T%z)"
  echo "purpose=release-command/physical-detachment mismatch"
  echo "condition=$WORKPOINT: payload=0.15kg ecc=0.10m peak_speed=${SPEED}m/s"
  echo "detach_delay_sec=$DETACH_DELAY"
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
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 $WORK_ENV \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=0.15 GRIP_DETACH_DELAY_SEC=$DETACH_DELAY"

arm_env() {
  if [ "$1" = A ]; then
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=false NMPC_GEOM_SOURCE=online NMPC_GEOM_RELEASE_MODE=event MHE_PAYLOAD_LOST_WATCH=0"
  else
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0"
  fi
}

run_one() {
  local pair="$1" arm="$2" idx="$3" t0 N stamp P S validity
  t0=$(date +%FT%T)
  echo "[delay-ab] === pair=$pair arm=$arm idx=$idx ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.delay_ab_before

  setsid nohup env $COMMON_ENV $(arm_env "$arm") \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/delay_ab_current.log 2>&1 9>&- &

  N=""
  for _ in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.delay_ab_before || echo "$f"
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
    grep -qE "DROP command issued|DROP: released gripper" "$N" 2>/dev/null && break
    sleep 5
  done
  local cmd=0 det=0 done=0
  grep -qE "DROP command issued|DROP: released gripper" "$N" 2>/dev/null && cmd=1

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
  echo "[delay-ab] stamp=$stamp command=$cmd physical_detach=$det complete=$done validity=$validity"
}

idx=0
for pair in $(seq 1 "$PAIRS"); do
  if [ -n "$ONLY_ARM" ]; then
    order="$ONLY_ARM"
  elif [ $((pair % 2)) -eq 1 ]; then
    order="A B"
  else
    order="B A"
  fi
  for arm in $order; do
    idx=$((idx + 1))
    run_one "$pair" "$arm" "$idx"
  done
done

cleanup
echo "[delay-ab] complete: $MAN"
