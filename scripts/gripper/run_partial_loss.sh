#!/bin/bash
# Partial payload loss, four-box staircase (2026-09-15).
# Pre-registration: ~/ros2_ws_HJH/部分丢失四箱实验_预注册_20260915.md
#
# Arm L: proximity detaches box4/box3/box2 at 45/80/115 s after the first attach
#        and never re-grips them; the controller and estimator receive nothing.
# Arm N: identical schedule, markers only ("dry"), all four boxes stay attached.
# SPEED=2 (W5 w) or SPEED=4 (W3 w). ABBA pairs, every attempt kept.
#
# Smoke: PAIRS=1 SPEED=2 bash src/scripts/gripper/run_partial_loss.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
STAMP=$(date +%Y%m%d_%H%M%S)
SPEED=${SPEED:-2}
PAIRS=${PAIRS:-5}
POST_SEC=${POST_SEC:-30}
SCHED=${SCHED:-box4:45,box3:80,box2:115}
MAN=${MAN_FILE:-$RES/partial_loss_S${SPEED}_${STAMP}.csv}
LOCK=/tmp/mainline_ab.lock
CHECK_LAST=$(echo "$SCHED" | tr ',' '\n' | tail -1 | cut -d: -f2)

case "$SPEED" in
  2) DYN_W=0.1415 ;;
  4) DYN_W=0.283 ;;
  *) echo "[partial] SPEED must be 2 or 4"; exit 2 ;;
esac

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[partial] refuse: PX4/Gazebo process already running (clean the stack first)"
  echo "$foreign" | cut -c1-100
  exit 2
fi

exec 9>"$LOCK"
flock -n 9 || { echo "[partial] another batch holds $LOCK"; exit 1; }

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
trap 'echo "[partial] interrupted"; cleanup; exit 130' INT TERM

echo "idx,pair,arm,stamp,started,finished,n_attach,n_loss_marks,validity" > "$MAN"
{
  echo "batch_started=$(date +%FT%T%z)"
  echo "purpose=partial payload loss, four 0.05 kg boxes, staircase"
  echo "speed=${SPEED}m/s dyn_w=$DYN_W schedule=$SCHED post_sec=$POST_SEC"
  echo "git_head=$(cd "$WS/src" && git rev-parse HEAD 2>/dev/null)"
  echo "dirty_list<<EOF"
  (cd "$WS/src" && git status --porcelain 2>/dev/null)
  echo "EOF"
  sha256sum \
    "$WS/src/offboard_test_acados/offboard_test_acados/gripper/proximity_gripper_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/acados_nmpc_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/mhe_node.py" \
    "$WS/src/offboard_test_acados/offboard_test_acados/payload_estimate.py" \
    "$WS/src/scripts/gripper/run_gripper_headless.sh"
} > "${MAN%.csv}.provenance.txt"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false MHE_RESID_CONFIRM=false \
CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 \
GRIP_NBOX=4 GRIP_BOX_KG=0.05 GRIP_PAYLOAD_KG=0.20 EVAL_TRUE_PAYLOAD_MASS=0.20 \
GRIP_ATTACH_TOL=0.08 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=0 \
GRIP_DYN_R=10.0 GRIP_DYN_W=$DYN_W GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=400.0"

run_one() {
  local pair="$1" arm="$2" idx="$3" t0 N P stamp sched
  t0=$(date +%FT%T)
  echo "[partial] === pair=$pair arm=$arm idx=$idx ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.partial_before
  if [ "$arm" = L ]; then sched="$SCHED"; else sched="dry:$SCHED"; fi

  setsid nohup env $COMMON_ENV GRIP_PARTIAL_LOSS_SCHEDULE="$sched" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/partial_current.log 2>&1 9>&- &

  N=""
  for _ in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.partial_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "$idx,$pair,$arm,NA,$t0,$(date +%FT%T),0,0,invalid:no-log" >> "$MAN"
    return
  fi
  stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  P="$RES/grip_proximity_${stamp}.log"

  # 等首次 ATTACH(最多 ~6 min),再等时刻表最后一次 + POST_SEC
  for _ in $(seq 1 72); do grep -q -- "-> ATTACH" "$P" 2>/dev/null && break; sleep 5; done
  if grep -q -- "-> ATTACH" "$P" 2>/dev/null; then
    sleep $((CHECK_LAST + POST_SEC + 5))
  fi
  local na nl validity
  na=$(grep -c -- "-> ATTACH" "$P" 2>/dev/null); na=${na:-0}
  nl=$(grep -c "PARTIAL LOSS" "$P" 2>/dev/null); nl=${nl:-0}
  validity=$(python3 "$WS/src/scripts/gripper/aggregate_partial_loss.py" --validity "$RES" "$stamp" 2>&1 || true)
  echo "$idx,$pair,$arm,$stamp,$t0,$(date +%FT%T),$na,$nl,\"$validity\"" >> "$MAN"
  echo "[partial] stamp=$stamp attach=$na loss_marks=$nl validity=$validity"
}

idx=0
for pair in $(seq 1 "$PAIRS"); do
  if [ $((pair % 2)) -eq 1 ]; then order="L N"; else order="N L"; fi
  for arm in $order; do
    idx=$((idx + 1))
    run_one "$pair" "$arm" "$idx"
  done
done
cleanup
echo "[partial] complete: $MAN"
