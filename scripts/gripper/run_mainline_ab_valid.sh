#!/bin/bash
# 连续主线验收·第二轮(2026-09-04):W3/W4/W5,每格 8 个**有效**配对。
#
# 与第一轮(run_mainline_ab.sh)的差别只有两处:
#   ① 修了三个缺陷后重跑 —— MHE 释放时把 s 的发布目标切零(不是放宽 conf 阈值)、
#      NMPC 发释放指令时无条件重新武装空载检测、payload-health 的 logger severity。
#   ② 预注册的**配对有效性**:drop 之前坠机 / attach 未兑现 = 该配对作废并补跑,
#      发生率单独统计(check_run_valid.py)。作废的轮次数据仍留在盘上可复查。
#
# 主线验收(预注册):有效的 24 次 B 臂必须全部完成 drop、confidence 达标、
# 且 drop 之后零失稳。
#
# 用法: [VALID_PAIRS=8] [MAX_TRIES=16] [ONLY_W=W3] bash src/scripts/gripper/run_mainline_ab_valid.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
MAN=$RES/mainline_ab2_manifest.csv
LOCK=/tmp/mainline_ab.lock          # 与第一轮同一把锁:同时只允许一个 SITL 批次
VALID_PAIRS=${VALID_PAIRS:-8}
MAX_TRIES=${MAX_TRIES:-16}          # 每格尝试上限;够不到 8 对就如实少报,不硬凑
POST_SEC=${POST_SEC:-40}
CHECK=$WS/src/scripts/gripper/check_run_valid.py

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[批次] 拒启:检测到非本工作区 px4/gz/make 在跑"; echo "$foreign" | cut -c1-100
  exit 2
fi
exec 9>"$LOCK"
flock -n 9 || { echo "[批次] 另一个批次正持锁,退出"; exit 1; }

cleanup() {
  for pat in "run_gripper[_]headless.sh" "px4_sitl_default/bin/px4" "make px4_sitl" \
             "gz sim" "gz_bridge" "ros_gz_bridge" "mavros/mavros_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "MicroXRCEAgent" "gcs_heartbeat"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
  done
  sleep 8
}
trap 'echo "[批次] 收到中断,清栈退出"; cleanup; exit 130' INT TERM

[ -f "$MAN" ] || echo "idx,wid,pair,arm,stamp,started,finished,status,validity" > "$MAN"

w_env() {
  case "$1" in
    W3) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
    W4) echo "GRIP_PAYLOAD_KG=0.30 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
    W5) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=30.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.1415 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
  esac
}
w_truth() { case "$1" in W4) echo 0.30 ;; *) echo 0.15 ;; esac; }

arm_env() {
  if [ "$1" = "A" ]; then
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=false NMPC_GEOM_SOURCE=online \
NMPC_GEOM_RELEASE_MODE=event MHE_PAYLOAD_LOST_WATCH=0"
  else
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0"
  fi
}

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
ATTACH_WINDOW_SEC=140.0"

IDX=0
LAST_LOG=""
LAST_STATUS=""

run_one() {
  local wid="$1" arm="$2" pair="$3"
  IDX=$((IDX + 1))
  local t0; t0=$(date +%FT%T)
  echo "[批次] === #$IDX  $wid pair$pair arm=$arm  $(date +%T) ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.mainline2_before

  setsid nohup env $COMMON_ENV $(w_env "$wid") $(arm_env "$arm") \
    EVAL_TRUE_PAYLOAD_MASS=$(w_truth "$wid") \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/mainline2_cur.log 2>&1 9>&- &

  local N="" i
  for i in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.mainline2_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    LAST_LOG=""; LAST_STATUS="no-log"
    echo "$IDX,$wid,$pair,$arm,NA,$t0,$(date +%FT%T),no-log,invalid" >> "$MAN"
    return
  fi
  local stamp; stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}

  LAST_STATUS="timeout"
  for i in $(seq 1 72); do
    grep -qE "DROP complete|DROP: released" "$N" 2>/dev/null && { LAST_STATUS="ok"; break; }
    sleep 5
  done
  [ "$LAST_STATUS" = "ok" ] && sleep "$POST_SEC"
  cleanup

  local v; v=$(python3 "$CHECK" "$N" 2>&1)
  LAST_LOG="$N"
  echo "$IDX,$wid,$pair,$arm,$stamp,$t0,$(date +%FT%T),$LAST_STATUS,\"$v\"" >> "$MAN"
  echo "[批次] #$IDX 结束: $LAST_STATUS | 有效性: $v"
  case "$v" in valid) return 0 ;; *) return 1 ;; esac
}

WIDS="${ONLY_W:-W3 W4 W5}"
for wid in $WIDS; do
  pair=0; tries=0
  while [ "$pair" -lt "$VALID_PAIRS" ] && [ "$tries" -lt "$MAX_TRIES" ]; do
    tries=$((tries + 1))
    # 对内 ABBA/BAAB 交替
    if [ $((tries % 2)) -eq 1 ]; then order="A B"; else order="B A"; fi
    ok=1
    for arm in $order; do
      run_one "$wid" "$arm" "$((pair + 1))" || ok=0
    done
    if [ "$ok" -eq 1 ]; then
      pair=$((pair + 1))
      echo "[批次] >>> $wid 有效配对 $pair/$VALID_PAIRS (尝试 $tries)"
    else
      echo "[批次] >>> $wid 本对作废(按预注册补跑),尝试 $tries/$MAX_TRIES"
    fi
  done
  echo "[批次] === $wid 收工:有效 $pair 对 / 尝试 $tries 次 ==="
done

cleanup
echo "[批次] 全部完成 $(date +%T) -> $MAN"
