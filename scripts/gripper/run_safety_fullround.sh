#!/bin/bash
# 整轮安全冒烟:当前默认配置下 吸附→吊起→figure-8→释放/脱落→事后飞行 (2026-09-17)
#
# 工况(每 rep 交替):
#   d0   正常释放:指令即脱离
#   d4   延迟脱离:指令后 4 s 才物理脱离      → 看是否"提前清除"(accept 早于 sep)
#   jam  释放失败:指令后永不脱离            → 应进 UNRESOLVED、制动悬停,不得清模型
#   loss 意外脱落:不发指令直接物理脱离      → 看能否自主检出、飞行是否稳定
# 配置 = headless 当前默认(MHE phys_full+区间平均+越界重锚;NMPC 冻结关)+ viz 的 4 m/s 主线
# 工作点;这里只钉工作点/载荷/事件链,不碰 MHE_TAU_SOURCE 等(让它走默认)。
# 这是冒烟(描述性),不是 A/B;每格 n=REPS。
#
# 用法: REPS=3 setsid bash src/scripts/gripper/run_safety_fullround.sh > /tmp/safety.log 2>&1 &
# 汇总: python3 src/scripts/gripper/aggregate_safety_fullround.py <batch_dir>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[sf] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 9>"/tmp/safety_fullround.lock"
if ! flock -n 9; then echo "[sf] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-3}"
CONDS="${CONDS:-d0 d4 jam loss}"
POST_SEC="${POST_SEC:-25}"          # 事件后再飞多久(jam 需 12s unresolved + 3s 制动 + 悬停)
# 工况旋钮(默认 = 09-17 那批:4 m/s、0.15kg)。改这两个即可换工作点/载荷,
# 其余判定规则不变,批次目录名会带上取值,便于区分。
W="${W:-0.283}"                     # 0.283=4 m/s, 0.1415=2 m/s
MASS="${MASS:-0.15}"                # 载荷质量 [kg]
TAG="${TAG:-w${W}_m${MASS}}"
STAMP=$(date +%Y%m%d_%H%M%S)
BDIR="$RUNDIR/safety_fullround_${TAG}_$STAMP"; mkdir -p "$BDIR/resid"
MAN="$BDIR/manifest.csv"
echo "idx,rep,cond,stamp,status,peak_pos_err,started,finished,resid_dir" > "$MAN"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=$W GRIP_DROP_AFTER=55.0 \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=$MASS MHE_RESID_CONFIRM=true \
CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 \
RELEASE_BASELINE=P"

cond_env() {
  case "$1" in
    d0)   echo "GRIP_DETACH_DELAY_SEC=0.0 GRIP_DETACH_JAM=false GRIP_UNCOMMANDED_LOSS=false" ;;
    d4)   echo "GRIP_DETACH_DELAY_SEC=4.0 GRIP_DETACH_JAM=false GRIP_UNCOMMANDED_LOSS=false" ;;
    jam)  echo "GRIP_DETACH_DELAY_SEC=0.0 GRIP_DETACH_JAM=true GRIP_UNCOMMANDED_LOSS=false" ;;
    loss) echo "GRIP_DETACH_DELAY_SEC=0.0 GRIP_DETACH_JAM=false GRIP_UNCOMMANDED_LOSS=true" ;;
    *)    echo "BAD" ;;
  esac
}

{
echo "batch=safety_fullround_${TAG}_$STAMP REPS=$REPS CONDS=$CONDS W=$W MASS=$MASS POST_SEC=$POST_SEC created=$(date +%FT%T)"
echo "common: $COMMON_ENV"
for c in $CONDS; do echo "$c: $(cond_env "$c")"; done
( cd "$WS/src" && git rev-parse HEAD && git status --short )
} > "$BDIR/CONFIG.txt" 2>&1

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
trap 'echo "[sf] interrupted"; cleanup; exit 130' INT TERM
cleanup

IDX=0
run_one() {
  local rep="$1" cond="$2" tag rdir launch t0 cenv
  IDX=$((IDX+1)); cenv=$(cond_env "$cond")
  [ "$cenv" = BAD ] && { echo "[sf] 未知工况 $cond"; return; }
  tag="${IDX}_${cond}_r${rep}"; rdir="$BDIR/resid/$tag"; launch="$BDIR/launch_$tag.log"
  mkdir -p "$rdir"; t0=$(date +%FT%T)
  echo "[sf] === idx=$IDX rep=$rep cond=$cond ==="
  env $COMMON_ENV $cenv RESID_LOG_DIR="$rdir" \
    EXP_RUN_ID="safety_${TAG}_${STAMP}_${IDX}" EXP_METHOD=P EXP_CONDITION="$cond" \
    EXP_BATCH="safety_fullround_${TAG}_$STAMP" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[sf]   FAIL: 拿不到 nmpc 日志名"
    echo "$IDX,$rep,$cond,-,NOLOG,99,$t0,$(date +%FT%T),$rdir" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  # 等事件:释放指令 / 意外脱落(最多 400 s;figure-8 进场后 55 s 才到)
  local waited=0 got=0
  while [ "$waited" -lt 400 ]; do
    grep -aqE "DROP command issued|UNCOMMANDED LOSS" "$nmpc" 2>/dev/null && { got=1; break; }
    grep -aq "Traceback" "$nmpc" 2>/dev/null && break
    sleep 5; waited=$((waited+5))
  done
  local status=ok peak
  if [ "$got" -eq 0 ]; then status=NOEVENT; else sleep "$POST_SEC"; fi
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && [ "$status" = ok ] && status=DIVERGED
  echo "$IDX,$rep,$cond,$nstamp,$status,$peak,$t0,$(date +%FT%T),$rdir" >> "$MAN"
  echo "[sf]   stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[sf] 开跑: $REPS rep × [$CONDS] -> $BDIR"
for rep in $(seq 1 "$REPS"); do
  for cond in $CONDS; do run_one "$rep" "$cond"; done
done
echo "[sf] 批次完成 -> $MAN"
