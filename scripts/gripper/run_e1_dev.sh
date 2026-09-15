#!/bin/bash
# E1 evidence-gating 开发集(实验计划 §6 / §12.2 / §14 第 4 步)。
#
# 设计(09-16 定):W5(0.15kg / ecc_y 0.10 / 2 m/s)
#   臂   O=ORACLE  A=A_PRIME  CS=C_SAME  CT=C-thrust  P=主线
#   条件 d0=正常释放  d4=物理脱离延迟 4 s  jam=永久卡死
#   每个区组 = 5 臂 × 3 条件 = 15 架次;区组内顺序按记录下来的种子随机化,
#   整张排程在开跑前写死(schedule.csv),中断后用 RESUME_DIR 续跑。
#
# 所有架次保留(intention-to-test,§12.3):check_run_valid 只记录不补跑。
# 每架次开 RESID_LOG_DIR(§13.4),EXP_* 进 provenance(§13.5)。
# 开发集用途仅限调阈值/估方差,不进确认性检验(§12.2)。
#
# 用法:
#   BLOCKS=1 bash run_e1_dev.sh                  # 冒烟 1 个区组(15 架次)
#   BLOCKS=12 bash run_e1_dev.sh                 # 开发集
#   RESUME_DIR=<批次目录> bash run_e1_dev.sh     # 续跑未完成的 idx
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
CHECK=$WS/src/scripts/gripper/check_run_valid.py
LOCK=/tmp/mainline_ab.lock
POST_SEC=${POST_SEC:-12}

if [ -n "${RESUME_DIR:-}" ]; then
  BDIR=$RESUME_DIR
  [ -f "$BDIR/schedule.csv" ] || { echo "[e1-dev] $BDIR 没有 schedule.csv"; exit 2; }
  BATCH=$(basename "$BDIR")
else
  BATCH=e1_dev_$(date +%Y%m%d_%H%M%S)
  BDIR=$RES/$BATCH
fi
MAN=$BDIR/manifest.csv

# ---- 起跑前守卫 ----
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[e1-dev] refuse: non-workspace PX4/Gazebo process detected"
  echo "$foreign" | cut -c1-100
  exit 2
fi
dirty=$(cd "$WS/src" && git status --porcelain 2>/dev/null)
if [ -n "$dirty" ] && [ "${ALLOW_DIRTY:-0}" != 1 ]; then
  echo "[e1-dev] refuse: 工作树不 clean,开发集不得用 dirty 代码采集(§12.2)"
  echo "$dirty" | head
  exit 2
fi

exec 9>"$LOCK"
flock -n 9 || { echo "[e1-dev] another batch holds $LOCK"; exit 1; }

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
trap 'echo "[e1-dev] interrupted"; cleanup; exit 130' INT TERM

# ---- 排程(新批次才生成) ----
if [ -z "${RESUME_DIR:-}" ]; then
  mkdir -p "$BDIR"
  BLOCKS=${BLOCKS:-12}
  SEED=${SEED:-$(date +%s)}
  python3 - "$BDIR/schedule.csv" "$BLOCKS" "$SEED" <<'EOF'
import csv, random, sys
out, blocks, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
arms = ['O', 'A', 'CS', 'CT', 'P']
conds = ['d0', 'd4', 'jam']
rng = random.Random(seed)
idx = 0
with open(out, 'w', newline='') as f:
    w = csv.writer(f, lineterminator='\n')   # bash read 不吃 \r
    w.writerow(['idx', 'block', 'arm', 'cond'])
    for b in range(1, blocks + 1):
        cells = [(a, c) for a in arms for c in conds]
        rng.shuffle(cells)
        for a, c in cells:
            idx += 1
            w.writerow([idx, b, a, c])
EOF
  echo "idx,block,arm,cond,method,stamp,started,finished,command_seen,detach_seen,completion_seen,unresolved_seen,validity,resid_dir" > "$MAN"
  {
    echo "batch=$BATCH"
    echo "batch_started=$(date +%FT%T%z)"
    echo "purpose=E1 evidence-gating dev set (plan §6, §12.2)"
    echo "workpoint=W5: payload=0.15kg ecc_y=0.10m peak_speed=2m/s"
    echo "arms=O:ORACLE A:A_PRIME CS:C_SAME CT:C-thrust(MHE_RELEASE_ARM_ON_COMMAND) P:mainline"
    echo "conds=d0:delay0 d4:delay4s jam:GRIP_DETACH_JAM"
    echo "blocks=$BLOCKS seed=$SEED"
    echo "git_head=$(cd "$WS/src" && git rev-parse HEAD 2>/dev/null)"
    echo "dirty_list<<EOF"
    echo "$dirty"
    echo "EOF"
  } > "$BDIR/batch_info.txt"
fi

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DROP_AFTER=30.0 GRIP_DYN_W=0.1415 \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=0.15 MHE_RESID_CONFIRM=true \
CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 \
GRIP_UNCOMMANDED_LOSS=false"

arm_env() {
  case "$1" in
    O)  echo "RELEASE_BASELINE=ORACLE MHE_RELEASE_ARM_ON_COMMAND=false" ;;
    A)  echo "RELEASE_BASELINE=A_PRIME MHE_RELEASE_ARM_ON_COMMAND=false" ;;
    CS) echo "RELEASE_BASELINE=C_SAME MHE_RELEASE_ARM_ON_COMMAND=false" ;;
    CT) echo "RELEASE_BASELINE=P MHE_RELEASE_ARM_ON_COMMAND=true" ;;
    P)  echo "RELEASE_BASELINE=P MHE_RELEASE_ARM_ON_COMMAND=false" ;;
    *)  echo "BAD_ARM" ;;
  esac
}
method_name() {
  case "$1" in O) echo ORACLE ;; A) echo A_PRIME ;; CS) echo C_SAME ;;
               CT) echo C_THRUST ;; P) echo P ;; esac
}
cond_env() {
  case "$1" in
    d0)  echo "GRIP_DETACH_DELAY_SEC=0.0 GRIP_DETACH_JAM=false" ;;
    d4)  echo "GRIP_DETACH_DELAY_SEC=4.0 GRIP_DETACH_JAM=false" ;;
    jam) echo "GRIP_DETACH_DELAY_SEC=0.0 GRIP_DETACH_JAM=true" ;;
    *)   echo "BAD_COND" ;;
  esac
}

run_one() {
  local idx="$1" block="$2" arm="$3" cond="$4" t0 N stamp P S validity rdir aenv cenv meth
  aenv=$(arm_env "$arm"); cenv=$(cond_env "$cond"); meth=$(method_name "$arm")
  case "$aenv$cenv" in *BAD_*) echo "[e1-dev] bad cell $arm/$cond"; exit 2 ;; esac
  rdir="$BDIR/resid/${idx}_${arm}_${cond}"
  mkdir -p "$rdir"
  t0=$(date +%FT%T)
  echo "[e1-dev] === idx=$idx block=$block arm=$arm($meth) cond=$cond ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.e1_dev_before

  setsid nohup env $COMMON_ENV $aenv $cenv \
    RESID_LOG_DIR="$rdir" \
    EXP_RUN_ID="${BATCH}_${idx}" EXP_BLOCK_ID="$block" EXP_METHOD="$meth" \
    EXP_CONDITION="$cond" EXP_BATCH="$BATCH" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > "$BDIR/headless_${idx}.log" 2>&1 9>&- 3<&- &

  N=""
  for _ in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.e1_dev_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "$idx,$block,$arm,$cond,$meth,NA,$t0,$(date +%FT%T),0,0,0,0,invalid:no-log,$rdir" >> "$MAN"
    return
  fi
  stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  P="$RES/grip_proximity_${stamp}.log"
  S="$RES/grip_state_${stamp}.log"

  for _ in $(seq 1 90); do
    grep -qE "DROP command issued|DROP: released gripper" "$N" 2>/dev/null && break
    sleep 5
  done
  local cmd=0 det=0 done=0 unres=0
  grep -qE "DROP command issued|DROP: released gripper" "$N" 2>/dev/null && cmd=1

  if [ "$cmd" -eq 1 ]; then
    # 物理脱离:d0 立即、d4 +4 s、jam 永不。窗口统一 20 s,jam 也等满,
    # 让三种条件的观测时长一致(不因条件不同而截短日志)。
    for _ in $(seq 1 20); do
      grep -q -- "-> DETACH" "$P" 2>/dev/null && break
      sleep 1
    done
    grep -q -- "-> DETACH" "$P" 2>/dev/null && det=1
    # 完成确认或 UNRESOLVED(τ_mission=12 s 超时),最多再等 24 s。
    for _ in $(seq 1 24); do
      grep -qE "DROP complete|DROP: released gripper|UNRESOLVED" "$N" 2>/dev/null && break
      sleep 1
    done
    grep -qE "DROP complete|DROP: released gripper" "$N" 2>/dev/null && done=1
    grep -q "UNRESOLVED" "$N" 2>/dev/null && unres=1
    sleep "$POST_SEC"
  fi

  validity=$(python3 "$CHECK" "$N" 2>&1 || true)
  [ -s "$S" ] || det=0
  echo "$idx,$block,$arm,$cond,$meth,$stamp,$t0,$(date +%FT%T),$cmd,$det,$done,$unres,\"$validity\",$rdir" >> "$MAN"
  echo "[e1-dev] stamp=$stamp cmd=$cmd detach=$det complete=$done unresolved=$unres validity=$validity"
}

total=$(($(wc -l < "$BDIR/schedule.csv") - 1))
if [ "${DRY_RUN:-0}" = 1 ]; then
  # 只核对排程与每格注入的环境变量:不 cleanup、不起栈。
  while IFS=, read -r -u 3 idx block arm cond; do
    [ "$idx" = idx ] && continue
    cond=${cond%$'\r'}
    echo "$idx b$block $arm/$cond | $(method_name "$arm") | $(arm_env "$arm") $(cond_env "$cond")"
  done 3< "$BDIR/schedule.csv"
  exit 0
fi
# 排程走 fd 3,不走 stdin:循环体里的 headless 栈/python 子进程会继承 stdin,
# 可能把排程后续行读掉导致静默跳轮。
while IFS=, read -r -u 3 idx block arm cond; do
  [ "$idx" = idx ] && continue
  cond=${cond%$'\r'}
  if grep -q "^$idx," "$MAN"; then
    continue
  fi
  echo "[e1-dev] progress $idx/$total ($(date +%T))"
  run_one "$idx" "$block" "$arm" "$cond" < /dev/null
done 3< "$BDIR/schedule.csv"

cleanup
echo "[e1-dev] complete: $MAN"
