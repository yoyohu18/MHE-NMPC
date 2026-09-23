#!/bin/bash
# 配对 A/B(**投放段**):旧默认(点采样) vs 新默认(区间平均 + 越界重锚) (2026-09-18)
#
# ── 要回答的问题 ──────────────────────────────────────────────────────────
#   09-17 晚翻默认的依据(pfovr_ab_20260917_193153,64 轮)**不含投放段**,而 09-05
#   当初否掉 phys_full 的那批恰恰是带投放的。本批补这个缺口:同样两臂,但每轮飞到
#   **计划投放 + 事后 25 s**,看释放语义与投放段安全性是否退化。
#   背景:记忆 liftoff-lb-stuck-mechanisms、tau-source-viz-headless-ab。
#
# ── 臂(唯一差异)────────────────────────────────────────────────────────────
#   A = MHE_TAU_SOURCE=command   MHE_MOTOR_AVG=0 MHE_REANCHOR_OVR=0   (09-17 晚之前的默认)
#   B = MHE_TAU_SOURCE=phys_full MHE_MOTOR_AVG=1 MHE_REANCHOR_OVR=1   (当前默认)
#   ⚠️ sensor-only 主线下两臂都不用 NMPC 指令力矩:真实差别是电机量**点采样 vs 区间平均**
#      (+越界重锚),臂名里的 command 只是参数取值。
#   其余全部显式钉死 = run_e1_dev.sh 的 COMMON_ENV(stuck 轮出处)+ 不投放。
#
# ── 工作点 × 配对 ─────────────────────────────────────────────────────────
#   只跑 w4 = GRIP_DYN_W=0.283 (4 m/s, 当前默认工作点),REPS 对。
#   r=10 z=6 lift 8.4 ramp 9.36 dz 0.8 0.15kg ecc_y 0.10 包线 0.3,两点相同。
#   臂序奇数 rep AB、偶数 rep BA(抗时间漂移)。偏心由吸附随机决定,不可控 ——
#   配对只配时间邻近,不配偏心。每轮飞完整流程:吸附 → 吊起 → figure-8 →
#   计划投放(figure-8 尖端,指令后 55 s)→ 事后再飞 POST_SEC。
#
# ── 判定规则(跑前钉死,不许事后换)──────────────────────────────────────────
#   有效轮:proximity 日志有 "-> ATTACH"(**物理吸附**,不用 m̂ 判 —— E1 的
#     m̂ 口径会把 stuck 当 attach-fail 剔掉,正是要测的现象)。
#     一对中任一轮无效 → 整对剔除,按臂/工作点报告剔除数。
#   ① 一票否决(安全):B 臂"发散"轮数 > A 臂 → 不得保留新默认,无论主判据。
#     发散 = peak pos_err > 2 m 或 Traceback 或 300s 内没进 figure-8。
#   ② 主判据(投放段语义,两臂逐项对比,**均为描述性**):
#     · 提前清除(accept 早于真实分离)——两臂都应为 0;
#     · 释放确认成功率 / UNRESOLVED 率;
#     · 检出延迟(真实分离 → accept)。
#   ③ 复核(已在无投放批次上显著,这里只看是否复现方向):
#     吊起段 m 贴下界占比 LB_lift 的配对差。
#   ④ 只描述:带载段 m̂ 偏差、投放后残留、solve failed、事件后 pos_err 峰值。
#   **功效声明**:REPS=8 对。提前清除/发散这类事件率低,二元终点**没有统计功效**,
#   本批只回答"新默认在投放段是否出现可见的退化",不做显著性声明。
#   **措辞纪律**:CI 含 0 只能写"未检出差异",不能写"无效";显著也只能写
#   "离地段贴下界减少",不能外推到投放段/释放判据(本批不投放)。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_pfovr_drop_ab.sh > /tmp/pfovr_drop.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_pfovr_drop_ab.py <batch_dir>
# ⚠️ 批次运行期间不要改 mhe_node.py / acados_nmpc_node.py / run_gripper_headless.sh
#    (headless 每轮会读脚本并启动节点,改动会混进后续轮次)。
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[ab] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 9>"/tmp/pfovr_drop_ab.lock"
if ! flock -n 9; then echo "[ab] 已有实例在跑,退出。"; exit 1; fi

echo "[ab] 等待 SITL 栈空闲(最多 40min)..."
for i in $(seq 1 480); do
  busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
  [ "$busy" -eq 0 ] && break
  sleep 5
done
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[ab] 栈仍被占用,放弃"; exit 2; }

REPS="${REPS:-8}"
STAMP=$(date +%Y%m%d_%H%M%S)
BDIR="$RUNDIR/pfovr_drop_ab_$STAMP"; mkdir -p "$BDIR/resid"
MAN="$BDIR/manifest.csv"
echo "idx,rep,arm,stamp,status,peak_pos_err,started,finished,resid_dir" > "$MAN"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
MHE_EVENT_TRIGGER=false RESID_STEP_ENABLE=false \
GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true \
GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DROP_AFTER=55.0 \
GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 ATTACH_WINDOW_SEC=140.0 \
EVAL_TRUE_PAYLOAD_MASS=0.15 MHE_RESID_CONFIRM=true \
CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0 \
GRIP_UNCOMMANDED_LOSS=false RELEASE_BASELINE=P"

arm_env() {
  case "$1" in
    A) echo "MHE_TAU_SOURCE=command MHE_MOTOR_AVG=0 MHE_REANCHOR_OVR=0" ;;
    B) echo "MHE_TAU_SOURCE=phys_full MHE_MOTOR_AVG=1 MHE_REANCHOR_OVR=1" ;;
  esac
}
POST_SEC="${POST_SEC:-25}"
EVENT_TIMEOUT="${EVENT_TIMEOUT:-500}"

cat > "$BDIR/PREREGISTRATION.txt" <<EOF
batch=pfovr_drop_ab_$STAMP  REPS=$REPS  created=$(date +%FT%T)
arms: A=$(arm_env A) | B=$(arm_env B)
workpoint: w4 GRIP_DYN_W=0.283 (4 m/s);计划投放 GRIP_DROP_AFTER=55,事后 POST_SEC=$POST_SEC s
common: $COMMON_ENV
判定规则见 run_pfovr_ab.sh 头部(① 否决 ② 主判据 LB_lift 配对 Wilcoxon ③ 二元只描述 ④ 次要)。
EOF
( cd "$WS/src" && git rev-parse HEAD && git status --short ) >> "$BDIR/PREREGISTRATION.txt" 2>&1

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
trap 'echo "[ab] interrupted"; cleanup; exit 130' INT TERM
cleanup

IDX=0
run_one() {
  local rep="$1" arm="$2" t0 tag rdir launch
  IDX=$((IDX+1))
  tag="${IDX}_${arm}_r${rep}"; rdir="$BDIR/resid/$tag"; launch="$BDIR/launch_$tag.log"
  mkdir -p "$rdir"; t0=$(date +%FT%T)
  echo "[ab] === idx=$IDX rep=$rep arm=$arm ==="
  env $COMMON_ENV $(arm_env "$arm") RESID_LOG_DIR="$rdir" \
    EXP_RUN_ID="pfovrdrop_${STAMP}_${IDX}" EXP_METHOD="$arm" EXP_CONDITION=drop \
    EXP_BATCH="pfovr_drop_ab_$STAMP" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[ab]   FAIL: 拿不到 nmpc 日志名"; tail -3 "$launch"
    echo "$IDX,$rep,$arm,-,NOLOG,99,$t0,$(date +%FT%T),$rdir" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  # 等计划投放指令,再飞 POST_SEC 观察事后行为
  local waited=0 got=0
  while [ "$waited" -lt "$EVENT_TIMEOUT" ]; do
    grep -aq "DROP command issued" "$nmpc" 2>/dev/null && { got=1; break; }
    grep -aq "Traceback" "$nmpc" 2>/dev/null && break
    sleep 5; waited=$((waited+5))
  done
  local status=ok peak
  if [ "$got" -eq 0 ]; then status=NOEVENT; else sleep "$POST_SEC"; fi
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && [ "$status" = ok ] && status=DIVERGED
  echo "$IDX,$rep,$arm,$nstamp,$status,$peak,$t0,$(date +%FT%T),$rdir" >> "$MAN"
  echo "[ab]   stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[ab] 开跑: $REPS 对 × 2 臂 = $((REPS*2)) 轮(含投放) -> $BDIR"
for rep in $(seq 1 "$REPS"); do
  if [ $((rep % 2)) -eq 1 ]; then arms="A B"; else arms="B A"; fi
  for arm in $arms; do run_one "$rep" "$arm"; done
done
echo "[ab] 批次完成 -> $MAN"
