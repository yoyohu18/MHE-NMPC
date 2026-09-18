#!/bin/bash
# 推力系数标定误差敏感性:只缩放**估计器侧**推力图 ±5% (2026-09-18)
#
# 动机:真机迁移的头号风险。仿真里被控对象与估计器共用同一条平方律、转速还是精确值;
# 真机上 k 由推力台标定,必然有误差。本批把 MHE 解读转速用的增益乘 0.95 / 1.00 / 1.05,
# 仿真物理和 NMPC 的油门映射都不动 —— 等价于"真机上 k 标定错了"。
#
# 臂(唯一差异 = MHE_THRUST_CAL_GAIN):
#   k095  0.95   预期(09-15 旧配置实测):载荷 4/4 从未被识别,带着货飞空机模型
#   k100  1.00   对照(当前默认)
#   k105  1.05   预期:投放后残留约 +0.09kg 幽灵载荷,3/4 停在 UNRESOLVED
# 其余配置 = 当前默认(4 m/s 主线,0.15kg,整轮含投放),每臂 REPS 轮。
#
# 判定规则(跑前钉死):
#   有效轮 = proximity 日志有 "-> ATTACH"(物理吸附口径,**不用 m̂ 判**——本批正是
#            m̂ 会失真的批次,用 m̂ 判有效性会把要观测的现象筛掉)。
#   ① 载荷识别:带载段 m̂ 峰值 − 空机质量 ≥ 0.08 kg(= 0.15kg 的一半)算"识别到"。
#      预期 k095 显著低于 k100。
#   ② 幽灵载荷:投放确认后 5~15 s 的 m̂ 中位 − 空机质量,≥ 0.05 kg 记一次幽灵。
#      预期 k105 显著高于 k100。
#   ③ 释放语义:DROP complete / UNRESOLVED / 提前清除 / 检出延迟(与安全冒烟同口径)。
#   ④ 只描述:带载段 m̂ 偏差、发散、solve failed、吊起段贴下界占比。
#   功效声明:n=REPS(默认 4)/臂,只能看"是否复现出定性失效模式",不做统计检验。
#
# 用法: REPS=4 setsid bash src/scripts/gripper/run_thrust_cal_sens.sh > /tmp/thrust_sens.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_thrust_cal_sens.py <batch_dir>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[tc] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 9>"/tmp/thrust_cal_sens.lock"
if ! flock -n 9; then echo "[tc] 已有实例在跑,退出。"; exit 1; fi
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[tc] SITL 栈被占用,放弃"; exit 2; }

REPS="${REPS:-4}"
ARMS="${ARMS:-k095 k100 k105}"
POST_SEC="${POST_SEC:-25}"
# 等待投放指令的上限。k=1.05 时估计器把质量高估 5%,NMPC 前馈推力偏大且无积分,
# 悬停有稳态高度偏差 → 下降段的完成判据要磨很久(预实验 20260918_033542 实测
# 下降段 325 s,投放推到 ~398 s),400 s 会把这类轮次误判成 NOEVENT。
EVENT_TIMEOUT="${EVENT_TIMEOUT:-900}"
STAMP=$(date +%Y%m%d_%H%M%S)
BDIR="$RUNDIR/thrust_cal_sens_$STAMP"; mkdir -p "$BDIR/resid"
MAN="$BDIR/manifest.csv"
echo "idx,rep,arm,k,stamp,status,peak_pos_err,started,finished,resid_dir" > "$MAN"

COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self \
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

arm_k() { case "$1" in k095) echo 0.95 ;; k100) echo 1.00 ;; k105) echo 1.05 ;; *) echo BAD ;; esac; }

{
echo "batch=thrust_cal_sens_$STAMP REPS=$REPS ARMS=$ARMS created=$(date +%FT%T)"
echo "唯一自变量: MHE_THRUST_CAL_GAIN (仅估计器侧;仿真物理与 NMPC 油门映射不变)"
for a in $ARMS; do echo "  $a -> $(arm_k "$a")"; done
echo "common: $COMMON_ENV"
echo "判定规则见 run_thrust_cal_sens.sh 头部(①识别 ②幽灵 ③释放语义 ④只描述)"
( cd "$WS/src" && git rev-parse HEAD && git status --short )
} > "$BDIR/PREREGISTRATION.txt" 2>&1

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
trap 'echo "[tc] interrupted"; cleanup; exit 130' INT TERM
cleanup

IDX=0
run_one() {
  local rep="$1" arm="$2" k tag rdir launch t0
  k=$(arm_k "$arm"); [ "$k" = BAD ] && { echo "[tc] 未知臂 $arm"; return; }
  IDX=$((IDX+1)); tag="${IDX}_${arm}_r${rep}"; rdir="$BDIR/resid/$tag"
  launch="$BDIR/launch_$tag.log"; mkdir -p "$rdir"; t0=$(date +%FT%T)
  echo "[tc] === idx=$IDX rep=$rep arm=$arm (k=$k) ==="
  env $COMMON_ENV MHE_THRUST_CAL_GAIN="$k" RESID_LOG_DIR="$rdir" \
    EXP_RUN_ID="tcal_${STAMP}_${IDX}" EXP_METHOD="$arm" EXP_CONDITION="k$k" \
    EXP_BATCH="thrust_cal_sens_$STAMP" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" i
  for i in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[tc]   FAIL: 拿不到 nmpc 日志名"
    echo "$IDX,$rep,$arm,$k,-,NOLOG,99,$t0,$(date +%FT%T),$rdir" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt "$EVENT_TIMEOUT" ]; do
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
  echo "$IDX,$rep,$arm,$k,$nstamp,$status,$peak,$t0,$(date +%FT%T),$rdir" >> "$MAN"
  echo "[tc]   stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[tc] 开跑: $REPS rep × [$ARMS] -> $BDIR (事件等待上限 ${EVENT_TIMEOUT}s)"
for rep in $(seq 1 "$REPS"); do
  for arm in $ARMS; do run_one "$rep" "$arm"; done
done
echo "[tc] 批次完成 -> $MAN"
