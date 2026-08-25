#!/bin/bash
# 遗忘因子 λ 的 4m/s 配对 A/B(2026-08-24 第一批,n>=8)
#
# **单变量**:唯一差异是 MHE_LAMBDA(1.0 vs 0.8),其余全部默认——
#   MHE_GEOM_COUPLED 不设(默认 0)、event_trigger 默认 True、geom_release 默认 event、
#   mhe_tau_source 默认 command、MHE_N 默认 20、drop 默认 0.0(关)。
#   08-24 那轮 +1.33% 是 GEOM_COUPLED=1 + λ0.8 + self **三开关叠加**的结果;
#   本批只测 λ 的**纯效应**,数值不必复现 +1.33%,基线由本批自己的 λ=1.0 臂提供。
#
# **主判据(跑前钉死,不许事后换)**:带载 figure8 稳态段 m_est 相对偏差 bias_pct。
#   其余(pos_err/MHE 失败数/触界率)一律次要指标,只作描述不作判定。
#   —— 07-17 decouple-publish-ab 同一批 A/B 判读三次全反转,就是事后挑指标挑出来的。
# **`attach 入带`指标本批不用**:±2% 带在 8 字下量的是波动不是收敛(08-24 已知坏)。
#
# 工作点 = 08-19 定案的新 4m/s:r=10 / w=0.283 / z=6 / lift=8.4。
#   ⚠️ r=10 在 z=2.5 **必发散坠毁**,z=6 与 z=10 全稳(边界机制未明),所以
#      GRIP_Z_HIGH/GRIP_LIFT_DUR 必须显式给,不能吃 headless 的 2.5/3.0 默认值。
#   ⚠️ drop 全程关闭:主判据是**带载**稳态偏差,drop 后撞下界会截断数据污染统计
#      (08-18 实测 drop 后卡下界 138s 不回)。
#   ⚠️ ATTACH_WINDOW_SEC=140:[attach-window] 逐帧日志是唯一分析数据源,40s 默认
#      覆盖不住 22.2s 周期 x 3 圈,会把稳态段截掉。
#
# **配对**:每个 rep 内两臂各跑一次、顺序轮转,rep 内配对做 Wilcoxon signed-rank。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_lambda_ab_4ms.sh > /tmp/lam4ms.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_lambda_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

exec 9>"/tmp/lambda_ab_4ms.lock"
if ! flock -n 9; then echo "[lam4] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-8}"
ARMS="${ARMS:-lam1.0 lam0.8}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:--1}"

RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math;print(f'{math.sqrt(2)*$R*$W:.2f}')")

STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/lambda_ab_4ms_${STAMP}.txt"
{
  echo "# 遗忘因子 λ 4m/s 配对 A/B  $STAMP"
  echo "# 列: arm rep lambda ramp settle laps nmpc_stamp status peak_pos_err"
  echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
  echo "#         laps=$LAPS ramp_eff=$RAMP_EFF settle=$SETTLE fly=${FLY}s"
  echo "# 载荷: mass=$MASS ecc=$ECC  drop=OFF  attach_window=140s"
  echo "# 其余全默认: GEOM_COUPLED=0 event_trigger=True release=event tau=command MHE_N=20"
  echo "# 主判据(钉死): 带载 figure8 稳态段 m_est 相对偏差 bias_pct"
} > "$MAN"
echo "[lam4] manifest: $MAN"
echo "[lam4] 工作点 v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[lam4] interrupted"; cleanup; exit 130' INT TERM

cleanup   # 起批前先清干净(跨工作区残留会抢 14540 端口)

run_one() {
  local arm="$1" rep="$2" lam="${1#lam}"
  local launch="$RUNDIR/lam4_launch_${STAMP}_${arm}_${rep}.log"
  echo "[lam4] === arm=$arm rep=$rep (λ=$lam) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  MHE_LAMBDA=$lam \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=online \
    GRIP_DYNAMIC=true GRIP_DYN_R=$R GRIP_DYN_W=$W GRIP_DYN_RAMP=$RAMP \
    GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$LIFT_DUR \
    GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[lam4] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $lam $RAMP_EFF $SETTLE $LAPS - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[lam4] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $lam $RAMP_EFF $SETTLE $LAPS $nstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[lam4]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $lam $RAMP_EFF $SETTLE $LAPS $nstamp $status $peak" >> "$MAN"
  echo "[lam4]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

# 每 rep 轮转臂序,抵消机器热身/漂移。用 python 生成,别在 shell 里手搓位置参数轮转。
for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[lam4] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[lam4] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_lambda_ab.py $MAN"
