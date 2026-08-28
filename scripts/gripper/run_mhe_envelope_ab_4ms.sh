#!/bin/bash
# MHE 侧几何标度 0.5 vs 0.7:4m/s 工作点配对 A/B (2026-08-28)
#
# ── 为什么做 ──────────────────────────────────────────────────────────────
#   08-28 排查 m_est 带载机动低估(记忆 mest-maneuver-underestimate),已排除:
#     · 推力标定 —— T_phys/g 与真值差 0.04%,MHE 吃的确实是 thrust_phys
#     · 倾角/cos —— 实测 tilt=0.11~0.44°, cos=0.99999
#     · 气动阻力 —— kd=0.05 → 0.14N(0.6%),且 vz≈0 时不进质量平衡,方向还反
#   并收窄到"**带载才有、空机没有**"(drop 后空机 m_est 与 T/g 差 ≈0)。
#   剩下的候选 H2':MHE 转动通路几何按包线 0.5kg 标度(真值 0.3kg,过估 67%),
#   虽然 legacy 档 ∂(J,c)/∂m≡0,但几何失配推动 quat 状态偏离,而 quat 出现在
#   平动方程的 R_q 里 —— 这条间接路径是通的。
#
#   单轮预试(0.5 三轮 vs 0.7 一轮)显示 m_est 对几何标度**有敏感性但方向与
#   H2' 预期相反**(0.7 反而更准 −0.99% vs −2.01%)。⚠️ 那个对比不能归因:
#   attach 几何有轮间随机性(三轮 MHE dJ = 0.0962/0.1022/0.0702,跨度 46%),
#   0.5 臂三轮偏差 {−1.42,−2.66,−1.96} 标准差 0.62pp,0.7 那一轮距均值仅 1.6σ。
#   本批就是把这个噪声量清楚再判。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = MHE_PAYLOAD_ENVELOPE(MHE 侧转动通路几何标度)。
#   ⚠️ NMPC 侧 GRIP_PAYLOAD_ENVELOPE **两臂钉死 0.5** —— 08-28 之前这两者共用
#      同一个变量,扫它分不清是哪一侧起作用;headless 已拆开(MHE_PAYLOAD_ENVELOPE)。
#   两臂相同:GRIP_PAYLOAD_ENVELOPE=0.5 / NMPC_GEOM_SOURCE=online /
#            DJ_TRACK_MEST=false(只在 online 生效,开了就不是单变量) /
#            USE_MHE=true / MHE_C_XY_EST=true / xi=OFF / GEOM_COUPLED=0 / MHE_N=20 /
#            drop=OFF(全程带载,要的是稳态 m_est)
#
# ── 判定规则(跑前钉死,不许事后换)──────────────────────────────────────────
#   ① 前置硬条件(一票否决):全轮 ok、无发散(peak_pos_err<2m)、无 TRACEBACK。
#      沿用 08-25 prior 批那条唯一抓住"均值非劣但 1/8 发散"的判据。
#   ② 主判据:d = |bias_m|(0.7) − |bias_m|(0.5) 的配对 95% CI。
#      bias_m = 稳态段 (m_est − T·cosθ/g) / (T·cosθ/g),从 [mass-diag] 取中位数
#      (⚠️ 用 T·cosθ/g 而不是真值 2.3643 作分母 —— 前者不含任务信息,
#       且 08-28 实测 tilt≈0.2° 时两者只差 0.04%)。
#      CI 不含 0 → 几何标度对 m_est 有显著影响 ⇒ H2' 方向由 d 的符号定。
#      CI 含 0 → 无证据,H2' 判负,回去找别的候选。
#   ③ 次要(只描述不判定):pos_err、solve failed、MHE 侧 dJ 实际值。
#
# 工作点与 run_geomsrc_ab_4ms.sh / run_prior_ab_4ms.sh 完全一致,可跨批比较。
#
# 用法: REPS=6 setsid bash src/scripts/gripper/run_mhe_envelope_ab_4ms.sh > /tmp/menv.log 2>&1 &
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[menv] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/mhe_envelope_ab_4ms.lock"
if ! flock -n 9; then echo "[menv] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-6}"
ARMS="${ARMS:-0.5 0.7}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:--1}"
NMPC_ENV="${NMPC_ENV:-0.5}"

RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math;print(f'{math.sqrt(2)*$R*$W:.2f}')")

STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/mhe_envelope_ab_4ms_${STAMP}.txt"
{
  echo "# MHE 侧几何标度 0.5 vs 0.7  4m/s 配对 A/B  $STAMP"
  echo "# 列: arm rep mhe_env nmpc_env laps nmpc_stamp mhe_stamp status peak_pos_err"
  echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
  echo "#         laps=$LAPS ramp_eff=$RAMP_EFF settle=$SETTLE fly=${FLY}s"
  echo "# 载荷: mass=$MASS ecc=$ECC  drop=OFF  attach_window=140s"
  echo "# 单变量: MHE_PAYLOAD_ENVELOPE (MHE 转动通路几何标度)"
  echo "# 两臂钉死: GRIP_PAYLOAD_ENVELOPE=$NMPC_ENV (NMPC 侧) NMPC_GEOM_SOURCE=online"
  echo "#           DJ_TRACK_MEST=false USE_MHE=true MHE_C_XY_EST=true GEOM_COUPLED=0 MHE_N=20"
  echo "# 判定(钉死): ①全轮ok无发散(一票否决) ②bias_m 配对差 95%CI 是否含 0"
  echo "#   bias_m = median((m_est - T*cos/g)/(T*cos/g)) 取自 [mass-diag] 稳态段(|vz|<0.05)"
  echo "# 动机: 排除推力标定/倾角/阻力后,检验 H2'=MHE 几何过估经 quat 间接压低质量"
} > "$MAN"
echo "[menv] manifest: $MAN"
echo "[menv] v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[menv] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local arm="$1" rep="$2"
  local launch="$RUNDIR/menv_launch_${STAMP}_${arm}_${rep}.log"
  echo "[menv] === arm=$arm rep=$rep (MHE_PAYLOAD_ENVELOPE=$arm, NMPC 侧=$NMPC_ENV) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  MHE_PAYLOAD_ENVELOPE=$arm GRIP_PAYLOAD_ENVELOPE=$NMPC_ENV \
  NMPC_GEOM_SOURCE=online DJ_TRACK_MEST=false \
    GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC \
    USE_MHE=true MHE_C_XY_EST=true \
    GRIP_DYNAMIC=true GRIP_DYN_R=$R GRIP_DYN_W=$W GRIP_DYN_RAMP=$RAMP \
    GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$LIFT_DUR \
    GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" mhe="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    mhe=$(grep -oE "/home/[^ ]*grip_mhe_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[menv] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $arm $NMPC_ENV $LAPS - - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp mstamp
  nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")
  mstamp=$(echo "$mhe"  | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[menv] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $arm $NMPC_ENV $LAPS $nstamp $mstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[menv]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $arm $NMPC_ENV $LAPS $nstamp $mstamp $status $peak" >> "$MAN"
  echo "[menv]   $arm rep$rep nstamp=$nstamp peak=$peak -> $status"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[menv] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[menv] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_mhe_envelope_ab.py $MAN"
