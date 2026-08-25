#!/bin/bash
# PX4 内环增益整定:用载荷**包线上界**(机架规格)代替**点估计先验**(任务信息)
# 配对 A/B(2026-08-25,n>=8)。"去先验"路线图第 #2 项 / 方案 b。
#
# ── 为什么必须在**轻载**跑 ────────────────────────────────────────────────
#   _scale_px4_rate_gains 的 ratio=(Jxx+dJ)/Jxx 有 **cap 5.0**,而
#   dJ(m_p)=[m_B·m_p/(m_B+m_p)]·d²(m_B=2.0643, d=0.47, Jxx=0.0142)
#   → 撞 cap 的临界载荷 **m_p* = 0.2937 kg**(解析解,已锁进
#     test/test_gain_envelope_standalone.py)。
#   主线的 0.3kg 工况裸 ratio 5.075 **已经在 cap 里**,那里包线与点估计给出
#   **逐位相同**的 MC_*RATE_K —— 测不出任何东西。
#   本批取 **m_p=0.2kg**:裸 ratio 3.836,不在 cap 里,包线会把增益一路提到 5.0
#   (+30%)。这是唯一能真正检验"换包线要不要付代价"的工作点。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = 内环增益整定的**来源**:
#     point0.2 : GRIP_GAIN_PRIOR=0.2     → ratio 3.836  (任务信息:知道这盒子多重)
#     env0.5   : GRIP_GAIN_ENVELOPE=0.5  → ratio 5.000  (机架规格:最多吊得动多少)
#   两臂**显式**钉死相同(不吃任何默认值,防默认漂移):
#     GRIP_GEOM_MP_PRIOR=0.2(模型侧 dJ) / MHE_SEED_FROM_THRUST=0(钉住 #4)
#     xi=OFF / GEOM_COUPLED=0 / MHE_N=20 / GEOM_MP_FLOOR=0.15 / GEOM_SOURCE=online
#
# ── 判定规则(跑前钉死,不许事后换)——**非劣性**检验,不是优效性 ─────────────
#   目标:"用包线代替点估计**不付代价**"。env 臂不需要更好,只需要不显著更差。
#   风险侧是**过增益**(env 臂增益高 30%):内环高频振荡 → 姿态抖 → 跟踪退化/发散。
#
#   ① 前置硬条件(一票否决):16/16 全 `ok`、无发散(peak_pos_err<2m)、无触界。
#      过增益若有害,最先在这里现形。任一轮发散 → 方案 b 不成立,②③再好也没用。
#   ② 主判据:d = pos_err中位(env0.5) − pos_err中位(point0.2),
#      **等效边界 δ = +0.015 m**。
#      δ 的依据:08-25 prior 批(20260825_144337,n=7/臂)的 pos_err 批内范围
#      0.115~0.122 与 0.108~0.115,跨度均 0.007m;取 2× 批内跨度。
#      ⚠️ 已知弱点:那批是 0.3kg 工况,本批 0.2kg 的批内噪声未必相同。若本批
#         实测批内跨度显著大于 0.007m,δ 需按**本批 point 臂**的跨度重算并
#         在报告里显式声明——但**只允许放大 δ 之前先声明**,不许看了 d 再调。
#      d 的 95% CI 上界 < δ → **非劣,包线可以取代点估计先验**;否则证据不足。
#   ③ 过增益的**直接**证据(描述性,不做硬判定——没有历史基线可定 δ):
#      body 角速度 ω 的一阶差分 RMS(figure8 稳态段),报告 env/point 比值。
#      比值≈1 → 没引入额外抖动;比值明显>1 → 即使 ①② 都过也要在论文里如实写。
#      为此本批**开启** residual 采集(RESID_LOG_DIR),拿 31Hz 的原始 body ω。
#   ④ 次要(只描述):solve failed、peak、m_est bias/iqr。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_gain_envelope_ab.sh > /tmp/genv.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_gain_envelope.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

exec 9>"/tmp/gain_envelope_ab.lock"
if ! flock -n 9; then echo "[ge] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-8}"
ARMS="${ARMS:-point0.2 env0.5}"
MASS="${MASS:-0.2}"; ECC="${ECC:-0.10}"
POINT_PRIOR="${POINT_PRIOR:-0.2}"     # 点估计臂:等于载荷真值(对 baseline 最有利)
ENVELOPE="${ENVELOPE:-0.5}"           # 包线臂:机架规格上界
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:--1}"

RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math;print(f'{math.sqrt(2)*$R*$W:.2f}')")
RATIOS=$(python3 -c "
mb,arm,J=2.0643,0.47,0.0142
f=lambda mp:(J+(mb*mp/(mb+mp))*arm**2)/J
print(f'point={min(f($POINT_PRIOR),5.0):.3f} env={min(f($ENVELOPE),5.0):.3f}')")

STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/gain_envelope_ab_${STAMP}.txt"
RESID_BASE="$RUNDIR/resid_genv_${STAMP}"
{
  echo "# 增益整定 包线上界 vs 点估计先验  配对 A/B  $STAMP"
  echo "# 列: arm rep gain_src ramp settle laps nmpc_stamp status peak_pos_err"
  echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
  echo "#         laps=$LAPS ramp_eff=$RAMP_EFF settle=$SETTLE fly=${FLY}s"
  echo "# 载荷: mass=$MASS ecc=$ECC  drop=OFF  attach_window=140s"
  echo "#   ⚠️ 轻载工作点:m_p=$MASS < 临界 0.2937kg,故两臂增益**确实不同**: $RATIOS"
  echo "# 两臂: point0.2=GAIN_PRIOR($POINT_PRIOR,任务信息) env0.5=GAIN_ENVELOPE($ENVELOPE,机架规格)"
  echo "# 两臂相同: GEOM_MP_PRIOR=$MASS SEED_FROM_THRUST=0 xi=OFF GEOM_COUPLED=0 MHE_N=20"
  echo "# 判定(钉死): ①16/16全ok无发散(一票否决) ②pos_err配对差 95%CI上界<+0.015m=非劣"
  echo "#             ③ω高频能量比值(描述性) ④solve failed/bias(描述性)"
  echo "# resid 采集: $RESID_BASE/<arm>_<rep>/"
} > "$MAN"
echo "[ge] manifest: $MAN"
echo "[ge] 工作点 v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"
echo "[ge] 增益比 $RATIOS  (临界载荷 0.2937kg,本批 $MASS kg 在临界以下=可测)"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[ge] interrupted"; cleanup; exit 130' INT TERM

cleanup   # 起批前先清干净(跨工作区残留会抢 14540 端口)

run_one() {
  local arm="$1" rep="$2"
  local launch="$RUNDIR/ge_launch_${STAMP}_${arm}_${rep}.log"
  local resid="$RESID_BASE/${arm}_${rep}"
  # 两臂的增益来源:point 走 GAIN_PRIOR,env 走 GAIN_ENVELOPE(优先级高,见 _gain_prior)
  local gp ge
  if [ "$arm" = "env0.5" ]; then gp="-1.0"; ge="$ENVELOPE"
  else                           gp="$POINT_PRIOR"; ge="-1.0"; fi
  echo "[ge] === arm=$arm rep=$rep (gain_prior=$gp gain_envelope=$ge) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  GRIP_GAIN_PRIOR=$gp GRIP_GAIN_ENVELOPE=$ge \
    GRIP_GEOM_MP_PRIOR=$MASS MHE_SEED_FROM_THRUST=0 \
    GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=online \
    GRIP_DYNAMIC=true GRIP_DYN_R=$R GRIP_DYN_W=$W GRIP_DYN_RAMP=$RAMP \
    GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$LIFT_DUR \
    GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS RESID_LOG_DIR="$resid" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[ge] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $gp/$ge $RAMP_EFF $SETTLE $LAPS - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[ge] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $gp/$ge $RAMP_EFF $SETTLE $LAPS $nstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[ge]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  # 记录该轮真实生效的增益(核对包线确实走了 envelope 路径,不是静默回落)
  grep -a "RATE_K:" "$nmpc" 2>/dev/null | head -1 | sed 's/^/[ge]   gain: /'
  echo "$arm $rep $gp/$ge $RAMP_EFF $SETTLE $LAPS $nstamp $status $peak" >> "$MAN"
  echo "[ge]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

# 每 rep 轮转臂序,抵消机器热身/漂移。用 python 生成,别在 shell 里手搓位置参数轮转。
for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[ge] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[ge] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_gain_envelope.py $MAN"
