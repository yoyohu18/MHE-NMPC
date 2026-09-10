#!/bin/bash
# NMPC 几何来源 truth vs online:4m/s 工作点配对 A/B (2026-08-28, n>=8)
#
# ── 为什么现在做 ──────────────────────────────────────────────────────────
#   08-26 去先验改造后,通往 NMPC 的四条路里 MHE 侧已经洗干净(几何标度只剩
#   grip_payload_envelope 机架规格包线),但 **run_gripper_headless.sh 的默认值
#   仍是 geom_source:=truth** —— 该分支的 _payload_geometry 用的是
#   self.grip_payload_mass(载荷**真值**,acados_nmpc_node.py:1699)。
#   近三周所有正式批次(prior_ab_4ms / mhe_prior_ab_4ms / gain_envelope_ab)都
#   显式传 NMPC_GEOM_SOURCE=online 覆盖掉它,累计 49 轮;默认 truth 现在只在
#   "没人传这个变量"时生效 —— 即单轮 viz 调试,以及新脚本忘了传的时候。
#   那是个静默陷阱:忘传 = 实验悄悄拿到载荷真值。本批为改掉这个默认值取证。
#
#   ⚠️ 已有的 49 轮 online 不能直接当证据:它们的"两臂相同"里 geom_source 是
#      **常量**,没有同工作点的 truth 对照臂。本批就是补这个对照。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = NMPC_GEOM_SOURCE:
#     truth : dJ/c_est 由 attach_offset 真值几何 + grip_payload_mass 真值质量算
#             (对照;**含任务信息,不是可接受落点**)
#     online: attach 瞬间 dJ/增益 用包线上界(机架规格)初始化,c_est 随后吃
#             /acados_nmpc/c_xy_est(τ_phys 电机转速反算)在线精修(目标)
#   两臂显式钉死相同(不吃默认值,防漂移):
#     GRIP_PAYLOAD_ENVELOPE=0.5(当前唯一的载荷质量信息,两臂都给)
#     DJ_TRACK_MEST=false —— ⚠️ 必须关:它只在 geom_source=online 下生效
#       (headless:133),开了就不是单变量,而是两个变量一起动。
#     USE_MHE=true / MHE_C_XY_EST=true(truth 臂也要算 c_xy_est,否则没有对表数据)
#     xi=OFF / GEOM_COUPLED=0 / MHE_N=20 / DROP=OFF
#
# ── 判定规则(跑前钉死,不许事后换)——**非劣性**检验,不是优效性 ─────────────
#   目标"去掉真值不付代价":online 不需要更好,只需不显著更差。
#   ① 前置硬条件(一票否决):16/16 全 ok、无发散(peak_pos_err<2m)、无 TRACEBACK。
#      ★ 这条是 08-25 prior 批唯一抓住"均值非劣但 1/8 发散"的判据,必须留着。
#   ② 主判据:d = |bias|(online) − |bias|(truth) 的 95% CI 上界 < δ=1.0pp → 非劣。
#      δ 沿用 08-25 两批的口径(08-24 λ=1.0 臂 n=8 的 bias 跨度 1.19pp = 批内噪声)。
#   ③ 次要(只描述不判定):pos_err、c_err、solve failed、m_est 收敛时间。
#
# 工作点/载荷/drop/attach_window 与 run_prior_ab_4ms.sh、run_mhe_prior_ab_4ms.sh
# 完全一致,可跨批比较。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_geomsrc_ab_4ms.sh > /tmp/gs4ms.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_prior_ab.py --base truth --test online <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

# --- 跨工作区守卫(共享 px4_sitl_default,他人栈会互杀) ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[gs] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/geomsrc_ab_4ms.lock"
if ! flock -n 9; then echo "[gs] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-8}"
ARMS="${ARMS:-truth online}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:--1}"
ENVELOPE="${ENVELOPE:-0.5}"

RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math;print(f'{math.sqrt(2)*$R*$W:.2f}')")
GAIN_RATIO=$(python3 -c "
mb,arm,J=2.0643,0.47,0.0142
f=lambda mp:(J+(mb*mp/(mb+mp))*arm**2)/J
print(f'包线{$ENVELOPE}: 裸ratio={f($ENVELOPE):.3f} capped={min(f($ENVELOPE),5.0):.3f} | '
      f'真值{$MASS}: 裸ratio={f($MASS):.3f} capped={min(f($MASS),5.0):.3f} (cap 5.0)')")

STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/geomsrc_ab_4ms_${STAMP}.txt"
{
  echo "# NMPC 几何来源 truth vs online 4m/s 配对 A/B  $STAMP"
  echo "# 列: arm rep geom_src ramp settle laps nmpc_stamp status peak_pos_err"
  echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
  echo "#         laps=$LAPS ramp_eff=$RAMP_EFF settle=$SETTLE fly=${FLY}s"
  echo "# 载荷: mass=$MASS ecc=$ECC  drop=OFF  attach_window=140s"
  echo "# 单变量: NMPC_GEOM_SOURCE  truth=attach真值几何+grip_payload_mass真值(对照,含任务信息)"
  echo "#                           online=包线初始化dJ/增益 + c_est吃τ_phys在线精修(目标)"
  echo "# 两臂相同: GRIP_PAYLOAD_ENVELOPE=$ENVELOPE DJ_TRACK_MEST=false(只在online生效,必须关)"
  echo "#           USE_MHE=true MHE_C_XY_EST=true xi=OFF GEOM_COUPLED=0 MHE_N=20"
  echo "#   增益比: $GAIN_RATIO"
  echo "# 判定(钉死): ①$((REPS*2))/$((REPS*2))全ok无发散(一票否决) ②|bias|配对差 95%CI上界<+1.0pp=非劣"
  echo "# 动机: headless 默认仍是 truth,近三周正式批次全靠显式覆盖成 online;本批为改默认值取证"
} > "$MAN"
echo "[gs] manifest: $MAN"
echo "[gs] 工作点 v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"

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
trap 'echo "[gs] interrupted"; cleanup; exit 130' INT TERM

cleanup   # 起批前先清干净(跨工作区残留会抢 14540 端口)

run_one() {
  local arm="$1" rep="$2"
  local launch="$RUNDIR/gs_launch_${STAMP}_${arm}_${rep}.log"
  echo "[gs] === arm=$arm rep=$rep (NMPC_GEOM_SOURCE=$arm) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  NMPC_GEOM_SOURCE=$arm \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC GRIP_PAYLOAD_ENVELOPE=$ENVELOPE \
    USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=false \
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
    echo "[gs] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $arm $RAMP_EFF $SETTLE $LAPS - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[gs] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $arm $RAMP_EFF $SETTLE $LAPS $nstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[gs]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $arm $RAMP_EFF $SETTLE $LAPS $nstamp $status $peak" >> "$MAN"
  echo "[gs]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

# 每 rep 轮转臂序,抵消机器热身/漂移。
for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[gs] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[gs] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_prior_ab.py --base truth --test online $MAN"
