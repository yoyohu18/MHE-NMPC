#!/bin/bash
# ============================================================================
# ⚠️⚠️ 本脚本已**作废**(2026-08-26 去先验改造)——仅供论文复现溯源,勿再运行。
# 它的研究对象(载荷质量先验:grip_payload_prior / grip_gain_prior /
# grip_geom_mp_prior / grip_geom_mp_floor / confirm_payload_prior /
# grip_true_payload_mass)已从 acados_nmpc_node.py 与 mhe_node.py 中**全部删除**,
# 理由:本项目前提是不能知道载荷质量,否则在线质量估计失去意义。
# 现在两个节点里唯一的载荷质量信息是 grip_payload_envelope(机架规格包线上界)。
# 脚本正文保持 08-26 之前的原样,以便按当时的配置解读历史批次数据。
# ============================================================================
# NMPC 的 dJ 先验能不能直接去掉:prior=0 vs prior=0.3 配对 A/B(2026-08-25,n>=8)
#
# **单变量**:唯一差异是 `GRIP_GEOM_MP_PRIOR`(模型侧 dJ),两臂的
#   `GRIP_GAIN_PRIOR=0.3` **相同** —— 这是 08-25 拆开 grip_payload_prior 后才可能做到的
#   单变量(拆之前动 prior 会连 PX4 内环增益一起动,08-24 那次 MHE 62 次失败就是
#   因为增益只放大 3.18× 而非 6.68×,根本没测到 dJ 那一维)。
#   xi(tau_lumped)**全程关闭** —— 08-25 四格实测判负(零收益、一格灾难)。
#   `MHE_SEED_FROM_THRUST=0` 两臂相同 —— 把同日落地的 #4(MHE 质量种子改 T_phys/g)
#   钉在历史行为上,使本批可与 08-24 的 λ 批跨批比较。#4 单独验,不混进来。
#
# 【2026-08-25 更正一处口径】拆分前 `GRIP_GEOM_MP_PRIOR` 这一个变量同时喂
#   NMPC 的 grip_payload_prior **和** MHE 的 grip_geom_mp_prior 两条路,所以本
#   脚本原来的"单变量"声明并不严格(prior=0 那臂 MHE 侧会落到 floor=0.15,
#   prior=0.3 那臂是 0.3)。08-25 已把两路拆成 GRIP_NMPC_MP_PRIOR /
#   GRIP_MHE_MP_PRIOR,本脚本传的 GRIP_GEOM_MP_PRIOR 现在**只**影响 NMPC 侧,
#   MHE 侧两臂同为 $GRIP_PAYLOAD_KG —— 从这一版起才真正是单变量。
#
# **判定规则(跑前钉死,不许事后换)**——注意这是**非劣性**检验不是优效性检验:
#   目标是"去掉先验且不付代价",prior=0 不需要更好,只需要不显著更差。
#   ① 前置硬条件(一票否决):16/16 全部 `ok`、无发散(pos_err<2m)、无触界。
#      "能飞"是前提;只要有一轮发散,"去掉先验"就不能写进论文,bias 再好也没用。
#   ② 主判据:|bias_pct| 配对差 d = |bias|(prior0) − |bias|(prior0.3),
#      **等效边界 δ = 1.0 个百分点**(依据:昨天 λ=1.0 臂 n=8 的 bias 范围
#      −2.21~−3.40 跨度 1.19pp,批内噪声本来就这么大)。
#      d 的 95% CI 上界 < +1.0pp → **非劣,先验可以去掉**;否则证据不足。
#   ③ 次要(只描述不判定):pos_err、iqr、solve failed。
#
# 工作点/载荷/drop/attach_window 与 run_lambda_ab_4ms.sh 完全一致,可跨批比较。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_prior_ab_4ms.sh > /tmp/p4ms.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_prior_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

exec 9>"/tmp/prior_ab_4ms.lock"
if ! flock -n 9; then echo "[p4] 已有实例在跑,退出。"; exit 1; fi

REPS="${REPS:-8}"
ARMS="${ARMS:-prior0.3 prior0.0}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:--1}"
# 内环增益整定的**来源**(2026-08-25 拆分后新增)。默认逐位兼容旧行为:
#   GAIN_PRIOR=0.3 / GAIN_ENVELOPE=-1.0 → 点估计先验(任务信息)。
# 换成 GAIN_ENVELOPE=0.5 GAIN_PRIOR=-1.0 → 机架规格包线上界(非任务信息)。
#   ⚠️ 本工作点 m_p=0.3:point 裸 ratio 5.075、envelope(0.5) 7.262,**都撞 cap 5.0**
#      → 两种来源给出**逐位相同**的 MC_*RATE_K。换包线**不改变任何物理**,
#      只是把配置里最后一处任务信息去掉,使"prior=0 这一臂完全不含载荷质量信息"
#      这句话在口径上成立;也因此本批与 08-25 GAIN_PRIOR=0.3 批仍可跨批比较。
#      (包线本身要不要付代价,必须在**轻载**测,见 run_gain_envelope_ab.sh。)
GAIN_PRIOR="${GAIN_PRIOR:-0.3}"
GAIN_ENVELOPE="${GAIN_ENVELOPE:--1.0}"

RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math;print(f'{math.sqrt(2)*$R*$W:.2f}')")

GAIN_SRC_DESC=$(python3 -c "
gp=$GAIN_PRIOR; ge=$GAIN_ENVELOPE
print(f'GAIN_ENVELOPE={ge}(机架规格,非任务信息)' if ge>=0 else f'GAIN_PRIOR={gp}(点估计,任务信息)')")
GAIN_RATIOS=$(python3 -c "
mb,arm,J=2.0643,0.47,0.0142
f=lambda mp:(J+(mb*mp/(mb+mp))*arm**2)/J
gp=$GAIN_PRIOR; ge=$GAIN_ENVELOPE
mp = ge if ge>=0 else gp
print(f'm_p_gain={mp} 裸ratio={f(mp):.3f} capped={min(f(mp),5.0):.3f} (cap 5.0, 临界载荷 0.2937kg)')")

STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/prior_ab_4ms_${STAMP}.txt"
{
  echo "# dJ 先验去除 4m/s 配对 A/B  $STAMP"
  echo "# 列: arm rep prior ramp settle laps nmpc_stamp status peak_pos_err"
  echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
  echo "#         laps=$LAPS ramp_eff=$RAMP_EFF settle=$SETTLE fly=${FLY}s"
  echo "# 载荷: mass=$MASS ecc=$ECC  drop=OFF  attach_window=140s"
  echo "# 两臂相同: 增益整定=$GAIN_SRC_DESC xi=OFF GEOM_COUPLED=0 MHE_N=20 SEED_FROM_THRUST=0"
  echo "#   增益比: $GAIN_RATIOS"
  echo "# 判定(钉死): ①16/16全ok无发散无触界(一票否决) ②|bias|配对差 95%CI上界<+1.0pp=非劣"
} > "$MAN"
echo "[p4] manifest: $MAN"
echo "[p4] 工作点 v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[p4] interrupted"; cleanup; exit 130' INT TERM

cleanup   # 起批前先清干净(跨工作区残留会抢 14540 端口)

run_one() {
  local arm="$1" rep="$2" pri="${1#prior}"
  local launch="$RUNDIR/p4_launch_${STAMP}_${arm}_${rep}.log"
  echo "[p4] === arm=$arm rep=$rep (模型 dJ prior=$pri, 增益 $GAIN_SRC_DESC) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  GRIP_GEOM_MP_PRIOR=$pri GRIP_GAIN_PRIOR=$GAIN_PRIOR GRIP_GAIN_ENVELOPE=$GAIN_ENVELOPE \
    MHE_SEED_FROM_THRUST=0 \
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
    echo "[p4] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $pri $RAMP_EFF $SETTLE $LAPS - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[p4] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $pri $RAMP_EFF $SETTLE $LAPS $nstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[p4]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $pri $RAMP_EFF $SETTLE $LAPS $nstamp $status $peak" >> "$MAN"
  echo "[p4]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

# 每 rep 轮转臂序,抵消机器热身/漂移。用 python 生成,别在 shell 里手搓位置参数轮转。
for rep in $(seq 1 "$REPS"); do
  order=$(python3 -c "
arms='''$ARMS'''.split()
k=($rep-1)%len(arms)
print(' '.join(arms[k:]+arms[:k]))")
  echo "[p4] rep=$rep 顺序: $order"
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[p4] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_prior_ab.py $MAN"
