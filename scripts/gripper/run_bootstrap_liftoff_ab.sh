#!/bin/bash
# J bootstrap 武装时机 A/B:attach 时 vs 真离地时 (2026-09-16, n=8/臂, 探索性)
#
# ── 要回答的问题 ──────────────────────────────────────────────────────────
#   把 J bootstrap 的武装从"attach 指令发出"推迟到"box 真离地",能不能消掉
#   attach 后 LIFT 前的悬停失稳?
#   根因见 memory attach-prelift-hover-instability:现状在 attach 瞬间 arm,
#   dJ 斜坡 1.4s 把 omega_scale 顶到 cap 5.0;而 box 要到 attach+4.6s 才离地。
#   这中间飞机是被地面按住的约束系统(模型里没有),增益拉满 + 接触随时崩开
#   = 经典 constrained->free 转换失稳。32 轮实测 7 轮失稳/3 轮穿地。
#
# ── ⚠️ 本批是探索性,不是确证性 ────────────────────────────────────────────
#   失稳率基础率 22%(7/32)。n=8/臂 时:即便新版把失稳降到 0,0/8 vs 2/8 的
#   Fisher p≈0.47,**根本检不出**。所以本批的目的只有两个:
#     (1) 确认机制在多轮下稳定生效(危险窗口内 omega_scale 峰值);
#     (2) 估 |w| 的效应量,用来给正式批次算 n。
#   **不许**用本批宣称"问题解决了"。要那个结论得按 evidence-bar 另立大批次。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = BOOTSTRAP_ON_LIFTOFF (false=现状 / true=改后)。
#   其余两臂显式钉死:GRIP_LIFT_DUR=8.4 GRIP_Z_HIGH=6.0 GRIP_DYNAMIC=false
#   GRIP_ECC_Y=0.10 (偏心是已知的最强判别量 p=0.0024,必须两臂同设定)
#   NMPC_GEOM_SOURCE=online USE_MHE=true MHE_C_XY_EST=true GRIP_DROP_AFTER=0
#
# ── 终点(跑前钉死)──────────────────────────────────────────────────────────
#   ① 机制生效检查(确定性,n=1 即可判):危险窗口(attach -> 离地)内 omega_scale
#      峰值。off 臂应 ≈5.00,on 臂应 ≈1.00。任一轮不符 = 改动没生效,整批作废。
#   ② 主终点:危险窗口内 **|w| 峰值 与 |w| RMS**,Mann-Whitney 双侧。
#      ⚠️ **不预设方向**:增益高既可能放大 ω_cmd 噪声(on 臂更小),也可能让姿态
#         保持更紧(on 臂更大)。双侧检验,方向由数据说。
#      历史噪声底(上一批 32 轮同配置,仅正常轮 n=25):
#        |w|峰值 0.0930±0.0554 rad/s → n=8/臂 可检出 0.0777 (83%)
#        |w| RMS 0.0259±0.0116 rad/s → n=8/臂 可检出 0.0162 (63%)
#      失稳轮参考:|w|峰值 2.30±1.97、RMS 0.918±0.589(25~35 倍),故一旦有轮次
#      失稳,秩和会强烈反应 —— 这正是选 |w| 而非二值失稳率的理由。
#   ③ 只描述不判定:失稳率(功效不足,见上)、solve failed、穿地数、离地时刻。
#   ④ 兜底监控:on 臂有多少轮是走"超时强制 arm"而不是真检出离地。若兜底比例
#      高,说明离地判据不可靠,①②的解读都要打折。
#
#   **措辞纪律**:失稳率无论好看成什么样,只能写"n=8 功效不足,仅作描述"。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_bootstrap_liftoff_ab.sh > /tmp/bsl_ab.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_bootstrap_liftoff_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[bsl] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/bootstrap_liftoff_ab.lock"
if ! flock -n 9; then echo "[bsl] 已有实例在跑,退出。"; exit 1; fi

echo "[bsl] 等待 SITL 栈空闲(最多 40min)..."
for i in $(seq 1 480); do
  busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
  [ "$busy" -eq 0 ] && break
  sleep 5
done
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[bsl] 栈仍被占用,放弃"; exit 2; }
echo "[bsl] 栈空闲 @ $(date +%H:%M:%S),静置 30s 复检"; sleep 30
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim" | wc -l)
[ "$busy" -ne 0 ] && { echo "[bsl] 复检不通过,放弃"; exit 2; }

REPS="${REPS:-8}"
ARMS="${ARMS:-off on}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
ENVELOPE="${ENVELOPE:-0.5}"
TAIL="${TAIL:-8}"
STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/bootstrap_liftoff_ab_$STAMP.txt"

{
echo "# J bootstrap 武装时机 A/B (attach 时 vs 真离地时)  $STAMP"
echo "# 列: arm rep on_liftoff na na na nmpc_stamp status peak_pos_err"
echo "# 单变量: BOOTSTRAP_ON_LIFTOFF  off=现状(attach 即 arm)  on=推迟到真离地"
echo "# 两臂相同: lift_dur=$LIFT_DUR z_high=$Z_HIGH ecc=$ECC mass=$MASS envelope=$ENVELOPE"
echo "#           figure-8=OFF drop=OFF geom=online MHE+c_xy_est=true"
echo "# ⚠️ 探索性批次:失稳率基础率 22%,n=8/臂 即便降到 0 也 p≈0.47 检不出。"
echo "#    目的只有 (1)确认机制生效 (2)估 |w| 效应量给正式批次算 n。"
echo "# 终点① 机制生效: 危险窗口(attach->离地) omega_scale 峰值, off≈5.00 on≈1.00"
echo "#      任一轮不符 = 改动没生效, 整批作废"
echo "# 终点② 主(双侧,不预设方向): 危险窗口 |w|峰值 与 |w|RMS, Mann-Whitney"
echo "#      噪声底(上批 n=25 正常轮): 峰值 0.0930±0.0554 可检出 0.0777(83%)"
echo "#                               RMS  0.0259±0.0116 可检出 0.0162(63%)"
echo "# 终点③ 只描述: 失稳率/solve failed/穿地/离地时刻   ④ on 臂兜底 arm 比例"
echo "# 措辞纪律: 失稳率只能写\"n=8 功效不足,仅作描述\""
} > "$MAN"

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
trap 'echo "[bsl] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local arm="$1" rep="$2" onlift
  case "$arm" in
    off) onlift=false ;;
    on)  onlift=true ;;
    *) echo "[bsl] 未知臂 $arm"; return ;;
  esac
  local launch="$RUNDIR/bsl_launch_${STAMP}_${arm}_${rep}.log"
  echo "[bsl] === arm=$arm rep=$rep (BOOTSTRAP_ON_LIFTOFF=$onlift) ==="
  BOOTSTRAP_ON_LIFTOFF=$onlift \
  NMPC_GEOM_SOURCE=online \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC GRIP_PAYLOAD_ENVELOPE=$ENVELOPE \
    USE_MHE=true MHE_C_XY_EST=true \
    GRIP_DYNAMIC=false GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$LIFT_DUR \
    GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[bsl] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $onlift na na na - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 200 ]; do
    grep -aq "LIFT: raising hover" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[bsl] $arm rep$rep FAIL: 200s 内没进 LIFT"
    echo "$arm $rep $onlift na na na $nstamp NOLIFT 99" >> "$MAN"; cleanup; return; fi
  local dur; dur=$(python3 -c "print(int($LIFT_DUR+$TAIL))")
  echo "[bsl]   已进 LIFT,再飞 ${dur}s"
  sleep "$dur"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && [ "$status" = ok ] && status=DIVERGED
  echo "$arm $rep $onlift na na na $nstamp $status $peak" >> "$MAN"
  echo "[bsl]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[bsl] bootstrap 武装时机 A/B, 共 $((REPS*2)) 轮 (只到抬升完成,不跑 figure-8)"
for rep in $(seq 1 "$REPS"); do
  for arm in $ARMS; do run_one "$arm" "$rep"; done
done
echo "[bsl] 批次完成 -> $MAN"
