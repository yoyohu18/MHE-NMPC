#!/bin/bash
# dJ 误差方向 A/B:dj_track_mest off vs on,4m/s 工作点配对 (2026-08-31, n=8/臂)
#
# ── 要回答的问题 ──────────────────────────────────────────────────────────
#   模型侧 dJ 的两种给法,哪种对闭环更好 —— 或者根本区分不出?
#     off(历史默认,1023 轮正式批次都是它):dJ = 包线常值 0.0889,**全程 +54%**
#     on (2026-08-30 改成的新默认)      :attach 用地板 0.0309(**−47%**),
#                                        ~3s 后棘轮接管,终值散布 **±10pp**
#                                        (实测三轮 −5.3% / +0.4% / +10.4%)
#   ⚠️ 这件事**至今没有任何实验支撑**。08-25 只证明了"全程 dJ=0 会发散"
#     (1/7 轮 15.1m),没有回答"+54% 与 −47% 哪个方向更糟"。而 08-30 改默认值
#     时依据只有 5 轮 n=1 观察,没达到 [evidence-bar] 的门槛 —— 本批就是补这个。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = DJ_TRACK_MEST。两臂显式钉死相同(不吃默认值,防以后默认再改):
#     NMPC_GEOM_SOURCE=online / GRIP_PAYLOAD_ENVELOPE=0.5 / USE_MHE=true
#     MHE_C_XY_EST=true / GRIP_DROP_AFTER=0(与设定噪声底的 geomsrc 批同配置,
#     可跨批比较;drop 通路由看门狗批次另行覆盖)
#   ⚠️ 增益侧两臂天然相同:它恒按包线上界整定,且棘轮在默认参数下够不着滞回
#     门槛(0.125 vs cap 0.6),所以本批测的是**纯模型侧 dJ** 的影响。
#
# ── 判定规则(跑前钉死,不许事后换)──────────────────────────────────────────
#   ① 前置硬条件(一票否决):16/16 全 ok、无发散(peak pos_err<2m)、无 TRACEBACK。
#   ② 主判据:**LIFT 段(attach → attach+20s)pos_err 峰值**的配对差,双侧 95% CI。
#      选它的两个理由:
#        机理 — LIFT 暂态正是载荷离地、惯量真正出现的窗口,也是两臂 dJ 差最大
#               的窗口(off 全程 +54%;on 在这段从 −47% 往上爬);
#        统计 — 08-28 geomsrc online 臂 n=7 实测批内 CV **2.6%**(均值 0.783,
#               SD 0.0202),而 figure-8 段 pos_err 中位数 CV 11.8%。
#      **功效声明(跑前算好,供 null 结果解释)**:n=8 配对、双侧 0.05、80% 功效
#      可检出 **0.029 m ≈ 3.7% 相对**。小于这个的差异本批看不见,不能声称"没差"。
#   ③ 次要(只描述不判定):figure-8 段 pos_err 中位数/p90、ω 跟踪误差 |err| 中位数、
#      solve failed、m_est bias、on 臂的 dJ 终值散布。
#
#   **措辞纪律**(教训见 cem-benefit-refuted):若 CI 含 0,结论只能写
#   "在 3.7% 分辨率下未检出差异",不能写"两者等价"或"改默认无害"。
#   若 off 显著更好 → 回退默认(改 acados_nmpc_node.py 的 dj_track_mest 声明)。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_djtrack_ab_4ms.sh > /tmp/dj_ab.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_djtrack_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

# --- 跨工作区守卫 ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[dj] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/djtrack_ab_4ms.lock"
if ! flock -n 9; then echo "[dj] 已有实例在跑,退出。"; exit 1; fi

# --- 排队:SITL 栈是跨会话单例,等别的批次交出来(最多 40 min)---
# 教训见 sitl-concurrent-session-hazard / sitl-batch-pitfalls:别的会话可能正在跑
# 批次,直接起会互杀。清完还要静置复检 —— headless 子脚本可能还在起节点。
echo "[dj] 等待 SITL 栈空闲(最多 40min)..."
for i in $(seq 1 480); do
  busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl|run_payload_lost_test.sh|run_fig_round.sh" | wc -l)
  [ "$busy" -eq 0 ] && break
  sleep 5
done
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[dj] 栈仍被占用,放弃"; exit 2; }
echo "[dj] 栈空闲 @ $(date +%H:%M:%S),静置 30s 复检"; sleep 30
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim" | wc -l)
[ "$busy" -ne 0 ] && { echo "[dj] 复检不通过,放弃"; exit 2; }

REPS="${REPS:-8}"
ARMS="${ARMS:-off on}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
ENVELOPE="${ENVELOPE:-0.5}"
LAPS="${LAPS:-3}"; SETTLE="${SETTLE:-3.0}"; RAMP="${RAMP:-9.36}"
FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
VPEAK=$(python3 -c "import math; print(f'{math.sqrt(2)*$R*$W:.2f}')")
STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/djtrack_ab_4ms_$STAMP.txt"

{
echo "# dJ 误差方向 A/B (dj_track_mest off vs on)  $STAMP"
echo "# 列: arm rep dj_track ramp settle laps nmpc_stamp status peak_pos_err"
echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR"
echo "#         laps=$LAPS ramp=$RAMP settle=$SETTLE fly=${FLY}s"
echo "# 载荷: mass=$MASS ecc=$ECC envelope=$ENVELOPE drop=OFF attach_window=140s"
echo "# 单变量: DJ_TRACK_MEST  off=模型dJ恒为包线0.0889(+54%)  on=地板0.0309(-47%)→棘轮"
echo "# 两臂相同: NMPC_GEOM_SOURCE=online USE_MHE=true MHE_C_XY_EST=true GRIP_DROP_AFTER=0"
echo "# 判定(钉死): ①16/16全ok无发散(一票否决) ②LIFT段(attach→+20s)pos_err峰值配对差 双侧95%CI"
echo "#   功效: n=8配对/80%可检出 0.029m(3.7%);批内噪声取自 geomsrc online 臂 n=7 (SD 0.0202)"
echo "# 措辞纪律: CI含0 只能写\"3.7%分辨率下未检出差异\",不能写\"等价/改默认无害\""
} > "$MAN"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[dj] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local arm="$1" rep="$2"
  local launch="$RUNDIR/dj_launch_${STAMP}_${arm}_${rep}.log"
  echo "[dj] === arm=$arm rep=$rep (DJ_TRACK_MEST=$arm) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  DJ_TRACK_MEST=$arm \
  NMPC_GEOM_SOURCE=online \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC GRIP_PAYLOAD_ENVELOPE=$ENVELOPE \
    USE_MHE=true MHE_C_XY_EST=true \
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
    echo "[dj] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $arm $RAMP $SETTLE $LAPS - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  local waited=0 got=0
  while [ "$waited" -lt 150 ]; do
    grep -aq "DYNAMIC: switch to figure8" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[dj] $arm rep$rep FAIL: 150s 内没进 DYNAMIC"
    echo "$arm $rep $arm $RAMP $SETTLE $LAPS $nstamp NODYN 99" >> "$MAN"; cleanup; return; fi
  echo "[dj]   已进 DYNAMIC,飞 ${FLY}s"
  sleep "$FLY"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $arm $RAMP $SETTLE $LAPS $nstamp $status $peak" >> "$MAN"
  echo "[dj]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[dj] 工作点 v_peak=${VPEAK}m/s, 每轮飞 ${FLY}s, 共 $((REPS*2)) 轮"
# 交替臂序:抗时间漂移(机器负载/温度等)在两臂间的系统性分配
for rep in $(seq 1 "$REPS"); do
  for arm in $ARMS; do run_one "$arm" "$rep"; done
done
echo "[dj] 批次完成 -> $MAN"
