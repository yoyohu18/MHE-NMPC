#!/bin/bash
# LIFT 抬升速率 A/B:lift_dur 8.4 vs 16.8,4m/s 工作点 (2026-09-16, n=8/臂)
#
# ── 要回答的问题 ──────────────────────────────────────────────────────────
#   "拉长离地过程能不能降低发散率?" 本批**不直接测发散率**(事件率太低,n=8
#   没有功效),只证**机制链的前半段**是否按预期动:
#       抬升慢 → 参考误差累积慢 → 推力爬升慢 → 突破地面平衡时超出量小
#              → 离地瞬间加速度小 → 振荡小
#              → (已知机制 nmpc-divergence-crash) 振荡未平息→推力跌破悬停→穿地
#   机制若成立,再另立批次用足够 n 测发散率;机制若不成立,这条路直接关掉。
#
# ── 单变量 ────────────────────────────────────────────────────────────────
#   唯一差异 = GRIP_LIFT_DUR(8.4 / 16.8),z_high 两臂都钉死 6.0。
#   ⚠️ 必须固定 z_high:否则撞上已知的高度边界混淆(r=10 在 z=2.5 必发散、
#      z=6/10 稳,见 high-maneuver-ablation),那是终点高度的效应不是速率的。
#   抬升速率 (6.0-0.55)/lift_dur = 0.649 vs 0.324 m/s。
#   其余两臂显式钉死相同(不吃默认值):NMPC_GEOM_SOURCE=online /
#   GRIP_PAYLOAD_ENVELOPE=0.5 / USE_MHE=true / MHE_C_XY_EST=true /
#   GRIP_DROP_AFTER=0 / GRIP_LIFT_HOLD 不开(hold 是另一条臂,别和速率混在一起)。
#
# ── 判定规则(跑前钉死,不许事后换)──────────────────────────────────────────
#   ① 前置硬条件(一票否决):16/16 全 ok、无 TRACEBACK。
#      ⚠️ 发散(peak pos_err>2m)**不作一票否决**——它本身就是次要终点,
#         否决掉等于把要观察的现象筛掉了。发散轮记 DIVERGED 但计入分母。
#   ② 主判据:ΔT_overshoot = T_peak(离地±1s) − T_steady(**抬升完成后 0.5~2.5s**)
#      的两臂差,双侧 95% CI(非配对 Welch)。
#      机理:这个量就是"突破地面平衡时超出了多少",是机制链的关键环节,
#            且不随参考变慢而平凡缩小(T_steady 由悬停+载荷定死)。
#      **只在未饱和轮上算**(T_peak<=26N)。推力打到饱和(真实上限 31.4N)时
#      T_peak 变成执行器上限、不再是"突破平衡的超出量",该量失去物理含义;
#      这类轮记 saturated 不进主判据,但**计入发散率分母**。
#      ⚠️ 这是终点定义的一部分,跑前钉死,不是事后筛选。噪声底也是在同一道门
#         下估的,门不一致则功效声明失效。
#      ⚠️ 若两臂 saturated 比例差很大,主判据存在选择偏倚,解读要打折并明写。
#      ⚠️ T_steady 必须取在**抬升完成之后**,且窗口落在 settle(3s)之内:
#         · 取"离地后 5~8s"两臂不可比 —— A(8.4s)那会儿已在 6m 悬停,
#           B(16.8s)还在 s≈0.5 的最大速度段,是两个不同的物理状态;
#         · 取 t_done+2~5s 会撞上 figure-8 的 ramp(历史轮次 settle=3s 后就切),
#           实测把 SD 从 0.823 撑到 2.165(差 2.6x)。
#      ⚠️ 轮次质量门(跑前钉死,两臂对称,剔除率按臂报告):
#         not-low(pre 段不在低空)/unsettled(pre 段 z 抖动>0.02m,LIFT 起点条件
#         不合格)/false-liftoff(t_off<0.3s,判据被抖动触发)/saturated(T_pk>26N)。
#      A 臂噪声底(09-16 天然样本 n=93,同定义同门):均值 1.767 N,SD 0.823。
#      **功效声明**:n=16/臂、双侧 0.05、80% 功效可检出 **0.82 N ≈ 46% 相对**。
#      机制预测:lift_dur 加倍→误差累积速率减半→ΔT_overshoot 约 1.77→0.9 N
#               (降 0.9 N),略大于分辨率。n=8 只能检出 1.15N,不够,故用 n=16。
#   ③ 次要:dT/dt 峰值(A 臂 5.30±3.36 N/s,n=16 可检出 3.32 N/s ≈ 63%)。
#   ④ 操纵检查:t_off(离地时长)。A 臂 1.52±0.20s,n=16 可检出 0.19s。B 臂必须显著更长,
#      否则说明 LIFT_DUR 根本没生效,整批作废。
#   ⑤ 只描述不判定:**LIFT 段**发散率、solve failed、m_est bias。
#      ⚠️ 本批**不跑 figure-8**(GRIP_DYNAMIC=false):主判据全在 LIFT 段,
#         figure-8 的 93s/轮对它零贡献。代价=看不到 figure-8 段的发散,
#         但那本来就要另立批次用足够 n 去测(n=16 在发散率上仍无功效)。
#      ⚠️ LIFT 段 pos_err 峰值**故意不做主判据**:参考变慢本身就让它变小,
#         是同义反复,不能当机制证据。
#
#   **措辞纪律**(教训见 cem-benefit-refuted):若 CI 含 0,结论只能写
#   "在 32% 分辨率下未检出差异",不能写"拉长无效"或"机制证伪"。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_liftdur_ab.sh > /tmp/liftdur_ab.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_liftdur_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

# --- 跨工作区守卫 ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[ld] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/liftdur_ab.lock"
if ! flock -n 9; then echo "[ld] 已有实例在跑,退出。"; exit 1; fi

# --- 排队:SITL 栈是跨会话单例,等别的批次交出来(最多 40 min)---
echo "[ld] 等待 SITL 栈空闲(最多 40min)..."
for i in $(seq 1 480); do
  busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl|run_payload_lost_test.sh|run_fig_round.sh" | wc -l)
  [ "$busy" -eq 0 ] && break
  sleep 5
done
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[ld] 栈仍被占用,放弃"; exit 2; }
echo "[ld] 栈空闲 @ $(date +%H:%M:%S),静置 30s 复检"; sleep 30
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim" | wc -l)
[ "$busy" -ne 0 ] && { echo "[ld] 复检不通过,放弃"; exit 2; }

REPS="${REPS:-16}"
ARMS="${ARMS:-a84 b168}"
MASS="${MASS:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; Z_HIGH="${Z_HIGH:-6.0}"
LIFT_A="${LIFT_A:-8.4}"; LIFT_B="${LIFT_B:-16.8}"
ENVELOPE="${ENVELOPE:-0.5}"
# 不跑 figure-8:每轮从 LIFT 开始只需盖住 抬升(lift_dur) + 到顶后 6s
# (T_steady 窗口 0.5~2.5s + 余量)。TAIL 两臂相同,时长差异全部来自 lift_dur。
TAIL="${TAIL:-8}"
VPEAK=$(python3 -c "import math; print(f'{math.sqrt(2)*$R*$W:.2f}')")
RATE_A=$(python3 -c "print(f'{(6.0-0.55)/$LIFT_A:.3f}')")
RATE_B=$(python3 -c "print(f'{(6.0-0.55)/$LIFT_B:.3f}')")
STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/liftdur_ab_$STAMP.txt"

{
echo "# LIFT 抬升速率 A/B (lift_dur $LIFT_A vs $LIFT_B)  $STAMP"
echo "# 列: arm rep lift_dur ramp settle laps nmpc_stamp status peak_pos_err"
echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH (两臂固定)"
echo "#         抬升速率 a84=${RATE_A}m/s  b168=${RATE_B}m/s"
echo "#         figure-8=OFF (GRIP_DYNAMIC=false), 每轮 LIFT 后再飞 lift_dur+${TAIL}s"
echo "# 载荷: mass=$MASS ecc=$ECC envelope=$ENVELOPE drop=OFF attach_window=140s"
echo "# 单变量: GRIP_LIFT_DUR  a84=$LIFT_A  b168=$LIFT_B"
echo "# 两臂相同: NMPC_GEOM_SOURCE=online USE_MHE=true MHE_C_XY_EST=true"
echo "#           GRIP_DROP_AFTER=0 GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_HOLD=off"
echo "# 判定(钉死): ①16/16全ok无TRACEBACK(发散不否决,计入分母)"
echo "#   ②主: ΔT_overshoot = T_peak(离地±1s) - T_steady(离地后5~8s) 两臂差 95%CI"
echo "#   功效: A臂噪声底 2.228±0.511N (n=89 天然样本); n=8/臂 80% 可检出 0.72N (32%)"
echo "#   ③次: dT/dt 峰 (A臂 4.76±1.41 N/s, 可检出 1.98N/s)"
echo "#   ④操纵检查: t_off (A臂 1.54±0.13s), B臂不显著更长则整批作废"
echo "#   ⑤只描述: 发散率/solve failed/m_est bias; pos_err峰值不做主判据(同义反复)"
echo "# 措辞纪律: CI含0 只能写\"32%分辨率下未检出差异\",不能写\"拉长无效\""
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
trap 'echo "[ld] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local arm="$1" rep="$2" ldur
  case "$arm" in
    a84)  ldur="$LIFT_A" ;;
    b168) ldur="$LIFT_B" ;;
    *) echo "[ld] 未知臂 $arm"; return ;;
  esac
  local launch="$RUNDIR/ld_launch_${STAMP}_${arm}_${rep}.log"
  echo "[ld] === arm=$arm rep=$rep (GRIP_LIFT_DUR=$ldur) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 里看不见,只能 fuser 查)
  NMPC_GEOM_SOURCE=online \
  GRIP_PAYLOAD_KG=$MASS GRIP_ECC_Y=$ECC GRIP_PAYLOAD_ENVELOPE=$ENVELOPE \
    USE_MHE=true MHE_C_XY_EST=true \
    GRIP_DYNAMIC=false \
    GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$ldur \
    GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[ld] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep $ldur na na na - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  # 等 LIFT 起步(不再等 figure-8)。两臂等的是同一个事件,不引入臂间差异。
  local waited=0 got=0
  while [ "$waited" -lt 200 ]; do
    grep -aq "LIFT: raising hover" "$nmpc" 2>/dev/null && { got=1; break; }
    sleep 5; waited=$((waited+5))
  done
  if [ "$got" -eq 0 ]; then
    echo "[ld] $arm rep$rep FAIL: 200s 内没进 LIFT"
    echo "$arm $rep $ldur na na na $nstamp NOLIFT 99" >> "$MAN"; cleanup; return; fi
  local dur; dur=$(python3 -c "print(int($ldur+$TAIL))")
  echo "[ld]   已进 LIFT,再飞 ${dur}s (lift_dur=$ldur + tail=$TAIL)"
  sleep "$dur"

  local status=ok peak
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  # DIVERGED 只是标记,不否决整批(见头部判定规则 ①)
  awk "BEGIN{exit !($peak>2.0)}" && [ "$status" = ok ] && status=DIVERGED
  echo "$arm $rep $ldur na na na $nstamp $status $peak" >> "$MAN"
  echo "[ld]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[ld] 抬升 ${RATE_A} vs ${RATE_B} m/s, figure-8 OFF, 共 $((REPS*2)) 轮"
# 交替臂序:抗时间漂移(机器负载/温度等)在两臂间的系统性分配
for rep in $(seq 1 "$REPS"); do
  for arm in $ARMS; do run_one "$arm" "$rep"; done
done
echo "[ld] 批次完成 -> $MAN"
