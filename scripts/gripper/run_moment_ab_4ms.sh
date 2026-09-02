#!/bin/bash
# 一阶质量矩增广 A/B(2b),4m/s 工作点配对 (2026-09-02, n=8/臂)
#
# ── 要回答的问题 ─────────────────────────────────────────────────────────
#   self 释放档下 m̂ 撞 m_min 硬下界的现象,能不能靠"把载荷几何从质量里拿出来"
#   根治?现状(cur 臂)模型内只留 _r_p_last、几何存在性只能靠 m_P⁺=(m−m_B)⁺ 熄灭
#   —— 09-01 的 0.15kg/4m/s 批次 16 轮里 15 轮 drop 后至少触界一次(最长连续 ~6s)。
#   凡 m̂=m_min 的样本都是**约束截断值**,不是估计值,只能说"无约束解更低"。
#
# ── 两臂 ─────────────────────────────────────────────────────────────────
#   cur  MHE_ESTIMATE_MOMENT=0                        (现状:coupled + self)
#   mom  MHE_ESTIMATE_MOMENT=1 MHE_MOMENT_A_MODE=frozen MHE_C_XY_FROM_MOMENT=true
#        s=m_P·r_xy 增广为状态(c_xy=s/m_T,m_P 约掉);A=μr_z² 幅值改由包线定成
#        常数(存在性仍由 geom 槽 r_z 门控)。
#   ⚠️ 两个开关必须**同时**开:只增广 s 而 A 仍随 m_P,优化器还留着"抬高 m 稀释
#      幽灵力矩"的第二条杠杆 —— 离线实测那样 drop 后 m̂ 反而爬到 +53%。
#
# ── 离线先验(test_mhe_geom_coupled_standalone.py,self 档 0.15kg,5 seed 全一致)─
#   cur  带载 +4.45% / drop 后 +34.8% / drop 段不入带 / solve fails 2~4
#   mom  带载 −0.04% / drop 后 +0.28% / drop 入带 1.3~1.4s / solve fails 0
#   本批就是把这个离线结论拿到闭环里验 —— 合成 plant 不含气动/内环/时延。
#
# ── 09-02 两轮 smoke 已知(n=1,只作预期,不作结论)───────────────────────────
#   触界:const 与 frozen 两轮都是 0%(现状历史 10 轮里 9 轮触界,占空比中位 16.7%)
#   跟踪:LIFT 峰值 0.757/0.735 vs 现状 0.745;fig8 p50 0.131/0.121 vs 0.129 —— 无差别
#   ⚠️ 代价:带载段 m̂ 偏到 −3.58%(const)/−3.23%(frozen),现状是 +0.13~+0.84%。
#      A 换成 frozen 收不回来 → 不是 A 幅值错,是**结构性**的:转动通路辨识的是
#      乘积 m_P·r_xy,s 一旦独占它,m 就退回平动通路精度(legacy 历史 −2.57%,量级吻合)。
#   ⚠️ 连带:带载 m_p 掉到负值 → 质量域释放判据误触发(早于真 drop 50s/16s)。
#      所以本批 mom 臂的"释放时刻"本身也是观测量,不是可信的卸载判据。
#   本批要回答的就是:这个取舍在 n=8 上有多大、稳不稳。
#
# ── 判定规则(跑前钉死,不许事后换)─────────────────────────────────────────
#   ① 前置硬条件(一票否决):16/16 全 ok、无发散(peak pos_err<2m)、无 TRACEBACK、
#      每轮 attach 与 drop 都实际发生。
#   ② 主判据:**drop 后 30s 窗口的触界占空比**(m̂ ≤ m_min+1e-4 的采样比例)配对差。
#      现状臂预期显著 >0(09-01 批次 15/16 轮触界);mom 臂若归零,n=8 配对符号检验
#      全同向即 p=0.0078。同时报最长连续触界时长(秒)。
#   ③ 次判据(**带载偏差已升级为并列主判据**,见上面 smoke 记录):
#      带载稳态偏差(真值 2.2143kg)的配对差 —— 若 mom 臂显著更差,本方案就是
#      "拿质量精度换触界消失",必须在论文里如实写成取舍,不能只报触界那一半。
#      其余:drop 后 m̂ 相对 m_B 偏差中位数(**触界样本单列**,
#      不混进均值)、带载稳态偏差(真值 2.2143kg)、figure8 段 pos_err 中位/p90、
#      solve failed 计数、mom 臂的 |s| 衰减曲线。
#   ④ 性能不回退:figure8 pos_err 中位数若 mom 臂显著更差,即便触界消失也不改默认。
#
#   **措辞纪律**(教训见记忆 cem-benefit-refuted):CI 含 0 只能写"未检出差异",
#   不能写"等价"。触界占空比是有界比例,配对差用 Wilcoxon 符号秩报。
#
# 用法: REPS=8 setsid bash src/scripts/gripper/run_moment_ab_4ms.sh > /tmp/mom_ab.log 2>&1 &
# 聚合: python3 src/scripts/gripper/aggregate_moment_ab.py <manifest.txt>
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"

# --- 跨工作区守卫(别的工作区在跑 px4/gz 就退,免得互杀)---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[mom] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi

exec 9>"/tmp/moment_ab_4ms.lock"
if ! flock -n 9; then echo "[mom] 已有实例在跑,退出。"; exit 1; fi

# --- 排队:SITL 栈是跨会话单例(记忆 sitl-concurrent-session-hazard)---
echo "[mom] 等待 SITL 栈空闲(最多 40min)..."
for i in $(seq 1 480); do
  busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl|run_gripper_headless.sh" | wc -l)
  [ "$busy" -eq 0 ] && break
  sleep 5
done
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim|make px4_sitl" | wc -l)
[ "$busy" -ne 0 ] && { echo "[mom] 栈仍被占用,放弃"; exit 2; }
echo "[mom] 栈空闲 @ $(date +%H:%M:%S),静置 30s 复检"; sleep 30
busy=$(pgrep -f "px4_sitl_default/bin/px4|gz sim" | wc -l)
[ "$busy" -ne 0 ] && { echo "[mom] 复检不通过,放弃"; exit 2; }

REPS="${REPS:-8}"
ARMS="${ARMS:-cur mom}"
# 配置 = 09-01 主线档(run_cxy_mass_repeat.sh),逐项对齐,便于跨批比较
MASS="${MASS:-0.15}"; ENVELOPE="${ENVELOPE:-0.3}"; ECC="${ECC:-0.10}"
R="${R:-10.0}"; W="${W:-0.283}"; RAMP="${RAMP:-9.36}"; DZ="${DZ:-0.8}"
Z_HIGH="${Z_HIGH:-6.0}"; LIFT_DUR="${LIFT_DUR:-8.4}"
DROP_AFTER="${DROP_AFTER:-55.0}"
# mom 臂的 A 档:frozen=A 用上一窗口 m̂ 现算(窗口内 ∂A/∂m≡0 断杠杆,数值仍跟质量);
# const=A 由包线钉死。09-02 smoke:const 档带载 m̂ −3.58%(现状 +0.4%)且把质量域
# 释放判据推成误触发(早真 drop 50s)—— 因为包线 0.3 对 0.15kg 载荷 A 过估 2×。
A_MODE="${A_MODE:-frozen}"
RECOVER_SEC="${RECOVER_SEC:-30}"     # drop 之后再录这么久(主判据窗口)
TIMEOUT="${TIMEOUT:-300}"            # 等 drop 的上限
STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/moment_ab_4ms_$STAMP.txt"
VPEAK=$(python3 -c "import math; print(f'{math.sqrt(2)*$R*$W:.2f}')")

{
echo "# 一阶质量矩增广 A/B (2b)  $STAMP"
echo "# 列: arm rep nmpc_stamp status peak_pos_err"
echo "# 工作点: r=$R w=$W v_peak=${VPEAK}m/s z_high=$Z_HIGH lift_dur=$LIFT_DUR ramp=$RAMP dz=$DZ"
echo "# 载荷: mass=$MASS envelope=$ENVELOPE ecc=$ECC drop_after=$DROP_AFTER recover=$RECOVER_SEC"
echo "# 释放: MHE geom_release_mode=self + NMPC event; 残差路径堵死; 质量域判据 mp=0.03 persist=20"
echo "# 单变量: MHE_ESTIMATE_MOMENT(+MOMENT_A_MODE=$A_MODE +C_XY_FROM_MOMENT) cur=0 mom=1"
echo "# 主判据: drop 后 ${RECOVER_SEC}s 触界占空比(m<=m_min+1e-4)配对差, Wilcoxon"
echo "# 措辞纪律: CI 含 0 只能写\"未检出差异\";触界样本是约束截断值,不进偏差均值"
} > "$MAN"
echo "[mom] manifest: $MAN"

cleanup() {
  for pat in "run_gripper_headless.sh" "px4_sitl_default/bin/px4" "make px4_sitl" \
             "gz sim" "ruby" "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" \
             "parameter_bridge" "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "MicroXRCEAgent" "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" \
             "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[mom] interrupted"; cleanup; exit 130' INT TERM
cleanup

run_one() {
  local arm="$1" rep="$2" mom=0 amode=coupled cxym=false
  if [ "$arm" = mom ]; then mom=1; amode=$A_MODE; cxym=true; fi
  local launch="$RUNDIR/mom_launch_${STAMP}_${arm}_${rep}.log"
  echo "[mom] === arm=$arm rep=$rep (ESTIMATE_MOMENT=$mom A_MODE=$amode) ==="
  # ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿(ps 看不见,只能 fuser 查)
  MHE_ESTIMATE_MOMENT=$mom MHE_MOMENT_A_MODE=$amode \
  MHE_C_XY_FROM_MOMENT=$cxym MHE_MP_ENVELOPE=$ENVELOPE \
  GRIP_PAYLOAD_KG=$MASS GRIP_PAYLOAD_ENVELOPE=$ENVELOPE GRIP_ECC_Y=$ECC \
    GRIP_DYN_R=$R GRIP_DYN_W=$W GRIP_DYN_RAMP=$RAMP GRIP_DYN_DZ=$DZ \
    GRIP_Z_HIGH=$Z_HIGH GRIP_LIFT_DUR=$LIFT_DUR GRIP_DYNAMIC=true \
    GRIP_DROP_AFTER=$DROP_AFTER GRIP_DROP_AT_TIP=true \
    DJ_TRACK_MEST=true GRIP_DJ_FLOOR_MP=0.05 DJ_RATCHET=false \
    MHE_C_XY_EST=true NMPC_GEOM_SOURCE=online \
    DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
    MHE_CONFIRM_ALPHA=1.5 MHE_RESID_STEP=0 \
    MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self NMPC_GEOM_RELEASE_MODE=event \
    MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
    MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
    EVAL_TRUE_PAYLOAD_MASS=$MASS \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 20); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && [ -f "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "[mom] $arm rep$rep FAIL: 拿不到 nmpc 日志名"; tail -5 "$launch"
    echo "$arm $rep - NOLOG 99" >> "$MAN"; cleanup; return; fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")

  # 等 drop 实际发生,再多录 RECOVER_SEC(主判据窗口就在这一段)
  local waited=0 dropped=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    grep -aq "DROP: released gripper" "$nmpc" 2>/dev/null && { dropped=1; break; }
    sleep 6; waited=$((waited+6))
  done
  local status=ok
  [ "$dropped" -eq 1 ] && sleep "$RECOVER_SEC" || status=NODROP
  grep -aq "Traceback" "$nmpc" && status=TRACEBACK
  local peak
  peak=$(grep -oE "pos_err=[0-9.]+" "$nmpc" 2>/dev/null | grep -oE "[0-9.]+" \
         | sort -rn | head -1); [ -z "$peak" ] && peak=99
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  echo "$arm $rep $nstamp $status $peak" >> "$MAN"
  echo "[mom]   $arm rep$rep stamp=$nstamp peak=$peak -> $status"
  cleanup
}

echo "[mom] 工作点 v_peak=${VPEAK}m/s, 共 $((REPS*2)) 轮"
# 交替臂序:抗时间漂移(机器负载/温度)在两臂间的系统性分配
for rep in $(seq 1 "$REPS"); do
  for arm in $ARMS; do run_one "$arm" "$rep"; done
done
echo "[mom] 批次完成 -> $MAN"
