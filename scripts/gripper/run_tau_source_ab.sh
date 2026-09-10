#!/bin/bash
# MHE 已知力矩来源 A/B(2026-09-05)。
#
# 【为什么跑这个】两个入口的默认值长期不一致,谁也没判过:
#   run_sitl_gripper_viz.sh  → 不传参 → 节点默认 tau_source=phys_full, motor_window_avg=True
#   run_gripper_headless.sh  → 显式覆盖 → command / False
# 也就是说**演示看到的**和**批次统计的**不是同一个估计器输入。必须判掉。
#
# 【臂的定义:这是配置包对比,不是单因素】
#   command  : MHE_TAU_SOURCE=command   MHE_MOTOR_AVG=0   (headless 现行默认)
#   physfull : MHE_TAU_SOURCE=phys_full MHE_MOTOR_AVG=1   (节点默认 = viz 实际在跑的)
# 两个变量捆在一起动是**有意**的,理由见 run_gripper_headless.sh 的注释:
#   · phys_full 把三轴力矩换成 250Hz 电机反算值,而 MHE 只有 10Hz —— 端点采样即混叠;
#     2026-08-24 的 coupled+self+phys 就是这么在 LIFT 段发散坠机的,当时留下的
#     待验假说正是"意图值不准但 20Hz 分段常值、与射击区间匹配;tau_phys 准但混叠"。
#   · 所以脚本注释早就钉死"phys_full 必须配 MHE_MOTOR_AVG=1 一起测"。
#   单开 phys_full(avg=0)是已知的坏组合,不值得再花 8 轮去复现。
#   代价:本批**不能**把功劳拆给 tau_source 或 avg 中的某一个;要拆得再跑 2×2。
#
# 【预注册判据,跑前钉死】
#  ① 前置硬条件(一票否决):**DROP 之前**无发散(peak_pre<2m 且 solve failed<=100)。
#     physfull 臂若在 attach/LIFT/figure8 段发散 → 直接判负(= 复现 08-24)。
#     ⚠️ 只看 drop 之前是有依据的、不是事后放宽:同底座的 command 档 12 轮
#     (cxy_mass_repeat 20260905_2003~2043)里有 3 轮在 **drop 后约 13s** 失稳穿地,
#     而它们 drop 前的 peak(0.71~0.74m)与正常轮无法区分。那是本底座已有的
#     ~25% 故障率,与本变量无关,且发生在主判据窗口之后;若一票否决会随机废掉
#     两臂的轮次。drop 后失稳因此单列为次要指标,按臂报发生率。
#  ② 主判据:带载稳态 m̂ 的**有符号**相对偏差(%),窗口 = DROP 前 20s(全在 figure8
#     机动段内,口径与 extract_ab_metrics.py 一致)。配对差 95% CI + 符号检验。
#     选它的理由:tau_source 改的就是 MHE 的 u_known 力矩,而 coupled 档下转动通路
#     承载 ~94% 的质量信息(记忆 mhe-geom-mass-coupling),力矩给错直接进偏差。
#     效应量参照:geom coupled A/B 同工况把偏差从 −2.57% 挪到 +0.45%,n=8 即 p=.0078。
#  ③ 次要(只描述不判定):|err| p50/p90、peak_pos_err、solve failed、solve 耗时
#     (阴性对照:phys_full 不改求解规模,这一列本就该没差)。
#
# 【底座】= 当前 4m/s 主线档,与 run_cxy_mass_repeat.sh 逐字对齐(0.15kg / 包线 0.3 /
#   ecc 0.10 / r=10 w=0.283 ramp=9.36 dz=0.8 / z=6 lift=8.4 / drop@tip 55s /
#   coupled + geom_release=self + NMPC event + DJ_RATCHET=false)。
#
# 用法: REPS=8 bash src/scripts/gripper/run_tau_source_ab.sh
#       (长跑必须 setsid 脱离会话:setsid bash ... > log 2>&1 < /dev/null &)
# 聚合: python3 src/scripts/gripper/aggregate_tau_source_ab.py <manifest.txt>
set -u

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

# 单实例锁(记忆 sitl-batch-pitfalls:双批碰撞会互相 kill、两批数据全废)。
# 9>&- 关掉子进程继承的 fd,否则孤儿持锁、ps 里看不见却抢不到锁。
exec 9>"/tmp/tau_source_ab.lock"
if ! flock -n 9; then
  echo "[ab] 已有实例在跑(/tmp/tau_source_ab.lock 被持有),退出。"; exit 1
fi

REPS="${REPS:-8}"
ARMS="${ARMS:-command physfull}"     # smoke 时可只给一臂
RECOVER_SEC="${RECOVER_SEC:-45}"     # drop 之后再录这么久(与主线批次一致)
TIMEOUT="${TIMEOUT:-300}"            # 单轮等 DROP 的上限(秒)
STAMP=$(date +%Y%m%d_%H%M%S)
RESID_ROOT="$RUNDIR/tausrc_ab_$STAMP"
MANIFEST="$RUNDIR/tau_source_ab_${STAMP}.txt"
mkdir -p "$RESID_ROOT"
{
  echo "# MHE 力矩来源 A/B  $STAMP"
  echo "# 列: arm rep nmpc_stamp status peak_pos_err nmpc_failed attach_ecc m_err_p50 m_err_p90 solve_med traj n_samples resid_dir"
  echo "# 臂: command=(tau_source=command,motor_avg=0,headless 现默认)  physfull=(phys_full,motor_avg=1,节点默认/viz 在跑的)"
  echo "# 底座: 4m/s 主线档 mass=0.15 envelope=0.3 ecc=0.10 r=10 w=0.283 ramp=9.36 dz=0.8 z=6 lift=8.4 drop@tip55 reps=$REPS"
  echo "# 主判据: 带载稳态(DROP 前 20s)m̂ 有符号相对偏差的配对差;前置硬条件 = 全 ok"
  echo "# status=CRASH/DIVERGED 行其余列不可用(坠机会把 solve/估计误差全部染色)"
  echo "# ⚠️ m_err_* 两列来自 MHE 的 [truth] 标签,而该标签的 m_true 受 _payload_attached 门控;"
  echo "#    主线档下它常年 False → 标签拿空机当真值、误差整体虚高 ~6pp(记忆 cxy-truth-label-defect)。"
  echo "#    **主判据不读这两列**,由 aggregate_tau_source_ab.py 按 NMPC 墙钟段落自算真值。"
} > "$MANIFEST"
echo "[ab] manifest: $MANIFEST"
echo "[ab] resid root: $RESID_ROOT"

cleanup() {
  # ⚠️ 2026-09-05 增补 "transport13/gz-transport-topic":headless 第 317 行的
  # `gz topic -e` 会 exec 成这个真进程,而既有 cleanup 的 "/gz "/"ruby" 两条模式
  # 都匹配不到它 —— 实测上一批 13 轮留下 13 个孤儿订阅者(记忆 sitl-stack-cleanup
  # 里"包装器被 kill、真进程还在"的同一个坑)。
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "transport13/gz-transport-topic" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(pgrep -f "$pat" 2>/dev/null); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[ab] interrupted"; cleanup; exit 130' INT TERM

run_one() {
  local arm="$1" rep="$2" tsrc avg
  if [ "$arm" = physfull ]; then tsrc=phys_full; avg=1; else tsrc=command; avg=0; fi
  local rdir="$RESID_ROOT/${arm}_rep${rep}"
  mkdir -p "$rdir"
  local launch="$RUNDIR/tausrc_launch_${STAMP}_${arm}_${rep}.log"
  echo "[ab] === arm=$arm rep=$rep (MHE_TAU_SOURCE=$tsrc MHE_MOTOR_AVG=$avg) $(date +%T) ==="
  cleanup
  MHE_TAU_SOURCE=$tsrc MHE_MOTOR_AVG=$avg \
  GRIP_PAYLOAD_KG=0.15 GRIP_PAYLOAD_ENVELOPE=0.3 GRIP_ECC_Y=0.10 \
    GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
    GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 \
    GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
    DJ_TRACK_MEST=true GRIP_DJ_FLOOR_MP=0.05 MHE_C_XY_EST=true \
    DJ_RATCHET=false \
    DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
    MHE_CONFIRM_ALPHA=1.5 MHE_RESID_STEP=0 \
    MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self NMPC_GEOM_RELEASE_MODE=event \
    MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
    MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
    RESID_LOG_DIR="$rdir" EVAL_TRUE_PAYLOAD_MASS=0.15 \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$launch" 2>&1 9>&-

  local nmpc="" k
  for k in $(seq 1 12); do
    nmpc=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$launch" 2>/dev/null | head -1)
    [ -n "$nmpc" ] && break; sleep 5
  done
  if [ -z "$nmpc" ]; then
    echo "$arm $rep - LAUNCH_FAIL - - - - - - - 0 $rdir" >> "$MANIFEST"
    echo "[ab] $arm rep$rep LAUNCH_FAIL"; cleanup; return
  fi
  local nstamp; nstamp=$(echo "$nmpc" | grep -oE "[0-9]{8}_[0-9]{6}")
  local mhe="${nmpc/grip_nmpc_/grip_mhe_}"

  # 起飞后核对参数真的生效了(启动行由 mhe_node 打),防止静默跑成同一档 —— 
  # 这个 A/B 的两臂只差两个参数,拼错变量名会得到"无差异"的假阴性。
  local k2
  for k2 in $(seq 1 12); do
    grep -q "MHE node initialized" "$mhe" 2>/dev/null && break; sleep 5
  done
  local initln; initln=$(grep -m1 "MHE node initialized" "$mhe" 2>/dev/null)
  if ! echo "$initln" | grep -q "tau_source=$tsrc"; then
    echo "[ab] !!! 参数未生效:$initln"
    echo "$arm $rep $nstamp PARAM_MISMATCH - - - - - - - 0 $rdir" >> "$MANIFEST"
    cleanup; return
  fi

  # 等 drop(连续主线打 'DROP complete',legacy 事件档打 'DROP: released'),
  # 再多录 RECOVER_SEC 秒
  local waited=0 dropped=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    if grep -qE "DROP complete|DROP: released" "$nmpc" 2>/dev/null; then dropped=1; break; fi
    sleep 6; waited=$((waited+6))
  done
  [ "$dropped" -eq 1 ] && sleep "$RECOVER_SEC"
  # 指标一律由 extract_ab_metrics.py 统一口径地抽(status 也在里面判)
  local metrics
  metrics=$(python3 "$WS/src/scripts/gripper/extract_ab_metrics.py" \
            "$nmpc" "$mhe" 2>/dev/null)
  [ -z "$metrics" ] && metrics="EXTRACT_FAIL 99 99 nan nan nan nan - 0"
  echo "$arm $rep $nstamp $metrics $rdir" >> "$MANIFEST"
  echo "[ab] $arm rep$rep stamp=$nstamp -> $metrics"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  # 配对交错:奇数轮 command 先,偶数轮 physfull 先(抵消机器热身/漂移)
  if [ $((rep % 2)) -eq 1 ]; then order="$ARMS"; else order=$(echo $ARMS | awk '{for(i=NF;i>0;i--)printf "%s ",$i}'); fi
  for arm in $order; do run_one "$arm" "$rep"; done
done
echo "[ab] done. manifest: $MANIFEST"
