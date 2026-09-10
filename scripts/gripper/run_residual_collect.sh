#!/bin/bash
# 【2026-08-26 去先验改造】本脚本原先顺带设置的载荷质量先验环境变量
# (GRIP_GEOM_MP_PRIOR / GRIP_GEOM_MP_FLOOR / MHE_CONFIRM_PRIOR 等)已删除:
# 对应的节点参数不复存在,留着只会静默失效并误导读者。载荷质量信息现在只有
# grip_payload_envelope 一条(机架规格包线上界,run_gripper_headless.sh 默认 0.3)。
# 本脚本自身的研究主题不受影响。
# 动力学残差**可辨识性诊断**数据采集(2026-07-31)。
#
# 【定位】这不是"神经网络训练数据采集"。采完先答四个问题,不全过就不上网络:
#   ①正确对齐+滤波后,残差 RMS 是否明显高于标定与导数不确定度
#   ②线性/低阶非线性模型在**按整轮划分**的测试集上是否明显优于常数模型
#   ③残差在未见飞行条件下是否保持正确方向与合理量级
#   ④跨轮次是否可重复(同条件不同轮的残差是否一致)
# 若结果显示残差主要来自时间错位/标定误差/数值微分噪声,则结论是"不该上网络"。
#
# 【为什么记异步原始流】motor(经 ros_gz_bridge 来自 Gazebo)、odom(经 mavros 来自
# PX4)、u_opt(本机 NMPC 直发)三者频率与时钟来源都不同。在线拼成一张表会把时间
# 错位烙进数据,而那正是要诊断的对象。故 residual_logger 四流各写各的,对齐留给
# 离线(可自由做延迟扫描/插值/最近邻/丢包检测)。
#
# 【条件矩阵】{0.2,0.3}kg × {0.05,0.10}m × {hover,figure8} = 8 条件,各 2 轮 = 16 轮,
# 余 4 轮给 figure8 高动态重复(残差最可能显形的工况)。
# 留出维度**优先选"未见轨迹"**:只有 2 质量 × 2 偏心时,留出质量/偏心会让训练集
# 只剩一个取值,过于苛刻;hover↔figure8 差异最大且不牺牲参数覆盖。质量/偏心的
# 留出结论只作参考,不作否决依据。
#
# 用法: REPS=2 bash src/scripts/gripper/run_residual_collect.sh
# 长跑须 setsid。产出: nmpc_test_results/residual/resid_{motor,odom,command,internal}_*.csv
#                     + resid_manifest_*.txt(标定常数/惯量/真值条件/语义声明)

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
OUTDIR="${OUTDIR:-$RUNDIR/residual}"

# 与其他 PX4/Gazebo 批处理共用同一把锁:cleanup 是同一组 kill -9 模式,
# 并发会互杀(07-23 实测作废过两批数据)。
exec 9>"/tmp/b4dec_verify.lock"
if ! flock -n 9; then
  echo "[resid] 已有同类批次在跑(锁 /tmp/b4dec_verify.lock),退出。"
  echo "[resid] 查残留(勿用 pgrep -f,会匹配自身): ps -eo pid,cmd | grep -E 'bin/px4|gz sim' | grep -v grep"
  exit 1
fi

REPS="${REPS:-2}"
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.05 0.10}"
TRAJS="${TRAJS:-hover figure8}"
EXTRA_DYN="${EXTRA_DYN:-4}"        # figure8 额外重复轮数
NEED_AW="${NEED_AW:-220}"
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="${MANIFEST:-$RUNDIR/residual_collect_${STAMP}.txt}"
mkdir -p "$OUTDIR"

SOLVE_HZ=$(python3 -c "
import re
s=open('$WS/src/offboard_test_acados/offboard_test_acados/acados_params.py').read()
N=int(re.search(r'^\s+N\s+=\s+(\d+)',s,re.M).group(1))
dt=float(re.search(r'^\s+dt\s+=\s+([\d.]+)',s,re.M).group(1))
print(f'{1/dt:.0f}Hz (N={N} dt={dt})')" 2>/dev/null || echo "未知")

if [ ! -f "$MANIFEST" ]; then
  {
    echo "# 残差诊断采集 $STAMP"
    echo "# 列: traj mass ecc rep stamp status"
    echo "# 配置: masses=${MASSES} eccs=${ECCS} trajs=${TRAJS} reps=${REPS} extra_dyn=${EXTRA_DYN}"
    echo "# 配置: NMPC solve=${SOLVE_HZ}  MHE=10Hz  几何先验=truth  θ=M0(主线配置)"
    echo "# 产出: $OUTDIR/resid_*_<stamp>.csv (四条异步流) + resid_manifest_<stamp>.txt"
    echo "# ⚠️三流时钟域未经验证(use_sim_time未设,motor来自Gazebo/odom来自PX4),"
    echo "#   离线第一步必须先做时钟域检查与延迟扫描,不可直接相减 header.stamp"
  } > "$MANIFEST"
fi
echo "[resid] manifest: $MANIFEST"
echo "[resid] 采集输出: $OUTDIR"
echo "[resid] NMPC solve=$SOLVE_HZ"

cleanup() {
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
trap 'echo "[resid] interrupted"; cleanup; exit 130' INT TERM

run_one() {
  local traj=$1 mass=$2 ecc=$3 rep=$4
  if grep -qE "^$traj $mass $ecc $rep .* ok " "$MANIFEST" 2>/dev/null; then
    echo "[resid] skip $traj $mass $ecc $rep (ok)"; return
  fi
  # hover = 不进动态段(只 attach 后悬停);figure8 = 走动态 8 字
  local DYN="true"
  [ "$traj" = hover ] && DYN="false"
  echo "[resid] === traj=$traj mass=$mass ecc=$ecc rep=$rep ==="
  local LAUNCH="$RUNDIR/resid_launch_${STAMP}_${traj}_${mass}_${ecc}_${rep}.log"
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    NMPC_GEOM_SOURCE=online \
    GRIP_DYNAMIC="$DYN" ATTACH_WINDOW_SEC=60.0 \
    RESID_LOG_DIR="$OUTDIR" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1 9>&-
  local NMPC="" k
  for k in $(seq 1 12); do
    NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
    [ -n "$NMPC" ] && break; sleep 5
  done
  local nstamp; nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
  local waited=0 aw=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    aw=$(grep -c "attach-window" "$NMPC" 2>/dev/null)
    [ -n "$aw" ] && [ "$aw" -ge "$NEED_AW" ] && break
    sleep 6; waited=$((waited+6))
  done
  local status=ok
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && status=TIMEOUT_${aw}
  # 采集健康度:四条流是否都产出了行
  local nrow=0
  for s in motor odom command internal; do
    f=$(ls -t "$OUTDIR"/resid_${s}_*.csv 2>/dev/null | head -1)
    [ -n "$f" ] && nrow=$((nrow + $(wc -l < "$f")))
  done
  [ "$nrow" -lt 100 ] && status="${status}_LOWROWS_${nrow}"
  echo "$traj $mass $ecc $rep $nstamp $status" >> "$MANIFEST"
  echo "[resid] $traj $mass $ecc $rep stamp=$nstamp aw=$aw rows=$nrow -> $status"
  cleanup
}

for rep in $(seq 1 "$REPS"); do
  for traj in $TRAJS; do for mass in $MASSES; do for ecc in $ECCS; do
    run_one "$traj" "$mass" "$ecc" "$rep"
  done; done; done
done
# 额外的 figure8 高动态重复(残差最可能显形处)
for i in $(seq 1 "$EXTRA_DYN"); do
  run_one figure8 0.3 0.10 $((100 + i))
done

echo "[resid] done. manifest: $MANIFEST"
echo "[resid] 离线第一步: 时钟域检查 + 延迟扫描(勿跳过)"
