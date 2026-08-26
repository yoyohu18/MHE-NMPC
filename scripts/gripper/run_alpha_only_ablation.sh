#!/bin/bash
# 【2026-08-26 去先验改造】本脚本原先顺带设置的载荷质量先验环境变量
# (GRIP_GEOM_MP_PRIOR / GRIP_GEOM_MP_FLOOR / MHE_CONFIRM_PRIOR 等)已删除:
# 对应的节点参数不复存在,留着只会静默失效并误导读者。载荷质量信息现在只有
# grip_payload_envelope 一条(机架规格包线上界,run_gripper_headless.sh 默认 0.5)。
# 本脚本自身的研究主题不受影响。
# α-only 消融:θ* 的收益到底来自哪一维?(2026-07-30)
#
# 【问什么】θ* = 4 维节奏 + 1 维无量纲阈值 α。B.4/holdout 只比过 {M0, 完整θ*},
#   从未把两者拆开。若收益主体来自 α 撞上界(0.9875)这一个标量,那"CEM 学调度"
#   可退化成"把一个阈值调大"的一行手工规则,方法学贡献基本不成立。
#
# 【四臂 = 正交 2×2 析因】(语义已核实正交:ParametricWeightSchedule.__init__ 只读
#   theta[0..3];α 走独立参数 confirm_thresh_alpha,阈值=α·g·confirm_payload_prior)
#
#                  │ 阈值 1.5N(固定路径)  │ 阈值 α=0.9875(→1.9N/2.9N)
#     ─────────────┼──────────────────────┼──────────────────────────
#     节奏 = M0    │ M0                   │ alphaonly
#     节奏 = θ*    │ rhythmonly           │ thetastar
#
#   两个主效应各有两条独立估计,还能读交互:
#     α 效应      = (alphaonly−M0) 和 (thetastar−rhythmonly)
#     节奏效应    = (rhythmonly−M0) 和 (thetastar−alphaonly)
#   ⚠️ rhythmonly 的阈值不用"M0 等效 α"(=1.5/(g·m),0.2kg→0.765、0.3kg→0.510)去
#   凑,而是直接传 α=-1.0 走 M0 那条固定 event_confirm_thresh_n 代码路径 —— 数值
#   与 M0 逐字相同而非"等效",把浮点与代码路径差异一并排除。
#
# 【为什么 GEOM_PRIOR 默认 truth】07-23 已证几何先验错配(0.25)时三臂优势全部
#   消失甚至反转(0.3kg θ* 反而慢 46%)。在信号已被淹掉的条件下做维度归因是在
#   噪声里做归因,故本实验固定在"信号存在"的 truth 先验下问归因问题。
#   错配条件下的归因是独立的后续问题,不在本批范围。
#
# 【配对交错 + 顺序轮转】run_b4_matrix/run_b4_decoupled_verify 的循环是 method
#   在最外层(所有 M0 跑完才跑 θ*),跨臂间隔可达小时级,暴露在 07-10 记录的
#   "M0 绝对基线跨时段漂移 ±0.5s"下。本脚本改为 rep 最外层、method 最内层
#   → 同 rep 内四臂背靠背;且每个 rep 用不同的臂序(4 阶拉丁方轮转),使
#   "第几个跑"与"哪个臂"解耦,消除批内顺序效应(如热机、渐进的磁盘/内存占用)。
#
# 【关于"同 seed 共享随机实现"——本设施做不到,如实说明】用户设计里要求四臂共享
#   同一轨迹/阶跃时刻/扰动/初始状态。本设施可控的部分**已全部共享且确定**:
#   轨迹参数、grip_lift_after_sec 阶跃时刻、世界初始状态、几何先验、
#   attach_window_sec 分析窗口,四臂逐字相同。但 **PX4 SITL 没有可注入的随机
#   种子**——非确定性来自运行时(线程调度、acados 求解时序、Gazebo 物理步进),
#   无法冻结。故这里的 rep ≠ 统计意义上的 seed:它是"同一标称配置下的重复",
#   配对的价值来自"背靠背相邻",不是"共享随机数流"。CRN(共同随机数)在
#   masschanger CEM 里能做是因为那是纯 wrench 注入;吊挂 SITL 做不到。
#
# 【M0 基线阈值口径】不显式传 MHE_CONFIRM_THRESH → headless 默认 1.5N,与
#   07-15/07-23 两批 B.4 矩阵逐字一致(⚠️注意 07-10 holdout 批的 M0 用的是
#   固定 0.8N,两批 M0 定义本就不同,跨批绝对值不可比——本批全部结论走配对差)。
#
# 用法:  REPS=8 MASSES="0.2 0.3" [GEOM_PRIOR=truth] [MANIFEST=续跑] \
#          setsid bash src/scripts/gripper/run_alpha_only_ablation.sh
#        聚合:  python3 src/scripts/gripper/aggregate_alpha_only.py <manifest>
# 长跑须 setsid 脱离会话(见记忆 nosignal-ablation 的双批碰撞教训)。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

# ⚠️ 故意共用 b4dec 的锁文件:本脚本的 cleanup 与 run_b4_decoupled_verify.sh 是
# 同一组 kill -9 模式,两者并发会互杀对方在跑的节点(07-23 已实测作废两批数据)。
# 独立锁名只会让两个批次都以为自己是唯一实例 → 必须同锁。
#
# ⚠️⚠️ fd 继承坑(2026-07-30 实测踩到):`exec 9>lock` 打开的 fd 9 会被**所有子进程
# 继承**,包括 headless 里 nohup 起的 mavros/ros2/ros_gz_bridge。主脚本被 kill 后,
# 这些孤儿子进程仍持有 fd 9 → 锁永不释放 → 下一批次误判"已有实例在跑"直接退出,
# 且 ps 里看不到任何 run_* 脚本,现象极具误导性。查法: fuser -v /tmp/b4dec_verify.lock
# 修法: 调 headless 时加 `9>&-` 关掉该 fd(见下方 run_gripper_headless.sh 调用处)。
# run_b4_decoupled_verify.sh 有同一缺陷,若日后复用需同样处理。
exec 9>"/tmp/b4dec_verify.lock"
if ! flock -n 9; then
  echo "[aonly] 已有 b4dec/α-only 类批次在跑(锁 /tmp/b4dec_verify.lock),退出。"
  echo "[aonly] 核验残留(勿用 pgrep -f,会匹配自身):"
  echo "        ps -eo pid,etime,cmd | grep -E 'bin/px4|gz sim|offboard_test_acados/' | grep -v grep"
  exit 1
fi

MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.10}"
REPS="${REPS:-8}"
GEOM_PRIOR="${GEOM_PRIOR:-truth}"
THETA_STAR="-4.8038,-1.2080,0.4930,-0.9602,0.9875"
STAR_RHYTHM="-4.8038,-1.2080,0.4930,-0.9602"   # θ* 前4维,不含 α
M0_RHYTHM="-4.0,0.0,0.0,0.0"
ALPHA_STAR="0.9875"
# ARMS_MODE=4(默认):完整 2×2 析因四臂,4 阶拉丁方,(rep-1)%4 选行,每臂每位置各1次。
# ARMS_MODE=2:只跑 M0 vs θ*,用于在**别的 solve 频率**下对上论文表2的口径
#   (2026-07-30 用途:主线已切 solve@20Hz,而表2数据出自 solve@10Hz,
#    需在 10Hz 下配对交错重测才能判表2真伪)。2 臂用简单交替消顺序效应。
# ARMS_MODE=3(2026-08-03 新增):**基线阈值口径消解**三臂 M0@1.5N / M0@0.8N / θ*。
#   问什么:07-10 holdout 那次 6/6 全胜是 CEM 路线**唯一的正面证据**,但它的 M0
#   基线用固定 **0.8N** 阈值,而 2×2 析因(表2,收益归零那个)用 headless 默认
#   **1.5N**。若 0.8N 本身就是个明显更差的基线,6/6 就只是"打赢了一个选差了的
#   基线",与 θ* 学到什么无关。三臂同批配对即可分离这两种解释。
#   ⚠️ 本批**不会**改变表2/§VI-B 的结论(那里全程单一基线),只定早期正面结果的地位。
#   工况固定在 07-10 的留出判决点(mass=0.225 × ecc=0.05):两轴都不在 CEM 训练
#   网格上,且小偏心 σ 最小(07-10 实测大偏心 σ 高达 0.3-0.7,判决优先看小偏心)。
ARMS_MODE="${ARMS_MODE:-4}"
if [ "$ARMS_MODE" = 3 ]; then
  # 3 阶拉丁方:每臂在每个位置各出现一次(rep 数为 3 的倍数时完全平衡)
  ORDER_0="M0 M0_08N thetastar"
  ORDER_1="thetastar M0 M0_08N"
  ORDER_2="M0_08N thetastar M0"
  ORDER_3="$ORDER_0"     # 占位:%NORDERS 取模后用不到,留着防手滑
  METHODS_ALL="M0 M0_08N thetastar"; NARMS=3; NORDERS=3
elif [ "$ARMS_MODE" = 2 ]; then
  ORDER_0="M0 thetastar";  ORDER_1="thetastar M0"
  ORDER_2="M0 thetastar";  ORDER_3="thetastar M0"
  METHODS_ALL="M0 thetastar"; NARMS=2
else
  ORDER_0="M0 alphaonly rhythmonly thetastar"
  ORDER_1="thetastar rhythmonly alphaonly M0"
  ORDER_2="alphaonly M0 thetastar rhythmonly"
  ORDER_3="rhythmonly thetastar M0 alphaonly"
  METHODS_ALL="M0 alphaonly rhythmonly thetastar"; NARMS=4
fi
# solve 频率不是 ROS 参数(源自 acados_params.py 的 N/dt),故记进 manifest 供追溯
# 直接读源文件而非 import(import 需先 source install/setup.bash,批次脚本里没有)
SOLVE_HZ=$(python3 -c "
import re
s=open('$WS/src/offboard_test_acados/offboard_test_acados/acados_params.py').read()
N=int(re.search(r'^\s+N\s+=\s+(\d+)',s,re.M).group(1))
dt=float(re.search(r'^\s+dt\s+=\s+([\d.]+)',s,re.M).group(1))
print(f'{1/dt:.0f}Hz (N={N} dt={dt} tf={N*dt:.2f}s)')" 2>/dev/null || echo "未知")
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz;比 07-23 的 110 宽,压 parse_fail
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="${MANIFEST:-$RUNDIR/alpha_only_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  {
    echo "# α-only 消融 $STAMP : θ* 收益的维度归因"
    echo "# 列: method mass ecc rep stamp status geom_prior"
    echo "# 配置: geom_source=online  geom_prior_mode=${GEOM_PRIOR}"
    echo "# 配置: masses=${MASSES}  eccs=${ECCS}  reps=${REPS}  methods=${METHODS_ALL}"
    echo "# 配置: theta_star=[${THETA_STAR}]  star_rhythm=[${STAR_RHYTHM}]  m0_rhythm=[${M0_RHYTHM}]  alpha_star=${ALPHA_STAR}"
    echo "# 配置: M0/rhythmonly 臂 alpha=-1.0 → 固定 event_confirm_thresh_n=1.5N (headless 默认, 同 B.4 矩阵)"
    [ "$ARMS_MODE" = 3 ] && echo "# 配置: M0_08N 臂 = 节奏同 M0, 仅固定阈值改 0.8N (=07-10 holdout 批 M0 口径); 本批用途=基线阈值口径消解"
    echo "# 配置: ARMS_MODE=${ARMS_MODE}  NMPC solve=${SOLVE_HZ}  (MHE 恒 10Hz)"
    echo "# 设计: 正交2x2析因(节奏M0/θ* × 阈值1.5N/α0.9875); rep最外method最内=各臂背靠背; 臂序逐rep轮转"
    echo "# 判据: DIVERGED=pos_err峰>2.0m ; NEED_AW=${NEED_AW} attach-window 帧"
  } > "$MANIFEST"
fi
echo "[aonly] manifest: $MANIFEST"
echo "[aonly] ARMS_MODE=${ARMS_MODE} 臂集: ${METHODS_ALL}"
echo "[aonly] NMPC solve=${SOLVE_HZ}   (MHE 恒 10Hz;solve频率源自 acados_params.py 的 N/dt)"
echo "[aonly] geom_prior=${GEOM_PRIOR}  reps=${REPS}  预计 $(( $(echo $MASSES|wc -w) * NARMS * REPS )) 轮 × ~85s"

# 13 条模式直接内联(⚠️ 记忆 sitl-stack-cleanup:永不从含 kill 的脚本里 sed 抽
# 片段执行,那会连 cleanup 一起跑、杀掉在跑的批次)。
cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(pgrep -f "$pat" 2>/dev/null); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[aonly] interrupted"; cleanup; exit 130' INT TERM

# rep 最外、method 最内 = 配对交错;臂序逐 rep 走拉丁方轮转
for rep in $(seq 1 "$REPS"); do
 eval "METHODS=\$ORDER_$(( (rep - 1) % ${NORDERS:-4} ))"
 echo "[aonly] --- rep=$rep 臂序: $METHODS ---"
 for mass in $MASSES; do for ecc in $ECCS; do for method in $METHODS; do
  if grep -qE "^$method $mass $ecc $rep .* ok " "$MANIFEST" 2>/dev/null; then
    echo "[aonly] skip $method $mass $ecc $rep (ok)"; continue
  fi
  # 2×2 析因:节奏维(TH) × 阈值路径(ALPHA)。ALPHA=-1.0 → 固定 1.5N 那条路径。
  # M0_08N:节奏与 M0 逐字相同,只把固定阈值从 headless 默认 1.5N 换成 0.8N,
  # 即 07-10 holdout 批 M0 基线的口径。走同一条 alpha=-1.0 固定阈值代码路径,
  # 与 M0 臂的唯一差别就是那个标量——这正是本批要分离的那一个变量。
  CONFIRM_N=""      # 空=不传,用 headless 默认 1.5N(与表2/B.4矩阵逐字一致)
  case "$method" in
    thetastar)  TH="[$THETA_STAR]";  ALPHA="$ALPHA_STAR"; PRIOR="$mass" ;;
    alphaonly)  TH="[$M0_RHYTHM]";   ALPHA="$ALPHA_STAR"; PRIOR="$mass" ;;
    rhythmonly) TH="[$STAR_RHYTHM]"; ALPHA="-1.0";        PRIOR="0.3"   ;;
    M0_08N)     TH="[$M0_RHYTHM]";   ALPHA="-1.0";        PRIOR="0.3"; CONFIRM_N="0.8" ;;
    *)          TH="[$M0_RHYTHM]";   ALPHA="-1.0";        PRIOR="0.3"   ;;
  esac
  if [ "$GEOM_PRIOR" = truth ]; then GP="$mass"; else GP="$GEOM_PRIOR"; fi

  echo "[aonly] === rep=$rep mass=$mass ecc=$ecc method=$method theta=$TH alpha=$ALPHA ==="
  LAUNCH="$RUNDIR/aonly_launch_${STAMP}_${method}_${mass}_${ecc}_${rep}.log"
  # ⚠️ 用 `env` 而非裸赋值前缀:可选项 `${CONFIRM_N:+NAME=val}` 若放在赋值前缀位,
  # bash 在**解析阶段**就已确定哪些词是赋值,参数展开发生在那之后 → 展开出来的
  # `MHE_CONFIRM_THRESH=0.8` 会被当成**命令名**而不是环境变量(报 command not found)。
  # env 把它们当普通参数收,展开后语义才正确。对既有各臂完全等价。
  env GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    NMPC_GEOM_SOURCE=online \
    MHE_SCHEDULE_THETA="$TH" MHE_CONFIRM_ALPHA="$ALPHA" \
    ${CONFIRM_N:+MHE_CONFIRM_THRESH=$CONFIRM_N} \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1 9>&-
  NMPC=""; for k in $(seq 1 12); do
    NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
    [ -n "$NMPC" ] && break; sleep 5
  done
  nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
  waited=0; aw=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    aw=$(grep -c "attach-window" "$NMPC" 2>/dev/null)
    [ -n "$aw" ] && [ "$aw" -ge "$NEED_AW" ] && break
    sleep 6; waited=$((waited+6))
  done
  peak=$(grep -oE "pos_err=[0-9.]+" "$NMPC" 2>/dev/null | grep -oE "[0-9.]+" | sort -rn | head -1)
  [ -z "$peak" ] && peak=99
  status=ok
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && status=TIMEOUT_${aw}
  echo "$method $mass $ecc $rep $nstamp $status $GP" >> "$MANIFEST"
  echo "[aonly] $method $mass $ecc $rep stamp=$nstamp aw=$aw peak=$peak -> $status"
  cleanup
 done; done; done
done
echo "[aonly] done. manifest: $MANIFEST"
echo "[aonly] 聚合: python3 $WS/src/scripts/gripper/aggregate_alpha_only.py $MANIFEST"
