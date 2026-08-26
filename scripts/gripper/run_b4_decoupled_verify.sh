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
# B.4 重验 @ 几何完全解耦(2026-07-21)。
#
# 【问什么】θ* 对 M0 的收益,在 **07-19 完全解耦几何路径 + 单一错误操作先验** 下
# 是否保持?
#   - 原 B.4 矩阵跑于 07-15,当时 `grip_geom_mp_prior` 参数还不存在,走的是
#     floor 棘轮路径(部分解耦,只护低估方向)。
#   - 07-19 落地 `grip_geom_mp_prior`(独立先验、完全不读 m_est,dJ/c_xy 同时脱
#     自举耦合)后,主线部署配置已变,**θ* 在新配置下还没被验证过**。
#   - 本脚本 = run_b4_matrix.sh 的重跑,**唯一有意改动的变量 = 几何先验来源**
#     (承"不同时上两个新变量"纪律:confirm prior / online / θ 全部与 B.4 一致)。
#
# 【关键设计:GEOM_PRIOR 用固定单值,不用逐工况真值】
#   run_gripper_headless.sh 的默认是 `grip_geom_mp_prior:=${GRIP_GEOM_MP_PRIOR:-$GRIP_PAYLOAD_KG}`
#   ——不显式设就等于该轮**真值**,数值上等于真值会弱化"无真值"claim。
#   故默认 GEOM_PRIOR=0.25(MASSES={0.2,0.3} 的中点):对 0.2kg 高估 +25%、对 0.3kg
#   低估 -16.7%,**两档都错、无一格精确**,正是真机情形("一个操作先验管所有包裹")。
#   已验证的支撑:几何先验用错 33% 时 m_est 仍准 0.3%(07-19)。
#   GEOM_PRIOR=truth 则退回逐工况真值(= B.4 老行为的等价物),留作对照。
#
# 【注意】GRIP_GEOM_MP_FLOOR 在此配置下是**死代码**——_payload_geometry 的优先级
#   是 ①grip_true_payload_mass > ②grip_geom_mp_prior > ③floor棘轮,②命中则③走不到。
#   这里仍显式传 0.15 只为与 B.4 命令行逐字可比,不影响行为。
#
# 用法:  MASSES="0.2 0.3" ECCS="0.10" REPS=5 GEOM_PRIOR=0.25 [MANIFEST=续跑] \
#         bash src/scripts/gripper/run_b4_decoupled_verify.sh
#        聚合:  python3 src/scripts/gripper/aggregate_b4_matrix.py <manifest>
# 长跑须 setsid 脱离会话(见记忆 nosignal-ablation 的双批碰撞教训)。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"

# 单实例锁(2026-07-23 加,当天实测教训):两个批次实例并发时,每轮 headless 的
# 前置清理会 kill -9 对方在跑的节点([ros2run]: Killed→TIMEOUT_0),且幸存轮的
# 话题也被对方栈污染——两批数据全部作废(b4_matrix_dec_20260723_150003/150101)。
# 这是 nosignal-ablation"双批碰撞"教训的第二次重演,故上机制护栏:flock 非阻塞
# 抢锁,抢不到=已有实例在跑,直接退出。锁随进程退出自动释放,无脏锁问题。
exec 9>"/tmp/b4dec_verify.lock"
if ! flock -n 9; then
  echo "[b4dec] 已有另一个实例在跑(锁 /tmp/b4dec_verify.lock 被持有),退出。"
  echo "[b4dec] 若确认无实例残留: ps -eo pid,cmd | grep run_b4_decoupled | grep -v grep"
  exit 1
fi
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.10}"
METHODS="${METHODS:-M0 thetastar}"
REPS="${REPS:-5}"
GEOM_PRIOR="${GEOM_PRIOR:-0.25}"       # 固定单一操作先验;'truth'=逐工况真值(对照)
THETA_STAR="-4.8038,-1.2080,0.4930,-0.9602,0.9875"
ALPHA_STAR="0.9875"
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)
# 名字里含 b4_matrix_ 前缀 → aggregate_b4_matrix.py 不传参时的默认 glob 也能取到。
MANIFEST="${MANIFEST:-$RUNDIR/b4_matrix_dec_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  # 承 07-20 教训(发散 sweep 未落分组元数据致不可复现):配置全量落盘。
  {
    echo "# B.4 重验@几何完全解耦 $STAMP"
    echo "# 列: method mass ecc rep stamp status geom_prior"
    echo "# 配置: geom_source=online  geom_prior_mode=${GEOM_PRIOR}  floor=0.15(死代码)"
    echo "# 配置: masses=${MASSES}  eccs=${ECCS}  reps=${REPS}  methods=${METHODS}"
    echo "# 配置: theta_star=[${THETA_STAR}]  alpha_star=${ALPHA_STAR}"
    echo "# 判据: DIVERGED=pos_err峰>2.0m ; NEED_AW=${NEED_AW} attach-window 帧"
  } > "$MANIFEST"
fi
echo "[b4dec] manifest: $MANIFEST"
echo "[b4dec] geom_prior=${GEOM_PRIOR} (固定单一先验;truth=逐工况真值对照)"

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
trap 'echo "[b4dec] interrupted"; cleanup; exit 130' INT TERM

for method in $METHODS; do for mass in $MASSES; do for ecc in $ECCS; do for rep in $(seq 1 "$REPS"); do
  if grep -qE "^$method $mass $ecc $rep .* ok " "$MANIFEST" 2>/dev/null; then
    echo "[b4dec] skip $method $mass $ecc $rep (ok)"; continue
  fi
  # 方法→θ + 确认阈值配置(与 run_b4_matrix.sh 逐字一致,本次不动这一层)
  if [ "$method" = thetastar ]; then
    TH="[$THETA_STAR]"; ALPHA="$ALPHA_STAR"; PRIOR="$mass"
  else
    TH="[-4.0,0.0,0.0,0.0]"; ALPHA="-1.0"; PRIOR="0.3"
  fi
  # 几何先验:固定单值(默认) 或 逐工况真值(GEOM_PRIOR=truth 对照)
  if [ "$GEOM_PRIOR" = truth ]; then GP="$mass"; else GP="$GEOM_PRIOR"; fi

  echo "[b4dec] === method=$method mass=$mass ecc=$ecc rep=$rep geom_prior=$GP ==="
  LAUNCH="$RUNDIR/b4dec_launch_${STAMP}_${method}_${mass}_${ecc}_${rep}.log"
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_FLOOR=0.15 GRIP_GEOM_MP_PRIOR="$GP" NMPC_GEOM_SOURCE=online \
    MHE_SCHEDULE_THETA="$TH" MHE_CONFIRM_ALPHA="$ALPHA" MHE_CONFIRM_PRIOR="$PRIOR" \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1
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
  # 第7字段=该轮实际几何先验(聚合器取 parts[:6],多余字段安全忽略)
  echo "$method $mass $ecc $rep $nstamp $status $GP" >> "$MANIFEST"
  echo "[b4dec] $method $mass $ecc $rep stamp=$nstamp aw=$aw peak=$peak -> $status"
  cleanup
done; done; done; done
echo "[b4dec] done. manifest: $MANIFEST"
echo "[b4dec] 聚合: python3 $WS/src/scripts/gripper/aggregate_b4_matrix.py $MANIFEST"
