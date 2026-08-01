#!/bin/bash
# VI-C 几何-质量解耦消融 @ 高应力(2026-07-27,补 07-20 未落盘的 0/15 vs 4/15)。
#
# 【问什么】高应力(MHE_M_MIN=1.0,让 m_est 有空间探到空机以下)下,几何路径
# **解耦** vs **耦合** 的发散率。机制:耦合路径 m_p_hat=max(m_est-m_nominal,0),
# m_est 探到 1.0(<空机2.064)→ m_p_hat=0 → dJ/c_xy 几何塌陷 → NMPC 失配发散;
# 解耦路径几何取独立先验、不读 m_est → 免疫。07-20 首测 decoupled 0/15 vs
# coupled 4/15 但**未落分组 manifest**,不可复现——本脚本重跑并全量落盘。
#
# 【两组唯一差异=几何路径】(承"不同时上两变量":m_min/mass/ecc 两组逐一对齐)
#   decoupled: GRIP_GEOM_MP_PRIOR=$mass(独立先验路径,不读 m_est)
#   coupled  : GRIP_GEOM_MP_PRIOR=0 + GRIP_GEOM_MP_FLOOR=0(纯 m_est 驱动棘轮,
#              07-19 前行为;floor 也关掉,否则 floor=0.15 已部分解耦、污染对照)
#   两组同为 MHE_M_MIN=1.0(高应力,暴露耦合的塌陷通道)。
#
# 用法:  MASSES="0.2 0.3" ECCS="0.05 0.10" REPS=4 [MANIFEST=续跑] \
#         bash src/scripts/gripper/run_geom_decouple_stress.sh
# 长跑须 setsid。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.05 0.10}"
GGRP="${GGRP:-decoupled coupled}"
REPS="${REPS:-4}"
M_MIN="${M_MIN:-1.0}"          # 高应力(默认 1.961,这里压到 1.0 暴露耦合)
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)

# --- 跨工作区守卫(共享 px4_sitl_default,他人栈会互杀,memory gripper-slung-load) ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[decstress] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 8>"/tmp/geom_decouple_stress.lock"
flock -n 8 || { echo "[decstress] 已有实例在跑,退出。"; exit 1; }

MANIFEST="${MANIFEST:-$RUNDIR/geom_decstress_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  {
    echo "# VI-C geom decouple @ high stress $STAMP : group mass ecc rep stamp status m_min geom_prior floor"
    echo "# 配置: m_min=${M_MIN}(高应力) groups=${GGRP} masses=${MASSES} eccs=${ECCS} reps=${REPS}"
    echo "# decoupled=独立先验(prior=mass,不读m_est); coupled=纯m_est棘轮(prior=0,floor=0)"
    echo "# 判据: DIVERGED=pos_err峰>2.0m; 交叉核验 solve-failed>100"
  } > "$MANIFEST"
fi
echo "[decstress] manifest: $MANIFEST (m_min=$M_MIN)"

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
             "ninja gz_x500" "make px4_sitl"; do
    pids=$(ps -eo pid,cmd | grep "$pat" | grep -v grep | grep -v ugrep | awk '{print $1}')
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done
  sleep 3
}
trap 'echo "[decstress] interrupted"; cleanup; exit 130' INT TERM

for group in $GGRP; do for mass in $MASSES; do for ecc in $ECCS; do for rep in $(seq 1 "$REPS"); do
  if grep -qE "^$group $mass $ecc $rep .* (ok|DIVERGED) " "$MANIFEST" 2>/dev/null; then
    echo "[decstress] skip $group $mass $ecc $rep (done)"; continue
  fi
  # ⚠️ 必须带小数:ROS DOUBLE 参数收到整数 "0" 会 InvalidParameterTypeException
  # 崩 MHE(2026-07-27 首跑踩,coupled 组全废)。
  if [ "$group" = decoupled ]; then GP="$mass"; FL=0.15; else GP=0.0; FL=0.0; fi
  echo "[decstress] === $group mass=$mass ecc=$ecc rep=$rep (prior=$GP floor=$FL m_min=$M_MIN) ==="
  LAUNCH="$RUNDIR/decstress_launch_${STAMP}_${group}_${mass}_${ecc}_${rep}.log"
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    MHE_M_MIN=$M_MIN GRIP_GEOM_MP_PRIOR=$GP GRIP_GEOM_MP_FLOOR=$FL \
    NMPC_GEOM_SOURCE=online \
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
  sf=$(grep -c "solve failed" "$NMPC" 2>/dev/null); sf=${sf:-0}
  status=ok
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && [ "$status" != DIVERGED ] && status=TIMEOUT_${aw}
  echo "$group $mass $ecc $rep $nstamp $status $M_MIN $GP $FL sf=$sf" >> "$MANIFEST"
  echo "[decstress] $group $mass $ecc $rep stamp=$nstamp aw=$aw peak=$peak sf=$sf -> $status"
  cleanup
done; done; done; done
echo "[decstress] done. manifest: $MANIFEST"
echo "[decstress] 聚合: grep -v '^#' $MANIFEST | awk '{d[\$1]+=(\$6==\"DIVERGED\");n[\$1]++} END{for(g in n)print g,d[g]\"/\"n[g]}'"
