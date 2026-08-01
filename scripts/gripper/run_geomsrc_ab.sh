#!/bin/bash
# 几何来源 A/B: geom_source=truth vs online (2026-07-29)。
#
# 【问什么】NMPC 的几何来自 attach 真值(truth) vs 在线估计(online)时,m_est 精度、
# c_est 精度、发散率是否可比。online 若可比,则整条链不再依赖 attach 真值几何
# ——B.3 立项目标(消 attach 真值依赖)达成,真机可迁移。
#
# 【前置改动 2026-07-28】acados_nmpc_node._update_online_geometry 已改成只更新
# c_est;dJ 在 attach 瞬间由 grip_payload_prior×grip_arm_d² 一次算定后不变。
# 改之前 online 路径每帧用 m_p=m_est-p.m 重推 dJ(NMPC 侧的 m_est→dJ 耦合,
# 无棘轮/地板保护)。**本 A/B 测的是改后的 online**。
#
# 【两组唯一差异=geom_source】其余全部对齐:
#   truth : NMPC_GEOM_SOURCE=truth  (dJ/c_est 从 attach_offset 真值算)
#   online: NMPC_GEOM_SOURCE=online (c_est 吃 τ_phys 在线估计, dJ 用先验)
#   两组 MHE 侧同为生产配置(GRIP_GEOM_MP_PRIOR=$mass, FLOOR=0.15, 默认 m_min)。
#   两组同开 MHE_C_XY_EST=true —— truth 组也要算 c_xy_est,否则没有对表数据。
#
# 用法:  MASSES="0.2 0.3" ECCS="0.05 0.10" REPS=2 [MANIFEST=续跑] \
#         bash src/scripts/gripper/run_geomsrc_ab.sh
# 长跑须 setsid。不用 set -e/-u。

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
MASSES="${MASSES:-0.2 0.3}"
ECCS="${ECCS:-0.05 0.10}"
GGRP="${GGRP:-truth online}"
REPS="${REPS:-2}"
NEED_AW="${NEED_AW:-220}"    # ~11s @ 20Hz
TIMEOUT="${TIMEOUT:-240}"
STAMP=$(date +%Y%m%d_%H%M%S)

# --- 跨工作区守卫(共享 px4_sitl_default,他人栈会互杀,memory gripper-slung-load) ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[geomsrc] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 8>"/tmp/geomsrc_ab.lock"
flock -n 8 || { echo "[geomsrc] 已有实例在跑,退出。"; exit 1; }

MANIFEST="${MANIFEST:-$RUNDIR/geomsrc_ab_${STAMP}.txt}"
if [ ! -f "$MANIFEST" ]; then
  {
    echo "# geom_source A/B $STAMP : group mass ecc rep stamp status m_est c_err_pct sf"
    echo "# 配置: groups=${GGRP} masses=${MASSES} eccs=${ECCS} reps=${REPS} (MHE 侧两组同为生产配置)"
    echo "# truth=attach真值几何; online=c_est吃τ_phys在线估计+dJ用先验(2026-07-28 解耦版)"
    echo "# 判据: DIVERGED=pos_err峰>2.0m; c_err_pct=|c_est-truth|/|truth| 末值(仅 online 有意义)"
  } > "$MANIFEST"
fi
echo "[geomsrc] manifest: $MANIFEST"

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
trap 'echo "[geomsrc] interrupted"; cleanup; exit 130' INT TERM

for group in $GGRP; do for mass in $MASSES; do for ecc in $ECCS; do for rep in $(seq 1 "$REPS"); do
  if grep -qE "^$group $mass $ecc $rep .* (ok|DIVERGED) " "$MANIFEST" 2>/dev/null; then
    echo "[geomsrc] skip $group $mass $ecc $rep (done)"; continue
  fi
  echo "[geomsrc] === $group mass=$mass ecc=$ecc rep=$rep ==="
  LAUNCH="$RUNDIR/geomsrc_launch_${STAMP}_${group}_${mass}_${ecc}_${rep}.log"
  # ⚠️ GRIP_GEOM_MP_PRIOR 必须带小数点语义(这里恒等于 $mass,非 0,无 ROS 类型坑)
  GRIP_PAYLOAD_KG=$mass GRIP_ECC_Y=$ecc USE_MHE=true MHE_C_XY_EST=true \
    GRIP_GEOM_MP_PRIOR=$mass GRIP_GEOM_MP_FLOOR=0.15 \
    NMPC_GEOM_SOURCE=$group \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1
  NMPC=""; for k in $(seq 1 12); do
    NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
    [ -n "$NMPC" ] && break; sleep 5
  done
  nstamp=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
  MHE="$RUNDIR/grip_mhe_${nstamp}.log"
  waited=0; aw=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    aw=$(grep -c "attach-window" "$NMPC" 2>/dev/null)
    [ -n "$aw" ] && [ "$aw" -ge "$NEED_AW" ] && break
    sleep 6; waited=$((waited+6))
  done
  peak=$(grep -oE "pos_err=[0-9.]+" "$NMPC" 2>/dev/null | grep -oE "[0-9.]+" | sort -rn | head -1)
  [ -z "$peak" ] && peak=99
  sf=$(grep -c "solve failed" "$NMPC" 2>/dev/null); sf=${sf:-0}
  m_est=$(grep -oE "MHE mass estimate: [0-9.]+" "$MHE" 2>/dev/null | tail -1 | grep -oE "[0-9]+\.[0-9]+")
  [ -z "$m_est" ] && m_est=nan
  # c_xy 对表:末条 [c_xy_est] 行形如
  #   [c_xy_est] cx=+0.0021 cy=-0.0085 m | truth c=[+0.0019,-0.0083] (m_p=0.295)
  c_err=$(grep "c_xy_est" "$MHE" 2>/dev/null | grep "truth c=" | tail -1 | \
    sed -E 's/.*cx=([+-][0-9.]+) cy=([+-][0-9.]+).*truth c=\[([+-][0-9.]+),([+-][0-9.]+)\].*/\1 \2 \3 \4/' | \
    awk '{ex=$1-$3; ey=$2-$4; n=sqrt($3*$3+$4*$4);
           if(n>1e-6) printf "%.1f", 100*sqrt(ex*ex+ey*ey)/n; else printf "nan"}')
  [ -z "$c_err" ] && c_err=nan
  status=ok
  awk "BEGIN{exit !($peak>2.0)}" && status=DIVERGED
  { [ -z "$aw" ] || [ "$aw" -lt "$NEED_AW" ]; } && [ "$status" != DIVERGED ] && status=TIMEOUT_${aw}
  echo "$group $mass $ecc $rep $nstamp $status $m_est $c_err $sf" >> "$MANIFEST"
  echo "[geomsrc] $group $mass $ecc $rep stamp=$nstamp aw=$aw peak=$peak m_est=$m_est c_err=${c_err}% sf=$sf -> $status"
  cleanup
done; done; done; done
echo "[geomsrc] done. manifest: $MANIFEST"
echo "[geomsrc] 聚合: grep -v '^#' $MANIFEST | awk '{d[\$1]+=(\$6==\"DIVERGED\");n[\$1]++;me[\$1]+=\$7;ce[\$1]+=\$8} END{for(g in n)printf \"%s div=%d/%d m_est_avg=%.3f c_err_avg=%.1f%%\\n\",g,d[g],n[g],me[g]/n[g],ce[g]/n[g]}'"
