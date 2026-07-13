#!/bin/bash
# 无信号消融批量驱动(2026-07-13,长期计划 A.2)。masschanger/wrench 场景下把
# 三个检测通道配置 × 两个阶跃幅度 × n 次重复全自动跑完,给论文"事件信号值多少"
# 的对照。masschanger 之前没有 n 循环批量器(07-03 那批手动跑),这里补上。
#
# 三配置(权重时间表 theta 全部固定 M0,只变检测通道,隔离"信号 vs 纯残差"):
#   fixed    : event_trigger_enable=false            —— 固定权重,不降权
#   signal   : trigger=true, signal_mode=external    —— 有信号版(外部事件武装+T_phys确认)
#   nosignal : trigger=true, signal_mode=residual    —— 无信号版(纯 T_phys 残差自触发)
# 两幅度:MASS_CHANGER_DELTA_KG = -0.3 / -0.8(drop 方向)。
#
# 每轮:cleanup → 起栈(复用 run_sitl_headless.sh 的组件顺序)→ 轮询 nmpc 日志
# 等 "Payload drop settled." → 再留 MHE 收敛日志窗口 → 收栈。STAMP↔(cfg,delta,rep)
# 映射写进 manifest,给 aggregate_nosignal.py 解析。
#
# 用法:  REPS=5 DELTAS="-0.3 -0.8" CONFIGS="fixed signal nosignal" \
#         bash src/scripts/masschanger/run_nosignal_ablation.sh
# 磁盘/参数护栏沿用记忆 sitl-px4-log-disk-blowup / px4-param-persistence-guard。
# 不用 set -e/-u:批量器靠显式 status 处理单轮失败(wait_for_drop 返回码 +
# cleanup 兜底),且 ROS 的 setup.bash 不是 set -u 安全的(AMENT_TRACE_SETUP_FILES
# 未定义)。

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
MASS_CHANGER_DIR="$WS/src/offboard_test_acados/gz_plugins/mass_changer"
RUNDIR="$WS/nmpc_test_results"
mkdir -p "$RUNDIR"

REPS="${REPS:-5}"
DELTAS="${DELTAS:--0.3 -0.8}"
CONFIGS="${CONFIGS:-fixed signal nosignal}"
PX4_LOG_CAP="${PX4_LOG_CAP:-5M}"
SETTLE_EXTRA_SEC="${SETTLE_EXTRA_SEC:-12}"   # drop settled 之后再留多久收 MHE 收敛日志
BOOT_TIMEOUT_SEC="${BOOT_TIMEOUT_SEC:-240}"  # 单轮等 drop settled 的上限(超则判失败弃轮)

# 每批次独立 manifest(带 BATCH_STAMP):不同批次(冒烟/验证/全量,可能混新旧
# 代码)绝不能被 aggregate 混在一起分组统计——聚合脚本吃指定的这一份。
BATCH_STAMP=$(date +%Y%m%d_%H%M%S)
MANIFEST="$RUNDIR/nosignal_ablation_${BATCH_STAMP}.txt"
echo "# nosignal ablation batch $BATCH_STAMP : cfg delta rep mhe_stamp nmpc_stamp status" > "$MANIFEST"

# ---- 一次性构建 ----
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test offboard_test_acados >/dev/null
source "$WS/install/setup.bash"
export ACADOS_SOURCE_DIR=/home/clear/acados
export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
export PYTHONUNBUFFERED=1
mkdir -p "$MASS_CHANGER_DIR/build"
(cd "$MASS_CHANGER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MASS_CHANGER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

cleanup_sim() {
  for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "ros_gz_bridge" "gcs_heartbeat.py" "ninja gz_x500" "make px4_sitl"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
  done
  sleep 3
}

launch_one() {  # args: cfg delta stamp -> sets globals MHE_LOG NODE_LOG
  local cfg="$1" delta="$2" stamp="$3"
  local trig mode
  case "$cfg" in
    fixed)    trig=false; mode=external ;;
    signal)   trig=true;  mode=external ;;
    nosignal) trig=true;  mode=residual ;;
  esac
  # drop 前置清历史 px4 verbose(护栏),幅度经 env 送进 gz plugin
  find "$RUNDIR" -maxdepth 1 -type f \( -name 'px4_*.log' \) -delete 2>/dev/null || true
  export MASS_CHANGER_DELTA_KG="$delta"

  nohup bash -c "cd '$PX4_DIR' && HEADLESS=1 PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_payload" 2>&1 \
      | ( head -c "$PX4_LOG_CAP" > "$RUNDIR/px4_$stamp.log"; cat >/dev/null ) &
  sleep 15
  nohup python3 "$WS/src/scripts/gcs_heartbeat.py" > "$RUNDIR/gcs_hb_$stamp.log" 2>&1 &
  nohup ros2 launch mavros px4.launch fcu_url:=udp://:14540@ > "$RUNDIR/mavros_$stamp.log" 2>&1 &
  sleep 8
  nohup ros2 run ros_gz_bridge parameter_bridge \
      "/x500_payload_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
      > "$RUNDIR/bridge_$stamp.log" 2>&1 &
  NODE_LOG="$RUNDIR/acados_nmpc_node_$stamp.log"
  nohup ros2 run offboard_test_acados acados_nmpc_node > "$NODE_LOG" 2>&1 &
  MHE_LOG="$RUNDIR/mhe_node_$stamp.log"
  nohup ros2 run offboard_test_acados mhe_node --ros-args \
      -p event_trigger_enable:="$trig" \
      -p schedule_theta:="[-4.0,0.0,0.0,0.0]" \
      -p event_confirm_thresh_n:=1.5 \
      -p event_signal_mode:="$mode" \
      -p resid_persist_frames:="${MHE_RESID_PERSIST:-2}" \
      > "$MHE_LOG" 2>&1 &
}

# 等 "Payload drop settled." 出现在 nmpc 日志;返回 0=成功 1=超时
wait_for_drop() {
  local nmpc_log="$1" waited=0
  while [ "$waited" -lt "$BOOT_TIMEOUT_SEC" ]; do
    if grep -q "Payload drop settled." "$nmpc_log" 2>/dev/null; then return 0; fi
    sleep 3; waited=$((waited+3))
  done
  return 1
}

trap 'echo "[batch] interrupted, cleaning up"; cleanup_sim; exit 130' INT TERM

total=0; ok=0
for delta in $DELTAS; do
  for cfg in $CONFIGS; do
    for rep in $(seq 1 "$REPS"); do
      total=$((total+1))
      cleanup_sim
      stamp=$(date +%Y%m%d_%H%M%S)
      echo "[batch] === cfg=$cfg delta=$delta rep=$rep stamp=$stamp ==="
      launch_one "$cfg" "$delta" "$stamp"
      if wait_for_drop "$RUNDIR/acados_nmpc_node_$stamp.log"; then
        sleep "$SETTLE_EXTRA_SEC"
        status=ok; ok=$((ok+1))
      else
        status=TIMEOUT
        echo "[batch] !! timeout waiting for drop (cfg=$cfg delta=$delta rep=$rep)"
      fi
      echo "$cfg $delta $rep $stamp $stamp $status" >> "$MANIFEST"
      cleanup_sim
    done
  done
done

echo "[batch] done: $ok/$total runs reached drop-settled. manifest: $MANIFEST"
