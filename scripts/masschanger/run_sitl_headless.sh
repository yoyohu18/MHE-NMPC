#!/bin/bash
# 全无头 SITL 批量实验用启动器(2026-07-03):不开任何 gnome-terminal/QGC/RViz/
# plot_logger——桌面渲染零负担,批量跑统计实验专用。与 run_sitl_acados.sh 的
# 差异:
#   - 所有组件 nohup 后台 + 日志重定向(文件名与原脚本同规则,解析管线不变);
#   - GCS 心跳不用 QGC:PX4 的解锁健康检查只要"有 GCS 类型的 MAVLink 心跳",
#     用 pymavlink 起一个 ~10 行的心跳器监听 14550 即可(QGC 本来也是这个口);
#   - 省掉纯可视化组件(RViz/robot_state_publisher/TF/旋翼动画/plot_logger);
#   - 保留 ros_gz_bridge 的 motor_speed 桥——mhe_node 的 T_phys 依赖它,不能省。
# 交互式调试/看轨迹仍用原 run_sitl_acados.sh。
# 环境变量同原脚本:MHE_EVENT_TRIGGER / MHE_SCHEDULE_THETA。
set -e

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
MASS_CHANGER_DIR="$WS/src/offboard_test_acados/gz_plugins/mass_changer"
RUNDIR="$WS/nmpc_test_results"
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUNDIR"

# 磁盘护栏(2026-07-08 加,同 gripper/run_gripper_headless.sh):PX4/gz stdout 无分析
# 价值但单次能涨到数十 GB,曾累计 479G 塞爆盘。分析数据在 mhe/nmpc 日志里。
# ①前置清历史 px4 verbose(留 mhe/nmpc);②本轮只留前 PX4_LOG_CAP 排错。
# 详见记忆 sitl-px4-log-disk-blowup。
PX4_LOG_CAP="${PX4_LOG_CAP:-5M}"
find "$RUNDIR" -maxdepth 1 -type f \( -name 'grip_px4_*.log' -o -name 'px4_*.log' \) -delete 2>/dev/null || true

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

# 1. PX4 SITL + gz(本来就 headless,现在连终端窗口也不开)
# stdout 经护栏消费者:前 PX4_LOG_CAP 存文件,超出的吞进 /dev/null(不关管道、
# px4 不被 SIGPIPE 杀,只是日志停在 CAP)。见 gripper/run_gripper_headless.sh 同段注释。
nohup bash -c "cd '$PX4_DIR' && HEADLESS=1 PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_payload" 2>&1 \
    | ( head -c "$PX4_LOG_CAP" > "$RUNDIR/px4_$STAMP.log"; cat >/dev/null ) &
sleep 15

# 2. GCS 心跳器(顶替 QGC,独立脚本文件保证 teardown 能按进程名 pkill)
nohup python3 "$WS/src/scripts/gcs_heartbeat.py" \
    > "$RUNDIR/gcs_hb_$STAMP.log" 2>&1 &

# 3. MAVROS
nohup ros2 launch mavros px4.launch fcu_url:=udp://:14540@ \
    > "$RUNDIR/mavros_$STAMP.log" 2>&1 &
sleep 8

# 4. 电机转速桥(mhe_node 的 T_phys 数据源,必须有)
nohup ros2 run ros_gz_bridge parameter_bridge \
    "/x500_payload_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
    > "$RUNDIR/bridge_$STAMP.log" 2>&1 &

# 5. NMPC 节点
NODE_LOG="$RUNDIR/acados_nmpc_node_$STAMP.log"
nohup ros2 run offboard_test_acados acados_nmpc_node \
    > "$NODE_LOG" 2>&1 &

# 6. MHE 节点
MHE_LOG="$RUNDIR/mhe_node_$STAMP.log"
nohup ros2 run offboard_test_acados mhe_node --ros-args \
    -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
    -p schedule_theta:="${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}" \
    -p event_confirm_thresh_n:=${MHE_CONFIRM_THRESH:-1.5} \
    -p event_signal_mode:=${MHE_SIGNAL_MODE:-external} \
    -p resid_persist_frames:=${MHE_RESID_PERSIST:-2} \
    > "$MHE_LOG" 2>&1 &

echo "headless stack up: nmpc=$NODE_LOG mhe=$MHE_LOG (event=${MHE_EVENT_TRIGGER:-true} theta=${MHE_SCHEDULE_THETA:-M0} signal=${MHE_SIGNAL_MODE:-external})"
