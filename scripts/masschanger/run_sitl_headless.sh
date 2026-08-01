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
# FIG8_R/FIG8_W:裸机 8 字的尺度(高机动消融,2026-07-29)。默认 1.0/0.3 跟历史
# 批次一致(包络 2m×1m、峰值速度仅 0.42 m/s)。换算:包络=2r×r,峰值速度
# v_peak=√2·r·w,峰值水平加速度=v_peak²/r。常用档(r=5 → 10m×5m 包络):
#   w=0.283→2m/s(4.7°)  0.424→3m/s(10.4°)  0.566→4m/s(18.1°)  0.707→5m/s(27.0°)
# FIG8_RAMP:0=沿用写死的 4.0s(默认,历史批次逐字节复现);-1=auto_ramp_time(w);
#   >0=显式秒数。**高机动别一律用 -1**(2026-07-30 修正原来那句"必须用 -1"):
#   auto=max(4.0,2.65/w) 是纯相对判据(把振幅渐增的额外速度压到轨迹特征速度的
#   一半),w 越小要的 ramp 越长——v=0.42 档它要 44.6s,只为把起振倾角压到 0.28°,
#   绝对意义上毫无必要(执行器上限 60°)。且 w≥0.663 时 2.65/w<4.0,高速档真正
#   生效的是 base=4.0 而非自适应项。r=5 各档 ramp=3.0s 的起振峰值倾角:
#   v=0.42→3.8° / 1.0→8.6° / 2.0→16.4° / 3.0→24.6°,全在预算内。
#   批量对比见 run_speed_ladder.sh(已固定 TRAJ_RAMP=3.0 消掉窗口起点的混杂)。
# TRAJ_SCALE_WEIGHTS:Bryson 权重是否跟着 (r,w) 走。L_vel/L_omega 在
#   acados_params 里按 r=1.0/w=0.3 写死,高机动下跟实际信号量级差一个数量级。
#   注意开启后 L_omega 也会一并被修正(原值是按圆形轨迹标的,对 8 字本就偏紧
#   3.2 倍)——速度尺度和这项修正的效应会混在一起,要分开归因就跑 on/off 两组。
# ⚠️ ROS2 参数是强类型的:`-p fig8_r:=5` 会被解析成 INTEGER,跟节点里
# declare_parameter(..., 1.0) 的 DOUBLE 冲突,直接抛 InvalidParameterTypeException
# 把节点打挂(2026-07-29 首轮 smoke 就是这么死的,FIG8_RAMP=-1)。统一规范化成
# 带小数点的字面量,这样 FIG8_R=5 / FIG8_RAMP=-1 这类自然写法都能用。
_f2d() { python3 -c "print(float('$1'))"; }
FIG8_R_D=$(_f2d "${FIG8_R:-1.0}")
FIG8_W_D=$(_f2d "${FIG8_W:-0.3}")
FIG8_RAMP_D=$(_f2d "${FIG8_RAMP:-0.0}")

NODE_LOG="$RUNDIR/acados_nmpc_node_$STAMP.log"
nohup ros2 run offboard_test_acados acados_nmpc_node --ros-args \
    -p fig8_r:=$FIG8_R_D -p fig8_w:=$FIG8_W_D \
    -p fig8_ramp:=$FIG8_RAMP_D \
    -p traj_scale_weights:=${TRAJ_SCALE_WEIGHTS:-false} \
    -p payload_enabled:=${PAYLOAD_ENABLED:-true} \
    > "$NODE_LOG" 2>&1 &

# 6. MHE 节点
MHE_LOG="$RUNDIR/mhe_node_$STAMP.log"
nohup ros2 run offboard_test_acados mhe_node --ros-args \
    -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
    -p schedule_theta:="${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}" \
    -p event_confirm_thresh_n:=$(_f2d "${MHE_CONFIRM_THRESH:-1.5}") \
    -p event_signal_mode:=${MHE_SIGNAL_MODE:-external} \
    -p resid_persist_frames:=${MHE_RESID_PERSIST:-2} \
    > "$MHE_LOG" 2>&1 &

echo "headless stack up: nmpc=$NODE_LOG mhe=$MHE_LOG (event=${MHE_EVENT_TRIGGER:-true} theta=${MHE_SCHEDULE_THETA:-M0} signal=${MHE_SIGNAL_MODE:-external})"
