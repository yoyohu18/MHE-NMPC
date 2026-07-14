#!/bin/bash
# 吊挂夹爪场景的全无头启动器(2026-07-07):跟 masschanger/run_sitl_headless.sh 是同一套
# 无头模式(不开任何 gnome-terminal/QGC,GCS 心跳用 gcs_heartbeat.py 顶替),
# 但载具换成 gripper/attach(空机 gz_x500 + magnetic_gripper + DetachableJoint
# 真刚体),不是 wrench(mass_changer)。这是给吊挂 CEM 学习(长期计划 §阶段B
# 步骤2)准备的批量驱动底座——今天(坏几何复测)已经验证过 dJ/c_xy + 棘轮修复
# 在物理可行偏心范围内(~0.05-0.12m)稳定可靠,可以放心当训练场用。
#
# 接口跟 masschanger/run_sitl_headless.sh 保持一致(环境变量,不是位置参数),方便以后
# 写 CEM 驱动脚本时直接复用同一种 subprocess.Popen(env=...) 调用方式:
#   GRIP_PAYLOAD_KG    载荷质量 kg (默认 0.3)
#   GRIP_ECC_Y         横向偏心 m,即 grip_y (默认 0.05)——box 固定在世界系
#                      (1.0,0.0),grip_y 是悬停目标,两者之差就是 attach 偏心
#                      (方法见 run_gripper_ecc_sweep.sh 文件头)
#   MHE_EVENT_TRIGGER  事件触发开关 (默认 true)
#   MHE_SCHEDULE_THETA 权重时间表 5 维 (默认 M0 规则版)
#   MHE_CONFIRM_THRESH 确认阈值 [N] (默认 1.5——⚠️这个值是给 wrench 的 gz CLI
#                      冷启动延迟调的,gripper attach 物理瞬时生效,不能直接沿用,
#                      需要针对 gripper 重新标定,这里只是占位默认值)
#
# 与 run_gripper_ecc_sweep.sh 的差异:那个是交互式单点复测(GUI+QGC,慢但直观);
# 这个是无头批量驱动(零桌面渲染负担),CEM/批量对比实验用这个。
set -e

GRIP_PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.3}"
GRIP_ECC_Y="${GRIP_ECC_Y:-0.05}"
BOX_I=$(python3 -c "print(f'{$GRIP_PAYLOAD_KG * 0.00375:.6f}')")
R_XY=$(python3 -c "print(f'{max(0.20, $GRIP_ECC_Y + 0.08):.3f}')")
USE_MHE="${USE_MHE:-true}"

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
RUNDIR="$WS/nmpc_test_results"
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUNDIR"

# 磁盘护栏(2026-07-08 加):PX4/gz 的 stdout 是无分析价值的 verbose 刷屏,
# 单次长跑(如 CEM 训练)能把 grip_px4_*.log 涨到数十 GB,344 个文件曾累计
# 479G 塞爆盘。分析数据全在 grip_mhe/grip_nmpc 里(每个才 KB-MB),px4 日志
# 只在启动排错时看前几秒。所以:①前置清掉历史 px4 verbose(保留 mhe/nmpc);
# ②本轮 px4 stdout 只留前 PX4_LOG_CAP(默认 5M)排错用,之后全丢 /dev/null。
# 详见记忆 sitl-px4-log-disk-blowup。
PX4_LOG_CAP="${PX4_LOG_CAP:-5M}"
find "$RUNDIR" -maxdepth 1 -type f \( -name 'grip_px4_*.log' -o -name 'px4_*.log' \) -delete 2>/dev/null || true

echo "cleaning up leftover sim processes..."
for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
           "lib/offboard_test_acados/proximity_gripper_node" \
           "lib/offboard_test_acados/acados_nmpc_node" \
           "lib/offboard_test_acados/mhe_node" \
           "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py" \
           "ninja gz_x500" "make px4_sitl"; do
  for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
done
sleep 2

# PX4 参数持久化护栏(2026-07-08 加):_scale_px4_rate_gains 用 mavros
# ParamSetV2 改 MC_ROLLRATE_K/MC_PITCHRATE_K,PX4 固件把它当普通 PARAM_SET
# 处理,会落盘到 rootfs/parameters.bson——"干净重启"只重启进程,不清这个
# 文件,下一局起飞会带着上一局 attach 后放大过的增益去纠正裸机小惯量的
# 正常扰动,导致电机打满爬不起来(ry=0.08 复测踩过)。每轮起飞前必须清空。
find /home/clear/PX4-Autopilot/build/px4_sitl_default/rootfs \
    -maxdepth 1 -name 'parameters*.bson' -delete 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test_acados >/dev/null
source "$WS/install/setup.bash"
export ACADOS_SOURCE_DIR=/home/clear/acados
export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH
export PYTHONUNBUFFERED=1

mkdir -p "$GRIPPER_DIR/build"
(cd "$GRIPPER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GRIPPER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

# box 质量/惯量按本次载荷改写(源文件不动,只改复制到 PX4 的那份)
sed -e "s|<mass>[0-9.]*</mass>|<mass>$GRIP_PAYLOAD_KG</mass>|" \
    -e "s|<ixx>[0-9.]*</ixx>|<ixx>$BOX_I</ixx>|" \
    -e "s|<iyy>[0-9.]*</iyy>|<iyy>$BOX_I</iyy>|" \
    -e "s|<izz>[0-9.]*</izz>|<izz>$BOX_I</izz>|" \
    "$WORLD_SRC" > "$PX4_WORLDS/gripper_test.sdf"

# 1. PX4 SITL + gz(headless,不开终端窗口)
# stdout 经护栏消费者:前 PX4_LOG_CAP 存文件(排错用),超出的全吞进 /dev/null。
# 关键——用 (head; cat>/dev/null) 而非 `| head`:head 读够就退,cat 继续吞剩余,
# 管道永不关闭,px4 收不到 SIGPIPE 不会被杀,只是日志停在 CAP。
nohup bash -c "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
    cd '$PX4_DIR' && HEADLESS=1 PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500" 2>&1 \
    | ( head -c "$PX4_LOG_CAP" > "$RUNDIR/grip_px4_$STAMP.log"; cat >/dev/null ) &
sleep 18

# 2. GCS 心跳器(顶替 QGC)
nohup python3 "$WS/src/scripts/gcs_heartbeat.py" \
    > "$RUNDIR/grip_gcs_hb_$STAMP.log" 2>&1 &

# 3. MAVROS
nohup ros2 launch mavros px4.launch fcu_url:=udp://:14540@ \
    > "$RUNDIR/grip_mavros_$STAMP.log" 2>&1 &
sleep 8

# 4. 电机转速桥(空机 x500_0,mhe_node 的 T_phys 数据源)
nohup ros2 run ros_gz_bridge parameter_bridge \
    "/x500_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
    > "$RUNDIR/grip_bridge_$STAMP.log" 2>&1 &
sleep 2

# 5. 接近触发节点
nohup ros2 run offboard_test_acados proximity_gripper_node --ros-args \
    --params-file "$PKG/config/gripper/gripper_params.yaml" \
    -p drone_model:=x500_0 -p r_xy:=$R_XY -p h_min:=0.35 -p h_max:=0.60 \
    > "$RUNDIR/grip_proximity_$STAMP.log" 2>&1 &

# 6. (方案a)gripper enable 不再由脚本持续发——改由 NMPC 节点在
#    descend 到位后主动发一次(见 acados_nmpc_node.py 的 _descend_phase)。
#    这样 attach 时机由控制器决定,是受控可重复事件,不再靠飞机爬升途中
#    路过几何窗口被动触发(旧行为让 attach 比 NMPC 接管早 5.5s,MHE 事件
#    窗口没机会热身)。

# 7. acados NMPC(gripper_mode,两段式接近->attach->定时抬升)
NODE_LOG="$RUNDIR/grip_nmpc_$STAMP.log"
nohup ros2 run offboard_test_acados acados_nmpc_node --ros-args \
    -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$GRIP_ECC_Y \
    -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$GRIP_PAYLOAD_KG -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 \
    -p use_mhe:=$USE_MHE \
    -p geom_source:=${NMPC_GEOM_SOURCE:-truth} \
    > "$NODE_LOG" 2>&1 &

# 8. MHE(dJ/c_xy + 棘轮修复已内建在 mhe_node.py,attach_offset 一到就自动生效)
# GRIP_TRUE_PAYLOAD_MASS(2026-07-08 诊断用,默认 0=关闭):>0 时 mhe_node 的
# dJ/c_xy 直接用这个真值算,不走 self.m_est 反推的棘轮——排查 0.15kg CEM
# 结构性失败是否是"用质量反推几何缩放"这个自举耦合导致的。诊断专用,不是
# 正式 CEM 花名册的默认路径。
MHE_LOG="$RUNDIR/grip_mhe_$STAMP.log"
nohup ros2 run offboard_test_acados mhe_node --ros-args \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
    -p schedule_theta:="${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}" \
    -p event_confirm_thresh_n:=${MHE_CONFIRM_THRESH:-1.5} \
    -p grip_true_payload_mass:=${GRIP_TRUE_PAYLOAD_MASS:-0.0} \
    -p grip_geom_mp_floor:=${GRIP_GEOM_MP_FLOOR:-0.0} \
    -p c_xy_est_enable:=${MHE_C_XY_EST:-false} \
    > "$MHE_LOG" 2>&1 &

echo "gripper headless stack up: nmpc=$NODE_LOG mhe=$MHE_LOG"
echo "  payload=${GRIP_PAYLOAD_KG}kg ecc_y=${GRIP_ECC_Y}m r_xy=$R_XY"
echo "  event=${MHE_EVENT_TRIGGER:-true} theta=${MHE_SCHEDULE_THETA:-M0} confirm_thresh=${MHE_CONFIRM_THRESH:-1.5}"
echo "  true_payload_mass=${GRIP_TRUE_PAYLOAD_MASS:-0.0} geom_mp_floor=${GRIP_GEOM_MP_FLOOR:-0.0}kg"
