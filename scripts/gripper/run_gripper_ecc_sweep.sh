#!/bin/bash
# 坏几何复测(长期计划_20260707.md §阶段B步骤1):扫横向偏心 ry,验证
# mhe_node.py 的自锁重锚修复(2026-07-06,_fail_streak>=5 时重锚先验)在
# 07-06 曾暴露自锁 bug 的大偏心几何下确实生效,顺带标定单集(attach->稳态)
# 耗时,供后续吊挂 CEM 学习(阶段B步骤2)做预算依据。
#
# 偏心怎么来:box 固定在 world 系 (1.0, 0.0, 0.075)(gripper_test.sdf),
# grip_y 是 acados_nmpc_node 的悬停目标,drone 稳态悬停在 (grip_x, grip_y)
# ——两者之差就是 attach 的横向偏心 ry(忽略 cm 级跟踪误差)。这是比等随机
# attach 命中偏心 confirmDeterministic 得多的做法。
#
# 与 run_sitl_gripper_acados.sh 的唯一差异:grip_y 从 0.0 参数化为 $2,
# proximity 的 r_xy 相应放宽(默认 0.15 卡不住 0.135 的偏心目标点)。
set -e

PAYLOAD_KG="${1:-0.3}"
ECC_Y="${2:-0.05}"
BOX_I=$(python3 -c "print(f'{$PAYLOAD_KG * 0.00375:.6f}')")
# r_xy 必须 > ECC_Y 才能触发 attach;+0.08 留出接近段跟踪误差余量。
R_XY=$(python3 -c "print(f'{max(0.20, $ECC_Y + 0.08):.3f}')")
USE_MHE="${USE_MHE:-true}"

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
LOGDIR="$WS/nmpc_test_results"
TS="ecc$(python3 -c "print(f'{$ECC_Y:.3f}'.replace('.','p'))")_$(date +%Y%m%d_%H%M%S)"
ACADOS_ENV="export ACADOS_SOURCE_DIR=/home/clear/acados && export LD_LIBRARY_PATH=/home/clear/acados/lib:\$LD_LIBRARY_PATH"
mkdir -p "$LOGDIR"

echo "cleaning up leftover sim processes..."
for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
           "lib/offboard_test_acados/proximity_gripper_node" \
           "lib/offboard_test_acados/gripper_flight_node" \
           "lib/offboard_test_acados/acados_nmpc_node" \
           "lib/offboard_test_acados/mhe_node" \
           "topic pub -r 2 /gripper/enable" "ros_gz_bridge" \
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
colcon build --packages-select offboard_test_acados > /tmp/ecc_sweep_build.log 2>&1 || \
    { echo "BUILD FAILED, see /tmp/ecc_sweep_build.log"; exit 1; }
source "$WS/install/setup.bash"

mkdir -p "$GRIPPER_DIR/build"
(cd "$GRIPPER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GRIPPER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

sed -e "s|<mass>[0-9.]*</mass>|<mass>$PAYLOAD_KG</mass>|" \
    -e "s|<ixx>[0-9.]*</ixx>|<ixx>$BOX_I</ixx>|" \
    -e "s|<iyy>[0-9.]*</iyy>|<iyy>$BOX_I</iyy>|" \
    -e "s|<izz>[0-9.]*</izz>|<izz>$BOX_I</izz>|" \
    "$WORLD_SRC" > "$PX4_WORLDS/gripper_test.sdf"
echo "copied world -> $PX4_WORLDS/gripper_test.sdf (box mass=$PAYLOAD_KG kg, I=$BOX_I)"

META="$LOGDIR/ecc_sweep_meta_$TS.txt"
{
  echo "ts=$TS"
  echo "payload_kg=$PAYLOAD_KG ecc_y=$ECC_Y r_xy=$R_XY"
  echo "purpose=badgeometry_retest_phase0 (长期计划 阶段B步骤1)"
  echo "git_rev=$(git -C "$WS/src" rev-parse --short HEAD) dirty=$(git -C "$WS/src" status --porcelain | wc -l)"
} > "$META"
echo "meta -> $META"

gnome-terminal --title="PX4 SITL + Gazebo (ecc sweep $ECC_Y)" -- bash -c \
    "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
     cd '$PX4_DIR' && PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

pkill -9 -f QGround 2>/dev/null || true
sleep 1
gnome-terminal --title="QGroundControl" -- bash -c "~/QGroundControl.AppImage; exec bash"

echo "Waiting for PX4 SITL + Gazebo to boot..."
sleep 18

nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@" \
    > "$LOGDIR/ecc_mavros_$TS.log" 2>&1 &

nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    ros2 run ros_gz_bridge parameter_bridge \
      /x500_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
    > "$LOGDIR/ecc_bridge_$TS.log" 2>&1 &

echo "Waiting for MAVROS / bridge..."
sleep 10

nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados proximity_gripper_node --ros-args \
      --params-file '$PKG/config/gripper/gripper_params.yaml' \
      -p drone_model:=x500_0 -p r_xy:=$R_XY -p h_min:=0.35 -p h_max:=0.60" \
    > "$LOGDIR/ecc_proximity_$TS.log" 2>&1 &

nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 topic pub -r 2 /gripper/enable std_msgs/msg/Bool '{data: true}'" \
    > "$LOGDIR/ecc_enable_$TS.log" 2>&1 &

NODE_LOG="$LOGDIR/ecc_nmpc_$TS.log"
echo "acados NMPC log: $NODE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados acados_nmpc_node --ros-args \
      -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$ECC_Y \
      -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
      -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG -p grip_arm_d:=0.47 \
      -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 \
      -p use_mhe:=$USE_MHE" \
    > "$NODE_LOG" 2>&1 &

MHE_LOG="$LOGDIR/ecc_mhe_$TS.log"
echo "MHE log: $MHE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados mhe_node --ros-args \
      -p motor_speed_topic:=/x500_0/command/motor_speed \
      -p event_trigger_enable:=true" \
    > "$MHE_LOG" 2>&1 &

echo "All components launched. ecc_y=$ECC_Y r_xy=$R_XY payload=$PAYLOAD_KG"
echo "  watch NMPC : tail -f $NODE_LOG"
echo "  watch MHE  : tail -f $MHE_LOG"
