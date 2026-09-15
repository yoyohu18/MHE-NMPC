#!/bin/bash
# 【保留 · 请勿删除 2026-07-20】这是**唯一不含 acados NMPC / MHE 的纯机构验证手段**:
# 它用 gripper_flight_node 自己走飞行剖面,不经控制器。夹爪吸不住 / DetachableJoint
# 出问题时,用它能把控制器整个摘出去、单独定位机构层故障——其余 gripper 脚本全部
# 耦合了 NMPC,做不到这种隔离。
# 也是唯一真正 `ros2 run` gripper_flight_node 的脚本(其它脚本里出现该名字都只是
# 清栈 kill 列表),所以 gripper_flight_node 与 setup.py 里对应的 entry_point 也须保留。
# 注意:本脚本不适合做控制/估计实验(无 NMPC、无 MHE、无护栏),那类请用
# run_gripper_headless.sh(批量)或 run_sitl_gripper_viz.sh(可视化)。
#
# 磁吸夹爪 attach 演示 SITL:plain x500(空机)飞到 box 上方 -> 接近触发
# attach(DetachableJoint 把 box 焊到 base_link)-> 带着 box 爬升悬停。
#
# 本脚本只启动 gripper 场景:
#   - 空机 gz_x500(不是 gz_x500_payload),没有质量阶跃插件
#   - 用 worlds/gripper/gripper_test.sdf(含独立 box + magnetic_gripper world 插件)
#   - 用 proximity_gripper_node 按接近条件触发 attach
#   - 用 gripper_flight_node 走 takeoff->goto box->hover->lift 剖面
#
# gz 带 GUI 启动(能直接看到吸附+起吊)。QGC 必须新开一个连到本次 PX4 实例
# —— PX4 的解锁健康检查需要 GCS 心跳,没有 QGC 心跳会一直 "Arming denied:
# Resolve system health failures first"。接近节点直接走 gz-transport 读位姿+
# 发 attach/detach,不需要 ros_gz_bridge。ROS 节点后台跑,日志写到
# nmpc_test_results/gripper_*.log。
set -e

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
LOGDIR="$WS/nmpc_test_results"
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
bash "$WS/src/scripts/record_experiment_provenance.sh" \
  "$LOGDIR/provenance_$TS" "gripper-mechanism:$TS"

# 启动前彻底清掉上一次的残留 —— 否则 px4-rc.gzsim 会检测到"已有 world 在跑"
# 直接接上旧 gz 服务器,新 PX4 的 EKF 与实际 gz 无人机脱节;或残留的旧飞行/
# 接近节点和新节点抢同一架无人机的 setpoint,把飞行打乱。这是本脚本反复踩过
# 的坑,务必先清干净。
echo "cleaning up any leftover sim processes..."
for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
           "lib/offboard_test_acados/gripper_flight_node" \
           "lib/offboard_test_acados/proximity_gripper_node" \
           "ninja gz_x500" "make px4_sitl"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null || true; done
done
sleep 2

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test_acados
source "$WS/install/setup.bash"

# 编译 magnetic_gripper 插件(独立 CMake)
mkdir -p "$GRIPPER_DIR/build"
(cd "$GRIPPER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GRIPPER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

# 让 PX4 能加载 gripper_test world
cp -f "$WORLD_SRC" "$PX4_WORLDS/gripper_test.sdf"
echo "copied world -> $PX4_WORLDS/gripper_test.sdf"

# 1. PX4 SITL + Gazebo(GUI 可见,空机 x500,gripper_test world)
gnome-terminal --title="PX4 SITL + Gazebo (gripper)" -- bash -c \
    "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
     cd '$PX4_DIR' && PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

# 2. QGroundControl(新开一个,连到本次 PX4,提供解锁所需 GCS 心跳)
pkill -9 -f QGround 2>/dev/null || true   # 没有旧实例时 pkill 返回 1,别让 set -e 中断
sleep 1
gnome-terminal --title="QGroundControl (gripper)" -- bash -c \
    "~/QGroundControl.AppImage; exec bash"

echo "Waiting for PX4 SITL + Gazebo to boot..."
sleep 18

# 3. MAVROS
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@" \
    > "$LOGDIR/gripper_mavros_$TS.log" 2>&1 &

echo "Waiting for MAVROS..."
sleep 10

# 4. 接近触发节点(参数文件 + 显式 drone_model)
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados proximity_gripper_node --ros-args \
      --params-file '$PKG/config/gripper/gripper_params.yaml' \
      -p drone_model:=x500_0" \
    > "$LOGDIR/gripper_proximity_$TS.log" 2>&1 &

# 5. 演示飞行节点
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados gripper_flight_node" \
    > "$LOGDIR/gripper_flight_$TS.log" 2>&1 &

echo "All gripper components launched."
echo "  PX4+gz : its own terminal window (GUI)"
echo "  logs   : $LOGDIR/gripper_{mavros,proximity,flight}_$TS.log"
echo "  watch  : gz topic -e -t /gripper/state   (ATTACHED/DETACHED + sim_time)"
