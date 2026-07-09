#!/bin/bash
# 夹爪吊挂 + acados NMPC + MHE 重载实验:
#   空机 plain x500 由 acados NMPC(姿态+推力控制)飞到 box 正上方低空悬停
#   -> proximity 节点触发 DetachableJoint 把真实 0.3kg box 焊上来
#   -> acados 定时抬升把 box 吊离地面(全额承重)
#   -> MHE 用电机转速反算真实推力,在线估出质量阶跃,闭环喂回 NMPC 补偿
#   验证:acados NMPC + MHE 能否扛住裸 PID 扛不住的 0.3kg 吊挂载荷突变。
#
# 与 masschanger/run_sitl_acados.sh(mass_changer 基线)独立:那个是满载起飞->drop 减质量,
# 这个是空机起飞->attach 加质量,且用真实独立刚体(DetachableJoint)而非改写
# base_link 惯量。acados 节点 gripper_mode:=true 时才走这套(默认关,不动基线)。
set -e

# 用法: run_sitl_gripper_acados.sh [payload_kg]   (默认 0.3)
# 载荷质量是唯一会在"world 里 box 的物理质量"和"NMPC 的 grip_payload_mass"
# 两处出现的量,必须由同一个变量派生,否则一处改了另一处忘了就是一次白跑
# 的实验。box 惯量按几何(0.15m 立方体)随质量线性缩放: I = m*0.00375。
PAYLOAD_KG="${1:-0.3}"
BOX_I=$(python3 -c "print(f'{$PAYLOAD_KG * 0.00375:.6f}')")
# USE_MHE=true:弱闭环——m 由 MHE 在线估计喂回 NMPC;dJ/c_xy/rate-gain 放大
# 仍按已知 payload 前馈(attach_offset 几何路径,与 MHE 无关)。
# USE_MHE=false:MHE 只作诊断,m_est 走 mass-step 真值前馈(隔离验证控制器)。
USE_MHE="${USE_MHE:-true}"

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
LOGDIR="$WS/nmpc_test_results"
TS=$(date +%Y%m%d_%H%M%S)
ACADOS_ENV="export ACADOS_SOURCE_DIR=/home/clear/acados && export LD_LIBRARY_PATH=/home/clear/acados/lib:\$LD_LIBRARY_PATH"
mkdir -p "$LOGDIR"

# 启动前彻底清残留(旧 gz 世界会被 px4-rc.gzsim 误接、旧节点抢舵)
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

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test offboard_test_acados
source "$WS/install/setup.bash"

mkdir -p "$GRIPPER_DIR/build"
(cd "$GRIPPER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GRIPPER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

# 把 world 里 box 的质量/惯量替换成本次实验的载荷值(源文件不动,只改
# 复制到 PX4 的那份)。sed 锚定 box_link 的 inertial 块里的具体标签。
sed -e "s|<mass>[0-9.]*</mass>|<mass>$PAYLOAD_KG</mass>|" \
    -e "s|<ixx>[0-9.]*</ixx>|<ixx>$BOX_I</ixx>|" \
    -e "s|<iyy>[0-9.]*</iyy>|<iyy>$BOX_I</iyy>|" \
    -e "s|<izz>[0-9.]*</izz>|<izz>$BOX_I</izz>|" \
    "$WORLD_SRC" > "$PX4_WORLDS/gripper_test.sdf"
echo "copied world -> $PX4_WORLDS/gripper_test.sdf (box mass=$PAYLOAD_KG kg, I=$BOX_I)"

# 实验元数据:每次实验固化一份参数快照,复盘时不用去猜"那次跑的是多重"。
# pos_err/力矩饱和/attach 偏移/求解器状态都在各自节点日志里(NMPC 日志的
# MASS+INERTIA+COM STEP / Torque near constraint / res_stat 行,proximity
# 日志的 attach offset 行),这里记下时间戳把它们串起来。
META="$LOGDIR/gacados_meta_$TS.txt"
{
  echo "ts=$TS"
  echo "payload_kg=$PAYLOAD_KG"
  echo "box_inertia=$BOX_I"
  echo "grip_arm_d_fallback=0.47"
  echo "grip_z_low=0.55 grip_z_high=2.5 lift_after=1.5 lift_dur=3.0"
  echo "use_mhe=$USE_MHE mass_step_sec=0.0 scale_px4_rate_gains=true"
  echo "git_rev=$(git -C "$WS/src" rev-parse --short HEAD) dirty=$(git -C "$WS/src" status --porcelain | wc -l)"
} > "$META"
echo "meta -> $META"

# 1. PX4 SITL + Gazebo(GUI 可见,空机 x500,gripper_test world)
gnome-terminal --title="PX4 SITL + Gazebo (gripper+acados)" -- bash -c \
    "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
     cd '$PX4_DIR' && PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

# 2. QGroundControl(新开,连本次 PX4,提供解锁所需 GCS 心跳)
pkill -9 -f QGround 2>/dev/null || true
sleep 1
gnome-terminal --title="QGroundControl" -- bash -c "~/QGroundControl.AppImage; exec bash"

echo "Waiting for PX4 SITL + Gazebo to boot..."
sleep 18

# 3. MAVROS
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@" \
    > "$LOGDIR/gacados_mavros_$TS.log" 2>&1 &

# 4. 电机转速桥(空机 x500_0),给 MHE 反算真实推力用
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    ros2 run ros_gz_bridge parameter_bridge \
      /x500_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
    > "$LOGDIR/gacados_bridge_$TS.log" 2>&1 &

echo "Waiting for MAVROS / bridge..."
sleep 10

# 5. 接近触发节点(r_xy 收紧到 0.15,让 box 几乎正下方才吸,小力臂)
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados proximity_gripper_node --ros-args \
      --params-file '$PKG/config/gripper/gripper_params.yaml' \
      -p drone_model:=x500_0 -p r_xy:=0.15 -p h_min:=0.35 -p h_max:=0.60" \
    > "$LOGDIR/gacados_proximity_$TS.log" 2>&1 &

# 6. 夹取使能(持续发 true,几何条件满足时 proximity 才真正 attach)
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    ros2 topic pub -r 2 /gripper/enable std_msgs/msg/Bool '{data: true}'" \
    > "$LOGDIR/gacados_enable_$TS.log" 2>&1 &

# 7. acados NMPC(gripper_mode:=true:空机起飞->低空悬停到 box->定时抬升)
NODE_LOG="$LOGDIR/gacados_nmpc_$TS.log"
echo "acados NMPC log: $NODE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados acados_nmpc_node --ros-args \
      -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=0.0 \
      -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
      -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG -p grip_arm_d:=0.47 \
      -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 \
      -p use_mhe:=$USE_MHE" \
    > "$NODE_LOG" 2>&1 &

# 8. MHE 在线质量估计(空机 motor 话题),闭环喂回 NMPC
MHE_LOG="$LOGDIR/gacados_mhe_$TS.log"
echo "MHE log: $MHE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
    $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
    ros2 run offboard_test_acados mhe_node --ros-args \
      -p motor_speed_topic:=/x500_0/command/motor_speed \
      -p event_trigger_enable:=true" \
    > "$MHE_LOG" 2>&1 &

echo "All components launched."
echo "  watch NMPC : tail -f $NODE_LOG   (pos_err / T / m_est 用法)"
echo "  watch MHE  : tail -f $MHE_LOG    (质量估计随 attach 阶跃)"
echo "  attach evt : gz topic -e -t /gripper/state"
