#!/bin/bash
# 吊挂夹爪场景 GUI 可视化启动器(2026-07-15):看**当前主配置**的完整飞行——
# Gazebo GUI(物理:drone 抓 box / 抬升 / figure8 / drop 掉落)+ RViz(控制端:
# figure8 参考 vs 实际轨迹 + NMPC 预测 horizon + drone 模型 + 旋翼动画)。
# 与 headless 批量版(run_gripper_headless.sh)的区别:开 GUI、接 RViz、用主配置
# (online 无真值几何 + θ* 学习调度 + α 无真值确认 + drop + figure8 动态)。
# 复用 masschanger 的 rviz 配置/URDF/viz 组件(NMPC 路径话题、drone 模型全通用)。
#
# 用法:  [GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 METHOD=thetastar GRIP_DROP_AFTER=60.0 \
#          GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true] bash src/scripts/gripper/run_sitl_gripper_viz.sh
# 收栈:关掉各 gnome-terminal 窗口即可;或 pkill -9 -f 'px4_sitl|gz sim|mhe_node|...'。

PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.3}"
# 载荷几何的独立操作先验(完全解耦路径,几何不读 m_est)。默认取本场景标称载荷,
# 与 run_gripper_headless.sh 保持一致——2026-07-20 发现这个可视化脚本漏传该参数,
# 会静默退回耦合路径(floor 兜底),导致"看到的行为"和"实验数据"跑的不是同一套架构。
GEOM_MP_PRIOR="${GRIP_GEOM_MP_PRIOR:-$PAYLOAD_KG}"
ECC_Y="${GRIP_ECC_Y:-0.10}"
METHOD="${METHOD:-thetastar}"          # thetastar | M0
# drop 时机(2026-07-15 改):之前默认 8.0s + drop_at_fig8_tip=false 是"动态切换
# 后不到 5 秒硬丢",连一圈 8 字(周期 2π/grip_dyn_w=2π/0.25≈25.1s)的零头都没跑完。
# 改成 tip 对齐 + 60s 门槛:grip_drop_after_sec 是"lift 完成后最早可丢"的门槛,
# 真正丢的时刻由 acados_nmpc_node._grip_drop_phase 里的 grip_drop_at_fig8_tip
# 逻辑收紧到下一次到达 8 字最左端(a=3π/2)那一帧。tip 出现在 dyn_settle_sec
# (3.0)+hover_time(2.0)+(3π/2+2πk)/w 处:k=1≈49.0s(1.75圈,门槛60s时已过,不会
# 命中)、k=2≈74.1s(2.75圈)——60s 门槛卡在两者中间,稳稳落到 k=2,保证飞满
# 两整圈以上才丢,同时留够余量不会因为 attach/lift 时长的小抖动误命中 k=1。
DROP_AFTER="${GRIP_DROP_AFTER:-60.0}"  # 0=不 drop
DROP_AT_TIP="${GRIP_DROP_AT_TIP:-true}"  # true=对齐到 8 字最左端丢,而非到点硬丢
DYNAMIC="${GRIP_DYNAMIC:-true}"        # figure8 动态
BOX_I=$(python3 -c "print(f'{$PAYLOAD_KG * 0.00375:.6f}')")
R_XY=$(python3 -c "print(f'{max(0.20, $ECC_Y + 0.08):.3f}')")

if [ "$METHOD" = thetastar ]; then
  THETA="[-4.8038,-1.2080,0.4930,-0.9602,0.9875]"; CALPHA="0.9875"; CPRIOR="$PAYLOAD_KG"
else
  THETA="[-4.0,0.0,0.0,0.0]"; CALPHA="-1.0"; CPRIOR="0.3"
fi

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
RVIZ_CONFIG="$PKG/config/masschanger/nmpc_view_acados.rviz"
URDF_FILE="$PKG/urdf/masschanger/x500.urdf"
LOGDIR="$WS/nmpc_test_results"
TS=$(date +%Y%m%d_%H%M%S)
ACADOS_ENV="export ACADOS_SOURCE_DIR=/home/clear/acados && export LD_LIBRARY_PATH=/home/clear/acados/lib:\$LD_LIBRARY_PATH"
mkdir -p "$LOGDIR"

echo "cleaning up leftover sim processes..."
for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
           "lib/offboard_test_acados/proximity_gripper_node" \
           "lib/offboard_test_acados/acados_nmpc_node" \
           "lib/offboard_test_acados/mhe_node" \
           "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "prop_joint_state" \
           "drone_tf_broadcaster" "robot_state_publisher" "rviz2" \
           "ninja gz_x500" "make px4_sitl"; do
  for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
done
sleep 2
# PX4 参数持久化护栏(见记忆 px4-param-persistence-guard):清落盘的放大过增益
find "$PX4_DIR/build/px4_sitl_default/rootfs" -maxdepth 1 -name 'parameters*.bson' -delete 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test offboard_test_acados
source "$WS/install/setup.bash"

mkdir -p "$GRIPPER_DIR/build"
(cd "$GRIPPER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make >/dev/null)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GRIPPER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

sed -e "s|<mass>[0-9.]*</mass>|<mass>$PAYLOAD_KG</mass>|" \
    -e "s|<ixx>[0-9.]*</ixx>|<ixx>$BOX_I</ixx>|" \
    -e "s|<iyy>[0-9.]*</iyy>|<iyy>$BOX_I</iyy>|" \
    -e "s|<izz>[0-9.]*</izz>|<izz>$BOX_I</izz>|" \
    "$WORLD_SRC" > "$PX4_WORLDS/gripper_test.sdf"
echo "world -> box mass=$PAYLOAD_KG I=$BOX_I ; method=$METHOD ecc=$ECC_Y drop_after=$DROP_AFTER dynamic=$DYNAMIC"

# 1. PX4 SITL + Gazebo(GUI 可见:HEADLESS 不设)
gnome-terminal --title="Gazebo (gripper viz)" -- bash -c \
  "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
   cd '$PX4_DIR' && PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

# 2. QGC(解锁所需 GCS 心跳)
pkill -9 -f QGround 2>/dev/null || true; sleep 1
gnome-terminal --title="QGroundControl" -- bash -c "~/QGroundControl.AppImage; exec bash"
echo "booting PX4 + Gazebo GUI..."; sleep 18

# 3. MAVROS
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@" > "$LOGDIR/gviz_mavros_$TS.log" 2>&1 &

# 4. 电机转速桥(x500_0)——MHE 的 T_phys/tau_phys 与旋翼动画共用这一份
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  ros2 run ros_gz_bridge parameter_bridge \
    /x500_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators" \
  > "$LOGDIR/gviz_bridge_$TS.log" 2>&1 &
sleep 10

# 5. RViz2(控制端:figure8 参考 vs 实际 + NMPC 预测 horizon + drone 模型)
gnome-terminal --title="RViz2 (gripper trajectories)" -- bash -c \
  "source /opt/ros/jazzy/setup.bash && rviz2 -d '$RVIZ_CONFIG'; exec bash"

# 6. drone 模型 TF + 旋翼动画(RViz 里的 RobotModel + 转动旋翼)
gnome-terminal --title="Drone Model + Rotors" -- bash -c \
  "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && trap 'kill 0' EXIT; \
   ros2 run robot_state_publisher robot_state_publisher \
     --ros-args -p robot_description:=\"\$(cat '$URDF_FILE')\" & \
   ros2 run offboard_test_acados drone_tf_broadcaster & \
   ros2 run offboard_test_acados prop_joint_state_publisher --ros-args \
     -p motor_speed_topic:=/x500_0/command/motor_speed & \
   wait"

# 7. 接近触发节点
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  export PYTHONUNBUFFERED=1 && \
  ros2 run offboard_test_acados proximity_gripper_node --ros-args \
    --params-file '$PKG/config/gripper/gripper_params.yaml' \
    -p drone_model:=x500_0 -p r_xy:=$R_XY -p h_min:=0.35 -p h_max:=0.60" \
  > "$LOGDIR/gviz_proximity_$TS.log" 2>&1 &

# 8. acados NMPC(主配置:gripper_mode + online 几何 + drop + figure8 动态)
NODE_LOG="$LOGDIR/gviz_nmpc_$TS.log"; echo "NMPC log: $NODE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
  ros2 run offboard_test_acados acados_nmpc_node --ros-args \
    -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$ECC_Y \
    -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 -p use_mhe:=true \
    -p geom_source:=online -p grip_payload_prior:=0.3 \
    -p grip_drop_after_sec:=$DROP_AFTER \
    -p grip_dynamic_after_lift:=$DYNAMIC \
    -p grip_drop_at_fig8_tip:=$DROP_AT_TIP \
    -p attach_window_sec:=${ATTACH_WINDOW_SEC:-40.0}" \
  > "$NODE_LOG" 2>&1 &

# 9. MHE(主配置:x500_0 + θ* 调度 + α 无真值确认 + floor + c_xy 在线估计)
MHE_LOG="$LOGDIR/gviz_mhe_$TS.log"; echo "MHE log: $MHE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
  ros2 run offboard_test_acados mhe_node --ros-args \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p event_trigger_enable:=true -p schedule_theta:='$THETA' \
    -p confirm_thresh_alpha:=$CALPHA -p confirm_payload_prior:=$CPRIOR \
    -p grip_geom_mp_floor:=0.15 -p c_xy_est_enable:=true \
    -p grip_geom_mp_prior:=$GEOM_MP_PRIOR" \
  > "$MHE_LOG" 2>&1 &

echo "All components up. Gazebo GUI = 物理飞行; RViz = 控制端跟踪。"
echo "  NMPC: tail -f $NODE_LOG   MHE: tail -f $MHE_LOG"
