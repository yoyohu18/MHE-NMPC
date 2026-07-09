#!/bin/bash
# Launch full SITL stack for offboard_test_acados/acados_nmpc_node:
#   1) PX4 SITL + Gazebo (gz_x500_payload, headless - no 3D GUI). This is a
#      new airframe/model registered specifically for the drop scenario -
#      identical to stock x500 (same x500_base mesh/dynamics, same 4 motor
#      plugins) plus one extra custom gz-sim C++ system plugin
#      (gz_plugins/mass_changer/, built separately below) that owns
#      base_link's mass/inertia as its single source of truth. New files
#      only: Tools/simulation/gz/models/x500_payload/, ROMFS airframe
#      22000_gz_x500_payload (PX4's reserved custom-model range) + one new
#      line in airframes/CMakeLists.txt. Nothing under the existing x500/ or
#      x500_base/ was touched.
#
#   Design note (why "drop" instead of "pick up", and why a custom mass
#   plugin instead of a real second rigid body): the original plan was to
#   leave the payload sitting on a stand somewhere in the world and have the
#   drone fly over and attach to it mid-flight via DetachableJoint. That
#   made the payload a dynamic rigid body that existed independently in the
#   world (whether spawned at runtime via `gz service create` or baked
#   statically into the world SDF) - and in every variant tested, the drone
#   went into a violent, periodic attitude oscillation (~5.6s period,
#   roll/pitch swinging 20-37 degrees, thrust setpoint pinned at saturation)
#   starting right from takeoff and never recovered, even with the payload
#   moved 50m away or spawned already at rest. A second attempt moved the
#   payload link inside the x500 model itself, attached/detached via
#   DetachableJoint at runtime - this turned out to rest on a false
#   assumption (DetachableJoint connecting a model to itself is silently a
#   no-op; a timestamped world-pose time series proved the constraint never
#   transmitted any force, it just looked stable while nothing was pulling
#   on it). Both attempts are dead ends now - the mass step is implemented
#   by a small custom system plugin that overwrites base_link's Inertial
#   component directly via the ECS (EntityComponentManager::SetComponentData),
#   with no extra rigid body of any kind ever existing in the world. It sets
#   LOADED (2.5kg = 2.0kg airframe + 0.5kg payload, CoM assumed coincident -
#   MHE here only estimates the mass scalar, not CoM offset/inertia, so an
#   offset would just be unmodeled torque noise) at Configure time, and
#   switches once to EMPTY (2.0kg) on a /payload/drop_mass message.
#   acados_nmpc_node fires that message after NMPC has been tracking for
#   drop_after_track_sec seconds - a mass step down instead of up. For
#   validating that MHE can converge on an online mass change, the direction
#   of the step doesn't matter, only that it tests the same online
#   convergence behavior.
#   2) MAVROS bridge (connects to PX4 over MAVLink UDP)
#   3) QGroundControl (auto-detects PX4 SITL; PX4's arming health check
#      requires a GCS heartbeat - MAVROS alone doesn't satisfy it)
#   4) RViz2 (reference / actual / predicted trajectory)
#   5) acados_nmpc_node (acados-based NMPC offboard controller)
#   6) plot_logger_acados
#   7) mhe_node (open-loop MHE mass estimator, diagnostic only - does not
#      feed back into the NMPC controller)
#   8) robot_state_publisher + drone_tf_broadcaster (URDF-based x500 model in
#      RViz, driven by a live map->base_link TF)
#   9) ros_gz_bridge + prop_joint_state_publisher (spins the rotor_0..3
#      meshes in RViz using PX4's actual commanded motor speed)
#
# This is the acados counterpart of run_sitl_nmpc.sh - kept as a fully
# separate script so the working CasADi/IPOPT pipeline is never touched.
#
# Gazebo runs headless (HEADLESS=1). Earlier suspected the 3D GUI (running
# it alongside RViz2 + QGroundControl) caused a one-off erratic flight, but
# a controlled re-run with the GUI off reproduced the exact same oscillation
# - that ruled the GUI out. The actual cause (see the design note above) was
# the payload being an independent dynamic body in the world; headless is
# kept here simply because there's no GUI-dependent need to turn it on.
#
# Each component runs in its own terminal window so logs stay readable.
# Close a window (or Ctrl+C inside it) to stop that component.

set -e

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
RVIZ_CONFIG="$WS/src/offboard_test_acados/config/masschanger/nmpc_view_acados.rviz"
URDF_FILE="$WS/src/offboard_test_acados/urdf/masschanger/x500.urdf"
MASS_CHANGER_DIR="$WS/src/offboard_test_acados/gz_plugins/mass_changer"
ACADOS_ENV="export ACADOS_SOURCE_DIR=/home/clear/acados && export LD_LIBRARY_PATH=/home/clear/acados/lib:\$LD_LIBRARY_PATH"

# Rebuild so the latest acados_nmpc_node.py changes are installed
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test offboard_test_acados
source "$WS/install/setup.bash"

# Build the mass_changer gz-sim plugin (plain CMake project, not a colcon
# package - it links against gz-sim8/gz-plugin2/gz-transport13 directly,
# nothing ROS-specific about it). Cheap no-op rebuild if nothing changed.
mkdir -p "$MASS_CHANGER_DIR/build"
(cd "$MASS_CHANGER_DIR/build" && cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null && make)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MASS_CHANGER_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"

# 1. PX4 SITL + Gazebo (headless: no gz sim GUI window). GZ_SIM_SYSTEM_PLUGIN_PATH
#    must be exported inside the gnome-terminal's bash -c too - env vars set
#    in this script's own shell aren't inherited by a fresh gnome-terminal.
gnome-terminal --title="PX4 SITL + Gazebo (headless)" -- bash -c \
    "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
     cd '$PX4_DIR' && HEADLESS=1 PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500_payload; exec bash"

echo "Waiting for PX4 SITL to boot..."
sleep 15

# 2. MAVROS bridge (PX4 sends offboard MAVLink to udp 14540 by default)
gnome-terminal --title="MAVROS" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 launch mavros px4.launch fcu_url:=udp://:14540@; exec bash"

# 3. QGroundControl - required for PX4's GCS-heartbeat arming health check
gnome-terminal --title="QGroundControl" -- bash -c \
    "~/QGroundControl.AppImage; exec bash"

sleep 5

# 4. RViz2 trajectory view
gnome-terminal --title="RViz2 - acados NMPC Trajectories" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && rviz2 -d '$RVIZ_CONFIG'; exec bash"

echo "Waiting for MAVROS to connect to PX4..."
sleep 8

# 5. acados NMPC node (hover_test_mode=True by default - see acados_nmpc_node.py)
# ACADOS_SOURCE_DIR/LD_LIBRARY_PATH set explicitly here rather than relying on
# inherited ~/.bashrc, in case this is ever launched from a context that
# doesn't source it.
# stdout/stderr 同时 tee 到日志文件 - 上次崩溃时 traceback 只留在 gnome-terminal
# 窗口里、进程退出后就看不到了,这次存盘,方便事后排查。
NODE_LOG="$WS/nmpc_test_results/acados_nmpc_node_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$WS/nmpc_test_results"
echo "acados NMPC node log: $NODE_LOG"
gnome-terminal --title="acados NMPC Node" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
     ros2 run offboard_test_acados acados_nmpc_node 2>&1 | tee '$NODE_LOG'; exec bash"

# 6. Plot logger - records reference/actual trajectory + tracking error,
#    saves a PNG to ~/ros2_ws_HJH/nmpc_test_results/ on Ctrl+C (close this window to save+exit)
gnome-terminal --title="Plot Logger (acados)" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     ros2 run offboard_test_acados plot_logger_acados; exec bash"

# 7. MHE mass estimator - open-loop diagnostic node (does not feed back into
#    the NMPC controller). Subscribes to the same odometry as acados_nmpc_node
#    plus its /acados_nmpc/u_opt diagnostic topic; publishes the running mass
#    estimate to /acados_nmpc/mhe_mass_estimate. stdout teed to its own log
#    file, same reasoning as the acados NMPC node's log.
#    事件触发权重调度(M0/M1,mhe_weight_learning.py)默认开启;跑"固定权重"
#    对照组用 MHE_EVENT_TRIGGER=false;权重时间表 theta 默认 M0 二值规则
#    [-4,0,0,0],M1 学出的 θ* 用 MHE_SCHEDULE_THETA="[-4.31,1.79,8.99,0.76]"。
MHE_LOG="$WS/nmpc_test_results/mhe_node_$(date +%Y%m%d_%H%M%S).log"
echo "MHE node log: $MHE_LOG (event_trigger=${MHE_EVENT_TRIGGER:-true}, theta=${MHE_SCHEDULE_THETA:-default M0})"
gnome-terminal --title="MHE Node" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
     ros2 run offboard_test_acados mhe_node --ros-args \
       -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
       -p schedule_theta:='${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}' \
       2>&1 | tee '$MHE_LOG'; exec bash"

# 8. URDF-based drone model in RViz2 (display "Drone Model" in
#    nmpc_view_acados.rviz, RobotModel subscribed to /robot_description).
#    robot_state_publisher loads x500.urdf and broadcasts all the internal
#    fixed-joint TF (base_link->rotor_0..3) on its own, no /joint_states
#    needed; drone_tf_broadcaster supplies the one TF it can't know by
#    itself - map->base_link, straight from live odometry.
#    Both are quiet, long-running visualization helpers with nothing worth
#    a dedicated window for, so they share one terminal ('trap ... EXIT'
#    makes closing the window/Ctrl+C kill both, not leak an orphan).
gnome-terminal --title="Drone Visualization (TF)" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     trap 'kill 0' EXIT; \
     ros2 run robot_state_publisher robot_state_publisher \
       --ros-args -p robot_description:=\"\$(cat '$URDF_FILE')\" & \
     ros2 run offboard_test_acados drone_tf_broadcaster & \
     wait"

# 9. Spinning rotors: bridge PX4's actual motor-speed command out of Gazebo
#    Transport (/x500_payload_0/command/motor_speed, gz.msgs.Actuators) into
#    ROS2 (actuator_msgs/msg/Actuators), then integrate it into /joint_states
#    for the continuous rotor_0..3_joint in x500.urdf. One-way GZ->ROS bridge
#    only (the trailing "[" in the topic spec). Same reasoning as step 8 for
#    sharing one window between the bridge and the integrator.
gnome-terminal --title="Spinning Rotors (GZ bridge)" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     trap 'kill 0' EXIT; \
     ros2 run ros_gz_bridge parameter_bridge \
       /x500_payload_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators & \
     ros2 run offboard_test_acados prop_joint_state_publisher & \
     wait"

echo "All components launched (acados NMPC node, hover-only test). Check the opened terminal windows for logs."
