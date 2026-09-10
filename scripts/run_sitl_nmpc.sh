#!/bin/bash
# Launch full SITL stack for offboard_test/nmpc_node:
#   1) PX4 SITL + Gazebo (gz_x500, headless - no 3D GUI)
#   2) MAVROS bridge (connects to PX4 over MAVLink UDP)
#   3) QGroundControl (auto-detects PX4 SITL)
#   4) RViz2 (reference / actual / predicted trajectory)
#   5) nmpc_node (CasADi NMPC offboard controller)
#
# Gazebo runs headless (HEADLESS=1) so RViz2 + QGroundControl are the only
# 3D/GUI renderers - running Gazebo's own 3D GUI alongside them can overload
# the X server and freeze the desktop.
#
# Each component runs in its own terminal window so logs stay readable.
# Close a window (or Ctrl+C inside it) to stop that component.

set -e

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
RVIZ_CONFIG="$WS/src/offboard_test/config/nmpc_view.rviz"
LOGDIR="$WS/nmpc_test_results"
TS=$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOGDIR"
bash "$WS/src/scripts/record_experiment_provenance.sh" \
  "$LOGDIR/provenance_$TS" "nmpc-sitl:$TS"

# Rebuild so the latest nmpc_node.py changes are installed
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test
source "$WS/install/setup.bash"

# 1. PX4 SITL + Gazebo (headless: no gz sim GUI window)
gnome-terminal --title="PX4 SITL + Gazebo (headless)" -- bash -c \
    "cd '$PX4_DIR' && HEADLESS=1 PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

echo "Waiting for PX4 SITL to boot..."
sleep 15

# 2. MAVROS bridge (PX4 sends offboard MAVLink to udp 14540 by default)
gnome-terminal --title="MAVROS" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && \
     ros2 launch mavros px4.launch fcu_url:=udp://:14540@; exec bash"

# 3. QGroundControl
gnome-terminal --title="QGroundControl" -- bash -c \
    "~/QGroundControl.AppImage; exec bash"

sleep 5

# 4. RViz2 trajectory view
gnome-terminal --title="RViz2 - NMPC Trajectories" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && rviz2 -d '$RVIZ_CONFIG'; exec bash"

echo "Waiting for MAVROS to connect to PX4..."
sleep 8

# 5. NMPC node (testing corrected hover thrust = 0.73)
# gnome-terminal --title="Offboard Hover Node" -- bash -c \
#     "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
#      ros2 run offboard_test offboard_node; exec bash"
gnome-terminal --title="NMPC Node" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     ros2 run offboard_test nmpc_node; exec bash"

# 6. Plot logger - records reference/actual trajectory + tracking error,
#    saves a PNG to ~/ros2_ws_HJH/nmpc_test_results/ on Ctrl+C (close this window to save+exit)
gnome-terminal --title="Plot Logger" -- bash -c \
    "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
     ros2 run offboard_test plot_logger; exec bash"

echo "All components launched (NMPC node, hover-only test). Check the opened terminal windows for logs."
