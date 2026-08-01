#!/bin/bash
# 吊挂夹爪场景 GUI 可视化启动器(2026-07-15):看**当前主配置**的完整飞行——
# Gazebo GUI(物理:drone 抓 box / 抬升 / figure8 / drop 掉落)+ RViz(控制端:
# figure8 参考 vs 实际轨迹 + NMPC 预测 horizon + drone 模型 + 旋翼动画)。
# 与 headless 批量版(run_gripper_headless.sh)的区别:开 GUI、接 RViz、用主配置
# (online 无真值几何 + θ* 学习调度 + α 无真值确认 + drop + figure8 动态)。
# 复用 masschanger 的 URDF/viz 组件(NMPC 路径话题、drone 模型全通用);RViz 配置
# 07-30 改用 gripper 专用的 config/gripper/nmpc_view_gripper_hifly.rviz——r=5 的
# 大 8 字会顶出原配置那个 ±5m 默认网格、Distance:12 也框不住,故单独一份
# (网格 24m、Distance 26、焦点挪到 attach 中心 x=1),masschanger 共用那份不动。
#
# 用法:  [GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 METHOD=thetastar GRIP_DYN_R=5.0 \
#          GRIP_DYN_W=0.283 GRIP_DYN_DZ=0.8 GRIP_DROP_AFTER=55.0 \
#          GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true] \
#          bash src/scripts/gripper/run_sitl_gripper_viz.sh
#        GRIP_DYN_DZ=0 可退回原来的平面 8 字。
# 收栈:关掉各 gnome-terminal 窗口即可;或 pkill -9 -f 'px4_sitl|gz sim|mhe_node|...'。

PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.3}"
# 载荷几何的独立操作先验(完全解耦路径,几何不读 m_est)。默认取本场景标称载荷,
# 与 run_gripper_headless.sh 保持一致——2026-07-20 发现这个可视化脚本漏传该参数,
# 会静默退回耦合路径(floor 兜底),导致"看到的行为"和"实验数据"跑的不是同一套架构。
GEOM_MP_PRIOR="${GRIP_GEOM_MP_PRIOR:-$PAYLOAD_KG}"
ECC_Y="${GRIP_ECC_Y:-0.10}"
METHOD="${METHOD:-M0}"                 # M0 | thetastar
# ⚠️2026-07-31:默认从 thetastar 改为 M0。M0 才是主线部署配置
# (run_gripper_headless 默认 [-4,0,0,0]),演示片理应录主线行为。θ* 已被 2×2 析因
# 消融(20Hz n=8/格)+10Hz 对照证明无可测收益(见记忆 cem-benefit-refuted),
# 继续拿它做演示/验收等于展示一个非交付配置。要录 θ* 对照仍可 METHOD=thetastar。
# 高机动 8 字(2026-07-30 改成与 run_gripper_headless.sh 验证过的那一版一致):
# 原来这里根本不传 grip_dyn_*,静默用节点默认 r=0.8/w=0.25——峰值速度仅
# 0.28 m/s、包络 1.6m×0.8m,肉眼几乎看不出是在"机动"。现在默认 r=5.0/w=0.283
# = 包络 10m×5m、峰值速度 2.0 m/s(倾角 4.7°),即 07-30 那轮实测过的配置
# (attach/detach 双向生效、m_est 误差 <1%、带载 pos_err 中位 0.077m)。
# v_peak=√2·r·w。
# RAMP 语义:0=写死 4.0s;-1=auto_ramp_time(w);>0=显式秒数。**本档显式取 3.0**
# (2026-07-30 定案,与 run_gripper_headless.sh 档位表一致):auto 是纯相对判据
# (把振幅渐增的额外速度压到轨迹特征速度的一半),w 越小要的 ramp 越长——2m/s 档
# 它要 9.37s,只为把起振倾角压到 6.3°,绝对意义上毫无必要(执行器上限 60°)。
# 取 3.0s 后起振峰值倾角 16.4°,仍低于 4m/s 档 auto 自己就要的 23.9°,每轮省 6.4s。
# 只有高速档(w≥0.663)才该用 -1,且那时真正生效的是 auto 里 base=4.0 的下限。
# ⚠️ 改 RAMP **不影响** drop 的 tip 相位:8 字相位 a=w·tc 从 hover 结束就推进,
# alpha 只缩放幅值不改相位(下面 DROP_AFTER 的推导因此不含 ramp 项)。
DYN_R="${GRIP_DYN_R:-5.0}"
DYN_W="${GRIP_DYN_W:-0.283}"
DYN_RAMP="${GRIP_DYN_RAMP:-3.0}"
# 立体 8 字的高度起伏 dz[m](2026-07-30):右叶抬高 dz、左叶压低 dz,交叉点等高
# (裸机 fig8 一直是 dz=0.5,带载这条原来写死平面 dz=0.0)。**节点默认仍是 0.0**
# 保历史批次不变,只有这个可视化脚本默认开成 0.8。
# 为什么是 0.8:z_high=2.0、box 挂在下方 grip_arm_d=0.47 → 低叶时 box 底离地
# 2.0-0.8-0.47=0.73m,跟已验证的抓取高度 grip_z_low=0.7 同量级;峰峰 1.6m 落在
# 5m 宽的叶子上,RViz 侧视看得出明显起伏。再大就啃地了(上限 z_high-arm_d-0.3)。
# ⚠️ DROP_AT_TIP 丢在 a=3π/2 = 轨迹**最低点**,dz 一开 drop 高度就降 dz;drop
# 相位本身不受 dz 影响(相位只由 w·tc 定),所以 DROP_AFTER 不用重算。
DYN_DZ="${GRIP_DYN_DZ:-0.8}"

# drop 时机(2026-07-15 立、07-30 随 w 重算):grip_drop_after_sec 是"lift 完成后
# 最早可丢"的门槛,真正丢的时刻由 acados_nmpc_node._grip_drop_phase 的
# grip_drop_at_fig8_tip 收紧到下一次到达 8 字最左端(a=3π/2)那一帧。
# 门槛与 tc(=drop_after-5.0,扣掉 dyn_settle 3.0 与 hover_time 2.0 的净偏移)对应,
# tip 落在 tc=(3π/2+2πk)/w:
#   w=0.283(2.0m/s,周期 22.2s) -> k=0:16.7s  k=1:38.9s  k=2:61.1s
# 取 55.0 门槛(tc=50.0)落在 k=1 与 k=2 之间 -> 稳稳命中 k=2 = 2.75 圈,
# 既飞满两整圈以上,又留足余量不会因 attach/lift 时长抖动误命中 k=1。
# ⚠️ 改 GRIP_DYN_W 必须重算这个门槛(旧的 60.0 是按 w=0.25/周期 25.1s 算的)。
DROP_AFTER="${GRIP_DROP_AFTER:-55.0}"  # 0=不 drop
DROP_AT_TIP="${GRIP_DROP_AT_TIP:-true}"  # true=对齐到 8 字最左端丢,而非到点硬丢
DYNAMIC="${GRIP_DYNAMIC:-true}"        # figure8 动态

# ⚠️ ROS2 参数强类型:`-p grip_dyn_r:=5` / `grip_drop_after_sec:=55` 会被解析成
# INTEGER,与节点里 DOUBLE 声明冲突抛 InvalidParameterTypeException **打挂节点**
# (07-29/30 在两个 headless 脚本上各踩一次)。这里同样统一规范化。
_f2d() { python3 -c "print(float('$1'))"; }
DYN_R_D=$(_f2d "$DYN_R"); DYN_W_D=$(_f2d "$DYN_W"); DYN_RAMP_D=$(_f2d "$DYN_RAMP")
DYN_DZ_D=$(_f2d "$DYN_DZ")
DROP_AFTER_D=$(_f2d "$DROP_AFTER")
ECC_Y_D=$(_f2d "$ECC_Y"); PAYLOAD_KG_D=$(_f2d "$PAYLOAD_KG")
GEOM_MP_PRIOR_D=$(_f2d "$GEOM_MP_PRIOR")
ATTACH_WINDOW_SEC_D=$(_f2d "${ATTACH_WINDOW_SEC:-40.0}")
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
RVIZ_CONFIG="${RVIZ_CONFIG:-$PKG/config/gripper/nmpc_view_gripper_hifly.rviz}"
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
# 日志护栏(2026-07-31 加):本脚本是长跑录屏用,曾产出**单个 3.0G** 的
# gviz_mavros 日志(整个 nmpc_test_results 3.6G 里它占 3.0G)。mavros stdout 无
# 分析价值、无脚本读取,故同 px4 只留前 MAVROS_LOG_CAP 排错。
# (head; cat>/dev/null) 而非 `| head`:让管道保持打开,mavros 不会吃 SIGPIPE 被杀。
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@" 2>&1 \
  | ( head -c "${MAVROS_LOG_CAP:-5M}" > "$LOGDIR/gviz_mavros_$TS.log"; cat >/dev/null ) &

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
    -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$ECC_Y_D \
    -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG_D -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 -p use_mhe:=true \
    -p geom_source:=online -p grip_payload_prior:=0.3 \
    -p grip_drop_after_sec:=$DROP_AFTER_D \
    -p grip_dynamic_after_lift:=$DYNAMIC \
    -p grip_dyn_r:=$DYN_R_D -p grip_dyn_w:=$DYN_W_D -p grip_dyn_ramp:=$DYN_RAMP_D \
    -p grip_dyn_dz:=$DYN_DZ_D \
    -p grip_drop_at_fig8_tip:=$DROP_AT_TIP \
    -p attach_window_sec:=$ATTACH_WINDOW_SEC_D" \
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
    -p grip_geom_mp_prior:=$GEOM_MP_PRIOR_D" \
  > "$MHE_LOG" 2>&1 &

echo "All components up. Gazebo GUI = 物理飞行; RViz = 控制端跟踪。"
echo "  NMPC: tail -f $NODE_LOG   MHE: tail -f $MHE_LOG"
