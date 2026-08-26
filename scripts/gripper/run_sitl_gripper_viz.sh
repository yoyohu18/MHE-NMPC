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
#          GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
#          GRIP_PAYLOAD_ENVELOPE=0.5 MHE_CONFIRM_ALPHA=0.9875] \
#          bash src/scripts/gripper/run_sitl_gripper_viz.sh
#        GRIP_DYN_DZ=0 可退回原来的平面 8 字。
# 收栈:关掉各 gnome-terminal 窗口即可;或 pkill -9 -f 'px4_sitl|gz sim|mhe_node|...'。

PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.3}"
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
# 悬停/8字高度与 LIFT 时长(2026-08-19 参数化,默认值不变)。
# 抬高高度的用途:大尺度轨迹(r=10)那轮是**被地面终止**的——求解器失败后推力跌到
# 18~21N(带载悬停需 23.2N),z 从 3.23 一路掉到 0.18 撞地,看不到失效的真实形态。
# 把地面移远才能观察"掉多少就稳住/还是持续发散"。
# ⚠️ 两者必须一起调:LIFT 是在 GRIP_LIFT_DUR 秒内从 grip_z_low(0.55) 抬到 Z_HIGH,
#    默认 (2.5-0.55)/3.0 = 0.65 m/s;抬到 10m 还用 3.0s 就是 3.2 m/s 爬升率。
#    保持 ~0.65 m/s 的话 LIFT_DUR ≈ (Z_HIGH-0.55)/0.65。
# ⚠️ z 实际范围是 Z_HIGH ± GRIP_DYN_DZ(立体 8 字),留够离地余量。
# ⚠️ ROS2 的 bool 也是强类型:`-p foo:=1` 会被当 INTEGER,声明为 BOOL 的参数直接
# 抛 InvalidParameterTypeException —— **节点启动即崩**,崩在自己的 nohup 日志里,
# 脚本照样往下走,整轮静默没有该节点(2026-08-25 headless 侧实测踩过)。
_b() { case "$(echo "${1:-}" | tr 'A-Z' 'a-z')" in
         1|true|yes|on|y) echo true ;; *) echo false ;; esac; }
_f2dv() { python3 -c "print(float('$1'))"; }
# LIFT 中途 hold + dJ 跟随 m_est + 机动门控(2026-08-25,默认全关)。
# 动机与语义见 acados_nmpc_node.py 的 grip_lift_hold_enable / dj_track_mest 注释,
# 以及 mhe_node.py 的 maneuver_gate_enable 注释。
LIFT_HOLD=$(_b "${GRIP_LIFT_HOLD:-false}")
LIFT_HOLD_DZ_D=$(_f2dv "${GRIP_LIFT_HOLD_DZ:-0.35}")
LIFT_HOLD_SEC_D=$(_f2dv "${GRIP_LIFT_HOLD_SEC:-3.0}")
DJ_TRACK=$(_b "${DJ_TRACK_MEST:-false}")
# drop 时是否把 mass_event 发给 MHE。false = 让 MHE 自己从 T_phys 残差看出来
# (须配 MHE_SIGNAL_MODE=residual)。默认 true = 历史行为。
DROP_PUB_EVENT=$(_b "${DROP_PUBLISH_MASS_EVENT:-true}")
MP_CAP_D=$(_f2dv "${GRIP_MP_CAP:-0.6}")
MANEUVER_GATE=$(_b "${MHE_MANEUVER_GATE:-false}")
RESID_RELEASE_GEOM=$(_b "${MHE_RESID_RELEASE_GEOM:-true}")
# 并行阶跃判据(2026-08-26,默认关):短窗前后均值差,不需慢基线因而没有预热失效面。
# 离线回放验证见 test/test_step_detector_replay.py(drop 延迟 0.30s、余量 2.4×、
# 稳态零误触发)。⚠️ 轨迹切换与质量突变在推力通道上不可分,所以释放几何用一条
# 单独的更高门槛 RESID_STEP_REL(默认 2.0N)。
RESID_STEP=$(_b "${MHE_RESID_STEP:-false}")
# c_xy 来源:1 = 用一阶质量矩 s/m_T(窗口内估计,无稳态门控,机动中也更新);
# 0 = 窗外 EMA + 稳态门控(现状,figure-8 中一次都不更新)。需 MHE_ESTIMATE_MOMENT=1。
C_XY_FROM_MOMENT=$(_b "${MHE_C_XY_FROM_MOMENT:-false}")
RESID_STEP_HALF="${MHE_RESID_STEP_HALF:-3}"
RESID_STEP_PERSIST="${MHE_RESID_STEP_PERSIST:-2}"
RESID_STEP_TH_D=$(_f2dv "${MHE_RESID_STEP_TH:-1.0}")
RESID_STEP_REL_D=$(_f2dv "${MHE_RESID_STEP_REL:-2.0}")
MG_OMEGA_D=$(_f2dv "${MHE_MG_OMEGA:-0.15}")
MG_VEL_D=$(_f2dv "${MHE_MG_VEL:-0.20}")
MG_CAP_D=$(_f2dv "${MHE_MG_CAP:-10000.0}")
MG_EXP_D=$(_f2dv "${MHE_MG_EXP:-2.0}")

Z_HIGH="${GRIP_Z_HIGH:-2.5}"
LIFT_DUR="${GRIP_LIFT_DUR:-3.0}"

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
Z_HIGH_D=$(_f2d "$Z_HIGH"); LIFT_DUR_D=$(_f2d "$LIFT_DUR")
DROP_AFTER_D=$(_f2d "$DROP_AFTER")
ECC_Y_D=$(_f2d "$ECC_Y"); PAYLOAD_KG_D=$(_f2d "$PAYLOAD_KG")
# 载荷质量信息的**唯一**入口(2026-08-26 去先验改造,与 run_gripper_headless.sh 对齐)。
#   GRIP_PAYLOAD_ENVELOPE = 机架**能挂的最大载荷**,是平台规格不是"这个包裹多重",
#   所以它**不违反"不能知道包裹质量"的原则**。同时驱动 NMPC 模型侧 dJ 初值、
#   PX4 内环增益整定、MHE 几何标度、事件确认阈值 —— 原先这四条路各有一个任务
#   信息型先验(grip_payload_prior/grip_gain_prior/grip_geom_mp_prior+floor/
#   confirm_payload_prior),已全部从节点里删除。
#   0.3kg 工况下任何 >=0.2937 的包线值都给出逐位相同的 MC_*RATE_K(ratio 撞
#   cap 5.0),数值精度从未被使用。
#   ⚠️ 轻载(0.15/0.2kg)裸 ratio 只有 3.18/3.84 不在 cap 里,换包线是**真的**改
#      整定,必须实测过增益/高频振荡才能采信。
PAYLOAD_ENVELOPE_D=$(_f2d "${GRIP_PAYLOAD_ENVELOPE:-0.5}")
ATTACH_WINDOW_SEC_D=$(_f2d "${ATTACH_WINDOW_SEC:-40.0}")
BOX_I=$(python3 -c "print(f'{$PAYLOAD_KG * 0.00375:.6f}')")
R_XY=$(python3 -c "print(f'{max(0.20, $ECC_Y + 0.08):.3f}')")

if [ "$METHOD" = thetastar ]; then
  THETA="[-4.8038,-1.2080,0.4930,-0.9602,0.9875]"; CALPHA="0.9875"
else
  THETA="[-4.0,0.0,0.0,0.0]"; CALPHA="-1.0"
fi
# 确认阈值 = α·g·grip_payload_envelope(CALPHA<0 时走固定 event_confirm_thresh_n)。
# 2026-08-26 前这里是 MHE_CONFIRM_PRIOR(默认回落到 $PAYLOAD_KG = box 真值)。
# α 与 θ 正交(ParametricWeightSchedule 只读 theta[0..3],α 走独立参数)——
# run_gripper_headless.sh 早就是两个独立环境变量,viz 这边却把它们绑死在 METHOD
# 分支里,导致"想让 α 阈值生效"只能连带切到已被析因消融证伪的
# θ*(见记忆 cem-benefit-refuted)。2026-08-22 解绑:MHE_CONFIRM_ALPHA 可单独
# 覆盖,于是能跑 run_alpha_only_ablation.sh 里的 alphaonly 臂 = M0 节奏 + α 阈值。
CALPHA="${MHE_CONFIRM_ALPHA:-$CALPHA}"

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

# 日志终端的窗口位置(2026-08-23):这 4 个 gnome-terminal 是**后起**的,z-order 必然
# 压在先起的 Gazebo 之上。08-23 take2 就是这么废掉的——Gazebo 落到副屏,4 个终端
# (+1944+27 / +1994+77 / +2039+144 / +2853+144)正好级联盖住它的 3D 视图核心区,
# 整半屏没法用。而 Gazebo 落哪块屏又不受控(gui.config 的 position_x 时灵时不灵)。
# DEMO_TERM_BOTTOM=1 把 4 个终端钉到两块屏的**底部窄条**,那里本来就是 Gazebo 的
# 播放条 / RViz 的状态栏,裁剪时反正要切掉。默认空 = 保持原行为不变。
TG0=(); TG1=(); TG2=(); TG3=()
if [ "${DEMO_TERM_BOTTOM:-0}" = 1 ]; then
  TG0=(--geometry=100x2+0+960);    TG1=(--geometry=100x2+900+960)
  TG2=(--geometry=100x2+1920+960); TG3=(--geometry=100x2+2820+960)
fi

# 1. PX4 SITL + Gazebo(GUI 可见:HEADLESS 不设)
gnome-terminal --title="Gazebo (gripper viz)" "${TG0[@]}" -- bash -c \
  "export GZ_SIM_SYSTEM_PLUGIN_PATH='$GZ_SIM_SYSTEM_PLUGIN_PATH' && \
   cd '$PX4_DIR' && PX4_GZ_WORLD=gripper_test PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_x500; exec bash"

# 2. QGC(解锁所需 GCS 心跳)
pkill -9 -f QGround 2>/dev/null || true; sleep 1
gnome-terminal --title="QGroundControl" "${TG1[@]}" -- bash -c "~/QGroundControl.AppImage; exec bash"
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
gnome-terminal --title="RViz2 (gripper trajectories)" "${TG2[@]}" -- bash -c \
  "source /opt/ros/jazzy/setup.bash && rviz2 -d '$RVIZ_CONFIG'; exec bash"

# 6. drone 模型 TF + 旋翼动画(RViz 里的 RobotModel + 转动旋翼)
gnome-terminal --title="Drone Model + Rotors" "${TG3[@]}" -- bash -c \
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
    -p grip_z_low:=0.55 -p grip_z_high:=$Z_HIGH_D \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG_D -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=$LIFT_DUR_D -p use_mhe:=true \
    -p grip_lift_hold_enable:=$LIFT_HOLD \
    -p grip_lift_hold_dz:=$LIFT_HOLD_DZ_D -p grip_lift_hold_sec:=$LIFT_HOLD_SEC_D \
    -p dj_track_mest:=$DJ_TRACK -p grip_mp_cap:=$MP_CAP_D \
    -p drop_publish_mass_event:=$DROP_PUB_EVENT \
    -p geom_source:=online -p grip_payload_envelope:=$PAYLOAD_ENVELOPE_D \
    -p geom_release_mode:=${NMPC_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-event}} \
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
    -p maneuver_gate_enable:=$MANEUVER_GATE \
    -p resid_release_geom:=$RESID_RELEASE_GEOM \
    -p resid_step_enable:=$RESID_STEP \
    -p resid_step_half:=$RESID_STEP_HALF -p resid_step_thresh:=$RESID_STEP_TH_D \
    -p resid_step_persist:=$RESID_STEP_PERSIST \
    -p resid_step_release_thresh:=$RESID_STEP_REL_D \
    -p maneuver_omega_thresh:=$MG_OMEGA_D -p maneuver_vel_thresh:=$MG_VEL_D \
    -p maneuver_q0_cap:=$MG_CAP_D -p maneuver_exponent:=$MG_EXP_D \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p event_trigger_enable:=true -p schedule_theta:='$THETA' \
    -p confirm_thresh_alpha:=$CALPHA \
    -p grip_payload_envelope:=$PAYLOAD_ENVELOPE_D -p c_xy_est_enable:=true \
    -p c_xy_from_moment:=$C_XY_FROM_MOMENT \
    -p eval_true_payload_mass:=$PAYLOAD_KG_D \
    -p geom_release_mode:=${MHE_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-event}} \
    -p event_signal_mode:=${MHE_SIGNAL_MODE:-external}" \
  > "$MHE_LOG" 2>&1 &

echo "All components up. Gazebo GUI = 物理飞行; RViz = 控制端跟踪。"
echo "  NMPC: tail -f $NODE_LOG   MHE: tail -f $MHE_LOG"
