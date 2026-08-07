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
# 高机动消融(2026-07-29 新增,默认值全部保持历史行为、老批次逐字节复现):
#   GRIP_DYN_R/W       带载 8 字的半径与角速率 (默认 0.8/0.25,峰值速度仅
#                      0.28 m/s)。包络=2r×r,峰值速度 v_peak=√2·r·w,峰值水平
#                      加速度=v_peak²/r。常用档(r=5.0 → 10m×5m 包络),括号内
#                      分别是稳态倾角和 ramp 段峰值倾角(见 GRIP_DYN_RAMP):
#                        w=0.283→2m/s(4.7° / ramp 16.4°)  RAMP=3.0
#                        w=0.424→3m/s(10.4° / ramp 14.0°) RAMP=-1 (6.25s)
#                        w=0.566→4m/s(18.1° / ramp 23.9°) RAMP=-1 (4.68s)
#                        w=0.707→5m/s(27.0° / ramp 34.6°) RAMP=-1 (4.00s)
#                      这些档位的推力/力矩需求都远在约束内(5m/s 峰值需 26N,
#                      Tmax=40.5N;yaw 力矩需 ~0.06Nm,tau_psi=0.2)。
#   GRIP_DYN_RAMP      振幅渐增时长。0=写死的 4.0s;-1=auto_ramp_time(w) 自适应;
#                      >0=显式秒数。**每档取值见上表**,别一律用 -1:
#                      auto 是个纯相对判据(把 ramp 额外速度压到轨迹特征速度的
#                      一半),w 越小要的 ramp 越长——2m/s 档它会要 9.37s,只为把
#                      起振倾角压到 6.3°,绝对意义上毫无必要。显式给 3.0s 后峰值
#                      倾角 16.4°,仍低于 4m/s 档 auto 自己的 23.9°,而每轮省 6.4s。
#                      高速档继续用 -1 兜底:w≥0.663 时 2.65/w<4.0,真正生效的是
#                      auto 里那个 base=4.0 的下限。
#                      ⚠️ 改 ramp 会平移 drop 在 8 字上的相位——除非开
#                      GRIP_DROP_AT_TIP=true(按 a=3π/2 几何锁相,与 ramp 无关)。
#                      跨档对比务必开它,否则各档 drop 相位不可比。
#   TRAJ_SCALE_WEIGHTS Bryson 权重是否跟着 (r,w) 走 (默认 false)。L_vel/L_omega
#                      在 acados_params 里按 r=1.0/w=0.3 写死,高机动下差一个
#                      数量级。⚠️开启会连带修正 L_omega(原值按圆形轨迹标的,
#                      对 8 字本就偏紧 3.2 倍),两种效应会混在一起——要分开
#                      归因就跑 on/off 两组,别混算(见 07-19 的分类教训)。
#   ⚠️ MHE 的 c_xy 在线估计在机动段本来就是冻结的(稳态门控 c_xy_steady_vel
#      =0.20 m/s / c_xy_steady_omega=0.15 rad/s,当前 w=0.25 的 yaw_rate 峰值
#      0.79 早已超阈)。提速不改变这个机制,c_xy 仍在 lift 后的悬停窗口收敛并
#      EMA 冻结——但论文里"在线估计"的表述要写准。
#
# 与 run_gripper_ecc_sweep.sh 的差异:那个是交互式单点复测(GUI+QGC,慢但直观);
# 这个是无头批量驱动(零桌面渲染负担),CEM/批量对比实验用这个。
set -e

GRIP_PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.3}"
GRIP_ECC_Y="${GRIP_ECC_Y:-0.05}"
BOX_I=$(python3 -c "print(f'{$GRIP_PAYLOAD_KG * 0.00375:.6f}')")
R_XY=$(python3 -c "print(f'{max(0.20, $GRIP_ECC_Y + 0.08):.3f}')")
USE_MHE="${USE_MHE:-true}"

# ⚠️ ROS2 参数是强类型的:`-p grip_dyn_r:=5` / `grip_drop_after_sec:=40` 会被解析
# 成 INTEGER,跟节点里 declare_parameter(..., 0.8) 的 DOUBLE 冲突,直接抛
# InvalidParameterTypeException **把节点打挂**。2026-07-29~30 连踩两次(先是
# FIG8_RAMP=-1,再是 GRIP_DROP_AFTER=40)——第一次只给新参数打了补丁,不是修根因。
# 这里把**所有**喂给 DOUBLE 参数的环境变量统一规范化成带小数点的字面量,
# 于是 GRIP_DROP_AFTER=40 / GRIP_DYN_R=5 / L1_OMEGA_C=1 这类自然写法都能用。
_f2d() { python3 -c "print(float('$1'))"; }
GRIP_DYN_R_D=$(_f2d "${GRIP_DYN_R:-0.8}")
GRIP_DYN_W_D=$(_f2d "${GRIP_DYN_W:-0.25}")
GRIP_DYN_RAMP_D=$(_f2d "${GRIP_DYN_RAMP:-0.0}")
GRIP_PAYLOAD_KG_D=$(_f2d "$GRIP_PAYLOAD_KG")
GRIP_ECC_Y_D=$(_f2d "$GRIP_ECC_Y")
PUBLISH_HZ_D=$(_f2d "${PUBLISH_HZ:-50.0}")
GRIP_GEOM_MP_PRIOR_D=$(_f2d "${GRIP_GEOM_MP_PRIOR:-$GRIP_PAYLOAD_KG}")
L1_A_GAIN_D=$(_f2d "${L1_A_GAIN:-10.0}")
L1_OMEGA_C_D=$(_f2d "${L1_OMEGA_C:-0.5}")
GRIP_DROP_AFTER_D=$(_f2d "${GRIP_DROP_AFTER:-0.0}")
ATTACH_WINDOW_SEC_D=$(_f2d "${ATTACH_WINDOW_SEC:-40.0}")
MHE_CONFIRM_THRESH_D=$(_f2d "${MHE_CONFIRM_THRESH:-1.5}")
MHE_CONFIRM_PRIOR_D=$(_f2d "${MHE_CONFIRM_PRIOR:-0.3}")
GRIP_TRUE_PAYLOAD_MASS_D=$(_f2d "${GRIP_TRUE_PAYLOAD_MASS:-0.0}")
GRIP_GEOM_MP_FLOOR_D=$(_f2d "${GRIP_GEOM_MP_FLOOR:-0.15}")

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
# mavros 日志护栏(2026-07-31 加):mavros 的 stdout 同 px4 一样是无分析价值的
# verbose 刷屏,且**没有任何分析脚本读它**(已 grep 验证:分析只读 grip_mhe/grip_nmpc)。
# 实测曾累积 1308 个文件、3.1G,单个 gviz_mavros 就 3.0G。护栏同 px4:只留前
# MAVROS_LOG_CAP 排错用,超出全吞。用 (head; cat>/dev/null) 而非 `| head`——
# head 读够就退,cat 继续吞,管道不关闭,mavros 收不到 SIGPIPE 不会被杀。
nohup ros2 launch mavros px4.launch fcu_url:=udp://:14540@ 2>&1 \
    | ( head -c "${MAVROS_LOG_CAP:-5M}" > "$RUNDIR/grip_mavros_$STAMP.log"; cat >/dev/null ) &
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
    -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$GRIP_ECC_Y_D \
    -p grip_z_low:=0.55 -p grip_z_high:=2.5 \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$GRIP_PAYLOAD_KG_D -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=1.5 -p grip_lift_dur:=3.0 \
    -p use_mhe:=$USE_MHE \
    -p decouple_publish:=${DECOUPLE_PUB:-true} -p publish_hz:=$PUBLISH_HZ_D \
    -p geom_source:=${NMPC_GEOM_SOURCE:-truth} \
    -p grip_payload_prior:=$GRIP_GEOM_MP_PRIOR_D \
    -p control_mode:=${NMPC_CONTROL_MODE:-mhe} \
    -p l1_a_gain:=$L1_A_GAIN_D \
    -p l1_omega_c:=$L1_OMEGA_C_D \
    -p grip_drop_after_sec:=$GRIP_DROP_AFTER_D \
    -p grip_dynamic_after_lift:=${GRIP_DYNAMIC:-false} \
    -p grip_dyn_r:=$GRIP_DYN_R_D -p grip_dyn_w:=$GRIP_DYN_W_D \
    -p grip_dyn_ramp:=$GRIP_DYN_RAMP_D \
    -p traj_scale_weights:=${TRAJ_SCALE_WEIGHTS:-false} \
    -p grip_drop_at_fig8_tip:=${GRIP_DROP_AT_TIP:-false} \
    -p attach_window_sec:=$ATTACH_WINDOW_SEC_D \
    > "$NODE_LOG" 2>&1 &

# 8. MHE(dJ/c_xy + 棘轮修复已内建在 mhe_node.py,attach_offset 一到就自动生效)
# GRIP_TRUE_PAYLOAD_MASS(2026-07-08 诊断用,默认 0=关闭):>0 时 mhe_node 的
# dJ/c_xy 直接用这个真值算,不走 self.m_est 反推的棘轮——排查 0.15kg CEM
# 结构性失败是否是"用质量反推几何缩放"这个自举耦合导致的。诊断专用,不是
# 正式 CEM 花名册的默认路径。
MHE_LOG="$RUNDIR/grip_mhe_$STAMP.log"
# ⚠️ residual_log_dir 只在非空时才传(2026-08-03 修):rclpy 的 --ros-args 解析
# 不接受空的参数值,`-p residual_log_dir:=` 会让 mhe_node 启动即崩
# (RCLError: Couldn't parse parameter override rule),而崩在自己的 nohup 日志里,
# 批次脚本照跑不误——整轮**静默没有 MHE**。07-31 加残差采集时引入,当时所有批次
# 都经 run_residual_collect.sh 带着 RESID_LOG_DIR 进来,所以一直没暴露;任何不设
# 该变量的裸跑都会中招。
RESID_ARG=()
if [ -n "${RESID_LOG_DIR:-}" ]; then
  RESID_ARG=(-p "residual_log_dir:=$RESID_LOG_DIR")
fi
nohup ros2 run offboard_test_acados mhe_node --ros-args \
    "${RESID_ARG[@]}" \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p event_signal_mode:=${MHE_SIGNAL_MODE:-external} \
    -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
    -p schedule_theta:="${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}" \
    -p event_confirm_thresh_n:=$MHE_CONFIRM_THRESH_D \
    -p confirm_thresh_alpha:=${MHE_CONFIRM_ALPHA:--1.0} \
    -p confirm_payload_prior:=$MHE_CONFIRM_PRIOR_D \
    -p grip_true_payload_mass:=$GRIP_TRUE_PAYLOAD_MASS_D \
    -p grip_geom_mp_floor:=$GRIP_GEOM_MP_FLOOR_D \
    -p grip_geom_mp_prior:=$GRIP_GEOM_MP_PRIOR_D \
    -p c_xy_est_enable:=${MHE_C_XY_EST:-false} \
    > "$MHE_LOG" 2>&1 &

echo "gripper headless stack up: nmpc=$NODE_LOG mhe=$MHE_LOG"
echo "  payload=${GRIP_PAYLOAD_KG}kg ecc_y=${GRIP_ECC_Y}m r_xy=$R_XY"
echo "  event=${MHE_EVENT_TRIGGER:-true} theta=${MHE_SCHEDULE_THETA:-M0} confirm_thresh=${MHE_CONFIRM_THRESH:-1.5}"
echo "  true_payload_mass=${GRIP_TRUE_PAYLOAD_MASS:-0.0} geom_mp_floor=${GRIP_GEOM_MP_FLOOR:-0.15}kg"
