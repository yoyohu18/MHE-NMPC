#!/bin/bash
# 吊挂夹爪场景的全无头启动器(2026-07-07):不开任何
# gnome-terminal/QGC,GCS 心跳用 gcs_heartbeat.py 顶替,
# 但载具换成 gripper/attach(空机 gz_x500 + magnetic_gripper + DetachableJoint
# 真刚体),不是 wrench(mass_changer)。这是给吊挂 CEM 学习(长期计划 §阶段B
# 步骤2)准备的批量驱动底座——今天(坏几何复测)已经验证过 dJ/c_xy + 棘轮修复
# 在物理可行偏心范围内(~0.05-0.12m)稳定可靠,可以放心当训练场用。
#
# 对外接口统一使用环境变量(不是位置参数),方便批量驱动脚本用
# subprocess.Popen(env=...) 调用:
#   GRIP_PAYLOAD_KG    实际载荷质量 kg (默认 0.15；安全包线仍为 0.3)
#   GRIP_ECC_Y         横向偏心 m,即 grip_y (默认 0.05)——box 固定在世界系
#                      (1.0,0.0),grip_y 是悬停目标,两者之差就是 attach 偏心
#                      (扫描入口见 run_cxy_ecc_sweep.sh)
#   MHE_EVENT_TRIGGER  事件触发开关 (默认 true)
#   MHE_SCHEDULE_THETA 权重时间表 5 维 (默认 M0 规则版)
#   MHE_CONFIRM_ALPHA  确认阈值系数 (★2026-09-04 默认 -1.0 -> 1.5,即
#                      thresh = α·g·包线 = 4.4N @0.3kg。见下面该变量处的说明)
#   MHE_CONFIRM_THRESH 固定确认阈值 [N] (默认 1.5,**仅在 MHE_CONFIRM_ALPHA<0
#                      时才生效**;该值原是给 wrench 的 gz CLI 冷启动延迟调的,
#                      对 gripper 从未标定过,不要再当默认路径用)
#
# 默认事件链与 run_sitl_gripper_viz.sh 的无信号主线一致:
#   DROP_PUBLISH_MASS_EVENT=false  NMPC_GEOM_RELEASE_MODE=event
#   MHE_SIGNAL_MODE=residual       MHE_GEOM_RELEASE_MODE=self
#   MHE_CXY_MASS_RELEASE_MP=0.03
# 需要复现旧的外部事件档时仍可通过同名环境变量显式覆盖。
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
# GUI 可视化请用 run_sitl_gripper_viz.sh；批量对比实验用本脚本。
set -e

GRIP_PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.15}"
GRIP_ECC_Y="${GRIP_ECC_Y:-0.05}"
BOX_I=$(python3 -c "print(f'{$GRIP_PAYLOAD_KG * 0.00375:.6f}')")
# 抓取偏心上限 [m] = 设计偏心 ECC_Y + 容差。**不是**"够得着"的判据,而是
# **配平力矩余量**的判据:带偏心悬停要常值输出 tau_roll = (m_P/m_T)*r_y*T
# ≈ 2.944*r_y N·m,而 NMPC 的 roll 约束是 ±0.5 —— r_y=0.17 时配平吃掉 99%,
# figure8 没有机动余量,实测直接姿态发散坠机(gviz_20260831_165224)。
# 408 轮历史回归:|r_y|>=0.15 坠机 38.5%、0.12~0.15 为 15.5%、<0.12 为 9.0%。
# 旧公式 max(0.20, ECC_Y+0.08) 给到 0.20,等于**不设防**(266 次 attach 只挡掉 1 次)。
# 容差 0.03 → ECC_Y=0.10 时上限 0.13,历史拒绝率 15.4%。⚠️ 容差不能给 0:
# 实际 d_xy 中位 0.096、p90 0.139,本来就在设计值附近抖(跟踪误差 + 旋翼下洗
# 吹动 box)。超限只是**这一 tick 不 attach**,drone 继续跟踪悬停点等它收敛;
# proximity 侧有节流日志 + 8s 未 attach 的 WARN。
GRIP_ATTACH_TOL="${GRIP_ATTACH_TOL:-0.03}"
R_XY=$(python3 -c "print(f'{$GRIP_ECC_Y + $GRIP_ATTACH_TOL:.3f}')")
USE_MHE="${USE_MHE:-true}"

# ⚠️ ROS2 参数是强类型的:`-p grip_dyn_r:=5` / `grip_drop_after_sec:=40` 会被解析
# 成 INTEGER,跟节点里 declare_parameter(..., 0.8) 的 DOUBLE 冲突,直接抛
# InvalidParameterTypeException **把节点打挂**。2026-07-29~30 连踩两次(先是
# FIG8_RAMP=-1,再是 GRIP_DROP_AFTER=40)——第一次只给新参数打了补丁,不是修根因。
# 这里把**所有**喂给 DOUBLE 参数的环境变量统一规范化成带小数点的字面量,
# 于是 GRIP_DROP_AFTER=40 / GRIP_DYN_R=5 / L1_OMEGA_C=1 这类自然写法都能用。
_f2d() { python3 -c "print(float('$1'))"; }
# ⚠️ bool 也是强类型的:`-p foo:=1` 会被 rclpy 当 INTEGER 解析,而声明成 BOOL 的
# 参数会抛 InvalidParameterTypeException —— **节点启动即崩**,崩在自己的 nohup
# 日志里,脚本照样打印 "stack up",整轮静默没有该节点(2026-08-25 实测踩到:
# GRIP_LIFT_HOLD=1 让 nmpc 和 mhe 双双起不来)。这与 residual_log_dir 传空值
# 是同一类坑。_b() 把 1/0/yes/no/on/off 一律规范成 true/false。
_b() { case "$(echo "${1:-}" | tr 'A-Z' 'a-z')" in
         1|true|yes|on|y) echo true ;; *) echo false ;; esac; }
GRIP_DYN_R_D=$(_f2d "${GRIP_DYN_R:-0.8}")
GRIP_DYN_W_D=$(_f2d "${GRIP_DYN_W:-0.25}")
GRIP_DYN_RAMP_D=$(_f2d "${GRIP_DYN_RAMP:-0.0}")
# 立体 8 字的 z 振幅(2026-08-28 补透传)。节点默认 0.0 = 平面 8 字,本脚本沿用
# 该默认 => 不设此变量的历史批次逐位不变。viz 脚本默认 0.8,跨脚本对比同一
# 工作点时必须显式对齐(z 实际范围 = GRIP_Z_HIGH ± GRIP_DYN_DZ,留够离地余量)。
GRIP_DYN_DZ_D=$(_f2d "${GRIP_DYN_DZ:-0.0}")
GRIP_PAYLOAD_KG_D=$(_f2d "$GRIP_PAYLOAD_KG")
GRIP_ECC_Y_D=$(_f2d "$GRIP_ECC_Y")
PUBLISH_HZ_D=$(_f2d "${PUBLISH_HZ:-50.0}")
# --- 载荷质量信息:唯一入口 = 包线上界(2026-08-26 去先验改造)---
# 本项目前提:**不能知道载荷质量**,否则在线质量估计失去意义。此前这里有六个
# 变量把载荷质量喂进两个节点,其中四个默认值直接回落到 $GRIP_PAYLOAD_KG
# (= box 的**真值**),等于默认让系统知道"这次抓的盒子正好多重":
#   GRIP_NMPC_MP_PRIOR / GRIP_GEOM_MP_PRIOR → grip_payload_prior (NMPC 模型侧 dJ)
#   GRIP_MHE_MP_PRIOR                       → grip_geom_mp_prior (MHE 几何)
#   GRIP_GAIN_PRIOR                         → grip_gain_prior    (内环增益整定)
#   MHE_CONFIRM_PRIOR                       → confirm_payload_prior (事件确认阈值)
#   GRIP_GEOM_MP_FLOOR                      → grip_geom_mp_floor (0.15kg 永久地板)
#   GRIP_TRUE_PAYLOAD_MASS                  → grip_true_payload_mass (真值直灌)
# 六个参数已从两个节点里全部删除,这些变量随之作废。
#
# 留下的这一个是**机架规格**("这架飞机最多吊得动多少"),写在整定表里、与飞
# 哪一趟无关 —— 拿它做几何标度和内环整定不算"知道载荷质量"。
# ⚠️ m_p>=0.2937kg 时 ratio 撞 cap 5.0,故 0.3kg 主线工况下包线与旧点估计先验
#    给出**逐位相同**的 MC_*RATE_K;轻载(0.15/0.2)不在 cap 里,是真的改了整定。
GRIP_PAYLOAD_ENVELOPE_D=$(_f2d "${GRIP_PAYLOAD_ENVELOPE:-0.3}")
# MHE 侧几何标度的**独立覆盖**(2026-08-28)。默认回落到 GRIP_PAYLOAD_ENVELOPE,
# 逐位兼容。拆开的理由:08-28 排查 m_est 带载低估时发现,同一个变量同时喂 NMPC
# (模型 dJ 初值 + PX4 增益)和 MHE(转动通路几何标度),扫它无法区分是哪一侧起
# 作用。要检验"MHE 几何过估经 quat 间接压低质量"必须只动 MHE 这一侧。
MHE_PAYLOAD_ENVELOPE_D=$(_f2d "${MHE_PAYLOAD_ENVELOPE:-${GRIP_PAYLOAD_ENVELOPE:-0.3}}")
EVAL_TRUE_PAYLOAD_D=$(_f2d "${EVAL_TRUE_PAYLOAD_MASS:-$GRIP_PAYLOAD_KG}")
L1_A_GAIN_D=$(_f2d "${L1_A_GAIN:-10.0}")
L1_OMEGA_C_D=$(_f2d "${L1_OMEGA_C:-0.5}")
GRIP_DROP_AFTER_D=$(_f2d "${GRIP_DROP_AFTER:-0.0}")
ATTACH_WINDOW_SEC_D=$(_f2d "${ATTACH_WINDOW_SEC:-40.0}")
MHE_CONFIRM_THRESH_D=$(_f2d "${MHE_CONFIRM_THRESH:-1.5}")
# 残差自触发的确认阈值。confirm_thresh_alpha>=0 时生效的是 α·g·包线(不含任务
# 信息,包线是机架规格);<0 才退回上面那个固定的 event_confirm_thresh_n。
# ★ 2026-09-04 默认 -1.0(固定 1.5N) -> 1.5(= α·g·0.3 = 4.4N),与
#   run_sitl_gripper_viz.sh 的 M0 分支对齐。为什么:0.15kg 载荷的卸载台阶只有
#   m_P·g=1.47N,固定 1.5N 这个阈值**恰好卡在台阶量级上**,drop 于是由残差路检出
#   —— 那既不是无信号主线要验证的机制,也把"残差信号在小载荷工况下不可用"这个
#   结论绕了过去。4.4N 把残差路堵死,drop 只能由质量域判据 / |s| 塌陷判据检出。
# ⚠️ 阈值随包线联动:GRIP_PAYLOAD_ENVELOPE=0.5 时变 7.36N。
# 回退(不推荐,仅为复现 09-04 前的旧批次):MHE_CONFIRM_ALPHA=-1
MHE_CONFIRM_ALPHA_D="${MHE_CONFIRM_ALPHA:-1.5}"
# 转动 lumped 扰动通道 xi(2026-08-25)。默认关 → model.p 的 xi 槽恒零,逐位兼容。
XI_MAX_D=$(_f2d "${XI_MAX:-40.0}")
XI_OMEGA_C_D=$(_f2d "${XI_OMEGA_C:-0.5}")
# ω_cmd 缩放(2026-08-25):增益调度的等效实现,走 setpoint 侧不碰 PX4 参数。
# 开启时**自动关闭** scale_px4_rate_gains(两者补同一件事,同开=双重补偿)。
OMEGA_SCALE_TAU_D=$(_f2d "${OMEGA_SCALE_TAU:-0.5}")
OMEGA_SCALE_CAP_D=$(_f2d "${OMEGA_SCALE_CAP:-5.0}")
# 抬升目标高度与抬升时长(2026-08-24 参数化,默认值 = 历史写死值,逐位兼容)。
# ⚠️ 必须一起调:LIFT 段在 GRIP_LIFT_DUR 秒内从 grip_z_low(0.55) 抬到 Z_HIGH,
# 抬升速率 = (Z_HIGH-0.55)/LIFT_DUR。08-19 定案的新 4m/s 工作点 r=10 在 z=2.5
# **必发散坠毁**、z=6 与 z=10 全稳(大尺度+低空有未知稳定性边界,机制未明),
# 所以 r=10 必须配 GRIP_Z_HIGH=6 GRIP_LIFT_DUR=8.4(速率与 2.5/3.0 同量级)。
GRIP_Z_HIGH_D=$(_f2d "${GRIP_Z_HIGH:-2.5}")
GRIP_LIFT_DUR_D=$(_f2d "${GRIP_LIFT_DUR:-3.0}")
# attach 命令到 LIFT 之间的等待(2026-09-04 补透传 + 默认 1.5 -> 3.0,与
# run_sitl_gripper_viz.sh 对齐)。原先这里是写死的 1.5。
# ★ 为什么必须改:连续主线下 NMPC 在发 enable 的同一帧就按**包线**把 J 抬到
#   0.0579(内环 ω 缩放到 cap 5.0),而吸附**没有回执**——box 没吸上时,这 5× 就
#   加在接近空机的构型上 = 07-14 那个过增益振荡崩法。09-04 viz 三轮实测坐实:
#   1.5 那轮 LIFT 段直接发散(822 次 solve failed、tilt 141°、pos_err 70m、
#   om_scale 钉在 5.0、MHE health=0 持续 21s),3.0 的两轮干净通过。
# 只影响抓取等待:lift_after 在 t_drop 与 dyn_t0 里自动抵消,figure8 相位、
# DROP_AFTER、任何进论文的量都不受影响,整条时间线只是平移。
GRIP_LIFT_AFTER_D=$(_f2d "${GRIP_LIFT_AFTER:-3.0}")
# LIFT 中途 hold + dJ 跟随 m_est(2026-08-25,默认全关)。动机见 acados_nmpc_node
# 的 grip_lift_hold_enable 注释:attach 后 box 还在地上时 MHE 学不到任何东西
# (实测 m_est 反而从 2.064 下漂到 2.023),真正该给的时间在**离地之后**。
# GRIP_LIFT_HOLD=1   抬升 GRIP_LIFT_HOLD_DZ 米后把斜坡冻结 GRIP_LIFT_HOLD_SEC 秒
# DJ_TRACK_MEST=1    让模型侧 dJ 跟着 m_est 棘轮走(只增不减 + 上界 cap + 下界地板)
# GRIP_DJ_FLOOR_MP=  模型 dJ 的下界载荷质量 kg(★ 2026-09-02 默认 0.15 → 0.05)。
#                    attach 时 dJ 从 0 起步,地板保证它不会掉到空机惯量附近
#                    (dJ→0 会力矩饱和级联发散)。设 0 = 无地板。改小的理由:0.15 和
#                    最小实验载荷同量级,会把 0.15kg 工况的 m_p_model 整个钉死在
#                    地板上,估计器等于没跑;0.05 仍覆盖 attach→收敛那 ~0.6s。
# DJ_RATCHET=        模型 dJ 走棘轮(只增不减)还是双向跟随 m_est。★ 2026-09-02
#                    默认 true → false:run_dj_ratchet_ab.sh 的 n=7 配对 A/B 判
#                    棘轮在带载段是不受控的正偏差补偿器(m_est 每次向上过冲都被
#                    永久记住),关掉后 dJ 稳态误差 +26.9% → +5.5%,7/7 同向
#                    p=.0156。代价 = dJ 跟着 m_est 抖(小载荷差分放大 ~7.9×)。
#                    必须与地板 0.05 配套:棘轮关掉后地板是唯一下界,留 0.15 等于
#                    换个东西继续钉死小载荷。增益侧不受影响,永远用棘轮峰值。
# ⚠️ DJ_TRACK_MEST 只在 geom_source=online 下生效(_update_online_geometry 是
#    唯一的刷新点);legacy 几何档不走那条路。
# ⚠️ PX4 内环增益**不跟着棘轮下调**:它在 attach 时按包线整定一次,只有棘轮涨过
#    包线才上调(而默认 cap=0.6 < 门槛 0.625,所以默认配置下它永不改动)。
# ── geom_source 默认值:2026-08-28 由 truth 改为 online(勿删,改回前先读)──────
# NMPC 的载荷几何(dJ/c_xy)从哪来。truth = 由 attach 真值几何 + grip_payload_mass
# **载荷真值质量**算(acados_nmpc_node._payload_geometry);online = attach 瞬间用
# grip_payload_envelope 机架规格包线初始化 dJ/内环增益,c_xy 随后吃 τ_phys 反算的
# /acados_nmpc/c_xy_est 在线精修。
#  · 为什么改:truth 读载荷真值,违背本项目前提"不能知道包裹质量",是通往 NMPC 的
#    四条路里最后一条脏的(MHE 侧 08-26 已用包线洗干净)。且默认值早与实践脱节——
#    14 个正式批次脚本都显式传 online 覆盖它,run_sitl_gripper_viz.sh 也写死
#    online;truth 只在"忘了传"时生效 = 实验静默拿到真值的陷阱。
#  · 证据:4m/s 首次 truth vs online 配对 A/B(nmpc_test_results/
#    geomsrc_ab_4ms_20260828_184240.txt):truth 3/8 失败 vs online 1/8
#    (Fisher p=0.569,不显著);bias 非劣成立(n=5 配对,d=+0.025pp,95%CI 上界
#    +0.788 < δ=1.0pp)。truth 在 4m/s 上**历史零数据**,是未经验证的配置;
#    online 已累计 57 轮。
#  · ⚠️ 诚实记录:该批**前置硬条件①(16/16 全 ok)未通过**(4 轮失败),本次改默认
#    是在知悉①未通过的情况下由用户拍板,**不是判据放行**。支持理由是 ①未过的
#    主因为工作点自身脆弱(两臂共有:yaw 力矩饱和 tau_psi=0.2Nm + LIFT 段 m_est
#    触 1.961 下界),非 geom_source;online 唯一那次失败(rep8)已诊断为 yaw 边界
#    所致、与被测变量无关。
#  · 回归风险:显式传 NMPC_GEOM_SOURCE 的脚本全不受影响,仅改变裸跑行为。
GRIP_LIFT_HOLD_DZ_D=$(_f2d "${GRIP_LIFT_HOLD_DZ:-0.35}")
GRIP_LIFT_HOLD_SEC_D=$(_f2d "${GRIP_LIFT_HOLD_SEC:-3.0}")
GRIP_MP_CAP_D=$(_f2d "${GRIP_MP_CAP:-0.6}")
GRIP_DJ_FLOOR_MP_D=$(_f2d "${GRIP_DJ_FLOOR_MP:-0.05}")

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
    -p grip_z_low:=0.55 -p grip_z_high:=$GRIP_Z_HIGH_D \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$GRIP_PAYLOAD_KG_D -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=$GRIP_LIFT_AFTER_D -p grip_lift_dur:=$GRIP_LIFT_DUR_D \
    -p grip_lift_hold_enable:=$(_b "${GRIP_LIFT_HOLD:-false}") \
    -p grip_lift_hold_dz:=$GRIP_LIFT_HOLD_DZ_D \
    -p grip_lift_hold_sec:=$GRIP_LIFT_HOLD_SEC_D \
    -p dj_track_mest:=$(_b "${DJ_TRACK_MEST:-true}") \
    -p drop_publish_mass_event:=$(_b "${DROP_PUBLISH_MASS_EVENT:-false}") \
    -p grip_mp_cap:=$GRIP_MP_CAP_D \
    -p grip_dj_floor_mp:=$GRIP_DJ_FLOOR_MP_D \
    -p dj_ratchet_enable:=$(_b "${DJ_RATCHET:-false}") \
    -p use_mhe:=$USE_MHE \
    -p continuous_payload_estimates:=${CONTINUOUS_PAYLOAD_ESTIMATES:-true} \
    -p decouple_publish:=${DECOUPLE_PUB:-true} -p publish_hz:=$PUBLISH_HZ_D \
    -p geom_source:=${NMPC_GEOM_SOURCE:-estimate} \
    -p geom_release_mode:=${NMPC_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-event}} \
    -p grip_payload_envelope:=$GRIP_PAYLOAD_ENVELOPE_D \
    -p attach_j_bootstrap_enable:=${ATTACH_J_BOOTSTRAP:-true} \
    -p control_mode:=${NMPC_CONTROL_MODE:-mhe} \
    -p l1_a_gain:=$L1_A_GAIN_D \
    -p l1_omega_c:=$L1_OMEGA_C_D \
    -p grip_drop_after_sec:=$GRIP_DROP_AFTER_D \
    -p grip_dynamic_after_lift:=${GRIP_DYNAMIC:-false} \
    -p grip_dyn_r:=$GRIP_DYN_R_D -p grip_dyn_w:=$GRIP_DYN_W_D \
    -p grip_dyn_ramp:=$GRIP_DYN_RAMP_D \
    -p grip_dyn_dz:=$GRIP_DYN_DZ_D \
    -p traj_scale_weights:=${TRAJ_SCALE_WEIGHTS:-false} \
    -p grip_drop_at_fig8_tip:=${GRIP_DROP_AT_TIP:-false} \
    -p attach_window_sec:=$ATTACH_WINDOW_SEC_D \
    -p tau_lumped_enable:=${NMPC_TAU_LUMPED:-false} \
    -p xi_max:=$XI_MAX_D -p xi_omega_c:=$XI_OMEGA_C_D \
    -p omega_scale_enable:=${NMPC_OMEGA_SCALE:-true} \
    -p omega_scale_source:=${OMEGA_SCALE_SRC:-djest} \
    -p omega_scale_tau:=$OMEGA_SCALE_TAU_D -p omega_scale_cap:=$OMEGA_SCALE_CAP_D \
    > "$NODE_LOG" 2>&1 &

# 8. MHE(dJ/c_xy + 棘轮修复已内建在 mhe_node.py,attach_offset 一到就自动生效)
# 载荷几何标度 = grip_payload_envelope(包线上界,见文件顶部)。2026-08-26 去先验
# 改造前这里还有 GRIP_TRUE_PAYLOAD_MASS(真值直灌)/GRIP_GEOM_MP_FLOOR(0.15kg 永久
# 地板)/GRIP_MHE_MP_PRIOR(操作先验)三条通路,全部已删。
# --- MHE<->NMPC 力矩通道解耦(2026-08-25,默认 command 不变)---
# MHE_TAU_SOURCE=phys_full:u_known 三轴力矩**全部**用电机转速反算,MHE 只吃
#   rate control + allocation 之后真实发生的量,与 NMPC 彻底解耦。
#   (phys 档只换 roll/pitch,yaw 仍是 NMPC 意图值 —— 实测那个意图值比执行器
#    实际输出大 3 倍,20 批残差 CSV 对表:roll 斜率 1.08/pitch 1.10/yaw 0.32。)
# MHE_MOTOR_AVG=1:把取自电机的分量换成 MHE 采样区间内的**平均值**。
#   motor_speed 是 250Hz、MHE 才 10Hz,取端点瞬时值当 0.1s 常量 = 混叠。
# ⚠️⚠️ 2026-08-24 血泪:coupled+self+phys 在 attach/LIFT 段就发散坠机,而
#   coupled+event+command 是已验证可用配置。当时留下的待验假说正是"NMPC 意图
#   力矩虽不准但 20Hz 分段常值、与射击区间匹配;tau_phys 准但混叠"。所以
#   **phys_full 必须配 MHE_MOTOR_AVG=1 一起测**,单开 phys_full 大概率复现那次
#   坠机(它只是把 yaw 也换成瞬时值,混叠更重)。两者都默认关,需显式 opt-in。
# --- 冷启动/"从头估计"(2026-08-25,默认全关)---
# MHE_PRE_OFFBOARD=1:NMPC 接管**之前**(posctl 悬停段)就开始估质量,收敛后才
#   往 /acados_nmpc/mhe_mass_estimate 发 —— NMPC 侧零改动(mhe_mass_cb 本来就
#   没有"是否已接管"的门控,收到就用)。
# MHE_NO_MASS_PRIOR=1:卸掉空机质量先验。注意通往 MHE 的空机质量先验有**三条**
#   路,这个开关一次卸两条(Q0[13] 0.1->1e-6、lbx 下界 0.95*m_B->0.2),第三条
#   (x0_bar 种子)降级为纯初始猜测。只改种子不动 lbx 是**假的零先验**——1.961
#   那个硬下界把答案圈在真值下方 5% 处。
# ⚠️⚠️ MHE_NO_MASS_PRIOR 会改 lbx 和 W_0 => **acados 必然重新生成+编译 solver**
#   (首轮启动慢约 30s),而且它和默认档共用同一个 codegen 目录 —— 跑完冷启动档
#   再跑默认档批次,那一轮也会重新生成一次。跟 MHE_M_MIN 是同一个坑,批次混跑
#   时把重生成的耗时算进超时预算,别当成"脚本卡住了"。
# 单测:src/offboard_test_acados/test/test_mhe_cold_start_standalone.py
#   MHE_NO_MASS_PRIOR=1 跑可验证"起点不影响最终解"(实测两个起点末值差 0.0000kg)。
MHE_LOG="$RUNDIR/grip_mhe_$STAMP.log"
# ⚠️ residual_log_dir 只在非空时才传(2026-08-03 修):rclpy 的 --ros-args 解析
# 不接受空的参数值,`-p residual_log_dir:=` 会让 mhe_node 启动即崩
# (RCLError: Couldn't parse parameter override rule),而崩在自己的 nohup 日志里,
# 批次脚本照跑不误——整轮**静默没有 MHE**。07-31 加残差采集时引入,当时所有批次
# 都经 run_residual_collect.sh 带着 RESID_LOG_DIR 进来,所以一直没暴露;任何不设
# 该变量的裸跑都会中招。
# ===== 几何-质量耦合档(2026-08-31 起默认开)=====
# MHE 的几何槽从 [dJ,cx,cy](窗外算好的常参数,∂(J,c)/∂m≡0)换成可测杆臂
# [rx,ry,rz],J(m)/c(m) 由被估质量在模型内现算 —— 质量因此能从**转动通路**辨识。
# 证据(记忆 geom-coupled-ab-n8,两批 n=8 配对 A/B,唯一差异就是本变量):
#   figure8 v_peak=2.0m/s:|err| p50 2.31%→0.53%,偏差中位 −2.57%→+0.45%,
#                          配对 8/8 同向 p=0.0078;
#   悬停带载            :0.58%→0.21%,−0.57%→+0.01%,7/7 同向 p=0.0156。
#   ★ legacy 两批合计 16/16 轮**全是负偏差**,coupled 围绕零 —— 这坐实了
#     "m_est 带载低估"的根源是 legacy 缺 ∂(J,c)/∂m 这条通路(记忆
#     mest-maneuver-underestimate 的 H2' 假说)。
# 代价:MHE solve 中位 4.20→5.95ms(10Hz 下占 6% 周期);首轮会重新 codegen
#       (MODEL_NAME 带 _coupled 后缀,与 legacy 目录隔离,不互相覆盖)。
# ⚠️ 只改**这一侧**:NMPC_GEOM_COUPLED 保持关(可用边界 r_y≲0.05m,而本场景
#    ecc=0.10 远在禁区外);推荐组合是 MHE coupled + NMPC 几何走 online。
# ⚠️ 未覆盖:4m/s 工作点。一阶质量矩见下面的 MHE_ESTIMATE_MOMENT 段。
# 回退:MHE_GEOM_COUPLED=0
export MHE_GEOM_COUPLED="${MHE_GEOM_COUPLED:-1}"
# ===== 一阶质量矩增广 2b(2026-09-02,无事件主线默认开)=====
# MHE_ESTIMATE_MOMENT=1:把 s=m_P·r_xy [kg·m] 增广成被估状态。载荷的"在不在/
#   偏多少"由 s 独立承担,c_xy=s/m_T 里 m_P 恰好约掉 —— 解掉 self 释放档下
#   "几何只能靠压低 m̂ 来熄灭"这个耦合(那正是 m̂ 撞 m_min 硬下界的来源)。
#   需配 MHE_C_XY_FROM_MOMENT=true,否则对外发布的 c_xy 仍走窗外 EMA(机动中不更新)。
# MHE_MOMENT_A_MODE=frozen:每个窗口内用上一拍 m_est 冻结 A=μ·r_z²，切断
#   优化器把 m 当作瞬时“惯量旋钮”的通路；窗口之间 A 仍随估计连续更新。
# 离线 attach/drop 回放:coupled 档 drop 后质量偏高 +53%；frozen 回到空机
#   −0.34%，s/dJ 同步衰减、无质量下界触碰、solve fails=0。
# ⚠️ 尚无 SITL 闭环证据；默认启用是为了让主线接口结构正确，闭环放飞仍需 smoke。
# MHE_MP_ENVELOPE 要与 GRIP_PAYLOAD_ENVELOPE 对齐(同一条包线,两侧各读各的)。
export MHE_ESTIMATE_MOMENT="${MHE_ESTIMATE_MOMENT:-1}"
export MHE_MOMENT_A_MODE="${MHE_MOMENT_A_MODE:-frozen}"
export MHE_MP_ENVELOPE="${MHE_MP_ENVELOPE:-${GRIP_PAYLOAD_ENVELOPE:-0.3}}"
# s 的到达代价 σ_s [kg·m](只在 moment 档有意义)。默认 0.1 实质无先验。
export MHE_SIGMA_S0="${MHE_SIGMA_S0:-0.1}"
# --- 机动门控(2026-08-25,默认关)---
# MHE_MANEUVER_GATE=1:机动期(|ω|或|v_xy| 超阈)按 lvl^exp 锚紧 Q0 的质量维,
#   悬停期用名义权重。动机:m_est 静态准 -0.17% 但机动中系统性低估 8%;机动期
#   的残差混着未建模气动/转子角加速度/内环滞后,都被归因到质量。降权同时也切细
#   了自举耦合 m_est→dJ/c→NMPC→飞行→残差→m_est 在机动期的增益。
# ⚠️ 与事件降权互斥(过渡期自动让位),避免两套 stage-0 权重互相覆盖。
MHE_MG_OMEGA_D=$(_f2d "${MHE_MG_OMEGA:-0.15}")
MHE_MG_VEL_D=$(_f2d "${MHE_MG_VEL:-0.20}")
MHE_MG_CAP_D=$(_f2d "${MHE_MG_CAP:-10000.0}")
MHE_MG_EXP_D=$(_f2d "${MHE_MG_EXP:-2.0}")

RESID_ARG=()
if [ -n "${RESID_LOG_DIR:-}" ]; then
  RESID_ARG=(-p "residual_log_dir:=$RESID_LOG_DIR")
fi
nohup ros2 run offboard_test_acados mhe_node --ros-args \
    "${RESID_ARG[@]}" \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p external_event_inputs:=$(_b "${MHE_EXTERNAL_EVENTS:-false}") \
    -p event_signal_mode:=${MHE_SIGNAL_MODE:-residual} \
    -p event_trigger_enable:=${MHE_EVENT_TRIGGER:-true} \
    -p schedule_theta:="${MHE_SCHEDULE_THETA:-[-4.0,0.0,0.0,0.0]}" \
    -p event_confirm_thresh_n:=$MHE_CONFIRM_THRESH_D \
    -p confirm_thresh_alpha:=$MHE_CONFIRM_ALPHA_D \
    -p grip_payload_envelope:=$MHE_PAYLOAD_ENVELOPE_D \
    -p eval_true_payload_mass:=$EVAL_TRUE_PAYLOAD_D \
    -p geom_release_mode:=${MHE_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-self}} \
    -p c_xy_mass_release_mp:=${MHE_CXY_MASS_RELEASE_MP:-0.03} \
    -p c_xy_mass_release_persist:=${MHE_CXY_MASS_RELEASE_PERSIST:-20} \
    -p c_xy_mass_arm_ratio:=${MHE_CXY_MASS_ARM_RATIO:-3.0} \
    -p c_xy_mass_arm_persist:=${MHE_CXY_MASS_ARM_PERSIST:-20} \
    -p c_xy_est_enable:=${MHE_C_XY_EST:-false} \
    -p c_xy_from_moment:=$(_b "${MHE_C_XY_FROM_MOMENT:-true}") \
    -p maneuver_gate_enable:=$(_b "${MHE_MANEUVER_GATE:-false}") \
    -p resid_release_geom:=$(_b "${MHE_RESID_RELEASE_GEOM:-true}") \
    -p resid_step_enable:=$(_b "${MHE_RESID_STEP:-false}") \
    -p resid_step_half:=${MHE_RESID_STEP_HALF:-3} \
    -p resid_step_thresh:=$(_f2d "${MHE_RESID_STEP_TH:-1.0}") \
    -p resid_step_persist:=${MHE_RESID_STEP_PERSIST:-2} \
    -p resid_step_release_thresh:=$(_f2d "${MHE_RESID_STEP_REL:-2.0}") \
    -p payload_lost_watch_enable:=$(_b "${MHE_PAYLOAD_LOST_WATCH:-false}") \
    -p payload_lost_step_thresh:=$(_f2d "${MHE_PL_STEP_TH:-2.0}") \
    -p payload_lost_step_half:=${MHE_PL_STEP_HALF:-3} \
    -p payload_lost_step_persist:=${MHE_PL_STEP_PERSIST:-2} \
    -p payload_lost_mass_margin:=$(_f2d "${MHE_PL_MASS_MARGIN:-0.10}") \
    -p payload_lost_hold_sec:=$(_f2d "${MHE_PL_HOLD_SEC:-3.0}") \
    -p payload_lost_vz_gate:=$(_f2d "${MHE_PL_VZ_GATE:-0.30}") \
    -p maneuver_omega_thresh:=$MHE_MG_OMEGA_D \
    -p maneuver_vel_thresh:=$MHE_MG_VEL_D \
    -p maneuver_q0_cap:=$MHE_MG_CAP_D \
    -p maneuver_exponent:=$MHE_MG_EXP_D \
    -p mhe_tau_source:=${MHE_TAU_SOURCE:-command} \
    -p motor_window_avg:=$(_b "${MHE_MOTOR_AVG:-false}") \
    -p pre_offboard_estimate:=$(_b "${MHE_PRE_OFFBOARD:-false}") \
    -p pre_offboard_min_z:=${MHE_PRE_OFF_MIN_Z_D:-0.5} \
    -p pre_offboard_conv_std:=${MHE_PRE_OFF_CONV_STD_D:-0.02} \
    -p pre_offboard_timeout_sec:=${MHE_PRE_OFF_TIMEOUT_D:-30.0} \
    > "$MHE_LOG" 2>&1 &

echo "gripper headless stack up: nmpc=$NODE_LOG mhe=$MHE_LOG"
echo "  payload=${GRIP_PAYLOAD_KG}kg ecc_y=${GRIP_ECC_Y}m r_xy=$R_XY"
echo "  event_trigger=${MHE_EVENT_TRIGGER:-true} signal=${MHE_SIGNAL_MODE:-residual} drop_publish=${DROP_PUBLISH_MASS_EVENT:-false}"
echo "  geom_release: NMPC=${NMPC_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-event}} MHE=${MHE_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-self}} cxy_release_mp=${MHE_CXY_MASS_RELEASE_MP:-0.03}"
echo "  theta=${MHE_SCHEDULE_THETA:-M0} confirm_alpha=$MHE_CONFIRM_ALPHA_D (alpha>=0 => thresh=alpha*g*envelope; <0 => fixed ${MHE_CONFIRM_THRESH:-1.5}N)"
echo "  payload_envelope=${GRIP_PAYLOAD_ENVELOPE:-0.3}kg (机架规格; attach 仅用它引导 J,m/s 仍来自 MHE)"
