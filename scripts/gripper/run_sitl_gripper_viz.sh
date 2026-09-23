#!/bin/bash
# 吊挂夹爪场景 GUI 可视化启动器(2026-07-15):看**当前主配置**的完整飞行——
# Gazebo GUI(物理:drone 抓 box / 抬升 / figure8 / drop 掉落)+ RViz(控制端:
# figure8 参考 vs 实际轨迹 + NMPC 预测 horizon + drone 模型 + 旋翼动画)。
# 与 headless 批量版(run_gripper_headless.sh)的区别:开 GUI、接 RViz、用主配置
# (online 无真值几何 + θ* 学习调度 + α 无真值确认 + drop + figure8 动态)。
# URDF 和 RViz 资源均归属 gripper 场景。RViz 配置于 07-30 改用
# config/gripper/nmpc_view_gripper_hifly.rviz——r=5 的
# 大 8 字会顶出原配置那个 ±5m 默认网格、Distance:12 也框不住,故单独一份
# (网格 24m、Distance 26、焦点挪到 attach 中心 x=1)。
#
# ★★ 2026-09-02:默认工作点整体换成**无信号主线 = 演示视频 20260901_141240 那轮**
#    (原样照抄 run_cxy_mass_repeat.sh 的环境变量)。改了两组东西:
#      工况   0.15kg/包线 0.3 / r=10 w=0.283 z=6.0 lift 8.4 ramp 9.36  (4m/s 工作点)
#      机制   事件不发(DROP_PUBLISH_MASS_EVENT=false) + MHE residual +
#             MHE 几何 self 档 + 质量域卸载判据 0.03kg + α=1.5(阈值 4.4N)
#    语义 = 三条给估计器的信息通道全堵死,drop 只由质量域判据检出(实测 +3.35s)。
#    ⚠️ r 与 z 必须配套:r=10 在 z=2.5 会必然发散坠毁(记忆 high-maneuver-ablation),
#       只改一个是危险的。
#    回到 2026-09-01 之前的旧默认(2m/s 工作点 + 有信号 event 主线),整行覆盖:
#      GRIP_PAYLOAD_KG=0.15 GRIP_PAYLOAD_ENVELOPE=0.3 GRIP_DYN_R=5.0 GRIP_DYN_RAMP=3.0 \
#      GRIP_Z_HIGH=2.5 GRIP_LIFT_DUR=3.0 GRIP_LIFT_AFTER=1.5 DROP_PUBLISH_MASS_EVENT=true \
#      MHE_SIGNAL_MODE=external MHE_GEOM_RELEASE_MODE=event MHE_CXY_MASS_RELEASE_MP=0 \
#      MHE_CONFIRM_ALPHA=-1 bash src/scripts/gripper/run_sitl_gripper_viz.sh
#
# 用法:  [GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 METHOD=thetastar GRIP_DYN_R=5.0 \
#          GRIP_DYN_W=0.283 GRIP_DYN_DZ=0.8 GRIP_DROP_AFTER=55.0 \
#          GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
#          GRIP_PAYLOAD_ENVELOPE=0.5 MHE_CONFIRM_ALPHA=0.9875] \
#          bash src/scripts/gripper/run_sitl_gripper_viz.sh
#        GRIP_DYN_DZ=0 可退回原来的平面 8 字。
# 收栈:关掉各 gnome-terminal 窗口即可;或 pkill -9 -f 'px4_sitl|gz sim|mhe_node|...'。

PAYLOAD_KG="${GRIP_PAYLOAD_KG:-0.15}"   # 09-02: 0.3 -> 0.15(演示轮载荷)
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
DYN_R="${GRIP_DYN_R:-10.0}"    # 09-02: 5.0 -> 10.0(4m/s 工作点,必配 Z_HIGH=6.0)
DYN_W="${GRIP_DYN_W:-0.283}"
# ★ 09-02: 3.0 -> 9.36 —— r 翻倍后 v_peak 到 4m/s,起振冲击也翻倍,取 auto 在
# w=0.283 下要的那个值(上面算过 9.37s),与演示轮/run_cxy_mass_repeat 一致。
DYN_RAMP="${GRIP_DYN_RAMP:-9.36}"
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
# 2026-08-30 默认改 true(与节点/headless 同步,用户拍板;证据未达 n>=8 门槛,
# 见 acados_nmpc_node.py 里 dj_track_mest 的声明注释)。
DJ_TRACK=$(_b "${DJ_TRACK_MEST:-true}")
# 载荷**意外**脱落看门狗(2026-08-30,与 headless 同名同默认)。演示视频用
# MHE_PAYLOAD_LOST_WATCH=1 打开;计划内 drop 不走这条路,见 payload-lost-watchdog。
PL_WATCH=$(_b "${MHE_PAYLOAD_LOST_WATCH:-false}")
# 模型 dJ 的下界载荷质量 kg。attach 时 dJ 从 0 起步,地板保证它不掉到空机惯量
# 附近。★ 2026-09-02 默认 0.15 → 0.05,与 DJ_RATCHET 默认翻 false 配套(见下)
# ——0.15 和最小实验载荷同量级,会把 0.15kg 工况的 m_p_model 整个钉死在地板上。
DJ_FLOOR_MP_D=$(_f2dv "${GRIP_DJ_FLOOR_MP:-0.05}")
# 模型 dJ 走棘轮(只增不减)还是双向跟随 m_est。★ 2026-09-02 默认 true → false:
# run_dj_ratchet_ab.sh 的 n=7 配对 A/B 判棘轮在带载段是不受控的正偏差补偿器,
# 关掉后 dJ 稳态误差 +26.9% → +5.5%,7/7 同向 p=.0156(记忆 mass-domain-
# payload-release)。代价 = dJ 跟着 m_est 抖,小载荷下差分放大 ~7.9×。
# 增益侧不受影响,永远用棘轮峰值("必须够大"语义)。
DJ_RATCHET_D=$(_b "${DJ_RATCHET:-false}")
# drop 时是否把 mass_event 发给 MHE。false = 让 MHE 自己从 T_phys 残差看出来
# (须配 MHE_SIGNAL_MODE=residual)。默认 true = 历史行为。
# ★ 09-02 默认 true -> false:无信号主线不给 MHE 发 drop 事件。
DROP_PUB_EVENT=$(_b "${DROP_PUBLISH_MASS_EVENT:-false}")
MP_CAP_D=$(_f2dv "${GRIP_MP_CAP:-0.6}")
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
# ⚠️ 未覆盖:4m/s 工作点；一阶质量矩在无事件主线默认开启。
# 注:MHE_GEOM_COUPLED 已删。estimate_moment 分支优先,09-04 起它再未生效过。
export MHE_ESTIMATE_MOMENT="${MHE_ESTIMATE_MOMENT:-1}"
export MHE_MOMENT_A_MODE="${MHE_MOMENT_A_MODE:-frozen}"
export MHE_SIGMA_S0="${MHE_SIGMA_S0:-0.1}"
MANEUVER_GATE=$(_b "${MHE_MANEUVER_GATE:-false}")
RESID_RELEASE_GEOM=$(_b "${MHE_RESID_RELEASE_GEOM:-true}")
# 并行阶跃判据(2026-08-26,默认关):短窗前后均值差,不需慢基线因而没有预热失效面。
# 离线回放验证见 test/test_step_detector_replay.py(drop 延迟 0.30s、余量 2.4×、
# 稳态零误触发)。⚠️ 轨迹切换与质量突变在推力通道上不可分,所以释放几何用一条
# 单独的更高门槛 RESID_STEP_REL(默认 2.0N)。
RESID_STEP=$(_b "${MHE_RESID_STEP:-false}")
# c_xy 来源:1 = 用一阶质量矩 s/m_T(窗口内估计,无稳态门控,机动中也更新);
# 0 = 窗外 EMA + 稳态门控(现状,figure-8 中一次都不更新)。需 MHE_ESTIMATE_MOMENT=1。
C_XY_FROM_MOMENT=$(_b "${MHE_C_XY_FROM_MOMENT:-true}")
RESID_STEP_HALF="${MHE_RESID_STEP_HALF:-3}"
RESID_STEP_PERSIST="${MHE_RESID_STEP_PERSIST:-2}"
RESID_STEP_TH_D=$(_f2dv "${MHE_RESID_STEP_TH:-1.0}")
RESID_STEP_REL_D=$(_f2dv "${MHE_RESID_STEP_REL:-2.0}")
MG_OMEGA_D=$(_f2dv "${MHE_MG_OMEGA:-0.15}")
MG_VEL_D=$(_f2dv "${MHE_MG_VEL:-0.20}")
MG_CAP_D=$(_f2dv "${MHE_MG_CAP:-10000.0}")
MG_EXP_D=$(_f2dv "${MHE_MG_EXP:-2.0}")

# ★ 09-02: z 2.5 -> 6.0、lift_dur 3.0 -> 8.4(4m/s 工作点;r=10 在 z=2.5 必发散)
Z_HIGH="${GRIP_Z_HIGH:-6.0}"
LIFT_DUR="${GRIP_LIFT_DUR:-8.4}"

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
# 看门狗的浮点参数必须放在 _f2d 定义**之后**(前面那几行只用 _b)。
PL_STEP_TH_D=$(_f2d "${MHE_PL_STEP_TH:-2.0}")
PL_MARGIN_D=$(_f2d "${MHE_PL_MASS_MARGIN:-0.10}")
PL_HOLD_D=$(_f2d "${MHE_PL_HOLD_SEC:-3.0}")
PL_VZ_D=$(_f2d "${MHE_PL_VZ_GATE:-0.30}")
DYN_R_D=$(_f2d "$DYN_R"); DYN_W_D=$(_f2d "$DYN_W"); DYN_RAMP_D=$(_f2d "$DYN_RAMP")
DYN_DZ_D=$(_f2d "$DYN_DZ")
Z_HIGH_D=$(_f2d "$Z_HIGH"); LIFT_DUR_D=$(_f2d "$LIFT_DUR")
# attach 命令到 LIFT 之间的等待(2026-09-04 补透传,原先硬编码 1.5)。加大它
# 给吸附更多稳定时间:d_xy 收不进 r_xy=0.13 时 box 吸不上,而 NMPC 的 J
# bootstrap 已按包线放大内环 -> 空机吃带载增益,LIFT 段发散(实测两次)。
# 只影响抓取等待,不碰 drop 机制/figure8/任何进论文的量。
# ★ 2026-09-04 默认 1.5 -> 3.0:同日三轮 viz 实测把它坐实成必须改的默认——
#   1.5 那轮(gviz_*_20260904_162428)LIFT 段直接发散:822 次 solve failed、
#   tilt 141°、pos_err 70m、om_scale 钉在 5.0、MHE health=0 持续 21s;
#   3.0 的两轮(163315/164651)同档位干净通过(0 solve failed,DROP complete)。
#   相位不受影响:lift_after 在 t_drop 与 dyn_t0 里自动抵消(见 DROP_AFTER 推导),
#   整条时间线只是后移 1.5s,DROP_AFTER 不必重算。
LIFT_AFTER_D=$(_f2d "${GRIP_LIFT_AFTER:-3.0}")
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
PAYLOAD_ENVELOPE_D=$(_f2d "${GRIP_PAYLOAD_ENVELOPE:-0.3}")   # 09-02: 0.5 -> 0.3
ATTACH_WINDOW_SEC_D=$(_f2d "${ATTACH_WINDOW_SEC:-40.0}")
BOX_I=$(python3 -c "print(f'{$PAYLOAD_KG * 0.00375:.6f}')")
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
R_XY=$(python3 -c "print(f'{$ECC_Y + $GRIP_ATTACH_TOL:.3f}')")

if [ "$METHOD" = thetastar ]; then
  THETA="[-4.8038,-1.2080,0.4930,-0.9602,0.9875]"; CALPHA="0.9875"
else
  # ★ 09-02: CALPHA -1.0(固定 1.5N) -> 1.5(= α·g·envelope = 4.4N @0.3kg 包线)。
  # 抬高阈值是**故意**的:0.15kg 卸载才 ~1.47N,4.4N 等于把残差检测这条路堵死,
  # 逼 drop 只能由质量域判据检出 —— 这正是无信号主线要展示的东西。
  # ⚠️ 阈值随 GRIP_PAYLOAD_ENVELOPE 联动:覆盖成 0.5 时阈值变 7.36N。
  THETA="[-4.0,0.0,0.0,0.0]"; CALPHA="1.5"
fi
# 确认阈值 = α·g·grip_payload_envelope(CALPHA<0 时走固定 event_confirm_thresh_n)。
# 2026-08-26 前这里是 MHE_CONFIRM_PRIOR(默认回落到 $PAYLOAD_KG = box 真值)。
# α 与 θ 正交(ParametricWeightSchedule 只读 theta[0..3],α 走独立参数)。
# MHE_CONFIRM_ALPHA 保持为独立覆盖项,便于单独调整确认阈值。
CALPHA="${MHE_CONFIRM_ALPHA:-$CALPHA}"

WS="/home/clear/ros2_ws_HJH"
PX4_DIR="/home/clear/PX4-Autopilot"
PKG="$WS/src/offboard_test_acados"
GRIPPER_DIR="$PKG/gz_plugins/magnetic_gripper"
WORLD_SRC="$PKG/worlds/gripper/gripper_test.sdf"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
RVIZ_CONFIG="${RVIZ_CONFIG:-$PKG/config/gripper/nmpc_view_gripper_hifly.rviz}"
URDF_FILE="$PKG/urdf/gripper/x500.urdf"
LOGDIR="$WS/nmpc_test_results"
TS=$(date +%Y%m%d_%H%M%S)
ACADOS_ENV="export ACADOS_SOURCE_DIR=/home/clear/acados && export LD_LIBRARY_PATH=/home/clear/acados/lib:\$LD_LIBRARY_PATH"
mkdir -p "$LOGDIR"
bash "$WS/src/scripts/record_experiment_provenance.sh" \
  "$LOGDIR/provenance_$TS" "gripper-viz:$TS"

echo "cleaning up leftover sim processes..."
for pat in "px4_sitl_default/bin/px4" "gz sim" "/gz " "ruby" "mavros/mavros_node" \
           "transport13/gz-transport-topic" \
           "lib/offboard_test_acados/proximity_gripper_node" \
           "lib/offboard_test_acados/acados_nmpc_node" \
           "lib/offboard_test_acados/mhe_node" \
           "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "prop_joint_state" \
           "gz topic -e -t /gripper/state" \
           "drone_tf_broadcaster" "robot_state_publisher" "rviz2" \
           "ninja gz_x500" "make px4_sitl"; do
  for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
done
sleep 2
# PX4 参数持久化护栏(见记忆 px4-param-persistence-guard):清落盘的放大过增益
find "$PX4_DIR/build/px4_sitl_default/rootfs" -maxdepth 1 -name 'parameters*.bson' -delete 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select offboard_test_acados
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

# 4b. 动态避让:把 RViz 摆到 Gazebo **不在**的那块屏(2026-09-01)。
# 双屏录 demo 时两个窗口叠在一起就录不到 Gazebo(后起的 RViz 压在上面),而
# `~/.gz/sim/8/gui.config` 的 position_x **时灵时不灵** —— 同一天实测 Gazebo
# 一次落主屏、一次落副屏,配置写的都是 1920(记忆 demo-video-pipeline 坑 6)。
# RViz 的 `Window Geometry: X` 则一直可靠,所以让可靠的那个去躲不可靠的那个:
# 起 RViz **之前**看一眼 Gazebo 实际落在哪,把 RViz 的 X 写成另一块屏。
# 只在双屏 + 有 xwininfo 时生效;单屏或读不到几何就原样不动。
# 写的是**临时副本**,不碰用户的配置文件。
if command -v xwininfo >/dev/null 2>&1 && command -v xrandr >/dev/null 2>&1; then
  _n_mon=$(xrandr --listmonitors 2>/dev/null | awk 'NR==1{print $2}')
  # Gazebo GUI 可能还没画出窗口,重试等它出现(最多 20s)
  _gz_x=""
  for _t in 1 2 3 4 5 6 7 8 9 10; do
    _gz_x=$(xwininfo -root -tree 2>/dev/null | grep 'gz-sim-gui' | head -1 \
            | grep -oE '\+-?[0-9]+\+-?[0-9]+$' | cut -d+ -f2)
    [ -n "$_gz_x" ] && break
    sleep 2
  done
  if [ "${_n_mon:-1}" -ge 2 ] && [ -n "${_gz_x:-}" ]; then
    # Gazebo 在主屏(x<960) -> RViz 去副屏 1920;否则 RViz 留主屏 0
    if [ "$_gz_x" -lt 960 ]; then _rv_x=1920; else _rv_x=0; fi
    _rv_tmp="/tmp/nmpc_view_gripper_auto_$TS.rviz"
    if sed -E "/^Window Geometry:/,/^[^ ]/ s/^  X: .*/  X: $_rv_x/" \
         "$RVIZ_CONFIG" > "$_rv_tmp" 2>/dev/null && [ -s "$_rv_tmp" ]; then
      echo "版面避让: Gazebo x=$_gz_x -> RViz X=$_rv_x ($_rv_tmp)"
      RVIZ_CONFIG="$_rv_tmp"
    fi
  fi
fi

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

# 7b. /gripper/state 物理真值录制(2026-09-05)
# 插件在真正建立/移除 DetachableJoint 时才发 ATTACHED/DETACHED。proximity 只
# 记录"发出了 ATTACH 请求",两者不是一回事:20260905_182456 那轮 proximity 条件
# 满足、请求已发,但 MHE 全程没看到载荷,而当时**没有保存回执**,无法判定关节到底
# 有没有建立。这条日志只进实验记录与离线有效性判定,NMPC/MHE 都不消费它,
# 因此不构成外部事件信号。
nohup gz topic -e -t /gripper/state > "$LOGDIR/gviz_state_$TS.log" 2>&1 &

# 8. acados NMPC(主配置:gripper_mode + online 几何 + drop + figure8 动态)
NODE_LOG="$LOGDIR/gviz_nmpc_$TS.log"; echo "NMPC log: $NODE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
  ros2 run offboard_test_acados acados_nmpc_node --ros-args \
    -p gripper_mode:=true -p grip_x:=1.0 -p grip_y:=$ECC_Y_D \
    -p grip_z_low:=0.55 -p grip_z_high:=$Z_HIGH_D \
    -p grip_mass_step_sec:=0.0 -p grip_payload_mass:=$PAYLOAD_KG_D -p grip_arm_d:=0.47 \
    -p grip_lift_after_sec:=$LIFT_AFTER_D -p grip_lift_dur:=$LIFT_DUR_D -p use_mhe:=true \
    -p continuous_payload_estimates:=${CONTINUOUS_PAYLOAD_ESTIMATES:-true} \
    -p grip_lift_hold_enable:=$LIFT_HOLD \
    -p grip_lift_hold_dz:=$LIFT_HOLD_DZ_D -p grip_lift_hold_sec:=$LIFT_HOLD_SEC_D \
    -p dj_track_mest:=$DJ_TRACK -p grip_mp_cap:=$MP_CAP_D \
    -p grip_dj_floor_mp:=$DJ_FLOOR_MP_D \
    -p dj_ratchet_enable:=$DJ_RATCHET_D \
    -p drop_publish_mass_event:=$DROP_PUB_EVENT \
    -p geom_source:=estimate -p grip_payload_envelope:=$PAYLOAD_ENVELOPE_D \
    -p attach_j_bootstrap_enable:=${ATTACH_J_BOOTSTRAP:-true} \
    -p cxy_freeze_in_maneuver:=${NMPC_CXY_FREEZE:-false} \
    -p cxy_freeze_window_sec:=${NMPC_CXY_FREEZE_WINDOW:-3.0} \
    -p omega_scale_enable:=${NMPC_OMEGA_SCALE:-true} -p omega_scale_source:=djest \
    -p geom_release_mode:=${NMPC_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-event}} \
    -p grip_drop_after_sec:=$DROP_AFTER_D \
    -p grip_dynamic_after_lift:=$DYNAMIC \
    -p grip_dyn_r:=$DYN_R_D -p grip_dyn_w:=$DYN_W_D -p grip_dyn_ramp:=$DYN_RAMP_D \
    -p grip_dyn_dz:=$DYN_DZ_D \
    -p grip_drop_at_fig8_tip:=$DROP_AT_TIP \
    -p attach_window_sec:=$ATTACH_WINDOW_SEC_D" \
  > "$NODE_LOG" 2>&1 &

# ★★ 2026-09-17 晚:MHE 力矩来源默认翻为 phys_full + 区间平均 + 越界重锚(用户拍板)。
#   依据 = 预注册配对 A/B pfovr_ab_20260917_193153(command vs 本组合,w2/w4 各 16 对,
#   吊起 + 8 字一圈、**无投放**):吊起段 m 贴下界 1.961 的帧占比 24.5%→3.2%,
#   配对差 +21.3pp 95%CI[+8.7,+33.9] Wilcoxon p=5.7e-5;一票否决未触发(发散 A1/B1,
#   B 那轮是与臂无关的爬升途中晚吸附);求解失败中位 4→0。机理见记忆
#   liftoff-lb-stuck-mechanisms(离地振荡时点采样的电机推力/力矩严重混叠,把 m 压到
#   下界 → J bootstrap 永不交接 → 内环 5× 过增益)。
#   ⚠️ sensor-only 主线(external_event_inputs=false)下 MHE 从不使用 NMPC 指令:
#   u_known 始终是电机反算值,mhe_tau_source 只决定 motor_window_avg 开时哪些力矩分量做
#   区间平均。所以 A/B 两臂的真实差别是"点采样 vs 区间平均"(+越界重锚),不是"指令 vs 实测"。
#   ⚠️ 未验证:投放/释放段(09-05 判负 phys_full 的那批是带投放的)。越界重锚在该批
#   32 轮里 0 次触发,效果只有开环回放证据。复现 09-17 晚之前的批次须显式
#   MHE_TAU_SOURCE=command MHE_MOTOR_AVG=0 MHE_REANCHOR_OVR=0。
# 9. MHE(主配置:x500_0 + θ* 调度 + α 无真值确认 + floor + c_xy 在线估计)
# --- MHE 已知力矩来源:与 run_gripper_headless.sh 对齐(2026-09-10)---
# 节点默认是 phys_full + motor_window_avg=True,而 headless 一直显式覆盖成
# command + false —— 两个入口长期不一致,**演示看到的和批次统计的不是同一个
# 估计器输入**。2026-09-05 用 n=8 配对判掉,结论是留在 command 这一档:
#   · physfull 臂 1/8 轮在 **drop 之前**发散(peak 40.6m、attach 后 4.3s 起
#     solve failed、推力塌到下界 0.50N),command 臂 0/8、同底座历史再 0/12;
#     这是 2026-08-24 那个失效模式的复现,配 motor_window_avg=1 也没消掉。
#   · 质量精度**未检出差异**(带载稳态偏差配对差 +0.21pp,95% CI [−0.05,+0.47],
#     5/7 同向 p=.45;跑前功效可检出 0.23pp)——不能反过来说两者等价。
#   · physfull 唯一显著的好处是 MHE 求解 9.09→7.41ms(−17.5%,7/7 同向 p=.0067),
#     但 10Hz MHE 预算 100ms,占比 9.1%→7.4%,没有实际意义。
#   判负的力量来自预注册的一票否决 + 08-24 先验 + viz 历史旁证(23 份 gviz 日志
#   里 2 份 drop 前 peak 就到 45.7/70.4m),**不是**本批的统计功效:单看计数
#   1/8 vs 0/8 Fisher p=1.0。所以措辞是"physfull 有 drop 前发散实例且无可测收益"。
#   记忆 tau-source-viz-headless-ab;设施 run_tau_source_ab.sh。
# ⚠️ 节点默认仍是 phys_full:两个入口都靠**显式覆盖**才一致,任何新写的入口或
#    裸 `ros2 run mhe_node` 仍会走 phys_full。留 MHE_TAU_SOURCE/MHE_MOTOR_AVG
#    两个环境变量是为了让对照实验还能 opt-in 回去(与 headless 同名同语义)。
MHE_LOG="$LOGDIR/gviz_mhe_$TS.log"; echo "MHE log: $MHE_LOG"
nohup bash -c "source /opt/ros/jazzy/setup.bash && source '$WS/install/setup.bash' && \
  $ACADOS_ENV && export PYTHONUNBUFFERED=1 && \
  ros2 run offboard_test_acados mhe_node --ros-args \
    --params-file '$PKG/config/gripper/gripper_params.yaml' \
    -p mhe_tau_source:=${MHE_TAU_SOURCE:-phys_full} \
    -p motor_window_avg:=$(_b "${MHE_MOTOR_AVG:-true}") \
    -p reanchor_on_moment_overrange:=$(_b "${MHE_REANCHOR_OVR:-true}") \
    -p maneuver_gate_enable:=$MANEUVER_GATE \
    -p resid_release_geom:=$RESID_RELEASE_GEOM \
    -p resid_step_enable:=$RESID_STEP \
    -p resid_step_half:=$RESID_STEP_HALF -p resid_step_thresh:=$RESID_STEP_TH_D \
    -p resid_step_persist:=$RESID_STEP_PERSIST \
    -p resid_step_release_thresh:=$RESID_STEP_REL_D \
    -p payload_lost_watch_enable:=$PL_WATCH \
    -p payload_lost_step_thresh:=$PL_STEP_TH_D \
    -p payload_lost_step_half:=${MHE_PL_STEP_HALF:-3} \
    -p payload_lost_step_persist:=${MHE_PL_STEP_PERSIST:-2} \
    -p payload_lost_mass_margin:=$PL_MARGIN_D \
    -p payload_lost_hold_sec:=$PL_HOLD_D \
    -p payload_lost_vz_gate:=$PL_VZ_D \
    -p maneuver_omega_thresh:=$MG_OMEGA_D -p maneuver_vel_thresh:=$MG_VEL_D \
    -p maneuver_q0_cap:=$MG_CAP_D -p maneuver_exponent:=$MG_EXP_D \
    -p motor_speed_topic:=/x500_0/command/motor_speed \
    -p external_event_inputs:=$(_b "${MHE_EXTERNAL_EVENTS:-false}") \
    -p event_trigger_enable:=true -p schedule_theta:='$THETA' \
    -p confirm_thresh_alpha:=$CALPHA \
    -p grip_payload_envelope:=$PAYLOAD_ENVELOPE_D -p c_xy_est_enable:=true \
    -p c_xy_from_moment:=$C_XY_FROM_MOMENT \
    -p eval_true_payload_mass:=$PAYLOAD_KG_D \
    -p geom_release_mode:=${MHE_GEOM_RELEASE_MODE:-${GEOM_RELEASE_MODE:-self}} \
    -p release_resid_enable:=$(_b "${RELEASE_RESID:-true}") \
    -p release_resid_floor_n:=${RELEASE_RESID_FLOOR:-0.9} \
    -p release_resid_strong_persist:=${RELEASE_RESID_STRONG_PERSIST:-4} \
    -p c_xy_mass_release_mp:=${MHE_CXY_MASS_RELEASE_MP:-0.03} -p c_xy_mass_release_persist:=${MHE_CXY_MASS_RELEASE_PERSIST:-20} -p c_xy_mass_arm_ratio:=${MHE_CXY_MASS_ARM_RATIO:-3.0} -p c_xy_mass_arm_persist:=${MHE_CXY_MASS_ARM_PERSIST:-20} -p event_signal_mode:=${MHE_SIGNAL_MODE:-residual}" \
  > "$MHE_LOG" 2>&1 &

echo "All components up. Gazebo GUI = 物理飞行; RViz = 控制端跟踪。"
echo "  NMPC: tail -f $NODE_LOG   MHE: tail -f $MHE_LOG"
