#!/usr/bin/env python3
# MHE(Moving Horizon Estimation,移动时域估计)的参数/权重配置。
# 跟 acados_params.py 一样,物理常数(g/kd/Jxx/Jyy/Jzz)直接复用
# offboard_test.nmpc_node.Params,避免两边各存一份却忘了同步;
# MHE 自己的窗口长度/噪声权重故意独立,不影响 NMPC 那边。
#
# 这一版只估"质量 m"这一个参数(配送场景里影响最大:决定推力->加速度映射、
# 悬停油门、推力裕度约束——见 配送无人机自适应学习控制_技术路线笔记.md 第4节、
# 第1.2节)。质心偏移 Δr 和惯量 J 暂不估,留给以后需要时再加。

import os

import numpy as np
from offboard_test.nmpc_node import Params as _BaseParams

_base = _BaseParams()


class MHEParams:
    # --- 物理常数,跟 offboard_test/acados 共享 ---
    g   = _base.g     # 重力加速度(m/s^2)
    kd  = _base.kd    # 线性空气阻力系数,跟动力学方程里的一致
    Jxx = _base.Jxx
    Jyy = _base.Jyy
    Jzz = _base.Jzz

    # --- 状态维度 ---
    nx     = 13  # 飞行器物理状态: pos(3)+vel(3)+quat(4)+omega(3),跟 NMPC 的 x 完全一致
    nm     = 1   # 待估参数: 总质量 m_T
    # 过程噪声维度。默认 = nx(只加在 13 个物理状态上,被估参数窗口内是刚性常数)。
    # param_noise=True 时把被估参数(m、s)也纳入噪声驱动 → 随机游走,这是文献
    # (NeuroMHE/DMHE)处理**突变参数**的标准做法:参数能在窗口内变化,突变不再
    # 依赖"等旧数据滑出窗口"或外部事件降权。
    # ⚠️ 2026-08-24 实测证明这不是可选项而是 2b 的**前提**:s 若 ṡ=0 刚性,drop
    #    瞬间 s 还残留非零 → 模型预测幽灵配平力矩 (s/m)·T,而实测 ω≈0;优化器
    #    消掉它的两条路是 s→0 或 **m→∞**(因为 c=s/m_T,质量在分母上),s 动不了
    #    就只能 m 背锅——离线实测 m 单调爬到 3.17 不回头、s 在 0 附近逐帧变号。
    param_noise = os.environ.get('MHE_PARAM_NOISE', '0') not in ('0', '', 'false', 'False')

    # --- 遗忘因子 / 衰减记忆(2026-08-24)---
    # 逐 stage 按"年龄"给测量+过程噪声权重打折:W_j = λ^(N-1-j)·blkdiag(R,Q),
    # j=0 最老、j=N-1 最新。λ=1 逐位退回现状。
    # 为什么是这个位置而不是"衰减到达代价":Q0 的质量项已经是 0.1(σ=3.16kg,
    # 实质无先验),再弱化它没有意义;真正让质量刚性的是"ṁ=0 + 事件前老量测以
    # 全权重把它往旧值拽"——所以要打折的是**老 stage 的 R/Q**。到达代价那一块
    # 不打折(它锚的是物理状态,动它会伤状态估计)。
    # 相对 param_noise(2a)的优势:**不增加状态、不新增噪声通道,Q 的标度一个字
    # 没动** → 绕开 2026-08-24 实测那个"参数噪声权重 1e2 vs 物理 1e4~1e5、
    # Hessian 病态 → 472 次求解失败"的坑。
    # 有效记忆长度 ≈ 1/(1−λ) 个 stage:λ=0.9→10 帧(1.0s)、λ=0.8→5 帧(0.5s)。
    # ⚠️ 顺带与既有证据一致:记忆 new-4ms-workpoint / mhe_params.N 注释记录过
    #    "4 m/s 上 N=10 比 N=20 好 3.7 倍",衰减记忆是它的连续版(不硬截断、
    #    QP 结构不变、到达代价仍在远端)。
    forgetting_lambda = float(os.environ.get('MHE_LAMBDA', '1.0'))

    nu_known = 4  # 已知输入: 总推力 T + 力矩 tau_x,tau_y,tau_z(跟 NMPC 实际发出的指令一致,
                  # 通过 model.p 传入,不是被估计的量)
    n_geom = 3    # 已知几何,**语义随 geom_coupled 切换**(维度都是 3,接口不变):
                  #   geom_coupled=False(legacy):[dJ, cx, cy]——在窗外由
                  #     mhe_node._payload_geometry 用某个 m_p_hat 先算好再喂进来,
                  #     于是 ∂(dJ,c)/∂m ≡ 0,质量对 om_dot 没有任何导数通路。
                  #   geom_coupled=True (2026-08-24):[rx, ry, rz]——**载荷相对
                  #     机体原点的几何偏移**(attach 可测的量,不是被估的质量),
                  #     模型内部按 m_P=m−m_B 现算 c(m)=(m_P/m)r_xy 与
                  #     J(m)=J_B+μ(m)(|r|²I−rrᵀ),μ=m_B·m_P/m。这样质量同时出现在
                  #     平动和转动两条通路里,MHE 的 Jacobian 才是一致的。
                  # 数值依据(scratchpad/ident.py,N=20/dt=0.1 窗口的 Cramér-Rao 下界):
                  #   ry=0.05 偏心悬停 σ_m: legacy 0.0109kg → coupled 0.0011kg(≈10×),
                  #   且 coupled 下 94% 的质量信息来自转动通路;居中(r_xy=0)且不机动
                  #   时两者相同(c≡0、ω̇≈0,转动通路本来就没信息)。
                  # ⚠️ coupled 下 r_xy 的先验误差会**直接**变成质量偏差(转动通路辨识
                  #   的其实是乘积 m_P·r_xy):实测 ry 给一半 → m 偏 +14.4%,ry 给 0 →
                  #   +41.6%;而 rz 给错 ±33% → 质量偏差 0.00%(rz 只进 dJ,悬停 ω̇≈0
                  #   时不携带信息)。所以 coupled 要求偏心量可测,rz 可以很糙。
                  # 跟 acados_model.py 的 dJ_sym/c_sym 同一套物理(平行轴定理+
                  # 推力对质心的力矩),照抄进 MHE 自己的 om_dot——2026-07-07
                  # 坏几何复测发现:MHE 内部动力学如果还用固定空机 J、不知道
                  # 吊挂惯量增量,给定力矩算出的角加速度系统性偏大,这个姿态
                  # 通道的模型失配会通过 quat->R_q->vel_dot 传导,被优化器拿
                  # "质量"这个自由参数背锅——偏心/力矩越大,质量估计偏得越狠
                  # (实测 ry=0.05、hover roll tau=0.1Nm 时 m_est 系统性偏低到
                  # 1.7kg,真值 2.36kg)。跟 NMPC 一样喂 dJ/c_xy 才是对症的修法,
                  # 而不是本文件头注释说的"暂不估"——这里不是新增被估计的自由
                  # 度,是把 MHE 自己模型里本来就该知道的已知物理喂给它,跟
                  # NMPC 的 model.p 处理方式完全对应。

    # --- 窗口长度/步长 ---
    # 窗口跨度 N*dt=2.0s:短到不会把一次质量阶跃(抓/放包裹)压成过长的过渡态。
    # ⚠️ 这里原先写的另一条理由是"比瞬时辨识更抗噪声、所以窗口越长越好"——
    #    2026-08-19 六格析因(2/4 m/s × N=10/20/40)在**带载高机动**工况下把它
    #    证伪了:4 m/s 上 N=10 的 m_est 偏差比 N=20 好 3.7 倍(-2.25% vs -8.30%)、
    #    IQR 也更小(0.027 vs 0.045);N=40 全程钉在下界 1.961、pos_err 升到同速度
    #    N=10 的 1.3 倍并出现 3 次 solve fail。机制:窗口越长,"窗口内质量恒定+
    #    动力学近似一致"这个辨识约束覆盖的轨迹弧长越大,近似越不成立。
    #    当前判别最支持"窗口占轨迹周期比"假设,但该批 r 固定 5.0 使占比与弧长
    #    共线、且 n<8,**结论未定,默认值暂不动**(见 工作日志_20260819.md §3-§5)。
    #    近悬停/低速工况下 20 仍是合适默认;高速档若要复现精度数字,须标注 N。
    # NMPC 已改为
    # 20Hz，但 MHE 仍独立保持 10Hz/2s 窗口；它在自己的定时器帧读取最新状态和
    # NMPC 输入，不要求两个求解器同频。
    # 可用环境变量 MHE_N 覆盖(默认 20),照 m_min 的先例:用环境变量而不是改默认
    # 值,防止消融实验后忘记恢复。窗口跨度 = N*dt。
    # ⚠️ 改 N 会让 acados 重新生成 solver(N_horizon/tf/qp_solver_cond_N 都变),
    #    第一轮启动慢约 30s;且 codegen 目录不含 N、各档共用一份
    #    (见 mhe_solver_builder.codegen_dir)→ **不同 N 的批次必须串行,并发会互相
    #    覆盖生成代码**。
    # ⚠️ 归因警告:改 N 同时动两件事——(a)窗口跨度,(b)代价里测量项相对到达代价
    #    的总权重(W=block_diag(R,Q) 是 per-stage 常数、不随 N 归一化,而 Q0 只在
    #    stage 0 出现一份)。所以"扫 N 看趋势"单独不可归因,要靠 (速度 x N) 析因
    #    里的等占比对角线把"窗口占轨迹周期比"与"纯 N 效应"分开。
    N  = int(os.environ['MHE_N']) if os.environ.get('MHE_N') else 20
    dt = 0.1

    # --- 质量先验/边界 ---
    # m_nominal: 空机质量(跟 acados_params.py 的 p.m 一致)。m_min/m_max 给一个
    # "物理上说得通"的范围(空机~满载3kg包裹),纯粹是防止早期窗口激励不足时
    # 优化器把质量推到离谱的值——不是真实约束,只是数值上的安全带。
    m_nominal = _base.m
    # m_min:质量的**物理下界地板**(solver lbx 硬约束,见 mhe_solver_builder.py)。
    # 原值 1.0 比空机(2.064kg)还轻 1.06kg,等于放任优化器在一整段**物理不可达**
    # 区间里搜索——飞机不可能比空机轻,载荷只加不减。2026-07-17 坐实这是 gripper
    # attach 25%(4/16 轮)失败的根因:de-weight 松开约束后 m_est 探底撞 m_min
    # (实测 1.046),一旦跌破 m_nominal,mhe_node 的 m_p_hat=max(m_est-m_nominal,0)
    # 就被钳成 0 → MHE 认定"无载荷"、dJ/c_xy 几何补偿全归零 → 失去爬回真值的驱动,
    # 锁进自洽的错误平衡(1.44~1.46kg),在 T_phys=23.1N(明示 2.36kg)面前 30s 不
    # 自纠;NMPC 消费这个错值后稳态下垂 0.6m、抬不到目标高度(不坠机,但任务失败)。
    # 修法与 grip_geom_mp_floor 同一哲学:用真机可用的弱物理先验做永久下界地板。
    # 取 0.95×空机而非空机本身:留 5% 裕度给标定误差,别卡死在标定值上。
    # 可用环境变量 MHE_M_MIN 覆盖(默认 0.95*m_nominal≈1.961)。加这个开关是为了
    # **消融实验**:m_min 本身会影响下垂塌陷率(1.0→1.961 实测把耦合路径的塌陷率
    # 从 37.5% 降到 ~8.7%),所以要单独测"几何解耦"的效应时,必须把 m_min 固定在
    # 同一档、避免两个变量混在一起。用环境变量而不是改默认值,是防止实验后忘记恢复。
    # ⚠️ 改这个值会让 acados 重新生成 solver(lbx 变了),第一轮启动会慢约 30s。
    # --- 冷启动:去掉空机质量先验(2026-08-25,"去先验"下半程第 #5 项)---
    # 背景:通向 MHE 的空机质量先验有**三条**路,只堵一条等于没堵:
    #   ① x0_bar 的质量维种子 = m_nominal   → 软(能被数据推开),见 seed_from_thrust
    #   ② Q0[13] = 0.1 即 σ=3.16kg          → 相对 2.06kg 的量级本来就≈无先验
    #   ③ m_min = 0.95·m_nominal = 1.961     → **lbx 硬约束,数据推不开**
    # ③ 才是真正的强先验:它把答案硬圈在真值下方 5% 处。铁证是那些"m_est 钉在
    # 下界 1.961"的现象(N=40 全程钉、4m/s drop 后卡 138s 不回)——如果下界不是从
    # m_B 派生的,这些现象不会恰好长在那个位置。只改种子不动 m_min 是**假的
    # 零先验**。开关打开时②③同时卸掉;① 降级为纯初始猜测(见 mhe_node 里
    # _seed_mass_from_thrust 的调用处注释:initial guess ≠ prior)。
    # 默认**关**(本仓惯例:新机制先需 n>=8 实测才能改默认值)。
    no_mass_prior = os.environ.get(
        'MHE_NO_MASS_PRIOR', '0') not in ('0', '', 'false', 'False')
    # 无先验下界 [kg]:唯一职责是防 1/m 除零奇异(动力学里 a=T/m-g),
    # **不含任何 m_B 信息**——取 0.2kg,低于任何合理四旋翼机架。
    # 上界 m_max=5.0 本来就是硬编码常数、不从 m_B 派生,不需要改。
    m_min_free = float(os.environ.get('MHE_M_MIN_FREE', '0.2'))

    # ⚠️ MHE_M_MIN 显式覆盖优先级最高(消融实验要把下界固定在同一档时用)。
    m_min = float(os.environ['MHE_M_MIN']) if os.environ.get('MHE_M_MIN') \
        else (m_min_free if no_mass_prior else 0.95 * m_nominal)
    m_max = 5.0
    # m_B 的显式别名:m_nominal 这个名字只说"名义",没说是**空机**质量。全仓
    # 统一约定(2026-08-24):m_B=空机(已知常数)、m_P=载荷(未知)、m_T=m_B+m_P
    # (MHE 估的就是它,即状态向量第 14 维和 self.m_est)。
    m_B = m_nominal

    # --- 质量种子源(2026-08-25,"去先验"下半程第 #4 项)---
    # MHE 的到达代价先验均值 x0_bar 里质量那一维,第一次求解时用什么做种子。
    # False(历史行为):用 m_nominal —— 但这是**空机标称质量**,即一个先验。
    # True(默认):用电机转速反算的物理推力做悬停近似 m≈T_phys/g。
    #   T_phys = k_f·Σω²(见 mhe_node.MOTOR_CONSTANT)与任何质量假设、与 MHE
    #   自身状态都完全解耦,是这条链上唯一**零先验**的质量观测量。
    #
    # 为什么这个替换是安全的(三条,缺一不可):
    #   ① 时机:timer_cb 在 u_known(NMPC 的 u_opt)到达前直接 return,而 NMPC 是在
    #      grip_approach_z≈1.5m **空中准悬停**才接管的 → 第一次求解必定不在地面,
    #      T_phys/g 的悬停近似成立(地面支持力那个坑在这里够不着)。
    #   ② 权重:Q0 质量维只有 0.1,比物理状态软 4~5 个量级,种子偏一点也会被
    #      窗口内的测量迅速拉走;它影响的是"从哪出发",不是"收敛到哪"。
    #   ③ 兜底:拿不到电机数据(thrust_phys is None)或推力低于 seed_thrust_min
    #      时回退 m_nominal 并打 WARN —— 宁可退回先验,也不用一个不成立的近似。
    # 同一个种子逻辑此前**已经**在 5 连败重锚路径上用了(mhe_node 里那段"用
    # thrust_phys 而不是可能已错的 m_est 重新出发"),这里只是把它从兜底提升为
    # 常规路径,两处现已共用 _seed_mass_from_thrust()。
    # 2026-08-25 起默认关(当时无 SITL A/B)。
    # ★ 2026-09-15 用户拍板改为默认 **开**,与 seed_window_balance 一起:
    #   09-14 瞬时 T/g 版 A/B(seedthrust_ab)机制修掉但 drop 前坠机压线;
    #   09-15 窗口版 A/B(seedwindow_ab_manifest.csv,W4,预注册见
    #   重锚推力种子_预注册_20260914.md 附录 A):S1 10/10 有效 vs S0 8/10,
    #   figure-8 重锚 29→1 次、种子误差 17.1%→2.8%、最长重锚串 17→1,
    #   完成 10/10、释放后坠机 0,预注册改默认条件全满足(另一会话独立复核)。
    #   保留意见:S1 仅 1 次重锚事件,机制在开启臂基本未被激发。
    # ⚠️ 复现 09-15 之前的历史批次必须显式 MHE_SEED_FROM_THRUST=0 MHE_SEED_WINDOW_BALANCE=0。
    seed_from_thrust = os.environ.get(
        'MHE_SEED_FROM_THRUST', '1') not in ('0', '', 'false', 'False')
    # 低推力门控 [N]:低于此值认为"没在飞"(未起飞/异常),悬停近似不成立。
    # 与 mhe_node._update_c_xy_est 里那个 T<1.0 的门控同口径。
    seed_thrust_min = float(os.environ.get('MHE_SEED_THRUST_MIN', '1.0'))
    # 窗口力平衡种子(2026-09-14 实现,09-15 改默认开;需 seed_from_thrust 同时打开):
    # 瞬时 T/g 在 4m/s figure-8 中摆 −42%~+30%(seedthrust_ab 实测),重锚种子
    # 仍会被机动带偏。窗口内逐帧 m·(g+a_z) = T·cosθ(竖直力平衡,忽略竖直气动),
    # 求和后 m = Σ T_i·cosθ_i / (N·g + (vz_N − vz_0)/dt) —— Σa_z 按速度端点精确
    # 伸缩求和,不做数值微分;只用 T_phys 与 odom,仍与 MHE 自身状态解耦。
    seed_window_balance = os.environ.get(
        'MHE_SEED_WINDOW_BALANCE', '1') not in ('0', '', 'false', 'False')

    # --- 几何-质量耦合开关(2026-08-24)---
    # False(默认,与历史批次逐位一致):dJ/c_xy 由窗外算好当常参数喂进来。
    # True:model.p 的几何槛位改装 r_p=[rx,ry,rz],J 与 c 在模型内部由被估质量
    #   现算 → 质量进旋转动力学。见上面 n_geom 注释里的 CRLB 数字。
    # 用环境变量而不是改默认值:①历史批次/论文数字必须能逐位复现;②切换会改
    # model.f_expl_expr → acados 必须重新 codegen(codegen 目录名已带 -coupled
    # 后缀,两档各自独立缓存,不会互相覆盖,可以并发)。
    geom_coupled = os.environ.get('MHE_GEOM_COUPLED', '0') not in ('0', '', 'false', 'False')
    # 载荷自身惯量的回转半径平方 k_I [m²]:J_P = k_I·m_P·I₃(点质量模型取 0)。
    # gripper 场景 box 的 SDF 惯量是 0.00375·m_P·I₃(见 run_gripper_headless.sh),
    # 相对 dJ≈0.058 只有 2%,默认按点质量忽略;要更严格就设 MHE_PAYLOAD_KI。
    payload_ki = float(os.environ.get('MHE_PAYLOAD_KI', '0.0'))
    # m_P=m−m_B 的平滑正部宽度 [kg]:m 的下界 m_min=0.95·m_B 允许 m<m_B(留标定
    # 裕度),此时 m_P<0 会让 μ<0、进而 J_xx+dJ 变负 → 1/J 爆掉。用
    # m_P⁺=½(Δ+√(Δ²+ε²)) 做光滑正部,既保证 J≻0 又处处可微(硬 max 会让
    # Gauss-Newton 在 Δ=0 附近抖)。
    mp_pos_eps = 0.02

    # --- 一阶质量矩增广(2b,2026-08-24)---
    # 动机(实测驱动,不是设计洁癖):geom_coupled + geom_release_mode='self' 实测
    # 能让几何自行熄灭、不需要告知 drop,**但 drop 后 m_est 系统性偏低 −2.81%、
    # 55 个采样里 14 个贴在 m_min 上**。机理:杆臂 r_p 还在模型里,要让幽灵偏心
    # 消失,优化器唯一的办法是把 m_P⁺ 压到 0 → 把 m 压到 m_B 以下 → 一路探到下界。
    # 也就是说"载荷在不在"这个信息被硬编码进了质量,两者纠缠。
    #
    # 修法:把**一阶质量矩** s=m_P·r_xy [kg·m] 直接增广成状态(2 维)。于是
    #   c_xy = s/m_T                      ← 不再需要 r_xy,也不含 m_P 作除数
    #   非对角惯量 = (m_B/m_T)·s·r_z      ← m_P 恰好约掉
    #   μ·r_z² 项 = (m_B·m_P/m_T)·r_z²    ← 只需弱先验 r_z(实测 ±33% 对质量零影响)
    #   O(r_xy²) 项 = (m_B/(m_T·(m_P+δ)))·s_i·s_j  ← 占 dJ 仅 2.2%,用 δ 正则化
    # 好处有两条,都对得上已有实测:
    #   ①drop 后 s 自己趋零(它有独立观测 τ_phys/T,B.3 验过 <2%),m 不再被逼向下界;
    #   ②消掉"转动通路辨识的是乘积 m_P·r_xy、r_xy 先验错 20% → 质量偏 3.5%"这个风险
    #     ——现在那个乘积本身就是被估量,不需要先验。
    # s 的动力学 ṡ=0(窗口内常数,与 m 同待遇),不加过程噪声;要不要给它/给 m 加
    # 随机游走(文献 NeuroMHE/DMHE 的做法)是**另一件事**(2a),不在这里做。
    # 一阶质量矩是无事件主线的必要状态:没有它,优化器只能通过把 m 压到下界来
    # 消除 drop 后的幽灵偏心。显式设 0 仍可复现实验旧档。
    estimate_moment = os.environ.get('MHE_ESTIMATE_MOMENT', '1') not in ('0', '', 'false', 'False')
    ns = 2 if estimate_moment else 0     # 一阶质量矩 s=[s_x,s_y]
    # r_z 弱先验 [m](载荷挂在机体下方的深度)。只进 μ·r_z² 与非对角项,实测
    # ±33% 误差对质量估计零影响,所以给个档位标称值就够,不需要测。
    rz_prior = float(os.environ.get('MHE_RZ_PRIOR', '-0.47'))
    # --- A 项(μ·r_z² 对角惯量增量)的质量依赖开关(2026-09-02)---
    # 动机(离线实测):moment 档把 c_xy 从 m 解耦之后,幽灵几何**还留着第二个杠杆**
    # —— A=μ·r_z²=(m_B·m_P/m_T)·r_z² 仍以 m_P 为线性因子。drop 后模型里残留杆臂
    # 产生的幽灵滚转角加速度 ≈ c_y·T/J_xx(m):分子有界(|c_xy|≤r_xy),分母随 m_P
    # 线性涨(r_z²=0.22 是 J_xx=0.0142 的 15 倍量级),于是**抬高 m 能把幽灵力矩
    # 稀释掉**。离线 0.3kg self 档实测 drop 后 m̂ 爬到 3.17kg(+53%)并停住,宁可
    # 吃平动通路 −3.4m/s² 的矛盾也要压 ω 残差 —— 与 SITL 里往下撞 m_min 是同一
    # 个病的两个出口:m 被拿去当几何旋钮。
    #   'coupled'(默认,历史行为):A 随被估 m_P 变化。
    #   'const' :A 由**载荷包线上界**算成常数(与 2026-08-26 去先验改造同一口径
    #            ——包线是机架规格,不是任务信息),∂A/∂m≡0,杠杆消失。
    #   'zero'  :A≡0(纯诊断用,量"A 项到底贡献多少")。
    #   'frozen':A 由**上一窗口解出的 m̂**(经 geom[0] 当参数喂进来)现算。窗口内
    #            ∂A/∂m≡0 → 杠杆同样断掉,但 A 的**数值**仍跟着真实质量走,不像
    #            'const' 那样被包线钉死(0.15kg 载荷配 0.3kg 包线 = A 过估 2×)。
    #            动机:coupled 档 94% 的质量信息来自转动通路,'const' 把 A 冻在
    #            错误值上,SITL smoke 实测带载 m̂ 偏到 −3.58%(现状 +0.4%),还连带
    #            把质量域释放判据推成误触发(早于真 drop 50s)。
    # 无事件主线默认 frozen:窗口内切断 m->A 的“惯量旋钮”,窗口间仍用上一拍
    # m_est 连续刷新 A。合成 attach/drop 回放中 coupled 会在 drop 后把 m 推高
    # 53%,frozen 则回到空机 -0.34%、s/dJ 同步衰减且无触界。
    moment_a_mode = os.environ.get('MHE_MOMENT_A_MODE', 'frozen')
    # 载荷质量包线上界 [kg](机架规格,非任务信息)。只在 moment_a_mode='const'
    # 下用来算那个常数 A;口径与 gripper 侧 grip_payload_envelope 一致。
    mp_envelope = float(os.environ.get('MHE_MP_ENVELOPE', '0.3'))
    # s 的箱约束:|s| <= m_P_max * r_max。给宽松值,只防优化器跑飞。
    s_abs_max = float(os.environ.get('MHE_S_ABS_MAX', '1.5'))
    # O(r_xy²) 项分母的正则化 [kg],防 m_P→0 时 0/0。
    mp_div_eps = 0.02

    # --- 测量噪声标准差(用于标定 R 权重,也用于独立测试脚本生成合成噪声) ---
    # 数量级参照 PX4 EKF2 融合 VIO/GPS 后典型的状态估计精度,不是实测值。
    std_pos   = 0.02   # m
    std_vel   = 0.05   # m/s
    std_quat  = 0.01   # 四元数分量(约 1.1°姿态误差)
    std_omega = 0.02   # rad/s

    # --- R: 测量噪声权重(Bryson's rule: 1/标准差^2,跟 acados_params.py 同一套
    #     "无量纲化"逻辑,只是这里的"误差量级"换成了传感器噪声标准差而不是
    #     跟踪误差容许量) ---
    R = np.diag([
        1/std_pos**2,   1/std_pos**2,   1/std_pos**2,
        1/std_vel**2,   1/std_vel**2,   1/std_vel**2,
        1/std_quat**2,  1/std_quat**2,  1/std_quat**2,  1/std_quat**2,
        1/std_omega**2, 1/std_omega**2, 1/std_omega**2,
    ])

    # --- Q: 过程噪声权重(13维,只作用在物理状态上) ---
    # 故意比 R 更"紧"(权重更大):这套 MHE 要的不是"平滑掉噪声"，而是"严格信任
    # 动力学方程本身(除了质量未知之外,模型结构是对的)，逼着优化器只能靠调整
    # 质量这个唯一自由参数去解释观测到的加速度,而不是含糊地把误差推给过程噪声。
    # omega 这一块松一档:角速度动力学里有陀螺耦合项(om x J*om),数值上比
    # 平动动力学更敏感,留一点余地避免病态。
    Q = np.diag([
        1e4, 1e4, 1e4,        # pos
        1e4, 1e4, 1e4,        # vel —— 质量主要通过这里的 1/m*T 影响,这一块的紧
                                # 程度直接决定"模型失配被归因到质量"的力度
        1e5, 1e5, 1e5, 1e5,   # quat
        1e3, 1e3, 1e3,        # omega
    ])

    # --- Q0: 到达代价(arrival cost)权重,14维(13个物理状态+质量) ---
    # 物理状态部分跟 Q 同量级(对窗口起点的先验给中等强度的锚定);质量这一维
    # 故意给得很软(0.1,比物理状态部分小 4-5 个数量级)——这是让窗口滑动式的
    # "质量阶跃"(抓/放包裹)能在大约一个窗口长度内重新收敛的关键旋钮,呼应
    # 技术路线笔记第7节"可加大 Q 或弱化 arrival cost 来加速重新收敛"。
    # arrival mean(先验均值)在每一步滑动后用上一次窗口的 x[1] 估计值更新
    # (shift-forward,跟 acados_nmpc_node.py 里 NMPC 的 warm-start shift 是同一套
    # 思路),不是固定不变的——这是一种简化的"平滑式"递归更新,不是教科书上严格
    # 传播协方差的滤波式 arrival cost,留作以后需要更高精度时再升级。
    Q0 = np.diag([
        1e3, 1e3, 1e3,
        1e3, 1e3, 1e3,
        1e4, 1e4, 1e4, 1e4,
        1e2, 1e2, 1e2,
        # 质量:默认 0.1(σ=3.16kg)已经是极软的锚;no_mass_prior 时降到 1e-6
        # (σ=1000kg)= 数值上的零先验。**不直接写 0**:Q0 全零会让质量维在激励
        # 不足的窗口里丢掉 Hessian 的正定性(参考 2a 参数随机游走那 472 次病态失败),
        # 1e-6 既不携带信息又保住数值健康。
        (1e-6 if no_mass_prior else 0.1),
    ])
    if estimate_moment:
        # s 的到达代价 σ_s [kg·m]。默认 0.1 = 很弱的锚。
        # ⚠️ 2026-09-02 n=8 A/B 实测:0.15kg/ecc0.10 工况下带载 |s| 真值只有
        # 0.0060,σ_s=0.1 是它的 17 倍 = 实质无先验,结果 s 估到 0.0236(**高估 4×**)
        # —— s 在吸收本不属于它的残差(未建模力矩/推力标定),顺带把 m̂ 拖低 3.83%
        # (8/8 同向 p=0.0078)。收紧它是想把被 s 抢走的信息还给 m。
        # ⚠️ 收太紧的风险在另一头:drop 后 s 要能自由归零,锚太强会拖慢熄灭
        #    → 触界可能回来。这是本旋钮的取舍两端,要一起看。
        sigma_s0 = float(os.environ.get('MHE_SIGMA_S0', '0.1'))
        Q0 = np.diag(np.concatenate([
            np.diag(Q0), [1/sigma_s0**2, 1/sigma_s0**2]]))

    nx_aug = nx + nm + ns   # 14(legacy) 或 16(+s_xy)
    nw = nx + (nm + ns if param_noise else 0)
    if param_noise:
        # 参数随机游走的强度。1/σ² 形式:σ_m 是"每秒允许的质量漂移量"[kg/s],
        # σ_s 同理 [kg·m/s]。取值理由:一次 attach/drop 是 0.3kg 的阶跃,要在
        # ~0.3s 内跟上就需要 σ_m ≈ 1 kg/s;给 0.5 留一点余量不至于噪声期漂太狠。
        # ⚠️ 这个值直接换掉"稳态方差 vs 突变跟踪速度"的工作点,是 2a 的主旋钮。
        sigma_m_dot = float(os.environ.get('MHE_SIGMA_M_DOT', '0.5'))
        sigma_s_dot = float(os.environ.get('MHE_SIGMA_S_DOT', '0.05'))
        _qp = [1 / sigma_m_dot ** 2] * nm + [1 / sigma_s_dot ** 2] * ns
        Q = np.diag(np.concatenate([np.diag(Q), _qp]))


p = MHEParams()
