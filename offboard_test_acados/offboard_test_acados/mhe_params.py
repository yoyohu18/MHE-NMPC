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
    nm     = 1   # 待估参数: 质量 m
    nx_aug = nx + nm  # MHE 自己优化的增广状态维度 = 14
    nw     = nx  # 过程噪声维度,只加在 13 个物理状态上,质量这一维 m_dot=0(窗口内当常数)
    nu_known = 4  # 已知输入: 总推力 T + 力矩 tau_x,tau_y,tau_z(跟 NMPC 实际发出的指令一致,
                  # 通过 model.p 传入,不是被估计的量)
    n_geom = 3    # 已知几何: 吊挂惯量增量 dJ(1)+ 复合质心水平偏移 c_xy(2)。
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
    # 窗口跨度 N*dt=2.0s:比"看一眼就给结果"的瞬时辨识更抗噪声,又不会长到让一次
    # 质量阶跃(配送场景里的抓/放包裹)被压成跨度过长的过渡态。NMPC 已改为
    # 20Hz，但 MHE 仍独立保持 10Hz/2s 窗口；它在自己的定时器帧读取最新状态和
    # NMPC 输入，不要求两个求解器同频。
    N  = 20
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
    m_min = float(os.environ['MHE_M_MIN']) if os.environ.get('MHE_M_MIN') \
        else 0.95 * m_nominal   # ≈1.961 kg
    m_max = 5.0

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
        0.1,                   # 质量,软锚定
    ])


p = MHEParams()
