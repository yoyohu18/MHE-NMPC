#!/usr/bin/env python3
# MHE 事件触发的参数化权重时间表(运行时组件)。
#
# 【2026-07-31 瘦身】本文件原含 M1 阶段的 CEM 训练场(纯 numpy 四旋翼 + 高度 PD
# + acados MHE 在环的廉价评估环境,约 200 行)。CEM 学出的 θ* 经 2×2 析因消融
# (solve@20Hz,四臂 n=8/格,配对交错+拉丁方)与 10Hz 对照批证明**无可测收益**
# (详见记忆 cem-benefit-refuted),训练场与优化脚本已随之删除。
#
# 保留下面这两样,因为**它们是 M0 基线的运行时依赖,不是 CEM 的东西**:
#   ParametricWeightSchedule —— mhe_node 每次 solve 前调 apply() 实施事件降权
#   M0_THETA=[-4,0,0,0]      —— mhe_node 的 schedule_theta 默认值,即手工二值规则
#                                在这套参数化下的坐标
# 事件触发 MHE 的收益(入带 ~4×)来自这条手工规则本身,与 CEM 无关,不受影响。
#
# M0 定案(2026-07-03)给出的设计约束仍然成立,故保留原始说明:
# 二值降权规则撞上"入带速度 vs 过冲"的墙——快估计喂回控制器 → 控制器更激进 →
# 暂态加速度更暴烈 → 降权窗口只信少数新帧 → m_est 跟着暂态过冲。所以实施的不是
# 一个标量,是"过渡期的权重时间表"。

import numpy as np
from scipy.linalg import block_diag

from .mhe_params import p as mhe_p

class ParametricWeightSchedule:
    """把 M0 的二值降权推广成 4 参数的过渡期权重时间表。

    theta(全部作用在"确认后到事件滑出窗口 + extra 帧"的过渡期内):
      theta[0] log10 事件前 stage 的 R/Q 整体缩放      (M0 规则 = -4)
      theta[1] log10 事件后 stage 的 R vel 块缩放      (M0 规则 = 0;
               调小 = 过渡期少信剧烈暂态的速度测量 → 压过冲的主旋钮)
      theta[2] 过渡期在事件滑出窗口后再延长的帧数       (M0 规则 = 0)
      theta[3] log10 过渡期 Q0 质量锚缩放              (M0 规则 = 0;
               调大 = 锚住上一窗口质量估计 → 阻尼,但太大拖收敛)

    M0 基线:theta = [-4, 0, 0, 0]。语义与 EventWeightScheduler 一致:
    notify_event(第一个事件后测量帧序号) + 每次 solve 前 apply(solver, fc)。
    """

    # 与 mhe_params 的块索引对应
    _VEL = slice(3, 6)

    def __init__(self, theta):
        self.theta = np.asarray(theta, dtype=float)
        d_pre = 10.0 ** self.theta[0]
        r_vel = 10.0 ** self.theta[1]
        self.extra = int(round(self.theta[2]))
        q0m = 10.0 ** self.theta[3]

        R_post = mhe_p.R.copy()
        R_post[self._VEL, self._VEL] *= r_vel
        Q0_t = mhe_p.Q0.copy()
        Q0_t[13, 13] *= q0m

        self.W_nom = block_diag(mhe_p.R, mhe_p.Q)
        self.W0_nom = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0)
        self.W_pre = block_diag(d_pre * mhe_p.R, d_pre * mhe_p.Q)
        self.W_post = block_diag(R_post, mhe_p.Q)           # 过渡期的事件后 stage
        self.W0_pre = block_diag(d_pre * mhe_p.R, d_pre * mhe_p.Q, Q0_t)
        self.W0_post = block_diag(R_post, mhe_p.Q, Q0_t)

        self.event_frame = None
        self._dirty = False

    def notify_event(self, next_frame):
        self.event_frame = int(next_frame)

    def apply(self, solver, fc):
        N = mhe_p.N
        if self.event_frame is None:
            return False
        n_pre = N - (fc - self.event_frame)
        if n_pre >= N:
            return True  # 代价里还没有事件后测量,保持名义(同 EventWeightScheduler)
        if n_pre <= -self.extra:
            if self._dirty:
                solver.cost_set(0, 'W', self.W0_nom)
                for j in range(1, N):
                    solver.cost_set(j, 'W', self.W_nom)
                self._dirty = False
            self.event_frame = None
            return False
        solver.cost_set(0, 'W', self.W0_pre if n_pre > 0 else self.W0_post)
        for j in range(1, N):
            solver.cost_set(j, 'W', self.W_pre if j < n_pre else self.W_post)
        self._dirty = True
        return True


M0_THETA = np.array([-4.0, 0.0, 0.0, 0.0])  # 二值规则在此参数化下的坐标
