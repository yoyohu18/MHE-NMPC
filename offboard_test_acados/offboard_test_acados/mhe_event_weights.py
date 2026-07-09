#!/usr/bin/env python3
# 事件触发的 MHE 权重调度器(M0,规则版;见 技术方案_MHE学习模块_20260703.md §4)。
#
# 要解决的问题:质量阶跃(抓/放包裹)后,滑动窗口里同时含阶跃前后的测量,而
# MHE 模型里质量这一维 m_dot=0 且没有过程噪声通道(mhe_model.py:61)——质量在
# 整个窗口内是刚性常数,阶跃前的旧测量帧以 R 的全权重把质量往旧值拽,优化器
# 只能给折中值,要等旧数据完全滑出窗口(N*dt=2s)才收敛。
#
# 为什么不是调 Q0 质量锚:它已经软到 0.1(mhe_params.py:94),对上面这个机制
# 根本不是瓶颈;瓶颈是"窗口内跨阶跃的旧测量 + m_dot=0 刚性"本身。
#
# 做法:抓/放是配送场景里的已知事件(夹爪自己发指令,见 技术路线笔记 §7)。
# 收到事件后,把窗口内仍属"事件前"的 stage 的 R/Q 块降权 deweight 倍——旧帧
# 的测量拟合和过程噪声惩罚都近乎免费,质量只由事件后的新帧决定,一两帧内就能
# 跳到新值;事件随窗口滑动完全滑出后,恢复名义权重。降权而不清缓冲区:不用等
# 窗口重新攒满,阶跃前的物理状态部分照常暖启动。
#
# 这里的逐 stage cost_set 注入机制与 M1(NN 出权重)复用同一条链路——M1 只是
# 把"事件前/后二值降权"换成网络输出的连续缩放。

import numpy as np
from scipy.linalg import block_diag

from .mhe_params import p as mhe_p


class EventWeightScheduler:
    """跟踪事件在滑动窗口里的位置,每次 solve 前把事件前 stage 降权。

    帧序号约定:调用方维护一个单调递增的"已入缓冲总帧数" frames;事件回调
    到达时用 notify_event(frames) 记下"第一个事件后测量帧"的全局序号(事件
    消息与下一帧测量之间最多差一个定时器周期 0.1s,偏差不超过一个 stage,
    对 2s 窗口无所谓)。之后每次 solve 前调 apply(solver, fc),fc = 窗口末端
    (最新一帧)的全局序号 = frames - 1。
    """

    def __init__(self, deweight: float = 1e-4):
        # deweight 取相对缩放而不是绝对置零:R/Q 名义量级 1e3~1e5,乘 1e-4 后
        # 仍是 0.1~10 的良态数字,不会让 QP 病态;又比事件后 stage 小 4 个数量
        # 级,足以让新帧完全主导质量。
        d = float(deweight)
        self.W_nom = block_diag(mhe_p.R, mhe_p.Q)
        self.W_pre = block_diag(d * mhe_p.R, d * mhe_p.Q)
        # stage 0 多带 Q0 到达代价:降权只动 R/Q 块,Q0 保持名义——物理状态的
        # 先验锚定照常(暖启动质量),质量维的锚本来就只有 0.1,不构成拖累。
        self.W0_nom = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0)
        self.W0_pre = block_diag(d * mhe_p.R, d * mhe_p.Q, mhe_p.Q0)

        self.event_frame = None  # 第一个事件后测量帧的全局序号
        self._dirty = False      # 求解器里还有 stage 停在降权状态,需恢复

    def notify_event(self, next_frame: int):
        self.event_frame = int(next_frame)

    def apply(self, solver, fc: int) -> bool:
        """每次 solve 之前调用。fc = 窗口末端帧的全局序号。
        返回 True 表示本窗口处于事件过渡期(供调用方打诊断日志)。"""
        N = mhe_p.N
        if self.event_frame is None:
            return False

        # 窗口测量 stage j 的全局序号 = fc-N+j,事件前 ⇔ j < n_pre
        n_pre = N - (fc - self.event_frame)

        if n_pre >= N:
            # 代价只覆盖 stage 0..N-1,窗口最新帧 y_N 不进代价(terminal
            # ny_e=0)——此刻代价里还没有任何事件后测量,全降权会让质量这一帧
            # 近乎不可观测(只剩 0.1 的软锚)。保持名义权重,等下一帧(第一个
            # 事件后测量滑进 stage N-1)再开始降权。
            return True

        if n_pre <= 0:
            # 事件已完全滑出窗口:恢复名义权重(一次),之后不再干预
            if self._dirty:
                solver.cost_set(0, 'W', self.W0_nom)
                for j in range(1, N):
                    solver.cost_set(j, 'W', self.W_nom)
                self._dirty = False
            self.event_frame = None
            return False

        # 事件仍在窗口内:stage 0..n_pre-1 属事件前,降权;其余名义。
        # 边界过渡那一步(stage n_pre-1 的过程噪声,连接阶跃前后状态、动力学
        # 本身就是错的)也在降权范围内,正好一并放掉。
        solver.cost_set(0, 'W', self.W0_pre)
        for j in range(1, N):
            solver.cost_set(j, 'W', self.W_pre if j < n_pre else self.W_nom)
        self._dirty = True
        return True
