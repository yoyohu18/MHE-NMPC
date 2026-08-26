#!/usr/bin/env python3
# 机动门控验证(2026-08-25):机动期锚紧 Q0 质量维,悬停期名义。不需 acados/ROS。
import math
import numpy as np
from scipy.linalg import block_diag

from offboard_test_acados import mhe_node as mn
from offboard_test_acados.mhe_params import p as mhe_p


def test_q0_mass_index_is_right():
    """W0=block_diag(R,Q,Q0) 里质量那一维的全局下标必须落在 Q0 块内、
    且等于 Q0 自己的质量维取值 —— 下标算错会静默地去锚一个物理状态。"""
    W0 = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0)
    idx = mhe_p.nx + mhe_p.nw + mhe_p.nx
    assert W0.shape[0] == mhe_p.nx + mhe_p.nw + mhe_p.nx_aug
    assert abs(W0[idx, idx] - mhe_p.Q0[mhe_p.nx, mhe_p.nx]) < 1e-15, \
        f'下标 {idx} 处是 {W0[idx,idx]}, Q0 质量维是 {mhe_p.Q0[mhe_p.nx,mhe_p.nx]}'
    # 相邻位置必须不同(证明确实定位到了那一维而不是碰巧)
    assert W0[idx-1, idx-1] != W0[idx, idx], '下标可能偏了一位'
    print(f'[1] Q0 质量维下标 OK (idx={idx}, 值={W0[idx,idx]:.3g}, '
          f'矩阵 {W0.shape[0]}×{W0.shape[0]})')


class _S:
    def __init__(self, om=0.0, v=0.0, **kw):
        self.maneuver_omega_thresh = 0.15
        self.maneuver_vel_thresh = 0.20
        self.maneuver_q0_cap = 1.0e4
        self.maneuver_exponent = 2.0
        self._mg_gain_applied = None
        self._mg_frames_gated = 0
        self._mg_frames_total = 0
        self.x_meas = np.zeros(13)
        self.x_meas[3:6] = [v, 0, 0]
        self.x_meas[10:13] = [om, 0, 0]
        self.sets = []
        self.__dict__.update(kw)
        self.solver = self

    def cost_set(self, stage, key, W):
        self.sets.append((stage, key, W.copy()))

    def get_logger(self):
        class _L:
            info = warn = staticmethod(lambda m: None)
        return _L()


_lvl = mn.MHENode._maneuver_level
_gate = mn.MHENode._apply_maneuver_gate
_S._maneuver_level = _lvl
idx = mhe_p.nx + mhe_p.nw + mhe_p.nx
Q0m = mhe_p.Q0[mhe_p.nx, mhe_p.nx]


def test_hover_is_nominal():
    """悬停(lvl<=1)必须写名义值 —— 与关闭开关时逐位相同。"""
    s = _S(om=0.05, v=0.05)
    assert _lvl(s) < 1.0
    _gate(s, False)
    assert len(s.sets) == 1
    assert abs(s.sets[0][2][idx, idx] - Q0m) < 1e-15, '悬停期 Q0 不该被改'
    assert s._mg_frames_gated == 0
    print(f'[2] 悬停 = 名义权重 OK (lvl={_lvl(s):.2f}, Q0[m]={Q0m:.3g})')


def test_maneuver_tightens_monotonically():
    """机动越猛锚越紧,且封顶。"""
    gains = []
    for om in (0.15, 0.3, 0.6, 1.2, 3.0, 30.0):
        s = _S(om=om)
        _gate(s, False)
        gains.append(s.sets[0][2][idx, idx] / Q0m)
    assert all(b >= a - 1e-9 for a, b in zip(gains, gains[1:])), gains
    assert gains[0] == 1.0, '恰在阈值处应为名义'
    assert gains[-1] <= s.maneuver_q0_cap * 1.26, f'未封顶: {gains[-1]:.1e}'
    assert gains[3] > 10, f'|ω|=8倍阈值时锚紧不足: {gains[3]:.1f}'
    print(f'[3] 机动锚紧单调+封顶 OK (倍数 {[f"{g:.3g}" for g in gains]})')


def test_quantization_throttles_cost_set():
    """量化到 1.25 的对数格:平滑变化的机动度不该每帧都 cost_set。"""
    s = _S()
    n_written = 0
    for k in range(200):
        s.x_meas[10] = 0.15 + 0.30 * k / 200.0     # 平滑爬升
        before = len(s.sets)
        _gate(s, False)
        n_written += len(s.sets) - before
    assert n_written <= 12, f'200 帧写了 {n_written} 次,量化失效'
    print(f'[4] 量化节流 OK (200 帧平滑变化只写 {n_written} 次)')


def test_transition_yields():
    """事件过渡期必须让位,一次都不能写(否则与事件降权互相覆盖)。"""
    s = _S(om=5.0)
    _gate(s, True)
    assert s.sets == [], '过渡期不该动 stage-0 的 W'
    assert s._mg_frames_total == 1, '统计仍应计数'
    print('[5] 事件过渡期让位 OK')


if __name__ == '__main__':
    test_q0_mass_index_is_right()
    test_hover_is_nominal()
    test_maneuver_tightens_monotonically()
    test_quantization_throttles_cost_set()
    test_transition_yields()
    print('\nall passed')
