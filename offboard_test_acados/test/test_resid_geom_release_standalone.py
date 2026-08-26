#!/usr/bin/env python3
# 无外部 drop 信号时,MHE 能否自己释放载荷几何(2026-08-25)。不需 acados/ROS。
#
# 背景:mass_event_cb(外部信号)做三件事 —— 释放几何 / 清棘轮 / 触发降权;而
# 残差自检测这条路原本只做第三件。于是一旦把 drop 事件关掉(NMPC 侧
# drop_publish_mass_event=false),MHE 就会永远挂着已经不存在的载荷几何,继续拿
# 它给 om_dot 算 dJ/c_xy。geom_release_mode='self' 在 legacy 档是被明确拒绝的
# (几何幅值来自先验/棘轮,不随 m_est 熄灭),所以只能走"自检测到推力下降 →
# 自己释放"这条路。本模块验证方向判别与释放动作。

import numpy as np
from offboard_test_acados import mhe_node as mn


class _Sched:
    def __init__(self): self.event_frame = None; self.notified = []
    def notify_event(self, f): self.event_frame = f; self.notified.append(f)


class _S:
    def __init__(self, **kw):
        self.thrust_phys = 20.0
        self.resid_ema_alpha = 0.033
        self.confirm_thresh = 1.5
        self.resid_persist = 2
        self.resid_warmup = 3
        self.resid_settle_tol = 0.5
        self._resid_baseline = None
        self._resid_pending = 0
        self._resid_cross_frame = None
        self._resid_settled = False
        self._resid_stable = 0
        self.frames = 0
        self.scheduler = _Sched()
        self._payload_attached = True
        # 注:MHE 侧棘轮 _m_p_hat_ratchet 已随 2026-08-26 去先验改造移除,
        # 这里保留一个同名字段只是为了让老断言不至于 AttributeError;
        # 释放路径不再碰它。
        self._m_p_hat_ratchet = 0.30
        self.attach_offset = np.array([0.006, -0.093, -0.516])
        self.resid_release_geom = True
        # 2026-08-26 _residual_detect 新增的并行阶跃判据状态。这里显式**关掉**
        # 它:本模块测的是"慢基线那条路上的几何释放",两条判据混在一起会说不清
        # 是谁触发的。阶跃判据自己的验证在 test_step_detector_replay.py。
        self.resid_step_enable = False
        self.resid_step_half = 3
        self.resid_step_thresh = 1.0
        self.resid_step_persist = 2
        self.resid_step_release_thresh = 2.0
        self._tphys_hist = []
        self._step_pending = 0
        self.logs = []
        self.__dict__.update(kw)

    def get_logger(self):
        stub = self
        class _L:
            def info(self, m): stub.logs.append(m)
            def warn(self, m): stub.logs.append(m)
        return _L()


_det = mn.MHENode._residual_detect


def _warm(s, T=23.2, n=8):
    """把基线喂到 settled(带载稳态 23.2N)。"""
    for _ in range(n):
        s.thrust_phys = T; s.frames += 1; _det(s)
    assert s._resid_settled, '基线应已预热'


def test_drop_releases_geometry():
    """T_phys 台阶式下降(23.2→20.1,载荷没了)→ 自检测确认 → 释放几何。"""
    s = _S(); _warm(s)
    for _ in range(4):
        s.thrust_phys = 20.1; s.frames += 1; _det(s)
    assert s.scheduler.notified, '应当自检测到事件'
    assert s._payload_attached is False, '未释放 _payload_attached'
    assert s.attach_offset is None, '未丢弃 attach 几何'
    assert any('DROP' in m for m in s.logs), '日志未标出方向'
    assert any('自主释放载荷几何' in m for m in s.logs)
    print('[1] 推力下降 → 自主释放几何 OK')


def test_attach_does_not_release():
    """T_phys 台阶式上升(20.1→23.2,上货)→ 同样确认事件,但**绝不能**释放几何
    —— 那会把刚挂上的载荷几何立刻扔掉。"""
    s = _S(thrust_phys=20.1, _payload_attached=True)
    _warm(s, T=20.1)
    for _ in range(4):
        s.thrust_phys = 23.2; s.frames += 1; _det(s)
    assert s.scheduler.notified, '应当自检测到事件'
    assert s._payload_attached is True, 'attach 方向不该释放几何'
    assert s.attach_offset is not None
    assert any('ATTACH' in m for m in s.logs)
    print('[2] 推力上升 → 保留几何 OK')


def test_switch_off_keeps_legacy():
    """resid_release_geom=False 时逐字节回到旧行为:只降权,不碰几何。"""
    s = _S(resid_release_geom=False); _warm(s)
    for _ in range(4):
        s.thrust_phys = 20.1; s.frames += 1; _det(s)
    assert s.scheduler.notified, '降权链路仍应触发'
    assert s._payload_attached is True, '开关关闭时不该动几何'
    print('[3] 开关关闭 = legacy(只降权) OK')


def test_no_release_when_already_empty():
    """空载时的推力扰动不该反复"释放"(幂等):_payload_attached 已 False 就跳过。"""
    s = _S(_payload_attached=False, attach_offset=None)
    _warm(s, T=20.3)
    for _ in range(4):
        s.thrust_phys = 17.0; s.frames += 1; _det(s)
    assert not any('自主释放载荷几何' in m for m in s.logs), '空载不该再释放一次'
    print('[4] 空载幂等 OK')


def test_gust_pulse_does_not_trigger():
    """单帧脉冲(阵风)不该确认 —— 两段式确认的第二段就是防这个。"""
    s = _S(); _warm(s)
    s.thrust_phys = 20.1; s.frames += 1; _det(s)     # 只越阈 1 帧
    s.thrust_phys = 23.2; s.frames += 1; _det(s)     # 回落
    assert not s.scheduler.notified, '单帧脉冲不该确认事件'
    assert s._payload_attached is True
    print('[5] 单帧阵风不误触发 OK')


if __name__ == '__main__':
    test_drop_releases_geometry()
    test_attach_does_not_release()
    test_switch_off_keeps_legacy()
    test_no_release_when_already_empty()
    test_gust_pulse_does_not_trigger()
    print('\nall passed')
