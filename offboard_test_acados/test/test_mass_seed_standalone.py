#!/usr/bin/env python3
# #4「去先验」:MHE 质量种子从 m_nominal 换成 T_phys/g 的单元验证(2026-08-25)。
# 不需要 ROS2/acados —— _seed_mass_from_thrust 只依赖 self.thrust_phys 和
# mhe_params,用一个最小替身把它从 MHENode 上摘下来单独测。
#
# 运行: python3 -m pytest test/test_mass_seed_standalone.py -q
#   或: python3 test/test_mass_seed_standalone.py

import math
import numpy as np

from offboard_test_acados.mhe_params import p as mhe_p
from offboard_test_acados.mhe_node import MHENode


class _Log:
    def __init__(self): self.warns = []
    def warn(self, m): self.warns.append(m)
    def info(self, m): pass


class _Stub:
    """只带 thrust_phys + logger 的替身,借用 MHENode 的未绑定方法。

    ⚠️ mhe_p.seed_from_thrust 默认是 **关**(2026-08-25 回退,见 mhe_params 注释),
    所以除 test_switch_off_restores_legacy 外,每个用例都要先把它打开 —— seed()
    封装了这件事,免得测到的其实是关闭分支。"""
    def __init__(self, T):
        self.thrust_phys = T
        self._log = _Log()
    def get_logger(self): return self._log

    def seed(self, why, enabled=True):
        orig = mhe_p.seed_from_thrust
        try:
            mhe_p.seed_from_thrust = enabled
            return MHENode._seed_mass_from_thrust(self, why)
        finally:
            mhe_p.seed_from_thrust = orig


def test_hover_thrust_gives_true_mass():
    """悬停推力 → 质量,零先验路径。2.364kg(空机+0.3载荷)对应 T=23.19N。"""
    m_true = mhe_p.m_B + 0.3
    s = _Stub(m_true * mhe_p.g)
    m, from_thrust = s.seed('test')
    assert from_thrust
    assert abs(m - m_true) < 1e-9, m
    assert not s._log.warns


def test_empty_frame_thrust():
    """空机悬停推力 → 空机质量(而不是因为 clip 被顶到 m_min)。"""
    s = _Stub(mhe_p.m_B * mhe_p.g)
    m, from_thrust = s.seed('test')
    assert from_thrust and abs(m - mhe_p.m_B) < 1e-9, m


def test_clip_to_bounds():
    """离谱推力被 clip 进 [m_min, m_max],不会把 solver 种到不可行区。"""
    assert _Stub(500.0).seed('t')[0] == mhe_p.m_max
    assert _Stub(5.0).seed('t')[0] == mhe_p.m_min      # 5N/g=0.51kg < m_min


def test_fallback_when_no_motor_data():
    """拿不到电机数据 → 回退 m_nominal 并 WARN(宁可退回先验,不用不成立的近似)。"""
    s = _Stub(None)
    m, from_thrust = s.seed('test')
    assert not from_thrust and m == mhe_p.m_nominal
    assert len(s._log.warns) == 1 and 'falling back' in s._log.warns[0]


def test_fallback_on_ground():
    """地面上推力低于门控 → 回退,不把地面支持力当成"飞机很轻"。"""
    s = _Stub(0.5)                       # < seed_thrust_min=1.0N
    m, from_thrust = s.seed('test')
    assert not from_thrust and m == mhe_p.m_nominal
    assert len(s._log.warns) == 1


def test_fallback_on_nan():
    s = _Stub(float('nan'))
    assert s.seed('test') == (mhe_p.m_nominal, False)


def test_switch_off_restores_legacy():
    """开关关闭(= 当前默认)→ 逐位回到历史行为(m_nominal),不打 WARN。"""
    s = _Stub(2.364 * mhe_p.g)
    assert s.seed('test', enabled=False) == (mhe_p.m_nominal, False)
    assert not s._log.warns


def test_default_is_off():
    """把"默认关"这件事本身锁进测试:改默认必须同时改这里,不会悄悄漂移。
    改默认的前提是 n>=8 的 SITL A/B —— 见 mhe_params.seed_from_thrust 注释。"""
    assert mhe_p.seed_from_thrust is False


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn(); print(f'  PASS  {name}')
            except AssertionError as e:
                fails += 1; print(f'  FAIL  {name}: {e}')
    print('全部通过' if not fails else f'{fails} 个失败')
    raise SystemExit(1 if fails else 0)
