#!/usr/bin/env python3
"""2026-09-04 主线批次暴露的三类 drop 确认缺陷的回归测试(不起 ROS)。

三类都来自 96 架次 A/B 批次的实测失败:
  1. 残余 s        —— drop 后 m 已回空机、s 停在幽灵偏心上,conf 被一票否决,
                      DROP 永远 complete 不了(7/48 轮,全在机动工况)。
  2. attach 失败   —— 载荷压根没上机,conf 全程 1.0,latch 在起飞后就被消耗,
                      真发 drop 指令时确认函数第一行直接 return(#19)。
  3. health 中断   —— confidence 的持续计数必须被不健康帧**打断**而不是暂停,
                      否则跨越估计器盲区拼出来的 hold 是假的。
"""

import dataclasses

import numpy as np

from offboard_test_acados import mhe_node as mn
from offboard_test_acados import acados_nmpc_node as mn_nmpc
from offboard_test_acados.mhe_params import p as mhe_p
from offboard_test_acados.payload_estimate import (
    PayloadEstimate, no_payload_confidence)


# ----------------------------------------------------------------- 1) 残余 s
class _Clock:
    def __init__(self, owner):
        self.owner = owner

    def now(self):
        class _Now:
            pass
        v = _Now()
        v.nanoseconds = int(self.owner.now_sec * 1e9)
        return v


class _Logger:
    def __init__(self):
        self.msgs = []

    def info(self, m):
        self.msgs.append(m)

    warn = error = info


class _MHEStub:
    """借 MHENode 的未绑定方法跑发布/释放这两条链,不碰 ROS。"""
    _release_payload = mn.MHENode._release_payload
    _publish_payload_estimate = mn.MHENode._publish_payload_estimate
    _payload_frame_health = mn.MHENode._payload_frame_health
    _payload_inputs_fresh = mn.MHENode._payload_inputs_fresh

    def __init__(self, s_est=(0.0175, 0.0)):
        self.now_sec = 100.0
        self.payload_input_fresh_sec = 0.30
        self.payload_solution_fresh_sec = 0.30
        self._last_odom_rx_sec = self._last_motor_rx_sec = 99.95
        self._last_u_rx_sec = self._last_solve_success_sec = 99.95
        self._last_solve_ok = True
        self.payload_output_tau = 0.30
        self._payload_conf_args = {}
        # drop 之后的实测状态:m 已回到空机,s 还停在幽灵偏心上
        self.m_est = mhe_p.m_B
        self.s_est = np.array(s_est, dtype=float)
        self.c_xy_est = np.array(s_est, dtype=float) / mhe_p.m_B
        self._c_xy_inited = True
        self._m_payload_out = 0.0
        self._s_out = np.array(s_est, dtype=float)
        self._no_payload_confidence = 0.0
        self._s_release_latched = False
        self._s_peak = 0.0
        self._s_low = 0
        # 2026-09-04 新增:armed latch 与 [s-decay] 日志的状态
        self._load_armed = True
        self._payload_present = True
        self._present_hi = 0
        self._present_lo = 0
        self.s_decay_log_frames = 0      # 单元测试里不打日志
        self._s_decay_n = 0
        self._s_out_prev = np.zeros(2)
        self._payload_attached = True
        self.attach_offset = np.array([0.0, 0.1, -0.47])
        self.published = []
        self._log = _Logger()

    def get_clock(self):
        return _Clock(self)

    def get_logger(self):
        return self._log

    @property
    def payload_estimate_pub(self):
        stub = self

        class _Pub:
            def publish(self, msg):
                stub.published.append(np.asarray(msg.data, dtype=float))
        return _Pub()

    @property
    def c_xy_est_pub(self):
        class _Pub:
            def publish(self, msg):
                pass
        return _Pub()


def _run(stub, n):
    for _ in range(n):
        stub._publish_payload_estimate()
    return PayloadEstimate.from_array(stub.published[-1])


def test_residual_s_blocks_confidence_before_release():
    """前提复现:光靠 m 回到空机,conf 仍被残余 s 否决。"""
    stub = _MHEStub()
    est = _run(stub, 200)
    assert est.m_total < mhe_p.m_B + 1e-3          # 质量已经回到空机
    assert np.linalg.norm(est.s_xy) > 0.006        # s 仍在 moment_zero 之上
    assert est.no_payload_confidence < 0.10        # 被一票否决


def test_release_drives_s_to_zero_continuously():
    """释放后 s 目标切零,conf 起得来,且对外没有阶跃。"""
    stub = _MHEStub()
    _run(stub, 5)
    s_before = np.linalg.norm(stub._s_out)
    stub._release_payload('test')
    assert stub._s_release_latched

    # 第一帧不能是阶跃:LPF 每帧最多走 alpha 比例
    stub._publish_payload_estimate()
    first = PayloadEstimate.from_array(stub.published[-1])
    step = s_before - np.linalg.norm(first.s_xy)
    alpha = 1.0 - np.exp(-mhe_p.dt / stub.payload_output_tau)
    assert 0.0 < step <= s_before * alpha * 1.05, '接口上出现了 s 阶跃'

    est = _run(stub, 300)
    assert np.linalg.norm(est.s_xy) < 1e-3
    assert est.no_payload_confidence > 0.90, est.no_payload_confidence


def test_reattach_restores_s_target():
    """重新带载后 s 的发布目标必须回到估计值,否则下一次任务是瞎的。"""
    stub = _MHEStub()
    stub._release_payload('test')
    stub._s_release_latched = False        # attach_event_cb 里做的事
    stub.s_est = np.array([0.02, 0.0])
    est = _run(stub, 300)
    assert np.linalg.norm(est.s_xy) > 0.015


# --------------------------------------------------- 2) attach 失败致 latch 消耗
class _NMPCStub:
    _confirm_no_payload_if_persistent = (
        mn_nmpc.AcadosNMPCNode._confirm_no_payload_if_persistent)
    _check_drop_unresolved = mn_nmpc.AcadosNMPCNode._check_drop_unresolved
    _payload_estimate_is_fresh = (
        mn_nmpc.AcadosNMPCNode._payload_estimate_is_fresh)

    def __init__(self):
        self._payload_estimate_received = True
        self._payload_estimate_rx_sec = 100.0
        self._payload_target = PayloadEstimate(
            m_total=2.06, s_xy=np.zeros(2), c_xy=np.zeros(2),
            dJ_diag=np.zeros(3), J_diag=np.ones(3),
            no_payload_confidence=1.0, healthy=True, solution_age_sec=0.0)
        self._payload_estimate_health_prev = True
        self.payload_estimate_fresh_sec = 0.30
        self.no_payload_confidence = 1.0
        self.no_payload_conf_threshold = 0.90
        self.no_payload_conf_hold_frames = 20
        self._no_payload_conf_frames = 0
        self._no_payload_latched = False
        self.control_mode = 'mhe'
        self.tau_lumped_enable = False
        self.gripper_mode = True
        self.grip_drop_pending = False
        self.grip_drop_done = False
        self.grip_dropped = False
        self.now_sec = 100.0
        # unresolved 路径(2026-09-05)
        self.drop_unresolved_timeout_sec = 12.0
        self.payload_unresolved = False
        self._drop_cmd_wall = None
        self.grip_dynamic_active = True
        self._log = _Logger()

    def get_clock(self):
        return _Clock(self)

    def get_logger(self):
        return self._log


def test_latch_consumed_before_drop_blocks_completion():
    """前提复现:attach 没成功时 latch 被提前吃掉,之后 drop 永不完成。"""
    node = _NMPCStub()
    for _ in range(40):
        node._confirm_no_payload_if_persistent()
    assert node._no_payload_latched                     # 空载期就 latch 了

    node.grip_drop_pending = True                       # 真 drop 指令来了
    for _ in range(80):
        node._confirm_no_payload_if_persistent()
    assert not node.grip_dropped, '这正是 #19 的失败:确认函数直接 return'


def test_unconditional_rearm_on_drop_command_completes():
    """修法:发释放指令时无条件复位 latch,drop 就能完成。"""
    node = _NMPCStub()
    for _ in range(40):
        node._confirm_no_payload_if_persistent()
    assert node._no_payload_latched

    node.grip_drop_pending = True
    node._no_payload_latched = False        # _grip_drop_phase 里新增的两行
    node._no_payload_conf_frames = 0
    for _ in range(node.no_payload_conf_hold_frames + 2):
        node._confirm_no_payload_if_persistent()
    assert node.grip_dropped and node.grip_drop_done
    assert not node.grip_drop_pending


def test_drop_phase_itself_rearms_the_latch():
    """不是模拟那两行,而是真跑 _grip_drop_phase —— 锁住改动本身。"""
    node = _NMPCStub()
    node._grip_drop_phase = (
        mn_nmpc.AcadosNMPCNode._grip_drop_phase.__get__(node))
    node.continuous_payload_estimates = True
    node.grip_drop_after_sec = 1.0
    node.grip_lift_after_sec = 0.0
    node.grip_lift_dur = 0.0
    node.attach_time = 0.0
    node.grip_drop_at_fig8_tip = False
    node.grip_dynamic_active = False
    node.drop_time = None
    node.enable_pub = type('P', (), {'publish': lambda self, m: None})()
    node._no_payload_latched = True          # 空载期已被消耗掉
    node._no_payload_conf_frames = 7

    node._grip_drop_phase(5.0)               # 过了 t_drop,发释放指令

    assert node.grip_drop_pending
    assert not node._no_payload_latched, '释放指令没有重新武装空载检测'
    assert node._no_payload_conf_frames == 0


# ------------------------------------------- 3) health 中断必须打断 hold 计数
def test_unhealthy_frame_breaks_confidence_hold():
    """不健康帧要**打断**持续计数,不是暂停 —— 否则跨盲区拼出来的 hold 是假的。"""
    node = _NMPCStub()
    node.grip_drop_pending = True
    hold = node.no_payload_conf_hold_frames

    for _ in range(hold - 1):               # 差一帧就达标
        node._confirm_no_payload_if_persistent()
    assert not node.grip_dropped

    node._payload_target = dataclasses.replace(   # frozen:换一帧,不是改一帧
        node._payload_target, healthy=False)       # 估计器瞎了一帧
    node._confirm_no_payload_if_persistent()
    assert node._no_payload_conf_frames == 0, '计数被暂停而不是打断'

    node._payload_target = dataclasses.replace(
        node._payload_target, healthy=True)
    for _ in range(hold - 1):               # 重新攒,仍差一帧
        node._confirm_no_payload_if_persistent()
    assert not node.grip_dropped
    node._confirm_no_payload_if_persistent()
    assert node.grip_dropped


def test_confidence_below_threshold_resets_counter():
    """conf 掉回阈值以下同样清零计数(载荷其实还在的情形)。"""
    node = _NMPCStub()
    node.grip_drop_pending = True
    for _ in range(node.no_payload_conf_hold_frames - 1):
        node._confirm_no_payload_if_persistent()
    node.no_payload_confidence = 0.10
    node._confirm_no_payload_if_persistent()
    assert node._no_payload_conf_frames == 0
    assert not node.grip_dropped


# ---------------------------------------------- 辅助:conf 判据本身的量纲检查
def test_confidence_vetoed_by_moment_alone():
    """质量为零但一阶矩不为零时 conf 必须是 0 —— 这条不能被"放宽"掉。"""
    assert no_payload_confidence(0.0, [0.0175, 0.0], [0.0, 0.0, 0.0]) < 1e-6
    assert no_payload_confidence(0.0, [0.0, 0.0], [0.0, 0.0, 0.0]) > 0.99


# ------------------------------------------- 4) 超时 = unresolved,不是"已卸载"
def test_timeout_goes_unresolved_not_empty():
    """★ 超时**不等于**已卸载:进 unresolved,且不得清模型/复位(2026-09-05)。

    释放判据现在强制要求 moment 证据,"ratio 不塌的 drop"是有意接受的漏检。
    此时飞机真实状态未知——猜"已卸载"会在载荷还挂着时按空机构型飞;继续按带载飞
    则可能带着幽灵偏心。两边都不猜:保持模型,退出机动转保守悬停。
    """
    node = _NMPCStub()
    node.no_payload_confidence = 0.30          # moment 未确认 => conf 上不去
    node._payload_target = dataclasses.replace(
        node._payload_target, no_payload_confidence=0.30)
    node._no_payload_latched = False
    node.grip_drop_pending = True
    node._drop_cmd_wall = node.now_sec

    node.now_sec += node.drop_unresolved_timeout_sec - 1.0
    node._payload_estimate_rx_sec = node.now_sec
    node._confirm_no_payload_if_persistent()
    assert not node.payload_unresolved, '未超时不该进 unresolved'
    assert node.grip_dynamic_active, '未超时不该退出机动'

    node.now_sec += 2.0
    node._payload_estimate_rx_sec = node.now_sec
    node._confirm_no_payload_if_persistent()
    assert node.payload_unresolved, '超时应进 unresolved'
    assert not node.grip_dynamic_active, 'unresolved 必须退出 figure-8 转悬停'
    assert not node.grip_dropped, 'unresolved 不得清模型'
    assert not node.grip_drop_done
    assert node.grip_drop_pending, 'pending 保持,交给上层处理'


def test_late_evidence_resolves_unresolved():
    """迟到的 moment 证据到达后,从 unresolved 恢复成正常卸载完成。"""
    node = _NMPCStub()
    node.no_payload_confidence = 0.30
    node._payload_target = dataclasses.replace(
        node._payload_target, no_payload_confidence=0.30)
    node._no_payload_latched = False
    node.grip_drop_pending = True
    node._drop_cmd_wall = node.now_sec
    node.now_sec += node.drop_unresolved_timeout_sec + 1.0
    node._payload_estimate_rx_sec = node.now_sec
    node._confirm_no_payload_if_persistent()
    assert node.payload_unresolved

    node.no_payload_confidence = 0.99
    node._payload_target = dataclasses.replace(
        node._payload_target, no_payload_confidence=0.99)
    for _ in range(node.no_payload_conf_hold_frames + 2):
        node.now_sec += 0.05
        node._payload_estimate_rx_sec = node.now_sec
        node._confirm_no_payload_if_persistent()
    assert not node.payload_unresolved, '证据到达应解除 unresolved'
    assert node.grip_dropped and node.grip_drop_done


if __name__ == '__main__':
    import sys
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'  PASS  {name}')
            except AssertionError as e:
                fails += 1
                print(f'  FAIL  {name}: {e}')
    print(f'\n{fails} failed')
    sys.exit(1 if fails else 0)
