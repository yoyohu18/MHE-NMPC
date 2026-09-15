#!/usr/bin/env python3
# resid_confirm_enable 开关(2026-09-14):只关慢基线确认判据,不碰其他路径。
#
# 背景:thresh=4.4N 的慢基线判据在 0.30kg 工况的 attach/drop 暂态会越阈,越阈后
# 有三个下游 —— 窗口降权、ATTACH 方向快速进 LOADED、DROP 方向投释放票。部署档
# 需要把它关掉,但不能用 event_trigger_enable=false:那个总开关在 timer_cb 里还
# 同时关掉 _update_release_residual()(释放判据的 fastB 信息源)。
#
# 验证:①关 → DROP/ATTACH 台阶都不降权、不改存在状态、不投票
#       ②开 → 历史行为(对照,确认测试输入确实会越阈)
#       ③关慢基线 + 开并行阶跃判据 → 阶跃判据照常工作(开关只管一支)
# 跑法: source ROS + install/setup.bash 后 python3 本文件

from test_resid_geom_release_standalone import _S, _det


def _feed(s, T, n):
    for _ in range(n):
        s.thrust_phys = T; s.frames += 1; _det(s)


def _step(s, T0, T1):
    _feed(s, T0, 12)   # 足够预热(resid_warmup=3)
    _feed(s, T1, 6)


def test_off_drop_silent():
    s = _S(resid_confirm_enable=False, resid_release_geom=True)
    _step(s, 23.2, 17.6)                      # -5.6N,远超 1.5N 阈值
    assert not s.scheduler.notified, '关闭后不应降权'
    assert not hasattr(s, '_residual_drop_evidence_until'), '关闭后不应投释放票'
    assert s._resid_baseline is None, '关闭后慢基线不应被播种'
    assert not any('self-detected' in m for m in s.logs)
    print('[1] 关:DROP 台阶零降权/零投票 OK')


def test_off_attach_silent():
    s = _S(resid_confirm_enable=False, _payload_present=False, _load_armed=False,
           thrust_phys=20.2)
    _step(s, 20.2, 25.2)                      # +5.0N
    assert not s.scheduler.notified
    assert s._payload_present is False, '关闭后不应经残差快速通道进 LOADED'
    print('[2] 关:ATTACH 台阶不进 LOADED OK')


def test_on_is_legacy_control():
    # 对照:同样输入在开的状态下必须触发,否则 [1][2] 的"静默"没有意义。
    # resid_release_geom=False 避开投票分支(stub 无 get_clock,既有缺陷,见说明)。
    s = _S(resid_confirm_enable=True, resid_release_geom=False)
    _step(s, 23.2, 17.6)
    assert s.scheduler.notified, '开启时同一台阶必须触发'
    assert any('self-detected' in m and 'DROP' in m for m in s.logs)
    print('[3] 开:同一输入照常触发(对照)OK')


def test_off_keeps_step_detector():
    s = _S(resid_confirm_enable=False, resid_release_geom=False,
           resid_step_enable=True, resid_step_thresh=0.6)
    _step(s, 23.2, 21.7)                      # -1.5N,双窗差分可见
    assert s.scheduler.notified, '慢基线关闭不应影响并行阶跃判据'
    assert any('[step-detect]' in m for m in s.logs)
    assert s._resid_baseline is None
    print('[4] 关慢基线 + 开 step:阶跃判据独立工作 OK')


if __name__ == '__main__':
    test_off_drop_silent()
    test_off_attach_silent()
    test_on_is_legacy_control()
    test_off_keeps_step_detector()
    print('\nall passed')
