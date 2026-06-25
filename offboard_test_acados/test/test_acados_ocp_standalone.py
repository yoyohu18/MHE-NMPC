#!/usr/bin/env python3
# 阶段 A 验证:不需要 ROS2/PX4/Gazebo,只测 acados 求解器本身对不对。
# 运行方式: source install/setup.bash 之后直接 python3 -m pytest 这个文件,
# 或者 python3 test_acados_ocp_standalone.py 直接跑(文件末尾有 __main__ 入口)。
#
# 运行前需要:
#   export ACADOS_SOURCE_DIR=/home/clear/acados
#   export LD_LIBRARY_PATH=/home/clear/acados/lib:$LD_LIBRARY_PATH

import os
import time

import numpy as np

os.environ.setdefault('ACADOS_SOURCE_DIR', '/home/clear/acados')
os.environ['LD_LIBRARY_PATH'] = (
    '/home/clear/acados/lib:' + os.environ.get('LD_LIBRARY_PATH', ''))

from offboard_test_acados.acados_params import p  # noqa: E402
from offboard_test_acados.acados_solver_builder import (  # noqa: E402
    ensure_acados_ocp_solver,
)

HOVER_X = np.array([0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def _set_reference(solver, xr):
    for i in range(p.N + 1):
        solver.set(i, 'p', xr)


def _solve_to_convergence(solver, max_iters=20):
    """MERIT_BACKTRACKING 线搜索每次只走一部分步长,参考点偏移大的时候一次
    solve() 调用未必能在 nlp_solver_max_iter 以内完全收敛——跟真实 NMPC 循环
    一样,靠多次调用、每次复用上一次的 warm-start 逐步逼近,而不是要求单次
    调用就给出最终解。"""
    status = None
    for _ in range(max_iters):
        status = solver.solve()
        if status == 0:
            return status
    return status


def _seed_initial_guess(solver, x_init):
    """acados 默认的初始迭代值(没设的话是它内部的默认值,不保证是合法的单位
    四元数)对 SQP 不友好,容易卡在 ACADOS_MAXITER 不收敛——原来 CasADi/IPOPT
    版本里 X_init/U_init 也是显式拿悬停状态/悬停推力去铺满整个时域初始化的,这里
    照样做,不能省。"""
    for i in range(p.N):
        solver.set(i, 'x', x_init)
        solver.set(i, 'u', p.u_hover)
    solver.set(p.N, 'x', x_init)


def test_converges_to_hover_thrust():
    """从悬停状态、悬停参考开始解,应该收敛到 u ≈ u_hover,不偏不倚。"""
    solver = ensure_acados_ocp_solver()
    _seed_initial_guess(solver, HOVER_X)
    solver.set(0, 'lbx', HOVER_X)
    solver.set(0, 'ubx', HOVER_X)
    _set_reference(solver, HOVER_X)

    last_u = None
    for _ in range(20):
        status = solver.solve()
        assert status == 0, f'solver failed with status {status}'
        last_u = solver.get(0, 'u')

    assert np.allclose(last_u, p.u_hover, atol=1e-3), (
        f'expected u≈{p.u_hover}, got {last_u}')


def test_thrust_saturates_at_tmax():
    """喂一个远高于当前位置的参考(逼着它拼命爬升),u[0] 必须卡在 Tmax,
    不能超过——这条专门用来抓"忘了设 idxbu 导致约束形同虚设"的情况。"""
    solver = ensure_acados_ocp_solver()
    _seed_initial_guess(solver, HOVER_X)
    solver.set(0, 'lbx', HOVER_X)
    solver.set(0, 'ubx', HOVER_X)

    far_above = HOVER_X.copy()
    far_above[2] += 1000.0  # 参考高度比当前高 1000m,逼近的合理推力需求远超 Tmax
    _set_reference(solver, far_above)

    status = _solve_to_convergence(solver)
    assert status == 0, f'solver failed with status {status}'
    u_opt = solver.get(0, 'u')

    assert u_opt[0] <= p.Tmax + 1e-6, f'thrust {u_opt[0]} exceeds Tmax={p.Tmax}'
    assert u_opt[0] > p.Tmax - 1e-3, (
        f'expected thrust to saturate near Tmax={p.Tmax}, got {u_opt[0]} '
        f'(约束可能没生效,检查 idxbu/lbu/ubu)')


def test_quaternion_norm_drift_within_tolerance():
    """acados 的 ERK 积分器不会像原来手写的 RK4 那样每步重新归一化四元数,
    这里实测一段水平时域上的漂移量,确认在可接受范围内(不是假设,是测出来的)。"""
    solver = ensure_acados_ocp_solver()
    _seed_initial_guess(solver, HOVER_X)
    solver.set(0, 'lbx', HOVER_X)
    solver.set(0, 'ubx', HOVER_X)

    # 参考点稍微偏一点,逼着它产生非零的姿态/角速度指令,而不是停在零输入的退化情况
    moved_ref = HOVER_X.copy()
    moved_ref[0] = 1.0
    _set_reference(solver, moved_ref)

    status = _solve_to_convergence(solver)
    assert status == 0, f'solver failed with status {status}'

    max_drift = 0.0
    for i in range(p.N + 1):
        x_i = solver.get(i, 'x')
        q_norm = np.linalg.norm(x_i[6:10])
        max_drift = max(max_drift, abs(q_norm - 1.0))

    assert max_drift < 1e-3, (
        f'四元数模长漂移 {max_drift:.2e} 超过 1e-3,需要考虑加 con_h_expr 模长约束')


def test_solver_cache_hit_is_fast():
    """配置没变的情况下,第二次构建求解器应该走缓存命中,不重新生成/编译 C 代码,
    耗时应该在 1 秒以内(对照第一次跑这个文件时的 10-30 秒编译)。"""
    ensure_acados_ocp_solver()  # 确保第一次该有的产物已经生成过
    t0 = time.time()
    ensure_acados_ocp_solver()
    elapsed = time.time() - t0
    assert elapsed < 1.0, f'cache hit took {elapsed:.2f}s, expected <1s'


if __name__ == '__main__':
    test_converges_to_hover_thrust()
    print('[PASS] test_converges_to_hover_thrust')
    test_thrust_saturates_at_tmax()
    print('[PASS] test_thrust_saturates_at_tmax')
    test_quaternion_norm_drift_within_tolerance()
    print('[PASS] test_quaternion_norm_drift_within_tolerance')
    test_solver_cache_hit_is_fast()
    print('[PASS] test_solver_cache_hit_is_fast')
    print('\nAll Stage A standalone checks passed.')
