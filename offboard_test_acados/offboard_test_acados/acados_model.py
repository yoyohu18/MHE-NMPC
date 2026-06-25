#!/usr/bin/env python3
# 把 offboard_test.nmpc_node 里已经验证过的动力学(build_dynamics)和代价残差
# (tracking_error_sym)包成一个 AcadosModel,不重新推导数学。
#
# 时变参考用 model.p(13维,跟 tracking_error_sym 的第二个参数 xr 对应)来传,
# 因为代价里的四元数误差项对参考值是非线性的(四元数乘法),没法写成简单的
# "y - yref" 形式,必须让参考值进 CasADi 表达式本身。

import casadi as cs
from acados_template import AcadosModel

from offboard_test.nmpc_node import build_dynamics, tracking_error_sym

from .acados_params import p

MODEL_NAME = 'offboard_test_acados_nmpc'


def build_acados_model():
    f_ode, x_sym, u_sym = build_dynamics()
    xr_sym = cs.MX.sym('xr', p.nx)

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x_sym
    model.u = u_sym
    model.p = xr_sym
    model.f_expl_expr = f_ode(x_sym, u_sym)

    e_track = tracking_error_sym(x_sym, xr_sym)        # 12维: [ep;ev;eq_vec;eomega]
    u_hover_const = cs.MX(p.u_hover)
    # Uref 在原 CasADi/IPOPT 版本里永远是常数 u_hover(从不随时间变,见
    # nmpc_node.solve_nmpc 里的 Uref_win),所以这里直接写成常量,不用再加一个
    # 运行时参数。
    model.cost_y_expr = cs.vertcat(e_track, u_sym - u_hover_const)   # 16维
    model.cost_y_expr_e = e_track                                     # 12维

    return model
