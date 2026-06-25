#!/usr/bin/env python3
# MHE 的 acados 模型:把 offboard_test.nmpc_node.build_dynamics() 里同一套四旋翼
# 动力学(旋转矩阵/四元数动力学/角速度动力学)搬过来,唯一的区别是质量 m 不再是
# 常数 p.m,而是增广进状态向量、跟其余 13 个物理状态一起被估计——这是 acados
# 官方 MHE 范式(见 acados/examples/acados_python/pendulum_on_cart/mhe/
# export_mhe_ode_model_with_param.py)在四旋翼上的对应写法:
#   x_aug = [pos(3); vel(3); quat(4); omega(3); m(1)]   14维,被估计的状态
#   u(MHE里复用'control'这个槛位装过程噪声 w,不是真的控制量) = w(13维)
#   p(MHE里的'parameter') = 已知输入 [T, tau_x, tau_y, tau_z](4维,不被估计)
#
# 质量这一维的 ODE 是 m_dot=0(窗口内当常数,没有过程噪声驱动)——这意味着每次
# 求解给出的是"整个窗口内统一的一个质量估计值",窗口滑动到新数据时这个值才会
# 变,这就是为什么质量阶跃(抓/放包裹)后会有一段跟窗口长度同量级的过渡态,
# 见 配送无人机自适应学习控制_技术路线笔记.md 第7节。

import casadi as cs
from acados_template import AcadosModel

from .mhe_params import p as mhe_p

MODEL_NAME = 'offboard_test_acados_mhe'


def build_mhe_model() -> AcadosModel:
    pos = cs.MX.sym('pos', 3)
    vel = cs.MX.sym('vel', 3)
    q_  = cs.MX.sym('q', 4)
    om  = cs.MX.sym('om', 3)
    m_  = cs.MX.sym('m', 1)
    x_aug = cs.vertcat(pos, vel, q_, om, m_)

    w_sym = cs.MX.sym('w', mhe_p.nw)          # 过程噪声,装在 MHE 的 'u' 槛位
    u_known = cs.MX.sym('u_known', mhe_p.nu_known)  # 已知 [T, taux, tauy, tauz]

    T_   = u_known[0]
    tau_ = u_known[1:4]

    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3]
    R_q = cs.vertcat(
        cs.horzcat(qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),        2*(qx*qz+qw*qy)),
        cs.horzcat(2*(qx*qy+qw*qz),          qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)),
        cs.horzcat(2*(qx*qz-qw*qy),          2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2)
    )

    g_vec = cs.MX([0.0, 0.0, mhe_p.g])
    # 跟 build_dynamics() 唯一的数学差异:用状态里的 m_(变量)而不是常数 p.m
    vel_dot = (1.0/m_) * (cs.mtimes(R_q, cs.vertcat(0.0, 0.0, T_)) - mhe_p.kd * vel) - g_vec

    Xi_q = cs.vertcat(
        cs.horzcat(-qx, -qy, -qz),
        cs.horzcat( qw, -qz,  qy),
        cs.horzcat( qz,  qw, -qx),
        cs.horzcat(-qy,  qx,  qw)
    )
    quat_dot = 0.5 * cs.mtimes(Xi_q, om)

    J_vec = cs.MX([mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz])
    Jom   = J_vec * om
    om_dot = (tau_ - cs.cross(om, Jom)) / J_vec

    m_dot = cs.MX.zeros(1)  # 窗口内质量当常数,见文件头注释

    f_expl = cs.vertcat(vel, vel_dot, quat_dot, om_dot, m_dot)
    # 过程噪声只加在 13 个物理状态上,质量这一维不加噪声(否则窗口内质量会
    # 自由漂移,失去"窗口内常数"这个让估计有意义的约束)
    f_expl = f_expl + cs.vertcat(w_sym, cs.MX.zeros(1))

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x_aug
    model.u = w_sym
    model.p = u_known
    model.f_expl_expr = f_expl

    # stage 0(窗口起点,带到达代价): [测量残差(13); 过程噪声残差(13); 到达代价残差(14)]
    model.cost_y_expr_0 = cs.vertcat(x_aug[0:mhe_p.nx], w_sym, x_aug)
    # stage 1..N-1: [测量残差(13); 过程噪声残差(13)]
    model.cost_y_expr = cs.vertcat(x_aug[0:mhe_p.nx], w_sym)
    # 终端不加代价(MHE 里窗口末端是"最新"的估计,不需要再往前看一步去惩罚)

    return model