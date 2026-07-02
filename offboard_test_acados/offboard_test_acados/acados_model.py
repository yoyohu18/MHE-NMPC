#!/usr/bin/env python3
# acados 专用的动力学,跟 offboard_test.nmpc_node.build_dynamics() 数学上完全
# 一致,唯一区别是质量 m 不再是常数 p.m,而是 model.p 里的一个运行时参数——
# 这样 mhe_node 估出来的质量才能真的喂给 NMPC 用,不是只发个诊断话题。
# (CasADi/IPOPT 版本的 offboard_test/nmpc_node.py 不需要这个能力,继续用它
# 自己原来的 build_dynamics(),两边没有耦合。)
#
# model.p = [xr(13维,参考状态); m(1维,当前质量估计); dJ(1维,吊挂惯量增量)]
# = 15维。时变参考用 model.p 而不是 yref 来传,是因为代价里的四元数误差项对
# 参考值是非线性的(四元数乘法),没法写成简单的 "y - yref" 形式,必须让参考值
# 进 CasADi 表达式本身——质量/惯量参数顺路放在同一个 model.p 向量里。

import casadi as cs
from acados_template import AcadosModel

from offboard_test.nmpc_node import tracking_error_sym

from .acados_params import p

MODEL_NAME = 'offboard_test_acados_nmpc'


def build_acados_model():
    x_sym = cs.MX.sym('x', p.nx)
    u_sym = cs.MX.sym('u', p.nu)
    xr_sym = cs.MX.sym('xr', p.nx)
    m_sym = cs.MX.sym('m', 1)
    # 吊挂载荷对滚转/俯仰惯量的增量 dJ = m_p*d^2(点质量 m_p 刚性焊在机体正
    # 下方 d 处,平行轴定理;载荷在机体 z 轴上,Izz 不变,CoM 沿轴下移也不产生
    # 推力/重力力矩——推力线始终过合成 CoM)。空机时 0。这是夹爪实验里"0.2kg
    # 就把姿态打崩"的主导缺失物理:0.3kg@0.47m 臂 → dJ≈0.066,是空机
    # Jxx(0.0217) 的 4 倍,NMPC 按空机惯量规划的角加速度真机做不到,力矩饱和
    # 后 SQP 级联发散。跟 m_sym 一样走 model.p,attach 时随质量阶跃一起喂。
    dJ_sym = cs.MX.sym('dJ', 1)

    vel = x_sym[3:6]
    q_  = x_sym[6:10]
    om  = x_sym[10:13]
    T_   = u_sym[0]
    tau_ = u_sym[1:4]

    qw = q_[0]; qx = q_[1]; qy = q_[2]; qz = q_[3]
    R_q = cs.vertcat(
        cs.horzcat(qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),        2*(qx*qz+qw*qy)),
        cs.horzcat(2*(qx*qy+qw*qz),          qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)),
        cs.horzcat(2*(qx*qz-qw*qy),          2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2)
    )

    g_vec = cs.MX([0.0, 0.0, p.g])
    # 跟 build_dynamics() 唯一的数学差异:用 model.p 里的 m_sym(变量)而不是
    # 常数 p.m——这一行就是闭环自适应的关键,质量估计变了,这里的加速度
    # 映射立刻跟着变,NMPC 下一次求解就会用新的质量重新规划推力。
    vel_dot = (1.0/m_sym) * (cs.mtimes(R_q, cs.vertcat(0.0, 0.0, T_)) - p.kd * vel) - g_vec

    Xi_q = cs.vertcat(
        cs.horzcat(-qx, -qy, -qz),
        cs.horzcat( qw, -qz,  qy),
        cs.horzcat( qz,  qw, -qx),
        cs.horzcat(-qy,  qx,  qw)
    )
    quat_dot = 0.5 * cs.mtimes(Xi_q, om)

    J_vec = cs.vertcat(p.Jxx + dJ_sym, p.Jyy + dJ_sym, p.Jzz)
    Jom   = J_vec * om
    om_dot = (tau_ - cs.cross(om, Jom)) / J_vec

    xdot = cs.vertcat(vel, vel_dot, quat_dot, om_dot)

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x_sym
    model.u = u_sym
    model.p = cs.vertcat(xr_sym, m_sym, dJ_sym)
    model.f_expl_expr = xdot

    e_track = tracking_error_sym(x_sym, xr_sym)        # 12维: [ep;ev;eq_vec;eomega]
    # u_hover 用 m_sym(已经是 model.p 的一部分)实时算,不再是基于固定标定
    # 质量 p.m 的编译时常量。早先(质量阶跃只有"抓取"场景、且单元测试用方向
    # 性检查代替精确相等断言时)判断过"惩罚基准用旧值不会让控制失效,影响
    # 很小,暂不处理"——这个判断在"投放"场景里被推翻了:满载 3.5kg 时真实
    # 悬停推力约 34N,但写死的基准还停在空载标定值约 20N,cost 里 u-u_hover
    # 这一项持续把推力"拽"向这个偏低的旧基准,实测会让追踪误差卡在 0.85m+
    # 完全不收敛,不是"影响很小"。改成 m_sym*g 之后,质量估计变了,惩罚中心
    # 跟着动态平移,不再固化一个过时的悬停推力假设。
    u_hover_dyn = cs.vertcat(m_sym * p.g, 0.0, 0.0, 0.0)
    model.cost_y_expr = cs.vertcat(e_track, u_sym - u_hover_dyn)     # 16维
    model.cost_y_expr_e = e_track                                     # 12维

    return model
