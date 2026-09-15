#!/usr/bin/env python3
# MHE 的 acados 模型：使用与 NMPC 一致的四旋翼
# 动力学(旋转矩阵/四元数动力学/角速度动力学)搬过来,唯一的区别是质量 m 不再是
# 常数 p.m,而是增广进状态向量、跟其余 13 个物理状态一起被估计——这是 acados
# 官方 MHE 范式(见 acados/examples/acados_python/pendulum_on_cart/mhe/
# export_mhe_ode_model_with_param.py)在四旋翼上的对应写法:
#   x_aug = [pos(3); vel(3); quat(4); omega(3); m(1)]   14维,被估计的状态
#   u(MHE里复用'control'这个槛位装过程噪声 w,不是真的控制量) = w(13维)
#   p(MHE里的'parameter') = 已知输入 [T, tau_x, tau_y, tau_z](4维,不被估计)
#                          + 已知几何 (3维;legacy=[dJ,cx,cy] / coupled=[rx,ry,rz])
#
# 【2026-08-24 几何-质量耦合】mhe_p.geom_coupled=True 时,惯量矩阵与复合质心不再
# 是外部常参数,而是被估质量的显函数 J(m)/c(m)(平行轴定理,完整 3×3 含非对角项)。
# 动机:legacy 下 ∂(J,c)/∂m ≡ 0,质量**只**通过 1/m·R·F 这一条平动通路被辨识,
# 转动残差对质量的导数恒零 → 窗口内 Cramér-Rao 下界 σ_m=0.0109kg;耦合后偏心
# 悬停 σ_m=0.0011kg(≈10×),94% 的信息来自转动通路(数值见 n_geom 注释)。
# 另一个同样重要的后果:legacy 那条"m_est → 窗外算 dJ/c → 下一次求解"的滞后回路
# 是**外层不动点迭代**(正反馈,曾在 0.15kg 轻载上自锁,见记忆
# gripper-slung-load-vs-mhe),耦合后它变成同一次求解内部一致的 Jacobian,不再有
# 自举正反馈——这是结构性修法,不是调参。
#
# 质量这一维的 ODE 是 m_dot=0(窗口内当常数,没有过程噪声驱动)——这意味着每次
# 求解给出的是"整个窗口内统一的一个质量估计值",窗口滑动到新数据时这个值才会
# 变,这就是为什么质量阶跃(抓/放包裹)后会有一段跟窗口长度同量级的过渡态,
# 见 配送无人机自适应学习控制_技术路线笔记.md 第7节。

import casadi as cs
from acados_template import AcadosModel

from .mhe_params import p as mhe_p

# codegen 目录/json 名都由 MODEL_NAME 派生(见 mhe_solver_builder),耦合档必须
# 带后缀,否则两档共用一份生成代码 → 互相覆盖(与 MHE_N 已知的并发坑同类)。
_SUF = '_pnoise' if mhe_p.param_noise else ''
_AM = ('' if mhe_p.moment_a_mode == 'coupled' else '_a' + mhe_p.moment_a_mode)
MODEL_NAME = (('offboard_test_acados_mhe_moment' + _AM + _SUF) if mhe_p.estimate_moment
              else ('offboard_test_acados_mhe_coupled' + _SUF) if mhe_p.geom_coupled
              else ('offboard_test_acados_mhe' + _SUF))


def build_mhe_model() -> AcadosModel:
    pos = cs.MX.sym('pos', 3)
    vel = cs.MX.sym('vel', 3)
    q_  = cs.MX.sym('q', 4)
    om  = cs.MX.sym('om', 3)
    m_  = cs.MX.sym('m', 1)
    if mhe_p.estimate_moment:
        # 一阶质量矩 s=m_P·r_xy [kg·m],被估状态(见 mhe_params.estimate_moment)
        s_  = cs.MX.sym('s', mhe_p.ns)
        x_aug = cs.vertcat(pos, vel, q_, om, m_, s_)
    else:
        s_ = None
        x_aug = cs.vertcat(pos, vel, q_, om, m_)

    w_sym = cs.MX.sym('w', mhe_p.nw)          # 过程噪声,装在 MHE 的 'u' 槛位
    u_known = cs.MX.sym('u_known', mhe_p.nu_known)  # 已知 [T, taux, tauy, tauz]
    # 已知几何(dJ, c_xy):跟 acados_model.py 的 dJ_sym/c_sym 同一套物理,见
    # mhe_params.py 的 n_geom 注释——不是新增被估计自由度,是喂给 MHE 自己
    # 动力学模型的已知参数,修掉"固定空机 J 导致质量估计被姿态失配拖偏"的问题。
    geom = cs.MX.sym('geom', mhe_p.n_geom)    # legacy:[dJ,cx,cy] / coupled:[rx,ry,rz]

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

    # ---- 转动通路:J 与 c 怎么来,由 geom_coupled / estimate_moment 决定 ----
    if mhe_p.estimate_moment:
        # 2b:载荷的"在不在/偏多少"由一阶质量矩 s 独立承担,不再编码进质量。
        #   c_xy = s/m_T                                  (不需要 r_xy)
        #   J = J_B + A·diag(1,1,0)... 见下,r_z 只作弱先验
        # 推导见 mhe_params.estimate_moment 注释。geom 槽里只用得到 r_z;若外部
        # 传了完整 r_p 就取其 z 分量,没传(全零)则退回 rz_prior。
        rz_ext = geom[2]
        rz = cs.if_else(cs.fabs(rz_ext) > 1e-6, rz_ext, mhe_p.rz_prior)
        dm = m_ - mhe_p.m_B
        m_p = 0.5 * (dm + cs.sqrt(dm * dm + mhe_p.mp_pos_eps ** 2))
        c_sym = s_ / m_                       # 复合质心水平偏移
        # A=μ·r_z²。三档见 mhe_params.moment_a_mode。
        if mhe_p.moment_a_mode == 'zero':
            A = cs.MX.zeros(1)
        elif mhe_p.moment_a_mode == 'frozen':
            m_frz = cs.if_else(geom[0] > 0.1, geom[0], mhe_p.m_B)
            _dmf = m_frz - mhe_p.m_B
            m_p_frz = 0.5 * (_dmf + cs.sqrt(_dmf * _dmf + mhe_p.mp_pos_eps ** 2))
            A = cs.if_else(cs.fabs(rz_ext) > 1e-6,
                           (mhe_p.m_B * m_p_frz / m_frz) * rz ** 2, 0.0)
        elif mhe_p.moment_a_mode == 'const':
            _mpe = mhe_p.mp_envelope
            A = cs.if_else(cs.fabs(rz_ext) > 1e-6,
                           (mhe_p.m_B * _mpe / (mhe_p.m_B + _mpe)) * rz ** 2,
                           0.0)
        else:
            A = (mhe_p.m_B * m_p / m_) * rz ** 2
        Bx = (mhe_p.m_B / m_) * s_[0] * rz                      # μ·r_x·r_z
        By = (mhe_p.m_B / m_) * s_[1] * rz                      # μ·r_y·r_z
        # O(r_xy²) 项(μr_x²、μr_y²、μr_xr_y)**故意丢弃**,置零。
        # 理由不是"嫌麻烦",是它们**引入退化区**:用 s 表达要写成
        # k·s_i·s_j,k=m_B/(m_T·m_P),m_P→0 时 k→∞,而 s 不受约束地留在非零值
        # = "极小载荷质量挂在极长力臂上",物理荒谬但模型允许。2026-08-24 离线
        # 实测:留着它们时 drop 后 m_est 单调爬到 3.17 不回头、s_y 逐帧变号
        # (病态方向在抖);丢掉后模型只剩 A=μr_z²(只需 m_P)与非对角
        # (m_B/m_T)·s·r_z(只需 s),**没有任何一项需要把 m_P 和 r_xy 分开知道**。
        # 代价:ΔJ 少算 ½(r_x²+r_y²)/(r_z²+½(r_x²+r_y²)) ≈ 2.2%(r_z 主导 97.8%),
        # 而 ΔJ 本身在悬停(ω̇≈0)时不携带质量信息——量级远小于它带来的病态。
        Cxx = cs.MX.zeros(1)
        Cyy = cs.MX.zeros(1)
        Cxy = cs.MX.zeros(1)
        J_B = cs.diag(cs.vertcat(mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz))
        # J = J_B + (A+Cxx+Cyy)·I − [[Cxx,Cxy,Bx],[Cxy,Cyy,By],[Bx,By,A]]
        J_mat = J_B + (A + Cxx + Cyy) * cs.MX.eye(3) - cs.vertcat(
            cs.horzcat(Cxx, Cxy, Bx),
            cs.horzcat(Cxy, Cyy, By),
            cs.horzcat(Bx,  By,  A))
        J_mat = J_mat + mhe_p.payload_ki * m_p * cs.MX.eye(3)
        Jom = cs.mtimes(J_mat, om)
        tau_thrust_com = cs.vertcat(-c_sym[1] * T_, c_sym[0] * T_, 0.0)
        om_dot = cs.mtimes(cs.inv(J_mat),
                           tau_ + tau_thrust_com - cs.cross(om, Jom))
    elif mhe_p.geom_coupled:
        # geom 槛位装的是**载荷相对机体原点的几何偏移** r_p=[rx,ry,rz](rz<0),
        # 这是 attach 时可测的几何量(gripper 相对位姿 / 下视相机),**不是**被估
        # 的质量。J 与 c 在这里由被估质量现算,于是 ∂ω̇/∂m ≠ 0:
        #   m_P = (m − m_B)⁺        (平滑正部,保证 μ≥0 → J≻0,见 mhe_params)
        #   r_c = (m_P/m)·r_p        复合质心(机体原点为参考)
        #   μ   = m_B·m_P/m          两点系的约化质量
        #   J   = J_B + k_I·m_P·I₃ + μ(|r_p|²I₃ − r_p r_pᵀ)   ← 平行轴定理(完整
        #         3×3,含非对角项;两点系对**复合质心**的贡献恰好等于 μ 乘这个
        #         括号,机体侧 m_B(|r_c|²I−r_c r_cᵀ) 与载荷侧已经合并在里面)
        #   τ_thrust,COM = (−r_c)×[0,0,T] = [−r_cy·T, +r_cx·T, 0]
        r_p = geom
        dm = m_ - mhe_p.m_B
        m_p = 0.5 * (dm + cs.sqrt(dm * dm + mhe_p.mp_pos_eps ** 2))
        mu = mhe_p.m_B * m_p / m_
        r2 = cs.dot(r_p, r_p)
        J_B = cs.diag(cs.vertcat(mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz))
        J_mat = J_B + mhe_p.payload_ki * m_p * cs.MX.eye(3) \
            + mu * (r2 * cs.MX.eye(3) - cs.mtimes(r_p, r_p.T))
        c_sym = (m_p / m_) * r_p[0:2]
        Jom = cs.mtimes(J_mat, om)
        # 推力对复合质心的力矩:(−r_c)×[0,0,T] = [−r_cy·T, +r_cx·T, 0]。
        tau_thrust_com = cs.vertcat(-c_sym[1] * T_, c_sym[0] * T_, 0.0)
        om_dot = cs.mtimes(cs.inv(J_mat),
                           tau_ + tau_thrust_com - cs.cross(om, Jom))
    else:
        # legacy:dJ/c_xy 是窗外算好的常参数,∂(J,c)/∂m ≡ 0 → 质量只从平动通路
        # 被辨识(转动通路对质量的 Jacobian 恒零)。**表达式逐字保持原样**(对角
        # J 走逐元素除法,不走 3×3 求逆),这样生成的 C 代码与历史批次一致。
        dJ_sym = geom[0]
        c_sym = geom[1:3]
        J_vec = cs.vertcat(mhe_p.Jxx + dJ_sym, mhe_p.Jyy + dJ_sym, mhe_p.Jzz)
        Jom = J_vec * om
        tau_thrust_com = cs.vertcat(-c_sym[1] * T_, c_sym[0] * T_, 0.0)
        om_dot = (tau_ + tau_thrust_com - cs.cross(om, Jom)) / J_vec

    m_dot = cs.MX.zeros(1)  # 窗口内质量当常数,见文件头注释
    if mhe_p.estimate_moment:
        # s 与 m 同待遇:窗口内常数、不加过程噪声。要不要给随机游走是 2a 的事。
        par_dot = cs.vertcat(m_dot, cs.MX.zeros(mhe_p.ns))
    else:
        par_dot = m_dot

    f_expl = cs.vertcat(vel, vel_dot, quat_dot, om_dot, par_dot)
    # 过程噪声只加在 13 个物理状态上,质量这一维不加噪声(否则窗口内质量会
    # 自由漂移,失去"窗口内常数"这个让估计有意义的约束)
    if mhe_p.param_noise:
        # 2a:被估参数也由过程噪声驱动(随机游走),w_sym 已是 nx+nm+ns 维
        f_expl = f_expl + w_sym
    else:
        f_expl = f_expl + cs.vertcat(w_sym, cs.MX.zeros(mhe_p.nm + mhe_p.ns))

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x_aug
    model.u = w_sym
    model.p = cs.vertcat(u_known, geom)
    model.f_expl_expr = f_expl

    # stage 0(窗口起点,带到达代价): [测量残差(13); 过程噪声残差(13); 到达代价残差(14)]
    model.cost_y_expr_0 = cs.vertcat(x_aug[0:mhe_p.nx], w_sym, x_aug)
    # stage 1..N-1: [测量残差(13); 过程噪声残差(13)]
    model.cost_y_expr = cs.vertcat(x_aug[0:mhe_p.nx], w_sym)
    # 终端不加代价(MHE 里窗口末端是"最新"的估计,不需要再往前看一步去惩罚)

    return model
