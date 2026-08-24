#!/usr/bin/env python3
"""质量可辨识性 / 敏感度分析(第 6 项的证据,离线、只依赖 casadi+scipy)。

为什么需要它:MHE 质量估计不收敛/有偏时,第一反应往往是"调 Q/R"。这个脚本先
回答**信息够不够**这个前置问题——在 N*dt 的窗口内,给定测量噪声标准差,质量的
Cramér-Rao 下界是多少,这些信息分别来自平动通路还是转动通路,以及模型结构改动
(把 J、复合质心写成被估质量的函数)能把下界改善多少。

两套模型:
  A = legacy 实现:dJ/c 是外部常参数,∂(J,c)/∂m ≡ 0 → 质量只走平动通路
  B = coupled 实现:J(m)/c(m) 由 m_B 与已知几何 r_p 代数生成 → 平动+转动双通路
真值 plant 一律用 B(物理正确的那个),用它造数据后再分别用 A/B 拟合。

输出五节:①边际 Fisher 信息与 σ_m ②(|q|,m) 尺度退化的机理核验 ③profile 代价
(查多解/平坦) ④模型误配/几何先验误差 → 质量偏差 ⑤m_P=α·m_B 重参数化的作用。

跑法: python3 analyze_mass_identifiability.py
"""
import numpy as np
import casadi as cs
from scipy.optimize import least_squares

mB, g, kd = 2.0643, 9.81, 0.05
Jxx, Jyy, Jzz = 0.0142, 0.0142, 0.0210
sp, sv, sq, so = 0.02, 0.05, 0.01, 0.02
SIG = np.array([sp]*3 + [sv]*3 + [sq]*4 + [so]*3)
dt, N = 0.1, 20


def geom_of_m(m, rp, avg_scalar=True, per_axis=False):
    """m_T -> (dJ_vec(3), c_xy(2))  代数生成, 与代码 _payload_geometry 同式"""
    mP = m - mB
    mu = mB * mP / m
    rx, ry, rz = rp[0], rp[1], rp[2]
    if per_axis:
        dJ = cs.vertcat(mu*(ry**2+rz**2), mu*(rx**2+rz**2), mu*(rx**2+ry**2))
    else:
        s = mu*(rz**2 + 0.5*(rx**2+ry**2))
        dJ = cs.vertcat(s, s, 0.0)          # 现行标量近似: Jzz 不变
    c = cs.vertcat((mP/m)*rx, (mP/m)*ry)
    return dJ, c


def make_f(variant, rp, per_axis=False):
    """返回 CasADi f(x(13), m(1), u(4), geomA(3)) -> xdot ; variant in {A,B}"""
    x = cs.MX.sym('x', 13); m = cs.MX.sym('m'); u = cs.MX.sym('u', 4)
    gA = cs.MX.sym('gA', 3)                    # A 用的外部 [dJ,cx,cy]
    v, q, om = x[3:6], x[6:10], x[10:13]
    T, tau = u[0], u[1:4]
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    R = cs.vertcat(
        cs.horzcat(qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
        cs.horzcat(2*(qx*qy+qw*qz), qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)),
        cs.horzcat(2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), qw**2-qx**2-qy**2+qz**2))
    vdot = (1.0/m)*(cs.mtimes(R, cs.vertcat(0, 0, T)) - kd*v) - cs.MX([0, 0, g])
    Xi = cs.vertcat(cs.horzcat(-qx, -qy, -qz), cs.horzcat(qw, -qz, qy),
                    cs.horzcat(qz, qw, -qx), cs.horzcat(-qy, qx, qw))
    qdot = 0.5*cs.mtimes(Xi, om)
    if variant == 'A':
        dJ = cs.vertcat(gA[0], gA[0], 0.0); c = gA[1:3]
    else:
        dJ, c = geom_of_m(m, rp, per_axis=per_axis)
    Jv = cs.vertcat(Jxx, Jyy, Jzz) + dJ
    tcom = cs.vertcat(-c[1]*T, c[0]*T, 0.0)
    omdot = (tau + tcom - cs.cross(om, Jv*om))/Jv
    xdot = cs.vertcat(v, vdot, qdot, omdot)
    return cs.Function('f', [x, m, u, gA], [xdot])


def rk4(f, x, m, u, gA):
    k1 = f(x, m, u, gA); k2 = f(x+dt/2*k1, m, u, gA)
    k3 = f(x+dt/2*k2, m, u, gA); k4 = f(x+dt*k3, m, u, gA)
    return x + dt/6*(k1+2*k2+2*k3+k4)


def rollout(f, x0, m, U, gA):
    xs = [np.asarray(x0, float).reshape(-1)]
    for j in range(U.shape[0]):
        xs.append(np.array(rk4(f, xs[-1], m, U[j], gA)).reshape(-1))
    return np.array(xs)


def stacked_resid(f, z, Y, U, gA, m_fixed=None):
    """z=[x0(13), m] (m_fixed 给定时 z=[x0]) -> 归一化残差"""
    if m_fixed is None:
        x0, m = z[:13], z[13]
    else:
        x0, m = z[:13], m_fixed
    sym = isinstance(z, cs.MX)
    r = []
    x = x0
    for j in range(U.shape[0]):
        r.append((x - Y[j])/SIG)
        x = rk4(f, x, m, U[j], gA)
        if not sym:
            x = np.array(x).reshape(-1)
    r.append((x - Y[-1])/SIG)
    return cs.vertcat(*r) if sym else np.concatenate([np.asarray(v).reshape(-1) for v in r])


def build_case(name, rp, mP_true, om_amp=0.0, om_w=0.0, vel=None):
    """造一段 N 步真值轨迹 + 已知输入 U(T 用真实物理推力, tau 用平衡力矩+激励)"""
    mT = mB + mP_true
    fB = make_f('B', rp)
    dJn, cn = geom_of_m(mT, rp)
    dJn = np.array(cs.DM(dJn)).reshape(-1); cn = np.array(cs.DM(cn)).reshape(-1)
    x0 = np.zeros(13); x0[2] = 3.0; x0[6] = 1.0
    if vel is not None: x0[3:6] = vel
    T = mT*g
    U = np.zeros((N, 4))
    for j in range(N):
        t = j*dt
        U[j, 0] = T
        # 稳态偏心平衡力矩 + 正弦激励(模拟机动)
        U[j, 1] = cn[1]*T + om_amp*np.sin(om_w*t)
        U[j, 2] = -cn[0]*T + om_amp*np.cos(om_w*t)
        U[j, 3] = 0.0
    Y = rollout(fB, x0, mT, U, np.zeros(3))
    return dict(name=name, rp=rp, mT=mT, U=U, Y=Y, x0=x0,
                gA=np.array([dJn[0], cn[0], cn[1]]))


def fisher_m(f, case, gA, m_at, rows=None):
    """在 (x0_true, m_at) 处算 stacked 残差 Jacobian, 返回 m 的 Schur 边际信息"""
    z = cs.MX.sym('z', 14)
    r = stacked_resid(f, z, case['Y'], case['U'], gA)
    G = cs.Function('G', [z], [cs.jacobian(r, z)])
    Gv = np.array(G(np.concatenate([case['x0'], [m_at]])))
    if rows is not None:
        mask = np.zeros(Gv.shape[0], bool)
        for k in range(N+1):
            mask[k*13 + np.array(rows)] = True
        Gv = Gv[mask]
    F = Gv.T @ Gv
    Fxx, Fxm, Fmm = F[:13, :13], F[:13, 13], F[13, 13]
    schur = Fmm - Fxm @ np.linalg.solve(Fxx + 1e-12*np.eye(13), Fxm)
    return Fmm, max(schur, 0.0), np.linalg.cond(F)


def profile(f, case, gA, grid):
    """对每个 m 把 x0 优化掉, 给出 profile 代价(查多解/平坦)"""
    out = []
    for m in grid:
        fun = lambda zz: np.asarray(
            stacked_resid(f, zz, case['Y'], case['U'], gA, m_fixed=m)).reshape(-1)
        s = least_squares(fun, case['x0'].copy(), xtol=1e-12, ftol=1e-12)
        out.append(0.5*np.sum(s.fun**2))
    return np.array(out)


def fit(f, case, gA, m0):
    fun = lambda zz: np.asarray(
        stacked_resid(f, zz, case['Y'], case['U'], gA)).reshape(-1)
    z0 = np.concatenate([case['x0'], [m0]])
    s = least_squares(fun, z0, xtol=1e-14, ftol=1e-14)
    return s.x[13], 0.5*np.sum(s.fun**2)


cases = [
    build_case('①悬停/居中载荷 r=[0,0,-0.47]', np.array([0., 0., -0.47]), 0.3),
    build_case('②悬停/偏心 ry=0.05', np.array([0., 0.05, -0.47]), 0.3),
    build_case('③机动/偏心(τ激励0.1Nm,1rad/s)', np.array([0., 0.05, -0.47]), 0.3,
               om_amp=0.10, om_w=1.0),
    build_case('④机动/居中(τ激励0.1Nm)', np.array([0., 0., -0.47]), 0.3,
               om_amp=0.10, om_w=1.0),
    build_case('⑤悬停/偏心/轻载0.15kg', np.array([0., 0.05, -0.47]), 0.15),
]

TRANS = list(range(0, 6)); ROT = list(range(6, 13))
print('='*100)
print('【1】m 的边际 Fisher 信息 (Schur补) 与 Cramer-Rao 下界 sigma_m,  N=20 dt=0.1 窗口')
print('='*100)
hdr = f"{'工况':<34}{'模型':<5}{'全通道 sig_m[kg]':>17}{'仅平动':>12}{'仅转动':>12}{'转动占比':>10}"
print(hdr)
for cse in cases:
    for var in ('A', 'B'):
        f = make_f(var, cse['rp'])
        _, s_all, _ = fisher_m(f, cse, cse['gA'], cse['mT'])
        _, s_tr, _ = fisher_m(f, cse, cse['gA'], cse['mT'], rows=TRANS)
        _, s_ro, _ = fisher_m(f, cse, cse['gA'], cse['mT'], rows=ROT)
        sig = lambda s: (1.0/np.sqrt(s)) if s > 1e-14 else float('inf')
        frac = s_ro/s_all*100 if s_all > 0 else 0.0
        print(f"{cse['name'] if var=='A' else '':<34}{var:<5}"
              f"{sig(s_all):>17.5f}{sig(s_tr):>12.5f}{sig(s_ro):>12.5f}{frac:>9.1f}%")

print()
print('='*100)
print('【2】"仅平动 inf" 的机理核验:四元数未归一化 -> (|q|, m) 尺度退化')
print('='*100)
cse = cases[1]
fA = make_f('A', cse['rp'])
z = cs.MX.sym('z', 14)
rr = stacked_resid(fA, z, cse['Y'], cse['U'], cse['gA'])
mask = np.zeros(13*(N+1), bool)
for k in range(N+1): mask[k*13+np.arange(0, 6)] = True
Gf = cs.Function('Gf', [z], [cs.jacobian(rr, z)])
Gv = np.array(Gf(np.concatenate([cse['x0'], [cse['mT']]])))[mask]
U_, S_, Vt_ = np.linalg.svd(Gv, full_matrices=False)
lbl = ['px','py','pz','vx','vy','vz','qw','qx','qy','qz','wx','wy','wz','m']
print(f'仅平动行 Jacobian 的奇异值(末4个): {S_[-4:]}')
for k in (1, 2, 3, 4):
    nv = Vt_[-k]
    print(f'  第{k}小奇异方向 (s={S_[-k]:.2e}): ' + ', '.join(
        f'{lbl[i]}={nv[i]:+.3f}' for i in range(14) if abs(nv[i]) > 0.05))
print('  解析预测的精确退化: R_q(q) 未归一化 -> q->s*q 使 R 放大 s^2,')
print('  与 m->s^2*m 精确抵消(仅剩 kd*v/m 的微弱残留)。直接数值检验:')
rfun = lambda x0, m: np.asarray(stacked_resid(
    fA, np.concatenate([x0, [m]]), cse['Y'], cse['U'], cse['gA'])).reshape(-1)[mask]
base = rfun(cse['x0'], cse['mT'])
for sc in (1.001, 1.01, 1.05):
    x0s = cse['x0'].copy(); x0s[6:10] *= sc
    r2 = rfun(x0s, cse['mT']*sc**2)
    r3 = rfun(cse['x0'], cse['mT']*sc**2)
    print(f'    s={sc}: ||r(q*s, m*s^2)-r(基准)||={np.linalg.norm(r2-base):.3e}'
          f'   而单独改 m 一项 ||r(q, m*s^2)-r||={np.linalg.norm(r3-base):.3e}')

print()
print('='*100)
print('【3】profile 代价 J(m)(把 x0 优化掉)—— 查多解/平坦区')
print('='*100)
grid = np.arange(1.95, 3.31, 0.05)
for cse in (cases[1], cases[2], cases[4]):
    for var in ('A', 'B'):
        f = make_f(var, cse['rp'])
        Jp = profile(f, cse, cse['gA'], grid)
        i = int(np.argmin(Jp))
        # 局部极小计数
        loc = sum(1 for k in range(1, len(Jp)-1) if Jp[k] < Jp[k-1] and Jp[k] < Jp[k+1])
        # 曲率(在最小点附近二阶差分)
        curv = (Jp[max(i-1,0)] - 2*Jp[i] + Jp[min(i+1,len(Jp)-1)])/0.05**2
        print(f"{cse['name']:<34}{var}  m*={grid[i]:.3f}(真值{cse['mT']:.3f})  "
              f"J_min={Jp[i]:.3e}  局部极小数={loc}  曲率={curv:.3e}  "
              f"J(真值±0.3kg)/J_min={(Jp[min(i+6,len(Jp)-1)]+1e-30)/(Jp[i]+1e-30):.2e}")

print()
print('='*100)
print('【4】模型误配的后果:用 A(dJ/c 与 m 脱钩,且外部 dJ/c 给错)拟合真值数据')
print('='*100)
cse = cases[1]
for scale, tag in ((1.0, '外部几何完全正确'), (0.67, '外部 dJ/c 偏低 33%'),
                   (1.33, '外部 dJ/c 偏高 33%'), (0.0, '外部几何=0(空机 J)')):
    fA = make_f('A', cse['rp'])
    m_hat, J = fit(fA, cse, cse['gA']*scale, 2.0643)
    print(f"  A + {tag:<22} m_hat={m_hat:.4f}  误差={100*(m_hat-cse['mT'])/cse['mT']:+.2f}%  J={J:.3e}")

print()
print('【4b】提议模型 B 对几何先验 r_p 误差的敏感度(mass 现在吃 r_p 误差)')
for f_rz, f_ry in ((1.0, 1.0), (0.8, 1.0), (1.2, 1.0), (0.67, 1.0),
                   (1.0, 0.5), (1.0, 2.0), (1.0, 0.0)):
    rp_w = cse['rp']*np.array([1.0, f_ry, f_rz])
    fBw = make_f('B', rp_w)
    m_hat, J = fit(fBw, cse, cse['gA'], 2.0643)
    print(f"  B + rz×{f_rz:.2f} ry×{f_ry:.2f}  ->  m_hat={m_hat:.4f}  "
          f"误差={100*(m_hat-cse['mT'])/cse['mT']:+.2f}%  ({m_hat-cse['mT']:+.3f}kg)  J={J:.3e}")

print()
print('='*100)
print('【5】重参数化 m_P = alpha*m_B 是否改善条件/唯一性')
print('='*100)
cse = cases[1]
for var in ('A', 'B'):
    f = make_f(var, cse['rp'])
    Fmm, s, cond = fisher_m(f, cse, cse['gA'], cse['mT'])
    # alpha 参数化: m = mB(1+alpha) -> dm/dalpha = mB, 信息按 mB^2 缩放
    print(f"  {var}: sigma_m={1/np.sqrt(s):.5f}kg   cond(F_m参数化)={cond:.3e}")
    print(f"     alpha 参数化: sigma_alpha={1/np.sqrt(s*mB**2):.5f} "
          f"= sigma_m/mB={1/np.sqrt(s)/mB:.5f}  -> 相对精度**逐位相同**")
print('  结论:alpha=mP/mB 是 m 的仿射变换,Fisher 信息按 (dm/dalpha)^2=mB^2 精确缩放,')
print('  条件数/多解结构不变;唯一的实际作用是把 0<alpha<1 的箱约束表达得更自然。')
