#!/usr/bin/env python3
# M1 阶段一:参数化 MHE 权重时间表 + 闭环训练场(见 技术方案_MHE学习模块_20260703.md §4 M1)。
#
# M0 的 SITL 定案(2026-07-03)给出了本模块的两条设计约束:
# 1. 二值降权规则(deweight=1e-4 固定)撞上"入带速度 vs 过冲"的墙——快估计喂回
#    控制器 → 控制器更激进 → 暂态加速度更暴烈 → 降权窗口只信少数新帧 → m_est
#    跟着暂态过冲(SITL 实测冲到 1.454 vs 真值 1.564)。所以要学的不是一个标量,
#    是"过渡期的权重时间表",且训练场必须是闭环的,开环仿真复现不了这个耦合。
# 2. 损失必须同时含入带速度与过冲惩罚,只优化速度会训出翻车解。
#
# 训练场:纯 numpy 四旋翼真值(复用 test_mhe_standalone 的动力学) + 模拟 NMPC
# 激进度的高度 PD 控制器(前馈用 m_est,反馈增益调到 0.5kg 阶跃能打出 ±5N 推力
# 摆——对标 SITL 里 NMPC 的行为) + 真 acados MHE 求解器在环 + 与 mhe_node 相同
# 的两段式确认(T 偏离基线>1.5N)。一集 12s ≈ 120 次 MHE 求解,秒级,可支撑无梯度
# 优化的几百次评估——这是不上批量 SITL、又不丢闭环耦合的折中。

import numpy as np
from scipy.linalg import block_diag

from .mhe_params import p as mhe_p

# 复用 standalone 测试的真值动力学(纯 numpy,与被测 MHE 的符号表达式相互独立)
from .mhe_event_weights import EventWeightScheduler  # 名义权重复位用


# =====================================================================
#  真值动力学(从 test_mhe_standalone 抄来,放包内避免 test 目录 import 环)
# =====================================================================
def _rotmat_from_quat(q):
    qw, qx, qy, qz = q
    return np.array([
        [qw**2+qx**2-qy**2-qz**2, 2*(qx*qy-qw*qz),         2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),         qw**2-qx**2+qy**2-qz**2, 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),         2*(qy*qz+qw*qx),         qw**2-qx**2-qy**2+qz**2],
    ])


def _quad_dynamics_np(x13, u4, m):
    vel = x13[3:6]
    q = x13[6:10]
    om = x13[10:13]
    T_, tau = u4[0], u4[1:4]
    R = _rotmat_from_quat(q)
    g_vec = np.array([0.0, 0.0, mhe_p.g])
    vel_dot = (1.0/m) * (R @ np.array([0.0, 0.0, T_]) - mhe_p.kd * vel) - g_vec
    qw, qx, qy, qz = q
    Xi = np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])
    quat_dot = 0.5 * Xi @ om
    J = np.array([mhe_p.Jxx, mhe_p.Jyy, mhe_p.Jzz])
    om_dot = (tau - np.cross(om, J*om)) / J
    return np.concatenate([vel, vel_dot, quat_dot, om_dot])


def _rk4_step_np(x13, u4, m, dt):
    k1 = _quad_dynamics_np(x13, u4, m)
    k2 = _quad_dynamics_np(x13 + dt/2*k1, u4, m)
    k3 = _quad_dynamics_np(x13 + dt/2*k2, u4, m)
    k4 = _quad_dynamics_np(x13 + dt*k3, u4, m)
    x_next = x13 + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
    x_next[6:10] /= np.linalg.norm(x_next[6:10])
    return x_next


# =====================================================================
#  参数化权重时间表
# =====================================================================
class ParametricWeightSchedule:
    """把 M0 的二值降权推广成 4 参数的过渡期权重时间表。

    theta(全部作用在"确认后到事件滑出窗口 + extra 帧"的过渡期内):
      theta[0] log10 事件前 stage 的 R/Q 整体缩放      (M0 规则 = -4)
      theta[1] log10 事件后 stage 的 R vel 块缩放      (M0 规则 = 0;
               调小 = 过渡期少信剧烈暂态的速度测量 → 压过冲的主旋钮)
      theta[2] 过渡期在事件滑出窗口后再延长的帧数       (M0 规则 = 0)
      theta[3] log10 过渡期 Q0 质量锚缩放              (M0 规则 = 0;
               调大 = 锚住上一窗口质量估计 → 阻尼,但太大拖收敛)

    M0 基线:theta = [-4, 0, 0, 0]。语义与 EventWeightScheduler 一致:
    notify_event(第一个事件后测量帧序号) + 每次 solve 前 apply(solver, fc)。
    """

    # 与 mhe_params 的块索引对应
    _VEL = slice(3, 6)

    def __init__(self, theta):
        self.theta = np.asarray(theta, dtype=float)
        d_pre = 10.0 ** self.theta[0]
        r_vel = 10.0 ** self.theta[1]
        self.extra = int(round(self.theta[2]))
        q0m = 10.0 ** self.theta[3]

        R_post = mhe_p.R.copy()
        R_post[self._VEL, self._VEL] *= r_vel
        Q0_t = mhe_p.Q0.copy()
        Q0_t[13, 13] *= q0m

        self.W_nom = block_diag(mhe_p.R, mhe_p.Q)
        self.W0_nom = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0)
        self.W_pre = block_diag(d_pre * mhe_p.R, d_pre * mhe_p.Q)
        self.W_post = block_diag(R_post, mhe_p.Q)           # 过渡期的事件后 stage
        self.W0_pre = block_diag(d_pre * mhe_p.R, d_pre * mhe_p.Q, Q0_t)
        self.W0_post = block_diag(R_post, mhe_p.Q, Q0_t)

        self.event_frame = None
        self._dirty = False

    def notify_event(self, next_frame):
        self.event_frame = int(next_frame)

    def apply(self, solver, fc):
        N = mhe_p.N
        if self.event_frame is None:
            return False
        n_pre = N - (fc - self.event_frame)
        if n_pre >= N:
            return True  # 代价里还没有事件后测量,保持名义(同 EventWeightScheduler)
        if n_pre <= -self.extra:
            if self._dirty:
                solver.cost_set(0, 'W', self.W0_nom)
                for j in range(1, N):
                    solver.cost_set(j, 'W', self.W_nom)
                self._dirty = False
            self.event_frame = None
            return False
        solver.cost_set(0, 'W', self.W0_pre if n_pre > 0 else self.W0_post)
        for j in range(1, N):
            solver.cost_set(j, 'W', self.W_pre if j < n_pre else self.W_post)
        self._dirty = True
        return True


M0_THETA = np.array([-4.0, 0.0, 0.0, 0.0])  # 二值规则在此参数化下的坐标


# =====================================================================
#  闭环一集仿真
# =====================================================================
class Scenario:
    def __init__(self, dm, lag_sec, t_step=5.0, duration=12.0, seed=0):
        self.dm = dm            # 质量阶跃(+抓取/-投放)
        self.lag_sec = lag_sec  # 事件通知 → 物理生效的延迟(gz CLI/机械延迟)
        self.t_step = t_step
        self.duration = duration
        self.seed = seed


def run_episode(solver, scenario, schedule=None, event_enabled=True):
    """跑一集闭环仿真。schedule=None 且 event_enabled=False → 固定权重基线;
    schedule=None 且 event_enabled=True → 不合法(事件版必须给时间表);
    返回逐帧记录 + 指标。"""
    rng = np.random.default_rng(scenario.seed)
    dt, N = mhe_p.dt, mhe_p.N
    n_steps = int(round(scenario.duration / dt))
    k_step = int(round(scenario.t_step / dt))          # 事件通知帧
    k_phys = k_step + int(round(scenario.lag_sec / dt))  # 物理生效帧

    m0 = mhe_p.m_nominal
    m_true = np.where(np.arange(n_steps + 1) >= k_phys, m0 + scenario.dm, m0)

    # 名义权重复位(solver 跨集复用,不能带着上一集的降权状态)
    _r = EventWeightScheduler()
    solver.cost_set(0, 'W', _r.W0_nom)
    for j in range(1, N):
        solver.cost_set(j, 'W', _r.W_nom)

    # 测量噪声(与 mhe_params 标定一致)
    std = np.array([mhe_p.std_pos]*3 + [mhe_p.std_vel]*3 +
                   [mhe_p.std_quat]*4 + [mhe_p.std_omega]*3)

    # 高度 PD:前馈 m_est*g,反馈增益对标 SITL 里 NMPC 的激进度
    # (0.5kg 阶跃 → 推力摆 ±5N 量级:kp*0.25m*m≈2.5N/kg → kp≈5, kd≈3)
    KP_Z, KD_Z = 5.0, 3.0
    z_ref = 3.0

    x = np.array([0., 0., z_ref, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.])
    y_buf, u_buf = [], []
    x0_bar = None
    x_guess = None
    m_est = m0

    # 两段式确认(与 mhe_node 相同:T 偏离武装基线 >1.5N)
    armed_baseline = None
    confirmed_frame = None

    rec = dict(t=[], m_est=[], m_true=[], T=[], z=[])

    for k in range(n_steps):
        t = k * dt
        # ---- 控制律(用 m_est,这里就是闭环耦合的来源) ----
        # 激励幅度 1.0N:够质量可观测(阶跃响应本身也是激励),又小于确认阈值
        # 1.5N——武装后 1s 内激励自身最多变化 ~1.3N,不会假触发确认
        T_cmd = (m_est * mhe_p.g
                 + m_est * (KP_Z * (z_ref - x[2]) + KD_Z * (0.0 - x[5]))
                 + 1.0 * np.sin(2*np.pi*0.2*t))
        # 上限 31.3N = SITL 物理可达推力(4k·ω²@norm=0.95 反解 clip),不是 NMPC
        # 约束的 40.5N——首版误用 40.5 让训练场里 +1.5kg 场景"能飞",而 x500
        # 物理上悬停都撑不住(需 34.9N>天花板 34.2N),2026-07-03 深夜发现。
        # 可飞幅度范围因此是 delta ∈ [-1.0, +1.0]。
        T_cmd = float(np.clip(T_cmd, 0.5, 31.3))
        # 姿态 PD 稳定(没有它飞机会被激励力矩翻滚,闭环直接发散——2026-07-03
        # 首版实测教训) + 小激励;增益按 J~0.014-0.02 取 ω_att≈5rad/s
        q_vec = x[7:10] * np.sign(x[6])
        tau = (-0.35 * q_vec - 0.10 * x[10:13]
               + np.array([0.005*np.sin(2*np.pi*0.1*t),
                           0.005*np.cos(2*np.pi*0.1*t), 0.0]))
        u = np.array([T_cmd, *tau])

        # ---- 真值推进(用 m_true) ----
        x = _rk4_step_np(x, u, m_true[k], dt)
        y = x + std * rng.standard_normal(13)
        y[6:10] /= np.linalg.norm(y[6:10])

        y_buf.append(y)
        u_buf.append(u)
        if len(y_buf) > N + 1:
            y_buf.pop(0)
        if len(u_buf) > N:
            u_buf.pop(0)

        # ---- 事件武装/确认(帧序号语义与 mhe_node 一致) ----
        if event_enabled and schedule is not None:
            if k == k_step:
                armed_baseline = T_cmd
            if (armed_baseline is not None and confirmed_frame is None
                    and abs(T_cmd - armed_baseline) > 1.5):
                confirmed_frame = k
                schedule.notify_event(k)

        # ---- MHE ----
        if len(y_buf) == N + 1:
            if x0_bar is None:
                x0_bar = np.concatenate([y_buf[0], [m0]])
                x_guess = [np.concatenate([y_buf[min(i, N)], [m0]])
                           for i in range(N + 1)]
            yref_0 = np.concatenate([y_buf[0], np.zeros(mhe_p.nw), x0_bar])
            solver.set(0, 'yref', yref_0)
            solver.set(0, 'p', u_buf[0])
            solver.set(0, 'x', x_guess[0])
            for j in range(1, N):
                solver.set(j, 'yref',
                           np.concatenate([y_buf[j], np.zeros(mhe_p.nw)]))
                solver.set(j, 'p', u_buf[j])
                solver.set(j, 'x', x_guess[j])
            solver.set(N, 'x', x_guess[N])

            if event_enabled and schedule is not None:
                schedule.apply(solver, k)

            if solver.solve() == 0:
                x_sol = [solver.get(i, 'x') for i in range(N + 1)]
                m_est = float(np.clip(x_sol[N][13], mhe_p.m_min, mhe_p.m_max))
                x0_bar = x_sol[1].copy()
                x_guess = [x_sol[min(i+1, N)].copy() for i in range(N + 1)]

        rec['t'].append(t)
        rec['m_est'].append(m_est)
        rec['m_true'].append(m_true[k])
        rec['T'].append(T_cmd)
        rec['z'].append(x[2])

    return _metrics(rec, scenario, k_phys)


def _metrics(rec, scenario, k_phys):
    t = np.array(rec['t'])
    m = np.array(rec['m_est'])
    z = np.array(rec['z'])
    m_post = mhe_p.m_nominal + scenario.dm
    band = max(0.08, 0.15 * abs(scenario.dm))   # 带宽随阶跃幅度走,跨幅度可比
    t_phys = k_phys * mhe_p.dt

    after = t >= t_phys
    inband = after & (np.abs(m - m_post) <= band)
    t_enter = (t[inband][0] - t_phys) if np.any(inband) else scenario.duration
    outside = after & (np.abs(m - m_post) > band)
    t_settle = (t[outside][-1] + mhe_p.dt - t_phys) if np.any(outside) else 0.0
    # 过冲:越过真值后向另一侧的最大超调
    overshoot = 0.0
    if np.any(inband):
        k_in = np.argmax(inband)
        seg = m[k_in:]
        overshoot = float(np.max(np.abs(seg - m_post)))
    # 稳态抖动:最后 3s
    tail = t >= (t[-1] - 3.0)
    jitter = float(np.std(m[tail]))
    # 控制层:阶跃后高度偏移峰值
    z_peak = float(np.max(np.abs(z[after] - 3.0))) if np.any(after) else 0.0

    loss = (1.0 * t_enter + 1.0 * t_settle
            + 8.0 * overshoot + 20.0 * jitter + 3.0 * z_peak)
    return dict(t_enter=t_enter, t_settle=t_settle, overshoot=overshoot,
                jitter=jitter, z_peak=z_peak, loss=loss, rec=rec)


# 训练集覆盖**物理可飞**的幅度范围:dm<-1.0 压破 solver m_min=1.0;dm>+1.0
# 超 SITL 推力天花板(见上 T_cmd clip 注释),两头都到 ±1.0 为止。曾放过 +1.5,
# 那是 Tmax 误用 40.5N 时的"纸面可飞"工况,已删。留出集测插值,不测外推。
DEFAULT_SCENARIOS = [
    Scenario(dm=-0.5, lag_sec=1.0, seed=1),   # SITL drop 工况
    Scenario(dm=+1.0, lag_sec=0.5, seed=2),   # 抓取方向最大可飞幅度
    Scenario(dm=-1.0, lag_sec=1.2, seed=3),   # 减载方向最大幅度
    Scenario(dm=+0.5, lag_sec=0.8, seed=4),
    Scenario(dm=-0.75, lag_sec=1.1, seed=5),
]


def evaluate(solver, theta, scenarios=None, event_enabled=True):
    """theta=None → 固定权重基线。返回平均 loss 与各集指标。"""
    scenarios = scenarios or DEFAULT_SCENARIOS
    results = []
    for sc in scenarios:
        schedule = (ParametricWeightSchedule(theta)
                    if (theta is not None and event_enabled) else None)
        results.append(run_episode(solver, sc, schedule,
                                   event_enabled=(theta is not None)))
    mean_loss = float(np.mean([r['loss'] for r in results]))
    return mean_loss, results
