#!/usr/bin/env python3
"""动力学残差**可辨识性**诊断(2026-07-31)。只读采集数据,不训练任何模型。

残差定义严格从 mhe_model.py:67 的模型方程逐项移项而来,不沿用文字加减号:
    模型:  J*om_dot = tau_act + tau_thrust_com - om x (J*om)
    移项:  r_tau   = tau_act + tau_thrust_com - om x (J*om) - J*om_dot
其中 tau_thrust_com = [-cy*T, +cx*T, 0](推力对复合质心的力矩),
     J = [Jxx+dJ, Jyy+dJ, Jzz]。

【诊断顺序(不可跳步)】
  step1 时钟域检查:各流 header vs 接收时刻,确认能否跨流相减
  step2 延迟扫描  :argmin_dt RMS(r(dt)) —— 先排除"残差只是时间错位"
  step3 导数估计  :裸差分 vs Savitzky-Golay,扫窗口,看残差对窗口是否敏感
  step4 分桶      :按 空载/带载 × hover/figure8 分别统计 mean/std/RMS
只有 step1-3 干净、step4 显示同状态区间内残差均值稳定,才谈得上"可学"。

⚠️本脚本第一阶段**只用空载段**(payload_attached=0):此时 c=0、dJ=0,
tau_thrust_com 与 dJ 都消失,残差退化成最干净的
    r = tau_act - om x (J*om) - J*om_dot
避免把"载荷几何估计误差"混进"模型失配"。带载段留到符号与量级都自检通过后再做。

用法: python3 diag_residual_structure.py [数据目录]
"""
import glob
import os
import sys

import numpy as np

_pos = [a for a in sys.argv[1:] if not a.startswith('--')]   # 排除 --step4 等开关
D = _pos[0] if _pos else \
    '/home/clear/ros2_ws_HJH/nmpc_test_results/residual'
sys.path.insert(0, '/home/clear/ros2_ws_HJH/install/offboard_test_acados/'
                   'lib/python3.12/site-packages')
JXX = JYY = 0.01420
JZZ = 0.02100


def load(stamp, kind):
    f = os.path.join(D, f'resid_{kind}_{stamp}.csv')
    if not os.path.exists(f):
        return None
    try:
        a = np.genfromtxt(f, delimiter=',', names=True)
        return a if a.size > 1 else None
    except Exception:
        return None


def stamps():
    # 文件名 resid_motor_<YYYYmmdd>_<HHMMSS>.csv —— 时间戳是**最后两段**,
    # split('_')[-1] 只会拿到 HHMMSS 而丢掉日期段,导致 load() 全部找不到文件。
    return sorted({os.path.basename(f)[len('resid_motor_'):-len('.csv')]
                   for f in glob.glob(os.path.join(D, 'resid_motor_*.csv'))})


def sg_deriv(t, y, win, order=2):
    """Savitzky-Golay 求导:局部多项式拟合后取解析导数。
    win 为奇数样本点。相比裸差分,把高频噪声压下去而不整体平移信号。"""
    n = len(y)
    d = np.full(n, np.nan)
    h = win // 2
    for i in range(h, n - h):
        tt = t[i - h:i + h + 1] - t[i]
        yy = y[i - h:i + h + 1]
        try:
            c = np.polyfit(tt, yy, order)
        except Exception:
            continue
        d[i] = c[-2]        # 一阶项系数 = 该点导数
    return d


def resid_unloaded(stamp, dt_shift=0.0, sg_win=0):
    """空载段残差 r = tau_act - om x (J*om) - J*om_dot。
    dt_shift: 把 omega 时间轴平移 dt_shift 秒后再与 tau 对齐(延迟扫描用)。"""
    mo, imu, inte = load(stamp, 'motor'), load(stamp, 'imu'), load(stamp, 'internal')
    if mo is None or imu is None:
        return None
    t_m = mo['t_recv_ns'] / 1e9
    t_i = imu['t_recv_ns'] / 1e9 + dt_shift
    # 只取空载段:internal 流的 payload_attached==0
    if inte is not None:
        t_int = inte['t_recv_ns'] / 1e9
        att = inte['payload_attached']
        if np.any(att > 0):
            t_cut = t_int[np.argmax(att > 0)]     # 首次 attach 时刻
            m_ok = t_m < t_cut
            i_ok = t_i < t_cut
            t_m, mo = t_m[m_ok], mo[m_ok]
            t_i, imu = t_i[i_ok], imu[i_ok]
    if len(t_i) < 60 or len(t_m) < 60:
        return None
    om = np.vstack([imu['wx'], imu['wy'], imu['wz']])
    # omega_dot:SG 或裸差分
    if sg_win >= 5:
        od = np.vstack([sg_deriv(t_i, om[k], sg_win) for k in range(3)])
    else:
        od = np.vstack([np.gradient(om[k], t_i) for k in range(3)])
    J = np.array([JXX, JYY, JZZ])[:, None]
    Jom = J * om
    gyro = np.cross(om.T, Jom.T).T                # om x (J*om)
    rhs = gyro + J * od                           # 模型要求的力矩
    # 把 rhs 插值到 motor 时刻(tau 是 250Hz,信息更全)
    good = np.all(np.isfinite(rhs), axis=0)
    if good.sum() < 40:
        return None
    rhs_m = np.vstack([np.interp(t_m, t_i[good], rhs[k][good]) for k in range(3)])
    tau = np.vstack([mo['tau_x'], mo['tau_y'], mo['tau_z']])
    r = tau - rhs_m
    return dict(t=t_m, r=r, tau=tau, rhs=rhs_m)


def main():
    ss = stamps()
    print(f'数据目录: {D}\n轮次数: {len(ss)}\n')
    print('=' * 74)
    print('step1  时钟域检查')
    print('=' * 74)
    s0 = ss[0]
    for kind in ('motor', 'imu', 'odom', 'command', 'internal'):
        a = load(s0, kind)
        if a is None:
            print(f'  {kind:<9} 无数据'); continue
        rc = a['t_recv_ns'] / 1e9
        hz = 1 / np.median(np.diff(rc)) if len(rc) > 2 else float('nan')
        if 't_header_ns' in a.dtype.names:
            h = a['t_header_ns']
            nv = int(np.sum(h > 0))
            off = (h[0] / 1e9 - rc[0]) if nv else float('nan')
            print(f'  {kind:<9} {hz:6.1f}Hz  header有效 {nv}/{len(h)}  '
                  f'header-recv偏移 {off:+.1f}s')
        else:
            print(f'  {kind:<9} {hz:6.1f}Hz  (无 header 字段,只有接收时刻)')
    print('  → 结论:跨流对齐只能用 t_recv_ns(同进程单调钟,天然同域);'
          'header 是 Unix 钟,与之相差一个常数偏移')

    print('\n' + '=' * 74)
    print('step2  延迟扫描 argmin_dt RMS(r)   [空载段,裸差分]')
    print('=' * 74)
    grid = np.arange(-0.10, 0.101, 0.005)
    for s in ss[:6]:
        rms = []
        for dt in grid:
            out = resid_unloaded(s, dt_shift=dt)
            rms.append(np.sqrt(np.mean(out['r'][1] ** 2)) if out else np.nan)
        rms = np.array(rms)
        if np.all(np.isnan(rms)):
            print(f'  {s}: 无有效空载段'); continue
        k = int(np.nanargmin(rms))
        print(f'  {s}: 最优 dt={grid[k]*1000:+6.1f} ms  '
              f'RMS {np.nanmin(rms)*1000:7.3f} mNm  '
              f'(dt=0 时 {rms[len(grid)//2]*1000:7.3f} mNm, '
              f'改善 {(1-np.nanmin(rms)/rms[len(grid)//2])*100:4.1f}%)')

    print('\n' + '=' * 74)
    print('step3  导数估计敏感性  [空载段, dt=0]')
    print('=' * 74)
    for s in ss[:4]:
        row = [f'  {s}:']
        for win, lab in ((0, '裸差分'), (5, 'SG5'), (9, 'SG9'), (15, 'SG15')):
            out = resid_unloaded(s, sg_win=win)
            row.append(f'{lab}={np.sqrt(np.mean(out["r"][1]**2))*1000:6.3f}'
                       if out else f'{lab}=n/a')
        print('  '.join(row) + '  mNm')
    print('  → 残差若随窗口显著单调下降,说明它主要是微分噪声,不是模型失配')


# =====================================================================
#  step4 残差结构:常数偏置? 线性可预测? 跨轮稳定?
# =====================================================================
DT_ALIGN = -0.025    # step2 六轮一致的链路时延差,后续分析一律先补偿
SG_WIN = 5           # step3 定案后固定,不再调(避免制造额外自由度)
M_NOM = 2.064


def load_manifest(path=None):
    """stamp -> (traj, mass, ecc, rep)。诊断分桶与留出全靠它。"""
    if path is None:
        c = sorted(glob.glob('/home/clear/ros2_ws_HJH/nmpc_test_results/'
                             'residual_collect_*.txt'), key=os.path.getmtime)
        if not c:
            return {}
        path = c[-1]
    rows = []
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        p = line.split()
        if len(p) >= 6 and p[5] == 'ok':
            rows.append((p[4], dict(traj=p[0], mass=float(p[1]),
                                    ecc=float(p[2]), rep=p[3])))
    # ⚠️manifest 的 stamp 是 headless 生成的**轮次** stamp,而 CSV 的 stamp 是
    # ResidualLogger 在 mhe_node 启动时自己生成的,两者差一个固定的启动耗时
    # (实测稳定 +31~32s)。故按"CSV stamp 之前、时间最近的 manifest 轮次"匹配,
    # 不能直接用 stamp 相等。(根因已在 residual_logger 侧修掉:改用外部传入
    # stamp;此分支保留是为了让**已采的这 20 轮**仍可分析。)
    import datetime as _dt
    _p = lambda s: _dt.datetime.strptime(s, '%Y%m%d_%H%M%S')
    rows.sort(key=lambda kv: kv[0])
    out = {}
    for cs in stamps():
        best, bestdt = None, None
        for ms, cond in rows:
            d = (_p(cs) - _p(ms)).total_seconds()
            if 0 <= d <= 120 and (bestdt is None or d < bestdt):
                best, bestdt = cond, d
        if best:
            out[cs] = best
    return out


def residual_full(stamp, cond):
    """完整残差,含带载段几何补偿。
       r = tau_act + tau_thrust_com - om x (J*om) - J*om_dot
       tau_thrust_com = [-cy*T, +cx*T, 0];  J = [Jxx+dJ, Jyy+dJ, Jzz]
       dJ/c 用 mhe_node._payload_geometry 同一公式(平行轴定理+复合质心),
       m_p 取真值(采集时 GRIP_GEOM_MP_PRIOR=真值),r_p 取 internal 流的 attach_offset。
    """
    mo, imu, inte = load(stamp, 'motor'), load(stamp, 'imu'), load(stamp, 'internal')
    if mo is None or imu is None:
        return None
    t_m = mo['t_recv_ns'] / 1e9
    t_i = imu['t_recv_ns'] / 1e9 + DT_ALIGN
    om = np.vstack([imu['wx'], imu['wy'], imu['wz']])
    od = np.vstack([sg_deriv(t_i, om[k], SG_WIN) for k in range(3)])
    good = np.all(np.isfinite(od), axis=0)
    if good.sum() < 60:
        return None
    om_m = np.vstack([np.interp(t_m, t_i[good], om[k][good]) for k in range(3)])
    od_m = np.vstack([np.interp(t_m, t_i[good], od[k][good]) for k in range(3)])
    tau = np.vstack([mo['tau_x'], mo['tau_y'], mo['tau_z']])
    T = mo['T_phys']

    # 带载与否 + 几何:从 internal 流取,按时间最近邻贴到 motor 时刻
    att = np.zeros(len(t_m), dtype=bool)
    dJ = np.zeros(len(t_m)); cx = np.zeros(len(t_m)); cy = np.zeros(len(t_m))
    if inte is not None and np.any(inte['payload_attached'] > 0):
        t_int = inte['t_recv_ns'] / 1e9
        a_int = inte['payload_attached'] > 0
        idx = np.clip(np.searchsorted(t_int, t_m) - 1, 0, len(t_int) - 1)
        att = a_int[idx]
        rx = np.nan_to_num(inte['att_off_x'][idx])
        ry = np.nan_to_num(inte['att_off_y'][idx])
        rz = np.nan_to_num(inte['att_off_z'][idx])
        m_p = cond['mass']                       # 采集用 truth 先验
        m_t = M_NOM + m_p
        mu = M_NOM * m_p / m_t
        dJ = np.where(att, mu * (rz**2 + 0.5 * (rx**2 + ry**2)), 0.0)
        cx = np.where(att, (m_p / m_t) * rx, 0.0)
        cy = np.where(att, (m_p / m_t) * ry, 0.0)

    J = np.vstack([JXX + dJ, JYY + dJ, np.full(len(t_m), JZZ)])
    Jom = J * om_m
    gyro = np.cross(om_m.T, Jom.T).T
    tau_com = np.vstack([-cy * T, cx * T, np.zeros(len(t_m))])
    r = tau + tau_com - gyro - J * od_m
    return dict(t=t_m, r=r, tau=tau, om=om_m, od=od_m, T=T, att=att,
                dJ=dJ, cond=cond)


def _stats(r):
    return np.mean(r), np.std(r), np.sqrt(np.mean(r ** 2))


def step4():
    man = load_manifest()
    ss = [s for s in stamps() if s in man]
    print('\n' + '=' * 74)
    print(f'step4  残差结构诊断  (对齐 dt={DT_ALIGN*1000:+.0f}ms, SG{SG_WIN} 固定)')
    print('=' * 74)
    if not ss:
        print('  无可用轮次(manifest 未匹配)'); return

    data = {}
    for s in ss:
        out = residual_full(s, man[s])
        if out:
            data[s] = out
    print(f'  可用轮次 {len(data)}/{len(ss)}\n')

    # --- 4.1 分桶统计 + Baseline0(去均值)降幅 ---
    print('--- 4.1 分桶: 每轴 mean/std/RMS 与 Baseline0(去常数偏置)降幅 ---')
    print(f'  {"分组":<22}{"轴":<5}{"mean":>9}{"std":>9}{"RMS":>9}'
          f'{"去均值RMS":>11}{"降幅":>8}   (mNm)')
    buckets = {}
    for s, d in data.items():
        for load_lab, mask in (('空载', ~d['att']), ('带载', d['att'])):
            if mask.sum() < 50:
                continue
            key = f"{d['cond']['traj']}/{load_lab}"
            buckets.setdefault(key, {0: [], 1: []})
            for ax in (0, 1):
                buckets[key][ax].append(d['r'][ax][mask])
    for key in sorted(buckets):
        for ax, nm in ((0, 'roll'), (1, 'pitch')):
            v = np.concatenate(buckets[key][ax]) if buckets[key][ax] else None
            if v is None or len(v) < 50:
                continue
            m, sd, rms = _stats(v)
            rms0 = np.sqrt(np.mean((v - m) ** 2))
            print(f'  {key:<22}{nm:<5}{m*1000:9.3f}{sd*1000:9.3f}{rms*1000:9.3f}'
                  f'{rms0*1000:11.3f}{(1-rms0/rms)*100:7.1f}%')

    # --- 4.2 相关性(残差 vs 状态/输入) ---
    print('\n--- 4.2 相关性 corr(r, feature)  [pitch 轴, 各轮独立] ---')
    feats = ['tau_y', 'od_y', 'om_y', 'T', 'om_y|om_y|']
    print(f'  {"轮次":<18}{"工况":<16}' + ''.join(f'{f:>12}' for f in feats))
    corr_by_cond = {}
    for s in sorted(data):
        d = data[s]
        r = d['r'][1]
        F = {'tau_y': d['tau'][1], 'od_y': d['od'][1], 'om_y': d['om'][1],
             'T': d['T'], 'om_y|om_y|': d['om'][1] * np.abs(d['om'][1])}
        cs = []
        for f in feats:
            x = F[f]
            ok = np.isfinite(x) & np.isfinite(r)
            cs.append(np.corrcoef(x[ok], r[ok])[0, 1] if ok.sum() > 50 and
                      np.std(x[ok]) > 1e-12 else np.nan)
        lab = f"{d['cond']['traj']}/{d['cond']['mass']}/{d['cond']['ecc']}"
        corr_by_cond.setdefault(d['cond']['traj'], []).append(cs)
        print(f'  {s:<18}{lab:<16}' + ''.join(f'{c:12.3f}' for c in cs))

    print('\n--- 4.3 跨轮稳定性: 相关系数的符号一致率 ---')
    for traj, rows in corr_by_cond.items():
        A = np.array(rows)
        print(f'  {traj}:')
        for j, f in enumerate(feats):
            col = A[:, j]
            col = col[np.isfinite(col)]
            if len(col) < 2:
                continue
            same = max((col > 0).sum(), (col < 0).sum()) / len(col)
            print(f'    {f:<12} 均值 {np.mean(col):+.3f}  '
                  f'范围[{col.min():+.3f},{col.max():+.3f}]  '
                  f'同号率 {same*100:.0f}% ({len(col)}轮)')


if __name__ == '__main__':
    if '--step4' in sys.argv:
        step4()
    else:
        main()
