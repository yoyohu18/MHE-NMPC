#!/usr/bin/env python3
"""转动 lumped 扰动通道 xi 的离线验证(2026-08-25),纯 numpy 无 ROS/acados。

验证的命题:**xi 能把"dJ 先验给错"造成的 om_dot 误差吸收掉**——这正是补齐
Hanover σ_m 转动三维的目的(去掉 NMPC 对 dJ 真值的依赖)。

物理设定:真实转动惯量 J_true = J_nom + dJ(带载),而标称模型只知道 J_nom(空机)。
同一个力矩 τ 下:
    ω̇_true = τ/J_true      (真实)
    ω̇_nom  = τ/J_nom       (L1 预测器用的标称)
两者之差就是 xi 该估出来的量:
    xi_∞ = τ·(1/J_true − 1/J_nom)          <0(真实惯量更大 → 真实角加速度更小)

跑: python3 src/offboard_test_acados/test/test_tau_lumped_standalone.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from offboard_test_acados.l1_adaptive import L1Augmentation  # noqa: E402

J_NOM = 0.0142          # 空机 Jxx(acados_params)
DJ = 0.3 * 0.47 ** 2    # 0.3kg 载荷 @0.47m 臂 = 0.06627 kg·m²
J_TRUE = J_NOM + DJ
DT = 1.0 / 31.0         # odom 实测频率 ~31Hz(不是控制周期)


def simulate(tau, secs=12.0, a_gain=10.0, omega_c=0.5, d_max=40.0):
    """单轴仿真:真实系统按 J_TRUE 演化,L1 拿 J_NOM 的标称加速度去估 xi。"""
    l1 = L1Augmentation(a_gain=a_gain, omega_c=omega_c, dim=1, d_max=d_max)
    om = 0.0
    hist = []
    for _ in range(int(secs / DT)):
        om_dot_true = tau / J_TRUE            # 单轴,陀螺项 ω×Jω 恒为 0
        om += om_dot_true * DT
        om_dot_nom = tau / J_NOM              # 标称模型(不知道 dJ)
        xi = l1.update(np.array([om]), np.array([om_dot_nom]), DT)
        hist.append(xi[0])
    return np.array(hist)


def main():
    ok = True

    # --- 1) 常值力矩:xi 收敛到理论补偿量 ---
    print('=== 1) xi 收敛性(场景:NMPC 与估计器**都**只知道空机 J) ===')
    print('    注意这是 J_model == J_nom 的场景——先验完全没给,xi 补全部 dJ。')
    for tau in (0.02, 0.05, 0.10):
        xi_inf = tau * (1.0 / J_TRUE - 1.0 / J_NOM)
        h = simulate(tau)
        xi_hat = h[-1]
        err = abs(xi_hat - xi_inf) / abs(xi_inf) * 100
        good = err < 5.0
        ok &= good
        print(f'  tau={tau:.3f}Nm  理论 xi_inf={xi_inf:+8.3f}  '
              f'估计={xi_hat:+8.3f} rad/s²  误差={err:5.2f}%  '
              f'{"OK" if good else "**FAIL**"}')

    # --- 2) 限幅够不够:tau_max 下需要多大的 xi ---
    print('\n=== 2) xi_max 该设多大(dJ=0.0663 是 J_nom 的 4.7 倍) ===')
    for tau in (0.1, 0.2, 0.35, 0.5):
        need = abs(tau * (1.0 / J_TRUE - 1.0 / J_NOM))
        print(f'  tau={tau:.2f}Nm(={tau/0.5*100:3.0f}% of tau_max) '
              f'→ 需要 |xi|={need:6.2f} rad/s²'
              f'{"   ← 会被 xi_max=20 截断" if need > 20 else ""}')
    need_max = abs(0.5 * (1.0 / J_TRUE - 1.0 / J_NOM))
    print(f'  ⇒ tau_max=0.5 时需要 {need_max:.1f} rad/s²,'
          f'xi_max 至少要 {int(np.ceil(need_max / 10) * 10)}')

    # --- 3) 时间尺度:多久跟上(omega_c 是唯一性能旋钮) ---
    print('\n=== 3) 收敛时间 vs omega_c(tau=0.05Nm) ===')
    tau = 0.05
    xi_inf = tau * (1.0 / J_TRUE - 1.0 / J_NOM)
    for wc in (0.5, 1.0, 2.0):
        h = simulate(tau, omega_c=wc)
        idx = np.where(np.abs(h - xi_inf) < 0.1 * abs(xi_inf))[0]
        t90 = idx[0] * DT if len(idx) else float('nan')
        print(f'  omega_c={wc:.1f} rad/s → 进入 ±10% 带用时 {t90:5.2f}s')

    # --- 4) drop 语义:扰动阶跃回零后 xi 必须自己回来 ---
    print('\n=== 4) drop 后 xi 回零(不 reset 也应自行衰减) ===')
    l1 = L1Augmentation(a_gain=10.0, omega_c=0.5, dim=1, d_max=40.0)
    om, xs = 0.0, []
    for k in range(int(20.0 / DT)):
        J_now = J_TRUE if k * DT < 10.0 else J_NOM      # t=10s 掉包
        om += (0.05 / J_now) * DT
        xs.append(l1.update(np.array([om]), np.array([0.05 / J_NOM]), DT)[0])
    xs = np.array(xs)
    tail = abs(xs[-1])
    good = tail < 0.5
    ok &= good
    print(f'  drop 前 xi={xs[int(9.5/DT)]:+.3f}, drop 后 10s xi={xs[-1]:+.3f} '
          f'rad/s²  {"OK(已回零)" if good else "**FAIL(有残留)**"}')

    # --- 5) 闭环稳定性:标称模型必须与控制器所用模型一致 ---
    # 2026-08-25 首版 SITL 当场坠机(grip_nmpc_003811,pos_err 70m/1094 次 solve
    # failed)的机制,固化成可执行检查。前四项**全过却没抓到它**——它们测的是
    # 孤立的估计器,而这个 bug 只在"估计器+控制器"回路里存在。
    #
    # 回路:NMPC 认为 ω̇ = τ/J_model + xi,要达成 u_des ⇒ τ = J_model·(u_des − xi);
    #      真实 ω̇ = τ/J_true;估计器用 J_est 算标称 ⇒ xi → τ·(1/J_true − 1/J_est)。
    # 代入得循环增益 = |J_model/J_est − 1|:
    #   J_est = J_model(对齐)   → 增益 0,xi→0,稳
    #   J_est = J_nom (首版 bug) → 增益 |0.0805/0.0142−1| = 4.7 ≫ 1,发散
    print('\n=== 5) 闭环:估计器标称 J 必须 == 控制器模型 J ===')
    def closed_loop(J_est, J_model, J_true, secs=25.0, amp=3.0):
        """带**持续激励**的闭环。第一版这里写成 u_des=−3·om 的纯镇定律,ω 很快
        被拉到 0 ⇒ τ→0 ⇒ xi→0,于是 bug 版也"收敛",测试给出假阳性。真实 8 字
        轨迹的力矩需求是持续的,所以这里用外部给定的角加速度需求(周期 22.2s
        = 新 4m/s 工作点的 8 字周期)驱动,τ 不会归零。"""
        l1 = L1Augmentation(a_gain=10.0, omega_c=0.5, dim=1, d_max=40.0)
        om, xi, peak_xi, peak_tau = 0.0, 0.0, 0.0, 0.0
        n = int(secs / DT)
        for k in range(n):
            u_des = amp * np.sin(2 * np.pi * (k * DT) / 22.2)
            tau = J_model * (u_des - xi)        # NMPC 按自己的模型反解力矩
            om += (tau / J_true) * DT           # 真实系统按 J_true 演化
            xi = l1.update(np.array([om]),
                           np.array([tau / J_est]), DT)[0]
            if not np.isfinite(xi):
                return float('inf'), float('inf')
            peak_xi = max(peak_xi, abs(xi))
            peak_tau = max(peak_tau, abs(tau))
        return peak_xi, peak_tau

    TAU_MAX = 0.5                                # acados_params.tau_max
    pk_bad, tau_bad = closed_loop(J_NOM, J_TRUE, J_TRUE)
    pk_ok, tau_ok = closed_loop(J_TRUE, J_TRUE, J_TRUE)
    print(f'  首版(J_est=J_nom):  循环增益 {abs(J_TRUE/J_NOM-1):.1f}  '
          f'xi 峰值 {pk_bad:9.1f}  tau 峰值 {tau_bad:9.2f}Nm '
          f'({tau_bad/TAU_MAX:5.1f}x tau_max)')
    print(f'  修正(J_est=J_model):循环增益 {0.0:.1f}  '
          f'xi 峰值 {pk_ok:9.3f}  tau 峰值 {tau_ok:9.4f}Nm '
          f'({tau_ok/TAU_MAX:5.2f}x tau_max)')
    # 判据用**绝对**阈值,不比相对大小(相对判据会把"两边都收敛"也判成通过):
    #   修正版:tau 必须留在执行器极限内、xi 不能逼近限幅
    #   首版  :tau 必须真的打爆 tau_max(SITL 实测到了 2.9Nm = 5.8x)
    good = (tau_ok < TAU_MAX and pk_ok < 5.0) and (tau_bad > 2.0 * TAU_MAX)
    ok &= good
    print(f'  → 修正版留在极限内 且 首版确实打爆执行器: '
          f'{"OK" if good else "**FAIL**"}')

    # 先验给错(而非完全不给)时仍须稳:增益 = |J_model/J_true − 1|
    print('\n  先验给错时的循环增益(修正版,J_est 始终 == J_model):')
    prior_ok = []
    for err in (-0.3, -0.2, 0.2, 0.5):
        J_m = J_TRUE * (1.0 + err)
        pk, tau_pk = closed_loop(J_m, J_m, J_TRUE)
        stable = tau_pk < TAU_MAX and pk < 5.0
        prior_ok.append(stable)
        print(f'    dJ 先验错 {err:+.0%} → 增益 {abs(J_m/J_TRUE-1):.2f}  '
              f'xi 峰值 {pk:6.3f}  tau 峰值 {tau_pk:.3f}Nm  '
              f'{"稳" if stable else "**超限**"}')

    ok &= all(prior_ok)
    print('\n' + ('全部通过' if ok else '** 有失败项 **'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
