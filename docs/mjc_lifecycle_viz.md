---
title: viz 场景下 m / J / c 的全生命周期
tags: [MHE, NMPC, gripper, payload]
基准: feature/noInitialMass @ 6419cbe (2026-09-04)
---
# viz 场景下 m / J / c 的全生命周期

按 `scripts/gripper/run_sitl_gripper_viz.sh` 的**当前默认档位**，逐阶段说明质量 $m$、
一阶质量矩 $s = m_P r_{xy}$、惯量增量 $\Delta J$、复合质心水平偏移 $c_{xy}$ 这几个量在
**MHE** 与 **NMPC** 两侧分别怎么变、被谁写、被谁用。

阶段：**ATTACH → 巡航（LIFT + 悬停）→ 八字 → DROP → DROP 后**。

> [!abstract] 2026-09-04 的接口大改（本次改写的主因）
> `6419cbe` 把 MHE→NMPC 换成**单条原子连续接口** `/mhe/payload_estimate`，并把
> 一阶质量矩默认打开。相对本文上一版（基准 `ea899ae`），下列结论**已被推翻**：
>
> | 旧版说法                                                                                                                                                                 | 现状                                                                                     |
> | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- |
> | $m$ 走 `/mhe/mass_est`、$c$ 走 `/acados_nmpc/c_xy_est`、$J$ 不发话题 | 三者**同一帧**走 `/mhe/payload_estimate`（14 字段），$J$ 现在**是**发的 |                                                                                          |
> | attach 时$\Delta J$ 写 floor `0.0108`                                                                                                                                | 写**bootstrap 包线** `0.0579`，随后交接给 MHE                                    |
> | attach 改写 PX4`MC_*RATE_K`（×5.0），drop 复位                                                                                                                        | **不再改 PX4 参数**；改走 setpoint 侧 `omega_scale`                              |
> | 一阶质量矩`MHE_ESTIMATE_MOMENT` 默认关                                                                                                                                 | **默认开**（`moment_a_mode=frozen`）                                             |
> | $c_{xy}$ 在八字里完全冻结                                                                                                                                              | `c_xy_from_moment` 默认开 ⇒ **机动中每帧更新**                                  |
> | MHE 仍吃`attach_offset` 真值                                                                                                                                           | 主线档`_model_r_p` 只用 $r_z$ 弱先验，**attach/drop 都不进 MHE**               |
> | drop 同帧清几何、`grip_dropped=True`                                                                                                                                   | drop 进**pending**，等 `no_payload_confidence` 确认才 complete                   |
> | `self` 档 drop 后 $m_{est}$ 偏低 −2.81 %                                                                                                                            | 09-04 实跑**−0.27 ~ −0.66 %**，不触下界                                          |
> | M0 ⇒ 确认阈值固定 1.5 N                                                                                                                                                 | 脚本 M0 分支给$\alpha=1.5$ ⇒ **4.4 N**；`MHE_CONFIRM_ALPHA=-1` 这个覆盖已废弃 |
>
> 09-02 那批工况改动（0.15 kg / 包线 0.3 / $r$=10 / $z$=6.0 / 4 m/s）**仍然有效**，
> 与更早的批次不可直接对比。

> [!success] 死锁已修，释放判据经 12 轮 fastA-free smoke 验收
> `232444` 轮暴露的死锁（$conf$ 卡 0.694、`grip_dropped` 永不置位）已定位到
> "释放判据要求 $\lVert s\rVert$ 塌到峰值 10% 以下，比估计器噪声本底还低"。
> 修法是把 drop 侧改成**相对塌陷 0.20 + 质量/惯量佐证 + 0.5 s 持续**，
> **不动** `moment_full`（那是 confidence 标定，与状态转移解耦）、
> 也**不**要求飞回低机动。`234516` 轮纯默认档全程走通。见 §3.4。

---

## 0. 档位与常数

### 0.1 关键参数

**场景（工况）**

| 参数                                       | 值                        | 含义                                                     |
| ------------------------------------------ | ------------------------- | -------------------------------------------------------- |
| `GRIP_PAYLOAD_KG`                        | **0.15** kg         | 载荷真值（只用于评估对表）                               |
| `GRIP_PAYLOAD_ENVELOPE`                  | **0.3** kg          | 机架规格，**不是**包裹质量                         |
| `GRIP_ECC_Y` / `GRIP_ATTACH_TOL`       | 0.10 / 0.03 m             | attach 偏心上限$r_{xy}=0.13$                           |
| `GRIP_DYN_R` / `W` / `RAMP` / `DZ` | 10.0 / 0.283 / 9.36 / 0.8 | 八字，$v_{peak}=4.0$ m/s                               |
| `GRIP_Z_HIGH` / `LIFT_DUR`             | 6.0 m / 8.4 s             | 爬升率 0.649 m/s                                         |
| `GRIP_LIFT_AFTER`                        | **3.0** s           | ★ 09-04 由 1.5 改默认——1.5 会在 LIFT 段发散，见 §3.1 |
| `GRIP_DROP_AFTER` / `DROP_AT_TIP`      | 55.0 s / true             | 粗门 + 尖点对齐                                          |
| `GRIP_LIFT_HOLD`                         | false                     | LIFT 中途停顿档（默认关）                                |
| `METHOD`                                 | `M0`                    | ⇒$\theta=[-4,0,0,0]$、$\alpha=1.5$                  |

**NMPC（连续接口）**

| 参数                                                 | 值                             | 含义                                                              |
| ---------------------------------------------------- | ------------------------------ | ----------------------------------------------------------------- |
| `continuous_payload_estimates`                     | **`true`**             | ★`/mhe/payload_estimate` 是**唯一**载荷输入              |
| `geom_source`                                      | **`estimate`**         | 被上一条强制（写别的值会被覆盖）                                  |
| `NMPC_GEOM_COUPLED`                                | `0`                          | 连续档**要求**为 0，否则 `__init__` 直接 `RuntimeError` |
| `attach_j_bootstrap_enable`                        | **`true`**             | attach 时按包线抬$\Delta J$ 地板                                |
| `attach_j_bootstrap_confirm_sec` / `release_sec` | 0.5 / 0.5 s                    | 交接确认 / 淡出时长                                               |
| `attach_j_bootstrap_min_dj`                        | 0.010 kg·m²                  | 交接需要的最小 MHE$\Delta J$                                    |
| `omega_scale_enable` / `_source`                 | **`true`** / `djest` | ★ 增益调度走 setpoint 侧                                         |
| `omega_scale_tau` / `_cap`                       | 0.5 s / 5.0                    | 一阶低通 + 硬上限                                                 |
| `omega_scale_hysteresis` / `rise` / `recover`  | 0.08 / 4.0 / 1.5 s⁻¹         | 目标滞回 + 非对称 slew                                            |
| `omega_scale_headroom_fraction`                    | 0.95                           | 保 5 % 体速率余量                                                 |
| `scale_px4_rate_gains`                             | **`false`**            | ★ =`not continuous`；**PX4 参数不动**                    |
| `payload_model_tau_sec`                            | 0.20 s                         | 模型侧 LPF                                                        |
| `payload_mass_slew_kg_s` / `moment` / `dj`     | 0.60 / 0.080 / 0.080           | 每秒最大变化                                                      |
| `no_payload_confidence_threshold` / `_hold_sec`  | 0.90 / 1.0 s                   | drop 完成判据                                                     |
| `drop_unresolved_timeout_sec` | **12.0** s | ★ 超时 ⇒ unresolved（**不等于**已卸载，见 §3.4） |
| `drop_unresolved_stop_sec` | **3.0** s | ★ unresolved 后从当前状态平滑制动至停止点悬停 |
| `payload_estimate_fresh_sec`                       | 0.35 s                         | ROS 收帧新鲜度                                                    |

**MHE**

| 参数                                                    | 值                              | 含义                                                                                                   |
| ------------------------------------------------------- | ------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `external_event_inputs`                               | **`false`**             | ★ 不订阅`attach_offset` / `mass_event` / `u_opt`（sensor-only）                                 |
| `MHE_GEOM_COUPLED`                                    | `1`                           | 几何槽装$r_p$，$J(m)/c(m)$ 模型内现算                                                              |
| `MHE_ESTIMATE_MOMENT`                                 | **`1`**                 | ★ 一阶质量矩$s=[s_x,s_y]$ 增广为状态（`ns=2`）                                                    |
| `MHE_MOMENT_A_MODE`                                   | **`frozen`**            | ★$A=\mu r_z^2$ 用**上一窗口** $\hat m$ 现算，窗口内 $\partial A/\partial m\equiv 0$       |
| `MHE_SIGMA_S0`                                        | 0.1                             | $s$ 的到达代价（实质无先验）                                                                         |
| `MHE_C_XY_FROM_MOMENT`                                | **`true`**              | ★$c_{xy}=s/m_T$，**无稳态门控**                                                               |
| `event_signal_mode`                                   | `residual`                    | 无外部信号，从$T_{phys}$ 残差自触发                                                                  |
| `confirm_thresh_alpha`                                | **`1.5`**               | ⇒ 阈值$=\alpha g\,\text{env}=$ **4.4 N**，故意堵死残差路（§3.4）。**不要再覆盖成 −1** |
| `resid_persist_frames` / `resid_step_enable`        | 2 /`false`                    | 第二段确认 / 并行阶跃判据关                                                                            |
| `c_xy_mass_release_mp` / `arm_ratio` / 两个 persist | 0.03 kg / 3.0 / 20 / 20 帧      | 质量域卸载判据（武装 0.09 kg，各持 2 s）                                                               |
| `payload_present_enter_mp` / `_persist`             | 0.09 kg / 20 帧                 | 载荷存在状态机进                                                                                       |
| `payload_present_exit_mp` / `_persist`              | 0.02 kg / 50 帧                 | 出（5 s，更慢）                                                                                        |
| `s_release_ratio` / `_persist`                      | **0.20** / **5 帧** | ★ drop 侧**相对塌陷**判据（09-04 由 0.10/20 改，见 §3.4）                                      |
| `s_release_mass_score_min` / `_inertia_score_min`   | 0.90 / 0.90                     | ★ 塌陷判据的质量/惯量佐证（与$conf$ 同标定，但不碰 moment 通道）                                    |
| `s_release_persist` / `_strong_persist` | 5 / 3 帧 | slow / fastB 的持续要求 |
| `release_resid_enable` / `_floor_n` | `true` / 0.9 N | ★ 独立释放票（`RELEASE_RESID=0` = 消融档） |
| `release_resid_strong_persist` | **4** 帧 | 强票；靠比 figure-8 的 3 帧多一帧把它甩开 |
| `payload_input_fresh_sec` / `solution_fresh_sec`    | 0.30 / 0.30 s                   | `healthy` 的两个条件                                                                                 |
| `payload_output_tau_sec`                              | 0.25 s                          | 发布侧 LPF                                                                                             |
| `MHE_RZ_PRIOR`                                        | **−0.47** m              | 唯一保留的几何先验（真值实测 −0.579）                                                                 |
| `geom_release_mode`                                   | `self`                        | 主线下**对模型槽无效**（见 §3.1）                                                               |
| `MHE_RESID_RELEASE_GEOM`                              | `true`                        | 残差检出即释放几何                                                                                     |
| `payload_lost_watch`                                  | `false`                       | 演示视频用`1` 打开                                                                                   |

> [!warning] ⚠️ 脚本里还在传、但主线**不再消费**的参数
> 连续档下 `_update_online_geometry` 根本不被调用、`_geom_slot` 走 continuous 分支、
> `_grip_drop_phase` 走 pending 分支，所以这几个仍在命令行里的参数是**死参数**：
> `dj_track_mest`、`dj_ratchet_enable`、`grip_dj_floor_mp`、`grip_mp_cap`、
> NMPC 侧的 `geom_release_mode`、`drop_publish_mass_event`。
> 改它们不会改变任何行为——查问题时别在这几个上浪费时间。

### 0.2 物理常数与派生量

| 量                                                    | 值                                                    |
| ----------------------------------------------------- | ----------------------------------------------------- |
| 空机质量$m_B$                                       | 2.0643 kg                                             |
| 空机惯量$J_{xx}$                                    | 0.0142 kg·m²                                        |
| 惯量先验力臂$r_z$ (`MHE_RZ_PRIOR`/`grip_arm_d`) | −0.47 m（实测真值 −0.579）                          |
| 载荷真值$m_P$ / 总质量                              | 0.15 / 2.2143 kg                                      |
| 悬停推力$T$                                         | 21.72 N                                               |
| 力矩约束$\tau_{max}$                                | 0.5 N·m                                              |
| MHE 窗口                                              | $N\cdot dt = 20\times 0.1 = 2.0$ s                  |
| NMPC 求解                                             | $dt=0.05$ s（20 Hz），horizon 1.0 s；解耦发布 50 Hz |

连续接口的惯量代数（`payload_estimate.inertia_from_mass_moment`，**只留对角、丢掉
$O(r_{xy}^2)$ 项**）：

$$
c_{xy} = \frac{s}{m_T},\qquad
\mu = \frac{m_B\,m_P}{m_T},\qquad
\Delta J_{xx} = \Delta J_{yy} = \mu\, r_z^2,\qquad \Delta J_{zz} = 0
$$

| $m_p$ 来源                | $m_p$ [kg]      | $\Delta J$     | $J_{xx}+\Delta J$ | $\omega$ 缩放比         |
| --------------------------- | ----------------- | ---------------- | ------------------- | ------------------------- |
| 空机                        | 0                 | 0                | 0.0142              | 1.00                      |
| **实测 $\hat m_p$** | **≈0.128** | **0.0267** | 0.0409              | **2.88**            |
| 真值                        | 0.15              | 0.0309           | 0.0451              | 3.18                      |
| **bootstrap 包线**    | **0.30**    | **0.0579** | **0.0721**    | **5.08 → cap 5.0** |

配平力矩（$r_y$ 实测 −0.101 m）：

$$
\tau_{trim} = c_y T = m_P g\, r_y \approx 0.149\ \text{N·m} = 30\ \%\ \tau_{max}
$$

---

## 1. 三个量的存放与流向

```mermaid
flowchart LR
    MOT["电机转速 ω_i<br/>250Hz,窗口平均"] --> TP["T_phys, τ_phys"]
    ODO["odometry"] --> MHE
    TP --> MHE["MHE 窗口 N=20 @10Hz<br/>状态含 m 与 s=m_P·r_xy"]
    RZ["r_z 先验 −0.47<br/>(唯一几何先验)"] --> MHE
    MHE --> PE["/mhe/payload_estimate<br/>14 字段原子帧"]
    PE --> NM["NMPC: LPF τ=0.2s + slew"]
    BOOT["attach J bootstrap<br/>包线 0.3kg,仅 J"] --> NM
    NM --> MOD["model.p = [m, ΔJ, c_x, c_y]"]
    NM --> OS["omega_scale = (Jxx+ΔJ)/Jxx<br/>→ ω_cmd 缩放(setpoint 侧)"]
```

`/mhe/payload_estimate` 是 `std_msgs/Float64MultiArray`，字段顺序由
`payload_estimate.FIELDS` 唯一定义：

```text
[m_total, s_x, s_y, c_x, c_y,
 dJ_xx, dJ_yy, dJ_zz, J_xx, J_yy, J_zz,
 no_payload_confidence, healthy, solution_age_sec]
```

| 量           | MHE 侧                                                | NMPC 侧                                                           |
| ------------ | ----------------------------------------------------- | ----------------------------------------------------------------- |
| $m$        | 被估状态（窗口第 14 维）                              | `self.m_est`，LPF + slew 0.60 kg/s                              |
| $s$        | **被估状态**（`ns=2`），有独立观测 $\tau/T$ | `self.s_est`，slew 0.080 kg·m/s                                |
| $\Delta J$ | 由$m,s,r_z$ 代数派生后发布                          | `self.dJ_est`，slew 0.080 kg·m²/s，attach 期叠 bootstrap 地板 |
| $c_{xy}$   | $s/m_T$（发布字段；另有 `c_xy_est` 诊断话题）     | `self.c_est = s_est / m_est`                                    |

> [!question] 上一版说"$J$ 不该发话题"，为什么现在发了
> 旧论证的三条理由**在旧接口下仍然成立**，改变的是接口本身：
>
> 1. **不再是"零新增信息"**：$s$ 增广后 $J$ 不再是 $m$ 的确定性函数，$\Delta J$ 与
>    $c_{xy}$ 各自带着 $s$ 的信息。NMPC 只拿 $m$ 已经推不出来了。
> 2. **防护没有撤掉，只是换了形式**：`floor/cap` 换成 **LPF + slew + attach bootstrap
>    地板**，且失效帧（`healthy=0` 或 `solution_age>0.30 s`）会**冻住模型**而不是跟随。
> 3. **表示对上了**：MHE 的非对角项**不出接口**，发的是 roll/pitch 对角增量，
>    与 NMPC legacy 槽同构。
>
> 代价写在明处：$O(r_{xy}^2)$ 项两边都被**故意丢掉**（病态），$r_z$ 用弱先验
> −0.47 而真值 −0.579（低估 19 %）——$\Delta J$ 只需量级对。

---

## 2. 时间线

以 **attach 命令时刻** $T_a$ 为原点。注意这个锚点的定义变了：连续档下
NMPC **不订阅** `/gripper/attach_offset`，`attach_time` 就是它自己发
`/gripper/enable=true` 的那一刻（`grip_descend_done_time`），不再等夹爪回执。

| 相对时刻（`lift_after` = **3.0**，当前默认）                                                                    | 事件                                                                  | 性质           |
| ----------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- | -------------- |
| $T_a$                                                                                                                 | **ATTACH 命令**发出，J bootstrap 武装                           | 事件           |
| $T_a + 3.0$ | LIFT 斜坡开始，$z: 0.55 \to 6.0$                                                                      | 事件                                                                  |                |
| $T_a + \sim 4.5$ | box 离地；MHE 见到推力台阶（$m_P g = 1.47$ N）                                                   | 物理                                                                  |                |
| $T_a + \sim 5.0$                                                                                                      | **bootstrap 交接**（持续证据 0.5 s 确认）→ 0.5 s 淡出          | 状态           |
| $T_a + 11.4$                                                                                                          | `lift_done_t` = $T_a$ + 3.0 + 8.4                                 | **公式** |
| $T_a + 14.4$                                                                                                          | **DYNAMIC** 切换，`grip_dyn_t0` = 此刻（+`dyn_settle` 3.0） | 事件           |
| $T_a + 16.4$ | 八字 `hover_time` 2.0 结束，相位开始推进（$t_c=0$）                                                | —                                                                    |                |
| $T_a + 77.46$                                                                                                         | **DROP 命令**：第一个 $t_c \ge 50$ 的尖点（$t_c = 61.06$）  | 事件           |
| $T_a + ?$ | **DROP complete**：$conf \ge 0.90$ 持续 1.0 s —— 默认阈值档下的延迟**尚未实测**，见 §3.4 | 状态                                                                  |                |

### 2.1 DROP 门槛怎么算

粗门只说"哪一圈以后可以丢"，真正的时刻由八字相位定：

$$
t_c = t - t_0^{dyn} - 2.0, \qquad
t_c^{thr} = \texttt{drop\_after} - 5.0, \qquad
t_c^{tip}(k) = \frac{3\pi/2 + 2\pi k}{w}
$$

净偏移 5.0 = `dyn_settle 3.0` + `hover_time 2.0`；`lift_after` 与 `lift_dur`
**自动抵消**（$t_{drop}$ 与 $t_0^{dyn}$ 都锚在 `attach_time` 上）。所以把
改 `GRIP_LIFT_AFTER` **不必**重算 `DROP_AFTER`——它只让整条时间线平移
（09-04 从 1.5 改到 3.0，就是整体后移了 1.5 s）。

$w = 0.283$ ⇒ 周期 22.20 s：

| $k$       | $t_c^{tip}$   | 圈数                             |
| ----------- | --------------- | -------------------------------- |
| 0           | 16.65           | 0.75                             |
| 1           | 38.85           | 1.75                             |
| **2** | **61.06** | **2.75** ← 第一个 ≥ 50.0 |

> [!warning] ⚠️ 改 `GRIP_DYN_W` 必须重算 `GRIP_DROP_AFTER`
> 门槛写的是**秒**，兑现的是**圈数**，汇率就是 $w$。同一个 `55.0`：
> $w{=}0.20$ 只飞 **1.75 圈**，$w{=}0.283$ 飞 **2.75 圈**，$w{=}0.40$ 飞 **3.75 圈**。
> 当 $t_c^{thr}$ 贴近某个尖点时（本档临界 $w \approx 0.3456$），$w$ 微调会让 drop 时刻
> 在"立刻"与"再飞一整圈"之间跳变。
> 改 $r$、`ramp`、`dz`、`lift_dur`、`lift_after`、`z_high` **都不影响相位**。

### 2.2 drop 那一瞬间的状态

$a = 3\pi/2 \Rightarrow \sin a = -1,\ \cos a = 0,\ \cos 2a = -1$：

| 量            | 值                                                        | 说明                 |
| ------------- | --------------------------------------------------------- | -------------------- |
| $x$         | $c_x - 10.0$ m | 八字**最左端**（$x$ 的折返点） |                      |
| $y$         | $c_y$                                                   | 正好在中线           |
| $z$         | $6.0 - 0.8 = 5.2$ m                                     | **轨迹最低点** |
| $v_x,\ v_z$ | 0                                                         | 都正比于$\cos a$   |
| $v_y$       | $-2.83$ m/s                                             | $rw\cos 2a$        |

box 底离地 4.73 m，自由落体 0.98 s 落地，期间横移约 2.78 m。

### 2.3 2026-09-04 实跑对照（四轮）

> [!caution] 🔥 这三轮都带着一个**已废弃**的覆盖
> 它们都传了 `MHE_CONFIRM_ALPHA=-1`（确认阈值回到固定 1.5 N），于是残差自检测这条路
> 是**开着**的，drop 由它检出。默认档（4.4 N）把这条路堵死，**drop 的检出与确认时序
> 会不同**。下表中 `DROP complete` 一列因此不代表默认档；其余各列（时序、发散与否、
> solve failed、以及 §3.2 的估计精度）不受该覆盖影响。

| 轮次                 | `lift_after`              | ATTACH | LIFT | DYNAMIC | DROP 命令 | DROP complete     | solve failed  | 结局                                       |
| -------------------- | --------------------------- | ------ | ---- | ------- | --------- | ----------------- | ------------- | ------------------------------------------ |
| `162428`           | **1.5**（当时的默认） | 6.6    | 8.1  | 19.5    | 82.6      | —                | **822** | **发散**（tilt 141°、pos_err 70 m） |
| `163315`           | 3.0                         | 6.4    | 9.4  | 20.8    | 83.9      | +2.34 s           | 0             | 正常                                       |
| `164651`           | 3.0                         | 6.4    | 9.4  | 20.9    | 84.0      | +2.40 s           | 0             | 正常                                       |
| **`232444`** | **3.0（新默认）**     | 6.6    | 9.6  | 21.0    | 84.1      | **从未**    | 0             | 飞行正常，**drop 确认死锁**（§3.4） |
| **`234516`** | 3.0                         | 6.3    | 9.3  | 20.7    | 83.8      | **+3.55 s** | 0             | ★ 改造后纯默认档**全程走通**        |

`232444` 是**纯默认档**（不带任何覆盖，`thresh=4.4N`），专为补齐默认阈值档的
drop 时序而跑。飞行本身完全正常（0 solve failed、bootstrap 正常交接
0.0579→0.0207、载荷物理上确实卸掉了），但 `DROP complete` **在命令后 190 s
仍未出现**——原因见 §3.4 的死锁分析。

`234516` 是塌陷判据改造后的复跑，同样是纯默认档：DROP 命令 → 释放 **2.07 s**
→ complete **3.55 s**，释放理由
`|s|=0.0036 = 0.148x peak 0.0243 < 0.20 x5帧, mass_sc=1.00 inert_sc=1.00`。
对比带 `ALPHA=-1` 覆盖的 2.4 s，只慢约 1.1 s，而这一轮**没有任何外部信号**。

单位 s，相对 NMPC 接管时刻。$T_a$ 到 DROP 命令：`163315`/`164651` 实测 77.5 / 77.6 s
≈ 公式的 $3.0 + 74.46$；`162428` 实测 76.0 s ≈ $1.5 + 74.46$。两者都对上，
说明 `lift_after` 只是平移，相位推导没问题。

---

## 3. 逐阶段

### 3.1 阶段一：ATTACH（$T_a$）

触发链：`_descend_phase` 发 `/gripper/enable=true` → proximity 焊 box。
**这条链到此为止**：NMPC 不再订阅 attach 回执，MHE 也不订阅——`attach_offset`
在主线里只剩 proximity 自己的日志（评估对表用）。

#### NMPC（`_grip_mass_step`，continuous 分支）

| 量              | 动作                                                                                    |
| --------------- | --------------------------------------------------------------------------------------- |
| $m,\ s$       | **完全不动**，纯跟 MHE                                                            |
| $\Delta J$    | 武装**bootstrap**：地板 = 包线 0.3 kg ⇒ $\Delta J = 0.0579$、$J_{xx}=0.0721$ |
| $c_{xy}$      | 仍是$s_{est}/m_{est}$，**不置零、不注真值**                                     |
| `omega_scale` | 随$\Delta J$ 连续爬到 cap 5.0（实测 4.998）                                           |
| PX4 参数        | **不动**（`scale_px4_rate_gains=false`）                                        |

```python
self.grip_mass_stepped = True
self.attach_time = self.grip_descend_done_time   # 锚在自己的 enable 命令
self._start_attach_j_bootstrap()                 # 只抬 J，不造 m/s
```

> [!info] 为什么 bootstrap 只动 $J$
> 箱子还站在地上时，**惯量不可观测**（载荷的重量走地面支持力，转动通路没有激励），
> 而 LIFT 一开始就需要正确的内环带宽。所以给 $J$ 一个保守的、临时的**下界**——
> 用包线（机架规格，不是任务信息），并且**只给 $J$**：不注入载荷质量，也不注入
> $s_{xy}$，那两个继续纯估计。
>
> 交接条件三条同时满足：估计帧新鲜、$\Delta J_{MHE} \ge 0.010$、
> $conf < 0.75$（阈值 −0.15），持续 0.5 s ⇒ 地板按 0.5 s 线性淡出。
> 实测 attach 后 4.9~5.0 s 交接，$\Delta J$ 从 0.0579 → 0.0267。

> [!danger] bootstrap 期是当前最脆的一段
> 地板把内环增益推到 5×，**若此时 box 其实没吸稳**，等于给接近空机的构型上带载增益
> —— 就是 07-14 那个过增益振荡崩法。09-04 `162428` 轮（`lift_after`=1.5）实测
> LIFT 段直接发散：822 次 solve failed、tilt 141°、`om_scale` 钉在 5.0、
> MHE `health=0` 持续 21 s。`lift_after`=3.0 的两轮同一档位干净通过
> —— 脚本默认已据此从 1.5 改为 **3.0**。注意这只是把风险窗口挪开，
> 并没有消除它：吸附本身没有回执，`3.0` 是经验值不是保证。
>
> 还有一条设计上的冻结：**交接已开始、估计随后失效**时，代码选择**冻住**当前
> $\Delta J$（`if bootstrap_active and releasing and not fresh: return`），
> 而不是把地板放掉——放掉会复现原始的 LIFT 失效。代价是估计器垮掉时
> `om_scale` 会**停在高位**（`162428` 的 5.0 就是这么来的）。

#### MHE

| 量     | 动作                                                                                                                                            |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| $m$  | 不动，靠窗口自己爬                                                                                                                              |
| $s$  | 同样靠窗口爬；有独立观测$\tau/T$，不依赖 $m$ 收敛                                                                                           |
| 几何槽 | **恒为** $[0,0,r_z^{prior}]$ —— attach/drop 都不改它                                                                                  |
| 事件   | `residual` 自触发，但默认阈值 4.4 N ≫ 台阶 1.47 N ⇒ 本工况**不触发**（无降权）；`external_event_inputs=false` ⇒ 也没有任何外部武装 |

```python
def _model_r_p(self):
    if not self.external_event_inputs and mhe_p.estimate_moment:
        return np.array([0.0, 0.0, mhe_p.rz_prior])   # 主线：提前返回
    ...  # 下面 self/event 两档的老逻辑主线走不到
```

> [!important] ❗ `geom_release_mode` 在主线下是**空转**的
> 上面那个提前 `return` 意味着 `self` / `event` 两档的差别对**模型几何槽**不再生效：
> 横向几何全部由被估的 $s$ 承担，纵向只有 $r_z$ 弱先验，且**永远在线**。
> 启动日志里那条 `geom_release_mode='self'` 的警告仍会打印，但它描述的是老路径。
> `_release_payload` 本身还在用——它现在管的是**发布侧的 $s$ 释放闩**（§3.4）。

耦合档模型内部（与 `mhe_model.py` 逐字同源）：

$$
m_P = (m - m_B)^+,\quad \mu = \frac{m_B m_P}{m},\quad
A = \mu r_z^2\ \text{(frozen)},\quad c = \frac{s}{m}
$$

> [!note] ✏️ `moment_a_mode=frozen` 是什么意思
> $A=\mu r_z^2$ 用**上一窗口解出的** $\hat m$ 现算（借 `geom[0]` 槽传进来）。
> 于是窗口内 $\partial A/\partial m \equiv 0$ —— 优化器**不能把 $m$ 当惯量旋钮**；
> 但 $A$ 的数值每拍仍跟着真实质量刷新，不像 `const` 那样被包线钉死
> （0.15 kg 载荷配 0.3 kg 包线 = $A$ 过估 2×）。
> 离线 attach/drop 回放：`coupled` 档 drop 后 $m$ 偏高 **+53 %**，`frozen` 回到空机
> **−0.34 %**，$s/\Delta J$ 同步衰减、无下界触碰、solve fails = 0。

---

### 3.2 阶段二：巡航（LIFT + 悬停）

这是全程 $m$、$s$ 信息质量最好的一段：box 已离地（载荷全额加载），飞机还没进大机动。
本档 LIFT 长达 8.4 s（爬到 6 m）。

NMPC 每次 solve 前调 `_update_continuous_payload_model`：

```python
alpha = 1 - exp(-dt / 0.20)                      # LPF τ=0.2s
m_est  = slew(m_est,  lpf(m_target),  0.60, dt)  # kg/s
s_est  = slew(s_est,  lpf(s_target),  0.080, dt) # kg·m/s
dj_tgt = dj_mhe + w_bootstrap * max(dj_floor - dj_mhe, 0)
dJ_est = slew(dJ_est, lpf(dj_tgt),    0.080, dt) # kg·m²/s
c_est  = s_est / m_est                           # 横向质心只认一阶质量矩
```

> [!important] ❗ 失效帧的语义是"冻住"，不是"归零"
> `_payload_estimate_is_fresh()` 要求 MHE 自报 `healthy=1`、
> `solution_age <= 0.35 s`、且 ROS 收帧不陈旧。任何一条不满足：
> **保持上一帧模型**，并且**打断** `no_payload_confidence` 的连续计时
> （置零而不是暂停）。日志里是 `[payload-health] unhealthy/stale; holding model`。

#### 09-04 实测（`164651`，带载段）

| 量           | 估计                      | 真值      | 偏差                     |
| ------------ | ------------------------- | --------- | ------------------------ |
| $\hat m_p$ | 0.123 ~ 0.132 kg          | 0.15      | **低估 ~14 %**     |
| $\hat s_y$ | −0.0149 ~ −0.0154 kg·m | −0.01515 | **2 % 以内**       |
| $\hat s_x$ | −0.005 ~ −0.011 kg·m   | +0.0006   | 系统性偏负 ~0.01         |
| $\Delta J$ | 0.0267                    | 0.0309    | 低估 14 %（跟着$m_p$） |

（真值来自 proximity 的 `attach offset = [+0.004, −0.101, −0.579] m`，
$s_{true} = m_P r_{xy}$。）

> [!note] ✏️ $s$ 准而 $m$ 低估——两件事，别混
> $s_y$ 准到 2 % 以内，说明**横向一阶矩这条观测通路是干净的**（$\tau/T$ 直接给它）。
> 而 $m$ 低估 14 % 是既有的"机动中质量低估"问题在小载荷上的放大
> （$m_p = \hat m - m_B$ 的差分放大 ~7.9×，见记忆 `mest-maneuver-underestimate`）。
> 因为 $c = s/m$，$m$ 低估会让 $c$ **高估** ~14 %，反解出的 $\hat r_y \approx -0.116$ m
> 而真值 −0.101 —— 力臂的表观误差是质量误差借道过来的，不是几何估歪了。
> $s_x$ 那 0.01 kg·m 的偏置对应 $c_x \approx -4.6$ mm，与既有的 pitch 通道污染
> （$c_z\times$前倾）同源。

---

### 3.3 阶段三：八字（$v_{peak} = 4.00$ m/s）

轨迹：$x = \alpha r\sin a$，$y = \tfrac12\alpha r\sin 2a$，$z = z_h + \alpha\,dz\sin a$，$a = wt_c$。
$r{=}10.0$，$w{=}0.283$，$dz{=}0.8$ ⇒ 包络 20 m × 10 m。

| 量                                                       | 这一段的行为                                     |
| -------------------------------------------------------- | ------------------------------------------------ |
| $m$                                                    | 照常 10 Hz 更新（机动会抬高估计误差，通路不变）  |
| $s$                                                    | **照常更新**——窗口内被估状态，与稳态无关 |
| $\Delta J$                                             | 跟随$(m,s)$，双向，LPF + slew                  |
| $c_{xy}$ | **每帧更新**（$= s/m_T$），不再冻结 |                                                  |

> [!success] $c_{xy}$ 的冻结问题已经解决
> 上一版记的坑是"峰值速度 4.00 m/s 是稳态门限 0.20 m/s 的 20 倍 ⇒ 八字里
> $c_{xy}$ 一次都不更新，drop 后 60 s 仍报 $c_y=-4.8$ mm"。
> `MHE_C_XY_FROM_MOMENT=true` 后走的是另一条路：
>
> ```python
> if self.c_xy_from_moment and mhe_p.ns:
>     self.c_xy_est = self.s_est / max(m_est, m_min)   # 无门控,直接发
>     return
> ```
>
> 窗外 EMA + 稳态门控那条老路整段被跳过。启动日志会打印
> `[c_xy] 来源 = 一阶质量矩 s/m_T(窗口内估计,**无稳态门控**)`。
> 注意：NMPC 主线**不订阅** `/acados_nmpc/c_xy_est`（它从原子帧拿 $c$），
> 这条话题现在只是诊断/兼容出口。

#### 内环增益：从改 PX4 参数改成缩 setpoint

|        | 旧（≤09-02）                                        | 新（主线）                                    |
| ------ | ---------------------------------------------------- | --------------------------------------------- |
| 手段   | `ParamSetV2` 改写 `MC_ROLLRATE_K/MC_PITCHRATE_K` | 缩放发给 PX4 的`body_rate` setpoint         |
| 时机   | attach 一次性，drop 复位                             | **每帧连续**（`_update_omega_scale`） |
| 副作用 | 参数被 PX4 当真机落盘，污染后续架次                  | 无；PX4 参数全程不动                          |

$$
s_\omega = \mathrm{clip}\!\left(\frac{J_{xx}+\Delta J}{J_{xx}},\,1,\,5\right),
\qquad
\omega_{cmd}' = \omega + s_\omega\,(\omega_{cmd} - \omega)
$$

只作用 roll/pitch。目标带 0.08 滞回（Schmitt 式死区），再过 $\tau=0.5$ s 低通，
上行 slew 4.0 s⁻¹、下行 1.5 s⁻¹（恢复要慢）。发布前还有一道
`headroom_limited_scale`：把公共比例压到不会顶穿 $0.95\,\omega_{max}$，
方向不变、下限 1.0。

实测（`164651`）：bootstrap 期峰值 4.998（触发过一次
`headroom limited 4.98→4.38`），带载八字段稳定在 **2.85 ~ 3.10**，
drop 后连续回落到 **1.00**。

> [!caution] 🔥 这一段仍有饱和
> `[flight-diag] t=15.6s`（LIFT/交接期）实测 `u_sat=[T0 r0 p15.0 y0]%`、
> `mot_peak=1.000 mot_sat=9.0%`。跟踪本身没问题（末段 `pos_err` 0.03~0.08 m），
> 但 pitch 力矩通道在这一段确实短暂打满——与既有认知
> （4 m/s 工作点真瓶颈是力矩不是推力）一致。

---

### 3.4 阶段四：DROP

> [!abstract] drop 不再是"一帧的事"
> 释放指令改变的是**被控对象**，不是控制器模型。所以 NMPC 发完指令进入
> **pending**，继续消费正在衰减的估计；只有 MHE 的空载置信度**持续**达标，
> 才算 complete 并复位自适应状态。

#### NMPC（`_grip_drop_phase`，continuous 分支）

```python
self.enable_pub.publish(Bool(data=False))   # → proximity 释放 box
self.grip_drop_pending = True               # 不是 grip_dropped
self._no_payload_latched = False            # 无条件重新武装空载检测
self._no_payload_conf_frames = 0
# 不发 mass_event、不清 dJ/c、不碰 PX4 参数
```

完成判据在 `_confirm_no_payload_if_persistent`（每次 solve 前）：
$conf \ge 0.90$ **连续** 1.0 s（且期间每帧估计都新鲜），才置
`grip_drop_done/grip_dropped`、复位 L1 与 $\xi$。

实测：DROP 命令 → complete **+2.34 s / +2.40 s**（两轮），$conf$ 到 0.998 / 0.999。
⚠️ 这两个数字来自 `ALPHA=-1` 覆盖轮，释放由**残差路**触发。纯默认档下的结果不是
"延迟更大"，而是**根本完不成**——见下。

> [!bug] ★ 默认档下 DROP 确认死锁（`232444` 实测，机制已坐实）
> 纯默认档跑一轮（`thresh=4.4N`，其余全默认）：飞行正常、载荷物理上确实卸掉了
> （$\hat m$ 回到 2.0515~2.0551 kg，即 $m_p \approx 0$；$\Delta J = 0$；
> `om_scale` 回 1.000），但 **`DROP complete` 在命令后 190 s 都没出现**。
> 实测发布帧：$conf = 0.694$、`healthy=1`，**卡在 0.90 阈值下方不动**。
>
> **为什么 $conf$ 上不去**：三通道乘积里质量项与惯量项都已满分，唯一压着它的是
> 一阶矩通道——发布的 $\lVert s\rVert = 0.0032$ kg·m 落在
> $[\text{moment\_full } 0.0015,\ \text{moment\_zero } 0.0060]$ 的过渡区中部，
> 单通道得分 ≈ 0.69。**这就是 $s$ 估计的残噪本底**，它天然高于 0.0015 的判定水位。
>
> 也就是说：$conf$ 达标**依赖** `_release_payload` 把发布目标切零
> （`_s_release_latched` ⇒ `_s_out` 按 LPF 衰减到 ~0），**不能**指望 $s_{est}$
> 自己收敛到判定水位以下。前两轮之所以能 complete，正是因为残差路调用了它。
>
> **而默认档下三条触发路径全断**：
>
> | 路径                    | 为什么不通                                                                                                                                     | 实测                                                              |
> | ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
> | 质量域判据              | `_mass_observable()` 要求 $\lvert\omega\rvert\le0.15$、$\lvert v_{xy}\rvert\le0.20$；drop 后飞机**继续飞八字**，释放计数被每帧清零 | yaw rate 1.03~1.05 rad/s，是阈值的 7 倍；判据已武装但一次都没累计 |
> | $\lVert s\rVert$ 塌陷 | 需跌破自身峰值的 10 %                                                                                                                          | 峰值 0.0229 → drop 后 0.0032，比值**0.14 > 0.10**，差一点  |
> | 残差自检测              | 阈值 4.4 N ≫ 台阶 1.47 N                                                                                                                      | 默认档**故意**堵死的那条                                    |
>
> 后果不止是日志里少一行：`grip_dropped` 不置位 ⇒ L1 / $\xi$ 状态不复位、
> 任务状态机停在 pending。**这是架构级死锁，不是调参问题**：NMPC 的完成判据
> 依赖 MHE 的释放动作，而默认档下没有任何一条路能触发那个动作。
> 与 `continuous-mainline-attach-gate` 记的那次（三条释放路径全被
> `_payload_attached` 挡死）是同构缺陷，只是这次挡在机动门控 + 阈值 + 比例门槛上。
>
> **修法（2026-09-04 晚已落地，用户拍板）**：走"drop 专用的相对塌陷确认"这条，
> 另外两条都不动——
>
> | 方案                                                           | 取舍                                                                                                                                     |
> | -------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
> | ★ 相对塌陷 0.10 →**0.20** + 质量/惯量佐证 + 0.5 s 持续 | 采纳。0.14 的实测比值说明**物理卸载后一阶矩已塌 86 %**，而 0.10 要求塌 >90 %，是在要求残噪低于一个没有物理必要性的水平             |
> | 抬`moment_full` 0.0015 → 0.0035                             | **不做**。那是 confidence 标定，抬它等于"噪声下不去就把满分线抬上来"，会把检测逻辑问题伪装成评分标定问题，且波及 attach 与其他载荷 |
> | drop 后飞回低机动再判                                          | **不做**。detector 本身没解决，只是改实验让旧 detector 恰好能工作；对"持续机动 + 无事件信号"这条论证是自我削弱                     |
>
> 更关键的是**因果方向**要摆正。原来是：
>
> ```text
> _release_payload() → 发布目标切零 → moment 通道涨 → conf>0.9 → 才能 _release_payload()
> ```
>
> 这就是死锁本体。正确关系是 $conf$ **只做 release 之后的状态质量指标**，
> 不作为触发 release 的前置条件：
>
> ```text
> 原始估计量的 release evidence → drop detector → _release_payload()
>     → 状态/发布目标清零 → conf 最终收敛
> ```
>
> 判据仍是**纯估计器内部信号**（$s$ 的相对塌陷 + 质量/惯量佐证），
> 不需要 NMPC 的 release command，无信号主线不受破坏。

##### 改造后的实测分界（5 轮，`[s-collapse]` 日志每 1 s 一行）

| stamp           | peak   | 释放时$\lVert s\rVert$ | **释放 ratio** | **带载 $ratio_{min}$** | 带载中位 | pre-drop 误触发 |
| --------------- | ------ | ------------------------ | -------------------- | ------------------------------ | -------- | --------------- |
| `234516`(viz) | 0.0243 | 0.0036                   | 0.148                | 0.810                          | 0.919    | 0 / 65          |
| `234903`      | 0.0275 | 0.0012                   | **0.045**      | 0.634                          | 0.786    | 0 / 52          |
| `235647`      | 0.0184 | 0.0015                   | 0.079                | **0.234**                | 0.718    | 0 / 58          |
| `000017`      | 0.0140 | 0.0016                   | 0.118                | 0.277                          | 0.636    | 0 / 53          |
| `000352`      | 0.0267 | 0.0049                   | **0.182**      | 0.399                          | 0.713    | 0 / 65          |

**5/5 释放成功、5/5 零误触发**，DROP 命令 → complete 延迟 3.55 / 3.40 / 3.90 / 9.30 / 4.10 s。

> [!warning] ⚠️ ★ 但两个分布几乎贴在一起，别被单轮的"5.5× 分离"骗了
> ```text
> 卸载后 ratio :  0.045  0.079  0.118  0.148  0.182          ← max 0.182
>                                              阈值 0.20 ↕
> 带载 ratio_min:              0.234  0.277  0.399  0.634  0.810   ← min 0.234
> ```
>
> **漏检裕度 1.10×、误触发裕度 1.17×** —— 都只有一成多。首轮那个
> "0.810 vs 0.148 = 5.5×" 是**运气好的一轮**，n=1 严重高估了分离度。
>
> 更要紧的是 ratio 的分子分母**各自都在动**：peak 0.0140~0.0275（2×，随 attach
> 实际偏心变），残噪 0.0012~0.0049（4×）。两者的**最坏组合**（残噪 0.0049 配
> peak 0.0140）给出 ratio ≈ 0.35，会直接漏检、退回死锁。这个组合本批没撞上，
> 但两个边界值都已经各自出现过。

> [!important] ❗ 判别力其实来自**合取**，不是 ratio
> 带载 $ratio_{min}$ 有两轮掉到 0.234 / 0.277 —— 已经在阈值门口，
> 但**误触发仍是 0/283 帧**。挡住它的不是 ratio，是质量通道。
> 也就是说：单看 ratio，两个分布重叠得几乎没有判别力。

##### 释放判据的最终形态（2026-09-05）

先按"三个真正独立的信息源"重构（上一版把 mass 与 inertia 当两票用是**虚假鲁棒性**
—— 连续接口里 $\Delta J = \mu(m_p) r_z^2$ 完全由 $m_{est}$ 与固定 $r_z$ 派生），
随后又**删掉了其中一条路径**，因为它在 SITL 里造成了真实的提前释放。

| 信息源 | 内容 | 可靠性（实测） |
|---|---|---|
| ① 一阶矩塌陷 | 相对 $ratio$；绝对 $\lVert s\rVert$ 只作 sanity 上限 | **唯一可靠**：带载 $ratio$ 最低 0.789 对阈值 0.30 有 2.6× 裕度 |
| ② 载荷量 | mass 与 inertia **合并成一条**通道（它们不独立） | 机动中假阳性：$m_{est}$ 长时间贴下界 |
| ③ 残差方向 | 独立的 release-residual 票，**不复用** `confirm_thresh` | 机动中假阳性：幅度不可分 |

```text
前提：必须可靠进入过 LOADED，且当前估计帧 health/fresh
slow  ：quantity_empty  AND moment_collapsed，持续 5 帧
fastB ：strong_residual AND moment_collapsed，持续 3 帧
★ 任何成功释放都必须带 moment 证据
禁止：仅 ratio；仅 mass/inertia；普通残差票配 moment；quantity 配普通票
```

> [!bug] 被删掉的 fastA：一次真实的安全失败
> 原第三条路径 `普通残差票 AND quantity_empty`（不含 moment）在 12 轮 smoke 里
> **9 轮跑出 3 轮提前释放，最早提前 51.1 s**。误释放现场那一帧：
>
> ```text
> ratio=0.694 (远高于阈值 0.30)  moment=0  quantity=1  resid=1  m_p=-0.0624
> ```
>
> 设计它时的假设——"载荷还在时 `quantity_empty` 恒假"——**是错的**：
> 4 m/s figure-8 中 $m_{est}$ 长时间贴在下界 $m_{min}$，$m_p$ **持续**为负。
> 带载段 $m_p < 0.03$ 的连续时长中位 **14 s**、最坏 48 s（drop 后是 48~52 s），
> 幅度与持续性**都分不开**。于是那条路径是两个都不可靠的通道相与，
> 没有任何东西能否决它。这正是 `_mass_observable` 机动门控存在的理由。
>
> 代价（有意接受）：$ratio$ 不塌的 drop 检不出 ⇒ **漏检**，交给 unresolved 路径，
> 绝不靠更弱的证据去释放。

##### 独立的 release-residual 票

**不降** `confirm_thresh = 4.4 N`（它还管事件权重调度与质量突变检测，降它会改变
MHE 收敛行为、混淆实验结论）。新判据只投"释放票"，形式是短窗方向阶跃
$\Delta T = \text{mean}(\text{post }0.3\,\text{s}) - \text{mean}(\text{pre }0.3\,\text{s})$，
阈值按**系统最低支持载荷 0.15 kg** 的检测规格定（$m_P g = 1.47$ N）。

> [!warning] 两个和直觉相反的标定结论（0.2 kg 高频回放）
> 1. **z 归一化没有区分力**：$\sigma(\text{MAD}) \approx 0.13$ N 太小，越过
>    $T_{floor}$ 的样本自动 $z > 6$ —— $z = 3/4/6$ 逐帧同结果。默认 0（关）。
> 2. **幅度上不可分**：figure-8 稳定段 $\Delta T$ 最负 −1.69 N，比 0.15 kg 卸载的
>    真信号 −1.47 N 还大。任何能检出 0.15 kg 的绝对阈值都会被机动瞬态碰到。
>
> 唯一可用的维度是**持续性**：$T_{floor}=0.9$ N 下 figure-8 最长连续 3 帧、
> drop 4 帧。故普通票 2 帧（只配质量通道用）、强票 **4 帧**（才允许配 moment）。

##### health/freshness 门控：re-anchor 的 `s_est=0` 不是证据

一次 slow 误释放的根因链（`152924`）：

```text
连续 5 次 solve failure → re-anchor 把 s_est 清零 → 释放判据(跑在 _solve_window
之前)在下一次成功解产生前照常运行 → ratio=0.000 被读成 moment_collapsed
→ 恰好 m_p 也探负 → slow 成立 ⇒ 误释放
```

判据完全没用上节点**已有的** health/freshness 状态，这才是缺陷本身。修法：

```python
frame_ok = _payload_frame_health() and not _s_reanchored
if not frame_ok:
    slow_frames = fast_frames = strong_frames = 0   # 清零而非暂停
    return                                          # 不允许释放
```

`_s_peak` **不清** —— 短暂求解失败不该丢掉本轮带载基准。没有加 $\varepsilon$
下界、没有加 cooldown：先验证严格门控本身是否已经足够。

##### DROP 超时 ⇒ unresolved，**不等于**"已卸载"

```text
moment 未确认 → payload state = unresolved
  → 不切 s_target、不清模型、不复位 L1/ξ、不置 grip_dropped
  → 同周期切换 ref_fn + ref_window_fn
  → 从当前 p/v/q/ω 连续进入 3 s 制动参考，之后在停止点悬停
  → 交上层介入 / 降落中止
```

两边都不猜：猜"已卸载"会在载荷还挂着时按空机构型飞；继续按带载飞则可能带着
幽灵偏心（07-02 实测 2/2 坠机）。迟到的 moment 证据到达可解除 unresolved。

> [!warning] 历史配置与当前实现不同
> 09-05 的 12 轮 smoke 代码只清了 `grip_dynamic_active`，没有切换 `ref_fn` /
> `ref_window_fn`；因此飞机仍按动态参考飞，不能把这些日志当作“保守悬停”的实测
> 验证。独立 `_grip_dyn_latched` 后只消除了 figure-8 重入，仍未兑现退出机动。
> 当前实现才新增连续 brake-to-hover 参考；必须另跑构造 SITL 回归后才能宣称飞行
> 结局得到验证。

##### 验收：12 轮 fastA-free smoke（`160013` ~ `164135`）

| stamp | 释放 | 相对 drop | 路径 | unresolved | solve failed |
|---|---|---|---|---|---|
| 160013 / 160343 / 160713 | 1 | +1.2 / +1.0 / +1.8 s | fastB | – | 0 |
| **161043** | 1 | +2.7 s | **slow** | – | 0 |
| 161413 | 1 | +1.1 s | fastB | – | 0 |
| **161743** | 1 | **+14.8 s** | fastB | **是** | 0 |
| 162128 | 1 | +2.1 s | fastB | – | 1 |
| **162459** | **0** | — | — | **是** | 4 |
| **163049** | 1 | **+14.2 s** | **slow** | **是** | 0 |
| 163434 / 163805 / 164135 | 1 | +1.7 / +2.4 / +1.9 s | fastB | – | 0 |

**带载段误释放 0**（删 fastA 前是 3/9 轮）。11 轮正常释放（延迟中位 1.9 s），
1 轮进 unresolved 终态且不释放、不清模型。路径分布 fastB 9 / slow 2 —— 两条都
在实际工作，互为备份；3 轮进过 unresolved，其中 2 轮拿到迟到证据后 RESOLVED。

`161743` 是 unresolved 状态判决的完整案例：
`+12 s UNRESOLVED → +16 s RESOLVED → DROP complete`。它验证迟到证据恢复，
**不验证**转悬停；该版本的 `ref_fn` 从未切离动态参考。

> [!warning] 旧日志不能用于 FPR
> `152924` **不能**作为删除 fastA 后的干净反事实 —— 那份日志产生于旧版 fastA
> 已提前清状态、presence 又重武装的闭环。而且离线回放把 1 Hz 日志的一帧折算成
> 在线 10 Hz 的五帧持续条件，所以回放给出的"1/548"不是新版 slow 的在线误释放率。
> **旧日志只用于定位根因，最终 FPR 必须来自全新的 fastA-free SITL。**

##### moment reference：ratio 的分母不能是 running max

`ratio = |s|/max_history(|s|)` 有一个致命前提：历史最大值是**真实的带载几何**。
实测否掉了它 —— figure-8 段的 $s$ 估计会系统性越过物理上限：

| 批次 | 越界帧/总帧 | 越界率 | 越界起点 | $\lVert s\rVert$ max |
|---|---|---|---|---|
| 早批（11 轮） | 49/644 | 7.6 % | attach+26~28 s | 0.0492 |
| 后批（4 轮） | 49/268 | 18.3 % | attach+27 s | 0.0520 |

物理上限 = 载荷包线 × attach 偏心上限 = $0.30 \times 0.13 = 0.039$ kg·m。
越界值反解 $r_{xy}$ 达 0.35 m，远超 attach 上限 —— **那不是几何，是动态失真**。
一旦进了分母，带载稳态的 ratio 就被压到阈值以下，`moment_collapsed` 会在载荷
还挂着时成立，与 fastA 栽掉的结构同型。

> [!warning] 一处因果误判（已更正）
> 我最初把"detector 提早 10 s 开始跑"当成了"污染发生在 attach 早期"，
> 并据此报告是 A 修复（`ddfe9d2`）引入的新缺陷。按时间分布重新核对后：
> **两批的越界起点都在 attach+26~28 s（figure-8 段）**，早批 11 轮里就有 4 轮越界。
> A 修复只是扩大了 detector 的观测窗口，把越界率从 7.6 % 抬到 18.3 %，**不是根因**。

**状态机**（`_update_moment_reference`）：

```text
LOADED(自主) → healthy/fresh → 自主识别的低动态窗口(|ω|≤0.15, |v_xy|≤0.20)
  → 剔除 |s|>0.039 的物理越界样本(剔除,不裁剪)
  → 稳健统计(截尾均值 + 离散度≤0.25 + 方向一致度≥0.90)
  → 冻结 s_ref_loaded → 进入机动后不再更新
ratio = |s_current| / |s_ref_loaded|
```

低动态窗口是**在线判据**，不是"attach 后等 N 秒"——后者只是当前脚本时序下的
巧合区间。越界帧的处置：不进 reference、不参与 moment 投票、清持续计数、
累计为 estimator-quality 指标。基准建不起来就不判决 ⇒ 交 unresolved，
**不降低释放门槛**。

`moment_abs_max` 由**共享的平台包线**派生而非写死：两个节点读同一份
`config/gripper/gripper_params.yaml` 的 `/**` 段（`payload_mass_envelope 0.30`
× `payload_rxy_envelope 0.13`），避免 0.039 / 0.30 / 0.13 三处漂移。
包线是机架规格，不进估计器观测通路。

##### 物理真值日志：`attach 请求` ≠ `关节已建立`

`20260905_182456` 暴露的判定盲区：proximity 条件满足、ATTACH 请求已发出，但
MHE 全程没看到载荷（$m_{est}$ 恒 1.9611 = 下界），而当时**没有保存
`/gripper/state` 回执**，无法判定 `DetachableJoint` 到底有没有建立。
"用 $m_p$ 反推 attach 失败率"是不成立的 —— 那把机构失败与估计器失败混成一个数。

现在每轮独立录制 `gz topic -e -t /gripper/state`（插件只在真正建立/移除关节时
才发 `ATTACHED/DETACHED`），manifest 三列分开记：

| 列 | 含义 | 用途 |
|---|---|---|
| `attach_requested` | proximity 发出过 ATTACH 请求 | 机构/接近失败的分类 |
| `physically_attached` | 插件回执 ATTACHED 且其后无 DETACHED | **有效性前置检查用它** |
| `mhe_load_observed` | 估计器看到载荷 | 只做统计：物理已 attach 但 MHE 未识别 = estimator observability failure |

⚠️ 该日志只进实验记录与离线判定，NMPC/MHE 都不消费，不构成外部事件信号。

##### 正式安全 smoke：12 轮（`200316` ~ `204345`）

冻结记录 `freeze_release_detector_20260905_200257.md`（代码 `9fbcd9b`+`7f0cedc`、
六文件 sha256、install↔src 一致性、共享包线、17 个判据参数、manifest 格式）。

| 指标 | 结果 |
|---|---|
| `physically_attached` | **12/12** |
| **带载段误释放** | **0/12** |
| NMPC `DROP complete` | **12/12** |
| `moment_ref_ready` | 11/12 |
| 进过 unresolved | 6/12，**全部恢复（RESOLVED 6/6）** |
| 释放路径 | fastB 9 / slow 2 |
| 五条停批条件 | **逐轮核查全部未命中** |

延迟是**二值**分布，与越界帧数严格对应：

| `ovr` | 轮数 | 延迟 |
|---|---|---|
| 1 帧 | 5 | +1.3 ~ +1.5 s |
| 10~25 帧 | 6 | +13.3 ~ +24.4 s（全部经 unresolved → RESOLVED） |

中间没有过渡值 —— 越界帧的多少不直接决定延迟长短，它只决定**会不会跨过 12 s
那道 unresolved 门**。

> [!note] 目标覆盖 0/3，**未获得自然样本**
> 目标组合（`physically_attached ∧ residual ATTACH ∧ _load_armed ∧ ¬_c_xy_mass_armed`）
> 12 轮一次都没出现：全部经由 `sustained m_p evidence` 进 LOADED，而那条路径的
> 水位 0.09 kg 同时把旧质量域分支也武装了。该组合的出现依赖 $m_{est}$ 收敛得
> 足够慢（慢到残差先检出 attach），不受实验控制。
>
> **决定（2026-09-05）**：停止自然补跑。目标组合的逻辑已由单元测试覆盖
> （`test_detector_runs_without_mass_domain_arming`）；要验证在线执行，
> 另跑 3 个**明确标注的 fault-injection 构造轮**即可，不必消耗随机样本。
> 本 12 轮作为**正式安全 smoke** 保留。

##### 唯一的 `mref=0`：LOADED 与低动态窗口的时序竞争

`201412`：`ATTACH t=5.9s → DYNAMIC t=20.4s → LOADED t=48.5s`（晚于 DYNAMIC **28 s**）。
那时飞机已在 4 m/s figure-8 里，低动态窗口不再出现，窗口计数**恒为 `0/20`**。
后续按设计走完：基准未就绪 → 不判决 → drop 后 `DROP complete (conf=0.993)`，
零误释放。安全性无问题，但那一轮 detector 全程没有可用的 moment 通道。

这是结构性的：LOADED 的进入依赖质量估计收敛，而低动态窗口只存在于 attach 后
约 14 s 内（LIFT+悬停），两者是竞争关系，当前没有机制保证前者赢。

##### 三个遗留问题的处置（2026-09-05 定案）

| # | 问题 | 处置 |
|---|---|---|
| **A** | **武装门控串联**：统一 detector 被旧的 `_c_xy_mass_armed` 挡住。`162459` 那轮自主状态机已 `EMPTY→LOADED`（`_load_armed=True`，靠残差 attach 事件），但 $m_{est}$ 全程偏低（进 LOADED 时 $m_p=-0.103$），旧的"质量持续过 0.09 kg"路径从未武装 ⇒ `[s-collapse]` 一行都没有，detector 整段没运行 | **已修**（`ddfe9d2`）：外层门控只保留 `_load_armed` + 内层 health/freshness；`_c_xy_mass_armed` 只留给旧质量域释放路径。**不是**新增 attach 阈值 —— 问题是两个武装状态被串回了一起。释放证据未放宽：仍必须满足 moment 且估计健康 |
| **B** | **drop 后 $s$ 不收敛回零**：$m_{est}$ 正确回到 2.049，$\hat s$ 仍在 −0.018 kg·m ⇒ moment 证据永不成立 | **按设计保留为安全漏检**。这是真实的估计覆盖问题，但**不能**依据 NMPC 的 drop 命令、超时或质量结果去强制清零 —— 那等于把事件捷径重新引回估计器，破坏无事件主线。当前的正确结果就是：moment 不成立 → 不释放估计器状态 → unresolved → 保留模型 → 安全悬停/降落。MHE 可观测性单独研究 |
| **C** | **保守悬停对 $s$ 可观测性的影响尚未测量**：旧代码没有切换 `ref_fn`，所以 `161743` / `162459` 都不能作为 hover 样本 | **当前代码实现 brake-to-hover，但结局仍待专门 A/B**。悬停的职责是降低飞行风险，不是保证辨识恢复；不能为了等 moment 继续高速 figure-8 |

> [!note] C 的后续实验设计（待做，不阻塞主线）
> 小型**配对**实验，第一阶段 6~8 对即可判方向，再决定是否扩大：
>
> | 臂 | 处置 |
> |---|---|
> | A | unresolved 后**悬停**（现状） |
> | B | unresolved 后施加**安全包线内的小幅辨识激励** |
>
> 比较：$ratio$ 恢复时间、resolve 比例、MHE failure 次数、位置误差、姿态饱和。

### 3.5 阶段五：DROP 后

| 量               | NMPC                     | MHE                                      |
| ---------------- | ------------------------ | ---------------------------------------- |
| $m$            | 跟原子帧连续回落         | 连续收敛回空机                           |
| $s$            | 跟`_s_out` 衰减到 0    | 发布目标闩零，窗口内$s_{est}$ 仍有残噪 |
| $\Delta J,\ c$ | 随$(m,s)$ 自然回零     | 同左                                     |
| `omega_scale`  | 按 1.5 s⁻¹ 恢复到 1.00 | —                                       |
| PX4 参数         | 全程没动过，无需复位     | —                                       |

> [!success] `self` 档"drop 后 $m_{est}$ 偏低"的老坑，在 moment 档下没有复现
> 旧记录：`geom_release_mode='self'` 下 drop 后 $m_{est}$ 系统性偏低 **−2.81 %**、
> 55 个采样里 14 个贴在 $m_{min}$ 上。机理是"载荷在不在"这个信息被硬编码进了质量——
> 杆臂还在模型里，优化器只能把 $m_P^+$ 压到 0 才能消掉幽灵偏心。
>
> $s$ 增广正是对症的修法（$s$ 有独立观测，会自己趋零，$m$ 不再被逼向下界），
> 而它**现在默认开着**。09-04 实测 drop 后 $\hat m$ = 2.0507 ~ 2.0587
> （相对 $m_B$ **−0.27 % ~ −0.66 %**），**一次都没碰下界 1.961**。
> 上一版那句"用了 `self`，却没开配套的 2b"已经不成立。

> [!danger] 仍然成立：控制器绝不能按幽灵偏心飞
> 旧实测（0.3 kg 档）：drop 后控制器继续按幽灵偏心配平，指令 0.424 N·m
> （$\tau_{max}$ 的 85 %）作用在空机 $J_{xx}$ 上 = 29.8 rad/s²，2/2 坠机。
> 新架构里这条防线换了位置：不再靠"NMPC 自己清几何"，而是靠
> **$s$ 的连续衰减 + $conf$ 确认**。所以 `[s-decay]` 那串日志是安全相关的，
> 不是诊断噪声——它若不下降，就是这条防线没生效。

---

## 4. 总表：4 量 × 5 阶段

### 4.1 NMPC 侧

|                 | ATTACH                     | 巡航                  | 八字                        | DROP 命令                  | DROP 后                                                           |
| --------------- | -------------------------- | --------------------- | --------------------------- | -------------------------- | ----------------------------------------------------------------- |
| $m$           | 不动                       | 跟原子帧              | 跟原子帧                    | **继续跟**           | 跟到空机                                                          |
| $s$           | 不动                       | 跟原子帧              | 跟原子帧                    | **继续跟**（衰减中） | → 0                                                              |
| $\Delta J$    | **bootstrap 0.0579** | 交接 → MHE 值 0.0267 | 跟随                        | 继续跟                     | → 0                                                              |
| $c_{xy}$      | $s/m$（≈0）             | $s/m$               | $s/m$，**每帧更新** | $s/m$                    | → 0                                                              |
| `omega_scale` | → cap 5.0                 | 交接后 ~2.9           | 2.85 ~ 3.10                 | 开始回落                   | 1.00                                                              |
| 状态机          | `grip_mass_stepped`      | —                    | —                          | `grip_drop_pending`      | `grip_dropped`（覆盖档 +2.4 s；**默认档死锁，见 §3.4**） |

### 4.2 MHE 侧

|                                            | ATTACH                     | 巡航               | 八字                | DROP                     | DROP 后                    |
| ------------------------------------------ | -------------------------- | ------------------ | ------------------- | ------------------------ | -------------------------- |
| $m$                                      | 不动，靠窗口爬             | 收敛（低估 ~14 %） | 继续估              | 不动                     | 连续回空机 −0.3 ~ −0.7 % |
| $s$ | 靠窗口爬 | 准（$s_y$ 误差 <2 %） | 继续估                     | 释放闩 → 目标零   | `_s_out` LPF → 0 |                          |                            |
| 几何槽                                     | $[0,0,-0.47]$            | 同左               | 同左                | 同左                     | 同左（**永不变**）   |
| $conf$                                   | 1.0 → ~0                  | ~0                 | 0.000               | 上升                     | ≥0.99                     |
| 降权                                       | 阈值 4.4 N ⇒ 本工况不触发 | —                 | —                  | 释放走质量域 /$s$ 塌陷 | —                         |

### 4.3 一句话

> **载荷认知只有一条路**：MHE 用 motor speed + odometry 估 $(m, s)$，
> 派生 $(c, \Delta J)$，打成一帧发出去；NMPC 平滑限速后喂模型和增益调度。
>
> **attach 和 drop 都不是"通知"，是"命令 + 确认"**：
> NMPC 发命令后只给 $J$ 一个临时安全地板（attach）或进入等待（drop），
> 真正的状态转移由估计器的持续证据决定。
> 全链路唯一保留的先验是机架的 $r_z \approx -0.47$ m。

---

## 5. 已知的坑与不一致

| #   | 问题                                                                                                                                                                     | 现状                                                                                                                                        |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | `GRIP_LIFT_AFTER` = 1.5 s **会发散**（09-04 `162428`：822 solve failed、tilt 141°、pos_err 70 m）；bootstrap 已按包线把内环推到 5×，box 却没吸稳             | **已修**：脚本默认 09-04 改为 3.0                                                                                                     |
| 2   | 交接开始后估计失效 ⇒`omega_scale` **冻在高位**（`162428` 钉在 5.0）；这是"不放掉地板"的有意代价                                                               | 无兜底；靠 1 避免进入                                                                                                                       |
| 3   | 带载$\hat m_p$ **低估 ~14 %**，$\Delta J$ 跟着低估同样比例                                                                                                     | 与$s$ 的 2 % 精度对比鲜明；根因见 `mest-maneuver-underestimate`                                                                         |
| 4   | $\hat s_x$ 系统性偏负 ~0.01 kg·m（$c_x \approx -4.6$ mm）                                                                                                           | pitch 通道污染，既有认知                                                                                                                    |
| 5   | ~~纯默认档 DROP 确认死锁~~                                                                                                                                              | **已修**（09-04 晚，相对塌陷判据）：`232444` 复现 → `234516` 走通，见 §3.4                                                      |
| 6   | `dj_track_mest` / `dj_ratchet_enable` / `grip_dj_floor_mp` / `grip_mp_cap` / NMPC `geom_release_mode` / `drop_publish_mass_event` 在主线**全是死参数** | 脚本仍在传，行为不受影响                                                                                                                    |
| 7   | MHE 启动日志仍打印`geom_release_mode='self'` 的坠机警告                                                                                                                | 描述的是主线走不到的老路径                                                                                                                  |
| 8   | $r_z$ 先验 −0.47 vs 实测 −0.579（低估 19 %） | 有意：$\Delta J$ 只需量级对                                                                                         |                                                                                                                                             |
| 9   | LIFT/交接期 pitch 力矩`u_sat` 达 15 %、`mot_sat` 9 %                                                                                                                 | 与"4 m/s 真瓶颈是力矩"一致                                                                                                                  |
| 10  | `event_confirm_timeout_sec = 3.0` > MHE 窗口 2.0 s ⇒ `external` 档超时兜底永远零降权                                                                                | 未修（主线不走 external）                                                                                                                   |
| 12 | 释放判据的三个遗留问题 | A 已修（`ddfe9d2`）；B/C **按设计保留**（安全漏检交 unresolved；悬停是安全动作不是辨识动作），见 §3.4 末表 |
| 10b | 四个批次脚本的 drop 判据只 grep legacy 的`"DROP: released"`，连续档打的是 `DROP complete` ⇒ **成功轮被静默记成 `no-drop`** 并空等 5 分钟                    | **已修**（09-04）：`run_cxy_mass_repeat` / `run_dj_ratchet_ab` / `run_geom_coupled_ab` / `run_moment_ab_4ms` 四处改为两种都认 |
| 11  | 这批默认值的证据基础：**离线回放 + 09-04 四轮 viz**（三轮带 `ALPHA=-1` 覆盖：2 成 1 败；一轮纯默认档：飞行成功但 drop 确认死锁），尚无成规模 SITL 批次           | 引用任何数字前先确认样本量与覆盖项                                                                                                          |

---

## 6. 日志速查

```bash
cd $WS/nmpc_test_results

# 阶段时刻(ATTACH 命令 / LIFT / DYNAMIC / DROP 命令 / DROP complete)
grep -E 'ATTACH command|LIFT:|DYNAMIC:|DROP command|DROP complete' gviz_nmpc_<stamp>.log

# J bootstrap 三步:武装 -> 交接 -> 释放
grep 'ATTACH J bootstrap' gviz_nmpc_<stamp>.log

# 估计帧健康 / 增益调度 / 饱和
grep -E '\[payload-health\]|\[omega_scale\]|\[flight-diag\]' gviz_nmpc_<stamp>.log
grep -c 'solve failed' gviz_nmpc_<stamp>.log          # >100 = 闭环失稳

# MHE:载荷存在状态机、释放来源、s 衰减、精度对表
grep -E '\[payload-state\]|载荷释放|\[s-decay\]|\[truth\]' gviz_mhe_<stamp>.log

# 这一轮到底用的哪个确认阈值
grep 'signal mode' gviz_mhe_<stamp>.log

# attach 几何真值(评估对表用,不进任何模型)
grep 'attach offset' gviz_proximity_<stamp>.log
```

判读要点：

- `DROP 命令` − `ATTACH 命令` 应 ≈ **`lift_after` + 74.46 s**（当前默认 3.0 ⇒ **77.5**；
  09-04 前的 1.5 ⇒ 76.0）。差得多说明 $w$ / `drop_after` / `dyn_settle` 与默认不同。
- `grep 'signal mode'` 那行的 `thresh=` 应为 **4.4**。若是 1.5，说明这轮带了已废弃的
  `MHE_CONFIRM_ALPHA=-1`，drop 会由残差路检出——该轮的卸载时序不能当默认档的数据用。
- 没有 `DROP complete` = $conf$ 没持续达标。**纯默认档下这是当前的已知常态**（§3.4）：
  先看 MHE 日志里有没有 `载荷释放` / `[s-decay]` —— 一行都没有，就是三条释放路径
  都没触发，$s$ 的发布目标从未切零，$conf$ 会停在 0.7 上下。
  其余可能：估计帧不健康（看 `[payload-health]`），或这轮压根没抓上载荷。
- `[truth]` 行里的 `m_true` 恒等于**空机** 2.0643（`grip_true_payload_mass` 不能开，
  会把真值灌进模型），所以带载段的 `err=+6.2%` 读作 $\hat m_p \approx 0.128$ kg，
  **不是**"估计误差 6.2 %"。真实误差要拿它跟 0.15 比。
- `s_hat` 对表用 proximity 的 attach offset 现算：$s_{true} = 0.15 \times r_{xy}$。
- `om_scale` 在 drop 后应回到 1.00；停在 2.9 以上 = 载荷认知没退，查 `[s-decay]`。

---

## 7. 源码索引

| 内容                              | 位置                                                                                                                                                |
| --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| 接口 schema / 载荷代数            | `payload_estimate.py :15 FIELDS`、`:35 no_payload_confidence`、`:54 inertia_from_mass_moment`、`:117 slew`、`:126 headroom_limited_scale` |
| 接口说明文档                      | `docs/continuous_payload_interface.md`                                                                                                            |
| NMPC 收帧 / 新鲜度                | `acados_nmpc_node.py :1245 payload_estimate_cb`、`:1266 _payload_estimate_is_fresh`                                                             |
| attach J bootstrap                | `acados_nmpc_node.py :1299 _start_attach_j_bootstrap`                                                                                             |
| 每帧模型更新(LPF+slew+交接)       | `acados_nmpc_node.py :1319 _update_continuous_payload_model`                                                                                      |
| drop 完成确认                     | `acados_nmpc_node.py :1396 _confirm_no_payload_if_persistent`                                                                                     |
| ω 缩放调度                       | `acados_nmpc_node.py :1870 _update_omega_scale`（应用在 `publish_attitude`）                                                                    |
| attach 阶段                       | `acados_nmpc_node.py :2148 _grip_mass_step`（continuous 分支在开头）                                                                              |
| 几何槽装配                        | `acados_nmpc_node.py :2304 _geom_slot`                                                                                                            |
| drop 全流程                       | `acados_nmpc_node.py :2512 _grip_drop_phase`                                                                                                      |
| PX4 增益改写(主线**不走**)  | `acados_nmpc_node.py :2380 _scale_px4_rate_gains`                                                                                                 |
| MHE 载荷存在状态机                | `mhe_node.py :1019 _update_payload_presence`                                                                                                      |
| MHE 释放(s 闩 / c_xy 归零)        | `mhe_node.py :1067 _release_payload`                                                                                                              |
| MHE 帧健康自检                    | `mhe_node.py :1391 _payload_frame_health`                                                                                                         |
| MHE 发布原子帧                    | `mhe_node.py :1401 _publish_payload_estimate`                                                                                                     |
| MHE 几何槽(主线提前 return)       | `mhe_node.py :1476 _model_r_p`                                                                                                                    |
| MHE c_xy(一阶矩路径) / 质量域判据 | `mhe_node.py :1850 _update_c_xy_est`                                                                                                              |
| frozen A 的实现                   | `mhe_node.py :2025`（`_solve_window` 内借 `geom[0]` 传 $\hat m$）                                                                           |
| 一阶质量矩参数                    | `mhe_params.py :227 estimate_moment`、`:231 rz_prior`、`:253 moment_a_mode`                                                                   |
| NMPC 动力学                       | `acados_model.py`                                                                                                                                 |
| 耦合档模型代数                    | `mhe_model.py`                                                                                                                                    |
| 视频叠加渲染                      | `scripts/demo_video/make_overlay.py`                                                                                                              |
