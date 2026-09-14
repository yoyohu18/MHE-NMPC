# 实验变量总清单 与 最优实验组合规划

> 2026-09-14 整理。来源:`scripts/**/run_*.sh` 的环境变量接口、`acados_nmpc_node.py` /
> `mhe_node.py` 的 `declare_parameter`、`mhe_params.py` / `acados_params.py` 的
> `os.environ` 开关,以及各脚本头部记录的实测结论。
>
> 两句话总结:**真正开放的因子只有 6 个**(其余要么已判定、要么是覆盖维);
> 而这 6 个里最值钱的一次设计不是"继续扫 MHE_N",而是**用 (r, w) 三点破掉
> 08-19 那批里"窗口占周期比"与"窗口内弧长"的共线**(见 §4 P3)。

---

## 1. 变量总清单

标注约定:**默认值**取"不显式设任何环境变量时的实际生效值";
`env` = 环境变量,`ros` = ROS2 参数(由 headless 脚本从 env 转发),
`code` = 只在源码里改。

### 1.1 A 类:场景 / 工况(covariate,论文要覆盖,不做单因子检验)

| 变量 | 入口 | 默认 | 已用档位 | 备注 |
|---|---|---|---|---|
| 场景 | 脚本选择 | — | `masschanger`(wrench 质量突变) / `gripper`(DetachableJoint 真刚体) | 两条线不共享 world/config |
| `GRIP_PAYLOAD_KG` | env | 0.3 | 0.15 / 0.2 / 0.3 | 0.15 曾结构性失败(自举几何路径,07-19 已修) |
| `GRIP_ECC_Y` | env | 0.05 | 0.05 / 0.08 / 0.10 / 0.12 | 物理可行区间 ~0.05–0.12 m |
| `MASS_CHANGER_DELTA_KG` | env | — | −0.3 / −0.8 | 仅 masschanger |
| `GRIP_DYNAMIC` | env | false | hover / figure8 | 带载轨迹开关 |
| `GRIP_DYN_R` | env | 0.8 | 0.8 / 5.0 / 10.0 | 包络 = 2r × r |
| `GRIP_DYN_W` | env | 0.25 | 0.283 / 0.424 / 0.566 / 0.707 | v_peak = √2·r·w |
| `GRIP_DYN_RAMP` | env | 0.0(=写死 4.0s) | −1(auto) / 3.0 / 显式 | 改 ramp 会平移 drop 相位 |
| `GRIP_DYN_DZ` / `grip_dyn_settle_sec` | env / ros | 0.0 / 3.0 | — | |
| `GRIP_Z_HIGH` / `GRIP_LIFT_DUR` | env | 2.5 / 3.0 | 2.5+3.0 / 6.0+8.4 / 10+… | **强耦合约束**:r=10 在 z=2.5 必发散,必须配 z≥6 |
| `GRIP_DROP_AFTER` | env | 0.0(关) | 0 / >0 | drop 后 m_est 会撞下界,污染稳态统计 |
| `GRIP_DROP_AT_TIP` | env | false | true | 跨速度档对比时**必开**,否则 drop 相位不可比 |
| `ATTACH_WINDOW_SEC` | env | 40.0 | 140.0 | 逐帧日志是唯一分析源,3 圈需要 140 |
| 风扰 `AXES` / `MAGS_Z` / `MAGS_X` | env | — | z:0.5–3.0;x:1–3 | C.3 FPR,仅 masschanger |
| `PAYLOAD_ENABLED` | ros | true | false | 纯轨迹跟踪速度阶梯用 |
| `FIG8_R` / `FIG8_W` / `FIG8_RAMP` | env | 1.0 / 0.3 / 0.0 | — | masschanger 侧轨迹 |
| `LAPS` / `SETTLE` | env | 3 / 3.0 | — | 飞**相同圈数**而非相同时长 |

### 1.2 B 类:MHE 估计器

| 变量 | 入口 | 默认 | 档位 | 状态 |
|---|---|---|---|---|
| `MHE_N` | env | 20 | 10 / 20 / 40 | **开放**(08-19 n=2,结论未定) |
| MHE `dt` | code | 0.1 | — | 固定,10 Hz |
| `MHE_LAMBDA` | env | 1.0 | 0.7 / 0.8 / 0.9 / 0.95 | 已有 4m/s 配对 A/B |
| `MHE_M_MIN` | env | 0.95·m_B ≈ 1.961 | 1.0(高应力) | 消融专用,勿混入其它批 |
| `MHE_GEOM_COUPLED` | env | 0 | 1 | 已有 A/B;切换会重 codegen(独立缓存) |
| `MHE_ESTIMATE_MOMENT` | env | 0 | 1 | **开放**,只有单测无 SITL |
| `MHE_RZ_PRIOR` / `MHE_S_ABS_MAX` | env | −0.47 / 1.5 | — | 仅 estimate_moment 生效;rz ±33% 对质量零影响 |
| `MHE_PARAM_NOISE` | env | 0 | 1 | **已判负**(Hessian 病态,472 次 solve 失败) |
| `MHE_SIGMA_M_DOT` / `_S_DOT` | env | 0.5 / 0.05 | — | 仅 param_noise 生效 |
| `MHE_SEED_FROM_THRUST` | env | 0 | 1 | **开放**(单测 7/7,无 SITL A/B) |
| `MHE_SEED_THRUST_MIN` | env | 1.0 | — | |
| `MHE_PAYLOAD_KI` | env | 0.0 | 0.00375 | 载荷自身惯量,相对 dJ 仅 2% |
| `MHE_EVENT_TRIGGER` | env | true | false | |
| `MHE_SIGNAL_MODE` | env | external | residual | A.2/A.3 无信号消融 |
| `MHE_SCHEDULE_THETA` | env | M0 `[-4,0,0,0]` | θ*(5 维) | **已判负**(2×2 析因 + 10Hz 对照,无可测收益) |
| `MHE_CONFIRM_THRESH` | env | 1.5 N | — | 该值是给 wrench 标的,gripper 未重标 |
| `MHE_CONFIRM_ALPHA` | env | −1.0(走固定阈值) | 0.9875 | 与 θ* 同批判负 |
| `MHE_CONFIRM_PRIOR` | env | 0.3 | — | |
| `MHE_RESID_PERSIST` 等 4 个 | ros | 2 / 3.0 / 20 / 0.8 | — | 残差自触发检测器内部 |
| `GRIP_MHE_MP_PRIOR` | env | =载荷真值 | 0.3 / 0.5(包线) | **开放**;**禁设 0**(回落自举路径) |
| `GRIP_GEOM_MP_FLOOR` | env | 0.15 | 0 | prior>0 时为死代码 |
| `GRIP_TRUE_PAYLOAD_MASS` | env | 0.0 | >0 | 诊断专用,生产实验不要开 |
| `MHE_GEOM_RELEASE_MODE` | env | event | self | `self` 目前**已知不可用** |
| `mhe_tau_source` | ros | command | phys | `phys` **已判负**(attach 段发散) |
| `MHE_C_XY_EST` | env | false | true | 需要对表数据的批次必开 |
| `c_xy_est_tau_sec` / `steady_omega` / `steady_vel` | ros | 2.0 / 0.15 / 0.20 | — | 机动段本就冻结,论文表述要写准 |

### 1.3 C 类:NMPC 控制器

| 变量 | 入口 | 默认 | 档位 | 状态 |
|---|---|---|---|---|
| `NMPC_CONTROL_MODE` | env | mhe | l1 | C.1 流派 B 对照,已跑 |
| `L1_A_GAIN` / `L1_OMEGA_C` | env | 10.0 / 0.5 | ω_c 1.0/2.0 炸机 | 已扫定 |
| `NMPC_GEOM_SOURCE` | env | truth | online | B.3 已结案(online ≈ truth) |
| `GRIP_NMPC_MP_PRIOR`(旧名 `GRIP_GEOM_MP_PRIOR`) | env | =载荷真值 | 0.0 / 0.25 / 0.3 | **开放**(prior=0 有 1/7 LIFT 发散) |
| `NMPC_TAU_LUMPED` + `XI_MAX` / `XI_OMEGA_C` | env | false / 40 / 0.5 | true | **已判负**(四格实测零收益、一格灾难) |
| `NMPC_GEOM_COUPLED` / `NMPC_PAYLOAD_KI` | env | 0 / 0.0 | 1 | 未做 SITL |
| `TRAJ_SCALE_WEIGHTS` | env | false | true | 会连带改 L_omega,两效应混在一起 |
| `DECOUPLE_PUB` / `PUBLISH_HZ` | env | true / 50.0 | — | |
| NMPC `N` / `dt` | code | 20 / 0.05(20 Hz, 1.0 s horizon) | — | 改需动源码 |
| Bryson `L_pos/L_vel/L_att/L_omega/L_T/L_tau_*` | code | 0.1/0.3/0.05/0.3/2.0/τ_max | — | 只在 `traj_scale_weights` 下随 (r,w) 变 |
| `use_mhe` | env | true | false | |
| NMPC 侧 `geom_release_mode` | env | event | self | 与 MHE 侧可分别设 |

### 1.4 D 类:执行器 / PX4 内环整定

| 变量 | 入口 | 默认 | 档位 | 状态 |
|---|---|---|---|---|
| `GRIP_GAIN_PRIOR` | env | −1.0(回落模型侧先验) | 0.2 / 0.3 | 任务信息 |
| `GRIP_GAIN_ENVELOPE` | env | −1.0(关) | 0.5 | **开放**;m_p≥0.2937 撞 cap 5.0 → **必须在轻载测** |
| `scale_px4_rate_gains` | ros | true | false | 与 omega_scale **互斥**(同开=双重补偿) |
| `NMPC_OMEGA_SCALE` + `OMEGA_SCALE_SRC/TAU/CAP` | env | false / thrust / 0.5 / 5.0 | mest | **开放**,无 A/B |
| `grip_arm_d` | ros | 0.47 | — | 几何常数 |

### 1.5 E 类:评估 / 基础设施(**不是因子**,但必须钉死)

`EVAL_TRUE_PAYLOAD_MASS`(只进评估,无通往模型的路径)、`RESID_LOG_DIR`、
`USE_MHE`、`REPS`、`MANIFEST`(断点续)、`PX4_LOG_CAP` / `MAVROS_LOG_CAP`(磁盘护栏)、
`BATCH_STAMP`、flock 单实例锁。

### 1.6 F 类:**已判负 / 禁用档**(不要再进设计矩阵)

| 档位 | 判负依据 |
|---|---|
| `MHE_SCHEDULE_THETA=θ*` + `MHE_CONFIRM_ALPHA=0.9875` | 2×2 析因(20Hz n=8/格)+ 10Hz 对照,无可测收益 |
| `NMPC_TAU_LUMPED=true`(xi 通道) | 08-25 四格实测,零收益 + 一格灾难 |
| `mhe_tau_source=phys` | 改后 attach 段即发散,比原来更早 |
| `geom_release_mode=self`(单独) | 已知不可用;只在配 `estimate_moment` 时才有理由重开 |
| `MHE_PARAM_NOISE=1`(单独) | 参数噪声权重 1e2 vs 物理 1e4~1e5,Hessian 病态 |
| `GRIP_MHE_MP_PRIOR=0` | 不是"去先验",是回落 07-19 修掉的自举 bug 路径 |
| r=10 配 z_high=2.5 | 必发散坠毁 |

---

## 2. 变量的角色分类(规划的真正依据)

把 ~60 个变量按**在本轮研究里的角色**压缩后,设计空间其实很小:

- **① 开放因子(6 个)** —— 还没有 n≥8 实测结论,是本轮要花 SITL 的全部对象:
  `GRIP_NMPC_MP_PRIOR` / `GRIP_GAIN_ENVELOPE` / `GRIP_MHE_MP_PRIOR` /
  `MHE_SEED_FROM_THRUST` / `MHE_N`(含 (r,w) 共线问题) / `MHE_ESTIMATE_MOMENT`
  (+ 半个:`NMPC_OMEGA_SCALE`,只在 prior=0 需要救援时才进场)
- **② 已定案常量(约 15 个)** —— 固定在主线值,写进每批脚本的"显式钉死集",
  **不许吃默认值**(默认会随仓库演进漂移)。
- **③ 覆盖维(4 个)** —— mass / ecc / 速度 / 轨迹类型。不做假设检验,出覆盖统计表。
- **④ 混淆源** —— `GRIP_Z_HIGH`+`GRIP_LIFT_DUR`(随 r 变)、`GRIP_DYN_RAMP`(随 w 变)、
  `ATTACH_WINDOW_SEC`(随圈数变)、`GRIP_DROP_AFTER`。这四个会**跟着**别的因子动,
  是跨批不可比的头号原因,必须在 manifest 里逐格记录实际值。

---

## 3. 开放问题清单

| # | 问题 | 现状 | 现有脚本 |
|---|---|---|---|
| O1 | NMPC 模型侧 dJ 先验能否直接去掉(prior=0) | 08-25 n=7,**前置硬条件挂了**(1/7 LIFT 发散) | `run_prior_ab_4ms.sh` |
| O2 | PX4 内环增益用**包线上界**代替点估计要不要付代价 | 脚本就位,须在 **0.2 kg** 轻载测 | `run_gain_envelope_ab.sh` |
| O3 | MHE 内部几何先验 0.3(真值)→ 0.5(包线) | 脚本就位;+67% 偏差**超出已验的 ±33%** | `run_mhe_prior_ab_4ms.sh` |
| O4 | MHE 质量种子 m_nominal → T_phys/g | 单测 7/7,**无 SITL A/B** | 无(需新建) |
| O5 | 4 m/s 上 m_est 低估 8% 的机制:窗口占周期比 / 纯 N / 纯速度 | 08-19 六格 n=2,**三假设未分离**(r 固定使占比与弧长共线) | `run_mhe_window_scan.sh` |
| O6 | 一阶质量矩增广 s(2b)能否解决 drop 后 m 探下界 | 默认关,无 SITL | 无(需新建) |
| O7 | ω_cmd 缩放 vs PX4 参数缩放 | 默认关,无 A/B | 无 |

---

## 4. 最优实验组合规划

### 4.0 为什么不做部分析因筛选(先回答这个)

6 个开放因子的 2^6 全析因 = 64 格;分辨率 IV 的 2^(6−2) 半析因 = 16 格。
看起来 16 格 × n=8 = 128 轮比"6 个串行 A/B × 16 轮 = 96 轮"更划算——**但不适用**,两条理由:

1. **主响应之一是二值稀有事件**。`前置硬条件`(无发散/无触界)是一票否决判据,
   O1 的失败正是 1/7 轮发散。稀有事件在混杂了别名结构的部分析因里**无法归因**:
   哪一格炸了说明不了是哪个主效应炸的。
2. **仓库已有两次"多开关叠加不可归因"的教训**(08-24 的 `GEOM_COUPLED+λ+self`
   三开关叠加、07-19 的权重缩放连带 L_omega)。单变量纪律不是保守,是被实测逼出来的。

所以最优策略是:**对互相正交且预期非劣的因子用串行单变量配对 A/B;
只对"已知单独会失败"的因子用小型 2×2(把救援机制作为第二因子)**。

### 4.1 通用设计纪律(每批都适用)

- **单变量**:两臂只差一个 env,其余**显式**写死(不吃默认值)。
- **配对 + 交错**:每个 rep 内两臂各跑一次、顺序轮转,rep 内配对做 Wilcoxon signed-rank。
- **判据跑前钉死**:① 前置硬条件(全部 `ok`、`peak_pos_err<2 m`、无触界)一票否决;
  ② 主判据带载 figure8 稳态段 `bias_pct`,**非劣性**检验,等效边界 δ = 1.0 pp;
  ③ 其余(pos_err / solve failed / IQR / 收敛时间)只描述不判定。
- **n ≥ 8 才动默认值**。
- **串行**:flock 单实例;`MHE_N` 与 `estimate_moment` 变更会重 codegen,**按档分组**执行以省启动时间。
- 主线工作点(除非该批就是在测它):
  `mass=0.3 ecc=0.10 r=10 w=0.283(v_peak 4 m/s) z_high=6 lift_dur=8.4 ramp=-1 laps=3`
  `drop=OFF attach_window=140 MHE_N=20 GEOM_COUPLED=0 λ=1.0 geom_source=online floor=0.15`

单轮成本 ≈ 4–6 min(建栈 ~30 s + 接近/attach/lift ~60–90 s + FLY 93 s + cleanup 8 s)。
下面按 **5 min/轮** 估算。

### 4.2 批次表

| 批 | 问题 | 唯一变量(两臂/格) | n | 轮数 | 预算 | 脚本 |
|---|---|---|---|---|---|---|
| **P1-a** | O4 | `MHE_SEED_FROM_THRUST` 0 vs 1 | 8 | 16 | 1.3 h | 新建(照抄 `run_prior_ab_4ms.sh` 骨架) |
| **P1-b** | O2 | `GRIP_GAIN_ENVELOPE=0.5` vs `GRIP_GAIN_PRIOR=0.2`,**@ mass=0.2** | 8 | 16 | 1.3 h | `run_gain_envelope_ab.sh` |
| **P1-c** | O3 | `GRIP_MHE_MP_PRIOR` 0.3 vs 0.5 | 8 | 16 | 1.3 h | `run_mhe_prior_ab_4ms.sh` |
| **P2** | O1 | **2×2**:`NMPC_MP_PRIOR{0.3,0.0}` × `救援{none, OMEGA_SCALE=true}` | 6 | 24 | 2.0 h | 新建 |
| **P3** | O5 | **(r,w) 三点**,N=20 固定(见 §4.3) | 8 | 24 | 2.0 h | 新建 |
| **P3b** | O5 确认 | `MHE_N` 10 vs 20 @ P3 判出的主导机制点 | 8 | 16 | 1.3 h | `run_mhe_window_scan.sh` 改 |
| **P4** | 合流 | 全零任务信息配置 vs 全先验配置,× mass{0.2,0.3} | 8 | 32 | 2.7 h | 新建 |
| **P5** | 论文覆盖 | 终配置 × mass{0.2,0.3} × ecc{0.05,0.10} × v{2,4} | 5 | 40 | 3.3 h | 改 `run_geom_grid.sh` |
| **P6** | 鲁棒性(可选) | 风扰 FPR / m_min 高应力 / O6 estimate_moment+drop | 3–5 | ~40 | 3.3 h | 已有 3 个脚本 + 1 新建 |

**合计 ≈ 184–224 轮 ≈ 15–19 h 纯 SITL**;按 30% 失败重跑余量 → **约 20–25 h**,
即 3–4 个工作日的挂机时间(必须 `setsid` 脱离会话)。

### 4.3 P3 —— 本规划里信息量最高的一批(替代"继续扫 N")

08-19 那批固定 `r=5.0` 扫 (速度 × N),结果**三个假设分不开**:

- 窗口跨度 `T_w = N·dt`
- 窗口占轨迹周期比 `T_w/P = N·dt·w/(2π)` —— 只由 (w, N) 决定
- 窗口内弧长 `≈ v·T_w = √2·r·w·N·dt` —— 由 (r, w, N) 决定

**r 固定时 弧长 ∝ 占比,严格共线**,再加多少 rep 也分不开。
把 **r 放进设计** 就能破共线,且**完全不用碰 N**(于是避开"改 N 同时改测量项相对
到达代价总权重"这个已知的第二重混淆,也不必重 codegen):

| 格 | r | w | v_peak | 周期 P | 占比(N=20) | 弧长 |
|---|---|---|---|---|---|---|
| **A** | 5.0 | 0.283 | 2 m/s | 22.2 s | 9 % | 4 m |
| **B** | 10.0 | 0.283 | 4 m/s | 22.2 s | 9 % | 8 m |
| **C** | 5.0 | 0.566 | 4 m/s | 11.1 s | 18 % | 8 m |

读法:

- **B vs C**:速度相同、弧长相同,只有占比差一倍 → **纯占比效应**。
- **A vs B**:占比相同,速度/弧长差一倍 → **纯速度+弧长效应**。

(第四格 r=10, w=0.566 → v_peak 8 m/s,超出已验包线,**不跑**;三点设计足够读出两个主效应。)

⚠️ 三格必须**统一** `GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4`——r=5 的历史默认是 z=2.5,
若按各自默认跑,高度就与 r 共线,整批作废。ramp 按各档定案值(w=0.283→−1/auto,
w=0.566→−1),manifest 逐格记录实际 `ramp_eff`。

P3 判出主导机制后,P3b 只需一次确认:若占比主导,则在同一工作点把 `MHE_N` 10↔20
(占比 4.5 % ↔ 9 %)应当复现 B↔C 同向同量级的差异。

### 4.4 P2 —— O1 为什么必须是 2×2 而不是 A/B

prior=0 单独跑已经**失败过**(1/7 轮 LIFT 段发散)。再跑一次同样的单变量 A/B
只会得到同一个结论。真实问题形式是"**去掉 dJ 先验之后,靠什么把 LIFT 段的
惯量失配补回来**",那就是一个因子×救援机制的交互问题:

|  | 救援 = none | 救援 = `NMPC_OMEGA_SCALE=true` |
|---|---|---|
| `NMPC_MP_PRIOR=0.3` | 基线(当前主线) | 冗余对照(读救援的副作用) |
| `NMPC_MP_PRIOR=0.0` | 已知失败格(复现) | **目标格** |

- 救援臂**不要用 `NMPC_TAU_LUMPED`(xi)**——已判负,再拿它当救援是重复已知失败。
- 若 O6(`MHE_ESTIMATE_MOMENT=1`)在 P6 里被验成立,它是第二候选救援机制,
  届时把 P2 的第二因子换成它重跑一次 2×2,而不是三臂混跑。
- 前置硬条件按格判:目标格 6/6 无发散才算通过。

### 4.5 执行顺序与理由

```
P1-a (seed)  ──┐  三者信息通路正交(种子 / 内环增益 / MHE 几何),
P1-b (gain)  ──┼─ 互不依赖,顺序任意;但必须串行执行(flock)。
P1-c (mhe先验)─┘  先跑它们:都是预期非劣、风险最低、结果直接固化进后续批的"钉死集"。
      ↓
P3 + P3b  ← 与 P1 无依赖,可插在任意空档;它决定 MHE_N 的最终默认值,
             而 MHE_N 是 P4/P5 的钉死项之一 ⇒ 必须早于 P4。
      ↓
P2 (O1 2×2)  ← 放在 P1 之后:此时 seed/gain/MHE先验三条路已固化,
                 2×2 的"其余钉死项"才不会在批中途改变。
      ↓
P4 (合流确认) ← 只有 P1+P2 全部通过才有意义;任一项判负,P4 的"零任务信息"
                 配置就要改写,提前跑是浪费。
      ↓
P5 (论文覆盖矩阵) ← 终配置冻结后才跑,不做检验只出表。
      ↓
P6 (鲁棒性,可与 P5 并列)
```

### 4.6 命令骨架

```bash
# 通用:长跑必须脱离会话
setsid bash src/scripts/gripper/<script>.sh > /tmp/<tag>.log 2>&1 &

# P1-b(O2:包线 vs 点估计,轻载 0.2kg —— 唯一能测出差异的工作点)
REPS=8 MASS=0.2 setsid bash src/scripts/gripper/run_gain_envelope_ab.sh

# P1-c(O3:MHE 几何先验 0.3 → 0.5 包线)
REPS=8 setsid bash src/scripts/gripper/run_mhe_prior_ab_4ms.sh
python3 src/scripts/gripper/aggregate_prior_ab.py --base mhe0.3 --test mhe0.5 <manifest.txt>

# P3(破共线三点;三格统一 z=6 / lift=8.4)
for cell in "5.0 0.283" "10.0 0.283" "5.0 0.566"; do
  set -- $cell
  GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 USE_MHE=true MHE_C_XY_EST=true \
  GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=online MHE_N=20 \
  GRIP_DYNAMIC=true GRIP_DYN_R=$1 GRIP_DYN_W=$2 GRIP_DYN_RAMP=-1 \
  GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 \
  GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=140.0 EVAL_TRUE_PAYLOAD_MASS=0.3 \
    bash src/scripts/gripper/run_gripper_headless.sh
done
```

### 4.7 每批必须落盘的东西(否则跨批不可比)

1. manifest 头写**完整配置指纹**:全部 A–D 类变量的**实际生效值**(不是"默认"两字)。
2. 逐格记录 `ramp_eff` / `z_high` / `lift_dur` / `attach_window` —— §2 的四个混淆源。
3. 每轮 `nmpc_stamp` + `status` + `peak_pos_err`,支持断点续跑。
4. 判据在 manifest 头写死,聚合脚本只读不改。

---

## 5. 一页速查:主线冻结集(每批脚本照抄)

```bash
GRIP_PAYLOAD_KG=0.3     GRIP_ECC_Y=0.10
GRIP_DYNAMIC=true       GRIP_DYN_R=10.0   GRIP_DYN_W=0.283   GRIP_DYN_RAMP=-1
GRIP_Z_HIGH=6.0         GRIP_LIFT_DUR=8.4 LAPS=3             SETTLE=3.0
GRIP_DROP_AFTER=0.0     ATTACH_WINDOW_SEC=140.0
USE_MHE=true            MHE_C_XY_EST=true
MHE_N=20                MHE_LAMBDA=1.0    MHE_GEOM_COUPLED=0
MHE_EVENT_TRIGGER=true  MHE_SIGNAL_MODE=external
MHE_SCHEDULE_THETA="[-4.0,0.0,0.0,0.0]"   MHE_CONFIRM_ALPHA=-1.0
MHE_SEED_FROM_THRUST=0  MHE_ESTIMATE_MOMENT=0   MHE_PARAM_NOISE=0
GEOM_RELEASE_MODE=event GRIP_GEOM_MP_FLOOR=0.15
NMPC_GEOM_SOURCE=online NMPC_CONTROL_MODE=mhe   NMPC_TAU_LUMPED=false
NMPC_OMEGA_SCALE=false  TRAJ_SCALE_WEIGHTS=false
EVAL_TRUE_PAYLOAD_MASS=$GRIP_PAYLOAD_KG
```

每批**只允许**把其中一项(P2/P3 是两项且为析因设计)改成对照档位。
