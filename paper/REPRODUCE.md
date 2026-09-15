# 论文数字溯源表

**目的**:论文里出现的每一个数字,都能指到一个带时间戳的原始日志,并用一条命令重建。
审稿人问"这个 4× 怎么来的"、面试官问"这个数据可信吗",答案在这张表里。

> **2026-09-14 当前口径：**正文已删除早期残差阶跃触发降权的正面贡献及其 3.8×
> 质量阶跃表/图、权重调度优化和风扰误触发结果；这些历史记录仅供审计。正文保留
> 两项透明消融：(1) 最终主线的 23 个有效 external-event/B 配对；(2) 最终一阶矩
> 接口的 step-on/off 负面结果，用于说明为何部署版本选择 step-off。

> **2026-09-15 口径更新（覆盖上一段中的 23 对 / 31 次）：**
> 1. **表 I/II 数据源换代**：W3/W4 用 `mainline_ab3d_manifest.csv`（09-14 重跑，两臂
>    `MHE_RESID_CONFIRM=false` 即固定权重），W5 沿用 `mainline_ab3b_manifest.csv`（慢基线自触发
>    armed 但 W5 两臂零触发，逻辑等价）。
> 2. **有效性改按物理 attach 判定**：`check_run_valid.py` 用 m̂ 判"是否挂上"，估计器锁死会被误分为
>    attach-fail 而剔出分母。汇总时读 `grip_proximity_<stamp>.log` 的 `attach offset` 重分类，
>    manifest 本身不改；原口径数字并列保留（31/32）。
> 3. 结果：配对 19 对（8/3/8），B 全样本 **32/36**，CP [73.9%, 96.9%]；in-maneuver 3.46 s，
>    brake-to-hover 16.56 s。
>    复算：`RELEASE_VALIDITY=physical RELEASE_MANIFESTS="mainline_ab3d_manifest.csv mainline_ab3b_manifest.csv:W5" python3 scripts/gripper/aggregate_release_table.py`
>    （在 `src/` 下执行；不设两个变量 = 09-11 历史口径，逐字不变）。
> 4. **复现 09-15 之前的任何批次**必须显式设 `MHE_SEED_FROM_THRUST=0 MHE_SEED_WINDOW_BALANCE=0`：
>    这两个开关 09-15 起默认开（窗口力平衡重锚种子）。复现 ab3/ab3b/ab3c 还须
>    `MHE_RESID_CONFIRM=true`（`run_mainline_ab_valid.sh` 现默认 false）。
> 5. **正文压缩、仅在此保留的细节**：自触发降权历史（开启时 W4 A 10/10、B 9/11 越阈；原批次物理
>    attach 口径 33/34）；窗口种子 A/B 全部数字（`seedwindow_ab_manifest.csv`，S0 8/10 有效、
>    S1 10/10；重锚 29→1、最长串 17→1、种子误差 17.1%→2.8%；预注册
>    `~/ros2_ws_HJH/重锚推力种子_预注册_20260914.md` 附录 A）与瞬时推力种子 A/B
>    （`seedthrust_ab_manifest.csv`，S1 drop 前坠机压线）；四箱 4 m/s 的第 3 级误判空载 1/5、
>    不丢箱对照臂 m̂ 偏差 −30 g（2 m/s −14 g，事后分析，非预注册）。

**一键复现**:
```bash
cd /home/clear/ros2_ws_HJH/src    # Git 仓库根目录
bash paper/reproduce.sh            # 校验数据源 → 重建 figs/*.pdf → 编译 main.pdf
bash paper/reproduce.sh --check    # 只校验数据源齐全性
```

`paper/` 位于 `src/.git` 的工作树内；论文实际依赖的冻结实验日志收录在
`paper/data/`，由 `SHA256SUMS` 校验字节级完整性。`reproduce.sh` 优先读取该仓库内目录，
仅为兼容旧工作区才回落到 `../nmpc_test_results/`。

**一条重要前提**:论文数据**不靠重跑仿真复现**。PX4 SITL 没有可注入的随机种子,重跑
得到的是**另一次实现**,不是同一组数字。
因此复现的定义是:**冻结的原始日志 → 图表 → PDF** 这一段逐次一致。原始日志是数据的
唯一真源,不可再生,不要删。

## 2026-09-10 数据—代码一致性审计（投稿前必读）

**结论：目前只能完整复算“冻结日志→图表”，不能对所有历史批次做
`代码 commit→重跑仿真`的字节级复现。** 07--08 月的批次日志绝大多数没有记录
`git HEAD`/dirty 状态，而多个批次发生在实验设施首次提交之前；少数写了 `git_rev`
的 metadata 也**全部同时为 dirty 且未保存 diff 或源码快照**（见下文例外小节），
同样不足以支撑 exact-commit 声明。下表的“首个包含提交”
是根据时间、历史 diff、批次 manifest 和启动日志还原的**最近可审计快照**，
不冒充当时已记录的 HEAD。这一限制不影响冻结数字的重算，但必须在对外的
artifact 说明中保留，不得把“首个包含提交”简写成“运行 commit”。

溯源等级：`E` = 冻结记录明确写入 commit；`R` = 运行时未记录 HEAD，仅能指向首个
包含该设施的提交；`S` = 示意图，没有仿真数据。

`E/R/S` 是本文档定义的**代码溯源等级**，只描述“能否把一次运行钉到一个可审计的源码
快照”。它与代码注释里的 `D1`（`l1_adaptive.py`/`acados_model.py` 中 L1 补偿注入方式的
**控制方案决策点**）无关；`D1` 不是 Git 证据等级，正式表述中不得混用。

### 例外：确实记录了 `git_rev` 的批次，以及它们为什么仍是 `R`

07--08 月并非每份记录都完全没有版本信息。`nmpc_test_results/` 中有 19 份 metadata
写了 `git_rev`，但**全部同时为 dirty，且没有一份保存 diff 或源码快照**：

| metadata 批次 | 份数 | 记录值 |
|---|---|---|
| `gacados_meta_20260702_204636` -- `20260703_020755` | 6 | `git_rev=67fb11c dirty=7` |
| `gacados_meta_20260703_182707` | 1 | `git_rev=71810ab dirty=17` |
| `gacados_meta_2026070{6_160516,7_002533}` | 2 | `git_rev=71810ab dirty=21` |
| `ecc_sweep_meta_*_20260707_16:22--17:10` | 4 | `git_rev=71810ab dirty=22` |
| `ecc_sweep_meta_*_20260707_17:27--17:48` | 6 | `git_rev=71810ab dirty=25` |

三点必须同时说清：

1. **dirty 计数是 7--25，不是单一的 22**；未提交改动的内容从未被冻结，因此
   “运行代码 = `67fb11c`/`71810ab`”这一等式在任何一份记录上都不成立。
2. **这 19 份 metadata 没有一份是论文表格的冻结数据源**。它们属于 07-02--07-07 的
   `gacados` / `ecc_sweep` 批次；论文的 Fig.4 用的是 `cxy_ecc_sweep_20260714_143710`，
   与 07-07 的 `ecc_sweep` 批次不是同一批。
3. ~~它们唯一的论文用途是对 Fig.3 的锚点提供间接旁证~~ —— **2026-09-12 起本条失效**：
   Fig.3 已改用 07-13 的 `nosignal_ablation_20260713_182612` 批次（原因见下表 Fig.3 行），
   不再依赖 07-03 的那两份日志。因此这 19 份 metadata **目前不为论文任何图表提供锚点**，
   本节仅保留作历史记录。（原文：`71810ab` 提交于 07-03 02:14，旧 Fig.3 的两份日志为
   07-03 03:18 与 16:20，同日 18:27 的 metadata 仍记 `71810ab`，把旧图夹在该提交内。）

### 对外（artifact / 审稿回复）的标准表述

允许：

> Most July--August experiment records did not capture a complete executable source
> snapshot, including both Git HEAD and the corresponding dirty-tree contents. Their
> code provenance is therefore classified as reconstructed (R). Referenced commits
> denote the nearest auditable or first-containing snapshots and must not be
> interpreted as the exact commits from which the experiments were run.

禁止：

> The July--August experiments were run from commit `a7b87f1`.

**即使日志记录了 `HEAD`，只要同时为 dirty 且未保存 diff/源码快照，也仍不得作
exact-commit 声明。** 这不否定数据有效性：冻结日志可重新统计，受限的只是
“运行代码对应某个精确 commit”这一主张。删除 07--08 月日志会使论文失去 Table
timing/event/learned/L1/wind 与 Figs.3--5 的证据，因此应保留并诚实标注版本限制。

| 论文项 | 冻结数据/生成源 | 代码溯源 | 实际配置与当前正文口径 | 处置 |
|---|---|---|---|---|
| Fig.1 architecture | `paper/figs/fig_gen.py` | S; 09-10 审计时 `src`=`a38ceed` | 当前 deployed 架构，非数据图 | 一致 |
| Fig.2 window deweight | `paper/figs/fig_gen.py` | S; 机制最早见 `e5c0c32` | 示意图，柱高 0.18 不是实验权重 | 一致；不得把示意值当数据 |
| Table timing | `grip_nmpc_2026073*.log` 冻结集（141 文件/1009 solve 样本）；`grip_mhe_20260803_144036.log` | R; NMPC 设施首收口 `a7b87f1`，MHE timing 首收口 `196cd9a` | **legacy mass-only MHE**，不是正文的 16-state moment MHE；NMPC 13/4、20 Hz 一致，MHE 当时为 14-state mass-only | **已修正**表注、state dim 和正文 headroom 声明;**16-state MHE 已于 09-11 实测并写入正文**(`grip_mhe_2026091*.log`,n=3740,弃每轮前 20 帧:中位 9.2ms / p90 24.6 / p99 59.2 / max 72.8,周期 100ms)⇒ 比 14-state 慢约 7×、尾部占周期 60%,**正文明确不作 onboard-headroom 声称** |
| Table event trigger | `nosignal_ablation_20260713_182612.txt` 中 fixed 与 nosignal 两臂的 24 组 MHE/NMPC 日志 | R; 执行设施首收口 `e5c0c32`，控制指标聚合为 `10efa34` | wrench；mass-only；fixed=不降权，nosignal=1.5 N 残差自触发 | 与最终 eventless interface 无关 |
| Fig.3 mass timeline | `nosignal_ablation_20260713_182612.txt` 的 δm=−0.3 kg 两臂各 6 轮（`fixed` + **`nosignal`**）| R; 与 Table 1 同批同源，执行设施首收口 `e5c0c32` | wrench；mass-only；**纯残差自触发，不消费外部事件** | **2026-09-12 重画**：原图用 07-03 的外部事件武装日志（`mass event [drop] received: armed`），与 eventless 主张冲突，已换到同批 `nosignal` 臂，n=6 中位+min–max 带 |
| Table learned schedule | `alpha_only_20260730_214611.txt` 及其 64 轮日志；10 Hz 补充用 `alpha_only_10hz_20260731_150621.txt` | R; 首个包含设施 `a7b87f1` | mass-only；外部 event；`geom_source=online`，但 `geom_prior_mode=truth`；M0/alpha/rhythm/$\theta^*$ 四臂如 manifest | 正文已标注 geometry prior at truth；不代表最终 estimate-only 主线 |
| Fig.4 $c_{xy}$ sweep | `cxy_ecc_sweep_20260714_143710.txt` | R; estimator=`8fa0fd8`，批次脚本首收口=`7afb97b` | mass-only MHE + **窗外慢滤波的 motor-torque inversion precursor**；非 moment-state MHE；0.08 m 格为旧 `grip_geom_mp_floor=.15` 重跑 | 正文已称 precursor；绘图脚本已改为从该文件解析，不再硬编四个点 |
| paired external-event baseline + autonomous completion | **09-15 换代**：W3/W4=`mainline_ab3d_manifest.csv`（09-14，`MHE_RESID_CONFIRM=false`），W5=`mainline_ab3b_manifest.csv`；manifest、`grip_{nmpc,mhe,proximity}` 日志均已收入 `paper/data/` 并登记 `SHA256SUMS`（四份 manifest 的全部 proximity 日志一并收入，供物理 attach 判定）。 | **E**；ab3d 运行时源码身份见 `mainline_ab3d_manifest.provenance.txt` | A 将 command 当作 detachment event；B 为最终无事件档；有效性按物理 attach（见文首 09-15 说明） | 配对 19 对（8/3/8），A、B 均 19/19；W4 仅 3 对因 A 臂 16 次中 11 次 LIFT 段坠机。B 全样本 32/36（原 m̂ 口径 31/32），CP [73.9%, 96.9%]；in-maneuver 3.46 s，brake-to-hover 16.56 s。4 次失败：3 次未识别载荷（170625、172347、194014，UNRESOLVED 未坠机）+ 185509 释放后坠机。 |
| §Limitations operating limit + estimator failures | 同上两份 manifest | 同上 | 执行器占用取 figure-8 切入 10 s 后至 DROP 指令前的 `[flight-diag]` 5 s 窗 | `python3 scripts/gripper/audit_limit_and_estimator_failures.py`：W4 pitch/yaw 饱和窗比例 98%/53%、scale 中位 4.39；悬停推力 74% Tmax；未识别载荷 4 次（含 W5 031845，已完成）；185509 drop 前不健康 48.2 s、释放后 +1.62 s 四通道全饱和。 |
| command-armed baseline C + unsignaled loss | `paper/data/cmdarm_ab_delayed_20260914_233555.csv`、`..._20260915_024512.csv`（延迟脱离 W5）；`cmdarm_ab_uncommanded_20260915_{001032,025126,063430}.csv`（W5）、`cmdarm_ab_uncommanded_W3_20260915_030417.csv`、`cmdarm_ab_uncommanded_W4_20260915_042341.csv`；`..._000521` 为注入器 bug 作废批，不计入。manifest、provenance、日志均已于 09-15 收入 `paper/data/` 并登记 `SHA256SUMS` | 批次与注入代码见 `夜间实验总结_20260915.md`（ros2-ws-hjh-5f 会话）；预注册 `~/ros2_ws_HJH/推力代理与matched基线_预注册_20260914.md`（含附录 A） | C = B + `release_arm_on_command`（指令开窗 + 推力降到空机 3 s 确认）；有效性按物理 attach | `python3 scripts/gripper/stage_physical_validity.py OUT paper/data/cmdarm_ab_*.csv`，再 `python3 scripts/gripper/aggregate_cmdarm_ab.py OUT/cmdarm_ab_delayed_*.csv OUT/cmdarm_ab_uncommanded_*.csv`：延迟脱离 6 对，提前清 B 0/6、C 0/6，脱离→确认中位 B 7.33 s、C 4.26 s；非计划丢失识别 B 10/18（W5 4/9、W3 4/5、W4 2/4）、C 0/18。（总结文档写的 B 11.5 s 与脚本不符，以脚本为准。）B 的 8 次漏检：2 次从未 LOADED，5 次丢失时一阶矩参考未形成，1 次参考已形成但 ratio=1.83。 |
| §Limitations thrust proxy (±5% estimator-side thrust map) | `paper/data/cmdarm_ab_thrustmap_20260915_033942.csv` + 日志（09-15 收入） | 注入代码 `thrust_map_fault_gain`（mhe_node，默认 1.0）；预注册同上实验 2 | W5；只改估计器读到的 kω²，Gazebo 与 NMPC 不变；每臂 4 轮；**0.95 臂 4/4 原判 attach-fail，物理上均已挂上** | `stage_physical_validity.py OUT paper/data/cmdarm_ab_thrustmap_*.csv` 后 `aggregate_cmdarm_ab.py OUT/cmdarm_ab_thrustmap_*.csv`：G0.95 drop 前质量偏差 −6.78%（=载荷隐形）、完成 1/4、UNRESOLVED 4/4；G1.00 完成 4/4；G1.05 放下后 m_end 2.15 kg（幽灵 +0.09）、UNRESOLVED 3/4；三臂指令前误释放均 0。 |
| pre-command false-release audit (text: 209 flights, 3.7 h, ≤1.4%) | ab3/ab3b/ab3c/ab3d、`delayed_detach_w5_{5pairs,pair5_retry}.csv`、`cmdarm_ab_delayed_*`、`cmdarm_ab_uncommanded_*`、`seedthrust_ab_manifest.csv`、`seedwindow_ab_manifest.csv`（均在 `paper/data/`） | 09-05 检测器修复后的全部非故障批次；**不含** thrustmap（故障注入）与四箱（无指令、按设计有丢失） | 暴露 = 首次 LOADED 至指令/注入丢失；计 `LOADED->EMPTY (drop` 与 `载荷释放(` | `python3 scripts/gripper/audit_precommand_false_release.py <上述 manifest>`：B 类 155 次（2.67 h）+ A 臂 54 次（0.99 h）全部 0 次。C 臂被 `release_arm_on_command` 抑制时只打 `[cmd-armed] suppressed`、不产生上述两类行，审计脚本对 C 恒为 0，故 C 臂**改用 suppressed 行判定**：23 次 / 17.6 min 共 273 行 suppressed，全部在指令或注入丢失之后，指令前 0 次。合计 132（B 类，不含 C）+54（A）+23（C）=209 次、3.66 h，单侧 95% 上界 1−0.05^(1/209)=1.42%。另一会话总结写的 0/139、2.1% 是更小的样本集，未注明组成，正文不用。 |
| §Limitations envelope disclosure | 主线 ab3d/ab3b W5 的 proximity 日志 | `gripper_params.yaml` 的 `r_xy: 0.15` 覆盖了 headless 的 `-p r_xy:=0.13`（ROS 2 按节点名写入参数文件的键优先）；MHE `payload_rxy_envelope` 默认 0.13 → `moment_abs_max`=0.039 偏紧，正确应 0.045 | 真实 \|s\|=m_P·r_xy（attach offset）| W4 13 次中 2 次真实 \|s\|>0.039，两次均未走 moment 路径；其余 11 次中 7 次同样未形成参考 ⇒ 非主因。**代码未修**（改 envelope 会改变检测器行为，需重跑验证）。 |
| partial payload loss (four-box) | `nmpc_test_results/partial_loss_S2_20260915_{005553,013531}.csv`（2 m/s）、`partial_loss_S4_20260915_{020610,061055}.csv`（4 m/s）；`..._004323/004639` 为冒烟，不计入 | 批次与注入代码见 ros2-ws-hjh-5f 会话的 `夜间实验总结_20260915.md`；四份 manifest 与 84 个 `grip_{nmpc,mhe,proximity}` 日志已于 09-15 收入 `paper/data/` 并登记 `SHA256SUMS` | 4×0.05 kg，45/80/115 s 依次无信号丢 3 个；L/N 两臂 ABBA；有效性按箱数（不经 m̂） | `python3 scripts/gripper/aggregate_partial_loss.py paper/data/partial_loss_S2_*.csv`（S4 同理；两份 manifest 的 level 行相加）：2 m/s 入带 14/15、误判空载 0；4 m/s 入带 12/15（压线）、第 3 级误判 1/5。预注册 `~/ros2_ws_HJH/部分丢失四箱实验_预注册_20260915.md`。 |
| delayed physical-detachment fault injection | `nmpc_test_results/delayed_detach_w5_5pairs.csv` + `delayed_detach_w5_pair5_retry.csv`; aggregate with `scripts/gripper/aggregate_delayed_detach_ab.py` | **E**；两份同名 provenance 文件记录源码哈希；冻结 manifest 见 `paper/data/delayed_detach_w5_manifest.csv`，原始日志仍在 workspace `nmpc_test_results/` | W5, 0.15 kg, 2 m/s；控制器正常发 release command，夹爪节点将 DetachableJoint 继续保持 4 s；A/B 顺序交替 | 12 个单臂尝试，1 个 PX4 启动失败，1 个有效 B 无法配对，得到 5 个有效配对。A 提前清模 5/5，B 0/5；实际保持 4.01 s；B 4/5 进入 UNRESOLVED。 |
| final-interface step-on/off ablation | `../nmpc_test_results/stepab_20260912_234849.txt` 与 16 组 `grip_{nmpc,mhe}_<stamp>.log`；每轮 provenance 保存 dirty reconstruction | R；各轮 `VERDICT.txt` 禁止 exact-commit 表述 | W3，8 对；step-on=`resid_step_enable=true`, 0.6 N；step-off=false；两臂 `resid_release_geom=false` | step-on 8/8，step-off 7/8；双方确认的 7 对中延迟差中位 +1.85 s，Wilcoxon p=.578；28 个 on 边沿中 12 个不对应 attach/release。该批尚未收入 `paper/data/`，复现需保留工作区原始日志。 |
| Table release replay | **已冻结**：`paper/manifests/release_replay_20260905.csv`（73 行有序清单 + 每文件 SHA-256，覆盖 `20260904_032225`–`20260905_000352`） | E（判决代码）=`078b054`；被回放日志的生成代码异质且多数为 R | detector 是 078 口径；输入队列混合历史配置 | **09-10 已恢复并核验**（七项指纹逐位一致，见 `manifests/release_replay_20260905.md`）；顺带修掉表内 48/59→53/59 的串阈值错误 |
| Table fastA smoke | `20260905_{150124,...,152924}` 九轮 MHE/NMPC 日志 | R; 运行时 HEAD 候选 `d73fe4a`，日志未写 HEAD | moment=1/frozen；external=false；geom=estimate；ratchet=false；floor=.05；tau=command；motor avg=false | 与表述一致；fastA 已否决，不是当前 detector |
| corrected 12-flight smoke (text) | `20260905_{160013,...,164135}` 十二轮 MHE/NMPC 日志 | R; 最近代码提交 `c43781d` | 同上；fastA 删除 + health gate；尚未加 frozen moment reference | 正文已限定为 fail-safe semantics，不冒充最终 detector 鲁棒验收 |
| Fig.5 legacy full flow | `grip_nmpc_20260715_165245.log` | R; 运行后首个包含 B.5 快照 `9567bbd`（drop bug 紧随修复 `983401b`） | mass-only；external event；geom=online；$\theta^*$，阈值约 2.9 N；不是最终 continuous interface | 正文已标 legacy/$\theta^*$ 例外；峰值 **0.19→0.21 m 已统一** |
| Table L1 matrix | `geom_grid_20260723_171711.txt` 及 60 轮日志 | R; 首个包含 L1/聚合设施 `a7b87f1` | mass-only；external event；M0 1.5 N；geom 为列名 truth/online/l1；旧 geometry floor=.15 | 一致；非最终 moment/estimate 主线 |
| Table wind FPR | `wind_fpr_20260727_150703.txt` 及 27 轮日志 | R; 首个包含批次设施 `a7b87f1` | wrench/no mass event；residual self-trigger；M0 1.5 N；几何/dJ 无关 | 一致；仅支持历史 1.5 N detector |

### 用户点名的 8 个默认值：历史数据实际口径

07--08 月主结果不能沉默继承 09-10 默认。对 Table event/learned/timing/L1/wind 与
Figs.3--5，统一口径是：

- `MHE_ESTIMATE_MOMENT=0` 的历史等价（当时尚无该参数，只估质量）；
  `MOMENT_A_MODE` 不适用。
- `external_event_inputs` 当时尚无开关，节点始终订阅外部 event/attach；仅
  residual/no-signal 臂的**权重调度**不依赖外部事件。这不等于当前
  `external_event_inputs=false` 的 sensor-only 接口。
- `geom_source` 依表臂为 `truth/online/l1`；当时的 `online` 是窗外 $c_{xy}$ 前身，
  不等于当前原子 `m/s/J` 的 `estimate`。
- `DJ_RATCHET` 和 `grip_dj_floor_mp` 尚不存在。部分 gripper 批次有旧参数
  `grip_geom_mp_floor=0.15`；不得将它写成当前 `DJ_RATCHET=false` +
  `grip_dj_floor_mp=0.05`。
- MHE 已知输入的历史等价是“电机转速反算 $T_{phys}$ + NMPC 意图力矩
  $\tau_{cmd}$”，即后来 `mhe_tau_source=command` 的口径；电机值取最新样本，
  等价于 `motor_window_avg=false`。

09-04 的 continuous-interface A/B 和 09-05 上午的两批 smoke 才使用
`moment=1 / A=frozen / external=false / ratchet=false / floor=.05 / tau=command /
motor_avg=false`；B 臂 `geom=estimate`，A 臂是明示的 legacy event/online 对照。

### 投稿阻断项（未解决前不得声称“按图复算全部闭合”）

1. ~~**Table release replay cohort 未冻结**~~ ✅ **09-10 已解**：cohort 已恢复并冻结为
   `paper/manifests/release_replay_20260905.csv`。**"无界 glob" 是记账错误** —— 同一
   截止时刻下无界 glob 给出 249 轮，与表里每个数字都不符；真实 cohort 是按 mtime
   排序、截至审计提交 `078b054`（2026-09-05 14:37:33）的**最后 73 个可回放轮次**，
   由七项独立指纹唯一确定（73/72/58/14/2614 帧/2.1 s/peak 0.0134×残噪 0.0203→ratio
   1.520/valid 59/LOOCV 59-59 选 0.30/留出 90 %）。生成器 `scripts/gripper/
   freeze_release_cohort.py`，溯源与核验表见 `manifests/release_replay_20260905.md`。
2. **大多数 07--08 月批次无 E 级 commit**：可用首个包含提交审计算法，但不可
   声称当时工作区 clean。无法追溯时应标 `R`，不得猜成 exact commit。少数写了
   `git_rev` 的批次同样是 `R`（全部 dirty，无 diff 快照），理由见上文例外小节。
   2026-09-11 起，当前四个底层实验入口统一调用 `scripts/record_experiment_provenance.sh`：
   每轮冻结 workspace/PX4/acados 的 HEAD、dirty 清单、tracked binary patch、未跟踪
   文件及 submodule 状态，并写出 `VERDICT.txt`。只有三个仓库均 clean 时才允许
   exact-commit 表述；dirty 轮次即使已有完整源码快照，也必须明确标为 `PROHIBITED`。
3. **"post-fix four flights"（49/268）身份已核验（2026-09-11）**：数据源为
   `grip_{mhe,nmpc}_20260905_{175935,180315,180710,181106}`。口径 = 每轮 MHE 日志中
   带 `|s|=` 数值的 `[s-collapse]` 行、截至同轮 NMPC `DROP command issued` 之前
   （"估计不健康,释放判决暂停"提示行不含 |s|，不计）。四项指纹逐位吻合：样本 **268**、
   越 0.039 kg·m **49**（18.3%）、最大 **0.0520**、全部样本中位 **0.0301** kg·m；首次越界
   在 attach 后 26.6–27.3 s。⚠️ 受影响轮次是 **3/4**（`181106` 无越界），原记"4 轮受影响"
   有误。该段正文位于 main.tex 的 `\iffalse` 块内，不进 PDF。

### 从冻结数据重算表格

旧聚合脚本在 `efe7986` 被退役，但仍在 Git 对象中。为避免将当前默认注入
历史批次，必须用表中指定提交的 parser：

```bash
cd /home/clear/ros2_ws_HJH/src
audit_e5=$(mktemp -d /tmp/paper-e5.XXXXXX)
git archive e5c0c32 | tar -x -C "$audit_e5"
python3 "$audit_e5/scripts/masschanger/aggregate_nosignal.py" \
  paper/data/nosignal_ablation_20260713_182612.txt

audit_a7=$(mktemp -d /tmp/paper-a7.XXXXXX)
git archive a7b87f1 | tar -x -C "$audit_a7"
PYTHONPATH="$audit_a7/scripts/masschanger:$audit_a7/scripts/gripper" \
  python3 "$audit_a7/scripts/gripper/aggregate_alpha_only.py" \
  paper/data/alpha_only_20260730_214611.txt
PYTHONPATH="$audit_a7/scripts/masschanger:$audit_a7/scripts/gripper" \
  python3 "$audit_a7/scripts/gripper/aggregate_alpha_only.py" \
  paper/data/alpha_only_10hz_20260731_150621.txt
PYTHONPATH="$audit_a7/scripts/masschanger:$audit_a7/scripts/gripper" \
  python3 "$audit_a7/scripts/gripper/aggregate_geom_grid.py" \
  paper/data/geom_grid_20260723_171711.txt

cd /home/clear/ros2_ws_HJH/src
# 表 II(B-only 31 次)以及正文中依赖样本数的全部统计:
# 完成率与 CP 区间、延迟双峰、释放路径、状态机转换、自触发声明、ovr Fisher、作废统计
python3 scripts/gripper/aggregate_release_table.py paper/data

# 09-04 旧批(已不是表 II 来源,仅备查)
python3 scripts/gripper/aggregate_mainline_ab.py paper/data/mainline_ab2_manifest.csv
python3 scripts/gripper/verify_mainline_ab2.py
```

`aggregate_release_table.py` 的口径写在脚本 docstring 里,要点:
- 纳入样本 = 三份 manifest 中 `arm=B` 且 `validity=valid` 的全部飞行；A 臂不进入论文统计。
- IQR 的 p25 用 numpy `method='nearest'`、p75 用 `method='higher'`。这是 21 对表格当初的
  实际口径(单一 numpy 方法复现不出全部格子),为保持可比沿用;属非标准混合口径。
- `ovr` = drop 指令之前 `[s-collapse]` WARN 行"本轮累计 N 帧"的最大值,**不是**普通行的
  `ovr=` 字段(后者只在 moment 基准就绪后才打印)。

上述 `git archive` 会把 parser 和其 helper 从**同一提交**一起恢复；不得与当前
parser 混用。复算后可删除 `$audit_e5`/`$audit_a7` 临时目录。

---

## 主结果

| 论文位置 | 数字 | 数据源 (`nmpc_test_results/`) | n |
|---|---|---|---|
| Table 1 | 自触发 settling 1.20→0.32 s (−0.3 kg,**3.8×**) | `nosignal_ablation_20260713_182612.txt` 的 fixed/nosignal 两臂 | 6/格 |
| Table 1 | 自触发 settling 1.57→0.57 s (−0.8 kg,**2.75×**) | 同上 | 6/格 |
| Table 1 | pos_err 峰 0.200→0.173 m / 0.545→0.482 m | 同上 | 6/格 |
| §IV-C | **N 版** CEM 训练 loss 11.05→9.74(−12%),96 集 0 失败 | `工作日志_20260710.md` §1 | 96 集 |
| §IV-C | N 版 θ\*=[−4.58,−1.46,0.34,−1.30] @1.485 N | 同上 | — |
| §IV-C | N 版留出 0.225 kg × {0.05,0.075} m 与 M0 **持平** | `工作日志_20260710.md` §2 | 3 配对 |
| §IV-C | **α 版** CEM 训练 loss **不降**(10.46→10.67),α 撞上界 0.9875 | `工作日志_20260710.md` §4 + 记忆 `cem-benefit-refuted` | 96 集 |
| §IV-C | α 版留出 **6/6 全胜**(Δloss +0.914/+0.557) | `工作日志_20260710.md` §4 | 3 配对×2 点 |
| §IV-C | 上一行的 M0 基线用 **0.8 N** 阈值而 Table 2 用 1.5 N → 已用三臂配对消解,见下 | `alpha_only_20260803_155130.txt` | 8/臂,24 runs |
| §IV-C | 三臂 settle 不可区分(+0.00/−0.00/+0.00 s,p≥.375) | 同上 | 8/臂 |
| §IV-C | 0.8N 基线控制层更差(pos_err 峰 +0.104 m,p=.031);θ\* 对它的优势换 1.5N 后消失 | 同上 | 8/臂 |
| §IV-C | 确认延迟按阈值严格单调 1.28/2.24/2.43 s(7/0,7/0,8/0)=操作检验 | 同上 | 8/臂 |
| Table 2 | 2×2 析因全部不显著(最小 p=.055),总效应中位 0 | `alpha_only_20260730_214611.txt` | 8/格,64 runs |
| Table 2 | 功效 97% / 87%(d_z=1.57/1.29) | 由上表配对差 σ=0.407/0.311 s 算得 | — |
| §VI-B | 10Hz 复现:0.2 kg **+0.70 s**(θ\* 更慢,仅 5/8 收敛) | `alpha_only_10hz_20260731_150621.txt` | 8/格,32 runs |
| §VI-B | M0 自身 10Hz 回到历史值 3.83 vs 3.89 s(设施可复现铁证) | 同上 | 8 |
| Table 4 | 暂态峰:逐格 L1 vs online 两格 p=.008/.012(未校正),8 次检验校正后不显著 → 正文改为"不声称暂态优势"并给出依据(原写"statistically indistinguishable"无检验) | 同上 | 5/格 |
| Table 4 | L1 三方矩阵,60/60 稳定零发散 | `geom_grid_20260723_171711.txt` | 5/格 |
| Table 4 | L1 恢复慢 **8–32%**(逐格配对 vs `online`,中位 +16.1%;4/4 格分别 +15.2/+8.2/+31.7/+16.9%,3 格 p<.05) | 同上 | 5/格 |
| — | ⚠️ **2026-09-12 勘误**:原记 15–35% 是拿 L1 区间端点除 ours 区间端点(跨格混搭 4.24/3.70、4.90/3.62),非逐格配对,上下界都不成立;正文与摘要已改。摘要原写"我们快 15–35%"与正文"L1 慢 15–35%"方向不等价(慢 16% ⇔ 快 13.8%),一并改掉 | | |
| Table 5 | 垂直风 FPR 在 1.5 N 阈值锐阶跃(0/3 → 3/3) | `wind_fpr_20260727_150703.txt` | 3/格 |
| Table 5 | 水平风 2× 阈值(3 N)仍 0/3 误触发 | 同上 | 3/格 |
| Table 6 | NMPC solve p50 1.3 / p99 2.2 ms @20 Hz | `grip_nmpc_2026073*.log` 池化 | 1009 样本 |
| Table 6 | MHE solve 均值 1.48 / 单帧峰 5.1 ms @10 Hz | `grip_mhe_20260803_144036.log` | 63 窗口×20 解 |
| Fig.3 | fixed/**nosignal** 质量估计时线 (δm=−0.3 kg) | `nosignal_ablation_20260713_182612.txt` 所列 6+6 轮 | 6+6 轮 |
| Fig.4 | c_xy 在线估计 vs 真值 | `cxy_ecc_sweep_20260714_143710.txt` | 4 格 |
| Fig.5 | B.5 抓取→8字→投放全流程 | `grip_nmpc_20260715_165245.log` | 1 轮 |

## 平台归属(**审稿关键**,勿混)

| 平台 | 机制 | 改变什么 | 承载哪些结论 |
|---|---|---|---|
| **P1** wrench | `Link::AddWorldForce` 每步注入 `(m_loaded−m_empty)·g` | **仅有效平动质量**,转动惯量不变 | Table 1 / Fig.3(事件触发、无信号消融)**仅此** |
| **P2** DetachableJoint | 磁吸夹爪物理吸附刚体 | 质量+质心+惯量,**全真** | Table 2/4/5、Fig.4/5,一切涉及几何/惯量/偏心的结论 |

P1 不能改运行时惯量是 **gz-sim Harmonic 的已知限制**(`SetComponentData<Inertial>` 对
DART 是 no-op,ECM 组件变了但积分器仍用装载时的质量,gz-sim issue #2733),不是建模选择。
已写进 §V 与 §Limitations。**过冲深度这类依赖惯量的量,永远不从 P1 报**。

## 复现命令逐条

```bash
# 图 1–5(论文正式矢量版)
(cd paper/figs && python3 fig_gen.py)

# 图 3/4/5 的另一套(诊断用大图,含更多子图)
python3 scripts/make_paper_figs.py

# 编译论文
(cd paper && pdflatex main.tex && bibtex main && pdflatex main.tex && pdflatex main.tex)

# Timing 表原始数据重采(会跑一轮新仿真,得到的是新样本不是同一批)
bash scripts/gripper/run_gripper_headless.sh     # 约 150 s 后清栈
grep -oP "solve=\K[0-9.]+" ../nmpc_test_results/grip_nmpc_<stamp>.log   # NMPC
grep -oP "solve=\K[0-9.]+" ../nmpc_test_results/grip_mhe_<stamp>.log    # MHE
```

## 基线阈值口径混杂 — 已消解(2026-08-03)

**问题**:07-10 那次留出 6/6 全胜是 CEM 路线唯一的正面证据,但其 M0 基线用固定 **0.8 N**
阈值,而 2×2 析因(Table 2)用 headless 默认 **1.5 N**。若 0.8 N 本身更差,6/6 就只是
"打赢了一个选差了的基线"。

**做法**:三臂配对 `M0@1.5N / M0@0.8N / θ*`,工况取 07-10 的留出判决点
`0.225 kg × 0.05 m`(两轴都不在 CEM 训练网格上;小偏心 σ 最小),两个 M0 臂的节奏维
逐字相同、**唯一变量就是那个标量阈值**。n=8,3 阶拉丁方,24/24 全 ok 零发散。
预注册见 `三臂基线阈值消解_预注册_20260803.md`(开跑后、出结果前写)。

**结果**:
- 主指标 `settle` 三臂**不可区分**(+0.00/−0.00/+0.00 s,p≥.375)→ 阈值混杂在主指标上
  量级低于检出下限,**再次印证"宽平台"**。
- 控制层上 0.8 N 确是更弱的基线(pos_err 峰 +0.104 m,p=.031),且 **θ\* 对它的优势
  (−0.091 m p=.047、恢复 −0.15 s p=.016)在换成 1.5 N 基线后全部消失**(+0.027 m、
  +0.03 s,均不显著)→ 早期 6/6 **部分是基线假象**。
- 操作检验:确认延迟按阈值严格单调(0.8N 1.28 s → 1.5N 2.24 s → θ\*@2.18N 2.43 s,
  三条 7/0、7/0、8/0 全显著)→ 阈值确实生效,而**不转化为 settle 收益**,独立复现
  Table 2 的核心发现。

**残留不确定性**(论文已如实写):本批只控了阈值这一项。07-10 与当前设施还差
求解频率(10 Hz vs 20 Hz)、几何路径(floor 棘轮 vs 完全解耦),且它按 CEM 复合 loss
判分,而该损失函数随 07-31 代码瘦身已删、无法复算。故结论是"阈值解释了**一部分**",
不是"早期结果已被完全解释"。

**一处右删失**:0.8 N 臂有 1 轮在观测窗口关闭时距真值 0.037 kg、超出 0.034 kg 带宽
(差 0.003 kg)被判 parse_fail 剔除,n=7。丢的是该臂最慢的轮次 → **低估**其劣势,
方向对结论保守。

## 各节实际使用的权重调度(逐批核实,2026-08-03)

论文声明"保留手工规则 M0"必须与数据一致,故逐批查了 `grip_mhe_*.log` 里
`weight schedule theta = ...` 那行:

| 论文位置 | 批次 | theta | 阈值 |
|---|---|---|---|
| §VI-A Table 1 | `nosignal_ablation_20260713` | `[-4,0,0,0]` M0 | 1.5 N |
| §VI-C c_xy 在线 | `grip_mhe_20260714_013435` | M0 | 1.5 N |
| §VI-C 几何解耦重验 | `b4_matrix_dec_20260723` | M0 | 1.5 N |
| §VI-C 几何 A/B | `geom_ab_20260730` | M0 | 1.5 N |
| **§VI-D 全流程 Fig.5** | `grip_*_20260715_165245` | **θ\* α版** `[-4.80,-1.21,0.49,-0.96,0.99]` | **2.9 N** |
| §VI-E Table 4 L1 三方 | `geom_grid_20260723` | M0 | 1.5 N |
| §VI-E Table 5 风扰 | `wind_fpr_20260727` | M0 | 1.5 N |

**唯一例外是 §VI-D**(定性全流程演示):录于 2026-07-15,早于 CEM 结论撤回,用的是 θ\*。
论文 Fig.5 的实际绘图源是 `grip_nmpc_20260715_165245.log`；旧文档曾误写为
`gviz_*_20260715_204725`，已在 09-10 审计中纠正。补充视频仍可能使用 20:47 轮，
因此**不得再声称图与视频同源**，除非视频脚本也改为 16:52 并重生成。
当前处理:论文 §VI-B 主动声明该例外,并指出 θ\* 与 M0 已被证统计不可区分、
该图不承载定量比较,故结论不受影响。

## 已知的取值口径

- `t = 0` 定义为 **`T_phys` 越过基线**的物理生效时刻,不是指令下发时刻。
- **入带**定义 ±0.08 kg 或阶跃的 15%（取大者；−0.8 kg 格为 ±0.12 kg）;主指标是 **settling(驻留收敛)**,不是 first-entry。
  first-entry 在本数据上不稳健(估计值可穿带再出),会把 M0 与 θ\* **排反**——
  一个早期的 −28% 结论就是这个指标的伪影,已撤回(§VI-B 末)。
- 所有学习相关比较一律 **run 内配对差**,绝不跨批次比绝对值:M0 基线本身跨会话
  漂移 0.9 s,大于被测效应。
- 发散判据:pos_err 峰 > 2.0 m。

## §VI-F 自主释放确认(2026-09-04)— 数据与分析口径

**数据**:`nmpc_test_results/mainline_ab2_manifest.csv`,76 架次 / 38 次尝试配对
(W3/W4/W5 各 8/6/8 个**有效**配对)。批次脚本
`src/scripts/gripper/run_mainline_ab_valid.sh`,验收脚本
`src/scripts/gripper/verify_mainline_ab2.py`,有效性判定
`src/scripts/gripper/check_run_valid.py`。

**预注册**(跑批之前定的,不是事后筛):drop 之前坠机 / attach 未兑现 = 该配对作废
并补跑,发生率单列;每格尝试上限 16 次,凑不满 8 对如实少报(W4 即为 n=6)。

**两处分析口径,复算前务必知道**:

1. **A 臂的 drop 完成判据与 B 臂不同**。A 臂(event 档)打 `DROP: released`,
   它的 drop 由自己的释放指令定义;B 臂(continuous 档)打 `DROP complete`,
   由 confidence 持续达标定义。**第一版验收脚本的正则漏了这个分支**
   (`\| DROP (?:command issued|: released)` 少了对 `DROP:` 无空格形式的匹配),
   把 A 臂 27 个有效轮全判成"drop 未完成",并且连带使 A 臂的
   `t_drop_cmd` 为 None → drop 后失稳判定退化成**全轮**判定。
   后者方向是保守的(全轮无失稳强于 drop 后无失稳),所以当时的
   "A 臂无失稳"结论不受影响;但表格里的 A 臂完成率是错的。
   已修正为 `\| DROP(?: command issued|: released)` 并**重新聚合**,
   论文表 II/III 用的是修正后的数字。

2. **性能比较是条件量**。表 III 的所有指标都条件于"抓取成功且存活到释放指令",
   而两臂的作废原因和作废率不同(A 臂 drop 前坠机 11/38、B 臂 4/38;
   attach 未兑现 A 0/38、B 3/38)。因此表 III 不能单独解读,必须与表 IV 同看,
   论文正文也已就此显式声明幸存者偏差。

> ⚠️ **2026-09-11 更新**:以下 09-04 的数字**已被最终代码上的重跑取代**(见审计矩阵该行)。
> B-only 结果：**31/31 完成**，CP 双侧 95% 区间 **[88.8%, 100%]**；延迟**双峰**——
> in-maneuver **3.46 s** [2.86, 4.46](42%)、via brake-to-hover **16.48 s** [16.36, 16.60](58%)，max 16.70 s。
> 释放路径 moment 19/31、质量域 7/31、MHE 未判 5/31；状态机恰两次转换 26/31；
> ovr：4 m/s 下 ovr=0 时 in-maneuver 7/8 vs ovr>0 时 1/12（Fisher p=0.0008）。
> **不得把两支合并成单一中位数**。以下保留为历史记录。

**统计**:22/22 的 Clopper--Pearson 双侧 95% 区间 = [84.6%, 100%],论文按此表述,
**不**写成"已证明失败率为零"。未做预注册等效性检验,故性能结论写
"未观察到具有工程意义的退化",**不**写"两臂不可区分"。

**机制验证**(表 II 脚注对应):22/22 轮状态机恰好 2 次转换、`s_out` 单调衰减至零
(100 帧全记录)、conf 完成值 0.997–0.999。注意内部 `s_est` 释放后**并不归零**
(持续在 3–16mm 震荡)——这正是"必须切发布目标、光调 conf 阈值绕不过去"的证据。

## §VI-F release detector 分布审计（2026-09-05）

### 09-10 配置总开关审计

`c_xy_est_enable` 不只是其名字所暗示的 CoM 在线估计开关。当前实现中，
`mhe_node.py::_update_c_xy_est()` 同时包含质量域释放判据、统一 detector，以及
`payload_estimate.release_decision()` 的唯一在线调用点；而该函数只在
`c_xy_est_enable=true` 时执行。参数声明和通用脚本
`run_gripper_headless.sh` 的默认值均为 `false`。

论文使用的 13/13 个 release 批次脚本均显式覆盖为 `MHE_C_XY_EST=true`，因此已报告
批次的 detector 结果有效；但它们不是裸默认配置的行为。未带该覆盖直接启动时，
在线 release decision 不运行，drop 后必然由 NMPC 的 unresolved timeout 兜底，不能
将这种结果记为 detector miss 或 detector 安全验证。复现任何 release 表格时必须把
以下配置写入 manifest 并在日志启动段核对：

```bash
MHE_C_XY_EST=true
```

这是待解耦的软件配置缺陷：后续应给统一 detector 独立开关，或在 eventless profile
中无条件执行，同时对“eventless + detector disabled”的矛盾启动配置直接报错。

**代码口径**：审计结果对应提交 `078b054`。其中
`src/offboard_test_acados/offboard_test_acados/payload_estimate.py` 的
`release_decision()` 是在线节点和离线回放共同调用的唯一判决函数。论文不能把
`dJ` 当成 mass 之外的独立一票：连续接口里 `dJ=μ(m_p)r_z²`，两者是同源证据。

**离线回放（冻结 cohort，可一键复现）**：

```bash
cd /home/clear/ros2_ws_HJH/src
python3 scripts/gripper/replay_release_detector.py \
    --manifest paper/manifests/release_replay_20260905.csv --loocv
```

该命令校验清单里 146 个文件的 SHA-256 后按冻结顺序回放，输出与提交 `078b054`
当时的审计逐位一致：73 轮（72 有 drop）、**58/72 检出、14/72 漏检**、带载
**2614 帧零误释放**、已检出轮延迟中位 **2.1 s**、对抗最坏 `ratio=1.520`、
LOOCV **59/59 折选 `ratio=0.30`**（留出轮成功 53/59 = 90 %）。剔除
`check_run_valid.py` 判 invalid 的架次后为 **53/59 检出、6/59 漏检**。

⚠️ 三条限定仍然成立：

1. **`48/59` 是 `ratio=0.20` 的数字**，不是审计阈值 0.30 的。论文表曾误写成
   0.30 下的 valid 子集，**09-10 已改为 53/59、6/59**。
2. LOOCV 只说明 0.30 优于候选网格里的旧 0.20，**不等于 detector 已通过鲁棒验收**。
3. 2614 帧零误释放已被下述在线 smoke 否证，**不能**继续作为当前 detector 的安全
   证据。表的作用限于阈值选择依据与漏检结构。

cohort 的定义、"无界 glob" 记账错误的纠正、七项指纹核验表与 validity 列来源，见
`paper/manifests/release_replay_20260905.md`；清单生成器
`scripts/gripper/freeze_release_cohort.py`。

📌 **这张表当前不在 `main.pdf` 里**：`main.tex` 只 `\input{sec_autonomous_release_concise}`，
而 Table release replay 与 Table fastA smoke 都只存在于**从未被引用过的**详版
`paper/sec_autonomous_release.tex`（`git log -p -- paper/main.tex` 里 `\input` 行只
出现过 concise 版一次）。所以 #9 的"投稿阻断"级别与 #11 同类，是**记账误判**；
cohort 仍已冻结，详版素材若要搬回正文可直接复算。

**绝对一阶矩不能作判据**：五轮分布采样来自：

- `gviz_mhe_20260904_234516.log`
- `grip_mhe_20260904_234903.log`
- `grip_mhe_20260904_235647.log`
- `grip_mhe_20260905_000017.log`
- `grip_mhe_20260905_000352.log`

带载 `|s|` 最小 0.0039 kg·m，低于卸载后最大 0.0049 kg·m，分布重叠。绝对值
只能作 sanity 上限；判别来自相对塌陷与载荷量证据的合取。

**随机 fast-path smoke（2026-09-05）**：计划 12 轮，在第 9 轮后停止，因为
`fastA = residual + quantity` 已在 3/9 个飞行中提前宣告释放。原始日志时间戳：

- `20260905_150124`, `150454`, `150824`, `151154`, `151524`, `151854`,
  `152224`, `152554`, `152924`
- #1--#5、#7：仅在真 drop 后宣告，延迟 +0.8～+1.1 s
- #6：−18.1 s、+1.1 s
- #8：−39.9 s、+1.0 s
- #9：−51.1 s、−24.9 s、−4.8 s、+1.2 s

代表性首个误判帧为 `ratio=0.694, moment=0, quantity=1, resid=1,
m_p=-0.0624`。带载段 `m_p` 的 p05 为 −0.1032 kg；低于 0.03 kg 的最长连续
时间中位 14 s、最坏 48 s。排除精确触及质量下界后，仍有 13.6% 的带载帧满足
`m_p<0.03`，所以“触界排除”只能缓解，不能让 quantity 独立可靠。首个误判之前，
各轮带载 moment ratio 最低 0.789，对 0.30 阈值仍有 2.6× 裕度。

#9 的三次提前宣告不能当作三个独立样本：第一次 fastA 误判先把发布的 `s` 目标
切到零，真实载荷仍在又使 presence 重武装，随后才出现 slow/fastA 连锁宣告。
因此论文报告“3/9 个飞行受影响、共 5 次提前宣告”，并明确后四次里含生命周期
污染。在线日志而不是事后回放是这项安全结论的主证据。

**删除 fastA 后的 71 轮回放不能把 `152924` 当作干净反事实**：它报出的 slow
提前 16.6 s 对应旧在线轮次里的第二次释放，而该轮此前已经发生 fastA 误释放、
`s_target` 清零和 presence 重武装。原始日志还给出了更直接的上游链：slow 触发前
连续 5 次 `MHE solve failed`，随后 `re-anchored ... after 5 consecutive failures`；
重锚代码明确执行 `self.s_est = zeros`。释放判据在新成功解到来前仍运行并累计这份
无效零状态。因此这 1 个回放误释放既不能用于估计新版在线 FPR，也不支持事后增加
`ratio>ε`；它支持的是对 release evidence 加 health/freshness 门控，并在 solve
failure、stale 或 re-anchor 时清零 slow/fastB persistence counter。后续日志应在
每条 `[s-collapse]` 中记录 `health` 和 `solution_age`，否则旧的 1 Hz 诊断行无法
忠实重建在线 10 Hz 的有效帧序列。

**论文结论边界**：主线 22/22 证明连续接口在有效轮中能够完成 drop；它早于当前
三源判据，不能当作当前 detector 的 22 轮验证。当前 detector 的历史回放仍有
14/72 漏检；审计时 M0 只暴露 4.4 N residual 门，高于 0.15 kg 载荷的 1.47 N
重量台阶。
因此正文必须写“deadlock fixed / interface demonstrated”，不能写“robust detector
validated”。release-only 短窗残差已在 `0e0c984` 实现，启动脚本续行问题在
`d73fe4a` 修正；但在线 smoke 已否决不含 moment 的 fastA。后续安全版本只能保留
moment-gated 的 slow（quantity+moment）与 fastB（strong residual+moment）。若超时，
结果必须保持为 unresolved 并进入保守 hover/land abort，不能把超时提升为空载证据，
也不能清模型或复位自适应状态。历史 `ratio=1.52` 是边界样本的合成组合；在没有
实测联合分布证据前，宁可把它记作漏检风险，也不能为覆盖它保留已实测 3/9 误释放
的 fastA。moment 必须进一步限定为来自最新一次健康、fresh、成功的 MHE 解；重锚
初始化值不具备投票资格。该门控做完并重新跑无污染 smoke 前，slow/fastB 也不能
宣称鲁棒通过。

## fastA-free + health gate 在线 smoke（2026-09-05）

12 轮时间戳：`160013 160343 160713 161043 161413 161743 162128 162459
163049 163434 163805 164135`（统一前缀 `20260905_`）。预先约定的安全验收是：
带载段零释放，真实 drop 后要么得到有效证据完成释放，要么进入 unresolved 且不清
状态。结果 12/12 满足：

- 带载段提前释放 0/12；对照 fastA 版本为 3/9，最早 −51.1 s。
- 11/12 得到有效释放，detector 延迟中位 1.9 s；路径 fastB 9、slow 2。
- 3/12 曾进入 12 s unresolved；`161743`、`163049` 在 +14.8/+14.2 s 收到迟到
  证据并 RESOLVED，`162459` 保持 unresolved 终态。
- `162459` 未切 `s_target`、未清模型、未复位 L1/ξ、未置 `grip_dropped`，符合
  fail-safe 的**估计器/模型状态保持**语义；不代表飞行参考已经切到悬停。

**历史配置更正（09-10 代码审计）**：上述 12 轮所用版本在 unresolved 时只执行
`grip_dynamic_active=false`，`ref_fn` / `ref_window_fn` 从未切离 `_grip_dyn_ref` /
`_grip_dyn_ref_window`，所以这些日志不能标注为“退出机动转保守悬停”。后续独立
`_grip_dyn_latched` 只修掉了 figure-eight 原地重启，也仍未兑现悬停语义。当前工作树
新增从实测状态连续制动、有限时间后固定停止点的 reference，并同时替换单点与窗口
函数；该控制动作在完成构造 SITL 回归前只能标为“已实现、未飞行验收”。

**`162459` 的独立覆盖缺陷**：日志有 `[payload-state] EMPTY->LOADED (attach,
m_p=-0.103kg)`，说明残差 ATTACH 已经令自主 `_load_armed=True`；但之后从未出现
“质量域判据已武装”，也没有 `[s-collapse]`。代码原因是统一 release detector 的
外层仍要求旧 `_c_xy_mass_armed`，后者必须 `m_p>0.09kg` 连续 20 帧。这把自主
load latch 与旧质量武装再次串联。修复方向是仅从统一 detector 外层移除
`_c_xy_mass_armed`，保留 `_load_armed + payload health/freshness`；旧质量域释放
分支仍保留自己的 `_c_xy_mass_armed`。

同轮真实 drop 后 `m_est` 回到约 2.049 kg，但 raw `s_hat` 留在约
0.018 kg·m，故 moment 永不成立。这是 estimator coverage/observability 的漏检，
不能用 NMPC drop command 强制清零，否则重新引入被本文排除的事件捷径。历史两轮
实际都没有切到悬停，不能用于判断 hover 是否帮助 moment 恢复；在专门 hover vs
低幅辨识激励 A/B 之前，悬停只按安全动作解释，不按辨识策略解释。

## 武装解耦后的停止批次与 moment-reference 更正（2026-09-05）

提交 `ddfe9d2` 将统一 detector 移出 `_c_xy_mass_armed` 外层，只保留
`_load_armed + health/freshness`。随后四轮 `175935 180315 180710 181106`（统一前缀
`grip_mhe_20260905_`）不是验收结果：批次在发现 running `_s_peak` 污染后停止。

最初把大 peak 归因于“attach 初期瞬态”是错误诊断。按自主 attach 时刻重新对齐后，
新旧两批的越界样本都从 attach+26～28 s 才开始，对应 DYNAMIC/figure-eight ramp：

- A 修复前 11 轮：49/644 帧越过 0.039 kg·m（7.6%），4 轮受影响；中位 0.0215，
  最大 0.0492 kg·m。
- A 修复后 4 轮：49/268 帧越界（18.3%），**3 轮**受影响（`181106` 无越界）；全部样本
  中位 0.0301，最大 0.0520 kg·m（均按 DROP 指令前窗口计，2026-09-11 已用原日志核验）。

因此 `ddfe9d2` 提前约 8～12 s 开始观察并加重了暴露，但没有制造缺陷；根因是动态
机动中 `s` 估计系统性越过已知仿真包线
`m_payload,max*r_xy,max = 0.30*0.13 = 0.039 kg·m`。attach+10～25 s 的低动态段
中位约 0.0108～0.023 kg·m，均在包线内。

下一版不能继续用 running max，也不能只按“attach 后等待固定时长”取参考，因为
那会把具体脚本节奏写进估计器。应由 MHE 自身的 health/freshness 和低动态条件识别
有效窗口，用稳健统计建立独立 `s_ref_loaded`，在进入动态前冻结；越界样本不裁剪后
使用，而是从 reference 候选中剔除并计入 estimator-quality 指标。若一直无法建立
可靠 reference，则保持 detector 未就绪并由 unresolved 安全承接。

## attach 有效性的物理真值缺口

`check_run_valid.py` 目前用 `m_est` 的 p90 判定 `attach-fail`。这个名字过强：它只能
说明 MHE 没有持续估到载荷，不能区分“物理没抓上”和“抓上但估计器没识别”。例如
定向轮 `182456` 的 `gviz_proximity` 日志明确包含 `attach condition met` 与
`-> ATTACH` 请求，但当时没有保存磁吸插件 `/gripper/state` 的 `ATTACHED` 回执；
请求发出也不等于 joint 已成功建立。MHE 的 `truth_attached` 在 continuous 档同样
不能充当物理真值，因为该字段沿用的是外部 attach 通知语义。

因此历史轮次只能拆成“attach 请求未发出”“请求已发出但物理结果未知”“MHE 载荷
有/无证据”，不能从 `m_p` 峰值重建物理 attach 失败率。后续每轮必须独立保存
`/gripper/state`：以 drop 前出现 `ATTACHED` 且没有提前 `DETACHED` 作为物理 capture
成功；同时把 MHE 是否进入 `_load_armed`、是否建立 moment reference 作为另一列。
论文表中的旧 `attachment not achieved` 已相应改名为 `payload not evidenced by MHE`。
## Recorded mission illustration (2026-09-11 addition)

`figs/fig_mission.py` generates `fig_mission.pdf` and the separate
`fig_mission_release_detail.pdf` from the frozen logs in
`data/mission_20260911_033026/`. The selected flight is the B arm of the
last complete W5 pair, not a best-tracking-error selection. Its controller log
contains 1903 dense samples; its MHE log contains 100 published first-moment
decay samples. Wall timestamps are aligned to the median controller handoff
epoch inferred from the dense samples. The source manifests and runtime
provenance accompany the logs; SHA-256 values and event times are recorded in
`figs/fig_mission_provenance.json`.

The spatial panel is **reference geometry only** (planar Gerono figure-eight,
radius 10 m), not measured XY, not a reconstruction of the ramp, and not an
actual-vs-reference comparison. Continuous XY was not stored in these logs.
The temporal panels plot recorded height, controller-consumed mass estimate,
and position-error norm from the first to the last dense log sample; they do
not include the earlier takeoff/approach segment. Confirmation occurs 4.456 s
after the release command. The separate detail figure shows only the logged
release-decay interval, without fabricating earlier confidence samples.
## Shared equation and figure typography

The manuscript preserves IEEEtran conference column dimensions and equation
numbering. Display equations use aligned relations and semantic line breaks;
the state-vector transpose is upright and payload-frame continuation lines
remain inside the column. No display is scaled down as an image.

`figs/paper_style.py` supplies the common Times-compatible font, STIX math,
blue/orange/green palette, axis styling, and embedded TrueType PDF fonts for
`figs/fig_gen.py` and `figs/fig_mission.py`. This palette is an editorial choice,
not a conference requirement. Regenerate both scripts from `paper/`, then run
`latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex`.
