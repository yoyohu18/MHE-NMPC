# Table release replay 的 73 轮 cohort:冻结与溯源(2026-09-10 恢复)

## 结论

cohort **已恢复并冻结**,不需要删表。

冻结件:`release_replay_20260905.csv` —— 73 行有序清单,每行含 stamp、MHE/NMPC
日志名、validity、两个文件的 SHA-256。一键复现:

```bash
cd /home/clear/ros2_ws_HJH/src
python3 scripts/gripper/replay_release_detector.py \
    --manifest paper/manifests/release_replay_20260905.csv --loocv
```

## cohort 的定义

> 按 **mtime 升序**排列 `nmpc_test_results/*mhe_*.log`,取截至审计提交
> `078b054`(committer time **2026-09-05 14:37:33 +0800**)的**最后 73 个可回放轮次**。

实际覆盖 `20260904_032225` … `20260905_000352`(09-04 03:22 → 09-05 00:03)。
"可回放"= `replay_release_detector.load_round()` 返回非 None(有配对 `grip_nmpc_*.log`、
能从 `[s-collapse]` 或 `[truth]` 行解析出 ≥15 帧)。

## 记账纠正:不是"无界 glob"

`REPRODUCE.md` 原先记成"无界 glob 得到 73 轮"。**这条记账是错的**:同一截止时刻下
无界 glob 给出的是 **249 轮**(229 有 drop、带载 8436 帧、3 轮带载段误释放、
对抗 ratio 621),与表里每一个数字都不符。73 轮是**探索模式 `--limit` 后缀**的自然
结果 —— 也正因为如此它才可以被重建:后缀由"截止时刻 + 轮数"唯一确定,而 09-05 之后
从未删除过 `grip_mhe_*` / `grip_nmpc_*` 历史日志。

## 七项独立指纹核验

用审计当时的判决代码(`078b054` 的 `payload_estimate.release_decision`)在重建
cohort 上复算,与 `078b054` 当时写下的 `docs/mjc_lifecycle_viz.md` §离线验证
和论文表逐位比对:

| 指纹 | 审计当时记录 | 重建 cohort 复算 |
|---|---|---|
| 可回放轮数 | 73 | **73** ✓ |
| 有 drop / 无 drop | 72 / 1 | **72 / 1** ✓ |
| 检出 / 漏检 / 带载段误释放 | 58 / 14 / 0 | **58 / 14 / 0** ✓ |
| 带载帧合计(零误释放) | 2614 | **2614** ✓ |
| 已检出轮延迟中位 | 2.1 s | **2.1 s** ✓ |
| 对抗:最小 peak × 最大残噪 | 0.0134 × 0.0203 | **0.0134 / 0.0203** ✓ |
| 对抗最坏 ratio | 1.52 | **1.520** ✓ |
| `check_run_valid` 判 valid 且有 drop | 59 | **59** ✓ |
| LOOCV 折数与选中阈值 | 59/59 选 0.30 | **59/59 选 0.30** ✓ |
| LOOCV 留出轮成功率 | 90 % | **53/59 = 90 %** ✓ |

无界 glob 的 249 轮在上表**十项里零项命中**,所以这不是"凑出来的 73",而是
被指纹唯一确定的那一个队列。

## validity 列的来源

来自预注册脚本 `scripts/gripper/check_run_valid.py`(2026-09-04 预注册,自
`6419cbe` 起**未改动一行**,`git diff 078b054 HEAD` 为空),判据是 pre-drop 发散
与 attach 失败两类。不是事后人工挑选。59 valid / 14 invalid。

## 顺带查出的表内缺陷(已改)

论文表原写 `valid-flight subset & 48/59 detected; 11/59 missed`。复算表明
**48/11 是 $\rho=0.20$ 下的数字**,审计阈值 $\rho=0.30$ 下 valid 子集是
**53/59 检出、6 漏**。`docs/mjc_lifecycle_viz.md` 当时同时写了
"0.20 只有 48/59" 与"剔除 invalid 后 11/59",两个阈值在转抄进论文时串了行。
已改表与正文。

## 两处口径说明(不影响数字复现)

1. **LOOCV 的 tie-break**:`078b054` 当时同分取**更大**阈值,会报 `0.35`;
   现行脚本同分取**更小**阈值(更保守),报 `0.30`。当时文档记的是 `0.30`,说明
   报告值对应修正后的 tie-break。重建用现行脚本即复现 `59/59 选 0.30`。
2. **LOOCV 的折集**:59 折 = **valid 子集**,不是全部 72 轮(全 72 轮为 72 折、
   留出成功 81 %)。现行 `--manifest` 模式已固定用 valid 子集做 LOOCV。

## 该表能支持什么、不能支持什么

仍按正文的限定:它是 `078b054` 口径 detector 在**异质历史配置**队列上的离线审计,
不是当前 detector 的安全证据 —— 2614 帧零误释放已被 09-05 的在线 fastA smoke
(3/9 误释放)否证。表的作用是给出阈值选择的交叉验证依据与漏检结构。
