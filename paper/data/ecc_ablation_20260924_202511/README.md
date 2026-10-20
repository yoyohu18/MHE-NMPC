# 偏心控制补偿消融冻结数据

2026-09-24，4 对，8 次；结论见 [完整报告](batch/report.md)。

原始日志、每轮代码与配置溯源、启动日志、清单、指标和图均随本目录冻结。源码含既有未提交修改，不能声称对应干净 commit。

从本目录复算（会重新生成统计报告，末尾人工结论不由脚本生成）：

```sh
sha256sum -c SHA256SUMS
python3 scripts/aggregate_ecc_ablation.py batch
MPLCONFIGDIR=/tmp/ecc_mpl python3 scripts/plot_ecc_ablation.py batch
```

完整批次启动：`python3 src/scripts/gripper/run_ecc_ablation.py`（在工作区根目录；会启动并重置共享 PX4/Gazebo 仿真栈）。仅用于没有其他仿真实验运行时。
