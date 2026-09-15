# PX4-nmpc

基于 ROS 2 + PX4 SITL 的四旋翼 **NMPC(非线性模型预测控制)** 实验工作区，包含 acados 控制器、在线 **MHE(移动窗口估计)** 模块，以及磁吸夹爪 / 吊挂负载的 Gazebo 仿真扩展。

本仓库是 ROS 2 工作区的 `src/` 目录，包含一个 ament_python 功能包和一组 SITL 启动脚本。

---

## 目录结构

```
src/
├── offboard_test_acados/     # acados NMPC + MHE + 夹爪/吊挂扩展
│   ├── config/gripper/, urdf/gripper/, worlds/gripper/
│   ├── gz_plugins/magnetic_gripper/
│   └── offboard_test_acados/gripper/          # 夹爪专属节点子包
└── scripts/                  # 一键启动的 SITL 脚本
    └── gripper/               # 磁吸夹爪场景的驱动/分析脚本
```

> 旧 `masschanger` 实验栈已退役并加入 `.gitignore`。仍被夹爪分析使用的通用
> 日志解析器和 RViz URDF 已迁入 `scripts/gripper/` 与 `urdf/gripper/`。

---

## 功能包

### `offboard_test_acados` — acados NMPC + MHE + 夹爪/吊挂

该包包含 **acados** NMPC、在线负载估计与负载操作场景。共享机架常数和数学工具集中在 `offboard_test_acados/common.py`，不依赖其他工作区内 Python 包。

| 可执行节点 | 说明 |
|-----------|------|
| `acados_nmpc_node` | acados 版 NMPC 控制器;支持 8 字/直线参考轨迹;可在跟踪一段时间后触发负载质量阶跃(drop) |
| `mhe_node` | **开环** MHE 质量估计(仅诊断,不反馈进控制器)。用电机实际转速反算的物理推力作为已知输入,在滑动窗口内估计机体质量,估计值发布到 `/acados_nmpc/mhe_mass_estimate` |
| `gripper_flight_node` | 磁吸夹爪演示飞行(纯位置 setpoint):预热 → 起飞 → 飞到 box 上方 → 触发吸附 → 带载爬升 |
| `proximity_gripper_node` | 近距离触发磁吸夹爪的 attach/detach |
| `drone_tf_broadcaster` | 发布 `map→base_link` TF,驱动 RViz 中的 URDF 模型 |
| `prop_joint_state_publisher` | 用 PX4 实际指令转速让 RViz 里的螺旋桨转起来 |
| `plot_logger_acados` | acados 版轨迹记录/画图 |

**配置 / 资源**(按场景分了文件夹)
- `config/gripper/gripper_params.yaml`、`config/gripper/gripper_bridge.yaml` — 夹爪参数与 ros_gz 桥接(bridge 现已不需要,保留作参考)
- `config/gripper/nmpc_view_gripper_hifly.rviz` — 夹爪飞行 RViz 视图
- `urdf/gripper/x500.urdf` — RViz 显示用机体模型
- `worlds/gripper/gripper_test.sdf` — 夹爪测试世界

**自定义 Gazebo(gz-sim)系统插件** —— 位于 `gz_plugins/`,需单独 CMake 构建:
- `magnetic_gripper/` — 磁吸夹爪(DetachableJoint)插件

---

## SITL 启动脚本(`scripts/`)

当前支持的夹爪场景入口位于 `gripper/`。

| 脚本 | 用途 |
|------|------|
| `gcs_heartbeat.py` | 无头模式下顶替 QGC 的 pymavlink GCS 心跳 |

**`gripper/`**

| 脚本 | 用途 |
|------|------|
| `run_sitl_gripper.sh` | 磁吸夹爪 attach 演示 SITL(纯位置 setpoint,不接 acados NMPC) |
| `run_gripper_headless.sh` | 同上的无头批量版,吊挂 CEM 学习的批量驱动底座 |
| `run_sitl_gripper_viz.sh` | 当前主配置的 Gazebo + RViz GUI 入口 |
| `run_cxy_ecc_sweep.sh` | 横向偏心批量扫描 |

> 各脚本头部有详细的设计说明与注意事项(如 Gazebo headless、负载质量阶跃的实现取舍等),运行前建议先阅读。

### 实验版本溯源

所有当前实验入口都会在 `nmpc_test_results/provenance_<时间戳>/` 调用统一的
`scripts/record_experiment_provenance.sh`。记录包含工作区、PX4 和 acados 三个
运行时代码仓的完整 HEAD、分支、commit 时间、dirty 清单、tracked binary patch、
未跟踪文件清单/哈希/归档、submodule 状态，以及经过白名单过滤的实验环境变量。

每个目录的 `VERDICT.txt` 是引用口径的唯一判据：只有三个代码仓都存在且 clean 时，
`exact_commit_claim=ALLOWED`；任何仓库缺失或 dirty 都是 `PROHIBITED`。后一种情况可用
HEAD + patch + untracked archive 重建当时的运行时源码，但论文仍不得声称实验“由某个
exact commit 运行”。`paper/data/**` 是不参与仿真执行的冻结论文数据，为避免每轮重复
约 48 MiB，明确排除在未跟踪源码归档之外。

---

## 依赖

- ROS 2(rclpy)、Gazebo(gz-sim)、`ros_gz_bridge`
- PX4-Autopilot(SITL)+ MAVROS(`mavros_msgs`)+ QGroundControl
- 消息:`geometry_msgs`、`nav_msgs`、`std_msgs`、`sensor_msgs`、`actuator_msgs`、`tf2_ros`
- 求解器：[acados](https://github.com/acados/acados) 及其 Python 接口
- `robot_state_publisher`

---

## 快速开始

```bash
# 假设本仓库位于 ~/ros2_ws_HJH/src
cd ~/ros2_ws_HJH

# 构建功能包
colcon build --packages-select offboard_test_acados
source install/setup.bash

# 另需单独构建 gz 自定义插件(见 gz_plugins/*/ 内的 build/)
# 然后运行对应的 SITL 脚本,例如:
./src/scripts/gripper/run_sitl_gripper_viz.sh
```

---

## 说明

- `mhe_node` 仅做诊断,**不会**修改发给 PX4 的任何指令。
- `.gitignore` 忽略了 `nmpc_acados_px4`(本地生成的 acados 代码/产物)。

## License

TODO(各 `package.xml` 中 license 字段待补充)。
