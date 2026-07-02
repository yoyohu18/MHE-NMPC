# PX4-nmpc

基于 ROS 2 + PX4 SITL 的四旋翼 **NMPC(非线性模型预测控制)** 实验工作区,包含两套并行的控制器实现、一个在线 **MHE(移动窗口质量估计)** 诊断模块,以及一套磁吸夹爪 / 吊挂负载的 Gazebo 仿真扩展。

本仓库是某 ROS 2 工作区的 `src/` 目录,包含两个 ament_python 功能包和一组 SITL 启动脚本。

---

## 目录结构

```
src/
├── offboard_test/            # 基线:CasADi/IPOPT 版 NMPC + 位置 setpoint 节点
├── offboard_test_acados/     # acados 版 NMPC 移植 + MHE + 夹爪/吊挂扩展
└── scripts/                  # 一键启动的 SITL 脚本
```

---

## 功能包

### 1. `offboard_test` — 基线控制器(CasADi / IPOPT)

原始的参考实现,便于与 acados 版本逐项对比。

| 可执行节点 | 说明 |
|-----------|------|
| `offboard_node` | 最简 MAVROS offboard 演示,位置 setpoint 画圆/悬停,用于打通 PX4 offboard 链路 |
| `nmpc_node` | CasADi + IPOPT 求解的 NMPC 姿态/推力控制器(状态机:预热 → 等 EKF2 → 切 OFFBOARD → 解锁 → 飞到起点 → NMPC 接管) |
| `plot_logger` | 记录参考/实际/预测轨迹,离线画图 |
| `metrics_collector` | 采集跟踪误差等指标 |

物理参数(`nmpc_node.py` 内 `Params`):x500 机型 `m≈2.06kg`,`J=diag(0.0142,0.0142,0.0210)`,预测步长 `dt=0.1s`、`N=10`(horizon 0.4s)。

### 2. `offboard_test_acados` — acados 版 NMPC + MHE + 夹爪/吊挂

将 `offboard_test/nmpc_node` 的 NMPC 内核换成 **acados** 求解器,状态机、坐标系、限幅、推力归一化、body-rate 斜坡等外围行为与 CasADi 版完全一致,便于直接对比两种求解器。在此基础上新增了在线质量估计与负载操作场景。

| 可执行节点 | 说明 |
|-----------|------|
| `acados_nmpc_node` | acados 版 NMPC 控制器;支持 8 字/直线参考轨迹;可在跟踪一段时间后触发负载质量阶跃(drop) |
| `mhe_node` | **开环** MHE 质量估计(仅诊断,不反馈进控制器)。用电机实际转速反算的物理推力作为已知输入,在滑动窗口内估计机体质量,估计值发布到 `/acados_nmpc/mhe_mass_estimate` |
| `gripper_flight_node` | 磁吸夹爪演示飞行(纯位置 setpoint):预热 → 起飞 → 飞到 box 上方 → 触发吸附 → 带载爬升 |
| `proximity_gripper_node` | 近距离触发磁吸夹爪的 attach/detach |
| `drone_tf_broadcaster` | 发布 `map→base_link` TF,驱动 RViz 中的 URDF 模型 |
| `prop_joint_state_publisher` | 用 PX4 实际指令转速让 RViz 里的螺旋桨转起来 |
| `plot_logger_acados` | acados 版轨迹记录/画图 |

**配置 / 资源**
- `config/gripper_params.yaml`、`config/gripper_bridge.yaml` — 夹爪参数与 ros_gz 桥接
- `config/nmpc_view_acados.rviz` — RViz 视图
- `urdf/x500.urdf` — RViz 显示用机体模型
- `worlds/gripper_test.sdf` — 夹爪测试世界

**自定义 Gazebo(gz-sim)系统插件** —— 位于 `gz_plugins/`,需单独 CMake 构建:
- `mass_changer/` — 通过 ECS 直接改写 `base_link` 的 Inertial 组件实现负载质量阶跃(LOADED 2.5kg ↔ EMPTY 2.0kg),不引入任何额外刚体,专为验证 MHE 能否收敛于在线质量变化而设计
- `magnetic_gripper/` — 磁吸夹爪(DetachableJoint)插件

---

## SITL 启动脚本(`scripts/`)

| 脚本 | 用途 |
|------|------|
| `run_sitl_nmpc.sh` | 基线:PX4 SITL + MAVROS + NMPC(CasADi 版) |
| `run_sitl_acados.sh` | acados 全栈:PX4 SITL(gz_x500_payload,含 mass_changer 插件)+ MAVROS + QGC + RViz + `acados_nmpc_node` + `plot_logger_acados` + `mhe_node` + TF/桥接 |
| `run_sitl_gripper.sh` | 磁吸夹爪场景 SITL |
| `run_sitl_gripper_acados.sh` | acados 版夹爪场景 SITL |

> 各脚本头部有详细的设计说明与注意事项(如 Gazebo headless、负载质量阶跃的实现取舍等),运行前建议先阅读。

---

## 依赖

- ROS 2(rclpy)、Gazebo(gz-sim)、`ros_gz_bridge`
- PX4-Autopilot(SITL)+ MAVROS(`mavros_msgs`)+ QGroundControl
- 消息:`geometry_msgs`、`nav_msgs`、`std_msgs`、`sensor_msgs`、`actuator_msgs`、`tf2_ros`
- 求解器:CasADi + IPOPT(`offboard_test`);[acados](https://github.com/acados/acados) 及其 Python 接口(`offboard_test_acados`)
- `robot_state_publisher`

---

## 快速开始

```bash
# 假设本仓库位于 ~/ros2_ws_HJH/src
cd ~/ros2_ws_HJH

# 构建两个功能包
colcon build --packages-select offboard_test offboard_test_acados
source install/setup.bash

# 另需单独构建 gz 自定义插件(见 gz_plugins/*/ 内的 build/)
# 然后运行对应的 SITL 脚本,例如:
./src/scripts/run_sitl_acados.sh
```

---

## 说明

- `offboard_test` 与 `offboard_test_acados` 刻意保持独立,以便在不动已验证的 CasADi/IPOPT 流水线的前提下试验 acados。
- `mhe_node` 仅做诊断,**不会**修改发给 PX4 的任何指令。
- `.gitignore` 忽略了 `nmpc_acados_px4`(本地生成的 acados 代码/产物)。

## License

TODO(各 `package.xml` 中 license 字段待补充)。
