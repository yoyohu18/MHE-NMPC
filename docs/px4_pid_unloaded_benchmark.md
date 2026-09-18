# 无载荷 PX4 原生级联控制基线

入口：`offboard_test_acados/px4_pid_benchmark.py`。不使用 NMPC 控制输出，向
`/mavros/setpoint_raw/local` 发送位置、速度前馈和 yaw，由 PX4 位置/速度/姿态/角速度环控制。
这不是单独的角速度 PID 对比。加速度和 yaw rate 均屏蔽，可用
`velocity_feedforward:=false` 切换为纯位置 setpoint。

复用 NMPC 的 `build_reference_figure8`，默认 r=1m、w=0.3rad/s、z=3m、dz=0.5m，
hover=2s、ramp=4s、yaw_ramp=false。xy 平移至节点启动时机体位置，z 是 MAVROS
本地 ENU 绝对高度。与 NMPC 比较时应统一原点、参考计时、yaw、PX4 参数和约束。
参考生成器沿用现有 numpy/casadi 依赖，但本节点不实例化 acados solver。

## 后续运行方法（本次未执行）

在工作区根目录构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select offboard_test_acados --symlink-install
source install/setup.bash
ros2 run offboard_test_acados px4_pid_benchmark --ros-args --params-file \
  src/offboard_test_acados/config/benchmark/px4_pid_unloaded.yaml
```

应使用无载荷机体，并确保没有 NMPC、gripper_flight 或其他节点同时发送控制 setpoint。
节点不修改机体载荷、PX4 参数，不调用解锁、模式切换或夹爪服务。
接收到新鲜的 MAVROS state/pose 后，在当前位置预发送至少 2s setpoint；操作者
切换 OFFBOARD 并解锁后，用 8s 五次曲线爬升至目标高度。连续进入 0.2m 容差 2s
后进入 TRACK，参考时间从零开始。TRACK 60s（含参考自身 hover/ramp）后用 4s
余弦相位减速，再持续 HOLD 在终点；降落由操作者执行。

如果退出 OFFBOARD、失去解锁状态、遥测超时或活动阶段 ROS 时间跳变/回调间隔
超过阈值，锁定 ABORT 并停止 setpoint，后续由 PX4 配置的退出/失联策略处理。
不会自动重新接管，需检查后重启节点。退出程序同样停止 setpoint，不自动降落。

观测话题：`/px4_pid_benchmark/reference`（PoseStamped）、
`/px4_pid_benchmark/phase`（String）、`/mavros/local_position/pose`。
TRACK 为主要误差统计区间，起飞与终点减速应分别统计；本版不自动记录 CSV 或计算指标。
没有仿真/实飞验证，离线测试不代表已验证闭环跟踪性能。

## 离线检查

不创建 ROS 节点、不连接 PX4：

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="src/offboard_test_acados:$PYTHONPATH" python3 -m pytest -q \
  src/offboard_test_acados/test/test_px4_pid_benchmark.py
```

接口依据：[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)、
[MAVROS raw setpoint 实现](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_raw.cpp)。
ROS 数据使用 ENU；`FRAME_LOCAL_NED` 标识 MAVLink 输出帧，ENU→NED 转换由 MAVROS 负责。
