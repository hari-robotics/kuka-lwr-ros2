# ultrasound_probe_impedance ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/ultrasound_probe_impedance`（实验室教程 "Program mode → force control"：`probe_incremental_force.launch` / `probe_incremental_impedance.launch`、`enable_force_control` 服务、`/probe_action_force`、`/probe_action_impedance`）。非 KUKA 专用包，放在 `src/ultrasound_probe_impedance`。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml, CMakeLists.txt | 同名 | catkin → ament_cmake |
| src/probe_incremental_force_node.cpp | 同名 | 只换 ROS 接口 |
| src/probe_incremental_impedance_node.cpp | 同名 | 只换 ROS 接口 |
| launch/probe_incremental_force.launch, probe_incremental_impedance.launch | launch/*.launch.py | 见 launch 对照 |
| README.md, STEP_DURATION_CONFIGURATION.md | 同名 | 原样复制（内容仍以 ROS1 命令描述） |
| include/ultrasound_probe_impedance/、test/（均为空目录）、.vscode/ | — | 不迁移 |

## 主要改动
- **节点**：公共 `nh` 与私有 `pnh{"~"}` 合并为一个 `rclcpp::Node`（节点名同 ROS1）；私有参数 → 节点参数，名称、类型、默认值不变。
- **服务**：私有服务 `~enable_force_control` → `~/enable_force_control`，全名仍为 `/probe_incremental_force_node/enable_force_control`、`/probe_incremental_impedance_node/enable_force_control`（`std_srvs/srv/SetBool`），处理函数原样。
- **话题**：订阅 `pose_topic`（深度 10）、`action_topic`（1）、`recovery_action_topic`（1，仅 force 节点）、`force_topic`（10）；发布 `command_topic`（`lwr_controllers/msg/CartesianImpedancePoint`，10）。消息字段 `x_FRI/k_FRI/d_FRI/f_FRI` → `x_fri/k_fri/d_fri/f_fri`（lwr_controllers 的 ROS2 接口，与 ultrasound_trajectory 的 ee_incremental_impedance_node 相同）。
- **时间与日志**：`ros::Time::now()` → `node->now()`（ROS 时间），`ROS_*_THROTTLE(t, ...)` → `RCLCPP_*_THROTTLE(..., t*1000 ms, ...)`。
- **构建**：C++17（ROS1 `-std=c++11`）。

## launch 对照
两个 launch 各启动一个节点，可执行名、节点名、output=screen、所有 arg 名称与默认值与 ROS1 相同；参数值经静态比对一致（含 `probe_length:=0.3` 覆盖分支）。标定参数从 `ultrasound_probe_motion/config/probe_calibration.yaml` 读取，按 arg 是否为空决定是否覆盖、数值/字符串参数的类型处理与 ultrasound_probe_motion 相同（见该包 NOTES 的 launch 对照）。

## 硬件通信部分
无（力传感器数据来自 `/Force_calibrated` 话题，由 sriforcesensor 发布）。

## 未迁移的原测试
无（ROS1 `test/` 为空目录）。

## 需人工确认
1. **实机未验证**：Gazebo 不模拟 KUKA 的笛卡尔阻抗，仿真测试只验证了节点的 ROS 接口与计算（`migration_tests/tutorial_packages` 第 [4] 部分：命令流、k/d 初值、启用服务的拒绝/接受、力参考按 `force_kp·(target−|F|)` 积分、反馈中断后自动关闭、探头坐标系中的动作目标）。力控闭环需在实机上确认。
2. **仿真时间**：与 ultrasound_probe_motion 相同，launch 未声明 `use_sim_time`，节点使用系统时间；实机无差别。
3. **代码默认值不同**：`probe_z_rotation_deg` 代码默认值在 force 节点为 0.0、impedance 节点为 5.0；通过 launch 启动时都取标定文件的 −7.5（ROS1 相同）。直接 `ros2 run` 而不加载标定文件时使用代码默认值（与 `rosrun` 相同）。

## 编译、运行与测试
```bash
colcon build --packages-select ultrasound_probe_motion ultrasound_probe_impedance
ros2 launch ultrasound_probe_impedance probe_incremental_force.launch.py
ros2 service call /probe_incremental_force_node/enable_force_control std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /probe_action_force geometry_msgs/msg/Twist "{linear: {x: 0.01}}"
```
测试见 `../../migration_tests/ultrasound_probe_impedance/README.md` 与 `../../migration_tests/tutorial_packages/README.md`。
