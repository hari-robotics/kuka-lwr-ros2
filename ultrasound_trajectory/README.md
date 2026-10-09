# ultrasound_trajectory — ROS 2 Humble

包已移植到 `src/ultrasound_trajectory`。根目录同名目录保留原始 ROS 1
源码和未参与原 CMake 构建的备份版本，并用 `COLCON_IGNORE` 排除重复包。
ROS 2 版本包含原 CMake 的全部 9 个可执行程序及工具相对运动辅助代码。

```bash
cd /home/hari/kuka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ultrasound_trajectory --symlink-install
source install/setup.bash
```

在全新工作空间中使用 `--packages-up-to ultrasound_trajectory` 构建依赖。

| 可执行程序 | 输入/用途 | 输出 |
| --- | --- | --- |
| `ultrasound_traj_node` | 预设多阶段超声扫描轨迹 | 笛卡尔阻抗命令 |
| `linear_traj_node` | 预设线性轨迹 | 笛卡尔阻抗命令 |
| `linear_traj_node_Luca` | 线性轨迹及 `/Force` 力反馈（Twist 类型） | 笛卡尔阻抗命令 |
| `pose_initialization` | 预设笛卡尔初始位姿 | 笛卡尔阻抗命令 |
| `ultrasound_traj_cartesian_node` | `/ee_cartesian_target`：绝对 PoseRPY 目标 | IK 位姿命令 |
| `pose_init` | 关节初始化并等待控制器执行结果 | FollowJointTrajectory action |
| `ee_incremental_motion` | `/ee_action`：工具坐标系中的 Twist 增量 | IK 位姿命令 |
| `ee_incremental_impedance_node` | `/ee_action_impedance` 增量及可选力环 | 笛卡尔阻抗命令 |
| `tool_relative_motion_node` | 工具相对运动、IK 限制检查和状态机 | IK 位姿命令/状态诊断 |

笛卡尔位姿输入默认为 `/cartesian_position/pose/end_effector`，可由已移植的
`cartesian_position_node` 提供。IK 命令话题为
`/lwr/one_task_inverse_kinematics/command`（`lwr_controllers/msg/PoseRPY`，完整位姿 id=0）。
阻抗命令话题为 `/lwr/cartesian_impedance_controller/command`
（`lwr_controllers/msg/CartesianImpedancePoint`）；这些节点需要对应的
`cartesian_impedance_controller` 来消费命令。
关节初始化使用 `/lwr/joint_trajectory_controller/follow_joint_trajectory` action，
等待控制器接受并返回执行结果；失败时报告原因并返回非零退出码。
`action_name` 可指定其他轨迹控制器，`joint_positions`（七个弧度值）和
`duration`（默认 5 秒）设置目标。旧 `command_topic` 参数已由 `action_name` 替代。
`server_timeout`（默认 15 秒，墙钟）限制服务发现/接受响应等待；
`execution_timeout`（默认 `duration + 30` 秒，节点 ROS 时钟）限制执行等待，超时请求取消。
成功表示控制器判定到位；节点明确设置七个关节的位置容差
`position_tolerance`（默认 0.01 rad），避免控制器默认禁用位置容差时提前报告成功。
命令位姿使用当前控制器的法兰坐标 `lwr_7_link`，与模型的 `tcp` 坐标有固定偏移。

先启动已有机器人，再单独启动一个轨迹节点。例如：

```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py
```

`pose_init` 需要激活关节轨迹控制器。Gazebo 中使用：

```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=joint_trajectory_controller
# 另一个已加载工作空间环境的终端：
ros2 run ultrasound_trajectory pose_init --ros-args -p use_sim_time:=true
```

默认启动工具相对运动节点，保留原源码的 `monitor_only=true`，只计算而不发布命令：

```bash
ros2 launch ultrasound_trajectory ultrasound_trajectory.launch.py use_sim_time:=true
```

选择其他程序，例如等待增量动作的节点：

```bash
ros2 launch ultrasound_trajectory ultrasound_trajectory.launch.py \
trajectory_node:=ee_incremental_motion use_sim_time:=true
```

`monitor_only` 参数仅适用于 `tool_relative_motion_node`。
预设扫描/初始化节点在收到有效位姿后自动开始自己的轨迹。
同一控制器的命令话题由一个轨迹节点负责；主机器人启动文件不会自动启动这些轨迹程序。

工具相对运动输入为 `/tool_relative_motion_node/move`（Twist），取消为
`/tool_relative_motion_node/cancel`（Empty），故障复位为
`/tool_relative_motion_node/fault_reset`（Empty），诊断为
`/tool_relative_motion_node/status`（String）。它从 `/robot_description`
读取模型，关节反馈默认 `/lwr/joint_states`，也支持启动参数 `robot_description`
直接提供 URDF 字符串。原 IK 限制、反馈超时和监测模式均保留。

启动参数可用 ROS 2 YAML 文件配置：

```bash
ros2 launch ultrasound_trajectory ultrasound_trajectory.launch.py \
trajectory_node:=ee_incremental_impedance_node params_file:=/absolute/path/params.yaml
```

```yaml
/**:
  ros__parameters:
    step_duration: 6.0
    target_force_z: 0.0
    force_feedback_timeout: 0.5
```

配置参数在启动后只读；可用 `ros2 run ... --ros-args -p 参数:=值` 设置，
`use_sim_time` 仍遵循 ROS 2 的标准行为。力环默认关闭，启用服务为
`/ee_incremental_impedance_node/enable_force_control`（`std_srvs/srv/SetBool`）。

迁移中的修复：

- 使用原生 `rclcpp`、ROS 2 消息/服务、`ament_cmake` 和 C++17；
  `CartesianImpedancePoint` 字段改为当前接口的 `x_fri/k_fri/d_fri/f_fri`。
- 用 ROS 时钟计时与定时器支持 Gazebo 的 `use_sim_time`；统一时间类型，
  避免混用系统时间和 ROS 时间。旧扫描节点按 100 Hz 推进，位姿回调只更新反馈，
  不再睡眠；位姿超过 `feedback_timeout`（默认 0.5 s）时暂停发布/推进。
- 关节初始化使用 action 的接受响应和执行结果，避免单次话题发布丢失后仍报告完成。
- 修复 Luca 版本的 `i_sample=50` 赋值及未填满缓冲区时的越界访问，初始化旧标量成员。
- 移除 `ultrasound_traj_node` 中与主扫描阶段同时执行、会提前 shutdown 的第二段实验正弦轨迹；
  主多阶段扫描保留，ROS 1 原始代码仍在根目录参考包中。
- 拒绝非有限位姿/增量，保留力反馈超时检查；模型通过 ROS 2 参数或话题读取，
  不再依赖 ROS 1 全局参数服务器。未使用的 OpenCV 3、message_filters 和旧插件导出已移除。

数学和节点测试使用合成反馈与隔离 ROS 域，不连接真实机器人：

```bash
colcon test --packages-select ultrasound_trajectory
colcon test-result --test-result-base build/ultrasound_trajectory --verbose
```

更详细的接口说明见 [工具相对运动](doc/tool_relative_motion_node.md) 和
[增量阻抗节点](doc/ee_incremental_impedance_node.md)。
ROS 2 模型解析使用官方 [kdl_parser](https://docs.ros.org/en/humble/p/kdl_parser/generated/file_include_kdl_parser_kdl_parser.hpp.html)
的字符串接口。

本次验证：全部 9 个程序编译安装成功，4 项数学测试和 19 项 ROS 2 节点
测试通过（colcon 另统计 2 项测试入口，共显示 25 项）。Gazebo Fortress
无界面联动中，工具相对运动节点成功加载当前 7 关节模型并接收关节反馈，
诊断为 `state=IDLE fault=none joint_age=0.01`，监测模式保持开启。
尚未在真实机器人上执行这些轨迹。
