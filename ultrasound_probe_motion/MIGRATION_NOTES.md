# ultrasound_probe_motion ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/ultrasound_probe_motion`。教程"force control"使用的 `ultrasound_probe_impedance` 的 launch 从本包读取探头标定文件（`config/probe_calibration.yaml`），且 package.xml 声明了对本包的运行依赖，因此一并移植。非 KUKA 专用包，放在 `src/ultrasound_probe_motion`。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml, CMakeLists.txt | 同名 | catkin → ament_cmake + rosidl |
| action/ProbeStep.action | 同名 | 逐字节相同；`add_action_files` → `rosidl_generate_interfaces` |
| include/ultrasound_probe_motion/quintic_duration.h, trajectory_progress_guard.h | 同名 | 逐字节相同（不含 ROS） |
| src/quintic_duration.cpp | 同名 | 逐字节相同（不含 ROS） |
| src/probe_relative_motion_node.cpp | 同名 | 只换 ROS 接口；actionlib 见"主要改动" |
| src/probe_scan_motion_node.cpp | 同名 | 只换 ROS 接口；actionlib、定时器见"主要改动" |
| config/probe_calibration.yaml | 同名 | 数值不变；改为 ROS2 参数文件格式（`/**: ros__parameters:`） |
| launch/probe_motion.launch, probe_scan_motion.launch | launch/*.launch.py | 见 launch 对照 |
| README.md, STEP_DURATION_CONFIGURATION.md, PROBE_SCAN_MOTION.md | 同名 | 原样复制（内容仍以 ROS1 命令描述） |
| test/test_quintic_duration.cpp, test/test_trajectory_progress_guard.cpp | — | 见"未迁移的原测试" |

## 主要改动
- **节点**：ROS1 的公共 `nh_` 与私有 `pnh_("~")` 合并为一个 `rclcpp::Node`（节点名同 ROS1 `ros::init`）；私有参数 → 节点参数，名称和默认值不变（`pnh_.param<T>(name, v, def)` → `declare_parameter<T>(name, def)`）。话题名都是绝对名或参数给定，未变。
- **actionlib → rclcpp_action**（`/probe_step`，`ultrasound_probe_motion/action/ProbeStep`）：
  - ROS1 的 goal 回调中先校验（拒绝），再 `setAccepted()` 并开始执行；ROS2 拆为 `handle_goal`（同样的校验，返回 REJECT / ACCEPT_AND_EXECUTE）和 `handle_accepted`（其余代码原样）。两者在同一次 executor 调用中依次执行，中间不会插入位姿回调，状态与 ROS1 相同。probe_scan_motion_node 中校验时算出的局部量（`guarded`、`trajectory_complete`、`local_z_profile`、`fixed_duration_profile`、`start_ee`）在 `handle_accepted` 中用同样的表达式重新计算。
  - `setSucceeded/setAborted(result)` → `succeed/abort(result)`；`publishFeedback` → `publish_feedback`。
  - 取消：ROS1 在取消回调中立即 `setCanceled`；ROS2 必须等 `handle_cancel` 返回、目标进入 CANCELING 后才能报告，因此状态清理（`executing_/settling_/has_step_goal_` 等）仍在取消回调中立即完成，`canceled(result)` 由一个 0 周期的一次性定时器在下一次 executor 循环中发出。
- **定时器**（scan 节点 `control_rate_hz`）：`nh_.createTimer`（ROS 时间）→ `rclcpp::create_timer(node, node->get_clock(), ...)`（同样跟随 ROS 时间）。
- **latched 话题**：scan 节点的 `~cruise_stopped`（`pnh_.advertise(..., latch=true)`）→ `~/cruise_stopped`，QoS depth 1 + transient_local；参数给绝对名时保持绝对名。
- **时间**：`ros::Time::now()` → `node->now()`，零时间 `ros::Time()` → `rclcpp::Time(0, 0, RCL_ROS_TIME)`，`isZero()` → `nanoseconds() == 0`。
- **构建**：C++17；ROS1 库目标名 `${PROJECT_NAME}`（libultrasound_probe_motion）在 ROS2 中被生成的接口目标占用，库改名为 `ultrasound_probe_motion_lib`（只被本包两个节点使用）。

## launch 对照
两个 launch 各启动一个节点（`/probe_relative_motion_node`、`/probe_scan_motion_node`），可执行名、节点名、output=screen 与 ROS1 相同；所有 arg 名称和默认值相同；静态比对全部一致（`migration_tests/ultrasound_probe_motion`）。

| 项 | ROS1 | ROS2 | 说明 |
|---|---|---|---|
| 标定文件 | `<rosparam file=".../probe_calibration.yaml"/>` | `parameters=[<yaml 路径>, {...}]`，yaml 在前 | 覆盖顺序相同 |
| `probe_length` 等 3 个标定参数 | `<param ... if="$(eval arg(x) != '')"/>`：arg 为空时不设置，用 yaml 的值 | `PythonExpression("'<arg>' or <yaml 值>")`：arg 为空时取 launch 读入的 yaml 值 | 最终参数值相同（ROS2 launch 不能按条件省略单个参数）；arg 非空时的分支也已比对 |
| 数值参数 | 由字符串推断类型（`3.0` → double） | `ParameterValue(..., value_type=float)` | 节点按 double 声明；传入 `3` 这类整数字面量时 ROS1 的 `param<double>` 也能读取，ROS2 若不强制 float 会因类型不符报错 |
| 字符串参数 | 字符串 | `ParameterValue(..., value_type=str)` | 允许 `recovery_action_topic:=""`（关闭恢复通道），与 ROS1 相同 |

## 硬件通信部分
无。

## 未迁移的原测试
`test/test_quintic_duration.cpp`、`test/test_trajectory_progress_guard.cpp`（catkin gtest，测试 ROS 无关的库代码；库代码逐字节相同）。未转换到 `migration_tests/`；本包的运行时行为由 `migration_tests/tutorial_packages/test_tutorial_packages.py` 第 [3] 部分覆盖。

## 需人工确认
1. **拒绝原因不再传给客户端**：ROS1 `setRejected(result, reason)` 会把 `result.reason`（`busy`、`pose_not_ready`、`invalid_goal`、`target_exceeds_step_limit`）发给客户端；rclcpp_action 的拒绝不带 result，现改为节点日志 `Step goal rejected: <reason>`。依赖拒绝原因的客户端（如 GPIS 编排节点，不在本工作空间）需要改为从日志或另行判断。
2. **取消完成的时刻**：`CANCELED` 结果在取消请求后的下一个 executor 循环发出（ROS1 为取消回调内）。这段时间内节点状态已空闲，可以接受新目标（仿真实测：取消后新目标被接受）。
3. **仿真时间**：ROS1 在仿真中通过全局 `/use_sim_time` 自动使用仿真时间；ROS2 launch 未声明 `use_sim_time`，节点使用系统时间（步长计时、定时器、结算超时）。实机无差别；仿真实时因子明显低于 1 时步长会按墙钟完成。需要时可用 `ros2 run ... --ros-args -p use_sim_time:=true --params-file ...` 启动。

## 编译、运行与测试
```bash
colcon build --packages-select ultrasound_probe_motion
ros2 launch ultrasound_probe_motion probe_motion.launch.py
ros2 topic pub --once /probe_action geometry_msgs/msg/Twist "{linear: {z: 0.01}}"
ros2 action send_goal /probe_step ultrasound_probe_motion/action/ProbeStep "{...}"
```
测试见 `../../migration_tests/ultrasound_probe_motion/README.md` 与 `../../migration_tests/tutorial_packages/README.md`。
