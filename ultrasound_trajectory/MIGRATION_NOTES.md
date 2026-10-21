# ultrasound_trajectory ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/ultrasound_trajectory`。前次 ROS2 版本对多数节点做了重写（如 pose_init 改为 action 客户端、参数改为只读并加校验、QoS 改为 best-effort、robot_description 改由话题获取、新增 launch/README/ros2_support.hpp），已丢弃并按原代码重新移植；其中与原代码只差 ROS API 的辅助文件（motion_state_machine、diagnostics、legacy_pose_publisher、joint_state_mapper）保留。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| src/pose_init.cpp | 同名 | 发布单点 JointTrajectory，等待订阅者、睡 6 s 后退出；只换 ROS API（见需人工确认 1） |
| src/pose_initialization.cpp, ultrasound_traj_node.cpp, linear_traj_node.cpp, linear_traj_node_Luca.cpp | 同名 | 类结构、轨迹/阻抗参数、回调内 `Rate.sleep()` 全部不变；`ros::NodeHandle nh` → 成员 `rclcpp::Node::SharedPtr nh`（节点名取自原 `ros::init`），新增 `node()` 访问器供 main 中 `rclcpp::spin` |
| — | include/ultrasound_trajectory/ros_rate.h | `ros::Rate` 的 ROS2 移植（同一周期逻辑，节点 ROS 时钟），见主要改动 |
| src/ultrasound_traj_cartesian_node.cpp | 同名 | 私有参数 `~T_nom` 等 → 节点参数（同名同默认值）；`ros::Timer` → 节点时钟上的 `rclcpp::create_timer` |
| src/ee_incremental_motion.cpp | 同名 | 只换 ROS API |
| src/ee_incremental_impedance_node.cpp | 同名 | 私有参数 → 节点参数；服务 `~enable_force_control`（std_srvs/SetBool）→ `~/enable_force_control` |
| src/tool_relative_motion_node.cpp + se3_trajectory.cpp, ik_safety_gate.cpp, motion_state_machine.cpp | 同名 | 私有参数（40 个）→ 节点参数；`~move`、`~cancel`、`~fault_reset`、`~status` → `~/…`；两个 `ros::Timer` → 节点时钟定时器；全局 `/robot_description` → 节点参数（见需人工确认 3） |
| src/se3_trajectory.cpp, ik_safety_gate.cpp；include/…/ik_safety_gate.h, se3_trajectory.h, trm_types.h, pseudo_inverse.h, smooth_traj.h；include/utils/pseudo_inversion.h；doc/*.md | 同名 | 逐字节复制（不含 ROS） |
| include/…/motion_state_machine.h, diagnostics.h, legacy_pose_publisher.h, joint_state_mapper.h；src/motion_state_machine.cpp | 同名 | 只换 ROS 类型（`ros::Time` → `rclcpp::Time`（ROS 时钟）、发布者类型）；JointStateMapper 新增 `setClock()`，用节点时钟打时间戳（等价 `ros::Time::now()`） |
| src/linear_traj_node_24_03.cpp, linear_traj_node_backup.cpp, `linear_traj_node (copy).cpp`, src/.vscode | — | ROS1 CMake 未编译的备份/编辑器文件，不迁移 |
| CMakeLists.txt, package.xml | 同名 | catkin → ament_cmake；9 个可执行与 ROS1 同名、同源文件；保留 `-fpermissive`；`find_package(OpenCV 3.3.1)` → `find_package(OpenCV)`（Humble 为 OpenCV 4，源码未调用 OpenCV） |

## 主要改动
- **节点名/话题名/QoS**：与 ROS1 相同（`queue_size n` → depth n，reliable）。私有名 `~x` → `~/x`。
- **参数**：`pnh.param("x", v, default)` → `v = declare_parameter("x", default)`，名称与默认值不变。
- **pose_init 的等待**：`ros::Duration(6.0).sleep()` → 在节点时钟上边 `spin_some` 边等待 6 s。rclcpp 时钟只有在节点处理 /clock 时才前进（roscpp 在内部线程更新），且照搬 ROS1 `Duration::sleep()` 的规则：起始时间为 0（节点尚未收到 /clock）时，等时间有效后再完整等待 6 s；握手期间节点一直在处理回调，实际使用中此时已收到 /clock，该分支基本不会触发，保留以对应 ROS1 语义。最初用 `clock->sleep_for()` 时实测节点 1.3 s 即退出、轨迹未执行。
- **pose_init 的发布握手**（`TODO(ros2-migration)`）：ROS1 中 `getNumSubscribers() > 0` 表示到控制器的 TCP 连接已建立，消息必定送达。DDS 的端点匹配是不对称的，Humble 也没有"订阅端已匹配本发布者"的 API。实测（Gazebo 运行中）同一发布者的第一条轨迹被 JTC 丢弃 3/15，3 s 后的第二条全部被采用；发布端 `wait_for_all_acked()` 在丢失时同样立即返回成功（同机共享内存），不能用作握手。因此在应用层完成握手，而不是固定时长等待：
  1. 发送前：除"看到 JTC 的订阅者"外，还要收到 JTC 的 `~/state`（与 ROS1 同名 `/lwr/joint_trajectory_controller/state`），即 JTC 一侧也已发现本节点；
  2. 发送后：由同一 `~/state` 确认 JTC 已采用该轨迹（期望速度非零，或期望位置已在目标处）；未确认则每 0.5 s 重发同一轨迹（相当于 TCP 重传），确认后停止并取消订阅。
  验证：仿真中 pose_init 连续 10 次全部执行（未触发重传）；用忽略前两条轨迹的假 JTC 验证重传路径：重传 2 次后被采用、共发送 3 条、随后正常退出。新增依赖 control_msgs。按用户决定保留该握手与重发（审查报告 F03），已知局限见需人工确认 2。
- **Rate**：ROS1 `ros::Rate` 按 ROS 时间（use_sim_time 时随 /clock），`rclcpp::Rate` 固定用系统时钟。pose_init、pose_initialization、ultrasound_traj_node、linear_traj_node、linear_traj_node_Luca 中的 `ros::Rate` 改为 `ultrasound_trajectory::RosRate`（ros_rate.h）：逐行移植 roscpp `Rate::sleep()`（期望结束时间、时间回跳处理、落后超过一个周期时重置起点），等待用节点时钟的 `sleep_until`。回调内调用 sleep 的结构、频率与调用位置不变。/clock 由节点的时钟线程处理（`NodeOptions::use_clock_thread` 默认 true），因此在单线程 executor 的回调里 sleep 时仿真时间仍会前进，与 roscpp 的内部线程相同。
- **时间**：类成员 `ros::Time` → `rclcpp::Time{0, 0, RCL_ROS_TIME}`（与 `node->now()` 同一时钟源，避免 rclcpp 不同时钟相减抛异常）；`isZero()` → `nanoseconds() == 0`；`ros::Duration(x).sleep()` → `clock->sleep_for(x)`。
- **消息字段**：`CartesianImpedancePoint` 的 `x_FRI/k_FRI/d_FRI/f_FRI` → `x_fri/k_fri/d_fri/f_fri`（ROS2 命名规则，见 lwr_controllers 的 NOTES）。
- **spin**：`ros::spin()` → `rclcpp::spin(obj.node())`；回调中的 `ros::shutdown()` → `rclcpp::shutdown()`，spin 随之返回。

## launch 对照
本包无 launch（ROS1 亦无；前次移植新增的 `ultrasound_trajectory.launch.py` 已删除）。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **pose_init 的话题**（`TODO(ros2-migration)`）：ROS1 发布到 `/lwr/joint_trajectory_controller/command`；Humble 的 joint_trajectory_controller 订阅 `~/joint_trajectory`（ros2_control 接口变化），因此 pose_init 直接发布到 `/lwr/joint_trajectory_controller/joint_trajectory`，无需重映射：`ros2 run ultrasound_trajectory pose_init --ros-args -p use_sim_time:=true`（仿真）。
2. **pose_init 握手与重发的局限**（审查报告 F03，按用户决定保持现状）：与 ROS1"等订阅者 → 发布一次 → 等 6 s → 退出"相比：
   - "已采用"判据是 `~/state` 中任一目标关节期望速度非零，或期望位置已在目标处；没有请求身份或时间校验。若 JTC 正在执行其他轨迹，首条 home 轨迹丢失或被拒时，仍在运动的旧状态也会被当作确认。
   - 发送前除命令订阅者外还要求收到 `/lwr/joint_trajectory_controller/state`；若 `~/state` 未发布或被重映射，ROS1 能发送，ROS2 会一直等待。发送后判据始终不满足时会无界重发（每 0.5 s），且不报告失败。
   - 重发的是零时间戳的新轨迹，不同于传输层重传；JTC 是否因此重置插值起点未实测。6 s 等待从确认之后开始计时。
   正常使用（JTC 空闲、`~/state` 存在）时仿真验证无误。如需完全回到基线行为，可删除握手，代价是 DDS 首条消息可能丢失（实测 3/15）。
3. **tool_relative_motion_node 的 robot_description**（`TODO(ros2-migration)`）：ROS1 用 `kdl_parser::treeFromParam("robot_description")` 读全局参数；ROS2 读本节点参数 `robot_description`，运行时需传入，例如 `--ros-args -p robot_description:="$(xacro …)"`。
4. **ee_incremental_impedance_node 的参数类型**：`declare_parameter` 按原默认值推导类型（double/int/bool），命令行传参时类型需一致（ROS1 `param()` 会做数值类型转换）。

## 编译、运行与测试
```bash
colcon build --packages-select ultrasound_trajectory
ros2 run ultrasound_trajectory ultrasound_traj_cartesian_node
```
