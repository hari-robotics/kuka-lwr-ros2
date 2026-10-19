# cartesian_position_node ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/cartesian_position_node`。前次 ROS2 版本是重写（参数化链路、URDF 话题订阅、按关节名映射、新增 launch/README），已全部丢弃并按原代码重新移植。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| src/cartesian_position_node.cpp | 同名 | main：`ros::init`/`NodeHandle`/`ros::spin` → `rclcpp::init`/`rclcpp::Node`/`rclcpp::spin`；`argv[1]` 取自 `rclcpp::remove_ros_arguments()`（ros::init 会从 argv 去掉 ROS 参数） |
| include/cartesian_position_node.h | 同名 | 类结构、`reorder()`、`order{0,1,6,2,3,4,5}`、tool_length 0.252、d1..d4、滤波系数 0.7、`std::cout` 输出全部不变；只换发布/订阅/日志与消息类型 |
| include/forward_kinematics.h | 同名 | 解析式 Jacobian 与 KDL 求解器不变；参数读取见"主要改动" |
| include/pseudo_inversion.h | 同名 | 逐字节复制 |
| CMakeLists.txt, package.xml | 同名 | catkin → ament_cmake；保留 `-fpermissive`；依赖 lwr_controllers（`utils/pseudo_inversion.h`、PoseRPY/RPY 消息）、kdl_parser、urdf、control_msgs |

## 主要改动
- **话题**：`/cartesian_position/pose/tooltip`、`/cartesian_position/pose/end_effector`（`lwr_controllers/msg/PoseRPY`）、`/cartesian_position/vel/tooltip`、`/cartesian_position/vel/end_effector`（`geometry_msgs/msg/Twist`），QoS depth 10；订阅 `argv[1]` 指定的 `sensor_msgs/msg/JointState`，depth 10。与 ROS1 相同。
- **参数**：ROS1 读取全局参数——`ros::param::search` 找到的 `/robot_description`、`/lwr/root`、`/lwr/tip`（来自 hw_interface.yaml）。ROS2 无全局参数，改为本节点参数 `robot_description`、`lwr.root`、`lwr.tip`，由 `single_lwr_tool.launch.py` 传入同一 URDF 与 hw_interface.yaml；未设置时与 ROS1 一样报错并不初始化求解器。
- **n.shutdown()**：forward_kinematics.h 在 URDF 读取失败时调用的 `n.shutdown()` 无直接对应，保留为 `TODO(ros2-migration)` 注释，init 返回 false（ROS1 随后同样继续 spin）。

## launch 对照
本包无 launch（ROS1 亦无）。由 `single_lwr_launch/launch/single_lwr_tool.launch.py` 启动，见该包 NOTES。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **/lwr/joint_states 的关节顺序**（`TODO(ros2-migration)`）：回调按固定重排 `{0,1,6,2,3,4,5}` 处理 JointState，依赖 ROS1 joint_state_controller 的字母序（a1..a6, e1，来自 resource manager 的 std::map）。Humble 的 joint_state_broadcaster（2.54）用 unordered_map 收集关节名，顺序不固定，也无法通过 `joints` 参数指定（实测 a2,a1,a3,a4,a5,e1,a6）。未适配时末端位姿与 TF 相差约 0.6 m。现在回调先按关节名排成 ROS1 的字母序，再执行原重排，其余计算不变；运行验证与 TF 一致（../../migration_tests/single_lwr_launch/test_sim_workflow.py）。
2. **未初始化成员（原样）**：`vel_previous_` 首次使用前未赋值（`geometry_msgs::msg::Twist` 默认零初始化，ROS1 消息同样零初始化，行为一致）。

## 编译、运行与测试
```bash
colcon build --packages-select cartesian_position_node
ros2 launch single_lwr_launch single_lwr_tool.launch.py
```
