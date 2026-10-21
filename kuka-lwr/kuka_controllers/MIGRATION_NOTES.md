# kuka_controllers ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka/kuka_controllers`（实验室教程 "kuka steps" Part 2：`single_lwr_tool.launch ... controller:=hierarchical_controller` + `kuka_teleop.py`）。ROS1 中位于 `kuka/`，ROS2 按"KUKA 相关包放入 kuka-lwr"放在 `src/kuka-lwr/kuka_controllers`。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml, CMakeLists.txt | 同名 | catkin → ament_cmake；库 `libkuka_controllers`、可执行 `rcm_marker` 保留 |
| kuka_controllers_plugins.xml | 同名 | 基类 `controller_interface::ControllerBase` → `controller_interface::ControllerInterface`；类名/类型名不变 |
| include/KinematicChainControllerBase.h | 同名 | ROS1 与 `lwr_controllers/KinematicChainControllerBase.h` 逐行相同（仅空白不同），定义同一个模板 `controller_interface::KinematicChainControllerBase`；ROS2 中改为包含已移植的 lwr_controllers 版本，避免同一 controller_manager 进程内出现两份同名模板定义 |
| src/KinematicChainControllerBase.cpp | 同名 | ROS1 内容是 `EffortJointInterface` 的 `getHandles()` 特化（额外取 `_stiffness/_damping/_set_point` 句柄）；ROS2 无此函数，见需人工确认 3 |
| include/basic_joint_controller.h, src/basic_joint_controller.cpp | 同名 | 只换 ROS 接口 |
| include/hierarchical_controller.h, src/hierarchical_controller.cpp | 同名 | 只换 ROS 接口；初始化读数位置见需人工确认 1 |
| src/very_basic_joint_controller.cpp | 同名 | 只换 ROS 接口；初始化读数位置见需人工确认 2 |
| src/rcm_marker.cpp | 同名 | 只换 ROS 接口 |
| src/hierarchical_controller(valentin).cpp, src/basic_joint_group_controller.cpp | — | ROS1 CMake 未编译的旧版本，不迁移 |
| kuka_launch/（嵌套的同名包） | — | 与 `kuka/kuka_launch` 同名，合并为 `kuka-lwr/kuka_launch`（见该包 NOTES） |

## 主要改动
- **控制器框架**：使用 `lwr_controllers` 已移植的适配层（`ControllerAdapter`/`KinematicChainControllerBase`，见 `lwr_controllers/MIGRATION_NOTES.md`）：ROS1 `init(hw, n)` → `configure(n)`，`update()` → `update_command()`，`starting()/stopping()` 在激活/停用时调用；句柄改为 ros2_control 借出的接口。接口类型：BasicJointController、VeryBasicJointController → `effort`；HierarchicalController → `position`（与 ROS1 的 `EffortJointInterface`/`PositionJointInterface` 对应，lwr_hw 据此选择 JOINT_IMPEDANCE / JOINT_POSITION）。
- **话题**：`command` 仍为控制器私有话题 `/lwr/<控制器名>/command`（类型 `std_msgs/msg/Float64MultiArray`、`std_msgs/msg/Float64`、`lwr_controllers/msg/PoseRPY`），队列深度 1 不变。
- **参数**：`root_name`/`tip_name`/`stiffness_gains`/`damping_gains`/`joint`/`gain`/`kp_main`/`kd_main`/`kp_rcm`/`kd_rcm` 为控制器节点参数（来自 controller_manager 参数文件）。ROS1 用 `getParam(float&)` 读 HierarchicalController 的四个增益（其中 `kd_main: 0`、`kd_rcm: 0` 是整数）；ROS2 参数没有 float 类型，先读为 double 再赋给原来的 float 成员，数值相同。
- **构建**：C++17（rclcpp 要求；ROS1 `-std=c++11`），保留 `-fpermissive`；`boost::scoped_ptr` → `std::unique_ptr`。ROS1 安装 `include/kuka_controllers/`（不存在），ROS2 安装 `include/` 到 `include/kuka_controllers`；ROS1 未安装 `rcm_marker`（rosrun 在 devel 空间找到），ROS2 安装到 `lib/kuka_controllers`。
- **日志**：`ROS_*` → `RCLCPP_*`（控制器节点 logger），注释中的旧日志行保持原样。

## launch 对照
本包无 launch（ROS1 亦无；嵌套的 kuka_launch 见 `kuka-lwr/kuka_launch`）。

## 硬件通信部分
无（控制器经 ros2_control 接口与 lwr_hw 交互，lwr_hw 未改动）。

## 未迁移的原测试
无。

## 需人工确认
1. **HierarchicalController 初始化读数**（`TODO(ros2-migration)`，`src/hierarchical_controller.cpp`）：ROS1 在 `init()`（控制器加载时）读取关节位置，算出期望末端位姿、RCM 点和工具向量；`starting()` 为空。ros2_control 在 configure 阶段还没有借出状态接口（读到 0），若原样保留，激活后第一个周期就会命令 q = 0。现把这段代码原样移到 `starting()`，只在 configure 后的**第一次激活**执行一次；之后停用再激活不会重读，这与 ROS1 一致（ROS1 也只在加载时读一次，重新 start 后命令的是加载时的位姿）。仿真实测：切换后关节漂移 0.0019 rad，PoseRPY 命令下工具尖端到位误差 2 mm（`migration_tests/tutorial_packages`）。
2. **VeryBasicJointController 初始化读数**（`TODO(ros2-migration)`）：ROS1 在 `init()` 中 `command_ = joint_.getPosition()`；同理改为第一次激活时读取。
3. **BasicJointController 不再占用刚度/阻尼/设定点接口**：ROS1 的 `getHandles()` 特化额外取了 `<joint>_stiffness/_damping/_set_point` 句柄（即占用这些资源），但从不写入；ROS2 只申请 `<joint>/effort`。lwr_hw 写给 FRI 的刚度/阻尼/设定点仍是各自命令变量的当前值，与 ROS1 相同；差别只在于其他控制器可以同时申请这些接口（ROS1 会因资源冲突拒绝）。BasicJointController 的 `init()` 中读关节状态同样读到 0，但这些值不参与控制律。
4. **保留的实验室代码问题（未修改）**：
   - HierarchicalController 的工具长度在 `init()` 为 0.252、在 `update()` 为 0.275，且 `cartesian_position_node` 的 tooltip 用 0.252；`kuka_teleop.py` 以 tooltip 作为目标时，控制器会把机械臂沿工具轴拉动约 2.3 cm（仿真中已观察到，ROS1 同样如此）。
   - `float init_tip[]={}`、`float tip[]={}` 为零长度数组再写 3 个元素（未定义行为，与 lwr_controllers 的 IK 相同）。
   - BasicJointController 控制律 `D*qdot` 为正反馈号；kuka_launch 的 controllers.yaml 未给 `stiffness_gains/damping_gains`，此时 K = D = 0，只输出重力补偿（启动时打印 "gain not set" 警告，ROS1 相同）。
   - HierarchicalController 在运动期间每个控制周期打印一次 "RCM error"（500/1000 Hz）。
5. **急停与停用**：由 lwr_controllers 适配层处理（`/lwr/emergency_stop` 为 true 时每周期触发 `stopping()+starting()`，停用时 effort 命令写 0），语义见 `lwr_controllers/MIGRATION_NOTES.md` 与审查报告 F02；HierarchicalController 的 `starting()` 只在第一次激活时有动作，因此急停重同步对它没有作用（ROS1 的 `starting()` 为空，同样没有作用）。
6. **rcm_marker**：`header.frame_id` 由 `"/world"` 改为 `"world"`（tf2 不接受前导斜杠，RViz2 会拒绝该帧）；`ros::Rate(30)` 按 ROS 时间，`rclcpp::Rate(30)` 按系统时钟，仿真时间下节拍不同（仅影响标记发布频率）。

## 编译、运行与测试
```bash
colcon build --packages-select kuka_controllers
# 教程 kuka steps Part 2（仿真）：
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=hierarchical_controller
ros2 topic pub --once /lwr/hierarchical_controller/command lwr_controllers/msg/PoseRPY "{id: 1, position: {x: ..., y: ..., z: ...}}"
ros2 run kuka_controllers rcm_marker
```
运行时测试见 `../../../migration_tests/tutorial_packages/README.md`。
