# lwr_controllers ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/lwr_controllers`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| include/lwr_controllers/KinematicChainControllerBase.h | 同名 | `controller_interface::Controller<JI>` → `ControllerInterface` 适配层；URDF/KDL 链、关节限值解析不变 |
| include/lwr_controllers/PIDKinematicChainControllerBase.h | 同名 | `control_toolbox::Pid::init(NodeHandle)` → 读 `pid_<joint>.p/i/d/i_clamp_*` 后 `initPid` |
| — | include/lwr_controllers/ros2_adapter.h | `NodeParameters`：保留 `nh_.getParam/advertise/subscribe` 调用点，内部用 ROS2 节点；kdl_conversions 替代函数 |
| src/*.cpp, include/lwr_controllers/*.h（14 个控制器） | 同名 | 只替换 ROS API（init→configure、update→update_command、消息类型、日志、`period.toSec()`→`seconds()`、PID 周期改纳秒）；控制律未改 |
| src/KinematicChainControllerBase.cpp | 同名 | ROS1 EffortJointInterface 特化（取 stiffness/damping/set_point 句柄）改由各控制器的 `command_interface_configuration` 声明 |
| src/cartesian_command_vito.cpp | 同名 | ROS1/ROS2 均不参与编译，仅随包保留 |
| src/joint_impedance_controller.cpp, .h（实验室重写，JLF 2022-12） | 同名 | 笛卡尔阻抗 + 零空间力矩控制律、增益、话题逐行保留，只换 ROS API（见"实验室改动"） |
| src/one_task_inverse_kinematics.cpp, .h（实验室修改） | 同名 | 自适应阻尼 SVD 伪逆、速度/加速度/步长/软限位保护、`task_space_gain`、工具长度处理逐行保留 |
| src/cartesian_impedance_controller.cpp, .h（实验室修改） | 同名 | 初始刚度 (800,800,200,100,100,100)、URDF→KDL 链、`/velocity_ee` 发布 |
| src/gravity_compensation.cpp, .h；src/computed_torque_controller.cpp；src/arm_state_controller.cpp, .h（实验室修改） | 同名 | 重力补偿输出 0.1；日志/注释；arm_state 头文件引入 FRI 头与成员（行为不变） |
| include/utils/pseudo_inverse.h, matrix_multiplication.h, null_space_projector.h（实验室新增） | 同名 | 逐字节复制（无源文件使用） |
| `* (backup).*`、`*_old.*`、`joint_impedance_controller_1218.cpp`、`cartesian_impedance_controller_M.cpp`（实验室） | — | 不参与 ROS1 编译的编辑器备份/旧版本，不迁移 |
| msg/*.msg, srv/*.srv | 同名 | 见"主要改动·接口" |
| lwr_controllers_plugins.xml | 同名 | base_class_type → `controller_interface::ControllerInterface` |
| launch/load_controller.launch, launch/controllers/*.launch（16 个） | 同名 .launch.py | 见"launch 对照" |

## 主要改动
- **节点结构**：控制器为 ros2_control 插件；`init(robot, nh)` → `on_configure`，`starting` 在 `on_activate` 中调用（loaned interface 此时才可用）。OneTaskInverseKinematics 把 ROS1 `init()` 中读取当前位姿/设定期望位姿移到 `starting()`，因为 ROS2 在 configure 时没有状态接口。
- **参数**：私有参数（root_name、tip_name、stiffness_gains、pid_<joint> 等）名称与默认值不变；`robot_description` 从控制器节点参数读取（由 launch 写入 `/**` 或由 gz_ros2_control 注入），对应 ROS1 `ros::param::search("robot_description")`。
- **话题/服务**：ROS1 控制器私有命名空间 → ROS2 `~/` 相对名，全名不变（如 `/lwr/one_task_inverse_kinematics/command`）；queue_size → QoS depth。
- **线程**：与 ROS1 相同，订阅/服务回调与 `update()` 之间不加锁（ROS1 AsyncSpinner 线程与控制循环并发）。前次移植的 `callback_mutex_` 已删除（见需人工确认 2）。
- **急停**：ROS1 `lwr_hw_fri_node` 在 `/lwr/emergency_stop` 为真期间每周期以 `resetControllers=true` 调用 `manager.update`，即对运行中的控制器 stopping()+starting()。controller_manager 无此接口，改由适配层实现：KinematicChainControllerBase 在 on_configure 订阅 `emergency_stop`（控制器命名空间，即 `/lwr/emergency_stop`，`std_msgs/Bool`，depth 1），`update()` 中为真时先 `stopping(time); starting(time);` 再执行控制器的 update，与 ROS1 manager 的顺序相同。适用于所有派生控制器；在 Gazebo/FRIL 下也生效（ROS1 只有 FRI 节点有急停），见 lwr_hw NOTES 需人工确认 3。
- **接口**：ROS2 字段名须小写，`CartesianImpedancePoint` 的 `x_FRI/k_FRI/d_FRI/f_FRI` → `x_fri/k_fri/d_fri/f_fri`；`Header` → `std_msgs/Header`；删除 `PoseRPY.msg` 中无效的 `#include` 行。
- **撤回的前次移植改动**：CartesianImpedanceController 取 7 个关节句柄（ROS1 为 6）、`set_command` 服务回调转发指令（ROS1 为空实现），均恢复 ROS1 行为；`package.xml` 中的 `<test_depend>` 已删除。
- **构建**：catkin → ament_cmake + rosidl；单一插件库 `lwr_controllers_plugins`；依赖 lwr_hw（ROS1 亦依赖；arm_state_controller.h 引入 `lwr_hw_fri.hpp`、FRI 头）。

## 实验室改动的移植
- **JointImpedanceController**：订阅 `/est_ext_torque`（lwr_hw 发布）与 `/stiffness`（回调为空，同 ROS1），发布 `/velocity`、`/velocity_jac`、`/cartesian_forces`；`command` 接受 7 维关节角或 6 维（只取前 3 维位置）；每周期写 effort = τ_task + τ_null，stiffness/damping 指令 0，set_point = 测量关节角。ROS1 在 init() 中从句柄读取的 K/D/q_des/q_start 在 ROS2 configure 时不可用：K/D/q_des 取默认后由 `stiffness_gains`/`damping_gains` 覆盖（同前次移植），`q_start_` 改在第一次 starting() 读取（`TODO(ros2-migration)`）。
- **OneTaskInverseKinematics**：`n.param("task_space_gain", …, 1.0)` → 参数缺省 1.0；`ROS_WARN_THROTTLE(1.0, …)` → `RCLCPP_WARN_THROTTLE(…, 1000, …)`。init() 中读关节位置/打印初始位姿的代码保留在 configure（此时状态接口未借出，读数为 0，`TODO(ros2-migration)`）；实验室新增的 starting() 重新同步当前位姿，行为以它为准。
- **CartesianImpedanceController**：`ros::param::search("robot_description")` → 控制器节点参数 `robot_description`（与 KinematicChainControllerBase 相同来源）；`n.shutdown()` 无对应，configure 返回 false；`/velocity_ee` 速度计算需要关节位置/速度，on_activate 中为 6 个关节句柄绑定 position/velocity/effort 状态接口（ROS1 由 PositionCartesianInterface 的关节句柄提供）。

## launch 对照
### load_controller.launch → load_controller.launch.py
| 节点全名 | 可执行 ROS1→ROS2 | 参数 | remap/args/条件/respawn | 差异 |
|---|---|---|---|---|
| /lwr/controller_spawner | controller_manager/spawner（同） | — | args `joint_state_controller $(controller)`，output=screen | 一致（`shell=True` 让 shell 拆分以空格分隔的控制器列表，与 roslaunch 相同） |
| /lwr/controller_spawner_stopped | 同上 | — | args `--stopped $(stopped_controllers)`，output=screen | 一致 |
launch 参数 controller（默认 `joint_position_trajectoy_controller`，原文拼写）、stopped_controllers（默认 `OneTaskInverseKinematics`）不变。ROS1 的 `rosparam load lwr_controllers/config/lwr_controllers.yaml` 在 ROS1 包中本就缺少该文件；ROS2 控制器参数属于 controller_manager，删除并加 TODO。

### controllers/*.launch → controllers/*.launch.py（16 个）
均 include `load_controller.launch.py` 并传入与 ROS1 相同的 `controller` 值（如 `BacksteppingController`、`joint0_effort_position_controller … joint6_effort_position_controller`）；`joint_state_controller.launch.py` 为单个 `/lwr/controller_spawner joint_state_controller`。差异：一致。
前次移植把这些控制器名改成了 single_lwr_robot 配置中的名字、把 7 个单关节控制器换成一个轨迹控制器，并在 controllers.yaml 新增了 3 个控制器；均已恢复。注意这些 launch 引用的控制器在 ROS1 与 ROS2 的配置中都不存在（同 ROS1），需要自备 controller_manager 参数文件。

## 硬件通信部分
无。

## 未迁移的原测试
无（ROS1 包无测试）。

## 需人工确认
1. **消息字段改名**：`lwr_controllers/msg/CartesianImpedancePoint` 字段 `*_FRI` → `*_fri`（ROS2 命名规则强制）。发布该消息的外部节点需同步改名。
2. **回调与 update 的并发**：已按 ROS1 去掉锁，回调与 `update()` 并发访问控制器成员（ROS1 相同的数据竞争，如多元素的期望位姿可能在一个周期内被部分更新）。ROS1 中部分控制器（multi_task_priority_inverse_kinematics、backstepping、minimum_effort、dynamic_sliding_mode 等）在 update 末尾调用 `ros::spinOnce()`；ROS2 删除（回调由 controller_manager 的 executor 处理），因此回调不再可能在 update 线程上执行，命令进入算法的周期边界可能不同，未测得延迟。不建议在 update 内 spin。
3. **stiffness_trajectory_controller**：ROS1 controllers.yaml 中的刚度轨迹控制器依赖刚度元数据关节的 PositionJointInterface（ROS1 实际上把刚度句柄注册在 EffortJointInterface，无法启动）；ROS2 未提供，需要时应基于 `stiffness` 指令接口另行实现。
4. **pose 消息转换**：`tf::poseMsgToKDL` 等改为 ros2_adapter.h 中的等价实现（四元数/向量逐分量），请在用到 CartesianImpedanceController 时复核坐标约定。
5. **实验室代码中的疑似缺陷（按规则原样保留）**：
   - `one_task_inverse_kinematics.cpp`：`float init_tip[]={};`、`float des_tip[]={};` 为零长数组却写入 3 个元素（栈越界写，未定义行为）。
   - `cartesian_impedance_controller.cpp`：只取 6 个关节句柄（ROS1 原样），实验室的 `/velocity_ee` 计算按 7 关节：第 7 关节角恒为 0、`vel_jnt_(6)` 未初始化，发布的末端速度不可信。
   - `joint_impedance_controller.cpp`：`update()` 每周期 7 次 `RCLCPP_INFO`（ROS1 为 `ROS_INFO`），加上方向分量的 3 次输出，实时循环内日志量大。
6. **适配层新增分支**（KinematicChainControllerBase.h）：`period <= 0` 时 update 直接返回（ROS1 控制器无此跳过，正常运行周期为正）；stopping 之后适配层把 effort 指令写 0 并释放句柄（ROS1 空 stopping 不写指令；硬件 doSwitch 本身也清附加力矩）。未验证切换周期的写入内容。
7. **InverseDynamicsController 的 n_dof_**：ROS1 构造函数与 init 均未初始化 `n_dof_`（读取未初始化值）；ROS2 在 configure 中赋值 `n_dof_ = kdl_chain_.getNrOfJoints()`。这是非 API 改动，未回退为未初始化读取；如实验室使用该控制器，应先记录 ROS1 部署二进制的实际维数。
8. **joint_impedance_controller 的 q_start_**：ROS1 在加载时读取，ROS2 在第一次激活时读取；`K_joint`、`D_joint` 全为 0 时零空间项与 q_start_ 无关。

## 编译、运行与测试
```bash
colcon build --packages-select lwr_controllers
ros2 launch single_lwr_launch single_lwr.launch.py controllers:=one_task_inverse_kinematics
```
测试见 ../../../migration_tests/README.md（`lwr_controllers/` 下 17 个期望清单）。
