# kuka_launch ROS1 → ROS2 Humble 迁移说明

## 同名包合并
ROS1 工作空间中有两个名为 `kuka_launch` 的包（catkin 不允许同一工作空间内重名，两者不能同时编译）：

| ROS1 | 内容 |
|---|---|
| `kuka_ws/src/kuka/kuka_launch` | launch/single_lwr.launch、launch/impedance_simulation.launch、config/{controllers,hw_interface,t1_joint_limits}.yaml、config/980241-FRI-Driver.init |
| `kuka_ws/src/kuka/kuka_controllers/kuka_launch`（嵌套） | 同上，但缺少 launch/single_lwr.launch；controllers.yaml 缺少 `one_task_inverse_kinematics.task_space_gain: 1.0`；t1_joint_limits.yaml 只有注释不同（数值相同）；其余文件逐字节相同 |

前者是后者的超集（且包含实验室后加的 `task_space_gain`），合并结果即 `kuka/kuka_launch` 的内容，放在 `src/kuka-lwr/kuka_launch`。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml, CMakeLists.txt | 同名 | catkin → ament_cmake；安装 launch/、config/ |
| launch/single_lwr.launch | launch/single_lwr.launch.py | 与 `single_lwr_launch/launch/single_lwr.launch` 只差配置包与三个默认值，ROS2 版由 `single_lwr_launch/launch/single_lwr.launch.py` 派生，只改这些差异（见 launch 对照） |
| launch/impedance_simulation.launch | launch/impedance_simulation.launch.py | 逐项转换 |
| config/controllers.yaml | 同名 | ROS1 全局参数 → `/lwr/controller_manager` 参数文件，转换规则与 `single_lwr_robot/config/controllers.yaml` 相同（见下） |
| config/hw_interface.yaml, config/980241-FRI-Driver.init | 同名 | 逐字节相同 |
| config/t1_joint_limits.yaml | 同名 | 逐字节相同（single_lwr.launch 的 t1 分支读取的是 single_lwr_robot 的文件，ROS1 相同） |

## 主要改动
- **controllers.yaml**：每个控制器的 `type` 进入 `controller_manager.ros__parameters`，其余参数放在 `/lwr/<控制器名>`；`joint_state_controller/JointStateController` → `joint_state_broadcaster/JointStateBroadcaster`（`publish_rate: 100` 保留，另设 `update_rate: 100`，因为 Humble 的广播器忽略 `publish_rate`）；`position_controllers/JointTrajectoryController` → `joint_trajectory_controller/JointTrajectoryController`（position 命令，position/velocity 状态）；`stiffness_trajectory_controller` 未提供（与 single_lwr_robot 的处理相同：依赖 ROS2 中关闭的刚度元数据关节）。增益与 ROS1 相同（`joint_impedance_controller.stiffness_gains: 300`、`very_basic_joint_controller.gain: 30`、`basic_joint_controller` 无增益）。
- `controller_manager.update_rate: 1000`（仿真步长；实机由 launch 参数 `update_rate` 覆盖），同 single_lwr_robot。

## launch 对照
### single_lwr.launch → single_lwr.launch.py
节点、条件、参数与 `single_lwr_launch/launch/single_lwr.launch.py` 完全相同，差异（均为 ROS1 两个文件之间本来的差异）：

| 项 | single_lwr_launch | kuka_launch |
|---|---|---|
| `file` 默认值 | single_lwr_robot/config/980241-FRI-Driver.init | kuka_launch/config/980241-FRI-Driver.init |
| hw_interface.yaml、controllers.yaml | single_lwr_robot/config | kuka_launch/config |
| `controllers` 默认值 | joint_trajectory_controller | basic_joint_controller |
| `stopped_controllers` 默认值 | gravity_compensation_controller | （空） |
| `gui` 默认值 | true | false |

ROS2 下不得不变的部分（Gazebo Fortress、spawner 参数、全局参数 → 节点/控制器参数、`update_rate` 参数、robot_state_publisher 命名空间等）与 single_lwr_launch 相同，逐条说明见 `single_lwr_launch/MIGRATION_NOTES.md`；静态比对的 mapping 文件也沿用该包的（`migration_tests/kuka_launch/mapping_single_lwr*.json`）。

### impedance_simulation.launch → impedance_simulation.launch.py
| 节点 | 可执行 ROS1→ROS2 | args / 条件 / output | 差异 |
|---|---|---|---|
| /kuka_teleop | kuka_teleop/kuka_teleop.py（同名） | `$(arg teleop_pose_topic_name) $(arg teleop_vel_topic_name)`；`if teleop` | 一致 |
| /boltzmann_force_model_diag | force_models/boltzmann_force_model_diag.py | — | 一致（包不存在，见需人工确认 2） |
| /stiffness_observer | kuka_stiffness_estimation/constrained_observer | 6 个参数；screen | 一致（包未移植） |
| /impedance_controller | kuka_impedance_controller/impedance_controller | 10 个参数；screen | 一致（包未移植） |

所有 arg 名称与默认值相同；注释掉的 sigma_master 节点保留为注释。

## 硬件通信部分
无（实机硬件由 lwr_hw 的 launch 启动，未改动）。

## 未迁移的原测试
无。

## 需人工确认
1. **`stopped_controllers` 默认为空**：ROS2 的 spawner 在 `--stopped` 后没有控制器名时以退出码 2 结束（仿真实测；该节点不是 required，不影响其他节点）。ROS1 spawner 对空列表的行为未在本机验证。若需避免，启动时传入 `stopped_controllers:=<控制器名>`。
2. **impedance_simulation 依赖未移植的包**：`kuka_stiffness_estimation`、`kuka_impedance_controller` 不在教程使用范围内，未移植；`force_models` 在 ROS1 工作空间中也不存在。在这些包有 ROS2 版本之前，此 launch 无法启动（ROS1 中缺 force_models 时同样无法启动该节点）。package.xml 未声明这三个包的依赖（rosdep 无法解析）。
3. **basic_joint_controller 无增益**：ROS1 配置如此（K = D = 0，只输出重力补偿），见 kuka_controllers NOTES。

## 编译、运行与测试
```bash
colcon build --packages-select kuka_launch
ros2 launch kuka_launch single_lwr.launch.py                                   # 仿真，basic_joint_controller
ros2 launch kuka_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true
```
静态比对见 `../../../migration_tests/kuka_launch/README.md`。
