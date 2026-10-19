# single_lwr_robot ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/single_lwr_example/single_lwr_robot`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| config/controllers.yaml | 同名 | ROS1 全局参数 → `/lwr/controller_manager` 参数文件（ros__parameters）；控制器名称、类型、root/tip、增益与实验室版本相同（publish_rate 200、joint_impedance stiffness_gains 1、task_space_gain 1.0、hierarchical/basic_joint_controller） |
| config/hw_interface.yaml | 同名 | 逐字节相同（由 launch 读取 root/tip 写入 ros2_control `<hardware>` 参数，tool launch 中也作为 cartesian_position_node 的参数） |
| config/t1_joint_limits.yaml | 同名 | 逐字节相同（实验室版本，见需人工确认 3） |
| config/980241-FRI-Driver.init | 同名 | 逐字节相同 |
| — | config/clock_bridge.yaml | Fortress /clock 桥接配置（`/world/default/clock`） |
| robot/single_lwr_robot.urdf.xacro | 同名 | 逐字节相同（1 m 底座 box，质量 50，手臂装在 z=1 处） |
| worlds/simple_environment.world | 同名 | world 名保持 `default`（实验室改为 `world`，Fortress 拒绝该保留名，见需人工确认 6）；Classic `model://ground_plane`、`model://sun` → Fortress SDF 内联地面与太阳，物理步长 1 ms（Classic 默认）；系统插件用 Fortress 名 `ignition::gazebo::systems::*` |

## 主要改动
- **控制器类型**：`joint_state_controller/JointStateController` → `joint_state_broadcaster/JointStateBroadcaster`；`position_controllers/JointTrajectoryController` → `joint_trajectory_controller/JointTrajectoryController`（command_interfaces: position）。
- **控制器 update_rate**：joint_state_controller 设 `update_rate: 200`（对应 ROS1 `publish_rate: 200`，见需人工确认 4）。
- **update_rate: 1000**：ROS1 仿真控制周期 = Gazebo 步长（0.001 s）；真机由 launch 参数 `update_rate` 覆盖（ROS1 = 1/getSampleTime()）。
- 前次移植把装配改成"world 原点、无底座"并去掉刚度被动关节等，均已恢复为实验室版本。

## launch 对照
本包无 launch（ROS1 亦无）。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **stiffness_trajectory_controller**：ROS1 配置中有，但依赖刚度元数据关节；ROS2 未提供（见 lwr_controllers 的 NOTES）。
2. **hierarchical_controller / basic_joint_controller**：类型 `kuka_controllers/*` 来自 `kuka_ws/src/kuka/kuka_controllers`，该包不在本工作空间，配置照搬但无法加载（ROS1 中未编译该包时同样无法加载）。
3. **t1_joint_limits.yaml（实验室版本）**：`joint_limits:` 下为空，各关节键位于顶层，速度上限 3.5、无加速度限制。`t1_limits:=true` 时 ROS1 roslaunch 无法设置空值参数而启动失败；ROS2 `move_group.launch.py` 叠加后同样因参数值 None 在启动时报错（`Unexpected type for parameter value None`）。保持一致，未修正。
4. **joint_state_controller publish_rate**：Humble 的 JointStateBroadcaster 不使用 `publish_rate`，每个控制周期（1000 Hz）都发布，`/lwr/joint_states` 及其下游（cartesian_position_node 的位姿、ee_incremental_motion 的 IK 指令）都变成 1000 Hz，Gazebo CPU 166%。已加控制器参数 `update_rate: 200`，实测 200 Hz，与 ROS1 一致（`publish_rate: 200` 保留）。
5. **底座 box 的 `<static>`**：写在 `<gazebo reference="box">` 内（link 级），Classic 与 Fortress 都不把它当作模型静态；模型以 box 为根自由放在地面上（ROS1 同样如此，质量 50 用于稳住底座）。
6. **world 名**：实验室把 world 名从 `default` 改为 `world`（Gazebo Classic 允许）；Fortress 报 `The supplied world name [world] is reserved` 并拒绝加载，因此 ROS2 仍为 `default`（spawn `-world default`、clock 桥接 `/world/default/clock`）。ROS 侧话题与行为不受影响。

## 编译、运行与测试
由 single_lwr_launch 使用，见其 NOTES。
