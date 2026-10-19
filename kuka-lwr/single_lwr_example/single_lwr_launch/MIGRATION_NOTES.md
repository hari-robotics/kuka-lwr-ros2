# single_lwr_launch ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/single_lwr_example/single_lwr_launch`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| launch/single_lwr.launch | launch/single_lwr.launch.py | 节点、参数、条件与 ROS1 一一对应 |
| launch/single_lwr_tool.launch | launch/single_lwr_tool.launch.py | 独立文件（ROS1 亦为独立文件）；与 single_lwr.launch.py 的差异只在 ROS1 两文件的三处差异 |
| launch/rviz_config.rviz | 同名 | 实验室版本转换为 rviz2 格式：类名 `rviz/*` → `rviz_common/*`、`rviz_default_plugins/*`；RobotModel 的 `Robot Description` → `Description Topic`（transient local）；Marker 的 `Marker Topic` → `Topic`；其余（Fixed Frame `world`、视角、窗口）不变 |
| launch/single_lwr_tool (copy).launch | — | 编辑器副本，不迁移 |

## 主要改动
- 声明式写法（无 OpaqueFunction），可被静态比对器完整解析；模型用 `Command(xacro ...)` 展开。
- 默认值按实验室版本：port 49938、ip 192.168.10.2、gui true、stopped_controllers `gravity_compensation_controller`。
- 删除前次移植新增的 `use_cartesian_position` 参数与对应节点（ROS1 single_lwr.launch 没有该节点；tool launch 无条件启动它，与 ROS1 相同）。

## launch 对照
### single_lwr.launch → single_lwr.launch.py
launch 参数 robot_name、use_lwr_sim、lwr_powered、port、ip、file、t1_limits、controllers、stopped_controllers、load_moveit、use_rviz、gui：名称与默认值不变。新增 `update_rate`（默认 500，仅真机，见 lwr_hw 的 NOTES）。`gui` 在 ROS1 中声明但未使用，ROS2 同样未使用（Gazebo 总是带 GUI，同 ROS1 `empty_world gui:=true`）。

| 节点全名 | 可执行 ROS1→ROS2 | 参数 | remap/args/条件/respawn | 差异 |
|---|---|---|---|---|
| /joint_state_publisher | joint_state_publisher（同） | source_list=['/lwr/joint_states']；robot_description | 无条件 | 全局 /robot_description → 节点参数 |
| /robot_state_publisher | robot_state_publisher（同） | robot_description | 无条件 | 全局 /robot_description → 节点参数。仿真时为 **/lwr/robot_state_publisher**，remap `joint_states→/joint_states`、`robot_description→/robot_description`：gz_ros2_control（Humble）只在自身 /lwr 命名空间查找 robot_state_publisher |
| /lwr_rviz | rviz/rviz → rviz2/rviz2 | — | `-d rviz_config.rviz`，if use_rviz，output=screen | 可执行改名 |
| /spawn_urdf | gazebo_ros/spawn_model → ros_gz_sim/create | robot_description | `-param robot_description -urdf -model <robot_name>` → `-world default -param robot_description -name <robot_name>`；if use_lwr_sim | Gazebo Classic → Fortress（Fortress 不允许 world 名 `world`，仍为 `default`，见 single_lwr_robot） |
| (include) gazebo_ros/empty_world.launch | → ros_gz_sim/gz_sim.launch.py | — | world_name→gz_args 中的 world；paused:=false→`-r`；gui/headless→不加 `-s` | Classic → Fortress；`use_sim_time:=true` → `SetParameter(use_sim_time=True)`（仿真时下发到所有节点） |
| /lwr_clock_bridge（新增） | ros_gz_bridge/parameter_bridge | config_file=clock_bridge.yaml | if use_lwr_sim | Fortress 不向 ROS 发布 /clock（ROS1 由 gazebo_ros 发布） |
| /lwr/controller_manager（运行时） | — | — | — | 仿真时由 gz_ros2_control 插件创建（ROS1 在 /gazebo 进程内由 lwr_hw_gazebo_plugin 创建）；真机时为 lwr_hw.launch.py 的 ros2_control_node（替代 /lwr/lwr_hw） |
| (include) single_lwr_moveit/move_group | move_group.launch.py | allow_trajectory_execution=true, fake_execution=false, info=true, debug=false | if load_moveit | 一致（MoveIt 2 参数布局见 single_lwr_moveit 的 NOTES） |
| (include) single_lwr_moveit/moveit_rviz | moveit_rviz.launch.py | config=true, debug=false | if load_moveit | 一致 |
| (rosparam) t1_joint_limits.yaml → /robot_description_planning | — | — | if t1_limits | move_group.launch.py 读取 t1_limits 并把 T1 限值叠加到 robot_description_planning（ROS2 无全局参数）。实验室 yaml 在两个版本中都会使该分支启动失败，见 single_lwr_robot 的 NOTES |
| (rosparam) hw_interface.yaml、controllers.yaml | — | — | — | 全局 /lwr/* → controllers.yaml 作为 controller_manager 参数文件，root/tip 作为 ros2_control `<hardware>` 参数 |
| (include, ns=lwr) lwr_hw/lwr_hw.launch | lwr_hw.launch.py | port、ip、file_with_path、name=lwr；新增 description_file、xacro_args、controllers_file、update_rate | if lwr_powered | /lwr/lwr_hw → /lwr/controller_manager（ros2_control_node），见 lwr_hw 的 NOTES；ROS1 在 ns 内复制的 /lwr/robot_description 改为 lwr_hw.launch.py 自行展开同一 xacro |
| /lwr/controller_spawner | controller_manager/spawner（同） | — | `joint_state_controller arm_state_controller $(controllers) --shutdown-timeout 2` → `… --controller-manager-timeout 30`；output=screen | Humble spawner 无 `--shutdown-timeout`；`--controller-manager-timeout 30` 对应 ROS1 spawner 默认 30 s 等待；`shell=True` 用于拆分控制器列表 |
| /lwr/controller_stopper | 同上 | — | `--stopped $(stopped_controllers) --shutdown-timeout 2` → `--stopped … --controller-manager-timeout 30` | 同上（Humble 对 `--stopped` 提示改用 `--inactive`，行为相同） |

### single_lwr_tool.launch → single_lwr_tool.launch.py
与 single_lwr.launch.py 相同（上表全部适用），另有 ROS1 两文件之间的三处差异：
| 项 | ROS1 | ROS2 | 差异 |
|---|---|---|---|
| controllers 默认值 | one_task_inverse_kinematics | one_task_inverse_kinematics | 一致 |
| 全局 robot_description | `xacro --inorder $(find lwr_robot_tool)/urdf/lwr_with_tool.urdf` | 同一文件经 xacro 展开，附加 ros2_control 硬件参数（同 single_lwr） | 传给 joint_state_publisher、robot_state_publisher、spawn_urdf、cartesian_position_node 的节点参数。真机分支的 /lwr 描述仍为 `single_lwr_robot.urdf.xacro`（ROS1 原样） |
| /cartesian_position_node | `cartesian_position_node`，args `/lwr/joint_states`，respawn=false | 同名同可执行，arguments `['/lwr/joint_states']`，respawn=False | 节点读取的全局 /robot_description、/lwr/root、/lwr/tip 改为节点参数 `robot_description`、`lwr.root`、`lwr.tip`（`lwr.joints` 随 hw_interface.yaml 一并传入） |

静态比对：single_lwr 的默认、display、real、moveit+rviz 分支与 single_lwr_tool 的默认、display、real 分支均无不一致；moveit+t1 分支为预期失败（见上）。详见 ../../../../migration_tests/single_lwr_launch。

运行验证（2026-10-19，Gazebo Fortress 仅服务端）：single_lwr_tool 默认分支启动后 joint_state_controller、arm_state_controller、one_task_inverse_kinematics 激活，gravity_compensation_controller 为 inactive；`/est_ext_torque`、`/cartesian_position/pose/*` 有数据；`controllers:=joint_impedance_controller` 时 `/velocity_jac`、`/cartesian_forces` 发布，切换到 cartesian_impedance_controller 后 `/velocity_ee` 发布。零位构型下 IK 因实验室的奇异保护（σ_min < 0.01）保持不动，属原代码行为。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **tool launch + load_moveit**：ROS1 move_group 读取全局 /robot_description（tool 描述，无 `box`），而 SRDF 链以 `box` 为基座，该组合在 ROS1 中无法规划；ROS2 move_group.launch.py 按 `robot_name` 展开 single_lwr_robot，未模拟此差异。
2. **真机控制频率**：`update_rate` 默认 500 Hz，对应随包 KRL 的 `friOpen(2)`；如现场 KRL 周期不同请在命令行指定。
3. **Gazebo GUI（WSL）**：本机 GUI 可能因 `Ogre::UnimplementedException (GL3PlusTextureGpu::copyTo)` 退出并带走仿真服务端；与本包代码无关。
4. **spawner 生命周期**：ROS1 spawner 常驻，退出时卸载控制器；Humble spawner 加载完即退出。

## 编译、运行与测试
```bash
colcon build && source install/setup.bash
ros2 launch single_lwr_launch single_lwr.launch.py                          # Gazebo
ros2 launch single_lwr_launch single_lwr_tool.launch.py                     # Gazebo + 工具 + cartesian_position_node
ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true
```
测试见 ../../../../migration_tests/README.md。
