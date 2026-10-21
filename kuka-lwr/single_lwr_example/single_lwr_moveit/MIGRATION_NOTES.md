# single_lwr_moveit ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/single_lwr_example/single_lwr_moveit`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| config/kinematics.yaml, ompl_planning.yaml, joint_limits.yaml, fake_controllers.yaml | 同名 | 逐字节相同 |
| config/controllers.yaml | 同名 | `controller_list` → MoveIt 2 `moveit_simple_controller_manager` 格式，控制器与关节不变 |
| config/single_lwr_robot.srdf | 同名 | 逐字节相同（链 `box → lwr_7_link`）。刚度被动关节在 ROS2 模型中默认不存在（`use_stiffness_joints:=false`），MoveIt 2 对这些未知关节只给出警告并忽略 |
| launch/planning_context.launch、planning_pipeline.launch.xml、ompl_planning_pipeline.launch.xml、trajectory_execution.launch.xml、*_moveit_controller_manager.launch.xml、sensor_manager.launch.xml、*_moveit_sensor_manager.launch.xml | launch/moveit_config.py | 这些 ROS1 子 launch 只做 `rosparam load`；MoveIt 2 由 `moveit_config.py` 组装为节点参数，取值来自同一组 config 文件 |
| launch/move_group.launch | launch/move_group.launch.py | 见"launch 对照" |
| launch/moveit_rviz.launch | launch/moveit_rviz.launch.py | 同上 |
| launch/moveit_planning_execution.launch | launch/moveit_planning_execution.launch.py | 同上（恢复为 include single_lwr.launch.py） |
| launch/demo.launch | launch/demo.launch.py | 同上 |
| launch/setup_assistant.launch | launch/setup_assistant.launch.py | 同上 |
| fake_moveit_controller_manager.launch.xml | launch/moveit_config.py（`fake_execution=True`） | MoveIt 2 无 MoveItFakeControllerManager：fake_controllers.yaml 的控制器改为 simple controller manager 下的 `/move_group/fake_full_lwr_controller`（见主要改动） |
| launch/default_warehouse_db.launch, warehouse.launch, warehouse_settings.launch.xml, run_benchmark_ompl.launch, joystick_control.launch | — | 未迁移，`db:=true` 显式拒绝（见需人工确认 2） |
| launch/moveit.rviz | 同名 | 不变 |

## 主要改动
- `move_group`/`moveit_rviz` 的 `robot_description` 按上层 launch 的 `robot_name`（默认 single_lwr_robot）展开，与 ROS1 读取全局 `/robot_description` 一致（前次移植固定为 single_lwr_robot）。
- `t1_limits` 采用叠加覆盖（ROS1 `rosparam load` 语义），不再整体替换。
- 去掉前次移植新增的 `use_sim_time`、`use_rviz` 参数与 `joint_states→/lwr/joint_states` remap：仿真时间由 single_lwr.launch.py 的 `SetParameter` 下发，`/joint_states` 由 joint_state_publisher 汇总（与 ROS1 相同）。
- **fake_execution**：ROS1 `fake_execution:=true` 把 move_group 的执行端换成 MoveItFakeControllerManager（fake_controllers.yaml 的 `fake_full_lwr_controller`，关节状态发布在 `/move_group/fake_controller_joint_states`）。MoveIt 2（Humble）只有 moveit_simple_controller_manager，因此 `fake_execution:=true` 时 move_group 使用由 fake_controllers.yaml 生成的 simple 控制器 `/move_group/fake_full_lwr_controller`（FollowJointTrajectory，`action_ns: follow_joint_trajectory`，default），而不再合入 controllers.yaml（`/lwr/joint_trajectory_controller`）：假执行不会发往机器人或仿真的控制器。该 action 由 demo.launch.py 的 mock 硬件提供；单独启动 `move_group.launch.py fake_execution:=true` 而没有该 server 时执行失败（不会退回实际控制器）。move_group.launch.py 按 t1_limits × fake_execution 声明 4 个带条件的 move_group 节点，恰好启动其中一个。
- **t1_limits 参数传递**：t1 分支的 robot_description_planning 改为写入临时参数文件传给节点。实验室 `t1_joint_limits.yaml` 有空的 `joint_limits:` 键（None），作为 dict 传入时 launch_ros 在构造 Node 时就报错，导致任何 move_group 启动（含 demo、`load_moveit:=true`）都失败；参数文件只在 t1 节点实际启动时被解析。`t1_limits:=true` 仍会失败（ROS1 roslaunch 同样无法设置该空键，静态检查记为 XFAIL）。
- MoveIt RViz 只给模型参数（robot_description、semantic、kinematics、planning），不再塞入整套 move_group 参数。

## launch 对照
### move_group.launch → move_group.launch.py
| 节点全名 | 可执行 | 参数 | remap/args/条件/respawn | 差异 |
|---|---|---|---|---|
| /move_group | moveit_ros_move_group/move_group | allow_trajectory_execution、max_safe_path_cost、jiggle_fraction 同名同值；`planning_scene_monitor/publish_*` → 顶层 `publish_*`；`capabilities` 改为 MoveIt 2 名称；全局 /robot_description_semantic、/robot_description_planning/*、/robot_description_kinematics/* 与 /move_group/* 管线参数 → 节点参数 | `--debug` if info；gdb 前缀 if debug；env DISPLAY；output=screen；respawn=false | MoveIt 2 参数布局；`MoveGroupExecuteService`→`MoveGroupExecuteTrajectoryAction`，`MoveGroupPickPlaceAction` 已移除；`max_safe_path_cost` int→double（值相同） |
launch 参数 debug、info、allow_trajectory_execution、fake_execution、max_safe_path_cost、jiggle_fraction、publish_monitored_planning_scene：名称与默认值不变。

### moveit_rviz.launch → moveit_rviz.launch.py
| /rviz_<anon> | rviz/rviz → rviz2/rviz2 | kinematics.yaml（私有参数）→ `robot_description_kinematics.*`；新增模型参数 | `-d moveit.rviz` if config；gdb 前缀 if debug | MoveIt 2 RViz 插件以节点参数读取模型 |

### moveit_planning_execution.launch → .launch.py
include single_lwr.launch.py：use_lwr_sim=sim、lwr_powered=!sim、ip、port、t1_limits、file=fril_init_file、load_moveit=true、controllers；参数名与默认值不变。差异同 single_lwr_launch 的仿真分支。

### demo.launch → demo.launch.py
| /joint_state_publisher | 同 | source_list `/move_group/fake_controller_joint_states`（同 ROS1） | — | 一致 |
| /move_group/controller_manager、spawner ×2 | controller_manager/ros2_control_node、spawner（新增） | mock_components/GenericSystem；`joint_state_controller`、`fake_full_lwr_controller`（joint_trajectory_controller，参数取自 single_lwr_robot 的 joint_trajectory_controller） | remap `/move_group/joint_states → /move_group/fake_controller_joint_states` | MoveIt 2（Humble）无 fake controller manager，由 /move_group 命名空间的 mock 硬件提供 `/move_group/fake_full_lwr_controller/follow_joint_trajectory` 与 `/move_group/fake_controller_joint_states`，与机器人（/lwr）隔离 |
| /robot_state_publisher | 同 | robot_description | respawn=true，output=screen | 一致 |
| /move_group、/rviz_<anon> | include | fake_execution=true、info=true、config=true | — | 同上 |
参数 db、debug 保留；`db:=true` 时（ROS1 include default_warehouse_db.launch）打印错误并 Shutdown，而不是静默忽略。demo 使用 OpaqueFunction 生成 mock 硬件描述，静态比对器无法解析，未纳入静态检查。

### setup_assistant.launch → .launch.py
`/moveit_setup_assistant`，args `--config_pkg=single_lwr_moveit`，debug 前缀：一致。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **fake_execution**：MoveIt 2 无 fake controller manager，`fake_execution:=true` 改用 `/move_group/fake_full_lwr_controller`（见主要改动），只有 demo.launch.py 提供该执行端。ROS1 fake 管理器不需要任何外部 server，按轨迹插值发布关节状态；ROS2 由 mock 硬件上的 joint_trajectory_controller 执行（轨迹按控制器插值与容差执行）。已运行验证：demo 中 move_group 加载 `/move_group/fake_full_lwr_controller`，发往该 action 的轨迹执行成功，`/joint_states` 到达目标。
2. **未迁移的 launch**：warehouse（mongo；MoveIt 2 Humble 无 `mongo_wrapper_ros.py`）、benchmark（`moveit_run_benchmark` 配置格式不同）、joystick（无 `moveit_joy.py`）不支持；`db:=true` 显式拒绝。需要时另行设计。
3. **MoveIt 2 规划适配器**：`request_adapters` 与 `start_state_max_bounds_error` 沿用前次移植的 MoveIt 2 默认集合（ROS1 ompl_planning_pipeline.launch.xml 的适配器名在 MoveIt 2 中已改名），请在规划行为上复核。

## 编译、运行与测试
```bash
ros2 launch single_lwr_moveit moveit_planning_execution.launch.py            # sim:=false 连接真机
ros2 launch single_lwr_moveit demo.launch.py
```
测试见 ../../../../migration_tests/README.md。
