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
| launch/default_warehouse_db.launch, warehouse.launch, warehouse_settings.launch.xml, run_benchmark_ompl.launch, joystick_control.launch, fake_moveit_controller_manager.launch.xml | — | 未迁移（见需人工确认 2） |
| launch/moveit.rviz | 同名 | 不变 |

## 主要改动
- `move_group`/`moveit_rviz` 的 `robot_description` 按上层 launch 的 `robot_name`（默认 single_lwr_robot）展开，与 ROS1 读取全局 `/robot_description` 一致（前次移植固定为 single_lwr_robot）。
- `t1_limits` 采用叠加覆盖（ROS1 `rosparam load` 语义），不再整体替换。
- 去掉前次移植新增的 `use_sim_time`、`use_rviz` 参数与 `joint_states→/lwr/joint_states` remap：仿真时间由 single_lwr.launch.py 的 `SetParameter` 下发，`/joint_states` 由 joint_state_publisher 汇总（与 ROS1 相同）。
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
| /joint_state_publisher | 同 | source_list `/move_group/fake_controller_joint_states` → `/lwr/joint_states` | — | MoveIt 2（Humble）无 fake controller manager，假执行改由 ros2_control `mock_components/GenericSystem` + joint_trajectory_controller 完成，额外启动 /lwr/controller_manager 与 spawner |
| /robot_state_publisher | 同 | robot_description | respawn=true，output=screen | 一致 |
| /move_group、/rviz_<anon> | include | fake_execution=true、info=true、config=true | — | 同上 |
参数 db、debug 保留；`db:=true` 的 warehouse 未迁移。demo 使用 OpaqueFunction 生成 mock 硬件描述，静态比对器无法解析，未纳入静态检查。

### setup_assistant.launch → .launch.py
`/moveit_setup_assistant`，args `--config_pkg=single_lwr_moveit`，debug 前缀：一致。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **fake_execution**（`TODO(ros2-migration)`）：参数保留但 MoveIt 2 无对应实现，demo 以 mock 硬件代替。
2. **未迁移的 launch**：warehouse（mongo）、benchmark、joystick 在 MoveIt 2 中结构不同，按需另行转换。
3. **MoveIt 2 规划适配器**：`request_adapters` 与 `start_state_max_bounds_error` 沿用前次移植的 MoveIt 2 默认集合（ROS1 ompl_planning_pipeline.launch.xml 的适配器名在 MoveIt 2 中已改名），请在规划行为上复核。

## 编译、运行与测试
```bash
ros2 launch single_lwr_moveit moveit_planning_execution.launch.py            # sim:=false 连接真机
ros2 launch single_lwr_moveit demo.launch.py
```
测试见 ../../../../migration_tests/README.md。
