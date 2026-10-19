# lwr_robot_tool ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka/lwr_robot_tool`。`single_lwr_launch/launch/single_lwr_tool.launch`（kuka-lwr 实验室版本）的全局 `robot_description` 来自本包，因此随 kuka-lwr 一起移植。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| urdf/tool.xacro | 同名 | 逐字节复制 |
| urdf/lwr_with_tool.xacro | 同名 | 逐字节复制（ROS1 launch 不使用它；与 lwr_with_tool.urdf 的工具几何不同，见需人工确认 1） |
| urdf/lwr_with_tool.urdf | 同名 | launch 实际使用的文件（xacro 生成物）。内容逐行保留，只替换 Gazebo/传动部分，见"主要改动" |
| meshes/collision, meshes/visual, launch/ | 同名 | ROS1 中为空目录，原样保留 |
| CMakeLists.txt, package.xml | 同名 | catkin → ament_cmake；安装 urdf/launch/meshes；rviz → rviz2；新增 exec_depend lwr_description（URDF 引用其 meshes 与宏） |

## 主要改动
`lwr_with_tool.urdf` 与 ROS1 的差异仅为：
- 文件头新增与 `lwr_description/model/kuka_lwr.urdf.xacro` 同名同默认值的 xacro 参数（`ros2_control_hardware_type`、`fri_port`、`fri_ip`、`hw_root` 等），并 include `kuka_lwr.gazebo.xacro`、`kuka_lwr.ros2_control.xacro`（ROS1 launch 本就对它调用 xacro）。
- `<gazebo><plugin liblwr_hw_gazebo_plugin.so>`、8 个 `<gazebo reference>` 与 7 个 `SimpleTransmission` → `kuka_lwr_gazebo` / `kuka_lwr_ros2_control` 宏（gz_ros2_control + ros2_control），与 lwr_description 的处理相同。
- 7 个 `*_stiffness` 元数据关节与 dummy link 用 `use_stiffness_joints` 包裹（默认不生成），与 lwr_description 相同。
- `world_joint` 只在 `ros2_control_hardware_type:=gazebo` 时不生成（见需人工确认 2）。
- 其余（工具几何 0.0275/0.055/0.116 偏移、`link_7` 内的 `<sensor>`）不变。

## launch 对照
本包无 launch 文件（ROS1 的 launch/ 为空）。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **工具几何的两个来源**：ROS1 launch 使用的 `lwr_with_tool.urdf` 中 tool 偏移 0.0275 m、半径 0.01，tool_ee 偏移 0.055 m、长 0.11；而 `tool.xacro`/`lwr_with_tool.xacro` 中偏移为 0、半径 0.02、tool_ee 长 0.2。ROS2 保持两者各自不变，launch 仍使用 .urdf。
2. **lwr_base_link 有两个父关节**：`world_lwr_base_joint`（宏生成）与 `world_joint`（装配文件）都以 world 为父、lwr_base_link 为子，变换相同（ROS1 原样）。urdfdom/KDL 接受；Gazebo Fortress 的 URDF→SDF 转换会把 lwr_base_link 以下整棵子树生成两次并拒绝创建模型（`link with name[lwr_base_link] already exists`），因此 Gazebo 描述中不生成 `world_joint`。显示与真机分支保持原样。

## 编译、运行与测试
```bash
colcon build --packages-select lwr_robot_tool
xacro $(ros2 pkg prefix lwr_robot_tool)/share/lwr_robot_tool/urdf/lwr_with_tool.urdf ros2_control_hardware_type:=none
```
由 `single_lwr_launch/launch/single_lwr_tool.launch.py` 使用，测试见 ../../../migration_tests/single_lwr_launch。
