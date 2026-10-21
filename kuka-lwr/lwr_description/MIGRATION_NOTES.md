# lwr_description ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/lwr_description`（实验室版本；本包与上游 ROS1 相同，实验室未修改）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| model/kuka_lwr.urdf.xacro | 同名 | 几何、惯量、关节限值、宏结构不变（含 ROS1 的重复 `<origin>` 与 `<geometry name=...>`）；新增 ros2_control 所需 xacro 参数；刚度元数据关节改为 `use_stiffness_joints` 可选；transmission 宏 → ros2_control 宏 |
| model/kuka_lwr.gazebo.xacro | 同名 | `liblwr_hw_gazebo_plugin.so`（Classic ModelPlugin）→ `gz_ros2_control-system`（Fortress），命名空间 `/${name}` 不变；Classic 材质脚本 `Gazebo/Orange`、`Gazebo/Grey` 删除 |
| model/kuka_lwr.transmission.xacro | 同名 | 逐字节保留，ROS2 模型不再包含 |
| — | model/kuka_lwr.ros2_control.xacro | 替代 transmission：7 个关节的 position/effort/stiffness/damping/set_point 指令接口与 30 个笛卡尔资源，关节顺序不变 |
| model/utils.xacro, kuka_materials.xacro, meshes/ | 同名 | 不变 |

前次移植加入的 `model/tool.xacro`、`lwr_with_tool.xacro`、`lwr_with_tool.urdf` 以及宏内强制挂载的工具已删除：工具属于 `kuka_ws/src/kuka/lwr_robot_tool`，已作为独立包 `lwr_robot_tool` 移植。

## 主要改动
- **xacro 写法**：`<cylinder_inertia_def>` 等宏调用补上 `xacro:` 前缀（ROS2 xacro 不再接受无前缀宏）。
- **硬件选择**：`ros2_control_hardware_type` = gazebo / real / none；none（纯显示）不生成 `<ros2_control>`；real 的插件由 `ros2_control_hardware_plugin`（默认 `lwr_hw/LWRSystemHardware`）给出；port/ip/file/root/tip 写入 `<hardware>` 参数（ROS1 为 lwr_hw 节点参数与 `/lwr/root`、`/lwr/tip`）。
- **刚度元数据关节**：ros2_control / Gazebo Fortress 下这些无质量的虚拟关节无法作为真实关节存在，改为可选（默认关闭，gazebo 下开启会报错）；刚度限值（0–5000、20000/s）由 lwr_hw 按同值实现。
- **package.xml**：保留 `<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>`，ros_gz_sim 的 `gz_sim.launch.py` 读取它来设置模型路径（self-check 的"gazebo_ros 依赖残留"为有意保留）。

## launch 对照
本包无 launch 文件。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
1. **Gazebo 材质**：Classic 的 `Gazebo/Orange` 等材质脚本在 Fortress 不可用，已删除，外观使用 URDF visual 材质。

## 编译、运行与测试
```bash
colcon build --packages-select lwr_description
xacro $(ros2 pkg prefix single_lwr_robot)/share/single_lwr_robot/robot/single_lwr_robot.urdf.xacro ros2_control_hardware_type:=none
```
