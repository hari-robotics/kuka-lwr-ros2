# single_lwr_example ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/single_lwr_example/single_lwr_example`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml | 同名 | 元包：format 2 → 3，catkin → ament_cmake，依赖成员包列表与 ROS1 相同（kuka_lwr、single_lwr_robot、single_lwr_moveit；前次移植改为 build depend 并加了 single_lwr_launch、漏了 kuka_lwr，已恢复） |
| CMakeLists.txt | 同名 | catkin_metapackage() → ament_package() |

## 主要改动
构建系统替换，无代码。

## launch 对照
无 launch 文件。

## 硬件通信部分
无。

## 未迁移的原测试
无。

## 需人工确认
无。

## 编译、运行与测试
`colcon build --packages-select single_lwr_example`
