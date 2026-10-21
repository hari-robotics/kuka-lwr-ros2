# kuka_lwr ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/kuka_lwr`（实验室版本）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml | 同名 | 元包：format 2 → 3，catkin → ament_cmake，依赖成员包列表不变 |
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
`colcon build --packages-select kuka_lwr`
