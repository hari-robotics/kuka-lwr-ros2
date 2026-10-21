# kuka_teleop ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka/kuka_teleop`（实验室教程 "kuka steps" Part 2：Sigma7 遥操作 + hierarchical_controller）。KUKA 相关包，放在 `src/kuka-lwr/kuka_teleop`。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| package.xml, CMakeLists.txt | 同名 | catkin → ament_cmake；脚本以原文件名安装到 `lib/kuka_teleop`（ROS1 未安装，rosrun 在源码目录找到可执行文件） |
| scripts/kuka_teleop.py | 同名 | rospy → rclpy，Python 2 → Python 3 |
| scripts/joint_traj.py | 同名 | rospy → rclpy |
| launch_order.txt | 同名 | 原样复制（ROS1 命令说明；ROS2 对应命令见下文） |
| scripts/kuka_teleop_old.py, scripts/kuka_teleop (copy).py | — | 备份文件，不迁移 |
| include/kuka_teleop/（空目录） | — | 不迁移 |

## 主要改动
- **Python 3**：ROS1 脚本为 Python 2（`print "..."`、制表符与空格混用的缩进）。`print` 改为函数，缩进统一为制表符；运算逻辑不变。
- **线程模型**：rospy 为每个订阅者开一个线程，回调中的 `rospy.sleep(0.1)` 只阻塞该订阅者。ROS2 用 `MultiThreadedExecutor`，每个订阅放在独立的 `MutuallyExclusiveCallbackGroup` 中，效果相同；`rospy.sleep()` 按 ROS 时间 → `node.get_clock().sleep_for()`。
- **启动等待**：ROS1 在 `rospy.get_rostime() == 0` 时循环等待（仿真时间未到）；ROS2 循环不变，但 executor 先在后台线程运行，以便节点时钟收到 `/clock`。
- **节点名**：`LineTrajectory`（同 ROS1 `init_node`）；从 kuka_launch 的 `impedance_simulation.launch.py` 启动时名为 `kuka_teleop`（同 ROS1 launch）。
- **话题/QoS**：发布 `/lwr/hierarchical_controller/command`（PoseRPY，深度 1，同 `queue_size=1`）；订阅 `/sigma/sigma0/pose`（PoseStamped）、`/cartesian_position/pose/tooltip`（PoseRPY）、`/sigma/sigma0/buttons`（Joy），深度 10（rospy 默认无界队列）。
- **masterCallback**：ROS1 `data.pose = Rotate(data.pose, R)` 把 PoseRPY 赋给 PoseStamped 的 pose 字段（rospy 不检查类型）；rclpy 字段只接受 `geometry_msgs/Pose`，旋转结果改存局部变量 `data_pose`，后续用法不变。
- **joint_traj.py**：话题 `/lwr/joint_trajectory_controller/command` → `/lwr/joint_trajectory_controller/joint_trajectory`（Humble joint_trajectory_controller 的订阅话题，与 ultrasound_trajectory/pose_init 的处理相同）；`positions` 列表中的整数改为浮点数（rclpy 的 float64[] 不接受 int）；`rospy.Duration(2)` → `builtin_interfaces/Duration(sec=2)`。
- 两个脚本中未使用的 `import tf` / `tf.transformations` 删除（Humble 无此模块）。

## launch 对照
本包无 launch（ROS1 亦无）；`kuka-lwr/kuka_launch/launch/impedance_simulation.launch.py` 启动 `kuka_teleop.py`。

## 硬件通信部分
无（Sigma7 主手由外部 `sigma` 包驱动，见需人工确认 1）。

## 未迁移的原测试
无。

## 需人工确认
1. **Sigma7 驱动不在工作空间**：`rosrun sigma master`（发布 `/sigma/sigma0/pose`、`/sigma/sigma0/buttons`）在 ROS1 工作空间中也没有源码，ROS2 下需要相应驱动或桥接。
2. **坐标旋转**：`theta = -45°`（注释："-45 for simulation, 135 for real robot"），与 ROS1 相同，实机使用前需按注释修改。
3. **工具长度不一致**：目标取自 `/cartesian_position/pose/tooltip`（0.252 m 工具），而 hierarchical_controller 的 `update()` 用 0.275 m；按下按钮后机械臂会沿工具轴移动约 2.3 cm（ROS1 相同，见 kuka_controllers NOTES）。

## 编译、运行与测试
```bash
colcon build --packages-select kuka_teleop
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=hierarchical_controller
ros2 run kuka_teleop kuka_teleop.py            # 仿真时加 --ros-args -p use_sim_time:=true
```
运行时测试（模拟 Sigma7 话题）见 `../../../migration_tests/tutorial_packages/README.md`。
