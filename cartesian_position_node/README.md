# cartesian_position_node (ROS 2 Humble)

从 `/lwr/joint_states` 和当前 URDF 计算法兰、TCP 的笛卡尔位姿与速度。
移植后的包位于 `src/cartesian_position_node`；根目录同名目录保留 ROS 1
源码，使用 `COLCON_IGNORE` 排除，避免重复包。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cartesian_position_node single_lwr_launch --symlink-install
source install/setup.bash
```

如果从干净的工作空间构建，先运行
`colcon build --packages-up-to cartesian_position_node single_lwr_launch --symlink-install`。

现有启动文件默认启动此节点：

```bash
ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=true use_rviz:=true
```

已启动机器人时可单独运行（避免重复启动同一节点）：

```bash
ros2 run cartesian_position_node cartesian_position_node
```

Gazebo 已启动时使用模拟时间：

```bash
ros2 launch cartesian_position_node cartesian_position.launch.py use_sim_time:=true
```

在主启动命令加入 `use_cartesian_position:=false` 可关闭自动启动。
该节点只订阅状态并发布计算结果，不发送运动命令。

| 输出话题 | ROS 2 消息类型 | 内容 |
| --- | --- | --- |
| `/cartesian_position/pose/end_effector` | `lwr_controllers/msg/PoseRPY` | 法兰位姿 |
| `/cartesian_position/pose/tooltip` | `lwr_controllers/msg/PoseRPY` | TCP 位姿 |
| `/cartesian_position/vel/end_effector` | `geometry_msgs/msg/Twist` | 法兰速度 |
| `/cartesian_position/vel/tooltip` | `geometry_msgs/msg/Twist` | TCP 速度 |

全部结果表达在 `root_link` 坐标系中：默认 `lwr_base_link`。
位置单位为 m，RPY 为 rad，线速度为 m/s，角速度为 rad/s。
保留原输出消息类型，因此消息不含时间戳和坐标系字段；发布由输入
JointState 触发，不对 ROS 时间或输入时间戳进行数值微分。

| 启动参数 | 默认值 |
| --- | --- |
| `joint_states_topic` | `/lwr/joint_states` |
| `robot_description_topic` | `/robot_description` |
| `robot_description` | 空；订阅模型话题 |
| `root_link` | `lwr_base_link` |
| `end_effector_link` | `lwr_7_link` |
| `tooltip_link` | `tcp` |
| `velocity_filter_alpha` | `0.7` |

这些配置参数在启动后只读。可通过 `--ros-args -p 参数:=值` 设置；
独立 launch 文件暴露除 URDF 字符串之外的全部参数以及 `use_sim_time`。
例如：

```bash
ros2 run cartesian_position_node cartesian_position_node --ros-args \
  -p root_link:=world -p velocity_filter_alpha:=1.0
```

节点支持直接传入 `robot_description` 字符串；否则等待模型话题。
模型订阅使用 reliable/transient-local，支持在 robot_state_publisher
启动后获取模型；参见 [ROS 2 QoS 接口](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html)。
关节状态订阅使用 SensorDataQoS，兼容 best-effort 和 reliable 发布者。
工具必须通过固定连接与法兰共享同一组驱动关节。

与 ROS 1 实现相比，节点按关节名称匹配，允许消息包含额外关节，拒绝
缺失关节、重名、非有限值及数组长度错误。未提供 velocity 时仅发布
位姿，并重置速度滤波；不推测速度或发布假零值。
位姿和完整六维速度均由 URDF/KDL 计算，不再使用硬编码的工具长度
和解析 Jacobian。当前模型法兰到 TCP 的偏移为 0.1985 m。
法兰速度不滤波；TCP 线速度保留一阶滤波，首帧使用实际速度，后续为
`alpha * 当前值 + (1 - alpha) * 上一滤波值`，`alpha=1` 关闭滤波。
角速度不滤波。

测试覆盖模型话题的延迟订阅、模型参数、关节乱序、异常输入、
缺失速度、滤波，以及用位姿数值导数校验法兰/TCP 的六维速度：

```bash
colcon test --packages-select cartesian_position_node
colcon test-result --test-result-base build/cartesian_position_node --verbose
```

本次验证：节点与主启动包编译成功；11 项节点测试及 16 项机器人模型
测试通过。Gazebo Fortress 无界面联动运行收到全部四个输出话题；
零位法兰高度为 1.1785 m，TCP 高度为 1.377 m（相对默认根坐标）。
