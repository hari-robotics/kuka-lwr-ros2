# sriforcesensor ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/sriforcesensor`。SRI M812X 六维力传感器（TCP，`192.168.0.108:4008`）驱动与工具重力补偿脚本。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| src/TCPClient.cpp, src/TCPClient.h | 同名 | **逐字节复制**（传感器 TCP 通信、AT 指令、帧解析） |
| src/sriforcesensor.cpp | 同名 | 只替换 ROS 接口行，见"硬件通信部分" |
| src/ANN/tool_comp_ros.py | 同名 | rospy → rclpy（工具重力补偿：`/Force` + 末端姿态 → `/Force_calibrated`） |
| src/ANN/record_calib.py | 同名 | rospy → rclpy（记录标定数据到 CSV） |
| src/ANN/tool_comp.py | 同名 | 逐字节复制（离线训练脚本，不含 ROS；内含原作者机器上的绝对路径） |
| src/ANN/realProbe.csv, realProbe.sav, .sav, Previous calibration/ | 同名 | 标定数据与模型，原样保留 |
| CMakeLists.txt, package.xml | 同名 | catkin → ament_cmake；保留共享库 `TCPClient2` 与可执行 `forcesensor`；Python 脚本以原文件名安装到 `lib/sriforcesensor` |
| src/old.cpp | — | ROS1 CMake 未编译的旧版本，不迁移 |
| src/CMakeLists.txt, src/cmake-build-debug/, .idea/ | — | CLion 工程与构建产物（src/CMakeLists.txt 引用不存在的 talker.cpp），不迁移 |
| src/ANN/.claude/ | — | 编辑工具配置，不迁移 |

## 主要改动
- **节点**：可执行 `forcesensor`，节点名 `talker`（同 ROS1 `ros::init`），发布 `/Force`（`geometry_msgs/msg/Twist`，depth 1000），1000 Hz 主循环。
- **构建**：C++17（rclcpp 要求；ROS1 为 C++14）。
- **Python 脚本**：ROS1 未安装这些脚本，而是由 `rosrun` 在源码目录中查找可执行文件；ROS2 用 `install(PROGRAMS)` 安装，`ros2 run sriforcesensor <脚本名>.py`。主循环结构不变；`rospy.Rate` → `node.create_rate()`，需要后台线程 spin（skill python.md §7），退出时 `try/finally` 关闭节点并等待线程；`rospy.sleep(1)` → `node.get_clock().sleep_for(Duration(seconds=1))`（与 rospy 一样按 ROS 时间）；`anonymous=True` 在 ROS2 中无对应，节点名不再带随机后缀；订阅 QoS depth 10（rospy 默认无界队列）。`tool_comp_ros.py` 的 shebang 由 `python` 改为 `python3`。
- **record_calib.py 参数**：ROS1 在导入时读取全局参数 `/file_name`（`rosparam set file_name ...`）；ROS2 无全局参数，改为节点参数 `file_name`，在节点创建后读取：`ros2 run sriforcesensor record_calib.py --ros-args -p file_name:=probe_data2.csv`。未设置时与 ROS1 一样抛出 `KeyError`。
- **相对路径**：`tool_comp_ros.py` 打开当前目录下的 `realProbe.sav`，`record_calib.py` 写当前目录，需在 `src/ANN` 等含模型文件的目录中运行（ROS1 相同）。

## launch 对照
本包无 launch（ROS1 亦无）。

## 硬件通信部分
- 逐字节复制：`src/TCPClient.cpp`、`src/TCPClient.h`（含 `exponentialSmoothing`）。校验：`cmp kuka_ws/src/sriforcesensor/src/TCPClient.cpp kuka_ros2_ws/src/sriforcesensor/src/TCPClient.cpp`。
- `src/sriforcesensor.cpp` 与 ROS1 的差异仅为：头文件（`rclcpp`、`*_msgs/msg/*.hpp`）、`ros::init`/`NodeHandle` → `rclcpp::init`/`rclcpp::Node("talker")`、`advertise` → `create_publisher`、`ros::Rate` → `rclcpp::Rate`、`ros::ok()` → `rclcpp::ok()`、`ros::Time::now()` → `n->now()`（同一位置）、`publish`、`ros::spinOnce()` → `rclcpp::spin_some(n)`（同一位置），以及打印 `Forcevalue.linear`：ROS2 消息没有 `operator<<`，改用 `geometry_msgs::msg::to_yaml()`（块格式，与 ROS1 输出相同）。连接、`ConfigSystem()` 的 AT 指令序列、解耦矩阵、偏置、滤波系数、绕 z 轴 −20° 旋转、循环顺序均未改动。
- `signal(SIGINT, sig_exit)` 仍在节点初始化之后注册，覆盖 rclcpp 的 SIGINT 处理（ROS1 中覆盖 roscpp 的处理），Ctrl+C 时关闭 TCP 连接并退出，行为相同。

## 未迁移的原测试
无。

## 需人工确认
1. **硬件实测**：只做了编译与无硬件启动检查。无传感器时：连接失败（"connect failed ... Force sensor connection failed"），随后 `ConfigSystem()` 向未连接的 socket 写入，进程因 SIGPIPE 退出——这是原通信代码的行为，ROS1 相同，未修改。需在连接传感器的机器上验证 `/Force` 数据。
2. **ANN 脚本依赖**：`tool_comp_ros.py`、`tool_comp.py` 需要 scikit-learn、pandas、seaborn、matplotlib（ROS1 package.xml 同样未声明）；本机未安装 scikit-learn，`realProbe.sav` 是 pickle 的 sklearn 模型，需与训练时相近的 sklearn 版本才能加载。

## 编译、运行与测试
```bash
colcon build --packages-select sriforcesensor
ros2 run sriforcesensor forcesensor
cd src/sriforcesensor/src/ANN && ros2 run sriforcesensor tool_comp_ros.py
```
