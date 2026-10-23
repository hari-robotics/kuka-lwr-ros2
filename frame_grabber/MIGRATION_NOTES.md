# frame_grabber ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/frame_grabber`。从外部相机/图像采集卡（OpenCV `VideoCapture(0)`）读取图像并发布，`cameratoROS_cropped.py` 裁剪出 TELEMED 超声图像。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| src/cameratoROS.py | 同名 | rospy → rclpy；发布 `/image_topic`（depth 1），节点 `image_topic_node`，30 Hz，显示窗口 `frame` |
| src/cameratoROS_cropped.py | 同名 | rospy → rclpy；发布 `/frame_grabber/image_cropped_topic`（depth 2），节点 `image_topic_node_cropped`，10 Hz（ROS1 为 30 Hz），裁剪 `[95:545, 410:860]` 后缩放到 480×480，`header.stamp` 为当前时间 |
| CMakeLists.txt, package.xml | 同名 | catkin → ament_cmake；两个脚本以原文件名安装到 `lib/frame_grabber` |
| src/cameratoROS_cropped.pyc, src/__pycache__/, build/, .vscode/, include/frame_grabber/（空） | — | 编译缓存、构建产物与编辑器配置不迁移（空 include 目录以 .gitkeep 保留） |

## 主要改动
- **相机与图像处理不变**：`VideoCapture(0)`、分辨率设置（1280×800 / 1280×720）、裁剪坐标、缩放、`imshow`/`waitKey`、`cv2_to_imgmsg(..., "bgr8")` 原样保留。
- **节点结构**：`roslib.load_manifest` 删除（ROS2 无对应）；ROS1 先创建发布者再 `init_node`，ROS2 必须先创建节点再创建发布者（顺序调整，话题与 QoS 不变）。
- **主循环**：`while not rospy.is_shutdown()` → `while rclpy.ok()`，循环体不变；`rospy.Rate(30)` → `node.create_rate(30)`，需后台线程 spin（skill python.md §7），退出时 `try/finally` 关闭节点并等待线程。
- **时间戳**：`rospy.Time.now()` → `node.get_clock().now().to_msg()`。
- **cameratoROS_cropped.py 的实机改动**（ROS1 没有）：读不到帧时（相机未连接或断开）每 2 s 警告一次并跳过，不再因裁剪 `None` 而崩溃；显示窗口 `frame` 加了鼠标回调，移动鼠标时打印像素坐标，便于调整裁剪区域；频率由 30 Hz 降为 10 Hz；Ctrl+C / SIGTERM 时释放相机、关闭窗口后退出。
- **构建**：ROS1 的脚本未安装，由 `rosrun` 在源码目录中查找；ROS2 用 `install(PROGRAMS)` 安装，`ros2 run frame_grabber cameratoROS_cropped.py`。shebang 由 `python` 改为 `python3`。依赖：rclpy、std_msgs、sensor_msgs、cv_bridge（ROS1 CMake 中 find_package 了 sensor_msgs、cv_bridge，package.xml 未声明）。

## launch 对照
本包无 launch（ROS1 亦无）。

## 硬件通信部分
相机访问全部经 OpenCV，代码原样保留，仅替换 ROS 接口行。

## 未迁移的原测试
无。

## 需人工确认
1. **硬件实测**：✅ `cameratoROS_cropped.py` 已在连接图像采集卡的实机上验证，图像与裁剪区域正常。`cameratoROS.py` 未改动：无相机时 `cap.read()` 返回 `None`，脚本在 `frame.shape` 处抛出异常并退出，与 ROS1 相同。

## 编译、运行与测试
```bash
colcon build --packages-select frame_grabber
ros2 run frame_grabber cameratoROS_cropped.py
```
