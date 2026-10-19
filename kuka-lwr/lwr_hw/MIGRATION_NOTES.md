# lwr_hw ROS1 → ROS2 Humble 迁移说明

基准：`kuka_ws/src/kuka-lwr/lwr_hw`（实验室版本，含 10.22 的外部力矩发布）。

## 文件对照
| ROS1 | ROS2 | 说明 |
|---|---|---|
| include/fri/friComm.h, friremote.h, friudp.h | 同名 | 逐字节复制 |
| src/fri/friremote.cpp, friudp.cpp | 同名 | 逐字节复制 |
| include/lwr_hw/lwr_hw.h, src/lwr_hw.cpp | 同名 | RobotHW → `hardware_interface::SystemInterface`；状态/指令缓冲区、reset()、模式选择、限幅公式与 ROS1 相同；实验室新增的 `joint_est_ext_trq_` 与 `/est_ext_torque` 发布见"主要改动" |
| include/lwr_hw/lwr_hw_fri.hpp | 同名 | 通信逻辑原样（见"硬件通信部分"），只换 ROS 接口行 |
| include/lwr_hw/lwr_hw_fril.hpp | 同名 | 同上（FRIL 后端，可选编译） |
| src/lwr_hw_fri_node.cpp | 同名 | ROS1 节点 main（参数、AsyncSpinner、ControllerManager 循环）→ 插件导出；循环由 `ros2_control_node` 提供 |
| src/lwr_hw_fril_node.cpp | 同名 | 同上 |
| include/lwr_hw/lwr_hw_gazebo.hpp, src/lwr_hw_gazebo_plugin.cpp | 同名 | Gazebo Classic ModelPlugin → `gz_ros2_control::GazeboSimSystemInterface`（Fortress）；仿真律不变 |
| include/cartesian_hardware_interface/*.h | 同名 | ROS1 自定义 RobotHW 接口在 ROS2 无对应，保留为 `CommandInterface` 别名 |
| launch/lwr_hw.launch | launch/lwr_hw.launch.py | 见"launch 对照" |
| krl/ros_control.src, ros_control.dat | 同名 | .dat 逐字节；.src 为现场网络配置（见需人工确认 1） |
| — | lwr_hw_plugins.xml, lwr_hw_gazebo_plugins.xml, lwr_hw_fril_plugins.xml | pluginlib 描述（ROS2 必需） |

## 主要改动
- **节点结构**：ROS1 `lwr_hw_fri_node` 自己创建 ControllerManager 并以 `1/getSampleTime()` 循环 read/update/write；ROS2 由 `ros2_control_node` 加载插件 `lwr_hw/LWRSystemHardware`（别名 `lwr_hw/LWRHWFRI`）。`on_init` 对应 `create()`，`on_activate` 调用 ROS1 `init()`，`on_deactivate/on_shutdown`/析构调用 `stop()`（= ROS1 析构：停止并 join 通信线程）。
- **参数**：ROS1 节点参数 `name/port/ip/file`（默认 lwr / 49939 / 192.168.0.10 / ""）与全局 `/lwr/root`、`/lwr/tip` → ros2_control `<hardware>` 参数（由 xacro 写入），默认值不变。
- **模式切换**：ROS1 `prepareSwitch()`/`doSwitch(start_list, stop_list)` → `prepare_command_mode_switch`/`perform_command_mode_switch`。接口族按 ROS1 的注册关系：`position` → PositionJointInterface → JOINT_POSITION(10)；`effort/stiffness/damping/set_point` → EffortJointInterface → JOINT_IMPEDANCE(30)；笛卡尔资源 → PositionCartesianInterface → CARTESIAN_IMPEDANCE(20)；同一次切换混用多族即拒绝。只停不启时按 ROS1 默认切回 JOINT_POSITION。
- **限幅**：ROS1 joint_limits_interface 句柄逐项复刻（effort/position 的 Saturation 或 SoftLimits 公式；刚度 0–5000、速率 20000/s；阻尼无限幅；切换时只重置 position 历史值）。刚度限值取 URDF 刚度关节（`use_stiffness_joints:=true` 时）或 xacro 中的同值常量（ros2_control 下刚度元数据关节不展开）。
- **外部力矩（实验室 10.22）**：`joint_est_ext_trq_` 由 FRI `getMsrEstExtJntTrq()` / FRIL `GetEstimatedExternalJointTorques()` 在 read() 中填写（与 ROS1 同一位置），并在 `enforceLimits()` 末尾每周期发布到全局 `/est_ext_torque`（`std_msgs/Float64MultiArray`，depth 10），消息构造代码逐行保留。ROS1 `Publisher::publish()` 在没有订阅者时直接返回、不序列化；rclcpp 每次都序列化，加上每周期增长的 layout，实测 Gazebo 实时率 5 分钟内从 0.45 降到 0.20（无任何订阅者）。因此只在有订阅者时调用 publish（与 ROS1 语义相同），修改后实时率稳定在 1.0；有订阅者时（如 joint_impedance_controller）开销照旧增长，与 ROS1 相同。ROS1 用 LWRHW 内的 `ros::NodeHandle` 在 `create()` 末尾 advertise；ROS2 在 `on_init()` 末尾用硬件内部节点 `/<name>/<name>_hw` 创建发布者（Gazebo 插件不走 on_configure，故放在 on_init），因此仿真中同样发布（全 0，与 ROS1 Gazebo 相同）。
- **时间**：`period.toSec()` → `period.seconds()`。
- **构建**：catkin → ament_cmake；FRIL 后端由 `-DLWR_HW_WITH_FRIL=ON` 显式启用；`boost::shared_ptr<friRemote/std::thread>` → `std::shared_ptr`；FRIL 头文件的 `boost::shared_ptr<FastResearchInterface>` 保持原样（硬件层代码）。
- **按 ROS1 恢复（2026-10-19）**：前次规范化保留的两处非 ROS1 行为已撤回：激活硬件时执行 semantic zero、semantic zero 把 set_point 设为测量位姿；`enforceLimits()` 对非有限指令返回 ERROR。现与 ROS1 相同（见需人工确认 4）。
- **构建**：`lwr_hw_gazebo_plugin` 不再放入导出目标集合（它链接 ignition-gazebo6，导出后下游 lwr_controllers 无法链接）。
- **撤回的前次移植改动**：异步快照通信线程（先收后发、100 ms 接收超时、generation/hold 机制、KRL 策略重映射）、FRI 库中的 `SetReceiveTimeout`/异常替代 `exit(1)`/`seqCount` 初始化、JOINT_EFFORT 自动模式、set_point 与阻尼截断、FRIL 的 `StopRobot()` 停机与笛卡尔拒绝、源码包内测试。均违背"硬件通信逻辑不动/不顺手修 bug"，已恢复 ROS1 行为，问题点列入"需人工确认"。

## launch 对照
### lwr_hw.launch → lwr_hw.launch.py
| 节点全名 | 可执行 ROS1→ROS2 | 参数 | remap/args/条件/respawn | 差异 |
|---|---|---|---|---|
| /lwr_hw → /<name>/controller_manager | lwr_hw/lwr_hw_fri_node → controller_manager/ros2_control_node | name/port/ip/file → `<hardware>` 参数；新增 robot_description、controllers_file、update_rate | required=true → on_exit=Shutdown()；output=screen；respawn=false | ROS1 节点自带 ControllerManager 循环，ROS2 由 ros2_control_node 加载 FRI 插件；未设 name，使 launch_ros 将 robot_description 写到 `/**` 供控制器读取。ROS1 从参数服务器读取模型与控制器参数，ROS2 需 `description_file`、`controllers_file`（新增参数）；控制循环频率由 `update_rate` 给定（ROS1 为 1/getSampleTime()） |
launch 参数 name/port/ip/file_with_path 名称与默认值不变（静态比对一致，见 ../../../migration_tests/lwr_hw）。

## 硬件通信部分
- 逐字节复制：`include/fri/friComm.h`、`friremote.h`、`friudp.h`、`src/fri/friremote.cpp`、`friudp.cpp`、`krl/ros_control.dat`、`doc/`。
- `lwr_hw_fri.hpp` 与 ROS1（实验室版）的差异仅为：`init/read/write/doSwitch` 签名与返回值（ros2_control 接口）；`on_configure` 从 `<hardware>` 参数读取 port/ip（原在节点 main 中）；析构体移入 `stop()` 以便生命周期调用；`read()` 在硬件未激活（device_ 为空）时直接返回（Humble 对 INACTIVE 硬件也调用 read）；`write()` 先 `pollEmergencyStop()`，`enforceLimits` 返回值检查；`doSwitch` 的"semantic zero"抽到基类 `semanticZero()`；`boost::shared_ptr` → `std::shared_ptr`；新增两行日志。`KRCCommThreadCallback`、`startFRI`、`stopFRI`、各控制策略的 `doXxxControl` 调用、读写顺序与 ROS1 完全相同。
- `lwr_hw_fril.hpp` 同样只换签名/参数来源/返回值，`StartRobot/StopRobot/WaitForKRCTick/IsMachineOK` 调用与顺序不变。
- 校验：`diff -u --strip-trailing-cr kuka_ws/src/kuka-lwr/lwr_hw/include/lwr_hw/lwr_hw_fri.hpp kuka_ros2_ws/src/kuka-lwr/lwr_hw/include/lwr_hw/lwr_hw_fri.hpp`。

## 未迁移的原测试
无（ROS1 包无测试）。前次移植放在 `test/` 的 `test_lwr_fri_thread.cpp` 测的是已撤回的异步实现，原样移到 `../../../migration_tests/lwr_hw/legacy_tests/`，仅供参考。

## 需人工确认
1. **krl/ros_control.src**：`friSetup("192.168.10.1", 49938, 49938)`（现场网络，实验室版本）。`lwr_hw.launch.py` 的默认端口按 lwr_hw.launch 为 49939；`single_lwr(_tool).launch.py` 按实验室版本默认 49938 / 192.168.10.2。
2. **控制频率**（`TODO(ros2-migration)` in lwr_hw_fri.hpp）：ROS1 以 `1/getSampleTime()` 运行；ROS2 由 launch 参数 `update_rate`（默认 500 Hz，对应 `friOpen(2)`）给定。若 KRL 采样周期不同须同步修改，否则按步数推进的控制器（如 ComputedTorque 的 `step_`）时间尺度会变化。
3. **急停**（`TODO(ros2-migration)` in lwr_hw.h）：ROS1 在 `/lwr/emergency_stop` 为真时调用 `manager.update(..., resetControllers)` 重启控制器（且 `resetControllers` 未初始化，为 UB）；ros2_control 硬件侧无法重置控制器，当前在为真期间每周期执行 semantic zero（保持测量位姿、附加力矩清零）。释放后控制器从其内部状态继续，与 ROS1 重启控制器不同。
4. **set_point 与启动时的指令（已恢复 ROS1，安全相关）**：ROS1 切到 JOINT_IMPEDANCE 时不重置 set_point（初值 0），控制器启动前发送的指令为 reset() 值（位置 0）。ROS2 现与之相同：若启动的控制器只写 effort（如 gravity_compensation），真机会以刚度 1000 被拉向零位。实验室的 joint_impedance_controller 每周期写 set_point = 测量位姿，不受影响。上真机前请确认。
5. **gravity_compensation 在真机上的语义**：按 ROS1 映射，只写 effort 的控制器进入 JOINT_IMPEDANCE（刚度为指令值，默认 1000），并非零刚度；前次移植的"JOINT_EFFORT 零刚度"已撤回，因为 ROS1 FRI `doSwitch` 不会为 JOINT_EFFORT 向 KRL 发送策略。需要零刚度重力补偿时应由控制器写 stiffness。
6. **通信健壮性（ROS1 原有，未修改）**：`friUdp::Recv` 阻塞无超时，机器人停发数据时 `stop()`/关停时 join 会挂起；`stopFRI()` 无限等待 KRL 确认，期间阻塞 ros2_control 更新线程；通信线程与 `read()/write()` 无锁共享 friRemote 缓冲区；socket/bind 失败时 `exit(1)` 会结束整个 `ros2_control_node`；`friRemote::seqCount` 未初始化。确认后再改。
7. **`/est_ext_torque` 消息无界增长（实验室代码原样，影响实时性）**：加载订阅它的 joint_impedance_controller 或 `ros2 topic echo` 后，控制周期会随运行时间变慢（ROS1 相同）。每次 `enforceLimits()` 都向 `msg_estExtTrq_.layout.dim` 追加两个维度且从不清空，消息体与内存随运行时间线性增长（500 Hz 下每分钟增加约 6 万个维度）。ROS1 同样如此；按"不顺手修 bug"保留，代码中有 `TODO(ros2-migration)`。建议改为只在首次设置 layout。另：发布在实时 write() 路径中同步执行（ROS1 亦然）。
8. **硬件实测**：本次只做了编译与 Gazebo 运行验证，未连接真机；FRI 路径需在接 KRC 的机器上验证。

## 编译、运行与测试
```bash
colcon build --packages-select lwr_hw     # FRIL: --cmake-args -DLWR_HW_WITH_FRIL=ON
ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true port:=49938 update_rate:=500
```
测试见 ../../../migration_tests/README.md。
