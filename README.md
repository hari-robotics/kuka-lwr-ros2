# NearLab Kuka LWR 4+ ROS2 Base Controller
This repository is based on [kuka-lwr](https://github.com/CentroEPiaggio/kuka-lwr), upgraded to ROS2 Humble with some self defined controllers.
The port follows the lab version in `kuka_ws/src/kuka-lwr`, `kuka_ws/src/ultrasound_trajectory` and `kuka_ws/src/cartesian_position_node` line by line (only the ROS interface is replaced); see each package's `MIGRATION_NOTES.md`.

## Validation status

| Area | Status |
|---|---|
| Build (colcon, Humble), static launch equivalence with the ROS 1 lab version | ✅ passed (static launch checks) |
| Kinematics in simulation (Gazebo Fortress): joint trajectory, forward kinematics / `cartesian_position_node`, `one_task_inverse_kinematics`, `pose_init`, `ee_incremental_motion` | ✅ passed (simulation workflow tests) |
| Packages used by the lab tutorial: `kuka_controllers` (hierarchical_controller), `kuka_teleop`, `ultrasound_probe_motion`, `ultrasound_probe_impedance` (node interface), `kuka_launch` | ✅ passed in simulation (`migration_tests/tutorial_packages`); force control itself ⚠️ needs the real robot |
| Real robot (FRI / KRC): connection, monitor → command mode, `/lwr/joint_states`, `cartesian_position_node`, `/est_ext_torque` | ✅ passed on the real KUKA LWR 4+ (2026-10-21, Tool 8) |
| Real robot: `one_task_inverse_kinematics`, `ee_incremental_motion` | ✅ passed on the real robot |
| Real robot: `cartesian_impedance_controller` (switch from `one_task_inverse_kinematics`, stiffness/damping commands via `stiffness_demo.py`) | ✅ passed on the real robot |
| `sriforcesensor` (`forcesensor`, `/Force`), `frame_grabber` (`cameratoROS_cropped.py`, `/frame_grabber/image_cropped_topic`) | ✅ passed with the real SRI force sensor and frame grabber |
| Force control (`ultrasound_probe_impedance` with `tool_comp_ros.py` / `/Force_calibrated`) | ⏳ ported and checked in simulation (node interface); next milestone on the real robot |
| Other dynamics controllers (`joint_impedance_controller`, torque-based controllers) | ⚠️ not validated: Gazebo does not simulate the KUKA joint/Cartesian impedance (interfaces only checked) |

The migration tests live in the workspace-level `migration_tests/` directory (next to `src/`, not
part of this repository; see [TESTING.md](TESTING.md)). Known differences from ROS 1 and items that
need confirmation before using the real robot are listed in each package's `MIGRATION_NOTES.md`
(see in particular `kuka-lwr/lwr_hw`).

## Project Structure
Same layout as the ROS 1 lab workspace (`kuka_ws/src`); the KUKA packages of `kuka_ws/src/kuka` used by
the lab tutorial are under `kuka-lwr/`:
```bash
├── cartesian_position_node        # Cartesian pose/velocity of flange and tooltip
├── frame_grabber                  # camera / frame grabber image publisher (ultrasound image)
├── kuka-lwr
│   ├── kuka_controllers           # hierarchical (RCM) / basic joint controllers (kuka_ws/src/kuka/kuka_controllers)
│   ├── kuka_launch                # single_lwr launch with kuka_controllers config (kuka/kuka_launch, merged with the same-name copy in kuka_controllers)
│   ├── kuka_lwr                   # meta package
│   ├── kuka_teleop                # Sigma7 teleoperation script (kuka_ws/src/kuka/kuka_teleop)
│   ├── lwr_controllers            # controllers (lab version)
│   ├── lwr_description            # LWR 4+ URDF and meshes
│   ├── lwr_hw                     # FRI hardware interface and Gazebo plugin (ros2_control)
│   ├── lwr_robot_tool             # LWR + ultrasound tool description (kuka_ws/src/kuka/lwr_robot_tool)
│   └── single_lwr_example         # robot config, launch files, MoveIt config
├── sriforcesensor                 # SRI 6-axis force sensor driver (TCP) and tool gravity compensation
├── ultrasound_probe_impedance     # probe-frame motion and z force control on the Cartesian impedance controller
├── ultrasound_probe_motion        # probe-frame motion on one_task_inverse_kinematics, /probe_step action, probe calibration
└── ultrasound_trajectory          # trajectory / motion nodes
```

## Installation

The supported environment is **Ubuntu 22.04 (Jammy), ROS 2 Humble and Gazebo
Fortress (Ignition Gazebo 6)**. The simulation plugin directly uses the Gazebo 6
API. Install ROS 2 Humble and configure its apt repository using the
[official ROS installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
before running the commands below.

### Install package dependencies with rosdep
```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

### Direct apt installation and Gazebo fallback

If an older rosdep database cannot resolve a system dependency, update rosdep
first. These are the corresponding apt packages for the external libraries and
simulation tools; they can also be installed directly:
```bash
sudo apt install -y liborocos-kdl-dev libeigen3-dev libboost-all-dev \
  libignition-gazebo6-dev ros-humble-gz-ros2-control \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-rqt-controller-manager
```


### Build
```bash
colcon build
source install/setup.bash
# zsh users: source install/setup.zsh
```

Run `colcon build` from the workspace root: colcon reads `colcon.meta` there, which
adds `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to every package, so each build refreshes
`build/<package>/compile_commands.json`. `.clangd` in the workspace root points the
sources of each package at its own database; with the VS Code clangd extension
(`.vscode/settings.json`) no extra step is needed. A newly added package must be listed
in both `colcon.meta` and `.clangd`.

## Configuration
`kuka-lwr/single_lwr_example/single_lwr_launch/launch/single_lwr_tool.launch.py` is the port of the lab
`single_lwr_tool.launch`; its launch arguments and defaults are the ROS 1 ones:
`robot_name:=single_lwr_robot use_lwr_sim:=true lwr_powered:=false port:=49938 ip:=192.168.10.2
t1_limits:=false controllers:=one_task_inverse_kinematics stopped_controllers:=gravity_compensation_controller
load_moveit:=false use_rviz:=false gui:=true`, plus `update_rate:=500` for the real robot. It
always starts `cartesian_position_node`, as in ROS 1.

Controller gains are configured in
`kuka-lwr/single_lwr_example/single_lwr_robot/config/controllers.yaml`; the chain frames are
in `config/hw_interface.yaml`. As in ROS 1, controllers run at the Gazebo step in
simulation (`update_rate: 1000`, 1 ms) and at the robot sample rate on the real
robot: pass `update_rate:=<Hz>` (default 500, matching KRL `friOpen(2)`).
When both `use_lwr_sim` and `lwr_powered` are false, bringup only publishes the
model and joint states for display, as in ROS 1.

Porting details and the items that need confirmation are in each package's
`MIGRATION_NOTES.md`.

## Run with gazebo simulator
To launch gazebo simulator with rviz:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=joint_trajectory_controller use_rviz:=true
```

And you can also launch `rqt_controller_manager` to manage active controllers in another terminal
```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

Make sure that you activated `joint_trajectory_controller`, `joint_state_controller` and `arm_state_controller`.
`pose_init` publishes on `/lwr/joint_trajectory_controller/joint_trajectory`, the topic of the Humble trajectory controller (ROS 1 `~/command`), so no remap is needed.

And now you can run the following command to initialize the robot pose for ultrasound scanning:
```bash
ros2 run ultrasound_trajectory pose_init --ros-args -p use_sim_time:=true
```

After initialization, deactivate `joint_trajectory_controller` and activate `one_task_inverse_kinematics`

Now you can launch the relative pose controller:
```bash
ros2 run ultrasound_trajectory ee_incremental_motion --ros-args -p use_sim_time:=true
```

The robot will subscribe the topic `/ee_action` for performing relative motion, to test the availiability of this controller, you can send some messages by hand:
```bash
ros2 topic pub --once /ee_action geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Notice, the motion is relative motion, only publish the topic once.


## Run with real LWR robots connected
PC setup (lab tutorial, "Communication preparation"):
- the robot NIC has the static address `192.168.10.1/24`; `ping 192.168.10.2` reaches the KRC;
- `/etc/security/limits.conf` uses the Linux **user name** (`whoami`), not the host name, e.g.
  `hari hard rtprio 95`, `hari soft memlock unlimited`; with `session required pam_limits.so` in
  `/etc/pam.d/common-session`, log in again or reboot and check that `ulimit -r -l` prints `95` and `unlimited`.

Before starting ROS, the FRI link can be checked passively (the KRL program started, robot in
monitor mode; nothing is sent back to the KRC):
```bash
ros2 run lwr_hw fri_monitor.py     # expect ~100 Hz from 192.168.10.2, state=MONITOR
```
`quality=UNACCEPTABLE` is expected here because no one answers; it becomes `PERFECT` once
`lwr_hw` runs. If the teach pendant reports `Load data incorrect` / `Fix erroneous dynamic mode`,
the selected tool (`BAS (#TOOL, n)` in the KRL program and *Configure - Set tool/base*) does not
match the mounted tool.

If you have configured everything correct, you can launch the real robot with following commands:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py use_lwr_sim:=false lwr_powered:=true
```

Otherwise, launch the robot with explict parameters (`update_rate` must match the KRL
`friOpen()` period, 10 ms → 100 in the lab programs):
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py \
use_lwr_sim:=false lwr_powered:=true \
ip:=192.168.10.2 port:=49938 update_rate:=100 \
load_moveit:=false use_rviz:=true
```

The remaining steps are the same as gazebo simulator, but do not use pose_init node since the real robot control already reads the initial position, you can adjust it through teching panel.

### Cartesian impedance control (lab tutorial, real robot)
`x_fri` of `lwr_controllers/CartesianImpedancePoint` is the absolute pose of the KRC `#TOOL` frame
in the robot base, so every command has to start from the current pose published on
`/lwr/cartesian_impedance_controller/cartesian_pose` (`publish_cartesian_pose: true` in
`controllers.yaml`).
```bash
ros2 service call /lwr/controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: cartesian_impedance_controller}"
ros2 service call /lwr/controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: cartesian_impedance_controller}"
ros2 service call /lwr/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [cartesian_impedance_controller], deactivate_controllers: [one_task_inverse_kinematics], strictness: 2}"
ros2 run lwr_controllers stiffness_demo.py   # holds the current pose and steps K: default → 1500 → 500 → 200 N/m → default
```
The switch makes the KRL program stop FRI, change `$stiffness.strategy` to 20 (CARTESIAN_IMPEDANCE)
and start FRI again; the controller then holds the current pose with K = 800/800/200 N/m, 100 Nm/rad, D = 0.8.
The `ros2 control ...` verbs below need `ros-humble-ros2controlcli`; the service calls above work without it.

### Force control (lab tutorial, real robot)
ROS 2 commands for the tutorial steps (teach pendant steps unchanged; Tool 8 / `ros_control_giorgia`):
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py use_lwr_sim:=false lwr_powered:=true
ros2 control switch_controllers --deactivate one_task_inverse_kinematics --activate cartesian_impedance_controller -c /lwr/controller_manager
# (cartesian_impedance_controller has to be loaded first, e.g. "Load, Configure and Activate" in rqt_controller_manager)
ros2 run sriforcesensor forcesensor
cd src/sriforcesensor/src/ANN && ros2 run sriforcesensor tool_comp_ros.py      # publishes /Force_calibrated
ros2 launch ultrasound_probe_impedance probe_incremental_force.launch.py        # or probe_incremental_impedance.launch.py
ros2 service call /probe_incremental_force_node/enable_force_control std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /probe_action_force geometry_msgs/msg/Twist "{linear: {x: 0.01}}"   # /probe_action_impedance for the impedance node
```

### Teleoperation with the hierarchical controller (tutorial "kuka steps", Part 2)
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=hierarchical_controller   # add use_lwr_sim:=false lwr_powered:=true for the robot
ros2 run kuka_teleop kuka_teleop.py      # needs a ROS 2 Sigma7 driver publishing /sigma/sigma0/pose and /sigma/sigma0/buttons
```

The explicit offline MoveIt demo remains available with
`ros2 launch single_lwr_moveit demo.launch.py`. Migration equivalence tests are
kept outside the source packages, see [TESTING.md](TESTING.md).


