# NearLab Kuka LWR 4+ ROS2 Base Controller
This repository is based on [kuka-lwr](https://github.com/CentroEPiaggio/kuka-lwr), upgraded to ROS2 Humble with some self defined controllers.
The port follows the lab version in `kuka_ws/src/kuka-lwr`, `kuka_ws/src/ultrasound_trajectory` and `kuka_ws/src/cartesian_position_node` line by line (only the ROS interface is replaced); see each package's `MIGRATION_NOTES.md`.

## Validation status

| Area | Status |
|---|---|
| Build (colcon, Humble), static launch equivalence with the ROS 1 lab version | ✅ passed (static launch checks) |
| Kinematics in simulation (Gazebo Fortress): joint trajectory, forward kinematics / `cartesian_position_node`, `one_task_inverse_kinematics`, `pose_init`, `ee_incremental_motion` | ✅ passed (simulation workflow tests) |
| Real robot (FRI / KRC) | ⏳ awaiting validation on the real KUKA LWR 4+ |
| Dynamics and impedance control (`joint_impedance_controller`, `cartesian_impedance_controller`, torque-based controllers, `/est_ext_torque`) | ⚠️ not validated: Gazebo does not simulate the KUKA joint/Cartesian impedance (interfaces only checked) |

The migration tests live in the workspace-level `migration_tests/` directory (next to `src/`, not
part of this repository; see [TESTING.md](TESTING.md)). Known differences from ROS 1 and items that
need confirmation before using the real robot are listed in each package's `MIGRATION_NOTES.md`
(see in particular `kuka-lwr/lwr_hw`).

## Project Structure
Same layout as the ROS 1 lab workspace (`kuka_ws/src`):
```bash
├── cartesian_position_node        # Cartesian pose/velocity of flange and tooltip
├── kuka-lwr
│   ├── kuka_lwr                   # meta package
│   ├── lwr_controllers            # controllers (lab version)
│   ├── lwr_description            # LWR 4+ URDF and meshes
│   ├── lwr_hw                     # FRI hardware interface and Gazebo plugin (ros2_control)
│   ├── lwr_robot_tool             # LWR + ultrasound tool description (kuka_ws/src/kuka/lwr_robot_tool)
│   └── single_lwr_example         # robot config, launch files, MoveIt config
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
`pose_init` keeps the ROS 1 topic name; the Humble trajectory controller listens on `~/joint_trajectory`, hence the remap.

And now you can run the following command to initialize the robot pose for ultrasound scanning:
```bash
ros2 run ultrasound_trajectory pose_init --ros-args -p use_sim_time:=true \
  -r /lwr/joint_trajectory_controller/command:=/lwr/joint_trajectory_controller/joint_trajectory
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
If you have configured everything correct, you can launch the real robot with following commands:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py use_lwr_sim:=false lwr_powered:=true
```

Otherwise, launch the robot with explict parameters:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py \
use_lwr_sim:=false lwr_powered:=true \
ip:=192.168.10.2 port:=49938 update_rate:=500 \
load_moveit:=false use_rviz:=true
```

The remaining steps are the same as gazebo simulator, but do not use pose_init node since the real robot control already reads the initial position, you can adjust it through teching panel.

The explicit offline MoveIt demo remains available with
`ros2 launch single_lwr_moveit demo.launch.py`. Migration equivalence tests are
kept outside the source packages, see [TESTING.md](TESTING.md).


