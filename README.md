# NearLab Kuka LWR 4+ ROS2 Base Controller
This repository is based on [kuka-lwr](https://github.com/CentroEPiaggio/kuka-lwr), upgraded to ROS2 Humble with some self defined controllers.

## Project Structure
```bash
├── cartesian_position_node    # self defined node for position initialization
├── kuka_lwr                   # kuka-lwr meta package
├── lwr_controllers            # kuka-lwr example controllers
├── lwr_description            # kuka urdfs and meshes
├── lwr_hw                     # kuka-lwr hardware fri interface and ROS2 wrapper
├── single_lwr_example         # launch scripts
└── ultrasound_trajectory      # self-defined controllers
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

### Tests (opt-in)
Test code is isolated from production code: each package keeps its tests in
`test/` (with its own `test/CMakeLists.txt`), and they are neither compiled nor
installed by a normal `colcon build`. Build and run them explicitly, ideally in
a separate workspace build directory:
```bash
colcon build --build-base build_test --install-base install_test \
  --cmake-args -DLWR_BUILD_TESTS=ON
colcon test --build-base build_test --install-base install_test
colcon test-result --test-result-base build_test --verbose
```
Test sources are kept locally (see `.gitignore`); only the per-package
`test/CMakeLists.txt` recipes are tracked.

## Configuration
You can edit the launching configuration in `single_lwr_example/single_lwr_launch/launch/single_lwr_tool.launch.py`:
```python
defaults = {
    'robot_name': 'single_lwr_robot', # the name of the robot
    'use_lwr_sim': 'true', # 'true' to launch gazebo sim, 'false' to launch real robots
    'lwr_powered': 'false', # 'true' to launch real robots, 'false' to launch gazebo sim
    'port': '49938', # connection port for the real robot
    'ip': '192.168.10.2', # ip address for the real robot
    'update_rate': '200', # connection speed, should be accrod with the real robot
    'file': str(robot / 'config/980241-FRI-Driver.init'), # initialization script
    't1_limits': 'false',
    'controllers': 'one_task_inverse_kinematics', # controllers to activate
    'stopped_controllers': 'gravity_compensation_controller', # controllers to deactivate
    'load_moveit': 'false', # use moveit for motion planning
    'use_rviz': 'true', # use rviz visuallization
    'gui': 'true', # launch gui
    'use_cartesian_position': 'true', # in cartesian coordinates
}
```

## Launch logic (same as ROS 1 `single_lwr.launch`)
| `use_lwr_sim` | `lwr_powered` | hardware started by the launch |
| --- | --- | --- |
| `true` | `false` | Gazebo (default) |
| `false` | `true` | real robot through FRI (`lwr_hw`) |
| `false` | `false` | none: spawners wait for an external `/lwr/controller_manager`, e.g. `ros2 launch lwr_hw lwr_hw.launch.py ...` |

Selecting both `use_lwr_sim` and `lwr_powered` is rejected. ros2_control mock
hardware is never chosen implicitly; it is only used by
`single_lwr_moveit/demo.launch.py` or with an explicit `use_mock_hardware:=true`.

As in ROS 1, `single_lwr.launch.py` always spawns `joint_state_controller` and
`arm_state_controller` plus `controllers`, loads `stopped_controllers` inactive,
republishes `/lwr/joint_states` on `/joint_states`, defaults `use_rviz` and
`load_moveit` to `false`, and opens MoveIt's RViz whenever `load_moveit:=true`.
`single_lwr_tool.launch.py` keeps this workspace's own defaults (RViz and
`cartesian_position_node` enabled).

## Run with gazebo simulator
To launch gazebo simulator with rviz:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py controllers:=joint_trajectory_controller
```

And you can also launch `rqt_controller_manager` to manage active controllers in another terminal
```bash
ros2 run rqt_controller_manager rqt_controller_manager
```

Make sure that you activated `joint_trajectory_controller`, `joint_state_controller` and `arm_state_controller`

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
If you have configured everything correct, you can launch the real robot with following commands:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py use_lwr_sim:=false lwr_powered:=true
```

Otherwise, launch the robot with explict parameters:
```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py \
use_lwr_sim:=false lwr_powered:=true \
fri_backend:=fri ip:=192.168.10.2 port:=49938 \
update_rate:=200 receive_timeout_ms:=100 \
load_moveit:=false use_rviz:=true
```

The remaining steps are the same as gazebo simulator, but do not use pose_init node since the real robot control already reads the initial position, you can adjust it through teching panel.


