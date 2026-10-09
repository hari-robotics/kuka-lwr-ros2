# lwr_description

ROS 2 Humble / Gazebo Fortress port of the ROS 1 package. The original `model/`
and `meshes/` layout, macro signature, meshes, joint geometry and physical
parameters are retained. Assembly and launch files live in the original
`single_lwr_example` packages.

The extra `model/kuka_lwr.ros2_control.xacro` replaces the ROS 1 transmission
macro in the expanded description. `kuka_lwr.transmission.xacro` remains as the
original reference file and is not included by the ROS 2 model.

See [migration notes](../../MIGRATION.md) for commands and validation.

The arm macro now includes the tool and TCP from the workspace's
`urdf/lwr_with_tool.urdf`. `model/tool.xacro` preserves that URDF's tool
geometry and mounting transforms (the supplied `urdf/tool.xacro` differs).
`model/lwr_with_tool.xacro` assembles the complete robot at the world origin;
`model/lwr_with_tool.urdf` is its ROS 2 expansion with the real FRI backend.
The existing single-arm assembly also receives the tool through the arm macro.
The duplicate base joint and ROS 1 transmissions in the supplied URDF are
replaced by a single base joint and the existing ROS 2 control configuration.

RViz uses `world` as its fixed frame and shows the tool and TCP axes.
MoveIt's `full_lwr` chain ends at `tcp`; rigid mounting overlaps are excluded
from self-collision checks. The hardware and low-level Cartesian controller
frames remain at `lwr_7_link`, matching the FRI flange pose convention.
Gazebo Fortress merges the massless fixed tool links into the flange; their
visuals, collisions and frame transforms are retained. The source supplies no
tool mass or inertia, so this model does not simulate additional payload mass.
See the [SDFormat fixed-joint documentation](https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/).

Build and launch the simulation with RViz and MoveIt:

```bash
colcon build --packages-select lwr_description single_lwr_robot single_lwr_moveit single_lwr_launch --symlink-install
source install/setup.bash
ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=true load_moveit:=true use_rviz:=true
```

For RViz with ros2_control mock hardware instead of Gazebo (development only;
mock hardware must be requested explicitly):

```bash
ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=false use_mock_hardware:=true use_rviz:=true
```

The single-arm assembly mounts directly to `world` at zero height, matching
the supplied URDF. The former box pedestal has been removed.
