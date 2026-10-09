# `ee_incremental_impedance_node`

Moves the KUKA LWR end-effector by translation/rotation deltas expressed in
the end-effector's **own** coordinate frame, same input convention as
`ee_incremental_motion.cpp`, but publishes to the **Cartesian impedance
controller** (`lwr_controllers/msg/CartesianImpedancePoint` on
`/lwr/cartesian_impedance_controller/command`) instead of the legacy
position-only `one_task_inverse_kinematics` controller. No existing file in
this package was modified beyond two additive `CMakeLists.txt` lines
(`add_executable` / `target_link_libraries`).

Source: `src/ee_incremental_impedance_node.cpp`. Design plan:
`~/.claude/plans/now-i-want-to-twinkly-fog.md`.

## What it does (main process)

1. **Pose callback** (`~pose_topic`, default
   `/cartesian_position/pose/end_effector`) — updates the current EE pose. On
   the *first* callback it latches the current pose as the held target and
   fills stiffness/damping/force once (mirrors `linear_traj_node.cpp`'s
   `flag_init` pattern); `header.frame_id` is set non-empty so `x_fri` is
   treated as an **absolute base-frame pose** by the controller, not a
   per-cycle tool-frame increment.
2. **Action callback** (`~action_topic`, default `/ee_action_impedance`) — a
   `geometry_msgs/msg/Twist` delta in the EE frame (`linear` = dx,dy,dz [m],
   `angular` = droll,dpitch,dyaw [rad]). Rejected with a `RCLCPP_WARN` if the
   pose isn't initialized yet, or if a step is already executing (one action
   in flight at a time — new ones are dropped, not queued). Otherwise:
   clamps the delta to `~max_linear_step` / `~max_angular_step`, rotates the
   local displacement into the base frame (`p_world_delta = R_current · Δp`),
   and composes the target orientation in the EE's own frame
   (`R_target = R_current · R_delta`), taking the shorter SLERP arc.
3. **Interpolation** (every pose-callback tick while executing) — a 5th-order
   (quintic) polynomial timing law driven by elapsed wall-clock time (not a
   callback counter, so it's correct regardless of the pose topic's actual
   publish rate), over `~step_duration` seconds. Position is linearly
   interpolated component-wise; orientation is SLERPed between start/target
   quaternions and written straight into `x_fri.orientation` (no RPY
   round-trip needed — `CartesianImpedancePoint`'s pose field is already a
   quaternion). On completion it holds the target and waits for the next
   action.
4. **z-axis force control (optional, EE frame, closed-loop)** — x, y and
   orientation are always position-controlled as above. z can additionally be
   handed to a continuous PI outer loop that trims the commanded position
   along the EE's *current* local z-axis so measured contact force converges
   to a fixed setpoint (`~target_force_z`), regardless of unknown
   tissue/surface stiffness. It reads `~force_topic`
   (`geometry_msgs/msg/Wrench.force.z`, same convention as `linear_traj_node.cpp`'s
   `/Force_calibrated`) and adds `dz_local · R_current.col(2)` (the trim,
   expressed in base frame) on top of whatever x/y/z the step above is
   commanding. **Starts disabled** — see the safety section below.

## Topics

| Topic (param) | Type | Direction |
|---|---|---|
| `~action_topic` (default `/ee_action_impedance`) | `geometry_msgs/msg/Twist` (linear=dx,dy,dz [m], angular=droll,dpitch,dyaw [rad], **EE frame**) | in |
| `~pose_topic` (default `/cartesian_position/pose/end_effector`) | `lwr_controllers/msg/PoseRPY` | in |
| `~force_topic` (default `/Force_calibrated`) | `geometry_msgs/msg/Wrench` (`force.z` used, EE/tip frame) | in |
| `~enable_force_control` | `std_srvs/srv/SetBool` | service |
| `~command_topic` (default `/lwr/cartesian_impedance_controller/command`) | `lwr_controllers/msg/CartesianImpedancePoint`, absolute base-frame pose | out |

Other params (all under the node's private namespace, all optional):
`~step_duration` (default `6.0` s), `~stiffness_linear` (default `1500`),
`~stiffness_angular` (default `250`), `~damping` (default `0.8`),
`~max_linear_step` (default `0.05` m), `~max_angular_step` (default `0.2`
rad).

**Force-control params** (all placeholders — verify on your actual
sensor/mechanism before trusting them):
`~target_force_z` (default `0.0` N — set explicitly), `~force_sign` (default
`1.0`, flip to `-1.0` if the loop pushes the wrong way), `~force_kp` (default
`0.0005` (m/s)/N), `~force_ki` (default `0.0`), `~max_force_step_rate`
(default `0.003` m/s), `~max_force_travel` (default `0.02` m, hard cap on
cumulative trim from where it was enabled), `~max_allowed_force_z` (default
`8.0` N, hard cutoff — auto-disables past this), `~force_feedback_timeout`
(default `0.5` s), `~stiffness_z_active` (default `300`, `k_fri.z` used while
force control is enabled — softer than `~stiffness_linear` so the real
controller doesn't fight the ROS-rate position trim).

> **Publisher ownership.** `~action_topic` defaults to a topic distinct from
> `/ee_action` (used by `ee_incremental_motion.cpp`) and `~move` (used by
> `tool_relative_motion_node.cpp`) on purpose — those two drive a *different*
> controller (`one_task_inverse_kinematics`). Don't remap `~action_topic` to
> `/ee_action` while one of those nodes is also running, or a single command
> could actuate the robot through two controllers at once.

> **Force-control safety.** Disabled at startup; arm it explicitly via
> `~enable_force_control`. It refuses to enable without recent force
> feedback, and auto-disables (logging `ROS_ERROR`, one-shot, no auto-retry)
> on stale feedback, `|measured F_z| > ~max_allowed_force_z`, or on the
> cumulative trim exceeding `~max_force_travel`. Disabling (manually or via
> fault) freezes the trim rather than snapping the arm back — the physically
> achieved contact position is kept, only active tracking stops. Start
> tuning with the arm free (not in contact) at a very low
> `~max_force_step_rate` to confirm `~force_sign` before ever touching tissue.

## ROS 2 commands

Configuration parameters are declared on this node and are read-only after startup.
Private topics/services use `~/name`; parameter names have no `~` prefix.
Set `use_sim_time:=true` through `--ros-args -p use_sim_time:=true` when using Gazebo.


### 1. Build

```bash
cd /home/hari/kuka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ultrasound_trajectory --symlink-install
source install/setup.bash
```

### 2. Bring up the robot (sim or hardware, as you already do)

```bash
ros2 launch single_lwr_launch single_lwr_tool.launch.py
```

Confirm the prerequisites are up before starting the new node:

```bash
ros2 topic hz /cartesian_position/pose/end_effector
ros2 topic info /lwr/cartesian_impedance_controller/command
```

### 3. Dry-run without the robot (validate the math only)

In one terminal, stub the current pose so the node can initialize:

```bash
ros2 topic pub --rate 50 /cartesian_position/pose/end_effector lwr_controllers/msg/PoseRPY \
  "id: 0
   position: {x: 0.5, y: 0.0, z: 0.3}
   orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}"
```

In a second terminal, start the node and watch its output:

```bash
ros2 run ultrasound_trajectory ee_incremental_impedance_node
```

```bash
ros2 topic echo /lwr/cartesian_impedance_controller/command
```

In a third terminal, send a small EE-frame move:

```bash
ros2 topic pub --once /ee_action_impedance geometry_msgs/msg/Twist \
  "linear:  {x: 0.02, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}"
```

Confirm `x_fri.position`/`orientation` interpolates smoothly from the start
pose to the expected target over `~step_duration` seconds (default 6 s),
then holds. Sending a second action mid-step should log `RCLCPP_WARN` ("still
executing") and leave the in-flight target unchanged.

### 4. Dry-run the force loop (still no robot, stub the force sensor too)

In a fourth terminal, stub force feedback and set a fixed target force via
ROS 2 startup parameters (restart the node to change them):

```bash
ros2 run ultrasound_trajectory ee_incremental_impedance_node \
  --ros-args -p target_force_z:=2.0 -p max_force_step_rate:=0.001
```

```bash
ros2 topic pub --rate 20 /Force_calibrated geometry_msgs/msg/Wrench \
  "force:  {x: 0.0, y: 0.0, z: 0.0}
   torque: {x: 0.0, y: 0.0, z: 0.0}"
```

With the pose stub from step 3 still running, try enabling before any force
message has arrived — should be refused:

```bash
ros2 service call /ee_incremental_impedance_node/enable_force_control std_srvs/srv/SetBool "data: true"
```

Now with `/Force_calibrated` actually publishing, enable it and watch the
trim slowly move `x_fri.position` (verify direction matches your sign
convention — flip `~force_sign` if it moves the wrong way):

```bash
ros2 service call /ee_incremental_impedance_node/enable_force_control std_srvs/srv/SetBool "data: true"
ros2 topic echo /lwr/cartesian_impedance_controller/command
```

Test the safety trips: publish a `force.z` beyond `~max_allowed_force_z`, or
stop publishing `/Force_calibrated` entirely for `~force_feedback_timeout`,
and confirm the node logs `ROS_ERROR` and auto-disables. Disable manually
at any time:

```bash
ros2 service call /ee_incremental_impedance_node/enable_force_control std_srvs/srv/SetBool "data: false"
```

### 5. Run against the real robot

Only after the dry-run above looks correct, and with a human at the enabling
switch / e-stop (same operating procedure as the other nodes in this
package):

```bash
ros2 run ultrasound_trajectory ee_incremental_impedance_node
```

Start with a very small delta first (e.g. `z: 0.005`) before increasing
request size.
