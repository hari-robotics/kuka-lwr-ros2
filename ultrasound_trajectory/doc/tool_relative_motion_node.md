# `tool_relative_motion_node`

Moves the KUKA LWR 4+ end-effector by translation/rotation deltas expressed in
the end-effector's **own** coordinate frame, on top of the existing
`lwr_controllers/msg/OneTaskInverseKinematics` absolute-pose controller. Follows
`KUKA_LWR4_Tool_Relative_Motion_Development_Guide.md`. No existing file in
this package was modified beyond additive `CMakeLists.txt`/`package.xml`
dependency lines.

## What it does (main process)

```
~move (Twist) ──▶ IDLE ──▶ PLANNING ──▶ EXECUTING ──▶ SETTLING ──▶ IDLE
                              │             │              │
                          (invalid)   (violation)     (timeout)
                              ▼             ▼              ▼
                            FAULT ◀── ~cancel ──▶ HOLD    FAULT
                                                    │
                                             ~fault_reset
                                                    ▼
                                                  IDLE
```

1. **IDLE** — waiting. A `~move` request is only accepted here; requests
   arriving in any other state are rejected (not queued).
2. **PLANNING** (one shot) — snapshots the measured joint state, computes the
   start pose via KDL forward kinematics, and composes the goal:
   `p_goal = p_start + R_start·Δp_tool`, `R_goal = R_start·R_delta` (body-fixed,
   frozen at the start of the request). Picks a conservative duration from the
   request size and the velocity/acceleration limits. Rejects (→ FAULT)
   non-finite input, an oversized request, or stale/missing joint feedback.
3. **EXECUTING** — at `publish_rate` (default 100 Hz): checks the following
   error against the last published waypoint (pauses the trajectory clock if
   it's out of gate, aborts if it's out of bounds too long); samples the next
   quintic-interpolated SE(3) waypoint; runs it through the damped
   weighted-least-squares joint-space safety gate (singularity damping,
   joint-limit margins, predicted `qdot`/`qddot`, branch-jump continuity); on
   rejection, shrinks the step or lengthens the trajectory rather than
   clipping a joint; publishes the accepted waypoint as `lwr_controllers/msg/PoseRPY`
   with `id=0` to the legacy controller.
4. **SETTLING** — holds, watching measured pose error and joint speed until
   they stay within `settle_*` thresholds for `settle_hold_duration`, then
   returns to IDLE (or FAULTs on `settle_timeout`).
5. **HOLD** — entered via `~cancel`. Publishes the current measured pose once
   as a hold target, then does nothing until `~fault_reset` returns it to
   IDLE.
6. **FAULT** — entered on any violation. Publishes one best-effort hold
   command, latches the first fault reason, and stays there until
   `~fault_reset` (only clears if fresh joint feedback is present again).

`monitor_only` (default `true`) computes and logs everything above but never
publishes to the legacy command topic until explicitly set `false`.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `~move` | `geometry_msgs/msg/Twist` (linear=dx,dy,dz [m], angular=droll,dpitch,dyaw [rad], tool-frame) | in |
| `~cancel` | `std_msgs/msg/Empty` | in |
| `~fault_reset` | `std_msgs/msg/Empty` | in |
| `~status` | `std_msgs/msg/String` (one diagnostics line, `diagnostics_rate` Hz) | out |
| `/lwr/joint_states` (param `joint_state_topic`) | `sensor_msgs/JointState` | in, safety-critical |
| `/cartesian_position/pose/end_effector` (param `cross_check_pose_topic`) | `lwr_controllers/msg/PoseRPY` | in, diagnostic only |
| `/lwr/one_task_inverse_kinematics/command` (param `legacy_command_topic`) | `lwr_controllers/msg/PoseRPY`, `id=0` always | out |

Key parameters (all under the node's private namespace, see
`ik_safety_gate.h`/`trm_types.h` for the full list and defaults):
`root_link`, `tip_link`, `monitor_only`, `publish_rate`,
`request_translation_max`, `request_rotation_max`, `v_trans_max`,
`a_trans_max`, `v_rot_max`, `a_rot_max`, `dq_waypoint_max`,
`joint_limit_margin`, `feedback_timeout`, `following_position_gate`,
`following_orientation_gate`, `settle_*`.

> **Publisher ownership.** Never run this node at the same time as
> `ee_incremental_motion` or `ultrasound_traj_node` — all three publish to the
> same legacy command topic.

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

Confirm the prerequisites are actually up before starting the new node:

```bash
ros2 topic hz /lwr/joint_states
ros2 topic info /robot_description
ros2 topic info /lwr/one_task_inverse_kinematics/command
```

### 3. Start the node in `monitor_only` mode (safe default)

```bash
ros2 run ultrasound_trajectory tool_relative_motion_node --ros-args -p monitor_only:=true
```

In another terminal, watch its diagnostics:

```bash
ros2 topic echo /tool_relative_motion_node/status
```

You should see `state=IDLE` at startup.

### 4. Send a small tool-frame move

```bash
ros2 topic pub --once /tool_relative_motion_node/move geometry_msgs/msg/Twist \
  "linear:  {x: 0.0, y: 0.0, z: 0.01}
   angular: {x: 0.0, y: 0.0, z: 0.0}"
```

Watch `~status` step through `PLANNING → EXECUTING → SETTLING → IDLE` with
sane `max_dq`/`sigma_min` values, and confirm (since `monitor_only:=true`)
**nothing** is published on the legacy command topic:

```bash
ros2 topic echo /lwr/one_task_inverse_kinematics/command
```

Other useful commands while testing:

```bash
# cancel an in-progress move -> HOLD
ros2 topic pub --once /tool_relative_motion_node/cancel std_msgs/msg/Empty "{}"

# clear a HOLD or FAULT -> IDLE
ros2 topic pub --once /tool_relative_motion_node/fault_reset std_msgs/msg/Empty "{}"
```

### 5. Only after the above looks correct: enable real commands

Start at sub-millimetre/sub-degree requests and low speed, per the guide's
commissioning procedure (Section 10.2):

```bash
ros2 run ultrasound_trajectory tool_relative_motion_node --ros-args -p monitor_only:=false
```

Then repeat step 4 with a very small delta first (e.g. `z: 0.001`), watching
`~status` and the legacy command topic the whole time, before increasing
request size or speed.
