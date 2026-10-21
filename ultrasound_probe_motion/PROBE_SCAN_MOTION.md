# probe_scan_motion_node

Combined primary-scan + relative-adjustment motion node for the position path.
It is the **sole** publisher on `/lwr/one_task_inverse_kinematics/command`, doing
two jobs in one process so there is no arbitration / mux / switch-jump:

- **CRUISE** — a smooth constant-velocity sweep along the probe frame's `+Y`
  axis (trapezoidal velocity profile). Replaces the stepped
  `translate_y_positive` hops that `probe_relative_motion_node` runs for primary
  Y, which had strong velocity ripple and a hard-to-tune `step_duration`.
- **STEP** — a relative 6-DoF probe-frame move (quintic + SLERP), identical to
  `probe_relative_motion_node`, used for the GPIS excursion and the recovery to
  the pre-adjustment pose.

`probe_relative_motion_node` and `probe_motion.launch` are unchanged — use them
for the stepped variants. Use `probe_scan_motion.launch` (this node) for the
cruise variants (`gpis_probe_adjustment_cruise.launch`,
`probe_motion_adjustment_test_cruise.launch`).

There is **no contact-force loop**: `one_task_inverse_kinematics` is stiff
position control and contact depth is owned by the operator.

## Feasibility note

There is no hardware Cartesian-velocity mode on this LWR. `one_task_inverse_kinematics`
is a position P-servo that holds its last setpoint. This node produces "velocity
control" by **streaming position setpoints at `control_rate_hz`** along a
trapezoidal velocity profile — the same technique `ultrasound_traj_cartesian_node`
uses.

## Topics

| Topic | Type | Dir | Default | Meaning |
|---|---|---|---|---|
| `cruise_topic` | `geometry_msgs/Twist` | in | `/probe_cruise_velocity` | probe-frame sweep velocity; `linear.y` is m/s. `0` decelerates to a stop. |
| `action_topic` | `geometry_msgs/Twist` | in | `/probe_action` | relative probe-frame step (`linear` m, `angular` rad ZYX), `step_duration` |
| `recovery_action_topic` | `geometry_msgs/Twist` | in | `/probe_action_recovery` | slower relative step, `recovery_step_duration` |
| `ee_pose_topic` | `lwr_controllers/PoseRPY` | in | `/cartesian_position/pose/end_effector` | EE pose feedback |
| `command_topic` | `lwr_controllers/PoseRPY` | out | `/lwr/one_task_inverse_kinematics/command` | absolute EE setpoint, `id=0` |
| `~cruise_stopped` | `std_msgs/Bool` (latched) | out | — | `true` when the sweep is fully stopped; an orchestrator waits for this before sending relative steps |

## Modes

`IDLE` → `CRUISING` (a non-zero `cruise_topic` command) → back to `IDLE` when the
speed ramps to 0. `IDLE` → `STEPPING` (a relative action) → either `SETTLING`
or directly back to `IDLE`, according to the acknowledged goal's completion
policy. **Relative actions are accepted only while `IDLE`** — the
orchestrator stops the cruise (and waits for `~cruise_stopped`) first. A
`cruise_topic` command is ignored while `STEPPING`.

Acknowledged goals default to `POSE_SETTLED`. The cruise GPIS configuration uses
`TRAJECTORY_COMPLETE` plus `FIXED_DURATION_PROFILE` for outward local-Z and fan
adjustments: they run for `step_duration` without following-error freezes or a
pose-settle delay. The GPIS orchestrator then leaves the motion node in `IDLE`,
where it continues publishing the final target, for its configured fixed
post-trajectory hold before sending another command. Stepped GPIS outward adjustments also use
`TRAJECTORY_COMPLETE`; guarded-dynamic cruise uses
`TRAJECTORY_COMPLETE_GUARDED`. Capped intermediate recovery can remain guarded,
and final saved-pose recovery remains `POSE_SETTLED`.

## Cruise mechanics (per control tick)

1. On entering `CRUISING`, latch the streamed target from the current probe pose
   (orientation frozen).
2. Ramp the commanded speed toward the target by `cruise_accel * dt` (trapezoid).
3. **Following-error freeze:** if the measured probe origin is more than
   `cruise_following_gate_m` behind the streamed target, skip the advance this
   tick so the servo catches up.
4. Advance the streamed target along probe `+Y` by `speed * dt`, convert to an EE
   setpoint, publish.
5. When target and commanded speed are both 0 → `IDLE`, publish
   `~cruise_stopped = true`, hold.

## Parameters

| Param | Default | Meaning |
|---|---:|---|
| `step_duration` | `1.5` s | fixed quintic time for legacy steps and cruise GPIS outward adjustments |
| `recovery_step_duration` | `1.5` s | fixed time for legacy recovery-topic steps; acknowledged recovery is distance-limited |
| `local_z_max_angular_velocity_deg_s` | `14.0625` deg/s | peak rate for dynamic local-Z goals; not used by fixed-duration cruise adjustments |
| `local_z_max_angular_acceleration_deg_s2` | `129.904` deg/s² | peak acceleration for dynamic local-Z goals; not used by fixed-duration cruise adjustments |
| `cruise_speed_max` | `0.02` m/s | hard ceiling on the commanded sweep speed |
| `cruise_accel` | `0.01` m/s² | trapezoidal ramp rate |
| `cruise_following_gate_m` | `0.005` m | streamed-target freeze threshold |
| `control_rate_hz` | `100` | control-loop / setpoint-stream rate |
| `probe_length` / `probe_y_offset` / `probe_z_rotation_deg` | from `probe_calibration.yaml` | flange→probe transform |

The **actual** cruise speed is sent by the orchestrator (GPIS node / test node)
on `cruise_topic` and clamped to `cruise_speed_max`.

## Run

```bash
roslaunch ultrasound_probe_motion probe_scan_motion.launch
# manual smoke test:
rostopic pub -1 /probe_cruise_velocity geometry_msgs/Twist '{linear: {y: 0.003}}'   # sweep at 3 mm/s
rostopic echo /lwr/one_task_inverse_kinematics/command                              # position.y advances
rostopic echo /probe_scan_motion_node/cruise_stopped                                # false while cruising
rostopic pub -1 /probe_cruise_velocity geometry_msgs/Twist '{linear: {y: 0.0}}'     # ramp to a stop
rostopic pub -1 /probe_action geometry_msgs/Twist '{angular: {z: 0.02}}'            # relative step (only while stopped)
```

**SAFETY:** only one publisher on `/lwr/one_task_inverse_kinematics/command`. Do
not run this at the same time as `probe_motion.launch`, or `ultrasound_trajectory`'s
`ee_incremental_motion` / `ultrasound_traj_node` / `tool_relative_motion_node`.

## Difference from `probe_relative_motion_node`

| | `probe_relative_motion_node` | `probe_scan_motion_node` |
|---|---|---|
| relative steps | yes | yes (identical) |
| primary +Y | stepped relative `linear.y` Twists from the orchestrator | continuous cruise on `cruise_topic` |
| control loop | driven by the pose callback (~200 Hz) | fixed `ros::Timer` at `control_rate_hz` |
| cruise / `~cruise_stopped` | — | yes |
