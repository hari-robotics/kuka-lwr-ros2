# ultrasound_probe_motion

Commands the KUKA LWR ultrasound probe with relative 6-DoF motions expressed
in the **probe's own frame** (not the end-effector frame, not the robot base
frame), converting them internally to absolute base-frame commands for the
existing `one_task_inverse_kinematics` controller.

## Calibration

The probe frame is rigidly offset from the end-effector (flange) frame by:

```
T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) * Trans_y(probe_y_offset)
```

Configurable in `config/probe_calibration.yaml`. This file is the single
source of truth for the whole workspace: `ultrasound_probe_impedance` and
`ultrasound_probe_reconstruction` load it at launch time via
`$(find ultrasound_probe_motion)/config/probe_calibration.yaml` instead of
keeping their own copies, so editing it here (and re-launching, no rebuild
needed) takes effect in all three packages. Each launch file still accepts
`probe_length:=...`/`probe_y_offset:=...`/`probe_z_rotation_deg:=...` as a
per-launch override on top of the yaml.

**To change the calibration permanently (e.g. after re-measuring the
probe), edit the values in this yaml file directly** — that is the only
edit needed, and it takes effect for `ultrasound_probe_motion`,
`ultrasound_probe_impedance`, and `ultrasound_probe_reconstruction` alike.
Passing `probe_length:=...` etc. on the `roslaunch` command line only
overrides that one launch invocation of that one package and does not
persist or propagate — use it for one-off testing only, not for updating
the calibration.

| Param | Default | Meaning |
|---|---|---|
| `probe_length` | 0.252 m | nominal probe length along the flange's +z axis |
| `probe_y_offset` | 0.00775 m | as-built origin offset along the probe's own (rotated) +y axis |
| `probe_z_rotation_deg` | 5.0 deg | as-built rotation of the probe frame about the shared z-axis |

## Topics

| Topic | Type | Direction | Default |
|---|---|---|---|
| `action_topic` | `geometry_msgs/Twist` | in | `/probe_action` — relative delta in the probe frame (`linear`=dx,dy,dz [m], `angular`=droll,dpitch,dyaw [rad]) |
| `ee_pose_topic` | `lwr_controllers/PoseRPY` | in | `/cartesian_position/pose/end_effector` — current EE pose feedback |
| `command_topic` | `lwr_controllers/PoseRPY` | out | `/lwr/one_task_inverse_kinematics/command` — absolute pose sent to the low-level IK controller |
| `recovery_action_topic` | `geometry_msgs/Twist` | in | `/probe_action_recovery` — second action channel; steps received here use `recovery_step_duration`. Set `""` to disable. |

One motion runs at a time (new commands are rejected while a step is executing),
interpolated over a quintic + quaternion-SLERP profile.

### Step-timing parameters

Defaults match `ultrasound_probe_impedance`'s `probe_incremental_force_node`.

| Param | Default | Meaning |
|---|---|---|
| `step_duration` | 3.0 s | quintic interpolation time for a normal action |
| `recovery_step_duration` | 3.0 s | interpolation time for actions on `recovery_action_topic` |
| `recovery_action_topic` | `/probe_action_recovery` | second `Twist` subscriber whose steps use `recovery_step_duration`; `""` disables it |

A non-finite or non-positive duration is rejected with a warning and the default
is kept. A plain `roslaunch ultrasound_probe_motion probe_motion.launch`
subscribes to both `/probe_action` and `/probe_action_recovery` with 3.0 s steps.

To change the duration and keep every paced publisher in sync, follow
`STEP_DURATION_CONFIGURATION.md`.

## Build

```bash
cd ~/kuka_ws
colcon build --packages-select ultrasound_probe_motion
source install/setup.bash
```

## Run

```bash
roslaunch ultrasound_probe_motion probe_motion.launch
```

**Safety:** `/lwr/one_task_inverse_kinematics/command` is a shared topic. Do
**not** run this node at the same time as `ee_incremental_motion`,
`ultrasound_traj_node`, or `tool_relative_motion_node` (all in
`ultrasound_trajectory`) — they publish to the same topic with no
arbitration between them.

## Manual test

```bash
# watch the resulting absolute controller command
rostopic echo /lwr/one_task_inverse_kinematics/command

# command a 10mm move along the probe's long (x) axis
rostopic pub -1 /probe_action geometry_msgs/Twist \
  '{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Expect the EE target logged by the node to move mostly along EE-x with a
small (~8.7%, i.e. ~0.00087 m) leak into EE-y — the signature of the 5°
calibration rotation being applied correctly. A pure `linear.z` command
should be calibration-invariant (identical to the same delta sent through
`ee_incremental_motion`'s `/ee_action`), since `Rot_z` leaves z-vectors
unchanged.
