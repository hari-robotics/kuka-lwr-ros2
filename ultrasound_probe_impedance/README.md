# ultrasound_probe_impedance

Drives the KUKA LWR **Cartesian impedance controller** from relative 6-DoF
motions expressed in the **ultrasound probe's own frame** (not the
end-effector frame, not the robot base frame). Combines:

- the probe-frame calibration transform from
  [`ultrasound_probe_motion`](../ultrasound_probe_motion)'s
  `probe_relative_motion_node`, and
- the impedance-controller output + optional z-axis contact-force loop from
  [`ultrasound_trajectory`](../ultrasound_trajectory)'s
  `ee_incremental_impedance_node`.

This package provides two nodes, sharing the same probe-frame calibration
and step-interpolation machinery but differing in **how** the z-axis
contact force is closed:

| Node | z-axis force mechanism | Use case |
|---|---|---|
| `probe_incremental_impedance_node` | admittance: trims `x_FRI.position` along the probe's current z-axis to hold `~target_force_z` | single contact events; z is either action-driven or force-driven, never both |
| `probe_incremental_force_node` | feedforward: drives `f_FRI.force.z` directly, contact-gated by `~threshold_f` | scanning under constant contact force: x/y/orientation stay action-driven **while** z force control runs concurrently |

Only run one of these two nodes (and never together with
`ee_incremental_impedance_node`) — see [Safety](#safety).

## Calibration

The probe frame is rigidly offset from the end-effector (flange) frame by:

```
T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) * Trans_y(probe_y_offset)
```

Loaded by default from the existing
`ultrasound_probe_motion/config/probe_calibration.yaml` (single source of
truth -- not duplicated here):

| Param | Default | Meaning |
|---|---|---|
| `probe_length` | 0.252 m | nominal probe length along the flange's +z axis |
| `probe_y_offset` | 0.00775 m | as-built origin offset along the probe's own (rotated) +y axis |
| `probe_z_rotation_deg` | 5.0 deg | as-built rotation of the probe frame about the shared z-axis |

**To change these values permanently, edit
[`ultrasound_probe_motion/config/probe_calibration.yaml`](../ultrasound_probe_motion/config/probe_calibration.yaml)**
directly — not a file in this package. That single edit takes effect here
and in `ultrasound_probe_reconstruction` too, with no rebuild needed.
Passing `probe_length:=...` etc. to this package's launch files only
overrides that one run and does not update the shared calibration.

## Topics

Common to both nodes:

| Topic | Type | Direction | Default |
|---|---|---|---|
| `~pose_topic` | `lwr_controllers/PoseRPY` | in | `/cartesian_position/pose/end_effector` — current EE pose feedback |
| `~force_topic` | `geometry_msgs/Wrench` | in | `/Force_calibrated` — measured contact force, used only if force control is enabled |
| `~command_topic` | `lwr_controllers/CartesianImpedancePoint` | out | `/lwr/cartesian_impedance_controller/command` |

`~action_topic` (`geometry_msgs/Twist`, in) — relative delta in the probe
frame (`linear`=dx,dy,dz [m], `angular`=droll,dpitch,dyaw [rad]) — defaults
to a **different topic per node** so a command meant for one can never
cross-trigger the other:

| Node | `~action_topic` default |
|---|---|
| `probe_incremental_impedance_node` | `/probe_action_impedance` |
| `probe_incremental_force_node` | `/probe_action_force` |

`probe_incremental_force_node` also accepts recovery-only Twist commands on
`~recovery_action_topic` (default `/probe_action_force_recovery`). Normal
commands use the 3.0 s `step_duration`; recovery commands use the 3.0 s
`recovery_step_duration`. Both topics share one executor, so commands are never
run concurrently and a new message is rejected while one is pending/executing.

One motion runs at a time per node (new commands are rejected while a step is
executing), using quintic translation and quaternion-SLERP orientation.
`probe_incremental_impedance_node` retains its 6 s default, while
`probe_incremental_force_node` uses a 3.0 s default exposed through the
`step_duration` launch argument (commissioned for safety — see
`STEP_DURATION_CONFIGURATION.md`).

## Safety

- `~command_topic` is shared by **all three** nodes that can drive the
  impedance controller in this workspace: `probe_incremental_impedance_node`,
  `probe_incremental_force_node`, and `ultrasound_trajectory`'s
  `ee_incremental_impedance_node`. **Only ever run one of them at a time**
  (or remap `~command_topic` so exactly one is left publishing) — there is
  no arbitration between publishers on that topic.
- z-axis force control on **both** nodes starts **disabled** and must be
  armed via the `~enable_force_control` (`std_srvs/SetBool`) service. Both
  refuse to arm without a live pose and recent force feedback. All force
  gains/limits below are placeholders — verify sign and magnitude on the
  real sensor/mechanism before relying on either for real tissue contact.
- **`probe_incremental_impedance_node`**: an admittance loop that trims
  `x_FRI.position` along the probe's current z-axis to converge
  `~target_force_z`. While enabled, z is taken away from `~action_topic`
  entirely (only x/y/orientation stay action-driven). Auto-disables on stale
  force feedback, on exceeding `~max_allowed_force_z`, or on exceeding
  `~max_force_travel`. Verify at a very low `~max_force_step_rate` first.
  The force-trim axis is the EE's current local z-axis, which is
  numerically identical to the probe's current local z-axis here because
  `T_E_P`'s rotation is a pure `Rot_z`, which leaves the z basis vector
  unchanged.
- **`probe_incremental_force_node`**: a feedforward loop that drives
  `f_FRI.force.z` directly and is gated by contact detection
  (`measured_force_z < ~threshold_f`). x/y/orientation stay action-driven
  **concurrently**, even while engaged — this is the node to use for
  scanning under a held contact force. On contact loss the feedforward
  reference resets to 0 and re-ramps on the next contact, any number of
  times, without re-calling the enable service. `~max_force_reference` is a
  hard safety cap on the feedforward command itself, independent of the
  control loop.

## Build

```bash
cd /home/wu/Documents/project/ROS/kuka_ws
catkin_make --pkg ultrasound_probe_impedance
source devel/setup.bash
```

## Run

```bash
# admittance z-force trim (single contact events)
roslaunch ultrasound_probe_impedance probe_incremental_impedance.launch

# OR feedforward z-force hold while scanning (mutually exclusive with the above)
roslaunch ultrasound_probe_impedance probe_incremental_force.launch
```

`probe_incremental_force.launch` additionally exposes `step_duration` (3.0 s),
`recovery_action_topic`, `recovery_step_duration` (3.0 s), `threshold_f`,
`force_kp`, `target_force_z`, and `max_force_reference` as launch args (see
[Safety](#safety) for what each controls).

## Manual test

```bash
# watch the resulting absolute controller command
rostopic echo /lwr/cartesian_impedance_controller/command

# command a 10mm move along the probe's long (x) axis
# (use /probe_action_force instead if running probe_incremental_force_node)
rostopic pub -1 /probe_action_impedance geometry_msgs/Twist \
  '{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Expect the EE target logged by the node to move mostly along EE-x with a
small (~8.7%, i.e. ~0.00087 m) leak into EE-y — the signature of the 5°
calibration rotation being applied correctly (same check as in
`ultrasound_probe_motion`'s README). A pure `linear.z` command should be
calibration-invariant and match what
`ee_incremental_impedance_node`'s `~action_topic` would produce for the same
z delta, since `Rot_z` leaves z-vectors unchanged.
