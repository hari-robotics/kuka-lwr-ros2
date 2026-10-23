# Configuring the Probe Motion Step Duration

## Purpose

`probe_relative_motion_node` executes each accepted probe-frame translation or
rotation as one smooth, fixed-duration trajectory (quintic time law +
quaternion SLERP) sent as an absolute `PoseRPY` to the
`one_task_inverse_kinematics` position controller.

- Ordinary commands arrive on `~action_topic` (default `/probe_action`).
- Slower saved-pose / reverse-angle returns arrive on `~recovery_action_topic`
  (default `/probe_action_recovery`; the recovery channel is on unless you pass
  `recovery_action_topic:=""`).

The legacy `Twist` topics run one trajectory at a time. A legacy command received
while a trajectory is still executing is dropped:

```text
[probe_relative_motion_node] Still executing previous step -- ignoring new action.
```

New orchestrators should use the `/probe_step` action instead. It rejects busy
goals explicitly and reports success only after measured pose has settled, so
publisher timing does not need to mirror the trajectory duration. Open-loop
legacy publishers must still synchronize their waits. This is the same rule as
`ultrasound_probe_impedance/STEP_DURATION_CONFIGURATION.md`; only the files,
defaults, and a few behaviours differ.

For the GPIS cruise publisher, intermediate adjustment and capped-return goals
use a relaxed 1.0 deg / 0.05 s pose gate. The final saved-pose return keeps the
strict 0.5 deg / 0.25 s gate. Thus the interval visible between chunk log lines
is trajectory plus measured convergence time, not an added inter-command sleep;
the GPIS scheduler itself re-enters in about 20 ms.

## Same procedure as the impedance path

The defaults match `ultrasound_probe_impedance`'s `probe_incremental_force_node`:
**3.0 s** normal and recovery (commissioned for safety after primary `+Y` moved
to velocity/cruise control, so `step_duration` only shapes the small GPIS
adjustment increments and the return move). The change procedure is identical:
three location-types must agree (launch arg ↔ C++ fallback ↔ downstream publisher
configs), and calibration is unrelated to timing.

Both motion nodes share these params and defaults: `probe_relative_motion_node`
(`probe_motion.launch`, stepped path) and `probe_scan_motion_node`
(`probe_scan_motion.launch`, cruise path).

Two behaviours still differ from the impedance node:

| | `probe_incremental_force_node` | `probe_relative_motion_node` / `probe_scan_motion_node` |
|---|---|---|
| bad duration value | **throws at startup** | **warns and keeps the previous value** — always confirm with `rosparam get` |
| low-level controller | Cartesian impedance (compliant) | `one_task_inverse_kinematics` (stiff position) — shortening the duration is riskier here |

Also: the GPIS / open-loop-test launches do **not** start the motion node;
the operator runs `probe_motion.launch` or `probe_scan_motion.launch` separately,
so its `step_duration:=` is the runtime source of truth.

**Cruise variant:** with `probe_scan_motion_node` primary `+Y` is a
constant-velocity sweep, not stepped — `step_duration` / `recovery_step_duration`
there only affect the adjustment increments and the return-to-saved move.
Primary Y is tuned by `cruise_speed_mps` (config) + `cruise_accel` /
`cruise_speed_max` (`probe_scan_motion.launch`). See `PROBE_SCAN_MOTION.md`.

## Current timing

```text
step_duration          = 3.0 s   (probe_motion.launch, stepped path)
recovery_step_duration = 3.0 s

step_duration          = 1.0 s   (probe_scan_motion.launch, cruise / GPIS path)
recovery_step_duration = 0.5 s
```

The cruise path uses 1.0 s trajectories for the GPIS excursion and return.
`gpis_probe_adjustment_node` sub-steps rotations into
`max_command_increment_deg` chunks and sends acknowledged absolute pose goals;
the next goal is not sent until measured pose settles within tolerance. Peak
angular speed of a 2.5° chunk is ~4.7°/s at
1.0 s (vs ~1.6°/s at 3.0 s); `one_task_inverse_kinematics` is stiff position
control with no compliance, so drop `step_duration` back toward 3.0 (and raise
`max_command_increment_deg` / `controller_timeout_sec` to match) if working near
tissue.

The open-loop hardware test waits an additional safety margin:

```text
controller_step_duration_sec + inter_command_margin_sec = 3.0 + 0.5 = 3.5 s
recovery_step_duration_sec  + inter_command_margin_sec  = 3.0 + 0.5 = 3.5 s
```

## Settings that must match

### 1. Motion launch files

`ultrasound_probe_motion/launch/probe_motion.launch` **and**
`ultrasound_probe_motion/launch/probe_scan_motion.launch`:

```xml
<arg name="step_duration" default="3.0"/>
<arg name="recovery_step_duration" default="3.0"/>
<arg name="recovery_action_topic" default="/probe_action_recovery"/>   <!-- probe_motion.launch only -->
...
<param name="step_duration" value="$(arg step_duration)"/>
<param name="recovery_step_duration" value="$(arg recovery_step_duration)"/>
```

This is the value used when the node is started normally with `roslaunch`.
For `probe_motion.launch`, `recovery_step_duration` has no effect if
`recovery_action_topic` is set to `""` (the recovery subscriber is never
created); `probe_scan_motion_node` always has both channels.

### 2. Motion node C++ fallback

`ultrasound_probe_motion/src/probe_relative_motion_node.cpp` **and**
`ultrasound_probe_motion/src/probe_scan_motion_node.cpp`:

```cpp
double T_          = 3.0;   // normal step duration
double recovery_T_ = 3.0;   // recovery step duration
...
load_duration("step_duration", T_);
load_duration("recovery_step_duration", recovery_T_);
```

The C++ values are only fallbacks used when the private ROS parameter is absent;
the launch value takes precedence. `load_duration` rejects a non-finite or
non-positive value with a warning and keeps the current value — it does **not**
abort startup, so a typo is silent. Always verify (below).

### 3. Paced publishers (downstream consumers)

Only nodes that pace themselves by the controller duration need updating. On the
motion path those are the vessel-reconstruction configs — the **stepped** pair:

- `gpis_probe_adjustment_motion_params.yaml`
- `probe_motion_adjustment_test_motion_params.yaml`

and the **cruise** pair (`step_duration` here only shapes the excursion + return):

- `gpis_probe_adjustment_cruise_params.yaml`
- `probe_motion_adjustment_test_cruise_params.yaml`

The stepped / `*_test_*` configs use:

```yaml
controller_step_duration_sec: 3.0
recovery_step_duration_sec: 3.0
# gpis_* also: controller_timeout_sec: 9.0   (must stay > max(step, recovery) + settling)
# *_test_* also: inter_command_margin_sec: 0.5
```

`gpis_probe_adjustment_cruise_params.yaml` uses the shortened cruise timing:

```yaml
controller_step_duration_sec: 1.0
recovery_step_duration_sec: 0.5
controller_timeout_sec: 5.0   # > max(step, recovery); headroom for the final return chunk
```

Notes:

- `gpis_probe_adjustment_node` refuses to start live if
  `controller_timeout_sec` is not greater than both durations.
- The consumer nodes' own C++ fallbacks (`gpis_probe_adjustment_node.cpp`,
  `probe_motion_adjustment_test_node.cpp`) are also `3.0`, but the `*_params.yaml`
  values are what actually apply — keep them in sync explicitly.
- Leave the impedance configs (`gpis_probe_adjustment_params.yaml`,
  `probe_motion_adjustment_test_params.yaml`) alone — they belong to the
  impedance path.

When changing either duration, update all locations together.

## Safe revision procedure

1. Stop the open-loop test, GPIS live executor, and `probe_relative_motion_node`.
2. Choose the normal and recovery durations and evaluate the resulting speeds
   (see below). Remember position control has no compliance.
3. Update the `probe_motion.launch` arg defaults **and** the C++ fallbacks in
   `probe_relative_motion_node.cpp`.
4. Update both `*_motion_params.yaml` to the same values; keep
   `controller_timeout_sec` greater than both durations plus any settling delay.
5. Rebuild:
   ```bash
   cd ~/kuka_ws
   colcon build --packages-select ultrasound_probe_motion
   source install/setup.bash
   ```
6. Relaunch the motion node (parameters are read only at startup):
   ```bash
   roslaunch ultrasound_probe_motion probe_motion.launch
   ```
7. Confirm the loaded values before commanding motion:
   ```bash
   rosparam get /probe_relative_motion_node/step_duration
   rosparam get /probe_relative_motion_node/recovery_step_duration
   ```
   and check the startup log:
   ```text
   [probe_relative_motion_node] Recovery channel enabled on /probe_action_recovery (T=3.00 s)
   [probe_relative_motion_node] Ready (T=3.00 s). Waiting for first pose on ...
   ```
8. Test at low-risk motion amplitudes with the emergency stop available; watch
   for `Still executing previous step -- ignoring new action`.

**One-off change (no rebuild):** pass `step_duration:=X recovery_step_duration:=Y`
to `probe_motion.launch` and set the same `controller_step_duration_sec` /
`recovery_step_duration_sec` in the YAML you are about to load. The rebuild path
(steps 3–5) is only for changing the persistent defaults.

## Speed implications

The node uses the normalized quintic time law:

```text
Sigma(r) = 10r^3 - 15r^4 + 6r^5
```

Its peak derivative is 1.875, so approximate peak Cartesian speeds are:

```text
peak linear speed  = 1.875 * translation distance / step_duration
peak angular speed = 1.875 * rotation angle       / step_duration
```

For the current 3.0 s normal duration:

| Action | Approximate peak speed |
|---|---:|
| 1 mm translation | 0.63 mm/s |
| 5 mm translation | 3.1 mm/s |
| 1 degree rotation | 0.63 deg/s |
| 3 degree rotation | 1.9 deg/s |

Shortening the duration raises both linear and angular peak velocity in
proportion (a 0.25 s step is 12× faster). The same duration applies to every
accepted action regardless of magnitude, so review safety against the largest
configured translation and rotation. Position control here has no compliance, so
prefer a longer duration when working near tissue.

`probe_relative_motion_node` additionally hard-clamps any single command to
`0.25 m` / `2 rad` (compile-time constants `MAX_LINEAR_STEP` / `MAX_ANGULAR_STEP`).

## Diagnosing rejected commands

If `Still executing previous step -- ignoring new action` appears:

1. Read the node's startup log and confirm the reported `T` value.
2. Read `/probe_relative_motion_node/step_duration` (and
   `recovery_step_duration`) from the parameter server.
3. Check the publisher's YAML `controller_step_duration_sec` /
   `recovery_step_duration_sec` — each must be **≥** the node's duration
   (`inter_command_margin_sec` is the slack on top of an equal match).
4. Inspect both action topics for unexpected publishers:
   ```bash
   rostopic info /probe_action
   rostopic info /probe_action_recovery
   ```
5. Ensure no manual `rostopic pub` process is repeatedly publishing.

Rejected actions are never queued and will not run later.

## Calibration is independent

`probe_length`, `probe_y_offset`, and `probe_z_rotation_deg` (loaded from
`config/probe_calibration.yaml`) define the fixed flange-to-probe transform.
They do not affect trajectory duration and are unrelated to this procedure.

## See also

`ultrasound_probe_impedance/STEP_DURATION_CONFIGURATION.md` — the same procedure
and the same 3.0 s defaults for the impedance (force-controlled) path; different
files, and the force node throws on a bad duration instead of warning.
