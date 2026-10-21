# Configuring the Probe Controller Step Duration

## Purpose

`probe_incremental_force_node` executes each accepted probe-frame translation
or rotation as one smooth, fixed-duration Cartesian trajectory. Ordinary
commands arrive on `/probe_action_force`; slower saved-pose/reverse-angle
returns arrive on `/probe_action_force_recovery`. The controller rejects either
kind while a previous trajectory is pending or executing.

This document covers the impedance (force-controlled) path. For the
position-control path — `ultrasound_probe_motion`'s `probe_relative_motion_node`
and `probe_scan_motion_node` driven by `gpis_probe_adjustment_motion.launch` /
`gpis_probe_adjustment_cruise.launch` / the `*_test_motion` / `*_test_cruise`
launches — see `ultrasound_probe_motion/STEP_DURATION_CONFIGURATION.md` instead.
It uses the same 3.0 s defaults but a different set of files.

The controller duration and every command publisher's waiting time must remain
synchronized. Otherwise, the controller reports:

```text
[probe_incremental_force_node] Previous step is executing or pending — ignoring new ... action.
```

An ignored action is not queued and will not be executed later.

## Commissioned configuration (3.0 s)

Both the normal and the recovery step now run over **3.0 s** — commissioned for
safety after the primary `+Y` scan moved to velocity/cruise control (so the
step duration only shapes the small GPIS adjustment increments and the
return-to-saved move, not a long stepped sweep). The earlier value was 0.25 s.

Keeping the duration synchronized requires coordinated values in:

- `probe_incremental_force_node` — 3.0 s C++ fallback, reads `~step_duration` /
  `~recovery_step_duration`, rejects non-finite / non-positive values at startup.
- `probe_incremental_force.launch` — `step_duration` / `recovery_step_duration`
  args, both `3.0`, passed into the node (the value used on the normal
  `roslaunch` path).
- `gpis_probe_adjustment_params.yaml` — `controller_step_duration_sec: 3.0`,
  `recovery_step_duration_sec: 3.0`; the GPIS node's C++ fallback matches.
- `probe_motion_adjustment_test_params.yaml` —
  `controller_step_duration_sec: 3.0`, `recovery_step_duration_sec: 3.0`,
  `inter_command_margin_sec: 0.5`. Ordinary commands wait 3.5 s; reverse-angle
  returns wait 3.5 s.
- The package README and open-loop test guide describe the 3.0 s duration.

`probe_z_rotation_deg` (shared calibration) is independent and is not part of
this procedure.

## Current Timing

```text
step_duration          = 3.0 s
recovery_step_duration  = 3.0 s
```

The open-loop hardware test waits an additional safety margin:

```text
controller_step_duration_sec + inter_command_margin_sec
= 3.0 s + 0.5 s
= 3.5 s between published actions

recovery_step_duration_sec + inter_command_margin_sec
= 3.0 s + 0.5 s
= 3.5 s after a recovery action
```

## Settings That Must Match

### 1. Force-controller launch file

In `ultrasound_probe_impedance/launch/probe_incremental_force.launch`:

```xml
<arg name="step_duration" default="3.0"/>
<arg name="recovery_step_duration" default="3.0"/>
...
<param name="step_duration" value="$(arg step_duration)"/>
<param name="recovery_step_duration" value="$(arg recovery_step_duration)"/>
```

This is the value used when the node is started normally with `roslaunch`.

### 2. Force-controller C++ fallback

In `ultrasound_probe_impedance/src/probe_incremental_force_node.cpp`:

```cpp
double T = 3.0;
double recovery_T = 3.0;
...
pnh.param("step_duration", T, T);
pnh.param("recovery_step_duration", recovery_T, recovery_T);
```

The C++ value is only a fallback when the private ROS parameter is absent. The
launch value takes precedence. The node rejects non-finite or non-positive
durations during startup.

### 3. Action publishers

The open-loop test value is in
`ultrasound_vessel_reconstruction/config/probe_motion_adjustment_test_params.yaml`:

```yaml
controller_step_duration_sec: 3.0
recovery_step_duration_sec: 3.0
inter_command_margin_sec: 0.5
```

The GPIS executor's declared controller duration is in
`ultrasound_vessel_reconstruction/config/gpis_probe_adjustment_params.yaml`:

```yaml
controller_step_duration_sec: 3.0
recovery_step_duration_sec: 3.0
```

When changing either duration, update the corresponding values in all three
locations together. Do not reduce the open-loop margin until command timing has
been verified on the real system.

## Safe Revision Procedure

1. Stop the open-loop test, GPIS live executor, and force-controller node.
2. Choose the normal and recovery durations and evaluate the resulting speeds.
3. Update both force-launch defaults and C++ fallbacks.
4. Update both vessel-reconstruction timing configurations to the same value.
5. Keep `controller_timeout_sec` greater than both durations and any expected
   tracking/settling delay.
6. Rebuild both packages.
7. Relaunch the force controller; parameters are read only during startup.
8. Confirm the loaded parameter before commanding motion.
9. Test at low-risk motion amplitudes with the emergency stop available.

Build commands:

```bash
cd /home/wu/Documents/project/ROS/kuka_ws
catkin_make --pkg ultrasound_probe_impedance ultrasound_vessel_reconstruction
source devel/setup.bash
```

Verify the runtime duration:

```bash
rosparam get /probe_incremental_force_node/step_duration
rosparam get /probe_incremental_force_node/recovery_step_duration
```

Expected output for the current configuration:

```text
3.0
3.0
```

## Speed Implications

The node uses the normalized quintic time law:

```text
Sigma(r) = 10r^3 - 15r^4 + 6r^5
```

Its peak derivative is 1.875. Approximate peak Cartesian speeds are therefore:

```text
peak linear speed  = 1.875 * translation distance / step duration
peak angular speed = 1.875 * rotation angle / step duration
```

For the current 3.0 s normal duration:

| Action | Approximate peak speed |
|---|---:|
| 1 mm translation | 0.63 mm/s |
| 5 mm translation | 3.1 mm/s |
| 1 degree rotation | 0.63 deg/s |
| 3 degree rotation | 1.9 deg/s |

Shortening the duration increases both linear and angular peak velocity. The
same duration applies to every accepted action regardless of its magnitude, so
the largest configured translation and rotation must be used for safety review.

## Diagnosing Rejected Commands

If `Previous step is executing or pending` appears:

1. Read the controller's startup log and confirm the reported `T` value.
2. Read `/probe_incremental_force_node/step_duration` from the parameter server.
3. Check the publisher interval in its YAML configuration.
4. Inspect both action topics for unexpected publishers:

   ```bash
   rostopic info /probe_action_force
   rostopic info /probe_action_force_recovery
   ```

5. Ensure no manual `rostopic pub` process is repeatedly publishing.

For the open-loop test, the publication interval must be at least:

```text
controller step duration + inter-command safety margin
```

The controller does not acknowledge or queue rejected actions. Successful
operation therefore depends on the publisher waiting long enough before sending
the next command.

## Calibration Is Independent

`probe_length`, `probe_y_offset`, and `probe_z_rotation_deg` define the fixed
flange-to-probe transform. They do not control trajectory duration. Their
runtime values are loaded from
`ultrasound_probe_motion/config/probe_calibration.yaml` and may override the
C++ fallback values without affecting the timing procedure described here.
