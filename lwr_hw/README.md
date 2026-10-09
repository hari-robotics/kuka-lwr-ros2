# lwr_hw

ROS 2 Humble port of the original hardware package. The `include/`, `src/fri/`,
`krl/` and `doc/` layout is retained. `friComm.h`, the KRL programs and the FRI
PDF remain byte-identical to ROS 1. The ROS 2 UDP/remote copies have small fixes
for sequence initialization, error handling and an optional receive timeout;
the wire format and device command calls are preserved.

`LWRHW` now derives from `hardware_interface::SystemInterface`. The original
FRI and FRIL buffers and device calls are retained; exported interfaces replace
RobotHW resource handles. `ros2_control_node` supplies the controller-manager
loop previously owned by `lwr_hw_fri_node.cpp` / `lwr_hw_fril_node.cpp`; those
source files now export hardware plugins rather than separate executables.

Plugins:

- `lwr_hw/LWRSystemHardware` (`lwr_hw/LWRHWFRI` alias): bundled KUKA FRI backend.
- `lwr_hw/LWRHWFRIL`: original external FRIL backend, built explicitly with
  `-DLWR_HW_WITH_FRIL=ON` and an existing `FastResearchInterface.h` / `libfril`.
- `lwr_hw/LWRHWGazebo`: original simulation behavior through the Fortress
  `gz_ros2_control::GazeboSimSystemInterface` API.

The external FRIL source is not present in the ROS 1 repository. The previous
CMake fetched `iocroblab/fril` at `a10695fad32ba8315817c42020e2dbc66f4505dd`.
This port neither fetches it during an ordinary build nor substitutes a backend.
Supply that library via `FRIL_INCLUDE_DIR` and `FRIL_LIBRARY` when enabling it.

Initialization and configuration do not connect to the robot. Activation opens
FRI and waits for KRL command-mode acknowledgement; deactivation requests and
waits for monitor mode. Both confirmation waits have a 10 s deadline.
Activation first synchronizes stale KRL command acknowledgement back to monitor
and explicitly selects position strategy 10, including on a warm restart.
The commands start from measured positions. Position mode claims all seven
position interfaces; effort mode claims all seven effort interfaces; impedance
mode also claims stiffness/damping/set_point; Cartesian mode claims the original
30 scalar GPIO resources. Mixed arm-wide modes and incomplete claims are rejected.
Effort mode uses KRL strategy 30 with zero joint stiffness and additional torque.

The FRI adapter now exchanges packets from the ros2_control read cycle, so one
thread owns the unchanged `friRemote` buffers. Position/impedance calls still
prepare the command for the next exchange; no background thread races with
read/write or mode switching. `receive_timeout_ms` defaults to 100 ms and bounds
each receive (valid range 1..10000). Send/receive failures reject further commands
until reactivation; stopping releases the socket even after a failed handshake.
This bounds host-side waiting, not robot stopping time. Real communication
quality and controller behavior still require hardware validation.
The supplied KRL opens a 2 ms connection; real bringup defaults to 500 Hz.

The ROS 2 FRI adapter receives the current measurement before sending its reply,
reflecting that measurement's sequence immediately. Commands prepared by `write`
are used in the next exchange. This avoids adding a full controller update period
to the reply latency. The bundled `friRemote::doDataExchange` API itself retains
its original send/receive order for other callers.

`/lwr/emergency_stop` retains its Bool topic. The adapter holds the measured
position and clears additional effort/wrench commands while it is true. This is
a software command hold, not the controller's physical emergency stop.

Simulation retains direct position setting, the original 0.2 velocity filter,
and `additional_effort + KDL_gravity`. The original simulation's stiffness term
was disabled and remains disabled. Cartesian FRI commands remain metadata only
in Gazebo, and were never physically implemented by the ROS 1 simulator.
FRIL likewise rejects Cartesian mode because its original implementation was empty.

See [the workspace migration guide](../../MIGRATION.md) for build/run instructions
and the distinction between offline validation and real-hardware validation.

Receive-only real-robot state monitoring (no FRI replies, command-mode requests,
or controller activation):

```bash
ros2 launch lwr_hw fri_receive_only.launch.py bind_ip:=192.168.10.1 robot_ip:=192.168.10.2 port:=49938
```

This launches `/lwr/fri_state_receiver`, publishes measured radians and torques
on `/lwr/joint_states` (sensor-data QoS), and prints angles in degrees every second.
It also publishes receive freshness and FRI state on `/lwr/fri_receive_status`.
Velocity is empty because the measurement packet has no measured velocity field.
The launch uses UDP-only Fast DDS for compatibility with isolated IPC namespaces;
subscribers in the same environment should use the same transport profile.
See [the Chinese receive-only guide](../../ROBOT_RECEIVE_ONLY_GUIDE_CN.md).

Set `LWR_FRI_DIAGNOSTICS=1` in the real bringup environment to log FRI state,
quality, drive power/error bits, communication statistics, software hold, and
measured versus last transmitted joint positions once per second. This optional
logging does not change control mode or packet commands. Values reflect the FRI
buffers used by the driver, rather than an independent network capture.
