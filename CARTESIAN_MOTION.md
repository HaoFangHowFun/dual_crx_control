# Separate robot bringup and Cartesian motion

Robot bringup and motion are independent. Start either the mock or physical
launch, then run `dual_test_5_cartesion_sychro_motion.py` in a second terminal.
Neither launch starts the motion script. The script uses the same topics, joint
names, FK/IK, and trajectory logic in both environments. The new 50 Hz / 500 Hz
linear-interpolation path has been verified in the mock only; it has not been
tested on physical robots.

The motion script gets the calibrated dual-arm URDF from the running launch's
latched `/robot_description` topic, including its base transforms and physical
TCP offsets. It then waits for both arms' live joint feedback and command
subscribers before moving both arms from their measured positions to the configured
initial joints. It captures the Cartesian starting poses only after both arms
reach and hold their initial targets. An explicit `robot_description`
ROS parameter remains available for tests or custom integration.

## Independent joint interpolation

See [INTERPOLATION.md](INTERPOLATION.md) for the current interface and launch parameters.
`rate:=50.0` controls motion/IK target generation. The separate `joint_interpolation`
node resamples joint sequences at a hardcoded 500 Hz, using `method:=linear` or
`method:=cubic`. It performs no Cartesian computation. Both mock and real motion
bringup include it once; scripts publish joint targets and never controller commands.
Set bringup `input_rate_hz` to match the motion rate. The old `command_rate` option
has been removed. Initial moves remain upstream quintic joint plans.

Interpolation validates joint data only. Motion planning keeps its existing
limits and feedback checks. When targets stop, interpolation completes the last
segment and continuously publishes its endpoint at 500 Hz. Finite motion observes
the command report to confirm its last target; there is no session or watchdog.

The measurements below predate the node split; current results are documented in
`INTERPOLATION_TEST_RESULTS.md`.

The RViz mock acceptance run measured 50.00 Hz IK and about 500.03 Hz publication,
with observed command rates of about 500 Hz on both topics. These are measured
averages, not hard real-time guarantees. The mock keeps its existing 100 Hz
feedback publication rate; it consumes the same 500 Hz command interface.

## Build the full workspace

```bash
cd /home/msc-crx/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the build in a fresh terminal with the ROS Jazzy underlay sourced. This
builds every package in the workspace using the standard `build/`, `install/`,
and `log/` directories. There is no separate build or install inside this package.

## Simulation: two terminals

Terminal 1 — start the stationary software robots and RViz:

```bash
cd /home/msc-crx/ws_fanuc
source install/setup.bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true
```

Mock mode uses ros2_control GenericSystem with the same forward position
controllers as hardware mode, plus interpolation and the calibrated global TF.
RViz defaults to enabled; use `rviz:=false` for headless bringup. Other options
include `input_rate_hz`, `method`, and the per-arm IP addresses. Neither mode
starts a motion script or overrides the terminal's ROS discovery configuration.

Terminal 2 — explicitly start the motion:

```bash
cd /home/msc-crx/ws_fanuc
source install/setup.bash
ros2 run dual_crx_control dual_test_5_cartesion_sychro_motion.py
```

Use two fresh terminals with the same normal ROS configuration. No mock-specific
domain export is required. If an older terminal still has the previous manual
domain-174 override, open a fresh terminal and restart the launch and script.
Neither executable changes the terminal's ROS domain or discovery settings.

By default, the script first moves to the initial joint poses in the table
below. It uses a shared quintic joint interpolation, with a minimum duration of
5 seconds, maximum joint speed of 0.1 rad/s, and maximum acceleration of
0.1 rad/s². Larger moves automatically take longer. Both arms must remain within
0.005 rad (about 0.29°) of their targets for 0.5 seconds before Cartesian motion
starts. Missing feedback, excess tracking error, or a settling timeout aborts
the sequence. A robot already at its target simply holds during this stage.

This is a direct joint-space move, not obstacle avoidance: its entire swept path
must be clear before running it on hardware. No physical motion was executed
while implementing or testing this feature.

Use `--ros-args -p move_to_initial:=false` to retain the earlier behavior of
starting the Cartesian trajectory around the current measured pose.

Default Cartesian motion is a 20 mm amplitude, 4 s period sine along world X at 50 Hz,
with fixed initial orientation and a one-second smootherstep amplitude ramp.
Motion parameters belong to the script, for example:

```bash
ros2 run dual_crx_control dual_test_5_cartesion_sychro_motion.py \
  --ros-args -p axis:=z -p amplitude:=0.01 -p period:=6.0 -p rate:=50.0
```

Ctrl+C in Terminal 2 stops the target stream. Interpolation completes its last
segment and keeps publishing the held joint positions at 500 Hz; and RViz remains open. Restarting the motion
script repeats the move to the configured initial joints before Cartesian motion. Ctrl+C in Terminal 1
closes bringup; restarting bringup resets the mock to these initial angles:

| Arm | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| left | 0 | 0 | 0 | 0 | -90 | 0 |
| right | -90 | 0 | 180 | 0 | 90 | 0 |

Initial world TCP positions are approximately `[0.430, -0.130, 0.415]` m and
`[0.44750425, -0.53384931, 0.42546828]` m. The script logs the complete initial
poses and joints when it starts motion.

## Physical bringup prepared for the upcoming test

`dual_arm.launch.py mock:=false` uses the same node definitions as mock mode:

- Right robot: `192.168.1.100`, namespace `/right`, prefix `right_`.
- Left robot: `192.168.2.100`, namespace `/left`, prefix `left_`.
- Each controller manager activates joint_state_broadcaster and the six-joint
  forward position controller through dedicated spawners.
- `motion_control:=1` enables the physical driver's motion authority, as in the
  existing motion-capable launch. This is physical bringup, not a read-only launch.
- A joint-state publisher merges the arms' feedback for the combined calibrated
  robot_state_publisher. The motion script reads the original per-arm feedback.
- Driver-local TF is remapped to `/<arm>/driver_tf` and
  `/<arm>/driver_tf_static`, so the calibrated dual model exclusively owns
  global `/tf` and `/tf_static`.
- No trajectory or motion-command script starts from this launch.

For the future hardware session, the bringup command is:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=false
```

Optional arguments: `left_robot_ip`, `right_robot_ip`, and `rviz`. The physical
launch inherits the terminal's established hardware ROS configuration; both
terminals must use that same configuration. Keep physical bringup and software
mock bringup in separate sessions because they expose the same command topics.

The same motion executable can then be run separately. A conservative first
hardware trajectory would use:

```bash
ros2 run dual_crx_control dual_test_5_cartesion_sychro_motion.py \
  --ros-args -p amplitude:=0.002 -p period:=8.0 -p rate:=50.0 -p cycles:=1
```

With `cycles:=1`, the Cartesian stage ramps down to its starting TCP targets
after one 8-second cycle, holds for 0.5 seconds, then stops its command stream.
The preceding move to initial joints takes additional time. The default
`cycles:=0` repeats Cartesian motion indefinitely.

These physical commands were **not executed**. The new launch was inspected and
its launch arguments validated without starting the drivers. Actual hardware
behavior remains unverified. Mock and physical modes now share the same launch and joint-target interface.

## Interfaces and checks

Both environments provide:

| Interface | Type / ordering |
| --- | --- |
| `/robot_description` | Transient-local `std_msgs/String`, calibrated dual URDF |
| `/left/joint_states`, `/right/joint_states` | `sensor_msgs/JointState`, matched by joint name |
| `/left/forward_position_controller/commands` | `Float64MultiArray`, `left_J1` through `left_J6`, radians |
| `/right/forward_position_controller/commands` | `Float64MultiArray`, `right_J1` through `right_J6`, radians |
| `/joint_states` | Joint feedback for the combined visualization model |

The custom NumPy DLS solver uses PyKDL FK/Jacobians and full position plus
rotation-vector error. There is no MoveIt and no duplicated base/TCP transform.
The previous accepted IK target seeds the next solution. Both arms share one
monotonic time and phase; both commands are checked before either is published,
then sent back-to-back without sleeping or spinning between publications.
Independent ROS topics do not provide an atomic hardware transaction.

During Cartesian motion, invalid/missing feedback or rejected IK targets keep
both arms on the previous segment, as described above. Upstream target step/velocity violations and startup failures stop new targets;
interpolation retains the last accepted endpoint. A model
change also requires a restart. Feedback freshness uses monotonic receipt time,
including a check after IK before a new segment is accepted.

All settings below are ROS parameters on the motion script:

| Parameter | Default |
| --- | ---: |
| `axis` | `x` |
| `amplitude` | 0.02 m |
| `period` | 4.0 s |
| `rate` | 50 Hz Cartesian target/IK updates |
| interpolation output | fixed 500 Hz in the separate node |
| `max_target_step` | 0.1 rad maximum joint change between accepted targets |
| `ramp_time` | 1.0 s |
| `cycles` | 0 (repeat indefinitely); positive integer for finite motion |
| `final_hold` | 0.5 s after finite Cartesian motion |
| `move_to_initial` | `true` |
| `left_initial_deg` | `[0., 0., 0., 0., -90., 0.]` |
| `right_initial_deg` | `[-90., 0., 180., 0., 90., 0.]` |
| `initial_move_time` | 5.0 s minimum; extended to respect speed/acceleration limits |
| `initial_max_velocity` | 0.1 rad/s, also capped by `max_velocity` and URDF limits |
| `initial_max_acceleration` | 0.1 rad/s² |
| `initial_tolerance` | 0.005 rad maximum error per joint |
| `initial_settle_time` | 0.5 s continuously inside tolerance |
| `initial_settle_timeout` | 5.0 s after interpolation ends |
| `initial_tracking_tolerance` | 0.1 rad maximum feedback error against previous command |
| `state_timeout` | 0.25 s |
| `ready_timeout` | 10 s |
| `max_cycle_step` | 0.03 rad |
| `max_velocity` | 0.5 rad/s, also bounded by URDF velocity limits |
| `damping` | 0.01 |
| `position_tolerance` | 0.00001 m |
| `orientation_tolerance` | 0.00001 rad |
| `max_iterations` | 100 |
| `ik_max_joint_step` | 0.05 rad |
| `ik_alpha` | 1.0 |

The ideal mock's hold behavior does not establish physical stopping behavior.
This experiment has no collision checking or hardware emergency stopping.
Tracking-error supervision applies to the initial joint move; the Cartesian
stage retains its joint-step, velocity, IK, and feedback-freshness checks. Those remain part of preparing a physical test.

## Command versus measured motion-axis plot

The motion script now automatically saves a plot after a finite run finishes,
after an abort with recorded Cartesian data, or after Ctrl+C. Run commands do
not change. When launched from `~/ws_fanuc`, output is in:

```text
~/ws_fanuc/cartesian_motion_results/
  axis_x_YYYYMMDD_HHMMSS_microseconds.png
  axis_x_YYYYMMDD_HHMMSS_microseconds.csv
  axis_x_YYYYMMDD_HHMMSS_microseconds.json
```

The terminal prints the exact saved paths. Each PNG has a left and a right TCP
panel, comparing commanded and measured displacement in millimetres along the
configured world `axis`. Both traces use the same measured Cartesian starting
pose as their zero. The initial joint-return phase is excluded, so it does not
hide the small Cartesian motion.

The command trace comes from FK of the joint positions actually published at
500 Hz, not from the desired sine or the 50 Hz IK targets. The measured trace
comes from FK of incoming `/left/joint_states` and `/right/joint_states` at their
native feedback rates. It is a joint-sensor-derived TCP estimate, not an external
TCP measurement. Command times and feedback receipt times share one local
monotonic clock; feedback transport delay is included. Original feedback ROS
header timestamps are also preserved in the CSV.

During motion the recorder only copies joints and timestamps into bounded memory
buffers (60,000 command pairs and 60,000 feedback samples per arm). Plotting,
FK for the plot, and disk output happen after commands stop. Very long runs keep
only the newest samples; the JSON flags truncation. Forced termination such as
SIGKILL cannot save the in-memory recording. Earlier runs made before this
recording feature cannot be reconstructed from the existing rate summaries.

To change the destination or disable recording, append respectively:

```bash
-p output_dir:=/your/output/directory
-p save_plot:=false
```

The installed-launch mock test saves its demonstration plots inside
`src/dual_crx_control/test_results/cartesian_mock/motion_plots/` and checks that
the data and plot are produced on Ctrl+C.

## Verification

After building and sourcing the workspace installation:

```bash
cd /home/msc-crx/ws_fanuc
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
colcon test --packages-select dual_crx_control \
  --ctest-args -R '^(cartesian|interpolation)$' --output-on-failure
colcon test-result --test-result-base build/dual_crx_control --verbose
/usr/bin/python3 src/dual_crx_control/tests/verify_cartesian_mock.py
```

Regression tests cover independent FK, finite-difference Jacobians,
full sine cycles on all axes, invalid/unreachable inputs, paired-command
failures, live ROS feedback loss, missing/malformed bringup descriptions,
initial-move speed/acceleration limits, initial tracking/settling failures, and
a full finite 2 mm cycle after returning a displaced mock to the initial joints.
They also check linear interpolation endpoints/midpoint/clamping, invalid inputs,
shared phase, held targets after rejection, and absence of IK on intermediate
command updates.
The full acceptance check uses localhost domain 174 with headless GenericSystem bringup. It first proves bringup is stationary and emits no motion commands,
then starts the installed motion executable as a separate process. It checks
TCP motion/orientation, TF, both achieved rates, and continued feedback after
stopping that process,
then closes the mock launch. Its files are under `src/dual_crx_control/test_results/cartesian_mock/` (relative to the workspace):
`launch.log`, `motion.log`, `metrics.json`, `tcp_tracking.png`, and `rviz.png`.

## Shared implementation

Robot bringup is in `launch/dual_arm.launch.py`; controller configuration is in
`config/dual_arm_controllers.yaml`. Startup and Cartesian/circle controllers are
in `scripts/motion/`. Joint-only interpolation is in `src/dual_crx_control/`.
`dual_mock_robot.py` remains a lightweight unit-test tool; the launch uses
GenericSystem. Current validation results are in `INTERPOLATION_TEST_RESULTS.md`.

## Separate circular-motion experiment

Use `dual_test_6_cartesian_circle_motion.py` for simultaneous TCP circles.
See [CIRCLE_MOTION.md](CIRCLE_MOTION.md) for radius/plane/direction options, mock
run commands, and circle plots. The sinusoidal executable remains available;
its controller implementation now lives in the shared
`scripts/motion/cartesian_controller.py` module.
