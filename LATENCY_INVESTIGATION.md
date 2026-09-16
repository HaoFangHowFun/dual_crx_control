# FANUC command-to-feedback latency investigation

## Physical trial follow-up

The operator subsequently completed instrumented physical trial 01. It measured
approximately 131 ms command-to-observer lag, including about 118 ms from the
send boundary to returned state, plus a separate 19 ms application callback lag.
Stream Motion exchanged commands/status at 125 Hz despite the 500 Hz ROS loop.
See [trial analysis](docs/latency/TRIAL_01_ANALYSIS.md),
[plot](docs/latency/trial_01_breakdown.png), and
[verified measurements](docs/latency/trial_01_verified_report.json).
The earlier findings and pre-trial validation below describe the investigation
before that operator-run test. Raw CSV recordings remain local.

## Initial findings as of 2026-09-15

The existing recordings confirm approximately **96–100 ms apparent phase lag**
on the main moving joints. They do not yet locate it inside the controller.
No physical motion was executed for this investigation.

The filenames below identify the previous experiment settings; the CSV metadata
does not independently record the driver's buffer parameter. These files contain
Cartesian X sine tests and all six joint positions, not isolated J1 sine tests.
Left J1 barely moves, so estimating its lag is unreliable; J2 is a stronger signal.

| Recording label | Left J2 lag | Right J2 lag | Left/right fitted gain |
| --- | ---: | ---: | --- |
| `latency_equal_8` | 96 ms | 96 ms | 1.0039 / 1.0015 |
| `latency_equal_16` | 114 ms | 116 ms | 0.9703 / 0.9596 |
| `latency_equal_2` | 95 ms | 97 ms | 0.8806 / 0.8900 |
| `period_2s` | 97 ms | 98 ms | 0.9992 / 1.0007 |
| `period_4s` | 100 ms | 99 ms | 0.9955 / 1.0001 |
| `period_8s` | 97 ms | 97 ms | 1.0022 / 1.0094 |

The period comparison supports a mostly fixed delay over these trials. It does
not exclude low-frequency filter group delay or controller dynamics. The buffer
16 comparison is consistent with additional buffering; buffer 2 leaves most of
the lag and changes the amplitude, so reducing it further is not the next step.
There are no repeated trials/confidence intervals to establish statistical
significance for the few-millisecond differences.

Analysis uses normalized cross-correlation of independently timestamped joint
signals. It evaluates `command(t)` against `feedback(t + lag)` on a common,
fixed-overlap 1 ms grid over ±500 ms; positive lag means feedback follows command.
Default trims exclude the first 1.5 s and last 1 s, plus 0.5 s at each boundary
for the lag search. It rejects stationary commands and large internal sampling
gaps. A fitted gain, bias, correlation and residual are included. A 1 ms search
grid is numerical resolution, not a claim of 1 ms measurement accuracy. Periodic
waveforms measure phase delay; they cannot uniquely identify pure transport delay.

Generated results:

- `test_results/latency/existing_recordings.json`: all joints and timing statistics.
- `test_results/latency/existing_comparisons.png`: buffer and period comparisons.
- `test_results/latency/passive_stationary.csv`: five-second passive live capture.

Both physical arms currently report an **active forward_position_controller**,
an **inactive scaled joint_trajectory_controller**, and an active GPIO controller.
The stationary capture retained about 2,400 feedback and scaling samples per arm
(after discovery); speed scaling was exactly 1.0 throughout. It contained no
command messages. This does not establish scaling during motion. The prior
verified 500 Hz controller/feedback rates remain accepted; they were not retested.

## Exact path and possible delay stages

```text
application target calculation / publish(Float64MultiArray)
  → DDS subscriber callback (forward_command_controller)
  → latest-command realtime box
  → controller_manager: read → update → write
  → FanucHardwareInterface::write: rad → degrees
  → FanucClient::writeJointTarget: timestamp and command queue
  → separate streamMotionThread, paced by returned status packets
  → queue-depth clock adjustment + linear interpolation
  → StreamMotionConnection::sendCommand → UDP socket send
  → FANUC internal processing and servos (not visible in host source)
  → UDP status packet → getStatusPacket → client state queue
  → readJointAngles drains queue to latest state
  → hardware read: degrees → rad
  → joint_state_broadcaster update / realtime publisher / DDS
  → application or passive observer callback
```

Source references (relative to `~/ws_fanuc/src/fanuc_driver`):

- `fanuc_hardware_interface/src/hardware_interface.cpp`: `read`, `write`.
- `fanuc_libs/fanuc_client/src/fanuc_client.cpp`: `writeJointTarget`,
  `streamMotionThread`, `readStateFromQueue`, `startRealtimeStream`.
- `fanuc_libs/stream_motion/src/stream.cpp`: socket `send`/`receive`,
  `sendCommand`, `getStatusPacket`.
- `fanuc_libs/stream_motion/include/stream_motion/packets.hpp`: controller sequence,
  raw timestamp, joint angles, safety scale.
- `fanuc_controllers/src/fanuc_gpio_controller.cpp`: safety-scale publishing.

The matching upstream ROS sources were inspected for installed versions
`ros2_controllers 4.40.1` and `ros2_control 4.45.2`, downloaded into
`test_results/latency/upstream/` from the matching GitHub tags:

- `forward_command_controller/src/forward_controllers_base.cpp`: callback stores
  latest command; update reads the realtime box and directly writes interfaces.
  It retains the previous reference if access fails; there is no trajectory
  smoothing in this controller. DDS subscription uses SystemDefaultsQoS.
- `controller_manager/src/ros2_control_node.cpp`: synchronous read/update/write.
- `joint_state_broadcaster/src/joint_state_broadcaster.cpp`: header stamp is the
  controller update timestamp, **not the robot's original sampling timestamp**.

| Stage | What can add delay | Priority |
| --- | --- | --- |
| Old joint-sine loop | `spin_once` once per command plus sleep; cached feedback can age; actual period includes processing time | Measure application vs passive feedback |
| Cartesian script | 50 Hz IK and a 20 ms interpolated segment; CSV commands are recorded after interpolation at publication | Can lag ideal target; cannot by itself explain lag between recorded published joints and measured joints |
| DDS / forward controller | transport, executor scheduling, callback queue, missed latest-reference access | Measure application publication to hardware write as a combined stage |
| controller_manager | phase to next 2 ms update; scheduling stalls | 500 Hz configuration is already verified; not proof of bounded latency |
| FANUC client command queue | every write enqueues timestamped joints; queue is not simply latest-value | Measure queue depth and represented command age |
| FANUC interpolator | `drift = .99*drift + (queue_size-target)*1e-6`; advances virtual time by controller-reported period plus drift; clamped linear interpolation | Target 8 suggests ~16 ms only if writes/status are actually 2 ms and steady; measure rather than assume |
| UDP / status pacing | nonblocking receive polls with 100 µs sleeps; reads one packet; socket backlog possible | Log send boundaries and status-return intervals/sequence gaps |
| Controller/servo | internal buffering, smoothing, collaborative clamp, dynamics | Unresolved; needs instrumented motion and Stream Motion documentation |
| Feedback return | status queued after command send; hardware drains to newest; publisher and subscriber scheduling | Compare raw status → hardware read → observer/app receipt |

The driver's interpolation timeline currently uses `high_resolution_clock`.
The added host event timestamps use `CLOCK_MONOTONIC`; trajectory timing has not
been changed. The derived `represented_age_s` intentionally uses the driver's
existing clock domain and must be treated cautiously if the host wall clock jumps.
The real controller's period comes from capability `sampling_rate` and is used
as milliseconds by the client; it is not derived from the ROS update-rate setting.

## Diagnostic changes

Inside `dual_crx_control`:

- `record_latency.py`: passive subscriptions only, captures both arms' command
  receipt, independently timed feedback receipt/header stamp, and scaling.
- `dual_test2_periodic.py --latency-csv ...`: optional application generation,
  pre-publication, post-publication and feedback callback events. The motion loop
  itself is unchanged. The preexisting ENTER prompt remains the motion start.
- `analyze_latency.py`: offline per-joint cross-correlation for old/new CSVs.
- `analyze_latency_trace.py`: correlates successive same-host pipeline stages,
  reports queue depth, represented age, send-call duration, interval statistics,
  and non-unit controller sequence steps.
- `tests/test_latency.py`: known positive/negative delays, gain/bias, sampling gaps,
  bounded recording, passive ROS capture on localhost domain 180, synthetic
  complete driver-pipeline report.
- `diagnostics/latency_trace.hpp`, `tests/latency_trace_smoke.cpp`: optional bounded
  C++ recorder and a standalone shutdown/overwrite test.

Outside the package, these driver source files were changed after notice:

1. `fanuc_libs/fanuc_client/include/fanuc_client/latency_trace.hpp` (new).
2. `fanuc_libs/fanuc_client/src/fanuc_client.cpp`.
3. `fanuc_hardware_interface/include/fanuc_robot_driver/hardware_interface.hpp`.
4. `fanuc_hardware_interface/src/hardware_interface.cpp`.

The complete reproducible change is `diagnostics/fanuc_latency.patch`, based on
FANUC driver commit `a5a88aee0a44689bbe6ed8ae2e44a4f3d60060a3`. It is **already
applied to this workspace source**; do not apply it again here. On a clean clone,
use `git apply --check` and `git apply` from the driver root.
No interpolation settings, controller safety settings or motion math changed.
Existing user launch-file renames were preserved.

The recorder is enabled only by `FANUC_LATENCY_DIR` before starting bringup.
Each single-producer buffer preallocates 180,000 records before motion; there are
hardware, client enqueue and stream buffers, approximately 60 MB per arm total.
At 500 Hz the stream buffer (status/send-begin/send-end) retains approximately
120 seconds; the other buffers retain longer. Oldest samples are overwritten,
with counts reported at shutdown. Stop bringup soon after a trial to retain it.
The hardware trace assumes the current synchronous read/write configuration.
There is no per-cycle disk or console output, locking, or buffer growth. Buffers
save on orderly object destruction during bringup shutdown, not when only the
motion script ends. SIGKILL or a crash may lose driver traces. The directory must
already exist and be writable; save errors are reported during shutdown.

`send_begin`/`send_end` bracket the client `sendCommand` call. They do **not**
prove packet arrival or execution; the underlying driver does not propagate UDP
send success through this method. `status_return` timestamps the status API
return, not NIC receive. Its controller timestamp remains raw, with no assumed
unit/epoch. Sequence attached to sends identifies the preceding status, not an
execution acknowledgement. Exact T1 controller update and T4 controller execution
are not instrumented; T0→hardware_write bounds the combined host handoff.

## Build and repeat analysis without motion

The custom package was rebuilt successfully. Both modified C++ translation units
passed compile checks against the installed dependencies, and the standalone
logger test passed. At that pre-trial stage, the driver was not rebuilt/installed into the active
launch, to avoid replacing mapped libraries. The operator subsequently restarted
with instrumentation for trial 01, whose results are linked above.

All eight latency tests passed. The installed diagnostic sine, passive recorder,
and analyzer also completed an eight-second motion test on the software mock,
isolated on localhost domain 181. It recovered 2 ms apparent lag at the
application callback and 1 ms at the observer, with correlation above 0.99999.
These values validate the measurement path against the ideal mock; they do not
predict the real controller's delay. To repeat that acceptance:

```bash
source ~/ws_fanuc/install/setup.bash
/usr/bin/python3 ~/ws_fanuc/src/dual_crx_control/tests/verify_latency_mock.py
```

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select dual_crx_control
source install/setup.bash
colcon test --packages-select dual_crx_control \
  --ctest-args -R '^latency$' --output-on-failure
```

Analyze the existing comparisons:

```bash
cd ~/ws_fanuc/src/dual_crx_control
source ~/ws_fanuc/install/setup.bash
ros2 run dual_crx_control analyze_latency.py \
  ~/ws_fanuc/cartesian_motion_results/*latency_equal*.csv \
  ~/ws_fanuc/cartesian_motion_results/*period_*s.csv \
  --output test_results/latency/existing_recordings.json
```

For a passive capture with the existing bringup, no restart is needed:

```bash
ros2 run dual_crx_control record_latency.py --duration 60 \
  --output "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/observer.csv"
```

All terminals observing real robots must use the same ROS domain and middleware
settings as their bringup (currently domain 87). This command cannot cause motion.

## Manual instrumented real test

First finish any motion, stop its script, and **shut down physical bringup before
rebuilding the driver**. Then in the workspace root:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select fanuc_libs fanuc_hardware_interface dual_crx_control
source install/setup.bash
export FANUC_LATENCY_DIR="$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01"
mkdir -p "$FANUC_LATENCY_DIR"
ros2 launch dual_crx_control dual_arm.launch.py mock:=false
```

Retain the startup PID/namespace mapping for the two `ros2_control_node` processes.
Trace filenames contain PID; enqueue/stream filenames also contain robot IP.
Default IPs are left 192.168.2.100 and right 192.168.1.100.

In a second terminal, start the passive recorder (same ROS environment):

```bash
source ~/ws_fanuc/install/setup.bash
ros2 run dual_crx_control record_latency.py --duration 60 \
  --output "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/observer.csv"
```

In a third terminal, **operator-reviewed physical motion**, one arm at a time:

```bash
source ~/ws_fanuc/install/setup.bash
ros2 run dual_crx_control dual_test2_periodic.py \
  --robot-namespace left --joint 1 --amplitude 1.0 --period 4.0 --time 32.0 \
  --initial-hold 1.0 --ramp-time 1.0 --rate 500 \
  --latency-csv "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/application.csv" \
  --plot-file "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/response.png"
```

The script prints its targets and waits for ENTER. The example amplitude is
1 degree; check the starting pose and motion clearance before starting. Use the
existing collaborative safety features. After motion/recording finish, shut down
bringup normally to write driver traces. Do not leave it running minutes afterward.

Identify that arm's PID from `192.168.2.100_enqueue_<PID>_*.csv`, then run
(replace `12345` with that PID):

```bash
ros2 run dual_crx_control analyze_latency_trace.py \
  --application "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/application.csv" \
  --observer "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/observer.csv" \
  --driver-dir "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01" \
  --driver-pid 12345 --arm left --joint 1 \
  --output "$HOME/ws_fanuc/src/dual_crx_control/test_results/latency/trial_01/report.json"
```

Use a fresh directory for each trial. Repeat for right by changing namespace,
arm and PID. Start with this baseline. If it leaves a controller-side residual,
compare periods 8/4/2 s at the same 1 degree amplitude, then amplitudes 0.5/1 degree
at the same 4 s period. Keep other settings fixed and repeat to assess variability.
32 seconds gives integer cycles in these examples. Do not start another buffer
sweep unless the internal measurements justify it.

## Reading the stage report

- Large `app_publish→hardware_write`: host DDS/executor/control-loop handoff.
- Large `client_enqueue→send_begin` with matching represented age: driver queue
  and virtual-time interpolation; inspect queue/alpha and actual interval statistics.
- Small host delay but large `send_begin→status_return`: residual includes network,
  controller buffering/filtering, servo response and feedback generation. Host
  traces alone cannot separate these; do not label all of it servo delay.
- Large `status_return→hardware_read`: client feedback queue/control-loop scheduling.
- Small observer delay but large `hardware_read→app_feedback`: application callback
  or cached-state lag, especially in the old spin-once/sleep sine loop.
- Changing scaling or fitted gain with aggressiveness: investigate controller-side
  clamping/dynamics; correlation alone cannot distinguish a filter from a pure delay.
- Irregular intervals or non-unit sequences: investigate scheduling/socket backlog
  and packet loss. Non-unit sequences include repeats and wraparound as well as gaps.
- Empty/missing channels, weak correlation, boundary peaks, overwritten data or
  large-gap errors mean the requested comparison is unavailable or uncertain.

All latency traces must come from the same host boot for monotonic timestamps to
be comparable. The passive command timestamp is callback receipt, not publication.
Do not combine old relative-time Cartesian CSVs with absolute driver timestamps.
Source-only observations cannot establish FANUC's internal execution timestamp.

No dedicated FANUC Stream Motion manual was found under the searched local paths.
Further documentation should cover J519/Stream Motion, R912/Remote Motion,
communication interval, controller input buffering, filtering/servo group delay,
and collaborative speed clamp. The alarm list B-83284EN-1/10 cannot establish
these settings. Do not change undocumented controller parameters based on guesses.
