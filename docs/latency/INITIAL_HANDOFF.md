# Dual CRX Latency Investigation — Test Summary and Codex Handoff

## Goal

We are investigating an approximately **0.1 s delay** between commanded sinusoidal joint motion and the measured robot joint response on the real FANUC CRX setup.

This document summarizes what has already been checked so Codex can inspect the whole project and continue the investigation without repeating completed work unnecessarily.

---

## System

- ROS 2: **Jazzy**
- Robots: dual FANUC CRX-5iA
- Controller: **R-30iB Mini Plus**
- ROS workspace:

```text
~/ws_fanuc
```

- Main custom package:

```text
~/ws_fanuc/src/dual_crx_control
```

- FANUC driver:

```text
~/ws_fanuc/src/fanuc_driver
```

Real robot command topics:

```text
/left/forward_position_controller/commands
/right/forward_position_controller/commands
```

Joint feedback:

```text
/left/joint_states
/right/joint_states
```

---

# 1. Observed Problem

During sinusoidal joint-motion tests, the measured joint response appears to follow the commanded sinusoid with approximately:

```text
~0.1 s delay
```

The trajectory shape is generally similar, so the current goal is to locate where the latency is introduced.

Possible layers:

```text
application command
        ↓
forward_position_controller
        ↓
ros2_control / controller_manager
        ↓
FANUC hardware interface
        ↓
fanuc_client / Stream Motion
        ↓
FANUC controller internal processing
        ↓
servo response / joint feedback
```

---

# 2. ROS Controller Rate — VERIFIED

Commands used:

```bash
ros2 param get /left/controller_manager update_rate
ros2 param get /right/controller_manager update_rate
```

Observed:

```text
Integer value is: 500
Integer value is: 500
```

So both controller managers are configured at **500 Hz**.

---

# 3. Joint-State Feedback Rate — VERIFIED

Measured with:

```bash
ros2 topic hz /left/joint_states
ros2 topic hz /right/joint_states
```

Typical stable results:

```text
left  ≈ 500 Hz
right ≈ 500 Hz
period ≈ 0.002 s
```

The right arm was especially stable around 500 Hz.

One later left-arm measurement showed a one-time large gap around:

```text
0.332 s
```

followed by the measured average returning toward 500 Hz.

Treat that as a transient unless it is reproducible during motion.

### Current conclusion

The approximately 100 ms delay is **not explained by ROS joint-state feedback running at only 50 or 100 Hz**.

---

# 4. FANUC Driver Interpolation Buffer — IDENTIFIED

Search performed:

```bash
grep -R "out_cmd_interp_buff_target"   ~/ws_fanuc/src/fanuc_driver   -n
```

The CRX-5iA driver Xacro contains:

```xml
<xacro:arg name="out_cmd_interp_buff_target" default="8"/>
```

Relevant path:

```text
fanuc_driver/
└── fanuc_hardware_interface/
    └── robot/
        └── crx5ia.urdf.xacro
```

The parameter is passed into the hardware interface and FANUC client.

Relevant locations include:

```text
fanuc_hardware_interface/src/hardware_interface.cpp
fanuc_hardware_interface/config/crx_physical_ros2_control_macro.xacro
fanuc_libs/fanuc_client/src/fanuc_client.cpp
fanuc_libs/fanuc_client/include/fanuc_client/fanuc_client.hpp
```

Known value:

```text
out_cmd_interp_buff_target = 8
```

At 500 Hz:

```text
1 cycle = 2 ms
8 samples × 2 ms ≈ 16 ms
```

So this buffer can contribute latency, but it does not obviously explain the full ~100 ms by itself.

### Important

Buffer-size experiments were already performed previously.

Do **not** make repeating `8 -> 4 -> 2` tests the first task unless inspection of the repository/logs shows the previous comparison was incomplete or not recorded.

---

# 5. Collaborative Speed-Clamping Topics — FOUND

The following topics exist:

```text
/left/fanuc_gpio_controller/collaborative_speed_scaling
/left/fanuc_gpio_controller/robot_status
/left/fanuc_gpio_controller/robot_status_ext

/right/fanuc_gpio_controller/collaborative_speed_scaling
/right/fanuc_gpio_controller/robot_status
/right/fanuc_gpio_controller/robot_status_ext
```

Also present:

```text
/left/joint_trajectory_controller/speed_scaling_input
/right/joint_trajectory_controller/speed_scaling_input
```

These topics have been discovered, but their behavior during the sinusoidal test still needs to be examined carefully.

---

# 6. Controller / Manual Review

The supplied FANUC document:

```text
B-83284EN-1/10
```

is primarily an **Alarm Code List**.

It contains collaborative-robot-related alarm information, but it is not the dedicated Stream Motion communication/configuration manual.

Searching it did not reveal direct Stream Motion settings such as:

```text
Stream Motion
STMO
Communication Interval
```

If other FANUC manuals are available locally, search specifically for:

```text
Stream Motion
J519
Remote Motion
R912
communication interval
interpolation buffer
streaming command timing
collaborative speed clamp
```

---

# 7. What Is Already Low Priority

Based on measurements:

```text
controller_manager accidentally running at low rate
```

is low priority because it reports 500 Hz.

```text
joint_states normally running at low rate
```

is also low priority because measured feedback is normally around 500 Hz.

```text
out_cmd_interp_buff_target
```

has already been identified and tested previously.

Do not restart the investigation from these points without new evidence.

---

# 8. Main Open Question

We still need to determine where the remaining approximately 100 ms latency occurs.

Useful timing points:

```text
T0: application generates/publishes target
        ↓
T1: forward_position_controller updates command
        ↓
T2: ros2_control hardware write()
        ↓
T3: fanuc_client sends Stream Motion command
        ↓
T4: controller accepts/executes command
        ↓
T5: physical joint feedback is returned
```

We want to estimate where most of:

```text
~100 ms
```

is introduced.

---

# 9. Recommended Codex Investigation

Inspect the relevant project paths before modifying anything:

```text
~/ws_fanuc/src/dual_crx_control
~/ws_fanuc/src/fanuc_driver
```

Focus especially on:

```text
dual_crx_control/scripts/
fanuc_hardware_interface/src/hardware_interface.cpp
fanuc_libs/fanuc_client/src/fanuc_client.cpp
```

Trace the exact path from:

```text
Float64MultiArray command
```

through:

```text
forward_position_controller
ros2_control
hardware_interface write()
fanuc_client
Stream Motion
```

---

# 10. Add Minimal Timing Instrumentation

Prefer measurement over speculation.

Add temporary/debug timing points such as:

```text
A. application command generated
B. hardware interface write() called
C. FANUC client command sent
D. joint feedback received
```

Use a monotonic clock.

Do not print every 2 ms because console I/O can disturb timing.

Instead:

```text
store timestamps in memory
save/report after the test
```

Useful statistics:

```text
mean
median
min
max
standard deviation
```

If practical, save CSV data containing:

```text
time
commanded joint position
measured joint position
collaborative_speed_scaling
driver timing markers
```

---

# 11. Measure Command-to-State Delay Automatically

The sine trajectory is useful for latency estimation.

For each robot/joint, compare:

```text
command(t)
measured_joint_state(t)
```

Use cross-correlation to estimate the time shift.

Desired output:

```text
Left J1 estimated delay:  XX ms
Right J1 estimated delay: XX ms
```

Do not rely only on visual inspection.

---

# 12. Test Collaborative Speed Scaling

During the same sine test, record:

```text
/left/fanuc_gpio_controller/collaborative_speed_scaling
/right/fanuc_gpio_controller/collaborative_speed_scaling
```

and, if useful:

```text
/left/fanuc_gpio_controller/robot_status_ext
/right/fanuc_gpio_controller/robot_status_ext
```

Determine whether collaborative speed scaling is constant or changes with motion.

If scaling changes with trajectory aggressiveness, investigate controller-side speed clamping/filtering.

---

# 13. Controlled Test Matrix

Use the same single-joint sinusoidal test with conservative variations in amplitude and period.

Interpretation:

```text
delay remains almost constant
    → likely fixed buffering / processing latency

delay increases as velocity increases
    → likely speed clamp / dynamic limiting / filtering

delay is highly irregular
    → likely scheduling / communication jitter
```

Change only one variable at a time.

---

# 14. Do Not Redesign Yet

Until the latency source is identified, do not unnecessarily redesign:

```text
IK solver
Cartesian trajectory generator
50 Hz -> 500 Hz interpolation architecture
URDF calibration
```

Use the existing simple synchronized joint-space sine test as the diagnostic signal.

---

# 15. Real-Robot Safety

Do not automatically run physical motion.

Codex may:

```text
inspect code
modify diagnostic/logging code
build packages
run non-motion ROS inspection commands
analyze saved logs
```

Real motion should remain a manual operator action after reviewing changes.

Do not disable collaborative safety features as part of this investigation.

---

# 16. Desired Codex Output

After examining the repository, report:

1. The complete command path from the test script to the FANUC controller.
2. Every buffering/interpolation/filtering stage found in code.
3. Which stages can theoretically introduce fixed latency.
4. Which stages are already reduced in priority by current measurements.
5. Where to add timestamp instrumentation.
6. A minimal patch for latency logging.
7. Exact build and run commands.
8. A short diagnostic test procedure.
9. How to interpret the resulting data.
10. Any FANUC driver/controller settings that deserve further investigation.

Avoid unrelated refactoring.

The immediate goal is **localizing the ~100 ms delay**, not improving trajectory smoothness yet.
