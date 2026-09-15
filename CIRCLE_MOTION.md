# Simultaneous dual-TCP circles

`dual_test_6_cartesian_circle_motion.py` is a separate executable. The existing
`dual_test_5_cartesion_sychro_motion.py` remains the sinusoidal experiment.
Both use the calibrated world-frame FK/IK, shared 50 Hz target updates, and
500 Hz linear interpolation/paired command publication.

## Build and mock run

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dual_crx_control
source install/setup.bash
ros2 launch dual_crx_control dual_cartesian_mock.launch.py
```

In a second terminal with the same ROS domain:

```bash
cd ~/ws_fanuc
source install/setup.bash
ros2 run dual_crx_control dual_test_6_cartesian_circle_motion.py
```

Run only one motion script at a time. The mock launch inherits your terminal's
ROS settings, just as the script does. No special domain export is needed for
normal use when both terminals have matching environments.

Defaults:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `radius` | 0.02 m | Circle radius, 20 mm |
| `plane` | `xy` | Common world plane: `xy`, `xz`, or `yz` |
| `direction` | `ccw` | `ccw` or `cw` in the ordered plane axes |
| `period` | 8.0 s | Average lap time for finite runs; cruise lap time when repeating indefinitely |
| `cycles` | 1 | Number of laps; 0 repeats until stopped |
| `rate` | 50.0 Hz | Cartesian trajectory + IK updates |
| `command_rate` | 500.0 Hz | Joint command publication |
| `move_to_initial` | true | Move both arms to the configured initial joints first |
| `save_plot` | true | Save circle plot, CSV, and JSON at completion/Ctrl+C |
| `output_dir` | `cartesian_motion_results` | Relative to the working directory |

Example: two clockwise 10 mm circles in world XZ:

```bash
ros2 run dual_crx_control dual_test_6_cartesian_circle_motion.py \
  --ros-args -p radius:=0.01 -p plane:=xz -p direction:=cw \
  -p period:=8.0 -p cycles:=2
```

`radius`, `plane`, and `direction` define this script's geometry. The inherited
sinusoidal `amplitude`, `axis`, and `ramp_time` settings do not shape the circle.
Other inherited startup, IK, interpolation, and command-limit settings keep
their existing meanings (see `CARTESIAN_MOTION.md`).

## Path and timing

Both arms receive exactly the same world displacement and share one phase;
each keeps its own initial TCP orientation. Their centers differ because their
starting TCP positions differ. They draw equal circles simultaneously.

The starting TCP lies on the perimeter, so there is no jump to a circle edge.
For ordered plane axes `(u, v)` and radius `r`:

```text
center = start - r * world_u
u(t) = start_u + r * (cos(theta) - 1)
v(t) = start_v + r * sin(theta)
```

For default XY, X displacement ranges from **−40 to 0 mm** and Y displacement
from **−20 to +20 mm**. This is a 20 mm radius circle, not a 20 mm bound on total
distance from the initial TCP. Counterclockwise starts toward positive Y; for
XZ/YZ, interpret direction in the same ordered-axis convention.

Angular speed ramps up once at the beginning, remains constant through all
intermediate lap boundaries, and ramps down only near the end of the final lap.
The radius stays constant. Each speed ramp lasts `period / 4` seconds and has
smooth endpoints. For finite runs, total circle time remains `cycles * period`;
cruise speed is slightly higher to compensate for the two endpoint ramps, so
`period` is the average lap time rather than the exact time of every lap.

With `cycles:=0`, angular speed ramps up once and then stays constant at one
lap per `period` seconds. No automatic slowdown is scheduled for an indefinite
run; Ctrl+C retains the existing command-stream-stop behavior. The commanded
path remains an approximation between the 50 Hz IK targets because
interpolation is linear in joint space.

Before the circle, the existing initial-joint move brings the left arm to
`[0, 0, 0, 0, -90, 0]` degrees and the right arm to
`[-90, 0, 180, 0, 90, 0]` degrees. The script waits for both to settle before
capturing their Cartesian starting poses. Use `move_to_initial:=false` to draw
circles through the current measured TCP poses instead.

For a finite run, each TCP returns approximately to its Cartesian start, holds
for 0.5 seconds, and stops the command stream. Ctrl+C can stop a run partway
through; the mock holds its last command. Existing target rejection keeps both
arms on their last accepted segment and then holds its endpoint.

## Facing TCPs and adjusted circle centers

The dedicated executable `dual_test_7_cartesian_facing_circle_motion.py` uses
the tested facing-circle settings as its defaults; no parameter file is needed.
After starting mock bringup in a terminal with matching ROS settings, run:

```bash
source ~/ws_fanuc/install/setup.bash
ros2 run dual_crx_control dual_test_7_cartesian_facing_circle_motion.py
```

Override settings as usual, for example `--ros-args -p period:=6.0 -p cycles:=3`
or `-p center_midpoint:="[0.55, -0.38, 0.30]" -p tcp_gap:=0.2`.
The motion script uses the same topics with mock and hardware bringup.
The original test 6 executable retains its previous defaults and profile support.
Both scripts use the shared `src/dual_crx_control/circle_controller.py`.

The installed `config/facing_circle_mock.yaml` profile enables two inward-facing
TCPs with a **200 mm gap**, 100 mm radius, world XZ clockwise circles, a 4 second
average lap time, 10 laps, and a 2 rad/s joint velocity limit. The provisional
tool-forward convention is **local TCP +X**; left +X points along world −Y,
right +X along world +Y, and both local +Z axes stay world-up. Both TCPs have
the same displacement throughout the circle, so their separation stays fixed.

| Parameter | Profile value | Meaning |
| --- | --- | --- |
| `face_each_other` | `true` | Enable the facing-pose approach (default is `false`) |
| `center_midpoint` | `[0.55, -0.38, 0.30]` | Midpoint of the two circle centers, world metres |
| `tcp_gap` | `0.2` | Center/TCP separation along world Y, metres |
| `minimum_scaled_sigma` | `0.1` | Lower bound for the sampled Jacobian conditioning metric |

The left circle center is `[0.55, -0.28, 0.30]` m and the right center is
`[0.55, -0.48, 0.30]` m. Circle starts are at X = 0.65 m. Startup first returns
both arms to the existing initial joints, then executes a slow synchronized
joint move to the facing circle starts, waits for settling, and begins the
40 second circle sequence. The additional approach uses the existing
`initial_move_time`, `initial_max_velocity`, and `initial_max_acceleration`
parameters; with defaults it takes about 46 seconds. `move_to_initial:=false`
skips the initial-joint return but still performs the facing approach.

For a mock test, launch bringup in one terminal:

```bash
cd ~/ws_fanuc
source install/setup.bash
export ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=''
ros2 launch dual_crx_control dual_cartesian_mock.launch.py
```

In another terminal, use the same ROS settings and run the profile:

```bash
cd ~/ws_fanuc
source install/setup.bash
export ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=''
ros2 run dual_crx_control dual_test_6_cartesian_circle_motion.py --ros-args \
  --params-file "$(ros2 pkg prefix --share dual_crx_control)/config/facing_circle_mock.yaml"
```

The explicit domain/local discovery above isolates this experiment. The launch
file itself continues to inherit terminal ROS settings. Parameters can be
overridden after `--params-file`, for example
`-p center_midpoint:="[0.55, -0.38, 0.30]" -p tcp_gap:=0.2`.
Different centers, gaps, radii, or tool-axis conventions need fresh verification.

The offline search sampled 48 candidate midpoints, then checked the selected
placement over all 10 laps at 50 Hz. Its minimum scaled singular value improved
from **0.1765 to 0.3801**, compared with a facing circle at the original center
placement with the same 200 mm gap. Minimum joint-limit margin improved from
0.0288 rad to 0.3928 rad. Predicted peak joint speeds were 0.491 rad/s (left) and
0.481 rad/s (right), below your 2 rad/s limit. The joint-interpolated approach
also passed its sampled conditioning check (minimum 0.3818).

The metric is the smallest singular value after dividing Jacobian linear rows
by a 0.5 m characteristic length. Higher values mean better conditioning under
this scaling. The controller checks the approach before execution and rejects
circle target pairs below the configured threshold. These are sampled model
checks, not a guarantee for arbitrary poses or physical robots. Collision
clearance, including physical tools, has **not** been checked.

To repeat the offline search or complete installed-executable mock acceptance:

```bash
cd ~/ws_fanuc/src/dual_crx_control
source ~/ws_fanuc/install/setup.bash
ros2 run dual_crx_control search_facing_circle.py --output-dir test_results/facing_circle
/usr/bin/python3 tests/verify_facing_circle_mock.py
```

The acceptance runs test 7 with its built-in defaults on localhost domain 178
and saves logs, CSV, a command/feedback
circle plot, RViz screenshot when a display is available, and measured metrics
under `test_results/facing_circle/live/`. It checks all 10 laps, radius, closure,
inward orientation, gap, conditioning, command velocity, and achieved rates.
It shuts down the mock processes it starts. No physical robot test was run.

The complete 10-lap acceptance measured **499.82 Hz commands and 50.00 Hz IK**,
with no aborts or rejected target pairs. Across the recorded command stream,
minimum scaled singular values were 0.38524 (left) and 0.38007 (right); peak
joint speeds were 0.5112 and 0.5049 rad/s. Maximum radial error was 0.0208 mm,
closure error below 0.0049 mm, and commanded TCP-gap error below 0.0081 mm.
These figures describe the ideal mock and interpolation, not hardware accuracy.
All four focused test suites passed (59 pytest cases).

## Saved circles

When run from `~/ws_fanuc`, the script saves:

```text
~/ws_fanuc/cartesian_motion_results/circle_xy_<timestamp>.png
~/ws_fanuc/cartesian_motion_results/circle_xy_<timestamp>.csv
~/ws_fanuc/cartesian_motion_results/circle_xy_<timestamp>.json
```

The PNG shows separate equal-scale circle plots for both TCPs, comparing FK of
published joint commands with FK of received joint feedback. Axes show world
plane displacement in millimetres from each starting TCP. CSV includes both
plane coordinates, command/feedback timestamps, and all six joint angles.
It is a joint-feedback-derived TCP estimate, not an external TCP measurement.

## Verification

```bash
cd ~/ws_fanuc
source install/setup.bash
colcon test --packages-select dual_crx_control \
  --ctest-args -R '^(cartesian|interpolation|circle|facing_circle)$' --output-on-failure
colcon test-result --test-result-base build/dual_crx_control --verbose
/usr/bin/python3 src/dual_crx_control/tests/verify_circle_mock.py
```

The last command uses localhost domain 177, runs the installed circle script
with the mock and RViz, checks command/feedback radius and closure, checks both
rates, saves artifacts under `src/dual_crx_control/test_results/circle_mock/`,
and shuts down the processes it started. Unit ROS tests use a separate domain
176. Geometry/IK tests cover all three planes in both directions. Timing tests check
nonzero cruise speed at intermediate lap crossings and zero speed only at run
endpoints. A two-lap ROS mock test checks published TCP speed across the first
lap boundary and confirms closure after the final lap.

Only mock motion was executed for this change. The first mock run measured
50.00 Hz IK and about 500.01 Hz commands, with radial errors below 0.003 mm.
These are ideal-model measurements, not physical robot accuracy claims. The
circle and initial-joint return do not include collision avoidance.

## Files changed

- `scripts/dual_test_6_cartesian_circle_motion.py`: new circle executable and parameters.
- `src/dual_crx_control/circular_trajectory.py`: circle geometry and smooth lap phase.
- `src/dual_crx_control/cartesian_controller.py`: shared controller extracted from
  the existing sinusoidal script, with a trajectory-offset override.
- `scripts/dual_test_5_cartesion_sychro_motion.py`: thin entry point retaining the
  existing sinusoidal controller behavior and executable name.
- `src/dual_crx_control/motion_recording.py`: optional two-axis circle plots and CSV.
- `tests/test_circle.py`: geometry, IK, finite streaming, and plot tests.
- `tests/verify_circle_mock.py`: installed-executable and RViz acceptance check.
- `CMakeLists.txt`: installs the new executable and registers circle tests.
- `CARTESIAN_MOTION.md` and `CIRCLE_MOTION.md`: updated documentation.

Existing launch files, calibrated URDF, kinematics, IK solver, and interpolation
math are unchanged.
