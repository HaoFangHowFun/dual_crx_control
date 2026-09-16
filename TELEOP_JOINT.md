# Python joint teleoperation bridge

Build and launch the joint controllers with mock hardware plus the `rclpy` bridge:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_crx_control
source install/setup.bash
ros2 launch dual_crx_control teleop_joint.launch.py mock:=true
```

For physical bringup use `mock:=false`. Optional launch arguments are
`rviz:=true` / `rviz:=false`,
`left_robot_ip:=192.168.2.100`, `right_robot_ip:=192.168.1.100`,
`state_publish_rate:=100.0`, and `method:=linear` / `method:=cubic`.
For an already running robot stack with interpolation configured at 20 Hz, start only the bridge with
`ros2 run dual_crx_control teleop_bridge`.

| Topic | Type | Contents |
| --- | --- | --- |
| `/teleop/joint_command` | `std_msgs/msg/Float64MultiArray` | Exactly 12 finite positions in radians, left J1–J6 then right J1–J6 |
| `/teleop/joint_states` | `sensor_msgs/msg/JointState` | `left_J1`–`left_J6`, then `right_J1`–`right_J6` |

The launch starts the shared joint-only interpolation node with **20 Hz input**
and **500 Hz output**. The bridge sends at most one fresh target every 50 ms to
`/interpolation/joint_targets`; only interpolation publishes the two controller
command topics. It rejects invalid lengths and nonfinite values, and the node
checks joint names, lengths and finite values only. There are no startup commands. Send external
targets at 20 Hz, starting from the measured pose.

When input pauses, interpolation completes its last segment and continuously
publishes the last joint positions at 500 Hz. New targets resume from that held
command. Feedback only initializes interpolation's first segment; there is no
input timeout, feedback watchdog, or session ownership. See
[INTERPOLATION.md](INTERPOLATION.md) for the joint-only interface and timing contract.

Feedback comes from `/left/joint_states` and `/right/joint_states`, reordered by
joint name. Publication begins only after both arms supply valid positions.
The latest valid samples are published at 100 Hz, without time synchronization;
the output timestamp is the older of the two source timestamps, including when
one source stops updating. Velocity is included only when both arms supply six
finite values. Effort is omitted because its reliability is not established.

The bridge feedback topics remain startup parameters `left_joint_state_topic` and
`right_joint_state_topic`. Interpolation publishes the fixed per-arm controller topics.
For example, the standalone executable accepts `--ros-args -p left_joint_state_topic:=/custom/state`.

`teleop_joint.launch.py` defines its own nodes and includes no other launch file.
Each arm has a `robot_state_publisher` providing the driver's CRX-5iA description,
a `ros2_control_node`, and a spawner for exactly two controllers:
`joint_state_broadcaster` and `forward_position_controller`. Their configuration
is in `config/dual_arm_controllers.yaml`. TF topics stay private to each arm.

Mock mode opens RViz by default. Use `mock:=true rviz:=false` for headless tests;
real mode defaults to no RViz and supports `mock:=false rviz:=true` to enable it.
The dedicated `rviz/teleop_joint.rviz` configuration displays the calibrated
dual-arm URDF. Its state publisher reads `/teleop/joint_states`, publishes the
combined global TF, and supplies `/teleop/robot_description` to RViz.

`mock:=true` selects `mock_components/GenericSystem`; `mock:=false` selects the
FANUC hardware plugin through the driver's Xacro `use_mock` argument. Both modes
use the same controller types, joint order, and ROS interfaces. This launch is
joint-only and starts no Cartesian controller, IK, MoveIt, trajectory controller,
or motion script. Mock hardware starts at the same poses as `dual_arm.launch.py mock:=true`:

| Arm | Initial J1–J6 (degrees) |
| --- | --- |
| Left | `[0, 0, 0, 0, -90, 0]` |
| Right | `[-90, 0, 180, 0, 90, 0]` |

Edit `MOCK_INITIAL_POSITIONS` in `src/dual_crx_control/robot_description.py` to change these
defaults (values there are radians). They initialize the mock hardware's position
state through `initial_value`, so feedback and RViz start at the same pose without
publishing a motion command. Real hardware receives no initial-position override.

Tests use isolated localhost ROS domains 183/184 and exercise command splitting,
invalid inputs, feedback ordering, optional velocity, continuous hold/resumption, and
an installed mock launch with small joint steps from its measured initial pose:

```bash
colcon test --packages-select dual_crx_control --ctest-args -R '^teleop_bridge$'
colcon test-result --verbose
```

For physical testing, first verify both measured joint states against the robots.
Start commands from those measured positions and change only one joint by a
small amount. An all-zero command is a mock test, not a physical starting pose.
