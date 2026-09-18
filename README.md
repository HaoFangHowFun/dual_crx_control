# Dual CRX Control

ROS 2 control and motion tools for the dual FANUC CRX arms.

## Build

Source the ROS distribution, then build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_crx_control
source install/setup.bash
```

If an earlier build used `--symlink-install` or the package layout changed, remove
only this package's generated build and install directories before rebuilding:

```bash
rm -rf build/dual_crx_control install/dual_crx_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_crx_control
source install/setup.bash
```

Do not remove the whole workspace `build/` or `install/` directory unless all
workspace packages are intended to be rebuilt.

## Launch

Normal dual-arm mock bringup:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=true
```

Enable FANUC force/torque broadcasters when needed:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=true wrench:=true
```

The wrench recorder is started separately:

```bash
ros2 run dual_crx_control record_wrench.py
```
