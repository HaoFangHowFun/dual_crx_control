# Dual CRX Control

ROS 2 control and motion tools for the dual FANUC CRX arms.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_crx_control
source install/setup.bash
```

If the package layout changed or an old symlink build is broken, remove only this
package's generated directories and rebuild:

```bash
rm -rf build/dual_crx_control install/dual_crx_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_crx_control
source install/setup.bash
```

Do not remove the whole workspace `build/` or `install/` unless all packages are
intended to be rebuilt.

## Launch

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=true
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=true wrench:=true
ros2 run dual_crx_control record_wrench.py
```
