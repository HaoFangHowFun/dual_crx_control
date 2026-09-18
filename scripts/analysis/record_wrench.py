#!/usr/bin/env python3
"""Record measured-feedback TCP poses and flange wrench; show 12 live channels.

Usage (source ROS and workspace in each terminal first):
    # 1. Start dual_arm.launch.py or dual_arm_teleop.launch.py as usual.
    # 2. Enable wrench feedback only when needed:
    ros2 launch dual_crx_control dual_arm.launch.py mock:=false wrench:=true
    # 3. Record and display:
    ros2 run dual_crx_control record_wrench.py
    ros2 run dual_crx_control record_wrench.py --no-plot --output-dir wrench_recordings
    ros2 run dual_crx_control record_wrench.py --help

Defaults: /left/force_torque_sensor_broadcaster/wrench and the corresponding
right topic, WrenchStamped. Override with --left-wrench-topic/--right-wrench-topic.
The 6x2 figure shows Fx/Fy/Fz [N], Tx/Ty/Tz [N*m], left/right columns; last 10 s
at 10 display updates/s. Requires a GUI backend (e.g. WSLg); use --no-plot remotely.
CSV records at 50 Hz per arm by default (`--record-rate-hz`), independently of plot rate;
ROS callbacks still receive the full source rate. Pose is world->left_tcp/right_tcp FK from
measured joints; wrench remains in its original flange frame. No TCP wrench
transform, zeroing, filtering, gravity compensation or motion commands are applied.
Pose pairing uses latest received feedback, NOT exact measurement synchronization.
Stale/missing poses are blank; WAITING/STALE wrench is never replaced by fake zeros.

Ctrl+C or closing the figure drains the CSV writer and saves metadata and the last
window as wrench.png. This stops recording ONLY, not the robot or broadcaster.
Each run creates a unique output subdirectory. Queue overflow is warned/counted;
disk errors stop recording. DDS loss and forced termination cannot be ruled out.
Mock force values are not physical contact simulation.
"""
from dual_crx_control.analysis.wrench_recording import main

if __name__ == '__main__':
    main()
