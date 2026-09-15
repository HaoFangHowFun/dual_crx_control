#!/usr/bin/env python3
"""Verify bringup alone is stationary, then run and stop the standalone motion.

Artifacts are written beneath the package's test_results/cartesian_mock directory.
This standalone acceptance check uses only localhost ROS domain 174.
"""

import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import xacro

from dual_crx_control.kinematics import CRXKinematics


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'test_results' / 'cartesian_mock'
INITIAL = {'left': np.radians([0, 0, 0, 0, -90, 0]),
           'right': np.radians([-90, 0, 180, 0, 90, 0])}


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for key, folder in [('ROS_LOG_DIR', 'ros_logs'), ('TMPDIR', 'tmp'),
                        ('XDG_CONFIG_HOME', 'config'), ('XDG_CACHE_HOME', 'cache'),
                        ('MPLCONFIGDIR', 'matplotlib')]:
        path = OUTPUT / folder
        path.mkdir(exist_ok=True)
        os.environ[key] = str(path)
    os.environ.update(ROS_DOMAIN_ID='174', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', PYTHONDONTWRITEBYTECODE='1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from tf2_ros import Buffer, TransformListener
    from ament_index_python.packages import get_package_prefix

    xml = xacro.process_file(str(ROOT / 'urdf' / 'dual_crx.urdf.xacro')).toxml()
    models = {s: CRXKinematics(xml, f'{s}_tcp') for s in INITIAL}
    starts = {s: m.fk(INITIAL[s]) for s, m in models.items()}
    rclpy.init()
    node = Node('cartesian_acceptance_monitor')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    records = {s: [] for s in INITIAL}
    commands = {s: [] for s in INITIAL}
    command_times = {s: [] for s in INITIAL}
    combined = []

    def feedback(side, msg):
        values = dict(zip(msg.name, msg.position))
        q = np.array([values[name] for name in models[side].joint_names])
        pose = models[side].fk(q)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        records[side].append((stamp, q, pose))

    def command(side, msg):
        command_times[side].append(time.monotonic())
        commands[side].append(list(msg.data))

    for side in INITIAL:
        node.create_subscription(JointState, f'/{side}/joint_states',
                                 lambda msg, s=side: feedback(s, msg), qos_profile_sensor_data)
        node.create_subscription(Float64MultiArray,
                                 f'/{side}/forward_position_controller/commands',
                                 lambda msg, s=side: command(s, msg), 100)
    node.create_subscription(JointState, '/joint_states', combined.append, qos_profile_sensor_data)
    process = None
    motion = None
    try:
        with (OUTPUT / 'launch.log').open('w') as log, (OUTPUT / 'motion.log').open('w') as motion_log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'dual_crx_control', 'dual_cartesian_mock.launch.py'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                assert process.poll() is None, 'Launch exited unexpectedly'
            assert all(len(rows) > 50 for rows in records.values()), 'Missing mock feedback'
            assert all(not rows for rows in commands.values()), 'Bringup published motion commands'
            for side, rows in records.items():
                for _, q, _ in rows:
                    np.testing.assert_array_equal(q, INITIAL[side])
            executable = (Path(get_package_prefix('dual_crx_control')) / 'lib' /
                          'dual_crx_control' / 'dual_test_5_cartesion_sychro_motion.py')
            # Same installed executable resolved by ros2 run; no description parameter:
            # it must discover the already-running launch's latched robot description.
            motion = subprocess.Popen([str(executable), '--ros-args', '-p',
                                       f'output_dir:={OUTPUT / "motion_plots"}'], stdout=motion_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 13
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                assert process.poll() is None, 'Launch exited unexpectedly'
                assert motion.poll() is None, 'Motion script exited unexpectedly'
            text = (OUTPUT / 'launch.log').read_text() + (OUTPUT / 'motion.log').read_text()
            assert 'ABORT' not in text and 'process has died' not in text, text
            assert 'OpenGl version' in text, 'RViz did not initialize rendering'
            assert all(len(v) > 5000 for v in commands.values()), 'Insufficient command pairs'
            assert combined and len(combined[-1].name) == 12
            metrics = {'bringup_without_commands': 'passed'}
            for side in INITIAL:
                rows = records[side]
                np.testing.assert_allclose(rows[0][1], INITIAL[side], atol=1e-12)
                offsets = np.array([p[:3, 3] - starts[side][:3, 3] for _, _, p in rows])
                rotations = [np.linalg.norm(Rotation.from_matrix(
                    p[:3, :3] @ starts[side][:3, :3].T).as_rotvec()) for _, _, p in rows]
                assert offsets[:, 0].max() > .0199 and offsets[:, 0].min() < -.0199
                assert np.max(np.abs(offsets[:, 1:])) < 2e-5
                assert max(rotations) < 2e-5
                tf = buffer.lookup_transform('world', f'{side}_tcp', rclpy.time.Time())
                latest = np.array([tf.transform.translation.x, tf.transform.translation.y,
                                   tf.transform.translation.z])
                assert np.linalg.norm(latest - rows[-1][2][:3, 3]) < .003
                metrics[side] = dict(command_count=len(commands[side]),
                                     observed_command_rate_hz=(len(command_times[side]) - 1) /
                                     (command_times[side][-1] - command_times[side][0]),
                                     x_min_m=float(offsets[:, 0].min()),
                                     x_max_m=float(offsets[:, 0].max()),
                                     max_off_axis_error_m=float(np.abs(offsets[:, 1:]).max()),
                                     max_orientation_error_rad=float(max(rotations)))
                assert 450 < metrics[side]['observed_command_rate_hz'] < 550
            # Compare targets recovered from corresponding command indices.
            count = min(len(v) for v in commands.values())
            displacements = {s: np.array([models[s].fk(q)[:3, 3] - starts[s][:3, 3]
                                          for q in commands[s][:count]]) for s in INITIAL}
            mismatch = np.max(np.linalg.norm(displacements['left'] - displacements['right'], axis=1))
            assert mismatch < 2e-5
            metrics['max_pair_cartesian_mismatch_m'] = float(mismatch)
            motion.send_signal(signal.SIGINT)
            assert motion.wait(timeout=15) == 0
            plot_log = (OUTPUT / 'motion.log').read_text()
            assert 'Saved command-versus-feedback plot:' in plot_log, plot_log
            assert list((OUTPUT / 'motion_plots').glob('*.csv'))
            summary = re.search(r'Achieved command rate: ([\d.]+) Hz; IK rate: ([\d.]+) Hz',
                                (OUTPUT / 'motion.log').read_text())
            assert summary, 'Missing two-rate diagnostics'
            metrics['controller_command_rate_hz'] = float(summary[1])
            metrics['controller_ik_rate_hz'] = float(summary[2])
            assert 450 < float(summary[1]) < 550
            assert 45 < float(summary[2]) < 55
            until = time.monotonic() + .3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            held = {s: records[s][-1][1].copy() for s in INITIAL}
            counts = {s: len(commands[s]) for s in INITIAL}
            start_indices = {s: len(records[s]) for s in INITIAL}
            until = time.monotonic() + .6
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            for side in INITIAL:
                assert len(commands[side]) == counts[side]
                assert len(records[side]) - start_indices[side] > 20
                for _, q, _ in records[side][start_indices[side]:]:
                    np.testing.assert_array_equal(q, held[side])
            metrics['hold_after_controller_stop'] = 'passed'
            (OUTPUT / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            for side in INITIAL:
                rows = records[side]
                t = np.array([row[0] for row in rows]) - records['left'][0][0]
                poses = [row[2] for row in rows]
                axes[0].plot(t, [(p[0, 3] - starts[side][0, 3]) * 1000 for p in poses], label=side)
                axes[1].plot(t, [np.linalg.norm(Rotation.from_matrix(
                    p[:3, :3] @ starts[side][:3, :3].T).as_rotvec()) for p in poses], label=side)
            axes[0].set_ylabel('World X displacement [mm]')
            axes[1].set_ylabel('Orientation error [rad]')
            axes[1].set_xlabel('Time [s]')
            for ax in axes:
                ax.grid(True)
                ax.legend()
            fig.tight_layout()
            fig.savefig(OUTPUT / 'tcp_tracking.png', dpi=150)
            plt.close(fig)
            # Capture the configured RViz window when an X11 display is available.
            if os.environ.get('DISPLAY'):
                from PyQt5.QtWidgets import QApplication
                app = QApplication([])
                windows = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
                for line in windows.splitlines():
                    if 'RViz' in line and '0x' in line:
                        window_id = int(line.strip().split()[0], 16)
                        app.primaryScreen().grabWindow(window_id).save(str(OUTPUT / 'rviz.png'))
                        break
            print(json.dumps(metrics, indent=2))
    finally:
        if motion is not None and motion.poll() is None:
            motion.send_signal(signal.SIGINT)
            try:
                motion.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(motion.pid, signal.SIGKILL)
                motion.wait()
        if process is not None and process.poll() is None:
            # Let launch signal its children once; a group signal duplicates it.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
