#!/usr/bin/env python3
"""Full facing-circle mock acceptance, isolated on localhost domain 178."""

import csv
import json
import os
from pathlib import Path
import re
import signal
import subprocess

import numpy as np
import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from dual_crx_control.facing_circle import scaled_min_singular_value, facing_start_poses
from dual_crx_control.kinematics import CRXKinematics

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'test_results/facing_circle/live'


def analyze(log):
    assert 'ABORT' not in log and 'Target pair rejected' not in log, log
    assert 'Finite motion finished' in log and 'Both TCPs face each other' in log
    rates = re.search(r'Achieved command rate: ([\d.]+) Hz; IK rate: ([\d.]+) Hz', log)
    assert rates and 450 < float(rates[1]) < 550 and 45 < float(rates[2]) < 55
    path = Path(re.search(r'Saved motion data: (.+)', log)[1])
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    share = Path(get_package_share_directory('dual_crx_control'))
    xml = xacro.process_file(str(share / 'urdf/dual_crx.urdf.xacro')).toxml()
    models = {s: CRXKinematics(xml, s + '_tcp') for s in ('left', 'right')}
    expected = facing_start_poses([.55, -.38, .30], .2, .1, 'xz')
    metrics = {'command_rate_hz': float(rates[1]), 'ik_rate_hz': float(rates[2]),
               'collision_checked': False}
    command_points = {}
    for side, model in models.items():
        center = expected[side][:3, 3] - [.1, 0., 0.]
        for source in ('command', 'feedback'):
            samples = [r for r in rows if r['arm'] == side and r['source'] == source]
            t = np.array([float(r['time_s']) for r in samples])
            q = np.array([[float(r[f'J{i}_rad']) for i in range(1, 7)] for r in samples])
            poses = np.array([model.fk(joints) for joints in q])
            points = poses[:, :3, 3]
            radial = points[:, [0, 2]] - center[[0, 2]]
            radius_error = float(np.max(np.abs(np.linalg.norm(radial, axis=1) - .1)))
            orientation_error = float(np.max(np.linalg.norm(
                poses[:, :3, :3] - expected[side][:3, :3], axis=(1, 2))))
            sigma = min(scaled_min_singular_value(model, joints) for joints in q)
            laps = float(-np.diff(np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))).sum()
                         / (2. * np.pi))
            closure = float(np.linalg.norm(points[-1] - expected[side][:3, 3]))
            assert radius_error < .0001 and closure < .0001
            assert orientation_error < .001 and sigma > .35
            assert np.max(np.abs(points[:, 1] - center[1])) < .0001
            assert abs(laps - 10.) < .001 and t[-1] >= 40.
            measurements = {'max_radius_error_mm': radius_error * 1000.,
                            'max_orientation_matrix_error': orientation_error,
                            'minimum_scaled_sigma': sigma, 'laps': laps,
                            'closure_error_mm': closure * 1000.}
            if source == 'command':
                speed = float(np.max(np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]))
                assert speed < 2.
                measurements['max_joint_velocity_rad_s'] = speed
                command_points[side] = points
            metrics[f'{side}_{source}'] = measurements
    gap_errors = np.abs(np.linalg.norm(command_points['left'] - command_points['right'], axis=1) - .2)
    assert gap_errors.max() < .0001
    metrics['max_tcp_gap_error_mm'] = float(gap_errors.max() * 1000.)
    (OUTPUT / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps(metrics, indent=2))
    print('Circle plot: ' + str(path.with_suffix('.png')))


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for key, name in [('ROS_LOG_DIR', 'logs'), ('TMPDIR', 'tmp'),
                      ('XDG_CONFIG_HOME', 'config'), ('XDG_CACHE_HOME', 'cache'),
                      ('MPLCONFIGDIR', 'matplotlib')]:
        folder = OUTPUT / name
        folder.mkdir(exist_ok=True)
        os.environ[key] = str(folder)
    os.environ.update(ROS_DOMAIN_ID='178', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', PYTHONDONTWRITEBYTECODE='1')
    launch = motion = None
    try:
        with (OUTPUT / 'launch.log').open('w') as launch_log, (OUTPUT / 'motion.log').open('w') as motion_log:
            launch = subprocess.Popen(['ros2', 'launch', 'dual_crx_control',
                                       'dual_cartesian_mock.launch.py'], stdout=launch_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            executable = Path(get_package_prefix('dual_crx_control')) / 'lib/dual_crx_control/dual_test_7_cartesian_facing_circle_motion.py'
            motion = subprocess.Popen([str(executable), '--ros-args',
                                       '-p', f'output_dir:={OUTPUT}'], stdout=motion_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            assert motion.wait(timeout=150) == 0, (OUTPUT / 'motion.log').read_text()
            assert 'OpenGl version' in (OUTPUT / 'launch.log').read_text()
            if os.environ.get('DISPLAY'):
                from PyQt5.QtWidgets import QApplication
                app = QApplication([])
                windows = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
                for line in windows.splitlines():
                    if 'RViz' in line and '0x' in line:
                        app.primaryScreen().grabWindow(int(line.strip().split()[0], 16)).save(str(OUTPUT / 'rviz.png'))
                        break
            analyze((OUTPUT / 'motion.log').read_text())
    finally:
        for process in (motion, launch):
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    main()
