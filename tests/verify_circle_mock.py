#!/usr/bin/env python3
"""Installed circle executable + RViz, isolated on localhost domain 177."""

import csv
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'test_results/circle_mock'


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for key, name in [('ROS_LOG_DIR', 'logs'), ('TMPDIR', 'tmp'),
                      ('XDG_CONFIG_HOME', 'config'), ('XDG_CACHE_HOME', 'cache'),
                      ('MPLCONFIGDIR', 'matplotlib')]:
        folder = OUTPUT / name
        folder.mkdir(exist_ok=True)
        os.environ[key] = str(folder)
    os.environ.update(ROS_DOMAIN_ID='177', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='', PYTHONDONTWRITEBYTECODE='1')
    launch = motion = None
    try:
        with (OUTPUT / 'launch.log').open('w') as log, (OUTPUT / 'motion.log').open('w') as motion_log:
            launch = subprocess.Popen(['ros2', 'launch', 'dual_crx_control',
                                       'dual_arm.launch.py', 'mock:=true', 'rviz:=false'],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            executable = (Path(get_package_prefix('dual_crx_control')) / 'lib/dual_crx_control' /
                          'dual_test_6_cartesian_circle_motion.py')
            motion = subprocess.Popen([str(executable), '--ros-args', '-p', f'output_dir:={OUTPUT}'],
                                      stdout=motion_log, stderr=subprocess.STDOUT, start_new_session=True)
            assert motion.wait(timeout=30) == 0
            text = (OUTPUT / 'motion.log').read_text()
            assert 'ABORT' not in text and 'Target pair rejected' not in text, text
            assert 'Finite motion finished' in text
            summary = re.search(r'Achieved command rate: ([\d.]+) Hz; IK rate: ([\d.]+) Hz', text)
            assert summary and 450 < float(summary[1]) < 550 and 45 < float(summary[2]) < 55
            data_path = Path(re.search(r'Saved motion data: (.+)', text)[1])
            with data_path.open() as stream:
                rows = list(csv.DictReader(stream))
            metrics = {'command_rate_hz': float(summary[1]), 'ik_rate_hz': float(summary[2])}
            commands = {}
            for side in ('left', 'right'):
                for source in ('command', 'feedback'):
                    samples = [row for row in rows if row['arm'] == side and row['source'] == source]
                    points = np.array([[float(row['displacement_mm']),
                                        float(row['displacement_y_mm'])] for row in samples])
                    radius_errors = np.abs(np.linalg.norm(points + [20., 0.], axis=1) - 20.)
                    assert radius_errors.max() < .03
                    assert points[:, 0].min() < -39.9
                    assert points[:, 1].min() < -19.9 and points[:, 1].max() > 19.9
                    assert np.linalg.norm(points[-1]) < .03
                    metrics[f'{side}_{source}_max_radius_error_mm'] = float(radius_errors.max())
                    if source == 'command':
                        commands[side] = points
            mismatch = np.linalg.norm(commands['left'] - commands['right'], axis=1).max()
            assert mismatch < .03
            metrics['max_command_pair_mismatch_mm'] = float(mismatch)
            (OUTPUT / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
            if os.environ.get('DISPLAY'):
                from PyQt5.QtWidgets import QApplication
                app = QApplication([])
                windows = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
                for line in windows.splitlines():
                    if 'RViz' in line and '0x' in line:
                        app.primaryScreen().grabWindow(int(line.strip().split()[0], 16)).save(str(OUTPUT / 'rviz.png'))
                        break
            print(json.dumps(metrics, indent=2))
            print('Circle plot: ' + str(data_path.with_suffix('.png')))
    finally:
        for process in (motion, launch):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    main()
