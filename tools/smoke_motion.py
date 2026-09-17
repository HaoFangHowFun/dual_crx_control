"""Run installed motion executables against the ideal ROS software mock.

Source ROS and a freshly built workspace, then run:
    ROS_DOMAIN_ID=177 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tools/smoke_motion.py
Outputs and subprocess logs are kept in a temporary directory. Never uses a hardware driver.
"""

import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import numpy as np
import xacro
import yaml


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    root = Path(tempfile.mkdtemp(prefix='dual-crx-motion-smoke-'))
    print(f'Artifacts: {root}', flush=True)
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '177'),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
               ROS_LOG_DIR=str(root / 'ros_log'), MPLCONFIGDIR=str(root / 'matplotlib'),
               OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
    share = Path(get_package_share_directory('dual_crx_control'))
    executables = Path(get_package_prefix('dual_crx_control')) / 'lib/dual_crx_control'
    xml = xacro.process_file(str(share / 'urdf/dual_crx.urdf.xacro')).toxml()
    params = root / 'mock.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': xml}}}))
    joint = ['--yes', '--amplitude-deg', '.5', '--period', '1', '--initial-hold', '.1', '--ramp-time', '.2']
    cart = ['--cycles', '1', '--initial-move-time', '.5', '--initial-settle-time', '.1',
            '--initial-max-velocity', '.8', '--initial-max-acceleration', '.8']
    cases = [
        ('single', 'joint_sine.py', [*joint, '--arms', 'left', '--duration', '1.2', '--final-hold', '.1',
                                   '--latency-csv', str(root / 'events.csv')], False),
        ('dual', 'joint_sine.py', [*joint, '--arms', 'left', 'right', '--duration', '1.2'], False),
        ('continuous', 'joint_sine.py', [*joint, '--continuous'], True),
        ('sine', 'cartesian_sine.py', [*cart, '--amplitude-m', '.002', '--period', '2', '--ramp-time', '.3'], False),
        ('circle', 'cartesian_circle.py', [*cart, '--radius-m', '.002', '--period', '2'], False),
        ('facing', 'facing_circle.py', [*cart, '--period', '3'], False),
    ]
    reports = []
    for label, executable, args, interrupt in cases:
        folder = root / label
        folder.mkdir()
        processes, streams = [], []

        def launch(name, extra, *, source_tool=False):
            stream = (folder / f'{name}.log').open('w')
            streams.append(stream)
            command = ([sys.executable, str(Path(__file__).resolve().parent / name)]
                       if source_tool else [str(executables / name)])
            process = subprocess.Popen([*command, *extra], env=env,
                                       stdout=stream, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        try:
            launch('dual_mock_robot.py', ['--ros-args', '--params-file', str(params)], source_tool=True)
            launch('interpolation_node', ['--ros-args', '-p', 'input_rate_hz:=50.0'])
            motion = launch(executable, [*args, '--output-dir', str(folder), '--ros-args', '--params-file', str(params)])
            if interrupt:
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline and motion.poll() is None:
                    csvs = list(folder.glob('*/joints.csv'))
                    if csvs:
                        with csvs[0].open() as stream:
                            targets = [row for row in csv.DictReader(stream) if row['source'] == 'target']
                        if len(targets) >= 150:
                            break
                    time.sleep(.1)
                else:
                    raise AssertionError('Continuous motion never produced enough targets')
                stop(motion)
            else:
                motion.wait(timeout=90)
            assert motion.returncode == 0, f'{label} exited {motion.returncode}; see {folder}'
            assert all(p.poll() is None for p in processes[:2]), f'Mock/interpolation exited: {folder}'
            csvs = list(folder.glob('*/joints.csv'))
            assert len(csvs) == 1
            with csvs[0].open() as stream:
                rows = list(csv.DictReader(stream))
            arms = {'left'} if label == 'single' else {'left', 'right'}
            for arm in arms:
                selected = [row for row in rows if row['arm'] == arm]
                assert {row['source'] for row in selected} == {'target', 'interpolated', 'feedback'}
                assert np.isfinite([[float(row[f'J{i}_rad']) for i in range(1, 7)] for row in selected]).all()
                assert (csvs[0].parent / f'{arm}_joints.png').stat().st_size > 0
            if len(arms) == 1:
                assert not (csvs[0].parent / 'right_joints.png').exists()
            if label in ('single', 'dual'):
                for arm in arms:
                    q = [[float(row[f'J{i}_rad']) for i in range(1, 7)] for row in rows
                         if row['arm'] == arm and row['source'] == 'target']
                    assert np.allclose(q[0], q[-1], atol=1e-12), 'Finite motion did not return to start'
            if label in ('sine', 'circle', 'facing'):
                tcp = list(csvs[0].parent.glob('*.json'))
                assert len(tcp) == 1, f'Missing TCP metadata: {folder}'
                data = json.loads(tcp[0].read_text())
                assert data['command_samples'] > 10 and data['feedback_samples']['left'] > 10
                assert data['settings']['cycles'] == 1
                assert list(csvs[0].parent.glob('axis_*.png')) or list(csvs[0].parent.glob('circle_*.png'))
            report = dict(case=label, samples=len(rows), output=str(csvs[0].parent))
            reports.append(report)
            print(json.dumps(report), flush=True)
        finally:
            for process in reversed(processes):
                stop(process)
            for stream in streams:
                stream.close()
    (root / 'summary.json').write_text(json.dumps(reports, indent=2) + '\n')
    print('All six ROS mock scenarios passed.', flush=True)


if __name__ == '__main__':
    main()
