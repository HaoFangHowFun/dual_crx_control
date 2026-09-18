"""Software-only integration: real ros2_control mock, optional broadcaster, synthetic recording.

Source the freshly built workspace, then run from the repository root:
    python3 tools/smoke_wrench.py
    python3 tools/smoke_wrench.py --gui  # also exercise the live window (requires GUI)
Uses isolated localhost ROS domain 179, NEVER real hardware. Artifacts go to /tmp.
Synthetic wrench topics are separate from the real broadcaster outputs.
"""
import csv
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ.update(ROS_DOMAIN_ID='179', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from controller_manager_msgs.srv import ListControllers
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    root = Path(tempfile.mkdtemp(prefix='dual-crx-wrench-smoke-'))
    os.environ.update(ROS_LOG_DIR=str(root / 'ros-log'), MPLBACKEND='TkAgg' if args.gui else 'Agg',
                      MPLCONFIGDIR=str(root / 'mpl'), OPENBLAS_NUM_THREADS='1')
    print(f'Artifacts: {root}', flush=True)
    processes, streams = [], []
    rclpy.init()
    node = Node('wrench_smoke_observer')
    def launch(label, cmd):
        stream = (root / f'{label}.log').open('w')
        streams.append(stream)
        process = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise AssertionError('Process did not shut down cleanly')
    def spin_until(predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError('Timed out; inspect logs')
    def controllers(side):
        client = node.create_client(ListControllers, f'/{side}/controller_manager/list_controllers')
        try:
            assert client.wait_for_service(timeout_sec=30)
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done()
            return {c.name: c.state for c in future.result().controller}
        finally:
            node.destroy_client(client)
    try:
        bringup = launch('bringup', ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
                                      'mock:=true', 'rviz:=false', 'method:=ruckig'])
        for side in ('left', 'right'):
            deadline = time.monotonic()+40
            while controllers(side).get('forward_position_controller') != 'active':
                assert time.monotonic() < deadline
                time.sleep(.2)
        addon = launch('wrench', ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
                                  'mock:=true', 'rviz:=false', 'wrench:=true'])
        samples = {}
        for side in ('left', 'right'):
            node.create_subscription(WrenchStamped, f'/{side}/force_torque_sensor_broadcaster/wrench',
                                     lambda msg, s=side: samples.__setitem__(s, msg), 10)
        spin_until(lambda: len(samples) == 2 and all(
            controllers(s).get('force_torque_sensor_broadcaster') == 'active' for s in ('left', 'right')),
                   timeout=30)
        for side in ('left', 'right'):
            assert samples[side].header.frame_id == f'{side}_fanuc_flange'
            assert controllers(side)['force_torque_sensor_broadcaster'] == 'active'
        exe = Path(get_package_prefix('dual_crx_control')) / 'lib/dual_crx_control'
        assert controllers('left')['force_torque_sensor_broadcaster'] == 'active'
        recorder = launch('recorder', [str(exe / 'record_wrench.py'), *([] if args.gui else ['--no-plot']), '--output-dir', str(root / 'recording'),
                                      '--left-wrench-topic', '/test/left_wrench', '--right-wrench-topic', '/test/right_wrench'])
        pubs = {s: node.create_publisher(WrenchStamped, f'/test/{s}_wrench', 10) for s in ('left', 'right')}
        spin_until(lambda: all(p.get_subscription_count() for p in pubs.values()))
        for i in range(150):
            for side, publisher in pubs.items():
                msg = WrenchStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = f'{side}_fanuc_flange'
                msg.wrench.force.x = math.sin(i*.05)
                msg.wrench.force.y, msg.wrench.force.z = 2., 3.
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = .1, .2, .3
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)
        stop(recorder)
        assert recorder.returncode == 0
        csv_path, = (root / 'recording').glob('*/wrench.csv')
        with csv_path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert {r['arm'] for r in rows} == {'left', 'right'}
        assert sum(r['pose_valid'] == 'True' for r in rows) > 100
        assert (csv_path.parent / 'wrench.png').stat().st_size > 1000
        summary = {'csv': str(csv_path), 'rows': len(rows), 'valid_poses': sum(r['pose_valid'] == 'True' for r in rows),
                   'broadcasters': 'passed', 'full_bringup': 'passed'}
        (root / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
    finally:
        for process in reversed(processes):
            stop(process)
        node.destroy_node()
        rclpy.shutdown()
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
