#!/usr/bin/env python3
"""Thirty-second measurements against installed launch, on isolated localhost DDS."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from dual_crx_control.interpolation_client import JointTargetClient
from dual_crx_control.joint_config import SIDES, JOINT_NAMES


def measure(rate, method, seconds, output, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_STATIC_PEERS='', ROS_LOG_DIR=str(output/'ros_logs'))
    rclpy.init(domain_id=domain)
    node = Node('interpolation_measurement')
    client = JointTargetClient(node, rate)
    states, reports = {}, []
    arrivals = {s: [] for s in SIDES}
    for side in SIDES:
        node.create_subscription(JointState, f'/{side}/joint_states',
                                 lambda msg,s=side: states.update({s: dict(zip(msg.name,msg.position))}), 10)
        node.create_subscription(Float64MultiArray, f'/{side}/forward_position_controller/commands',
                                 lambda msg,s=side: arrivals[s].append(time.monotonic()), 100)
    node.create_subscription(JointState, '/interpolation/joint_commands', reports.append, 100)
    process = None
    try:
        with (output/f'{method}_{rate:g}.log').open('w') as log:
            process = subprocess.Popen(['ros2','launch','dual_crx_control','dual_arm.launch.py', 'mock:=true',
                                        'rviz:=false',f'input_rate_hz:={rate}',f'method:={method}'],
                                       env=env, stdout=log, stderr=subprocess.STDOUT,start_new_session=True)
            deadline = time.monotonic()+15
            while len(states)<2 or not client.available():
                rclpy.spin_once(node,timeout_sec=.01)
                assert time.monotonic()<deadline and process.poll() is None
            starts = {s: np.array([states[s][n] for n in JOINT_NAMES[s]]) for s in SIDES}
            t0 = next_tick = time.monotonic()
            while time.monotonic()-t0 < seconds+1:
                now = time.monotonic()
                if now >= next_tick:
                    targets = {s: q+.002*np.sin(2*np.pi*(now-t0)/4) for s,q in starts.items()}
                    client.publish(targets)
                    next_tick += (int((now-next_tick)*rate)+1)/rate
                rclpy.spin_once(node,timeout_sec=.0005)
            client.publish(targets)
            deadline = time.monotonic() + .2
            while time.monotonic() < deadline:
                rclpy.spin_once(node,timeout_sec=.001)
            stamps = np.array([m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in reports])
            steady = stamps[stamps >= stamps[0]+1]
            intervals = np.diff(steady)
            rate_out = (len(steady)-1)/(steady[-1]-steady[0])
            endpoint = float(np.max(np.abs(np.array(reports[-1].position)-np.concatenate(list(targets.values())))))
            paired_count = min(len(v) for v in arrivals.values())
            skew = np.abs(np.array(arrivals['left'][:paired_count])-np.array(arrivals['right'][:paired_count]))
            assert 475 <= rate_out <= 525, rate_out
            assert endpoint < 1e-10
            for side in SIDES:
                assert [i.node_name for i in node.get_publishers_info_by_topic(
                    f'/{side}/forward_position_controller/commands')] == ['joint_interpolation']
            return dict(method=method,input_rate_hz=rate,output_rate_hz=rate_out,
                        measurement_seconds=float(steady[-1]-steady[0]),samples=len(steady),
                        interval_p95_ms=float(np.percentile(intervals,95)*1000),
                        interval_p99_ms=float(np.percentile(intervals,99)*1000),
                        interval_max_ms=float(intervals.max()*1000),endpoint_error_rad=endpoint,
                        paired_receipt_skew_p99_ms=float(np.percentile(skew,99)*1000),
                        paired_receipt_skew_max_ms=float(skew.max()*1000),
                        timing_basis='CLOCK_MONOTONIC output sample stamps; skew is observer callback receipt difference')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=30.)
    parser.add_argument('--output',type=Path,default=Path('test_results/interpolation'))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for i,(rate,method) in enumerate((r,m) for r in (20.,50.,100.,500.) for m in ('linear','cubic')):
        result=measure(rate,method,args.seconds,args.output,190+i)
        results.append(result)
        (args.output/'metrics.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
