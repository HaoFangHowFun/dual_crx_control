"""Known-delay recovery, timestamp hygiene, and isolated passive ROS capture."""
import csv
import importlib.util
import time
from pathlib import Path

import numpy as np
import pytest

from dual_crx_control.latency_analysis import estimate_delay
from dual_crx_control.latency_recording import LatencyRecording


@pytest.mark.parametrize('delay', [-.04, 0., .1, .117])
def test_known_delay_with_gain_bias_and_independent_sampling(delay):
    rng = np.random.default_rng(73)
    ct = np.arange(0., 14., .002)
    ft = np.arange(0., 14., .0021) + rng.uniform(-.0001, .0001, len(np.arange(0., 14., .0021)))
    def wave(t):
        return .02*np.sin(2*np.pi*t/3.7) + .003*np.sin(2*np.pi*t/1.3)
    result = estimate_delay(ct, wave(ct), ft, .9*wave(ft-delay)+.4)
    assert abs(result['delay_ms'] - delay*1000) <= 1.
    assert result['correlation'] > .9999
    assert abs(result['gain'] - .9) < .001


def test_large_gap_and_stationary_rejected():
    t = np.arange(0., 10., .002)
    with pytest.raises(ValueError, match='stationary'):
        estimate_delay(t, np.ones(len(t)), t, np.ones(len(t)))
    valid = (t < 4.) | (t > 4.3)
    with pytest.raises(ValueError, match='gap'):
        estimate_delay(t, np.sin(t), t[valid], np.sin(t[valid]-.1))


def test_bounded_capture_reports_overwrite(tmp_path):
    recording = LatencyRecording(capacity=2)
    for t in range(3):
        recording.add('left', 'command', [t]*6, received_at=t)
    path = recording.save(tmp_path/'events.csv')
    rows = list(csv.DictReader(path.open()))
    assert [float(r['time_s']) for r in rows] == [1., 2.]
    import json
    assert json.loads(path.with_suffix('.json').read_text())['overwritten'] == 1


def test_passive_capture_on_isolated_ros_domain(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from fanuc_msgs.msg import CollaborativeSpeedScaling
    path = Path(__file__).resolve().parents[1]/'scripts/record_latency.py'
    spec = importlib.util.spec_from_file_location('observer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = Context()
    rclpy.init(context=context, domain_id=180)
    observer = module.LatencyObserver(context=context)
    publisher = Node('synthetic_latency_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    executor.add_node(publisher)
    command = publisher.create_publisher(Float64MultiArray, '/left/forward_position_controller/commands', 10)
    state = publisher.create_publisher(JointState, '/left/joint_states', qos_profile_sensor_data)
    scaling = publisher.create_publisher(CollaborativeSpeedScaling, '/left/fanuc_gpio_controller/collaborative_speed_scaling', 10)
    try:
        deadline = time.monotonic()+3.
        while command.get_subscription_count() == 0 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.01)
        # Observer must have no publisher on either robot's command topic.
        for arm in ('left', 'right'):
            infos = observer.get_publishers_info_by_topic(f'/{arm}/forward_position_controller/commands')
            assert all(i.node_name != observer.get_name() for i in infos)
        for _ in range(20):
            command.publish(Float64MultiArray(data=[.1]*6))
            state.publish(JointState(name=[f'left_J{i}' for i in range(6,0,-1)], position=[float(i) for i in range(6,0,-1)]))
            scaling.publish(CollaborativeSpeedScaling(collaborative_speed_scaling=.8))
            for _ in range(3):
                executor.spin_once(timeout_sec=.01)
        assert {'command','feedback','scaling'} <= {r[2] for r in observer.recording.rows}
        q = next(r[3] for r in observer.recording.rows if r[2]=='feedback')
        assert q == (1.,2.,3.,4.,5.,6.)
        observer.recording.save(tmp_path/'passive.csv')
    finally:
        executor.shutdown()
        observer.destroy_node()
        publisher.destroy_node()
        context.shutdown()


def test_driver_stage_report_recovers_known_pipeline(tmp_path):
    import json
    import subprocess
    import sys
    times = np.arange(0., 12., .005) + 100000.
    app = tmp_path/'application.csv'
    recording = LatencyRecording()
    for t in times:
        for source, delay in [('generated', 0.), ('command', .001), ('feedback', .100)]:
            recording.add('left', source, [np.sin((t-delay)*2.)]*6, received_at=t)
    recording.save(app)
    stages = [('hardware_write', .003), ('client_enqueue', .003),
              ('send_begin', .019), ('send_end', .019),
              ('status_return', .099), ('hardware_read', .100)]
    fields = ['time_s','stage',*[f'J{i}_rad' for i in range(1,7)], 'queue_depth',
              'represented_age_s','alpha','controller_sequence','scaling','call_duration_s']
    with (tmp_path/'synthetic_123_0.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(fields)
        for i,t in enumerate(times):
            for stage,delay in stages:
                writer.writerow([t,stage,*([np.sin((t-delay)*2.)]*6),8,.016,.5,i,1.,.00001])
    output=tmp_path/'report.json'
    script=Path(__file__).resolve().parents[1]/'scripts/analyze_latency_trace.py'
    subprocess.run([sys.executable,str(script),'--application',str(app),'--driver-dir',str(tmp_path),
                    '--driver-pid','123','--arm','left','--output',str(output)],
                   check=True,capture_output=True,text=True)
    report=json.loads(output.read_text())
    assert abs(report['delays']['client_enqueue->send_begin']['delay_ms']-16.) < 1.1
    assert abs(report['delays']['send_begin->status_return']['delay_ms']-80.) < 1.1
    assert report['driver_statistics']['represented_age_ms']['mean'] == 16.
