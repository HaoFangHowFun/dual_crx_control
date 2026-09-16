"""Exercise the external ROS API on isolated localhost domains; no hardware."""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from dual_crx_control.teleop_bridge import JOINT_NAMES, SIDES, TeleopBridge


def test_initial_pose_only_modifies_mock_hardware_description():
    from ament_index_python.packages import get_package_share_directory

    launch_path = Path(__file__).resolve().parents[1] / 'launch/teleop_joint.launch.py'
    spec = importlib.util.spec_from_file_location('teleop_joint_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    xacro_path = str(Path(get_package_share_directory('fanuc_hardware_interface'))
                     / 'robot/crx5ia.urdf.xacro')
    for side in SIDES:
        for mock in (True, False):
            root = ET.fromstring(module.arm_description(xacro_path, side, '127.0.0.1', mock))
            control = root.find('ros2_control')
            initials = control.findall("joint/state_interface[@name='position']/param[@name='initial_value']")
            if mock:
                assert control.findtext('hardware/plugin') == 'mock_components/GenericSystem'
                assert [float(p.text) for p in initials] == module.MOCK_INITIAL_POSITIONS[side]
            else:
                assert control.findtext('hardware/plugin') == 'fanuc_robot_driver/FanucHardwareInterface'
                assert not initials


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate(), 'Timed out waiting for ROS messages/discovery'


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


@pytest.fixture
def ros_pair():
    context = Context()
    rclpy.init(context=context, domain_id=183)
    bridge = TeleopBridge(context=context)
    peer = Node('teleop_test_device', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    executor.add_node(peer)
    try:
        yield bridge, peer, executor
    finally:
        executor.shutdown()
        peer.destroy_node()
        bridge.destroy_node()
        context.shutdown()


def test_command_target_validation_and_twenty_hz_timer(ros_pair):
    from unittest.mock import patch
    bridge, peer, executor = ros_pair
    assert bridge.input_rate == 20.
    assert any(timer.timer_period_ns == 50_000_000 for timer in bridge.timers)
    with patch.object(bridge.target_client, 'available', return_value=True), \
            patch.object(bridge.target_client, 'publish') as publish:
        bridge.receive_command(Float64MultiArray(data=[.001]*12))
        bridge.send_target()
        publish.assert_called_once_with({'left': [.001]*6, 'right': [.001]*6})
        accepted = bridge.pending_command
        for data in ([], [0.]*11, [0.]*13, [float('nan')]*12, [float('inf')]*12):
            bridge.receive_command(Float64MultiArray(data=data))
        assert bridge.pending_command == accepted
        bridge.send_target()
        assert publish.call_count == 1
    assert not any('forward_position_controller/commands' in pub.topic_name for pub in bridge.publishers)


def test_feedback_requires_both_arms_and_orders_by_name(ros_pair):
    bridge, peer, executor = ros_pair
    received = []
    peer.create_subscription(JointState, '/teleop/joint_states', received.append, 10)
    publishers = {side: peer.create_publisher(
        JointState, f'/{side}/joint_states', qos_profile_sensor_data) for side in SIDES}
    spin_until(executor, lambda: bridge.state_publisher.get_subscription_count() == 1
               and all(pub.get_subscription_count() == 1 for pub in publishers.values()))
    left = JointState(name=list(reversed(JOINT_NAMES['left'])),
                      position=[6., 5., 4., 3., 2., 1.], velocity=[.6, .5, .4, .3, .2, .1])
    left.header.stamp.sec = 10
    publishers['left'].publish(left)
    spin_for(executor, 0.1)
    assert not received
    right = JointState(name=JOINT_NAMES['right'], position=[7., 8., 9., 10., 11., 12.])
    right.header.stamp.sec = 11
    publishers['right'].publish(right)
    spin_until(executor, lambda: bool(received))
    assert received[-1].name == JOINT_NAMES['left'] + JOINT_NAMES['right']
    assert list(received[-1].position) == [float(i) for i in range(1, 13)]
    assert not received[-1].velocity and not received[-1].effort
    assert received[-1].header.stamp.sec == 10
    right.velocity = [.7, .8, .9, 1., 1.1, 1.2]
    publishers['right'].publish(right)
    spin_until(executor, lambda: len(received[-1].velocity) == 12)
    assert list(received[-1].velocity) == pytest.approx([i / 10. for i in range(1, 13)])
    # Malformed or nonfinite position feedback cannot overwrite the last valid sample.
    for bad in (JointState(name=JOINT_NAMES['right'][:-1], position=[0.] * 5),
                JointState(name=JOINT_NAMES['right'], position=[0.] * 5),
                JointState(name=['right_J1'] * 6, position=[0.] * 6),
                JointState(name=JOINT_NAMES['right'], position=[float('nan')] * 6)):
        publishers['right'].publish(bad)
        spin_for(executor, 0.03)
        assert list(received[-1].position) == [float(i) for i in range(1, 13)]
    right.velocity = [float('inf')] * 6
    publishers['right'].publish(right)
    spin_until(executor, lambda: not received[-1].velocity)


@pytest.mark.parametrize('launch_name', ['teleop_joint.launch.py', 'dual_arm.launch.py'])
def test_installed_mock_launch_round_trip(tmp_path, launch_name):
    context = Context()
    rclpy.init(context=context, domain_id=184)
    peer = Node('teleop_mock_acceptance', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    feedback = []
    command_counts = {side: 0 for side in SIDES}
    arm_feedback = {side: [] for side in SIDES}

    def count(side):
        command_counts[side] += 1

    for side in SIDES:
        peer.create_subscription(JointState, f'/{side}/joint_states',
                                 arm_feedback[side].append, qos_profile_sensor_data)
        peer.create_subscription(Float64MultiArray,
                                 f'/{side}/forward_position_controller/commands',
                                 lambda msg, s=side: count(s), 10)
    teleop = launch_name == 'teleop_joint.launch.py'
    peer.create_subscription(JointState, '/teleop/joint_states' if teleop else '/joint_states', feedback.append, 10)
    publisher = peer.create_publisher(
        Float64MultiArray if teleop else JointState,
        '/teleop/joint_command' if teleop else '/interpolation/joint_targets', 1)

    def publish_target(target):
        message = Float64MultiArray(data=target) if teleop else JointState(
            name=JOINT_NAMES['left'] + JOINT_NAMES['right'], position=target)
        publisher.publish(message)
    env = dict(os.environ, ROS_DOMAIN_ID='184', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_STATIC_PEERS='', ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
    with_rviz = os.environ.get('TELEOP_TEST_RVIZ') == '1'
    if with_rviz:
        from tf2_ros import Buffer, TransformListener
        buffer = Buffer()
        listener = TransformListener(buffer, peer)
    process = None
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'dual_crx_control', launch_name, 'mock:=true',
                 f'rviz:={str(with_rviz).lower()}'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(executor, lambda: len(feedback) > 10 and len(feedback[-1].position) == 12, timeout=15.0)
            assert process.poll() is None
            assert command_counts == {'left': 0, 'right': 0}
            assert feedback[-1].name == JOINT_NAMES['left'] + JOINT_NAMES['right']
            initial = [0., 0., 0., 0., -math.pi / 2, 0.,
                       -math.pi / 2, 0., math.pi, 0., math.pi / 2, 0.]
            # Check actual hardware feedback. The standard joint_state_publisher
            # can expose partially populated merged positions during discovery.
            spin_until(executor, lambda: all(arm_feedback.values()))
            for index, side in enumerate(SIDES):
                for message in arm_feedback[side]:
                    by_name = dict(zip(message.name, message.position))
                    assert [by_name[n] for n in JOINT_NAMES[side]] == pytest.approx(
                        initial[index*6:(index+1)*6])
            if with_rviz:
                spin_until(executor, lambda: ('teleop_rviz' if teleop else 'rviz2') in peer.get_node_names())
                spin_until(executor, lambda: all(buffer.can_transform(
                    'world', f'{side}_tcp', rclpy.time.Time()) for side in SIDES))
            # Verify the actual controller stack, and wait for command readiness.
            for side in SIDES:
                client = peer.create_client(ListControllers, f'/{side}/controller_manager/list_controllers')
                spin_until(executor, client.service_is_ready)
                deadline = time.monotonic() + 10.0
                while True:
                    future = client.call_async(ListControllers.Request())
                    spin_until(executor, future.done)
                    controllers = {c.name: c.state for c in future.result().controller}
                    if controllers == {'joint_state_broadcaster': 'active',
                                       'forward_position_controller': 'active'}:
                        break
                    assert time.monotonic() < deadline, controllers
                    spin_for(executor, 0.05)
            spin_until(executor, lambda: publisher.get_subscription_count() == 1 and all(
                peer.count_subscribers(f'/{side}/forward_position_controller/commands') == 2
                for side in SIDES))
            assert list(feedback[-1].position) == pytest.approx(initial)
            assert command_counts == {'left': 0, 'right': 0}
            target = initial.copy()
            target[0] += .00872665
            publish_target(target)
            spin_until(executor, lambda: np.allclose(feedback[-1].position, target, atol=1e-7))
            assert all(count > 10 for count in command_counts.values())
            spin_for(executor, .4)
            held_counts = command_counts.copy()
            spin_for(executor, .1)
            assert all(command_counts[s] > held_counts[s]+20 for s in SIDES)
            assert list(feedback[-1].position) == pytest.approx(target)
            # No timeout: the node holds at 500 Hz, then accepts the next target.
            target[0] += .001
            publish_target(target)
            spin_until(executor, lambda: np.allclose(feedback[-1].position, target, atol=1e-7))
            spin_for(executor, .4)
            for side in SIDES:
                assert [i.node_name for i in peer.get_publishers_info_by_topic(
                    f'/{side}/forward_position_controller/commands')] == ['joint_interpolation']
            if with_rviz:
                spin_until(executor, lambda: 'OpenGl version' in (tmp_path / 'launch.log').read_text())
            assert list(feedback[-1].velocity) == pytest.approx([0.0] * 12)
            if teleop:
                assert not feedback[-1].effort
            before = len(feedback)
            started = time.monotonic()
            spin_for(executor, 0.5)
            rate = (len(feedback) - before) / (time.monotonic() - started)
            assert 70 < rate < 130, rate
            print(f'Mock feedback: {rate:.1f} Hz; small commands, continuous hold and resume passed')
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        if with_rviz:
            listener.unregister()
        peer.destroy_node()
        context.shutdown()
    assert process.returncode == 0, (tmp_path / 'launch.log').read_text()
