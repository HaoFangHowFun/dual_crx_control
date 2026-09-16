"""Live joint resampling: data checks, continuous hold, feedback initialization."""
import importlib.util
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
import xacro

from dual_crx_control.interpolation_node import InterpolationNode
from dual_crx_control.interpolation_client import JointTargetClient
from dual_crx_control.joint_config import JOINT_NAMES, SIDES

ROOT = Path(__file__).resolve().parents[1]


def spin(executor, seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.001)


def until(executor, predicate, timeout=3.):
    deadline = time.monotonic()+timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.005)
    assert predicate(), 'ROS condition timed out'


@pytest.fixture
def pipeline(request):
    rate, method = getattr(request, 'param', (50., 'linear'))
    context = Context()
    rclpy.init(context=context, domain_id=177)
    xml = xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()
    spec = importlib.util.spec_from_file_location('mock', ROOT / 'scripts/dual_mock_robot.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    mock = module.DualMockRobot(context=context, parameter_overrides=[Parameter('robot_description', value=xml)])
    node = InterpolationNode(context=context, parameter_overrides=[
        Parameter('input_rate_hz', value=rate), Parameter('method', value=method)])
    peer = Node('joint_target_test', context=context)
    client = JointTargetClient(peer, rate)
    executor = SingleThreadedExecutor(context=context)
    for n in (mock, node, peer):
        executor.add_node(n)
    commands = []
    peer.create_subscription(JointState, '/interpolation/joint_commands',
                             lambda msg: commands.append((time.monotonic(), msg)), 100)
    until(executor, lambda: client.available() and len(node.positions)==2)
    try:
        yield node, mock, peer, client, executor, commands
    finally:
        executor.shutdown()
        for n in (peer, node, mock):
            n.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('pipeline', [(r, m) for r in (20., 50., 100., 500.) for m in ('linear','cubic')], indirect=True)
def test_live_rate_hold_and_resume(pipeline):
    node, mock, peer, client, executor, commands = pipeline
    start = {s: q.copy() for s,q in mock.positions.items()}
    assert not commands
    client.publish(start)
    until(executor, lambda: len(commands)>2)
    t0 = next_target = time.monotonic()
    while time.monotonic()-t0 < .3:
        now = time.monotonic()
        if now >= next_target:
            target = {s: q + .003*np.sin(2*np.pi*(now-t0)) for s,q in start.items()}
            client.publish(target)
            next_target = now+1/node.input_rate
        executor.spin_once(timeout_sec=.0005)
    client.publish(target)
    spin(executor,.1)
    np.testing.assert_array_equal(commands[-1][1].position,np.concatenate(list(target.values())))
    before=len(commands)
    spin(executor,.4)
    assert len(commands)>before+140
    for _,msg in commands[before:]:
        np.testing.assert_array_equal(msg.position,np.concatenate(list(target.values())))
    client.publish(start)
    spin(executor,.1)
    np.testing.assert_array_equal(commands[-1][1].position,np.concatenate(list(start.values())))
    assert not any(name.endswith('/session') for name,_ in peer.get_service_names_and_types())


@pytest.mark.parametrize('bad', [JointState(),
    JointState(name=JOINT_NAMES['left'],position=[0.]*5),
    JointState(name=['left_J1']*6,position=[0.]*6),
    JointState(name=JOINT_NAMES['left'],position=[float('nan')]*6),
    JointState(name=JOINT_NAMES['left'],position=[float('inf')]*6),
    JointState(name=JOINT_NAMES['left'][:3]+JOINT_NAMES['right'][:3],position=[0.]*6)])
def test_invalid_data_retains_last_target(pipeline,bad):
    node,mock,peer,client,executor,commands=pipeline
    start={s:q.copy() for s,q in mock.positions.items()}
    client.publish(start)
    spin(executor,.05)
    client.publisher.publish(bad)
    spin(executor,.08)
    assert commands
    np.testing.assert_array_equal(commands[-1][1].position,np.concatenate(list(start.values())))


def test_feedback_is_only_used_to_initialize_and_sources_need_no_session(pipeline):
    node,mock,peer,client,executor,commands=pipeline
    start={s:q.copy() for s,q in mock.positions.items()}
    client.publish(start)
    spin(executor,.05)
    mock.destroy_timer(mock._timers[0])
    spin(executor,.4)
    second=JointTargetClient(peer,50.)
    until(executor,second.available)
    target={s:q+.1 for s,q in start.items()}  # Deliberately faster than the removed 0.5 rad/s limit.
    second.publish(target)
    spin(executor,.1)
    np.testing.assert_array_equal(commands[-1][1].position,np.concatenate(list(target.values())))
    count=len(commands)
    spin(executor,.3)
    assert len(commands)>count+100


def test_single_arm_and_name_order(pipeline):
    node,mock,peer,client,executor,commands=pipeline
    target=mock.positions['left']+.001
    client.publisher.publish(JointState(name=JOINT_NAMES['left'][::-1],position=target[::-1].tolist()))
    spin(executor,.1)
    assert commands[-1][1].name==JOINT_NAMES['left']
    np.testing.assert_array_equal(commands[-1][1].position,target)
    assert 'right' not in node.last_q


def test_first_target_waits_for_feedback_without_zero_start(pipeline):
    node,mock,peer,client,executor,commands=pipeline
    node.positions.clear()
    mock.destroy_timer(mock._timers[0])
    spin(executor,.03)
    node.positions.clear()
    target={s:q.copy()+.001 for s,q in mock.positions.items()}
    client.publish(target)
    spin(executor,.05)
    assert not commands
    for side in SIDES:
        node.feedback(side,JointState(name=JOINT_NAMES[side],position=mock.positions[side].tolist()))
    spin(executor,.1)
    np.testing.assert_array_equal(commands[-1][1].position,np.concatenate(list(target.values())))


def test_no_urdf_or_motion_limit_parameters(pipeline):
    node,*_=pipeline
    for name in ('robot_description','max_velocity','max_cycle_step','state_timeout','command_timeout'):
        assert not node.has_parameter(name)
