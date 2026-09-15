"""Independent geometry checks and paired-command failure regression tests."""

import importlib.util
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xacro

from dual_crx_control.kinematics import CRXKinematics
from dual_crx_control.ik_solver import DampedLeastSquaresIK, IKResult, pose_error


ROOT = Path(__file__).resolve().parents[1]
INITIAL = {'left': np.radians([0, 0, 0, 0, -90, 0]),
           'right': np.radians([-90, 0, 180, 0, 90, 0])}


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def description():
    return xacro.process_file(str(ROOT / 'urdf' / 'dual_crx.urdf.xacro')).toxml()


@pytest.mark.parametrize('side', INITIAL)
def test_fk_and_world_jacobian_independently(description, side):
    # Independent homogeneous-transform URDF traversal, without KDL.
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(description)
    model = CRXKinematics(description, f'{side}_tcp')
    q = INITIAL[side] + np.array([.1, -.2, .15, -.1, .2, .05])
    by_name = dict(zip(model.joint_names, q))
    expected = np.eye(4)
    for name in robot.get_chain('world', f'{side}_tcp', joints=True, links=False):
        joint = robot.joint_map[name]
        origin = np.eye(4)
        if joint.origin is not None:
            origin[:3, :3] = Rotation.from_euler('xyz', joint.origin.rpy or [0, 0, 0]).as_matrix()
            origin[:3, 3] = joint.origin.xyz or [0, 0, 0]
        expected = expected @ origin
        if joint.type == 'revolute':
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(np.array(joint.axis) * by_name[name]).as_matrix()
            expected = expected @ motion
    np.testing.assert_allclose(model.fk(q), expected, atol=1e-12)
    jacobian = model.jacobian(q)
    for j in range(6):
        epsilon = np.eye(6)[j] * 1e-7
        difference = pose_error(model.fk(q + epsilon), model.fk(q - epsilon)) / 2e-7
        np.testing.assert_allclose(jacobian[:, j], difference, atol=2e-8)


@pytest.mark.parametrize('side', INITIAL)
@pytest.mark.parametrize('axis', range(3))
def test_full_sine_fixed_orientation(description, side, axis):
    model = CRXKinematics(description, f'{side}_tcp')
    solver = DampedLeastSquaresIK(model)
    seed = INITIAL[side].copy()
    start = model.fk(seed)
    for elapsed in np.linspace(0, 8, 401):
        target = start.copy()
        target[axis, 3] += .02 * np.sin(2 * np.pi * elapsed / 4)
        result = solver.solve(target, seed)
        assert result.success, result
        error = pose_error(target, model.fk(result.q))
        assert np.linalg.norm(error[:3]) <= solver.position_tolerance
        assert np.linalg.norm(error[3:]) <= solver.orientation_tolerance
        assert model.valid_joints(result.q)
        assert np.max(np.abs(result.q - seed)) / .02 < .5
        seed = result.q


def test_invalid_unreachable_and_iteration_bound(description):
    model = CRXKinematics(description, 'left_tcp')
    solver = DampedLeastSquaresIK(model, max_iterations=3, max_joint_step=.01)
    q = INITIAL['left']
    pose = model.fk(q)
    assert not solver.solve(pose, np.full(6, np.nan)).success
    assert not solver.solve(pose, model.upper + 1).success
    assert not solver.solve(np.full((4, 4), np.inf), q).success
    invalid = pose.copy()
    invalid[:3, :3] *= 2
    assert not solver.solve(invalid, q).success
    pose[:3, 3] += 10
    result = solver.solve(pose, q)
    assert not result.success
    assert result.iterations <= 3
    assert model.valid_joints(result.q)
    assert np.max(np.abs(result.q - q)) <= .03 + 1e-12


def test_rotation_error_at_pi():
    current, target = np.eye(4), np.eye(4)
    target[:3, :3] = Rotation.from_rotvec([np.pi, 0, 0]).as_matrix()
    assert np.isclose(np.linalg.norm(pose_error(target, current)[3:]), np.pi)


@pytest.fixture
def controller(description):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    context = Context()
    rclpy.init(context=context, domain_id=175)
    module = load_script('dual_test_5_cartesion_sychro_motion')
    node = module.DualCartesianController(
        context=context, parameter_overrides=[Parameter('robot_description', value=description),
                                             Parameter('move_to_initial', value=False)])
    node.timer.cancel()
    for side in INITIAL:
        node.positions[side] = INITIAL[side].copy()
        node.received[side] = time.monotonic()
    with patch.object(node.arm_publishers['left'], 'get_subscription_count', return_value=1), \
            patch.object(node.arm_publishers['right'], 'get_subscription_count', return_value=1), \
            patch.object(node.arm_publishers['left'], 'publish') as left, \
            patch.object(node.arm_publishers['right'], 'publish') as right:
        yield node, left, right
    node.destroy_node()
    context.shutdown()


@pytest.mark.parametrize('failure', ['ik', 'stale', 'subscriber', 'step', 'velocity', 'limit', 'nan'])
def test_failure_retains_previous_target_for_both_arms(controller, failure):
    node, left, right = controller
    node.cycle()
    assert left.call_count == right.call_count == 1
    previous_target = node.segment_target.copy()
    left.reset_mock()
    right.reset_mock()
    node.next_target_time = time.monotonic()
    if failure == 'stale':
        node.received['right'] -= 1
        node.cycle()
    elif failure == 'subscriber':
        with patch.object(node.arm_publishers['right'], 'get_subscription_count', return_value=0):
            node.cycle()
    else:
        q = node.segment_target[1].copy()
        if failure in ('step', 'velocity'):
            q[0] += .2 if failure == 'step' else .02
        if failure == 'limit':
            q[0] = node.models['right'].upper[0] + 1
        if failure == 'nan':
            q[0] = np.nan
        result = IKResult(failure != 'ik', q, 2, .1, .1, 'injected failure')
        with patch.object(node.solvers['right'], 'solve', return_value=result):
            node.cycle()
    assert not node.failed
    np.testing.assert_array_equal(node.segment_target, previous_target)
    assert left.call_count == right.call_count == 1
    np.testing.assert_array_equal(left.call_args.args[0].data, previous_target[0])
    np.testing.assert_array_equal(right.call_args.args[0].data, previous_target[1])


def test_feedback_order_and_nonfinite_abort(controller):
    from sensor_msgs.msg import JointState
    node, _, _ = controller
    q = INITIAL['left']
    node.feedback('left', JointState(name=node.models['left'].joint_names[::-1],
                                     position=q[::-1].tolist()))
    np.testing.assert_array_equal(node.positions['left'], q)
    node.feedback('right', JointState(name=node.models['right'].joint_names,
                                      position=[float('nan')] * 6))
    assert node.failed


def test_live_ros_feedback_loss_holds_both_targets(description):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import Float64MultiArray
    context = Context()
    rclpy.init(context=context, domain_id=175)
    parameters = [Parameter('robot_description', value=description),
                  Parameter('move_to_initial', value=False)]
    mock = load_script('dual_mock_robot').DualMockRobot(
        context=context, parameter_overrides=parameters)
    controller = load_script('dual_test_5_cartesion_sychro_motion').DualCartesianController(
        context=context, parameter_overrides=parameters)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(mock)
    executor.add_node(controller)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)

    try:
        deadline = time.monotonic() + 3
        while controller.starts is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert controller.starts is not None
        spin_for(.25)
        assert not controller.failed
        assert not np.array_equal(mock.positions['left'], INITIAL['left'])
        # The mock rejects bad commands without altering its accepted position.
        before = mock.positions['right'].copy()
        mock.command('right', Float64MultiArray(data=[float('nan')] * 6))
        np.testing.assert_array_equal(mock.positions['right'], before)
        with patch.object(mock.arm_publishers['right'], 'publish'):
            spin_for(.4)
            assert not controller.failed
            held = {s: q.copy() for s, q in mock.positions.items()}
            spin_for(.1)
            for side in INITIAL:
                np.testing.assert_array_equal(mock.positions[side], held[side])
        # Valid feedback allows new target pairs to be accepted again.
        spin_for(.1)
        assert not controller.failed
        assert time.monotonic() - controller.received['right'] < .1
    finally:
        executor.shutdown()
        controller.destroy_node()
        mock.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('failure', ['missing', 'malformed'])
def test_standalone_waits_for_valid_bringup_description(failure):
    import rclpy
    from rclpy.context import Context
    from std_msgs.msg import String
    context = Context()
    rclpy.init(context=context, domain_id=175)
    node = load_script('dual_test_5_cartesion_sychro_motion').DualCartesianController(context=context)
    node.timer.cancel()
    try:
        node.cycle()
        assert not node.failed
        assert not node.arm_publishers
        if failure == 'missing':
            node.created_at -= node.settings['ready_timeout'] + 1
            node.next_target_time = time.monotonic()
            node.cycle()
        else:
            node.description_callback(String(data='<robot'))
        assert node.failed
        assert not node.arm_publishers
    finally:
        node.destroy_node()
        context.shutdown()


def test_finite_two_mm_cycle_with_live_mock(description):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    context = Context()
    rclpy.init(context=context, domain_id=175)
    parameters = [Parameter('robot_description', value=description)]
    mock = load_script('dual_mock_robot').DualMockRobot(
        context=context, parameter_overrides=parameters)
    controller = load_script('dual_test_5_cartesion_sychro_motion').DualCartesianController(
        context=context, parameter_overrides=parameters + [
            Parameter('amplitude', value=.002), Parameter('period', value=8.),
            Parameter('cycles', value=1), Parameter('max_velocity', value=.1),
            Parameter('initial_move_time', value=.1)])
    for side in INITIAL:
        mock.positions[side] = INITIAL[side] + np.array([.01, -.01, .005, 0., 0., 0.])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(mock)
    executor.add_node(controller)
    offsets = {side: [] for side in INITIAL}
    try:
        deadline = time.monotonic() + 12
        while not controller.finished and not controller.failed and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if controller.starts:
                for side in INITIAL:
                    pose = controller.models[side].fk(mock.positions[side])
                    offsets[side].append(pose[0, 3] - controller.starts[side][0, 3])
        assert controller.finished and not controller.failed
        for side in INITIAL:
            initial_pose = controller.models[side].fk(INITIAL[side])
            np.testing.assert_allclose(controller.starts[side], initial_pose, atol=1e-10)
        # Drain the last command pair before measuring final mock feedback.
        until = time.monotonic() + .1
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        for side in INITIAL:
            assert max(offsets[side]) > .00198 and min(offsets[side]) < -.00198
            error = pose_error(controller.starts[side], controller.models[side].fk(mock.positions[side]))
            assert np.linalg.norm(error[:3]) <= controller.settings['position_tolerance']
            assert np.linalg.norm(error[3:]) <= controller.settings['orientation_tolerance']
        previous = {s: q.copy() for s, q in mock.positions.items()}
        controller.cycle()
        for side in INITIAL:
            np.testing.assert_array_equal(previous[side], mock.positions[side])
    finally:
        executor.shutdown()
        controller.destroy_node()
        mock.destroy_node()
        context.shutdown()


def test_initial_joint_move_respects_limits_and_shared_timing(description):
    from dual_crx_control.startup_motion import InitialJointMove
    models = {s: CRXKinematics(description, f'{s}_tcp') for s in INITIAL}
    starts = {s: q + np.array([.1, -.2, .05, .3, 0., -.15]) for s, q in INITIAL.items()}
    move = InitialJointMove(models, starts, INITIAL, .1, .1, .1)
    samples = {s: [] for s in INITIAL}
    times = np.linspace(0., move.duration, 1001)
    for t in times:
        for side, q in move.sample(t).items():
            assert models[side].valid_joints(q)
            samples[side].append(q)
    dt = times[1] - times[0]
    for side in INITIAL:
        q = np.array(samples[side])
        np.testing.assert_array_equal(q[0], starts[side])
        np.testing.assert_array_equal(q[-1], INITIAL[side])
        assert np.max(np.abs(np.diff(q, axis=0))) / dt <= .1
        assert np.max(np.abs(np.diff(q, n=2, axis=0))) / dt**2 <= .1
    invalid = {s: q.copy() for s, q in INITIAL.items()}
    invalid['right'][0] = 100
    with pytest.raises(ValueError, match='invalid initial-move'):
        InitialJointMove(models, starts, invalid, 1., .1, .1)


@pytest.mark.parametrize('failure', ['tracking', 'settle', 'stale'])
def test_initial_move_failure_prevents_cartesian_start(controller, failure):
    node, left, right = controller
    node.move_to_initial = True
    node.settings['initial_move_time'] = .01
    if failure == 'tracking':
        for side in INITIAL:
            node.positions[side] = INITIAL[side] + .3
    node.cycle()
    assert node.starts is None
    left.reset_mock()
    right.reset_mock()
    if failure == 'stale':
        node.received['right'] -= 1
    elif failure == 'tracking':
        node.previous['right'] = INITIAL['right'].copy()
    else:
        # Target commands are held but the robot remains just outside tolerance.
        node.positions['right'] = INITIAL['right'] + .006
        node.initial_move_started -= node.initial_move.duration + node.settings['initial_settle_timeout'] + 1
    node.next_target_time = time.monotonic()
    node.cycle()
    assert node.failed
    assert node.starts is None
    if failure != 'settle':
        assert left.call_count == right.call_count == 0


def test_intermediate_commands_share_phase_without_running_ik(controller):
    node, left, right = controller
    with patch('time.monotonic', return_value=100.) as clock:
        node.received = {'left': 100., 'right': 100.}
        node.cycle()
        q0 = node.segment_target.copy()
        q1 = q0 + np.array([[.001] * 6, [-.002] * 6])
        results = [IKResult(True, q, 1, 0., 0., 'test target') for q in q1]
        with patch.object(node.solvers['left'], 'solve', return_value=results[0]) as lsolve, \
                patch.object(node.solvers['right'], 'solve', return_value=results[1]) as rsolve:
            clock.return_value = 100.02
            node.cycle()
            for t, phase in [(100.024, .2), (100.030, .5), (100.038, .9)]:
                clock.return_value = t
                node.cycle()
                expected = q0 + phase * (q1 - q0)
                np.testing.assert_allclose(left.call_args.args[0].data, expected[0], atol=1e-12)
                np.testing.assert_allclose(right.call_args.args[0].data, expected[1], atol=1e-12)
            assert lsolve.call_count == rsolve.call_count == 1
            assert node.ik_count == 2
            clock.return_value = 100.04
            node.cycle()
            np.testing.assert_array_equal(lsolve.call_args.args[1], q1[0])
            np.testing.assert_array_equal(rsolve.call_args.args[1], q1[1])
            np.testing.assert_array_equal(node.segment_start, q1)
            assert not node.failed


def test_recording_preserves_distinct_command_and_feedback(description, tmp_path):
    import csv
    import json
    from dual_crx_control.motion_recording import MotionRecording
    models = {s: CRXKinematics(description, f'{s}_tcp') for s in INITIAL}
    starts = {s: m.fk(INITIAL[s]) for s, m in models.items()}
    recording = MotionRecording(0, max_samples=2)
    recording.command(99., INITIAL)
    assert not recording.commands
    recording.begin(100., starts)
    commanded = {s: q.copy() for s, q in INITIAL.items()}
    commanded['left'][1] += .01
    for t in [100., 100.01, 100.02]:
        recording.command(t, commanded)
    for side in INITIAL:
        recording.measured(side, 100.015, INITIAL[side], 1234.5)
    commanded['left'][1] += 1
    png, csv_path, metadata_path = recording.save(models, tmp_path)
    assert png.stat().st_size > 1000
    with csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    commands = [r for r in rows if r['arm'] == 'left' and r['source'] == 'command']
    feedback = [r for r in rows if r['arm'] == 'left' and r['source'] == 'feedback']
    assert len(commands) == 2 and len(feedback) == 1
    assert float(commands[-1]['J2_rad']) == .01
    assert float(feedback[0]['J2_rad']) == 0.
    assert abs(float(commands[-1]['displacement_mm'])) > 1
    assert float(feedback[0]['displacement_mm']) == 0.
    assert np.isclose(float(feedback[0]['time_s']), .015)
    assert float(feedback[0]['feedback_ros_stamp_s']) == 1234.5
    assert json.loads(metadata_path.read_text())['recording_truncated']
