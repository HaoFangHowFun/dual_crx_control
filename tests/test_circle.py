"""Circle geometry, full-pose IK, and inherited streaming checks."""

import importlib.util
from pathlib import Path
import time

import numpy as np
import pytest
import xacro

from dual_crx_control.circular_trajectory import CircularTrajectory
from dual_crx_control.kinematics import CRXKinematics
from dual_crx_control.ik_solver import DampedLeastSquaresIK, pose_error
from dual_crx_control.startup_motion import INITIAL_JOINTS_DEG

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('plane', ['xy', 'xz', 'yz'])
@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_circle_geometry_and_full_pose_ik(plane, direction):
    circle = CircularTrajectory(plane=plane, direction=direction)
    xml = xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()
    models = {s: CRXKinematics(xml, f'{s}_tcp') for s in INITIAL_JOINTS_DEG}
    seeds = {s: np.radians(q) for s, q in INITIAL_JOINTS_DEG.items()}
    starts = {s: m.fk(seeds[s]) for s, m in models.items()}
    center = np.zeros(3)
    center[circle.axes[0]] = -circle.radius
    solvers = {s: DampedLeastSquaresIK(m) for s, m in models.items()}
    for t in np.linspace(0., circle.period, 401):
        offset = circle.offset(t)
        assert np.isclose(np.linalg.norm(offset - center), circle.radius)
        for side in models:
            target = starts[side].copy()
            target[:3, 3] += offset
            result = solvers[side].solve(target, seeds[side])
            assert result.success, result
            error = pose_error(target, models[side].fk(result.q))
            assert np.linalg.norm(error[:3]) <= 1e-5
            assert np.linalg.norm(error[3:]) <= 1e-5
            assert np.max(np.abs(result.q - seeds[side])) / .02 < .5
            seeds[side] = result.q
    np.testing.assert_array_equal(circle.offset(0.), np.zeros(3))
    np.testing.assert_array_equal(circle.offset(8.), np.zeros(3))
    np.testing.assert_array_equal(circle.offset(9.), np.zeros(3))
    sign = 1 if direction == 'ccw' else -1
    assert sign * circle.offset(2.)[circle.axes[1]] > 0


@pytest.mark.parametrize('kwargs', [{'radius': -1}, {'radius': np.nan}, {'period': 0},
                                    {'plane': 'ab'}, {'direction': 'unknown'}])
def test_bad_circle_parameters(kwargs):
    with pytest.raises(ValueError):
        CircularTrajectory(**kwargs)


def test_full_circle_stream_and_path_plot(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    def load(name):
        spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    xml = xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()
    context = Context()
    rclpy.init(context=context, domain_id=176)
    parameters = [Parameter('robot_description', value=xml)]
    mock = load('dual_mock_robot').DualMockRobot(context=context, parameter_overrides=parameters)
    controller = load('dual_test_6_cartesian_circle_motion').DualCircleController(
        context=context, parameter_overrides=parameters + [Parameter('initial_move_time', value=.1),
                                                           Parameter('cycles', value=2),
                                                           Parameter('period', value=4.)])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(mock)
    executor.add_node(controller)
    try:
        deadline = time.monotonic() + 12
        while not controller.finished and not controller.failed and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.002)
        assert controller.finished and not controller.failed
        assert controller.ik_count > 380
        assert controller.command_count > 3800
        assert controller.settings['cycles'] == 2
        # Inspect actual published joint commands around the first lap boundary.
        circle = controller.circle
        crossing = (2 * circle.period - circle.period / 4.) / 2. + circle.period / 8.
        for side_index, side in enumerate(INITIAL_JOINTS_DEG):
            samples = [(t, controller.models[side].fk(q[side_index])[:3, 3])
                       for t, q in controller.recording.commands if abs(t - crossing) < .15]
            dt = np.diff([t for t, _ in samples])
            speed = np.linalg.norm(np.diff([p for _, p in samples], axis=0), axis=1) / dt
            assert len(speed) > 20
            assert np.percentile(speed, 5) > .02  # No stop between laps.
        for side in INITIAL_JOINTS_DEG:
            error = pose_error(controller.starts[side], controller.models[side].fk(mock.positions[side]))
            assert np.linalg.norm(error[:3]) < 2e-5
            assert np.linalg.norm(error[3:]) < 2e-5
        png, csv_path, _ = controller.recording.save(controller.models, tmp_path)
        assert 'circle_xy_' in png.name and png.stat().st_size > 1000
        assert 'world_y_m' in csv_path.read_text().splitlines()[0]
    finally:
        executor.shutdown()
        mock.destroy_node()
        controller.destroy_node()
        context.shutdown()


@pytest.mark.parametrize('cycles', [1, 2, 5, 0])
def test_speed_ramps_only_at_run_endpoints(cycles):
    circle = CircularTrajectory(period=8.)
    ramp = circle.period / 4.
    duration = cycles * circle.period if cycles else 40.
    expected_rate = cycles / (duration - ramp) if cycles else 1. / circle.period
    epsilon = 1e-4

    def speed(t):
        return (circle.phase(t + epsilon, cycles) - circle.phase(t - epsilon, cycles)) / (2 * epsilon)

    for t in np.linspace(ramp + .1, duration - ramp - .1, 31):
        assert np.isclose(speed(t), expected_rate, rtol=1e-6)
    assert circle.phase(epsilon, cycles) / epsilon < 1e-8
    if cycles:
        assert (cycles - circle.phase(duration - epsilon, cycles)) / epsilon < 1e-8
        assert circle.phase(duration, cycles) == cycles
        for lap in range(1, cycles):
            crossing = lap / expected_rate + ramp / 2.
            assert np.isclose(circle.phase(crossing, cycles), lap)
            assert np.isclose(speed(crossing), expected_rate, rtol=1e-6)
    else:
        # Indefinite runs never schedule a final slowdown.
        assert np.isclose(speed(80.), expected_rate, rtol=1e-6)
