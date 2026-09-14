"""Offline and isolated ROS tests. No test publishes to a real robot domain.

Run with the Python environment documented in CIRCULAR_MOTION.md:
  python -m unittest discover -s src/dual_crx_control/tests -v
"""

import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.interpolate import CubicSpline

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dual_test4 as motion  # noqa: E402


class PlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.args = motion.parse_args(['--dry-run'])
        cls.robot = motion.RobotModel(cls.args.urdf)
        cls.robot.set_initial_configuration(np.zeros(12))
        cls.plan = motion.CirclePlan(cls.robot, cls.args)

    def test_zero_pose_mapping_and_full_circle(self):
        robot, plan = self.robot, self.plan
        np.testing.assert_allclose(robot.arms[0].initial_position, [0.575, -0.430, 0.595])
        np.testing.assert_allclose(robot.arms[1].initial_position, [0.575, 0.170, 0.595])
        self.assertEqual([a.namespace for a in robot.arms], ['robot1', 'robot2'])
        self.assertTrue(robot.arms[0].q_indices[0] > robot.arms[1].q_indices[0])
        for t in np.linspace(0, plan.duration, 301):
            q, targets = plan.sample(t)
            for arm, pose, target, center in zip(robot.arms, robot.poses(q), targets, plan.centers):
                np.testing.assert_allclose(pose.translation, target, atol=motion.POSITION_TOLERANCE)
                self.assertAlmostEqual(np.linalg.norm(target[:2] - center[:2]), 0.1)
                self.assertAlmostEqual(target[2], arm.initial_position[2])
                np.testing.assert_array_equal(q[arm.q_indices[3:]], np.zeros(3))
        np.testing.assert_allclose(plan.sample(0)[0], robot.initial_q, atol=1e-12)
        np.testing.assert_allclose(plan.sample(plan.duration)[0], robot.initial_q, atol=1e-12)
        self.assertGreater(plan.summary['certified_collision_clearance_lower_bound_m'], self.args.collision_margin)
        self.assertTrue(np.all(plan.speed_bound <= robot.model.velocityLimit * self.args.velocity_scale))
        self.assertTrue(np.all(plan.acceleration_bound <= self.args.max_acceleration))

    def test_clockwise_multiple_cycles_and_retiming(self):
        args = motion.parse_args(['--dry-run', '--direction', 'cw', '--cycles', '2', '--duration', '0.1'])
        # Reuse collision proofs for the same geometric curve traversed in reverse.
        # IK and all Cartesian/derivative checks still run for this variant.
        with patch.object(self.robot, 'check_collision_interval', return_value=0.01):
            plan = motion.CirclePlan(self.robot, args)
        self.assertGreater(plan.duration, args.duration)
        np.testing.assert_allclose(plan.sample(plan.duration / 2)[0], self.robot.initial_q, atol=1e-12)
        self.assertLess(plan.targets_at_phase(0.25)[0, 1], plan.centers[0, 1])
        self.assertTrue(np.all(plan.acceleration_bound <= args.max_acceleration))

    def test_unreachable_target_and_invalid_start_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'IK failed'):
            self.robot.solve_ik(self.robot.initial_q, self.robot.arms[0], np.array([10., 0., 0.]), False)
        q = self.robot.initial_q.copy()
        q[self.robot.arms[0].q_indices[0]] = np.radians(1)
        with self.assertRaisesRegex(RuntimeError, 'moved since the initial snapshot'):
            motion.validate_start(self.robot, self.args, q)
        # A permitted small deviation must have a validated continuous blend.
        q[self.robot.arms[0].q_indices[0]] = np.radians(0.1)
        motion.validate_start(self.robot, self.args, q)

    def test_collision_checker_catches_interior_and_static_pairs(self):
        robot = self.robot
        pairs = [{robot.geometry.geometryObjects[p.first].name,
                  robot.geometry.geometryObjects[p.second].name}
                 for p in robot.geometry.collisionPairs]
        self.assertIn({'left_base_link_0', 'calibration_table_surface_0'}, pairs)
        with patch.object(robot, 'clearance', return_value=(-0.001, 'test_pair')):
            with self.assertRaisesRegex(RuntimeError, 'Collision clearance'):
                robot.check_collision_interval(lambda s: robot.initial_q, 0, 1, np.ones(12), 0.002)

    def test_spline_extrema_include_overshoot(self):
        # With zero endpoints and opposing derivatives, the interior reaches 1.
        spline = CubicSpline([0, 1], [[0], [0]], bc_type=((1, [4]), (1, [-4])))
        low, high, speed, acceleration = motion.spline_bounds(spline)
        np.testing.assert_allclose([low[0], high[0], speed[0], acceleration[0]], [0, 1, 4, 8])

    def test_circle_from_nonzero_joint_snapshot_preserves_wrists_and_closes(self):
        robot = motion.RobotModel(self.args.urdf)
        q = np.zeros(12)
        for arm, joints in zip(robot.arms, ([-.2, .1, .15, 0, .2, 0], [.1, -.05, .1, 0, -.2, .02])):
            q[arm.q_indices] = joints
        robot.set_initial_configuration(q)
        plan = motion.CirclePlan(robot, self.args)
        for t in np.linspace(0, plan.duration, 31):
            command, targets = plan.sample(t)
            for arm, pose, target in zip(robot.arms, robot.poses(command), targets):
                np.testing.assert_allclose(pose.translation, target, atol=motion.POSITION_TOLERANCE)
                np.testing.assert_allclose(command[arm.q_indices[3:]], q[arm.q_indices[3:]], atol=1e-12)
        np.testing.assert_allclose(plan.sample(0)[0], q, atol=1e-12)
        np.testing.assert_allclose(plan.sample(plan.duration)[0], q, atol=1e-12)

    def test_reported_real_pose_is_accepted_as_start_but_collision_is_retained(self):
        robot = motion.RobotModel(self.args.urdf)
        q = np.zeros(12)
        degrees = ([-90.0005, -.0025, 180.0005, -.0001, 90.0002, -.0008],
                   [0, -.0013, 0, .0001, -90.0001, .0007])
        for arm, angles in zip(robot.arms, degrees):
            q[arm.q_indices] = np.radians(angles)
        robot.set_initial_configuration(q)
        np.testing.assert_array_equal(robot.initial_q, q)
        with self.assertRaisesRegex(RuntimeError, 'Initial configuration collision.*left_J2_link.*right_J4_link'):
            motion.CirclePlan(robot, self.args)


class InputTests(unittest.TestCase):
    def test_saved_snapshot_is_explicit_and_dry_run_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'initial_joints.json'
            snapshot = {'robot1': [-.2, .1, .15, 0, .2, 0], 'robot2': [.1, -.05, .1, 0, -.2, .02]}
            path.write_text(json.dumps(snapshot))
            args = motion.parse_args(['--dry-run', '--initial-joints-file', str(path)])
            robot = motion.RobotModel(args.urdf)
            metadata = motion.capture_initial_configuration(robot, args)
            self.assertEqual(metadata['source'], str(path))
            for arm in robot.arms:
                np.testing.assert_array_equal(robot.initial_q[arm.q_indices], snapshot[arm.namespace])
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                motion.parse_args(['--initial-joints-file', str(path)])
            snapshot['robot2'] = [0, 0]
            path.write_text(json.dumps(snapshot))
            with self.assertRaisesRegex(ValueError, 'six finite joint angles'):
                motion.capture_initial_configuration(robot, args)

    def test_rotation_jacobian_matches_world_frame_finite_difference(self):
        target = motion.pin.exp3(np.array([0.2, -0.3, 0.4]))
        current = motion.pin.exp3(np.array([-0.1, 0.2, 0.05]))
        world_increment = np.array([0.7, 0.8, -0.1])
        error, jacobian = motion.orientation_task(target, current, np.eye(3))
        epsilon = 1e-7
        moved_rotation = motion.pin.exp3(epsilon * world_increment) @ current
        moved_error = motion.pin.log3(target @ moved_rotation.T)
        np.testing.assert_allclose((moved_error - error) / epsilon,
                                   -jacobian @ world_increment, atol=2e-8)

    def test_joint_names_override_feedback_order(self):
        names = [f'robot1_J{i}' for i in range(6, 0, -1)]
        np.testing.assert_array_equal(motion.ordered_positions(names, list(range(6, 0, -1)), 'robot1'), range(1, 7))
        self.assertIsNone(motion.ordered_positions(names[:-1], [0] * 5, 'robot1'))
        self.assertIsNone(motion.ordered_positions(names, [0] * 5 + [np.nan], 'robot1'))
        self.assertIsNone(motion.ordered_positions(names, [0] * 5, 'robot1'))
        self.assertIsNone(motion.ordered_positions(names + names[:1], [0] * 7, 'robot1'))

    def test_invalid_cli_parameters(self):
        for arguments in (['--radius', 'nan'], ['--radius', '-1'], ['--cycles', '0'],
                          ['--duration', '0'], ['--velocity-scale', '2'], ['--rate', '0']):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    motion.parse_args(arguments)


class RosInterfaceTests(unittest.TestCase):
    """Real ROS messages/services on a dedicated domain, with fake controllers."""

    @classmethod
    def setUpClass(cls):
        # A separate ROS domain plus localhost-only discovery keeps the actual
        # robot namespaces isolated from hardware, even on the robot workstation.
        os.environ['ROS_DOMAIN_ID'] = '173'
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from controller_manager_msgs.srv import ListControllers
        from controller_manager_msgs.msg import ControllerState

        cls.rclpy = rclpy
        rclpy.init()
        cls.args = motion.parse_args(['--state-timeout', '0.2', '--ready-timeout', '3'])
        cls.robot = motion.RobotModel(cls.args.urdf)
        cls.nodes, cls.received = [], {'robot1': [], 'robot2': []}
        cls.states = {'robot1': np.zeros(6), 'robot2': np.zeros(6)}
        cls.enabled = {'robot1': True, 'robot2': True}
        cls.active = {'robot1': True, 'robot2': True}
        cls.executor = SingleThreadedExecutor()

        def commands(ns, message):
            cls.received[ns].append(list(message.data))
            cls.states[ns] = np.array(message.data)

        def list_controllers(ns, request, response):
            response.controller = [ControllerState(
                name='forward_position_controller', state='active' if cls.active[ns] else 'inactive')]
            return response

        def publish_state(ns, node, publisher):
            if not cls.enabled[ns]:
                return
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = [f'{ns}_J{i}' for i in range(6, 0, -1)]
            message.position = cls.states[ns][::-1].tolist()
            publisher.publish(message)

        for ns in ('robot1', 'robot2'):
            node = Node('forward_position_controller', namespace=ns)
            node.declare_parameter('joints', [f'{ns}_J{i}' for i in range(1, 7)])
            node.create_subscription(Float64MultiArray, f'/{ns}/forward_position_controller/commands',
                                     lambda msg, ns=ns: commands(ns, msg), 1)
            node.create_service(ListControllers, f'/{ns}/controller_manager/list_controllers',
                                lambda req, res, ns=ns: list_controllers(ns, req, res))
            publisher = node.create_publisher(JointState, f'/{ns}/joint_states', qos_profile_sensor_data)
            node.create_timer(0.005, lambda ns=ns, node=node, pub=publisher: publish_state(ns, node, pub))
            cls.nodes.append(node)
            cls.executor.add_node(node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.thread.join(timeout=3)
        for node in cls.nodes:
            node.destroy_node()
        cls.rclpy.shutdown()

    def setUp(self):
        self.robot.set_initial_configuration(np.zeros(12))
        for ns in ('robot1', 'robot2'):
            self.enabled[ns], self.active[ns] = True, True
            self.states[ns] = np.zeros(6)
            self.received[ns].clear()
        self.io = motion.RobotIO(self.robot, self.args)
        self.io.wait_ready()
        self.io.verify_interfaces()

    def tearDown(self):
        if self.io is not None:
            self.io.node.destroy_node()

    def test_pair_commands_and_stale_arm_abort(self):
        q = self.robot.initial_q.copy()
        q[self.robot.arms[0].q_indices] = np.arange(6) / 1000
        q[self.robot.arms[1].q_indices] = -np.arange(6) / 1000
        self.io.guard(self.robot.initial_q)
        self.io.publish_pair(q)
        deadline = time.monotonic() + 2
        while not all(self.received.values()) and time.monotonic() < deadline:
            time.sleep(0.01)
        for arm in self.robot.arms:
            np.testing.assert_allclose(self.received[arm.namespace][-1], q[arm.q_indices])
        self.enabled['robot2'] = False
        # Drain the final queued sample, then let only robot2 go stale.
        self.io.spin()
        time.sleep(self.args.state_timeout + 0.05)
        with self.assertRaisesRegex(RuntimeError, 'Stale feedback'):
            self.io.guard(q)
        self.assertEqual([len(self.received[ns]) for ns in ('robot1', 'robot2')], [1, 1])

    def test_inactive_controller_and_tracking_error_abort(self):
        self.active['robot2'] = False
        self.io.check_controllers(force=True)
        deadline = time.monotonic() + 2
        with self.assertRaisesRegex(RuntimeError, 'not active'):
            while time.monotonic() < deadline:
                self.io.spin()
                self.io.check_controllers()
                time.sleep(0.01)
        self.active['robot2'] = True
        self.io.controller_futures.clear()
        self.io.verify_interfaces()
        wrong_command = np.full(12, np.radians(10))
        self.io.guard(wrong_command)
        time.sleep(self.args.tracking_timeout + 0.05)
        with self.assertRaisesRegex(RuntimeError, 'tracking error persisted'):
            self.io.guard(wrong_command)

    def test_missing_subscriber_and_joint_order_rejected(self):
        ns = 'robot2'
        with patch.object(self.io.publishers[ns], 'get_subscription_count', return_value=0):
            with self.assertRaisesRegex(RuntimeError, 'missing command subscriber'):
                self.io.guard(self.robot.initial_q)
        from rclpy.parameter import Parameter
        node = self.nodes[1]
        node.set_parameters([Parameter('joints', value=[f'{ns}_J{i}' for i in range(6, 0, -1)])])
        try:
            with self.assertRaisesRegex(RuntimeError, 'must be ordered'):
                self.io.verify_interfaces()
        finally:
            node.set_parameters([Parameter('joints', value=[f'{ns}_J{i}' for i in range(1, 7)])])

    def short_test_plan(self):
        args = motion.parse_args([
            '--rate', '100', '--approach-time', '0.1', '--initial-hold', '0.05',
            '--final-hold', '0.1', '--max-lateness', '0.15', '--state-timeout', '0.1',
        ])

        def sample(elapsed):
            q = self.robot.initial_q.copy()
            # Tiny synthetic motion tests the streamer, not Cartesian planning
            # (the actual 10 cm geometry is covered by PlanningTests).
            for arm in self.robot.arms:
                q[arm.q_indices[0]] += 0.001 * np.sin(np.pi * elapsed / 0.3)**2
            targets = np.array([pose.translation for pose in self.robot.poses(q)])
            return q, targets

        return SimpleNamespace(robot=self.robot, args=args, duration=0.3, sample=sample,
                               centers=np.array([a.initial_position - [args.radius, 0, 0]
                                                 for a in self.robot.arms]))

    def test_execution_loop_and_saved_measurements(self):
        self.io.node.destroy_node()
        self.io = None
        plan = self.short_test_plan()
        records = []
        # The test fixture owns this isolated ROS context; execute owns its node.
        with patch('rclpy.init'), patch('rclpy.shutdown'), patch('builtins.input', return_value=''):
            motion.execute(plan, plan.args, records)
        self.assertGreater(len(records), 10)
        self.assertEqual(records[-1]['stage'], 'final_hold')
        np.testing.assert_allclose(records[-1]['command'], self.robot.initial_q, atol=1e-12)
        with tempfile.TemporaryDirectory() as directory:
            result = motion.save_records(plan, records, Path(directory), 'test')
            self.assertTrue((Path(directory) / 'test.csv').is_file())
            self.assertTrue((Path(directory) / 'test_cartesian.png').is_file())
            self.assertTrue((Path(directory) / 'test_joints.png').is_file())
            self.assertIn('max_publish_span_s', result)

    def test_interrupted_stream_preserves_partial_records(self):
        self.io.node.destroy_node()
        self.io = None
        plan = self.short_test_plan()
        records = []
        original_publish = motion.RobotIO.publish_pair

        def publish_then_interrupt(adapter, q):
            if len(records) == 5:
                raise KeyboardInterrupt()
            return original_publish(adapter, q)

        with patch('rclpy.init'), patch('rclpy.shutdown'), patch('builtins.input', return_value=''):
            with patch.object(motion.RobotIO, 'publish_pair', publish_then_interrupt):
                with self.assertRaises(KeyboardInterrupt):
                    motion.execute(plan, plan.args, records)
        self.assertEqual(len(records), 5)
        with tempfile.TemporaryDirectory() as directory:
            motion.save_records(plan, records, Path(directory), 'interrupted')
            lines = (Path(directory) / 'interrupted.csv').read_text().splitlines()
            self.assertEqual(len(lines), 6)  # Header plus every successfully sent pair.

    def test_live_capture_is_read_only_and_stream_returns_to_nonzero_snapshot(self):
        self.io.node.destroy_node()
        self.io = None
        for ns, joints in zip(('robot1', 'robot2'), ([-.2, .1, .15, 0, .2, 0], [.1, -.05, .1, 0, -.2, .02])):
            self.states[ns] = np.array(joints)
        reader = motion.RobotIO(self.robot, self.args, read_only=True)
        try:
            reader.wait_ready()
            self.assertEqual(reader.publishers, {})
            self.assertEqual(reader.controller_clients, {})
            with self.assertRaisesRegex(RuntimeError, 'cannot publish'):
                reader.publish_pair(np.zeros(12))
        finally:
            reader.node.destroy_node()
        with patch('rclpy.init'), patch('rclpy.shutdown'):
            metadata = motion.capture_initial_configuration(self.robot, self.args)
        self.assertEqual(metadata['source'], 'live_joint_states')
        for arm in self.robot.arms:
            np.testing.assert_array_equal(self.robot.initial_q[arm.q_indices], self.states[arm.namespace])
        self.assertTrue(all(len(commands) == 0 for commands in self.received.values()))
        captured = self.robot.initial_q.copy()
        plan = self.short_test_plan()
        records = []
        with patch('rclpy.init'), patch('rclpy.shutdown'), patch('builtins.input', return_value=''):
            motion.execute(plan, plan.args, records)
        np.testing.assert_allclose(records[0]['command'], captured, atol=1e-12)
        np.testing.assert_allclose(records[-1]['command'], captured, atol=1e-12)

    def test_movement_during_operator_prompt_rejects_before_any_command(self):
        self.io.node.destroy_node()
        self.io = None
        plan = self.short_test_plan()
        records = []

        def move_during_prompt(prompt):
            self.states['robot1'][0] += np.radians(2)
            return ''

        with patch('rclpy.init'), patch('rclpy.shutdown'), patch('builtins.input', side_effect=move_during_prompt):
            with self.assertRaisesRegex(RuntimeError, 'moved since the initial snapshot'):
                motion.execute(plan, plan.args, records)
        self.assertEqual(records, [])
        self.assertTrue(all(len(commands) == 0 for commands in self.received.values()))

    def test_capture_times_out_without_both_arms_instead_of_using_zero(self):
        self.io.node.destroy_node()
        self.io = None
        self.enabled['robot2'] = False
        args = motion.parse_args(['--dry-run', '--ready-timeout', '0.15'])
        with patch('rclpy.init'), patch('rclpy.shutdown'):
            with self.assertRaisesRegex(RuntimeError, 'Robots not ready'):
                motion.capture_initial_configuration(self.robot, args)
        self.assertTrue(all(len(commands) == 0 for commands in self.received.values()))


if __name__ == '__main__':
    unittest.main()
