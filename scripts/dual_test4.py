#!/usr/bin/env python3
"""Plan and stream simultaneous world-XY circles for two FANUC CRX arms.

robot1 is right_; robot2 is left_. All internal distances are metres and
joint angles are radians. Initial joints come from fresh ROS feedback.
Run --dry-run first; it reads feedback but never creates command publishers.
See ../CIRCULAR_MOTION.md for dependencies, assumptions and examples.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from functools import partial
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline


DEFAULT_URDF = Path(__file__).resolve().parents[1] / 'description/urdf/dual_crx.urdf.xacro'
POSITION_TOLERANCE = 0.001
ORIENTATION_TOLERANCE = math.radians(0.5)
IK_TOLERANCE = 1e-9
IK_DAMPING = 1e-5
PATH_SEGMENTS = 720


@dataclass
class Arm:
    """Explicit correspondence between one hardware arm and the URDF model."""

    namespace: str
    prefix: str
    frame: int
    q_indices: np.ndarray
    v_indices: np.ndarray
    initial_position: np.ndarray
    initial_rotation: np.ndarray


def smoothstep(u):
    """Quintic blend, with zero velocity and acceleration at either end."""
    u = np.clip(u, 0.0, 1.0)
    return u**3 * (10.0 + u * (-15.0 + 6.0 * u))


def ordered_positions(names, positions, namespace):
    """Decode feedback by name, never by the incoming array order."""
    if len(names) != len(positions) or len(set(names)) != len(names):
        return None
    by_name = dict(zip(names, positions))
    try:
        q = np.array([by_name[f'{namespace}_J{i}'] for i in range(1, 7)])
    except KeyError:
        return None
    return q if np.all(np.isfinite(q)) else None


def orientation_task(target_rotation, current_rotation, world_angular_jacobian):
    """Return world rotation error and the Jacobian that decreases that error.

    E = R_target R_current.T; a world rotation increment gives E exp(-omega).
    Pinocchio's Jlog3 differentiates this right-multiplied increment.
    """
    error_rotation = target_rotation @ current_rotation.T
    return pin.log3(error_rotation), pin.Jlog3(error_rotation) @ world_angular_jacobian


class RobotModel:
    """URDF kinematics and collision geometry, without any ROS node."""

    def __init__(self, urdf_path):
        import xacro
        from ament_index_python.packages import get_package_share_directory

        self.xml = xacro.process_file(str(urdf_path)).toxml()
        root = ET.fromstring(self.xml)
        # Resolve package:// meshes explicitly using the sourced ROS overlay.
        for mesh in root.iter('mesh'):
            filename = mesh.attrib['filename']
            if filename.startswith('package://'):
                package, relative = filename[10:].split('/', 1)
                mesh.set('filename', str(Path(get_package_share_directory(package)) / relative))
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = pin.buildModelFromXML(self.xml)
        self.data = self.model.createData()
        # Neutral coordinates bootstrap frame/index discovery only. Planning is
        # forbidden until set_initial_configuration() receives a real snapshot.
        self.initial_q = pin.neutral(self.model)
        self.initial_configuration_set = False
        if self.model.nq != 12 or self.model.nv != 12:
            raise ValueError('Expected a fixed-base model with 12 revolute joints.')

        pin.framesForwardKinematics(self.model, self.data, self.initial_q)
        self.arms = []
        for namespace, prefix in (('robot1', 'right_'), ('robot2', 'left_')):
            joints = []
            for i in range(1, 7):
                name = f'{prefix}J{i}'
                if not self.model.existJointName(name):
                    raise ValueError(f'Missing model joint: {name}')
                joint = self.model.joints[self.model.getJointId(name)]
                if joint.nq != 1 or joint.nv != 1:
                    raise ValueError(f'{name} must be a one-coordinate revolute joint.')
                joints.append(joint)
            frame_name = f'{prefix}ee_mount'
            if not self.model.existFrame(frame_name):
                raise ValueError(f'Missing EEF frame: {frame_name}')
            frame = self.model.getFrameId(frame_name)
            pose = self.data.oMf[frame]
            self.arms.append(Arm(
                namespace, prefix, frame,
                np.array([j.idx_q for j in joints]),
                np.array([j.idx_v for j in joints]),
                pose.translation.copy(), pose.rotation.copy(),
            ))
        indices = np.concatenate([a.q_indices for a in self.arms])
        if len(set(indices)) != 12:
            raise ValueError('Arm mappings must cover 12 distinct joints.')

        self.geometry = pin.buildGeomFromUrdfString(
            self.model, self.xml, pin.GeometryType.COLLISION)
        # Include fixed base/table pairs too; addAllCollisionPairs() omits shapes
        # sharing a parent joint, including multiple shapes attached to world.
        for i in range(len(self.geometry.geometryObjects)):
            for j in range(i + 1, len(self.geometry.geometryObjects)):
                self.geometry.addCollisionPair(pin.CollisionPair(i, j))
        adjacent = {
            frozenset((j.find('parent').get('link'), j.find('child').get('link')))
            for j in root.findall('joint')
        }
        # Only direct URDF neighbours and shapes on the same link are excluded.
        # In particular, never exclude cross-arm pairs based on joint index 0.
        self.excluded_pairs = []
        for pair in list(self.geometry.collisionPairs):
            links = [self.model.frames[self.geometry.geometryObjects[i].parentFrame].name
                     for i in (pair.first, pair.second)]
            if links[0] == links[1] or frozenset(links) in adjacent:
                self.excluded_pairs.append(links)
                self.geometry.removeCollisionPair(pair)
        if not self.geometry.collisionPairs:
            raise ValueError('No collision pairs loaded.')
        self.geometry_data = self.geometry.createData()

        # Conservative upper bound on distance from ANY joint to ANY point on
        # its downstream collision shapes. Triangle inequality along the tree.
        origin_sum = sum(np.linalg.norm(j.translation) for j in self.model.jointPlacements)
        shape_reach = 0.0
        for obj in self.geometry.geometryObjects:
            obj.geometry.computeLocalAABB()
            reach = (np.linalg.norm(obj.placement.translation)
                     + np.linalg.norm(obj.geometry.aabb_center) + obj.geometry.aabb_radius)
            shape_reach = max(shape_reach, reach)
        self.motion_radius = float(origin_sum + shape_reach)

    def set_initial_configuration(self, q):
        """Freeze the captured joint snapshot and derive both world EEF poses."""
        q = np.asarray(q, dtype=float)
        if q.shape != (self.model.nq,):
            raise ValueError('Initial configuration must contain exactly 12 joint angles.')
        self.check_limits(q)
        self.initial_q = q.copy()
        for arm, pose in zip(self.arms, self.poses(q)):
            arm.initial_position = pose.translation.copy()
            arm.initial_rotation = pose.rotation.copy()
        self.initial_configuration_set = True

    def poses(self, q):
        """Return independent world-frame EEF poses, in robot1/robot2 order."""
        pin.framesForwardKinematics(self.model, self.data, q)
        return [self.data.oMf[a.frame].copy() for a in self.arms]

    def check_limits(self, q):
        """Reject invalid coordinates instead of clipping a planned command."""
        if (not np.all(np.isfinite(q))
                or np.any(q < self.model.lowerPositionLimit)
                or np.any(q > self.model.upperPositionLimit)):
            raise RuntimeError('Nonfinite or out-of-limit joint target.')

    def clearance(self, q):
        """Return the closest checked pair and signed distance in metres."""
        index = pin.computeDistances(
            self.model, self.data, self.geometry, self.geometry_data, q)
        pair = self.geometry.collisionPairs[index]
        names = [self.geometry.geometryObjects[i].name for i in (pair.first, pair.second)]
        distance = float(self.geometry_data.distanceResults[index].min_distance)
        if not math.isfinite(distance):
            raise RuntimeError('Collision distance is not finite.')
        return distance, ' / '.join(names)

    def check_collision_interval(self, evaluate, a, b, speed_bound, margin, depth=0):
        """Certify an interval using clearance and a conservative motion bound.

        speed_bound bounds |dq/ds| throughout this interval. From its midpoint,
        each shape moves at most R*sum(|dq/ds|)*(b-a)/2. Two shapes can approach
        by twice that amount. Subdivide if this bound cannot prove clearance.
        """
        middle = (a + b) / 2.0
        distance, pair = self.clearance(evaluate(middle))
        if distance <= margin:
            raise RuntimeError(f'Collision clearance {distance:.6f} m at {middle:.6f}: {pair}')
        approach_bound = self.motion_radius * np.sum(speed_bound) * (b - a)
        if distance > margin + approach_bound:
            return distance - approach_bound
        if depth >= 18:
            raise RuntimeError(f'Could not certify clearance near {middle:.6f}: {pair}')
        return min(
            self.check_collision_interval(evaluate, a, middle, speed_bound, margin, depth + 1),
            self.check_collision_interval(evaluate, middle, b, speed_bound, margin, depth + 1),
        )

    def solve_ik(self, seed, arm, target, fixed_orientation):
        """Damped IK; position mode preserves the captured wrist joint angles."""
        q = seed.copy()
        count = 6 if fixed_orientation else 3
        qi, vi = arm.q_indices[:count], arm.v_indices[:count]
        for _ in range(150):
            pin.framesForwardKinematics(self.model, self.data, q)
            pose = self.data.oMf[arm.frame].copy()
            error = target - pose.translation
            jacobian = pin.computeFrameJacobian(
                self.model, self.data, q, arm.frame, pin.LOCAL_WORLD_ALIGNED)[:, vi]
            if fixed_orientation:
                rotation_error, jacobian[3:] = orientation_task(
                    arm.initial_rotation, pose.rotation, jacobian[3:])
                error = np.concatenate((error, rotation_error))
            else:
                jacobian = jacobian[:3]
            if np.linalg.norm(error) < IK_TOLERANCE:
                return q
            singular = np.linalg.svd(jacobian, compute_uv=False)[-1]
            damping = IK_DAMPING + max(0.0, 0.01 - singular) * 0.01
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping**2 * np.eye(len(error)), error)
            step *= min(1.0, 0.08 / max(np.linalg.norm(step), 1e-15))
            # Backtracking keeps each accepted iterate inside joint limits and
            # reduces task error; clipping joints would silently distort IK.
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = q.copy()
                candidate[qi] += scale * step
                if (np.any(candidate < self.model.lowerPositionLimit)
                        or np.any(candidate > self.model.upperPositionLimit)):
                    continue
                pin.framesForwardKinematics(self.model, self.data, candidate)
                new_pose = self.data.oMf[arm.frame]
                new_error = target - new_pose.translation
                if fixed_orientation:
                    new_error = np.concatenate((new_error, pin.log3(arm.initial_rotation @ new_pose.rotation.T)))
                if np.linalg.norm(new_error) < np.linalg.norm(error):
                    q, accepted = candidate, True
                    break
            if not accepted:
                break
        raise RuntimeError(f'{arm.namespace} IK failed for world target {target.tolist()}')


def spline_bounds(spline):
    """Exact per-joint bounds for a piecewise cubic and its first two derivatives."""
    c, widths = spline.c, np.diff(spline.x)
    low, high = np.full(c.shape[2], np.inf), np.full(c.shape[2], -np.inf)
    velocity, acceleration = np.zeros(c.shape[2]), np.zeros(c.shape[2])
    for k, width in enumerate(widths):
        for j in range(c.shape[2]):
            a, b, cc, d = c[:, k, j]
            samples = [0.0, width]
            roots = np.roots([3 * a, 2 * b, cc]) if a or b else []
            samples += [float(r.real) for r in roots if abs(r.imag) < 1e-12 and 0 < r.real < width]
            values = [((a * s + b) * s + cc) * s + d for s in samples]
            low[j], high[j] = min(low[j], min(values)), max(high[j], max(values))
            v_samples = [0.0, width]
            if a and 0 < -b / (3 * a) < width:
                v_samples.append(-b / (3 * a))
            velocity[j] = max(velocity[j], *(abs((3 * a * s + 2 * b) * s + cc) for s in v_samples))
            acceleration[j] = max(acceleration[j], abs(2 * b), abs(6 * a * width + 2 * b))
    return low, high, velocity, acceleration


class CirclePlan:
    """One closed joint-space spline, traversed with a shared smooth clock."""

    def __init__(self, robot, args):
        self.robot, self.args = robot, args
        if not robot.initial_configuration_set:
            raise RuntimeError('Capture both initial joint states before planning.')
        distance, pair = robot.clearance(robot.initial_q)
        if distance <= args.collision_margin:
            raise RuntimeError(f'Initial configuration collision clearance {distance:.6f} m: {pair}')
        self.centers = np.array([a.initial_position - [args.radius, 0, 0] for a in robot.arms])
        self.direction = 1 if args.direction == 'ccw' else -1
        phases = np.linspace(0.0, 1.0, PATH_SEGMENTS + 1)
        q = robot.initial_q.copy()
        waypoints = [q.copy()]
        for phase in phases[1:]:
            for arm, target in zip(robot.arms, self.targets_at_phase(phase)):
                try:
                    q = robot.solve_ik(q, arm, target, args.orientation_mode == 'fixed')
                except RuntimeError as exc:
                    raise RuntimeError(f'Circle phase {phase:.6f}: {exc}') from exc
            waypoints.append(q.copy())
        closure = float(np.max(np.abs(q - robot.initial_q)))
        if closure > 1e-6:
            raise RuntimeError(f'Joint path does not close at captured start: {math.degrees(closure):.6f} deg.')
        # This correction is at most 1 microradian, not an end-of-motion return.
        # The actual periodic interpolant is validated below including its seam.
        waypoints[-1] = robot.initial_q.copy()
        self.path = CubicSpline(phases, waypoints, bc_type='periodic')
        low, high, path_speed, path_acceleration = spline_bounds(self.path)
        robot.check_limits(low)
        robot.check_limits(high)

        # Global derivative bounds give conservative speed/acceleration limits
        # even between validation samples. u=t/T, max s'=1.875, max |s''|=10/sqrt(3).
        phase_speed = args.cycles * 1.875
        phase_acceleration = args.cycles * 10 / math.sqrt(3)
        speed_numerator = path_speed * phase_speed
        acceleration_numerator = (path_acceleration * phase_speed**2
                                  + path_speed * phase_acceleration)
        velocity_limit = robot.model.velocityLimit * args.velocity_scale
        required_time = max(
            float(np.max(speed_numerator / velocity_limit)),
            math.sqrt(float(np.max(acceleration_numerator)) / args.max_acceleration),
        )
        self.duration = max(args.duration, required_time * 1.01)
        self.speed_bound = speed_numerator / self.duration
        self.acceleration_bound = acceleration_numerator / self.duration**2
        print(f'Validating interpolated path; duration {self.duration:.3f} s.', flush=True)

        clearance = math.inf
        for a, b in zip(phases[:-1], phases[1:]):
            clearance = min(clearance, robot.check_collision_interval(
                self.path, a, b, path_speed, args.collision_margin))
        self.summary = {
            'duration_s': self.duration,
            'requested_duration_s': args.duration,
            'joint_closure_before_endpoint_correction_rad': closure,
            'velocity_upper_bounds_rad_s': self.speed_bound.tolist(),
            'acceleration_upper_bounds_rad_s2': self.acceleration_bound.tolist(),
            'certified_collision_clearance_lower_bound_m': clearance,
            'collision_exclusions': robot.excluded_pairs,
            'ik_base_damping': IK_DAMPING,
            'arms': {},
        }
        self.validate_geometry(np.linspace(0, 1, PATH_SEGMENTS * 4 + 1))

    def targets_at_phase(self, phase):
        angle = self.direction * 2 * np.pi * phase
        return self.centers + self.args.radius * np.array([np.cos(angle), np.sin(angle), 0.0])

    def phase_at_time(self, elapsed):
        return self.args.cycles * smoothstep(elapsed / self.duration)

    def sample(self, elapsed):
        """Evaluate joint commands and Cartesian targets at the same elapsed time."""
        phase = self.phase_at_time(elapsed)
        return self.path(phase % 1.0), self.targets_at_phase(phase)

    def validate_geometry(self, phases):
        errors = [[] for _ in self.robot.arms]
        radii = [[] for _ in self.robot.arms]
        heights = [[] for _ in self.robot.arms]
        angles = [[] for _ in self.robot.arms]
        singular_values = [[] for _ in self.robot.arms]
        for phase in phases:
            q = self.path(phase)
            for i, (arm, pose, target) in enumerate(zip(
                    self.robot.arms, self.robot.poses(q), self.targets_at_phase(phase))):
                errors[i].append(float(np.linalg.norm(target - pose.translation)))
                radii[i].append(abs(np.linalg.norm(pose.translation[:2] - self.centers[i, :2]) - self.args.radius))
                heights[i].append(abs(pose.translation[2] - arm.initial_position[2]))
                angles[i].append(float(np.linalg.norm(pin.log3(arm.initial_rotation @ pose.rotation.T))))
                jac = pin.computeFrameJacobian(
                    self.robot.model, self.robot.data, q, arm.frame, pin.LOCAL_WORLD_ALIGNED)
                jac = jac[:, arm.v_indices] if self.args.orientation_mode == 'fixed' else jac[:3, arm.v_indices[:3]]
                singular_values[i].append(float(np.linalg.svd(jac, compute_uv=False)[-1]))
        for i, arm in enumerate(self.robot.arms):
            if max(errors[i]) > POSITION_TOLERANCE:
                raise RuntimeError(f'{arm.namespace}: interpolated FK error exceeds 1 mm.')
            if self.args.orientation_mode == 'fixed' and max(angles[i]) > ORIENTATION_TOLERANCE:
                raise RuntimeError(f'{arm.namespace}: orientation error exceeds 0.5 deg.')
            self.summary['arms'][arm.namespace] = {
                'initial_joints_rad': self.robot.initial_q[arm.q_indices].tolist(),
                'initial_world_m': arm.initial_position.tolist(),
                'center_world_m': self.centers[i].tolist(),
                'max_position_error_m': max(errors[i]),
                'rms_position_error_m': float(np.sqrt(np.mean(np.square(errors[i])))),
                'max_radius_error_m': max(radii[i]),
                'max_z_drift_m': max(heights[i]),
                'max_orientation_change_deg': math.degrees(max(angles[i])),
                'min_task_jacobian_singular_value': min(singular_values[i]),
            }


class RobotIO:
    """ROS feedback reader, with command interfaces enabled only for execution."""

    def __init__(self, robot, args, read_only=False):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from controller_manager_msgs.srv import ListControllers
        from rcl_interfaces.srv import GetParameters

        self.rclpy, self.message_type = rclpy, Float64MultiArray
        self.robot, self.args = robot, args
        self.read_only = read_only
        self.node = rclpy.create_node(
            'dual_crx_initial_state_reader' if read_only else 'dual_crx_circular_motion')
        self.positions, self.received_at, self.stamps = {}, {}, {}
        self.publishers, self.controller_clients, self.parameter_clients = {}, {}, {}
        self.controller_service, self.parameter_service = ListControllers, GetParameters
        self.controller_futures = {}
        self.controller_checked_at = {}
        self.next_controller_check = 0.0
        self.bad_tracking_since = None
        for arm in robot.arms:
            ns = arm.namespace
            self.node.create_subscription(
                JointState, f'/{ns}/joint_states', partial(self.feedback, ns), qos_profile_sensor_data)
            if read_only:
                continue
            self.publishers[ns] = self.node.create_publisher(
                Float64MultiArray, f'/{ns}/forward_position_controller/commands', 1)
            self.controller_clients[ns] = self.node.create_client(
                ListControllers, f'/{ns}/controller_manager/list_controllers')
            self.parameter_clients[ns] = self.node.create_client(
                GetParameters, f'/{ns}/forward_position_controller/get_parameters')

    def feedback(self, namespace, message):
        positions = ordered_positions(message.name, message.position, namespace)
        if positions is not None:
            self.positions[namespace] = positions
            self.received_at[namespace] = time.monotonic()
            self.stamps[namespace] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def spin(self):
        for _ in range(8):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def unready(self):
        now = time.monotonic()
        return [a.namespace for a in self.robot.arms
                if a.namespace not in self.positions
                or now - self.received_at[a.namespace] > self.args.state_timeout
                or (not self.read_only and not self.publishers[a.namespace].get_subscription_count())]

    def read_q(self):
        q = self.robot.initial_q.copy()
        for arm in self.robot.arms:
            q[arm.q_indices] = self.positions[arm.namespace]
        return q

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        while self.rclpy.ok():
            self.spin()
            if not self.unready():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Robots not ready: {self.unready()}')
            time.sleep(0.01)
        raise RuntimeError('ROS shutdown while waiting for robots.')

    def check_controllers(self, force=False):
        now = time.monotonic()
        for ns, future in list(self.controller_futures.items()):
            if future.done():
                result = future.result()
                if result is None or not any(
                        c.name == 'forward_position_controller' and c.state == 'active'
                        for c in result.controller):
                    raise RuntimeError(f'{ns}: forward_position_controller is not active.')
                self.controller_checked_at[ns] = now
                del self.controller_futures[ns]
        if force or now >= self.next_controller_check:
            for ns, client in self.controller_clients.items():
                if ns not in self.controller_futures:
                    self.controller_futures[ns] = client.call_async(self.controller_service.Request())
            self.next_controller_check = now + 0.5

    def verify_interfaces(self):
        """Check active controllers AND their configured six-joint array order."""
        for arm in self.robot.arms:
            ns = arm.namespace
            for client in (self.controller_clients[ns], self.parameter_clients[ns]):
                if not client.wait_for_service(timeout_sec=self.args.ready_timeout):
                    raise RuntimeError(f'{ns}: controller service unavailable.')
            request = self.parameter_service.Request(names=['joints'])
            future = self.parameter_clients[ns].call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.args.ready_timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'{ns}: could not read controller joint order.')
            values = future.result().values
            expected = [f'{ns}_J{i}' for i in range(1, 7)]
            if len(values) != 1 or list(values[0].string_array_value) != expected:
                raise RuntimeError(f'{ns}: controller joints must be ordered {expected}.')
        self.check_controllers(force=True)
        deadline = time.monotonic() + self.args.ready_timeout
        while len(self.controller_checked_at) < 2:
            self.spin()
            self.check_controllers()
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out checking active controllers.')
            time.sleep(0.01)

    def guard(self, previous_command):
        """Reject either arm's failure before the next pair of commands is sent."""
        if not self.rclpy.ok():
            raise RuntimeError('ROS shutdown.')
        self.spin()
        self.check_controllers()
        pending = self.unready()
        if pending:
            raise RuntimeError(f'Stale feedback or missing command subscriber: {pending}')
        now = time.monotonic()
        if any(now - self.controller_checked_at.get(a.namespace, 0) > self.args.ready_timeout
               for a in self.robot.arms):
            raise RuntimeError('Controller state checks timed out.')
        q = self.read_q()
        self.robot.check_limits(q)
        error = np.max(np.abs(q - previous_command))
        if error > math.radians(self.args.tracking_error_deg):
            if self.bad_tracking_since is None:
                self.bad_tracking_since = now
            if now - self.bad_tracking_since >= self.args.tracking_timeout:
                raise RuntimeError(f'Joint tracking error persisted: {math.degrees(error):.3f} deg.')
        else:
            self.bad_tracking_since = None
        return q

    def publish_pair(self, q):
        """Prepare both messages first, then send them without spinning between."""
        if self.read_only:
            raise RuntimeError('The initial-state reader cannot publish motion commands.')
        messages = [self.message_type(data=q[a.q_indices].tolist()) for a in self.robot.arms]
        before = time.monotonic()
        for arm, message in zip(self.robot.arms, messages):
            self.publishers[arm.namespace].publish(message)
        return time.monotonic() - before


def capture_initial_configuration(robot, args):
    """Read both arms before planning, without creating command publishers.

    An explicit saved snapshot is supported only for offline dry-run replay.
    There is deliberately no fallback to zero joints when feedback is missing.
    """
    if args.initial_joints_file is not None:
        snapshot = json.loads(args.initial_joints_file.read_text())
        q = robot.initial_q.copy()
        for arm in robot.arms:
            values = np.asarray(snapshot[arm.namespace], dtype=float)
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise ValueError(f'{arm.namespace}: expected six finite joint angles in radians.')
            q[arm.q_indices] = values
        robot.set_initial_configuration(q)
        return {'source': str(args.initial_joints_file.resolve())}

    import rclpy

    print('Waiting for fresh initial joint states from robot1 and robot2...', flush=True)
    rclpy.init()
    reader = None
    try:
        reader = RobotIO(robot, args, read_only=True)
        reader.wait_ready()
        robot.set_initial_configuration(reader.read_q())
        now = time.monotonic()
        return {
            'source': 'live_joint_states',
            'feedback_stamp_s': reader.stamps.copy(),
            'feedback_age_s': {ns: now - received for ns, received in reader.received_at.items()},
        }
    finally:
        if reader is not None:
            reader.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def validate_start(robot, args, q):
    """Reject drift since capture and validate the small blend back to that snapshot."""
    if not robot.initial_configuration_set:
        raise RuntimeError('Capture both initial joint states before validating execution.')
    robot.check_limits(q)
    error = np.max(np.abs(q - robot.initial_q))
    if error > math.radians(args.start_tolerance_deg):
        details = {a.namespace: np.degrees((q - robot.initial_q)[a.q_indices]).round(4).tolist()
                   for a in robot.arms}
        raise RuntimeError(
            f'Robots moved since the initial snapshot; joint differences in degrees: {details}. '
            'Restart to capture fresh states and replan.')
    duration = args.approach_time
    delta = np.abs(q - robot.initial_q)
    if (np.any(delta * 1.875 / duration > robot.model.velocityLimit * args.velocity_scale)
            or np.any(delta * (10 / math.sqrt(3)) / duration**2 > args.max_acceleration)):
        raise RuntimeError('Initial blend exceeds motion limits; increase --approach-time.')
    robot.check_collision_interval(
        lambda u: q + smoothstep(u) * (robot.initial_q - q),
        0.0, 1.0, delta * 1.875, args.collision_margin)


def execute(plan, args, records):
    """Stream the prevalidated plan; retain every sent pair for later reporting."""
    import rclpy

    robot = plan.robot
    rclpy.init()
    io = None
    try:
        io = RobotIO(robot, args)
        io.wait_ready()
        io.verify_interfaces()
        validate_start(robot, args, io.read_q())
        input('Press ENTER to execute the validated circle on BOTH REAL robots (robot1/right, robot2/left)...')
        io.positions.clear()
        io.received_at.clear()
        io.controller_checked_at.clear()
        io.wait_ready()
        io.verify_interfaces()
        starts = io.read_q()
        validate_start(robot, args, starts)
        # Recheck after planning the short approach; do not use a moved robot's
        # old start as the first command. This is tighter than the snapshot drift tolerance.
        fresh = io.guard(starts)
        if np.max(np.abs(fresh - starts)) > math.radians(0.05):
            raise RuntimeError('Robot moved during initial-blend validation; restart the script.')

        motion_start = args.approach_time + args.initial_hold
        total_time = motion_start + plan.duration + args.final_hold
        previous_q = starts.copy()
        start_time = next_tick = time.monotonic()
        previous_time = start_time
        while True:
            measured = io.guard(previous_q)
            now = time.monotonic()
            elapsed = now - start_time
            gap = now - previous_time
            if gap > args.max_lateness:
                raise RuntimeError(f'Command loop stalled for {gap:.4f} s.')
            if elapsed < args.approach_time:
                stage = 'approach'
                q = starts + smoothstep(elapsed / args.approach_time) * (robot.initial_q - starts)
                targets = np.array([p.translation for p in robot.poses(q)])
                circle_time = 0.0
            else:
                circle_time = float(np.clip(elapsed - motion_start, 0, plan.duration))
                q, targets = plan.sample(circle_time)
                stage = ('initial_hold' if elapsed < motion_start else
                         'circle' if elapsed < motion_start + plan.duration else 'final_hold')
            robot.check_limits(q)
            if np.max(np.abs(q - previous_q)) > math.radians(args.max_step_deg):
                raise RuntimeError('Next command exceeds --max-step-deg; aborting both arms.')
            publish_span = io.publish_pair(q)
            records.append({
                'time': elapsed, 'circle_time': circle_time, 'stage': stage,
                'command': q.copy(), 'state': measured.copy(), 'target': targets.copy(),
                'feedback_age': [now - io.received_at[a.namespace] for a in robot.arms],
                'feedback_stamp': [io.stamps[a.namespace] for a in robot.arms],
                'publish_span': publish_span, 'lateness': max(0, now - next_tick),
            })
            previous_q, previous_time = q, now
            if elapsed >= total_time:
                if np.max(np.abs(measured - robot.initial_q)) > math.radians(args.start_tolerance_deg):
                    raise RuntimeError('Final feedback has not settled near the captured initial configuration.')
                break
            next_tick += 1.0 / args.rate
            now = time.monotonic()
            if next_tick <= now:
                next_tick += (math.floor((now - next_tick) * args.rate) + 1) / args.rate
            time.sleep(max(0, next_tick - time.monotonic()))
    finally:
        # The hardware write() continues sending its stored target. Exiting this
        # publisher is NOT an emergency stop, and must not trigger a return home.
        if io is not None:
            io.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def save_records(plan, records, output, name, show_plot=False):
    """Compute FK and render only after streaming, keeping the loop lightweight."""
    if not records:
        return {}
    robot = plan.robot
    measured = all('state' in r for r in records)
    times = np.array([r['time'] for r in records])
    actual = np.array([[p.translation for p in robot.poses(r['state'] if measured else r['command'])]
                       for r in records])
    commanded = np.array([[p.translation for p in robot.poses(r['command'])] for r in records])
    targets = np.array([r['target'] for r in records])
    header = ['time_s', 'circle_time_s', 'stage', 'publish_span_s', 'lateness_s']
    for arm in robot.arms:
        ns = arm.namespace
        header += [f'{ns}_command_J{j}_rad' for j in range(1, 7)]
        if measured:
            header += [f'{ns}_state_J{j}_rad' for j in range(1, 7)]
        for label in ('target', 'command_fk', 'feedback_fk' if measured else 'planned_fk'):
            header += [f'{ns}_{label}_{axis}_m' for axis in 'xyz']
        if measured:
            header += [f'{ns}_feedback_age_s', f'{ns}_feedback_stamp_s']
    with (output / f'{name}.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for k, record in enumerate(records):
            row = [record['time'], record['circle_time'], record['stage'],
                   record.get('publish_span', 0), record.get('lateness', 0)]
            for i, arm in enumerate(robot.arms):
                row.extend(record['command'][arm.q_indices])
                if measured:
                    row.extend(record['state'][arm.q_indices])
                row.extend(targets[k, i])
                row.extend(commanded[k, i])
                row.extend(actual[k, i])
                if measured:
                    row.extend([record['feedback_age'][i], record['feedback_stamp'][i]])
            writer.writerow(row)

    # Write measurements before importing plotting libraries, so a plotting
    # failure cannot discard a completed or interrupted run's data.
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/dual_crx_matplotlib')
    import matplotlib
    if not show_plot:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    summary = {}
    fig, axes = plt.subplots(3, 2, figsize=(12, 11))
    for i, arm in enumerate(robot.arms):
        error = np.linalg.norm(actual[:, i] - targets[:, i], axis=1)
        summary[arm.namespace] = {
            'max_position_error_m': float(max(error)),
            'rms_position_error_m': float(np.sqrt(np.mean(error**2))),
        }
        circle = np.array([r['stage'] == 'circle' for r in records])
        if np.any(circle):
            radius_error = np.abs(np.linalg.norm(
                actual[circle, i, :2] - plan.centers[i, :2], axis=1) - plan.args.radius)
            summary[arm.namespace]['max_circle_radius_error_m'] = float(max(radius_error))
            summary[arm.namespace]['max_circle_z_drift_m'] = float(np.max(
                np.abs(actual[circle, i, 2] - arm.initial_position[2])))
        summary[arm.namespace]['final_joint_error_rad'] = (
            records[-1]['state' if measured else 'command'][arm.q_indices]
            - robot.initial_q[arm.q_indices]).tolist()
        axes[0, i].plot(targets[:, i, 0], targets[:, i, 1], label='Target')
        axes[0, i].plot(actual[:, i, 0], actual[:, i, 1], '--', label='Feedback FK' if measured else 'Planned FK')
        axes[0, i].set(xlabel='World X [m]', ylabel='World Y [m]', title=arm.namespace)
        axes[0, i].set_aspect('equal', adjustable='box')
        axes[1, i].plot(times, targets[:, i, 2], label='Target Z')
        axes[1, i].plot(times, actual[:, i, 2], '--', label='FK Z')
        axes[1, i].set(xlabel='Time [s]', ylabel='World Z [m]')
        z_min = min(np.min(actual[:, i, 2]), np.min(targets[:, i, 2]))
        z_max = max(np.max(actual[:, i, 2]), np.max(targets[:, i, 2]))
        axes[1, i].set_ylim(z_min - 0.001, z_max + 0.001)
        axes[1, i].ticklabel_format(axis='y', useOffset=False, style='plain')
        axes[2, i].plot(times, error * 1000, label='Position error')
        axes[2, i].set(xlabel='Time [s]', ylabel='Error [mm]')
        axes[2, i].set_ylim(0, max(1.0, float(max(error)) * 1100))
    for ax in axes.flat:
        ax.grid(True)
        ax.legend()
    fig.tight_layout()
    fig.savefig(output / f'{name}_cartesian.png', dpi=150)
    joints, joint_axes = plt.subplots(6, 2, figsize=(12, 14), sharex=True)
    for i, arm in enumerate(robot.arms):
        for j, index in enumerate(arm.q_indices):
            ax = joint_axes[j, i]
            ax.plot(times, [math.degrees(r['command'][index]) for r in records], label='Command')
            if measured:
                ax.plot(times, [math.degrees(r['state'][index]) for r in records], '--', label='Feedback')
            ax.set(ylabel=f'{arm.namespace} J{j + 1} [deg]')
            ax.grid(True)
            ax.legend()
        joint_axes[-1, i].set_xlabel('Time [s]')
    joints.tight_layout()
    joints.savefig(output / f'{name}_joints.png', dpi=150)
    if show_plot:
        plt.show()
    plt.close(fig)
    plt.close(joints)
    if measured:
        summary['max_publish_span_s'] = max(r['publish_span'] for r in records)
        summary['max_lateness_s'] = max(r['lateness'] for r in records)
        summary['late_ticks_over_one_period'] = sum(r['lateness'] > 1 / plan.args.rate for r in records)
        summary['max_feedback_age_s'] = np.max([r['feedback_age'] for r in records], axis=0).tolist()
        summary['comparison_note'] = 'Latest received joint feedback; not time-aligned or externally measured TCP.'
        summary['mean_publish_rate_hz'] = (len(records) - 1) / (times[-1] - times[0]) if len(records) > 1 else 0
    return summary


def parse_args(argv=None):
    """Parse explicit units and reject invalid values before loading the model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF)
    parser.add_argument('--radius', type=float, default=0.10, help='Circle radius [m].')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--duration', type=float, default=20.0, help='Total circle time [s]; extended if needed.')
    parser.add_argument('--direction', choices=('ccw', 'cw'), default='ccw')
    parser.add_argument('--orientation-mode', choices=('position', 'fixed'), default='position',
                        help='position: solve J1-J3, hold J4-J6 at their captured values; '
                             'fixed: solve all six for initial orientation.')
    parser.add_argument('--rate', type=float, default=500.0, help='Target command frequency [Hz].')
    parser.add_argument('--initial-hold', type=float, default=1.0)
    parser.add_argument('--final-hold', type=float, default=1.0)
    parser.add_argument('--approach-time', type=float, default=2.0)
    parser.add_argument('--start-tolerance-deg', type=float, default=0.5,
                        help='Maximum per-joint drift from captured initial states before execution [deg].')
    parser.add_argument('--state-timeout', type=float, default=1.0)
    parser.add_argument('--ready-timeout', type=float, default=10.0)
    parser.add_argument('--tracking-error-deg', type=float, default=5.0)
    parser.add_argument('--tracking-timeout', type=float, default=0.2)
    parser.add_argument('--max-step-deg', type=float, default=0.5)
    parser.add_argument('--max-lateness', type=float, default=0.05, help='Maximum gap between commands [s].')
    parser.add_argument('--velocity-scale', type=float, default=0.1, help='Fraction of URDF velocity limits.')
    parser.add_argument('--max-acceleration', type=float, default=0.5, help='Per-joint acceleration limit [rad/s^2].')
    parser.add_argument('--collision-margin', type=float, default=0.002, help='Required geometry clearance [m].')
    parser.add_argument('--dry-run', action='store_true',
                        help='Capture live initial states and validate without publishing motion commands.')
    parser.add_argument('--initial-joints-file', type=Path,
                        help='Offline dry-run only: JSON with robot1/robot2 J1-J6 arrays in radians.')
    parser.add_argument('--output-dir', type=Path, default=Path('circular_motion_results'))
    parser.add_argument('--show-plot', action='store_true')
    args = parser.parse_args(argv)
    nonnegative = {'initial_hold', 'final_hold'}
    for key, value in vars(args).items():
        if isinstance(value, float):
            if not math.isfinite(value) or value < 0 or (value == 0 and key not in nonnegative):
                constraint = 'nonnegative' if key in nonnegative else 'positive'
                parser.error(f'--{key.replace("_", "-")} must be finite and {constraint}.')
    if args.cycles < 1:
        parser.error('--cycles must be a positive integer.')
    if args.velocity_scale > 1:
        parser.error('--velocity-scale must not exceed 1.')
    if args.max_lateness <= 1 / args.rate:
        parser.error('--max-lateness must exceed one command period.')
    if not args.urdf.is_file():
        parser.error(f'URDF/Xacro not found: {args.urdf}')
    if args.initial_joints_file is not None:
        if not args.dry_run:
            parser.error('--initial-joints-file is only allowed with --dry-run; execution uses live feedback.')
        if not args.initial_joints_file.is_file():
            parser.error(f'Initial joint snapshot not found: {args.initial_joints_file}')
    return args


def main(argv=None):
    """Plan, optionally execute, and always write a final status report."""
    args = parse_args(argv)
    output = args.output_dir / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    report = {'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              'pinocchio_version': pin.__version__, 'status': 'planning',
              'geometry_scope': 'Specified URDF only: arms and 0.30 m table; no external TCP measurement.'}
    records, robot = [], None
    exit_code = 0
    try:
        print(f'Loading {args.urdf}', flush=True)
        robot = RobotModel(args.urdf)
        (output / 'expanded.urdf').write_text(robot.xml)
        report['initial_state'] = capture_initial_configuration(robot, args)
        snapshot = {a.namespace: robot.initial_q[a.q_indices].tolist() for a in robot.arms}
        report['initial_state']['joints_rad'] = snapshot
        (output / 'initial_joints.json').write_text(json.dumps(snapshot, indent=2) + '\n')
        for arm in robot.arms:
            angles = np.degrees(robot.initial_q[arm.q_indices]).round(4).tolist()
            print(f'{arm.namespace} ({arm.prefix}): captured J1-J6 [deg] {angles}; '
                  f'initial EEF world {arm.initial_position.tolist()}', flush=True)
        plan = CirclePlan(robot, args)
        report['plan'] = plan.summary
        offline = []
        for t in np.linspace(0, plan.duration, max(2, math.ceil(plan.duration * 100) + 1)):
            q, target = plan.sample(t)
            offline.append({'time': float(t), 'circle_time': float(t), 'stage': 'circle',
                            'command': q, 'target': target})
        save_records(plan, offline, output, 'planned', args.show_plot and args.dry_run)
        print(f'Plan validated. Results: {output.resolve()}', flush=True)
        report['status'] = 'dry_run_passed' if args.dry_run else 'ready_to_execute'
        # Persist the concrete plan before operator input or any hardware command.
        (output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        if not args.dry_run:
            execute(plan, args, records)
            report['status'] = 'completed'
    except (KeyboardInterrupt, EOFError):
        report['status'], report['error'], exit_code = 'interrupted', 'Operator interrupted execution.', 130
    except Exception as exc:
        report['status'], report['error'], exit_code = 'failed', str(exc), 1
        print(f'Aborted: {exc}', file=sys.stderr)
    finally:
        if robot is not None and records:
            try:
                report['execution'] = save_records(plan, records, output, 'executed', args.show_plot)
            except Exception as exc:
                report['recording_error'] = str(exc)
                exit_code = exit_code or 1
        (output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(f'Status: {report["status"]}. Report: {output.resolve() / "summary.json"}', flush=True)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
