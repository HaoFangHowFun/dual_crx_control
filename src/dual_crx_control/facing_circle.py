"""Facing TCP poses and numerical conditioning checks for mock circle placement."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def scaled_min_singular_value(model, q, characteristic_length=0.5):
    """Dimensionless Jacobian metric: linear rows divided by a 0.5 m scale.

    Higher is better conditioned. This is not a collision or hardware safety test.
    """
    jacobian = model.jacobian(q).copy()
    jacobian[:3] /= characteristic_length
    return float(np.linalg.svd(jacobian, compute_uv=False)[-1])


def facing_start_poses(midpoint, gap, radius, plane):
    midpoint = np.asarray(midpoint, dtype=float)
    if midpoint.shape != (3,) or not np.all(np.isfinite(midpoint)):
        raise ValueError('center_midpoint must contain three finite world coordinates')
    if not np.isfinite(gap) or gap <= 0 or not np.isfinite(radius) or radius <= 0:
        raise ValueError('tcp_gap and radius must be finite and positive')
    if plane not in ('xy', 'xz', 'yz'):
        raise ValueError('Circle plane must be xy/xz/yz')
    poses = {}
    for side, sign in [('left', 1.), ('right', -1.)]:
        pose = np.eye(4)
        # Local +X points toward the other TCP; local +Z points world-up.
        forward = np.array([0., -sign, 0.])
        up = np.array([0., 0., 1.])
        pose[:3, :3] = np.column_stack((forward, np.cross(up, forward), up))
        pose[:3, 3] = midpoint + [0., sign * gap / 2., 0.]
        pose['xyz'.index(plane[0]), 3] += radius
        poses[side] = pose
    return poses


def solve_facing_start(models, solvers, starts, poses):
    """Find the facing IK branch through small pose/rotation increments."""
    targets = {}
    for side, model in models.items():
        q = np.asarray(starts[side], dtype=float).copy()
        initial = model.fk(q)
        rotate = Slerp([0., 1.], Rotation.from_matrix([initial[:3, :3], poses[side][:3, :3]]))
        for fraction in np.linspace(0., 1., 51):
            pose = np.eye(4)
            pose[:3, :3] = rotate(fraction).as_matrix()
            pose[:3, 3] = (1. - fraction) * initial[:3, 3] + fraction * poses[side][:3, 3]
            result = solvers[side].solve(pose, q)
            if not result.success:
                raise ValueError(f'{side}: facing-pose IK failed at {fraction:.2f}: {result.reason}')
            q = result.q
        targets[side] = q
    return targets


def check_joint_approach(models, starts, targets, minimum_sigma):
    """Sample the actual joint-interpolated approach, including its endpoints."""
    worst = float('inf')
    samples = max(2, int(np.ceil(max(np.max(np.abs(targets[s] - starts[s]))
                                    for s in models) / .01)) + 1)
    for fraction in np.linspace(0., 1., samples):
        for side, model in models.items():
            q = starts[side] + fraction * (targets[side] - starts[side])
            if not model.valid_joints(q):
                raise ValueError(f'{side}: invalid approach joints')
            value = scaled_min_singular_value(model, q)
            worst = min(worst, value)
            if value < minimum_sigma:
                raise ValueError(f'{side}: approach scaled sigma {value:.6g} < {minimum_sigma}')
    return worst
