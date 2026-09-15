"""Custom six-dimensional damped least-squares inverse kinematics."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def pose_error(target, current):
    """Position and shortest rotation-vector error, both in world axes."""
    return np.concatenate((target[:3, 3] - current[:3, 3],
                           Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()))


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    iterations: int
    position_error: float
    orientation_error: float
    reason: str


class DampedLeastSquaresIK:
    def __init__(self, kinematics, *, damping=0.01, position_tolerance=1e-5,
                 orientation_tolerance=1e-5, max_iterations=100,
                 max_joint_step=0.05, alpha=1.0):
        values = [damping, position_tolerance, orientation_tolerance, max_joint_step, alpha]
        if not np.all(np.isfinite(values)) or min(values) <= 0 or alpha > 1:
            raise ValueError('IK parameters must be finite and positive; alpha <= 1')
        if not isinstance(max_iterations, int) or isinstance(max_iterations, bool) or max_iterations < 1:
            raise ValueError('max_iterations must be a positive integer')
        self.kinematics = kinematics
        self.damping = damping
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.max_iterations = max_iterations
        self.max_joint_step = max_joint_step
        self.alpha = alpha

    def solve(self, target, seed):
        q = np.asarray(seed, dtype=float).copy()
        target = np.asarray(target, dtype=float)
        if not self.kinematics.valid_joints(q):
            return IKResult(False, q, 0, np.inf, np.inf, 'invalid seed / joint-limit violation')
        if (target.shape != (4, 4) or not np.all(np.isfinite(target))
                or not np.allclose(target[3], [0, 0, 0, 1])
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(target[:3, :3]), 1, atol=1e-7)):
            return IKResult(False, q, 0, np.inf, np.inf, 'invalid target pose')
        reason = 'maximum iterations reached'
        for iteration in range(self.max_iterations + 1):
            error = pose_error(target, self.kinematics.fk(q))
            pos, rot = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
            if not np.all(np.isfinite(error)):
                reason = 'nonfinite pose error'
                break
            if pos <= self.position_tolerance and rot <= self.orientation_tolerance:
                return IKResult(True, q, iteration, pos, rot, 'converged')
            if iteration == self.max_iterations:
                break
            jac = self.kinematics.jacobian(q)
            try:
                step = self.alpha * jac.T @ np.linalg.solve(
                    jac @ jac.T + self.damping**2 * np.eye(6), error)
            except np.linalg.LinAlgError:
                reason = 'DLS linear solve failed'
                break
            if not np.all(np.isfinite(step)):
                reason = 'nonfinite joint step'
                break
            step *= min(1., self.max_joint_step / max(np.max(np.abs(step)), 1e-15))
            # Shorten the entire step to stay within limits, preserving direction.
            scale = 1.
            for j, delta in enumerate(step):
                if delta > 0:
                    scale = min(scale, (self.kinematics.upper[j] - q[j]) / delta)
                elif delta < 0:
                    scale = min(scale, (self.kinematics.lower[j] - q[j]) / delta)
            if scale < 1e-10 or np.max(np.abs(step)) < 1e-12:
                reason = 'joint-limit boundary or stalled iteration'
                break
            q += step * (scale * 0.999999 if scale < 1 else 1.)
        return IKResult(False, q, iteration, float(pos), float(rot), reason)
