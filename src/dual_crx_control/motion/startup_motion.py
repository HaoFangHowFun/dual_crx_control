"""Synchronized, bounded joint approach before Cartesian motion."""

import math

import numpy as np


from dual_crx_control.robot.joint_config import INITIAL_JOINTS_DEG


class InitialJointMove:
    """Quintic joint interpolation with one duration shared by both arms.

    This limits joint speed/acceleration; it does not plan around obstacles.
    """

    def __init__(self, models, starts, targets, minimum_duration, max_velocity, max_acceleration):
        values = [minimum_duration, max_velocity, max_acceleration]
        if not np.all(np.isfinite(values)) or min(values) <= 0:
            raise ValueError('Initial-move duration, velocity, and acceleration must be positive')
        self.starts = {s: np.array(q, dtype=float).copy() for s, q in starts.items()}
        self.targets = {s: np.array(q, dtype=float).copy() for s, q in targets.items()}
        self.duration = minimum_duration
        for side, model in models.items():
            if not model.valid_joints(self.starts[side]) or not model.valid_joints(self.targets[side]):
                raise ValueError(f'{side}: invalid initial-move start or target / joint-limit violation')
            delta = np.abs(self.targets[side] - self.starts[side])
            # Exact maxima of quintic smootherstep derivatives: 15/8 and 10/sqrt(3).
            speed_time = np.max(1.875 * delta / np.minimum(model.velocity, max_velocity))
            acceleration_time = math.sqrt(float(np.max((10. / math.sqrt(3.)) * delta / max_acceleration)))
            self.duration = max(self.duration, float(speed_time) * 1.05, acceleration_time * 1.05)

    def sample(self, elapsed):
        if not np.isfinite(elapsed):
            raise ValueError('Initial-move time must be finite')
        u = float(np.clip(elapsed / self.duration, 0., 1.))
        if u >= 1.:
            return {s: q.copy() for s, q in self.targets.items()}
        blend = u**3 * (10. + u * (-15. + 6. * u))
        return {s: q + blend * (self.targets[s] - q) for s, q in self.starts.items()}
