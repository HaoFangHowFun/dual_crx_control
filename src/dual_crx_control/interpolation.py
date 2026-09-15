"""ROS-independent linear interpolation of finite, matching NumPy arrays."""

import numpy as np


def linear_interpolate(q0, q1, phase):
    """Return a new array between q0 and q1, clamping finite phase to [0, 1]."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    if q0.shape != q1.shape:
        raise ValueError('q0 and q1 must have the same shape')
    if not np.all(np.isfinite(q0)) or not np.all(np.isfinite(q1)):
        raise ValueError('Joint values must be finite')
    phase = float(phase)
    if not np.isfinite(phase):
        raise ValueError('phase must be finite')
    phase = float(np.clip(phase, 0., 1.))
    return np.array((1. - phase) * q0 + phase * q1, copy=True)
