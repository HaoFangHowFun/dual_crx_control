"""ROS-independent joint trajectory interpolation."""

import numpy as np
from scipy.interpolate import CubicSpline


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


OUTPUT_RATE_HZ = 500.0


def validate_rate(rate):
    if not np.isfinite(rate) or not 0 < rate <= OUTPUT_RATE_HZ:
        raise ValueError('input_rate_hz must be finite and in (0, 500]')
    return float(rate)


class JointSegment:
    """Joint-only segment with finite data and increasing local sample times."""

    def __init__(self, start_time, end_time, start, target, *, method='linear', history=()):
        self.start_time, self.end_time = start_time, end_time
        self.start, self.target = np.asarray(start, float), np.asarray(target, float)
        linear_interpolate(self.start, self.target, 0.)
        if not np.isfinite([start_time, end_time]).all() or end_time <= start_time:
            raise ValueError('segment times must be finite and increasing')
        if method not in ('linear', 'cubic'):
            raise ValueError('method must be linear or cubic')
        self.spline = None
        if method == 'cubic' and len(history) >= 2:
            points = [(t, q) for t, q in history if t < start_time][-5:]
            points += [(start_time, self.start), (end_time, self.target)]
            times = np.array([t for t, _ in points])
            if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
                raise ValueError('history times must be finite and strictly increasing')
            self.spline = CubicSpline(times - start_time, np.array([q for _, q in points]),
                                      bc_type='natural', axis=0, extrapolate=False)

    def sample(self, now):
        if not np.isfinite(now):
            raise ValueError('sample time must be finite')
        if now >= self.end_time:
            return self.target.copy()
        if now <= self.start_time:
            return self.start.copy()
        if self.spline is not None:
            return self.spline(now - self.start_time)
        return linear_interpolate(self.start, self.target,
                                  (now - self.start_time) / (self.end_time - self.start_time))
