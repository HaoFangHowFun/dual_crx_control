"""A world-frame circle through the starting TCP, without an approach jump."""

import math

import numpy as np


class CircularTrajectory:
    def __init__(self, radius=0.02, period=8., plane='xy', direction='ccw'):
        if not np.all(np.isfinite([radius, period])) or radius <= 0 or period <= 0:
            raise ValueError('Circle radius and period must be finite and positive')
        if plane not in ('xy', 'xz', 'yz') or direction not in ('cw', 'ccw'):
            raise ValueError('plane must be xy/xz/yz and direction must be cw/ccw')
        self.radius, self.period, self.plane, self.direction = radius, period, plane, direction
        self.axes = tuple('xyz'.index(axis) for axis in plane)

    def phase(self, elapsed, cycles=1):
        """Unwrapped laps: one startup ramp, cruise, and one final slowdown."""
        if not np.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Circle elapsed time must be finite and nonnegative')
        ramp = self.period / 4.
        duration = cycles * self.period
        # Integral of smootherstep velocity, with integral(1) = 1/2.
        def integrated_ramp(u):
            return u**4 * (2.5 + u * (-3. + u))

        if cycles:
            if elapsed >= duration:
                return float(cycles)
            # Preserve the requested total duration despite the endpoint ramps.
            cruise_rate = cycles / (duration - ramp)
            if elapsed > duration - ramp:
                return cycles - cruise_rate * ramp * integrated_ramp((duration - elapsed) / ramp)
        else:
            cruise_rate = 1. / self.period
        if elapsed < ramp:
            return cruise_rate * ramp * integrated_ramp(elapsed / ramp)
        return cruise_rate * (elapsed - ramp / 2.)

    def offset(self, elapsed, cycles=1):
        phase = self.phase(elapsed, cycles)
        if cycles and phase >= cycles:
            return np.zeros(3)
        theta = (1. if self.direction == 'ccw' else -1.) * 2. * math.pi * phase
        offset = np.zeros(3)
        offset[self.axes[0]] = self.radius * (math.cos(theta) - 1.)
        offset[self.axes[1]] = self.radius * math.sin(theta)
        return offset
