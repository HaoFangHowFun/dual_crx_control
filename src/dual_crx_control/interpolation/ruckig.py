"""Time-synchronized Ruckig reference state for one or both arms."""
import math

import numpy as np

from dual_crx_control._ruckig import Generator
from dual_crx_control.joint_config import SIDES

# CRX-5iA model: fanuc_crx_description/urdf/crx5ia_urdf_macro.xacro.
MAX_VELOCITY = tuple(math.radians(v) for v in (150, 150, 180, 225, 225, 225))
# Experimental planning values, NOT verified CRX-5iA hardware acceleration limits.
# Starting point: fanuc_moveit_config/config/joint_limits.yaml (CRX-10iA).
MAX_ACCELERATION = (3.0, 3.0, 4.5, 4.5, 4.5, 4.5)
# Experimental 0.1 s acceleration ramp: jerk = acceleration / 0.1.
MAX_JERK = (30.0, 30.0, 45, 45.0, 45.0, 45.0)


class RuckigInterpolation:
    def __init__(self, dt):
        self.generator = Generator(dt, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
        self.active = set()

    def target(self, commands, positions, velocities):
        for side, q in commands.items():
            self.generator.target(SIDES.index(side), q, positions[side], velocities[side])
            self.active.add(side)

    def step(self):
        positions, _, _ = self.generator.step()
        q = np.asarray(positions).reshape(2, 6)
        return {s: q[i].copy() for i, s in enumerate(SIDES) if s in self.active}
