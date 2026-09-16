#!/usr/bin/env python3
"""Run synchronized inward-facing circles at the mock-validated world centers.

Local TCP +X faces inward, +Z stays world-up. Placement was checked for
kinematic conditioning in mock; physical tool/link collisions were not checked.
All defaults below remain overridable through standard ROS parameters.
"""

from motion.circle_controller import DualCircleController
from motion.cartesian_controller import main as run_controller


class DualFacingCircleController(DualCircleController):
    def __init__(self, **kwargs):
        super().__init__(
            node_name='dual_test_7_cartesian_facing_circle_motion',
            motion_defaults={'period': 3., 'cycles': 10, 'max_velocity': 3.,
                             'rate': 50.},
            circle_defaults={'radius': 0.1, 'plane': 'xz', 'direction': 'cw',
                             'face_each_other': True, 'tcp_gap': 0.02,
                             'center_midpoint': [0.55, -0.38, 0.35],
                             'minimum_scaled_sigma': 0.1},
            **kwargs)


def main():
    run_controller(DualFacingCircleController)


if __name__ == '__main__':
    main()
