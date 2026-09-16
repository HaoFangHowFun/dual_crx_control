"""Facing-pose geometry and conditioned full-circle IK from the mock home."""

from pathlib import Path

import numpy as np
import pytest
import xacro

from motion.facing_circle import (facing_start_poses, solve_facing_start,
                                           check_joint_approach, scaled_min_singular_value)
from dual_crx_control.kinematics import CRXKinematics
from dual_crx_control.ik_solver import DampedLeastSquaresIK, pose_error
from dual_crx_control.circular_trajectory import CircularTrajectory
from motion.startup_motion import INITIAL_JOINTS_DEG

ROOT = Path(__file__).resolve().parents[1]


def test_facing_geometry_and_constant_gap():
    curve = CircularTrajectory(.1, 4., 'xz', 'cw')
    poses = facing_start_poses([.55, -.38, .30], .2, .1, 'xz')
    for t in np.linspace(0, 40, 1001):
        points = {s: pose[:3, 3] + curve.offset(t, 10) for s, pose in poses.items()}
        displacement = points['right'] - points['left']
        assert np.isclose(np.linalg.norm(displacement), .2)
        np.testing.assert_allclose(poses['left'][:3, 0], displacement / .2, atol=1e-12)
        np.testing.assert_allclose(poses['right'][:3, 0], -displacement / .2, atol=1e-12)
        for pose in poses.values():
            assert np.isclose(np.linalg.det(pose[:3, :3]), 1.)
            np.testing.assert_array_equal(pose[:3, 2], [0, 0, 1])


@pytest.mark.parametrize('midpoint,gap', [([0, 0], .2), ([0, np.nan, 0], .2),
                                         ([0, 0, 0], 0), ([0, 0, 0], np.inf)])
def test_bad_facing_geometry(midpoint, gap):
    with pytest.raises(ValueError):
        facing_start_poses(midpoint, gap, .1, 'xz')


def test_selected_center_approach_and_ten_lap_path():
    xml = xacro.process_file(str(ROOT / 'urdf/dual_crx.urdf.xacro')).toxml()
    models = {s: CRXKinematics(xml, s + '_tcp') for s in INITIAL_JOINTS_DEG}
    solvers = {s: DampedLeastSquaresIK(m) for s, m in models.items()}
    home = {s: np.radians(q) for s, q in INITIAL_JOINTS_DEG.items()}
    poses = facing_start_poses([.55, -.38, .30], .2, .1, 'xz')
    starts = solve_facing_start(models, solvers, home, poses)
    assert check_joint_approach(models, home, starts, .1) > .35
    curve = CircularTrajectory(.1, 4., 'xz', 'cw')
    seeds = {s: q.copy() for s, q in starts.items()}
    for t in np.linspace(0., 40., 2001):
        for side, model in models.items():
            pose = poses[side].copy()
            pose[:3, 3] += curve.offset(t, 10)
            result = solvers[side].solve(pose, seeds[side])
            assert result.success
            assert model.valid_joints(result.q)
            assert np.max(np.abs(result.q - seeds[side])) / .02 < 2.
            assert scaled_min_singular_value(model, result.q) > .35
            error = pose_error(pose, model.fk(result.q))
            assert np.linalg.norm(error[:3]) < 1e-5
            assert np.linalg.norm(error[3:]) < 1e-5
            seeds[side] = result.q
