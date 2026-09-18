#!/usr/bin/env python3
"""Offline, URDF-only search for well-conditioned facing circles. No ROS nodes.

From the repository root, after building and sourcing ROS and the workspace:
    python3 tools/search_facing_circle.py --help
    python3 tools/search_facing_circle.py --gap 0.2 --output-dir test_results/facing_circle

Uses the installed dual_crx_control URDF and left_/right_ joint prefixes.
Writes search_report.json and facing_circle_mock.yaml. Gap is in metres.
This development tool is not installed as a ros2 run executable. It sends no
motion commands; its kinematic checks do not include tool/link collisions.
"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from dual_crx_control.motion.circular_trajectory import CircularTrajectory
from dual_crx_control.motion.facing_circle import (facing_start_poses, solve_facing_start,
                                           check_joint_approach, scaled_min_singular_value)
from dual_crx_control.robot.kinematics import CRXKinematics
from dual_crx_control.robot.ik_solver import DampedLeastSquaresIK
from dual_crx_control.motion.startup_motion import INITIAL_JOINTS_DEG


def evaluate(models, solvers, home, midpoint, gap, fine=False):
    poses = facing_start_poses(midpoint, gap, .1, 'xz')
    starts = solve_facing_start(models, solvers, home, poses)
    approach_sigma = check_joint_approach(models, home, starts, .1)
    seeds = {s: q.copy() for s, q in starts.items()}
    curve = CircularTrajectory(.1, 4., 'xz', 'cw')
    sigma = dict.fromkeys(models, float('inf'))
    max_velocity = dict.fromkeys(models, 0.)
    min_limit_margin = float('inf')
    times = np.linspace(0., 40., 2001) if fine else np.linspace(0., 4., 101)
    for t in times:
        offset = curve.offset(t, 10 if fine else 1)
        for side, model in models.items():
            pose = poses[side].copy()
            pose[:3, 3] += offset
            result = solvers[side].solve(pose, seeds[side])
            if not result.success:
                raise ValueError(f'{side}: circle IK failed at {t:.3f} s')
            velocity = float(np.max(np.abs(result.q - seeds[side]))) / (times[1] - times[0])
            if velocity > 2.:
                raise ValueError(f'{side}: excessive joint velocity')
            value = scaled_min_singular_value(model, result.q)
            if value < .1:
                raise ValueError(f'{side}: circle too near a singularity')
            sigma[side] = min(sigma[side], value)
            max_velocity[side] = max(max_velocity[side], velocity)
            min_limit_margin = min(min_limit_margin, float(np.min(result.q - model.lower)),
                                   float(np.min(model.upper - result.q)))
            seeds[side] = result.q
    return {'center_midpoint_m': list(midpoint), 'tcp_gap_m': gap,
            'minimum_scaled_sigma': sigma, 'approach_minimum_scaled_sigma': approach_sigma,
            'max_joint_velocity_rad_s': max_velocity, 'minimum_joint_limit_margin_rad': min_limit_margin,
            'start_joints_rad': {s: q.tolist() for s, q in starts.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gap', type=float, default=.2)
    parser.add_argument('--output-dir', type=Path, default=Path('test_results/facing_circle'))
    args = parser.parse_args()
    share = Path(get_package_share_directory('dual_crx_control'))
    xml = xacro.process_file(str(share / 'urdf/dual_crx.urdf.xacro')).toxml()
    models = {s: CRXKinematics(xml, s + '_tcp') for s in INITIAL_JOINTS_DEG}
    solvers = {s: DampedLeastSquaresIK(m, max_iterations=150) for s, m in models.items()}
    home = {s: np.radians(q) for s, q in INITIAL_JOINTS_DEG.items()}
    # Same center placement as the original circle: current start minus radius X.
    baseline = np.mean([models[s].fk(home[s])[:3, 3] for s in models], axis=0) - [.1, 0, 0]
    results, rejected = [], []
    for midpoint in itertools.product([.25, .35, .45, .55], [-.30, -.34, -.38], [.30, .40, .50, .60]):
        try:
            results.append(evaluate(models, solvers, home, midpoint, args.gap))
        except ValueError as exc:
            rejected.append({'midpoint': midpoint, 'reason': str(exc)})
    results.sort(key=lambda r: min(r['minimum_scaled_sigma'].values()), reverse=True)
    if not results:
        raise RuntimeError('No acceptable center found in the sampled grid')
    selected = evaluate(models, solvers, home, results[0]['center_midpoint_m'], args.gap, fine=True)
    try:
        reference = evaluate(models, solvers, home, baseline, args.gap, fine=True)
    except ValueError as exc:
        reference = {'center_midpoint_m': baseline.tolist(), 'rejected': str(exc)}
    report = {'selected': selected, 'baseline': reference, 'accepted_candidates': results,
              'rejected_candidates': rejected, 'jacobian_linear_scale_m': .5,
              'validation_dt_s': .02, 'validated_duration_s': 40.,
              'assumptions': 'local TCP +X points out of tool; local +Z world-up',
              'collision_checked': False, 'scope': 'mock only; sampled conditioning, not a global optimum'}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'search_report.json').write_text(json.dumps(report, indent=2) + '\n')
    params = {'radius': .1, 'plane': 'xz', 'direction': 'cw', 'period': 4., 'cycles': 10,
              'max_velocity': 2., 'rate': 50., 'face_each_other': True,
              'tcp_gap': args.gap, 'center_midpoint': selected['center_midpoint_m'],
              'minimum_scaled_sigma': .1}
    (args.output_dir / 'facing_circle_mock.yaml').write_text(yaml.safe_dump(
        {'cartesian_circle': {'ros__parameters': params}}, sort_keys=False))
    print(json.dumps({'selected': selected, 'baseline': reference}, indent=2))


if __name__ == '__main__':
    main()
