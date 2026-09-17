"""Regression checks for motion modes, CLI mapping and bounded plotting.

Run with colcon test, or after sourcing ROS and the built workspace:
    python3 -m pytest tools/test_motion_refactor.py
"""

import csv
import math
from types import SimpleNamespace

import numpy as np
import pytest

from dual_crx_control.motion.joint_sine import JointSine, parse_args
from dual_crx_control.motion.cli import parse_parameters
from dual_crx_control.analysis.motion_recording import JointRecording


def test_single_and_dual_defaults():
    single = parse_args(['--arms', 'right'])
    dual = parse_args([])
    assert (single.return_mode, single.final_hold) == ('direct', 2.)
    assert (dual.return_mode, dual.return_duration, dual.final_hold) == ('smooth', 1., .2)
    assert single.amplitude_deg == dual.amplitude_deg == 20.
    assert single.duration == dual.duration == 10.


@pytest.mark.parametrize('args', [
    ['--arms', 'left', 'left'], ['--duration', '1', '--continuous'],
    ['--rate', '501'], ['--rate', 'nan'], ['--amplitude-deg', '-1'],
    ['--return-duration', '0'], ['--state-timeout', 'nan'],
])
def test_invalid_joint_options(args):
    with pytest.raises(SystemExit):
        parse_args(args)


def test_sine_and_smooth_return_match_previous_formula():
    args = parse_args(['--duration', '1.3'])
    motion = SimpleNamespace(args=args, return_offset=0.)
    assert JointSine.offset(motion, .5) == 0.
    # Legacy quintic amplitude envelope and same-phase sine target.
    t = .5
    blend = t**3 * (10. + t * (-15. + 6. * t))
    expected = blend * math.radians(20.) * math.sin(2 * math.pi * t / 4.)
    assert JointSine.offset(motion, args.initial_hold + t) == pytest.approx(expected)
    last = JointSine.offset(motion, args.initial_hold + 1.28)
    assert JointSine.offset(motion, args.initial_hold + args.duration + .5) == pytest.approx(last * .5)
    assert JointSine.offset(motion, args.initial_hold + args.duration + 1.) == pytest.approx(0., abs=1e-14)


def test_direct_return_and_continuous_mode():
    motion = SimpleNamespace(args=parse_args(['--arms', 'left']), return_offset=1.)
    assert JointSine.offset(motion, 11.) == 0.
    motion.args = parse_args(['--continuous'])
    assert JointSine.offset(motion, 102.) == pytest.approx(math.radians(20.))


def test_cli_maps_units_and_leaves_defaults_to_controller():
    assert parse_parameters('cartesian_circle', []) == []
    values = {p.name: p.value for p in parse_parameters('cartesian_circle', [
        '--radius-m', '.03', '--rate', '50', '--cycles', '2', '--no-move-to-initial',
        '--center-midpoint', '.55', '-.38', '.35'])}
    assert values == dict(radius=.03, rate=50., cycles=2, move_to_initial=False,
                          center_midpoint=[.55, -.38, .35])


def test_single_arm_csv_is_complete_and_plots_are_bounded(tmp_path, monkeypatch):
    from matplotlib.axes import Axes
    original_plot = Axes.plot
    lengths = []

    def plot(self, x, *args, **kwargs):
        lengths.append(len(x))
        return original_plot(self, x, *args, **kwargs)

    monkeypatch.setattr(Axes, 'plot', plot)
    recorder = JointRecording(tmp_path, target_source='target', arms=('right',))
    for i in range(21001):
        recorder.add(recorder.started_at + i * .002, 'right', 'target', np.full(6, i * 1e-6))
    paths = recorder.save()
    recorder.flush()
    recorder.close()
    assert recorder.save() == paths
    assert [p.name for p in paths] == ['joints.csv', 'right_joints.png']
    assert max(lengths) <= 10001
    with recorder.csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 21001
    assert float(rows[-1]['J1_rad']) == pytest.approx(.021)
