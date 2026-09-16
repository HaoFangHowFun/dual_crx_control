"""Minimal linear interpolation contract tests."""

import numpy as np
import pytest

from dual_crx_control.interpolation import linear_interpolate


@pytest.mark.parametrize('phase,expected', [(0., 0.), (1., 1.), (.5, .5), (-2., 0.), (2., 1.)])
def test_six_joint_vectors(phase, expected):
    q0, q1 = np.arange(6.), np.arange(6.) + 2
    result = linear_interpolate(q0, q1, phase)
    np.testing.assert_array_equal(result, q0 + 2 * expected)
    assert not np.shares_memory(result, q0)
    assert not np.shares_memory(result, q1)


def test_shape_mismatch():
    with pytest.raises(ValueError, match='same shape'):
        linear_interpolate(np.zeros(6), np.zeros((2, 6)), .5)


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_nonfinite_rejected(bad):
    for q0, q1, phase in [([bad] * 6, [0.] * 6, .5),
                          ([0.] * 6, [bad] * 6, .5), ([0.] * 6, [1.] * 6, bad)]:
        with pytest.raises(ValueError, match='finite'):
            linear_interpolate(q0, q1, phase)


from dual_crx_control.interpolation import JointSegment, validate_rate


@pytest.mark.parametrize('rate', [0, -1, 501, np.nan, np.inf])
def test_bad_input_rate(rate):
    with pytest.raises(ValueError):
        validate_rate(rate)


@pytest.mark.parametrize('method', ['linear', 'cubic'])
def test_segment_endpoints_and_no_extrapolation(method):
    segment = JointSegment(1., 2., np.zeros(12), np.ones(12), method=method,
                           history=[(.8, np.full(12, -.2)), (.9, np.full(12, -.1))])
    np.testing.assert_array_equal(segment.sample(-10), np.zeros(12))
    np.testing.assert_allclose(segment.sample(1.5), np.full(12, .5))
    np.testing.assert_array_equal(segment.sample(20), np.ones(12))
    if method == 'cubic':
        assert segment.spline is not None


def test_cubic_has_no_motion_limits_or_fallback():
    segment = JointSegment(0., 1., [0.], [0.], method='cubic',
                           history=[(-.2, [-.8]), (-.1, [-.4])])
    assert segment.spline is not None
    assert max(segment.sample(t)[0] for t in np.linspace(0, 1, 101)) > .01
    fast = JointSegment(0., .002, [0.], [100.])
    np.testing.assert_array_equal(fast.sample(.002), [100.])


def test_duplicate_history_rejected():
    with pytest.raises(ValueError, match='increasing'):
        JointSegment(1., 2., [0.], [1.], method='cubic',
                     history=[(.5, [0.]), (.5, [0.])])
