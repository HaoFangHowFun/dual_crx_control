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
