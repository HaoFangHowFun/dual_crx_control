"""Offline delay estimates on independently timestamped signals (positive = response lags)."""

import numpy as np


def statistics(values):
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):
        return None
    return dict(count=len(x), mean=float(x.mean()), median=float(np.median(x)),
                min=float(x.min()), max=float(x.max()), std=float(x.std()))


def clean_series(t, x):
    t, x = np.asarray(t, dtype=float), np.asarray(x, dtype=float)
    good = np.isfinite(t) & np.isfinite(x)
    t, x = t[good], x[good]
    order = np.argsort(t, kind='stable')
    t, x = t[order], x[order]
    # Keep the last value for duplicate timestamps.
    keep = np.r_[np.diff(t) > 0, True] if len(t) else np.array([], dtype=bool)
    return t[keep], x[keep]


def estimate_delay(command_t, command, feedback_t, feedback, *, max_lag=.5,
                   step=.001, trim_start=1.5, trim_end=1., minimum_span=1e-4):
    """Normalized cross-correlation on a fixed overlap and lag grid.

    Compare command(t) with feedback(t + lag). Fit gain/offset at the best lag.
    A periodic response yields an apparent phase delay, not a unique transport delay.
    Large gaps are rejected rather than silently interpolated into evidence.
    """
    if max_lag <= 0 or step <= 0 or trim_start < 0 or trim_end < 0:
        raise ValueError('lag/step must be positive and trims nonnegative')
    ct, cq = clean_series(command_t, command)
    ft, fq = clean_series(feedback_t, feedback)
    if min(len(ct), len(ft)) < 20:
        raise ValueError('insufficient samples')
    lo = max(ct[0], ft[0]) + trim_start + max_lag
    hi = min(ct[-1], ft[-1]) - trim_end - max_lag
    if hi - lo < .5:
        raise ValueError('insufficient common duration after trimming')
    for t in (ct, ft):
        intervals = np.diff(t)
        overlap = (t[:-1] <= hi + max_lag) & (t[1:] >= lo - max_lag)
        if np.any(intervals[overlap] > max(.05, 10 * np.median(intervals))):
            raise ValueError('large sampling gap in analysis window; split the recording')
    grid = np.arange(lo, hi, step)
    x = np.interp(grid, ct, cq)
    if np.ptp(x) < minimum_span:
        raise ValueError('command is stationary or too small for reliable delay estimation')
    xc = x - x.mean()
    lags = np.linspace(-max_lag, max_lag, int(np.ceil(2 * max_lag / step)) + 1)
    scores = []
    for lag in lags:
        y = np.interp(grid + lag, ft, fq)
        yc = y - y.mean()
        denominator = np.linalg.norm(xc) * np.linalg.norm(yc)
        scores.append(float(xc @ yc / denominator) if denominator > 1e-18 else -1.)
    index = int(np.argmax(scores))
    lag = lags[index]
    y = np.interp(grid + lag, ft, fq)
    gain, bias = np.linalg.lstsq(np.column_stack((x, np.ones(len(x)))), y, rcond=None)[0]
    warnings = []
    if index in (0, len(lags) - 1):
        warnings.append('peak at search boundary; expand lag range or inspect waveform')
    if scores[index] < .98:
        warnings.append('weak waveform agreement; do not interpret as pure time delay')
    return dict(delay_ms=float(lag * 1000), correlation=scores[index], gain=float(gain),
                bias=float(bias), residual_rms=float(np.sqrt(np.mean((y - gain*x - bias)**2))),
                command_span=float(np.ptp(x)), resolution_ms=step*1000,
                window_start_s=float(lo), window_end_s=float(hi), warnings=warnings,
                command_interval_ms=statistics(np.diff(ct)*1000),
                feedback_interval_ms=statistics(np.diff(ft)*1000))
