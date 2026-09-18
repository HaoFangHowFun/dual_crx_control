"""Offline wrench recording, plotting and lifecycle regression tests."""
import csv
import json
import queue
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dual_crx_control.analysis.wrench_recording import Recording, CsvWriter, LivePlot, parse_args


@pytest.fixture
def recording(tmp_path):
    args, _ = parse_args(['--output-dir', str(tmp_path), '--no-plot', '--plot-capacity', '3', '--record-rate-hz', '1e9'])
    rec = Recording(args)
    yield rec
    rec.close({})


def test_pairing_and_csv_preserves_stamps(recording):
    t = recording.started
    pose = (t, 123., [1., 2., 3., 0., 0., 0., 1.])
    fresh = recording.add('left', [1, 2, 3, 4, 5, 6], 'left_fanuc_flange', 120., pose, t+.05)
    stale = recording.add('right', [6, 5, 4, 3, 2, 1], '', 0., pose, t+.2)
    missing = recording.add('left', [0]*6, '', 0., None, t+.3)
    assert fresh[5] and fresh[10:13] == [1, 2, 3]
    assert fresh[2:4] == [120., 123.]  # raw stamp ordering is not changed
    assert not stale[5] and stale[10:17] == ['']*7
    assert missing[3:5] == ['', '']
    recording.close({})
    with (recording.output / 'wrench.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3
    assert float(rows[0]['tz_Nm']) == 6
    metadata = json.loads((recording.output / 'metadata.json').read_text())
    assert metadata['written'] == 3 and metadata['counts']['right.zero_stamp'] == 1


def test_bounded_plot_does_not_truncate_csv(recording):
    for i in range(25):
        recording.add('left', [i]*6, 'frame', i, None)
    assert len(recording.buffers['left']) == 3
    recording.close({})
    assert recording.writer.written == 25


def test_plot_gaps_frames_and_invalid_values(recording):
    t = recording.started
    recording.add('left', [1]*6, 'a', 0., None, t)
    recording.add('left', [2]*6, 'a', 0., None, t+.6)
    recording.add('left', [np.nan]*6, 'b', 0., None, t+.7)
    plot = LivePlot(recording, live=False)
    try:
        plot.update()
        assert plot.axes.shape == (6, 2)
        assert 'WAITING' in plot.status['right'].get_text()
        assert np.isnan(plot.lines['left', 0].get_ydata()).sum() >= 2
        plot.fig.savefig(recording.output / 'test.png')
        assert (recording.output / 'test.png').stat().st_size > 1000
    finally:
        plot.plt.close(plot.fig)


def test_queue_overflow_never_blocks():
    writer = CsvWriter.__new__(CsvWriter)
    writer.error = None
    writer.queue = queue.Queue(maxsize=1)
    assert writer.put([1])
    assert not writer.put([2])


def test_disk_error_is_exposed(tmp_path):
    writer = CsvWriter.__new__(CsvWriter)
    writer.queue = queue.Queue()
    writer.queue.put([1])
    writer.stop = threading.Event()
    writer.stop.set()
    writer.error, writer.written = None, 0
    class FailingWriter:
        def writerow(self, row):
            raise OSError('disk full')
    writer.writer = FailingWriter()
    writer.stream = SimpleNamespace(close=lambda: None)
    writer.run()
    assert writer.error == 'disk full'
    assert not writer.put([2])


@pytest.mark.parametrize('args', [['--window-s', '0'], ['--plot-rate-hz', 'nan'],
                                 ['--queue-capacity', '-1'], ['--plot-rate-hz', '61'], ['--typo']])
def test_invalid_options(args):
    with pytest.raises(SystemExit):
        parse_args(args)
