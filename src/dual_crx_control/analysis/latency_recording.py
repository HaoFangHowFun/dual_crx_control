"""Bounded monotonic event recording; disk I/O runs only after capture stops."""
from collections import deque
import csv
import json
from pathlib import Path
import time


class LatencyRecording:
    def __init__(self, capacity=360000):
        self.rows = deque(maxlen=capacity)
        self.count = 0
        self.invalid = 0

    def add(self, arm, source, q=(), stamp='', scaling='', received_at=None):
        self.rows.append((time.monotonic() if received_at is None else received_at,
                          arm, source, tuple(q), stamp, scaling))
        self.count += 1

    def save(self, path, **metadata):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'arm', 'source', 'feedback_ros_stamp_s', 'scaling',
                             *[f'J{i}_rad' for i in range(1, 7)]])
            for t, arm, source, q, stamp, scaling in self.rows:
                writer.writerow([t, arm, source, stamp, scaling, *(q if q else ['']*6)])
        path.with_suffix('.json').write_text(json.dumps(dict(
            clock='Linux CLOCK_MONOTONIC seconds; same-host driver timestamps comparable',
            samples=len(self.rows), total=self.count, overwritten=self.count-len(self.rows),
            invalid_messages=self.invalid, **metadata), indent=2) + '\n')
        return path
