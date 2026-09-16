"""Record published joints and received feedback; compute TCP plots after motion."""

from collections import deque
import csv
from datetime import datetime
import json
from pathlib import Path

import numpy as np


class MotionRecording:
    """Bounded in-memory recording with independent command/feedback timestamps."""

    def __init__(self, axis, max_samples=60000, plane=None):
        self.axis = axis
        if plane is not None and plane not in ('xy', 'xz', 'yz'):
            raise ValueError('plot plane must be xy/xz/yz')
        self.plane = plane
        self.second_axis = 'xyz'.index(plane[1]) if plane else None
        self.started_at = None
        self.origins = {}
        self.commands = deque(maxlen=max_samples)
        self.targets = deque(maxlen=max_samples)
        self.target_count = 0
        self.feedback = {s: deque(maxlen=max_samples) for s in ('left', 'right')}
        self.command_count = 0
        self.feedback_count = dict.fromkeys(self.feedback, 0)

    def begin(self, started_at, starts):
        self.started_at = started_at
        self.origins = {s: float(pose[self.axis, 3]) for s, pose in starts.items()}
        self.second_origins = ({s: float(pose[self.second_axis, 3]) for s, pose in starts.items()}
                               if self.plane else {})

    def command(self, received_at, joints):
        if self.started_at is not None:
            self.commands.append((received_at - self.started_at,
                                  np.array([joints[s] for s in ('left', 'right')], copy=True)))
            self.command_count += 1

    def target(self, received_at, joints):
        if self.started_at is not None:
            self.targets.append((received_at - self.started_at,
                                 np.array([joints[s] for s in ('left', 'right')], copy=True)))
            self.target_count += 1

    def measured(self, side, received_at, joints, stamp):
        if self.started_at is not None:
            self.feedback[side].append((received_at - self.started_at, joints.copy(), stamp))
            self.feedback_count[side] += 1

    def save(self, models, output_dir):
        """Run after publication stops. Return PNG, raw-joint/TCP CSV, metadata paths."""
        if not self.commands:
            return None
        output = Path(output_dir).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        label = f'circle_{self.plane}' if self.plane else f'axis_{"xyz"[self.axis]}'
        name = f'{label}_{datetime.now():%Y%m%d_%H%M%S_%f}'
        png, csv_path, metadata_path = [output / (name + ext) for ext in ('.png', '.csv', '.json')]
        series = {side: {'target': [], 'command': [], 'feedback': []} for side in self.feedback}
        with csv_path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['arm', 'source', 'time_s', 'feedback_ros_stamp_s',
                             f'world_{"xyz"[self.axis]}_m', 'displacement_mm',
                             *([f'world_{self.plane[1]}_m', f'displacement_{self.plane[1]}_mm']
                               if self.plane else []),
                             *[f'J{i}_rad' for i in range(1, 7)]])
            for i, side in enumerate(self.feedback):
                data = [(t, q[i], '') for t, q in self.commands]
                for source, rows in [('target', [(t, q[i], '') for t, q in self.targets]),
                                     ('command', data), ('feedback', self.feedback[side])]:
                    for t, q, stamp in rows:
                        pose = models[side].fk(q)
                        value = float(pose[self.axis, 3])
                        displacement = 1000. * (value - self.origins[side])
                        extra = []
                        if self.plane:
                            second = float(pose[self.second_axis, 3])
                            extra = [second, 1000. * (second - self.second_origins[side])]
                        writer.writerow([side, source, t, stamp, value, displacement, *extra, *q])
                        series[side][source].append((t, displacement, *extra[1:]))
        # Importing matplotlib, FK, and disk output never run in the command loop.
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = (plt.subplots(1, 2, figsize=(11, 5)) if self.plane
                     else plt.subplots(2, 1, figsize=(10, 7), sharex=True))
        for ax, side in zip(axes, self.feedback):
            for source, label, style in [('command', 'Command (published joints → FK)', '-'),
                                         ('feedback', 'Measured (joint feedback → FK)', '--')]:
                points = np.array(series[side][source])
                if len(points):
                    x, y = (points[:, 1], points[:, 2]) if self.plane else (points[:, 0], points[:, 1])
                    ax.plot(x, y, style, label=label, linewidth=1.5)
            if self.plane:
                ax.set_aspect('equal', adjustable='box')
                ax.set_xlabel(f'World {self.plane[0].upper()} displacement [mm]')
                ax.set_ylabel(f'World {self.plane[1].upper()} displacement [mm]')
                ax.set_title(f'{side.capitalize()} TCP — {self.plane.upper()} circle')
            else:
                ax.set_title(f'{side.capitalize()} TCP — world {"xyz"[self.axis].upper()}')
                ax.set_ylabel('Displacement from start [mm]')
            ax.grid(True)
            ax.legend()
        if not self.plane:
            axes[-1].set_xlabel('Time since Cartesian motion started [s]')
        fig.suptitle('Command versus measured TCP position')
        fig.tight_layout()
        fig.savefig(png, dpi=150)
        plt.close(fig)
        metadata = {
            'axis': 'xyz'[self.axis], 'initial_world_axis_m': self.origins,
            'plane': self.plane, 'initial_world_second_axis_m': self.second_origins,
            'time_basis': 'local monotonic target publication / interpolation sample time / feedback receipt time',
            'target_samples': len(self.targets), 'target_samples_total': self.target_count,
            'measurement': 'FK of joint feedback; not an external TCP measurement',
            'command_samples': len(self.commands), 'command_samples_total': self.command_count,
            'feedback_samples': {s: len(rows) for s, rows in self.feedback.items()},
            'feedback_samples_total': self.feedback_count,
            'recording_truncated': (self.target_count > len(self.targets) or self.command_count > len(self.commands) or any(
                self.feedback_count[s] > len(rows) for s, rows in self.feedback.items())),
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + '\n')
        return png, csv_path, metadata_path
