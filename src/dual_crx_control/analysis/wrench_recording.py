"""Bounded asynchronous wrench recording with measured-FK pose association."""
import argparse
from collections import Counter, deque
import csv
from datetime import datetime
from functools import partial
import hashlib
import json
from pathlib import Path
import queue
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

from dual_crx_control.robot.joint_config import SIDES, ordered_feedback
from dual_crx_control.robot.kinematics import CRXKinematics

FIELDS = ['time_s', 'arm', 'wrench_stamp_s', 'joint_stamp_s', 'pose_age_s', 'pose_valid',
          'wrench_valid', 'wrench_frame', 'pose_frame', 'tcp_frame',
          'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw',
          'fx_N', 'fy_N', 'fz_N', 'tx_Nm', 'ty_Nm', 'tz_Nm']
CHANNELS = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class CsvWriter:
    def __init__(self, path, capacity):
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.error = None
        self.written = 0
        self.stream = path.open('w', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(FIELDS)
        self.stream.flush()
        self.thread = threading.Thread(target=self.run, name='wrench_csv', daemon=True)
        self.thread.start()

    def put(self, row):
        if self.error:
            return False
        try:
            self.queue.put_nowait(row)
            return True
        except queue.Full:
            return False

    def run(self):
        flushed = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    row = None
                if row is not None:
                    self.writer.writerow(row)
                    self.written += 1
                if time.monotonic() - flushed >= 1.:
                    self.stream.flush()
                    flushed = time.monotonic()
        except Exception as exc:
            self.error = str(exc)
        finally:
            try:
                self.stream.close()
            except Exception as exc:
                self.error = str(exc)

    def close(self):
        self.stop.set()
        self.thread.join()


class Recording:
    def __init__(self, args):
        self.args = args
        self.started = time.monotonic()
        self.output = Path(args.output_dir).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.output.mkdir(parents=True)
        self.writer = CsvWriter(self.output / 'wrench.csv', args.queue_capacity)
        self.lock = threading.Lock()
        self.buffers = {s: deque(maxlen=args.plot_capacity) for s in SIDES}
        self.latest = {}
        self.counts = Counter()
        self.frames = {s: set() for s in SIDES}
        self.last_written = {}
        self.closed = False

    def add(self, side, values, frame, wrench_stamp, pose, now=None):
        now = time.monotonic() if now is None else now
        age = now - pose[0] if pose is not None else None
        valid_pose = age is not None and 0 <= age <= self.args.pose_timeout_s
        valid_wrench = bool(np.isfinite(values).all())
        row = [now - self.started, side, wrench_stamp, pose[1] if pose else '',
               age if age is not None else '', valid_pose, valid_wrench, frame, 'world', f'{side}_tcp',
               *(pose[2] if valid_pose else [''] * 7), *values]
        self.counts[f'{side}.received'] += 1
        if not valid_pose:
            self.counts[f'{side}.invalid_pose'] += 1
        if not valid_wrench:
            self.counts[f'{side}.invalid_wrench'] += 1
        if wrench_stamp == 0:
            self.counts[f'{side}.zero_stamp'] += 1
        if now - self.last_written.get(side, -float('inf')) >= 1. / self.args.record_rate_hz:
            if not self.writer.put(row):
                self.counts['queue_dropped'] += 1
            else:
                self.last_written[side] = now
                self.counts[f'{side}.written'] += 1
        else:
            self.counts[f'{side}.downsampled'] += 1
        with self.lock:
            self.frames[side].add(frame)
            self.latest[side] = row
            self.buffers[side].append((row[0], np.array(values) if valid_wrench else np.full(6, np.nan), frame))
            self.trim(side, row[0])
        return row

    def trim(self, side, elapsed):
        while self.buffers[side] and self.buffers[side][0][0] < elapsed - self.args.window_s:
            self.buffers[side].popleft()

    def snapshot(self):
        now = time.monotonic() - self.started
        with self.lock:
            for side in SIDES:
                self.trim(side, now)
            return now, {s: list(b) for s, b in self.buffers.items()}, dict(self.latest)

    def close(self, metadata):
        if self.closed:
            return
        self.closed = True
        self.writer.close()
        metadata.update(settings=vars(self.args), counts=dict(self.counts), written=self.writer.written,
                        writer_error=self.writer.error, frames={s: sorted(v) for s, v in self.frames.items()},
                        units={'position': 'm', 'force': 'N', 'torque': 'N*m'},
                        pairing='latest received valid joint feedback; monotonic receive age, not synchronized measurement',
                        stamps='original ROS stamps preserved, zero means unknown; not comparable to monotonic time',
                        wrench_reference='source frame; FANUC default is flange', compensation='unknown',
                        qos='best_effort, volatile, depth=5; DDS losses not measurable here')
        (self.output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')


class WrenchRecorder(Node):
    def __init__(self, recording, **kwargs):
        super().__init__('wrench_recorder', **kwargs)
        self.recording = recording
        self.models, self.poses = {}, {}
        self.description = ''
        self.fatal = None
        for side in SIDES:
            self.create_subscription(JointState, f'/{side}/joint_states', partial(self.feedback, side), qos_profile_sensor_data)
            self.create_subscription(WrenchStamped, getattr(recording.args, f'{side}_wrench_topic'),
                                     partial(self.wrench, side), qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.description_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        description = self.declare_parameter('robot_description', '').value
        if description:
            self.configure(description)

    def configure(self, description):
        if self.description and self.description != description:
            raise ValueError('robot_description changed; restart recorder')
        if not self.description:
            self.models = {s: CRXKinematics(description, f'{s}_tcp') for s in SIDES}
            self.description = description

    def description_callback(self, message):
        try:
            self.configure(message.data)
        except Exception as exc:
            self.fatal = f'Robot model error: {exc}'

    def feedback(self, side, message):
        if side not in self.models:
            return
        received = time.monotonic()
        try:
            q = np.asarray(ordered_feedback(message, side), float)
            if not self.models[side].valid_joints(q):
                raise ValueError('invalid joint feedback')
            transform = self.models[side].fk(q)
            pose = [*transform[:3, 3], *Rotation.from_matrix(transform[:3, :3]).as_quat()]
            self.poses[side] = received, stamp(message), pose
        except (ValueError, KeyError, TypeError):
            self.poses.pop(side, None)
            self.recording.counts[f'{side}.invalid_joint_message'] += 1

    def wrench(self, side, message):
        if self.fatal or self.recording.writer.error:
            return
        w = message.wrench
        frame = message.header.frame_id
        if not frame:
            self.get_logger().warning(f'{side}: wrench frame is unknown', throttle_duration_sec=5.)
        self.recording.add(side, [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z],
                           frame, stamp(message), self.poses.get(side))
        if self.recording.counts['queue_dropped']:
            self.get_logger().warning(f"CSV queue dropped {self.recording.counts['queue_dropped']} samples", throttle_duration_sec=5.)


class LivePlot:
    def __init__(self, recording, live=True):
        import matplotlib
        if not live:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        self.plt, self.recording = plt, recording
        self.fig, self.axes = plt.subplots(6, 2, figsize=(13, 11), sharex=True)
        if live and self.fig.canvas.manager is not None:
            if not getattr(type(self.fig.canvas), 'required_interactive_framework', None):
                plt.close(self.fig)
                raise RuntimeError('No interactive Matplotlib backend; use --no-plot or enable WSLg/GUI')
        self.lines = {}
        for col, side in enumerate(SIDES):
            for i, channel in enumerate(CHANNELS):
                ax = self.axes[i, col]
                self.lines[side, i], = ax.plot([], [], linewidth=.9)
                ax.set_ylabel(f'{channel} [{"N" if i < 3 else "N*m"}]')
                ax.grid(True)
            self.axes[-1, col].set_xlabel('Time since recording started [s]')
        self.fig.subplots_adjust(top=.88, hspace=.3)
        self.status = {s: self.fig.text(.25 + .5 * i, .95, '', ha='center', va='top', fontsize=9)
                       for i, s in enumerate(SIDES)}

    def update(self):
        now, buffers, latest = self.recording.snapshot()
        for col, side in enumerate(SIDES):
            row = latest.get(side)
            state = 'WAITING' if row is None else ('STALE' if now-row[0] > self.recording.args.wrench_timeout_s else 'LIVE')
            if row and not row[6] and state == 'LIVE':
                state = 'INVALID'
            pose = 'pose unavailable'
            if row and row[5]:
                age = row[4] + now - row[0]
                pose = f'XYZ {np.round(row[10:13], 4)}; age {age:.3f}s'
                if age > self.recording.args.pose_timeout_s:
                    pose += ' (STALE)'
            self.status[side].set_text(f'{side.upper()} {state} | frame: {(row[7] or "unknown") if row else "unknown"}\n{pose}')
            points = []
            previous = None
            for t, values, frame in buffers[side]:
                if previous and (t-previous[0] > self.recording.args.wrench_timeout_s or frame != previous[1]):
                    points.append([t, *([np.nan]*6)])
                points.append([t, *values])
                previous = t, frame
            data = np.array(points).reshape(-1, 7)
            for i in range(6):
                self.lines[side, i].set_data(data[:, 0], data[:, i+1])
                ax = self.axes[i, col]
                ax.relim()
                ax.autoscale_view(scalex=False)
                ax.set_xlim(max(0., now-self.recording.args.window_s), max(now, .1))
        self.fig.suptitle(f"Wrench recording | CSV rows: {self.recording.writer.written} | dropped: {self.recording.counts['queue_dropped']}")
        self.fig.canvas.draw_idle()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for side in SIDES:
        parser.add_argument(f'--{side}-wrench-topic', default=f'/{side}/force_torque_sensor_broadcaster/wrench')
    parser.add_argument('--output-dir', default='wrench_recordings')
    parser.add_argument('--no-plot', action='store_true')
    for name, default in [('window-s', 10.), ('plot-rate-hz', 10.), ('record-rate-hz', 50.),
                          ('pose-timeout-s', .1), ('wrench-timeout-s', .5)]:
        parser.add_argument('--'+name, type=float, default=default)
    for name, default in [('queue-capacity', 50000), ('plot-capacity', 10000)]:
        parser.add_argument('--'+name, type=int, default=default)
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('Unknown arguments: ' + ' '.join(ros_args))
    for key in ('window_s', 'plot_rate_hz', 'record_rate_hz', 'pose_timeout_s', 'wrench_timeout_s', 'queue_capacity', 'plot_capacity'):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f'{key} must be finite and positive')
    if args.plot_rate_hz > 60:
        parser.error('plot-rate-hz must not exceed 60')
    return args, ros_args


def main():
    args, ros_args = parse_args()
    recording = Recording(args)
    node = executor = thread = plot = None
    spin_error = []
    stop = threading.Event()
    handlers = {}
    failure = None
    print(f'Recording to {recording.output.resolve()}', flush=True)
    try:
        plot = LivePlot(recording, live=not args.no_plot)
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, lambda *_: stop.set())
        node = WrenchRecorder(recording)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass
            except Exception as exc:
                spin_error.append(str(exc))
        thread = threading.Thread(target=spin, daemon=True)
        thread.start()
        if not args.no_plot:
            plot.plt.show(block=False)
        while rclpy.ok() and not stop.is_set():
            if recording.writer.error or node.fatal or spin_error:
                raise RuntimeError(recording.writer.error or node.fatal or spin_error[0])
            if args.no_plot:
                time.sleep(.1)
            else:
                if not plot.plt.fignum_exists(plot.fig.number):
                    break
                plot.update()
                plot.plt.pause(1. / args.plot_rate_hz)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        failure = str(exc)
    finally:
        if executor:
            executor.shutdown()
        if thread:
            thread.join()
        metadata = {'error': failure, 'robot_description_sha256': hashlib.sha256(node.description.encode()).hexdigest() if node else None}
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            recording.close(metadata)
            if plot:
                plot.update()
                plot.fig.savefig(recording.output / 'wrench.png', dpi=120)
        except Exception as exc:
            failure = failure or str(exc)
        finally:
            if plot:
                plot.plt.close(plot.fig)
        failure = failure or recording.writer.error
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if failure:
        raise SystemExit(f'Recording failed: {failure}')
