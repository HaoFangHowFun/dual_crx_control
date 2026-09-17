#!/usr/bin/env python3

import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node
from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.joint_config import canonical_side

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from dual_crx_control.latency_recording import LatencyRecording, ordered_feedback


AMPLITUDE_DEG = 20.0
RUN_TIME = 10.0
MOTION_PERIOD = 4.0
RATE_HZ = 50.0
DEFAULT_JOINT = 1
DEFAULT_ROBOT_NAMESPACE = 'right'
INITIAL_HOLD_TIME = 1.0
RAMP_TIME = 1.0
PLOT_FILE_TEMPLATE = 'j{joint}_periodic_response.png'


def clean_namespace(namespace):
    return canonical_side(namespace)


def namespaced_topic(robot_namespace, topic):
    if not robot_namespace:
        return f'/{topic}'

    return f'/{robot_namespace}/{topic}'


class JointTest(Node):
    def __init__(
        self,
        run_time,
        motion_period,
        amplitude_deg,
        joint,
        initial_hold_time,
        ramp_time,
        rate_hz,
        robot_namespace,
        plot_file,
        show_plot,
        latency_csv=None
    ):
        super().__init__('crx_periodic_j1_test', namespace=robot_namespace)

        if run_time <= 0.0:
            raise ValueError('run_time must be positive.')
        if motion_period <= 0.0:
            raise ValueError('motion_period must be positive.')
        if joint < 1 or joint > 6:
            raise ValueError('joint must be between 1 and 6.')
        if initial_hold_time < 0.0:
            raise ValueError('initial_hold_time must be zero or positive.')
        if ramp_time < 0.0:
            raise ValueError('ramp_time must be zero or positive.')
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive.')

        self.run_time = run_time
        self.motion_period = motion_period
        self.amplitude_deg = amplitude_deg
        self.amplitude_rad = math.radians(amplitude_deg)
        self.joint = joint
        self.joint_index = joint - 1
        self.joint_label = f'J{joint}'
        self.initial_hold_time = initial_hold_time
        self.ramp_time = ramp_time
        self.rate_hz = rate_hz
        self.robot_namespace = robot_namespace
        self.selected_joint_name = (
            f'{robot_namespace}_J{joint}' if robot_namespace else self.joint_label
        )
        self.joint_state_topic = namespaced_topic(robot_namespace, 'joint_states')
        self.command_topic = '/interpolation/joint_targets'
        print(f'Joint state topic: {self.joint_state_topic}')
        print(f'Command topic: {self.command_topic}')
        self.plot_file = plot_file
        self.show_plot = show_plot

        self.latency_csv = latency_csv
        self.latency_recording = LatencyRecording() if latency_csv else None
        self.current_joint_state = None

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )

        self.target_client = JointTargetClient(self, arms=[robot_namespace])
        self.publisher = self.target_client.arm_publisher(robot_namespace)
        if self.latency_recording is not None:
            self.create_subscription(JointState, '/interpolation/joint_commands', self.output_callback, 100)

    def output_callback(self, msg):
        if self.latency_recording is not None:
            values = dict(zip(msg.name, msg.position))
            names = [f'{self.robot_namespace}_J{i}' for i in range(1, 7)]
            if not set(names).issubset(values):
                return
            self.latency_recording.add(self.robot_namespace, 'command', [values[n] for n in names],
                                       received_at=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        if self.latency_recording is not None:
            try:
                q = ordered_feedback(msg, self.robot_namespace)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.latency_recording.add(self.robot_namespace, 'feedback', q, stamp=stamp)
            except (KeyError, ValueError):
                self.latency_recording.invalid += 1

    def publish_recorded(self, message):
        if self.latency_recording is not None:
            self.latency_recording.add(self.robot_namespace, 'target', message.data)
        self.publisher.publish(message)
        if self.latency_recording is not None:
            self.latency_recording.add(self.robot_namespace, 'target_publish_return', message.data)

    def current_joint_position(self):
        if (
            self.current_joint_state is None
            or len(self.current_joint_state.position) <= self.joint_index
        ):
            return None

        return self.current_joint_state.position[self.joint_index]

    def amplitude_scale(self, elapsed):
        if self.ramp_time <= 0.0:
            return 1.0

        alpha = min(max(elapsed / self.ramp_time, 0.0), 1.0)

        # Smootherstep: zero velocity and acceleration at both endpoints.
        return alpha * alpha * alpha * (
            alpha * (alpha * 6.0 - 15.0) + 10.0
        )

    def wait_for_joint_state(self):
        self.get_logger().info(f'Waiting for {self.joint_state_topic} ...')

        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            raise RuntimeError('No joint state received.')

        self.get_logger().info(
            f'Joint names: {list(self.current_joint_state.name)}'
        )

        self.get_logger().info(
            f'Current positions: {list(self.current_joint_state.position)}'
        )

    def run_test(self):
        self.wait_for_joint_state()

        start = list(self.current_joint_state.position)

        if len(start) < 6:
            raise RuntimeError(
                f'Expected 6 joints, received {len(start)}'
            )

        start = start[:6]

        positive_peak = start.copy()
        negative_peak = start.copy()
        positive_peak[self.joint_index] += self.amplitude_rad
        negative_peak[self.joint_index] -= self.amplitude_rad

        self.get_logger().info('-----------------------------------')
        self.get_logger().info(f'Namespace     : /{self.robot_namespace}')
        self.get_logger().info(f'Joint states  : {self.joint_state_topic}')
        self.get_logger().info(f'Command topic : {self.command_topic}')
        self.get_logger().info(f'Start         : {start}')
        self.get_logger().info(f'Joint         : {self.selected_joint_name}')
        self.get_logger().info(
            f'{self.joint_label} +amplitude : '
            f'{positive_peak[self.joint_index]} rad'
        )
        self.get_logger().info(
            f'{self.joint_label} -amplitude : '
            f'{negative_peak[self.joint_index]} rad'
        )
        self.get_logger().info(
            f'Run time      : {self.run_time:.3f} s'
        )
        self.get_logger().info(
            f'Period        : {self.motion_period:.3f} s'
        )
        self.get_logger().info(
            f'Amplitude     : {self.amplitude_deg:.3f} deg'
        )
        self.get_logger().info(
            f'Initial hold  : {self.initial_hold_time:.3f} s'
        )
        self.get_logger().info(
            f'Amplitude ramp: {self.ramp_time:.3f} s'
        )
        self.get_logger().info(
            f'Rate          : {self.rate_hz:.3f} Hz'
        )
        self.get_logger().info('-----------------------------------')

        input(
            f'Press ENTER to start periodic {self.selected_joint_name} motion '
            'on the REAL robot...'
        )

        sample_period = 1.0 / self.rate_hz
        command = start.copy()
        history = {
            'time': [],
            'command': [],
            'state': [],
        }

        start_time = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            if elapsed >= self.initial_hold_time:
                break

            if self.latency_recording is not None:
                self.latency_recording.add(self.robot_namespace, 'generated', command)
            msg = Float64MultiArray()
            msg.data = command
            self.publish_recorded(msg)

            for _callback in range(16):
                rclpy.spin_once(self, timeout_sec=0.0)
            history['time'].append(elapsed)
            history['command'].append(command[self.joint_index])
            history['state'].append(self.current_joint_position())
            time.sleep(sample_period)

        motion_start_time = time.monotonic()
        while rclpy.ok():

            motion_elapsed = time.monotonic() - motion_start_time
            if motion_elapsed >= self.run_time:
                break
            elapsed = self.initial_hold_time + motion_elapsed

            phase = 2.0 * math.pi * motion_elapsed / self.motion_period
            scale = self.amplitude_scale(motion_elapsed)
            command = start.copy()
            command[self.joint_index] = (
                start[self.joint_index]
                + scale * self.amplitude_rad * math.sin(phase)
            )

            if self.latency_recording is not None:
                self.latency_recording.add(self.robot_namespace, 'generated', command)
            msg = Float64MultiArray()
            msg.data = command

            self.publish_recorded(msg)

            for _callback in range(16):
                rclpy.spin_once(self, timeout_sec=0.0)
            history['time'].append(elapsed)
            history['command'].append(command[self.joint_index])
            history['state'].append(self.current_joint_position())
            time.sleep(sample_period)

        # Return the selected joint to the start position and hold briefly.
        msg = Float64MultiArray()
        msg.data = start

        for _ in range(100):
            self.publish_recorded(msg)
            for _callback in range(16):
                rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(sample_period)

        self.get_logger().info('Motion complete.')
        self.get_logger().info(
            f'{self.joint_label} periodic motion finished after '
            f'{self.initial_hold_time + self.run_time:.3f} s '
            f'({self.initial_hold_time:.3f} s hold + '
            f'{self.run_time:.3f} s motion, '
            f'{self.ramp_time:.3f} s ramp).'
        )

        self.plot_response(history)

    def plot_response(self, history):
        if not history['time']:
            self.get_logger().warning('No motion samples recorded; skipping plot.')
            return

        try:
            os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

            import matplotlib

            if self.plot_file and not self.show_plot:
                matplotlib.use('Agg')

            import matplotlib.pyplot as plt
        except ImportError as exc:
            self.get_logger().error(
                f'Could not create plot because matplotlib is unavailable: {exc}'
            )
            return

        command_deg = [math.degrees(q) for q in history['command']]
        state_deg = [
            math.degrees(q) if q is not None else float('nan')
            for q in history['state']
        ]

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(
            history['time'],
            command_deg,
            label=f'{self.joint_label} command',
            linewidth=2.0
        )
        ax.plot(
            history['time'],
            state_deg,
            label=f'{self.joint_label} joint state',
            linewidth=1.5
        )
        ax.set_title(f'{self.joint_label} Periodic Motion Time Response')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{self.joint_label} Position [deg]')
        ax.grid(True)
        ax.legend()

        if self.plot_file:
            fig.savefig(self.plot_file, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved response plot: {self.plot_file}')

        if self.show_plot:
            plt.show()

        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run adjustable periodic motion on one selected joint.'
    )
    parser.add_argument(
        '--joint',
        type=int,
        default=DEFAULT_JOINT,
        help=f'joint number to move, 1 through 6 (default: {DEFAULT_JOINT})'
    )
    parser.add_argument(
        '--robot-namespace',
        '--namespace',
        default=DEFAULT_ROBOT_NAMESPACE,
        help=(
            'robot namespace from dual_arm.launch.py '
            f'(default: {DEFAULT_ROBOT_NAMESPACE})'
        )
    )
    parser.add_argument(
        '--time',
        type=float,
        default=RUN_TIME,
        help=f'total motion time in seconds (default: {RUN_TIME})'
    )
    parser.add_argument(
        '--period',
        type=float,
        default=MOTION_PERIOD,
        help=f'sine period in seconds (default: {MOTION_PERIOD})'
    )
    parser.add_argument(
        '--amplitude',
        type=float,
        default=AMPLITUDE_DEG,
        help=f'selected joint sine amplitude in degrees (default: {AMPLITUDE_DEG})'
    )
    parser.add_argument(
        '--initial-hold',
        type=float,
        default=INITIAL_HOLD_TIME,
        help=(
            'time to hold the current position before periodic motion, '
            f'in seconds (default: {INITIAL_HOLD_TIME})'
        )
    )
    parser.add_argument(
        '--ramp-time',
        type=float,
        default=RAMP_TIME,
        help=(
            'time to smoothly ramp sine amplitude after initial hold, '
            f'in seconds (default: {RAMP_TIME})'
        )
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=RATE_HZ,
        help=f'command publish rate in Hz (default: {RATE_HZ})'
    )
    parser.add_argument(
        '--plot-file',
        default=None,
        help='path for saved response plot (default: j<joint>_periodic_response.png)'
    )
    parser.add_argument(
        '--show-plot',
        action='store_true',
        help='show the response plot window after motion completes'
    )
    parser.add_argument('--latency-csv', help='optional separate monotonic command/feedback event CSV')
    args = parser.parse_args()

    if args.joint < 1 or args.joint > 6:
        parser.error('--joint must be between 1 and 6.')
    if args.time <= 0.0:
        parser.error('--time must be positive.')
    if args.period <= 0.0:
        parser.error('--period must be positive.')
    if args.amplitude < 0.0:
        parser.error('--amplitude must be zero or positive.')
    if args.initial_hold < 0.0:
        parser.error('--initial-hold must be zero or positive.')
    if args.ramp_time < 0.0:
        parser.error('--ramp-time must be zero or positive.')
    if args.rate <= 0.0:
        parser.error('--rate must be positive.')
    args.robot_namespace = clean_namespace(args.robot_namespace)
    if args.plot_file is None:
        args.plot_file = PLOT_FILE_TEMPLATE.format(joint=args.joint)

    return args


def main():
    args = parse_args()

    rclpy.init()

    node = JointTest(
        run_time=args.time,
        motion_period=args.period,
        amplitude_deg=args.amplitude,
        joint=args.joint,
        initial_hold_time=args.initial_hold,
        ramp_time=args.ramp_time,
        rate_hz=args.rate,
        robot_namespace=args.robot_namespace,
        plot_file=args.plot_file,
        show_plot=args.show_plot,
        latency_csv=args.latency_csv
    )

    try:
        node.run_test()

    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by user.')

    finally:
        if node.latency_recording is not None:
            node.latency_recording.save(node.latency_csv,
                command_time='interpolation CLOCK_MONOTONIC sample stamp',
                generated_time='after sine target calculation',
                feedback_time='application callback receipt',
                motion_period=args.period, amplitude_deg=args.amplitude,
                initial_hold=args.initial_hold, ramp_time=args.ramp_time)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
