#!/usr/bin/env python3

import argparse
import math
import os
import time
from functools import partial

import rclpy
from rclpy.node import Node
from dual_crx_control.interpolation.client import JointTargetClient
from dual_crx_control.joint_config import canonical_side
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


AMPLITUDE_DEG = 20.0
RUN_TIME = 10.0
MOTION_PERIOD = 4.0
RATE_HZ = 50.0
DEFAULT_JOINT = 1
DEFAULT_ROBOT_NAMESPACES = ('right', 'left')
INITIAL_HOLD_TIME = 1.0
RAMP_TIME = 1.0
PLOT_FILE_TEMPLATE = 'dual_j{joint}_periodic_response.png'
STATE_TIMEOUT = 1.0
READY_TIMEOUT = 10.0


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
        robot_namespaces,
        plot_file,
        show_plot
    ):
        super().__init__('dual_crx_synchronized_periodic_test')

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
        if amplitude_deg < 0.0:
            raise ValueError('amplitude_deg must be zero or positive.')
        for value in (run_time, motion_period, amplitude_deg,
                      initial_hold_time, ramp_time, rate_hz):
            if not math.isfinite(value):
                raise ValueError('Motion parameters must be finite.')
        robot_namespaces = tuple(clean_namespace(ns) for ns in robot_namespaces)
        if len(robot_namespaces) != 2 or len(set(robot_namespaces)) != 2:
            raise ValueError('Two distinct robot namespaces are required.')
        if not all(robot_namespaces):
            raise ValueError('Robot namespaces must not be empty.')

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
        self.robot_namespaces = robot_namespaces
        self.plot_file = plot_file
        self.show_plot = show_plot
        self.positions = {}
        self.state_received_at = {}
        self.target_client = JointTargetClient(self, arms=robot_namespaces)
        self.command_publishers = {}
        for ns in robot_namespaces:
            self.create_subscription(
                JointState,
                namespaced_topic(ns, 'joint_states'),
                partial(self.joint_state_callback, ns),
                qos_profile_sensor_data,
            )
            self.command_publishers[ns] = self.target_client.arm_publisher(ns)

    def joint_state_callback(self, namespace, msg):
        # Controller commands must be ordered J1..J6, regardless of feedback order.
        by_name = dict(zip(msg.name, msg.position))
        names = [f'{namespace}_J{i}' for i in range(1, 7)]
        if not all(name in by_name for name in names):
            return
        positions = [by_name[name] for name in names]
        if not all(math.isfinite(q) for q in positions):
            return
        self.positions[namespace] = positions
        self.state_received_at[namespace] = time.monotonic()

    def unready_robots(self):
        now = time.monotonic()
        return [
            ns for ns in self.robot_namespaces
            if ns not in self.positions
            or now - self.state_received_at[ns] > STATE_TIMEOUT
            or self.command_publishers[ns].get_subscription_count() == 0
        ]

    def process_feedback(self):
        for _ in range(16):
            rclpy.spin_once(self, timeout_sec=0.0)

    def amplitude_scale(self, elapsed):
        if self.ramp_time <= 0.0:
            return 1.0

        alpha = min(max(elapsed / self.ramp_time, 0.0), 1.0)

        # Smootherstep: zero velocity and acceleration at both endpoints.
        return alpha * alpha * alpha * (
            alpha * (alpha * 6.0 - 15.0) + 10.0
        )

    def wait_for_robots(self):
        self.get_logger().info('Waiting for both joint states and command subscribers...')
        deadline = time.monotonic() + READY_TIMEOUT
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            pending = self.unready_robots()
            if not pending:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Robots not ready: {", ".join(pending)}')
        raise RuntimeError('ROS shutdown while waiting for robots.')

    def publish_pair(self, starts, offset, elapsed, history):
        self.process_feedback()
        pending = self.unready_robots()
        if pending:
            raise RuntimeError(
                f'Stale feedback or missing command subscriber: {", ".join(pending)}'
            )
        messages = {}
        for ns in self.robot_namespaces:
            command = starts[ns].copy()
            command[self.joint_index] += offset
            msg = Float64MultiArray()
            msg.data = command
            messages[ns] = msg

        # Both messages use the same phase; no spinning or sleeping between sends.
        for ns in self.robot_namespaces:
            self.command_publishers[ns].publish(messages[ns])
        history['time'].append(elapsed)
        for ns in self.robot_namespaces:
            history[ns]['command'].append(messages[ns].data[self.joint_index])
            history[ns]['state'].append(self.positions[ns][self.joint_index])

    def run_test(self):
        self.wait_for_robots()
        self.get_logger().info(f'Robots        : {", ".join(self.robot_namespaces)}')
        self.get_logger().info(f'Joint         : {self.joint_label} on both robots')
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
            f'Press ENTER to start simultaneous periodic {self.joint_label} '
            f'motion on BOTH REAL robots ({", ".join(self.robot_namespaces)})...'
        )
        # Refresh both start positions after the operator's potentially long pause.
        self.positions.clear()
        self.state_received_at.clear()
        self.wait_for_robots()
        starts = {ns: self.positions[ns].copy() for ns in self.robot_namespaces}
        for ns, start in starts.items():
            self.get_logger().info(f'{ns} start: {start}')
        sample_period = 1.0 / self.rate_hz
        history = {'time': []}
        for ns in self.robot_namespaces:
            history[ns] = {'command': [], 'state': []}
        motion_end = self.initial_hold_time + self.run_time
        return_time = max(self.ramp_time, 1.0)
        total_time = motion_end + return_time + 0.2
        return_offset = 0.0
        start_time = time.monotonic()
        next_tick = start_time
        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            if elapsed >= total_time:
                self.publish_pair(starts, 0.0, elapsed, history)
                break
            if elapsed < self.initial_hold_time:
                offset = 0.0
            elif elapsed < motion_end:
                motion_elapsed = elapsed - self.initial_hold_time
                phase = 2.0 * math.pi * motion_elapsed / self.motion_period
                offset = (self.amplitude_scale(motion_elapsed)
                          * self.amplitude_rad * math.sin(phase))
                return_offset = offset
            else:
                # Avoid stepping straight to the start after a partial sine cycle.
                alpha = min((elapsed - motion_end) / return_time, 1.0)
                blend = alpha ** 3 * (alpha * (6.0 * alpha - 15.0) + 10.0)
                offset = return_offset * (1.0 - blend)

            self.publish_pair(starts, offset, elapsed, history)
            next_tick += sample_period
            now = time.monotonic()
            if next_tick <= now:
                next_tick += (math.floor((now - next_tick) / sample_period) + 1) * sample_period
            time.sleep(max(0.0, next_tick - time.monotonic()))

        self.get_logger().info('Both robots finished.' if rclpy.ok() else 'ROS shutdown.')
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

        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        for ax, ns in zip(axes, self.robot_namespaces):
            ax.plot(history['time'],
                    [math.degrees(q) for q in history[ns]['command']],
                    label=f'{self.joint_label} command', linewidth=2.0)
            ax.plot(history['time'],
                    [math.degrees(q) for q in history[ns]['state']],
                    label=f'{self.joint_label} joint state', linewidth=1.5)
            ax.set_title(f'{ns}: {self.joint_label} Periodic Motion')
            ax.set_ylabel('Position [deg]')
            ax.grid(True)
            ax.legend()
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()

        if self.plot_file:
            fig.savefig(self.plot_file, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved response plot: {self.plot_file}')

        if self.show_plot:
            plt.show()

        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run same-phase periodic motion on the selected joint of both robots.'
    )
    parser.add_argument(
        '--joint',
        type=int,
        default=DEFAULT_JOINT,
        help=f'joint number to move, 1 through 6 (default: {DEFAULT_JOINT})'
    )
    parser.add_argument(
        '--robot-namespaces',
        nargs=2,
        default=DEFAULT_ROBOT_NAMESPACES,
        help=(
            'two robot namespaces; joint prefixes must be <namespace>_ '
            '(default: robot1 robot2)'
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
        help='path for saved plot (default: dual_j<joint>_periodic_response.png)'
    )
    parser.add_argument(
        '--show-plot',
        action='store_true',
        help='show the response plot window after motion completes'
    )
    args = parser.parse_args()

    for name in ('time', 'period', 'amplitude', 'initial_hold', 'ramp_time', 'rate'):
        if not math.isfinite(getattr(args, name)):
            parser.error(f'--{name.replace("_", "-")} must be finite.')
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
    args.robot_namespaces = tuple(clean_namespace(ns) for ns in args.robot_namespaces)
    if not all(args.robot_namespaces) or len(set(args.robot_namespaces)) != 2:
        parser.error('--robot-namespaces requires two distinct nonempty namespaces.')
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
        robot_namespaces=args.robot_namespaces,
        plot_file=args.plot_file,
        show_plot=args.show_plot
    )

    try:
        node.run_test()

    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by user.')

    except RuntimeError as exc:
        node.get_logger().error(f'Motion aborted: {exc}')
        raise

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
