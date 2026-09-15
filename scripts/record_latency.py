#!/usr/bin/env python3
"""Passive command/feedback/scaling recorder. Never publishes motion commands."""
import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from fanuc_msgs.msg import CollaborativeSpeedScaling

from dual_crx_control.latency_recording import LatencyRecording, ordered_feedback


class LatencyObserver(Node):
    def __init__(self, arms=('left', 'right'), **kwargs):
        super().__init__('fanuc_latency_observer', **kwargs)
        self.recording = LatencyRecording()
        for arm in arms:
            self.create_subscription(Float64MultiArray, f'/{arm}/forward_position_controller/commands',
                lambda msg, s=arm: self.command(s, msg), qos_profile_sensor_data)
            self.create_subscription(JointState, f'/{arm}/joint_states',
                lambda msg, s=arm: self.feedback(s, msg), qos_profile_sensor_data)
            self.create_subscription(CollaborativeSpeedScaling,
                f'/{arm}/fanuc_gpio_controller/collaborative_speed_scaling',
                lambda msg, s=arm: self.recording.add(s, 'scaling', scaling=msg.collaborative_speed_scaling),
                qos_profile_sensor_data)

    def command(self, arm, message):
        if len(message.data) != 6 or not all(math.isfinite(x) for x in message.data):
            self.recording.invalid += 1
            return
        self.recording.add(arm, 'command', message.data)

    def feedback(self, arm, message):
        received = time.monotonic()
        try:
            q = ordered_feedback(message, arm)
            if not all(math.isfinite(x) for x in q):
                raise ValueError('nonfinite feedback')
        except (KeyError, ValueError):
            self.recording.invalid += 1
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.recording.add(arm, 'feedback', q, stamp=stamp, received_at=received)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--output', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    rclpy.init(args=ros_args)
    node = LatencyObserver()
    node.get_logger().info('Passive capture only: command receipt, feedback receipt, and speed scaling.')
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(node.recording.save(args.output,
              command_time='observer callback receipt, NOT application publication',
              feedback_time='observer callback receipt; header stamp kept separately'))


if __name__ == '__main__':
    main()
