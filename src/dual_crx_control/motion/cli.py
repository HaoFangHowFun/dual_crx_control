"""Translate motion CLI options to existing ROS controller parameters."""

import argparse

from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args

from dual_crx_control.joint_config import INITIAL_JOINTS_DEG
from dual_crx_control.motion.cartesian_controller import DualCartesianController, main
from dual_crx_control.motion.circle_controller import DualCircleController


class DualFacingCircleController(DualCircleController):
    def __init__(self, **kwargs):
        super().__init__(
            node_name='facing_circle',
            motion_defaults={'period': 3., 'cycles': 10, 'max_velocity': 3., 'rate': 50.},
            circle_defaults={'radius': .1, 'plane': 'xz', 'direction': 'cw',
                             'face_each_other': True, 'tcp_gap': .02,
                             'center_midpoint': [.55, -.38, .35], 'minimum_scaled_sigma': .1},
            **kwargs)


def parse_parameters(kind, argv=None):
    parser = argparse.ArgumentParser(
        description=f'Synchronized {kind} motion on left_/right_ joints. '
                    'CLI options map to ROS parameters; explicit CLI values take precedence.',
        epilog='Match --rate to launch input_rate_hz. Output: timestamped joints.csv, '
               'six-joint plots and additional TCP CSV/plot/JSON. Ctrl+C saves recorded data.')
    defaults = dict(DualCartesianController.DEFAULTS,
                    move_to_initial=True, save_plot=True, output_dir='motion_recordings', robot_description='',
                    **{f'{side}_initial_deg': angles for side, angles in INITIAL_JOINTS_DEG.items()})
    if kind != 'cartesian_sine':
        defaults.update(DualCircleController.CIRCLE_DEFAULTS)
    aliases = {'amplitude': 'amplitude-m', 'radius': 'radius-m', 'tcp_gap': 'tcp-gap-m'}
    units = {'rate': 'Target Hz; match launch input_rate_hz', 'period': 'Period [s]',
             'cycles': 'Number of cycles; 0 repeats until Ctrl+C',
             'amplitude': 'Translation amplitude [m]', 'radius': 'Circle radius [m]',
             'tcp_gap': 'Facing TCP gap [m]', 'center_midpoint': 'World center midpoint X Y Z [m]',
             'output_dir': 'Parent directory for timestamped recordings',
             'save_plot': 'Enable joint CSV/plots and additional TCP analysis',
             'move_to_initial': 'Approach configured initial joints before Cartesian motion'}
    choices = {'axis': ('x', 'y', 'z'), 'plane': ('xy', 'xz', 'yz'), 'direction': ('cw', 'ccw')}
    for name, value in defaults.items():
        option = '--' + aliases.get(name, name.replace('_', '-'))
        kwargs = dict(dest=name, default=argparse.SUPPRESS, help=units.get(name, f'ROS parameter: {name}'))
        if isinstance(value, bool):
            kwargs['action'] = argparse.BooleanOptionalAction
        elif isinstance(value, list):
            kwargs.update(type=float, nargs=len(value))
        else:
            kwargs['type'] = type(value)
        if name in choices:
            kwargs['choices'] = choices[name]
        parser.add_argument(option, **kwargs)
    values = vars(parser.parse_args(argv))
    return [Parameter(name, value=value) for name, value in values.items()]


def run(kind):
    overrides = parse_parameters(kind, remove_ros_args()[1:])
    controller = {'cartesian_sine': DualCartesianController,
                  'cartesian_circle': DualCircleController,
                  'facing_circle': DualFacingCircleController}[kind]
    main(controller, parameter_overrides=overrides)
