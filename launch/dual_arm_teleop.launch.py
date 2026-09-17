"""Dual-arm teleoperation: teleop bridge, shared interpolator and joint recorder.

Before use (in each terminal):
    cd /home/msc-crx/ws_fanuc
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

Usage: ros2 launch dual_crx_control dual_arm_teleop.launch.py argument:=value
    # Mock hardware, no RViz, Ruckig interpolation:
    ros2 launch dual_crx_control dual_arm_teleop.launch.py mock:=true rviz:=false method:=ruckig
    # Real hardware, linear interpolation, targets actually sent at 100 Hz:
    ros2 launch dual_crx_control dual_arm_teleop.launch.py mock:=false method:=linear input_rate_hz:=100.0
    # List all arguments:
    ros2 launch dual_crx_control dual_arm_teleop.launch.py --show-args

Main arguments and defaults:
    mock:=true; rviz defaults to mock (on for mock, off for real); can be overridden.
    method:=linear (choices: linear / cubic / ruckig); input_rate_hz:=100.0.
    input_rate_hz is the expected input frequency, with 0 < Hz <= 500; it does not throttle input.
    linear/cubic use 1/input_rate_hz as the transition time; match the actual input rate.
    ruckig uses velocity, acceleration and jerk limits, not this value, to set arrival time.
    All methods output at 500 Hz.
    state_publish_rate:=100.0: merged feedback frequency, independent of target frequency.
    left_robot_ip:=192.168.2.100; right_robot_ip:=192.168.1.100.
    record:=true: automatically record joints; use record:=false to disable.
    record_output_dir:=/home/msc-crx/ws_fanuc/teleop_recordings: recording parent directory.
    Ctrl+C saves joints.csv and left/right plots in a timestamped subdirectory.

External input: /teleop/joint_command (Float64MultiArray; 12 radians, left J1-J6 then right J1-J6).
Merged feedback: /teleop/joint_states (JointState).
Joint names: left_J1..left_J6 / right_J1..right_J6; there is no prefix launch argument.
Choose either this launch or dual_arm.launch.py for the same pair of robots.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


from dual_crx_control.robot.description import arm_description, MOCK_INITIAL_POSITIONS


def launch_setup(context):
    share = FindPackageShare('dual_crx_control')
    driver_share = FindPackageShare('fanuc_hardware_interface')
    controllers = PathJoinSubstitution([share, 'config', 'dual_arm_controllers.yaml'])
    mock = LaunchConfiguration('mock').perform(context) == 'true'
    xacro_path = PathJoinSubstitution([driver_share, 'robot', 'crx5ia.urdf.xacro']).perform(context)
    interpolation = Node(
        package='dual_crx_control', executable='interpolation_node', output='screen',
        parameters=[{
            'input_rate_hz': ParameterValue(LaunchConfiguration('input_rate_hz'), value_type=float),
            'method': LaunchConfiguration('method'),
        }])
    actions = [
        Node(package='dual_crx_control', executable='record_teleoperation.py', output='screen',
             condition=IfCondition(LaunchConfiguration('record')),
             parameters=[{'output_dir': ParameterValue(
                 LaunchConfiguration('record_output_dir'), value_type=str)}],
             sigterm_timeout='60', sigkill_timeout='10'),
        interpolation,
    ]
    for side in ('left', 'right'):
        description = ParameterValue(arm_description(
            xacro_path, side, LaunchConfiguration(f'{side}_robot_ip').perform(context), mock),
            value_type=str)
        actions.extend([
            # Each controller manager receives its own latched robot description.
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=side, output='screen',
                 parameters=[{'robot_description': description}],
                 remappings=[('/tf', f'/{side}/tf'), ('/tf_static', f'/{side}/tf_static')]),
            Node(package='controller_manager', executable='ros2_control_node',
                 namespace=side, output='screen', parameters=[controllers],
                 remappings=[('robot_description', f'/{side}/robot_description')]),
            Node(package='controller_manager', executable='spawner',
                 namespace=side, name='joint_controller_spawner', output='screen',
                 arguments=['joint_state_broadcaster', 'forward_position_controller',
                            '--controller-manager', f'/{side}/controller_manager',
                            '--controller-manager-timeout', '180',
                            '--param-file', controllers]),
        ])
    actions.append(Node(
        package='dual_crx_control', executable='teleop_bridge', output='screen',
        parameters=[{'state_publish_rate': ParameterValue(
            LaunchConfiguration('state_publish_rate'), value_type=float)}]))
    # One combined model owns global TF; each driver's single-arm TF stays private.
    combined_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([share, 'urdf', 'dual_crx.urdf.xacro']),
    ]), value_type=str)
    actions.extend([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='teleop_robot_state_publisher', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'robot_description': combined_description, 'publish_frequency': 100.0}],
             remappings=[('joint_states', '/teleop/joint_states'),
                         ('robot_description', '/teleop/robot_description')]),
        Node(package='rviz2', executable='rviz2', name='teleop_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'teleop_joint.rviz'])]),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value=LaunchConfiguration('mock'),
                              choices=['true', 'false'],
                              description='Show both arms; defaults on for mock, off for real hardware'),
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.2.100'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('state_publish_rate', default_value='100.0'),
        DeclareLaunchArgument('input_rate_hz', default_value='100.0',
                              description='Expected upstream command rate (0 < Hz <= 500); sets interpolation horizon'),
        DeclareLaunchArgument('method', default_value='linear', choices=['linear', 'cubic', 'ruckig']),
        DeclareLaunchArgument('record', default_value='true', choices=['true', 'false'],
                              description='Save joint CSV and left/right plots on shutdown'),
        DeclareLaunchArgument('record_output_dir',
                              default_value='/home/msc-crx/ws_fanuc/teleop_recordings',
                              description='Parent directory for timestamped joint recordings'),
        OpaqueFunction(function=launch_setup),
    ])
