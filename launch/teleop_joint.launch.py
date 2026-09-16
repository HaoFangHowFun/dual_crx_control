"""Standalone dual-arm joint control with real or ros2_control mock hardware."""

import math
import xml.etree.ElementTree as ET

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Radians, J1-J6; same starting poses as dual_arm_mock.launch.py.
# These initialize GenericSystem state only, never physical robot commands.
MOCK_INITIAL_POSITIONS = {
    'left': [0.0, 0.0, 0.0, 0.0, -math.pi / 2, 0.0],
    'right': [-math.pi / 2, 0.0, math.pi, 0.0, math.pi / 2, 0.0],
}


def arm_description(xacro_path, side, robot_ip, mock):
    description = xacro.process_file(xacro_path, mappings={
        'robot_ip': robot_ip, 'use_mock': str(mock).lower(),
        'prefix': f'{side}_', 'child_link': f'{side}_ee_mount', 'motion_control': '1',
    }).toxml()
    if not mock:
        return description
    # The driver's mock macro does not expose initial-position arguments.
    # Add ros2_control's standard initial_value parameter to the expanded XML.
    root = ET.fromstring(description)
    for index, position in enumerate(MOCK_INITIAL_POSITIONS[side], start=1):
        interface = root.find(
            f"ros2_control/joint[@name='{side}_J{index}']/state_interface[@name='position']")
        if interface is None:
            raise ValueError(f'Missing mock position interface for {side}_J{index}')
        initial = interface.find("param[@name='initial_value']")
        if initial is None:
            initial = ET.SubElement(interface, 'param', name='initial_value')
        initial.text = str(position)
    return ET.tostring(root, encoding='unicode')


def launch_setup(context):
    share = FindPackageShare('dual_crx_control')
    driver_share = FindPackageShare('fanuc_hardware_interface')
    controllers = PathJoinSubstitution([share, 'config', 'teleop_joint_controllers.yaml'])
    mock = LaunchConfiguration('mock').perform(context) == 'true'
    xacro_path = PathJoinSubstitution([driver_share, 'robot', 'crx5ia.urdf.xacro']).perform(context)
    actions = []
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
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in ('state_publish_rate', 'command_timeout')
        }]))
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
        DeclareLaunchArgument('command_timeout', default_value='0.2'),
        OpaqueFunction(function=launch_setup),
    ])
