"""Standalone dual-arm joint control with real or ros2_control mock hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


from dual_crx_control.robot_description import arm_description, MOCK_INITIAL_POSITIONS


def launch_setup(context):
    share = FindPackageShare('dual_crx_control')
    driver_share = FindPackageShare('fanuc_hardware_interface')
    controllers = PathJoinSubstitution([share, 'config', 'dual_arm_controllers.yaml'])
    mock = LaunchConfiguration('mock').perform(context) == 'true'
    xacro_path = PathJoinSubstitution([driver_share, 'robot', 'crx5ia.urdf.xacro']).perform(context)
    interpolation = Node(
        package='dual_crx_control', executable='interpolation_node', output='screen',
        parameters=[{'input_rate_hz': 20.0, 'method': LaunchConfiguration('method')}])
    actions = [interpolation]
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
        parameters=[{'input_rate_hz': 20.0}, {
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name in ('state_publish_rate',)
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
        DeclareLaunchArgument('method', default_value='linear', choices=['linear', 'cubic']),
        OpaqueFunction(function=launch_setup),
    ])
