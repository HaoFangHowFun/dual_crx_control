"""Physical dual-CRX bringup; does not launch a trajectory or motion script."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('dual_crx_control')
    description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([share, 'urdf', 'dual_crx.urdf.xacro'])]), value_type=str)
    physical_launch = PathJoinSubstitution([
        FindPackageShare('fanuc_hardware_interface'), 'launch', 'fanuc_physical_control.launch.py'])
    actions = [
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.2.100'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    for side in ('right', 'left'):
        # The driver's single-arm publishers use their own origin transforms.
        # Keep their TF private so only the calibrated dual URDF owns global TF.
        actions.append(GroupAction(actions=[
            SetRemap(src='/tf', dst=f'/{side}/driver_tf'),
            SetRemap(src='/tf_static', dst=f'/{side}/driver_tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(physical_launch),
                launch_arguments={
                    'robot_ip': LaunchConfiguration(f'{side}_robot_ip'),
                    'robot_series': 'crx',
                    'robot_model': 'crx5ia',
                    'namespace': side,
                    'prefix': f'{side}_',
                    'child_link': f'{side}_ee_mount',
                    'launch_rviz': 'false',
                    'motion_control': '1',
                    'initial_controller': 'none',
                }.items()),
        ]))
        actions.append(Node(
            package='controller_manager', executable='spawner',
            name=f'{side}_forward_position_spawner', output='screen',
            arguments=['forward_position_controller', '--controller-manager',
                       f'/{side}/controller_manager', '--controller-manager-timeout', '180']))
    actions.extend([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='dual_crx_robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='dual_crx_joint_state_merger', output='screen',
             parameters=[{'source_list': ['/left/joint_states', '/right/joint_states'],
                          'rate': 50, 'publish_default_positions': False}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'dual_cartesian.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
    return LaunchDescription(actions)
