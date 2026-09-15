"""Software robot bringup only; run the Cartesian motion script separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('dual_crx_control')
    description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([share, 'urdf', 'dual_crx.urdf.xacro'])]), value_type=str)
    parameters = {'robot_description': description}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='dual_crx_robot_state_publisher', parameters=[parameters], output='screen'),
        Node(package='dual_crx_control', executable='dual_mock_robot.py',
             parameters=[parameters], output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'dual_cartesian.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
