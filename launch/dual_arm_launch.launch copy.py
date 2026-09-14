from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    fanuc_physical_launch = PathJoinSubstitution([
        FindPackageShare("fanuc_hardware_interface"),
        "launch",
        "fanuc_physical_control.launch.py",
    ])

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fanuc_physical_launch),
        launch_arguments={
            "robot_ip": "192.168.1.100",
            "robot_series": "crx",
            "robot_model": "crx5ia",

            "namespace": "robot1",
            "prefix": "robot1_",

            "use_mock": "false",
            "launch_rviz": "false",
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fanuc_physical_launch),
        launch_arguments={
            "robot_ip": "192.168.2.100",
            "robot_series": "crx",
            "robot_model": "crx5ia",

            "namespace": "robot2",
            "prefix": "robot2_",

            "use_mock": "false",
            "launch_rviz": "false",
        }.items(),
    )

    return LaunchDescription([
        robot1,
        robot2,
    ])