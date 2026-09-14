from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"),
        "urdf",
        "dual_crx.urdf.xacro",
    ])

    robot_description = Command([
        "xacro ",
        xacro_file,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

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

            "namespace": "right",
            "prefix": "right_",

            "use_mock": "false",
            "launch_rviz": "false",
            "initial_controller": "none",
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fanuc_physical_launch),
        launch_arguments={
            "robot_ip": "192.168.2.100",
            "robot_series": "crx",
            "robot_model": "crx5ia",

            "namespace": "left",
            "prefix": "left_",

            "use_mock": "false",
            "launch_rviz": "false",
            "initial_controller": "none",
        }.items(),
    )

    forward_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ns}_forward_position_spawner",
            arguments=[
                "forward_position_controller",
                "--controller-manager", f"/{ns}/controller_manager",
                "--controller-manager-timeout", "180",
            ],
            output="screen",
        )
        for ns in ["right", "left"] 
    ]

    return LaunchDescription([
        robot1,
        robot2,
        *forward_spawners,
        robot_state_publisher,
        rviz,
    ])
