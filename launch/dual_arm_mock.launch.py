from launch import LaunchDescription
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
        name="dual_crx_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{
            "robot_description": robot_description,

            # radians
            "zeros": {
                "left_J1": 0.0,
                "left_J2": 0.0,
                "left_J3": 0.0,
                "left_J4": 0.0,
                "left_J5": -1.57079632679,
                "left_J6": 0.0,

                "right_J1": -1.57079632679,
                "right_J2": 0.0,
                "right_J3": 3.14159265359,
                "right_J4": 0.0,
                "right_J5": 1.57079632679,
                "right_J6": 0.0,
            }
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])