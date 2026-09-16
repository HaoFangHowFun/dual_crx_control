"""Shared dual-arm bringup: GenericSystem mock or FANUC hardware, joint commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from dual_crx_control.robot_description import arm_description


def launch_setup(context):
    mock = LaunchConfiguration("mock").perform(context) == "true"
    xacro_file = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "urdf", "dual_crx.urdf.xacro",
    ])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    driver_xacro = PathJoinSubstitution([
        FindPackageShare("fanuc_hardware_interface"), "robot", "crx5ia.urdf.xacro",
    ]).perform(context)
    controllers = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "config", "dual_arm_controllers.yaml",
    ])
    right_description = arm_description(
        driver_xacro, "right", LaunchConfiguration("right_robot_ip").perform(context), mock)
    left_description = arm_description(
        driver_xacro, "left", LaunchConfiguration("left_robot_ip").perform(context), mock)

    # Driver descriptions own controller interfaces; the combined model owns global TF.
    right_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="right", parameters=[{"robot_description": right_description}],
        remappings=[("/tf", "/right/driver_tf"), ("/tf_static", "/right/driver_tf_static")],
        output="screen",
    )
    left_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace="left", parameters=[{"robot_description": left_description}],
        remappings=[("/tf", "/left/driver_tf"), ("/tf_static", "/left/driver_tf_static")],
        output="screen",
    )
    robot1 = Node(
        package="controller_manager", executable="ros2_control_node", namespace="right",
        parameters=[controllers], remappings=[("robot_description", "/right/robot_description")],
        output="screen",
    )
    robot2 = Node(
        package="controller_manager", executable="ros2_control_node", namespace="left",
        parameters=[controllers], remappings=[("robot_description", "/left/robot_description")],
        output="screen",
    )
    forward_spawners = [
        Node(
            package="controller_manager", executable="spawner", namespace=side,
            name="joint_controller_spawner", output="screen",
            arguments=["joint_state_broadcaster", "forward_position_controller",
                       "--controller-manager", f"/{side}/controller_manager",
                       "--controller-manager-timeout", "180", "--param-file", controllers],
        )
        for side in ("right", "left")
    ]
    interpolation = Node(
        package="dual_crx_control", executable="interpolation_node", output="screen",
        parameters=[{
            "input_rate_hz": ParameterValue(LaunchConfiguration("input_rate_hz"), value_type=float),
            "method": LaunchConfiguration("method"),
        }],
    )
    joint_state_merger = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        name="dual_crx_joint_state_merger", output="screen",
        parameters=[{"robot_description": robot_description,
                     "source_list": ["/left/joint_states", "/right/joint_states"],
                     "rate": 100, "publish_default_positions": False}],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="dual_crx_robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("dual_crx_control"), "rviz", "dual_cartesian.rviz",
        ])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [
        right_state_publisher,
        left_state_publisher,
        robot1,
        robot2,
        *forward_spawners,
        interpolation,
        joint_state_merger,
        robot_state_publisher,
        rviz,
    ]


def generate_launch_description():
    # Resolve mock/IP arguments before expanding driver Xacro descriptions.
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.2.100"),
        DeclareLaunchArgument("input_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("method", default_value="linear", choices=["linear", "cubic"]),
        OpaqueFunction(function=launch_setup),
    ])
