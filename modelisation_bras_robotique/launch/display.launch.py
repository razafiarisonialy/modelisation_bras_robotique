from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("modelisation_bras_robotique")
    default_model = PathJoinSubstitution([pkg_share, "urdf", "arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "view_robot.rviz"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", LaunchConfiguration("model")]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Absolute path to the robot Xacro model.",
        ),
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for interactive joint control.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_gui")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
