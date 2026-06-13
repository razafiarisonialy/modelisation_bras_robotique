import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def load_text(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def generate_launch_description():
    description_pkg = "modelisation_bras_robotique"
    moveit_pkg = "modelisation_bras_robotique_moveit_config"

    xacro_file = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        "urdf",
        "arm.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_text(moveit_pkg, "config/bras_robotique.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_pkg, "config/kinematics.yaml")
    }

    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml(moveit_pkg, "config/ompl_planning.yaml"),
    }

    moveit_controllers = load_yaml(moveit_pkg, "config/moveit_controllers.yaml")
    trajectory_execution = load_yaml(moveit_pkg, "config/trajectory_execution.yaml")
    joint_limits = {
        "robot_description_planning": load_yaml(moveit_pkg, "config/joint_limits.yaml")
    }

    rviz_config = os.path.join(
        get_package_share_directory(moveit_pkg),
        "config",
        "moveit.rviz",
    )

    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        planning_pipeline,
        moveit_controllers,
        trajectory_execution,
        joint_limits,
        {"planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/attached_collision_object",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        }},
    ]

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common_parameters,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            arguments=["-d", rviz_config],
            output="screen",
            parameters=common_parameters,
        ),
    ])
