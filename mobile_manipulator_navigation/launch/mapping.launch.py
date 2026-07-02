#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    nav_share = FindPackageShare("mobile_manipulator_navigation")
    description_share = FindPackageShare("mobile_manipulator_description")
    nav2_params = PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"])
    slam_params = PathJoinSubstitution([nav_share, "config", "slam_toolbox.yaml"])
    rviz_config = PathJoinSubstitution([nav_share, "config", "navigation.rviz"])
    robot_xacro = PathJoinSubstitution(
        [description_share, "urdf", "mobile_manipulator.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_xacro]), value_type=str
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_robot_state_publisher", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_robot_state_publisher),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
