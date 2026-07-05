#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    max_known_free_marker_voxels = LaunchConfiguration("max_known_free_marker_voxels")
    max_known_occupied_marker_voxels = LaunchConfiguration("max_known_occupied_marker_voxels")
    max_unknown_marker_voxels = LaunchConfiguration("max_unknown_marker_voxels")
    package_share = FindPackageShare("mobile_manipulator_fkie_nbv")
    params = PathJoinSubstitution([package_share, "config", "fkie_nbv_planner.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rmw_implementation", default_value="rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("max_known_free_marker_voxels", default_value="5000"),
            DeclareLaunchArgument("max_known_occupied_marker_voxels", default_value="50000"),
            DeclareLaunchArgument("max_unknown_marker_voxels", default_value="10000"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            Node(
                package="mobile_manipulator_fkie_nbv",
                executable="fkie_nbv_planner_node",
                name="fkie_nbv_planner",
                output="screen",
                parameters=[
                    params,
                    {
                        "use_sim_time": use_sim_time,
                        "max_known_free_marker_voxels": max_known_free_marker_voxels,
                        "max_known_occupied_marker_voxels": max_known_occupied_marker_voxels,
                        "max_unknown_marker_voxels": max_unknown_marker_voxels,
                    },
                ],
            ),
        ]
    )
