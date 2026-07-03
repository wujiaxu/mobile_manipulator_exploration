#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_share = FindPackageShare("mobile_manipulator_navigation")
    octomap_params = PathJoinSubstitution([nav_share, "config", "wrist_octomap.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="wrist_depth_to_points",
                output="screen",
                remappings=[
                    ("image_rect", "/wrist_camera/depth/image_rect_raw"),
                    ("camera_info", "/wrist_camera/color/camera_info"),
                    ("points", "/wrist_camera/depth/points"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="mobile_manipulator_navigation",
                executable="wrist_depth_octomap_node",
                name="wrist_depth_octomap",
                output="screen",
                parameters=[octomap_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
