#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 1. Build MoveIt Config to extract the master unified robot description xacro
    moveit_config = (
        MoveItConfigsBuilder("mobile_manipulator", package_name="mobile_manipulator_moveit_config")
        .robot_description(file_path="config/mobile_manipulator_gazebo.urdf.xacro")
        .to_moveit_configs()
    )

    # 2. Master Gazebo Simulator Starter
    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        gazebo_simulator,

        # 3. Clock Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            output='screen'
        ),

        # 4. State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        ),

        # 5. Gazebo Robot Spawner
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", "mobile_manipulator",
                "-z", "0.1" 
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # ==================== CONTROLLER SPAWNERS ====================
        
        # 6. Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", 
                "--controller-manager", "/controller_manager", 
                "--controller-manager-timeout", "60"
            ],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # 7. Base Controller Spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "base_controller", 
                "--controller-manager", "/controller_manager", 
                "--controller-manager-timeout", "60"
            ],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # 8. Arm Controller Spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_controller", 
                "--controller-manager", "/controller_manager", 
                "--controller-manager-timeout", "60"
            ],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])