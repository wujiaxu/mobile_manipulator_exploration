#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    
    voxel_bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("mobile_manipulator_moveit_bridge"),
            "config",
            "octomap_voxel_planning_scene.yaml",
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "mobile_manipulator",
            package_name="mobile_manipulator_moveit_config",
        )
        .robot_description(
            file_path="config/mobile_manipulator_gazebo.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/mobile_manipulator.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False, # Gazebo ros2_control manager will handle this
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }
    move_group_parameters = [
        moveit_config.to_dict(),
        sim_time_parameter,
        {
            "allow_trajectory_execution": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_robot_state_publisher", default_value="true"),
            
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description, sim_time_parameter],
                condition=IfCondition(use_robot_state_publisher),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            Node(
                package="mobile_manipulator_moveit_bridge",
                executable="pose_goal_planner",
                output="screen",
                parameters=[moveit_config.to_dict(), sim_time_parameter],
            ),
            Node(
                package="mobile_manipulator_moveit_bridge",
                executable="octomap_voxel_planning_scene_bridge",
                name="octomap_voxel_planning_scene_bridge",
                output="screen",
                parameters=[voxel_bridge_params, sim_time_parameter],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.planning_pipelines,
                    sim_time_parameter,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )