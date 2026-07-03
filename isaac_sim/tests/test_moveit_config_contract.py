#!/usr/bin/env python3
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CONFIG_PACKAGE = WORKSPACE / "mobile_manipulator_moveit_config"
CONFIG_DIR = CONFIG_PACKAGE / "config"
BRIDGE_PACKAGE = WORKSPACE / "mobile_manipulator_moveit_bridge"
BRIDGE_CONFIG_DIR = BRIDGE_PACKAGE / "config"
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
EXPECTED_LIMITS = {
    "joint1": (-6.283185307179586, 6.283185307179586, 3.14),
    "joint2": (-2.059, 2.0944, 3.14),
    "joint3": (-6.283185307179586, 6.283185307179586, 3.14),
    "joint4": (-0.19198, 3.927, 3.14),
    "joint5": (-6.283185307179586, 6.283185307179586, 3.14),
    "joint6": (-1.69297, 3.141592653589793, 3.14),
    "joint7": (-6.283185307179586, 6.283185307179586, 3.14),
}


class MoveItConfigContractTest(unittest.TestCase):
    def load_xml(self, relative_path):
        path = CONFIG_PACKAGE / relative_path
        self.assertTrue(path.is_file(), f"Missing {path}")
        return ET.parse(path).getroot()

    def load_yaml(self, filename):
        path = CONFIG_DIR / filename
        self.assertTrue(path.is_file(), f"Missing {path}")
        with path.open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_required_configuration_files_exist(self):
        expected = (
            "mobile_manipulator.urdf.xacro",
            "mobile_manipulator.srdf",
            "kinematics.yaml",
            "joint_limits.yaml",
            "ompl_planning.yaml",
            "moveit_controllers.yaml",
            "initial_positions.yaml",
            "moveit.rviz",
        )
        for filename in expected:
            self.assertTrue((CONFIG_DIR / filename).is_file(), filename)

    def test_package_declares_moveit_and_description_dependencies(self):
        package = self.load_xml("package.xml")
        dependencies = {
            element.text
            for element in package
            if element.tag in {"depend", "exec_depend"}
        }
        expected = {
            "mobile_manipulator_description",
            "mobile_manipulator_moveit_bridge",
            "moveit_configs_utils",
            "moveit_kinematics",
            "moveit_planners_ompl",
            "moveit_ros_move_group",
            "moveit_ros_visualization",
            "moveit_simple_controller_manager",
            "robot_state_publisher",
            "rviz2",
            "xacro",
        }
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

    def test_description_wrapper_includes_unified_xacro(self):
        wrapper = CONFIG_DIR / "mobile_manipulator.urdf.xacro"
        self.assertTrue(wrapper.is_file(), wrapper)
        source = wrapper.read_text(encoding="utf-8")
        self.assertIn("mobile_manipulator_description", source)
        self.assertIn("mobile_manipulator.urdf.xacro", source)

    def test_srdf_defines_xarm7_chain_without_gripper_group(self):
        srdf = self.load_xml("config/mobile_manipulator.srdf")
        group = srdf.find("./group[@name='xarm7']")
        self.assertIsNotNone(group)
        chain = group.find("chain")
        self.assertEqual(chain.attrib["base_link"], "link_base")
        self.assertEqual(chain.attrib["tip_link"], "link_eef")
        self.assertIsNone(srdf.find("./end_effector"))

    def test_controller_uses_expected_action_and_joint_order(self):
        controllers = self.load_yaml("moveit_controllers.yaml")
        controller = controllers["moveit_simple_controller_manager"][
            "arm_controller"
        ]
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertEqual(controller["type"], "FollowJointTrajectory")
        self.assertEqual(controller["joints"], ARM_JOINTS)

    def test_kinematics_uses_kdl(self):
        kinematics = self.load_yaml("kinematics.yaml")
        self.assertEqual(
            kinematics["xarm7"]["kinematics_solver"],
            "kdl_kinematics_plugin/KDLKinematicsPlugin",
        )

    def test_joint_limits_match_unified_description(self):
        limits = self.load_yaml("joint_limits.yaml")["joint_limits"]
        self.assertEqual(list(limits), ARM_JOINTS)
        for joint_name, (lower, upper, velocity) in EXPECTED_LIMITS.items():
            joint = limits[joint_name]
            self.assertTrue(joint["has_position_limits"])
            self.assertEqual(joint["min_position"], lower)
            self.assertEqual(joint["max_position"], upper)
            self.assertTrue(joint["has_velocity_limits"])
            self.assertEqual(joint["max_velocity"], velocity)


class MoveItBridgeContractTest(unittest.TestCase):
    def test_bridge_manifest_declares_control_dependencies(self):
        package_path = BRIDGE_PACKAGE / "package.xml"
        self.assertTrue(package_path.is_file(), package_path)
        package = ET.parse(package_path).getroot()
        dependencies = {
            element.text
            for element in package
            if element.tag in {"depend", "exec_depend"}
        }
        expected = {
            "control_msgs",
            "rclcpp",
            "rclcpp_action",
            "sensor_msgs",
            "trajectory_msgs",
        }
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

    def test_trajectory_bridge_exposes_required_interfaces(self):
        source_path = BRIDGE_PACKAGE / "src/isaac_trajectory_bridge.cpp"
        self.assertTrue(source_path.is_file(), source_path)
        source = source_path.read_text(encoding="utf-8")
        expected_literals = (
            "/arm_controller/follow_joint_trajectory",
            "/arm_joint_commands",
            "/joint_states",
            "goal_position_tolerance",
            "path_position_tolerance",
            "goal_time_tolerance",
        )
        for literal in expected_literals:
            self.assertIn(literal, source)

        cmake = (BRIDGE_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(isaac_trajectory_bridge", cmake)
        self.assertRegex(
            cmake,
            re.compile(
                r"install\s*\(\s*TARGETS[^\)]*isaac_trajectory_bridge",
                re.DOTALL,
            ),
        )
        self.assertIn("RUNTIME DESTINATION lib/${PROJECT_NAME}", cmake)

    def test_pose_goal_planner_exposes_required_interfaces(self):
        source_path = BRIDGE_PACKAGE / "src/pose_goal_planner.cpp"
        self.assertTrue(source_path.is_file(), source_path)
        source = source_path.read_text(encoding="utf-8")
        for literal in ("/arm_target_pose", "xarm7", "link_eef", "base_link"):
            self.assertIn(literal, source)

        package = ET.parse(BRIDGE_PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in package
            if element.tag in {"depend", "exec_depend"}
        }
        expected = {
            "geometry_msgs",
            "moveit_ros_planning_interface",
            "tf2_geometry_msgs",
            "tf2_ros",
        }
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

        cmake = (BRIDGE_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(pose_goal_planner", cmake)

    def test_octomap_voxel_bridge_publishes_planning_scene_diff(self):
        source_path = BRIDGE_PACKAGE / "src/octomap_voxel_planning_scene_bridge.cpp"
        self.assertTrue(source_path.is_file(), source_path)
        source = source_path.read_text(encoding="utf-8")
        for literal in (
            "octomap_voxel_planning_scene_bridge",
            "sensor_msgs/msg/point_cloud2.hpp",
            "moveit_msgs/msg/planning_scene.hpp",
            "moveit_msgs/srv/apply_planning_scene.hpp",
            "shape_msgs/msg/solid_primitive.hpp",
            "/octomap_occupied_points",
            "/planning_scene",
            "/apply_planning_scene",
            "planning_frame_",
            "source_frame",
            "apply_planning_scene_service_",
            "workspace_min_x",
            "workspace_max_x",
            "workspace_min_y",
            "workspace_max_y",
            "workspace_min_z",
            "workspace_max_z",
            "voxel_box_size",
            "max_boxes",
            "nbv_octomap_occupied_voxels",
            "CollisionObject::ADD",
            "scene.is_diff = true",
        ):
            self.assertIn(literal, source)

        config_path = BRIDGE_CONFIG_DIR / "octomap_voxel_planning_scene.yaml"
        self.assertTrue(config_path.is_file(), config_path)
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "octomap_voxel_planning_scene_bridge"
        ]["ros__parameters"]
        self.assertEqual(params["occupied_cloud_topic"], "/octomap_occupied_points")
        self.assertEqual(params["planning_scene_topic"], "/planning_scene")
        self.assertEqual(params["apply_planning_scene_service"], "/apply_planning_scene")
        self.assertEqual(params["planning_frame"], "base_link")
        self.assertEqual(params["workspace_min_x"], -0.5)
        self.assertEqual(params["workspace_max_x"], 1.2)
        self.assertEqual(params["workspace_min_y"], -0.8)
        self.assertEqual(params["workspace_max_y"], 0.8)
        self.assertEqual(params["workspace_min_z"], 0.0)
        self.assertEqual(params["workspace_max_z"], 1.8)
        self.assertEqual(params["voxel_box_size"], 0.05)
        self.assertGreater(params["max_boxes"], 0)

        package = ET.parse(BRIDGE_PACKAGE / "package.xml").getroot()
        dependencies = {
            element.text
            for element in package
            if element.tag in {"depend", "exec_depend"}
        }
        expected = {"moveit_msgs", "shape_msgs"}
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

        cmake = (BRIDGE_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(octomap_voxel_planning_scene_bridge", cmake)
        self.assertIn("install(DIRECTORY config", cmake)


class MoveItLaunchContractTest(unittest.TestCase):
    def test_integrated_launch_contains_required_nodes_and_arguments(self):
        launch_path = CONFIG_PACKAGE / "launch/moveit_isaac.launch.py"
        self.assertTrue(launch_path.is_file(), launch_path)
        source = launch_path.read_text(encoding="utf-8")
        for literal in (
            "use_sim_time",
            "use_rviz",
            "robot_state_publisher",
            "move_group",
            "isaac_trajectory_bridge",
            "pose_goal_planner",
            "octomap_voxel_planning_scene_bridge",
            "octomap_voxel_planning_scene.yaml",
            "rviz2",
        ):
            self.assertIn(literal, source)


if __name__ == "__main__":
    unittest.main()
