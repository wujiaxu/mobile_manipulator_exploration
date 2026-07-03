#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
NAV_PACKAGE = ROOT / "mobile_manipulator_navigation"
CONFIG_DIR = NAV_PACKAGE / "config"
LAUNCH_DIR = NAV_PACKAGE / "launch"
SRC = NAV_PACKAGE / "src/wrist_depth_octomap_node.cpp"


class WristOctomapContractTest(unittest.TestCase):
    def test_manifest_and_cmake_declare_octomap_node_dependencies(self):
        package = ET.parse(NAV_PACKAGE / "package.xml").getroot()
        dependencies = {
            node.text
            for node in package
            if node.tag in {"depend", "exec_depend", "build_depend"}
        }
        expected = {
            "geometry_msgs",
            "octomap",
            "octomap_msgs",
            "rclcpp",
            "sensor_msgs",
            "tf2_ros",
            "tf2_sensor_msgs",
        }
        self.assertTrue(expected.issubset(dependencies), expected - dependencies)

        cmake = (NAV_PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(wrist_depth_octomap_node", cmake)
        self.assertIn("RUNTIME DESTINATION lib/${PROJECT_NAME}", cmake)

    def test_octomap_config_uses_map_frame_and_wrist_point_cloud(self):
        config_path = CONFIG_DIR / "wrist_octomap.yaml"
        self.assertTrue(config_path.is_file(), config_path)
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "wrist_depth_octomap"
        ]["ros__parameters"]
        self.assertEqual(params["map_frame"], "map")
        self.assertEqual(params["cloud_topic"], "/wrist_camera/depth/points")
        self.assertEqual(params["octomap_topic"], "/octomap_binary")
        self.assertEqual(params["occupied_cloud_topic"], "/octomap_occupied_points")
        self.assertEqual(params["resolution"], 0.05)
        self.assertGreater(params["max_range"], 0.0)

    def test_octomap_node_transforms_clouds_into_map_and_publishes_rviz_cloud(self):
        self.assertTrue(SRC.is_file(), SRC)
        source = SRC.read_text(encoding="utf-8")
        for required in (
            "wrist_depth_octomap",
            "sensor_msgs/msg/point_cloud2.hpp",
            "sensor_msgs/point_cloud2_iterator.hpp",
            "octomap/OcTree.h",
            "octomap_msgs/conversions.h",
            "tf2_ros/transform_listener.h",
            "lookupTransform",
            "map_frame_",
            "cloud_topic_",
            "octomap_topic_",
            "occupied_cloud_topic_",
            "binaryMapToMsg",
            "insertPointCloud",
            "begin_leafs",
            "isNodeOccupied",
            "occupied_cloud_publisher_",
        ):
            self.assertIn(required, source)

    def test_launch_converts_wrist_depth_to_points_and_starts_map_frame_octomap(self):
        launch_path = LAUNCH_DIR / "wrist_octomap.launch.py"
        self.assertTrue(launch_path.is_file(), launch_path)
        source = launch_path.read_text(encoding="utf-8")
        for required in (
            "depth_image_proc",
            "point_cloud_xyz_node",
            "/wrist_camera/depth/image_rect_raw",
            "/wrist_camera/color/camera_info",
            "/wrist_camera/depth/points",
            "wrist_depth_octomap_node",
            "wrist_octomap.yaml",
            "use_sim_time",
        ):
            self.assertIn(required, source)
        self.assertNotIn("moveit_ros_occupancy_map_server", source)

    def test_navigation_rviz_shows_wrist_depth_and_octomap_voxels(self):
        rviz_path = CONFIG_DIR / "navigation.rviz"
        self.assertTrue(rviz_path.is_file(), rviz_path)
        source = rviz_path.read_text(encoding="utf-8")
        for required in (
            "Wrist Depth Image",
            "rviz_default_plugins/Image",
            "/wrist_camera/depth/image_rect_raw",
            "Wrist Depth Points",
            "/wrist_camera/depth/points",
            "Wrist OctoMap Occupied Voxels",
            "/octomap_occupied_points",
            "Color Transformer: AxisColor",
            "Axis: Z",
        ):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
