#!/usr/bin/env python3
import importlib.util
import os
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ISAAC_DIR = Path(__file__).resolve().parents[1]
RUNNER = ISAAC_DIR / "scripts/run_factory_navigation.py"
IMPORTER = ISAAC_DIR / "scripts/import_mobile_manipulator.py"
ROOT = ISAAC_DIR.parent
NAV_PACKAGE = ROOT / "mobile_manipulator_navigation"
LAUNCHER = ROOT / "start_navigation_test.sh"


class FactoryNavigationRunnerContract(unittest.TestCase):
    def test_runner_exists(self):
        self.assertTrue(RUNNER.is_file(), f"Missing navigation runner: {RUNNER}")

    def test_runner_arguments(self):
        spec = importlib.util.spec_from_file_location("run_factory_navigation", RUNNER)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = module.parse_args(["--headless", "--duration", "5"])
        self.assertTrue(args.headless)
        self.assertEqual(args.duration, 5.0)
        self.assertFalse(hasattr(args, "disable_lidar"))
        self.assertTrue(str(args.factory_usd).endswith("compact_factory.usd"))
        self.assertTrue(str(args.robot_usd).endswith("mobile_manipulator_ros.usd"))

    def test_importer_bakes_lidar_and_scan_publisher_into_robot_usd(self):
        source = IMPORTER.read_text(encoding="utf-8")
        for required in (
            '"/mobile_manipulator/livox_frame/NavLidar"',
            '"MobileManipulator_Nav2D"',
            '"isaacsim.ros2.bridge.ROS2RtxLidarHelper"',
            '"isaacsim.ros2.bridge.ROS2CameraHelper"',
            '"isaacsim.ros2.bridge.ROS2CameraInfoHelper"',
            '"isaacsim.core.nodes.IsaacCreateRenderProduct"',
            '("CreateLidarRenderProduct.outputs:execOut", "ScanPublisher.inputs:execIn")',
            (
                '"CreateLidarRenderProduct.outputs:renderProductPath",\n'
                '                    "ScanPublisher.inputs:renderProductPath"'
            ),
            (
                '"CreateLidarRenderProduct.inputs:cameraPrim",\n'
                '                    [usdrt.Sdf.Path("/mobile_manipulator/livox_frame/NavLidar")]'
            ),
            '("ScanPublisher.inputs:topicName", "scan")',
            '("ScanPublisher.inputs:frameId", "livox_frame")',
            '("ScanPublisher.inputs:type", "laser_scan")',
            '("ScanPublisher.inputs:useSystemTime", False)',
        ):
            self.assertIn(required, source)
        self.assertNotIn("get_render_product_path()", source)
        self.assertNotIn('"ScanPublisher.inputs:renderProductPath",\n                    lidar_', source)

    def test_runner_composes_factory_and_requires_baked_lidar(self):
        source = RUNNER.read_text(encoding="utf-8")
        for required in (
            '"/Factory"',
            '"/mobile_manipulator/livox_frame/NavLidar"',
            '"/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera"',
            '"/mobile_manipulator/ActionGraph"',
            '"/ActionGraph"',
            '"ScanPublisher"',
            '"WristRgbPublisher"',
            '"WristDepthPublisher"',
            '"WristCameraInfoPublisher"',
            "/wrist_camera/color/image_raw",
            "/wrist_camera/depth/image_rect_raw",
            "/wrist_camera/color/camera_info",
            '"/app/sensors/nv/lidar/profileBaseFolder"',
            "ROBOT_SPAWN",
        ):
            self.assertIn(required, source)
        self.assertIn("new_stage", source)
        self.assertIn("subLayerPaths", source)
        self.assertLess(source.index("str(factory_usd)"), source.index("str(robot_usd)"))
        self.assertNotIn("open_stage(str(robot_usd))", source)
        self.assertNotIn("disable_lidar", source)
        self.assertNotIn("LidarRtx", source)
        self.assertNotIn("ROS2RtxLidarHelper", source)
        self.assertNotIn("og.Controller.edit", source)

    def test_runner_uses_the_imported_action_graph(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn('("/mobile_manipulator/ActionGraph", "/ActionGraph")', source)
        self.assertIn("og.Controller.graph(candidate)", source)
        self.assertIn('node_path = f"{graph_path}/{node_name}"', source)
        self.assertIn("Sdf.Path(node_path)", source)
        self.assertNotIn("og.Controller.edit(", source)
        self.assertNotIn(
            'og.Controller.edit(\n            {"graph_path": "/ActionGraph"',
            source,
        )
        self.assertNotIn(
            '("OnPlaybackTick.outputs:tick", "ScanPublisher.inputs:execIn")',
            source,
        )


class NavigationPackageContract(unittest.TestCase):
    def setUp(self):
        self.package_xml = NAV_PACKAGE / "package.xml"
        self.slam_path = NAV_PACKAGE / "config/slam_toolbox.yaml"
        self.nav2_path = NAV_PACKAGE / "config/nav2_params.yaml"
        self.launch_path = NAV_PACKAGE / "launch/mapping.launch.py"

    def test_required_files_exist(self):
        for path in (
            NAV_PACKAGE / "CMakeLists.txt",
            self.package_xml,
            self.slam_path,
            self.nav2_path,
            NAV_PACKAGE / "config/navigation.rviz",
            self.launch_path,
        ):
            self.assertTrue(path.is_file(), str(path))

    def test_manifest_declares_navigation_dependencies(self):
        root = ET.parse(self.package_xml).getroot()
        dependencies = {node.text for node in root.findall("exec_depend")}
        self.assertTrue(
            {
                "mobile_manipulator_description",
                "nav2_bringup",
                "robot_state_publisher",
                "rviz2",
                "slam_toolbox",
                "xacro",
            }.issubset(dependencies)
        )

    def test_slam_frames_and_scan_topic(self):
        params = yaml.safe_load(self.slam_path.read_text())["slam_toolbox"][
            "ros__parameters"
        ]
        self.assertTrue(params["use_sim_time"])
        self.assertEqual(params["map_frame"], "map")
        self.assertEqual(params["odom_frame"], "odom")
        self.assertEqual(params["base_frame"], "base_link")
        self.assertEqual(params["scan_topic"], "/scan")
        self.assertEqual(params["resolution"], 0.05)
        self.assertGreaterEqual(params["scan_buffer_size"], 10)
        self.assertGreaterEqual(params["throttle_scans"], 1)
        self.assertGreater(params["transform_timeout"], 0.0)

    def test_nav2_costmaps_and_speed_contract(self):
        config = yaml.safe_load(self.nav2_path.read_text())
        controller = config["controller_server"]["ros__parameters"]
        self.assertLessEqual(controller["FollowPath"]["max_vel_x"], 0.35)
        self.assertLessEqual(controller["FollowPath"]["max_vel_theta"], 0.8)
        for server_name in ("global_costmap", "local_costmap"):
            params = config[server_name][server_name]["ros__parameters"]
            self.assertEqual(params["robot_base_frame"], "base_link")
            self.assertIn("obstacle_layer", params["plugins"])
            self.assertIn("inflation_layer", params["plugins"])
            self.assertEqual(params["obstacle_layer"]["scan"]["topic"], "/scan")
            self.assertEqual(
                params["obstacle_layer"]["scan"]["data_type"], "LaserScan"
            )
            self.assertIn("footprint", params)
        local_params = config["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertIs(type(local_params["width"]), int)
        self.assertIs(type(local_params["height"]), int)

    def test_mapping_launch_has_one_description_and_no_localization(self):
        source = self.launch_path.read_text(encoding="utf-8")
        for required in (
            "robot_state_publisher",
            "async_slam_toolbox_node",
            "slam_toolbox.yaml",
            "navigation_launch.py",
            "use_sim_time",
            "use_rviz",
            "navigation.rviz",
        ):
            self.assertIn(required, source)
        self.assertNotIn("mapper_params_online_async.yaml", source)
        self.assertNotIn("online_async_launch.py", source)
        for forbidden in ("amcl", "map_server", "joint_state_publisher", "move_group"):
            self.assertNotIn(forbidden, source)


class NavigationLauncherContract(unittest.TestCase):
    def test_launcher_exists_and_is_executable(self):
        self.assertTrue(LAUNCHER.is_file(), str(LAUNCHER))
        self.assertTrue(os.access(LAUNCHER, os.X_OK))

    def test_launcher_supervises_both_middleware_processes(self):
        source = LAUNCHER.read_text(encoding="utf-8")
        for required in (
            "--dry-run",
            "--use-wrist-octomap",
            "rmw_fastrtps_cpp",
            "rmw_cyclonedds_cpp",
            "run_factory_navigation.py",
            "mapping.launch.py",
            "wrist_octomap.launch.py",
            "setsid",
            "trap cleanup INT TERM EXIT",
        ):
            self.assertIn(required, source)

    def test_launcher_dry_run(self):
        result = subprocess.run(
            [str(LAUNCHER), "--dry-run"],
            cwd="/tmp",
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Isaac navigation command:", result.stdout)
        self.assertIn("ROS navigation command:", result.stdout)


if __name__ == "__main__":
    unittest.main()
