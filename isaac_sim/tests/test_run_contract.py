#!/usr/bin/env python3
import importlib.util
import sys
import unittest
from pathlib import Path


RUNNER = Path(__file__).resolve().parents[1] / "scripts/run_mobile_manipulator.py"
FACTORY_RUNNER = Path(__file__).resolve().parents[1] / "scripts/run_factory_navigation.py"
SCRIPT_DIR = RUNNER.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location("run_mobile_manipulator", RUNNER)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class RunnerArgumentTest(unittest.TestCase):
    def test_pose_report_period_defaults_to_zero(self):
        args = module.parse_args([])
        self.assertEqual(args.pose_report_period, 0.0)

    def test_pose_report_period_accepts_positive_seconds(self):
        args = module.parse_args(["--pose-report-period", "1.5"])
        self.assertEqual(args.pose_report_period, 1.5)

    def test_pose_report_period_rejects_negative_seconds(self):
        with self.assertRaises(SystemExit):
            module.parse_args(["--pose-report-period", "-1"])

    def test_control_is_enabled_by_default(self):
        args = module.parse_args([])
        self.assertFalse(args.disable_control)

    def test_control_can_be_disabled_for_diagnostics(self):
        args = module.parse_args(["--disable-control"])
        self.assertTrue(args.disable_control)

    def test_action_graph_is_enabled_by_default(self):
        args = module.parse_args([])
        self.assertFalse(args.disable_action_graph)

    def test_action_graph_can_be_disabled_for_diagnostics(self):
        args = module.parse_args(["--disable-action-graph"])
        self.assertTrue(args.disable_action_graph)

    def test_pose_reporting_reads_the_physx_root_body(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertNotIn("SingleArticulation", source)
        self.assertNotIn("get_world_pose", source)
        self.assertIn("get_articulation_root_body", source)
        self.assertIn("get_rigid_body_pose", source)

    def test_standalone_runner_spawns_the_articulation_root_link(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn('stage.GetPrimAtPath("/mobile_manipulator/base_link")', source)
        self.assertIn("base_xform.SetTranslate", source)

    def test_factory_runner_spawns_the_articulation_root_link(self):
        source = FACTORY_RUNNER.read_text(encoding="utf-8")
        self.assertIn('stage.GetPrimAtPath(Sdf.Path("/mobile_manipulator/base_link"))', source)
        self.assertIn("base_xform.SetTranslate", source)
        self.assertIn(
            "robot_xform.SetTranslate((ROBOT_SPAWN[0], ROBOT_SPAWN[1], 0.0))",
            source,
        )
        self.assertIn(
            "base_xform.SetTranslate((0.0, 0.0, ROBOT_SPAWN[2]))",
            source,
        )
        self.assertIn("get_articulation_root_body", source)
        self.assertIn("set_rigid_body_pose", source)
        self.assertIn("set_rigid_body_linear_velocity", source)
        self.assertIn("set_rigid_body_angular_velocity", source)
        self.assertNotIn("getattr(args, \"robot_spawn\"", source)
        self.assertNotIn("ROBOT_SPAWN = getattr", source)


if __name__ == "__main__":
    unittest.main()
