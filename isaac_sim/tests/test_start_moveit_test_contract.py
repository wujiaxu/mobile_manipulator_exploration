#!/usr/bin/env python3
import os
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "start_moveit_test.sh"


class StartMoveItTestContract(unittest.TestCase):
    def test_launcher_exists_and_is_executable(self):
        self.assertTrue(LAUNCHER.is_file(), f"Missing launcher: {LAUNCHER}")
        self.assertTrue(os.access(LAUNCHER, os.X_OK), "Launcher is not executable")

    def test_launcher_contains_required_process_contract(self):
        source = LAUNCHER.read_text(encoding="utf-8")
        for required in (
            "set -Eeuo pipefail",
            "run_mobile_manipulator.py",
            "moveit_isaac.launch.py",
            "rmw_fastrtps_cpp",
            "rmw_cyclonedds_cpp",
            "use_rviz:=true",
            "use_sim_time:=true",
            "setsid",
            "trap cleanup INT TERM EXIT",
            "--dry-run",
        ):
            self.assertIn(required, source)

    def test_dry_run_prints_both_commands_without_starting_processes(self):
        result = subprocess.run(
            [str(LAUNCHER), "--dry-run"],
            cwd="/tmp",
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Isaac command:", result.stdout)
        self.assertIn("MoveIt command:", result.stdout)
        self.assertIn("run_mobile_manipulator.py", result.stdout)
        self.assertIn("moveit_isaac.launch.py", result.stdout)


if __name__ == "__main__":
    unittest.main()
