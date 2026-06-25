#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
TRACER_XACRO = (
    WORKSPACE
    / "mobile_manipulator_description/urdf/tracer_base.urdf.xacro"
)
IMPORT_URDF = WORKSPACE / "build/isaac_import/mobile_manipulator.urdf"


class DriveAxisContractTest(unittest.TestCase):
    def assert_drive_axes(self, robot):
        right = robot.find(".//joint[@name='right_wheel']/axis")
        left = robot.find(".//joint[@name='left_wheel']/axis")
        self.assertEqual(right.attrib["xyz"], "0 1 0")
        self.assertEqual(left.attrib["xyz"], "0 -1 0")

    def test_package_source_uses_consistent_world_drive_axes(self):
        self.assert_drive_axes(ET.parse(TRACER_XACRO).getroot())

    def test_generated_import_urdf_matches_package_source(self):
        self.assert_drive_axes(ET.parse(IMPORT_URDF).getroot())


if __name__ == "__main__":
    unittest.main()
