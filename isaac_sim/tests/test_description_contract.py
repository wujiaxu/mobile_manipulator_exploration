#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
TRACER_XACRO = (
    WORKSPACE
    / "mobile_manipulator_description/urdf/tracer_base.urdf.xacro"
)
UNIFIED_XACRO = (
    WORKSPACE
    / "mobile_manipulator_description/urdf/mobile_manipulator.urdf.xacro"
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

    def test_unified_description_declares_wrist_camera_frames(self):
        source = UNIFIED_XACRO.read_text(encoding="utf-8")
        for required in (
            'name="wrist_camera_link"',
            'name="link_eef_to_wrist_camera"',
            '<parent link="link_eef"/>',
            '<child link="wrist_camera_link"/>',
            'name="wrist_camera_color_optical_frame"',
            'name="wrist_camera_link_to_color_optical_frame"',
            '<child link="wrist_camera_color_optical_frame"/>',
        ):
            self.assertIn(required, source)
        self.assertIn(
            '<joint name="wrist_camera_link_to_color_optical_frame" type="fixed">\n'
            '    <parent link="wrist_camera_link"/>\n'
            '    <child link="wrist_camera_color_optical_frame"/>\n'
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            source,
        )


if __name__ == "__main__":
    unittest.main()
