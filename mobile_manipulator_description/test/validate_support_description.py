#!/usr/bin/env python3
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


class SupportDescriptionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        package_share = get_package_share_directory(
            "mobile_manipulator_description"
        )
        xacro_file = f"{package_share}/urdf/mobile_manipulator.urdf.xacro"
        with tempfile.NamedTemporaryFile(suffix=".urdf") as output:
            subprocess.run(
                ["xacro", xacro_file, "-o", output.name],
                check=True,
            )
            cls.robot = ET.parse(output.name).getroot()

    def test_support_has_visual_and_collision_box(self):
        support = self.robot.find("./link[@name='arm_support_link']")
        self.assertIsNotNone(support)
        expected_size = "0.3 0.3 0.49692302"
        self.assertEqual(
            support.find("./visual/geometry/box").attrib["size"], expected_size
        )
        self.assertEqual(
            support.find("./collision/geometry/box").attrib["size"], expected_size
        )

    def test_support_is_fixed_at_the_arm_mount(self):
        joint = self.robot.find("./joint[@name='frame_base_to_arm_support']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.attrib["type"], "fixed")
        self.assertEqual(joint.find("parent").attrib["link"], "frame_base")
        self.assertEqual(joint.find("child").attrib["link"], "arm_support_link")
        self.assertEqual(
            joint.find("origin").attrib,
            {"rpy": "0 0 0", "xyz": "-0.076 0 0.27053849"},
        )

    def test_support_has_expected_mass_and_box_inertia(self):
        support = self.robot.find("./link[@name='arm_support_link']")
        self.assertIsNotNone(support)
        inertial = support.find("inertial")
        self.assertEqual(inertial.find("mass").attrib["value"], "10.0")
        self.assertEqual(
            inertial.find("inertia").attrib,
            {
                "ixx": "0.2807770732",
                "ixy": "0",
                "ixz": "0",
                "iyy": "0.2807770732",
                "iyz": "0",
                "izz": "0.15",
            },
        )

    def test_arm_calibration_is_unchanged(self):
        joint = self.robot.find("./joint[@name='frame_base_to_frame_1']")
        self.assertEqual(
            joint.find("origin").attrib,
            {"rpy": "0 0 0", "xyz": "-0.076 0 0.519"},
        )


if __name__ == "__main__":
    unittest.main()
