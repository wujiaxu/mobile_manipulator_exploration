#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

ASSET_DIR = Path(__file__).resolve().parents[1] / "assets/robots/mobile_manipulator"
BASE_USD = ASSET_DIR / "mobile_manipulator.usd"
ROS_USD = ASSET_DIR / "mobile_manipulator_ros.usd"
ROBOT_PATH = "/mobile_manipulator"


class GeneratedAssetTest(unittest.TestCase):
    def test_imported_assets_exist(self):
        self.assertTrue(BASE_USD.is_file(), f"Missing imported asset: {BASE_USD}")
        self.assertTrue(ROS_USD.is_file(), f"Missing ROS overlay: {ROS_USD}")


@unittest.skipUnless(BASE_USD.is_file() and ROS_USD.is_file(), "USD assets not generated")
class ImportedRobotContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from isaacsim import SimulationApp

        cls.simulation_app = SimulationApp({"headless": True})
        global Usd, UsdGeom, UsdPhysics
        from pxr import Usd, UsdGeom, UsdPhysics

        cls.stage = Usd.Stage.Open(str(BASE_USD))
        if cls.stage is None:
            raise AssertionError(f"Could not open imported asset: {BASE_USD}")

    @classmethod
    def tearDownClass(cls):
        cls.simulation_app.close()

    def ros_stage(self):
        stage = Usd.Stage.Open(str(ROS_USD))
        self.assertIsNotNone(stage)
        return stage

    def assert_input(self, prim, name, expected):
        attribute = prim.GetAttribute(f"inputs:{name}")
        self.assertTrue(attribute.IsValid(), f"Missing inputs:{name}")
        actual = attribute.Get()
        if isinstance(expected, list):
            actual = list(actual)
        self.assertEqual(actual, expected)

    def assert_connected_from(self, stage, destination, source):
        attribute = stage.GetAttributeAtPath(destination)
        self.assertTrue(attribute.IsValid(), destination)
        self.assertIn(source, [str(path) for path in attribute.GetConnections()])

    def assert_target(self, prim, name, expected):
        relationship = prim.GetRelationship(f"inputs:{name}")
        self.assertTrue(relationship.IsValid(), f"Missing inputs:{name}")
        self.assertEqual([str(path) for path in relationship.GetTargets()], expected)

    def test_robot_is_a_movable_meter_scale_articulation(self):
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)
        robot = self.stage.GetPrimAtPath(ROBOT_PATH)
        self.assertTrue(robot.IsValid())
        articulation_roots = [
            prim
            for prim in Usd.PrimRange(robot)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        self.assertEqual(len(articulation_roots), 1)
        self.assertFalse(self.stage.GetPrimAtPath(f"{ROBOT_PATH}/root_joint").IsValid())

    def test_calibration_and_support_links_are_preserved(self):
        for link_name in (
            "base_link",
            "frame_base",
            "frame_1",
            "link_base",
            "livox_frame",
            "arm_support_link",
        ):
            self.assertTrue(
                self.stage.GetPrimAtPath(f"{ROBOT_PATH}/{link_name}").IsValid(),
                link_name,
            )

    def test_arm_support_has_mass_and_collision(self):
        support = self.stage.GetPrimAtPath(f"{ROBOT_PATH}/arm_support_link")
        self.assertAlmostEqual(support.GetAttribute("physics:mass").Get(), 10.0)
        colliders = [
            prim
            for prim in Usd.PrimRange(support)
            if prim.HasAPI(UsdPhysics.CollisionAPI)
        ]
        self.assertTrue(colliders)

    def test_expected_movable_joints_exist(self):
        joint_names = {
            "right_wheel",
            "left_wheel",
            "fl_castor_wheel",
            "fr_castor_wheel",
            "rr_castor_wheel",
            "rl_castor_wheel",
            "fl_wheel",
            "fr_wheel",
            "rr_wheel",
            "rl_wheel",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        }
        for joint_name in joint_names:
            self.assertTrue(
                self.stage.GetPrimAtPath(
                    f"{ROBOT_PATH}/joints/{joint_name}"
                ).IsValid(),
                joint_name,
            )

    def test_drive_wheels_use_velocity_drive_gains(self):
        for joint_name in ("left_wheel", "right_wheel"):
            joint = self.stage.GetPrimAtPath(
                f"{ROBOT_PATH}/joints/{joint_name}"
            )
            drive = UsdPhysics.DriveAPI.Get(joint, "angular")
            self.assertTrue(drive)
            self.assertEqual(drive.GetStiffnessAttr().Get(), 0.0)
            self.assertEqual(drive.GetDampingAttr().Get(), 10000.0)

    def test_ros_overlay_references_base_and_has_publishers(self):
        stage = self.ros_stage()
        self.assertTrue(stage.GetPrimAtPath("/ActionGraph/PublishJointState").IsValid())
        self.assertTrue(stage.GetPrimAtPath("/ActionGraph/PublishClock").IsValid())
        self.assertTrue(stage.GetPrimAtPath(ROBOT_PATH).IsValid())

    def test_ros_overlay_preserves_base_stage_metrics(self):
        stage = self.ros_stage()
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    def test_ros_overlay_has_cmd_vel_control(self):
        stage = self.ros_stage()
        for name in (
            "SubscribeTwist",
            "BreakLinearVelocity",
            "BreakAngularVelocity",
            "DifferentialController",
            "BaseArticulationController",
        ):
            self.assertTrue(
                stage.GetPrimAtPath(f"/ActionGraph/{name}").IsValid(), name
            )

        subscriber = stage.GetPrimAtPath("/ActionGraph/SubscribeTwist")
        controller = stage.GetPrimAtPath("/ActionGraph/DifferentialController")
        base_controller = stage.GetPrimAtPath(
            "/ActionGraph/BaseArticulationController"
        )
        self.assert_input(subscriber, "topicName", "cmd_vel")
        self.assert_input(controller, "wheelRadius", 0.0605)
        self.assert_input(controller, "wheelDistance", 0.34)
        self.assert_input(
            base_controller, "jointNames", ["left_wheel", "right_wheel"]
        )

        connections = (
            (
                "/ActionGraph/BreakLinearVelocity.inputs:tuple",
                "/ActionGraph/SubscribeTwist.outputs:linearVelocity",
            ),
            (
                "/ActionGraph/BreakAngularVelocity.inputs:tuple",
                "/ActionGraph/SubscribeTwist.outputs:angularVelocity",
            ),
            (
                "/ActionGraph/DifferentialController.inputs:linearVelocity",
                "/ActionGraph/BreakLinearVelocity.outputs:x",
            ),
            (
                "/ActionGraph/DifferentialController.inputs:angularVelocity",
                "/ActionGraph/BreakAngularVelocity.outputs:z",
            ),
            (
                "/ActionGraph/BaseArticulationController.inputs:velocityCommand",
                "/ActionGraph/DifferentialController.outputs:velocityCommand",
            ),
        )
        for destination, source in connections:
            self.assert_connected_from(stage, destination, source)
        self.assert_connected_from(
            stage,
            "/ActionGraph/BaseArticulationController.inputs:execIn",
            "/ActionGraph/OnPlaybackTick.outputs:tick",
        )

    def test_ros_overlay_has_arm_position_control(self):
        stage = self.ros_stage()
        subscriber = stage.GetPrimAtPath("/ActionGraph/SubscribeArmJointState")
        controller = stage.GetPrimAtPath("/ActionGraph/ArmArticulationController")
        self.assertTrue(subscriber.IsValid())
        self.assertTrue(controller.IsValid())
        self.assert_input(subscriber, "topicName", "arm_joint_commands")
        self.assert_connected_from(
            stage,
            "/ActionGraph/ArmArticulationController.inputs:jointNames",
            "/ActionGraph/SubscribeArmJointState.outputs:jointNames",
        )
        self.assert_connected_from(
            stage,
            "/ActionGraph/ArmArticulationController.inputs:positionCommand",
            "/ActionGraph/SubscribeArmJointState.outputs:positionCommand",
        )
        self.assert_connected_from(
            stage,
            "/ActionGraph/ArmArticulationController.inputs:execIn",
            "/ActionGraph/SubscribeArmJointState.outputs:execOut",
        )

    def test_ros_overlay_has_wrist_rgbd_camera_publishers(self):
        stage = self.ros_stage()
        camera = stage.GetPrimAtPath(
            f"{ROBOT_PATH}/wrist_camera_color_optical_frame/D455Camera"
        )
        self.assertTrue(camera.IsValid())
        rotate = camera.GetAttribute("xformOp:rotateXYZ")
        self.assertTrue(rotate.IsValid())
        self.assertEqual(tuple(rotate.Get()), (180.0, 0.0, 0.0))
        focal_length = camera.GetAttribute("focalLength").Get()
        horizontal_aperture = camera.GetAttribute("horizontalAperture").Get()
        vertical_aperture = camera.GetAttribute("verticalAperture").Get()
        horizontal_offset = camera.GetAttribute("horizontalApertureOffset").Get()
        vertical_offset = camera.GetAttribute("verticalApertureOffset").Get()
        self.assertAlmostEqual(focal_length, 2.0, places=6)
        self.assertAlmostEqual(horizontal_aperture, 848.0 * 2.0 / 429.0, places=6)
        self.assertAlmostEqual(vertical_aperture, 480.0 * 2.0 / 427.0, places=6)
        self.assertAlmostEqual(horizontal_offset, (425.0 - 848.0 / 2.0) / 429.0, places=6)
        self.assertAlmostEqual(vertical_offset, 0.0, places=6)
        for name in (
            "CreateWristCameraRenderProduct",
            "WristRgbPublisher",
            "WristDepthPublisher",
            "WristCameraInfoPublisher",
        ):
            self.assertTrue(stage.GetPrimAtPath(f"/ActionGraph/{name}").IsValid(), name)

        render_product = stage.GetPrimAtPath("/ActionGraph/CreateWristCameraRenderProduct")
        rgb = stage.GetPrimAtPath("/ActionGraph/WristRgbPublisher")
        depth = stage.GetPrimAtPath("/ActionGraph/WristDepthPublisher")
        info = stage.GetPrimAtPath("/ActionGraph/WristCameraInfoPublisher")
        self.assert_target(
            render_product,
            "cameraPrim",
            [f"{ROBOT_PATH}/wrist_camera_color_optical_frame/D455Camera"],
        )
        self.assert_input(render_product, "width", 848)
        self.assert_input(render_product, "height", 480)
        self.assert_input(rgb, "topicName", "wrist_camera/color/image_raw")
        self.assert_input(rgb, "frameId", "wrist_camera_color_optical_frame")
        self.assert_input(rgb, "type", "rgb")
        self.assert_input(depth, "topicName", "wrist_camera/depth/image_rect_raw")
        self.assert_input(depth, "frameId", "wrist_camera_color_optical_frame")
        self.assert_input(depth, "type", "depth")
        self.assert_input(info, "topicName", "wrist_camera/color/camera_info")
        self.assert_input(info, "frameId", "wrist_camera_color_optical_frame")
        for destination in (
            "WristRgbPublisher.inputs:renderProductPath",
            "WristDepthPublisher.inputs:renderProductPath",
            "WristCameraInfoPublisher.inputs:renderProductPath",
        ):
            self.assert_connected_from(
                stage,
                f"/ActionGraph/{destination}",
                "/ActionGraph/CreateWristCameraRenderProduct.outputs:renderProductPath",
            )

    def test_ros_overlay_has_odometry_and_one_dynamic_base_tf(self):
        stage = self.ros_stage()
        compute = stage.GetPrimAtPath("/ActionGraph/ComputeOdometry")
        odometry = stage.GetPrimAtPath("/ActionGraph/PublishOdometry")
        odom_tf = stage.GetPrimAtPath("/ActionGraph/PublishOdomTF")
        self.assertTrue(compute.IsValid())
        self.assertTrue(odometry.IsValid())
        self.assertTrue(odom_tf.IsValid())

        self.assert_target(compute, "chassisPrim", [f"{ROBOT_PATH}/base_link"])
        self.assert_input(odometry, "topicName", "odom")
        self.assert_input(odometry, "odomFrameId", "odom")
        self.assert_input(odometry, "chassisFrameId", "base_link")
        self.assert_input(odom_tf, "topicName", "tf")
        self.assert_input(odom_tf, "parentFrameId", "odom")
        self.assert_input(odom_tf, "childFrameId", "base_link")
        self.assert_input(odom_tf, "staticPublisher", False)

        connections = (
            ("ComputeOdometry.inputs:execIn", "OnPlaybackTick.outputs:tick"),
            ("PublishOdometry.inputs:execIn", "OnPlaybackTick.outputs:tick"),
            ("PublishOdomTF.inputs:execIn", "OnPlaybackTick.outputs:tick"),
            ("PublishOdometry.inputs:timeStamp", "ReadSimTime.outputs:simulationTime"),
            ("PublishOdomTF.inputs:timeStamp", "ReadSimTime.outputs:simulationTime"),
            ("PublishOdometry.inputs:position", "ComputeOdometry.outputs:position"),
            ("PublishOdometry.inputs:orientation", "ComputeOdometry.outputs:orientation"),
            ("PublishOdometry.inputs:linearVelocity", "ComputeOdometry.outputs:linearVelocity"),
            ("PublishOdometry.inputs:angularVelocity", "ComputeOdometry.outputs:angularVelocity"),
            ("PublishOdomTF.inputs:translation", "ComputeOdometry.outputs:position"),
            ("PublishOdomTF.inputs:rotation", "ComputeOdometry.outputs:orientation"),
        )
        for destination, source in connections:
            self.assert_connected_from(stage, f"/ActionGraph/{destination}", f"/ActionGraph/{source}")


if __name__ == "__main__":
    result = unittest.main(verbosity=2, exit=False).result
    sys.exit(not result.wasSuccessful())
