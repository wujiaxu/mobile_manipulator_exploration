#!/usr/bin/env python3
import argparse
import os
from pathlib import Path


WRIST_CAMERA_WIDTH = 848
WRIST_CAMERA_HEIGHT = 480
WRIST_CAMERA_FX = 429.0
WRIST_CAMERA_FY = 427.0
WRIST_CAMERA_CX = 425.0
WRIST_CAMERA_CY = 240.0
WRIST_CAMERA_FOCAL_LENGTH = 2.0
WRIST_CAMERA_HORIZONTAL_APERTURE = (
    WRIST_CAMERA_WIDTH * WRIST_CAMERA_FOCAL_LENGTH / WRIST_CAMERA_FX
)
WRIST_CAMERA_VERTICAL_APERTURE = (
    WRIST_CAMERA_HEIGHT * WRIST_CAMERA_FOCAL_LENGTH / WRIST_CAMERA_FY
)
WRIST_CAMERA_HORIZONTAL_APERTURE_OFFSET = (
    (WRIST_CAMERA_CX - WRIST_CAMERA_WIDTH / 2.0) / WRIST_CAMERA_FX
)
WRIST_CAMERA_VERTICAL_APERTURE_OFFSET = (
    (WRIST_CAMERA_CY - WRIST_CAMERA_HEIGHT / 2.0) / WRIST_CAMERA_FY
)


def parse_args():
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Import the unified mobile manipulator into Isaac Sim 4.5."
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=workspace / "build/isaac_import/mobile_manipulator.urdf",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=workspace / "isaac_sim/assets/robots/mobile_manipulator",
    )
    parser.add_argument("--gui", action="store_true", help="Run with a viewport.")
    return parser.parse_args()


def main():
    args = parse_args()
    urdf_path = args.urdf.resolve()
    output_dir = args.output_dir.resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    workspace = Path(__file__).resolve().parents[2]
    package_path = str(workspace)
    current_package_path = os.environ.get("ROS_PACKAGE_PATH", "")
    os.environ["ROS_PACKAGE_PATH"] = (
        f"{package_path}:{current_package_path}" if current_package_path else package_path
    )

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": not args.gui})

    import omni.graph.core as og
    import omni.kit.commands
    import omni.usd
    import usdrt
    import carb.settings
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    simulation_app.update()

    lidar_config_dir = str((workspace / "isaac_sim/config/lidar").resolve()) + "/"
    lidar_profile_folders = carb.settings.get_settings().get(
        "/app/sensors/nv/lidar/profileBaseFolder"
    )
    if lidar_profile_folders is None:
        lidar_profile_folders = []
    if lidar_config_dir not in lidar_profile_folders:
        carb.settings.get_settings().set(
            "/app/sensors/nv/lidar/profileBaseFolder",
            [lidar_config_dir, *lidar_profile_folders],
        )

    base_usd = output_dir / "mobile_manipulator.usd"
    ros_usd = output_dir / "mobile_manipulator_ros.usd"

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("Isaac Sim did not create a URDF import configuration")
    import_config.fix_base = False
    import_config.merge_fixed_joints = False
    import_config.import_inertia_tensor = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = True
    import_config.set_collision_from_visuals(False)

    status, imported_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=import_config,
        dest_path=str(base_usd),
    )
    if not status:
        raise RuntimeError("URDFParseAndImportFile failed")
    simulation_app.update()

    base_stage = Usd.Stage.Open(str(base_usd))
    if base_stage is None:
        raise RuntimeError(f"Could not open imported USD: {base_usd}")
    robot = base_stage.GetPrimAtPath("/mobile_manipulator")
    if not robot.IsValid():
        raise RuntimeError("Imported robot prim /mobile_manipulator is missing")
    articulation_roots = [
        prim
        for prim in Usd.PrimRange(robot)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        paths = [str(prim.GetPath()) for prim in articulation_roots]
        raise RuntimeError(f"Expected one articulation root, found: {paths}")
    articulation_path = str(articulation_roots[0].GetPath())
    if UsdGeom.GetStageMetersPerUnit(base_stage) != 1.0:
        raise RuntimeError("Imported stage is not in meters")

    for joint_name in ("left_wheel", "right_wheel"):
        joint = base_stage.GetPrimAtPath(
            f"/mobile_manipulator/joints/{joint_name}"
        )
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if not drive:
            raise RuntimeError(f"Missing angular drive for {joint_name}")
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetDampingAttr().Set(10000.0)
    base_stage.GetRootLayer().Save()

    overlay_stage = Usd.Stage.CreateNew(str(ros_usd))
    overlay_stage.GetRootLayer().subLayerPaths = [f"./{base_usd.name}"]
    UsdGeom.SetStageUpAxis(
        overlay_stage, UsdGeom.GetStageUpAxis(base_stage)
    )
    UsdGeom.SetStageMetersPerUnit(
        overlay_stage, UsdGeom.GetStageMetersPerUnit(base_stage)
    )
    overlay_stage.GetRootLayer().Save()

    if not omni.usd.get_context().open_stage(str(ros_usd)):
        raise RuntimeError(f"Could not open ROS overlay: {ros_usd}")
    simulation_app.update()
    overlay_stage = omni.usd.get_context().get_stage()
    robot = overlay_stage.GetPrimAtPath("/mobile_manipulator")
    overlay_stage.SetDefaultPrim(robot)

    status, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/mobile_manipulator/livox_frame/NavLidar",
        parent=None,
        config="MobileManipulator_Nav2D",
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if not status or not lidar_prim:
        raise RuntimeError("Could not create /mobile_manipulator/livox_frame/NavLidar")

    camera = UsdGeom.Camera.Define(
        overlay_stage,
        "/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera",
    )
    # USD cameras look along local -Z. Rotate the prim so its rendered FOV
    # points along ROS optical +Z while the published frame remains optical.
    UsdGeom.XformCommonAPI(camera.GetPrim()).SetRotate((180.0, 0.0, 0.0))
    camera.CreateHorizontalApertureAttr(WRIST_CAMERA_HORIZONTAL_APERTURE)
    camera.CreateVerticalApertureAttr(WRIST_CAMERA_VERTICAL_APERTURE)
    camera.CreateHorizontalApertureOffsetAttr(WRIST_CAMERA_HORIZONTAL_APERTURE_OFFSET)
    camera.CreateVerticalApertureOffsetAttr(WRIST_CAMERA_VERTICAL_APERTURE_OFFSET)
    camera.CreateFocalLengthAttr(WRIST_CAMERA_FOCAL_LENGTH)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.15, 8.0))
    simulation_app.update()

    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                (
                    "PublishOdomTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinearVelocity", "omni.graph.nodes.BreakVector3"),
                ("BreakAngularVelocity", "omni.graph.nodes.BreakVector3"),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                (
                    "BaseArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "SubscribeArmJointState",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArmArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "CreateLidarRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                ("ScanPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                (
                    "CreateWristCameraRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                ("WristRgbPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("WristDepthPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (
                    "WristCameraInfoPublisher",
                    "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                ),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                (
                    "OnPlaybackTick.outputs:tick",
                    "CreateLidarRenderProduct.inputs:execIn",
                ),
                ("CreateLidarRenderProduct.outputs:execOut", "ScanPublisher.inputs:execIn"),
                (
                    "CreateLidarRenderProduct.outputs:renderProductPath",
                    "ScanPublisher.inputs:renderProductPath",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "CreateWristCameraRenderProduct.inputs:execIn",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:execOut",
                    "WristRgbPublisher.inputs:execIn",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:execOut",
                    "WristDepthPublisher.inputs:execIn",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:execOut",
                    "WristCameraInfoPublisher.inputs:execIn",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:renderProductPath",
                    "WristRgbPublisher.inputs:renderProductPath",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:renderProductPath",
                    "WristDepthPublisher.inputs:renderProductPath",
                ),
                (
                    "CreateWristCameraRenderProduct.outputs:renderProductPath",
                    "WristCameraInfoPublisher.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdometry.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdomTF.inputs:timeStamp",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdometry.inputs:position",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdomTF.inputs:translation",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdomTF.inputs:rotation",
                ),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                (
                    "OnPlaybackTick.outputs:tick",
                    "BaseArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                (
                    "SubscribeTwist.outputs:execOut",
                    "DifferentialController.inputs:execIn",
                ),
                (
                    "SubscribeTwist.outputs:linearVelocity",
                    "BreakLinearVelocity.inputs:tuple",
                ),
                (
                    "SubscribeTwist.outputs:angularVelocity",
                    "BreakAngularVelocity.inputs:tuple",
                ),
                (
                    "BreakLinearVelocity.outputs:x",
                    "DifferentialController.inputs:linearVelocity",
                ),
                (
                    "BreakAngularVelocity.outputs:z",
                    "DifferentialController.inputs:angularVelocity",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "BaseArticulationController.inputs:velocityCommand",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeArmJointState.inputs:execIn",
                ),
                (
                    "SubscribeArmJointState.outputs:execOut",
                    "ArmArticulationController.inputs:execIn",
                ),
                (
                    "Context.outputs:context",
                    "SubscribeArmJointState.inputs:context",
                ),
                (
                    "SubscribeArmJointState.outputs:jointNames",
                    "ArmArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeArmJointState.outputs:positionCommand",
                    "ArmArticulationController.inputs:positionCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:topicName", "joint_states"),
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(articulation_path)],
                ),
                ("PublishClock.inputs:topicName", "clock"),
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [usdrt.Sdf.Path(articulation_path)],
                ),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "base_link"),
                ("PublishOdomTF.inputs:topicName", "tf"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_link"),
                ("PublishOdomTF.inputs:staticPublisher", False),
                ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                ("DifferentialController.inputs:wheelRadius", 0.0605),
                ("DifferentialController.inputs:wheelDistance", 0.34),
                (
                    "BaseArticulationController.inputs:jointNames",
                    ["left_wheel", "right_wheel"],
                ),
                (
                    "BaseArticulationController.inputs:targetPrim",
                    [usdrt.Sdf.Path(articulation_path)],
                ),
                (
                    "SubscribeArmJointState.inputs:topicName",
                    "arm_joint_commands",
                ),
                (
                    "ArmArticulationController.inputs:targetPrim",
                    [usdrt.Sdf.Path(articulation_path)],
                ),
                (
                    "CreateLidarRenderProduct.inputs:cameraPrim",
                    [usdrt.Sdf.Path("/mobile_manipulator/livox_frame/NavLidar")],
                ),
                ("CreateLidarRenderProduct.inputs:width", 1),
                ("CreateLidarRenderProduct.inputs:height", 1),
                ("ScanPublisher.inputs:topicName", "scan"),
                ("ScanPublisher.inputs:frameId", "livox_frame"),
                ("ScanPublisher.inputs:type", "laser_scan"),
                ("ScanPublisher.inputs:useSystemTime", False),
                ("ScanPublisher.inputs:resetSimulationTimeOnStop", True),
                (
                    "CreateWristCameraRenderProduct.inputs:cameraPrim",
                    [
                        usdrt.Sdf.Path(
                            "/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera"
                        )
                    ],
                ),
                ("CreateWristCameraRenderProduct.inputs:width", WRIST_CAMERA_WIDTH),
                ("CreateWristCameraRenderProduct.inputs:height", WRIST_CAMERA_HEIGHT),
                ("WristRgbPublisher.inputs:topicName", "wrist_camera/color/image_raw"),
                ("WristRgbPublisher.inputs:frameId", "wrist_camera_color_optical_frame"),
                ("WristRgbPublisher.inputs:type", "rgb"),
                ("WristRgbPublisher.inputs:useSystemTime", False),
                ("WristRgbPublisher.inputs:resetSimulationTimeOnStop", True),
                ("WristDepthPublisher.inputs:topicName", "wrist_camera/depth/image_rect_raw"),
                ("WristDepthPublisher.inputs:frameId", "wrist_camera_color_optical_frame"),
                ("WristDepthPublisher.inputs:type", "depth"),
                ("WristDepthPublisher.inputs:useSystemTime", False),
                ("WristDepthPublisher.inputs:resetSimulationTimeOnStop", True),
                ("WristCameraInfoPublisher.inputs:topicName", "wrist_camera/color/camera_info"),
                (
                    "WristCameraInfoPublisher.inputs:frameId",
                    "wrist_camera_color_optical_frame",
                ),
                ("WristCameraInfoPublisher.inputs:useSystemTime", False),
                (
                    "WristCameraInfoPublisher.inputs:resetSimulationTimeOnStop",
                    True,
                ),
            ],
        },
    )
    simulation_app.update()
    overlay_stage.GetRootLayer().Save()

    print(f"Imported robot: {imported_path}")
    print(f"Articulation root: {articulation_path}")
    print(f"Base USD: {base_usd}")
    print(f"ROS overlay: {ros_usd}")
    simulation_app.close()


if __name__ == "__main__":
    main()
