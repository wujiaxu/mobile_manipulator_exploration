#!/usr/bin/env python3
import argparse
import sys
import time
from math import cos, degrees, sin
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from factory_layout import ROBOT_SPAWN

OBSERVATION_ARM_JOINT_POSITIONS = {
    "joint1": 1.5707963267948966,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": -1.5707963267948966,
    "joint7": 1.5707963267948966,
}


def apply_observation_arm_pose(dynamic_control, articulation_handle, invalid_handle):
    for joint_name, joint_position in OBSERVATION_ARM_JOINT_POSITIONS.items():
        dof_handle = dynamic_control.find_articulation_dof(articulation_handle, joint_name)
        if dof_handle == invalid_handle:
            raise RuntimeError(f"Could not find arm DOF: {joint_name}")
        dynamic_control.set_dof_position(dof_handle, joint_position)
        dynamic_control.set_dof_position_target(dof_handle, joint_position)


def parse_args(argv=None):
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Run the mobile manipulator in the compact factory for Nav2."
    )
    parser.add_argument(
        "--factory-usd",
        type=Path,
        default=workspace
        / "isaac_sim/assets/environments/compact_factory/compact_factory.usd",
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=workspace
        / "isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--spawn",
        type=float,
        nargs=4,
        metavar=("X", "Y", "Z", "YAW"),
        default=ROBOT_SPAWN,
        help="Robot spawn pose in meters/radians: x y z yaw.",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="Wall seconds; zero runs until closed."
    )
    args = parser.parse_args(argv)
    if args.duration < 0.0:
        parser.error("--duration must be nonnegative")
    return args


def main():
    args = parse_args()
    factory_usd = args.factory_usd.resolve()
    robot_usd = args.robot_usd.resolve()
    if not factory_usd.is_file():
        raise FileNotFoundError(factory_usd)
    if not robot_usd.is_file():
        raise FileNotFoundError(robot_usd)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})

    import omni.graph.core as og
    import omni.timeline
    import omni.usd
    import carb.settings
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    app.update()
    lidar_config_dir = str((SCRIPT_DIR.parents[0] / "config/lidar").resolve()) + "/"
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

    context = omni.usd.get_context()
    context.new_stage()
    app.update()
    stage = context.get_stage()
    stage.GetRootLayer().subLayerPaths = [str(factory_usd), str(robot_usd)]
    app.update()
    stage.SetEditTarget(stage.GetSessionLayer())

    factory = stage.GetPrimAtPath(Sdf.Path("/Factory"))
    if not factory.IsValid():
        raise RuntimeError("Missing /Factory")
        
    robot = stage.GetPrimAtPath(Sdf.Path("/mobile_manipulator"))
    if not robot.IsValid():
        raise RuntimeError("Missing /mobile_manipulator")

    robot_xform = UsdGeom.XformCommonAPI(robot)
    spawn_pose_values = tuple(args.spawn)
    robot_xform.SetTranslate((spawn_pose_values[0], spawn_pose_values[1], 0.0))
    robot_xform.SetRotate((0.0, 0.0, degrees(spawn_pose_values[3])))
    base_link = stage.GetPrimAtPath(Sdf.Path("/mobile_manipulator/base_link"))
    if not base_link.IsValid():
        raise RuntimeError("Missing /mobile_manipulator/base_link")
    base_xform = UsdGeom.XformCommonAPI(base_link)
    base_xform.SetTranslate((0.0, 0.0, spawn_pose_values[2]))
    app.update()

    articulation_roots = [
        prim
        for prim in Usd.PrimRange(robot)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(f"Expected one articulation root, found {len(articulation_roots)}")
    articulation_path = str(articulation_roots[0].GetPath())
    if not stage.GetPrimAtPath(Sdf.Path("/mobile_manipulator/livox_frame/NavLidar")).IsValid():
        raise RuntimeError("Robot USD is missing /mobile_manipulator/livox_frame/NavLidar")
    graph_path = None
    for candidate in ("/mobile_manipulator/ActionGraph", "/ActionGraph"):
        if og.Controller.graph(candidate) is not None:
            graph_path = candidate
            break
    if graph_path is None:
        raise RuntimeError("Robot USD is missing ActionGraph")
    required_robot_prims = (
        "/mobile_manipulator/livox_frame/NavLidar",
        "/mobile_manipulator/wrist_camera_color_optical_frame/D455Camera",
    )
    for prim_path in required_robot_prims:
        if not stage.GetPrimAtPath(Sdf.Path(prim_path)).IsValid():
            raise RuntimeError(f"Robot USD is missing {prim_path}")
    required_graph_nodes = (
        "ScanPublisher",
        "WristRgbPublisher",
        "WristDepthPublisher",
        "WristCameraInfoPublisher",
    )
    for node_name in required_graph_nodes:
        node_path = f"{graph_path}/{node_name}"
        if not stage.GetPrimAtPath(Sdf.Path(node_path)).IsValid():
            raise RuntimeError(f"Robot USD is missing {node_path}")
    app.update()

    if not args.headless:
        set_camera_view(
            eye=[8.5, 8.5, 7.0],
            target=[0.0, 0.0, 0.8],
            camera_prim_path="/OmniverseKit_Persp",
        )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app.update()

    dynamic_control = _dynamic_control.acquire_dynamic_control_interface()
    articulation_handle = dynamic_control.get_articulation(articulation_path)
    if articulation_handle == _dynamic_control.INVALID_HANDLE:
        raise RuntimeError(f"Could not access articulation: {articulation_path}")
    root_body = dynamic_control.get_articulation_root_body(articulation_handle)
    yaw = spawn_pose_values[3]
    spawn_pose = _dynamic_control.Transform(
        (spawn_pose_values[0], spawn_pose_values[1], spawn_pose_values[2]),
        (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)),
    )
    dynamic_control.set_rigid_body_pose(root_body, spawn_pose)
    dynamic_control.set_rigid_body_linear_velocity(root_body, (0.0, 0.0, 0.0))
    dynamic_control.set_rigid_body_angular_velocity(root_body, (0.0, 0.0, 0.0))
    apply_observation_arm_pose(dynamic_control, articulation_handle, _dynamic_control.INVALID_HANDLE)
    dynamic_control.wake_up_articulation(articulation_handle)
    app.update()
    
    started = time.monotonic()
    print(f"Factory: {factory_usd}")
    print(f"Robot: {robot_usd}")
    print(
        "Publishing: /clock /joint_states /odom /tf /scan "
        "/wrist_camera/color/image_raw /wrist_camera/depth/image_rect_raw "
        "/wrist_camera/color/camera_info"
    )
    
    while app.is_running():
        app.update()
        if args.duration > 0.0 and time.monotonic() - started >= args.duration:
            break

    timeline.stop()
    app.close()

if __name__ == "__main__":
    main()
