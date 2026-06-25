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
    parser.add_argument("--disable-lidar", action="store_true")
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
    import numpy as np
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.sensors.rtx import LidarRtx
    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    app.update()

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
    robot_xform.SetTranslate((ROBOT_SPAWN[0], ROBOT_SPAWN[1], 0.0))
    robot_xform.SetRotate((0.0, 0.0, degrees(ROBOT_SPAWN[3])))
    base_link = stage.GetPrimAtPath(Sdf.Path("/mobile_manipulator/base_link"))
    if not base_link.IsValid():
        raise RuntimeError("Missing /mobile_manipulator/base_link")
    base_xform = UsdGeom.XformCommonAPI(base_link)
    base_xform.SetTranslate((0.0, 0.0, ROBOT_SPAWN[2]))
    app.update()

    articulation_roots = [
        prim
        for prim in Usd.PrimRange(robot)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(f"Expected one articulation root, found {len(articulation_roots)}")
    articulation_path = str(articulation_roots[0].GetPath())

    lidar = None
    if not args.disable_lidar:
        lidar = LidarRtx(
            prim_path="/mobile_manipulator/livox_frame/NavLidar",
            name="nav_lidar",
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            config_file_name="Example_Rotary_2D",
        )
        action_graph = og.Controller.graph("/ActionGraph")
        if action_graph is None:
            raise RuntimeError("Robot USD is missing /ActionGraph")
        og.Controller.edit(
            action_graph,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ScanPublisher", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        "/ActionGraph/OnPlaybackTick.outputs:tick",
                        "ScanPublisher.inputs:execIn",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "ScanPublisher.inputs:renderProductPath",
                        lidar.get_render_product_path(),
                    ),
                    ("ScanPublisher.inputs:topicName", "scan"),
                    ("ScanPublisher.inputs:frameId", "livox_frame"),
                    ("ScanPublisher.inputs:type", "laser_scan"),
                    ("ScanPublisher.inputs:useSystemTime", False),
                    ("ScanPublisher.inputs:resetSimulationTimeOnStop", True),
                ],
            },
        )
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
    yaw = ROBOT_SPAWN[3]
    spawn_pose = _dynamic_control.Transform(
        (ROBOT_SPAWN[0], ROBOT_SPAWN[1], ROBOT_SPAWN[2]),
        (0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)),
    )
    dynamic_control.set_rigid_body_pose(root_body, spawn_pose)
    dynamic_control.set_rigid_body_linear_velocity(root_body, (0.0, 0.0, 0.0))
    dynamic_control.set_rigid_body_angular_velocity(root_body, (0.0, 0.0, 0.0))
    dynamic_control.wake_up_articulation(articulation_handle)
    app.update()
    
    started = time.monotonic()
    print(f"Factory: {factory_usd}")
    print(f"Robot: {robot_usd}")
    print("Publishing: /clock /joint_states /odom /tf /scan")
    
    while app.is_running():
        app.update()
        if args.duration > 0.0 and time.monotonic() - started >= args.duration:
            break

    timeline.stop()
    lidar = None
    app.close()

if __name__ == "__main__":
    main()
