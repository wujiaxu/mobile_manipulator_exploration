#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

from factory_layout import ROBOT_CLEARANCE_Z


def parse_args(argv=None):
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Run the unified mobile manipulator with ROS 2 state publishing."
    )
    parser.add_argument(
        "--usd",
        type=Path,
        default=workspace
        / "isaac_sim/assets/robots/mobile_manipulator/mobile_manipulator_ros.usd",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--disable-control",
        action="store_true",
        help="Disable articulation command nodes for physics diagnostics.",
    )
    parser.add_argument(
        "--disable-action-graph",
        action="store_true",
        help="Disable the complete ROS action graph for physics diagnostics.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run; zero runs until the app is closed.",
    )
    parser.add_argument(
        "--pose-report-period",
        type=float,
        default=0.0,
        help="Print articulation world pose at this interval; zero disables reporting.",
    )
    args = parser.parse_args(argv)
    if args.pose_report_period < 0:
        parser.error("--pose-report-period must be nonnegative")
    return args


def main():
    args = parse_args()
    usd_path = args.usd.resolve()
    if not usd_path.is_file():
        raise FileNotFoundError(usd_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    if not omni.usd.get_context().open_stage(str(usd_path)):
        raise RuntimeError(f"Could not open stage: {usd_path}")
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    print(
        "Stage metadata: "
        f"up_axis={UsdGeom.GetStageUpAxis(stage)} "
        f"meters_per_unit={UsdGeom.GetStageMetersPerUnit(stage)}"
    )

    robot = stage.GetPrimAtPath("/mobile_manipulator")
    if not robot.IsValid():
        raise RuntimeError("Missing /mobile_manipulator")
    UsdGeom.XformCommonAPI(robot).SetTranslate((0.0, 0.0, 0.0))
    base_link = stage.GetPrimAtPath("/mobile_manipulator/base_link")
    if not base_link.IsValid():
        raise RuntimeError("Missing /mobile_manipulator/base_link")
    base_xform = UsdGeom.XformCommonAPI(base_link)
    base_xform.SetTranslate((0.0, 0.0, ROBOT_CLEARANCE_Z))

    if args.disable_action_graph:
        action_graph = stage.GetPrimAtPath("/ActionGraph")
        if action_graph.IsValid():
            action_graph.SetActive(False)

    if args.disable_control:
        for node_name in (
            "BaseArticulationController",
            "ArmArticulationController",
        ):
            node = stage.GetPrimAtPath(f"/ActionGraph/{node_name}")
            if node.IsValid():
                node.SetActive(False)

    articulation_roots = [
        prim
        for prim in Usd.PrimRange(robot)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        paths = [str(prim.GetPath()) for prim in articulation_roots]
        raise RuntimeError(f"Expected one articulation root, found: {paths}")
    articulation_path = str(articulation_roots[0].GetPath())

    if not stage.GetPrimAtPath("/groundPlane").IsValid():
        PhysicsSchemaTools.addGroundPlane(
            stage,
            "/groundPlane",
            "Z",
            100.0,
            Gf.Vec3f(0.0, 0.0, 0.0),
            Gf.Vec3f(0.45, 0.45, 0.45),
        )

    if not args.headless:
        set_camera_view(
            eye=[2.4, 2.4, 1.8],
            target=[0.0, 0.0, 0.65],
            camera_prim_path="/OmniverseKit_Persp",
        )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    dynamic_control = None
    root_body = None
    if args.pose_report_period > 0:
        dynamic_control = _dynamic_control.acquire_dynamic_control_interface()
        articulation_handle = dynamic_control.get_articulation(articulation_path)
        if articulation_handle == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError(f"Could not access articulation: {articulation_path}")
        root_body = dynamic_control.get_articulation_root_body(articulation_handle)
    started = time.monotonic()
    next_pose_report = started
    print(f"Running: {usd_path}")
    print("Publishing: /clock and /joint_states")
    while simulation_app.is_running():
        simulation_app.update()
        now = time.monotonic()
        if args.pose_report_period > 0 and now >= next_pose_report:
            pose = dynamic_control.get_rigid_body_pose(root_body)
            print(
                f"Pose t={now - started:.3f} "
                f"position={[pose.p.x, pose.p.y, pose.p.z]} "
                f"quaternion_xyzw={[pose.r.x, pose.r.y, pose.r.z, pose.r.w]}"
            )
            next_pose_report = now + args.pose_report_period
        if args.duration > 0 and now - started >= args.duration:
            break
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
