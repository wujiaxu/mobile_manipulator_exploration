#!/usr/bin/env python3
import argparse
from math import degrees
from pathlib import Path

from factory_layout import ARENA_SIZE, NAV_GOALS, PRIMITIVES, ROBOT_SPAWN


def parse_args(argv=None):
    default_output = (
        Path(__file__).resolve().parents[1]
        / "assets/environments/compact_factory/compact_factory.usd"
    )
    parser = argparse.ArgumentParser(description="Generate the compact factory USD.")
    parser.add_argument("--output", type=Path, default=default_output)
    return parser.parse_args(argv)


def create_material(stage, path, color, UsdShade, Sdf):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.72)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(prim, material, UsdShade):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def add_lighting(stage, modules):
    UsdGeom, UsdLux = modules
    stage.DefinePrim("/Factory/Lights", "Scope")

    ambient = UsdLux.DomeLight.Define(stage, "/Factory/Lights/Ambient")
    ambient.CreateIntensityAttr(350.0)
    ambient.CreateColorAttr((1.0, 0.96, 0.90))

    overhead_specs = (
        ("/Factory/Lights/OverheadSouth", (0.0, -2.0, 4.2), 900.0),
        ("/Factory/Lights/OverheadNorth", (0.0, 2.0, 4.2), 900.0),
    )
    for path, position, intensity in overhead_specs:
        light = UsdLux.RectLight.Define(stage, path)
        light.CreateIntensityAttr(intensity)
        light.CreateWidthAttr(7.5)
        light.CreateHeightAttr(3.0)
        light.CreateColorAttr((1.0, 0.96, 0.90))
        UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(position)


def add_box(stage, item, material, modules):
    UsdGeom, UsdPhysics, UsdShade, _ = modules
    cube = UsdGeom.Cube.Define(stage, item["path"])
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform.SetTranslate(item["position"])
    xform.SetScale(item["size"])
    bind_material(cube.GetPrim(), material, UsdShade)
    if item["collision"]:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def add_cylinder(stage, item, material, modules):
    UsdGeom, UsdPhysics, UsdShade, _ = modules
    cylinder = UsdGeom.Cylinder.Define(stage, item["path"])
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(item["radius"])
    cylinder.CreateHeightAttr(item["height"])
    xform = UsdGeom.XformCommonAPI(cylinder.GetPrim())
    xform.SetTranslate(item["position"])
    xform.SetRotate(tuple(degrees(value) for value in item["rotation"]))
    bind_material(cylinder.GetPrim(), material, UsdShade)
    if item["collision"]:
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())


def generate(output_path):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    factory = stage.DefinePrim("/Factory", "Xform")
    stage.SetDefaultPrim(factory)
    stage.DefinePrim("/Factory/Looks", "Scope")
    add_lighting(stage, (UsdGeom, UsdLux))

    materials = {}
    for item in PRIMITIVES:
        color = item["color"]
        if color not in materials:
            materials[color] = create_material(
                stage,
                f"/Factory/Looks/Material{len(materials) + 1:02d}",
                Gf.Vec3f(*color),
                UsdShade,
                Sdf,
            )

    ground_item = {
        "path": "/Factory/Ground",
        "position": (0.0, 0.0, -0.05),
        "size": (ARENA_SIZE[0], ARENA_SIZE[1], 0.1),
        "collision": True,
    }
    ground_material = create_material(
        stage, "/Factory/Looks/Ground", Gf.Vec3f(0.34, 0.36, 0.35), UsdShade, Sdf
    )
    add_box(stage, ground_item, ground_material, (UsdGeom, UsdPhysics, UsdShade, Sdf))

    modules = (UsdGeom, UsdPhysics, UsdShade, Sdf)
    for item in PRIMITIVES:
        material = materials[item["color"]]
        if item["shape"] == "box":
            add_box(stage, item, material, modules)
        else:
            add_cylinder(stage, item, material, modules)

    spawn = stage.DefinePrim("/Factory/RobotSpawn", "Xform")
    UsdGeom.XformCommonAPI(spawn).SetTranslate(ROBOT_SPAWN[:3])
    UsdGeom.XformCommonAPI(spawn).SetRotate((0.0, 0.0, degrees(ROBOT_SPAWN[3])))
    stage.DefinePrim("/Factory/NavGoals", "Xform")
    for index, goal in enumerate(NAV_GOALS, 1):
        marker = stage.DefinePrim(f"/Factory/NavGoals/Goal{index}", "Xform")
        UsdGeom.XformCommonAPI(marker).SetTranslate((goal[0], goal[1], 0.0))
        UsdGeom.XformCommonAPI(marker).SetRotate((0.0, 0.0, degrees(goal[2])))

    stage.GetRootLayer().Save()
    return output_path


def main():
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    output = generate(args.output)
    print(f"Generated factory: {output}")
    print(f"Obstacles: {len(PRIMITIVES)}; goals: {len(NAV_GOALS)}")
    app.close()


if __name__ == "__main__":
    main()
