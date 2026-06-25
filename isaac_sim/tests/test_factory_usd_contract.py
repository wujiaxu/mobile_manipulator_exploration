#!/usr/bin/env python3
import importlib.util
import sys
import unittest
from pathlib import Path


ISAAC_DIR = Path(__file__).resolve().parents[1]
GENERATOR = ISAAC_DIR / "scripts/generate_factory_environment.py"
LAYOUT_PATH = ISAAC_DIR / "scripts/factory_layout.py"
FACTORY_USD = ISAAC_DIR / "assets/environments/compact_factory/compact_factory.usd"


class FactoryGeneratorContract(unittest.TestCase):
    def test_generator_and_asset_exist(self):
        self.assertTrue(GENERATOR.is_file(), f"Missing generator: {GENERATOR}")
        self.assertTrue(FACTORY_USD.is_file(), f"Missing factory asset: {FACTORY_USD}")

    def test_generator_authors_required_usd_contract(self):
        source = GENERATOR.read_text(encoding="utf-8")
        for required in (
            "Usd.Stage.CreateNew",
            "UsdGeom.SetStageUpAxis",
            "UsdGeom.SetStageMetersPerUnit",
            "UsdPhysics.CollisionAPI.Apply",
            "UsdLux.DomeLight.Define",
            "UsdLux.RectLight.Define",
            'DefinePrim("/Factory"',
            'DefinePrim("/Factory/RobotSpawn"',
            'DefinePrim("/Factory/NavGoals"',
        ):
            self.assertIn(required, source)


@unittest.skipUnless(FACTORY_USD.is_file(), "Factory USD not generated")
class GeneratedFactoryUsdContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from isaacsim import SimulationApp

        cls.app = SimulationApp({"headless": True})
        global Usd, UsdGeom, UsdLux, UsdPhysics
        from pxr import Usd, UsdGeom, UsdLux, UsdPhysics

        spec = importlib.util.spec_from_file_location("factory_layout", LAYOUT_PATH)
        cls.layout = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.layout)
        cls.stage = Usd.Stage.Open(str(FACTORY_USD))
        if cls.stage is None:
            raise AssertionError(f"Could not open {FACTORY_USD}")

    @classmethod
    def tearDownClass(cls):
        cls.app.close()

    def test_stage_metadata_and_markers(self):
        self.assertEqual(UsdGeom.GetStageUpAxis(self.stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)
        self.assertEqual(str(self.stage.GetDefaultPrim().GetPath()), "/Factory")
        ground = self.stage.GetPrimAtPath("/Factory/Ground")
        self.assertTrue(ground.IsValid())
        self.assertTrue(ground.HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(self.stage.GetPrimAtPath("/Factory/RobotSpawn").IsValid())
        for index in range(1, 4):
            self.assertTrue(self.stage.GetPrimAtPath(f"/Factory/NavGoals/Goal{index}").IsValid())

    def test_all_layout_primitives_exist_with_collision(self):
        for item in self.layout.PRIMITIVES:
            prim = self.stage.GetPrimAtPath(item["path"])
            self.assertTrue(prim.IsValid(), item["path"])
            if item["collision"]:
                self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI), item["path"])

    def test_factory_has_usable_lighting(self):
        lights = [
            prim
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/Factory"))
            if prim.GetTypeName() in {"DomeLight", "RectLight"}
        ]
        self.assertGreaterEqual(len(lights), 3)
        expected_lights = {
            "/Factory/Lights/Ambient": "DomeLight",
            "/Factory/Lights/OverheadSouth": "RectLight",
            "/Factory/Lights/OverheadNorth": "RectLight",
        }
        for path, type_name in expected_lights.items():
            prim = self.stage.GetPrimAtPath(path)
            self.assertTrue(prim.IsValid(), path)
            self.assertEqual(prim.GetTypeName(), type_name, path)


if __name__ == "__main__":
    result = unittest.main(verbosity=2, exit=False).result
    sys.exit(not result.wasSuccessful())
