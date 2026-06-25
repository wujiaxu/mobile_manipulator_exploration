#!/usr/bin/env python3
import importlib.util
import unittest
from collections import Counter
from pathlib import Path


LAYOUT_PATH = Path(__file__).resolve().parents[1] / "scripts/factory_layout.py"


def load_layout():
    spec = importlib.util.spec_from_file_location("factory_layout", LAYOUT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FactoryLayoutContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = load_layout()

    def test_arena_routes_spawn_and_goals(self):
        self.assertEqual(self.layout.ARENA_SIZE, (10.0, 8.0))
        self.assertGreaterEqual(self.layout.MAIN_AISLE_WIDTH, 1.2)
        self.assertGreaterEqual(self.layout.SERVICE_AISLE_WIDTH, 0.8)
        self.assertEqual(len(self.layout.ROBOT_SPAWN), 4)
        self.assertGreaterEqual(self.layout.ROBOT_SPAWN[2], 0.30)
        self.assertEqual(len(self.layout.NAV_GOALS), 3)
        self.assertGreaterEqual(len(self.layout.CLEARANCE_ZONES), 3)

        for zone in self.layout.CLEARANCE_ZONES:
            self.assertGreaterEqual(zone["width"], self.layout.SERVICE_AISLE_WIDTH)
            self.assertIn(zone["kind"], {"main", "service"})
            if zone["kind"] == "main":
                self.assertGreaterEqual(zone["width"], self.layout.MAIN_AISLE_WIDTH)

    def test_required_factory_density(self):
        counts = Counter(item["category"] for item in self.layout.PRIMITIVES)
        minimums = {
            "perimeter_wall": 4,
            "partition_wall": 2,
            "column": 6,
            "cabinet": 4,
            "tank": 3,
            "pump": 2,
            "pipe_rack": 4,
            "vertical_pipe": 12,
            "horizontal_pipe": 12,
            "guardrail": 4,
        }
        for category, minimum in minimums.items():
            self.assertGreaterEqual(counts[category], minimum, category)

    def test_horizontal_pipes_are_low_enough_for_visual_navigation_test(self):
        horizontal_pipes = [
            item
            for item in self.layout.PRIMITIVES
            if item["category"] == "horizontal_pipe"
        ]
        self.assertGreaterEqual(len(horizontal_pipes), 12)
        for pipe in horizontal_pipes:
            self.assertGreaterEqual(pipe["position"][2], 0.7, pipe["path"])
            self.assertLessEqual(pipe["position"][2], 1.4, pipe["path"])

    def test_primitive_schema_paths_and_bounds(self):
        paths = [item["path"] for item in self.layout.PRIMITIVES]
        self.assertEqual(len(paths), len(set(paths)))
        half_x = self.layout.ARENA_SIZE[0] / 2.0
        half_y = self.layout.ARENA_SIZE[1] / 2.0

        for item in self.layout.PRIMITIVES:
            self.assertTrue(item["path"].startswith("/Factory/"), item["path"])
            self.assertIn(item["shape"], {"box", "cylinder"})
            self.assertEqual(len(item["position"]), 3)
            self.assertEqual(len(item["rotation"]), 3)
            self.assertEqual(len(item["color"]), 3)
            self.assertIsInstance(item["collision"], bool)
            width, depth = item["footprint"]
            self.assertGreater(width, 0.0, item["path"])
            self.assertGreater(depth, 0.0, item["path"])
            x, y, z = item["position"]
            self.assertGreaterEqual(x - width / 2.0, -half_x - 1e-9, item["path"])
            self.assertLessEqual(x + width / 2.0, half_x + 1e-9, item["path"])
            self.assertGreaterEqual(y - depth / 2.0, -half_y - 1e-9, item["path"])
            self.assertLessEqual(y + depth / 2.0, half_y + 1e-9, item["path"])
            self.assertGreaterEqual(z, 0.0, item["path"])
            if item["shape"] == "box":
                self.assertEqual(len(item["size"]), 3)
                self.assertTrue(all(value > 0.0 for value in item["size"]))
            else:
                self.assertGreater(item["radius"], 0.0)
                self.assertGreater(item["height"], 0.0)

    def test_spawn_and_goals_are_not_inside_obstacle_footprints(self):
        points = [self.layout.ROBOT_SPAWN[:2]] + [goal[:2] for goal in self.layout.NAV_GOALS]
        for point in points:
            for item in self.layout.PRIMITIVES:
                if not item["collision"]:
                    continue
                x, y = item["position"][:2]
                width, depth = item["footprint"]
                inside = (
                    abs(point[0] - x) < width / 2.0
                    and abs(point[1] - y) < depth / 2.0
                )
                self.assertFalse(inside, f"{point} intersects {item['path']}")


if __name__ == "__main__":
    unittest.main()
