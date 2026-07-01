#!/usr/bin/env python3
"""Deterministic primitive layout for the compact factory benchmark."""

from math import pi


ARENA_SIZE = (10.0, 8.0)
MAIN_AISLE_WIDTH = 1.2
SERVICE_AISLE_WIDTH = 0.8
# The imported chassis collision mesh extends below the older 0.242 m spawn,
# so keep a small clearance above the ground plane to avoid interpenetration.
ROBOT_CLEARANCE_Z = 0.3
ROBOT_SPAWN = (-3.8, -2.8, ROBOT_CLEARANCE_Z, 0.0)
NAV_GOALS = (
    (-0.5, -2.8, 0.0),
    (3.7, -2.4, pi / 2.0),
    (3.4, 2.7, pi),
)

CLEARANCE_ZONES = (
    {"name": "south_main_aisle", "kind": "main", "width": 1.2, "bounds": (-4.2, 4.2, -3.4, -2.2)},
    {"name": "central_main_aisle", "kind": "main", "width": 1.2, "bounds": (-1.6, -0.4, -2.2, 3.2)},
    {"name": "east_service_corridor", "kind": "service", "width": 0.8, "bounds": (3.3, 4.1, -2.2, 1.0)},
)

CONCRETE = (0.42, 0.45, 0.48)
STEEL = (0.25, 0.30, 0.34)
PIPE_BLUE = (0.10, 0.32, 0.55)
PIPE_GREEN = (0.12, 0.42, 0.26)
SAFETY_RED = (0.72, 0.08, 0.05)
CABINET_GRAY = (0.58, 0.61, 0.60)
TANK_GRAY = (0.48, 0.53, 0.55)


def box(path, category, position, size, color, collision=True):
    return {
        "path": path,
        "category": category,
        "shape": "box",
        "position": tuple(position),
        "rotation": (0.0, 0.0, 0.0),
        "size": tuple(size),
        "footprint": (size[0], size[1]),
        "color": tuple(color),
        "collision": collision,
    }


def cylinder(path, category, position, radius, height, color, rotation=(0.0, 0.0, 0.0), axis="z"):
    if axis == "x":
        footprint = (height, radius * 2.0)
    elif axis == "y":
        footprint = (radius * 2.0, height)
    else:
        footprint = (radius * 2.0, radius * 2.0)
    return {
        "path": path,
        "category": category,
        "shape": "cylinder",
        "position": tuple(position),
        "rotation": tuple(rotation),
        "radius": radius,
        "height": height,
        "footprint": footprint,
        "color": tuple(color),
        "collision": True,
    }


items = [
    box("/Factory/Walls/North", "perimeter_wall", (0.0, 3.9, 1.25), (10.0, 0.2, 2.5), CONCRETE),
    box("/Factory/Walls/South", "perimeter_wall", (0.0, -3.9, 1.25), (10.0, 0.2, 2.5), CONCRETE),
    box("/Factory/Walls/West", "perimeter_wall", (-4.9, 0.0, 1.25), (0.2, 8.0, 2.5), CONCRETE),
    box("/Factory/Walls/East", "perimeter_wall", (4.9, 0.0, 1.25), (0.2, 8.0, 2.5), CONCRETE),
    box("/Factory/Walls/PartitionWest", "partition_wall", (-1.8, 1.8, 1.1), (0.18, 3.4, 2.2), CONCRETE),
    box("/Factory/Walls/PartitionEast", "partition_wall", (2.2, 0.35, 1.1), (0.18, 2.5, 2.2), CONCRETE),
]

for index, position in enumerate(((-3.5, 0.0), (-3.5, 2.8), (0.3, 2.8), (3.2, 1.8), (4.2, 0.7), (0.2, -0.6)), 1):
    items.append(cylinder(f"/Factory/Structure/Column{index:02d}", "column", (*position, 1.3), 0.16, 2.6, CONCRETE))

for index, position in enumerate(((-4.35, 2.8), (-4.35, 1.8), (1.0, 3.45), (2.0, 3.45)), 1):
    items.append(box(f"/Factory/Equipment/Cabinet{index:02d}", "cabinet", (*position, 0.8), (0.55, 0.45, 1.6), CABINET_GRAY))

for index, (x, y, radius, height) in enumerate(((-2.8, 1.45, 0.42, 1.8), (2.9, 1.25, 0.48, 2.0), (4.15, 2.45, 0.38, 1.65)), 1):
    items.append(cylinder(f"/Factory/Equipment/Tank{index:02d}", "tank", (x, y, height / 2.0), radius, height, TANK_GRAY))

for index, position in enumerate(((-2.8, -0.9), (1.1, 1.15)), 1):
    items.append(box(f"/Factory/Equipment/Pump{index:02d}", "pump", (*position, 0.35), (0.75, 0.55, 0.7), STEEL))

for index, position in enumerate(((-0.8, 2.7), (1.4, 2.7), (2.8, -0.8), (4.1, -0.8)), 1):
    items.append(box(f"/Factory/PipeRacks/Rack{index:02d}", "pipe_rack", (*position, 1.25), (0.12, 0.6, 2.5), STEEL))

vertical_positions = (
    (-3.8, 3.25), (-3.35, 3.25), (-2.9, 3.25), (-2.45, 3.25),
    (-1.25, 2.75), (-0.8, 2.75), (1.15, 2.75), (1.55, 2.75),
    (2.55, 1.25), (3.2, 1.25), (-3.05, -0.4), (-2.5, -0.4),
)
for index, position in enumerate(vertical_positions, 1):
    items.append(cylinder(f"/Factory/Pipes/Vertical{index:02d}", "vertical_pipe", (*position, 1.25), 0.075, 2.5, PIPE_GREEN))

horizontal_specs = (
    (-3.1, 3.25, 1.2, "x", 2.0), (-3.1, 3.05, 1.4, "x", 2.0),
    (0.2, 2.75, 1.1, "x", 2.8), (0.2, 2.55, 1.35, "x", 2.8),
    (3.15, 1.25, 1.0, "y", 2.7), (3.35, 1.25, 1.3, "y", 2.7),
    (-2.75, -0.4, 0.7, "x", 1.2), (-2.75, -0.6, 0.9, "x", 1.2),
    (2.9, -0.8, 1.1, "x", 1.0), (4.1, -0.8, 1.35, "y", 1.8),
    (-4.35, 0.2, 0.7, "y", 2.0), (1.75, 1.1, 0.7, "y", 1.4),
)
for index, (x, y, z, axis, length) in enumerate(horizontal_specs, 1):
    rotation = (0.0, pi / 2.0, 0.0) if axis == "x" else (pi / 2.0, 0.0, 0.0)
    items.append(cylinder(f"/Factory/Pipes/Horizontal{index:02d}", "horizontal_pipe", (x, y, z), 0.065, length, PIPE_BLUE, rotation, axis))

guardrails = (
    (2.35, 2.15, 1.4, 0.07), (3.55, 2.15, 1.0, 0.07),
    (2.15, 2.65, 0.07, 1.0), (3.75, 2.65, 0.07, 1.0),
)
for index, (x, y, width, depth) in enumerate(guardrails, 1):
    items.append(box(f"/Factory/Safety/Guardrail{index:02d}", "guardrail", (x, y, 0.55), (width, depth, 1.1), SAFETY_RED))

PRIMITIVES = tuple(items)
