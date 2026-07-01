#!/usr/bin/env python3
"""Deterministic 2x2 room benchmark for controlled SLAM/Nav2 tests."""

from math import pi


ROOM_SIZE = 6.0
ARENA_SIZE = (12.0, 12.0)
WALL_THICKNESS = 0.16
WALL_HEIGHT = 2.4
DOOR_WIDTH = 1.8

# Same spawn height convention as the factory runner: the robot is dropped after
# physics starts, so keep enough clearance to avoid initial mesh-floor overlap.
ROBOT_CLEARANCE_Z = 0.6
ROBOT_SPAWN = (-4.5, -4.5, ROBOT_CLEARANCE_Z, 0.0)
NAV_GOALS = (
    (-1.4, -4.4, 0.0),
    (3.8, -4.3, pi / 2.0),
    (3.8, 3.8, pi),
    (-4.4, 3.8, -pi / 2.0),
)

CONCRETE = (0.42, 0.45, 0.48)
FLOOR_GRAY = (0.34, 0.36, 0.35)
PIPE_BLUE = (0.10, 0.32, 0.55)
PIPE_GREEN = (0.12, 0.42, 0.26)
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


def cylinder(
    path,
    category,
    position,
    radius,
    height,
    color,
    rotation=(0.0, 0.0, 0.0),
    axis="z",
):
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


def wall(path, position, size):
    return box(path, "wall", position, size, CONCRETE)


items = [
    # Outer walls.
    wall("/Factory/Walls/North", (0.0, 6.0, WALL_HEIGHT / 2.0), (12.0, WALL_THICKNESS, WALL_HEIGHT)),
    wall("/Factory/Walls/South", (0.0, -6.0, WALL_HEIGHT / 2.0), (12.0, WALL_THICKNESS, WALL_HEIGHT)),
    wall("/Factory/Walls/West", (-6.0, 0.0, WALL_HEIGHT / 2.0), (WALL_THICKNESS, 12.0, WALL_HEIGHT)),
    wall("/Factory/Walls/East", (6.0, 0.0, WALL_HEIGHT / 2.0), (WALL_THICKNESS, 12.0, WALL_HEIGHT)),
    # Interior vertical wall at x=0 with door gaps connecting room 1-2 and 3-4.
    wall("/Factory/Walls/CenterVerticalSouthEnd", (0.0, -5.025, WALL_HEIGHT / 2.0), (WALL_THICKNESS, 1.95, WALL_HEIGHT)),
    wall("/Factory/Walls/CenterVerticalMiddle", (0.0, 0.0, WALL_HEIGHT / 2.0), (WALL_THICKNESS, 4.2, WALL_HEIGHT)),
    wall("/Factory/Walls/CenterVerticalNorthEnd", (0.0, 5.025, WALL_HEIGHT / 2.0), (WALL_THICKNESS, 1.95, WALL_HEIGHT)),
    # Interior horizontal wall at y=0 with door gaps connecting room 1-3 and 2-4.
    wall("/Factory/Walls/CenterHorizontalWestEnd", (-5.025, 0.0, WALL_HEIGHT / 2.0), (1.95, WALL_THICKNESS, WALL_HEIGHT)),
    wall("/Factory/Walls/CenterHorizontalMiddle", (0.0, 0.0, WALL_HEIGHT / 2.0), (4.2, WALL_THICKNESS, WALL_HEIGHT)),
    wall("/Factory/Walls/CenterHorizontalEastEnd", (5.025, 0.0, WALL_HEIGHT / 2.0), (1.95, WALL_THICKNESS, WALL_HEIGHT)),
]

# Room 1: two 1 m vertical pipes in the middle, with horizontal pipes from their
# tops to the nearest wall.
room1_pipe_tops = ((-5.4, -3.0),(-5.0, -3.0),(-4.6, -3.0),
                   (-4.2, -3.0),(-3.8, -3.0),(-3.4, -3.0), 
                   (-3.0, -3.0),(-2.6, -3.0))
for index, (x, y) in enumerate(room1_pipe_tops, 1):
    items.append(
        cylinder(
            f"/Factory/Room1/VerticalPipe{index:02d}",
            "vertical_pipe",
            (x, y, 0.5),
            0.07,
            1.0,
            PIPE_GREEN,
        )
    )
    length_to_west_wall = x - (-6.0)
    items.append(
        cylinder(
            f"/Factory/Room1/HorizontalPipeToWall{index:02d}",
            "horizontal_pipe",
            ((x - length_to_west_wall / 2.0), y, 1.0),
            0.055,
            length_to_west_wall,
            PIPE_BLUE,
            (0.0, pi / 2.0, 0.0),
            "x",
        )
    )

# Room 2: four vertical pipes in a line, 0.5 m from the south wall.
for index, x in enumerate((1.4, 2.5, 3.6, 4.7), 1):
    items.append(
        cylinder(
            f"/Factory/Room2/WallLinePipe{index:02d}",
            "vertical_pipe",
            (x, -5.5, 0.75),
            0.07,
            1.5,
            PIPE_GREEN,
        )
    )

# Room 3: one tank, 1 m diameter, 0.7 m height.
items.append(
    cylinder(
        "/Factory/Room3/Tank01",
        "tank",
        (-3.0, 3.0, 0.35),
        0.5,
        0.7,
        TANK_GRAY,
    )
)

# Room 4: two pipe lines near different walls. One line includes a horizontal
# pipe connecting all pipes in that line.
room4_south_line = ((1.3, 5.5), (2.4, 5.5), (3.5, 5.5), (4.6, 5.5))
for index, (x, y) in enumerate(room4_south_line, 1):
    items.append(
        cylinder(
            f"/Factory/Room4/SouthLinePipe{index:02d}",
            "vertical_pipe",
            (x, y, 0.75),
            0.07,
            1.5,
            PIPE_GREEN,
        )
    )
items.append(
    cylinder(
        "/Factory/Room4/SouthLineHorizontalConnector",
        "horizontal_pipe",
        (2.95, 5.5, 1.5),
        0.055,
        3.3,
        PIPE_BLUE,
        (0.0, pi / 2.0, 0.0),
        "x",
    )
)

for index, y in enumerate((1.3, 2.4, 3.5, 4.6), 1):
    items.append(
        cylinder(
            f"/Factory/Room4/EastLinePipe{index:02d}",
            "vertical_pipe",
            (5.5, y, 0.75),
            0.07,
            1.5,
            PIPE_GREEN,
        )
    )

PRIMITIVES = tuple(items)
