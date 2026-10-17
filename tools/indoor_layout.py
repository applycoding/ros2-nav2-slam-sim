"""Authored geometry for the indoor office.

The Gazebo world, the navigation map, and the sample waypoint file are all
generated from this module so the floor plan cannot drift between them.
"""

from __future__ import annotations

from dataclasses import dataclass


RESOLUTION = 0.05
ORIGIN_X = -4.0
ORIGIN_Y = -3.0
MAP_WIDTH_M = 8.0
MAP_HEIGHT_M = 6.0

WALL_THICKNESS = 0.12
WALL_HEIGHT = 0.80

INNER_MIN_X = -3.40
INNER_MAX_X = 3.40
INNER_MIN_Y = -2.40
INNER_MAX_Y = 2.40

DOOR_HALF_WIDTH = 0.40

SPAWN_X = -1.70
SPAWN_Y = -1.55
SPAWN_YAW = 1.5708

OFFICE = ("office", -1.70, 1.45, 0.0)
KITCHEN = ("kitchen", 1.70, 1.45, 3.1416)
LOBBY = ("lobby", SPAWN_X, SPAWN_Y, SPAWN_YAW)
DOCK = ("dock", 1.70, -1.55, SPAWN_YAW)

MISSION_WAYPOINTS = (OFFICE, KITCHEN, LOBBY)


@dataclass(frozen=True)
class Box:
    """Axis-aligned box. x/y/z are the center, s* are full sizes."""

    name: str
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float
    collision: bool = True


def _wall(name: str, x: float, y: float, sx: float, sy: float) -> Box:
    return Box(name, x, y, WALL_HEIGHT / 2.0, sx, sy, WALL_HEIGHT, collision=True)


def _segments_with_gaps(
    start: float,
    end: float,
    gaps: tuple[tuple[float, float], ...],
) -> list[tuple[float, float]]:
    """Return [start, end] intervals after removing inclusive gaps."""
    cursor = start
    pieces: list[tuple[float, float]] = []
    for gap_start, gap_end in gaps:
        if gap_start > cursor:
            pieces.append((cursor, gap_start))
        cursor = max(cursor, gap_end)
    if cursor < end:
        pieces.append((cursor, end))
    return pieces


def collision_boxes() -> tuple[Box, ...]:
    """Walls and furniture that block the robot and the occupancy grid."""
    thickness = WALL_THICKNESS
    span_x = (INNER_MAX_X - INNER_MIN_X) + (2.0 * thickness)
    span_y = (INNER_MAX_Y - INNER_MIN_Y) + (2.0 * thickness)
    mid_x = (INNER_MIN_X + INNER_MAX_X) / 2.0
    mid_y = (INNER_MIN_Y + INNER_MAX_Y) / 2.0
    boxes = [
        _wall("wall_south", mid_x, INNER_MIN_Y - thickness / 2.0, span_x, thickness),
        _wall("wall_north", mid_x, INNER_MAX_Y + thickness / 2.0, span_x, thickness),
        _wall("wall_west", INNER_MIN_X - thickness / 2.0, mid_y, thickness, span_y),
        _wall("wall_east", INNER_MAX_X + thickness / 2.0, mid_y, thickness, span_y),
    ]

    hallway_south = -0.70
    hallway_north = 0.50
    door_gaps = ((-2.10, -1.30), (1.30, 2.10))
    for index, (x0, x1) in enumerate(
        _segments_with_gaps(INNER_MIN_X, INNER_MAX_X, door_gaps)
    ):
        center = (x0 + x1) / 2.0
        length = x1 - x0
        boxes.append(_wall(f"wall_hall_north_{index}", center, hallway_north, length, thickness))
        boxes.append(_wall(f"wall_hall_south_{index}", center, hallway_south, length, thickness))

    boxes.append(
        _wall(
            "wall_office_kitchen",
            0.0,
            (hallway_north + INNER_MAX_Y) / 2.0,
            thickness,
            INNER_MAX_Y - hallway_north,
        )
    )
    boxes.append(
        _wall(
            "wall_lobby_dock",
            0.0,
            (INNER_MIN_Y + hallway_south) / 2.0,
            thickness,
            hallway_south - INNER_MIN_Y,
        )
    )
    boxes.extend(
        [
            Box("table_office", -2.45, 2.00, 0.25, 0.70, 0.40, 0.50),
            Box("counter_kitchen", 2.45, 2.00, 0.30, 0.80, 0.40, 0.60),
        ]
    )
    return tuple(boxes)


def dock_marker() -> Box:
    """Flat visual pad. It is not an obstacle."""
    return Box("dock_marker", DOCK[1], DOCK[2], 0.005, 0.45, 0.45, 0.01, collision=False)


def classify(x: float, y: float) -> str:
    """Return occupied, free, or unknown for a world point."""
    for box in collision_boxes():
        if abs(x - box.x) <= box.sx / 2.0 and abs(y - box.y) <= box.sy / 2.0:
            return "occupied"
    if INNER_MIN_X < x < INNER_MAX_X and INNER_MIN_Y < y < INNER_MAX_Y:
        return "free"
    return "unknown"


def map_size() -> tuple[int, int]:
    width = int(round(MAP_WIDTH_M / RESOLUTION))
    height = int(round(MAP_HEIGHT_M / RESOLUTION))
    return width, height


def cell_center(column: int, row_from_bottom: int) -> tuple[float, float]:
    x = ORIGIN_X + (column + 0.5) * RESOLUTION
    y = ORIGIN_Y + (row_from_bottom + 0.5) * RESOLUTION
    return x, y


def raster() -> list[list[int]]:
    """Return PGM gray values, row 0 at the top (maximum y).

    0 is occupied, 254 is free, and 205 is unknown. Those values match a
    trinary map_server image with negate 0.
    """
    width, height = map_size()
    shades = {"occupied": 0, "free": 254, "unknown": 205}
    rows: list[list[int]] = []
    for row_from_top in range(height):
        row_from_bottom = height - 1 - row_from_top
        row: list[int] = []
        for column in range(width):
            x, y = cell_center(column, row_from_bottom)
            row.append(shades[classify(x, y)])
        rows.append(row)
    return rows
