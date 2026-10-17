"""Small 2D pose helpers shared by the parser and the mission runner."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


def distance(left: Pose2D, right: Pose2D) -> float:
    return math.hypot(left.x - right.x, left.y - right.y)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return x, y, z, w for a yaw rotation about Z."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))
