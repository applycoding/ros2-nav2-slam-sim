"""Load a mission YAML file into waypoints.

The schema is intentionally small:

    frame_id: map
    start: {x, y, yaw}
    dock: {name, x, y, yaw}    # optional
    waypoints:
      - {name, x, y, yaw}
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from indoor_robot_mission.geometry import Pose2D


class WaypointConfigError(ValueError):
    """The waypoint file is missing, malformed, or fails validation."""


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    frame_id: str

    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw)


@dataclass(frozen=True)
class MissionPlan:
    frame_id: str
    waypoints: tuple[Waypoint, ...]
    start: Pose2D
    dock: Waypoint | None


_TOP_LEVEL_KEYS = frozenset({"frame_id", "start", "dock", "waypoints"})


def load_waypoints(path: str | Path) -> MissionPlan:
    file_path = Path(path)
    try:
        text = file_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise WaypointConfigError(f"cannot read {file_path}: {exc}") from exc
    return parse_waypoints(text)


def parse_waypoints(text: str) -> MissionPlan:
    try:
        loaded = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise WaypointConfigError(f"invalid YAML: {exc}") from exc
    if not isinstance(loaded, dict):
        raise WaypointConfigError("waypoint file must be a mapping")
    unknown = set(loaded) - _TOP_LEVEL_KEYS
    if unknown:
        names = ", ".join(sorted(str(key) for key in unknown))
        raise WaypointConfigError(f"unknown keys: {names}")
    if "waypoints" not in loaded:
        raise WaypointConfigError("waypoints is required")

    frame_id = loaded.get("frame_id", "map")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise WaypointConfigError("frame_id must be a non-empty string")
    frame_id = frame_id.strip()

    raw_waypoints = loaded["waypoints"]
    if not isinstance(raw_waypoints, list):
        raise WaypointConfigError("waypoints must be a list")
    waypoints = tuple(
        _parse_waypoint(item, index, frame_id) for index, item in enumerate(raw_waypoints)
    )
    _require_unique_names([waypoint.name for waypoint in waypoints], "waypoint")

    dock = None
    if "dock" in loaded and loaded["dock"] is not None:
        dock = _parse_waypoint(loaded["dock"], 0, frame_id, label="dock")
        if dock.name in {waypoint.name for waypoint in waypoints}:
            raise WaypointConfigError(f"dock name {dock.name!r} collides with a waypoint")

    if "start" in loaded and loaded["start"] is not None:
        start = _parse_pose(loaded["start"], label="start")
    elif waypoints:
        start = waypoints[0].pose()
    else:
        start = Pose2D(0.0, 0.0, 0.0)

    return MissionPlan(frame_id=frame_id, waypoints=waypoints, start=start, dock=dock)


def _parse_waypoint(raw: object, index: int, frame_id: str, label: str = "waypoint") -> Waypoint:
    where = label if label == "dock" else f"waypoint {index}"
    if not isinstance(raw, dict):
        raise WaypointConfigError(f"{where} must be a mapping")
    allowed = {"name", "x", "y", "yaw"}
    unknown = set(raw) - allowed
    if unknown:
        names = ", ".join(sorted(str(key) for key in unknown))
        raise WaypointConfigError(f"{where} has unknown keys: {names}")
    name = raw.get("name")
    if not isinstance(name, str) or not name.strip():
        raise WaypointConfigError(f"{where} name must be a non-empty string")
    yaw = _number(raw, "yaw", where, default=0.0)
    return Waypoint(
        name=name.strip(),
        x=_number(raw, "x", where),
        y=_number(raw, "y", where),
        yaw=yaw if yaw is not None else 0.0,
        frame_id=frame_id,
    )


def _parse_pose(raw: object, label: str) -> Pose2D:
    if not isinstance(raw, dict):
        raise WaypointConfigError(f"{label} must be a mapping")
    allowed = {"x", "y", "yaw"}
    unknown = set(raw) - allowed
    if unknown:
        names = ", ".join(sorted(str(key) for key in unknown))
        raise WaypointConfigError(f"{label} has unknown keys: {names}")
    yaw = _number(raw, "yaw", label, default=0.0)
    return Pose2D(
        x=_number(raw, "x", label),
        y=_number(raw, "y", label),
        yaw=yaw if yaw is not None else 0.0,
    )


def _number(
    raw: dict[str, object],
    key: str,
    where: str,
    default: float | None = None,
) -> float | None:
    if key not in raw:
        if default is None:
            raise WaypointConfigError(f"{where} is missing {key}")
        return default
    value = raw[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointConfigError(f"{where} {key} must be a number")
    return float(value)


def _require_unique_names(names: list[str], label: str) -> None:
    seen: set[str] = set()
    for name in names:
        if name in seen:
            raise WaypointConfigError(f"duplicate {label} name: {name}")
        seen.add(name)
