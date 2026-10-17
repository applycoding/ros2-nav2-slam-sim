"""Waypoint YAML parsing. No ROS imports."""

from pathlib import Path

import pytest
import yaml

import indoor_layout
from indoor_robot_mission.waypoints import (
    WaypointConfigError,
    load_waypoints,
    parse_waypoints,
)


ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT / "src" / "indoor_robot_mission" / "config" / "waypoints.yaml"


def test_sample_file_matches_the_floor_plan():
    plan = load_waypoints(SAMPLE)
    assert plan.frame_id == "map"
    assert [waypoint.name for waypoint in plan.waypoints] == ["office", "kitchen", "lobby"]
    assert plan.dock is not None
    assert plan.dock.name == "dock"
    assert plan.start.x == pytest.approx(indoor_layout.SPAWN_X)
    assert plan.start.y == pytest.approx(indoor_layout.SPAWN_Y)
    assert plan.start.yaw == pytest.approx(indoor_layout.SPAWN_YAW)
    for waypoint, spec in zip(plan.waypoints, indoor_layout.MISSION_WAYPOINTS):
        assert waypoint.name == spec[0]
        assert waypoint.x == pytest.approx(spec[1])
        assert waypoint.y == pytest.approx(spec[2])
        assert waypoint.yaw == pytest.approx(spec[3])
        assert waypoint.frame_id == "map"


def test_yaw_and_frame_default():
    plan = parse_waypoints(
        """
        waypoints:
          - name: lobby
            x: 1
            y: -2
        """
    )
    assert plan.frame_id == "map"
    assert plan.waypoints[0].yaw == 0.0
    assert plan.start.x == pytest.approx(1.0)
    assert plan.start.y == pytest.approx(-2.0)
    assert plan.dock is None


def test_explicit_frame_and_start():
    plan = parse_waypoints(
        """
        frame_id: odom
        start: {x: 0.5, y: 0.25, yaw: 1.0}
        waypoints:
          - {name: a, x: 1, y: 2, yaw: 0.3}
        """
    )
    assert plan.frame_id == "odom"
    assert plan.waypoints[0].frame_id == "odom"
    assert plan.start.x == pytest.approx(0.5)
    assert plan.start.yaw == pytest.approx(1.0)


def test_duplicate_names_rejected():
    with pytest.raises(WaypointConfigError, match="duplicate"):
        parse_waypoints(
            """
            waypoints:
              - {name: office, x: 0, y: 0}
              - {name: office, x: 1, y: 1}
            """
        )


def test_missing_coordinate_rejected():
    with pytest.raises(WaypointConfigError, match="missing x"):
        parse_waypoints(
            """
            waypoints:
              - {name: office, y: 1}
            """
        )


def test_boolean_is_not_a_coordinate():
    with pytest.raises(WaypointConfigError, match="must be a number"):
        parse_waypoints(
            """
            waypoints:
              - {name: office, x: true, y: 1}
            """
        )


def test_empty_name_rejected():
    with pytest.raises(WaypointConfigError, match="name"):
        parse_waypoints(
            """
            waypoints:
              - {name: "  ", x: 0, y: 0}
            """
        )


def test_unknown_key_rejected():
    with pytest.raises(WaypointConfigError, match="unknown keys"):
        parse_waypoints(
            """
            waypoints: []
            speed: 1
            """
        )


def test_waypoint_file_must_be_a_mapping():
    with pytest.raises(WaypointConfigError, match="mapping"):
        parse_waypoints("- just a list\n")


def test_invalid_yaml():
    with pytest.raises(WaypointConfigError, match="invalid YAML"):
        parse_waypoints("waypoints: [\n")


def test_dock_name_cannot_collide():
    with pytest.raises(WaypointConfigError, match="collides"):
        parse_waypoints(
            """
            dock: {name: office, x: 0, y: 0}
            waypoints:
              - {name: office, x: 1, y: 1}
            """
        )


def test_empty_waypoint_list_parses():
    plan = parse_waypoints("waypoints: []\n")
    assert plan.waypoints == ()
    assert plan.start.x == 0.0


def test_missing_file():
    with pytest.raises(WaypointConfigError, match="cannot read"):
        load_waypoints(ROOT / "config" / "missing.yaml")


def test_sample_file_is_plain_yaml():
    loaded = yaml.safe_load(SAMPLE.read_text(encoding="utf-8"))
    assert isinstance(loaded["waypoints"], list)
