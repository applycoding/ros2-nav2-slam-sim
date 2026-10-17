"""Write the world, map, and sample waypoints from indoor_layout."""

from __future__ import annotations

import sys
from pathlib import Path

import indoor_layout as layout


ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = ROOT / "src" / "indoor_robot_gazebo" / "worlds" / "indoor_office.world"
MAP_PGM_PATH = ROOT / "src" / "indoor_robot_nav" / "maps" / "indoor_office.pgm"
MAP_YAML_PATH = ROOT / "src" / "indoor_robot_nav" / "maps" / "indoor_office.yaml"
WAYPOINT_PATH = ROOT / "src" / "indoor_robot_mission" / "config" / "waypoints.yaml"


def _fmt(value: float) -> str:
    return f"{value:.4f}"


def render_world() -> str:
    models: list[str] = []
    visuals = list(layout.collision_boxes()) + [layout.dock_marker()]
    for box in visuals:
        color = "0.16 0.45 0.28 1" if box.name == "dock_marker" else "0.72 0.70 0.64 1"
        collision = ""
        if box.collision:
            collision = f"""
      <collision name="collision">
        <geometry>
          <box>
            <size>{_fmt(box.sx)} {_fmt(box.sy)} {_fmt(box.sz)}</size>
          </box>
        </geometry>
      </collision>"""
        models.append(
            f"""
    <model name="{box.name}">
      <static>true</static>
      <pose>{_fmt(box.x)} {_fmt(box.y)} {_fmt(box.z)} 0 0 0</pose>
      <link name="link">{collision}
        <visual name="visual">
          <geometry>
            <box>
              <size>{_fmt(box.sx)} {_fmt(box.sy)} {_fmt(box.sz)}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
        )
    body = "".join(models)
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="indoor_office">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <gravity>0 0 -9.8</gravity>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.70 0.78 0.85 1</background>
      <shadows>true</shadows>
    </scene>
{body}
  </world>
</sdf>
"""


def render_pgm() -> str:
    width, height = layout.map_size()
    lines = ["P2", f"{width} {height}", "255"]
    for row in layout.raster():
        values = [str(value) for value in row]
        for offset in range(0, len(values), 20):
            lines.append(" ".join(values[offset : offset + 20]))
    return "\n".join(lines) + "\n"


def render_map_yaml() -> str:
    return f"""image: indoor_office.pgm
mode: trinary
resolution: {layout.RESOLUTION:.2f}
origin: [{layout.ORIGIN_X:.1f}, {layout.ORIGIN_Y:.1f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""


def render_waypoints() -> str:
    rendered_waypoints = []
    for name, x, y, yaw in layout.MISSION_WAYPOINTS:
        rendered_waypoints.append(
            "  - name: {name}\n    x: {x:.2f}\n    y: {y:.2f}\n    yaw: {yaw:.4f}".format(
                name=name, x=x, y=y, yaw=yaw
            )
        )
    dock_name, dock_x, dock_y, dock_yaw = layout.DOCK
    return f"""frame_id: map
start:
  x: {layout.SPAWN_X:.2f}
  y: {layout.SPAWN_Y:.2f}
  yaw: {layout.SPAWN_YAW:.4f}
dock:
  name: {dock_name}
  x: {dock_x:.2f}
  y: {dock_y:.2f}
  yaw: {dock_yaw:.4f}
waypoints:
{chr(10).join(rendered_waypoints)}
"""


def _check_poses() -> None:
    poses = [layout.OFFICE, layout.KITCHEN, layout.LOBBY, layout.DOCK]
    poses.append(("spawn", layout.SPAWN_X, layout.SPAWN_Y, layout.SPAWN_YAW))
    for name, x, y, _yaw in poses:
        kind = layout.classify(x, y)
        if kind != "free":
            raise SystemExit(f"{name} at ({x}, {y}) is {kind}, expected free")


def write_assets() -> None:
    _check_poses()
    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    MAP_PGM_PATH.parent.mkdir(parents=True, exist_ok=True)
    WAYPOINT_PATH.parent.mkdir(parents=True, exist_ok=True)
    WORLD_PATH.write_text(render_world(), encoding="utf-8")
    MAP_PGM_PATH.write_text(render_pgm(), encoding="utf-8")
    MAP_YAML_PATH.write_text(render_map_yaml(), encoding="utf-8")
    WAYPOINT_PATH.write_text(render_waypoints(), encoding="utf-8")


def main() -> None:
    write_assets()
    print(f"wrote {WORLD_PATH.relative_to(ROOT)}")
    print(f"wrote {MAP_PGM_PATH.relative_to(ROOT)}")
    print(f"wrote {MAP_YAML_PATH.relative_to(ROOT)}")
    print(f"wrote {WAYPOINT_PATH.relative_to(ROOT)}")


if __name__ == "__main__":
    sys.exit(main())
