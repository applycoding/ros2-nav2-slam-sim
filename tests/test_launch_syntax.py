"""Syntax checks for launch files, XML, YAML, and the CI workflow.

These tests do not import ROS or start a simulator.
"""

import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import indoor_layout
from generate_assets import render_map_yaml, render_pgm, render_waypoints, render_world


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def _python_files() -> list[Path]:
    return sorted(SRC.rglob("*.py"))


def _launch_files() -> list[Path]:
    return sorted(SRC.rglob("*.launch.py"))


def test_python_sources_parse():
    for path in _python_files():
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_launch_files_define_generate_launch_description():
    launches = _launch_files()
    assert len(launches) >= 6
    for path in launches:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        names = [node.name for node in tree.body if isinstance(node, ast.FunctionDef)]
        assert "generate_launch_description" in names, path.name


def test_expected_launch_entry_points_exist():
    relative = {path.relative_to(SRC).as_posix() for path in _launch_files()}
    expected = {
        "indoor_robot_description/launch/description.launch.py",
        "indoor_robot_gazebo/launch/world.launch.py",
        "indoor_robot_slam/launch/slam.launch.py",
        "indoor_robot_slam/launch/mapping_sim.launch.py",
        "indoor_robot_nav/launch/navigation.launch.py",
        "indoor_robot_nav/launch/navigation_sim.launch.py",
        "indoor_robot_mission/launch/mission.launch.py",
    }
    assert expected <= relative


def test_package_xml_files_are_well_formed_and_unique():
    names = []
    for path in sorted(SRC.glob("*/package.xml")):
        root = ET.parse(path).getroot()
        assert root.tag == "package"
        name = root.findtext("name")
        assert name == path.parent.name
        assert root.findtext("license") == "MIT"
        names.append(name)
    assert names == [
        "indoor_robot_description",
        "indoor_robot_gazebo",
        "indoor_robot_mission",
        "indoor_robot_nav",
        "indoor_robot_slam",
    ]


def test_xml_models_and_yaml_configs_parse():
    xacro = SRC / "indoor_robot_description" / "urdf" / "indoor_robot.urdf.xacro"
    robot = ET.parse(xacro).getroot()
    assert robot.tag == "robot"
    joint_names = {element.get("name") for element in robot.iter("joint")}
    assert {"base_joint", "laser_joint", "caster_joint", "imu_joint"} <= joint_names
    description = xacro.read_text(encoding="utf-8")
    assert 'prefix="left"' in description
    assert 'prefix="right"' in description
    assert "<left_joint>left_wheel_joint</left_joint>" in description
    assert "<right_joint>right_wheel_joint</right_joint>" in description
    plugins = {element.get("filename") for element in robot.iter("plugin")}
    assert "libgazebo_ros_diff_drive.so" in plugins
    assert "libgazebo_ros_ray_sensor.so" in plugins

    world = ET.parse(SRC / "indoor_robot_gazebo" / "worlds" / "indoor_office.world").getroot()
    assert world.tag == "sdf"
    model_names = {element.get("name") for element in world.iter("model")}
    for box in indoor_layout.collision_boxes():
        assert box.name in model_names

    configs = list(SRC.rglob("*.yaml")) + list(SRC.rglob("*.rviz"))
    assert configs
    for path in configs:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert loaded is not None


def test_slam_and_nav_parameters_match_this_robot():
    slam = yaml.safe_load(
        (SRC / "indoor_robot_slam" / "config" / "mapper_params_online_async.yaml").read_text(
            encoding="utf-8"
        )
    )["slam_toolbox"]["ros__parameters"]
    assert slam["solver_plugin"] == "solver_plugins::CeresSolver"
    assert slam["base_frame"] == "base_footprint"
    assert slam["odom_frame"] == "odom"
    assert slam["scan_topic"] == "/scan"
    assert slam["mode"] == "mapping"
    assert slam["max_laser_range"] == pytest.approx(8.0)
    assert slam["do_loop_closing"] is True

    nav = yaml.safe_load(
        (SRC / "indoor_robot_nav" / "config" / "nav2_params.yaml").read_text(encoding="utf-8")
    )
    for node in (
        "amcl",
        "bt_navigator",
        "controller_server",
        "planner_server",
        "behavior_server",
        "map_server",
        "velocity_smoother",
    ):
        assert node in nav
    amcl = nav["amcl"]["ros__parameters"]
    assert amcl["base_frame_id"] == "base_footprint"
    assert amcl["initial_pose.x"] == pytest.approx(indoor_layout.SPAWN_X)
    assert amcl["initial_pose.y"] == pytest.approx(indoor_layout.SPAWN_Y)
    assert amcl["initial_pose.yaw"] == pytest.approx(indoor_layout.SPAWN_YAW)
    assert amcl["set_initial_pose"] is True
    follow = nav["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["max_vel_x"] == pytest.approx(0.22)
    costmap = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["robot_radius"] == pytest.approx(0.12)
    assert costmap["inflation_layer"]["inflation_radius"] == pytest.approx(0.30)


def test_floor_plan_poses_are_free_and_walls_are_occupied():
    for name, x, y, _yaw in (
        indoor_layout.OFFICE,
        indoor_layout.KITCHEN,
        indoor_layout.LOBBY,
        indoor_layout.DOCK,
    ):
        assert indoor_layout.classify(x, y) == "free", name
    assert indoor_layout.classify(indoor_layout.SPAWN_X, indoor_layout.SPAWN_Y) == "free"
    south = next(box for box in indoor_layout.collision_boxes() if box.name == "wall_south")
    assert indoor_layout.classify(south.x, south.y) == "occupied"
    for door_x, door_y in ((-1.70, -0.70), (1.70, -0.70), (-1.70, 0.50), (1.70, 0.50)):
        assert indoor_layout.classify(door_x, door_y) == "free"
    width, height = indoor_layout.map_size()
    rows = indoor_layout.raster()
    for name, x, y, _yaw in (
        indoor_layout.OFFICE,
        indoor_layout.KITCHEN,
        indoor_layout.LOBBY,
        indoor_layout.DOCK,
    ):
        column = int((x - indoor_layout.ORIGIN_X) / indoor_layout.RESOLUTION)
        row_from_bottom = int((y - indoor_layout.ORIGIN_Y) / indoor_layout.RESOLUTION)
        shade = rows[height - 1 - row_from_bottom][column]
        assert shade == 254, name
    assert width == 160 and height == 120


def test_generated_assets_match_the_files_in_the_tree():
    assert (SRC / "indoor_robot_gazebo" / "worlds" / "indoor_office.world").read_text(
        encoding="utf-8"
    ) == render_world()
    assert (SRC / "indoor_robot_nav" / "maps" / "indoor_office.pgm").read_text(
        encoding="utf-8"
    ) == render_pgm()
    assert (SRC / "indoor_robot_nav" / "maps" / "indoor_office.yaml").read_text(
        encoding="utf-8"
    ) == render_map_yaml()
    assert (SRC / "indoor_robot_mission" / "config" / "waypoints.yaml").read_text(
        encoding="utf-8"
    ) == render_waypoints()


def _run_scripts(workflow: str) -> str:
    scripts: list[str] = []
    lines = workflow.splitlines()
    index = 0
    while index < len(lines):
        match = re.match(r"^(\s*)run:\s*(.*)$", lines[index])
        if match is None:
            index += 1
            continue
        indent = len(match.group(1))
        remainder = match.group(2).strip()
        index += 1
        if remainder in {"|", ">"}:
            block: list[str] = []
            while index < len(lines):
                line = lines[index]
                if line.strip() and (len(line) - len(line.lstrip())) <= indent:
                    break
                block.append(line)
                index += 1
            scripts.append("\n".join(block))
        else:
            scripts.append(remainder)
    return "\n".join(scripts)


def test_ci_runs_pytest_and_does_not_launch_a_simulator():
    workflow_path = ROOT / ".github" / "workflows" / "ci.yml"
    text = workflow_path.read_text(encoding="utf-8")
    loaded = yaml.safe_load(text)
    jobs = loaded["jobs"]
    assert "pytest" in jobs
    assert "colcon-build" in jobs
    assert "ros:humble-ros-base" in text
    assert "pytest" in text
    scripts = _run_scripts(text).lower()
    assert "gazebo" not in scripts
    assert "gz sim" not in scripts
    assert "ros2 launch" not in scripts
