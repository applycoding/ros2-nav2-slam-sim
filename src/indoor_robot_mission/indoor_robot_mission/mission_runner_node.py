"""ROS 2 node that runs a waypoint mission through Nav2 NavigateToPose.

The mission decisions live in mission_runner. This module only adapts the
action client, parameters, and status topics.
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from indoor_robot_mission.geometry import yaw_to_quaternion
from indoor_robot_mission.mission_fsm import MissionStatus, State
from indoor_robot_mission.mission_runner import (
    CommandQueue,
    GoalResult,
    runner_from_mapping,
)
from indoor_robot_mission.waypoints import Waypoint, load_waypoints


_RESULT_BY_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: GoalResult.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: GoalResult.CANCELED,
    GoalStatus.STATUS_ABORTED: GoalResult.ABORTED,
}

_ACCEPTED_COMMANDS = frozenset({"pause", "resume", "cancel"})


class Nav2Navigator:
    """Blocking NavigateToPose client used by MissionRunner."""

    def __init__(
        self,
        node: Node,
        action_name: str,
        server_timeout_sec: float,
        goal_timeout_sec: float,
    ) -> None:
        self._node = node
        self._server_timeout_sec = server_timeout_sec
        self._goal_timeout_sec = goal_timeout_sec
        self._client = ActionClient(node, NavigateToPose, action_name)

    def navigate(self, waypoint: Waypoint) -> GoalResult:
        if not self._client.wait_for_server(timeout_sec=self._server_timeout_sec):
            self._node.get_logger().error("NavigateToPose action server is not available")
            return GoalResult.REJECTED

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = waypoint.frame_id
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint.x)
        goal.pose.pose.position.y = float(waypoint.y)
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(waypoint.yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self._client.send_goal_async(goal)
        try:
            goal_handle = send_future.result(timeout=self._server_timeout_sec)
        except Exception as exc:
            self._node.get_logger().error(f"failed to send goal {waypoint.name}: {exc}")
            return GoalResult.REJECTED
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(f"goal rejected for {waypoint.name}")
            return GoalResult.REJECTED

        result_future = goal_handle.get_result_async()
        try:
            wrapped = result_future.result(timeout=self._goal_timeout_sec)
        except Exception as exc:
            self._node.get_logger().error(f"goal timed out for {waypoint.name}: {exc}")
            goal_handle.cancel_goal_async()
            return GoalResult.TIMEOUT
        return _RESULT_BY_STATUS.get(wrapped.status, GoalResult.ABORTED)


class MissionRunnerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_runner")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("server_timeout_sec", 10.0)
        self.declare_parameter("max_attempts", 3)
        self.declare_parameter("backoff_seconds", [2.0, 4.0, 8.0])
        self.declare_parameter("battery_capacity", 100.0)
        self.declare_parameter("battery_initial", 100.0)
        self.declare_parameter("battery_low_threshold", 25.0)
        self.declare_parameter("battery_critical_threshold", 8.0)
        self.declare_parameter("battery_drain_per_meter", 4.0)
        self.declare_parameter("battery_attempt_cost", 1.0)
        self.declare_parameter("battery_charge_per_second", 5.0)
        self.declare_parameter("dock_resume_percent", 80.0)
        self.declare_parameter("status_topic", "mission_status")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("command_topic", "mission_command")

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            status_qos,
        )
        self._battery_pub = self.create_publisher(
            BatteryState,
            str(self.get_parameter("battery_topic").value),
            10,
        )
        self._commands = CommandQueue()
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
        )

    def _on_command(self, message: String) -> None:
        command = message.data.strip().lower()
        if command not in _ACCEPTED_COMMANDS:
            self.get_logger().warning(f"ignoring mission command {command!r}")
            return
        self._commands.push(command)

    def execute(self):
        waypoints_file = str(self.get_parameter("waypoints_file").value)
        if not waypoints_file:
            raise RuntimeError("waypoints_file parameter is empty")
        plan = load_waypoints(waypoints_file)
        navigator = Nav2Navigator(
            self,
            action_name=str(self.get_parameter("action_name").value),
            server_timeout_sec=float(self.get_parameter("server_timeout_sec").value),
            goal_timeout_sec=float(self.get_parameter("goal_timeout_sec").value),
        )
        names = (
            "max_attempts",
            "backoff_seconds",
            "battery_capacity",
            "battery_initial",
            "battery_low_threshold",
            "battery_critical_threshold",
            "battery_drain_per_meter",
            "battery_attempt_cost",
            "battery_charge_per_second",
            "dock_resume_percent",
        )
        parameters = {name: self.get_parameter(name).value for name in names}
        runner = runner_from_mapping(
            plan,
            navigator,
            parameters,
            commands=self._commands,
            on_status=self.publish_status,
        )
        self.get_logger().info(
            f"starting mission with {len(plan.waypoints)} waypoints from {waypoints_file}"
        )
        report = runner.run(sleeper=self._sleep)
        self.get_logger().info(
            f"mission {report.state.value} failure_reason={report.failure_reason}"
        )
        return report

    def publish_status(self, status: MissionStatus) -> None:
        message = String()
        message.data = json.dumps(status.to_dict(), sort_keys=True)
        self._status_pub.publish(message)

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = "base_link"
        battery.present = True
        battery.percentage = max(0.0, min(1.0, status.battery_percent / 100.0))
        # The stub does not model electrical units. Leave them unset as NaN
        # so a plot of voltage is not mistaken for a measurement.
        battery.voltage = float("nan")
        battery.current = float("nan")
        battery.charge = float("nan")
        battery.capacity = float("nan")
        battery.design_capacity = float("nan")
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        if status.state == State.CHARGING.value:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._battery_pub.publish(battery)

    def _sleep(self, seconds: float) -> None:
        """Wait on the node clock so use_sim_time applies to retries and charging."""
        if seconds <= 0.0:
            return
        deadline = self.get_clock().now() + Duration(seconds=float(seconds))
        while rclpy.ok() and self.get_clock().now() < deadline:
            time.sleep(0.05)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionRunnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    exit_code = 0
    try:
        report = node.execute()
        if report.state != State.SUCCEEDED:
            exit_code = 2
    except KeyboardInterrupt:
        exit_code = 2
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 2
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
