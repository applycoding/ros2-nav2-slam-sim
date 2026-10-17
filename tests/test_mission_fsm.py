"""Mission state machine, battery stub, and runner scenarios."""

import sys

import pytest

from indoor_robot_mission.battery import Battery, BatteryConfig
from indoor_robot_mission.geometry import yaw_to_quaternion
from indoor_robot_mission.mission_fsm import (
    ChargeComplete,
    DockArrived,
    GoalDispatched,
    GoalFailed,
    GoalSucceeded,
    InvalidTransition,
    LowBattery,
    MissionStateMachine,
    Pause,
    Resume,
    RetryElapsed,
    Start,
    State,
)
from indoor_robot_mission.mission_runner import (
    GoalResult,
    MissionRunner,
    runner_from_mapping,
)
from indoor_robot_mission.retry_policy import RetryPolicy
from indoor_robot_mission.waypoints import parse_waypoints


def _machine(count: int = 2, attempts: int = 3) -> MissionStateMachine:
    return MissionStateMachine(count, RetryPolicy(max_attempts=attempts, backoff_seconds=(1.0,)))


def _states(report) -> list[str]:
    ordered: list[str] = []
    for snap in report.history:
        if not ordered or ordered[-1] != snap.state:
            ordered.append(snap.state)
    return ordered


class AlwaysSucceed:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def navigate(self, waypoint):
        self.calls.append(waypoint.name)
        return GoalResult.SUCCEEDED


def test_happy_path_events_visit_each_waypoint_once():
    machine = _machine()
    assert machine.transition(Start()) == State.RUNNING
    machine.transition(GoalDispatched("office"))
    assert machine.state == State.NAVIGATING
    assert machine.attempt == 1
    machine.transition(GoalSucceeded())
    assert machine.state == State.RUNNING
    assert machine.waypoint_index == 1
    machine.transition(GoalDispatched("kitchen"))
    machine.transition(GoalSucceeded())
    assert machine.state == State.SUCCEEDED
    assert machine.reached == ["office", "kitchen"]


def test_failure_schedules_one_retry_then_can_succeed():
    machine = _machine(count=1, attempts=2)
    machine.transition(Start())
    machine.transition(GoalDispatched("office"))
    assert machine.transition(GoalFailed("aborted")) == State.RETRY_WAIT
    assert machine.transition(RetryElapsed()) == State.RUNNING
    machine.transition(GoalDispatched("office"))
    assert machine.attempt == 2
    assert machine.transition(GoalSucceeded()) == State.SUCCEEDED


def test_timeout_exhausts_the_attempt_budget():
    machine = _machine(count=1, attempts=1)
    machine.transition(Start())
    machine.transition(GoalDispatched("office"))
    assert machine.transition(GoalFailed("timeout")) == State.FAILED
    assert machine.failure_reason == "timeout"


def test_canceled_goal_cancels_the_mission():
    machine = _machine(count=1)
    machine.transition(Start())
    machine.transition(GoalDispatched("office"))
    assert machine.transition(GoalFailed("canceled")) == State.CANCELED


def test_empty_mission_fails_on_start():
    machine = _machine(count=0)
    assert machine.transition(Start()) == State.FAILED
    assert machine.failure_reason == "empty_mission"


def test_pause_and_resume_restore_the_previous_state():
    machine = _machine()
    machine.transition(Start())
    machine.transition(Pause())
    assert machine.state == State.PAUSED
    machine.transition(Resume())
    assert machine.state == State.RUNNING


def test_illegal_transitions_raise():
    machine = _machine()
    with pytest.raises(InvalidTransition):
        machine.transition(Pause())
    machine.transition(Start())
    with pytest.raises(InvalidTransition):
        machine.transition(Resume())
    machine.transition(GoalDispatched("office"))
    machine.transition(GoalSucceeded())
    machine.transition(GoalDispatched("kitchen"))
    machine.transition(GoalSucceeded())
    with pytest.raises(InvalidTransition):
        machine.transition(Start())


def test_unknown_event_type_is_rejected():
    machine = _machine()
    with pytest.raises(TypeError, match="unhandled event"):
        machine.transition(object())


def test_low_battery_docks_charges_and_keeps_the_waypoint_index():
    machine = _machine(count=2)
    machine.transition(Start())
    machine.transition(GoalDispatched("office"))
    machine.transition(GoalSucceeded())
    assert machine.waypoint_index == 1
    machine.transition(LowBattery())
    assert machine.state == State.DOCKING
    assert machine.attempt == 0
    machine.transition(GoalDispatched("dock"))
    machine.transition(DockArrived())
    assert machine.state == State.CHARGING
    machine.transition(ChargeComplete())
    assert machine.state == State.RUNNING
    assert machine.waypoint_index == 1


def test_runner_completes_two_waypoints_and_publishes_navigating():
    navigator = AlwaysSucceed()
    seen: list[str] = []
    plan = parse_waypoints(
        """
        start: {x: 0, y: 0}
        waypoints:
          - {name: office, x: 1, y: 0}
          - {name: kitchen, x: 2, y: 0}
        """
    )
    runner = MissionRunner(plan, navigator, on_status=lambda status: seen.append(status.state))
    report = runner.run()
    assert report.state == State.SUCCEEDED
    assert navigator.calls == ["office", "kitchen"]
    assert "navigating" in seen
    assert seen[-1] == "succeeded"
    payload = report.history[-1].to_dict()
    assert payload["waypoint_count"] == 2
    assert payload["failure_reason"] is None
    assert "battery_percent" in payload


def test_runner_diverts_to_the_dock_before_a_leg_it_cannot_afford():
    plan = parse_waypoints(
        """
        start: {x: 0, y: 0, yaw: 0}
        dock: {name: dock, x: 0, y: 1, yaw: 0}
        waypoints:
          - {name: office, x: 2, y: 0, yaw: 0}
        """
    )
    navigator = AlwaysSucceed()
    battery = Battery(
        BatteryConfig(
            capacity=100.0,
            initial_charge=30.0,
            low_threshold=25.0,
            critical_threshold=5.0,
            drain_per_meter=10.0,
            attempt_cost=1.0,
            charge_per_second=25.0,
        )
    )
    runner = MissionRunner(
        plan,
        navigator,
        battery=battery,
        dock_resume_percent=80.0,
    )
    report = runner.run()
    assert report.state == State.SUCCEEDED
    assert navigator.calls[0] == "dock"
    assert navigator.calls[-1] == "office"
    assert "charging" in _states(report)
    assert report.final_battery > 25.0


def test_full_battery_still_fails_a_leg_longer_than_the_resume_level():
    plan = parse_waypoints(
        """
        start: {x: 0, y: 0}
        dock: {name: dock, x: 0, y: 1}
        waypoints:
          - {name: office, x: 3, y: 0}
        """
    )
    navigator = AlwaysSucceed()
    battery = Battery(
        BatteryConfig(
            initial_charge=100.0,
            low_threshold=25.0,
            drain_per_meter=50.0,
            charge_per_second=5.0,
        )
    )
    runner = MissionRunner(plan, navigator, battery=battery, dock_resume_percent=80.0)
    report = runner.run()
    assert report.state == State.FAILED
    assert report.failure_reason == "insufficient_range"
    assert navigator.calls == []


def test_low_battery_without_a_dock_fails():
    plan = parse_waypoints(
        """
        start: {x: 0, y: 0}
        waypoints:
          - {name: office, x: 3, y: 0}
        """
    )
    battery = Battery(
        BatteryConfig(initial_charge=20.0, low_threshold=25.0, critical_threshold=5.0)
    )
    runner = MissionRunner(plan, AlwaysSucceed(), battery=battery)
    report = runner.run()
    assert report.state == State.FAILED
    assert report.failure_reason == "battery_low"


def test_pause_command_holds_the_mission_until_resume():
    plan = parse_waypoints(
        """
        waypoints:
          - {name: office, x: 1, y: 0}
        """
    )
    runner = MissionRunner(plan, AlwaysSucceed())
    runner.commands.push("pause")

    def sleeper(_seconds: float) -> None:
        if runner.sm.state == State.PAUSED:
            runner.commands.push("resume")

    report = runner.run(sleeper=sleeper)
    assert "paused" in _states(report)
    assert report.state == State.SUCCEEDED
    assert report.calls == ["office"]


def test_cancel_command_stops_before_navigation():
    plan = parse_waypoints(
        """
        waypoints:
          - {name: office, x: 1, y: 0}
        """
    )
    navigator = AlwaysSucceed()
    runner = MissionRunner(plan, navigator)
    runner.commands.push("cancel")
    report = runner.run()
    assert report.state == State.CANCELED
    assert report.failure_reason == "canceled"
    assert navigator.calls == []


def test_runner_from_mapping_uses_parameter_values():
    plan = parse_waypoints("waypoints:\n  - {name: office, x: 1, y: 0}\n")
    runner = runner_from_mapping(
        plan,
        AlwaysSucceed(),
        {
            "max_attempts": 2,
            "backoff_seconds": [0.2, 0.4],
            "battery_initial": 90.0,
            "battery_low_threshold": 20.0,
            "battery_critical_threshold": 5.0,
            "dock_resume_percent": 70.0,
        },
    )
    assert runner.retry_policy.max_attempts == 2
    assert runner.battery.charge == pytest.approx(90.0)
    assert runner.run().state == State.SUCCEEDED


def test_battery_clamps_and_projection_does_not_mutate():
    battery = Battery(BatteryConfig(initial_charge=10.0, drain_per_meter=4.0, charge_per_second=50.0))
    assert battery.project(100.0) == 0.0
    assert battery.charge == pytest.approx(10.0)
    battery.consume_distance(100.0)
    assert battery.charge == 0.0
    battery.charge_for(10.0)
    assert battery.charge == pytest.approx(100.0)
    with pytest.raises(ValueError):
        battery.consume_distance(-1.0)
    assert battery.needs_dock is False


def test_battery_config_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        BatteryConfig(low_threshold=5.0, critical_threshold=10.0)


def test_yaw_zero_and_half_turn():
    assert yaw_to_quaternion(0.0)[3] == pytest.approx(1.0)
    _x, _y, z, w = yaw_to_quaternion(3.141592653589793)
    assert z == pytest.approx(1.0, abs=1e-6)
    assert w == pytest.approx(0.0, abs=1e-6)


def test_logic_modules_do_not_import_rclpy():
    assert "rclpy" not in sys.modules
