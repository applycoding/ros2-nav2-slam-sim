"""Retry policy and the runner's use of it."""

import pytest

from indoor_robot_mission.mission_fsm import State
from indoor_robot_mission.mission_runner import GoalResult, MissionRunner
from indoor_robot_mission.retry_policy import (
    RetryPolicy,
    backoff_for_failure,
    should_retry,
)
from indoor_robot_mission.waypoints import parse_waypoints


def _plan() -> object:
    return parse_waypoints(
        """
        start: {x: 0, y: 0, yaw: 0}
        waypoints:
          - {name: office, x: 1, y: 0, yaw: 0}
          - {name: kitchen, x: 2, y: 0, yaw: 0}
        """
    )


class ScriptedNavigator:
    def __init__(self, script: dict[str, list[GoalResult]]) -> None:
        self.script = {name: list(results) for name, results in script.items()}
        self.calls: list[str] = []

    def navigate(self, waypoint):
        self.calls.append(waypoint.name)
        queue = self.script[waypoint.name]
        if not queue:
            raise AssertionError(f"no scripted result left for {waypoint.name}")
        return queue.pop(0)


def test_aborted_and_timeout_retry_until_the_budget_is_spent():
    policy = RetryPolicy(max_attempts=3, backoff_seconds=(0.5, 1.5, 4.0))
    assert should_retry(1, policy, "aborted")
    assert should_retry(2, policy, "timeout")
    assert not should_retry(3, policy, "aborted")


def test_canceled_and_rejected_are_not_retried():
    policy = RetryPolicy(max_attempts=5, backoff_seconds=(1.0,))
    assert not should_retry(1, policy, "canceled")
    assert not should_retry(1, policy, "rejected")


def test_unknown_failure_kind():
    policy = RetryPolicy()
    with pytest.raises(ValueError, match="unknown failure"):
        should_retry(1, policy, "exploded")


def test_backoff_uses_the_last_delay_after_the_list_ends():
    policy = RetryPolicy(max_attempts=5, backoff_seconds=(0.5, 1.5))
    assert backoff_for_failure(1, policy) == pytest.approx(0.5)
    assert backoff_for_failure(2, policy) == pytest.approx(1.5)
    assert backoff_for_failure(4, policy) == pytest.approx(1.5)


def test_policy_rejects_empty_or_negative_settings():
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0, backoff_seconds=(1.0,))
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=1, backoff_seconds=())
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=1, backoff_seconds=(-1.0,))


def test_runner_retries_then_continues():
    navigator = ScriptedNavigator(
        {
            "office": [GoalResult.ABORTED, GoalResult.TIMEOUT, GoalResult.SUCCEEDED],
            "kitchen": [GoalResult.SUCCEEDED],
        }
    )
    sleeps: list[float] = []
    runner = MissionRunner(
        _plan(),
        navigator,
        retry_policy=RetryPolicy(max_attempts=3, backoff_seconds=(0.5, 1.5, 4.0)),
    )
    report = runner.run(sleeper=sleeps.append)
    assert report.state == State.SUCCEEDED
    assert navigator.calls == ["office", "office", "office", "kitchen"]
    assert sleeps == [0.5, 1.5]
    assert report.waypoints_reached == ["office", "kitchen"]


def test_runner_stops_when_retries_are_exhausted():
    navigator = ScriptedNavigator(
        {
            "office": [GoalResult.ABORTED, GoalResult.ABORTED, GoalResult.ABORTED],
            "kitchen": [GoalResult.SUCCEEDED],
        }
    )
    runner = MissionRunner(
        _plan(),
        navigator,
        retry_policy=RetryPolicy(max_attempts=3, backoff_seconds=(0.25, 0.25)),
    )
    report = runner.run()
    assert report.state == State.FAILED
    assert report.failure_reason == "aborted"
    assert navigator.calls == ["office", "office", "office"]


def test_runner_does_not_retry_a_rejected_goal():
    navigator = ScriptedNavigator(
        {"office": [GoalResult.REJECTED], "kitchen": [GoalResult.SUCCEEDED]}
    )
    sleeps: list[float] = []
    runner = MissionRunner(_plan(), navigator)
    report = runner.run(sleeper=sleeps.append)
    assert report.state == State.FAILED
    assert report.failure_reason == "rejected"
    assert navigator.calls == ["office"]
    assert sleeps == []
