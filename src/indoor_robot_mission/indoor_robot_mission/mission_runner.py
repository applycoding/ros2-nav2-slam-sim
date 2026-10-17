"""Run a waypoint mission against a navigate() backend.

The runner is synchronous and ROS-free. A backend returns a GoalResult for
each waypoint. Tests pass a scripted backend. The Nav2 node adapts an action
client to the same method.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Protocol

from indoor_robot_mission.battery import Battery, BatteryConfig
from indoor_robot_mission.geometry import Pose2D, distance
from indoor_robot_mission.mission_fsm import (
    PAUSABLE,
    TERMINAL,
    Cancel,
    ChargeComplete,
    DockArrived,
    Fail,
    GoalDispatched,
    GoalFailed,
    GoalSucceeded,
    LowBattery,
    MissionStateMachine,
    MissionStatus,
    Pause,
    Resume,
    RetryElapsed,
    Start,
    State,
    WaypointsExhausted,
)
from indoor_robot_mission.retry_policy import RetryPolicy, backoff_for_failure
from indoor_robot_mission.waypoints import MissionPlan, Waypoint


class GoalResult(Enum):
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"
    TIMEOUT = "timeout"


class Navigator(Protocol):
    def navigate(self, waypoint: Waypoint) -> GoalResult:
        """Block until the goal finishes and return the outcome."""


Sleeper = Callable[[float], None]


_COMMANDS = frozenset({"pause", "resume", "cancel"})


class CommandQueue:
    """Thread-safe list of pause, resume, and cancel commands."""

    def __init__(self) -> None:
        self._items: list[str] = []
        self._lock = threading.Lock()

    def push(self, command: str) -> None:
        if command not in _COMMANDS:
            raise ValueError(f"unknown command: {command}")
        with self._lock:
            self._items.append(command)

    def drain(self) -> list[str]:
        with self._lock:
            items = self._items
            self._items = []
            return items


@dataclass
class MissionReport:
    state: State
    waypoints_reached: list[str]
    calls: list[str]
    failure_reason: str | None
    final_battery: float
    history: list[MissionStatus] = field(default_factory=list)


def _backoff_seconds(parameters: dict[str, object]) -> tuple[float, ...]:
    raw = parameters.get("backoff_seconds", (1.0, 2.0, 4.0))
    if isinstance(raw, (str, bytes)) or not isinstance(raw, (list, tuple)):
        raise ValueError("backoff_seconds must be a sequence of numbers")
    return tuple(float(value) for value in raw)


def runner_from_mapping(
    plan: MissionPlan,
    navigator: Navigator,
    parameters: dict[str, object],
    commands: CommandQueue | None = None,
    on_status: Callable[[MissionStatus], None] | None = None,
) -> MissionRunner:
    """Build a runner from the flat parameter mapping used by the ROS node."""
    policy = RetryPolicy(
        max_attempts=int(parameters.get("max_attempts", 3)),
        backoff_seconds=_backoff_seconds(parameters),
    )
    battery = Battery(
        BatteryConfig(
            capacity=float(parameters.get("battery_capacity", 100.0)),
            initial_charge=float(parameters.get("battery_initial", 100.0)),
            low_threshold=float(parameters.get("battery_low_threshold", 25.0)),
            critical_threshold=float(parameters.get("battery_critical_threshold", 8.0)),
            drain_per_meter=float(parameters.get("battery_drain_per_meter", 4.0)),
            attempt_cost=float(parameters.get("battery_attempt_cost", 1.0)),
            charge_per_second=float(parameters.get("battery_charge_per_second", 5.0)),
        )
    )
    return MissionRunner(
        plan=plan,
        navigator=navigator,
        retry_policy=policy,
        battery=battery,
        commands=commands,
        dock_resume_percent=float(parameters.get("dock_resume_percent", 80.0)),
        on_status=on_status,
    )


class MissionRunner:
    """Walk the plan, retry failed goals, and divert to the dock when low."""

    def __init__(
        self,
        plan: MissionPlan,
        navigator: Navigator,
        retry_policy: RetryPolicy | None = None,
        battery: Battery | None = None,
        commands: CommandQueue | None = None,
        dock_resume_percent: float = 80.0,
        on_status: Callable[[MissionStatus], None] | None = None,
    ) -> None:
        self.plan = plan
        self.navigator = navigator
        self.retry_policy = retry_policy if retry_policy is not None else RetryPolicy()
        self.battery = battery if battery is not None else Battery()
        self.commands = commands if commands is not None else CommandQueue()
        self.dock_resume_percent = dock_resume_percent
        self._on_status = on_status
        self._validate_resume_level()
        self.sm = MissionStateMachine(len(plan.waypoints), self.retry_policy)
        self.pose = plan.start
        self.calls: list[str] = []
        self.history: list[MissionStatus] = []

    def run(self, sleeper: Sleeper | None = None) -> MissionReport:
        sleep = sleeper if sleeper is not None else (lambda _seconds: None)
        self._record()
        self._emit(Start())
        self._record()
        while self.sm.state not in TERMINAL:
            if not self._apply_commands():
                break
            if self.sm.state == State.PAUSED:
                self._record()
                sleep(0.05)
                continue
            if self.sm.state == State.RUNNING:
                self._tick_running()
            elif self.sm.state == State.RETRY_WAIT:
                delay = backoff_for_failure(self.sm.attempt, self.retry_policy)
                sleep(delay)
                if self.sm.state == State.RETRY_WAIT:
                    self._emit(RetryElapsed())
            elif self.sm.state == State.DOCKING:
                if self.plan.dock is None:
                    self._emit(Fail("no_dock"))
                else:
                    self._travel(self.plan.dock, purpose="dock")
            elif self.sm.state == State.CHARGING:
                self._charge(sleep)
            else:
                raise RuntimeError(f"runner cannot step state {self.sm.state.value}")
            self._record()
        self._record()
        return self.report()

    def report(self) -> MissionReport:
        return MissionReport(
            state=self.sm.state,
            waypoints_reached=list(self.sm.reached),
            calls=list(self.calls),
            failure_reason=self.sm.failure_reason,
            final_battery=self.battery.charge,
            history=list(self.history),
        )

    def _tick_running(self) -> None:
        if self.sm.waypoint_index >= len(self.plan.waypoints):
            self._emit(WaypointsExhausted())
            return
        waypoint = self.plan.waypoints[self.sm.waypoint_index]
        leg = distance(self.pose, waypoint.pose())
        projected = self.battery.project(leg)
        if projected < self.battery.config.low_threshold:
            if self.plan.dock is None:
                self._emit(Fail("battery_low"))
                return
            resume_charge = self.battery.config.capacity * self.dock_resume_percent / 100.0
            if self.battery.charge + 1e-9 >= resume_charge:
                self._emit(Fail("insufficient_range"))
                return
            self._emit(LowBattery())
            return
        self._travel(waypoint, purpose="mission")

    def _travel(self, waypoint: Waypoint, purpose: str) -> None:
        self._emit(GoalDispatched(waypoint.name))
        result = self.navigator.navigate(waypoint)
        self.calls.append(waypoint.name)
        if result == GoalResult.SUCCEEDED:
            self.battery.consume_distance(distance(self.pose, waypoint.pose()))
            self.pose = Pose2D(waypoint.x, waypoint.y, waypoint.yaw)
            if purpose == "dock":
                self._emit(DockArrived())
            else:
                self._emit(GoalSucceeded())
            return
        self.battery.consume_attempt()
        self._emit(GoalFailed(result.value))

    def _charge(self, sleep: Sleeper) -> None:
        target = self.battery.config.capacity * self.dock_resume_percent / 100.0
        steps = 0
        while self.sm.state == State.CHARGING and self.battery.charge + 1e-9 < target:
            if not self._apply_commands():
                return
            if self.sm.state != State.CHARGING:
                return
            self.battery.charge_for(1.0)
            self._record()
            sleep(1.0)
            steps += 1
            if steps > 100000:
                raise RuntimeError("charging did not reach the resume level")
        if self.sm.state == State.CHARGING:
            self._emit(ChargeComplete())

    def _apply_commands(self) -> bool:
        for command in self.commands.drain():
            if self.sm.state in TERMINAL:
                break
            if command == "cancel":
                self._emit(Cancel())
                return False
            if command == "pause" and self.sm.state in PAUSABLE:
                self._emit(Pause())
            elif command == "resume" and self.sm.state == State.PAUSED:
                self._emit(Resume())
        return self.sm.state not in TERMINAL

    def _emit(self, event: object) -> State:
        state = self.sm.transition(event)
        self._record()
        return state

    def _record(self) -> None:
        snap = self.sm.snapshot(self.battery.percent)
        if self.history and self.history[-1] == snap:
            return
        self.history.append(snap)
        if self._on_status is not None:
            self._on_status(snap)

    def _validate_resume_level(self) -> None:
        if not 0.0 < self.dock_resume_percent <= 100.0:
            raise ValueError("dock_resume_percent must be in (0, 100]")
        resume_charge = self.battery.config.capacity * self.dock_resume_percent / 100.0
        if resume_charge <= self.battery.config.low_threshold:
            raise ValueError("dock resume level must be above the low battery threshold")
