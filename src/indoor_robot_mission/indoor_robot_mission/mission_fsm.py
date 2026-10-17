"""Mission state machine.

The machine records where the mission is. It does not talk to Nav2. The
runner applies navigation results, battery checks, and operator commands as
events.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from indoor_robot_mission.retry_policy import RetryPolicy, should_retry


class State(Enum):
    IDLE = "idle"
    RUNNING = "running"
    NAVIGATING = "navigating"
    RETRY_WAIT = "retry_wait"
    DOCKING = "docking"
    CHARGING = "charging"
    PAUSED = "paused"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


TERMINAL = frozenset({State.SUCCEEDED, State.FAILED, State.CANCELED})
PAUSABLE = frozenset(
    {State.RUNNING, State.NAVIGATING, State.RETRY_WAIT, State.DOCKING, State.CHARGING}
)


class InvalidTransition(Exception):
    def __init__(self, state: State, event: object) -> None:
        self.state = state
        self.event_name = type(event).__name__
        super().__init__(f"no transition from {state.value} on {self.event_name}")


@dataclass(frozen=True)
class Start:
    pass


@dataclass(frozen=True)
class GoalDispatched:
    name: str


@dataclass(frozen=True)
class GoalSucceeded:
    pass


@dataclass(frozen=True)
class GoalFailed:
    kind: str


@dataclass(frozen=True)
class RetryElapsed:
    pass


@dataclass(frozen=True)
class LowBattery:
    pass


@dataclass(frozen=True)
class DockArrived:
    pass


@dataclass(frozen=True)
class ChargeComplete:
    pass


@dataclass(frozen=True)
class Pause:
    pass


@dataclass(frozen=True)
class Resume:
    pass


@dataclass(frozen=True)
class Cancel:
    pass


@dataclass(frozen=True)
class Fail:
    reason: str


@dataclass(frozen=True)
class WaypointsExhausted:
    pass


@dataclass(frozen=True)
class MissionStatus:
    state: str
    waypoint: str | None
    waypoint_index: int
    waypoint_count: int
    attempt: int
    max_attempts: int
    battery_percent: float
    docking: bool
    message: str
    failure_reason: str | None

    def to_dict(self) -> dict[str, object]:
        return {
            "state": self.state,
            "waypoint": self.waypoint,
            "waypoint_index": self.waypoint_index,
            "waypoint_count": self.waypoint_count,
            "attempt": self.attempt,
            "max_attempts": self.max_attempts,
            "battery_percent": round(self.battery_percent, 4),
            "docking": self.docking,
            "message": self.message,
            "failure_reason": self.failure_reason,
        }


class MissionStateMachine:
    def __init__(self, waypoint_count: int, retry_policy: RetryPolicy) -> None:
        if waypoint_count < 0:
            raise ValueError("waypoint_count must be >= 0")
        self.waypoint_count = waypoint_count
        self.retry_policy = retry_policy
        self.state = State.IDLE
        self.waypoint_index = 0
        self.attempt = 0
        self.active_name: str | None = None
        self.failure_reason: str | None = None
        self.message = "idle"
        self.paused_from: State | None = None
        self.retry_returns_to: State | None = None
        self.reached: list[str] = []

    def transition(self, event: object) -> State:
        if isinstance(event, Start):
            return self._on_start()
        if isinstance(event, GoalDispatched):
            return self._on_goal_dispatched(event)
        if isinstance(event, GoalSucceeded):
            return self._on_goal_succeeded()
        if isinstance(event, GoalFailed):
            return self._on_goal_failed(event)
        if isinstance(event, RetryElapsed):
            return self._on_retry_elapsed()
        if isinstance(event, LowBattery):
            return self._on_low_battery()
        if isinstance(event, DockArrived):
            return self._on_dock_arrived()
        if isinstance(event, ChargeComplete):
            return self._on_charge_complete()
        if isinstance(event, Pause):
            return self._on_pause(event)
        if isinstance(event, Resume):
            return self._on_resume(event)
        if isinstance(event, Cancel):
            return self._on_cancel(event)
        if isinstance(event, Fail):
            return self._on_fail(event)
        if isinstance(event, WaypointsExhausted):
            return self._on_waypoints_exhausted(event)
        raise TypeError(f"unhandled event type: {type(event).__name__}")

    def snapshot(self, battery_percent: float) -> MissionStatus:
        docking = self.state in {State.DOCKING, State.CHARGING} or self.paused_from in {
            State.DOCKING,
            State.CHARGING,
        }
        return MissionStatus(
            state=self.state.value,
            waypoint=self.active_name,
            waypoint_index=self.waypoint_index,
            waypoint_count=self.waypoint_count,
            attempt=self.attempt,
            max_attempts=self.retry_policy.max_attempts,
            battery_percent=battery_percent,
            docking=docking,
            message=self.message,
            failure_reason=self.failure_reason,
        )

    def _on_start(self) -> State:
        self._require(State.IDLE, Start())
        if self.waypoint_count < 1:
            self.failure_reason = "empty_mission"
            self.message = "empty mission"
            self.state = State.FAILED
            return self.state
        self.message = "mission started"
        self.state = State.RUNNING
        return self.state

    def _on_goal_dispatched(self, event: GoalDispatched) -> State:
        if self.state == State.RUNNING:
            self.active_name = event.name
            self.attempt += 1
            self.message = f"navigating to {event.name} (attempt {self.attempt})"
            self.state = State.NAVIGATING
            return self.state
        if self.state == State.DOCKING:
            self.active_name = event.name
            self.attempt += 1
            self.message = f"docking at {event.name} (attempt {self.attempt})"
            return self.state
        raise InvalidTransition(self.state, event)

    def _on_goal_succeeded(self) -> State:
        self._require(State.NAVIGATING, GoalSucceeded())
        assert self.active_name is not None
        self.reached.append(self.active_name)
        self.waypoint_index += 1
        self.attempt = 0
        if self.waypoint_index >= self.waypoint_count:
            self.message = "mission complete"
            self.state = State.SUCCEEDED
            return self.state
        self.message = f"reached {self.active_name}"
        self.state = State.RUNNING
        return self.state

    def _on_goal_failed(self, event: GoalFailed) -> State:
        if self.state not in {State.NAVIGATING, State.DOCKING}:
            raise InvalidTransition(self.state, event)
        returns_to = State.RUNNING if self.state == State.NAVIGATING else State.DOCKING
        if event.kind == "canceled":
            self.failure_reason = "canceled"
            self.message = "goal canceled"
            self.state = State.CANCELED
            return self.state
        if should_retry(self.attempt, self.retry_policy, event.kind):
            self.retry_returns_to = returns_to
            self.message = f"retrying {self.active_name} after {event.kind}"
            self.state = State.RETRY_WAIT
            return self.state
        if returns_to == State.DOCKING:
            self.failure_reason = f"dock_{event.kind}"
        else:
            self.failure_reason = event.kind
        self.message = f"failed: {self.failure_reason}"
        self.state = State.FAILED
        return self.state

    def _on_retry_elapsed(self) -> State:
        self._require(State.RETRY_WAIT, RetryElapsed())
        if self.retry_returns_to is None:
            raise InvalidTransition(self.state, RetryElapsed())
        self.state = self.retry_returns_to
        self.message = f"retrying {self.active_name}"
        return self.state

    def _on_low_battery(self) -> State:
        self._require(State.RUNNING, LowBattery())
        self.attempt = 0
        self.message = "battery low, diverting to dock"
        self.state = State.DOCKING
        return self.state

    def _on_dock_arrived(self) -> State:
        self._require(State.DOCKING, DockArrived())
        self.attempt = 0
        self.message = "docked, charging"
        self.state = State.CHARGING
        return self.state

    def _on_charge_complete(self) -> State:
        self._require(State.CHARGING, ChargeComplete())
        self.message = "charge complete, resuming mission"
        self.state = State.RUNNING
        return self.state

    def _on_pause(self, event: Pause) -> State:
        if self.state not in PAUSABLE:
            raise InvalidTransition(self.state, event)
        self.paused_from = self.state
        self.message = "paused"
        self.state = State.PAUSED
        return self.state

    def _on_resume(self, event: Resume) -> State:
        if self.state != State.PAUSED or self.paused_from is None:
            raise InvalidTransition(self.state, event)
        self.state = self.paused_from
        self.paused_from = None
        self.message = "resumed"
        return self.state

    def _on_cancel(self, event: Cancel) -> State:
        if self.state in TERMINAL:
            raise InvalidTransition(self.state, event)
        self.failure_reason = "canceled"
        self.message = "canceled"
        self.state = State.CANCELED
        return self.state

    def _on_fail(self, event: Fail) -> State:
        if self.state in TERMINAL:
            raise InvalidTransition(self.state, event)
        self.failure_reason = event.reason
        self.message = event.reason
        self.state = State.FAILED
        return self.state

    def _on_waypoints_exhausted(self, event: WaypointsExhausted) -> State:
        if self.state != State.RUNNING or self.waypoint_index < self.waypoint_count:
            raise InvalidTransition(self.state, event)
        self.message = "mission complete"
        self.state = State.SUCCEEDED
        return self.state

    def _require(self, expected: State, event: object) -> None:
        if self.state != expected:
            raise InvalidTransition(self.state, event)
