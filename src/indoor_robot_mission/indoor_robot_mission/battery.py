"""Software battery and docking thresholds.

Charge is an internal counter, not a measurement from hardware. The mission
runner drains it by planned distance and refills it while the robot is
modeled as sitting on the dock.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BatteryConfig:
    capacity: float = 100.0
    initial_charge: float = 100.0
    low_threshold: float = 25.0
    critical_threshold: float = 8.0
    drain_per_meter: float = 4.0
    attempt_cost: float = 1.0
    charge_per_second: float = 5.0

    def __post_init__(self) -> None:
        if self.capacity <= 0.0:
            raise ValueError("capacity must be positive")
        if not 0.0 <= self.initial_charge <= self.capacity:
            raise ValueError("initial_charge must be inside [0, capacity]")
        if not 0.0 <= self.critical_threshold <= self.low_threshold <= self.capacity:
            raise ValueError("thresholds must satisfy 0 <= critical <= low <= capacity")
        if self.drain_per_meter < 0.0 or self.attempt_cost < 0.0:
            raise ValueError("drain costs must be >= 0")
        if self.charge_per_second <= 0.0:
            raise ValueError("charge_per_second must be positive")


class Battery:
    def __init__(self, config: BatteryConfig | None = None) -> None:
        self.config = config if config is not None else BatteryConfig()
        self.charge = self.config.initial_charge

    @property
    def percent(self) -> float:
        return 100.0 * self.charge / self.config.capacity

    @property
    def needs_dock(self) -> bool:
        return self.charge <= self.config.low_threshold

    def project(self, meters: float) -> float:
        if meters < 0.0:
            raise ValueError("meters must be >= 0")
        return max(0.0, self.charge - meters * self.config.drain_per_meter)

    def consume_distance(self, meters: float) -> None:
        if meters < 0.0:
            raise ValueError("meters must be >= 0")
        self.charge = max(0.0, self.charge - meters * self.config.drain_per_meter)

    def consume_attempt(self) -> None:
        self.charge = max(0.0, self.charge - self.config.attempt_cost)

    def charge_for(self, seconds: float) -> None:
        if seconds < 0.0:
            raise ValueError("seconds must be >= 0")
        gained = seconds * self.config.charge_per_second
        self.charge = min(self.config.capacity, self.charge + gained)
