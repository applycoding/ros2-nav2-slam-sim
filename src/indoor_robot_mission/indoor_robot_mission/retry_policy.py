"""Retry decisions for a single NavigateToPose goal."""

from __future__ import annotations

from dataclasses import dataclass


RETRYABLE_FAILURES = frozenset({"aborted", "timeout"})
NON_RETRYABLE_FAILURES = frozenset({"canceled", "rejected"})
KNOWN_FAILURES = RETRYABLE_FAILURES | NON_RETRYABLE_FAILURES


@dataclass(frozen=True)
class RetryPolicy:
    """max_attempts counts the original try plus retries.

    backoff_seconds[i] is the wait after failure number i + 1. A policy with
    max_attempts 3 and backoff (1, 2) waits 1s after the first failure and 2s
    after the second, then stops.
    """

    max_attempts: int = 3
    backoff_seconds: tuple[float, ...] = (1.0, 2.0, 4.0)

    def __post_init__(self) -> None:
        if self.max_attempts < 1:
            raise ValueError("max_attempts must be at least 1")
        if not self.backoff_seconds:
            raise ValueError("backoff_seconds must not be empty")
        if any(delay < 0.0 for delay in self.backoff_seconds):
            raise ValueError("backoff delays must be >= 0")


def should_retry(failed_attempts: int, policy: RetryPolicy, failure_kind: str) -> bool:
    """Return true when another attempt is allowed after this failure.

    failed_attempts is how many tries have already failed, including the one
    described by failure_kind.
    """
    if failed_attempts < 1:
        raise ValueError("failed_attempts must be at least 1")
    if failure_kind not in KNOWN_FAILURES:
        raise ValueError(f"unknown failure kind: {failure_kind}")
    if failure_kind in NON_RETRYABLE_FAILURES:
        return False
    return failed_attempts < policy.max_attempts


def backoff_for_failure(failed_attempts: int, policy: RetryPolicy) -> float:
    """Seconds to wait before the next try. failed_attempts is 1-based."""
    if failed_attempts < 1:
        raise ValueError("failed_attempts must be at least 1")
    index = failed_attempts - 1
    if index >= len(policy.backoff_seconds):
        return policy.backoff_seconds[-1]
    return policy.backoff_seconds[index]
