"""Deterministic replay input boundary without a presentation renderer."""

from __future__ import annotations

from dataclasses import dataclass

from loaded_cmj.runtime.results import RolloutResult


@dataclass(frozen=True, slots=True)
class ReplayInput:
    """The exact immutable trace a future renderer is allowed to consume."""

    attempt_id: str
    trace_identity: str
    samples: tuple[object, ...]


def replay_input(result: RolloutResult) -> ReplayInput:
    """Bind replay to one result and its sealed live trace."""
    if not isinstance(result, RolloutResult):
        raise TypeError("replay_input requires a RolloutResult")
    if not result.trace_identity:
        raise ValueError("rollout result has no trace identity")
    return ReplayInput(result.attempt_id, result.trace_identity, result.trace)


__all__ = ["ReplayInput", "replay_input"]
