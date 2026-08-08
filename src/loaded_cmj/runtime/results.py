"""Immutable rollout status, metrics, trace, and termination ownership."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any

import numpy as np

from loaded_cmj.runtime.errors import InvalidRuntimeContract


class EvaluationOutcome(str, Enum):
    OK = "ok"
    INVALID_POLICY = "invalid_policy"
    INTERNAL_ERROR = "internal_error"
    OBJECTIVE_SUCCESS = "objective_success"
    PHYSICAL_FAILURE = "physical_failure"
    INCOMPLETE = "incomplete"
    AGENT_FAULT = "agent_fault"


class TerminationReason(str, Enum):
    OBJECTIVE_COMPLETE = "OBJECTIVE_COMPLETE"
    PHYSICAL_FALL = "PHYSICAL_FALL"
    PHYSICS_NONFINITE_FAULT = "PHYSICS_NONFINITE_FAULT"
    INCOMPLETE_HORIZON = "INCOMPLETE_HORIZON"
    AGENT_FAULT = "AGENT_FAULT"
    INTERNAL_EVALUATION_ERROR = "INTERNAL_EVALUATION_ERROR"

@dataclass(frozen=True, slots=True)
class RolloutResult:
    """Frozen separation of attempt identity, outcome, events, metrics, and trace."""

    outcome: EvaluationOutcome
    termination_reason: TerminationReason
    completed_steps: int
    objective_completed: bool
    metrics: Mapping[str, float] = field(default_factory=dict)
    attempt_id: str = ""
    model_revision: str = ""
    runtime_revision: str = ""
    evaluation_outcome: str | None = None
    physical_metrics: Mapping[str, Any] = field(default_factory=dict)
    events: tuple[Any, ...] = ()
    fault_data: Mapping[str, Any] = field(default_factory=dict)
    trace: tuple[Any, ...] = ()
    trace_identity: str = ""
    sample_count: int = 0
    first_sample_time_s: float | None = None
    final_sample_time_s: float | None = None
    completed_control_steps: int | None = None
    completed_physics_steps: int | None = None
    simulated_duration_s: float | None = None
    rollout_valid: bool = True
    episode_terminated: bool = False
    complete_rollout: bool = False
    evidence_identity: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.completed_steps, bool) or self.completed_steps < 0:
            raise InvalidRuntimeContract("completed_steps must be a non-negative integer")
        for name in ("completed_control_steps", "completed_physics_steps", "sample_count"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or int(value) < 0):
                raise InvalidRuntimeContract(f"{name} must be a non-negative integer")
        if self.evaluation_outcome is None:
            object.__setattr__(self, "evaluation_outcome", self.outcome.value)
        else:
            object.__setattr__(self, "evaluation_outcome", str(self.evaluation_outcome))
        for name in (
            "attempt_id",
            "model_revision",
            "runtime_revision",
            "trace_identity",
            "evidence_identity",
        ):
            object.__setattr__(self, name, str(getattr(self, name)))
        for name in ("first_sample_time_s", "final_sample_time_s", "simulated_duration_s"):
            value = getattr(self, name)
            if value is not None:
                value = float(value)
                if not np.isfinite(value):
                    raise InvalidRuntimeContract(f"{name} must be finite when present")
                object.__setattr__(self, name, value)
        object.__setattr__(self, "metrics", _freeze_value(self.metrics))
        object.__setattr__(self, "physical_metrics", _freeze_value(self.physical_metrics))
        object.__setattr__(self, "events", _freeze_value(tuple(self.events)))
        object.__setattr__(self, "fault_data", _freeze_value(self.fault_data))
        frozen_trace = []
        for item in self.trace:
            seal = getattr(item, "seal", None)
            if callable(seal):
                item = seal()
            frozen_trace.append(item)
        object.__setattr__(self, "trace", _freeze_value(tuple(frozen_trace)))


def _freeze_value(value: Any) -> Any:
    """Detach mutable result payloads at the public result boundary."""
    if isinstance(value, np.ndarray):
        copied = np.array(value, copy=True)
        copied.setflags(write=False)
        return copied
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze_value(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze_value(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_freeze_value(item) for item in value)
    if isinstance(value, set):
        return frozenset(_freeze_value(item) for item in value)
    return value

__all__ = [
    "EvaluationOutcome",
    "RolloutResult",
    "TerminationReason",
]
