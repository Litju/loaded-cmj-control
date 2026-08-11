"""Immutable rollout status, metrics, trace, and termination ownership."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Sequence

import numpy as np

from loaded_cmj.runtime.errors import InvalidRuntimeContract
from loaded_cmj.simulation.constants import PHYSICS_TIMESTEP_S


TRACE_SCHEMA_VERSION = "LCMJ-V1-TRACE-1.0.0"
"""Immutable canonical physics-cadence trace schema identity."""


def _finite_float(value: Any, *, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise InvalidRuntimeContract(f"canonical trace field {name} is non-finite")
    return result


def _finite_vector(value: Any, *, name: str, shape: tuple[int, ...]) -> list[Any]:
    array = np.asarray(value, dtype=np.float64).reshape(shape)
    if not np.isfinite(array).all():
        raise InvalidRuntimeContract(f"canonical trace field {name} is non-finite")
    return [float(item) for item in array.reshape(-1)]


def _optional_finite_vector(
    value: Any,
    *,
    name: str,
    shape: tuple[int, ...],
) -> list[Any] | None:
    return None if value is None else _finite_vector(value, name=name, shape=shape)


def _required_sample_vector(sample: Any, name: str, shape: tuple[int, ...]) -> list[Any]:
    value = getattr(sample, name, None)
    if value is None:
        raise InvalidRuntimeContract(f"live trace sample is missing {name}")
    return _finite_vector(value, name=name, shape=shape)


def canonical_trace_record(sample: Any) -> dict[str, Any]:
    """Serialize one trusted post-step sample in fixed field order.

    ``BiomechanicalSample`` remains the owner of measurement construction;
    this function is the sole owner of the sealed trace wire representation.
    It deliberately excludes event-derived metrics, whose authority remains
    ``CMJEventDetector``.
    """
    physics_index = getattr(sample, "physics_index", None)
    if physics_index is None or isinstance(physics_index, bool) or int(physics_index) < 1:
        raise InvalidRuntimeContract("canonical trace requires a positive physics_sample_index")
    physics_index = int(physics_index)
    time_s = _finite_float(getattr(sample, "time_s"), name="time_s")
    expected_time = physics_index * float(PHYSICS_TIMESTEP_S)
    if abs(time_s - expected_time) > 1e-12:
        raise InvalidRuntimeContract(
            "canonical trace time_s does not match physics_sample_index at 8 kHz"
        )
    plate = np.asarray(getattr(sample, "plate_wrench_N_Nm"), dtype=np.float64).reshape(3, 6)
    if not np.isfinite(plate).all():
        raise InvalidRuntimeContract("canonical trace plate wrench is non-finite")
    whole = getattr(sample, "whole_support_wrench_N_Nm", None)
    if whole is None:
        raise InvalidRuntimeContract("canonical trace sample is missing whole support wrench")
    cop = getattr(sample, "cop_world_xy_m", None)
    cop_valid = getattr(sample, "cop_validity", None)
    if cop is None or cop_valid is None:
        raise InvalidRuntimeContract("canonical trace sample is missing COP validity data")
    action = getattr(sample, "accepted_action", None)
    torque = getattr(sample, "realized_anatomical_torque_Nm", None)
    if action is None or torque is None:
        raise InvalidRuntimeContract("canonical trace sample is missing action or realized torque")
    override_flags = getattr(sample, "actuator_override_flags", None)
    ordered_flags = None
    if override_flags is not None:
        ordered_flags = {
            str(key): [bool(item) for item in np.asarray(override_flags[key], dtype=bool).reshape(-1)]
            for key in sorted(override_flags)
        }
    cop_array = np.asarray(cop, dtype=np.float64).reshape(2, 2)
    cop_valid_array = np.asarray(cop_valid, dtype=bool).reshape(2)
    if not np.isfinite(cop_array).all():
        raise InvalidRuntimeContract("canonical trace COP is non-finite")
    origin_world_array = getattr(sample, "plate_origin_world_m", None)
    if origin_world_array is None:
        raise InvalidRuntimeContract("canonical trace sample is missing plate origins")
    origin_world_array = np.asarray(origin_world_array, dtype=np.float64).reshape(2, 3)
    if not np.isfinite(origin_world_array).all():
        raise InvalidRuntimeContract("canonical trace plate origins are non-finite")
    origin_world = [[float(item) for item in row] for row in origin_world_array]
    # Dict insertion order is intentional: it is the documented wire order.
    record: dict[str, Any] = {
        "time_s": time_s,
        "physics_sample_index": physics_index,
        "control_index": None if getattr(sample, "control_index", None) is None else int(sample.control_index),
        "left_plate_wrench_world_N_Nm": [float(item) for item in plate[0]],
        "right_plate_wrench_world_N_Nm": [float(item) for item in plate[1]],
        "off_plate_wrench_world_N_Nm": [float(item) for item in plate[2]],
        "force_world_N": _finite_vector(np.asarray(whole).reshape(6)[:3], name="force_world_N", shape=(3,)),
        "moment_common_origin_world_Nm": _finite_vector(
            np.asarray(whole).reshape(6)[3:],
            name="moment_common_origin_world_Nm",
            shape=(3,),
        ),
        "common_origin_world_m": [0.0, 0.0, 0.0],
        "plate_origin_world_m": origin_world,
        "left_cop_world_xy_m": [float(item) for item in cop_array[0]],
        "right_cop_world_xy_m": [float(item) for item in cop_array[1]],
        "left_cop_valid": bool(cop_valid_array[0]),
        "right_cop_valid": bool(cop_valid_array[1]),
        "left_permitted_contact": bool(np.asarray(sample.permitted_support, dtype=bool).reshape(2)[0]),
        "right_permitted_contact": bool(np.asarray(sample.permitted_support, dtype=bool).reshape(2)[1]),
        "prohibited_contact": bool(sample.prohibited_contact),
        "com_position_world_m": _finite_vector(sample.com_position_m, name="com_position_world_m", shape=(3,)),
        "com_velocity_world_mps": _finite_vector(sample.com_velocity_mps, name="com_velocity_world_mps", shape=(3,)),
        "com_acceleration_world_mps2": _required_sample_vector(
            sample, "com_acceleration_world_mps2", (3,)
        ),
        "linear_momentum_world_kg_mps": _required_sample_vector(
            sample, "linear_momentum_world_kg_mps", (3,)
        ),
        "centroidal_h_world_kgm2ps": _required_sample_vector(
            sample, "centroidal_h_world_kgm2ps", (3,)
        ),
        "centroidal_hdot_world_kgm2ps2": _required_sample_vector(
            sample, "centroidal_hdot_world_kgm2ps2", (3,)
        ),
        "accepted_action": _finite_vector(action, name="accepted_action", shape=(15,)),
        "realized_torque_Nm": _finite_vector(torque, name="realized_torque_Nm", shape=(15,)),
        "native_limit_active": bool(sample.native_limit_active),
        "actuator_override_flags": ordered_flags,
        "actuator_capacity_lower_Nm": _optional_finite_vector(
            getattr(sample, "actuator_capacity_lower_Nm", None),
            name="actuator_capacity_lower_Nm",
            shape=(15,),
        ),
        "actuator_capacity_upper_Nm": _optional_finite_vector(
            getattr(sample, "actuator_capacity_upper_Nm", None),
            name="actuator_capacity_upper_Nm",
            shape=(15,),
        ),
        "active_power_signed_W": _finite_float(sample.active_power_signed_W, name="active_power_signed_W"),
        "active_power_positive_W": _finite_float(sample.active_power_positive_W, name="active_power_positive_W"),
        "active_power_negative_W": _finite_float(sample.active_power_negative_W, name="active_power_negative_W"),
        "passive_power_W": _finite_float(sample.passive_power_W, name="passive_power_W"),
        "damping_power_W": _finite_float(sample.damping_power_W, name="damping_power_W"),
        "limit_power_W": _finite_float(sample.limit_power_W, name="limit_power_W"),
        "accepted_action_digest": sample.accepted_action_digest,
        "post_step_state_digest": sample.post_step_state_digest,
        "model_revision": sample.model_revision,
        "runtime_revision": sample.runtime_revision,
        "extraction_status": str(sample.extraction_status),
    }
    return record


def canonical_trace_bytes(samples: Sequence[Any]) -> bytes:
    """Return deterministic UTF-8 JSON for a complete live physics trace."""
    ordered_samples = tuple(samples)
    records: list[dict[str, Any]] = []
    previous_index: int | None = None
    for sample in ordered_samples:
        record = canonical_trace_record(sample)
        index = int(record["physics_sample_index"])
        if previous_index is not None and index != previous_index + 1:
            raise InvalidRuntimeContract("canonical trace physics_sample_index has a dropped interval")
        previous_index = index
        records.append(record)
    payload = {
        "schema_version": TRACE_SCHEMA_VERSION,
        "physics_rate_hz": 8000,
        "physics_timestep_s": float(PHYSICS_TIMESTEP_S),
        "sample_count": len(records),
        "samples": records,
    }
    try:
        return json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise InvalidRuntimeContract("canonical trace serialization failed") from exc


def canonical_trace_digest(attempt_id: str, samples: Sequence[Any]) -> str:
    """Hash the schema-tagged canonical trace with explicit attempt framing."""
    payload = canonical_trace_bytes(samples)
    attempt = str(attempt_id).encode("utf-8")
    digest = hashlib.sha256()
    digest.update(TRACE_SCHEMA_VERSION.encode("ascii"))
    digest.update(len(attempt).to_bytes(8, "little"))
    digest.update(attempt)
    digest.update(len(payload).to_bytes(8, "little"))
    digest.update(payload)
    return digest.hexdigest()


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
    "TRACE_SCHEMA_VERSION",
    "canonical_trace_bytes",
    "canonical_trace_digest",
    "canonical_trace_record",
]
