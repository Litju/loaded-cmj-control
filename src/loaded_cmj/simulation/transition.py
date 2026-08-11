"""The authoritative physical transition for one loaded-CMJ control interval.

This module owns exactly one task-level accepted-action projection and one
40-substep MuJoCo transition.  Runtime sampling and event evaluation remain
callers: ``on_substep`` is only a callback at the existing post-physics
boundary, so those concerns do not become part of the physical owner.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import (
    ACTION_DIM,
    CONTROL_PERIOD_S,
    PHYSICS_TIMESTEP_S,
    SUBSTEPS_PER_CONTROL,
)
from loaded_cmj.simulation.plant import Plant


ACCEPTED_ACTION_MAX_STEP = 0.20

if SUBSTEPS_PER_CONTROL * PHYSICS_TIMESTEP_S != CONTROL_PERIOD_S:
    raise RuntimeError("frozen control interval is not 40 physics substeps")


@dataclass(frozen=True)
class TransitionResult:
    """Trusted outputs at the end of one live physical control interval."""

    accepted_action: np.ndarray
    realized_anatomical_torque: np.ndarray
    capacity_lower: np.ndarray
    capacity_upper: np.ndarray
    substeps_executed: int


SubstepCallback = Callable[
    [int, np.ndarray, Mapping[str, Any], np.ndarray, Mapping[str, float]],
    bool,
]


def _action_precondition(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (ACTION_DIM,):
        raise ValueError(f"{name} must have the frozen 15-D action shape")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite")
    if np.any(result < -1.0) or np.any(result > 1.0):
        raise ValueError(f"{name} must be in [-1, 1]")
    return result.copy()


def project_accepted_action(
    previous_accepted_action: np.ndarray,
    raw_action: np.ndarray,
) -> np.ndarray:
    """Project one validated raw command against the previous accepted one."""

    previous = _action_precondition(previous_accepted_action, "previous accepted action")
    raw = _action_precondition(raw_action, "raw action")
    return previous + np.clip(
        raw - previous,
        -ACCEPTED_ACTION_MAX_STEP,
        ACCEPTED_ACTION_MAX_STEP,
    )


def step_5ms(
    *,
    plant: Plant,
    data: mujoco.MjData,
    drive_state: drive.DriveState,
    previous_accepted_action: np.ndarray,
    raw_action: np.ndarray,
    on_substep: SubstepCallback | None = None,
) -> TransitionResult:
    """Advance the existing plant through exactly one 5 ms control interval.

    ``raw_action`` is expected to have passed the existing public action
    validator.  The precondition check is repeated here at the shared seam so
    direct characterization callers cannot bypass the frozen 15-D contract.
    ``on_substep`` runs after each MuJoCo step and may return ``False`` only
    for the runtime's existing terminal/fault boundary; without a callback,
    this function always executes all 40 physics steps.
    """

    accepted = project_accepted_action(previous_accepted_action, raw_action)
    realized = np.zeros(ACTION_DIM, dtype=np.float64)
    capacity_lower = np.zeros(ACTION_DIM, dtype=np.float64)
    capacity_upper = np.zeros(ACTION_DIM, dtype=np.float64)
    substeps_executed = 0

    for substep in range(SUBSTEPS_PER_CONTROL):
        drive_result = drive.drive_state_step(
            accepted,
            drive_state,
            plant.anatomical_coordinates(data),
            plant.anatomical_rates(data),
            PHYSICS_TIMESTEP_S,
        )
        realized = np.asarray(drive_result["tau"], dtype=np.float64).copy()
        capacity_lower = np.asarray(drive_result["capacity_lower"], dtype=np.float64).copy()
        capacity_upper = np.asarray(drive_result["capacity_upper"], dtype=np.float64).copy()
        interval_start_power = plant.realized_power_components(data, realized)
        plant.apply_anatomical_torque(data, realized)
        mujoco.mj_step(plant.model, data)
        substeps_executed = substep + 1
        if on_substep is not None and not on_substep(
            substep,
            accepted.copy(),
            drive_result,
            realized.copy(),
            interval_start_power,
        ):
            break

    return TransitionResult(
        accepted_action=accepted.copy(),
        realized_anatomical_torque=realized.copy(),
        capacity_lower=capacity_lower.copy(),
        capacity_upper=capacity_upper.copy(),
        substeps_executed=substeps_executed,
    )


__all__ = [
    "ACCEPTED_ACTION_MAX_STEP",
    "SubstepCallback",
    "TransitionResult",
    "project_accepted_action",
    "step_5ms",
]
