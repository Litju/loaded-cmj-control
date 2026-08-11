"""Local 5 ms plant-effectiveness diagnostics for F3.

The estimator is deliberately a characterization utility.  It uses the real
DriveState and MuJoCo transition, has no optimizer, and has no policy/event
imports.  It is not part of the public observation path.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, isfinite
import mujoco
import numpy as np

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import PHYSICS_TIMESTEP_S, SUBSTEPS_PER_CONTROL
from loaded_cmj.simulation.plant import Plant
from loaded_cmj.simulation.transition import step_5ms


EFFECTIVENESS_CONTRACT_VERSION = "LCMJ-V1-F3-EFFECTIVENESS-1.0.0"
ACTION_DIMENSION = 15
OUTPUT_NAMES = (
    "support_Fz_N",
    "support_Fx_N",
    "support_pitch_moment_about_COM_Nm",
    "support_margin_m",
    "COP_x_m",
    "COM_vx_mps",
    "COM_vz_mps",
    "H_pitch_kgm2ps",
    "Hdot_pitch_kgm2ps2",
    "trunk_pitch_rad",
    "trunk_angular_rate_pitch_rads",
    *tuple(f"realized_tau_{i:02d}_Nm" for i in range(ACTION_DIMENSION)),
    "active_power_W",
    "minimum_actuator_headroom_Nm",
    "contact_valid",
)
SAGITTAL_OUTPUT_INDICES = (0, 1, 2, 3, 5, 6, 7, 8)
# Fixed dimensional scales are reported with the conditioning result.  They
# are unit normalization for SVD only, never an objective or weighted score.
SAGITTAL_ROW_SCALES = np.asarray((1000.0, 1000.0, 1000.0, 0.1, 1.0, 1.0, 1.0, 10.0), dtype=np.float64)


@dataclass(frozen=True)
class LocalEffectivenessResult:
    contract_version: str
    physics_timestep_s: float
    control_interval_s: float
    substeps: int
    perturbation_epsilon: float
    action: np.ndarray
    baseline_output: np.ndarray
    matrix: np.ndarray
    accepted_channels: tuple[int, ...]
    rejected_channels: tuple[int, ...]
    contact_mode_preserved: bool
    actual_drive_path_used: bool
    output_names: tuple[str, ...]
    singular_values_raw: np.ndarray
    singular_values_scaled: np.ndarray
    scaled_rank: int
    scaled_condition_number: float
    channel_effect_norms_scaled: np.ndarray
    direct_effect_channel_by_output: dict[str, int]


def _finite_vector(values: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains non-finite values")
    return result


def _trunk_state(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, float]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_head_arms")
    rotation = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
    pitch = atan2(float(rotation[0, 2]), float(rotation[2, 2]))
    velocity = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, int(mujoco.mjtObj.mjOBJ_BODY), body_id, velocity, 0)
    return pitch, float(velocity[1])


def _output_sample(
    plant: Plant,
    data: mujoco.MjData,
    tau: np.ndarray,
    capacity_lower: np.ndarray,
    capacity_upper: np.ndarray,
    baseline_active_by_foot: np.ndarray,
) -> tuple[np.ndarray, bool]:
    mujoco.mj_forward(plant.model, data)
    com = plant.center_of_mass(data)
    summary = plant.contact_wrench_summary(data)
    active_by_foot = np.asarray(summary["active_by_foot"], dtype=bool)
    valid = bool(
        summary["contact_active"]
        and np.array_equal(active_by_foot, baseline_active_by_foot)
        and not summary["prohibited_contact"]
        and np.asarray(summary["cop_valid"], dtype=bool).all()
    )
    support_wrench = np.asarray(summary["whole_wrench"], dtype=np.float64)
    moment_com = support_wrench[3:] - np.cross(com, support_wrench[:3])
    h = plant.centroidal_angular_momentum(data, com)
    hdot = plant.centroidal_hdot_from_external_wrench(data, com, support_wrench=support_wrench)
    latched = tuple(bool(x) for x in baseline_active_by_foot)
    support_margin = plant.support_margin(data, com, latched)
    cop_world = np.asarray(summary["cop_world_xy"], dtype=np.float64)
    cop_x = float(np.mean(cop_world[:, 0])) if valid else 0.0
    pitch, pitch_rate = _trunk_state(plant.model, data)
    power = plant.realized_power_components(data, tau)
    lower_headroom = _finite_vector(tau - capacity_lower, "lower actuator headroom")
    upper_headroom = _finite_vector(capacity_upper - tau, "upper actuator headroom")
    minimum_headroom = float(np.min(np.minimum(lower_headroom, upper_headroom)))
    output = np.r_[
        support_wrench[2],
        support_wrench[0],
        moment_com[1],
        support_margin,
        cop_x,
        plant.center_of_mass_velocity(data)[0],
        plant.center_of_mass_velocity(data)[2],
        h[1],
        hdot[1],
        pitch,
        pitch_rate,
        tau,
        float(power["active_power_signed_W"]),
        minimum_headroom,
        1.0 if valid else 0.0,
    ]
    if len(output) != len(OUTPUT_NAMES) or not np.isfinite(output).all():
        raise ValueError("effectiveness output is non-finite or schema-mismatched")
    return output.astype(np.float64), valid


def _transition(
    plant: Plant,
    initial_data: mujoco.MjData,
    initial_drive_state: drive.DriveState,
    action: np.ndarray,
    baseline_active_by_foot: np.ndarray,
) -> tuple[np.ndarray, bool, bool]:
    data = mujoco.MjData(plant.model)
    mujoco.mj_copyData(data, plant.model, initial_data)
    state = initial_drive_state.copy()
    transition = step_5ms(
        plant=plant,
        data=data,
        drive_state=state,
        previous_accepted_action=action,
        raw_action=action,
    )
    output, valid = _output_sample(
        plant,
        data,
        transition.realized_anatomical_torque,
        transition.capacity_lower,
        transition.capacity_upper,
        baseline_active_by_foot,
    )
    return output, valid, bool(np.isfinite(transition.realized_anatomical_torque).all())


def estimate_local_effectiveness(
    plant: Plant,
    initial_data: mujoco.MjData,
    initial_drive_state: drive.DriveState,
    action: np.ndarray,
    *,
    epsilon: float = 1.0e-3,
) -> LocalEffectivenessResult:
    """Estimate `dy/du` by symmetric finite differences over one control step."""

    base_action = _finite_vector(action, "action")
    if base_action.size != ACTION_DIMENSION or np.any(np.abs(base_action) > 1.0):
        raise ValueError("effectiveness action must be a finite 15-D action in [-1, 1]")
    epsilon = float(epsilon)
    if not isfinite(epsilon) or epsilon <= 0.0 or epsilon >= 0.1:
        raise ValueError("effectiveness epsilon must be in (0, 0.1)")
    baseline_summary = plant.contact_wrench_summary(initial_data)
    baseline_active = np.asarray(baseline_summary["active_by_foot"], dtype=bool).copy()
    baseline_output, baseline_valid, baseline_drive_ok = _transition(
        plant, initial_data, initial_drive_state, base_action, baseline_active
    )
    matrix = np.zeros((len(OUTPUT_NAMES), ACTION_DIMENSION), dtype=np.float64)
    accepted: list[int] = []
    rejected: list[int] = []
    drive_ok = baseline_drive_ok
    for channel in range(ACTION_DIMENSION):
        if base_action[channel] - epsilon < -1.0 or base_action[channel] + epsilon > 1.0:
            rejected.append(channel)
            continue
        plus = base_action.copy()
        minus = base_action.copy()
        plus[channel] += epsilon
        minus[channel] -= epsilon
        output_plus, valid_plus, drive_plus = _transition(
            plant, initial_data, initial_drive_state, plus, baseline_active
        )
        output_minus, valid_minus, drive_minus = _transition(
            plant, initial_data, initial_drive_state, minus, baseline_active
        )
        drive_ok = drive_ok and drive_plus and drive_minus
        if not (baseline_valid and valid_plus and valid_minus):
            rejected.append(channel)
            continue
        matrix[:, channel] = (output_plus - output_minus) / (2.0 * epsilon)
        accepted.append(channel)
    scaled = matrix[np.asarray(SAGITTAL_OUTPUT_INDICES)] / SAGITTAL_ROW_SCALES[:, None]
    singular_raw = np.linalg.svd(matrix[np.asarray(SAGITTAL_OUTPUT_INDICES)], compute_uv=False)
    singular_scaled = np.linalg.svd(scaled, compute_uv=False)
    rank_tolerance = 100.0 * max(scaled.shape) * singular_scaled[0] * np.finfo(np.float64).eps
    rank = int(np.count_nonzero(singular_scaled > rank_tolerance)) if singular_scaled.size else 0
    positive = singular_scaled[singular_scaled > rank_tolerance]
    condition = float(positive[0] / positive[-1]) if positive.size else float("inf")
    norms = np.linalg.norm(scaled, axis=0)
    direct = {
        OUTPUT_NAMES[row]: int(np.argmax(np.abs(matrix[row])))
        for row in SAGITTAL_OUTPUT_INDICES
    }
    return LocalEffectivenessResult(
        contract_version=EFFECTIVENESS_CONTRACT_VERSION,
        physics_timestep_s=PHYSICS_TIMESTEP_S,
        control_interval_s=PHYSICS_TIMESTEP_S * SUBSTEPS_PER_CONTROL,
        substeps=SUBSTEPS_PER_CONTROL,
        perturbation_epsilon=epsilon,
        action=base_action,
        baseline_output=baseline_output,
        matrix=matrix,
        accepted_channels=tuple(accepted),
        rejected_channels=tuple(rejected),
        contact_mode_preserved=not rejected and baseline_valid,
        actual_drive_path_used=drive_ok,
        output_names=OUTPUT_NAMES,
        singular_values_raw=singular_raw,
        singular_values_scaled=singular_scaled,
        scaled_rank=rank,
        scaled_condition_number=condition,
        channel_effect_norms_scaled=norms,
        direct_effect_channel_by_output=direct,
    )


__all__ = [
    "ACTION_DIMENSION",
    "EFFECTIVENESS_CONTRACT_VERSION",
    "LocalEffectivenessResult",
    "OUTPUT_NAMES",
    "SAGITTAL_OUTPUT_INDICES",
    "SAGITTAL_ROW_SCALES",
    "estimate_local_effectiveness",
]
