"""V3 causal loaded-CMJ launch runtime and native telemetry (RES-85).

Authority: ``LCMJ_RES85_CAUSAL_LAUNCH_CONTROL_V1``

This module owns the deterministic episode: the sealed RES-83 Plant is
initialised to a settled flat bilateral stance, the :class:`V3LaunchController`
runs at native resolution (500 Hz) with sample-then-step frames exactly as the
RES-84 authority streams them, and every native sample required to
independently reconstruct the episode is recorded.

Event quantities (takeoff occurrence, confirmation, apex/H2, comparator,
impulse cross-check) are computed **only** from the RES-84 measurement
primitives; this module never redefines contact, support, SYSTEM_COM or event
semantics.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Any, Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3 import measurement as M
from loaded_cmj.v3.actuation import CHANNELS, STAGE_NAMES
from loaded_cmj.v3.controller import (
    PHASE_ORDER,
    V3ControllerConfig,
    V3ControllerFault,
    V3ControlStep,
    V3LaunchController,
)
from loaded_cmj.v3.plant import V3Plant
from loaded_cmj.v3.plant import build_zero_passive_model as P_build_zero_passive_model

V3_LAUNCH_RUNTIME_AUTHORITY_ID = "LCMJ_RES85_CAUSAL_LAUNCH_CONTROL_V1"

PHASE_INDEX = {phase.value: i for i, phase in enumerate(PHASE_ORDER)}
STAGE_INDEX = {name: i for i, name in enumerate(STAGE_NAMES)}

DEFAULT_HORIZON_S = 3.0


class V3RuntimeError(RuntimeError):
    """Explicit runtime failure (fail closed; never a silent zero action)."""


# ===========================================================================
# deterministic initial state (settled flat bilateral stance)
# ===========================================================================
def settle_standing_stance(plant: V3Plant, data: mujoco.MjData,
                           ) -> float:
    """Materialize the settled flat bilateral stance with Fz = system weight.

    The penetration depth is found by deterministic bisection on the static
    forward solve (same construction as the RES-84 flat-stance probe); the
    Plant, its contact parameters and its solver are untouched.
    """
    angles = {name: 0.0 for name in CHANNELS}
    plant.reset(data)
    plant.set_joint_angles(data, angles)
    plant.drop_to_floor(data, clearance_m=0.0)
    plant.balance_root_x(data)
    mujoco.mj_forward(plant.model, data)
    base = data.qpos.copy()

    def total_fz(depth: float) -> float:
        data.qpos[:] = base
        data.qpos[plant.idx.qadr["root_tz"]] -= depth
        mujoco.mj_forward(plant.model, data)
        return float(M.total_ground_wrench(plant, data).force_world_n[2])

    lo, hi = 0.0, 2.0e-3
    for _ in range(90):
        mid = 0.5 * (lo + hi)
        if total_fz(mid) < C.V3_SYSTEM_WEIGHT_N:
            lo = mid
        else:
            hi = mid
    data.qpos[:] = base
    data.qpos[plant.idx.qadr["root_tz"]] -= hi
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(plant.model, data)
    return float(hi)


# ===========================================================================
# telemetry bundle
# ===========================================================================
@dataclass
class V3LaunchTelemetry:
    """Every native sample required to independently reconstruct the episode."""

    index: np.ndarray
    time_s: np.ndarray
    phase_code: np.ndarray
    phase_name: np.ndarray
    transition_reason: np.ndarray
    commanded_nm: np.ndarray
    applied_nm: np.ndarray
    torque_rate_nm_per_s: np.ndarray
    saturation_stage_code: np.ndarray
    saturation_stage: np.ndarray
    moment_margin_nm: np.ndarray
    rate_margin_nm: np.ndarray
    power_margin_w: np.ndarray
    symmetry_asymmetry_nm: np.ndarray
    joint_q: np.ndarray
    joint_qd: np.ndarray
    joint_power_w: np.ndarray
    joint_work_j: np.ndarray
    mtp_active_power_w: np.ndarray
    mtp_active_work_j: np.ndarray
    mtp_passive_power_w: np.ndarray
    mtp_passive_work_j: np.ndarray
    mtp_total_power_w: np.ndarray
    mtp_total_work_j: np.ndarray
    mtp_active_work_signed_j: np.ndarray
    mtp_passive_work_signed_j: np.ndarray
    mtp_total_work_signed_j: np.ndarray
    mtp_gated: np.ndarray
    mtp_passive_moment_nm: np.ndarray
    mtp_passive_reconstruction_residual_nm: np.ndarray
    joint_limit_margin: np.ndarray
    com_world_m: np.ndarray
    com_velocity_world_m_s: np.ndarray
    left_wrench: np.ndarray
    right_wrench: np.ndarray
    total_wrench: np.ndarray
    legal_plantar_active: np.ndarray
    legal_plantar_detected: np.ndarray
    prohibited_detected: np.ndarray
    prohibited_active: np.ndarray
    nonplantar_floor_active: np.ndarray
    cop_validity: np.ndarray
    cop_x_m: np.ndarray
    support_mode: np.ndarray
    support_sagittal_margin_m: np.ndarray
    support_hull_area_m2: np.ndarray
    left_clearance_m: np.ndarray
    right_clearance_m: np.ndarray
    fz_des_n: np.ndarray
    fx_des_n: np.ndarray
    cop_des_x_m: np.ndarray
    cop_clamped_x_m: np.ndarray
    cop_clamped: np.ndarray
    az_des_m_s2: np.ndarray
    quiet_samples: np.ndarray
    takeoff_candidate_index: np.ndarray
    takeoff_confirmed: np.ndarray
    out_of_plane_fy_n: np.ndarray
    out_of_plane_mx_nm: np.ndarray
    out_of_plane_mz_nm: np.ndarray
    out_of_plane_fy_over_fz: np.ndarray
    out_of_plane_mx_over_fz_m: np.ndarray
    out_of_plane_mz_over_fz_m: np.ndarray

    def arrays(self) -> dict[str, np.ndarray]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    def canonical_digest(self) -> str:
        """Order-independent digest over dtype/shape/bytes of every array."""
        digest = hashlib.sha256()
        for name in sorted(self.__dataclass_fields__):
            arr = getattr(self, name)
            digest.update(name.encode("utf-8"))
            digest.update(str(arr.dtype).encode("utf-8"))
            digest.update(str(arr.shape).encode("utf-8"))
            digest.update(canonical_array_bytes(arr))
        return digest.hexdigest()


def canonical_array_bytes(arr: np.ndarray) -> bytes:
    """Deterministic byte view of one telemetry array.

    Object-dtype arrays (labels, phase names, validation states) are encoded as
    UTF-8 records separated by the unit-separator byte; numeric arrays are
    contiguous little-endian buffers.  The encoding is content-only, so two
    identical episodes always produce identical bytes.
    """
    array = np.asarray(arr)
    if array.dtype == object:
        flat = [str(x) for x in array.ravel()]
        payload = "\x1f".join(flat).encode("utf-8")
        return f"object:{array.shape}:".encode("utf-8") + payload
    return np.ascontiguousarray(array).tobytes()


@dataclass
class V3LaunchEpisode:
    status: str
    frames: list[M.V3NativeFrame]
    steps: list[V3ControlStep]
    telemetry: V3LaunchTelemetry
    events: dict[str, Any]
    phases_visited: list[str]
    fault: str | None = None
    warnings: list[str] = field(default_factory=list)


# ===========================================================================
# episode runner
# ===========================================================================
def _empty_telemetry() -> dict[str, list]:
    fields = [
        "index", "time_s", "phase_code", "phase_name", "transition_reason",
        "commanded_nm", "applied_nm", "torque_rate_nm_per_s", "saturation_stage_code",
        "saturation_stage", "moment_margin_nm", "rate_margin_nm", "power_margin_w",
        "symmetry_asymmetry_nm", "joint_q", "joint_qd", "joint_power_w", "joint_work_j",
        "mtp_active_power_w", "mtp_active_work_j", "mtp_passive_power_w",
        "mtp_passive_work_j", "mtp_total_power_w", "mtp_total_work_j",
        "mtp_active_work_signed_j", "mtp_passive_work_signed_j",
        "mtp_total_work_signed_j", "mtp_gated", "mtp_passive_moment_nm",
        "mtp_passive_reconstruction_residual_nm",
        "joint_limit_margin", "com_world_m", "com_velocity_world_m_s",
        "left_wrench", "right_wrench", "total_wrench", "legal_plantar_active",
        "legal_plantar_detected", "prohibited_detected", "prohibited_active",
        "nonplantar_floor_active", "cop_validity", "cop_x_m", "support_mode",
        "support_sagittal_margin_m", "support_hull_area_m2", "left_clearance_m",
        "right_clearance_m", "fz_des_n", "fx_des_n", "cop_des_x_m", "cop_clamped_x_m",
        "cop_clamped", "az_des_m_s2", "quiet_samples", "takeoff_candidate_index",
        "takeoff_confirmed", "out_of_plane_fy_n", "out_of_plane_mx_nm",
        "out_of_plane_mz_nm", "out_of_plane_fy_over_fz", "out_of_plane_mx_over_fz_m",
        "out_of_plane_mz_over_fz_m",
    ]
    return {name: [] for name in fields}


def run_launch_episode(*, horizon_s: float = DEFAULT_HORIZON_S,
                       zero_passive: bool = False,
                       controller_config: V3ControllerConfig | None = None,
                       plant: V3Plant | None = None,
                       ) -> V3LaunchEpisode:
    """Run one deterministic RES-85 launch episode at native resolution."""
    if plant is None:
        plant = V3Plant() if not zero_passive else V3Plant(P_build_zero_passive_model())
    model = plant.model
    dt = float(model.opt.timestep)
    if abs(dt - M.NATIVE_DT_S) > 1e-12:
        raise V3RuntimeError(f"native timestep {dt!r} s != RES-84 native dt {M.NATIVE_DT_S!r} s")
    data = plant.make_data()
    settle_standing_stance(plant, data)

    config = controller_config or V3ControllerConfig(dt_s=dt)
    controller = V3LaunchController(plant, data, config=config)

    n_samples = int(round(horizon_s / dt))
    frames: list[M.V3NativeFrame] = []
    steps: list[V3ControlStep] = []
    log = _empty_telemetry()
    cumulative_work = np.zeros(len(CHANNELS))
    mtp_active_work = np.zeros(2)
    mtp_passive_work = np.zeros(2)
    mtp_total_work = np.zeros(2)
    mtp_active_work_signed = np.zeros(2)
    mtp_passive_work_signed = np.zeros(2)
    mtp_total_work_signed = np.zeros(2)
    phases_visited: list[str] = []
    warnings: list[str] = []
    fault: str | None = None
    status = "COMPLETED"

    for k in range(n_samples):
        t = k * dt
        frame = M.native_frame(plant, data, k, t)
        snapshot = M.measure(plant, data, flight_context=controller.phase.value in
                             ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"))
        frames.append(frame)
        try:
            step = controller.update(frame, snapshot, frames, plant, data)
        except V3ControllerFault as exc:
            fault = str(exc)
            status = "FAIL_CLOSED"
            break
        steps.append(step)
        if not phases_visited or phases_visited[-1] != step.phase:
            phases_visited.append(step.phase)

        qdot = np.array([data.qvel[plant.idx.vadr[name]] for name in CHANNELS])
        applied = step.applied_nm
        joint_power = applied * qdot
        cumulative_work = cumulative_work + joint_power * dt
        left_mtp_qd = float(data.qvel[plant.idx.vadr["left_mtp"]])
        right_mtp_qd = float(data.qvel[plant.idx.vadr["right_mtp"]])
        passive_left = float(data.qfrc_passive[plant.idx.vadr["left_mtp"]])
        passive_right = float(data.qfrc_passive[plant.idx.vadr["right_mtp"]])
        active_power = np.array([applied[7] * left_mtp_qd, applied[8] * right_mtp_qd])
        passive_power = np.array([passive_left * left_mtp_qd, passive_right * right_mtp_qd])
        total_power = active_power + passive_power
        mtp_active_work = mtp_active_work + np.maximum(active_power, 0.0) * dt
        mtp_passive_work = mtp_passive_work + np.maximum(passive_power, 0.0) * dt
        mtp_total_work = mtp_total_work + np.maximum(total_power, 0.0) * dt
        mtp_active_work_signed = mtp_active_work_signed + active_power * dt
        mtp_passive_work_signed = mtp_passive_work_signed + passive_power * dt
        mtp_total_work_signed = mtp_active_work_signed + mtp_passive_work_signed

        log["index"].append(int(frame.index))
        log["time_s"].append(float(frame.time_s))
        log["phase_code"].append(PHASE_INDEX[step.phase])
        log["phase_name"].append(step.phase)
        log["transition_reason"].append(step.transition_reason)
        log["commanded_nm"].append(np.asarray(step.actuation.commanded_nm))
        log["applied_nm"].append(np.asarray(applied))
        log["torque_rate_nm_per_s"].append(np.asarray(step.actuation.torque_rate_nm_per_s))
        log["saturation_stage_code"].append(
            np.array([STAGE_INDEX[s] for s in step.actuation.saturation_stage]))
        log["saturation_stage"].append(np.asarray(step.actuation.saturation_stage))
        log["moment_margin_nm"].append(np.asarray(step.actuation.moment_margin_nm))
        log["rate_margin_nm"].append(np.asarray(step.actuation.rate_margin_nm))
        log["power_margin_w"].append(np.asarray(step.actuation.power_margin_w))
        log["symmetry_asymmetry_nm"].append(step.actuation.symmetry_asymmetry_nm)
        log["joint_q"].append(np.array(data.qpos, dtype=np.float64, copy=True))
        log["joint_qd"].append(np.array(data.qvel, dtype=np.float64, copy=True))
        log["joint_power_w"].append(joint_power)
        log["joint_work_j"].append(cumulative_work.copy())
        log["mtp_active_power_w"].append(active_power)
        log["mtp_active_work_j"].append(mtp_active_work.copy())
        log["mtp_passive_power_w"].append(passive_power)
        log["mtp_passive_work_j"].append(mtp_passive_work.copy())
        log["mtp_total_power_w"].append(total_power)
        log["mtp_total_work_j"].append(mtp_total_work.copy())
        log["mtp_active_work_signed_j"].append(mtp_active_work_signed.copy())
        log["mtp_passive_work_signed_j"].append(mtp_passive_work_signed.copy())
        log["mtp_total_work_signed_j"].append(mtp_total_work_signed.copy())
        log["mtp_gated"].append(np.asarray(step.actuation.mtp_gated))
        log["mtp_passive_moment_nm"].append(np.array([passive_left, passive_right]))
        log["mtp_passive_reconstruction_residual_nm"].append(
            controller.passive_reconstruction_residual_nm)
        log["joint_limit_margin"].append(controller._joint_limit_margin(data))
        log["com_world_m"].append(np.asarray(frame.com_world_m))
        log["com_velocity_world_m_s"].append(np.asarray(frame.com_velocity_world_m_s))
        log["left_wrench"].append(snapshot.left_foot_wrench.as_array())
        log["right_wrench"].append(snapshot.right_foot_wrench.as_array())
        log["total_wrench"].append(snapshot.total_ground_wrench.as_array())
        log["legal_plantar_active"].append(int(frame.legal_plantar_active))
        log["legal_plantar_detected"].append(int(frame.legal_plantar_detected))
        log["prohibited_detected"].append(int(frame.prohibited_detected))
        log["prohibited_active"].append(int(frame.prohibited_active))
        log["nonplantar_floor_active"].append(int(frame.nonplantar_floor_active))
        log["cop_validity"].append(frame.cop_validity)
        log["cop_x_m"].append(np.nan if frame.cop_x_m is None else float(frame.cop_x_m))
        log["support_mode"].append(frame.support_mode)
        log["support_sagittal_margin_m"].append(
            np.nan if snapshot.support_hull.sagittal_margin_m is None
            else float(snapshot.support_hull.sagittal_margin_m))
        log["support_hull_area_m2"].append(
            np.nan if snapshot.support_hull.area_m2 is None
            else float(snapshot.support_hull.area_m2))
        log["left_clearance_m"].append(float(frame.left_clearance_m))
        log["right_clearance_m"].append(float(frame.right_clearance_m))
        log["fz_des_n"].append(step.fz_des_n)
        log["fx_des_n"].append(step.fx_des_n)
        log["cop_des_x_m"].append(step.x_cop_des_m)
        log["cop_clamped_x_m"].append(step.x_cop_clamped_m)
        log["cop_clamped"].append(bool(step.cop_clamped))
        log["az_des_m_s2"].append(step.az_des_m_s2)
        log["quiet_samples"].append(int(step.quiet_samples))
        log["takeoff_candidate_index"].append(
            -1 if step.takeoff_candidate_index is None else int(step.takeoff_candidate_index))
        log["takeoff_confirmed"].append(bool(step.takeoff_confirmed))
        log["out_of_plane_fy_n"].append(float(snapshot.out_of_plane.total_fy_n))
        log["out_of_plane_mx_nm"].append(float(snapshot.out_of_plane.total_mx_nm))
        log["out_of_plane_mz_nm"].append(float(snapshot.out_of_plane.total_mz_nm))
        log["out_of_plane_fy_over_fz"].append(float(snapshot.out_of_plane.fy_over_fz))
        log["out_of_plane_mx_over_fz_m"].append(float(snapshot.out_of_plane.mx_over_fz_m))
        log["out_of_plane_mz_over_fz_m"].append(float(snapshot.out_of_plane.mz_over_fz_m))

        data.ctrl[:] = applied
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    telemetry = V3LaunchTelemetry(**{name: _stack(name, values)
                                     for name, values in log.items()})
    events = extract_events(frames, controller, telemetry)
    if controller._pending_candidate is not None and not controller._takeoff_confirmed:
        warnings.append("TAKEOFF_CONFIRM_UNRESOLVED_AT_HORIZON")
    return V3LaunchEpisode(
        status=status,
        frames=frames,
        steps=steps,
        telemetry=telemetry,
        events=events,
        phases_visited=phases_visited,
        fault=fault,
        warnings=warnings,
    )


def _stack(name: str, values: list) -> np.ndarray:
    if not values:
        if name in ("phase_name", "transition_reason", "cop_validity", "support_mode"):
            return np.asarray([], dtype=object)
        if name in ("saturation_stage",):
            return np.asarray([], dtype=object)
        return np.zeros((0,), dtype=np.float64)
    if name in ("phase_name", "transition_reason", "cop_validity", "support_mode",
                "saturation_stage"):
        try:
            return np.asarray(values, dtype=object)
        except ValueError:
            return np.asarray(values, dtype=object)
    first = np.asarray(values[0])
    if first.ndim == 0:
        if name in ("cop_clamped", "takeoff_confirmed", "mtp_gated"):
            return np.asarray(values, dtype=np.bool_)
        return np.asarray(values, dtype=np.float64)
    return np.asarray(values, dtype=np.float64)


# ===========================================================================
# event extraction (RES-84 primitives only)
# ===========================================================================
def extract_events(frames: Sequence[M.V3NativeFrame], controller: V3LaunchController,
                   telemetry: V3LaunchTelemetry) -> dict[str, Any]:
    events: dict[str, Any] = {}
    occurrence = M.detect_takeoff_occurrence(frames) if frames else None
    events["takeoff_occurrence"] = None if occurrence is None else {
        "valid": occurrence.valid,
        "time_s": occurrence.occurrence_time_s,
        "native_index": occurrence.native_index,
        "last_support_index": occurrence.last_support_index,
        "bracket_weight": occurrence.bracket_weight,
        "interpolated": occurrence.interpolated,
        "com_z_m": None if occurrence.com_world_m is None else occurrence.com_world_m[2],
        "com_vz_m_s": (None if occurrence.com_velocity_world_m_s is None
                       else occurrence.com_velocity_world_m_s[2]),
        "left_clearance_m": occurrence.left_clearance_m,
        "right_clearance_m": occurrence.right_clearance_m,
        "reason": occurrence.reason,
    }
    confirmation = controller._confirmation
    if confirmation is None and occurrence is not None and occurrence.valid and frames:
        confirmation = M.confirm_takeoff(frames, occurrence)
    events["takeoff_confirmation"] = None if confirmation is None else {
        "confirmed": confirmation.confirmed,
        "checks": [{"check": n, "pass": ok, "detail": d} for n, ok, d in confirmation.checks],
        "failed_checks": list(confirmation.failed_checks),
        "confirmation_sample": confirmation.confirmation_sample,
        "confirmation_time_s": confirmation.confirmation_time_s,
        "confirmation_elapsed_s": confirmation.confirmation_elapsed_s,
        "bilateral_clearance_max_m": confirmation.bilateral_clearance_max_m,
        "clearance_guard_m": confirmation.clearance_guard_m,
    }
    comparator = None
    if occurrence is not None and occurrence.valid and frames:
        result = M.force_takeoff_comparator(frames, occurrence)
        comparator = {
            "status": result.status,
            "triggered": result.triggered,
            "invalid": result.invalid,
            "onset_time_s": result.onset_time_s,
            "onset_offset_s": result.onset_offset_s,
            "confirmation_time_s": result.confirmation_time_s,
            "confirmation_offset_s": result.confirmation_offset_s,
            "k_d": result.k_d,
            "required_true_samples": result.required_true_samples,
            "true_sample_count": result.true_sample_count,
            "left_fz_at_trigger_n": result.left_fz_at_trigger_n,
            "right_fz_at_trigger_n": result.right_fz_at_trigger_n,
            "total_fz_at_trigger_n": result.total_fz_at_trigger_n,
            "note": result.note,
        }
    events["diagnostic_comparator"] = comparator
    if confirmation is not None and confirmation.confirmed:
        apex = M.detect_apex(frames, confirmation)
        events["apex_h2"] = {
            "evaluable": apex.evaluable,
            "apex_time_s": apex.apex_time_s,
            "com_z_apex_m": apex.com_z_apex_m,
            "com_z_takeoff_m": apex.com_z_takeoff_m,
            "h2_support_m": apex.h2_support_m,
            "takeoff_vz_m_s": apex.takeoff_vz_m_s,
            "ballistic_height_m": apex.ballistic_height_m,
            "ballistic_cross_check_delta_m": apex.ballistic_cross_check_delta_m,
            "reason": apex.reason,
        }
        k = confirmation.occurrence.native_index
        apex_index = next((i for i in range(k + 1, len(frames))
                           if frames[i].com_velocity_world_m_s[2] <= 0.0), None)
        touchdown = next((i for i in range(k + 1, len(frames))
                          if frames[i].legal_plantar_active > 0), None)
        i1 = apex_index if apex_index is not None else (
            len(frames) - 1 if touchdown is None else touchdown)
        impulse = M.vertical_impulse_between(frames, k, i1)
        derived_delta_vz = impulse / M.SYSTEM_MASS_KG
        measured_delta_vz = (frames[i1].com_velocity_world_m_s[2]
                             - frames[k].com_velocity_world_m_s[2])
        events["impulse_cross_check"] = {
            "occurrence_index": k,
            "cross_check_end_index": i1,
            "cross_check_end_reason": ("APEX_VZ_NONPOSITIVE_CROSSING"
                                       if apex_index is not None else "TOUCHDOWN_OR_STREAM_END"),
            "impulse_n_s": impulse,
            "impulse_derived_delta_vz_m_s": derived_delta_vz,
            "measured_delta_vz_m_s": measured_delta_vz,
            "occurrence_vz_m_s": confirmation.occurrence.com_velocity_world_m_s[2],
            "residual_m_s": derived_delta_vz - measured_delta_vz,
            "rule": "LEFT_RECTANGLE_INTEGRATOR_CONSISTENT",
        }
    else:
        events["apex_h2"] = {"evaluable": False, "reason": "TAKEOFF_NOT_CONFIRMED"}
        events["impulse_cross_check"] = {"evaluable": False}

    claim_end = None
    for i in range(len(telemetry.index)):
        if telemetry.phase_name[i] == "LANDING_PREP" and telemetry.legal_plantar_active[i] > 0:
            claim_end = {
                "sample": int(telemetry.index[i]),
                "time_s": float(telemetry.time_s[i]),
                "reason": "FIRST_LEGAL_PLANTAR_RECONTACT_AFTER_FLIGHT",
            }
            break
    events["res85_claim_end"] = claim_end

    mtp = telemetry
    events["mtp_energy"] = {
        "left_active_positive_work_j": float(mtp.mtp_active_work_j[-1, 0]) if len(mtp.index) else 0.0,
        "right_active_positive_work_j": float(mtp.mtp_active_work_j[-1, 1]) if len(mtp.index) else 0.0,
        "left_total_positive_work_j": float(mtp.mtp_total_work_j[-1, 0]) if len(mtp.index) else 0.0,
        "right_total_positive_work_j": float(mtp.mtp_total_work_j[-1, 1]) if len(mtp.index) else 0.0,
        "left_passive_positive_work_j": float(mtp.mtp_passive_work_j[-1, 0]) if len(mtp.index) else 0.0,
        "right_passive_positive_work_j": float(mtp.mtp_passive_work_j[-1, 1]) if len(mtp.index) else 0.0,
        "max_active_positive_work_j": (float(max(mtp.mtp_active_work_j[-1]))
                                       if len(mtp.index) else 0.0),
        "left_active_work_signed_j": (float(mtp.mtp_active_work_signed_j[-1, 0])
                                      if len(mtp.index) else 0.0),
        "left_passive_work_signed_j": (float(mtp.mtp_passive_work_signed_j[-1, 0])
                                       if len(mtp.index) else 0.0),
        "left_total_work_signed_j": (float(mtp.mtp_total_work_signed_j[-1, 0])
                                     if len(mtp.index) else 0.0),
        "ledger_identity_left_residual_j": (
            float(mtp.mtp_total_work_signed_j[-1, 0]
                  - mtp.mtp_active_work_signed_j[-1, 0]
                  - mtp.mtp_passive_work_signed_j[-1, 0]) if len(mtp.index) else 0.0),
        "ledger_identity_right_residual_j": (
            float(mtp.mtp_total_work_signed_j[-1, 1]
                  - mtp.mtp_active_work_signed_j[-1, 1]
                  - mtp.mtp_passive_work_signed_j[-1, 1]) if len(mtp.index) else 0.0),
    }
    # AEI-1f takeoff dominance: ankle positive work must exceed active MTP work
    ankle_idx = [CHANNELS.index("left_ankle"), CHANNELS.index("right_ankle")]
    ankle_positive_work = 0.0
    if len(mtp.index):
        ankle_power = mtp.joint_power_w[:, ankle_idx]
        ankle_positive_work = float(np.sum(np.maximum(ankle_power, 0.0)) * M.NATIVE_DT_S)
    events["takeoff_dominance"] = {
        "ankle_positive_work_j": ankle_positive_work,
        "active_mtp_positive_work_j": events["mtp_energy"]["max_active_positive_work_j"],
        "ankle_exceeds_active_mtp": ankle_positive_work >
        events["mtp_energy"]["max_active_positive_work_j"],
    }
    return events


__all__ = [
    "DEFAULT_HORIZON_S",
    "PHASE_INDEX",
    "V3LaunchEpisode",
    "V3LaunchTelemetry",
    "V3RuntimeError",
    "V3_LAUNCH_RUNTIME_AUTHORITY_ID",
    "extract_events",
    "run_launch_episode",
    "settle_standing_stance",
]
