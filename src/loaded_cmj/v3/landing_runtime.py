"""V3 RES-86 landing runtime.

Authority: ``LCMJ_RES86_LANDING_CONTROL_V1``.

The runtime preserves the qualified RES-85 launch/flight controller and its
actuation-authority history through physical first contact (E8), then hands the
**same** plant state and the **same** actuation history to the RES-86 landing
controller.  No state is reset, no torque history is zeroed and no MTP ledger
is reinitialised at touchdown.

The launch phase is byte-identical to :func:`loaded_cmj.v3.launch_runtime.run_launch_episode`
for every sample before E8; the landing phase then records every derived
quantity required by the independent event authority
(:mod:`loaded_cmj.v3.landing_events`).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3 import measurement as M
from loaded_cmj.v3.actuation import (
    MOMENT_CEILING_NM,
    POWER_CEILING_W,
    V3ActuationAuthority,
    V3ActuationState,
)
from loaded_cmj.v3.active_set_capture import ActiveSetRecorder, V3ActiveSetTables
from loaded_cmj.v3.contact_realization import (
    RES86_CANDIDATE_DECLARED_SOLREF,
    RES86_CANDIDATE_REALIZED_SOLREF,
    RES86_CONTACT_CANDIDATE_LABEL,
    RES86_NOMINAL_REALIZED_SOLREF,
    build_contact_realization_plant,
    effective_floor_contacts,
    verify_effective_contact_realization,
)
from loaded_cmj.v3.controller import (
    V3ControllerConfig,
    V3ControllerFault,
    V3LaunchController,
)
from loaded_cmj.v3.landing_control import (
    CONTROL_COORDINATES,
    V3LandingConfig,
    V3LandingController,
    V3LandingFault,
    V3LandingPhase,
    V3LandingStep,
)
from loaded_cmj.v3.landing_events import (
    V3LandingTrace,
    evaluate_landing_trace,
)
from loaded_cmj.v3.landing_metrics import centroidal_hy_kg_m2_s
from loaded_cmj.v3.launch_runtime import settle_standing_stance
from loaded_cmj.v3.plant import V3Plant

V3_LANDING_RUNTIME_AUTHORITY_ID = "LCMJ_RES86_LANDING_RUNTIME_V1"

PRE_TOUCHDOWN_SAMPLE = 790
PRE_TOUCHDOWN_TIME_S = 1.580
E8_SAMPLE = 791
E8_TIME_S = 1.582
E8_STATE_SHA256 = "e411462929c178b97a1132babb46f2fde8c7e12c5d40e40e8f18fa7ce7869355"
PRE_TOUCHDOWN_STATE_SHA256 = (
    "08605746ced78fa130c6fc210661fdd611e22e0bbbe20b9758f85b060d93cb42")

# Post-apex upstream handoff: the first native sample of the launch's
# LANDING_PREP phase (the launch phase machine enters it just after the apex at
# sample 702).  From here the RES-86 landing controller owns the touchdown
# preparation, preserving takeoff occurrence/confirmation, the apex, the
# genuine flight and zero contact before the physical touchdown.
POST_APEX_SAMPLE = 705
POST_APEX_TIME_S = 1.410
POST_APEX_STATE_SHA256 = (
    "31ddc10f883b785e5c240ba920c4d4fcf738dac74f4ae938464b92f6823cb220")
APEX_SAMPLE = 702

DEFAULT_HORIZON_S = 3.5


class V3LandingRuntimeError(RuntimeError):
    """Explicit runtime failure (fail closed)."""


@dataclass(frozen=True)
class V3HandoffCertificate:
    """Exact restorable RES-85 -> RES-86 handoff point."""

    sample_index: int
    time_s: float
    state_vector: np.ndarray
    ctrl_nm: np.ndarray
    qacc_warmstart: np.ndarray
    actuation: V3ActuationState
    state_sha256: str


def capture_handoff_certificate(plant: V3Plant, data: mujoco.MjData, *,
                                sample_index: int, time_s: float,
                                authority: V3ActuationAuthority) -> V3HandoffCertificate:
    size = int(mujoco.mj_stateSize(plant.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    vector = np.zeros(size, dtype=np.float64)
    mujoco.mj_getState(plant.model, data, vector, mujoco.mjtState.mjSTATE_INTEGRATION)
    return V3HandoffCertificate(
        sample_index=int(sample_index),
        time_s=float(time_s),
        state_vector=vector,
        ctrl_nm=np.array(data.ctrl, dtype=np.float64, copy=True),
        qacc_warmstart=np.array(data.qacc_warmstart, dtype=np.float64, copy=True),
        actuation=authority.snapshot_state(),
        state_sha256=hashlib.sha256(vector.tobytes()).hexdigest(),
    )


@dataclass
class V3LandingTelemetry:
    index: np.ndarray
    time_s: np.ndarray
    phase_name: np.ndarray
    transition_reason: np.ndarray
    control_update: np.ndarray
    fallback: np.ndarray
    fallback_reason: np.ndarray
    failed_gate: np.ndarray
    trust_region_level: np.ndarray
    linear_prediction_residual: np.ndarray
    proposed_desired_nm: np.ndarray
    validated_desired_nm: np.ndarray
    applied_nm: np.ndarray
    derivative_central: np.ndarray
    derivative_one_sided: np.ndarray
    derivative_unavailable: np.ndarray
    live_branch_identity: np.ndarray
    validated_branch_hash: np.ndarray
    qpos: np.ndarray
    qvel: np.ndarray
    com_world_m: np.ndarray
    com_velocity_world_m_s: np.ndarray
    left_fz_n: np.ndarray
    right_fz_n: np.ndarray
    total_floor_fz_n: np.ndarray
    legal_plantar_active: np.ndarray
    left_legal_plantar_active: np.ndarray
    right_legal_plantar_active: np.ndarray
    prohibited_detected: np.ndarray
    prohibited_active: np.ndarray
    hy_kg_m2_s: np.ndarray
    root_pitch_rad: np.ndarray
    trunk_pitch_rad: np.ndarray
    root_pitch_rate_rad_s: np.ndarray
    trunk_pitch_rate_rad_s: np.ndarray
    max_penetration_m: np.ndarray
    rom_margin_min_rad: np.ndarray
    max_abs_moment_ratio: np.ndarray
    max_abs_power_ratio: np.ndarray
    mtp_active_applied_nm: np.ndarray
    mode_signature_nominal: np.ndarray

    def arrays(self) -> dict[str, np.ndarray]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    def canonical_digest(self) -> str:
        digest = hashlib.sha256()
        for name in sorted(self.__dataclass_fields__):
            array = np.asarray(getattr(self, name))
            digest.update(name.encode("utf-8"))
            if array.dtype == object:
                digest.update("\x1f".join(str(x) for x in array.ravel()).encode("utf-8"))
            else:
                digest.update(np.ascontiguousarray(array).tobytes())
        return digest.hexdigest()


@dataclass
class V3LandingEpisode:
    status: str
    handoff: V3HandoffCertificate
    steps: list[V3LandingStep]
    telemetry: V3LandingTelemetry
    trace: V3LandingTrace
    events: dict[str, Any]
    launch_events: dict[str, Any]
    active_set_tables: V3ActiveSetTables | None = None
    fault: str | None = None
    warnings: list[str] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)


def _empty_telemetry_lists() -> dict[str, list]:
    return {
        "index": [], "time_s": [], "phase_name": [], "transition_reason": [],
        "control_update": [], "fallback": [], "fallback_reason": [], "failed_gate": [],
        "trust_region_level": [], "linear_prediction_residual": [],
        "proposed_desired_nm": [], "validated_desired_nm": [], "applied_nm": [],
        "derivative_central": [], "derivative_one_sided": [], "derivative_unavailable": [],
        "live_branch_identity": [], "validated_branch_hash": [],
        "qpos": [], "qvel": [], "com_world_m": [], "com_velocity_world_m_s": [],
        "left_fz_n": [], "right_fz_n": [], "total_floor_fz_n": [],
        "legal_plantar_active": [], "left_legal_plantar_active": [],
        "right_legal_plantar_active": [], "prohibited_detected": [], "prohibited_active": [],
        "hy_kg_m2_s": [], "root_pitch_rad": [], "trunk_pitch_rad": [],
        "root_pitch_rate_rad_s": [], "trunk_pitch_rate_rad_s": [],
        "max_penetration_m": [], "rom_margin_min_rad": [],
        "max_abs_moment_ratio": [], "max_abs_power_ratio": [],
        "mtp_active_applied_nm": [], "mode_signature_nominal": [],
    }


def _stack(name: str, values: list) -> np.ndarray:
    if not values:
        if name in ("phase_name", "transition_reason", "fallback_reason", "failed_gate",
                    "validated_branch_hash", "mode_signature_nominal"):
            return np.asarray([], dtype=object)
        return np.zeros((0,), dtype=np.float64)
    if name in ("phase_name", "transition_reason", "fallback_reason", "failed_gate",
                "validated_branch_hash", "mode_signature_nominal"):
        return np.asarray(values, dtype=object)
    first = np.asarray(values[0])
    if first.ndim == 0:
        if name in ("control_update", "fallback"):
            return np.asarray(values, dtype=np.bool_)
        if name == "live_branch_identity":
            return np.asarray(values, dtype=object)
        return np.asarray(values, dtype=np.float64)
    return np.asarray(values, dtype=np.float64)


def _per_foot_legal_active(records) -> tuple[int, int]:
    left = sum(1 for r in records if r.active_legal_plantar and r.side == "left")
    right = sum(1 for r in records if r.active_legal_plantar and r.side == "right")
    return left, right


def _max_penetration(records) -> float:
    return max((float(r.penetration_m) for r in records), default=0.0)


def _rom_margin_min(data: mujoco.MjData, plant: V3Plant) -> float:
    margin = float("inf")
    for name in C.V3_JOINT_NAMES:
        rng = C.V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        q = float(data.qpos[plant.idx.qadr[name]])
        margin = min(margin, q - float(rng[0]), float(rng[1]) - q)
    return margin


def _integration_state_sha256(plant: V3Plant, data: mujoco.MjData) -> str:
    size = int(mujoco.mj_stateSize(plant.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    vector = np.zeros(size, dtype=np.float64)
    mujoco.mj_getState(plant.model, data, vector, mujoco.mjtState.mjSTATE_INTEGRATION)
    return hashlib.sha256(vector.tobytes()).hexdigest()


def _run_launch_to_post_apex(plant: V3Plant, data: mujoco.MjData, *,
                             n_samples: int, config: V3ControllerConfig,
                             capture_active_set: bool = False
                             ) -> tuple[V3LaunchController, V3HandoffCertificate, dict[str, Any],
                                        ActiveSetRecorder | None]:
    """Run the sealed launch to the first post-apex LANDING_PREP sample.

    The launch law is byte-identical to the RES-85 canonical episode through
    this sample; the apex (maximum SYSTEM_COM height) is strictly before the
    handoff and the takeoff occurrence/confirmation are preserved.
    """
    settle_standing_stance(plant, data)
    launch = V3LaunchController(plant, data, config=config)
    frames: list[M.V3NativeFrame] = []
    recorder = ActiveSetRecorder(plant) if capture_active_set else None
    dt = float(plant.model.opt.timestep)
    apex_index = -1
    apex_height = -np.inf
    for k in range(n_samples):
        t = k * dt
        frame = M.native_frame(plant, data, k, t)
        snapshot = M.measure(plant, data, flight_context=launch.phase.value in
                             ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"))
        frames.append(frame)
        com = M.system_com_state(plant, data)
        if com.com_world_m[2] > apex_height:
            apex_height = float(com.com_world_m[2])
            apex_index = int(k)
        if recorder is not None and k >= PRE_TOUCHDOWN_SAMPLE:
            recorder.append(data, sample_index=k, time_s=t, records=snapshot.records)
        if launch.phase.value == "LANDING_PREP" and k >= POST_APEX_SAMPLE:
            handoff = capture_handoff_certificate(
                plant, data, sample_index=k, time_s=t, authority=launch.actuation)
            events = {
                "post_apex_sample": int(k),
                "apex_sample": int(apex_index),
                "apex_com_height_m": float(apex_height),
                "takeoff_occurrence_sample": (
                    None if launch.accepted_occurrence is None
                    else int(launch.accepted_occurrence.native_index)),
                "takeoff_confirmation_sample": (
                    None if launch.confirmation is None
                    else int(launch.confirmation.confirmation_sample)),
                "post_apex_state_sha256": handoff.state_sha256,
            }
            return launch, handoff, events, recorder
        step = launch.update(frame, snapshot, frames, plant, data)
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)
    raise V3LandingRuntimeError("no post-apex LANDING_PREP sample within the horizon")


def _run_launch_to_handoff(plant: V3Plant, data: mujoco.MjData, *,
                           n_samples: int, config: V3ControllerConfig,
                           capture_active_set: bool = False
                           ) -> tuple[V3LaunchController, V3HandoffCertificate, dict[str, Any], ActiveSetRecorder | None]:
    settle_standing_stance(plant, data)
    launch = V3LaunchController(plant, data, config=config)
    frames: list[M.V3NativeFrame] = []
    recorder = ActiveSetRecorder(plant) if capture_active_set else None
    pre_touchdown_certificate: dict[str, Any] | None = None
    dt = float(plant.model.opt.timestep)
    handoff: V3HandoffCertificate | None = None
    for k in range(n_samples):
        t = k * dt
        frame = M.native_frame(plant, data, k, t)
        snapshot = M.measure(plant, data, flight_context=launch.phase.value in
                             ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"))
        frames.append(frame)
        if recorder is not None and k >= PRE_TOUCHDOWN_SAMPLE:
            recorder.append(data, sample_index=k, time_s=t, records=snapshot.records)
        if k == PRE_TOUCHDOWN_SAMPLE:
            pre_touchdown_certificate = {
                "sample": k,
                "time_s": t,
                "state_sha256": _integration_state_sha256(plant, data),
            }
        if (frame.legal_plantar_active > 0 and launch.accepted_occurrence is not None
                and k > int(launch.accepted_occurrence.native_index)):
            handoff = capture_handoff_certificate(
                plant, data, sample_index=k, time_s=t, authority=launch.actuation)
            events = {
                "takeoff_occurrence_sample": int(launch.accepted_occurrence.native_index),
                "takeoff_confirmation_sample": (
                    None if launch.confirmation is None else launch.confirmation.confirmation_sample),
                "pre_touchdown_state_sha256_at_branch": (
                    None if pre_touchdown_certificate is None
                    else pre_touchdown_certificate["state_sha256"]),
            }
            return launch, handoff, events, recorder
        step = launch.update(frame, snapshot, frames, plant, data)
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)
    raise V3LandingRuntimeError("no post-flight legal contact within the horizon")


def _restore_handoff(plant: V3Plant, data: mujoco.MjData,
                     certificate: V3HandoffCertificate) -> V3ActuationAuthority:
    mujoco.mj_setState(plant.model, data, certificate.state_vector,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    data.ctrl[:] = certificate.ctrl_nm
    data.qacc_warmstart[:] = certificate.qacc_warmstart
    mujoco.mj_forward(plant.model, data)
    authority = V3ActuationAuthority(float(plant.model.opt.timestep))
    authority.restore_state(certificate.actuation)
    return authority


def build_handoff_certificate(*, controller_config: V3ControllerConfig | None = None,
                              plant: V3Plant | None = None,
                              ) -> tuple[V3HandoffCertificate, dict[str, Any]]:
    """Run the canonical RES-85 launch to E8 and return the exact handoff."""
    plant = plant if plant is not None else V3Plant()
    data = plant.make_data()
    dt = float(plant.model.opt.timestep)
    _, handoff, events, _ = _run_launch_to_handoff(
        plant, data, n_samples=int(round(DEFAULT_HORIZON_S / dt)),
        config=controller_config or V3ControllerConfig(dt_s=dt),
        capture_active_set=False)
    return handoff, events


def run_landing_episode(*, horizon_s: float = DEFAULT_HORIZON_S,
                        controller_config: V3ControllerConfig | None = None,
                        landing_config: V3LandingConfig | None = None,
                        landing_actuation: V3ActuationAuthority | None = None,
                        handoff_certificate: V3HandoffCertificate | None = None,
                        plant: V3Plant | None = None,
                        capture_active_set: bool = False,
                        stop_after_e10_s: float | None = None,
                        upstream_prep: bool = True,
                        contact_realization: str = "candidate",
                        ) -> V3LandingEpisode:
    """Run the canonical launch and the RES-86 landing controller.

    With ``upstream_prep`` (default) the launch is reproduced byte-identically
    to the first post-apex LANDING_PREP sample and the landing controller then
    owns the touchdown preparation and the whole landing.  With
    ``upstream_prep=False`` the legacy handoff at the E8 first-contact sample is
    preserved.

    ``contact_realization`` selects the qualified RES-86 plantar contact
    realization: ``"candidate"`` (the RES-86 solution-verification candidate,
    realized plantar ``solref`` (0.015, 1.0)) or ``"nominal"`` (the RES-84
    provisional baseline, realized (0.02, 1.0)).  The Plant topology, ``condim``,
    ``solimp`` and friction are unchanged in both cases.
    """
    if contact_realization not in ("candidate", "nominal"):
        raise V3LandingRuntimeError(
            f"unknown contact realization {contact_realization!r}")
    # The sealed RES-85 launch owns the standing settle and the flight phase;
    # the RES-86 contact realization is a landing-phase realization only (it
    # cannot act in flight).  Unless the caller supplies an explicit Plant, the
    # launch is reproduced on the sealed nominal Plant and its exact integration
    # state is transferred to the realization Plant at the flight handoff, so
    # the sealed launch identity is preserved for every landing realization.
    if plant is None:
        declared = (RES86_CANDIDATE_DECLARED_SOLREF
                    if contact_realization == "candidate" else None)
        landing_plant = build_contact_realization_plant(declared)
        launch_plant = V3Plant()
    else:
        landing_plant = launch_plant = plant
    dt = float(landing_plant.model.opt.timestep)
    if abs(dt - M.NATIVE_DT_S) > 1e-12:
        raise V3LandingRuntimeError(f"native dt {dt!r} != {M.NATIVE_DT_S!r}")
    n_samples = int(round(horizon_s / dt))
    warnings: list[str] = []
    launch_events: dict[str, Any] = {}
    recorder: ActiveSetRecorder | None = None
    handoff: V3HandoffCertificate

    if handoff_certificate is None:
        launch_data = launch_plant.make_data()
        if upstream_prep:
            _launch, handoff, launch_events, recorder = _run_launch_to_post_apex(
                launch_plant, launch_data, n_samples=n_samples,
                config=controller_config or V3ControllerConfig(dt_s=dt),
                capture_active_set=capture_active_set)
        else:
            _launch, handoff, launch_events, recorder = _run_launch_to_handoff(
                launch_plant, launch_data, n_samples=n_samples,
                config=controller_config or V3ControllerConfig(dt_s=dt),
                capture_active_set=capture_active_set)
    else:
        handoff = handoff_certificate
    plant = landing_plant
    data = plant.make_data()
    authority = _restore_handoff(plant, data, handoff)
    if landing_actuation is not None:
        landing_actuation.restore_state(handoff.actuation)
        authority = landing_actuation
    if upstream_prep and handoff.sample_index == POST_APEX_SAMPLE:
        if handoff.state_sha256 != POST_APEX_STATE_SHA256:
            warnings.append(f"POST_APEX_STATE_SHA256_DIFFERS:{handoff.state_sha256}")
    elif handoff.state_sha256 != E8_STATE_SHA256:
        warnings.append(f"E8_STATE_SHA256_DIFFERS:{handoff.state_sha256}")

    start_phase = (V3LandingPhase.PRE_TOUCHDOWN
                   if handoff.sample_index == POST_APEX_SAMPLE else None)
    landing = V3LandingController(plant, data,
                                  config=landing_config or V3LandingConfig(),
                                  actuation=authority, start_phase=start_phase)
    log = _empty_telemetry_lists()
    steps: list[V3LandingStep] = []
    fault: str | None = None
    status = "COMPLETED"
    stop_after = (landing.config.stop_after_e10_s if stop_after_e10_s is None
                  else float(stop_after_e10_s))
    e10_time: float | None = None

    for k in range(int(handoff.sample_index), n_samples):
        t = k * dt
        frame = M.native_frame(plant, data, k, t)
        snapshot = M.measure(
            plant, data,
            flight_context=landing.phase == V3LandingPhase.PRE_TOUCHDOWN)
        try:
            step = landing.update(frame, snapshot, [], plant, data)
        except V3LandingFault as exc:
            fault = f"LANDING_FAULT:{exc}"
            status = "FAIL_CLOSED"
            break
        steps.append(step)
        records = snapshot.records
        left_legal, right_legal = _per_foot_legal_active(records)
        hy = centroidal_hy_kg_m2_s(plant, data)
        orientation = snapshot.orientation
        com = snapshot.system_com
        derivative_counts = {"CENTRAL": 0, "PLUS_ONLY": 0, "MINUS_ONLY": 0, "UNAVAILABLE": 0}
        nominal_mode = ""
        for column in step.derivatives:
            derivative_counts[column.classification] = derivative_counts.get(
                column.classification, 0) + 1
            nominal_mode = column.nominal_mode_hash
        moment_ratio = float(np.max(np.abs(np.asarray(step.actuation.applied_nm)) /
                                    MOMENT_CEILING_NM))
        power_ratio = float(np.max(np.abs(np.asarray(step.actuation.joint_power_w)) /
                                   POWER_CEILING_W))
        log["index"].append(int(frame.index))
        log["time_s"].append(float(frame.time_s))
        log["phase_name"].append(step.phase)
        log["transition_reason"].append(step.transition_reason)
        log["control_update"].append(bool(step.control_update))
        log["fallback"].append(bool(step.fallback))
        log["fallback_reason"].append(step.fallback_reason)
        log["failed_gate"].append(step.failed_gate)
        log["trust_region_level"].append(int(step.trust_region_level))
        log["linear_prediction_residual"].append(float(step.linear_prediction_max_residual))
        log["proposed_desired_nm"].append(np.asarray(step.proposed_desired_nm))
        log["validated_desired_nm"].append(np.asarray(step.validated_desired_nm))
        log["applied_nm"].append(np.asarray(step.applied_nm))
        log["derivative_central"].append(int(derivative_counts["CENTRAL"]))
        log["derivative_one_sided"].append(
            int(derivative_counts["PLUS_ONLY"] + derivative_counts["MINUS_ONLY"]))
        log["derivative_unavailable"].append(int(derivative_counts["UNAVAILABLE"]))
        log["live_branch_identity"].append(
            None if step.live_branch_identity is None else bool(step.live_branch_identity))
        log["validated_branch_hash"].append(step.validated_branch_state_sha256)
        log["qpos"].append(np.array(data.qpos, dtype=np.float64, copy=True))
        log["qvel"].append(np.array(data.qvel, dtype=np.float64, copy=True))
        log["com_world_m"].append(np.asarray(com.com_world_m, dtype=np.float64))
        log["com_velocity_world_m_s"].append(
            np.asarray(com.com_velocity_world_m_s, dtype=np.float64))
        log["left_fz_n"].append(float(frame.left_foot_force_world_n[2]))
        log["right_fz_n"].append(float(frame.right_foot_force_world_n[2]))
        log["total_floor_fz_n"].append(float(frame.total_floor_force_world_n[2]))
        log["legal_plantar_active"].append(int(frame.legal_plantar_active))
        log["left_legal_plantar_active"].append(int(left_legal))
        log["right_legal_plantar_active"].append(int(right_legal))
        log["prohibited_detected"].append(int(frame.prohibited_detected))
        log["prohibited_active"].append(int(frame.prohibited_active))
        log["hy_kg_m2_s"].append(float(hy))
        log["root_pitch_rad"].append(float(orientation.root_pitch_rad))
        log["trunk_pitch_rad"].append(float(orientation.trunk_absolute_pitch_rad))
        log["root_pitch_rate_rad_s"].append(float(orientation.root_pitch_rate_rad_s))
        log["trunk_pitch_rate_rad_s"].append(float(orientation.trunk_absolute_pitch_rate_rad_s))
        log["max_penetration_m"].append(float(_max_penetration(records)))
        log["rom_margin_min_rad"].append(float(_rom_margin_min(data, plant)))
        log["max_abs_moment_ratio"].append(moment_ratio)
        log["max_abs_power_ratio"].append(power_ratio)
        log["mtp_active_applied_nm"].append(np.asarray(step.actuation.mtp_active_applied_nm))
        log["mode_signature_nominal"].append(nominal_mode)
        if recorder is not None and k >= PRE_TOUCHDOWN_SAMPLE:
            recorder.append(data, sample_index=k, time_s=t, records=records)

        if landing.e10_confirmed_time_s is not None and e10_time is None:
            e10_time = landing.e10_confirmed_time_s
        if e10_time is not None and t >= e10_time + stop_after:
            break

        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)

    if status == "COMPLETED" and landing.e10_confirmed_time_s is None:
        warnings.append("E10_NOT_CONFIRMED_WITHIN_HORIZON")
        status = "NO_E10_WITHIN_HORIZON"

    telemetry = V3LandingTelemetry(**{name: _stack(name, values)
                                      for name, values in log.items()})
    legal_series = np.asarray(telemetry.legal_plantar_active, dtype=np.int64)
    if legal_series.size and bool(np.any(legal_series > 0)):
        first_contact_position = int(np.argmax(legal_series > 0))
    else:
        first_contact_position = -1
    trace = V3LandingTrace(
        index=np.asarray(telemetry.index, dtype=np.int64),
        time_s=np.asarray(telemetry.time_s, dtype=np.float64),
        legal_plantar_active=np.asarray(telemetry.legal_plantar_active, dtype=np.int64),
        left_legal_plantar_active=np.asarray(telemetry.left_legal_plantar_active, dtype=np.int64),
        right_legal_plantar_active=np.asarray(telemetry.right_legal_plantar_active, dtype=np.int64),
        left_fz_n=np.asarray(telemetry.left_fz_n, dtype=np.float64),
        right_fz_n=np.asarray(telemetry.right_fz_n, dtype=np.float64),
        prohibited_detected=np.asarray(telemetry.prohibited_detected, dtype=np.int64),
        prohibited_active=np.asarray(telemetry.prohibited_active, dtype=np.int64),
        com_world_m=np.asarray(telemetry.com_world_m, dtype=np.float64),
        com_velocity_world_m_s=np.asarray(telemetry.com_velocity_world_m_s, dtype=np.float64),
        hy_kg_m2_s=np.asarray(telemetry.hy_kg_m2_s, dtype=np.float64),
        root_pitch_rad=np.asarray(telemetry.root_pitch_rad, dtype=np.float64),
        trunk_pitch_rad=np.asarray(telemetry.trunk_pitch_rad, dtype=np.float64),
        root_pitch_rate_rad_s=np.asarray(telemetry.root_pitch_rate_rad_s, dtype=np.float64),
        trunk_pitch_rate_rad_s=np.asarray(telemetry.trunk_pitch_rate_rad_s, dtype=np.float64),
        total_floor_fz_n=np.asarray(telemetry.total_floor_fz_n, dtype=np.float64),
        max_penetration_m=np.asarray(telemetry.max_penetration_m, dtype=np.float64),
        rom_margin_min_rad=np.asarray(telemetry.rom_margin_min_rad, dtype=np.float64),
        max_abs_moment_ratio=np.asarray(telemetry.max_abs_moment_ratio, dtype=np.float64),
        max_abs_power_ratio=np.asarray(telemetry.max_abs_power_ratio, dtype=np.float64),
        first_contact_position=first_contact_position,
    )
    if trace.length == 0:
        events = {"status": "FAIL", "reason": fault or "EMPTY_LANDING_TRACE"}
        tables = None
    elif first_contact_position < 0:
        events = {"status": "FAIL", "reason": fault or "NO_FIRST_LEGAL_CONTACT"}
        tables = recorder.finalize() if recorder is not None else None
    else:
        events = evaluate_landing_trace(trace)
        events["controller_phase_final"] = landing.phase.value
        tables = recorder.finalize() if recorder is not None else None
    expected_realized = (RES86_CANDIDATE_REALIZED_SOLREF
                         if contact_realization == "candidate"
                         else RES86_NOMINAL_REALIZED_SOLREF)
    realization_verification = verify_effective_contact_realization(
        plant, data, require_plantar=False, expected_solref=expected_realized)
    diagnostics = {
        "contact_realization": contact_realization,
        "contact_realization_label": (RES86_CONTACT_CANDIDATE_LABEL
                                       if contact_realization == "candidate"
                                       else "RES84_PROVISIONAL_NOMINAL"),
        "contact_realization_declared_geom_solref": [
            float(v) for v in (RES86_CANDIDATE_DECLARED_SOLREF
                               if contact_realization == "candidate"
                               else RES86_NOMINAL_REALIZED_SOLREF)],
        "contact_realization_expected_realized_solref": [float(v)
                                                         for v in expected_realized],
        "contact_realization_verification": realization_verification,
        "contact_realization_final_rows": [
            {
                "geom": row.other_geom, "side": row.side, "region": row.region,
                "dim": row.dim, "solref": list(row.solref), "solimp": list(row.solimp),
                "friction": list(row.friction), "includemargin_m": row.includemargin_m,
            }
            for row in effective_floor_contacts(plant, data) if row.is_plantar_floor],
        "handoff_sample": int(handoff.sample_index),
        "handoff_time_s": float(handoff.time_s),
        "handoff_state_sha256": handoff.state_sha256,
        "handoff_kind": ("POST_APEX_LANDING_PREP" if handoff.sample_index == POST_APEX_SAMPLE
                         else "E8_FIRST_CONTACT"),
        "e8_state_sha256_expected": E8_STATE_SHA256,
        "post_apex_state_sha256_expected": POST_APEX_STATE_SHA256,
        "pre_touchdown_state_sha256_expected": PRE_TOUCHDOWN_STATE_SHA256,
        "landing_samples": int(telemetry.index.shape[0]),
        "e10_confirmed_time_s": landing.e10_confirmed_time_s,
        "e10_window_violated_time_s": landing.e10_window_violated_time_s,
        "e10_window_violation_reason": landing.e10_window_violation_reason,
        "fallback_count": int(np.count_nonzero(telemetry.fallback)),
        "trust_region_shrinks": int(np.count_nonzero(telemetry.trust_region_level > 0)),
        "live_branch_identity_failures": int(sum(
            1 for value in telemetry.live_branch_identity if value is False)),
        "derivative_central_total": int(np.sum(telemetry.derivative_central)),
        "derivative_one_sided_total": int(np.sum(telemetry.derivative_one_sided)),
        "derivative_unavailable_total": int(np.sum(telemetry.derivative_unavailable)),
        "status": status,
    }
    return V3LandingEpisode(
        status=status,
        handoff=handoff,
        steps=steps,
        telemetry=telemetry,
        trace=trace,
        events=events,
        launch_events=launch_events,
        active_set_tables=tables,
        fault=fault,
        warnings=warnings,
        diagnostics=diagnostics,
    )


__all__ = [
    "DEFAULT_HORIZON_S",
    "E8_SAMPLE",
    "E8_STATE_SHA256",
    "PRE_TOUCHDOWN_SAMPLE",
    "PRE_TOUCHDOWN_STATE_SHA256",
    "V3HandoffCertificate",
    "V3LandingEpisode",
    "V3LandingRuntimeError",
    "V3LandingTelemetry",
    "build_handoff_certificate",
    "capture_handoff_certificate",
    "run_landing_episode",
]
