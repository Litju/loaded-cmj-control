"""V3 active-set-safe whole-body landing capture controller (RES-86B).

Authority: ``LCMJ_RES86_LANDING_CONTROL_V1``.

This module owns the downstream RES-86 landing controller that inherits the
qualified RES-85 launch/flight trajectory at physical first contact and drives
it to a valid whole-body E10 landing capture.  It is deliberately separate from
the RES-85 launch law:

* the RES-85 controller and its actuation authority remain unchanged through
  touchdown; the landing controller receives the **same** actuation-authority
  instance (no history reset, no zeroed previous torque, no new MTP ledger);
* the landing decision cell is the native physics interval
  ``CONTROL_INTERVAL_S = 0.010 s`` (5 native steps at dt = 0.002 s), and every
  prediction/validation interval is exactly the interval that executes;
* local derivatives are built from **real MuJoCo branch perturbations** of the
  exact restored ``mjSTATE_INTEGRATION`` state and the exact actuation history
  through the RES-86A lossless contact/EFC recorder.  A central difference is
  never taken across incompatible contact modes: the frozen
  ``RES86_CONTACT_MODE_SIGNATURE_V1`` decides CENTRAL / one-sided / unavailable;
* every proposed action is replayed through exact MuJoCo over exactly the
  interval that will execute; validation of every native sample rejects
  prohibited contact, penetration beyond 0.010 m, structural-ROM violation,
  hard moment/power/MTP-authority violation, non-finite state and unpermitted
  support loss;
* the optimizer proposes, the exact branch decides.  When no candidate
  survives, a declared fallback (hold the previous applied torque) is itself
  exactly validated; if the fallback cannot pass hard safety the controller
  fails closed.

The controller never writes Plant state or applied forces: the only actuation
path is :class:`loaded_cmj.v3.actuation.V3ActuationAuthority`.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3 import measurement as M
from loaded_cmj.v3.active_set_capture import (
    DERIVATIVE_CENTRAL_ALLOWED,
    DERIVATIVE_ONE_SIDED_SAME_MODE,
    DERIVATIVE_UNAVAILABLE,
    ActiveSetRecorder,
    derivative_eligibility,
)
from loaded_cmj.v3.actuation import (
    CHANNELS,
    MOMENT_CEILING_NM,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    MTP_LATE_ACTIVE_FRACTION,
    MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
    N_CHANNELS,
    POWER_CEILING_W,
    RATE_CEILING_NM_PER_S,
    V3ActuationAuthority,
    V3ActuationState,
    V3AppliedTorque,
    plant_mtp_passive_moment_nm,
)
from loaded_cmj.v3.controller import (
    JOINT_ROM_BARRIER_GAIN,
    JOINT_ROM_BRAKE_ZONE_RAD,
    JOINT_ROM_MARGIN_RAD,
    JOINT_ROM_VELOCITY_GAIN,
    ROM_BRAKE_HORIZON_S,
    STRUCTURAL_FLEXION_SAFETY_RAD,
    TRUNK_ROM_GUARD_MARGIN_RAD,
    TRUNK_ROM_POSITION_GAIN,
    TRUNK_ROM_VELOCITY_GAIN,
)
from loaded_cmj.v3.landing_authority import (
    D_BL_S,
    V3_STRUCTURAL_ROM_TOLERANCE_RAD,
    dwell_confirmed,
)
from loaded_cmj.v3.landing_metrics import centroidal_hy_kg_m2_s
from loaded_cmj.v3.plant import V3Plant

V3_LANDING_CONTROL_AUTHORITY_ID = "LCMJ_RES86_LANDING_CONTROL_V1"

# ---------------------------------------------------------------------------
# frozen control-cell geometry and task thresholds
# ---------------------------------------------------------------------------
NATIVE_DT_S = M.NATIVE_DT_S
CONTROL_INTERVAL_S = 0.010
CONTROL_INTERVAL_NATIVE_STEPS = int(round(CONTROL_INTERVAL_S / NATIVE_DT_S))
assert CONTROL_INTERVAL_NATIVE_STEPS * NATIVE_DT_S == CONTROL_INTERVAL_S

F_THR_N = 10.0
V_ABS_TAIL_M_S = 0.05
PENETRATION_LIMIT_M = 0.010
PENETRATION_SOLVE_MARGIN_M = 0.0
ROM_SOLVE_MARGIN_RAD = 0.0
SUPPORT_FREE_RUN_LIMIT = CONTROL_INTERVAL_NATIVE_STEPS
# Control-margin limit on the *cumulative* consecutive support-free samples.
# The qualification gate is the PHYSICAL-time material-reflight definition
# (a legal-support-free interval of at least D_BL_S = 0.050 s); the controller
# plans against a stricter declared physical-time budget so a chain of short
# interval-local losses can never accumulate into a material reflight.
SUPPORT_FREE_RUN_CONTROL_LIMIT = 16
SUPPORT_FREE_RUN_CONTROL_LIMIT_S = SUPPORT_FREE_RUN_CONTROL_LIMIT * NATIVE_DT_S

# Bounded landing ROM guard: the RES-85 guard structure with a declared
# velocity ceiling and a bounded per-channel contribution, so a fast foot-slap
# rate (several rad/s is physical after a loaded toe strike) can never turn the
# desired moment into a runaway proposal.  The hard structural-ROM surface is
# still enforced exactly by branch validation and the frozen ROM limits.
JOINT_ROM_GUARD_MAX_RATE_RAD_S = 4.0
JOINT_ROM_GUARD_FRACTION = 0.35
DESIRED_MOMENT_FRACTION_CAP = 0.9

# One deterministic predeclared epsilon shrink schedule, relative to the
# frozen moment ceiling of each independent coordinate.
FD_EPS_FRACTIONS: tuple[float, ...] = (0.02, 0.01, 0.005, 0.0025, 0.00125)

# Independent desired-moment coordinates (sagittal bilateral symmetry).
CONTROL_COORDINATES: tuple[str, ...] = ("trunk", "hip_pair", "knee_pair", "ankle_pair", "mtp_pair")
COORDINATE_CHANNELS: dict[str, tuple[int, ...]] = {
    "trunk": (0,),
    "hip_pair": (1, 2),
    "knee_pair": (3, 4),
    "ankle_pair": (5, 6),
    "mtp_pair": (7, 8),
}

_OUTPUT_NAMES: tuple[str, ...] = (
    "com_vx_terminal",
    "com_vz_terminal",
    "hy_terminal",
    "root_pitch_terminal",
    "trunk_pitch_terminal",
    "root_pitch_rate_terminal",
    "trunk_pitch_rate_terminal",
    "min_left_fz",
    "min_right_fz",
    "terminal_left_fz",
    "terminal_right_fz",
    "max_penetration",
    "min_rom_margin",
    "cop_margin_terminal",
    "support_fraction",
)
OUTPUT_INDEX: dict[str, int] = {name: i for i, name in enumerate(_OUTPUT_NAMES)}
N_OUTPUTS = len(_OUTPUT_NAMES)

CONSTRAINT_ROW_NAMES: tuple[str, ...] = (
    "FZ_LEFT_MIN",
    "FZ_RIGHT_MIN",
    "MAX_PENETRATION",
    "STRUCTURAL_ROM",
    "HARD_MOMENT",
    "HARD_POWER",
    "MTP_AUTHORITY",
    "PROHIBITED_CONTACT",
    "SUPPORT_RETENTION",
)

OBJECTIVE_WEIGHTS: dict[str, float] = {
    # The causal stopping-time force reference in the baseline carries the
    # arrest; the local velocity objective keeps the optimizer from trading the
    # arrest away for posture, but it is bounded so a single 10 ms cell can
    # never demand a force the leg cannot deliver without unloading contact.
    "com_vz_terminal": 2.0,
    "com_vx_terminal": 5.0,
    "hy_terminal": 0.5,
    "root_pitch_terminal": 0.5,
    "trunk_pitch_terminal": 0.5,
    "root_pitch_rate_terminal": 1.0,
    "trunk_pitch_rate_terminal": 1.0,
}

# Sagittal flexion family direction of the landing law.  The landing family is
# anchored at the exact handoff pose and keeps the ankle contribution small: the
# knee/hip carry the absorption while the ankle holds its frozen structural ROM
# reserve (a standing-anchored family with a large ankle coefficient would drive
# the ankle reference past the frozen ROM during the absorption stroke).
LANDING_FLEXION_DIRECTION = np.array(
    [0.0, 0.6, 0.6, 1.0, 1.0, 0.2, 0.2, 0.0, 0.0], dtype=np.float64)
_FLEXION_DIRECTION = LANDING_FLEXION_DIRECTION


class V3LandingFault(RuntimeError):
    """Explicit fail-closed landing-controller state."""


class V3LandingPhase(str, Enum):
    PRE_TOUCHDOWN = "PRE_TOUCHDOWN"
    LANDING_WAIT = "LANDING_WAIT"
    IMPACT_ABSORPTION = "IMPACT_ABSORPTION"
    LANDING_CAPTURE = "LANDING_CAPTURE"
    E10_CONFIRMED = "E10_CONFIRMED"


@dataclass
class V3LandingConfig:
    """Declared engineering configuration of the RES-86 landing controller."""

    control_interval_native_steps: int = CONTROL_INTERVAL_NATIVE_STEPS
    t_stop_s: float = 0.12
    t_capture_x_s: float = 0.25
    t_capture_h_s: float = 0.25
    t_attitude_s: float = 0.30
    a_z_max_m_s2: float = 40.0
    fx_fraction_max: float = 0.5
    friction_mu: float = 0.9
    cop_hull_margin_m: float = 0.002
    toe_phase_support_width_m: float = 0.03
    toe_phase_ground_map_scale: float = 0.3
    toe_phase_ankle_ground_scale: float = 0.0
    toe_phase_ankle_posture_scale: float = 1.0
    toe_phase_ankle_rate_brake_nm_per_rad_s: float = 1.0
    toe_phase_ankle_moment_cap_nm: float = 30.0
    toe_phase_contact_fz_ref_n: float = 340.0
    toe_phase_contact_gain_nm_per_n: float = 0.2
    toe_phase_contact_cap_nm: float = 80.0
    fz_reference_ramp_bw_per_interval: float = 0.5
    fz_reference_ramp_s: float = 0.05
    joint_kp: tuple[float, ...] = (120.0, 60.0, 60.0, 80.0, 80.0, 60.0, 60.0, 0.0, 0.0)
    joint_kd: tuple[float, ...] = (20.0, 8.0, 8.0, 10.0, 10.0, 8.0, 8.0, 0.0, 0.0)
    trunk_posture_kp: float = 250.0
    trunk_posture_kd: float = 30.0
    trust_region_coordinates_nm: tuple[float, ...] = (
        0.5 * 220.0, 0.5 * 330.0, 0.5 * 380.0, 0.5 * 260.0, 0.5 * 45.0)
    trust_shrink_factor: float = 0.5
    max_refinements: int = 4
    solver_iterations: int = 600
    solver_regularization: float = 1.0
    penalty_base: float = 1.0e3
    penalty_growth: float = 16.0
    penalty_cap: float = 1.0e8
    linear_prediction_relative_tolerance: float = 0.5
    include_mtp_coordinate: bool = True
    stop_after_e10_s: float = 0.10
    # Post-apex touchdown preparation (upstream extension of the landing law
    # into the flight LANDING_PREP window).  The prep reference is a bounded
    # posture move from the exact handoff pose; it never snaps and never
    # commands contact.
    prep_flexion_target_rad: float = 0.10
    prep_ankle_offset_rad: float = -0.20
    prep_posture_rate_rad_s: float = 2.0
    prep_ankle_rate_rad_s: float = 2.0
    prep_moment_scale: float = 0.9
    # Free-flight posture gains are deliberately a small fraction of the
    # supported-phase gains: in flight a stiff PD against a fixed target
    # excites a discrete limit cycle in the low-inertia distal chain.
    prep_position_gain_scale: float = 0.2
    prep_damping_gain_scale: float = 1.0

    def coordinate_trust(self, coordinate: str) -> float:
        index = CONTROL_COORDINATES.index(coordinate)
        return float(self.trust_region_coordinates_nm[index])

    def coordinate_ceiling(self, coordinate: str) -> float:
        return float(min(MOMENT_CEILING_NM[channel] for channel in COORDINATE_CHANNELS[coordinate]))


# ===========================================================================
# branch state / outcome
# ===========================================================================
@dataclass(frozen=True)
class V3BranchState:
    """Exact restorable branch point (state + history), never live-mutating."""

    sample_index: int
    time_s: float
    state_vector: np.ndarray
    ctrl_nm: np.ndarray
    qacc_warmstart: np.ndarray
    actuation: V3ActuationState


@dataclass(frozen=True)
class V3BranchSampleSummary:
    offset: int
    time_s: float
    com_world_m: tuple[float, float, float]
    com_velocity_world_m_s: tuple[float, float, float]
    left_fz_n: float
    right_fz_n: float
    total_floor_fz_n: float
    legal_plantar_active: int
    prohibited_detected: int
    max_penetration_m: float
    min_rom_margin_rad: float
    finite: bool


@dataclass(frozen=True)
class V3BranchOutcome:
    """Complete exact outcome of one replayed candidate interval."""

    base: V3BranchState
    desired_nm: np.ndarray
    native_steps: int
    samples: tuple[V3BranchSampleSummary, ...]
    outputs: np.ndarray
    mode_signature: str
    terminal_state_vector: np.ndarray
    terminal_actuation: V3ActuationState
    terminal_time_s: float
    finite: bool
    prohibited_contact: bool
    max_penetration_m: float
    min_rom_margin_rad: float
    min_left_fz_n: float
    min_right_fz_n: float
    terminal_left_fz_n: float
    terminal_right_fz_n: float
    max_support_free_run: int
    terminal_support_free_run: int
    terminal_legal_support: int
    support_fraction: float
    max_moment_ratio: float
    max_power_ratio: float
    hard_moment_ok: bool
    hard_power_ok: bool
    mtp_authority_ok: bool
    no_safety_override: bool
    cop_margin_terminal_m: float
    max_mtp_active_work_j: float
    max_mtp_total_work_j: float

    def output(self, name: str) -> float:
        return float(self.outputs[OUTPUT_INDEX[name]])

    @property
    def terminal_state_sha256(self) -> str:
        return hashlib.sha256(
            np.ascontiguousarray(self.terminal_state_vector).tobytes()).hexdigest()


def _bounded_joint_margin(data: mujoco.MjData, plant: V3Plant) -> float:
    margin = float("inf")
    for name in C.V3_JOINT_NAMES:
        rng = C.V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        q = float(data.qpos[plant.idx.qadr[name]])
        margin = min(margin, q - float(rng[0]), float(rng[1]) - q)
    return margin


def _per_foot_legal_support(records: Sequence[M.V3ContactRecord]) -> tuple[bool, bool]:
    left = any(r.active_legal_plantar and r.side == "left" for r in records)
    right = any(r.active_legal_plantar and r.side == "right" for r in records)
    return left, right


def _per_foot_normal_force(records: Sequence[M.V3ContactRecord], side: str) -> float:
    return float(sum(r.normal_force_n for r in records
                     if r.active_legal_plantar and r.side == side))


def _total_floor_vertical_force(records: Sequence[M.V3ContactRecord]) -> float:
    total = 0.0
    for record in records:
        if not record.active_constraint:
            continue
        if record.contact_class in (M.V3ContactClass.LEGAL_PLANTAR_FLOOR,
                                    M.V3ContactClass.PROHIBITED_FLOOR):
            total += float(record.force_on_ground_side_world()[0][2])
    return float(total)


# ===========================================================================
# exact branch engine
# ===========================================================================
class V3BranchEngine:
    """Real MuJoCo branch perturbations from exact restored state.

    No synthetic table mutation, no live-data mutation: one independent
    ``MjData`` scratch instance and one scratch actuation authority are
    restored for every evaluated candidate.
    """

    def __init__(self, plant: V3Plant) -> None:
        self.plant = plant
        self.model = plant.model
        self._dt = float(self.model.opt.timestep)
        self._state_size = int(mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_INTEGRATION))
        self._data = plant.make_data()
        self._authority = V3ActuationAuthority(self._dt)

    def capture_state(self, data: mujoco.MjData, *, sample_index: int, time_s: float,
                      authority: V3ActuationAuthority) -> V3BranchState:
        vector = np.zeros(self._state_size, dtype=np.float64)
        mujoco.mj_getState(self.model, data, vector, mujoco.mjtState.mjSTATE_INTEGRATION)
        return V3BranchState(
            sample_index=int(sample_index),
            time_s=float(time_s),
            state_vector=vector,
            ctrl_nm=np.array(data.ctrl, dtype=np.float64, copy=True),
            qacc_warmstart=np.array(data.qacc_warmstart, dtype=np.float64, copy=True),
            actuation=authority.snapshot_state(),
        )

    # ------------------------------------------------------------------
    def evaluate(self, base: V3BranchState, desired_nm: Sequence[float], *,
                 native_steps: int = CONTROL_INTERVAL_NATIVE_STEPS,
                 phase: str) -> V3BranchOutcome:
        """Replay one candidate over exactly ``native_steps`` native steps."""
        desired = np.asarray(desired_nm, dtype=np.float64)
        if desired.shape != (N_CHANNELS,):
            raise V3LandingFault(f"desired_nm shape {desired.shape} != ({N_CHANNELS},)")
        if not np.all(np.isfinite(desired)):
            raise V3LandingFault("desired_nm contains a non-finite value")
        data = self._data
        mujoco.mj_setState(self.model, data, base.state_vector,
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        data.ctrl[:] = base.ctrl_nm
        data.qacc_warmstart[:] = base.qacc_warmstart
        mujoco.mj_forward(self.model, data)
        authority = self._authority
        authority.restore_state(base.actuation)

        recorder = ActiveSetRecorder(self.plant)
        records_now = M.contact_records(self.plant, data)
        recorder.append(data, sample_index=0, time_s=base.time_s, records=records_now)

        samples: list[V3BranchSampleSummary] = []
        finite = True
        prohibited = False
        max_penetration = 0.0
        min_rom = float("inf")
        min_left = float("inf")
        min_right = float("inf")
        supported_steps = 0
        max_free_run = 0
        current_free_run = 0
        max_moment_ratio = 0.0
        max_power_ratio = 0.0
        hard_moment_ok = True
        hard_power_ok = True
        mtp_ok = True
        no_override = True
        max_active_work = 0.0
        max_total_work = 0.0
        terminal_support = 0

        for j in range(int(native_steps)):
            left_supported, right_supported = _per_foot_legal_support(records_now)
            qdot = np.array([data.qvel[self.plant.idx.vadr[name]] for name in CHANNELS],
                            dtype=np.float64)
            passive = (float(data.qfrc_passive[self.plant.idx.vadr["left_mtp"]]),
                       float(data.qfrc_passive[self.plant.idx.vadr["right_mtp"]]))
            applied, record, _ = authority.apply(
                desired, qdot, phase=phase,
                mtp_active_allowed=(left_supported, right_supported),
                mtp_passive_moment_nm=passive,
                mtp_ledger=authority.internal_mtp_ledger)
            data.ctrl[:] = applied
            mujoco.mj_step(self.model, data)
            mujoco.mj_forward(self.model, data)
            records_now = M.contact_records(self.plant, data)
            recorder.append(data, sample_index=j + 1, time_s=base.time_s + (j + 1) * self._dt,
                            records=records_now)

            com = M.system_com_state(self.plant, data)
            left_fz = _per_foot_normal_force(records_now, "left")
            right_fz = _per_foot_normal_force(records_now, "right")
            legal_count = sum(1 for r in records_now if r.active_legal_plantar)
            prohibited_now = any(r.prohibited for r in records_now)
            penetration = max((float(r.penetration_m) for r in records_now), default=0.0)
            rom_margin = _bounded_joint_margin(data, self.plant)
            sample_finite = bool(
                np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
                and all(math.isfinite(float(v)) for v in com.com_world_m)
                and all(math.isfinite(float(v)) for v in com.com_velocity_world_m_s))
            samples.append(V3BranchSampleSummary(
                offset=j + 1,
                time_s=base.time_s + (j + 1) * self._dt,
                com_world_m=com.com_world_m,
                com_velocity_world_m_s=com.com_velocity_world_m_s,
                left_fz_n=left_fz,
                right_fz_n=right_fz,
                total_floor_fz_n=_total_floor_vertical_force(records_now),
                legal_plantar_active=legal_count,
                prohibited_detected=int(prohibited_now),
                max_penetration_m=float(penetration),
                min_rom_margin_rad=float(rom_margin),
                finite=sample_finite,
            ))
            finite = finite and sample_finite
            prohibited = prohibited or prohibited_now
            max_penetration = max(max_penetration, penetration)
            min_rom = min(min_rom, rom_margin)
            min_left = min(min_left, left_fz)
            min_right = min(min_right, right_fz)
            if legal_count > 0:
                supported_steps += 1
                current_free_run = 0
            else:
                current_free_run += 1
                max_free_run = max(max_free_run, current_free_run)
            terminal_support = legal_count

            overrides = np.asarray(record.safety_override, dtype=bool)
            no_override = no_override and not bool(overrides.any())
            moment_margin = np.asarray(record.moment_margin_nm, dtype=np.float64)
            power_margin = np.asarray(record.power_margin_w, dtype=np.float64)
            hard_moment_ok = hard_moment_ok and bool(np.all(moment_margin >= -1.0e-9))
            hard_power_ok = hard_power_ok and bool(np.all(power_margin >= -1.0e-9))
            max_moment_ratio = max(max_moment_ratio, float(np.max(
                np.abs(np.asarray(record.applied_nm)) / MOMENT_CEILING_NM)))
            max_power_ratio = max(max_power_ratio, float(np.max(
                np.abs(np.asarray(record.joint_power_w)) / POWER_CEILING_W)))
            gated = tuple(bool(v) for v in record.mtp_gated)
            active_mtp = np.asarray(record.mtp_active_applied_nm, dtype=np.float64)
            if (gated[0] and abs(active_mtp[0]) > 1.0e-12) or (gated[1] and abs(active_mtp[1]) > 1.0e-12):
                mtp_ok = False
            for entry in authority.internal_mtp_ledger:
                max_active_work = max(max_active_work, float(entry.active_positive_work_j))
                max_total_work = max(max_total_work, float(entry.total_positive_work_j))
            if (max_active_work > MTP_ACTIVE_POSITIVE_WORK_BUDGET_J * MTP_LATE_ACTIVE_FRACTION + 1.0e-9
                    or max_total_work > MTP_TOTAL_POSITIVE_WORK_BUDGET_J + 1.0e-9):
                mtp_ok = False

        tables = recorder.finalize()
        mode_signature = tables.branch_mode_signature(0, int(native_steps))
        terminal = samples[-1]
        hy = centroidal_hy_kg_m2_s(self.plant, data)
        orientation = M.orientation_state(self.plant, data)
        cop_margin = self._sagittal_cop_margin(data)
        support_fraction = supported_steps / float(native_steps)
        outputs = np.zeros(N_OUTPUTS, dtype=np.float64)
        outputs[OUTPUT_INDEX["com_vx_terminal"]] = terminal.com_velocity_world_m_s[0]
        outputs[OUTPUT_INDEX["com_vz_terminal"]] = terminal.com_velocity_world_m_s[2]
        outputs[OUTPUT_INDEX["hy_terminal"]] = hy
        outputs[OUTPUT_INDEX["root_pitch_terminal"]] = orientation.root_pitch_rad
        outputs[OUTPUT_INDEX["trunk_pitch_terminal"]] = orientation.trunk_absolute_pitch_rad
        outputs[OUTPUT_INDEX["root_pitch_rate_terminal"]] = orientation.root_pitch_rate_rad_s
        outputs[OUTPUT_INDEX["trunk_pitch_rate_terminal"]] = orientation.trunk_absolute_pitch_rate_rad_s
        outputs[OUTPUT_INDEX["min_left_fz"]] = min_left
        outputs[OUTPUT_INDEX["min_right_fz"]] = min_right
        outputs[OUTPUT_INDEX["terminal_left_fz"]] = terminal.left_fz_n
        outputs[OUTPUT_INDEX["terminal_right_fz"]] = terminal.right_fz_n
        outputs[OUTPUT_INDEX["max_penetration"]] = max_penetration
        outputs[OUTPUT_INDEX["min_rom_margin"]] = min_rom
        outputs[OUTPUT_INDEX["cop_margin_terminal"]] = cop_margin
        outputs[OUTPUT_INDEX["support_fraction"]] = support_fraction
        terminal_vector = np.zeros(self._state_size, dtype=np.float64)
        mujoco.mj_getState(self.model, data, terminal_vector,
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        return V3BranchOutcome(
            base=base,
            desired_nm=desired.copy(),
            native_steps=int(native_steps),
            samples=tuple(samples),
            outputs=outputs,
            mode_signature=mode_signature,
            terminal_state_vector=terminal_vector,
            terminal_actuation=authority.snapshot_state(),
            terminal_time_s=base.time_s + int(native_steps) * self._dt,
            finite=finite,
            prohibited_contact=prohibited,
            max_penetration_m=float(max_penetration),
            min_rom_margin_rad=float(min_rom),
            min_left_fz_n=float(min_left),
            min_right_fz_n=float(min_right),
            terminal_left_fz_n=float(terminal.left_fz_n),
            terminal_right_fz_n=float(terminal.right_fz_n),
            max_support_free_run=int(max_free_run),
            terminal_support_free_run=int(current_free_run),
            terminal_legal_support=int(terminal_support),
            support_fraction=float(support_fraction),
            max_moment_ratio=float(max_moment_ratio),
            max_power_ratio=float(max_power_ratio),
            hard_moment_ok=bool(hard_moment_ok),
            hard_power_ok=bool(hard_power_ok),
            mtp_authority_ok=bool(mtp_ok),
            no_safety_override=bool(no_override),
            cop_margin_terminal_m=float(cop_margin),
            max_mtp_active_work_j=float(max_active_work),
            max_mtp_total_work_j=float(max_total_work),
        )

    def _sagittal_cop_margin(self, data: mujoco.MjData) -> float:
        cop = M.cop_from_plant(self.plant, data)
        if cop.cop_x_m is None or cop.cop_y_m is None:
            return float("nan")
        hull = M.active_support_hull(self.plant, data)
        sagittal, _planar, _lateral = M.support_margin_from_point(
            (float(cop.cop_x_m), float(cop.cop_y_m)), hull)
        if sagittal is None:
            return float("nan")
        return float(sagittal)


# ===========================================================================
# derivative columns / constraint rows
# ===========================================================================
@dataclass(frozen=True)
class V3DerivativeColumn:
    coordinate: str
    classification: str
    initial_epsilon_nm: float
    accepted_epsilon_nm: float | None
    nominal_mode_hash: str
    plus_mode_hash: str | None
    minus_mode_hash: str | None
    epsilons_tried_nm: tuple[float, ...]
    jacobian: np.ndarray | None


@dataclass(frozen=True)
class V3ConstraintRow:
    name: str
    kind: str
    value: float
    bound: float | None
    sensitivity: np.ndarray | None
    active: bool

    @property
    def satisfied(self) -> bool:
        if self.bound is None:
            return True
        if self.kind == "lower":
            return self.value >= self.bound - 1.0e-12
        return self.value <= self.bound + 1.0e-12


@dataclass(frozen=True)
class V3LandingStep:
    """One native-sample landing control record (proposed/validated/applied)."""

    index: int
    time_s: float
    phase: str
    previous_phase: str
    transition_reason: str
    control_update: bool
    proposed_desired_nm: np.ndarray
    validated_desired_nm: np.ndarray
    applied_nm: np.ndarray
    fallback: bool
    fallback_reason: str
    failed_gate: str
    trust_region_level: int
    trust_region_nm: np.ndarray
    linear_prediction_max_residual: float
    validated_branch_state_sha256: str
    live_branch_identity: bool | None
    derivatives: tuple[V3DerivativeColumn, ...]
    constraints: tuple[V3ConstraintRow, ...]
    outcome: V3BranchOutcome | None
    actuation: V3AppliedTorque


# ===========================================================================
# deterministic bounded least-squares / penalty solver
# ===========================================================================
def _projected_gradient_solve(objective_jacobian: np.ndarray, residual_target: np.ndarray,
                              weights: np.ndarray, constraint_rows: Sequence[V3ConstraintRow],
                              penalty_weights: Sequence[float], trust: np.ndarray,
                              regularization: float, iterations: int) -> np.ndarray:
    """Deterministic box-constrained regularized least squares with projection.

    ``objective_jacobian`` is ``(n_outputs, n_variables)`` and maps the variable
    step to the output change; ``residual_target`` is the desired output change
    (target - current).  A linearized constraint row is
    ``value + sensitivity @ du >= bound`` (``lower``) or ``<= bound`` (``upper``).

    The solve is: fixed-iteration projected gradient descent on the objective,
    then deterministic alternating projections onto every violated half-space
    in the trust-normalized metric (so a correction is distributed in
    proportion to the declared coordinate trust, never dominated by a noisy
    derivative with a tiny trust region).  Every iterate is clipped to the box.
    The solver only proposes - exact branch validation decides.
    """
    n = int(objective_jacobian.shape[1])
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    weighted_jacobian = objective_jacobian * weights[:, None]
    lipschitz = float(np.sum(weighted_jacobian * weighted_jacobian)) + float(regularization)
    step = 1.0 / max(lipschitz, 1.0e-12)
    rows: list[tuple[np.ndarray, float, float, bool]] = []
    for row, penalty in zip(constraint_rows, penalty_weights):
        if row.sensitivity is None or row.bound is None:
            continue
        if row.kind == "lower":
            violation = row.bound - row.value
            is_lower = True
        else:
            violation = row.value - row.bound
            is_lower = False
        rows.append((np.asarray(row.sensitivity, dtype=np.float64), float(violation),
                     float(penalty), is_lower))
    rows.sort(key=lambda item: item[2], reverse=True)
    du = np.zeros(n, dtype=np.float64)
    for _ in range(int(iterations)):
        residual = (objective_jacobian @ du - residual_target) * weights
        gradient = 2.0 * (objective_jacobian.T @ residual) + 2.0 * float(regularization) * du
        du = np.clip(du - step * gradient, -trust, trust)
    # Trust-normalized alternating projections restore the linearized hard
    # constraint surface while staying as close as possible to the objective
    # iterate in the declared trust metric.  Each row is corrected on the
    # single most effective coordinate (|sensitivity| * trust), so a noisy
    # small-trust derivative can never dominate the correction, and the exact
    # branch validation plus the outer step-scale line search absorbs the
    # secant overestimate near a contact-mode cliff.
    for _ in range(4 * int(iterations)):
        projected_any = False
        for sensitivity, violation, _penalty, is_lower in rows:
            weighted = np.abs(sensitivity) * trust
            index = int(np.argmax(weighted))
            if weighted[index] <= 0.0:
                continue
            shortfall = (violation - float(sensitivity[index] * du[index])) if is_lower else (
                violation + float(sensitivity[index] * du[index]))
            if shortfall > 0.0:
                step_value = shortfall / abs(float(sensitivity[index]))
                if is_lower:
                    du[index] += step_value if sensitivity[index] > 0.0 else -step_value
                else:
                    du[index] += -step_value if sensitivity[index] > 0.0 else step_value
                du = np.clip(du, -trust, trust)
                projected_any = True
        if not projected_any:
            break
    return du


# ===========================================================================
# controller
# ===========================================================================
class V3LandingController:
    """Active-set-safe whole-body landing capture controller."""

    def __init__(self, plant: V3Plant, data: mujoco.MjData, *,
                 config: V3LandingConfig | None = None,
                 actuation: V3ActuationAuthority | None = None,
                 start_phase: V3LandingPhase | None = None) -> None:
        self.plant = plant
        self.config = config or V3LandingConfig()
        self.actuation = actuation if actuation is not None else V3ActuationAuthority(
            float(plant.model.opt.timestep))
        self.engine = V3BranchEngine(plant)
        self._start_phase = start_phase
        self._dt = float(plant.model.opt.timestep)
        if abs(self._dt - NATIVE_DT_S) > 1e-12:
            raise V3LandingFault(f"native dt {self._dt!r} != RES-84 native dt {NATIVE_DT_S!r}")
        self._joint_names = list(CHANNELS)
        self._dof = [int(plant.idx.vadr[name]) for name in self._joint_names]
        self._qadr = [int(plant.idx.qadr[name]) for name in self._joint_names]
        self._rom_lo = np.asarray([
            -np.inf if C.V3_JOINT_RANGES_RAD[name] is None else C.V3_JOINT_RANGES_RAD[name][0]
            for name in self._joint_names], dtype=np.float64)
        self._rom_hi = np.asarray([
            np.inf if C.V3_JOINT_RANGES_RAD[name] is None else C.V3_JOINT_RANGES_RAD[name][1]
            for name in self._joint_names], dtype=np.float64)
        self._calibration_data = plant.make_data()
        self._q_stand, self._s_table, self._z_table = self._calibrate_flexion_family(data)
        self._mtp_ledger = self.actuation.snapshot_state().mtp_ledger
        self._reset_state(data)

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def _reset_state(self, data: mujoco.MjData) -> None:
        self.phase = self._start_phase or V3LandingPhase.LANDING_WAIT
        self._previous_phase = self.phase
        self._transition_reason = ""
        self._step_index = 0
        self._handoff_index: int | None = None
        self._handoff_time_s: float | None = None
        self._first_contact_time_s: float | None = None
        self._bilateral_onset_time_s: float | None = None
        self._e10_confirmed_time_s: float | None = None
        self._sustain_start_time_s: float | None = None
        self._q_handoff = np.zeros(N_CHANNELS, dtype=np.float64)
        self._q_prep_ref = np.zeros(N_CHANNELS, dtype=np.float64)
        self._s_handoff = 0.0
        self._s_depth_tracked = 0.0
        self._last_support_width_m = 0.0
        self._contact_establishment = False
        self._desired = np.zeros(N_CHANNELS, dtype=np.float64)
        self._pending_live_identity: np.ndarray | None = None
        self._gate_penalty = {name: 1.0 for name in CONSTRAINT_ROW_NAMES}
        self._fault_reason: str | None = None
        self._consecutive_support_free = 0
        self._calibrate_handoff_pose(data)

    def _calibrate_handoff_pose(self, data: mujoco.MjData) -> None:
        self._q_handoff = np.array([data.qpos[a] for a in self._qadr], dtype=np.float64)
        self._q_prep_ref = self._q_handoff.copy()
        # Anchor the landing flexion family at the exact incoming landing pose:
        # the handoff configuration is not on the standing family, and a
        # standing-anchored depth table would command a deep squat reference
        # against a leg that is not there.
        self._q_stand, self._s_table, self._z_table = \
            self._calibrate_flexion_family(data, base=self._q_handoff)

    def reset(self, data: mujoco.MjData) -> None:
        self._mtp_ledger = self.actuation.snapshot_state().mtp_ledger
        self._reset_state(data)

    @property
    def e10_confirmed_time_s(self) -> float | None:
        return self._e10_confirmed_time_s

    @property
    def bilateral_onset_time_s(self) -> float | None:
        return self._bilateral_onset_time_s

    @property
    def first_contact_time_s(self) -> float | None:
        return self._first_contact_time_s

    # ------------------------------------------------------------------
    # equipment calibration (Plant geometry only)
    # ------------------------------------------------------------------
    def _calibrate_flexion_family(self, data: mujoco.MjData,
                                  base: np.ndarray | None = None
                                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Calibrated sagittal flexion family and its SYSTEM_COM depth table.

        ``base`` is the family anchor pose.  The landing family is anchored at
        the exact handoff pose (the incoming landing configuration is not on the
        standing family), so ``_s_from_z`` measures additional flexion from the
        handoff configuration and the posture reference tracks the actual leg.
        """
        scratch = self._calibration_data
        q_stand = (np.zeros(N_CHANNELS, dtype=np.float64) if base is None
                   else np.asarray(base, dtype=np.float64).copy())
        ankle = C.V3_JOINT_RANGES_RAD["left_ankle"][1]
        knee = C.V3_JOINT_RANGES_RAD["left_knee"][1]
        hip = C.V3_JOINT_RANGES_RAD["left_hip"][1]
        s_struct = float(max(min(2.0 * ankle, knee, 2.0 * hip) - STRUCTURAL_FLEXION_SAFETY_RAD, 0.1))
        s_table = np.linspace(0.0, s_struct, 72)
        z_table = np.zeros_like(s_table)
        x_stand = float(M.system_com_state(self.plant, data).com_world_m[0])
        for i, s in enumerate(s_table):
            self.plant.reset(scratch)
            reference = q_stand + _FLEXION_DIRECTION * float(s)
            angles = {name: float(reference[k]) for k, name in enumerate(self._joint_names)}
            self.plant.set_joint_angles(scratch, angles)
            self.plant.drop_to_floor(scratch, clearance_m=0.0)
            scratch.qpos[self.plant.idx.qadr["root_tx"]] = x_stand
            mujoco.mj_forward(self.plant.model, scratch)
            z_table[i] = float(M.system_com_state(self.plant, scratch).com_world_m[2])
        # The depth table must be strictly decreasing in s for the inverse map
        # to be well defined: a small flexion offset can momentarily raise the
        # SYSTEM_COM (foot/ankle geometry), so the running minimum is the
        # declared depth of each flexion level and the flat leading region is
        # compacted away.
        z_table = np.minimum.accumulate(z_table)
        keep = np.concatenate(([True], np.diff(z_table) < 0.0))
        s_table = s_table[keep]
        z_table = z_table[keep]
        return q_stand, s_table, z_table

    def _s_from_z(self, z: float) -> float:
        z_clamped = float(np.clip(z, self._z_table[-1], self._z_table[0]))
        return float(np.interp(z_clamped, self._z_table[::-1], self._s_table[::-1]))

    def _posture_reference(self, z: float) -> np.ndarray:
        s = max(self._s_from_z(z), self._s_handoff)
        self._s_depth_tracked = max(self._s_depth_tracked, s)
        return self._q_handoff + _FLEXION_DIRECTION * (s - self._s_handoff)

    # ------------------------------------------------------------------
    # state machine (controller-private; event authority is separate)
    # ------------------------------------------------------------------
    def _transition(self, target: V3LandingPhase, reason: str) -> None:
        if target == self.phase:
            return
        allowed = {
            V3LandingPhase.PRE_TOUCHDOWN: (V3LandingPhase.IMPACT_ABSORPTION,),
            V3LandingPhase.LANDING_WAIT: (V3LandingPhase.IMPACT_ABSORPTION,),
            V3LandingPhase.IMPACT_ABSORPTION: (V3LandingPhase.LANDING_CAPTURE,),
            V3LandingPhase.LANDING_CAPTURE: (V3LandingPhase.E10_CONFIRMED,),
            V3LandingPhase.E10_CONFIRMED: (),
        }
        if target not in allowed[self.phase]:
            self._fault(f"ILLEGAL_LANDING_PHASE_TRANSITION:{self.phase.value}->{target.value}")
        self._previous_phase = self.phase
        self.phase = target
        self._transition_reason = reason

    def _fault(self, reason: str) -> None:
        self._fault_reason = reason
        raise V3LandingFault(reason)

    @staticmethod
    def _bilateral_loaded(frame: M.V3NativeFrame) -> bool:
        return bool(frame.legal_plantar_active > 0
                    and float(frame.left_foot_force_world_n[2]) > F_THR_N
                    and float(frame.right_foot_force_world_n[2]) > F_THR_N)

    def _update_phase(self, frame: M.V3NativeFrame) -> None:
        if self._first_contact_time_s is None and frame.legal_plantar_active > 0:
            self._first_contact_time_s = float(frame.time_s)
        if self.phase in (V3LandingPhase.PRE_TOUCHDOWN, V3LandingPhase.LANDING_WAIT):
            if frame.legal_plantar_active > 0:
                self._transition(V3LandingPhase.IMPACT_ABSORPTION,
                                 "FIRST_LEGAL_POST_FLIGHT_CONTACT")
        bilateral = self._bilateral_loaded(frame)
        if self.phase == V3LandingPhase.IMPACT_ABSORPTION and bilateral:
            self._bilateral_onset_time_s = float(frame.time_s)
            self._sustain_start_time_s = None
            self._transition(V3LandingPhase.LANDING_CAPTURE, "BILATERAL_LOADED_SUPPORT_ESTABLISHED")
        if self.phase == V3LandingPhase.LANDING_CAPTURE:
            predicate = bilateral and abs(float(frame.com_velocity_world_m_s[2])) < V_ABS_TAIL_M_S
            if predicate:
                if self._sustain_start_time_s is None:
                    self._sustain_start_time_s = float(frame.time_s)
                if dwell_confirmed(self._sustain_start_time_s, float(frame.time_s), D_BL_S):
                    self._e10_confirmed_time_s = float(frame.time_s)
                    self._transition(V3LandingPhase.E10_CONFIRMED, "SUSTAINED_WHOLE_BODY_E10_PREDICATE")
            else:
                self._sustain_start_time_s = None

    # ------------------------------------------------------------------
    # post-apex touchdown preparation (flight LANDING_PREP window)
    # ------------------------------------------------------------------
    def _prep_desired(self, data: mujoco.MjData) -> np.ndarray:
        """Bounded causal posture law for the post-apex preparation window.

        There is no ground contact in this window, so the law is a declared
        joint-space posture move along the calibrated flexion family from the
        exact handoff pose with the frozen structural-ROM guard; it never uses
        the ground-force map and never commands contact.
        """
        q = np.array([data.qpos[a] for a in self._qadr], dtype=np.float64)
        qdot = np.array([data.qvel[d] for d in self._dof], dtype=np.float64)
        kp = np.asarray(self.config.joint_kp, dtype=np.float64).copy()
        kd = np.asarray(self.config.joint_kd, dtype=np.float64).copy()
        kp[0] = self.config.trunk_posture_kp
        kd[0] = self.config.trunk_posture_kd
        kp = kp * float(self.config.prep_position_gain_scale)
        kd = kd * float(self.config.prep_damping_gain_scale)
        tau = (np.array(data.qfrc_bias[self._dof], dtype=np.float64)
               - kp * (q - self._q_prep_ref) - kd * qdot)
        tau = self._rom_guard(q, qdot, tau)
        cap = float(self.config.prep_moment_scale) * MOMENT_CEILING_NM
        return np.clip(tau, -cap, cap)

    def _prep_target(self, z: float) -> np.ndarray:
        """Declared touchdown-preparation reference from the exact handoff pose.

        The reference is the handoff pose plus the declared flexion offset along
        the sagittal flexion direction, with an additional declared ankle offset
        for dorsiflexion ROM reserve at touchdown.  The reference is reached
        through the bounded prep rate, never as a snap.
        """
        del z  # the prep reference is a declared offset from the exact handoff pose
        q_ref = (self._q_handoff
                 + _FLEXION_DIRECTION * float(self.config.prep_flexion_target_rad))
        q_ref[5] += float(self.config.prep_ankle_offset_rad)
        q_ref[6] += float(self.config.prep_ankle_offset_rad)
        return q_ref

    def _prep_update(self, *, frame: M.V3NativeFrame, data: mujoco.MjData) -> V3LandingStep:
        dt = self._dt
        target = self._prep_target(float(frame.com_world_m[2]))
        rates = np.full(N_CHANNELS, float(self.config.prep_posture_rate_rad_s),
                        dtype=np.float64)
        rates[5] = rates[6] = float(self.config.prep_ankle_rate_rad_s)
        self._q_prep_ref = self._q_prep_ref + np.clip(
            target - self._q_prep_ref, -rates * dt, rates * dt)
        desired = self._prep_desired(data)
        base = self.engine.capture_state(data, sample_index=int(frame.index),
                                         time_s=float(frame.time_s),
                                         authority=self.actuation)
        # The preparation law is a causal feedback law recomputed and
        # revalidated at every native sample: the validated interval is exactly
        # the one native step that executes.  Holding a free-flight posture
        # action for a whole control cell excites a discrete limit cycle in the
        # low-inertia distal chain.
        outcome = self.engine.evaluate(base, desired, native_steps=1,
                                       phase=self.phase.value)
        failures = self._hard_gate_failures(
            outcome, bilateral_established=False, e10_sustain_active=False,
            prior_support_free_run=int(self._consecutive_support_free),
            enforce_support=False)
        fallback = False
        fallback_reason = ""
        failed_gate = ""
        if failures:
            fallback = True
            fallback_reason = "PREP_ACTION_REJECTED:" + ",".join(failures)
            desired = np.asarray(self.actuation.previous_applied, dtype=np.float64)
            outcome = self.engine.evaluate(base, desired, native_steps=1,
                                           phase=self.phase.value)
            fallback_failures = self._hard_gate_failures(
                outcome, bilateral_established=False, e10_sustain_active=False,
                prior_support_free_run=int(self._consecutive_support_free),
                enforce_support=False)
            if fallback_failures:
                self._fault("PREP_FALLBACK_FAILS_HARD_SAFETY:"
                            + ",".join(fallback_failures))
            failed_gate = failures[0]
        self._desired = np.asarray(desired, dtype=np.float64).copy()
        applied, record, ledger = self._apply_live(self._desired, data)
        self._mtp_ledger = ledger
        return V3LandingStep(
            index=int(frame.index), time_s=float(frame.time_s), phase=self.phase.value,
            previous_phase=self._previous_phase.value,
            transition_reason=self._transition_reason, control_update=True,
            proposed_desired_nm=np.asarray(desired).copy(),
            validated_desired_nm=np.asarray(desired).copy(),
            applied_nm=np.asarray(applied).copy(),
            fallback=bool(fallback), fallback_reason=fallback_reason,
            failed_gate=failed_gate, trust_region_level=0,
            trust_region_nm=np.zeros(len(CONTROL_COORDINATES)),
            linear_prediction_max_residual=0.0,
            validated_branch_state_sha256=outcome.terminal_state_sha256,
            live_branch_identity=None, derivatives=(), constraints=(),
            outcome=outcome, actuation=record)

    # ------------------------------------------------------------------
    # baseline desired moment (causal virtual model + posture)
    # ------------------------------------------------------------------
    def _virtual_joint_moments(self, data: mujoco.MjData, fz_total: float,
                               fx_total: float, x_cop: float,
                               channel_scale: np.ndarray | None = None) -> np.ndarray:
        tau = np.array(data.qfrc_bias[self._dof], dtype=np.float64)
        per_foot_fz = 0.5 * fz_total
        per_foot_fx = 0.5 * fx_total
        y_foot = {"left": 0.085, "right": -0.085}
        for side in C.V3_SIDES:
            force = np.array([per_foot_fx, 0.0, per_foot_fz], dtype=np.float64)
            point = np.array([x_cop, y_foot[side], 0.0], dtype=np.float64)
            for name in ("hip", "knee", "ankle", "mtp"):
                joint_name = f"{side}_{name}"
                jid = int(self.plant.idx.joint[joint_name])
                anchor = np.asarray(data.xanchor[jid], dtype=np.float64)
                axis = np.asarray(data.xaxis[jid], dtype=np.float64)
                arm = np.cross(axis, point - anchor)
                channel = CHANNELS.index(joint_name)
                scale = 1.0 if channel_scale is None else float(channel_scale[channel])
                tau[channel] -= scale * float(np.dot(arm, force))
        return tau

    def _rom_guard(self, q: np.ndarray, qdot: np.ndarray, desired: np.ndarray,
                   guard_scale: np.ndarray | None = None) -> np.ndarray:
        margin = np.full(N_CHANNELS, JOINT_ROM_MARGIN_RAD, dtype=np.float64)
        margin[0] = TRUNK_ROM_GUARD_MARGIN_RAD
        pos_gain = np.full(N_CHANNELS, JOINT_ROM_BARRIER_GAIN, dtype=np.float64)
        pos_gain[0] = TRUNK_ROM_POSITION_GAIN
        vel_gain = np.full(N_CHANNELS, JOINT_ROM_VELOCITY_GAIN, dtype=np.float64)
        vel_gain[0] = TRUNK_ROM_VELOCITY_GAIN
        upper = self._rom_hi - margin
        lower = self._rom_lo + margin
        guard = np.zeros(N_CHANNELS, dtype=np.float64)
        over = q > upper
        under = q < lower
        guard[over] -= pos_gain[over] * (q[over] - upper[over])
        guard[under] += pos_gain[under] * (lower[under] - q[under])
        rate = np.clip(qdot, -JOINT_ROM_GUARD_MAX_RATE_RAD_S, JOINT_ROM_GUARD_MAX_RATE_RAD_S)
        gap_hi = upper - q
        zone_hi = np.maximum(np.abs(rate) * ROM_BRAKE_HORIZON_S, JOINT_ROM_BRAKE_ZONE_RAD)
        frac_hi = np.clip(1.0 - gap_hi / zone_hi, 0.0, 1.0)
        guard -= vel_gain * np.maximum(rate, 0.0) * frac_hi
        gap_lo = q - lower
        zone_lo = np.maximum(np.abs(rate) * ROM_BRAKE_HORIZON_S, JOINT_ROM_BRAKE_ZONE_RAD)
        frac_lo = np.clip(1.0 - gap_lo / zone_lo, 0.0, 1.0)
        guard += vel_gain * np.maximum(-rate, 0.0) * frac_lo
        cap = JOINT_ROM_GUARD_FRACTION * MOMENT_CEILING_NM
        guard = np.clip(guard, -cap, cap)
        if guard_scale is not None:
            guard = guard * guard_scale
        return desired + guard

    def _baseline_desired(self, data: mujoco.MjData, frame: M.V3NativeFrame,
                          snapshot: M.V3MeasurementSnapshot) -> np.ndarray:
        config = self.config
        x = float(frame.com_world_m[0])
        z = float(frame.com_world_m[2])
        vx = float(frame.com_velocity_world_m_s[0])
        vz = float(frame.com_velocity_world_m_s[2])
        q = np.array([data.qpos[a] for a in self._qadr], dtype=np.float64)
        qdot = np.array([data.qvel[d] for d in self._dof], dtype=np.float64)
        hy = centroidal_hy_kg_m2_s(self.plant, data)
        a_z = float(np.clip(-vz / config.t_stop_s, 0.0, config.a_z_max_m_s2))
        fz_stop_demand = max(M.SYSTEM_MASS_KG * (M.GRAVITY_M_S2 + a_z), 0.0)
        # Declared physical-time force-reference ramp from physical first contact.
        # The demand is not throttled by the measured contact force: coupling it
        # to the measured force deadlocks the capture (a contact that is not yet
        # loaded can never ask for the load that would arrest the body).  The
        # ramp starts at the current body weight and reaches the stopping-time
        # demand within the declared ramp time; exact branch validation and the
        # frozen penetration/ROM/support gates decide every executed action.
        if self._first_contact_time_s is None:
            ramp_fraction = 0.0
        else:
            elapsed = float(frame.time_s) - float(self._first_contact_time_s)
            ramp_fraction = float(np.clip(
                elapsed / max(float(config.fz_reference_ramp_s), 1.0e-9), 0.0, 1.0))
        fz_floor = M.SYSTEM_MASS_KG * M.GRAVITY_M_S2
        fz_total = float(min(fz_stop_demand,
                             fz_floor + ramp_fraction * max(fz_stop_demand - fz_floor, 0.0)))
        x_cop_des = x + (hy / config.t_capture_h_s
                         + z * M.SYSTEM_MASS_KG * vx / config.t_capture_x_s) / fz_total
        interval = self._support_interval(snapshot)
        if interval is not None and interval[0] <= interval[1]:
            x_cop = float(np.clip(x_cop_des, interval[0], interval[1]))
        else:
            x_cop = x
        fx_total = float(np.clip(fz_total * (x - x_cop) / max(z, 1.0e-6),
                                 -config.fx_fraction_max * fz_total,
                                 config.fx_fraction_max * fz_total))
        friction_cap = config.friction_mu * fz_total
        fx_total = float(np.clip(fx_total, -friction_cap, friction_cap))
        support_width = 0.0
        hull = snapshot.support_hull
        if hull.evaluable and hull.vertices_xy:
            xs = [v[0] for v in hull.vertices_xy]
            support_width = float(max(xs) - min(xs))
        # Stage A (legal foot-roll/contact establishment): a narrow (toe-only)
        # support keeps the full ground-force map.  The CoP request is clipped
        # to the actual support hull above, and the map's ankle/MTP terms are
        # exactly the plantarflexion moment that holds the toe on the floor
        # while the proximal chain loads.  The support width is retained as
        # declared diagnostic evidence only.
        self._last_support_width_m = support_width
        toe_phase = support_width < config.toe_phase_support_width_m
        self._contact_establishment = bool(toe_phase)
        tau = self._virtual_joint_moments(data, fz_total, fx_total, x_cop)
        q_ref = self._posture_reference(z)
        kp = np.asarray(config.joint_kp, dtype=np.float64).copy()
        kd = np.asarray(config.joint_kd, dtype=np.float64).copy()
        kp[0] = config.trunk_posture_kp
        kd[0] = config.trunk_posture_kd
        tau = tau - kp * (q - q_ref) - kd * qdot
        tau = self._rom_guard(q, qdot, tau)
        cap = DESIRED_MOMENT_FRACTION_CAP * MOMENT_CEILING_NM
        return np.clip(tau, -cap, cap)

    def _support_interval(self, snapshot: M.V3MeasurementSnapshot
                          ) -> tuple[float, float] | None:
        hull = snapshot.support_hull
        if hull.evaluable and hull.vertices_xy:
            xs = [v[0] for v in hull.vertices_xy]
            return (float(min(xs)) + self.config.cop_hull_margin_m,
                    float(max(xs)) - self.config.cop_hull_margin_m)
        return None

    # ------------------------------------------------------------------
    # response map (real branch perturbations)
    # ------------------------------------------------------------------
    def _build_response(self, base: V3BranchState, baseline: np.ndarray, *, phase: str
                        ) -> tuple[V3BranchOutcome, tuple[V3DerivativeColumn, ...]]:
        nominal = self.engine.evaluate(base, baseline, phase=phase)
        coordinates = [name for name in CONTROL_COORDINATES
                       if self.config.include_mtp_coordinate or name != "mtp_pair"]
        columns: list[V3DerivativeColumn] = []
        for coordinate in coordinates:
            ceiling = self.config.coordinate_ceiling(coordinate)
            initial_epsilon = FD_EPS_FRACTIONS[0] * ceiling
            accepted: float | None = None
            classification = DERIVATIVE_UNAVAILABLE
            jacobian: np.ndarray | None = None
            plus_hash: str | None = None
            minus_hash: str | None = None
            tried: list[float] = []
            for fraction in FD_EPS_FRACTIONS:
                epsilon = initial_epsilon * (fraction / FD_EPS_FRACTIONS[0])
                tried.append(float(epsilon))
                plus_desired = baseline.copy()
                minus_desired = baseline.copy()
                for channel in COORDINATE_CHANNELS[coordinate]:
                    plus_desired[channel] += epsilon
                    minus_desired[channel] -= epsilon
                plus = self.engine.evaluate(base, plus_desired, phase=phase)
                minus = self.engine.evaluate(base, minus_desired, phase=phase)
                plus_hash = plus.mode_signature
                minus_hash = minus.mode_signature
                verdict = derivative_eligibility(nominal.mode_signature, plus.mode_signature,
                                                 minus.mode_signature)
                if verdict == DERIVATIVE_CENTRAL_ALLOWED:
                    classification = "CENTRAL"
                    jacobian = (plus.outputs - minus.outputs) / (2.0 * epsilon)
                    accepted = float(epsilon)
                    break
                if verdict == DERIVATIVE_ONE_SIDED_SAME_MODE:
                    if plus.mode_signature == nominal.mode_signature:
                        classification = "PLUS_ONLY"
                        jacobian = (plus.outputs - nominal.outputs) / epsilon
                    else:
                        classification = "MINUS_ONLY"
                        jacobian = (nominal.outputs - minus.outputs) / epsilon
                    accepted = float(epsilon)
                    break
            columns.append(V3DerivativeColumn(
                coordinate=coordinate,
                classification=classification,
                initial_epsilon_nm=float(initial_epsilon),
                accepted_epsilon_nm=accepted,
                nominal_mode_hash=nominal.mode_signature,
                plus_mode_hash=plus_hash,
                minus_mode_hash=minus_hash,
                epsilons_tried_nm=tuple(tried),
                jacobian=jacobian,
            ))
        return nominal, tuple(columns)

    # ------------------------------------------------------------------
    # constraint rows (fixed catalog, never disappearing)
    # ------------------------------------------------------------------
    def _constraint_rows(self, columns: Sequence[V3DerivativeColumn], nominal: V3BranchOutcome,
                         *, bilateral_established: bool,
                         e10_sustain_active: bool = False) -> tuple[V3ConstraintRow, ...]:
        variable_columns = [c for c in columns if c.jacobian is not None]
        index_of = {c.coordinate: i for i, c in enumerate(variable_columns)}

        def sensitivity_for(output_name: str) -> np.ndarray | None:
            if not variable_columns:
                return None
            vector = np.zeros(len(variable_columns), dtype=np.float64)
            for column in variable_columns:
                vector[index_of[column.coordinate]] = float(column.jacobian[OUTPUT_INDEX[output_name]])
            return vector

        fz_bound = F_THR_N if (bilateral_established and e10_sustain_active) else None
        rows = [
            V3ConstraintRow("FZ_LEFT_MIN", "lower", nominal.min_left_fz_n, fz_bound,
                            sensitivity_for("min_left_fz"), active=fz_bound is not None),
            V3ConstraintRow("FZ_RIGHT_MIN", "lower", nominal.min_right_fz_n, fz_bound,
                            sensitivity_for("min_right_fz"), active=fz_bound is not None),
            V3ConstraintRow("MAX_PENETRATION", "upper", nominal.max_penetration_m,
                            PENETRATION_LIMIT_M - PENETRATION_SOLVE_MARGIN_M,
                            sensitivity_for("max_penetration"), active=True),
            V3ConstraintRow("STRUCTURAL_ROM", "lower", nominal.min_rom_margin_rad,
                            ROM_SOLVE_MARGIN_RAD, sensitivity_for("min_rom_margin"), active=True),
            V3ConstraintRow("HARD_MOMENT", "upper", nominal.max_moment_ratio, 1.0, None, active=True),
            V3ConstraintRow("HARD_POWER", "upper", nominal.max_power_ratio, 1.0, None, active=True),
            V3ConstraintRow("MTP_AUTHORITY", "lower",
                            1.0 if nominal.mtp_authority_ok else 0.0, 1.0, None, active=True),
            V3ConstraintRow("PROHIBITED_CONTACT", "lower",
                            0.0 if nominal.prohibited_contact else 1.0, 1.0, None, active=True),
            V3ConstraintRow("SUPPORT_RETENTION", "upper", float(nominal.max_support_free_run),
                            float(SUPPORT_FREE_RUN_LIMIT - 1), None, active=True),
        ]
        return tuple(rows)

    # ------------------------------------------------------------------
    # targets
    # ------------------------------------------------------------------
    def _objective_targets(self, nominal: V3BranchOutcome) -> np.ndarray:
        config = self.config
        t_int = CONTROL_INTERVAL_S
        terminal = nominal.samples[-1]
        vx = float(terminal.com_velocity_world_m_s[0])
        vz = float(terminal.com_velocity_world_m_s[2])
        hy = nominal.output("hy_terminal")
        rp = nominal.output("root_pitch_terminal")
        tp = nominal.output("trunk_pitch_terminal")
        rpr = nominal.output("root_pitch_rate_terminal")
        tpr = nominal.output("trunk_pitch_rate_terminal")
        rpr_command = -rp / config.t_attitude_s - rpr
        tpr_command = -tp / config.t_attitude_s - tpr
        target = np.zeros(N_OUTPUTS, dtype=np.float64)
        target[OUTPUT_INDEX["com_vx_terminal"]] = vx * (1.0 - t_int / config.t_capture_x_s)
        target[OUTPUT_INDEX["com_vz_terminal"]] = vz * (1.0 - t_int / config.t_stop_s)
        target[OUTPUT_INDEX["hy_terminal"]] = hy * (1.0 - t_int / config.t_capture_h_s)
        target[OUTPUT_INDEX["root_pitch_terminal"]] = rp + rpr * t_int
        target[OUTPUT_INDEX["trunk_pitch_terminal"]] = tp + tpr * t_int
        target[OUTPUT_INDEX["root_pitch_rate_terminal"]] = rpr + rpr_command * t_int
        target[OUTPUT_INDEX["trunk_pitch_rate_terminal"]] = tpr + tpr_command * t_int
        return target

    # ------------------------------------------------------------------
    # validation
    # ------------------------------------------------------------------
    def _hard_gate_failures(self, outcome: V3BranchOutcome, *, bilateral_established: bool,
                            e10_sustain_active: bool = False,
                            prior_support_free_run: int = 0,
                            enforce_support: bool = True) -> list[str]:
        failures: list[str] = []
        if not outcome.finite:
            failures.append("NONFINITE_STATE")
        if outcome.prohibited_contact:
            failures.append("PROHIBITED_CONTACT")
        if outcome.max_penetration_m > PENETRATION_LIMIT_M:
            failures.append("MAX_PENETRATION")
        if outcome.min_rom_margin_rad < -V3_STRUCTURAL_ROM_TOLERANCE_RAD:
            failures.append("STRUCTURAL_ROM")
        if not outcome.hard_moment_ok:
            failures.append("HARD_MOMENT")
        if not outcome.hard_power_ok:
            failures.append("HARD_POWER")
        if not outcome.mtp_authority_ok:
            failures.append("MTP_AUTHORITY")
        if enforce_support:
            if outcome.max_support_free_run >= SUPPORT_FREE_RUN_LIMIT:
                failures.append("SUPPORT_RETENTION")
            if prior_support_free_run + outcome.terminal_support_free_run >= \
                    SUPPORT_FREE_RUN_CONTROL_LIMIT:
                failures.append("SUPPORT_RETENTION")
        if bilateral_established and e10_sustain_active:
            # Once the E10 sustain run is active the bilateral loaded predicate
            # must hold at every native sample of the executed interval.
            if outcome.min_left_fz_n < F_THR_N:
                failures.append("FZ_LEFT_MIN")
            if outcome.min_right_fz_n < F_THR_N:
                failures.append("FZ_RIGHT_MIN")
        if bilateral_established and outcome.terminal_legal_support == 0:
            # The executed interval may not end with the system airborne: a
            # contact-preserving action exists throughout the landing, and
            # accepting an airborne interval end is how a reflight begins.
            failures.append("SUPPORT_RETENTION")
        return failures

    @staticmethod
    def _prediction_residual(predicted: np.ndarray, actual: np.ndarray,
                             nominal: np.ndarray) -> float:
        finite = np.isfinite(predicted) & np.isfinite(actual) & np.isfinite(nominal)
        if not np.any(finite):
            return 0.0
        scale = np.maximum(np.abs(actual[finite]), np.maximum(np.abs(nominal[finite]), 1.0e-6))
        return float(np.max(np.abs(predicted[finite] - actual[finite]) / scale))

    # ------------------------------------------------------------------
    # update
    # ------------------------------------------------------------------
    def update(self, frame: M.V3NativeFrame, snapshot: M.V3MeasurementSnapshot,
               frames: Sequence[M.V3NativeFrame], plant: V3Plant,
               data: mujoco.MjData) -> V3LandingStep:
        self._validate_frame(frame)
        self._step_index = int(frame.index)
        self._previous_phase = self.phase
        self._transition_reason = ""
        if self._handoff_index is None:
            self._handoff_index = int(frame.index)
            self._handoff_time_s = float(frame.time_s)
            self._first_contact_time_s = float(frame.time_s)
            self._calibrate_handoff_pose(data)
            self._s_handoff = self._s_from_z(float(frame.com_world_m[2]))
            self._s_depth_tracked = self._s_handoff
        self._update_phase(frame)
        if frame.legal_plantar_active > 0:
            self._consecutive_support_free = 0
        else:
            self._consecutive_support_free = int(self._consecutive_support_free) + 1
        bilateral = self.phase in (V3LandingPhase.LANDING_CAPTURE, V3LandingPhase.E10_CONFIRMED)
        steps = int(self.config.control_interval_native_steps)
        control_update = ((int(frame.index) - int(self._handoff_index)) % steps == 0)
        if self.phase == V3LandingPhase.PRE_TOUCHDOWN:
            return self._prep_update(frame=frame, data=data)
        if control_update:
            return self._control_update(data=data, frame=frame, snapshot=snapshot,
                                        bilateral=bilateral)
        return self._apply_held(frame=frame, data=data)

    def _validate_frame(self, frame: M.V3NativeFrame) -> None:
        values = (frame.index, frame.time_s, *frame.com_world_m, *frame.com_velocity_world_m_s,
                  frame.left_clearance_m, frame.right_clearance_m,
                  frame.legal_plantar_normal_force_n, *frame.total_floor_force_world_n)
        if not all(math.isfinite(float(v)) for v in values):
            self._fault("MALFORMED_OBSERVATION_NON_FINITE")

    def _apply_live(self, desired: np.ndarray, data: mujoco.MjData):
        qdot = np.array([data.qvel[d] for d in self._dof], dtype=np.float64)
        records = M.contact_records(self.plant, data)
        left_supported, right_supported = _per_foot_legal_support(records)
        passive = (float(data.qfrc_passive[self.plant.idx.vadr["left_mtp"]]),
                   float(data.qfrc_passive[self.plant.idx.vadr["right_mtp"]]))
        return self.actuation.apply(
            desired, qdot, phase=self.phase.value,
            mtp_active_allowed=(left_supported, right_supported),
            mtp_passive_moment_nm=passive,
            mtp_ledger=self._mtp_ledger)

    def _apply_held(self, *, frame: M.V3NativeFrame, data: mujoco.MjData) -> V3LandingStep:
        applied, record, ledger = self._apply_live(self._desired, data)
        self._mtp_ledger = ledger
        return V3LandingStep(
            index=int(frame.index), time_s=float(frame.time_s), phase=self.phase.value,
            previous_phase=self._previous_phase.value, transition_reason=self._transition_reason,
            control_update=False, proposed_desired_nm=self._desired.copy(),
            validated_desired_nm=self._desired.copy(), applied_nm=np.asarray(applied).copy(),
            fallback=False, fallback_reason="", failed_gate="", trust_region_level=-1,
            trust_region_nm=np.zeros(len(CONTROL_COORDINATES)), linear_prediction_max_residual=0.0,
            validated_branch_state_sha256="", live_branch_identity=None,
            derivatives=(), constraints=(), outcome=None, actuation=record)

    def _check_live_identity(self, data: mujoco.MjData) -> bool | None:
        """The live state at the new solve must equal the validated branch end."""
        pending = self._pending_live_identity
        if pending is None:
            return None
        live = np.zeros_like(pending)
        mujoco.mj_getState(self.plant.model, data, live, mujoco.mjtState.mjSTATE_INTEGRATION)
        if not np.array_equal(live, pending):
            self._fault("LIVE_BRANCH_IDENTITY_MISMATCH")
        self._pending_live_identity = None
        return True

    def _control_update(self, *, data: mujoco.MjData, frame: M.V3NativeFrame,
                        snapshot: M.V3MeasurementSnapshot, bilateral: bool) -> V3LandingStep:
        live_identity = self._check_live_identity(data)
        base = self.engine.capture_state(data, sample_index=int(frame.index),
                                         time_s=float(frame.time_s),
                                         authority=self.actuation)
        baseline = self._baseline_desired(data, frame, snapshot)
        # The decision variable must lie inside the moment envelope the
        # actuation authority can actually reach over exactly this interval
        # (previous applied +/- rate limit * native steps).  Outside that
        # envelope every command produces the same saturated applied
        # trajectory, so the local finite-difference map would be identically
        # zero; inside it, the perturbations are physically meaningful and the
        # active-set-safe one-sided derivatives appear exactly where they
        # should.
        reach = RATE_CEILING_NM_PER_S * self._dt * int(self.config.control_interval_native_steps)
        previous_applied = np.asarray(self.actuation.previous_applied, dtype=np.float64)
        baseline = np.clip(baseline, previous_applied - reach, previous_applied + reach)
        nominal, columns = self._build_response(base, baseline, phase=self.phase.value)
        target = self._objective_targets(nominal)
        e10_sustain_active = self._sustain_start_time_s is not None
        prior_support_free_run = int(self._consecutive_support_free)
        constraint_rows = self._constraint_rows(
            columns, nominal, bilateral_established=bilateral,
            e10_sustain_active=e10_sustain_active)
        proposed = baseline
        validated = baseline
        fallback = False
        fallback_reason = ""
        failed_gate = ""
        trust_level = 0
        residual = 0.0
        outcome = nominal
        variable_columns = [c for c in columns if c.jacobian is not None]
        nominal_failures = self._hard_gate_failures(
            nominal, bilateral_established=bilateral, e10_sustain_active=e10_sustain_active,
            prior_support_free_run=prior_support_free_run)
        # Penalty weights are a per-solve refinement bias only; they never
        # persist across control cells and are bounded so a flat local response
        # cannot poison later solves.
        self._gate_penalty = {name: 1.0 for name in CONSTRAINT_ROW_NAMES}
        penalty_weights = [float(self._gate_penalty[row.name]) for row in constraint_rows]
        trust_region_nm = np.asarray([self.config.coordinate_trust(c) for c in CONTROL_COORDINATES],
                                     dtype=np.float64)

        if variable_columns:
            jacobian = np.nan_to_num(
                np.stack([c.jacobian for c in variable_columns], axis=1),
                nan=0.0, posinf=0.0, neginf=0.0)
            weights = np.asarray([OBJECTIVE_WEIGHTS.get(name, 0.0) for name in _OUTPUT_NAMES],
                                 dtype=np.float64)
            residual_target = np.nan_to_num(target - nominal.outputs, nan=0.0,
                                            posinf=0.0, neginf=0.0)
            trust_variables = np.asarray([self.config.coordinate_trust(c.coordinate)
                                          for c in variable_columns], dtype=np.float64)
            final_level = int(self.config.max_refinements)
            step_scales = (1.0, 0.75, 0.5, 0.25, 0.125)
            for level in range(final_level + 1):
                trust_level = level
                du = _projected_gradient_solve(
                    jacobian, residual_target, weights, constraint_rows, penalty_weights,
                    trust_variables, self.config.solver_regularization,
                    self.config.solver_iterations)
                accepted = False
                for scale in step_scales:
                    candidate = baseline.copy()
                    for index, column in enumerate(variable_columns):
                        for channel in COORDINATE_CHANNELS[column.coordinate]:
                            candidate[channel] += scale * du[index]
                    proposed = candidate
                    candidate_outcome = self.engine.evaluate(base, candidate,
                                                             phase=self.phase.value)
                    failures = self._hard_gate_failures(
                        candidate_outcome, bilateral_established=bilateral,
                        e10_sustain_active=e10_sustain_active,
                        prior_support_free_run=prior_support_free_run)
                    predicted = nominal.outputs + jacobian @ (scale * du)
                    residual = self._prediction_residual(predicted, candidate_outcome.outputs,
                                                         nominal.outputs)
                    if not failures:
                        # The exact branch passes every hard gate: exact
                        # validation is the authority.  The linearization
                        # residual is recorded evidence, never a rejection on
                        # its own.
                        validated = candidate
                        outcome = candidate_outcome
                        accepted = True
                        break
                    failed_gate = failures[0]
                    for name in failures:
                        self._gate_penalty[name] = min(
                            float(self._gate_penalty[name]) * float(self.config.penalty_growth),
                            float(self.config.penalty_cap))
                if accepted:
                    break
                penalty_weights = [float(self._gate_penalty[row.name]) for row in constraint_rows]
                trust_variables = trust_variables * float(self.config.trust_shrink_factor)
                if level == final_level:
                    if not nominal_failures:
                        # The exact-validated baseline proposal itself survives
                        # every hard gate: it is a real candidate, not a
                        # fallback, and it wins over any failing refinement.
                        validated = baseline
                        outcome = nominal
                        residual = 0.0
                    else:
                        fallback = True
                        fallback_reason = "NO_VALIDATED_CANDIDATE_AFTER_TRUST_REGION_REFINEMENTS"
                        fallback_desired = np.asarray(self.actuation.previous_applied,
                                                      dtype=np.float64)
                        fallback_outcome = self.engine.evaluate(base, fallback_desired,
                                                                phase=self.phase.value)
                        fallback_failures = self._hard_gate_failures(
                            fallback_outcome, bilateral_established=bilateral,
                            e10_sustain_active=e10_sustain_active,
                            prior_support_free_run=prior_support_free_run)
                        if fallback_failures:
                            self._fault("FALLBACK_FAILS_HARD_SAFETY:" + ",".join(fallback_failures))
                        validated = fallback_desired
                        outcome = fallback_outcome
        else:
            if not nominal_failures:
                validated = baseline
                outcome = nominal
            else:
                fallback = True
                fallback_reason = "NO_USABLE_DERIVATIVE_COLUMN"
                fallback_desired = np.asarray(self.actuation.previous_applied, dtype=np.float64)
                outcome = self.engine.evaluate(base, fallback_desired, phase=self.phase.value)
                fallback_failures = self._hard_gate_failures(
                    outcome, bilateral_established=bilateral,
                    e10_sustain_active=e10_sustain_active,
                    prior_support_free_run=prior_support_free_run)
                if fallback_failures:
                    self._fault("FALLBACK_FAILS_HARD_SAFETY:" + ",".join(fallback_failures))
                validated = fallback_desired

        self._desired = np.asarray(validated, dtype=np.float64).copy()
        self._pending_live_identity = np.array(outcome.terminal_state_vector, copy=True)
        applied, record, ledger = self._apply_live(self._desired, data)
        self._mtp_ledger = ledger
        return V3LandingStep(
            index=int(frame.index), time_s=float(frame.time_s), phase=self.phase.value,
            previous_phase=self._previous_phase.value,
            transition_reason=self._transition_reason, control_update=True,
            proposed_desired_nm=np.asarray(proposed).copy(),
            validated_desired_nm=np.asarray(validated).copy(),
            applied_nm=np.asarray(applied).copy(),
            fallback=bool(fallback), fallback_reason=fallback_reason, failed_gate=failed_gate,
            trust_region_level=int(trust_level),
            trust_region_nm=trust_region_nm,
            linear_prediction_max_residual=float(residual),
            validated_branch_state_sha256=outcome.terminal_state_sha256,
            live_branch_identity=live_identity,
            derivatives=columns, constraints=constraint_rows, outcome=outcome,
            actuation=record)

    # ------------------------------------------------------------------
    # evidence surface
    # ------------------------------------------------------------------
    def authority_record(self) -> dict[str, object]:
        return {
            "authority_id": V3_LANDING_CONTROL_AUTHORITY_ID,
            "control_interval_s": CONTROL_INTERVAL_S,
            "control_interval_native_steps": CONTROL_INTERVAL_NATIVE_STEPS,
            "coordinates": list(CONTROL_COORDINATES),
            "coordinate_channels": {k: list(v) for k, v in COORDINATE_CHANNELS.items()},
            "fd_eps_fractions": list(FD_EPS_FRACTIONS),
            "derivative_rule": {
                "central": "MODE(nominal)==MODE(+eps)==MODE(-eps)",
                "one_sided": "exactly one perturbed side remains in nominal mode",
                "unavailable": DERIVATIVE_UNAVAILABLE,
                "version": "RES86_CONTACT_MODE_SIGNATURE_V1",
            },
            "constraint_rows": list(CONSTRAINT_ROW_NAMES),
            "objective_weights": dict(OBJECTIVE_WEIGHTS),
            "penetration_limit_m": PENETRATION_LIMIT_M,
            "support_free_run_limit": SUPPORT_FREE_RUN_LIMIT,
            "support_free_run_control_limit_s": SUPPORT_FREE_RUN_CONTROL_LIMIT_S,
            "f_thr_n": F_THR_N,
            "phase_order": [phase.value for phase in V3LandingPhase],
            "post_apex_prep": {
                "phase": V3LandingPhase.PRE_TOUCHDOWN.value,
                "flexion_target_rad": self.config.prep_flexion_target_rad,
                "ankle_offset_rad": self.config.prep_ankle_offset_rad,
                "posture_rate_rad_s": self.config.prep_posture_rate_rad_s,
                "ankle_rate_rad_s": self.config.prep_ankle_rate_rad_s,
                "moment_scale": self.config.prep_moment_scale,
                "law": "CAUSAL_POSTURE_PD_FROM_HANDOFF_POSE_WITH_FROZEN_ROM_GUARD",
                "contact": "NO_CONTACT_COMMAND_NO_GROUND_FORCE_MAP",
            },
        }


__all__ = [
    "CONTROL_COORDINATES",
    "CONTROL_INTERVAL_NATIVE_STEPS",
    "CONTROL_INTERVAL_S",
    "COORDINATE_CHANNELS",
    "CONSTRAINT_ROW_NAMES",
    "FD_EPS_FRACTIONS",
    "F_THR_N",
    "OBJECTIVE_WEIGHTS",
    "OUTPUT_INDEX",
    "V3BranchEngine",
    "V3BranchOutcome",
    "V3BranchState",
    "V3DerivativeColumn",
    "V3LandingConfig",
    "V3LandingController",
    "V3LandingFault",
    "V3LandingPhase",
    "V3LandingStep",
    "V3_LANDING_CONTROL_AUTHORITY_ID",
]
