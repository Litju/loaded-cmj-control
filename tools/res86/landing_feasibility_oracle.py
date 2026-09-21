#!/usr/bin/env python3
"""RES-86 exact controller-independent landing feasibility oracle.

MISSION: RES86_TOE_PHASE_FEASIBILITY_AND_RESOLUTION_001
LINEAR ISSUE: RES-86

Purpose
-------
The RES-86B landing controller failed to generate positive vertical impulse at
the canonical +20 kg toe-first E8 contact.  This tool answers the prior
question *independently of that controller*: does the exact nominal Plant/contact
realization admit a legal action sequence that can absorb the E8 vertical
momentum?

The oracle never calls the existing landing controller.  Its decision variables
are symmetric sagittal desired-moment profiles (V1 reduction) mapped through the
frozen :class:`loaded_cmj.v3.actuation.V3ActuationAuthority`.  Every branch is
an exact MuJoCo forward-dynamics replay from the sealed PRE_TOUCHDOWN sample-790
certificate; physical touchdown stays endogenous.  The E8 sample-791 certificate
is retained as an identity cross-check.

Two diagnostic classes (mission Phase 2):

* ``toe``   -- any branch whose *active* legal plantar contact mode leaves the
  initial bilateral toe-support mode is rejected.
* ``legal`` -- any legal plantar contact sequence is allowed (toe, forefoot,
  heel or legal combinations), provided no prohibited/fall contact occurs and
  every hard gate passes.

Hard branch gates (mission authority): finite state, no prohibited/fall
contact, ``MAX_PENETRATION <= 0.010 m``, strict structural ROM at the frozen
``1e-9`` comparison tolerance, hard moment/power/MTP-authority PASS, no illegal
support mechanism, no state manipulation, no hidden external force, peak total
floor Fz below the frozen 8-BW fail-safe envelope, and no material reflight.

Optimization discipline: one deterministic low-dimensional action
parameterization (piecewise-linear symmetric moment knots), a deterministic
bounded derivative-free solver (SciPy Powell, no random component), a declared
deterministic initialization ordering and a hard evaluation budget.  The exact
branch is the only authority; the optimizer only proposes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Sequence

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.actuation import (  # noqa: E402
    CHANNELS,
    MOMENT_CEILING_NM,
    MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
    MTP_LATE_ACTIVE_FRACTION,
    MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
    POWER_CEILING_W,
    V3ActuationAuthority,
    V3ActuationState,
    V3MtpLedgerEntry,
)
from loaded_cmj.v3.constants import V3_JOINT_NAMES, V3_JOINT_RANGES_RAD  # noqa: E402
from loaded_cmj.v3.controller import V3ControllerConfig, V3LaunchController  # noqa: E402
from loaded_cmj.v3.landing_authority import D_BL_S, V3_STRUCTURAL_ROM_TOLERANCE_RAD  # noqa: E402
from loaded_cmj.v3.landing_control import (  # noqa: E402
    CONTROL_COORDINATES,
    COORDINATE_CHANNELS,
)
from loaded_cmj.v3.landing_metrics import centroidal_hy_kg_m2_s  # noqa: E402
from loaded_cmj.v3.launch_runtime import settle_standing_stance  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402

V3_FEASIBILITY_ORACLE_AUTHORITY_ID = "LCMJ_RES86_LANDING_FEASIBILITY_ORACLE_V1"

PRE_TOUCHDOWN_SAMPLE = 790
PRE_TOUCHDOWN_TIME_S = 1.580
PRE_TOUCHDOWN_STATE_SHA256 = (
    "08605746ced78fa130c6fc210661fdd611e22e0bbbe20b9758f85b060d93cb42")
E8_SAMPLE = 791
E8_TIME_S = 1.582
E8_STATE_SHA256 = "e411462929c178b97a1132babb46f2fde8c7e12c5d40e40e8f18fa7ce7869355"

NATIVE_DT_S = 0.002
F_THR_N = 10.0
PENETRATION_LIMIT_M = 0.010
PEAK_FZ_LIMIT_BW = 8.0
BW_N = float(C.V3_SYSTEM_WEIGHT_N)
V_ABS_TAIL_M_S = 0.05
MATERIAL_REFLIGHT_SAMPLES = 25  # diagnostic count at nominal dt; never the authority
# Cache schema tag: bump whenever the branch certificate encoding or the
# actuation-ledger semantics change, so a stale development cache is rejected.
BRANCH_CACHE_VERSION = 2

# Deterministic oracle search constants (declared, not tuned post hoc).
DEFAULT_KNOTS = 7
DEFAULT_BUDGET = 4000
SOLVER_MAXITER = 250
SOLVER_XTOL = 1.0e-4
SOLVER_FTOL = 1.0e-6
# Gate penalties are deliberately far above the arrest term scale (|vz| < 4).
PENALTY_PENETRATION = 2.0e4
PENALTY_PEN_INTEGRAL = 4.0e5
PENALTY_PROHIBITED = 1.0e6
PENALTY_ROM = 2.0e4
PENALTY_ROM_INTEGRAL = 4.0e5
PENALTY_MOMENT = 2.0e4
PENALTY_POWER = 2.0e4
PENALTY_MTP = 2.0e4
PENALTY_PEAK_FZ = 2.0e3
PENALTY_FZ_INTEGRAL = 2.0e2
PENALTY_REFLIGHT = 2.0e5
PENALTY_NO_TERMINAL_SUPPORT = 5.0e4
PENALTY_NONFINITE = 1.0e8
PENALTY_TOE_MODE = 1.0e6  # toe-restricted class: leaving the toe mode is a hard rejection

# Piecewise-linear knots for every independent sagittal control coordinate,
# including the MTP pair: the shared actuation authority now latches its MTP
# energy ledger only on energy exhaustion, so a transient flight/support gate
# no longer removes the active MTP channel from the landing control space.  The
# MTP remains bounded by its frozen moment/power ceilings and per-foot energy
# budgets exactly like every other coordinate.
ORACLE_COORDINATES: tuple[str, ...] = tuple(CONTROL_COORDINATES)
N_ORACLE_COORDS = len(ORACLE_COORDINATES)
COORDINATE_CEILING: tuple[float, ...] = tuple(
    min(float(MOMENT_CEILING_NM[channel]) for channel in COORDINATE_CHANNELS[name])
    for name in ORACLE_COORDINATES)


# ===========================================================================
# branch certificate capture (sealed PRE_TOUCHDOWN / E8 identity)
# ===========================================================================
@dataclass(frozen=True)
class BranchCertificate:
    sample_index: int
    time_s: float
    state_vector: np.ndarray
    state_sha256: str
    ctrl_nm: np.ndarray
    qacc_warmstart: np.ndarray
    actuation: V3ActuationState


@dataclass(frozen=True)
class CanonicalBranch:
    pre_touchdown: BranchCertificate
    e8: BranchCertificate


def capture_canonical_branch(plant: V3Plant | None = None,
                             cache_path: Path | None = None) -> CanonicalBranch:
    """Reproduce the sealed RES-85 canonical launch and capture both boundary states.

    The launch loop mirrors ``landing_runtime._run_launch_to_handoff`` exactly
    (same controller, same measurement, same frames history) and asserts the
    sealed state digests at samples 790 and 791.  ``cache_path`` is an optional
    development cache: a cache is only accepted when both certified digests
    match, so it can never displace the sealed identity.
    """
    if cache_path is not None and Path(cache_path).exists():
        cached = _load_branch_cache(Path(cache_path))
        if cached is not None:
            return cached
    plant = plant if plant is not None else V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    launch = V3LaunchController(plant, data, config=V3ControllerConfig(dt_s=NATIVE_DT_S))
    frames: list[M.V3NativeFrame] = []
    flight_phases = ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP")
    captured: dict[int, BranchCertificate] = {}
    size = int(mujoco.mj_stateSize(plant.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    for k in range(E8_SAMPLE + 1):
        frame = M.native_frame(plant, data, k, k * NATIVE_DT_S)
        snapshot = M.measure(plant, data,
                             flight_context=launch.phase.value in flight_phases)
        frames.append(frame)
        if k in (PRE_TOUCHDOWN_SAMPLE, E8_SAMPLE):
            vector = np.zeros(size, dtype=np.float64)
            mujoco.mj_getState(plant.model, data, vector,
                               mujoco.mjtState.mjSTATE_INTEGRATION)
            captured[k] = BranchCertificate(
                sample_index=k,
                time_s=k * NATIVE_DT_S,
                state_vector=vector.copy(),
                state_sha256=hashlib.sha256(vector.tobytes()).hexdigest(),
                ctrl_nm=np.array(data.ctrl, dtype=np.float64, copy=True),
                qacc_warmstart=np.array(data.qacc_warmstart, dtype=np.float64, copy=True),
                actuation=launch.actuation.snapshot_state(),
            )
        step = launch.update(frame, snapshot, frames, plant, data)
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(plant.model, data)
        mujoco.mj_forward(plant.model, data)
    pre = captured[PRE_TOUCHDOWN_SAMPLE]
    e8 = captured[E8_SAMPLE]
    if pre.state_sha256 != PRE_TOUCHDOWN_STATE_SHA256:
        raise RuntimeError(f"PRE_TOUCHDOWN identity failure: {pre.state_sha256}")
    if e8.state_sha256 != E8_STATE_SHA256:
        raise RuntimeError(f"E8 identity failure: {e8.state_sha256}")
    if cache_path is not None:
        _save_branch_cache(Path(cache_path), CanonicalBranch(pre, e8))
    return CanonicalBranch(pre_touchdown=pre, e8=e8)


def _certificate_record(cert: BranchCertificate) -> dict[str, np.ndarray]:
    ledger = cert.actuation.mtp_ledger
    return {
        "sample_index": np.array([cert.sample_index], dtype=np.int64),
        "time_s": np.array([cert.time_s], dtype=np.float64),
        "state_vector": cert.state_vector,
        "state_sha256": np.array([cert.state_sha256], dtype=object),
        "ctrl_nm": cert.ctrl_nm,
        "qacc_warmstart": cert.qacc_warmstart,
        "previous_applied_nm": np.asarray(cert.actuation.previous_applied_nm,
                                          dtype=np.float64),
        "step_index": np.array([cert.actuation.step_index], dtype=np.int64),
        "ledger_active": np.array([entry.active_positive_work_j for entry in ledger]),
        "ledger_total": np.array([entry.total_positive_work_j for entry in ledger]),
        "ledger_gated": np.array([entry.active_gated for entry in ledger]),
        "ledger_late": np.array([entry.late_phase for entry in ledger]),
        "ledger_passive_exceeded": np.array([entry.total_budget_exceeded_by_passive
                                             for entry in ledger]),
    }


def _save_branch_cache(path: Path, branch: CanonicalBranch) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"cache_version": np.array([BRANCH_CACHE_VERSION], dtype=np.int64)}
    for label, cert in (("pre", branch.pre_touchdown), ("e8", branch.e8)):
        for name, array in _certificate_record(cert).items():
            payload[f"{label}_{name}"] = array
    np.savez_compressed(path, **payload)


def _load_branch_cache(path: Path) -> CanonicalBranch | None:
    try:
        data = np.load(path, allow_pickle=True)
        # A development cache is only accepted when it was written by the
        # current cache schema; a stale ledger/gate encoding can never silently
        # displace the current authority semantics.
        if int(data["cache_version"][0]) != BRANCH_CACHE_VERSION:
            return None
        certificates = {}
        for label, expected_sha in (("pre", PRE_TOUCHDOWN_STATE_SHA256),
                                    ("e8", E8_STATE_SHA256)):
            sha = str(data[f"{label}_state_sha256"][0])
            if sha != expected_sha:
                return None
            ledger = tuple(
                V3MtpLedgerEntry(
                    active_positive_work_j=float(data[f"{label}_ledger_active"][foot]),
                    total_positive_work_j=float(data[f"{label}_ledger_total"][foot]),
                    active_gated=bool(data[f"{label}_ledger_gated"][foot]),
                    late_phase=bool(data[f"{label}_ledger_late"][foot]),
                    total_budget_exceeded_by_passive=bool(
                        data[f"{label}_ledger_passive_exceeded"][foot]))
                for foot in (0, 1))
            certificates[label] = BranchCertificate(
                sample_index=int(data[f"{label}_sample_index"][0]),
                time_s=float(data[f"{label}_time_s"][0]),
                state_vector=np.asarray(data[f"{label}_state_vector"], dtype=np.float64),
                state_sha256=sha,
                ctrl_nm=np.asarray(data[f"{label}_ctrl_nm"], dtype=np.float64),
                qacc_warmstart=np.asarray(data[f"{label}_qacc_warmstart"], dtype=np.float64),
                actuation=V3ActuationState(
                    previous_applied_nm=tuple(float(v) for v in
                                              data[f"{label}_previous_applied_nm"]),
                    mtp_ledger=ledger,
                    step_index=int(data[f"{label}_step_index"][0])),
            )
        return CanonicalBranch(pre_touchdown=certificates["pre"],
                               e8=certificates["e8"])
    except (KeyError, ValueError, OSError):
        return None


# ===========================================================================
# exact branch evaluation
# ===========================================================================
@dataclass
class BranchSample:
    offset: int
    time_s: float
    fz_total_n: float
    left_legal_fz_n: float
    right_legal_fz_n: float
    active_regions: tuple[tuple[str, str], ...]
    max_penetration_m: float
    min_rom_margin_rad: float
    prohibited: bool
    prohibited_active: bool
    finite: bool
    com_vz_m_s: float
    com_vx_m_s: float
    com_z_m: float


@dataclass
class BranchResult:
    profile: np.ndarray  # (n_knots, n_coords)
    native_steps: int
    samples: list[BranchSample]
    terminal_state_vector: np.ndarray
    terminal_actuation: V3ActuationState
    terminal_time_s: float
    finite: bool
    terminated_early: bool
    prohibited_any: bool
    prohibited_active_any: bool
    max_penetration_m: float
    min_rom_margin_rad: float
    peak_total_fz_n: float
    min_left_legal_fz_n: float
    min_right_legal_fz_n: float
    terminal_legal_support: bool
    max_support_free_run: int
    material_reflight: bool
    chatter_transitions: int
    toe_mode_escaped: bool
    max_moment_ratio: float
    max_power_ratio: float
    mtp_authority_ok: bool
    terminal_com_vx_m_s: float
    terminal_com_vz_m_s: float
    terminal_hy_kg_m2_s: float
    terminal_root_pitch_rad: float
    terminal_trunk_pitch_rad: float
    terminal_root_pitch_rate_rad_s: float
    terminal_trunk_pitch_rate_rad_s: float
    net_vertical_impulse_ns: float
    grf_vertical_impulse_ns: float
    mode_sequence: list[tuple[tuple[str, str], ...]]
    desired_nm: np.ndarray
    penetration_excess_integral_m_s: float = 0.0
    rom_excess_integral_rad_s: float = 0.0
    peak_fz_excess_integral_n_s: float = 0.0
    evaluation_index: int = 0

    @property
    def terminal_state_sha256(self) -> str:
        return hashlib.sha256(
            np.ascontiguousarray(self.terminal_state_vector).tobytes()).hexdigest()

    @property
    def hard_gate_failures(self) -> list[str]:
        failures: list[str] = []
        if not self.finite:
            failures.append("NONFINITE_STATE")
        if self.prohibited_any:
            failures.append("PROHIBITED_CONTACT")
        if self.max_penetration_m > PENETRATION_LIMIT_M:
            failures.append("MAX_PENETRATION")
        if self.min_rom_margin_rad < -V3_STRUCTURAL_ROM_TOLERANCE_RAD:
            failures.append("STRUCTURAL_ROM")
        if self.max_moment_ratio > 1.0 + 1.0e-9:
            failures.append("HARD_MOMENT")
        if self.max_power_ratio > 1.0 + 1.0e-9:
            failures.append("HARD_POWER")
        if not self.mtp_authority_ok:
            failures.append("MTP_AUTHORITY")
        if self.peak_total_fz_n > PEAK_FZ_LIMIT_BW * BW_N:
            failures.append("PEAK_FZ")
        if self.material_reflight:
            failures.append("MATERIAL_REFLIGHT")
        if not self.terminal_legal_support:
            failures.append("SUPPORT_RETENTION")
        if self.toe_mode_escaped:
            failures.append("TOE_MODE_ESCAPED")
        return failures

    @property
    def admissible(self) -> bool:
        return not self.hard_gate_failures and self.terminal_com_vz_m_s >= -V_ABS_TAIL_M_S


@dataclass
class BranchStabilizer:
    """Fixed, declared action-parameterization stabilizer.

    The oracle decision variables are the profile knots; this fixed term is a
    deterministic function of the exact branch state (frozen RES-85D structural
    ROM guard, declared joint damping, declared trunk posture PD).  The
    stabilizer is part of the *action parameterization*, never an authority
    write: every executed desired moment still passes through
    :class:`V3ActuationAuthority`.  The recorded per-step desired sequence is
    the actual action sequence and is replayed open-loop by
    :meth:`LandingFeasibilityOracle.evaluate_sequence` for the witness.
    """

    enabled: bool = True
    rom_guard_scale: float = 1.0
    joint_damping: float = 15.0
    trunk_kp: float = 250.0
    trunk_kd: float = 30.0
    trunk_ref_rad: float = 0.0
    pen_governor_limit_m: float | None = None
    pen_governor_gain_per_m: float = 0.0

    def __post_init__(self) -> None:
        from loaded_cmj.v3.controller import (
            JOINT_ROM_BARRIER_GAIN,
            JOINT_ROM_BRAKE_ZONE_RAD,
            JOINT_ROM_MARGIN_RAD,
            JOINT_ROM_VELOCITY_GAIN,
            ROM_BRAKE_HORIZON_S,
            TRUNK_ROM_GUARD_MARGIN_RAD,
            TRUNK_ROM_POSITION_GAIN,
            TRUNK_ROM_VELOCITY_GAIN,
        )
        self._margin = np.full(9, JOINT_ROM_MARGIN_RAD, dtype=np.float64)
        self._margin[0] = TRUNK_ROM_GUARD_MARGIN_RAD
        self._pos_gain = np.full(9, JOINT_ROM_BARRIER_GAIN, dtype=np.float64)
        self._pos_gain[0] = TRUNK_ROM_POSITION_GAIN
        self._vel_gain = np.full(9, JOINT_ROM_VELOCITY_GAIN, dtype=np.float64)
        self._vel_gain[0] = TRUNK_ROM_VELOCITY_GAIN
        self._brake_zone = JOINT_ROM_BRAKE_ZONE_RAD
        self._brake_horizon = ROM_BRAKE_HORIZON_S

    def contribution(self, oracle: "LandingFeasibilityOracle", data: mujoco.MjData,
                     desired: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return desired
        profile_term = np.asarray(desired, dtype=np.float64)
        if self.pen_governor_limit_m is not None and self.pen_governor_gain_per_m > 0.0:
            metrics = oracle._fast_metrics(data)
            excess = metrics.max_penetration - float(self.pen_governor_limit_m)
            scale = float(np.clip(1.0 - self.pen_governor_gain_per_m * max(0.0, excess),
                                  0.0, 1.0))
            profile_term = profile_term * scale
        q = np.asarray(data.qpos[oracle._qadr], dtype=np.float64)
        qdot = np.asarray(data.qvel[oracle._dof], dtype=np.float64)
        guard = np.zeros(9, dtype=np.float64)
        upper = oracle._rom_hi - self._margin
        lower = oracle._rom_lo + self._margin
        over = q > upper
        under = q < lower
        guard[over] -= self._pos_gain[over] * (q[over] - upper[over])
        guard[under] += self._pos_gain[under] * (lower[under] - q[under])
        rate = np.clip(qdot, -4.0, 4.0)
        gap_hi = upper - q
        zone_hi = np.maximum(np.abs(rate) * self._brake_horizon, self._brake_zone)
        guard -= self._vel_gain * np.maximum(rate, 0.0) * np.clip(1.0 - gap_hi / zone_hi, 0.0, 1.0)
        gap_lo = q - lower
        zone_lo = np.maximum(np.abs(rate) * self._brake_horizon, self._brake_zone)
        guard += self._vel_gain * np.maximum(-rate, 0.0) * np.clip(1.0 - gap_lo / zone_lo, 0.0, 1.0)
        guard = np.clip(guard, -0.35 * MOMENT_CEILING_NM, 0.35 * MOMENT_CEILING_NM)
        stabilizer = guard * self.rom_guard_scale
        stabilizer -= self.joint_damping * qdot
        stabilizer[0] -= (self.trunk_kp * (q[0] - self.trunk_ref_rad)
                          + self.trunk_kd * qdot[0])
        return profile_term + stabilizer


# ===========================================================================
# structured witness generator (deterministic action-sequence parameterization)
# ===========================================================================
@dataclass(frozen=True)
class StructuredWitnessParameters:
    """Declared low-dimensional action-sequence parameterization.

    This is a *witness generator*, not a controller: it produces a causal
    desired-moment sequence from the exact branch state, the sequence is
    recorded, and the witness is the recorded sequence replayed open-loop and
    verified by the oracle.  The parameterization mirrors the physical landing
    structure (a bounded force reference ramped against the measured contact
    force, a declared CoP request inside the active support, and constant
    coordinate biases), and is never the production landing controller.
    """

    t_stop_s: float
    force_ramp_bw_per_s: float
    fz_cap_bw: float
    cop_blend: float
    ankle_bias_nm: float
    knee_bias_nm: float
    hip_bias_nm: float
    early_hold_bw: float
    early_hold_s: float
    roll_bias_scale: float = 0.0


STRUCTURED_PARAMETER_BOUNDS: tuple[tuple[float, float], ...] = (
    (0.06, 0.60),     # t_stop_s
    (1.0, 60.0),      # force_ramp_bw_per_s
    (1.0, 8.0),       # fz_cap_bw
    (0.0, 1.0),       # cop_blend
    (-150.0, 60.0),   # ankle_bias_nm
    (-250.0, 250.0),  # knee_bias_nm
    (-250.0, 250.0),  # hip_bias_nm
    (0.0, 2.0),       # early_hold_bw
    (0.0, 0.10),      # early_hold_s
    (0.0, 1.0),       # roll_bias_scale
)

STRUCTURED_STARTS: tuple[StructuredWitnessParameters, ...] = (
    StructuredWitnessParameters(0.15, 15.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    StructuredWitnessParameters(0.25, 8.0, 5.0, 0.0, -40.0, -80.0, 0.0, 0.3, 0.04),
    StructuredWitnessParameters(0.12, 30.0, 7.0, 0.3, 0.0, 60.0, 60.0, 0.6, 0.02),
    StructuredWitnessParameters(0.35, 20.0, 6.0, 0.0, -20.0, -150.0, 0.0, 0.5, 0.06),
    StructuredWitnessParameters(0.20, 40.0, 8.0, 0.5, 20.0, 120.0, 60.0, 0.8, 0.03),
)


def _virtual_model_moments(oracle: LandingFeasibilityOracle, data: mujoco.MjData,
                           fz_n: float, fx_n: float, x_cop_m: float,
                           *, bias_scale: float = 1.0) -> np.ndarray:
    """Causal ``bias_scale * bias - J^T F`` sagittal moments at the requested CoP.

    ``bias_scale = 0`` with ``F = 0`` is exactly the slack (zero desired torque)
    roll phase; ``bias_scale = 1`` is the full virtual support model.
    """
    tau = float(bias_scale) * np.asarray(data.qfrc_bias[oracle._dof], dtype=np.float64)
    for side in C.V3_SIDES:
        force = np.array([fx_n * 0.5, 0.0, fz_n * 0.5], dtype=np.float64)
        point = np.array([x_cop_m, 0.085 if side == "left" else -0.085, 0.0])
        for name in ("hip", "knee", "ankle", "mtp"):
            joint_name = f"{side}_{name}"
            jid = int(oracle.plant.idx.joint[joint_name])
            anchor = np.asarray(data.xanchor[jid], dtype=np.float64)
            axis = np.asarray(data.xaxis[jid], dtype=np.float64)
            arm = np.cross(axis, point - anchor)
            channel = CHANNELS.index(joint_name)
            tau[channel] -= float(np.dot(arm, force))
    return tau


def _structured_desired(oracle: LandingFeasibilityOracle, params: StructuredWitnessParameters,
                        data: mujoco.MjData, time_s: float, *,
                        mode_gate: bool = False) -> np.ndarray:
    metrics = oracle._fast_metrics(data)
    com_v = oracle._system_com_velocity(data)
    vz = float(com_v[2])
    vx = float(com_v[0])
    com_x = float(data.subtree_com[0][0])
    cap = float(params.fz_cap_bw) * BW_N
    gate_open = True
    if mode_gate:
        gate_open = any(region != "toe" for _side, region in metrics.active_regions)
    if time_s < params.early_hold_s or (mode_gate and not gate_open):
        fz_ref = min(float(params.early_hold_bw) * BW_N, cap)
        bias_scale = float(params.roll_bias_scale)
    else:
        fz_ref = min(BW_N + float(C.V3_SYSTEM_MASS_KG) * max(0.0, -vz)
                     / max(params.t_stop_s, 1.0e-6), cap)
        bias_scale = 1.0
    fz_cmd = min(fz_ref, metrics.fz_total
                 + float(params.force_ramp_bw_per_s) * BW_N * oracle.dt)
    if metrics.support_x_mean is None:
        x_cop = com_x
    else:
        x_cop = float(metrics.support_x_mean) + (com_x - float(metrics.support_x_mean)) \
            * float(params.cop_blend)
    fx = float(np.clip(-0.5 * float(C.V3_SYSTEM_MASS_KG) * vx, -0.5 * fz_cmd, 0.5 * fz_cmd))
    tau = _virtual_model_moments(oracle, data, fz_cmd, fx, x_cop,
                                 bias_scale=bias_scale)
    for channel in COORDINATE_CHANNELS["hip_pair"]:
        tau[channel] += float(params.hip_bias_nm)
    for channel in COORDINATE_CHANNELS["knee_pair"]:
        tau[channel] += float(params.knee_bias_nm)
    for channel in COORDINATE_CHANNELS["ankle_pair"]:
        tau[channel] += float(params.ankle_bias_nm)
    return np.clip(tau, -0.95 * MOMENT_CEILING_NM, 0.95 * MOMENT_CEILING_NM)


class LandingFeasibilityOracle:
    """Exact controller-independent branch oracle from the sealed 790 certificate."""

    def __init__(self, branch: CanonicalBranch, *, class_name: str = "legal",
                 plant: V3Plant | None = None, native_dt_s: float = NATIVE_DT_S) -> None:
        if class_name not in ("legal", "toe"):
            raise ValueError(f"unknown oracle class {class_name!r}")
        self.branch = branch
        self.class_name = class_name
        self.plant = plant if plant is not None else V3Plant()
        self.model = self.plant.model
        self.dt = float(self.model.opt.timestep)
        self.native_dt_s = float(native_dt_s)
        if abs(self.dt - self.native_dt_s) > 1e-12:
            raise RuntimeError(f"native dt {self.dt!r} != declared oracle dt {self.native_dt_s!r}")
        # D_BL physical-time material-reflight interval count on this native grid.
        self.material_reflight_samples = int(np.ceil(
            D_BL_S / self.dt - 1.0e-9)) if self.dt > 0 else 0
        self.data = self.plant.make_data()
        self._state_size = int(
            mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_INTEGRATION))
        self._dof = np.asarray([int(self.plant.idx.vadr[name]) for name in CHANNELS])
        self._qadr = np.asarray([int(self.plant.idx.qadr[name]) for name in CHANNELS])
        self._mtp_dofs = (int(self.plant.idx.vadr["left_mtp"]),
                          int(self.plant.idx.vadr["right_mtp"]))
        # Fast contact classification tables (authority semantics, no decode).
        self._floor_gid = int(self.plant.idx.geom[C.V3_FLOOR_GEOM])
        self._legal_region: dict[int, tuple[str, str]] = {
            int(self.plant.idx.geom[C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]):
            (side, region)
            for side in C.V3_SIDES for region in C.V3_SUPPORT_REGIONS}
        self._prohibited_geoms = {int(self.plant.idx.geom[name])
                                  for name in C.V3_PROHIBITED_FLOOR_GEOMS}
        self._bounded_joints = [(name, int(self.plant.idx.qadr[name]),
                                 float(V3_JOINT_RANGES_RAD[name][0]),
                                 float(V3_JOINT_RANGES_RAD[name][1]))
                                for name in V3_JOINT_NAMES
                                if V3_JOINT_RANGES_RAD[name] is not None]
        self._cf = np.zeros(6, dtype=np.float64)
        self._frame = np.zeros(9, dtype=np.float64)
        self._rom_lo = np.asarray([
            -np.inf if V3_JOINT_RANGES_RAD[name] is None else float(V3_JOINT_RANGES_RAD[name][0])
            for name in CHANNELS], dtype=np.float64)
        self._rom_hi = np.asarray([
            np.inf if V3_JOINT_RANGES_RAD[name] is None else float(V3_JOINT_RANGES_RAD[name][1])
            for name in CHANNELS], dtype=np.float64)
        self._system_body_ids = np.asarray(
            [b for b in range(self.model.nbody) if b != 0 and self.model.body_mass[b] > 0.0],
            dtype=np.int64)
        self._system_masses = np.asarray(
            self.model.body_mass[self._system_body_ids], dtype=np.float64)
        self._evaluation_count = 0
        self._last_result: BranchResult | None = None

    # ------------------------------------------------------------------
    # profile parameterization
    # ------------------------------------------------------------------
    def knot_times(self, native_steps: int, n_knots: int) -> np.ndarray:
        horizon = native_steps * self.dt
        if n_knots < 2:
            raise ValueError("at least two knots are required")
        return np.linspace(0.0, horizon, n_knots)

    def profile_from_knots(self, knots: np.ndarray, native_steps: int,
                           n_knots: int) -> np.ndarray:
        """(n_knots, n_coords) -> (native_steps, 9) piecewise-linear desired moments."""
        knots = np.asarray(knots, dtype=np.float64)
        if knots.shape != (n_knots, N_ORACLE_COORDS):
            raise ValueError(f"knots shape {knots.shape} != ({n_knots}, {N_ORACLE_COORDS})")
        times = self.knot_times(native_steps, n_knots)
        sample_times = (np.arange(native_steps, dtype=np.float64) + 0.5) * self.dt
        channel_values = np.zeros((native_steps, 9), dtype=np.float64)
        for index, coordinate in enumerate(ORACLE_COORDINATES):
            interpolated = np.interp(sample_times, times, knots[:, index])
            for channel in COORDINATE_CHANNELS[coordinate]:
                channel_values[:, channel] = interpolated
        return channel_values

    # ------------------------------------------------------------------
    # exact evaluation
    # ------------------------------------------------------------------
    def evaluate_sequence(self, sequence: np.ndarray, native_steps: int, *,
                          collect_samples: bool = True,
                          evaluation_index: int = 0) -> BranchResult:
        """Replay an explicit per-step desired-moment sequence over the horizon."""
        sequence = np.asarray(sequence, dtype=np.float64)
        if sequence.shape != (native_steps, 9):
            raise ValueError(f"sequence shape {sequence.shape} != ({native_steps}, 9)")
        return self._replay(sequence, native_steps, collect_samples=collect_samples,
                            evaluation_index=evaluation_index)

    def evaluate(self, knots: np.ndarray, native_steps: int, *,
                 collect_samples: bool = True, evaluation_index: int = 0,
                 stabilizer: BranchStabilizer | None = None,
                 record_actions: bool = False) -> BranchResult:
        n_knots = int(np.asarray(knots).shape[0])
        channel_values = self.profile_from_knots(knots, native_steps, n_knots)
        return self._replay(channel_values, native_steps, collect_samples=collect_samples,
                            evaluation_index=evaluation_index, stabilizer=stabilizer,
                            record_actions=record_actions)

    def _replay(self, channel_values: np.ndarray | None, native_steps: int, *,
                collect_samples: bool = True, evaluation_index: int = 0,
                stabilizer: BranchStabilizer | None = None,
                record_actions: bool = False,
                policy: Callable[[mujoco.MjData, int], np.ndarray] | None = None) -> BranchResult:
        data = self.data
        model = self.model
        mujoco.mj_setState(model, data, self.branch.pre_touchdown.state_vector,
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        data.ctrl[:] = self.branch.pre_touchdown.ctrl_nm
        data.qacc_warmstart[:] = self.branch.pre_touchdown.qacc_warmstart
        mujoco.mj_forward(model, data)
        authority = V3ActuationAuthority(self.dt)
        authority.restore_state(self.branch.pre_touchdown.actuation)

        samples: list[BranchSample] = []
        finite = True
        prohibited_any = False
        prohibited_active_any = False
        max_penetration = 0.0
        min_rom = float("inf")
        peak_fz = 0.0
        min_left = float("inf")
        min_right = float("inf")
        max_moment_ratio = 0.0
        max_power_ratio = 0.0
        mtp_ok = True
        support_free_run = 0
        max_support_free_run = 0
        chatter = 0
        prev_support: bool | None = None
        mode_sequence: list[tuple[tuple[str, str], ...]] = []
        toe_mode_escaped = False
        net_impulse = 0.0
        grf_impulse = 0.0
        penetration_excess_integral = 0.0
        rom_excess_integral = 0.0
        peak_fz_excess_integral = 0.0
        terminated_early = False

        current = self._fast_metrics(data)
        if record_actions or stabilizer is not None:
            executed_actions = np.zeros((int(native_steps), 9), dtype=np.float64)
        else:
            executed_actions = None
        for j in range(int(native_steps)):
            desired = np.zeros(9, dtype=np.float64)
            if policy is not None:
                desired[:] = np.asarray(policy(data, j), dtype=np.float64)
            elif channel_values is not None:
                desired[:] = channel_values[j]
            else:
                raise ValueError("either channel_values or policy is required")
            if stabilizer is not None:
                desired = stabilizer.contribution(self, data, desired)
            if executed_actions is not None:
                executed_actions[j] = desired
            left_support = current.left_legal
            right_support = current.right_legal
            qdot = np.asarray(data.qvel[self._dof], dtype=np.float64)
            passive = (float(data.qfrc_passive[self._mtp_dofs[0]]),
                       float(data.qfrc_passive[self._mtp_dofs[1]]))
            applied, record, _ = authority.apply(
                desired, qdot, phase="IMPACT_ABSORPTION",
                mtp_active_allowed=(left_support, right_support),
                mtp_passive_moment_nm=passive,
                mtp_ledger=authority.internal_mtp_ledger)
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            current = self._fast_metrics(data)

            com_v = self._system_com_velocity(data) if collect_samples else None
            fz_total = current.fz_total
            net_impulse += (fz_total - BW_N) * self.dt
            grf_impulse += fz_total * self.dt
            penetration_excess_integral += max(
                0.0, current.max_penetration - PENETRATION_LIMIT_M) * self.dt
            peak_fz_excess_integral += max(
                0.0, fz_total - PEAK_FZ_LIMIT_BW * BW_N) * self.dt
            peak_fz = max(peak_fz, fz_total)
            max_penetration = max(max_penetration, current.max_penetration)
            rom_margin = self._rom_margin(data)
            rom_excess_integral += max(0.0, -rom_margin) * self.dt
            min_rom = min(min_rom, rom_margin)
            min_left = min(min_left, current.left_fz)
            min_right = min(min_right, current.right_fz)
            prohibited_any = prohibited_any or current.prohibited
            prohibited_active_any = prohibited_active_any or current.prohibited_active
            finite = finite and current.finite
            moment_ratio = float(np.max(np.abs(np.asarray(record.applied_nm))
                                        / MOMENT_CEILING_NM))
            power_ratio = float(np.max(np.abs(np.asarray(record.joint_power_w))
                                       / POWER_CEILING_W))
            max_moment_ratio = max(max_moment_ratio, moment_ratio)
            max_power_ratio = max(max_power_ratio, power_ratio)
            gated = tuple(bool(v) for v in record.mtp_gated)
            active_mtp = np.asarray(record.mtp_active_applied_nm, dtype=np.float64)
            if (gated[0] and abs(active_mtp[0]) > 1.0e-12) or \
                    (gated[1] and abs(active_mtp[1]) > 1.0e-12):
                mtp_ok = False
            for entry in authority.internal_mtp_ledger:
                if (entry.active_positive_work_j
                        > MTP_ACTIVE_POSITIVE_WORK_BUDGET_J * MTP_LATE_ACTIVE_FRACTION + 1.0e-9
                        or entry.total_positive_work_j > MTP_TOTAL_POSITIVE_WORK_BUDGET_J + 1.0e-9):
                    mtp_ok = False
            support = current.legal_active_count > 0
            if prev_support is not None and support != prev_support:
                chatter += 1
            prev_support = support
            if support:
                support_free_run = 0
            else:
                support_free_run += 1
                max_support_free_run = max(max_support_free_run, support_free_run)
            if self.class_name == "toe":
                for _side, region in current.active_regions:
                    if region != "toe":
                        toe_mode_escaped = True
            if collect_samples:
                samples.append(BranchSample(
                    offset=j + 1,
                    time_s=self.branch.pre_touchdown.time_s + (j + 1) * self.dt,
                    fz_total_n=fz_total,
                    left_legal_fz_n=current.left_fz,
                    right_legal_fz_n=current.right_fz,
                    active_regions=tuple(sorted(current.active_regions)),
                    max_penetration_m=current.max_penetration,
                    min_rom_margin_rad=rom_margin,
                    prohibited=current.prohibited,
                    prohibited_active=current.prohibited_active,
                    finite=current.finite,
                    com_vz_m_s=float(com_v[2]),
                    com_vx_m_s=float(com_v[0]),
                    com_z_m=float(data.subtree_com[0][2]),
                ))
            if collect_samples:
                active = tuple(sorted(current.active_regions))
                if not mode_sequence or mode_sequence[-1] != active:
                    mode_sequence.append(active)
            if not finite:
                terminated_early = True
                break

        terminal_vector = np.zeros(self._state_size, dtype=np.float64)
        mujoco.mj_getState(model, data, terminal_vector,
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        orientation = M.orientation_state(self.plant, data)
        terminal_com_v = self._system_com_velocity(data)
        terminal_support = current.legal_active_count > 0
        material_reflight = max_support_free_run >= self.material_reflight_samples
        result = BranchResult(
            profile=np.asarray(channel_values, dtype=np.float64).copy(),
            native_steps=int(native_steps),
            samples=samples,
            terminal_state_vector=terminal_vector,
            terminal_actuation=authority.snapshot_state(),
            terminal_time_s=self.branch.pre_touchdown.time_s + int(native_steps) * self.dt,
            finite=finite,
            terminated_early=terminated_early,
            prohibited_any=prohibited_any,
            prohibited_active_any=prohibited_active_any,
            max_penetration_m=float(max_penetration),
            min_rom_margin_rad=float(min_rom),
            peak_total_fz_n=float(peak_fz),
            min_left_legal_fz_n=float(min_left),
            min_right_legal_fz_n=float(min_right),
            terminal_legal_support=bool(terminal_support),
            max_support_free_run=int(max_support_free_run),
            material_reflight=bool(material_reflight),
            chatter_transitions=int(chatter),
            toe_mode_escaped=bool(toe_mode_escaped),
            max_moment_ratio=float(max_moment_ratio),
            max_power_ratio=float(max_power_ratio),
            mtp_authority_ok=bool(mtp_ok),
            terminal_com_vx_m_s=float(terminal_com_v[0]),
            terminal_com_vz_m_s=float(terminal_com_v[2]),
            terminal_hy_kg_m2_s=float(centroidal_hy_kg_m2_s(self.plant, data)),
            terminal_root_pitch_rad=float(orientation.root_pitch_rad),
            terminal_trunk_pitch_rad=float(orientation.trunk_absolute_pitch_rad),
            terminal_root_pitch_rate_rad_s=float(orientation.root_pitch_rate_rad_s),
            terminal_trunk_pitch_rate_rad_s=float(orientation.trunk_absolute_pitch_rate_rad_s),
            net_vertical_impulse_ns=float(net_impulse),
            grf_vertical_impulse_ns=float(grf_impulse),
            mode_sequence=mode_sequence,
            desired_nm=(executed_actions if executed_actions is not None
                        else (np.asarray(channel_values, dtype=np.float64)
                              if channel_values is not None
                              else np.zeros((int(native_steps), 9), dtype=np.float64))),
            penetration_excess_integral_m_s=float(penetration_excess_integral),
            rom_excess_integral_rad_s=float(rom_excess_integral),
            peak_fz_excess_integral_n_s=float(peak_fz_excess_integral),
            evaluation_index=int(evaluation_index),
        )
        self._evaluation_count = max(self._evaluation_count, int(evaluation_index) + 1)
        self._last_result = result
        return result

    # ------------------------------------------------------------------
    # fast authority-consistent contact decoding
    # ------------------------------------------------------------------
    @dataclass
    class _FastMetrics:
        fz_total: float
        left_fz: float
        right_fz: float
        left_legal: bool
        right_legal: bool
        legal_active_count: int
        active_regions: set
        max_penetration: float
        prohibited: bool
        prohibited_active: bool
        finite: bool
        support_x_mean: float | None = None

    def _fast_metrics(self, data: mujoco.MjData) -> "_FastMetrics":
        floor = self._floor_gid
        fz_total = 0.0
        left_fz = 0.0
        right_fz = 0.0
        active_regions: set = set()
        max_penetration = 0.0
        prohibited = False
        prohibited_active = False
        support_x_values: list[float] = []
        finite = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
        for contact_id in range(int(data.ncon)):
            contact = data.contact[contact_id]
            g1 = int(contact.geom1)
            g2 = int(contact.geom2)
            dist = float(contact.dist)
            if dist < 0.0:
                max_penetration = max(max_penetration, -dist)
            if g1 == floor:
                other = g2
                athlete_is_geom2 = True
            elif g2 == floor:
                other = g1
                athlete_is_geom2 = False
            else:
                continue
            mujoco.mj_contactForce(self.model, data, contact_id, self._cf)
            # World-frame force on the athlete side: raw force acts on geom2's
            # body (RES-84 measured convention); flip when the athlete is geom1.
            frame = np.asarray(contact.frame, dtype=np.float64).reshape(3, 3)
            world_force = frame.T @ self._cf[:3]
            sign = 1.0 if athlete_is_geom2 else -1.0
            wz = sign * float(world_force[2])
            normal = float(self._cf[0])
            region = self._legal_region.get(other)
            if region is not None:
                if normal > 0.0:
                    side, region_name = region
                    if side == "left":
                        left_fz += normal
                    else:
                        right_fz += normal
                    active_regions.add((side, region_name))
                    support_x_values.append(float(contact.pos[0]))
            elif other in self._prohibited_geoms:
                prohibited = True
                if normal > 0.0:
                    prohibited_active = True
            fz_total += wz
        left_legal = any(region[0] == "left" for region in active_regions)
        right_legal = any(region[0] == "right" for region in active_regions)
        return LandingFeasibilityOracle._FastMetrics(
            fz_total=float(fz_total),
            left_fz=float(left_fz),
            right_fz=float(right_fz),
            left_legal=bool(left_legal),
            right_legal=bool(right_legal),
            legal_active_count=len(active_regions),
            active_regions=active_regions,
            max_penetration=float(max_penetration),
            prohibited=bool(prohibited),
            prohibited_active=bool(prohibited_active),
            finite=finite,
            support_x_mean=(float(np.mean(support_x_values)) if support_x_values else None),
        )

    def _rom_margin(self, data: mujoco.MjData) -> float:
        margin = float("inf")
        for _name, qadr, lower, upper in self._bounded_joints:
            q = float(data.qpos[qadr])
            margin = min(margin, q - lower, upper - q)
        return margin

    def _system_com_velocity(self, data: mujoco.MjData) -> np.ndarray:
        """Bit-identical vectorized SYSTEM_COM velocity (RES-84 mass state formula)."""
        body_ids = self._system_body_ids
        masses = self._system_masses
        reference = np.asarray(data.subtree_com[0], dtype=np.float64)
        cvel = np.asarray(data.cvel[body_ids], dtype=np.float64)
        xipos = np.asarray(data.xipos[body_ids], dtype=np.float64)
        velocities = cvel[:, 3:] + np.cross(cvel[:, :3], xipos - reference)
        return (masses[:, None] * velocities).sum(axis=0) / float(C.V3_SYSTEM_MASS_KG)

    # ------------------------------------------------------------------
    # cost function for the deterministic bounded solver
    # ------------------------------------------------------------------
    def cost(self, result: BranchResult) -> float:
        if not result.finite:
            return PENALTY_NONFINITE
        value = -result.terminal_com_vz_m_s
        value += PENALTY_PENETRATION * max(0.0, result.max_penetration_m - PENETRATION_LIMIT_M)
        value += PENALTY_PEN_INTEGRAL * result.penetration_excess_integral_m_s
        if result.prohibited_any:
            value += PENALTY_PROHIBITED
        value += PENALTY_ROM * max(0.0, -result.min_rom_margin_rad)
        value += PENALTY_ROM_INTEGRAL * result.rom_excess_integral_rad_s
        value += PENALTY_MOMENT * max(0.0, result.max_moment_ratio - 1.0)
        value += PENALTY_POWER * max(0.0, result.max_power_ratio - 1.0)
        if not result.mtp_authority_ok:
            value += PENALTY_MTP
        value += PENALTY_PEAK_FZ * max(
            0.0, result.peak_total_fz_n / BW_N - PEAK_FZ_LIMIT_BW)
        value += PENALTY_FZ_INTEGRAL * result.peak_fz_excess_integral_n_s
        if result.material_reflight:
            value += PENALTY_REFLIGHT
        if not result.terminal_legal_support:
            value += PENALTY_NO_TERMINAL_SUPPORT
        if self.class_name == "toe" and result.toe_mode_escaped:
            value += PENALTY_TOE_MODE
        return float(value)

    # ------------------------------------------------------------------
    # deterministic bounded search
    # ------------------------------------------------------------------
    def search(self, native_steps: int, n_knots: int, budget: int,
               *, initial_knots: Sequence[np.ndarray] | None = None,
               verbose: bool = False) -> dict:
        from scipy.optimize import minimize

        bounds_lower = np.tile([-ceiling for ceiling in COORDINATE_CEILING],
                               (n_knots, 1))
        bounds_upper = np.tile([+ceiling for ceiling in COORDINATE_CEILING],
                               (n_knots, 1))
        starts: list[np.ndarray] = []
        if initial_knots is not None:
            starts.extend(np.asarray(knots, dtype=np.float64)
                          for knots in initial_knots)
        if not starts:
            starts.append(np.zeros((n_knots, N_ORACLE_COORDS), dtype=np.float64))
        evaluations = {"count": 0}

        def objective(flat: np.ndarray) -> float:
            evaluations["count"] += 1
            knots = flat.reshape(n_knots, N_ORACLE_COORDS)
            result = self.evaluate(knots, native_steps,
                                   collect_samples=False,
                                   evaluation_index=evaluations["count"])
            return self.cost(result)

        history: list[dict] = []
        best: dict | None = None
        for start_index, start in enumerate(starts):
            if evaluations["count"] >= budget:
                break
            remaining = budget - evaluations["count"]
            options = {"maxfev": max(1, int(remaining)),
                       "xtol": SOLVER_XTOL, "ftol": SOLVER_FTOL, "disp": False}
            solution = minimize(objective, start.ravel(), method="Powell",
                                bounds=list(zip(bounds_lower.ravel(), bounds_upper.ravel())),
                                options=options)
            knots = solution.x.reshape(n_knots, N_ORACLE_COORDS)
            result = self.evaluate(knots, native_steps, collect_samples=True,
                                   evaluation_index=evaluations["count"])
            final_cost = self.cost(result)
            history.append({
                "start_index": start_index,
                "cost": float(final_cost),
                "terminal_vz": float(result.terminal_com_vz_m_s),
                "admissible": bool(result.admissible),
                "hard_gate_failures": result.hard_gate_failures,
                "solver_success": bool(solution.success),
                "solver_message": str(solution.message),
            })
            candidate = {"knots": knots, "result": result, "cost": final_cost}
            if best is None or final_cost < best["cost"]:
                best = candidate
            if verbose:
                print(json.dumps(history[-1]), flush=True)
            if result.admissible:
                break
        assert best is not None
        return {
            "best": best,
            "evaluations": int(evaluations["count"]),
            "budget": int(budget),
            "history": history,
            "n_knots": int(n_knots),
            "native_steps": int(native_steps),
            "horizon_s": float(native_steps * self.dt),
            "class": self.class_name,
        }

    # ------------------------------------------------------------------
    # deterministic coordinate-descent profile search
    # ------------------------------------------------------------------
    def sequence_to_knots(self, sequence: np.ndarray, native_steps: int,
                          n_knots: int) -> np.ndarray:
        """Encode an explicit per-step sequence as piecewise-linear knots."""
        sequence = np.asarray(sequence, dtype=np.float64)
        times = self.knot_times(native_steps, n_knots)
        sample_times = (np.arange(native_steps, dtype=np.float64) + 0.5) * self.dt
        knots = np.zeros((n_knots, N_ORACLE_COORDS), dtype=np.float64)
        for index, coordinate in enumerate(ORACLE_COORDINATES):
            channel = COORDINATE_CHANNELS[coordinate][0]
            knots[:, index] = np.interp(times, sample_times, sequence[:, channel])
        return knots

    def search_coordinate_descent(self, native_steps: int, n_knots: int, budget: int,
                                  *, initial_knots: Sequence[np.ndarray] | None = None,
                                  stabilizer: BranchStabilizer | None = None,
                                  max_sweeps: int = 6, verbose: bool = False) -> dict:
        """Bounded deterministic coordinate descent over the profile knots.

        Each knot coordinate is line-searched over a declared multiscale lattice
        around its current value (fractions of the coordinate ceiling).  No
        random component; the exact branch always decides.  When a
        :class:`BranchStabilizer` is supplied it is active during the search and
        the recorded executed action sequence is the witness action sequence.
        """
        starts = [np.asarray(k, dtype=np.float64).copy()
                  for k in (initial_knots or [])]
        if not starts:
            starts = [np.zeros((n_knots, N_ORACLE_COORDS), dtype=np.float64)]
        ceiling = np.asarray(COORDINATE_CEILING, dtype=np.float64)
        lattice = (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125)
        evaluations = {"count": 0}

        def evaluate_knots(knots: np.ndarray) -> tuple[float, BranchResult]:
            evaluations["count"] += 1
            result = self.evaluate(knots, native_steps, collect_samples=False,
                                   evaluation_index=evaluations["count"],
                                   stabilizer=stabilizer, record_actions=True)
            return self.cost(result), result

        history: list[dict] = []
        best: dict | None = None
        for start_index, start in enumerate(starts):
            current = start.copy()
            current_cost, current_result = evaluate_knots(current)
            for sweep in range(int(max_sweeps)):
                improved = False
                for knot in range(n_knots):
                    if evaluations["count"] >= budget:
                        break
                    for coord in range(N_ORACLE_COORDS):
                        if evaluations["count"] >= budget:
                            break
                        base = float(current[knot, coord])
                        candidates: list[float] = []
                        for fraction in lattice:
                            delta = fraction * float(ceiling[coord])
                            for sign in (+1.0, -1.0):
                                value = float(np.clip(base + sign * delta,
                                                      -ceiling[coord], ceiling[coord]))
                                if value not in candidates:
                                    candidates.append(value)
                        for value in candidates:
                            if evaluations["count"] >= budget:
                                break
                            trial = current.copy()
                            trial[knot, coord] = value
                            trial_cost, trial_result = evaluate_knots(trial)
                            if trial_cost < current_cost - 1.0e-9:
                                current = trial
                                current_cost = trial_cost
                                current_result = trial_result
                                improved = True
                if verbose:
                    history.append({
                        "start_index": start_index, "sweep": sweep,
                        "cost": float(current_cost),
                        "terminal_vz": float(current_result.terminal_com_vz_m_s),
                        "admissible": bool(current_result.admissible),
                        "evaluations": int(evaluations["count"]),
                    })
                    print(json.dumps(history[-1]), flush=True)
                if not improved or evaluations["count"] >= budget:
                    break
            candidate = {"knots": current, "result": current_result, "cost": current_cost}
            if best is None or candidate["cost"] < best["cost"]:
                best = candidate
            if current_result.admissible:
                break
        assert best is not None
        return {
            "best": best,
            "evaluations": int(evaluations["count"]),
            "budget": int(budget),
            "history": history,
            "n_knots": int(n_knots),
            "native_steps": int(native_steps),
            "horizon_s": float(native_steps * self.dt),
            "class": self.class_name,
            "solver": "deterministic_bounded_coordinate_descent_v1",
        }

    # ------------------------------------------------------------------
    # full-authority witness verification
    # ------------------------------------------------------------------
    def evaluate_structured(self, params: StructuredWitnessParameters, native_steps: int, *,
                            collect_samples: bool = True, evaluation_index: int = 0,
                            stabilizer: BranchStabilizer | None = None,
                            mode_gate: bool = False) -> BranchResult:
        """Replay the declared structured witness parameterization.

        ``mode_gate`` holds the braking demand at the early-hold level until the
        active legal contact mode leaves the initial toe-only mode, so the foot
        can roll flat before load is accepted.
        """
        return self._replay(None, native_steps, collect_samples=collect_samples,
                            evaluation_index=evaluation_index, stabilizer=stabilizer,
                            record_actions=True,
                            policy=lambda data, j: _structured_desired(
                                self, params, data,
                                self.branch.pre_touchdown.time_s + j * self.dt,
                                mode_gate=mode_gate))

    def search_structured(self, native_steps: int, budget: int, *,
                          stabilizer: BranchStabilizer | None = None,
                          starts: Sequence[StructuredWitnessParameters] | None = None,
                          mode_gate: bool = False,
                          verbose: bool = False) -> dict:
        """Deterministic bounded Powell search over the structured witness law."""
        from scipy.optimize import minimize

        start_list = list(starts) if starts is not None else list(STRUCTURED_STARTS)
        bounds = list(STRUCTURED_PARAMETER_BOUNDS)
        evaluations = {"count": 0}

        def flatten(params: StructuredWitnessParameters) -> np.ndarray:
            return np.asarray([getattr(params, field) for field in
                               StructuredWitnessParameters.__dataclass_fields__],
                              dtype=np.float64)

        def unflatten(vector: np.ndarray) -> StructuredWitnessParameters:
            vector = np.clip(vector, [b[0] for b in bounds], [b[1] for b in bounds])
            return StructuredWitnessParameters(
                **{field: float(vector[index]) for index, field in enumerate(
                    StructuredWitnessParameters.__dataclass_fields__)})

        def objective(vector: np.ndarray) -> float:
            evaluations["count"] += 1
            result = self.evaluate_structured(
                unflatten(vector), native_steps, collect_samples=False,
                evaluation_index=evaluations["count"], stabilizer=stabilizer,
                mode_gate=mode_gate)
            return self.cost(result)

        history: list[dict] = []
        best: dict | None = None
        for start_index, start in enumerate(start_list):
            if evaluations["count"] >= budget:
                break
            remaining = budget - evaluations["count"]
            solution = minimize(
                objective, flatten(start), method="Powell", bounds=bounds,
                options={"maxfev": max(1, int(remaining)), "xtol": SOLVER_XTOL,
                         "ftol": SOLVER_FTOL, "disp": False})
            params = unflatten(solution.x)
            result = self.evaluate_structured(params, native_steps, collect_samples=True,
                                              evaluation_index=evaluations["count"],
                                              stabilizer=stabilizer,
                                              mode_gate=mode_gate)
            value = self.cost(result)
            history.append({
                "start_index": start_index,
                "cost": float(value),
                "terminal_vz": float(result.terminal_com_vz_m_s),
                "admissible": bool(result.admissible),
                "hard_gate_failures": result.hard_gate_failures,
                "evaluations": int(evaluations["count"]),
            })
            if verbose:
                print(json.dumps(history[-1]), flush=True)
            candidate = {"params": params, "result": result, "cost": value}
            if best is None or value < best["cost"]:
                best = candidate
            if result.admissible:
                break
        assert best is not None
        return {
            "best": best,
            "evaluations": int(evaluations["count"]),
            "budget": int(budget),
            "history": history,
            "native_steps": int(native_steps),
            "horizon_s": float(native_steps * self.dt),
            "class": self.class_name,
            "solver": "deterministic_bounded_powell_structured_law_v1",
        }

    def verify(self, actions: np.ndarray, native_steps: int) -> dict:
        """Replay the witness with the full RES-84 decoder and authority metrics.

        ``actions`` is the recorded per-step desired-moment sequence (exactly the
        action sequence that was executed during the search).  The result must
        match the fast-path evaluation; any mismatch fails closed.
        """
        from loaded_cmj.v3.active_set_capture import ActiveSetRecorder
        from loaded_cmj.v3.landing_metrics import (
            material_reflight_intervals,
            max_penetration_m,
        )

        actions = np.asarray(actions, dtype=np.float64)
        if actions.shape != (native_steps, 9):
            raise ValueError(f"witness actions shape {actions.shape} != ({native_steps}, 9)")
        fast = self.evaluate_sequence(actions, native_steps, collect_samples=False)
        profile = actions
        data = self.plant.make_data()
        model = self.model
        mujoco.mj_setState(model, data, self.branch.pre_touchdown.state_vector,
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        data.ctrl[:] = self.branch.pre_touchdown.ctrl_nm
        data.qacc_warmstart[:] = self.branch.pre_touchdown.qacc_warmstart
        mujoco.mj_forward(model, data)
        authority = V3ActuationAuthority(self.dt)
        authority.restore_state(self.branch.pre_touchdown.actuation)
        recorder = ActiveSetRecorder(self.plant)
        records_now = M.contact_records(self.plant, data)
        recorder.append(data, sample_index=self.branch.pre_touchdown.sample_index,
                        time_s=self.branch.pre_touchdown.time_s, records=records_now)
        fz_series = []
        support_series = []
        legal_fz = {"left": [], "right": []}
        penetration_series = []
        prohibited_detected = []
        prohibited_active = []
        rom_series = []
        moment_ratio_series = []
        power_ratio_series = []
        com_vz_series = []
        for j in range(int(native_steps)):
            desired = profile[j]
            left_support = any(r.active_legal_plantar and r.side == "left"
                               for r in records_now)
            right_support = any(r.active_legal_plantar and r.side == "right"
                                for r in records_now)
            qdot = np.asarray(data.qvel[self._dof], dtype=np.float64)
            passive = (float(data.qfrc_passive[self._mtp_dofs[0]]),
                       float(data.qfrc_passive[self._mtp_dofs[1]]))
            applied, record, _ = authority.apply(
                desired, qdot, phase="IMPACT_ABSORPTION",
                mtp_active_allowed=(left_support, right_support),
                mtp_passive_moment_nm=passive,
                mtp_ledger=authority.internal_mtp_ledger)
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            records_now = M.contact_records(self.plant, data)
            recorder.append(data,
                            sample_index=self.branch.pre_touchdown.sample_index + j + 1,
                            time_s=self.branch.pre_touchdown.time_s + (j + 1) * self.dt,
                            records=records_now)
            fz_total = sum(float(r.force_on_ground_side_world()[0][2]) for r in records_now
                           if r.active_constraint and r.contact_class in (
                               M.V3ContactClass.LEGAL_PLANTAR_FLOOR,
                               M.V3ContactClass.PROHIBITED_FLOOR))
            fz_series.append(fz_total)
            support_series.append(any(r.active_legal_plantar for r in records_now))
            for side in ("left", "right"):
                legal_fz[side].append(sum(r.normal_force_n for r in records_now
                                          if r.active_legal_plantar and r.side == side))
            penetration_series.append(max_penetration_m(records_now))
            prohibited_detected.append(any(r.prohibited for r in records_now))
            prohibited_active.append(any(r.prohibited and r.active_constraint
                                         for r in records_now))
            rom_series.append(self._rom_margin(data))
            moment_ratio_series.append(float(np.max(np.abs(np.asarray(record.applied_nm))
                                                    / MOMENT_CEILING_NM)))
            power_ratio_series.append(float(np.max(np.abs(np.asarray(record.joint_power_w))
                                                   / POWER_CEILING_W)))
            com_vz_series.append(float(M.system_com_state(self.plant, data)
                                       .com_velocity_world_m_s[2]))
        times = [self.branch.pre_touchdown.time_s + (j + 1) * self.dt
                 for j in range(int(native_steps))]
        support = np.asarray(support_series, dtype=bool)
        reflights = material_reflight_intervals(support, 0, sample_times_s=times,
                                                min_duration_s=D_BL_S)
        chatter = int(np.count_nonzero(support[1:] != support[:-1])) if support.size > 1 else 0
        checks = {
            "peak_fz_match": bool(abs(max(fz_series) - fast.peak_total_fz_n) <= 1.0e-6),
            "penetration_match": bool(abs(max(penetration_series)
                                          - fast.max_penetration_m) <= 1.0e-9),
            "rom_match": bool(abs(min(rom_series) - fast.min_rom_margin_rad) <= 1.0e-9),
            "terminal_vz_match": bool(abs(com_vz_series[-1] - fast.terminal_com_vz_m_s)
                                      <= 1.0e-9),
            "prohibited_match": bool(any(prohibited_detected) == fast.prohibited_any),
            "chatter_match": bool(chatter == fast.chatter_transitions),
            "reflight_match": bool(bool(reflights) == fast.material_reflight),
            "net_impulse_match": bool(
                abs(sum((fz - BW_N) * self.dt for fz in fz_series)
                    - fast.net_vertical_impulse_ns) <= 1.0e-9),
            "moment_match": bool(abs(max(moment_ratio_series) - fast.max_moment_ratio)
                                 <= 1.0e-12),
            "power_match": bool(abs(max(power_ratio_series) - fast.max_power_ratio)
                                <= 1.0e-12),
        }
        return {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "fast": {
                "terminal_vz": fast.terminal_com_vz_m_s,
                "max_penetration_m": fast.max_penetration_m,
                "peak_total_fz_bw": fast.peak_total_fz_n / BW_N,
                "net_vertical_impulse_ns": fast.net_vertical_impulse_ns,
                "hard_gate_failures": fast.hard_gate_failures,
                "admissible": fast.admissible,
                "mode_sequence": fast.mode_sequence,
                "max_moment_ratio": fast.max_moment_ratio,
                "max_power_ratio": fast.max_power_ratio,
                "min_rom_margin_rad": fast.min_rom_margin_rad,
                "chatter_transitions": fast.chatter_transitions,
                "material_reflight": fast.material_reflight,
                "terminal_legal_support": fast.terminal_legal_support,
            },
            "full": {
                "max_penetration_m": float(max(penetration_series)),
                "peak_total_fz_bw": float(max(fz_series) / BW_N),
                "terminal_vz": float(com_vz_series[-1]),
                "net_vertical_impulse_ns": float(sum((fz - BW_N) * self.dt
                                                     for fz in fz_series)),
                "chatter_transitions": int(chatter),
                "material_reflight_intervals": [list(pair) for pair in reflights],
                "prohibited_detected_samples": int(np.count_nonzero(prohibited_detected)),
                "prohibited_active_samples": int(np.count_nonzero(prohibited_active)),
                "min_rom_margin_rad": float(min(rom_series)),
                "max_moment_ratio": float(max(moment_ratio_series)),
                "max_power_ratio": float(max(power_ratio_series)),
                "mode_sequence": fast.mode_sequence,
            },
        }


# ===========================================================================
# deterministic initial profiles (declared ordering, no random component)
# ===========================================================================
def static_support_seed(oracle: LandingFeasibilityOracle, n_knots: int,
                        scales: Sequence[float]) -> list[np.ndarray]:
    """Quasi-static support moment seed from the E8 geometry at the toe CoP.

    ``tau = bias - J^T F`` with ``F = (0, 0, BW)`` applied at the instantaneous
    toe support centroid, exactly the causal virtual-model construction owned by
    the RES-85 controller, evaluated once at the certificate state.
    """
    plant = oracle.plant
    data = oracle.data
    mujoco.mj_setState(plant.model, data, oracle.branch.pre_touchdown.state_vector,
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(plant.model, data)
    # advance to the E8 state so the seed sees the first contact geometry
    data.ctrl[:] = oracle.branch.e8.ctrl_nm
    mujoco.mj_step(plant.model, data)
    mujoco.mj_forward(plant.model, data)
    records = M.contact_records(plant, data)
    active = [r for r in records if r.active_legal_plantar]
    if not active:
        raise RuntimeError("static seed requires an active legal contact at E8")
    x_cop = float(np.mean([r.position_world_m[0] for r in active]))
    tau = np.asarray(data.qfrc_bias[oracle._dof], dtype=np.float64)
    for side in C.V3_SIDES:
        force = np.array([0.0, 0.0, BW_N], dtype=np.float64)
        point = np.array([x_cop, 0.085 if side == "left" else -0.085, 0.0])
        for name in ("hip", "knee", "ankle", "mtp"):
            joint_name = f"{side}_{name}"
            jid = int(plant.idx.joint[joint_name])
            anchor = np.asarray(data.xanchor[jid], dtype=np.float64)
            axis = np.asarray(data.xaxis[jid], dtype=np.float64)
            arm = np.cross(axis, point - anchor)
            channel = CHANNELS.index(joint_name)
            tau[channel] -= float(np.dot(arm, force))
    seeds: list[np.ndarray] = []
    for scale in scales:
        knots = np.zeros((n_knots, N_ORACLE_COORDS), dtype=np.float64)
        for index, coordinate in enumerate(ORACLE_COORDINATES):
            channel = COORDINATE_CHANNELS[coordinate][0]
            knots[:, index] = float(np.clip(
                scale * tau[channel], -COORDINATE_CEILING[index],
                COORDINATE_CEILING[index]))
        seeds.append(knots)
    return seeds


def default_seed_ordering(oracle: LandingFeasibilityOracle, n_knots: int) -> list[np.ndarray]:
    """Deterministic initialization ordering (no random component)."""
    seeds = [np.zeros((n_knots, N_ORACLE_COORDS), dtype=np.float64)]
    try:
        seeds.extend(static_support_seed(oracle, n_knots, (0.5, 1.0, 1.5, 2.0)))
    except RuntimeError:
        pass
    return seeds


# ===========================================================================
# CLI
# ===========================================================================
def _result_record(result: BranchResult, oracle: LandingFeasibilityOracle,
                   cost: float) -> dict:
    return {
        "class": oracle.class_name,
        "native_steps": result.native_steps,
        "horizon_s": result.native_steps * NATIVE_DT_S,
        "terminal_time_s": result.terminal_time_s,
        "cost": float(cost),
        "admissible": bool(result.admissible),
        "hard_gate_failures": result.hard_gate_failures,
        "terminal_com_vz_m_s": result.terminal_com_vz_m_s,
        "terminal_com_vx_m_s": result.terminal_com_vx_m_s,
        "net_vertical_impulse_ns": result.net_vertical_impulse_ns,
        "grf_vertical_impulse_ns": result.grf_vertical_impulse_ns,
        "peak_total_fz_n": result.peak_total_fz_n,
        "peak_total_fz_bw": result.peak_total_fz_n / BW_N,
        "max_penetration_m": result.max_penetration_m,
        "min_rom_margin_rad": result.min_rom_margin_rad,
        "min_left_legal_fz_n": result.min_left_legal_fz_n,
        "min_right_legal_fz_n": result.min_right_legal_fz_n,
        "terminal_legal_support": result.terminal_legal_support,
        "max_support_free_run": result.max_support_free_run,
        "material_reflight": result.material_reflight,
        "chatter_transitions": result.chatter_transitions,
        "toe_mode_escaped": result.toe_mode_escaped,
        "prohibited_any": result.prohibited_any,
        "max_moment_ratio": result.max_moment_ratio,
        "max_power_ratio": result.max_power_ratio,
        "mtp_authority_ok": result.mtp_authority_ok,
        "terminal_hy_kg_m2_s": result.terminal_hy_kg_m2_s,
        "terminal_root_pitch_rad": result.terminal_root_pitch_rad,
        "terminal_trunk_pitch_rad": result.terminal_trunk_pitch_rad,
        "terminal_root_pitch_rate_rad_s": result.terminal_root_pitch_rate_rad_s,
        "terminal_trunk_pitch_rate_rad_s": result.terminal_trunk_pitch_rate_rad_s,
        "mode_sequence": [[list(entry) for entry in mode] for mode in result.mode_sequence],
        "knots": np.asarray(result.profile, dtype=np.float64).tolist(),
        "profile_channel_nm": np.asarray(result.desired_nm, dtype=np.float64).tolist(),
        "evaluation_index": result.evaluation_index,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--class", dest="class_name", choices=("legal", "toe"), default="legal")
    parser.add_argument("--horizon-s", type=float, default=0.300)
    parser.add_argument("--knots", type=int, default=DEFAULT_KNOTS)
    parser.add_argument("--budget", type=int, default=DEFAULT_BUDGET)
    parser.add_argument("--solver", choices=("cd", "powell"), default="cd")
    parser.add_argument("--stabilizer", choices=("on", "off"), default="on")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--certificate-cache", type=Path, default=None)
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    t0 = time.time()
    branch = capture_canonical_branch(cache_path=args.certificate_cache)
    oracle = LandingFeasibilityOracle(branch, class_name=args.class_name)
    native_steps = int(round(args.horizon_s / NATIVE_DT_S))
    stabilizer = BranchStabilizer(enabled=args.stabilizer == "on")
    if args.solver == "cd":
        seeds = default_seed_ordering(oracle, args.knots)
        search = oracle.search_coordinate_descent(
            native_steps, args.knots, args.budget, initial_knots=seeds,
            stabilizer=stabilizer, verbose=args.verbose)
    else:
        seeds = default_seed_ordering(oracle, args.knots)
        search = oracle.search(native_steps, args.knots, args.budget,
                               initial_knots=seeds, verbose=args.verbose)
    best = search["best"]
    result: BranchResult = best["result"]
    record = _result_record(result, oracle, best["cost"])
    record["oracle_authority_id"] = V3_FEASIBILITY_ORACLE_AUTHORITY_ID
    record["oracle_evaluations_used"] = search["evaluations"]
    record["oracle_evaluation_budget"] = search["budget"]
    record["oracle_solver"] = search.get("solver", "deterministic_bounded_powell")
    record["stabilizer"] = {
        "enabled": stabilizer.enabled,
        "joint_damping": stabilizer.joint_damping,
        "trunk_kp": stabilizer.trunk_kp,
        "trunk_kd": stabilizer.trunk_kd,
        "rom_guard_scale": stabilizer.rom_guard_scale,
        "role": "FIXED_ACTION_PARAMETERIZATION_TERM_NOT_AUTHORITY_WRITE",
    }
    record["pre_touchdown_state_sha256"] = branch.pre_touchdown.state_sha256
    record["e8_state_sha256"] = branch.e8.state_sha256
    record["search_history"] = search["history"]
    record["wall_s"] = time.time() - t0
    if args.verify:
        record["full_verification"] = oracle.verify(result.desired_nm, native_steps)
    print(json.dumps({k: v for k, v in record.items()
                      if k not in ("profile_channel_nm", "knots")},
                     indent=2, default=str))
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(record, indent=2, default=str) + "\n")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
