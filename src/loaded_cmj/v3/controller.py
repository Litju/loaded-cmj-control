"""V3 causal loaded-CMJ launch / flight controller (RES-85).

Authority: ``LCMJ_RES85_PHASE_MACHINE_AUTHORITY_V1`` (phase machine),
``LCMJ_RES85_ACTUATION_AUTHORITY_V1`` (actuation) and
``LCMJ_RES85_MTP_ENERGY_AUTHORITY_V1`` (MTP energetics).

Design summary
--------------
* The controller consumes the RES-84 measurement primitives through
  :class:`loaded_cmj.v3.measurement.V3NativeFrame` and
  :class:`~loaded_cmj.v3.measurement.V3MeasurementSnapshot` **only**.  Contact
  truth, support classification, SYSTEM_COM, takeoff occurrence, takeoff
  confirmation, the diagnostic comparator, PHYSICAL_TIME dwell, clearance and
  the CoP/support hull are never re-implemented here.
* The phase machine is causal: the descent is driven by a vertical-force
  reference whose velocity command is frozen and whose tracking error comes
  from the measured SYSTEM_COM state; BRAKING is entered from the measured
  descent state against the structural flexion bound; PROPULSION is entered on
  the physical vertical reversal.  Wall-clock time is used for the declared
  quiet-stand debounce only.
* The control law is a virtual-model controller: a desired SYSTEM_COM
  acceleration determines a desired ground reaction force; the required net
  joint moments are ``bias - J^T F`` over the supporting leg chains, plus a
  joint-space posture error term.  The CoP request is clamped into the active
  support hull and the horizontal force is re-derived from the clamped CoP.
* Every applied moment passes through :class:`V3ActuationAuthority`; no phase
  code writes ``data.ctrl`` directly.
* Flight posture is continuous with terminal propulsion (no snap to a zero
  pose); LANDING_PREP prepares a posture only.  RES-85 claims no landing
  capture and no recovery optimisation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import measurement as M
from loaded_cmj.v3.actuation import (
    CHANNELS,
    N_CHANNELS,
    V3ActuationAuthority,
    V3AppliedTorque,
    V3MtpLedgerEntry,
    plant_mtp_passive_moment_nm,
)
from loaded_cmj.v3.constants import (
    V3_JOINT_NAMES,
    V3_JOINT_RANGES_RAD,
    V3_SIDES,
)
from loaded_cmj.v3.plant import V3Plant

# ===========================================================================
# frozen control-law constants (engineering nominal, declared)
# ===========================================================================
V3_CONTROLLER_AUTHORITY_ID = "LCMJ_RES85_CAUSAL_LAUNCH_CONTROLLER_V1"

QUIET_STAND_SAMPLES = 125             # 0.25 s of native samples
QUIET_VZ_MAX_M_S = 0.02
QUIET_VX_MAX_M_S = 0.02
QUIET_SAGITTAL_MARGIN_MIN_M = 0.02

A_COUNTERMOVEMENT_M_S2 = -1.0
V_COUNTERMOVEMENT_CMD_M_S = -0.35
A_BRAKE_M_S2 = 2.5
V_BRAKE_TRIGGER_M_S = -0.55
BRAKING_TRIGGER_MARGIN_M = 0.02
A_THRUST_M_S2 = 17.0
THRUST_AZ_MIN_M_S2 = -4.0
THRUST_AZ_MAX_M_S2 = 21.0
REFERENCE_LEAD_MAX_M = 0.02
REFERENCE_LAG_MAX_M = 0.05
REFERENCE_V_REF_MAX_M_S = 4.0

# ---------------------------------------------------------------------------
# RES-85C propulsion-contact engineering variables (declared, bounded, causal)
#
# Diagnosis (RES-85C propulsion-deficit report): during PROPULSION the leg
# extension rate exceeded the family-consistent rate for the measured
# SYSTEM_COM rise rate, so the foot progressively lost penetration, the
# contact force collapsed and a premature support loss truncated the stroke
# about 0.10 m below full extension.  The variables below let the sagittal
# joint rates be slaved to the measured SYSTEM_COM velocity through the
# calibrated plant pose family, keep the trunk inside its ROM instead of
# reacting to the hip-extension moment, and keep a controlled foot preload.
# ---------------------------------------------------------------------------
EXTENSION_RATE_FF_GAIN = 0.5
CONTACT_PRELOAD_M = 0.005
TRUNK_LEAN_FRAC = 0.35
TRUNK_EXTEND_FRAC = 0.40
TRUNK_KP = 240.0
TRUNK_KD = 30.0

# ---------------------------------------------------------------------------
# RES-85D strict structural-ROM guard (state causal, predictive)
#
# The RES-85C barrier was position-only: it reacted after the joint entered the
# margin band and could not stop a joint whose outward velocity had already
# been built up by the coupled propulsion dynamics, so the measured
# trunk_pelvis coordinate crossed the frozen +-35 deg structural bound.  The
# guard is now POSITION_GUARD + OUTWARD_VELOCITY_BRAKING: within a declared
# brake zone it applies an opposing moment proportional to the measured
# outward joint velocity, and the zone grows with a declared stopping horizon
# (|qdot| * ROM_BRAKE_HORIZON_S), so a fast joint begins braking before it can
# cross the frozen bound.  The guard is exactly zero in the interior, never
# propels a joint toward a limit, uses only measured state and stays inside
# the frozen actuation authority (it is a desired-moment term; the actuation
# layer still enforces moment/power/rate).  It never writes Plant state.
# ---------------------------------------------------------------------------
JOINT_ROM_MARGIN_RAD = 0.08
JOINT_ROM_BARRIER_GAIN = 800.0
JOINT_ROM_VELOCITY_GAIN = 5000.0
TRUNK_ROM_GUARD_MARGIN_RAD = 0.12
TRUNK_ROM_POSITION_GAIN = 2500.0
TRUNK_ROM_VELOCITY_GAIN = 5000.0
ROM_BRAKE_HORIZON_S = 0.10
JOINT_ROM_BRAKE_ZONE_RAD = 0.15

STRUCTURAL_FLEXION_SAFETY_RAD = 0.05

KZ_STAND = 40.0
DZ_STAND = 10.0
KZ_TRACK = 60.0
DZ_TRACK = 14.0
KX_BALANCE = 30.0
DX_BALANCE = 8.0
FX_FRACTION_MAX = 0.4
COP_HULL_MARGIN_M = 0.005

JOINT_KP = np.array([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 4.0, 4.0])
JOINT_KD = np.array([6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 0.4, 0.4])

LANDING_PREP_FLEXION = 0.5
LANDING_PREP_RATE_PER_S = 1.0

FLEXION_DIRECTION = np.array([0.0, 0.5, 0.5, 1.0, 1.0, 0.5, 0.5, 0.0, 0.0])


class V3Phase(str, Enum):
    STAND = "STAND"
    COUNTERMOVEMENT = "COUNTERMOVEMENT"
    BRAKING = "BRAKING"
    PROPULSION = "PROPULSION"
    TAKEOFF_CONFIRM = "TAKEOFF_CONFIRM"
    FLIGHT = "FLIGHT"
    LANDING_PREP = "LANDING_PREP"


PHASE_ORDER = (
    V3Phase.STAND, V3Phase.COUNTERMOVEMENT, V3Phase.BRAKING, V3Phase.PROPULSION,
    V3Phase.TAKEOFF_CONFIRM, V3Phase.FLIGHT, V3Phase.LANDING_PREP,
)

SUPPORTED_PHASES = (V3Phase.STAND, V3Phase.COUNTERMOVEMENT, V3Phase.BRAKING,
                    V3Phase.PROPULSION)


class V3ControllerFault(RuntimeError):
    """Explicit fail-closed controller state (never a silent zero action)."""


@dataclass
class V3ControllerConfig:
    """Frozen engineering-nominal configuration of the RES-85 controller."""

    dt_s: float = M.NATIVE_DT_S
    quiet_stand_samples: int = QUIET_STAND_SAMPLES
    a_thrust_m_s2: float = A_THRUST_M_S2
    thrust_az_max_m_s2: float = THRUST_AZ_MAX_M_S2
    reference_lag_max_m: float = REFERENCE_LAG_MAX_M
    extension_rate_ff_gain: float = EXTENSION_RATE_FF_GAIN
    contact_preload_m: float = CONTACT_PRELOAD_M
    trunk_lean_frac: float = TRUNK_LEAN_FRAC
    trunk_extend_frac: float = TRUNK_EXTEND_FRAC
    trunk_kp: float = TRUNK_KP
    trunk_kd: float = TRUNK_KD
    joint_rom_margin_rad: float = JOINT_ROM_MARGIN_RAD
    joint_rom_barrier_gain: float = JOINT_ROM_BARRIER_GAIN
    joint_rom_velocity_gain: float = JOINT_ROM_VELOCITY_GAIN
    trunk_rom_guard_margin_rad: float = TRUNK_ROM_GUARD_MARGIN_RAD
    trunk_rom_position_gain: float = TRUNK_ROM_POSITION_GAIN
    trunk_rom_velocity_gain: float = TRUNK_ROM_VELOCITY_GAIN
    v_countermovement_cmd_m_s: float = V_COUNTERMOVEMENT_CMD_M_S
    a_brake_m_s2: float = A_BRAKE_M_S2
    v_brake_trigger_m_s: float = V_BRAKE_TRIGGER_M_S
    braking_trigger_margin_m: float = BRAKING_TRIGGER_MARGIN_M
    mtp_active_budget_j: float | None = None
    mtp_moment_ceiling_nm: float | None = None


@dataclass(frozen=True)
class V3ControlStep:
    """Per-native-sample control output and bookkeeping."""

    index: int
    time_s: float
    phase: str
    previous_phase: str
    transition_reason: str
    desired_nm: np.ndarray
    applied_nm: np.ndarray
    actuation: V3AppliedTorque
    q_ref: np.ndarray
    z_ref: float
    v_ref: float
    az_des_m_s2: float
    fz_des_n: float
    fx_des_n: float
    x_cop_des_m: float
    x_cop_clamped_m: float
    cop_clamped: bool
    mtp_ledger: tuple[V3MtpLedgerEntry, V3MtpLedgerEntry]
    quiet_samples: int
    takeoff_candidate_index: int | None
    takeoff_confirmed: bool


@dataclass
class V3ControllerDiagnostics:
    s_table: np.ndarray
    z_table: np.ndarray
    z_stand_m: float
    z_struct_min_m: float
    s_struct: float
    x_stand_m: float
    q_stand: np.ndarray
    descent_budget_m: float
    faults: list[str] = field(default_factory=list)


class V3LaunchController:
    """Causal loaded-CMJ launch/flight controller on the sealed V3 Plant."""

    def __init__(self, plant: V3Plant, data: mujoco.MjData, *,
                 config: V3ControllerConfig | None = None) -> None:
        self.plant = plant
        self.config = config or V3ControllerConfig()
        self.actuation = V3ActuationAuthority(
            self.config.dt_s,
            mtp_active_positive_work_budget_j=self.config.mtp_active_budget_j,
            mtp_moment_ceiling_nm=self.config.mtp_moment_ceiling_nm)
        self._mj = mujoco.MjData(plant.model)
        self._joint_names = list(CHANNELS)
        self._dof = [int(plant.idx.vadr[name]) for name in self._joint_names]
        self._qadr = [int(plant.idx.qadr[name]) for name in self._joint_names]
        self._anchor_index = {name: int(plant.idx.joint[name]) for name in self._joint_names}
        self.reset(data)

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def reset(self, data: mujoco.MjData) -> None:
        self.actuation.reset()
        self.phase = V3Phase.STAND
        self._step_index = 0
        self._quiet_samples = 0
        self._v_ref = 0.0
        self._q_hold = np.zeros(N_CHANNELS, dtype=np.float64)
        self._pending_candidate: M.V3TakeoffOccurrence | None = None
        self._candidate_rejected: list[str] = []
        self._candidate_history: list[dict[str, object]] = []
        self._accepted_occurrence: M.V3TakeoffOccurrence | None = None
        self._takeoff_confirmed = False
        self._confirmation: M.V3TakeoffConfirmation | None = None
        self._prev_frame: M.V3NativeFrame | None = None
        self._fault_reason: str | None = None
        self._last_supported_phase = V3Phase.STAND
        self._last_support_interval: tuple[float, float] | None = None
        self._transition_reason = ""
        self._mtp_ledger: tuple[V3MtpLedgerEntry, V3MtpLedgerEntry] = (
            V3MtpLedgerEntry(), V3MtpLedgerEntry())

        q0 = np.array([data.qpos[a] for a in self._qadr], dtype=np.float64)
        x_stand = float(M.system_com_state(self.plant, data).com_world_m[0])
        z_stand = float(M.system_com_state(self.plant, data).com_world_m[2])
        s_struct = self._structural_flexion_bound()
        s_table, z_table = self._calibrate_z_of_s(data, q0, s_struct)
        self.diag = V3ControllerDiagnostics(
            s_table=s_table,
            z_table=z_table,
            z_stand_m=z_stand,
            z_struct_min_m=float(z_table.min()),
            s_struct=s_struct,
            x_stand_m=x_stand,
            q_stand=q0,
            descent_budget_m=float(z_stand - z_table.min()),
        )
        self._z_ref = z_stand
        self._q_hold = q0.copy()
        # family-consistent joint rate per unit SYSTEM_COM rise rate: the
        # sagittal extension rate that keeps the planted foot on the floor at
        # the measured SYSTEM_COM vertical velocity (calibrated Plant geometry).
        self._ds_dz_table = np.gradient(s_table, z_table)
        self._s_depth_tracked = 0.0
        # declared joint-ROM barrier arrays in the actuated channel order
        self._rom_lo = np.asarray([
            -np.inf if V3_JOINT_RANGES_RAD[name] is None else V3_JOINT_RANGES_RAD[name][0]
            for name in self._joint_names], dtype=np.float64)
        self._rom_hi = np.asarray([
            np.inf if V3_JOINT_RANGES_RAD[name] is None else V3_JOINT_RANGES_RAD[name][1]
            for name in self._joint_names], dtype=np.float64)

    # ------------------------------------------------------------------
    # plant-derived calibration (geometry only, no controller claim)
    # ------------------------------------------------------------------
    def _structural_flexion_bound(self) -> float:
        """Flexion coordinate bound implied by the sealed Plant joint ROM.

        With the symmetric mapping (hip = ankle = knee/2) the binding joint
        limits are the ankle dorsiflexion ROM and the knee ROM.  The bound is a
        *structural bound* on the reference; it is never a movement target.
        """
        ankle = V3_JOINT_RANGES_RAD["left_ankle"][1]
        knee = V3_JOINT_RANGES_RAD["left_knee"][1]
        hip = V3_JOINT_RANGES_RAD["left_hip"][1]
        s = min(2.0 * ankle, knee, 2.0 * hip)
        return float(max(s - STRUCTURAL_FLEXION_SAFETY_RAD, 0.1))

    def _flexion_reference(self, s: float) -> np.ndarray:
        ref = self.diag.q_stand + FLEXION_DIRECTION * float(s)
        depth = max(0.0, self._s_depth_tracked - float(s))
        ref[0] = (ref[0] + self.config.trunk_lean_frac * float(s)
                  - self.config.trunk_extend_frac * depth)
        return ref

    def _calibrate_z_of_s(self, data: mujoco.MjData, q_stand: np.ndarray,
                          s_struct: float) -> tuple[np.ndarray, np.ndarray]:
        """SYSTEM_COM height along the sagittal flexion family (Plant geometry).

        Pure forward kinematics on a scratch MjData: the pose family is set,
        dropped to the floor and positioned at the standing root x.  The table
        is used for the structural descent budget and for kinematic leg-length
        following; it is not a movement target.
        """
        # monotone flexion family (s >= 0): z(s) decreases with s, so the
        # inverse used for kinematic following is well posed.  s < 0 (leg
        # extension beyond the settled stance) is clipped to s = 0.
        s_table = np.linspace(0.0, s_struct, 72)
        z_table = np.zeros_like(s_table)
        x_stand = float(M.system_com_state(self.plant, data).com_world_m[0])
        for i, s in enumerate(s_table):
            self.plant.reset(self._mj)
            ref = q_stand + FLEXION_DIRECTION * float(s)
            angles = {name: float(ref[k]) for k, name in enumerate(self._joint_names)}
            self.plant.set_joint_angles(self._mj, angles)
            self.plant.drop_to_floor(self._mj, clearance_m=0.0)
            self._mj.qpos[self.plant.idx.qadr["root_tx"]] = x_stand
            mujoco.mj_forward(self.plant.model, self._mj)
            z_table[i] = float(M.system_com_state(self.plant, self._mj).com_world_m[2])
        return s_table, z_table

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _s_from_z(self, z: float) -> float:
        z_clamped = float(np.clip(z, self.diag.z_table[-1], self.diag.z_table[0]))
        return float(np.interp(z_clamped, self.diag.z_table[::-1], self.diag.s_table[::-1]))

    def _q_ref_kinematic(self, z: float) -> np.ndarray:
        s = float(np.clip(self._s_from_z(z - self.config.contact_preload_m),
                          0.0, self.diag.s_table[-1]))
        self._s_depth_tracked = max(self._s_depth_tracked, s)
        return self._flexion_reference(s)

    def _family_rate_reference(self, z: float, vz: float,
                               extending: bool = False) -> np.ndarray:
        """Family-consistent sagittal joint rate for the measured COM rise.

        ``dq_ref/dz`` is the calibrated Plant pose-family derivative; the trunk
        channel follows the declared trunk-lean mapping.  Applied only through
        the declared extension-rate feedforward gain.
        """
        z_clamped = float(np.clip(z, self.diag.z_table[-1], self.diag.z_table[0]))
        ds_dz = float(np.interp(z_clamped, self.diag.z_table[::-1],
                                self._ds_dz_table[::-1]))
        rate_s = ds_dz * float(vz)
        ref_rate = FLEXION_DIRECTION * rate_s
        trunk_slope = self.config.trunk_lean_frac + (
            self.config.trunk_extend_frac if extending else 0.0)
        ref_rate[0] = ref_rate[0] + trunk_slope * rate_s
        return ref_rate

    def _rom_guard(self, q: np.ndarray, qdot: np.ndarray,
                   desired: np.ndarray) -> np.ndarray:
        """State-causal predictive structural-ROM guard on the measured state.

        Two declared, additive, one-sided terms per bounded actuated joint:

        * POSITION_GUARD: exactly zero until the measured coordinate enters the
          declared margin band inside a frozen structural limit, then opposes
          only motion toward that limit;
        * OUTWARD_VELOCITY_BRAKING: within the declared brake zone (the larger
          of ``JOINT_ROM_BRAKE_ZONE_RAD`` and the declared stopping horizon
          ``|qdot| * ROM_BRAKE_HORIZON_S``), the measured outward velocity is
          opposed with ``velocity_gain * qdot``, so the moment begins before
          the coordinate approaches the bound and can be rate-ramped by the
          actuation layer in time.

        Both terms are zero in the interior, never have a sign that propels the
        joint toward the limit they protect, and are pure desired-moment
        corrections: the Plant state is never written and the actuation
        authority still owns the applied moment.
        """
        margin = np.full(N_CHANNELS, float(self.config.joint_rom_margin_rad))
        margin[0] = float(self.config.trunk_rom_guard_margin_rad)
        pos_gain = np.full(N_CHANNELS, float(self.config.joint_rom_barrier_gain))
        pos_gain[0] = float(self.config.trunk_rom_position_gain)
        vel_gain = np.full(N_CHANNELS, float(self.config.joint_rom_velocity_gain))
        vel_gain[0] = float(self.config.trunk_rom_velocity_gain)

        upper = self._rom_hi - margin
        lower = self._rom_lo + margin
        out = desired.copy()
        over = q > upper
        under = q < lower
        out[over] -= pos_gain[over] * (q[over] - upper[over])
        out[under] += pos_gain[under] * (lower[under] - q[under])

        gap_hi = upper - q
        zone_hi = np.maximum(np.abs(qdot) * ROM_BRAKE_HORIZON_S,
                             JOINT_ROM_BRAKE_ZONE_RAD)
        frac_hi = np.clip(1.0 - gap_hi / zone_hi, 0.0, 1.0)
        out -= vel_gain * np.maximum(qdot, 0.0) * frac_hi

        gap_lo = q - lower
        zone_lo = np.maximum(np.abs(qdot) * ROM_BRAKE_HORIZON_S,
                             JOINT_ROM_BRAKE_ZONE_RAD)
        frac_lo = np.clip(1.0 - gap_lo / zone_lo, 0.0, 1.0)
        out += vel_gain * np.maximum(-qdot, 0.0) * frac_lo
        return out

    def _joint_limit_margin(self, data: mujoco.MjData) -> np.ndarray:
        out = []
        for name in V3_JOINT_NAMES:
            rng = V3_JOINT_RANGES_RAD[name]
            if rng is None:
                out.append(float("inf"))
            else:
                q = float(data.qpos[self.plant.idx.qadr[name]])
                out.append(min(q - rng[0], rng[1] - q))
        return np.asarray(out, dtype=np.float64)

    @property
    def passive_reconstruction_residual_nm(self) -> float:
        """|Plant passive MTP generalised force - FM-09 reconstruction| (N*m)."""
        return float(getattr(self, "_passive_reconstruction_residual_nm", 0.0))

    @property
    def accepted_occurrence(self) -> M.V3TakeoffOccurrence | None:
        """The unique controller-accepted TAKEOFF_OCCURRENCE (None if unconfirmed)."""
        return self._accepted_occurrence

    @property
    def confirmation(self) -> M.V3TakeoffConfirmation | None:
        """The last RES-84 confirmation evaluated by the phase machine."""
        return self._confirmation

    @property
    def candidate_history(self) -> list[dict[str, object]]:
        """Controller-side candidate dispositions (PENDING/ACCEPTED/REJECTED)."""
        return [dict(entry) for entry in self._candidate_history]

    def _fault(self, reason: str) -> None:
        self._fault_reason = reason
        raise V3ControllerFault(reason)

    def _validate_frame(self, frame: M.V3NativeFrame) -> None:
        values = (
            frame.index, frame.time_s,
            *frame.com_world_m, *frame.com_velocity_world_m_s,
            frame.left_clearance_m, frame.right_clearance_m,
            frame.legal_plantar_normal_force_n,
            *frame.total_floor_force_world_n,
            *frame.legal_ground_force_world_n,
        )
        if not all(math.isfinite(float(v)) for v in values):
            self._fault("MALFORMED_OBSERVATION_NON_FINITE")
        if not frame.qpos or not frame.qvel:
            self._fault("MALFORMED_OBSERVATION_MISSING_PLANT_STATE")

    def _transition(self, target: V3Phase, reason: str) -> None:
        if target == self.phase:
            return
        allowed = {
            V3Phase.STAND: (V3Phase.COUNTERMOVEMENT,),
            V3Phase.COUNTERMOVEMENT: (V3Phase.BRAKING, V3Phase.TAKEOFF_CONFIRM, V3Phase.STAND),
            V3Phase.BRAKING: (V3Phase.PROPULSION, V3Phase.TAKEOFF_CONFIRM, V3Phase.STAND),
            V3Phase.PROPULSION: (V3Phase.TAKEOFF_CONFIRM, V3Phase.BRAKING),
            V3Phase.TAKEOFF_CONFIRM: (V3Phase.FLIGHT, V3Phase.PROPULSION, V3Phase.BRAKING),
            V3Phase.FLIGHT: (V3Phase.LANDING_PREP,),
            V3Phase.LANDING_PREP: (),
        }
        if target not in allowed[self.phase]:
            self._fault(f"ILLEGAL_PHASE_TRANSITION:{self.phase.value}->{target.value}")
        if self.phase in SUPPORTED_PHASES:
            self._last_supported_phase = self.phase
        self._previous_phase = self.phase
        self.phase = target
        self._phase_entry_index = self._step_index
        self._transition_reason = reason

    # ------------------------------------------------------------------
    # phase machine
    # ------------------------------------------------------------------
    def _update_phase(self, frame: M.V3NativeFrame, snapshot: M.V3MeasurementSnapshot,
                      frames: Sequence[M.V3NativeFrame]) -> None:
        prev = self._prev_frame
        vz = float(frame.com_velocity_world_m_s[2])

        if self.phase == V3Phase.STAND:
            quiet_ok = (
                frame.legal_plantar_active >= 1
                and frame.prohibited_active == 0
                and abs(vz) <= QUIET_VZ_MAX_M_S
                and abs(float(frame.com_velocity_world_m_s[0])) <= QUIET_VX_MAX_M_S
                and snapshot.support_hull.evaluable
                and snapshot.support_hull.sagittal_margin_m is not None
                and snapshot.support_hull.sagittal_margin_m >= QUIET_SAGITTAL_MARGIN_MIN_M
            )
            self._quiet_samples = self._quiet_samples + 1 if quiet_ok else 0
            if self._quiet_samples >= self.config.quiet_stand_samples:
                self._transition(V3Phase.COUNTERMOVEMENT, "QUIET_SUPPORTED_STATE_ESTABLISHED")

        if self.phase == V3Phase.COUNTERMOVEMENT:
            if prev is not None and prev.legal_plantar_active > 0 and frame.legal_plantar_active == 0:
                self._enter_takeoff_confirm(frames, "SUPPORT_LOSS_DURING_COUNTERMOVEMENT")
            else:
                budget = float(frame.com_world_m[2]) - self.diag.z_struct_min_m
                d_stop = (vz * vz) / (2.0 * self.config.a_brake_m_s2)
                if (d_stop >= budget - self.config.braking_trigger_margin_m
                        or vz <= self.config.v_brake_trigger_m_s):
                    self._transition(V3Phase.BRAKING, "PREDICTIVE_BRAKING_TRIGGER")

        if self.phase == V3Phase.BRAKING:
            if prev is not None and prev.legal_plantar_active > 0 and frame.legal_plantar_active == 0:
                self._enter_takeoff_confirm(frames, "SUPPORT_LOSS_DURING_BRAKING")
            elif (vz >= 0.0 and prev is not None
                  and float(prev.com_velocity_world_m_s[2]) < 0.0):
                self._transition(V3Phase.PROPULSION, "PHYSICAL_UPWARD_REVERSAL")

        if self.phase == V3Phase.PROPULSION:
            if prev is not None and prev.legal_plantar_active > 0 and frame.legal_plantar_active == 0:
                self._enter_takeoff_confirm(frames, "RES84_TAKEOFF_OCCURRENCE_CANDIDATE")

        if self.phase == V3Phase.TAKEOFF_CONFIRM:
            self._update_takeoff_confirm(frame, frames)

        if self.phase == V3Phase.FLIGHT:
            vz_prev = float(prev.com_velocity_world_m_s[2]) if prev is not None else vz
            descent = (vz <= 0.0 and vz_prev <= 0.0) or vz <= -0.25
            predicted = vz < -0.05 and (float(frame.com_world_m[2]) / max(-vz, 1e-9)) <= 0.05
            if descent or predicted:
                self._transition(V3Phase.LANDING_PREP, "ACTUAL_DESCENT_OR_PREDICTED_TOUCHDOWN")

        if frame.prohibited_active > 0 and self.phase in SUPPORTED_PHASES:
            self._fault("PROHIBITED_CONTACT_WHILE_SUPPORTED")

    def _enter_takeoff_confirm(self, frames: Sequence[M.V3NativeFrame], reason: str) -> None:
        candidates = M.scan_takeoff_candidates(frames)
        if not candidates:
            return
        self._pending_candidate = candidates[-1]
        self._candidate_history.append({
            "occurrence_index": int(candidates[-1].native_index),
            "disposition": "PENDING",
            "rejection_reason": None,
            "source_phase": self.phase.value,
            "entry_reason": reason,
        })
        self._transition(V3Phase.TAKEOFF_CONFIRM, reason)

    def _record_candidate_disposition(self, disposition: str,
                                      rejection_reason: str | None) -> None:
        for entry in reversed(self._candidate_history):
            if entry["disposition"] == "PENDING":
                entry["disposition"] = disposition
                entry["rejection_reason"] = rejection_reason
                return

    def _update_takeoff_confirm(self, frame: M.V3NativeFrame,
                                frames: Sequence[M.V3NativeFrame]) -> None:
        candidate = self._pending_candidate
        if candidate is None:
            self._fault("TAKEOFF_CONFIRM_WITHOUT_CANDIDATE")
        confirmation = M.confirm_takeoff(frames, candidate)
        self._confirmation = confirmation
        if confirmation.confirmed:
            self._takeoff_confirmed = True
            self._accepted_occurrence = candidate
            self._record_candidate_disposition("ACCEPTED", None)
            self._transition(V3Phase.FLIGHT, "RES84_TAKEOFF_CONFIRMATION")
            return
        failed = set(confirmation.failed_checks)
        if "no_legal_plantar_recontact" in failed or frame.legal_plantar_active > 0:
            self._candidate_rejected.append("LEGAL_RECONTACT_BEFORE_CONFIRMATION")
            self._record_candidate_disposition("REJECTED",
                                               "LEGAL_RECONTACT_BEFORE_CONFIRMATION")
            self._pending_candidate = None
            target = (V3Phase.PROPULSION
                      if float(frame.com_velocity_world_m_s[2]) > 0.0 else V3Phase.BRAKING)
            self._transition(target, "RECONTACT_CANCELS_PROVISIONAL_FLIGHT")
            return
        if "no_prohibited_contact" in failed:
            self._candidate_rejected.append("PROHIBITED_CONTACT_DURING_CONFIRMATION")
            self._record_candidate_disposition("REJECTED",
                                                "PROHIBITED_CONTACT_DURING_CONFIRMATION")
            self._fault("PROHIBITED_CONTACT_DURING_CONFIRMATION")
        if confirmation.dwell is not None and confirmation.dwell.coverage_complete:
            self._candidate_rejected.append("CONFIRMATION_WINDOW_CLOSED_NO_LATCH")
            self._record_candidate_disposition("REJECTED",
                                                "CONFIRMATION_WINDOW_CLOSED_NO_LATCH")

    # ------------------------------------------------------------------
    # control law
    # ------------------------------------------------------------------
    def _virtual_joint_moments(self, data: mujoco.MjData, fz_total: float,
                               fx_total: float, x_cop: float) -> np.ndarray:
        """Net joint moments realizing a desired ground reaction force.

        Static/instantaneous form ``tau = bias - J^T F`` evaluated on the two
        supporting leg chains (hip, knee, ankle, mtp anchors).
        """
        tau = np.array(data.qfrc_bias[self._dof], dtype=np.float64)
        per_foot_fz = 0.5 * fz_total
        per_foot_fx = 0.5 * fx_total
        y_foot = {"left": 0.085, "right": -0.085}
        for side in V3_SIDES:
            force = np.array([per_foot_fx, 0.0, per_foot_fz], dtype=np.float64)
            point = np.array([x_cop, y_foot[side], 0.0], dtype=np.float64)
            for name in ("hip", "knee", "ankle", "mtp"):
                joint_name = f"{side}_{name}"
                jid = self._anchor_index[joint_name]
                anchor = np.asarray(data.xanchor[jid], dtype=np.float64)
                axis = np.asarray(data.xaxis[jid], dtype=np.float64)
                arm = np.cross(axis, point - anchor)
                tau[CHANNELS.index(joint_name)] -= float(np.dot(arm, force))
        return tau

    def _support_interval(self, snapshot: M.V3MeasurementSnapshot
                          ) -> tuple[float, float] | None:
        hull = snapshot.support_hull
        if hull.evaluable and hull.vertices_xy:
            xs = [v[0] for v in hull.vertices_xy]
            return float(min(xs)) + COP_HULL_MARGIN_M, float(max(xs)) - COP_HULL_MARGIN_M
        return self._last_support_interval

    # ------------------------------------------------------------------
    # update
    # ------------------------------------------------------------------
    def update(self, frame: M.V3NativeFrame, snapshot: M.V3MeasurementSnapshot,
               frames: Sequence[M.V3NativeFrame], plant: V3Plant,
               data: mujoco.MjData) -> V3ControlStep:
        self._validate_frame(frame)
        self._step_index = int(frame.index)
        self._transition_reason = ""
        self._previous_phase = self.phase
        self._update_phase(frame, snapshot, frames)

        interval = self._support_interval(snapshot)
        if interval is not None:
            self._last_support_interval = interval

        q = np.array([data.qpos[a] for a in self._qadr], dtype=np.float64)
        qdot = np.array([data.qvel[d] for d in self._dof], dtype=np.float64)
        x = float(frame.com_world_m[0])
        z = float(frame.com_world_m[2])
        vx = float(frame.com_velocity_world_m_s[0])
        vz = float(frame.com_velocity_world_m_s[2])

        az_des = 0.0
        ax_des = -KX_BALANCE * (x - self.diag.x_stand_m) - DX_BALANCE * vx
        fz_total = 0.0
        fx_total = 0.0
        x_cop_des = x
        clipped = x
        cop_clamped = False

        if self.phase in SUPPORTED_PHASES:
            if self.phase == V3Phase.STAND:
                az_des = -KZ_STAND * (z - self.diag.z_stand_m) - DZ_STAND * vz
            elif self.phase == V3Phase.COUNTERMOVEMENT:
                self._v_ref = max(self._v_ref + A_COUNTERMOVEMENT_M_S2 * self.config.dt_s,
                                  self.config.v_countermovement_cmd_m_s)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - self.config.reference_lag_max_m,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (A_COUNTERMOVEMENT_M_S2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            elif self.phase == V3Phase.BRAKING:
                self._v_ref = min(self._v_ref + self.config.a_brake_m_s2 * self.config.dt_s, 0.0)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - self.config.reference_lag_max_m,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (self.config.a_brake_m_s2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            elif self.phase == V3Phase.PROPULSION:
                self._v_ref = min(self._v_ref + self.config.a_thrust_m_s2 * self.config.dt_s,
                                  REFERENCE_V_REF_MAX_M_S)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - self.config.reference_lag_max_m,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (self.config.a_thrust_m_s2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            az_des = float(np.clip(az_des, THRUST_AZ_MIN_M_S2,
                                   self.config.thrust_az_max_m_s2))
            fz_total = max(M.SYSTEM_MASS_KG * (M.GRAVITY_M_S2 + az_des), 0.0)
            fx_total = float(np.clip(M.SYSTEM_MASS_KG * ax_des,
                                     -FX_FRACTION_MAX * fz_total, FX_FRACTION_MAX * fz_total))
            if interval is not None and fz_total > 0.0 and interval[0] <= interval[1]:
                x_lo, x_hi = interval
                x_cop_des = x - z * fx_total / fz_total
                clipped = float(np.clip(x_cop_des, x_lo, x_hi))
                cop_clamped = abs(clipped - x_cop_des) > 1.0e-12
                fx_total = float(np.clip(fz_total * (x - clipped) / z,
                                         -FX_FRACTION_MAX * fz_total,
                                         FX_FRACTION_MAX * fz_total))
            else:
                cop_clamped = fz_total <= 0.0

        # posture reference
        if self.phase == V3Phase.STAND:
            # quiet stance holds the settled standing posture: the leg is a
            # strut and the vertical-force loop is not coupled to a kinematic
            # leg-length follow
            q_ref = self.diag.q_stand.copy()
            self._q_hold = q_ref.copy()
        elif self.phase in SUPPORTED_PHASES:
            q_ref = self._q_ref_kinematic(z)
            self._q_hold = q_ref.copy()
        elif self.phase in (V3Phase.TAKEOFF_CONFIRM, V3Phase.FLIGHT):
            q_ref = self._q_hold.copy()
        else:  # LANDING_PREP: bounded posture preparation, never a snap
            target = self._flexion_reference(
                float(np.clip(self._s_from_z(z), LANDING_PREP_FLEXION,
                              self.diag.s_table[-1])))
            delta = np.clip(target - self._q_hold,
                            -LANDING_PREP_RATE_PER_S * self.config.dt_s,
                            LANDING_PREP_RATE_PER_S * self.config.dt_s)
            self._q_hold = self._q_hold + delta
            q_ref = self._q_hold.copy()

        desired = self._virtual_joint_moments(data, fz_total, fx_total, clipped)
        # declared sagittal gains (the trunk carries its own declared gains so
        # the hip-extension reaction cannot drive it past its ROM)
        kp = JOINT_KP.copy()
        kd = JOINT_KD.copy()
        kp[0] = self.config.trunk_kp
        kd[0] = self.config.trunk_kd
        # extension-rate feedforward: penalize sagittal joint rates that differ
        # from the calibrated family-consistent rate for the measured COM rise
        # (the foot cannot outrun the SYSTEM_COM and lose contact)
        s_now = self._s_from_z(z - self.config.contact_preload_m)
        extending = s_now < self._s_depth_tracked - 1.0e-9
        rate_ref = (self._family_rate_reference(z, vz, extending)
                    * self.config.extension_rate_ff_gain
                    if self.phase in SUPPORTED_PHASES else np.zeros(N_CHANNELS))
        desired = desired - kp * (q - q_ref) - kd * (qdot - rate_ref)
        desired = self._rom_guard(q, qdot, desired)

        mtp_q = (float(data.qpos[plant.idx.qadr["left_mtp"]]),
                 float(data.qpos[plant.idx.qadr["right_mtp"]]))
        mtp_qd = (float(data.qvel[plant.idx.vadr["left_mtp"]]),
                  float(data.qvel[plant.idx.vadr["right_mtp"]]))
        passive = (float(data.qfrc_passive[plant.idx.vadr["left_mtp"]]),
                   float(data.qfrc_passive[plant.idx.vadr["right_mtp"]]))
        passive_reconstruction = plant_mtp_passive_moment_nm(mtp_q, mtp_qd)
        self._passive_reconstruction_residual_nm = max(
            abs(passive[0] - passive_reconstruction[0]),
            abs(passive[1] - passive_reconstruction[1]))
        left_supported = any(r.side == "left" for r in snapshot.records
                             if r.active_legal_plantar)
        right_supported = any(r.side == "right" for r in snapshot.records
                              if r.active_legal_plantar)
        mtp_allowed = (self.phase in SUPPORTED_PHASES and left_supported,
                       self.phase in SUPPORTED_PHASES and right_supported)

        applied, record, ledger = self.actuation.apply(
            desired, qdot, phase=self.phase.value, mtp_active_allowed=mtp_allowed,
            mtp_passive_moment_nm=passive, mtp_ledger=self._mtp_ledger)
        self._mtp_ledger = ledger
        self._prev_frame = frame

        return V3ControlStep(
            index=int(frame.index),
            time_s=float(frame.time_s),
            phase=self.phase.value,
            previous_phase=self._previous_phase.value,
            transition_reason=self._transition_reason,
            desired_nm=desired,
            applied_nm=applied,
            actuation=record,
            q_ref=q_ref,
            z_ref=float(self._z_ref),
            v_ref=float(self._v_ref),
            az_des_m_s2=float(az_des),
            fz_des_n=float(fz_total),
            fx_des_n=float(fx_total),
            x_cop_des_m=float(x_cop_des),
            x_cop_clamped_m=float(clipped),
            cop_clamped=bool(cop_clamped),
            mtp_ledger=ledger,
            quiet_samples=int(self._quiet_samples),
            takeoff_candidate_index=(None if self._pending_candidate is None
                                     else self._pending_candidate.native_index),
            takeoff_confirmed=bool(self._takeoff_confirmed),
        )

    # ------------------------------------------------------------------
    # evidence surface
    # ------------------------------------------------------------------
    def authority_record(self) -> dict[str, object]:
        return {
            "authority_id": V3_CONTROLLER_AUTHORITY_ID,
            "phase_order": [p.value for p in PHASE_ORDER],
            "supported_phases": [p.value for p in SUPPORTED_PHASES],
            "phase_machine_authority": "LCMJ_RES85_PHASE_MACHINE_AUTHORITY_V1",
            "actuation": self.actuation.authority_record(),
            "control_law": {
                "quiet_stand_samples": QUIET_STAND_SAMPLES,
                "a_countermovement_m_s2": A_COUNTERMOVEMENT_M_S2,
                "a_brake_m_s2": self.config.a_brake_m_s2,
                "v_brake_trigger_m_s": self.config.v_brake_trigger_m_s,
                "braking_trigger_margin_m": self.config.braking_trigger_margin_m,
                "v_countermovement_cmd_m_s": self.config.v_countermovement_cmd_m_s,
                "a_thrust_m_s2": self.config.a_thrust_m_s2,
                "thrust_az_bounds_m_s2": [THRUST_AZ_MIN_M_S2,
                                          self.config.thrust_az_max_m_s2],
                "reference_lead_max_m": REFERENCE_LEAD_MAX_M,
                "reference_lag_max_m": self.config.reference_lag_max_m,
                "extension_rate_ff_gain": self.config.extension_rate_ff_gain,
                "contact_preload_m": self.config.contact_preload_m,
                "trunk_lean_frac": self.config.trunk_lean_frac,
                "trunk_extend_frac": self.config.trunk_extend_frac,
                "joint_rom_margin_rad": self.config.joint_rom_margin_rad,
                "joint_rom_barrier_gain": self.config.joint_rom_barrier_gain,
                "joint_rom_velocity_gain": self.config.joint_rom_velocity_gain,
                "trunk_rom_guard_margin_rad": self.config.trunk_rom_guard_margin_rad,
                "trunk_rom_position_gain": self.config.trunk_rom_position_gain,
                "trunk_rom_velocity_gain": self.config.trunk_rom_velocity_gain,
                "rom_brake_horizon_s": ROM_BRAKE_HORIZON_S,
                "joint_rom_brake_zone_rad": JOINT_ROM_BRAKE_ZONE_RAD,
                "rom_guard_structure": "POSITION_GUARD_PLUS_OUTWARD_VELOCITY_BRAKING",
                "trunk_kp": self.config.trunk_kp,
                "trunk_kd": self.config.trunk_kd,
                "joint_kp": [float(v) for v in JOINT_KP],
                "joint_kd": [float(v) for v in JOINT_KD],
                "fx_fraction_max": FX_FRACTION_MAX,
                "cop_hull_margin_m": COP_HULL_MARGIN_M,
                "landing_prep_flexion": LANDING_PREP_FLEXION,
                "structural_flexion_safety_rad": STRUCTURAL_FLEXION_SAFETY_RAD,
            },
            "fault_mode": "FAIL_CLOSED_EXPLICIT_EXCEPTION",
            "scorer_private_memory": "NONE",
        }


__all__ = [
    "LANDING_PREP_FLEXION",
    "PHASE_ORDER",
    "SUPPORTED_PHASES",
    "V3_CONTROLLER_AUTHORITY_ID",
    "V3ControlStep",
    "V3ControllerConfig",
    "V3ControllerDiagnostics",
    "V3ControllerFault",
    "V3LaunchController",
    "V3Phase",
]
