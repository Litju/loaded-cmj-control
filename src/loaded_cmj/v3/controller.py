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
A_THRUST_M_S2 = 8.0
THRUST_AZ_MIN_M_S2 = -4.0
THRUST_AZ_MAX_M_S2 = 12.0
REFERENCE_LEAD_MAX_M = 0.02
REFERENCE_LAG_MAX_M = 0.05
REFERENCE_V_REF_MAX_M_S = 4.0

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
        self.actuation = V3ActuationAuthority(self.config.dt_s)
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
        return self.diag.q_stand + FLEXION_DIRECTION * float(s)

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
        s = float(np.clip(self._s_from_z(z), 0.0, self.diag.s_table[-1]))
        return self._flexion_reference(s)

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
                d_stop = (vz * vz) / (2.0 * A_BRAKE_M_S2)
                if d_stop >= budget - BRAKING_TRIGGER_MARGIN_M or vz <= V_BRAKE_TRIGGER_M_S:
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
        self._transition(V3Phase.TAKEOFF_CONFIRM, reason)

    def _update_takeoff_confirm(self, frame: M.V3NativeFrame,
                                frames: Sequence[M.V3NativeFrame]) -> None:
        candidate = self._pending_candidate
        if candidate is None:
            self._fault("TAKEOFF_CONFIRM_WITHOUT_CANDIDATE")
        confirmation = M.confirm_takeoff(frames, candidate)
        self._confirmation = confirmation
        if confirmation.confirmed:
            self._takeoff_confirmed = True
            self._transition(V3Phase.FLIGHT, "RES84_TAKEOFF_CONFIRMATION")
            return
        failed = set(confirmation.failed_checks)
        if "no_legal_plantar_recontact" in failed or frame.legal_plantar_active > 0:
            self._candidate_rejected.append("LEGAL_RECONTACT_BEFORE_CONFIRMATION")
            self._pending_candidate = None
            target = (V3Phase.PROPULSION
                      if float(frame.com_velocity_world_m_s[2]) > 0.0 else V3Phase.BRAKING)
            self._transition(target, "RECONTACT_CANCELS_PROVISIONAL_FLIGHT")
            return
        if "no_prohibited_contact" in failed:
            self._candidate_rejected.append("PROHIBITED_CONTACT_DURING_CONFIRMATION")
            self._fault("PROHIBITED_CONTACT_DURING_CONFIRMATION")
        if confirmation.dwell is not None and confirmation.dwell.coverage_complete:
            self._candidate_rejected.append("CONFIRMATION_WINDOW_CLOSED_NO_LATCH")

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
                                  V_COUNTERMOVEMENT_CMD_M_S)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - REFERENCE_LAG_MAX_M,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (A_COUNTERMOVEMENT_M_S2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            elif self.phase == V3Phase.BRAKING:
                self._v_ref = min(self._v_ref + A_BRAKE_M_S2 * self.config.dt_s, 0.0)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - REFERENCE_LAG_MAX_M,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (A_BRAKE_M_S2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            elif self.phase == V3Phase.PROPULSION:
                self._v_ref = min(self._v_ref + self.config.a_thrust_m_s2 * self.config.dt_s,
                                  REFERENCE_V_REF_MAX_M_S)
                self._z_ref += self._v_ref * self.config.dt_s
                self._z_ref = float(np.clip(self._z_ref, z - REFERENCE_LAG_MAX_M,
                                            z + REFERENCE_LEAD_MAX_M))
                az_des = (self.config.a_thrust_m_s2
                          + KZ_TRACK * (self._z_ref - z) + DZ_TRACK * (self._v_ref - vz))
            az_des = float(np.clip(az_des, THRUST_AZ_MIN_M_S2, THRUST_AZ_MAX_M_S2))
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
        desired = desired - JOINT_KP * (q - q_ref) - JOINT_KD * qdot

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
                "v_countermovement_cmd_m_s": V_COUNTERMOVEMENT_CMD_M_S,
                "a_brake_m_s2": A_BRAKE_M_S2,
                "v_brake_trigger_m_s": V_BRAKE_TRIGGER_M_S,
                "braking_trigger_margin_m": BRAKING_TRIGGER_MARGIN_M,
                "a_thrust_m_s2": A_THRUST_M_S2,
                "thrust_az_bounds_m_s2": [THRUST_AZ_MIN_M_S2, THRUST_AZ_MAX_M_S2],
                "reference_lead_max_m": REFERENCE_LEAD_MAX_M,
                "reference_lag_max_m": REFERENCE_LAG_MAX_M,
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
