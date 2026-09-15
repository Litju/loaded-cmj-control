"""V3 measurement / contact authority surface (RES-84).

Authority: ``LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1``
Bundle:    ``audit/EXP-RES84-V3-MEASUREMENT-CONTACT-AUTHORITY-001``

This module is the single V3-local surface that answers, for the sealed RES-83
Plant (:mod:`loaded_cmj.v3.plant`):

* what a MuJoCo contact actually *is* at runtime
  (DETECTED vs ACTIVE vs ACTIVE_LEGAL_PLANTAR vs PROHIBITED),
* how contact-frame forces become world-frame wrenches
  (``GROUND_ON_ATHLETE``), aggregate wrenches, CoP, active support hull,
* true foot clearance, orientation, prohibited/penetration state,
* SYSTEM_COM / ATHLETE_COM and force-COM consistency observables,
* native-vs-canonical sampling, signal-processing and event authority
  (TAKEOFF_OCCURRENCE / TAKEOFF_CONFIRMATION / 10 N comparator / apex + H2
  support quantities).

Frozen facts established by RES-84 deterministic probes (see the evidence
bundle; every one is re-executed by ``tests/test_res84_v3_measurement_contact.py``):

* ``mj_contactForce`` returns ``[force, torque]`` in the *contact frame* whose
  row 0 is the contact normal.  The world vector is
  ``frame.reshape(3,3).T @ vector``.  The force acts on the body of the
  *second* contact geom (``mjContact.geom2``); the sign is therefore resolved
  by body identity, never by geom index or ordering.
* ``data.ncon > 0`` does **not** imply force-generating contact.  With the
  sealed Plant (margin = gap = 0) contacts are detected only when penetrating,
  but geometry and solver redundancy still produce detected contacts with
  ``efc_address >= 0`` and *zero* force (state SATISFIED).  An
  ACTIVE_CONSTRAINT_CONTACT requires ``efc_address >= 0`` **and** normal force
  ``> 0.0`` (inactive contacts return a bit-exact zero force vector).
* A known static load of the 99 kg system returns ``Fz = +971.19 N`` on the
  athlete (measured, ``99 * 9.81``), which fixes the published sign convention.
* ``data.cvel`` is MuJoCo's *com-based* spatial velocity anchored at
  ``data.subtree_com[0]`` (the whole-model subtree COM), so the per-body CoM
  velocity is recovered as
  ``cvel_linear + omega x (xipos - subtree_com[0])``.  Validated against
  ``mj_jacSubtreeCom`` and against central finite differences.
* ``mj_step`` leaves ``qpos``/``qvel`` at the new time while the derived arrays
  (``xpos``/``xmat``/``xipos``/``cvel``/``subtree_com``/contacts) still describe
  the pre-integration state.  Streamed frames must therefore be
  internally consistent: sample-then-step, or call ``mj_forward`` after every
  ``mj_step`` before sampling.  RES-84 streams use sample-then-step.

This module contains no controller, no trajectory, no scorer and no RES-85
actuator/phase-machine science.  V1/V2 code is never imported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3.plant import V3Plant

# ===========================================================================
# 1. Authority constants (RES-84 frozen)
# ===========================================================================

V3_MEASUREMENT_AUTHORITY_ID = "LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1"
V3_MEASUREMENT_AUTHORITY_BUNDLE = "audit/EXP-RES84-V3-MEASUREMENT-CONTACT-AUTHORITY-001"
V3_MEASUREMENT_MODEL_ID = C.V3_MODEL_ID

# --- world / plate / wrench frame freeze -----------------------------------
PLATE_FRAME_ID = "WORLD_FRAME"
"""The floor geom is a world plane at z = 0; there is no separate plate body.
The plate frame is therefore the world frame (identity rotation)."""

SUPPORT_PLANE_Z_M = 0.0
WRENCH_REFERENCE_ORIGIN_M = (0.0, 0.0, 0.0)
WRENCH_SIGN_CONVENTION = "GROUND_ON_ATHLETE"
"""Published wrenches are the force/moment applied *to the athlete* by the
ground.  At a settled stance the measured total is (0, 0, +m*g)."""

SYSTEM_MASS_KG = C.V3_SYSTEM_MASS_KG
ATHLETE_MASS_KG = C.V3_ATHLETE_MASS_KG
GRAVITY_M_S2 = C.V3_GRAVITY_M_S2

# --- contact semantics freeze ----------------------------------------------
ACTIVE_FORCE_STRICT_GT_ZERO = True
"""Active-constraint support requires a strictly positive normal force.
Measured: inactive/redundant contacts return a bit-exact 0.0 force vector and
``efc_address`` may still be >= 0 (state SATISFIED)."""

ACTIVE_CONTACT_MATERIAL_FORCE_N = 1.0e-9
"""Telemetry-only material-force floor for reporting.  Classification uses the
strict ``> 0.0`` rule; this constant never gates support, only diagnostics."""

# --- clearance guard (EM-07 / DF-02, sealed by RES-84) ----------------------
CLEARANCE_GUARD_EFFECTIVE_MARGIN_M = 0.0
"""Effective contact margin of the sealed Plant: ``geom_margin`` = 0.0 for
floor and plantar geoms (measured).  RES-84 seals the compiled value; it is
not a physiological claim."""

CLEARANCE_GUARD_MEASURED_MAX_PENETRATION_M = 3.55785e-4
"""Measured maximum contact penetration across the RES-84 declared probes
(flat bilateral stance static equilibrium at system weight; larger of the flat
stance equilibrium and the deterministic fall/landing probe)."""

CLEARANCE_GUARD_PENETRATION_ALLOWANCE_M = 7.1157e-4
"""Verified numerical/penetration allowance = 2 x measured maximum
penetration.  Owner of the safety factor: RES-84.  RES-86 owns the final
solution-verification sweep (DF-08)."""

CLEARANCE_GUARD_M = max(2.0e-3, CLEARANCE_GUARD_EFFECTIVE_MARGIN_M + CLEARANCE_GUARD_PENETRATION_ALLOWANCE_M)
"""``max(0.002 m, effective margin + verified penetration allowance)``.
The 2 mm floor dominates (allowance = 0.71 mm)."""

# --- CoP validity (sealed by RES-84) ----------------------------------------
COP_LOW_FZ_TOLERANCE_N = 1.0e-3
"""Derived low-Fz CoP tolerance.

Derivation (RES-84, recorded in COP_AUTHORITY_AUDIT.json):
  * measured no-contact / inactive-contact force floor: exactly 0.0 N
    (bit-exact zero vector; verified),
  * declared solver force resolution: ``SYSTEM_WEIGHT_N * eps`` =
    2.16e-13 N,
  * horizontal moment scale bound: ``SYSTEM_WEIGHT_N * FOOT_LENGTH`` =
    267.1 N*m,
  * CoP error budget: 1.0e-3 m (``COP_REPORTING_RESOLUTION_M``),
  * conditioning floor ``Fz >= |M| * dFz / budget**2`` evaluated at the
    declared force resolution gives ~2.4e-4 N,
  * sealed value = 1.0e-3 N (4.2x above the derived floor).

It is emphatically *not* the 10 N force-plate comparator convention."""

COP_REPORTING_RESOLUTION_M = 1.0e-3
COP_NUMERICAL_BOUND_M = 2.0
"""A reconstructed CoP further than this from the origin is a numerical
failure (NOT_EVALUABLE_NUMERICAL), not a physical result."""

# --- event authority ---------------------------------------------------------
TAKEOFF_DWELL_S = 0.050
"""Genuine-flight dwell required for TAKEOFF_CONFIRMATION (EM-06)."""

COMPARATOR_FORCE_N = 10.0
COMPARATOR_DWELL_S = 0.010
"""Force-plate comparator only (EM-10).  Never defines physical takeoff."""

# --- sampling / resampling authority ----------------------------------------
NATIVE_DT_S = 0.002
NATIVE_FREQUENCY_HZ = 500.0
NATIVE_STREAM_STATUS = "RAW_NATIVE_EVENT_TRUTH"
CANONICAL_FREQUENCY_HZ = 1000.0
CANONICAL_DT_S = 0.001
CANONICAL_STREAM_STATUS = "DERIVED_UPSAMPLED_NOT_EVENT_TRUTH"
REQUIRED_CANDIDATE_MAX_DT_S = 0.001
"""A candidate claiming >= 1000 Hz temporal resolution must run dt <= 0.001 s.
RES-84 freezes the requirement; RES-86/89/91 own final timestep selection."""

INTERPOLATION_METHOD = "LINEAR_BETWEEN_BRACKETING_NATIVE_SAMPLES"
FILTER_FAMILY = "NONE"
ANTI_ALIAS = "NONE_REQUIRED_FOR_UPSAMPLING"
DOWN_SAMPLING_AUTHORIZED = False
"""RES-84 declares no anti-alias filter; therefore native -> coarser canonical
resampling is not authorized and fails closed."""

DIFFERENTIATION_METHOD = "CENTRAL_FINITE_DIFFERENCE_ON_NATIVE_SAMPLES"
INTEGRATION_METHOD = "TRAPEZOIDAL_ON_NATIVE_SAMPLES"
IMPULSE_INTEGRATION_RULE = "LEFT_RECTANGLE_INTEGRATOR_CONSISTENT"
"""Discrete impulse rule measured to reproduce the native simulator's own
velocity change (`v_{k+1} = v_k + dt (F_k - M g)/M`): MuJoCo's Euler
integrator applies the force evaluated at the beginning of the step."""
NATIVE_POSITION_VELOCITY_HALF_STEP_OFFSET = (
    "CENTRAL_DIFFERENCE(position)[i] = (v_i + v_i+1)/2 = v(t_i + dt/2); offset = +a dt/2"
)
"""Measured property of the native integrator (semi-implicit Euler pairs
``qpos_{i+1} = qpos_i + dt v_{i+1}``): a central position difference returns
the velocity half a native step later, ``v(t + dt/2) = v(t) + a dt/2``.
Native velocity (cvel / Jacobian) is the authoritative velocity; position
differences must never be used as event truth."""
EVENT_INTERPOLATION_METHOD = "LINEAR_CONTACT_FORCE_EXTRAPOLATION_CLAMPED_TO_NATIVE_INTERVAL"


# ===========================================================================
# 2. Contact records and semantics
# ===========================================================================
class V3ContactClass(str, Enum):
    LEGAL_PLANTAR_FLOOR = "LEGAL_PLANTAR_FLOOR"
    PROHIBITED_FLOOR = "PROHIBITED_FLOOR"
    OTHER_FLOOR = "OTHER_FLOOR"
    SELF = "SELF"


@dataclass(frozen=True)
class V3ContactRecord:
    """One decoded MuJoCo contact.

    Naming follows the mission vocabulary: ``geom0``/``body0`` are MuJoCo's
    ``geom1``/``geom1``-body (the *first* contact geom); ``geom1``/``body1``
    are MuJoCo's ``geom2``/``geom2``-body.
    """

    contact_id: int
    geom0: str
    geom1: str
    geom0_id: int
    geom1_id: int
    body0: str
    body1: str
    body0_id: int
    body1_id: int
    contact_class: V3ContactClass
    side: str | None
    region: str | None
    position_world_m: tuple[float, float, float]
    frame: tuple[float, ...]
    dist_m: float
    margin_m: float
    gap_m: float
    includemargin_m: float
    dim: int
    friction: tuple[float, ...]
    mu_slide: float
    efc_address: int
    efc_state: int | None
    detected: bool
    constraint_row_included: bool
    active_constraint: bool
    active_legal_plantar: bool
    prohibited: bool
    force_contact_frame_n: tuple[float, float, float]
    torque_contact_frame_nm: tuple[float, float, float]
    normal_force_n: float
    penetration_m: float
    # raw per-contact solref/solimp as solved
    solref: tuple[float, float]
    solimp: tuple[float, float, float, float, float]

    def force_on_body_world(self, body_id: int) -> np.ndarray | None:
        """World-frame force applied *to* ``body_id`` by this contact (or None)."""
        if body_id not in (self.body0_id, self.body1_id):
            return None
        raw = _contact_frame_vector_to_world(self.frame, self.force_contact_frame_n)
        # Measured convention: the raw force acts on body1 (MuJoCo geom2's body).
        return raw if body_id == self.body1_id else -raw

    def torque_on_body_world(self, body_id: int) -> np.ndarray | None:
        """World-frame contact torque applied *to* ``body_id`` (or None)."""
        if body_id not in (self.body0_id, self.body1_id):
            return None
        raw = _contact_frame_vector_to_world(self.frame, self.torque_contact_frame_nm)
        return raw if body_id == self.body1_id else -raw

    def force_on_ground_side_world(self) -> tuple[np.ndarray, np.ndarray]:
        """(force, torque) applied to the *athlete* side of a floor contact.

        The athlete side is the non-world body; the sign is resolved by body
        identity so no geom ordering can flip the published GRF sign.
        """
        if self.body0_id == 0 == self.body1_id:
            raise ValueError("world-world contact cannot carry a ground wrench")
        target = self.body1_id if self.body1_id != 0 else self.body0_id
        force = self.force_on_body_world(target)
        torque = self.torque_on_body_world(target)
        assert force is not None and torque is not None
        return force, torque


def _contact_frame_vector_to_world(frame: tuple[float, ...], vector: Sequence[float]) -> np.ndarray:
    """Contact frame (row 0 = normal) -> world frame.

    The frame is stored row-major as three row vectors; because the world
    vector is the coordinate expansion in the frame basis, the transform is
    ``frame.T @ vector``.  Validated bit-exactly against ``data.cfrc_ext``
    per body and against a two-box probe with both geom orderings.
    """
    rot = np.asarray(frame, dtype=np.float64).reshape(3, 3)
    return rot.T @ np.asarray(vector, dtype=np.float64)


def _decode_contact(plant: V3Plant, model: mujoco.MjModel, data: mujoco.MjData,
                    contact_id: int) -> V3ContactRecord:
    contact = data.contact[contact_id]
    g0_id, g1_id = int(contact.geom1), int(contact.geom2)
    g0, g1 = plant.geom_name(g0_id), plant.geom_name(g1_id)
    b0_id, b1_id = int(model.geom_bodyid[g0_id]), int(model.geom_bodyid[g1_id])
    b0, b1 = plant.body_name(b0_id), plant.body_name(b1_id)

    if g0 == C.V3_FLOOR_GEOM or g1 == C.V3_FLOOR_GEOM:
        other = g1 if g0 == C.V3_FLOOR_GEOM else g0
        if other in C.V3_PLANTAR_SUPPORT_GEOMS:
            contact_class = V3ContactClass.LEGAL_PLANTAR_FLOOR
            side, region = _SUPPORT_GEOM_TO_SIDE_REGION[other]
        elif other in C.V3_PROHIBITED_FLOOR_GEOMS:
            contact_class = V3ContactClass.PROHIBITED_FLOOR
            side, region = None, None
        else:
            contact_class = V3ContactClass.OTHER_FLOOR
            side, region = None, None
    else:
        contact_class = V3ContactClass.SELF
        side, region = None, None

    force_cf = np.zeros(6, dtype=np.float64)
    mujoco.mj_contactForce(model, data, contact_id, force_cf)
    efc_address = int(contact.efc_address)
    constraint_row_included = efc_address >= 0
    efc_state = int(data.efc_state[efc_address]) if constraint_row_included and efc_address < data.nefc else None
    normal_force = float(force_cf[0])
    active_constraint = bool(constraint_row_included and normal_force > 0.0)

    return V3ContactRecord(
        contact_id=contact_id,
        geom0=g0,
        geom1=g1,
        geom0_id=g0_id,
        geom1_id=g1_id,
        body0=b0,
        body1=b1,
        body0_id=b0_id,
        body1_id=b1_id,
        contact_class=contact_class,
        side=side,
        region=region,
        position_world_m=(float(contact.pos[0]), float(contact.pos[1]), float(contact.pos[2])),
        frame=tuple(float(v) for v in np.asarray(contact.frame, dtype=np.float64).reshape(-1)),
        dist_m=float(contact.dist),
        margin_m=float(model.geom_margin[g0_id]) + float(model.geom_margin[g1_id]),
        gap_m=float(model.geom_gap[g0_id]) + float(model.geom_gap[g1_id]),
        includemargin_m=float(contact.includemargin),
        dim=int(contact.dim),
        friction=tuple(float(v) for v in np.asarray(contact.friction, dtype=np.float64).reshape(-1)),
        mu_slide=float(contact.mu),
        efc_address=efc_address,
        efc_state=efc_state,
        detected=True,
        constraint_row_included=constraint_row_included,
        active_constraint=active_constraint,
        active_legal_plantar=bool(active_constraint and contact_class == V3ContactClass.LEGAL_PLANTAR_FLOOR),
        prohibited=bool(contact_class == V3ContactClass.PROHIBITED_FLOOR),
        force_contact_frame_n=(float(force_cf[0]), float(force_cf[1]), float(force_cf[2])),
        torque_contact_frame_nm=(float(force_cf[3]), float(force_cf[4]), float(force_cf[5])),
        normal_force_n=normal_force,
        penetration_m=max(0.0, -float(contact.dist)),
        solref=(float(contact.solref[0]), float(contact.solref[1])),
        solimp=tuple(float(v) for v in np.asarray(contact.solimp, dtype=np.float64).reshape(-1)),
    )


_SUPPORT_GEOM_TO_SIDE_REGION: dict[str, tuple[str, str]] = {
    C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]: (side, region)
    for side in C.V3_SIDES
    for region in C.V3_SUPPORT_REGIONS
}


def contact_records(plant: V3Plant, data: mujoco.MjData) -> list[V3ContactRecord]:
    """Decode every contact in the runtime buffer (never a plantar-only loop)."""
    return [_decode_contact(plant, plant.model, data, i) for i in range(data.ncon)]


def legal_plantar_records(records: Iterable[V3ContactRecord]) -> list[V3ContactRecord]:
    return [r for r in records if r.contact_class == V3ContactClass.LEGAL_PLANTAR_FLOOR]


def active_legal_plantar_records(records: Iterable[V3ContactRecord]) -> list[V3ContactRecord]:
    return [r for r in records if r.active_legal_plantar]


def prohibited_records(records: Iterable[V3ContactRecord]) -> list[V3ContactRecord]:
    return [r for r in records if r.prohibited]


@dataclass(frozen=True)
class V3ContactState:
    """Aggregate contact-buffer state (prohibited / penetration visible)."""

    total_detected: int
    total_active: int
    legal_plantar_detected: int
    legal_plantar_active: int
    prohibited_detected: int
    prohibited_active: int
    self_contacts_detected: int
    self_contacts_active: int
    other_floor_detected: int
    max_penetration_all_detected_m: float
    max_penetration_legal_m: float
    max_penetration_prohibited_m: float
    prohibited_present: bool
    body_floor_fall_visible: bool

    @property
    def legal_support_active(self) -> bool:
        return self.legal_plantar_active > 0


def contact_state(plant: V3Plant, data: mujoco.MjData) -> V3ContactState:
    records = contact_records(plant, data)
    legal = [r for r in records if r.contact_class == V3ContactClass.LEGAL_PLANTAR_FLOOR]
    prohibited = [r for r in records if r.prohibited]
    self_c = [r for r in records if r.contact_class == V3ContactClass.SELF]
    other = [r for r in records if r.contact_class == V3ContactClass.OTHER_FLOOR]
    return V3ContactState(
        total_detected=len(records),
        total_active=sum(1 for r in records if r.active_constraint),
        legal_plantar_detected=len(legal),
        legal_plantar_active=sum(1 for r in legal if r.active_legal_plantar),
        prohibited_detected=len(prohibited),
        prohibited_active=sum(1 for r in prohibited if r.active_constraint),
        self_contacts_detected=len(self_c),
        self_contacts_active=sum(1 for r in self_c if r.active_constraint),
        other_floor_detected=len(other),
        max_penetration_all_detected_m=max((r.penetration_m for r in records), default=0.0),
        max_penetration_legal_m=max((r.penetration_m for r in legal), default=0.0),
        max_penetration_prohibited_m=max((r.penetration_m for r in prohibited), default=0.0),
        prohibited_present=bool(prohibited),
        body_floor_fall_visible=bool(prohibited),
    )


# ===========================================================================
# 3. Wrench aggregation
# ===========================================================================
@dataclass(frozen=True)
class V3Wrench:
    force_world_n: tuple[float, float, float]
    moment_world_nm: tuple[float, float, float]
    reference_origin_m: tuple[float, float, float]
    contact_count: int
    normal_force_n: float

    def as_array(self) -> np.ndarray:
        return np.array([*self.force_world_n, *self.moment_world_nm], dtype=np.float64)


def _wrench_from_records(records: Sequence[V3ContactRecord], origin: Sequence[float]) -> V3Wrench:
    force = np.zeros(3)
    moment = np.zeros(3)
    normal = 0.0
    origin_v = np.asarray(origin, dtype=np.float64)
    for record in records:
        raw_force, raw_torque = record.force_on_ground_side_world()
        p = np.asarray(record.position_world_m, dtype=np.float64)
        force += raw_force
        moment += raw_torque + np.cross(p - origin_v, raw_force)
        normal += record.normal_force_n
    return V3Wrench(
        force_world_n=(float(force[0]), float(force[1]), float(force[2])),
        moment_world_nm=(float(moment[0]), float(moment[1]), float(moment[2])),
        reference_origin_m=(float(origin_v[0]), float(origin_v[1]), float(origin_v[2])),
        contact_count=len(records),
        normal_force_n=float(normal),
    )


def total_ground_wrench(plant: V3Plant, data: mujoco.MjData) -> V3Wrench:
    """Aggregate of every ACTIVE legal plantar contact, at the declared origin."""
    records = active_legal_plantar_records(contact_records(plant, data))
    return _wrench_from_records(records, WRENCH_REFERENCE_ORIGIN_M)


def foot_wrench(plant: V3Plant, data: mujoco.MjData, side: str) -> V3Wrench:
    if side not in C.V3_SIDES:
        raise ValueError(f"unknown side {side!r}")
    records = [r for r in active_legal_plantar_records(contact_records(plant, data)) if r.side == side]
    return _wrench_from_records(records, WRENCH_REFERENCE_ORIGIN_M)


def ground_reaction_wrench(plant: V3Plant, data: mujoco.MjData, include_prohibited: bool = True) -> V3Wrench:
    """What a force plate under the floor would read.

    Includes every ACTIVE floor contact (legal plantar and, when
    ``include_prohibited`` is set, prohibited body/bar-floor contacts) so that a
    fall is never invisible in the force channel.
    """
    records = []
    for r in contact_records(plant, data):
        if not r.active_constraint:
            continue
        if r.contact_class == V3ContactClass.LEGAL_PLANTAR_FLOOR:
            records.append(r)
        elif include_prohibited and r.contact_class == V3ContactClass.PROHIBITED_FLOOR:
            records.append(r)
    return _wrench_from_records(records, WRENCH_REFERENCE_ORIGIN_M)


@dataclass(frozen=True)
class V3OutOfPlaneObservables:
    """Out-of-plane reaction observables (RES-84 exposes, does not threshold)."""

    total_fy_n: float
    total_mx_nm: float
    total_mz_nm: float
    left_fy_n: float
    left_mx_nm: float
    left_mz_nm: float
    right_fy_n: float
    right_mx_nm: float
    right_mz_nm: float
    fy_over_fz: float
    mx_over_fz_m: float
    mz_over_fz_m: float
    structural_note: str


def out_of_plane_observables(plant: V3Plant, data: mujoco.MjData) -> V3OutOfPlaneObservables:
    total = total_ground_wrench(plant, data)
    left = foot_wrench(plant, data, "left")
    right = foot_wrench(plant, data, "right")
    fz = total.force_world_n[2]
    scale = fz if fz > COP_LOW_FZ_TOLERANCE_N else 0.0

    def _ratio(value: float) -> float:
        return float(value / scale) if scale > 0.0 else float("nan")

    return V3OutOfPlaneObservables(
        total_fy_n=total.force_world_n[1],
        total_mx_nm=total.moment_world_nm[0],
        total_mz_nm=total.moment_world_nm[2],
        left_fy_n=left.force_world_n[1],
        left_mx_nm=left.moment_world_nm[0],
        left_mz_nm=left.moment_world_nm[2],
        right_fy_n=right.force_world_n[1],
        right_mx_nm=right.moment_world_nm[0],
        right_mz_nm=right.moment_world_nm[2],
        fy_over_fz=_ratio(total.force_world_n[1]),
        mx_over_fz_m=_ratio(total.moment_world_nm[0]),
        mz_over_fz_m=_ratio(total.moment_world_nm[2]),
        structural_note=(
            "root y translation, roll and yaw are structurally removed from the "
            "V3 Plant; these observables expose whatever out-of-plane reaction "
            "information exists at runtime and carry no balance-authority claim."
        ),
    )


# ===========================================================================
# 4. Centre of pressure
# ===========================================================================
class V3CopValidity(str, Enum):
    VALID = "VALID"
    NOT_EVALUABLE_LOW_FZ = "NOT_EVALUABLE_LOW_FZ"
    NOT_EVALUABLE_NO_SUPPORT = "NOT_EVALUABLE_NO_SUPPORT"
    NOT_EVALUABLE_FLIGHT = "NOT_EVALUABLE_FLIGHT"
    NOT_EVALUABLE_PROHIBITED_CONTACT = "NOT_EVALUABLE_PROHIBITED_CONTACT"
    NOT_EVALUABLE_NUMERICAL = "NOT_EVALUABLE_NUMERICAL"


@dataclass(frozen=True)
class V3CopResult:
    validity: V3CopValidity
    cop_x_m: float | None
    cop_y_m: float | None
    free_vertical_moment_nm: float | None
    normal_force_n: float | None
    reference_origin_m: tuple[float, float, float]
    support_plane_z_m: float
    note: str = ""


def cop_from_wrench(
    wrench: V3Wrench,
    *,
    support_plane_z_m: float = SUPPORT_PLANE_Z_M,
    flight_context: bool = False,
    prohibited_active: bool = False,
    has_legal_support: bool = True,
) -> V3CopResult:
    """Reconstruct the CoP from the aggregate world wrench.

    With the wrench reference origin ``o = (0, 0, z_o)`` and the support plane
    ``z = 0``, the CoP is the point ``r = (rx, ry, 0)`` where the horizontal
    moment components vanish::

        rx = -(Moy + z_o * Fx) / Fz
        ry =  (Mox - z_o * Fy) / Fz
        Mz_free = Moz - (rx * Fy - ry * Fx)

    The ``z_o`` lever-arm terms are included and validated with synthetic
    wrenches evaluated at a non-zero reference origin.
    """
    fx, fy, fz = wrench.force_world_n
    mox, moy, moz = wrench.moment_world_nm
    ox, oy, oz = wrench.reference_origin_m
    if not has_legal_support:
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_FLIGHT if flight_context
                            else V3CopValidity.NOT_EVALUABLE_NO_SUPPORT, wrench, support_plane_z_m)
    if prohibited_active:
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_PROHIBITED_CONTACT, wrench, support_plane_z_m)
    if not all(math.isfinite(v) for v in (fx, fy, fz, mox, moy, moz, ox, oy, oz)):
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_NUMERICAL, wrench, support_plane_z_m)
    if abs(fz) < COP_LOW_FZ_TOLERANCE_N or fz <= 0.0:
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_LOW_FZ, wrench, support_plane_z_m)
    z_o = float(oz) - float(support_plane_z_m)
    rx = -(moy + z_o * fx) / fz
    ry = (mox - z_o * fy) / fz
    if not (math.isfinite(rx) and math.isfinite(ry)):
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_NUMERICAL, wrench, support_plane_z_m)
    if math.hypot(rx, ry) > COP_NUMERICAL_BOUND_M:
        return _cop_invalid(V3CopValidity.NOT_EVALUABLE_NUMERICAL, wrench, support_plane_z_m)
    free_mz = moz - (rx * fy - ry * fx)
    return V3CopResult(
        validity=V3CopValidity.VALID,
        cop_x_m=float(rx),
        cop_y_m=float(ry),
        free_vertical_moment_nm=float(free_mz),
        normal_force_n=float(fz),
        reference_origin_m=wrench.reference_origin_m,
        support_plane_z_m=float(support_plane_z_m),
        note="",
    )


def _cop_invalid(validity: V3CopValidity, wrench: V3Wrench, support_plane_z_m: float) -> V3CopResult:
    return V3CopResult(
        validity=validity,
        cop_x_m=None,
        cop_y_m=None,
        free_vertical_moment_nm=None,
        normal_force_n=float(wrench.force_world_n[2]),
        reference_origin_m=wrench.reference_origin_m,
        support_plane_z_m=float(support_plane_z_m),
    )


def cop_from_plant(plant: V3Plant, data: mujoco.MjData,
                   *, flight_context: bool = False) -> V3CopResult:
    records = contact_records(plant, data)
    legal_active = [r for r in records if r.active_legal_plantar]
    prohibited_active = [r for r in records if r.prohibited and r.active_constraint]
    wrench = _wrench_from_records(legal_active, WRENCH_REFERENCE_ORIGIN_M)
    return cop_from_wrench(
        wrench,
        flight_context=flight_context,
        prohibited_active=bool(prohibited_active),
        has_legal_support=bool(legal_active),
    )


# ===========================================================================
# 5. Active support hull and support margin
# ===========================================================================
class V3SupportMode(str, Enum):
    NOT_EVALUABLE = "NOT_EVALUABLE"
    NONE = "NONE"
    LEFT_ONLY = "LEFT_ONLY"
    RIGHT_ONLY = "RIGHT_ONLY"
    BILATERAL = "BILATERAL"


@dataclass(frozen=True)
class V3SupportHull:
    evaluable: bool
    support_mode: V3SupportMode
    regions: tuple[tuple[str, str], ...]
    vertices_xy: tuple[tuple[float, float], ...]
    area_m2: float | None
    centroid_xy: tuple[float, float] | None
    sagittal_margin_m: float | None
    planar_margin_m: float | None
    lateral_margin_m: float | None
    com_projection_xy: tuple[float, float] | None
    sagittal_claim: str = (
        "1-D sagittal margin: signed x-distance from the SYSTEM_COM ground "
        "projection to the active support interval."
    )
    lateral_claim: str = (
        "Lateral (y) margin is model geometry only; V3 suppresses root y "
        "translation, roll and yaw, so it carries no lateral balance authority."
    )


SUPPORT_FOOTPRINT_TOLERANCE_M = 5.0e-4
"""Corner-selection band for the active support footprint: a patch corner is
part of the support face when its clearance is within this band of the patch's
lowest corner.  Derived from the measured settled-stance penetration
(3.56e-4 m) with margin; a tilted patch exceeding the band contributes only
its deepest corner(s), which is the correct rigid-box behaviour."""


def _patch_corner_table(plant: V3Plant, data: mujoco.MjData,
                        ) -> dict[tuple[str, str], tuple[np.ndarray, list[np.ndarray]]]:
    """World corners for every legal plantar patch, with body-frame offsets."""
    table: dict[tuple[str, str], tuple[np.ndarray, list[np.ndarray]]] = {}
    for side in C.V3_SIDES:
        for region in C.V3_SUPPORT_REGIONS:
            geom_id = plant.idx.geom[C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]
            size = np.asarray(plant.model.geom_size[geom_id], dtype=np.float64)
            rot = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
            pos = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
            offsets = []
            world = []
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    for sz in (-1.0, 1.0):
                        local = np.array([sx, sy, sz]) * size
                        offsets.append(local)
                        world.append(pos + rot @ local)
            table[(side, region)] = (np.asarray(world), offsets)
    return table


def active_support_hull(plant: V3Plant, data: mujoco.MjData,
                        *, flight_context: bool = False) -> V3SupportHull:
    """Support geometry from *currently active legal plantar support* only."""
    records = active_legal_plantar_records(contact_records(plant, data))
    if not records:
        return V3SupportHull(
            evaluable=False,
            support_mode=V3SupportMode.NOT_EVALUABLE if flight_context else V3SupportMode.NONE,
            regions=(),
            vertices_xy=(),
            area_m2=None,
            centroid_xy=None,
            sagittal_margin_m=None,
            planar_margin_m=None,
            lateral_margin_m=None,
            com_projection_xy=None,
        )
    active_regions = sorted({(r.side, r.region) for r in records})
    table = _patch_corner_table(plant, data)
    points: list[tuple[float, float]] = []
    for key in active_regions:
        world, _ = table[key]
        z_min = float(world[:, 2].min())
        selected = world[world[:, 2] <= z_min + SUPPORT_FOOTPRINT_TOLERANCE_M]
        for p in selected:
            points.append((float(p[0]), float(p[1])))
    vertices = _convex_hull_2d(points)
    signed_area = _signed_area(vertices) if len(vertices) >= 3 else 0.0
    area = abs(signed_area)
    centroid = _polygon_centroid(vertices, signed_area) if area > 0.0 else (
        (float(np.mean([p[0] for p in points])), float(np.mean([p[1] for p in points]))) if points else None)
    com = system_com_state(plant, data).com_world_m
    com_xy = (float(com[0]), float(com[1]))
    sides = {r.side for r in records}
    mode = V3SupportMode.BILATERAL if len(sides) == 2 else (
        V3SupportMode.LEFT_ONLY if "left" in sides else V3SupportMode.RIGHT_ONLY)
    return V3SupportHull(
        evaluable=True,
        support_mode=mode,
        regions=tuple(active_regions),
        vertices_xy=tuple(vertices),
        area_m2=float(area),
        centroid_xy=centroid,
        sagittal_margin_m=_sagittal_margin(com_xy, vertices),
        planar_margin_m=_planar_margin(com_xy, vertices),
        lateral_margin_m=_lateral_margin(com_xy, vertices),
        com_projection_xy=com_xy,
    )


def support_margin_from_point(point_xy: Sequence[float], hull: V3SupportHull,
                              ) -> tuple[float | None, float | None, float | None]:
    """(sagittal, planar, lateral) signed margins for an arbitrary point.

    Returns ``(None, None, None)`` when the hull is not evaluable; a flight
    state can therefore never report a positive support margin.
    """
    if not hull.evaluable or not hull.vertices_xy:
        return None, None, None
    point = (float(point_xy[0]), float(point_xy[1]))
    return (
        _sagittal_margin(point, hull.vertices_xy),
        _planar_margin(point, hull.vertices_xy),
        _lateral_margin(point, hull.vertices_xy),
    )


def _convex_hull_2d(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Counter-clockwise convex hull (monotone chain); collinear points removed."""
    unique = sorted({(float(x), float(y)) for x, y in points})
    if len(unique) <= 2:
        return list(unique)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _signed_area(vertices: Sequence[tuple[float, float]]) -> float:
    area = 0.0
    n = len(vertices)
    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return area / 2.0


def _polygon_centroid(vertices: Sequence[tuple[float, float]], signed_area: float) -> tuple[float, float]:
    if abs(signed_area) <= 0.0:
        xs = [p[0] for p in vertices]
        ys = [p[1] for p in vertices]
        return (float(np.mean(xs)), float(np.mean(ys)))
    cx = 0.0
    cy = 0.0
    n = len(vertices)
    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    return (float(cx / (6.0 * signed_area)), float(cy / (6.0 * signed_area)))


def _sagittal_margin(point: tuple[float, float], vertices: Sequence[tuple[float, float]]) -> float:
    xs = [p[0] for p in vertices]
    return float(min(point[0] - min(xs), max(xs) - point[0]))


def _lateral_margin(point: tuple[float, float], vertices: Sequence[tuple[float, float]]) -> float:
    ys = [p[1] for p in vertices]
    return float(min(point[1] - min(ys), max(ys) - point[1]))


def _planar_margin(point: tuple[float, float], vertices: Sequence[tuple[float, float]]) -> float:
    """Signed distance to a convex region (positive inside, negative outside).

    For a convex polygon the signed distance is the maximum over the edge
    supporting lines of the outward-normal projection; a degenerate hull
    (point or segment) has zero interior and returns a negative distance.
    """
    if not vertices:
        return float("nan")
    if len(vertices) == 1:
        return float(-math.hypot(point[0] - vertices[0][0], point[1] - vertices[0][1]))
    if len(vertices) == 2:
        return float(-_point_segment_distance(point, vertices[0], vertices[1]))
    if _signed_area(vertices) < 0.0:
        vertices = list(reversed(vertices))
    signed_max: float | None = None
    n = len(vertices)
    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        ex, ey = x1 - x0, y1 - y0
        length = math.hypot(ex, ey)
        if length <= 0.0:
            continue
        nx, ny = ey / length, -ex / length  # outward normal for CCW winding
        signed = (point[0] - x0) * nx + (point[1] - y0) * ny
        signed_max = signed if signed_max is None else max(signed_max, signed)
    if signed_max is None:
        return float("nan")
    # max-over-edges is positive outside / negative inside; flip to the
    # "positive inside" support-margin convention.
    return float(-signed_max)


def _point_segment_distance(point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    px, py = point
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = min(max(t, 0.0), 1.0)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


# ===========================================================================
# 6. True foot clearance
# ===========================================================================
@dataclass(frozen=True)
class V3FootClearance:
    side: str
    min_gap_m: float
    governing_body: str
    governing_geom: str
    governing_region: str
    governing_material_point_world_m: tuple[float, float, float]
    governing_material_point_body_frame_m: tuple[float, float, float]
    normal_velocity_m_s: float
    region_gaps_m: tuple[tuple[str, float], ...]


def _body_point_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_id: int,
                         point_world: np.ndarray) -> np.ndarray:
    """Velocity of the material point ``point_world`` on body ``body_id``.

    Measured convention (RES-84 probe): ``data.cvel`` is MuJoCo's *com-based*
    spatial velocity anchored at ``data.subtree_com[0]`` (the whole-model
    subtree COM), i.e. ``cvel[3:]`` is the velocity of the material point at
    that reference, not at the body CoM.  For a rigid body the velocity field
    is ``v(p) = cvel_linear + omega x (p - reference)``.  Validated against
    central finite differences of the same material point in dynamic states.
    """
    cvel = np.asarray(data.cvel[body_id], dtype=np.float64)
    omega = cvel[:3]
    v_reference = cvel[3:]
    reference = np.asarray(data.subtree_com[0], dtype=np.float64)
    return v_reference + np.cross(omega, point_world - reference)


def foot_clearance(plant: V3Plant, data: mujoco.MjData, side: str,
                   ) -> V3FootClearance:
    """True minimum signed floor gap of all relevant plantar material geometry.

    Every plantar patch is a box; for a convex box against the plane ``z = 0``
    the minimum signed distance is attained at a vertex, so the corner scan is
    exact (brute-force surface-sampled validation is part of the evidence).
    """
    if side not in C.V3_SIDES:
        raise ValueError(f"unknown side {side!r}")
    table = _patch_corner_table(plant, data)
    best_gap = math.inf
    best: tuple[str, str, str, np.ndarray, np.ndarray] | None = None
    region_gaps: list[tuple[str, float]] = []
    for region in C.V3_SUPPORT_REGIONS:
        world, offsets = table[(side, region)]
        z = world[:, 2]
        idx = int(np.argmin(z))
        gap = float(z[idx])
        region_gaps.append((region, gap))
        if gap < best_gap:
            geom_name = C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]
            body = plant.body_name(int(plant.model.geom_bodyid[plant.idx.geom[geom_name]]))
            best_gap = gap
            best = (geom_name, body, region, world[idx], offsets[idx])
    assert best is not None
    geom_name, body, region, point_world, offset_body = best
    body_id = int(plant.model.geom_bodyid[plant.idx.geom[geom_name]])
    velocity = _body_point_velocity(plant.model, data, body_id, point_world)
    return V3FootClearance(
        side=side,
        min_gap_m=best_gap,
        governing_body=body,
        governing_geom=geom_name,
        governing_region=region,
        governing_material_point_world_m=(float(point_world[0]), float(point_world[1]), float(point_world[2])),
        governing_material_point_body_frame_m=(float(offset_body[0]), float(offset_body[1]), float(offset_body[2])),
        normal_velocity_m_s=float(velocity[2]),
        region_gaps_m=tuple(region_gaps),
    )


# ===========================================================================
# 7. SYSTEM_COM / ATHLETE_COM and momentum
# ===========================================================================
@dataclass(frozen=True)
class V3MassState:
    mass_kg: float
    com_world_m: tuple[float, float, float]
    com_velocity_world_m_s: tuple[float, float, float]
    linear_momentum_kg_m_s: tuple[float, float, float]


def _mass_state(model: mujoco.MjModel, data: mujoco.MjData, body_ids: Sequence[int],
                mass_kg: float) -> V3MassState:
    masses = np.asarray([model.body_mass[b] for b in body_ids], dtype=np.float64)
    coms = np.asarray([data.xipos[b] for b in body_ids], dtype=np.float64)
    reference = np.asarray(data.subtree_com[0], dtype=np.float64)
    vels = []
    for b in body_ids:
        cvel = np.asarray(data.cvel[b], dtype=np.float64)
        vels.append(cvel[3:] + np.cross(cvel[:3], np.asarray(data.xipos[b], dtype=np.float64) - reference))
    vels = np.asarray(vels, dtype=np.float64)
    com = (masses[:, None] * coms).sum(axis=0) / mass_kg
    velocity = (masses[:, None] * vels).sum(axis=0) / mass_kg
    return V3MassState(
        mass_kg=float(mass_kg),
        com_world_m=(float(com[0]), float(com[1]), float(com[2])),
        com_velocity_world_m_s=(float(velocity[0]), float(velocity[1]), float(velocity[2])),
        linear_momentum_kg_m_s=(float(mass_kg * velocity[0]), float(mass_kg * velocity[1]),
                                float(mass_kg * velocity[2])),
    )


def system_body_ids(plant: V3Plant) -> tuple[int, ...]:
    """All bodies carrying mass (every body except ``world``, in id order)."""
    return tuple(b for b in range(plant.model.nbody) if b != 0 and plant.model.body_mass[b] > 0.0)


def athlete_body_ids(plant: V3Plant) -> tuple[int, ...]:
    bar_id = plant.idx.body["bar"]
    return tuple(b for b in system_body_ids(plant) if b != bar_id)


def system_com_state(plant: V3Plant, data: mujoco.MjData) -> V3MassState:
    body_ids = system_body_ids(plant)
    mass = float(sum(plant.model.body_mass[b] for b in body_ids))
    return _mass_state(plant.model, data, body_ids, mass)


def athlete_com_state(plant: V3Plant, data: mujoco.MjData) -> V3MassState:
    body_ids = athlete_body_ids(plant)
    mass = float(sum(plant.model.body_mass[b] for b in body_ids))
    return _mass_state(plant.model, data, body_ids, mass)


# ===========================================================================
# 8. Orientation
# ===========================================================================
@dataclass(frozen=True)
class V3OrientationState:
    root_pitch_rad: float
    root_pitch_rate_rad_s: float
    root_pitch_from_quaternion_rad: float
    pelvis_quaternion_wxyz: tuple[float, float, float, float]
    trunk_pelvis_relative_pitch_rad: float
    trunk_pelvis_relative_pitch_rate_rad_s: float
    trunk_absolute_pitch_rad: float
    trunk_absolute_pitch_rate_rad_s: float
    hat_pitch_from_quaternion_rad: float
    hat_quaternion_wxyz: tuple[float, float, float, float]
    hat_angular_velocity_world_rad_s: tuple[float, float, float]
    structural_note: str


def _quat_to_matrix(quat: Sequence[float]) -> np.ndarray:
    mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=np.float64))
    return mat.reshape(3, 3)


def _pitch_about_y(rot: np.ndarray) -> float:
    return float(math.atan2(rot[0, 2], rot[0, 0]))


def orientation_state(plant: V3Plant, data: mujoco.MjData) -> V3OrientationState:
    pelvis_quat = tuple(float(v) for v in np.asarray(data.xquat[plant.idx.body["pelvis"]], dtype=np.float64))
    hat_quat = tuple(float(v) for v in np.asarray(data.xquat[plant.idx.body["HAT"]], dtype=np.float64))
    root = float(data.qpos[plant.idx.qadr["root_ry"]])
    root_rate = float(data.qvel[plant.idx.vadr["root_ry"]])
    trunk_rel = float(data.qpos[plant.idx.qadr["trunk_pelvis"]])
    trunk_rel_rate = float(data.qvel[plant.idx.vadr["trunk_pelvis"]])
    return V3OrientationState(
        root_pitch_rad=root,
        root_pitch_rate_rad_s=root_rate,
        root_pitch_from_quaternion_rad=_pitch_about_y(_quat_to_matrix(pelvis_quat)),
        pelvis_quaternion_wxyz=pelvis_quat,
        trunk_pelvis_relative_pitch_rad=trunk_rel,
        trunk_pelvis_relative_pitch_rate_rad_s=trunk_rel_rate,
        trunk_absolute_pitch_rad=root + trunk_rel,
        trunk_absolute_pitch_rate_rad_s=root_rate + trunk_rel_rate,
        hat_pitch_from_quaternion_rad=_pitch_about_y(_quat_to_matrix(hat_quat)),
        hat_quaternion_wxyz=hat_quat,
        hat_angular_velocity_world_rad_s=tuple(
            float(v) for v in np.asarray(data.cvel[plant.idx.body["HAT"]], dtype=np.float64)[:3]),
        structural_note=(
            "root roll and yaw are not degrees of freedom (planar root: tx/tz/ry); "
            "pitch is read from the actual compiled joint state and independently "
            "recovered from the pelvis/HAT quaternions, never assumed identity."
        ),
    )


# ===========================================================================
# 9. Contact parameter authority
# ===========================================================================
@dataclass(frozen=True)
class V3ContactParameterRecord:
    geom: str
    body: str
    collision_class: str
    contype: int
    conaffinity: int
    condim: int
    friction_3: tuple[float, float, float]
    margin_m: float
    gap_m: float
    solref: tuple[float, float]
    solimp: tuple[float, float, float, float, float]


def contact_parameter_inventory(plant: V3Plant) -> tuple[V3ContactParameterRecord, ...]:
    model = plant.model
    records = []
    for name in C.V3_GEOM_NAMES:
        gid = plant.idx.geom[name]
        records.append(V3ContactParameterRecord(
            geom=name,
            body=plant.body_name(int(model.geom_bodyid[gid])),
            collision_class=plant.geom_collision_class(gid),
            contype=int(model.geom_contype[gid]),
            conaffinity=int(model.geom_conaffinity[gid]),
            condim=int(model.geom_condim[gid]),
            friction_3=(float(model.geom_friction[gid][0]), float(model.geom_friction[gid][1]),
                        float(model.geom_friction[gid][2])),
            margin_m=float(model.geom_margin[gid]),
            gap_m=float(model.geom_gap[gid]),
            solref=(float(model.geom_solref[gid][0]), float(model.geom_solref[gid][1])),
            solimp=tuple(float(v) for v in np.asarray(model.geom_solimp[gid], dtype=np.float64).reshape(-1)),
        ))
    return tuple(records)


def contact_parameter_authority(plant: V3Plant) -> dict[str, object]:
    """RES-84 sealed contact-parameter decisions (inventory + provenance)."""
    inventory = contact_parameter_inventory(plant)
    model = plant.model
    return {
        "authority_id": V3_MEASUREMENT_AUTHORITY_ID,
        "condim": {
            "nominal_plantar_floor": C.V3_PLANTAR_FLOOR_CONDIM,
            "nominal_other_contacts": C.V3_OTHER_CONTACT_CONDIM,
            "sensitivity_domain": [3, 4, 6],
            "semantics": "condim 3 = normal + sliding; 4 adds torsional; 6 adds rolling",
            "note": "contact authority is never collapsed to one scalar friction coefficient",
        },
        "sliding_friction": {
            "nominal": C.V3_SLIDING_FRICTION_NOMINAL,
            "sensitivity_domain": list(C.V3_SLIDING_FRICTION_SENSITIVITY),
            "classification": "ENGINEERING_NOMINAL_WITH_SENSITIVITY (CC-06); not a measured universal "
                              "athletic-surface truth",
        },
        "torsional_friction": {
            "declared": 0.0,
            "classification": "RES84_SEALED_NUMERICAL; friction-cone term off for the box-like plantar "
                              "patches; sensitivity obligation recorded",
        },
        "rolling_friction": {
            "declared": 0.0,
            "classification": "ENGINEERING_NOMINAL (CC-08) for box-like planar patches",
        },
        "margin_gap": {
            "margin_m": CLEARANCE_GUARD_EFFECTIVE_MARGIN_M,
            "gap_m": 0.0,
            "provenance": "as-compiled sealed Plant values (MuJoCo 3.8.0 defaults); RES-84 seals the "
                          "effective numeric values used by the clearance guard",
        },
        "solref_solimp": {
            "solref": [0.02, 1.0],
            "solimp": [0.9, 0.95, 0.001, 0.5, 2.0],
            "classification": "PROVISIONAL_NUMERICAL_BASELINE_SEALED_BY_RES84; final solution verification "
                              "belongs to RES-86 (CC-10/CC-11/DF-08)",
        },
        "solver": {
            "solver": int(model.opt.solver),
            "integrator": int(model.opt.integrator),
            "cone": int(model.opt.cone),
            "timestep_s": float(model.opt.timestep),
            "iterations": int(model.opt.iterations),
            "tolerance": float(model.opt.tolerance),
            "provenance": "compiled MuJoCo 3.8.0 defaults; timestep/solver verification is RES-86 (DF-08)",
        },
        "penetration": {
            "measured_max_penetration_m": CLEARANCE_GUARD_MEASURED_MAX_PENETRATION_M,
            "verified_allowance_m": CLEARANCE_GUARD_PENETRATION_ALLOWANCE_M,
            "clearance_guard_m": CLEARANCE_GUARD_M,
            "owner": "RES-84 (numeric); RES-86 solution verification",
        },
        "inventory": [
            {
                "geom": r.geom,
                "body": r.body,
                "class": r.collision_class,
                "contype": r.contype,
                "conaffinity": r.conaffinity,
                "condim": r.condim,
                "friction_3": list(r.friction_3),
                "margin_m": r.margin_m,
                "gap_m": r.gap_m,
                "solref": list(r.solref),
                "solimp": list(r.solimp),
            }
            for r in inventory
        ],
    }


# ===========================================================================
# 10. Native frames, streams, sampling authority
# ===========================================================================
@dataclass(frozen=True)
class V3NativeFrame:
    """One raw native sample (event truth).  Every canonical sample derives
    from these by declared interpolation with bracketing provenance."""

    index: int
    time_s: float
    com_world_m: tuple[float, float, float]
    com_velocity_world_m_s: tuple[float, float, float]
    athlete_com_world_m: tuple[float, float, float]
    left_clearance_m: float
    right_clearance_m: float
    legal_plantar_detected: int
    legal_plantar_active: int
    legal_plantar_normal_force_n: float
    prohibited_detected: int
    prohibited_active: int
    total_floor_force_world_n: tuple[float, float, float]
    legal_ground_force_world_n: tuple[float, float, float]
    legal_ground_moment_world_nm: tuple[float, float, float]
    cop_validity: str
    cop_x_m: float | None
    support_mode: str
    qpos: tuple[float, ...] = ()
    qvel: tuple[float, ...] = ()


def native_frame(plant: V3Plant, data: mujoco.MjData, index: int, time_s: float) -> V3NativeFrame:
    records = contact_records(plant, data)
    legal_active = [r for r in records if r.active_legal_plantar]
    legal = [r for r in legal_active]
    prohibited = [r for r in records if r.prohibited]
    wrench = _wrench_from_records(legal, WRENCH_REFERENCE_ORIGIN_M)
    plate = ground_reaction_wrench(plant, data, include_prohibited=True)
    cop = cop_from_plant(plant, data)
    hull = active_support_hull(plant, data)
    com = system_com_state(plant, data)
    athlete = athlete_com_state(plant, data)
    left = foot_clearance(plant, data, "left")
    right = foot_clearance(plant, data, "right")
    return V3NativeFrame(
        index=index,
        time_s=float(time_s),
        com_world_m=com.com_world_m,
        com_velocity_world_m_s=com.com_velocity_world_m_s,
        athlete_com_world_m=athlete.com_world_m,
        left_clearance_m=left.min_gap_m,
        right_clearance_m=right.min_gap_m,
        legal_plantar_detected=len([r for r in records if r.contact_class == V3ContactClass.LEGAL_PLANTAR_FLOOR]),
        legal_plantar_active=len(legal_active),
        legal_plantar_normal_force_n=float(wrench.normal_force_n),
        prohibited_detected=len(prohibited),
        prohibited_active=sum(1 for r in prohibited if r.active_constraint),
        total_floor_force_world_n=plate.force_world_n,
        legal_ground_force_world_n=wrench.force_world_n,
        legal_ground_moment_world_nm=wrench.moment_world_nm,
        cop_validity=cop.validity.value,
        cop_x_m=cop.cop_x_m,
        support_mode=hull.support_mode.value,
        qpos=tuple(float(v) for v in np.asarray(data.qpos, dtype=np.float64)),
        qvel=tuple(float(v) for v in np.asarray(data.qvel, dtype=np.float64)),
    )


@dataclass(frozen=True)
class V3CanonicalSample:
    index: int
    time_s: float
    native_index0: int
    native_index1: int
    weight: float
    com_world_m: tuple[float, float, float]
    com_velocity_world_m_s: tuple[float, float, float]
    total_floor_force_world_n: tuple[float, float, float]
    legal_ground_force_world_n: tuple[float, float, float]
    left_clearance_m: float
    right_clearance_m: float


@dataclass(frozen=True)
class V3CanonicalStream:
    status: str
    source_status: str
    native_dt_s: float
    native_frequency_hz: float
    canonical_dt_s: float
    canonical_frequency_hz: float
    interpolation: str
    filter_family: str
    anti_alias: str
    boundary_handling: str
    sample_count: int
    samples: tuple[V3CanonicalSample, ...]


def _bracket(frames: Sequence[V3NativeFrame], t_s: float) -> tuple[int, int, float]:
    times = [f.time_s for f in frames]
    if t_s <= times[0]:
        return 0, 0, 0.0
    if t_s >= times[-1]:
        return len(frames) - 1, len(frames) - 1, 0.0
    lo, hi = 0, len(frames) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t_s:
            lo = mid
        else:
            hi = mid
    span = times[hi] - times[lo]
    weight = 0.0 if span <= 0.0 else (t_s - times[lo]) / span
    return lo, hi, float(weight)


def interpolate_state(frames: Sequence[V3NativeFrame], t_s: float) -> tuple[int, int, float, V3NativeFrame]:
    """Declared linear interpolation of the native state channels.

    Returns ``(native_index0, native_index1, weight, interpolated_frame)``.
    Contact sets are never interpolated: the frame keeps the *detected* count
    of the lower bracket and callers must consult the raw bracketing frames.
    """
    if not frames:
        raise ValueError("empty frame sequence")
    i0, i1, w = _bracket(frames, t_s)
    a, b = frames[i0], frames[i1]

    def lerp(x: float, y: float) -> float:
        return float(x + (y - x) * w)

    def lerp3(p: Sequence[float], q: Sequence[float]) -> tuple[float, float, float]:
        return (lerp(p[0], q[0]), lerp(p[1], q[1]), lerp(p[2], q[2]))

    interpolated = V3NativeFrame(
        index=i0,
        time_s=float(t_s),
        com_world_m=lerp3(a.com_world_m, b.com_world_m),
        com_velocity_world_m_s=lerp3(a.com_velocity_world_m_s, b.com_velocity_world_m_s),
        athlete_com_world_m=lerp3(a.athlete_com_world_m, b.athlete_com_world_m),
        left_clearance_m=lerp(a.left_clearance_m, b.left_clearance_m),
        right_clearance_m=lerp(a.right_clearance_m, b.right_clearance_m),
        legal_plantar_detected=a.legal_plantar_detected,
        legal_plantar_active=a.legal_plantar_active,
        legal_plantar_normal_force_n=lerp(a.legal_plantar_normal_force_n, b.legal_plantar_normal_force_n),
        prohibited_detected=a.prohibited_detected,
        prohibited_active=a.prohibited_active,
        total_floor_force_world_n=lerp3(a.total_floor_force_world_n, b.total_floor_force_world_n),
        legal_ground_force_world_n=lerp3(a.legal_ground_force_world_n, b.legal_ground_force_world_n),
        legal_ground_moment_world_nm=lerp3(a.legal_ground_moment_world_nm, b.legal_ground_moment_world_nm),
        cop_validity=a.cop_validity,
        cop_x_m=a.cop_x_m if w == 0.0 else (b.cop_x_m if w == 1.0 else None),
        support_mode=a.support_mode,
    )
    return i0, i1, w, interpolated


def canonical_1000hz_stream(frames: Sequence[V3NativeFrame]) -> V3CanonicalStream:
    """Derived 1000 Hz comparison stream with per-sample native provenance.

    Native 500 Hz < canonical 1000 Hz, therefore the canonical stream is an
    *upsampled* reporting product: ``DERIVED_UPSAMPLED_NOT_EVENT_TRUTH``.
    No filter is applied; no anti-alias is required for upsampling; canonical
    samples are only produced inside the native time span.
    """
    if not frames:
        raise ValueError("empty frame sequence")
    t0, t1 = frames[0].time_s, frames[-1].time_s
    n = int(math.floor((t1 - t0) / CANONICAL_DT_S)) + 1
    samples = []
    for j in range(n):
        t = t0 + j * CANONICAL_DT_S
        i0, i1, w, interp = interpolate_state(frames, t)
        samples.append(V3CanonicalSample(
            index=j,
            time_s=float(t),
            native_index0=i0,
            native_index1=i1,
            weight=w,
            com_world_m=interp.com_world_m,
            com_velocity_world_m_s=interp.com_velocity_world_m_s,
            total_floor_force_world_n=interp.total_floor_force_world_n,
            legal_ground_force_world_n=interp.legal_ground_force_world_n,
            left_clearance_m=interp.left_clearance_m,
            right_clearance_m=interp.right_clearance_m,
        ))
    return V3CanonicalStream(
        status=CANONICAL_STREAM_STATUS,
        source_status=NATIVE_STREAM_STATUS,
        native_dt_s=NATIVE_DT_S,
        native_frequency_hz=NATIVE_FREQUENCY_HZ,
        canonical_dt_s=CANONICAL_DT_S,
        canonical_frequency_hz=CANONICAL_FREQUENCY_HZ,
        interpolation=INTERPOLATION_METHOD,
        filter_family=FILTER_FAMILY,
        anti_alias=ANTI_ALIAS,
        boundary_handling="CANONICAL_GRID_CLIPPED_TO_NATIVE_SPAN; NO_EXTRAPOLATION",
        sample_count=len(samples),
        samples=tuple(samples),
    )


def sampling_authority(dt_s: float, frequency_hz: float) -> dict[str, object]:
    """Declared sampling status for a stream with the given native cadence."""
    is_native_1khz = dt_s <= REQUIRED_CANDIDATE_MAX_DT_S + 1e-15
    return {
        "authority_id": V3_MEASUREMENT_AUTHORITY_ID,
        "native_dt_s": float(dt_s),
        "native_frequency_hz": float(frequency_hz),
        "native_status": NATIVE_STREAM_STATUS if abs(dt_s - NATIVE_DT_S) < 1e-15 else "CALLER_DECLARED_NATIVE",
        "canonical_frequency_hz": CANONICAL_FREQUENCY_HZ,
        "canonical_status": NATIVE_STREAM_STATUS if is_native_1khz else CANONICAL_STREAM_STATUS,
        "interpolation": INTERPOLATION_METHOD,
        "filter_family": FILTER_FAMILY,
        "anti_alias": ANTI_ALIAS,
        "down_sampling_authorized": DOWN_SAMPLING_AUTHORIZED,
        "event_truth": "NATIVE_CONTACT_SET_AND_ACTIVE_CONSTRAINT_STATE",
        "required_candidate_max_dt_s": REQUIRED_CANDIDATE_MAX_DT_S,
        "note": (
            "The RES-83 Plant inherits the MuJoCo 3.8.0 default timestep 0.002 s "
            "(500 Hz).  A derived 1000 Hz stream is NOT native 1000 Hz.  Final "
            "candidate qualification must use dt <= 0.001 s to claim >= 1000 Hz "
            "temporal resolution; RES-86/89/91 own the final selection."
        ),
    }


def signal_processing_authority() -> dict[str, object]:
    """Frozen signal-processing policy: no hidden smoothing anywhere."""
    return {
        "authority_id": V3_MEASUREMENT_AUTHORITY_ID,
        "raw_native_retention": "ALL_NATIVE_SAMPLES_RETAINED",
        "interpolation": INTERPOLATION_METHOD,
        "anti_alias": ANTI_ALIAS,
        "filter_family": FILTER_FAMILY,
        "filter_cutoff": None,
        "filter_order": None,
        "filter_phase": "NONE_NO_FILTER",
        "boundary_handling": "NO_EXTRAPOLATION; CANONICAL_GRID_CLIPPED_TO_NATIVE_SPAN",
        "differentiation": DIFFERENTIATION_METHOD,
        "integration": INTEGRATION_METHOD,
        "impulse_integration_rule": IMPULSE_INTEGRATION_RULE,
        "position_velocity_relationship": NATIVE_POSITION_VELOCITY_HALF_STEP_OFFSET,
        "event_interpolation": EVENT_INTERPOLATION_METHOD,
        "hidden_smoothing_before": {
            "contact_event_detection": "NONE",
            "peak_force": "NONE",
            "landing_rate": "NONE",
            "impulse_integration": "NONE",
        },
        "note": (
            "Primary native event truth does not depend on any reporting filter. "
            "Any future smoothing must be separately justified, named and "
            "declared before use."
        ),
    }


def central_difference(times: Sequence[float], values: Sequence[float]) -> list[float]:
    """Declared native differentiation: central differences, one-sided at ends."""
    n = len(values)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    out = [0.0] * n
    for i in range(n):
        if i == 0:
            out[i] = (values[1] - values[0]) / (times[1] - times[0])
        elif i == n - 1:
            out[i] = (values[-1] - values[-2]) / (times[-1] - times[-2])
        else:
            out[i] = (values[i + 1] - values[i - 1]) / (times[i + 1] - times[i - 1])
    return [float(v) for v in out]


def trapezoidal_integral(times: Sequence[float], values: Sequence[float], i0: int, i1: int) -> float:
    """Declared native integration: trapezoidal over native samples [i0, i1]."""
    total = 0.0
    for i in range(i0, i1):
        dt = times[i + 1] - times[i]
        total += 0.5 * (values[i] + values[i + 1]) * dt
    return float(total)


# ===========================================================================
# 11. Event authority: takeoff occurrence / confirmation / comparator / apex
# ===========================================================================
@dataclass(frozen=True)
class V3TakeoffOccurrence:
    valid: bool
    occurrence_time_s: float | None
    native_index: int | None
    last_support_index: int | None
    bracket_weight: float | None
    interpolated: bool
    com_world_m: tuple[float, float, float] | None
    com_velocity_world_m_s: tuple[float, float, float] | None
    left_clearance_m: float | None
    right_clearance_m: float | None
    legal_plantar_normal_force_n: float | None
    total_floor_force_world_n: tuple[float, float, float] | None
    support_mode_before: str | None
    prohibited_detected_at_bracket: tuple[int, int] | None
    reason: str


def _takeoff_candidate(frames: Sequence[V3NativeFrame], k: int) -> V3TakeoffOccurrence:
    """Build the candidate whose first zero-support sample is ``frames[k]``."""
    last = k - 1
    weight = 1.0
    interpolated = False
    if last >= 1:
        f_prev = frames[last - 1].legal_plantar_normal_force_n
        f_last = frames[last].legal_plantar_normal_force_n
        denom = f_last - f_prev
        if f_last > 0.0 and denom > 0.0:
            weight = min(max(f_last / denom, 0.0), 1.0)
            interpolated = weight < 1.0
    t_star = frames[last].time_s + weight * (frames[k].time_s - frames[last].time_s)
    i0, i1, _, state = interpolate_state(frames, t_star)
    return V3TakeoffOccurrence(
        valid=True,
        occurrence_time_s=float(t_star),
        native_index=k,
        last_support_index=last,
        bracket_weight=float(weight),
        interpolated=bool(interpolated),
        com_world_m=state.com_world_m,
        com_velocity_world_m_s=state.com_velocity_world_m_s,
        left_clearance_m=state.left_clearance_m,
        right_clearance_m=state.right_clearance_m,
        legal_plantar_normal_force_n=state.legal_plantar_normal_force_n,
        total_floor_force_world_n=state.total_floor_force_world_n,
        support_mode_before=frames[last].support_mode,
        prohibited_detected_at_bracket=(frames[i0].prohibited_detected, frames[i1].prohibited_detected),
        reason="SUPPORT_TO_ZERO_TRANSITION",
    )


def scan_takeoff_candidates(frames: Sequence[V3NativeFrame]) -> list[V3TakeoffOccurrence]:
    """Every ACTIVE legal plantar support -> zero support transition, in order.

    A candidate rejected by :func:`confirm_takeoff` is never shifted; a caller
    that needs the *final* transition of a genuine flight scans this list and
    confirms candidates in order.
    """
    return [_takeoff_candidate(frames, k) for k in range(1, len(frames))
            if frames[k].legal_plantar_active == 0 and frames[k - 1].legal_plantar_active > 0]


def detect_takeoff_occurrence(frames: Sequence[V3NativeFrame], *, dt_s: float | None = None) -> V3TakeoffOccurrence:
    """Final ACTIVE legal plantar support -> zero active legal plantar support.

    The occurrence is anchored between the last active native sample ``k-1``
    and the first zero-support native sample ``k``.  The primary timestamp is
    the declared linear contact-force extrapolation
    ``t* = t[k-1] + dt * clamp(F[k-1] / (F[k-1] - F[k-2]), 0, 1)`` (clamped to
    the native interval, never later than ``t[k]``); this uses legal plantar
    *force* only, never clearance.  When the extrapolation precondition fails,
    ``t* = t[k]``.  All recorded state is the declared interpolation of the
    bracketing native samples.

    This returns the first candidate transition; ``TAKEOFF_CONFIRMATION``
    rejects or accepts it, and a rejected candidate must be followed by the
    next entry of :func:`scan_takeoff_candidates`, never by a shifted time.
    """
    if not frames:
        raise ValueError("empty frame sequence")
    del dt_s  # native spacing is taken from the frames themselves
    candidates = scan_takeoff_candidates(frames)
    if candidates:
        return candidates[0]
    return V3TakeoffOccurrence(
        valid=False,
        occurrence_time_s=None,
        native_index=None,
        last_support_index=None,
        bracket_weight=None,
        interpolated=False,
        com_world_m=None,
        com_velocity_world_m_s=None,
        left_clearance_m=None,
        right_clearance_m=None,
        legal_plantar_normal_force_n=None,
        total_floor_force_world_n=None,
        support_mode_before=None,
        prohibited_detected_at_bracket=None,
        reason="NO_SUPPORT_TO_ZERO_TRANSITION_FOUND",
    )


@dataclass(frozen=True)
class V3TakeoffConfirmation:
    confirmed: bool
    occurrence: V3TakeoffOccurrence
    checks: tuple[tuple[str, bool, str], ...]
    window_end_time_s: float | None
    clearance_guard_m: float
    dwell_s: float
    bilateral_clearance_max_m: float | None

    @property
    def failed_checks(self) -> tuple[str, ...]:
        return tuple(name for name, ok, _ in self.checks if not ok)


def confirm_takeoff(
    frames: Sequence[V3NativeFrame],
    occurrence: V3TakeoffOccurrence,
    *,
    dt_s: float | None = None,
    dwell_s: float = TAKEOFF_DWELL_S,
    clearance_guard_m: float = CLEARANCE_GUARD_M,
) -> V3TakeoffConfirmation:
    """Fail-closed confirmation of a candidate occurrence (EM-03/EM-06).

    Checks: zero active legal plantar support persists for >= ``dwell_s`` from
    the occurrence; bilateral clearance reaches the clearance guard inside the
    window; SYSTEM_COM vertical velocity at the occurrence is > 0; no
    prohibited contact is detected inside the window; no legal plantar
    recontact occurs.  A failed confirmation rejects the occurrence and never
    shifts or redefines the timestamp.
    """
    if not occurrence.valid or occurrence.occurrence_time_s is None or occurrence.native_index is None:
        return V3TakeoffConfirmation(
            confirmed=False,
            occurrence=occurrence,
            checks=(("occurrence_valid", False, "no candidate occurrence"),),
            window_end_time_s=None,
            clearance_guard_m=clearance_guard_m,
            dwell_s=dwell_s,
            bilateral_clearance_max_m=None,
        )
    dt = float(dt_s if dt_s is not None else (frames[1].time_s - frames[0].time_s) if len(frames) > 1 else NATIVE_DT_S)
    t_star = occurrence.occurrence_time_s
    k = occurrence.native_index
    t_end = t_star + dwell_s
    window = [f for f in frames if t_star <= f.time_s <= t_end + 1e-12]
    coverage_ok = bool(window) and (window[-1].time_s >= t_end - dt - 1e-12)
    recontact_free = all(f.legal_plantar_active == 0 for f in window)
    max_bilateral = max((min(f.left_clearance_m, f.right_clearance_m) for f in window), default=None)
    clearance_ok = bool(max_bilateral is not None and max_bilateral >= clearance_guard_m)
    prohibited_free = all(f.prohibited_detected == 0 for f in window)
    vz_ok = bool(occurrence.com_velocity_world_m_s is not None and occurrence.com_velocity_world_m_s[2] > 0.0)
    support_zero_at_k = bool(frames[k].legal_plantar_active == 0)
    checks = (
        ("occurrence_valid", True, "candidate derived from a support-to-zero transition"),
        ("zero_support_at_native_index", support_zero_at_k, f"native index {k} has zero active legal plantar support"),
        ("dwell_coverage", coverage_ok, f"window covers >= {dwell_s:.3f} s of native samples"),
        ("no_legal_plantar_recontact", recontact_free, "zero active legal plantar contact throughout the window"),
        ("bilateral_clearance_reaches_guard", clearance_ok,
         f"max bilateral clearance {max_bilateral} vs guard {clearance_guard_m}"),
        ("system_com_vz_positive", vz_ok, "SYSTEM_COM vertical velocity at the occurrence is positive"),
        ("no_prohibited_contact", prohibited_free, "no prohibited floor contact inside the window"),
    )
    return V3TakeoffConfirmation(
        confirmed=all(ok for _, ok, _ in checks),
        occurrence=occurrence,
        checks=checks,
        window_end_time_s=float(t_end),
        clearance_guard_m=clearance_guard_m,
        dwell_s=dwell_s,
        bilateral_clearance_max_m=max_bilateral,
    )


@dataclass(frozen=True)
class V3ComparatorResult:
    triggered: bool
    comparator_time_s: float | None
    offset_s: float | None
    threshold_n: float
    dwell_s: float
    note: str = (
        "Diagnostic/comparability only.  Never defines physical takeoff, flight "
        "or the H2 time origin."
    )


def force_takeoff_comparator(
    frames: Sequence[V3NativeFrame],
    occurrence: V3TakeoffOccurrence,
    *,
    threshold_n: float = COMPARATOR_FORCE_N,
    dwell_s: float = COMPARATOR_DWELL_S,
) -> V3ComparatorResult:
    """Total vertical GRF < ``threshold_n`` continuously for >= ``dwell_s``."""
    if not frames:
        raise ValueError("empty frame sequence")
    dt = frames[1].time_s - frames[0].time_s if len(frames) > 1 else NATIVE_DT_S
    need = int(math.ceil(dwell_s / dt - 1e-12))
    run_start: int | None = None
    for i, f in enumerate(frames):
        below = abs(f.total_floor_force_world_n[2]) < threshold_n
        if below and run_start is None:
            run_start = i
        if not below:
            run_start = None
        if run_start is not None and i - run_start + 1 >= max(need, 1):
            t_cmp = frames[run_start].time_s
            offset = None if occurrence.occurrence_time_s is None else float(t_cmp - occurrence.occurrence_time_s)
            return V3ComparatorResult(
                triggered=True,
                comparator_time_s=float(t_cmp),
                offset_s=offset,
                threshold_n=threshold_n,
                dwell_s=dwell_s,
            )
    return V3ComparatorResult(
        triggered=False,
        comparator_time_s=None,
        offset_s=None,
        threshold_n=threshold_n,
        dwell_s=dwell_s,
    )


@dataclass(frozen=True)
class V3ApexResult:
    evaluable: bool
    apex_time_s: float | None
    com_z_apex_m: float | None
    com_z_takeoff_m: float | None
    h2_support_m: float | None
    takeoff_vz_m_s: float | None
    ballistic_height_m: float | None
    ballistic_cross_check_delta_m: float | None
    interpolation: str
    reason: str = ""


def detect_apex(
    frames: Sequence[V3NativeFrame],
    confirmation: V3TakeoffConfirmation,
    *,
    gravity_m_s2: float = GRAVITY_M_S2,
) -> V3ApexResult:
    """APEX inside confirmed genuine flight: COM_vz + -> non-positive crossing."""
    if not confirmation.confirmed or confirmation.occurrence.occurrence_time_s is None:
        return V3ApexResult(
            evaluable=False,
            apex_time_s=None,
            com_z_apex_m=None,
            com_z_takeoff_m=None,
            h2_support_m=None,
            takeoff_vz_m_s=None,
            ballistic_height_m=None,
            ballistic_cross_check_delta_m=None,
            interpolation=INTERPOLATION_METHOD,
            reason="TAKEOFF_OCCURRENCE_NOT_CONFIRMED",
        )
    k = confirmation.occurrence.native_index
    touchdown = next((i for i in range(k + 1, len(frames)) if frames[i].legal_plantar_active > 0), None)
    if touchdown is None:
        return V3ApexResult(
            evaluable=False,
            apex_time_s=None,
            com_z_apex_m=None,
            com_z_takeoff_m=None,
            h2_support_m=None,
            takeoff_vz_m_s=None,
            ballistic_height_m=None,
            ballistic_cross_check_delta_m=None,
            interpolation=INTERPOLATION_METHOD,
            reason="INCOMPLETE_FLIGHT_STREAM_NO_TOUCHDOWN",
        )
    apex: tuple[int, int, float] | None = None
    for i in range(k, touchdown):
        if frames[i].com_velocity_world_m_s[2] > 0.0 and frames[i + 1].com_velocity_world_m_s[2] <= 0.0:
            vz0 = frames[i].com_velocity_world_m_s[2]
            vz1 = frames[i + 1].com_velocity_world_m_s[2]
            w = 0.0 if vz0 == vz1 else float(vz0 / (vz0 - vz1))
            apex = (i, i + 1, min(max(w, 0.0), 1.0))
            break
    if apex is None:
        return V3ApexResult(
            evaluable=False,
            apex_time_s=None,
            com_z_apex_m=None,
            com_z_takeoff_m=None,
            h2_support_m=None,
            takeoff_vz_m_s=None,
            ballistic_height_m=None,
            ballistic_cross_check_delta_m=None,
            interpolation=INTERPOLATION_METHOD,
            reason="NO_VZ_SIGN_CHANGE_IN_GENUINE_FLIGHT",
        )
    prohibited = any(frames[i].prohibited_detected > 0 for i in range(k, apex[0] + 1))
    if prohibited:
        return V3ApexResult(
            evaluable=False,
            apex_time_s=None,
            com_z_apex_m=None,
            com_z_takeoff_m=None,
            h2_support_m=None,
            takeoff_vz_m_s=None,
            ballistic_height_m=None,
            ballistic_cross_check_delta_m=None,
            interpolation=INTERPOLATION_METHOD,
            reason="PROHIBITED_CONTACT_BEFORE_APEX",
        )
    i0, i1, w = apex
    t_apex = frames[i0].time_s + w * (frames[i1].time_s - frames[i0].time_s)
    z_apex = frames[i0].com_world_m[2] + w * (frames[i1].com_world_m[2] - frames[i0].com_world_m[2])
    vz_takeoff = confirmation.occurrence.com_velocity_world_m_s[2]
    z_takeoff = confirmation.occurrence.com_world_m[2]
    ballistic = vz_takeoff * vz_takeoff / (2.0 * gravity_m_s2)
    h2 = z_apex - z_takeoff
    return V3ApexResult(
        evaluable=True,
        apex_time_s=float(t_apex),
        com_z_apex_m=float(z_apex),
        com_z_takeoff_m=float(z_takeoff),
        h2_support_m=float(h2),
        takeoff_vz_m_s=float(vz_takeoff),
        ballistic_height_m=float(ballistic),
        ballistic_cross_check_delta_m=float(h2 - ballistic),
        interpolation=f"{INTERPOLATION_METHOD}:VZ_ZERO_CROSSING",
    )


def vertical_impulse_between(frames: Sequence[V3NativeFrame], i0: int, i1: int,
                             mass_kg: float = SYSTEM_MASS_KG,
                             rule: str = "LEFT_RECTANGLE_INTEGRATOR_CONSISTENT") -> float:
    """Impulse-momentum cross-check data: integral of (Fz - M g) dt over native samples.

    With ``Fz`` the total vertical ground reaction on the athlete, the change
    in SYSTEM_COM vertical velocity over the interval equals this impulse
    divided by the system mass.

    ``rule``:
      * ``LEFT_RECTANGLE_INTEGRATOR_CONSISTENT`` (default): ``dt * sum_k (Fz_k - M g)``
        for ``k = i0 .. i1-1``.  Measured to reproduce the native simulator's
        own velocity change to ~0.02% because MuJoCo's Euler integrator applies
        the force evaluated at the beginning of each step.
      * ``TRAPEZOIDAL``: ``sum_k 0.5 (Fz_k + Fz_{k+1}) dt``; exact for the
        in-flight (zero force) case but systematically low where the contact
        force changes rapidly within one native step.
    """
    values = [f.total_floor_force_world_n[2] - mass_kg * GRAVITY_M_S2 for f in frames]
    if rule == "TRAPEZOIDAL":
        times = [f.time_s for f in frames]
        return trapezoidal_integral(times, values, i0, i1)
    if rule != "LEFT_RECTANGLE_INTEGRATOR_CONSISTENT":
        raise ValueError(f"unknown integration rule {rule!r}")
    total = 0.0
    for k in range(i0, i1):
        dt = frames[k + 1].time_s - frames[k].time_s
        total += values[k] * dt
    return float(total)


# ===========================================================================
# 12. Convenience snapshot
# ===========================================================================
@dataclass(frozen=True)
class V3MeasurementSnapshot:
    records: tuple[V3ContactRecord, ...]
    contact_state: V3ContactState
    total_ground_wrench: V3Wrench
    left_foot_wrench: V3Wrench
    right_foot_wrench: V3Wrench
    plate_wrench: V3Wrench
    cop: V3CopResult
    support_hull: V3SupportHull
    left_clearance: V3FootClearance
    right_clearance: V3FootClearance
    system_com: V3MassState
    athlete_com: V3MassState
    orientation: V3OrientationState
    out_of_plane: V3OutOfPlaneObservables


def measure(plant: V3Plant, data: mujoco.MjData,
            *, flight_context: bool = False) -> V3MeasurementSnapshot:
    """One deterministic measurement snapshot of the current Plant state."""
    records = tuple(contact_records(plant, data))
    return V3MeasurementSnapshot(
        records=records,
        contact_state=contact_state(plant, data),
        total_ground_wrench=total_ground_wrench(plant, data),
        left_foot_wrench=foot_wrench(plant, data, "left"),
        right_foot_wrench=foot_wrench(plant, data, "right"),
        plate_wrench=ground_reaction_wrench(plant, data, include_prohibited=True),
        cop=cop_from_plant(plant, data, flight_context=flight_context),
        support_hull=active_support_hull(plant, data),
        left_clearance=foot_clearance(plant, data, "left"),
        right_clearance=foot_clearance(plant, data, "right"),
        system_com=system_com_state(plant, data),
        athlete_com=athlete_com_state(plant, data),
        orientation=orientation_state(plant, data),
        out_of_plane=out_of_plane_observables(plant, data),
    )


__all__ = [
    "ACTIVE_CONTACT_MATERIAL_FORCE_N",
    "ACTIVE_FORCE_STRICT_GT_ZERO",
    "ANTI_ALIAS",
    "ATHLETE_MASS_KG",
    "CANONICAL_DT_S",
    "CANONICAL_FREQUENCY_HZ",
    "CANONICAL_STREAM_STATUS",
    "CLEARANCE_GUARD_EFFECTIVE_MARGIN_M",
    "CLEARANCE_GUARD_M",
    "CLEARANCE_GUARD_MEASURED_MAX_PENETRATION_M",
    "CLEARANCE_GUARD_PENETRATION_ALLOWANCE_M",
    "COMPARATOR_DWELL_S",
    "COMPARATOR_FORCE_N",
    "COP_LOW_FZ_TOLERANCE_N",
    "COP_NUMERICAL_BOUND_M",
    "COP_REPORTING_RESOLUTION_M",
    "DIFFERENTIATION_METHOD",
    "DOWN_SAMPLING_AUTHORIZED",
    "EVENT_INTERPOLATION_METHOD",
    "FILTER_FAMILY",
    "GRAVITY_M_S2",
    "IMPULSE_INTEGRATION_RULE",
    "NATIVE_POSITION_VELOCITY_HALF_STEP_OFFSET",
    "INTEGRATION_METHOD",
    "INTERPOLATION_METHOD",
    "NATIVE_DT_S",
    "NATIVE_FREQUENCY_HZ",
    "NATIVE_STREAM_STATUS",
    "PLATE_FRAME_ID",
    "REQUIRED_CANDIDATE_MAX_DT_S",
    "SUPPORT_FOOTPRINT_TOLERANCE_M",
    "SUPPORT_PLANE_Z_M",
    "SYSTEM_MASS_KG",
    "TAKEOFF_DWELL_S",
    "V3ApexResult",
    "V3CanonicalSample",
    "V3CanonicalStream",
    "V3ComparatorResult",
    "V3ContactClass",
    "V3ContactParameterRecord",
    "V3ContactRecord",
    "V3ContactState",
    "V3CopResult",
    "V3CopValidity",
    "V3FootClearance",
    "V3MassState",
    "V3MeasurementSnapshot",
    "V3NativeFrame",
    "V3OrientationState",
    "V3OutOfPlaneObservables",
    "V3SupportHull",
    "V3SupportMode",
    "V3TakeoffConfirmation",
    "V3TakeoffOccurrence",
    "V3Wrench",
    "V3_MEASUREMENT_AUTHORITY_BUNDLE",
    "V3_MEASUREMENT_AUTHORITY_ID",
    "V3_MEASUREMENT_MODEL_ID",
    "WRENCH_REFERENCE_ORIGIN_M",
    "WRENCH_SIGN_CONVENTION",
    "active_legal_plantar_records",
    "active_support_hull",
    "athlete_body_ids",
    "athlete_com_state",
    "canonical_1000hz_stream",
    "central_difference",
    "confirm_takeoff",
    "contact_parameter_authority",
    "contact_parameter_inventory",
    "contact_records",
    "contact_state",
    "cop_from_plant",
    "cop_from_wrench",
    "detect_apex",
    "detect_takeoff_occurrence",
    "foot_clearance",
    "foot_wrench",
    "force_takeoff_comparator",
    "ground_reaction_wrench",
    "interpolate_state",
    "legal_plantar_records",
    "measure",
    "native_frame",
    "orientation_state",
    "out_of_plane_observables",
    "prohibited_records",
    "scan_takeoff_candidates",
    "sampling_authority",
    "signal_processing_authority",
    "support_margin_from_point",
    "system_body_ids",
    "system_com_state",
    "total_ground_wrench",
    "trapezoidal_integral",
    "vertical_impulse_between",
]
