"""V3 actuation authority layer (RES-85).

Authority: ``LCMJ_RES85_ACTUATION_AUTHORITY_V1``
Bundle:    ``audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001``

This module is the **only** place in V3 that may turn a desired net joint
moment into an applied net joint moment.  It owns, in the frozen order:

1. bilateral symmetry projection for the four mirrored pairs,
2. the torque-rate limit (whose sole history state is the previous applied
   torque),
3. the net-moment ceiling,
4. the joint-power ceiling,
5. the MTP energy gates of ``LCMJ_RES85_MTP_ENERGY_AUTHORITY_V1``.

No controller, phase or scorer code may bypass this layer: they produce a
*desired* moment and receive an :class:`V3AppliedTorque` record that carries the
applied moment, the saturation stage, the margins and the joint power.

The Plant is never modified: the layer only writes ``data.ctrl`` (nine motor
actuators, gear = 1, so ``ctrl`` is the physical net joint moment in N*m).

Provenance classes for the nine frozen ceilings are recorded in the authority
bundle; every scalar is ``ENGINEERING_NOMINAL_WITH_SENSITIVITY`` with a
declared sweep and never a copied V2 or R001 value.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from loaded_cmj.v3.constants import V3_ACTUATOR_NAMES

# ===========================================================================
# Frozen numeric authority (mirrors ACTUATION_AUTHORITY.json exactly)
# ===========================================================================
V3_ACTUATION_AUTHORITY_ID = "LCMJ_RES85_ACTUATION_AUTHORITY_V1"
V3_ACTUATION_AUTHORITY_BUNDLE = "audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001"

CHANNELS: tuple[str, ...] = (
    "trunk_pelvis",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
    "left_mtp",
    "right_mtp",
)
N_CHANNELS = len(CHANNELS)

MOMENT_CEILING_NM = np.array([220.0, 330.0, 330.0, 380.0, 380.0, 260.0, 260.0, 45.0, 45.0])
POWER_CEILING_W = np.array([600.0, 1600.0, 1600.0, 1700.0, 1700.0, 1100.0, 1100.0, 120.0, 120.0])
RATE_CEILING_NM_PER_S = np.array([3000.0, 6000.0, 6000.0, 8000.0, 8000.0, 6000.0, 6000.0, 1500.0, 1500.0])

MIRRORED_PAIRS: tuple[tuple[int, int], ...] = ((1, 2), (3, 4), (5, 6), (7, 8))
MTP_CHANNEL_INDICES: tuple[int, int] = (7, 8)
SIDE_OF_MTP: dict[int, str] = {7: "left", 8: "right"}

QDDOT_EPSILON_RAD_PER_S = 1.0e-6
SYMMETRY_TOLERANCE_NM = 1.0e-9
WORK_TOLERANCE_J = 1.0e-12

# MTP energy authority (mirrors MTP_ENERGY_AUTHORITY.json exactly)
V3_MTP_ENERGY_AUTHORITY_ID = "LCMJ_RES85_MTP_ENERGY_AUTHORITY_V1"
MTP_ACTIVE_POSITIVE_WORK_BUDGET_J = 25.0
MTP_TOTAL_POSITIVE_WORK_BUDGET_J = 40.0
MTP_LATE_ACTIVE_FRACTION = 0.5

STAGE_NAMES = ("none", "symmetry", "rate", "moment", "power", "mtp_budget",
               "mtp_phase_gate", "safety_override_moment_ceiling",
               "safety_override_joint_power_ceiling",
               "safety_override_mtp_energy_gate",
               "safety_override_mtp_phase_gate")

# ---------------------------------------------------------------------------
# RES-85C coherent actuation contract
#
#   torque-rate limit          = NOMINAL_SLEW_BOUND (smoothness, not safety)
#   moment / power / MTP gate  = HARD_SAFETY_BOUNDS  (never exceeded)
#
# A hard safety bound always wins.  When a changing hard bound makes the
# slew-feasible interval empty the instantaneous reduction is recorded as an
# explicit SAFETY_OVERRIDE carrying the single binding hard constraint and the
# violated nominal slew margin; a sample inside a safety override is never
# reported as compliant with the nominal slew bound.
# ---------------------------------------------------------------------------
TORQUE_RATE_ROLE = "NOMINAL_SLEW_BOUND"
HARD_SAFETY_BOUNDS = ("moment_ceiling", "joint_power_ceiling", "mtp_energy_gate")
SAFETY_OVERRIDE_REASONS = ("moment_ceiling", "joint_power_ceiling",
                           "mtp_energy_gate", "mtp_phase_gate")

# The RES-86 landing phases are classified in the same LATE class as the
# RES-85 LANDING_PREP: the late active-MTP fraction (0.5) applies unchanged and
# no ceiling, budget or rate value is altered.  This is the conservative
# reading of the frozen MTP energy authority for the downstream landing.
LATE_PHASES = ("PROPULSION", "TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP",
               "IMPACT_ABSORPTION", "LANDING_CAPTURE", "E10_CONFIRMED")

PLANT_MTP_STIFFNESS_NM_PER_RAD = 25.0
PLANT_MTP_DAMPING_NMS_PER_RAD = 2.0
PLANT_MTP_NEUTRAL_RAD = 0.0


class V3ActuationError(ValueError):
    """Raised on a malformed command or an invalid authority configuration."""


@dataclass(frozen=True)
class V3MtpLedgerEntry:
    """Per-foot MTP energy ledger (active and passive kept strictly separate)."""

    active_positive_work_j: float = 0.0
    total_positive_work_j: float = 0.0
    active_gated: bool = False
    late_phase: bool = False
    total_budget_exceeded_by_passive: bool = False


@dataclass(frozen=True)
class V3AppliedTorque:
    """One native-sample application record from the authority layer."""

    commanded_nm: tuple[float, ...]
    applied_nm: tuple[float, ...]
    torque_rate_nm_per_s: tuple[float, ...]
    joint_power_w: tuple[float, ...]
    saturation_stage: tuple[str, ...]
    moment_margin_nm: tuple[float, ...]
    rate_margin_nm: tuple[float, ...]
    power_margin_w: tuple[float, ...]
    symmetry_asymmetry_nm: float
    symmetry_projected: bool
    mtp_active_applied_nm: tuple[float, float]
    mtp_gated: tuple[bool, bool]
    safety_override: tuple[bool, ...] = ()
    safety_override_reason: tuple[str, ...] = ()
    nominal_slew_exceedance_nm_per_s: tuple[float, ...] = ()

    def as_array(self) -> np.ndarray:
        return np.asarray(self.applied_nm, dtype=np.float64)


def plant_mtp_passive_moment_nm(q_rad: Sequence[float], qdot_rad_s: Sequence[float],
                                stiffness: float = PLANT_MTP_STIFFNESS_NM_PER_RAD,
                                damping: float = PLANT_MTP_DAMPING_NMS_PER_RAD,
                                neutral: float = PLANT_MTP_NEUTRAL_RAD,
                                ) -> tuple[float, float]:
    """FM-09 passive MTP prior moment (Plant authority, never a controller command)."""
    left = -stiffness * (float(q_rad[0]) - neutral) - damping * float(qdot_rad_s[0])
    right = -stiffness * (float(q_rad[1]) - neutral) - damping * float(qdot_rad_s[1])
    return float(left), float(right)


@dataclass(frozen=True)
class V3ActuationState:
    """Validated snapshot of the actuation-authority history state.

    The authority owns exactly two history quantities: the previous applied
    moment per channel (the sole torque-rate history) and the per-foot MTP
    energy ledger.  The step counter is carried for evidence only.  A snapshot
    can be restored onto this or another authority instance so that the
    RES-85 -> RES-86 handoff and every exact branch replay continue the same
    rate-limit and MTP-budget history with no reset and no hidden zeroing.
    """

    previous_applied_nm: tuple[float, ...]
    mtp_ledger: tuple[V3MtpLedgerEntry, V3MtpLedgerEntry]
    step_index: int

    def validate(self) -> list[str]:
        failures: list[str] = []
        if len(self.previous_applied_nm) != N_CHANNELS:
            failures.append("PREVIOUS_APPLIED_SHAPE")
        elif not np.all(np.isfinite(np.asarray(self.previous_applied_nm, dtype=np.float64))):
            failures.append("PREVIOUS_APPLIED_NON_FINITE")
        for foot, entry in enumerate(self.mtp_ledger):
            for name in ("active_positive_work_j", "total_positive_work_j"):
                value = float(getattr(entry, name))
                if not np.isfinite(value) or value < 0.0:
                    failures.append(f"MTP_LEDGER_FOOT_{foot}_{name.upper()}")
        if self.step_index < 0:
            failures.append("STEP_INDEX_NEGATIVE")
        return failures


class V3ActuationAuthority:
    """The single V3 actuation-authority layer for the nine motor channels."""

    def __init__(self, dt_s: float, *,
                 mtp_active_positive_work_budget_j: float | None = None,
                 mtp_total_positive_work_budget_j: float | None = None,
                 mtp_moment_ceiling_nm: float | None = None,
                 mtp_power_ceiling_w: float | None = None) -> None:
        dt = float(dt_s)
        if not np.isfinite(dt) or dt <= 0.0:
            raise V3ActuationError(f"dt_s must be positive and finite, got {dt_s!r}")
        self.dt_s = dt
        # Declared sensitivity overrides (RES-85C Blocker D).  ``None`` keeps the
        # sealed authority values exactly; a value here is an explicitly
        # declared experiment and is recorded in the authority record.
        self._active_budget_override = mtp_active_positive_work_budget_j
        self._total_budget_override = mtp_total_positive_work_budget_j
        self._mtp_moment_override = mtp_moment_ceiling_nm
        self._mtp_power_override = mtp_power_ceiling_w
        for name, value in (("mtp_active_positive_work_budget_j",
                             self._active_budget_override),
                            ("mtp_total_positive_work_budget_j",
                             self._total_budget_override),
                            ("mtp_moment_ceiling_nm", self._mtp_moment_override),
                            ("mtp_power_ceiling_w", self._mtp_power_override)):
            if value is not None and (not np.isfinite(value) or value < 0.0):
                raise V3ActuationError(f"{name} override must be finite and >= 0, got {value!r}")
        self.reset()

    @property
    def effective_mtp_active_budget_j(self) -> float:
        return (MTP_ACTIVE_POSITIVE_WORK_BUDGET_J if self._active_budget_override is None
                else float(self._active_budget_override))

    @property
    def effective_mtp_total_budget_j(self) -> float:
        return (MTP_TOTAL_POSITIVE_WORK_BUDGET_J if self._total_budget_override is None
                else float(self._total_budget_override))

    @property
    def sensitivity_overrides(self) -> dict[str, float | None]:
        return {
            "mtp_active_positive_work_budget_j": self._active_budget_override,
            "mtp_total_positive_work_budget_j": self._total_budget_override,
            "mtp_moment_ceiling_nm": self._mtp_moment_override,
            "mtp_power_ceiling_w": self._mtp_power_override,
        }

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._previous_applied = np.zeros(N_CHANNELS, dtype=np.float64)
        self._step_index = 0
        self._mtp_ledger: tuple[V3MtpLedgerEntry, V3MtpLedgerEntry] = (
            V3MtpLedgerEntry(), V3MtpLedgerEntry())

    @property
    def previous_applied(self) -> np.ndarray:
        """Previous applied torque per channel — the sole history state."""
        return self._previous_applied.copy()

    @property
    def step_index(self) -> int:
        return self._step_index

    @property
    def internal_mtp_ledger(self) -> tuple[V3MtpLedgerEntry, V3MtpLedgerEntry]:
        """The authority-held MTP ledger (mirrors the returned ledger)."""
        return (self._mtp_ledger[0], self._mtp_ledger[1])

    # ------------------------------------------------------------------
    # exact history snapshot / restore (RES-85 -> RES-86 handoff + branches)
    # ------------------------------------------------------------------
    def snapshot_state(self) -> V3ActuationState:
        """Capture the complete history state (no reset, no zeroing)."""
        return V3ActuationState(
            previous_applied_nm=tuple(float(v) for v in self._previous_applied),
            mtp_ledger=(self._mtp_ledger[0], self._mtp_ledger[1]),
            step_index=int(self._step_index),
        )

    def restore_state(self, state: V3ActuationState) -> None:
        """Restore a validated snapshot exactly (fail closed when malformed)."""
        failures = state.validate()
        if failures:
            raise V3ActuationError("invalid actuation state: " + "; ".join(failures))
        self._previous_applied = np.asarray(state.previous_applied_nm, dtype=np.float64).copy()
        self._mtp_ledger = (state.mtp_ledger[0], state.mtp_ledger[1])
        self._step_index = int(state.step_index)

    # ------------------------------------------------------------------
    # the single entry point
    # ------------------------------------------------------------------
    def apply(
        self,
        desired_nm: Sequence[float],
        qdot_rad_s: Sequence[float],
        *,
        phase: str,
        mtp_active_allowed: bool | Sequence[bool],
        mtp_passive_moment_nm: Sequence[float],
        mtp_ledger: tuple[V3MtpLedgerEntry, V3MtpLedgerEntry],
        dt_s: float | None = None,
    ) -> tuple[np.ndarray, V3AppliedTorque, tuple[V3MtpLedgerEntry, V3MtpLedgerEntry]]:
        """Apply the frozen enforcement chain and return ``(ctrl, record, ledger)``.

        ``desired_nm`` is the controller's desired net moment per channel in
        :data:`CHANNELS` order.  ``qdot_rad_s`` is the Plant joint velocity of
        the same nine channels.  ``mtp_passive_moment_nm`` is the Plant's
        FM-09 passive MTP moment per foot (left, right).  ``mtp_ledger`` carries
        the cumulative MTP energy state; it is returned updated and never
        mutated in place.
        """
        dt = self.dt_s if dt_s is None else float(dt_s)
        if dt <= 0.0:
            raise V3ActuationError("dt_s must be positive")
        commanded = np.asarray(desired_nm, dtype=np.float64)
        qdot = np.asarray(qdot_rad_s, dtype=np.float64)
        passive = np.asarray(mtp_passive_moment_nm, dtype=np.float64)
        if commanded.shape != (N_CHANNELS,):
            raise V3ActuationError(f"desired_nm must have shape ({N_CHANNELS},), got {commanded.shape}")
        if qdot.shape != (N_CHANNELS,):
            raise V3ActuationError(f"qdot_rad_s must have shape ({N_CHANNELS},), got {qdot.shape}")
        if passive.shape != (2,):
            raise V3ActuationError("mtp_passive_moment_nm must have shape (2,)")
        if not np.all(np.isfinite(commanded)):
            raise V3ActuationError("desired_nm contains a non-finite value")
        if not np.all(np.isfinite(qdot)):
            raise V3ActuationError("qdot_rad_s contains a non-finite value")
        if not np.all(np.isfinite(passive)):
            raise V3ActuationError("mtp_passive_moment_nm contains a non-finite value")

        stage = np.array(["none"] * N_CHANNELS, dtype=object)

        # ------------------------------------------------------------------
        # 1. bilateral symmetry projection
        # ------------------------------------------------------------------
        asymmetry = 0.0
        symmetric = commanded.copy()
        for i, j in MIRRORED_PAIRS:
            asymmetry = max(asymmetry, abs(float(commanded[i] - commanded[j])))
            mean = 0.5 * (commanded[i] + commanded[j])
            symmetric[i] = mean
            symmetric[j] = mean
        symmetry_projected = bool(asymmetry > 0.0)
        if symmetry_projected:
            for i, j in MIRRORED_PAIRS:
                stage[i] = "symmetry"
                stage[j] = "symmetry"

        # ------------------------------------------------------------------
        # 2..5. feasible-interval projection
        #
        # The applied moment must satisfy, simultaneously:
        #   * the torque-rate ceiling around the previous applied moment,
        #   * the net-moment ceiling,
        #   * the joint-power ceiling,
        #   * the MTP energy gates (per foot, active and passive separate).
        #
        # Each constraint is an interval for this sample's moment, so their
        # intersection is projected onto directly.  When a rapidly shrinking
        # power/MTP ceiling makes the intersection empty, the moment/power/MTP
        # limit takes precedence over smoothness and the instantaneous
        # reduction is recorded as a declared emergency stage.
        # ------------------------------------------------------------------
        rate_limit = RATE_CEILING_NM_PER_S * dt
        effective_qdot = np.where(np.abs(qdot) > QDDOT_EPSILON_RAD_PER_S, qdot, 1.0)
        power_ceiling = POWER_CEILING_W.copy()
        if self._mtp_power_override is not None:
            for idx in MTP_CHANNEL_INDICES:
                power_ceiling[idx] = float(self._mtp_power_override)
        power_cap_nm = np.where(np.abs(qdot) > QDDOT_EPSILON_RAD_PER_S,
                                power_ceiling / np.abs(effective_qdot),
                                np.inf)
        moment_cap_nm = MOMENT_CEILING_NM.copy()
        if self._mtp_moment_override is not None:
            for idx in MTP_CHANNEL_INDICES:
                moment_cap_nm[idx] = float(self._mtp_moment_override)
        mtp_cap_nm = np.full(N_CHANNELS, np.inf, dtype=np.float64)
        mtp_cap_reason = ["mtp_energy_gate"] * N_CHANNELS
        effective_cap_nm = np.minimum(moment_cap_nm, power_cap_nm)

        if isinstance(mtp_active_allowed, bool):
            allowed_pair = (bool(mtp_active_allowed), bool(mtp_active_allowed))
        else:
            allowed_pair = (bool(mtp_active_allowed[0]), bool(mtp_active_allowed[1]))
        ledger = [mtp_ledger[0], mtp_ledger[1]]
        gated = [False, False]
        mtp_cap_binding = [False, False]
        mtp_exhausted = [False, False]
        late_phase = phase in LATE_PHASES
        for foot, idx in enumerate(MTP_CHANNEL_INDICES):
            entry = ledger[foot]
            active_budget = self.effective_mtp_active_budget_j * (
                MTP_LATE_ACTIVE_FRACTION if late_phase else 1.0)
            remaining_active = max(active_budget - entry.active_positive_work_j, 0.0)
            remaining_total = max(self.effective_mtp_total_budget_j
                                  - entry.total_positive_work_j, 0.0)
            mtp_exhausted[foot] = bool(remaining_active <= WORK_TOLERANCE_J
                                       or remaining_total <= WORK_TOLERANCE_J)
            if not allowed_pair[foot]:
                mtp_cap_nm[idx] = 0.0
                mtp_cap_reason[idx] = "mtp_phase_gate"
                gated[foot] = True
                stage[idx] = "mtp_phase_gate"
            elif entry.active_gated or mtp_exhausted[foot]:
                mtp_cap_nm[idx] = 0.0
                mtp_cap_reason[idx] = "mtp_energy_gate"
                gated[foot] = True
                stage[idx] = "mtp_budget"
            else:
                budget_cap = np.inf
                if abs(effective_qdot[idx]) > QDDOT_EPSILON_RAD_PER_S:
                    # hard budget: the remaining budget may be spent in this step
                    budget_cap = min(remaining_active, remaining_total) / (
                        abs(effective_qdot[idx]) * dt)
                if budget_cap < effective_cap_nm[idx]:
                    effective_cap_nm[idx] = budget_cap
                    mtp_cap_nm[idx] = budget_cap
                    mtp_cap_binding[foot] = True
            effective_cap_nm[idx] = min(effective_cap_nm[idx], mtp_cap_nm[idx])

        lo = np.maximum(-effective_cap_nm, self._previous_applied - rate_limit)
        hi = np.minimum(effective_cap_nm, self._previous_applied + rate_limit)
        safety_override = lo > hi
        applied = np.clip(symmetric, lo, hi)
        if np.any(safety_override):
            applied = np.where(safety_override, np.clip(self._previous_applied,
                                                        -effective_cap_nm, effective_cap_nm),
                               applied)

        override_reason = [""] * N_CHANNELS
        override_active = [bool(v) for v in safety_override]
        for idx in range(N_CHANNELS):
            if not override_active[idx]:
                continue
            candidates = (("moment_ceiling", float(moment_cap_nm[idx])),
                          ("joint_power_ceiling", float(power_cap_nm[idx])),
                          (str(mtp_cap_reason[idx]), float(mtp_cap_nm[idx])))
            override_reason[idx] = min(candidates, key=lambda item: item[1])[0]

        for idx in range(N_CHANNELS):
            pre_stage = str(stage[idx])
            at_cap = abs(applied[idx]) >= effective_cap_nm[idx] - 1.0e-12
            at_rate = abs(applied[idx] - self._previous_applied[idx]) >= rate_limit[idx] - 1.0e-12
            is_mtp = idx in MTP_CHANNEL_INDICES
            if override_active[idx]:
                stage[idx] = "safety_override_" + override_reason[idx]
            elif at_cap and effective_cap_nm[idx] < np.inf:
                if pre_stage in ("mtp_phase_gate", "mtp_budget"):
                    stage[idx] = pre_stage
                elif is_mtp and mtp_cap_binding[MTP_CHANNEL_INDICES.index(idx)]:
                    stage[idx] = "mtp_budget"
                elif power_cap_nm[idx] < moment_cap_nm[idx]:
                    stage[idx] = "power"
                else:
                    stage[idx] = "moment"
            elif at_rate:
                stage[idx] = "rate"
            elif pre_stage != "symmetry":
                stage[idx] = "none"
        moment_margin = MOMENT_CEILING_NM - np.abs(applied)
        power_margin = POWER_CEILING_W - np.abs(applied * qdot)
        slew_exceedance = np.maximum(
            np.abs(applied - self._previous_applied) / dt - RATE_CEILING_NM_PER_S, 0.0)
        slew_exceedance = np.where(safety_override, slew_exceedance, 0.0)

        # ------------------------------------------------------------------
        # 6. MTP energy ledger update from the applied moment
        # ------------------------------------------------------------------
        for foot, idx in enumerate(MTP_CHANNEL_INDICES):
            entry = ledger[foot]
            tau = float(applied[idx])
            qd = float(qdot[idx])
            passive_moment = float(passive[foot])
            active_power = tau * qd
            total_power = active_power + passive_moment * qd
            total_exceeded_by_passive = entry.total_budget_exceeded_by_passive
            total_budget = self.effective_mtp_total_budget_j
            if max(total_power, 0.0) * dt > max(
                    total_budget - entry.total_positive_work_j, 0.0) + WORK_TOLERANCE_J:
                if max(passive_moment * qd, 0.0) >= max(
                        total_budget - entry.total_positive_work_j, 0.0) / dt:
                    total_exceeded_by_passive = True
            ledger[foot] = V3MtpLedgerEntry(
                active_positive_work_j=entry.active_positive_work_j + max(active_power, 0.0) * dt,
                total_positive_work_j=entry.total_positive_work_j + max(total_power, 0.0) * dt,
                active_gated=bool(entry.active_gated or gated[foot] or mtp_exhausted[foot]),
                late_phase=late_phase,
                total_budget_exceeded_by_passive=bool(total_exceeded_by_passive),
            )
            gated[foot] = bool(gated[foot])

        torque_rate = (applied - self._previous_applied) / dt
        rate_margin = rate_limit - np.abs(applied - self._previous_applied)
        self._previous_applied = applied.copy()
        self._step_index += 1
        self._mtp_ledger = (ledger[0], ledger[1])

        record = V3AppliedTorque(
            commanded_nm=tuple(float(v) for v in commanded),
            applied_nm=tuple(float(v) for v in applied),
            torque_rate_nm_per_s=tuple(float(v) for v in torque_rate),
            joint_power_w=tuple(float(v) for v in applied * qdot),
            saturation_stage=tuple(str(s) for s in stage),
            moment_margin_nm=tuple(float(v) for v in moment_margin),
            rate_margin_nm=tuple(float(v) for v in rate_margin),
            power_margin_w=tuple(float(v) for v in power_margin),
            symmetry_asymmetry_nm=float(asymmetry),
            symmetry_projected=symmetry_projected,
            mtp_active_applied_nm=(float(applied[MTP_CHANNEL_INDICES[0]]),
                                   float(applied[MTP_CHANNEL_INDICES[1]])),
            mtp_gated=(bool(gated[0]), bool(gated[1])),
            safety_override=tuple(bool(v) for v in override_active),
            safety_override_reason=tuple(str(v) for v in override_reason),
            nominal_slew_exceedance_nm_per_s=tuple(float(v) for v in slew_exceedance),
        )
        return applied, record, (ledger[0], ledger[1])

    # ------------------------------------------------------------------
    # introspection / evidence surface
    # ------------------------------------------------------------------
    def authority_record(self) -> dict[str, object]:
        return {
            "authority_id": V3_ACTUATION_AUTHORITY_ID,
            "bundle": V3_ACTUATION_AUTHORITY_BUNDLE,
            "channels": list(CHANNELS),
            "actuator_names": list(V3_ACTUATOR_NAMES),
            "moment_ceiling_nm": [float(v) for v in MOMENT_CEILING_NM],
            "power_ceiling_w": [float(v) for v in POWER_CEILING_W],
            "rate_ceiling_nm_per_s": [float(v) for v in RATE_CEILING_NM_PER_S],
            "mirrored_pairs": [list(p) for p in MIRRORED_PAIRS],
            "symmetry_tolerance_nm": SYMMETRY_TOLERANCE_NM,
            "symmetry_mode": "ENFORCED_FOR_ALL_RES85_PHASES",
            "mtp_budgets": {
                "active_positive_work_j": self.effective_mtp_active_budget_j,
                "total_positive_work_j": self.effective_mtp_total_budget_j,
                "late_active_fraction": MTP_LATE_ACTIVE_FRACTION,
                "sealed_active_positive_work_j": MTP_ACTIVE_POSITIVE_WORK_BUDGET_J,
                "sealed_total_positive_work_j": MTP_TOTAL_POSITIVE_WORK_BUDGET_J,
                "declared_sensitivity_overrides": self.sensitivity_overrides,
            },
            "mtp_energy_authority_id": V3_MTP_ENERGY_AUTHORITY_ID,
            "torque_rate_role": TORQUE_RATE_ROLE,
            "hard_safety_bounds": list(HARD_SAFETY_BOUNDS),
            "safety_override_reasons": list(SAFETY_OVERRIDE_REASONS),
            "safety_override_semantics": (
                "a hard safety bound always wins; an empty slew-feasible "
                "intersection is recorded as an explicit safety override with "
                "the single binding hard constraint and the violated nominal "
                "slew margin; an override sample is never reported as compliant "
                "with the nominal slew bound"),
            "stages": list(STAGE_NAMES),
            "qdot_epsilon_rad_per_s": QDDOT_EPSILON_RAD_PER_S,
            "plant_passive_prior": {
                "stiffness_nm_per_rad": PLANT_MTP_STIFFNESS_NM_PER_RAD,
                "damping_nms_per_rad": PLANT_MTP_DAMPING_NMS_PER_RAD,
                "neutral_rad": PLANT_MTP_NEUTRAL_RAD,
                "owner": "Plant authority (FM-09); not a controller command",
            },
        }


__all__ = [
    "CHANNELS",
    "LATE_PHASES",
    "MIRRORED_PAIRS",
    "MOMENT_CEILING_NM",
    "MTP_ACTIVE_POSITIVE_WORK_BUDGET_J",
    "MTP_CHANNEL_INDICES",
    "MTP_LATE_ACTIVE_FRACTION",
    "MTP_TOTAL_POSITIVE_WORK_BUDGET_J",
    "N_CHANNELS",
    "PLANT_MTP_DAMPING_NMS_PER_RAD",
    "PLANT_MTP_NEUTRAL_RAD",
    "PLANT_MTP_STIFFNESS_NM_PER_RAD",
    "POWER_CEILING_W",
    "RATE_CEILING_NM_PER_S",
    "STAGE_NAMES",
    "SYMMETRY_TOLERANCE_NM",
    "V3ActuationAuthority",
    "V3ActuationError",
    "V3ActuationState",
    "V3AppliedTorque",
    "V3MtpLedgerEntry",
    "V3_ACTUATION_AUTHORITY_BUNDLE",
    "V3_ACTUATION_AUTHORITY_ID",
    "V3_MTP_ENERGY_AUTHORITY_ID",
    "plant_mtp_passive_moment_nm",
]
