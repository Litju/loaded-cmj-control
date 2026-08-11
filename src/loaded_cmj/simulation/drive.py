"""The complete trusted actuator transformation.

The actuator model is public and deterministic. A policy may inspect the
model to predict its own torques, but the isolated policy process cannot
execute trusted plant code.
Deterministic: no RNG, identical results for identical command, hidden state,
anatomical coordinates and rates.

The exact ordered pipeline is executed once per physics substep.  The public
command is held constant across the forty substeps of the final 200 Hz
control period::

    validated action u in [-1,1]^15
      -> section 1  exact separate positive/negative activation update
    signed aggregate drive d = a_plus - a_minus
      -> section 2  torque-angle capacity
      -> section 3  torque-velocity capacity
      -> section 4  desired anatomical torque
      -> section 5  signed power projection
      -> section 6  rate projection toward feasible torque
      -> section 7  hard capacity
      -> section 8  hard power
      -> section 9  virtual-work map (owned by Plant)
      -> section 10 record state, torque and override flags
    anatomical torque tau_eta in R^15

Saturation may reduce magnitude.  Saturation may never reverse the requested
anatomical torque sign (section 1.1).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from loaded_cmj.simulation.constants import (
    ACTION_CHANNELS,
    ACTION_DIM,
    BALL_JOINT_CHANNELS,
    CHANNEL_BALL_JOINT,
    HOLD_TAU_PREV_INITIAL,
    HOLD_Z_INITIAL,
    PHYSICS_TIMESTEP_S,
    SUBSTEPS_PER_CONTROL,
)


class DriveModelError(RuntimeError):
    """Trusted drive-model invariant violated.  Callers raise InternalEvaluationError."""


# ==========================================================================
# Section 10 -- drive parameters selected from the final-authority candidate
# ranges.  These are qualification-bound parameters, not controller knobs.
# ==========================================================================
TAU_UP = 0.050        # s, rise candidate
TAU_DOWN = 0.070      # s, decay candidate
RHO = 20.0            # 1/s, retained bounded-rate candidate

if not TAU_UP < TAU_DOWN:  # section 2.6 hard invariant, checked at import
    raise DriveModelError("TAU_UP must be strictly less than TAU_DOWN")
if not RHO * PHYSICS_TIMESTEP_S < 1.0:
    raise DriveModelError("rho * h must be < 1 so one substep cannot traverse the range")

_ZERO_SNAP = 1e-300   # section 2.4: magnitudes below this collapse to exactly +0.0

# ==========================================================================
# Section 4 -- torque-angle envelope (qualification parameters)
# ==========================================================================
F_MIN = 0.30          # hard invariant: no task-relevant authority dead band

# (tau_bar_pos, s_opt_pos, sigma_pos, tau_bar_neg, s_opt_neg, sigma_neg)
# Ball channels carry 3-vectors: the SAME eta_j drives all three channels of
# that joint, so hip flexion capacity legitimately depends on abduction and
# rotation.  Hinge channels carry scalars.
_TORQUE_ANGLE: dict[str, tuple] = {
    "lumbar_flexion": (
        200.0, (0.35, 0.0, 0.0), (0.80, 1.20, 1.20),
        300.0, (-0.15, 0.0, 0.0), (0.80, 1.20, 1.20)),
    "lumbar_lateral": (
        180.0, (0.0, 0.20, 0.0), (1.20, 0.70, 1.20),
        180.0, (0.0, -0.20, 0.0), (1.20, 0.70, 1.20)),
    "lumbar_axial": (
        120.0, (0.0, 0.0, 0.20), (1.20, 1.20, 0.80),
        120.0, (0.0, 0.0, -0.20), (1.20, 1.20, 0.80)),
    "left_hip_flexion": (
        180.0, (0.90, 0.0, 0.0), (1.10, 1.40, 1.40),
        250.0, (1.20, 0.0, 0.0), (1.20, 1.40, 1.40)),
    "left_hip_abduction": (
        120.0, (0.0, 0.25, 0.0), (1.40, 0.80, 1.40),
        120.0, (0.0, -0.10, 0.0), (1.40, 0.80, 1.40)),
    "left_hip_rotation": (
        70.0, (0.0, 0.0, 0.20), (1.40, 1.40, 0.80),
        70.0, (0.0, 0.0, -0.20), (1.40, 1.40, 0.80)),
    "right_hip_flexion": (
        180.0, (0.90, 0.0, 0.0), (1.10, 1.40, 1.40),
        250.0, (1.20, 0.0, 0.0), (1.20, 1.40, 1.40)),
    "right_hip_abduction": (
        120.0, (0.0, 0.25, 0.0), (1.40, 0.80, 1.40),
        120.0, (0.0, -0.10, 0.0), (1.40, 0.80, 1.40)),
    "right_hip_rotation": (
        70.0, (0.0, 0.0, 0.20), (1.40, 1.40, 0.80),
        70.0, (0.0, 0.0, -0.20), (1.40, 1.40, 0.80)),
    # Knee extension is deliberately weak at full extension: s_opt_neg =
    # 1.0472 rad with sigma_neg = 0.90 puts f_q ~ 0.64 at 0 rad flexion, which
    # is what forces a real countermovement instead of a static leg press.
    "left_knee_flexion": (140.0, 0.5236, 1.00, 300.0, 1.0472, 0.90),
    "right_knee_flexion": (140.0, 0.5236, 1.00, 300.0, 1.0472, 0.90),
    "left_ankle_dorsiflexion": (50.0, 0.0873, 0.60, 220.0, 0.1745, 0.55),
    "right_ankle_dorsiflexion": (50.0, 0.0873, 0.60, 220.0, 0.1745, 0.55),
    "left_ankle_eversion": (40.0, 0.0, 0.45, 40.0, 0.0, 0.45),
    "right_ankle_eversion": (40.0, 0.0, 0.45, 40.0, 0.0, 0.45),
}

# ==========================================================================
# Section 11 -- torque-velocity envelope
# ==========================================================================
FV_MIN = 0.10         # final-authority concentric asymptote candidate
F_ECC = 1.40          # final-authority eccentric asymptote candidate
F_ECC_MAX = F_ECC
S_C = 1.0             # concentric speed scale
S_E = (F_ECC - 1.0) * S_C / (1.0 - FV_MIN)
# Compatibility names retained as inspectable aliases; the final curve is the
# closed-form C1 exponential, not the former piecewise Hermite construction.
C_C = S_C
C_E = S_E
BRIDGE_DELTA = 0.0

_OMEGA_MAX: dict[str, tuple[float, float]] = {
    "lumbar_flexion": (8.0, 8.0),
    "lumbar_lateral": (8.0, 8.0),
    "lumbar_axial": (8.0, 8.0),
    "left_hip_flexion": (12.0, 12.0),
    "right_hip_flexion": (12.0, 12.0),
    "left_hip_abduction": (8.0, 8.0),
    "right_hip_abduction": (8.0, 8.0),
    "left_hip_rotation": (8.0, 8.0),
    "right_hip_rotation": (8.0, 8.0),
    "left_knee_flexion": (14.0, 14.0),
    "right_knee_flexion": (14.0, 14.0),
    "left_ankle_dorsiflexion": (12.0, 12.0),
    "right_ankle_dorsiflexion": (12.0, 12.0),
    "left_ankle_eversion": (8.0, 8.0),
    "right_ankle_eversion": (8.0, 8.0),
}

# ==========================================================================
# Section 6 -- power caps (qualification parameters). P- = 1.5 * P+.
# ==========================================================================
_POWER: dict[str, tuple[float, float]] = {
    "lumbar_flexion": (800.0, 1200.0),
    "lumbar_lateral": (400.0, 600.0),
    "lumbar_axial": (300.0, 450.0),
    "left_hip_flexion": (1500.0, 2250.0),
    "right_hip_flexion": (1500.0, 2250.0),
    "left_hip_abduction": (500.0, 750.0),
    "right_hip_abduction": (500.0, 750.0),
    "left_hip_rotation": (300.0, 450.0),
    "right_hip_rotation": (300.0, 450.0),
    "left_knee_flexion": (1800.0, 2700.0),
    "right_knee_flexion": (1800.0, 2700.0),
    "left_ankle_dorsiflexion": (1200.0, 1800.0),
    "right_ankle_dorsiflexion": (1200.0, 1800.0),
    "left_ankle_eversion": (200.0, 300.0),
    "right_ankle_eversion": (200.0, 300.0),
}

# Section 7 -- rate constraint coefficient (qualified parameter range [15, 50])
RATE_COEFFICIENT = 25.0

# Separate positive and negative activation states.  The four directional
# time constants are explicit so the update remains auditable and exact.
TAU_ACT_POS = np.full(ACTION_DIM, TAU_UP, dtype=np.float64)
TAU_ACT_NEG = np.full(ACTION_DIM, TAU_UP, dtype=np.float64)
TAU_DEACT_POS = np.full(ACTION_DIM, TAU_DOWN, dtype=np.float64)
TAU_DEACT_NEG = np.full(ACTION_DIM, TAU_DOWN, dtype=np.float64)

# --------------------------------------------------------------------------
# Vectorised parameter arrays, in canonical channel order.
# --------------------------------------------------------------------------
_N = ACTION_DIM
TAU_BAR_POS = np.array([_TORQUE_ANGLE[c][0] for c in ACTION_CHANNELS], dtype=np.float64)
TAU_BAR_NEG = np.array([_TORQUE_ANGLE[c][3] for c in ACTION_CHANNELS], dtype=np.float64)
OMEGA_MAX_POS = np.array([_OMEGA_MAX[c][0] for c in ACTION_CHANNELS], dtype=np.float64)
OMEGA_MAX_NEG = np.array([_OMEGA_MAX[c][1] for c in ACTION_CHANNELS], dtype=np.float64)
POWER_POS = np.array([_POWER[c][0] for c in ACTION_CHANNELS], dtype=np.float64)
POWER_NEG = np.array([_POWER[c][1] for c in ACTION_CHANNELS], dtype=np.float64)
TAU_BAR_MAX = np.maximum(TAU_BAR_POS, TAU_BAR_NEG)
RATE_MAX = RATE_COEFFICIENT * TAU_BAR_MAX          # N*m/s per channel

if np.any(TAU_BAR_POS <= 0.0) or np.any(TAU_BAR_NEG <= 0.0):
    raise DriveModelError("tau_bar must be strictly positive on every channel")
if np.any(OMEGA_MAX_POS <= 0.0) or np.any(OMEGA_MAX_NEG <= 0.0):
    raise DriveModelError("omega_max must be strictly positive on every channel")

# Per-channel (s_opt, weight) with weight = 1/sigma^2, shaped for the branch.
_S_OPT_POS: list[np.ndarray] = []
_W_POS: list[np.ndarray] = []
_S_OPT_NEG: list[np.ndarray] = []
_W_NEG: list[np.ndarray] = []
for _c in ACTION_CHANNELS:
    _tp, _sp, _gp, _tn, _sn, _gn = _TORQUE_ANGLE[_c]
    _S_OPT_POS.append(np.atleast_1d(np.asarray(_sp, dtype=np.float64)))
    _W_POS.append(1.0 / np.atleast_1d(np.asarray(_gp, dtype=np.float64)) ** 2)
    _S_OPT_NEG.append(np.atleast_1d(np.asarray(_sn, dtype=np.float64)))
    _W_NEG.append(1.0 / np.atleast_1d(np.asarray(_gn, dtype=np.float64)) ** 2)
del _c, _tp, _sp, _gp, _tn, _sn, _gn

# Channel index -> slice of the anatomical coordinate vector that drives f_q.
_CHANNEL_S_SLICE: list[tuple[int, int]] = []
for _i in range(_N):
    _joint = CHANNEL_BALL_JOINT[_i]
    if _joint is None:
        _CHANNEL_S_SLICE.append((_i, _i + 1))
    else:
        _idx = BALL_JOINT_CHANNELS[_joint]
        _CHANNEL_S_SLICE.append((_idx[0], _idx[0] + 3))
del _i, _joint

DRIVE_PARAMS = {
    "model_revision": "loaded-cmj-model-1",
    "tau_up_s": TAU_UP,
    "tau_down_s": TAU_DOWN,
    "rho_per_s": RHO,
    "f_min": F_MIN,
    "fv_min": FV_MIN,
    "f_ecc": F_ECC,
    "s_c": S_C,
    "s_e": S_E,
    "rate_coefficient": RATE_COEFFICIENT,
    "physics_timestep_s": PHYSICS_TIMESTEP_S,
    "substeps_per_control": SUBSTEPS_PER_CONTROL,
    "channels": ACTION_CHANNELS,
    "tau_bar_pos": TAU_BAR_POS.tolist(),
    "tau_bar_neg": TAU_BAR_NEG.tolist(),
    "omega_max_pos": OMEGA_MAX_POS.tolist(),
    "omega_max_neg": OMEGA_MAX_NEG.tolist(),
    "power_pos_W": POWER_POS.tolist(),
    "power_neg_W": POWER_NEG.tolist(),
    "rate_max_Nm_per_s": RATE_MAX.tolist(),
    "tau_act_pos_s": TAU_ACT_POS.tolist(),
    "tau_act_neg_s": TAU_ACT_NEG.tolist(),
    "tau_deact_pos_s": TAU_DEACT_POS.tolist(),
    "tau_deact_neg_s": TAU_DEACT_NEG.tolist(),
}


# ==========================================================================
# Section 2 -- drive state and its exact discrete update
# ==========================================================================
@dataclass(init=False)
class DriveState:
    """Hidden actuator state required by the final authority.

    ``a_plus`` and ``a_minus`` are independent directional activations.  The
    signed aggregate drive is the derived quantity ``d = a_plus - a_minus``;
    retaining both states is what makes reversal continuous and prevents a
    command sign change from teleporting authority across zero.
    """

    a_plus: np.ndarray = field(default_factory=lambda: np.zeros(_N, dtype=np.float64))
    a_minus: np.ndarray = field(default_factory=lambda: np.zeros(_N, dtype=np.float64))
    tau_prev: np.ndarray = field(
        default_factory=lambda: np.asarray(HOLD_TAU_PREV_INITIAL, dtype=np.float64).copy()
    )
    previous_command: np.ndarray = field(default_factory=lambda: np.zeros(_N, dtype=np.float64))
    override_flags: dict[str, np.ndarray] = field(default_factory=dict)
    reversal_phase: np.ndarray = field(default_factory=lambda: np.zeros(_N, dtype=np.int8))

    def __init__(
        self,
        a_plus: np.ndarray | None = None,
        a_minus: np.ndarray | None = None,
        tau_prev: np.ndarray | None = None,
        previous_command: np.ndarray | None = None,
        override_flags: dict[str, np.ndarray] | None = None,
        reversal_phase: np.ndarray | None = None,
        *,
        z: np.ndarray | None = None,
    ) -> None:
        if z is not None:
            if a_plus is not None or a_minus is not None:
                raise DriveModelError("DriveState accepts either z or a_plus/a_minus, not both")
            signed = np.asarray(z, dtype=np.float64).reshape(_N)
            a_plus = np.maximum(signed, 0.0)
            a_minus = np.maximum(-signed, 0.0)
        if a_plus is None:
            a_plus = np.maximum(np.asarray(HOLD_Z_INITIAL, dtype=np.float64), 0.0)
        if a_minus is None:
            a_minus = np.maximum(-np.asarray(HOLD_Z_INITIAL, dtype=np.float64), 0.0)
        self.a_plus = np.asarray(a_plus, dtype=np.float64).reshape(_N).copy()
        self.a_minus = np.asarray(a_minus, dtype=np.float64).reshape(_N).copy()
        self.tau_prev = np.asarray(
            HOLD_TAU_PREV_INITIAL if tau_prev is None else tau_prev, dtype=np.float64
        ).reshape(_N).copy()
        self.previous_command = np.asarray(
            np.zeros(_N, dtype=np.float64) if previous_command is None else previous_command,
            dtype=np.float64,
        ).reshape(_N).copy()
        self.reversal_phase = np.asarray(
            np.zeros(_N, dtype=np.int8) if reversal_phase is None else reversal_phase,
            dtype=np.int8,
        ).reshape(_N).copy()
        self.override_flags = _copy_override_flags(override_flags)
        self._validate()

    @property
    def z(self) -> np.ndarray:
        """Compatibility view of the signed aggregate drive."""
        return self.a_plus - self.a_minus

    @z.setter
    def z(self, value: np.ndarray) -> None:
        signed = np.asarray(value, dtype=np.float64).reshape(_N)
        self.a_plus = np.maximum(signed, 0.0).copy()
        self.a_minus = np.maximum(-signed, 0.0).copy()
        self._validate()

    def _validate(self) -> None:
        if not all(np.isfinite(x).all() for x in (
            self.a_plus, self.a_minus, self.tau_prev, self.previous_command,
        )):
            raise DriveModelError("DriveState contains a non-finite value")
        if np.any(self.a_plus < 0.0) or np.any(self.a_minus < 0.0):
            raise DriveModelError("directional activation is negative")
        if np.any(self.a_plus > 1.0) or np.any(self.a_minus > 1.0):
            raise DriveModelError("directional activation left [0, 1]")

    def reset(self) -> None:
        signed = np.asarray(HOLD_Z_INITIAL, dtype=np.float64)
        self.a_plus[:] = np.maximum(signed, 0.0)
        self.a_minus[:] = np.maximum(-signed, 0.0)
        self.tau_prev[:] = HOLD_TAU_PREV_INITIAL
        self.previous_command[:] = signed
        self.reversal_phase[:] = 0
        self.override_flags = _empty_override_flags()

    def copy(self) -> DriveState:
        return DriveState(
            a_plus=self.a_plus.copy(), a_minus=self.a_minus.copy(),
            tau_prev=self.tau_prev.copy(), previous_command=self.previous_command.copy(),
            override_flags=self.override_flags, reversal_phase=self.reversal_phase.copy(),
        )


def _empty_override_flags() -> dict[str, np.ndarray]:
    return {
        "capacity_override": np.zeros(_N, dtype=bool),
        "power_override": np.zeros(_N, dtype=bool),
        "rate_override": np.zeros(_N, dtype=bool),
        "rate_override_by_capacity": np.zeros(_N, dtype=bool),
        "rate_override_by_power": np.zeros(_N, dtype=bool),
        "hard_capacity": np.zeros(_N, dtype=bool),
        "hard_power": np.zeros(_N, dtype=bool),
    }


def _copy_override_flags(flags: dict[str, np.ndarray] | None) -> dict[str, np.ndarray]:
    out = _empty_override_flags()
    if flags is not None:
        for key, value in flags.items():
            out[str(key)] = np.asarray(value, dtype=bool).reshape(_N).copy()
    return out


def _snap_zero(value: float) -> float:
    """Return canonical positive zero for the exact crossing point."""
    return 0.0 if abs(value) < _ZERO_SNAP else value


def activation_update(
    command: np.ndarray,
    a_plus: np.ndarray,
    a_minus: np.ndarray,
    h: float = PHYSICS_TIMESTEP_S,
    *,
    state: DriveState | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply the exact exponential positive/negative activation update."""
    u = np.asarray(command, dtype=np.float64).reshape(_N)
    ap = np.asarray(a_plus, dtype=np.float64).reshape(_N)
    am = np.asarray(a_minus, dtype=np.float64).reshape(_N)
    if not np.isfinite(u).all() or np.any(np.abs(u) > 1.0 + 1e-12):
        raise DriveModelError("activation command is not finite or is outside [-1, 1]")
    if not np.isfinite(ap).all() or not np.isfinite(am).all():
        raise DriveModelError("activation state is not finite")
    cp = np.maximum(u, 0.0)
    cm = np.maximum(-u, 0.0)
    tp = np.where(cp > ap, TAU_ACT_POS, TAU_DEACT_POS)
    tm = np.where(cm > am, TAU_ACT_NEG, TAU_DEACT_NEG)
    with np.errstate(over="raise", invalid="raise"):
        ap_next = cp + (ap - cp) * np.exp(-float(h) / tp)
        am_next = cm + (am - cm) * np.exp(-float(h) / tm)
    ap_next = np.clip(ap_next, 0.0, 1.0)
    am_next = np.clip(am_next, 0.0, 1.0)
    if state is not None:
        state.a_plus[:] = ap_next
        state.a_minus[:] = am_next
        state._validate()
    return ap_next, am_next


# ==========================================================================
# Section 4 -- torque-angle envelope
# ==========================================================================
def torque_angle(s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(f_q_pos, f_q_neg)`` for all fifteen channels at anatomical ``s``.

    ``f_q = f_min + (1 - f_min) exp[-0.5 (s - s_opt)^T W (s - s_opt)]``.
    ``f_q >= f_min = 0.30`` everywhere by construction.
    """
    ss = np.asarray(s, dtype=np.float64).reshape(_N)
    if not np.isfinite(ss).all():
        raise DriveModelError("anatomical coordinate s is not finite")
    fq_pos = np.empty(_N, dtype=np.float64)
    fq_neg = np.empty(_N, dtype=np.float64)
    for i in range(_N):
        lo, hi = _CHANNEL_S_SLICE[i]
        sub = ss[lo:hi]
        dp = sub - _S_OPT_POS[i]
        dn = sub - _S_OPT_NEG[i]
        fq_pos[i] = F_MIN + (1.0 - F_MIN) * np.exp(-0.5 * float(dp @ (_W_POS[i] * dp)))
        fq_neg[i] = F_MIN + (1.0 - F_MIN) * np.exp(-0.5 * float(dn @ (_W_NEG[i] * dn)))
    if not np.isfinite(fq_pos).all() or not np.isfinite(fq_neg).all():
        raise DriveModelError("torque-angle envelope evaluated non-finite")
    return fq_pos, fq_neg


# ==========================================================================
# Section 11 -- exact C1 torque-velocity envelope
# ===========================================================================
def torque_velocity(nu: np.ndarray) -> np.ndarray:
    """Bounded C1 concentric/eccentric speed curve from authority section 11."""
    arr = np.asarray(nu, dtype=np.float64)
    if not np.isfinite(arr).all():
        raise DriveModelError("normalized rate nu is not finite")
    out = np.where(
        arr >= 0.0,
        FV_MIN + (1.0 - FV_MIN) * np.exp(-arr / S_C),
        F_ECC - (F_ECC - 1.0) * np.exp(arr / S_E),
    )
    # The authority makes power nonbinding in a declared near-zero velocity
    # band.  Returning the exact neutral multiplier there also makes the
    # sampled zero-speed value independent of signed round-off.
    out = np.where(np.abs(arr) <= 1.0e-6, 1.0, out)
    if not np.isfinite(out).all() or np.any(out <= 0.0):
        raise DriveModelError("torque-velocity envelope evaluated non-finite or non-positive")
    return out


def capacity_envelope(
    s: np.ndarray, s_dot: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Section 8 envelope ``(tau_min, tau_max)`` at the current ``(s, s_dot)``."""
    sd = np.asarray(s_dot, dtype=np.float64).reshape(_N)
    if not np.isfinite(sd).all():
        raise DriveModelError("anatomical rate s_dot is not finite")
    fq_pos, fq_neg = torque_angle(s)
    fv_pos = torque_velocity(sd / OMEGA_MAX_POS)
    fv_neg = torque_velocity(-sd / OMEGA_MAX_NEG)
    tau_max = TAU_BAR_POS * fq_pos * fv_pos
    tau_min = -(TAU_BAR_NEG * fq_neg * fv_neg)
    return tau_min, tau_max


# ==========================================================================
# Sections 4--10 -- ordered capacity, power, rate and recording pipeline
# ==========================================================================
def project_signed_power(
    tau: np.ndarray,
    eta: np.ndarray,
    positive_power: np.ndarray | float,
    negative_power: np.ndarray | float,
) -> np.ndarray:
    """Project torque onto ``-P_neg <= tau * eta <= P_pos``.

    The interval endpoints are evaluated in signed velocity coordinates and
    sorted, so both positive and negative anatomical rates are handled without
    a division-by-zero branch or a sign flip.
    """
    tt = np.asarray(tau, dtype=np.float64).reshape(-1)
    ee = np.asarray(eta, dtype=np.float64).reshape(tt.shape)
    pp = np.broadcast_to(np.asarray(positive_power, dtype=np.float64), tt.shape)
    pn = np.broadcast_to(np.asarray(negative_power, dtype=np.float64), tt.shape)
    if not all(np.isfinite(x).all() for x in (tt, ee, pp, pn)) or np.any(pp < 0.0) or np.any(pn < 0.0):
        raise DriveModelError("signed power projection received a non-finite or invalid input")
    out = tt.copy()
    active = np.abs(ee) > 1e-12
    with np.errstate(divide="ignore", invalid="ignore"):
        e1 = -pn / ee
        e2 = pp / ee
    lo = np.minimum(e1, e2)
    hi = np.maximum(e1, e2)
    out[active] = np.clip(tt[active], lo[active], hi[active])
    if not np.isfinite(out).all():
        raise DriveModelError("signed power projection evaluated non-finite")
    return out.reshape(np.asarray(tau).shape)


def _ordered_torque_projection(
    a_plus: np.ndarray,
    a_minus: np.ndarray,
    s: np.ndarray,
    s_dot: np.ndarray,
    tau_previous: np.ndarray,
    h: float,
) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray, np.ndarray]:
    """Run authority stages 2--8 and return torque plus auditable flags."""
    sd = np.asarray(s_dot, dtype=np.float64).reshape(_N)
    previous = np.asarray(tau_previous, dtype=np.float64).reshape(_N)
    tau_min, tau_max = capacity_envelope(s, sd)

    # Stage 4: directional desired anatomical torque.
    desired = np.asarray(a_plus) * tau_max + np.asarray(a_minus) * tau_min

    # Stage 5: signed two-sided power interval.
    power_target = project_signed_power(desired, sd, POWER_POS, POWER_NEG)
    power_override = np.abs(power_target - desired) > 1e-12

    # Stage 6: rate limit toward the feasible target.
    delta_max = RATE_MAX * float(h)
    rate_target = previous + np.clip(power_target - previous, -delta_max, delta_max)
    rate_override = np.abs(rate_target - power_target) > 1e-12

    # Stages 7 and 8: hard capacity then hard signed power.
    capacity_target = np.clip(power_target, tau_min, tau_max)
    rate_override_by_capacity = np.abs(rate_target - capacity_target) > 1e-12
    hard_capacity = np.clip(rate_target, tau_min, tau_max)
    hard_capacity_flag = np.abs(hard_capacity - rate_target) > 1e-12
    tau = project_signed_power(hard_capacity, sd, POWER_POS, POWER_NEG)
    hard_power_flag = np.abs(tau - hard_capacity) > 1e-12
    flags = {
        "capacity_override": np.asarray(rate_override_by_capacity, dtype=bool),
        "power_override": np.asarray(power_override, dtype=bool),
        "rate_override": np.asarray(rate_override, dtype=bool),
        "rate_override_by_capacity": np.asarray(rate_override_by_capacity, dtype=bool),
        "rate_override_by_power": np.asarray(rate_override | power_override, dtype=bool),
        "hard_capacity": np.asarray(hard_capacity_flag, dtype=bool),
        "hard_power": np.asarray(hard_power_flag, dtype=bool),
    }
    if not np.isfinite(tau).all() or np.any(tau < tau_min - 1e-9) or np.any(tau > tau_max + 1e-9):
        raise DriveModelError("ordered torque pipeline left the capacity envelope")
    return tau, flags, tau_min, tau_max


def _zero_crossing_time(
    command: np.ndarray,
    a_plus: np.ndarray,
    a_minus: np.ndarray,
    h: float,
    channel: int,
) -> float | None:
    """Find the exact continuous-time net-drive zero within one substep."""
    cp = float(max(command[channel], 0.0))
    cm = float(max(-command[channel], 0.0))
    ap0 = float(a_plus[channel])
    am0 = float(a_minus[channel])
    tp = float(TAU_ACT_POS[channel] if cp > ap0 else TAU_DEACT_POS[channel])
    tm = float(TAU_ACT_NEG[channel] if cm > am0 else TAU_DEACT_NEG[channel])

    def net(t: float) -> float:
        ap = cp + (ap0 - cp) * np.exp(-t / tp)
        am = cm + (am0 - cm) * np.exp(-t / tm)
        return float(ap - am)

    left, right = 0.0, float(h)
    f_left, f_right = net(left), net(right)
    if f_left == 0.0:
        return 0.0
    if f_right == 0.0:
        return float(h)
    if f_left * f_right > 0.0:
        return None
    for _ in range(80):
        mid = 0.5 * (left + right)
        f_mid = net(mid)
        if f_mid == 0.0:
            return mid
        if f_left * f_mid <= 0.0:
            right, f_right = mid, f_mid
        else:
            left, f_left = mid, f_mid
    return 0.5 * (left + right)


def drive_state_step(
    command: np.ndarray,
    state: DriveState,
    s: np.ndarray,
    s_dot: np.ndarray,
    h: float = PHYSICS_TIMESTEP_S,
) -> dict[str, Any]:
    """Advance hidden drive state and realize one authority-ordered torque."""
    u = np.asarray(command, dtype=np.float64).reshape(_N)
    if not isinstance(state, DriveState):
        raise DriveModelError("drive_state_step requires DriveState")
    if not np.isfinite(u).all() or np.any(np.abs(u) > 1.0 + 1e-12):
        raise DriveModelError("drive command is not finite or is outside [-1, 1]")
    old_plus = state.a_plus.copy()
    old_minus = state.a_minus.copy()
    old_drive = old_plus - old_minus
    tau_previous = state.tau_prev.copy()
    ap, am = activation_update(u, old_plus, old_minus, h, state=state)
    new_drive = ap - am
    crossing: float | None = None
    crossing_channels = np.zeros(_N, dtype=bool)
    for i in range(_N):
        if old_drive[i] * new_drive[i] <= 0.0 and old_drive[i] != new_drive[i]:
            value = _zero_crossing_time(u, old_plus, old_minus, h, i)
            if value is not None:
                crossing_channels[i] = True
                crossing = value if crossing is None else min(crossing, value)
    state.reversal_phase[:] = 0
    state.reversal_phase[old_drive > 0.0] = 1
    state.reversal_phase[old_drive < 0.0] = -1
    tau, flags, tau_min, tau_max = _ordered_torque_projection(ap, am, s, s_dot, tau_previous, h)
    flags["zero_crossing"] = crossing_channels
    reported_drive = new_drive.copy()
    # A crossing is an explicit sampled event.  The continuous activation
    # state remains the exact post-substep state; the public drive sample marks
    # the event at zero so downstream diagnostics can identify the sample at
    # or immediately after the analytic crossing without sign teleportation.
    reported_drive[crossing_channels] = 0.0
    state.tau_prev[:] = tau
    state.previous_command[:] = u
    state.override_flags = _copy_override_flags(flags)
    state._validate()
    return {
        "drive": reported_drive,
        "a_plus": ap.copy(),
        "a_minus": am.copy(),
        "tau": tau.copy(),
        "tau_previous": tau_previous,
        "override_flags": _copy_override_flags(flags),
        "reversal_phase": state.reversal_phase.copy(),
        "capacity_lower": tau_min.copy(),
        "capacity_upper": tau_max.copy(),
        "zero_crossing_time_s": crossing,
    }



__all__ = [
    "DRIVE_PARAMS",
    "FV_MIN",
    "F_ECC",
    "F_ECC_MAX",
    "F_MIN",
    "OMEGA_MAX_NEG",
    "OMEGA_MAX_POS",
    "POWER_NEG",
    "POWER_POS",
    "RATE_MAX",
    "RHO",
    "TAU_ACT_NEG",
    "TAU_ACT_POS",
    "TAU_BAR_NEG",
    "TAU_BAR_POS",
    "TAU_DEACT_NEG",
    "TAU_DEACT_POS",
    "TAU_DOWN",
    "TAU_UP",
    "DriveModelError",
    "DriveState",
    "activation_update",
    "capacity_envelope",
    "drive_state_step",
    "project_signed_power",
    "torque_angle",
    "torque_velocity",
]
