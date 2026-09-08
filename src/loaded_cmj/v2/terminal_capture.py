"""V2.1 frozen true-momentum terminal-capture law (RES-56 executed, RES-58 sealed).

Reusable project authority for RES10_SYNC_TRUE_MOMENTUM_TERMINAL_CAPTURE_S50_002.

Recovered verbatim from RES-56 external evidence archive
EXP-RES10-TRUE-MOMENTUM-CAPTURE-001 / executed_source_archive / tools_res56 /
capture_law.py (SHA256 06e73896cbeab9dd04c49be56ace70f471c40bd3153e17f840e5b543c9fecd4d).
Mathematical behavior is frozen and identical; only module docstring authority
header is extended to record the RES-58 sealing lineage. No constant, equation,
sign, clamp, saturation edge, or function semantic was changed.

Frozen law (no tuning, no search, no alternate path):
  VZ_TARGET  = 0.0 m/s
  CONTROL_DT = 0.005 s
  MASS_KG    = 95.0 kg (RES-54 whole-body authority)
  G          = 9.81 m/s^2
  BW         = MASS_KG * G = 931.95 N
  FZ_MIN     = 0.6 * BW
  FZ_MAX     = 1.5 * BW

At every 5-ms synchronized control update, from corrected whole-body COM
velocity vz_true (RES-54 jacSubtreeCom authority):
  V_ERR    = VZ_TARGET - vz_true
  FZ_RAW   = BW + MASS_KG * V_ERR / CONTROL_DT
           = BW - MASS_KG * vz_true / CONTROL_DT
  FZ_DES   = clamp(FZ_RAW, FZ_MIN, FZ_MAX)

Consistent next-step COM-vz target for the sealed RES-52 full-7DOF
exact-forward inner controller (same commanded force, corrected whole-body
dynamics, explicit Euler over CONTROL_DT):
  vz_target_next = vz_true + CONTROL_DT * (FZ_DES / MASS_KG - G)

Interpretation:
  vz_true < 0 -> Fz > BW to arrest descent
  vz_true = 0 -> Fz = BW
  vz_true > 0 -> Fz < BW to remove upward overshoot

Saturation edges (analytic, verified by sympy in RES-58):
  FZ hits FZ_MAX for vz <= -0.5*g*DT = -0.024525 m/s
  FZ hits FZ_MIN for vz >= +0.4*g*DT = +0.019620 m/s
Linear band between is the one-step deadbeat solution of
  m*(0 - vz_true) = (FZ_DES - m*g)*DT.

Realization authority: sealed RES-52 SoftContactPolicy only. No direct
force/root/qpos/qvel writes. No event/scorer circularity. No E10/E11/E12.
Support semantics: RES-57 bilateral-support continuity contract
(SUPPORT_CONTINUITY_CONTRACT_SHA256=cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734,
CONTROL_RELEVANT_LOSS_DWELL=0.005 s = 40 physics steps = one control
interval / control-relevance dwell; NOT a Nyquist period).

Prohibitions (frozen):
- No FZ_MIN/FZ_MAX/VZ_TARGET/CONTROL_DT tuning.
- No fixed-profile-integral COM-vz target when it conflicts with this law.
- No confusion of desired (commanded) vs actual (measured) Fz.
- No Plant/contact/solver/actuator/event/scorer/takeoff/propulsion changes.
"""

from __future__ import annotations

MASS_KG: float = 95.0
G: float = 9.81
BW_N: float = MASS_KG * G  # 931.95
CONTROL_DT: float = 0.005
VZ_TARGET: float = 0.0
FZ_MIN_N: float = 0.6 * BW_N
FZ_MAX_N: float = 1.5 * BW_N

# Analytic saturation edges (m/s), derived from the clamp equalities.
VZ_SAT_HIGH: float = -0.5 * G * CONTROL_DT  # -0.024525 -> FZ_MAX below this
VZ_SAT_LOW: float = 0.4 * G * CONTROL_DT  # +0.019620 -> FZ_MIN above this

# Sealing lineage (metadata only; not part of the control math).
RES56_ARCHIVED_SOURCE_SHA256 = "06e73896cbeab9dd04c49be56ace70f471c40bd3153e17f840e5b543c9fecd4d"
RES56_MISSION = "RES10_SYNC_TRUE_MOMENTUM_TERMINAL_CAPTURE_S50_001"
RES58_MISSION = "RES10_SYNC_TRUE_MOMENTUM_TERMINAL_CAPTURE_S50_002"
LAW_IDENTITY = "FZ_RAW=BW-m*vz_true/CONTROL_DT; FZ_DES=clamp(FZ_RAW,0.6BW,1.5BW); vz_target_next=vz_true+CONTROL_DT*(FZ_DES/m-g)"


def capture_force_command(vz_true_mps: float) -> float:
    """Commanded whole-body support force FZ_DES in Newtons (frozen law)."""
    fz_raw = BW_N - MASS_KG * float(vz_true_mps) / CONTROL_DT
    if fz_raw < FZ_MIN_N:
        return float(FZ_MIN_N)
    if fz_raw > FZ_MAX_N:
        return float(FZ_MAX_N)
    return float(fz_raw)


def capture_vz_target_next(vz_true_mps: float, fz_des_n: float) -> float:
    """Consistent next-step COM-vz target (m/s) for the inner layer."""
    return float(vz_true_mps) + CONTROL_DT * (float(fz_des_n) / MASS_KG - G)


def capture_command(vz_true_mps: float) -> tuple[float, float]:
    """Joint outer command: (FZ_DES_N, VZ_TARGET_NEXT_MPS)."""
    fz = capture_force_command(vz_true_mps)
    return fz, capture_vz_target_next(vz_true_mps, fz)
