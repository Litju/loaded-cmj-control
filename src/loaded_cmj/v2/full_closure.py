#!/usr/bin/env python3
"""V2.1 frozen full E1->E12 controller composition (RES-76, single composition).

MISSION=RES10_SYNC_FULL_E1_E12_CLOSURE_001
EXPERIMENT_ID=EXP-RES10-SYNC-FULL-E1-E12-CLOSURE-001

One-way composition without state restoration:
  PRELANDING (Res72Policy incl. C01/RES58 TERMINAL)
  -> BALANCE (exact RES-73 BalanceController)
  -> RECOVERY (exact RES-74 StableRecoveryController RISE/SETTLE/HANDOFF)
  -> STANDING_HOLD (exact RES43 inside RECOVERY HANDOFF)

Transitions use committed controller-local physical predicates + dwell/timer
semantics, evaluated at physics rate on synchronized samples, switching at the
exact physics sample where dwell completes (truncated control interval,
phase-local 5-ms grid reset). No V2EventDetector/scorer object is used as a
controller input. Scorer runs observationally in parallel.

Frozen authorities: RES-54/55/57/58, C01, RES-72/73/74, RES43, scorer predicates.
No tuning, no search, no direct live writes, no root assistance.
"""
from __future__ import annotations
import numpy as np

# Frozen dwell lengths (physics samples, DT=0.000125)
E10_DWELL_SAMPLES = 160  # 0.020 s impact_absorption
RR_DWELL_S = 0.10
HANDOFF_SUSTAIN_S = 0.05  # inside StableRecoveryController (committed)

# Frozen thresholds (mirrors, not imports, to avoid scorer circularity)
E10_VZ_ABS_MAX = 0.05
E10_BILATERAL_FZ_MIN_N = 10.0

# RECOVERY_READY thresholds from recovery_ready_spec.json (SPEC 0e6638aa...)
RR_THRESHOLDS = {
    "COM_VX_ABS_MAX": 0.15,
    "COM_VZ_ABS_MAX": 0.05,
    "HY_ABS_MAX": 1.5,
    "ROOT_PITCH_RATE_ABS_MAX": 0.5,
    "WORLD_TRUNK_TILT_RATE_ABS_MAX": 1.0,
    "JOINT_QDOT_ABS_MAX": 1.0,
    "SUPPORT_MARGIN_MIN": 0.05,
    "BILATERAL_FZ_MIN_N": 10.0,
    "PEN_MAX_M": 0.01,
}

POLICY_IDENTITY = (
    "PRELANDING=Res72Policy exact (HOLD/SUPPORTED/FLIGHT/LANDING_PREP/IMPACT/TERMINAL+C01+RES58+RES52); "
    "E10->BALANCE: |vz|<0.05 & bilateral>10, 160 physics samples, TERMINAL-active, switch at dwell sample (truncate); "
    "BALANCE=BalanceController exact T_BAL=0.27; "
    "BALANCE->RECOVERY: RR thresholds+dwell 0.10s (800 samples), switch at dwell sample (truncate); "
    "RECOVERY=StableRecoveryController exact T_RISE=6.375 manifold13 HANDOFF 0.05s; "
    "HANDOFF=RES43 exact q0 Kp400 Kd10 ff0"
)

def e10_predicate_ok(com_vz: float, left_fz: float, right_fz: float, fall: bool, prohib: bool) -> bool:
    return bool(abs(float(com_vz)) < E10_VZ_ABS_MAX
                and float(left_fz) > E10_BILATERAL_FZ_MIN_N
                and float(right_fz) > E10_BILATERAL_FZ_MIN_N
                and not bool(fall) and not bool(prohib))

def rr_predicate_ok(com_vx: float, com_vz: float, hy: float, root_ry: float,
                    trunkW: float, qdot_max: float, margin: float,
                    left_fz: float, right_fz: float, pen: float) -> bool:
    th = RR_THRESHOLDS
    return bool(abs(float(com_vx)) <= float(th["COM_VX_ABS_MAX"])
                and abs(float(com_vz)) <= float(th["COM_VZ_ABS_MAX"])
                and abs(float(hy)) <= float(th["HY_ABS_MAX"])
                and abs(float(root_ry)) <= float(th["ROOT_PITCH_RATE_ABS_MAX"])
                and abs(float(trunkW)) <= float(th["WORLD_TRUNK_TILT_RATE_ABS_MAX"])
                and float(qdot_max) <= float(th["JOINT_QDOT_ABS_MAX"])
                and float(margin) >= float(th["SUPPORT_MARGIN_MIN"])
                and float(left_fz) > float(th["BILATERAL_FZ_MIN_N"])
                and float(right_fz) > float(th["BILATERAL_FZ_MIN_N"])
                and float(pen) <= float(th["PEN_MAX_M"]))
