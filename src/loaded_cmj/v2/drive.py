"""V2 transparent bounded torque drive.

No hidden state. No activation, no power clipping.
tau = limit * u,  u in [-1,1].
Fail-closed on nonfinite or out-of-bounds.
"""

from __future__ import annotations

import numpy as np

from loaded_cmj.v2.constants import V2_TORQUE_LIMITS_NM, V2_MJ_ACTUATOR_NAMES

_LIMITS = np.array(
    [V2_TORQUE_LIMITS_NM[name] for name in (
        "lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"
    )],
    dtype=np.float64,
)
# map physical channel index -> limit
# order: lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle
LIMITS = _LIMITS.copy()
ACTUATOR_NAMES = V2_MJ_ACTUATOR_NAMES

def action_to_torque(action: np.ndarray) -> np.ndarray:
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape[0] != len(LIMITS):
        raise ValueError(f"expected {len(LIMITS)}-D action, got {a.shape}")
    if not np.isfinite(a).all():
        raise ValueError("action nonfinite")
    if np.any(a < -1.0 - 1e-12) or np.any(a > 1.0 + 1e-12):
        raise ValueError(f"action outside [-1,1]: {a}")
    tau = LIMITS * np.clip(a, -1.0, 1.0)
    if not np.isfinite(tau).all():
        raise ValueError("torque nonfinite")
    return tau

def check_torque_feasible(tau: np.ndarray) -> bool:
    t = np.asarray(tau, dtype=np.float64).reshape(-1)
    if t.shape[0] != len(LIMITS):
        return False
    return bool(np.all(np.abs(t) <= LIMITS + 1e-9) and np.isfinite(t).all())

def capacity_envelope() -> tuple[np.ndarray, np.ndarray]:
    """Return (tau_min, tau_max) for diagnostics."""
    return -LIMITS.copy(), LIMITS.copy()
