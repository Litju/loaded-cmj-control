"""V3 landing-phase metric primitives (RES-86A).

Authority: ``LCMJ_RES86_V3_LANDING_ACCEPTANCE_AUTHORITY_V1``.

This module exposes the *measurement primitives* used by the RES-86 landing
acceptance authority.  It never redefines contact, support or SYSTEM_COM
semantics: contact records come from :mod:`loaded_cmj.v3.measurement`, support
state is the RES-84 ``legal_plantar_active`` count, and the SYSTEM_COM is the
RES-84 mass state.

The centroidal angular momentum convention is the one already sealed by the
RES-85E telemetry (``centroidal_H``/``centroidal_Hy``)::

    H = sum_b [ R_b diag(I_b) R_b^T omega_b + (xipos_b - com) x (m_b v_b) ]

with ``omega_b``/``v_b`` from ``mj_objectVelocity`` in the world frame and the
sum over every non-world body (including the jointless 20 kg external load).
``centroidal_hy_kg_m2_s`` reproduces the sealed per-sample values bit-exactly.
"""

from __future__ import annotations

from typing import Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import measurement as M

V3_LANDING_METRICS_AUTHORITY_ID = "LCMJ_RES86_V3_LANDING_METRICS_V1"


def centroidal_angular_momentum_world(model: mujoco.MjModel, data: mujoco.MjData,
                                      com_world_m: Sequence[float]) -> np.ndarray:
    """Whole-body centroidal angular momentum about ``com_world_m`` (world).

    Sealed RES-85E convention; the sum covers every non-world body.
    """
    center = np.asarray(com_world_m, dtype=np.float64).reshape(3)
    h = np.zeros(3, dtype=np.float64)
    vel = np.zeros(6, dtype=np.float64)
    for body_id in range(1, model.nbody):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, int(body_id), vel, 0)
        rot = np.asarray(data.ximat[body_id], dtype=np.float64).reshape(3, 3)
        inertia = np.diag(np.asarray(model.body_inertia[body_id], dtype=np.float64))
        h += (rot @ inertia @ rot.T) @ vel[:3]
        h += np.cross(np.asarray(data.xipos[body_id], dtype=np.float64) - center,
                      float(model.body_mass[body_id]) * vel[3:6])
    return h


def centroidal_hy_kg_m2_s(plant, data: mujoco.MjData) -> float:
    """Sagittal (world y) whole-body centroidal angular momentum, kg*m^2/s."""
    com = M.system_com_state(plant, data).com_world_m
    return float(centroidal_angular_momentum_world(plant.model, data, com)[1])


def max_penetration_m(records: Sequence[M.V3ContactRecord]) -> float:
    """Maximum detected-contact penetration ``max(0, -dist)`` (metres)."""
    return max((float(r.penetration_m) for r in records), default=0.0)


def support_indicator(frames: Sequence[M.V3NativeFrame]) -> np.ndarray:
    """Boolean legal plantar support per native sample (RES-84 count > 0)."""
    return np.asarray([f.legal_plantar_active > 0 for f in frames], dtype=bool)


def chatter_transition_count(support: np.ndarray, start_index: int) -> int:
    """Number of legal-support on/off transitions from ``start_index`` on.

    A chatter transition is one native sample ``k >= start_index`` whose
    boolean legal-support state differs from sample ``k - 1``.
    """
    state = np.asarray(support, dtype=bool)
    if state.size < 2 or start_index < 1:
        return 0
    return int(np.count_nonzero(state[start_index:] != state[start_index - 1:-1]))


def support_free_intervals(support: np.ndarray, start_index: int) -> list[tuple[int, int]]:
    """Maximal zero-legal-support intervals ``[first, last]`` after ``start``.

    ``start_index`` is the landing first-contact sample; transitions before it
    are never counted.  A trailing open interval is reported up to the last
    sample of the stream.
    """
    state = np.asarray(support, dtype=bool)
    intervals: list[tuple[int, int]] = []
    run_start: int | None = None
    for k in range(int(start_index), state.size):
        if not state[k]:
            if run_start is None:
                run_start = k
        elif run_start is not None:
            intervals.append((run_start, k - 1))
            run_start = None
    if run_start is not None:
        intervals.append((run_start, state.size - 1))
    return intervals


def material_reflight_intervals(support: np.ndarray, start_index: int,
                                min_samples: int) -> list[tuple[int, int]]:
    """Support-free intervals whose length is at least ``min_samples``.

    The declared material-reflight threshold in the RES-86 authority is the
    bilateral-establishment dwell ``D_BL``; shorter losses are chatter.
    """
    return [(a, b) for a, b in support_free_intervals(support, start_index)
            if (b - a + 1) >= int(min_samples)]


__all__ = [
    "V3_LANDING_METRICS_AUTHORITY_ID",
    "centroidal_angular_momentum_world",
    "centroidal_hy_kg_m2_s",
    "chatter_transition_count",
    "material_reflight_intervals",
    "max_penetration_m",
    "support_free_intervals",
    "support_indicator",
]
