"""Predictive sagittal support-reserve diagnostics for F3.

This module contains a bounded mechanics diagnostic only.  It does not infer
events, select actions, or modify the public policy observation contract.
Support geometry is supplied by the retained Plant support-polygon owner; the
module performs only the finite-horizon kinematics and reserve calculation.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import copysign, isfinite


CAPTURABILITY_CONTRACT_VERSION = "LCMJ-V1-F3-CAPTURABILITY-1.0.0"


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _support_reserve(com_x_m: float, support_bounds_m: tuple[float, float]) -> float:
    lower, upper = support_bounds_m
    return min(com_x_m - lower, upper - com_x_m)


@dataclass(frozen=True)
class SupportReservePrediction:
    """One deterministic finite-horizon sagittal reserve prediction.

    Positions and reserves are metres, velocities are m/s, accelerations are
    m/s^2, horizon is seconds, external wrench is N and N m, centroidal
    angular momentum is kg m^2/s, and trunk pitch is radians.  The scalar
    reserve is the support-interval distance in the sagittal world-X
    projection; positive is inside and negative is beyond an edge.
    """

    contract_version: str
    T_horizon_s: float
    com_x_m: float
    com_vx_mps: float
    com_vz_mps: float
    feasible_upward_acceleration_mps2: float
    feasible_horizontal_deceleration_mps2: float
    support_geometry_bounds_m: tuple[float, float]
    current_admissible_support_reserve_m: float
    predicted_com_drift_m: float
    predicted_com_x_m: float
    predicted_support_reserve_m: float
    actual_support_reserve_at_horizon_m: float | None
    prediction_error_m: float | None
    external_wrench_N_Nm: tuple[float, ...] | None
    centroidal_pitch_angular_momentum_kgm2ps: float | None
    trunk_pitch_rad: float | None
    posture_support_correction_m: float

def predict_support_reserve(
    *,
    com_x_m: float,
    com_vx_mps: float,
    com_vz_mps: float,
    feasible_upward_acceleration_mps2: float,
    feasible_horizontal_deceleration_mps2: float,
    support_geometry_bounds_m: tuple[float, float],
    current_admissible_support_reserve_m: float | None = None,
    actual_com_x_at_horizon_m: float | None = None,
    external_wrench_N_Nm: tuple[float, ...] | None = None,
    centroidal_pitch_angular_momentum_kgm2ps: float | None = None,
    trunk_pitch_rad: float | None = None,
) -> SupportReservePrediction:
    """Predict reserve through the vertical stopping/reversal horizon.

    ``T_horizon_s`` is the time required to arrest a downward COM velocity
    using the supplied feasible upward acceleration.  A currently upward COM
    has zero reversal horizon for this diagnostic.  Sagittal drift uses the
    supplied feasible horizontal deceleration, clamped at horizontal stop;
    with zero horizontal capability it is inertial drift.

    The reserve equation is deliberately explicit and unweighted:

    ``predicted_reserve = current_reserve - abs(predicted_drift)``.

    No posture or wrench score is hidden in that equation.  Those quantities
    are retained as separately reported diagnostic context and the correction
    term is fixed at zero in V1.
    """

    x = _finite("com_x_m", com_x_m)
    vx = _finite("com_vx_mps", com_vx_mps)
    vz = _finite("com_vz_mps", com_vz_mps)
    upward = _finite("feasible_upward_acceleration_mps2", feasible_upward_acceleration_mps2)
    horizontal = _finite("feasible_horizontal_deceleration_mps2", feasible_horizontal_deceleration_mps2)
    if upward <= 0.0:
        raise ValueError("feasible_upward_acceleration_mps2 must be positive")
    if horizontal < 0.0:
        raise ValueError("feasible_horizontal_deceleration_mps2 must be non-negative")
    if len(support_geometry_bounds_m) != 2:
        raise ValueError("support geometry requires lower and upper sagittal bounds")
    lower = _finite("support_geometry_lower_m", support_geometry_bounds_m[0])
    upper = _finite("support_geometry_upper_m", support_geometry_bounds_m[1])
    if not lower < upper:
        raise ValueError("support geometry lower bound must be below upper bound")
    bounds = (lower, upper)
    current = (
        _support_reserve(x, bounds)
        if current_admissible_support_reserve_m is None
        else _finite("current_admissible_support_reserve_m", current_admissible_support_reserve_m)
    )

    horizon = max(0.0, -vz / upward)
    if horizontal == 0.0 or vx == 0.0:
        drift = vx * horizon
    else:
        horizontal_stop_s = abs(vx) / horizontal
        braking_time_s = min(horizon, horizontal_stop_s)
        drift = vx * braking_time_s - 0.5 * copysign(horizontal, vx) * braking_time_s**2
    predicted_x = x + drift
    predicted_reserve = current - abs(drift)

    actual_reserve: float | None = None
    error: float | None = None
    if actual_com_x_at_horizon_m is not None:
        actual_x = _finite("actual_com_x_at_horizon_m", actual_com_x_at_horizon_m)
        actual_reserve = _support_reserve(actual_x, bounds)
        error = predicted_reserve - actual_reserve

    wrench = None
    if external_wrench_N_Nm is not None:
        wrench = tuple(_finite("external_wrench_N_Nm", value) for value in external_wrench_N_Nm)
    h_pitch = None if centroidal_pitch_angular_momentum_kgm2ps is None else _finite(
        "centroidal_pitch_angular_momentum_kgm2ps", centroidal_pitch_angular_momentum_kgm2ps
    )
    trunk = None if trunk_pitch_rad is None else _finite("trunk_pitch_rad", trunk_pitch_rad)

    return SupportReservePrediction(
        contract_version=CAPTURABILITY_CONTRACT_VERSION,
        T_horizon_s=horizon,
        com_x_m=x,
        com_vx_mps=vx,
        com_vz_mps=vz,
        feasible_upward_acceleration_mps2=upward,
        feasible_horizontal_deceleration_mps2=horizontal,
        support_geometry_bounds_m=bounds,
        current_admissible_support_reserve_m=current,
        predicted_com_drift_m=drift,
        predicted_com_x_m=predicted_x,
        predicted_support_reserve_m=predicted_reserve,
        actual_support_reserve_at_horizon_m=actual_reserve,
        prediction_error_m=error,
        external_wrench_N_Nm=wrench,
        centroidal_pitch_angular_momentum_kgm2ps=h_pitch,
        trunk_pitch_rad=trunk,
        posture_support_correction_m=0.0,
    )


__all__ = [
    "CAPTURABILITY_CONTRACT_VERSION",
    "SupportReservePrediction",
    "predict_support_reserve",
]
