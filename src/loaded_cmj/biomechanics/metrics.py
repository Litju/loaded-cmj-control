"""Canonical force-time metrics for the LCMJ V1 8 kHz mechanics trace.

This module consumes physical samples plus event boundaries supplied by the
independent event owner.  It never infers or mutates event state and is not
imported by the public observation or policy paths.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any, Callable

import numpy as np

from loaded_cmj.simulation.constants import GRAVITY_MAGNITUDE, PHYSICS_TIMESTEP_S, TOTAL_MASS_KG


METRIC_CONTRACT_VERSION = "LCMJ-V1-F3-FORCE-TIME-1.0.0"
LOADING_RATE_WINDOW_S = 0.005
LOADING_RATE_FILTER = "none; raw 8 kHz force"
_EPS_TIME = 1.0e-12
_EPS_IMPULSE = 1.0e-12


def _support(sample: Any) -> np.ndarray:
    value = getattr(sample, "whole_support_wrench_N_Nm", None)
    if value is None:
        value = getattr(sample, "support_wrench_N_Nm")
    return np.asarray(value, dtype=np.float64).reshape(6)


def _feet(sample: Any) -> np.ndarray:
    value = getattr(sample, "foot_normal_force_N", None)
    if value is None:
        value = np.asarray(getattr(sample, "plate_wrench_N_Nm"), dtype=np.float64)[:2, 2]
    return np.asarray(value, dtype=np.float64).reshape(2)


def _times(samples: Sequence[Any]) -> np.ndarray:
    if len(samples) < 2:
        raise ValueError("force-time metrics require at least two trace samples")
    times = np.asarray([float(sample.time_s) for sample in samples], dtype=np.float64)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0.0):
        raise ValueError("force-time trace times must be finite and strictly increasing")
    return times


def _value_at(
    samples: Sequence[Any],
    time_s: float,
    getter: Callable[[Any], np.ndarray | float],
    times: np.ndarray | None = None,
) -> np.ndarray | float:
    if times is None:
        times = np.asarray([float(sample.time_s) for sample in samples], dtype=np.float64)
    t = float(time_s)
    if t < times[0] - _EPS_TIME or t > times[-1] + _EPS_TIME:
        raise ValueError("metric boundary lies outside the canonical trace")
    if t <= times[0]:
        return np.asarray(getter(samples[0])).copy() if not np.isscalar(getter(samples[0])) else float(getter(samples[0]))
    if t >= times[-1]:
        return np.asarray(getter(samples[-1])).copy() if not np.isscalar(getter(samples[-1])) else float(getter(samples[-1]))
    right = int(np.searchsorted(times, t, side="left"))
    if abs(times[right] - t) <= _EPS_TIME:
        value = getter(samples[right])
        return np.asarray(value).copy() if not np.isscalar(value) else float(value)
    left = right - 1
    alpha = (t - times[left]) / (times[right] - times[left])
    v0 = np.asarray(getter(samples[left]), dtype=np.float64)
    v1 = np.asarray(getter(samples[right]), dtype=np.float64)
    value = v0 + alpha * (v1 - v0)
    return float(value) if value.ndim == 0 else value


def _nodes(samples: Sequence[Any], start_s: float, end_s: float) -> np.ndarray:
    if end_s <= start_s + _EPS_TIME:
        raise ValueError("metric interval must have positive duration")
    times = _times(samples)
    interior = times[(times > start_s + _EPS_TIME) & (times < end_s - _EPS_TIME)]
    return np.concatenate(([float(start_s)], interior, [float(end_s)])).astype(np.float64)


def _integrate(samples: Sequence[Any], start_s: float, end_s: float, getter: Callable[[Any], np.ndarray | float]) -> np.ndarray:
    nodes = _nodes(samples, start_s, end_s)
    times = _times(samples)
    values = np.asarray([_value_at(samples, t, getter, times) for t in nodes], dtype=np.float64)
    return np.trapezoid(values, nodes, axis=0)


def _phase_force(samples: Sequence[Any], start_s: float, end_s: float) -> dict[str, Any]:
    nodes = _nodes(samples, start_s, end_s)
    times = _times(samples)
    force = np.asarray([_value_at(samples, t, lambda sample: _support(sample)[2], times) for t in nodes], dtype=np.float64)
    feet = np.asarray([_value_at(samples, t, _feet, times) for t in nodes], dtype=np.float64)
    return {
        "start_time_s": float(start_s),
        "end_time_s": float(end_s),
        "duration_s": float(end_s - start_s),
        "sample_count": int(len(nodes)),
        "total_force_peak_N": float(np.max(force)),
        "total_force_mean_N": float(np.trapezoid(force, nodes) / (end_s - start_s)),
        "left_force_peak_N": float(np.max(feet[:, 0])),
        "right_force_peak_N": float(np.max(feet[:, 1])),
        "nodes_s": nodes,
        "force_N": force,
        "feet_N": feet,
    }


def _asymmetry(left: float, right: float) -> float:
    return float(abs(left - right) / max(_EPS_IMPULSE, abs(left) + abs(right)))


def derive_propulsive_metrics(
    samples: Sequence[Any],
    *,
    reversal_time_s: float,
    takeoff_time_s: float,
    supported_index: int | None = None,
    mass_kg: float = TOTAL_MASS_KG,
    gravity_mps2: float = GRAVITY_MAGNITUDE,
) -> dict[str, Any]:
    """Derive propulsion once from raw trace wrenches and event boundaries."""
    reversal = float(reversal_time_s)
    takeoff = float(takeoff_time_s)
    interval = _phase_force(samples, reversal, takeoff)
    support_force = lambda sample: np.asarray(
        [_support(sample)[0], _support(sample)[1], _support(sample)[2] - mass_kg * gravity_mps2],
        dtype=np.float64,
    )
    impulse = _integrate(samples, reversal, takeoff, support_force)
    times = _times(samples)
    start_velocity = np.asarray(_value_at(samples, reversal, lambda sample: sample.com_velocity_mps, times), dtype=np.float64)
    takeoff_velocity = np.asarray(_value_at(samples, takeoff, lambda sample: sample.com_velocity_mps, times), dtype=np.float64)
    delta_p = mass_kg * (takeoff_velocity - start_velocity)
    vertical = float(impulse[2])
    horizontal_ratio = float(np.linalg.norm(impulse[:2]) / max(_EPS_IMPULSE, vertical))
    beta = np.asarray([0.5, 0.5], dtype=np.float64)
    if supported_index is not None:
        quiet = _feet(samples[int(supported_index)])
        if float(np.sum(quiet)) > _EPS_IMPULSE:
            beta = quiet / float(np.sum(quiet))
    bilateral = np.asarray([
        _integrate(samples, reversal, takeoff, lambda sample, foot=foot: _feet(sample)[foot] - beta[foot] * mass_kg * gravity_mps2)
        for foot in (0, 1)
    ], dtype=np.float64)
    residual = float(np.linalg.norm(impulse - delta_p))
    return {
        "interval_start_time_s": reversal,
        "interval_end_time_s": takeoff,
        "interval_duration_s": takeoff - reversal,
        "start_velocity_mps": start_velocity,
        "takeoff_velocity_mps": takeoff_velocity,
        "linear_momentum_start_kgmps": mass_kg * start_velocity,
        "linear_momentum_takeoff_kgmps": mass_kg * takeoff_velocity,
        "net_impulse_Ns": impulse,
        "vertical_impulse_Ns": vertical,
        "J_prop_Ns": vertical,
        "takeoff_vertical_velocity_impulse_mps": float(start_velocity[2] + vertical / mass_kg),
        "horizontal_impulse_ratio": horizontal_ratio,
        "bilateral_vertical_impulse_Ns": bilateral,
        "bilateral_asymmetry": _asymmetry(float(bilateral[0]), float(bilateral[1])),
        "impulse_momentum_residual_Ns": residual,
        "impulse_momentum_residual_relative": residual / max(_EPS_IMPULSE, float(np.linalg.norm(impulse))),
        "ballistic_height_diagnostic_m": max(0.0, float(start_velocity[2] + vertical / mass_kg)) ** 2 / (2.0 * gravity_mps2),
        "force_plate_interval_sample_count": interval["sample_count"],
        "integration_method": "trapezoidal_endpoint_interpolation",
        "mass_kg": float(mass_kg),
        "support_wrench_includes_off_plate": True,
        "phase_force": interval,
    }


def _loading_rate(samples: Sequence[Any], start_s: float, end_s: float) -> dict[str, Any]:
    nodes = _nodes(samples, start_s, end_s)
    times = _times(samples)
    force = np.asarray([_value_at(samples, t, lambda sample: _support(sample)[2], times) for t in nodes], dtype=np.float64)
    slopes = np.diff(force) / np.diff(nodes)
    return {
        "method": "maximum_positive_forward_secant",
        "start_event": "valid_landing",
        "end_reference": "min(valid_landing + 0.005 s, impact_absorption)",
        "window_start_s": float(start_s),
        "window_end_s": float(end_s),
        "raw_force": True,
        "filter": LOADING_RATE_FILTER,
        "impact_discontinuity": "do not use a pre-contact sample; slopes begin at the interpolated landing boundary",
        "loading_rate_N_per_s": float(max(0.0, np.max(slopes))),
        "peak_force_N": float(np.max(force)),
    }


def derive_force_time_metrics(
    samples: Sequence[Any],
    event_times_s: Mapping[str, float],
    *,
    landing_window_end_s: float | None = None,
    mass_kg: float = TOTAL_MASS_KG,
    gravity_mps2: float = GRAVITY_MAGNITUDE,
) -> dict[str, Any]:
    """Return the one canonical force-time metric projection.

    Event names are boundaries from `CMJEventDetector`; this function never
    creates or changes them. Landing metrics use a dedicated post-contact
    window and cannot overlap the reversal-to-takeoff propulsion window.
    """
    times = _times(samples)
    events = {str(key): float(value) for key, value in event_times_s.items()}
    required = ("upward_reversal", "valid_takeoff")
    missing = [name for name in required if name not in events]
    if missing:
        raise ValueError(f"canonical metrics missing event boundaries: {missing}")
    reference_time = events.get("supported_start", float(times[0]))
    reference = _phase_force(samples, reference_time, min(float(times[-1]), reference_time + PHYSICS_TIMESTEP_S))
    bodyweight = float(mass_kg * gravity_mps2)
    reference_force = float(_value_at(samples, reference_time, lambda sample: _support(sample)[2], times))
    reference_feet = np.asarray(_value_at(samples, reference_time, _feet, times), dtype=np.float64)

    onset = events.get("countermovement_onset")
    reversal = events["upward_reversal"]
    min_force = None
    min_force_time = None
    if onset is not None and reversal > onset + _EPS_TIME:
        phase = _phase_force(samples, onset, reversal)
        min_index = int(np.argmin(phase["force_N"]))
        min_force = float(phase["force_N"][min_index])
        min_force_time = float(phase["nodes_s"][min_index])

    braking_start = float(min_force_time if min_force_time is not None else (onset if onset is not None else reversal))
    braking = _phase_force(samples, braking_start, reversal) if reversal > braking_start + _EPS_TIME else None
    propulsion = derive_propulsive_metrics(
        samples, reversal_time_s=reversal, takeoff_time_s=events["valid_takeoff"],
        supported_index=None, mass_kg=mass_kg, gravity_mps2=gravity_mps2,
    )
    landing_start = events.get("valid_landing")
    landing = None
    landing_rate = None
    if landing_start is not None:
        if landing_start <= events["valid_takeoff"] + _EPS_TIME:
            raise ValueError("landing boundary is not strictly after takeoff")
        default_end = min(float(times[-1]), float(landing_start) + LOADING_RATE_WINDOW_S)
        landing_end = float(landing_window_end_s if landing_window_end_s is not None else events.get("impact_absorption", default_end))
        landing_end = min(float(times[-1]), max(default_end, landing_end))
        landing = _phase_force(samples, float(landing_start), landing_end)
        landing_rate = _loading_rate(samples, float(landing_start), min(landing_end, float(landing_start) + LOADING_RATE_WINDOW_S))

    propulsion_nodes = _nodes(samples, reversal, events["valid_takeoff"])
    landing_nodes = None if landing is None else landing["nodes_s"]
    leak = bool(landing_nodes is not None and np.any(landing_nodes <= events["valid_takeoff"] + _EPS_TIME))
    out: dict[str, Any] = {
        "contract_version": METRIC_CONTRACT_VERSION,
        "sample_rate_Hz": 1.0 / float(PHYSICS_TIMESTEP_S),
        "sample_dt_s": float(PHYSICS_TIMESTEP_S),
        "mass_kg": float(mass_kg),
        "gravity_mps2": float(gravity_mps2),
        "stance_reference": {
            "loaded_bodyweight_N": bodyweight,
            "reference_time_s": reference_time,
            "measured_total_force_N": reference_force,
            "measured_left_right_force_N": reference_feet,
            "reference_phase": reference,
        },
        "unweighting": {
            "interval_start_time_s": float(onset) if onset is not None else None,
            "interval_end_time_s": min_force_time,
            "minimum_vertical_force_N": min_force,
            "time_of_minimum_force_s": min_force_time,
            "duration_s": None if onset is None or min_force_time is None else float(min_force_time - onset),
        },
        "braking": None if braking is None else {
            "interval_start_time_s": braking["start_time_s"],
            "interval_end_time_s": braking["end_time_s"],
            "duration_s": braking["duration_s"],
            "net_vertical_braking_impulse_Ns": float(_integrate(samples, braking_start, reversal, lambda sample: _support(sample)[2] - bodyweight)),
            "phase_peak_force_N": braking["total_force_peak_N"],
            "phase_mean_force_N": braking["total_force_mean_N"],
        },
        "propulsion": {
            "reversal_to_takeoff_interval": {key: value for key, value in propulsion.items() if key != "phase_force"},
            "J_prop_Ns": float(propulsion["vertical_impulse_Ns"]),
            "takeoff_velocity_from_impulse_mps": float(propulsion["takeoff_vertical_velocity_impulse_mps"]),
            "phase_peak_force_N": propulsion["phase_force"]["total_force_peak_N"],
            "phase_mean_force_N": propulsion["phase_force"]["total_force_mean_N"],
            "left_propulsive_impulse_Ns": float(propulsion["bilateral_vertical_impulse_Ns"][0]),
            "right_propulsive_impulse_Ns": float(propulsion["bilateral_vertical_impulse_Ns"][1]),
            "bilateral_asymmetry": float(propulsion["bilateral_asymmetry"]),
        },
        "landing": None if landing is None else {
            "window_start_time_s": float(landing["start_time_s"]),
            "window_end_time_s": float(landing["end_time_s"]),
            "landing_impulse_Ns": float(_integrate(samples, landing["start_time_s"], landing["end_time_s"], lambda sample: _support(sample)[2])),
            "landing_net_vertical_impulse_Ns": float(_integrate(samples, landing["start_time_s"], landing["end_time_s"], lambda sample: _support(sample)[2] - bodyweight)),
            "peak_total_force_N": landing["total_force_peak_N"],
            "peak_left_force_N": landing["left_force_peak_N"],
            "peak_right_force_N": landing["right_force_peak_N"],
            "left_right_asymmetry": _asymmetry(landing["left_force_peak_N"], landing["right_force_peak_N"]),
            "loading_rate": landing_rate,
        },
        "force_time_morphology": {
            "trace_start_time_s": float(times[0]),
            "trace_end_time_s": float(times[-1]),
            "trace_duration_s": float(times[-1] - times[0]),
            "raw_wrench_owner": "BiomechanicalSample.plate_wrench_N_Nm",
            "total_force_owner": "BiomechanicalSample.whole_support_wrench_N_Nm",
        },
        "landing_propulsion_leak": {
            "landing_samples_in_propulsion_interval": int(np.count_nonzero(landing_nodes is not None and landing_nodes <= events["valid_takeoff"] + _EPS_TIME)),
            "pass": not leak,
        },
    }
    # Keep the explicit intervals in the diagnostic output for independent
    # auditing; the raw force-time arrays are not replaced by these summaries.
    out["intervals"] = {
        "propulsion_nodes_s": propulsion_nodes,
        "landing_nodes_s": landing_nodes,
        "propulsion_start_s": reversal,
        "propulsion_end_s": events["valid_takeoff"],
        "landing_start_s": landing_start,
    }
    return out


def native(value: Any) -> Any:
    """Convert metric output to JSON-native values for evidence writers."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): native(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [native(item) for item in value]
    return value
