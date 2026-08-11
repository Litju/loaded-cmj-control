"""Hybrid CMJ hybrid events, terminations, and raw physical metrics.

This module is the CMJ evaluation boundary.  It consumes physics-substep
physical samples and never consumes an action or a controller assertion. The
public :class:`BiomechanicalSample`
is deliberately a measurement record: bilateral six-axis virtual
force-plate wrenches, complete-system COM state, contact classification, and
the conformance diagnostics needed by the final authority.

The implementation follows the frozen event DAG:

* contact latches use F_on/F_off hysteresis and confirmation backdating;
* crossings are linearly interpolated, while dwell is counted continuously at
  physics substeps;
* event order is monotone and later gates are not evaluated as successes after
  an earlier gate fails;
* all force contributions, including off-plate and prohibited contacts, remain
  in the support-wrench and impulse sums;
* non-success terminal classes are disjoint: ``PHYSICAL_FALL``,
  ``PHYSICS_NONFINITE_FAULT``, ``INCOMPLETE_HORIZON``, ``AGENT_FAULT``, and
  ``INTERNAL_EVALUATION_ERROR``.

All event thresholds are read through the model registry in
``simulation.constants``. This module does not tune them or aggregate an
objective score.
"""

from __future__ import annotations

import math
from bisect import bisect_left
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from itertools import pairwise
from typing import Any
from types import MappingProxyType

import numpy as np
from loaded_cmj.simulation.constants import (
    CONTACT_F_ACTIVE_N,
    CONTACT_F_OFF_N,
    CONTACT_F_ON_N,
    CONTACT_N_OFF,
    CONTACT_N_ON,
    CONTACT_T_OFF_S,
    CONTACT_T_ON_S,
    EVENT_NAMES,
    EVENT_THRESHOLDS,
    GRAVITY_MAGNITUDE,
    PHYSICS_TIMESTEP_S,
    TERMINATION_THRESHOLDS,
    TOTAL_MASS_KG,
)
from loaded_cmj.biomechanics.metrics import derive_propulsive_metrics

BASELINE_DT_S = float(PHYSICS_TIMESTEP_S)
MASS_KG = float(TOTAL_MASS_KG)
GRAVITY_MPS2 = float(GRAVITY_MAGNITUDE)
MASS_WEIGHT_N = MASS_KG * GRAVITY_MPS2

_EPS_TIME = 1.0e-12
_EPS_FORCE = 1.0e-9
_EPS_IMPULSE = 1.0e-12
_EPS_LIMIT_WORK_J = 1.0e-5


EventName = Enum(
    "EventName",
    {name.upper(): name for name in EVENT_NAMES},
    type=str,
)


class TerminationClass(str, Enum):
    """Terminal outcomes with distinct physics, worker, and evaluator faults."""

    OBJECTIVE_COMPLETE = "OBJECTIVE_COMPLETE"
    PHYSICAL_FALL = "PHYSICAL_FALL"
    PHYSICS_NONFINITE_FAULT = "PHYSICS_NONFINITE_FAULT"
    INCOMPLETE_HORIZON = "INCOMPLETE_HORIZON"
    AGENT_FAULT = "AGENT_FAULT"
    INTERNAL_EVALUATION_ERROR = "INTERNAL_EVALUATION_ERROR"


_CANONICAL_THRESHOLDS = MappingProxyType(
    {**EVENT_THRESHOLDS, **TERMINATION_THRESHOLDS}
)
_EVENT_THRESHOLD_KEYS = MappingProxyType({
    "quiet_dwell_s": "E1_dwell_s",
    "reset_foot_load_fraction": "E1_force_floor_bw",
    "reset_com_margin_m": "E1_com_margin_m",
    "reset_trunk_tilt_max_rad": "E1_trunk_tilt_max_rad",
    "quiet_com_speed_mps": "E1_speed_max_mps",
    "onset_vz_on_mps": "E2_vz_on_mps",
    "onset_vz_off_mps": "E2_vz_off_mps",
    "onset_dwell_s": "E2_dwell_s",
    "onset_force_floor_bw": "E2_force_floor_bw",
    "minimum_descent_s": "E3_minimum_descent_s",
    "zero_band_mps": "E4_vz_up_mps",
    "reversal_down_band_mps": "E4_vz_down_mps",
    "reversal_dwell_s": "E4_dwell_s",
    "countermovement_depth_min_m": "E3_depth_m",
    "countermovement_force_floor_bw": "E3_force_floor_bw",
    "countermovement_max_descent_speed_mps": "E3_max_descent_speed_mps",
    "countermovement_dwell_s": "E3_dwell_s",
    "takeoff_vz_min_mps": "E6_takeoff_vz_min_mps",
    "propulsion_force_floor_bw": "E5_force_bw",
    "propulsion_dwell_s": "E5_dwell_s",
    "takeoff_dwell_s": "E6_dwell_s",
    "horizontal_impulse_ratio_max": "E5_horizontal_impulse_ratio_max",
    "propulsion_asymmetry_max": "E5_propulsion_asymmetry_max",
    "impulse_residual_relative_max": "E5_impulse_residual_relative_max",
    "mechanics_residual_trans_max_N": "E5_mechanics_residual_trans_max_N",
    "mechanics_residual_rot_max_Nm": "E5_mechanics_residual_rot_max_Nm",
    "release_skew_max_s": "E6_release_skew_max_s",
    "flight_dwell_min_s": "E7_dwell_s",
    "apex_dwell_s": "E8_dwell_s",
    "sole_separation_min_m": "E7_sole_separation_min_m",
    "ballistic_position_residual_max_m": "E7_ballistic_position_residual_max_m",
    "ballistic_velocity_residual_max_mps": "E7_ballistic_velocity_residual_max_mps",
    "flight_tilt_max_rad": "E7_trunk_tilt_max_rad",
    "landing_descent_min_mps": "E9_landing_descent_min_mps",
    "landing_skew_max_s": "E9_landing_skew_max_s",
    "landing_dwell_s": "E9_dwell_s",
    "absorption_window_max_s": "E10_absorption_window_max_s",
    "absorption_timeout_s": "E10_timeout_after_recontact_s",
    "absorption_arrest_mps": "E10_vz_arrest_mps",
    "absorption_dwell_s": "E10_dwell_s",
    "rebound_max_mps": "E10_rebound_max_mps",
    "landing_peak_force_multiple": "E10_landing_peak_force_multiple",
    "limit_work_max_j": "E10_limit_work_max_J",
    "recovery_dwell_s": "E12_dwell_s",
    "capture_dwell_s": "E11_dwell_s",
    "recovery_foot_load_fraction": "E11_foot_load_fraction",
    "recovery_com_speed_max_mps": "E11_com_speed_max_mps",
    "recovery_joint_rate_max_radps": "E11_joint_rate_max_radps",
    "recovery_angular_speed_max_radps": "E11_angular_speed_max_radps",
    "recovery_centroidal_h_max_kgm2ps": "E11_centroidal_h_max_kgm2ps",
    "recovery_cop_margin_min_m": "E11_com_margin_m",
    "recovery_rebound_max_mps": "E11_rebound_max_mps",
    "recovery_tilt_max_rad": "E12_trunk_tilt_max_rad",
    "fall_height_ratio": "FALL_height_ratio",
    "fall_tilt_max_rad": "FALL_trunk_tilt_rad",
    "fall_dwell_s": "FALL_dwell_s",
})


@dataclass(frozen=True)
class EventThresholds:
    """Read-only view over the canonical CMJ threshold registry."""

    registry: Mapping[str, float] = _CANONICAL_THRESHOLDS

    def __post_init__(self) -> None:
        if dict(self.registry) != dict(_CANONICAL_THRESHOLDS):
            raise ValueError("CMJ thresholds must be the canonical model registry")
        object.__setattr__(self, "registry", _CANONICAL_THRESHOLDS)

    def __getattr__(self, name: str) -> float:
        try:
            key = _EVENT_THRESHOLD_KEYS[name]
            return float(self.registry[key])
        except KeyError as exc:
            raise AttributeError(name) from exc


DEFAULT_THRESHOLDS = EventThresholds()


@dataclass
class BiomechanicalSample:
    """One trusted physics-substep measurement.

    ``plate_wrench_N_Nm`` has rows ``[left_plate, right_plate, off_plate]``
    and columns ``[Fx,Fy,Fz,Mx,My,Mz]``.  The off-plate row is diagnostic but
    is always retained in ``support_wrench`` and all impulse accounting.
    """

    time_s: float
    com_position_m: Sequence[float]
    com_velocity_mps: Sequence[float]
    plate_wrench_N_Nm: Sequence[Sequence[float]]
    # World-frame whole-system mechanical quantities.  These are populated by
    # ``from_plant`` for every live physics sample; ``None`` remains permitted
    # for small event fixtures that intentionally exercise only event logic.
    com_acceleration_world_mps2: Sequence[float] | None = None
    linear_momentum_world_kg_mps: Sequence[float] | None = None
    centroidal_h_world_kgm2ps: Sequence[float] | None = None
    centroidal_hdot_world_kgm2ps2: Sequence[float] | None = None
    pelvis_position_world_m: Sequence[float] | None = None
    pelvis_velocity_world_mps: Sequence[float] | None = None
    qpos: Sequence[float] | None = None
    qvel: Sequence[float] | None = None
    whole_support_wrench_N_Nm: Sequence[float] | None = None
    cop_world_xy_m: Sequence[Sequence[float]] | None = None
    cop_validity: Sequence[bool] | None = None
    plate_origin_world_m: Sequence[Sequence[float]] | None = None
    plate_origin_world_xy_m: Sequence[Sequence[float]] = ((0.0, 0.0), (0.0, 0.0))
    permitted_support: Sequence[bool] = (True, True)
    support_polygon_margin_m: Sequence[float] = (0.05, 0.05)
    sole_separation_m: float = 0.02
    trunk_tilt_rad: float = 0.0
    angular_speed_radps: float = 0.0
    joint_rate_max_radps: float = 0.0
    centroidal_h_kgm2ps: float = 0.0
    phi: float = 0.0
    prohibited_contact: bool = False
    native_limit_active: bool = False
    mechanics_residual_trans_N: float = 0.0
    mechanics_residual_rot_Nm: float = 0.0
    support_posture_valid: bool = True
    flight_posture_valid: bool = True
    recovery_posture_valid: bool = True
    active_power_signed_W: float = 0.0
    active_power_positive_W: float | None = None
    active_power_negative_W: float | None = None
    passive_power_W: float = 0.0
    damping_power_W: float = 0.0
    limit_power_W: float = 0.0
    # The post-step sample is the only physical state record.  These optional
    # scalar endpoints carry the realized Plant power immediately before the
    # corresponding mj_step so trapezoidal work integration can represent the
    # first interval (0, h] without inventing a t=0 BiomechanicalSample.
    interval_start_active_power_signed_W: float | None = None
    interval_start_active_power_positive_W: float | None = None
    interval_start_active_power_negative_W: float | None = None
    interval_start_passive_power_W: float | None = None
    interval_start_damping_power_W: float | None = None
    interval_start_limit_power_W: float | None = None
    accepted_action: Sequence[float] | None = None
    realized_anatomical_torque_Nm: Sequence[float] | None = None
    anatomical_position_rad: Sequence[float] | None = None
    anatomical_velocity_radps: Sequence[float] | None = None
    controller_phase: str | None = None
    controller_reference_rad: Sequence[float] | None = None
    controller_reference_pelvis_z_m: float | None = None
    controller_support_force_target_N: float | None = None
    actuator_override_flags: Mapping[str, Sequence[bool]] | None = None
    actuator_capacity_lower_Nm: Sequence[float] | None = None
    actuator_capacity_upper_Nm: Sequence[float] | None = None
    agent_fault_reason: str | None = None
    physics_fault_reason: str | None = None
    attempt_id: str | None = None
    control_index: int | None = None
    physics_index: int | None = None
    accepted_action_digest: str | None = None
    post_step_state_digest: str | None = None
    model_revision: str | None = None
    runtime_revision: str | None = None
    extraction_status: str = "UNSPECIFIED"
    _sealed: bool = field(default=False, init=False, repr=False, compare=False)

    def __setattr__(self, name: str, value: Any) -> None:
        if getattr(self, "_sealed", False):
            raise RuntimeError("BiomechanicalSample is sealed")
        object.__setattr__(self, name, value)

    def __post_init__(self) -> None:
        self.time_s = float(self.time_s)
        self.com_position_m = np.asarray(self.com_position_m, dtype=np.float64).reshape(3).copy()
        self.com_velocity_mps = np.asarray(self.com_velocity_mps, dtype=np.float64).reshape(3).copy()
        for name, size in (
            ("com_acceleration_world_mps2", 3),
            ("linear_momentum_world_kg_mps", 3),
            ("centroidal_h_world_kgm2ps", 3),
            ("centroidal_hdot_world_kgm2ps2", 3),
            ("pelvis_position_world_m", 3),
            ("pelvis_velocity_world_mps", 3),
            ("qpos", 25),
            ("qvel", 21),
        ):
            value = getattr(self, name)
            if value is not None:
                value = np.asarray(value, dtype=np.float64).reshape(size).copy()
                if not np.isfinite(value).all():
                    raise ValueError(f"{name} must be finite")
                setattr(self, name, value)
        self.plate_wrench_N_Nm = np.asarray(self.plate_wrench_N_Nm, dtype=np.float64).reshape(3, 6).copy()
        for name in (
            "accepted_action",
            "realized_anatomical_torque_Nm",
            "anatomical_position_rad",
            "anatomical_velocity_radps",
            "controller_reference_rad",
            "actuator_capacity_lower_Nm",
            "actuator_capacity_upper_Nm",
        ):
            value = getattr(self, name)
            if value is not None:
                value = np.asarray(value, dtype=np.float64).reshape(15).copy()
                if not np.isfinite(value).all():
                    raise ValueError(f"{name} must be finite")
                setattr(self, name, value)
        if self.controller_phase is not None:
            self.controller_phase = str(self.controller_phase)
        if self.controller_reference_pelvis_z_m is not None:
            self.controller_reference_pelvis_z_m = float(self.controller_reference_pelvis_z_m)
            if not np.isfinite(self.controller_reference_pelvis_z_m):
                raise ValueError("controller_reference_pelvis_z_m must be finite")
        if self.controller_support_force_target_N is not None:
            self.controller_support_force_target_N = float(self.controller_support_force_target_N)
            if not np.isfinite(self.controller_support_force_target_N):
                raise ValueError("controller_support_force_target_N must be finite")
        if self.actuator_override_flags is not None:
            self.actuator_override_flags = {
                str(key): np.asarray(value, dtype=bool).reshape(15).copy()
                for key, value in self.actuator_override_flags.items()
            }
        if self.whole_support_wrench_N_Nm is not None:
            self.whole_support_wrench_N_Nm = np.asarray(
                self.whole_support_wrench_N_Nm, dtype=np.float64
            ).reshape(6).copy()
        if self.cop_world_xy_m is not None:
            self.cop_world_xy_m = np.asarray(
                self.cop_world_xy_m, dtype=np.float64
            ).reshape(2, 2).copy()
        if self.cop_validity is not None:
            self.cop_validity = np.asarray(self.cop_validity, dtype=bool).reshape(2).copy()
        if self.plate_origin_world_m is not None:
            self.plate_origin_world_m = np.asarray(
                self.plate_origin_world_m, dtype=np.float64
            ).reshape(2, 3).copy()
        self.plate_origin_world_xy_m = np.asarray(
            self.plate_origin_world_xy_m, dtype=np.float64
        ).reshape(2, 2).copy()
        self.permitted_support = np.asarray(self.permitted_support, dtype=bool).reshape(2).copy()
        self.support_polygon_margin_m = np.asarray(
            self.support_polygon_margin_m, dtype=np.float64
        ).reshape(2).copy()
        for name in (
            "sole_separation_m",
            "trunk_tilt_rad",
            "angular_speed_radps",
            "joint_rate_max_radps",
            "centroidal_h_kgm2ps",
            "phi",
            "mechanics_residual_trans_N",
            "mechanics_residual_rot_Nm",
            "active_power_signed_W",
            "passive_power_W",
            "damping_power_W",
            "limit_power_W",
        ):
            setattr(self, name, float(getattr(self, name)))
        active_power = float(self.active_power_signed_W)
        if self.active_power_positive_W is None:
            self.active_power_positive_W = max(active_power, 0.0)
        else:
            self.active_power_positive_W = float(self.active_power_positive_W)
        if self.active_power_negative_W is None:
            self.active_power_negative_W = min(active_power, 0.0)
        else:
            self.active_power_negative_W = float(self.active_power_negative_W)
        for name in (
            "interval_start_active_power_signed_W",
            "interval_start_active_power_positive_W",
            "interval_start_active_power_negative_W",
            "interval_start_passive_power_W",
            "interval_start_damping_power_W",
            "interval_start_limit_power_W",
        ):
            value = getattr(self, name)
            if value is not None:
                setattr(self, name, float(value))
        if self.attempt_id is not None:
            self.attempt_id = str(self.attempt_id)
        if self.control_index is not None:
            if isinstance(self.control_index, bool) or int(self.control_index) < 0:
                raise ValueError("control_index must be a non-negative integer")
            self.control_index = int(self.control_index)
        if self.physics_index is not None:
            if isinstance(self.physics_index, bool) or int(self.physics_index) < 0:
                raise ValueError("physics_index must be a non-negative integer")
            self.physics_index = int(self.physics_index)
        for name in (
            "accepted_action_digest",
            "post_step_state_digest",
            "model_revision",
            "runtime_revision",
        ):
            value = getattr(self, name)
            if value is not None:
                setattr(self, name, str(value))
        self.extraction_status = str(self.extraction_status)

    def seal(self) -> "BiomechanicalSample":
        """Make this causal sample immutable for the sealed rollout trace."""
        for name in (
            "com_position_m",
            "com_velocity_mps",
            "com_acceleration_world_mps2",
            "linear_momentum_world_kg_mps",
            "centroidal_h_world_kgm2ps",
            "centroidal_hdot_world_kgm2ps2",
            "pelvis_position_world_m",
            "pelvis_velocity_world_mps",
            "qpos",
            "qvel",
            "plate_wrench_N_Nm",
            "whole_support_wrench_N_Nm",
            "cop_world_xy_m",
            "cop_validity",
            "plate_origin_world_m",
            "plate_origin_world_xy_m",
            "permitted_support",
            "support_polygon_margin_m",
            "accepted_action",
            "realized_anatomical_torque_Nm",
            "anatomical_position_rad",
            "anatomical_velocity_radps",
            "controller_reference_rad",
            "actuator_capacity_lower_Nm",
            "actuator_capacity_upper_Nm",
        ):
            value = getattr(self, name)
            if isinstance(value, np.ndarray):
                value.setflags(write=False)
        if self.actuator_override_flags is not None:
            for value in self.actuator_override_flags.values():
                value.setflags(write=False)
        object.__setattr__(self, "_sealed", True)
        return self

    @classmethod
    def from_plant(
        cls,
        plant: Any,
        data: Any,
        *,
        time_s: float,
        prohibited_contact: bool = False,
        support_polygon_margin_m: Sequence[float] = (0.05, 0.05),
        realized_anatomical_torque: Sequence[float] | None = None,
        active_power_signed_W: float | None = None,
        active_power_positive_W: float | None = None,
        active_power_negative_W: float | None = None,
        passive_power_W: float | None = None,
        damping_power_W: float | None = None,
        limit_power_W: float | None = None,
        interval_start_power_components: Mapping[str, float] | None = None,
        accepted_action: Sequence[float] | None = None,
        controller_observability: Mapping[str, Any] | None = None,
        actuator_override_flags: Mapping[str, Sequence[bool]] | None = None,
        actuator_capacity_lower_Nm: Sequence[float] | None = None,
        actuator_capacity_upper_Nm: Sequence[float] | None = None,
        agent_fault_reason: str | None = None,
        physics_fault_reason: str | None = None,
        attempt_id: str | None = None,
        control_index: int | None = None,
        physics_index: int | None = None,
        accepted_action_digest: str | None = None,
        post_step_state_digest: str | None = None,
        model_revision: str | None = None,
        runtime_revision: str | None = None,
        extraction_status: str = "LIVE_POST_MJ_STEP",
    ) -> BiomechanicalSample:
        """Adapt the public Plant force-plate/kinematic diagnostics.

        This adapter reads only physical state and the trusted force-plate
        reconstruction.  It intentionally does not read a policy command or
        the legacy reset fixed-hold aggregate.
        """
        import mujoco  # local import keeps the pure fixture path lightweight

        summary = plant.contact_wrench_summary(data)
        plate = np.asarray(summary["plate_wrench"], dtype=np.float64)
        permitted_support = np.asarray(summary.get("active_by_foot", (False, False)), dtype=bool).reshape(2)
        derived_prohibited_contact = bool(summary.get("prohibited_contact", False))
        angular = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            plant.model,
            data,
            int(mujoco.mjtObj.mjOBJ_BODY),
            int(plant.idx.torso_body),
            angular,
            0,
        )
        com = plant.center_of_mass(data)
        com_velocity = plant.center_of_mass_velocity(data)
        support_wrench = np.asarray(summary["whole_wrench"], dtype=np.float64)
        com_acceleration = plant.center_of_mass_acceleration(
            data,
            support_wrench=support_wrench,
        )
        linear_momentum = plant.linear_momentum(data)
        h = plant.centroidal_angular_momentum(data, com)
        hdot = plant.centroidal_hdot_from_external_wrench(
            data,
            com,
            support_wrench=support_wrench,
        )
        residual = plant.dynamics_residual(data)
        limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        native_limit = bool(
            data.nefc
            and np.any(np.asarray(data.efc_type[: data.nefc], dtype=np.int32) == limit_type)
        )
        anatomical = plant.anatomical_coordinates(data)
        powers = plant.realized_power_components(data, realized_anatomical_torque) if realized_anatomical_torque is not None else {}
        if active_power_signed_W is None:
            active_power_signed_W = powers.get("active_power_signed_W", 0.0)
        if active_power_positive_W is None:
            active_power_positive_W = powers.get("active_power_positive_W")
        if active_power_negative_W is None:
            active_power_negative_W = powers.get("active_power_negative_W")
        if passive_power_W is None:
            passive_power_W = powers.get("passive_power_W", 0.0)
        if damping_power_W is None:
            damping_power_W = powers.get("damping_power_W", 0.0)
        if limit_power_W is None:
            limit_power_W = powers.get("limit_power_W", 0.0)
        start = dict(interval_start_power_components or {})
        controller = dict(controller_observability or {})
        return cls(
            time_s=time_s,
            com_position_m=com,
            com_velocity_mps=com_velocity,
            com_acceleration_world_mps2=com_acceleration,
            linear_momentum_world_kg_mps=linear_momentum,
            centroidal_h_world_kgm2ps=h,
            centroidal_hdot_world_kgm2ps2=hdot,
            pelvis_position_world_m=np.asarray(data.qpos[:3], dtype=np.float64),
            pelvis_velocity_world_mps=np.asarray(data.qvel[:3], dtype=np.float64),
            qpos=np.asarray(data.qpos, dtype=np.float64),
            qvel=np.asarray(data.qvel, dtype=np.float64),
            plate_wrench_N_Nm=plate,
            whole_support_wrench_N_Nm=support_wrench,
            cop_world_xy_m=np.asarray(summary["cop_world_xy"], dtype=np.float64),
            cop_validity=np.asarray(summary["cop_valid"], dtype=bool),
            plate_origin_world_m=np.asarray(summary["plate_origin_world_m"], dtype=np.float64),
            plate_origin_world_xy_m=np.asarray(summary["cop_origin_world_xy"], dtype=np.float64),
            permitted_support=permitted_support,
            support_polygon_margin_m=support_polygon_margin_m,
            sole_separation_m=float(np.min(plant.pad_gaps(data))),
            trunk_tilt_rad=float(plant.trunk_tilt_rad(data)),
            angular_speed_radps=float(np.linalg.norm(angular[:3])),
            joint_rate_max_radps=float(np.max(np.abs(plant.anatomical_rates(data)))),
            centroidal_h_kgm2ps=float(np.linalg.norm(h)),
            phi=float(plant.region_phi(anatomical)),
            prohibited_contact=bool(prohibited_contact or derived_prohibited_contact),
            native_limit_active=native_limit,
            mechanics_residual_trans_N=float(residual["mechanics_residual_trans_N"]),
            mechanics_residual_rot_Nm=float(residual["mechanics_residual_rot_Nm"]),
            active_power_signed_W=float(active_power_signed_W),
            active_power_positive_W=active_power_positive_W,
            active_power_negative_W=active_power_negative_W,
            passive_power_W=float(passive_power_W),
            damping_power_W=float(damping_power_W),
            limit_power_W=float(limit_power_W),
            interval_start_active_power_signed_W=start.get("active_power_signed_W"),
            interval_start_active_power_positive_W=start.get("active_power_positive_W"),
            interval_start_active_power_negative_W=start.get("active_power_negative_W"),
            interval_start_passive_power_W=start.get("passive_power_W"),
            interval_start_damping_power_W=start.get("damping_power_W"),
            interval_start_limit_power_W=start.get("limit_power_W"),
            accepted_action=accepted_action,
            realized_anatomical_torque_Nm=realized_anatomical_torque,
            anatomical_position_rad=anatomical,
            anatomical_velocity_radps=plant.anatomical_rates(data),
            controller_phase=controller.get("phase"),
            controller_reference_rad=controller.get("reference_joint_position_rad"),
            controller_reference_pelvis_z_m=controller.get("reference_pelvis_z_m"),
            controller_support_force_target_N=controller.get("support_force_target_N"),
            actuator_override_flags=actuator_override_flags,
            actuator_capacity_lower_Nm=actuator_capacity_lower_Nm,
            actuator_capacity_upper_Nm=actuator_capacity_upper_Nm,
            agent_fault_reason=agent_fault_reason,
            physics_fault_reason=physics_fault_reason,
            attempt_id=attempt_id,
            control_index=control_index,
            physics_index=physics_index,
            accepted_action_digest=accepted_action_digest,
            post_step_state_digest=post_step_state_digest,
            model_revision=model_revision,
            runtime_revision=runtime_revision,
            extraction_status=extraction_status,
        )

    @property
    def normal_force_N(self) -> np.ndarray:
        return self.plate_wrench_N_Nm[:, 2].copy()

    @property
    def foot_normal_force_N(self) -> np.ndarray:
        return self.normal_force_N[:2]

    @property
    def support_wrench_N_Nm(self) -> np.ndarray:
        """Complete ground-contact wrench, including off-plate diagnostics."""
        if self.whole_support_wrench_N_Nm is not None:
            return self.whole_support_wrench_N_Nm.copy()
        return np.sum(self.plate_wrench_N_Nm, axis=0)

    @property
    def active_by_foot(self) -> np.ndarray:
        return self.foot_normal_force_N > CONTACT_F_ACTIVE_N

    @property
    def active_off_plate(self) -> bool:
        return bool(self.normal_force_N[2] > CONTACT_F_ACTIVE_N)

    @property
    def active_force_producing_contact(self) -> bool:
        return bool(np.any(self.normal_force_N > CONTACT_F_ACTIVE_N))

    @property
    def cop_xy_m(self) -> np.ndarray:
        if self.cop_world_xy_m is not None:
            return self.cop_world_xy_m.copy()
        result = np.zeros((2, 2), dtype=np.float64)
        for foot in (0, 1):
            fz = float(self.foot_normal_force_N[foot])
            if fz > float(_COP_MIN_N):
                result[foot] = self.plate_origin_world_xy_m[foot] + np.asarray(
                    (
                        -float(self.plate_wrench_N_Nm[foot, 4]) / fz,
                        float(self.plate_wrench_N_Nm[foot, 3]) / fz,
                    ),
                    dtype=np.float64,
                )
        return result

    @property
    def cop_valid(self) -> np.ndarray:
        if self.cop_validity is not None:
            return self.cop_validity.copy()
        cop = self.cop_xy_m
        valid = np.zeros(2, dtype=bool)
        for foot in (0, 1):
            fz = float(self.foot_normal_force_N[foot])
            valid[foot] = fz > float(_COP_MIN_N) and np.isfinite(cop[foot]).all()
        return valid


class InternalEvaluationError(RuntimeError):
    """The trusted evaluator could not reconstruct or qualify physical data."""


_COP_MIN_N = 20.0


@dataclass(frozen=True)
class ContactTransition:
    foot: int
    kind: str
    crossing_time_s: float
    confirmation_index: int


@dataclass
class _FootLatchState:
    latched: bool = False
    on_steps: int = 0
    off_steps: int = 0
    pending_on_time_s: float | None = None
    pending_off_time_s: float | None = None
    last_off_time_s: float | None = None


class ContactLatchTracker:
    """Deterministic bilateral F_on/F_off latch with a real chatter predicate."""

    def __init__(self, dt_s: float = BASELINE_DT_S) -> None:
        self.dt_s = float(dt_s)
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise InternalEvaluationError("contact-latch dt must be finite and positive")
        self.on_required = max(CONTACT_N_ON, math.ceil(CONTACT_T_ON_S / self.dt_s - 1e-12))
        self.off_required = max(CONTACT_N_OFF, math.ceil(CONTACT_T_OFF_S / self.dt_s - 1e-12))
        self._feet = [_FootLatchState(), _FootLatchState()]
        self._previous_force: np.ndarray | None = None
        self.chatter = False
        self.chatter_times_s: list[float] = []
        self._step_index = 0
        self._last_time_s = -math.inf

    @property
    def latched(self) -> np.ndarray:
        return np.asarray([state.latched for state in self._feet], dtype=bool)

    def _interpolated_crossing(
        self,
        previous_force: float | None,
        current_force: float,
        previous_time_s: float | None,
        current_time_s: float,
        threshold: float,
    ) -> float:
        if previous_force is None or previous_time_s is None:
            return current_time_s
        denominator = current_force - previous_force
        if abs(denominator) <= _EPS_FORCE:
            return current_time_s
        fraction = (threshold - previous_force) / denominator
        fraction = max(0.0, min(1.0, fraction))
        return float(previous_time_s + fraction * (current_time_s - previous_time_s))

    def step(self, time_s: float, normal_force_N: Sequence[float]) -> list[ContactTransition]:
        time_s = float(time_s)
        force = np.asarray(normal_force_N, dtype=np.float64).reshape(2)
        if not math.isfinite(time_s) or not np.isfinite(force).all():
            raise InternalEvaluationError("contact latch received non-finite state")
        if self._previous_force is not None and time_s <= self._last_time_s:
            raise InternalEvaluationError("contact latch time is not strictly increasing")
        previous_time_s = None if self._previous_force is None else self._last_time_s
        transitions: list[ContactTransition] = []
        for foot, state in enumerate(self._feet):
            previous = None if self._previous_force is None else float(self._previous_force[foot])
            current = float(force[foot])
            if not state.latched:
                state.off_steps = 0
                if current >= CONTACT_F_ON_N:
                    if state.pending_on_time_s is None:
                        state.pending_on_time_s = self._interpolated_crossing(
                            previous, current, previous_time_s, time_s, CONTACT_F_ON_N
                        )
                    state.on_steps += 1
                    if state.on_steps >= self.on_required:
                        state.latched = True
                        state.on_steps = self.on_required
                        transitions.append(
                            ContactTransition(foot, "on", state.pending_on_time_s, self._step_index)
                        )
                        state.pending_on_time_s = None
                else:
                    if state.pending_on_time_s is not None:
                        self._mark_chatter(time_s)
                    state.pending_on_time_s = None
                    state.on_steps = 0
            else:
                state.on_steps = 0
                if current <= CONTACT_F_OFF_N:
                    if state.pending_off_time_s is None:
                        state.pending_off_time_s = self._interpolated_crossing(
                            previous, current, previous_time_s, time_s, CONTACT_F_OFF_N
                        )
                    state.off_steps += 1
                    if state.off_steps >= self.off_required:
                        state.latched = False
                        state.off_steps = self.off_required
                        transition_time = float(state.pending_off_time_s)
                        state.last_off_time_s = transition_time
                        transitions.append(
                            ContactTransition(foot, "off", transition_time, self._step_index)
                        )
                        state.pending_off_time_s = None
                else:
                    if state.pending_off_time_s is not None:
                        self._mark_chatter(time_s)
                    state.pending_off_time_s = None
                    state.off_steps = 0
            if (
                state.last_off_time_s is not None
                and current >= CONTACT_F_ON_N
                and time_s - state.last_off_time_s <= max(8.0 * BASELINE_DT_S, 2.0 * self.dt_s)
            ):
                self._mark_chatter(time_s)
        self._previous_force = force.copy()
        self._last_time_s = time_s
        self._step_index += 1
        return transitions

    def _mark_chatter(self, time_s: float) -> None:
        self.chatter = True
        self.chatter_times_s.append(float(time_s))


def detect_contact_chatter(
    times_s: Sequence[float], normal_force_N: Sequence[Sequence[float]], *, dt_s: float | None = None
) -> bool:
    """Return the trusted runtime chatter predicate for a force history."""
    times = np.asarray(times_s, dtype=np.float64).reshape(-1)
    forces = np.asarray(normal_force_N, dtype=np.float64).reshape(-1, 2)
    if len(times) != len(forces) or len(times) == 0:
        raise InternalEvaluationError("chatter history is empty or shape-inconsistent")
    if dt_s is None:
        dt_s = float(np.median(np.diff(times))) if len(times) > 1 else BASELINE_DT_S
    tracker = ContactLatchTracker(float(dt_s))
    for time_s, force in zip(times, forces):
        tracker.step(float(time_s), force)
    return bool(tracker.chatter)


@dataclass(frozen=True)
class _SampleState:
    latch: np.ndarray
    transitions: tuple[ContactTransition, ...]
    chatter: bool


@dataclass
class CMJEventResult:
    """Serializable CMJ evaluation projection with physical metrics only."""

    events: dict[str, float] = field(default_factory=dict)
    event_valid: dict[str, bool] = field(default_factory=dict)
    raw_metrics: dict[str, Any] = field(default_factory=dict)
    flags: dict[str, Any] = field(default_factory=dict)
    termination_class: TerminationClass = TerminationClass.INCOMPLETE_HORIZON
    termination_reason: str = "horizon ended before recovery completion"

    def to_jsonable(self) -> dict[str, Any]:
        return _native(
            {
                "events": self.events,
                "event_valid": self.event_valid,
                "raw_metrics": self.raw_metrics,
                "flags": self.flags,
                "termination_class": self.termination_class.value,
                "termination_reason": self.termination_reason,
            }
        )


def _native(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _native(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_native(item) for item in value]
    return value


def _linear_crossing_time(t0: float, v0: float, t1: float, v1: float, level: float = 0.0) -> float:
    denominator = v1 - v0
    if abs(denominator) <= _EPS_TIME:
        return float(t1)
    fraction = (level - v0) / denominator
    fraction = max(0.0, min(1.0, fraction))
    return float(t0 + fraction * (t1 - t0))


def _valid_interval(samples: Sequence[BiomechanicalSample], start: float, end: float) -> list[BiomechanicalSample]:
    """Return linearly interpolated endpoints plus all strict interior samples."""
    if end < start - _EPS_TIME:
        raise InternalEvaluationError("metric interval is reversed")
    if end <= start + _EPS_TIME:
        return [sample_at(samples, start)]
    result = [sample_at(samples, start)]
    result.extend(sample for sample in samples if start + _EPS_TIME < sample.time_s < end - _EPS_TIME)
    result.append(sample_at(samples, end))
    return result


def sample_at(samples: Sequence[BiomechanicalSample], time_s: float) -> BiomechanicalSample:
    """Linearly interpolate a physical state at an event boundary."""
    if not samples:
        raise InternalEvaluationError("cannot interpolate an empty trace")
    times = [sample.time_s for sample in samples]
    index = bisect_left(times, float(time_s))
    if index == 0:
        if abs(times[0] - time_s) <= _EPS_TIME:
            return samples[0]
        raise InternalEvaluationError("event precedes trace")
    if index == len(samples):
        if abs(times[-1] - time_s) <= _EPS_TIME:
            return samples[-1]
        raise InternalEvaluationError("event exceeds trace")
    before = samples[index - 1]
    after = samples[index]
    if abs(before.time_s - time_s) <= _EPS_TIME:
        return before
    if abs(after.time_s - time_s) <= _EPS_TIME:
        return after
    fraction = (float(time_s) - before.time_s) / (after.time_s - before.time_s)

    def lerp(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.asarray(a) + fraction * (np.asarray(b) - np.asarray(a))

    return BiomechanicalSample(
        time_s=float(time_s),
        com_position_m=lerp(before.com_position_m, after.com_position_m),
        com_velocity_mps=lerp(before.com_velocity_mps, after.com_velocity_mps),
        com_acceleration_world_mps2=(
            lerp(before.com_acceleration_world_mps2, after.com_acceleration_world_mps2)
            if before.com_acceleration_world_mps2 is not None
            and after.com_acceleration_world_mps2 is not None
            else None
        ),
        linear_momentum_world_kg_mps=(
            lerp(before.linear_momentum_world_kg_mps, after.linear_momentum_world_kg_mps)
            if before.linear_momentum_world_kg_mps is not None
            and after.linear_momentum_world_kg_mps is not None
            else None
        ),
        centroidal_h_world_kgm2ps=(
            lerp(before.centroidal_h_world_kgm2ps, after.centroidal_h_world_kgm2ps)
            if before.centroidal_h_world_kgm2ps is not None
            and after.centroidal_h_world_kgm2ps is not None
            else None
        ),
        centroidal_hdot_world_kgm2ps2=(
            lerp(before.centroidal_hdot_world_kgm2ps2, after.centroidal_hdot_world_kgm2ps2)
            if before.centroidal_hdot_world_kgm2ps2 is not None
            and after.centroidal_hdot_world_kgm2ps2 is not None
            else None
        ),
        plate_wrench_N_Nm=lerp(before.plate_wrench_N_Nm, after.plate_wrench_N_Nm),
        whole_support_wrench_N_Nm=(
            lerp(before.whole_support_wrench_N_Nm, after.whole_support_wrench_N_Nm)
            if before.whole_support_wrench_N_Nm is not None
            and after.whole_support_wrench_N_Nm is not None
            else None
        ),
        cop_world_xy_m=(
            lerp(before.cop_xy_m, after.cop_xy_m)
            if before.cop_world_xy_m is not None and after.cop_world_xy_m is not None
            else None
        ),
        cop_validity=before.cop_valid if fraction < 0.5 else after.cop_valid,
        plate_origin_world_m=(
            lerp(before.plate_origin_world_m, after.plate_origin_world_m)
            if before.plate_origin_world_m is not None
            and after.plate_origin_world_m is not None
            else None
        ),
        plate_origin_world_xy_m=before.plate_origin_world_xy_m if fraction < 0.5 else after.plate_origin_world_xy_m,
        permitted_support=before.permitted_support if fraction < 0.5 else after.permitted_support,
        support_polygon_margin_m=lerp(before.support_polygon_margin_m, after.support_polygon_margin_m),
        sole_separation_m=float(before.sole_separation_m + fraction * (after.sole_separation_m - before.sole_separation_m)),
        trunk_tilt_rad=float(before.trunk_tilt_rad + fraction * (after.trunk_tilt_rad - before.trunk_tilt_rad)),
        angular_speed_radps=float(before.angular_speed_radps + fraction * (after.angular_speed_radps - before.angular_speed_radps)),
        joint_rate_max_radps=float(before.joint_rate_max_radps + fraction * (after.joint_rate_max_radps - before.joint_rate_max_radps)),
        centroidal_h_kgm2ps=float(before.centroidal_h_kgm2ps + fraction * (after.centroidal_h_kgm2ps - before.centroidal_h_kgm2ps)),
        phi=float(before.phi + fraction * (after.phi - before.phi)),
        prohibited_contact=before.prohibited_contact or after.prohibited_contact,
        native_limit_active=before.native_limit_active or after.native_limit_active,
        mechanics_residual_trans_N=float(before.mechanics_residual_trans_N + fraction * (after.mechanics_residual_trans_N - before.mechanics_residual_trans_N)),
        mechanics_residual_rot_Nm=float(before.mechanics_residual_rot_Nm + fraction * (after.mechanics_residual_rot_Nm - before.mechanics_residual_rot_Nm)),
        support_posture_valid=before.support_posture_valid and after.support_posture_valid,
        flight_posture_valid=before.flight_posture_valid and after.flight_posture_valid,
        recovery_posture_valid=before.recovery_posture_valid and after.recovery_posture_valid,
        active_power_signed_W=float(before.active_power_signed_W + fraction * (after.active_power_signed_W - before.active_power_signed_W)),
        active_power_positive_W=float(before.active_power_positive_W + fraction * (after.active_power_positive_W - before.active_power_positive_W)),
        active_power_negative_W=float(before.active_power_negative_W + fraction * (after.active_power_negative_W - before.active_power_negative_W)),
        passive_power_W=float(before.passive_power_W + fraction * (after.passive_power_W - before.passive_power_W)),
        damping_power_W=float(before.damping_power_W + fraction * (after.damping_power_W - before.damping_power_W)),
        limit_power_W=float(before.limit_power_W + fraction * (after.limit_power_W - before.limit_power_W)),
        interval_start_active_power_signed_W=(
            float(before.interval_start_active_power_signed_W + fraction * (
                after.interval_start_active_power_signed_W - before.interval_start_active_power_signed_W
            ))
            if before.interval_start_active_power_signed_W is not None
            and after.interval_start_active_power_signed_W is not None
            else None
        ),
        interval_start_active_power_positive_W=(
            float(before.interval_start_active_power_positive_W + fraction * (
                after.interval_start_active_power_positive_W - before.interval_start_active_power_positive_W
            ))
            if before.interval_start_active_power_positive_W is not None
            and after.interval_start_active_power_positive_W is not None
            else None
        ),
        interval_start_active_power_negative_W=(
            float(before.interval_start_active_power_negative_W + fraction * (
                after.interval_start_active_power_negative_W - before.interval_start_active_power_negative_W
            ))
            if before.interval_start_active_power_negative_W is not None
            and after.interval_start_active_power_negative_W is not None
            else None
        ),
        interval_start_passive_power_W=(
            float(before.interval_start_passive_power_W + fraction * (
                after.interval_start_passive_power_W - before.interval_start_passive_power_W
            ))
            if before.interval_start_passive_power_W is not None
            and after.interval_start_passive_power_W is not None
            else None
        ),
        interval_start_damping_power_W=(
            float(before.interval_start_damping_power_W + fraction * (
                after.interval_start_damping_power_W - before.interval_start_damping_power_W
            ))
            if before.interval_start_damping_power_W is not None
            and after.interval_start_damping_power_W is not None
            else None
        ),
        interval_start_limit_power_W=(
            float(before.interval_start_limit_power_W + fraction * (
                after.interval_start_limit_power_W - before.interval_start_limit_power_W
            ))
            if before.interval_start_limit_power_W is not None
            and after.interval_start_limit_power_W is not None
            else None
        ),
        extraction_status="INTERPOLATED_EVENT_BOUNDARY",
        agent_fault_reason=before.agent_fault_reason or after.agent_fault_reason,
        physics_fault_reason=before.physics_fault_reason or after.physics_fault_reason,
    )


def _integrate(samples: Sequence[BiomechanicalSample], start: float, end: float, values: Any) -> np.ndarray:
    interval = _valid_interval(samples, start, end)
    times = np.asarray([sample.time_s for sample in interval], dtype=np.float64)
    array = np.asarray([values(sample) for sample in interval], dtype=np.float64)
    if len(times) == 1:
        return np.zeros(array.shape[1:] if array.ndim > 1 else (), dtype=np.float64)
    return np.trapezoid(array, times, axis=0)


def _max_continuous_duration(times: Sequence[float], predicate: Sequence[bool]) -> tuple[float, int | None, int | None]:
    best = 0.0
    best_start: int | None = None
    best_end: int | None = None
    start: int | None = None
    for index, good in enumerate(predicate):
        if good and start is None:
            start = index
        if (not good or index == len(predicate) - 1) and start is not None:
            end = index if good and index == len(predicate) - 1 else index - 1
            duration = float(times[end] - times[start])
            if duration > best + _EPS_TIME:
                best, best_start, best_end = duration, start, end
            start = None
    return best, best_start, best_end


class CMJEventDetector:
    """Evaluate the frozen CMJ event DAG and raw physical metric registry."""

    def __init__(self, thresholds: EventThresholds = DEFAULT_THRESHOLDS) -> None:
        self.thresholds = thresholds
        self._live_trace: list[BiomechanicalSample] | None = None
        self._live_states: list[_SampleState] = []
        self._live_transitions: list[ContactTransition] = []
        self._live_tracker: ContactLatchTracker | None = None
        self._live_terminal_result: CMJEventResult | None = None
        self._live_sealed = False
        self._live_supported_index: int | None = None
        self._live_supported_run_start: int | None = None
        self._live_onset_candidate_time: float | None = None
        self._live_onset_candidate_index: int | None = None
        self._live_onset_time: float | None = None
        self._live_onset_index: int | None = None
        self._live_countermovement_start: int | None = None
        self._live_countermovement_time: float | None = None
        self._live_countermovement_index: int | None = None
        self._live_had_downward_band = False
        self._live_reversal_candidate_time: float | None = None
        self._live_reversal_candidate_index: int | None = None
        self._live_positive_band_seen = False
        self._live_reversal_time: float | None = None
        self._live_release_times: dict[int, float] = {}
        self._live_propulsion_probed = False
        self._live_propulsion_valid = False
        self._live_propulsion_probe_result: CMJEventResult | None = None
        self._live_takeoff_time: float | None = None
        self._live_landing_candidates: dict[int, ContactTransition] = {}
        self._live_land_first_time: float | None = None
        self._live_land_bilateral_time: float | None = None
        self._live_arrest_time: float | None = None
        self._live_landing_probe_done = False
        self._live_impact_absorption_time: float | None = None
        self._live_recovery_run_start: int | None = None
        self._live_objective_probe_done = False
        self._live_fall_reference_z: float | None = None
        self._live_fall_run_start: int | None = None
        self._live_fall_probe_done = False

    def reset(self) -> None:
        """Start one stateful live CMJ evaluation with an empty causal trace."""
        self._live_trace = []
        self._live_states = []
        self._live_transitions = []
        self._live_tracker = ContactLatchTracker(BASELINE_DT_S)
        self._live_terminal_result = None
        self._live_sealed = False
        self._live_supported_index = None
        self._live_supported_run_start = None
        self._live_onset_candidate_time = None
        self._live_onset_candidate_index = None
        self._live_onset_time = None
        self._live_onset_index = None
        self._live_countermovement_start = None
        self._live_countermovement_time = None
        self._live_countermovement_index = None
        self._live_had_downward_band = False
        self._live_reversal_candidate_time = None
        self._live_reversal_candidate_index = None
        self._live_positive_band_seen = False
        self._live_reversal_time = None
        self._live_release_times = {}
        self._live_propulsion_probed = False
        self._live_propulsion_valid = False
        self._live_propulsion_probe_result = None
        self._live_takeoff_time = None
        self._live_landing_candidates = {}
        self._live_land_first_time = None
        self._live_land_bilateral_time = None
        self._live_arrest_time = None
        self._live_landing_probe_done = False
        self._live_impact_absorption_time = None
        self._live_recovery_run_start = None
        self._live_objective_probe_done = False
        self._live_fall_reference_z = None
        self._live_fall_run_start = None
        self._live_fall_probe_done = False

    @property
    def live_trace(self) -> tuple[BiomechanicalSample, ...]:
        """Return the one ordered live trace without exposing its list owner."""
        if self._live_trace is None:
            raise InternalEvaluationError("live evaluator has not been reset")
        return tuple(self._live_trace)

    def update(self, sample: BiomechanicalSample) -> CMJEventResult | None:
        """Consume exactly one post-step sample.

        Contact latches and phase guards are advanced once per call.  The
        sealed evaluator is probed only at bounded phase milestones and at a
        terminal candidate; it is never evaluated for every growing prefix.
        """
        if self._live_trace is None:
            self.reset()
        if self._live_sealed or self._live_terminal_result is not None:
            raise InternalEvaluationError("live CMJ trace is already terminal or sealed")
        assert self._live_trace is not None
        assert self._live_tracker is not None
        self._validate_sample(sample)
        if self._live_trace and sample.time_s <= self._live_trace[-1].time_s:
            raise InternalEvaluationError("live physical sample times must be strictly increasing")

        index = len(self._live_trace)
        transitions = self._live_tracker.step(sample.time_s, sample.foot_normal_force_N)
        fixed_transitions = tuple(
            ContactTransition(t.foot, t.kind, t.crossing_time_s, index)
            for t in transitions
        )
        self._live_trace.append(sample)
        self._live_transitions.extend(fixed_transitions)
        self._live_states.append(
            _SampleState(
                self._live_tracker.latched.copy(),
                fixed_transitions,
                bool(self._live_tracker.chatter),
            )
        )

        if self._live_fall_reference_z is None:
            self._live_fall_reference_z = float(sample.com_position_m[2])
        fall_candidate = bool(
            sample.com_position_m[2]
            <= self.thresholds.fall_height_ratio * self._live_fall_reference_z
            or sample.trunk_tilt_rad >= self.thresholds.fall_tilt_max_rad
        )
        if fall_candidate:
            if self._live_fall_run_start is None:
                self._live_fall_run_start = index
            if (
                not self._live_fall_probe_done
                and
                sample.time_s - self._live_trace[self._live_fall_run_start].time_s
                >= self.thresholds.fall_dwell_s - _EPS_TIME
            ):
                self._live_fall_probe_done = True
                return self._live_terminal_probe()
        else:
            self._live_fall_run_start = None

        self._live_update_phase_state(index)
        return self._live_terminal_result

    def finalize(self, *, horizon_s: float | None = None) -> CMJEventResult:
        """Seal the trace and return the sole CMJ result for this attempt."""
        if self._live_trace is None or not self._live_trace:
            raise InternalEvaluationError("cannot finalize an empty live trace")
        if self._live_sealed:
            if self._live_terminal_result is None:
                raise InternalEvaluationError("sealed live evaluator has no result")
            return self._live_terminal_result
        if self._live_terminal_result is None:
            try:
                self._live_terminal_result = self._evaluate_validated(
                    self._live_trace,
                    horizon_s=horizon_s,
                )
            except InternalEvaluationError as exc:
                self._live_terminal_result = CMJEventResult(
                    flags={"internal_evaluation_error": True},
                    termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                    termination_reason=f"{type(exc).__name__}: {exc}",
                )
            except Exception as exc:  # noqa: BLE001 - fail closed at the owner boundary
                self._live_terminal_result = CMJEventResult(
                    flags={"internal_evaluation_error": True, "unexpected": True},
                    termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                    termination_reason=f"unexpected evaluator exception: {type(exc).__name__}: {exc}",
                )
        for sample in self._live_trace:
            sample.seal()
        self._live_sealed = True
        return self._live_terminal_result

    def _live_terminal_probe(self) -> CMJEventResult | None:
        """Run one bounded sealed-evaluator probe at a terminal candidate."""
        if self._live_terminal_result is not None:
            return self._live_terminal_result
        assert self._live_trace is not None
        try:
            result = self._evaluate_validated(
                self._live_trace,
                horizon_s=self._live_trace[-1].time_s,
            )
        except InternalEvaluationError as exc:
            result = CMJEventResult(
                flags={"internal_evaluation_error": True},
                termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                termination_reason=f"{type(exc).__name__}: {exc}",
            )
        except Exception as exc:  # noqa: BLE001 - fail closed at the owner boundary
            result = CMJEventResult(
                flags={"internal_evaluation_error": True, "unexpected": True},
                termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                termination_reason=f"unexpected evaluator exception: {type(exc).__name__}: {exc}",
            )
        if result.termination_class is not TerminationClass.INCOMPLETE_HORIZON:
            self._live_terminal_result = result
        return self._live_terminal_result

    def _live_update_phase_state(self, index: int) -> None:
        assert self._live_trace is not None
        sample = self._live_trace[index]
        state = self._live_states[index]

        if self._live_supported_index is None:
            quiet = bool(
                self._supported_predicate(sample, state.latch, quiet=True)
                and np.linalg.norm(sample.com_velocity_mps)
                <= self.thresholds.quiet_com_speed_mps
            )
            if quiet:
                if self._live_supported_run_start is None:
                    self._live_supported_run_start = index
                if (
                    sample.time_s
                    - self._live_trace[self._live_supported_run_start].time_s
                    >= self.thresholds.quiet_dwell_s - _EPS_TIME
                ):
                    self._live_supported_index = index
            else:
                self._live_supported_run_start = None

        if self._live_supported_index is not None and self._live_onset_time is None:
            if self._live_onset_candidate_time is None and index > 0:
                previous = self._live_trace[index - 1]
                if (
                    previous.com_velocity_mps[2] > self.thresholds.onset_vz_on_mps
                    and sample.com_velocity_mps[2] <= self.thresholds.onset_vz_on_mps
                ):
                    self._live_onset_candidate_time = _linear_crossing_time(
                        previous.time_s,
                        previous.com_velocity_mps[2],
                        sample.time_s,
                        sample.com_velocity_mps[2],
                        self.thresholds.onset_vz_on_mps,
                    )
                    self._live_onset_candidate_index = index
            if self._live_onset_candidate_time is not None:
                elapsed = sample.time_s - self._live_onset_candidate_time
                if elapsed <= self.thresholds.onset_dwell_s + _EPS_TIME:
                    sustained = bool(
                        sample.com_velocity_mps[2] <= self.thresholds.onset_vz_off_mps
                        and self._supported_predicate(sample, state.latch, quiet=False)
                        and np.all(
                            sample.foot_normal_force_N
                            >= self.thresholds.onset_force_floor_bw * MASS_WEIGHT_N
                        )
                    )
                    if not sustained:
                        self._live_onset_candidate_time = None
                        self._live_onset_candidate_index = None
                elif elapsed >= self.thresholds.onset_dwell_s - _EPS_TIME:
                    self._live_onset_time = self._live_onset_candidate_time
                    self._live_onset_index = self._live_onset_candidate_index
                    self._live_onset_candidate_time = None
                    self._live_onset_candidate_index = None

        if self._live_onset_index is not None and self._live_countermovement_time is None:
            reference_z = float(self._live_trace[self._live_onset_index].com_position_m[2])
            depth = reference_z - float(sample.com_position_m[2])
            valid = bool(
                depth >= self.thresholds.countermovement_depth_min_m - _EPS_TIME
                and abs(float(sample.com_velocity_mps[2]))
                <= self.thresholds.countermovement_max_descent_speed_mps + _EPS_TIME
                and np.all(
                    sample.foot_normal_force_N
                    >= self.thresholds.countermovement_force_floor_bw * MASS_WEIGHT_N
                )
                and self._supported_predicate(sample, state.latch, quiet=False)
            )
            if valid:
                if self._live_countermovement_start is None:
                    self._live_countermovement_start = index
                if (
                    sample.time_s
                    - self._live_trace[self._live_countermovement_start].time_s
                    >= self.thresholds.countermovement_dwell_s - _EPS_TIME
                ):
                    self._live_countermovement_time = sample.time_s
                    self._live_countermovement_index = index
            else:
                self._live_countermovement_start = None

        if self._live_countermovement_index is not None and self._live_reversal_time is None:
            if index > self._live_countermovement_index:
                previous = self._live_trace[index - 1]
                self._live_had_downward_band = self._live_had_downward_band or bool(
                    previous.com_velocity_mps[2] <= self.thresholds.reversal_down_band_mps
                )
                if (
                    self._live_reversal_candidate_time is None
                    and self._live_had_downward_band
                    and sample.com_velocity_mps[2] >= 0.0
                    and self._supported_predicate(sample, state.latch, quiet=False)
                ):
                    crossing = _linear_crossing_time(
                        previous.time_s,
                        previous.com_velocity_mps[2],
                        sample.time_s,
                        sample.com_velocity_mps[2],
                    )
                    if (
                        crossing - self._live_trace[self._live_onset_index].time_s
                        >= self.thresholds.minimum_descent_s - _EPS_TIME
                    ):
                        self._live_reversal_candidate_time = crossing
                        self._live_reversal_candidate_index = index
                        self._live_positive_band_seen = False
                if self._live_reversal_candidate_time is not None:
                    if sample.com_velocity_mps[2] >= self.thresholds.zero_band_mps:
                        positive = True
                    else:
                        positive = False
                    self._live_positive_band_seen = self._live_positive_band_seen or positive
                    if (
                        self._live_positive_band_seen
                        and sample.time_s - self._live_reversal_candidate_time
                        >= self.thresholds.reversal_dwell_s - _EPS_TIME
                    ):
                        self._live_reversal_time = self._live_reversal_candidate_time
                        self._live_reversal_candidate_time = None
                        self._live_reversal_candidate_index = None

        for transition in self._live_states[index].transitions:
            if (
                transition.kind == "off"
                and self._live_reversal_time is not None
                and transition.crossing_time_s
                >= self._live_reversal_time - _EPS_TIME
            ):
                self._live_release_times.setdefault(transition.foot, transition.crossing_time_s)

        if (
            self._live_reversal_time is not None
            and len(self._live_release_times) == 2
            and not self._live_propulsion_probed
        ):
            self._live_propulsion_probed = True
            release_times = self._live_release_times
            takeoff = max(release_times.values())
            if (
                abs(release_times[0] - release_times[1])
                <= self.thresholds.release_skew_max_s + _EPS_TIME
                and takeoff - self._live_reversal_time
                >= self.thresholds.takeoff_dwell_s - _EPS_TIME
            ):
                probe = CMJEventResult(event_valid={name: False for name in EVENT_NAMES})
                self._live_propulsion_probe_result = probe
                self._live_propulsion_valid = self._compute_propulsion_metrics(
                    self._live_trace,
                    self._live_reversal_time,
                    takeoff,
                    self._live_supported_index,
                    probe,
                )
                if self._live_propulsion_valid:
                    self._live_takeoff_time = takeoff

        if self._live_takeoff_time is not None:
            for transition in self._live_states[index].transitions:
                if (
                    transition.kind != "on"
                    or transition.crossing_time_s <= self._live_takeoff_time + _EPS_TIME
                    or transition.foot in self._live_landing_candidates
                ):
                    continue
                confirmation = self._live_trace[
                    min(transition.confirmation_index, len(self._live_trace) - 1)
                ]
                preimpact = sample_at(
                    self._live_trace, transition.crossing_time_s
                ).com_velocity_mps[2]
                if preimpact >= -self.thresholds.landing_descent_min_mps:
                    continue
                if (
                    not bool(confirmation.permitted_support[transition.foot])
                    or confirmation.prohibited_contact
                    or confirmation.active_off_plate
                ):
                    continue
                self._live_landing_candidates[transition.foot] = transition
            if len(self._live_landing_candidates) == 2 and self._live_land_first_time is None:
                times = [
                    transition.crossing_time_s
                    for transition in self._live_landing_candidates.values()
                ]
                self._live_land_first_time = min(times)
                self._live_land_bilateral_time = max(times)

        if (
            self._live_land_first_time is not None
            and self._live_arrest_time is None
            and index > 0
            and sample.time_s > self._live_land_first_time + _EPS_TIME
        ):
            previous = self._live_trace[index - 1]
            if previous.com_velocity_mps[2] < 0.0 <= sample.com_velocity_mps[2]:
                crossing = _linear_crossing_time(
                    previous.time_s,
                    previous.com_velocity_mps[2],
                    sample.time_s,
                    sample.com_velocity_mps[2],
                )
                if (
                    crossing >= self._live_land_first_time - _EPS_TIME
                    and crossing - self._live_land_first_time
                    <= self.thresholds.absorption_window_max_s + _EPS_TIME
                ):
                    self._live_arrest_time = crossing

        if (
            self._live_arrest_time is not None
            and not self._live_landing_probe_done
            and sample.time_s
            >= max(
                self._live_arrest_time + self.thresholds.absorption_dwell_s,
                float(self._live_land_bilateral_time) + self.thresholds.landing_dwell_s,
            )
            - _EPS_TIME
        ):
            probe = CMJEventResult(event_valid={name: False for name in EVENT_NAMES})
            if self._live_propulsion_probe_result is not None:
                probe.raw_metrics.update(self._live_propulsion_probe_result.raw_metrics)
                probe.flags.update(self._live_propulsion_probe_result.flags)
            landing_info = self._compute_flight_and_landing(
                self._live_trace,
                self._live_states,
                self._live_transitions,
                self._live_takeoff_time,
                probe,
            )
            self._live_landing_probe_done = True
            if landing_info is not None:
                self._live_impact_absorption_time = landing_info.get(
                    "impact_absorption_time_s"
                )

        if self._live_impact_absorption_time is not None and not self._live_objective_probe_done:
            good = bool(
                sample.time_s >= self._live_impact_absorption_time - _EPS_TIME
                and self._recovery_predicate(sample, state.latch)
            )
            if good:
                if self._live_recovery_run_start is None:
                    self._live_recovery_run_start = index
                if (
                    sample.time_s
                    - self._live_trace[self._live_recovery_run_start].time_s
                    >= self.thresholds.recovery_dwell_s - _EPS_TIME
                ):
                    self._live_objective_probe_done = True
                    self._live_terminal_probe()
            else:
                self._live_recovery_run_start = None

    def evaluate(
        self,
        samples: Iterable[BiomechanicalSample],
        *,
        horizon_s: float | None = None,
    ) -> CMJEventResult:
        """Evaluate a complete trace, converting evaluator failures to a terminal class."""
        try:
            trace = list(samples)
            return self._evaluate_validated(trace, horizon_s=horizon_s)
        except InternalEvaluationError as exc:
            return CMJEventResult(
                flags={"internal_evaluation_error": True},
                termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                termination_reason=f"{type(exc).__name__}: {exc}",
            )
        except Exception as exc:  # noqa: BLE001 - the evaluator must fail closed on unexpected bugs
            return CMJEventResult(
                flags={"internal_evaluation_error": True, "unexpected": True},
                termination_class=TerminationClass.INTERNAL_EVALUATION_ERROR,
                termination_reason=f"unexpected evaluator exception: {type(exc).__name__}: {exc}",
            )

    def _evaluate_validated(
        self, samples: list[BiomechanicalSample], *, horizon_s: float | None
    ) -> CMJEventResult:
        self._validate_trace(samples)
        if horizon_s is None:
            horizon_s = float(samples[-1].time_s)
        horizon_s = float(horizon_s)
        if not math.isfinite(horizon_s) or horizon_s < samples[-1].time_s - _EPS_TIME:
            raise InternalEvaluationError("horizon is non-finite or precedes the trace")

        dt_s = float(np.median(np.diff([sample.time_s for sample in samples]))) if len(samples) > 1 else BASELINE_DT_S
        tracker = ContactLatchTracker(dt_s)
        states: list[_SampleState] = []
        all_transitions: list[ContactTransition] = []
        for index, sample in enumerate(samples):
            transitions = tracker.step(sample.time_s, sample.foot_normal_force_N)
            # The tracker records the confirmation index as its pre-increment
            # index.  Keeping an explicit trace state prevents later code from
            # accidentally using a scalar zero crossing as a channel vector.
            fixed_transitions = tuple(
                ContactTransition(t.foot, t.kind, t.crossing_time_s, index) for t in transitions
            )
            all_transitions.extend(fixed_transitions)
            states.append(_SampleState(tracker.latched.copy(), fixed_transitions, bool(tracker.chatter)))

        result = CMJEventResult()
        result.event_valid.update({name: False for name in EVENT_NAMES})
        result.flags.update(
            {
                "contact_chatter": bool(tracker.chatter),
                "contact_chatter_times_s": list(tracker.chatter_times_s),
                "reachable_torque_interval_used": False,
                "legacy_reset_fixed_hold_used": False,
                "event_source": "physical_state_and_force_plate",
                "evaluation_rate": "physics_substep",
                "numerical_profile": {
                    "timestep_s": BASELINE_DT_S,
                    "solver": "Newton",
                    "iterations": 100,
                    "line_search_iterations": 50,
                    "tolerance": 1e-10,
                    "contact_solref": [0.004, 1.0],
                    "contact_solimp": [0.99, 0.99, 0.001, 0.5, 2.0],
                },
            }
        )
        self._compute_episode_work_metrics(samples, result)

        first_agent_fault = next(
            (index for index, sample in enumerate(samples) if sample.agent_fault_reason), None
        )
        first_physics_fault = next(
            (index for index, sample in enumerate(samples) if sample.physics_fault_reason), None
        )
        fault_indices = [
            index for index in (first_physics_fault, first_agent_fault) if index is not None
        ]
        analysis_end = min(fault_indices) if fault_indices else len(samples) - 1
        analysis = samples[: analysis_end + 1]
        analysis_states = states[: analysis_end + 1]
        if not analysis:
            raise InternalEvaluationError("fault truncation removed the entire trace")

        fall_index = self._fall_index(analysis)
        result.flags["physical_fall_index"] = fall_index
        result.flags["agent_fault_index"] = first_agent_fault
        result.flags["physics_fault_index"] = first_physics_fault
        if first_agent_fault is not None:
            result.flags["agent_fault_reason"] = samples[first_agent_fault].agent_fault_reason
        if first_physics_fault is not None:
            result.flags["physics_fault_reason"] = samples[first_physics_fault].physics_fault_reason

        supported_index = self._supported_start_index(analysis, analysis_states, result)
        if supported_index is not None:
            result.events[EventName.SUPPORTED_START.value] = analysis[supported_index].time_s
            result.event_valid[EventName.SUPPORTED_START.value] = True
            result.raw_metrics["reset_support"] = self._support_metric(analysis[supported_index])
        else:
            result.event_valid[EventName.SUPPORTED_START.value] = False
            result.flags["collapse_rejection"] = True

        onset_time: float | None = None
        onset_index: int | None = None
        if supported_index is not None:
            onset_time, onset_index = self._countermovement_onset(
                analysis, analysis_states, supported_index
            )
        if onset_time is not None and onset_index is not None:
            result.events[EventName.COUNTERMOVEMENT_ONSET.value] = onset_time
            result.event_valid[EventName.COUNTERMOVEMENT_ONSET.value] = True
        else:
            result.event_valid[EventName.COUNTERMOVEMENT_ONSET.value] = False

        reversal_time: float | None = None
        reversal_index: int | None = None
        countermovement_time: float | None = None
        countermovement_index: int | None = None
        if onset_time is not None and onset_index is not None:
            countermovement_time, countermovement_index = self._valid_countermovement(
                analysis, analysis_states, onset_index
            )
        if countermovement_time is not None and countermovement_index is not None:
            result.events[EventName.VALID_COUNTERMOVEMENT.value] = countermovement_time
            result.event_valid[EventName.VALID_COUNTERMOVEMENT.value] = True
        if countermovement_time is not None and countermovement_index is not None:
            first_release_after_onset = min(
                (
                    transition.crossing_time_s
                    for transition in all_transitions
                    if transition.kind == "off" and transition.crossing_time_s > countermovement_time
                ),
                default=None,
            )
            reversal_time, reversal_index = self._reversal(
                analysis, analysis_states, countermovement_index, end_time_s=first_release_after_onset
            )
        if reversal_time is not None and reversal_index is not None:
            result.events[EventName.UPWARD_REVERSAL.value] = reversal_time
            result.event_valid[EventName.UPWARD_REVERSAL.value] = True

        release_times: dict[int, float] = {}
        if reversal_time is not None:
            for transition in all_transitions:
                if transition.kind == "off" and transition.crossing_time_s >= reversal_time - _EPS_TIME:
                    release_times.setdefault(transition.foot, transition.crossing_time_s)
        takeoff_time: float | None = None
        prop_valid = False
        if reversal_time is not None and len(release_times) == 2:
            result.raw_metrics["release_times_s"] = [release_times[0], release_times[1]]
            release_skew = abs(release_times[0] - release_times[1])
            result.raw_metrics.setdefault("flight", {})["release_skew_s"] = release_skew
            if (
                release_skew <= self.thresholds.release_skew_max_s + _EPS_TIME
                and max(release_times.values()) - reversal_time >= self.thresholds.takeoff_dwell_s - _EPS_TIME
            ):
                takeoff_time = max(release_times.values())
                prop_valid = self._compute_propulsion_metrics(
                    analysis,
                    reversal_time,
                    takeoff_time,
                    supported_index,
                    result,
                )
                if prop_valid:
                    result.events[EventName.POSITIVE_PROPULSION.value] = reversal_time
            else:
                result.flags["release_skew_rejection"] = True
        else:
            result.flags["bilateral_release_missing"] = True

        if takeoff_time is not None and prop_valid:
            result.events[EventName.VALID_TAKEOFF.value] = takeoff_time
            result.event_valid[EventName.VALID_TAKEOFF.value] = True

        landing_info: dict[str, Any] | None = None
        if takeoff_time is not None and prop_valid:
            landing_info = self._compute_flight_and_landing(
                analysis,
                analysis_states,
                all_transitions,
                takeoff_time,
                result,
            )

        landing_absorption_valid = False
        impact_absorption_time: float | None = None
        arrest_time: float | None = None
        if landing_info is not None:
            if landing_info.get("flight_valid"):
                result.events[EventName.CONTACT_FREE_FLIGHT.value] = float(
                    landing_info["flight_event_time_s"]
                )
                result.event_valid[EventName.CONTACT_FREE_FLIGHT.value] = True
                apex_time = landing_info.get("apex_time_s")
                apex_dwell_end = (
                    self._first_substep_at_or_after(
                        analysis,
                        float(apex_time) + self.thresholds.apex_dwell_s,
                        float(landing_info["land_first_time_s"]),
                    )
                    if apex_time is not None
                    else None
                )
                if apex_time is not None and apex_dwell_end is not None:
                    result.events[EventName.APEX.value] = float(landing_info["apex_time_s"])
                    result.event_valid[EventName.APEX.value] = True
                else:
                    result.event_valid[EventName.APEX.value] = False
            else:
                result.event_valid[EventName.CONTACT_FREE_FLIGHT.value] = False
                result.event_valid[EventName.APEX.value] = False

            if landing_info.get("descending_recontact_valid"):
                result.events[EventName.VALID_LANDING.value] = float(
                    landing_info.get("valid_landing_time_s", landing_info["land_bilateral_time_s"])
                )
                result.event_valid[EventName.VALID_LANDING.value] = True

            arrest_time = landing_info.get("arrest_time_s") if landing_info.get("flight_valid") else None
            landing_absorption_valid = bool(
                landing_info.get("flight_valid") and landing_info.get("landing_absorption_valid")
            )
            if landing_absorption_valid and arrest_time is not None:
                impact_absorption_time = landing_info.get("impact_absorption_time_s")
                if impact_absorption_time is not None:
                    result.events[EventName.IMPACT_ABSORPTION.value] = float(impact_absorption_time)
                    result.event_valid[EventName.IMPACT_ABSORPTION.value] = True

        recovery_valid = False
        recovery_start_time = impact_absorption_time if impact_absorption_time is not None else arrest_time
        if landing_absorption_valid and recovery_start_time is not None:
            recovery_valid = self._compute_recovery_metrics(
                analysis,
                analysis_states,
                recovery_start_time,
                result,
            )

        if recovery_valid:
            result.events[EventName.CAPTURED_SUPPORTED_STATE.value] = float(
                result.raw_metrics["recovery"]["capture_time_s"]
            )
            result.event_valid[EventName.CAPTURED_SUPPORTED_STATE.value] = True
            result.events[EventName.BOUNDED_RECOVERY.value] = float(
                result.raw_metrics["recovery"]["recovery_dwell_end_time_s"]
            )
            result.event_valid[EventName.BOUNDED_RECOVERY.value] = True
            result.events[EventName.COMPLETION.value] = result.events[EventName.BOUNDED_RECOVERY.value]
            result.event_valid[EventName.COMPLETION.value] = True
        elif result.raw_metrics.get("recovery", {}).get("capture_candidate", False):
            result.event_valid[EventName.CAPTURED_SUPPORTED_STATE.value] = True

        # Explicit precedence is applied at the first terminal sample.  A
        # fault at the same physical sample outranks objective/fall; objective
        # completion outranks a simultaneous fall; horizon is the only
        # terminal class left when no finite physical terminal sample exists.
        terminal_candidates: list[tuple[float, int, TerminationClass, str]] = []
        if first_physics_fault is not None:
            terminal_candidates.append(
                (
                    samples[first_physics_fault].time_s,
                    0,
                    TerminationClass.PHYSICS_NONFINITE_FAULT,
                    str(samples[first_physics_fault].physics_fault_reason),
                )
            )
        if first_agent_fault is not None:
            terminal_candidates.append(
                (
                    samples[first_agent_fault].time_s,
                    1,
                    TerminationClass.AGENT_FAULT,
                    str(samples[first_agent_fault].agent_fault_reason),
                )
            )
        if recovery_valid and EventName.COMPLETION.value in result.events:
            terminal_candidates.append(
                (
                    result.events[EventName.COMPLETION.value],
                    2,
                    TerminationClass.OBJECTIVE_COMPLETE,
                    "continuous recovery dwell completed",
                )
            )
        if fall_index is not None:
            terminal_candidates.append(
                (
                    samples[fall_index].time_s,
                    3,
                    TerminationClass.PHYSICAL_FALL,
                    "fall predicate held continuously for the qualified dwell",
                )
            )
        if terminal_candidates:
            _, _, result.termination_class, result.termination_reason = min(
                terminal_candidates, key=lambda item: (item[0], item[1])
            )
        else:
            result.termination_class = TerminationClass.INCOMPLETE_HORIZON
            result.termination_reason = "horizon ended before recovery completion"

        self._fill_phase_metrics(analysis, onset_time, reversal_time, takeoff_time, landing_info, arrest_time, result)
        self._fill_raw_physical_event_metrics(analysis, analysis_states, result)
        result.flags["event_ordering"] = self._event_ordering_ok(result.events)
        result.flags["no_phase_overlap"] = self._no_phase_overlap(result.events)
        if not result.flags["event_ordering"]:
            raise InternalEvaluationError("event timestamps are not causally ordered")
        return result

    @staticmethod
    def _compute_episode_work_metrics(
        samples: Sequence[BiomechanicalSample], result: CMJEventResult
    ) -> None:
        """Integrate live W-valued fields once into episode-level J fields.

        A live sample is post-``mj_step`` and the scored trace intentionally
        has no sample at ``t=0``.  The Plant therefore supplies the realized
        pre-step power as scalar interval metadata on each sample.  When that
        metadata is present, each physics interval is integrated directly as
        ``h * (P_before + P_after) / 2``; this includes ``(0, h]`` without a
        fabricated physical sample.  Legacy sealed fixtures beginning at
        ``t=0`` retain their existing endpoint-trapezoid convention.
        """
        start_time_s = float(samples[0].time_s)
        end_time_s = float(samples[-1].time_s)
        interval_fields = (
            "interval_start_active_power_signed_W",
            "interval_start_active_power_positive_W",
            "interval_start_active_power_negative_W",
            "interval_start_passive_power_W",
            "interval_start_damping_power_W",
            "interval_start_limit_power_W",
        )
        has_interval_sources = start_time_s > _EPS_TIME and all(
            all(getattr(sample, name) is not None for name in interval_fields)
            for sample in samples
        )

        if has_interval_sources:
            def interval_sum(start_name: str, end_name: str) -> float:
                total = 0.0
                previous_time = 0.0
                for sample in samples:
                    width = float(sample.time_s - previous_time)
                    if width <= 0.0 or not math.isfinite(width):
                        raise InternalEvaluationError("power interval timestamps are not increasing")
                    total += 0.5 * float(
                        getattr(sample, start_name) + getattr(sample, end_name)
                    ) * width
                    previous_time = sample.time_s
                return float(total)

            signed = interval_sum(
                "interval_start_active_power_signed_W", "active_power_signed_W"
            )
            positive = interval_sum(
                "interval_start_active_power_positive_W", "active_power_positive_W"
            )
            negative = interval_sum(
                "interval_start_active_power_negative_W", "active_power_negative_W"
            )
            passive = interval_sum("interval_start_passive_power_W", "passive_power_W")
            damping = interval_sum("interval_start_damping_power_W", "damping_power_W")
            limit = interval_sum("interval_start_limit_power_W", "limit_power_W")
            integration_start_time_s = 0.0
            integration_method = "trapezoidal_pre_step_to_post_step_power"
        else:
            signed = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.active_power_signed_W, dtype=np.float64),
                )
            )
            positive = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.active_power_positive_W, dtype=np.float64),
                )
            )
            negative = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.active_power_negative_W, dtype=np.float64),
                )
            )
            passive = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.passive_power_W, dtype=np.float64),
                )
            )
            damping = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.damping_power_W, dtype=np.float64),
                )
            )
            limit = float(
                _integrate(
                    samples,
                    start_time_s,
                    end_time_s,
                    lambda sample: np.asarray(sample.limit_power_W, dtype=np.float64),
                )
            )
            integration_start_time_s = start_time_s
            integration_method = "trapezoidal_endpoint_interpolation"
        result.raw_metrics["work"] = {
            "initial_cumulative_work_J": 0.0,
            "active_work_cumulative_J": signed,
            "active_work_positive_cumulative_J": positive,
            "active_work_negative_cumulative_J": negative,
            "passive_work_cumulative_J": passive,
            "damping_work_cumulative_J": damping,
            "limit_work_cumulative_signed_J": limit,
            "limit_work_cumulative_abs_J": abs(limit),
            "integration_start_time_s": integration_start_time_s,
            "integration_end_time_s": end_time_s,
            "integration_timestep_s": BASELINE_DT_S,
            "integration_method": integration_method,
            "contact_work": "excluded",
        }

    @staticmethod
    def _fill_raw_physical_event_metrics(
        samples: Sequence[BiomechanicalSample],
        states: Sequence[_SampleState],
        result: CMJEventResult,
    ) -> None:
        """Project raw physical quantities at every qualified event boundary."""
        times = [sample.time_s for sample in samples]
        force_plate: dict[str, Any] = {}
        kinematics: dict[str, Any] = {}
        contact_state: dict[str, Any] = {}
        for event_name, event_time in result.events.items():
            sample = sample_at(samples, event_time)
            state_index = bisect_left(times, float(event_time))
            if state_index >= len(states):
                state_index = len(states) - 1
            elif state_index > 0 and abs(times[state_index - 1] - event_time) <= _EPS_TIME:
                state_index -= 1
            latch = states[state_index].latch
            force_plate[event_name] = {
                "time_s": float(event_time),
                "bilateral_wrench_N_Nm": sample.plate_wrench_N_Nm[:2].copy(),
                "off_plate_wrench_N_Nm": sample.plate_wrench_N_Nm[2].copy(),
                "support_wrench_N_Nm": sample.support_wrench_N_Nm.copy(),
                "normal_force_N": sample.normal_force_N.copy(),
                "cop_xy_m": sample.cop_xy_m.copy(),
                "cop_valid": sample.cop_valid.copy(),
            }
            kinematics[event_name] = {
                "time_s": float(event_time),
                "com_position_m": sample.com_position_m.copy(),
                "com_velocity_mps": sample.com_velocity_mps.copy(),
                "linear_momentum_kgmps": (MASS_KG * sample.com_velocity_mps).copy(),
            }
            contact_state[event_name] = {
                "time_s": float(event_time),
                "foot_normal_force_N": sample.foot_normal_force_N.copy(),
                "active_by_foot": sample.active_by_foot.copy(),
                "latched_by_foot": latch.copy(),
                "permitted_support": sample.permitted_support.copy(),
                "active_off_plate": sample.active_off_plate,
                "active_force_producing_contact": sample.active_force_producing_contact,
                "prohibited_contact": sample.prohibited_contact,
            }
        result.raw_metrics.setdefault("force_plate", {})["event_samples"] = force_plate
        result.raw_metrics["kinematics"] = {"event_samples": kinematics}
        result.raw_metrics["contact_state"] = {"event_samples": contact_state}

    def _validate_trace(self, samples: Sequence[BiomechanicalSample]) -> None:
        if not samples:
            raise InternalEvaluationError("physical trace is empty")
        previous_time = -math.inf
        for sample in samples:
            if not math.isfinite(sample.time_s) or sample.time_s <= previous_time:
                raise InternalEvaluationError("physical sample times must be finite and strictly increasing")
            previous_time = sample.time_s
            self._validate_sample(sample)

    @staticmethod
    def _validate_sample(sample: BiomechanicalSample) -> None:
        for name, value in (
            ("com_position_m", sample.com_position_m),
            ("com_velocity_mps", sample.com_velocity_mps),
            ("plate_wrench_N_Nm", sample.plate_wrench_N_Nm),
            ("support_polygon_margin_m", sample.support_polygon_margin_m),
            ("plate_origin_world_xy_m", sample.plate_origin_world_xy_m),
            ("cop_xy_m", sample.cop_xy_m),
        ):
            if not np.isfinite(value).all():
                raise InternalEvaluationError(f"{name} contains non-finite values")
        for name in (
            "com_acceleration_world_mps2",
            "linear_momentum_world_kg_mps",
            "centroidal_h_world_kgm2ps",
            "centroidal_hdot_world_kgm2ps2",
            "plate_origin_world_m",
        ):
            value = getattr(sample, name)
            if value is not None and not np.isfinite(value).all():
                raise InternalEvaluationError(f"{name} contains non-finite values")
        if sample.whole_support_wrench_N_Nm is not None and not np.isfinite(
            sample.whole_support_wrench_N_Nm
        ).all():
            raise InternalEvaluationError("whole_support_wrench_N_Nm contains non-finite values")
        if np.any(sample.normal_force_N < -_EPS_FORCE):
            raise InternalEvaluationError("negative reconstructed normal force violates nonadhesive contact")
        scalar_values = (
            sample.sole_separation_m,
            sample.trunk_tilt_rad,
            sample.angular_speed_radps,
            sample.joint_rate_max_radps,
            sample.centroidal_h_kgm2ps,
            sample.phi,
            sample.mechanics_residual_trans_N,
            sample.mechanics_residual_rot_Nm,
            sample.active_power_signed_W,
            sample.active_power_positive_W,
            sample.active_power_negative_W,
            sample.passive_power_W,
            sample.damping_power_W,
            sample.limit_power_W,
            sample.interval_start_active_power_signed_W,
            sample.interval_start_active_power_positive_W,
            sample.interval_start_active_power_negative_W,
            sample.interval_start_passive_power_W,
            sample.interval_start_damping_power_W,
            sample.interval_start_limit_power_W,
        )
        if not all(value is None or math.isfinite(float(value)) for value in scalar_values):
            raise InternalEvaluationError("scalar physical metric is non-finite")
        if sample.agent_fault_reason is not None and not isinstance(sample.agent_fault_reason, str):
            raise InternalEvaluationError("agent fault reason must be a string")
        if sample.physics_fault_reason is not None and not isinstance(sample.physics_fault_reason, str):
            raise InternalEvaluationError("physics fault reason must be a string")

    def _supported_predicate(self, sample: BiomechanicalSample, latch: np.ndarray, *, quiet: bool) -> bool:
        loads_ok = sample.foot_normal_force_N >= (
            self.thresholds.reset_foot_load_fraction * MASS_WEIGHT_N
            if quiet
            else self.thresholds.recovery_foot_load_fraction * MASS_WEIGHT_N
        )
        margin_min = (
            self.thresholds.reset_com_margin_m
            if quiet
            else self.thresholds.recovery_cop_margin_min_m
        )
        tilt_max = (
            self.thresholds.reset_trunk_tilt_max_rad
            if quiet
            else self.thresholds.recovery_tilt_max_rad
        )
        return bool(
            np.all(latch)
            and np.all(sample.active_by_foot)
            and np.all(sample.permitted_support)
            and not sample.prohibited_contact
            and not sample.active_off_plate
            and sample.phi < 1.0
            and sample.support_posture_valid
            and np.all(loads_ok)
            and np.all(sample.support_polygon_margin_m >= margin_min)
            and sample.trunk_tilt_rad <= tilt_max
        )

    def _supported_start_index(
        self, samples: Sequence[BiomechanicalSample], states: Sequence[_SampleState], result: CMJEventResult
    ) -> int | None:
        run_start: int | None = None
        for index, (sample, state) in enumerate(zip(samples, states)):
            valid = self._supported_predicate(sample, state.latch, quiet=True) and (
                np.linalg.norm(sample.com_velocity_mps) <= self.thresholds.quiet_com_speed_mps
            )
            if valid:
                if run_start is None:
                    run_start = index
                if sample.time_s - samples[run_start].time_s >= self.thresholds.quiet_dwell_s - _EPS_TIME:
                    result.flags["supported_start_quiet_run_s"] = sample.time_s - samples[run_start].time_s
                    return index
            else:
                run_start = None
        result.flags["supported_start_quiet_run_s"] = 0.0
        return None

    def _countermovement_onset(
        self, samples: Sequence[BiomechanicalSample], states: Sequence[_SampleState], start_index: int
    ) -> tuple[float | None, int | None]:
        onset_level = self.thresholds.onset_vz_on_mps
        release_level = self.thresholds.onset_vz_off_mps
        dwell = self.thresholds.onset_dwell_s
        for index in range(max(start_index + 1, 1), len(samples)):
            previous = samples[index - 1]
            current = samples[index]
            if previous.com_velocity_mps[2] > onset_level and current.com_velocity_mps[2] <= onset_level:
                crossing = _linear_crossing_time(
                    previous.time_s,
                    previous.com_velocity_mps[2],
                    current.time_s,
                    current.com_velocity_mps[2],
                    onset_level,
                )
                sustained = True
                for later_index, later in enumerate(samples[index:], start=index):
                    if later.time_s - crossing > dwell + _EPS_TIME:
                        break
                    if later.com_velocity_mps[2] > release_level or not self._supported_predicate(
                        later, states[later_index].latch, quiet=False
                    ) or np.any(
                        later.foot_normal_force_N
                        < self.thresholds.onset_force_floor_bw * MASS_WEIGHT_N
                    ):
                        sustained = False
                        break
                if sustained:
                    return crossing, index
        return None, None

    def _valid_countermovement(
        self,
        samples: Sequence[BiomechanicalSample],
        states: Sequence[_SampleState],
        onset_index: int,
    ) -> tuple[float | None, int | None]:
        """Confirm the E3 depth, load, speed, and dwell predicate."""
        reference_z = float(samples[onset_index].com_position_m[2])
        candidate_start: int | None = None
        for index in range(onset_index, len(samples)):
            sample = samples[index]
            depth = reference_z - float(sample.com_position_m[2])
            valid = bool(
                depth >= self.thresholds.countermovement_depth_min_m - _EPS_TIME
                and abs(float(sample.com_velocity_mps[2]))
                <= self.thresholds.countermovement_max_descent_speed_mps + _EPS_TIME
                and np.all(
                    sample.foot_normal_force_N
                    >= self.thresholds.countermovement_force_floor_bw * MASS_WEIGHT_N
                )
                and self._supported_predicate(sample, states[index].latch, quiet=False)
            )
            if valid:
                if candidate_start is None:
                    candidate_start = index
                if sample.time_s - samples[candidate_start].time_s >= self.thresholds.countermovement_dwell_s - _EPS_TIME:
                    return sample.time_s, index
            else:
                candidate_start = None
        return None, None

    def _reversal(
        self,
        samples: Sequence[BiomechanicalSample],
        states: Sequence[_SampleState],
        onset_index: int,
        *,
        end_time_s: float | None,
    ) -> tuple[float | None, int | None]:
        had_downward_band = False
        for index in range(max(onset_index + 1, 1), len(samples)):
            previous = samples[index - 1]
            current = samples[index]
            if end_time_s is not None and previous.time_s >= end_time_s - _EPS_TIME:
                break
            had_downward_band = had_downward_band or bool(
                previous.com_velocity_mps[2] <= self.thresholds.reversal_down_band_mps
            )
            if (
                had_downward_band
                and current.com_velocity_mps[2] >= 0.0
            ):
                if not self._supported_predicate(current, states[index].latch, quiet=False):
                    continue
                crossing = _linear_crossing_time(
                    previous.time_s, previous.com_velocity_mps[2], current.time_s, current.com_velocity_mps[2]
                )
                positive_band_confirmed = any(
                    later.time_s >= crossing - _EPS_TIME
                    and later.com_velocity_mps[2] >= self.thresholds.zero_band_mps
                    for later in samples[index:]
                )
                if not positive_band_confirmed:
                    continue
                dwell_end = self._first_substep_at_or_after(
                    samples,
                    crossing + self.thresholds.reversal_dwell_s,
                    end_time_s if end_time_s is not None else samples[-1].time_s,
                )
                if (
                    dwell_end is not None
                    and crossing - samples[onset_index].time_s >= self.thresholds.minimum_descent_s - _EPS_TIME
                ):
                    return crossing, index
        return None, None

    def _fall_index(self, samples: Sequence[BiomechanicalSample]) -> int | None:
        if not samples:
            return None
        reference_z = float(samples[0].com_position_m[2])
        candidate = [
            bool(
                sample.com_position_m[2] <= self.thresholds.fall_height_ratio * reference_z
                or sample.trunk_tilt_rad >= self.thresholds.fall_tilt_max_rad
            )
            for sample in samples
        ]
        duration, start, _ = _max_continuous_duration([s.time_s for s in samples], candidate)
        if start is not None and duration >= self.thresholds.fall_dwell_s - _EPS_TIME:
            return start
        return None

    def _compute_propulsion_metrics(
        self,
        samples: Sequence[BiomechanicalSample],
        reversal_time_s: float,
        takeoff_time_s: float,
        supported_index: int | None,
        result: CMJEventResult,
    ) -> bool:
        """Apply event validity to the canonical force-time metric owner."""
        if takeoff_time_s <= reversal_time_s + _EPS_TIME:
            result.flags["nonpositive_propulsion_interval"] = True
            return False

        propulsion = derive_propulsive_metrics(
            samples,
            reversal_time_s=reversal_time_s,
            takeoff_time_s=takeoff_time_s,
            supported_index=supported_index,
            mass_kg=MASS_KG,
            gravity_mps2=GRAVITY_MPS2,
        )
        phase_force = propulsion.pop("phase_force")
        interval = _valid_interval(samples, reversal_time_s, takeoff_time_s)
        impulse = np.asarray(propulsion["net_impulse_Ns"], dtype=np.float64)
        actual_delta_p = (
            np.asarray(propulsion["linear_momentum_takeoff_kgmps"], dtype=np.float64)
            - np.asarray(propulsion["linear_momentum_start_kgmps"], dtype=np.float64)
        )
        residual = float(propulsion["impulse_momentum_residual_Ns"])
        vertical_impulse = float(propulsion["vertical_impulse_Ns"])
        horizontal_ratio = float(propulsion["horizontal_impulse_ratio"])
        asymmetry = float(propulsion["bilateral_asymmetry"])
        v_takeoff_impulse = float(propulsion["takeoff_vertical_velocity_impulse_mps"])
        propulsion["J_prop_Ns"] = vertical_impulse
        propulsion["phase_peak_force_N"] = float(phase_force["total_force_peak_N"])
        propulsion["phase_mean_force_N"] = float(phase_force["total_force_mean_N"])
        result.raw_metrics["propulsion"] = propulsion
        result.raw_metrics["impulses"] = {
            "propulsion_net_impulse_Ns": impulse.copy(),
            "propulsion_momentum_change_kgmps": actual_delta_p.copy(),
        }
        result.raw_metrics.setdefault("force_plate", {})["propulsion_start_support_wrench_N_Nm"] = (
            sample_at(samples, reversal_time_s).support_wrench_N_Nm.copy()
        )
        result.raw_metrics["force_plate"]["propulsion_end_support_wrench_N_Nm"] = (
            sample_at(samples, takeoff_time_s).support_wrench_N_Nm.copy()
        )

        residual_bound = self.thresholds.impulse_residual_relative_max * max(
            1.0, float(np.linalg.norm(impulse))
        )
        force_floor_duration, _, _ = _max_continuous_duration(
            [sample.time_s for sample in interval],
            [
                sample.support_wrench_N_Nm[2]
                >= self.thresholds.propulsion_force_floor_bw * MASS_WEIGHT_N
                for sample in interval
            ],
        )
        valid = bool(
            v_takeoff_impulse >= self.thresholds.takeoff_vz_min_mps
            and takeoff_time_s - reversal_time_s >= self.thresholds.propulsion_dwell_s - _EPS_TIME
            and vertical_impulse > 0.0
            and force_floor_duration >= self.thresholds.propulsion_dwell_s - _EPS_TIME
            and horizontal_ratio <= self.thresholds.horizontal_impulse_ratio_max + _EPS_TIME
            and asymmetry <= self.thresholds.propulsion_asymmetry_max + _EPS_TIME
            and residual <= residual_bound + _EPS_IMPULSE
            and all(not sample.prohibited_contact for sample in interval)
            and all(not sample.active_off_plate for sample in interval)
            and all(sample.phi < 1.0 for sample in interval)
            and all(not sample.native_limit_active for sample in interval)
            and all(
                sample.mechanics_residual_trans_N
                <= self.thresholds.mechanics_residual_trans_max_N
                for sample in interval
            )
            and all(
                sample.mechanics_residual_rot_Nm
                <= self.thresholds.mechanics_residual_rot_max_Nm
                for sample in interval
            )
        )
        if not valid:
            result.flags["propulsion_rejection"] = {
                "takeoff_vertical_velocity": v_takeoff_impulse < self.thresholds.takeoff_vz_min_mps,
                "propulsion_dwell": takeoff_time_s - reversal_time_s < self.thresholds.propulsion_dwell_s,
                "nonpositive_vertical_impulse": vertical_impulse <= 0.0,
                "force_floor": force_floor_duration < self.thresholds.propulsion_dwell_s,
                "horizontal_impulse": horizontal_ratio > self.thresholds.horizontal_impulse_ratio_max,
                "bilateral_asymmetry": asymmetry > self.thresholds.propulsion_asymmetry_max,
                "impulse_residual": residual > residual_bound + _EPS_IMPULSE,
                "prohibited_contact": any(sample.prohibited_contact for sample in interval),
                "off_plate_contact": any(sample.active_off_plate for sample in interval),
                "phi": any(sample.phi >= 1.0 for sample in interval),
                "native_limit": any(sample.native_limit_active for sample in interval),
                "mechanics_residual_trans": any(
                    sample.mechanics_residual_trans_N > self.thresholds.mechanics_residual_trans_max_N
                    for sample in interval
                ),
                "mechanics_residual_rot": any(
                    sample.mechanics_residual_rot_Nm > self.thresholds.mechanics_residual_rot_max_Nm
                    for sample in interval
                ),
            }
        result.event_valid[EventName.POSITIVE_PROPULSION.value] = bool(valid)
        return valid

    def _compute_flight_and_landing(
        self,
        samples: Sequence[BiomechanicalSample],
        states: Sequence[_SampleState],
        transitions: Sequence[ContactTransition],
        takeoff_time_s: float,
        result: CMJEventResult,
    ) -> dict[str, Any] | None:
        landing_candidates: dict[int, ContactTransition] = {}
        invalid_landing = False
        ascending_anomaly = False
        for transition in transitions:
            if transition.kind != "on" or transition.crossing_time_s <= takeoff_time_s + _EPS_TIME:
                continue
            confirmation = samples[min(transition.confirmation_index, len(samples) - 1)]
            preimpact = sample_at(samples, transition.crossing_time_s).com_velocity_mps[2]
            if preimpact >= -self.thresholds.landing_descent_min_mps:
                ascending_anomaly = True
                continue
            if (
                not bool(confirmation.permitted_support[transition.foot])
                or confirmation.prohibited_contact
                or confirmation.active_off_plate
            ):
                invalid_landing = True
                continue
            landing_candidates.setdefault(transition.foot, transition)

        if len(landing_candidates) != 2:
            result.flags["descending_recontact_missing"] = True
            result.flags["ascending_contact_anomaly"] = ascending_anomaly
            result.flags["invalid_landing_contact"] = invalid_landing
            return {
                "flight_valid": False,
                "descending_recontact_valid": False,
                "landing_absorption_valid": False,
            }

        land_times = {foot: trans.crossing_time_s for foot, trans in landing_candidates.items()}
        land_first = min(land_times.values())
        land_bilateral = max(land_times.values())
        landing_skew = land_bilateral - land_first
        interval = _valid_interval(samples, takeoff_time_s, land_first)
        active_contact = any(
            sample.active_force_producing_contact for sample in interval[1:-1]
        )
        chatter_in_flight = bool(
            any(
                state.chatter
                for sample, state in zip(samples, states)
                if takeoff_time_s - _EPS_TIME <= sample.time_s <= land_first + _EPS_TIME
            )
        )
        flight_dwell = land_first - takeoff_time_s
        flight_posture = all(sample.flight_posture_valid and sample.trunk_tilt_rad <= self.thresholds.flight_tilt_max_rad for sample in interval)
        separation = float(min(sample.sole_separation_m for sample in interval))
        takeoff = sample_at(samples, takeoff_time_s)
        v_takeoff = float(result.raw_metrics["propulsion"]["takeoff_vertical_velocity_impulse_mps"])
        z_takeoff = float(takeoff.com_position_m[2])
        pos_residual = 0.0
        vel_residual = 0.0
        for sample in interval:
            elapsed = sample.time_s - takeoff_time_s
            expected_z = z_takeoff + v_takeoff * elapsed - 0.5 * GRAVITY_MPS2 * elapsed * elapsed
            expected_v = v_takeoff - GRAVITY_MPS2 * elapsed
            pos_residual = max(pos_residual, abs(float(sample.com_position_m[2]) - expected_z))
            vel_residual = max(vel_residual, abs(float(sample.com_velocity_mps[2]) - expected_v))
        ballistic_residual = max(
            pos_residual / max(_EPS_TIME, self.thresholds.ballistic_position_residual_max_m),
            vel_residual / max(_EPS_TIME, self.thresholds.ballistic_velocity_residual_max_mps),
        )
        apex_time = self._apex_time(samples, takeoff_time_s, land_first)
        flight_event_time = self._first_substep_at_or_after(
            samples, takeoff_time_s + self.thresholds.flight_dwell_min_s, land_first
        )
        flight_valid = bool(
            flight_event_time is not None
            and flight_dwell >= self.thresholds.flight_dwell_min_s - _EPS_TIME
            and not active_contact
            and not chatter_in_flight
            and not ascending_anomaly
            and not invalid_landing
            and separation >= self.thresholds.sole_separation_min_m - _EPS_TIME
            and ballistic_residual <= 1.0 + _EPS_TIME
            and flight_posture
            and landing_skew <= self.thresholds.landing_skew_max_s + _EPS_TIME
        )
        result.raw_metrics["flight"] = {
            "takeoff_time_s": takeoff_time_s,
            "land_first_time_s": land_first,
            "land_bilateral_time_s": land_bilateral,
            "support_free_duration_s": flight_dwell,
            "release_to_flight_event_duration_s": flight_event_time - takeoff_time_s if flight_event_time is not None else 0.0,
            "ballistic_position_residual_m": pos_residual,
            "ballistic_velocity_residual_mps": vel_residual,
            "ballistic_residual_normalized": ballistic_residual,
            "sole_separation_min_m": separation,
            "contact_exclusion": not active_contact,
            "release_chatter_exclusion": not chatter_in_flight,
            "apex_time_s": apex_time,
            "release_skew_s": float(result.raw_metrics.get("release_times_s", [takeoff_time_s, takeoff_time_s])[1] - result.raw_metrics.get("release_times_s", [takeoff_time_s, takeoff_time_s])[0]),
            "landing_skew_s": landing_skew,
        }
        if not flight_valid:
            result.flags["flight_rejection"] = {
                "dwell": flight_dwell < self.thresholds.flight_dwell_min_s,
                "active_contact": active_contact,
                "chatter": chatter_in_flight,
                "ascending_contact": ascending_anomaly,
                "invalid_landing_contact": invalid_landing,
                "separation": separation < self.thresholds.sole_separation_min_m,
                "ballistic": ballistic_residual > 1.0,
                "attitude": not flight_posture,
                "landing_skew": landing_skew > self.thresholds.landing_skew_max_s,
            }

        preimpact_sample = self._left_sample(samples, land_first)
        preimpact_vz = -float(preimpact_sample.com_velocity_mps[2])
        arrest_time = self._arrest_time(samples, land_first)
        result.raw_metrics.setdefault("landing", {})["preimpact_descent_speed_mps"] = preimpact_vz
        if arrest_time is None:
            result.flags["momentum_arrest_missing"] = True
            return {
                "flight_valid": flight_valid,
                "flight_event_time_s": flight_event_time,
                "apex_time_s": apex_time,
                "descending_recontact_valid": bool(flight_valid and preimpact_vz > self.thresholds.landing_descent_min_mps),
                "land_first_time_s": land_first,
                "land_bilateral_time_s": land_bilateral,
                "arrest_time_s": None,
                "landing_absorption_valid": False,
            }

        landing = self._compute_landing_metrics(
            samples, land_first, land_bilateral, arrest_time, preimpact_sample, result
        )
        valid_landing_end = self._first_substep_at_or_after(
            samples,
            land_bilateral + self.thresholds.landing_dwell_s,
            arrest_time,
        )
        landing_dwell_valid = bool(
            valid_landing_end is not None
            and all(
                bool(np.all(sample.permitted_support))
                and not sample.prohibited_contact
                and not sample.active_off_plate
                for sample in _valid_interval(samples, land_bilateral, valid_landing_end)
            )
        )
        impact_absorption_time = self._first_substep_at_or_after(
            samples,
            arrest_time + self.thresholds.absorption_dwell_s,
            samples[-1].time_s,
        )
        impact_absorption_valid = bool(
            landing.get("valid", False)
            and impact_absorption_time is not None
            and all(
                sample.com_velocity_mps[2] >= self.thresholds.absorption_arrest_mps - _EPS_TIME
                and sample.com_velocity_mps[2] <= self.thresholds.rebound_max_mps + _EPS_TIME
                for sample in _valid_interval(samples, arrest_time, impact_absorption_time)
            )
        )
        landing["valid_landing_time_s"] = valid_landing_end if landing_dwell_valid else None
        landing["impact_absorption_time_s"] = (
            impact_absorption_time if impact_absorption_valid else None
        )
        landing.update(
            {
                "flight_valid": flight_valid,
                "flight_event_time_s": flight_event_time,
                "apex_time_s": apex_time,
                "descending_recontact_valid": bool(
                    flight_valid
                    and preimpact_vz > self.thresholds.landing_descent_min_mps
                    and landing_dwell_valid
                ),
                "land_first_time_s": land_first,
                "land_bilateral_time_s": land_bilateral,
                "arrest_time_s": arrest_time,
                "impact_absorption_valid": impact_absorption_valid,
            }
        )
        return landing

    @staticmethod
    def _left_sample(samples: Sequence[BiomechanicalSample], time_s: float) -> BiomechanicalSample:
        previous = [sample for sample in samples if sample.time_s < time_s - _EPS_TIME]
        if previous:
            return previous[-1]
        return sample_at(samples, time_s)

    @staticmethod
    def _first_substep_at_or_after(
        samples: Sequence[BiomechanicalSample], target_time_s: float, end_time_s: float
    ) -> float | None:
        for sample in samples:
            if sample.time_s + _EPS_TIME >= target_time_s and sample.time_s <= end_time_s + _EPS_TIME:
                return sample.time_s
        return None

    @staticmethod
    def _apex_time(samples: Sequence[BiomechanicalSample], start_time_s: float, end_time_s: float) -> float | None:
        interval = [sample for sample in samples if start_time_s - _EPS_TIME <= sample.time_s <= end_time_s + _EPS_TIME]
        for previous, current in pairwise(interval):
            if previous.time_s < start_time_s - _EPS_TIME:
                continue
            if previous.com_velocity_mps[2] > 0.0 >= current.com_velocity_mps[2]:
                return _linear_crossing_time(
                    previous.time_s, previous.com_velocity_mps[2], current.time_s, current.com_velocity_mps[2]
                )
        return None

    def _arrest_time(self, samples: Sequence[BiomechanicalSample], land_first_time_s: float) -> float | None:
        end_time = min(
            samples[-1].time_s,
            land_first_time_s
            + min(self.thresholds.absorption_window_max_s, self.thresholds.absorption_timeout_s),
        )
        interval = _valid_interval(samples, land_first_time_s, end_time)
        for previous, current in pairwise(interval):
            if previous.com_velocity_mps[2] < 0.0 <= current.com_velocity_mps[2]:
                crossing = _linear_crossing_time(
                    previous.time_s, previous.com_velocity_mps[2], current.time_s, current.com_velocity_mps[2]
                )
                if crossing - land_first_time_s <= self.thresholds.absorption_window_max_s + _EPS_TIME:
                    return crossing
        return None

    def _compute_landing_metrics(
        self,
        samples: Sequence[BiomechanicalSample],
        land_first_time_s: float,
        land_bilateral_time_s: float,
        arrest_time_s: float,
        preimpact_sample: BiomechanicalSample,
        result: CMJEventResult,
    ) -> dict[str, Any]:
        interval = _valid_interval(samples, land_first_time_s, arrest_time_s)
        net_force = lambda sample: np.asarray(
            [
                sample.support_wrench_N_Nm[0],
                sample.support_wrench_N_Nm[1],
                sample.support_wrench_N_Nm[2] - MASS_WEIGHT_N,
            ],
            dtype=np.float64,
        )
        landing_impulse = _integrate(samples, land_first_time_s, arrest_time_s, net_force)
        preimpact_velocity = preimpact_sample.com_velocity_mps.copy()
        arrest_sample = sample_at(samples, arrest_time_s)
        momentum_change = MASS_KG * (arrest_sample.com_velocity_mps - preimpact_velocity)
        closure = float(np.linalg.norm(landing_impulse - momentum_change))
        peak_fz = float(max(sample.support_wrench_N_Nm[2] for sample in interval))
        rebound = float(
            max(
                [
                    max(0.0, float(sample.com_velocity_mps[2]))
                    for sample in _valid_interval(
                        samples,
                        arrest_time_s,
                        min(samples[-1].time_s, arrest_time_s + self.thresholds.absorption_window_max_s),
                    )
                ]
                or [0.0]
            )
        )
        beta = np.asarray([0.5, 0.5], dtype=np.float64)
        common_landing_interval = _valid_interval(samples, land_bilateral_time_s, arrest_time_s)
        bilateral = np.asarray(
            [
                _integrate(
                    samples,
                    land_bilateral_time_s,
                    arrest_time_s,
                    lambda sample, foot=foot: np.asarray(
                        sample.foot_normal_force_N[foot] - beta[foot] * MASS_WEIGHT_N,
                        dtype=np.float64,
                    ),
                )
                for foot in (0, 1)
            ],
            dtype=np.float64,
        ).reshape(2)
        bilateral_asymmetry = float(
            abs(bilateral[0] - bilateral[1]) / max(_EPS_IMPULSE, abs(bilateral[0]) + abs(bilateral[1]))
        )
        active_work_signed = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.active_power_signed_W, dtype=np.float64),
            )
        )
        active_work_positive = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.active_power_positive_W, dtype=np.float64),
            )
        )
        active_work_negative = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.active_power_negative_W, dtype=np.float64),
            )
        )
        passive_work = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.passive_power_W, dtype=np.float64),
            )
        )
        damping_work = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.damping_power_W, dtype=np.float64),
            )
        )
        limit_work = float(
            _integrate(
                samples,
                land_first_time_s,
                arrest_time_s,
                lambda sample: np.asarray(sample.limit_power_W, dtype=np.float64),
            )
        )
        preimpact_speed = max(0.0, -float(preimpact_velocity[2]))
        impulse_bound = self.thresholds.impulse_residual_relative_max * max(
            1.0, float(np.linalg.norm(landing_impulse))
        )
        landing = {
            "window_start_time_s": land_first_time_s,
            "window_end_time_s": arrest_time_s,
            "window_duration_s": arrest_time_s - land_first_time_s,
            "preimpact_velocity_mps": preimpact_velocity.copy(),
            "linear_momentum_preimpact_kgmps": (MASS_KG * preimpact_velocity).copy(),
            "linear_momentum_arrest_kgmps": (MASS_KG * arrest_sample.com_velocity_mps).copy(),
            "momentum_change_kgmps": momentum_change.copy(),
            "preimpact_descent_speed_mps": preimpact_speed,
            "net_impulse_Ns": landing_impulse.copy(),
            "vertical_impulse_Ns": float(landing_impulse[2]),
            "peak_force_N": peak_fz,
            "contact_skew_s": land_bilateral_time_s - land_first_time_s,
            "bilateral_vertical_impulse_Ns": bilateral.copy(),
            "bilateral_impulse_asymmetry": bilateral_asymmetry,
            "rebound_speed_mps": rebound,
            "impulse_momentum_residual_Ns": closure,
            "impulse_momentum_residual_relative": closure / max(_EPS_IMPULSE, float(np.linalg.norm(landing_impulse))),
            "landing_active_work_signed_J": active_work_signed,
            "landing_active_work_positive_J": active_work_positive,
            "landing_active_work_negative_J": active_work_negative,
            "landing_passive_work_J": passive_work,
            "landing_damping_work_J": damping_work,
            "landing_limit_work_signed_J": limit_work,
            "landing_limit_work_abs_J": abs(limit_work),
            "matched_endpoint_sample_count": len(interval),
            "common_bilateral_interval_sample_count": len(common_landing_interval),
            "integration_method": "trapezoidal_endpoint_interpolation",
            "mass_kg": MASS_KG,
            "support_wrench_includes_off_plate": True,
        }
        result.raw_metrics["landing"] = landing
        result.raw_metrics.setdefault("impulses", {}).update(
            {
                "landing_net_impulse_Ns": landing_impulse.copy(),
                "landing_momentum_change_kgmps": momentum_change.copy(),
            }
        )
        result.raw_metrics["momentum_arrest"] = {
            "time_s": float(arrest_time_s),
            "window_start_time_s": float(land_first_time_s),
            "window_duration_s": float(arrest_time_s - land_first_time_s),
            "preimpact_vertical_velocity_mps": float(preimpact_velocity[2]),
            "arrest_vertical_velocity_mps": float(arrest_sample.com_velocity_mps[2]),
            "linear_momentum_preimpact_kgmps": (MASS_KG * preimpact_velocity).copy(),
            "linear_momentum_arrest_kgmps": (MASS_KG * arrest_sample.com_velocity_mps).copy(),
            "momentum_change_kgmps": momentum_change.copy(),
            "landing_impulse_Ns": landing_impulse.copy(),
            "impulse_momentum_residual_Ns": closure,
            "impulse_momentum_residual_relative": closure
            / max(_EPS_IMPULSE, float(np.linalg.norm(landing_impulse))),
        }
        result.raw_metrics.setdefault("force_plate", {})["landing_start_support_wrench_N_Nm"] = sample_at(
            samples, land_first_time_s
        ).support_wrench_N_Nm.copy()
        result.raw_metrics["force_plate"]["landing_arrest_support_wrench_N_Nm"] = arrest_sample.support_wrench_N_Nm.copy()
        valid = bool(
            preimpact_speed > self.thresholds.landing_descent_min_mps
            and landing_impulse[2] > 0.0
            and peak_fz <= self.thresholds.landing_peak_force_multiple * MASS_WEIGHT_N
            and landing["contact_skew_s"] <= self.thresholds.landing_skew_max_s + _EPS_TIME
            and rebound <= self.thresholds.rebound_max_mps + _EPS_TIME
            and closure <= impulse_bound + _EPS_IMPULSE
            and abs(limit_work) <= self.thresholds.limit_work_max_j + _EPS_IMPULSE
            and all(not sample.prohibited_contact for sample in interval)
            and all(not sample.active_off_plate for sample in interval)
            and all(sample.phi < 1.0 for sample in interval)
            and all(not sample.native_limit_active for sample in interval)
        )
        landing["valid"] = valid
        if not valid:
            result.flags["landing_rejection"] = {
                "preimpact_descent": not (preimpact_speed > self.thresholds.landing_descent_min_mps),
                "nonpositive_impulse": not (landing_impulse[2] > 0.0),
                "peak_force": peak_fz > self.thresholds.landing_peak_force_multiple * MASS_WEIGHT_N,
                "contact_skew": landing["contact_skew_s"] > self.thresholds.landing_skew_max_s,
                "rebound": rebound > self.thresholds.rebound_max_mps,
                "impulse_residual": closure > impulse_bound + _EPS_IMPULSE,
                "limit_work": abs(limit_work) > self.thresholds.limit_work_max_j,
                "prohibited_contact": any(sample.prohibited_contact for sample in interval),
                "off_plate_contact": any(sample.active_off_plate for sample in interval),
                "phi": any(sample.phi >= 1.0 for sample in interval),
                "native_limit": any(sample.native_limit_active for sample in interval),
            }
        landing["landing_absorption_valid"] = valid
        return landing

    def _recovery_predicate(self, sample: BiomechanicalSample, latch: np.ndarray) -> bool:
        return bool(
            self._supported_predicate(sample, latch, quiet=False)
            and np.linalg.norm(sample.com_velocity_mps) <= self.thresholds.recovery_com_speed_max_mps
            and np.all(sample.cop_valid)
            and np.all(sample.support_polygon_margin_m >= self.thresholds.recovery_cop_margin_min_m)
            and sample.trunk_tilt_rad <= self.thresholds.recovery_tilt_max_rad
            and sample.angular_speed_radps <= self.thresholds.recovery_angular_speed_max_radps
            and sample.joint_rate_max_radps <= self.thresholds.recovery_joint_rate_max_radps
            and sample.centroidal_h_kgm2ps <= self.thresholds.recovery_centroidal_h_max_kgm2ps
            and sample.sole_separation_m >= 0.0
            and sample.recovery_posture_valid
            and sample.com_velocity_mps[2] <= self.thresholds.recovery_rebound_max_mps
            and not sample.prohibited_contact
            and not sample.active_off_plate
            and sample.phi < 1.0
            and not sample.native_limit_active
        )

    def _compute_recovery_metrics(
        self,
        samples: Sequence[BiomechanicalSample],
        states: Sequence[_SampleState],
        arrest_time_s: float,
        result: CMJEventResult,
    ) -> bool:
        predicate = [
            sample.time_s >= arrest_time_s - _EPS_TIME
            and self._recovery_predicate(sample, state.latch)
            for sample, state in zip(samples, states)
        ]
        times = [sample.time_s for sample in samples]
        duration, start, end = _max_continuous_duration(times, predicate)
        dwell_end = (
            self._first_substep_at_or_after(
                samples,
                times[start] + self.thresholds.recovery_dwell_s,
                times[end],
            )
            if start is not None and end is not None and duration >= self.thresholds.recovery_dwell_s - _EPS_TIME
            else None
        )
        capture_time = (
            self._first_substep_at_or_after(
                samples,
                times[start] + self.thresholds.capture_dwell_s,
                times[end],
            )
            if start is not None and end is not None and duration >= self.thresholds.capture_dwell_s - _EPS_TIME
            else None
        )
        recovery: dict[str, Any] = {
            "recovery_dwell_s": duration,
            "capture_candidate": start is not None,
            "capture_start_time_s": times[start] if start is not None else None,
            "capture_time_s": capture_time,
            "recovery_dwell_end_time_s": dwell_end,
            "recovery_predicate_trace": predicate,
            "load_min_fraction": float(
                min(
                    (min(sample.foot_normal_force_N) / MASS_WEIGHT_N for sample, good in zip(samples, predicate) if good),
                    default=0.0,
                )
            ),
            "cop_margin_min_m": float(
                min(
                    (min(sample.support_polygon_margin_m) for sample, good in zip(samples, predicate) if good),
                    default=0.0,
                )
            ),
            "com_speed_max_mps": float(
                max((np.linalg.norm(sample.com_velocity_mps) for sample, good in zip(samples, predicate) if good), default=0.0)
            ),
            "tilt_max_rad": float(max((sample.trunk_tilt_rad for sample, good in zip(samples, predicate) if good), default=0.0)),
            "angular_speed_max_radps": float(max((sample.angular_speed_radps for sample, good in zip(samples, predicate) if good), default=0.0)),
            "centroidal_h_max_kgm2ps": float(max((sample.centroidal_h_kgm2ps for sample, good in zip(samples, predicate) if good), default=0.0)),
            "joint_rate_max_radps": float(max((sample.joint_rate_max_radps for sample, good in zip(samples, predicate) if good), default=0.0)),
            "no_rebound": all(
                sample.com_velocity_mps[2] <= self.thresholds.recovery_rebound_max_mps
                for sample, good in zip(samples, predicate)
                if good
            ),
        }
        result.raw_metrics["recovery"] = recovery
        valid = bool(dwell_end is not None and capture_time is not None and start is not None and end is not None)
        if not valid:
            result.flags["recovery_dwell_interrupted"] = bool(start is not None and duration < self.thresholds.recovery_dwell_s)
        return valid

    @staticmethod
    def _event_ordering_ok(events: dict[str, float]) -> bool:
        ordered = [
            EventName.SUPPORTED_START.value,
            EventName.COUNTERMOVEMENT_ONSET.value,
            EventName.VALID_COUNTERMOVEMENT.value,
            EventName.UPWARD_REVERSAL.value,
            EventName.POSITIVE_PROPULSION.value,
            EventName.VALID_TAKEOFF.value,
            EventName.CONTACT_FREE_FLIGHT.value,
            EventName.APEX.value,
            EventName.VALID_LANDING.value,
            EventName.IMPACT_ABSORPTION.value,
            EventName.CAPTURED_SUPPORTED_STATE.value,
            EventName.BOUNDED_RECOVERY.value,
            EventName.COMPLETION.value,
        ]
        present = [events[name] for name in ordered if name in events]
        return all(after + _EPS_TIME >= before for before, after in pairwise(present))

    @staticmethod
    def _no_phase_overlap(events: dict[str, float]) -> bool:
        pairs = (
            (EventName.SUPPORTED_START.value, EventName.COUNTERMOVEMENT_ONSET.value),
            (EventName.COUNTERMOVEMENT_ONSET.value, EventName.VALID_COUNTERMOVEMENT.value),
            (EventName.VALID_COUNTERMOVEMENT.value, EventName.UPWARD_REVERSAL.value),
            (EventName.UPWARD_REVERSAL.value, EventName.POSITIVE_PROPULSION.value),
            (EventName.POSITIVE_PROPULSION.value, EventName.VALID_TAKEOFF.value),
            (EventName.VALID_TAKEOFF.value, EventName.CONTACT_FREE_FLIGHT.value),
            (EventName.CONTACT_FREE_FLIGHT.value, EventName.APEX.value),
            (EventName.APEX.value, EventName.VALID_LANDING.value),
            (EventName.VALID_LANDING.value, EventName.IMPACT_ABSORPTION.value),
            (EventName.IMPACT_ABSORPTION.value, EventName.CAPTURED_SUPPORTED_STATE.value),
            (EventName.CAPTURED_SUPPORTED_STATE.value, EventName.BOUNDED_RECOVERY.value),
            (EventName.BOUNDED_RECOVERY.value, EventName.COMPLETION.value),
        )
        return all(
            left not in events or right not in events or events[right] + _EPS_TIME >= events[left]
            for left, right in pairs
        )

    @staticmethod
    def _support_metric(sample: BiomechanicalSample) -> dict[str, Any]:
        return {
            "bilateral_force_N": sample.foot_normal_force_N.copy(),
            "support_wrench_N_Nm": sample.support_wrench_N_Nm.copy(),
            "cop_xy_m": sample.cop_xy_m.copy(),
            "cop_valid": sample.cop_valid.copy(),
            "linear_momentum_kgmps": (MASS_KG * sample.com_velocity_mps).copy(),
        }

    @staticmethod
    def _fill_phase_metrics(
        samples: Sequence[BiomechanicalSample],
        onset_time_s: float | None,
        reversal_time_s: float | None,
        takeoff_time_s: float | None,
        landing_info: dict[str, Any] | None,
        arrest_time_s: float | None,
        result: CMJEventResult,
    ) -> None:
        phase = result.raw_metrics.setdefault("phase", {})
        phase["countermovement_duration_s"] = (
            float(reversal_time_s - onset_time_s)
            if onset_time_s is not None and reversal_time_s is not None
            else 0.0
        )
        phase["propulsion_interval_s"] = (
            float(takeoff_time_s - reversal_time_s)
            if takeoff_time_s is not None and reversal_time_s is not None
            else 0.0
        )
        if landing_info is not None and takeoff_time_s is not None and landing_info.get("land_first_time_s") is not None:
            phase["flight_duration_s"] = float(landing_info["land_first_time_s"] - takeoff_time_s)
        else:
            phase["flight_duration_s"] = 0.0
        if landing_info is not None and arrest_time_s is not None and landing_info.get("land_first_time_s") is not None:
            phase["landing_absorption_duration_s"] = float(arrest_time_s - landing_info["land_first_time_s"])
        else:
            phase["landing_absorption_duration_s"] = 0.0
        phase["trace_duration_s"] = float(samples[-1].time_s - samples[0].time_s)
        phase["event_times_s"] = dict(result.events)


__all__ = [
    "BASELINE_DT_S",
    "DEFAULT_THRESHOLDS",
    "MASS_KG",
    "ContactLatchTracker",
    "ContactTransition",
    "CMJEventResult",
    "EventName",
    "CMJEventDetector",
    "EventThresholds",
    "InternalEvaluationError",
    "BiomechanicalSample",
    "TerminationClass",
    "detect_contact_chatter",
    "sample_at",
]
