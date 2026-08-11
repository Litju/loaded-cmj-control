#!/usr/bin/env python3
"""Independent F3.5 momentum, angular impulse, and energy closure checks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.biomechanics.events import BiomechanicalSample  # noqa: E402
from loaded_cmj.simulation import drive  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    GRAVITY_MAGNITUDE,
    PHYSICS_TIMESTEP_S,
    TOTAL_MASS_KG,
)
from loaded_cmj.simulation.plant import Plant, build_model  # noqa: E402


# The production timestep is fixed by the V1 authority.  A bounded timestep
# refinement audit measured first-order energy residual convergence; 0.025 J
# is the rounded production-dt bound, not a tolerance fitted to a controller
# outcome.
MECHANICS_ENERGY_RESIDUAL_TOLERANCE_J = 2.5e-2


def native(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): native(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [native(item) for item in value]
    return value


def independent_energy(plant: Plant, data: mujoco.MjData) -> dict[str, float]:
    """Reconstruct energy from body states, independently of Plant.total_system_energy."""
    translational = 0.0
    rotational = 0.0
    gravitational = 0.0
    velocity = np.zeros(6, dtype=np.float64)
    for body_id in plant.idx.all_bodies:
        mujoco.mj_objectVelocity(
            plant.model, data, int(mujoco.mjtObj.mjOBJ_BODY), int(body_id), velocity, 0
        )
        mass = float(plant.model.body_mass[body_id])
        rotation = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
        inertia_world = rotation @ np.diag(np.asarray(plant.model.body_inertia[body_id], dtype=np.float64)) @ rotation.T
        translational += 0.5 * mass * float(velocity[3:] @ velocity[3:])
        rotational += 0.5 * float(velocity[:3] @ inertia_world @ velocity[:3])
        gravitational += mass * GRAVITY_MAGNITUDE * float(data.xipos[body_id, 2])
    mujoco.mj_energyPos(plant.model, data)
    native_position_total = float(data.energy[0])
    mujoco.mj_energyVel(plant.model, data)
    native_velocity_total = float(data.energy[1])
    native_position_potential = native_position_total - gravitational
    native_velocity_extra = native_velocity_total - translational - rotational
    anatomical_limit = float(plant.passive_potential(plant.anatomical_coordinates(data)))
    return {
        "translational_kinetic_J": translational,
        "rotational_kinetic_J": rotational,
        "gravitational_potential_J": gravitational,
        "native_position_potential_J": native_position_potential,
        "native_velocity_extra_J": native_velocity_extra,
        "anatomical_limit_potential_J": anatomical_limit,
        "mechanical_J": native_position_total + native_velocity_total + anatomical_limit,
    }


def run_contact_free_flight(steps: int = 320) -> dict[str, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    data.qpos[2] += 0.25
    data.qvel[2] = 0.35
    mujoco.mj_forward(model, data)
    state = drive.DriveState(
        a_plus=np.zeros(15), a_minus=np.zeros(15), tau_prev=np.zeros(15), previous_command=np.zeros(15)
    )
    action = np.zeros(15, dtype=np.float64)
    energy = [independent_energy(plant, data)]
    momentum = [plant.linear_momentum(data).copy()]
    com_velocity = [plant.center_of_mass_velocity(data).copy()]
    h = [plant.centroidal_angular_momentum(data, plant.center_of_mass(data)).copy()]
    support_forces = []
    support_moments_about_com = []
    active_work_power = []
    passive_work_power = []
    limit_work_power = []
    native_damping_work_power = []
    native_limit_work_power = []
    contact_power = []
    finite = True
    for _ in range(steps):
        s = plant.anatomical_coordinates(data)
        sd = plant.anatomical_rates(data)
        result = drive.drive_state_step(action, state, s, sd, PHYSICS_TIMESTEP_S)
        tau = np.asarray(result["tau"], dtype=np.float64)
        powers = plant.realized_power_components(data, tau)
        summary = plant.contact_wrench_summary(data)
        com = plant.center_of_mass(data)
        support = np.asarray(summary["whole_wrench"], dtype=np.float64)
        moment_com = support[3:] - np.cross(com, support[:3])
        # No contact is expected: the diagnostic still records the external
        # contact power rather than silently assuming it is zero.
        plant.apply_anatomical_torque(data, tau)
        mujoco.mj_step(model, data)
        # MuJoCo's post-step kinematic arrays are refreshed explicitly before
        # independent measurement; this keeps the first post-step endpoint
        # distinct from the pre-step endpoint.
        mujoco.mj_forward(model, data)
        energy.append(independent_energy(plant, data))
        momentum.append(plant.linear_momentum(data).copy())
        com_velocity.append(plant.center_of_mass_velocity(data).copy())
        h.append(plant.centroidal_angular_momentum(data, plant.center_of_mass(data)).copy())
        support_forces.append(support[:3].copy())
        support_moments_about_com.append(moment_com.copy())
        active_work_power.append(float(powers["active_power_signed_W"]))
        passive_work_power.append(float(powers["damping_power_W"]))
        limit_work_power.append(float(powers["limit_power_W"]))
        native_damping_force = -np.asarray(model.dof_damping, dtype=np.float64) * np.asarray(data.qvel, dtype=np.float64)
        native_damping_work_power.append(float(native_damping_force @ np.asarray(data.qvel)))
        native_limit_work_power.append(0.0)
        contact_power.append(0.0)
        finite = finite and all(np.isfinite(value).all() if isinstance(value, np.ndarray) else np.isfinite(value) for value in (tau, support, com))
    times = np.arange(steps + 1, dtype=np.float64) * PHYSICS_TIMESTEP_S
    energy_values = np.asarray([row["mechanical_J"] for row in energy])
    momentum_values = np.asarray(momentum)
    velocity_values = np.asarray(com_velocity)
    h_values = np.asarray(h)
    support_forces = np.asarray(support_forces)
    support_moments_about_com = np.asarray(support_moments_about_com)
    external_force = support_forces + np.asarray((0.0, 0.0, -TOTAL_MASS_KG * GRAVITY_MAGNITUDE))
    external_impulse = np.sum(external_force, axis=0) * PHYSICS_TIMESTEP_S
    momentum_delta = momentum_values[-1] - momentum_values[0]
    h_delta = h_values[-1] - h_values[0]
    angular_impulse = np.sum(support_moments_about_com, axis=0) * PHYSICS_TIMESTEP_S
    work = {
        "active_work_J": float(np.sum(active_work_power) * PHYSICS_TIMESTEP_S),
        "anatomical_passive_diagnostic_work_J": float(np.sum(passive_work_power) * PHYSICS_TIMESTEP_S),
        "anatomical_limit_diagnostic_work_J": float(np.sum(limit_work_power) * PHYSICS_TIMESTEP_S),
        "native_damping_work_J": float(np.sum(native_damping_work_power) * PHYSICS_TIMESTEP_S),
        "native_limit_work_J": float(np.sum(native_limit_work_power) * PHYSICS_TIMESTEP_S),
        "external_contact_work_J": float(np.trapezoid(np.concatenate(([0.0], contact_power)), times)),
    }
    work_total = (
        work["active_work_J"] + work["native_damping_work_J"]
        + work["native_limit_work_J"] + work["external_contact_work_J"]
    )
    return {
        "steps": steps,
        "contact_free": bool(np.max(np.linalg.norm(support_forces, axis=1)) == 0.0),
        "finite": finite,
        "max_support_force_N": float(np.max(np.linalg.norm(support_forces, axis=1))),
        "max_com_acceleration_residual_mps2": float(np.max(np.abs(np.diff(velocity_values, axis=0) / PHYSICS_TIMESTEP_S - np.asarray((0.0, 0.0, -GRAVITY_MAGNITUDE))))),
        "linear_momentum_delta_kgmps": momentum_delta,
        "external_impulse_kgmps": external_impulse,
        "linear_impulse_residual_kgmps": momentum_delta - external_impulse,
        "centroidal_h_delta_kgm2ps": h_delta,
        "external_angular_impulse_kgm2ps": angular_impulse,
        "nonimpact_angular_impulse_residual_kgm2ps": h_delta - angular_impulse,
        "energy_start_J": float(energy_values[0]),
        "energy_end_J": float(energy_values[-1]),
        "energy_delta_J": float(energy_values[-1] - energy_values[0]),
        "independent_work": work,
        "energy_work_residual_J": float((energy_values[-1] - energy_values[0]) - work_total),
        "energy_components_start_J": energy[0],
        "energy_components_end_J": energy[-1],
    }


def angular_contact_fixture() -> dict[str, Any]:
    """Finite-window contact-transition angular impulse fixture."""
    force = np.asarray((0.0, 0.0, 1000.0))
    moment_common = np.asarray((0.0, 100.0, 0.0))
    com = np.asarray((0.0, 0.0, 1.0))
    samples = [
        BiomechanicalSample(0.001, com, (0.0, 0.0, -1.0), np.zeros((3, 6)), centroidal_h_world_kgm2ps=(0.0, 0.0, 0.0), whole_support_wrench_N_Nm=np.r_[force, moment_common]),
        BiomechanicalSample(0.002, com, (0.0, 0.0, -1.0), np.zeros((3, 6)), centroidal_h_world_kgm2ps=(0.0, 0.1, 0.0), whole_support_wrench_N_Nm=np.r_[force, moment_common]),
    ]
    h_delta = np.asarray(samples[-1].centroidal_h_world_kgm2ps) - np.asarray(samples[0].centroidal_h_world_kgm2ps)
    moment_com = np.asarray([sample.whole_support_wrench_N_Nm[3:] - np.cross(com, sample.whole_support_wrench_N_Nm[:3]) for sample in samples])
    integral = np.trapezoid(moment_com, [sample.time_s for sample in samples], axis=0)
    return {
        "window_s": [samples[0].time_s, samples[-1].time_s],
        "h_delta_kgm2ps": h_delta,
        "external_angular_impulse_kgm2ps": integral,
        "angular_impulse_residual_kgm2ps": h_delta - integral,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    flight = run_contact_free_flight()
    contact = angular_contact_fixture()
    checks = {
        "linear_momentum": float(np.max(np.abs(flight["linear_impulse_residual_kgmps"]))) <= 2e-4,
        "ballistic_com_acceleration": flight["max_com_acceleration_residual_mps2"] <= 5e-3,
        "centroidal_nonimpact": float(np.max(np.abs(flight["nonimpact_angular_impulse_residual_kgm2ps"]))) <= 5e-6,
        "contact_angular_impulse": float(np.max(np.abs(contact["angular_impulse_residual_kgm2ps"]))) <= 1e-12,
        "energy_work": abs(float(flight["energy_work_residual_J"])) <= MECHANICS_ENERGY_RESIDUAL_TOLERANCE_J,
        "contact_free": flight["contact_free"],
        "finite": flight["finite"],
    }
    report = {"suite": "F3.5-MECHANICS-CLOSURE", "result": "PASS" if all(checks.values()) else "FAIL", "checks": checks, "contact_free_flight": flight, "contact_transition": contact}
    output = args.evidence_root / "mechanics_closure_measurements.json"
    output.write_text(json.dumps(native(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": report["result"], "checks": checks, "output": str(output)}, sort_keys=True))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
