#!/usr/bin/env python3
"""Bounded F2 model, trace, COM, momentum, and centroidal qualifications."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.biomechanics.events import BiomechanicalSample
from loaded_cmj.runtime.results import (
    TRACE_SCHEMA_VERSION,
    canonical_trace_bytes,
    canonical_trace_digest,
)
from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import (
    ATHLETE_MASS_KG,
    EXTERNAL_LOAD_MASS_KG,
    GRAVITY_MAGNITUDE,
    HOLD_ACTION,
    PHYSICS_TIMESTEP_S,
    TOTAL_MASS_KG,
)
from loaded_cmj.simulation.plant import Plant, build_model


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


class Checks:
    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []

    def add(self, test_id: str, assertion: str, ok: bool, measured: Any, tolerance: Any) -> None:
        self.rows.append({
            "test_id": test_id,
            "assertion": assertion,
            "result": "PASS" if bool(ok) else "FAIL",
            "measured": native(measured),
            "tolerance": native(tolerance),
        })

    @property
    def failures(self) -> list[dict[str, Any]]:
        return [row for row in self.rows if row["result"] == "FAIL"]


def _manual_h(plant: Plant, data: mujoco.MjData, com: np.ndarray) -> np.ndarray:
    h = np.zeros(3, dtype=np.float64)
    velocity = np.zeros(6, dtype=np.float64)
    for body_id in plant.idx.all_bodies:
        mujoco.mj_objectVelocity(
            plant.model, data, int(mujoco.mjtObj.mjOBJ_BODY),
            int(body_id), velocity, 0,
        )
        rotation = np.asarray(data.ximat[body_id], dtype=np.float64).reshape(3, 3)
        inertia_world = rotation @ np.diag(plant.model.body_inertia[body_id]) @ rotation.T
        mass = float(plant.model.body_mass[body_id])
        h += inertia_world @ velocity[:3]
        h += np.cross(np.asarray(data.xipos[body_id]) - com, mass * velocity[3:])
    return h


def geometry_checks(checks: Checks) -> dict[str, Any]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    load = int(plant.idx.load_body)
    torso = int(plant.idx.torso_body)
    load_geoms = [gid for gid in plant.idx.shell_geoms if int(model.geom_bodyid[gid]) == load]
    load_geom = load_geoms[0] if len(load_geoms) == 1 else -1
    load_mass = float(model.body_mass[load])
    athlete_mass = float(np.sum(model.body_mass[list(plant.idx.athlete_bodies)]))
    total_mass = float(np.sum(model.body_mass[1:]))
    local_pos = np.asarray(model.body_pos[load], dtype=np.float64)
    local_quat = np.asarray(model.body_quat[load], dtype=np.float64)
    inertia = np.asarray(model.body_inertia[load], dtype=np.float64)
    reference = np.asarray((0.0, 0.0, 0.420), dtype=np.float64)
    radius = float(model.geom_size[load_geom, 0])
    half_length = float(model.geom_size[load_geom, 1])
    expected_inertia = (
        load_mass * (3.0 * radius**2 + (2.0 * half_length)**2) / 12.0,
        0.5 * load_mass * radius**2,
        load_mass * (3.0 * radius**2 + (2.0 * half_length)**2) / 12.0,
    )
    reset_load = np.asarray(data.xpos[load], dtype=np.float64).copy()
    reset_parent = np.asarray(data.xpos[torso], dtype=np.float64).copy()
    reset_expected = reset_parent + local_pos
    carriage_before = plant.load_carriage_transform(data)
    axis_world = np.asarray(data.geom_xmat[load_geom]).reshape(3, 3)[:, 2]
    support_data = plant.make_data()
    plant.reset_fixed_hold(support_data)
    support_summary = plant.contact_wrench_summary(support_data)
    no_load_contact = not any(
        int(data.contact[index].geom1) == load_geom
        or int(data.contact[index].geom2) == load_geom
        for index in range(int(data.ncon))
    )
    data.qpos[7:11] = np.asarray((math.cos(0.12), 0.0, math.sin(0.12), 0.0))
    mujoco.mj_forward(model, data)
    carriage_after = plant.load_carriage_transform(data)
    expected_rotated = np.asarray(data.xpos[torso]) + np.asarray(
        data.xmat[torso]
    ).reshape(3, 3) @ local_pos
    summary = support_summary
    checks.add("F2-198-001", "MJCF dimensions compile", (model.nq, model.nv, model.nu) == (25, 21, 15), (model.nq, model.nv, model.nu), "(25,21,15)")
    checks.add("F2-198-002", "athlete mass is 75 kg", abs(athlete_mass - ATHLETE_MASS_KG) <= 1e-12, athlete_mass, "1e-12 kg")
    checks.add("F2-198-003", "external load mass is 20 kg", abs(load_mass - EXTERNAL_LOAD_MASS_KG) <= 1e-12, load_mass, "1e-12 kg")
    checks.add("F2-198-004", "whole mass is 95 kg", abs(total_mass - TOTAL_MASS_KG) <= 1e-12, total_mass, "1e-12 kg")
    checks.add("F2-198-005", "load is torso child with no joint", int(model.body_parentid[load]) == torso and int(model.body_jntnum[load]) == 0, {"parent": int(model.body_parentid[load]), "torso": torso, "jntnum": int(model.body_jntnum[load])}, "parent=torso; jntnum=0")
    checks.add("F2-198-006", "load is at upper-trunk high-bar reference", np.allclose(local_pos, reference, atol=1e-12, rtol=0.0), local_pos, "[0,0,0.420] m")
    checks.add("F2-198-007", "load attachment is bilateral centerline", np.allclose(local_pos[:2], 0.0, atol=1e-12, rtol=0.0), local_pos[:2], "x=y=0 m")
    checks.add("F2-198-008", "load long axis is mediolateral", abs(float(np.dot(axis_world, (0.0, 1.0, 0.0)))) >= 1.0 - 1e-12, axis_world, "axis dot +y >= 1-1e-12")
    checks.add("F2-198-009", "cylinder visual/inertia match", np.allclose(inertia, expected_inertia, atol=1e-12, rtol=0.0), {"compiled": inertia, "expected": expected_inertia}, "1e-12 kg m2")
    checks.add("F2-198-010", "canonical reset load position is attached to torso", np.allclose(reset_load, reset_expected, atol=1e-12, rtol=0.0), {"actual": reset_load, "expected": reset_expected}, "1e-12 m")
    checks.add("F2-198-011", "load carriage is rigid under torso rotation", np.allclose(carriage_before["rotation"], carriage_after["rotation"], atol=1e-12, rtol=0.0) and np.allclose(carriage_before["translation"], carriage_after["translation"], atol=1e-12, rtol=0.0), {"before": carriage_before, "after": carriage_after}, "unchanged local transform")
    checks.add("F2-198-012", "rotated world load position follows parent", np.allclose(np.asarray(data.xpos[load]), expected_rotated, atol=1e-12, rtol=0.0), {"actual": data.xpos[load], "expected": expected_rotated}, "1e-12 m")
    checks.add("F2-198-013", "load has no unintended reset contact", no_load_contact and not plant.shell_contact(data), {"ncon": int(data.ncon), "shell_contact": plant.shell_contact(data)}, "no load-floor contact")
    checks.add("F2-198-014", "high-bar reference offset is explicit", np.allclose(local_pos - reference, 0.0, atol=1e-12, rtol=0.0), local_pos - reference, "[0,0,0] m")
    checks.add("F2-198-015", "support/COP reset remains valid", np.asarray(summary["normal_force"]).min() > 0.0 and np.asarray(summary["cop_valid"]).all(), {"normal_force": summary["normal_force"], "cop_valid": summary["cop_valid"]}, "bilateral positive support")
    return {
        "load_body": "external_load",
        "parent_body": "torso_head_arms",
        "load_mass_kg": load_mass,
        "athlete_mass_kg": athlete_mass,
        "total_mass_kg": total_mass,
        "load_pos_parent_m": local_pos,
        "load_pos_world_reset_m": reset_load,
        "high_bar_reference_parent_m": reference,
        "offset_from_reference_m": local_pos - reference,
        "load_orientation_wxyz": local_quat,
        "load_inertia_kgm2": inertia,
        "load_geom_id": load_geom,
    }


def make_trace(*, moving: bool, steps: int = 64) -> tuple[Plant, mujoco.MjData, tuple[BiomechanicalSample, ...]]:
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    if moving:
        data.qpos[2] += 0.10
        data.qvel[2] = 0.35
        mujoco.mj_forward(model, data)
    action = np.asarray(HOLD_ACTION, dtype=np.float64)
    state = drive.DriveState(
        a_plus=reset.a_plus,
        a_minus=reset.a_minus,
        tau_prev=reset.previous_torque,
        previous_command=reset.previous_command,
        override_flags=reset.override_flags,
        reversal_phase=reset.reversal_phase,
    )
    samples: list[BiomechanicalSample] = []
    for physics_index in range(1, steps + 1):
        result = drive.drive_state_step(
            action, state, plant.anatomical_coordinates(data),
            plant.anatomical_rates(data), PHYSICS_TIMESTEP_S,
        )
        torque = np.asarray(result["tau"], dtype=np.float64).copy()
        plant.apply_anatomical_torque(data, torque)
        mujoco.mj_step(model, data)
        samples.append(BiomechanicalSample.from_plant(
            plant, data, time_s=physics_index * PHYSICS_TIMESTEP_S,
            accepted_action=action, realized_anatomical_torque=torque,
            actuator_override_flags=result["override_flags"],
            actuator_capacity_lower_Nm=result["capacity_lower"],
            actuator_capacity_upper_Nm=result["capacity_upper"],
            control_index=(physics_index - 1) // 40,
            physics_index=physics_index,
            extraction_status="F2_FIXED_MECHANICS_FIXTURE",
        ))
    return plant, data, tuple(samples)


def mechanics_checks(checks: Checks) -> dict[str, Any]:
    _, _, supported = make_trace(moving=False)
    _, _, moving = make_trace(moving=True)
    closure: dict[str, Any] = {}
    for name, trace in (("supported", supported), ("moving", moving)):
        times = np.asarray([sample.time_s for sample in trace])
        velocity = np.asarray([sample.com_velocity_mps for sample in trace])
        momentum = np.asarray([sample.linear_momentum_world_kg_mps for sample in trace])
        acceleration = np.asarray([sample.com_acceleration_world_mps2 for sample in trace])
        h = np.asarray([sample.centroidal_h_world_kgm2ps for sample in trace])
        hdot = np.asarray([sample.centroidal_hdot_world_kgm2ps2 for sample in trace])
        net_force = np.asarray([sample.whole_support_wrench_N_Nm[:3] for sample in trace])
        net_force += np.asarray((0.0, 0.0, -TOTAL_MASS_KG * GRAVITY_MAGNITUDE))
        impulse = np.trapezoid(net_force, times, axis=0)
        momentum_delta = momentum[-1] - momentum[0]
        acceleration_fd = np.diff(velocity, axis=0) / PHYSICS_TIMESTEP_S
        acceleration_avg = 0.5 * (acceleration[1:] + acceleration[:-1])
        hdot_fd = np.diff(h, axis=0) / PHYSICS_TIMESTEP_S
        hdot_avg = 0.5 * (hdot[1:] + hdot[:-1])
        raw_wrench_residual = []
        for sample in trace:
            plate = np.asarray(sample.plate_wrench_N_Nm)
            origins = np.asarray(sample.plate_origin_world_m)
            common = np.zeros(6, dtype=np.float64)
            for foot in (0, 1):
                force = plate[foot, :3]
                common[:3] += force
                common[3:] += plate[foot, 3:] + np.cross(origins[foot], force)
            common += plate[2]
            raw_wrench_residual.append(
                sample.centroidal_hdot_world_kgm2ps2
                - (common[3:] - np.cross(sample.com_position_m, common[:3]))
            )
        closure[name] = {
            "sample_count": len(trace),
            "momentum_delta_kgmps": momentum[-1] - momentum[0],
            "external_impulse_kgmps": impulse,
            "force_momentum_residual_kgmps": momentum_delta - impulse,
            "max_acceleration_fd_residual_mps2": np.max(np.abs(acceleration_fd - acceleration_avg), axis=0),
            "max_hdot_fd_residual_kgm2ps2": np.max(np.abs(hdot_fd - hdot_avg), axis=0),
            "max_raw_plate_wrench_hdot_residual_Nm": np.max(np.abs(raw_wrench_residual), axis=0),
            "max_com_acceleration_mps2": np.max(np.abs(acceleration), axis=0),
            "max_h_kgm2ps": np.max(np.abs(h), axis=0),
        }
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    plant.reset_supported(data)
    data.qpos[7:11] = np.asarray((math.cos(0.18), 0.0, math.sin(0.18), 0.0))
    data.qvel[0] = 0.25
    data.qvel[6] = 0.31
    mujoco.mj_forward(model, data)
    com = plant.center_of_mass(data)
    h_production = plant.centroidal_angular_momentum(data, com)
    h_independent = _manual_h(plant, data, com)
    checks.add("F2-200-001", "COM acceleration is finite", all(np.isfinite(sample.com_acceleration_world_mps2).all() for trace in (supported, moving) for sample in trace), {name: value["max_com_acceleration_mps2"] for name, value in closure.items()}, "finite")
    checks.add("F2-200-002", "p=Mv holds sample-wise", all(np.isfinite(sample.linear_momentum_world_kg_mps).all() and np.allclose(sample.linear_momentum_world_kg_mps, TOTAL_MASS_KG * sample.com_velocity_mps, atol=1e-12, rtol=0.0) for trace in (supported, moving) for sample in trace), "all samples", "1e-12 kg m s-1")
    checks.add("F2-200-003", "force impulse closes linear momentum", max(np.max(np.abs(value["force_momentum_residual_kgmps"])) for value in closure.values()) <= 1.0e-3, {name: value["force_momentum_residual_kgmps"] for name, value in closure.items()}, "1e-3 kg m s-1; 8 kHz trapezoid")
    checks.add("F2-200-004", "finite-difference velocity independently agrees with acceleration", max(np.max(value["max_acceleration_fd_residual_mps2"]) for value in closure.values()) <= 1.0e-2, {name: value["max_acceleration_fd_residual_mps2"] for name, value in closure.items()}, "1e-2 m s-2; 8 kHz fixed fixtures")
    checks.add("F2-230-001", "H is finite 3-D vector", all(np.isfinite(sample.centroidal_h_world_kgm2ps).all() and np.asarray(sample.centroidal_h_world_kgm2ps).shape == (3,) for trace in (supported, moving) for sample in trace), {name: value["max_h_kgm2ps"] for name, value in closure.items()}, "finite shape (3,)")
    checks.add("F2-230-002", "H contains rotational and translational terms", np.allclose(h_production, h_independent, atol=1e-12, rtol=0.0) and np.linalg.norm(h_production) > 0.0, {"production": h_production, "independent": h_independent}, "1e-12 kg m2 s-1")
    checks.add("F2-230-003", "Hdot is finite and independently differentiated", all(np.isfinite(sample.centroidal_hdot_world_kgm2ps2).all() for trace in (supported, moving)) and max(np.max(value["max_hdot_fd_residual_kgm2ps2"]) for value in closure.values()) <= 1.0e-1, {name: value["max_hdot_fd_residual_kgm2ps2"] for name, value in closure.items()}, "1e-1 N m; 8 kHz fixed fixtures")
    checks.add("F2-230-004", "plate wrench shift reproduces Hdot", max(np.max(value["max_raw_plate_wrench_hdot_residual_Nm"]) for value in closure.values()) <= 1.0e-9, {name: value["max_raw_plate_wrench_hdot_residual_Nm"] for name, value in closure.items()}, "1e-9 N m")
    return {"closure": closure, "h_fixture": {"production": h_production, "independent": h_independent}}


def _json_type_matches(value: Any, expected: str) -> bool:
    if expected == "null":
        return value is None
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "string":
        return isinstance(value, str)
    if expected == "array":
        return isinstance(value, list)
    if expected == "object":
        return isinstance(value, dict)
    raise AssertionError(f"unsupported schema type {expected!r}")


def _validate_json_schema(
    value: Any,
    schema: dict[str, Any],
    root_schema: dict[str, Any],
    path: str = "$",
) -> None:
    reference = schema.get("$ref")
    if reference is not None:
        prefix = "#/$defs/"
        if not reference.startswith(prefix):
            raise AssertionError(f"{path}: unsupported schema reference {reference!r}")
        schema = root_schema["$defs"][reference[len(prefix):]]
    if "const" in schema and value != schema["const"]:
        raise AssertionError(f"{path}: expected const {schema['const']!r}, got {value!r}")
    expected_types = schema.get("type")
    if expected_types is not None:
        if isinstance(expected_types, str):
            expected_types = [expected_types]
        if not any(_json_type_matches(value, expected) for expected in expected_types):
            raise AssertionError(f"{path}: JSON type does not match {expected_types!r}")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            raise AssertionError(f"{path}: value is below minimum")
    if isinstance(value, dict):
        required = schema.get("required", [])
        missing = [key for key in required if key not in value]
        if missing:
            raise AssertionError(f"{path}: missing required keys {missing!r}")
        properties = schema.get("properties", {})
        additional = schema.get("additionalProperties", True)
        unknown = [key for key in value if key not in properties]
        if additional is False and unknown:
            raise AssertionError(f"{path}: unexpected keys {unknown!r}")
        for key, child_schema in properties.items():
            if key in value:
                _validate_json_schema(value[key], child_schema, root_schema, f"{path}.{key}")
        if isinstance(additional, dict):
            for key in unknown:
                _validate_json_schema(value[key], additional, root_schema, f"{path}.{key}")
    elif isinstance(value, list):
        if "minItems" in schema and len(value) < schema["minItems"]:
            raise AssertionError(f"{path}: too few items")
        if "maxItems" in schema and len(value) > schema["maxItems"]:
            raise AssertionError(f"{path}: too many items")
        prefix_items = schema.get("prefixItems", [])
        if schema.get("items") is False and len(value) > len(prefix_items):
            raise AssertionError(f"{path}: unexpected trailing items")
        for index, child_schema in enumerate(prefix_items):
            if index < len(value):
                _validate_json_schema(value[index], child_schema, root_schema, f"{path}[{index}]")
        additional = schema.get("items")
        if isinstance(additional, dict):
            for index in range(len(prefix_items), len(value)):
                _validate_json_schema(value[index], additional, root_schema, f"{path}[{index}]")


def trace_checks(checks: Checks, schema_path: Path) -> dict[str, Any]:
    _, _, first = make_trace(moving=False, steps=24)
    _, _, second = make_trace(moving=False, steps=24)
    first_bytes = canonical_trace_bytes(first)
    second_bytes = canonical_trace_bytes(second)
    first_digest = canonical_trace_digest("f2-fixed-fixture", first)
    second_digest = canonical_trace_digest("f2-fixed-fixture", second)
    record = json.loads(first_bytes.decode("utf-8"))
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    _validate_json_schema(record, schema, schema)
    checks.add(
        "F2-233-001",
        "actual canonical bytes conform to sealed machine schema",
        True,
        {"schema_path": str(schema_path), "top_level_keys": list(record)},
        "schema validator accepts exact payload",
    )
    checks.add(
        "F2-233-002",
        "canonical-byte schema is attempt-independent and closed",
        "attempt_id" not in record
        and "attempt_id" not in schema.get("required", [])
        and "attempt_id" not in schema.get("properties", {})
        and schema.get("additionalProperties") is False,
        {
            "payload_attempt_id": "attempt_id" in record,
            "schema_attempt_id": "attempt_id" in schema.get("properties", {}),
            "top_level_additional_properties": schema.get("additionalProperties"),
        },
        "attempt_id absent; top-level closed",
    )
    checks.add(
        "F2-233-003",
        "sample_count equals actual sample length",
        record["sample_count"] == len(first),
        {"declared": record["sample_count"], "actual": len(first)},
        "exact equality",
    )
    indices = [int(sample["physics_sample_index"]) for sample in record["samples"]]
    checks.add(
        "F2-233-004",
        "sample indices are contiguous and terminal sample is retained",
        indices == list(range(1, len(first) + 1)),
        {"first": indices[0], "last": indices[-1], "count": len(indices)},
        "1..sample_count",
    )
    required = {
        "time_s", "physics_sample_index", "left_plate_wrench_world_N_Nm",
        "right_plate_wrench_world_N_Nm", "force_world_N",
        "moment_common_origin_world_Nm", "left_cop_world_xy_m",
        "right_cop_world_xy_m", "left_cop_valid", "right_cop_valid",
        "com_position_world_m", "com_velocity_world_mps",
        "com_acceleration_world_mps2", "linear_momentum_world_kg_mps",
        "centroidal_h_world_kgm2ps", "centroidal_hdot_world_kgm2ps2",
        "accepted_action", "realized_torque_Nm",
    }
    checks.add("F2-199-001", "canonical schema and required fields present", record["schema_version"] == TRACE_SCHEMA_VERSION and required <= set(record["samples"][0]), {"schema": record["schema_version"], "missing": sorted(required - set(record["samples"][0]))}, "schema and required raw fields")
    checks.add("F2-199-002", "first and terminal samples are retained", first[0].physics_index == 1 and first[-1].physics_index == len(first) and record["sample_count"] == len(first), {"first": first[0].physics_index, "last": first[-1].physics_index, "count": len(first)}, "contiguous 8 kHz indices")
    checks.add("F2-199-003", "identical fixed executions serialize identically", first_bytes == second_bytes and first_digest == second_digest, {"digest_a": first_digest, "digest_b": second_digest, "bytes": len(first_bytes)}, "byte-identical")
    digest_a = canonical_trace_digest("erratum-attempt-a", first)
    digest_b = canonical_trace_digest("erratum-attempt-b", first)
    checks.add(
        "F2-233-005",
        "attempt id is digest framing only",
        first_bytes == second_bytes and digest_a != digest_b,
        {"bytes_equal": first_bytes == second_bytes, "digest_equal": digest_a == digest_b},
        "bytes equal; digest differs by attempt id",
    )
    sealed = first[0].seal()
    immutable = False
    try:
        sealed.com_position_m[0] = 99.0
    except (ValueError, RuntimeError):
        immutable = True
    checks.add("F2-199-004", "sealed samples are immutable", immutable, immutable, "write rejected")
    return {"schema_version": TRACE_SCHEMA_VERSION, "sample_count": len(first), "digest": first_digest, "serialized_bytes": len(first_bytes)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", required=True)
    parser.add_argument("--schema", required=True)
    args = parser.parse_args()
    root = Path(args.evidence_root)
    schema_path = Path(args.schema)
    root.mkdir(parents=True, exist_ok=True)
    checks = Checks()
    measurements: dict[str, Any] = {}
    try:
        measurements["high_bar"] = geometry_checks(checks)
        measurements["trace"] = trace_checks(checks, schema_path)
        measurements["centroidal_mechanics"] = mechanics_checks(checks)
    except Exception as exc:
        checks.add("F2-UNCAUGHT", "focused qualification completes", False, f"{type(exc).__name__}: {exc}", "no exception")
    report = {
        "suite": "qualification_f2_model_measurement",
        "schema_version": TRACE_SCHEMA_VERSION,
        "result": "PASS" if not checks.failures else "FAIL",
        "checks": checks.rows,
        "failures": checks.failures,
        "measurements": measurements,
        "production_paths": {
            "com_acceleration": "Plant.center_of_mass_acceleration",
            "linear_momentum": "Plant.linear_momentum",
            "centroidal_h": "Plant.centroidal_angular_momentum",
            "centroidal_hdot": "Plant.centroidal_hdot_from_external_wrench",
            "trace": "runtime.results.canonical_trace_bytes/digest",
        },
    }
    (root / "qualification_f2_model_measurement.json").write_text(
        json.dumps(native(report), indent=2, sort_keys=True), encoding="utf-8"
    )
    print(json.dumps(native({"result": report["result"], "failures": report["failures"], "measurements": measurements}), sort_keys=True))
    print(f"RESULT={report['result']} F2-MODEL-MEASUREMENT")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
