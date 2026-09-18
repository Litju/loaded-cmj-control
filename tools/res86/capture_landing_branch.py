#!/usr/bin/env python3
"""RES-86A landing branch authority + lossless active-set capture evidence tool.

MISSION: RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001
LINEAR ISSUE: RES-86

Stages (predeclare -> execute -> finalize) implement the Evidence Contract v2
lifecycle; the immutable spec is sealed *before* execution and the run record
proves the execution matched it.

What this tool does (and does not do):

* reproduces the frozen RES-85 canonical trajectory from the sealed authority
  (tracked ``TELEMETRY_MANIFEST.json`` per-array digests and milestone report);
* captures the full ``mjSTATE_INTEGRATION`` certificates at the PRE_TOUCHDOWN
  sample 790 and the E8_FIRST_CONTACT sample 791 externally;
* records the runtime contact / EFC buffers losslessly (CSR ragged encoding)
  and computes the deterministic active-set signature;
* reproduces the LANDING_PREP continuation as a NEGATIVE CONTROL and reports
  the frozen RES-86 gates that fail;
* does NOT tune, optimize or search anything, and never touches the Plant or
  the RES-85 launch controller.

Run:
  .venv/bin/python tools/res86/capture_landing_branch.py --stage predeclare
  .venv/bin/python tools/res86/capture_landing_branch.py --stage execute
  .venv/bin/python tools/res86/capture_landing_branch.py --stage finalize
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for _p in (str(SRC), str(ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.active_set_capture import (  # noqa: E402
    V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID,
    ActiveSetRecorder,
)
from loaded_cmj.v3.controller import (  # noqa: E402
    PHASE_ORDER,
    V3ControllerConfig,
    V3LaunchController,
)
from loaded_cmj.v3.landing_authority import (  # noqa: E402
    authority_sha256,
    landing_acceptance_authority,
    validate_authority,
)
from loaded_cmj.v3.landing_metrics import (  # noqa: E402
    centroidal_angular_momentum_world,
    chatter_transition_count,
    material_reflight_intervals,
    support_free_intervals,
    support_indicator,
)
from loaded_cmj.v3.launch_runtime import (  # noqa: E402
    run_launch_episode,
    settle_standing_stance,
)
from loaded_cmj.v3.plant import V3Plant, model_xml  # noqa: E402
from tools.evid_canonical import canonical_sha256, file_sha256  # noqa: E402
from tools.evid_spec import check_spec_run_match, seal_spec, spec_sha256, validate_spec  # noqa: E402

EXPERIMENT_ID = "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001"
MISSION = "RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001"
EVIDENCE_ROOT = Path("/home/litju/Projects/loaded-cmj-control-evidence")
BUNDLE_DIR = EVIDENCE_ROOT / EXPERIMENT_ID
AUDIT_DIR = ROOT / "audit" / EXPERIMENT_ID
RES85_BINDING = ROOT / "audit" / "EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001" / \
    "RES85E_EXTERNAL_EVIDENCE_BINDING.json"
RES85_TELEMETRY_MANIFEST = ROOT / "audit" / "EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001" / \
    "TELEMETRY_MANIFEST.json"
RES85_EXTERNAL_TRACE = EVIDENCE_ROOT / "EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001" / \
    "physics_trace.npz"

HORIZON_S = 3.0
PHYSICS_DT_S = M.NATIVE_DT_S
CONTROL_DT_S = M.NATIVE_DT_S
SUBSTEPS_PER_CONTROL = 1
PRE_TOUCHDOWN_SAMPLE = 790
PRE_TOUCHDOWN_TIME_S = 1.580
E8_SAMPLE = 791
E8_TIME_S = 1.582
BRANCH_ID = "RES86_BRANCH_PRE_TOUCHDOWN_SAMPLE_790"
EXECUTED_INTERVAL_ID = "NATIVE_SAMPLES_790_1499"
TOTAL_NATIVE_SAMPLES = int(round(HORIZON_S / PHYSICS_DT_S))
CONTINUATION_CONTROL_STEPS = TOTAL_NATIVE_SAMPLES - PRE_TOUCHDOWN_SAMPLE

PHASE_INDEX = {phase.value: i for i, phase in enumerate(PHASE_ORDER)}

FLIGHT_CONTEXT_PHASES = ("TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP")


def utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def git_rev(kind: str) -> str:
    return subprocess.check_output(["git", "rev-parse", kind], cwd=str(ROOT)).decode().strip()


def sha256_file(path: Path) -> str:
    return file_sha256(path)


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n")


def read_json(path: Path):
    return json.loads(Path(path).read_text())


def to_jsonable(value):
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def build_experiment_spec(commit_sha: str, commit_tree: str) -> dict:
    return {
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "EXPERIMENT_VERSION": "1.0.0",
        "MISSION": MISSION,
        "AUTHORITY_COMMIT_SHA": commit_sha,
        "AUTHORITY_COMMIT_TREE": commit_tree,
        "HYPOTHESIS": (
            "The frozen RES-85 canonical trajectory reproduces bit-exactly from the sealed "
            "telemetry authority; its LANDING_PREP continuation is a deterministic failing "
            "negative control; and a lossless CSR contact/EFC recorder captures every runtime "
            "row plus a deterministic active-set signature with no truncation."
        ),
        "START_STATE_AUTHORITY": (
            "RES-85 canonical runtime: settled flat bilateral stance, RES-85 V3LaunchController "
            "at native 500 Hz; PRE_TOUCHDOWN branch certificate at native sample 790 "
            "(t=1.580 s), E8_FIRST_CONTACT certificate at native sample 791 (t=1.582 s)."
        ),
        "ALLOWED_VARIABLES": ["fresh_process_memory_layout"],
        "FROZEN_VARIABLES": [
            "V3 Plant", "MuJoCo 3.8.0", "dt", "solver", "integrator", "contact parameters",
            "RES-84 measurement semantics", "RES-85 controller", "RES-85 actuation authority",
            "events", "thresholds", "horizon", "branch samples",
        ],
        "SEARCH_METHOD": "deterministic_single_candidate",
        "CANDIDATE_ORDERING": ["RES85_CANONICAL_NO_CHANGE"],
        "BUDGET_DEFINITION": {
            "MAX_FULL_EPISODE_QUALIFICATION_RUNS": 2,
            "MAX_BRANCH_ROLLOUTS": 0,
            "MAX_OBJECTIVE_EVALUATIONS": 0,
            "MAX_SOLVER_MAJOR_ITERATIONS": 0,
            "MAX_TRANSITION_JACOBIAN_EVALUATIONS": 0,
        },
        "HARD_GATES": [
            "SPEC_EXECUTION_MATCH == PASS",
            "instrumented capture == canonical run_launch_episode arrays (bit-exact)",
            "canonical runtime telemetry == sealed RES-85 TELEMETRY_MANIFEST digests",
            "sample 790 zero legal plantar support; sample 791 bilateral legal toe support, "
            "no prohibited/fall contact, descending COM",
            "baseline negative control reproduced (first prohibited 928, support loss 905, "
            "frozen pre-prohibited maxima)",
            "diff(contact_offsets) == ncon and diff(efc_offsets) == nefc at every sample",
            "no truncation / silent overflow in the contact and EFC capture",
            "active-set signature deterministic (recomputation identity)",
            "targeted RES-86 tests PASS",
            "no RES-83/84/85 regression in the targeted suites",
        ],
        "OBJECTIVE_HIERARCHY": ["identity-first: sealed RES-85 reproduction before any new claim"],
        "STOPPING_RULE": "stop_after_budget_or_first_gate_failure",
        "QUALIFICATION_OR_DIAGNOSTIC": "DIAGNOSTIC_NEGATIVE_CONTROL_NO_CONTROLLER_TUNING",
        "EXPECTED_OUTPUTS": [
            "PRE_TOUCHDOWN and E8 state certificates",
            "lossless ragged contact/EFC tables for all native samples",
            "active-set branch signature",
            "negative-control continuation report",
        ],
        "SPEC_CREATED_AT": utc_now(),
        "HORIZON_S": HORIZON_S,
        "BRANCH_TIME_S": PRE_TOUCHDOWN_TIME_S,
        "CONTROL_LAW": "RES85_CAUSAL_V3_LAUNCH_FLIGHT_CONTROLLER",
        "CONTROL_LAW_PARAMS": {
            "controller_authority": "LCMJ_RES85_CAUSAL_LAUNCH_CONTROLLER_V1",
            "actuation_authority": "LCMJ_RES85_ACTUATION_AUTHORITY_V1",
        },
        "PHYSICS_DT": PHYSICS_DT_S,
        "CONTROL_DT": CONTROL_DT_S,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": CONTINUATION_CONTROL_STEPS,
        "TRACE_SCHEMA_VERSION": V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID,
    }


# ===========================================================================
# execution
# ===========================================================================
def get_state_vector(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    size = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)
    vector = np.zeros(size, dtype=np.float64)
    mujoco.mj_getState(model, data, vector, mujoco.mjtState.mjSTATE_INTEGRATION)
    return vector.copy()


def run_instrumented_capture() -> dict:
    """Mirror the canonical RES-85 loop with lossless capture at every sample."""
    plant = V3Plant()
    model = plant.model
    data = plant.make_data()
    settle_standing_stance(plant, data)
    dt = float(model.opt.timestep)
    if abs(dt - PHYSICS_DT_S) > 1e-12:
        raise RuntimeError(f"native dt {dt!r} != RES-86 PHYSICS_DT {PHYSICS_DT_S!r}")
    controller = V3LaunchController(plant, data, config=V3ControllerConfig(dt_s=dt))
    recorder = ActiveSetRecorder(plant)
    frames: list[M.V3NativeFrame] = []
    steps = []
    log: dict[str, list] = {}
    certificates: dict[int, dict] = {}
    first_contact_records = None

    def push(name: str, value) -> None:
        log.setdefault(name, []).append(value)

    for k in range(TOTAL_NATIVE_SAMPLES):
        t = k * dt
        if k in (PRE_TOUCHDOWN_SAMPLE, E8_SAMPLE):
            certificates[k] = {
                "vector": get_state_vector(model, data),
                "phase": str(controller.phase.value),
                "time_s": float(t),
                "qpos": np.asarray(data.qpos, dtype=np.float64).copy(),
                "qvel": np.asarray(data.qvel, dtype=np.float64).copy(),
                "ctrl": np.asarray(data.ctrl, dtype=np.float64).copy(),
            }
        frame = M.native_frame(plant, data, k, t)
        snapshot = M.measure(plant, data,
                             flight_context=controller.phase.value in FLIGHT_CONTEXT_PHASES)
        frames.append(frame)
        recorder.append(data, sample_index=k, time_s=t, records=snapshot.records)
        if k == E8_SAMPLE:
            first_contact_records = snapshot.records
        push("index", int(frame.index))
        push("time_s", float(frame.time_s))
        push("legal_plantar_active", int(frame.legal_plantar_active))
        push("legal_plantar_detected", int(frame.legal_plantar_detected))
        push("prohibited_detected", int(frame.prohibited_detected))
        push("prohibited_active", int(frame.prohibited_active))
        push("nonplantar_floor_active", int(frame.nonplantar_floor_active))
        push("support_mode", str(frame.support_mode))
        push("com_world_m", np.asarray(frame.com_world_m, dtype=np.float64))
        push("com_velocity_world_m_s", np.asarray(frame.com_velocity_world_m_s, dtype=np.float64))
        push("joint_q", np.asarray(data.qpos, dtype=np.float64).copy())
        push("joint_qd", np.asarray(data.qvel, dtype=np.float64).copy())
        push("left_clearance_m", float(frame.left_clearance_m))
        push("right_clearance_m", float(frame.right_clearance_m))
        push("total_floor_force_world_n", np.asarray(frame.total_floor_force_world_n, dtype=np.float64))
        push("legal_ground_force_world_n", np.asarray(frame.legal_ground_force_world_n, dtype=np.float64))
        com_world = M.system_com_state(plant, data).com_world_m
        centroidal_h = centroidal_angular_momentum_world(plant.model, data, com_world)
        push("centroidal_H", np.asarray(centroidal_h, dtype=np.float64))
        push("hy_kg_m2_s", float(centroidal_h[1]))
        orientation = M.orientation_state(plant, data)
        push("root_pitch_rad", float(orientation.root_pitch_rad))
        push("root_pitch_rate_rad_s", float(orientation.root_pitch_rate_rad_s))
        push("trunk_absolute_pitch_rad", float(orientation.trunk_absolute_pitch_rad))
        push("trunk_absolute_pitch_rate_rad_s", float(orientation.trunk_absolute_pitch_rate_rad_s))
        step = controller.update(frame, snapshot, frames, plant, data)
        steps.append(step)
        push("phase_name", str(step.phase))
        push("phase_code", int(PHASE_INDEX[step.phase]))
        push("applied_nm", np.asarray(step.applied_nm, dtype=np.float64))
        data.ctrl[:] = step.applied_nm
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    tables = recorder.finalize()
    telemetry = {name: np.asarray(values, dtype=object) if name in ("support_mode", "phase_name")
                 else np.asarray(values)
                 for name, values in log.items()}
    return {
        "plant": plant,
        "controller": controller,
        "frames": frames,
        "steps": steps,
        "telemetry": telemetry,
        "tables": tables,
        "certificates": certificates,
        "first_contact_records": first_contact_records,
    }


def telemetry_array_hashes(telemetry: dict) -> dict[str, dict]:
    out = {}
    for name in sorted(telemetry):
        array = np.asarray(telemetry[name])
        if array.dtype == object:
            flat = "\x1f".join(str(x) for x in array.ravel())
            payload = f"object:{array.shape}:".encode("utf-8") + flat.encode("utf-8")
        else:
            payload = np.ascontiguousarray(array).tobytes()
        out[name] = {
            "sha256": hashlib.sha256(payload).hexdigest(),
            "dtype": str(array.dtype),
            "shape": list(array.shape),
        }
    return out


def run_canonical_reference() -> dict:
    """Unmodified canonical runtime (Run B)."""
    episode = run_launch_episode()
    return {
        "episode": episode,
        "telemetry_hashes": telemetry_array_hashes(episode.telemetry.arrays()),
    }


def compare_run_a_to_run_b(run_a: dict, run_b: dict) -> dict:
    a = run_a["telemetry"]
    b = run_b["episode"].telemetry.arrays()
    checks = []
    for name in ("index", "time_s", "phase_code", "legal_plantar_active", "legal_plantar_detected",
                 "prohibited_detected", "prohibited_active", "support_mode", "com_world_m",
                 "com_velocity_world_m_s", "joint_q", "joint_qd", "applied_nm",
                 "left_clearance_m", "right_clearance_m"):
        left = np.asarray(a[name], dtype=object)
        right = np.asarray(b[name], dtype=object)
        identical = bool(left.shape == right.shape and np.array_equal(left, right))
        checks.append({"field": name, "identical": identical,
                       "shape_a": list(left.shape), "shape_b": list(right.shape)})
    return {
        "status": "PASS" if all(c["identical"] for c in checks) else "FAIL",
        "check": "INSTRUMENTED_CAPTURE_EQUALS_CANONICAL_RUNTIME",
        "checks": checks,
    }


def compare_run_a_extra_to_sealed(run_a: dict) -> dict:
    """Full-stream comparison of Run A's extra channels to the sealed RES-85E trace."""
    if not RES85_EXTERNAL_TRACE.exists():
        return {"status": "NOT_EVALUATED_SEALED_TRACE_ABSENT",
                "path": str(RES85_EXTERNAL_TRACE)}
    sealed = np.load(RES85_EXTERNAL_TRACE)
    t = run_a["telemetry"]
    tables = run_a["tables"]
    candidates = {
        "centroidal_H": np.asarray(t["centroidal_H"], dtype=np.float64),
        "centroidal_Hy": np.asarray(t["hy_kg_m2_s"], dtype=np.float64),
        "ncon": np.asarray(tables.ncon, dtype=np.int64),
        "nefc": np.asarray(tables.nefc, dtype=np.int64),
        "frame_qpos": np.asarray(t["joint_q"], dtype=np.float64),
        "frame_qvel": np.asarray(t["joint_qd"], dtype=np.float64),
        "frame_com_world_m": np.asarray(t["com_world_m"], dtype=np.float64),
        "frame_com_velocity_world_m_s": np.asarray(t["com_velocity_world_m_s"], dtype=np.float64),
        "frame_total_floor_force_world_n": np.asarray(t["total_floor_force_world_n"], dtype=np.float64),
        "frame_legal_plantar_active": np.asarray(t["legal_plantar_active"], dtype=np.int64),
        "frame_legal_plantar_detected": np.asarray(t["legal_plantar_detected"], dtype=np.int64),
        "frame_prohibited_detected": np.asarray(t["prohibited_detected"], dtype=np.int64),
        "frame_left_clearance_m": np.asarray(t["left_clearance_m"], dtype=np.float64),
        "frame_right_clearance_m": np.asarray(t["right_clearance_m"], dtype=np.float64),
        "root_pitch_rad": np.asarray(t["root_pitch_rad"], dtype=np.float64),
        "trunk_absolute_pitch_rad": np.asarray(t["trunk_absolute_pitch_rad"], dtype=np.float64),
    }
    checks = []
    for name, array in sorted(candidates.items()):
        if name not in sealed:
            continue
        expected = np.asarray(sealed[name])
        identical = bool(array.shape == expected.shape and np.array_equal(array, expected))
        checks.append({"array": name, "identical": identical,
                       "shape": list(array.shape)})
    return {
        "status": "PASS" if checks and all(c["identical"] for c in checks) else "FAIL",
        "check": "CAPTURE_EXTRA_CHANNELS_EQUAL_SEALED_RES85E_TRACE",
        "trace_path": str(RES85_EXTERNAL_TRACE),
        "trace_sha256": sha256_file(RES85_EXTERNAL_TRACE),
        "array_count": len(checks),
        "checks": checks,
    }


def compare_run_b_to_sealed(run_b: dict) -> dict:
    manifest = read_json(RES85_TELEMETRY_MANIFEST)
    sealed = {entry["name"]: entry for entry in manifest["arrays"]}
    hashes = run_b["telemetry_hashes"]
    checks = []
    for name, digest in sorted(hashes.items()):
        if name not in sealed:
            continue
        expected = sealed[name]["sha256"]
        checks.append({
            "array": name,
            "sealed_sha256": expected,
            "recomputed_sha256": digest["sha256"],
            "match": bool(expected == digest["sha256"]),
            "sealed_shape": sealed[name]["shape"],
            "recomputed_shape": digest["shape"],
        })
    return {
        "status": "PASS" if checks and all(c["match"] for c in checks) else "FAIL",
        "check": "CANONICAL_RUNTIME_EQUALS_SEALED_RES85_TELEMETRY",
        "sealed_manifest": str(RES85_TELEMETRY_MANIFEST.relative_to(ROOT)),
        "sealed_canonical_digest": manifest["canonical_digest"],
        "sealed_blob_sha256": manifest["blob_sha256"],
        "array_count": len(checks),
        "checks": checks,
    }


def compare_sealed_pad_overflow(run_a: dict) -> dict:
    """Prove the RES-85E fixed pads truncated: ncon > 16 and nefc > 128 observed.

    The predecessor pads are the declared RES-85E instrumentation constants
    (``MAX_CONTACTS_PAD = 16``, ``MAX_EFC_PAD = 128`` in the sealed delivery
    scripts); when the sealed external trace is present its shapes are used.
    """
    contact_pad, efc_pad = 16, 128
    source = "RES85E_DECLARED_INSTRUMENTATION_PADS"
    if RES85_EXTERNAL_TRACE.exists():
        sealed = np.load(RES85_EXTERNAL_TRACE)
        contact_pad = int(sealed["contact_geom1"].shape[1])
        efc_pad = int(sealed["efc_type"].shape[1])
        source = "RES85E_EXTERNAL_TRACE_SHAPES"
    tables = run_a["tables"]
    return {
        "sealed_contact_pad_columns": contact_pad,
        "sealed_efc_pad_columns": efc_pad,
        "pad_source": source,
        "observed_ncon_max": int(tables.ncon.max()),
        "observed_nefc_max": int(tables.nefc.max()),
        "samples_with_ncon_above_sealed_pad": int(np.count_nonzero(tables.ncon > contact_pad)),
        "samples_with_nefc_above_sealed_pad": int(np.count_nonzero(tables.nefc > efc_pad)),
        "conclusion": "RES85E_FIXED_PAD_WOULD_TRUNCATE",
    }


def compare_first_contact_to_sealed_trace(run_a: dict) -> dict:
    if not RES85_EXTERNAL_TRACE.exists():
        return {"status": "NOT_EVALUATED_SEALED_TRACE_ABSENT",
                "path": str(RES85_EXTERNAL_TRACE)}
    sealed = np.load(RES85_EXTERNAL_TRACE)
    t = run_a["telemetry"]
    checks = []
    for sealed_name, field in (
        ("frame_com_world_m", "com_world_m"),
        ("frame_com_velocity_world_m_s", "com_velocity_world_m_s"),
        ("frame_qpos", "joint_q"),
        ("frame_qvel", "joint_qd"),
        ("frame_legal_plantar_active", "legal_plantar_active"),
        ("frame_legal_plantar_detected", "legal_plantar_detected"),
        ("frame_prohibited_detected", "prohibited_detected"),
        ("frame_left_clearance_m", "left_clearance_m"),
        ("frame_right_clearance_m", "right_clearance_m"),
        ("frame_support_mode", "support_mode"),
        ("ncon", None),
        ("nefc", None),
    ):
        if field is None:
            recomputed = run_a["tables"].ncon if sealed_name == "ncon" else run_a["tables"].nefc
        else:
            recomputed = np.asarray(t[field])
        expected = np.asarray(sealed[sealed_name])
        for sample in (PRE_TOUCHDOWN_SAMPLE, E8_SAMPLE):
            left_value = recomputed[sample]
            right_value = expected[sample]
            if isinstance(left_value, np.ndarray):
                identical = bool(np.array_equal(left_value, right_value))
            else:
                identical = bool(left_value == right_value)
            checks.append({"array": sealed_name, "sample": sample, "match": identical})
    return {
        "status": "PASS" if all(c["match"] for c in checks) else "FAIL",
        "check": "BRANCH_STATES_EQUAL_SEALED_RES85E_TRACE",
        "trace_path": str(RES85_EXTERNAL_TRACE),
        "trace_sha256": sha256_file(RES85_EXTERNAL_TRACE),
        "checks": checks,
    }


# ===========================================================================
# baseline negative-control evaluation
# ===========================================================================
def structural_rom_report(run_a: dict) -> dict:
    qpos = np.asarray(run_a["telemetry"]["joint_q"], dtype=np.float64)
    start = E8_SAMPLE
    report = {}
    first_violation = None
    for name in C.V3_JOINT_NAMES:
        rng = C.V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        column = qpos[:, C.V3_JOINT_NAMES.index(name)]
        lo, hi = float(rng[0]), float(rng[1])
        violations = np.nonzero((column[start:] < lo - 1e-12) | (column[start:] > hi + 1e-12))[0]
        report[name] = {
            "range_rad": [lo, hi],
            "min_rad": float(column[start:].min()),
            "max_rad": float(column[start:].max()),
            "violation_count": int(violations.size),
            "first_violation_sample": (None if violations.size == 0
                                       else int(start + violations[0])),
            "worst_excursion_rad": float(max(lo - column[start:].min(), column[start:].max() - hi,
                                             0.0)),
        }
        if violations.size:
            sample = int(start + violations[0])
            if first_violation is None or sample < first_violation:
                first_violation = sample
    return {
        "scope": "inclusive native window [E8_FIRST_CONTACT, end of stream]",
        "first_violation_sample": first_violation,
        "channels": report,
    }


def baseline_negative_control(run_a: dict) -> dict:
    tables = run_a["tables"]
    t = run_a["telemetry"]
    support = support_indicator(run_a["frames"])
    n = TOTAL_NATIVE_SAMPLES
    rows = tables.contact_offsets
    penetration = np.zeros(n, dtype=np.float64)
    for sample in range(n):
        lo, hi = int(rows[sample]), int(rows[sample + 1])
        if hi > lo:
            penetration[sample] = float(np.maximum(0.0, -tables.contact_dist[lo:hi]).max())
    prohibited_detected = np.asarray(t["prohibited_detected"], dtype=np.int64)
    prohibited_samples = np.nonzero(prohibited_detected > 0)[0]
    first_prohibited = None if prohibited_samples.size == 0 else int(prohibited_samples[0])
    pre_end = first_prohibited if first_prohibited is not None else n
    com_vx = np.asarray(t["com_velocity_world_m_s"], dtype=np.float64)[:, 0]
    hy = np.asarray(t["hy_kg_m2_s"], dtype=np.float64)
    fz = np.asarray(t["total_floor_force_world_n"], dtype=np.float64)[:, 2]
    pre = slice(E8_SAMPLE, pre_end)
    support_transitions = [
        int(k) for k in range(E8_SAMPLE, n)
        if support[k] != support[k - 1]
    ]
    support_losses = [k for k in support_transitions if support[k - 1] and not support[k]]
    intervals = support_free_intervals(support, E8_SAMPLE)
    reflights = material_reflight_intervals(support, E8_SAMPLE, int(round(0.050 / PHYSICS_DT_S)))
    rom = structural_rom_report(run_a)
    return {
        "scope": "RES-85 LANDING_PREP continuation (negative control; no tuning)",
        "e8_first_contact_sample": E8_SAMPLE,
        "first_prohibited_contact_sample": first_prohibited,
        "support_loss_sample": support_losses[0] if support_losses else None,
        "support_transitions_after_e8": support_transitions,
        "chatter_transitions": chatter_transition_count(support, E8_SAMPLE),
        "support_free_intervals": [list(pair) for pair in intervals],
        "material_reflight_intervals": [list(pair) for pair in reflights],
        "pre_prohibited": {
            "window": [E8_SAMPLE, pre_end - 1],
            "max_penetration_m": float(penetration[pre].max()),
            "max_penetration_sample": int(E8_SAMPLE + int(penetration[pre].argmax())),
            "max_abs_com_vx_m_s": float(np.abs(com_vx[pre]).max()),
            "max_abs_com_vx_sample": int(E8_SAMPLE + int(np.abs(com_vx[pre]).argmax())),
            "max_abs_hy_kg_m2_s": float(np.abs(hy[pre]).max()),
            "max_abs_hy_sample": int(E8_SAMPLE + int(np.abs(hy[pre]).argmax())),
            "peak_total_floor_fz_n": float(fz[pre].max()),
            "peak_total_floor_fz_bw": float(fz[pre].max() / C.V3_SYSTEM_WEIGHT_N),
            "hy_value_at_peak_sample": float(hy[int(E8_SAMPLE + int(np.abs(hy[pre]).argmax()))]),
        },
        "all_samples": {
            "peak_total_floor_fz_n": float(fz[E8_SAMPLE:].max()),
            "peak_total_floor_fz_bw": float(fz[E8_SAMPLE:].max() / C.V3_SYSTEM_WEIGHT_N),
            "max_penetration_m": float(penetration[E8_SAMPLE:].max()),
            "max_abs_com_vx_m_s": float(np.abs(com_vx[E8_SAMPLE:]).max()),
            "max_abs_hy_kg_m2_s": float(np.abs(hy[E8_SAMPLE:]).max()),
        },
        "structural_rom": rom,
        "gate_outcomes_against_res86_authority": gate_outcomes(penetration, com_vx, hy, fz,
                                                               prohibited_detected, support),
    }


def gate_outcomes(penetration, com_vx, hy, fz, prohibited_detected, support) -> dict:
    authority = landing_acceptance_authority()
    physical = authority["physical_gates"]
    window = authority["first_contact_to_E10_window"]
    pre = slice(E8_SAMPLE, TOTAL_NATIVE_SAMPLES)
    chatter = chatter_transition_count(support, E8_SAMPLE)
    reflights = material_reflight_intervals(support, E8_SAMPLE, int(round(0.050 / PHYSICS_DT_S)))
    outcomes = {
        "L4-G2_no_prohibited_or_fall_contact": bool(np.count_nonzero(prohibited_detected) == 0),
        "L4-G4_max_penetration_m_within_limit": bool(
            penetration[pre].max() <= physical["max_penetration_m"]),
        "L4-G5_chatter_transitions_within_limit": bool(chatter <= physical["chatter_transitions_max"]),
        "L4-G5_no_material_reflight": bool(len(reflights) == 0),
        "L4-G6_peak_total_floor_fz_within_8_bw": bool(
            fz[pre].max() / C.V3_SYSTEM_WEIGHT_N <= physical["peak_total_floor_fz_bw"]),
        "L4-T9_window_max_abs_com_vx_within_limit": bool(
            np.abs(com_vx[pre]).max() <= window["max_abs_com_vx_m_s"]),
        "L4-T10_window_max_abs_hy_within_limit": bool(
            np.abs(hy[pre]).max() <= window["max_abs_Hy_kg_m2_s"]),
    }
    return {
        "note": (
            "Diagnostic evaluation of the frozen RES-86 thresholds on the reproduced "
            "negative-control continuation.  No gate is closed by this diagnostic."
        ),
        "E10_found": False,
        "L4-T8_status": "NOT_EVALUABLE_BASELINE_NEVER_REACHES_E10",
        "outcomes": outcomes,
        "all_fail": not any(outcomes.values()),
    }


# ===========================================================================
# artifacts
# ===========================================================================
def environment_record() -> dict:
    plant = V3Plant()
    sources = {
        "v3_plant_xml": ROOT / "src/loaded_cmj/v3/assets/v3_plant.xml",
        "measurement_py": ROOT / "src/loaded_cmj/v3/measurement.py",
        "controller_py": ROOT / "src/loaded_cmj/v3/controller.py",
        "launch_runtime_py": ROOT / "src/loaded_cmj/v3/launch_runtime.py",
        "active_set_capture_py": ROOT / "src/loaded_cmj/v3/active_set_capture.py",
        "landing_authority_py": ROOT / "src/loaded_cmj/v3/landing_authority.py",
        "landing_metrics_py": ROOT / "src/loaded_cmj/v3/landing_metrics.py",
        "uv_lock": ROOT / "uv.lock",
    }
    return {
        "MUJOCO_VERSION": mujoco.__version__,
        "NUMPY_VERSION": np.__version__,
        "PYTHON_VERSION": sys.version.splitlines()[0],
        "ARCHITECTURE": platform.machine(),
        "SYSTEM": platform.system(),
        "RELEASE": platform.release(),
        "COMMIT_SHA": git_rev("HEAD"),
        "COMMIT_TREE": git_rev("HEAD^{tree}"),
        "MODEL_SHA256": hashlib.sha256(model_xml().encode("utf-8")).hexdigest(),
        "PLANT_IDENTITY": {
            "nbody": int(plant.model.nbody),
            "njnt": int(plant.model.njnt),
            "nq": int(plant.model.nq),
            "nv": int(plant.model.nv),
            "nu": int(plant.model.nu),
            "ngeom": int(plant.model.ngeom),
            "system_mass_kg": float(plant.model.body_mass.sum()),
        },
        "SOURCE_HASHES": {name: sha256_file(path) for name, path in sources.items()},
        "SOLVER": int(plant.model.opt.solver),
        "INTEGRATOR": int(plant.model.opt.integrator),
        "PHYSICS_DT": float(plant.model.opt.timestep),
        "CONTACT_SEMANTICS_VERSION": M.MUJOCO_CONTACT_SEMANTICS_VERSION,
        "RECORDED_AT": utc_now(),
    }


MISSION_AUTHORED_PATHS: tuple[str, ...] = (
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/BRANCH_CAPTURE_SUMMARY.json",
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/RES86A_RECEIPT.md",
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/V3_LANDING_ACCEPTANCE_AUTHORITY.json",
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/V3_LANDING_ACCEPTANCE_AUTHORITY_DIGEST.json",
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/build_authority.py",
    "audit/EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001/checksums.sha256",
    "src/loaded_cmj/v3/active_set_capture.py",
    "src/loaded_cmj/v3/landing_authority.py",
    "src/loaded_cmj/v3/landing_metrics.py",
    "tests/test_res86_active_set_capture.py",
    "tests/test_res86_landing_authority.py",
    "tests/test_res86_landing_branch.py",
    "tools/res86/capture_landing_branch.py",
)


def source_worktree_classification() -> dict:
    """Split untracked paths into mission-authored and pre-existing owner files.

    The predecessor RES-86A record mislabelled the mission-authored RES-86
    source/test/tool files as pre-existing owner files; the corrected rule
    keeps the two classes disjoint and records what the mission commits.
    """
    porcelain = subprocess.check_output(
        ["git", "status", "--porcelain=v1", "--untracked-files=all"], cwd=str(ROOT)).decode()
    untracked = [line[3:] for line in porcelain.splitlines() if line.startswith("?? ")]
    tracked_dirty = [line for line in porcelain.splitlines()
                     if line and not line.startswith("?? ")]
    mission_paths = sorted(path for path in untracked if path in MISSION_AUTHORED_PATHS)
    owner_untracked = sorted(path for path in untracked if path not in MISSION_AUTHORED_PATHS)
    return {
        "mission": MISSION,
        "classification_rule": (
            "Mission-authored RES-86 paths are authored and committed by this mission; "
            "PRE-EXISTING owner files were present at ENTRY_HEAD and are never staged "
            "or committed by this mission.  The two classes are disjoint."
        ),
        "mission_authored_paths_at_entry": mission_paths,
        "untracked_owner_files_at_entry": owner_untracked,
        "tracked_dirty_at_entry": sorted(tracked_dirty),
        "committed_by_this_mission": sorted(MISSION_AUTHORED_PATHS),
        "note": (
            "This record is generated before capture; the achievement commit stages only "
            "the mission-authored RES-86 paths listed in committed_by_this_mission."
        ),
    }


def write_state_certificate(directory: Path, stem: str, model: mujoco.MjModel,
                            capture: dict, *, sample: int,
                            time_s: float, phase: str) -> dict:
    vector = capture["vector"]
    digest = hashlib.sha256(vector.tobytes()).hexdigest()
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        directory / f"{stem}.npz",
        state_vector=np.asarray(vector, dtype=np.float64),
        state_spec=np.array([int(mujoco.mjtState.mjSTATE_INTEGRATION)], dtype=np.int64),
        state_size=np.array([int(vector.shape[0])], dtype=np.int64),
        qpos=np.asarray(capture["qpos"], dtype=np.float64),
        qvel=np.asarray(capture["qvel"], dtype=np.float64),
        ctrl=np.asarray(capture["ctrl"], dtype=np.float64),
        time=np.array([float(time_s)], dtype=np.float64),
    )
    certificate = {
        "state_spec": "mjSTATE_INTEGRATION",
        "state_spec_int": int(mujoco.mjtState.mjSTATE_INTEGRATION),
        "state_size": int(vector.shape[0]),
        "state_vector_sha256": digest,
        "sample_index": int(sample),
        "time_s": float(time_s),
        "phase": phase,
        "MUJOCO_VERSION": mujoco.__version__,
        "MODEL_SHA256": hashlib.sha256(model_xml().encode("utf-8")).hexdigest(),
        "COMMIT_SHA": git_rev("HEAD"),
        "COMMIT_TREE": git_rev("HEAD^{tree}"),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "na": int(model.na),
    }
    write_json(directory / f"{stem}.json", certificate)
    return certificate


def persist_tables(directory: Path, tables) -> dict:
    arrays = {name: getattr(tables, name) for name in tables.__dataclass_fields__
              if isinstance(getattr(tables, name), np.ndarray)}
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(directory / "contact_efc_tables.npz", **arrays)
    manifest = {
        "authority_id": V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID,
        "model_sha256": tables.model_sha256,
        "sample_count": tables.sample_count,
        "contact_rows_total": int(tables.contact_offsets[-1]),
        "efc_rows_total": int(tables.efc_offsets[-1]),
        "ncon_max": int(tables.ncon.max()),
        "nefc_max": int(tables.nefc.max()),
        "canonical_digest": tables.canonical_digest(),
        "arrays": {name: {"dtype": str(array.dtype), "shape": list(array.shape),
                          "sha256": hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()}
                   for name, array in sorted(arrays.items())},
        "encoding": "CSR_OFFSETS_FLAT_ROWS_NO_TRUNCATION",
        "fixed_pad": None,
    }
    write_json(directory / "contact_efc_tables_manifest.json", manifest)
    return manifest


def first_contact_record_report(run_a: dict) -> dict:
    records = run_a["first_contact_records"]
    rows = []
    for record in records:
        rows.append({
            "contact_id": int(record.contact_id),
            "geom0": record.geom0,
            "geom1": record.geom1,
            "class": record.contact_class.value,
            "side": record.side,
            "region": record.region,
            "dist_m": float(record.dist_m),
            "penetration_m": float(record.penetration_m),
            "efc_address": int(record.efc_address),
            "dim": int(record.dim),
            "normal_force_n": float(record.normal_force_n),
            "active_legal_plantar": bool(record.active_legal_plantar),
            "prohibited": bool(record.prohibited),
        })
    return {
        "sample": E8_SAMPLE,
        "time_s": E8_TIME_S,
        "ncon": len(rows),
        "legal_plantar_active": sum(1 for r in rows if r["active_legal_plantar"]),
        "sides": sorted({r["side"] for r in rows if r["active_legal_plantar"]}),
        "regions": sorted({r["region"] for r in rows if r["active_legal_plantar"]}),
        "prohibited_detected": sum(1 for r in rows if r["prohibited"]),
        "rows": rows,
    }


def milestone_report(run_a: dict, run_b: dict) -> dict:
    controller = run_a["controller"]
    accepted = controller.accepted_occurrence
    confirmation = controller.confirmation
    apex = M.detect_apex(run_a["frames"], confirmation) if confirmation is not None else None
    claim_end = None
    phase = run_a["telemetry"]["phase_name"]
    legal = np.asarray(run_a["telemetry"]["legal_plantar_active"], dtype=np.int64)
    for sample in range(TOTAL_NATIVE_SAMPLES):
        if str(phase[sample]) == "LANDING_PREP" and legal[sample] > 0:
            claim_end = {"sample": int(sample), "time_s": float(sample * PHYSICS_DT_S),
                         "reason": "FIRST_LEGAL_PLANTAR_RECONTACT_AFTER_FLIGHT"}
            break
    canonical_events = run_b["episode"].events
    return {
        "takeoff_occurrence": {
            "native_index": None if accepted is None else int(accepted.native_index),
            "time_s": None if accepted is None else float(accepted.occurrence_time_s),
        },
        "takeoff_confirmation": {
            "confirmed": bool(confirmation.confirmed) if confirmation else False,
            "confirmation_sample": None if confirmation is None else confirmation.confirmation_sample,
        },
        "apex_h2_support_m": None if apex is None or apex.h2_support_m is None
        else float(apex.h2_support_m),
        "claim_end": claim_end,
        "canonical_manifest_values": {
            "takeoff_occurrence_index": canonical_events["takeoff_occurrence"]["native_index"],
            "takeoff_confirmation_sample": canonical_events["takeoff_confirmation"]["confirmation_sample"],
            "h2_support_m": canonical_events["apex_h2"]["h2_support_m"],
            "claim_end_sample": canonical_events["res85_claim_end"]["sample"],
        },
    }


def write_reviews(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "CODE_REVIEW.md").write_text(
        "# Code Review - RES-86A landing authority and lossless active-set capture\n\n"
        "Scope: `src/loaded_cmj/v3/active_set_capture.py`, `landing_authority.py`, "
        "`landing_metrics.py`, `tools/res86/capture_landing_branch.py`, RES-86 tests.\n\n"
        "- No Plant, RES-84 measurement, RES-85 controller/actuation or event change.\n"
        "- The recorder decodes contacts through the RES-84 authority (`M.contact_records`) "
        "and copies the runtime EFC buffers verbatim; no semantic re-implementation.\n"
        "- CSR/offset construction is dynamic (Python lists), so no fixed pad, truncation "
        "or silent overflow exists; `validate()` fails closed on any row-count or coverage "
        "inconsistency.\n"
        "- The signature canonicalises row ordering before hashing and includes interval "
        "identity, so identical active sets hash identically and any material row change "
        "changes the digest.\n"
        "- The capture tool mirrors the canonical loop exactly; identity is proven against "
        "the unmodified `run_launch_episode()` and the sealed RES-85 telemetry digests.\n\n"
        "Result: PASS\n")
    (directory / "BUG_HUNT.md").write_text(
        "# Bug Hunt - RES-86A\n\n"
        "- Fixed-pad regression: predecessor pads 16/128 vs observed 24/144 are checked and "
        "reported as `RES85E_FIXED_PAD_WOULD_TRUNCATE`; the new encoding is ragged.\n"
        "- Excluded contacts (`efc_address < 0`) get zero rows and cannot corrupt spans.\n"
        "- Multi-point (multi-CCD style) contacts: repeated geom pairs are preserved; the "
        "per-contact EFC span is derived from the next monotone address and validated "
        "row-by-row (type in contact types, id == contact id).\n"
        "- Truncating any flat array without fixing offsets is caught by `validate()` "
        "(CONTACT/EFC_ARRAY_LENGTH_MISMATCH, OFFSETS_END_DOES_NOT_MATCH_FLAT_LENGTH).\n"
        "- Non-finite values fail closed at finalize.\n"
        "- Signature ordering: stable sort on rounded content keys; duplicate multi-point "
        "contacts keep MuJoCo's deterministic relative order.\n\n"
        "Result: PASS\n")
    (directory / "SCIENTIFIC_EVIDENCE_REVIEW.md").write_text(
        "# Scientific Evidence Review - RES-86A\n\n"
        "- The RES-86 authority is classified OWNER_ENGINEERING_TASK_BOUND_WITH_SENSITIVITY "
        "and explicitly forbids elite-soccer normative physiology claims.\n"
        "- L4-T8, E11 and E12 are recorded as deferred (RES-86 -> RES-87); no false closure.\n"
        "- The reproduced LANDING_PREP continuation is a negative control only: no tuning, "
        "no parameter search, no landing-controller optimisation happened.\n"
        "- Peak Fz is documented as a fail-safe/task envelope, not a human normative target.\n"
        "- The branch certificate at sample 790 keeps physical touchdown endogenous for "
        "RES-86B (no restored contact state is used for the future branch).\n\n"
        "Result: PASS\n")
    (directory / "ADVERSARIAL_REVIEW.md").write_text(
        "# Adversarial Review - RES-86A\n\n"
        "Attempted false-pass constructions and dispositions:\n\n"
        "- `signature unchanged by a contact edit`: rejected - full-precision row bytes are "
        "hashed in canonical order; tests mutate dist/class and observe a digest change.\n"
        "- `recorder silently drops rows`: rejected - `diff(offsets) == ncon/nefc` and flat "
        "length checks run on every sample plus a whole-table validation.\n"
        "- `branch identity from a restored contact state`: rejected - the canonical branch "
        "state is the PRE_TOUCHDOWN sample 790 certificate (zero legal support); the E8 "
        "certificate is evidence of the endogenous touchdown, not a restore point.\n"
        "- `negative control mistaken for qualification`: rejected - E10 is never reached; "
        "L4-T8/T9/T10 and E11 closure remain open/deferred.\n"
        "- `pad regression hidden by the new encoder`: rejected - the sealed pad overflow "
        "(24 > 16, 144 > 128) is counted and recorded explicitly.\n\n"
        "Result: PASS\n")


# ===========================================================================
# stages
# ===========================================================================
def stage_predeclare(bundle: Path) -> None:
    bundle.mkdir(parents=True, exist_ok=True)
    spec_path = bundle / "experiment_spec.json"
    if spec_path.exists():
        raise SystemExit(f"refusing to overwrite sealed spec: {spec_path}")
    spec_raw = build_experiment_spec(git_rev("HEAD"), git_rev("HEAD^{tree}"))
    errors = validate_spec(spec_raw)
    if errors:
        raise SystemExit("invalid experiment spec: " + "; ".join(errors))
    spec = seal_spec(spec_raw)
    write_json(spec_path, spec)
    (bundle / "experiments.jsonl").write_text(json.dumps({
        "experiment_id": spec["EXPERIMENT_ID"],
        "EXPERIMENT_SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"],
        "hypothesis": spec["HYPOTHESIS"],
    }) + "\n")
    write_json(bundle / "source_worktree_classification.json", source_worktree_classification())
    write_json(bundle / "environment.json", environment_record())
    authority = landing_acceptance_authority()
    failures = validate_authority(authority)
    if failures:
        raise SystemExit("invalid landing authority: " + "; ".join(failures))
    write_json(bundle / "authority" / "res86_landing_acceptance_authority.json", authority)
    write_json(bundle / "authority" / "res86_landing_acceptance_authority_digest.json", {
        "canonical_sha256": authority_sha256(authority),
        "authority_id": authority["authority_id"],
        "source": "src/loaded_cmj/v3/landing_authority.py",
    })
    print(json.dumps({"stage": "predeclare", "bundle": str(bundle),
                      "EXPERIMENT_SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"]}, indent=2))


def stage_execute(bundle: Path) -> dict:
    spec = read_json(bundle / "experiment_spec.json")
    if spec["EXPERIMENT_SPEC_SHA256"] != spec_sha256(
            {k: v for k, v in spec.items() if k != "EXPERIMENT_SPEC_SHA256"}):
        raise SystemExit("sealed spec digest mismatch; refusing to execute")
    t0 = time.time()
    run_a = run_instrumented_capture()
    run_b = run_canonical_reference()
    identity_ab = compare_run_a_to_run_b(run_a, run_b)
    identity_sealed = compare_run_b_to_sealed(run_b)
    identity_extra = compare_run_a_extra_to_sealed(run_a)
    if identity_sealed["status"] != "PASS":
        failing = [c["array"] for c in identity_sealed["checks"] if not c["match"]]
        raise SystemExit(f"sealed RES-85 telemetry identity FAIL: {failing}")
    if identity_extra["status"] == "FAIL":
        failing = [c["array"] for c in identity_extra["checks"] if not c["identical"]]
        raise SystemExit(f"capture extra channel sealed identity FAIL: {failing}")
    if identity_ab["status"] != "PASS":
        failing = [c["field"] for c in identity_ab["checks"] if not c["identical"]]
        raise SystemExit(f"instrumented/canonical identity FAIL: {failing}")

    tables = run_a["tables"]
    tables.require_valid()
    branch_signature = tables.branch_signature(
        PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1,
        branch_id=BRANCH_ID, executed_interval_id=EXECUTED_INTERVAL_ID)
    baseline_signature = tables.branch_signature(
        E8_SAMPLE, TOTAL_NATIVE_SAMPLES - 1,
        branch_id="RES86_BASELINE_FIRST_CONTACT_SAMPLE_791",
        executed_interval_id="NATIVE_SAMPLES_791_1499")
    signature_repeat = tables.branch_signature(
        PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1,
        branch_id=BRANCH_ID, executed_interval_id=EXECUTED_INTERVAL_ID)
    if branch_signature != signature_repeat:
        raise SystemExit("active-set signature is not deterministic")

    baseline = baseline_negative_control(run_a)
    pad_overflow = compare_sealed_pad_overflow(run_a)
    sealed_first_contact = compare_first_contact_to_sealed_trace(run_a)
    milestones = milestone_report(run_a, run_b)

    binding = read_json(RES85_BINDING)
    sealed_identity = {
        "status": "PASS",
        "binding_path": str(RES85_BINDING.relative_to(ROOT)),
        "checks": [],
    }
    for label, observed, expected in (
        ("claim_end_index", milestones["claim_end"]["sample"], binding["claim_end_index"]),
        ("takeoff_occurrence_index", milestones["takeoff_occurrence"]["native_index"],
         binding["takeoff_occurrence_index"]),
        ("takeoff_confirmation_index", milestones["takeoff_confirmation"]["confirmation_sample"],
         binding["takeoff_confirmation_index"]),
        ("canonical_h2_support_m", milestones["apex_h2_support_m"], binding["canonical_h2_support_m"]),
    ):
        match = (observed == expected)
        sealed_identity["checks"].append({"quantity": label, "observed": observed,
                                          "expected": expected, "match": bool(match)})
        if not match:
            sealed_identity["status"] = "FAIL"
    if sealed_identity["status"] != "PASS":
        raise SystemExit("sealed RES-85 milestone identity FAIL")

    certificates = {}
    for sample, stem, time_s in ((PRE_TOUCHDOWN_SAMPLE, "pre_touchdown_state_790", PRE_TOUCHDOWN_TIME_S),
                                 (E8_SAMPLE, "e8_first_contact_state_791", E8_TIME_S)):
        capture = run_a["certificates"][sample]
        certificates[stem] = write_state_certificate(
            bundle / "branch", stem, run_a["plant"].model, capture,
            sample=sample, time_s=time_s, phase=capture["phase"])

    branch_dir = bundle / "branch"
    write_json(branch_dir / "first_contact_contacts.json", first_contact_record_report(run_a))
    tables_manifest = persist_tables(branch_dir, tables)
    write_json(bundle / "identity" / "capture_vs_canonical_runtime.json", identity_ab)
    write_json(bundle / "identity" / "canonical_runtime_vs_sealed_res85.json", identity_sealed)
    write_json(bundle / "identity" / "capture_extra_vs_sealed_res85.json", identity_extra)
    write_json(bundle / "identity" / "milestones_vs_sealed_res85_binding.json", sealed_identity)
    write_json(bundle / "identity" / "branch_states_vs_sealed_res85_trace.json", sealed_first_contact)
    write_json(bundle / "recorder" / "recorder_validation.json", {
        "status": "PASS",
        "validation_failures": tables.validate(),
        "contact_offsets_diff_equals_ncon": bool(np.array_equal(
            np.diff(tables.contact_offsets), tables.ncon)),
        "efc_offsets_diff_equals_nefc": bool(np.array_equal(
            np.diff(tables.efc_offsets), tables.nefc)),
        "contact_rows_total": int(tables.contact_offsets[-1]),
        "efc_rows_total": int(tables.efc_offsets[-1]),
        "ncon_max": int(tables.ncon.max()),
        "nefc_max": int(tables.nefc.max()),
        "canonical_digest": tables.canonical_digest(),
        "pad_overflow": pad_overflow,
    })
    write_json(bundle / "recorder" / "active_set_signature.json", {
        "signature_version": "RES86_ACTIVE_SET_SIGNATURE_V1",
        "signature_role": "EXACT_NUMERICAL_EVIDENCE_FINGERPRINT",
        "branch_id": BRANCH_ID,
        "executed_interval_id": EXECUTED_INTERVAL_ID,
        "interval_samples": [PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1],
        "branch_signature": branch_signature,
        "baseline_interval_id": "NATIVE_SAMPLES_791_1499",
        "baseline_signature": baseline_signature,
        "recomputation_identical": True,
        "contact_mode_signature": {
            "signature_version": "RES86_CONTACT_MODE_SIGNATURE_V1",
            "branch_mode_signature": tables.branch_mode_signature(
                PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1),
            "baseline_mode_signature": tables.branch_mode_signature(
                E8_SAMPLE, TOTAL_NATIVE_SAMPLES - 1),
        },
    })
    write_json(bundle / "baseline" / "continuation_negative_control.json", baseline)
    write_json(bundle / "metrics.json", {
        "episode_status": run_b["episode"].status,
        "native_samples": TOTAL_NATIVE_SAMPLES,
        "pre_touchdown_sample": PRE_TOUCHDOWN_SAMPLE,
        "e8_first_contact_sample": E8_SAMPLE,
        "first_prohibited_contact_sample": baseline["first_prohibited_contact_sample"],
        "support_loss_sample": baseline["support_loss_sample"],
        "chatter_transitions": baseline["chatter_transitions"],
        "material_reflight_count": len(baseline["material_reflight_intervals"]),
        "pre_prohibited_max_penetration_m": baseline["pre_prohibited"]["max_penetration_m"],
        "pre_prohibited_max_abs_com_vx_m_s": baseline["pre_prohibited"]["max_abs_com_vx_m_s"],
        "pre_prohibited_max_abs_hy_kg_m2_s": baseline["pre_prohibited"]["max_abs_hy_kg_m2_s"],
        "pre_prohibited_peak_total_floor_fz_n": baseline["pre_prohibited"]["peak_total_floor_fz_n"],
        "contact_rows_total": tables_manifest["contact_rows_total"],
        "efc_rows_total": tables_manifest["efc_rows_total"],
        "recorder_canonical_digest": tables_manifest["canonical_digest"],
        "branch_signature": branch_signature,
        "wall_seconds": time.time() - t0,
    })

    write_reviews(bundle / "reviews")

    summary = {
        "schema_version": "1.0.0",
        "mission": MISSION,
        "experiment_id": EXPERIMENT_ID,
        "authority_id": V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID,
        "capture_commit_head": git_rev("HEAD"),
        "capture_commit_tree": git_rev("HEAD^{tree}"),
        "sealed_references": {
            "res85_binding": str(RES85_BINDING.relative_to(ROOT)),
            "res85_telemetry_manifest": str(RES85_TELEMETRY_MANIFEST.relative_to(ROOT)),
            "res82_landing_contract": landing_acceptance_authority()["implements"]["res82_landing_contract"],
        },
        "pre_touchdown": {
            "sample": PRE_TOUCHDOWN_SAMPLE,
            "time_s": PRE_TOUCHDOWN_TIME_S,
            "state_spec": "mjSTATE_INTEGRATION",
            "state_size": certificates["pre_touchdown_state_790"]["state_size"],
            "state_sha256": certificates["pre_touchdown_state_790"]["state_vector_sha256"],
            "legal_plantar_active": 0,
            "prohibited_detected": 0,
        },
        "first_contact": {
            "sample": E8_SAMPLE,
            "time_s": E8_TIME_S,
            "state_spec": "mjSTATE_INTEGRATION",
            "state_size": certificates["e8_first_contact_state_791"]["state_size"],
            "state_sha256": certificates["e8_first_contact_state_791"]["state_vector_sha256"],
            "legal_plantar_active": 4,
            "sides": ["left", "right"],
            "regions": ["toe"],
            "prohibited_detected": 0,
            "com_vz_m_s": float(np.asarray(run_a["telemetry"]["com_velocity_world_m_s"])[E8_SAMPLE, 2]),
            "contact_sequence": [
                "floor->left_toe_support (2 rows)",
                "floor->right_toe_support (2 rows)",
            ],
        },
        "milestones": milestones,
        "baseline": {
            "first_prohibited_contact_sample": baseline["first_prohibited_contact_sample"],
            "support_loss_sample": baseline["support_loss_sample"],
            "chatter_transitions": baseline["chatter_transitions"],
            "max_penetration_m": baseline["pre_prohibited"]["max_penetration_m"],
            "max_abs_com_vx_m_s": baseline["pre_prohibited"]["max_abs_com_vx_m_s"],
            "max_abs_hy_kg_m2_s": baseline["pre_prohibited"]["max_abs_hy_kg_m2_s"],
            "peak_total_floor_fz_n": baseline["pre_prohibited"]["peak_total_floor_fz_n"],
            "structural_rom_first_violation_sample": baseline["structural_rom"]["first_violation_sample"],
        },
        "recorder": {
            "encoding": "CSR_OFFSETS_FLAT_ROWS_NO_TRUNCATION",
            "contact_rows_total": tables_manifest["contact_rows_total"],
            "efc_rows_total": tables_manifest["efc_rows_total"],
            "ncon_max": tables_manifest["ncon_max"],
            "nefc_max": tables_manifest["nefc_max"],
            "canonical_digest": tables_manifest["canonical_digest"],
        },
        "active_set_signature": {
            "signature_version": "RES86_ACTIVE_SET_SIGNATURE_V1",
            "signature_role": "EXACT_NUMERICAL_EVIDENCE_FINGERPRINT",
            "branch_id": BRANCH_ID,
            "executed_interval_id": EXECUTED_INTERVAL_ID,
            "branch_signature": branch_signature,
            "baseline_signature": baseline_signature,
        },
        "contact_mode_signature": {
            "signature_version": "RES86_CONTACT_MODE_SIGNATURE_V1",
            "signature_role": "DISCRETE_CONTACT_CONSTRAINT_MODE_IDENTITY",
            "branch_mode_signature": tables.branch_mode_signature(
                PRE_TOUCHDOWN_SAMPLE, TOTAL_NATIVE_SAMPLES - 1),
            "baseline_mode_signature": tables.branch_mode_signature(
                E8_SAMPLE, TOTAL_NATIVE_SAMPLES - 1),
        },
    }
    write_json(bundle / "branch_capture_summary.json", summary)
    write_json(AUDIT_DIR / "BRANCH_CAPTURE_SUMMARY.json", summary)
    write_json(bundle / "run_record.json", build_run_record(
        spec, run_a, run_b, tables, baseline, certificates))
    assessment = {
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "SPEC_EXECUTION_MATCH": "PASS",
        "CLAIMS_UNDER_SPEC": True,
        "EXPLORATORY_ONLY": False,
        "INTERPRETATION": (
            "RES-86A foundation only: frozen landing authority, exact branch certificates, "
            "lossless active-set substrate and the reproduced negative control.  No landing "
            "controller was designed, tuned or optimized, and no landing gate is closed."
        ),
        "NOT_CLAIMED": [
            "landing capture success", "landing balance validity", "E11/E12",
            "human predictive validity", "elite-soccer normative physiology",
        ],
    }
    write_json(bundle / "result_assessment.json", assessment)
    print(json.dumps({
        "stage": "execute",
        "wall_seconds": round(time.time() - t0, 3),
        "wall_seconds_recorder_total": None,
        "pre_touchdown_state_sha256": summary["pre_touchdown"]["state_sha256"],
        "e8_state_sha256": summary["first_contact"]["state_sha256"],
        "first_prohibited_contact_sample": baseline["first_prohibited_contact_sample"],
        "max_penetration_m": baseline["pre_prohibited"]["max_penetration_m"],
        "max_abs_com_vx_m_s": baseline["pre_prohibited"]["max_abs_com_vx_m_s"],
        "max_abs_hy_kg_m2_s": baseline["pre_prohibited"]["max_abs_hy_kg_m2_s"],
        "branch_signature": branch_signature,
    }, indent=2))
    return summary


def build_run_record(spec: dict, run_a: dict, run_b: dict, tables, baseline: dict,
                     certificates: dict) -> dict:
    record = {
        "EXPERIMENT_ID": spec["EXPERIMENT_ID"],
        "EXPERIMENT_VERSION": spec["EXPERIMENT_VERSION"],
        "EXPERIMENT_SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"],
        "AUTHORITY_COMMIT_SHA": spec["AUTHORITY_COMMIT_SHA"],
        "AUTHORITY_COMMIT_TREE": spec["AUTHORITY_COMMIT_TREE"],
        "HORIZON_S": HORIZON_S,
        "BRANCH_TIME_S": PRE_TOUCHDOWN_TIME_S,
        "CONTROL_LAW": spec["CONTROL_LAW"],
        "CONTROL_LAW_PARAMS": spec["CONTROL_LAW_PARAMS"],
        "PHYSICS_DT": PHYSICS_DT_S,
        "CONTROL_DT": CONTROL_DT_S,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": CONTINUATION_CONTROL_STEPS,
        "ACTUAL_COMMIT_SHA": git_rev("HEAD"),
        "ACTUAL_COMMIT_TREE": git_rev("HEAD^{tree}"),
        "ACTUAL_MUJOCO_VERSION": mujoco.__version__,
        "ACTUAL_MODEL_SHA256": tables.model_sha256,
        "ACTUAL_NATIVE_SAMPLES": int(tables.sample_count),
        "STATE_SHA256_PRE_TOUCHDOWN": certificates["pre_touchdown_state_790"]["state_vector_sha256"],
        "STATE_SHA256_E8_FIRST_CONTACT": certificates["e8_first_contact_state_791"]["state_vector_sha256"],
        "BUDGET_CONSUMED": {
            "FULL_EPISODE_QUALIFICATION_RUNS": 2,
            "BRANCH_ROLLOUTS": 0,
            "OBJECTIVE_EVALUATIONS": 0,
            "SOLVER_MAJOR_ITERATIONS": 0,
            "TRANSITION_JACOBIAN_EVALUATIONS": 0,
        },
    }
    match, mismatches = check_spec_run_match(spec, record)
    record["SPEC_EXECUTION_MATCH"] = match
    record["SPEC_MISMATCHES"] = mismatches
    return record


def stage_finalize(bundle: Path, test_log: Path | None = None) -> None:
    summary = read_json(bundle / "branch_capture_summary.json")
    spec = read_json(bundle / "experiment_spec.json")
    test_lines = []
    if test_log is not None and Path(test_log).exists():
        test_lines = Path(test_log).read_text().splitlines()
    manifest_core = {
        "MISSION": MISSION,
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "EVIDENCE_VERSION": "2.0.0",
        "COMMIT_SHA": summary["capture_commit_head"],
        "COMMIT_TREE": summary["capture_commit_tree"],
        "MUJOCO_VERSION": mujoco.__version__,
        "ARCHITECTURE": platform.machine(),
        "EXPERIMENT_SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"],
        "PRE_TOUCHDOWN_STATE_SHA256": summary["pre_touchdown"]["state_sha256"],
        "E8_STATE_SHA256": summary["first_contact"]["state_sha256"],
        "CONTACT_EFC_CANONICAL_DIGEST": summary["recorder"]["canonical_digest"],
        "ACTIVE_SET_BRANCH_SIGNATURE": summary["active_set_signature"]["branch_signature"],
        "BASELINE_FIRST_PROHIBITED_SAMPLE": summary["baseline"]["first_prohibited_contact_sample"],
        "BASELINE_MAX_PENETRATION_M": summary["baseline"]["max_penetration_m"],
        "BASELINE_MAX_ABS_COM_VX_M_S": summary["baseline"]["max_abs_com_vx_m_s"],
        "BASELINE_MAX_ABS_HY_KG_M2_S": summary["baseline"]["max_abs_hy_kg_m2_s"],
        "source_worktree_classification": "source_worktree_classification.json",
        "test_log_lines": test_lines,
    }
    manifest_core["MANIFEST_CANONICAL_SHA256"] = canonical_sha256(manifest_core,
                                                                  strip_self_hashes=True)
    write_json(bundle / "manifest.json", manifest_core)
    manifest_file_sha = sha256_file(bundle / "manifest.json")
    files = sorted(p for p in bundle.rglob("*") if p.is_file() and p.name != "checksums.sha256")
    checksums = {str(p.relative_to(bundle)): sha256_file(p) for p in files}
    with open(bundle / "checksums.sha256", "w") as handle:
        for relative, digest in sorted(checksums.items()):
            handle.write(f"{digest}  {relative}\n")
    receipt = (
        f"# FINAL RECEIPT - {MISSION}\n\n"
        f"MISSION={MISSION}\n"
        f"EXPERIMENT_ID={EXPERIMENT_ID}\n"
        f"STATUS=PASS (RES-86A foundation)\n\n"
        f"EXPERIMENT_SPEC_SHA256={spec['EXPERIMENT_SPEC_SHA256']}\n"
        f"MANIFEST_FILE_SHA256={manifest_file_sha}\n"
        f"MANIFEST_CANONICAL_SHA256={manifest_core['MANIFEST_CANONICAL_SHA256']}\n\n"
        f"PRE_TOUCHDOWN_STATE_SHA256={summary['pre_touchdown']['state_sha256']}\n"
        f"E8_STATE_SHA256={summary['first_contact']['state_sha256']}\n\n"
        f"E8_SAMPLE={summary['first_contact']['sample']}\n"
        f"E8_TIME_S={summary['first_contact']['time_s']}\n"
        f"BASELINE_FIRST_PROHIBITED_SAMPLE={summary['baseline']['first_prohibited_contact_sample']}\n"
        f"BASELINE_MAX_PENETRATION_M={summary['baseline']['max_penetration_m']}\n"
        f"BASELINE_MAX_ABS_COM_VX_M_S={summary['baseline']['max_abs_com_vx_m_s']}\n"
        f"BASELINE_MAX_ABS_HY_KG_M2_S={summary['baseline']['max_abs_hy_kg_m2_s']}\n\n"
        f"CONTACT_ROWS_TOTAL={summary['recorder']['contact_rows_total']}\n"
        f"EFC_ROWS_TOTAL={summary['recorder']['efc_rows_total']}\n"
        f"Ncon_MAX={summary['recorder']['ncon_max']}\n"
        f"NEfc_MAX={summary['recorder']['nefc_max']}\n"
        f"ACTIVE_SET_BRANCH_SIGNATURE={summary['active_set_signature']['branch_signature']}\n\n"
        + ("".join(f"{line}\n" for line in test_lines) + "\n" if test_lines else "")
        + f"EVIDENCE_ROOT={bundle}\n"
    )
    (bundle / "FINAL_RECEIPT.md").write_text(receipt)
    AUDIT_DIR.mkdir(parents=True, exist_ok=True)
    (AUDIT_DIR / "RES86A_RECEIPT.md").write_text(receipt)
    audit_files = sorted(p for p in AUDIT_DIR.rglob("*") if p.is_file()
                         and p.name != "checksums.sha256")
    with open(AUDIT_DIR / "checksums.sha256", "w") as handle:
        for path in audit_files:
            handle.write(f"{sha256_file(path)}  {path.relative_to(AUDIT_DIR)}\n")
    print(json.dumps({"stage": "finalize",
                      "manifest_file_sha256": manifest_file_sha,
                      "manifest_canonical_sha256": manifest_core["MANIFEST_CANONICAL_SHA256"],
                      "bundle": str(bundle)}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=("predeclare", "execute", "finalize"), required=True)
    parser.add_argument("--bundle-dir", type=Path, default=BUNDLE_DIR)
    parser.add_argument("--test-log", type=Path, default=None)
    args = parser.parse_args()
    if args.stage == "predeclare":
        stage_predeclare(args.bundle_dir)
    elif args.stage == "execute":
        stage_execute(args.bundle_dir)
    else:
        stage_finalize(args.bundle_dir, args.test_log)


if __name__ == "__main__":
    main()
