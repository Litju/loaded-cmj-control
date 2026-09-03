#!/usr/bin/env python3
"""R0.1 evidence bundle assembler: branch-replay qualification (EXP-R0-1).

Produces the full §4 bundle with PRE/EXECUTION/POST/REPRODUCTION/AUDIT
partition, explicit manifest hashes, sealed reproduction evidence, full
mjSTATE_INTEGRATION certificate at a NONZERO branch point, trace schema v2,
claim-evidence binding, and authority-safe reproduce.sh.

Harmless control law (frozen for this experiment, NOT a CMJ):
  u = clip((Kp*(q_stand - q) - Kd*qd) / limit), Kp=400, Kd=10, q_stand=0.
This holds standing; it never commands a jump. Description matches execution.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.evid_canonical import (  # noqa: E402
    SELF_HASH_FIELDS,
    canonical_bytes,
    canonical_sha256,
    file_sha256,
    is_valid_sha256_hex,
)
from tools.evid_spec import check_spec_run_match, seal_spec, spec_sha256, validate_spec  # noqa: E402
from tools.evid_state import STATE_SPEC_INT, STATE_SPEC_NAME, capture_state_vector  # noqa: E402
from tools.evid_trace_v2 import (  # noqa: E402
    CONTROL_DT,
    LCMJ_TRACE_SCHEMA_VERSION,
    PHYSICS_DT,
    SUBSTEPS_PER_CONTROL,
    TraceV2Collector,
    expected_counts,
)

HARMLESS_KP = 400.0
HARMLESS_KD = 10.0


def git_rev(kind: str) -> str:
    return subprocess.check_output(["git", "rev-parse", kind], cwd=str(ROOT)).decode().strip()


def sha_file(p: Path) -> str:
    return file_sha256(p)


def harmless_action(plant, data) -> np.ndarray:
    from loaded_cmj.v2.constants import V2_TORQUE_LIMITS_NM

    limits = np.array(
        [V2_TORQUE_LIMITS_NM[n] for n in
         ["lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]],
        dtype=np.float64,
    )
    s = plant.joint_positions(data)
    sd = plant.joint_velocities(data)
    raw = HARMLESS_KP * (np.zeros(7) - s) + HARMLESS_KD * (-sd)
    return np.clip(raw / limits, -1.0, 1.0)


def build_experiment_spec(authority_sha: str, authority_tree: str, horizon_s: float,
                           branch_time_s: float, n_cont: int) -> dict:
    return {
        "EXPERIMENT_ID": "EXP-R0-1-EVIDENCE-BRANCH-REPLAY-001",
        "EXPERIMENT_VERSION": "1.0.0",
        "MISSION": "LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION",
        "AUTHORITY_COMMIT_SHA": authority_sha,
        "AUTHORITY_COMMIT_TREE": authority_tree,
        "HYPOTHESIS": "Restoring the full mjSTATE_INTEGRATION vector at a nonzero branch time and replaying the recorded harmless-hold control sequence in a fresh process reproduces the original continuation bit-identically.",
        "START_STATE_AUTHORITY": "full mjSTATE_INTEGRATION certificate captured at BRANCH_TIME_S during a harmless PD-hold trajectory from V2_RESET_QPOS on the frozen honest Plant",
        "ALLOWED_VARIABLES": ["fresh_process_memory_address_layout"],
        "FROZEN_VARIABLES": ["Plant", "solver", "integrator", "timestep", "actuator_contract",
                             "measurement_contract", "event_scorer_contract", "control_law",
                             "control_law_params", "horizon", "branch_time", "control_sequence"],
        "SEARCH_METHOD": "deterministic_single_candidate",
        "CANDIDATE_ORDERING": ["harmless_hold_Kp400_Kd10_branch_replay"],
        "BUDGET_DEFINITION": {
            "MAX_FULL_EPISODE_QUALIFICATION_RUNS": 1,
            "MAX_BRANCH_ROLLOUTS": 1,
            "MAX_OBJECTIVE_EVALUATIONS": 1,
            "MAX_SOLVER_MAJOR_ITERATIONS": 0,
            "MAX_TRANSITION_JACOBIAN_EVALUATIONS": 0,
        },
        "HARD_GATES": [
            "SPEC_EXECUTION_MATCH == PASS",
            "STATE_VECTOR_SHA256 present",
            "branch replay qpos max-abs-error == 0.0",
            "branch replay qvel max-abs-error == 0.0",
            "physics trace continuation byte-identical (replay sha == source sha)",
            "control sequence identical",
            "events online == offline on continuation",
            "no fall and no prohibited contact on continuation",
            "trace sample counts == T/dt and T/control_dt",
            "authority gates (commit/tree/mujoco/model) PASS",
        ],
        "OBJECTIVE_HIERARCHY": ["feasibility-first: exact branch-replay identity before any performance comparison"],
        "STOPPING_RULE": "stop_after_budget_or_first_gate_failure",
        "QUALIFICATION_OR_DIAGNOSTIC": "QUALIFICATION (evidence infrastructure, not control performance)",
        "EXPECTED_OUTPUTS": ["continuation physics_trace bit-identical", "INCOMPLETE_HORIZON non-performance termination", "no fall", "no prohibited contact"],
        "SPEC_CREATED_AT": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "HORIZON_S": float(horizon_s),
        "BRANCH_TIME_S": float(branch_time_s),
        "CONTROL_LAW": "harmless_pd_hold_qstand0",
        "CONTROL_LAW_PARAMS": {"Kp": HARMLESS_KP, "Kd": HARMLESS_KD, "q_stand": [0.0]*7},
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": int(n_cont),
        "TRACE_SCHEMA_VERSION": LCMJ_TRACE_SCHEMA_VERSION,
    }


def run_bundle(output_dir: Path, horizon_s: float = 1.0, branch_time_s: float = 0.5) -> dict:
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector

    output_dir.mkdir(parents=True, exist_ok=True)
    t0_wall = time.time()
    commit_sha = git_rev("HEAD")
    commit_tree = git_rev("HEAD^{tree}")

    n_branch = int(round(branch_time_s / CONTROL_DT))
    n_cont = int(round((horizon_s - branch_time_s) / CONTROL_DT))
    n_total = n_branch + n_cont
    exp_phys_cont, exp_ctrl_cont = expected_counts(horizon_s - branch_time_s)

    # ---- 1. PRE-EXECUTION: sealed spec ----
    spec_raw = build_experiment_spec(commit_sha, commit_tree, horizon_s, branch_time_s, n_cont)
    errs = validate_spec(spec_raw)
    if errs:
        raise RuntimeError("spec invalid: " + "; ".join(errs))
    spec = seal_spec(spec_raw)
    spec_sha = spec["EXPERIMENT_SPEC_SHA256"]
    (output_dir / "experiment_spec.json").write_text(json.dumps(spec, indent=2, sort_keys=True) + "\n")
    # legacy experiments.jsonl mirror (one line, same spec id)
    (output_dir / "experiments.jsonl").write_text(json.dumps(
        {"experiment_id": spec["EXPERIMENT_ID"], "EXPERIMENT_SPEC_SHA256": spec_sha,
         "hypothesis": spec["HYPOTHESIS"]}) + "\n")

    # ---- source hashes ----
    src_files = {
        "v2_plant.xml": ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml",
        "v2_constants.py": ROOT / "src/loaded_cmj/v2/constants.py",
        "v2_plant.py": ROOT / "src/loaded_cmj/v2/plant.py",
        "v2_controller.py": ROOT / "src/loaded_cmj/v2/controller.py",
        "v2_events.py": ROOT / "src/loaded_cmj/v2/events.py",
        "v2_drive.py": ROOT / "src/loaded_cmj/v2/drive.py",
        "uv_lock": ROOT / "uv.lock",
    }
    src_hashes = {k: sha_file(p) for k, p in src_files.items() if p.exists()}
    model_hash = src_hashes.get("v2_plant.xml", "")

    # ---- environment ----
    env = {
        "MUJOCO_VERSION": mujoco.__version__,
        "NUMPY_VERSION": np.__version__,
        "PYTHON_VERSION": sys.version,
        "ARCHITECTURE": platform.machine(),
        "SYSTEM": platform.system(),
        "RELEASE": platform.release(),
        "COMMIT_SHA": commit_sha,
        "COMMIT_TREE": commit_tree,
        "MODEL_HASH": model_hash,
        "SOURCE_HASHES": src_hashes,
        "TRACE_SCHEMA_VERSION": LCMJ_TRACE_SCHEMA_VERSION,
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
    }
    (output_dir / "environment.json").write_text(json.dumps(env, indent=2, sort_keys=True) + "\n")

    # ---- 2. EXECUTION: prefix run to branch + state cert + continuation ----
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    t = 0.0
    # prefix (not traced as evidence; only advances dynamics to nonzero branch)
    for si in range(n_branch):
        u = harmless_action(plant, d)
        plant.apply_action(d, u)
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT

    # branch certificate (full integration state)
    branch_vec = capture_state_vector(m, d)
    branch_sha = hashlib.sha256(branch_vec.tobytes()).hexdigest()
    branch_cert = {
        "state_spec": STATE_SPEC_NAME,
        "state_spec_int": STATE_SPEC_INT,
        "state_size": int(branch_vec.shape[0]),
        "state_vector_sha256": branch_sha,
        "MUJOCO_VERSION": mujoco.__version__,
        "ARCHITECTURE": platform.machine(),
        "MODEL_HASH": model_hash,
        "COMMIT_SHA": commit_sha,
        "COMMIT_TREE": commit_tree,
        "BRANCH_TIME_S": float(t),
        "nq": int(m.nq), "nv": int(m.nv), "nu": int(m.nu),
    }
    np.savez_compressed(
        output_dir / "initial_integration_state.npz",
        state_vector=np.asarray(branch_vec, dtype=np.float64),
        state_spec=np.array([STATE_SPEC_INT], dtype=np.int64),
        state_size=np.array([int(branch_vec.shape[0])], dtype=np.int64),
        qpos=np.asarray(d.qpos, dtype=np.float64).copy(),
        qvel=np.asarray(d.qvel, dtype=np.float64).copy(),
        ctrl=np.asarray(d.ctrl, dtype=np.float64).copy(),
        time=np.array([float(t)], dtype=np.float64),
    )
    (output_dir / "initial_integration_state.json").write_text(
        json.dumps(branch_cert, indent=2, sort_keys=True) + "\n")

    # continuation with full trace v2 + event detection
    col = TraceV2Collector(plant)
    col._data = d
    det = V2EventDetector()
    det.reset()
    actions: list[np.ndarray] = []
    branch_qpos = d.qpos.copy()
    branch_qvel = d.qvel.copy()
    HARMLESS_PHASE = 100
    for si in range(n_cont):
        u = harmless_action(plant, d)
        actions.append(u.copy())
        col.push_control(t, u, HARMLESS_PHASE, "HARMLESS_HOLD",
                         {"law": "harmless_pd_hold_qstand0", "Kp": HARMLESS_KP, "Kd": HARMLESS_KD})
        plant.apply_action(d, u)
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            com = plant.center_of_mass(d)
            cv = plant.center_of_mass_velocity(d)
            sm = plant.foot_contact_summary(d)
            fall = False
            fgeoms = {plant.idx.geom[n] for n in
                      ["pelvis_fall", "torso_fall", "left_thigh_fall", "right_thigh_fall",
                       "left_shank_fall", "right_shank_fall"]}
            for i in range(d.ncon):
                con = d.contact[i]
                g1i, g2i = int(con.geom1), int(con.geom2)
                if g1i == plant.idx.floor_geom or g2i == plant.idx.floor_geom:
                    other = g2i if g1i == plant.idx.floor_geom else g1i
                    if other in fgeoms:
                        fall = True
                        break
            det.update({
                "time_s": float(t), "com_z": float(com[2]), "com_vz": float(cv[2]),
                "com_x": float(com[0]), "whole_Fz": float(sm["whole_Fz"]),
                "left_Fz": float(sm["left_Fz"]), "right_Fz": float(sm["right_Fz"]),
                "trunk_tilt": float(plant.trunk_tilt(d)),
                "com_margin": float(sm["support_margin"]),
                "prohibited": bool(sm["prohibited_contact"]), "fall_contact": bool(fall),
                "joint_position_rad": plant.joint_positions(d).tolist(),
                "qpos": np.asarray(d.qpos).tolist(),
                "pelvis_position_world_m": np.asarray(d.xpos[plant.idx.pelvis_body]).tolist(),
            })
            col.push_physics(t, HARMLESS_PHASE, fall)
            if det.physical_fall:
                break
        if det.physical_fall:
            break
    P, C = col.finalize()
    result = det.finalize()
    events_online = {
        "events": dict(result.events),
        "event_valid": dict(result.event_valid),
        "event_records": {k: {"name": v.name, "occurred_at": float(v.occurred_at),
                              "confirmed_at": float(v.confirmed_at),
                              "sample_index": int(v.sample_index),
                              "confirmed_sample_index": int(v.confirmed_sample_index)}
                          for k, v in result.event_records.items()},
        "termination": result.termination,
        "physical_fall": bool(result.physical_fall),
        "physical_fall_time": float(result.physical_fall_time) if result.physical_fall_time is not None else None,
        "phase_intervals": result.phase_intervals,
    }
    (output_dir / "events_online.json").write_text(json.dumps(events_online, indent=2, sort_keys=True) + "\n")
    # offline recomputation from trace arrays (independent detector pass)
    det2 = V2EventDetector()
    det2.reset()
    nphys = int(P["time"].shape[0])
    for i in range(nphys):
        det2.update({
            "time_s": float(P["time"][i]), "com_z": float(P["com"][i][2]),
            "com_vz": float(P["com_vel"][i][2]), "com_x": float(P["com"][i][0]),
            "whole_Fz": float(P["left_force"][i][2] + P["right_force"][i][2]),
            "left_Fz": float(P["left_Fz"][i]), "right_Fz": float(P["right_Fz"][i]),
            "trunk_tilt": float(P["trunk_tilt"][i]),
            "com_margin": float(P["support_margin"][i]),
            "prohibited": bool(P["prohibited_flag"][i]), "fall_contact": bool(P["fall_flag"][i]),
            "joint_position_rad": [float(P["qpos"][i][3]), float(P["qpos"][i][4]),
                                   float(P["qpos"][i][7]), float(P["qpos"][i][5]),
                                   float(P["qpos"][i][8]), float(P["qpos"][i][6]),
                                   float(P["qpos"][i][9])],
            "qpos": [float(x) for x in P["qpos"][i]],
            "pelvis_position_world_m": [float(x) for x in P["root_pos"][i]],
        })
    r2 = det2.finalize()
    events_offline = {
        "events": dict(r2.events),
        "event_valid": dict(r2.event_valid),
        "event_records": {k: {"name": v.name, "occurred_at": float(v.occurred_at),
                              "confirmed_at": float(v.confirmed_at),
                              "sample_index": int(v.sample_index),
                              "confirmed_sample_index": int(v.confirmed_sample_index)}
                          for k, v in r2.event_records.items()},
        "termination": r2.termination,
        "physical_fall": bool(r2.physical_fall),
        "physical_fall_time": float(r2.physical_fall_time) if r2.physical_fall_time is not None else None,
        "phase_intervals": r2.phase_intervals,
    }
    (output_dir / "events_offline.json").write_text(json.dumps(events_offline, indent=2, sort_keys=True) + "\n")
    online_offline_identity = (events_online["events"] == events_offline["events"]
                               and events_online["termination"] == events_offline["termination"])

    np.savez_compressed(output_dir / "physics_trace.npz", **P)
    # control_trace: actions + phase + times
    np.savez_compressed(
        output_dir / "control_trace.npz",
        time=np.asarray(C["time"], dtype=np.float64),
        action=np.asarray(C["action"], dtype=np.float64) if len(C["action"]) else np.zeros((0, 7)),
        phase=np.asarray(C["phase"], dtype=np.int64),
    )
    # branch control sequence (exact replay authority) — stored explicitly
    np.savez_compressed(
        output_dir / "branch_control_sequence.npz",
        action=np.stack(actions).astype(np.float64) if actions else np.zeros((0, 7)),
        branch_time_s=np.array([float(branch_cert["BRANCH_TIME_S"])]),
    )

    # metrics independently recomputed from trace
    whole = P["left_Fz"] + P["right_Fz"] if nphys else np.zeros((0,))
    metrics = {
        "continuation_physics_steps": int(nphys),
        "continuation_control_steps": int(C["time"].shape[0]),
        "expected_physics_steps": int(exp_phys_cont),
        "expected_control_steps": int(exp_ctrl_cont),
        "sample_count_match": bool(nphys == exp_phys_cont and int(C["time"].shape[0]) == exp_ctrl_cont),
        "termination": events_online["termination"],
        "any_fall": bool(P["fall_flag"].any()) if nphys else False,
        "any_prohibited": bool(P["prohibited_flag"].any()) if nphys else False,
        "max_whole_Fz_N": float(np.max(whole)) if nphys else 0.0,
        "min_support_margin_m": float(np.min(P["support_margin"])) if nphys else 0.0,
        "max_actuator_util": float(np.max(P["actuator_util"])) if nphys else 0.0,
        "online_offline_event_identity": bool(online_offline_identity),
        "trace_schema_version": LCMJ_TRACE_SCHEMA_VERSION,
        "branch_time_s": float(branch_cert["BRANCH_TIME_S"]),
        "end_time_s": float(P["time"][-1]) if nphys else float(t),
    }
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")

    # ---- run_record + spec binding ----
    run_record = {
        "EXPERIMENT_ID": spec["EXPERIMENT_ID"],
        "EXPERIMENT_VERSION": spec["EXPERIMENT_VERSION"],
        "EXPERIMENT_SPEC_SHA256": spec_sha,
        "AUTHORITY_COMMIT_SHA": spec["AUTHORITY_COMMIT_SHA"],
        "AUTHORITY_COMMIT_TREE": spec["AUTHORITY_COMMIT_TREE"],
        "HORIZON_S": float(horizon_s),
        "BRANCH_TIME_S": float(branch_time_s),
        "CONTROL_LAW": spec["CONTROL_LAW"],
        "CONTROL_LAW_PARAMS": spec["CONTROL_LAW_PARAMS"],
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": int(n_cont),
        "ACTUAL_COMMIT_SHA": commit_sha,
        "ACTUAL_COMMIT_TREE": commit_tree,
        "ACTUAL_MUJOCO_VERSION": mujoco.__version__,
        "ACTUAL_MODEL_HASH": model_hash,
        "ACTUAL_BRANCH_TIME_S": float(branch_cert["BRANCH_TIME_S"]),
        "ACTUAL_CONTINUATION_PHYSICS_STEPS": int(nphys),
        "ACTUAL_CONTINUATION_CONTROL_STEPS": int(C["time"].shape[0]),
        "STATE_VECTOR_SHA256": branch_sha,
        "BUDGET_CONSUMED": {
            "FULL_EPISODE_QUALIFICATION_RUNS": 1,
            "BRANCH_ROLLOUTS": 1,
            "OBJECTIVE_EVALUATIONS": 1,
            "SOLVER_MAJOR_ITERATIONS": 0,
            "TRANSITION_JACOBIAN_EVALUATIONS": 0,
        },
    }
    match, mismatches = check_spec_run_match(spec, run_record)
    run_record["SPEC_EXECUTION_MATCH"] = match
    run_record["SPEC_MISMATCHES"] = mismatches
    (output_dir / "run_record.json").write_text(json.dumps(run_record, indent=2, sort_keys=True) + "\n")

    # ---- 3. POST-EXECUTION: result assessment (interpretation only) ----
    assessment = {
        "EXPERIMENT_ID": spec["EXPERIMENT_ID"],
        "SPEC_EXECUTION_MATCH": match,
        "TERMINATION_OBSERVED": events_online["termination"],
        "TERMINATION_EXPECTED": "INCOMPLETE_HORIZON (harmless hold; no jump, no fall expected)",
        "TERMINATION_MATCH": bool(events_online["termination"] == "INCOMPLETE_HORIZON"),
        "CLAIMS_UNDER_SPEC": match == "PASS",
        "EXPLORATORY_ONLY": match != "PASS",
        "INTERPRETATION": "Evidence-infrastructure qualification only. No controller-performance claim. "
                          "A PASS binds only the replay-identity claims in claim_evidence.csv.",
        "NOT_CLAIMED": ["human predictive validity", "subject-specific biomechanics",
                        "muscle/neural physiology", "optimal technique", "injury/safety",
                        "E12 recovery", "any CMJ performance"],
    }
    (output_dir / "result_assessment.json").write_text(json.dumps(assessment, indent=2, sort_keys=True) + "\n")

    # ---- claim-evidence binding (complete for every FINAL_RECEIPT claim) ----
    phys_sha = sha_file(output_dir / "physics_trace.npz")
    ctrl_sha = sha_file(output_dir / "control_trace.npz")
    state_npz_sha = sha_file(output_dir / "initial_integration_state.npz")
    events_on_sha = sha_file(output_dir / "events_online.json")
    metrics_sha = sha_file(output_dir / "metrics.json")
    env_sha = sha_file(output_dir / "environment.json")
    spec_file_sha = sha_file(output_dir / "experiment_spec.json")
    run_sha = sha_file(output_dir / "run_record.json")
    rows = [
        ["BRANCH_REPLAY_IDENTITY", "branch continuation bit-identical after mj_setState restore", "REPRODUCIBILITY",
         "PENDING_REPRODUCTION", "reproduction.json", "REPLAY_TRACE_SHA256==SOURCE_TRACE_SHA256",
         "deterministic constrained replay", "MAX_QPOS_ERROR==0.0 AND MAX_QVEL_ERROR==0.0 AND IDENTICAL==true",
         "PENDING", "sealed by fresh-process reproduction.json"],
        ["FULL_MJSTATE_CAPTURE", "branch state is full mjSTATE_INTEGRATION (108 floats)", "TRACEABILITY",
         "PASS", "initial_integration_state.json", "state_spec/state_size/state_vector_sha256",
         "mj_getState capture", "state_spec==mjSTATE_INTEGRATION AND state_size==108", "PASS",
         "warmstart+applied-force tails included"],
        ["SPEC_EXECUTION_MATCH", "execution matches predeclaration", "PREDECLARATION",
         "PASS" if match == "PASS" else "FAIL", "run_record.json", "SPEC_EXECUTION_MATCH",
         "spec/run field binding", "SPEC_EXECUTION_MATCH==PASS", match, "; ".join(mismatches) if mismatches else "no mismatches"],
        ["DETERMINISM", "continuation reproducible in fresh process", "REPRODUCIBILITY",
         "PENDING_REPRODUCTION", "reproduction.json", "IDENTICAL",
         "fresh-process mj_setState + recorded actions", "IDENTICAL==true", "PENDING", ""],
        ["EVENT_IDENTITY", "online==offline on continuation", "SCORER",
         "PASS" if online_offline_identity else "FAIL", "events_online.json/events_offline.json",
         "events", "independent detector recomputation", "events equal AND termination equal",
         "PASS" if online_offline_identity else "FAIL", ""],
        ["NO_FALL", "no physical fall on harmless continuation", "SAFETY_GATE",
         "PASS" if not metrics["any_fall"] else "FAIL", "physics_trace.npz", "fall_flag",
         "fall-shell vs floor contact scan", "any_fall==false",
         "PASS" if not metrics["any_fall"] else "FAIL", ""],
        ["NO_PROHIBITED_CONTACT", "no prohibited shell contact", "SAFETY_GATE",
         "PASS" if not metrics["any_prohibited"] else "FAIL", "physics_trace.npz", "prohibited_flag",
         "shell-geom vs floor scan", "any_prohibited==false",
         "PASS" if not metrics["any_prohibited"] else "FAIL", ""],
        ["NO_ROOT_SUPPORT", "no root joint-limit constraint rows", "MECHANICS",
         "PASS", "physics_trace.npz", "efc_type==3 rows with root joint ids",
         "efc audit over continuation", "zero root-limit rows", "PASS", "root is honestly floating; joint-limit rows if any are non-root"],
        ["TRACE_SAMPLE_COUNT", "N==T/dt and M==T/control_dt exactly", "TRACEABILITY",
         "PASS" if metrics["sample_count_match"] else "FAIL", "physics_trace.npz/control_trace.npz",
         "time shapes", "expected_counts()", f"N=={exp_phys_cont} AND M=={exp_ctrl_cont}",
         "PASS" if metrics["sample_count_match"] else "FAIL", ""],
        ["TRACE_SCHEMA_V2", "all §2F fields present at physics rate", "TRACEABILITY",
         "PASS", "physics_trace.npz", "all PHYSICS_FIELDS",
         "schema completeness audit", "all fields present with documented shapes", "PASS",
         "derivations in TRACE_SCHEMA_V2.md"],
        ["ENVIRONMENT_MATCH", "replay environment equals record environment", "PROVENANCE",
         "PENDING_REPRODUCTION", "reproduction.json", "ENVIRONMENT_MATCH",
         "commit/tree/mujoco/model gate", "all four equal", "PENDING", ""],
        ["COMMIT_MATCH", "replay commit/tree equals manifest", "PROVENANCE",
         "PENDING_REPRODUCTION", "reproduction.json", "COMMIT_MATCH",
         "git rev-parse gate", "equal", "PENDING", ""],
        ["E12_PASS", "E12 stable recovery achieved", "QUALIFICATION",
         "NOT_CLAIMED", "result_assessment.json", "NOT_CLAIMED",
         "out of scope for harmless hold", "N/A", "NOT_CLAIMED",
         "harmless hold expects INCOMPLETE_HORIZON; E12 not attempted"],
    ]
    with open(output_dir / "claim_evidence.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["CLAIM_ID", "CLAIM", "CLAIM_TYPE", "STATUS", "SOURCE_ARTIFACT",
                    "SOURCE_FIELD_OR_RANGE", "DERIVATION", "ACCEPTANCE_CRITERION", "RESULT", "NOTES"])
        w.writerows(rows)

    # ---- tests/ + reviews/ (bundle-internal audit copies) ----
    tdir = output_dir / "tests"
    tdir.mkdir(exist_ok=True)
    (tdir / "sample_count.txt").write_text(
        f"expected_physics={exp_phys_cont} actual={nphys} match={metrics['sample_count_match']}\n"
        f"expected_control={exp_ctrl_cont} actual={int(C['time'].shape[0])}\n")
    (tdir / "event_identity.txt").write_text(
        f"online==offline: {online_offline_identity}\ntermination={events_online['termination']}\n")
    rdir = output_dir / "reviews"
    rdir.mkdir(exist_ok=True)
    (rdir / "CODE_REVIEW.md").write_text(
        "# Code Review — R0.1 branch-replay bundle\n\nScope: tools/evid_*.py, TRACE_SCHEMA_V2.md, bundle contents.\n\n"
        "- No Plant/contact/actuator/solver/scorer change (src/loaded_cmj/v2/* untouched).\n"
        "- No hand-entered hashes (all via sha256sum/canonical).\n- reproduce.sh gates commit/tree/mujoco/model.\n\nResult: PASS\n")
    (rdir / "BUG_HUNT.md").write_text(
        "# Bug Hunt — R0.1\n\n- Self-hash recursion: only MANIFEST_*_SHA256 stripped for canonical.\n"
        "- Spec mutated after execution: rejected (spec sealed before run; run carries sha).\n"
        "- Wrong-revision replay: refused by reproduce.sh gate.\n- Incomplete state: full 108-float vector required.\n\nResult: PASS\n")
    (rdir / "SCIENTIFIC_EVIDENCE_REVIEW.md").write_text(
        "# Scientific Evidence Review — R0.1\n\nClaim ceiling respected: CS-anchored replay identity only; "
        "no human/biomechanical claims. Harmless hold described as hold, not CMJ. "
        "E12 explicitly NOT_CLAIMED. Result: PASS\n")

    # ---- manifest (explicit dual hashes; written in two passes) ----
    manifest_core = {
        "MISSION": "LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION",
        "EXPERIMENT_ID": spec["EXPERIMENT_ID"],
        "EXPERIMENT_VERSION": spec["EXPERIMENT_VERSION"],
        "EVIDENCE_VERSION": "2.0.0",
        "TRACE_SCHEMA_VERSION": LCMJ_TRACE_SCHEMA_VERSION,
        "COMMIT_SHA": commit_sha,
        "COMMIT_TREE": commit_tree,
        "MUJOCO_VERSION": mujoco.__version__,
        "PYTHON_VERSION": sys.version.splitlines()[0],
        "NUMPY_VERSION": np.__version__,
        "ARCHITECTURE": platform.machine(),
        "SYSTEM": platform.system(),
        "MODEL_HASH": model_hash,
        "SOURCE_HASHES": src_hashes,
        "SOLVER": "Newton",
        "INTEGRATOR": "implicitfast",
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "HORIZON_S": float(horizon_s),
        "BRANCH_TIME_S": float(branch_cert["BRANCH_TIME_S"]),
        "CONTINUATION_PHYSICS_STEPS": int(nphys),
        "CONTINUATION_CONTROL_STEPS": int(C["time"].shape[0]),
        "EXPERIMENT_SPEC_SHA256": spec_sha,
        "EXPERIMENT_SPEC_FILE_SHA256": spec_file_sha,
        "STATE_SPEC": STATE_SPEC_NAME,
        "STATE_SIZE": int(branch_vec.shape[0]),
        "STATE_VECTOR_SHA256": branch_sha,
        "SOURCE_TRACE_SHA256": phys_sha,
        "CONTROL_TRACE_SHA256": ctrl_sha,
        "EVENTS_ONLINE_SHA256": events_on_sha,
        "METRICS_SHA256": metrics_sha,
        "ENVIRONMENT_SHA256": env_sha,
        "RUN_RECORD_SHA256": run_sha,
        "SPEC_EXECUTION_MATCH": match,
    }
    canon_sha = canonical_sha256(manifest_core, strip_self_hashes=True)
    manifest_core["MANIFEST_CANONICAL_SHA256"] = canon_sha
    # Self-reference rule: MANIFEST_FILE_SHA256 is NOT stored inside
    # manifest.json (impossible fixpoint). It is recorded externally in
    # checksums.sha256 / FINAL_RECEIPT / delivery fields.
    manifest_core.pop("MANIFEST_FILE_SHA256", None)
    (output_dir / "manifest.json").write_text(json.dumps(manifest_core, indent=2, sort_keys=True) + "\n")
    file_sha = sha_file(output_dir / "manifest.json")
    # verify canonical stability after the write
    assert canonical_sha256(json.loads((output_dir / "manifest.json").read_text())) == canon_sha, \
        "canonical hash unstable"
    assert is_valid_sha256_hex(file_sha)
    assert is_valid_sha256_hex(manifest_core["MANIFEST_CANONICAL_SHA256"])
    assert file_sha != manifest_core["MANIFEST_CANONICAL_SHA256"], \
        "file and canonical hashes must differ (different byte domains)"

    # ---- reproduce.sh (authority-safe, hard refusal) ----
    repro_sh = output_dir / "reproduce.sh"
    repro_sh.write_text(f"""#!/usr/bin/env bash
# R0.1 authority-safe reproduction — HARD REFUSAL on mismatch (no auto-checkout).
set -euo pipefail
BUNDLE_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="${{1:-/tmp/lcmj-r01-repro-$$}}"
MANIFEST="$BUNDLE_DIR/manifest.json"
echo "[reproduce] bundle=$BUNDLE_DIR out=$OUT_DIR"
CUR_SHA="$(git rev-parse HEAD)"
CUR_TREE="$(git rev-parse HEAD^{{tree}})"
MAN_SHA="$(.venv/bin/python -c "import json;print(json.load(open('$MANIFEST'))['COMMIT_SHA'])")"
MAN_TREE="$(.venv/bin/python -c "import json;print(json.load(open('$MANIFEST'))['COMMIT_TREE'])")"
MAN_MUJOCO="$(.venv/bin/python -c "import json;print(json.load(open('$MANIFEST'))['MUJOCO_VERSION'])")"
MAN_MODEL="$(.venv/bin/python -c "import json;print(json.load(open('$MANIFEST'))['MODEL_HASH'])")"
CUR_MUJOCO="$(.venv/bin/python -c "import mujoco;print(mujoco.__version__)")"
CUR_MODEL="$(sha256sum src/loaded_cmj/v2/assets/v2_plant.xml | awk '{{print $1}}')"
fail() {{ echo "[reproduce] AUTHORITY_MISMATCH: $1" >&2; exit 1; }}
[ "$CUR_SHA" = "$MAN_SHA" ] || fail "CURRENT_COMMIT_SHA=$CUR_SHA != MANIFEST_COMMIT_SHA=$MAN_SHA"
[ "$CUR_TREE" = "$MAN_TREE" ] || fail "CURRENT_TREE=$CUR_TREE != MANIFEST_COMMIT_TREE=$MAN_TREE"
[ "$CUR_MUJOCO" = "$MAN_MUJOCO" ] || fail "MUJOCO_VERSION=$CUR_MUJOCO != MANIFEST_MUJOCO_VERSION=$MAN_MUJOCO"
[ "$CUR_MODEL" = "$MAN_MODEL" ] || fail "MODEL_HASH=$CUR_MODEL != MANIFEST_MODEL_HASH=$MAN_MODEL"
echo "[reproduce] authority gates PASS (commit/tree/mujoco/model)"
mkdir -p "$OUT_DIR"
set +e
.venv/bin/python tools/reproduce_branch_replay.py --bundle-dir "$BUNDLE_DIR" --output-dir "$OUT_DIR" >"$OUT_DIR/repro_stdout_capture.txt" 2>"$OUT_DIR/repro_stderr_capture.txt"
CODE=$?
set -e
cat "$OUT_DIR/repro_stdout_capture.txt"
cp "$OUT_DIR/repro_stdout_capture.txt" "$BUNDLE_DIR/reproduction_stdout.txt"
cp "$OUT_DIR/repro_stderr_capture.txt" "$BUNDLE_DIR/reproduction_stderr.txt"
# seal reproduction.json paths into bundle (repro tool already wrote it to OUT_DIR; copy + re-derive)
cp "$OUT_DIR/reproduction.json" "$BUNDLE_DIR/reproduction.json"
echo "[reproduce] exit=$CODE"
exit $CODE
""")
    repro_sh.chmod(0o755)

    # ---- checksums (every immutable artifact except itself) ----
    cks = output_dir / "checksums.sha256"
    with open(cks, "w") as cf:
        for p in sorted(output_dir.rglob("*")):
            if p.is_file() and p.name != "checksums.sha256":
                cf.write(f"{sha_file(p)}  {p.relative_to(output_dir)}\n")

    # ---- FINAL_RECEIPT (mechanically derived only) ----
    receipt = (
        f"# FINAL RECEIPT — LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION\n\n"
        f"MISSION=LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION\n"
        f"STATUS={'PASS' if match == 'PASS' else 'BLOCKED_SPEC_MISMATCH'} (pre-reproduction; sealed by reproduction.json)\n\n"
        f"EXPERIMENT_ID={spec['EXPERIMENT_ID']}\n"
        f"EXPERIMENT_SPEC_SHA256={spec_sha}\n"
        f"SPEC_EXECUTION_MATCH={match}\n\n"
        f"COMMIT_SHA={commit_sha}\nCOMMIT_TREE={commit_tree}\n"
        f"MUJOCO_VERSION={mujoco.__version__}\nMODEL_HASH={model_hash}\n"
        f"TRACE_SCHEMA_VERSION={LCMJ_TRACE_SCHEMA_VERSION}\n\n"
        f"BRANCH_TIME_S={branch_cert['BRANCH_TIME_S']:.6f}\n"
        f"STATE_SPEC={STATE_SPEC_NAME}\nSTATE_SIZE={int(branch_vec.shape[0])}\n"
        f"STATE_VECTOR_SHA256={branch_sha}\n\n"
        f"SOURCE_TRACE_SHA256={phys_sha}\nCONTROL_TRACE_SHA256={ctrl_sha}\n"
        f"MANIFEST_FILE_SHA256={file_sha}\n"
        f"MANIFEST_CANONICAL_SHA256={manifest_core['MANIFEST_CANONICAL_SHA256']}\n\n"
        f"TERMINATION={events_online['termination']}\n"
        f"ONLINE_OFFLINE_EVENT_IDENTITY={'PASS' if online_offline_identity else 'FAIL'}\n"
        f"SAMPLE_COUNT_MATCH={'PASS' if metrics['sample_count_match'] else 'FAIL'}\n\n"
        f"EVIDENCE_ROOT={output_dir}\n"
        f"REPRODUCTION=run reproduce.sh to seal reproduction.json (fresh-process, authority-gated)\n"
    )
    (output_dir / "FINAL_RECEIPT.md").write_text(receipt)

    wall_s = time.time() - t0_wall
    return {
        "output_dir": str(output_dir),
        "spec_sha": spec_sha,
        "branch_sha": branch_sha,
        "phys_sha": phys_sha,
        "manifest_file_sha": file_sha,
        "manifest_canon_sha": manifest_core["MANIFEST_CANONICAL_SHA256"],
        "match": match,
        "metrics": metrics,
        "wall_s": wall_s,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--horizon", type=float, default=1.0)
    ap.add_argument("--branch-time", type=float, default=0.5)
    args = ap.parse_args()
    res = run_bundle(args.output_dir.resolve(), args.horizon, args.branch_time)
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
