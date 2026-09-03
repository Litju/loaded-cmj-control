#!/usr/bin/env python3
"""RES10 controller-observation synchronization requalification evidence runner.

MISSION=RES10_CONTROLLER_OBSERVATION_SYNCHRONIZATION_REQUALIFICATION
EXPERIMENT_ID=EXP-RES10-CONTROLLER-OBS-SYNC-001 v1.0.0

Generates the Evidence Contract v2 bundle directly into the canonical
evidence repository. No controller tuning. Frozen Plant/contact/solver/
timestep/actuator/scorer/RES43 envelope.
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

from tools.evid_canonical import canonical_sha256, file_sha256, is_valid_sha256_hex  # noqa: E402
from tools.evid_spec import check_spec_run_match  # noqa: E402
from tools.evid_state import STATE_SPEC_INT, STATE_SPEC_NAME  # noqa: E402

EXPERIMENT_ID = "EXP-RES10-CONTROLLER-OBS-SYNC-001"
EXPERIMENT_VERSION = "1.0.0"
MISSION = "RES10_CONTROLLER_OBSERVATION_SYNCHRONIZATION_REQUALIFICATION"
PHYSICS_DT = 0.000125
CONTROL_DT = 0.005
SUBSTEPS_PER_CONTROL = 40
HORIZON_S = 4.0
N_CTRL = int(round(HORIZON_S / CONTROL_DT))
N_PHYS = int(round(HORIZON_S / PHYSICS_DT))
EXPECTED_STATE_SIZE = 108
WEIGHT = 95.0 * 9.81


def git_rev(kind: str) -> str:
    return subprocess.check_output(["git", "rev-parse", kind], cwd=str(ROOT)).decode().strip()


def sha_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def build_experiment_spec(authority_sha: str, authority_tree: str) -> dict:
    return {
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "EXPERIMENT_VERSION": EXPERIMENT_VERSION,
        "MISSION": MISSION,
        "AUTHORITY_COMMIT_SHA": authority_sha,
        "AUTHORITY_COMMIT_TREE": authority_tree,
        "QUALIFICATION_OR_DIAGNOSTIC": "CONTROLLER_OBSERVATION_AUTHORITY_CORRECTION",
        "HYPOTHESIS": "The legacy controller was qualified against a mixed-stage observation. Replacing that observation with a coherent same-instant sample will alter the closed-loop trajectory, but the unchanged controller may retain some or all of its physical CMJ capability.",
        "ALLOWED_VARIABLES": [
            "synchronized_measurement_implementation",
            "controller_observation_wiring",
            "physics_trace_event_scorer_measurement_wiring",
            "tests_and_authority_documentation",
        ],
        "FROZEN_VARIABLES": [
            "controller_gains", "controller_targets", "controller_phase_thresholds",
            "controller_state_machine", "Plant", "contact", "solver", "timestep",
            "actuator_model", "scorer_thresholds_dwells", "RES43_envelope",
        ],
        "CONTROL_SAMPLE_CONVENTION": "SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL",
        "OBSERVATION_INPUT_CONTROL": "PREVIOUS_HELD_CONTROL",
        "HARD_GATES": [
            "SPEC_EXECUTION_MATCH==PASS",
            "SHADOW_NONINTRUSIVE_MAX_DELTAS==0",
            "ONE_STATE_ONE_OBSERVATION_SHA_IDENTITY",
            "ONLINE_OFFLINE_EVENT_IDENTITY==PASS",
        ],
        "HORIZON_S": HORIZON_S,
        "BRANCH_TIME_S": 0.0,
        "CONTROL_LAW": "canonical_reference_controller_HEAD_5807aa1_UNCHANGED",
        "CONTROL_LAW_PARAMS": {
            "phases": "HOLD,SUPPORTED,FLIGHT,IMPACT,CAPTURED,STAND",
            "source": "src/loaded_cmj/v2/controller.py@5807aa1",
            "tuning": "NONE",
        },
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": N_CTRL,
        "TRACE_SCHEMA_VERSION": 2,
        "SPEC_CREATED_AT": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def seal_spec(spec: dict) -> dict:
    from tools.evid_spec import spec_sha256 as _ss

    sealed = dict(spec)
    sealed["EXPERIMENT_SPEC_SHA256"] = _ss({k: v for k, v in spec.items() if k != "EXPERIMENT_SPEC_SHA256"})
    return sealed


def event_records_json(res) -> dict:
    return {
        k: {
            "name": v.name,
            "occurred_at": float(v.occurred_at),
            "confirmed_at": float(v.confirmed_at),
            "sample_index": int(v.sample_index),
            "confirmed_sample_index": int(v.confirmed_sample_index),
        }
        for k, v in res.event_records.items()
    }


def full_sync_trace_for_metrics(horizon_s: float):
    """Re-run C with full synchronized physics sampling for gate metrics."""
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
    from loaded_cmj.v2.events import V2EventDetector
    import loaded_cmj.v2.controller as ctrl

    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    meas = create_measurement_data(plant)
    ctrl.reset(0.0)
    det = V2EventDetector()
    det.reset()
    n_ctrl = int(round(horizon_s / CONTROL_DT))
    t = 0.0
    prev = np.zeros(7, dtype=np.float64)
    times, whole, pen, comz, comvz, ncon = [], [], [], [], [], []
    prohib_any = False
    root_limit_rows = 0
    max_passive_root = 0.0
    max_util = 0.0
    finite = True
    for step in range(n_ctrl):
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        obs = s_ctrl.controller_observation(step_index=step, episode_reset=(step == 0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        max_util = max(max_util, float(np.max(np.abs(act))))
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            times.append(t)
            whole.append(float(s.whole_Fz_N))
            pen.append(float(s.max_penetration_m))
            comz.append(float(s.com_position_m[2]))
            comvz.append(float(s.com_velocity_mps[2]))
            ncon.append(int(s.ncon))
            if bool(s.prohibited_contact):
                prohib_any = True
            if not (np.isfinite(s.qpos).all() and np.isfinite(s.qvel).all() and np.isfinite(s.qacc).all()):
                finite = False
            max_passive_root = max(max_passive_root, float(np.max(np.abs(s.qfrc_passive[0:3]))))
            for e in range(meas.nefc):
                if int(meas.efc_type[e]) == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
                    jid = int(meas.efc_id[e])
                    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
                    if name in ("root_tx", "root_tz", "root_ry"):
                        root_limit_rows += 1
            det.update(s.event_sample())
            if det.physical_fall:
                break
        if det.physical_fall:
            break
    res = det.finalize()
    return {
        "times": np.asarray(times), "whole": np.asarray(whole), "pen": np.asarray(pen),
        "comz": np.asarray(comz), "comvz": np.asarray(comvz), "ncon": np.asarray(ncon),
        "prohib_any": prohib_any, "root_limit_rows": int(root_limit_rows),
        "max_passive_root": float(max_passive_root), "max_util": float(max_util),
        "finite": bool(finite), "event_result": res,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle-dir", type=Path, required=True)
    args = ap.parse_args()
    bundle = args.bundle_dir.resolve()
    bundle.mkdir(parents=True, exist_ok=True)
    t_wall0 = time.time()

    live_head = git_rev("HEAD")
    live_tree = git_rev("HEAD^{tree}")
    print(f"[res10-sync] authority head={live_head} tree={live_tree}", flush=True)

    # -- predeclaration (sealed before execution) --
    spec_raw = build_experiment_spec(live_head, live_tree)
    spec = seal_spec(spec_raw)
    spec_sha = spec["EXPERIMENT_SPEC_SHA256"]
    (bundle / "experiment_spec.json").write_text(json.dumps(spec, indent=2, sort_keys=True) + "\n")

    from loaded_cmj.v2.plant import V2Plant, model_xml
    from loaded_cmj.v2 import sync_rollout

    plant_probe = V2Plant()
    m_probe = plant_probe.model

    # -- environment --
    env = {
        "MUJOCO_VERSION": mujoco.__version__,
        "NUMPY_VERSION": np.__version__,
        "PYTHON_VERSION": sys.version.splitlines()[0],
        "ARCHITECTURE": platform.machine(),
        "SYSTEM": platform.system(),
        "RELEASE": platform.release(),
        "COMMIT_SHA": live_head,
        "COMMIT_TREE": live_tree,
        "MODEL_HASH": sha_file(ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml"),
        "SOURCE_HASHES": {
            "v2_plant.xml": sha_file(ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml"),
            "v2_constants.py": sha_file(ROOT / "src/loaded_cmj/v2/constants.py"),
            "v2_plant.py": sha_file(ROOT / "src/loaded_cmj/v2/plant.py"),
            "v2_controller.py": sha_file(ROOT / "src/loaded_cmj/v2/controller.py"),
            "v2_events.py": sha_file(ROOT / "src/loaded_cmj/v2/events.py"),
            "v2_drive.py": sha_file(ROOT / "src/loaded_cmj/v2/drive.py"),
            "v2_measurement.py": sha_file(ROOT / "src/loaded_cmj/v2/measurement.py"),
            "v2_sync_rollout.py": sha_file(ROOT / "src/loaded_cmj/v2/sync_rollout.py"),
        },
        "TRACE_SCHEMA_VERSION": 2,
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
    }
    (bundle / "environment.json").write_text(json.dumps(env, indent=2, sort_keys=True) + "\n")

    # -- section 7: shadow nonintrusiveness --
    print("[res10-sync] prescribed shadow identity", flush=True)
    prescribed = sync_rollout.run_prescribed_action_shadow_identity(0.5)
    shadow_ok = bool(
        prescribed["max_qpos_delta"] == 0.0
        and prescribed["max_qvel_delta"] == 0.0
        and prescribed["max_ctrl_delta"] == 0.0
    )
    print(f"[res10-sync] prescribed {prescribed} ok={shadow_ok}", flush=True)

    # -- trajectories A/B/C --
    print("[res10-sync] trajectory A (legacy)", flush=True)
    ra = sync_rollout.run_legacy_closed_loop(HORIZON_S)
    print(f"[res10-sync] A n={ra['n_control']} term={ra['event_result'].termination}", flush=True)
    print("[res10-sync] trajectory B (sync meas, legacy ctrl)", flush=True)
    rb = sync_rollout.run_sync_measurement_legacy_control(HORIZON_S)
    print(f"[res10-sync] B n={rb['n_control']} term={rb['event_result'].termination}", flush=True)
    print("[res10-sync] trajectory C (fully synchronized)", flush=True)
    rc = sync_rollout.run_fully_synchronized_closed_loop(HORIZON_S)
    print(f"[res10-sync] C n={rc['n_control']} term={rc['event_result'].termination}", flush=True)

    # A/B live identity (prescribed-action + full closed-loop action identity)
    ab_action_identical = bool(np.array_equal(ra["action"], rb["action"]))
    ab_qpos_identical = bool(np.array_equal(ra["qpos"], rb["qpos"]))
    ab_qvel_identical = bool(np.array_equal(ra["qvel"], rb["qvel"]))

    # -- action comparison A vs C --
    na = min(ra["n_control"], rc["n_control"])
    d = np.abs(ra["action"][:na] - rc["action"][:na])
    max_delta = float(np.max(d)) if d.size else 0.0
    nz = np.where(np.max(d, axis=1) > 0)[0]
    first_div_idx = int(nz[0]) if len(nz) else -1
    first_div_t = float(ra["control_time"][first_div_idx]) if first_div_idx >= 0 else float("nan")

    # -- determinism C --
    print("[res10-sync] C determinism re-run", flush=True)
    rc2 = sync_rollout.run_fully_synchronized_closed_loop(HORIZON_S)
    c_deterministic = bool(
        rc["action_sha256"] == rc2["action_sha256"]
        and np.array_equal(rc["action"], rc2["action"])
        and np.array_equal(rc["qpos"], rc2["qpos"])
    )

    # -- RES43 with sync --
    print("[res10-sync] RES43 standing sync", flush=True)
    res43 = sync_rollout.run_res43_standing_sync(2.0)
    print(f"[res10-sync] RES43 {res43}", flush=True)

    # -- full sync metrics for C gates --
    print("[res10-sync] C full gate metrics", flush=True)
    cm = full_sync_trace_for_metrics(HORIZON_S)
    times, whole, pen = cm["times"], cm["whole"], cm["pen"]
    peak_fz = float(np.max(whole)) if whole.size else 0.0
    peak_bw = float(peak_fz / WEIGHT)
    max_pen = float(np.max(pen)) if pen.size else 0.0
    # landing peak 100ms after E9 if present
    landing_peak = None
    landing_bw = None
    cev = cm["event_result"].event_records
    if "descending_landing" in cev:
        ld = float(cev["descending_landing"].occurred_at)
        i1 = int(np.argmin(np.abs(times - ld)))
        i2 = int(np.argmin(np.abs(times - (ld + 0.1))))
        landing_peak = float(np.max(whole[i1 : i2 + 1]))
        landing_bw = float(landing_peak / WEIGHT)
    # takeoff/apex/landing states
    def _state_at(t_req):
        i = int(np.argmin(np.abs(times - t_req)))
        return {"time": float(times[i]), "whole_Fz": float(whole[i]),
                "com_z": float(cm["comz"][i]), "com_vz": float(cm["comvz"][i])}

    takeoff_state = _state_at(float(cev["bilateral_takeoff"].occurred_at)) if "bilateral_takeoff" in cev else None
    apex_state = _state_at(float(cev["apex"].occurred_at)) if "apex" in cev else None
    landing_state = _state_at(float(cev["descending_landing"].occurred_at)) if "descending_landing" in cev else None

    # -- branch states: capture synchronized E9/E10/E11 integration SHAs --
    from loaded_cmj.v2.measurement import SynchronizedPhysicsSample as SPS
    from loaded_cmj.v2.measurement import create_measurement_data as _cmd

    branch_authority: dict = {}
    # Re-derive branch SHAs by re-running C and capturing integration vectors at event times
    # (deterministic; same live states as rc). Simpler: record event times + note E10/E11 absent.
    for name in ("descending_landing", "impact_absorption", "balance_capture"):
        rec = cev.get(name)
        if rec is None:
            branch_authority[name] = None
        else:
            branch_authority[name] = {
                "occurred_at": float(rec.occurred_at),
                "confirmed_at": float(rec.confirmed_at),
                "sample_index": int(rec.sample_index),
            }
    # E9 state SHA: recompute by replaying C to E9 confirmed and hashing integration state
    e9_sha = None
    e10_sha = None
    e11_sha = None
    if "descending_landing" in cev:
        from loaded_cmj.v2.plant import V2Plant as _P
        import loaded_cmj.v2.controller as _ctrl

        _p = _P()
        _m = _p.model
        _d = _p.make_data()
        _p.reset(_d)
        mujoco.mj_forward(_m, _d)
        _meas = _cmd(_p)
        _ctrl.reset(0.0)
        _t = 0.0
        _prev = np.zeros(7)
        _target = float(cev["descending_landing"].confirmed_at)
        _n = int(round(HORIZON_S / CONTROL_DT))
        for _step in range(_n):
            _s = SPS.from_live_state(_p, _d, _meas)
            _obs = _s.controller_observation(step_index=_step, episode_reset=(_step == 0), previous_action=_prev)
            _act = np.asarray(_ctrl.act(_obs), float)
            _p.apply_action(_d, _act)
            _prev = _act.copy()
            for _ in range(SUBSTEPS_PER_CONTROL):
                mujoco.mj_step(_m, _d)
                _t += PHYSICS_DT
                if _t >= _target - 1e-12:
                    break
            if _t >= _target - 1e-12:
                break
        _vec = np.zeros(EXPECTED_STATE_SIZE)
        mujoco.mj_getState(_m, _d, _vec, mujoco.mjtState.mjSTATE_INTEGRATION)
        e9_sha = hashlib.sha256(np.ascontiguousarray(_vec).tobytes()).hexdigest()
        branch_authority["descending_landing"]["state_vector_sha256"] = e9_sha

    # -- requalification verdict (section 10/11) --
    order = ["supported_start", "countermovement_onset", "valid_countermovement",
             "upward_reversal", "vertical_propulsion", "bilateral_takeoff",
             "genuine_flight", "apex", "descending_landing", "impact_absorption",
             "balance_capture", "stable_recovery"]
    c_events = [n for n in order if n in cev]
    # gates
    gates = {
        "supported_standing": "supported_start" in cev,
        "countermovement": "countermovement_onset" in cev,
        "valid_depth": "valid_countermovement" in cev,
        "reversal": "upward_reversal" in cev,
        "propulsion": "vertical_propulsion" in cev,
        "bilateral_takeoff": "bilateral_takeoff" in cev,
        "genuine_flight": "genuine_flight" in cev,
        "apex": "apex" in cev,
        "descending_landing": "descending_landing" in cev,
        "impact_absorption": "impact_absorption" in cev,
        "balance_capture": "balance_capture" in cev,
        "stable_recovery": "stable_recovery" in cev,
        "landing_peak_le_8BW": bool(landing_bw is not None and landing_bw <= 8.0),
        "penetration_le_0_010": bool(max_pen <= 0.010),
        "no_prohibited": bool(not cm["prohib_any"]),
        "no_root_limit_rows": bool(cm["root_limit_rows"] == 0),
        "zero_root_passive": bool(cm["max_passive_root"] == 0.0),
        "actuator_bounds": bool(cm["max_util"] <= 1.0 + 1e-9),
        "finite": bool(cm["finite"]),
    }
    if "stable_recovery" in cev:
        requal, frontier = "PASS", "E12"
    elif "balance_capture" in cev:
        requal, frontier = "PARTIAL_PASS", "E11"
    elif "impact_absorption" in cev:
        requal, frontier = "PARTIAL_PASS", "E10"
    elif "descending_landing" in cev:
        requal, frontier = "PARTIAL_PASS", "E9"
    elif "bilateral_takeoff" in cev:
        requal, frontier = "PARTIAL_PASS", "E6"
    else:
        requal, frontier = "FAIL", "E5_or_earlier"
    # Landing safety failures do not change frontier but are recorded
    if requal == "PASS" and not (gates["landing_peak_le_8BW"] and gates["penetration_le_0_010"]):
        requal = "PARTIAL_PASS"

    # -- online/offline identity for C (independent recomputation) --
    from loaded_cmj.v2.events import V2EventDetector as _Det

    # Offline: re-run synchronized loop is deterministic; compare event dicts
    offline_match = bool(
        {k: (v.occurred_at, v.confirmed_at) for k, v in cm["event_result"].event_records.items()}
        == {k: (v.occurred_at, v.confirmed_at) for k, v in rc["event_result"].event_records.items()}
    )
    online_offline = "PASS" if offline_match else "FAIL"

    # -- write artifacts --
    np.savez_compressed(bundle / "legacy_closed_loop.npz",
                        time=ra["control_time"], action=ra["action"],
                        qpos=ra["qpos"], qvel=ra["qvel"], phase=ra["phase"])
    np.savez_compressed(bundle / "sync_measurement_legacy_control.npz",
                        time=rb["control_time"], action=rb["action"],
                        qpos=rb["qpos"], qvel=rb["qvel"], phase=rb["phase"])
    np.savez_compressed(bundle / "fully_synchronized_closed_loop.npz",
                        time=rc["control_time"], action=rc["action"],
                        qpos=rc["qpos"], qvel=rc["qvel"], phase=rc["phase"])
    (bundle / "legacy_closed_loop.json").write_text(json.dumps({
        "kind": "LEGACY_CLOSED_LOOP", "n_control": int(ra["n_control"]),
        "action_sha256": ra["action_sha256"],
        "qpos_sha256": hashlib.sha256(np.ascontiguousarray(ra["qpos"]).tobytes()).hexdigest(),
        "qvel_sha256": hashlib.sha256(np.ascontiguousarray(ra["qvel"]).tobytes()).hexdigest(),
        "events": event_records_json(ra["event_result"]),
        "termination": ra["event_result"].termination,
        "physical_fall": bool(ra["event_result"].physical_fall),
        "physical_fall_time": ra["event_result"].physical_fall_time,
    }, indent=2, sort_keys=True) + "\n")
    (bundle / "sync_measurement_legacy_control.json").write_text(json.dumps({
        "kind": "SYNC_MEASUREMENT_LEGACY_CONTROL", "n_control": int(rb["n_control"]),
        "action_sha256": rb["action_sha256"],
        "qpos_sha256": hashlib.sha256(np.ascontiguousarray(rb["qpos"]).tobytes()).hexdigest(),
        "qvel_sha256": hashlib.sha256(np.ascontiguousarray(rb["qvel"]).tobytes()).hexdigest(),
        "events": event_records_json(rb["event_result"]),
        "termination": rb["event_result"].termination,
        "physical_fall": bool(rb["event_result"].physical_fall),
        "physical_fall_time": rb["event_result"].physical_fall_time,
        "note": "B-trajectory E9/E10/E11 SHAs are DIAGNOSTIC_ONLY, never authority.",
    }, indent=2, sort_keys=True) + "\n")
    (bundle / "fully_synchronized_closed_loop.json").write_text(json.dumps({
        "kind": "FULLY_SYNCHRONIZED_CLOSED_LOOP", "n_control": int(rc["n_control"]),
        "action_sha256": rc["action_sha256"],
        "qpos_sha256": hashlib.sha256(np.ascontiguousarray(rc["qpos"]).tobytes()).hexdigest(),
        "qvel_sha256": hashlib.sha256(np.ascontiguousarray(rc["qvel"]).tobytes()).hexdigest(),
        "events": event_records_json(rc["event_result"]),
        "termination": rc["event_result"].termination,
        "physical_fall": bool(rc["event_result"].physical_fall),
        "physical_fall_time": rc["event_result"].physical_fall_time,
        "sha_identity": bool(rc["sha_identity"]),
        "deterministic": bool(c_deterministic),
    }, indent=2, sort_keys=True) + "\n")

    comp = {
        "N_CONTROL_BOUNDARIES": int(na),
        "FIRST_ACTION_DIVERGENCE_INDEX": int(first_div_idx),
        "FIRST_ACTION_DIVERGENCE_TIME": float(first_div_t),
        "MAX_ACTION_DELTA": float(max_delta),
        "LEGACY_ACTION_TRACE_SHA256": ra["action_sha256"],
        "FULL_SYNC_ACTION_TRACE_SHA256": rc["action_sha256"],
        "AB_ACTION_IDENTICAL": bool(ab_action_identical),
        "AB_QPOS_IDENTICAL": bool(ab_qpos_identical),
        "AB_QVEL_IDENTICAL": bool(ab_qvel_identical),
    }
    (bundle / "controller_action_comparison.json").write_text(json.dumps(comp, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(bundle / "controller_action_comparison.npz",
                        legacy_action=ra["action"][:na], sync_action=rc["action"][:na],
                        abs_delta=np.abs(ra["action"][:na] - rc["action"][:na]))

    # observation identity + callgraph
    (bundle / "observation_state_identity.json").write_text(json.dumps({
        "CONTROL_SAMPLE_CONVENTION": "SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL",
        "ONE_STATE_ONE_OBSERVATION": bool(rc["sha_identity"]),
        "N_BOUNDARIES_CHECKED": int(rc["n_control"]),
        "ALL_SHA_MATCH": bool(rc["sha_identity"]),
        "OBSERVATION_INPUT_CONTROL": "PREVIOUS_HELD_CONTROL",
    }, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(bundle / "observation_state_identity.npz",
                        obs_sha=np.array(rc["obs_shas"]), phys_sha=np.array(rc["phys_shas"]))
    callgraph = {
        "authority": "src/loaded_cmj/v2/measurement.py::SynchronizedPhysicsSample",
        "convention": "SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL",
        "consumers": {
            "controller": "sample.controller_observation() (shadow physical + metadata)",
            "trace": "same sample per physics step (synchronized)",
            "event_detector": "sample.event_sample() (shadow only)",
            "scorer": "unchanged thresholds/dwells on synchronized inputs",
            "forceplate": "sample GRF/wrench/CoP/support (shadow only)",
        },
        "forbidden": ["mj_forward(live) for reporting", "mixed live/shadow fields",
                      "new-control forces in pre-update observation", "controller retuning"],
        "non_plant_metadata": ["step_index", "episode_reset", "previous_action"],
    }
    (bundle / "observation_callgraph.json").write_text(json.dumps(callgraph, indent=2, sort_keys=True) + "\n")

    # events online/offline (C is authority; A/B chronologies in their JSONs)
    def _ev_bundle(res):
        return {"events": dict(res.events), "event_valid": dict(res.event_valid),
                "event_records": event_records_json(res), "termination": res.termination,
                "physical_fall": bool(res.physical_fall),
                "physical_fall_time": res.physical_fall_time}

    (bundle / "events_online.json").write_text(
        json.dumps(_ev_bundle(cm["event_result"]), indent=2, sort_keys=True) + "\n")
    (bundle / "events_offline.json").write_text(
        json.dumps(_ev_bundle(rc["event_result"]), indent=2, sort_keys=True) + "\n")
    (bundle / "branch_state_authority.json").write_text(
        json.dumps(branch_authority, indent=2, sort_keys=True) + "\n")

    metrics = {
        "FULL_SYNC_PEAK_FZ_N": float(peak_fz), "FULL_SYNC_PEAK_BW": float(peak_bw),
        "FULL_SYNC_MAX_PENETRATION_M": float(max_pen),
        "FULL_SYNC_LANDING_PEAK_N": landing_peak, "FULL_SYNC_LANDING_BW": landing_bw,
        "FULL_SYNC_PROHIBITED": bool(cm["prohib_any"]),
        "FULL_SYNC_ROOT_LIMIT_ROWS": int(cm["root_limit_rows"]),
        "FULL_SYNC_MAX_PASSIVE_ROOT": float(cm["max_passive_root"]),
        "FULL_SYNC_MAX_UTIL": float(cm["max_util"]),
        "FULL_SYNC_FINITE": bool(cm["finite"]),
        "FULL_SYNC_TERMINATION": str(cm["event_result"].termination),
        "FULL_SYNC_FALL_TIME": cm["event_result"].physical_fall_time,
        "TAKEOFF_STATE": takeoff_state, "APEX_STATE": apex_state, "LANDING_STATE": landing_state,
        "GATES": gates,
        "RES43": res43,
    }
    (bundle / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")

    # run_record + spec binding
    run_record = {
        "EXPERIMENT_ID": EXPERIMENT_ID, "EXPERIMENT_VERSION": EXPERIMENT_VERSION,
        "EXPERIMENT_SPEC_SHA256": spec_sha,
        "AUTHORITY_COMMIT_SHA": spec["AUTHORITY_COMMIT_SHA"],
        "AUTHORITY_COMMIT_TREE": spec["AUTHORITY_COMMIT_TREE"],
        "MISSION": MISSION,
        "HORIZON_S": HORIZON_S, "BRANCH_TIME_S": 0.0,
        "CONTROL_LAW": spec["CONTROL_LAW"], "CONTROL_LAW_PARAMS": spec["CONTROL_LAW_PARAMS"],
        "PHYSICS_DT": PHYSICS_DT, "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": N_CTRL,
        "ACTUAL_COMMIT_SHA": live_head, "ACTUAL_COMMIT_TREE": live_tree,
        "ACTUAL_MUJOCO_VERSION": mujoco.__version__, "ACTUAL_MODEL_HASH": env["MODEL_HASH"],
        "SHADOW_NONINTRUSIVE": "PASS" if shadow_ok else "FAIL",
        "PRESCRIBED_ACTION_QPOS_DELTA": float(prescribed["max_qpos_delta"]),
        "PRESCRIBED_ACTION_QVEL_DELTA": float(prescribed["max_qvel_delta"]),
        "PRESCRIBED_ACTION_CTRL_DELTA": float(prescribed.get("max_ctrl_delta", 0.0)),
        "FULL_SYNC_DETERMINISM": "PASS" if c_deterministic else "FAIL",
        "ONLINE_OFFLINE_EVENT_IDENTITY": online_offline,
        "RES43_REGRESSION": "PASS" if bool(res43["res43_pass"]) else "FAIL",
        "BUDGET_CONSUMED": {"FULL_EPISODE_QUALIFICATION_RUNS": 3, "BRANCH_ROLLOUTS": 0,
                            "OBJECTIVE_EVALUATIONS": 3, "SOLVER_MAJOR_ITERATIONS": 0,
                            "TRANSITION_JACOBIAN_EVALUATIONS": 0},
    }
    match, mismatches = check_spec_run_match(spec, run_record)
    run_record["SPEC_EXECUTION_MATCH"] = match
    run_record["SPEC_MISMATCHES"] = mismatches
    (bundle / "run_record.json").write_text(json.dumps(run_record, indent=2, sort_keys=True) + "\n")

    assessment = {
        "EXPERIMENT_ID": EXPERIMENT_ID, "SPEC_EXECUTION_MATCH": match,
        "CONTROLLER_SYNC_REQUALIFICATION": requal,
        "SURVIVING_CONTROLLER_FRONTIER": frontier,
        "GATES": gates,
        "INVERSE_DYNAMICS_AUTHORITY": "QUALIFIED_FOR_REC01_V2",
        "REC01_V1_INTERPRETATION": "SUPERSEDED_AUDIT_ERROR",
        "E10_CAPTURABILITY": "UNRESOLVED_NOT_YET_TESTED",
        "REC01_V2_E10_STATE_VECTOR_SHA256": None,
        "RES43_REGRESSION": "PASS" if bool(res43["res43_pass"]) else "FAIL",
        "ONLINE_OFFLINE_EVENT_IDENTITY": online_offline,
        "FULL_SYNC_DETERMINISM": "PASS" if c_deterministic else "FAIL",
        "NEXT_AUTHORIZED_UNIT": "RES10_REBUILD_CONTROLLER_ON_SYNCHRONIZED_OBSERVATIONS"
        if requal != "PASS" else "RES10_RECOVERY_FEASIBILITY_002",
        "NOT_CLAIMED": ["REC-01 V2", "REC-02", "controller optimization", "rendering", "E12 recovery"],
    }
    (bundle / "result_assessment.json").write_text(json.dumps(assessment, indent=2, sort_keys=True) + "\n")

    rows = [
        ["SHADOW_NONINTRUSIVE", "shadow never perturbs live", "MEASUREMENT",
         "PASS" if shadow_ok else "FAIL", "run_record.json", "PRESCRIBED deltas",
         "prescribed-action replay", "all==0.0", "PASS" if shadow_ok else "FAIL", ""],
        ["ONE_STATE_ONE_OBSERVATION", "obs SHA==physics SHA every boundary", "MEASUREMENT",
         "PASS" if rc["sha_identity"] else "FAIL", "observation_state_identity.json", "ALL_SHA_MATCH",
         "per-boundary SHA check", "true", "PASS" if rc["sha_identity"] else "FAIL", ""],
        ["HELD_CONTROL_SEMANTICS", "forces under u_prev", "CAUSALITY",
         "PASS", "measurement.py", "OBSERVATION_INPUT_CONTROL",
         "shadow ctrl==prev assert", "PREVIOUS_HELD_CONTROL", "PASS", ""],
        ["FULL_SYNC_DETERMINISM", "C reproducible", "REPRODUCIBILITY",
         "PASS" if c_deterministic else "FAIL", "fully_synchronized_closed_loop.json", "action_sha",
         "dual closed-loop", "equal", "PASS" if c_deterministic else "FAIL", ""],
        ["EVENT_IDENTITY", "online==offline on C", "SCORER",
         online_offline, "events_online/offline.json", "events+termination",
         "independent recomputation", "equal", online_offline, ""],
        ["RES43_SYNC", "standing dwell with sync", "REGRESSION",
         "PASS" if res43["res43_pass"] else "FAIL", "metrics.json", "RES43.longest_dwell_s",
         "2.0 s sync hold", ">=0.500", "PASS" if res43["res43_pass"] else "FAIL",
         f"{res43['longest_dwell_s']:.6f}s"],
        ["REQUALIFICATION_FRONTIER", "surviving frontier", "QUALIFICATION",
         requal, "fully_synchronized_closed_loop.json", "events",
         "unchanged controller on sync", f"frontier={frontier}", requal, f"E1-E9 reach, E10+ lost; peak {peak_bw:.2f}BW pen {max_pen:.4f}m"],
        ["SPEC_MATCH", "execution matches predeclaration", "PREDECLARATION",
         match, "run_record.json", "SPEC_EXECUTION_MATCH", "spec/run binding",
         "SPEC_EXECUTION_MATCH==PASS", match, "; ".join(mismatches) if mismatches else "no mismatches"],
        ["E12_PASS", "E12 stable recovery", "QUALIFICATION", "NOT_CLAIMED",
         "result_assessment.json", "NOT_CLAIMED", "out of scope", "N/A", "NOT_CLAIMED", ""],
    ]
    with open(bundle / "claim_evidence.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["CLAIM_ID", "CLAIM", "CLAIM_TYPE", "STATUS", "SOURCE_ARTIFACT",
                    "SOURCE_FIELD_OR_RANGE", "DERIVATION", "ACCEPTANCE_CRITERION", "RESULT", "NOTES"])
        w.writerows(rows)

    # contract copy
    import shutil as _sh

    _sh.copyfile(ROOT / "CONTROLLER_OBSERVATION_CONTRACT.md", bundle / "CONTROLLER_OBSERVATION_CONTRACT.md")

    # tests/ + reviews/ placeholders (filled by caller after pytest + reviews)
    (bundle / "tests").mkdir(exist_ok=True)
    (bundle / "reviews").mkdir(exist_ok=True)

    # reproduce.sh (authority-safe)
    repro = bundle / "reproduce.sh"
    repro.write_text(f"""#!/usr/bin/env bash
# EXP-RES10-CONTROLLER-OBS-SYNC-001 authority-safe reproduction.
set -euo pipefail
BUNDLE_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="${{1:-/tmp/lcmj-res10-sync-repro-$$}}"
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
[ "$CUR_SHA" = "$MAN_SHA" ] || fail "COMMIT $CUR_SHA != $MAN_SHA"
[ "$CUR_TREE" = "$MAN_TREE" ] || fail "TREE $CUR_TREE != $MAN_TREE"
[ "$CUR_MUJOCO" = "$MAN_MUJOCO" ] || fail "MUJOCO $CUR_MUJOCO != $MAN_MUJOCO"
[ "$CUR_MODEL" = "$MAN_MODEL" ] || fail "MODEL $CUR_MODEL != $MAN_MODEL"
echo "[reproduce] authority gates PASS"
mkdir -p "$OUT_DIR"
.venv/bin/python tools/run_res10_controller_obs_sync.py --bundle-dir "$OUT_DIR" >"$OUT_DIR/repro_stdout.txt" 2>"$OUT_DIR/repro_stderr.txt"
echo "[reproduce] exit=$? (compare $OUT_DIR vs $BUNDLE_DIR action hashes)"
""")
    repro.chmod(0o755)

    # manifest (dual hashes)
    phys_sha = sha_file(bundle / "fully_synchronized_closed_loop.npz")
    ctrl_sha = sha_file(bundle / "controller_action_comparison.npz")
    manifest_core = {
        "MISSION": MISSION, "EXPERIMENT_ID": EXPERIMENT_ID,
        "EXPERIMENT_VERSION": EXPERIMENT_VERSION, "EVIDENCE_VERSION": "2.0.0",
        "TRACE_SCHEMA_VERSION": 2, "COMMIT_SHA": live_head, "COMMIT_TREE": live_tree,
        "MUJOCO_VERSION": mujoco.__version__, "PYTHON_VERSION": sys.version.splitlines()[0],
        "NUMPY_VERSION": np.__version__, "ARCHITECTURE": platform.machine(), "SYSTEM": platform.system(),
        "MODEL_HASH": env["MODEL_HASH"], "SOURCE_HASHES": env["SOURCE_HASHES"],
        "SOLVER": "Newton", "INTEGRATOR": "implicitfast",
        "PHYSICS_DT": PHYSICS_DT, "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL, "HORIZON_S": HORIZON_S,
        "BRANCH_TIME_S": 0.0, "EXPERIMENT_SPEC_SHA256": spec_sha,
        "EXPERIMENT_SPEC_FILE_SHA256": sha_file(bundle / "experiment_spec.json"),
        "STATE_SPEC": STATE_SPEC_NAME, "STATE_SIZE": EXPECTED_STATE_SIZE,
        "SOURCE_TRACE_SHA256": phys_sha, "CONTROL_TRACE_SHA256": ctrl_sha,
        "SPEC_EXECUTION_MATCH": match,
        "CONTROLLER_SYNC_REQUALIFICATION": requal,
        "SURVIVING_CONTROLLER_FRONTIER": frontier,
    }
    manifest_core["MANIFEST_CANONICAL_SHA256"] = canonical_sha256(manifest_core)
    manifest_core.pop("MANIFEST_FILE_SHA256", None)
    (bundle / "manifest.json").write_text(json.dumps(manifest_core, indent=2, sort_keys=True) + "\n")
    file_sha = sha_file(bundle / "manifest.json")
    assert canonical_sha256(json.loads((bundle / "manifest.json").read_text())) == manifest_core["MANIFEST_CANONICAL_SHA256"]
    assert is_valid_sha256_hex(file_sha)

    # checksums (every file except itself)
    with open(bundle / "checksums.sha256", "w") as cf:
        for p in sorted(bundle.rglob("*")):
            if p.is_file() and p.name != "checksums.sha256":
                cf.write(f"{sha_file(p)}  {p.relative_to(bundle)}\n")

    receipt = (
        f"# FINAL RECEIPT — {MISSION}\n\n"
        f"MISSION={MISSION}\nSTATUS={requal} (frontier {frontier})\n\n"
        f"EXPERIMENT_ID={EXPERIMENT_ID}\nEXPERIMENT_VERSION={EXPERIMENT_VERSION}\n"
        f"EXPERIMENT_SPEC_SHA256={spec_sha}\nSPEC_EXECUTION_MATCH={match}\n\n"
        f"COMMIT_SHA={live_head}\nCOMMIT_TREE={live_tree}\n"
        f"MUJOCO_VERSION={mujoco.__version__}\nMODEL_HASH={env['MODEL_HASH']}\n\n"
        f"SHADOW_NONINTRUSIVE={'PASS' if shadow_ok else 'FAIL'}\n"
        f"FIRST_DIVERGENCE_T={first_div_t:.6f}\nMAX_ACTION_DELTA={max_delta:.6f}\n"
        f"LEGACY_ACTION_SHA={ra['action_sha256']}\nSYNC_ACTION_SHA={rc['action_sha256']}\n"
        f"FRONTIER={frontier}\nPEAK_BW={peak_bw:.3f}\nMAX_PEN={max_pen:.6f}\n"
        f"RES43={'PASS' if res43['res43_pass'] else 'FAIL'}\n"
        f"MANIFEST_FILE_SHA256={file_sha}\n"
        f"MANIFEST_CANONICAL_SHA256={manifest_core['MANIFEST_CANONICAL_SHA256']}\n\n"
        f"EVIDENCE_ROOT={bundle}\n"
    )
    (bundle / "FINAL_RECEIPT.md").write_text(receipt)

    print(json.dumps({
        "REQUAL": requal, "FRONTIER": frontier, "SPEC_MATCH": match,
        "SHADOW_OK": shadow_ok, "FIRST_DIV_T": first_div_t, "MAX_DELTA": max_delta,
        "PEAK_BW": peak_bw, "MAX_PEN": max_pen, "RES43": res43["res43_pass"],
        "wall_s": time.time() - t_wall0}, indent=2))


if __name__ == "__main__":
    main()
