#!/usr/bin/env python3
"""Fresh-process branch replay (R0.1 §2C + §9).

Authority-gated by reproduce.sh before invocation. Restores the exact saved
mjSTATE_INTEGRATION vector with mj_setState, replays the recorded control
sequence verbatim, and writes sealed reproduction evidence:

  reproduction.json, reproduction_stdout.txt, reproduction_stderr.txt
  (stdout/stderr captures are finalized by reproduce.sh from this process's
  streams; this tool writes reproduction.json into --output-dir and prints
  the human-readable comparison to stdout.)

reproduction.json minimum fields: COMMAND, START_TIME, END_TIME, EXIT_CODE,
SOURCE_TRACE_SHA256, REPLAY_TRACE_SHA256, IDENTICAL, MAX_QPOS_ERROR,
MAX_QVEL_ERROR, ONLINE_OFFLINE_EVENT_IDENTITY, ENVIRONMENT_MATCH,
COMMIT_MATCH.
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

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.evid_state import restore_state_vector  # noqa: E402
from tools.evid_trace_v2 import TraceV2Collector  # noqa: E402


def sha_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle-dir", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    bundle = args.bundle_dir.resolve()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    start_wall = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    t_start = time.time()
    command = f"python tools/reproduce_branch_replay.py --bundle-dir {bundle} --output-dir {out}"

    manifest = json.loads((bundle / "manifest.json").read_text())
    env_rec = json.loads((bundle / "environment.json").read_text())

    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector

    plant = V2Plant()
    m = plant.model
    # restore branch state
    arr = np.load(bundle / "initial_integration_state.npz")
    vec = np.asarray(arr["state_vector"], dtype=np.float64).reshape(-1)
    d = plant.make_data()
    restore_state_vector(m, d, vec)
    branch_time = float(arr["time"][0])
    # recorded control sequence
    seq = np.load(bundle / "branch_control_sequence.npz")
    actions = np.asarray(seq["action"], dtype=np.float64)
    n_cont = int(actions.shape[0])

    col = TraceV2Collector(plant)
    col._data = d
    det = V2EventDetector()
    det.reset()
    t = branch_time
    HARMLESS_PHASE = 100
    from tools.evid_trace_v2 import SUBSTEPS_PER_CONTROL

    for si in range(n_cont):
        u = actions[si]
        col.push_control(t, u, HARMLESS_PHASE, "HARMLESS_HOLD",
                         {"law": "harmless_pd_hold_qstand0", "Kp": 400.0, "Kd": 10.0})
        plant.apply_action(d, u)
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += 0.000125
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
    P, C = col.finalize()
    r = det.finalize()
    np.savez_compressed(out / "physics_trace.npz", **P)
    np.savez_compressed(out / "control_trace.npz",
                        time=np.asarray(C["time"]), action=np.asarray(C["action"]),
                        phase=np.asarray(C["phase"]))
    (out / "events_replayed.json").write_text(json.dumps(
        {"events": dict(r.events), "termination": r.termination,
         "physical_fall": bool(r.physical_fall)}, indent=2, sort_keys=True) + "\n")

    # comparisons
    src_trace_sha = manifest["SOURCE_TRACE_SHA256"]
    replay_trace_sha = sha_file(out / "physics_trace.npz")
    orig = np.load(bundle / "physics_trace.npz")
    max_qpos = float(np.max(np.abs(orig["qpos"] - P["qpos"]))) if orig["qpos"].size else 0.0
    max_qvel = float(np.max(np.abs(orig["qvel"] - P["qvel"]))) if orig["qvel"].size else 0.0
    orig_ctrl = np.load(bundle / "control_trace.npz")
    max_ctrl = float(np.max(np.abs(orig_ctrl["action"] - np.asarray(C["action"])))) \
        if orig_ctrl["action"].size else 0.0
    online = json.loads((bundle / "events_online.json").read_text())
    identical = bool(replay_trace_sha == src_trace_sha)
    event_identity = bool(dict(r.events) == online["events"] and r.termination == online["termination"])
    # environment / commit match
    cur_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(ROOT)).decode().strip()
    cur_tree = subprocess.check_output(["git", "rev-parse", "HEAD^{tree}"], cwd=str(ROOT)).decode().strip()
    env_match = bool(cur_sha == manifest["COMMIT_SHA"] and cur_tree == manifest["COMMIT_TREE"]
                     and mujoco.__version__ == manifest["MUJOCO_VERSION"]
                     and sha_file(ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml") == manifest["MODEL_HASH"])
    commit_match = bool(cur_sha == manifest["COMMIT_SHA"] and cur_tree == manifest["COMMIT_TREE"])
    exit_code = 0 if (identical and max_qpos == 0.0 and max_qvel == 0.0 and event_identity and env_match) else 1
    end_wall = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    repro = {
        "COMMAND": command,
        "START_TIME": start_wall,
        "END_TIME": end_wall,
        "WALL_SECONDS": time.time() - t_start,
        "EXIT_CODE": exit_code,
        "SOURCE_TRACE_SHA256": src_trace_sha,
        "REPLAY_TRACE_SHA256": replay_trace_sha,
        "IDENTICAL": identical,
        "MAX_QPOS_ERROR": max_qpos,
        "MAX_QVEL_ERROR": max_qvel,
        "MAX_CTRL_ERROR": max_ctrl,
        "ONLINE_OFFLINE_EVENT_IDENTITY": event_identity,
        "ENVIRONMENT_MATCH": env_match,
        "COMMIT_MATCH": commit_match,
        "MUJOCO_VERSION_REPLAY": mujoco.__version__,
        "ARCHITECTURE_REPLAY": platform.machine(),
        "BRANCH_TIME_S": branch_time,
        "CONTINUATION_CONTROL_STEPS": n_cont,
    }
    (out / "reproduction.json").write_text(json.dumps(repro, indent=2, sort_keys=True) + "\n")
    print(f"[replay] source  {src_trace_sha}")
    print(f"[replay] replay  {replay_trace_sha}")
    print(f"[replay] IDENTICAL={identical} MAX_QPOS_ERROR={max_qpos:.3e} MAX_QVEL_ERROR={max_qvel:.3e} "
          f"MAX_CTRL_ERROR={max_ctrl:.3e}")
    print(f"[replay] EVENT_IDENTITY={event_identity} ENVIRONMENT_MATCH={env_match} COMMIT_MATCH={commit_match}")
    print(f"[replay] EXIT_CODE={exit_code}")
    # also mirror into bundle when invoked directly (reproduce.sh copies anyway)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
