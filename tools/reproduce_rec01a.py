#!/usr/bin/env python3
"""Fresh-process reproduction for EXP-RES10-REC01A-SYNC-DYNAMICS-001.

Authority-gated by reproduce.sh before invocation. Replays the canonical
prefix closed-loop from reset, verifies trace/event/E10 identity, and
spot-recomputes the E10 synchronized vs mixed-stage inverse errors.

Writes reproduction.json into --output-dir and prints the comparison.
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


def sha_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle-dir", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    bundle = args.bundle_dir.resolve()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    start_wall = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    t0 = time.time()
    command = f"python tools/reproduce_rec01a.py --bundle-dir {bundle} --output-dir {out}"

    manifest = json.loads((bundle / "manifest.json").read_text())
    run_record = json.loads((bundle / "run_record.json").read_text())

    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector
    from loaded_cmj.v2 import controller as ctrl

    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    det = V2EventDetector()
    det.reset()

    from tools.rec01a_sync_dynamics import (  # noqa: E402
        N_CTRL, N_PHYS, PHYSICS_DT, SUBSTEPS_PER_CONTROL,
    )

    fall_geoms = {plant.idx.geom[n] for n in
                  ["pelvis_fall", "torso_fall", "left_thigh_fall", "right_thigh_fall",
                   "left_shank_fall", "right_shank_fall"]}
    floor = plant.idx.floor_geom
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    DT = PHYSICS_DT
    t = 0.0
    prev = np.zeros(m.nu, dtype=np.float64)
    qpos_all = np.zeros((N_PHYS, m.nq))
    qvel_all = np.zeros((N_PHYS, m.nv))
    k = 0
    for step in range(N_CTRL):
        obs = plant.public_observation(d, scored_time_s=t, step_index=step,
                                       episode_reset=(step == 0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += DT
            qpos_all[k, :] = d.qpos
            qvel_all[k, :] = d.qvel
            com = plant.center_of_mass(d)
            cv = plant.center_of_mass_velocity(d)
            sm = plant.foot_contact_summary(d)
            fall = False
            for i in range(d.ncon):
                con = d.contact[i]
                g1i, g2i = int(con.geom1), int(con.geom2)
                if g1i == floor or g2i == floor:
                    other = g2i if g1i == floor else g1i
                    if other in fall_geoms:
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
            k += 1
    r = det.finalize()
    np.savez_compressed(out / "physics_trace.npz", time=np.array(
        [(i + 1) * DT for i in range(N_PHYS)]), qpos=qpos_all, qvel=qvel_all)
    (out / "events_replayed.json").write_text(json.dumps(
        {"events": dict(r.events), "termination": r.termination}, indent=2, sort_keys=True) + "\n")

    src_trace_sha = manifest["SOURCE_TRACE_SHA256"]
    # Compare against the bundle's full physics trace via qpos/qvel identity
    # (the bundle trace carries more arrays; identity is proven on qpos/qvel
    # plus event identity plus E10 state sha).
    orig = np.load(bundle / "physics_trace.npz")
    max_qpos = float(np.max(np.abs(orig["qpos"] - qpos_all)))
    max_qvel = float(np.max(np.abs(orig["qvel"] - qvel_all)))
    online = json.loads((bundle / "events_online.json").read_text())
    event_identity = bool(dict(r.events) == online["events"] and r.termination == online["termination"])
    identical = bool(max_qpos == 0.0 and max_qvel == 0.0 and event_identity)

    # E10 state sha + spot inverse errors
    e10_time = float(json.loads((bundle / "metrics.json").read_text())["E10_CONFIRMED"])
    idx_e10 = int(np.argmin(np.abs(orig["time"] - e10_time)))
    cap = np.load(bundle / "captured_states" / "E10_CONFIRMED.npz")
    e10_sha = hashlib.sha256(np.ascontiguousarray(cap["state_vector"]).tobytes()).hexdigest()
    e10_ok = bool(e10_sha == manifest["E10_STATE_VECTOR_SHA256"])

    d_s = plant.make_data()
    mujoco.mj_setState(m, d_s, np.ascontiguousarray(cap["state_vector"]), spec)
    mujoco.mj_forward(m, d_s)
    d_i = plant.make_data()
    d_i.qpos[:] = d_s.qpos[:]
    d_i.qvel[:] = d_s.qvel[:]
    d_i.qacc[:] = np.asarray(d_s.qacc).copy()
    d_i.ctrl[:] = d_s.ctrl[:]
    d_i.qfrc_applied[:] = d_s.qfrc_applied[:]
    d_i.xfrc_applied[:] = d_s.xfrc_applied[:]
    mujoco.mj_inverse(m, d_i)
    inv_s = np.asarray(d_i.qfrc_inverse)
    sync_f = float(np.max(np.abs(inv_s - (np.asarray(d_s.qfrc_actuator) + np.asarray(d_s.qfrc_applied)))))
    sync_r = float(np.linalg.norm(inv_s[0:3]))
    d_m = plant.make_data()
    mujoco.mj_setState(m, d_m, np.ascontiguousarray(cap["state_vector"]), spec)
    # stale post-step qacc at E10 sample from saved trace
    d_m.qacc[:] = np.ascontiguousarray(orig["qacc"][idx_e10])
    mujoco.mj_inverse(m, d_m)
    inv_o = np.asarray(d_m.qfrc_inverse)
    old_f = float(np.max(np.abs(inv_o - np.ascontiguousarray(orig["qfrc_actuator"][idx_e10]))))
    old_r = float(np.linalg.norm(inv_o[0:3]))
    sync_ok = bool(sync_f == run_record["SYNC_E10_FORCE"] and sync_r == run_record["SYNC_E10_ROOT"])
    old_ok = bool(old_f == run_record["OLD_E10_FORCE"] and old_r == run_record["OLD_E10_ROOT"])

    cur_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(ROOT)).decode().strip()
    cur_tree = subprocess.check_output(["git", "rev-parse", "HEAD^{tree}"], cwd=str(ROOT)).decode().strip()
    env_match = bool(cur_sha == manifest["COMMIT_SHA"] and cur_tree == manifest["COMMIT_TREE"]
                     and mujoco.__version__ == manifest["MUJOCO_VERSION"]
                     and sha_file(ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml") == manifest["MODEL_HASH"])
    commit_match = bool(cur_sha == manifest["COMMIT_SHA"] and cur_tree == manifest["COMMIT_TREE"])
    exit_code = 0 if (identical and e10_ok and sync_ok and old_ok and env_match) else 1
    end_wall = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    repro = {
        "COMMAND": command, "START_TIME": start_wall, "END_TIME": end_wall,
        "WALL_SECONDS": time.time() - t0, "EXIT_CODE": exit_code,
        "SOURCE_TRACE_SHA256": src_trace_sha,
        "REPLAY_TRACE_SHA256": sha_file(out / "physics_trace.npz"),
        "REPLAY_QPOS_QVEL_IDENTITY": identical,
        "IDENTICAL": identical,
        "MAX_QPOS_ERROR": max_qpos, "MAX_QVEL_ERROR": max_qvel,
        "ONLINE_OFFLINE_EVENT_IDENTITY": event_identity,
        "E10_STATE_SHA256_OK": e10_ok,
        "SYNC_E10_SPOT_OK": sync_ok, "SYNC_E10": [sync_f, sync_r],
        "OLD_E10_SPOT_OK": old_ok, "OLD_E10": [old_f, old_r],
        "ENVIRONMENT_MATCH": env_match, "COMMIT_MATCH": commit_match,
        "MUJOCO_VERSION_REPLAY": mujoco.__version__,
        "ARCHITECTURE_REPLAY": platform.machine(),
    }
    (out / "reproduction.json").write_text(json.dumps(repro, indent=2, sort_keys=True) + "\n")
    print(f"[replay] QPOS={max_qpos:.3e} QVEL={max_qvel:.3e} EVENT={event_identity} E10={e10_ok}")
    print(f"[replay] SYNC_SPOT={sync_ok} {sync_f:.3e}/{sync_r:.3e} OLD_SPOT={old_ok} {old_f:.6f}/{old_r:.6f}")
    print(f"[replay] ENV={env_match} COMMIT={commit_match} EXIT={exit_code}")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
