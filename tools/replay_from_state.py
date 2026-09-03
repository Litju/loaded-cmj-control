#!/usr/bin/env python3
"""Fresh-process replay from recorded initial state — part of evidence bundle.

Restores MjData from initial_integration_state.npz and re-runs identical
control sequence to verify bit-identical reproducibility.

Usage:
  .venv/bin/python tools/replay_from_state.py --bundle-dir <path> --output-dir <path>
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

def sha256_file(p: Path) -> str:
    import hashlib
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
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

    manifest = json.loads((bundle / "manifest.json").read_text())
    print(f"[replay] bundle {bundle} manifest {manifest['manifest_sha256']} commit {manifest['commit_sha']}")

    # Load initial state
    init = np.load(bundle / "initial_integration_state.npz")
    horizon = float(manifest["horizon_s"])
    DT = float(manifest["timestep"])
    SUB = int(manifest["substeps_per_control"])

    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector

    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    # Restore before any forward
    d.qpos[:] = init["qpos"]
    d.qvel[:] = init["qvel"]
    try:
        d.qacc[:] = init["qacc"]
    except Exception:
        pass
    d.ctrl[:] = init["ctrl"]
    d.time = float(init["time"][0])
    mujoco.mj_forward(m, d)

    import loaded_cmj.v2.controller as ctrl
    ctrl.reset(0.0)

    det = V2EventDetector()
    det.reset()

    physics_times = []
    physics_qpos = []
    physics_qvel = []
    physics_ctrl = []
    physics_whole = []
    physics_left = []
    physics_right = []
    physics_com_z = []
    physics_com_vz = []
    physics_trunk = []
    physics_margin = []
    physics_prohibited = []
    physics_fall = []

    t = float(d.time)
    steps = int(round(horizon / 0.005))
    prev_action = np.zeros(7, dtype=np.float64)

    for step_index in range(steps):
        obs = plant.public_observation(d, scored_time_s=t, step_index=step_index, episode_reset=(step_index == 0), previous_action=prev_action)
        act = np.asarray(ctrl.act(obs), dtype=np.float64).reshape(7)
        plant.apply_action(d, act)
        prev_action = act.copy()
        for _ in range(SUB):
            mujoco.mj_step(m, d)
            t += DT
            com = plant.center_of_mass(d)
            com_vel = plant.center_of_mass_velocity(d)
            summary = plant.foot_contact_summary(d)
            fall = False
            fall_geoms = {plant.idx.geom[n] for n in ["pelvis_fall", "torso_fall", "left_thigh_fall", "right_thigh_fall", "left_shank_fall", "right_shank_fall"]}
            for i in range(d.ncon):
                con = d.contact[i]
                g1 = int(con.geom1); g2 = int(con.geom2)
                if g1 == plant.idx.floor_geom or g2 == plant.idx.floor_geom:
                    other = g2 if g1 == plant.idx.floor_geom else g1
                    if other in fall_geoms:
                        fall = True
                        break
            sample = {
                "time_s": float(t),
                "com_z": float(com[2]),
                "com_vz": float(com_vel[2]),
                "com_x": float(com[0]),
                "whole_Fz": float(summary["whole_Fz"]),
                "left_Fz": float(summary["left_Fz"]),
                "right_Fz": float(summary["right_Fz"]),
                "trunk_tilt": float(plant.trunk_tilt(d)),
                "com_margin": float(summary["support_margin"]),
                "prohibited": bool(summary["prohibited_contact"]),
                "fall_contact": bool(fall),
            }
            det.update(sample)
            physics_times.append(float(t))
            physics_qpos.append(np.asarray(d.qpos.copy(), dtype=np.float64))
            physics_qvel.append(np.asarray(d.qvel.copy(), dtype=np.float64))
            physics_ctrl.append(np.asarray(d.ctrl.copy(), dtype=np.float64))
            physics_whole.append(float(summary["whole_Fz"]))
            physics_left.append(float(summary["left_Fz"]))
            physics_right.append(float(summary["right_Fz"]))
            physics_com_z.append(float(com[2]))
            physics_com_vz.append(float(com_vel[2]))
            physics_trunk.append(float(plant.trunk_tilt(d)))
            physics_margin.append(float(summary["support_margin"]))
            physics_prohibited.append(bool(summary["prohibited_contact"]))
            physics_fall.append(bool(fall))
        if det.physical_fall:
            break

    result = det.finalize()
    print(f"[replay] termination {result.termination} events {len(result.events)} steps {len(physics_times)}")

    # Write physics_trace.npz
    np.savez_compressed(
        out / "physics_trace.npz",
        time=np.asarray(physics_times, dtype=np.float64),
        qpos=np.stack(physics_qpos) if physics_qpos else np.zeros((0, 10)),
        qvel=np.stack(physics_qvel) if physics_qvel else np.zeros((0, 10)),
        ctrl=np.stack(physics_ctrl) if physics_ctrl else np.zeros((0, 7)),
        whole_Fz=np.asarray(physics_whole, dtype=np.float64),
        left_Fz=np.asarray(physics_left, dtype=np.float64),
        right_Fz=np.asarray(physics_right, dtype=np.float64),
        com_z=np.asarray(physics_com_z, dtype=np.float64),
        com_vz=np.asarray(physics_com_vz, dtype=np.float64),
        trunk_tilt=np.asarray(physics_trunk, dtype=np.float64),
        support_margin=np.asarray(physics_margin, dtype=np.float64),
        prohibited=np.asarray(physics_prohibited, dtype=np.bool_),
        fall=np.asarray(physics_fall, dtype=np.bool_),
    )
    # Also write events for comparison
    events = {
        "events": result.events,
        "event_valid": result.event_valid,
        "termination": result.termination,
        "physical_fall": bool(result.physical_fall),
        "physical_fall_time": float(result.physical_fall_time) if result.physical_fall_time is not None else None,
    }
    (out / "events_replayed.json").write_text(json.dumps(events, indent=2, sort_keys=True) + "\n")

    # Compare to original
    orig_sha = manifest["physics_trace_sha256"]
    new_sha = sha256_file(out / "physics_trace.npz")
    print(f"[replay] original sha256 {orig_sha}")
    print(f"[replay] replayed sha256 {new_sha}")
    if orig_sha == new_sha:
        print("[replay] IDENTICAL — PASS")
    else:
        # Also check numeric diff
        orig = np.load(bundle / "physics_trace.npz")
        new = np.load(out / "physics_trace.npz")
        max_qpos_err = float(np.max(np.abs(orig["qpos"] - new["qpos"])) ) if orig["qpos"].size else 0.0
        max_qvel_err = float(np.max(np.abs(orig["qvel"] - new["qvel"])) ) if orig["qvel"].size else 0.0
        print(f"[replay] max_qpos_err {max_qpos_err:.3e} max_qvel_err {max_qvel_err:.3e}")
        if max_qpos_err < 1e-12 and max_qvel_err < 1e-12:
            print("[replay] NUMERIC PASS (within 1e-12) despite sha mismatch (possible metadata diff)")
        else:
            print("[replay] FAIL")
            sys.exit(1)

if __name__ == "__main__":
    main()
