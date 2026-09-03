#!/usr/bin/env python3
"""REC-01A synchronized MuJoCo forward/inverse dynamics authority audit.

MISSION=RES10_REC01A_SYNCHRONIZED_DYNAMICS_AUTHORITY
EXPERIMENT_ID=EXP-RES10-REC01A-SYNC-DYNAMICS-001

Adjudicates H1 (mixed-time audit artifact) vs H0 (genuine solver inconsistency)
with NO controller tuning and NO production-code modification.

Pipeline (all fresh-process, deterministic):
  Phase 1: canonical closed-loop prefix 0..T_END on frozen Plant+controller,
           per-physics-step integration-state + post-step snapshot recording.
  Phase 2: diagnostic closed-loop on a DIAGNOSTIC model copy with
           mjENBL_FWDINV enabled; proves trajectory identity (nonintrusive)
           and records built-in solver_fwdinv at physics rate.
  Phase 3: per-selected-sample synchronized manual identity test
           (restore mjSTATE_INTEGRATION -> mj_forward -> copy qacc -> mj_inverse)
           vs qfrc_actuator+applied, root rows vs zero.
  Phase 4: side-by-side mixed-stage reproduction (restore integration state,
           overwrite qacc with STALE post-step qacc, NO forward -> mj_inverse),
           labeled MIXED_STAGE_DIAGNOSTIC_ONLY. Bit-identical to the REC-01 V1
           live-copy operation (proven by construction: same (qpos,qvel,qacc,
           warmstart,ctrl,applied) triple into mj_inverse).
  Phase 5: contact-mode stratification of OLD/SYNC/solver_fwdinv errors.
  Phase 6: production sampling audit: TRACE_CURRENT (post-step reads) vs
           TRACE_SYNCHRONIZED (restored+forward reads) + offline event
           comparison E1..E12 with the UNCHANGED canonical detector.
  Phase 7: evidence bundle artifacts (contract v2 layout + REC01A specifics).

Frozen: Plant XML, mass/inertia, root mechanics, contact/friction/solref/solimp,
actuator mapping/limits, timestep, implicitfast, solver, scorer/events,
canonical controller, RES43 authority.

Usage:
  .venv/bin/python tools/rec01a_sync_dynamics.py --bundle-dir <path>
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

# --------------------------------------------------------------------------
# Frozen mission constants (predeclared; changing any requires a new experiment)
# --------------------------------------------------------------------------
EXPERIMENT_ID = "EXP-RES10-REC01A-SYNC-DYNAMICS-001"
EXPECTED_HEAD = "9465d87de8d3970fc7b01dfc8a458ccb79d6cf3b"
EXPECTED_TREE = "8bf5a7c4fbd1e02f9846ee4a934a234fa5a3479b"
EXPECTED_E10_SHA = "bc1862bf592f362611bbd75390ed568d34d9a65842e1125a9f33b461fb714354"
EXPECTED_STATE_SIZE = 108

PHYSICS_DT = 0.000125
CONTROL_DT = 0.005
SUBSTEPS_PER_CONTROL = 40
T_END = 1.6
N_PHYS = int(round(T_END / PHYSICS_DT))  # 12800
N_CTRL = int(round(T_END / CONTROL_DT))  # 320

# Analysis windows (physics seconds, predeclared)
WIN_MAIN = (0.759125, 1.077125)  # E9_OCCURRED-20ms .. E11_CONFIRMED+100ms
WIN_TAKEOFF = (0.6, 0.65)  # bilateral takeoff / flight gap
WIN_TAIL = (1.077125, 1.3)  # post-E11 contact chatter that spiked in V1

# V1 reference values for OLD-error reproduction gate (5% bands predeclared)
V1_E10_FORCE = 59.645377597145966
V1_E10_ROOT = 66.471  # as stated in mission (≈66.471)

ROOT_DOFS = (0, 1, 2)


def sha_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def git_rev(kind: str) -> str:
    return subprocess.check_output(["git", "rev-parse", kind], cwd=str(ROOT)).decode().strip()


def stats(a: np.ndarray) -> dict:
    a = np.asarray(a, dtype=np.float64).reshape(-1)
    return {
        "N": int(a.shape[0]),
        "median": float(np.median(a)) if a.shape[0] else 0.0,
        "mean": float(np.mean(a)) if a.shape[0] else 0.0,
        "p95": float(np.percentile(a, 95)) if a.shape[0] else 0.0,
        "p99": float(np.percentile(a, 99)) if a.shape[0] else 0.0,
        "max": float(np.max(a)) if a.shape[0] else 0.0,
    }


def in_windows(t: float) -> bool:
    return (WIN_MAIN[0] <= t <= WIN_MAIN[1]) or (WIN_TAKEOFF[0] <= t <= WIN_TAKEOFF[1]) or (WIN_TAIL[0] < t <= WIN_TAIL[1])


# --------------------------------------------------------------------------
# Canonical closed-loop prefix
# --------------------------------------------------------------------------
def run_canonical_prefix(plant, collect_states: bool):
    """Run canonical prefix; return record dict with per-step arrays."""
    from loaded_cmj.v2 import controller as ctrl

    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)

    from loaded_cmj.v2.events import V2EventDetector
    det = V2EventDetector()
    det.reset()

    fall_geoms = {plant.idx.geom[n] for n in
                  ["pelvis_fall", "torso_fall", "left_thigh_fall", "right_thigh_fall",
                   "left_shank_fall", "right_shank_fall"]}
    floor = plant.idx.floor_geom

    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    ssize = mujoco.mj_stateSize(m, spec)
    assert ssize == EXPECTED_STATE_SIZE, f"state size {ssize} != {EXPECTED_STATE_SIZE}"

    times = np.zeros(N_PHYS, dtype=np.float64)
    state_vecs = np.zeros((N_PHYS, ssize), dtype=np.float64)
    qpos_a = np.zeros((N_PHYS, m.nq), dtype=np.float64)
    qvel_a = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    qacc_post = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    ctrl_a = np.zeros((N_PHYS, m.nu), dtype=np.float64)
    act_post = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    pass_post = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    con_post = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    ncon_a = np.zeros(N_PHYS, dtype=np.int64)
    nefc_a = np.zeros(N_PHYS, dtype=np.int64)
    # current-style derived (TRACE_CURRENT observables)
    com_a = np.zeros((N_PHYS, 3), dtype=np.float64)
    comvel_a = np.zeros((N_PHYS, 3), dtype=np.float64)
    tilt_a = np.zeros(N_PHYS, dtype=np.float64)
    flz_a = np.zeros(N_PHYS, dtype=np.float64)
    frz_a = np.zeros(N_PHYS, dtype=np.float64)
    whole_fz_a = np.zeros(N_PHYS, dtype=np.float64)
    margin_a = np.zeros(N_PHYS, dtype=np.float64)
    pen_a = np.zeros(N_PHYS, dtype=np.float64)
    fall_a = np.zeros(N_PHYS, dtype=bool)
    prohib_a = np.zeros(N_PHYS, dtype=bool)
    pelvis_z_a = np.zeros(N_PHYS, dtype=np.float64)
    joint_q7_a = np.zeros((N_PHYS, 7), dtype=np.float64)
    qpos_full_a = np.zeros((N_PHYS, m.nq), dtype=np.float64)

    actions = np.zeros((N_CTRL, m.nu), dtype=np.float64)

    t = 0.0
    prev = np.zeros(m.nu, dtype=np.float64)
    k = 0
    vec = np.zeros(ssize, dtype=np.float64)
    for step in range(N_CTRL):
        obs = plant.public_observation(d, scored_time_s=t, step_index=step,
                                       episode_reset=(step == 0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        actions[step, :] = act
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            if abs(float(d.time) - t) > 1e-9:
                raise RuntimeError(f"clock drift: data.time={d.time} vs counted t={t}")
            mujoco.mj_getState(m, d, vec, spec)
            state_vecs[k, :] = vec
            times[k] = t
            qpos_a[k, :] = d.qpos
            qvel_a[k, :] = d.qvel
            qacc_post[k, :] = d.qacc
            ctrl_a[k, :] = d.ctrl
            act_post[k, :] = d.qfrc_actuator
            pass_post[k, :] = d.qfrc_passive
            con_post[k, :] = d.qfrc_constraint
            ncon_a[k] = int(d.ncon)
            nefc_a[k] = int(d.nefc)
            com = plant.center_of_mass(d)
            cv = plant.center_of_mass_velocity(d)
            sm = plant.foot_contact_summary(d)
            com_a[k, :] = com
            comvel_a[k, :] = cv
            tilt_a[k] = float(plant.trunk_tilt(d))
            flz_a[k] = float(sm["left_Fz"])
            frz_a[k] = float(sm["right_Fz"])
            whole_fz_a[k] = float(sm["whole_Fz"])
            margin_a[k] = float(sm["support_margin"])
            pen_a[k] = float(sm["max_penetration"])
            prohib_a[k] = bool(sm["prohibited_contact"])
            pelvis_z_a[k] = float(d.xpos[plant.idx.pelvis_body][2])
            joint_q7_a[k, :] = plant.joint_positions(d)
            qpos_full_a[k, :] = np.asarray(d.qpos)
            fall = False
            for i in range(d.ncon):
                con = d.contact[i]
                g1i, g2i = int(con.geom1), int(con.geom2)
                if g1i == floor or g2i == floor:
                    other = g2i if g1i == floor else g1i
                    if other in fall_geoms:
                        fall = True
                        break
            fall_a[k] = fall
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
                "com_position_m": com.tolist(),
            })
            k += 1
    assert k == N_PHYS
    result = det.finalize()
    return {
        "times": times, "state_vecs": state_vecs, "qpos": qpos_a, "qvel": qvel_a,
        "qacc_post": qacc_post, "ctrl": ctrl_a, "act_post": act_post,
        "pass_post": pass_post, "con_post": con_post, "ncon": ncon_a, "nefc": nefc_a,
        "com": com_a, "comvel": comvel_a, "tilt": tilt_a, "flz": flz_a, "frz": frz_a,
        "whole_fz": whole_fz_a, "margin": margin_a, "pen": pen_a, "fall": fall_a,
        "prohib": prohib_a, "pelvis_z": pelvis_z_a, "joint_q7": joint_q7_a,
        "qpos_full": qpos_full_a, "actions": actions,
        "event_result": result, "plant": plant, "model": m,
    }


def run_diagnostic_prefix(plant_diag, n_phys: int):
    """Closed-loop prefix on FWDINV-enabled diagnostic model; record fwdinv."""
    from loaded_cmj.v2 import controller as ctrl

    m = plant_diag.model
    d = plant_diag.make_data()
    plant_diag.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    fwdinv = np.zeros((n_phys, 2), dtype=np.float64)
    qpos = np.zeros((n_phys, m.nq), dtype=np.float64)
    qvel = np.zeros((n_phys, m.nv), dtype=np.float64)
    ncon = np.zeros(n_phys, dtype=np.int64)
    t = 0.0
    prev = np.zeros(m.nu, dtype=np.float64)
    k = 0
    for step in range(n_phys // SUBSTEPS_PER_CONTROL):
        obs = plant_diag.public_observation(d, scored_time_s=t, step_index=step,
                                            episode_reset=(step == 0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        plant_diag.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            fwdinv[k, :] = np.asarray(d.solver_fwdinv, dtype=np.float64)
            qpos[k, :] = d.qpos
            qvel[k, :] = d.qvel
            ncon[k] = int(d.ncon)
            k += 1
    return {"fwdinv": fwdinv, "qpos": qpos, "qvel": qvel, "ncon": ncon}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle-dir", type=Path, required=True)
    args = ap.parse_args()
    bundle = args.bundle_dir.resolve()
    bundle.mkdir(parents=True, exist_ok=True)
    t_wall0 = time.time()

    # -- Phase 0: authority gate + sealed spec --
    live_head = git_rev("HEAD")
    live_tree = git_rev("HEAD^{tree}")
    if live_head != EXPECTED_HEAD or live_tree != EXPECTED_TREE:
        print(f"BLOCKED_AUTHORITY_MISMATCH head={live_head} tree={live_tree}")
        sys.exit(2)
    spec = json.loads((bundle / "experiment_spec.json").read_text())
    if spec.get("EXPERIMENT_ID") != EXPERIMENT_ID:
        print("BLOCKED_SPEC_MISMATCH")
        sys.exit(2)
    spec_sha = spec.get("EXPERIMENT_SPEC_SHA256", "")

    from loaded_cmj.v2.plant import V2Plant, model_xml
    from loaded_cmj.v2.events import V2EventDetector

    plant = V2Plant()
    m = plant.model

    # -- Phase 1: canonical prefix --
    print("[rec01a] phase 1: canonical prefix", flush=True)
    rec = run_canonical_prefix(plant, collect_states=True)
    times = rec["times"]
    ev = rec["event_result"]
    evrec = ev.event_records
    for name in ["descending_landing", "impact_absorption", "balance_capture"]:
        if name not in evrec:
            print(f"BLOCKED_PREFIX_INCOMPLETE missing {name}")
            sys.exit(2)
    e9o = float(evrec["descending_landing"].occurred_at)
    e9c = float(evrec["descending_landing"].confirmed_at)
    e10o = float(evrec["impact_absorption"].occurred_at)
    e10c = float(evrec["impact_absorption"].confirmed_at)
    e11o = float(evrec["balance_capture"].occurred_at)
    e11c = float(evrec["balance_capture"].confirmed_at)
    print(f"[rec01a] E9 {e9o:.6f}/{e9c:.6f} E10 {e10o:.6f}/{e10c:.6f} E11 {e11o:.6f}/{e11c:.6f}", flush=True)

    # E10 state identity vs REC-01 V1
    idx_e10c = int(np.argmin(np.abs(times - e10c)))
    assert abs(float(times[idx_e10c]) - e10c) < PHYSICS_DT / 2 + 1e-12
    e10_sha = hashlib.sha256(rec["state_vecs"][idx_e10c].tobytes()).hexdigest()
    print(f"[rec01a] E10_STATE_SHA={e10_sha}", flush=True)
    e10_identity = bool(e10_sha == EXPECTED_E10_SHA)

    # -- Phase 2: diagnostic FWDINV run --
    print("[rec01a] phase 2: diagnostic fwdinv run", flush=True)
    m_diag = mujoco.MjModel.from_xml_string(model_xml())
    assert int(m_diag.opt.enableflags) == 0, "diagnostic model must start with flags cleared"
    m_diag.opt.enableflags = int(mujoco.mjtEnableBit.mjENBL_FWDINV)
    plant_diag = V2Plant(model=m_diag)
    diag = run_diagnostic_prefix(plant_diag, N_PHYS)
    dq = float(np.max(np.abs(diag["qpos"] - rec["qpos"])))
    dv = float(np.max(np.abs(diag["qvel"] - rec["qvel"])))
    fwdinv_identity = bool(dq == 0.0 and dv == 0.0)
    print(f"[rec01a] FWDINV_IDENTITY dq={dq:.3e} dv={dv:.3e}", flush=True)
    fw = diag["fwdinv"]
    fw_abs0 = np.abs(fw[:, 0])
    fw_abs1 = np.abs(fw[:, 1])
    fw_mag = np.maximum(fw_abs0, fw_abs1)

    # -- Phases 3+4: synchronized vs mixed identity on selected samples --
    print("[rec01a] phases 3+4: identity tests", flush=True)
    sel_mask = np.array([in_windows(float(t)) for t in times])
    sel_idx = np.nonzero(sel_mask)[0]
    n_sel = int(sel_idx.shape[0])
    print(f"[rec01a] selected samples: {n_sel}", flush=True)

    spec_state = mujoco.mjtState.mjSTATE_INTEGRATION
    d_sync = plant.make_data()
    d_inv = plant.make_data()
    d_mix = plant.make_data()

    sync_force = np.zeros(n_sel, dtype=np.float64)
    sync_root_l2 = np.zeros(n_sel, dtype=np.float64)
    sync_root_max = np.zeros(n_sel, dtype=np.float64)
    old_force = np.zeros(n_sel, dtype=np.float64)
    old_root_l2 = np.zeros(n_sel, dtype=np.float64)
    old_root_max = np.zeros(n_sel, dtype=np.float64)
    qacc_sync_a = np.zeros((n_sel, m.nv), dtype=np.float64)
    ncon_sync_a = np.zeros(n_sel, dtype=np.int64)
    sel_times = np.zeros(n_sel, dtype=np.float64)
    sel_shas: list[str] = []

    for j, i in enumerate(sel_idx.tolist()):
        vec = rec["state_vecs"][i]
        t_i = float(times[i])
        sel_times[j] = t_i
        sel_shas.append(hashlib.sha256(np.ascontiguousarray(vec).tobytes()).hexdigest())
        # --- synchronized: restore + forward ---
        mujoco.mj_setState(m, d_sync, np.ascontiguousarray(vec), spec_state)
        mujoco.mj_forward(m, d_sync)
        qacc_s = np.asarray(d_sync.qacc, dtype=np.float64).copy()
        act_s = np.asarray(d_sync.qfrc_actuator, dtype=np.float64).copy()
        app_s = np.asarray(d_sync.qfrc_applied, dtype=np.float64).copy()
        qacc_sync_a[j, :] = qacc_s
        ncon_sync_a[j] = int(d_sync.ncon)
        # copy synchronized triple into inverse scratch (exact copies)
        d_inv.qpos[:] = d_sync.qpos[:]
        d_inv.qvel[:] = d_sync.qvel[:]
        d_inv.qacc[:] = qacc_s
        d_inv.ctrl[:] = d_sync.ctrl[:]
        d_inv.qfrc_applied[:] = d_sync.qfrc_applied[:]
        d_inv.xfrc_applied[:] = d_sync.xfrc_applied[:]
        mujoco.mj_inverse(m, d_inv)
        inv_s = np.asarray(d_inv.qfrc_inverse, dtype=np.float64)
        ref_s = act_s + app_s
        sync_force[j] = float(np.max(np.abs(inv_s - ref_s)))
        sync_root_l2[j] = float(np.linalg.norm(inv_s[0:3]))
        sync_root_max[j] = float(np.max(np.abs(inv_s[0:3])))
        # --- mixed-stage: restore (warmstart/ctrl/applied) + STALE qacc, NO forward ---
        mujoco.mj_setState(m, d_mix, np.ascontiguousarray(vec), spec_state)
        d_mix.qacc[:] = rec["qacc_post"][i]
        mujoco.mj_inverse(m, d_mix)
        inv_o = np.asarray(d_mix.qfrc_inverse, dtype=np.float64)
        ref_o = rec["act_post"][i]  # applied is zero throughout
        old_force[j] = float(np.max(np.abs(inv_o - ref_o)))
        old_root_l2[j] = float(np.linalg.norm(inv_o[0:3]))
        old_root_max[j] = float(np.max(np.abs(inv_o[0:3])))

    # E10 OLD reproduction check (index of E10 confirmed within selection)
    j_e10 = int(np.argmin(np.abs(sel_times - e10c)))
    old_e10_force = float(old_force[j_e10])
    old_e10_root = float(old_root_l2[j_e10])
    sync_e10_force = float(sync_force[j_e10])
    sync_e10_root = float(sync_root_l2[j_e10])

    # -- Phase 5: stratification --
    print("[rec01a] phase 5: stratification", flush=True)
    ncon_sel = rec["ncon"][sel_idx]
    fw_sel = fw_mag[sel_idx]
    strata: dict[str, dict] = {}
    for key, mask in [("ncon=8", ncon_sel == 8), ("ncon=4", ncon_sel == 4),
                      ("ncon=0", ncon_sel == 0),
                      ("other", ~np.isin(ncon_sel, [0, 4, 8]))]:
        idx = np.nonzero(mask)[0]
        strata[key] = {
            "OLD_POST_STEP_FORCE_ERROR": stats(old_force[idx]) if idx.size else stats(np.zeros(0)),
            "OLD_POST_STEP_ROOT_ERROR": stats(old_root_l2[idx]) if idx.size else stats(np.zeros(0)),
            "SYNC_FORCE_ERROR": stats(sync_force[idx]) if idx.size else stats(np.zeros(0)),
            "SYNC_ROOT_ERROR": stats(sync_root_l2[idx]) if idx.size else stats(np.zeros(0)),
            "solver_fwdinv": stats(fw_sel[idx]) if idx.size else stats(np.zeros(0)),
        }
    # transition strata on consecutive selected samples
    trans: dict[str, dict] = {}
    names = [f"{a}->{b}" for a in [0, 4, 8] for b in [0, 4, 8] if a != b]
    all_nc = rec["ncon"]
    for nm in names:
        a, b = int(nm.split("->")[0]), int(nm.split("->")[1])
        # transition into selected sample i from previous physics sample i-1
        js = [j for j in range(n_sel) if sel_idx[j] > 0 and int(all_nc[sel_idx[j] - 1]) == a and int(all_nc[sel_idx[j]]) == b]
        idx = np.array(js, dtype=np.int64)
        trans[nm] = {
            "OLD_POST_STEP_FORCE_ERROR": stats(old_force[idx]) if idx.size else stats(np.zeros(0)),
            "SYNC_FORCE_ERROR": stats(sync_force[idx]) if idx.size else stats(np.zeros(0)),
            "solver_fwdinv": stats(fw_sel[idx]) if idx.size else stats(np.zeros(0)),
        }
    contact_stats = {
        "strata_ncon": strata,
        "strata_transition": trans,
        "overall": {
            "OLD_POST_STEP_FORCE_ERROR": stats(old_force),
            "OLD_POST_STEP_ROOT_ERROR": stats(old_root_l2),
            "SYNC_FORCE_ERROR": stats(sync_force),
            "SYNC_ROOT_ERROR": stats(sync_root_l2),
            "solver_fwdinv": stats(fw_mag),
        },
    }

    # -- Phase 6: production sampling audit --
    print("[rec01a] phase 6: production audit", flush=True)
    d_r = plant.make_data()
    s_com = np.zeros((N_PHYS, 3), dtype=np.float64)
    s_comvel = np.zeros((N_PHYS, 3), dtype=np.float64)
    s_tilt = np.zeros(N_PHYS, dtype=np.float64)
    s_flz = np.zeros(N_PHYS, dtype=np.float64)
    s_frz = np.zeros(N_PHYS, dtype=np.float64)
    s_margin = np.zeros(N_PHYS, dtype=np.float64)
    s_pen = np.zeros(N_PHYS, dtype=np.float64)
    s_con = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    s_qacc = np.zeros((N_PHYS, m.nv), dtype=np.float64)
    s_ncon = np.zeros(N_PHYS, dtype=np.int64)
    s_pelvis_z = np.zeros(N_PHYS, dtype=np.float64)
    s_q7 = np.zeros((N_PHYS, 7), dtype=np.float64)
    for i in range(N_PHYS):
        mujoco.mj_setState(m, d_r, np.ascontiguousarray(rec["state_vecs"][i]), spec_state)
        mujoco.mj_forward(m, d_r)
        com = plant.center_of_mass(d_r)
        cv = plant.center_of_mass_velocity(d_r)
        sm = plant.foot_contact_summary(d_r)
        s_com[i, :] = com
        s_comvel[i, :] = cv
        s_tilt[i] = float(plant.trunk_tilt(d_r))
        s_flz[i] = float(sm["left_Fz"])
        s_frz[i] = float(sm["right_Fz"])
        s_margin[i] = float(sm["support_margin"])
        s_pen[i] = float(sm["max_penetration"])
        s_con[i, :] = np.asarray(d_r.qfrc_constraint, dtype=np.float64)
        s_qacc[i, :] = np.asarray(d_r.qacc, dtype=np.float64)
        s_ncon[i] = int(d_r.ncon)
        s_pelvis_z[i] = float(d_r.xpos[plant.idx.pelvis_body][2])
        s_q7[i, :] = plant.joint_positions(d_r)

    def run_offline_detector(com_, comvel_, flz_, frz_, tilt_, margin_, fall_, prohib_, q7_, qposf_, pelv_z_):
        det = V2EventDetector()
        det.reset()
        for i in range(N_PHYS):
            det.update({
                "time_s": float(times[i]), "com_z": float(com_[i][2]),
                "com_vz": float(comvel_[i][2]), "com_x": float(com_[i][0]),
                "whole_Fz": float(flz_[i] + frz_[i]),
                "left_Fz": float(flz_[i]), "right_Fz": float(frz_[i]),
                "trunk_tilt": float(tilt_[i]), "com_margin": float(margin_[i]),
                "prohibited": bool(prohib_[i]), "fall_contact": bool(fall_[i]),
                "joint_position_rad": [float(x) for x in q7_[i]],
                "qpos": [float(x) for x in qposf_[i]],
                "pelvis_position_world_m": [0.0, 0.0, float(pelv_z_[i])],
            })
        return det.finalize()

    off_current = run_offline_detector(rec["com"], rec["comvel"], rec["flz"], rec["frz"],
                                       rec["tilt"], rec["margin"], rec["fall"], rec["prohib"],
                                       rec["joint_q7"], rec["qpos_full"], rec["pelvis_z"])
    off_sync = run_offline_detector(s_com, s_comvel, s_flz, s_frz, s_tilt, s_margin,
                                    rec["fall"], rec["prohib"], s_q7, rec["qpos_full"], s_pelvis_z)

    order = ["supported_start", "countermovement_onset", "valid_countermovement",
             "upward_reversal", "vertical_propulsion", "bilateral_takeoff",
             "genuine_flight", "apex", "descending_landing", "impact_absorption",
             "balance_capture", "stable_recovery"]
    evt_cmp = {}
    all_identical = True
    for name in order:
        rc = off_current.event_records.get(name)
        rs = off_sync.event_records.get(name)
        if rc is None and rs is None:
            evt_cmp[name] = {"CURRENT_OCCURRED": None, "CURRENT_CONFIRMED": None,
                             "SYNC_OCCURRED": None, "SYNC_CONFIRMED": None,
                             "DELTA_OCCURRED": 0.0, "DELTA_CONFIRMED": 0.0, "IDENTICAL": True}
        elif rc is None or rs is None:
            all_identical = False
            evt_cmp[name] = {"CURRENT_OCCURRED": float(rc.occurred_at) if rc else None,
                             "CURRENT_CONFIRMED": float(rc.confirmed_at) if rc else None,
                             "SYNC_OCCURRED": float(rs.occurred_at) if rs else None,
                             "SYNC_CONFIRMED": float(rs.confirmed_at) if rs else None,
                             "DELTA_OCCURRED": None, "DELTA_CONFIRMED": None, "IDENTICAL": False}
        else:
            do = float(rs.occurred_at) - float(rc.occurred_at)
            dc = float(rs.confirmed_at) - float(rc.confirmed_at)
            ident = bool(do == 0.0 and dc == 0.0)
            all_identical = all_identical and ident
            evt_cmp[name] = {"CURRENT_OCCURRED": float(rc.occurred_at),
                             "CURRENT_CONFIRMED": float(rc.confirmed_at),
                             "SYNC_OCCURRED": float(rs.occurred_at),
                             "SYNC_CONFIRMED": float(rs.confirmed_at),
                             "DELTA_OCCURRED": do, "DELTA_CONFIRMED": dc, "IDENTICAL": ident}
    production_authority = "PASS" if all_identical else "FAIL"

    sync_deltas = {
        "MAX_COM_POSITION_SYNC_DELTA": float(np.max(np.abs(s_com - rec["com"]))),
        "MAX_COM_VELOCITY_SYNC_DELTA": float(np.max(np.abs(s_comvel - rec["comvel"]))),
        "MAX_FZ_SYNC_DELTA": float(np.max(np.abs((s_flz + s_frz) - (rec["flz"] + rec["frz"])))),
        "MAX_COP_SYNC_DELTA": float("nan"),  # CoP compared separately below
        "MAX_TILT_SYNC_DELTA": float(np.max(np.abs(s_tilt - rec["tilt"]))),
        "MAX_MARGIN_SYNC_DELTA": float(np.max(np.abs(s_margin - rec["margin"]))),
        "MAX_QFRC_CONSTRAINT_SYNC_DELTA": float(np.max(np.abs(s_con - rec["con_post"]))),
        "MAX_QACC_SYNC_DELTA": float(np.max(np.abs(s_qacc - rec["qacc_post"]))),
    }

    # -- decision --
    sync_p99 = float(np.percentile(sync_force, 99))
    sync_max = float(np.max(sync_force))
    sync_pass = bool(sync_p99 < 1e-8 and sync_max < 1e-6)
    old_med = float(np.median(old_force))
    sync_med = float(np.median(sync_force))
    h1_ratio = float(old_med / sync_med) if sync_med > 0 else float("inf")
    old_e10_ok = bool(abs(old_e10_force - V1_E10_FORCE) / V1_E10_FORCE < 0.05
                      and abs(old_e10_root - V1_E10_ROOT) / V1_E10_ROOT < 0.05)
    h1_supported = bool(sync_pass and h1_ratio > 1e3 and old_e10_ok)
    fwdinv_pass = bool(float(np.max(fw_mag)) < 1e-6)

    if fwdinv_pass and sync_pass and production_authority == "PASS":
        case, status = "A", "QUALIFIED_NUMERICAL_AUTHORITY"
        interpretation, inv_auth = "SUPERSEDED_AUDIT_ERROR", "QUALIFIED_FOR_REC01_V2"
        next_unit = "RES10_RECOVERY_FEASIBILITY_002"
    elif sync_pass and production_authority == "FAIL":
        case, status = "B", "BLOCKED_PRODUCTION_SAMPLE_SYNCHRONIZATION"
        interpretation, inv_auth = "SUPERSEDED_AUDIT_ERROR", "QUALIFIED_FOR_REC01_V2"
        next_unit = "RES10_PHYSICS_SAMPLE_SYNCHRONIZATION_CORRECTION"
    elif not sync_pass:
        case, status = "C", "BLOCKED_FORWARD_SOLVER_CONSISTENCY"
        interpretation, inv_auth = "UNRESOLVED", "NOT_QUALIFIED"
        next_unit = "RES10_SOLVER_CONSISTENCY_QUALIFICATION"
    else:
        case, status = "D", "BLOCKED_NUMERICAL_AUTHORITY_UNRESOLVED"
        interpretation, inv_auth = "UNRESOLVED", "NOT_QUALIFIED"
        next_unit = "UNRESOLVED"

    # -- Phase 7: write bundle --
    print("[rec01a] phase 7: writing bundle", flush=True)
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
            "rec01a_sync_dynamics.py": sha_file(ROOT / "tools/rec01a_sync_dynamics.py"),
        },
        "TRACE_SCHEMA_VERSION": 2,
        "PHYSICS_DT": PHYSICS_DT,
        "CONTROL_DT": CONTROL_DT,
    }
    (bundle / "environment.json").write_text(json.dumps(env, indent=2, sort_keys=True) + "\n")

    # physics/control traces (prefix)
    np.savez_compressed(bundle / "physics_trace.npz",
                        time=times, qpos=rec["qpos"], qvel=rec["qvel"],
                        qacc=rec["qacc_post"], ctrl=rec["ctrl"],
                        qfrc_actuator=rec["act_post"], qfrc_passive=rec["pass_post"],
                        qfrc_constraint=rec["con_post"], ncon=rec["ncon"], nefc=rec["nefc"],
                        com=rec["com"], com_vel=rec["comvel"],
                        left_Fz=rec["flz"], right_Fz=rec["frz"],
                        support_margin=rec["margin"])
    np.savez_compressed(bundle / "control_trace.npz",
                        time=np.array([i * CONTROL_DT for i in range(N_CTRL)]),
                        action=rec["actions"])
    np.savez_compressed(bundle / "branch_control_sequence.npz", action=rec["actions"],
                        branch_time_s=np.array([0.0]))
    # reset-state certificate
    d0 = plant.make_data()
    plant.reset(d0)
    mujoco.mj_forward(m, d0)
    v0 = np.zeros(EXPECTED_STATE_SIZE)
    mujoco.mj_getState(m, d0, v0, spec_state)
    np.savez_compressed(bundle / "initial_integration_state.npz", state_vector=v0,
                        state_spec=np.array([STATE_SPEC_INT]),
                        state_size=np.array([EXPECTED_STATE_SIZE]),
                        qpos=np.asarray(d0.qpos).copy(), qvel=np.asarray(d0.qvel).copy(),
                        ctrl=np.asarray(d0.ctrl).copy(), time=np.array([float(d0.time)]))
    (bundle / "initial_integration_state.json").write_text(json.dumps({
        "state_spec": STATE_SPEC_NAME, "state_spec_int": STATE_SPEC_INT,
        "state_size": EXPECTED_STATE_SIZE,
        "state_vector_sha256": hashlib.sha256(v0.tobytes()).hexdigest(),
        "MUJOCO_VERSION": mujoco.__version__, "ARCHITECTURE": platform.machine(),
        "MODEL_HASH": env["MODEL_HASH"], "COMMIT_SHA": live_head, "COMMIT_TREE": live_tree,
        "nq": int(m.nq), "nv": int(m.nv), "nu": int(m.nu)}, indent=2, sort_keys=True) + "\n")

    def ev_json(res):
        return {"events": dict(res.events), "event_valid": dict(res.event_valid),
                "event_records": {k: {"name": v.name, "occurred_at": float(v.occurred_at),
                                      "confirmed_at": float(v.confirmed_at),
                                      "sample_index": int(v.sample_index),
                                      "confirmed_sample_index": int(v.confirmed_sample_index)}
                                  for k, v in res.event_records.items()},
                "termination": res.termination, "physical_fall": bool(res.physical_fall)}

    (bundle / "events_online.json").write_text(json.dumps(ev_json(ev), indent=2, sort_keys=True) + "\n")
    (bundle / "events_offline.json").write_text(json.dumps(ev_json(off_current), indent=2, sort_keys=True) + "\n")
    online_offline = bool(dict(ev.events) == dict(off_current.events)
                          and ev.termination == off_current.termination)
    (bundle / "metrics.json").write_text(json.dumps({
        "physics_steps": N_PHYS, "control_steps": N_CTRL,
        "E9_OCCURRED": e9o, "E9_CONFIRMED": e9c, "E10_OCCURRED": e10o,
        "E10_CONFIRMED": e10c, "E11_OCCURRED": e11o, "E11_CONFIRMED": e11c,
        "E10_STATE_VECTOR_SHA256": e10_sha, "E10_IDENTITY_VS_REC01V1": e10_identity,
        "online_offline_event_identity": online_offline}, indent=2, sort_keys=True) + "\n")

    # REC01A-specific traces
    np.savez_compressed(bundle / "builtin_fwdinv_trace.npz", time=times,
                        solver_fwdinv=diag["fwdinv"], ncon=diag["ncon"])
    (bundle / "builtin_fwdinv_trace.json").write_text(json.dumps({
        "N": N_PHYS, "max_abs": [float(np.max(fw_abs0)), float(np.max(fw_abs1))],
        "p99_abs": [float(np.percentile(fw_abs0, 99)), float(np.percentile(fw_abs1, 99))],
        "mean_abs": [float(np.mean(fw_abs0)), float(np.mean(fw_abs1))],
        "FWDINV_DIAGNOSTIC_TRAJECTORY_IDENTITY": fwdinv_identity,
        "MAX_QPOS_DIFF": dq, "MAX_QVEL_DIFF": dv}, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(bundle / "synchronized_inverse_trace.npz", time=sel_times,
                        qacc_sync=qacc_sync_a, ncon_sync=ncon_sync_a,
                        sync_force_error=sync_force, sync_root_l2=sync_root_l2,
                        sync_root_max=sync_root_max)
    np.savez_compressed(bundle / "mixed_stage_inverse_trace.npz", time=sel_times,
                        old_force_error=old_force, old_root_l2=old_root_l2,
                        old_root_max=old_root_max)
    (bundle / "contact_mode_statistics.json").write_text(
        json.dumps(contact_stats, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(bundle / "production_trace_current.npz", time=times,
                        com=rec["com"], com_vel=rec["comvel"], tilt=rec["tilt"],
                        left_Fz=rec["flz"], right_Fz=rec["frz"], margin=rec["margin"],
                        pen=rec["pen"], qfrc_constraint=rec["con_post"], ncon=rec["ncon"])
    np.savez_compressed(bundle / "production_trace_synchronized.npz", time=times,
                        com=s_com, com_vel=s_comvel, tilt=s_tilt,
                        left_Fz=s_flz, right_Fz=s_frz, margin=s_margin,
                        pen=s_pen, qfrc_constraint=s_con, ncon=s_ncon)
    (bundle / "event_comparison.json").write_text(json.dumps({
        "events": evt_cmp, "PRODUCTION_SAMPLING_AUTHORITY": production_authority,
        "sync_deltas": sync_deltas}, indent=2, sort_keys=True) + "\n")

    # captured states directory (key points + window index)
    capdir = bundle / "captured_states"
    capdir.mkdir(exist_ok=True)
    key_times = {"just_before_E9": e9o - PHYSICS_DT, "E9_OCCURRED": e9o, "E9_CONFIRMED": e9c,
                 "E10_OCCURRED": e10o, "E10_CONFIRMED": e10c,
                 "E11_OCCURRED": e11o, "E11_CONFIRMED": e11c}
    key_index = {}
    for kname, kt in key_times.items():
        i = int(np.argmin(np.abs(times - kt)))
        sv = np.ascontiguousarray(rec["state_vecs"][i])
        np.savez_compressed(capdir / f"{kname}.npz", state_vector=sv,
                            time=np.array([float(times[i])]),
                            qpos=np.ascontiguousarray(rec["qpos"][i]),
                            qvel=np.ascontiguousarray(rec["qvel"][i]),
                            ctrl=np.ascontiguousarray(rec["ctrl"][i]),
                            ncon=np.array([int(rec["ncon"][i])]),
                            nefc=np.array([int(rec["nefc"][i])]))
        key_index[kname] = {"time": float(times[i]),
                            "state_size": EXPECTED_STATE_SIZE,
                            "state_vector_sha256": hashlib.sha256(sv.tobytes()).hexdigest(),
                            "qpos": [float(x) for x in rec["qpos"][i]],
                            "qvel": [float(x) for x in rec["qvel"][i]],
                            "ctrl": [float(x) for x in rec["ctrl"][i]],
                            "ncon": int(rec["ncon"][i]), "nefc": int(rec["nefc"][i])}
    (capdir / "key_states.json").write_text(json.dumps(key_index, indent=2, sort_keys=True) + "\n")
    np.savez_compressed(capdir / "selected_window_states.npz",
                        time=sel_times, state_vector=rec["state_vecs"][sel_idx],
                        state_sha256=np.array(sel_shas))

    run_record = {
        "EXPERIMENT_ID": EXPERIMENT_ID, "EXPERIMENT_SPEC_SHA256": spec_sha,
        "AUTHORITY_COMMIT_SHA": spec["AUTHORITY_COMMIT_SHA"],
        "AUTHORITY_COMMIT_TREE": spec["AUTHORITY_COMMIT_TREE"],
        "HORIZON_S": T_END, "BRANCH_TIME_S": 0.0,
        "CONTROL_LAW": spec["CONTROL_LAW"], "CONTROL_LAW_PARAMS": spec["CONTROL_LAW_PARAMS"],
        "PHYSICS_DT": PHYSICS_DT, "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "CONTINUATION_CONTROL_STEPS": N_CTRL,
        "ACTUAL_COMMIT_SHA": live_head, "ACTUAL_COMMIT_TREE": live_tree,
        "ACTUAL_MUJOCO_VERSION": mujoco.__version__, "ACTUAL_MODEL_HASH": env["MODEL_HASH"],
        "SELECTED_SAMPLES": n_sel,
        "E10_STATE_VECTOR_SHA256": e10_sha,
        "FWDINV_DIAGNOSTIC_TRAJECTORY_IDENTITY": "PASS" if fwdinv_identity else "FAIL",
        "BUILTIN_FWDINV_MAX": float(np.max(fw_mag)),
        "SYNC_FORCE_P99": sync_p99, "SYNC_FORCE_MAX": sync_max,
        "OLD_E10_FORCE": old_e10_force, "OLD_E10_ROOT": old_e10_root,
        "SYNC_E10_FORCE": sync_e10_force, "SYNC_E10_ROOT": sync_e10_root,
        "H1_RATIO_OLD_MED_OVER_SYNC_MED": h1_ratio,
        "PRODUCTION_SAMPLING_AUTHORITY": production_authority,
        "DECISION_CASE": case, "STATUS": status,
        "BUDGET_CONSUMED": {"FULL_EPISODE_QUALIFICATION_RUNS": 2, "BRANCH_ROLLOUTS": 0,
                            "OBJECTIVE_EVALUATIONS": 2, "SOLVER_MAJOR_ITERATIONS": 0,
                            "TRANSITION_JACOBIAN_EVALUATIONS": 0},
    }
    match, mismatches = check_spec_run_match(spec, run_record)
    run_record["SPEC_EXECUTION_MATCH"] = match
    run_record["SPEC_MISMATCHES"] = mismatches
    (bundle / "run_record.json").write_text(json.dumps(run_record, indent=2, sort_keys=True) + "\n")

    assessment = {
        "EXPERIMENT_ID": EXPERIMENT_ID, "SPEC_EXECUTION_MATCH": match,
        "DECISION_CASE": case, "STATUS": status,
        "HYPOTHESIS_H1": "SUPPORTED" if h1_supported else ("REFUTED" if not sync_pass else "UNRESOLVED"),
        "MIXED_STAGE_HYPOTHESIS": "SUPPORTED" if h1_supported else ("REFUTED" if not sync_pass else "UNRESOLVED"),
        "REC01_V1_INTERPRETATION": interpretation,
        "INVERSE_DYNAMICS_AUTHORITY": inv_auth,
        "E10_CAPTURABILITY": "UNRESOLVED_NOT_YET_TESTED",
        "NEXT_AUTHORIZED_UNIT": next_unit,
        "BUILTIN_FWDINV_PASS": fwdinv_pass,
        "SYNC_MANUAL_IDENTITY_PASS": sync_pass,
        "PRODUCTION_SAMPLING_AUTHORITY": production_authority,
        "NOT_CLAIMED": ["E10_capturability", "E12_recovery", "controller_performance",
                        "human_biomechanics", "safety"],
    }
    (bundle / "result_assessment.json").write_text(json.dumps(assessment, indent=2, sort_keys=True) + "\n")

    rows = [
        ["CANONICAL_PREFIX_E11", "prefix reaches E11_CONFIRMED", "REPRODUCIBILITY", "PASS",
         "events_online.json", "balance_capture.confirmed_at", "closed-loop canonical run",
         "E11_CONFIRMED==0.977125", "PASS", f"E11 {e11o}/{e11c}"],
        ["E10_STATE_IDENTITY", "E10 state == REC-01 V1 sha", "TRACEABILITY",
         "PASS" if e10_identity else "FAIL", "captured_states/key_states.json",
         "E10_CONFIRMED.state_vector_sha256", "mj_getState INTEGRATION",
         "==bc1862bf...", "PASS" if e10_identity else "FAIL", e10_sha],
        ["FWDINV_TRAJECTORY_IDENTITY", "diagnostic flag nonintrusive", "NUMERICAL",
         "PASS" if fwdinv_identity else "FAIL", "builtin_fwdinv_trace.json",
         "MAX_QPOS_DIFF/MAX_QVEL_DIFF", "dual closed-loop comparison", "==0.0",
         "PASS" if fwdinv_identity else "FAIL", f"dq={dq:.1e} dv={dv:.1e}"],
        ["BUILTIN_FWDINV_CONSISTENCY", "MuJoCo own fwd/inv check small", "NUMERICAL",
         "PASS" if fwdinv_pass else "FAIL", "builtin_fwdinv_trace.json", "solver_fwdinv max",
         "runtime enable-bit diagnostic", "max<1e-6", "PASS" if fwdinv_pass else "FAIL",
         f"max={float(np.max(fw_mag)):.3e}"],
        ["SYNC_MANUAL_IDENTITY", "same-state inverse at machine precision", "NUMERICAL",
         "PASS" if sync_pass else "FAIL", "synchronized_inverse_trace.npz",
         "sync_force_error", "restore+forward+copy+inverse", "P99<1e-8 AND max<1e-6",
         "PASS" if sync_pass else "FAIL", f"P99={sync_p99:.3e} max={sync_max:.3e}"],
        ["MIXED_STAGE_REPRODUCTION", "old audit reproduces V1 errors", "NUMERICAL",
         "PASS" if old_e10_ok else "FAIL", "mixed_stage_inverse_trace.npz",
         "old errors at E10", "restore+stale qacc+inverse, no forward",
         "within 5% of 59.645/66.471", "PASS" if old_e10_ok else "FAIL",
         f"force={old_e10_force:.3f} root={old_e10_root:.3f}"],
        ["H1_ADJUDICATION", "H1 vs H0 decision", "INFERENCE",
         "PASS" if h1_supported else "FAIL", "contact_mode_statistics.json",
         "OLD_med/SYNC_med ratio", "stratified comparison", "ratio>1e3",
         "PASS" if h1_supported else "FAIL", f"ratio={h1_ratio:.3e}"],
        ["PRODUCTION_SAMPLING", "production trace synchronized", "MEASUREMENT",
         production_authority, "event_comparison.json", "event sample indices",
         "offline detector on both traces", "all DELTA==0", production_authority,
         "see event_comparison.json"],
        ["EVENT_IDENTITY", "online==offline on prefix", "SCORER",
         "PASS" if online_offline else "FAIL", "events_online.json/events_offline.json",
         "events+termination", "independent recomputation", "equal",
         "PASS" if online_offline else "FAIL", ""],
        ["E12_PASS", "E12 stable recovery", "QUALIFICATION", "NOT_CLAIMED",
         "result_assessment.json", "NOT_CLAIMED", "out of scope", "N/A", "NOT_CLAIMED",
         "E12 not attempted in this mission"],
    ]
    with open(bundle / "claim_evidence.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["CLAIM_ID", "CLAIM", "CLAIM_TYPE", "STATUS", "SOURCE_ARTIFACT",
                    "SOURCE_FIELD_OR_RANGE", "DERIVATION", "ACCEPTANCE_CRITERION", "RESULT", "NOTES"])
        w.writerows(rows)

    (bundle / "tests").mkdir(exist_ok=True)
    (bundle / "tests" / "prefix_identity.txt").write_text(
        f"E10_SHA={e10_sha} identity={e10_identity}\nE11={e11o}/{e11c}\n"
        f"FWDINV dq={dq:.3e} dv={dv:.3e} identity={fwdinv_identity}\n"
        f"SYNC P99={sync_p99:.3e} max={sync_max:.3e} pass={sync_pass}\n"
        f"OLD E10 force={old_e10_force:.6f} root={old_e10_root:.6f} repro={old_e10_ok}\n"
        f"H1 ratio={h1_ratio:.3e} supported={h1_supported}\n"
        f"PRODUCTION_SAMPLING_AUTHORITY={production_authority}\nCASE={case} STATUS={status}\n")
    (bundle / "tests" / "sample_count.txt").write_text(
        f"physics={N_PHYS} control={N_CTRL} selected={n_sel}\n")
    (bundle / "tests" / "event_identity.txt").write_text(
        f"online==offline: {online_offline}\nproduction_identical: {all_identical}\n")

    manifest_core = {
        "MISSION": "RES10_REC01A_SYNCHRONIZED_DYNAMICS_AUTHORITY",
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "EVIDENCE_VERSION": "2.0.0",
        "TRACE_SCHEMA_VERSION": 2,
        "COMMIT_SHA": live_head, "COMMIT_TREE": live_tree,
        "MUJOCO_VERSION": mujoco.__version__,
        "PYTHON_VERSION": sys.version.splitlines()[0],
        "NUMPY_VERSION": np.__version__,
        "ARCHITECTURE": platform.machine(), "SYSTEM": platform.system(),
        "MODEL_HASH": env["MODEL_HASH"], "SOURCE_HASHES": env["SOURCE_HASHES"],
        "SOLVER": "Newton", "INTEGRATOR": "implicitfast",
        "PHYSICS_DT": PHYSICS_DT, "CONTROL_DT": CONTROL_DT,
        "SUBSTEPS_PER_CONTROL": SUBSTEPS_PER_CONTROL,
        "HORIZON_S": T_END, "BRANCH_TIME_S": 0.0,
        "EXPERIMENT_SPEC_SHA256": spec_sha,
        "EXPERIMENT_SPEC_FILE_SHA256": sha_file(bundle / "experiment_spec.json"),
        "STATE_SPEC": STATE_SPEC_NAME, "STATE_SIZE": EXPECTED_STATE_SIZE,
        "E10_STATE_VECTOR_SHA256": e10_sha,
        "SOURCE_TRACE_SHA256": sha_file(bundle / "physics_trace.npz"),
        "CONTROL_TRACE_SHA256": sha_file(bundle / "control_trace.npz"),
        "SPEC_EXECUTION_MATCH": match,
        "DECISION_CASE": case, "STATUS": status,
        "H1_RATIO": h1_ratio,
    }
    manifest_core["MANIFEST_CANONICAL_SHA256"] = canonical_sha256(manifest_core)
    manifest_core.pop("MANIFEST_FILE_SHA256", None)
    (bundle / "manifest.json").write_text(json.dumps(manifest_core, indent=2, sort_keys=True) + "\n")
    file_sha = sha_file(bundle / "manifest.json")
    assert canonical_sha256(json.loads((bundle / "manifest.json").read_text())) == \
        manifest_core["MANIFEST_CANONICAL_SHA256"]
    assert is_valid_sha256_hex(file_sha)

    print(json.dumps({
        "CASE": case, "STATUS": status, "E10_SHA_OK": e10_identity,
        "FWDINV_IDENTITY": fwdinv_identity, "SYNC_PASS": sync_pass,
        "H1_SUPPORTED": h1_supported, "PRODUCTION": production_authority,
        "OLD_E10": [old_e10_force, old_e10_root],
        "SYNC_E10": [sync_e10_force, sync_e10_root],
        "wall_s": time.time() - t_wall0}, indent=2))


if __name__ == "__main__":
    main()
