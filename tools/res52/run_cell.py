#!/usr/bin/env python3
"""RES-52: run one predeclared soft-contact qualification cell.

Exact physics-rate trace with synchronized soft-contact state; full local
model diagnostics at control rate. Advances LIVE only with exact mj_step.
Nominal baseline: RES-43 PD hold (Kp400 Kd10) toward the frozen cell-start
posture. COM vz target: predeclared profile integral from cell start.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import core52 as C
from core52 import (PHYSICS_DT, SUB, WEIGHT_N, MASS_KG, G, sha_arr, seed_detector,
                    make_plant, restore_state, get_state_vector,
                    soft_contact_state, bind_plant, foot_gap_half_height,
                    ROOT_JOINT_NAMES, JOINT_NAMES)
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
from soft_contact import SoftContactPolicy
from tools.evid_trace_v2 import centroidal_H_world

OUT = C.BUNDLE


def fz_des_n(t_rel: float, nodes_bw: list) -> float:
    """Piecewise-linear predeclared profile (nodes [(t_s, BW), ...])."""
    ts = [n[0] for n in nodes_bw]
    vs = [n[1] for n in nodes_bw]
    if t_rel <= ts[0]:
        v = vs[0]
    elif t_rel >= ts[-1]:
        v = vs[-1]
    else:
        i = next(j for j in range(len(ts) - 1) if ts[j] <= t_rel <= ts[j + 1])
        f = (t_rel - ts[i]) / (ts[i + 1] - ts[i])
        v = vs[i] + f * (vs[i + 1] - vs[i])
    return float(np.clip(v * WEIGHT_N, 0.0, 7.5 * WEIGHT_N))


def make_vz_des(vz_start: float, nodes_bw: list, horizon: float):
    """Profile-integral COM vz target: vz0 + integral of az_des dt."""
    n = int(round(horizon / PHYSICS_DT)) + 1
    tg = np.arange(n) * PHYSICS_DT
    az = np.array([(fz_des_n(tt, nodes_bw) / MASS_KG) - G for tt in tg])
    vz = vz_start + np.concatenate([[0.0], np.cumsum((az[1:] + az[:-1]) * 0.5) * PHYSICS_DT])
    return lambda t: float(np.interp(min(max(t, 0.0), horizon), tg, vz))


def run_cell(cell: dict, constants: dict, start_vec: np.ndarray,
             ref_events: dict) -> tuple:
    plant = make_plant()
    bind_plant(plant)
    m = plant.model
    d = plant.make_data()
    meas = create_measurement_data(plant)
    restore_state(m, d, start_vec)
    t0 = float(d.time)
    gap_hh = foot_gap_half_height(m)
    probes = [plant.make_data() for _ in range(17)]
    validation_probe = probes[16]
    policy = SoftContactPolicy(constants, plant, probes)
    policy.gap_hh = gap_hh
    det = seed_detector(ref_events)
    acc = C.TraceAcc()

    s0 = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
    u_prev = np.asarray(s0.ctrl, float).copy()
    assert np.all(np.abs(u_prev) <= 1.0 + 1e-9)
    vz0 = float(s0.com_velocity_mps[2])
    horizon = float(cell["HORIZON_S"])
    nodes = cell["FZ_NODES_BW"]
    vz_des_f = make_vz_des(vz0, nodes, horizon)
    n_ctrl = int(round(horizon / 0.005))
    root_rows = 0
    max_pass = 0.0
    prohib = False
    finite = True
    matrices = []
    infos = []
    stop = False
    fall_t = None

    for step in range(n_ctrl):
        if stop:
            break
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        assert s_ctrl.check_time_identity()
        cstate = soft_contact_state(m, meas, gap_half_height=gap_hh)
        assert sha_arr(get_state_vector(m, d)) == s_ctrl.state_vector_sha256
        t_rel = float(s_ctrl.time) - t0
        fz_des = fz_des_n(t_rel + 0.005, nodes)
        vz_des = vz_des_f(t_rel + 0.005)
        info = policy.act(get_state_vector(m, d), u_prev, fz_des, vz_des, SUB,
                          validation_probe)
        matrices.append(np.asarray(info["G"], float))
        infos.append(info)
        u = np.asarray(info["u"], float)
        assert np.all(np.isfinite(u)) and np.all(np.abs(u) <= 1.0 + 1e-9)
        plant.apply_action(d, u)
        u_prev = u.copy()
        acc.add_control(float(s_ctrl.time), 0, {
            "t_rel": t_rel, "FZ_DES": fz_des, "VZ_DES": vz_des, "u": u.copy(),
            "u_nom": info["u_nom"], "du": info["du"], "rho": info["rho"],
            "shrinks": info["shrinks"], "VALIDATED": info["VALIDATED"],
            "Y_NOM": info["Y_NOM"], "Y_VAL": info["Y_VAL"],
            "Y_PRED": info["Y_PRED"], "VAL_ERR": info["VAL_ERR"],
            "FLAGS_NOM": info["FLAGS_NOM"], "FLAGS_VAL": info["FLAGS_VAL"],
            "RHO_AFTER": info["RHO_AFTER"], "FALLBACK": info["FALLBACK"],
        })
        for _ in range(SUB):
            mujoco.mj_step(m, d)
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            cs = soft_contact_state(m, meas, gap_half_height=gap_hh)
            hy = float(centroidal_H_world(m, meas, plant.center_of_mass(meas))[1])
            acc.add_physics(s, cs, hy, 0)
            det.update(s.event_sample())
            for e in range(meas.nefc):
                if int(meas.efc_type[e]) == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
                    nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(meas.efc_id[e]))
                    if nm in ROOT_JOINT_NAMES:
                        root_rows += 1
            if bool(s.prohibited_contact):
                prohib = True
            if not (np.isfinite(s.qpos).all() and np.isfinite(s.qvel).all()
                    and np.isfinite(s.qacc).all()):
                finite = False
            max_pass = max(max_pass, float(np.max(np.abs(s.qfrc_passive[0:3]))))
            if det.physical_fall:
                fall_t = float(det.physical_fall_time)
                stop = True
                break

    arr = acc.arrays()
    n_phys = len(arr["t"])
    t_rel_arr = arr["t"] - t0
    FZw = arr["fzl"] + arr["fzr"]
    fz_des_arr = np.array([fz_des_n(tt, nodes) for tt in t_rel_arr])
    GATE = constants["GATES"]
    WALK_N = int(round(constants["WALK_WINDOW_S"] / PHYSICS_DT))
    # boundary (interval-end synchronized) samples: the regulated instants
    n_ctrl = len(infos)
    bnd_idx = np.array([SUB * (k + 1) - 1 for k in range(n_ctrl)
                        if SUB * (k + 1) - 1 < n_phys], dtype=int)
    t_bnd = t_rel_arr[bnd_idx]
    fz_bnd = FZw[bnd_idx]
    fzd_bnd = fz_des_arr[bnd_idx]
    err_bnd = fz_bnd - fzd_bnd
    late = t_bnd > constants["WALK_WINDOW_S"]
    # per-interval ripple (physics rate)
    ripples = np.array([float(np.ptp(FZw[k * SUB:(k + 1) * SUB]))
                        for k in range(n_ctrl) if (k + 1) * SUB <= n_phys])
    t_ripple = np.array([k * 0.005 for k in range(len(ripples))])
    ripples_late = ripples[t_ripple > constants["WALK_WINDOW_S"]]
    # per-foot contact activity and episodes
    actL = (arr["fzl"] > 10.0)
    actR = (arr["fzr"] > 10.0)
    below = (FZw < 10.0)
    trans = int(np.sum(np.diff(below.astype(np.int8)) != 0))
    refl_runs = _runs(below)
    reflight_episodes = [r for r in refl_runs if r >= GATE["REFLIGHT_MIN_DURATION_PHYSICS_STEPS"]]
    lossL = _runs(~actL)
    lossR = _runs(~actR)
    either = ~actL | ~actR
    starts_either = _run_starts(either)
    lens_either = _runs(either)
    post_walk_loss_episodes = int(sum(
        1 for st, ln in zip(starts_either, lens_either)
        if (arr["t"][st] - t0) > constants["WALK_WINDOW_S"]))
    out = {
        "CELL_ID": cell["CELL_ID"],
        "START_STATE_TIME": float(t0),
        "START_STATE_SHA256": sha_arr(start_vec),
        "HORIZON_S": horizon,
        "N_PHYS": int(n_phys), "N_CTRL": int(n_ctrl),
        "FZ_DES_MEAN_N": float(np.mean(fz_des_arr)),
        "FZ_MEAN_N": float(np.mean(FZw)),
        "FZ_TRACK_BOUNDARY_RMS_ERR_N": float(np.sqrt(np.mean(err_bnd ** 2))),
        "FZ_TRACK_BOUNDARY_RMS_ERR_BW": float(np.sqrt(np.mean(err_bnd ** 2)) / WEIGHT_N),
        "FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW": float(np.max(np.abs(err_bnd)) / WEIGHT_N),
        "FZ_TRACK_LATE_RMS_ERR_BW": float(np.sqrt(np.mean(err_bnd[late] ** 2)) / WEIGHT_N) if late.any() else 0.0,
        "FZ_TRACK_LATE_MAX_ABS_ERR_BW": float(np.max(np.abs(err_bnd[late])) / WEIGHT_N) if late.any() else 0.0,
        "FZ_RIPPLE_P2P_MAX_BW": float(np.max(ripples) / WEIGHT_N),
        "FZ_RIPPLE_LATE_MAX_BW": float(np.max(ripples_late) / WEIGHT_N) if len(ripples_late) else 0.0,
        "FZ_L_TRACK_RMS_ERR_N": float(np.sqrt(np.mean((arr["fzl"] - fz_des_arr / 2) ** 2))),
        "FZ_R_TRACK_RMS_ERR_N": float(np.sqrt(np.mean((arr["fzr"] - fz_des_arr / 2) ** 2))),
        "PEAK_FZ_N": float(np.max(FZw)), "PEAK_FZ_BW": float(np.max(FZw) / WEIGHT_N),
        "MAX_PENETRATION": float(max(np.max(arr["penL"]), np.max(arr["penR"]))),
        "MIN_WHOLE_FZ_N": float(np.min(FZw)),
        "CHATTER_TRANSITIONS": trans,
        "REFLIGHT_EPISODES": [int(r) for r in reflight_episodes],
        "CONTACT_INACTIVE_FRACTION_L": float(1.0 - np.mean(actL)),
        "CONTACT_INACTIVE_FRACTION_R": float(1.0 - np.mean(actR)),
        "MAX_INACTIVE_RUN_L": max(_runs(~actL), default=0),
        "MAX_INACTIVE_RUN_R": max(_runs(~actR), default=0),
        "POST_WALK_LOSS_EPISODES": int(post_walk_loss_episodes),
        "NVEL_MIN_L": float(np.min(arr["nvelL"])), "NVEL_MAX_L": float(np.max(arr["nvelL"])),
        "NVEL_MIN_R": float(np.min(arr["nvelR"])), "NVEL_MAX_R": float(np.max(arr["nvelR"])),
        "TRUST_SHRINK_EVENTS": int(policy.trust_shrink_events),
        "TRUST_FALLBACKS": int(policy.trust_fallbacks),
        "ACTIVE_SET_CROSSINGS_IN_PROBES": int(policy.active_set_crossings),
        "MAX_VALIDATION_ERROR": float(policy.max_validation_error),
        "ROOT_LIMIT_ROWS": int(root_rows),
        "ROOT_PASSIVE_MAX": float(max_pass),
        "PROHIBITED": bool(prohib), "FINITE": bool(finite),
        "PHYSICAL_FALL_TIME": fall_t,
        "MAX_ACTUATOR_UTIL": float(np.max(np.abs(np.stack(
            [np.asarray(i["u"], float) for i in infos])))),
        "END_STATE_SHA256": sha_arr(get_state_vector(m, d)),
        "END_TIME": float(d.time),
    }
    g = {}
    g["finite_honest"] = bool(out["FINITE"] and not out["PROHIBITED"]
                              and out["ROOT_LIMIT_ROWS"] == 0
                              and out["ROOT_PASSIVE_MAX"] == 0.0
                              and out["MAX_ACTUATOR_UTIL"] <= 1.0 + 1e-9)
    g["contact_retention"] = bool(
        out["CONTACT_INACTIVE_FRACTION_L"] <= GATE["CONTACT_INACTIVE_FRACTION_MAX"]
        and out["CONTACT_INACTIVE_FRACTION_R"] <= GATE["CONTACT_INACTIVE_FRACTION_MAX"]
        and out["MAX_INACTIVE_RUN_L"] <= GATE["CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS"]
        and out["MAX_INACTIVE_RUN_R"] <= GATE["CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS"]
        and not out["REFLIGHT_EPISODES"]
        and out["POST_WALK_LOSS_EPISODES"] <= GATE["POST_WALK_LOSS_EPISODES_MAX"])
    g["no_chatter"] = bool(out["CHATTER_TRANSITIONS"] <= GATE["CHATTER_TRANSITIONS_MAX"])
    g["force_safety"] = bool(out["PEAK_FZ_BW"] <= GATE["PEAK_FZ_MAX_BW"])
    g["pen_safety"] = bool(out["MAX_PENETRATION"] <= GATE["PEN_SAFETY_LIMIT_M"])
    g["tracking"] = bool(
        out["FZ_TRACK_BOUNDARY_RMS_ERR_BW"] <= GATE["FZ_TRACK_BOUNDARY_RMS_MAX_BW"]
        and out["FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW"] <= GATE["FZ_TRACK_BOUNDARY_MAX_ABS_BW"]
        and out["FZ_TRACK_LATE_RMS_ERR_BW"] <= GATE["FZ_TRACK_LATE_RMS_MAX_BW"]
        and out["FZ_TRACK_LATE_MAX_ABS_ERR_BW"] <= GATE["FZ_TRACK_LATE_MAX_ABS_BW"]
        and out["FZ_RIPPLE_LATE_MAX_BW"] <= GATE["LATE_RIPPLE_P2P_MAX_BW"])
    nvel_lo, nvel_hi = constants["QUALIFIED_NVEL_DOMAIN_MPS"]
    g["nvel_domain"] = bool(out["NVEL_MIN_L"] >= nvel_lo and out["NVEL_MAX_L"] <= nvel_hi
                            and out["NVEL_MIN_R"] >= nvel_lo and out["NVEL_MAX_R"] <= nvel_hi)
    g["trust_honesty"] = bool(out["TRUST_FALLBACKS"] <= GATE["TRUST_FALLBACKS_MAX"])
    g["no_fall"] = bool(out["PHYSICAL_FALL_TIME"] is None)
    out["GATES"] = g
    out["QUALIFIED"] = all(g.values())

    # ---- predeclared arrest-segment analysis ([0, ARREST_SEGMENT_S]) ----
    seg_s = float(cell.get("ARREST_SEGMENT_S", horizon))
    seg_mask = t_rel_arr <= seg_s + 1e-12
    seg_idx = np.where(seg_mask)[0]
    FZs = FZw[seg_idx]
    actLs, actRs = actL[seg_idx], actR[seg_idx]
    seg_bnd = t_bnd <= seg_s + 1e-12
    errs_seg = err_bnd[seg_bnd]
    rips_seg = ripples[t_ripple <= seg_s + 1e-12]
    either_s = (~actLs) | (~actRs)
    seg_loss = [st for st, ln in zip(_run_starts(either_s), _runs(either_s))]
    seg_loss_episodes = len(seg_loss)  # segment includes the walk window
    seg = {
        "ARREST_SEGMENT_S": seg_s,
        "PEAK_FZ_BW": float(np.max(FZs) / WEIGHT_N),
        "MAX_PENETRATION": float(max(np.max(arr["penL"][seg_idx]),
                                     np.max(arr["penR"][seg_idx]))),
        "BOUNDARY_RMS_ERR_BW": float(np.sqrt(np.mean(errs_seg ** 2)) / WEIGHT_N),
        "BOUNDARY_MAX_ABS_ERR_BW": float(np.max(np.abs(errs_seg)) / WEIGHT_N),
        "RIPPLE_P2P_MAX_BW": float(np.max(rips_seg) / WEIGHT_N) if len(rips_seg) else 0.0,
        "INACTIVE_FRACTION_L": float(1.0 - np.mean(actLs)),
        "INACTIVE_FRACTION_R": float(1.0 - np.mean(actRs)),
        "MAX_INACTIVE_RUN_L": max(_runs(~actLs), default=0),
        "MAX_INACTIVE_RUN_R": max(_runs(~actRs), default=0),
        "LOSS_EPISODES": int(seg_loss_episodes),
        "NVEL_MIN_L": float(np.min(arr["nvelL"][seg_idx])),
        "NVEL_MAX_L": float(np.max(arr["nvelL"][seg_idx])),
        "NVEL_MIN_R": float(np.min(arr["nvelR"][seg_idx])),
        "NVEL_MAX_R": float(np.max(arr["nvelR"][seg_idx])),
        "REFLIGHT_EPISODES": [int(r) for r in _runs((FZs < 10.0))
                              if r >= GATE["REFLIGHT_MIN_DURATION_PHYSICS_STEPS"]],
    }
    seg["SEGMENT_GATES"] = {
        "contact_retention": bool(
            seg["INACTIVE_FRACTION_L"] <= GATE["CONTACT_INACTIVE_FRACTION_MAX"]
            and seg["INACTIVE_FRACTION_R"] <= GATE["CONTACT_INACTIVE_FRACTION_MAX"]
            and seg["MAX_INACTIVE_RUN_L"] <= GATE["CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS"]
            and seg["MAX_INACTIVE_RUN_R"] <= GATE["CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS"]
            and not seg["REFLIGHT_EPISODES"]),
        "tracking": bool(
            seg["BOUNDARY_RMS_ERR_BW"] <= GATE["FZ_TRACK_BOUNDARY_RMS_MAX_BW"]
            and seg["BOUNDARY_MAX_ABS_ERR_BW"] <= GATE["FZ_TRACK_BOUNDARY_MAX_ABS_BW"]
            and seg["RIPPLE_P2P_MAX_BW"] <= GATE["LATE_RIPPLE_P2P_MAX_BW"]),
        "force_safety": bool(seg["PEAK_FZ_BW"] <= GATE["PEAK_FZ_MAX_BW"]),
        "pen_safety": bool(seg["MAX_PENETRATION"] <= GATE["PEN_SAFETY_LIMIT_M"]),
        "nvel_domain": bool(seg["NVEL_MIN_L"] >= nvel_lo and seg["NVEL_MAX_L"] <= nvel_hi
                            and seg["NVEL_MIN_R"] >= nvel_lo and seg["NVEL_MAX_R"] <= nvel_hi),
    }
    seg["SEGMENT_QUALIFIED"] = all(seg["SEGMENT_GATES"].values())
    out["ARREST_SEGMENT"] = seg
    return out, acc, arr, matrices, infos, policy


def _runs(mask: np.ndarray) -> list:
    """Lengths of maximal True runs in mask (empty if none)."""
    runs = []
    run = 0
    for b in mask:
        if b:
            run += 1
        elif run:
            runs.append(run)
            run = 0
    if run:
        runs.append(run)
    return runs


def _run_starts(mask: np.ndarray) -> list:
    """Start indices of maximal True runs in mask."""
    starts = []
    run = 0
    for i, b in enumerate(mask):
        if b:
            if run == 0:
                starts.append(i)
            run += 1
        else:
            run = 0
    return starts
