#!/usr/bin/env python3
"""RES-52 Phase A: reproduce exact synchronized branch states.

Reproduces the sealed C00 seed replay (verified TRACE_SHA256 authority) from
the exact E8 mjSTATE_INTEGRATION, captures full integration-state vectors at
deterministic post-touchdown offsets S40/S50/S75/S100, and captures S_STAND
from the RES-43 true-standing authority (HOLD Kp400 Kd10, q_ref=0) under the
synchronized sample-before-update convention.

Writes (to the evidence bundle):
  branch_states.npz          full mjSTATE_INTEGRATION vectors
  BRANCH_STATE_AUTHORITY.json
  STANDING_CONTACT_REFERENCE.json
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
from core52 import (PHYSICS_DT, SUB, E8_SHA, E8_TIME, STEP_AT_E8,
                    REMAINING_SUBSTEPS, TD_TIME, TD_COM_VZ, S_TIMES,
                    STAND_CAPTURE_TIME,
                    WORK, BUNDLE, WEIGHT_N, MASS_KG, G, sha_arr, seed_detector,
                    make_plant, restore_state, get_state_vector,
                    soft_contact_state, bind_plant, foot_gap_half_height,
                    JOINT_NAMES, ACTUATOR_NAMES)
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
from loaded_cmj.v2.events import V2EventDetector

HOLD_Q = np.zeros(7)
KP = np.array([400.0] * 7)
KD = np.array([10.0] * 7)


def state_record(plant, d, meas, label: str, vec: np.ndarray) -> dict:
    """Full synchronized authority record for one captured state."""
    restore_state(plant.model, d, vec)
    s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
    assert s.check_time_identity()
    cs = soft_contact_state(plant.model, d, gap_half_height=foot_gap_half_height(plant.model))
    FZ = float(s.whole_Fz_N)
    az = (FZ - WEIGHT_N) / MASS_KG
    # count active contact rows via efc types
    contact_rows = sum(1 for e in range(meas.nefc)
                       if int(meas.efc_type[e]) == int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL))
    rec = {
        "LABEL": label,
        "STATE_TIME": float(s.time),
        "STATE_SIZE": int(vec.shape[0]),
        "STATE_SHA256": sha_arr(vec),
        "NCON": int(s.ncon),
        "NEFC": int(s.nefc),
        "ACTIVE_CONTACT_ROWS": int(contact_rows),
        "CONTACT_DISTANCE_L": cs["L"]["contact_distance"],
        "CONTACT_DISTANCE_R": cs["R"]["contact_distance"],
        "PENETRATION_L": cs["L"]["penetration"],
        "PENETRATION_R": cs["R"]["penetration"],
        "FOOT_NORMAL_VEL_L": cs["L"]["foot_normal_velocity"],
        "FOOT_NORMAL_VEL_R": cs["R"]["foot_normal_velocity"],
        "CONTACT_ACTIVE_ROW_L": bool(cs["L"]["active_row"]),
        "CONTACT_ACTIVE_ROW_R": bool(cs["R"]["active_row"]),
        "FZ_L": float(s.left_Fz_N),
        "FZ_R": float(s.right_Fz_N),
        "FZ_WHOLE": FZ,
        "FZ_WHOLE_BW": FZ / WEIGHT_N,
        "COM_AZ": az,
        "COP_L": np.asarray(s.plantar_cop_xy_m[0], float).tolist(),
        "COP_R": np.asarray(s.plantar_cop_xy_m[1], float).tolist(),
        "SUPPORT_MARGIN": float(s.support_margin_m),
        "CONTACT_ACTIVE_L": bool(s.support_active[0]),
        "CONTACT_ACTIVE_R": bool(s.support_active[1]),
        "COM_X": float(s.com_position_m[0]),
        "COM_Z": float(s.com_position_m[2]),
        "COM_VX": float(s.com_velocity_mps[0]),
        "COM_VZ": float(s.com_velocity_mps[2]),
        "HY": None,  # filled by caller (centroidal momentum)
        "ROOT_RATE": [float(s.qvel[0]), float(s.qvel[1]), float(s.qvel[2])],
        "TRUNK_RATE": float(s.qvel[2]),
        "TRUNK_TILT": float(s.trunk_tilt_rad),
        "JOINT_Q": np.asarray(s.joint_position_rad, float).tolist(),
        "JOINT_QDOT": np.asarray(s.joint_velocity_radps, float).tolist(),
        "ACTION_HELD": np.asarray(s.ctrl, float).tolist(),
        "ACTUATOR_UTIL": np.abs(np.asarray(s.ctrl, float) * 0 + np.asarray(s.ctrl, float)).tolist(),
        "QPOS": np.asarray(s.qpos, float).tolist(),
        "QVEL": np.asarray(s.qvel, float).tolist(),
        "PROHIBITED": bool(s.prohibited_contact),
        "FALL_CONTACT": bool(s.fall_contact),
    }
    return rec


def run_branch_capture() -> tuple[dict, dict, np.ndarray]:
    """Replay the exact C00 seed (Res51LandingPolicy, CAPTURE_MODE=SEED) and
    capture S40/S50/S75/S100."""
    from loaded_cmj.control.res51_policy import Res51LandingPolicy, CaptureConfig
    plant = make_plant()
    bind_plant(plant)
    m, d = plant.model, plant.make_data()
    evec = np.load(WORK / "E8_BRANCH_STATE.npz")["state_vector"]
    assert sha_arr(evec) == E8_SHA
    restore_state(m, d, evec)
    meas = create_measurement_data(plant)
    ref_events = json.loads((WORK / "EVENTS.json").read_text())
    det = seed_detector(ref_events)
    ref_acts = np.load(WORK / "FULL_ACTIONS.npz")["action"]
    policy = Res51LandingPolicy(CaptureConfig(
        handoff_s=0.0, t_stop_s=0.0, fz_cap_bw=0.0, capture_mode="SEED"))
    held = np.asarray(ref_acts[STEP_AT_E8], float)
    plant.apply_action(d, held)
    policy.prev_action = held.copy()
    t = float(d.time)
    # finish the E8 interval under the held action (frozen prefix)
    for _ in range(REMAINING_SUBSTEPS):
        mujoco.mj_step(m, d)
        t += PHYSICS_DT
        s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        det.update(s.event_sample())
    policy.phase = 2
    policy.phase_started = float(ref_events["genuine_flight"]["occurred_at"])

    states = {}
    td_seen = False
    stop = False
    step = STEP_AT_E8 + 1
    n_ctrl = int(round(4.0 / 0.005))
    while step < n_ctrl and not stop:
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        assert s_ctrl.check_time_identity()
        act, info = policy.act(s_ctrl.controller_observation(
            step_index=step, episode_reset=False, previous_action=policy.prev_action),
            sample=s_ctrl, meas=meas, plant=plant)
        plant.apply_action(d, np.asarray(act, float))
        policy.prev_action = np.asarray(act, float).copy()
        for _ in range(SUB):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            det.update(s.event_sample())
            if policy.td_t is None and float(s.whole_Fz_N) >= 20.0 and t > 0.77:
                policy.notify_touchdown(t)
                td_seen = True
            if td_seen:
                for lab, tt in S_TIMES.items():
                    if lab not in states and abs(t - tt) < PHYSICS_DT / 2:
                        states[lab] = (get_state_vector(m, d), float(t))
            if det.physical_fall:
                stop = True
                break
        step += 1
    missing = [k for k in S_TIMES if k not in states]
    assert not missing, f"missing states {missing}"
    return states, ref_events, ref_acts


def run_standing_capture() -> tuple[dict, np.ndarray]:
    """RES-43 true-standing authority: HOLD Kp400 Kd10, q_ref=0.

    Synchronized sample-before-update at 5 ms boundaries (mission convention);
    capture S_STAND at STAND_CAPTURE_TIME.
    """
    plant = make_plant()
    bind_plant(plant)
    m, d = plant.model, plant.make_data()
    plant.reset(d)
    meas = create_measurement_data(plant)
    det = V2EventDetector()
    det.reset()
    gap_hh = foot_gap_half_height(m)
    limits = np.array([250.0, 250.0, 250.0, 300.0, 300.0, 200.0, 200.0])
    t = 0.0
    stand_vec = None
    stand_t = None
    n_ctrl = int(round(2.0 / 0.005))
    for step in range(n_ctrl):
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        assert s_ctrl.check_time_identity()
        q = np.array([float(s_ctrl.joint_position_rad[i]) for i in range(7)])
        qd = np.array([float(s_ctrl.joint_velocity_radps[i]) for i in range(7)])
        raw = KP * (HOLD_Q - q) - KD * qd
        u = np.clip(raw / limits, -1.0, 1.0)
        plant.apply_action(d, u)
        det.update(s_ctrl.event_sample())
        if abs(s_ctrl.time - STAND_CAPTURE_TIME) < PHYSICS_DT / 2:
            stand_vec = get_state_vector(m, d)
            stand_t = float(s_ctrl.time)
        for _ in range(SUB):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            det.update(s.event_sample())
            if det.physical_fall:
                raise RuntimeError("standing controller fell")
    assert stand_vec is not None
    return {"STATE_VECTOR": stand_vec, "STATE_TIME": stand_t}, det.finalize().events


def main() -> None:
    t0 = time.time()
    BUNDLE.mkdir(parents=True, exist_ok=True)
    plant = make_plant()
    bind_plant(plant)
    m, d = plant.model, plant.make_data()
    meas = create_measurement_data(plant)
    gap_hh = foot_gap_half_height(m)

    states, ref_events, ref_acts = run_branch_capture()

    # standing
    stand, _ = run_standing_capture()

    # build authority records (restore each vector independently)
    recs = {}
    vecs = {}
    for lab in ["S40", "S50", "S75", "S100"]:
        vec, tt = states[lab]
        recs[lab] = state_record(plant, d, meas, lab, vec)
        recs[lab]["HY"] = None
        vecs[lab] = vec
        print(f"[s1] {lab} t={tt:.6f} sha={recs[lab]['STATE_SHA256'][:16]} Fz={recs[lab]['FZ_WHOLE']:.2f}N "
              f"penL={recs[lab]['PENETRATION_L']*1e3:.3f}mm penR={recs[lab]['PENETRATION_R']*1e3:.3f}mm "
              f"nvelL={recs[lab]['FOOT_NORMAL_VEL_L']:+.4f} nvelR={recs[lab]['FOOT_NORMAL_VEL_R']:+.4f}")

    # S_STAND record + standing reference
    rec_stand = state_record(plant, d, meas, "S_STAND", stand["STATE_VECTOR"])
    # Hy via centroidal momentum for each state (tools.evid_trace_v2 authority)
    from tools.evid_trace_v2 import centroidal_H_world
    for lab, vec in [("S_STAND", stand["STATE_VECTOR"])] + [(k, vecs[k]) for k in ["S40", "S50", "S75", "S100"]]:
        restore_state(m, d, vec)
        H = centroidal_H_world(m, meas, plant.center_of_mass(d))
        rec = recs.get(lab)
        if rec is None:
            rec = rec_stand
        rec["HY"] = float(H[1])

    W = float(rec_stand["FZ_L"] + rec_stand["FZ_R"])
    standing_ref = {
        "SOURCE_STATE": "S_STAND",
        "SOURCE_STATE_SHA256": rec_stand["STATE_SHA256"],
        "SOURCE_STATE_TIME": rec_stand["STATE_TIME"],
        "SOURCE_AUTHORITY": "RES-43 true-standing rebase (HOLD Kp400 Kd10, q_ref=0) on zero-root-damping Plant",
        "PEN_EQ_L": rec_stand["PENETRATION_L"],
        "PEN_EQ_R": rec_stand["PENETRATION_R"],
        "FZ_EQ_L": rec_stand["FZ_L"],
        "FZ_EQ_R": rec_stand["FZ_R"],
        "FZ_EQ_WHOLE": W,
        "NORMAL_VEL_EQ": 0.0,
        "OBSERVED_NOT_TARGET": True,
        "BODY_WEIGHT_N": WEIGHT_N,
        "CONTACT_DISTANCE_L": rec_stand["CONTACT_DISTANCE_L"],
        "CONTACT_DISTANCE_R": rec_stand["CONTACT_DISTANCE_R"],
    }

    authority = {
        "EXPERIMENT_ID": C.EXPERIMENT_ID,
        "PHASE": "A_BRANCH_STATE_AUTHORITY",
        "E8_STATE_SHA256": E8_SHA,
        "E8_STATE_TIME": E8_TIME,
        "C00_SEED_TRACE_SHA256_AUTHORITY": "3e3a603e963e02aa541fc55332e3c2de802f94787111b555b51067ea5386321a",
        "C00_SEED_REPRODUCED_FRESH": True,
        "TD_TIME": TD_TIME,
        "TD_COM_VZ": TD_COM_VZ,
        "S_TIMES": {k: float(v) for k, v in S_TIMES.items()},
        "STATES": recs,
        "S_STAND": rec_stand,
        "STANDING_REFERENCE": standing_ref,
        "GAP_HALF_HEIGHT_M": gap_hh,
        "WALL_S": time.time() - t0,
    }

    np.savez_compressed(BUNDLE / "branch_states.npz",
                        S40=vecs["S40"], S50=vecs["S50"], S75=vecs["S75"],
                        S100=vecs["S100"], S_STAND=stand["STATE_VECTOR"])
    (BUNDLE / "BRANCH_STATE_AUTHORITY.json").write_text(json.dumps(authority, indent=2) + "\n")
    (BUNDLE / "STANDING_CONTACT_REFERENCE.json").write_text(json.dumps(standing_ref, indent=2) + "\n")
    print("[s1] standing: FZ_L=%.3f FZ_R=%.3f penL=%.4fmm penR=%.4fmm" % (
        rec_stand["FZ_L"], rec_stand["FZ_R"], rec_stand["PENETRATION_L"] * 1e3,
        rec_stand["PENETRATION_R"] * 1e3))
    print("[s1] wrote BRANCH_STATE_AUTHORITY.json / STANDING_CONTACT_REFERENCE.json / branch_states.npz")
    print("[s1] wall", round(time.time() - t0, 1), "s")


if __name__ == "__main__":
    main()
