#!/usr/bin/env python3
"""V2.1 canonical qualification runtime (RES-78, production entrypoint).

MISSION=RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001
EXPERIMENT_ID=EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001
CANDIDATE_ID=V2.1-R001

Thin production execution layer over the already sealed V2.1 composition:
  PRELANDING Res72Policy (C01/RES58) -> BALANCE BalanceController (RES-73)
  -> RECOVERY StableRecoveryController (RES-74 incl RES43 HANDOFF).

Frozen authorities: Plant/contact/solver/timestep/integrator/root/actuators/
measurements/support/C01/RES58/RES73/RES74/RES43/phase guards/gains/recovery/
T_RISE/scorer predicates/dwells/action-history. No tuning, no search.

Must directly use V2Plant + exact committed composition + V2EventDetector
observationally. Must NOT import RES-76 research runner, RES-11 verifier,
or V1 Gen1 engine. No intermediate state restore, no qpos/qvel snap,
no scorer state for control. Runs through E12 + 0.30s posthold, emits
deterministic machine-readable results.

Canonical command:
  python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
if str(REPO / "src") not in sys.path:
    sys.path.insert(0, str(REPO / "src"))
if str(REPO / "tools") not in sys.path:
    sys.path.insert(0, str(REPO / "tools"))
if str(REPO / "tools" / "res52") not in sys.path:
    sys.path.insert(0, str(REPO / "tools" / "res52"))

import numpy as np
import mujoco

from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.measurement import create_measurement_data, SynchronizedPhysicsSample
from loaded_cmj.v2.events import V2EventDetector
from loaded_cmj.v2 import support_continuity as SC
from loaded_cmj.v2.res72_integration import Res72Policy
from loaded_cmj.v2.balance_capture import BalanceController
from loaded_cmj.v2.stable_recovery import StableRecoveryController
import core52 as C52
from soft_contact import SoftContactPolicy
from evid_trace_v2 import centroidal_H_world

MISSION = "RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001"
EXPERIMENT_ID = "EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001"
CANDIDATE_ID = "V2.1-R001"
CANONICAL_RUNTIME_MODULE = "loaded_cmj.v2.canonical_runtime"
CANONICAL_COMMAND = "python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001"

PHYS_DT = 0.000125
CTRL_DT = 0.005
NSUB = 40
MASS_W = 95.0 * 9.81
HORIZON = 20.0
POST_HOLD = 0.30

E10_NEED = 160
RR_HOLD_S = 0.10
E12_NEED = 4000

EVID74 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-STABLE-RECOVERY-E11-E12-001")
EVID_BAL = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-BALANCE-CAPTURE-E10-E11-001")
EVID52 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001")


def _load_authorities():
    rr_spec = json.loads((EVID_BAL / "recovery_ready_spec.json").read_text())
    manifold = json.loads((EVID74 / "RECOVERY_MANIFOLD_AUTHORITY.json").read_text())
    hspec = json.loads((EVID74 / "stand_handoff_ready_spec.json").read_text())
    tsc = json.loads((EVID74 / "RECOVERY_TIME_SCALING_AUTHORITY.json").read_text())
    spec52 = json.loads((EVID52 / "experiment_spec.json").read_text())
    return rr_spec, manifold, hspec, float(tsc["T_RISE"]), spec52["CONTROLLER_CONSTANTS"]


def _vec_sha(v: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.asarray(v, float)).tobytes()).hexdigest()


def _fetch_vec(m, d) -> np.ndarray:
    n = int(mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_INTEGRATION))
    vv = np.zeros(n, float)
    mujoco.mj_getState(m, d, vv, mujoco.mjtState.mjSTATE_INTEGRATION)
    return np.ascontiguousarray(vv)


def run_canonical_episode(out_dir: Path | None = None, tag: str = "V2.1-R001", verbose: bool = False) -> dict:
    """Execute one canonical episode from canonical reset through E12+posthold.

    Deterministic given pinned env. No intermediate restore, no scorer-for-control.
    """
    rr_spec, manifold, hspec, t_rise, k52 = _load_authorities()
    rr_th = rr_spec["THRESHOLDS"]
    rr_dwell = float(rr_spec["DWELL_S"])

    t_wall0 = time.time()
    plant = V2Plant()
    C52.bind_plant(plant)
    mm = plant.model
    live = plant.make_data()
    plant.reset(live)
    mujoco.mj_forward(mm, live)
    shadow = create_measurement_data(plant)
    clearance = C52.foot_gap_half_height(mm)

    probes_terminal = [plant.make_data() for _ in range(17)]
    vprobe_terminal = plant.make_data()
    inner_terminal = SoftContactPolicy(k52, plant, probes_terminal)
    inner_terminal.gap_hh = clearance
    prelanding = Res72Policy()
    prelanding.bind_terminal(inner_terminal, vprobe_terminal)

    probes_bal = [plant.make_data() for _ in range(15)]
    vprobe_bal = plant.make_data()
    probes_rec = [plant.make_data() for _ in range(15)]
    vprobe_rec = plant.make_data()
    balancer = None
    recoverer = None

    watcher = V2EventDetector()
    watcher.reset()

    regime = "PRELANDING"
    e10_streak = 0
    e10_armed = False
    e10_switch_t = None
    rr_since = None
    rr_entry_t = None
    rr_armed = False
    rr_conf_t = None

    phys_times: list[float] = []
    ctrl_times: list[float] = []
    ctrl_modes: list[str] = []
    ctrl_actions: list[np.ndarray] = []
    ctrl_substeps: list[int] = []
    event_samples_raw: list[dict] = []

    fzL_hist: list[float] = []
    fzR_hist: list[float] = []
    whole_hist: list[float] = []
    rowL_hist: list[bool] = []
    rowR_hist: list[bool] = []
    nvelL_hist: list[float] = []
    nvelR_hist: list[float] = []
    distL_hist: list[float] = []
    distR_hist: list[float] = []

    log_t: list[float] = []
    log_vx: list[float] = []
    log_vz: list[float] = []
    log_hy: list[float] = []
    log_fz: list[float] = []
    log_fzL: list[float] = []
    log_fzR: list[float] = []
    log_pen: list[float] = []
    log_margin: list[float] = []
    log_tilt: list[float] = []
    log_q7: list[list[float]] = []
    log_root: list[list[float]] = []
    log_pelvisz: list[float] = []
    log_comx: list[float] = []
    log_comz: list[float] = []
    log_prohib: list[bool] = []
    log_fall: list[bool] = []

    checkpoints: dict[str, np.ndarray] = {}
    checkpoint_t: dict[str, float] = {}
    seen_online: set[str] = set()

    past_vecs: deque = deque(maxlen=1200)
    trace_stream = hashlib.sha256()
    n_phys_total = 0

    peak_fz = 0.0
    max_pen = 0.0
    max_util = 0.0
    max_root_passive = 0.0
    root_rows = 0
    saw_fall = False
    saw_prohib = False
    finite_ok = True

    handoff_t = None
    handoff_vec = None
    e12_occ = None
    e12_conf = None
    e12_vec = None
    post_t0 = None
    post_bad = 0
    e12_pre_handoff = None
    e12_under_res43 = None

    indep_streak = 0
    indep_best = 0
    indep_e12_t = None
    guard_streak = 0
    guard_best = 0

    sim_t = 0.0
    held = np.zeros(7, float)
    n_ctrl = int(round(HORIZON / CTRL_DT))
    terminal = None

    def _log(msg: str):
        if verbose:
            print(msg, flush=True)

    for step_idx in range(n_ctrl):
        sync0 = SynchronizedPhysicsSample.from_live_state(plant, live, shadow)
        assert sync0.check_time_identity(), f"time identity fail step {step_idx}"
        obs0 = sync0.controller_observation(step_index=step_idx, episode_reset=(step_idx == 0), previous_action=held)
        assert np.array_equal(np.asarray(sync0.ctrl, float), np.asarray(held, float)), f"held mismatch {step_idx}"
        com0 = np.asarray(sync0.com_position_m, float)
        cv0 = np.asarray(sync0.com_velocity_mps, float)
        mom0 = centroidal_H_world(mm, shadow, com0)
        ang0 = np.zeros(6, float)
        mujoco.mj_objectVelocity(mm, shadow, mujoco.mjtObj.mjOBJ_BODY, int(plant.idx.torso_body), ang0, 0)
        trunk_rate0 = float(ang0[1])
        env0 = watcher._is_true_standing_neighborhood({
            "joint_position_rad": sync0.joint_position_rad,
            "com_z": float(sync0.com_position_m[2]), "com_position_m": sync0.com_position_m,
            "pelvis_position_world_m": sync0.pelvis_position_world_m,
            "trunk_tilt": float(sync0.trunk_tilt_rad),
            "left_Fz": float(sync0.left_Fz_N), "right_Fz": float(sync0.right_Fz_N),
            "plantar_normal_force_N": np.array([sync0.left_Fz_N, sync0.right_Fz_N]),
            "fall_contact": bool(sync0.fall_contact), "prohibited": bool(sync0.prohibited_contact),
            "prohibited_contact": bool(sync0.prohibited_contact)})
        state0 = _fetch_vec(mm, live)

        if regime == "PRELANDING" and e10_armed:
            balancer = BalanceController(plant, probes_bal, vprobe_bal, clearance)
            regime = "BALANCE"
            e10_switch_t = float(sync0.time)
            e10_armed = False
            _log(f"[{tag}] E10->BALANCE at t={e10_switch_t:.6f} step={step_idx}")
        if regime == "BALANCE" and rr_armed:
            recoverer = StableRecoveryController(plant, manifold, hspec, t_rise, probes_rec, vprobe_rec, clearance)
            regime = "RECOVERY"
            rr_armed = False
            held = np.zeros(7, float)
            _log(f"[{tag}] BALANCE->RECOVERY at t={float(sync0.time):.6f} step={step_idx} u_prev=zeros (action-history reset, no state reset)")

        if regime == "PRELANDING":
            u_cmd, cinfo = prelanding.act(obs0, sample=sync0, meas=shadow, plant=plant)
            u_cmd = np.asarray(u_cmd, float)
            phase_tag = f"PRL_{cinfo.get('phase_name', cinfo.get('phase'))}"
        elif regime == "BALANCE":
            assert balancer is not None
            u_cmd, binfo = balancer.step(state0, held, float(sync0.time), com0, cv0, mom0)
            u_cmd = np.asarray(u_cmd, float)
            phase_tag = "BALANCE"
        elif regime == "RECOVERY":
            assert recoverer is not None
            sm0 = plant.foot_contact_summary(shadow)
            u_cmd, rinfo = recoverer.step(state0, held, float(sync0.time), com0, cv0, mom0, sync0, sm0, trunk_rate0, bool(env0))
            u_cmd = np.asarray(u_cmd, float)
            if getattr(recoverer, "mode", None) == "HANDOFF" and handoff_t is None:
                handoff_t = float(sync0.time)
                handoff_vec = np.ascontiguousarray(state0).copy()
                _log(f"[{tag}] RECOVERY->RES43 HANDOFF at t={handoff_t:.6f} s={recoverer.s}")
            phase_tag = f"REC_{recoverer.mode}"
        else:
            raise RuntimeError(regime)

        max_util = max(max_util, float(np.max(np.abs(u_cmd))))
        plant.apply_action(live, u_cmd)
        held = u_cmd.copy()
        ctrl_times.append(float(sync0.time))
        ctrl_modes.append(regime + "|" + str(phase_tag))
        ctrl_actions.append(u_cmd.copy())
        executed = 0
        truncate = False

        for _k in range(NSUB):
            mujoco.mj_step(mm, live)
            sim_t += PHYS_DT
            executed += 1
            sync1 = SynchronizedPhysicsSample.from_live_state(plant, live, shadow)
            sm1 = plant.foot_contact_summary(shadow)
            com1 = np.asarray(sync1.com_position_m, float)
            cv1 = np.asarray(sync1.com_velocity_mps, float)
            mom1 = centroidal_H_world(mm, shadow, com1)
            ang1 = np.zeros(6, float)
            mujoco.mj_objectVelocity(mm, shadow, mujoco.mjtObj.mjOBJ_BODY, int(plant.idx.torso_body), ang1, 0)
            trunk_rate = float(ang1[1])
            if not (bool(np.isfinite(sync1.qpos).all()) and bool(np.isfinite(sync1.qvel).all())):
                finite_ok = False
            max_root_passive = max(max_root_passive, float(np.max(np.abs([live.qfrc_passive[plant.idx.vadr[nm]] for nm in ("root_tx", "root_tz", "root_ry")]))))
            peak_fz = max(peak_fz, float(sm1["whole_Fz"]))
            max_pen = max(max_pen, float(sync1.max_penetration_m))
            cs1 = C52.soft_contact_state(mm, shadow, gap_half_height=clearance)
            fzL_hist.append(float(sm1["left_Fz"])); fzR_hist.append(float(sm1["right_Fz"])); whole_hist.append(float(sm1["whole_Fz"]))
            rowL_hist.append(bool(cs1["L"]["active_row"])); rowR_hist.append(bool(cs1["R"]["active_row"]))
            nvelL_hist.append(float(cs1["L"]["foot_normal_velocity"])); nvelR_hist.append(float(cs1["R"]["foot_normal_velocity"]))
            distL_hist.append(float(cs1["L"]["contact_distance"])); distR_hist.append(float(cs1["R"]["contact_distance"]))
            phys_times.append(float(sim_t))
            log_t.append(float(sim_t))
            log_vx.append(float(cv1[0])); log_vz.append(float(cv1[2])); log_hy.append(float(mom1[1]))
            log_fz.append(float(sm1["whole_Fz"])); log_fzL.append(float(sm1["left_Fz"])); log_fzR.append(float(sm1["right_Fz"]))
            log_pen.append(float(sync1.max_penetration_m)); log_margin.append(float(sync1.support_margin_m))
            log_tilt.append(float(sync1.trunk_tilt_rad))
            q7_now = [float(x) for x in np.asarray(sync1.joint_position_rad, float)]
            log_q7.append(q7_now)
            log_root.append([float(sync1.qpos[0]), float(sync1.qpos[1]), float(sync1.qpos[2])])
            log_pelvisz.append(float(sync1.pelvis_position_world_m[2]))
            log_comx.append(float(com1[0])); log_comz.append(float(com1[2]))
            log_prohib.append(bool(sync1.prohibited_contact)); log_fall.append(bool(sync1.fall_contact))
            ev1 = sync1.event_sample()
            ev1 = dict(ev1)
            ev1["time_s"] = float(sim_t)
            event_samples_raw.append(ev1)

            ivec = _fetch_vec(mm, live)
            trace_stream.update(np.ascontiguousarray(ivec).tobytes())
            n_phys_total += 1

            cur_phase = int(prelanding.phase) if regime == "PRELANDING" else -1
            if regime == "PRELANDING" and prelanding.td_physics_t is None and sim_t > 0.6 and cur_phase in (6, 3, 7) and float(sync1.whole_Fz_N) >= 20.0:
                prelanding.notify_touchdown_physics(float(sim_t))

            past_vecs.append((float(sim_t), np.ascontiguousarray(ivec).copy()))

            if regime == "PRELANDING" and (not e10_armed) and e10_switch_t is None:
                if int(prelanding.phase) == 7:
                    e10_ok = bool(abs(float(cv1[2])) < 0.05 and float(sm1["left_Fz"]) > 10.0 and float(sm1["right_Fz"]) > 10.0 and not bool(sync1.fall_contact) and not bool(sync1.prohibited_contact))
                    if e10_ok:
                        e10_streak += 1
                        if e10_streak >= E10_NEED and (not e10_armed):
                            e10_armed = True
                            _log(f"[{tag}] E10 dwell 160 complete at t={sim_t:.6f} (truncate)")
                            checkpoints["S_E10_ctrl"] = np.ascontiguousarray(ivec).copy()
                            checkpoint_t["S_E10_ctrl_t"] = float(sim_t)
                            truncate = True
                    else:
                        e10_streak = 0
                else:
                    e10_streak = 0

            if regime == "BALANCE" and (not rr_armed):
                qd_max = float(np.max(np.abs(np.asarray(sync1.joint_velocity_radps, float))))
                rry = float(sync1.qvel[2])
                rr_ok = bool(abs(float(cv1[0])) <= float(rr_th["COM_VX_ABS_MAX"]) and abs(float(cv1[2])) <= float(rr_th["COM_VZ_ABS_MAX"]) and abs(float(mom1[1])) <= float(rr_th["HY_ABS_MAX"]) and abs(rry) <= float(rr_th["ROOT_PITCH_RATE_ABS_MAX"]) and abs(trunk_rate) <= float(rr_th["WORLD_TRUNK_TILT_RATE_ABS_MAX"]) and qd_max <= float(rr_th["JOINT_QDOT_ABS_MAX"]) and float(sync1.support_margin_m) >= float(rr_th["SUPPORT_MARGIN_MIN"]) and float(sm1["left_Fz"]) > float(rr_th["BILATERAL_FZ_MIN_N"]) and float(sm1["right_Fz"]) > float(rr_th["BILATERAL_FZ_MIN_N"]) and float(sync1.max_penetration_m) <= float(rr_th["PEN_MAX_M"]))
                if rr_ok:
                    if rr_since is None:
                        rr_since = float(sim_t)
                        rr_entry_t = float(sim_t)
                    if float(sim_t) - rr_since >= rr_dwell - 1e-9:
                        rr_armed = True
                        rr_conf_t = float(sim_t)
                        _log(f"[{tag}] RR dwell entry {rr_entry_t:.6f} -> conf {rr_conf_t:.6f} (truncate)")
                        checkpoints["S_RR_CONFIRMED_ctrl"] = np.ascontiguousarray(ivec).copy()
                        checkpoint_t["S_RR_CONFIRMED_ctrl_t"] = float(sim_t)
                        best = None
                        best_dt = 1e9
                        for (ht, hv) in past_vecs:
                            dd = abs(ht - float(rr_entry_t))
                            if dd < best_dt:
                                best_dt = dd
                                best = hv
                        if best is not None:
                            checkpoints["S_RR_ctrl"] = best.copy()
                            checkpoint_t["S_RR_ctrl_t"] = float(rr_entry_t)
                        truncate = True
                else:
                    rr_since = None

            watcher.update(dict(ev1))
            for nm in list(watcher.event_records.keys()):
                if nm not in seen_online:
                    seen_online.add(nm)
                    try:
                        checkpoints[f"S_DET_{nm}"] = np.ascontiguousarray(ivec).copy()
                        checkpoint_t[f"S_DET_{nm}_t"] = float(sim_t)
                        _log(f"[{tag}] DET {nm} conf t={sim_t:.6f} occ {watcher.event_records[nm].occurred_at:.6f}")
                    except Exception as exc:
                        _log(f"[{tag}] checkpoint save fail {nm}: {exc}")

            for ee in range(shadow.nefc):
                if int(shadow.efc_type[ee]) == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT):
                    nm2 = mujoco.mj_id2name(mm, mujoco.mjtObj.mjOBJ_JOINT, int(shadow.efc_id[ee]))
                    if nm2 in ("root_tx", "root_tz", "root_ry"):
                        root_rows += 1

            env_ok = watcher._is_true_standing_neighborhood({
                "joint_position_rad": sync1.joint_position_rad,
                "com_z": float(sync1.com_position_m[2]), "com_position_m": sync1.com_position_m,
                "pelvis_position_world_m": sync1.pelvis_position_world_m,
                "trunk_tilt": float(sync1.trunk_tilt_rad),
                "left_Fz": float(sync1.left_Fz_N), "right_Fz": float(sync1.right_Fz_N),
                "plantar_normal_force_N": np.array([sync1.left_Fz_N, sync1.right_Fz_N]),
                "fall_contact": bool(sync1.fall_contact), "prohibited": bool(sync1.prohibited_contact),
                "prohibited_contact": bool(sync1.prohibited_contact)})
            low_ok = bool(abs(float(sync1.trunk_tilt_rad)) < 0.2618 and abs(float(sync1.com_velocity_mps[2])) < 0.05 and float(sync1.whole_Fz_N) > 0.5 * MASS_W)
            guard_ok = bool(env_ok and low_ok)
            if guard_ok:
                guard_streak += 1
                guard_best = max(guard_best, guard_streak)
            else:
                guard_streak = 0
            if guard_ok:
                indep_streak += 1
                indep_best = max(indep_best, indep_streak)
                if indep_streak >= E12_NEED and indep_e12_t is None:
                    indep_e12_t = float(sim_t)
            else:
                indep_streak = 0

            if "stable_recovery" in watcher.event_records and e12_occ is None:
                rr = watcher.event_records["stable_recovery"]
                e12_occ = float(rr.occurred_at)
                e12_conf = float(rr.confirmed_at)
                _log(f"[{tag}] E12 occ {e12_occ} conf {e12_conf} at t={sim_t}")
                e12_vec = np.ascontiguousarray(ivec).copy()
                post_t0 = float(sim_t)
                if handoff_t is not None:
                    e12_pre_handoff = max(0.0, float(handoff_t) - float(e12_occ))
                    e12_under_res43 = float(e12_conf) - max(float(e12_occ), float(handoff_t))
                else:
                    e12_pre_handoff = None
                    e12_under_res43 = None
            if post_t0 is not None:
                if not guard_ok:
                    post_bad += 1
                if float(sim_t) - post_t0 >= POST_HOLD - 1e-12:
                    terminal = "PASS_E12_POST_HOLD"
                    break
            if bool(sync1.fall_contact):
                saw_fall = True
                terminal = terminal or "FAIL_FALL"
                break
            saw_prohib = saw_prohib or bool(sync1.prohibited_contact)
            if truncate:
                _log(f"[{tag}] truncated interval break t={sim_t:.6f} regime={regime}")
                break
        ctrl_substeps.append(int(executed))
        if terminal is not None:
            break
        if watcher.physical_fall and "stable_recovery" not in watcher.event_records:
            if saw_fall:
                terminal = "FAIL_FALL"
                break

    fin = watcher.finalize()
    _log(f"[{tag}] TERMINATION {fin.termination} outcome {terminal} t={sim_t}")

    full_adj = SC.adjudicate_trajectory(fz_l=np.asarray(fzL_hist), fz_r=np.asarray(fzR_hist), whole_fz=np.asarray(whole_hist),
        has_row_l=np.asarray(rowL_hist), has_row_r=np.asarray(rowR_hist), nvel_l=np.asarray(nvelL_hist), nvel_r=np.asarray(nvelR_hist),
        dist_l=np.asarray(distL_hist), dist_r=np.asarray(distR_hist))
    Tarr = np.asarray(log_t, float)

    def _sl(t0, t1):
        if len(Tarr) == 0:
            return slice(0, 0)
        i0 = int(np.argmin(np.abs(Tarr - float(t0)))) if t0 is not None else 0
        i1 = int(np.argmin(np.abs(Tarr - float(t1)))) if t1 is not None else len(Tarr) - 1
        i0 = max(0, i0 - 5); i1 = min(len(Tarr) - 1, i1 + 5)
        return slice(i0, i1 + 1)

    def _adj(s):
        return SC.adjudicate_trajectory(fz_l=np.asarray(fzL_hist)[s], fz_r=np.asarray(fzR_hist)[s], whole_fz=np.asarray(whole_hist)[s],
            has_row_l=np.asarray(rowL_hist)[s], has_row_r=np.asarray(rowR_hist)[s], nvel_l=np.asarray(nvelL_hist)[s], nvel_r=np.asarray(nvelR_hist)[s],
            dist_l=np.asarray(distL_hist)[s], dist_r=np.asarray(distR_hist)[s])

    try:
        adj_td_e10 = _adj(_sl(0.875, 0.9935))
    except Exception as exc:
        adj_td_e10 = {"error": str(exc)}
    try:
        adj_e10_rr = _adj(_sl(0.9935, rr_conf_t if rr_conf_t is not None else sim_t))
    except Exception as exc:
        adj_e10_rr = {"error": str(exc)}
    try:
        adj_rr_e12 = _adj(_sl(rr_conf_t if rr_conf_t is not None else 1.337, e12_conf if e12_conf is not None else float(sim_t)))
    except Exception as exc:
        adj_rr_e12 = {"error": str(exc)}

    try:
        land_idx = int(np.argmin(np.abs(Tarr - 0.875))) if len(Tarr) else 0
        pls = slice(land_idx, len(Tarr))
        adj_post = _adj(pls)
        post_loss = int(adj_post.get("CONTROL_RELEVANT_SUPPORT_LOSS_COUNT", -1))
        post_refl = list(adj_post.get("CANONICAL_REFLIGHT", []))
        post_chat = int(adj_post.get("CHATTER_TRANSITIONS", -1))
    except Exception as exc:
        adj_post = {"error": str(exc)}
        post_loss, post_refl, post_chat = -1, [], -1

    ev_online = {k: {"occurred_at": float(v.occurred_at), "confirmed_at": float(v.confirmed_at), "sample_index": int(v.sample_index), "confirmed_sample_index": int(v.confirmed_sample_index)} for k, v in fin.event_records.items()}

    ck_sha = {}
    for kk, vv in checkpoints.items():
        try:
            ck_sha[kk] = _vec_sha(vv)
        except Exception:
            pass

    # Canonical 7 checkpoint map (required gate)
    def _ck(key_candidates):
        for k in key_candidates:
            if k in ck_sha:
                return ck_sha[k]
        return None

    apex_sha = _ck(["S_DET_apex"])
    e10_sha = _ck(["S_E10_ctrl", "S_DET_impact_absorption"])
    e11_sha = _ck(["S_DET_balance_capture"])
    rr_sha = _ck(["S_RR_ctrl"])
    rrc_sha = _ck(["S_RR_CONFIRMED_ctrl"])
    handoff_sha = _vec_sha(handoff_vec) if handoff_vec is not None else None
    e12_sha = _vec_sha(e12_vec) if e12_vec is not None else _ck(["S_DET_stable_recovery"])

    trace_sha = trace_stream.hexdigest()

    # Substep histogram
    import collections as _co
    hist = dict(_co.Counter(int(x) for x in ctrl_substeps))

    # Offline recompute (fresh detector from stored raw samples, no controller)
    from loaded_cmj.v2.events import V2EventDetector as _DetOff
    det_off = _DetOff(); det_off.reset()
    for ev in event_samples_raw:
        det_off.update(dict(ev))
    fin_off = det_off.finalize()
    ev_off = {k: {"occurred_at": float(v.occurred_at), "confirmed_at": float(v.confirmed_at), "sample_index": int(v.sample_index), "confirmed_sample_index": int(v.confirmed_sample_index)} for k, v in fin_off.event_records.items()}
    online_offline = bool(ev_online == ev_off and str(fin.termination) == str(fin_off.termination))

    result = {
        "MISSION": MISSION,
        "EXPERIMENT_ID": EXPERIMENT_ID,
        "CANDIDATE_ID": CANDIDATE_ID,
        "CANONICAL_COMMAND": CANONICAL_COMMAND,
        "CANONICAL_RUNTIME_MODULE": CANONICAL_RUNTIME_MODULE,
        "TAG": tag,
        "OUTCOME": terminal,
        "TERMINATION": str(fin.termination),
        "T_END": float(sim_t),
        "N_CTRL": int(len(ctrl_times)),
        "N_PHYS": int(len(Tarr)),
        "MODE_FINAL": regime,
        "E10_HANDOFF_T": e10_switch_t,
        "RR_ENTRY_T": rr_entry_t,
        "RR_CONF_T": rr_conf_t,
        "HANDOFF_T": handoff_t,
        "E12_OCC": e12_occ, "E12_CONF": e12_conf,
        "E12_PRE_HANDOFF_S": e12_pre_handoff, "E12_UNDER_RES43_S": e12_under_res43,
        "POST_VIOL": int(post_bad),
        "PEAK_BW": float(peak_fz / MASS_W), "MAXPEN_M": float(max_pen), "MAXUTIL": float(max_util), "MAXROOTPASSIVE": float(max_root_passive),
        "ROOT_ROWS": int(root_rows), "FINITE": bool(finite_ok), "FALL": bool(saw_fall), "PROHIB": bool(saw_prohib),
        "SUPPORT_FULL": full_adj,
        "SUPPORT_TD_E10": adj_td_e10,
        "SUPPORT_E10_RR": adj_e10_rr,
        "SUPPORT_RR_E12": adj_rr_e12,
        "SUPPORT_POST_LANDING": adj_post,
        "POST_LANDING_LOSS": post_loss, "POST_LANDING_REFLIGHT": post_refl, "POST_LANDING_CHATTER": post_chat,
        "EVENTS": ev_online,
        "EVENTS_OFFLINE": ev_off,
        "ONLINE_OFFLINE_IDENTITY": "PASS" if online_offline else "FAIL",
        "INDEP_E12T": indep_e12_t, "INDEP_BEST_S": float(indep_best * PHYS_DT),
        "GUARD_BEST_S": float(guard_best * PHYS_DT),
        "TRACE_SHA256": trace_sha,
        "CHECKPOINT_SHAS": {k: ck_sha.get(k) for k in sorted(ck_sha)},
        "CHECKPOINT_TIMES": {k: float(v) for k, v in checkpoint_t.items()},
        "CANONICAL_CHECKPOINTS": {
            "S_APEX": apex_sha,
            "S_E10": e10_sha,
            "S_E11": e11_sha,
            "S_RR": rr_sha,
            "S_RR_CONFIRMED": rrc_sha,
            "S_STAND_HANDOFF": handoff_sha,
            "S_E12": e12_sha,
        },
        "CTRL_SUBSTEP_HISTOGRAM": {str(k): int(v) for k, v in sorted(hist.items())},
        "WALL_S": float(time.time() - t_wall0),
        "ACTION_DIM": 7, "NQ": 10, "NV": 10, "NU": 7,
        "PHYSICS_DT": PHYS_DT, "CONTROL_DT_NOMINAL": CTRL_DT,
        "HORIZON_S": HORIZON, "POST_HOLD_S": POST_HOLD,
    }

    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / f"{tag}_CANONICAL_RESULT.json").write_text(json.dumps(result, indent=2))
        np.save(out_dir / f"{tag}_ctrl_t.npy", np.asarray(ctrl_times, float))
        np.save(out_dir / f"{tag}_ctrl_u.npy", np.asarray(ctrl_actions, float) if len(ctrl_actions) else np.zeros((0, 7)))
        np.save(out_dir / f"{tag}_ctrl_mode.npy", np.asarray(ctrl_modes))
        np.save(out_dir / f"{tag}_ctrl_substeps.npy", np.asarray(ctrl_substeps, dtype=np.int64))
        for kk, vv in checkpoints.items():
            if kk.startswith("S_E10") or kk.startswith("S_RR") or kk.startswith("S_DET"):
                np.save(out_dir / f"{tag}_{kk}_vector.npy", np.asarray(vv, float))
        if handoff_vec is not None:
            np.save(out_dir / f"{tag}_S_STAND_HANDOFF_full_vector.npy", np.asarray(handoff_vec, float))
        if e12_vec is not None:
            np.save(out_dir / f"{tag}_S_E12_full_vector.npy", np.asarray(e12_vec, float))
        np.savez_compressed(out_dir / f"{tag}_ACTION_SCHEDULE.npz",
            action=np.asarray(ctrl_actions, float) if len(ctrl_actions) else np.zeros((0, 7)),
            control_time=np.asarray(ctrl_times, float),
            substeps=np.asarray(ctrl_substeps, dtype=np.int64),
            mode=np.asarray(ctrl_modes))

    return result


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="V2.1 canonical qualification runtime (RES-78)")
    ap.add_argument("--candidate", required=True, help="Candidate ID, must be V2.1-R001")
    ap.add_argument("--out", default="", help="Output JSON path for machine-readable result")
    ap.add_argument("--artifacts-dir", default="", help="Optional directory for npy artifacts")
    ap.add_argument("--verbose", action="store_true", help="Verbose progress")
    args = ap.parse_args(argv)
    if args.candidate != CANDIDATE_ID:
        print(f"FAIL: unknown candidate {args.candidate!r}, expected {CANDIDATE_ID!r}", file=sys.stderr)
        return 2
    out_dir = Path(args.artifacts_dir) if args.artifacts_dir else None
    result = run_canonical_episode(out_dir=out_dir, tag=args.candidate, verbose=args.verbose)
    out_path = Path(args.out) if args.out else Path(f"canonical_{args.candidate}_result.json")
    # If artifacts dir given but --out not, also keep JSON inside artifacts dir
    out_path.write_text(json.dumps(result, indent=2))
    print(json.dumps({k: result[k] for k in ["MISSION", "EXPERIMENT_ID", "CANDIDATE_ID", "OUTCOME", "TERMINATION", "T_END", "N_CTRL", "N_PHYS", "ONLINE_OFFLINE_IDENTITY", "TRACE_SHA256"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
