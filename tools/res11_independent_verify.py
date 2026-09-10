#!/usr/bin/env python3
"""RES-11 independent deterministic 12/12 offline verifier (fresh implementation).

MISSION=RES11_EXACT_FORWARD_DETERMINISTIC_12_OF_12_OFFLINE_001
EXPERIMENT_ID=EXP-RES11-DETERMINISTIC-12OF12-OFFLINE-001

Independent harness: does NOT import or execute RES-76 run_full_qualification.py,
does NOT use RES-76 saved states/actions/events as simulation inputs.
Exercises committed production composition directly:
  PRELANDING Res72Policy -> BALANCE BalanceController -> RECOVERY StableRecoveryController -> RES43 HANDOFF
with controller-local physical predicates, truncated hybrid intervals,
phase-local grid reset, BALANCE->RECOVERY u_prev=zeros (action-history reset only).

Closed-loop: fresh canonical reset, exact mj_step sole physics authority,
synchronized SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL, observational scorer.
Open-loop: fresh reset + exact replay of Run A schedule (no controller).
Offline: fresh V2EventDetector after simulation from stored raw trace.
"""
from __future__ import annotations
import sys
import json
import hashlib
import time
from pathlib import Path
from collections import deque

REPO = Path("/home/litju/Projects/loaded-cmj-control")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools"))
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


def run_closed_loop(out_dir: Path, tag: str) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
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

    # per-step log for offline reconstruction + support + dwell
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
            print(f"[{tag}] E10->BALANCE at t={e10_switch_t:.6f} step={step_idx}", flush=True)
        if regime == "BALANCE" and rr_armed:
            recoverer = StableRecoveryController(plant, manifold, hspec, t_rise, probes_rec, vprobe_rec, clearance)
            regime = "RECOVERY"
            rr_armed = False
            # Intentional BALANCE->RECOVERY action-history reset (control discontinuity only, NOT state reset)
            held = np.zeros(7, float)
            print(f"[{tag}] BALANCE->RECOVERY at t={float(sync0.time):.6f} step={step_idx} u_prev=zeros (action-history reset, no state reset)", flush=True)

        if regime == "PRELANDING":
            u_cmd, cinfo = prelanding.act(obs0, sample=sync0, meas=shadow, plant=plant)
            u_cmd = np.asarray(u_cmd, float)
            phase_tag = f"PRL_{cinfo.get('phase_name', cinfo.get('phase'))}"
            rec_s = float("nan")
            rec_sub = prelanding.phase
        elif regime == "BALANCE":
            assert balancer is not None
            u_cmd, binfo = balancer.step(state0, held, float(sync0.time), com0, cv0, mom0)
            u_cmd = np.asarray(u_cmd, float)
            phase_tag = "BALANCE"
            rec_s = float("nan")
            rec_sub = "BAL"
        elif regime == "RECOVERY":
            assert recoverer is not None
            sm0 = plant.foot_contact_summary(shadow)
            u_cmd, rinfo = recoverer.step(state0, held, float(sync0.time), com0, cv0, mom0, sync0, sm0, trunk_rate0, bool(env0))
            u_cmd = np.asarray(u_cmd, float)
            if getattr(recoverer, "mode", None) == "HANDOFF" and handoff_t is None:
                handoff_t = float(sync0.time)
                handoff_vec = np.ascontiguousarray(state0).copy()
                print(f"[{tag}] RECOVERY->RES43 HANDOFF at t={handoff_t:.6f} s={recoverer.s}", flush=True)
            phase_tag = f"REC_{recoverer.mode}"
            rec_s = float(recoverer.s)
            rec_sub = recoverer.mode
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

            # streaming integration hash (post-step live state)
            ivec = _fetch_vec(mm, live)
            trace_stream.update(np.ascontiguousarray(ivec).tobytes())
            n_phys_total += 1

            # touchdown latch for Res72 (TD-relative handoff)
            cur_phase = int(prelanding.phase) if regime == "PRELANDING" else -1
            if regime == "PRELANDING" and prelanding.td_physics_t is None and sim_t > 0.6 and cur_phase in (6, 3, 7) and float(sync1.whole_Fz_N) >= 20.0:
                prelanding.notify_touchdown_physics(float(sim_t))

            past_vecs.append((float(sim_t), np.ascontiguousarray(ivec).copy()))

            # controller-local E10 dwell (physics-rate, TERMINAL-active, no detector)
            if regime == "PRELANDING" and (not e10_armed) and e10_switch_t is None:
                if int(prelanding.phase) == 7:
                    e10_ok = bool(abs(float(cv1[2])) < 0.05 and float(sm1["left_Fz"]) > 10.0 and float(sm1["right_Fz"]) > 10.0 and not bool(sync1.fall_contact) and not bool(sync1.prohibited_contact))
                    if e10_ok:
                        e10_streak += 1
                        if e10_streak >= E10_NEED and (not e10_armed):
                            e10_armed = True
                            print(f"[{tag}] E10 dwell 160 complete at t={sim_t:.6f} (truncate)", flush=True)
                            checkpoints["S_E10_ctrl"] = np.ascontiguousarray(ivec).copy()
                            checkpoint_t["S_E10_ctrl_t"] = float(sim_t)
                            truncate = True
                    else:
                        e10_streak = 0
                else:
                    e10_streak = 0

            # controller-local RR dwell (BALANCE)
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
                        print(f"[{tag}] RR dwell entry {rr_entry_t:.6f} -> conf {rr_conf_t:.6f} (truncate)", flush=True)
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

            # observational detector (must not influence control)
            watcher.update(dict(ev1))
            for nm in list(watcher.event_records.keys()):
                if nm not in seen_online:
                    seen_online.add(nm)
                    try:
                        checkpoints[f"S_DET_{nm}"] = np.ascontiguousarray(ivec).copy()
                        checkpoint_t[f"S_DET_{nm}_t"] = float(sim_t)
                        print(f"[{tag}] DET {nm} conf t={sim_t:.6f} occ {watcher.event_records[nm].occurred_at:.6f}", flush=True)
                    except Exception as exc:
                        print(f"[{tag}] checkpoint save fail {nm}: {exc}", flush=True)

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
                print(f"[{tag}] E12 occ {e12_occ} conf {e12_conf} at t={sim_t}", flush=True)
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
                print(f"[{tag}] truncated interval break t={sim_t:.6f} regime={regime}", flush=True)
                break
        ctrl_substeps.append(int(executed))
        if terminal is not None:
            break
        if step_idx % 200 == 199:
            extra = ""
            if regime == "RECOVERY" and recoverer is not None:
                extra = f" rec={recoverer.mode} s={float(recoverer.s):.4f}"
            print(f"[{tag}] step {step_idx} t={sim_t:.2f} {regime}{extra} peak={peak_fz/MASS_W:.2f}BW wall={time.time()-t_wall0:.0f}s", flush=True)
        if watcher.physical_fall and "stable_recovery" not in watcher.event_records:
            if saw_fall:
                terminal = "FAIL_FALL"
                break

    fin = watcher.finalize()
    print(f"[{tag}] TERMINATION {fin.termination} outcome {terminal} t={sim_t}", flush=True)

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

    # post-landing scoped (landing at ~0.875): zero loss/reflight/chatter required
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

    trace_sha = trace_stream.hexdigest()
    result = {
        "MISSION": "RES11_EXACT_FORWARD_DETERMINISTIC_12_OF_12_OFFLINE_001",
        "EXPERIMENT_ID": "EXP-RES11-DETERMINISTIC-12OF12-OFFLINE-001",
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
        "INDEP_E12T": indep_e12_t, "INDEP_BEST_S": float(indep_best * PHYS_DT),
        "GUARD_BEST_S": float(guard_best * PHYS_DT),
        "TRACE_SHA256": trace_sha,
        "CHECKPOINT_SHAS": {k: ck_sha.get(k) for k in sorted(ck_sha)},
        "CHECKPOINT_TIMES": {k: float(v) for k, v in checkpoint_t.items()},
        "WALL_S": float(time.time() - t_wall0),
    }

    # persist
    (out_dir / f"{tag}_CLOSED_LOOP.json").write_text(json.dumps(result, indent=2))
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
    # action schedule with truncated durations (replay authority)
    np.savez_compressed(out_dir / f"{tag}_ACTION_SCHEDULE.npz",
        action=np.asarray(ctrl_actions, float) if len(ctrl_actions) else np.zeros((0, 7)),
        control_time=np.asarray(ctrl_times, float),
        substeps=np.asarray(ctrl_substeps, dtype=np.int64),
        mode=np.asarray(ctrl_modes))
    # raw trace for offline recompute (event samples + support + logs)
    np.savez_compressed(out_dir / f"{tag}_trace_compact.npz",
        t=np.asarray(log_t, float), vx=np.asarray(log_vx, float), vz=np.asarray(log_vz, float),
        Hy=np.asarray(log_hy, float), Fz=np.asarray(log_fz, float), FzL=np.asarray(log_fzL, float),
        FzR=np.asarray(log_fzR, float), pen=np.asarray(log_pen, float), margin=np.asarray(log_margin, float),
        tilt=np.asarray(log_tilt, float), pelvis_z=np.asarray(log_pelvisz, float),
        comx=np.asarray(log_comx, float), comz=np.asarray(log_comz, float),
        ctrl_t=np.asarray(ctrl_times, float))
    # save event samples raw for exact offline (json-friendly conversion)
    serializable = []
    for ev in event_samples_raw:
        row = {}
        for k, v in ev.items():
            if isinstance(v, (float, np.floating)):
                row[k] = float(v)
            elif isinstance(v, (bool, np.bool_)):
                row[k] = bool(v)
            elif isinstance(v, (int, np.integer)):
                row[k] = int(v)
            elif isinstance(v, (list, np.ndarray)):
                row[k] = np.asarray(v).tolist()
            else:
                row[k] = str(v)
        serializable.append(row)
    (out_dir / f"{tag}_event_samples.json").write_text(json.dumps(serializable))
    # also save q7/root separately (needed for offline true-standing)
    np.savez_compressed(out_dir / f"{tag}_trace_aux.npz",
        q7=np.asarray(log_q7, float) if len(log_q7) else np.zeros((0, 7)),
        root=np.asarray(log_root, float) if len(log_root) else np.zeros((0, 3)),
        prohib=np.asarray(log_prohib, bool), fall=np.asarray(log_fall, bool))
    (out_dir / f"{tag}_events_online.json").write_text(json.dumps(ev_online, indent=2))
    return result


def run_open_replay(schedule_path: Path, out_path: Path) -> dict:
    schedule_path = Path(schedule_path)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bundle = np.load(str(schedule_path))
    actions = np.asarray(bundle["action"], float)
    substeps = np.asarray(bundle["substeps"], dtype=np.int64).tolist()
    assert len(actions) == len(substeps), "schedule length mismatch"
    # fresh canonical reset, no controller
    plant = V2Plant()
    C52.bind_plant(plant)
    mm = plant.model
    live = plant.make_data()
    plant.reset(live)
    mujoco.mj_forward(mm, live)
    shadow = create_measurement_data(plant)
    watcher = V2EventDetector()
    watcher.reset()
    stream = hashlib.sha256()
    sim_t = 0.0
    peak = 0.0
    for u, nsub in zip(actions, substeps):
        u = np.asarray(u, float)
        plant.apply_action(live, u)
        for _ in range(int(nsub)):
            mujoco.mj_step(mm, live)
            sim_t += PHYS_DT
            sync1 = SynchronizedPhysicsSample.from_live_state(plant, live, shadow)
            ev = dict(sync1.event_sample())
            ev["time_s"] = float(sim_t)
            watcher.update(ev)
            peak = max(peak, float(sync1.whole_Fz_N))
            ivec = _fetch_vec(mm, live)
            stream.update(np.ascontiguousarray(ivec).tobytes())
    fin = watcher.finalize()
    final_vec = _fetch_vec(mm, live)
    res = {
        "N_CTRL": int(len(actions)),
        "N_PHYS": int(sum(int(x) for x in substeps)),
        "T_END": float(sim_t),
        "TERMINATION": str(fin.termination),
        "TRACE_SHA256": stream.hexdigest(),
        "FINAL_SHA256": _vec_sha(final_vec),
        "EVENTS": {k: {"occurred_at": float(v.occurred_at), "confirmed_at": float(v.confirmed_at), "sample_index": int(v.sample_index), "confirmed_sample_index": int(v.confirmed_sample_index)} for k, v in fin.event_records.items()},
        "PEAK_BW": float(peak / MASS_W),
    }
    out_path.write_text(json.dumps(res, indent=2))
    np.save(str(out_path).replace(".json", "_final_vector.npy"), np.asarray(final_vec, float))
    return res


def recompute_offline_closed(trace_dir: Path, tag: str, out_dir: Path) -> dict:
    trace_dir = Path(trace_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # Prefer exact stored event samples (bit-identical floats logged online)
    raw_path = trace_dir / f"{tag}_event_samples.json"
    aux_path = trace_dir / f"{tag}_trace_aux.npz"
    raw = json.loads(raw_path.read_text())
    aux = np.load(str(aux_path))
    q7_all = np.asarray(aux["q7"], float)
    root_all = np.asarray(aux["root"], float)
    det = V2EventDetector()
    det.reset()
    for idx, ev in enumerate(raw):
        # rebuild full sample expected by detector (same keys as online)
        q7 = q7_all[idx].tolist() if idx < len(q7_all) else ev.get("joint_position_rad")
        # qpos order: root_tx,tz,ry,lumbar,lhip,lknee,lank,rhip,rknee,rank
        # online qpos from SynchronizedPhysicsSample.qpos (10). Our stored event_samples already contain qpos.
        # Ensure joint_position_rad consistent with q7 log (same floats)
        ev2 = dict(ev)
        ev2["joint_position_rad"] = [float(x) for x in q7]
        det.update(ev2)
    fin = det.finalize()
    ev_off = {k: {"occurred_at": float(v.occurred_at), "confirmed_at": float(v.confirmed_at), "sample_index": int(v.sample_index), "confirmed_sample_index": int(v.confirmed_sample_index)} for k, v in fin.event_records.items()}
    (out_dir / "EVENTS_OFFLINE_RECOMPUTED.json").write_text(json.dumps(ev_off, indent=2))
    # independent E12 dwell: 4000-sample / 0.499875 s interval audit from raw trace
    # Recompute guard per sample using fresh logic mirroring online but from stored fields
    # Stored event samples contain needed fields for true-standing except pelvis/com details -> use aux + raw
    dwell_run = 0
    dwell_best = 0
    dwell_conf_t = None
    # need to evaluate guard per sample: reuse detector predicate via reconstructed sample dict
    probe_det = V2EventDetector()
    for idx, ev in enumerate(raw):
        q7 = [float(x) for x in q7_all[idx]] if idx < len(q7_all) else ev.get("joint_position_rad")
        root = root_all[idx] if idx < len(root_all) else [0, 0.9, 0]
        sample = {
            "joint_position_rad": q7,
            "com_z": float(ev.get("com_z")),
            "com_position_m": ev.get("com_position_m"),
            "pelvis_position_world_m": ev.get("pelvis_position_world_m"),
            "trunk_tilt": float(ev.get("trunk_tilt")),
            "left_Fz": float(ev.get("left_Fz")), "right_Fz": float(ev.get("right_Fz")),
            "plantar_normal_force_N": [float(ev.get("left_Fz")), float(ev.get("right_Fz"))],
            "fall_contact": bool(ev.get("fall_contact")),
            "prohibited": bool(ev.get("prohibited")), "prohibited_contact": bool(ev.get("prohibited")),
            "qpos": ev.get("qpos"),
        }
        in_env = probe_det._is_true_standing_neighborhood(sample)
        low = bool(abs(float(ev.get("trunk_tilt"))) < 0.2618 and abs(float(ev.get("com_vz"))) < 0.05 and float(ev.get("whole_Fz")) > 0.5 * MASS_W)
        ok = bool(in_env and low)
        if ok:
            dwell_run += 1
            dwell_best = max(dwell_best, dwell_run)
            if dwell_run >= 4000 and dwell_conf_t is None:
                dwell_conf_t = float(ev.get("time_s"))
        else:
            dwell_run = 0
    audit = {
        "E12_NEED_SAMPLES": E12_NEED,
        "E12_NEED_S": float((E12_NEED - 1) * PHYS_DT),
        "E12_NEED_S_SAMPLES_X_DT": float(E12_NEED * PHYS_DT),
        "INDEP_BEST_SAMPLES": int(dwell_best),
        "INDEP_BEST_S": float(dwell_best * PHYS_DT),
        "INDEP_CONF_T": dwell_conf_t,
        "TERMINATION": str(fin.termination),
    }
    (out_dir / "E12_INDEPENDENT_DWELL_AUDIT.json").write_text(json.dumps(audit, indent=2))
    return {"events_offline": ev_off, "dwell": audit, "termination": str(fin.termination)}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["closed", "replay", "offline"], required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default="RUN_A")
    ap.add_argument("--schedule", default="")
    ap.add_argument("--trace-dir", default="")
    args = ap.parse_args()
    if args.mode == "closed":
        res = run_closed_loop(Path(args.out), args.tag)
        print(json.dumps({k: res[k] for k in ["TAG", "TERMINATION", "OUTCOME", "T_END", "N_CTRL", "N_PHYS", "TRACE_SHA256"]}, indent=2))
    elif args.mode == "replay":
        res = run_open_replay(Path(args.schedule), Path(args.out))
        print(json.dumps(res, indent=2))
    elif args.mode == "offline":
        res = recompute_offline_closed(Path(args.trace_dir), args.tag, Path(args.out))
        print(json.dumps(res["dwell"], indent=2))
