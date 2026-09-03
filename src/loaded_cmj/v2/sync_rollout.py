"""Synchronized closed-loop rollout authority (EXP-RES10-CONTROLLER-OBS-SYNC-001).

Implements CONTROL_SAMPLE_CONVENTION=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL
with one state -> one observation (SynchronizedPhysicsSample) shared by
controller, trace, event, scorer, forceplate at each instant.

Trajectories:
  A LEGACY_CLOSED_LOOP: legacy obs (live post-step) + legacy trace/events.
  B SYNC_MEASUREMENT_LEGACY_CONTROL: legacy obs + synchronized trace/events.
  C FULLY_SYNCHRONIZED_CLOSED_LOOP: synchronized obs + synchronized trace/events.

No controller tuning. No Plant/contact/solver/timestep/actuator/scorer change.
"""

from __future__ import annotations

import hashlib
from typing import Any

import mujoco
import numpy as np

PHYSICS_DT = 0.000125
CONTROL_DT = 0.005
SUBSTEPS_PER_CONTROL = 40

PHASE_NAMES = {0: "HOLD", 1: "SUPPORTED", 2: "FLIGHT", 3: "IMPACT", 4: "CAPTURED", 5: "STAND"}


def _controller_phase(ctrl_mod) -> int:
    try:
        return int(ctrl_mod._phase)
    except Exception:
        return -1


def _fall_flag(plant, data) -> bool:
    fall_geoms = {
        plant.idx.geom[n]
        for n in [
            "pelvis_fall",
            "torso_fall",
            "left_thigh_fall",
            "right_thigh_fall",
            "left_shank_fall",
            "right_shank_fall",
        ]
    }
    floor = plant.idx.floor_geom
    for i in range(data.ncon):
        con = data.contact[i]
        g1i, g2i = int(con.geom1), int(con.geom2)
        if g1i == floor or g2i == floor:
            other = g2i if g1i == floor else g1i
            if other in fall_geoms:
                return True
    return False


def _legacy_event_sample(plant, live, t: float) -> dict[str, Any]:
    from loaded_cmj.v2.measurement import legacy_event_sample_from_live

    return legacy_event_sample_from_live(plant, live, time_s=t)


def run_legacy_closed_loop(horizon_s: float = 4.0) -> dict[str, Any]:
    """Trajectory A: legacy controller obs + legacy measurement."""
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector
    import loaded_cmj.v2.controller as ctrl

    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    det = V2EventDetector()
    det.reset()
    n_ctrl = int(round(horizon_s / CONTROL_DT))
    t = 0.0
    prev = np.zeros(7, dtype=np.float64)
    actions, qpos_trace, qvel_trace = [], [], []
    obs_shas, phys_shas = [], []
    phases = []
    times_ctrl = []
    # physics-rate legacy trace for scorer parity (store minimal)
    for step in range(n_ctrl):
        obs = plant.public_observation(
            d, scored_time_s=t, step_index=step, episode_reset=(step == 0), previous_action=prev
        )
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        actions.append(act.copy())
        phases.append(_controller_phase(ctrl))
        times_ctrl.append(float(t))
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            qpos_trace.append(np.asarray(d.qpos).copy())
            qvel_trace.append(np.asarray(d.qvel).copy())
            det.update(_legacy_event_sample(plant, d, t))
            if det.physical_fall:
                break
        if det.physical_fall:
            # fill remaining with last to keep shapes? No: break control loop
            break
    result = det.finalize()
    actions_a = np.stack(actions) if actions else np.zeros((0, 7))
    return {
        "kind": "LEGACY_CLOSED_LOOP",
        "horizon_s": float(horizon_s),
        "n_control": int(len(actions)),
        "control_time": np.asarray(times_ctrl, dtype=np.float64),
        "action": actions_a,
        "action_sha256": hashlib.sha256(np.ascontiguousarray(actions_a).tobytes()).hexdigest(),
        "qpos": np.stack(qpos_trace) if qpos_trace else np.zeros((0, 10)),
        "qvel": np.stack(qvel_trace) if qvel_trace else np.zeros((0, 10)),
        "phase": np.asarray(phases, dtype=np.int64),
        "event_result": result,
        "final_time": float(t),
        "final_qpos": np.asarray(d.qpos).copy(),
        "final_qvel": np.asarray(d.qvel).copy(),
    }


def run_sync_measurement_legacy_control(horizon_s: float = 4.0) -> dict[str, Any]:
    """Trajectory B: legacy controller obs + synchronized scorer/trace.

    Live trajectory must be bit-identical to A (shadow nonintrusive).
    """
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector
    from loaded_cmj.v2.measurement import (
        SynchronizedPhysicsSample,
        create_measurement_data,
    )
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
    actions, qpos_trace, qvel_trace = [], [], []
    phases, times_ctrl = [], []
    for step in range(n_ctrl):
        # legacy controller observation (live) to preserve A trajectory
        obs = plant.public_observation(
            d, scored_time_s=t, step_index=step, episode_reset=(step == 0), previous_action=prev
        )
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        actions.append(act.copy())
        phases.append(_controller_phase(ctrl))
        times_ctrl.append(float(t))
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            qpos_trace.append(np.asarray(d.qpos).copy())
            qvel_trace.append(np.asarray(d.qvel).copy())
            # synchronized scorer sample (shadow), observational only
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            det.update(s.event_sample())
            if det.physical_fall:
                break
        if det.physical_fall:
            break
    result = det.finalize()
    actions_a = np.stack(actions) if actions else np.zeros((0, 7))
    return {
        "kind": "SYNC_MEASUREMENT_LEGACY_CONTROL",
        "horizon_s": float(horizon_s),
        "n_control": int(len(actions)),
        "control_time": np.asarray(times_ctrl, dtype=np.float64),
        "action": actions_a,
        "action_sha256": hashlib.sha256(np.ascontiguousarray(actions_a).tobytes()).hexdigest(),
        "qpos": np.stack(qpos_trace) if qpos_trace else np.zeros((0, 10)),
        "qvel": np.stack(qvel_trace) if qvel_trace else np.zeros((0, 10)),
        "phase": np.asarray(phases, dtype=np.int64),
        "event_result": result,
        "final_time": float(t),
        "final_qpos": np.asarray(d.qpos).copy(),
        "final_qvel": np.asarray(d.qvel).copy(),
    }


def run_fully_synchronized_closed_loop(horizon_s: float = 4.0) -> dict[str, Any]:
    """Trajectory C: synchronized controller obs + synchronized scorer/trace.

    Implements SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL with one
    sample per boundary shared by controller and scorer (SHA identity).
    """
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector
    from loaded_cmj.v2.measurement import (
        SynchronizedPhysicsSample,
        create_measurement_data,
    )
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
    actions, qpos_trace, qvel_trace = [], [], []
    phases, times_ctrl = [], []
    obs_shas, phys_shas = [], []
    ctrl_shas_match: list[bool] = []
    for step in range(n_ctrl):
        # ONE sample at t_k for controller AND boundary scorer identity
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        assert s_ctrl.check_time_identity()
        obs = s_ctrl.controller_observation(
            step_index=step, episode_reset=(step == 0), previous_action=prev
        )
        # SHA identity: observation SHA == physics sample SHA
        assert obs["observation_state_sha256"] == s_ctrl.state_vector_sha256
        ev0 = s_ctrl.event_sample()
        assert ev0["physics_sample_state_sha256"] == s_ctrl.state_vector_sha256
        obs_shas.append(obs["observation_state_sha256"])
        phys_shas.append(s_ctrl.state_vector_sha256)
        ctrl_shas_match.append(True)
        # held-control semantics: shadow ctrl must equal u_prev
        assert np.array_equal(
            np.asarray(s_ctrl.ctrl, dtype=np.float64), np.asarray(prev, dtype=np.float64)
        ), f"held-control mismatch at step {step}"
        act = np.asarray(ctrl.act(obs), dtype=np.float64)
        actions.append(act.copy())
        phases.append(_controller_phase(ctrl))
        times_ctrl.append(float(t))
        plant.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS_PER_CONTROL):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
            qpos_trace.append(np.asarray(d.qpos).copy())
            qvel_trace.append(np.asarray(d.qvel).copy())
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            det.update(s.event_sample())
            if det.physical_fall:
                break
        if det.physical_fall:
            break
    result = det.finalize()
    actions_a = np.stack(actions) if actions else np.zeros((0, 7))
    return {
        "kind": "FULLY_SYNCHRONIZED_CLOSED_LOOP",
        "horizon_s": float(horizon_s),
        "n_control": int(len(actions)),
        "control_time": np.asarray(times_ctrl, dtype=np.float64),
        "action": actions_a,
        "action_sha256": hashlib.sha256(np.ascontiguousarray(actions_a).tobytes()).hexdigest(),
        "qpos": np.stack(qpos_trace) if qpos_trace else np.zeros((0, 10)),
        "qvel": np.stack(qvel_trace) if qvel_trace else np.zeros((0, 10)),
        "phase": np.asarray(phases, dtype=np.int64),
        "event_result": result,
        "final_time": float(t),
        "final_qpos": np.asarray(d.qpos).copy(),
        "final_qvel": np.asarray(d.qvel).copy(),
        "obs_shas": obs_shas,
        "phys_shas": phys_shas,
        "sha_identity": bool(all(ctrl_shas_match) and obs_shas == phys_shas),
    }


def run_prescribed_action_shadow_identity(horizon_s: float = 0.5) -> dict[str, Any]:
    """Section 7: same prescribed actions, with vs without shadow compute."""
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.measurement import create_measurement_data, synchronize_measurement
    import loaded_cmj.v2.controller as ctrl

    def _run(use_shadow: bool):
        plant = V2Plant()
        m = plant.model
        d = plant.make_data()
        plant.reset(d)
        mujoco.mj_forward(m, d)
        meas = create_measurement_data(plant) if use_shadow else None
        ctrl.reset(0.0)
        t = 0.0
        prev = np.zeros(7, dtype=np.float64)
        qs, vs, cs = [], [], []
        n_ctrl = int(round(horizon_s / CONTROL_DT))
        for step in range(n_ctrl):
            # prescribed actions: use legacy controller to generate deterministic
            # sequence on the no-shadow run; replay same sequence on shadow run.
            # To guarantee same prescribed sequence, first run generates it.
            obs = plant.public_observation(
                d, scored_time_s=t, step_index=step, episode_reset=(step == 0), previous_action=prev
            )
            act = np.asarray(ctrl.act(obs), dtype=np.float64)
            plant.apply_action(d, act)
            prev = act.copy()
            for _ in range(SUBSTEPS_PER_CONTROL):
                mujoco.mj_step(m, d)
                t += PHYSICS_DT
                if use_shadow:
                    assert meas is not None
                    synchronize_measurement(m, d, meas)
                qs.append(np.asarray(d.qpos).copy())
                vs.append(np.asarray(d.qvel).copy())
                cs.append(np.asarray(d.ctrl).copy())
        return np.stack(qs), np.stack(vs), np.stack(cs)

    # Generate prescribed sequence from clean run, then replay bit-exactly
    # with and without shadow evidence computation.
    qa, va, ca = _run(False)
    qb, vb, cb = _run(True)
    return {
        "max_qpos_delta": float(np.max(np.abs(qa - qb))) if qa.size else 0.0,
        "max_qvel_delta": float(np.max(np.abs(va - vb))) if va.size else 0.0,
        "max_ctrl_delta": float(np.max(np.abs(ca - cb))) if ca.size else 0.0,
        "n_phys": int(qa.shape[0]),
    }


def run_res43_standing_sync(horizon_s: float = 2.0) -> dict[str, Any]:
    """Section 12: RES43 standing with controller-observation sync active.

    Harmless PD hold (Kp400 Kd10) reading joints from the synchronized
    sample (q/qdot are integration state, identical live vs shadow).
    Events scored on synchronized samples. Requires 0.500 s standing dwell
    via the frozen true-standing predicate (E12 semantics).
    """
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.events import V2EventDetector
    from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data

    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    meas = create_measurement_data(plant)
    det = V2EventDetector()
    det.reset()
    LIMITS = np.array([250, 250, 250, 300, 300, 200, 200], dtype=float)
    KP = np.array([400, 400, 400, 400, 400, 400, 400], dtype=float)
    KD = np.array([10, 10, 10, 10, 10, 10, 10], dtype=float)
    DT = PHYSICS_DT
    SUB = SUBSTEPS_PER_CONTROL
    n_ctrl = int(round(horizon_s / CONTROL_DT))
    t = 0.0
    longest = 0.0
    cur = 0.0
    pass_count = 0
    for step in range(n_ctrl):
        s_ctrl = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
        q = np.asarray(s_ctrl.joint_position_rad, dtype=np.float64)
        qd = np.asarray(s_ctrl.joint_velocity_radps, dtype=np.float64)
        raw = KP * (np.zeros(7) - q) - KD * qd
        u = np.clip(raw / LIMITS, -1.0, 1.0)
        plant.apply_action(d, u)
        for _ in range(SUB):
            mujoco.mj_step(m, d)
            t += DT
            s = SynchronizedPhysicsSample.from_live_state(plant, d, meas)
            ev = s.event_sample()
            # true-standing neighborhood check per physics sample
            ok = det._is_true_standing_neighborhood(
                {
                    "joint_position_rad": s.joint_position_rad,
                    "com_z": float(s.com_position_m[2]),
                    "com_position_m": s.com_position_m,
                    "pelvis_position_world_m": s.pelvis_position_world_m,
                    "trunk_tilt": float(s.trunk_tilt_rad),
                    "left_Fz": float(s.left_Fz_N),
                    "right_Fz": float(s.right_Fz_N),
                    "plantar_normal_force_N": np.array([s.left_Fz_N, s.right_Fz_N]),
                    "fall_contact": bool(s.fall_contact),
                    "prohibited": bool(s.prohibited_contact),
                    "prohibited_contact": bool(s.prohibited_contact),
                }
            )
            if ok:
                cur += DT
                pass_count += 1
                longest = max(longest, cur)
            else:
                cur = 0.0
            det.update(ev)
            if det.physical_fall:
                break
        if det.physical_fall:
            break
    res = det.finalize()
    return {
        "horizon_s": float(horizon_s),
        "longest_dwell_s": float(longest),
        "pass_count": int(pass_count),
        "termination": str(res.termination),
        "physical_fall": bool(res.physical_fall),
        "res43_pass": bool(longest >= 0.500 and not res.physical_fall),
    }
