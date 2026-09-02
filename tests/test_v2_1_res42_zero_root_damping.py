"""RES-42 durable tests for zero world-anchored root damping — 18 checks.

Covers:
1. root_tx damping = 0
2. root_tz damping = 0
3. root_ry damping = 0
4. all root limits absent
5. all root stiffness = 0
6. all root armature = 0
7. all root frictionloss = 0
8. zero root qfrc_passive during standing
9. zero root qfrc_passive during dynamic replay
10. mass/inertia identity (95 kg)
11. topology identity (nq/nv/nu etc)
12. actuator identity
13. contact identity (solref/solimp/friction)
14. standing physical support
15. forceplate contract
16. physical fall behavior
17. no nonfinite state
18. no replacement hidden root constraint
"""

import sys
from pathlib import Path
import mujoco
import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(TASK_ROOT / "src"))

from loaded_cmj.v2.constants import (
    V2_TOTAL_MASS_KG,
    V2_PHYSICS_TIMESTEP_S,
    V2_CONTROL_PERIOD_S,
    V2_CONTACT_SOLREF,
    V2_CONTACT_SOLIMP,
    V2_FRICTION_FOOT,
    V2_FRICTION_FLOOR,
    V2_TORQUE_LIMITS_NM,
)
from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2 import controller as ctrl

DT = V2_PHYSICS_TIMESTEP_S
SUBSTEPS = int(V2_CONTROL_PERIOD_S / DT)
TX_DOF = 0
TZ_DOF = 1
RY_DOF = 2

def _plant():
    return V2Plant()

def test_root_tx_damping_zero():
    p = _plant()
    m = p.model
    vadr = p.idx.vadr["root_tx"]
    assert float(m.dof_damping[vadr]) == 0.0

def test_root_tz_damping_zero():
    p = _plant()
    m = p.model
    vadr = p.idx.vadr["root_tz"]
    assert float(m.dof_damping[vadr]) == 0.0

def test_root_ry_damping_zero():
    p = _plant()
    m = p.model
    vadr = p.idx.vadr["root_ry"]
    assert float(m.dof_damping[vadr]) == 0.0

def test_all_root_limits_absent():
    p = _plant()
    m = p.model
    for name in ["root_tx","root_tz","root_ry"]:
        jid = p.idx.joint[name]
        assert int(m.jnt_limited[jid]) == 0
        # range should be [0,0] inactive
        assert np.allclose(m.jnt_range[jid], [0.0, 0.0])

def test_all_root_stiffness_zero():
    p = _plant()
    m = p.model
    for name in ["root_tx","root_tz","root_ry"]:
        jid = p.idx.joint[name]
        assert float(m.jnt_stiffness[jid]) == 0.0
        vadr = p.idx.vadr[name]
        # dof stiffness via? Actually jnt_stiffness per joint, but also check dof?
        assert float(m.jnt_stiffness[jid]) == 0.0

def test_all_root_armature_zero():
    p = _plant()
    m = p.model
    for name in ["root_tx","root_tz","root_ry"]:
        vadr = p.idx.vadr[name]
        assert float(m.dof_armature[vadr]) == 0.0

def test_all_root_frictionloss_zero():
    p = _plant()
    m = p.model
    for name in ["root_tx","root_tz","root_ry"]:
        vadr = p.idx.vadr[name]
        assert float(m.dof_frictionloss[vadr]) == 0.0

def test_zero_root_qfrc_passive_during_standing():
    p = _plant()
    m = p.model
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    LIMITS = np.array([250,250,250,300,300,200,200], dtype=float)
    HOLD_Q = np.zeros(7)
    KP = np.array([400,400,400,400,400,400,400], dtype=float)
    KD = np.array([10,10,10,10,10,10,10], dtype=float)
    t = 0.0
    for step in range(int(2.0 / V2_CONTROL_PERIOD_S)):
        s = np.array([float(d.qpos[p.idx.qadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        sd = np.array([float(d.qvel[p.idx.vadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        err = HOLD_Q - s
        raw = KP*err - KD*sd
        u = np.clip(raw / LIMITS, -1, 1)
        for i, aname in enumerate(["m_lumbar","m_left_hip","m_right_hip","m_left_knee","m_right_knee","m_left_ankle","m_right_ankle"]):
            d.ctrl[p.idx.actuator[aname]] = float(u[i])
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
            t += DT
            qf = np.asarray(d.qfrc_passive)
            assert float(qf[TX_DOF]) == 0.0
            assert float(qf[TZ_DOF]) == 0.0
            assert float(qf[RY_DOF]) == 0.0
            # also damper should be zero
            qd = np.asarray(d.qfrc_damper)
            assert float(qd[TX_DOF]) == 0.0
            assert float(qd[TZ_DOF]) == 0.0
            assert float(qd[RY_DOF]) == 0.0

def test_zero_root_qfrc_passive_during_dynamic_replay():
    p = _plant()
    m = p.model
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    t = 0.0
    prev = np.zeros(7)
    for step_idx in range(int(2.0 / V2_CONTROL_PERIOD_S)):
        obs = p.public_observation(d, scored_time_s=t, step_index=step_idx, episode_reset=(step_idx==0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=float)
        p.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
            t += DT
            qf = np.asarray(d.qfrc_passive)
            assert float(qf[TX_DOF]) == 0.0, f"tx passive {qf[TX_DOF]} at t {t}"
            assert float(qf[TZ_DOF]) == 0.0, f"tz passive {qf[TZ_DOF]} at t {t}"
            assert float(qf[RY_DOF]) == 0.0, f"ry passive {qf[RY_DOF]} at t {t}"

def test_mass_inertia_identity():
    p = _plant()
    m = p.model
    assert abs(float(m.body_mass.sum()) - 95.0) < 1e-9
    assert abs(float(m.body_mass.sum()) - V2_TOTAL_MASS_KG) < 1e-9
    expected_mass = {
        "pelvis": 10.65,
        "torso_head_arms": 40.20,
        "external_load": 20.0,
        "left_thigh": 7.5,
        "left_shank": 3.4875,
        "left_foot": 1.0875,
        "right_thigh": 7.5,
        "right_shank": 3.4875,
        "right_foot": 1.0875,
    }
    for name, mass in expected_mass.items():
        bid = p.idx.body[name]
        assert abs(float(m.body_mass[bid]) - mass) < 1e-9
    # inertia check for pelvis
    bid = p.idx.body["pelvis"]
    assert np.allclose(m.body_inertia[bid], [0.086975, 0.040115, 0.09230])

def test_topology_identity():
    p = _plant()
    m = p.model
    assert int(m.nq) == 10
    assert int(m.nv) == 10
    assert int(m.nu) == 7
    assert int(m.nbody) == 10
    assert int(m.ngeom) == 16
    assert int(m.njnt) == 10

def test_actuator_identity():
    p = _plant()
    m = p.model
    for name, limit in V2_TORQUE_LIMITS_NM.items():
        act_name = f"m_{name}"
        aid = p.idx.actuator[act_name]
        lo, hi = m.actuator_forcerange[aid]
        assert abs(lo + limit) < 1e-9
        assert abs(hi - limit) < 1e-9

def test_contact_identity():
    p = _plant()
    m = p.model
    assert np.allclose(m.geom_solref[p.idx.floor_geom], V2_CONTACT_SOLREF)
    assert np.allclose(m.geom_solref[p.idx.left_foot_geom], V2_CONTACT_SOLREF)
    assert np.allclose(m.geom_solimp[p.idx.floor_geom], V2_CONTACT_SOLIMP)
    assert np.allclose(m.geom_solimp[p.idx.left_foot_geom], V2_CONTACT_SOLIMP)
    assert np.allclose(m.geom_friction[p.idx.floor_geom][:3], V2_FRICTION_FLOOR)
    assert np.allclose(m.geom_friction[p.idx.left_foot_geom][:3], V2_FRICTION_FOOT)

def test_standing_physical_support():
    p = _plant()
    m = p.model
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    LIMITS = np.array([250,250,250,300,300,200,200], dtype=float)
    HOLD_Q = np.zeros(7)
    KP = np.array([400,400,400,400,400,400,400], dtype=float)
    KD = np.array([10,10,10,10,10,10,10], dtype=float)
    t = 0.0
    for step in range(int(2.0 / V2_CONTROL_PERIOD_S)):
        s = np.array([float(d.qpos[p.idx.qadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        sd = np.array([float(d.qvel[p.idx.vadr[n]]) for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]])
        err = HOLD_Q - s
        raw = KP*err - KD*sd
        u = np.clip(raw / LIMITS, -1, 1)
        for i, aname in enumerate(["m_lumbar","m_left_hip","m_right_hip","m_left_knee","m_right_knee","m_left_ankle","m_right_ankle"]):
            d.ctrl[p.idx.actuator[aname]] = float(u[i])
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
            t += DT
    summary = p.foot_contact_summary(d)
    assert summary["left_Fz"] > 10
    assert summary["right_Fz"] > 10
    assert abs(summary["whole_Fz"] - 931.95) < 5.0
    assert summary["support_margin"] > 0
    assert not summary["prohibited_contact"]
    assert np.all(np.isfinite(d.qpos))
    # No fall
    fall_geoms = [p.idx.geom[n] for n in ["pelvis_fall","torso_fall","left_thigh_fall","right_thigh_fall","left_shank_fall","right_shank_fall"]]
    floor = p.idx.floor_geom
    fall_contact = False
    for i in range(d.ncon):
        con = d.contact[i]
        g1, g2 = int(con.geom1), int(con.geom2)
        if g1 == floor or g2 == floor:
            other = g2 if g1 == floor else g1
            if other in fall_geoms:
                fall_contact = True
    assert not fall_contact

def test_forceplate_contract():
    p = _plant()
    m = p.model
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    t = 0.0
    prev = np.zeros(7)
    for step_idx in range(int(1.0 / V2_CONTROL_PERIOD_S)):
        obs = p.public_observation(d, scored_time_s=t, step_index=step_idx, episode_reset=(step_idx==0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=float)
        p.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
            t += DT
            summary = p.foot_contact_summary(d)
            assert abs(summary["whole_Fz"] - (summary["left_Fz"] + summary["right_Fz"])) < 1e-6
            # CoP valid when Fz >20
            if summary["left_Fz"] > 20:
                assert summary["left_cop_valid"]
            if summary["right_Fz"] > 20:
                assert summary["right_cop_valid"]
            # fall not counted
            # Check that fall geoms not in plantar
            fall = {p.idx.geom[n] for n in ["pelvis_fall","torso_fall","left_thigh_fall","right_thigh_fall","left_shank_fall","right_shank_fall"]}
            foot = {p.idx.left_foot_geom, p.idx.right_foot_geom}
            assert fall.isdisjoint(foot)

def test_physical_fall_behavior():
    p = _plant()
    # Verify fall shells exist and are contype 4
    for name in ["pelvis_fall","torso_fall","left_thigh_fall","right_thigh_fall","left_shank_fall","right_shank_fall"]:
        gid = p.idx.geom[name]
        # contype should be 4, conaffinity 0
        assert int(p.model.geom_contype[gid]) == 4
        assert int(p.model.geom_conaffinity[gid]) == 0
    # Verify shells are non-plantar
    fall = {p.idx.geom[n] for n in ["pelvis_fall","torso_fall","left_thigh_fall","right_thigh_fall","left_shank_fall","right_shank_fall"]}
    foot = {p.idx.left_foot_geom, p.idx.right_foot_geom}
    assert fall.isdisjoint(foot)

def test_no_nonfinite_state():
    p = _plant()
    m = p.model
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    ctrl.reset(0.0)
    t = 0.0
    prev = np.zeros(7)
    for step_idx in range(int(8.0 / V2_CONTROL_PERIOD_S)):
        obs = p.public_observation(d, scored_time_s=t, step_index=step_idx, episode_reset=(step_idx==0), previous_action=prev)
        act = np.asarray(ctrl.act(obs), dtype=float)
        p.apply_action(d, act)
        prev = act.copy()
        for _ in range(SUBSTEPS):
            mujoco.mj_step(m, d)
            t += DT
            assert np.all(np.isfinite(d.qpos))
            assert np.all(np.isfinite(d.qvel))
            assert np.all(np.isfinite(d.qacc))

def test_no_replacement_hidden_root_constraint():
    p = _plant()
    m = p.model
    # Ensure no equality constraints
    assert int(m.neq) == 0
    assert int(m.na) == 0
    # Ensure root has no spring, no friction, no viscosity via body
    for name in ["root_tx","root_tz","root_ry"]:
        jid = p.idx.joint[name]
        # stiffness zero (already)
        assert float(m.jnt_stiffness[jid]) == 0.0
        vadr = p.idx.vadr[name]
        assert float(m.dof_damping[vadr]) == 0.0
        assert float(m.dof_armature[vadr]) == 0.0
        assert float(m.dof_frictionloss[vadr]) == 0.0
    # Check that no other passive body damping added via tendon etc
    assert int(m.ntendon) == 0
    # Check actuator still bounded (no hidden drive)
    for aname in ["m_lumbar","m_left_hip","m_right_hip","m_left_knee","m_right_knee","m_left_ankle","m_right_ankle"]:
        aid = p.idx.actuator[aname]
        # ensure ctrllimited and forcelimited true
        assert int(m.actuator_ctrllimited[aid]) == 1
        assert int(m.actuator_forcelimited[aid]) == 1
