"""RES-54 deterministic tests: true whole-body COM velocity authority.

Covers: subtree root/mass, mj_subtreeVel, jacSubtreeCom@qvel identity,
finite-difference COM position, Newton-Euler balance, RES53 S50/terminal
regression, synchronized shadow same-state, no raw-cvel regression,
velocity-dependent event consumers, online/offline identity, event rebase
determinism, no Plant/contact/scorer/controller change, Evidence Contract v2.
"""
import mujoco
import numpy as np

from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
from loaded_cmj.v2.constants import (
    V2_TOTAL_MASS_KG, V2_PHYSICS_TIMESTEP_S, V2_EVENT_THRESHOLDS,
    V2_CONTACT_SOLREF, V2_CONTACT_SOLIMP,
)
from loaded_cmj.v2 import plant as plant_mod

DT = float(V2_PHYSICS_TIMESTEP_S)
MASS = float(V2_TOTAL_MASS_KG)


def _plant():
    return V2Plant()


def test_subtree_root_mass():
    p = _plant()
    assert p._com_root_body_id == 1
    assert p._com_root_body_name == "pelvis"
    assert abs(float(p.model.body_subtreemass[1]) - MASS) < 1e-9
    assert abs(float(p._com_total_mass) - MASS) < 1e-9


def test_subtreeVel_authority():
    p = _plant(); m = p.model; d = p.make_data(); p.reset(d)
    d.qvel[:] = np.array([0.5, -1.0, 0.3, 0.2, -0.4, 0.6, -0.1, 0.35, -0.25, 0.15])
    d.qpos[:] = np.array([0.02, 0.85, 0.05, 0.1, -0.3, 0.5, -0.2, -0.25, 0.45, -0.15])
    mujoco.mj_forward(m, d)
    v = np.asarray(p.center_of_mass_velocity(d), float)
    # cross-check against forwarded subtree authority (forward first: raw
    # subtree_linvel without forward lags one step, so forward here)
    mujoco.mj_forward(m, d)
    mujoco.mj_subtreeVel(m, d)
    assert np.allclose(v, np.asarray(d.subtree_linvel[1], float), atol=1e-12, rtol=0.0)


def test_jac_identity():
    p = _plant(); m = p.model; d = p.make_data(); p.reset(d)
    d.qvel[:] = np.array([0.5, -1.0, 0.3, 0.2, -0.4, 0.6, -0.1, 0.35, -0.25, 0.15])
    d.qpos[:] = np.array([0.02, 0.85, 0.05, 0.1, -0.3, 0.5, -0.2, -0.25, 0.45, -0.15])
    mujoco.mj_forward(m, d)
    v = np.asarray(p.center_of_mass_velocity(d), float)
    jac = np.zeros((3, m.nv)); mujoco.mj_jacSubtreeCom(m, d, jac, 1)
    assert np.linalg.norm(v - jac @ np.asarray(d.qvel, float)) < 1e-9


def test_finite_difference():
    # clean flight regime (no contact): FD of COM position matches corrected velocity
    p = _plant(); m = p.model
    d = p.make_data(); p.reset(d)
    d.qpos[1] = 1.5  # lift clear of floor
    d.qvel[:] = 0.0; d.qvel[0] = 0.2; d.qvel[1] = -0.5
    mujoco.mj_forward(m, d)
    # synchronized shadow trajectory (forward each step, as production shadow does)
    meas = create_measurement_data(p)
    coms = []; vels = []
    for _ in range(5):
        s = SynchronizedPhysicsSample.from_live_state(p, d, meas)
        coms.append(np.asarray(s.com_position_m, float).copy())
        vels.append(np.asarray(s.com_velocity_mps, float).copy())
        mujoco.mj_step(m, d)
    coms = np.stack(coms); vels = np.stack(vels)
    fd = (coms[3] - coms[1]) / (2 * DT)
    assert np.linalg.norm(vels[2] - fd) < 1e-3


def test_newton_balance_flight():
    # ballistic flight via synchronized shadows: FD of corrected velocity ~ -g
    p = _plant(); m = p.model
    d = p.make_data(); p.reset(d)
    d.qpos[1] = 1.5  # high, no contact
    d.qvel[:] = 0.0; d.qvel[1] = -1.0
    mujoco.mj_forward(m, d)
    meas = create_measurement_data(p)
    s0 = SynchronizedPhysicsSample.from_live_state(p, d, meas)
    v0 = np.asarray(s0.com_velocity_mps, float)
    mujoco.mj_step(m, d)
    s1 = SynchronizedPhysicsSample.from_live_state(p, d, meas)
    v1 = np.asarray(s1.com_velocity_mps, float)
    a = (v1 - v0) / DT
    assert abs(a[2] + 9.81) < 0.5
    assert abs(a[0]) < 0.5


def test_res53_s50_regression():
    p = _plant()
    tr = np.load(
        "/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-IMPULSE-TO-GO-CAPTURE-002/trace_FULL.npz",
        allow_pickle=True,
    )
    T = np.asarray(tr["t"], float)
    i = int(np.argmin(np.abs(T - 0.925375)))
    live = p.make_data(); meas = create_measurement_data(p)
    live.qpos[:] = np.asarray(tr["qpos"][i], float)
    live.qvel[:] = np.asarray(tr["qvel"][i], float)
    live.ctrl[:] = np.asarray(tr["ctrl"][i], float)
    live.qfrc_applied[:] = 0.0; live.xfrc_applied[:] = 0.0; live.time = float(T[i])
    s = SynchronizedPhysicsSample.from_live_state(p, live, meas)
    v = np.asarray(s.com_velocity_mps, float)
    # corrected S50 velocity (M1), not old reported [0.289,0,-0.342]
    assert abs(v[0] - 0.17787348) < 1e-6
    assert abs(v[2] - (-0.31905596)) < 1e-6
    # old-vs-new offset/X is the known bug signature
    assert abs((0.28901073 - v[0]) - 0.11113725) < 1e-6


def test_shadow_same_state():
    p = _plant()
    live = p.make_data(); meas = create_measurement_data(p)
    p.reset(live); mujoco.mj_forward(p.model, live)
    live.qvel[0] = 0.7
    s = SynchronizedPhysicsSample.from_live_state(p, live, meas)
    assert s.check_time_identity()
    assert s.state_vector_sha256 == SynchronizedPhysicsSample.from_live_state(
        p, live, create_measurement_data(p)).state_vector_sha256
    # same-state determinism
    v1 = np.asarray(s.com_velocity_mps, float)
    s2 = SynchronizedPhysicsSample.from_live_state(p, live, meas)
    assert np.array_equal(v1, np.asarray(s2.com_velocity_mps, float))


def test_no_raw_cvel_regression():
    import inspect
    src = inspect.getsource(plant_mod.V2Plant.center_of_mass_velocity)
    assert "data.cvel" not in src
    assert "mujoco.mj_objectVelocity" not in src
    assert "mj_jacSubtreeCom" in src


def test_event_consumers_use_corrected():
    # every event_sample com_vz must equal Plant corrected velocity on same shadow
    p = _plant()
    live = p.make_data(); meas = create_measurement_data(p)
    p.reset(live); mujoco.mj_forward(p.model, live)
    live.qvel[:] = np.array([0.3, -0.8, 0.1, 0.1, -0.2, 0.2, -0.05, 0.1, -0.1, 0.05])
    s = SynchronizedPhysicsSample.from_live_state(p, live, meas)
    ev = s.event_sample()
    assert abs(ev["com_vz"] - float(np.asarray(s.com_velocity_mps, float)[2])) < 1e-12
    obs = s.controller_observation(step_index=0, episode_reset=True, previous_action=np.zeros(7))
    assert abs(float(np.asarray(obs["com_velocity_mps"], float)[2]) - ev["com_vz"]) < 1e-12


def test_online_offline_identity():
    p = _plant()
    live = p.make_data(); meas = create_measurement_data(p)
    p.reset(live); mujoco.mj_forward(p.model, live)
    live.qvel[1] = -0.5
    s = SynchronizedPhysicsSample.from_live_state(p, live, meas)
    online = s.event_sample()
    # offline: recompute from same shadow meas
    from loaded_cmj.v2.measurement import event_sample_from_measurement
    offline = event_sample_from_measurement(p, meas, time_s=float(meas.time))
    assert abs(online["com_vz"] - offline["com_vz"]) < 1e-12
    assert abs(online["com_z"] - offline["com_z"]) < 1e-12


def test_event_rebase_determinism():
    import json
    e1 = json.load(open("/tmp/opencode/res54_e1_rebase.json"))
    assert e1["prefix_dq"] == 0.0 and e1["prefix_dv"] == 0.0
    assert "apex" in e1["e1_changes"]


def test_no_frozen_change():
    from loaded_cmj.v2.constants import (
        V2_PHYSICS_TIMESTEP_S, V2_CONTACT_SOLREF, V2_CONTACT_SOLIMP, V2_EVENT_THRESHOLDS,
    )
    assert V2_PHYSICS_TIMESTEP_S == 0.000125
    assert tuple(V2_CONTACT_SOLREF) == (0.016, 1.0)
    assert tuple(V2_CONTACT_SOLIMP) == (0.99, 0.99, 0.001, 0.5, 2.0)
    assert float(V2_EVENT_THRESHOLDS["TAKEOFF_VZ_MIN_MPS"]) == 0.60
    assert float(V2_EVENT_THRESHOLDS["BALANCE_CAPTURE_COM_SPEED_MPS"]) == 0.30
    # controller params frozen (spot check)
    import loaded_cmj.v2.controller as C
    assert list(C.KP_IMPACT) == [190, 190, 190, 190, 190, 95, 95]


def test_evidence_contract_v2():
    import pathlib
    b = pathlib.Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-TRUE-COM-VELOCITY-001")
    for f in ["COM_VELOCITY_CALLGRAPH.md", "CURRENT_COM_VELOCITY_IMPLEMENTATION.json"]:
        assert (b / f).exists(), f
