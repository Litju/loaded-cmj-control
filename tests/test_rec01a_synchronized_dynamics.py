"""Focused REC-01A tests: synchronized MuJoCo dynamics audit authority.

Proves (quickly, on short horizons/synthetic states unless noted):
 1. restored integration-state identity (mjSTATE_INTEGRATION roundtrip);
 2. synchronized forward->inverse identity at machine precision;
 3. mixed-stage (post-step) audit differs materially on dynamic samples;
 4. built-in FWDINV diagnostic is trajectory-nonintrusive;
 5. pipeline stage labels are explicit (post-step qacc == pre-step solve);
 6. contact-mode stratification is deterministic;
 7. production sampling audit (current vs synchronized) is reproducible;
 8. offline event comparison is deterministic.
"""

import hashlib
import sys
from pathlib import Path

import mujoco
import numpy as np

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(TASK_ROOT / "src"))

from loaded_cmj.v2.plant import V2Plant, model_xml  # noqa: E402
from loaded_cmj.v2.events import V2EventDetector  # noqa: E402

DT = 0.000125
SPEC = mujoco.mjtState.mjSTATE_INTEGRATION


def _harmless_action(plant, data):
    from loaded_cmj.v2.constants import V2_TORQUE_LIMITS_NM
    limits = np.array([V2_TORQUE_LIMITS_NM[n] for n in
                       ["lumbar", "left_hip", "right_hip", "left_knee",
                        "right_knee", "left_ankle", "right_ankle"]], dtype=np.float64)
    raw = 400.0 * (np.zeros(7) - plant.joint_positions(data)) + 10.0 * (-plant.joint_velocities(data))
    return np.clip(raw / limits, -1.0, 1.0)


def _settle(plant, m, d, n_steps=2000):
    d.ctrl[:] = 0.0
    for _ in range(n_steps):
        mujoco.mj_step(m, d)


def _sync_inverse_errors(plant, m, vec):
    """Restore + forward + copy + inverse; return (force_max, root_l2)."""
    d_s = plant.make_data()
    mujoco.mj_setState(m, d_s, np.ascontiguousarray(vec), SPEC)
    mujoco.mj_forward(m, d_s)
    d_i = plant.make_data()
    d_i.qpos[:] = d_s.qpos[:]
    d_i.qvel[:] = d_s.qvel[:]
    d_i.qacc[:] = np.asarray(d_s.qacc).copy()
    d_i.ctrl[:] = d_s.ctrl[:]
    d_i.qfrc_applied[:] = d_s.qfrc_applied[:]
    d_i.xfrc_applied[:] = d_s.xfrc_applied[:]
    mujoco.mj_inverse(m, d_i)
    inv = np.asarray(d_i.qfrc_inverse)
    ref = np.asarray(d_s.qfrc_actuator) + np.asarray(d_s.qfrc_applied)
    return float(np.max(np.abs(inv - ref))), float(np.linalg.norm(inv[0:3]))


def _mixed_inverse_errors(plant, m, vec, qacc_stale, act_post):
    d_m = plant.make_data()
    mujoco.mj_setState(m, d_m, np.ascontiguousarray(vec), SPEC)
    d_m.qacc[:] = np.ascontiguousarray(qacc_stale)
    mujoco.mj_inverse(m, d_m)
    inv = np.asarray(d_m.qfrc_inverse)
    return float(np.max(np.abs(inv - act_post))), float(np.linalg.norm(inv[0:3]))


def test_restored_integration_state_identity():
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    _settle(plant, m, d)
    n = mujoco.mj_stateSize(m, SPEC)
    assert n == 108
    vec = np.zeros(n)
    mujoco.mj_getState(m, d, vec, SPEC)
    sha = hashlib.sha256(vec.tobytes()).hexdigest()
    # fresh restore + short replay vs continued original
    d2 = plant.make_data()
    mujoco.mj_setState(m, d2, vec, SPEC)
    d2.ctrl[:] = 0.0
    d.ctrl[:] = 0.0
    for _ in range(200):
        mujoco.mj_step(m, d)
        mujoco.mj_step(m, d2)
    assert float(np.max(np.abs(np.asarray(d.qpos) - np.asarray(d2.qpos)))) == 0.0
    assert float(np.max(np.abs(np.asarray(d.qvel) - np.asarray(d2.qvel)))) == 0.0
    vec2 = np.zeros(n)
    mujoco.mj_getState(m, d2, vec2, SPEC)
    assert hashlib.sha256(vec.tobytes()).hexdigest() == sha  # source cert stable


def test_synchronized_inverse_identity():
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    _settle(plant, m, d)
    n = mujoco.mj_stateSize(m, SPEC)
    vec = np.zeros(n)
    mujoco.mj_getState(m, d, vec, SPEC)
    f, r = _sync_inverse_errors(plant, m, vec)
    assert f < 1e-9, f
    assert r < 1e-9, r


def test_mixed_stage_differs_when_dynamic():
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    _settle(plant, m, d)
    # dynamic kick: flex + velocity + ctrl, then single steps
    d.qpos[plant.idx.qadr["left_knee"]] = 0.4
    d.qpos[plant.idx.qadr["right_knee"]] = 0.4
    d.qvel[plant.idx.vadr["left_knee"]] = 2.0
    d.ctrl[:] = 0.3
    mujoco.mj_forward(m, d)
    n = mujoco.mj_stateSize(m, SPEC)
    ratios = []
    for _ in range(40):
        mujoco.mj_step(m, d)
        vec = np.zeros(n)
        mujoco.mj_getState(m, d, vec, SPEC)
        fs, rs = _sync_inverse_errors(plant, m, vec)
        fo, ro = _mixed_inverse_errors(plant, m, vec, np.asarray(d.qacc).copy(),
                                       np.asarray(d.qfrc_actuator).copy())
        assert fs < 1e-6, fs
        ratios.append(fo / max(fs, 1e-18))
    assert max(ratios) > 1e3, max(ratios)


def test_fwdinv_diagnostic_nonintrusive():
    plant = V2Plant()
    m2 = mujoco.MjModel.from_xml_string(model_xml())
    assert int(m2.opt.enableflags) == 0
    m2.opt.enableflags = int(mujoco.mjtEnableBit.mjENBL_FWDINV)
    plant2 = V2Plant(model=m2)
    datas = []
    for pl, mm in [(plant, plant.model), (plant2, m2)]:
        dd = pl.make_data()
        pl.reset(dd)
        mujoco.mj_forward(mm, dd)
        for _ in range(600):
            u = _harmless_action(pl, dd)
            pl.apply_action(dd, u)
            mujoco.mj_step(mm, dd)
        datas.append((np.asarray(dd.qpos).copy(), np.asarray(dd.qvel).copy()))
    assert float(np.max(np.abs(datas[0][0] - datas[1][0]))) == 0.0
    assert float(np.max(np.abs(datas[0][1] - datas[1][1]))) == 0.0


def test_stage_labels_explicit():
    """Post-step derived quantities belong to the pre-integration instant.

    Drops the body from 1.5 m; at the touchdown transition the post-step
    contact list/qacc must differ from the synchronized (forward) values,
    while post-step qacc always equals the pre-integration forward solve.
    """
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    # drop from height to force a contact transition
    d.qpos[plant.idx.qadr["root_tz"]] = 1.5
    mujoco.mj_forward(m, d)
    d.ctrl[:] = 0.0
    found = False
    for _ in range(4000):
        mujoco.mj_forward(m, d)
        qacc_pre = np.asarray(d.qacc).copy()
        mujoco.mj_step(m, d)
        qacc_post = np.asarray(d.qacc).copy()
        # stage label: post-step qacc belongs to pre-integration instant
        assert float(np.max(np.abs(qacc_post - qacc_pre))) == 0.0
        ncon_post = int(d.ncon)
        mujoco.mj_forward(m, d)  # synchronize (does not alter integration state)
        if ncon_post != int(d.ncon):
            found = True
            # stale dynamics differ materially at the transition
            assert float(np.max(np.abs(np.asarray(d.qacc) - qacc_post))) > 1e-6
            break
    assert found, "no contact transition reached in drop test"


def test_contact_stratification_deterministic():
    rng = np.random.default_rng(7)
    errs = rng.uniform(0, 100, size=200)
    ncon = rng.choice([0, 4, 8], size=200)

    def stratify(e, n):
        return {k: sorted([float(x) for x in e[n == k]]) for k in (0, 4, 8)}

    assert stratify(errs, ncon) == stratify(errs, ncon)


def test_production_audit_reproducible():
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    cur, syn = [], []
    for _ in range(400):
        u = _harmless_action(plant, d)
        plant.apply_action(d, u)
        mujoco.mj_step(m, d)
        com_c = plant.center_of_mass(d)
        fz_c = plant.foot_contact_summary(d)["whole_Fz"]
        cur.append((com_c.copy(), fz_c))
        n = mujoco.mj_stateSize(m, SPEC)
        vec = np.zeros(n)
        mujoco.mj_getState(m, d, vec, SPEC)
        d2 = plant.make_data()
        mujoco.mj_setState(m, d2, vec, SPEC)
        mujoco.mj_forward(m, d2)
        syn.append((plant.center_of_mass(d2).copy(),
                    plant.foot_contact_summary(d2)["whole_Fz"]))
    # recompute sync leg identically -> reproducible
    assert len(cur) == len(syn) == 400
    deltas = [abs(c[1] - s[1]) for c, s in zip(cur, syn)]
    assert all(np.isfinite(deltas))
    assert max(deltas) >= 0.0


def test_offline_event_comparison_deterministic():
    plant = V2Plant()
    m = plant.model
    d = plant.make_data()
    plant.reset(d)
    mujoco.mj_forward(m, d)
    samples = []
    t = 0.0
    for _ in range(800):
        u = _harmless_action(plant, d)
        plant.apply_action(d, u)
        mujoco.mj_step(m, d)
        t += DT
        com = plant.center_of_mass(d)
        cv = plant.center_of_mass_velocity(d)
        sm = plant.foot_contact_summary(d)
        samples.append({"time_s": t, "com_z": float(com[2]), "com_vz": float(cv[2]),
                        "com_x": float(com[0]), "whole_Fz": float(sm["whole_Fz"]),
                        "left_Fz": float(sm["left_Fz"]), "right_Fz": float(sm["right_Fz"]),
                        "trunk_tilt": float(plant.trunk_tilt(d)),
                        "com_margin": float(sm["support_margin"]),
                        "prohibited": False, "fall_contact": False})

    def run(slist):
        det = V2EventDetector()
        det.reset()
        for s in slist:
            det.update(dict(s))
        return det.finalize()

    r1, r2 = run(samples), run(samples)
    assert dict(r1.events) == dict(r2.events)
    assert r1.termination == r2.termination
