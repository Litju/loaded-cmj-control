"""R0.1: full integration-state save/restore + branch replay + gates (§9, §11)."""
import hashlib
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(TASK_ROOT))
if str(TASK_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(TASK_ROOT / "src"))

from tools.evid_state import STATE_SPEC_INT, STATE_SPEC_NAME, capture_state_vector, restore_state_vector
from tools.evid_trace_v2 import (
    LCMJ_TRACE_SCHEMA_VERSION,
    PHYSICS_DT,
    CONTROL_DT,
    PHYSICS_FIELDS,
    expected_counts,
)
from tools.evid_bundle import harmless_action


def _fresh_plant():
    from loaded_cmj.v2.plant import V2Plant

    p = V2Plant()
    return p, p.model


def test_state_spec_and_size():
    p, m = _fresh_plant()
    assert STATE_SPEC_NAME == "mjSTATE_INTEGRATION"
    assert STATE_SPEC_INT == int(mujoco.mjtState.mjSTATE_INTEGRATION)
    assert mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_INTEGRATION) == 108
    assert (m.nq, m.nv, m.nu) == (10, 10, 7)


def test_full_state_save_restore_identity():
    p, m = _fresh_plant()
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    t = 0.0
    # evolve dynamics to a NONZERO branch point (25 control steps)
    for _ in range(25):
        u = harmless_action(p, d)
        p.apply_action(d, u)
        for _ in range(40):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
    vec = capture_state_vector(m, d)
    assert vec.shape[0] == 108
    sha = hashlib.sha256(vec.tobytes()).hexdigest()
    assert len(sha) == 64
    # warmstart tail is nonzero after evolution (proves qpos/qvel-only restore is insufficient)
    assert np.max(np.abs(vec[21:31])) > 0.0
    d2 = p.make_data()
    restore_state_vector(m, d2, vec)
    assert float(d2.time) == float(d.time)
    assert np.max(np.abs(d2.qpos - d.qpos)) == 0.0
    assert np.max(np.abs(d2.qvel - d.qvel)) == 0.0
    assert np.max(np.abs(d2.qacc_warmstart - d.qacc_warmstart)) == 0.0
    assert np.max(np.abs(d2.ctrl - d.ctrl)) == 0.0


def test_nonzero_branch_replay_identity_short():
    """End-to-end branch replay on a short horizon (fast unit version)."""
    p, m = _fresh_plant()
    d = p.make_data()
    p.reset(d)
    mujoco.mj_forward(m, d)
    t = 0.0
    for _ in range(10):  # prefix to t=0.05
        u = harmless_action(p, d)
        p.apply_action(d, u)
        for _ in range(40):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
    assert t > 0.0
    vec = capture_state_vector(m, d)
    actions = []
    for _ in range(10):  # continuation 10 control steps
        u = harmless_action(p, d)
        actions.append(u.copy())
        p.apply_action(d, u)
        for _ in range(40):
            mujoco.mj_step(m, d)
            t += PHYSICS_DT
    qpos_o, qvel_o = d.qpos.copy(), d.qvel.copy()
    d2 = p.make_data()
    restore_state_vector(m, d2, vec)
    t2 = float(d2.time)
    for u in actions:
        p.apply_action(d2, u)
        for _ in range(40):
            mujoco.mj_step(m, d2)
            t2 += PHYSICS_DT
    assert np.max(np.abs(d2.qpos - qpos_o)) == 0.0
    assert np.max(np.abs(d2.qvel - qvel_o)) == 0.0


def test_trace_sample_count_consistency():
    n, mc = expected_counts(0.5)
    assert (n, mc) == (4000, 100)
    n2, mc2 = expected_counts(1.0)
    assert (n2, mc2) == (8000, 200)


def test_trace_schema_completeness():
    required = {"time", "qpos", "qvel", "qacc", "ctrl", "qfrc_actuator", "qfrc_passive",
                "qfrc_constraint", "root_pos", "root_vel", "com", "com_vel", "Hy",
                "trunk_tilt", "trunk_angvel", "left_Fz", "contact_geom1", "contact_dist",
                "contact_count", "efc_type", "efc_id", "efc_force", "support_margin",
                "actuator_torque", "controller_phase", "fall_flag", "prohibited_flag",
                "reflight_flag"}
    assert required.issubset(set(PHYSICS_FIELDS))
    assert LCMJ_TRACE_SCHEMA_VERSION == 2


def test_authority_gates_present():
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(TASK_ROOT)).decode().strip()
    tree = subprocess.check_output(["git", "rev-parse", "HEAD^{tree}"], cwd=str(TASK_ROOT)).decode().strip()
    assert len(sha) == 40 and len(tree) == 40
    assert mujoco.__version__ == "3.8.0"
    import hashlib as _h

    model_hash = _h.sha256((TASK_ROOT / "src/loaded_cmj/v2/assets/v2_plant.xml").read_bytes()).hexdigest()
    assert model_hash == "5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191"
