"""RES-74 deterministic tests: corrected stable recovery E11/RR -> E12.
MISSION=RES10_SYNC_STABLE_RECOVERY_E11_TO_E12_001
EXPERIMENT_ID=EXP-RES10-SYNC-STABLE-RECOVERY-E11-E12-001
"""
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np

REPO = Path("/home/litju/Projects/loaded-cmj-control")
WORK = Path("/tmp/opencode/res74/work")
EVID = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-STABLE-RECOVERY-E11-E12-001")
EVID73 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-BALANCE-CAPTURE-E10-E11-001")

S_RR_CONF_SHA = "02559b91904d7103177bae29761f65b48d7b4e9db69cd3e6c886805422766a06"
S_E11_SHA = "105e66d23e3e55602240d2195ee0e94e77c3a5cec16a72805ad646fd4f1a6d27"
S_RR_SHA = "846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710"
HANDOFF_SPEC_SHA = "7bb37234fdd6dc5521678bd06acbbc290e4cec348ca3213109107f06b66126f2"


def _handoff_spec():
    return json.loads((WORK / "stand_handoff_ready_spec.json").read_text())


def test_01_rr_confirmed_start():
    vec = np.load(WORK / "S_RECOVERY_READY_CONFIRMED_vector.npy")
    assert hashlib.sha256(np.ascontiguousarray(vec).tobytes()).hexdigest() == S_RR_CONF_SHA
    auth = json.loads((WORK / "RECOVERY_START_AUTHORITY.json").read_text())
    assert auth["S_E11_REPLAY"] == "BIT_IDENTICAL"
    assert auth["S_RR_ENTRY_REPLAY"] == "BIT_IDENTICAL"
    assert abs(auth["E11_CONFIRMED_AT"] - 1.1435000000000461) < 1e-9
    assert abs(auth["RECOVERY_READY_AT"] - 1.2369999999999943) < 1e-9


def test_02_e12_res43_authority():
    from loaded_cmj.v2.constants import V2_EVENT_THRESHOLDS, V2_TRUE_STANDING_ENVELOPE
    from loaded_cmj.v2.events import WIN, DWELL_S
    assert abs(float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_DWELL_S"]) - 0.500) < 1e-12
    assert WIN["stable_recovery"] == 4000
    assert DWELL_S["stable_recovery"] == 0.5
    assert abs(float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_TILT_MAX_RAD"]) - 0.2618) < 1e-12
    # envelope one-ULP (spot check lumbar + COM-z)
    env = V2_TRUE_STANDING_ENVELOPE
    assert float(np.nextafter(env["Q_STAND_MIN"][0], -np.inf)) == env["Q_STAND_ENVELOPE"][0][0]
    assert float(np.nextafter(env["COM_Z_STAND_MAX"], np.inf)) == env["COM_Z_STAND_ENVELOPE"][1]
    # RES43 hold identity in controller
    from loaded_cmj.v2 import stable_recovery as SR
    assert list(SR.KP) == [400.0] * 7 and list(SR.KD) == [10.0] * 7
    assert list(SR.LIMITS) == [250.0, 250.0, 250.0, 300.0, 300.0, 200.0, 200.0]


def test_03_handoff_spec_immutable():
    spec = _handoff_spec()
    assert spec["SPEC_SHA256"] == HANDOFF_SPEC_SHA
    blob = json.dumps({k: v for k, v in spec.items() if k != "SPEC_SHA256"}, indent=2, sort_keys=True) + "\n"
    assert hashlib.sha256(blob.encode()).hexdigest() == HANDOFF_SPEC_SHA


def test_04_fixed_foot_path():
    man = json.loads((WORK / "RECOVERY_MANIFOLD_AUTHORITY.json").read_text())
    assert len(man["order"]) == 13
    # lam=0 is the corrected confirmed start (full qpos), not historical E11
    import sys
    sys.path.insert(0, str(REPO / "src"))
    from loaded_cmj.v2.plant import V2Plant
    import mujoco
    plant = V2Plant()
    vec = np.load(WORK / "S_RECOVERY_READY_CONFIRMED_vector.npy")
    d = plant.make_data()
    mujoco.mj_setState(plant.model, d, np.ascontiguousarray(vec), mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(plant.model, d)
    qpos0 = np.asarray(d.qpos, float)
    lam0 = np.asarray(man["nodes"]["0.0"]["x"], float)
    qadr = [int(plant.idx.qadr[n]) for n in (["root_tx", "root_tz", "root_ry", "lumbar",
            "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"])]
    expect = np.array([qpos0[a] for a in qadr], float)
    # lam0 is solved to symmetric-mean feet (9um from raw start spots)
    assert np.max(np.abs(lam0 - expect)) < 1e-4
    # lam=1 is translated standing (q7 ~ 0), feet unchanged
    lam1 = man["nodes"]["1.0"]
    assert max(abs(v) for v in lam1["info"]["q7"]) < 2e-3
    assert lam1["info"]["foot_err"] <= 0.001
    # monotonic COM rise + trunk decrease
    coms = [man["nodes"][str(k)]["info"]["com"] for k in man["order"]]
    assert all(b[1] >= a[1] - 1e-9 for a, b in zip(coms, coms[1:]))
    # every node qualified
    assert all(man["nodes"][str(k)]["qual"]["qualified"] for k in man["order"])


def test_05_no_naive_interpolation():
    from loaded_cmj.v2 import stable_recovery as SR
    src = inspect.getsource(SR)
    # controller carries no historical joint/root coordinates
    for tok in ("0.64763", "0.32158", "0.42263", "0.85526"):
        assert tok not in src
    # path comes from the frozen manifold file, tracked per node
    assert "manifold" in src and "target_at" in src


def test_06_pause_no_jump():
    from loaded_cmj.v2 import stable_recovery as SR
    src = inspect.getsource(SR.StableRecoveryController.step)
    assert "pauses" in src and "min(1.0" in src
    assert "HANDOFF" in src and "handoff_ok_since" in src
    # one-way switch only (no return path from HANDOFF)
    assert src.count('self.mode = "HANDOFF"') == 1


def test_07_timescaling_identity():
    tsc = json.loads((WORK / "RECOVERY_TIME_SCALING_AUTHORITY.json").read_text())
    assert 1.0 <= float(tsc["boundary_lo"]) <= float(tsc["boundary_hi"]) <= 16.0
    assert abs(float(tsc["T_RISE"]) - 6.375) < 1e-9
    assert "bisection" in tsc["method"] or "bracketing" in tsc["method"]


def test_08_full_r7_no_extdir():
    from loaded_cmj.v2 import stable_recovery as SR
    src = inspect.getsource(SR)
    assert "EXT_DIR" not in src
    assert "G = np.zeros((22, 7))" in src
    assert "lsq_linear" in src


def test_09_authorities():
    import sys
    sys.path.insert(0, str(REPO / "src"))
    from loaded_cmj.v2 import plant as PM
    assert "mj_jacSubtreeCom" in inspect.getsource(PM.V2Plant.center_of_mass_velocity)
    from loaded_cmj.v2.support_continuity import load_spec
    spec = load_spec(REPO / "support_continuity_spec.json")
    assert spec["SPEC_SHA256"] == "cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734"
    from loaded_cmj.v2 import terminal_capture as TC
    assert TC.MASS_KG == 95.0 and TC.CONTROL_DT == 0.005


def test_10_no_scorer_circularity():
    from loaded_cmj.v2 import stable_recovery as SR
    body = inspect.getsource(SR)
    assert "V2EventDetector" not in body and "event_records" not in body


def test_11_no_direct_writes():
    from loaded_cmj.v2 import stable_recovery as SR
    body = inspect.getsource(SR)
    assert ".ctrl[:] =" not in body
    assert "plant.apply_action" in body
    # live state never written: mj_setState only on branch copies (probes/vprobe)
    assert body.count("mj_setState") == 2


def test_12_e12_identity_and_post_hold():
    qual = json.loads((WORK / "STABLE_RECOVERY_QUALIFICATION.json").read_text())
    assert qual["outcome"] == "PASS_E12_POST_HOLD"
    assert qual["e12_occ"] is not None and qual["e12_conf"] is not None
    # 4000-sample dwell spans 3999 physics intervals = 0.499875 s (frozen WIN
    # semantics, same as RES-43's 15200-sample/1.9 s window)
    assert qual["e12_conf"] - qual["e12_occ"] >= 0.499875 - 1e-6
    # handoff precedes confirmation (E12 dwell may straddle the one-way
    # switch since box entry coincides with envelope entry while sustain <
    # dwell; RES43 demonstrably holds through confirmation + post window)
    assert qual["handoff_t"] is not None and qual["handoff_t"] < qual["e12_conf"]
    assert qual["e12_conf"] - qual["handoff_t"] >= 0.400  # RES43-held majority
    # independent counter agrees (within one dwell of detector confirmation)
    assert qual["indep_e12t"] is not None
    assert abs(qual["indep_e12t"] - qual["e12_conf"]) < 0.500 + 1e-6
    assert qual["S_handoff_sha"] is not None and qual["S_e12_sha"] is not None
    assert qual["post_viol"] == 0
    assert qual["peak_BW"] <= 8.0 and qual["maxpen_mm"] <= 10.0
    assert qual["support"] == "QUALIFIED" and qual["ctrl_loss"] == 0
    assert qual["reflight"] == [] and qual["chatter"] <= 8
    assert qual["fall"] is False and qual["prohib"] is False


def test_13_evidence_contract():
    assert (WORK / "S_STAND_HANDOFF_vector.npy").exists()
    assert (WORK / "S_E12_CORRECTED_vector.npy").exists()
    assert (WORK / "recovery_trace_compact.npz").exists()
    assert (EVID / "checksums.sha256").exists()
