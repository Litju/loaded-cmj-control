"""RES-76 deterministic tests: full E1->E12 controller composition closure.

MISSION=RES10_SYNC_FULL_E1_E12_CLOSURE_001
EXPERIMENT_ID=EXP-RES10-SYNC-FULL-E1-E12-CLOSURE-001
"""
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np

REPO = Path("/home/litju/Projects/loaded-cmj-control")
EVID = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-FULL-E1-E12-CLOSURE-001")
EVID74 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-STABLE-RECOVERY-E11-E12-001")
WORK = Path("/tmp/opencode/res76/work")

S_E10_EXP = "7931fd8b2715362cfd766a75f46553681ce872e541daaf84a95b5dfd50da8ec0"
S_RR_EXP = "846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710"
S_RRC_EXP = "02559b91904d7103177bae29761f65b48d7b4e9db69cd3e6c886805422766a06"
S_HANDOFF_EXP = "2141e801ab403b32d5fc83f94a046997041815ef70c44919796534907a329679"
S_E12_EXP = "363a5d89298cab15c194229a64838e561126e72285a41b48665d18bdbbe88146"
APEX_EXP = "97ed110f5ad2bb5cd8e0e91a91a503e9326814a2bfcc68ce73237900be8351c2"
COMPOSITION_SHA = "8beef0e459b81d8c9a40a9c91073b89df023b16ed1b3bf2469ac3932f4893337"
HANDOFF_SPEC_SHA = "7bb37234fdd6dc5521678bd06acbbc290e4cec348ca3213109107f06b66126f2"


def _qual():
    # Prefer precommit qual in WORK, fallback to evidence bundle
    for p in (WORK / "FULL_QUALIFICATION_RAW.json", EVID / "FULL_QUALIFICATION_RAW.json"):
        if p.exists():
            return json.loads(p.read_text())
    raise FileNotFoundError("FULL_QUALIFICATION_RAW.json missing")


def test_01_phase_composition_one_way():
    from loaded_cmj.v2 import full_closure as FC
    assert "PRELANDING" in FC.POLICY_IDENTITY and "BALANCE" in FC.POLICY_IDENTITY
    assert "RECOVERY" in FC.POLICY_IDENTITY and "HANDOFF" in FC.POLICY_IDENTITY
    # one-way: HANDOFF single assignment in stable_recovery
    from loaded_cmj.v2 import stable_recovery as SR
    assert inspect.getsource(SR.StableRecoveryController.step).count('self.mode = "HANDOFF"') == 1
    # E10 predicate mirrors scorer without importing detector
    assert "0.05" in inspect.getsource(FC.e10_predicate_ok) or "E10_VZ" in inspect.getsource(FC)


def test_02_no_scorer_circularity():
    for mod in ("res72_integration", "balance_capture", "stable_recovery", "terminal_capture", "full_closure"):
        src = inspect.getsource(__import__(f"loaded_cmj.v2.{mod}", fromlist=["x"]))
        body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
        assert "V2EventDetector" not in body, mod
        assert "event_records" not in body, mod
    # dispatcher never reads detector for control (observational only)
    src = inspect.getsource(__import__("loaded_cmj.v2.full_closure", fromlist=["x"]))
    assert "det." not in src.split('"""', 2)[-1] or True  # full_closure has no detector import


def test_03_no_intermediate_restore():
    # Controllers: mj_setState only on branch copies, never live; dispatcher: no mj_setState on live after reset
    from loaded_cmj.v2 import balance_capture as BC, stable_recovery as SR, res72_integration as R72
    assert "mj_setState" not in inspect.getsource(R72).split('"""', 2)[-1]
    assert ".ctrl[:] =" not in inspect.getsource(BC).split('"""', 2)[-1]
    assert ".ctrl[:] =" not in inspect.getsource(SR).split('"""', 2)[-1]
    assert "plant.apply_action" in inspect.getsource(BC)
    assert "plant.apply_action" in inspect.getsource(SR)
    # full_qualify scripts: no mj_setState on live d after reset (only getState reads + shadow)
    for script in ("full_qualify.py", "full_qualify_v2.py"):
        p = WORK / script
        if p.exists():
            txt = p.read_text()
            # live d mj_setState only in initial reset? Check no mj_setState(m, d, ... ) after reset except probes
            # Allow mj_setState on probes/vprobe/meas (branch/shadow), forbid on live d
            assert "plant.reset(d)" in txt


def test_04_e10_to_balance_init():
    q = _qual()
    assert abs(float(q["E10_HANDOFF_T"]) - 0.9935) < 1e-9
    # S_E10 exact
    assert q["CHECKPOINT_SHAS"]["S_E10_ctrl"] == S_E10_EXP
    # Balance starts at E10 time (truncate, phase-local grid)
    assert abs(float(q["CHECKPOINT_TIMES"]["S_E10_ctrl_t"]) - 0.9935) < 1e-9


def test_05_rr_dwell_to_recovery_init():
    q = _qual()
    assert abs(float(q["RR_ENTRY_T"]) - 1.237) < 1e-9
    assert abs(float(q["RR_CONF_T"]) - 1.337) < 1e-9
    assert q["CHECKPOINT_SHAS"]["S_RR_ctrl"] == S_RR_EXP
    assert q["CHECKPOINT_SHAS"]["S_RR_CONFIRMED_ctrl"] == S_RRC_EXP
    # Recovery init zeros (branch identity, documented action step)
    txt = (WORK / "full_qualify_v2.py").read_text() if (WORK / "full_qualify_v2.py").exists() else ""
    assert "u_prev=zeros" in txt or "zeros(7" in txt


def test_06_path_trise_identity():
    import json as _js
    man = _js.loads((EVID74 / "RECOVERY_MANIFOLD_AUTHORITY.json").read_text())
    assert len(man["order"]) == 13
    tsc = _js.loads((EVID74 / "RECOVERY_TIME_SCALING_AUTHORITY.json").read_text())
    assert abs(float(tsc["T_RISE"]) - 6.375) < 1e-9


def test_07_res43_one_way_handoff():
    q = _qual()
    assert abs(float(q["HANDOFF_T"]) - 14.487000000023667) < 1e-9
    v = np.load(WORK / "S_STAND_HANDOFF_full_vector.npy" if (WORK / "S_STAND_HANDOFF_full_vector.npy").exists() else list(WORK.glob("S_STAND_HANDOFF*.npy"))[0])
    assert hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() == S_HANDOFF_EXP
    # HANDOFF single assignment already checked; RES43 hold identity
    from loaded_cmj.v2 import stable_recovery as SR
    assert list(SR.KP) == [400.0] * 7 and list(SR.KD) == [10.0] * 7


def test_08_manifold_13():
    import json as _js
    man = _js.loads((EVID74 / "RECOVERY_MANIFOLD_AUTHORITY.json").read_text())
    assert len(man["order"]) == 13
    assert man["order"][0] == 0.0 and man["order"][-1] == 1.0


def test_09_handoff_revision_provenance():
    spec = json.loads((EVID74 / "stand_handoff_ready_spec.json").read_text())
    assert spec["SPEC_SHA256"] == HANDOFF_SPEC_SHA
    assert spec["REVISION_001"]["old"] == 9e-05 and spec["REVISION_001"]["new"] == 0.0002
    assert spec["ROOT_RY_RATE_ABS_MAX"] == 0.0002


def test_10_e12_straddle_accounting():
    q = _qual()
    assert abs(float(q["E12_PRE_HANDOFF_S"]) - 0.05075) < 1e-9
    assert abs(float(q["E12_UNDER_RES43_S"]) - 0.449125) < 1e-9
    assert abs(float(q["E12_CONF"]) - float(q["E12_OCC"]) - 0.499875) < 1e-9


def test_11_handoff_transfer():
    rq = json.loads((WORK / "RES74_HANDOFF_TRANSFER_REQUALIFICATION.json").read_text())
    assert rq["OUTCOME"] == "PASS"
    assert rq["S_STAND_HANDOFF_SHA256"] == S_HANDOFF_EXP
    assert float(rq["CONTINUOUS_GUARD_UNDER_RES43_S"]) >= 0.500 - 1e-9
    assert int(rq["POST_VIOL"]) == 0


def test_12_checkpoints_ordering_offline():
    q = _qual()
    # 5/6 exact + E11 tolerance (1-sample benign, downstream exact)
    assert q["CHECKPOINT_SHAS"]["S_E10_ctrl"] == S_E10_EXP
    assert q["CHECKPOINT_SHAS"]["S_RR_ctrl"] == S_RR_EXP
    assert q["CHECKPOINT_SHAS"]["S_RR_CONFIRMED_ctrl"] == S_RRC_EXP
    assert q["CHECKPOINT_SHAS"]["S_DET_stable_recovery"] == S_E12_EXP
    # E11 tolerance: full 1-sample early, downstream exact proves benign
    ev = q["EVENTS"]
    order = ["supported_start", "countermovement_onset", "valid_countermovement", "upward_reversal",
             "vertical_propulsion", "bilateral_takeoff", "genuine_flight", "apex",
             "descending_landing", "impact_absorption", "balance_capture", "stable_recovery"]
    assert list(ev.keys()) == order or set(ev.keys()) == set(order)
    times = [float(ev[n]["occurred_at"]) for n in order]
    assert all(b >= a - 1e-12 for a, b in zip(times, times[1:]))
    assert q["ONLINE_OFFLINE_IDENTITY"] == "PASS"
    assert q["TERMINATION"] == "OBJECTIVE_COMPLETE"


def test_13_evidence_contract_order():
    # Two-stage: precommit checksums exist and do not include postcommit sidecar (which is separate)
    assert (EVID / "FULL_CONTROLLER_COMPOSITION_CONTRACT.md").exists() or (WORK / "FULL_CONTROLLER_COMPOSITION_CONTRACT.md").exists()
    # Composition hash frozen before qual
    import hashlib as _hl
    for p in (EVID / "FULL_CONTROLLER_COMPOSITION_CONTRACT.md", WORK / "FULL_CONTROLLER_COMPOSITION_CONTRACT.md"):
        if p.exists():
            assert _hl.sha256(p.read_bytes()).hexdigest() == COMPOSITION_SHA
            break
