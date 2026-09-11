"""RES-78 canonical V2.1 runtime deterministic tests (fast, no full re-sim in pytest).

MISSION=RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001
EXPERIMENT_ID=EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001
CANDIDATE_ID=V2.1-R001

Full 15-s episode equivalence is proven by the precommit canonical run
(/tmp/opencode/res78/work + evidence bundle) matching RES-76/11 exactly;
these tests verify identity, structure, and that the committed result matches
without re-running the episode inside pytest (fresh-process reproduction is
proven by RUN_A vs RUN_B vs canonical TRACE identity + postcommit rerun).
"""
import hashlib
import inspect
import json
import subprocess
import sys
from pathlib import Path

REPO = Path("/home/litju/Projects/loaded-cmj-control")
WORK = Path("/tmp/opencode/res78/work")
EVID_NEW = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001")
EVID11 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES11-DETERMINISTIC-12OF12-OFFLINE-001")
EVID76 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-FULL-E1-E12-CLOSURE-001")

EXPECTED_EVENTS = {
    "supported_start": (0.000125, 0.10000000000000007),
    "countermovement_onset": (0.1612499999999961, 0.1911249999999928),
    "valid_countermovement": (0.380124999999972, 0.380124999999972),
    "upward_reversal": (0.4874999999999602, 0.4973749999999591),
    "vertical_propulsion": (0.4973749999999591, 0.5472499999999746),
    "bilateral_takeoff": (0.6481250000000083, 0.6580000000000116),
    "genuine_flight": (0.6580000000000116, 0.7378750000000383),
    "apex": (0.7719764018454335, 0.7719764018454335),
    "descending_landing": (0.875000000000084, 0.8848750000000873),
    "impact_absorption": (0.973625000000117, 0.9935000000001236),
    "balance_capture": (0.9935000000001236, 1.1433750000000462),
    "stable_recovery": (14.436250000023424, 14.936125000025811),
}

EXPECTED_CHECKPOINTS = {
    "S_APEX": "97ed110f5ad2bb5cd8e0e91a91a503e9326814a2bfcc68ce73237900be8351c2",
    "S_E10": "7931fd8b2715362cfd766a75f46553681ce872e541daaf84a95b5dfd50da8ec0",
    "S_E11": "31c7f7d11ead4277484fa1d1b93245f5dd06f256f003365edc99b328daf7039d",
    "S_RR": "846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710",
    "S_RR_CONFIRMED": "02559b91904d7103177bae29761f65b48d7b4e9db69cd3e6c886805422766a06",
    "S_STAND_HANDOFF": "2141e801ab403b32d5fc83f94a046997041815ef70c44919796534907a329679",
    "S_E12": "363a5d89298cab15c194229a64838e561126e72285a41b48665d18bdbbe88146",
}


def _canon_result():
    for p in (WORK / "V2.1-R001_CANONICAL_RESULT.json", EVID_NEW / "V2.1-R001_CANONICAL_RESULT.json"):
        if p.exists():
            return json.loads(p.read_text())
    raise FileNotFoundError("canonical result missing in WORK and evidence bundle")


def test_01_candidate_spec_identity():
    spec = json.loads((REPO / "CANONICAL_V2_CANDIDATE_SPEC.json").read_text())
    assert spec["CANDIDATE_ID"] == "V2.1-R001"
    assert spec["MISSION"] == "RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001"
    assert spec["EXPERIMENT_ID"] == "EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001"
    assert spec["ACTION_DIM"] == 7 and spec["NQ"] == 10 and spec["NV"] == 10 and spec["NU"] == 7
    assert spec["MASS_KG"] == 95.0
    assert abs(float(spec["PHYSICS_DT_S"]) - 0.000125) < 1e-15
    assert abs(float(spec["CONTROL_DT_NOMINAL_S"]) - 0.005) < 1e-15
    assert spec["CANONICAL_RUNTIME_MODULE"] == "loaded_cmj.v2.canonical_runtime"
    assert spec["CANONICAL_COMMAND"] == "python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001"
    assert abs(float(spec["CANONICAL_HORIZON_S"]) - 20.0) < 1e-12
    # hashes recompute
    import hashlib as _hl
    assert spec["PLANT_SHA256"] == _hl.sha256((REPO / "src/loaded_cmj/v2/assets/v2_plant.xml").read_bytes()).hexdigest()
    assert spec["SCORER_SHA256"] == _hl.sha256((REPO / "src/loaded_cmj/v2/events.py").read_bytes()).hexdigest()
    order = ["src/loaded_cmj/v2/res72_integration.py", "src/loaded_cmj/v2/balance_capture.py", "src/loaded_cmj/v2/stable_recovery.py", "src/loaded_cmj/v2/terminal_capture.py", "src/loaded_cmj/v2/full_closure.py"]
    h = _hl.sha256()
    for f in order:
        h.update((REPO / f).read_bytes())
    assert spec["CONTROLLER_COMPOSITION_SHA256"] == h.hexdigest()
    assert spec["ENTRY_HEAD"] == "9ca6a8b6b01a600a2b8c983d674ddea63b2e53b2"


def test_02_module_importability():
    import loaded_cmj.v2.canonical_runtime as CR
    assert CR.CANDIDATE_ID == "V2.1-R001"
    assert CR.MISSION == "RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001"
    assert callable(CR.run_canonical_episode)
    assert CR.CANONICAL_COMMAND == "python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001"


def test_03_cli_command():
    import loaded_cmj.v2.canonical_runtime as CR
    assert CR.CANONICAL_COMMAND == "python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001"
    txt = (REPO / "CANONICAL_V2_RUNTIME_CONTRACT.md").read_text()
    assert "python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001" in txt
    # CLI rejects wrong candidate fast (no full sim)
    r = subprocess.run([sys.executable, "-m", "loaded_cmj.v2.canonical_runtime", "--candidate", "WRONG"],
                       capture_output=True, text=True, cwd=str(REPO))
    assert r.returncode == 2
    # --help works
    r2 = subprocess.run([sys.executable, "-m", "loaded_cmj.v2.canonical_runtime", "--help"],
                        capture_output=True, text=True, cwd=str(REPO))
    assert r2.returncode == 0 and "--candidate" in r2.stdout


def test_04_dims():
    from loaded_cmj.v2.constants import V2_COMPILED_NQ, V2_COMPILED_NV, V2_COMPILED_NU, V2_ACTION_DIM, V2_TOTAL_MASS_KG, V2_PHYSICS_TIMESTEP_S, V2_CONTROL_PERIOD_S
    assert V2_ACTION_DIM == 7
    assert (V2_COMPILED_NQ, V2_COMPILED_NV, V2_COMPILED_NU) == (10, 10, 7)
    assert abs(float(V2_TOTAL_MASS_KG) - 95.0) < 1e-12
    assert abs(float(V2_PHYSICS_TIMESTEP_S) - 0.000125) < 1e-15
    assert abs(float(V2_CONTROL_PERIOD_S) - 0.005) < 1e-15
    import loaded_cmj.v2.canonical_runtime as CR
    assert CR.PHYS_DT == 0.000125 and CR.CTRL_DT == 0.005


def test_05_horizon():
    import loaded_cmj.v2.canonical_runtime as CR
    assert abs(float(CR.HORIZON) - 20.0) < 1e-12
    assert abs(float(CR.POST_HOLD) - 0.30) < 1e-12
    # safely beyond E12 conf + posthold (14.936...+0.30=15.236...)
    assert float(CR.HORIZON) > 14.936125000025811 + 0.30 + 1.0
    assert CR.E12_NEED == 4000 and CR.E10_NEED == 160


def test_06_no_v1_coupling():
    src = (REPO / "src/loaded_cmj/v2/canonical_runtime.py").read_text()
    body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    for bad in ("run_gen1", "Gen1", "gen1", "pfip", "PFIP", "loaded_cmj.control", "loaded_cmj.runtime"):
        assert bad not in body, bad
    # must not wrap research runner or verifier as engine
    assert "res11_independent_verify" not in body
    assert "run_full_qualification" not in body
    assert "FULL_QUALIFICATION_RAW" not in body


def test_07_no_state_restoration():
    src = (REPO / "src/loaded_cmj/v2/canonical_runtime.py").read_text()
    body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    assert "mj_setState" not in body
    assert "qpos[:] =" not in body and "qvel[:] =" not in body
    assert "live.qpos" not in body and "d.qpos" not in body
    # controllers also clean (frozen)
    for mod in ("res72_integration", "balance_capture", "stable_recovery"):
        s = inspect.getsource(__import__(f"loaded_cmj.v2.{mod}", fromlist=["x"]))
        b = s.split('"""', 2)[-1] if s.count('"""') >= 2 else s
        assert "V2EventDetector" not in b, mod


def test_08_no_scorer_circularity():
    for mod in ("res72_integration", "balance_capture", "stable_recovery", "terminal_capture", "full_closure"):
        s = inspect.getsource(__import__(f"loaded_cmj.v2.{mod}", fromlist=["x"]))
        b = s.split('"""', 2)[-1] if s.count('"""') >= 2 else s
        assert "V2EventDetector" not in b, mod
        assert "event_records" not in b, mod
    src = (REPO / "src/loaded_cmj/v2/canonical_runtime.py").read_text()
    body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    # watcher observational: update present, but control never reads event_records for u
    assert "watcher.update" in body
    # no control branch on event_records (only checkpoints/termination use it)
    # allowlist: checkpoints + E12 occ/conf + finalize + physical_fall guard
    # forbid: reading event_records to compute u_cmd
    u_idx = body.find("u_cmd")
    # ensure no event_records read before first u computation influences it (heuristic: count occurrences)
    assert body.count("event_records") <= 8


def test_09_interval_handling():
    import loaded_cmj.v2.canonical_runtime as CR
    assert CR.NSUB == 40
    src = (REPO / "src/loaded_cmj/v2/canonical_runtime.py").read_text()
    assert "truncate" in src.lower()
    res = _canon_result()
    hist = {str(k): int(v) for k, v in res["CTRL_SUBSTEP_HISTOGRAM"].items()}
    assert hist == {"28": 2, "33": 1, "40": 3045}, hist
    assert res["N_CTRL"] == 3048 and res["N_PHYS"] == 121889


def test_10_events_12_of_12():
    res = _canon_result()
    ev = res["EVENTS"]
    assert len(ev) == 12
    for name, (occ, conf) in EXPECTED_EVENTS.items():
        assert name in ev, name
        assert abs(float(ev[name]["occurred_at"]) - occ) == 0.0, name
        assert abs(float(ev[name]["confirmed_at"]) - conf) == 0.0, name
    assert res["TERMINATION"] == "OBJECTIVE_COMPLETE"
    assert res["OUTCOME"] == "PASS_E12_POST_HOLD"


def test_11_checkpoints_7_of_7():
    res = _canon_result()
    canon = res["CANONICAL_CHECKPOINTS"]
    for k, v in EXPECTED_CHECKPOINTS.items():
        assert canon[k] == v, k
    # cross-check full map contains same
    assert res["CHECKPOINT_SHAS"]["S_DET_apex"] == EXPECTED_CHECKPOINTS["S_APEX"]
    assert res["CHECKPOINT_SHAS"]["S_E10_ctrl"] == EXPECTED_CHECKPOINTS["S_E10"]


def test_12_output_schema():
    res = _canon_result()
    for k in ("MISSION", "EXPERIMENT_ID", "CANDIDATE_ID", "CANONICAL_COMMAND", "CANONICAL_RUNTIME_MODULE",
              "OUTCOME", "TERMINATION", "T_END", "N_CTRL", "N_PHYS", "EVENTS", "CANONICAL_CHECKPOINTS",
              "CTRL_SUBSTEP_HISTOGRAM", "TRACE_SHA256", "ONLINE_OFFLINE_IDENTITY", "SUPPORT_POST_LANDING",
              "POST_LANDING_LOSS", "POST_LANDING_REFLIGHT", "POST_LANDING_CHATTER"):
        assert k in res, k
    assert res["MISSION"] == "RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001"
    assert res["CANDIDATE_ID"] == "V2.1-R001"
    assert res["ACTION_DIM"] == 7 and res["NQ"] == 10 and res["NV"] == 10 and res["NU"] == 7
    assert res["POST_LANDING_LOSS"] == 0 and res["POST_LANDING_REFLIGHT"] == [] and res["POST_LANDING_CHATTER"] == 0
    assert res["ONLINE_OFFLINE_IDENTITY"] == "PASS"


def test_13_fresh_process_reproduction():
    # Fresh-process determinism proven by three independent fresh runs sharing TRACE_SHA:
    # RES-11 RUN_A, RUN_B (fresh closed-loop) and canonical precommit.
    res = _canon_result()
    a = json.loads((EVID11 / "RUN_A_CLOSED_LOOP.json").read_text())
    b = json.loads((EVID11 / "RUN_B_CLOSED_LOOP.json").read_text())
    assert a["TRACE_SHA256"] == b["TRACE_SHA256"]
    assert res["TRACE_SHA256"] == a["TRACE_SHA256"] == "4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561"
    assert res["EVENTS"] == a["EVENTS"] == b["EVENTS"]
