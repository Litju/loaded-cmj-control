"""RES-58 deterministic tests: seal corrected terminal momentum capture.

MISSION=RES10_SYNC_TRUE_MOMENTUM_TERMINAL_CAPTURE_S50_002
EXPERIMENT_ID=EXP-RES10-TRUE-MOMENTUM-CAPTURE-002

Covers (per RES-58 Tests section):
- exact RES-56 archived-law reconstruction;
- frozen equation/constants identity;
- no hidden tuning/candidate path;
- RES-54 COM authority usage;
- RES-55 point-velocity authority usage;
- RES-57 contract hash and semantics;
- one-control-interval terminology (NOT Nyquist period);
- exact S50 start SHA;
- 40-ms near-zero dwell;
- sample dropout retained separately from control-relevant episode;
- historical RES-56 FAIL preserved in evidence metadata;
- no Plant/contact/scorer/takeoff/E10 changes;
- Evidence Contract v2 identity.
"""
import hashlib
import inspect
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path("/home/litju/Projects/loaded-cmj-control")
SEALED52 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001")
B56 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-TRUE-MOMENTUM-CAPTURE-001")
B57 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-BILATERAL-SUPPORT-CONTINUITY-001")
B58 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-TRUE-MOMENTUM-CAPTURE-002")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools" / "res52"))

S50_SHA = "bb56c0d3df3cd50aa55be515783af5cfb24f212eaaa29d17427bd3e60c6ceae2"
ARCH_CAPTURE_SHA = "06e73896cbeab9dd04c49be56ace70f471c40bd3153e17f840e5b543c9fecd4d"
CONTRACT_SHA = "cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734"


def _arch_capture():
    sys.path.insert(0, str(B56 / "executed_source_archive" / "tools_res56"))
    import capture_law as A
    return A


def test_01_archived_law_reconstruction():
    # Archived executed source hash verified before use.
    p = B56 / "executed_source_archive" / "tools_res56" / "capture_law.py"
    h = hashlib.sha256(p.read_bytes()).hexdigest()
    assert h == ARCH_CAPTURE_SHA
    arch = json.loads((B56 / "RES56_SOURCE_ARCHIVE.json").read_text())
    assert any(f["path"] == "tools_res56/capture_law.py" and f["sha256"] == ARCH_CAPTURE_SHA for f in arch["FILES"])
    # Reusable seal matches archived behavior exactly over sweep.
    from loaded_cmj.v2 import terminal_capture as N
    A = _arch_capture()
    for vz in list(np.linspace(-0.5, 0.5, 201)) + [-0.024525, -0.024526, 0.01962, 0.019621, 0.0]:
        assert A.capture_force_command(float(vz)) == N.capture_force_command(float(vz))
        assert A.capture_vz_target_next(float(vz), A.capture_force_command(float(vz))) == N.capture_vz_target_next(float(vz), N.capture_force_command(float(vz)))


def test_02_frozen_equation_constants_identity():
    from loaded_cmj.v2 import terminal_capture as N
    assert N.MASS_KG == 95.0 and N.G == 9.81 and N.BW_N == 95.0 * 9.81
    assert N.CONTROL_DT == 0.005 and N.VZ_TARGET == 0.0
    assert N.FZ_MIN_N == 0.6 * N.BW_N and N.FZ_MAX_N == 1.5 * N.BW_N
    assert abs(N.VZ_SAT_HIGH - (-0.024525)) < 1e-12
    assert abs(N.VZ_SAT_LOW - 0.01962) < 1e-12
    assert N.LAW_IDENTITY == "FZ_RAW=BW-m*vz_true/CONTROL_DT; FZ_DES=clamp(FZ_RAW,0.6BW,1.5BW); vz_target_next=vz_true+CONTROL_DT*(FZ_DES/m-g)"
    assert N.RES56_ARCHIVED_SOURCE_SHA256 == ARCH_CAPTURE_SHA
    # Sign map (anti-RES53): vz<0 -> >=BW, vz=0 -> ==BW, vz>0 -> <=BW.
    assert N.capture_force_command(-0.3) == N.FZ_MAX_N >= N.BW_N
    assert N.capture_force_command(0.0) == N.BW_N
    assert N.capture_force_command(0.3) == N.FZ_MIN_N <= N.BW_N
    # Clamp edges exactly [0.6, 1.5] BW.
    assert N.capture_force_command(-10.0) == N.FZ_MAX_N
    assert N.capture_force_command(10.0) == N.FZ_MIN_N
    # Consistent next-step target: unsaturated band maps to 0.
    fz, vn = N.capture_command(0.01)
    assert abs(vn - (0.01 + 0.005 * (fz / 95.0 - 9.81))) < 1e-15


def test_03_no_hidden_tuning_candidate_path():
    from loaded_cmj.v2 import terminal_capture as N
    src = inspect.getsource(N)
    assert "VZ_TARGET" in src and "CONTROL_DT" in src
    # Frozen module exposes only the sealed law surface; no tuning/search API.
    allowed = {"MASS_KG", "G", "BW_N", "CONTROL_DT", "VZ_TARGET", "FZ_MIN_N", "FZ_MAX_N",
               "VZ_SAT_HIGH", "VZ_SAT_LOW", "RES56_ARCHIVED_SOURCE_SHA256", "RES56_MISSION",
               "RES58_MISSION", "LAW_IDENTITY", "capture_force_command", "capture_vz_target_next",
               "capture_command", "annotations"}
    names = [n for n in dir(N) if not n.startswith("_")]
    assert set(names) == allowed, names
    # Code body (excluding docstring prohibitions) must not execute tuning or physics writes.
    body = src.split('"""', 2)[-1] if src.count('"""') >= 2 else src
    for tok in ("mj_step(", "mj_forward(", "efc_force", "import mujoco", "import numpy", "import scipy"):
        assert tok not in body, tok
    # No optimizer/search imports.
    assert "import random" not in body and "import opt" not in body


def test_04_res54_com_authority_used():
    import mujoco
    from loaded_cmj.v2.plant import V2Plant
    from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
    sys.path.insert(0, str(REPO / "tools" / "res52"))
    import core52 as C
    states = np.load(SEALED52 / "branch_states.npz")
    p = V2Plant()
    d = p.make_data()
    meas = create_measurement_data(p)
    C.restore_state(p.model, d, states["S50"])
    s = SynchronizedPhysicsSample.from_live_state(p, d, meas)
    vz = float(s.com_velocity_mps[2])
    assert abs(vz - (-0.31905596)) < 1e-6
    assert abs(vz - (-0.34929)) > 0.01  # stale pre-RES54 metadata prohibited
    J = np.zeros((3, p.model.nv))
    mujoco.mj_jacSubtreeCom(p.model, d, J, 1)
    assert abs(float((J @ np.asarray(d.qvel, float))[2]) - vz) < 1e-12
    from loaded_cmj.v2 import plant as pm
    psrc = inspect.getsource(pm.V2Plant.center_of_mass_velocity)
    assert "mj_jacSubtreeCom" in psrc and "data.cvel" not in psrc


def test_05_res55_point_velocity_authority_used():
    sys.path.insert(0, str(REPO / "tools" / "res52"))
    from core52 import make_plant, bind_plant, restore_state, soft_contact_state, foot_gap_half_height
    states = np.load(SEALED52 / "branch_states.npz")
    p = make_plant()
    bind_plant(p)
    m, d = p.model, p.make_data()
    restore_state(m, d, states["S50"])
    gap = foot_gap_half_height(m)
    cs = soft_contact_state(m, d, gap_half_height=gap)
    assert cs["L"]["active_row"] and cs["R"]["active_row"]
    assert cs["L"]["foot_normal_velocity"] > 0.05
    import mujoco
    for side in ("L", "R"):
        fg = p.idx.left_foot_geom if side == "L" else p.idx.right_foot_geom
        rows = [i for i in range(d.ncon)
                if (int(d.contact[i].geom1) == p.idx.floor_geom or int(d.contact[i].geom2) == p.idx.floor_geom)
                and (int(d.contact[i].geom1) == fg or int(d.contact[i].geom2) == fg)]
        deep = rows[int(np.argmin([float(d.contact[j].dist) for j in rows]))]
        assert cs[side]["foot_normal_velocity"] == float(d.efc_vel[d.contact[deep].efc_address])


def test_06_res57_contract_hash_and_semantics():
    from loaded_cmj.v2.support_continuity import load_spec, spec_sha256
    spec = load_spec(REPO / "support_continuity_spec.json")
    assert spec["SPEC_SHA256"] == CONTRACT_SHA and spec_sha256(spec) == CONTRACT_SHA
    assert spec["CONTROL_RELEVANT_LOSS_STEPS"] == 40
    assert spec["CONTROL_RELEVANT_LOSS_DWELL_S"] == 0.005
    assert spec["PER_FOOT_FZ_THRESHOLD_N"] == 10.0
    assert spec["PHYSICS_DT_S"] == 0.000125 and spec["CONTROL_DT_S"] == 0.005
    from loaded_cmj.v2 import support_continuity as sc
    assert sc.CONTROL_RELEVANT_LOSS_STEPS == 40
    assert sc.PER_FOOT_FZ_THRESHOLD_N == 10.0
    assert sc.REFLIGHT_MIN_STEPS == 4 and sc.CHATTER_TRANSITIONS_MAX == 8


def test_07_one_control_interval_terminology():
    # 5 ms is one control interval / control-relevance dwell, NOT a Nyquist period.
    from loaded_cmj.v2 import support_continuity as sc
    assert sc.CONTROL_DT_S == 0.005 and sc.CONTROL_RELEVANT_LOSS_DWELL_S == 0.005
    assert sc.STEPS_PER_CONTROL_INTERVAL == 40
    doc = (REPO / "TERMINAL_CAPTURE_SEAL_AUTHORITY.md").read_text()
    assert "one control interval" in doc
    assert "control-relevance dwell" in doc
    assert "Nyquist period" not in doc  # prohibited wording for the 5-ms dwell


def test_08_exact_s50_start_sha():
    sys.path.insert(0, str(REPO / "tools" / "res52"))
    import core52 as C
    states = np.load(SEALED52 / "branch_states.npz")
    assert C.sha_arr(states["S50"]) == S50_SHA
    qual = json.loads((B58 / "TERMINAL_CAPTURE_S50_QUALIFICATION.json").read_text())
    assert qual["START_STATE_SHA256"] == S50_SHA
    assert qual["S50_SHA256"] == S50_SHA


def test_09_40ms_nearzero_dwell():
    qual = json.loads((B58 / "TERMINAL_CAPTURE_S50_QUALIFICATION.json").read_text())
    assert abs(qual["NEARZERO_ENTRY_T_REL"] - 0.051875) < 1e-9
    assert abs(qual["ZERO_CROSS_T_REL"] - 0.063125) < 1e-9
    assert abs(qual["NEARZERO_ENTRY_DWELL_S"] - 0.09825) < 1e-12
    assert qual["NEARZERO_ENTRY_DWELL_STEPS"] == 786
    assert qual["CAPTURE_GATE_PASS"] is True
    # 40 ms = 320 physics steps at 0.125 ms.
    assert 320 * 0.000125 == 0.04
    assert qual["NEARZERO_ENTRY_DWELL_STEPS"] >= 320


def test_10_sample_dropout_vs_control_episode():
    qual = json.loads((B58 / "TERMINAL_CAPTURE_S50_QUALIFICATION.json").read_text())
    assert qual["UNILATERAL_FORCE_DROPOUT_SAMPLE_COUNT"] == 3
    assert qual["GEOMETRIC_LIFTOFF_SAMPLE_COUNT"] == 0
    assert qual["CONTROL_RELEVANT_SUPPORT_LOSS_COUNT"] == 0
    assert qual["CANONICAL_REFLIGHT"] == []
    assert qual["CHATTER_TRANSITIONS"] == 0
    assert qual["DROPOUT_INDICES"] == [1016, 1025, 1165]
    # Sample flickers retained separately; episodes remain non-control-relevant.
    assert len(qual["CONTROL_RELEVANT_EPISODES_L"]) + len(qual["CONTROL_RELEVANT_EPISODES_R"]) == 3
    for e in qual["CONTROL_RELEVANT_EPISODES_L"] + qual["CONTROL_RELEVANT_EPISODES_R"]:
        assert e["CONTROL_RELEVANT"] is False and e["length"] == 1


def test_11_historical_res56_fail_preserved():
    rec = json.loads((B58 / "TERMINAL_CAPTURE_AUTHORITY_RECONSTRUCTION.json").read_text())
    assert rec["RES56_HISTORICAL_STRICT_VERDICT"] == "FAIL"
    assert rec["RES56_HISTORICAL_POST_WALK_LOSS"] == 3
    assert rec["RES56_HISTORICAL_POST_WALK_ALLOW"] == 1
    b56res = json.loads((B56 / "result_assessment.json").read_text())
    assert b56res["OVERALL"] == "FAIL"
    qual = json.loads((B58 / "TERMINAL_CAPTURE_S50_QUALIFICATION.json").read_text())
    assert qual["HISTORICAL_RES56_STRICT_POST_WALK_LOSS"] == 3
    assert qual["CURRENT_SUPPORT_CONTINUITY"] == "QUALIFIED"


def test_12_no_plant_contact_scorer_takeoff_e10_changes():
    from loaded_cmj.v2.constants import V2_CONTACT_SOLREF, V2_CONTACT_SOLIMP, V2_FRICTION_FLOOR, V2_TOTAL_MASS_KG, V2_PHYSICS_TIMESTEP_S
    assert tuple(V2_CONTACT_SOLREF) == (0.016, 1.0)
    assert tuple(V2_CONTACT_SOLIMP) == (0.99, 0.99, 0.001, 0.5, 2.0)
    assert tuple(V2_FRICTION_FLOOR) == (0.9, 0.005, 0.0001)
    assert V2_TOTAL_MASS_KG == 95.0 and V2_PHYSICS_TIMESTEP_S == 0.000125
    spec = json.loads((B58 / "experiment_spec.json").read_text())
    assert "E10" in spec["SCOPE_EXCLUSIONS"] and "E11" in spec["SCOPE_EXCLUSIONS"] and "E12" in spec["SCOPE_EXCLUSIONS"]
    assert spec["SEARCH_METHOD"].startswith("NONE")
    qual = json.loads((B58 / "TERMINAL_CAPTURE_S50_QUALIFICATION.json").read_text())
    assert abs(qual["PEAK_FZ_BW"] - 1.797891) < 1e-4
    assert abs(qual["MAX_PENETRATION"] - 0.00815642) < 1e-7


def test_13_evidence_contract_v2_identity():
    man = json.loads((B58 / "manifest.json").read_text())
    spec = json.loads((B58 / "experiment_spec.json").read_text())
    run = json.loads((B58 / "run_record.json").read_text())
    res = json.loads((B58 / "result_assessment.json").read_text())
    assert man["EVIDENCE_CONTRACT"] == "v2"
    assert man["EXPERIMENT_ID"] == spec["EXPERIMENT_ID"] == run["EXPERIMENT_ID"] == res["EXPERIMENT_ID"] == "EXP-RES10-TRUE-MOMENTUM-CAPTURE-002"
    assert run["SPEC_EXECUTION_MATCH"] == "PASS" and res["SPEC_EXECUTION_MATCH"] == "PASS"
    assert man["SPEC_EXECUTION_MATCH"] == "PASS"
