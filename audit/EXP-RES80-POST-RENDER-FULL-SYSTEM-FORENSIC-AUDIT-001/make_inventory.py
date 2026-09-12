#!/usr/bin/env python3
"""Generate AUDIT_REPOSITORY_INVENTORY.json, SOURCE_REVIEW_COVERAGE.json, TEST_FILE_INVENTORY.csv."""
import csv
import json
import re
import subprocess
from pathlib import Path

REPO = Path("/home/litju/Projects/loaded-cmj-control")
OUT = Path(__file__).resolve().parent

tracked = subprocess.check_output(["git", "-C", str(REPO), "ls-files"], text=True).split()
untracked = subprocess.check_output(
    ["git", "-C", str(REPO), "status", "--porcelain"], text=True
).split("\n")
untracked_files = sorted(l[3:] for l in untracked if l.startswith("?? ") and not l[3:].startswith("audit/"))

def lang(p):
    p = str(p)
    for ext, name in [(".py", "python"), (".xml", "xml"), (".json", "json"), (".jsonl", "jsonl"),
                      (".md", "markdown"), (".toml", "toml"), (".csv", "csv"), (".lock", "lock"),
                      (".sh", "shell")]:
        if p.endswith(ext):
            return name
    return "other"

def lines(p):
    try:
        return sum(1 for _ in open(REPO / p, errors="replace"))
    except Exception:
        return 0

def classify(p):
    p = str(p)
    if p.startswith("src/loaded_cmj/v2/") and p.endswith(".py"):
        return "AUDITED_EXECUTION_CRITICAL"
    if p in ("CANONICAL_V2_CANDIDATE_SPEC.json", "CANONICAL_CANDIDATE_IDENTITY.json",
             "CANONICAL_V2_RUNTIME_CONTRACT.md", "CANONICAL_RUNTIME_AUTHORITY.md"):
        return "AUDITED_EXECUTION_CRITICAL"
    if p == "src/loaded_cmj/v2/assets/v2_plant.xml":
        return "AUDITED_EXECUTION_CRITICAL"
    if p.startswith("tools/res52/") or p in ("tools/evid_trace_v2.py", "tools/res79_smoke_replay.py",
                                             "tools/run_canonical_rollout.py", "tools/diagnostic_replay.py"):
        return "AUDITED_EXECUTION_CRITICAL"
    if p.startswith("tests/"):
        return "AUDITED_TEST"
    if p.endswith(".md") or p.endswith(".json") or p.endswith(".jsonl") or p.endswith(".csv"):
        return "AUDITED_DOCUMENTATION"
    if p.startswith("src/loaded_cmj/") and not p.startswith("src/loaded_cmj/v2/"):
        return "OUT_OF_SCOPE_WITH_REASON"
    if p.startswith("docs/"):
        return "AUDITED_DOCUMENTATION"
    if p.startswith(".skills/") or p == "README.md":
        return "AUDITED_SUPPORTING"
    if p in ("pyproject.toml", "uv.lock", ".gitignore"):
        return "AUDITED_SUPPORTING"
    return "OUT_OF_SCOPE_WITH_REASON"

def reason(p):
    p = str(p)
    if p.startswith("src/loaded_cmj/biomechanics/") or p.startswith("src/loaded_cmj/oracle/") \
       or p.startswith("src/loaded_cmj/simulation/") or p.startswith("src/loaded_cmj/runtime/") \
       or p.startswith("src/loaded_cmj/rendering/"):
        return "Legacy V1/F4/Gen1 subsystem not on the V2.1 canonical execution path; reviewed only for import coupling (see PROVENANCE_REVIEW.md)."
    if p.startswith("src/loaded_cmj/control/"):
        return "Legacy V1 control subsystem not on the V2.1 canonical path (res51/gen3 are untracked WIP)."
    return "No role in V2.1 execution/release; not reviewed line-by-line."

coverage = []
for p in tracked:
    coverage.append({
        "PATH": p, "LANGUAGE": lang(p), "LINES": lines(p),
        "CLASS": classify(p), "REASON": reason(p) if classify(p).startswith("OUT_OF") else "",
    })
total = len(coverage)
audited = sum(1 for r in coverage if not r["CLASS"].startswith("OUT_OF"))
out_of_scope = total - audited
src = sum(1 for r in coverage if r["CLASS"] == "AUDITED_EXECUTION_CRITICAL")
test = sum(1 for r in coverage if r["CLASS"] == "AUDITED_TEST")
doc = sum(1 for r in coverage if r["CLASS"] == "AUDITED_DOCUMENTATION")
sup = sum(1 for r in coverage if r["CLASS"] == "AUDITED_SUPPORTING")

inv = {
    "MISSION": "V2_1_POST_RENDER_FULL_SYSTEM_FORENSIC_AUDIT_001",
    "BASELINE_HEAD": "8f26736db1231042cbedc61f61ca4862b7be871c",
    "BASELINE_TREE": "855fe3b418c09b5028396adeaed3ba8cda51e89f",
    "TRACKED_FILES_TOTAL": total,
    "TRACKED_LINES_TOTAL": sum(r["LINES"] for r in coverage),
    "UNTRACKED_PRESERVED_FILES": untracked_files,
    "UNTRACKED_NOTE": "Untracked WIP preserved; not part of the frozen baseline. Includes 2 files whose collection breaks pytest (test_public_support_wrench_contract.py is untracked; test_ml241_qacc_resolution.py is tracked and needs absent external evidence).",
    "CATEGORIES": {
        "EXECUTION_CRITICAL": [],
        "CONTROLLER_COMPOSITION": [],
        "MEASUREMENT": [],
        "EVENTS_SCORER": [],
        "CANONICAL_RUNTIME_PROVENANCE": [],
        "ASSETS_MODEL": [],
        "TOOLS": [],
        "TESTS": [],
        "DOCS_AUTHORITY": [],
        "SPECS_JSON": [],
        "RENDER_REPLAY": [],
    },
}
catmap = [
    ("CONTROLLER_COMPOSITION", ["src/loaded_cmj/v2/res72_integration.py", "src/loaded_cmj/v2/balance_capture.py",
                                "src/loaded_cmj/v2/stable_recovery.py", "src/loaded_cmj/v2/terminal_capture.py",
                                "src/loaded_cmj/v2/full_closure.py", "src/loaded_cmj/v2/controller.py",
                                "src/loaded_cmj/v2/drive.py"]),
    ("MEASUREMENT", ["src/loaded_cmj/v2/measurement.py", "src/loaded_cmj/v2/plant.py", "src/loaded_cmj/v2/support_continuity.py"]),
    ("EVENTS_SCORER", ["src/loaded_cmj/v2/events.py", "src/loaded_cmj/v2/constants.py"]),
    ("CANONICAL_RUNTIME_PROVENANCE", ["src/loaded_cmj/v2/canonical_runtime.py", "src/loaded_cmj/v2/sync_rollout.py",
                                      "CANONICAL_V2_CANDIDATE_SPEC.json", "CANONICAL_CANDIDATE_IDENTITY.json",
                                      "CANONICAL_V2_RUNTIME_CONTRACT.md", "CANONICAL_RUNTIME_AUTHORITY.md",
                                      "AUTHORITY_GRAPH.md", "AUTHORITY_LEDGER.json", "EXPERIMENT_REGISTRY.jsonl",
                                      "EVIDENCE_CONTRACT.md", "VVUQ_AND_QUALIFICATION_TAXONOMY.md",
                                      "INTENDED_USE_AND_CLAIM_BOUNDARY.md", "EXPERIMENT_PROTOCOL.md",
                                      "TRACE_SCHEMA_V2.md", "BILATERAL_SUPPORT_CONTINUITY_CONTRACT.md",
                                      "TRUE_COM_VELOCITY_CONTRACT.md", "TRUE_FOOT_POINT_VELOCITY_CONTRACT.md",
                                      "SUPPORT_SEMANTICS_CALLGRAPH.md", "CONTROLLER_OBSERVATION_CONTRACT.md",
                                      "TERMINAL_CAPTURE_SEAL_AUTHORITY.md", "PRODUCTION_MEASUREMENT_CALLGRAPH.md"]),
    ("ASSETS_MODEL", ["src/loaded_cmj/v2/assets/v2_plant.xml"]),
    ("RENDER_REPLAY", ["tools/res79_smoke_replay.py", "tools/diagnostic_replay.py", "tools/run_canonical_rollout.py",
                       "tools/replay_from_state.py", "src/loaded_cmj/rendering/replay.py"]),
]
for name, paths in catmap:
    inv["CATEGORIES"][name] = [p for p in paths if (REPO / p).exists()]
inv["CATEGORIES"]["TOOLS"] = [p for p in tracked if p.startswith("tools/")]
inv["CATEGORIES"]["TESTS"] = [p for p in tracked if p.startswith("tests/")]
inv["CATEGORIES"]["DOCS_AUTHORITY"] = [p for p in tracked if p.endswith(".md")]
inv["CATEGORIES"]["SPECS_JSON"] = [p for p in tracked if p.endswith((".json", ".jsonl", ".csv", ".toml", ".lock"))]
inv["SOURCE_REVIEW_COVERAGE_SUMMARY"] = {
    "TRACKED_FILES_TOTAL": total, "TRACKED_FILES_AUDITED": audited,
    "TRACKED_FILES_OUT_OF_SCOPE": out_of_scope,
    "AUDITED_EXECUTION_CRITICAL": src, "AUDITED_TEST": test,
    "AUDITED_DOCUMENTATION": doc, "AUDITED_SUPPORTING": sup,
    "TOTAL_TRACKED_LINES": sum(r["LINES"] for r in coverage),
    "AUDITED_LINES": sum(r["LINES"] for r in coverage if not r["CLASS"].startswith("OUT_OF")),
}
(OUT / "AUDIT_REPOSITORY_INVENTORY.json").write_text(json.dumps(inv, indent=2) + "\n")

cov_doc = {
    "MISSION": "V2_1_POST_RENDER_FULL_SYSTEM_FORENSIC_AUDIT_001",
    "METHOD": ("Every tracked file classified. AUDITED_EXECUTION_CRITICAL = read in full during this audit "
               "(V2.1 plant/controller/measurement/events/runtime + packaging/authority files). "
               "AUDITED_TEST/DOCUMENTATION/SUPPORTING = audited at the level required by the mission. "
               "OUT_OF_SCOPE = legacy V1/F4/Gen1 or unrelated subsystem not on the V2.1 canonical path; "
               "not claimed as line-audited."),
    "SUMMARY": inv["SOURCE_REVIEW_COVERAGE_SUMMARY"],
    "FILES": coverage,
}
(OUT / "SOURCE_REVIEW_COVERAGE.json").write_text(json.dumps(cov_doc, indent=2) + "\n")

# ---------------- test inventory ----------------
CLASS = {
 "tests/test_ml241_qacc_resolution.py": "BROKEN_EXTERNAL_PATH",
 "tests/test_public_support_wrench_contract.py": "BROKEN_EXTERNAL_PATH",
 "tests/test_v2_1_res10_honest_full_jump.py": "FALSE_POSITIVE_RISK",
 "tests/test_v2_1_res8_captured_squat.py": "FALSE_POSITIVE_RISK",
 "tests/test_res11_deterministic_offline.py": "FALSE_POSITIVE_RISK",
 "tests/test_res72_corrected_apex_to_e10.py": "VALID_BUT_HISTORICAL",
 "tests/test_res73_balance_capture.py": "VALID_BUT_HISTORICAL",
 "tests/test_res74_stable_recovery.py": "VALID_BUT_HISTORICAL",
 "tests/test_res78_canonical_runtime.py": "WEAK",
 "tests/test_res76_full_closure.py": "WEAK",
 "tests/test_res55_true_foot_point_velocity.py": "WEAK",
 "tests/test_res52_soft_contact.py": "WEAK",
 "tests/test_res51_centroidal_landing.py": "WEAK",
 "tests/test_res54_true_com_velocity.py": "WEAK",
 "tests/test_res10_controller_obs_sync.py": "WEAK",
 "tests/test_res10_physics_sample_sync.py": "WEAK",
 "tests/test_v2_1_res5_honest_fall.py": "WEAK",
 "tests/test_v2_1_res6_compliant_landing_contact.py": "WEAK",
 "tests/test_v2_1_res31_honest_planar_root.py": "WEAK",
 "tests/test_v2_1_res16_true_standing.py": "WEAK",
 "tests/test_r01_bundle_contract.py": "WEAK",
 "tests/test_public_qualification.py": "WEAK",
 "tests/test_rec01a_synchronized_dynamics.py": "WEAK",
 "tests/qualification_causal_runtime.py": "FALSE_POSITIVE_RISK",
 "tests/qualification_ml241_wrapped_derivatives.py": "FALSE_POSITIVE_RISK",
}
QUAL_SUITES = {"tests/qualification_" + n for n in (
    "actuator_dynamics", "causal_runtime", "cmj_events", "contact_force_plates", "determinism",
    "f2_model_measurement", "f3_actuator", "f3_capturability", "f3_closure", "f3_effectiveness",
    "f3_metrics", "f3_plant", "mechanics", "ml241_wrapped_derivatives", "ml242_transcription",
    "numerical_mechanics", "passive_energetics", "policy_isolation", "semantic_contracts")}

rows = []
for p in sorted(t for t in tracked if t.startswith("tests/")):
    text = (REPO / p).read_text(errors="replace")
    tests = len(re.findall(r"^def test_", text, re.M))
    is_qual = p in QUAL_SUITES
    kind = "QUALIFICATION_SCRIPT" if is_qual else ("PYTEST" if tests else "SUPPORT_MODULE")
    ext = bool(re.search(r"/home/litju|/tmp/opencode", text))
    assert_true = len(re.findall(r"assert True", text))
    or_true = len(re.findall(r"or True", text))
    skips = len(re.findall(r"pytest\.skip", text))
    pass_body = len(re.findall(r"def test_[a-zA-Z0-9_]+\([^)]*\):\s*\n(?:\s*#.*\n)*\s*pass\b", text))
    source_string = bool(re.search(r"inspect\.getsource|\.read_text\(\)|split\('\"\"\"'", text))
    behavioral = bool(re.search(r"mujoco|mj_step|mj_forward|V2Plant|np\.allclose|assert abs|assert np", text))
    canonical = "canonical_runtime" in text and "run_canonical_episode()" in text and "callable" not in text
    cls = CLASS.get(p, "TRUSTED_CURRENT" if not is_qual else "TRUSTED_CURRENT")
    rows.append({
        "PATH": p, "TEST_COUNT": tests, "KIND": kind,
        "CURRENT_AUTHORITY": "YES" if any(k in p for k in ("res78", "res79", "res57", "res58", "res11", "res10_honest")) else "MIXED",
        "HISTORICAL_ONLY": "YES" if cls == "VALID_BUT_HISTORICAL" else "NO",
        "EXTERNAL_EVIDENCE_DEPENDENCY": "YES" if ext else "NO",
        "ABSOLUTE_PATH_DEPENDENCY": "YES" if "/home/litju" in text else "NO",
        "PLACEHOLDER_TESTS": assert_true + pass_body,
        "TAUTOLOGICAL_TESTS": or_true,
        "SOURCE_STRING_TESTS": "YES" if source_string else "NO",
        "BEHAVIORAL_TESTS": "YES" if behavioral else "NO",
        "SCIENTIFIC_GATE_TESTS": "YES" if re.search(r"dwell|gate|PASS|threshold|envelope|E1[0-2]", text) else "NO",
        "FALSE_PASS_RISK": "HIGH" if cls == "FALSE_POSITIVE_RISK" else ("MEDIUM" if cls == "WEAK" else "LOW"),
        "CLASSIFICATION": cls,
        "ACTION": {"BROKEN_EXTERNAL_PATH": "REPAIR PATH OR MARK ENV-GATED",
                   "FALSE_POSITIVE_RISK": "REPLACE WITH BEHAVIORAL GATES",
                   "VALID_BUT_HISTORICAL": "MARK HISTORICAL, DO NOT CITE AS CURRENT",
                   "WEAK": "STRENGTHEN OR BIND TO CURRENT COMPOSITION",
                   "TRUSTED_CURRENT": "KEEP"}[cls],
    })
with open(OUT / "TEST_FILE_INVENTORY.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

# hard-gate counters
counters = {
    "ASSERT_TRUE_COUNT": sum(r["PLACEHOLDER_TESTS"] for r in rows),
    "OR_TRUE_COUNT": sum(r["TAUTOLOGICAL_TESTS"] for r in rows),
    "PASS_ONLY_COUNT": sum(1 for r in rows if r["PLACEHOLDER_TESTS"] > 0),
    "PLACEHOLDER_COUNT": sum(r["PLACEHOLDER_TESTS"] for r in rows),
    "ABSOLUTE_TMP_PATH_COUNT": sum(1 for r in rows if "/tmp/opencode" in (REPO / r["PATH"]).read_text(errors="replace")),
    "ABSOLUTE_HOME_PATH_COUNT": sum(1 for r in rows if "/home/litju" in (REPO / r["PATH"]).read_text(errors="replace")),
    "SOURCE_STRING_TEST_COUNT": sum(1 for r in rows if r["SOURCE_STRING_TESTS"] == "YES"),
    "CURRENT_BEHAVIORAL_TEST_COUNT": sum(1 for r in rows if r["BEHAVIORAL_TESTS"] == "YES"),
    "SCIENTIFIC_GATE_TEST_COUNT": sum(1 for r in rows if r["SCIENTIFIC_GATE_TESTS"] == "YES"),
}
# exact counts for the report (from source, not row-approx)
all_text = "\n".join((REPO / r["PATH"]).read_text(errors="replace") for r in rows)
counters["ASSERT_TRUE_COUNT"] = len(re.findall(r"assert True", all_text))
counters["OR_TRUE_COUNT"] = len(re.findall(r"or True", all_text))
counters["PASS_ONLY_COUNT"] = len(re.findall(r"def test_[a-zA-Z0-9_]+\([^)]*\):\s*\n(?:\s*#.*\n)*\s*pass\b", all_text))
counters["PLACEHOLDER_COUNT"] = counters["ASSERT_TRUE_COUNT"] + counters["PASS_ONLY_COUNT"]
(OUT / "TEST_FORENSIC_COUNTERS.json").write_text(json.dumps(counters, indent=2) + "\n")
print(json.dumps({"inventory": inv["SOURCE_REVIEW_COVERAGE_SUMMARY"], "test_rows": len(rows),
                  "counters": counters}, indent=2))
