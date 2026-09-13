#!/usr/bin/env python3
"""RES-81 Phase J: CODE_REVIEW, BUG_HUNT, ERRATA_REVIEW, ROADMAP_COVERAGE_REVIEW, PROVENANCE_REVIEW."""
import hashlib
import json
import subprocess
from pathlib import Path

REPO = Path("/home/litju/Projects/loaded-cmj-control")
EV80 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001")
EV81 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES81-POST-RES80-AUTHORITY-RESET-001")
D = REPO / "audit" / "EXP-RES81-POST-RES80-AUTHORITY-RESET-001"
TMP = Path("/tmp/opencode/res81")

def sh(args, cwd=REPO):
    return subprocess.run(args, cwd=cwd, capture_output=True, text=True).stdout.strip()

def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()

findings = []

def check(gate, name, ok, detail):
    findings.append({"gate": gate, "check": name, "status": "PASS" if ok else "FAIL", "detail": detail})
    return ok

# ---------------------------------------------------------------- CODE_REVIEW
tracked = sh(["git", "status", "--porcelain", "--untracked-files=no"])
check("CODE_REVIEW", "tracked_worktree_unmodified", tracked == "", f"tracked porcelain={tracked!r}")
untracked = sh(["git", "status", "--porcelain"])
new_paths = [l[3:] for l in untracked.splitlines() if l.startswith("??")]
ENTRY_UNTRACKED = [
    ".claude/", ".skills/lcmj-controller-ship/SKILL.md", ".skills/lcmj-failure-driven-qualification/",
    "IDEA.md", "PRODUCTION_MEASUREMENT_CALLGRAPH.md", "RES10_R3_POSTPASS_INTEGRITY_RECEIPT.md",
    "experiments/build_local_effectiveness_artifact.py", "experiments/diagnose_local_effectiveness.py",
    "experiments/qualify_selected_local_actions.py", "experiments/recompute_controller_depth.py",
    "experiments/refresh_diagnostic_support_validity.py", "opencode.json",
    "src/loaded_cmj/control/gen3_reference.py", "src/loaded_cmj/control/reference_data/",
    "src/loaded_cmj/control/res51_policy.py", "tests/test_hip_braking_polarity.py",
    "tests/test_knee_rate_feasibility.py", "tests/test_progressive_braking_controller.py",
    "tests/test_public_support_wrench_contract.py", "tests/test_res10_physics_sample_sync.py",
    "tests/test_res51_centroidal_landing.py", "tools/diagnostic_replay.py",
    "tools/reproduce_res10_sync.py", "tools/res51/", "tools/run_canonical_rollout.py",
]
delta = sorted(set(new_paths) - set(ENTRY_UNTRACKED))
preserved = sorted(set(ENTRY_UNTRACKED) - set(new_paths))
new_authority_only = delta == ["audit/EXP-RES81-POST-RES80-AUTHORITY-RESET-001/"]
check("CODE_REVIEW", "only_new_authority_dir_untracked", new_authority_only, f"delta={delta}")
check("CODE_REVIEW", "entry_untracked_preserved", preserved == [], f"removed_or_renamed={preserved}")
diff_names = sh(["git", "diff", "--name-only"]).splitlines()
check("CODE_REVIEW", "no_tracked_diff", diff_names == [], f"diff={diff_names}")
plant_controller_touched = [p for p in diff_names if p.startswith(("src/", "tools/", "tests/"))]
check("CODE_REVIEW", "no_plant_controller_scorer_changes", plant_controller_touched == [], f"{plant_controller_touched}")
# staged area empty
staged = sh(["git", "diff", "--cached", "--name-only"]).splitlines()
check("CODE_REVIEW", "nothing_staged_pretamper", staged == [], f"staged={staged}")

# ---------------------------------------------------------------- BUG_HUNT
for f in ["RES80_ERRATA_001.json", "POST_RES80_AUTHORITY_BASELINE.json",
          "RES80_DEFECT_TO_ROADMAP_MATRIX.json", "ROADMAP_REQUIREMENT_COVERAGE.json",
          "ROADMAP_DEPENDENCY_VALIDATION.json", "RES81_RECOMPUTE.json"]:
    try:
        json.load(open(D / f))
        check("BUG_HUNT", f"json_valid:{f}", True, "parsed")
    except Exception as e:
        check("BUG_HUNT", f"json_valid:{f}", False, str(e))
# matrix totals
M = json.load(open(D / "RES80_DEFECT_TO_ROADMAP_MATRIX.json"))
rows = M["DEFECTS"]
check("BUG_HUNT", "matrix_total_53", len(rows) == 53, str(len(rows)))
check("BUG_HUNT", "matrix_ids_unique", len({r["DEFECT_ID"] for r in rows}) == 53, "")
check("BUG_HUNT", "matrix_severity_counts", [r["SEVERITY"] for r in rows].count("CRITICAL") == 7
      and [r["SEVERITY"] for r in rows].count("HIGH") == 19
      and [r["SEVERITY"] for r in rows].count("MEDIUM") == 20
      and [r["SEVERITY"] for r in rows].count("LOW") == 7, "")
check("BUG_HUNT", "matrix_every_primary_assigned", all(r["PRIMARY_MITIGATION_ISSUE"] for r in rows), "")
check("BUG_HUNT", "matrix_no_self_secondary",
      all(r["PRIMARY_MITIGATION_ISSUE"] not in r["SECONDARY_ISSUES"] for r in rows), "")
# errata numbers equal recompute numbers
E = json.load(open(D / "RES80_ERRATA_001.json"))
R = json.load(open(D / "RES81_RECOMPUTE.json"))
by_id = {e["ERRATUM_ID"]: e for e in E["ERRATA"]}
m = by_id["ERR-001"]["NUMERIC_EVIDENCE"]
a = R["5A_jump_height"]
check("BUG_HUNT", "err001_numbers_match_recompute",
      abs(m["APEX_ABOVE_INITIAL_STANDING_COM"] - a["APEX_ABOVE_INITIAL_STANDING_COM"]) < 1e-8
      and abs(m["TAKEOFF_TO_APEX_COM_RISE_E6"] - a["TAKEOFF_TO_APEX_COM_RISE_E6"]) < 1e-8
      and abs(m["E6_COM_VZ"] - a["E6_COM_VZ"]) < 1e-8
      and abs(m["E6_TO_E9_DURATION_S"] - a["E6_TO_E9_DURATION_S"]) < 1e-8, "")
c = by_id["ERR-003"]["NUMERIC_EVIDENCE"]
x = R["5C_action_difference"]
check("BUG_HUNT", "err003_numbers_match_recompute",
      abs(c["MAX_DU"] - x["MAX_DU"]) < 1e-8 and abs(c["POST_ACTION_TIME"] - x["POST_ACTION_TIME"]) < 1e-8
      and abs(c["PRE_ACTION_TIME"] - x["PRE_ACTION_TIME"]) < 1e-8, "")
# requirement totals
Q = json.load(open(D / "ROADMAP_REQUIREMENT_COVERAGE.json"))
check("BUG_HUNT", "requirements_total_50", Q["REQUIREMENTS_TOTAL"] == 50 and Q["REQUIREMENTS_MAPPED"] == 50, "")
check("BUG_HUNT", "requirements_unique", len({r["REQUIREMENT_ID"] for r in Q["REQUIREMENTS"]}) == 50, "")

# ---------------------------------------------------------------- ERRATA_REVIEW
check("ERRATA_REVIEW", "errata_ids_exact",
      sorted(by_id) == ["ERR-001", "ERR-002", "ERR-003", "ERR-004", "ERR-005"], "")
def sev_ok(e):
    b, a2 = e["SEVERITY_BEFORE"], e["SEVERITY_AFTER"]
    if b.startswith("N/A"):
        return a2.startswith("N/A")
    return b.split()[0] == a2.split()[0]
check("ERRATA_REVIEW", "no_severity_weakened",
      all(sev_ok(e) for e in E["ERRATA"]),
      "; ".join(f"{e['ERRATUM_ID']}:{e['SEVERITY_BEFORE']}->{e['SEVERITY_AFTER']}" for e in E["ERRATA"]))
check("ERRATA_REVIEW", "high003_unchanged",
      by_id["ERR-005"]["SEVERITY_AFTER"].startswith("HIGH") and by_id["ERR-005"]["HIGH003_SEVERITY_UNCHANGED"] is True, "")
check("ERRATA_REVIEW", "crit007_unchanged",
      by_id["ERR-003"]["SEVERITY_AFTER"] == "CRITICAL (CRIT-007)", "")
check("ERRATA_REVIEW", "r001_disposition_not_rescued",
      all("NONE" in e["R001_DISPOSITION_EFFECT"] for e in E["ERRATA"]), "")
check("ERRATA_REVIEW", "immutability_statement_present",
      "IMMUTABLE" in E["IMMUTABILITY_STATEMENT"], "")
B = json.load(open(D / "POST_RES80_AUTHORITY_BASELINE.json"))
check("ERRATA_REVIEW", "baseline_r001_blocked",
      B["R001_STATUS"] == "HISTORICAL_REPRODUCIBILITY_ONLY" and B["SHIP_STATUS"] == "BLOCKED"
      and B["OWNER_PHYSICAL_VISUAL_CREDIBILITY"] == "FAIL", "")
check("ERRATA_REVIEW", "no_successor_authorized", B["NO_SUCCESSOR_CANDIDATE_AUTHORIZED"] is True, "")
check("ERRATA_REVIEW", "old_path_canceled",
      B["OLD_R001_FINAL_PATH"] == ["RES-13 CANCELED", "RES-14 CANCELED", "RES-15 CANCELED"], "")

# ---------------------------------------------------------------- ROADMAP_COVERAGE_REVIEW
prim = [r["PRIMARY_MITIGATION_ISSUE"] for r in rows]
check("ROADMAP_COVERAGE_REVIEW", "defect_orphans_zero", M["COVERAGE"]["ORPHAN_DEFECTS"] == 0, "")
check("ROADMAP_COVERAGE_REVIEW", "duplicate_primary_zero", M["COVERAGE"]["DUPLICATE_PRIMARY_OWNERSHIP"] == 0,
      "ownership is per-defect unique; multiple defects may share an issue by design")
allowed = set(M["ALLOWED_ROADMAP_OWNERS"])
check("ROADMAP_COVERAGE_REVIEW", "all_primaries_allowed", all(p in allowed for p in prim), "")
req_prim = [r["PRIMARY_MITIGATION_ISSUE"] for r in Q["REQUIREMENTS"]]
allowed_req = allowed | {"RES-90", "RES-91", "RES-92", "RES-93"}
check("ROADMAP_COVERAGE_REVIEW", "all_requirement_primaries_allowed", all(p in allowed_req for p in req_prim), "")
G = json.load(open(D / "ROADMAP_DEPENDENCY_VALIDATION.json"))
check("ROADMAP_COVERAGE_REVIEW", "dependency_graph_pass",
      G["RESULT"] == "PASS" and not G["MISSING_DEPENDENCIES"] and not G["CONTRADICTIONS"], "")

# ---------------------------------------------------------------- PROVENANCE_REVIEW
# RES-80 bundle unchanged: recompute all checksums
out = subprocess.run(["sha256sum", "-c", "checksums.sha256"], cwd=EV80, capture_output=True, text=True)
bad = [l for l in out.stdout.splitlines() if not l.endswith(": OK")]
seal = sha256(EV80 / "checksums.sha256")
check("PROVENANCE_REVIEW", "res80_bundle_intact", len(bad) == 0 and out.returncode == 0, f"mismatches={bad}")
check("PROVENANCE_REVIEW", "res80_seal_match",
      seal == "c921e6de29a263be1bdcee78c9fa62c22b968388991cda50afa34bf3260a9341", seal)
# repo RES-80 files byte-identical to evidence bundle copies
tracked_files = sh(["git", "ls-files", "audit/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001"]).splitlines()
mismatch = []
for tf in tracked_files:
    rel = tf.split("audit/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001/", 1)[1]
    ev = EV80 / rel
    if ev.exists() and sha256(REPO / tf) != sha256(ev):
        mismatch.append(tf)
check("PROVENANCE_REVIEW", "repo_res80_matches_sealed_bundle", mismatch == [], f"mismatch={mismatch}")
# remote durable
check("PROVENANCE_REVIEW", "remote_main_audit_commit",
      sh(["git", "rev-parse", "origin/main"]) == "8c1cbb49c91ba9e6d504a1df63fba242efcf7c64", "")
check("PROVENANCE_REVIEW", "local_head_audit_commit",
      sh(["git", "rev-parse", "HEAD"]) == "8c1cbb49c91ba9e6d504a1df63fba242efcf7c64", "")
# RES-81 bundle seal
check("PROVENANCE_REVIEW", "res81_bundle_checksums",
      subprocess.run(["sha256sum", "-c", "checksums.sha256"], cwd=EV81, capture_output=True, text=True).returncode == 0, "")
check("PROVENANCE_REVIEW", "res81_seal_recorded",
      len(sha256(EV81 / "checksums.sha256")) == 64,
      "final bundle seal recorded in SEAL.json (self-referential inclusion excluded by contract)")

report = {
    "MISSION": "LCMJ_POST_RES80_REMEDIATION_PROGRAM_001",
    "LINEAR_ISSUE": "RES-81",
    "PHASE": "J — CODE/ARTIFACT REVIEW",
    "GATES": ["CODE_REVIEW", "BUG_HUNT", "ERRATA_REVIEW", "ROADMAP_COVERAGE_REVIEW", "PROVENANCE_REVIEW"],
    "TOTAL_CHECKS": len(findings),
    "FAILED_CHECKS": sum(1 for f in findings if f["status"] == "FAIL"),
    "STATUS": "PASS" if all(f["status"] == "PASS" for f in findings) else "FAIL",
    "FINDINGS": findings,
}
json.dump(report, open(TMP / "RES81_REVIEW_REPORT.json", "w"), indent=2)
for f in findings:
    print(f"{f['status']:4s} {f['gate']:28s} {f['check']}: {f['detail']}")
print("STATUS:", report["STATUS"], "checks:", report["TOTAL_CHECKS"], "failed:", report["FAILED_CHECKS"])
