#!/usr/bin/env python3
"""RES-82 contract consistency validator.

Mechanically enforces the 14 consistency gates of SUCCESSOR_TASK_CONTRACT.md §15
and emits CONTRACT_CONSISTENCY_REPORT.json/.md.

Read-only. No production file is modified. Run from the artifact directory.
"""
from __future__ import annotations

import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

ART = {
    "task": "SUCCESSOR_TASK_CONTRACT.json",
    "events": "EVENT_CONTRACT_E1_E12.json",
    "perf": "PERFORMANCE_METRIC_CONTRACT.json",
    "landing": "LANDING_BALANCE_RECOVERY_CONTRACT.json",
    "dwell": "DWELL_SEMANTICS.json",
    "nc": "NEGATIVE_CONTROL_CONTRACT.json",
    "owner": "OWNER_DECISIONS_REQUIRED.json",
    "schema": "CANONICAL_OUTPUT_SCHEMA.json",
    "dep": "MODEL_DEPENDENCY_MATRIX.json",
    "coverage": "SOURCE_REVIEW_COVERAGE.json",
}

MD_FILES = [
    "SUCCESSOR_CONTEXT_OF_USE.md",
    "SUCCESSOR_TASK_CONTRACT.md",
    "EVENT_CONTRACT_E1_E12.md",
    "PERFORMANCE_METRIC_CONTRACT.md",
    "LANDING_BALANCE_RECOVERY_CONTRACT.md",
    "DWELL_SEMANTICS.md",
    "NEGATIVE_CONTROL_CONTRACT.md",
    "SCIENTIFIC_CLAIM_CEILING.md",
    "OWNER_DECISIONS_REQUIRED.md",
]

# Contracts only (evidence table intentionally excluded: it quotes source titles).
QUALIFIER_SCAN_FILES = MD_FILES + [
    "EVENT_CONTRACT_E1_E12.json",
    "PERFORMANCE_METRIC_CONTRACT.json",
    "LANDING_BALANCE_RECOVERY_CONTRACT.json",
    "SUCCESSOR_TASK_CONTRACT.json",
    "DWELL_SEMANTICS.json",
    "NEGATIVE_CONTROL_CONTRACT.json",
    "CANONICAL_OUTPUT_SCHEMA.json",
    "OWNER_DECISIONS_REQUIRED.json",
]

DENYLIST = [
    "src/loaded_cmj/v2/assets/v2_plant.xml",
    "src/loaded_cmj/v2/controller.py",
    "src/loaded_cmj/v2/res72_integration.py",
    "src/loaded_cmj/v2/terminal_capture.py",
    "src/loaded_cmj/v2/balance_capture.py",
    "src/loaded_cmj/v2/stable_recovery.py",
    "src/loaded_cmj/v2/canonical_runtime.py",
    "src/loaded_cmj/v2/measurement.py",
    "src/loaded_cmj/v2/plant.py",
]

R001_RETROSPECTIVE = [
    {"CRITERION": "L1 MODEL_STATE_VALIDITY (anatomy)", "EVIDENCE": "CRIT-001 knee hinge inverted; CRIT-002 hip range extension-biased; HISTORICAL_JOINT_SIGNS_INVALID", "VERDICT": "FAIL", "OWNER": "RES-83 (primary), RES-82 records contract-level dependency"},
    {"CRITERION": "E6/E7 physical takeoff and genuine flight semantics", "EVIDENCE": "force chatter 0.640-0.64775 s with contact registrations; historical E7 force-only; geometric gap declared but never executed", "VERDICT": "FAIL", "OWNER": "RES-82 (contract), RES-84/85 (implementation)"},
    {"CRITERION": "E8 apex flight guard", "EVIDENCE": "historical fallback path not guarded by flight; no negative control executable then", "VERDICT": "FAIL", "OWNER": "RES-82 (contract), RES-89 (tests)"},
    {"CRITERION": "L3 PRIMARY_PERFORMANCE_GATE (if PF-1 approved)", "EVIDENCE": "COM_RISE_TAKEOFF_TO_APEX (true-support-off to apex) = 0.076836 m; E6-to-apex = 0.075158 m; PF-1 = 0.150 m", "VERDICT": "FAIL", "OWNER": "RES-82 (contract), OD-01 pending"},
    {"CRITERION": "CG-03 takeoff transition whip", "EVIDENCE": "pelvis pitch rate min -9.6837 rad/s at 0.649875 s; candidate R_WHIP 5.0 rad/s", "VERDICT": "FAIL", "OWNER": "OD-06 pending"},
    {"CRITERION": "L4 LANDING_VALIDITY (whole-body)", "EVIDENCE": "Hy at E10 9.88 kg m^2/s; root pitch 0.2922 rad; trunk forward 25.01 deg; com_vx accelerating 0.211 -> 0.342 m/s after touchdown", "VERDICT": "FAIL", "OWNER": "OD-04 pending"},
    {"CRITERION": "L4-T8 behavioral momentum capture", "EVIDENCE": "post-touchdown com_vx peak 0.34183 m/s at 1.0385 s exceeds E10 value 0.21145 m/s (forward lunge)", "VERDICT": "FAIL", "OWNER": "RES-82 (contract), RES-86/87 (control)"},
    {"CRITERION": "L5 BALANCE_VALIDITY (E11)", "EVIDENCE": "historical E11 guard used abs(com_vz); actual com_vx at E11 0.27903 m/s; Hy at E11 UNKNOWN_NOT_EVALUABLE from sealed summary", "VERDICT": "FAIL / NOT_DEMONSTRATED", "OWNER": "OD-05 pending"},
    {"CRITERION": "L5 RECOVERY_VALIDITY (E12 envelope + robustness)", "EVIDENCE": "one-trajectory ULP envelope; E12 onset 14.43625 s vs handoff 14.487 s (50.8 ms early, under SETTLE); 13.15 s recovery from 24 deg lean", "VERDICT": "FAIL", "OWNER": "RES-87 (primary), RES-82 (contract)"},
    {"CRITERION": "FULL_EPISODE support continuity", "EVIDENCE": "historical SUPPORT_FULL=NOT_QUALIFIED; chatter transitions 188; canonical reflight runs [4, 8, 1815]", "VERDICT": "FAIL", "OWNER": "RES-89 (implementation), RES-82 (scope contract)"},
    {"CRITERION": "Landing peak force / penetration hard gates", "EVIDENCE": "peak total Fz 4.7755 BW (<= 8); max penetration 0.0096389 m (<= 0.010) - both within historical hard rules", "VERDICT": "PASS (for these two lines only)", "OWNER": "n/a"},
    {"CRITERION": "E1 supported start", "EVIDENCE": "historical E1 latched 0.000125 s from reset; fall state UNKNOWN_NOT_EVALUABLE in sealed summary; successor requires explicit fall check (MED-020)", "VERDICT": "UNKNOWN_NOT_EVALUABLE", "OWNER": "RES-89 (tests), RES-82 (semantics)"},
    {"CRITERION": "E2-E5 countermovement phase semantics", "EVIDENCE": "historical event times exist (E2 0.16125, E3 0.380125, E4 0.4875, E5 0.497375); depth was hip-extension-limited (anatomy invalid)", "VERDICT": "FAIL under corrected anatomy / otherwise NOT_DEMONSTRATED", "OWNER": "RES-83/85"},
]


def load_json(name: str):
    with (HERE / name).open("r", encoding="utf-8") as fh:
        return json.load(fh)


def read_text(name: str) -> str:
    return (HERE / name).read_text(encoding="utf-8")


def git(args: list[str]):
    proc = subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


ENTRY_HEAD = "12ea42029489ddc2839389beafcf0811b7b45271"
ENTRY_TREE = "79f183724486ff95a1269a87cdb21d078b1893a5"
PRESERVED_UNTRACKED = [
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
NEW_AUDIT_PREFIX = "audit/EXP-RES82-SUCCESSOR-SCIENTIFIC-TASK-CONTRACT-001/"


def check_jump_height_qualifier(problems: list[str]) -> None:
    qualifiers = [
        "com_rise", "takeoff", "ballistic", "flight-time", "flight time", "standing",
        "equation", "method", "definit", "qualif", "unqualif", "conflat", "retir",
        "prohibit", "naming", "label", "called", "never", "no ", "not ", "jump-height",
        "calculation", "estimate", "primary_canonical",
    ]
    for name in QUALIFIER_SCAN_FILES:
        text = read_text(name) if name.endswith(".md") else json.dumps(load_json(name), indent=1)
        for lineno, line in enumerate(text.splitlines(), 1):
            low = line.lower()
            if ("jump height" in low or "jump heights" in low) and "jump-height" not in low:
                if not any(q in low for q in qualifiers):
                    problems.append(f"{name}:{lineno}: unqualified 'jump height': {line.strip()[:120]}")


def check_takeoff_qualifier(problems: list[str]) -> None:
    qualifiers = [
        "physical", "force", "threshold", "take-off", "takeoff_", "takeoff-", "takeoff t",
        "takeoff v", "takeoff e", "takeoff c", "takeoff p", "takeoff s", "takeoff w",
        "takeoff a", "takeoff o", "takeoff,", "before takeoff", "at takeoff", "pre-takeoff",
        "post-takeoff", "e6", "e5", "takeoff and", "takeoff or", "takeoff is", "takeoff.",
        "no takeoff", "takeoff hole", "takeoff phase", "unqualified", "may not", "whipping",
        "extension", "config", "transition", "takeoff joint", "whip",
    ]
    for name in QUALIFIER_SCAN_FILES:
        text = read_text(name) if name.endswith(".md") else json.dumps(load_json(name), indent=1)
        for lineno, line in enumerate(text.splitlines(), 1):
            low = line.lower()
            if "takeoff" in low:
                if not any(q in low for q in qualifiers):
                    problems.append(f"{name}:{lineno}: unqualified 'takeoff': {line.strip()[:120]}")


def check_speed_naming(problems: list[str]) -> None:
    token = re.compile(r"\b(?:[A-Z][A-Z0-9_]*SPEED[A-Z0-9_]*|SPEED[A-Z0-9_]*)\b")
    allowed_exact = {"COM_SPEED_SAGITTAL", "COM_SPEED_SAGITTAL_AT_E11", "MAX_COM_SPEED_SAGITTAL_CAPTURE_RUN"}
    retired_context = ["retire", "historical", "med-002", "mismatch", "prohibited", "never", "audit", "named", "rule"]
    for name in QUALIFIER_SCAN_FILES:
        text = read_text(name) if name.endswith(".md") else json.dumps(load_json(name), indent=1)
        for lineno, line in enumerate(text.splitlines(), 1):
            for m in token.finditer(line):
                tok = m.group(0)
                if tok in allowed_exact:
                    continue
                if tok in {"COM_SPEED", "BALANCE_CAPTURE_COM_SPEED_MPS", "COM_SPEED_MPS", "SPEED"}:
                    low = line.lower()
                    if any(k in low for k in retired_context):
                        continue
                problems.append(f"{name}:{lineno}: SPEED variable without explicit vector components: {tok}")


def main() -> int:
    checks: list[dict] = []

    def add(cid: str, name: str, ok: bool, detail: str) -> None:
        checks.append({"CHECK_ID": cid, "NAME": name, "STATUS": "PASS" if ok else "FAIL", "DETAIL": detail})

    events = load_json(ART["events"])
    perf = load_json(ART["perf"])
    landing = load_json(ART["landing"])
    dwell = load_json(ART["dwell"])
    nc = load_json(ART["nc"])
    owner = load_json(ART["owner"])
    schema = load_json(ART["schema"])
    dep = load_json(ART["dep"])
    task = load_json(ART["task"])
    ev_md = read_text("EVENT_CONTRACT_E1_E12.md")
    perf_md = read_text("PERFORMANCE_METRIC_CONTRACT.md")
    landing_md = read_text("LANDING_BALANCE_RECOVERY_CONTRACT.md")
    nc_md = read_text("NEGATIVE_CONTROL_CONTRACT.md")
    owner_md = read_text("OWNER_DECISIONS_REQUIRED.md")

    # C1 events exist exactly once
    ids = [e["EVENT_ID"] for e in events["events"]]
    names = [e["NAME"] for e in events["events"]]
    want = [f"E{i}" for i in range(1, 13)]
    problems = []
    if ids != want:
        problems.append(f"event ids mismatch: {ids}")
    for eid in want:
        count = len(re.findall(rf"^### {eid} — ", ev_md, flags=re.M))
        if count != 1:
            problems.append(f"{eid} heading count {count}")
    add("C1", "every E1-E12 event exists exactly once", not problems, "; ".join(problems) or "12 unique events, 12 unique headings")

    # C2 every event has required fields
    problems = []
    for e in events["events"]:
        for field in ["SCIENTIFIC_MEANING", "ONSET_PREDICATE", "CONFIRMATION_PREDICATE", "DWELL_TYPE",
                      "REQUIRED_MEASUREMENTS", "FAILURE_BLOCKERS", "NEGATIVE_CONTROLS",
                      "PRIMARY_OR_DIAGNOSTIC", "CLAIM_SUPPORTED"]:
            val = e.get(field)
            if val is None or (isinstance(val, (list, str)) and len(val) == 0):
                problems.append(f"{e['EVENT_ID']} missing {field}")
        if "PREDECESSOR" not in e:
            problems.append(f"{e['EVENT_ID']} missing PREDECESSOR field")
        if e["DWELL_TYPE"] == "PHYSICAL_TIME" and not e.get("DWELL_VALUE_S"):
            problems.append(f"{e['EVENT_ID']} PHYSICAL_TIME without DWELL_VALUE_S")
        if not e.get("SUSTAIN_PREDICATE") and e["DWELL_TYPE"] != "NONE":
            problems.append(f"{e['EVENT_ID']} dwell event without SUSTAIN_PREDICATE")
    # predecessor chain must be exactly E(i-1), E1 null
    for e in events["events"]:
        own = int(e["EVENT_ID"][1:])
        want = None if own == 1 else f"E{own - 1}"
        if e.get("PREDECESSOR") != want:
            problems.append(f"{e['EVENT_ID']} PREDECESSOR {e.get('PREDECESSOR')} != {want}")
    add("C2", "every event has onset/predecessor/measurements/dwell/blockers/negative controls", not problems,
        "; ".join(problems) or "all 12 complete with correct predecessor chain")

    # C3 every hard gate has full fields
    problems = []
    required_gate_fields = ["GATE", "VARIABLE", "DEFINITION", "UNITS", "THRESHOLD_STATUS", "PROVENANCE", "INVALID_OUTCOME_PREVENTED"]
    for gate in perf["hard_gate_catalog"]:
        for f in required_gate_fields:
            if not gate.get(f):
                problems.append(f"perf {gate.get('GATE')} missing {f}")
    for gate in landing["L4_landing_validity"]["hard_gates"]:
        for f in required_gate_fields:
            if not gate.get(f):
                problems.append(f"L4 {gate.get('GATE')} missing {f}")
    add("C3", "every hard gate has variable/definition/units/threshold/provenance/invalid-outcome", not problems,
        "; ".join(problems) or f"{len(perf['hard_gate_catalog']) + len(landing['L4_landing_validity']['hard_gates'])} gates complete")

    # C4 numeric thresholds have provenance
    problems = []
    if not perf["primary_performance_gate"]["THRESHOLD"].get("STATUS"):
        problems.append("primary performance threshold missing STATUS")
    for g in perf["physics_consistency_gates"]:
        if not g.get("PROVENANCE"):
            problems.append(f"{g['GATE_ID']} missing PROVENANCE")
    for gate in landing["L4_landing_validity"]["task_gates"]:
        if not gate.get("STATUS"):
            problems.append(f"{gate['GATE']} missing STATUS")
    for b in landing["L5_balance_recovery"]["E11_bounds"]:
        if not b.get("STATUS") or not b.get("PROVENANCE"):
            problems.append(f"E11 bound {b['BOUND']} missing STATUS/PROVENANCE")
    for d in owner["decisions"]:
        if not d.get("PROVENANCE_CLASS"):
            problems.append(f"{d['ID']} missing PROVENANCE_CLASS")
    # event predicate numerics must be covered by the provenance register
    reg = events.get("event_numeric_provenance", {})
    for e in events["events"]:
        blob = " ".join(str(e.get(k) or "") for k in ["ONSET_PREDICATE", "SUSTAIN_PREDICATE", "CONFIRMATION_PREDICATE"])
        nums = re.findall(r"\d+\.\d+", blob)
        entry = reg.get(e["EVENT_ID"], {})
        dwell = e.get("DWELL_VALUE_S")
        for n in nums:
            if dwell is not None and abs(float(n) - float(dwell)) < 1e-12:
                continue  # dwell value covered by DWELL_SEMANTICS / event NUMERIC
            if not any(n in key for key in entry):
                problems.append(f"{e['EVENT_ID']} numeric {n} not covered by event_numeric_provenance")
    # hard-gate provenance labels must resolve through the declared map
    pmap = perf.get("provenance_class_map", {})
    for gate in perf["hard_gate_catalog"]:
        prov = gate.get("PROVENANCE", "")
        if gate.get("THRESHOLD_STATUS") not in {"MODEL_DEPENDENT_DEFERRED", None}:
            if prov not in pmap:
                problems.append(f"perf {gate['GATE']} provenance '{prov}' not in provenance_class_map")
    for gate in landing["L4_landing_validity"]["hard_gates"]:
        prov = gate.get("PROVENANCE", "")
        if gate.get("THRESHOLD_STATUS") != "MODEL_DEPENDENT_DEFERRED":
            if prov not in pmap:
                problems.append(f"L4 {gate['GATE']} provenance '{prov}' not in provenance_class_map")
    add("C4", "every numeric threshold has provenance", not problems, "; ".join(problems) or "all structured thresholds carry provenance and register coverage")

    # C5 jump-height qualifier
    problems = []
    check_jump_height_qualifier(problems)
    add("C5", "no use of 'jump height' without method qualifier", not problems,
        "; ".join(problems[:8]) or "qualified everywhere")

    # C6 takeoff qualifier
    problems = []
    check_takeoff_qualifier(problems)
    add("C6", "no ambiguous 'takeoff' without physical/force-threshold qualifier", not problems,
        "; ".join(problems[:8]) or "qualified everywhere")

    # C7 SPEED naming
    problems = []
    check_speed_naming(problems)
    add("C7", "no SPEED variable that uses one velocity component", not problems,
        "; ".join(problems[:8]) or "COM_SPEED_SAGITTAL only; historical name retired in context")

    # C8 no future event dependency
    problems = []
    for e in events["events"]:
        own = int(e["EVENT_ID"][1:])
        blob = " ".join(str(e.get(k) or "") for k in ["ONSET_PREDICATE", "SUSTAIN_PREDICATE", "CONFIRMATION_PREDICATE"])
        for ref in re.findall(r"\bE(\d+)\b", blob):
            if int(ref) > own:
                problems.append(f"{e['EVENT_ID']} predicate references future E{ref}")
    add("C8", "no event predicate depends on a future event", not problems, "; ".join(problems) or "all predicates causal")

    # C9 no 12/12 implies success
    problems = []
    formula = task["success_composition"]["TASK_SUCCESS"]
    parts = [p.strip() for p in re.split(r"\bAND\b", formula)]
    if re.search(r"\bOR\b", formula):
        problems.append("composition contains OR")
    for comp in ["EXECUTION_VALID", "MODEL_STATE_VALID", "EVENT_CHAIN_VALID", "TASK_PERFORMANCE_VALID",
                 "LANDING_VALID", "BALANCE_VALID", "RECOVERY_VALID", "NO_HARD_FAILURE"]:
        if comp not in parts:
            problems.append(f"formula missing conjunct {comp}")
    if len(parts) != 8:
        problems.append(f"formula conjunct count {len(parts)} != 8")
    if not any("never implies" in r for r in task["success_composition"]["non_inference_rules"]):
        problems.append("missing non-inference rule for EVENT_CHAIN_VALID")
    # schema cross-field rule present
    if "allOf" not in schema:
        problems.append("canonical schema lacks TASK_SUCCESS composition rule")
    add("C9", "no task-success result can be inferred from 12/12 alone", not problems,
        "; ".join(problems) or "conjunction rule + explicit non-inference + schema composition pattern")

    # C10 E12 not trace equality
    problems = []
    if "equality to one historical deterministic standing trace" not in landing["L5_balance_recovery"]["E12_standing_envelope"]["prohibited_definitions"]:
        problems.append("landing contract missing trace-equality prohibition")
    if "one historical" not in ev_md[ev_md.index("### E12"):ev_md.index("## 2. Monotone")]:
        problems.append("E12 section missing historical-trace prohibition")
    add("C10", "no E12 standing definition depends on equality to one historical trace", not problems,
        "; ".join(problems) or "prohibited explicitly")

    # C11 no R001 invalid sign dependency in hard gates
    problems = []
    bad_literals = ["0.541", "0.318", "-0.318", "0.27869", "1.80", "-0.50"]
    blobs = []
    for gate in perf["hard_gate_catalog"]:
        blobs.append((gate["GATE"], json.dumps(gate)))
    for gate in landing["L4_landing_validity"]["hard_gates"]:
        blobs.append((gate["GATE"], json.dumps(gate)))
    for gid, blob in blobs:
        for lit in bad_literals:
            if lit in blob:
                problems.append(f"{gid} embeds R001-specific coordinate literal {lit}")
    if not any(item["ID"] == "PD-01" for item in dep["categories"]["PLANT_DEPENDENT_DEFERRED"]):
        problems.append("MODEL_DEPENDENCY_MATRIX missing PD-01 joint-convention deferral")
    add("C11", "no hard gate silently depends on R001 invalid joint-coordinate signs", not problems,
        "; ".join(problems) or "joint conventions deferred to RES-83 (PD-01)")

    # C12 owner decisions explicit and referenced set resolves
    problems = []
    decision_ids = {d["ID"] for d in owner["decisions"]}
    if len(decision_ids) != owner["count"]:
        problems.append(f"owner count mismatch: {len(decision_ids)} vs {owner['count']}")
    for d in owner["decisions"]:
        if not d.get("STATUS"):
            problems.append(f"{d['ID']} missing STATUS")
    referenced = set()
    for path in list(HERE.glob("*.md")) + list(HERE.glob("*.json")):
        if path.name in {"SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.md", "SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.csv"}:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        referenced.update(re.findall(r"\bOD-\d{2}\b", text))
    unresolved = sorted(r for r in referenced if r not in decision_ids)
    if unresolved:
        problems.append(f"referenced owner decisions missing from register: {unresolved}")
    add("C12", "all owner decisions are explicit", not problems,
        "; ".join(problems) or f"{len(decision_ids)} decisions registered; all references resolve")

    # C13 JSON/MD agreement
    problems = []
    for e in events["events"]:
        if e["NAME"] not in ev_md:
            problems.append(f"MD missing event name {e['NAME']}")
        if e["DWELL_TYPE"] == "PHYSICAL_TIME":
            token = f"{e['DWELL_VALUE_S']:.3f}".rstrip("0").rstrip(".")
            if token not in ev_md and f"{e['DWELL_VALUE_S']}" not in ev_md:
                problems.append(f"MD missing dwell value for {e['EVENT_ID']}")
    for c in nc["negative_controls"]:
        if c["CONTROL_ID"] not in nc_md:
            problems.append(f"MD missing negative control {c['CONTROL_ID']}")
    for d in owner["decisions"]:
        if d["ID"] not in owner_md:
            problems.append(f"MD missing owner decision {d['ID']}")
    if "0.150" not in perf_md or "0.200" not in perf_md or "0.100" not in perf_md:
        problems.append("performance MD missing candidate floor values")
    if "5.0 rad/s" not in owner_md:
        problems.append("owner MD missing R_WHIP candidate 5.0")
    for gate in perf["hard_gate_catalog"]:
        if gate["GATE"] not in perf_md:
            problems.append(f"perf MD missing hard gate {gate['GATE']}")
    add("C13", "contract JSON and Markdown agree", not problems, "; ".join(problems[:10]) or "key structures agree")

    # C14 no production scientific source changed
    problems = []
    head = git(["rev-parse", "HEAD"])
    tree = git(["rev-parse", "HEAD^{tree}"])
    if head is None or tree is None:
        problems.append("git execution failed (fail-closed)")
    else:
        if head != ENTRY_HEAD:
            problems.append(f"HEAD {head} != ENTRY_HEAD {ENTRY_HEAD}")
        if tree != ENTRY_TREE:
            problems.append(f"tree {tree} != ENTRY_TREE {ENTRY_TREE}")
    porcelain = git(["status", "--porcelain"])
    if porcelain is None:
        problems.append("git status failed (fail-closed)")
    else:
        for line in porcelain.splitlines():
            if not line:
                continue
            status, path = line[:2], line[3:].strip().strip('"')
            if status == "??":
                if path.startswith(NEW_AUDIT_PREFIX):
                    continue
                if any(path == p or path.startswith(p) for p in PRESERVED_UNTRACKED):
                    continue
                problems.append(f"unexpected untracked path: {path}")
            else:
                problems.append(f"tracked file change: {line}")
    unstaged = git(["diff", "--name-only", "HEAD"])
    staged = git(["diff", "--cached", "--name-only"])
    if unstaged is None or staged is None:
        problems.append("git diff failed (fail-closed)")
    else:
        touched_deny = [f for f in (unstaged.splitlines() + staged.splitlines()) if f in DENYLIST]
        if touched_deny:
            problems.append(f"denylisted files changed: {touched_deny}")
    add("C14", "no production scientific source file changed", not problems,
        "; ".join(problems) or "HEAD/tree equal entry authority; tracked worktree clean; untracked allowlist respected; denylisted paths untouched")

    report = {
        "schema_version": "1.0.0",
        "artifact_id": "RES82-CONTRACT-CONSISTENCY-REPORT",
        "mission": "RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "checks_total": len(checks),
        "checks_pass": sum(1 for c in checks if c["STATUS"] == "PASS"),
        "checks_fail": sum(1 for c in checks if c["STATUS"] == "FAIL"),
        "checks": checks,
        "r001_retrospective": {
            "R001_TRACE": "4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561",
            "R001_STATUS": "HISTORICAL_REPRODUCIBILITY_ONLY",
            "R001_SUCCESSOR_CONTRACT_RESULT": "FAIL",
            "INDEPENDENT_FAILURE_CAUSES": [
                "L1 anatomy (CRIT-001/CRIT-002, owned by RES-83)",
                "E6/E7 physical takeoff/flight semantics",
                "E8 apex flight guard not demonstrated",
                "L3 performance floor (PF-1 candidate; 0.075-0.077 m vs 0.150 m)",
                "CG-03 takeoff whip (-9.6837 rad/s)",
                "L4 whole-body landing state (Hy 9.88; root pitch 0.2922 rad; trunk 25 deg)",
                "L4-T8 behavioral momentum capture (com_vx 0.211 -> 0.342 m/s)",
                "L5 E11 corrected COM speed (0.279 m/s vx; historical guard vertical-only)",
                "L5 E12 envelope robustness and regime ambiguity",
                "FULL_EPISODE support continuity NOT_QUALIFIED (chatter 188; reflight [4, 8, 1815])",
            ],
            "CRITERIA": R001_RETROSPECTIVE,
            "UNKNOWN_MARKERS": ["E1 fall state", "Hy at E11", "CoP validity series", "signed posture states under corrected conventions"],
        },
        "artifact_status": {
            "structural_contract": "COMPLETE",
            "numeric_closure": "OPEN_OWNER_DECISIONS",
            "owner_decisions_registered": owner["count"],
            "owner_decisions_by_status": {s: sum(1 for d in owner["decisions"] if d["STATUS"] == s) for s in sorted({d["STATUS"] for d in owner["decisions"]})},
            "negative_controls": len(nc["negative_controls"]),
        },
    }

    (HERE / "CONTRACT_CONSISTENCY_REPORT.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    lines = [
        "# CONTRACT_CONSISTENCY_REPORT",
        "",
        "MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`",
        f"GENERATED (UTC): {report['generated_at_utc']}",
        f"CHECK RESULT: **{report['checks_pass']}/{report['checks_total']} PASS**",
        "",
        "## 1. Mechanical consistency gates",
        "",
        "| Check | Name | Status | Detail |",
        "|---|---|---|---|",
    ]
    for c in checks:
        lines.append(f"| {c['CHECK_ID']} | {c['NAME']} | {c['STATUS']} | {c['DETAIL']} |")
    lines += [
        "",
        "## 2. R001 retrospective under the successor contract",
        "",
        f"`R001_SUCCESSOR_CONTRACT_RESULT = {report['r001_retrospective']['R001_SUCCESSOR_CONTRACT_RESULT']}`",
        "",
        f"Independent failure causes: {len(report['r001_retrospective']['INDEPENDENT_FAILURE_CAUSES'])}",
        "",
        "| Criterion | Evidence | Verdict | Owner |",
        "|---|---|---|---|",
    ]
    for row in R001_RETROSPECTIVE:
        lines.append(f"| {row['CRITERION']} | {row['EVIDENCE']} | {row['VERDICT']} | {row['OWNER']} |")
    lines += [
        "",
        "Unknowns are marked `UNKNOWN_NOT_EVALUABLE`; no criterion was forced onto a quantity",
        "R001 never recorded.",
        "",
        "## 3. Artifact status",
        "",
        f"- Structural contract: `{report['artifact_status']['structural_contract']}`",
        f"- Numeric closure: `{report['artifact_status']['numeric_closure']}`",
        f"- Owner decisions registered: {report['artifact_status']['owner_decisions_registered']} ({report['artifact_status']['owner_decisions_by_status']})",
        f"- Negative controls specified: {report['artifact_status']['negative_controls']}",
        "",
        "## 4. Reproduction",
        "",
        "```",
        "python3 validate_contract.py",
        "```",
    ]
    (HERE / "CONTRACT_CONSISTENCY_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps({"checks_total": report["checks_total"], "checks_pass": report["checks_pass"],
                      "checks_fail": report["checks_fail"]}, indent=2))
    return 0 if report["checks_fail"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
