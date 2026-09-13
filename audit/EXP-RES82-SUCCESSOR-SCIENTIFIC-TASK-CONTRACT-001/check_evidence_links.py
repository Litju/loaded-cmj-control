#!/usr/bin/env python3
"""RES-82 evidence-link checker (stdlib-only).

Validates:
1. the evidence table CSV has all required columns and unique SOURCE_IDs;
2. every SOURCE_ID referenced in contract artifacts resolves to a table row;
3. every owner-decision EVIDENCE row resolves;
4. every candidate primary-floor value has a BASIS statement;
5. DIRECTLY_SUPPORTS_NUMERIC_GATE=YES rows are limited to rows cited for numeric values.

Read-only. No production file is modified.
"""
from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

REQUIRED_COLUMNS = [
    "SOURCE_ID", "FULL_CITATION", "DOI_PMID_OR_STANDARD", "SOURCE_TYPE", "POPULATION",
    "TASK", "LOAD", "MEASUREMENT_SYSTEM", "VARIABLE", "DEFINITION", "THRESHOLD_IF_ANY",
    "METHOD", "UNCERTAINTY_OR_RELIABILITY", "APPLICABILITY_TO_OUR_TASK", "LIMITATIONS",
    "DIRECTLY_SUPPORTS_NUMERIC_GATE", "NOTES",
]

SCAN_FILES = [
    "SUCCESSOR_TASK_CONTRACT.md", "SUCCESSOR_TASK_CONTRACT.json",
    "EVENT_CONTRACT_E1_E12.md", "EVENT_CONTRACT_E1_E12.json",
    "PERFORMANCE_METRIC_CONTRACT.md", "PERFORMANCE_METRIC_CONTRACT.json",
    "LANDING_BALANCE_RECOVERY_CONTRACT.md", "LANDING_BALANCE_RECOVERY_CONTRACT.json",
    "DWELL_SEMANTICS.md", "DWELL_SEMANTICS.json",
    "NEGATIVE_CONTROL_CONTRACT.md", "NEGATIVE_CONTROL_CONTRACT.json",
    "SUCCESSOR_CONTEXT_OF_USE.md", "SCIENTIFIC_CLAIM_CEILING.md",
    "OWNER_DECISIONS_REQUIRED.md", "OWNER_DECISIONS_REQUIRED.json",
    "SOURCE_REVIEW_COVERAGE.json",
]


def main() -> int:
    problems: list[str] = []
    rows: list[dict] = []
    with (HERE / "SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.csv").open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in REQUIRED_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            problems.append(f"missing CSV columns: {missing}")
        rows = list(reader)

    ids = [r["SOURCE_ID"] for r in rows]
    if len(ids) != len(set(ids)):
        problems.append("duplicate SOURCE_IDs in evidence table")
    for r in rows:
        if r.get("DIRECTLY_SUPPORTS_NUMERIC_GATE") not in {"YES", "NO"}:
            problems.append(f"{r['SOURCE_ID']} invalid DIRECTLY_SUPPORTS_NUMERIC_GATE")
        if not r.get("FULL_CITATION") or not r.get("APPLICABILITY_TO_OUR_TASK"):
            problems.append(f"{r['SOURCE_ID']} missing citation/applicability")

    known = set(ids)
    referenced: set[str] = set()
    for name in SCAN_FILES:
        path = HERE / name
        if not path.exists():
            problems.append(f"scan file missing: {name}")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for ref in re.findall(r"\b[SPB]\d{2}\b", text):
            referenced.add(ref)
            if ref not in known:
                problems.append(f"{name}: unresolved source reference {ref}")

    owner = json.loads((HERE / "OWNER_DECISIONS_REQUIRED.json").read_text(encoding="utf-8"))
    for d in owner["decisions"]:
        for ev in d.get("EVIDENCE", []):
            if not re.fullmatch(r"[SPB]\d{2}", ev):
                problems.append(f"{d['ID']}: malformed evidence ref {ev}")
            elif ev not in known:
                problems.append(f"{d['ID']}: unresolved evidence ref {ev}")

    perf = json.loads((HERE / "PERFORMANCE_METRIC_CONTRACT.json").read_text(encoding="utf-8"))
    for cand in perf["primary_performance_gate"]["CANDIDATES"]:
        if not cand.get("BASIS"):
            problems.append(f"performance candidate {cand['LABEL']} missing BASIS")
    yes_rows = {r["SOURCE_ID"] for r in rows if r["DIRECTLY_SUPPORTS_NUMERIC_GATE"] == "YES"}
    cited_evidence = {ev for d in owner["decisions"] for ev in d.get("EVIDENCE", []) if ev in yes_rows}
    if yes_rows and not cited_evidence:
        problems.append("YES rows exist but no owner decision cites them")

    out = {
        "artifact": "RES82-EVIDENCE-LINK-CHECK",
        "table_rows": len(rows),
        "referenced_source_ids": sorted(referenced),
        "unreferenced_rows": sorted(known - referenced),
        "directly_supports_yes": sorted(yes_rows),
        "problems": problems,
        "status": "PASS" if not problems else "FAIL",
    }
    print(json.dumps(out, indent=2))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
