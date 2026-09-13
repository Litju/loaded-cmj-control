#!/usr/bin/env python3
"""RES-82 schema validator (stdlib-only).

Validates:
1. every JSON artifact parses;
2. the canonical output schema structure (required keys, result properties,
   event pattern, metric catalog);
3. referential integrity of x-metric-classes against the metrics catalog;
4. optional JSON-Schema validation when the `jsonschema` package is available.

Read-only. No production file is modified.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

JSON_FILES = [
    "SUCCESSOR_TASK_CONTRACT.json",
    "EVENT_CONTRACT_E1_E12.json",
    "PERFORMANCE_METRIC_CONTRACT.json",
    "LANDING_BALANCE_RECOVERY_CONTRACT.json",
    "DWELL_SEMANTICS.json",
    "NEGATIVE_CONTROL_CONTRACT.json",
    "CANONICAL_OUTPUT_SCHEMA.json",
    "MODEL_DEPENDENCY_MATRIX.json",
    "OWNER_DECISIONS_REQUIRED.json",
    "SOURCE_REVIEW_COVERAGE.json",
    "CONTRACT_CONSISTENCY_REPORT.json",
]


def main() -> int:
    problems: list[str] = []
    parsed: dict[str, object] = {}

    for name in JSON_FILES:
        path = HERE / name
        if not path.exists():
            problems.append(f"missing JSON artifact: {name}")
            continue
        try:
            parsed[name] = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            problems.append(f"invalid JSON: {name}: {exc}")

    schema = parsed.get("CANONICAL_OUTPUT_SCHEMA.json")
    if not isinstance(schema, dict):
        problems.append("canonical schema artifact is not a JSON object")
    else:
        if schema.get("$schema", "").find("2020-12") == -1:
            problems.append("canonical schema is not JSON Schema 2020-12")
        root_required = set(schema.get("required", []))
        for key in ["schema_version", "artifact_id", "candidate_id", "provenance", "events", "result", "metrics", "support_continuity", "hard_gates", "consistency_gates", "owner_decisions_pending"]:
            if key not in root_required:
                problems.append(f"canonical schema missing required root key {key}")
        result_props = schema["properties"]["result"]["properties"]
        for key in ["EXECUTION_VALID", "MODEL_STATE_VALID", "EVENT_CHAIN_VALID", "TASK_PERFORMANCE_VALID",
                    "LANDING_VALID", "BALANCE_VALID", "RECOVERY_VALID", "NO_HARD_FAILURE", "TASK_SUCCESS", "TASK_SUCCESS_RULE"]:
            if key not in result_props:
                problems.append(f"canonical result missing {key}")
        metric_props = schema["properties"]["metrics"]["properties"]
        classes = schema.get("x-metric-classes", {})
        seen: dict[str, str] = {}
        for cls, names in classes.items():
            for n in names:
                if n in seen:
                    problems.append(f"metric {n} classified twice: {seen[n]} and {cls}")
                seen[n] = cls
                if n not in metric_props:
                    problems.append(f"x-metric-classes {cls} references unknown metric {n}")
        # every required metric is described and classified exactly once
        for n in schema["properties"]["metrics"]["required"]:
            if n not in metric_props:
                problems.append(f"required metric without property description: {n}")
            if n not in seen:
                problems.append(f"required metric without a class: {n}")
        # cross-field composition rule present
        if "allOf" not in schema:
            problems.append("canonical schema lacks TASK_SUCCESS cross-field composition rule")
        if "x-open-decision-rule" not in schema or "x-full-episode-rule" not in schema:
            problems.append("canonical schema lacks open-decision/full-episode normative rules")
        # result flags duplicated in metrics.TASK_SUCCESS_COMPONENTS must carry the same keys
        comp_props = metric_props.get("TASK_SUCCESS_COMPONENTS", {}).get("properties", {})
        for key in ["EXECUTION_VALID", "MODEL_STATE_VALID", "EVENT_CHAIN_VALID", "TASK_PERFORMANCE_VALID",
                    "LANDING_VALID", "BALANCE_VALID", "RECOVERY_VALID", "NO_HARD_FAILURE", "TASK_SUCCESS"]:
            if key not in comp_props:
                problems.append(f"TASK_SUCCESS_COMPONENTS missing {key}")
        # FULL_EPISODE enum must allow NOT_EVALUABLE
        fe = schema["properties"]["support_continuity"]["properties"]["FULL_EPISODE"].get("enum", [])
        if "NOT_EVALUABLE" not in fe:
            problems.append("support_continuity.FULL_EPISODE enum lacks NOT_EVALUABLE")

    events = parsed.get("EVENT_CONTRACT_E1_E12.json")
    if isinstance(events, dict):
        ids = [e.get("EVENT_ID") for e in events.get("events", [])]
        if ids != [f"E{i}" for i in range(1, 13)]:
            problems.append(f"event ids not E1..E12 exactly once: {ids}")

    task = parsed.get("SUCCESSOR_TASK_CONTRACT.json")
    if isinstance(task, dict):
        conv = task["success_composition"]["TASK_SUCCESS"]
        if "EVENT_CHAIN_VALID" not in conv or "TASK_PERFORMANCE_VALID" not in conv:
            problems.append("TASK_SUCCESS formula incomplete")

    owner = parsed.get("OWNER_DECISIONS_REQUIRED.json")
    if isinstance(owner, dict):
        n = len(owner.get("decisions", []))
        if n != owner.get("count"):
            problems.append(f"owner decision count mismatch: {n} vs {owner.get('count')}")

    # Optional strict validation with jsonschema when available.
    jsonschema_note = "jsonschema package not installed; structural checks only"
    instance_results: list[dict] = []
    try:
        import jsonschema  # type: ignore
        jsonschema.Draft202012Validator.check_schema(schema)
        validator = jsonschema.Draft202012Validator(schema)

        def build_valid_instance() -> dict:
            inst = {
                "schema_version": "1.0.0",
                "artifact_id": "RES82-EXECUTED-INSTANCE-SMOKE",
                "candidate_id": "SMOKE",
                "provenance": {
                    "ENTRY_HEAD": "0" * 40,
                    "ENTRY_TREE": "0" * 40,
                    "MODEL_IDENTITY": "smoke",
                    "RUNTIME_IDENTITY": "smoke",
                    "TRACE_SHA256": "0" * 64,
                },
                "events": {},
                "result": {
                    "EXECUTION_VALID": False, "MODEL_STATE_VALID": False, "EVENT_CHAIN_VALID": False,
                    "TASK_PERFORMANCE_VALID": False, "LANDING_VALID": False, "BALANCE_VALID": False,
                    "RECOVERY_VALID": False, "NO_HARD_FAILURE": False, "TASK_SUCCESS": False,
                    "TASK_SUCCESS_RULE": "EXECUTION_VALID AND MODEL_STATE_VALID AND EVENT_CHAIN_VALID AND TASK_PERFORMANCE_VALID AND LANDING_VALID AND BALANCE_VALID AND RECOVERY_VALID AND NO_HARD_FAILURE (and RECOVERY_VALID internally requires FULL_EPISODE QUALIFIED)",
                    "CANDIDATE_CREDIBILITY_VALID": None,
                },
                "metrics": {},
                "support_continuity": {"SUPPORT_CONTINUITY_SCOPE_RESULTS": [], "FULL_EPISODE": "QUALIFIED"},
                "hard_gates": [],
                "consistency_gates": [],
                "owner_decisions_pending": [],
            }
            for i in range(1, 13):
                inst["events"][f"E{i}"] = {
                    "NAME": f"e{i}", "CONFIRMED": False, "ONSET_SAMPLE": None, "FIRST_TRUE_TIME": None,
                    "CONFIRMATION_SAMPLE": None, "CONFIRMATION_TIME": None, "DWELL_TYPE": "NONE",
                    "REQUIRED_DURATION": None, "K_D": None, "vDWELL": None, "BLOCKERS": [],
                }
            for name in schema["properties"]["metrics"]["required"]:
                if name == "TASK_SUCCESS_COMPONENTS":
                    inst["metrics"][name] = {k: False for k in ["EXECUTION_VALID", "MODEL_STATE_VALID", "EVENT_CHAIN_VALID",
                        "TASK_PERFORMANCE_VALID", "LANDING_VALID", "BALANCE_VALID", "RECOVERY_VALID", "NO_HARD_FAILURE", "TASK_SUCCESS"]}
                else:
                    inst["metrics"][name] = None
            return inst

        def errs(instance: dict) -> list[str]:
            return [f"{list(e.path)}: {e.message}" for e in validator.iter_errors(instance)]

        valid = build_valid_instance()
        valid_errors = errs(valid)
        instance_results.append({"INSTANCE": "SMOKE_VALID_ALL_FALSE", "EXPECTED": "valid", "ERRORS": valid_errors,
                                 "STATUS": "PASS" if not valid_errors else "FAIL"})

        m1 = json.loads(json.dumps(valid))
        m1["result"].update({"TASK_SUCCESS": True, "EVENT_CHAIN_VALID": True})
        m1["metrics"]["TASK_SUCCESS_COMPONENTS"].update({"TASK_SUCCESS": True, "EVENT_CHAIN_VALID": True})
        e1 = errs(m1)
        instance_results.append({"INSTANCE": "NEG_TASK_SUCCESS_WITH_FALSE_COMPONENTS", "EXPECTED": "invalid",
                                 "ERRORS": e1[:3], "STATUS": "PASS" if e1 else "FAIL"})

        m2 = json.loads(json.dumps(valid))
        m2["result"] = {k: True for k in ["EXECUTION_VALID", "MODEL_STATE_VALID", "EVENT_CHAIN_VALID",
            "TASK_PERFORMANCE_VALID", "LANDING_VALID", "BALANCE_VALID", "RECOVERY_VALID", "NO_HARD_FAILURE", "TASK_SUCCESS"]}
        m2["result"]["TASK_SUCCESS_RULE"] = valid["result"]["TASK_SUCCESS_RULE"]
        m2["result"]["CANDIDATE_CREDIBILITY_VALID"] = None
        m2["metrics"]["TASK_SUCCESS_COMPONENTS"] = {k: True for k in m2["metrics"]["TASK_SUCCESS_COMPONENTS"]}
        m2["support_continuity"]["FULL_EPISODE"] = "NOT_QUALIFIED"
        e2 = errs(m2)
        instance_results.append({"INSTANCE": "NEG_TASK_SUCCESS_WITH_FULL_EPISODE_NOT_QUALIFIED", "EXPECTED": "invalid",
                                 "ERRORS": e2[:3], "STATUS": "PASS" if e2 else "FAIL"})

        m3 = json.loads(json.dumps(m2))
        m3["support_continuity"]["FULL_EPISODE"] = "QUALIFIED"
        m3["owner_decisions_pending"] = [{"OWNER_DECISION": "OD-01", "STATUS": "OPEN"}]
        e3 = errs(m3)
        instance_results.append({"INSTANCE": "NEG_TASK_SUCCESS_WITH_OPEN_OWNER_DECISION", "EXPECTED": "invalid",
                                 "ERRORS": e3[:3], "STATUS": "PASS" if e3 else "FAIL"})

        failures = [r["INSTANCE"] for r in instance_results if r["STATUS"] == "FAIL"]
        if failures:
            problems.append(f"instance negative controls failed to reject: {failures}")
        jsonschema_note = "jsonschema meta-validation passed; 4 instance-level fixtures executed (1 valid, 3 must-reject)"
    except ImportError:
        pass
    except Exception as exc:  # noqa: BLE001
        problems.append(f"jsonschema validation failed: {exc}")

    out = {
        "artifact": "RES82-SCHEMA-VALIDATION",
        "json_files_checked": len(JSON_FILES),
        "problems": problems,
        "status": "PASS" if not problems else "FAIL",
        "jsonschema": jsonschema_note,
        "instance_negative_controls": instance_results,
    }
    print(json.dumps(out, indent=2))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
