#!/usr/bin/env python3
"""F3.4 qualification of the single canonical force-time metric owner."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
if str(_TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(_TASK_ROOT))

from loaded_cmj.biomechanics.events import CMJEventDetector  # noqa: E402
from loaded_cmj.biomechanics.metrics import derive_force_time_metrics  # noqa: E402


def load_fixture_module() -> Any:
    path = _TASK_ROOT / "tests" / "qualification_cmj_events.py"
    spec = importlib.util.spec_from_file_location("qualification_cmj_events_fixture", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load event fixture module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    fixtures = load_fixture_module()
    samples = fixtures.valid_trace()
    event_result = CMJEventDetector().evaluate(samples)
    metrics = derive_force_time_metrics(
        samples,
        event_result.events,
        landing_window_end_s=event_result.events["impact_absorption"],
    )
    propulsion = metrics["propulsion"]["reversal_to_takeoff_interval"]
    event_propulsion = event_result.raw_metrics["propulsion"]
    bad_events = dict(event_result.events)
    bad_events["valid_landing"] = bad_events["valid_takeoff"] - 0.001
    landing_order_rejected = False
    try:
        derive_force_time_metrics(samples, bad_events)
    except ValueError:
        landing_order_rejected = True
    propulsion_nodes = np.asarray(metrics["intervals"]["propulsion_nodes_s"], dtype=np.float64)
    landing_nodes = np.asarray(metrics["intervals"]["landing_nodes_s"], dtype=np.float64)
    checks = {
        "canonical_owner_used": metrics["contract_version"] == "LCMJ-V1-F3-FORCE-TIME-1.0.0",
        "propulsive_impulse_matches_event_projection": abs(float(propulsion["J_prop_Ns"]) - float(event_propulsion["J_prop_Ns"])) <= 1e-12,
        "propulsive_interval_is_reversal_to_takeoff": float(propulsion["interval_start_time_s"]) == event_result.events["upward_reversal"] and float(propulsion["interval_end_time_s"]) == event_result.events["valid_takeoff"],
        "landing_window_is_post_takeoff": bool(np.min(landing_nodes) > np.max(propulsion_nodes)),
        "landing_propulsion_leak_free": bool(metrics["landing_propulsion_leak"]["pass"]),
        "loading_rate_frozen_raw_8khz": metrics["landing"]["loading_rate"]["raw_force"] and metrics["landing"]["loading_rate"]["filter"] == "none; raw 8 kHz force" and metrics["landing"]["loading_rate"]["loading_rate_N_per_s"] >= 0.0,
        "asymmetry_present": "bilateral_asymmetry" in metrics["propulsion"] and "left_right_asymmetry" in metrics["landing"],
        "landing_order_negative_control": landing_order_rejected,
        "raw_force_owner_retained": metrics["force_time_morphology"]["raw_wrench_owner"] == "BiomechanicalSample.plate_wrench_N_Nm",
    }
    report = {
        "suite": "F3.4-FORCE-TIME-METRICS",
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "event_digest_projection": {
            "events": event_result.events,
            "event_propulsive_J_Ns": event_propulsion["J_prop_Ns"],
            "canonical_propulsive_J_Ns": propulsion["J_prop_Ns"],
        },
        "metrics": metrics,
    }
    output = args.evidence_root / "force_time_metric_validation.json"
    output.write_text(json.dumps(report, indent=2, default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": report["result"], "output": str(output), "checks": checks}, sort_keys=True))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
