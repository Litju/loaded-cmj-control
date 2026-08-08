#!/usr/bin/env python3
"""Run the frozen Generation-1 controller and write replayable artifacts."""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
from typing import Any

import numpy as np

from loaded_cmj.runtime.engine import run_rollout


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if hasattr(value, "items"):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {field.name: _jsonable(getattr(value, field.name)) for field in dataclasses.fields(value)}
    if hasattr(value, "value") and type(value).__module__ == "enum":
        return value.value
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/loaded-cmj-gen1"))
    parser.add_argument("--attempt-id", default="loaded-cmj-gen1")
    parser.add_argument("--drop-privileges", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    result = run_rollout(
        root / "src/loaded_cmj/control/pfip.py",
        attempt_id=args.attempt_id,
        policy_spec_path=root / "src/loaded_cmj/control/policy_spec.json",
        drop_privileges=args.drop_privileges,
    )
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {name: _jsonable(getattr(result, name)) for name in (
        "outcome",
        "termination_reason",
        "completed_steps",
        "objective_completed",
        "metrics",
        "attempt_id",
        "model_revision",
        "runtime_revision",
        "evaluation_outcome",
        "physical_metrics",
        "events",
        "fault_data",
        "trace_identity",
        "sample_count",
        "first_sample_time_s",
        "final_sample_time_s",
        "completed_control_steps",
        "completed_physics_steps",
        "simulated_duration_s",
        "rollout_valid",
        "episode_terminated",
        "complete_rollout",
        "evidence_identity",
    )}
    (args.output / "result.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    with (args.output / "trace.jsonl").open("w") as handle:
        for trace_index, sample in enumerate(result.trace):
            record = _jsonable(sample)
            record["trace_index"] = trace_index
            handle.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    print(json.dumps({
        "termination": result.termination_reason.value,
        "events": _jsonable(result.events),
        "control_steps": result.completed_control_steps,
        "physics_steps": result.completed_physics_steps,
        "duration_s": result.simulated_duration_s,
        "trace_identity": result.trace_identity,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
