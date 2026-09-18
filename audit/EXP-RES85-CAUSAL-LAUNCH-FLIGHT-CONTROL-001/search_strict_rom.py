"""RES-85D bounded deterministic strict-ROM controller search.

MISSION: RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001
LINEAR ISSUE: RES-85

The first correction attempt (the RES-85D predictive structural-ROM guard with
the RES-85C propulsion reference and guard strength) leaves the measured
trunk_pelvis peak above the frozen +0.610865 rad bound, so one deterministic
bounded LOCAL search is authorized by the mission.  No stochastic search, no
change of the acceptance rule and no authority increase are used.

Predeclared variables and levels, evaluated in this fixed order (staged
coordinate ascent):

  1. trunk_rom_guard_margin_rad  : [0.08, 0.10, 0.12, 0.15]
  2. trunk_rom_position_gain     : [2500.0, 5000.0, 8000.0]
  3. trunk_kd                    : [30.0, 45.0, 60.0]
  4. trunk_lean_frac             : [0.35, 0.30, 0.25, 0.20]
  5. trunk_extend_frac           : [0.40, 0.30, 0.25, 0.20]

The start point is the RES-85C configuration (margin 0.08, position gain 2500,
trunk_kd 30, lean 0.35, extend 0.40); the RES-85D guard mechanics
(OUTWARD_VELOCITY_BRAKING, horizon, generic gains) are fixed and not swept.

Declared objective (lexicographic):
  1. feasibility: strict structural ROM PASS (global min margin >= -1e-9) AND
     direct SYSTEM_COM H2 >= 0.150 m AND confirmed takeoff AND claim end present;
  2. maximize the minimum joint-ROM margin;
  3. minimize the declared change from the RES-85C controller configuration.

Change metric (declared scales): sum of |value - RES-85C| / scale over the five
variables with scales (0.05 rad, 4000 N*m/rad, 20 N*m*s/rad, 0.20, 0.20).

MAX_EVALUATIONS = 30.  The search is run twice and must be byte identical in
evaluation order, candidate metrics, selected candidate and final config.
"""

from __future__ import annotations

import hashlib
import json
import multiprocessing as mp
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SRC = REPO / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import build_evidence as BE  # noqa: E402, I001

from loaded_cmj.v3.controller import V3ControllerConfig  # noqa: E402
from loaded_cmj.v3.launch_runtime import run_launch_episode  # noqa: E402

MISSION_ID = "RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001"
HORIZON_S = 3.0
MAX_EVALUATIONS = 30

RES85C_VALUES: dict[str, float] = {
    "trunk_rom_guard_margin_rad": 0.08,
    "trunk_rom_position_gain": 2500.0,
    "trunk_kd": 30.0,
    "trunk_lean_frac": 0.35,
    "trunk_extend_frac": 0.40,
}

VARIABLES: tuple[tuple[str, tuple[float, ...]], ...] = (
    ("trunk_rom_guard_margin_rad", (0.08, 0.10, 0.12, 0.15)),
    ("trunk_rom_position_gain", (2500.0, 5000.0, 8000.0)),
    ("trunk_kd", (30.0, 45.0, 60.0)),
    ("trunk_lean_frac", (0.35, 0.30, 0.25, 0.20)),
    ("trunk_extend_frac", (0.40, 0.30, 0.25, 0.20)),
)

CHANGE_SCALES: dict[str, float] = {
    "trunk_rom_guard_margin_rad": 0.05,
    "trunk_rom_position_gain": 4000.0,
    "trunk_kd": 20.0,
    "trunk_lean_frac": 0.20,
    "trunk_extend_frac": 0.20,
}

H_ANTI_TRIVIALITY_FLOOR_M = 0.150
STRICT_ROM_TOLERANCE_RAD = 1.0e-9


def _worker(payload: tuple[int, dict[str, float]]) -> tuple[int, dict[str, Any]]:
    index, overrides = payload
    config = replace(V3ControllerConfig(), **overrides)
    episode = run_launch_episode(horizon_s=HORIZON_S, controller_config=config)
    audit = BE.strict_structural_rom_audit(episode)
    apex = episode.events.get("apex_h2") or {}
    confirmation = episode.events.get("takeoff_confirmation") or {}
    occurrence = episode.events.get("takeoff_occurrence") or {}
    h2 = apex.get("h2_support_m")
    feasible = bool(
        audit["status"] == "PASS"
        and h2 is not None
        and float(h2) >= H_ANTI_TRIVIALITY_FLOOR_M
        and confirmation.get("confirmed") is True
        and episode.events.get("res85_claim_end") is not None)
    change = sum(abs(float(overrides[name]) - RES85C_VALUES[name])
                 / CHANGE_SCALES[name] for name in RES85C_VALUES)
    return index, {
        "index": index,
        "overrides": {name: float(overrides[name]) for name in sorted(overrides)},
        "config": {field: getattr(config, field)
                   for field in config.__dataclass_fields__},
        "episode_status": episode.status,
        "fault": episode.fault,
        "h2_m": None if h2 is None else float(h2),
        "takeoff_vz_m_s": apex.get("takeoff_vz_m_s"),
        "takeoff_occurrence_index": occurrence.get("native_index"),
        "takeoff_confirmation": bool(confirmation.get("confirmed")),
        "claim_end_sample": (None if episode.events.get("res85_claim_end") is None
                             else int(episode.events["res85_claim_end"]["sample"])),
        "strict_rom_status": audit["status"],
        "global_min_structural_rom_margin_rad":
            audit["global_min_structural_rom_margin_rad"],
        "per_joint_min_margin_rad": {
            name: channel["min_rom_margin_rad"]
            for name, channel in sorted(audit["channels"].items())},
        "rom_failures": audit["failures"],
        "feasible": feasible,
        "change_from_res85c": change,
    }


def _rank(result: dict[str, Any]) -> tuple[float, float, float]:
    """Declared lexicographic rank: feasibility, min margin, minimal change."""
    return (
        1.0 if result["feasible"] else 0.0,
        float(result["global_min_structural_rom_margin_rad"]),
        -float(result["change_from_res85c"]),
    )


def _evaluate(payloads: list[tuple[int, dict[str, float]]],
              workers: int) -> dict[int, dict[str, Any]]:
    with mp.Pool(processes=workers) as pool:
        results = dict(pool.map(_worker, payloads))
    return results


def run_search(workers: int = 6) -> dict[str, Any]:
    """One deterministic staged coordinate-ascent pass (declared order)."""
    evaluations: list[dict[str, Any]] = []
    evaluated: dict[tuple[tuple[str, float], ...], int] = {}

    def key_of(overrides: dict[str, float]) -> tuple[tuple[str, float], ...]:
        return tuple(sorted((k, float(v)) for k, v in overrides.items()))

    def ensure(overrides_list: list[dict[str, float]]) -> list[dict[str, Any]]:
        payloads: list[tuple[int, dict[str, float]]] = []
        pending: list[tuple[int, tuple[tuple[str, float], ...]]] = []
        for overrides in overrides_list:
            key = key_of(overrides)
            if key in evaluated:
                continue
            if len(evaluations) + len(payloads) >= MAX_EVALUATIONS:
                raise RuntimeError("declared evaluation cap reached")
            index = len(evaluations) + len(payloads)
            payloads.append((index, overrides))
            pending.append((index, key))
        if payloads:
            results = _evaluate(payloads, workers)
            for index, key in pending:
                evaluated[key] = index
                evaluations.append(results[index])
        return [evaluations[evaluated[key_of(overrides)]]
                for overrides in overrides_list]

    best = ensure([dict(RES85C_VALUES)])[0]
    for variable, levels in VARIABLES:
        candidates: list[dict[str, float]] = []
        for level in levels:
            candidate = dict(best["overrides"])
            candidate[variable] = float(level)
            candidates.append(candidate)
        stage_results = ensure(candidates)
        stage_best = max(stage_results, key=_rank)
        if _rank(stage_best) > _rank(best):
            best = stage_best
    selected = {
        "index": best["index"],
        "overrides": best["overrides"],
        "config": best["config"],
        "h2_m": best["h2_m"],
        "strict_rom_status": best["strict_rom_status"],
        "global_min_structural_rom_margin_rad":
            best["global_min_structural_rom_margin_rad"],
        "change_from_res85c": best["change_from_res85c"],
        "feasible": best["feasible"],
    }
    return {
        "schema_version": "1.0.0",
        "mission": MISSION_ID,
        "linear_issue": "RES-85",
        "predeclared": {
            "variables": {name: list(levels) for name, levels in VARIABLES},
            "start_point": RES85C_VALUES,
            "evaluation_order": (
                "staged coordinate ascent over the declared variable order "
                "and level order; the start point is evaluated first"),
            "objective": (
                "1 feasible (strict structural ROM PASS and H2 >= 0.150 m and "
                "confirmed takeoff and claim end); 2 maximize the minimum "
                "joint-ROM margin; 3 minimize the declared change from RES-85C"),
            "change_metric": {
                "kind": "normalized L1 over the five declared variables",
                "scales": CHANGE_SCALES,
                "res85c_values": RES85C_VALUES,
            },
            "max_evaluations": MAX_EVALUATIONS,
            "h_anti_triviality_floor_m": H_ANTI_TRIVIALITY_FLOOR_M,
            "strict_rom_tolerance_rad": STRICT_ROM_TOLERANCE_RAD,
        },
        "total_evaluations": len(evaluations),
        "evaluations": evaluations,
        "selected": selected,
        "winner_config": selected["config"],
    }


def _canonical_payload(search: dict[str, Any]) -> str:
    payload = {k: v for k, v in search.items() if k != "two_run_identity"}
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--out", type=Path, default=HERE / "STRICT_ROM_SEARCH.json")
    args = parser.parse_args(argv)

    first = run_search(workers=args.workers)
    second = run_search(workers=args.workers)
    first_sha = hashlib.sha256(_canonical_payload(first).encode("utf-8")).hexdigest()
    second_sha = hashlib.sha256(_canonical_payload(second).encode("utf-8")).hexdigest()
    identity = {
        "runs": 2,
        "byte_identical": bool(first_sha == second_sha),
        "sha256": [first_sha, second_sha],
        "canonical_encoding": ("json(indent=2, sort_keys=True) + trailing newline, "
                               "payload without the two_run_identity field"),
        "compared": ["evaluation_order", "candidate_metrics", "selected_candidate",
                     "final_controller_config"],
    }
    first["two_run_identity"] = identity
    args.out.write_text(json.dumps(first, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "total_evaluations": first["total_evaluations"],
        "selected_index": first["selected"]["index"],
        "selected_overrides": first["selected"]["overrides"],
        "h2_m": first["selected"]["h2_m"],
        "strict_rom_status": first["selected"]["strict_rom_status"],
        "global_min_margin": first["selected"][
            "global_min_structural_rom_margin_rad"],
        "feasible": first["selected"]["feasible"],
        "two_run_identity": identity["byte_identical"],
    }, indent=2))
    return 0 if identity["byte_identical"] and first["selected"]["feasible"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
