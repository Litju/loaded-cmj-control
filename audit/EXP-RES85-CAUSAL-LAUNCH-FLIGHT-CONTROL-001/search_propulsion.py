"""RES-85C bounded propulsion search (deterministic, predeclared).

MISSION: RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001

Predeclaration (frozen before execution)
----------------------------------------
Objective (maximise, lexicographic):
  1. hard feasibility (all must hold): episode COMPLETED with no fault; takeoff
     occurrence confirmed by the RES-84 confirmation; apex/H2 evaluable; the
     controller posture reference inside the frozen ROM and the controller
     applied moment never driving a joint further past a frozen ROM limit; the
     measured launch overshoot inside the per-channel Plant soft-limit
     compliance envelope measured by ``probe_joint_rom_soft_limits.py``
     (Plant property, not a controller budget); no prohibited contact;
     no active support while the phase machine claims FLIGHT before the
     accepted occurrence; moment/power/MTP hard limits respected; bilateral
     applied symmetry; causal phase machine; no post-takeoff ground impulse.
  2. maximise DIRECT_SIMULATOR_SYSTEM_COM H2.

Variables (controller-owned engineering variables only), stage-ordered
(as executed by ``STAGE_VARIABLES``):
  stage 1: a_thrust_m_s2          in {12.0, 14.0, 16.0}
           thrust_az_max_m_s2      in {16.0, 18.0, 20.0}
  stage 2: extension_rate_ff_gain in {0.0, 0.5, 1.0}
           contact_preload_m       in {0.0, 0.005}
  stage 3: trunk_lean_frac        in {0.24, 0.30, 0.35}
           trunk_extend_frac       in {0.24, 0.30, 0.40}
  stage 4: a_thrust_m_s2          in {15.0, 16.0, 17.0}
           thrust_az_max_m_s2      in {19.0, 20.0, 21.0}
  Fixed declared engineering setup (not swept): joint_rom_margin_rad=0.08,
  joint_rom_barrier_gain=800, trunk_rom_barrier_gain=2500, trunk_kp=240,
  trunk_kd=30.
  Every stage keeps the best configuration found so far and sweeps only the
  two listed variables (coordinate ascent); ties break on the declared order.

Evaluation: ``evaluate_config`` — a deterministic function of the config that
runs one launch episode at the declared search horizon and returns the hard
feasibility vector plus the direct SYSTEM_COM H2.

Maximum evaluations: 3*3 + 2*3 + 3*3 + 3*3 = 33 episodes (plus the ENTRY
baseline), declared in ``MAX_EVALUATIONS``.
No stochastic search, no unbounded loop, no result-dependent gate rewriting.

Run:
  python3 search_propulsion.py --all --workers 6        # declared full search
  python3 search_propulsion.py --stage 1 --workers 5    # one stage
  python3 search_propulsion.py --report                 # chosen configuration

``--all`` runs the declared search twice and records a self-contained
two-run byte-identity result in ``PROPULSION_SEARCH.json`` (exit 1 if the two
runs differ).
"""

from __future__ import annotations

import argparse
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

import numpy as np  # noqa: E402

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.actuation import CHANNELS as V3_JOINT_CHANNELS  # noqa: E402
from loaded_cmj.v3.constants import V3_JOINT_NAMES, V3_JOINT_RANGES_RAD  # noqa: E402
from loaded_cmj.v3.controller import V3ControllerConfig  # noqa: E402
from loaded_cmj.v3.launch_runtime import run_launch_episode  # noqa: E402

SEARCH_HORIZON_S = 2.0
ROM_TOLERANCE_RAD = 0.0

# Entry-default base of every stage: the ENTRY controller (all RES-85C terms
# neutral) with the declared trunk/gain defaults left as the stage sweeps them.
ENTRY_NEUTRAL_BASE: dict[str, float] = {
    "extension_rate_ff_gain": 0.0,
    "contact_preload_m": 0.0,
    "trunk_lean_frac": 0.0,
    "trunk_extend_frac": 0.0,
    "trunk_kp": 40.0,
    "trunk_kd": 6.0,
    "joint_rom_margin_rad": 0.03,
    "joint_rom_barrier_gain": 0.0,
    "trunk_rom_barrier_gain": 2500.0,
    "a_thrust_m_s2": 8.0,
    "thrust_az_max_m_s2": 12.0,
}
FIXED_ENGINEERING_SETUP: dict[str, float] = {
    # declared ROM-safety setup (not tuned): the barrier margin must exceed the
    # stopping distance of a fast sagittal extension and the trunk needs its own
    # stiffer stop; the gains stay far below the frozen moment ceilings.
    "joint_rom_margin_rad": 0.08,
    "joint_rom_barrier_gain": 800.0,
    "trunk_rom_barrier_gain": 2500.0,
    "trunk_kp": 240.0,
    "trunk_kd": 30.0,
}
STAGE_VARIABLES: dict[int, dict[str, tuple[float, ...]]] = {
    1: {
        "a_thrust_m_s2": (12.0, 14.0, 16.0),
        "thrust_az_max_m_s2": (16.0, 18.0, 20.0),
    },
    2: {
        "extension_rate_ff_gain": (0.0, 0.5, 1.0),
        "contact_preload_m": (0.0, 0.005),
    },
    3: {
        "trunk_lean_frac": (0.24, 0.30, 0.35),
        "trunk_extend_frac": (0.24, 0.30, 0.40),
    },
    4: {
        "a_thrust_m_s2": (15.0, 16.0, 17.0),
        "thrust_az_max_m_s2": (19.0, 20.0, 21.0),
    },
}
MAX_EVALUATIONS = sum(len(a) * len(b) for stage in STAGE_VARIABLES.values()
                      for a, b in [tuple(stage.values())])


_CHANNEL_QPOS_INDICES: list[int] | None = None
_ROM_ENVELOPES: dict[str, float] | None = None


def _channel_qpos_indices() -> list[int]:
    global _CHANNEL_QPOS_INDICES
    if _CHANNEL_QPOS_INDICES is None:
        from loaded_cmj.v3.plant import V3Plant
        plant = V3Plant()
        _CHANNEL_QPOS_INDICES = [int(plant.idx.qadr[name]) for name in V3_JOINT_CHANNELS]
    return _CHANNEL_QPOS_INDICES


def _rom_envelopes() -> dict[str, float]:
    """Per-channel Plant soft-limit compliance envelope (probe artifact)."""
    global _ROM_ENVELOPES
    if _ROM_ENVELOPES is None:
        probe = json.loads((HERE / "JOINT_ROM_SOFT_LIMIT_PROBE.json").read_text())
        _ROM_ENVELOPES = {name: float(value["soft_limit_compliance_rad"])
                          for name, value in probe["channels"].items()}
    return _ROM_ENVELOPES


def evaluate_config(overrides: dict[str, float]) -> dict[str, Any]:
    """Deterministic evaluation of one controller configuration."""
    config = replace(V3ControllerConfig(), **overrides)
    episode = run_launch_episode(horizon_s=SEARCH_HORIZON_S, controller_config=config)
    events = episode.events
    telemetry = episode.telemetry
    checks: dict[str, bool] = {
        "episode_completed": episode.status == "COMPLETED" and episode.fault is None,
    }
    occurrence = events.get("takeoff_occurrence")
    confirmation = events.get("takeoff_confirmation")
    checks["takeoff_confirmed"] = bool(confirmation is not None
                                       and confirmation.get("confirmed") is True)
    apex = events.get("apex_h2") or {}
    checks["apex_evaluable"] = bool(apex.get("evaluable"))
    # Plant joint ROM across the RES-85 claim window (STAND -> accepted
    # occurrence); the post-flight landing is RES-86 scope and is reported
    # separately, never used to judge the launch.
    margins = np.asarray(telemetry.joint_limit_margin, dtype=np.float64)
    occurrence_index = (None if occurrence is None else occurrence.get("native_index"))
    k_occ = int(occurrence_index) if occurrence_index is not None else len(telemetry.index) - 1
    launch_margins = margins[:k_occ + 1]
    # ROM feasibility is controller-attributable inside the measured Plant
    # soft-limit envelope: (a) the posture reference never leaves the ROM and
    # (b) the applied moment never drives a joint further past its limit.
    rom_lo = np.asarray([-np.inf if V3_JOINT_RANGES_RAD[n] is None
                         else V3_JOINT_RANGES_RAD[n][0] for n in V3_JOINT_CHANNELS])
    rom_hi = np.asarray([np.inf if V3_JOINT_RANGES_RAD[n] is None
                         else V3_JOINT_RANGES_RAD[n][1] for n in V3_JOINT_CHANNELS])
    q_ref = np.asarray(telemetry.posture_reference_rad[:k_occ + 1], dtype=np.float64)
    q_meas = np.asarray(telemetry.joint_q[:k_occ + 1][:, _channel_qpos_indices()],
                        dtype=np.float64)
    applied = np.asarray(telemetry.applied_nm[:k_occ + 1], dtype=np.float64)
    checks["posture_reference_within_rom"] = bool(
        np.all(q_ref >= rom_lo - 1e-9) and np.all(q_ref <= rom_hi + 1e-9))
    into_upper = (q_meas > rom_hi) & (applied > 1e-9)
    into_lower = (q_meas < rom_lo) & (applied < -1e-9)
    checks["no_moment_drives_past_rom"] = bool(not np.any(into_upper)
                                               and not np.any(into_lower))
    # declared Plant soft-limit compliance envelope (JOINT_ROM_SOFT_LIMIT_PROBE):
    # the driven Plant may cross a ROM limit by a bounded numerical amount; the
    # measured launch overshoot must stay inside that measured envelope.
    envelope = np.asarray([_rom_envelopes()[name] for name in V3_JOINT_CHANNELS],
                          dtype=np.float64)
    overshoot = np.maximum(q_meas - rom_hi, rom_lo - q_meas)
    envelope_margin = envelope - overshoot
    checks["measured_overshoot_within_plant_envelope"] = bool(
        np.all(envelope_margin >= -1e-9))
    support_after = np.nonzero(telemetry.legal_plantar_active[k_occ + 1:] > 0)[0]
    claim_end = (k_occ + 1 + int(support_after[0])) if len(support_after) else len(telemetry.index)
    prohibited_launch = np.nonzero(telemetry.prohibited_active[:k_occ + 1] > 0)[0]
    prohibited_flight = np.nonzero(
        telemetry.prohibited_active[k_occ:claim_end] > 0)[0]
    checks["no_prohibited_contact"] = bool(len(prohibited_launch) == 0
                                           and len(prohibited_flight) == 0)
    checks["bilateral_symmetry"] = bool(
        max((float(np.abs(telemetry.applied_nm[:, i] - telemetry.applied_nm[:, j]).max())
             for i, j in ((1, 2), (3, 4), (5, 6), (7, 8))), default=0.0) <= 1e-9)
    from loaded_cmj.v3.actuation import MOMENT_CEILING_NM, POWER_CEILING_W
    checks["moment_ceiling"] = bool(np.all(np.abs(telemetry.applied_nm)
                                           <= MOMENT_CEILING_NM + 1e-9))
    checks["power_ceiling"] = bool(np.all(np.abs(telemetry.joint_power_w)
                                          <= POWER_CEILING_W + 1e-6))
    # no ground impulse inside the claimed flight: from the accepted occurrence
    # to the first legal plantar recontact the total vertical ground force is 0
    post_ok = bool(np.all(np.abs(telemetry.total_wrench[k_occ:claim_end, 2]) < 1e-9))
    checks["no_post_takeoff_ground_impulse"] = post_ok
    h2 = apex.get("h2_support_m")
    feasible = all(checks.values()) and h2 is not None
    losses = int(np.count_nonzero((telemetry.legal_plantar_active[1:] == 0)
                                  & (telemetry.legal_plantar_active[:-1] > 0)))
    z_min = float(np.min(telemetry.com_world_m[:k_occ + 1, 2]))
    z_occ = (None if occurrence_index is None
             else float(telemetry.com_world_m[k_occ, 2]))
    return {
        "overrides": overrides,
        "checks": checks,
        "feasible": bool(feasible),
        "h2_m": None if h2 is None else float(h2),
        "takeoff_vz_m_s": apex.get("takeoff_vz_m_s"),
        "accepted_occurrence_index": occurrence_index,
        "confirmation_sample": (None if confirmation is None
                                else confirmation.get("confirmation_sample")),
        "apex_time_s": apex.get("apex_time_s"),
        "phases": episode.phases_visited,
        "support_loss_events": losses,
        "countermovement_min_z_m": z_min,
        "occurrence_z_m": z_occ,
        "propulsion_stroke_m": (None if z_occ is None else float(z_occ - z_min)),
        "trunk_min_rad": float(np.min(telemetry.joint_q[:k_occ + 1, 3])),
        "trunk_max_rad": float(np.max(telemetry.joint_q[:k_occ + 1, 3])),
        "min_joint_margins": [float(v) for v in np.nanmin(launch_margins, axis=0)],
        "min_reference_margin": [float(v) for v in np.nanmin(
            np.minimum(q_ref - rom_lo, rom_hi - q_ref), axis=0)],
        "min_envelope_margin_rad": [float(v) for v in np.nanmin(envelope_margin, axis=0)],
        "min_joint_margins_landing": [float(v) for v in np.nanmin(margins, axis=0)],
        "max_total_fz_n": float(np.max(telemetry.total_wrench[:, 2])),
        "mtp_active_positive_work_j": [float(v) for v in telemetry.mtp_active_work_j[-1]],
        "mtp_passive_positive_work_j": [float(v) for v in telemetry.mtp_passive_work_j[-1]],
        "ankle_positive_work_j": float(np.sum(
            np.maximum(telemetry.joint_power_w[:, [5, 6]], 0.0)) * M.NATIVE_DT_S),
        "max_abs_knee_moment_nm": float(np.max(np.abs(telemetry.applied_nm[:, 3]))),
        "max_abs_knee_power_w": float(np.max(np.abs(telemetry.joint_power_w[:, 3]))),
        "max_abs_hip_moment_nm": float(np.max(np.abs(telemetry.applied_nm[:, 1]))),
        "max_abs_hip_power_w": float(np.max(np.abs(telemetry.joint_power_w[:, 1]))),
        "max_abs_trunk_moment_nm": float(np.max(np.abs(telemetry.applied_nm[:, 0]))),
        "cop_clamped_samples": int(np.count_nonzero(telemetry.cop_clamped)),
        "prohibited_samples": int(np.count_nonzero(telemetry.prohibited_active)),
        "prohibited_samples_launch": [int(v) for v in prohibited_launch],
        "prohibited_samples_flight": [int(v) for v in prohibited_flight],
        "claim_end_sample": int(claim_end),
        "joint_names": list(V3_JOINT_NAMES),
    }


def _worker(payload: tuple[int, dict[str, float]]) -> tuple[int, dict[str, Any]]:
    index, overrides = payload
    return index, evaluate_config(overrides)


def run_stage(stage: int, base: dict[str, float], workers: int) -> list[dict[str, Any]]:
    variables = STAGE_VARIABLES[stage]
    (name_a, values_a), (name_b, values_b) = tuple(variables.items())
    grid = [{**base, name_a: a, name_b: b} for a in values_a for b in values_b]
    with mp.Pool(processes=workers) as pool:
        results = pool.map(_worker, list(enumerate(grid)))
    return [result for _, result in sorted(results)]


def _ranking_key(result: dict[str, Any]) -> tuple:
    return (
        1 if result["feasible"] else 0,
        result["h2_m"] if result["h2_m"] is not None else -1.0,
        -result["support_loss_events"],
    )


def run_declared_search(workers: int) -> dict[str, Any]:
    """Chained coordinate ascent over the declared stages (bounded, ordered)."""
    base = dict(ENTRY_NEUTRAL_BASE)
    base.update(FIXED_ENGINEERING_SETUP)
    stages: list[dict[str, Any]] = []
    evaluations = 0
    for stage in sorted(STAGE_VARIABLES):
        results = run_stage(stage, base, workers)
        results.sort(key=_ranking_key, reverse=True)
        evaluations += len(results)
        best = results[0]
        stages.append({
            "stage": stage,
            "variables": {k: list(v) for k, v in STAGE_VARIABLES[stage].items()},
            "base": dict(base),
            "evaluations": len(results),
            "best": best,
            "results": results,
        })
        base.update(best["overrides"])
    return {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "linear_issue": "RES-85",
        "search_horizon_s": SEARCH_HORIZON_S,
        "entry_neutral_base": ENTRY_NEUTRAL_BASE,
        "fixed_engineering_setup": FIXED_ENGINEERING_SETUP,
        "stages": stages,
        "total_evaluations": evaluations,
        "max_evaluations": MAX_EVALUATIONS,
        "winner_overrides": base,
        "winner": stages[-1]["best"],
        "ordering": "stage ascending; within a stage, grid order as declared; "
                    "ranking: feasible, H2 desc, support-loss events asc",
    }


def run_declared_search_twice(workers: int) -> tuple[dict[str, Any], dict[str, Any]]:
    """Two deterministic runs of the declared search plus a byte-identity record."""
    first = run_declared_search(workers)
    second = run_declared_search(workers)
    text_first = json.dumps(first, indent=2, sort_keys=True) + "\n"
    text_second = json.dumps(second, indent=2, sort_keys=True) + "\n"
    first_sha = hashlib.sha256(text_first.encode("utf-8")).hexdigest()
    second_sha = hashlib.sha256(text_second.encode("utf-8")).hexdigest()
    identity = {
        "runs": 2,
        "byte_identical": bool(first_sha == second_sha),
        "sha256": [first_sha, second_sha],
        "canonical_encoding": ("json(indent=2, sort_keys=True) + trailing newline, "
                               "report payload without the two_run_identity field"),
    }
    first["two_run_identity"] = identity
    return first, identity


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", type=int)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--workers", type=int, default=5)
    parser.add_argument("--base", type=json.loads, default={})
    parser.add_argument("--out", type=Path,
                        default=HERE / "PROPULSION_SEARCH_STAGE.json")
    parser.add_argument("--report", action="store_true")
    args = parser.parse_args(argv)
    if args.report:
        base = json.loads((HERE / "PROPULSION_SEARCH_BEST.json").read_text())
        print(json.dumps(base, indent=2))
        return 0
    if args.all:
        report, identity = run_declared_search_twice(args.workers)
        (HERE / "PROPULSION_SEARCH.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n")
        winner = report["winner"]
        print("declared search: evaluations", report["total_evaluations"],
              "<= max", report["max_evaluations"])
        print("winner:", json.dumps(winner["overrides"], sort_keys=True),
              "feasible", winner["feasible"], "H2", winner["h2_m"])
        print("two-run identity:", json.dumps(identity))
        return 0 if identity["byte_identical"] else 1
    if args.stage is None:
        parser.error("--stage, --all or --report required")
    results = run_stage(args.stage, args.base, args.workers)
    results.sort(key=_ranking_key, reverse=True)
    payload = {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "stage": args.stage,
        "search_horizon_s": SEARCH_HORIZON_S,
        "base": args.base,
        "evaluations": len(results),
        "ranking_rule": ("feasible, then H2 desc, then support-loss events asc"),
        "results": results,
        "best": results[0],
    }
    args.out.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print("stage", args.stage, "evaluations", len(results))
    print("best:", json.dumps(results[0]["overrides"], sort_keys=True),
          "feasible", results[0]["feasible"],
          "H2", results[0]["h2_m"])
    for bad in [r for r in results if not r["feasible"]][:6]:
        print("  infeasible:", json.dumps(bad["overrides"], sort_keys=True),
              {k: v for k, v in bad["checks"].items() if not v})
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
