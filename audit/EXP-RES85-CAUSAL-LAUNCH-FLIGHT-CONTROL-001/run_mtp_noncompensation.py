"""RES-85C Blocker D — MTP non-compensation matrix (deterministic).

MISSION: RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001

Question: does the accepted launch depend on replacing removed passive MTP
mechanics with large active toe work?  Boundedness of the active budget alone
does not answer this; the zero-passive model must be exercised with the active
authority removed and with it tightly bounded.

Cases (declared, deterministic):
  M0  nominal passive MTP   + nominal bounded active MTP (sealed budget)
  M1  zero-passive MTP      + zero active MTP
  M2  zero-passive MTP      + tightly bounded active MTP (2.5 J per foot)
  M3  zero-passive MTP      + nominal active authority (sealed budget)

Active-work-cap sweep (zero-passive, declared values):
  0 J, the observed nominal active-work scale of M0, 2.5 J, 12.5 J, 25 J.

MTP moment-authority sweep (both passive models, declared sensitivity values
from ACTUATION_AUTHORITY.json): 0.0, 22.5, 45.0 (nominal), 60.0 N*m.

Every case reports phase feasibility, takeoff occurrence/confirmation, takeoff
vz, H2, ankle positive work, MTP active/passive positive work, the ratio of MTP
active positive work to total positive joint work over the RES-85 claim window,
whether the MTP budget binds, and whether qualitative success (confirmed
takeoff + H2 evaluable + H2 >= H_ANTI_TRIVIALITY_FLOOR) changes.

Run:
  python3 run_mtp_noncompensation.py --config '{"...": ...}' --workers 6
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
from loaded_cmj.v3.actuation import CHANNELS  # noqa: E402
from loaded_cmj.v3.controller import V3ControllerConfig  # noqa: E402
from loaded_cmj.v3.launch_runtime import run_launch_episode  # noqa: E402

HORIZON_S = 2.0
H_ANTI_TRIVIALITY_FLOOR_M = 0.150
TIGHT_ACTIVE_BUDGET_J = 2.5
MOMENT_SWEEP_NM = (0.0, 22.5, 45.0, 60.0)

MTP_IDX = [CHANNELS.index("left_mtp"), CHANNELS.index("right_mtp")]
ANKLE_IDX = [CHANNELS.index("left_ankle"), CHANNELS.index("right_ankle")]


def _case(*, zero_passive: bool, active_budget_j: float | None,
          moment_ceiling_nm: float | None, config: V3ControllerConfig) -> dict[str, Any]:
    cfg = replace(config, mtp_active_budget_j=active_budget_j,
                  mtp_moment_ceiling_nm=moment_ceiling_nm)
    episode = run_launch_episode(horizon_s=HORIZON_S, zero_passive=zero_passive,
                                 controller_config=cfg)
    telemetry = episode.telemetry
    events = episode.events
    occurrence = events.get("takeoff_occurrence") or {}
    confirmation = events.get("takeoff_confirmation") or {}
    apex = events.get("apex_h2") or {}
    k_occ = occurrence.get("native_index")
    end = int(k_occ) + 1 if k_occ is not None else len(telemetry.index)
    dt = M.NATIVE_DT_S
    joint_positive = np.maximum(telemetry.joint_power_w[:end], 0.0) * dt
    total_positive_work = float(joint_positive.sum())
    ankle_positive = float(joint_positive[:, ANKLE_IDX].sum())
    mtp_active = [float(v) for v in telemetry.mtp_active_work_j[end - 1]]
    mtp_passive_pos = [float(v) for v in telemetry.mtp_passive_work_j[end - 1]]
    mtp_active_total = float(sum(mtp_active))
    stage_counts: dict[str, int] = {}
    for row in telemetry.saturation_stage[:end]:
        for stage in row:
            name = str(stage)
            if name.startswith("mtp"):
                stage_counts[name] = stage_counts.get(name, 0) + 1
    budget_binds = bool(stage_counts.get("mtp_budget", 0) > 0
                        or stage_counts.get("mtp_phase_gate", 0) > 0)
    h2 = apex.get("h2_support_m")
    qualitative_success = bool(confirmation.get("confirmed")
                               and apex.get("evaluable")
                               and h2 is not None
                               and float(h2) >= H_ANTI_TRIVIALITY_FLOOR_M)
    mtp_moment_max = float(np.max(np.abs(telemetry.applied_nm[:end][:, MTP_IDX])))
    return {
        "zero_passive": bool(zero_passive),
        "active_budget_override_j": active_budget_j,
        "moment_ceiling_override_nm": moment_ceiling_nm,
        "episode_status": episode.status,
        "fault": episode.fault,
        "phases": episode.phases_visited,
        "takeoff_occurrence_index": k_occ,
        "takeoff_confirmation": bool(confirmation.get("confirmed")),
        "takeoff_vz_m_s": apex.get("takeoff_vz_m_s"),
        "h2_m": None if h2 is None else float(h2),
        "ankle_positive_work_j": ankle_positive,
        "mtp_active_positive_work_j": mtp_active,
        "mtp_passive_positive_work_j": mtp_passive_pos,
        "mtp_active_total_positive_work_j": mtp_active_total,
        "total_positive_joint_work_j": total_positive_work,
        "ratio_mtp_active_over_total_positive": (
            None if total_positive_work <= 0.0 else mtp_active_total / total_positive_work),
        "mtp_budget_binds": budget_binds,
        "mtp_stage_counts": stage_counts,
        "mtp_moment_max_abs_nm": mtp_moment_max,
        "qualitative_success": qualitative_success,
    }


def _worker(payload):
    name, kwargs, config = payload
    return name, _case(config=config, **kwargs)


def build_matrix(config: V3ControllerConfig, workers: int) -> dict[str, Any]:
    """Run the declared deterministic case matrix once."""
    # observed nominal active-work scale from CASE M0 is resolved in two passes:
    # first run M0, then declare it as a sweep point.
    m0 = _case(zero_passive=False, active_budget_j=None,
               moment_ceiling_nm=None, config=config)
    nominal_scale = float(max(m0["mtp_active_positive_work_j"]))
    budget_sweep = (0.0, nominal_scale, TIGHT_ACTIVE_BUDGET_J, 12.5, 25.0)
    cases: list[tuple[str, dict[str, Any]]] = [
        ("M0_nominal_passive_nominal_active",
         dict(zero_passive=False, active_budget_j=None, moment_ceiling_nm=None)),
        ("M1_zero_passive_zero_active",
         dict(zero_passive=True, active_budget_j=0.0, moment_ceiling_nm=None)),
        ("M2_zero_passive_tight_active",
         dict(zero_passive=True, active_budget_j=TIGHT_ACTIVE_BUDGET_J,
              moment_ceiling_nm=None)),
        ("M3_zero_passive_nominal_active",
         dict(zero_passive=True, active_budget_j=None, moment_ceiling_nm=None)),
    ]
    for budget in budget_sweep:
        cases.append((f"SWEEP_active_budget_{budget:g}J",
                      dict(zero_passive=True, active_budget_j=budget,
                           moment_ceiling_nm=None)))
    for ceiling in MOMENT_SWEEP_NM:
        cases.append((f"SWEEP_moment_normal_passive_{ceiling:g}Nm",
                      dict(zero_passive=False, active_budget_j=None,
                           moment_ceiling_nm=ceiling)))
        cases.append((f"SWEEP_moment_zero_passive_{ceiling:g}Nm",
                      dict(zero_passive=True, active_budget_j=None,
                           moment_ceiling_nm=ceiling)))

    payloads = [(name, kwargs, config) for name, kwargs in cases]
    with mp.Pool(processes=workers) as pool:
        results = dict(pool.map(_worker, payloads))
    results["M0_nominal_passive_nominal_active"] = m0
    matrix = {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "linear_issue": "RES-85",
        "controller_config": {
            field: getattr(config, field) for field in config.__dataclass_fields__
        },
        "horizon_s": HORIZON_S,
        "h_anti_triviality_floor_m": H_ANTI_TRIVIALITY_FLOOR_M,
        "observed_nominal_active_work_scale_j": nominal_scale,
        "declared_cases": [name for name, _ in cases],
        "case_results": results,
    }
    # required conclusion, computed from the matrix
    m1 = results["M1_zero_passive_zero_active"]
    m0_conclusion = {
        "zero_passive_zero_active_feasible":
            bool(m1["takeoff_confirmation"] and m1["h2_m"] is not None),
        "zero_passive_zero_active_h2_m": m1["h2_m"],
        "nominal_h2_m": m0["h2_m"],
        "m0_h2_retained_with_zero_passive":
            bool(m1["h2_m"] is not None and m0["h2_m"] is not None
                 and m1["h2_m"] >= 0.5 * float(m0["h2_m"])),
        "mtp_active_ratio_nominal": m0["ratio_mtp_active_over_total_positive"],
        "conclusion": (
            "the accepted launch does not depend on replacing removed passive "
            "MTP mechanics with large active toe work"
            if (m1["mtp_active_total_positive_work_j"] == 0.0
                and m1["takeoff_confirmation"])
            else "zero-passive sensitivity materially changes the launch: reported honestly"),
    }
    matrix["required_conclusion"] = m0_conclusion
    return matrix


def _canonical_payload(matrix: dict[str, Any]) -> str:
    payload = {k: v for k, v in matrix.items() if k != "two_run_identity"}
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=json.loads, default={})
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--out", type=Path,
                        default=HERE / "MTP_NONCOMPENSATION_MATRIX.json")
    args = parser.parse_args(argv)
    config = replace(V3ControllerConfig(), **args.config)

    first = build_matrix(config, args.workers)
    second = build_matrix(config, args.workers)
    first_sha = hashlib.sha256(_canonical_payload(first).encode("utf-8")).hexdigest()
    second_sha = hashlib.sha256(_canonical_payload(second).encode("utf-8")).hexdigest()
    identity = {
        "runs": 2,
        "byte_identical": bool(first_sha == second_sha),
        "sha256": [first_sha, second_sha],
        "canonical_encoding": ("json(indent=2, sort_keys=True) + trailing newline, "
                               "report payload without the two_run_identity field"),
    }
    first["two_run_identity"] = identity
    args.out.write_text(json.dumps(first, indent=2, sort_keys=True) + "\n")
    for name, value in first["case_results"].items():
        print(f"{name:42s} vz={value['takeoff_vz_m_s']} H2={value['h2_m']} "
              f"mtp_act={value['mtp_active_total_positive_work_j']:.5f} "
              f"mtp_pass={sum(value['mtp_passive_positive_work_j']):.5f} "
              f"binds={value['mtp_budget_binds']} success={value['qualitative_success']}")
    print("conclusion:", json.dumps(first["required_conclusion"], indent=1))
    print("two-run identity:", json.dumps(identity))
    return 0 if identity["byte_identical"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
