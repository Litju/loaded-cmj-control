#!/usr/bin/env python3
"""RES-86 nominal contact characterization + declared contact/timestep sensitivity.

MISSION: RES86_TOE_PHASE_FEASIBILITY_AND_RESOLUTION_001
LINEAR ISSUE: RES-86

Phase 3 (nominal characterization)
----------------------------------
Report the exact-branch ``Fz`` / penetration / relative-normal-velocity record
of the initial legal toe phase for the nominal model, plus the best nominal
unrestricted feasibility branch.  No linear-spring inference is drawn from a
single trajectory; every number is an exact MuJoCo branch sample.

Phase 4 (declared sensitivity only)
-----------------------------------
The RES-84 authority predeclares the sensitivity domain (``mu_slide``
[0.5, 0.9, 1.5], plantar ``condim`` [3, 4, 6], ``dt`` [0.001, 0.002, 0.004]).
For solref/solimp the smallest defensible deterministic stiffening around the
provisional nominal baseline is audited (time constant 0.02 -> 0.01 s, and the
impedance scaling raised toward 1.0), because the nominal values are classified
PROVISIONAL_NUMERICAL and RES-86 owns solution verification (CC-10/CC-11).
No parameter is tuned to manufacture a PASS; the audit only measures whether a
declared numerics change materially expands the safe Fz/impulse frontier below
the 10 mm penetration ceiling.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3.plant import V3Plant, model_xml  # noqa: E402
from tools.res86.landing_feasibility_oracle import (  # noqa: E402
    BW_N,
    DEFAULT_KNOTS,
    BranchStabilizer,
    LandingFeasibilityOracle,
    NATIVE_DT_S,
    capture_canonical_branch,
)

V3_CONTACT_SENSITIVITY_AUTHORITY_ID = "LCMJ_RES86_CONTACT_SENSITIVITY_AUDIT_V1"


@dataclass(frozen=True)
class ContactRealization:
    label: str
    dt_s: float
    mu_slide: float
    plantar_condim: int
    solref: tuple[float, float]
    solimp: tuple[float, float, float, float, float]
    declaration: str


NOMINAL = ContactRealization(
    label="nominal",
    dt_s=0.002,
    mu_slide=0.9,
    plantar_condim=4,
    solref=(0.02, 1.0),
    solimp=(0.9, 0.95, 0.001, 0.5, 2.0),
    declaration="RES84 provisional baseline (MuJoCo 3.8.0 compiled defaults)",
)

SENSITIVITY_LATTICE: tuple[ContactRealization, ...] = (
    NOMINAL,
    ContactRealization("mu_0.5", 0.002, 0.5, 4, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 mu_slide sensitivity domain"),
    ContactRealization("mu_1.5", 0.002, 1.5, 4, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 mu_slide sensitivity domain"),
    ContactRealization("condim_3", 0.002, 0.9, 3, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 plantar condim sensitivity domain"),
    ContactRealization("condim_6", 0.002, 0.9, 6, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 plantar condim sensitivity domain"),
    ContactRealization("dt_0.001", 0.001, 0.9, 4, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 timestep sensitivity domain"),
    ContactRealization("dt_0.004", 0.004, 0.9, 4, (0.02, 1.0), (0.9, 0.95, 0.001, 0.5, 2.0),
                       "RES84 timestep sensitivity domain"),
    ContactRealization("solref_tau_0.01", 0.002, 0.9, 4, (0.01, 1.0),
                       (0.9, 0.95, 0.001, 0.5, 2.0),
                       "smallest defensible stiffening of the provisional solref"),
    ContactRealization("solimp_stiff", 0.002, 0.9, 4, (0.02, 1.0),
                       (0.95, 0.99, 0.001, 0.5, 2.0),
                       "impedance scaling raised toward 1.0 (still within [0,1])"),
    ContactRealization("solref_tau_0.01_solimp_stiff", 0.002, 0.9, 4, (0.01, 1.0),
                       (0.95, 0.99, 0.001, 0.5, 2.0),
                       "combined declared stiffening (causal discrimination only)"),
)


def build_realization(realization: ContactRealization) -> V3Plant:
    """Compile an in-memory Plant realization for a declared sensitivity point.

    The sealed Plant XML is never mutated on disk; this is an audit-only model
    copy used to measure causal sensitivity.
    """
    spec = mujoco.MjSpec.from_string(model_xml())
    spec.option.timestep = float(realization.dt_s)
    support_names = set(C.V3_PLANTAR_SUPPORT_GEOMS)
    for geom in spec.geoms:
        if geom.name in support_names:
            geom.condim = int(realization.plantar_condim)
            geom.solref = np.asarray(realization.solref, dtype=np.float64)
            geom.solimp = np.asarray(realization.solimp, dtype=np.float64)
        geom.friction = np.asarray([realization.mu_slide, 0.0, 0.0], dtype=np.float64)
    return V3Plant(model=spec.compile())


def nominal_toe_phase_characterization(branch, actions: np.ndarray, native_steps: int) -> dict:
    """Exact Fz / penetration / normal-velocity record of the initial toe phase."""
    oracle = LandingFeasibilityOracle(branch)
    result = oracle.evaluate_sequence(actions, native_steps, collect_samples=True)
    samples = []
    previous_penetration = 0.0
    toe_phase_end = None
    for sample in result.samples:
        regions = {region for _side, region in sample.active_regions}
        penetration = sample.max_penetration_m
        relative_normal_velocity = (penetration - previous_penetration) / oracle.dt
        previous_penetration = penetration
        if regions and regions != {"toe"} and toe_phase_end is None:
            toe_phase_end = sample.offset
        samples.append({
            "offset": sample.offset,
            "time_s": sample.time_s,
            "regions": sorted(regions),
            "total_fz_n": sample.fz_total_n,
            "total_fz_bw": sample.fz_total_n / BW_N,
            "max_penetration_m": penetration,
            "relative_normal_velocity_m_s": relative_normal_velocity,
            "com_vz_m_s": sample.com_vz_m_s,
        })
    toe_samples = [s for s in samples if s["regions"] == ["toe"]]
    within_limit = [s for s in samples if s["max_penetration_m"] <= 0.010]
    return {
        "actions_source": "sealed canonical launch continuation negative control",
        "toe_phase_end_offset": toe_phase_end,
        "toe_phase_max_penetration_m": max((s["max_penetration_m"] for s in toe_samples),
                                           default=0.0),
        "toe_phase_peak_total_fz_bw": max((s["total_fz_bw"] for s in toe_samples),
                                          default=0.0),
        "max_fz_bw_within_10mm_penetration": max(
            (s["total_fz_bw"] for s in within_limit), default=0.0),
        "samples": samples,
        "interpretation": (
            "EXACT_BRANCH_RECORD_ONLY; the Fz achievable at a given penetration "
            "depends on the joint torque state and the relative normal velocity, "
            "so no linear-spring stiffness is inferred from one trajectory"
        ),
    }


def evaluate_realization(branch, realization: ContactRealization, actions: np.ndarray | None,
                         knots: np.ndarray | None, *, native_steps: int,
                         search_budget: int, search_knots: int) -> dict:
    t0 = time.time()
    plant = build_realization(realization)
    oracle = LandingFeasibilityOracle(branch, plant=plant, native_dt_s=realization.dt_s)
    stabilizer = BranchStabilizer(enabled=True)
    record: dict = {
        "label": realization.label,
        "declared": {
            "dt_s": realization.dt_s,
            "mu_slide": realization.mu_slide,
            "plantar_condim": realization.plantar_condim,
            "solref": list(realization.solref),
            "solimp": list(realization.solimp),
            "declaration": realization.declaration,
        },
        "native_steps": native_steps,
        "horizon_s": native_steps * realization.dt_s,
    }
    if actions is not None:
        sequence = np.asarray(actions, dtype=np.float64)
        source_steps, variant_steps = sequence.shape[0], native_steps
        if source_steps != variant_steps:
            source_times = (np.arange(source_steps) + 0.5) / source_steps
            target_times = (np.arange(variant_steps) + 0.5) / variant_steps
            sequence = np.stack([
                np.interp(target_times, source_times, sequence[:, channel])
                for channel in range(sequence.shape[1])], axis=1)
        result = oracle.evaluate_sequence(sequence, native_steps, collect_samples=False)
        record["fixed_action_replay"] = {
            "terminal_com_vz_m_s": result.terminal_com_vz_m_s,
            "max_penetration_m": result.max_penetration_m,
            "net_vertical_impulse_ns": result.net_vertical_impulse_ns,
            "peak_total_fz_bw": result.peak_total_fz_n / BW_N,
            "hard_gate_failures": result.hard_gate_failures,
            "admissible": result.admissible,
        }
    if search_budget > 0:
        starts = [np.zeros((search_knots, 4), dtype=np.float64)]
        if knots is not None:
            source_steps = 150  # nominal reference horizon
            profile = LandingFeasibilityOracle(branch).profile_from_knots(
                knots, source_steps, knots.shape[0])
            times = np.linspace(0.0, native_steps * realization.dt_s, search_knots)
            source_times = (np.arange(source_steps) + 0.5) * NATIVE_DT_S
            seed = np.zeros((search_knots, 4), dtype=np.float64)
            coordinate_channel = (0, 1, 3, 5)  # trunk, hip pair, knee pair, ankle pair
            for index, channel in enumerate(coordinate_channel):
                seed[:, index] = np.interp(times, source_times, profile[:, channel])
            starts.append(seed)
        search = oracle.search_coordinate_descent(
            native_steps, search_knots, search_budget, initial_knots=starts,
            stabilizer=stabilizer, max_sweeps=5, verbose=False)
        best = search["best"]
        result = best["result"]
        record["search"] = {
            "evaluations": search["evaluations"],
            "budget": search_budget,
            "cost": best["cost"],
            "admissible": result.admissible,
            "hard_gate_failures": result.hard_gate_failures,
            "terminal_com_vz_m_s": result.terminal_com_vz_m_s,
            "net_vertical_impulse_ns": result.net_vertical_impulse_ns,
            "grf_vertical_impulse_ns": result.grf_vertical_impulse_ns,
            "peak_total_fz_bw": result.peak_total_fz_n / BW_N,
            "max_penetration_m": result.max_penetration_m,
            "min_rom_margin_rad": result.min_rom_margin_rad,
            "max_moment_ratio": result.max_moment_ratio,
            "max_power_ratio": result.max_power_ratio,
            "mtp_authority_ok": result.mtp_authority_ok,
            "prohibited_any": result.prohibited_any,
            "material_reflight": result.material_reflight,
            "terminal_legal_support": result.terminal_legal_support,
            "toe_mode_escaped": result.toe_mode_escaped,
            "mode_sequence": [[list(entry) for entry in mode]
                              for mode in result.mode_sequence],
        }
    record["wall_s"] = time.time() - t0
    return record


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--budget", type=int, default=1200)
    parser.add_argument("--search-knots", type=int, default=11)
    parser.add_argument("--nominal-actions", type=Path, default=None,
                        help="npz with the nominal best recorded action sequence")
    parser.add_argument("--nominal-knots", type=Path, default=None,
                        help="npz with the nominal best profile knots")
    parser.add_argument("--skip-characterization", action="store_true")
    parser.add_argument("--only", type=str, default=None,
                        help="comma-separated realization labels to run")
    args = parser.parse_args()

    report: dict = {
        "authority_id": V3_CONTACT_SENSITIVITY_AUTHORITY_ID,
        "mission": "RES86_TOE_PHASE_FEASIBILITY_AND_RESOLUTION_001",
        "nominal": {
            "dt_s": NOMINAL.dt_s,
            "solref": list(NOMINAL.solref),
            "solimp": list(NOMINAL.solimp),
            "plantar_condim": NOMINAL.plantar_condim,
            "mu_slide": NOMINAL.mu_slide,
        },
        "declared_lattice": [realization.label for realization in SENSITIVITY_LATTICE],
    }
    only = set(args.only.split(",")) if args.only else None
    prior_rows: dict[str, dict] = {}
    if args.out is not None and args.out.exists():
        try:
            prior = json.loads(args.out.read_text())
            for row in prior.get("phase4_sensitivity", []):
                prior_rows[row["label"]] = row
            if "phase3_nominal_toe_phase" in prior:
                report["phase3_nominal_toe_phase"] = prior["phase3_nominal_toe_phase"]
        except (json.JSONDecodeError, KeyError):
            prior_rows = {}
    actions = None
    knots = None
    if args.nominal_actions is not None and args.nominal_actions.exists():
        with np.load(args.nominal_actions) as data:
            actions = np.asarray(data["actions"], dtype=np.float64)
    if args.nominal_knots is not None and args.nominal_knots.exists():
        with np.load(args.nominal_knots) as data:
            knots = np.asarray(data["knots"], dtype=np.float64)

    report["phase4_sensitivity"] = []
    print("capturing sealed canonical branch certificates ...", flush=True)
    branch = capture_canonical_branch()
    if not args.skip_characterization and actions is not None \
            and "phase3_nominal_toe_phase" not in report:
        print("phase-3 nominal toe-phase characterization ...", flush=True)
        report["phase3_nominal_toe_phase"] = nominal_toe_phase_characterization(
            branch, actions, actions.shape[0])
    for realization in SENSITIVITY_LATTICE:
        if only is not None and realization.label not in only:
            if realization.label in prior_rows:
                report["phase4_sensitivity"].append(prior_rows[realization.label])
            continue
        print(f"phase-4 realization {realization.label} ...", flush=True)
        native_steps = int(round(0.300 / realization.dt_s))
        record = evaluate_realization(
            branch, realization, actions, knots, native_steps=native_steps,
            search_budget=args.budget if realization.label != "nominal" else max(args.budget, 1500),
            search_knots=args.search_knots)
        report["phase4_sensitivity"].append(record)
        search = record.get("search", {})
        print(json.dumps({
            "label": realization.label,
            "admissible": search.get("admissible"),
            "terminal_vz": search.get("terminal_com_vz_m_s"),
            "penetration_m": search.get("max_penetration_m"),
            "peak_fz_bw": search.get("peak_total_fz_bw"),
            "failures": search.get("hard_gate_failures"),
            "wall_s": round(record["wall_s"], 1),
        }), flush=True)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
