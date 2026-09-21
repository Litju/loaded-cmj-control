#!/usr/bin/env python3
"""RES-86 contact-candidate qualification (solution verification, CC-10/CC-11).

MISSION: RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR ISSUE: RES-86

The RES-84 contact-parameter authority sealed the geom-level nominal
``solref`` (0.02, 1.0) as ``PROVISIONAL_NUMERICAL`` and handed the final
solution verification to RES-86.  This tool qualifies one explicit candidate:
the smallest stiffening declared by the RES-86 audit lattice, the plantar
support-geom ``solref`` time constant 0.02 -> 0.01 s.  The label is never the
claim: the engine mixes the support geom's ``solref`` with the floor's
compiled default through ``solmix`` (both 1.0, equal priority), so the runtime
``mjContact`` row the solver consumes carries 0.015 s, and every number below
is stated against the realized row.

Qualification battery
---------------------

Phase 1 - runtime realization identity at dt = 0.001 / 0.002 / 0.004:
``solref``, ``solimp``, ``friction`` (5 components), ``dim`` and
``includemargin`` of the engine-mixed contact rows, the frozen solver
semantics (Euler / Newton / pyramidal / 100 iterations, ``refsafe`` enabled, no
disabled contact path) and the realized-time-constant resolution.

Phase 2 - fixed-action contact response, like-for-like: the prior nominal
bounded-search witness is replayed from the same branch state under both
realizations at every dt over a short contact-response window (40 ms) where the
solution is still governed by the contact realization.  This is the declared
candidate-vs-nominal comparison: penetration, vertical impulse, peak Fz,
support/contact sequence, structural ROM, moment/power/MTP authority, chatter,
material reflight and prohibited contact.  The 0.002 s and 0.001 s steps are
the in-domain cells; the 0.004 s step moves 8 mm per sample at the sealed
~2 m/s descent and is recorded as an out-of-domain stress boundary, never as a
qualification pass.

Phase 3 - full-horizon witness battery: both prior bounded-search witnesses
replayed under both realizations at every dt, with deterministic repeated
replay (bit-identical terminal digests) and bit-exact reproduction of the
recorded witness outcomes at their own production cell.

Boundaries (never promoted)
---------------------------

* the candidate witness is a vertical-arrest / hard-safety witness only; its
  terminal CAM and posture gates fail, so it is not an E10 witness;
* the nominal 10.2115 mm bounded-search result is a bounded-search result,
  never a global infeasibility proof;
* contact compliance is proven *materially causal*, not monotone: the same
  fixed actions produce materially different trajectories under the two
  realizations.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.contact_realization import (  # noqa: E402
    DECLARED_SOLIMP,
    RES86_CANDIDATE_DECLARED_SOLREF,
    RES86_CANDIDATE_REALIZED_SOLREF,
    RES86_CONTACT_CANDIDATE_LABEL,
    RES86_NOMINAL_DECLARED_SOLREF,
    RES86_NOMINAL_REALIZED_SOLREF,
    build_contact_realization_plant,
    effective_floor_contacts,
    engine_solver_semantics,
    mixed_solref,
    timeconst_resolution,
    verify_effective_contact_realization,
)
from loaded_cmj.v3.landing_authority import (  # noqa: E402
    V3_STRUCTURAL_ROM_TOLERANCE_RAD,
)
from loaded_cmj.v3.landing_runtime import settle_standing_stance  # noqa: E402
from tools.res86.landing_feasibility_oracle import (  # noqa: E402
    BW_N,
    LandingFeasibilityOracle,
    capture_canonical_branch,
)

V3_CONTACT_CANDIDATE_QUALIFICATION_AUTHORITY_ID = (
    "LCMJ_RES86_CONTACT_CANDIDATE_QUALIFICATION_V1")

DT_GRID_S = (0.001, 0.002, 0.004)
IN_DOMAIN_DT_S = (0.001, 0.002)
STRESS_DT_S = (0.004,)
PRODUCTION_DT_S = 0.002
SHORT_HORIZON_S = 0.040
SHORT_HORIZON_REFERENCE_WITNESS = "nominal_witness"
HORIZON_S = 0.4
REFERENCE_WITNESSES = (
    ROOT / "audit" / "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001"
    / "contact_candidate" / "REFERENCE_WITNESSES.json")
REPRODUCTION_RTOL = 1.0e-12
REPRODUCTION_ATOL = 1.0e-15
PEAK_FZ_LIMIT_BW = 8.0
PENETRATION_LIMIT_M = 0.010
CHATTER_LIMIT = 8


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _resample(sequence: np.ndarray, source_dt: float, target_dt: float) -> np.ndarray:
    """Linear resample of a per-native-step action sequence in physical time."""
    source_steps, channels = sequence.shape
    target_steps = int(round(source_steps * source_dt / target_dt))
    source_times = (np.arange(source_steps) + 0.5) * source_dt
    target_times = (np.arange(target_steps) + 0.5) * target_dt
    return np.stack([
        np.interp(target_times, source_times, sequence[:, channel])
        for channel in range(channels)], axis=1)


def _run_lengths(sequence: list) -> list[dict]:
    runs: list[dict] = []
    for value in sequence:
        if runs and runs[-1]["value"] == value:
            runs[-1]["samples"] += 1
        else:
            runs.append({"value": value, "samples": 1})
    return runs


def _region_sequence(samples) -> list[list]:
    return [sorted([f"{side}:{region}" for side, region in sample.active_regions])
            for sample in samples]


def _contact_metrics(result, *, native_dt_s: float) -> dict:
    regions = _region_sequence(result.samples)
    support = [bool(sample.active_regions) for sample in result.samples]
    return {
        "native_steps": int(result.native_steps),
        "horizon_s": float(result.native_steps * native_dt_s),
        "max_penetration_m": float(result.max_penetration_m),
        "net_vertical_impulse_ns": float(result.net_vertical_impulse_ns),
        "grf_vertical_impulse_ns": float(result.grf_vertical_impulse_ns),
        "peak_total_fz_n": float(result.peak_total_fz_n),
        "peak_total_fz_bw": float(result.peak_total_fz_n / BW_N),
        "min_rom_margin_rad": float(result.min_rom_margin_rad),
        "max_moment_ratio": float(result.max_moment_ratio),
        "max_power_ratio": float(result.max_power_ratio),
        "mtp_authority_ok": bool(result.mtp_authority_ok),
        "prohibited_any": bool(result.prohibited_any),
        "prohibited_active_any": bool(result.prohibited_active_any),
        "material_reflight": bool(result.material_reflight),
        "chatter_transitions": int(result.chatter_transitions),
        "max_support_free_run": int(result.max_support_free_run),
        "terminal_legal_support": bool(result.terminal_legal_support),
        "toe_mode_escaped": bool(result.toe_mode_escaped),
        "terminal_com_vx_m_s": float(result.terminal_com_vx_m_s),
        "terminal_com_vz_m_s": float(result.terminal_com_vz_m_s),
        "terminal_hy_kg_m2_s": float(result.terminal_hy_kg_m2_s),
        "terminal_root_pitch_rad": float(result.terminal_root_pitch_rad),
        "terminal_trunk_pitch_rad": float(result.terminal_trunk_pitch_rad),
        "terminal_root_pitch_rate_rad_s": float(result.terminal_root_pitch_rate_rad_s),
        "terminal_trunk_pitch_rate_rad_s": float(result.terminal_trunk_pitch_rate_rad_s),
        "hard_gate_failures": list(result.hard_gate_failures),
        "admissible": bool(result.admissible),
        "finite": bool(result.finite),
        "support_fraction": float(np.mean(support)) if support else 0.0,
        "support_transitions": int(sum(
            1 for a, b in zip(support, support[1:]) if a != b)),
        "region_run_lengths": _run_lengths(regions),
        "sequence_sha256": hashlib.sha256(
            "|".join(str(sample.active_regions) for sample in result.samples)
            .encode("utf-8")).hexdigest(),
        "terminal_state_sha256": result.terminal_state_sha256,
    }


def _realization_audit(label: str, declared_solref, realized_solref, dt: float,
                       *, expect_candidate: bool) -> dict:
    plant = build_contact_realization_plant(declared_solref, dt_s=dt)
    model = plant.model
    expected = (RES86_CANDIDATE_REALIZED_SOLREF if expect_candidate
                else RES86_NOMINAL_REALIZED_SOLREF)
    if tuple(realized_solref) != tuple(expected):
        raise RuntimeError(
            f"declared realization expectation {expected} != asserted {realized_solref}")
    data = plant.make_data()
    settle_standing_stance(plant, data)
    report = verify_effective_contact_realization(
        plant, data, expected_solref=realized_solref, expected_solimp=DECLARED_SOLIMP,
        expected_dim=4, expected_sliding_friction=0.9,
        expected_includemargin_m=0.0)
    return {
        "label": label,
        "declared_geom_solref": [float(v) for v in declared_solref],
        "expected_mixed_solref": [float(v) for v in mixed_solref(declared_solref)],
        "expected_realized_solref": [float(v) for v in expected],
        "realized_status": report["status"],
        "realized_failures": report["failures"],
        "plantar_rows": report["plantar_rows"],
        "floor_row_count": report["floor_row_count"],
        "solver_semantics": engine_solver_semantics(model),
        "timeconst_resolution": timeconst_resolution(declared_solref, dt),
        "fingerprint": hashlib.sha256(
            "|".join(
                f"{row['geom']}:{row['dim']}:{row['solref']}:{row['solimp']}:"
                f"{row['friction']}:{row['includemargin_m']}"
                for row in report["plantar_rows"]).encode("utf-8")).hexdigest(),
    }


def _replay(oracle, sequence: np.ndarray, *, dt: float) -> dict:
    first = oracle.evaluate_sequence(sequence, sequence.shape[0], collect_samples=True)
    second = oracle.evaluate_sequence(sequence, sequence.shape[0], collect_samples=False)
    metrics = _contact_metrics(first, native_dt_s=dt)
    metrics["deterministic_replay"] = bool(
        first.terminal_state_sha256 == second.terminal_state_sha256
        and np.array_equal(first.terminal_state_vector, second.terminal_state_vector))
    return metrics


def _reproduce(recorded: dict, metrics: dict) -> dict:
    checks: dict[str, bool] = {}
    for key in ("max_penetration_m", "terminal_com_vz_m_s", "terminal_com_vx_m_s",
                "peak_total_fz_bw", "min_rom_margin_rad", "max_moment_ratio",
                "max_power_ratio", "net_vertical_impulse_ns", "grf_vertical_impulse_ns",
                "terminal_hy_kg_m2_s", "terminal_root_pitch_rad",
                "terminal_trunk_pitch_rad", "terminal_root_pitch_rate_rad_s",
                "terminal_trunk_pitch_rate_rad_s"):
        if key not in recorded:
            continue
        checks[key] = bool(np.isclose(float(metrics[key]), float(recorded[key]),
                                      rtol=REPRODUCTION_RTOL, atol=REPRODUCTION_ATOL))
    for key in ("admissible", "prohibited_any", "material_reflight"):
        if key in recorded:
            checks[key] = bool(metrics[key]) == bool(recorded[key])
    for key in ("chatter_transitions", "max_support_free_run"):
        if key in recorded:
            checks[key] = int(metrics[key]) == int(recorded[key])
    checks["hard_gate_failures"] = (
        list(metrics["hard_gate_failures"]) == list(recorded["hard_gate_failures"])
        if "hard_gate_failures" in recorded else True)
    return checks


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--witnesses", type=Path, default=REFERENCE_WITNESSES)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--skip-battery", action="store_true")
    args = parser.parse_args()

    witness_document = json.loads(args.witnesses.read_text())
    witnesses = witness_document["witnesses"]
    report: dict = {
        "authority_id": V3_CONTACT_CANDIDATE_QUALIFICATION_AUTHORITY_ID,
        "mission": "RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001",
        "linear_issue": "RES-86",
        "candidate": {
            "label": RES86_CONTACT_CANDIDATE_LABEL,
            "declared_geom_solref": list(RES86_CANDIDATE_DECLARED_SOLREF),
            "expected_realized_solref": list(RES86_CANDIDATE_REALIZED_SOLREF),
            "nominal_declared_geom_solref": list(RES86_NOMINAL_DECLARED_SOLREF),
            "nominal_realized_solref": list(RES86_NOMINAL_REALIZED_SOLREF),
            "selection_rule": (
                "smallest declared stiffening of the RES-86 predeclared audit lattice "
                "(plantar support-geom solref time constant 0.02 -> 0.01 s); the "
                "impedance-scaling and combined lattice entries are additional degrees and "
                "are therefore not smaller; condim, solimp and friction are unchanged"),
            "declared_realization_after_mixing": (
                "the geom-level 0.01 s time constant is realized at 0.015 s because MuJoCo "
                "mixes the plantar geom solref with the floor's compiled default (0.02, 1.0) "
                "through equal solmix weights and equal priority; every claim is stated "
                "against the realized row"),
        },
        "reference_witnesses": {
            "path": str(args.witnesses),
            "sha256": _sha256(args.witnesses),
            "sources": witness_document.get("sources", {}),
            "grid": witness_document.get("grid", {}),
        },
        "dt_grid_s": list(DT_GRID_S),
        "in_domain_dt_s": list(IN_DOMAIN_DT_S),
        "stress_dt_s": list(STRESS_DT_S),
        "short_horizon_s": SHORT_HORIZON_S,
        "short_horizon_reference_witness": SHORT_HORIZON_REFERENCE_WITNESS,
        "boundary": {
            "not_e10": (
                "the candidate witness is a vertical-arrest / hard-safety witness only; its "
                "terminal CAM and posture gates fail, so it is not an E10 witness"),
            "nominal_10mm_is_not_infeasibility": (
                "the nominal 10.2115 mm from the bounded search is a bounded-search result, "
                "never a global infeasibility proof"),
            "contact_compliance_causal": (
                "the realized solref change is proven materially causal by the fixed-action "
                "cross-realization replays"),
            "stress_dt": (
                "dt = 0.004 s moves 8 mm per sample at the sealed ~2 m/s descent, which is "
                "comparable to the 10 mm penetration budget; the 0.004 s step is recorded as "
                "an out-of-domain stress boundary and is never used as a qualification pass"),
        },
        "realization_audit": {},
        "short_horizon": {},
        "battery": {},
        "criteria": {},
        "status": "PENDING",
    }

    print("phase 1: runtime mjContact realization identity ...", flush=True)
    for dt in DT_GRID_S:
        key = f"dt_{dt:.3f}"
        nominal = _realization_audit(
            "nominal", RES86_NOMINAL_DECLARED_SOLREF, RES86_NOMINAL_REALIZED_SOLREF, dt,
            expect_candidate=False)
        candidate = _realization_audit(
            "candidate", RES86_CANDIDATE_DECLARED_SOLREF, RES86_CANDIDATE_REALIZED_SOLREF,
            dt, expect_candidate=True)
        report["realization_audit"][key] = {
            "dt_s": dt,
            "nominal": nominal,
            "candidate": candidate,
            "nominal_timestep_ok": nominal["solver_semantics"]["timestep_s"] == dt,
            "candidate_timestep_ok": candidate["solver_semantics"]["timestep_s"] == dt,
        }
        print(json.dumps({
            "dt": dt,
            "nominal_row_solref": nominal["plantar_rows"][0]["solref"] if nominal["plantar_rows"] else None,
            "candidate_row_solref": candidate["plantar_rows"][0]["solref"] if candidate["plantar_rows"] else None,
            "candidate_status": candidate["realized_status"],
            "refsafe": candidate["solver_semantics"]["refsafe_enabled"],
            "timeconst_over_dt": candidate["timeconst_resolution"]["timeconst_over_dt"],
        }), flush=True)

    if not args.skip_battery:
        print("phase 2: fixed-action short-horizon contact response ...", flush=True)
        branch = capture_canonical_branch()
        probe = LandingFeasibilityOracle(branch, native_dt_s=PRODUCTION_DT_S)
        approach = probe.evaluate_sequence(np.zeros((1, 9)), 1, collect_samples=True)
        report["approach_state"] = {
            "branch_pre_touchdown_sha256": branch.pre_touchdown.state_sha256,
            "branch_e8_sha256": branch.e8.state_sha256,
            "one_step_com_vz_m_s": float(approach.samples[0].com_vz_m_s),
            "one_step_com_vx_m_s": float(approach.samples[0].com_vx_m_s),
            "one_step_penetration_m": float(approach.samples[0].max_penetration_m),
        }
        for witness_name, witness in witnesses.items():
            desired = np.asarray(witness["desired_nm"], dtype=np.float64)
            report["short_horizon"][witness_name] = {}
            for realization_name, declared in (
                    ("nominal", RES86_NOMINAL_DECLARED_SOLREF),
                    ("candidate", RES86_CANDIDATE_DECLARED_SOLREF)):
                for dt in DT_GRID_S:
                    steps = int(round(SHORT_HORIZON_S / dt))
                    sequence = _resample(desired[:steps], PRODUCTION_DT_S, dt)[:steps]
                    plant = build_contact_realization_plant(declared, dt_s=dt)
                    oracle = LandingFeasibilityOracle(branch, plant=plant, native_dt_s=dt)
                    metrics = _replay(oracle, sequence, dt=dt)
                    key = f"{realization_name}_dt_{dt:.3f}"
                    report["short_horizon"][witness_name][key] = metrics
                    print(json.dumps({
                        "witness": witness_name, "realization": realization_name, "dt": dt,
                        "penetration_m": metrics["max_penetration_m"],
                        "peak_fz_bw": metrics["peak_total_fz_bw"],
                        "rom_margin": metrics["min_rom_margin_rad"],
                        "failures": metrics["hard_gate_failures"],
                        "deterministic": metrics["deterministic_replay"],
                    }), flush=True)

        print("phase 3: full-horizon witness battery ...", flush=True)
        for witness_name, witness in witnesses.items():
            desired = np.asarray(witness["desired_nm"], dtype=np.float64)
            report["battery"][witness_name] = {}
            for realization_name, declared in (
                    ("nominal", RES86_NOMINAL_DECLARED_SOLREF),
                    ("candidate", RES86_CANDIDATE_DECLARED_SOLREF)):
                for dt in DT_GRID_S:
                    plant = build_contact_realization_plant(declared, dt_s=dt)
                    oracle = LandingFeasibilityOracle(branch, plant=plant, native_dt_s=dt)
                    sequence = _resample(desired, PRODUCTION_DT_S, dt)
                    metrics = _replay(oracle, sequence, dt=dt)
                    report["battery"][witness_name][
                        f"{realization_name}_dt_{dt:.3f}"] = metrics
                    print(json.dumps({
                        "witness": witness_name, "realization": realization_name, "dt": dt,
                        "penetration_m": metrics["max_penetration_m"],
                        "peak_fz_bw": metrics["peak_total_fz_bw"],
                        "net_impulse_ns": metrics["net_vertical_impulse_ns"],
                        "rom_margin": metrics["min_rom_margin_rad"],
                        "chatter": metrics["chatter_transitions"],
                        "reflight": metrics["material_reflight"],
                        "prohibited": metrics["prohibited_any"],
                        "deterministic": metrics["deterministic_replay"],
                        "admissible": metrics["admissible"],
                        "failures": metrics["hard_gate_failures"],
                    }), flush=True)

    # ------------------------------------------------------------------
    # declared criteria
    # ------------------------------------------------------------------
    checks: dict[str, object] = {}
    if report["realization_audit"]:
        identity_ok = True
        semantics_ok = True
        resolution_ok = True
        details: dict[str, object] = {}
        nominal_semantics = None
        for key, cell in report["realization_audit"].items():
            nominal = cell["nominal"]
            candidate = cell["candidate"]
            identity_ok = identity_ok and bool(
                candidate["realized_status"] == "PASS"
                and nominal["realized_status"] == "PASS"
                and tuple(candidate["plantar_rows"][0]["solref"])
                == RES86_CANDIDATE_REALIZED_SOLREF
                and tuple(nominal["plantar_rows"][0]["solref"])
                == RES86_NOMINAL_REALIZED_SOLREF
                and all(row["dim"] == 4 for row in candidate["plantar_rows"])
                and all(tuple(row["solimp"]) == DECLARED_SOLIMP
                        for row in candidate["plantar_rows"]))
            if nominal_semantics is None:
                nominal_semantics = dict(nominal["solver_semantics"])
            for name in ("integrator", "cone", "solver", "iterations", "tolerance",
                         "ls_iterations", "disableflags"):
                semantics_ok = semantics_ok and bool(
                    candidate["solver_semantics"][name]
                    == nominal_semantics[name] == nominal["solver_semantics"][name])
            semantics_ok = semantics_ok and bool(
                candidate["solver_semantics"]["refsafe_enabled"]
                and not candidate["solver_semantics"]["contact_disabled"])
            resolution_ok = resolution_ok and bool(
                nominal["timeconst_resolution"]["resolved"]
                and candidate["timeconst_resolution"]["resolved"])
            details[key] = {
                "candidate_realized": candidate["plantar_rows"][0]["solref"],
                "nominal_realized": nominal["plantar_rows"][0]["solref"],
                "candidate_timeconst_over_dt":
                    candidate["timeconst_resolution"]["timeconst_over_dt"],
                "nominal_timeconst_over_dt":
                    nominal["timeconst_resolution"]["timeconst_over_dt"],
            }
        checks["V1_realization_identity"] = bool(identity_ok)
        checks["V2_engine_semantics_unchanged"] = bool(semantics_ok)
        checks["V3_timeconst_resolved"] = bool(resolution_ok)
        checks["V3_detail"] = details

    if report["short_horizon"]:
        reference = report["short_horizon"][SHORT_HORIZON_REFERENCE_WITNESS]
        in_domain_ok = True
        causality = False
        in_domain_detail: dict[str, object] = {}
        for dt in DT_GRID_S:
            cell = reference[f"nominal_dt_{dt:.3f}"]
            candidate_cell = reference[f"candidate_dt_{dt:.3f}"]
            row = {
                "nominal_penetration_m": cell["max_penetration_m"],
                "candidate_penetration_m": candidate_cell["max_penetration_m"],
                "nominal_failures": cell["hard_gate_failures"],
                "candidate_failures": candidate_cell["hard_gate_failures"],
                "nominal_peak_fz_bw": cell["peak_total_fz_bw"],
                "candidate_peak_fz_bw": candidate_cell["peak_total_fz_bw"],
                "deterministic": bool(cell["deterministic_replay"]
                                      and candidate_cell["deterministic_replay"]),
            }
            in_domain_detail[f"dt_{dt:.3f}"] = row
            if dt in IN_DOMAIN_DT_S:
                in_domain_ok = in_domain_ok and bool(
                    candidate_cell["max_penetration_m"] < cell["max_penetration_m"]
                    and set(candidate_cell["hard_gate_failures"]).issubset(
                        set(cell["hard_gate_failures"]))
                    and candidate_cell["deterministic_replay"]
                    and not candidate_cell["prohibited_any"]
                    and not candidate_cell["material_reflight"]
                    and row["deterministic"])
            delta = candidate_cell["max_penetration_m"] - cell["max_penetration_m"]
            relative = abs(delta) / max(abs(cell["max_penetration_m"]), 1.0e-12)
            if relative > 0.01:
                causality = True
        checks["V7_material_causality"] = bool(causality)
        checks["V8_in_domain_dt_comparison"] = bool(in_domain_ok)
        checks["V8_detail"] = in_domain_detail
        stress_ok = True
        for dt in STRESS_DT_S:
            cell = reference[f"nominal_dt_{dt:.3f}"]
            candidate_cell = reference[f"candidate_dt_{dt:.3f}"]
            stress_ok = stress_ok and bool(
                candidate_cell["max_penetration_m"] <= cell["max_penetration_m"]
                and candidate_cell["max_penetration_m"] > PENETRATION_LIMIT_M)
        checks["V9_stress_dt_is_out_of_domain"] = bool(stress_ok)
        checks["V9_detail"] = {
            f"dt_{dt:.3f}": {
                "nominal_penetration_m": reference[f"nominal_dt_{dt:.3f}"]["max_penetration_m"],
                "candidate_penetration_m": reference[f"candidate_dt_{dt:.3f}"]["max_penetration_m"],
                "impact_step_m": abs(report["approach_state"]["one_step_com_vz_m_s"]) * dt,
            } for dt in STRESS_DT_S}

    if report["battery"]:
        determinism = True
        reproduction = True
        reproduction_detail: dict[str, object] = {}
        nominal_hard = None
        candidate_hard = None
        for witness_name, cells in report["battery"].items():
            recorded = witnesses[witness_name]["recorded_outcome"]
            for key, metrics in cells.items():
                determinism = determinism and bool(metrics["deterministic_replay"])
                if witness_name == "candidate_witness" and key == "candidate_dt_0.002":
                    candidate_hard = metrics
                    reproduce_checks = _reproduce(recorded, metrics)
                elif witness_name == "nominal_witness" and key == "nominal_dt_0.002":
                    nominal_hard = metrics
                    reproduce_checks = _reproduce(recorded, metrics)
                else:
                    reproduce_checks = {}
                for ok in reproduce_checks.values():
                    reproduction = reproduction and bool(ok)
                reproduction_detail[f"{witness_name}:{key}"] = reproduce_checks
        checks["V4_replay_determinism"] = bool(determinism)
        checks["V5_witness_reproduction"] = bool(reproduction)
        checks["V5_detail"] = reproduction_detail
        checks["V6_candidate_admissible"] = bool(
            candidate_hard is not None and candidate_hard["admissible"])
        checks["V6_nominal_boundary_failure"] = (
            None if nominal_hard is None else list(nominal_hard["hard_gate_failures"]))
        if candidate_hard is not None:
            checks["V10_candidate_hard_authority_hygiene"] = bool(
                candidate_hard["max_moment_ratio"] <= 1.0 + 1.0e-9
                and candidate_hard["max_power_ratio"] <= 1.0 + 1.0e-9
                and candidate_hard["mtp_authority_ok"]
                and candidate_hard["min_rom_margin_rad"]
                >= -V3_STRUCTURAL_ROM_TOLERANCE_RAD
                and candidate_hard["peak_total_fz_bw"] <= PEAK_FZ_LIMIT_BW
                and candidate_hard["chatter_transitions"] <= CHATTER_LIMIT
                and not candidate_hard["prohibited_any"]
                and not candidate_hard["material_reflight"])

    report["criteria"] = checks
    required = ("V1_realization_identity", "V2_engine_semantics_unchanged",
                "V3_timeconst_resolved", "V4_replay_determinism",
                "V5_witness_reproduction", "V6_candidate_admissible",
                "V7_material_causality", "V8_in_domain_dt_comparison",
                "V9_stress_dt_is_out_of_domain",
                "V10_candidate_hard_authority_hygiene")
    missing = [name for name in required if name not in checks]
    failed = [name for name in required if name in checks and not bool(checks[name])]
    report["status"] = "PASS" if not missing and not failed else "FAIL"
    report["required_criteria"] = list(required)
    report["missing_criteria"] = missing
    report["failed_criteria"] = failed
    summary = {name: checks.get(name) for name in required}
    summary["V6_nominal_boundary_failure"] = checks.get("V6_nominal_boundary_failure")
    print(json.dumps({"status": report["status"], "criteria": summary},
                     indent=2, default=str))

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=1, default=str) + "\n")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
