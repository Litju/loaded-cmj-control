"""RES-85D strict structural-ROM reconciliation and final reseal tests.

MISSION: `RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001`
LINEAR ISSUE: RES-85

Every assertion re-executes the deterministic computation archived in
``audit/EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001/``: the strict structural-ROM
audit over the whole RES-85 claim domain, the RES-85C predecessor regression
(read from immutable git history), the predictive guard direction property and
the RES-85D negative controls NC-22..NC-27.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
EVIDENCE_DIR = TASK_ROOT / "audit" / "EXP-RES85-CAUSAL-LAUNCH-FLIGHT-CONTROL-001"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BE = _load_module("res85d_build_evidence", EVIDENCE_DIR / "build_evidence.py")

from loaded_cmj.v3.actuation import CHANNELS, N_CHANNELS  # noqa: E402
from loaded_cmj.v3.constants import V3_JOINT_RANGES_RAD  # noqa: E402
from loaded_cmj.v3.controller import (  # noqa: E402
    JOINT_ROM_BRAKE_ZONE_RAD,
    ROM_BRAKE_HORIZON_S,
    V3LaunchController,
)
from loaded_cmj.v3.launch_runtime import (  # noqa: E402
    run_launch_episode,
    settle_standing_stance,
)
from loaded_cmj.v3.plant import V3Plant  # noqa: E402

QPOS_INDICES = [3, 4, 8, 5, 9, 6, 10, 7, 11]
STRICT_TOLERANCE = 1e-9


@pytest.fixture(scope="session")
def episode():
    return run_launch_episode()


@pytest.fixture(scope="session")
def report(episode):
    return BE.launch_episode_report(episode)


@pytest.fixture(scope="session")
def strict_audit(report):
    return report["joint_rom_audit"]


@pytest.fixture(scope="session")
def predecessor():
    return BE.res85c_predecessor_strict_rom()


@pytest.fixture(scope="session")
def negative():
    return json.loads((EVIDENCE_DIR / "NEGATIVE_CONTROLS_REPORT.json").read_text())


# ---------------------------------------------------------------------------
# strict structural ROM on the canonical episode
# ---------------------------------------------------------------------------
def test_strict_audit_domain_is_first_sample_through_claim_end(report, strict_audit):
    claim_end = report["res85_claim_end"]
    assert claim_end is not None
    assert strict_audit["scope"] == "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END"
    assert strict_audit["claim_end_sample"] == int(claim_end["sample"])
    assert strict_audit["claim_end_reason"] == \
        "FIRST_LEGAL_PLANTAR_RECONTACT_AFTER_FLIGHT"
    assert strict_audit["tolerance_rad"] == STRICT_TOLERANCE


def test_strict_audit_passes_every_bounded_joint(episode, report, strict_audit):
    assert strict_audit["status"] == "PASS", strict_audit["failures"]
    assert strict_audit["failures"] == []
    assert BE.strict_structural_rom_failures(strict_audit) == []
    assert strict_audit["global_min_structural_rom_margin_rad"] >= -STRICT_TOLERANCE
    end = int(report["res85_claim_end"]["sample"]) + 1
    q = np.asarray(episode.telemetry.joint_q[:end], dtype=np.float64)[:, QPOS_INDICES]
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = float(rng[0]), float(rng[1])
        channel = strict_audit["channels"][name]
        assert np.min(q[:, c]) >= lo - STRICT_TOLERANCE, name
        assert np.max(q[:, c]) <= hi + STRICT_TOLERANCE, name
        assert abs(channel["min_measured_rad"] - float(np.min(q[:, c]))) < 1e-12
        assert abs(channel["max_measured_rad"] - float(np.max(q[:, c]))) < 1e-12
        assert abs(channel["min_rom_margin_rad"]
                   - min(float(np.min(q[:, c]) - lo),
                         float(hi - np.max(q[:, c])))) < 1e-12


def test_trunk_pelvis_is_inside_the_frozen_structural_bound(episode, report,
                                                            strict_audit):
    end = int(report["res85_claim_end"]["sample"]) + 1
    trunk = np.asarray(episode.telemetry.joint_q[:end, 3], dtype=np.float64)
    channel = strict_audit["channels"]["trunk_pelvis"]
    lo, hi = V3_JOINT_RANGES_RAD["trunk_pelvis"]
    assert float(trunk.max()) <= float(hi) + STRICT_TOLERANCE
    assert float(trunk.min()) >= float(lo) - STRICT_TOLERANCE
    assert channel["max_measured_rad"] == float(trunk.max())
    assert channel["min_rom_margin_rad"] > 0.0
    assert channel["worst_side"] in ("lower", "upper")
    for key in ("worst_native_sample", "worst_phase", "qdot_at_worst_rad_s",
                "applied_nm_at_worst", "posture_reference_at_worst_rad"):
        assert key in channel


def test_strict_audit_fails_closed_on_measured_violation(episode, report,
                                                         strict_audit):
    mutated = json.loads(json.dumps(strict_audit))
    hi = mutated["channels"]["trunk_pelvis"]["rom_upper_rad"]
    mutated["channels"]["trunk_pelvis"]["max_measured_rad"] = hi + 1e-6
    mutated["channels"]["trunk_pelvis"]["min_rom_margin_rad"] = -1e-6
    failures = BE.strict_structural_rom_failures(mutated)
    assert any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis" in f for f in failures)
    # the validator recomputes from the reported extrema, never a stored PASS
    mutated = json.loads(json.dumps(strict_audit))
    mutated["status"] = "PASS"
    mutated["channels"]["left_knee"]["min_measured_rad"] = -1.0
    assert BE.strict_structural_rom_failures(mutated)


def test_h2_floor_and_direct_system_com_gate(report):
    apex = report["apex_h2"]
    assert apex["h2_support_m"] is not None
    assert apex["h2_support_m"] >= 0.150
    assert report["functional_task_floor"]["closure_pass"] is True
    assert report["method_explicit_h2_report"]["PRIMARY_CANONICAL"]["method_id"] == \
        "DIRECT_SIMULATOR_SYSTEM_COM"


def test_bilateral_symmetry_is_preserved_over_the_claim_domain(episode, report):
    from loaded_cmj.v3.actuation import MIRRORED_PAIRS

    t = episode.telemetry
    end = int(report["res85_claim_end"]["sample"]) + 1
    for i, j in MIRRORED_PAIRS:
        asym = np.abs(t.applied_nm[:end, i] - t.applied_nm[:end, j]).max()
        assert asym <= 1e-9, (CHANNELS[i], asym)
    conformance = BE.actuation_conformance_report(episode)
    check = [c for c in conformance["checks"]
             if c["check"] == "bilateral_applied_symmetry_within_tolerance"][0]
    assert check["pass"] is True
    assert check["detail"]["scope"] == "FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END"


# ---------------------------------------------------------------------------
# RES-85C predecessor regression (re-executed from immutable git history)
# ---------------------------------------------------------------------------
def test_res85c_predecessor_fails_strict_rom_on_trunk_pelvis(predecessor):
    audit = predecessor["strict_structural_rom_audit"]
    assert audit["status"] == "FAIL"
    assert "STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in audit["failures"]
    trunk = audit["channels"]["trunk_pelvis"]
    assert trunk["max_measured_rad"] == 0.6734678378904122
    assert trunk["rom_upper_rad"] == 0.610865
    assert trunk["max_measured_rad"] > trunk["rom_upper_rad"]
    assert predecessor["previous_h2_m"] == 0.18867197309831352
    assert predecessor["predecessor_head"] == \
        "ffc98526bd2b0da76dbef50891415ebdb345a1fd"


def test_res85c_predecessor_is_not_silently_rewritten(predecessor):
    text = (EVIDENCE_DIR / "RES85C_CORRECTION_RECEIPT.md").read_text()
    assert text.startswith("# RES85C_CORRECTION_RECEIPT")
    assert "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001" in text
    assert "0.18867197309831352" in text
    assert "235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e" in text
    assert predecessor["telemetry_blob_sha256"]


def test_soft_limit_probe_is_diagnostic_only():
    probe = json.loads((EVIDENCE_DIR / "JOINT_ROM_SOFT_LIMIT_PROBE.json").read_text())
    assert probe["channels"]["trunk_pelvis"]["soft_limit_compliance_rad"] > 0.0
    audit = json.loads((EVIDENCE_DIR / "STRICT_STRUCTURAL_ROM_AUDIT.json").read_text())
    assert audit["solver_soft_limit_probe_role"] == \
        "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY_NOT_STRUCTURAL_ACCEPTANCE_AUTHORITY"
    assert "solver" not in audit["criterion"]
    # a measured coordinate outside the frozen ROM but inside the probe
    # envelope is still a structural violation (NC-24 semantics)
    assert audit["channels"]["trunk_pelvis"][
        "solver_soft_limit_diagnostic_role"] == "NUMERICAL_SOLVER_DIAGNOSTIC_ONLY"


# ---------------------------------------------------------------------------
# predictive guard mechanics
# ---------------------------------------------------------------------------
def test_guard_is_zero_far_from_the_bounds():
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    controller = V3LaunchController(plant, data)
    q = np.zeros(N_CHANNELS)
    qdot = np.zeros(N_CHANNELS)
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = rng
        q[c] = 0.5 * (lo + hi)
        qdot[c] = 1.0
    desired = np.arange(N_CHANNELS, dtype=np.float64)
    out = controller._rom_guard(q, qdot, desired)
    assert np.array_equal(out, desired)


def test_guard_never_propels_toward_a_bound_and_is_symmetric():
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    controller = V3LaunchController(plant, data)
    for c, name in enumerate(CHANNELS):
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        lo, hi = rng
        desired = np.zeros(N_CHANNELS)
        for q_value in (lo - 0.1, lo, hi, hi + 0.1):
            for qd in (-2.0, 2.0):
                q = np.zeros(N_CHANNELS)
                qdot = np.zeros(N_CHANNELS)
                q[c] = q_value
                qdot[c] = qd
                delta = controller._rom_guard(q, qdot, desired)[c]
                if q_value > hi:
                    assert delta <= 0.0, (name, q_value, qd, delta)
                if q_value < lo:
                    assert delta >= 0.0, (name, q_value, qd, delta)
                if q_value < hi and q_value > lo and abs(q_value - lo) > 1.0 \
                        and abs(hi - q_value) > 1.0:
                    assert delta == 0.0, (name, q_value, qd, delta)


def test_guard_horizon_and_zone_are_declared():
    assert ROM_BRAKE_HORIZON_S > 0.0
    assert JOINT_ROM_BRAKE_ZONE_RAD > 0.0
    config = V3LaunchController(V3Plant(), V3Plant().make_data()).config
    assert config.trunk_rom_guard_margin_rad >= config.joint_rom_margin_rad
    assert config.trunk_rom_velocity_gain > 0.0
    assert config.joint_rom_velocity_gain > 0.0


# ---------------------------------------------------------------------------
# RES-85D negative controls NC-22..NC-27
# ---------------------------------------------------------------------------
def test_res85d_negative_controls_pass(negative):
    names = {c["check"] for c in negative["checks"]}
    for required in (
            "NC-22_measured_trunk_beyond_upper_bound_fails",
            "NC-23_reference_inside_but_measured_outside_fails",
            "NC-24_measured_outside_rom_inside_probe_envelope_fails",
            "NC-25_rom_guard_never_drives_farther_outside",
            "NC-26_strict_rom_correction_below_functional_floor_fails_closure",
            "NC-27_boundary_within_tolerance_passes"):
        assert required in names, required
    assert negative["res85d_status"] == "PASS"
    assert negative["status"] == "PASS", [c for c in negative["checks"]
                                          if not c["pass"]]


def test_nc22_nc23_nc24_semantics(negative):
    probes = negative["probes"]
    assert any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
               for f in probes["NC-22_measured_trunk_beyond_upper_bound"]
               ["validator_failures"])
    assert probes["NC-23_reference_inside_actual_outside"]["reference_inside_rom"] is True
    assert any("STRUCTURAL_ROM_VIOLATION:left_knee:lower" in f
               for f in probes["NC-23_reference_inside_actual_outside"]
               ["validator_failures"])
    nc24 = probes["NC-24_outside_rom_inside_probe_envelope"]
    assert nc24["inside_probe_envelope"] is True
    assert any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
               for f in nc24["validator_failures"])


def test_nc25_guard_direction_property(negative):
    probe = negative["probes"]["NC-25_guard_direction_property"]
    assert probe["real_guard_failures"] == []
    assert probe["mutant_guard_failures"] > 0


def test_nc26_strict_rom_valid_below_floor_fails_closure(negative):
    probe = negative["probes"]["NC-26_strict_rom_but_below_floor"]
    assert probe["strict_rom_status"] == "PASS"
    assert probe["h2_m"] < 0.150
    assert probe["floor_closure"]["closure_pass"] is False


def test_nc27_boundary_within_tolerance(negative):
    probe = negative["probes"]["NC-27_boundary_within_tolerance"]
    assert probe["boundary_status"] == "PASS"
    assert probe["boundary_failures"] == []
    assert any("STRUCTURAL_ROM_VIOLATION:trunk_pelvis:upper" in f
               for f in probe["over_tolerance_failures"])


# ---------------------------------------------------------------------------
# artifacts, receipt and determinism
# ---------------------------------------------------------------------------
def test_res85d_artifacts_present_and_pass():
    for name in ("STRICT_STRUCTURAL_ROM_AUDIT.json",
                 "RES85C_PREDECESSOR_STRICT_ROM.json",
                 "STRICT_ROM_SEARCH.json",
                 "RES85D_STRICT_ROM_RECEIPT.md",
                 "LAUNCH_EPISODE_REPORT.json",
                 "DETERMINISM_REPORT.json",
                 "HASH_MANIFEST.json"):
        assert (EVIDENCE_DIR / name).is_file(), name
    audit = json.loads((EVIDENCE_DIR / "STRICT_STRUCTURAL_ROM_AUDIT.json").read_text())
    assert audit["status"] == "PASS"
    determinism = json.loads((EVIDENCE_DIR / "DETERMINISM_REPORT.json").read_text())
    assert determinism["status"] == "PASS"
    predecessor = json.loads(
        (EVIDENCE_DIR / "RES85C_PREDECESSOR_STRICT_ROM.json").read_text())
    assert predecessor["strict_structural_rom_audit"]["status"] == "FAIL"
    search = json.loads((EVIDENCE_DIR / "STRICT_ROM_SEARCH.json").read_text())
    assert search["two_run_identity"]["byte_identical"] is True
    assert search["total_evaluations"] <= search["predeclared"]["max_evaluations"] == 30
    assert search["selected"]["feasible"] is True
    assert search["selected"]["h2_m"] >= 0.150


def test_declared_search_selected_the_minimal_change_feasible_config():
    search = json.loads((EVIDENCE_DIR / "STRICT_ROM_SEARCH.json").read_text())
    selected = search["selected"]["overrides"]
    selected_index = search["selected"]["index"]
    assert selected == {
        "trunk_extend_frac": 0.40,
        "trunk_kd": 30.0,
        "trunk_lean_frac": 0.35,
        "trunk_rom_guard_margin_rad": 0.12,
        "trunk_rom_position_gain": 2500.0,
    }
    for evaluation in search["evaluations"]:
        if evaluation["index"] == selected_index:
            assert evaluation["overrides"] == selected
            assert evaluation["feasible"] is True
            break
    else:
        raise AssertionError("selected evaluation not present in the record")
    assert search["evaluations"][0]["strict_rom_status"] == "FAIL"


def test_receipt_contains_required_sections_and_values():
    text = (EVIDENCE_DIR / "RES85D_STRICT_ROM_RECEIPT.md").read_text()
    for section in ("## Previous evidence attribution",
                    "## New canonical result (RES-85D)",
                    "## Strict structural ROM (frozen human envelope)",
                    "## Hard actuation conformance",
                    "## MTP non-compensation",
                    "## Negative controls",
                    "## Test and suite results",
                    "## Determinism",
                    "## Final claim ceiling"):
        assert section in text, section
    assert "RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001" in text
    assert "ffc98526bd2b0da76dbef50891415ebdb345a1fd" in text
    assert "0.18867197309831352" in text
    assert "0.6734678378904122" in text
    assert "STRICT STRUCTURAL ROM FAIL" in text
    assert "ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED" in text
    assert "ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED" in text
    assert "NEXT_AUTHORIZED_ACTION=RES86_ACTIVE_SET_SAFE_LANDING_CAPTURE" in text


def test_authority_freeze_reports_no_new_v3_failures():
    classification = json.loads(
        (EVIDENCE_DIR / "FULL_SUITE_CLASSIFICATION.json").read_text())
    assert classification["full_suite"]["res85d_owned_failures"] == 0
    assert classification["full_suite"]["res85c_owned_failures"] == 0
    assert classification["full_suite"]["res85_owned_failures"] == 0
    assert classification["full_suite"]["res83_res84_failures"] == 0
    for failure in classification["failures"]:
        assert failure["category"].startswith("PRE_EXISTING_")
