"""RES-85 Achievement A — deterministic scientific-control-authority checks.

Authority: LCMJ_RES85_*_V1 (this bundle)
Mission:   RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001

Every check below is deterministic and re-executed by
``tests/test_res85_v3_causal_launch.py``.  The script writes:

    AUTHORITY_CHECK_REPORT.json
    AUTHORITY_REDTEAM_REPORT.json
    SOURCE_PROVENANCE.json
    HASH_MANIFEST.json

Run:  python3 build_authority_checks.py --print
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SRC = REPO / "src"

PLANT_XML = SRC / "loaded_cmj" / "v3" / "assets" / "v3_plant.xml"
MEASUREMENT_SRC = SRC / "loaded_cmj" / "v3" / "measurement.py"
PLANT_SRC = SRC / "loaded_cmj" / "v3" / "plant.py"
CONSTANTS_SRC = SRC / "loaded_cmj" / "v3" / "constants.py"

RES83_PLANT_XML_SHA256 = "eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d"
RES84_EVIDENCE_SEAL_SHA256 = "223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda"
RES84_BUNDLE = "EXP-RES84-V3-MEASUREMENT-CONTACT-AUTHORITY-001"

AUTHORITY_FILES = (
    "METHOD_COMPARATOR_PANEL.json",
    "ACTUATION_AUTHORITY.json",
    "MTP_ENERGY_AUTHORITY.json",
    "PHASE_MACHINE_AUTHORITY.json",
)

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"

R001_FORBIDDEN_LITERALS = (
    "0.07516", "0.07684", "0.03627", "0.0752",
    "9.6837", "0.648125", "14.43625", "0.9439",
    "1.21446", "1.22743",
)
V2_TORQUE_LITERALS = ("250.0", "300.0", "200.0")
WITHDRAWN_HEIGHT_LITERALS = ("0.20 m", "0.28", "0.35", "0.40")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def load(name: str) -> dict[str, Any]:
    return json.loads((HERE / name).read_text())


def _check(checks: list[dict[str, Any]], name: str, passed: bool, detail: Any = "") -> None:
    checks.append({"check": name, "pass": bool(passed), "detail": detail})


def _status(checks: list[dict[str, Any]]) -> str:
    return STATUS_PASS if all(c["pass"] for c in checks) else STATUS_FAIL


def _git(*args: str) -> str:
    proc = subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True)
    return proc.stdout.strip()


# ===========================================================================
# 1. Method comparator panel
# ===========================================================================
def method_comparator_checks() -> dict[str, Any]:
    panel = load("METHOD_COMPARATOR_PANEL.json")
    checks: list[dict[str, Any]] = []
    h2 = panel["h2_authority"]
    _check(checks, "h2_definition_frozen",
           h2["definition"] == "SYSTEM_COM_z(APEX) - SYSTEM_COM_z(TAKEOFF_OCCURRENCE)",
           h2["definition"])
    _check(checks, "h2_origin_is_res84_occurrence",
           h2["origin_authority"] == "LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1")
    _check(checks, "elite_h2_hard_gate_not_established",
           h2["elite_soccer_plus20_h2_hard_gate"] == "NOT_ESTABLISHED")
    _check(checks, "elite_h2_target_not_established",
           h2["elite_soccer_plus20_h2_target"] == "NOT_ESTABLISHED")
    floor = h2["h_anti_triviality_floor"]
    _check(checks, "functional_floor_value_declared",
           floor["symbol"] == "H_ANTI_TRIVIALITY_FLOOR" and floor["value_m"] == 0.150)
    _check(checks, "functional_floor_role_declared",
           floor["role"] == "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY")
    _check(checks, "functional_floor_is_minimum_success_condition",
           floor["is_minimum_functional_success_condition"] is True
           and "minimum functional success condition" in floor["closure_rule"])
    _check(checks, "functional_floor_is_not_elite_norm_or_target",
           floor["is_elite_performance_norm"] is False
           and floor["is_optimization_target"] is False
           and floor["is_expected_value"] is False
           and floor["is_population_claim"] is False)
    statements = " ".join(floor["explicit_statements"])
    _check(checks, "functional_floor_explicitly_negates_elite_framing",
           "NOT an elite-performance norm" in statements
           and "NOT an optimization target" in statements
           and "IS a minimum functional success condition" in statements)
    _check(checks, "functional_floor_ballistic_cross_check_declared",
           "1.716" in floor["diagnostic_scale_cross_check"]["rule"]
           and "never replaced" in floor["diagnostic_scale_cross_check"]["use"])
    methods = {m["method_id"] for m in panel["methods"]}
    _check(checks, "four_method_families_separated",
           methods == {
               "DIRECT_SIMULATOR_SYSTEM_COM",
               "FORCE_PLATFORM_IMPULSE_MOMENTUM",
               "FORCE_PLATFORM_FLIGHT_TIME",
               "BAR_LVT_DISPLACEMENT_VELOCITY",
           }, sorted(methods))
    primary = [m for m in panel["methods"] if m["status"] == "PRIMARY_CANONICAL"]
    _check(checks, "exactly_one_primary_method",
           len(primary) == 1 and primary[0]["method_id"] == "DIRECT_SIMULATOR_SYSTEM_COM")
    _check(checks, "cross_method_conversion_refused",
           panel["conversion_policy"]["cross_method_conversion_status"] == "NOT_ESTABLISHED"
           and panel["conversion_policy"]["single_target_conversion"] == "PROHIBITED")
    must_not = panel["res85_reporting_requirements"]["must_not"]
    _check(checks, "method_qualified_reporting_required",
           not any("gate task success on 0.150 m" in item for item in must_not)
           and any("elite norm" in item for item in must_not)
           and any("close the task with an H2 below H_ANTI_TRIVIALITY_FLOOR" in item
                   for item in must_not)
           and any("unqualified 'jump height'" in item for item in must_not))
    must_report = " ".join(panel["res85_reporting_requirements"]["must_report"])
    _check(checks, "floor_classification_is_reported",
           "H_ANTI_TRIVIALITY_FLOOR classification" in must_report)
    _check(checks, "no_elite_h2_target_asserted",
           panel["red_team_assertions"]["no_elite_h2_target"] is True
           and panel["red_team_assertions"]["functional_floor_is_declared"] is True)
    bar = [m for m in panel["methods"] if m["method_id"] == "BAR_LVT_DISPLACEMENT_VELOCITY"][0]
    _check(checks, "bar_lvt_bias_documented",
           any("overestimation" in b for b in bar["known_biases"]))
    return {"status": _status(checks), "checks": checks, "panel": "METHOD_COMPARATOR_PANEL.json"}


# ===========================================================================
# 2. Actuation authority
# ===========================================================================
def actuation_checks() -> dict[str, Any]:
    auth = load("ACTUATION_AUTHORITY.json")
    checks: list[dict[str, Any]] = []
    expected = ["trunk_pelvis", "left_hip", "right_hip", "left_knee", "right_knee",
                "left_ankle", "right_ankle", "left_mtp", "right_mtp"]
    got = [c["channel"] for c in auth["channels"]]
    _check(checks, "nine_channels_exact", got == expected, got)
    for ch in auth["channels"]:
        name = ch["channel"]
        for key in ("moment_ceiling_nm", "power_ceiling_w", "torque_rate_ceiling_nm_per_s",
                    "provenance_class", "evidence_class", "sensitivity_role"):
            _check(checks, f"{name}.{key}_defined", bool(ch.get(key)), ch.get(key))
        _check(checks, f"{name}.ceiling_positive", ch["moment_ceiling_nm"] > 0.0)
        _check(checks, f"{name}.provenance_class_honest",
               ch["provenance_class"] in (
                   "ENGINEERING_NOMINAL_WITH_SENSITIVITY",
                   "DYNAMIC_EVIDENCE_DIRECT",
               ), ch["provenance_class"])
        _check(checks, f"{name}.sensitivity_band_monotone",
               list(ch["sensitivity_values"]) == sorted(ch["sensitivity_values"]))
    # bilateral symmetry: mirrored pairs identical
    by_name = {c["channel"]: c for c in auth["channels"]}
    for lname, rname in (("left_hip", "right_hip"), ("left_knee", "right_knee"),
                         ("left_ankle", "right_ankle"), ("left_mtp", "right_mtp")):
        left_ch, right_ch = by_name[lname], by_name[rname]
        _check(checks, f"pair_identical.{lname}",
               left_ch["moment_ceiling_nm"] == right_ch["moment_ceiling_nm"]
               and left_ch["power_ceiling_w"] == right_ch["power_ceiling_w"]
               and left_ch["torque_rate_ceiling_nm_per_s"] == right_ch["torque_rate_ceiling_nm_per_s"])
    sym = auth["bilateral_symmetry_rules"]
    _check(checks, "symmetry_nominal_mode_enforced",
           sym["nominal_mode"] == "ENFORCED_FOR_ALL_RES85_PHASES")
    _check(checks, "symmetry_covers_all_pairs", len(sym["mirrored_pairs"]) == 4)
    _check(checks, "no_plant_mutation_declared",
           auth["ctrl_semantics"]["no_plant_mutation"].startswith("the authority is enforced"))
    order = auth["enforcement_order"]
    _check(checks, "enforcement_order_frozen",
           auth["enforcement_order_frozen"] is True and len(order) == 8)
    _check(checks, "previous_applied_torque_is_sole_history",
           any("sole history state" in entry for entry in order))
    _check(checks, "safe_interval_projection_declared",
           "feasible_interval_note" in auth
           and "projection onto the intersection of the hard-safety interval" in auth["feasible_interval_note"])
    # RES-85C coherent actuation contract: nominal slew bound + hard safety bounds
    rate_sem = auth["torque_rate_semantics"]
    _check(checks, "torque_rate_is_nominal_slew_bound",
           rate_sem["role"] == "NOMINAL_SLEW_BOUND"
           and rate_sem["hard_ceiling_claim"] is False
           and auth["torque_rate_semantics"]["primary_authority"] == "NOMINAL_SLEW_BOUND")
    _check(checks, "hard_safety_bounds_declared",
           auth["hard_safety_bounds_semantics"]["bounds"]
           == ["moment_ceiling", "joint_power_ceiling", "mtp_energy_gate"]
           and auth["hard_safety_bounds_semantics"]["never_exceeded"] is True
           and "always wins" in auth["hard_safety_bounds_semantics"]["precedence"])
    _check(checks, "safety_override_attribution_declared",
           "exactly one binding hard constraint"
           in auth["hard_safety_bounds_semantics"]["attribution"]
           and "violated nominal slew margin" in rate_sem["override_rule"]
           and ("never reported as nominal-slew compliant" in order[5]
                or "never reported as nominal-slew compliant" in rate_sem["override_rule"]))
    _check(checks, "no_emergency_precedence_over_hard_bound",
           "power_emergency" not in json.dumps(auth) and "moment_emergency" not in json.dumps(auth))
    _check(checks, "no_activation_filter",
           auth["torque_rate_semantics"]["activation_smoothing_secondary"].startswith("none"))
    _check(checks, "power_enforcement_declared",
           auth["joint_power_semantics"]["enforcement"].startswith(
               "HARD_CEILING_ON_ABS_POWER"))
    # forbidden imports
    v2 = auth["forbidden_imports"]
    _check(checks, "v2_torque_limits_explicitly_rejected",
           "explicitly NOT imported" in v2["v2_torque_limits"]
           and "250" in v2["v2_torque_limits"] and "300" in v2["v2_torque_limits"])
    _check(checks, "r001_values_explicitly_rejected", "r001" in v2["r001_values"].lower())
    _check(checks, "isokinetic_maxima_not_constants",
           "PROHIBITED" in v2["isokinetic_maxima_as_dynamic_constants"])
    return {"status": _status(checks), "checks": checks,
            "authority": "ACTUATION_AUTHORITY.json", "channel_count": len(got)}


# ===========================================================================
# 3. MTP energy authority
# ===========================================================================
def mtp_checks() -> dict[str, Any]:
    auth = load("MTP_ENERGY_AUTHORITY.json")
    checks: list[dict[str, Any]] = []
    d = auth["definitions"]
    _check(checks, "active_moment_defined", "motor actuator" in d["active_moment"]["definition"])
    _check(checks, "passive_moment_is_plant_prior",
           "25 N*m/rad" in d["passive_moment"]["definition"]
           and "not a controller quantity" in d["passive_moment"]["source"])
    _check(checks, "active_and_passive_powers_separate",
           d["active_power"]["definition"] == "tau_active * qdot"
           and d["passive_power"]["definition"] == "tau_passive * qdot")
    _check(checks, "total_power_identity_explicit",
           "P_active + P_passive" in d["total_power"]["identity"])
    _check(checks, "work_identity_explicit", "W_active + W_passive" in d["total_work"]["identity"])
    budgets = auth["budgets_per_foot"]
    _check(checks, "budgets_positive_and_swept",
           budgets["active_positive_work_budget_j"] > 0
           and len(budgets["sensitivity_values_active_budget_j"]) >= 3
           and 0.0 in budgets["sensitivity_values_active_budget_j"])
    gate = auth["supported_phase_active_authority"]["phase_gate"]
    _check(checks, "flight_and_confirm_deny_active_mtp",
           gate["TAKEOFF_CONFIRM"]["active_allowed"] is False
           and gate["FLIGHT"]["active_allowed"] is False
           and gate["LANDING_PREP"]["active_allowed"] is False)
    _check(checks, "propulsion_active_is_gated",
           gate["PROPULSION"]["active_allowed"] is True
           and "late-phase budget fraction" in gate["PROPULSION"]["rationale"])
    late = auth["late_phase_restriction"]
    _check(checks, "late_fraction_declared", late["late_active_positive_work_fraction"] == 0.5)
    zp = auth["zero_passive_sensitivity"]
    _check(checks, "zero_passive_budgets_model_independent",
           "model-independent" in zp["rule"])
    _check(checks, "zero_passive_anti_compensation",
           "violation" in zp["anti_compensation_statement"])
    aei = auth["anti_energy_injection_rule"]
    _check(checks, "aei_rule_has_gate_and_gating",
           len(aei["enforcement"]) == 6
           and aei["enforcement"][3].startswith("AEI-1d")
           and "gated to exactly 0.0" in aei["enforcement"][3])
    _check(checks, "no_double_counting_frozen",
           len(auth["double_counting_prohibitions"]) == 4)
    _check(checks, "ledger_identity_declared",
           "W_total = W_active + W_passive" in auth["energy_ledger_identity"]["statement"])
    return {"status": _status(checks), "checks": checks,
            "authority": "MTP_ENERGY_AUTHORITY.json"}


# ===========================================================================
# 4. Phase machine authority
# ===========================================================================
def phase_machine_checks() -> dict[str, Any]:
    auth = load("PHASE_MACHINE_AUTHORITY.json")
    checks: list[dict[str, Any]] = []
    states = [s["state"] for s in auth["states"]]
    _check(checks, "seven_states_exact",
           states == ["STAND", "COUNTERMOVEMENT", "BRAKING", "PROPULSION",
                      "TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"], states)
    for st in auth["states"]:
        name = st["state"]
        _check(checks, f"{name}.observable_inputs_defined", bool(st.get("observable_inputs")))
        _check(checks, f"{name}.exit_defined",
               bool(st.get("exit_predicate") or st.get("exit_predicates")))
        _check(checks, f"{name}.hysteresis_debounce_defined", bool(st.get("hysteresis_debounce")))
        _check(checks, f"{name}.failure_reversion_defined", bool(st.get("failure_reversion")))
        _check(checks, f"{name}.wall_clock_role_defined", bool(st.get("wall_clock_role")))
        _check(checks, f"{name}.no_wall_clock_authority",
               "none" in st["wall_clock_role"] or "physical-time" in st["wall_clock_role"],
               st["wall_clock_role"])
    banned_keys = ("schedule", "timer", "wall_clock_t", "replay_time", "planned_time",
                   "transition_time", "deadline")
    structural = []
    for st in auth["states"]:
        for key in st:
            if key.lower() in banned_keys:
                structural.append(f"{st['state']}.{key}")
        deb = st.get("hysteresis_debounce", {})
        for key in deb:
            if key.lower() in banned_keys:
                structural.append(f"{st['state']}.hysteresis_debounce.{key}")
    _check(checks, "no_primary_schedule_keys", not structural, structural)
    _check(checks, "braking_to_propulsion_is_reversal",
           "SYSTEM_COM_vz >= 0.0" in json.dumps(auth["states"][2]["hysteresis_debounce"]))
    _check(checks, "propulsion_exit_is_res84_occurrence",
           auth["states"][3]["hysteresis_debounce"]["type"] == "MEASUREMENT_EVENT_ONLY"
           and "support-to-zero" in auth["states"][3]["hysteresis_debounce"]["rule"])
    _check(checks, "comparator_never_takeoff_authority",
           "never the takeoff authority" in json.dumps(auth["states"][3]))
    fc = auth["states"][4]
    _check(checks, "flight_latch_requires_confirmation",
           "TAKEOFF_CONFIRMATION confirmed == true" in json.dumps(fc["exit_predicates"]))
    _check(checks, "recontact_reversion_mapping_frozen",
           fc["reversion_mapping"]["recontact_with_vz_positive"] == "PROPULSION"
           and fc["reversion_mapping"]["recontact_with_vz_nonpositive"] == "BRAKING")
    _check(checks, "provisional_candidate_never_shifted",
           "never shifted" in json.dumps(fc["reversion_mapping"]))
    _check(checks, "flight_is_monotone_latch",
           "cannot be un-entered" in auth["states"][5]["entry"])
    _check(checks, "landing_prep_stops_res85_claim",
           "RES-85 claim ends" in auth["states"][6]["exit_predicate"])
    glob = auth["global_rules"]
    _check(checks, "scorer_private_memory_prohibited",
           "may not read scorer internals" in glob["no_scorer_private_memory"]["rule"])
    _check(checks, "wall_clock_prohibition_frozen",
           "may implement minimum dwell/debounce only" in glob["wall_clock_prohibition"]["statement"])
    _check(checks, "duplicated_event_definitions_prohibited",
           "no local re-implementation" in glob["duplicated_event_definitions"]["prohibition"])
    _check(checks, "fail_closed_required",
           "raises/returns an explicit failure state" in glob["fail_closed"]["rule"])
    _check(checks, "negative_controls_supported",
           len(auth["mandatory_negative_controls_supported_by_this_machine"]) == 7)
    return {"status": _status(checks), "checks": checks,
            "authority": "PHASE_MACHINE_AUTHORITY.json"}


# ===========================================================================
# 5. Red-team scan
# ===========================================================================
def redteam_checks() -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    panel_text = (HERE / "METHOD_COMPARATOR_PANEL.json").read_text()
    other_texts = {
        name: (HERE / name).read_text()
        for name in AUTHORITY_FILES if name != "METHOD_COMPARATOR_PANEL.json"
    }

    # copied R001 values
    hits: list[str] = []
    for name, text in [("METHOD_COMPARATOR_PANEL.json", panel_text)] + list(other_texts.items()):
        for lit in R001_FORBIDDEN_LITERALS:
            if lit in text:
                hits.append(f"{name}:{lit}")
    _check(checks, "no_copied_r001_literals", not hits, hits)

    # copied V2 torque limits (as ceilings)
    auth = load("ACTUATION_AUTHORITY.json")
    ceilings = [c["moment_ceiling_nm"] for c in auth["channels"]]
    v2 = {"lumbar": 250.0, "hip": 250.0, "knee": 300.0, "ankle": 200.0}
    copied = [c["channel"] for c in auth["channels"]
              if c["channel"].endswith("hip") and c["moment_ceiling_nm"] == v2["hip"]]
    copied += [c["channel"] for c in auth["channels"]
               if c["channel"].endswith("knee") and c["moment_ceiling_nm"] == v2["knee"]]
    copied += [c["channel"] for c in auth["channels"]
               if c["channel"].endswith("ankle") and c["moment_ceiling_nm"] == v2["ankle"]]
    copied += [c["channel"] for c in auth["channels"]
               if c["channel"] == "trunk_pelvis" and c["moment_ceiling_nm"] == v2["lumbar"]]
    _check(checks, "no_copied_v2_torque_ceilings", not copied, {"copied": copied,
                                                                 "ceilings": ceilings})

    # hidden height targets
    target_tokens = ("height_target", "performance_floor",
                     "target_band", "jump_height_target")
    panel_scan = panel_text.replace('"no_hidden_height_target": true', "")
    hidden = [f"{name}:{tok}" for name, text in other_texts.items() for tok in target_tokens
              if tok in text]
    hidden += [f"METHOD_COMPARATOR_PANEL.json:{tok}" for tok in target_tokens
               if tok in panel_scan]
    _check(checks, "no_hidden_height_targets", not hidden, hidden)
    withdrawn = [f"{name}:{lit}" for name, text in
                 [("METHOD_COMPARATOR_PANEL.json", panel_text)] + list(other_texts.items())
                 for lit in WITHDRAWN_HEIGHT_LITERALS if lit in text]
    _check(checks, "no_withdrawn_height_targets", not withdrawn, withdrawn)
    _check(checks, "functional_floor_is_declared_not_hidden",
           '"symbol": "H_ANTI_TRIVIALITY_FLOOR"' in panel_text
           and '"role": "HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY"' in panel_text
           and '"is_minimum_functional_success_condition": true' in panel_text
           and '"is_elite_performance_norm": false' in panel_text
           and '"is_optimization_target": false' in panel_text,
           {"panel_has_floor_symbol": "H_ANTI_TRIVIALITY_FLOOR" in panel_text})
    _check(checks, "floor_literal_not_in_control_authorities",
           all("0.150" not in t for t in other_texts.values()),
           [n for n, t in other_texts.items() if "0.150" in t])
    _check(checks, "no_0150_in_control_authorities",
           all("0.150" not in t for t in other_texts.values()),
           [n for n, t in other_texts.items() if "0.150" in t])

    # method mismatch
    _check(checks, "method_separation_declared",
           load("METHOD_COMPARATOR_PANEL.json")["red_team_assertions"]["method_separation_explicit"])

    # double-counted MTP energy
    mtp = load("MTP_ENERGY_AUTHORITY.json")
    _check(checks, "mtp_energy_not_double_counted",
           mtp["red_team_assertions"]["no_double_counted_mtp_energy"] is True
           and len(mtp["double_counting_prohibitions"]) == 4)

    # asymmetric nominal control
    _check(checks, "nominal_control_symmetric",
           auth["red_team_assertions"]["no_asymmetric_nominal_control"] is True
           and auth["bilateral_symmetry_rules"]["nominal_mode"] == "ENFORCED_FOR_ALL_RES85_PHASES")

    # duplicated event definitions
    phase = load("PHASE_MACHINE_AUTHORITY.json")
    _check(checks, "no_duplicated_event_definitions",
           phase["red_team_assertions"]["no_duplicated_event_definitions"] is True)

    # time-programmed primary transitions
    _check(checks, "no_time_programmed_primary_transitions",
           phase["red_team_assertions"]["no_time_programmed_primary_transitions"] is True)
    for st in phase["states"]:
        wc = st["wall_clock_role"]
        if st["state"] in ("BRAKING", "PROPULSION", "TAKEOFF_CONFIRM", "FLIGHT", "LANDING_PREP"):
            _check(checks, f"no_clock_in_{st['state']}", wc == "none", wc)

    _check(checks, "all_red_team_assertions_declared",
           all(phase["red_team_assertions"].values())
           and all(load("ACTUATION_AUTHORITY.json")["red_team_assertions"].values())
           and all(load("MTP_ENERGY_AUTHORITY.json")["red_team_assertions"].values())
           and all(load("METHOD_COMPARATOR_PANEL.json")["red_team_assertions"].values()))
    return {"status": _status(checks), "checks": checks}


# ===========================================================================
# 6. Source provenance
# ===========================================================================
def source_provenance() -> dict[str, Any]:
    plant_xml_sha = sha256_file(PLANT_XML)
    return {
        "schema_version": "1.0.0",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
        "plant": {
            "xml_path": "src/loaded_cmj/v3/assets/v3_plant.xml",
            "xml_sha256": plant_xml_sha,
            "res83_sealed_sha256": RES83_PLANT_XML_SHA256,
            "matches_res83_seal": plant_xml_sha == RES83_PLANT_XML_SHA256,
        },
        "measurement_authority": {
            "module": "src/loaded_cmj/v3/measurement.py",
            "module_sha256": sha256_file(MEASUREMENT_SRC),
            "authority_id": "LCMJ_RES84_V3_MEASUREMENT_CONTACT_AUTHORITY_V1",
            "external_bundle": RES84_BUNDLE,
            "external_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
        },
        "plant_module": {"module": "src/loaded_cmj/v3/plant.py",
                         "module_sha256": sha256_file(PLANT_SRC)},
        "constants_module": {"module": "src/loaded_cmj/v3/constants.py",
                             "module_sha256": sha256_file(CONSTANTS_SRC)},
        "authority_files": {
            name: {"sha256": sha256_file(HERE / name)} for name in AUTHORITY_FILES
        },
        "git": {
            "head_at_authoring": _git("rev-parse", "HEAD"),
            "tree_at_authoring": _git("rev-parse", "HEAD^{tree}"),
        },
        "import_boundary": {
            "plant_constants_imported_from": "loaded_cmj.v3.constants",
            "measurement_primitives_consumed_from": "loaded_cmj.v3.measurement",
            "v1_v2_controller_imports": "PROHIBITED",
        },
    }


def hash_manifest(artifacts: dict[str, Any]) -> dict[str, Any]:
    files = {}
    for name in sorted(AUTHORITY_FILES):
        files[name] = sha256_file(HERE / name)
    return {
        "schema_version": "1.0.0",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
        "achievement": "A: RES-85 scientific control authority frozen",
        "files": files,
        "plant_xml_sha256": sha256_file(PLANT_XML),
        "res84_evidence_seal_sha256": RES84_EVIDENCE_SEAL_SHA256,
        "report_sha256": {
            "AUTHORITY_CHECK_REPORT.json": sha256_text(
                json.dumps(artifacts["check_report"], indent=2, sort_keys=True) + "\n"),
        },
    }


def build_all() -> dict[str, Any]:
    reports = {
        "METHOD_COMPARATOR_PANEL": method_comparator_checks(),
        "ACTUATION_AUTHORITY": actuation_checks(),
        "MTP_ENERGY_AUTHORITY": mtp_checks(),
        "PHASE_MACHINE_AUTHORITY": phase_machine_checks(),
    }
    redteam = redteam_checks()
    check_report = {
        "schema_version": "1.0.0",
        "mission": "RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001",
        "achievement": "A: FREEZE RES-85 SCIENTIFIC CONTROL AUTHORITY",
        "status": _status(
            [{"pass": r["status"] == STATUS_PASS} for r in reports.values()]
            + [{"pass": redteam["status"] == STATUS_PASS}]
        ),
        "sub_reports": {k: {"status": v["status"],
                            "failed": [c["check"] for c in v["checks"] if not c["pass"]]}
                        for k, v in reports.items()},
        "redteam_status": redteam["status"],
        "redteam_failed": [c["check"] for c in redteam["checks"] if not c["pass"]],
    }
    return {
        "sub_reports": reports,
        "redteam": redteam,
        "check_report": check_report,
        "provenance": source_provenance(),
    }


def main(argv: list[str] | None = None) -> int:
    artifacts = build_all()
    (HERE / "AUTHORITY_CHECK_REPORT.json").write_text(
        json.dumps(artifacts["check_report"], indent=2) + "\n")
    (HERE / "SOURCE_PROVENANCE.json").write_text(
        json.dumps(artifacts["provenance"], indent=2) + "\n")
    (HERE / "HASH_MANIFEST.json").write_text(
        json.dumps(hash_manifest(artifacts), indent=2) + "\n")
    if argv and "--print" in argv:
        print(json.dumps(artifacts["check_report"], indent=2))
    if artifacts["check_report"]["status"] != STATUS_PASS:
        print("FAILED:", artifacts["check_report"]["sub_reports"],
              artifacts["check_report"]["redteam_failed"], file=sys.stderr)
        return 1
    print("RES-85 authority checks PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
