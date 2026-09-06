#!/usr/bin/env python3
"""RES-52: seal the predeclared soft-contact qualification spec (before any
qualification execution of the new controller).

Frozen after Phase A/B authority capture (branch states + standing reference
exist), BEFORE any run of the new soft-contact realization layer.
"""
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import core52 as C
from tools.evid_spec import spec_sha256

W = C.WEIGHT_N

CONTROLLER_CONSTANTS = {
    "CONTROL_DT_S": 0.005,
    "PHYSICS_DT_S": 0.000125,
    "SUBSTEPS_PER_CONTROL": 40,
    "LOCAL_MODEL": "TWO_SIDED_CENTRAL_DIFFERENCE_EXACT_FORWARD_5MS",
    "RHO_PERT_U": 0.25,
    "RHO0_U": 0.25,
    "RHO_MIN_U": 0.0025,
    "RHO_SHRINK_FACTOR": 0.5,
    "MAX_TRUST_SHRINKS_PER_UPDATE": 3,
    "RHO_RECOVERY_FACTOR_PER_UPDATE": float(2.0 ** 0.25),
    "TRUST_REL": 0.10,
    "VALIDATION_TOL": {
        "FZ_WHOLE_ABS_N": 20.0,
        "FZ_L_R_ABS_N": 15.0,
        "DIST_ABS_M": 0.0005,
        "NVEL_ABS_MPS": 0.05,
        "COM_VZ_ABS_MPS": 0.01,
        "QDOT_ABS_RADPS": 0.05,
    },
    "LS_WEIGHTS": {
        "W_FZ_WHOLE": 1.0,
        "W_FZ_L": 0.5,
        "W_FZ_R": 0.5,
        "W_NVEL": 1.0,
        "W_DIST_ENGAGE": 4.0,
        "W_DIST_MAX": 4.0,
        "W_DIST_RETAIN": 2.0,
        "W_VZ": 1.0,
        "W_QDOT": 0.3,
        "W_FZ_MIN": 0.5,
        "W_REG": 1e-3,
    },
    "SCALES": {
        "FZ_SCALE_N": 100.0,
        "NVEL_SCALE_MPS": 0.05,
        "DIST_SCALE_M": 0.0005,
        "VZ_SCALE_MPS": 0.02,
        "QDOT_SCALE_RADPS": 0.3,
    },
    "PEN_MIN_RETAIN_M": 0.0005,
    "PEN_MAX_M": 0.0090,
    "PEN_SAFETY_LIMIT_M": 0.010,
    "FZ_MIN_MARGIN_N": 25.0,
    "FZ_TARGET_MIN_N": 0.0,
    "FZ_TARGET_MAX_N": 7.5 * W,
    "FZ_SPLIT_LR": "SYMMETRIC_HALF",
    "NOMINAL_ACTION": "PREVIOUS_APPLIED_ACTION (continuity walk; trust region bounds |u_k - u_{k-1}|)",
    "VZ_TARGET_MODE": "PROFILE_INTEGRAL_FROM_START",
    "NOMINAL_FALLBACK_ON_TRUST_FAIL": True,
    "QUALIFIED_NVEL_DOMAIN_MPS": [-0.35, 0.35],
    "QUALIFIED_PEN_DOMAIN_M": [0.0, 0.010],
    "QUALIFIED_NVEL_DOMAIN_MPS": [-0.35, 0.35],
    "QUALIFIED_PEN_DOMAIN_M": [0.0, 0.010],
    "CONTACT_ACTIVE_THRESHOLD_N": 10.0,
    "WALK_WINDOW_S": 0.030,
    "GATES": {
        "CONTACT_INACTIVE_FRACTION_MAX": 0.005,
        "CONTACT_MAX_INACTIVE_RUN_PHYSICS_STEPS": 2,
        "REFLIGHT_MIN_DURATION_PHYSICS_STEPS": 4,
        "POST_WALK_LOSS_EPISODES_MAX": 1,
        "CHATTER_TRANSITIONS_MAX": 8,
        "PEAK_FZ_MAX_BW": 8.0,
        "PEN_SAFETY_LIMIT_M": 0.010,
        "FZ_TRACK_BOUNDARY_RMS_MAX_BW": 0.15,
        "FZ_TRACK_BOUNDARY_MAX_ABS_BW": 0.60,
        "FZ_TRACK_LATE_RMS_MAX_BW": 0.08,
        "FZ_TRACK_LATE_MAX_ABS_BW": 0.20,
        "LATE_RIPPLE_P2P_MAX_BW": 0.60,
        "TRUST_FALLBACKS_MAX": 5,
    },
    "ACTION_SPACE_DIM": 7,
    "OLD_EXT_DIR_USED": False,
    "FORBIDDEN": ["direct qpos/qvel writes", "direct contact-force writes",
                  "direct root actuation", "contact-model changes",
                  "EXT_DIR scalar press direction", "tensile ground force",
                  "outer parameter search", "profile mutation after execution"],
}

PROFILES = [
    {
        "CELL_ID": "P0a_BODYWEIGHT_HOLD_STAND",
        "START_STATE": "S_STAND",
        "FZ_NODES_BW": [[0.0, 1.0], [0.100, 1.0]],
        "HORIZON_S": 0.100,
        "ARREST_SEGMENT_S": 0.100,
        "PURPOSE": "body-weight hold from qualified standing state",
    },
    {
        "CELL_ID": "P0b_BODYWEIGHT_HOLD_S75",
        "START_STATE": "S75",
        "FZ_NODES_BW": [[0.0, 1.0], [0.100, 1.0]],
        "HORIZON_S": 0.100,
        "ARREST_SEGMENT_S": 0.100,
        "PURPOSE": "body-weight hold from low-compression landing state",
    },
    {
        "CELL_ID": "P1a_MODERATE_S50",
        "START_STATE": "S50",
        "FZ_NODES_BW": [[0.0, 1.5], [0.075, 1.5], [0.125, 1.0]],
        "HORIZON_S": 0.125,
        "ARREST_SEGMENT_S": 0.125,
        "PURPOSE": "moderate landing support from S50 with smooth ramp to body weight",
    },
    {
        "CELL_ID": "P1b_MODERATE_S75",
        "START_STATE": "S75",
        "FZ_NODES_BW": [[0.0, 1.5], [0.075, 1.5], [0.125, 1.0]],
        "HORIZON_S": 0.125,
        "ARREST_SEGMENT_S": 0.125,
        "PURPOSE": "moderate landing support from S75 with smooth ramp to body weight",
    },
    {
        "CELL_ID": "P2_LANDING_BRAKE_S50",
        "START_STATE": "S50",
        "FZ_NODES_BW": [[0.0, 2.0], [0.050, 2.0], [0.100, 1.0]],
        "HORIZON_S": 0.100,
        "ARREST_SEGMENT_S": 0.050,
        "PURPOSE": ("landing-brake support from S50 with smooth ramp to body weight. "
                    "ARREST_SEGMENT=[0,50ms] is the momentum-arrest-relevant segment "
                    "(predeclared); the declared az>=0 ramp tail is reported separately "
                    "as profile-imposed unloading evidence."),
    },
    {
        "CELL_ID": "P3_VIABILITY_BOUNDARY_S100",
        "START_STATE": "S100",
        "FZ_NODES_BW": [[0.0, 1.0], [0.100, 1.0]],
        "HORIZON_S": 0.100,
        "ARREST_SEGMENT_S": 0.100,
        "PURPOSE": ("conservative current-state support target (clamped to 1.0 BW; "
                    "S100 measured whole Fz=745.1 N < 1.0 BW); maps the contact-viability "
                    "boundary. Failure of this cell alone does not falsify the layer."),
    },
]


def git_rev(kind: str) -> str:
    return subprocess.check_output(
        ["git", "-C", "/home/litju/Projects/loaded-cmj-control", "rev-parse", kind]
    ).decode().strip()


def build_spec(branch_authority: dict) -> dict:
    head, tree = git_rev("HEAD"), git_rev("HEAD^{tree}")
    assert head == "0c1b977f002e35ff2f713ece9ddd927fa9c68c1a"
    assert tree == "3a7b13d99fadb75b64e8eba00a77021cd6c6319d"
    return {
        "EXPERIMENT_ID": C.EXPERIMENT_ID,
        "EXPERIMENT_VERSION": "1.0.0",
        "MISSION": "RES10_SYNC_SOFT_CONTACT_FORCE_REALIZATION_AUTHORITY_001",
        "PARENT_AUTHORITY": "RES-10 / RES-52 (Linear)",
        "AUTHORITY_COMMIT_SHA": head,
        "AUTHORITY_COMMIT_TREE": tree,
        "QUALIFICATION_OR_DIAGNOSTIC": "CONTROL_QUALIFICATION",
        "HYPOTHESIS_H1": C.__doc__ or "",
        "HYPOTHESIS_H1_TEXT": (
            "There exists a bounded seven-actuator feedback realization that can maintain the "
            "current soft unilateral bilateral foot contacts and track support-force profiles in "
            "the range needed for landing momentum arrest, with explicit contact-compression / "
            "foot-normal-motion regulation and full actuator-direction allocation."),
        "HYPOTHESIS_H0_TEXT": (
            "Even with full seven-actuator deterministic local allocation and explicit soft-contact "
            "state feedback, the required support-force profiles cannot be sustained without "
            "contact loss, >10 mm penetration, >8 BW, prohibited contact, root support, or "
            "actuator violation."),
        "SCOPE_EXCLUSIONS": ["E10", "E11", "E12", "takeoff redesign", "propulsion redesign",
                             "outer candidate search"],
        "START_STATE_AUTHORITY": {
            "BRANCH_STATE_AUTHORITY_SHA256": None,  # sealed by caller
            "S_TIMES": {k: float(v) for k, v in C.S_TIMES.items()},
            "TD_TIME": C.TD_TIME,
            "STAND_CAPTURE_TIME": C.STAND_CAPTURE_TIME,
        },
        "CONTROLLER_CONSTANTS": CONTROLLER_CONSTANTS,
        "PROFILES": PROFILES,
        "PROFILE_DERIVATION": {
            "BODY_WEIGHT_N": W,
            "S_STAND_REFERENCE": "STANDING_CONTACT_REFERENCE.json (observed, not invented)",
            "RES51_REQUIRED_IMPEDANCE_ENVELOPE": (
                "RES-51 POST_TOUCHDOWN_FORCE_FEASIBILITY: required remaining arrest support "
                "1.69-3.06 BW over TD+50..150 ms; P1/P2 use 1.5-2.0 BW inside that envelope"),
            "P3_CONSERVATIVE_TARGET_RULE": (
                "Fz_des = clamp(measured whole Fz at start, 1.0 BW, 2.0 BW)"),
        },
        "CONTROL_SAMPLE_CONVENTION": "SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL",
        "SEARCH_METHOD": "NONE (fixed predeclared qualification matrix; no candidate search)",
        "BUDGET_DEFINITION": {
            "QUALIFICATION_PROFILE_RUNS": len(PROFILES),
            "MAX_TRUST_RESOLVES_PER_UPDATE": CONTROLLER_CONSTANTS["MAX_TRUST_SHRINKS_PER_UPDATE"],
            "BRANCH_PROBES_PER_CONTROL_UPDATE": 16,
            "MAX_RANDOM_OR_HEURISTIC_SEARCH_EVALUATIONS": 0,
        },
        "PHYSICS_DT": 0.000125,
        "CONTROL_DT": 0.005,
        "SUBSTEPS_PER_CONTROL": 40,
        "FROZEN_VARIABLES": [
            "Plant/model_XML/masses/inertias", "root_topology_and_zero_root_damping",
            "no_root_limits", "contact_geometry_parameters_friction_solref_solimp",
            "MuJoCo_3.8.0", "implicitfast_integrator_and_solver_configuration",
            "physics_dt_0.000125", "control_dt_0.005", "substeps_per_control_40",
            "actuator_model_and_limits", "synchronized_measurement_authority",
            "event_DAG_thresholds_scorer", "RES43_standing_contract",
            "all_E1_E8_controller_behavior", "C00_seed_branch_prefix",
        ],
        "SPEC_EXECUTION_MATCH": "PASS required",
        "STOPPING_RULE": (
            "run every declared profile cell exactly once; failures are retained as evidence; "
            "no retrospective modification of profiles, weights, tolerances, or trust region"),
        "FAILURE_CLASSES": {
            "A": "required force outside feasible envelope at the start state",
            "B": "required impulse cannot be accumulated before penetration/contact/configuration limits",
            "C": "realization cannot track a feasible planned force (controller/numerical)",
            "D": "profile tracked but a hard gate (safety/honesty) violated",
        },
        "EXPECTED_OUTPUTS": [
            "BRANCH_STATE_AUTHORITY.json", "STANDING_CONTACT_REFERENCE.json",
            "SOFT_CONTACT_STATE_CONTRACT.md", "LOCAL_ACTION_EFFECTIVENESS_METHOD.md",
            "per-profile physics-rate traces", "local-model validation traces",
            "action-effectiveness matrices", "CONTACT_VIABILITY_MAP.json",
            "FORCE_PROFILE_QUALIFICATION.json", "claim_evidence.csv",
            "tests/reviews", "reproduction.json/reproduce.sh", "checksums.sha256",
            "FINAL_RECEIPT.md",
        ],
        "SPEC_CREATED_AT": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def main() -> None:
    auth = json.loads((C.BUNDLE / "BRANCH_STATE_AUTHORITY.json").read_text())
    spec = build_spec(auth)
    spec["START_STATE_AUTHORITY"]["BRANCH_STATE_AUTHORITY_SHA256"] = C.sha_bytes(
        C.BUNDLE / "BRANCH_STATE_AUTHORITY.json")
    sealed = dict(spec)
    sealed["EXPERIMENT_SPEC_SHA256"] = spec_sha256(
        {k: v for k, v in spec.items() if k != "EXPERIMENT_SPEC_SHA256"})
    C.BUNDLE.mkdir(parents=True, exist_ok=True)
    (C.BUNDLE / "experiment_spec.json").write_text(
        json.dumps(sealed, indent=2, sort_keys=True) + "\n")
    print("[spec] sealed:", sealed["EXPERIMENT_ID"])
    print("[spec] SHA256:", sealed["EXPERIMENT_SPEC_SHA256"])


if __name__ == "__main__":
    main()
