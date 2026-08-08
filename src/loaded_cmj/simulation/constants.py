"""Single source of every public constant for Loaded CMJ Control.

model: loaded-cmj-model-1
scenario: ``loaded_jump_fixed_20kg`` -- 75.0 kg athlete + 20.0 kg rigid load.

Authoritative values are declared here.  Registry projections are derived only
from those local declarations; nothing is read from the environment or a
file, and nothing depends on import order.  This module is PUBLIC: it ships
read-only by the simulation and policy-contract layers.

This module must not import MuJoCo or runtime code.
"""

from __future__ import annotations

from types import MappingProxyType

MODEL_REVISION = "loaded-cmj-model-1"
MODEL_ID = "loaded-cmj-20kg-athlete"

# --------------------------------------------------------------------------
# Fixed scenario  (FIXED_SCENARIO_CONTRACT.json)
# --------------------------------------------------------------------------
SCENARIO_ID = "loaded_jump_fixed_20kg"

ATHLETE_MASS_KG = 75.0
EXTERNAL_LOAD_MASS_KG = 20.0
TOTAL_MASS_KG = 95.0

GRAVITY_MPS2 = (0.0, 0.0, -9.81)
GRAVITY_MAGNITUDE = 9.81
BODY_WEIGHT_N = 931.95  # TOTAL_MASS_KG * 9.81

MASS_CLOSURE_TOLERANCE_KG = 1e-9
COM_CLOSURE_TOLERANCE_M = 1e-9
INERTIA_CLOSURE_TOLERANCE_REL = 1e-9

OBSERVATION_NOISE = "zero"
ARTIFICIAL_DELAY = "zero"
RANDOMIZATION = "none"
HIDDEN_PHYSICAL_VARIATION = "none"
DETERMINISTIC_SEED = 0
SEED_IS_CONSUMED = False

# --------------------------------------------------------------------------
# Timing  (FIXED_SCENARIO_CONTRACT.json section timing)
# --------------------------------------------------------------------------
PHYSICS_TIMESTEP_S = 0.000125
SUBSTEPS_PER_CONTROL = 40
CONTROL_PERIOD_S = 0.005
CONTROL_RATE_HZ = 200.0
FIXED_HOLD_DURATION_S = 0.300
FIXED_HOLD_SUBSTEPS = 2400
EPISODE_HORIZON_S = 4.000
EPISODE_CONTROL_STEPS = 800
EPISODE_PHYSICS_SUBSTEPS = 32000
TOTAL_SIMULATED_S = 4.300

# --------------------------------------------------------------------------
# Plant inventory  (PLANT_TOPOLOGY_CONTRACT.md)
# --------------------------------------------------------------------------
COMPILED_NQ = 25
COMPILED_NV = 21
COMPILED_NU = 15
COMPILED_NBODY = 10
COMPILED_NJNT = 10
COMPILED_NGEOM = 16
COMPILED_NEQ = 0
COMPILED_NA = 0

MODEL_XML_FILENAME = "loaded_jump_athlete.xml"

BODY_NAMES = (
    "pelvis",
    "torso_head_arms",
    "external_load",
    "left_thigh",
    "left_shank",
    "left_foot",
    "right_thigh",
    "right_shank",
    "right_foot",
)
ATHLETE_BODY_NAMES = tuple(n for n in BODY_NAMES if n != "external_load")
LOAD_BODY_NAME = "external_load"

JOINT_NAMES = (
    "root",
    "lumbar",
    "left_hip",
    "left_knee_flexion",
    "left_ankle_dorsiflexion",
    "left_ankle_eversion",
    "right_hip",
    "right_knee_flexion",
    "right_ankle_dorsiflexion",
    "right_ankle_eversion",
)
BALL_JOINT_NAMES = ("lumbar", "left_hip", "right_hip")
HINGE_JOINT_NAMES = (
    "left_knee_flexion",
    "left_ankle_dorsiflexion",
    "left_ankle_eversion",
    "right_knee_flexion",
    "right_ankle_dorsiflexion",
    "right_ankle_eversion",
)
ROOT_JOINT_NAME = "root"

MJ_ACTUATOR_NAMES = (
    "mj_lumbar_tx",
    "mj_lumbar_ty",
    "mj_lumbar_tz",
    "mj_left_hip_tx",
    "mj_left_hip_ty",
    "mj_left_hip_tz",
    "mj_right_hip_tx",
    "mj_right_hip_ty",
    "mj_right_hip_tz",
    "mj_left_knee_flexion",
    "mj_right_knee_flexion",
    "mj_left_ankle_dorsiflexion",
    "mj_right_ankle_dorsiflexion",
    "mj_left_ankle_eversion",
    "mj_right_ankle_eversion",
)

# --------------------------------------------------------------------------
# The canonical public action channel order  (ACTION_CONTRACT.json)
#
# There is exactly ONE order in the entire system.  It indexes the public
# action u, the drive state z, the anatomical coordinates s and rates s_dot,
# the anatomical torque tau_eta, and the observation fields
# joint_position_rad / joint_velocity_radps / previous_action.
# --------------------------------------------------------------------------
ACTION_CHANNELS = (
    "lumbar_flexion",
    "lumbar_lateral",
    "lumbar_axial",
    "left_hip_flexion",
    "left_hip_abduction",
    "left_hip_rotation",
    "right_hip_flexion",
    "right_hip_abduction",
    "right_hip_rotation",
    "left_knee_flexion",
    "right_knee_flexion",
    "left_ankle_dorsiflexion",
    "right_ankle_dorsiflexion",
    "left_ankle_eversion",
    "right_ankle_eversion",
)
ACTION_DIM = 15
ACTION_DTYPE = "float64"
ACTION_MINIMUM = -1.0
ACTION_MAXIMUM = 1.0
ACTION_BOUNDS_BEHAVIOR = "reject"
ACTION_MAX_SERIALIZED_BYTES = 4096
NEUTRAL_ACTION = (0.0,) * 15

# Public fixed-hold reset protocol (CG-001-FIXED-HOLD-RESET).  Public zero
# remains the neutral participant action; this distinct action is used only
# during the unscored reset dwell and ends before participant control begins.
HOLD_ACTION = (
    -5.778611745277828e-05, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    0.3109199948701661, 0.3109199948701661,
    0.3665532076673518, 0.3665532076673518, 0.0, 0.0,
)
HOLD_Z_INITIAL = HOLD_ACTION
HOLD_TAU_PREV_INITIAL = (
    -0.01712438707638823, 0.0, 0.0,
    -4.199191836050696, 0.0, 0.0,
    -4.199191836050696, 0.0, 0.0,
    34.82350235487921, 34.82350235487921,
    18.327660362769333, 18.327660362769333, 0.0, 0.0,
)

# Channel index -> ball joint name, for the nine ball channels; None for hinges.
CHANNEL_BALL_JOINT = (
    "lumbar", "lumbar", "lumbar",
    "left_hip", "left_hip", "left_hip",
    "right_hip", "right_hip", "right_hip",
    None, None, None, None, None, None,
)
# Channel index -> hinge joint name for the six hinge channels; None for balls.
CHANNEL_HINGE_JOINT = (
    None, None, None, None, None, None, None, None, None,
    "left_knee_flexion",
    "right_knee_flexion",
    "left_ankle_dorsiflexion",
    "right_ankle_dorsiflexion",
    "left_ankle_eversion",
    "right_ankle_eversion",
)
# Ball joint name -> its three channel indices, in anatomical axis order.
BALL_JOINT_CHANNELS = MappingProxyType(
    {"lumbar": (0, 1, 2), "left_hip": (3, 4, 5), "right_hip": (6, 7, 8)}
)
# Ball joint name -> its three MuJoCo tangent actuator names, same axis order.
BALL_JOINT_MJ_ACTUATORS = MappingProxyType(
    {
        "lumbar": ("mj_lumbar_tx", "mj_lumbar_ty", "mj_lumbar_tz"),
        "left_hip": ("mj_left_hip_tx", "mj_left_hip_ty", "mj_left_hip_tz"),
        "right_hip": ("mj_right_hip_tx", "mj_right_hip_ty", "mj_right_hip_tz"),
    }
)
# Hinge channel index -> the MuJoCo actuator that carries it (identity map).
HINGE_CHANNEL_MJ_ACTUATOR = MappingProxyType(
    {
        9: "mj_left_knee_flexion",
        10: "mj_right_knee_flexion",
        11: "mj_left_ankle_dorsiflexion",
        12: "mj_right_ankle_dorsiflexion",
        13: "mj_left_ankle_eversion",
        14: "mj_right_ankle_eversion",
    }
)

# --------------------------------------------------------------------------
# Geometry and contact  (PHYSICAL_PARAMETER_CONTRACT.json)
# --------------------------------------------------------------------------
FLOOR_GEOM_NAME = "floor"
LEFT_PAD_GEOM_NAMES = (
    "left_pad_heel_medial",
    "left_pad_heel_lateral",
    "left_pad_fore_medial",
    "left_pad_fore_lateral",
)
RIGHT_PAD_GEOM_NAMES = (
    "right_pad_heel_medial",
    "right_pad_heel_lateral",
    "right_pad_fore_medial",
    "right_pad_fore_lateral",
)
ADMISSIBLE_CONTACT_GEOMS = LEFT_PAD_GEOM_NAMES + RIGHT_PAD_GEOM_NAMES
SHELL_GEOM_NAMES = (
    "pelvis_shell",
    "torso_shell",
    "load_shell",
    "left_thigh_shell",
    "right_thigh_shell",
    "left_shank_shell",
    "right_shank_shell",
)
PAD_SPHERE_RADIUS_M = 0.022
GEOM_DISTANCE_CUTOFF_M = 0.05

FOOT_NAMES = ("left_foot", "right_foot")

STATURE_REFERENCE_M = 1.75
SEGMENT_GEOMETRY_M = MappingProxyType(
    {
        "ankle_height": 0.070,
        "shank_length": 0.430,
        "thigh_length": 0.430,
        "hip_half_width": 0.095,
        "lumbar_height_above_pelvis_origin": 0.100,
        "pelvis_origin_height_at_full_extension": 0.930,
        "foot_length": 0.260,
        "foot_width": 0.095,
        "foot_thickness": 0.070,
        "ankle_to_heel_x": -0.070,
        "ankle_to_toe_x": 0.190,
    }
)

FLOOR_FRICTION = (0.9, 0.005, 0.0001)
FOOT_PAD_FRICTION = (0.9, 0.005, 0.0001)
SHELL_FRICTION = (0.5, 0.005, 0.0001)
CONTROLLER_FRICTION_MARGIN = 0.15
CONTROLLER_EFFECTIVE_MU = 0.75

# Numerical profile (CONTACT_AND_NUMERICAL_CONTRACT.md section 2)
PHYSICS_GRID_S = (0.00025, 0.000125, 0.0000625, 0.00003125)
NUMERICAL_SELECTION_RULE = "LEAST_EXPENSIVE_ALL_PASSING"
SOLVER = "Newton"
INTEGRATOR = "implicitfast"
SOLVER_ITERATIONS = 100
SOLVER_LS_ITERATIONS = 50
SOLVER_TOLERANCE = 1e-10
SOLVER_LS_TOLERANCE = 0.01
FRICTION_CONE = "elliptic"
IMPRATIO = 10.0
JACOBIAN = "dense"
GEOM_SOLREF = (0.004, 1.0)
GEOM_SOLIMP = (0.99, 0.99, 0.001, 0.5, 2.0)
JOINT_SOLREFLIMIT = (0.010, 1.0)
JOINT_SOLIMPLIMIT = (0.90, 0.98, 0.010, 0.5, 2.0)
CONDIM = 3

# --------------------------------------------------------------------------
# Joint domains  (PHYSICAL_PARAMETER_CONTRACT.json joint_axes_and_limits)
# --------------------------------------------------------------------------
HINGE_RANGES_RAD = MappingProxyType(
    {
        "left_knee_flexion": (-0.0873, 2.4435),
        "right_knee_flexion": (-0.0873, 2.4435),
        "left_ankle_dorsiflexion": (-0.8727, 0.5236),
        "right_ankle_dorsiflexion": (-0.8727, 0.5236),
        "left_ankle_eversion": (-0.3491, 0.2618),
        "right_ankle_eversion": (-0.3491, 0.2618),
    }
)
HINGE_AXES_CHILD_FRAME = MappingProxyType(
    {
        "left_knee_flexion": (0.0, -1.0, 0.0),
        "right_knee_flexion": (0.0, -1.0, 0.0),
        "left_ankle_dorsiflexion": (0.0, 1.0, 0.0),
        "right_ankle_dorsiflexion": (0.0, 1.0, 0.0),
        "left_ankle_eversion": (1.0, 0.0, 0.0),
        "right_ankle_eversion": (1.0, 0.0, 0.0),
    }
)
# Anatomical admissible box on eta = A^T Log(R), per ball joint, axis order 1,2,3.
BALL_ADMISSIBLE_BOX_RAD = MappingProxyType(
    {
        "lumbar": ((-0.6981, 0.6981), (-0.5236, 0.5236), (-0.6109, 0.6109)),
        "left_hip": ((-0.5236, 2.0944), (-0.4363, 0.7854), (-0.7854, 0.6981)),
        "right_hip": ((-0.5236, 2.0944), (-0.4363, 0.7854), (-0.7854, 0.6981)),
    }
)
BALL_CONE_LIMIT_RAD = MappingProxyType(
    {"lumbar": 1.1000, "left_hip": 2.4000, "right_hip": 2.4000}
)
BALL_BOX_CORNER_NORM_RAD = MappingProxyType(
    {"lumbar": 1.0652, "left_hip": 2.3707, "right_hip": 2.3707}
)

ACTUATOR_RANGE_NM = MappingProxyType(
    {
        "mj_lumbar_tx": (-410.0, 410.0),
        "mj_lumbar_ty": (-410.0, 410.0),
        "mj_lumbar_tz": (-410.0, 410.0),
        "mj_left_hip_tx": (-450.0, 450.0),
        "mj_left_hip_ty": (-450.0, 450.0),
        "mj_left_hip_tz": (-450.0, 450.0),
        "mj_right_hip_tx": (-450.0, 450.0),
        "mj_right_hip_ty": (-450.0, 450.0),
        "mj_right_hip_tz": (-450.0, 450.0),
        "mj_left_knee_flexion": (-300.0, 140.0),
        "mj_right_knee_flexion": (-300.0, 140.0),
        "mj_left_ankle_dorsiflexion": (-220.0, 50.0),
        "mj_right_ankle_dorsiflexion": (-220.0, 50.0),
        "mj_left_ankle_eversion": (-40.0, 40.0),
        "mj_right_ankle_eversion": (-40.0, 40.0),
    }
)

# --------------------------------------------------------------------------
# Reset parameters
# --------------------------------------------------------------------------
RESET_ID = "loaded-cmj-fixed-hold-1"
# These values are fixed plant parameters. The hip reference is selected so
# the neutral foot pitch and the eight-pad support geometry are coincident.
#
#   hip eta_1: declared 0.300000 -> 0.209440 rad (delta 0.090560, allowed +/-0.10)
#     The declared angles leave a residual world foot pitch of
#     hip - knee + ankle = 0.300000 - 0.296706 + 0.087266 = 0.090560 rad, so the
#     fore pads sit 0.019 m below the heel pads and R1 (|gap| <= 0.0005 m on all
#     eight pads) is unsatisfiable at ANY pelvis height.  Setting the hip to
#     knee - ankle makes the net foot pitch exactly zero.  The hip is the only
#     one of the three angles whose spring reference is unaffected: a ball
#     joint's qpos_spring is structurally the identity quaternion, so moving the
#     hip cannot break the "springref equals the reset pose" invariant that the
#     knee and ankle carry (DEC-IC-PAS-003).
#
#   pelvis z: declared 0.891000 -> 0.918967162046 m
#     MEASURED_DURING_IMPLEMENTATION per RESET_CONTRACT.md section 2: the exact
#     height that puts every pad at gap 0.000 m, by binary search on qpos[2]
#     after the model compiles.  Residual max |gap| = 6.2e-17 m.
RESET_QPOS = (
    0.000000, 0.000000, 0.918967162046,          # pelvis xyz  (MEASURED)
    1.0, 0.0, 0.0, 0.0,                          # pelvis quat
    1.0, 0.0, 0.0, 0.0,                          # lumbar quat
    0.994522, 0.0, 0.104529, 0.0,                # left_hip quat  (eta_1 = 0.209440)
    0.296706,                                    # left knee
    0.087266,                                    # left ankle dorsiflexion
    0.000000,                                    # left ankle eversion
    0.994522, 0.0, 0.104529, 0.0,                # right_hip quat (eta_1 = 0.209440)
    0.296706,                                    # right knee
    0.087266,                                    # right ankle dorsiflexion
    0.000000,                                    # right ankle eversion
)

# --------------------------------------------------------------------------
# Trusted contact latch  (CONTACT_AND_NUMERICAL_CONTRACT.md section 3)
# --------------------------------------------------------------------------
CONTACT_F_ON_N = 2.0
CONTACT_F_OFF_N = 0.5
CONTACT_F_ACTIVE_N = 0.5
CONTACT_F_COP_MIN_N = 20.0
CONTACT_N_ON = 2
CONTACT_N_OFF = 4
CONTACT_T_ON_S = CONTACT_N_ON * PHYSICS_TIMESTEP_S
CONTACT_T_OFF_S = CONTACT_N_OFF * PHYSICS_TIMESTEP_S
CONTACT_GAP_MAX_M = 0.002

# --------------------------------------------------------------------------
# Canonical event registry (EVENT_ENGINE_CONTRACT.md section 4)
#
# The rows below are the only event vocabulary and threshold authority.  The
# flat EVENT_THRESHOLDS view is derived from these rows for public consumers;
# EVENT must not declare a second threshold table.
# --------------------------------------------------------------------------
EVENT_REGISTRY = (
    MappingProxyType({
        "id": "E1", "name": "supported_start",
        "thresholds": MappingProxyType({
            "E1_speed_max_mps": 0.05,
            "E1_com_margin_m": 0.020,
            "E1_trunk_tilt_max_rad": 0.1745,
            "E1_dwell_s": 0.100,
            "E1_timeout_s": 0.150,
            "E1_force_floor_bw": 0.30,
        }),
    }),
    MappingProxyType({
        "id": "E2", "name": "countermovement_onset",
        "thresholds": MappingProxyType({
            "E2_vz_on_mps": -0.080,
            "E2_vz_off_mps": -0.030,
            "E2_force_floor_bw": 0.30,
            "E2_dwell_s": 0.030,
            "E2_debounce_s": CONTACT_T_ON_S,
        }),
    }),
    MappingProxyType({
        "id": "E3", "name": "valid_countermovement",
        "thresholds": MappingProxyType({
            "E3_depth_m": 0.100,
            "E3_force_floor_bw": 0.30,
            "E3_max_descent_speed_mps": 2.5,
            "E3_dwell_s": 0.0005,
            "E3_minimum_descent_s": 0.20,
        }),
    }),
    MappingProxyType({
        "id": "E4", "name": "upward_reversal",
        "thresholds": MappingProxyType({
            "E4_vz_down_mps": -0.020,
            "E4_vz_up_mps": 0.020,
            "E4_dwell_s": 0.010,
        }),
    }),
    MappingProxyType({
        "id": "E5", "name": "positive_propulsion",
        "thresholds": MappingProxyType({
            "E5_force_bw": 1.05,
            "E5_dwell_s": 0.050,
            "E5_horizontal_impulse_ratio_max": 0.15,
            "E5_propulsion_asymmetry_max": 0.20,
            "E5_impulse_residual_relative_max": 0.005,
            "E5_mechanics_residual_trans_max_N": 1.0e-6,
            "E5_mechanics_residual_rot_max_Nm": 1.0e-6,
        }),
    }),
    MappingProxyType({
        "id": "E6", "name": "valid_takeoff",
        "thresholds": MappingProxyType({
            "E6_dwell_s": 0.010,
            "E6_takeoff_vz_min_mps": 0.20,
            "E6_release_skew_max_s": 0.020,
        }),
    }),
    MappingProxyType({
        "id": "E7", "name": "contact_free_flight",
        "thresholds": MappingProxyType({
            "E7_gap_min_m": 0.010,
            "E7_dwell_s": 0.080,
            "E7_sole_separation_min_m": 0.010,
            "E7_ballistic_position_residual_max_m": 0.003,
            "E7_ballistic_velocity_residual_max_mps": 0.020,
            "E7_trunk_tilt_max_rad": 0.5235987755982988,
        }),
    }),
    MappingProxyType({
        "id": "E8", "name": "apex",
        "thresholds": MappingProxyType({"E8_dwell_s": 0.005}),
    }),
    MappingProxyType({
        "id": "E9", "name": "valid_landing",
        "thresholds": MappingProxyType({
            "E9_dwell_s": 0.010,
            "E9_landing_descent_min_mps": 0.10,
            "E9_landing_skew_max_s": 0.025,
        }),
    }),
    MappingProxyType({
        "id": "E10", "name": "impact_absorption",
        "thresholds": MappingProxyType({
            "E10_vz_arrest_mps": -0.050,
            "E10_dwell_s": 0.020,
            "E10_timeout_after_recontact_s": 0.600,
            "E10_absorption_window_max_s": 0.30,
            "E10_rebound_max_mps": 0.08,
            "E10_landing_peak_force_multiple": 8.0,
            "E10_limit_work_max_J": 1.0e-5,
        }),
    }),
    MappingProxyType({
        "id": "E11", "name": "captured_supported_state",
        "thresholds": MappingProxyType({
            "E11_com_speed_max_mps": 0.300,
            "E11_qvel_norm_max": 3.0,
            "E11_com_margin_m": 0.010,
            "E11_trunk_tilt_max_rad": 0.5236,
            "E11_joint_limit_margin_rad": 0.02,
            "E11_dwell_s": 0.150,
            "E11_foot_load_fraction": 0.25,
            "E11_joint_rate_max_radps": 0.15,
            "E11_angular_speed_max_radps": 0.15,
            "E11_centroidal_h_max_kgm2ps": 1.0,
            "E11_rebound_max_mps": 0.02,
        }),
    }),
    MappingProxyType({
        "id": "E12", "name": "bounded_recovery",
        "thresholds": MappingProxyType({
            "E12_height_ratio": 0.90,
            "E12_trunk_tilt_max_rad": 0.2618,
            "E12_qvel_norm_max": 1.0,
            "E12_dwell_s": 0.500,
        }),
    }),
    MappingProxyType({"id": "E13", "name": "completion", "thresholds": MappingProxyType({})}),
)

EVENT_IDS = tuple(row["id"] for row in EVENT_REGISTRY)
EVENT_NAMES = tuple(row["name"] for row in EVENT_REGISTRY)
EVENT_THRESHOLDS = MappingProxyType(
    {
        key: value
        for row in EVENT_REGISTRY
        for key, value in row["thresholds"].items()
    }
)
TERMINATION_THRESHOLDS = MappingProxyType(
    {
        "FALL_height_ratio": 0.45,
        "FALL_trunk_tilt_rad": 1.222,
        "FALL_dwell_s": 0.010,
    }
)
DETECTED_EVENT_COUNT = len(EVENT_REGISTRY)
REQUIRED_EVENT_COUNT = 13
BOOKKEEPING_EVENTS = ("apex",)

# Highest trusted stage k (0..10) reached, keyed by the event that establishes it.
STAGE_BY_EVENT = MappingProxyType(
    {
        "supported_start": 1,
        "valid_countermovement": 2,
        "upward_reversal": 3,
        "positive_propulsion": 4,
        "valid_takeoff": 5,
        "contact_free_flight": 6,
        "valid_landing": 7,
        "impact_absorption": 8,
        "captured_supported_state": 9,
        "bounded_recovery": 10,
    }
)

TERMINATION_CLASSES = (
    "OBJECTIVE_COMPLETE",
    "INCOMPLETE_HORIZON",
    "PHYSICAL_FALL",
    "PHYSICS_NONFINITE_FAULT",
    "AGENT_FAULT",
    "INTERNAL_EVALUATION_ERROR",
)

# --------------------------------------------------------------------------
# Public observation  (OBSERVATION_CONTRACT.json)
# --------------------------------------------------------------------------
OBSERVATION_FIELDS = (
    "time_s",
    "step_index",
    "control_dt_s",
    "episode_reset",
    "joint_position_rad",
    "joint_velocity_radps",
    "pelvis_position_world_m",
    "pelvis_orientation_world_quat_wxyz",
    "pelvis_linear_velocity_world_mps",
    "pelvis_angular_velocity_body_radps",
    "trunk_load_orientation_world_quat_wxyz",
    "trunk_load_angular_velocity_body_radps",
    "plantar_normal_force_N",
    "plantar_cop_xy_m",
    "plantar_cop_valid",
    "previous_action",
)
OBSERVATION_FIELD_COUNT = 16
OBSERVATION_MAX_SERIALIZED_BYTES = 65536
PUBLIC_COP_FORCE_FLOOR_N = 20.0
POLICY_PROTOCOL_VERSION = 2
POLICY_ENTRYPOINT = "act"

# --------------------------------------------------------------------------
# PolicyWorker limits  (POLICY_ISOLATION_CONTRACT.md section 5)
# --------------------------------------------------------------------------
WORKER_STEP_TIMEOUT_S = 0.020
WORKER_FIRST_CALL_TIMEOUT_S = 30.000
WORKER_MAX_REQUEST_BYTES = 262_144
WORKER_MAX_RESPONSE_BYTES = 65_536
WORKER_MAX_STDERR_CHARS = 8_000
WORKER_MAX_ADDRESS_SPACE_BYTES = 2_147_483_648
WORKER_MAX_PROCESSES = 64
WORKER_MAX_CPU_SECONDS = 120
WORKER_MAX_OPEN_FILES = 256

AGENT_FAULT_REASONS = (
    "missing_policy_artifact",
    "policy_not_regular_file",
    "policy_import_error",
    "policy_no_entrypoint",
    "policy_exception",
    "policy_timeout",
    "policy_exited",
    "policy_protocol_error",
    "invalid_action_shape",
    "invalid_action_dtype",
    "invalid_action_nonfinite",
    "invalid_action_out_of_bounds",
    "policy_resource_violation",
    "hidden_data_access",
)

__all__ = [name for name in dir() if not name.startswith("_")]
