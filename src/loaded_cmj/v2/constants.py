"""V2 single-source constants for LCMJ Plant V2.

Scenario: nominal 20 kg loaded CMJ, 75 kg athlete + 20 kg rigid load, sagittal-dominant.
This module is V2 authority. It does NOT import V1 constants at runtime (sources are documented in V2_ACTUATOR_AUTHORITY.md).
"""

from __future__ import annotations

from types import MappingProxyType

V2_MODEL_REVISION = "loaded-cmj-model-2"
V2_MODEL_ID = "loaded-cmj-20kg-athlete-v2"
V2_SCENARIO_ID = "loaded_jump_fixed_20kg_v2"

# Preserve V1 anthropometry
V2_ATHLETE_MASS_KG = 75.0
V2_EXTERNAL_LOAD_MASS_KG = 20.0
V2_TOTAL_MASS_KG = 95.0
V2_GRAVITY_MAGNITUDE = 9.81
V2_BODY_WEIGHT_N = V2_TOTAL_MASS_KG * V2_GRAVITY_MAGNITUDE  # 931.95 N

# Timing — keep V1 physics fidelity but expose for V2
V2_PHYSICS_TIMESTEP_S = 0.000125
V2_SUBSTEPS_PER_CONTROL = 40
V2_CONTROL_PERIOD_S = 0.005
V2_CONTROL_RATE_HZ = 200.0
V2_EPISODE_HORIZON_S = 8.0  # R3 extended to 8.0 to observe E12 for T up to 4.0 (E11 2.935 + T 4.0 +0.5 dwell +0.5 observation =7.935) with margin, timestep unchanged
V2_EPISODE_CONTROL_STEPS = 1600  # 8.0 / 0.005

# Sagittal constraint method (record exact method per spec)
V2_SAGITTAL_CONSTRAINT_METHOD = "reduced_joint_topology: pelvis has x-slide + z-slide + y-hinge; lateral translation, roll, yaw eliminated mechanically; non-sagittal hip/ankle frontal DoFs welded; task is sagittal-plane dominant but MuJoCo scene remains 3-D with bilateral feet"

# Plant inventory — V2.1
V2_COMPILED_NBODY = 10
V2_COMPILED_NJNT = 10  # 3 root + 7 sagittal hinges (lumbar, 2 hip, 2 knee, 2 ankle)
V2_COMPILED_NQ = 10
V2_COMPILED_NV = 10
V2_COMPILED_NU = 7  # 1 lumbar + 2 hip + 2 knee + 2 ankle = 7 physical; 4 effective sym commands
V2_COMPILED_NGEOM = 16  # V2 baseline 10 + 6 fall-only shells (pelvis, torso, 2 thigh, 2 shank); V2.1 honest fall
V2_COMPILED_NEQ = 0
V2_COMPILED_NA = 0

V2_MODEL_XML_FILENAME = "v2_plant.xml"

V2_BODY_NAMES = (
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
V2_ATHLETE_BODY_NAMES = tuple(n for n in V2_BODY_NAMES if n != "external_load")
V2_LOAD_BODY_NAME = "external_load"

V2_JOINT_NAMES = (
    "root_tx",
    "root_tz",
    "root_ry",
    "lumbar",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
)
V2_ROOT_JOINT_NAMES = ("root_tx", "root_tz", "root_ry")
V2_LUMBAR_JOINT = "lumbar"
V2_HIP_JOINTS = ("left_hip", "right_hip")
V2_KNEE_JOINTS = ("left_knee", "right_knee")
V2_ANKLE_JOINTS = ("left_ankle", "right_ankle")
V2_SAGITTAL_JOINTS = ("lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")

V2_MJ_ACTUATOR_NAMES = (
    "m_lumbar",
    "m_left_hip",
    "m_right_hip",
    "m_left_knee",
    "m_right_knee",
    "m_left_ankle",
    "m_right_ankle",
)

# 4 effective symmetric commands (nominal); physical still 7 but policy may pair hips/knees/ankles
V2_ACTION_CHANNELS_EFFECTIVE = ("lumbar", "hip_pair", "knee_pair", "ankle_pair")
V2_ACTION_CHANNELS_PHYSICAL = ("lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")
V2_ACTION_DIM_PHYSICAL = 7
V2_ACTION_DIM_EFFECTIVE = 4
V2_ACTION_DIM = 7  # runtime is physical 7

V2_GEOM_NAMES = (
    "floor",
    "pelvis_shell",
    "pelvis_fall",
    "torso_shell",
    "torso_fall",
    "load_shell",
    "left_thigh_shell",
    "left_thigh_fall",
    "right_thigh_shell",
    "right_thigh_fall",
    "left_shank_shell",
    "left_shank_fall",
    "right_shank_shell",
    "right_shank_fall",
    "left_foot_box",
    "right_foot_box",
)
V2_CONTACT_GEOMS = ("left_foot_box", "right_foot_box")
V2_FLOOR_GEOM_NAME = "floor"

# Physics
V2_GRAVITY = (0.0, 0.0, -9.81)
V2_SOLVER = "Newton"
V2_INTEGRATOR = "implicitfast"
V2_SOLVER_ITERATIONS = 100
V2_SOLVER_LS_ITERATIONS = 50
V2_SOLVER_TOLERANCE = 1e-10
V2_CONTACT_SOLREF = (0.016, 1.0)  # calibrated landing compliance 0.016 (RES-6), monotonic ladder T1..T6, baseline 0.004
V2_CONTACT_SOLIMP = (0.99, 0.99, 0.001, 0.5, 2.0)
V2_JOINT_SOLREFLIMIT = (0.02, 1.0)
V2_JOINT_SOLIMPLIMIT = (0.90, 0.95, 0.001, 0.5, 2.0)
V2_FRICTION_FLOOR = (0.9, 0.005, 0.0001)
V2_FRICTION_FOOT = (0.9, 0.005, 0.0001)

# Joint limits (rad or m) — V2.1 RES-31: sagittal root is honestly floating (no positional catch)
V2_JOINT_RANGES = MappingProxyType({
    "root_tx": (float("-inf"), float("inf")),  # unlimited sagittal translation
    "root_tz": (float("-inf"), float("inf")),  # unlimited vertical (already limited=false)
    "root_ry": (float("-inf"), float("inf")),  # unlimited sagittal pitch
    "lumbar": (-0.60, 0.60),
    "left_hip": (-0.50, 1.80),
    "right_hip": (-0.50, 1.80),
    "left_knee": (0.0, 2.40),  # 0 extended, flexion positive
    "right_knee": (0.0, 2.40),
    "left_ankle": (-0.70, 0.50),  # - dorsif? We'll set: negative plantar, positive dorsif
    "right_ankle": (-0.70, 0.50),
})

# Standing pose — most passively stable per exhaustive 2-s drift test (straight leg).
# hip 0, knee 0, ankle 0 gives foot flat, COM over support, root 0.90 for zero gap, drift 0.010 with Kp100 Kd20, 0.001 with Kp800.
# This is near full extension, typical standing, and allows visible countermovement to 0.8 rad knee.
V2_RESET_QPOS = (
    0.0,          # root_tx
    0.9000,       # root_tz — MEASURED 0.9000000476837158 for hip0 knee0 ankle0
    0.0,          # root_ry
    0.0,          # lumbar
    0.0,          # left_hip
    0.0,          # left_knee
    0.0,          # left_ankle
    0.0,          # right_hip
    0.0,          # right_knee
    0.0,          # right_ankle
)

# Torque limits — see V2_ACTUATOR_AUTHORITY.md for provenance; frozen before controller
# Symmetric limits (Nm) for direct motor: tau = limit * u, u in [-1,1]
V2_TORQUE_LIMITS_NM = MappingProxyType({
    "lumbar": 250.0,
    "left_hip": 250.0,
    "right_hip": 250.0,
    "left_knee": 300.0,
    "right_knee": 300.0,
    "left_ankle": 200.0,
    "right_ankle": 200.0,
})
# For effective symmetric pair, use same limit
V2_EFFECTIVE_TORQUE_LIMITS = MappingProxyType({
    "lumbar": 250.0,
    "hip_pair": 250.0,
    "knee_pair": 300.0,
    "ankle_pair": 200.0,
})

# Force-plate thresholds (V2 task)
V2_CONTACT_FZ_THRESHOLD_N = 10.0  # per foot low threshold for bilateral detection; separate from event hysteresis
V2_EVENT_THRESHOLDS = MappingProxyType({
    "SUPPORTED_START_DWELL_S": 0.10,
    "SUPPORTED_START_COM_MARGIN_M": 0.02,
    "SUPPORTED_START_TRUNK_TILT_MAX_RAD": 0.1745,
    "SUPPORTED_START_FZ_FLOOR_BW": 0.30,
    "COUNTERMOVEMENT_ONSET_VZ_MPS": -0.08,
    "COUNTERMOVEMENT_ONSET_DWELL_S": 0.03,
    "VALID_COUNTERMOVEMENT_DEPTH_M": 0.10,
    "VALID_COUNTERMOVEMENT_FORCE_FLOOR_BW": 0.30,
    "UPWARD_REVERSAL_VZ_DOWN_MPS": -0.02,
    "UPWARD_REVERSAL_VZ_UP_MPS": 0.02,
    "UPWARD_REVERSAL_DWELL_S": 0.010,
    "VERTICAL_PROPULSION_FORCE_BW": 1.05,
    "VERTICAL_PROPULSION_DWELL_S": 0.050,
    "BILATERAL_TAKEOFF_FZ_N": 10.0,
    "BILATERAL_TAKEOFF_DWELL_S": 0.010,
    "GENUINE_FLIGHT_DWELL_S": 0.08,
    "GENUINE_FLIGHT_GAP_M": 0.010,
    "APEX_DWELL_S": 0.005,
    "DESCENDING_LANDING_DWELL_S": 0.010,
    "DESCENDING_LANDING_VZ_MPS": -0.10,
    "IMPACT_ABSORPTION_DWELL_S": 0.020,
    "BALANCE_CAPTURE_DWELL_S": 0.150,
    "BALANCE_CAPTURE_COM_SPEED_MPS": 0.30,
    "STABLE_RECOVERY_DWELL_S": 0.500,
    "STABLE_RECOVERY_TILT_MAX_RAD": 0.2618,
    "TAKEOFF_VZ_MIN_MPS": 0.60,
    "TAKEOFF_VZ_PREFERRED_MPS": 0.80,
})

# --------------------------------------------------------------------------
# V2.1 True-Standing Recovery Contract (RES-16)
# Frozen empirical neighborhood derived from qualified 2.0 s standing hold.
# See 03_QUALIFIED_STANDING_TRACE.json / 05_STANDING_ENVELOPE.json
# --------------------------------------------------------------------------
V2_EVENT_CONTRACT_VERSION = "2.1-true-standing-recovery-v1"

V2_TRUE_STANDING_REFERENCE = MappingProxyType({
    "Q_STAND_REF": (-7.815675406786884e-05, 4.432221158197431e-05, 4.432221158197231e-05, -1.3226150450247243e-05, -1.3226150450247385e-05, 0.00014468478071002664, 0.00014468478071002924),
    "COM_Z_STAND_REF": 1.0748482293284303,
    "ROOT_Z_STAND_REF": 0.8999333849297984,
    "TRUNK_PITCH_STAND_REF": 0.0004001663622836646,
    "provenance": "componentwise median over [SUPPORTED_START_CONFIRMED_AT=0.100125, 2.0] (15200 physics samples, HOLD Kp400 Kd10, V2Plant, MuJoCo 3.8.0)",
})

V2_TRUE_STANDING_ENVELOPE = MappingProxyType({
    # Controlled joints: lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle
    "Q_STAND_MIN": (-0.0010165676005093132, -5.9256685312122565e-05, -5.925668531212091e-05, -0.00013928028759005342, -0.00013928028759005294, -8.127035230230237e-05, -8.127035230230364e-05),
    "Q_STAND_MAX": (0.00010766947760009481, 0.0005937610508289982, 0.0005937610508289969, 5.806894595319409e-05, 5.806894595319458e-05, 0.0013418169340970035, 0.0013418169340969999),
    # Expanded by exactly one ULP via np.nextafter for floating equality guard
    "Q_STAND_ENVELOPE": (
        (-0.0010165676005093134, 0.00010766947760009482),
        (-5.925668531212257e-05, 0.0005937610508289983),
        (-5.925668531212092e-05, 0.000593761050828997),
        (-0.00013928028759005345, 5.80689459531941e-05),
        (-0.00013928028759005296, 5.806894595319459e-05),
        (-8.127035230230239e-05, 0.0013418169340970038),
        (-8.127035230230365e-05, 0.001341816934097),
    ),
    "COM_Z_STAND_MIN": 1.0748460583846553,
    "COM_Z_STAND_MAX": 1.0748489727829738,
    "COM_Z_STAND_ENVELOPE": (1.074846058384655, 1.074848972782974),
    "ROOT_Z_STAND_MIN": 0.8999322748778121,
    "ROOT_Z_STAND_MAX": 0.8999341099256057,
    "ROOT_Z_STAND_ENVELOPE": (0.899932274877812, 0.8999341099256059),
    "TRUNK_PITCH_STAND_MIN": 0.0,
    "TRUNK_PITCH_STAND_MAX": 0.0032135278671374776,
    "TRUNK_PITCH_STAND_ENVELOPE": (-5e-324, 0.003213527867137478),
    "expansion": "np.nextafter(lower, -inf) / np.nextafter(upper, +inf)",
    "no_arbitrary_factors": True,
})

V2_OBSERVATION_FIELDS = (
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
    "plantar_normal_force_N",
    "plantar_cop_xy_m",
    "plantar_cop_valid",
    "previous_action",
    "plantar_force_world_N",
    "plantar_moment_world_origin_Nm",
    "com_position_m",
    "com_velocity_mps",
)
