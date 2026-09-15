"""V3 single-source constants for the elite-soccer loaded-CMJ Plant V3.

Authority: LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1
Repository bundle: audit/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001/

Every numeric in this module is traceable to a RES-95 decision ID
(PT/AB/UB/LB/FM/JC/CC/SL/RA).  This module is Plant authority transcription
only; it contains no controller, trajectory, scorer or candidate assumption,
and it freezes no RES-84 contact calibration or RES-85 actuator limit.

V1 (src/loaded_cmj/simulation, src/loaded_cmj/assets) and V2
(src/loaded_cmj/v2) remain historical implementations and are not imported.
"""

from __future__ import annotations

from types import MappingProxyType

# ---------------------------------------------------------------------------
# Identity (PT-01)
# ---------------------------------------------------------------------------
V3_MODEL_REVISION = "loaded-cmj-model-3"
V3_MODEL_ID = "loaded-cmj-20kg-elite-soccer-v3"
V3_AUTHORITY_ID = "LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1"
V3_AUTHORITY_BUNDLE = "audit/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001"
V3_SPORT_CONTEXT_ID = "ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1"
V3_SYSTEM_AUTHORITY_ID = "LCMJ_ELITE_SOCCER_SYSTEM_AUTHORITY_V1"

# ---------------------------------------------------------------------------
# Reference athlete / system (RA-02..RA-07, AB-04)
# ---------------------------------------------------------------------------
V3_ATHLETE_STATURE_M = 1.835
V3_ATHLETE_MASS_KG = 79.0
V3_LOAD_MASS_KG = 20.0
V3_SYSTEM_MASS_KG = 99.0
V3_GRAVITY_M_S2 = 9.81
V3_GRAVITY = (0.0, 0.0, -9.81)
V3_SYSTEM_WEIGHT_N = 971.19
V3_ATHLETE_WEIGHT_N = 774.99
V3_LOAD_TO_ATHLETE_MASS_RATIO = 0.25316455696202533

# ---------------------------------------------------------------------------
# Topology (PT-02..PT-04, FM-01)
# ---------------------------------------------------------------------------
V3_COMPILED_NBODY = 14  # including world
V3_COMPILED_NJNT = 12
V3_COMPILED_NQ = 12
V3_COMPILED_NV = 12
V3_COMPILED_NU = 9
V3_COMPILED_NEQ = 0
V3_COMPILED_NA = 0
V3_COMPILED_NGEOM = 17
V3_XML_FILENAME = "v3_plant.xml"

V3_BODY_NAMES = (
    "world",
    "pelvis",
    "HAT",
    "bar",
    "left_thigh",
    "left_shank",
    "left_hindfoot",
    "left_forefoot",
    "left_toe",
    "right_thigh",
    "right_shank",
    "right_hindfoot",
    "right_forefoot",
    "right_toe",
)

V3_JOINT_NAMES = (
    "root_tx",
    "root_tz",
    "root_ry",
    "trunk_pelvis",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_mtp",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_mtp",
)
V3_ROOT_JOINT_NAMES = ("root_tx", "root_tz", "root_ry")
V3_MAJOR_JOINT_NAMES = (
    "trunk_pelvis",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_ankle",
    "right_ankle",
)
V3_MTP_JOINT_NAMES = ("left_mtp", "right_mtp")
V3_LEG_JOINT_SUFFIXES = ("hip", "knee", "ankle", "mtp")
V3_SIDES = ("left", "right")

V3_ACTUATOR_NAMES = (
    "m_trunk_pelvis",
    "m_left_hip",
    "m_right_hip",
    "m_left_knee",
    "m_right_knee",
    "m_left_ankle",
    "m_right_ankle",
    "m_left_mtp",
    "m_right_mtp",
)
# Actuation channel semantics (PT-04): ctrl is the physical net joint moment in
# N*m because every motor has gear = 1.  No torque/rate/power limit is frozen
# here; those are RES-85 authority (DF-03/DF-04).
V3_ACTUATOR_GEAR = 1.0

V3_GEOM_NAMES = (
    "floor",
    "pelvis_collision",
    "hat_torso_collision",
    "hat_head_collision",
    "bar_shaft",
    "bar_sleeve_left",
    "bar_sleeve_right",
    "left_thigh_collision",
    "left_shank_collision",
    "left_heel_support",
    "left_forefoot_support",
    "left_toe_support",
    "right_thigh_collision",
    "right_shank_collision",
    "right_heel_support",
    "right_forefoot_support",
    "right_toe_support",
)

# Plantar support geometry (FM-06): the only legal floor-support regions.
V3_PLANTAR_SUPPORT_GEOMS = (
    "left_heel_support",
    "left_forefoot_support",
    "left_toe_support",
    "right_heel_support",
    "right_forefoot_support",
    "right_toe_support",
)
V3_SUPPORT_REGIONS = ("heel", "forefoot", "toe")
V3_SUPPORT_GEOM_BY_FOOT_REGION = MappingProxyType({
    "left": MappingProxyType({
        "heel": "left_heel_support",
        "forefoot": "left_forefoot_support",
        "toe": "left_toe_support",
    }),
    "right": MappingProxyType({
        "heel": "right_heel_support",
        "forefoot": "right_forefoot_support",
        "toe": "right_toe_support",
    }),
})
# Bodies whose floor contact is prohibited/fall contact (CC-02).
V3_PROHIBITED_FLOOR_BODIES = ("pelvis", "HAT", "left_thigh", "right_thigh",
                              "left_shank", "right_shank", "bar")
V3_PROHIBITED_FLOOR_GEOMS = (
    "pelvis_collision",
    "hat_torso_collision",
    "hat_head_collision",
    "left_thigh_collision",
    "right_thigh_collision",
    "left_shank_collision",
    "right_shank_collision",
    "bar_shaft",
    "bar_sleeve_left",
    "bar_sleeve_right",
)
V3_FLOOR_GEOM = "floor"

# ---------------------------------------------------------------------------
# Geometry (AB-05/AB-06, AB-09/AB-10/AB-11, FM-05/FM-06/FM-07, SL-01)
# ---------------------------------------------------------------------------
V3_THIGH_LENGTH_M = 0.4449875           # 0.2425 * H  (AB-05)
V3_SHANK_LENGTH_M = 0.4574655           # 0.2493 * H  (AB-06)
V3_FOOT_LENGTH_M = 0.275                # AB-09
V3_FOOT_WIDTH_M = 0.105                 # AB-09
V3_ANKLE_HEIGHT_M = 0.071565            # 0.039 * H  (AB-10)
V3_ANKLE_X_FROM_HEEL_M = 0.055          # AB-11
V3_MIDTARSAL_X_FROM_HEEL_M = 0.07       # FM-07
V3_MTP_X_FROM_HEEL_M = 0.20625          # 0.75 * foot length (FM-05)
V3_TOE_TIP_X_FROM_HEEL_M = 0.275        # FM-07
V3_TOE_LENGTH_M = 0.06875               # FM-05
V3_LEG_PLANE_CENTERLINES_Y_M = (-0.085, 0.085)  # SL-01; geometry nominal only
V3_SOLE_PLANE_Z_IN_ANKLE_M = -V3_ANKLE_HEIGHT_M  # FM-13

# Foot body-chain offsets (DERIVED_QUANTITIES.foot, FM-07)
# hindfoot frame origin = ankle joint center; forefoot welded at the midtarsal
# point; toe at the MTP joint.
V3_HINDFOOT_ORIGIN_IN_ANKLE_M = (0.0, 0.0, 0.0)
V3_FOREFOOT_ORIGIN_IN_HINDFOOT_M = (
    V3_MIDTARSAL_X_FROM_HEEL_M - V3_ANKLE_X_FROM_HEEL_M,  # 0.015
    0.0,
    -V3_ANKLE_HEIGHT_M + 0.021565,                        # -0.050
)
V3_TOE_ORIGIN_IN_FOREFOOT_M = (
    V3_MTP_X_FROM_HEEL_M - V3_MIDTARSAL_X_FROM_HEEL_M,    # 0.13625
    0.0,
    0.01,
)

# Foot plantar contact patches (FM-06/FM-13).  Each patch is a box whose
# bottom face lies exactly on the sole plane z = -0.071565 in the ankle frame.
V3_CONTACT_REGIONS = MappingProxyType({
    "heel": MappingProxyType({
        "x_from_heel_start_m": 0.0,
        "x_from_heel_end_m": 0.07,
        "width_m": 0.07,
        "body": "hindfoot",
        "box_center_in_body_m": (-0.02, 0.0, -0.066565),
        "box_half_extent_m": (0.035, 0.035, 0.005),
    }),
    "forefoot": MappingProxyType({
        "x_from_heel_start_m": 0.07,
        "x_from_heel_end_m": 0.20625,
        "width_m": 0.105,
        "body": "forefoot",
        "box_center_in_body_m": (0.068125, 0.0, -0.016565),
        "box_half_extent_m": (0.068125, 0.0525, 0.005),
    }),
    "toe": MappingProxyType({
        "x_from_heel_start_m": 0.20625,
        "x_from_heel_end_m": 0.275,
        "width_m": 0.09,
        "body": "toe",
        "box_center_in_body_m": (0.034375, 0.0, -0.026565),
        "box_half_extent_m": (0.034375, 0.045, 0.005),
    }),
})

# ---------------------------------------------------------------------------
# Body inertial authority (AB-08/AB-14, UB-03..UB-06, LB-01..LB-08, FM-04/07/08)
# pos = body-frame COM, inertia about COM.
# diaginertia only for the diagonal bodies; HAT carries the full tensor.
# ---------------------------------------------------------------------------
V3_PELVIS_MASS_KG = 8.8243
V3_PELVIS_COM_M = (0.0, 0.0, 0.05966063512349223)
V3_PELVIS_DIAGINERTIA_KG_M2 = (
    0.06317964198451871,
    0.07870896370431779,
    0.07170511974256877,
)

V3_HAT_MASS_KG = 38.7969
V3_HAT_COM_M = (-0.01596456566162191, -2.8616281832444254e-18, 0.4496738763907584)
V3_HAT_FULLINERTIA_KG_M2 = (
    1.692352492111382,
    1.4753589080582343,
    0.8504250560234003,
    0.0,
    -0.02077944879714553,
    0.0,
)
V3_HAT_PRINCIPAL_MOMENTS_KG_M2 = (1.6928650336618478, 1.475358908058234, 0.8499125144729343)
V3_HAT_PRINCIPAL_ROTATION_ABOUT_Y_DEG = -1.4129592461485285

V3_BAR_MASS_KG = 20.0
V3_BAR_LENGTH_M = 2.2
V3_BAR_GRIP_DIAMETER_M = 0.028
V3_BAR_GRIP_SECTION_LENGTH_M = 1.31
V3_BAR_SLEEVE_DIAMETER_M = 0.05
V3_BAR_SLEEVE_LENGTH_M = 0.415
V3_BAR_SHAFT_LENGTH_M = 1.37
V3_BAR_RHO_EFF_KG_M3 = 8086.422350243007
V3_BAR_SHAFT_MASS_KG = 6.821547880650857
V3_BAR_SLEEVE_EACH_MASS_KG = 6.589226059674571
V3_BAR_DIAGINERTIA_KG_M2 = (
    11.75585696777048,
    0.004786777979600392,
    11.75585696777048,
)
V3_BAR_CENTER_IN_HAT_FRAME_M = (-0.095, 0.0, 0.6)
V3_BAR_SLEEVE_SPAN_M = (0.685, 1.1)

V3_THIGH_MASS_KG = 11.1864
V3_THIGH_COM_BELOW_HJC_M = 0.18222238124999998
V3_THIGH_DIAGINERTIA_KG_M2 = (
    0.23976057065071502,
    0.23976057065071502,
    0.04917660063207587,
)

V3_SHANK_MASS_KG = 3.4207
V3_SHANK_COM_BELOW_KJC_M = 0.20398386645
V3_SHANK_DIAGINERTIA_KG_M2 = (
    0.04438440231686326,
    0.04654918083021296,
    0.007594621444486416,
)

V3_HINDFOOT_MASS_KG = 0.4675536
V3_HINDFOOT_COM_M = (0.008310000000000005, 0.0, -0.027700000000000002)
V3_HINDFOOT_DIAGINERTIA_KG_M2 = (
    0.0004337502415040432,
    0.0005182470417970387,
    0.0004168508814454441,
)

V3_FOREFOOT_MASS_KG = 0.4588952
V3_FOREFOOT_COM_IN_FOREFOOT_M = (0.05708875, 0.0, 0.00419)
V3_FOREFOOT_DIAGINERTIA_KG_M2 = (
    0.0003822233613175279,
    0.0004723188679138023,
    0.0006006367106418295,
)

V3_TOE_MASS_KG = 0.1558512
V3_TOE_COM_IN_TOE_M = (0.029975, 0.0, 0.0)
V3_TOE_DIAGINERTIA_KG_M2 = (
    0.0001150951698085389,
    6.454356581420024e-05,
    0.00015255751919720058,
)

V3_TWO_FEET_MASS_KG = 2.1646
V3_WHOLE_FOOT_DELEVA_REFERENCE = MappingProxyType({
    "mass_kg": V3_TWO_FEET_MASS_KG / 2.0,
    "length_frozen_m": V3_FOOT_LENGTH_M,
    "com_from_heel_m": 0.1214125,
    "inertia_about_sagittal_axis_through_com_kg_m2": 0.005406040472937501,
    "assembled_com_x_from_heel_m": 0.11525194999999999,
    "com_x_delta_vs_deleva_m": -0.006160550000000015,
})

V3_HAT_BAR_SYSTEM = MappingProxyType({
    "mass_kg": 58.7969,
    "com_m": (-0.04284878382223176, -1.888233945369835e-18, 0.5008079067934639),
    "inertia_about_com_kg_m2": (
        (13.746432717170366, 6.454194752049098e-18, 0.1360143545930838),
        (6.454194752049098e-18, 1.8608048237640293, -2.2075708375430198e-18),
        (0.1360143545930838, -2.2075708375430198e-18, 12.688717904231572),
    ),
})
V3_SYSTEM_IYY_SENSITIVITY_RANGE_KG_M2 = (1.7316734138069028, 2.0243773264768694)

# Segment chain closure (AB-13): declared, never silently repaired.
V3_SEGMENT_CHAIN_CLOSURE = MappingProxyType({
    "hip_height_m": 0.9740179999999999,
    "midh_to_vertex_m": 0.8918879954049395,
    "sum_m": 1.8659059954049395,
    "stature_m": V3_ATHLETE_STATURE_M,
    "delta_m": 0.030905995404939546,
    "delta_fraction_of_stature": 0.016842504307868964,
})

# ---------------------------------------------------------------------------
# Joint coordinate / ROM authority (JC-02..JC-09, PT-07, FM-09)
# ---------------------------------------------------------------------------
# Axis vectors, sign semantics and structural envelope.  Values in radians
# exactly as frozen in AUTHORITY_INPUTS.json rom_rad.
V3_JOINT_AXIS = MappingProxyType({
    "root_tx": (1.0, 0.0, 0.0),
    "root_tz": (0.0, 0.0, 1.0),
    "root_ry": (0.0, 1.0, 0.0),
    "trunk_pelvis": (0.0, 1.0, 0.0),
    "left_hip": (0.0, -1.0, 0.0),
    "right_hip": (0.0, -1.0, 0.0),
    "left_knee": (0.0, 1.0, 0.0),
    "right_knee": (0.0, 1.0, 0.0),
    "left_ankle": (0.0, -1.0, 0.0),
    "right_ankle": (0.0, -1.0, 0.0),
    "left_mtp": (0.0, -1.0, 0.0),
    "right_mtp": (0.0, -1.0, 0.0),
})
V3_JOINT_RANGES_RAD = MappingProxyType({
    "root_tx": None,  # unrestricted, no catch
    "root_tz": None,
    "root_ry": None,
    "trunk_pelvis": (-0.610865, 0.610865),
    "left_hip": (-0.349066, 2.268928),
    "right_hip": (-0.349066, 2.268928),
    "left_knee": (0.0, 2.443461),
    "right_knee": (0.0, 2.443461),
    "left_ankle": (-0.959931, 0.785398),
    "right_ankle": (-0.959931, 0.785398),
    "left_mtp": (-0.523599, 1.570796),
    "right_mtp": (-0.523599, 1.570796),
})
V3_JOINT_RANGES_DEG = MappingProxyType({
    "trunk_pelvis": (-35.0, 35.0),
    "hip": (-20.0, 130.0),
    "knee": (0.0, 140.0),
    "ankle": (-55.0, 45.0),
    "mtp": (-30.0, 90.0),
})
# Major joints: zero passive stiffness/damping/armature (PT-07).
V3_MAJOR_JOINT_STIFFNESS = 0.0
V3_MAJOR_JOINT_DAMPING = 0.0
V3_MAJOR_JOINT_ARMATURE = 0.0
# MTP passive engineering prior (FM-09): neutral 0.0, k = 25 N*m/rad,
# c = 2 N*m*s/rad.  Engineering prior only; zero-passive case must remain
# reachable with the active channel present and zeroed.
V3_MTP_NEUTRAL_RAD = 0.0
V3_MTP_PASSIVE_STIFFNESS_NM_PER_RAD = 25.0
V3_MTP_PASSIVE_DAMPING_NMS_PER_RAD = 2.0
V3_MTP_PASSIVE_SENSITIVITY = MappingProxyType({
    "k_Nm_per_rad": (0.0, 12.5, 25.0, 40.0),
    "c_Nms_per_rad": (0.0, 0.4, 2.0),
})

# ---------------------------------------------------------------------------
# Collision/contact policy (CC-01..CC-13, FM-13)
# ---------------------------------------------------------------------------
# Collision class bits:
#   bit0 (1) floor; bit1 (2) legal plantar support; bit2 (4) prohibited bodies.
V3_CONTYPE_FLOOR = 1
V3_CONTYPE_SUPPORT = 2
V3_CONTYPE_PROHIBITED = 4
V3_FLOOR_CONAFFINITY = V3_CONTYPE_SUPPORT | V3_CONTYPE_PROHIBITED  # 6
V3_BODY_CONAFFINITY = 7  # floor | support | prohibited

# The complete explicit exclusion list.  Every entry is an adjacent
# anatomical parent-child pair where self-contact is impossible, or the
# intentional rigid bar/HAT attachment (CC-03).  MuJoCo additionally applies
# its default filterparent rule through welded bodies, so the effective
# matrix also filters (pelvis, bar), (shank, forefoot) and (hindfoot, toe)
# per side; those are documented in COLLISION_MATRIX.json with the same
# adjacence rationale.
V3_EXCLUDED_BODY_PAIRS = (
    ("pelvis", "HAT"),
    ("pelvis", "left_thigh"),
    ("pelvis", "right_thigh"),
    ("left_thigh", "left_shank"),
    ("right_thigh", "right_shank"),
    ("left_shank", "left_hindfoot"),
    ("right_shank", "right_hindfoot"),
    ("left_hindfoot", "left_forefoot"),
    ("right_hindfoot", "right_forefoot"),
    ("left_forefoot", "left_toe"),
    ("right_forefoot", "right_toe"),
    ("HAT", "bar"),
)

V3_PLANTAR_FLOOR_CONDIM = 4      # CC-05 nominal
V3_OTHER_CONTACT_CONDIM = 3      # CC-05 nominal
V3_SLIDING_FRICTION_NOMINAL = 0.9  # CC-06 nominal, sweep [0.5, 1.5]
V3_SLIDING_FRICTION_SENSITIVITY = (0.5, 1.5)
V3_TORSIONAL_FRICTION = 0.0      # CC-07 numeric DEFERRED to RES-84; placeholder
V3_ROLLING_FRICTION = 0.0        # CC-08 nominal for box-like plantar contacts

# ---------------------------------------------------------------------------
# Deferred / provisional numerics (DF-01..DF-12, CC-07..CC-12)
# ---------------------------------------------------------------------------
# Every non-authority numeric that exists only so the Plant can be
# instantiated or probed is registered here and is marked
# PROVISIONAL_NUMERICAL.  None of these is physiological or final authority.
V3_PROVISIONAL_NUMERICAL = MappingProxyType({
    "contact.solref": (
        "not set at V3; MuJoCo 3.8.0 compiler default applies. "
        "PROVISIONAL_NUMERICAL; final values owned by RES-84/RES-86 (CC-10)."
    ),
    "contact.solimp": (
        "not set at V3; MuJoCo 3.8.0 compiler default applies. "
        "PROVISIONAL_NUMERICAL; final values owned by RES-84/RES-86 (CC-11)."
    ),
    "contact.margin": (
        "not set at V3; MuJoCo 3.8.0 default 0.0. "
        "PROVISIONAL_NUMERICAL; exact clearance guard is RES-84 (CC-09/DF-02)."
    ),
    "contact.gap": (
        "not set at V3; MuJoCo 3.8.0 default 0.0. "
        "PROVISIONAL_NUMERICAL; declared by RES-84 (CC-09)."
    ),
    "contact.torsional_friction": (
        "0.0 placeholder (friction cone term off). "
        "PROVISIONAL_NUMERICAL; numeric value owned by RES-84 (CC-07)."
    ),
    "contact.rolling_friction": (
        "0.0 placeholder (box-like plantar contacts, CC-08 nominal). "
        "PROVISIONAL_NUMERICAL; numeric value owned by RES-84 (CC-08)."
    ),
    "contact.penetration_tolerance": (
        "not frozen at V3; RES-84/RES-86 solution verification (CC-12)."
    ),
    "solver.timestep": (
        "not set at V3; MuJoCo 3.8.0 default 0.002 s. "
        "PROVISIONAL_NUMERICAL; timestep verification owned by RES-86 (DF-08)."
    ),
    "solver.integrator": (
        "not set at V3; MuJoCo 3.8.0 default 'Euler'. "
        "PROVISIONAL_NUMERICAL; solver verification owned by RES-86 (DF-08)."
    ),
    "solver.solver": (
        "not set at V3; MuJoCo 3.8.0 defaults (Newton/dense). "
        "PROVISIONAL_NUMERICAL; owned by RES-86 (DF-08)."
    ),
    "actuator.limits": (
        "no torque/rate/power limit frozen; gear=1 net-moment topology only. "
        "RES-85 authority (DF-03/DF-04)."
    ),
    "friction.sliding": (
        "0.9 is the RES-95 nominal (CC-06), not a final calibration; "
        "mandatory sensitivity sweep [0.5, 1.5] owned downstream."
    ),
})

# RES-84 / RES-85 owned values that must NOT appear as frozen constants here.
V3_FORBIDDEN_FINAL_OWNERSHIP = MappingProxyType({
    "final_contact_solref_solimp_margin_gap": "RES-84/RES-86",
    "final_torsional_rolling_friction": "RES-84",
    "final_takeoff_clearance_guard": "RES-84",
    "final_actuator_torque_rate_power_limits": "RES-85",
    "elite_H2_target": "RES-85/RES-91",
})
