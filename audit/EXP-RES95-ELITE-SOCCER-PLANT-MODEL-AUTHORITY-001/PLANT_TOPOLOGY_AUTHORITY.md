# PLANT_TOPOLOGY_AUTHORITY — Plant Topology Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen successor Plant topology: bodies, joints, generalized dimensions, actuation channels and planar-root obligations.

## Context

This is not a 6-DOF free root. It is an unconstrained planar floating base in tx/tz/ry with structurally removed lateral, roll and yaw dynamics.

The out-of-plane reaction audit (PT-06) is fail-closed: if the omitted DOFs materially support the trajectory, V1 fails.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `PT-01` | Model revision identity. | `{"MODEL_REVISION": "loaded-cmj-model-3", "MODEL_ID": "loaded-cmj-20kg-elite-soccer-v3"}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `PT-02` | Body list including world. | `["world", "pelvis", "HAT", "bar", "left_thigh", "right_thigh", "left_shank", "right_shank", "left_hindfoot", "right_hindfoot", "left_forefoot", "right_forefoot", "left_toe", "right_toe"]` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01;S04 |
| `PT-03` | Joint topology and generalized dimensions. | `{"joints": {"root_tx": 1, "root_tz": 1, "root_ry": 1, "trunk_pelvis_sagittal_hinge": 1, "hips": 2, "knees": 2, "ankles": 2, "mtp": 2}, "NQ": 12, "NV": 12}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `PT-04` | Actuator channel topology. | `{"channels": ["trunk_pelvis_hinge", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "left_mtp", "right_mtp"], "NU": 9}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S08;S09 |
| `PT-05` | Planar-root consequence and nominal symmetry. | because lateral translation, roll and yaw are absent, V1 nominal sagittal actions must be bilaterally symmetric to numerical tolerance | — | `PHYSICS_IDENTITY` | — |
| `PT-06` | Out-of-plane reaction audit (V1 fail-closed rule). | qualification must audit out-of-plane contact/reaction quantities (Fy, roll/yaw moments where available) and demonstrate they are negligible; if omitted DOFs materially support the trajectory, V1 fails | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `PT-07` | Passive mechanics defaults. | hip/knee/ankle/trunk-pelvis passive stiffness and damping default to 0 at Plant authority level; root passives exactly 0; any numerical damping needed for stability is declared, justified, sensitivity-tested and classified as numerical regularization | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `PT-08` | Body inertial provenance (summary table). | `{"pelvis": "AB-08", "HAT": "UB-03..UB-06", "bar": "LB-07..LB-08", "thigh": "AB-14", "shank": "AB-14", "hindfoot/forefoot/toe": "FM-04, FM-07, FM-08"}` | — | `LITERATURE_DIRECT` | S01;S02;S04 |
| `PT-09` | No candidate creation in RES-95. | RES-95 freezes model authority only; candidate authorization remains RES-90 | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |

## Sensitivity and downstream obligations

- **PT-04** — final torque/rate/power limits owned by RES-85
- **PT-05** — bilateral tolerance declared and audited
- **PT-06** — mandatory audit in every qualification report
- **PT-07** — any non-zero value requires a declared justification

## Notes and locators

- **PT-02** — Expected NBODY including world = 14. (locator: world, pelvis, HAT, bar, bilateral thigh/shank/hindfoot/forefoot/toe; sources: S01;S04).
- **PT-03** — 3 root + 1 trunk + 2*4 limb DOF = 12. (locator: planar root + trunk-pelvis hinge + bilateral lower limb chain; sources: —).
- **PT-04** — The superseded 7-channel V2 topology is not successor authority. (locator: 9 physical net-moment channels incl. bilateral MTP; sources: S08;S09).
- **PT-05** — Materially asymmetric corrective control requires a future true 3-D/6-DOF Plant. (locator: system authority section 7; sources: —).
- **PT-06** — External reaction quantities are evidence, never a control channel. (locator: mission freeze item 7; system authority section 7; sources: —).
- **PT-07** — R001 major-joint passive stiffness/damping may not be carried forward as human physiology. (locator: system authority section 15; sources: S22).
- **PT-08** — No body inertia value may live only in implementation code. (locator: per-body authority mapping; sources: S01;S02;S04).
- **PT-09** — No successor candidate ID is created here. (locator: mission freeze item; Linear RES-90 boundary; sources: —).

## Cross-references

- `JOINT_COORDINATE_ROM_AUTHORITY`
- `COLLISION_CONTACT_POLICY`
- `UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY`
- `LOAD_BAR_AUTHORITY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
