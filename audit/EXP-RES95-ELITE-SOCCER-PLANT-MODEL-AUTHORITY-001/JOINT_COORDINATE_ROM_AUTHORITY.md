# JOINT_COORDINATE_ROM_AUTHORITY — Joint Coordinate and ROM Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen world/joint coordinate conventions, structural ROM envelope and analytic forward-kinematics sign probes.

## Context

These are structural capability bounds, not technique targets. General-human anatomy, not soccer-specific technique, is authoritative for the limits.

Every joint sign has an analytic FK probe (JC-09) that RES-83 must implement as a deterministic test.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `JC-01` | World axis convention. | `{"+x": "anterior/forward", "+y": "athlete left", "+z": "up"}` | — | `PHYSICS_IDENTITY` | — |
| `JC-02` | Root joint semantics. | unconstrained planar base: root_tx, root_tz, root_ry with zero stiffness, zero damping, zero armature and no positional catch; NOT a 6-DOF free root | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `JC-03` | Trunk-pelvis sagittal hinge. | `{"semantic": "TRUNK_PELVIS_SAGITTAL_HINGE (legacy code name 'lumbar' permitted only with this semantic exposed)", "range_rad": [-0.610865, 0.610865], "range_deg": [-35.0, 35.0]}` | rad / deg | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S19 |
| `JC-04` | Hip structural ROM. | `{"range_rad": [-0.349066, 2.268928], "range_deg": [-20.0, 130.0]}` | rad / deg | `HUMAN_ANATOMY_REFERENCE` | S19 |
| `JC-05` | Knee structural ROM. | `{"range_rad": [0.0, 2.443461], "range_deg": [0.0, 140.0]}` | rad / deg | `HUMAN_ANATOMY_REFERENCE` | S19 |
| `JC-06` | Ankle structural ROM. | `{"range_rad": [-0.959931, 0.785398], "range_deg": [-55.0, 45.0]}` | rad / deg | `HUMAN_ANATOMY_REFERENCE` | S19 |
| `JC-07` | MTP structural ROM. | `{"range_rad": [-0.523599, 1.570796], "range_deg": [-30.0, 90.0]}` | rad / deg | `HUMAN_ANATOMY_REFERENCE` | S19 |
| `JC-08` | Sagittal joint sign conventions. | `{"root_ry": "+ about +y = forward whole-body pitch", "trunk_pelvis": "+ about +y = forward trunk flexion relative to pelvis", "hip": "axis -y, +q = hip flexion (knee moves anteriorly)", "knee": "axis +y, +q = knee flexion (ankle moves posteriorly relative ... | — | `PHYSICS_IDENTITY` | — |
| `JC-09` | Analytic FK sign probes (mandatory deterministic validation). | `{"root_ry_probe": "root_ry = +10 deg with all joints zero: a point initially on +z above the root moves to +x by L*sin(10 deg); COM x increases", "hip_probe": "hip = +10 deg (axis -y): knee joint x increases by L_thigh*sin(10 deg) (anterior)", "knee_probe"... | — | `PHYSICS_IDENTITY` | — |
| `JC-10` | ROM values are capability bounds, not targets. | no controller may chase a structural limit; joint-limit hits and saturated-limit dwell are telemetry warnings | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S19 |

## Sensitivity and downstream obligations

- **JC-03** — sensitivity on the reduced bound is part of the strategy-envelope study, not a calibration to exceed

## Notes and locators

- **JC-01** — Right-handed frame. (locator: system authority section 5; sources: —).
- **JC-02** — Lateral translation, roll and yaw are structurally removed; removed DOFs must never be called 'free'. (locator: system authority section 7; sources: —).
- **JC-03** — Explicit reduced-model engineering bound; must not be presented as measured lumbar ROM. (locator: aggregate reduced-model trunk-pelvis flexion-extension; not a literal lumbar-spine joint; sources: S19).
- **JC-04** — Structural capability bound, not a technique target. (locator: general-human hip flexion/extension capacity; sources: S19).
- **JC-05** — No nominal hyperextension; reverse-knee geometry is structurally impossible under the V3 coordinate authority. (locator: general-human knee flexion capacity; sources: S19).
- **JC-06** — The former +20 deg dorsiflexion cap is withdrawn as too restrictive for a structural human envelope. (locator: weight-bearing dorsiflexion ~39-45 deg; soccer jump-landing peaks ~32-38 deg; sources: S19).
- **JC-07** — Negative is plantarflexion; positive is toe dorsiflexion. (locator: general-human first-MTP dorsiflexion capacity; sources: S19).
- **JC-08** — Propulsion must reduce hip/knee flexion, drive ankle negative (plantarflexion) and permit positive MTP dorsiflexion in the forefoot/toe rocker. (locator: system authority section 5; deterministic FK derivation; sources: —).
- **JC-09** — RES-83 must implement these probes as tests; any sign disagreement is a build failure, not a tuning problem. (locator: analytic forward kinematics of the frozen topology; sources: —).

## Cross-references

- `PLANT_TOPOLOGY_AUTHORITY`
- `FOOT_MTP_MODEL_AUTHORITY`
- `COLLISION_CONTACT_POLICY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
