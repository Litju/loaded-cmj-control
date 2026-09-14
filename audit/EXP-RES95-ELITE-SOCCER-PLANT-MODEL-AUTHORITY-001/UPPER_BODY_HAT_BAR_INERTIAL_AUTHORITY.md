# UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY — Reduced Upper-Body (HAT) and Bar Inertial Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen reduced HAT decomposition, fixed bar-hold representation, and the resulting numeric HAT and HAT+bar inertial properties.

## Context

The reduced HAT is a single rigid body. Its inertial properties are computed from de Leva component segments under one frozen engineering bar-hold posture with the parallel-axis theorem and full 3x3 tensors.

Every unmeasured posture or placement parameter is ENGINEERING_NOMINAL_WITH_SENSITIVITY; none is presented as anthropometric truth. Sensitivity cases are computed in DERIVED_QUANTITIES.json (hat_sensitivity).

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `UB-01` | Frozen reduced HAT decomposition. | head + upper trunk + middle trunk + 2x upper arm + 2x forearm + 2x hand (pelvis excluded and carried by the root pelvis body) | — | `LITERATURE_DIRECT` | S01 |
| `UB-02` | Fixed bar-hold posture parameterization (inertia aggregation reference posture). | `{"trunk": "MPT+UPT collinear on +z", "head_pitch_deg": 0.0, "shoulder_joint_x_m": 0.0, "shoulder_joint_y_m": 0.16, "shoulder_joint_z_m": 0.5433328546812176, "wrist_dz_m": -0.035, "hand_grip_half_width_y_m": 0.24, "elbow_rule": "two-link IK in the vertical ... | m / deg | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01;S04 |
| `UB-03` | Aggregated HAT mass. | 38.7969 | kg | `LITERATURE_DIRECT` | S01 |
| `UB-04` | Aggregated HAT COM in the HAT frame (origin MIDH). | `[-0.01596456566162191, -2.8616281832444254e-18, 0.4496738763907584]` | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01 |
| `UB-05` | Aggregated HAT inertia tensor about its COM (full 3x3, HAT frame). | `[[1.692352492111382, 3.469446951953614e-18, -0.02077944879714553], [3.469446951953614e-18, 1.4753589080582343, 3.469446951953614e-18], [-0.02077944879714553, 3.469446951953614e-18, 0.8504250560234003]]` | kg m^2 | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01 |
| `UB-06` | HAT principal moments and principal rotation. | `{"principal_moments_kg_m2": [1.6928650336618478, 1.475358908058234, 0.8499125144729343], "principal_rotation_about_y_deg": -1.4129592461485285}` | kg m^2 / deg | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `UB-07` | Bar local placement relative to the HAT frame. | `{"center_m": [-0.095, 0.0, 0.6], "orientation": "identity (bar axis along +y)"}` | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S04 |
| `UB-08` | Combined HAT+bar mass, COM and inertia about the combined COM. | `{"mass_kg": 58.7969, "com_m": [-0.04284878382223176, -1.888233945369835e-18, 0.5008079067934639], "inertia_about_com_kg_m2": [[13.746432717170366, 6.454194752049098e-18, 0.1360143545930838], [6.454194752049098e-18, 1.8608048237640293, -2.2075708375430198e-... | kg / m / kg m^2 | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01;S04 |
| `UB-09` | Arm-fold disclosure. | `{"elbow_flexion_deg": 155.04504872475866, "elbow_interior_angle_deg": 24.954951275241342}` | deg | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01 |
| `UB-10` | No arm swing, no bar roll, no bar compliance. | arms/hands rigidly attach to the bar; bar is rigidly attached to HAT in V1; sleeve rotation, bar flex and hand compliance are outside V1 | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S04;S16 |
| `UB-11` | System COM versus athlete COM. | SYSTEM_COM (athlete + rigid bar, 99 kg) is the primary mechanics authority; ATHLETE_COM is secondary/report-only | — | `PHYSICS_IDENTITY` | — |

## Sensitivity and downstream obligations

- **UB-02** — declared perturbations: {'bar_placement_m': [0.025, 0.025], 'head_pitch_deg': [-10.0, 0.0, 10.0], 'shoulder_joint_x_m': [-0.03, 0.03], 'shoulder_joint_y_m': [0.14, 0.18], 'wrist_dz_m': [-0.055, -0.015], 'hand_grip_half_width_y_m': [0.18, 0.3]}
- **UB-04** — covered by UB-02 perturbations
- **UB-05** — covered by UB-02 perturbations; system Iyy range [1.73167341, 2.02437733] kg m^2 over the declared set
- **UB-07** — placement sensitivity +/- [0.025, 0.025] m in x and z
- **UB-08** — covered by placement sensitivity
- **UB-09** — covered by grip-width and wrist-offset sensitivity

## Notes and locators

- **UB-01** — HAT mass 38.7969 kg equals the Linear authority value 38.7969 kg. (locator: S01 Table 4 component rows; sources: S01).
- **UB-02** — Every unmeasured posture parameter is engineering nominal with mandatory sensitivity; none is anthropometric truth. (locator: de Leva component geometry; frozen engineering bar-hold pose; sources: S01;S04).
- **UB-03** — Machine-recomputed. (locator: sum of de Leva mass fractions; sources: S01).
- **UB-04** — The component masses are literature-direct; the aggregate COM inherits the engineering posture classification and is not an anthropometric measurement. (locator: de Leva component masses (LITERATURE_DIRECT) aggregated under the frozen engineering bar-hold posture UB-02; sources: S01).
- **UB-05** — Ixy=Iyz=0 by left-right symmetry; Ixz is non-zero and must not be silently dropped. (locator: de Leva component tensors (LITERATURE_DIRECT) rotated into the HAT frame and translated under the frozen engineering bar-hold posture UB-02; sources: S01).
- **UB-06** — The principal rotation is small (~1.4 deg); a diagonal implementation is permitted only with this declared rotation or with the full tensor. (locator: eigendecomposition of UB-05 (which inherits the engineering posture classification); sources: —).
- **UB-07** — RES-83 must not silently move the nominal; any change is an authority amendment. (locator: upper-back/shoulder-girdle placement, posterior of the trunk axis; sources: S04).
- **UB-08** — The sagittal-plane relevant inertia is Iyy = 1.86080482 kg m^2; transverse (pitch-axis) values are larger because of the 2.2 m bar. (locator: rigid HAT (engineering posture) and rigid bar at the engineering placement UB-07; combination uses exact parallel-axis identity; sources: S01;S04).
- **UB-09** — The reduced HAT inertially represents a fixed bar-hold; it is not a claim about achievable articulated elbow angles. (locator: geometric consequence of frozen hand placement and published arm lengths; sources: S01).
- **UB-10** — Any future arm-swing or bar-compliance variant is a new model revision. (locator: task definition; system authority section 8; sources: S04;S16).
- **UB-11** — Force-impulse and ballistic identities use SYSTEM_COM and 99 kg. (locator: carried-bar closure of the airborne system; sources: —).

## Cross-references

- `ANTHROPOMETRY_BSIP_AUTHORITY`
- `LOAD_BAR_AUTHORITY`
- `COLLISION_CONTACT_POLICY`
- `DERIVED_QUANTITIES.json`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
