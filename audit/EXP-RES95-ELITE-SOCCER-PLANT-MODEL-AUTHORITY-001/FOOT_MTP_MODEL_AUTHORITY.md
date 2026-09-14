# FOOT_MTP_MODEL_AUTHORITY — Foot and MTP Model Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen multi-segment foot abstraction, plantar contact geometry, foot-segment inertia and the passive/active MTP contract.

## Context

The locked-midfoot nominal is a declared material model-form uncertainty with a mandatory compliant/articulated-midfoot alternative in final qualification.

Passive MTP values are engineering priors only; zero-passive and alternative passive-property cases are mandatory, and passive and active MTP work must be reported separately.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `FM-01` | Frozen foot abstraction. | per foot: hindfoot body, forefoot body welded/fixed to the hindfoot in V1 (separate geometry/inertia, no midtarsal DOF), toe/phalanx body, one sagittal MTP hinge; ankle hinge connects shank -> hindfoot | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02 |
| `FM-02` | Locked midfoot classification. | a rigid arch / locked midfoot is a MATERIAL MODEL-FORM UNCERTAINTY, not harmless omission | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02 |
| `FM-03` | Required midfoot alternative sensitivity. | `{"requirement": "final qualification must compare the locked-midfoot Plant against a plausible compliant/articulated-midfoot alternative (at least one bounded sagittal midtarsal DOF or equivalent compliant coupling with documented range/stiffness)", "chang... | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02 |
| `FM-04` | Foot segment masses. | `{"hindfoot": 0.4675536, "forefoot": 0.4588952, "phalanx": 0.1558512}` | kg | `LITERATURE_DIRECT` | S01;S02 |
| `FM-05` | MTP nominal axis location and toe length. | `{"MTP_X_FROM_HEEL_M": 0.20625, "TOE_LENGTH_M": 0.06875, "ratio_of_foot_length": 0.75}` | m / — | `LITERATURE_INFORMED_SYNTHETIC` | S02;S20 |
| `FM-06` | Plantar contact regions and dimensions (heel / forefoot / toe). | `{"heel": {"x_from_heel_start_m": 0.0, "x_from_heel_end_m": 0.07, "width_m": 0.07}, "forefoot": {"x_from_heel_start_m": 0.07, "x_from_heel_end_m": 0.20625, "width_m": 0.105}, "toe": {"x_from_heel_start_m": 0.20625, "x_from_heel_end_m": 0.275, "width_m": 0.0... | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02;S18 |
| `FM-07` | Foot frame, joint centers and segment partition. | `{"ankle_joint_center": [0.0, 0.0, 0.0], "mid_tarsal_x_from_heel_m": 0.07, "mtp_x_from_heel_m": 0.20625, "toe_tip_x_from_heel_m": 0.275, "segment_lengths_m": {"hindfoot": 0.07, "forefoot": 0.13624999999999998, "phalanx": 0.06875000000000003}, "segment_com_a... | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02 |
| `FM-08` | Foot segment inertia rule. | `{"rule": "I_segment_about_com_diagonal = I_rel_diagonal * m_segment^(5/3)", "segment_inertia_about_com_kg_m2": {"hindfoot": [0.0004337502415040432, 0.0005182470417970387, 0.0004168508814454441], "forefoot": [0.0003822233613175279, 0.0004723188679138023, 0.... | kg m^2 | `LITERATURE_DIRECT` | S02 |
| `FM-09` | Passive MTP baseline mechanics. | `{"neutral_rad": 0.0, "k_nominal_Nm_per_rad": 25.0, "c_nominal_Nms_per_rad": 2.0}` | N m/rad / N m s/rad | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S10 |
| `FM-10` | Active MTP channel. | one bounded net plantarflexor/resistive moment per foot in parallel with the passive baseline; reduced net moment, not a literal single-muscle actuator; no hidden high-power toe motor or arbitrary flight-energy injection | — | `LITERATURE_INFORMED_SYNTHETIC` | S08;S09 |
| `FM-11` | MTP work accounting. | qualification must report passive MTP work, active MTP work, total MTP work/power and an active-vs-passive sensitivity, proving no double-counting of distal power and no hidden toe-energy source | — | `LITERATURE_INFORMED_SYNTHETIC` | S08;S09;S10 |
| `FM-13` | Sole plane and no-subsole-geometry rule. | `{"sole_plane_z_in_foot_frame_m": -0.071565, "rule": "no collision geom may extend below the sole plane, and all three contact patches are planar on the sole plane"}` | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02 |
| `FM-12` | Foot geometry sensitivity. | ankle x +/- 0.010 m; MTP location span 0.70-0.79 L; foot length +/- 0.010 m; midfoot alternative; segment inertia 0.5x/1.5x | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02;S18;S20;S21 |

## Sensitivity and downstream obligations

- **FM-01** — locked-midfoot alternative required (FM-03)
- **FM-02** — mandatory compliant/articulated-midfoot alternative
- **FM-03** — required
- **FM-04** — foot-inertia sensitivity 0.5x/1.5x
- **FM-05** — MTP location sensitivity [0.70, 0.79] x foot length
- **FM-06** — patch dimension sensitivity +/- 0.010 m; only these regions may create valid support
- **FM-07** — midtarsal placement sensitivity +/- 0.010 m; segment-inertia sensitivity 0.5x/1.5x
- **FM-08** — foot-inertia sensitivity 0.5x/1.5x; midfoot alternative
- **FM-09** — k set [0, 12.5, 25, 40]; c set [0, 0.4, 2]; zero-passive case (k=0,c=0)
- **FM-10** — final torque/rate/power caps owned by RES-85; supported-phase saturation telemetry mandatory
- **FM-11** — active-vs-passive separation required in every qualification report
- **FM-12** — declared

## Notes and locators

- **FM-01** — The old single rigid box foot is rejected. (locator: system authority section 6; S02 three-segment model; sources: S02).
- **FM-02** — Rigid-foot assumptions can materially distort ankle ROM and quasi-stiffness. (locator: multi-segment foot literature; sources: S02).
- **FM-03** — If locking the midfoot materially changes these outputs, the V1 locked-midfoot Plant is not qualified. (locator: system authority section 6; sources: S02).
- **FM-04** — Sum equals the de Leva whole-foot mass. (locator: de Leva whole-foot mass fraction 0.0137 x 79 kg, split 43.2/42.4/14.4 % per S02; sources: S01;S02).
- **FM-05** — 0.75 is a reduced-axis nominal, not subject-specific anatomy. (locator: first-MTP fulcrum locations span ~70-79% of foot length in published data; sources: S02;S20).
- **FM-06** — The three regions are independently observable; RES-84 builds the support hull/CoP authority from their active contacts. (locator: reduced contact patches matching the frozen foot partition; heel breadth is materially below ball breadth (S18); sources: S02;S18).
- **FM-07** — Assembled foot COM cross-check against the de Leva whole-foot COM is declared in AB-15. (locator: S02 joint-chain proximal conventions applied to the frozen reduced geometry; sources: S02).
- **FM-08** — Diagonal terms used with a declared axis mapping; off-diagonal products are not silently imported into the sagittal model. (locator: S02 Table 3 relative tensors scaled by m^(5/3); sources: S02).
- **FM-09** — Passive values are engineering priors only; material dependence reopens authority. (locator: predictive gait-model prior; not a CMJ measurement; sources: S10).
- **FM-10** — A passive-only MTP is NOT sufficient as the sole mechanism for a human-valid maximal-CMJ claim. (locator: MTP restriction reduces CMJ height/impulse; intrinsic foot muscle activity drives MTP stiffness; the actuator channel is a reduced design decision informed by that evidence; sources: S08;S09).
- **FM-11** — Separation must be demonstrated, not asserted. (locator: S09/S10 mechanism and double-count risk; sources: S08;S09;S10).
- **FM-13** — The foot's visual/collision body above the sole is a RES-83 implementation detail, but no collision geom may protrude below the sole plane, and only the frozen patches may create valid support. (locator: locked-midfoot reduced geometry; all three contact patches are planar on the sole plane; sources: S02).
- **FM-12** — No sensitivity may be silently omitted from the qualification manifest. (locator: frozen foot geometry; sources: S02;S18;S20;S21).

## Cross-references

- `ANTHROPOMETRY_BSIP_AUTHORITY`
- `JOINT_COORDINATE_ROM_AUTHORITY`
- `COLLISION_CONTACT_POLICY`
- `DEFERRED_NUMERICAL_CALIBRATIONS`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
