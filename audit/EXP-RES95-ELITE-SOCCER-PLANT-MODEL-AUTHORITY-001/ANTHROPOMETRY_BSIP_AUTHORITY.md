# ANTHROPOMETRY_BSIP_AUTHORITY — Anthropometry and BSIP Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen body-segment inertial authority (de Leva-adjusted Zatsiorsky-Seluyanov), geometry, foot partition and reproducibility rules.

## Context

Machine-readable coefficients live in AUTHORITY_INPUTS.json; all numbers in this artifact are recomputed deterministically by build_authority_numbers.py into DERIVED_QUANTITIES.json.

The segment-chain closure check (AB-13) is declared rather than silently repaired: the joint-center-resolved chain overshoots the reference stature by about 3.1 cm, and RES-83 must absorb that in the standing pose, never by rescaling frozen lengths.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `AB-01` | Primary BSIP authority. | de Leva (1996) adjustment of Zatsiorsky-Seluyanov parameters, joint-center referenced | — | `LITERATURE_DIRECT` | S01 |
| `AB-02` | Male segment mass fractions (head 0.0694; upper trunk 0.1596; middle trunk 0.1633; pelvis/lower trunk 0.1117; upper arm 0.0271 each; forearm 0.0162 each; hand 0.0061 each; thigh 0.1416 each; shank 0.0433 each; whole foot 0.0137 each). | `[0.0694, 0.1596, 0.1633, 0.1117, 0.0271, 0.0162, 0.0061, 0.1416, 0.0433, 0.0137]` | fraction of athlete mass | `LITERATURE_DIRECT` | S01 |
| `AB-03` | Segment masses at the reference athlete mass. | `{"head": 5.482600000000001, "upt": 12.6084, "mpt": 12.9007, "pelvis": 8.8243, "upper_arm": 2.1409, "forearm": 1.2797999999999998, "hand": 0.48190000000000005, "thigh": 11.1864, "shank": 3.4207, "whole_foot": 1.0823}` | kg | `LITERATURE_DIRECT` | S01 |
| `AB-04` | Athlete and system mass closure. | `{"hat": 38.7969, "pelvis": 8.8243, "two_thighs": 22.3728, "two_shanks": 6.8414, "two_feet": 2.1646, "athlete_total": 79.0, "bar": 20.0, "system_total": 99.0}` | kg | `PHYSICS_IDENTITY` | S01 |
| `AB-05` | Thigh segment length (normative form). | 0.4449875 | m | `LITERATURE_DIRECT` | S01 |
| `AB-06` | Shank segment length (normative form). | 0.45746549999999997 | m | `LITERATURE_DIRECT` | S01 |
| `AB-07` | HAT component segment lengths (head VERT-CERV; upper trunk CERV-XYPH; middle trunk XYPH-OMPH). | `{"head": 0.2560146467547386, "upt": 0.255171453187823, "mpt": 0.22713526708788048}` | m | `LITERATURE_DIRECT` | S01 |
| `AB-08` | Pelvis body parameterization. | `{"mass_kg": 8.8243, "com_z_above_midh_m": 0.05966063512349223, "length_m": 0.1535666283744974}` | kg / m | `LITERATURE_DIRECT` | S01 |
| `AB-09` | Foot length and width nominal. | `{"FOOT_LENGTH_M": 0.275, "FOOT_WIDTH_M": 0.105}` | m | `LITERATURE_INFORMED_SYNTHETIC` | S03;S20;S21 |
| `AB-10` | Ankle joint center height above the sole. | 0.071565 | m | `HUMAN_ANATOMY_REFERENCE` | S03 |
| `AB-11` | Ankle joint center horizontal position from the heel. | 0.055 | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01;S03 |
| `AB-12` | de Leva radius-of-gyration axis interpretation. | `{"sagittal": "about the model medio-lateral axis (sagittal-plane rotation)", "transverse": "about the in-plane axis perpendicular to the segment long axis", "longitudinal": "about the segment long axis"}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S01 |
| `AB-13` | Segment-chain closure check against reference stature. | `{"hip_height_m": 0.9740179999999999, "midh_to_vertex_m": 0.8918879954049395, "sum_m": 1.8659059954049395, "stature_m": 1.835, "delta_m": 0.030905995404939546, "delta_fraction_of_stature": 0.016842504307868964}` | m | `PHYSICS_IDENTITY` | S01;S03 |
| `AB-14` | Lower-limb segment inertias about their own COMs (thigh Ixx=Iyy/Izz; shank Ixx/Iyy/Izz). | `{"thigh_kg_m2": [[0.23976057065071502, 0.0, 0.0], [0.0, 0.23976057065071502, 0.0], [0.0, 0.0, 0.04917660063207587]], "shank_kg_m2": [[0.04438440231686326, 0.0, 0.0], [0.0, 0.04654918083021296, 0.0], [0.0, 0.0, 0.007594621444486416]], "pelvis_kg_m2": [[0.06... | kg m^2 | `LITERATURE_DIRECT` | S01 |
| `AB-15` | Whole-foot de Leva cross-check versus the multi-segment foot construction. | `{"whole_foot_sagittal_I_about_com_kg_m2": 0.005406040472937501, "assembled_segment_com_x_from_heel_m": 0.11525194999999999, "deleva_com_x_from_heel_m": 0.1214125, "com_x_delta_m": -0.006160550000000015}` | kg m^2 / m | `LITERATURE_DIRECT` | S01;S02 |
| `AB-16` | Stature scaling rule for de Leva component lengths. | linear scaling factor s = H / 1.741 = 1.05399196 | — | `LITERATURE_INFORMED_SYNTHETIC` | S01 |
| `AB-17` | No silent anthropometric reinterpretation. | any change to a frozen ratio, mass fraction or length is an explicit authority amendment with downstream requalification | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |

## Sensitivity and downstream obligations

- **AB-09** — foot geometry sensitivity: FOOT_LENGTH +/- 0.010 m; FOOT_WIDTH +/- 0.007 m
- **AB-10** — ankle-height sensitivity +/- 0.010 m
- **AB-11** — sensitivity [0.045, 0.065] m
- **AB-12** — segment-inertia sensitivity 0.9x/1.1x on all segment inertia terms
- **AB-14** — segment-inertia sensitivity 0.9x/1.1x
- **AB-15** — foot-inertia sensitivity 0.5x/1.5x (Matsumoto practice) plus the compliant-midfoot alternative
- **AB-16** — anthropometry sensitivity: stature set [1.835, 1.7764, 1.8936] m and mass set [79, 72.69, 85.31] kg (approximately +/-1 SD of the S05 cohort: 5.86 cm, 6.31 kg)

## Notes and locators

- **AB-01** — Machine-readable coefficients in AUTHORITY_INPUTS.json; no uniform-density torso shortcut may be called anthropometric authority. (locator: S01 Tables 1-4; sources: S01).
- **AB-02** — Fractions sum to 1.0000 across all segments. (locator: S01 Table 4, male column; sources: S01).
- **AB-03** — Recomputed deterministically by build_authority_numbers.py. (locator: S01 Table 4 mass fractions * 79.0 kg; sources: S01).
- **AB-04** — Machine test: athlete total = 79.0 kg and system total = 99.0 kg within numerical tolerance. (locator: sum of body segments; plus bar mass; sources: S01).
- **AB-05** — Normative 0.2425 * H = 0.4449875 m; de Leva-scaled cross-check 0.444995405 m (delta 7.90493969e-06 m). (locator: frozen stature ratio 0.2425 * H; de Leva Table 4 male thigh (HJC-KJC) 422.2 mm x stature scale as cross-check; sources: S01).
- **AB-06** — Normative 0.2493 * H = 0.4574655 m; de Leva-scaled cross-check 0.45743251 m (delta -3.29899483e-05 m). (locator: frozen stature ratio 0.2493 * H; de Leva Table 4 male shank (KJC-LMAL) 434.0 mm x stature scale as cross-check; sources: S01).
- **AB-07** — Stature scaling rule: lengths scale linearly with H / 1.741 m. (locator: S01 Table 4 male alternative-endpoint rows x stature scale; sources: S01).
- **AB-08** — The pelvis body carries the de Leva lower-trunk (LPT) parameters; MIDH is the trunk-pelvis hinge center. (locator: S01 Table 4 male lower trunk OMPH-MIDH; sources: S01).
- **AB-09** — 0.275 x 0.105 m is a literature-informed synthetic nominal, not one-paper direct. (locator: S03 D-C foot length 0.152H = 0.279 m at H; S20/S21 elite and adolescent soccer feet; sources: S03;S20;S21).
- **AB-10** — Used for standing geometry and contact placement. (locator: Drillis-Contini Fig. 4.1: ankle height = 0.039 H; sources: S03).
- **AB-11** — Explicit engineering reduction; must not be presented as measured anatomy. (locator: no joint-center-resolved public value located; 0.20 * foot length; sources: S01;S03).
- **AB-12** — Axis-to-model mapping is an engineering reduction of the published axes; it is applied identically to every segment. (locator: S01 Table 4 column semantics (sagittal / transverse / longitudinal r); sources: S01).
- **AB-13** — The joint-center-resolved chain overshoots stature by ~3.1 cm (~1.7%); the causes are mixed landmark systems between sources. RES-83 must absorb this in the standing pose and must not rescale the frozen segment lengths to force closure. (locator: ankle height + shank + thigh + (MIDH to vertex chain); sources: S01;S03).
- **AB-14** — Body-frame origins: pelvis at MIDH, thigh at HJC, shank at KJC; z is proximal-to-distal. (locator: S01 Table 4 radii of gyration * segment lengths; sources: S01).
- **AB-15** — Cross-check deltas are declared, not hidden; the three-segment construction controls because it is what the Plant implements. (locator: S01 whole-foot row vs S02 three-segment construction; sources: S01;S02).
- **AB-16** — Applies to HAT components, pelvis and whole-foot reference length; thigh/shank are frozen by their stature ratios. (locator: S01 male sample stature 1.741 m; sources: S01).
- **AB-17** — Silent tuning is prohibited. (locator: System authority section 22; sources: S22).

## Cross-references

- `REFERENCE_ATHLETE_SPEC`
- `FOOT_MTP_MODEL_AUTHORITY`
- `UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY`
- `DERIVED_QUANTITIES.json`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
