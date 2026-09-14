# LOAD_BAR_AUTHORITY — External Load (Barbell) Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen IWF-style 20 kg composite bar geometry, inertial surrogate, collision embodiment and placement.

## Context

The bar is a separate rigid body with its own mass and inertia. A uniform 2.200 m x 28 mm rod must never be presented as IWF geometry; the nominal surrogate is the documented stepped-cylinder composite, and the uniform rod survives only as a declared sensitivity case.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `LB-01` | Bar mass. | 20.0 | kg | `LITERATURE_DIRECT` | S04 |
| `LB-02` | Bar total length. | 2.2 | m | `LITERATURE_DIRECT` | S04 |
| `LB-03` | Bar grip-section diameter. | 0.028 | m | `LITERATURE_DIRECT` | S04 |
| `LB-04` | Grip section length. | 1.31 | m | `LITERATURE_DIRECT` | S04 |
| `LB-05` | Sleeve diameter. | 0.05 | m | `LITERATURE_DIRECT` | S04 |
| `LB-06` | Sleeve length (each). | 0.415 | m | `LITERATURE_DIRECT` | S04 |
| `LB-07` | Composite inertial surrogate construction. | `{"shaft_length_m": 1.37, "shaft_diameter_m": 0.028, "sleeves": [0.415, 0.05], "rho_eff_kg_m3": 8086.422350243007, "m_shaft_kg": 6.821547880650857, "m_sleeve_each_kg": 6.589226059674571}` | m / kg/m^3 / kg | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S04 |
| `LB-08` | Composite bar inertia about its COM (bar axis along y). | `{"I_transverse_kg_m2": 11.75585696777048, "I_axis_kg_m2": 0.004786777979600392}` | kg m^2 | `PHYSICS_IDENTITY` | S04 |
| `LB-09` | Bar collision geometry. | composite collision representation matching LB-07 dimensions (grip/shaft segment plus two sleeves); exact MuJoCo primitive choice is a RES-83 implementation detail; bar-floor collision physically enabled and its contact is a task failure | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S04 |
| `LB-10` | Attachment semantics. | rigid attachment to HAT in V1; no bar roll/compliance; intentional bar/HAT/hand attachment contacts excluded from the fall/prohibited-contact register | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S04;S16 |

## Sensitivity and downstream obligations

- **LB-07** — declared alternative surrogate: uniform 2.200 m x 0.028 m rod (non-IWF) with I_transverse 8.06764667 kg m^2
- **LB-08** — surrogate-shape sensitivity: uniform-rod case; sleeve/shaft mass redistribution +/-10%

## Notes and locators

- **LB-07** — A uniform 2.2 m x 28 mm rod must never be presented as the IWF bar geometry. (locator: coaxial stepped-cylinder surrogate; rho_eff chosen so total mass equals exactly 20 kg; sources: S04).
- **LB-08** — Bar pitch inertia materially exceeds a uniform-rod estimate (mass concentration in the sleeves). (locator: analytic stepped-cylinder inertia; sources: S04).
- **LB-09** — No disabled bar shell may simultaneously serve as evidence for a prohibited-contact claim. (locator: system authority section 8; COLLISION_CONTACT_POLICY; sources: S04).

## Cross-references

- `REFERENCE_ATHLETE_SPEC`
- `UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY`
- `COLLISION_CONTACT_POLICY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
