# COLLISION_CONTACT_POLICY — Collision and Contact Policy

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen legal-support/fall-contact boundary and explicit MuJoCo contact-parameter classification with bounded deferrals.

## Context

Contact authority is never reduced to one scalar friction coefficient. Each MuJoCo contact dimension is either frozen, classified engineering-nominal with sensitivity, or explicitly deferred to RES-84/86 with its qualification domain frozen here.

No disabled collision shell may simultaneously serve as evidence for a prohibited-contact claim.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `CC-01` | Valid floor support. | only the active plantar heel / forefoot / toe regions of either foot may create valid floor support | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S02;S22 |
| `CC-02` | Prohibited / fall contact. | pelvis, HAT, thigh, shank or bar contact with the floor is prohibited/fall contact; body/bar-floor contact is a task failure | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `CC-03` | Collision enablement matrix (summary). | `{"enabled": ["plantar regions vs floor", "bar vs floor (fail register)", "bar vs nonattached lower limb", "opposite-foot collisions", "nonadjacent self-collision where physically possible"], "excluded_with_rationale": ["intentional bar/HAT/hand attachment ... | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `CC-04` | No artificial support or catch. | no artificial root limits, root springs, root damping, hidden supports or catch planes; root DOF passives are exactly zero | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `CC-05` | Contact dimension (condim) classification. | `{"nominal_plantar_floor": 4, "nominal_other_contacts": 3, "sensitivity": [3, 4, 6], "rationale": "plantar-floor contacts carry sliding + torsional friction authority; other contacts are declared nominal"}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `CC-06` | Sliding friction. | `{"mu_slide_nominal": 0.9, "sensitivity": [0.5, 1.5]}` | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `CC-07` | Torsional friction. | `{"status": "numeric value DEFERRED_TO_LATER_AUTHORITY (RES-84)", "ownership": "RES-84 owns the numeric value and its solution-verification justification", "qualification_domain_frozen_here": "must be selected by solution verification, declared in candidate... | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `CC-08` | Rolling friction. | `{"nominal": 0.0, "rationale": "nominal foot contact is box-like; rolling friction is not needed for box-floor contact", "condition": "if any round/capsule plantar contact geometry is introduced, a numeric rolling-friction value becomes DEFERRED_TO_LATER_AU... | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `CC-09` | Contact margin and gap. | `{"margin": "numeric value DEFERRED_TO_LATER_AUTHORITY (RES-84); semantics frozen: included in the takeoff clearance guard max(2 mm, margin + verified numerical/penetration allowance)", "gap": "numeric value DEFERRED_TO_LATER_AUTHORITY (RES-84); semantics f... | m | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `CC-10` | solref ownership. | `{"owner": "RES-84 / RES-86", "provisional_rule": "any provisional standing-contact setting is labeled PROVISIONAL_NUMERICAL and cannot silently become final candidate authority", "must_survive": ["timestep refinement", "impact/contact sensitivity", "no qua... | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `CC-11` | solimp ownership. | `{"owner": "RES-84 / RES-86", "provisional_rule": "provisional values labeled PROVISIONAL_NUMERICAL; final values selected by solution verification, never by labeling them human soft tissue"}` | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `CC-12` | Penetration authority. | `{"numeric_tolerance": "DEFERRED_TO_LATER_AUTHORITY (RES-84/RES-86 solution verification)", "frozen_constraints": ["bounded penetration with no tunneling", "no contact inversion/chatter exploitation", "penetration allowance declared in the clearance guard",... | m | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `CC-13` | Engineered-contact realism rule. | final candidate must show no material dependence on the declared contact/friction sensitivity and no uncontrolled slip, artificial sticking, friction-cone saturation or solver-dependent qualitative reversal | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |

## Sensitivity and downstream obligations

- **CC-01** — contact-parameter sensitivity required
- **CC-03** — representative-pose collision audit mandatory: standing, deep legal countermovement, takeoff extension, flight, touchdown, landing absorption, recovery
- **CC-04** — zero-value verification required
- **CC-05** — condim sensitivity must be executed if enabled dimensions differ from nominal
- **CC-06** — mandatory sweep [0.50, 1.50]
- **CC-07** — required alongside CC-05/CC-06
- **CC-08** — required if round contact geoms are introduced
- **CC-09** — RES-84 must seal exact evaluated values in the candidate identity
- **CC-10** — mandatory solution-verification study
- **CC-11** — mandatory solution-verification study
- **CC-12** — declared tolerance in candidate identity
- **CC-13** — material qualitative reversal blocks final authority until explained or narrowed

## Notes and locators

- **CC-01** — Landing may legitimately begin toe/forefoot-first before heel. (locator: system authority section 9; sources: S02;S22).
- **CC-02** — No disabled collision shell may simultaneously serve as evidence for a prohibited-contact claim. (locator: system authority sections 9 and 16; sources: S22).
- **CC-03** — All exclusions machine-readable with rationale. (locator: system authority section 9; sources: S22).
- **CC-04** — State overrides/freezes fail NO_ARTIFICIAL_SUPPORT. (locator: system authority sections 7 and 15; sources: —).
- **CC-05** — Contact authority is never reduced to one scalar friction coefficient. (locator: MuJoCo contact model semantics; sources: S22).
- **CC-06** — 0.90 is not a measured universal athletic-surface truth. (locator: sports shoe/surface measurements span materially different values by outsole and method; sources: S22).
- **CC-07** — Deferral is bounded: value + sensitivity domain are frozen by RES-84, not silently chosen at run time. (locator: MuJoCo friction-cone semantics; sources: S22).
- **CC-08** — Nominal zero is a statement about the nominal geometry, not a claim about shoes. (locator: MuJoCo rolling-friction semantics; sources: S22).
- **CC-09** — Margin/gap may never silently shrink the takeoff clearance definition. (locator: MuJoCo margin/gap semantics; EVENT_MEASUREMENT_BOUNDARY; sources: S22).
- **CC-10** — Extremely stiff contact is not automatically more physical. (locator: MuJoCo contact compliance; sources: S22).
- **CC-11** — Same bounded-deferral rule as CC-10. (locator: MuJoCo contact impedance; sources: S22).
- **CC-12** — Penetration evidence must be reported raw, not only aggregated. (locator: solution verification; RES-82 project hard-rule candidate; sources: S22).
- **CC-13** — No numerical contact choice may be presented as a sports-science measurement. (locator: system authority section 10; sources: S22).

## Cross-references

- `FOOT_MTP_MODEL_AUTHORITY`
- `PLANT_TOPOLOGY_AUTHORITY`
- `DEFERRED_NUMERICAL_CALIBRATIONS`
- `EVENT_MEASUREMENT_BOUNDARY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
