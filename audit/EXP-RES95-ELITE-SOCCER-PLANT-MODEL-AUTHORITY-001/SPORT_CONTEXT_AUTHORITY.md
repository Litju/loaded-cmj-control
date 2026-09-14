# SPORT_CONTEXT_AUTHORITY — Sport Context Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen context of use for the elite-soccer successor Loaded-CMJ Plant: reference population, task, evidence hierarchy and claim boundary.

## Context

This artifact freezes the sport context and context-of-use (COU) for the successor Plant. It is binding on RES-83/84/85/87/89/90/91 and on any qualification report derived from this model revision.

The context is a synthetic nominal reference, not a claim about any individual athlete or about an exact population mean.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `SC-01` | Sport-context identifier for the successor Plant reference population. | ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1 | — | `LITERATURE_INFORMED_SYNTHETIC` | S05;S06;S07 |
| `SC-02` | Reference population definition. | healthy senior adult male first-team professional outfield soccer player, top-tier European domestic league / UEFA-caliber environment; goalkeepers excluded; field position not frozen | — | `LITERATURE_INFORMED_SYNTHETIC` | S05;S06;S07 |
| `SC-03` | Task definition. | maximal voluntary bilateral +20.0 kg back-loaded countermovement jump, hands fixed to the bar, no arm swing, dual-force-plate laboratory setting, full standing-countermovement-flight-landing-recovery sequence | — | `LITERATURE_INFORMED_SYNTHETIC` | S13;S14;S16 |
| `SC-04` | Claim boundary of the V1 reduction. | sagittal-plane mechanics only; lateral translation, roll and yaw are structurally removed; no 3-D cutting/lateral/asymmetric validity claim | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `SC-05` | Evidence hierarchy for authority decisions. | 1 human anatomy/joint mechanics/BSIP; 2 elite senior male first-division soccer anthropometry; 3 elite/professional soccer CMJ testing evidence; 4 soccer +20 kg loaded-jump evidence supplemented by broader loaded-CMJ work; 5 NBA elite CMJ data as secondary ... | — | `LITERATURE_INFORMED_SYNTHETIC` | S01;S05;S06;S07;S13;S14 |
| `SC-06` | Use of NBA CMJ cohort data. | secondary elite movement-strategy comparator only; it sets neither soccer anthropometry nor +20 kg performance thresholds | — | `LITERATURE_INFORMED_SYNTHETIC` | S07 |
| `SC-07` | Claim ceiling after the Plant rebuild (M6). | the Plant may be called a sagittal-dominant rigid-body model parameterized to a nominal elite senior male professional outfield soccer athlete and configured for a maximal 20 kg loaded countermovement jump; no subject-specific, population-norm, injury-risk,... | — | `LITERATURE_INFORMED_SYNTHETIC` | S22 |

## Sensitivity and downstream obligations

None beyond the generic V&V obligations.

## Notes and locators

- **SC-01** — Synthetic population identifier; not a claim about any individual. (locator: Linear Elite Soccer System Authority V1 section 1; Bongiovanni 2023 cohort; sources: S05;S06;S07).
- **SC-02** — Position-specific variants are future calibrations. (locator: System authority section 1; S05 positional differences; sources: S05;S06;S07).
- **SC-03** — Sagittal-dominant reduced rigid-body model. (locator: System authority section 1; loaded-CMJ task evidence; sources: S13;S14;S16).
- **SC-04** — Any materially asymmetric control claim requires a future true 3-D/6-DOF Plant. (locator: System authority sections 7 and 20; PLAN_TOPOLOGY_AUTHORITY; sources: S22).
- **SC-05** — No engineering value may be relabeled as an elite-soccer measurement. (locator: Linear RES-95 description; System authority section 2; sources: S01;S05;S06;S07;S13;S14).
- **SC-06** — Recorded to prevent strategy-template drift. (locator: System authority sections 1 and 14; sources: S07).
- **SC-07** — verification != validation; determinism != biomechanical validity. (locator: System authority section 20; RES-82 SCIENTIFIC_CLAIM_CEILING.md; sources: S22).

## Cross-references

- `REFERENCE_ATHLETE_SPEC`
- `PLANT_TOPOLOGY_AUTHORITY`
- `PERFORMANCE_AUTHORITY_BOUNDARY`
- `RES-82 SCIENTIFIC_CLAIM_CEILING.md`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
