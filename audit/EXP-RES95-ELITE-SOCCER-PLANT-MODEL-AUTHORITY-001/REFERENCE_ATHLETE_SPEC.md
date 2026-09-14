# REFERENCE_ATHLETE_SPEC — Reference Athlete Specification

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen synthetic nominal reference athlete, external load and total mechanical system quantities.

## Context

The reference athlete is a rounded synthetic consensus inside contemporary first-team professional male outfield distributions. It is not a real person and not the exact population mean.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `RA-01` | Synthetic nominal athlete sex and age class. | male; senior_adult; reference age 24 y (nominal) | — | `LITERATURE_INFORMED_SYNTHETIC` | S05;S06 |
| `RA-02` | Reference stature. | 1.835 | m | `LITERATURE_INFORMED_SYNTHETIC` | S05;S06 |
| `RA-03` | Reference athlete mass. | 79.0 | kg | `LITERATURE_INFORMED_SYNTHETIC` | S05;S06 |
| `RA-04` | External load mass. | 20.0 | kg | `LITERATURE_DIRECT` | S04;S13;S14 |
| `RA-05` | Total mechanical system mass. | 99.0 | kg | `PHYSICS_IDENTITY` | — |
| `RA-06` | Load-to-athlete mass ratio. | 0.25316455696202533 | — | `PHYSICS_IDENTITY` | — |
| `RA-07` | Nominal system weight at the project gravity constant. | 971.19 | N | `PHYSICS_IDENTITY` | — |
| `RA-09` | Reconciliation of the historical RES-82 body-weight constant. | `{"RES82_BW": "95.0 kg * 9.81 = 931.95 N (historical V2 75 kg athlete + 20 kg load)", "SUCCESSOR_SYSTEM_WEIGHT": "99.0 kg * 9.81 = 971.19 N", "rule": "RES-82 thresholds expressed in units of BW are recomputed on the 99.0 kg successor base; no downstream art... | N | `PHYSICS_IDENTITY` | S22 |
| `RA-08` | Reference athlete is a synthetic nominal, not a measured subject. | no individual prediction or exact-mean claim is permitted from this nominal | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S05;S06 |

## Sensitivity and downstream obligations

- **RA-02** — sensitivity set [1.835, 1.7764, 1.8936] m
- **RA-03** — sensitivity set [79, 72.69, 85.31] kg

## Notes and locators

- **RA-01** — Not a real person and not the exact population mean. (locator: System authority section 3; sources: S05;S06).
- **RA-02** — Inside contemporary first-team professional distributions. (locator: S05 183.72 +/- 5.86 cm; S06 184 +/- 7 cm; rounded synthetic consensus; sources: S05;S06).
- **RA-03** — Rounded synthetic consensus; not a population mean claim. (locator: S05 79.91 +/- 6.31 kg; S06 81 +/- 9 kg; sources: S05;S06).
- **RA-04** — Rigid 20 kg composite bar surrogate; see LOAD_BAR_AUTHORITY. (locator: IWF men's bar standard; loaded-CMJ task; sources: S04;S13;S14).
- **RA-05** — Force/impulse and ballistic identities use this mass; machine check in ANTHROPOMETRY_BSIP_AUTHORITY. (locator: athlete + rigid bar mass sum; sources: —).
- **RA-06** — Derived identity. (locator: 20.0 / 79.0; sources: —).
- **RA-07** — Athlete-only weight is 774.99 N; system authority uses the full carried system. (locator: 99.0 kg * 9.81 m/s^2; sources: —).
- **RA-09** — Historical RES-82 BW remains in RES-82's sealed artifacts; this decision is the successor reconciliation so force thresholds are computed on one base. (locator: RES-82 EVENT_CONTRACT term BW versus the controlling Linear system authority section 3; sources: S22).
- **RA-08** — All downstream claims inherit this ceiling. (locator: System authority section 3; sources: S05;S06).

## Cross-references

- `SPORT_CONTEXT_AUTHORITY`
- `ANTHROPOMETRY_BSIP_AUTHORITY`
- `LOAD_BAR_AUTHORITY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
