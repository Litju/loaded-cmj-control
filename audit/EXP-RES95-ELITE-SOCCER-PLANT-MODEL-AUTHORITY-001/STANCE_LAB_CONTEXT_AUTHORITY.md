# STANCE_LAB_CONTEXT_AUTHORITY — Stance and Laboratory Context Authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen stance/laboratory context: leg-plane geometry, force-plate chain and reporting conventions.

## Context

The 0.170 m leg-plane separation is reduced-model geometry. It is not an elite-soccer stance-width claim, and it must not be relabeled as one.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `SL-01` | Model leg-plane separation. | `{"leg_plane_centerlines_y_m": [-0.085, 0.085], "separation_m": 0.17}` | m | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `SL-02` | Foot orientation in the reduced model. | feet parallel (yaw removed); no toe-out angle is modeled | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | — |
| `SL-03` | Laboratory context. | indoor force-plate setting with training shoes; not cleat-on-turf | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S22 |
| `SL-04` | Force-plate measurement chain. | `{"plates": "dual force plates", "export_rate_hz": 1000, "rule": ">=1000 Hz canonical comparison export with documented anti-aliasing/filter path; raw native signals retained"}` | Hz | `LITERATURE_INFORMED_SYNTHETIC` | S11;S12;S22 |
| `SL-05` | Countermovement depth and stance are self-selected. | no single ideal knee/hip depth is frozen; professional protocols commonly yield ~60-90 deg knee flexion while elite data show legitimate strategy variability | — | `LITERATURE_INFORMED_SYNTHETIC` | S06;S07;S16 |
| `SL-06` | Relative-to-quiet-stance reporting. | all performance and force metrics are reported relative to the quiet-standing reference, with raw and filtered traces retained | — | `LITERATURE_INFORMED_SYNTHETIC` | S11;S12 |

## Sensitivity and downstream obligations

- **SL-01** — plane-separation sensitivity only as engineering geometry (+/- 0.010 m)
- **SL-05** — strategy-envelope study required

## Notes and locators

- **SL-01** — 0.170 m is NOT an elite-soccer CMJ stance-width claim; true stance width and lateral balance are outside V1. (locator: sagittal reduction geometry; sources: —).
- **SL-02** — Do not run a biological stance-width sensitivity by sliding feet laterally without a coherent 3-D pelvis/hip model. (locator: yaw structurally removed; sources: —).
- **SL-03** — Field/turf/cleat validity is outside the claim. (locator: task context of use; sources: S22).
- **SL-04** — Native simulator state/contact resolution remains event authority. (locator: force-platform sampling literature recorded in the system authority; sources: S11;S12;S22).
- **SL-05** — Microscopic/noise-only dips must be rejected, but no elite squat depth may be hard-coded as optimal. (locator: system authority sections 12 and 14; sources: S06;S07;S16).

## Cross-references

- `PLANT_TOPOLOGY_AUTHORITY`
- `EVENT_MEASUREMENT_BOUNDARY`
- `SPORT_CONTEXT_AUTHORITY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
