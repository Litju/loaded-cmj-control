# DEFERRED_NUMERICAL_CALIBRATIONS — Deferred Numerical Calibrations

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Explicit isolation of every numerical/calibration decision that RES-95 must not freeze, with its owner and qualification domain.

## Context

Deferral is bounded: each deferred value has an owner, a frozen qualification domain and a required evidence form. No deferred value may silently become final candidate authority.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `DF-01` | Contact numerical calibration. | solref, solimp, margin, gap, torsional-friction and rolling-friction numeric values, penetration tolerance | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-02` | Takeoff clearance guard exact value. | max(2 mm, contact_margin + verified numerical/penetration allowance) evaluated and sealed | m | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-03` | MTP active moment limits. | torque cap, rate limit, power limit and allowed stance-phase action for the reduced net MTP moment | — | `DEFERRED_TO_LATER_AUTHORITY` | S08;S09 |
| `DF-04` | Major-joint actuator limits. | hip/knee/ankle/trunk torque caps, torque-rate limits and activation/smoothing constants | — | `DEFERRED_TO_LATER_AUTHORITY` | S16;S22 |
| `DF-05` | Balance capture and recovery envelopes. | E11 capture envelope, E12 robust standing envelope and post-E12 observation window | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-06` | Landing force gate. | peak landing force as a hard +20 kg elite scalar gate | — | `DEFERRED_TO_LATER_AUTHORITY` | S15 |
| `DF-07` | Elite +20 kg H2 mapping. | method-matched comparator/mapping analysis before any elite hard gate | — | `DEFERRED_TO_LATER_AUTHORITY` | S07;S11;S12 |
| `DF-08` | Solver and timestep verification. | timestep refinement, solver iteration/tolerance sensitivity, contact compliance sweep, friction sweep, MTP passive sweep, midfoot alternative, placement/pose/anthropometry/initial-pose perturbations, deterministic repeatability | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-09` | Controller phase machine and calibration. | state-causal hybrid phase machine, transition hysteresis/persistence and command continuity | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-10` | Bar placement versus RES-83 collision shells. | post-implementation non-penetration confirmation in standing and all representative poses; any nominal placement change requires an authority amendment | — | `DEFERRED_TO_LATER_AUTHORITY` | S04;S22 |
| `DF-11` | Stance/lateral validation. | true stance width and lateral balance authority | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `DF-12` | Candidate authorization and qualification. | successor candidate creation, provenance/determinism/test/visual qualification | — | `DEFERRED_TO_LATER_AUTHORITY` | S22 |

## Sensitivity and downstream obligations

- **DF-01** — classification and qualification domain frozen in COLLISION_CONTACT_POLICY CC-07..CC-12
- **DF-02** — sealed in candidate identity
- **DF-03** — must be validated against jump/foot mechanics; saturation telemetry mandatory
- **DF-04** — elite isokinetic data are plausibility bounds only; not direct motor constants
- **DF-05** — physical and robustness-calibrated; one-trace/ULP envelopes prohibited
- **DF-06** — report/validation comparator until method-matched +20 kg data exist
- **DF-07** — required before gate authorization
- **DF-08** — each sensitivity run records event ordering, H2, takeoff velocity, GRF, joint-limit margins, prohibited contacts, saturation, landing capture, final standing
- **DF-09** — no wall-clock-only transitions; no instantaneous torque cliffs
- **DF-10** — report-level confirmation; nominal frozen in UB-07
- **DF-11** — outside V1 claim
- **DF-12** — RES-95 creates no candidate

## Notes and locators

- **DF-01** — Provisional settings must be labeled PROVISIONAL_NUMERICAL. (locator: RES-84 / RES-86; sources: S22).
- **DF-02** — Confirmation-only semantics frozen here. (locator: RES-84; sources: S22).
- **DF-03** — Must not become a hidden high-power toe motor. (locator: RES-85; sources: S08;S09).
- **DF-06** — No numeric transfer from drop-landing or unloaded studies. (locator: later method-matched authority; sources: S15).
- **DF-07** — See PA-02/PA-04. (locator: RES-85 / RES-91; sources: S07;S11;S12).
- **DF-08** — Execution of the frozen sensitivity sets is a downstream obligation; omission is a qualification defect. (locator: RES-86 / RES-89 / RES-91; sources: S22).
- **DF-10** — No silent nominal movement. (locator: RES-83 + this authority; sources: S04;S22).
- **DF-11** — 0.170 m remains a model geometry nominal only. (locator: future 3-D/6-DOF Plant program; sources: S22).

## Cross-references

- `COLLISION_CONTACT_POLICY`
- `FOOT_MTP_MODEL_AUTHORITY`
- `PLANT_TOPOLOGY_AUTHORITY`
- `PERFORMANCE_AUTHORITY_BOUNDARY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
