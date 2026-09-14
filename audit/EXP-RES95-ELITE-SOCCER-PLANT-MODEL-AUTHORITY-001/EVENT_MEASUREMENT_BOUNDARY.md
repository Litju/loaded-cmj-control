# EVENT_MEASUREMENT_BOUNDARY — Event and Measurement Boundary

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen event-time semantics, primary COM authority, H2 anchoring and measurement-comparator boundaries.

## Context

Physical geometry/contact state is authoritative. Force thresholds are measurement comparators, never the truth source for takeoff or flight.

TAKEOFF_OCCURRENCE and TAKEOFF_CONFIRMATION are separate; confirmation must never shift the H2 time origin.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `EM-01` | Primary COM signal. | SYSTEM_COM (athlete + rigid 20 kg bar) = primary mechanics authority; ATHLETE_COM = secondary report/comparability signal | — | `PHYSICS_IDENTITY` | — |
| `EM-02` | Primary takeoff time. | TAKEOFF_OCCURRENCE = the final transition from at least one active legal plantar support contact to zero active legal plantar support contacts, with interpolated/contact-event time if available | — | `PHYSICS_IDENTITY` | S17 |
| `EM-03` | Takeoff confirmation. | TAKEOFF_CONFIRMATION = a later fail-closed confirmation that bilateral geometric clearance exceeds the numerical guard, SYSTEM_COM_vz(TAKEOFF_OCCURRENCE) > 0, no prohibited contact exists, and no legal plantar recontact occurs throughout the genuine-flight ... | — | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S17 |
| `EM-04` | Confirmation must not shift the takeoff timestamp. | TAKEOFF_CONFIRMATION must never replace TAKEOFF_OCCURRENCE as the H2 time origin | — | `PHYSICS_IDENTITY` | — |
| `EM-05` | Primary jump-height metric. | H2 = SYSTEM_COM_z(apex) - SYSTEM_COM_z(TAKEOFF_OCCURRENCE) | m | `PHYSICS_IDENTITY` | — |
| `EM-06` | Genuine flight dwell. | starting from the candidate TAKEOFF_OCCURRENCE, zero legal plantar contacts must persist for at least 0.050 s; within the confirmation window both feet establish positive clearance above the numerical guard and no prohibited contact occurs | s | `ENGINEERING_NOMINAL_WITH_SENSITIVITY` | S17 |
| `EM-07` | Clearance guard. | clearance guard = max(2 mm, contact_margin + verified numerical penetration/error allowance); exact evaluated value sealed by RES-84 | m | `DEFERRED_TO_LATER_AUTHORITY` | S22 |
| `EM-08` | Apex definition. | inside genuine flight, SYSTEM_COM_vz crosses positive -> nonpositive; compute the zero crossing by interpolation; no arbitrary apex dwell | — | `PHYSICS_IDENTITY` | — |
| `EM-09` | Touchdown definition. | first renewed valid plantar-floor contact after genuine flight; toe/forefoot-first is valid; body/bar-floor contact is not | — | `PHYSICS_IDENTITY` | S15 |
| `EM-10` | Measurement takeoff comparator. | bilateral per-foot vertical GRF threshold 10 N with 0.010 s persistence (the RES-82 F_thr convention; an equivalent total-GRF display may be reported) is comparator/diagnostic only and never defines the simulated physical event | N / s | `LITERATURE_INFORMED_SYNTHETIC` | S17 |
| `EM-11` | Force-plate comparison export. | resampled at >=1000 Hz (1000 Hz canonical) with a documented anti-aliasing/filter path; raw native signals retained for independent recomputation | Hz | `LITERATURE_INFORMED_SYNTHETIC` | S11 |
| `EM-12` | Event chain and dwell semantics (with explicit takeoff supersession). | `{"retained_from_RES82": "E1-E12 chain, PHYSICAL_TIME dwell convention, negative controls, cross-event independence", "superseded_for_successor": "the clearance-gated historical takeoff anchor; for the successor the H2 time origin is TAKEOFF_OCCURRENCE (fin... | — | `LITERATURE_INFORMED_SYNTHETIC` | S22 |
| `EM-13` | No clearance-defined delayed H2 takeoff. | the H2 time origin is the last legal plantar contact loss; a later clearance-defined sample must never become the H2 origin | — | `PHYSICS_IDENTITY` | — |

## Sensitivity and downstream obligations

- **EM-03** — dwell and clearance guard sensitivity required downstream
- **EM-06** — dwell sensitivity (e.g. 0.030/0.050/0.080 s) required downstream
- **EM-07** — RES-84 seals exact value; guard participates in confirmation only
- **EM-10** — comparator threshold sensitivity is a reporting obligation
- **EM-11** — processing sensitivity is a reporting obligation
- **EM-12** — downstream sensitivity execution owned by later RES

## Notes and locators

- **EM-01** — All event predicates use SYSTEM_COM. (locator: carried-bar closure; sources: —).
- **EM-02** — This is the physical takeoff time used for H2 and takeoff velocity. (locator: Linear system authority section 12, physical takeoff (no RES-82 E-number label); sources: S17).
- **EM-03** — If confirmation fails the candidate takeoff occurrence is rejected, never shifted. (locator: Linear system authority section 12, takeoff occurrence/confirmation split; sources: S17).
- **EM-04** — Prevents a delayed-clearance threshold from artificially lowering H2. (locator: mission freeze item 10; system authority section 13; sources: —).
- **EM-05** — Mandatory cross-checks: H_ballistic = vz_takeoff^2 / (2g); H_impulse from total vertical impulse / 99 kg. (locator: direct last-legal-contact-loss to apex displacement; sources: —).
- **EM-06** — The 50 ms dwell is an engineering debounce, not a measured human constant. (locator: Linear system authority section 12, genuine flight; sources: S17).
- **EM-07** — The clearance guard is a confirmation tolerance, not the definition of the takeoff instant. (locator: system authority section 12; sources: S22).
- **EM-09** — Landing absorption must be realized by the calibrated contact model and human joint motion, not a hidden vertical clamp. (locator: Linear system authority section 12, physical touchdown; sources: S15).
- **EM-11** — Native simulator states/contacts remain event authority. (locator: force-platform processing literature; sources: S11).
- **EM-12** — This supersession is explicit so zero unresolved conflict remains between RES-82 labels and the controlling Linear takeoff semantics. (locator: RES-82 EVENT_CONTRACT_E1_E12.json; DWELL_SEMANTICS.json; Linear system authority section 12; sources: S22).
- **EM-13** — This closes the historical delayed-takeoff defect class. (locator: mission freeze item 10; sources: —).

## Cross-references

- `PERFORMANCE_AUTHORITY_BOUNDARY`
- `COLLISION_CONTACT_POLICY`
- `PLANT_TOPOLOGY_AUTHORITY`
- `RES-82 EVENT_CONTRACT_E1_E12.json`
- `RES-82 DWELL_SEMANTICS.json`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
