# PERFORMANCE_AUTHORITY_BOUNDARY — Performance Authority Boundary

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`  
**LINEAR:** `RES-95` (active unit; parent RES-83)  
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`  
**SPORT_CONTEXT_ID:** `ELITE_MALE_PRO_SOCCER_OUTFIELD_TOP_TIER_EUROPE_V1`  
**MODEL:** `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3`  
**STATUS:** FROZEN_FOR_RES83_IMPLEMENTATION  
**INPUTS:** `AUTHORITY_INPUTS.json` → `DERIVED_QUANTITIES.json` (deterministic; see `build_authority_numbers.py`)  

## Purpose

Frozen performance boundary: anti-triviality floor only, withdrawn elite targets, and the mandatory method-matched mapping.

## Context

No current elite +20 kg direct-H2 target or band is frozen. RES-85/91 must produce a method-matched comparator/mapping analysis before any elite numeric hard gate is authorized.

Controller development maximizes physically valid H2 subject to the frozen constraints instead of tuning to a fabricated height target.

## Frozen decisions

| ID | Decision | Value | Units | Classification | Sources |
|---|---|---|---|---|---|
| `PA-01` | Anti-triviality floor. | `{"H_ANTI_TRIVIALITY_FLOOR_M": 0.15, "meaning": "negative-control / trivial-hop rejection only; necessary but NOT sufficient for elite-context qualification"}` | m | `LITERATURE_INFORMED_SYNTHETIC` | S13 |
| `PA-02` | Elite +20 kg direct H2 target. | `{"ELITE_SOCCER_PLUS20_DIRECT_H2_TARGET": "NOT_FROZEN", "ELITE_SOCCER_PLUS20_DIRECT_H2_HARD_GATE": "DEFERRED_PENDING_METHOD_MATCHED_MAPPING", "H_ANTI_TRIVIALITY_FLOOR_M": 0.15}` | — | `DEFERRED_TO_LATER_AUTHORITY` | S07;S11;S12;S13 |
| `PA-03` | Withdrawn performance authority (superseded, not current). | `{"0.200 m elite-context target": "WITHDRAWN as qualification authority; remains at most a non-authoritative historical notation", "0.280-0.400 m expected band": "WITHDRAWN", "0.350 m nominal target": "WITHDRAWN"}` | m | `DEFERRED_TO_LATER_AUTHORITY` | S07;S11;S12;S13;S14 |
| `PA-04` | Mandatory method-matched comparator analysis. | `{"methods_to_compare": ["direct SYSTEM_COM takeoff->apex", "impulse-momentum", "takeoff-velocity", "flight-time", "bar/LVT displacement"], "requirement": "RES-85/91 predeclare the mapping before any elite-performance hard gate is created"}` | — | `DEFERRED_TO_LATER_AUTHORITY` | S07;S11;S12 |
| `PA-05` | Controller objective. | maximize physically valid direct SYSTEM_COM takeoff->apex H2 subject to the frozen human actuation, energy/work, joint-limit, contact, landing and recovery constraints | — | `LITERATURE_INFORMED_SYNTHETIC` | S16;S22 |
| `PA-06` | Report-only comparators. | `["countermovement depth", "knee/hip/ankle/MTP ROM", "eccentric/braking/propulsive durations", "peak and mean GRF normalized to athlete and system mass", "joint moments/powers/work", "total and joint impulse/work contributions", "peak landing force (method-... | — | `LITERATURE_INFORMED_SYNTHETIC` | S06;S07;S15;S16 |

## Sensitivity and downstream obligations

- **PA-02** — RES-85/91 must produce the mapping before any hard gate
- **PA-04** — owned by RES-85/RES-91
- **PA-06** — report-only until method-matched data exist

## Notes and locators

- **PA-01** — Retained as the only frozen numeric height gate. (locator: RES-82 PF-1 candidate; mixed-athlete +20 kg evidence; sources: S13).
- **PA-02** — No fabricated conversion when uncertainty cannot be bounded. (locator: method heterogeneity across unloaded impulse-momentum, +20 kg flight-time and +20 kg LVT studies; sources: S07;S11;S12;S13).
- **PA-03** — These values must not reappear as current authority in any downstream artifact. (locator: adversarial correction recorded in the Linear system authority; sources: S07;S11;S12;S13;S14).
- **PA-04** — Results far below comparator evidence trigger scientific review, not automatic failure. (locator: system authority section 13; sources: S07;S11;S12).
- **PA-05** — Not tune to a fabricated height target. (locator: system authority section 13; sources: S16;S22).

## Cross-references

- `EVENT_MEASUREMENT_BOUNDARY`
- `DEFERRED_NUMERICAL_CALIBRATIONS`
- `SPORT_CONTEXT_AUTHORITY`

---

This artifact is part of the RES-95 model authority bundle. It freezes specification only: it implements no Plant, controller or scorer code.
