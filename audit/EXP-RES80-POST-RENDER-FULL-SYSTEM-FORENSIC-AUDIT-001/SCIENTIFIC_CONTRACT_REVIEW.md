# SCIENTIFIC_CONTRACT_REVIEW (RES-80)

Method: compare declared scientific words in authority documents and code docstrings against executable
predicates and measured behavior. No contract text was modified.

## 1. Contract-to-implementation mismatches

| Declared statement | Executable reality | Register |
|---|---|---|
| "Genuine flight" requires a 0.010 m geometric gap (GENUINE_FLIGHT_GAP_M) | E7 checks only left/right Fz < 10 N for 0.08 s; the constant is unused | HIGH-010 |
| Apex has a 0.005 s dwell (APEX_DWELL_S) | `DWELL_S["apex"]=0.0`; single-sample vz sign crossing with an unguarded fallback path | HIGH-011 |
| Balance capture requires CoM speed < 0.30 m/s (BALANCE_CAPTURE_COM_SPEED_MPS) | Uses `abs(com_vz)` only | MED-002 |
| Impact absorption means momentum absorption | Predicate is only `abs(com_vz)<0.05` + bilateral Fz | HIGH-001 |
| True standing envelope is a qualified standing neighborhood | Micron-scale windows derived from one deterministic 2 s hold; expanded by 1 ULP | HIGH-012 |
| Support margin is distance to the support polygon | AABB over both full foot boxes, no rotation, inactive feet included, positive in flight | HIGH-008 |
| CoP is reported in the plate frame with validity Fz>20 N | Relative to the moving ankle projection; threshold hardcoded (10 N in constants) | MED-005 |
| `prohibited_contact` detects non-plantar support | Scans only non-colliding geoms (`contype=0`); always False | MED-006 |
| "Scorer runs observationally only" / "never used for control" | Scorer private predicate gates SETTLE->HANDOFF | HIGH-007 |
| "Frozen candidate" with a composition hash | Hash covers 5 files incl. dead code; executed closure unhashed; no runtime check | CRIT-004 |
| "Frozen authorities" loaded from evidence | Loaded unverified at runtime | CRIT-005 |
| Canonical result schema includes hashes and CTRL_MODES | None of the required fields are emitted | HIGH-013 |
| `apex_height` metric | Equals absolute COM z, not jump height (FLIGHT_RISE_FROM_TAKEOFF carries the rise) | MED-011 |
| "Primary loading rate" | `peak / (t_peak - t_landing)`, not a force rise rate / max slope | MED-011 |
| CoP invalid intervals reported | Unreachable/dead code; event samples carry no CoP keys | MED-011 |
| Knee "0 extended, flexion positive" | Positive q moves the ankle anteriorly (reverse-knee); human flexion out of range | CRIT-001 |
| Hip range supports a squat | Only 0.50 rad of hip flexion; the range is extension-biased | CRIT-002 |
| Dwell durations (0.10/0.03/0.01/0.05/0.08/0.02/0.15/0.50 s) | All confirm one physics sample early (systematic -0.000125 s) | MED-001 |
| E12 stable recovery certified | E12 occurrence precedes the RES-43 handoff by 50.8 ms | MED-013 |
| Support continuity qualified | Full-trajectory adjudication is NOT_QUALIFIED; gates use the post-landing slice | MED-012 |

## 2. Words that overstate executable semantics

- **genuine** (flight): force-threshold only; no geometry.
- **balance capture**: vertical-speed latch; no horizontal/angular/posture criterion.
- **impact absorption**: vertical-speed latch only.
- **stable recovery**: micron-envelope identity to one trajectory; 13.15 s recovery from a 24-degree
  forward lean with no posture gate at entry.
- **validated** (soft-contact trust region): validates the finite-difference prediction, not the
  physical desirability; fallback returns the held action; FZ_MIN constraint rows are dropped on
  refinement.
- **frozen candidate/authority**: partial fingerprint plus unverified external JSON.
- **honest** (honest full jump / honest fall / honest planar root): the associated test file is the
  least trustworthy in the suite (10 `assert True`, >=9 accepted as 12/12).

## 3. Quantitative acceptance gaps

The only takeoff-related numerical threshold is `TAKEOFF_VZ_MIN_MPS=0.60` checked once at E6 onset.
There is no gate on: jump height, COM rise, flight duration, geometric foot clearance, ankle/forefoot
work, landing posture, horizontal momentum, CAM at capture, or recovery posture at entry. A trajectory
that latches 12 events with a 3.6 cm hop, a -9.7 rad/s takeoff pitch and a 25-degree forward landing
lean satisfies every executable acceptance predicate.

## 4. Verdict

The scientific vocabulary of the V2.1 authority documents is materially stronger than the executable
predicates. Several named properties are either unimplemented declarations or are verified only in
regimes never exercised by R001. Contract language must be reduced to what is executed, or the
predicates must be implemented before any successor qualification claim.
