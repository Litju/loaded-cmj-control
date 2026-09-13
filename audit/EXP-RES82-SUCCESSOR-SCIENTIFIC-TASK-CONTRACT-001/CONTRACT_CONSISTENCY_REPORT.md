# CONTRACT_CONSISTENCY_REPORT

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
GENERATED (UTC): 2026-09-13T05:09:33.327661+00:00
CHECK RESULT: **14/14 PASS**

## 1. Mechanical consistency gates

| Check | Name | Status | Detail |
|---|---|---|---|
| C1 | every E1-E12 event exists exactly once | PASS | 12 unique events, 12 unique headings |
| C2 | every event has onset/predecessor/measurements/dwell/blockers/negative controls | PASS | all 12 complete with correct predecessor chain |
| C3 | every hard gate has variable/definition/units/threshold/provenance/invalid-outcome | PASS | 27 gates complete |
| C4 | every numeric threshold has provenance | PASS | all structured thresholds carry provenance and register coverage |
| C5 | no use of 'jump height' without method qualifier | PASS | qualified everywhere |
| C6 | no ambiguous 'takeoff' without physical/force-threshold qualifier | PASS | qualified everywhere |
| C7 | no SPEED variable that uses one velocity component | PASS | COM_SPEED_SAGITTAL only; historical name retired in context |
| C8 | no event predicate depends on a future event | PASS | all predicates causal |
| C9 | no task-success result can be inferred from 12/12 alone | PASS | conjunction rule + explicit non-inference + schema composition pattern |
| C10 | no E12 standing definition depends on equality to one historical trace | PASS | prohibited explicitly |
| C11 | no hard gate silently depends on R001 invalid joint-coordinate signs | PASS | joint conventions deferred to RES-83 (PD-01) |
| C12 | all owner decisions are explicit | PASS | 13 decisions registered; all references resolve |
| C13 | contract JSON and Markdown agree | PASS | key structures agree |
| C14 | no production scientific source file changed | PASS | HEAD/tree equal entry authority; tracked worktree clean; untracked allowlist respected; denylisted paths untouched |

## 2. R001 retrospective under the successor contract

`R001_SUCCESSOR_CONTRACT_RESULT = FAIL`

Independent failure causes: 10

| Criterion | Evidence | Verdict | Owner |
|---|---|---|---|
| L1 MODEL_STATE_VALIDITY (anatomy) | CRIT-001 knee hinge inverted; CRIT-002 hip range extension-biased; HISTORICAL_JOINT_SIGNS_INVALID | FAIL | RES-83 (primary), RES-82 records contract-level dependency |
| E6/E7 physical takeoff and genuine flight semantics | force chatter 0.640-0.64775 s with contact registrations; historical E7 force-only; geometric gap declared but never executed | FAIL | RES-82 (contract), RES-84/85 (implementation) |
| E8 apex flight guard | historical fallback path not guarded by flight; no negative control executable then | FAIL | RES-82 (contract), RES-89 (tests) |
| L3 PRIMARY_PERFORMANCE_GATE (if PF-1 approved) | COM_RISE_TAKEOFF_TO_APEX (true-support-off to apex) = 0.076836 m; E6-to-apex = 0.075158 m; PF-1 = 0.150 m | FAIL | RES-82 (contract), OD-01 pending |
| CG-03 takeoff transition whip | pelvis pitch rate min -9.6837 rad/s at 0.649875 s; candidate R_WHIP 5.0 rad/s | FAIL | OD-06 pending |
| L4 LANDING_VALIDITY (whole-body) | Hy at E10 9.88 kg m^2/s; root pitch 0.2922 rad; trunk forward 25.01 deg; com_vx accelerating 0.211 -> 0.342 m/s after touchdown | FAIL | OD-04 pending |
| L4-T8 behavioral momentum capture | post-touchdown com_vx peak 0.34183 m/s at 1.0385 s exceeds E10 value 0.21145 m/s (forward lunge) | FAIL | RES-82 (contract), RES-86/87 (control) |
| L5 BALANCE_VALIDITY (E11) | historical E11 guard used abs(com_vz); actual com_vx at E11 0.27903 m/s; Hy at E11 UNKNOWN_NOT_EVALUABLE from sealed summary | FAIL / NOT_DEMONSTRATED | OD-05 pending |
| L5 RECOVERY_VALIDITY (E12 envelope + robustness) | one-trajectory ULP envelope; E12 onset 14.43625 s vs handoff 14.487 s (50.8 ms early, under SETTLE); 13.15 s recovery from 24 deg lean | FAIL | RES-87 (primary), RES-82 (contract) |
| FULL_EPISODE support continuity | historical SUPPORT_FULL=NOT_QUALIFIED; chatter transitions 188; canonical reflight runs [4, 8, 1815] | FAIL | RES-89 (implementation), RES-82 (scope contract) |
| Landing peak force / penetration hard gates | peak total Fz 4.7755 BW (<= 8); max penetration 0.0096389 m (<= 0.010) - both within historical hard rules | PASS (for these two lines only) | n/a |
| E1 supported start | historical E1 latched 0.000125 s from reset; fall state UNKNOWN_NOT_EVALUABLE in sealed summary; successor requires explicit fall check (MED-020) | UNKNOWN_NOT_EVALUABLE | RES-89 (tests), RES-82 (semantics) |
| E2-E5 countermovement phase semantics | historical event times exist (E2 0.16125, E3 0.380125, E4 0.4875, E5 0.497375); depth was hip-extension-limited (anatomy invalid) | FAIL under corrected anatomy / otherwise NOT_DEMONSTRATED | RES-83/85 |

Unknowns are marked `UNKNOWN_NOT_EVALUABLE`; no criterion was forced onto a quantity
R001 never recorded.

## 3. Artifact status

- Structural contract: `COMPLETE`
- Numeric closure: `OPEN_OWNER_DECISIONS`
- Owner decisions registered: 13 ({'ACKNOWLEDGMENT_REQUESTED': 1, 'OPEN': 12})
- Negative controls specified: 12

## 4. Reproduction

```
python3 validate_contract.py
```
