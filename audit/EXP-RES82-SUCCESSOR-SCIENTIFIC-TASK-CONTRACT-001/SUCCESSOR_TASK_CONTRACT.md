# SUCCESSOR_TASK_CONTRACT — Nominal 20 kg Loaded Countermovement Jump

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
LINEAR_ISSUE: `RES-82`
STATUS: **STRUCTURALLY FROZEN** (open numeric items are enumerated in
`OWNER_DECISIONS_REQUIRED.md`; model-dependent items in `MODEL_DEPENDENCY_MATRIX.json`)
ENTRY_HEAD: `12ea42029489ddc2839389beafcf0811b7b45271`
ENTRY_TREE: `79f183724486ff95a1269a87cdb21d078b1893a5`

This is the top-level successor scientific success contract. It is **not** a Plant,
controller, scorer, test, or optimization artifact, and it changes no production
scientific code.

---

## 0. Core distinction

```
EVENT_CHAIN_VALID  ≠  TASK_SUCCESS
```

A trajectory may latch all twelve event labels (E1–E12) and still fail the task. R001
is the canonical negative example: 12/12 event labels under the historical system,
owner-observed physical-visual credibility FAIL, ship BLOCKED. The successor contract
is constructed so that event completion can never imply success.

## 1. Success architecture (layers)

Success is an **independent-layer** structure; a single Boolean "12/12 PASS" is
retired.

| Layer | Name | Question | Owner |
|---|---|---|---|
| `L0` | `EXECUTION_VALIDITY` | Did the canonical runtime execute the episode to a valid completion (no fault, sufficient horizon, deterministic identity)? | RES-88 (runtime), contract-defined here |
| `L1` | `MODEL_STATE_VALIDITY` | Is every state physically admissible (finite, no fall, no prohibited contact, penetration bounded, joint/actuator limits respected, no artificial support)? | this contract |
| `L2` | `EVENT_SEQUENCE_VALIDITY` | Did E1–E12 occur in the declared monotone order with the declared physical semantics and dwells? | this contract |
| `L3` | `TASK_PERFORMANCE_VALIDITY` | Did the jump meet the primary performance floor and the physics consistency checks? | this contract |
| `L4` | `LANDING_CAPTURE_VALIDITY` | Was the landing a whole-body admissible capture (not a vertical arrest / lunge)? | this contract |
| `L5` | `RECOVERY_VALIDITY` | Were balance captured and the robust standing envelope achieved? | this contract + RES-87 calibration |
| `L6` | `CANDIDATE_CREDIBILITY` | Provenance, determinism, test validity, owner visual review | RES-88/RES-89/RES-91 + owner |

### 1.1 Success composition (normative)

```
TASK_SUCCESS =
    EXECUTION_VALID
AND MODEL_STATE_VALID
AND EVENT_CHAIN_VALID
AND TASK_PERFORMANCE_VALID
AND LANDING_VALID
AND BALANCE_VALID
AND RECOVERY_VALID
AND NO_HARD_FAILURE
```

Candidate qualification later adds:

```
AND PROVENANCE_VALID
AND DETERMINISM_VALID
AND TEST_VALID
AND OWNER_VISUAL_VALID
```

RES-82 owns the scientific task portion (L1–L5 and the L2 semantics); L0 and L6 are
declared here and implemented by later RES. `EVENT_CHAIN_VALID=true` never implies any
other component.

### 1.2 Mandatory machine-readable result

```json
{
  "EXECUTION_VALID": null,
  "MODEL_STATE_VALID": null,
  "EVENT_CHAIN_VALID": null,
  "TASK_PERFORMANCE_VALID": null,
  "LANDING_VALID": null,
  "BALANCE_VALID": null,
  "RECOVERY_VALID": null,
  "NO_HARD_FAILURE": null,
  "TASK_SUCCESS": null
}
```

`TASK_SUCCESS` is `true` only when every mandatory component is `true`. A component
whose upstream owner decision is open (OD-01..OD-12 affected gates) is reported
`false`/unresolved, never inferred.

## 2. Terminology freeze (normative definitions)

The following terms have exactly one definition. No term may be used for two
quantities. Where a term had a historical ambiguous usage, the retirement is stated.

| Term | Definition |
|---|---|
| `INITIAL_STANDING_COM_Z` | COM world `z` at the E1 confirmation reference sample, i.e., the quiet-standing reference height |
| `COUNTERMOVEMENT_MIN_COM_Z` | minimum COM world `z` reached after E1 and before physical takeoff |
| `COUNTERMOVEMENT_DEPTH` | `INITIAL_STANDING_COM_Z − COUNTERMOVEMENT_MIN_COM_Z` |
| `UPWARD_REVERSAL` | the supported transition of COM vertical velocity from negative to positive (E4) |
| `PROPULSION` | sustained net-positive upward COM propulsion under bilateral support (E5 window `[E5_onset, E6)`) |
| `FORCE_THRESHOLD_TAKEOFF` | the measurement observable: first sample with bilateral `Fz < F_thr` sustained `0.010 s`; comparability only, NEVER physical truth |
| `FIRST_TRANSIENT_FORCE_DROPOUT` | first sample with transient bilateral force below `F_thr` regardless of contact (R001 defect evidence; diagnostic) |
| `SUSTAINED_FORCE_OFF` | first sample of the final sustained bilateral below-`F_thr` run not interrupted by above-threshold force before landing (diagnostic) |
| `PHYSICAL_TAKEOFF` | first sample with both foot contact sets empty, positive geometric clearance both feet, upward COM velocity, and no non-plantar support (E6) |
| `GENUINE_FLIGHT` | sustained physical no-contact + positive clearance for both feet for the declared dwell (E7); executable geometry, not a force-only condition |
| `PHYSICAL_FLIGHT_INTERVAL` | `[PHYSICAL_TAKEOFF, descending-landing first contact]`; must contain no foot-floor contact rows |
| `APEX` | deterministic COM `vz` positive-to-non-positive crossing strictly inside the physical flight interval (E8); no dwell |
| `COM_APEX_Z` | COM world `z` at the apex (absolute coordinate; never called a height) |
| `COM_APEX_ABOVE_INITIAL_STANDING` | `COM_APEX_Z − INITIAL_STANDING_COM_Z` (context metric, not the primary jump height) |
| `COM_RISE_TAKEOFF_TO_APEX` | `COM_APEX_Z − TAKEOFF_COM_Z` = **PRIMARY_CANONICAL_JUMP_HEIGHT** |
| `TAKEOFF_COM_Z` | COM world `z` at physical takeoff [m]; the reference of `COM_RISE_TAKEOFF_TO_APEX` |
| `TAKEOFF_COM_VZ` | COM vertical velocity at physical takeoff |
| `BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ` | `TAKEOFF_COM_VZ² / (2g)` (cross-check) |
| `FLIGHT_TIME_FORCE_DEFINED` | force-threshold landing time minus force-threshold takeoff time (comparability only) |
| `FLIGHT_TIME_PHYSICAL` | `LANDING_FIRST_CONTACT_TIME − PHYSICAL_TAKEOFF_TIME` |
| `LEFT_FOOT_CLEARANCE` / `RIGHT_FOOT_CLEARANCE` | instantaneous signed minimum distance of the foot's lowest geometry point to the floor; `> 0` separated |
| `LANDING_FIRST_CONTACT` | first physical floor contact after flight with a contact row and `foot_clearance <= 0` |
| `BILATERAL_LANDING` | both feet establish plantar contact with load within the declared establishment dwell `D_BL` |
| `IMPACT_ABSORPTION` | E10: bilateral-support run with residual vertical COM speed below `V_ABS_TAIL` and an admissible L4 whole-body state |
| `BALANCE_CAPTURE` | E11: sustained whole-body captured state (sagittal COM speed, `Hy`, support margin, CoP validity, bilateral support) |
| `RECOVERY_READY` | reported state between capture and stable standing: captured and in a recovery-entry set from which standing recovery can begin without an extreme corrective maneuver |
| `STABLE_STANDING` | E12: robust physical standing-envelope membership (posture, velocity, `Hy`, support, CoP, force, no fall/reflight) sustained for `0.500 s`; the envelope is RES-87-calibrated from perturbed holds |
| `FALL` | a fall-shell collision geom contacts/loads the floor (physical fall) |
| `PROHIBITED_CONTACT` | support by any non-plantar, non-fall geom (load, torso, limb shells, or any artificial support) |
| `REFLIGHT` | force-only canonical de-chatter rule (RES-57): whole `Fz < 10.0 N` for `N_true ≥ 4` consecutive samples (**declared SAMPLE_COUNT semantics**; span 0.375 ms; historical 0.5 ms wording is an erratum); no separation evidence required; window-bound after E9 confirmation. Chatter transitions are separate |
| `SUPPORT_CONTINUITY` | scoped support-episode quality per `SUPPORT_CONTINUITY_SCOPE_ID`; results are `QUALIFIED`/`NOT_QUALIFIED`/`NOT_EVALUABLE`; the phrase "support continuity PASS" is prohibited |
| `BW` / `SYSTEM_WEIGHT` | `m_system · g = 95.0 kg · 9.81 m/s² = 931.95 N`; the single normative body-weight quantity (athlete + load); never athlete-only |
| `g` | 9.81 m/s² (project V2 constant, identical to the Plant gravity) |
| `cop_valid` | CoP valid under the declared per-foot validity threshold; per-foot validities and resultant CoP reported; frame per RES-84 |

**Retired conflations:** `apex_height` as an absolute coordinate called a height;
unqualified "jump height"; a vertical component called "COM speed"; "takeoff" without
`PHYSICAL_`/`FORCE_THRESHOLD_` qualifier; slice support results presented as
full-episode qualification. `0.0363 m` and `0.0752 m` may never share one label.

## 3. Event chain (summary; normative detail in `EVENT_CONTRACT_E1_E12.md`)

| ID | Name | Dwell |
|---|---|---|
| E1 | supported_start | PHYSICAL_TIME 0.100 s |
| E2 | countermovement_onset | PHYSICAL_TIME 0.030 s |
| E3 | valid_countermovement | NONE |
| E4 | upward_reversal | PHYSICAL_TIME 0.010 s |
| E5 | vertical_propulsion | PHYSICAL_TIME 0.050 s |
| E6 | physical_takeoff | NONE |
| E7 | genuine_flight | PHYSICAL_TIME 0.050 s (OD-03) |
| E8 | apex | NONE |
| E9 | descending_landing | PHYSICAL_TIME 0.010 s |
| E10 | impact_absorption | PHYSICAL_TIME 0.020 s |
| E11 | balance_capture | PHYSICAL_TIME 0.150 s |
| E12 | stable_recovery | PHYSICAL_TIME 0.500 s |

Every event has a declared onset predicate, predecessor, required measurements,
failure blockers, negative controls, primary/diagnostic class, and claim statement.
No event predicate depends on a future event.

## 4. Performance contract (summary; detail in `PERFORMANCE_METRIC_CONTRACT.md`)

- **Primary canonical jump height:** `COM_RISE_TAKEOFF_TO_APEX` (direct model-truth
  displacement), with `BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ` as the physics cross-check,
  `FLIGHT_TIME_HEIGHT` as an experimental comparability metric, and
  `COM_APEX_ABOVE_INITIAL_STANDING` as context.
- **One** primary magnitude gate: `COM_RISE_TAKEOFF_TO_APEX >= H_MIN` (OD-01;
  candidate PF-1 = 0.150 m recommended).
- **Physics consistency:** in-flight vertical COM acceleration equals `−g` within
  numerical tolerance; ballistic residual bounded (model-dependent).
- **Minimum physical-flight validity:** E7 dwell; apex inside flight; no contact in
  the interval; no force-threshold participation.
- **Anti-whip and extension gates:** `CG-03`, `CG-04` (model-dependent bounds).
- Ballistically related quantities are not triple-gated.

## 5. Landing/balance/recovery contract (summary; detail in
`LANDING_BALANCE_RECOVERY_CONTRACT.md`)

- L4 is a whole-body post-impact admissible state; vertical arrest alone is
  insufficient. Point-in-time bounds are evaluated at E10 confirmation; the behavioral
  anti-lunge window is explicitly `[E10_confirmation, E11_onset]` (`L4-T8`), the
  impact-interval horizontal bound is `L4-T9`, and the landing transient-window maxima
  are `L4-T10`.
- Anti-lunge behavioral gate: post-E10 forward momentum must be captured, not
  amplified (R001's `com_vx` 0.211 → 0.342 m/s is excluded); if E11 never onsets,
  `L4-T8` is `NOT_EVALUABLE` and `LANDING_VALID=false`.
- L5 capture uses `com_speed_sagittal = sqrt(vx²+vz²)`, `Hy`, support margin, CoP
  validity, bilateral support.
- `RECOVERY_VALID` additionally requires recovery maintenance over
  `[E11_confirmation, E12_confirmation]` (E11 bounds hold at every sample),
  `FULL_EPISODE = QUALIFIED`, and the post-E12 observation margin (candidate 0.250 s;
  OD-13); state overrides/freezes fail `NO_ARTIFICIAL_SUPPORT`.
- E12 standing envelope is physical and robustness-calibrated (RES-87); one-trace/ULP
  envelopes are prohibited; eventual success cannot retroactively validate a bad
  L4/L5 state.

## 6. Dwell semantics (summary; normative in `DWELL_SEMANTICS.md`)

`PHYSICAL_TIME` on the physics grid: an event first true at sample `i` and
continuously true through sample `j` confirms when `K = j − i >= K_D = ceil(D/dt)`;
`occurred_at = t[i]`; `confirmed_at = t[i + K_D]`; `confirmed_at − occurred_at >= D`.
The historical one-sample-early convention is explicitly retired (MED-001).

## 7. Support-continuity scopes (summary)

`SUPPORT_CONTINUITY_SCOPE_ID` ∈ {`SUPPORTED_PHASE`, `PROPULSION`,
`TAKEOFF_TRANSITION`, `FLIGHT`, `LANDING_CAPTURE`, `RECOVERY`, `FULL_EPISODE`}.
`TASK_SUCCESS` requires `FULL_EPISODE = QUALIFIED`; `RECOVERY_VALID` is false unless
`FULL_EPISODE = QUALIFIED`; if a closing event does not confirm, the window closes at
the episode horizon and the scope result is `NOT_EVALUABLE` (never silently
`QUALIFIED`). Slice results are diagnostics (MED-012). The top-level `FULL_EPISODE`
field must agree with the `FULL_EPISODE` entry in the scope results.

## 8. Regime/handoff semantics (summary)

E12 is a physical-state event, independent of controller mode; `CONTROLLER_HANDOFF_TIME`
and `CONTROLLER_REGIME_AT_E12_ONSET` are separately reported. No event acquires
meaning from the mode timeline (MED-013).

## 9. Metric classes and threshold provenance

Every canonical metric carries exactly one class: `HARD_GATE`, `CONSISTENCY_GATE`,
`DIAGNOSTIC`, `REPORT_ONLY`. Every numeric threshold carries exactly one provenance
class:

| Provenance class | Meaning |
|---|---|
| `LITERATURE_DIRECT` | value taken from a source row with `DIRECTLY_SUPPORTS_NUMERIC_GATE=YES` |
| `LITERATURE_INFORMED` | structure/scale informed by literature; value is a task decision with stated margin |
| `PHYSICS_IDENTITY` | exact physical relation (e.g., `a = −g` in flight) |
| `NUMERICAL_TOLERANCE` | bound set by solver/integration numerics |
| `OWNER_TASK_DECISION` | task definition chosen by the owner (option list provided) |
| `MODEL_DEPENDENT_DEFERRED` | requires RES-83/RES-84/RES-87 model work before a value exists |

No arbitrary engineering value is presented as a literature value.

## 10. Required metric retention

The canonical output must retain every metric listed in `CANONICAL_OUTPUT_SCHEMA.json`
(section 27 of the mission), including the whole-body landing state, the
`COM_SPEED_SAGITTAL` at E11, `Hy`, support margin, CoP validity, recovery-ready time,
stable-hold duration, reflight/fall/prohibited flags, and the task-success components.
No important performance variable may be computed and then discarded (HIGH-013).

## 11. Negative controls

NC-01..NC-12 are frozen in `NEGATIVE_CONTROL_CONTRACT.md`; executable implementation
is a RES-89/RES-91 obligation. NC-11 closes the moving-start hybrid and NC-12 the
freeze/short-horizon recovery exploit. A contract that passes any negative control is not
accepted. The adversarial review must attempt a physically bad trajectory that
satisfies the contract.

## 12. R001 retrospective

`R001_SUCCESSOR_CONTRACT_RESULT = FAIL` with multiple independent causes:
performance floor (if approved), physical takeoff/flight transition semantics, model
state (anatomical, owned by RES-83), landing whole-body state, balance capture
(corrected COM speed), recovery envelope robustness, and full-episode support
continuity. Missing historical quantities are marked `UNKNOWN_NOT_EVALUABLE`; the
retrospective does not force criteria onto quantities R001 never recorded.

## 13. Model dependency

Plant-independent requirements are frozen here. Plant-dependent numerics are listed in
`MODEL_DEPENDENCY_MATRIX.json` and must not be silently filled. A successor Plant
(RES-83) rebuild triggers COU re-adjudication per `SUCCESSOR_CONTEXT_OF_USE.md` §10.

## 14. Claim ceiling and V&V

A PASS supports only the simulation-qualification claim in
`SCIENTIFIC_CLAIM_CEILING.md`; it never establishes human predictive validity,
injury-risk, optimality, or population norms. `verification ≠ validation`;
`determinism ≠ biomechanical validity`; `event completion ≠ task success`;
`visual plausibility ≠ experimental validation`.

## 15. Consistency obligations (mechanical)

`validate_contract.py` mechanically enforces:

1. E1–E12 exist exactly once.
2. Every event has onset, predecessor, measurements, dwell semantics, blockers,
   negative controls.
3. Every hard gate has variable, definition, units, threshold/status, provenance,
   invalid outcome prevented.
4. Every numeric threshold has provenance.
5. No unqualified "jump height".
6. No unqualified "takeoff" where physical/force ambiguity exists.
7. No variable named `SPEED` that uses only one velocity component.
8. No event predicate depends on a future event.
9. No task-success result can be inferred from 12/12 alone.
10. No E12 definition depends on equality to one historical trajectory.
11. No hard gate silently depends on R001's invalid joint-coordinate signs.
12. All owner decisions are explicit.
13. Contract JSON and Markdown agree.
14. No production scientific source file changed.

## 16. Status

The structural contract is complete. The contract is **not** fully numerically
closed: 12 owner/model decisions remain open by design (`OWNER_DECISIONS_REQUIRED.md`).
Unresolved numeric choices are explicitly owner decisions, not hidden assumptions.
