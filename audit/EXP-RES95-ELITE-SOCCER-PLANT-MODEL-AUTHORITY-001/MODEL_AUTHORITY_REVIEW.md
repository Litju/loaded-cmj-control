# MODEL_AUTHORITY_REVIEW — RES-95 elite-soccer successor Plant model authority

**MISSION:** `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`
**LINEAR:** `RES-95` (active unit; parent RES-83, milestone M6)
**AUTHORITY_ID:** `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`
**STATUS AT REVIEW:** `FROZEN_FOR_RES83_IMPLEMENTATION` after closure of every ranked finding below

---

## 1. Review scope and method

This review covers the RES-95 authority bundle only. It is the closure review for the mission
freeze list (items 1–11) and for the adversarial corrections recorded in the controlling Linear
authority (`LCMJ_ELITE_SOCCER_SYSTEM_AUTHORITY_V1`, updated 2026-09-14).

Three independent verification lines were executed:

1. **Deterministic builder recomputation** — `build_authority_numbers.py` derives every numeric
   value in `DERIVED_QUANTITIES.json` from `AUTHORITY_INPUTS.json`.
2. **Independent validator** — `validate_authority.py` re-implements the numerics in a separate
   code path (different tensor construction and IK expression), checks all rendered artifacts
   against its own recomputation, checks schema/link/taxonomy/evidence-table consistency, runs
   the stale-authority red-team scan, verifies the hash manifest and proves no production path
   changed (gates G1–G9, 239 checks; see `AUTHORITY_VALIDATION_REPORT.json`).
3. **Independent adversarial agents** — a read-only independent verifier performed a full
   authority audit (falsification of frozen values, contradiction hunting, independent grep
   red-team), and a separate numeric replication agent re-implemented the entire numeric model
   from the inputs specification and compared every frozen quantity.

## 2. Independent numeric replication result

The numeric replication agent re-derived all 55 compared quantities from `AUTHORITY_INPUTS.json`
with its own implementation. Result: **REPLICATED** — maximum relative difference 1.3e-16
(double-precision round-off), zero flags above 1e-9, including the full 15-case HAT/bar
sensitivity block. Key replicated values: HAT mass 38.7969 kg; HAT inertia Iyy 1.4753589080582343
kg m²; bar transverse inertia 11.75585696777048 kg m²; system Iyy 1.8608048237640293 kg m²;
foot segment masses 0.4675536/0.4588952/0.1558512 kg; closure delta 0.030905995404939546 m;
sensitivity system Iyy range [1.7316734138069028, 2.0243773264768694] kg m².

## 3. Adversarial findings and dispositions

The independent verifier returned CRITICAL/HIGH/MEDIUM/LOW/NIT findings. All were closed before
sealing. No finding is deferred.

| ID | Severity | Finding | Disposition | Evidence |
|---|---|---|---|---|
| C1 | CRITICAL | `deleva_foot_com_x_from_heel_m` was a mass-moment (0.1314047 m) instead of a position, and AB-15's declared delta was therefore wrong by a factor 1.0823 | CORRECTED to 0.1214125 m; assembled COM 0.11525195 m; delta −0.00616055 m; builder, derived JSON and AB-15 now agree, and the validator gates AB-15 exactly | `DERIVED_QUANTITIES.json` → `foot.*`; `ANTHROPOMETRY_BSIP_AUTHORITY.md` AB-15; validator G2 |
| H1 | HIGH | Bundle was unsealed (no manifest/review/receipt) | CLOSED by `MODEL_AUTHORITY_HASH_MANIFEST.json` + this review + `RES95_RECEIPT.md`; validator G8 verifies every manifest entry | manifest; validator G8 |
| H2 | HIGH | RES-82 `BW = 95.0 kg = 931.95 N` (historical V2 75 kg athlete) conflicted silently with the successor 99.0 kg system | CLOSED by new decision RA-09: RES-82 force thresholds expressed in BW are recomputed on the 99.0 kg successor base; the historical value stays only in RES-82's sealed artifacts | `REFERENCE_ATHLETE_SPEC.md` RA-09 |
| H3 | HIGH | Takeoff-anchor and E-number collision between RES-82 (`E6 PHYSICAL_TAKEOFF`, clearance-gated) and the controlling Linear semantics (`TAKEOFF_OCCURRENCE` = contact loss) | CLOSED by explicit supersession in EM-12 and by removing RES-82 E-number locators from EM-02/03/06/08/09; the clearance-gated concept survives only as `TAKEOFF_CONFIRMATION` and must not shift H2 | `EVENT_MEASUREMENT_BOUNDARY.md` EM-02/03/12 |
| M1 | MEDIUM | `bar.sleeve_span_m` double-counted the collar bridge ([0.655, 1.13]) | CORRECTED to [0.685, 1.10] (grip end 0.655 + bridge 0.03 → sleeve start 0.685; outer end at L/2 = 1.10) | `DERIVED_QUANTITIES.json` → `bar.sleeve_span_m` |
| M2 | MEDIUM | UB-04/UB-05/UB-06/UB-08 were tagged `LITERATURE_DIRECT` although they depend on the engineering bar-hold posture; FM-10 was tagged `LITERATURE_DIRECT` although it is a design contract | RECLASSIFIED: UB-04/05/06/08 → `ENGINEERING_NOMINAL_WITH_SENSITIVITY` with explicit "component masses are literature-direct; aggregate inherits posture" notes; FM-10 → `LITERATURE_INFORMED_SYNTHETIC` | `UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY.md`; `FOOT_MTP_MODEL_AUTHORITY.md` FM-10 |
| M3 | MEDIUM | Thigh/shank had two numeric forms (de Leva-scaled vs 0.2425H/0.2493H) with no single normative value | CLOSED: the stature-ratio forms are normative; de Leva-scaled values are declared cross-checks with deltas 7.9e-6 m / 3.3e-5 m; all dependent quantities (COM, inertia, closure) were recomputed on the normative lengths | `ANTHROPOMETRY_BSIP_AUTHORITY.md` AB-05/AB-06/AB-13; `DERIVED_QUANTITIES.json` |
| M4 | MEDIUM | Validator assurance gaps (AB-15, foot inertia, sleeve span, sensitivity ranges, principal moments unchecked; red-team self-noise) | CLOSED: G2 now checks AB-15, foot COMs/inertias, sleeve span, principal moments, sensitivity min/max, surrogate inertia; the red-team scope is explicitly the authority artifacts (tooling regex literals are not authority text) | `validate_authority.py` |
| M5 | MEDIUM | Comparator threshold defined on total GRF while RES-82 defines a per-foot F_thr (≈20 N total) | CLOSED: EM-10 now freezes the bilateral per-foot 10 N comparator with an optional equivalent total-GRF display; comparator-only status unchanged | `EVENT_MEASUREMENT_BOUNDARY.md` EM-10 |
| M6 | MEDIUM | The controlling Linear document carries the pre-closure status label "ADVERSARIAL RESEARCH CORRECTION OPEN — NOT YET SEALED" | CLOSED at closure: the RES-95 repository bundle + dedicated commit are the declared closure gate (Linear 2026-09-14 comment: "RES-95 must not be moved to Done until the repository-tracked authority bundle, hash manifest, evidence table, receipt and dedicated commit exist"); the Linear status is updated only after the commit | `RES95_RECEIPT.md` §7; Linear RES-95 |
| L1 | LOW | AB-16 sensitivity text (±5.7 kg) did not match the declared set | CLOSED: sensitivity sets are now ±1 SD of the S05 cohort (stature 1.7764/1.835/1.8936 m; mass 72.69/79.0/85.31 kg) and the text states this | `AUTHORITY_INPUTS.json`; AB-16 |
| L2 | LOW | Plantar geometry was footprint-only; sole plane and sub-sole constraints were unfrozen | CLOSED by new decision FM-13: sole plane z = −0.071565 m in the foot frame; all three patches planar on the sole plane; no collision geom may protrude below it; only the frozen patches may create valid support | `FOOT_MTP_MODEL_AUTHORITY.md` FM-13 |
| L3 | LOW | S06 "~0.35 m unloaded CMJ" adjacent to withdrawn-target language | ACCEPTED WITH CLARIFICATION: S06 is declared a report/calibration anchor and explicitly "not a +20 kg direct SYSTEM_COM norm"; the withdrawn values are named in PA-03 only | evidence table S06; `PERFORMANCE_AUTHORITY_BOUNDARY.md` PA-03 |
| L4 | LOW | RA-04 note "Real rigid 20 kg composite bar" could be read as a literal IWF bar | WORDING CORRECTED to "rigid 20 kg composite bar surrogate" | `REFERENCE_ATHLETE_SPEC.md` RA-04 |
| N1 | NIT | Builder internal name swap for the elbow angle | RENAMED; outputs expose `elbow_flexion_deg` (deviation from full extension, 155.045°) and `elbow_interior_angle_deg` (24.955°) | `DERIVED_QUANTITIES.json`; UB-09 |
| N2 | NIT | UB-08 note called Iyy "dominant" although Ixx/Izz are numerically larger | REWORDED: "sagittal-plane relevant inertia is Iyy"; the larger transverse values come from the 2.2 m bar | UB-08 |
| N3 | NIT | Tooling scripts contain stale tokens as detection text | RESOLVED BY SCOPE: the red-team scan covers the authority artifacts; the tooling regex literals are detection patterns, not authority assertions. The independent agent additionally scanned the tooling and found no unmarked issue | `validate_authority.py` G6 note |

## 4. Red-team stale-authority scan

Scope: all 26 rendered authority artifacts, `AUTHORITY_INPUTS.json`, `DERIVED_QUANTITIES.json`,
the evidence table (CSV + MD), this review and the receipt. Every occurrence of a stale pattern
must carry an explicit supersession marker (withdrawn / superseded / not-current / "not" /
never / non-IWF / former / prohibited). Result: **zero unmarked occurrences** for all ten
mission red-team classes:

| Stale class | Result |
|---|---|
| `0.245H` / `0.246H` thigh-shank ratios | absent (0.2425H / 0.2493H frozen; supersession note present) |
| ankle `+20°` dorsiflexion cap | only "former … cap is withdrawn" (JC-06) |
| `NU=7` | absent from the bundle (the 9-channel topology is frozen; PT-04 notes only that the superseded V2 topology is not authority) |
| passive-only MTP | only "NOT sufficient as the sole mechanism" (FM-10) |
| biological `0.170 m` stance | only "NOT an elite-soccer CMJ stance-width claim" (SL-01) |
| 6-DOF / free-root claim | only "NOT a 6-DOF free root" (JC-02, PT-05) |
| scalar-friction-only contact | only "never reduced to one scalar friction coefficient" (CC-05) |
| uniform 2.2 m × 28 mm bar as IWF | only the declared non-IWF sensitivity surrogate (LB-07) |
| `0.200 m` / `0.28–0.40 m` / `0.350 m` elite targets | only PA-03, marked WITHDRAWN |
| clearance-defined delayed H2 takeoff | only EM-13's prohibition and EM-04's "must never replace" |

An additional repo-level scan (informational, not RES-95 scope) found two legacy lines in
`CANONICAL_RUNTIME_AUTHORITY.md` and `CANONICAL_V2_RUNTIME_CONTRACT.md` that describe the
superseded V2 runtime (`NQ=10/NV=10/NU=7`, 95 kg). They describe the historical V2 candidate,
do not conflict with the successor model authority, and their explicit retirement marking is
owned by RES-94; RES-95 did not modify them (scope discipline).

## 5. Validation gates

`AUTHORITY_VALIDATION_REPORT.json` (generated by `validate_authority.py`):

| Gate | Scope | Result |
|---|---|---|
| G1 | reference quantities exact (1.835 m / 79.0 / 20.0 / 99.0 kg) | PASS |
| G2 | independent numeric recomputation + JSON value agreement | PASS |
| G3 | artifact schema, status, taxonomy, cross-file links | PASS |
| G4 | JSON/Markdown agreement incl. critical literals | PASS |
| G5 | evidence table completeness and source resolution (22 sources, 136 decisions) | PASS |
| G6 | red-team stale-authority scan | PASS (0 unmarked) |
| G7 | topology and ROM invariants (NBODY=14, NQ=NV=12, NU=9, ROM span checks) | PASS |
| G8 | hash manifest verification | PASS after sealing |
| G9 | no production/controller/test/experiment path modified | PASS |

## 6. Residual risks and honest limitations

1. **Arm-fold nominal.** The fixed bar-hold posture produces an elbow flexion of 155.0°
   (interior 25.0°) — a geometric consequence of the frozen hand placement and published arm
   lengths. It is an inertial reduction only, explicitly labelled engineering nominal with
   declared sensitivity, and is not a claim about achievable articulated elbow postures (UB-09).
2. **Cross-source closure delta.** The joint-center-resolved chain overshoots stature by
   0.030906 m (1.68 %). This is an inherent landmark-system inconsistency between the de Leva
   chain and the Drillis-Contini ankle height, declared in AB-13; RES-83 must absorb it in the
   standing pose and must not rescale frozen lengths.
3. **Foot model form.** The locked-midfoot nominal remains a material model-form uncertainty;
   the compliant/articulated alternative is mandatory downstream (FM-02/FM-03).
4. **Downstream deferrals.** Contact numerics, actuator limits, recovery envelopes, the exact
   clearance guard, the elite H2 mapping and the full sensitivity campaign are explicitly
   deferred with owners and qualification domains (`DEFERRED_NUMERICAL_CALIBRATIONS.md`).
5. **No predictive claim.** This authority freezes specification only; it establishes no human
   predictive validity, population norm, injury-risk or optimality claim.

## 7. Closure statement

All mission freeze items 1–11 are frozen; every scalar/model-form decision carries exactly one
classification from the required taxonomy; every numeric value is reproducible from
machine-readable inputs to machine precision; the red-team scan has zero unmarked stale
occurrences; the validator passes every gate after sealing; no production, controller, scorer,
test or experiment file was modified. **Zero unresolved current authority conflicts remain.**

---

This review is part of the RES-95 model authority bundle and is included in
`MODEL_AUTHORITY_HASH_MANIFEST.json`.
