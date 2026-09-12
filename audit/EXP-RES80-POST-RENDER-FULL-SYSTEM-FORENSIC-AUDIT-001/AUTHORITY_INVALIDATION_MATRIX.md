# AUTHORITY_INVALIDATION_MATRIX (RES-80)

Classification key: **VALID_UNCHANGED**, **VALID_WITH_LIMITATION**, **SUPERSEDED_FOR_SUCCESSOR**,
**INVALIDATED**, **HISTORICAL_ONLY**, **REQUIRES_REQUALIFICATION**.

Core principle: reproducibility evidence for R001 remains valid even where the scientific claim is
invalidated. Nothing here erases the historical trace, checkpoints, or event records.

| Authority / artifact | Class | Rationale |
|---|---|---|
| Plant XML `v2_plant.xml` | **INVALIDATED** (for human-task claims) / REPRODUCIBILITY_VALID | Frozen and hash-verified; reproduces R001. Knee/hip direction & ranges are anatomically inverted (CRIT-001/002); foot has no MTP/toe (HIGH-019). Cannot support human-technique or loaded-CMJ-realistic claims. |
| Plant hash `5f224414...` | VALID_UNCHANGED | Matches the sealed XML; identifies historical reproducibility only. |
| `plant.py` measurement surface (Fz/force) | VALID_WITH_LIMITATION | Force transform verified against `qfrc_constraint`; support-margin semantics broken (HIGH-008); CoP frame misdeclared (MED-005); `prohibited` dead (MED-006). |
| `measurement.py` synchronized sample authority (RES-54 state staging) | VALID_WITH_LIMITATION | State staging correct; hardcoded pelvis quat (MED-003), unsigned tilt (MED-004), event sample omits CoP. |
| Event scorer `events.py` / `V2EventDetector` | **SCORER_SPEC_DEFECT - REQUIRES_REQUALIFICATION** | E7 gap unused (HIGH-010), apex fallback (HIGH-011), E10/E11 underconstrained (HIGH-001/MED-002), E12 overfit (HIGH-012), dwell off-by-one (MED-001), E1 fall gap (MED-020). R001 event records remain historically valid as computed. |
| C01 landing policy (predictive TD/vdamp lineage) | VALID_WITH_LIMITATION | Reproduces its role in R001; underlies E10 vertical-only behavior and impact Hy growth (HIGH-001/002). |
| RES-52 soft contact (`core52.py`, `soft_contact.py`) | VALID_WITH_LIMITATION | Solver deterministic; FZ_MIN row loss and crossing handling (HIGH-014); rotated-foot geometry (HIGH-009). Requires requalification for successors. |
| RES-54 true COM velocity | VALID_UNCHANGED | Independently re-verified: Jacobian COM velocity matches finite differences and subtree velocity. |
| RES-55 true foot point velocity | **REQUIRES_REQUALIFICATION** | Velocity authority is wrong at rotated feet because the material point is wrong (HIGH-009); flat-foot R001 unaffected. |
| RES-57 support continuity contract | VALID_WITH_LIMITATION | Classification logic consistent; full-trajectory verdict NOT_QUALIFIED while gates use a slice (MED-012); geometric liftoff built on the wrong lowest-point formula (HIGH-009). |
| RES-58 terminal capture seal | **INVALIDATED as sufficient** (vertical-only) | Mathematically reproduced, but has no horizontal/angular/CoP authority (HIGH-002); cannot certify "terminal capture" of a pitching landing. HISTORICAL computation remains valid. |
| RES-73 balance capture | **INVALIDATED as sufficient** | Braking-only, one-sided authority (HIGH-003); CoP projection clamp residual (MED-009); E11 vertical-speed definition (MED-002). |
| RES-74 stable recovery | VALID_WITH_LIMITATION | Reproduces the long recovery; RR entry has no posture gate (HIGH-004); E12 envelope overfit (HIGH-012). Requires requalification for successors. |
| RES-43 true standing | **REQUIRES_REQUALIFICATION** | Envelope is one-trajectory micron-scale (HIGH-012); E12 crosses the handoff boundary (MED-013). REFERENCE values remain valid as a deterministic reference only. |
| RES-76 composition (`full_closure.py`) | **INVALIDATED** (as the executed composition) | Hash-bound but dead: no production module imports it; live logic is inline in `canonical_runtime.py` (CRIT-004/MED-018). |
| RES-78 canonical runtime / identity | **INVALIDATED as a freeze** | Emits results and hashes; does not bind the executing closure; external authorities unverified (CRIT-004/005); output schema incomplete (HIGH-013); identity entry heads fragmented (MED-015). |
| RES-12 canonical qualification (12/12) | **HISTORICAL_ONLY / VALID_WITH_LIMITATION** | Correctly records the declared event and gate outcomes; does not support a task-success claim (CRIT-003) and its performance metrics are absent from the result (HIGH-013). |
| RES-79 render replay | **VALID_UNCHANGED** | Rendering was non-invasive and correctly transmitted the accepted trajectory (P45 confirmed); visual review exposed model/control defects, not renderer defects. |
| R001 candidate identity | **HISTORICAL_ONLY** | `V2.1-R001` identifies the historical candidate; the identity is not enforceable against the executing system (CRIT-004). |
| R001 TRACE `4d047879...` | **VALID_UNCHANGED (reproducibility)** | Bit-exact reproducible; historical evidence preserved. |
| R001 event records | **VALID_WITH_LIMITATION (historical)** | Correct outputs of the implemented predicates; predicate semantics themselves are defective. |
| R001 checkpoints | **VALID_UNCHANGED (historical)** | Sealed vectors/times reproduce. |
| R001 visual artifacts | **VALID_UNCHANGED (historical)** | Accurate render of the accepted trajectory; owner adjudication stands. |
| V2.1 tests | **REQUIRES_REQUALIFICATION** | Placeholders/neutralized assertions/no live canonical test (HIGH-015..019). |
| Packaging/dependency authority | **INVALIDATED** | Clean install cannot run; scipy undeclared; tools excluded (CRIT-006). |
| Claim boundary documents | **REQUIRES_REQUALIFICATION** | Contain claims stronger than executable semantics (SCIENTIFIC_CONTRACT_REVIEW, CLAIM_REVIEW). |
| RES-13 (parent) | **BLOCKED** | Remains blocked by RES-80; this audit does not unblock it. |

## Rules for the successor

1. Reuse R001 evidence only as reproducibility history, never as a physical-performance precedent.
2. Do not patch around CRIT-001/002: any knee/hip sign or range change alters the Plant hash and
   invalidates all controller/event constants fitted to it; plan a full requalification.
3. Any authority whose row is INVALIDATED or REQUIRES_REQUALIFICATION must be re-derived (not merely
   retested) before it is cited by a successor candidate.
4. Keep the historical bundles immutable; successor evidence goes to a new experiment ID.
