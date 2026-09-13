# RES-80 Defect-to-Roadmap Matrix (RES-81)

**THE ORIGINAL RES-80 BUNDLE IS IMMUTABLE.** This matrix is additive; it does not rewrite the historical defect register.

Coverage: 53/53 defects mapped; orphans=0; duplicate primary ownership=0.

| Defect | Severity | Primary | Secondary | Milestone | Title |
|---|---|---|---|---|---|
| CRIT-001 | CRITICAL | RES-83 | RES-84, RES-85, RES-89 | M6 — Human Biomechanics Plant & Measurement Rebuild | Knee hinge direction is anatomically inverted; human knee flexion is outside the joint range |
| CRIT-002 | CRITICAL | RES-83 | RES-85, RES-89 | M6 — Human Biomechanics Plant & Measurement Rebuild | Hip range/convention allocates 1.80 rad to extension and only 0.50 rad to flexion; countermovement is driven into hip extension |
| CRIT-003 | CRITICAL | RES-82 | RES-89 | M5 — Authority Reset & Scientific Contract Closure | No meaningful jump-performance requirement exists; a 3.6 cm hop qualifies as task success |
| CRIT-004 | CRITICAL | RES-88 | RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Candidate identity does not freeze the executing system |
| CRIT-005 | CRITICAL | RES-88 | RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Five external evidence JSON authorities are loaded at runtime without hash verification |
| CRIT-006 | CRITICAL | RES-88 | RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Production package is not self-contained and cannot run from a clean install |
| CRIT-007 | CRITICAL | RES-85 | RES-82, RES-89 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Premature SUPPORTED->FLIGHT switch (single-sample guard) commands a ~0.94 action step while feet are still loaded; violent non-human takeoff |
| HIGH-001 | HIGH | RES-86 | RES-82, RES-87 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | E10 impact-absorption predicate is underconstrained (vertical-only, no horizontal/angular/posture) |
| HIGH-002 | HIGH | RES-86 | RES-82 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | RES-58 terminal capture regulates only vertical momentum; it has no horizontal/angular/CoP authority |
| HIGH-003 | HIGH | RES-87 | RES-82, RES-86 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Balance controller authority is braking-only and cannot command forward CoP/Fx; forward lunge is uncorrectable in principle |
| HIGH-004 | HIGH | RES-87 | RES-82 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | RECOVERY_READY predicate constrains rates only; recovery begins from a 24-degree forward lean with 0.65 rad knee and 15 cm forward CoM |
| HIGH-005 | HIGH | RES-87 | RES-88, RES-82 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | BALANCE->RECOVERY resets the held action to zeros, discarding up to 0.545 and breaking the previous-action continuity nominal |
| HIGH-006 | HIGH | RES-85 | RES-83, RES-82 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Direct torque at 200 Hz with no activation dynamics or action-rate limiting produces non-physiological command steps |
| HIGH-007 | HIGH | RES-88 | RES-82, RES-87 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Scorer-code predicate is a control input (RES-43 handoff gate) despite the declared scorer/controller separation |
| HIGH-008 | HIGH | RES-84 | RES-82, RES-86, RES-87 | M6 — Human Biomechanics Plant & Measurement Rebuild | Support margin does not represent the active support polygon and is positive even in flight |
| HIGH-009 | HIGH | RES-84 | RES-86, RES-87 | M6 — Human Biomechanics Plant & Measurement Rebuild | Inactive-foot geometric gap and normal velocity are wrong when the foot is rotated (up to 0.094 m) |
| HIGH-010 | HIGH | RES-82 | RES-84, RES-89 | M5 — Authority Reset & Scientific Contract Closure | Declared genuine-flight geometric gap is never implemented (E7 checks force only) |
| HIGH-011 | HIGH | RES-82 | RES-84 | M5 — Authority Reset & Scientific Contract Closure | Apex has no dwell and its fallback path can latch without a flight check |
| HIGH-012 | HIGH | RES-87 | RES-82, RES-89 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | True-standing envelope is a one-trajectory overfit (micron-scale windows, one-U LP expansion) |
| HIGH-013 | HIGH | RES-88 | RES-82, RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Canonical output omits all jump-performance metrics and violates its own required result schema |
| HIGH-014 | HIGH | RES-86 | RES-87, RES-85 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Soft-contact FZ-min rows are silently dropped during LS refinement; active-set crossings are not handled in the derivative |
| HIGH-015 | HIGH | RES-89 | RES-88 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Legacy 'honest full jump' acceptance file is false assurance: 10 assert-True stubs and >=9 events accepted as 12/12 |
| HIGH-016 | HIGH | RES-89 | - | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Seven assertions are neutralized by `or True` (unconditional pass) and one test body is a bare pass |
| HIGH-017 | HIGH | RES-89 | RES-88 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | No test executes the canonical composition; the suite validates stored JSON and breaks at collection |
| HIGH-018 | HIGH | RES-89 | - | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Qualification orchestration omits two scripts (one hardcodes PASS); test_public_qualification runs only 10 of 19 suites |
| HIGH-019 | HIGH | RES-83 | RES-85, RES-92 | M6 — Human Biomechanics Plant & Measurement Rebuild | Rigid single-box foot: no MTP/toe/arch; R001 push-off is flat-footed with no heel rise |
| MED-001 | MEDIUM | RES-82 | RES-89 | M5 — Authority Reset & Scientific Contract Closure | Systematic dwell off-by-one: every dwell event confirms one physics sample (0.125 ms) before the declared duration |
| MED-002 | MEDIUM | RES-82 | RES-87 | M5 — Authority Reset & Scientific Contract Closure | E11 threshold is named COM_SPEED but implemented on com_vz only |
| MED-003 | MEDIUM | RES-84 | RES-82 | M6 — Human Biomechanics Plant & Measurement Rebuild | Pelvis orientation observation is a hardcoded identity quaternion |
| MED-004 | MEDIUM | RES-84 | RES-82 | M6 — Human Biomechanics Plant & Measurement Rebuild | Trunk tilt measurement is unsigned |
| MED-005 | MEDIUM | RES-84 | RES-87, RES-92 | M6 — Human Biomechanics Plant & Measurement Rebuild | CoP frame/origin is misdeclared and its validity threshold disagrees with the declared constant |
| MED-006 | MEDIUM | RES-84 | RES-82 | M6 — Human Biomechanics Plant & Measurement Rebuild | prohibited_contact is structurally always False; PROHIB=false in the canonical result carries no information |
| MED-007 | MEDIUM | RES-83 | RES-84 | M6 — Human Biomechanics Plant & Measurement Rebuild | Collision topology permits anatomically impossible intersections and the 20 kg load has no collision geometry at all |
| MED-008 | MEDIUM | RES-85 | RES-82 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | FLEX->EXTEND program switch is a hard scheduled step (0.568) unrelated to state |
| MED-009 | MEDIUM | RES-87 | RES-86 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Post-projection clamps can leave the CoP/Fx/Hdot triple off the feasible line |
| MED-010 | MEDIUM | RES-94 | RES-88 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Legacy controller swallows all exceptions and commands zero action |
| MED-011 | MEDIUM | RES-84 | RES-82, RES-88 | M6 — Human Biomechanics Plant & Measurement Rebuild | Reporting metrics are misdefined or dead: loading rate, apex_height alias, CoP invalid-interval analysis |
| MED-012 | MEDIUM | RES-82 | RES-89, RES-84 | M5 — Authority Reset & Scientific Contract Closure | Full-trajectory support adjudication is NOT_QUALIFIED while reported gates use the post-landing slice only |
| MED-013 | MEDIUM | RES-82 | RES-87, RES-89 | M5 — Authority Reset & Scientific Contract Closure | E12 fires 50.8 ms before the RES-43 handoff; part of its dwell is earned under SETTLE, not the final hold |
| MED-014 | MEDIUM | RES-88 | RES-82 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Horizon authority drift: four coexisting horizons (4.0/8.0/20.0) with no runtime binding |
| MED-015 | MEDIUM | RES-88 | RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Provenance artifacts disagree on entry heads and the authority ledger is stale/corrupted |
| MED-016 | MEDIUM | RES-88 | RES-89 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Production modules hardcode developer absolute paths and inject sys.path; evid_trace_v2 is imported under two module identities |
| MED-017 | MEDIUM | RES-86 | RES-88 | M7 — Launch, Flight, Landing & Recovery Control Rebuild | Production swallows diagnostics and reports sentinel values instead of failing closed |
| MED-018 | MEDIUM | RES-88 | RES-82 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Dwell/threshold literals are duplicated across runtime, mirror module and external JSON with no binding |
| MED-019 | MEDIUM | RES-89 | RES-88 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Multiple V2.1-named tests exercise the legacy controller, not the canonical composition |
| MED-020 | MEDIUM | RES-82 | RES-89 | M5 — Authority Reset & Scientific Contract Closure | E1 supported-start guard omits the fall flag (only the dead prohibited flag is checked) |
| LOW-001 | LOW | RES-94 | RES-83 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Plant XML header documents ngeom=14 while the model has 16 |
| LOW-002 | LOW | RES-94 | - | M8 — Runtime, Provenance & Qualification Infrastructure Closure | README describes 15 bounded anatomical channels while ACTION_DIM=7 |
| LOW-003 | LOW | RES-94 | RES-84 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Stale/incorrect comments and dead branches in contact and support code |
| LOW-004 | LOW | RES-94 | RES-85 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | IMPACT KD 'ramp' is an identity no-op presented as a ramp |
| LOW-005 | LOW | RES-94 | - | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Dead code: unused y_of branch, unused unilateral_dropout_samples with misleading return, dead conditional expression |
| LOW-006 | LOW | RES-94 | RES-88 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | WALL_S makes the canonical result JSON non-byte-deterministic |
| LOW-007 | LOW | RES-94 | RES-83 | M8 — Runtime, Provenance & Qualification Infrastructure Closure | Ankle sign comment is unresolved and geometrically inverted |

## Primary-owner histogram

- RES-82: 8
- RES-83: 4
- RES-84: 7
- RES-85: 3
- RES-86: 4
- RES-87: 5
- RES-88: 9
- RES-89: 5
- RES-94: 8

## Ownership resolutions (multiple claims in Linear descriptions)

- MED-011: candidates ['RES-82', 'RES-84'] -> primary RES-84. Metric definitions (loading-rate semantics, apex-height alias, CoP invalid intervals) are measurement/reporting definitions; RES-84 acceptance explicitly includes metric-definition unit tests. RES-82 remains the reporting-contract authority (secondary).
- HIGH-012: candidates ['RES-82', 'RES-87'] -> primary RES-87. Closure requires a perturbed-hold robustness study; RES-87 acceptance requires the standing/recovery envelope to pass declared perturbation robustness rather than one nominal trace. RES-82 declares the envelope tolerance (secondary).
- MED-013: candidates ['RES-82', 'RES-87'] -> primary RES-82. The defect is the missing declaration of whether E12 must complete under RES-43 or across regimes; that is an event/handoff contract decision. RES-87 implements regime transitions (secondary).
