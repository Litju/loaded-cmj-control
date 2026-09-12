# AUDIT_RECEIPT (RES-80)

MISSION=V2_1_POST_RENDER_FULL_SYSTEM_FORENSIC_AUDIT_001
LINEAR_ISSUE=RES-80
MODE=READ_ONLY_FULL_SYSTEM_FORENSIC_AUDIT

STATUS=COMPLETE_AUDIT_SEALED
FAILURE_CLASS=MODEL_FORM_AND_CONTROL_INVALIDITY_WITH_PARTIAL_PROVENANCE_FREEZE

ENTRY_HEAD=8f26736db1231042cbedc61f61ca4862b7be871c
ENTRY_TREE=855fe3b418c09b5028396adeaed3ba8cda51e89f
ENTRY_COMMIT_MESSAGE=V2.1: add accepted trajectory visual smoke replay
TRACKED_WORKTREE_AT_ENTRY=CLEAN
PRESERVED_UNTRACKED_FILES=see AUDIT_REPOSITORY_INVENTORY.json (20 pre-audit WIP entries, preserved)

CANDIDATE_ID=V2.1-R001
R001_TRACE_SHA256=4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561
R001_HISTORICAL_REPRODUCIBILITY=PASS
R001_PHYSICAL_VISUAL_CREDIBILITY=FAIL

## Baseline verification

| Hash | Declared | Recomputed | Result |
|---|---|---|---|
| PLANT_SHA256 | 5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191 | same | MATCH |
| CONTROLLER_COMPOSITION_SHA256 | 6f56ffa180b12c128d67ea13fd9c08413554a9d964d175615f16a8e572185d12 | same | MATCH |
| SCORER_SHA256 | 286ef328e4b334a15775df01ba6aad971cf8f808ddbcb028fcda4032164f2deb | same | MATCH |
| CANDIDATE_SPEC_SHA256 | b02c74b8af547692b6544790b93c87b16a86c0ac04c76abeac47aaf2b9972e0c | same | MATCH |
| RUNTIME_CONTRACT_SHA256 | cafb91fe838b68c8110157659d35f6fae561295e98409fc0b6bc94375d933c1e | same | MATCH |
| R001 TRACE_SHA256 | 4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561 | present in sealed artifacts | MATCH |

## Scope

REPOSITORY_FILES_TOTAL=186 tracked (+ untracked WIP preserved)
REPOSITORY_FILES_AUDITED=133
REPOSITORY_FILES_OUT_OF_SCOPE=53 (legacy V1/F4/Gen1 subsystems; reasons in SOURCE_REVIEW_COVERAGE.json)
TRACKED_LINES_TOTAL=56854; AUDITED_LINES=33335

## Defect counts

DEFECT_TOTAL=53

CRITICAL_COUNT=7
CRITICAL_IDS=CRIT-001, CRIT-002, CRIT-003, CRIT-004, CRIT-005, CRIT-006, CRIT-007

HIGH_COUNT=19
HIGH_IDS=HIGH-001, HIGH-002, HIGH-003, HIGH-004, HIGH-005, HIGH-006, HIGH-007, HIGH-008, HIGH-009, HIGH-010, HIGH-011, HIGH-012, HIGH-013, HIGH-014, HIGH-015, HIGH-016, HIGH-017, HIGH-018, HIGH-019

MEDIUM_COUNT=20
MEDIUM_IDS=MED-001, MED-002, MED-003, MED-004, MED-005, MED-006, MED-007, MED-008, MED-009, MED-010, MED-011, MED-012, MED-013, MED-014, MED-015, MED-016, MED-017, MED-018, MED-019, MED-020

LOW_COUNT=7
LOW_IDS=LOW-001, LOW-002, LOW-003, LOW-004, LOW-005, LOW-006, LOW-007

DEFECT_COUNTS_BY_TYPE (multi-label):
SOFTWARE_BUG=16; CONTROL_BUG=10; MODEL_FORM_DEFECT=6; MEASUREMENT_BUG=7; SCORER_SPEC_DEFECT=10;
QUALIFICATION_DEFECT=13; PROVENANCE_DEFECT=8; TEST_DEFECT=5; DOCUMENTATION_DEFECT=14

PREFLIGHT_FINDINGS_CONFIRMED=44
PREFLIGHT_FINDINGS_REJECTED=1 (P31 contact-frame ordering: implementation correct; comments wrong -> LOW-003)
PREFLIGHT_FINDINGS_LIMITATIONS=0
PREFLIGHT_FINDINGS_NEED_MORE_EVIDENCE=0
NEW_FINDINGS_BEYOND_PREFLIGHT=17 (see PROVISIONAL_FINDING_ADJUDICATION.json)

## R001 measured performance (deterministic recomputation)

R001_JUMP_CLASSIFICATION=SMALL_HOP
(criteria: net jump height above standing 0.0363 m; takeoff-to-apex COM rise 0.0768 m; true support-off
duration 0.2268 s; max geometric foot clearance 0.0496 m; no minimum-height/flight/clearance gate exists;
takeoff at 31 degrees knee flexion with a flat foot, so "functional loaded CMJ" is not defensible and
"ambiguous" is excluded by the measured magnitudes.)

R001_TAKEOFF_COM_VZ=1.2274 m/s (true support-off start; 1.2145 m/s at E6 occurrence)
R001_FLIGHT_DURATION=0.22675 s (true bilateral support-off; E6->E9 event time 0.226875 s)
R001_COM_FLIGHT_RISE=0.07684 m (true flight; 0.07516 m from E6 occurrence)
R001_MAX_FOOT_CLEARANCE_LEFT=0.049606 m
R001_MAX_FOOT_CLEARANCE_RIGHT=0.049606 m
R001_NET_JUMP_HEIGHT_ABOVE_STANDING=0.03627 m
R001_PELVIS_PITCH_RATE_AT_TAKEOFF_MIN=-9.68 rad/s (t=0.649875)
R001_TAKEOFF_SWITCH_LEAD_MS=8.125 (control switch t=0.640 vs true support loss t=0.648125)
R001_LANDING_HY_PEAK=9.880 kg m2/s (E10 occurrence; 3.333 at E9)
R001_POST_TOUCHDOWN_COM_VX_PEAK=0.3418 m/s

EVENT_CONTRACT_RESULT=12/12 declared events latch; 10 dwell events confirm one physics sample (0.125 ms) early (MED-001)
CONTROLLER_AUDIT_RESULT=FAIL_FOR_HUMAN_CREDIBILITY (premature takeoff switch, 0.944 action step, one-sided balance, vertical-only terminal, action-history reset)
PLANT_MODEL_RESULT=FAIL_ANATOMICAL_KINEMATICS (knee reversed; hip extension-biased; no MTP/toe)
FOOT_MODEL_RESULT=INADEQUATE_FOR_HUMAN_PUSH_OFF (flat-foot takeoff; no MTP/arch; latent rotated-foot gap/velocity bug)
MEASUREMENT_AUDIT_RESULT=PARTIAL (force/COM authorities correct; support margin, CoP frame, pelvis quat, unsigned tilt, prohibited flag defective)
CONTACT_AUDIT_RESULT=NO_ACTIVE_CRITICAL (two HIGH latent: support-margin semantics; rotated-foot gap/velocity)
LANDING_AUDIT_RESULT=FAIL_FOR_CREDIBILITY (forward lunge: vx 0.18->0.34 m/s, Hy x3, 25-degree lean, 13.15 s recovery)
RECOVERY_AUDIT_RESULT=OVERFIT_CERTIFICATE (one-trajectory micron envelope; E12 crosses handoff boundary)
CANONICAL_RUNTIME_RESULT=NOT_A_FREEZE + PERFORMANCE_METRICS_OMITTED
PROVENANCE_RESULT=PARTIAL (reproducible artifact; unfrozen execution surface; unverified external authorities)
PACKAGE_REPRODUCIBILITY_RESULT=FAIL (clean wheel cannot import canonical runtime; scipy undeclared/absent)
TEST_SUITE_FORENSIC_RESULT=INSUFFICIENT (collection errors; 11 placeholders; 7 neutralized assertions; no live canonical test)

ASSERT_TRUE_COUNT=10
OR_TRUE_COUNT=7
PLACEHOLDER_TEST_COUNT=11 (10 assert-True + 1 pass-only body)
BROKEN_EXTERNAL_PATH_TEST_COUNT=2 (tracked test needing absent external evidence; untracked WIP import error)

AUTHORITY_INVALIDATION_SUMMARY=Plant invalidated for human claims (reproducibility valid); RES-52/55/57
limited-requalification; RES-58/73 invalidated as sufficient; RES-76 invalidated (dead composition);
RES-78 invalidated as a freeze; RES-12 historical; RES-79 valid; R001 identity historical; trace/checkpoints
valid as reproducibility evidence. Full matrix: AUTHORITY_INVALIDATION_MATRIX.md.

R001_VALID_CLAIM_CEILING=bit-exact deterministic simulated event-chain complete jump episode, anatomically
invalid and performance-trivial; NOT a successful loaded countermovement jump (CLAIM_CEILING.md)

MODEL_FORM_LIMITATIONS_RESULT=3 task-invalidating model-form items for human claims, 2 measurement
authorities to rebuild, remainder acceptable/unvalidated with declared boundaries

## Artifacts

NEXT_CANDIDATE_REQUIREMENTS_PATH=audit/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001/NEXT_CANDIDATE_REQUIREMENTS.md
DEFECT_REGISTER_JSON=DEFECT_REGISTER.json
DEFECT_REGISTER_MD=DEFECT_REGISTER.md
AUTHORITY_INVALIDATION_MATRIX=AUTHORITY_INVALIDATION_MATRIX.md
TEST_SUITE_FORENSIC_REPORT=TEST_SUITE_FORENSIC_REPORT.md
MODEL_FORM_LIMITATIONS=MODEL_FORM_LIMITATIONS.md
CLAIM_CEILING=CLAIM_CEILING.md
SOURCE_REVIEW_COVERAGE=SOURCE_REVIEW_COVERAGE.json

EVIDENCE_BUNDLE_PATH=/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001/
EVIDENCE_SEAL_SHA256=recorded in the bundle SEAL.json and POSTCOMMIT_SIDECAR.json after the audit commit

## Reviews (all completed)

REVIEWS=CODE_REVIEW(complete), BUG_HUNT(complete), SCIENTIFIC_CONTRACT_REVIEW(complete),
BIOMECHANICS_MODEL_REVIEW(complete), CONTACT_REVIEW(complete), CONTROL_REVIEW(complete),
TEST_ADVERSARY(complete), PROVENANCE_REVIEW(complete), CLAIM_REVIEW(complete)

## Commit

COMMIT_CREATED=yes (single dedicated audit commit; no fixes; unrelated untracked dirt preserved)
COMMIT_MESSAGE=V2.1: seal post-render forensic defect audit
FINAL_HEAD=recorded externally in POSTCOMMIT_SIDECAR.json (a commit cannot contain its own SHA)
FINAL_COMMIT_TREE=recorded externally in POSTCOMMIT_SIDECAR.json
TRACKED_WORKTREE_AFTER_COMMIT=clean

LINEAR_RES80_STATUS=Done (audit complete; owner review next)

NEXT_AUTHORIZED_ACTION=OWNER_REVIEW_AND_ISSUE_GRAPH_GENERATION

Do not return a repair patch. Do not implement a fix. Do not create R002.
