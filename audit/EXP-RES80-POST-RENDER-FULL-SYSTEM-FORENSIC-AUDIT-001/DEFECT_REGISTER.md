# RES-80 Defect Register (V2.1 post-render full-system forensic audit)

Baseline `8f26736db1231042cbedc61f61ca4862b7be871c` / tree `855fe3b418c09b5028396adeaed3ba8cda51e89f`, candidate `V2.1-R001`, audit-only (no fixes).

## Counts

| Severity | Count |
|---|---|
| CRITICAL | 7 |
| HIGH | 19 |
| MEDIUM | 20 |
| LOW | 7 |

| Type | Count (multi-label) |
|---|---|
| CONTROL_BUG | 10 |
| DOCUMENTATION_DEFECT | 14 |
| MEASUREMENT_BUG | 7 |
| MODEL_FORM_DEFECT | 6 |
| PROVENANCE_DEFECT | 8 |
| QUALIFICATION_DEFECT | 13 |
| SCORER_SPEC_DEFECT | 10 |
| SOFTWARE_BUG | 16 |
| TEST_DEFECT | 5 |

## Index

| ID | Severity | Title | Type | Status |
|---|---|---|---|---|
| CRIT-001 | CRITICAL | Knee hinge direction is anatomically inverted; human knee flexion is outside the joint range | MODEL_FORM_DEFECT, SOFTWARE_BUG | CONFIRMED |
| CRIT-002 | CRITICAL | Hip range/convention allocates 1.80 rad to extension and only 0.50 rad to flexion; countermovement is driven into hip extension | MODEL_FORM_DEFECT, CONTROL_BUG | CONFIRMED |
| CRIT-003 | CRITICAL | No meaningful jump-performance requirement exists; a 3.6 cm hop qualifies as task success | SCORER_SPEC_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| CRIT-004 | CRITICAL | Candidate identity does not freeze the executing system | PROVENANCE_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| CRIT-005 | CRITICAL | Five external evidence JSON authorities are loaded at runtime without hash verification | PROVENANCE_DEFECT | CONFIRMED |
| CRIT-006 | CRITICAL | Production package is not self-contained and cannot run from a clean install | PROVENANCE_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| CRIT-007 | CRITICAL | Premature SUPPORTED->FLIGHT switch (single-sample guard) commands a ~0.94 action step while feet are still loaded; violent non-human takeoff | CONTROL_BUG, MODEL_FORM_DEFECT | CONFIRMED |
| HIGH-001 | HIGH | E10 impact-absorption predicate is underconstrained (vertical-only, no horizontal/angular/posture) | SCORER_SPEC_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| HIGH-002 | HIGH | RES-58 terminal capture regulates only vertical momentum; it has no horizontal/angular/CoP authority | CONTROL_BUG | CONFIRMED |
| HIGH-003 | HIGH | Balance controller authority is braking-only and cannot command forward CoP/Fx; forward lunge is uncorrectable in principle | CONTROL_BUG | CONFIRMED |
| HIGH-004 | HIGH | RECOVERY_READY predicate constrains rates only; recovery begins from a 24-degree forward lean with 0.65 rad knee and 15 cm forward CoM | CONTROL_BUG, SCORER_SPEC_DEFECT | CONFIRMED |
| HIGH-005 | HIGH | BALANCE->RECOVERY resets the held action to zeros, discarding up to 0.545 and breaking the previous-action continuity nominal | CONTROL_BUG | CONFIRMED |
| HIGH-006 | HIGH | Direct torque at 200 Hz with no activation dynamics or action-rate limiting produces non-physiological command steps | MODEL_FORM_DEFECT, CONTROL_BUG | CONFIRMED |
| HIGH-007 | HIGH | Scorer-code predicate is a control input (RES-43 handoff gate) despite the declared scorer/controller separation | SCORER_SPEC_DEFECT, PROVENANCE_DEFECT | CONFIRMED |
| HIGH-008 | HIGH | Support margin does not represent the active support polygon and is positive even in flight | MEASUREMENT_BUG, QUALIFICATION_DEFECT | CONFIRMED |
| HIGH-009 | HIGH | Inactive-foot geometric gap and normal velocity are wrong when the foot is rotated (up to 0.094 m) | MEASUREMENT_BUG, SOFTWARE_BUG | CONFIRMED |
| HIGH-010 | HIGH | Declared genuine-flight geometric gap is never implemented (E7 checks force only) | SCORER_SPEC_DEFECT, DOCUMENTATION_DEFECT | CONFIRMED |
| HIGH-011 | HIGH | Apex has no dwell and its fallback path can latch without a flight check | SCORER_SPEC_DEFECT, SOFTWARE_BUG | CONFIRMED |
| HIGH-012 | HIGH | True-standing envelope is a one-trajectory overfit (micron-scale windows, one-U LP expansion) | QUALIFICATION_DEFECT, SCORER_SPEC_DEFECT | CONFIRMED |
| HIGH-013 | HIGH | Canonical output omits all jump-performance metrics and violates its own required result schema | QUALIFICATION_DEFECT, DOCUMENTATION_DEFECT | CONFIRMED |
| HIGH-014 | HIGH | Soft-contact FZ-min rows are silently dropped during LS refinement; active-set crossings are not handled in the derivative | SOFTWARE_BUG, CONTROL_BUG | CONFIRMED |
| HIGH-015 | HIGH | Legacy 'honest full jump' acceptance file is false assurance: 10 assert-True stubs and >=9 events accepted as 12/12 | TEST_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| HIGH-016 | HIGH | Seven assertions are neutralized by `or True` (unconditional pass) and one test body is a bare pass | TEST_DEFECT | CONFIRMED |
| HIGH-017 | HIGH | No test executes the canonical composition; the suite validates stored JSON and breaks at collection | TEST_DEFECT | CONFIRMED |
| HIGH-018 | HIGH | Qualification orchestration omits two scripts (one hardcodes PASS); test_public_qualification runs only 10 of 19 suites | TEST_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| HIGH-019 | HIGH | Rigid single-box foot: no MTP/toe/arch; R001 push-off is flat-footed with no heel rise | MODEL_FORM_DEFECT | CONFIRMED |
| MED-001 | MEDIUM | Systematic dwell off-by-one: every dwell event confirms one physics sample (0.125 ms) before the declared duration | SCORER_SPEC_DEFECT, SOFTWARE_BUG | CONFIRMED |
| MED-002 | MEDIUM | E11 threshold is named COM_SPEED but implemented on com_vz only | SCORER_SPEC_DEFECT, DOCUMENTATION_DEFECT | CONFIRMED |
| MED-003 | MEDIUM | Pelvis orientation observation is a hardcoded identity quaternion | MEASUREMENT_BUG, SOFTWARE_BUG | CONFIRMED |
| MED-004 | MEDIUM | Trunk tilt measurement is unsigned | MEASUREMENT_BUG | CONFIRMED |
| MED-005 | MEDIUM | CoP frame/origin is misdeclared and its validity threshold disagrees with the declared constant | MEASUREMENT_BUG, DOCUMENTATION_DEFECT | CONFIRMED |
| MED-006 | MEDIUM | prohibited_contact is structurally always False; PROHIB=false in the canonical result carries no information | MEASUREMENT_BUG, SOFTWARE_BUG | CONFIRMED |
| MED-007 | MEDIUM | Collision topology permits anatomically impossible intersections and the 20 kg load has no collision geometry at all | MODEL_FORM_DEFECT | CONFIRMED |
| MED-008 | MEDIUM | FLEX->EXTEND program switch is a hard scheduled step (0.568) unrelated to state | CONTROL_BUG | CONFIRMED |
| MED-009 | MEDIUM | Post-projection clamps can leave the CoP/Fx/Hdot triple off the feasible line | SOFTWARE_BUG, CONTROL_BUG | CONFIRMED |
| MED-010 | MEDIUM | Legacy controller swallows all exceptions and commands zero action | SOFTWARE_BUG | CONFIRMED |
| MED-011 | MEDIUM | Reporting metrics are misdefined or dead: loading rate, apex_height alias, CoP invalid-interval analysis | MEASUREMENT_BUG, DOCUMENTATION_DEFECT | CONFIRMED |
| MED-012 | MEDIUM | Full-trajectory support adjudication is NOT_QUALIFIED while reported gates use the post-landing slice only | QUALIFICATION_DEFECT | CONFIRMED |
| MED-013 | MEDIUM | E12 fires 50.8 ms before the RES-43 handoff; part of its dwell is earned under SETTLE, not the final hold | QUALIFICATION_DEFECT | CONFIRMED |
| MED-014 | MEDIUM | Horizon authority drift: four coexisting horizons (4.0/8.0/20.0) with no runtime binding | DOCUMENTATION_DEFECT, QUALIFICATION_DEFECT | CONFIRMED |
| MED-015 | MEDIUM | Provenance artifacts disagree on entry heads and the authority ledger is stale/corrupted | PROVENANCE_DEFECT, DOCUMENTATION_DEFECT | CONFIRMED |
| MED-016 | MEDIUM | Production modules hardcode developer absolute paths and inject sys.path; evid_trace_v2 is imported under two module identities | PROVENANCE_DEFECT, SOFTWARE_BUG | CONFIRMED |
| MED-017 | MEDIUM | Production swallows diagnostics and reports sentinel values instead of failing closed | SOFTWARE_BUG, QUALIFICATION_DEFECT | CONFIRMED |
| MED-018 | MEDIUM | Dwell/threshold literals are duplicated across runtime, mirror module and external JSON with no binding | SOFTWARE_BUG, PROVENANCE_DEFECT | CONFIRMED |
| MED-019 | MEDIUM | Multiple V2.1-named tests exercise the legacy controller, not the canonical composition | TEST_DEFECT, DOCUMENTATION_DEFECT | CONFIRMED |
| MED-020 | MEDIUM | E1 supported-start guard omits the fall flag (only the dead prohibited flag is checked) | SCORER_SPEC_DEFECT | CONFIRMED |
| LOW-001 | LOW | Plant XML header documents ngeom=14 while the model has 16 | DOCUMENTATION_DEFECT | CONFIRMED |
| LOW-002 | LOW | README describes 15 bounded anatomical channels while ACTION_DIM=7 | DOCUMENTATION_DEFECT | CONFIRMED |
| LOW-003 | LOW | Stale/incorrect comments and dead branches in contact and support code | DOCUMENTATION_DEFECT, SOFTWARE_BUG | CONFIRMED |
| LOW-004 | LOW | IMPACT KD 'ramp' is an identity no-op presented as a ramp | SOFTWARE_BUG, DOCUMENTATION_DEFECT | CONFIRMED |
| LOW-005 | LOW | Dead code: unused y_of branch, unused unilateral_dropout_samples with misleading return, dead conditional expression | SOFTWARE_BUG, DOCUMENTATION_DEFECT | CONFIRMED |
| LOW-006 | LOW | WALL_S makes the canonical result JSON non-byte-deterministic | SOFTWARE_BUG, PROVENANCE_DEFECT | CONFIRMED |
| LOW-007 | LOW | Ankle sign comment is unresolved and geometrically inverted | DOCUMENTATION_DEFECT | CONFIRMED |

## CRIT-001 — Knee hinge direction is anatomically inverted; human knee flexion is outside the joint range

- **SEVERITY**: CRITICAL
- **TYPE**: MODEL_FORM_DEFECT, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/assets/v2_plant.xml, src/loaded_cmj/v2/constants.py
- **LINE_OR_FUNCTION**: v2_plant.xml:97 (left_knee axis 0 -1 0, range 0 2.40); v2_plant.xml:125 (right_knee); constants.py:138-139
- **DECLARED_BEHAVIOR**: XML/constants comment: knee '0 extended, flexion positive'; range [0, 2.40]
- **IMPLEMENTED_BEHAVIOR**: Positive knee q rotates the shank so the ankle moves anteriorly (+x) and up relative to the knee; negative q (human flexion direction, heel toward buttocks) is outside [0, 2.40].
- **REPRODUCTION_OR_PROOF**: Deterministic forward-kinematics probe on the frozen XML (probes/probe_geometry.py, PROBE A1): knee q=0.4/0.8/1.2 gives ankle_x-knee_x=+0.167/+0.309/+0.401 m. Accepted trajectory: knee q peaks at +1.269 rad at t=0.4875 s; at E3 the ankle is +0.181 m anterior of the knee and the knee is 29 cm posterior to the ankle-plane.
- **NUMERIC_EVIDENCE**: `{"R001_ankle_minus_knee_x_at_E3_m": 0.181, "R001_knee_at_E6_rad": 0.5413, "R001_knee_max_rad": 1.2690245, "ankle_minus_knee_x_m": [0.0, 0.1674, 0.3085, 0.4008], "knee_q": [0.0, 0.4, 0.8, 1.2]}`
- **OBSERVED_EFFECT**: The accepted countermovement/landing is performed with a reverse-bending knee (knee apex posterior); the model cannot represent a human knee-flexed pose.
- **ROOT_CAUSE**: Hinge axis sign (0 -1 0) plus positive-only range; XML/constants naming asserts the opposite convention.
- **R001_IMPACT**: Entire accepted R001 squat/extension/landing is reverse-knee; the owner-observed knee/leg flick and back/forward leg motion are direct consequences.
- **SCIENTIFIC_CLAIM_IMPACT**: Biomechanical claim 'loaded countermovement jump' is invalid at the kinematic level; no human technique interpretation is possible.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: Plant XML, Plant hash (content changes under any fix), all trajectory-dependent evidence
- **DEPENDENCIES**: CRIT-002, HIGH-019, MED-007
- **FIX_CLASS**: MODEL_FORM_CHANGE (flip knee axis/range and re-derive controller knee signs) + full requalification
- **REQUIRED_REGRESSION_TEST**: FK sign test: positive knee q must move the ankle posteriorly (-x) relative to the knee; R001-class run must show human knee flexion during countermovement.
- **REQUALIFICATION_SCOPE**: Plant, controller, events/scorer, measurements, full V2.1 requalification; historical R001 remains reproducible-only.
- **EVIDENCE_POINTERS**: audit/.../probes/probe_geometry.py, audit/.../probes/probe_skeleton.py, DEFECT_REGISTER.md#CRIT-001
- **PREFLIGHT_REFERENCES**: P13-adjacent

## CRIT-002 — Hip range/convention allocates 1.80 rad to extension and only 0.50 rad to flexion; countermovement is driven into hip extension

- **SEVERITY**: CRITICAL
- **TYPE**: MODEL_FORM_DEFECT, CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/assets/v2_plant.xml, src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: v2_plant.xml:88 (axis 0 1 0, range -0.50 1.80); constants.py:136-137; res72_integration.py:59-60 (FLEX_TAU/EXTEND_TAU positive hip)
- **DECLARED_BEHAVIOR**: Hip flexion positive (human squat requires ~0.8-1.5 rad hip flexion)
- **IMPLEMENTED_BEHAVIOR**: Positive hip q rotates the thigh posteriorly (extension); the range excludes human deep hip flexion beyond 0.50 rad. FLEX_TAU=+20 Nm drives extension during the countermovement (q rises +0.135 -> +0.587 rad).
- **REPRODUCTION_OR_PROOF**: FK probe (probe_geometry.py) plus accepted qpos trace; at deepest crouch the thigh is rotated posteriorly with the knee behind the pelvis.
- **NUMERIC_EVIDENCE**: `{"R001_hip_max_rad": 0.5874, "hip_at_E4_rad": 0.376, "hip_at_E6_rad": -0.3181, "range_rad": [-0.5, 1.8]}`
- **OBSERVED_EFFECT**: The body sinks by extending the hip rather than flexing it, forcing a non-anatomical leg 'Z' and contributing to the backward body motion.
- **ROOT_CAUSE**: Joint axis sign + range allocation inverted relative to human hip flexion; controller 'FLEX' program uses positive hip torque.
- **R001_IMPACT**: Countermovement posture is anatomically invalid; contributes to reverse-knee geometry and to the takeoff lurch.
- **SCIENTIFIC_CLAIM_IMPACT**: Loaded-CMJ biomechanics claim invalid; hip flexion range is a task-critical requirement.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: Plant XML, controller FLEX/EXTEND programs, countermovement metrics
- **DEPENDENCIES**: CRIT-001, HIGH-019
- **FIX_CLASS**: MODEL_FORM_CHANGE (hip range/sign) + controller sign re-derivation + requalification
- **REQUIRED_REGRESSION_TEST**: Hip flexion >= 1.0 rad reachable during countermovement with human femoral direction; postural signature test.
- **REQUALIFICATION_SCOPE**: Plant/controller/events; full V2.1 requalification.
- **EVIDENCE_POINTERS**: audit/.../probes/probe_geometry.py, traj_summary.json

## CRIT-003 — No meaningful jump-performance requirement exists; a 3.6 cm hop qualifies as task success

- **SEVERITY**: CRITICAL
- **TYPE**: SCORER_SPEC_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/events.py, src/loaded_cmj/v2/canonical_runtime.py
- **LINE_OR_FUNCTION**: constants.py:181-209 (V2_EVENT_THRESHOLDS); events.py:193-202 (only TAKEOFF_VZ_MIN checked at E6 onset); events.py:1076-1079 (apex metrics computed); canonical_runtime.py:540-590 (output omits them)
- **DECLARED_BEHAVIOR**: Task acceptance criteria include a genuine loaded CMJ; TAKEOFF_VZ_MIN=0.60, PREFERRED=0.80
- **IMPLEMENTED_BEHAVIOR**: No minimum jump height, no minimum flight time, no minimum foot clearance, no minimum COM rise/apex gate. GENUINE_FLIGHT_GAP_M=0.010, APEX_DWELL_S=0.005 and TAKEOFF_VZ_PREFERRED_MPS=0.80 are declared but never referenced by executable code.
- **REPRODUCTION_OR_PROOF**: Source grep (unused constants) + accepted-run quantification: apex COM z 1.11118 m vs standing 1.07485 m = 0.0363 m net jump height; takeoff-to-apex rise 0.0768 m; flight 0.2268 s; max foot clearance 0.0496 m.
- **NUMERIC_EVIDENCE**: `{"TAKEOFF_VZ_MIN_MPS": 0.6, "com_rise_from_takeoff_m": 0.07684, "flight_duration_s": 0.22675, "max_foot_clearance_m": 0.049606, "net_jump_height_above_standing_m": 0.03627, "takeoff_vz_mps": 1.2274}`
- **OBSERVED_EFFECT**: The qualification logic cannot distinguish a small hop from a functional loaded CMJ; the owner's 'extremely little visible flight' is compatible with a PASS.
- **ROOT_CAUSE**: Task contract lacks performance gates; only event-completion gates exist.
- **R001_IMPACT**: R001 PASS is compatible with a 3.6 cm hop; SHIP_STATUS blocked mainly on visual review, not on any quantitative floor.
- **SCIENTIFIC_CLAIM_IMPACT**: Any 'successful loaded countermovement jump' claim is unsupported; 'computational event-chain PASS' is the maximum valid claim.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), constants.py (unhashed), qualification criteria
- **DEPENDENCIES**: HIGH-010, HIGH-011, HIGH-013, MED-011
- **FIX_CLASS**: SPEC/QUALIFICATION_CHANGE (add owner-approved performance gates) - REQUIRES_OWNER/SCIENTIFIC_CONTRACT_DECISION for values
- **REQUIRED_REGRESSION_TEST**: Negative-control trajectory that completes 12/12 events with an objectively trivial hop must FAIL the task gate.
- **REQUALIFICATION_SCOPE**: Scorer spec + qualification scope; no plant change required for the gate itself.
- **EVIDENCE_POINTERS**: traj_summary.json, DEFECT_REGISTER.md#CRIT-003
- **PREFLIGHT_REFERENCES**: P01, P02, D1, D2, D3, D4, D5, D6, D7, D8

## CRIT-004 — Candidate identity does not freeze the executing system

- **SEVERITY**: CRITICAL
- **TYPE**: PROVENANCE_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: CANONICAL_V2_CANDIDATE_SPEC.json, CANONICAL_CANDIDATE_IDENTITY.json, src/loaded_cmj/v2/canonical_runtime.py, tests/test_res78_canonical_runtime.py
- **LINE_OR_FUNCTION**: CANONICAL_V2_CANDIDATE_SPEC.json (CONTROLLER_COMPOSITION_SHA256 over 5 files); test_res78_canonical_runtime.py:75-79; canonical_runtime.py:50-55 (imports)
- **DECLARED_BEHAVIOR**: CONTROLLER_COMPOSITION_SHA256 is the frozen controller identity
- **IMPLEMENTED_BEHAVIOR**: The hash covers res72_integration, balance_capture, stable_recovery, terminal_capture, full_closure. full_closure is imported only by tests (dead on the canonical path); the executed orchestration lives in unhashed canonical_runtime.py, plus unhashed constants.py, plant.py, measurement.py, drive.py, support_continuity.py and tools/{core52, soft_contact, evid_trace_v2}.py. No runtime recomputation of any hash exists.
- **REPRODUCTION_OR_PROOF**: Hash recomputation (composition hash verified correct over the 5 files), import-graph grep, wheel inspection; canonical_runtime only checks the --candidate string.
- **NUMERIC_EVIDENCE**: `{"executed_but_unhashed_modules": 9, "hash_bearing_file_dead_on_canonical_path": "full_closure.py", "hashed_files": 5, "runtime_hash_checks": 0}`
- **OBSERVED_EFFECT**: Two different working trees can both emit a result labeled V2.1-R001 with the declared hashes intact.
- **ROOT_CAUSE**: Hash domain chosen as a five-file concatenation rather than the executable closure; orchestrator/authorities excluded.
- **R001_IMPACT**: R001's own reproducibility is real (same tree), but 'frozen candidate' provenance is not enforceable.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Frozen V2.1-R001' identity claim is invalid; candidate identity is a partial fingerprint only.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: CONTROLLER_COMPOSITION_SHA256, CANDIDATE_SPEC_SHA256, RUNTIME_CONTRACT_SHA256, CONTROLLER_FILES map
- **DEPENDENCIES**: CRIT-005, MED-015, MED-016, HIGH-007
- **FIX_CLASS**: PROVENANCE_CHANGE (expand hash domain to full executable closure and verify at runtime)
- **REQUIRED_REGRESSION_TEST**: Clean-clone test: mutate any executed-but-unhashed module -> identity verifier must FAIL before execution.
- **REQUALIFICATION_SCOPE**: Successor candidate identity definition; R001 historical hash remains valid as a partial fingerprint.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, DEFECT_REGISTER.md#CRIT-004

## CRIT-005 — Five external evidence JSON authorities are loaded at runtime without hash verification

- **SEVERITY**: CRITICAL
- **TYPE**: PROVENANCE_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:74-85 (_load_authorities); canonical_runtime.py:104-106, 250, 355
- **DECLARED_BEHAVIOR**: Frozen authorities RR spec, recovery manifold, handoff spec, T_RISE, terminal controller constants
- **IMPLEMENTED_BEHAVIOR**: Runtime json.loads() of /home/litju/.../EXP-RES10-*/{recovery_ready_spec.json, RECOVERY_MANIFOLD_AUTHORITY.json, stand_handoff_ready_spec.json, RECOVERY_TIME_SCALING_AUTHORITY.json, experiment_spec.json} with no sha256/SPEC_SHA256 check and no fallback.
- **REPRODUCTION_OR_PROOF**: Source inspection; no hash call on this path; support_continuity.load_spec() (which verifies a spec hash) is never called by the runtime.
- **NUMERIC_EVIDENCE**: `{"authority_files_loaded_unverified": 5, "hash_checks": 0, "paths_absolute": true}`
- **OBSERVED_EFFECT**: Mutating non-Git evidence can change the RR dwell/thresholds, the 13-node recovery manifold, handoff gates/T_RISE and terminal BVLS constants while the candidate identity is unchanged.
- **ROOT_CAUSE**: External authorities are consumed as trusted inputs outside the hash domain.
- **R001_IMPACT**: R001 evidence was produced from one evidence state; the provenance of that state is not cryptographically bound to the candidate.
- **SCIENTIFIC_CLAIM_IMPACT**: Reproducibility claim depends on external mutable files; 'same candidate' cannot be assumed across evidence mutations.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: RR spec, RECOVERY_MANIFOLD_AUTHORITY, stand_handoff_ready_spec, RECOVERY_TIME_SCALING_AUTHORITY, RES-52 experiment_spec
- **DEPENDENCIES**: CRIT-004, CRIT-006
- **FIX_CLASS**: PROVENANCE_CHANGE (vendor authorities into repo or pin hashes in identity and verify at load)
- **REQUIRED_REGRESSION_TEST**: Hash-mutation negative test: alter one authority byte -> run must fail closed before any controller construction.
- **REQUALIFICATION_SCOPE**: Successor qualification; historical evidence remains valid as reproduced.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, DEFECT_REGISTER.md#CRIT-005
- **PREFLIGHT_REFERENCES**: P25

## CRIT-006 — Production package is not self-contained and cannot run from a clean install

- **SEVERITY**: CRITICAL
- **TYPE**: PROVENANCE_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: pyproject.toml, uv.lock, src/loaded_cmj/v2/canonical_runtime.py, tools/res52/soft_contact.py
- **LINE_OR_FUNCTION**: pyproject.toml:6-9 (deps), :18-19 (packages); uv.lock (no scipy); canonical_runtime.py:53-55 (imports core52/soft_contact/evid_trace_v2); balance_capture.py:26 / stable_recovery.py:34 / soft_contact.py:35 (scipy)
- **DECLARED_BEHAVIOR**: Declared runtime environment: mujoco 3.8.0, numpy 2.5.1, scipy 1.18.1; canonical command `python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001`
- **IMPLEMENTED_BEHAVIOR**: Wheel contains only src/loaded_cmj; tools/ modules are absent; scipy is not declared and not present in uv.lock; all controller JSON authorities use absolute developer paths.
- **REPRODUCTION_OR_PROOF**: Built wheel with `uv build --wheel` (audit/.../probes runs in /tmp): 64 entries, tools/ absent, v2_plant.xml present, no scipy dependency metadata. `import core52`, `from soft_contact import SoftContactPolicy`, `from tools.evid_trace_v2 import ...` follow immediately from canonical_runtime imports.
- **NUMERIC_EVIDENCE**: `{"scipy_in_metadata": false, "scipy_in_uv_lock": false, "tools_in_wheel": false, "wheel_entries": 64}`
- **OBSERVED_EFFECT**: No clean environment can execute the canonical candidate; deployment reproducibility fails.
- **ROOT_CAUSE**: Packaging boundary excludes the modules that implement the executed controller; dependency declaration incomplete.
- **R001_IMPACT**: R001 was reproduced only in the developer tree with a manually provisioned scipy and the external evidence tree.
- **SCIENTIFIC_CLAIM_IMPACT**: Deployment/reproducibility claims invalid outside the author's machine.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: pyproject.toml, uv.lock, wheel contents, CANONICAL_COMMAND
- **DEPENDENCIES**: CRIT-004, CRIT-005, MED-016
- **FIX_CLASS**: PACKAGING_CHANGE (include tools or move modules into package; declare scipy; remove absolute paths)
- **REQUIRED_REGRESSION_TEST**: Clean-venv install test: install wheel + declared deps, run canonical runtime against a vendored evidence fixture.
- **REQUALIFICATION_SCOPE**: Successor release qualification.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, wheel listing captured in AUDIT_COMMAND_LOG.txt
- **PREFLIGHT_REFERENCES**: P32, P33

## CRIT-007 — Premature SUPPORTED->FLIGHT switch (single-sample guard) commands a ~0.94 action step while feet are still loaded; violent non-human takeoff

- **SEVERITY**: CRITICAL
- **TYPE**: CONTROL_BUG, MODEL_FORM_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: res72_integration.py:156-159 (maxf<10 single sample, no dwell/direction); res72_integration.py:201-203 (zero-pose FLIGHT PD)
- **DECLARED_BEHAVIOR**: Flight starts at bilateral physical support loss with a bounded/continuous action
- **IMPLEMENTED_BEHAVIOR**: Phase 1->2 fires on one control sample with maxf<10; the flight law commands a stiff PD to zero pose (KP 80/KD 12), causing a 0.944 (hip/knee ~236/270 Nm) step in one 5 ms interval at t=0.640, 6.75 ms before true bilateral support loss (0.64675) / 8.125 ms before sealed E6 (0.648125).
- **REPRODUCTION_OR_PROOF**: Sealed action schedule analysis (probe_skeleton.py) + true support reconstruction from ctrl-aware contact forces (probe_forces2.py) + pelvis pitch-rate trace.
- **NUMERIC_EVIDENCE**: `{"E6_occ_s": 0.648125, "max_du": 0.9439, "pelvis_pitch_rate_min_radps": -9.6837, "pelvis_pitch_rate_min_t_s": 0.649875, "switch_t_s": 0.64, "true_support_loss_t_s": 0.648125, "u_after": [0.179, 0.621, 0.621, 0.358, 0.358, 0.1, 0.1], "u_before": [-0.01, -0.323, -0.323, -0.543, -0.543, -0.1, -0.1]}`
- **OBSERVED_EFFECT**: The body is whipped backward at ~9.7 rad/s at takeoff; whole-body backward motion and a violent transition are direct consequences; takeoff occurs before physical separation.
- **ROOT_CAUSE**: Guard lacks dwell/hysteresis and vz direction; flight reference is zero pose with no ballistic/COM authority; free root pitch is unregulated.
- **R001_IMPACT**: R001 takeoff is control-induced rather than physics-caused; contributes to the low jump and the owner's takeoff observations.
- **SCIENTIFIC_CLAIM_IMPACT**: Takeoff mechanics claim invalid; E6 is latched after the fact and does not certify a physical takeoff timing.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: res72_integration.py (CONTROLLER_COMPOSITION_SHA256), E6 semantics, controller phase guards
- **DEPENDENCIES**: HIGH-005, HIGH-006, HIGH-002, MED-008
- **FIX_CLASS**: CONTROL_CHANGE (dwell/direction gate; state-based extension completion; flight authority)
- **REQUIRED_REGRESSION_TEST**: Adversarial guard test: single noisy control sample of maxf<10 must not switch to FLIGHT; takeoff action step must be bounded and occur at/after support loss.
- **REQUALIFICATION_SCOPE**: Controller + events requalification; successor candidate.
- **EVIDENCE_POINTERS**: probe_skeleton.py output, probe_forces2.py output, DEFECT_REGISTER.md#CRIT-007
- **PREFLIGHT_REFERENCES**: P04, P05, P20-adjacent

## HIGH-001 — E10 impact-absorption predicate is underconstrained (vertical-only, no horizontal/angular/posture)

- **SEVERITY**: HIGH
- **TYPE**: SCORER_SPEC_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/events.py, src/loaded_cmj/v2/canonical_runtime.py
- **LINE_OR_FUNCTION**: events.py:241-245 (_guard_impact_absorption); canonical_runtime.py:338 (E10 streak predicate)
- **DECLARED_BEHAVIOR**: Impact absorption means vertical momentum is absorbed with a stable posture
- **IMPLEMENTED_BEHAVIOR**: Guard is only abs(com_vz)<0.05, plus bilateral Fz>10 N and no fall/prohib at the runtime switch. Horizontal velocity, angular momentum, trunk posture and CoP are unconstrained.
- **REPRODUCTION_OR_PROOF**: Source inspection + R001 trace: during impact Hy rises from 3.33 to 9.88 kg m2/s while com_vx rises 0.181->0.211 m/s; E10 latches regardless.
- **NUMERIC_EVIDENCE**: `{"Hy_at_E10_occ_kgm2ps": 9.88, "Hy_at_E9_occ_kgm2ps": 3.333, "com_vx_at_E10_occ_mps": 0.2115, "com_vx_at_E9_mps": 0.1814}`
- **OBSERVED_EFFECT**: The system declares absorption while forward rotational momentum is growing; the subsequent forward lunge is uncertified.
- **ROOT_CAUSE**: Predicate omits horizontal/angular/posture state; canonical runtime mirrors the same vertical-only condition.
- **R001_IMPACT**: E10 timing in R001 is driven by a transient |vz|<0.05 window; the lunge occurs during/after it.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Impact absorption PASS' does not imply absorbed landing; claim is limited to vertical-speed capture.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), E10 definition, landing qualification
- **DEPENDENCIES**: HIGH-002, HIGH-003, HIGH-004, CRIT-003
- **FIX_CLASS**: SCORER_SPEC_CHANGE + CONTROL_CHANGE (add horizontal/angular/postural criteria)
- **REQUIRED_REGRESSION_TEST**: Negative control: a landing with large inherited vx/Hy must not latch E10.
- **REQUALIFICATION_SCOPE**: Events + landing qualification; successor candidate.
- **EVIDENCE_POINTERS**: traj_summary.json, probe_landing.py output
- **PREFLIGHT_REFERENCES**: P08

## HIGH-002 — RES-58 terminal capture regulates only vertical momentum; it has no horizontal/angular/CoP authority

- **SEVERITY**: HIGH
- **TYPE**: CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/terminal_capture.py, tools/res52/soft_contact.py
- **LINE_OR_FUNCTION**: terminal_capture.py:79-97 (capture_force_command/vz target); soft_contact.py:219-237 (no Fx/Hy/CoP rows)
- **DECLARED_BEHAVIOR**: Terminal capture arrests the fall and stabilizes the body
- **IMPLEMENTED_BEHAVIOR**: FZ_DES = clamp(BW - m*vz/DT, 0.6BW, 1.5BW); the inner LS has Fz/FzL/FzR, dist, nvel, vz, qdot rows only. No Fx, Hy/Hdot, CoP, or absolute posture is commanded.
- **REPRODUCTION_OR_PROOF**: Source inspection; accepted-run Hy peak 9.88 kg m2/s and com_vx 0.28-0.34 m/s at/after E10.
- **NUMERIC_EVIDENCE**: `{"Hy_peak_after_impact_kgm2ps": 9.88, "com_vx_peak_after_impact_mps": 0.3418}`
- **OBSERVED_EFFECT**: The landing's forward pitch/translation cannot be caught by the terminal layer; the-9.7 rad/s takeoff pitch and impact Hy are structurally outside its authority.
- **ROOT_CAUSE**: Vertical-only outer law derived from a fall-arrest objective.
- **R001_IMPACT**: R001 reaches BALANCE with substantial forward momentum that the terminal layer never addressed.
- **SCIENTIFIC_CLAIM_IMPACT**: Terminal-capture claim is limited to vertical arrest.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: terminal_capture.py (CONTROLLER_COMPOSITION_SHA256), RES-58 authority, E10->BALANCE handoff
- **DEPENDENCIES**: HIGH-003, HIGH-004, HIGH-009
- **FIX_CLASS**: CONTROL_CHANGE (add horizontal/CAM/CoP authority or a dedicated landing strategy)
- **REQUIRED_REGRESSION_TEST**: Landing with a lateral/forward perturbation must keep CoM/CAM bounded through capture.
- **REQUALIFICATION_SCOPE**: Controller + landing requalification.
- **EVIDENCE_POINTERS**: probe_landing.py output
- **PREFLIGHT_REFERENCES**: P14

## HIGH-003 — Balance controller authority is braking-only and cannot command forward CoP/Fx; forward lunge is uncorrectable in principle

- **SEVERITY**: HIGH
- **TYPE**: CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/balance_capture.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: balance_capture.py:33-34 (FX_BRAKE_MIN=-120, FX_BRAKE_MAX=0; HDOT_MIN=-60, HDOT_MAX=0); balance_capture.py:143 (CoP clamp)
- **DECLARED_BEHAVIOR**: Centroidal balance with feasible CoP projection
- **IMPLEMENTED_BEHAVIOR**: Fx demand is clamped to [-120, 0] and Hy reduction to [-60, 0]; only backward braking/reduction is representable. Accepted R001 needs forward acceleration of the CoM (from -0.058 at t=2.117; but the forward lunge requires +Fx earlier) and the controller cannot ask for it.
- **REPRODUCTION_OR_PROOF**: Source + accepted trace: com_vx accelerates 0.181 -> 0.342 m/s after touchdown while the controller can only brake; forward CoM reversal occurs at t=2.117 s.
- **NUMERIC_EVIDENCE**: `{"FX_range": [-120.0, 0.0], "HDOT_range": [-60.0, 0.0], "com_vx_at_RR_conf_mps": 0.0729, "com_vx_peak_after_touchdown_mps": 0.3418}`
- **OBSERVED_EFFECT**: The owner-observed forward lunge/bounce is not preventable by the balance layer's authority set.
- **ROOT_CAUSE**: One-sided authority deliberately frozen (braking only).
- **R001_IMPACT**: R001's balance/capture phase is a waiting game until momentum decays; posture degrades (trunk pitch to 25 deg) meanwhile.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Balance capture' claim is weaker than named; only backward braking is implemented.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: balance_capture.py (CONTROLLER_COMPOSITION_SHA256), RES-73 authority, E11 definition
- **DEPENDENCIES**: HIGH-001, HIGH-002, HIGH-004, MED-009
- **FIX_CLASS**: CONTROL_CHANGE (two-sided feasible CoP/Fx authority within support)
- **REQUIRED_REGRESSION_TEST**: Perturbation test requiring both forward and backward CoP authority; posture bound during capture.
- **REQUALIFICATION_SCOPE**: Balance/landing requalification.
- **EVIDENCE_POINTERS**: probe_landing.py, DEFECT_REGISTER.md#HIGH-003
- **PREFLIGHT_REFERENCES**: P23

## HIGH-004 — RECOVERY_READY predicate constrains rates only; recovery begins from a 24-degree forward lean with 0.65 rad knee and 15 cm forward CoM

- **SEVERITY**: HIGH
- **TYPE**: CONTROL_BUG, SCORER_SPEC_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/full_closure.py, src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: full_closure.py:62-75 (rr_predicate_ok); canonical_runtime.py:352-361 (inline RR); stable_recovery.py:612-634 (_handoff_ready)
- **DECLARED_BEHAVIOR**: Recovery-ready means the athlete can begin standing recovery
- **IMPLEMENTED_BEHAVIOR**: RR conditions: |vx|<=0.15, |vz|<=0.05, |Hy|<=1.5, |root pitch rate|<=0.5, |trunk rate|<=1.0, qdot<=1.0, margin>=0.05, Fz>10, pen<=0.01. No absolute trunk pitch, joint configuration, COM-x relative to feet, or CoP-position constraint.
- **REPRODUCTION_OR_PROOF**: Source + accepted trace at RR entry/confirmation: root pitch 0.4226/0.4315 rad, knee 0.6476/0.6529, COM x +0.149/+0.160 m.
- **NUMERIC_EVIDENCE**: `{"com_x_at_RR_conf_m": 0.1604, "com_x_at_RR_entry_m": 0.1493, "knee_at_RR_entry_rad": 0.6476, "recovery_span_s": 13.15, "root_pitch_at_RR_entry_rad": 0.4226}`
- **OBSERVED_EFFECT**: The 13.15 s RISE/SETTLE recovery starts from a poorly postured state; the long recovery is a symptom, not a designed behavior.
- **ROOT_CAUSE**: Predicate built only from rate/velocity envelopes.
- **R001_IMPACT**: RR entry is much later than E11 (E11 occ 0.9935, RR entry 1.237) because velocity decays; posture is never corrected before RISE.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Stable recovery' certificate begins only after 13 s and does not encode posture quality at entry.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: full_closure.py (in composition hash but dead), RR spec external JSON, E11/E12 semantics
- **DEPENDENCIES**: HIGH-003, CRIT-005, MED-013
- **FIX_CLASS**: SCORER_SPEC_CHANGE + CONTROL_CHANGE (add absolute posture/CoP envelope to RR)
- **REQUIRED_REGRESSION_TEST**: Posture-at-entry test: recovery may only start inside a declared standing-approach posture set.
- **REQUALIFICATION_SCOPE**: Balance/recovery requalification.
- **EVIDENCE_POINTERS**: probe_landing.py, DEFECT_REGISTER.md#HIGH-004
- **PREFLIGHT_REFERENCES**: P16

## HIGH-005 — BALANCE->RECOVERY resets the held action to zeros, discarding up to 0.545 and breaking the previous-action continuity nominal

- **SEVERITY**: HIGH
- **TYPE**: CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:253 (held = zeros); stable_recovery.py:127 (u_prev=zeros)
- **DECLARED_BEHAVIOR**: Action-history continuity nominal: each control layer starts from the previously applied action
- **IMPLEMENTED_BEHAVIOR**: At the BALANCE->RECOVERY switch the runtime sets held=zeros; the recovery controller initializes u_prev=zeros; the first recovery action is bounded relative to a false origin.
- **REPRODUCTION_OR_PROOF**: Sealed control schedule: |du|=0.5449 at t=1.337 from the last balance action to the first recovery action.
- **NUMERIC_EVIDENCE**: `{"max_du_at_switch": 0.5449, "switch_t_s": 1.337}`
- **OBSERVED_EFFECT**: A 0.545 controller step occurs exactly at the transition, in addition to the law change.
- **ROOT_CAUSE**: Explicit action-history reset at regime switch.
- **R001_IMPACT**: R001 recovery entry contains an unnecessary action discontinuity.
- **SCIENTIFIC_CLAIM_IMPACT**: Continuity claim in the controller documentation is violated at this switch.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: canonical_runtime.py (unhashed), StableRecoveryController, soft-contact PREVIOUS_APPLIED_ACTION nominal
- **DEPENDENCIES**: CRIT-007, HIGH-006
- **FIX_CLASS**: CONTROL_CHANGE (carry held action across the switch)
- **REQUIRED_REGRESSION_TEST**: Continuity test: sup |u_k - u_{k-1}| bounded by the trust region across every regime switch.
- **REQUALIFICATION_SCOPE**: Controller requalification.
- **EVIDENCE_POINTERS**: probe_skeleton.py output
- **PREFLIGHT_REFERENCES**: P19

## HIGH-006 — Direct torque at 200 Hz with no activation dynamics or action-rate limiting produces non-physiological command steps

- **SEVERITY**: HIGH
- **TYPE**: MODEL_FORM_DEFECT, CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/drive.py, src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: drive.py:25-36 (tau = limit*u); res72_integration.py:191-210 (discrete program switches)
- **DECLARED_BEHAVIOR**: Credible model of human loaded-CMJ actuation
- **IMPLEMENTED_BEHAVIOR**: Each 5 ms control interval applies a directly commanded bounded torque; there is no activation state, no action slew, and phase programs switch in one interval. Observed steps: 0.568 (t=0.400), 0.944 (t=0.640), 0.545 (t=1.337).
- **REPRODUCTION_OR_PROOF**: Sealed action schedule (probe_skeleton.py).
- **NUMERIC_EVIDENCE**: `{"action_steps": [{"du": 0.568, "t": 0.4}, {"du": 0.944, "t": 0.64}, {"du": 0.545, "t": 1.337}], "control_rate_hz": 200.0}`
- **OBSERVED_EFFECT**: Muscle-like force rise times (tens of ms) are absent; takeoff extension and landing absorption are impulse-like command changes.
- **ROOT_CAUSE**: Direct-torque actuator model chosen for transparency; no activation/slew layer.
- **R001_IMPACT**: R001 takeoff shock and the sense of a mechanical/non-human transition are partially attributable to this abstraction.
- **SCIENTIFIC_CLAIM_IMPACT**: Any claim of human-like loaded-CMJ actuation is unsupported; this is a declared model simplification with material trajectory effect.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: drive.py, controller programs, R001 action schedule
- **DEPENDENCIES**: CRIT-007, HIGH-005, MED-008
- **FIX_CLASS**: MODEL_FORM_CHANGE (activation dynamics or command slew; may be scoped as declared limitation instead)
- **REQUIRED_REGRESSION_TEST**: Activation/slew regression test: command steps bounded by the declared rise-time model and documented as a limitation.
- **REQUALIFICATION_SCOPE**: Plant/controller model-form decision + requalification.
- **EVIDENCE_POINTERS**: probe_skeleton.py, DEFECT_REGISTER.md#HIGH-006
- **PREFLIGHT_REFERENCES**: P20

## HIGH-007 — Scorer-code predicate is a control input (RES-43 handoff gate) despite the declared scorer/controller separation

- **SEVERITY**: HIGH
- **TYPE**: SCORER_SPEC_DEFECT, PROVENANCE_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/events.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:232-240 (_is_true_standing_neighborhood -> env0); canonical_runtime.py:268 (env0 passed to recoverer.step); stable_recovery.py:580 (in_envelope gates HANDOFF); events.py:259-390
- **DECLARED_BEHAVIOR**: Scorer runs observationally only; no scorer state for control
- **IMPLEMENTED_BEHAVIOR**: The runtime calls the scorer's private method _is_true_standing_neighborhood and passes the boolean into StableRecoveryController.step, which uses it (with the 0.05 s sustain) to switch SETTLE->HANDOFF. events.py is bound only by SCORER_SHA256; its constants live in unhashed constants.py.
- **REPRODUCTION_OR_PROOF**: Static call-graph proof; the private predicate resolves TRUE_STANDING_ENVELOPE/BILATERAL_FZ_THRESHOLD from constants.py.
- **NUMERIC_EVIDENCE**: `{"constants_hash": null, "handoff_gate": "in_envelope (scorer private predicate)", "scorer_hash_separate_from_controller_hash": true}`
- **OBSERVED_EFFECT**: Modifying events.py or constants.py can change the applied action stream and the handoff time, while the controller composition hash stays valid.
- **ROOT_CAUSE**: Separation claim is prose-only; a pure physical predicate implemented in scorer code is consumed by control.
- **R001_IMPACT**: R001's handoff time (14.487 s) depends on this predicate; E12 occurred 50.8 ms before that handoff.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Scorer observational only' claim is false as implemented; provenance of control depends on scorer code.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), constants.py, RES-43 handoff authority, CANONICAL_RUNTIME_AUTHORITY.md:121
- **DEPENDENCIES**: CRIT-004, CRIT-005, MED-013
- **FIX_CLASS**: PROVENANCE/CONTRACT_CHANGE (move shared physical predicate into a hashed neutral module or extend controller hash)
- **REQUIRED_REGRESSION_TEST**: Separation test: changing scorer-only code must not change the action stream; shared predicate must live in the hashed closure.
- **REQUALIFICATION_SCOPE**: Successor candidate identity + separation proof.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, CONTROL_REVIEW.md
- **PREFLIGHT_REFERENCES**: P26

## HIGH-008 — Support margin does not represent the active support polygon and is positive even in flight

- **SEVERITY**: HIGH
- **TYPE**: MEASUREMENT_BUG, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py, src/loaded_cmj/v2/constants.py
- **LINE_OR_FUNCTION**: plant.py:285-300 (AABB from both foot geom centers, hx=0.150, hy=0.060); constants.py:183 (0.02 gate); full_closure.py:42 (0.05 gate)
- **DECLARED_BEHAVIOR**: Support margin = distance from COM-xy to the edge of the actual contact support polygon
- **IMPLEMENTED_BEHAVIOR**: The margin is computed from an axis-aligned box spanning BOTH foot geom centers with the FULL box half-extents; foot rotation is ignored, and inactive/airborne feet are included. The y-term is structurally ~0.155 m because the model has no lateral DOF.
- **REPRODUCTION_OR_PROOF**: Recomputation over all 121,889 samples using the production formula: minimum margin 0.0839 m; during true flight 0.0966-0.1285 m. Sample at t=1.06525: fzR=0 N (right foot airborne) yet margin=0.1463 m. True contact-row-based margin can be smaller.
- **NUMERIC_EVIDENCE**: `{"gate_E1_m": 0.02, "gate_RR_m": 0.05, "lateral_DOFs": 0, "margin_during_flight_m": [0.0966, 0.12852], "margin_min_all_samples_m": 0.08385}`
- **OBSERVED_EFFECT**: All margin-based predicates (E1 >=0.02 m, RR/tube/handoff >=0.05 m) are trivially satisfied; the metric cannot detect loss of support.
- **ROOT_CAUSE**: Support polygon approximated by AABB of full foot boxes, ignoring rotation and active contact.
- **R001_IMPACT**: R001 passes all margin gates without them carrying information.
- **SCIENTIFIC_CLAIM_IMPACT**: Any claim resting on 'COM inside the support polygon' is unsupported.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py (unhashed), support_margin semantics, E1/RR/tube gates, TRACE_SCHEMA_V2.md:61-62
- **DEPENDENCIES**: CRIT-003, MED-005
- **FIX_CLASS**: MEASUREMENT_CHANGE (compute the convex hull of active contact points; expose support active flags)
- **REQUIRED_REGRESSION_TEST**: Support-margin adversarial test: single-support and airborne states must not report positive margins; rotated-foot hull test.
- **REQUALIFICATION_SCOPE**: Measurement + qualification requalification; invalidates margin-based gate evidence.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md, probe_forces2.py output
- **PREFLIGHT_REFERENCES**: P12

## HIGH-009 — Inactive-foot geometric gap and normal velocity are wrong when the foot is rotated (up to 0.094 m)

- **SEVERITY**: HIGH
- **TYPE**: MEASUREMENT_BUG, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: tools/res52/core52.py, tools/res52/soft_contact.py, src/loaded_cmj/v2/balance_capture.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: core52.py:203-218, 226-232; soft_contact.py:85-97; balance_capture.py:63-67, 106-110; stable_recovery.py:179-185
- **DECLARED_BEHAVIOR**: Lowest foot material point used for the inactive-foot geometric gap and its normal velocity
- **IMPLEMENTED_BEHAVIOR**: dist = geom_xpos[2] - geom_size[2] (half thickness 0.010) and p_low = geom_xpos - [0,0,0.010], valid only when the foot is flat. The true lowest corner is geom center + R @ (±hx,±hy,±hz).
- **REPRODUCTION_OR_PROOF**: Deterministic probe on the frozen XML (probe A2): at ankle q=-0.70 the true lowest z is -0.0582 m vs code value +0.0360 m (error 0.0943 m); at q=+0.35 error 0.0508 m. Accepted R001 keeps feet nearly flat (max |foot pitch| 0.1384 rad at E9; <=0.004 rad during stance), so R001 is not exposed.
- **NUMERIC_EVIDENCE**: `{"foot_pitch_rad_R001_max": 0.1384, "gap_error_at_0.35rad_m": 0.0508, "gap_error_at_limit_m": 0.0943}`
- **OBSERVED_EFFECT**: For rotated feet the soft-contact layer can believe a foot is ~9 cm clear when it is near contact, corrupting re-engagement targets and normal-velocity rows.
- **ROOT_CAUSE**: Geometric lowest-point approximation assumes a flat foot; frozen in TRUE_FOOT_POINT_VELOCITY_CONTRACT.md and SOFT_CONTACT_STATE_CONTRACT.md.
- **R001_IMPACT**: Latent in R001 (flat feet); a successor with foot roll/heel-rise will be materially affected.
- **SCIENTIFIC_CLAIM_IMPACT**: Contracts that define 'lowest foot-box corner' with this formula are invalid for pitched feet.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: core52.py (unhashed), soft_contact.py (unhashed), RES-55 contract, RES-52 contract, support continuity liftoff evidence
- **DEPENDENCIES**: HIGH-008, CRIT-004
- **FIX_CLASS**: MEASUREMENT_CHANGE (true rotated-box lowest-corner via geom_xmat)
- **REQUIRED_REGRESSION_TEST**: Rotated-foot gap test: code gap must equal true lowest-corner height within 1e-6 m for ankle q in [-0.7,0.5].
- **REQUALIFICATION_SCOPE**: Measurement/soft-contact requalification; rotated-foot successors blocked until fixed.
- **EVIDENCE_POINTERS**: probe_geometry.py PROBE A2, CONTACT_REVIEW.md
- **PREFLIGHT_REFERENCES**: P11

## HIGH-010 — Declared genuine-flight geometric gap is never implemented (E7 checks force only)

- **SEVERITY**: HIGH
- **TYPE**: SCORER_SPEC_DEFECT, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: constants.py:198 (GENUINE_FLIGHT_GAP_M=0.010); events.py:211-216 (_guard_genuine_flight)
- **DECLARED_BEHAVIOR**: Genuine flight requires a minimum geometric foot-floor gap of 0.010 m
- **IMPLEMENTED_BEHAVIOR**: E7 guard checks only left_Fz<10 and right_Fz<10 for an 0.08 s dwell; no clearance geometry is consulted anywhere in the event detector.
- **REPRODUCTION_OR_PROOF**: grep: GENUINE_FLIGHT_GAP_M appears only at its definition; event sample contains no clearance field.
- **NUMERIC_EVIDENCE**: `{"GENUINE_FLIGHT_GAP_M": 0.01, "guard_inputs": ["left_Fz", "right_Fz"], "uses": 1}`
- **OBSERVED_EFFECT**: A 10 N force dropout of 0.08 s counts as 'genuine flight'; geometric clearance is uncertified.
- **ROOT_CAUSE**: Declared threshold never wired into the predicate.
- **R001_IMPACT**: R001 true flight is real (0.043-0.050 m clearance for ~0.223 s), so the accepted result is not false for this run; the specification is still false.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Genuine flight' claim rests on force thresholds, not geometry; contract wording is unsupported.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), constants.py, E7 definition, BILATERAL_SUPPORT_CONTINUITY_CONTRACT
- **DEPENDENCIES**: CRIT-003, HIGH-011
- **FIX_CLASS**: SCORER_SPEC_CHANGE (wire geometric clearance into E7 or remove the false declaration)
- **REQUIRED_REGRESSION_TEST**: Force-dropout-without-clearance negative control must not latch E7.
- **REQUALIFICATION_SCOPE**: Event requalification.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md, probe_forces2.py
- **PREFLIGHT_REFERENCES**: P06

## HIGH-011 — Apex has no dwell and its fallback path can latch without a flight check

- **SEVERITY**: HIGH
- **TYPE**: SCORER_SPEC_DEFECT, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: constants.py:199 (APEX_DWELL_S=0.005); events.py:56 (DWELL_S['apex']=0.0); events.py:408-439 (_detect_apex_crossing); events.py:430-436 (fallback)
- **DECLARED_BEHAVIOR**: Apex requires a dwell above/below the zero-crossing after genuine flight
- **IMPLEMENTED_BEHAVIOR**: DWELL_S['apex']=0.0 and WIN=1; APEX_DWELL_S is unused. _detect_apex_crossing first tries a two-sided flight check but then falls through to an unconditional sign-crossing path that does not verify either foot is unloaded.
- **REPRODUCTION_OR_PROOF**: Source inspection; first branch condition (lines 414-425) plus fallback at 430-436.
- **NUMERIC_EVIDENCE**: `{"APEX_DWELL_S": 0.005, "implemented_dwell_s": 0.0}`
- **OBSERVED_EFFECT**: Any com_vz sign crossing after E7 latches apex, including one during ground contact; apex time can be recorded without a physical apex.
- **ROOT_CAUSE**: Declared dwell not wired; fallback lacks the flight check.
- **R001_IMPACT**: R001 apex (t=0.771976) is inside true flight and plausible; the false-PASS path is latent.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Apex' event semantics are weaker than declared.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), E8 definition, constants.py
- **DEPENDENCIES**: HIGH-010, CRIT-003
- **FIX_CLASS**: SCORER_SPEC_CHANGE (implement dwell + mandatory flight condition)
- **REQUIRED_REGRESSION_TEST**: Contact-phase vz-crossing negative control must not latch E8.
- **REQUALIFICATION_SCOPE**: Event requalification.
- **EVIDENCE_POINTERS**: EVENT analysis in TEST_SUITE_FORENSIC_REPORT.md
- **PREFLIGHT_REFERENCES**: P07

## HIGH-012 — True-standing envelope is a one-trajectory overfit (micron-scale windows, one-U LP expansion)

- **SEVERITY**: HIGH
- **TYPE**: QUALIFICATION_DEFECT, SCORER_SPEC_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: constants.py:220-253 (V2_TRUE_STANDING_REFERENCE/ENVELOPE); events.py:259-390 (_is_true_standing_neighborhood)
- **DECLARED_BEHAVIOR**: Frozen empirical neighborhood for true standing (RES-16/RES-43)
- **IMPLEMENTED_BEHAVIOR**: Envelope derived from one deterministic 2 s standing hold: joint windows ~1e-5..6e-4 rad; COM z window ~3.2e-6 m; root z window ~2e-6 m; expanded by one ULP. E12/true-standing certification requires all 7 joints plus COM/root/trunk inside these windows.
- **REPRODUCTION_OR_PROOF**: Numeric inspection of constants.py; provenance comment states componentwise median over [0.100125, 2.0] from one run on the zero-damping plant.
- **NUMERIC_EVIDENCE**: `{"com_z_window_m": 3.220416e-06, "joint_window_examples_rad": [-0.0010646, 0.0001107], "root_z_window_m": 1.992e-06}`
- **OBSERVED_EFFECT**: A physically valid standing pose with slightly different numerical conditioning or a different step size would fail E12; the envelope certifies 'same trajectory', not 'standing'.
- **ROOT_CAUSE**: Envelope width driven by deterministic reproducibility rather than a physical stability region.
- **R001_IMPACT**: R001 passes E12 essentially by reproducing the reference hold's numerics.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Stable recovery' qualification is overfit; population/robustness generality unsupported.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: V2_TRUE_STANDING_ENVELOPE, RES-16/RES-43 authorities, E12 semantics
- **DEPENDENCIES**: HIGH-004, MED-013
- **FIX_CLASS**: QUALIFICATION_CHANGE (physical standing envelope with declared tolerance, or robustness replication)
- **REQUIRED_REGRESSION_TEST**: Multiple perturbed standing holds must pass/fail E12 consistently with the declared envelope; envelope width must be traceable to a robustness requirement.
- **REQUALIFICATION_SCOPE**: Recovery qualification; successor candidate.
- **EVIDENCE_POINTERS**: DEFECT_REGISTER.md#HIGH-012, recovery evidence
- **PREFLIGHT_REFERENCES**: P36

## HIGH-013 — Canonical output omits all jump-performance metrics and violates its own required result schema

- **SEVERITY**: HIGH
- **TYPE**: QUALIFICATION_DEFECT, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, CANONICAL_V2_RUNTIME_CONTRACT.md, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:501, 537 (only event_records extracted); canonical_runtime.py:540-590 (result dict); CANONICAL_V2_RUNTIME_CONTRACT.md:107-115 (required fields)
- **DECLARED_BEHAVIOR**: Canonical result exposes the declared schema including ENTRY_HEAD/TREE, hashes, CTRL_MODES, and the candidate's performance
- **IMPLEMENTED_BEHAVIOR**: Events raw_metrics (APEX_COM_Z, FLIGHT_RISE_FROM_TAKEOFF, TAKEOFF_VZ, FLIGHT_DURATION, COUNTERMOVEMENT_DEPTH, landing peaks, etc.) are computed in events.py but never serialized; ENTRY_HEAD/TREE, PLANT_SHA256, CONTROLLER_COMPOSITION_SHA256, SCORER_SHA256, CONTROL_DT_SEMANTICS, CTRL_MODES are absent from the JSON (CTRL_MODES exists only in the optional npz).
- **REPRODUCTION_OR_PROOF**: Code inspection + key listing of the sealed V2.1-R001_CANONICAL_RESULT.json.
- **NUMERIC_EVIDENCE**: `{"raw_metrics_serialized": false, "required_schema_fields_present": 0, "result_keys": "OUTCOME/TERMINATION/T_END/EVENTS/SUPPORT_*/... only"}`
- **OBSERVED_EFFECT**: A reader cannot distinguish a 3.6 cm hop from a functional CMJ from the canonical output alone; the qualification artifact cannot substantiate performance claims.
- **ROOT_CAUSE**: Metrics discarded at the serialization boundary; contract schema not implemented.
- **R001_IMPACT**: R001's canonical result is silent on the very quantities the owner judged visually.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Canonical qualification' artifact completeness claim is false; performance claims are unverifiable from the sealed output.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: CANONICAL_V2_RUNTIME_CONTRACT.md, canonical result schema, RES-12 qualification evidence
- **DEPENDENCIES**: CRIT-003, MED-011, MED-012
- **FIX_CLASS**: OUTPUT_SCHEMA_CHANGE (emit required fields + performance metrics + hashes)
- **REQUIRED_REGRESSION_TEST**: Schema conformance test against CANONICAL_V2_RUNTIME_CONTRACT.md; performance metrics presence test.
- **REQUALIFICATION_SCOPE**: Successor qualification output.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, canonical result JSON key listing
- **PREFLIGHT_REFERENCES**: P27, P28, D9

## HIGH-014 — Soft-contact FZ-min rows are silently dropped during LS refinement; active-set crossings are not handled in the derivative

- **SEVERITY**: HIGH
- **TYPE**: SOFTWARE_BUG, CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: tools/res52/soft_contact.py
- **LINE_OR_FUNCTION**: soft_contact.py:239-248 (FZ_MIN rows appended once); soft_contact.py:250-263 (refinement rebuilds base_rows + dist rows); soft_contact.py:182-192 (build_G crossing handling)
- **DECLARED_BEHAVIOR**: FZ_MIN constraints protect against losing plantar force during each control interval; G is a local model valid across contact-mode changes
- **IMPLEMENTED_BEHAVIOR**: On the first refinement pass rows are rebuilt as base_rows + distance rows, dropping the appended W_FZ_MIN rows (proved with an instrumented synthetic solve: pass 0 has 18 rows incl. FZ_MIN, passes 1-2 have 15 rows without). build_G increments active_set_crossings but still computes the two-sided secant across the contact-mode discontinuity.
- **REPRODUCTION_OR_PROOF**: probes/probe_fzmin.py (deterministic, monkeypatched lsq_linear) and source inspection.
- **NUMERIC_EVIDENCE**: `{"FZ_MIN_present_pass0": true, "FZ_MIN_present_pass1": false, "FZ_MIN_present_pass2": false, "crossing_handling": "counted only", "passes": 3}`
- **OBSERVED_EFFECT**: The terminal/balance inner layer may accept a solution that violates the min-force margin exactly when contact is marginal.
- **ROOT_CAUSE**: Refinement loop rebuilds the row set from base_rows only; crossing branches were never implemented in this module (later modules added one-sided fallbacks).
- **R001_IMPACT**: Latent in R001 (validation gates may catch gross violations); no proof that R001 was affected.
- **SCIENTIFIC_CLAIM_IMPACT**: Inner-solver force guarantees are weaker than documented; 'soft-contact force realization' claim is conditional.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: soft_contact.py (unhashed), RES-52 authority/spec, terminal/balance inner layer
- **DEPENDENCIES**: CRIT-004, HIGH-002, HIGH-003
- **FIX_CLASS**: SOFTWARE_FIX (rebuild rows including all constraint families; add crossing one-sided fallback)
- **REQUIRED_REGRESSION_TEST**: Unit test: FZ_MIN row must be present in every refinement pass; crossing test must be one-sided/zero but never a two-sided secant across modes.
- **REQUALIFICATION_SCOPE**: Soft-contact/terminal/balance requalification.
- **EVIDENCE_POINTERS**: probe_fzmin.py output, CONTROL_REVIEW.md
- **PREFLIGHT_REFERENCES**: P21, P22

## HIGH-015 — Legacy 'honest full jump' acceptance file is false assurance: 10 assert-True stubs and >=9 events accepted as 12/12

- **SEVERITY**: HIGH
- **TYPE**: TEST_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: tests/test_v2_1_res10_honest_full_jump.py
- **LINE_OR_FUNCTION**: lines 1, 100, 119, 135, 154, 157, 162, 179, 208, 211, 219 (assert True); lines 137-145, 164-176 (>=9 events); lines 18, 186 (legacy controller)
- **DECLARED_BEHAVIOR**: 25 checks certifying the honest full jump
- **IMPLEMENTED_BEHAVIOR**: 10 of 25 tests are `assert True`; test_e12/test_event_count/test_no_fall accept >=9 events; test_no_fall cannot detect a fall; all runs use the legacy module-level loaded_cmj.v2.controller, not the canonical composition.
- **REPRODUCTION_OR_PROOF**: Read of the full file; count of assert True across tests/ = 10 (all in this file).
- **NUMERIC_EVIDENCE**: `{"assert_true": 10, "canonical_composition_used": false, "min_events_accepted": 9, "tests_total": 25}`
- **OBSERVED_EFFECT**: A run that stops at E9 or falls can pass the tests whose names claim E12/no-fall.
- **ROOT_CAUSE**: Mission-closure leniency was codified into the test file.
- **R001_IMPACT**: R001's own evidence does not come from this file, but the repo's V2.1 test narrative rests on it.
- **SCIENTIFIC_CLAIM_IMPACT**: The test-based '12/12' narrative is not evidence of task success.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: test suite credibility, V2.1 qualification narrative
- **DEPENDENCIES**: HIGH-016, HIGH-017
- **FIX_CLASS**: TEST_FIX (replace placeholders with behavioral invariants; test the canonical composition)
- **REQUIRED_REGRESSION_TEST**: Mutation test: a trajectory truncated at E9 must fail every test whose name claims E12/stable recovery.
- **REQUALIFICATION_SCOPE**: Test suite requalification.
- **EVIDENCE_POINTERS**: TEST_SUITE_FORENSIC_REPORT.md, TEST_FILE_INVENTORY.csv
- **PREFLIGHT_REFERENCES**: P42

## HIGH-016 — Seven assertions are neutralized by `or True` (unconditional pass) and one test body is a bare pass

- **SEVERITY**: HIGH
- **TYPE**: TEST_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: tests/test_v2_1_res31_honest_planar_root.py, tests/test_res10_controller_obs_sync.py, tests/test_res11_deterministic_offline.py, tests/test_v2_1_res6_compliant_landing_contact.py, tests/test_res52_soft_contact.py, tests/test_res76_full_closure.py, tests/test_res73_balance_capture.py, tests/test_v2_1_res8_captured_squat.py
- **LINE_OR_FUNCTION**: test_v2_1_res31_honest_planar_root.py:197; test_res10_controller_obs_sync.py:199; test_res11_deterministic_offline.py:91; test_v2_1_res6_compliant_landing_contact.py:296; test_res52_soft_contact.py:372; test_res76_full_closure.py:55; test_res73_balance_capture.py:143; test_v2_1_res8_captured_squat.py:167-170 (pass body); test_rec01a_synchronized_dynamics.py:208-234 (max(abs)>=0); test_res52_soft_contact.py:288-295 (monotone counter)
- **DECLARED_BEHAVIOR**: Named scientific invariants are actually asserted
- **IMPLEMENTED_BEHAVIOR**: Each listed assertion parses as `(condition) or True` (or has a body of pass / a monotonicity check that cannot decrease), so the claimed condition can never fail.
- **REPRODUCTION_OR_PROOF**: Read of exact lines; count `or True` in tests/ = 7; pass-only test bodies located.
- **NUMERIC_EVIDENCE**: `{"always_true_asserts": 2, "or_true_count": 7, "pass_only_tests": 1}`
- **OBSERVED_EFFECT**: Named checks for scorer circularity, root contact parameters, live-forward purity, E11-alone sufficiency and fall timing cannot detect regressions.
- **ROOT_CAUSE**: Debug-time neutralization left in place.
- **R001_IMPACT**: R001 is not certified by these checks regardless of their names.
- **SCIENTIFIC_CLAIM_IMPACT**: Test coverage claims for these invariants are void.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: test suite credibility
- **DEPENDENCIES**: HIGH-015, HIGH-017
- **FIX_CLASS**: TEST_FIX (restore real assertions; keep intent but make them failure-capable)
- **REQUIRED_REGRESSION_TEST**: Each restored assertion must be shown to fail on a deliberately mutated implementation.
- **REQUALIFICATION_SCOPE**: Test suite requalification.
- **EVIDENCE_POINTERS**: TEST_SUITE_FORENSIC_REPORT.md
- **PREFLIGHT_REFERENCES**: P42

## HIGH-017 — No test executes the canonical composition; the suite validates stored JSON and breaks at collection

- **SEVERITY**: HIGH
- **TYPE**: TEST_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: tests/test_res78_canonical_runtime.py, tests/test_ml241_qacc_resolution.py, tests/test_public_support_wrench_contract.py
- **LINE_OR_FUNCTION**: test_res78_canonical_runtime.py:52-56 (_canon_result reads stored JSON); :87 (only callability of run_canonical_episode); :215-223 (reads external RUN_A/B); test_ml241_qacc_resolution.py:21 (import-time external evidence path); test_public_support_wrench_contract.py:26 (missing symbol)
- **DECLARED_BEHAVIOR**: The canonical runtime is tested end-to-end
- **IMPLEMENTED_BEHAVIOR**: The only invocation of run_canonical_episode in tests is a callable() check; events/checkpoints are asserted against a stored result JSON; two tracked test modules fail at collection (missing external evidence file; ImportError of shift_wrench_to_origin).
- **REPRODUCTION_OR_PROOF**: pytest --collect-only run during the audit: `Interrupted: 2 errors during collection`.
- **NUMERIC_EVIDENCE**: `{"live_canonical_episode_tests": 0, "pytest_collect_errors": 2, "stored_result_assertions": 13}`
- **OBSERVED_EFFECT**: A stale but self-consistent evidence bundle passes; the suite cannot reproduce the candidate on a clean machine.
- **ROOT_CAUSE**: Acceptance anchored to external evidence artifacts instead of executing the composition.
- **R001_IMPACT**: R001's tests certify the stored artifact, not the current source.
- **SCIENTIFIC_CLAIM_IMPACT**: The test suite cannot substantiate current-code qualification.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: test suite credibility, canonical runtime, clean-clone reproducibility
- **DEPENDENCIES**: HIGH-015, HIGH-018, CRIT-006
- **FIX_CLASS**: TEST_FIX (one bounded live smoke run or explicit artifact-identity binding; fix collection failures)
- **REQUIRED_REGRESSION_TEST**: Fresh-process canonical smoke test with hash binding; collection must succeed on a clean clone without external evidence.
- **REQUALIFICATION_SCOPE**: Test suite + qualification requalification.
- **EVIDENCE_POINTERS**: TEST_SUITE_FORENSIC_REPORT.md, pytest_collect.txt transcript
- **PREFLIGHT_REFERENCES**: P42

## HIGH-018 — Qualification orchestration omits two scripts (one hardcodes PASS); test_public_qualification runs only 10 of 19 suites

- **SEVERITY**: HIGH
- **TYPE**: TEST_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: tests/test_public_qualification.py, tests/qualification_causal_runtime.py, tests/qualification_ml241_wrapped_derivatives.py
- **LINE_OR_FUNCTION**: test_public_qualification.py:14-25 (QUALIFICATION_SUITES), :49 (PASS check exempts policy isolation); qualification_causal_runtime.py:470-497 (result hardcoded PASS; return 0); qualification_ml241_wrapped_derivatives.py:835 (return 0 unconditional)
- **DECLARED_BEHAVIOR**: The public qualification entry point runs all qualification suites and gates on their PASS/FAIL
- **IMPLEMENTED_BEHAVIOR**: Only 10 of 19 qualification scripts are invoked; qualification_causal_runtime.py writes result=PASS and exits 0 unconditionally and is not invoked by the orchestrator; qualification_ml241_wrapped_derivatives.py also returns 0 regardless of evidence.
- **REPRODUCTION_OR_PROOF**: Read of the orchestrator list and the two scripts' exit paths.
- **NUMERIC_EVIDENCE**: `{"invoked": 10, "qualification_scripts_total": 19, "ungated_pass_scripts": 2}`
- **OBSERVED_EFFECT**: Two 'proof producer' scripts cannot fail; the qualification suite's PASS surface is incomplete.
- **ROOT_CAUSE**: Orchestration list stale; results hardcoded during development.
- **R001_IMPACT**: Not part of R001's V2.1 path directly, but part of the repo's qualification credibility.
- **SCIENTIFIC_CLAIM_IMPACT**: Repository-wide qualification status is not defensible.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: public qualification, test suite credibility
- **DEPENDENCIES**: HIGH-017
- **FIX_CLASS**: TEST_FIX (include all suites; remove hardcoded PASS; gate on exit codes and evidence)
- **REQUIRED_REGRESSION_TEST**: Orchestrator coverage test: enumerated scripts == executed scripts; each script must fail on a mutated input.
- **REQUALIFICATION_SCOPE**: Qualification orchestration requalification.
- **EVIDENCE_POINTERS**: TEST_SUITE_FORENSIC_REPORT.md
- **PREFLIGHT_REFERENCES**: P42

## HIGH-019 — Rigid single-box foot: no MTP/toe/arch; R001 push-off is flat-footed with no heel rise

- **SEVERITY**: HIGH
- **TYPE**: MODEL_FORM_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/assets/v2_plant.xml
- **LINE_OR_FUNCTION**: v2_plant.xml:105-109, 133-137 (single box; ankle only); v2_plant.xml:106, 134 (ankle range -0.70..0.50)
- **DECLARED_BEHAVIOR**: Foot mechanics adequate for a loaded CMJ push-off (ankle plantarflexion/forefoot rocker)
- **IMPLEMENTED_BEHAVIOR**: One rigid box per foot attached at the ankle; no metatarsophalangeal joint, no toe segment, no arch. Geometric foot pitch during R001 stance is <=0.004 rad (takeoff 0.0043 rad) and never rolls; peak |pitch| 0.1384 rad during landing is the entire range.
- **REPRODUCTION_OR_PROOF**: geometry probe A2 + R001 foot-pitch computed from geom_xmat over all 121,889 samples (probe_footpitch.py).
- **NUMERIC_EVIDENCE**: `{"MTP_joints": 0, "R001_max_abs_pitch_rad": 0.1384, "pitch_at_E6_rad": 0.0043, "stance_max_abs_pitch_rad": 0.00397, "toe_segments": 0}`
- **OBSERVED_EFFECT**: No forefoot rocker; the push-off is a flat-foot knee/hip extension; toe-off mechanics observed as 'clearly inadequate' are structural, not a controller tuning issue.
- **ROOT_CAUSE**: Foot model form; single hinge with no segmentation.
- **R001_IMPACT**: R001's takeoff lacks human ankle/forefoot contribution.
- **SCIENTIFIC_CLAIM_IMPACT**: 'Loaded CMJ' human foot mechanics claim unsupported; acceptable only under an explicitly reduced-model claim.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: Plant XML, foot geometry, contact model
- **DEPENDENCIES**: CRIT-001, HIGH-009
- **FIX_CLASS**: MODEL_FORM_CHANGE (add MTP/toe segmentation or declare the limitation and exclude push-off claims)
- **REQUIRED_REGRESSION_TEST**: Foot-roll/heel-rise capability test with declared geoms and joint; push-off visual/physics coherence review.
- **REQUALIFICATION_SCOPE**: Successor plant + full requalification.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md, probe_footpitch.py
- **PREFLIGHT_REFERENCES**: P10, P45-adjacent

## MED-001 — Systematic dwell off-by-one: every dwell event confirms one physics sample (0.125 ms) before the declared duration

- **SEVERITY**: MEDIUM
- **TYPE**: SCORER_SPEC_DEFECT, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: events.py:600-601, 660-661, 701-702, 738-739, 776-777, 831-832 (elapsed=idx-start+1); events.py:64-68 (_win)
- **DECLARED_BEHAVIOR**: Dwell of D seconds requires D seconds of sustained condition
- **IMPLEMENTED_BEHAVIOR**: Each event latches when idx-start+1 >= round(D/DT), i.e. after (n-1)*DT physical time.
- **REPRODUCTION_OR_PROOF**: Sealed event records: all 10 dwell events confirm exactly one sample early (0.099875 vs 0.10, 0.029875 vs 0.03, 0.009875 vs 0.01, 0.049875 vs 0.05, 0.079875 vs 0.08, 0.019875 vs 0.02, 0.149875 vs 0.15, 0.499875 vs 0.50).
- **NUMERIC_EVIDENCE**: `{"affected_events": 10, "dwell_errors_s": -0.000125}`
- **OBSERVED_EFFECT**: Systematic but numerically negligible timing shift (0.125 ms) in event certificates.
- **ROOT_CAUSE**: Sample counting treats the first sample as elapsed time.
- **R001_IMPACT**: R001 event times are each one sample early relative to declared dwells.
- **SCIENTIFIC_CLAIM_IMPACT**: Event timing certificates are off by one physics sample; material only for tight dwell semantics.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), all event times
- **DEPENDENCIES**: HIGH-011, CRIT-003
- **FIX_CLASS**: SOFTWARE_FIX (require idx-start >= n or document sample-count dwell semantics)
- **REQUIRED_REGRESSION_TEST**: Dwell boundary unit test at the exact declared duration.
- **REQUALIFICATION_SCOPE**: Event requalification (numeric shift only).
- **EVIDENCE_POINTERS**: EVENT adjudication table in DEFECT_REGISTER.md
- **PREFLIGHT_REFERENCES**: P29

## MED-002 — E11 threshold is named COM_SPEED but implemented on com_vz only

- **SEVERITY**: MEDIUM
- **TYPE**: SCORER_SPEC_DEFECT, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/events.py, src/loaded_cmj/v2/constants.py
- **LINE_OR_FUNCTION**: events.py:247-257 (_guard_balance_capture); constants.py:204 (BALANCE_CAPTURE_COM_SPEED_MPS)
- **DECLARED_BEHAVIOR**: Balance capture requires CoM speed below 0.30 m/s (bidirectional)
- **IMPLEMENTED_BEHAVIOR**: Guard checks abs(com_vz) < 0.30; horizontal speed is never used.
- **REPRODUCTION_OR_PROOF**: Source inspection; in R001 both variants cross 0.30 at the same sample (t=0.990), so the defect is latent here.
- **NUMERIC_EVIDENCE**: `{"R001_first_speed_below_threshold_s": 0.99, "R001_first_vz_below_threshold_s": 0.99, "speed_at_E11_occ_mps": 0.2792}`
- **OBSERVED_EFFECT**: A candidate with large horizontal speed but small vertical speed would latch E11 early.
- **ROOT_CAUSE**: Name/spec mismatch: speed versus vertical component.
- **R001_IMPACT**: No R001 impact (coincident thresholds).
- **SCIENTIFIC_CLAIM_IMPACT**: E11 semantics weaker than named; latent false-PASS for horizontal-momentum landings.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), E11 definition
- **DEPENDENCIES**: HIGH-001
- **FIX_CLASS**: SCORER_SPEC_FIX (use horizontal or total CoM speed as declared)
- **REQUIRED_REGRESSION_TEST**: E11 negative control with |vz|<0.30 but speed>0.30.
- **REQUALIFICATION_SCOPE**: Event requalification.
- **EVIDENCE_POINTERS**: probe_landing.py, DEFECT_REGISTER.md#MED-002
- **PREFLIGHT_REFERENCES**: P09

## MED-003 — Pelvis orientation observation is a hardcoded identity quaternion

- **SEVERITY**: MEDIUM
- **TYPE**: MEASUREMENT_BUG, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py, src/loaded_cmj/v2/measurement.py
- **LINE_OR_FUNCTION**: plant.py:362-364; measurement.py:249
- **DECLARED_BEHAVIOR**: pelvis_orientation_world_quat_wxyz is the true world orientation of the pelvis (root_ry pitch)
- **IMPLEMENTED_BEHAVIOR**: Both observation builders return [1,0,0,0] unconditionally; root pitch is available in qpos/root_ry but is not exposed as a quaternion.
- **REPRODUCTION_OR_PROOF**: Source read; root pitch reaches 0.4315 rad at RR entry, so the reported quaternion is wrong whenever the body pitches.
- **NUMERIC_EVIDENCE**: `{"reported_quat": [1, 0, 0, 0], "root_pitch_max_rad_in_R001": 0.5918}`
- **OBSERVED_EFFECT**: Any consumer of the declared observation field receives a false pelvis orientation; V2.1 controllers do not consume this field, so R001 control is unaffected.
- **ROOT_CAUSE**: Placeholder left in the observation implementation.
- **R001_IMPACT**: R001 controllers unaffected; observation contract broken for external consumers.
- **SCIENTIFIC_CLAIM_IMPACT**: CANONICAL observation contract ('pelvis orientation') is not implemented.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py, measurement.py (CONTROLLER_OBSERVATION_CONTRACT), observation schema
- **DEPENDENCIES**: MED-004
- **FIX_CLASS**: MEASUREMENT_FIX (compose the root_ry quaternion)
- **REQUIRED_REGRESSION_TEST**: Observation test: quaternion must match MuJoCo pelvis xmat/quat under nonzero root pitch.
- **REQUALIFICATION_SCOPE**: Measurement requalification.
- **EVIDENCE_POINTERS**: DEFECT_REGISTER.md#MED-003
- **PREFLIGHT_REFERENCES**: P18

## MED-004 — Trunk tilt measurement is unsigned

- **SEVERITY**: MEDIUM
- **TYPE**: MEASUREMENT_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: plant.py:178-185 (arccos of torso z-axis); events.py:395 (abs(trunk_tilt))
- **DECLARED_BEHAVIOR**: Signed trunk pitch (forward/backward distinguishable)
- **IMPLEMENTED_BEHAVIOR**: trunk_tilt_rad is arccos(up_world[2]) in [0, pi], losing sign; event predicates then take abs().
- **REPRODUCTION_OR_PROOF**: Source + R001 trace: at E4 the trunk is tilted backward ~18 deg but reports +0.31 rad; at E11 forward ~25 deg reports +0.437.
- **NUMERIC_EVIDENCE**: `{"forward_at_E11_deg": 25.0, "reported_rad": 0.437, "trunk_tilt_backward_at_E4_deg": -18.0}`
- **OBSERVED_EFFECT**: Backward and forward lean are indistinguishable; guards that already use abs() are unaffected, but claims about posture direction cannot be made.
- **ROOT_CAUSE**: Unsigned angle by construction.
- **R001_IMPACT**: R001 gate behavior unaffected; posture forensics limited.
- **SCIENTIFIC_CLAIM_IMPACT**: Posture-direction claims unsupported by measurement.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py, measurement.py, event predicates
- **DEPENDENCIES**: HIGH-004
- **FIX_CLASS**: MEASUREMENT_FIX (atan2 signed pitch)
- **REQUIRED_REGRESSION_TEST**: Signed-tilt test under forward and backward lean.
- **REQUALIFICATION_SCOPE**: Measurement/event requalification.
- **EVIDENCE_POINTERS**: DEFECT_REGISTER.md#MED-004
- **PREFLIGHT_REFERENCES**: P17

## MED-005 — CoP frame/origin is misdeclared and its validity threshold disagrees with the declared constant

- **SEVERITY**: MEDIUM
- **TYPE**: MEASUREMENT_BUG, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py, TRACE_SCHEMA_V2.md, src/loaded_cmj/v2/constants.py
- **LINE_OR_FUNCTION**: plant.py:263-277 (origin = foot body xpos xy at z=0; validity Fz>20); constants.py:180 (V2_CONTACT_FZ_THRESHOLD_N=10); TRACE_SCHEMA_V2.md:51
- **DECLARED_BEHAVIOR**: plantar_cop_xy_m in the plate frame with the documented validity threshold
- **IMPLEMENTED_BEHAVIOR**: CoP is computed relative to the moving ankle projection, not the plate/box center; validity uses Fz>20 N while the named per-foot threshold is 10 N; no origin is exposed; no production controller consumes the measured CoP (analytic clamps are used instead).
- **REPRODUCTION_OR_PROOF**: Source read; numeric example: flat stance CoP should be at box center (0.045, +-0.095) but is reported as (0.045, 0) relative to the ankle.
- **NUMERIC_EVIDENCE**: `{"declared_threshold_n": 10.0, "origin": "ankle xy projection", "validity_threshold_implemented_n": 20.0}`
- **OBSERVED_EFFECT**: CoP evidence is frame-ambiguous and cannot be compared across samples without the foot pose; external reuse is error-prone.
- **ROOT_CAUSE**: Origin choice and magic threshold not tied to the authority constant.
- **R001_IMPACT**: R001: CoP not used by control; evidence semantics only.
- **SCIENTIFIC_CLAIM_IMPACT**: CoP-based claims (support/interaction) are unsupported as reported.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py, TRACE_SCHEMA_V2.md, V2_CONTACT_FZ_THRESHOLD_N
- **DEPENDENCIES**: HIGH-008, MED-011
- **FIX_CLASS**: MEASUREMENT_FIX (define origin per plate center, expose origin, unify threshold)
- **REQUIRED_REGRESSION_TEST**: Frame-identity test: CoP + origin + force must reconstruct the world wrench at the contact plane.
- **REQUALIFICATION_SCOPE**: Measurement requalification.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md
- **PREFLIGHT_REFERENCES**: P13

## MED-006 — prohibited_contact is structurally always False; PROHIB=false in the canonical result carries no information

- **SEVERITY**: MEDIUM
- **TYPE**: MEASUREMENT_BUG, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py, src/loaded_cmj/v2/canonical_runtime.py
- **LINE_OR_FUNCTION**: plant.py:252-261 (shell geoms scan); v2_plant.xml:42-45 (shell contype=0 conaffinity=0); canonical_runtime.py:444,561
- **DECLARED_BEHAVIOR**: prohibited_contact detects non-plantar body support
- **IMPLEMENTED_BEHAVIOR**: The scanned shell geoms can never generate contacts (contype/conaffinity zero), so the flag is always False; the only real fall signal is fall_contact (fall shells contype=4 vs floor).
- **REPRODUCTION_OR_PROOF**: Source + wheel/model inspection; also noted in tests/test_v2_1_res5_honest_fall.py:229-231.
- **NUMERIC_EVIDENCE**: `{"prohibited_true_possible": false}`
- **OBSERVED_EFFECT**: A false assurance field: 'no prohibited contact' means only 'no impossible contact could be detected'.
- **ROOT_CAUSE**: Body shell collision disabled by design; the flag was left scanning disabled geoms.
- **R001_IMPACT**: R001 PROHIB=false is vacuous; FALL=false is the meaningful signal.
- **SCIENTIFIC_CLAIM_IMPACT**: Prohibited-contact certification is void.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py, canonical result PROHIB field, TRACE_SCHEMA_V2.md prohibited_flag
- **DEPENDENCIES**: MED-007
- **FIX_CLASS**: MEASUREMENT_FIX (scan actual load/body colliders or remove the field and its claims)
- **REQUIRED_REGRESSION_TEST**: Injected fall-contact test must raise prohibited/fall flags.
- **REQUALIFICATION_SCOPE**: Measurement/qualification requalification.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md
- **PREFLIGHT_REFERENCES**: P30-adjacent

## MED-007 — Collision topology permits anatomically impossible intersections and the 20 kg load has no collision geometry at all

- **SEVERITY**: MEDIUM
- **TYPE**: MODEL_FORM_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/assets/v2_plant.xml
- **LINE_OR_FUNCTION**: v2_plant.xml:42-45 (shell class 0/0); v2_plant.xml:64-83 (pelvis/torso/load shells); v2_plant.xml:143-153 (contact excludes)
- **DECLARED_BEHAVIOR**: Body self-collision and load collision reasonably approximated
- **IMPLEMENTED_BEHAVIOR**: All body shells are non-colliding; the load cylinder has no fall shell; adjacent-segment pairs and pelvis-torso/pelvis-load/torso-load are excluded. Self-overlap is kinematically possible in deep flexion/falls and generates no contact. Foot-shank exclusion has ~6 mm minimum clearance.
- **REPRODUCTION_OR_PROOF**: Contact-matrix analysis and geometry probe; fall states are only detected by the six fall shells vs floor.
- **NUMERIC_EVIDENCE**: `{"fall_shells": 6, "load_collision_geoms": 0, "self_collision": false}`
- **OBSERVED_EFFECT**: In fall/landing-collapse states the model can show impossible interpenetration or a load passing through the floor without detection.
- **ROOT_CAUSE**: Reduced collision model for robustness; load modeled as inertia only.
- **R001_IMPACT**: R001 (flat, symmetric, no fall) is not affected.
- **SCIENTIFIC_CLAIM_IMPACT**: Fall-state and load-interaction claims unsupported; visual credibility risk in successors.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: Plant XML, fall detection, load model
- **DEPENDENCIES**: HIGH-019, MED-006
- **FIX_CLASS**: MODEL_FORM_CHANGE (add minimal load/self colliders or declare the limitation and forbid fall-state claims)
- **REQUIRED_REGRESSION_TEST**: Fall-state interpenetration audit; load-floor collision detection test.
- **REQUALIFICATION_SCOPE**: Successor plant; fall requalification.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md
- **PREFLIGHT_REFERENCES**: P30

## MED-008 — FLEX->EXTEND program switch is a hard scheduled step (0.568) unrelated to state

- **SEVERITY**: MEDIUM
- **TYPE**: CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: res72_integration.py:59-60 (FLEX_TAU/EXTEND_TAU); res72_integration.py:191-200 (t_in<0.30 switch)
- **DECLARED_BEHAVIOR**: Smooth countermovement-to-extension transition
- **IMPLEMENTED_BEHAVIOR**: Torque program switches at t_in=0.30 from +20 Nm to -75/-150 Nm with no slew or state condition; observed |du|=0.568 in one 5 ms interval at t=0.400.
- **REPRODUCTION_OR_PROOF**: Sealed action schedule analysis.
- **NUMERIC_EVIDENCE**: `{"max_du": 0.568, "switch_t_s": 0.4, "tau_step_hip_nm": 95.0, "tau_step_knee_nm": 170.0}`
- **OBSERVED_EFFECT**: An impulsive extension onset at a fixed time rather than a state-triggered reversal; contributes to the mechanical feel.
- **ROOT_CAUSE**: Time-programmed controller with no state-feedback termination.
- **R001_IMPACT**: R001 extension onset is a fixed-time torque step; contributes to the non-human transition feel.
- **SCIENTIFIC_CLAIM_IMPACT**: Countermovement-to-extension timing is not a physical reversal detection; technique interpretation limited.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: res72_integration.py, confirmation timing
- **DEPENDENCIES**: HIGH-006, CRIT-007
- **FIX_CLASS**: CONTROL_CHANGE (state-based reversal detection/slew)
- **REQUIRED_REGRESSION_TEST**: Program-switch continuity test bounded by declared slew.
- **REQUALIFICATION_SCOPE**: Controller requalification.
- **EVIDENCE_POINTERS**: CONTROL_REVIEW.md, probe_skeleton.py output
- **PREFLIGHT_REFERENCES**: P03

## MED-009 — Post-projection clamps can leave the CoP/Fx/Hdot triple off the feasible line

- **SEVERITY**: MEDIUM
- **TYPE**: SOFTWARE_BUG, CONTROL_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/balance_capture.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: balance_capture.py:149-175; stable_recovery.py:307-325
- **DECLARED_BEHAVIOR**: Fx/Hdot are projected onto the CoP-feasible line before being commanded
- **IMPLEMENTED_BEHAVIOR**: After the weighted projection, Fx is re-clipped to the brake bounds and then Hdot is derived and clipped; if the Hdot clip binds, Fx is recomputed and clipped again, so the final pair can be off the line. COP_X bounds are static and unrelated to actual foot pose.
- **REPRODUCTION_OR_PROOF**: Line-by-line control-flow analysis of both implementations.
- **NUMERIC_EVIDENCE**: `{"residual_offline_error_bound": "~cz*|dFx|/Fz", "static_cop_bounds_m": [-0.023, 0.237]}`
- **OBSERVED_EFFECT**: Bounded feasibility error in the outer wrench demand; inner layer limits physical consequence.
- **ROOT_CAUSE**: Iterative clamp without a final joint projection.
- **R001_IMPACT**: No demonstrated R001 effect; latent.
- **SCIENTIFIC_CLAIM_IMPACT**: CoP-feasibility guarantee is approximate, not exact.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: balance_capture.py, stable_recovery.py
- **DEPENDENCIES**: HIGH-003
- **FIX_CLASS**: SOFTWARE_FIX (single final projection onto the feasible set)
- **REQUIRED_REGRESSION_TEST**: Property test: commanded (Fx,Hdot,Fz) always satisfies the CoP line within tolerance.
- **REQUALIFICATION_SCOPE**: Balance/recovery requalification.
- **EVIDENCE_POINTERS**: CONTROL_REVIEW.md
- **PREFLIGHT_REFERENCES**: P24

## MED-010 — Legacy controller swallows all exceptions and commands zero action

- **SEVERITY**: MEDIUM
- **TYPE**: SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/controller.py
- **LINE_OR_FUNCTION**: controller.py:127-128 (bare except -> [0]*7)
- **DECLARED_BEHAVIOR**: Controller failures are surfaced
- **IMPLEMENTED_BEHAVIOR**: Any exception in act() (including malformed observation, nonfinite state) results in a silent all-zero action.
- **REPRODUCTION_OR_PROOF**: Source read; module is not used by the canonical composition but is exercised by multiple tests.
- **NUMERIC_EVIDENCE**: `{"bare_except": true, "failure_action": "[0]*7"}`
- **OBSERVED_EFFECT**: Silent zero-torque failure is indistinguishable from a valid command, producing falls without diagnostics.
- **ROOT_CAUSE**: Catch-all for robustness.
- **R001_IMPACT**: No R001 production impact (legacy module); latent test/legacy risk.
- **SCIENTIFIC_CLAIM_IMPACT**: Legacy controller qualification claims unsupported.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: controller.py, legacy tests
- **DEPENDENCIES**: MED-019
- **FIX_CLASS**: SOFTWARE_FIX (fail loudly or return explicit fallback state)
- **REQUIRED_REGRESSION_TEST**: Malformed-observation test must surface an error, not zeros.
- **REQUALIFICATION_SCOPE**: Legacy module deprecation or requalification.
- **EVIDENCE_POINTERS**: BUG_HUNT.md
- **PREFLIGHT_REFERENCES**: P37

## MED-011 — Reporting metrics are misdefined or dead: loading rate, apex_height alias, CoP invalid-interval analysis

- **SEVERITY**: MEDIUM
- **TYPE**: MEASUREMENT_BUG, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/events.py, src/loaded_cmj/v2/measurement.py
- **LINE_OR_FUNCTION**: events.py:1133-1136 (loading_rate = peak/(t_peak-land_time)); events.py:1078 (apex_height = apex_z absolute); events.py:1224-1240 (CoP invalid intervals loop/dead variable); measurement.py:279-305 (event_sample lacks CoP keys)
- **DECLARED_BEHAVIOR**: Loading rate is a force rise rate; apex_height is jump height; CoP validity intervals are reported
- **IMPLEMENTED_BEHAVIOR**: loading_rate is a secant peak/time value ignoring the starting force and is not a maximum slope; apex_height equals absolute COM z (FLIGHT_RISE_FROM_TAKEOFF carries the rise); the CoP interval loop breaks after the first sample and its result is unused, and event samples carry no CoP validity keys so the branch is unreachable.
- **REPRODUCTION_OR_PROOF**: Source read + key inspection of event_sample output.
- **NUMERIC_EVIDENCE**: `{"apex_height": "absolute COM z", "cop_interval_reported": false, "loading_rate_definition": "primary_peak/max(dt_peak,1e-9)"}`
- **OBSERVED_EFFECT**: Evidence reports can state misleading numbers; CoP handling claims in metrics are unsupported.
- **ROOT_CAUSE**: Metric definitions not reconciled with their names; event sample never industrialized.
- **R001_IMPACT**: No effect on R001 control; reporting only.
- **SCIENTIFIC_CLAIM_IMPACT**: Performance/loading claims derived from these fields are unreliable.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), measurement.event_sample, canonical output
- **DEPENDENCIES**: HIGH-013
- **FIX_CLASS**: REPORTING_FIX (define each metric, wire CoP validity, rename/derive jump height)
- **REQUIRED_REGRESSION_TEST**: Metric definition tests against a synthetic force trace; CoP validity interval presence test.
- **REQUALIFICATION_SCOPE**: Scorer reporting requalification.
- **EVIDENCE_POINTERS**: DEFECT_REGISTER.md#MED-011
- **PREFLIGHT_REFERENCES**: P40, P41, P28

## MED-012 — Full-trajectory support adjudication is NOT_QUALIFIED while reported gates use the post-landing slice only

- **SEVERITY**: MEDIUM
- **TYPE**: QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/support_continuity.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:459-461 (SUPPORT_FULL); :482-499 (sliced adjudications); :562-567 (result fields)
- **DECLARED_BEHAVIOR**: Support-continuity contract applies to the trajectory; qualification gates reflect it
- **IMPLEMENTED_BEHAVIOR**: SUPPORT_FULL reports NOT_QUALIFIED (CONTROL_RELEVANT_SUPPORT_LOSS_COUNT=2, CANONICAL_REFLIGHT=[4,8,1815], CHATTER_TRANSITIONS=188) because flight is counted as reflight; the declared hard gates use SUPPORT_POST_LANDING (from t=0.875) which is QUALIFIED.
- **REPRODUCTION_OR_PROOF**: Sealed canonical result SUPPORT_FULL / SUPPORT_POST_LANDING blocks.
- **NUMERIC_EVIDENCE**: `{"SUPPORT_FULL": "NOT_QUALIFIED", "SUPPORT_POST_LANDING": "QUALIFIED", "chatter": 188, "reflights": [4, 8, 1815]}`
- **OBSERVED_EFFECT**: The qualification gate set is a slice choice; the full-contract verdict is negative and this is not surfaced in the receipt.
- **ROOT_CAUSE**: Flight windows inherently produce reflight under the full-trajectory predicate; gating was scoped to post-landing.
- **R001_IMPACT**: R001 passes the scoped gates; the full adjudication would fail.
- **SCIENTIFIC_CLAIM_IMPACT**: Support-continuity claim must be stated as post-landing only; the full verdict is negative.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: support_continuity.py, post-landing gates, RES-57 evidence
- **DEPENDENCIES**: HIGH-008, HIGH-013
- **FIX_CLASS**: QUALIFICATION_CHANGE (state and justify the exact adjudicated window in the contract and receipt)
- **REQUIRED_REGRESSION_TEST**: Contract test: the reported gate window must match the declared contract scope.
- **REQUALIFICATION_SCOPE**: Qualification reporting.
- **EVIDENCE_POINTERS**: support_continuity output in canonical result

## MED-013 — E12 fires 50.8 ms before the RES-43 handoff; part of its dwell is earned under SETTLE, not the final hold

- **SEVERITY**: MEDIUM
- **TYPE**: QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/stable_recovery.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:270-273 (handoff capture), :428-430 (E12 pre-handoff calc); stable_recovery.py:580-586 (SETTLE->HANDOFF)
- **DECLARED_BEHAVIOR**: Stable recovery is certified under the final standing hold
- **IMPLEMENTED_BEHAVIOR**: E12 occurrence at 14.436 s precedes the RES-43 handoff at 14.487 s by 50.8 ms; E12 confirmation at 14.936 s includes 0.449 s under RES-43 and 0.0508 s under SETTLE.
- **REPRODUCTION_OR_PROOF**: Sealed canonical result E12_PRE_HANDOFF_S=0.05075 and E12_UNDER_RES43_S=0.449125.
- **NUMERIC_EVIDENCE**: `{"E12_occ_s": 14.43625, "E12_pre_handoff_s": 0.05075, "E12_under_RES43_s": 0.44913, "handoff_s": 14.487}`
- **OBSERVED_EFFECT**: The scorer certificate is not exclusively tied to the final controller mode.
- **ROOT_CAUSE**: Events are observational and cross regime boundaries by design; the receipt presents E12 as the handoff certificate.
- **R001_IMPACT**: R001 E12 crossing the handoff is a qualification-presentation issue.
- **SCIENTIFIC_CLAIM_IMPACT**: 'E12 under RES-43 hold' claim is only 89.8% of the dwell.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: E12 semantics, RES-43 handoff, canonical result
- **DEPENDENCIES**: HIGH-007, HIGH-012
- **FIX_CLASS**: QUALIFICATION_CHANGE (define whether E12 must complete under RES-43 or across regimes)
- **REQUIRED_REGRESSION_TEST**: Regime-boundary test: E12 dwell must satisfy the declared regime coverage.
- **REQUALIFICATION_SCOPE**: Recovery qualification reporting.
- **EVIDENCE_POINTERS**: canonical result E12 fields

## MED-014 — Horizon authority drift: four coexisting horizons (4.0/8.0/20.0) with no runtime binding

- **SEVERITY**: MEDIUM
- **TYPE**: DOCUMENTATION_DEFECT, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py, src/loaded_cmj/v2/canonical_runtime.py, tools/res52/core52.py, src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: constants.py:27-28 (8.0/1600); canonical_runtime.py:67 (20.0); core52.py:50 (4.0); events.py:859 (finalize default 4.0)
- **DECLARED_BEHAVIOR**: One canonical episode horizon
- **IMPLEMENTED_BEHAVIOR**: Canonical runtime uses 20.0 s; constants declare 8.0 s; the research core uses 4.0 s; finalize's horizon_s parameter is unused. The doc/authority ledger still says 8.0 s.
- **REPRODUCTION_OR_PROOF**: Source + sealed environment/registry.
- **NUMERIC_EVIDENCE**: `{"constants_horizon_s": 8.0, "horizons_s": [4.0, 8.0, 20.0], "runtime_horizon_s": 20.0}`
- **OBSERVED_EFFECT**: Readers and successors can pick the wrong horizon; constants-based tooling is inconsistent with the canonical run.
- **ROOT_CAUSE**: Const declined to be updated across the V2.1 history; canonical hardcodes its own.
- **R001_IMPACT**: R001 ran 15.236 s of a 20 s budget; the 8 s constant is only documentation-stale here.
- **SCIENTIFIC_CLAIM_IMPACT**: Horizon claims must cite the canonical runtime, not constants.py.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: constants.py, CANONICAL_V2_CANDIDATE_SPEC, AUTHORITY_LEDGER.json, core52.py
- **DEPENDENCIES**: MED-015
- **FIX_CLASS**: DOCUMENTATION/FIX (single horizon authority)
- **REQUIRED_REGRESSION_TEST**: Authority-consistency test comparing constants/spec/runtime horizon values.
- **REQUALIFICATION_SCOPE**: Documentation and constants reconciliation.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md
- **PREFLIGHT_REFERENCES**: P35

## MED-015 — Provenance artifacts disagree on entry heads and the authority ledger is stale/corrupted

- **SEVERITY**: MEDIUM
- **TYPE**: PROVENANCE_DEFECT, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: CANONICAL_CANDIDATE_IDENTITY.json, CANONICAL_V2_CANDIDATE_SPEC.json, CANONICAL_V2_RUNTIME_CONTRACT.md, tools/res79_smoke_replay.py, AUTHORITY_LEDGER.json
- **LINE_OR_FUNCTION**: identity ENTRY_HEAD=20e7488/tree a8da465; spec/contract ENTRY_HEAD=9ca6a8b/tree 41e4dc81; res79 smoke ENTRY_HEAD=395c194/tree 532dfb4b; actual audit HEAD=8f26736 (commit message V2.1: add accepted trajectory visual smoke replay); AUTHORITY_LEDGER.json uv_lock_hash malformed/stale entries
- **DECLARED_BEHAVIOR**: One canonical entry head/tree identifies the candidate's source state
- **IMPLEMENTED_BEHAVIOR**: At least four different entry heads coexist; test_res78 enforces the spec's 9ca6a8b while the identity file says 20e7488; the ledger references a pre-V2.1 head and a corrupted uv_lock hash.
- **REPRODUCTION_OR_PROOF**: grep of all authority artifacts; test_res78:80.
- **NUMERIC_EVIDENCE**: `{"entry_heads": ["9ca6a8b6b01a600a2b8c983d674ddea63b2e53b2", "20e7488ae727c02b581ec4c8e668476ec4432fa5", "395c19426448ea7a3c1b7e612aff1a64e086972f", "8f26736db1231042cbedc61f61ca4862b7be871c"]}`
- **OBSERVED_EFFECT**: A reader cannot determine which commit is the authoritative candidate source.
- **ROOT_CAUSE**: Authority files updated at different times without a single source of truth.
- **R001_IMPACT**: R001 reproducibility is unaffected; provenance claims are ambiguous.
- **SCIENTIFIC_CLAIM_IMPACT**: Entry-head provenance claims unsupported until reconciled.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: CANONICAL_CANDIDATE_IDENTITY, spec/contract, AUTHORITY_LEDGER, res79 metadata
- **DEPENDENCIES**: CRIT-004, MED-014
- **FIX_CLASS**: PROVENANCE_FIX (single identity artifact; regenerate ledger)
- **REQUIRED_REGRESSION_TEST**: Identity-consistency test over spec/contract/identity/tool.
- **REQUALIFICATION_SCOPE**: Successor provenance.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md

## MED-016 — Production modules hardcode developer absolute paths and inject sys.path; evid_trace_v2 is imported under two module identities

- **SEVERITY**: MEDIUM
- **TYPE**: PROVENANCE_DEFECT, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/balance_capture.py, src/loaded_cmj/v2/stable_recovery.py, src/loaded_cmj/v2/canonical_runtime.py, tools/res52/core52.py
- **LINE_OR_FUNCTION**: balance_capture.py:18-25; stable_recovery.py:24-34; canonical_runtime.py:35-41,74-76; core52.py:35-37
- **DECLARED_BEHAVIOR**: Package code is location-independent
- **IMPLEMENTED_BEHAVIOR**: Library modules insert /home/litju/... into sys.path; canonical_runtime also bootstraps sibling dirs; evid_trace_v2 loads both as top-level and as tools.evid_trace_v2 (duplicate module identity).
- **REPRODUCTION_OR_PROOF**: Source grep (35 absolute /home/litju occurrences in src+tools).
- **NUMERIC_EVIDENCE**: `{"absolute_path_refs": 35, "duplicate_module_identity": ["evid_trace_v2", "tools.evid_trace_v2"]}`
- **OBSERVED_EFFECT**: Relocated clones/integrated deployments fail or import divergent copies; fixes can be masked.
- **ROOT_CAUSE**: Research-time path shortcuts left on production import paths.
- **R001_IMPACT**: The author's machine works; no R001 numerical effect.
- **SCIENTIFIC_CLAIM_IMPACT**: Clean-clone/reproducibility claims limited.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: package layout, sys.modules identity, clean-clone audit
- **DEPENDENCIES**: CRIT-006
- **FIX_CLASS**: PACKAGING_FIX (relative imports; single module identity)
- **REQUIRED_REGRESSION_TEST**: Relocated-clone test with a different root and CWD.
- **REQUALIFICATION_SCOPE**: Successor release qualification.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md, CRIT-006
- **PREFLIGHT_REFERENCES**: P34

## MED-017 — Production swallows diagnostics and reports sentinel values instead of failing closed

- **SEVERITY**: MEDIUM
- **TYPE**: SOFTWARE_BUG, QUALIFICATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/res72_integration.py, tools/res52/soft_contact.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:388-389 (checkpoint exception swallowed); canonical_runtime.py:477-499 (adjudication errors -> {'error'} and -1 sentinels); res72_integration.py:232-236 (only VALIDATED/FALLBACK/shrinks forwarded); soft_contact.py:327-343 (Y_VAL etc. describe the rejected candidate on fallback)
- **DECLARED_BEHAVIOR**: Fail closed with diagnostics on errors and record what was actually executed
- **IMPLEMENTED_BEHAVIOR**: Checkpoint failures are logged and ignored; support-adjudication exceptions become sentinels that the runtime itself does not gate; on soft-contact fallback the returned diagnostics (Y_VAL/du) still describe the rejected trial action while u is the held action.
- **REPRODUCTION_OR_PROOF**: Source inspection.
- **NUMERIC_EVIDENCE**: `{"fallback_telemetry_mismatch": true, "post_loss_sentinel": -1, "swallowed_checkpoint_exceptions": true}`
- **OBSERVED_EFFECT**: A failed adjudication or checkpoint silently degrades evidence; fallback telemetry can mislead post-hoc analysis.
- **ROOT_CAUSE**: Broad except blocks added for robustness without fail-closed gating.
- **R001_IMPACT**: No demonstrated R001 effect (adjudications succeeded).
- **SCIENTIFIC_CLAIM_IMPACT**: Evidence completeness/trust limited; a silent failure could pass as an empty result.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: canonical_runtime.py, soft_contact.py, adjudication fields
- **DEPENDENCIES**: CRIT-004
- **FIX_CLASS**: SOFTWARE_FIX (fail closed; record executed action in fallback)
- **REQUIRED_REGRESSION_TEST**: Fault-injection test: forced adjudication exception must terminate or flag the run.
- **REQUALIFICATION_SCOPE**: Runtime/evidence requalification.
- **EVIDENCE_POINTERS**: BUG_HUNT.md
- **PREFLIGHT_REFERENCES**: P38

## MED-018 — Dwell/threshold literals are duplicated across runtime, mirror module and external JSON with no binding

- **SEVERITY**: MEDIUM
- **TYPE**: SOFTWARE_BUG, PROVENANCE_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py, src/loaded_cmj/v2/full_closure.py, src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:70 (E10_NEED=160), :338 (E10 literals); full_closure.py:26 (E10_DWELL_SAMPLES=160), :35-45 (RR_THRESHOLDS); external recovery_ready_spec.json consumed at runtime
- **DECLARED_BEHAVIOR**: One frozen authority per constant
- **IMPLEMENTED_BEHAVIOR**: E10 dwell/thresholds exist as runtime literals and in full_closure mirrors; RR thresholds/dwell come from external JSON while full_closure carries a different frozen copy that the runtime never reads. Changing either side diverges silently.
- **REPRODUCTION_OR_PROOF**: Source comparison (full_closure is hash-bound but dead; runtime literals unhashed).
- **NUMERIC_EVIDENCE**: `{"e10_dwell_duplicates": 2, "full_closure_executed": false, "rr_threshold_copies": 2}`
- **OBSERVED_EFFECT**: Authority drift risk; qualification mirrors may not match executed logic.
- **ROOT_CAUSE**: Refactor left mirrors for provenance while the executable path was re-implemented.
- **R001_IMPACT**: R001 constants happen to agree.
- **SCIENTIFIC_CLAIM_IMPACT**: Frozen-constant claims are not verifiable from a single source.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: full_closure.py, canonical_runtime.py, external RR spec
- **DEPENDENCIES**: CRIT-004, MED-014
- **FIX_CLASS**: PROVENANCE_FIX (single imported constant source)
- **REQUIRED_REGRESSION_TEST**: Authority-consistency test over all copies.
- **REQUALIFICATION_SCOPE**: Successor provenance.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md

## MED-019 — Multiple V2.1-named tests exercise the legacy controller, not the canonical composition

- **SEVERITY**: MEDIUM
- **TYPE**: TEST_DEFECT, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: tests/test_v2_1_res10_honest_full_jump.py, tests/test_v2_1_res16_true_standing.py, tests/test_v2_1_res5_honest_fall.py, tests/test_v2_1_res6_compliant_landing_contact.py, tests/test_v2_1_res8_captured_squat.py, tests/test_v2_1_res31_honest_planar_root.py, tests/test_v2_1_res42_zero_root_damping.py, tests/test_res10_controller_obs_sync.py, tests/test_res10_physics_sample_sync.py
- **LINE_OR_FUNCTION**: controller.py module-level act/reset; test files import loaded_cmj.v2.controller
- **DECLARED_BEHAVIOR**: V2.1 controller behavior is tested
- **IMPLEMENTED_BEHAVIOR**: These suites import the 128-line legacy module (module-global state, catch-all except) while the production path is Res72Policy/Balance/Recovery; no test imports/executes the canonical composition.
- **REPRODUCTION_OR_PROOF**: grep of test imports; legacy module has no importers in src.
- **NUMERIC_EVIDENCE**: `{"canonical_composition_test_files": 0, "legacy_controller_test_files": 9}`
- **OBSERVED_EFFECT**: V2.1 test names overstate what is exercised; legacy behavior may be discontiguous with production.
- **ROOT_CAUSE**: Historical tests were not migrated when the composition was split.
- **R001_IMPACT**: R001 is not certified by these tests.
- **SCIENTIFIC_CLAIM_IMPACT**: V2.1 test-coverage narrative is not defensible.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: test suite credibility, legacy controller
- **DEPENDENCIES**: HIGH-015, HIGH-017
- **FIX_CLASS**: TEST_FIX (migrate to canonical composition or mark historical)
- **REQUIRED_REGRESSION_TEST**: Coverage test: V2.1-named suites must import the canonical entrypoint.
- **REQUALIFICATION_SCOPE**: Test suite requalification.
- **EVIDENCE_POINTERS**: TEST_FILE_INVENTORY.csv, TEST_SUITE_FORENSIC_REPORT.md

## MED-020 — E1 supported-start guard omits the fall flag (only the dead prohibited flag is checked)

- **SEVERITY**: MEDIUM
- **TYPE**: SCORER_SPEC_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/events.py
- **LINE_OR_FUNCTION**: events.py:126-141 (_guard_supported_start checks prohibited, not fall_contact)
- **DECLARED_BEHAVIOR**: Supported start excludes falling states
- **IMPLEMENTED_BEHAVIOR**: The guard checks prohibited (structurally always False) but not fall_contact; a fall shell on the floor during the first 0.1 s would not block E1 (the fall latch would later set physical_fall).
- **REPRODUCTION_OR_PROOF**: Source read + MED-006.
- **NUMERIC_EVIDENCE**: `{"fall_checked": false, "prohibited_effective": false}`
- **OBSERVED_EFFECT**: E1 can latch in a fall-adjacent state; subsequent fall handling still fails the run.
- **ROOT_CAUSE**: Predicate uses the wrong flag.
- **R001_IMPACT**: Latent in R001 (no fall).
- **SCIENTIFIC_CLAIM_IMPACT**: E1 semantics weaker than declared.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: events.py (SCORER_SHA256), E1 definition
- **DEPENDENCIES**: MED-006
- **FIX_CLASS**: SCORER_SPEC_FIX (check fall_contact in E1)
- **REQUIRED_REGRESSION_TEST**: Fall-at-start negative control must not latch E1.
- **REQUALIFICATION_SCOPE**: Event requalification.
- **EVIDENCE_POINTERS**: TEST_SUITE_FORENSIC_REPORT.md

## LOW-001 — Plant XML header documents ngeom=14 while the model has 16

- **SEVERITY**: LOW
- **TYPE**: DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/assets/v2_plant.xml
- **LINE_OR_FUNCTION**: v2_plant.xml:12
- **DECLARED_BEHAVIOR**: Header identity block matches the compiled model
- **IMPLEMENTED_BEHAVIOR**: Header says ngeom=14; constants assert 16 (baseline 10 + 6 fall shells).
- **REPRODUCTION_OR_PROOF**: XML/constants comparison.
- **NUMERIC_EVIDENCE**: `{"actual_ngeom": 16, "header_ngeom": 14}`
- **OBSERVED_EFFECT**: Documentation inconsistency only.
- **ROOT_CAUSE**: Header not updated for fall shells.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: Identity comment is stale.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: Plant XML documentary header
- **DEPENDENCIES**: 
- **FIX_CLASS**: DOC_FIX
- **REQUIRED_REGRESSION_TEST**: Header-vs-model assertion test.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md

## LOW-002 — README describes 15 bounded anatomical channels while ACTION_DIM=7

- **SEVERITY**: LOW
- **TYPE**: DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: README.md
- **LINE_OR_FUNCTION**: README.md:13
- **DECLARED_BEHAVIOR**: Documented action dimension
- **IMPLEMENTED_BEHAVIOR**: README says 15 channels; V2.1 has 7 (4 effective symmetric channels).
- **REPRODUCTION_OR_PROOF**: Text vs candidate spec.
- **NUMERIC_EVIDENCE**: `{"action_dim": 7, "readme_channels": 15}`
- **OBSERVED_EFFECT**: Reader confusion only.
- **ROOT_CAUSE**: README not updated across the V1->V2.1 transition.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: Documentation drift.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: README.md
- **DEPENDENCIES**: MED-014
- **FIX_CLASS**: DOC_FIX
- **REQUIRED_REGRESSION_TEST**: Docs consistency check.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: DOCUMENTATION review in DEFECT_REGISTER.md

## LOW-003 — Stale/incorrect comments and dead branches in contact and support code

- **SEVERITY**: LOW
- **TYPE**: DOCUMENTATION_DEFECT, SOFTWARE_BUG
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/plant.py
- **LINE_OR_FUNCTION**: plant.py:283-286 (comment 0.13/0.0475 vs code 0.150/0.060); plant.py:210-231 (wrong frame-convention comments and dead sys_idx branch)
- **DECLARED_BEHAVIOR**: Comments describe the implementation
- **IMPLEMENTED_BEHAVIOR**: Support-half-size comment disagrees with the literals; the contact-frame comment asserts the opposite transform and a dead branch remains.
- **REPRODUCTION_OR_PROOF**: Source read; verified transform behavior by contact-force vs qfrc_constraint equality.
- **NUMERIC_EVIDENCE**: `{"comment_half_sizes": [0.13, 0.0475], "support_half_sizes_code": [0.15, 0.06]}`
- **OBSERVED_EFFECT**: Maintenance hazard; readers may 'fix' correct code or copy the wrong convention.
- **ROOT_CAUSE**: Comments left from earlier iterations.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: Documentation/robustness only.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: plant.py
- **DEPENDENCIES**: HIGH-008
- **FIX_CLASS**: DOC/CLEANUP_FIX
- **REQUIRED_REGRESSION_TEST**: Comments-consistency review.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md

## LOW-004 — IMPACT KD 'ramp' is an identity no-op presented as a ramp

- **SEVERITY**: LOW
- **TYPE**: SOFTWARE_BUG, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/res72_integration.py
- **LINE_OR_FUNCTION**: res72_integration.py:129-134 (_impact_kd returns IMPACT_KD + (IMPACT_KD-IMPACT_KD)*ramp)
- **DECLARED_BEHAVIOR**: KD ramps from KD_INIT to KD_FINAL over 30 ms after touchdown
- **IMPLEMENTED_BEHAVIOR**: KD_INIT==KD_FINAL==IMPACT_KD, so the ramp is identically zero; the constant smoothing call is dead.
- **REPRODUCTION_OR_PROOF**: Source read.
- **NUMERIC_EVIDENCE**: `{"kd_final": 20.0, "kd_init": 20.0}`
- **OBSERVED_EFFECT**: None numerically; misleading code.
- **ROOT_CAUSE**: Sealed candidate had equal init/final gains; the ramp skeleton was retained.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: None.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: res72_integration.py
- **DEPENDENCIES**: 
- **FIX_CLASS**: CLEANUP_FIX
- **REQUIRED_REGRESSION_TEST**: None required (or implement a real ramp).
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: CONTROL_REVIEW.md

## LOW-005 — Dead code: unused y_of branch, unused unilateral_dropout_samples with misleading return, dead conditional expression

- **SEVERITY**: LOW
- **TYPE**: SOFTWARE_BUG, DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/balance_capture.py, src/loaded_cmj/v2/support_continuity.py, tools/res52/soft_contact.py
- **LINE_OR_FUNCTION**: balance_capture.py:52-74 (y_of unused); support_continuity.py:97-105 (returns array, docstring says Boolean arrays; unused); soft_contact.py:245 ("FZ_SCALE_M" if False else)
- **DECLARED_BEHAVIOR**: Clean minimal implementation
- **IMPLEMENTED_BEHAVIOR**: Dead functions/expressions remain; the helper's contract docstring does not match its return.
- **REPRODUCTION_OR_PROOF**: Call-site grep.
- **NUMERIC_EVIDENCE**: `{"unilateral_helper_calls": 0, "unused_y_of": true}`
- **OBSERVED_EFFECT**: Maintenance/latent misuse only.
- **ROOT_CAUSE**: Iteration leftovers.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: None.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: balance_capture.py, support_continuity.py, soft_contact.py
- **DEPENDENCIES**: 
- **FIX_CLASS**: CLEANUP_FIX
- **REQUIRED_REGRESSION_TEST**: Dead-code check.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: CODE_REVIEW.md

## LOW-006 — WALL_S makes the canonical result JSON non-byte-deterministic

- **SEVERITY**: LOW
- **TYPE**: SOFTWARE_BUG, PROVENANCE_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/canonical_runtime.py
- **LINE_OR_FUNCTION**: canonical_runtime.py:586
- **DECLARED_BEHAVIOR**: Deterministic machine-readable output
- **IMPLEMENTED_BEHAVIOR**: Wall-clock seconds are embedded in the result; byte identity of the JSON is impossible across runs.
- **REPRODUCTION_OR_PROOF**: Source read; noted in the code itself.
- **NUMERIC_EVIDENCE**: `{"wall_s_example": 752.18}`
- **OBSERVED_EFFECT**: Artifact-level byte comparison requires excluding WALL_S; no scientific effect.
- **ROOT_CAUSE**: Diagnostic convenience left in the serialized result.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: Determinism claim limited to scientific fields.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: canonical result
- **DEPENDENCIES**: HIGH-013
- **FIX_CLASS**: CLEANUP_FIX (move WALL_S to sidecar)
- **REQUIRED_REGRESSION_TEST**: Determinism test on the scientific subset.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: PROVENANCE_REVIEW.md
- **PREFLIGHT_REFERENCES**: P18-adjacent

## LOW-007 — Ankle sign comment is unresolved and geometrically inverted

- **SEVERITY**: LOW
- **TYPE**: DOCUMENTATION_DEFECT
- **STATUS**: CONFIRMED
- **FILE**: src/loaded_cmj/v2/constants.py
- **LINE_OR_FUNCTION**: constants.py:140-141 (comment: negative plantar, positive dorsiflex; question mark included)
- **DECLARED_BEHAVIOR**: Ankle sign convention documented
- **IMPLEMENTED_BEHAVIOR**: The comment is self-questioning; with axis +y and the box forward of the ankle, positive q pitches the foot toe-down.
- **REPRODUCTION_OR_PROOF**: FK/geometry probe; no numeric R001 effect.
- **NUMERIC_EVIDENCE**: `{"range_rad": [-0.7, 0.5]}`
- **OBSERVED_EFFECT**: Documentation/authority clarity only.
- **ROOT_CAUSE**: Sign convention never closed.
- **R001_IMPACT**: None.
- **SCIENTIFIC_CLAIM_IMPACT**: Ankle authority description unreliable.
- **AUTHORITY_INVALIDATED_OR_LIMITED**: constants.py
- **DEPENDENCIES**: LOW-003
- **FIX_CLASS**: DOC_FIX
- **REQUIRED_REGRESSION_TEST**: Sign test for the ankle axis.
- **REQUALIFICATION_SCOPE**: None.
- **EVIDENCE_POINTERS**: CONTACT_REVIEW.md

