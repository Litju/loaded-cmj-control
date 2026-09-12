# NEXT_CANDIDATE_REQUIREMENTS (RES-80)

This is **not** an R002 implementation plan. It is the scientific requirement set derived from the sealed
defect register. Each requirement names the defects it must close, the gate that must be satisfied before
the requirement can be considered implemented, and the acceptance evidence required.

Values marked `REQUIRES_OWNER/SCIENTIFIC_CONTRACT_DECISION` must not be invented by an implementer.

## PLANT

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| P-1 | Correct knee joint anatomy: positive flexion moves the ankle posteriorly; remove reversed range exclusion. | CRIT-001 | any successor controller work | FK sign probe + a countermovement pose proof; new Plant hash |
| P-2 | Correct hip joint range/convention so human hip flexion (>=1.0 rad) is reachable and the controller drives flexion during the countermovement. | CRIT-002 | with P-1 | Range/FK proof + crouch pose signature |
| P-3 | Foot model with MTP/toe (and declared arch/rocker behavior) or an explicit owner-approved reduced-foot claim. | HIGH-019 | any push-off fidelity claim | Geometry/joint tests, foot-roll evidence, push-off comparison |
| P-4 | Activation dynamics or documented command-slew limit if human-like actuation is claimed; otherwise declare direct-torque as a limitation and exclude human-dynamics claims. REQ- Decision: activation model vs declared limitation. | HIGH-006 | human-dynamics claims | Rise-time model + test, or an approved claim boundary |
| P-5 | Define collision/fall-state collision policy incl. load collider and self-collision; document exclusions with margins. | MED-007 | any fall-state claim | Collision matrix test, fall interpenetration audit |
| P-6 | Record anthropometry/inertia provenance against an external reference dataset. | MODEL_FORM_LIMITATIONS #10 | population claims | Traceable table with citations |

## MEASUREMENT

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| M-1 | Support margin = distance to the convex hull/support interval of *active* contacts; expose active-foot flags; no positive margin in flight. | HIGH-008 | any support-based gate | Hull tests incl. rotated/single support; flight margin <=0 |
| M-2 | Correct rotated lowest-foot-point geometry for gap and velocity (true corner via body/geom rotation). | HIGH-009, RES-55 requalification | rotated-foot successors | Gap/velocity tests at ankle extremes |
| M-3 | Define CoP origin/frame explicitly, expose origin, unify validity threshold with the named constant, and use measured CoP where control claims it. | MED-005 | CoP-based claims | Frame-reconstruction test (origin + force + moment -> world wrench) |
| M-4 | True pelvis orientation quaternion; signed trunk pitch/rate. | MED-003, MED-004 | observation contract claims | Orientation/tilt tests under nonzero pitch |
| M-5 | `prohibited`/penetration flags must scan actual colliders or be removed from claims. | MED-006 | prohibited-contact claims | Injected-contact tests |
| M-6 | Metric definitions corrected: jump height vs apex z, loading rate as max slope, CoP invalid intervals reported. | MED-011 | performance reporting | Definition tests on synthetic signals |

## CONTROL

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| C-1 | Takeoff switching must require sustained bilateral unload (dwell) and physical separation; no single-sample guard. | CRIT-007 | takeoff qualification | Adversarial guard tests |
| C-2 | Takeoff command must be extension/COM-authoritative, bounded in rate, and respect a declared action-slew/activation model. | CRIT-007, HIGH-006 | takeoff qualification | Bounded-step test + ballistic consistency |
| C-3 | Terminal/landing control must have horizontal CAM/CoP authority or an explicitly reduced landing claim. | HIGH-001, HIGH-002 | landing qualification | Perturbation landing test with CoM/CAM bounds |
| C-4 | Balance authority must be two-sided and support-feasible (or the claim must be reduced to braking-only). | HIGH-003, MED-009 | balance claims | Two-sided perturbation test; CoP-line exactness |
| C-5 | Action continuity across every regime switch (no zero-reset); trust region anchored to the actually applied action. | HIGH-005 | continuity claims | sup|du| bounded at all switches |
| C-6 | Soft-contact inner layer: constrain every refinement pass to include all active constraint families (FZ_MIN, etc.); handle active-set crossings one-sided. | HIGH-014 | terminal/balance claims | Unit proofs: FZ_MIN row present each pass; crossing test |
| C-7 | No `assert` for runtime contracts; fail-closed errors with diagnostics; fallback telemetry must describe the executed action. | MED-017, BUG-06 | release | Fault-injection tests |
| C-8 | Truncated control intervals must be represented in validation (or excluded from claims). | CONTROL_REVIEW §2.5, BUG-11 | release | Interval-coverage test |

## EVENTS

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| E-1 | Wire `GENUINE_FLIGHT_GAP_M` into E7 (or delete the declaration). | HIGH-010 | event qualification | Force-dropout-without-clearance negative control |
| E-2 | Implement apex dwell and require physical flight in every apex path. | HIGH-011 | event qualification | Contact-phase crossing negative control |
| E-3 | E10 must include horizontal/angular/posture criteria; E11 must use the declared CoM speed (or be renamed); E12 must be a physical standing envelope, not ULP-adjacent reproducibility. | HIGH-001, MED-002, HIGH-012 | event qualification | Negative controls per predicate |
| E-4 | Dwell timing must match declared durations (fix off-by-one or declare sample-count semantics). | MED-001 | event certification | Boundary tests at exact D |
| E-5 | E1 must check fall contact. | MED-020 | event certification | Fall-at-start negative control |
| E-6 | State adjudicated window for support-continuity gates and surface the full verdict. | MED-012 | qualification reporting | Slice-scope contract test |
| E-7 | Define E12/handoff regime coverage (E12 under RES-43 or explicit partial). | MED-013 | recovery certification | Regime-boundary test |

## TASK PERFORMANCE (all values REQUIRES_OWNER/SCIENTIFIC_CONTRACT_DECISION)

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| T-1 | Declare minimum jump performance: jump height, COM rise, flight time, geometric clearance, and takeoff extension criteria. | CRIT-003 | any success claim | Owner-approved task contract + gates |
| T-2 | Declare landing criteria: horizontal momentum, CAM/trunk posture, support continuity, CoP excursions. | HIGH-001/002/003/004 | landing claim | Task contract + gates |
| T-3 | Declare recovery criteria: posture at entry, standing envelope robustness (perturbed holds), hold duration. | HIGH-004, HIGH-012 | recovery claim | Task contract + robustness study |
| T-4 | Canonical result must expose all performance/lading/recovery metrics and the declared gates. | HIGH-013, MED-011 | qualification artifact | Schema + metrics presence tests |

## LANDING / BALANCE / RECOVERY

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| L-1 | A landing strategy with explicit inheritance of horizontal/angular momentum and a declared capture objective. | HIGH-001/002/003 | landing qualification | Momentum-budget trace + perturbation tests |
| L-2 | Balance capture with measured CoP feedback and two-sided authority inside the true support hull. | HIGH-003, HIGH-008, MED-005 | balance qualification | CoP tracking within hull |
| L-3 | Recovery entry gate with absolute posture and CoP criteria. | HIGH-004 | recovery qualification | Posture-at-entry test |
| L-4 | Recovery path tracked against the actual entry state; no 13 s quasi-open-loop transit unless declared. | HIGH-004 | recovery claim | Tracking error bound + duration rationale |

## TESTS

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| TS-1 | Remove placeholders/`or True`/pass-only bodies; restore failure-capable assertions. | HIGH-015/016 | any test-based claim | Mutation tests that fail |
| TS-2 | At least one test must execute the canonical composition end-to-end (bounded smoke) with hash binding; suite must collect on a clean clone without external evidence. | HIGH-017, CRIT-006 | release | Fresh-process smoke test in CI |
| TS-3 | Qualification orchestrator must run all suites; no hardcoded PASS; scripts must fail on mutated inputs. | HIGH-018 | release | Coverage + mutation tests |
| TS-4 | Tests must exercise the current composition, not the legacy controller; historical suites marked. | MED-019 | release | Import-coverage audit |
| TS-5 | Scientific-gate tests must bind to declared contracts (not local re-implementations). | WEAK-class tests | release | Contract-binding review |

## PROVENANCE

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| PR-1 | Candidate identity must cover the full executed closure (orchestrator, constants, plant, measurement, drive, support continuity, tools modules) and be verified at runtime before execution. | CRIT-004 | any candidate execution | Identity verifier + mutation negative test |
| PR-2 | External authorities must be vendored or hash-pinned in the identity and verified at load. | CRIT-005 | any candidate execution | Authority hash verification test |
| PR-3 | Package must be self-contained and installable (tools moved into package, scipy declared, no absolute developer paths; single module identity). | CRIT-006, MED-016 | release | Clean-venv install + canonical run test |
| PR-4 | Single entry-head/identity artifact; ledger/registry regenerated; result schema includes hashes and ENTRY_HEAD/TREE. | MED-015, HIGH-013, MED-014 | release | Identity-consistency + schema tests |
| PR-5 | Move shared physical predicates out of scorer code or include scorer code in the control identity; document the coupling. | HIGH-007 | release | Separation test with mutation |

## RENDER / VISUAL REVIEW

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| V-1 | Keep the RES-79 non-invasive replay contract; every successor candidate requires an owner visual review before success classification. | owner-observed failures | ship | Stills + video + owner adjudication |
| V-2 | Overlay the truly gated quantities (flight clearance, support hull, CoP, posture) in the review render. | HIGH-008/013, MED-005 | owner review | Review packet with numeric overlays |

## V&V

| # | Requirement | Defects closed | Before | Acceptance evidence |
|---|---|---|---|---|
| V-3 | Independent validator must recompute the new performance gates from raw arrays, not stored JSON. | HIGH-013/017 | qualification | Independent recomputation report |
| V-4 | Negative-control candidates (trivial hop, single-support landing, perturbed standing) must FAIL. | CRIT-003, HIGH-012 | qualification | Calibration/negative-control study |
| V-5 | Re-derive the true-standing envelope from multiple perturbed holds with a declared robustness target. | HIGH-012 | recovery claim | Envelope robustness study |

## Cross-cutting decision requests (owner)

1. **Task performance contract** (T-1/T-2/T-3 values) - no numeric target may be invented by engineering.
2. **Activation model vs declared limitation** (P-4).
3. **Reduced-foot claim vs MTP model** (P-3).
4. **Landing posture/CAM criteria and recovery entry criteria** (T-2/T-3, L-3).
5. **Whether direct-torque, one-sided balance and vertical-only capture are acceptable under a reduced
   claim** - if yes, the claim ceiling must be updated accordingly.
