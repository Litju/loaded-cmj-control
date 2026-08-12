# ML212B — E3→E4 feasibility owner closure

Status: **accepted owner amendment; implementation gates remain separate**
Scope: current loaded-cmj-control authority only

## Authority scope

```text
CURRENT_IMPLEMENTATION_AUTHORITY=/home/litju/Projects/loaded-cmj-control
CURRENT_EVIDENCE_AUTHORITY=/home/litju/Projects/loaded-cmj-control-evidence
LEGACY_ALIGNERR_REPOSITORY=/home/litju/Projects/loaded-jump-control-tm101
LEGACY_ALIGNERR_REPOSITORY_ROLE=HISTORICAL_ONLY_NOT_CURRENT_AUTHORITY
```

The legacy repository does not control Plant or MJCF identity, E3 identity,
ML-212/241/242/243 semantics, derivative semantics, or feasibility results.
Historical evidence artifacts are immutable and are not rewritten by this
amendment.

This amendment freezes owner decisions required before the subsequent
mechanical NLP repair. It authorizes no Plant, MJCF, DriveState, controller,
event, scorer, solver, or test implementation changes.

## Current identity and authoritative E3

The current authority is `main` at
`1e25df954a81c9f27c74cc650638460619a34797`, tree
`76f0fa6573fc04d6901305e393e3964a16108143`.

```text
PLANT_IDENTITY=loaded-cmj-20kg-athlete / loaded-cmj-model-1
MJCF_IDENTITY=src/loaded_cmj/assets/loaded_jump_athlete.xml
```

```text
AUTHORITATIVE_E3_ID=E3_R024_VALID_COUNTERMOVEMENT_GRID_0770000
AUTHORITATIVE_E3_TIME_S=0.770000
AUTHORITATIVE_E3_STATE_HASH=2cdd3cda6c6b91ba211dfec696e2348a5446992f32fdb65d53f51951eafac208
AUTHORITATIVE_E3_COM_VZ_MPS=-0.2747849764551471
AUTHORITATIVE_E3_PLANT_HASH=1d1e1dc753786cd5f070e115210d8fcc5c4ee4b0b58753a2eeadeb5aa81a2dbd
AUTHORITATIVE_E3_MJCF_HASH=cc122cee734faa2a72d2b1bb829f824c7e865ba3365695887468fc7844dfc615
```

The state is the current R2 authority-chain selection: the qualified R024
`valid_countermovement` event at `0.767875 s`, rounded forward to the first
exact 5 ms control knot. The supporting snapshot digest is
`3b521727aff1225c0303564719d925c0c8c41b7217cff1afcd7b0872fa9f952f`.
The `0.770125 s` physical receipt and the inconsistent snapshot timestamp are
evidence-generation/timing defects, not new physical E3 states; their hashes
do not alter the authoritative state hash. Implementations must load or
reconstruct this exact state, never a nearby approximation.

Primary receipts: current evidence
`F4-ORACLE-FEASIBILITY/ML212-E3-E4-SAME-PLANT/20260811T230148Z/10_E3_START_PROVENANCE.md`
and `11_E3_START_SNAPSHOT_IDENTITY.json`. The R2 adjudication receipt was
`/tmp/lcmj-ml212a-r2-20260812T021009Z/AUTHORITY_RECEIPT.md`.

R2 receipt status for the repair gate:

```text
CURRENT_REPOSITORY_IDENTITY=PASS
CURRENT_E3_IDENTITY=PASS
CURRENT_PLANT_IDENTITY=PASS
CURRENT_MJCF_IDENTITY=PASS
FIXED_VARIABLE_DERIVATIVE_OVERDEMAND=CONFIRMED
NATIVE_STATE_DOMAIN_NOT_ENCODED=CONFIRMED
BOUND_ACTIVE_CENTRAL_FD_INVALID=CONFIRMED
TRUE_INTERIOR_SIGN_KINK=CONFIRMED
A2_CONTACT_RESULT=BOTH
PHASE_I_OBJECTIVE_STATUS=DEFECT_CONFIRMED
PHASE_I_SCALES_STATUS=UNRESOLVED_BEFORE_THIS_AMENDMENT
ML212_SOLVE_HARNESS=PREFLIGHT_ONLY
PHYSICAL_PLANT_EDIT_REQUIRED=NO
FIXED_MODE_SPARSE_TRANSCRIPTION_RETAINED=YES
IPOPT_RETAINED=YES
```

## Phase-I normalization owner

```text
PHASE_I_NORMALIZATION_OWNER=E3_E4_NLP_CONSTRAINT_AUTHORITY
PHASE_I_LENGTH_SCALE_M=1.0
PHASE_I_ACTIVE_SCALE_ROWS=41 SUPPORT_MARGIN rows + 1 E3_E4_HORIZONTAL row
PHASE_I_SCALES_STATUS=OWNER_RESOLVED
```

All currently approved elastic E3→E4 rows are length-valued. For every such
row (i), the optimization normalization is:

```text
S_i = 1.0 m
s_i >= 0
rho >= 0
s_i <= rho * 1.0 m
```

Phase I minimizes `rho` subject to the original approved elastic relaxation
and these epigraph constraints. This common scale preserves relative
weighting, gives the epigraph derivative with respect to `rho` unit magnitude,
and leaves feasibility semantics unchanged: `rho=0` if and only if all
approved elastic slacks are zero.

`1.0 m` is an optimization normalization, not a physical tolerance, Plant
parameter, support-margin threshold, E4 acceptance threshold, or biomechanics
assumption. It must not be derived from current violation, support margin,
body weight, gravity, timestep, COM velocity, support-polygon width, or
row-specific tuning. A future elastic constraint with another physical
dimension requires an explicit dimension-specific owner decision before it
enters Phase I.

## Hard reversal

```text
E3_E4_REVERSAL_CLASS=HARD_NONELASTIC
SLACK=NONE
PHASE_I_SCALE=NONE
RHO_NORMALIZATION=NONE
```

The E3→E4 reversal remains a direct physical feasibility requirement. No
implementation may elasticize it without a new owner amendment.

## First existence witness

R2 confirms the following eight raw-action channels are genuine zero-action
sign kinks and are fixed at exactly zero for the first existence witness:

```text
WITNESS_FIXED_CONTROLS=
1 lumbar_lateral
2 lumbar_axial
4 left_hip_abduction
5 left_hip_rotation
7 right_hip_abduction
8 right_hip_rotation
13 left_ankle_eversion
14 right_ankle_eversion
```

The free witness controls are:

```text
WITNESS_FREE_CONTROLS=
0 lumbar_flexion
3 left_hip_flexion
6 right_hip_flexion
9 left_knee_flexion
10 right_knee_flexion
11 left_ankle_dorsiflexion
12 right_ankle_dorsiflexion
```

Indices above are zero-based, matching the current 15-channel implementation.
The witness is a strict subset of the full control domain:

```text
U_witness subset U_full
WITNESS_PASS => FULL_PLANT_EXISTENCE_ESTABLISHED
WITNESS_FAIL != FULL_PLANT_INFEASIBILITY
```

A witness failure requires bounded escalation review and must not be reported
as physical infeasibility of the full 15-dimensional Plant.

## Fixed E3 semantics

```text
E3_X0_ROLE=FIXED_PARAMETER
E3_X0_IS_NLP_DECISION=NO
```

Interval zero is:

```text
h0 = x1 boxminus Phi(x_E3, u0)
```

The NLP Jacobian must contain no `d h0 / d x0` columns. This is a structural
decision: ML-242 must remove x0 from the decision vector. It does not permit
zero-filling A0, suppressing an exception while retaining x0, perturbing E3,
or altering E3 bounds.

## Derivative policy

Every required derivative column is classified before evaluation as one of:

```text
CENTRAL_INTERIOR
FORWARD_FEASIBLE_SIDE
BACKWARD_FEASIBLE_SIDE
FIXED_ELIMINATED
TRUE_KINK_INVALID
PHYSICAL_CONTACT_SWITCH_INVALID
MANIFOLD_INVALID
NUMERICAL_CERTIFICATE_ONLY
```

Smooth interior coordinates may use central differences. Smooth lower and
upper physical bounds use the declared second-order feasible-side stencil.
Fixed/nondecision coordinates are eliminated. Genuine interior kinks and
physical contact-topology changes are rejected; no classical derivative is
fabricated. Manifold outputs use canonical boxplus/boxminus in one frozen
output tangent frame. The already-qualified qacc numerical-null/bounded-error
treatment remains a numerical certificate, not an invented physical
derivative.

Epsilon shifts, side averaging at kinks, arbitrary one-sided derivatives at
interior kinks, repeated h shrinking, zero filling, and fabricated derivatives
are prohibited.

## A[2] and contact ownership

```text
A2_AT_E3=PHYSICAL_CONTACT_SWITCH
A2_REQUIRED_AFTER_X0_ELIMINATION=NO
```

R2 found that A[2] loses physical contacts under the plus perturbation during
the first eight 0.125 ms substeps while BASE/MINUS retain them. This does not
justify a Plant edit, contact smoothing, or an invented derivative. It is an
interval-zero derivative demand that disappears with x0 elimination. The same
physical switch at a free future knot remains a derivative-qualification
rejection.

```text
SUPPORT_ACTIVITY_OWNER=PLANT_CONTACT_OWNER
SUPPORT_MARGIN_VALUE_OWNER=PLANT_SUPPORT_MARGIN_OWNER
```

Concretely, `Plant.contact_wrench_summary` owns contact/support activity and
`Plant.support_margin` owns the signed support-margin value. Task-local E3→E4
code must not force bilateral support independently when canonical Plant
activity is available. Canonical support physics and its derivatives must not
be duplicated in task-local code; task-specific row scheduling and event
semantics may remain task-local.

## Retained solver architecture and boundary

```text
FIXED_MODE_SPARSE_TRANSCRIPTION_RETAINED=YES
IPOPT_RETAINED=YES
SAME_PLANT_REQUIREMENT_PRESERVED=YES
PHYSICAL_PLANT_EDIT_REQUIRED=NO
IPOPT_BOUND_RELAX_FACTOR=0.0
IPOPT_BOUND_PUSH_STATUS=QUALIFICATION_REQUIRED
IPOPT_BOUND_FRAC_STATUS=QUALIFICATION_REQUIRED
```

The pre-repair ML-212 attempt did not reach a legitimate nonlinear solve after
the derivative/domain preconditions. It therefore establishes neither
`IPOPT_FAILURE`, physical infeasibility, nor a Plant defect. MPCC, FESD,
contact-implicit optimization, smooth surrogate dynamics, and an alternate
solver are not authorized. `bound_push` and `bound_frac` remain subject to a
later deterministic initialization qualification after corrected NLP bounds
exist; no arbitrary values are frozen here. No Ipopt solve may occur before
that qualification passes.

## Repair dependency order

```text
GATE_2  ML242_STRUCTURAL_NLP_REPAIR
GATE_3  ML241_DERIVATIVE_AND_BRANCH_CERTIFICATE_REPAIR
GATE_4  CANONICAL_CONSTRAINT_DERIVATIVE_AND_SUPPORT_OWNERSHIP_REPAIR
GATE_5  ML212_PHASE_I_RHO_EPIGRAPH_MATERIALIZATION
GATE_6  ML243_IPOPT_DOMAIN_CONFIGURATION_AND_INITIALIZATION_QUALIFICATION
GATE_7  ML212_SOLVE_HARNESS_WIRING
GATE_8  FULL_DETERMINISTIC_PRE_SOLVER_REQUALIFICATION
GATE_9  ONE_AUTHORIZED_SEVEN_CONTROL_IPOPT_WITNESS_ATTEMPT
GATE_10 EXACT_SAME_PLANT_REPLAY_AND_INDEPENDENT_E4_ACCEPTANCE
```

Repairs remain separate gates with separate tests and receipts.

## Next authorized gate: ML242 only

The next implementation authorization is `ML242_STRUCTURAL_NLP_REPAIR`,
limited to:

- remove x0 from the NLP decision vector and represent E3 as a fixed parameter;
- encode native Euclidean Drive/action-memory domains;
- preserve exact manifold-aware state handling;
- create exact full and seven-control witness layouts;
- update sparse structure consistently and derive dimensions from live source.

Phase-I rho may be structurally reserved only if required for exact layout
accounting. ML-241, ML-243, solve-harness, Plant, Drive, controller, event,
scorer, and test changes are outside this next gate.
