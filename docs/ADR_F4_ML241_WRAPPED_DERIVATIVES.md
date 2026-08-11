# ADR F4 ML-241 — Wrapped local derivatives

Status: qualified; ML-241 PASS after bounded qacc_warmstart blocker resolution

## Decision

The privileged local differential owner is
`src/loaded_cmj/oracle/derivatives.py:linearize_step_5ms`. It computes the
132×132 state matrix and 132×15 raw-action matrix by restoring an immutable
`MacroSnapshot`, perturbing through the frozen `boxplus`, calling the frozen
`src/loaded_cmj/simulation/transition.py:step_5ms`, and expressing both output
perturbations with `boxminus` in the common tangent frame at the base next
state.

The raw-action derivative includes the existing validation seam, accepted
action projection, accepted-action slew, `DriveState`, realized torque, and
all 40 MuJoCo substeps. No simplified actuator or second plant is permitted.

The authoritative wrapped derivatives are `A ∈ R^(132×132)` and
`B ∈ R^(132×15)`. ML-241 passed after resolving the bounded
`qacc_warmstart` blocker. `qacc_warmstart` remains part of the exact 132-D
restart/tangent state. Its incoming A columns `111:132` are
`NUMERICALLY_NULL_WITH_BOUNDED_ERROR`: they are qualified-zero columns, not
missing or unavailable derivatives. The qacc numerical-null certificate
passed.

The material nonzero derivative blocks use their qualified finite-difference
plateau contract. Directional consistency, derivative repeatability,
fixed-contact active-set qualification, and action-slew kink classification
all passed. ML-242 is authorized to consume the qualified A/B matrices and
validity metadata.

## Frozen contract

- State dimension: 132, layout `LCMJ-V1-TANGENT-STATE-132-1.0.0`.
- Raw action dimension: 15.
- Snapshot schema: `LCMJ-V1-MACRO-SNAPSHOT-1.0.0`.
- Derivatives are blockwise unit-aware finite differences; no universal step.
- Every perturbation restores the identical base snapshot. Perturbations are
  never chained through a live `MjData`.
- Contact identities, support activity, COP validity, native-limit rows,
  friction/prohibited-contact truth, and actuator projection flags form the
  local active-set fingerprint.
- Central differences through action-slew/action-box kinks and unscheduled
  contact/guard changes are rejected and explicitly masked.
- Source-owner output sensitivities use only ML-240-approved continuous
  owners. Event order/dwell, contact creation/loss, capturability hardening,
  interval aggregates, and post-trace predicates are not differentiated.
- `mjd_transitionFD` is a diagnostic cross-check only; it is not equivalent to
  the wrapped derivative.
- The earlier blocked qacc plateau result was an intermediate qualification
  state and is superseded by the final blocker-resolution evidence. The
  qacc translation and rotation/joint blocks are qualified zeros under the
  bounded-error numerical-null contract; they are not missing derivatives.

This ADR does not authorize changes to the physical model, transition,
snapshot, tangent geometry, actuator law, contact law, event/metric owners,
public observations, or constraint catalog. The resolution changed none of
the physical model, `step_5ms`, snapshot schema, tangent layout, constraint
catalog, or production solver. It does not define ML-242 source work or an
optimization problem.
