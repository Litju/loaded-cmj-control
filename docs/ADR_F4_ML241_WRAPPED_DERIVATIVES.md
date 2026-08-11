# ADR F4 ML-241 — Wrapped local derivatives

Status: implementation present; qualification blocked 2026-08-10

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
- The prospective step study currently has no stable plateau for the causal
  `qacc_warmstart` translation or rotation/joint blocks. Those steps remain
  unfrozen; no fallback derivative is accepted and no ML-242 consumer may
  treat the exploratory qacc columns as qualified.

This ADR does not authorize changes to the physical model, transition,
snapshot, tangent geometry, actuator law, contact law, event/metric owners,
public observations, or constraint catalog. It does not define ML-242 source
work or an optimization problem.
