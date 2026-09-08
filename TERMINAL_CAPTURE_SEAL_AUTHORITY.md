# Terminal Momentum Capture Seal Authority (RES-58, V2.1)

`MISSION=RES10_SYNC_TRUE_MOMENTUM_TERMINAL_CAPTURE_S50_002`
`EXPERIMENT_ID=EXP-RES10-TRUE-MOMENTUM-CAPTURE-002`

This is a replay / requalification / sealing mission. It does not design a
new controller. It seals the exact executed RES-56 true-momentum law under
RES-54 / RES-55 measurement authorities and the RES-57 bilateral-support
continuity semantics.

## Frozen controller law (RES-56 executed, RES-58 sealed)

```text
VZ_TARGET = 0.0 m/s
CONTROL_DT = 0.005 s
BW = m*g, m = 95.0 kg, g = 9.81 m/s^2, BW = 931.95 N
FZ_RAW = BW - m*vz_true/CONTROL_DT
FZ_DES = clamp(FZ_RAW, 0.6*BW, 1.5*BW)
vz_target_next = vz_true + CONTROL_DT*(FZ_DES/m - g)
```

Reusable implementation: `src/loaded_cmj/v2/terminal_capture.py`.
Archived executed source SHA256:
`06e73896cbeab9dd04c49be56ace70f471c40bd3153e17f840e5b543c9fecd4d`.
Functional identity over the full vz sweep is exact (max diff 0.0).

Inner realization: sealed RES-52 full-7DOF exact-forward SoftContactPolicy.
Measurements: RES-54 whole-body COM velocity (`mj_jacSubtreeCom(pelvis) @ qvel`)
and RES-55 foot-point velocity (`J_point @ qvel` + `efc_vel`, +z separating).
No direct force/root/state writes. No event/scorer circularity.

## Support continuity (RES-57 authority)

Contract SHA256:
`cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734`.
Control-relevance dwell: `0.005 s = 40 physics steps`.
That dwell is one control interval / control-relevance dwell.

Sample-level unilateral dropouts are retained separately from
finite-duration control-relevant episodes. Geometric liftoff samples are
counted dist-exactly and never hidden by the duration rule.

## Start states

```text
S50_SHA256=bb56c0d3df3cd50aa55be515783af5cfb24f212eaaa29d17427bd3e60c6ceae2
```

S50/S75 start COM velocity is recomputed from current RES-54 authority at
every restored state; stale pre-RES54 metadata is never used.

## Qualification horizon and gates (150 ms)

Horizon: exactly 150 ms = 30 control updates = 1200 physics steps.

Require:

```text
abs(vz_true) < 0.05 continuously >= 40 ms (320 physics steps)
CONTROL_RELEVANT_SUPPORT_LOSS_COUNT == 0
CANONICAL_REFLIGHT == []
CHATTER_TRANSITIONS <= 8
peak Fz <= 8 BW
penetration <= 10 mm
prohibited=false
root limit rows=0
root passive=0
actuator bounds respected
finite=true
trust fallbacks within authority (<=5)
momentum balance within RES-54 tolerance (velocity RMSE < 0.002 m/s)
```

Retain separately:

```text
UNILATERAL_FORCE_DROPOUT_SAMPLE_COUNT
GEOMETRIC_LIFTOFF_SAMPLE_COUNT
```

Expected RES-56 quantitative behavior (reproduction tolerance):

```text
near-zero entry = 51.875 ms
zero crossing = 63.125 ms
near-zero dwell = 98.25 ms
peak = 1.797891 BW
penetration = 8.15642 mm
dropout samples at 1016 / 1025 / 1165 (0.125 ms each, non-control-relevant)
```

Historical record: RES-56 remains FAIL under its predeclared strict
`POST_WALK<=1` count (observed 3). That verdict is never rewritten. RES-58
qualifies the same trajectory under the prospectively sealed RES-57
support-continuity contract.

## Design, units, statistics, algebra

- experimental-design: replay with predeclared gates and frozen law; unit is
  one deterministic rollout per cell (S50 mandatory, S75 reference only);
  replication is fresh-process bit-identity; 1200 physics steps are not
  independent replicates; no randomization/blocking applies to deterministic
  MuJoCo; budget is 2 qualification rollouts + reproductions; no tuning or
  candidate search.
- uncertainty-and-units: units attached at input (`m/s`, `N`, `s`, `mm`);
  `FZ_RAW=BW-m*vz/DT` is `[N]-[kg*(m/s)/s]`; `vz_target_next` is
  `[m/s]+[s]*([N]/[kg]-[m/s^2])`; BW=931.95 N; peak ~1.80 BW and pen ~8.16 mm
  are plausible for 95 kg soft contact with solref 0.016; momentum tolerance
  is velocity RMSE < 0.002 m/s; dwell conversions use exact 0.000125 s steps.
- statistical-analysis: confirmatory qualification only; no p-values or
  post-hoc test shopping; descriptives plus exact gate comparisons reported;
  determinism checked by END_SHA and flicker-index identity.
- sympy: one-step deadbeat `m*(0-vz)=(FZ-m*g)*DT` gives `FZ=BW-m*vz/DT`;
  saturation edges `-0.5*g*DT=-0.024525` and `+0.4*g*DT=+0.019620` verified;
  unsaturated `vz_target_next` is identically 0.

## Scope exclusions

No E10 / E11 / E12. No takeoff changes. No propulsion changes. No
Plant/contact/scorer changes. No solver/event/support-contract changes.
If S50 fails any current qualified gate: no commit, owner adjudication.
