# ADR — current public LCMJ V1 architecture after F2 / ML-237

Status: accepted for the current public V1 boundary
Model: `LCMJ-V1-20KG-HIGHBAR@1.0.0`
Repository: `/home/litju/Projects/loaded-cmj-control`
Branch: `main`

## Owners

- `Plant` owns the compiled MuJoCo model, reset, contact/force plates/COP,
  whole-system COM, acceleration, linear momentum, and centroidal `H/Hdot`.
- `DriveState.drive_state_step` owns production actuation, realized torque,
  saturation, rate/power limits, and trusted drive flags.
- `BiomechanicalSample.from_plant` owns trusted post-step sample construction.
- `runtime.results` owns immutable result sealing and canonical trace bytes/
  digest (`LCMJ-V1-TRACE-1.0.0`); `RolloutEngine` owns the causal clock.
- `CMJEventDetector` remains the independent event authority.
- `biomechanics.metrics.derive_force_time_metrics` is the single canonical
  owner of force-time derived metrics; the event result projects that owner
  and does not maintain a parallel impulse/loading-rate formula.
- `biomechanics.capturability.predict_support_reserve` owns the internal
  finite-horizon sagittal support-reserve diagnostic. It consumes Plant-owned
  support geometry and does not infer events or enter policy observations.
- `biomechanics.effectiveness.estimate_local_effectiveness` owns the bounded
  local characterization diagnostic. It consumes the shared exact 5 ms
  transition and is not a controller allocator or search.
- `PolicyWorker` owns process isolation and the public action/observation
  boundary; private trusted mechanics are not policy observations.
- `replay_input` consumes the sealed trace and never resimulates.

## ML-237 transition ownership

- `simulation.transition.step_5ms` is the one authoritative exact 5 ms
  transition owner: one accepted-action projection followed by 40
  `PHYSICS_TIMESTEP_S` substeps.
- `simulation.transition.project_accepted_action` is the one generic
  accepted-action slew owner, with the frozen 0.20 per-control-interval
  maximum.
- `RolloutEngine` remains orchestration, PolicyWorker lifecycle, sampling,
  event, termination, and result construction owner; it does not own the
  physical substep loop.
- `DriveState.drive_state_step` remains the sole realized-actuation owner.
- `Plant.apply_anatomical_torque` and MuJoCo remain downstream physical
  owners; no second Plant is introduced.
- `biomechanics.effectiveness` consumes the same transition owner for local
  characterization. A future privileged oracle may consume that owner after
  the later snapshot/restore and oracle gates; no oracle infrastructure is
  added by ML-237.
- The transition owner does not own public observations, PolicyWorker
  process management, events, scoring, termination, replay, or snapshots.

## Causal direction

`policy action -> PolicyWorker -> protocol validation -> RolloutEngine ->
DriveState.drive_state_step -> Plant/MuJoCo -> BiomechanicalSample ->
CMJEventDetector -> RolloutResult -> replay_input`.

The bilateral six-axis plate reconstruction, total common-origin wrench, COP
validity, permitted/prohibited contact semantics, propulsion/landing intervals,
PFIP behavior, public observation schema, and event thresholds are protected.
No second measurement, trace, event, runtime, replay, or actuator owner may be
introduced. F3/F4 controller work is out of scope.

F2 removed the proven-dead aggregate drive compatibility paths and migrated
their qualification callers to copied `DriveState` propagation.

F3 independently qualified the Plant, DriveState/passive mechanics, force
plates/trace, force-time metrics, momentum/wrench/energy closure,
support-reserve diagnostic, local effectiveness diagnostic, event DAG, and
numerical/runtime substrate. The F3 freeze is a hash-manifest freeze because
the worktree intentionally retains pre-F3 dirty controller/runtime history;
the protected F3 source set is recorded in the terminal evidence bundle.

The requested codebase-memory `manage_adr` endpoint was unavailable in this
session. The external evidence ADR and this repository ADR are the explicit
fallback records; graph search/query/architecture receipts remain authoritative
for structural discovery only.

## ML-238 exact macro-state boundary

`simulation.snapshot.MacroSnapshot` is the single raw restart owner. It is a
frozen, copy-safe object and restores into the same qualified `Plant`/`MjData`
ownership before the caller invokes `simulation.transition.step_5ms`. Restore
does not call `mj_forward`: the shared transition reads four selected body
rotation rows from MuJoCo's post-step kinematic cache before the first physics
substep. Those rows are therefore retained in the raw snapshot together with
`qpos`, `qvel`, `time`, `qacc_warmstart`, `ctrl`, causal DriveState fields,
DriveState diagnostics, and the single previous accepted action. Applied-force
buffers are rejected when nonzero and canonicalized to zero on restore.

The frozen raw schema is `LCMJ-V1-MACRO-SNAPSHOT-1.0.0`. The future optimizer
macro state contains current configuration tangent coordinates (21), the
independent three-ball-joint relative-rotation cache (9), qvel (21), causal
DriveState (`a_plus`, `a_minus`, `tau_prev`, 45), previous accepted action (15),
and qacc warm-start (21): 132 dimensions. The qpos representation remains
exact MuJoCo qpos; ML-239 defines its local configuration tangent without
changing this raw schema.

`previous_command`, `reversal_phase`, and `override_flags` are retained only
for raw DriveState identity/diagnostics. `previous_command` is a redundant
alias of the macro previous accepted action after each transition and is not
counted twice. Time is retained for raw replay identity but is not an
optimizer variable because V1 physical code is autonomous in absolute time.
Policy-worker memory, PFIP memory, RolloutEngine counters, event latches,
metrics, scorer state, trace serialization, and attempt identifiers remain
evaluator/orchestration state and are excluded from the physical boundary.

## ML-239 local tangent geometry

simulation.tangent.boxplus and simulation.tangent.boxminus own the
privileged local geometry only. Their canonical layout is
LCMJ-V1-TANGENT-STATE-132-1.0.0:
configuration [0:21), cache SO(3) [21:30), qvel [30:51), causal DriveState
[51:96), previous accepted action [96:111), and qacc warm-start [111:132).

Configuration uses MuJoCo mj_integratePos and mj_differentiatePos.
The three cache blocks are right local coordinates of the source-qualified
relative rotations R_parent.T @ R_child for lumbar, left hip, and right hip;
their raw xmat reconstruction retains the body-1 anchor and reconstructs the
three child rows. The cache coordinates are numerical transition state, not
physical DOFs. Euclidean blocks use direct add/subtract with no clipping,
action slew, or implicit scaling. The SO(3) local branch is strict at
theta < 1.0 rad; nonlocal pairs are rejected. This layer does not own
dynamics, derivatives, constraints, solvers, oracle solves, or public state.
