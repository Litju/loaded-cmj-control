# Controller Observation Contract — Synchronized Sample-Before-Update Authority

**Status:** NORMATIVE for RES10_CONTROLLER_OBSERVATION_SYNCHRONIZATION_REQUALIFICATION
(EXP-RES10-CONTROLLER-OBS-SYNC-001 v1.0.0).
**Supersedes:** legacy mixed-stage controller observation (qpos/qvel current,
derived forces stale by one physics step).
**Frozen companions:** docs/MUJOCO_STATE_STAGE_CONTRACT.md,
src/loaded_cmj/v2/measurement.py, src/loaded_cmj/v2/controller.py (unchanged),
V2 Plant/contact/solver/timestep/actuator/scorer thresholds/RES43 envelope.

---

## 1. Control-sample convention (frozen)

`CONTROL_SAMPLE_CONVENTION=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL`

At each control boundary `t_k` define:

- `x_k` = exact live `mjSTATE_INTEGRATION` at `t_k` (108 floats:
  time, qpos, qvel, qacc_warmstart, ctrl, qfrc_applied, xfrc_applied).
- `u_prev` = control physically held over `[t_{k-1}, t_k)` and still present
  in `live.ctrl` immediately before the new controller command.
  At `k=0`, `u_prev = zeros(7)` from reset.

Evaluation order (mandatory):

1. Copy ONLY authoritative integration-state inputs from LIVE into persistent
   SHADOW via `mj_getState(live) → mj_setState(shadow)`.
2. Run `mj_forward(model, shadow)`. The shadow solve uses `x_k` and `u_prev`.
3. Construct the complete public controller observation from SHADOW only.
4. Compute `u_k = controller(observation_sync)`.
5. Only after controller output is finalized: `apply u_k to LIVE` and advance
   the next 5 ms physical interval (40 × 0.000125 s `mj_step` on LIVE only).

Do NOT call `mj_forward` on live `MjData` for observation synchronization.

## 2. One state → one observation (frozen)

Owner: `src/loaded_cmj/v2/measurement.py::SynchronizedPhysicsSample`.

At every control boundary:

`CONTROLLER_OBSERVATION_STATE_SHA256 == PHYSICS_SAMPLE_STATE_SHA256`

for that same `t_k`. The observation dict carries
`observation_state_sha256`; the event sample carries
`physics_sample_state_sha256`; both equal the sample's
`state_vector_sha256 = SHA256(mjSTATE_INTEGRATION bytes at t_k)`.

No mixed live/shadow fields. No qpos from LIVE plus Fz from SHADOW.
Everything physical in the observation comes from the same synchronized
sample, except explicitly external bookkeeping fields:

- `step_index`
- `episode_reset`
- `previous_action`

which are labeled non-Plant metadata (`NON_PLANT_METADATA_KEYS`).
`observation_time_s` equals the sample time `t_k`.

Physics trace, event detector, scorer, and forceplate observation must
consume the same `SynchronizedPhysicsSample` (via `.controller_observation()`
and `.event_sample()`), never independent reconstructions.

## 3. Pre-update force semantics (frozen)

`OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_CONTROL`

Force / wrench / CoP in the controller observation at `t_k` are the
instantaneous forward-dynamics quantities at state `x_k` under the command
held immediately before the controller update (`u_prev`).

They are NOT forces resulting from the newly computed `u_k`. This avoids an
algebraic control loop.

`NEW_CONTROL_EFFECTIVE_INTERVAL=[t_k, t_{k+1})`

Consequences:

- Shadow `ctrl` after sync equals `u_prev` (it is part of `x_k`).
- `previous_action` metadata passed to the observation must equal `u_prev`
  (checked by tests; mismatch is fail-closed).
- `qfrc_actuator` in the sample is recomputed from `u_prev` (safe).
- `qacc(T)` is the instantaneous acceleration at `T` under `u_prev`, NOT the
  acceleration that produced `T-dt → T`.

## 4. Shadow nonintrusiveness (frozen)

Same prescribed action sequence:

- legacy LIVE simulation
- vs LIVE simulation with synchronized shadow observation/evidence computation

requires `MAX_QPOS_DELTA=0`, `MAX_QVEL_DELTA=0`, `MAX_CTRL_DELTA=0`.

Shadow measurement is observational only: read-only `mj_getState` on live,
`mj_setState` + `mj_forward` on shadow, all derived reads from shadow.
Live integration state, live derived buffers, and live trajectory are
bit-identical with or without the shadow present.

## 5. Legacy contract (superseded, preserved for comparison)

`LEGACY_OBSERVATION_CONTRACT=MIXED_STAGE_POST_STEP_READ`

Legacy observation read derived quantities from LIVE immediately after
`mj_step` at control boundaries: qpos/qvel/ctrl/time current at `t_k`,
but COM/COM-velocity/GRF/moment/CoP/contact/qacc stale (belonging to
`t_k − 0.000125 s`). Controller phase transitions driven by stale Fz/com_vel
differ materially from synchronized (first divergence ≈0.615 s, max
normalized action delta ≈0.703647 in EXP-RES10-PHYSICS-SAMPLE-SYNC-001).

Legacy trajectory claims are classified
`HISTORICALLY_REPRODUCIBLE_UNDER_LEGACY_OBSERVATION`,
`SUPERSEDED_FOR_CURRENT_CLOSED_LOOP_AUTHORITY`. R5A2/R5A3 evidence is
preserved; Plant/contact/actuator/inverse-dynamics/event-definitions are
unchanged.

## 6. New contract (current authority)

`NEW_OBSERVATION_CONTRACT=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL`

with `OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_CONTROL`,
`ONE_STATE_ONE_OBSERVATION` SHA identity at every `t_k`,
and no `mj_forward(live)` in any production sampling path.

`SPEC_EXECUTION_MATCH` must PASS for any claim under this contract.
