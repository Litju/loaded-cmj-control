# lcmj-controller-ship learnings

This file is append-only by default. It contains verified evidence pointers,
not authority that can supersede a current owner mission, live source, frozen
contracts, or immutable receipts.

## AUTHORITY RECOVERY

Verified by:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-PROJECT-RECOVERY-20260817/AUTHORITY_RECEIPT.md`

* Hermes context is quarantined.
* Hermes is non-authoritative.
* R-series PFIP is the active track.
* R036 is the accepted baseline controller.
* The rollout-ledger pointer is the authority receipt above: cap 60, used 43,
  remaining 17, next rollout none at mission start.

## CLOSED R038-R043 LESSONS

Verified by the R-series receipts under
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-PRIMARY-CONTROLLER-SHIP-V1-20260817T040330Z/rollouts/`.
These entries are not reinterpreted beyond those receipts.

* R037: global knee increase regressed.
* R038/R039: knee-support protection did not close reversal.
* R040: hip-only slew retention insufficient.
* R041: upstream command was later allocator-limited.
* R042: command-delivery blocker closed; earlier accepted action can
  physically arrive.
* R043: predictive arrest demand implemented; structural failure classified as
  allocator unable to realize demand.
* The local timing/micro-patch path is exhausted.

## PUBLIC-WRENCH CONTRACT LESSON

Verified by:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-PUBLIC-SUPPORT-WRENCH-CONTRACT-V1_1-20260818/PUBLIC_SUPPORT_WRENCH_CONTRACT_V1_1.md`
and `tests/test_public_support_wrench_contract.py`.

* The aggregate-only V1 contract was mathematically insufficient to recover
  independent left/right force-plate observations.
* V1.1 preserves per-foot 6-D wrenches.
* The whole-support wrench is derived by summation about a common origin.

## CURRENT NEXT UNIT

Only after this mission PASS:

```text
PFIP BRAKING-v2
control-effectiveness model
+
rate/support constrained incremental allocator
```

No R044 until its offline qualification passes.

## MISSION ENTRY — 2026-08-18

Mission:
`LCMJ_PUBLIC_SUPPORT_WRENCH_CONTRACT_V1_1`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-PUBLIC-SUPPORT-WRENCH-CONTRACT-V1_1-20260818/`

Result: PASS for the additive public support-wrench contract and deterministic
proof suite; historical R025-R043 traces are not directly V1.1-schema-ready
and were not rewritten.

Newly established invariants:

* public plantar force is `(2,3)` in `[left,right] x [Fx,Fy,Fz]`, world frame,
  environment-on-system, in N;
* public plantar moment is `(2,3)` in `[left,right] x [Mx,My,Mz]`, world frame
  about `[0,0,0]`, environment-on-system, in N*m;
* the two public moments share one origin, so their bilateral wrench is their
  direct sum;
* existing Fz, CoP, CoP validity, timing, and PFIP behavior remain unchanged;
* no development rollout was consumed.

Explicit next authorized unit:
zero-rollout PFIP BRAKING-v2 control-effectiveness modeling plus
rate/support-constrained incremental allocator implementation and offline
qualification. Do not run R044 until that qualification passes.

## MISSION ENTRY — 2026-08-18

Mission:
`LCMJ_BRAKING_V2_DATA_AND_IDENTIFIABILITY_GATE`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-BRAKING-V2-DATA-GATE-20260818/FINAL_RECEIPT.md`

Projection result: PASS; historical R025-R043 traces were unchanged and
projected into the V1.1 per-foot world-origin wrench contract.

Causal lag result: PASS; `CAUSAL_CONTROL_LAG_TICKS=1`.

Numerical rank: 3.

Qualified input subspace: full `[hip,knee,ankle]` three-dimensional retained
SVD subspace; basis and projection are in `IDENTIFIABILITY_RECEIPT.json`.

R036 coverage result: 18/68, fraction `0.2647058823529412`; the required
1.0 coverage gate therefore failed.

R036 residual bound path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-BRAKING-V2-DATA-GATE-20260818/R036_VALIDATION_METRICS.json`

Allocator authorization: NO.

Next authorized unit: NONE; stop before allocator implementation or new
rollout data and require owner authorization for any scope expansion.

## Self-evolution protocol

Future explicitly authorized missions that invoke this skill must:

1. read the final immutable receipt;
2. append only evidence-backed lessons;
3. record date, mission, receipt path, accepted/rejected result, newly closed
   hypothesis, newly established invariant, and explicit next authorized unit;
4. never record speculation as fact;
5. never rewrite history; and
6. never alter rollout counts from memory.

`SKILL.md` changes require either explicit owner authorization or a durable
operating rule supported by at least two independent receipts and consistent
with higher authority. Each such change appends:

```text
DOCTRINE_CHANGE
date=
evidence_receipts=
reason=
```

Prior lessons are never automatically deleted.

## MISSION ENTRY — 2026-08-19

Mission:
`LCMJ_BRAKING_V2_MODEL_VALIDITY_CLOSURE`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-BRAKING-V2-MODEL-VALIDITY-20260818/FINAL_RECEIPT.md`

MODEL_S result: the mandated no-intercept one-tap model was fit from 922
valid symmetric BRAKING records from R025-R042 excluding R036 and R043. It
was evaluated on 68 independent R036 BRAKING records; its per-component and
whole-support residual metrics are recorded in the receipt artifacts.

MODEL_D result: the mandated no-intercept two-tap model had finite outputs,
deterministic coefficients, numerical rank 6, and condition number
`2.938415537254189`. It failed the deterministic Phase-6 adjudication because
whole-Mx RMSE was not <= MODEL_S and whole-My absolute lag-1 residual
autocorrelation increased. No Phase-7 authority test was entered after that
failure.

Qualified model or blocker: no model qualified;
`PRIMARY_BLOCKER=STATIC_AND_TWO_TAP_MODELS_INADEQUATE`.

R036 model-error bound path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-BRAKING-V2-MODEL-VALIDITY-20260818/MODEL_D_R036_METRICS.json`
contains the componentwise R036 MAX_ABS bound. The bound was not used to make
an allocator-authority claim because MODEL_D failed Phase 6.

Robust-authority result: `NOT_RUN_MODEL_D_PHASE6_FAIL`; zero R036 authority
ticks were adjudicated, with zero rollouts consumed.

Explicit next authorized unit: STOP. Do not run R044, collect new data, or
select another model without owner authorization.

## MISSION ENTRY — 2026-08-24

Mission:
`LCMJ_SHIP_FUNCTIONAL_COMPLETE_JUMP_TODAY`

Evidence root:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-FUNCTIONAL-JUMP-SHIP-20260824/`

### R045 — REJECTED

* Controller SHA256: `3839d3bcf2a60c2685e9bea159779d4fe31423dc220c003e67003fe36eede1fb`
* Canonical result: PHYSICAL_FALL at physics index 9564 (t=1.1955 s);
  canonical events reached: supported_start (0.10025 s),
  countermovement_onset (0.25241 s). Failed before valid_countermovement.
* Frontier vs R036: no regression risk taken deliberately; the candidate moved
  the BRAKING takeover from R036's 0.095 m to 0.060 m COM depth.
* Earliest failed phase: ARREST (new law), entered prematurely.
* Exact causal repair performed next: restore R036's 0.095 m takeover so the
  entire countermovement remains byte-identical nominal behavior; ARRET must
  work from the deep, fast state instead.

### Newly closed hypotheses / invariants

* CLOSED: "Early takeover (shallow depth) makes reversal easier." At 0.06 m
  the arrest bite works briefly but support sags before the E3 gate; the
  canonical full-rate detector then sees a fall without valid_countermovement.
* ESTABLISHED: offline event-chain verification MUST sample near the canonical
  full physics rate. A 200 Hz diagnostic view credited valid_countermovement
  where the canonical 8000 Hz engine correctly did not (detector dwell/ordering
  is sampling-sensitive near the E3 boundary).
* ESTABLISHED: plant actuator asymmetry under load — knee-extension
  torque-angle capacity collapses with flexion depth (~300 N*m bar to <110
  N*m realized at squat angles); hip extension loads/unloads plates
  asymmetrically; ankle plantarflexion commands beyond about -0.45 action
  rotate the foot onto its envelope and fold the leg chain. Sustained vertical
  surplus at compliant depths is a knife-edge; only small trims survive.

### R046 — REJECTED (second and final development rollout of this mission)

* Controller: R036 + quasi-static ARREST_DRIVE extension stage (settle-gated),
  takeover depth restored to R036's 0.095 m.
* Canonical result: PHYSICAL_FALL before valid_countermovement (2 canonical
  events). Same frontier as R045 despite the repair.
* Exact causal repair attempted: restore nominal takeover + settle-gated slow
  extension. Outcome: still no reversal; the supported braking equilibrium has
  effectively zero stability margin — every added sustained moment (hip
  extension, ankle surge small or large, lumbar ramp, knee trim) tips the
  state into collapse instead of reversal.

### MISSION CLOSE-OUT — capability evidence for the owner

Brute-force capability probe (max-extension action bursts from controller-
reached states): peak vertical ground reaction reached only 1.29–1.39 BW for
~50 ms near the E3 gate; required mean surplus for upward_reversal from the
canonical entry state is ~1.34 BW sustained ~60 ms plus activation ramp lag.
The window is knife-edge marginal; 19 distinct deterministic feedback designs
(extension schedules, velocity servos, two-stage sequencing, proximal-distal
gating, warm/cold channel scheduling, rescue-preserving bias ramps) all fell
on one side of it.

Explicit next authorized unit (owner decision required, outside this
mission's authority):
either (a) authorize unfreezing the nominal-region takeover timing/descent
target so arrest begins while quadriceps capacity is far from its ceiling, or
(b) authorize an oracle-informed reference study of what whole-body trajectory
the frozen plant CAN jump with, as a diagnostic (not a controller), before any
further shipping rollouts are spent.

## MISSION ENTRY — 2026-08-25

Mission:
`LCMJ_PROGRESSIVE_BRAKING_ROOT_FIX_NEXT_CANDIDATE`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-PROGRESSIVE-BRAKING-ROOT-FIX-20260825/FINAL_RECEIPT.md`

Candidate: R047, policy SHA256
`960f988e7074bca6805311948eb4315b6e263979e34cb1011910d058b84aec1b`
(R036 with the late 0.095 m depth-gated takeover replaced by progressive
braking: onset = first upward whole-Fz crossing of system weight while
descending after the canonical E2 onset dwell; braking authority ratcheted by
at most one reachable accepted-action increment along the R036 braking vector
while measured whole Fz is below M*(g + vz^2 / (2*d_remaining)), d_remaining
measured to the authoritative 0.12 m counter target; single vertical
authority; ankle commands floored at the documented -0.45 fold threshold.)

Measured (canonical rollout, fresh worker):
* braking onset t=0.515125 s at depth 0.0429 m, vz -0.2792 m/s,
  whole Fz 949.84 N;
* braking duration 0.42963 s; mean whole Fz during braking 745.64 N;
  peak 1300.20 N; net braking impulse -80.07 N*s;
* events reached: supported_start (0.100250), countermovement_onset
  (0.252411), valid_countermovement (0.694500);
* earliest failed event: upward_reversal; termination PHYSICAL_FALL at
  0.944875 s; peak downward COM velocity -1.1221 m/s.

Accepted/rejected: REJECTED (Case B adjudication).

Root conclusion: PROGRESSIVE_BRAKING_INSUFFICIENT. The prescribed physical
trigger fired correctly and the initial arrest was real (vz -0.279 -> -0.223
within ~23 ms, peak Fz above requirement), but sustained delivery crossed the
documented plant ceiling: support force collapsed below system weight (mean
below weight over the window), feet unloaded momentarily (weak-foot force
0.0 N), and no bounded action restored an arrestable state. The binding
constraint on E4 is whole-body support margin during mid-descent braking,
not trigger timing or braking-command magnitude. Newly reinforced invariant:
any mid-descent law that drives sustained vertical surplus at compliant
depths will lose weak-foot support before reversal on this frozen plant.

Next permitted action: NONE without owner authorization. R048/R049 remain
unspent deliberately. Owner decision required between (a) a
support-preservation study for mid-descent braking (CoP/trunk/envelope), or
(b) the oracle-informed diagnostic study of what trajectory the frozen plant
can jump with (diagnostic only).

## MISSION ENTRY — 2026-08-25

Mission:
`LCMJ_R048_SUPPORT_COLLAPSE_ROOT_FIX`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-R048-SUPPORT-COLLAPSE-ROOT-FIX-20260825/FINAL_RECEIPT.md`

Result: PASS for the audit; hypothesis REJECTED; zero rollouts spent
(R048/R049 remain unspent). PFIP unchanged at R047
(`960f988e7074bca6805311910d058b84aec1b` prefix `960f988e...`, verified at
entry and exit).

Newly established evidence (deterministic full-rate replay of exact R047,
byte-identical to the immutable canonical receipt, determinism re-run
verified):

* CLOSED: "Weak-foot support scaling initiated the mid-descent collapse."
  Every collapse-onset stage precedes the first support-scale withdrawal:
  single-substep chatter dropout t=0.531000 s, decay from the 1300.202 N peak
  from ~0.533 s, sustained sub-BW crossing ~0.5556-0.5560 s — versus first
  BRAKING-phase withdrawal (scale<1) at control tick t=0.560 s and scale=0 at
  t=0.565 s. Support scaling materially AMPLIFIED the terminal fall (global
  blend-to-HOLD erased posture authority on 74% of post-withdrawal ticks;
  force fell to ~550 N with 0 N chatter; vz ran to -1.122 m/s) but did not
  initiate it.
* ESTABLISHED: the earliest binding mechanism is bilateral knee flexion-rate
  runaway under load: with brake_level HELD at 0.444 and whole Fz still above
  fz_required (~956-980 N), knee flexion rate grew monotonically -2.08 ->
  -4.36 rad/s (t=0.520-0.555) while flexion deepened -0.450 -> -0.572 rad,
  driving the joints into the documented flexion-depth torque-capacity
  collapse zone. The ratchet law's hold condition (whole_fz >= fz_required)
  was satisfied throughout the decay — the law could not see this failure mode.
* ESTABLISHED: feet were force-symmetric during braking until collapse
  ("weak foot" = both feet); CoP stayed valid with stable x (-0.112 ->
  -0.099 m); trunk lean stable; ankle-envelope and knee guards active from
  the first braking tick without preventing the healthy 1300 N build (not
  dominant binders).
* METHOD INVARIANT reinforced: full-rate (8000 Hz) traces are mandatory for
  collapse adjudication — single-substep contact chatter dropouts (e.g.
  t=0.531000 s amid 1250-1300 N readings) masquerade as threshold crossings
  at any coarser sampling.

Explicit next authorized unit: NONE without owner authorization. Owner
decision required between (a) a read-only capability study of realized
knee-extension capacity vs flexion depth in braking states, or (b) the
oracle-informed diagnostic trajectory study (diagnostic only). Do not spend
rollouts on weak-foot blending semantics.

## MISSION ENTRY — 2026-08-26

Mission:
`LCMJ_R048_KNEE_STATE_VIABILITY_ROOT_FIX`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-R048-KNEE-STATE-VIABILITY-20260825/FINAL_RECEIPT.md`

Result: PASS for the audit; KNEE_AUTHORITY_CLASS=UNUSED_AUTHORITY_COMMAND_PATH_FAILURE
established from deterministic read-only replay; ONE complete canonical R048
run (policy SHA256 `1af994e6e3e32afda63586ef67a4046b208442462df794f1cf46901fd1a556a3`)
and adjudicated. Canonical frontier UNCHANGED (supported_start,
countermovement_onset, valid_countermovement PASS; upward_reversal FAIL;
PHYSICAL_FALL at 0.982125 s vs R047 0.944875 s).

Newly closed hypotheses / established evidence:

* CLOSED (adjudicated): "the knee extensor channels were already at their
  realizable extension/braking capacity" (Case A REFUTED). During the entire
  R047 runaway window the knee envelope held 56-70 N*m of unused directional
  reserve (utilization 39-52%, never >=99% anywhere in 0.400-0.700 s).
* ESTABLISHED (Case B CONFIRMED): from the first BRAKING tick the horizontal-
  capture allocator pinned its knee residual at the -0.20 step limit (tiny
  forward com_vx drift suffices via G_FX[2]) and the residual cap
  `min(command, HOLD+delta)` withdrew the composed arrest demand (1.34-2.24)
  down to a returned 0.200 while flexion rate ran -2.07 -> -4.35 rad/s.
  Slew was NOT binding; support_scale=1.000; joint guards inactive.
* ESTABLISHED sign convention (source-bound): anatomical knee s=-1.0*q_mjc;
  flexion = negative anatomical coordinate/rate; arrest = positive anatomical
  torque (a_plus side, tau_bar 140 N·m); ctrl = -1.0*tau_anat into MJCF
  forcerange [-300,+140].
* R048 repair: residual cap may trim the composed base but never below the
  committed progressive-braking support level (HOLD+brake_level*KNEE_ACTION),
  mirroring the existing hip rate-feasibility doctrine; no new gain,
  threshold, or bound.
* R048 canonical effect: every braking-phase quantity improved (peak knee
  rate -4.69 -> -4.18 rad/s; vz min -1.122 -> -0.988 m/s; mean Fz 745.6 ->
  787.7 N; net impulse deficit -80.07 -> -67.37 N*s; fall delayed +37 ms;
  E3 preserved) but the sustained sub-BW collapse and PHYSICAL_FALL persist:
  delivering the withheld knee authority is necessary, not sufficient.
* METHOD INVARIANT: the runtime worker step timeout (WORKER_STEP_TIMEOUT_S =
  0.020 s incl. IPC) is fragile on loaded hosts — seven of eight canonical
  executions faulted POLICY_TIMEOUT at non-deterministic ticks before one
  complete episode landed; all faulted receipts are preserved alongside the
  authoritative complete receipt.

Explicit next authorized unit: NONE without owner authorization. Evidence-
ranked next binding question: whole-body vertical support margin at compliant
depths with committed knee authority delivered — hip extension channels
still realize flexion-side torque (~-68 N*m realized vs ~+180 N*m unused
extension envelope during braking) while ankles run at 70-81% utilization.
Do not spend rollouts on knee-command starvation again; it is fixed.

## MISSION ENTRY — 2026-08-26

Mission:
`LCMJ_R049_HIP_AUTHORITY_AND_TASK_PRIORITY_CERTIFICATE`

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-R049-HIP-AUTHORITY-TASK-PRIORITY-20260826/FINAL_RECEIPT.md`

Result: Stage-A certificate PASS with
`HIP_AUTHORITY_CLASS=DISCRETE_COMMAND_PATH_CANCELLATION`; ONE minimal R049
correction implemented, qualified (52/52 focused tests incl. V1.1 wrench,
three reviews with no HIGH/CRITICAL), and adjudicated by exactly one complete
canonical rollout after eight preserved AGENT_FAULT infra receipts.
R049 policy SHA256 `5ba8dcd47fda558cbc96b44c7bdf0223815e8fa12305ce6bd5579b5bea38d231`.
Canonical frontier UNCHANGED (E1/E2/E3 PASS at 0.100250/0.252411/0.698875;
upward_reversal FAIL; PHYSICAL_FALL at 1.18825 s vs R048's 0.982125 s).

Newly closed hypotheses / established invariants:

* BOUND SIGN CONVENTION (numeric probe + source): public hip coordinate is
  same-sign conjugate with torque — POSITIVE hip action/torque = FLEXION;
  hip EXTENSION is the NEGATIVE side (hold-state check cap_lo=-181.145 ==
  -250*f_q_neg(0)). The historical "realized -68 N*m flexion-side" gloss was
  sign-inverted: braking hip torque was already extension-side, just capped.
* CLOSED: "hip authority may be misallocated or cancelled during braking."
  CONFIRMED as discrete cancellation: pfip BRAKING ratchet hip legs
  (`+= 0.45*brake_level`) injected flexion-side demand on every braking tick,
  trimming composed extension support (-0.20..-0.45) while 162-169 N*m of
  directional reserve sat idle (utilization 29-30%, no Drive flags). R049
  flipped exactly that polarity (no gain/bound/threshold change).
* ESTABLISHED: delivering the withheld hip authority works mechanically
  (realized extension torque 73.9 -> 96.3 N*m peak, hips extend to
  -3.01 rad/s, peak Fz 1337.8 -> 1368.1 N, E3 29 ms earlier, fall 206 ms
  later) but does NOT advance the frontier — support still decays through
  body weight at ~0.561 s. Hip command-path repair is exhausted as the
  binding bottleneck.
* NEW BINDING REGIME (post-E3 deep sink): once the depth target is exhausted,
  fz_required becomes None and the allocator vertical solve REACTIVATES,
  injecting flexion-side hip demand up to its +1.0 clamp; knees saturate
  their directional envelope (utilization 1.000), the knee rate runaway
  returns (-5.38 rad/s), trunk folds (lumbar s0 to -1.014), support decays
  with repeated bilateral dropouts (mean post-E3 Fz 622.8 N).
* BEST CONTROLLER remains R048 on braking-phase magnitudes (sink -0.99 vs
  -2.15 m/s; impulse deficit -67 vs -177 N*s); neither dominates on frontier.

Explicit next authorized unit: NONE without owner authorization. Owner
decision required between (a) read-only study of the post-E3 deep-sink regime
(knee envelope exhaustion vs support decay; allocator post-depth vertical-solve
reactivation polarity — now the largest identified anti-support command-path
term), or (b) accepting R048 as the shipping frontier and re-scoping away from
mid-descent braking. Do not reopen hip braking-vector polarity (fixed and
covered by tests/test_hip_braking_polarity.py).

## MISSION ENTRY — 2026-08-29

Mission:
`RES4_SEAL_QUALIFIED_V2_BASELINE`
Linear: `RES-4` (Research — Loaded CMJ — MuJoCo V2.1 Ship)

Receipt path:
`/home/litju/Projects/loaded-cmj-control-evidence/LCMJ-V2-BASELINE-SEAL-20260829/CHECKPOINT_RECEIPT.md`

Result: PASS — qualified V2 baseline sealed through descending_landing (9/9 upstream).

Checkpoint:
- ENTRY_HEAD=5497c122eab60101a2a017e7c84061478555adda (V1 live head, preserved)
- BASELINE_COMMIT_SHA=a04b5739cd00f0a5bc8f70a3e7c04db0dee1754b
- BASELINE_COMMIT_TREE=5e9213e790ec2fe94544f1d6d76fea8d3354760f
- V1_PRESERVED=YES (no V1 Plant/MJCF/Drive/Event/Scorer mutation; V2 isolated under src/loaded_cmj/v2/)
- V2_SYSTEM_ID=loaded-cmj-20kg-athlete-v2 (MJCF sha 8926ea6a5355e369831e00e38b3dc9ee1b8a0451da5255d081c135bace22c575, controller sha 71a0d045090eed040b828f98a1c298cecc9e298fba477b9e172ae7a6c48eee4f)

V2 qualified frontier (deterministically re-verified at 8000 Hz, same Plant):

- supported_start 0.00025 PASS
- countermovement_onset 0.16288 PASS
- valid_countermovement 0.38075 depth 0.733 PASS
- upward_reversal 0.53175 PASS
- vertical_propulsion 0.53175 PASS
- bilateral_takeoff 0.76013 vz 0.717 PASS (≥0.60)
- genuine_flight 0.76013 PASS
- apex 0.83854 PASS
- descending_landing 0.93250 PASS
- stable_recovery OPEN (INCOMPLETE_HORIZON, peak landing 12760 N ~13.7 BW, root at 0.20 limit, deep collapse)

Qualification:
- V2_MJCF_LOAD PASS (nq10 nv10 nu7 nbody10)
- V2_MASS_CHECK PASS (95.0 kg)
- V2_ACTUATOR_CHECK PASS (250/300/200 symmetric forcerange)
- V2_FORCEPLATE_CHECK PASS (whole 940 bilateral 470 CoP valid)
- V2_STANDING_2S PASS (max drift 0.000102 <0.005)
- V2_EVENT_TESTS PASS (9/9)
- V2_BASELINE_REPLAY PASS (deterministic)
- CODE_REVIEW PASS (no HIGH/CRITICAL)
- BUG_HUNT PASS
- PONYTAIL_REVIEW PASS

Known baseline limitations preserved for V2.1 (not blockers):
- stable recovery not achieved
- artificial lower root-z limit may participate in failed landing
- landing impact ~12.8kN / ~13.7 BW
- landing collapses into deep state not captured squat
- event/telemetry ordering requires V2.1 repair
- no final MP4 yet

Newly established invariant:
- V2 Plant V2 is frozen at commit a04b573 and remains the only authority for landing work. V1 historical evidence R025-R052 preserved under LCMJ-*V1* namespaces; V2 candidates V2-R001..R005 preserved under LCMJ-V2-PLANT-CONTROLLER-CODESIGN-20260828.

Explicit next authorized unit:
RES-5 — V2.1 REMOVE ARTIFICIAL ROOT VERTICAL CATCH AND ESTABLISH HONEST FALL MECHANICS

Do not start RES-5 in this run. Do not change landing mechanics, contact parameters, or controller parameters in this baseline.

