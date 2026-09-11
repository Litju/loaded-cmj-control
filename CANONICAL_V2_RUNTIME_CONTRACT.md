# Canonical V2.1 Runtime Contract (RES-78, frozen)

MISSION=RES12A_ESTABLISH_CANONICAL_V2_RUNTIME_001
EXPERIMENT_ID=EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001
CANDIDATE_ID=V2.1-R001

Entry authority (RES-78 requires exactly):
EXPECTED_ENTRY_HEAD=9ca6a8b6b01a600a2b8c983d674ddea63b2e53b2
EXPECTED_ENTRY_TREE=41e4dc81fa28ef511644e20ed797f63c42977e51
ENTRY_ACHIEVEMENT=9ca6a8b6b01a600a2b8c983d674ddea63b2e53b2 / V2.1: qualify deterministic 12/12 episode offline

Prerequisites Done: RES-10, RES-76, RES-11. RES-12 blocked until this contract seals.

This is RELEASE ENGINEERING ONLY. No controller research, tuning, search, or physics change.

## 1. Candidate identity

No prior V2.1 candidate authority exists in committed source/history (searched
`V2.1-R*`, `CANDIDATE_ID`, `EXPERIMENT_REGISTRY.jsonl`; only historical V1 C00/C01
and V2-R001..R005 codesign drafts, plus a prohibitive "Do NOT run V2.1-R001" note).
Therefore the first explicit V2.1 candidate authority is established as:

CANDIDATE_ID=V2.1-R001

No candidate family is created. No search registry. One frozen accepted identity.

## 2. Production authorities (frozen, unchanged)

- Plant entrypoint: `loaded_cmj.v2.plant.V2Plant`
  XML: `src/loaded_cmj/v2/assets/v2_plant.xml`
  SHA256=5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191
  NQ=10 NV=10 NU=7 ACTION_DIM=7 mass=95kg (75+20)
  physics_dt=0.000125 nominal control_dt=0.005 (40 substeps nominal)
  solver=Newton integrator=implicitfast iterations=100 ls=50 tol=1e-10
  solref=(0.016,1.0) solimp=(0.99,0.99,0.001,0.5,2.0) friction=(0.9,0.005,0.0001)
  root: unlimited, zero damping/stiffness/armature, zero passive (RES-42)
  actuators: tau=limit*u u in [-1,1] limits [250,250,250,300,300,200,200]
  measurements: RES-54 jacSubtreeCom pelvis root, RES-55 J_point, RES-57 support
  support semantics: RES-57 bilateral continuity (10N, 40-step dwell)
  C01 landing policy, RES-58 terminal law (FZ_RAW=BW-m*vz/DT clamp [0.6,1.5]BW),
  RES-73 T_BAL=0.27, RES-74 T_RISE=6.375 manifold13, RES43 q0 Kp400 Kd10 ff0
  phase guards, gains, recovery path, scorer predicates/dwells, action-history
  semantics: all frozen per RES-78. No mutation.

- Production full-controller composition entrypoint (committed library):
  `loaded_cmj.v2.res72_integration.Res72Policy` (PRELANDING incl C01/RES58 TERMINAL)
  `loaded_cmj.v2.balance_capture.BalanceController` (BALANCE exact RES-73)
  `loaded_cmj.v2.stable_recovery.StableRecoveryController` (RECOVERY exact RES-74 RISE/SETTLE/HANDOFF incl RES43)
  Composition contract: `src/loaded_cmj/v2/full_closure.py` (predicates/dwells, no detector import)
  POLICY_IDENTITY=PRELANDING=Res72Policy exact; E10->BALANCE |vz|<0.05 bilateral>10 160 samples truncate;
  BALANCE T_BAL=0.27; BALANCE->RECOVERY RR thresholds+dwell 0.10s truncate; RECOVERY T_RISE=6.375 manifold13 HANDOFF 0.05s; HANDOFF RES43 q0 Kp400 Kd10 ff0
  CONTROLLER_COMPOSITION_SHA256 (concatenated res72+balance+stable+terminal+full_closure in order)=6f56ffa180b12c128d67ea13fd9c08413554a9d964d175615f16a8e572185d12
  Individual:
  res72_integration.py=a7d09f80f1c4f7e071dabe38ec0b2b8610210d4f51a8b28425a6427904d0e843
  balance_capture.py=37b2e703ef0c77b9926b0260121a995b3b4ea5d7adee8d60e5b8561aec225979
  stable_recovery.py=fbee58f5dde40cc98c9d309a1f061ce20fe73bee7e9a3df864676be485229711
  terminal_capture.py=b189a9d68364ed47e597c5915abf6dc3e6be5c064796efc81e90497a0d35c697
  full_closure.py=aa0efc9cc632ac8c8e1f78230f10893f6be5cf922e032e77eb37233d1eeacba4

- Scorer/event authority (observational only):
  `loaded_cmj.v2.events.V2EventDetector` (12-gate monotone DAG, frozen dwells)
  SCORER_SHA256 (events.py)=286ef328e4b334a15775df01ba6aad971cf8f808ddbcb028fcda4032164f2deb
  Never used for control. No scorer->controller circularity.

- Synchronized observation authority:
  `loaded_cmj.v2.measurement.SynchronizedPhysicsSample.from_live_state`
  CONTROL_SAMPLE_CONVENTION=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL
  OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_CONTROL
  One state -> one observation (SHA identity), shadow mj_forward only, live never forwarded for reporting.

- Support adjudication: `loaded_cmj.v2.support_continuity.adjudicate_trajectory` (RES-57)

## 3. Canonical runtime (new tracked production entrypoint)

Module: `src/loaded_cmj/v2/canonical_runtime.py`
Must directly use V2Plant + exact committed composition above + V2EventDetector observationally.
Must NOT import or wrap: RES-76 research runner (`full_qualify*.py`, `run_full_qualification.py`),
RES-11 independent verifier (`tools/res11_independent_verify.py`), V1 Gen1 engine/pfip path.

Canonical command (repository-equivalent, exact final):
python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001

Alternate explicit form (same):
.venv/bin/python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001

Only `--candidate V2.1-R001` is accepted. Any other candidate fails closed.

## 4. Reset / transport / stepping (frozen)

- Canonical reset only: V2Plant.reset (V2_RESET_QPOS root_tz 0.90, qvel 0, ctrl 0, qfrc 0) + mj_forward, held=zeros(7), detector reset, Res72Policy reset(0.0), Balance/Recovery lazily constructed at handoffs, probes preallocated (17 terminal +1 vprobe, 15 balance +1 vprobe, 15 recovery +1 vprobe), C52.bind_plant, gap_hh from model, K52/RR/manifold/hspec/T_RISE from sealed evidence (see spec).
- Sample-before-update at each control boundary t_k: sync0=from_live_state(live,shadow), check_time_identity, obs=controller_observation(step, reset, held), assert sync0.ctrl==held, u=controller(obs), apply_action(live,u), held=u.
- Control/action transport: plant.apply_action only (u in [-1,1] -> tau=limit*u -> data.ctrl). No direct .ctrl writes, no qpos/qvel snap, no mj_setState on live after reset (probes/shadow only).
- Hybrid truncated intervals: nominal 40 substeps per control; truncated exactly where controller-local dwells complete mid-interval (E10 160-sample and RR 0.10s) plus final posthold termination mid-interval. Expected schedule: 3045x40, 2x28, 1x33 (total 3048 controls, 121889 physics). Same controller mode/submode transitions as RES-76/11. Never restore intermediate states.
- Scorer invocation: after each mj_step, sync1=from_live_state, sm=foot_contact_summary(shadow), H=centroidal_H_world, trunk rate via mj_objectVelocity torso, soft_contact_state for support, watcher.update(event_sample with time_s=sim_t) observationally. finalize at end. No detector read for control; separate controller-local dwell counters (e10_run 160, rr 0.10s, E12 4000).
- Horizon: HORIZON_S=20.0 (4000 controls max), early termination PASS_E12_POST_HOLD when E12 confirmed +0.30s guard_ok hold with post_bad==0. T_END authority 15.236125000027243, N_CTRL 3048, N_PHYS 121889.
- Termination: OBJECTIVE_COMPLETE requires 12/12 in order + no fall + offline identity; else INCOMPLETE/FAIL. Same objective as RES-76/11.
- Post-E12 hold: POST_HOLD_S=0.30 beyond E12 conf, guard_ok=true-standing+low thresholds, post_bad must be 0.

## 5. Environment / packages (pinned)

mujoco==3.8.0 numpy==2.5.1 scipy==1.18.1 python 3.13.13 (venv) x86_64 Linux
uv.lock SHA256=a80b951e5898e0a7f4984cf5a9a6fbc7d50a4ce9633d1be46bb5434dedb6c5a4
Model XML SHA above. No other solver/physics change.

## 6. Output schema (deterministic machine-readable)

`run_canonical_episode()` returns dict and `--out` JSON contains at minimum:
MISSION, EXPERIMENT_ID, CANDIDATE_ID, CANONICAL_COMMAND, CANONICAL_RUNTIME_MODULE,
ENTRY_HEAD/TREE, PLANT_SHA256, CONTROLLER_COMPOSITION_SHA256, SCORER_SHA256,
ACTION_DIM/NQ/NV/NU, PHYSICS_DT, CONTROL_DT_SEMANTICS, HORIZON_S, POST_HOLD_S,
OUTCOME/TERMINATION, T_END/N_CTRL/N_PHYS, EVENTS (12 occurred/confirmed/sample indices),
CHECKPOINT_SHAS (7 required + full DET map), CHECKPOINT_TIMES, CTRL_SUBSTEPS histogram,
CTRL_MODES, PEAK_BW/MAXPEN/MAXUTIL/ROOT_ROWS/FINITE/FALL/PROHIB,
SUPPORT_POST_LANDING (loss 0, reflight [], chatter 0) + full/phase adjudications,
ONLINE_OFFLINE_IDENTITY, TRACE_SHA256, WALL_S.

Result JSON is deterministic given pinned env (TRACE_SHA256=4d047879... for closed-loop stream; note harness trace representation differs from RES-76 file hash but physical authority identical).

## 7. Evidence semantics

Precommit: this contract + candidate spec hashed before qualification (see spec).
Execution: one canonical episode from canonical reset, compare 12/12 +7/7 + hard gates.
Bundle: EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001 under evidence root, Evidence Contract v2
(precommit checksums + postcommit sidecar, reproduce.sh, manifest, FINAL_RECEIPT).
Results/evidence go to evidence bundle, never mutated into V1 semantics.
RES-12 consumes this contract/command/candidate without redefining authority.

## 8. PASS criteria (exact equivalence gate)

- 12/12 exact event identity (occurred/confirmed as below)
- 7/7 checkpoint identity (S_APEX/S_E10/S_E11/S_RR/S_RR_CONFIRMED/S_STAND_HANDOFF/S_E12)
- exact 28/33/40 schedule (3045x40,2x28,1x33)
- same mode/submode transitions, termination OBJECTIVE_COMPLETE/PASS_E12_POST_HOLD
- same landing/recovery hard gates, post-landing loss 0 reflight [] chatter 0
- fresh-process reproduction identical

Accepted event authority:
E1  0.000125 / 0.10000000000000007
E2  0.1612499999999961 / 0.1911249999999928
E3  0.380124999999972 / 0.380124999999972
E4  0.4874999999999602 / 0.4973749999999591
E5  0.4973749999999591 / 0.5472499999999746
E6  0.6481250000000083 / 0.6580000000000116
E7  0.6580000000000116 / 0.7378750000000383
E8  0.7719764018454335 / same
E9  0.875000000000084 / 0.8848750000000873
E10 0.973625000000117 / 0.9935000000001236
E11 0.9935000000001236 / 1.1433750000000462
E12 14.436250000023424 / 14.936125000025811

Accepted checkpoints:
S_APEX=97ed110f5ad2bb5cd8e0e91a91a503e9326814a2bfcc68ce73237900be8351c2
S_E10=7931fd8b2715362cfd766a75f46553681ce872e541daaf84a95b5dfd50da8ec0
S_E11=31c7f7d11ead4277484fa1d1b93245f5dd06f256f003365edc99b328daf7039d
S_RR=846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710
S_RR_CONFIRMED=02559b91904d7103177bae29761f65b48d7b4e9db69cd3e6c886805422766a06
S_STAND_HANDOFF=2141e801ab403b32d5fc83f94a046997041815ef70c44919796534907a329679
S_E12=363a5d89298cab15c194229a64838e561126e72285a41b48665d18bdbbe88146

If any mismatch: fix only runtime interface/transport. DO NOT MODIFY CONTROLLER SCIENCE. No alternate architectures.
