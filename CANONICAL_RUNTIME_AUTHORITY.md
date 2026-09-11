# CANONICAL RUNTIME AUTHORITY — RES-12 Phase 0 Audit

MISSION=RES12_CANONICAL_12_OF_12_QUALIFICATION_001
EXPERIMENT_ID=EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001
CANDIDATE_ID=V2.1-R001
PHASE=0_AUDIT_ONLY
DATE_UTC=2026-09-11

## 1. Entry authority (sealed RES-78, explicitly enforced)

Repository: `/home/litju/Projects/loaded-cmj-control`
Branch: `main`

ENTRY_HEAD=20e7488ae727c02b581ec4c8e668476ec4432fa5
ENTRY_TREE=a8da465a7595cada5940c173b5c796e23e0e4400
ENTRY_ACHIEVEMENT="V2.1: establish canonical qualification runtime"

Verification (executed, not described):
- `git rev-parse HEAD` == ENTRY_HEAD: PASS
- `git rev-parse HEAD^{tree}` == ENTRY_TREE: PASS
- `git status --porcelain` tracked dirt: NONE (only `??` untracked preserved dirt, unmodified/uncommitted): TRACKED_CLEAN=PASS
- Provenance via direct HEAD/tree comparison. `reproduce.sh --allow-new-head` NOT used as provenance proof: ENFORCED.

Preserved known untracked evidence/dirt remains untouched (not modified, deleted, absorbed, or committed).

## 2. Frozen identities (exact, recomputed)

CANDIDATE_SPEC_SHA256=b02c74b8af547692b6544790b93c87b16a86c0ac04c76abeac47aaf2b9972e0c
  source: CANONICAL_V2_CANDIDATE_SPEC.json (sha256sum recomputed): PASS
RUNTIME_CONTRACT_SHA256=cafb91fe838b68c8110157659d35f6fae561295e98409fc0b6bc94375d933c1e
  source: CANONICAL_V2_RUNTIME_CONTRACT.md (sha256sum recomputed): PASS
PLANT_SHA256=5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191
  source: src/loaded_cmj/v2/assets/v2_plant.xml (sha256sum recomputed): PASS
CONTROLLER_COMPOSITION_SHA256=6f56ffa180b12c128d67ea13fd9c08413554a9d964d175615f16a8e572185d12
  source: cat res72_integration.py+balance_capture.py+stable_recovery.py+terminal_capture.py+full_closure.py | sha256sum (in order): PASS
  individuals:
    res72_integration.py=a7d09f80f1c4f7e071dabe38ec0b2b8610210d4f51a8b28425a6427904d0e843
    balance_capture.py=37b2e703ef0c77b9926b0260121a995b3b4ea5d7adee8d60e5b8561aec225979
    stable_recovery.py=fbee58f5dde40cc98c9d309a1f061ce20fe73bee7e9a3df864676be485229711
    terminal_capture.py=b189a9d68364ed47e597c5915abf6dc3e6be5c064796efc81e90497a0d35c697
    full_closure.py=aa0efc9cc632ac8c8e1f78230f10893f6be5cf922e032e77eb37233d1eeacba4
SCORER_SHA256=286ef328e4b334a15775df01ba6aad971cf8f808ddbcb028fcda4032164f2deb
  source: src/loaded_cmj/v2/events.py (sha256sum recomputed): PASS

All five match RES-12 mission requirement exactly. Any difference would be FAIL_A_CANONICAL_RUNTIME_AUTHORITY with STOP, no tuning.

## 3. Canonical executable / command

CANONICAL_RUNTIME_MODULE=loaded_cmj.v2.canonical_runtime
CANONICAL_COMMAND=python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001
Module path: src/loaded_cmj/v2/canonical_runtime.py (636 lines, tracked at ENTRY_HEAD)
CLI: `--candidate V2.1-R001` only; any other candidate exits 2 (verified via code + test_03 pattern, no full sim).
`--help` works without simulation. Wrong-candidate fast-reject verified by inspection (no episode consumed).

## 4. Production controller entrypoint (frozen, unchanged)

- Plant: `loaded_cmj.v2.plant.V2Plant`, XML `src/loaded_cmj/v2/assets/v2_plant.xml`
  NQ=10 NV=10 NU=7 ACTION_DIM=7 mass=95kg (75+20)
- Composition (committed library, no research runner):
  `loaded_cmj.v2.res72_integration.Res72Policy` (PRELANDING incl C01/RES58 TERMINAL)
  `loaded_cmj.v2.balance_capture.BalanceController` (BALANCE exact RES-73, T_BAL=0.27)
  `loaded_cmj.v2.stable_recovery.StableRecoveryController` (RECOVERY exact RES-74 T_RISE=6.375 manifold13 incl RES43 HANDOFF q0 Kp400 Kd10 ff0)
  Composition contract: `src/loaded_cmj/v2/full_closure.py` (predicates/dwells, no detector import)
- Must NOT import/wrap (verified absent from executable body except docstring mention):
  `full_qualify*.py`, `run_full_qualification.py`, `tools/res11_independent_verify.py`, V1 Gen1 engine/pfip path.
  `res11_independent_verify`, `run_full_qualification`, `FULL_QUALIFICATION_RAW`, `run_gen1/Gen1/pfip`, `loaded_cmj.control`, `loaded_cmj.runtime` all absent: PASS.
- Controllers contain no V2EventDetector/event_records in executable body (docstring mentions only, stripped by tests): PASS.
  Runtime `watcher.update` present observationally; `event_records` count==7 (checkpoints/termination/finalize only, <=8 allowlist): PASS.
  No `u_cmd` computed from `event_records`: PASS (u from prelanding/balancer/recoverer only).

## 5. Environment / package pins

- python=3.13.13 (venv, `sys.version`): PASS (matches spec ENVIRONMENT.python)
- mujoco==3.8.0 (`mujoco.__version__`): PASS
- numpy==2.5.1: PASS
- scipy==1.18.1: PASS
- arch=x86_64 (`platform.machine()`), os=Linux: PASS
- uv.lock SHA256=a80b951e5898e0a7f4984cf5a9a6fbc7d50a4ce9633d1be46bb5434dedb6c5a4 (sha256sum recomputed): PASS
- pyproject pins mujoco==3.8.0, numpy>=2,<3: consistent.

## 6. Physics authority

- XML `<option timestep="0.000125" integrator="implicitfast" solver="Newton"`: PASS
- Live model query: opt.timestep=0.000125, opt.integrator==mjINT_IMPLICITFAST(3), opt.solver==mjSOL_NEWTON(2): PASS
- opt.iterations=100, opt.ls_iterations=50, opt.tolerance=1e-10: PASS
- solref=(0.016,1.0) feet/floor, solimp=(0.99,0.99,0.001,0.5,2.0), friction=(0.9,0.005,0.0001): PASS (grep)
- Root: unlimited (limited=false), zero damping/stiffness/armature, zero passive (RES-42): verified via XML grep + MAXROOTPASSIVE authority (0.0 in RES12A result).
- Actuators: tau=limit*u, u in [-1,1], limits [250,250,250,300,300,200,200] (XML forcerange + plant.apply_action enforcement): PASS
- Mass total 95.0 (model body_mass sum): PASS

## 7. Canonical reset / initial state

- `V2Plant.reset`: mj_resetData, qpos=V2_RESET_QPOS, qvel=0, ctrl=0, qfrc_applied=0, xfrc_applied=0, mj_forward: PASS (plant.py:124-131)
- V2_RESET_QPOS=[0.0,0.9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] (root_tz 0.90, hip/knee/ankle 0): PASS
- Runtime canonical reset: V2Plant()+C52.bind_plant, make_data, reset(live), mj_forward, shadow=create_measurement_data, held=zeros(7), detector reset, Res72Policy reset(0.0), Balance/Recovery lazily at handoffs, probes preallocated (17+1 terminal, 15+1 balance, 15+1 recovery), C52.bind_plant, gap_hh from model, K52/RR/manifold/hspec/T_RISE from sealed evidence: PASS (canonical_runtime.py:109-128 + _load_authorities)
- No other reset path accepted.

## 8. Sample-before-update semantics

- CONTROL_SAMPLE_CONVENTION=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL (measurement.py:42): PASS
- OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_CONTROL (measurement.py:43): PASS
- Runtime per control boundary: sync0=from_live_state(live,shadow), check_time_identity assert, obs=controller_observation(step,reset,held), assert sync0.ctrl==held, u=controller(obs), apply_action(live,u), held=u: PASS (lines 222-280)
- One state -> one observation (SHA identity), shadow mj_forward only, live never forwarded for reporting (SynchronizedPhysicsSample authority): PASS by code inspection.

## 9. Action / control transport + hybrid schedule

- Transport: plant.apply_action only (u in [-1,1] -> tau=limit*u -> data.ctrl). No direct `.ctrl` writes, no qpos/qvel snap, no mj_setState on live after reset (probes/shadow only): PASS
  - `mj_setState` absent from runtime body: PASS
  - `qpos[:] =`/`qvel[:] =` absent: PASS
  - `live.qpos`/`d.qpos` absent: PASS
  - `.ctrl` only in assert + apply_action internals: PASS
- Hybrid truncated intervals: nominal NSUB=40; truncate exactly where E10 160-sample and RR 0.10s dwells complete mid-interval plus final posthold termination mid-interval: PASS (truncate flag, E10_NEED=160, RR_HOLD_S=0.10, E12_NEED=4000)
- Expected schedule: 3045x40, 2x28, 1x33 (N_CTRL=3048, N_PHYS=121889): structure verified, values proven by RES12A/RES-76/11 authority; Phase A must reproduce exactly.
- Never restore intermediate states: PASS.
- Same mode/submode transitions as RES-76/11 (PRELANDING|PRL_*, BALANCE, RECOVERY|REC_*): verified by code + RES12A result ctrl_modes.

## 10. Observational scorer separation

- Scorer: `loaded_cmj.v2.events.V2EventDetector` (12-gate monotone DAG, frozen dwells), SCORER_SHA above: PASS
- Invocation: after each mj_step, sync1=from_live_state, sm=foot_contact_summary(shadow), H=centroidal_H_world, trunk rate via mj_objectVelocity torso, soft_contact_state, watcher.update(event_sample with time_s=sim_t) observationally, finalize at end: PASS
- No detector read for control (u never from event_records); separate controller-local dwell counters (e10_streak 160, rr 0.10s, E12 indep 4000): PASS
- `watcher._is_true_standing_neighborhood` used for env0/guard is pure physical predicate, identical to RES-76 run_full_qualification.py:134/329 and RES-11 verifier:210/387 usage (bool passed to recoverer.step). Not event memory; frozen composition semantics preserved: DOCUMENTED, NOT a circularity.
- Support adjudication: `loaded_cmj.v2.support_continuity.adjudicate_trajectory` (RES-57) post-hoc only: PASS

## 11. Horizon / termination / post-E12 hold

- HORIZON_S=20.0 (4000 controls max), CTRL_DT nominal 0.005: PASS
- Early termination PASS_E12_POST_HOLD when E12 confirmed +0.30s guard_ok hold with post_bad==0: PASS (POST_HOLD=0.30, post_t0/post_bad logic, terminal break)
- T_END authority 15.236125000027243, N_CTRL 3048, N_PHYS 121889 (RES-76/11/12A): horizon safely beyond E12 conf 14.936...+0.30: PASS
- OBJECTIVE_COMPLETE requires 12/12 in order + no fall + offline identity; else INCOMPLETE/FAIL: PASS (watcher.finalize + online/offline check)
- Post-E12 hold: POST_HOLD_S=0.30 beyond E12 conf, guard_ok=true-standing+low thresholds, post_bad must be 0: PASS
- Formal post-landing gate uses SUPPORT_POST_LANDING (loss 0, reflight [], chatter 0), NOT SUPPORT_FULL/SUPPORT_TD_E10: ENFORCED for Phase A.

## 12. Output schema / evidence destination

- `run_canonical_episode()` returns dict and `--out` JSON contains at minimum MISSION/EXPERIMENT_ID/CANDIDATE_ID/COMMAND/MODULE/ENTRY/PLANT/CONTROLLER/SCORER/ACTION_DIM/NQ/NV/NU/PHYSICS_DT/CONTROL_DT/HORIZON/POST_HOLD/OUTCOME/TERMINATION/T_END/N_CTRL/N_PHYS/EVENTS/CHECKPOINT_SHAS/TIMES/CTRL_SUBSTEPS/CTRL_MODES/PEAK_BW/MAXPEN/MAXUTIL/ROOT_ROWS/FINITE/FALL/PROHIB/SUPPORT_POST_LANDING+full adjudications/ONLINE_OFFLINE/TRACE_SHA/WALL_S: PASS by code inspection (result dict lines 540-590).
- Result deterministic given pinned env (TRACE_SHA256=4d047879... authority, WALL_S excluded from scientific identity): DOCUMENTED.
- Evidence destination (Phase C): Evidence Contract v2 bundle under evidence root for EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001 (non-Git authority is bundle hash; Git history remains source authority).

## 13. Phase 0 verdict

All Phase 0 gates PASS. No full canonical episode consumed in Phase 0 (only --help/wrong-candidate fast paths, imports, file reads, and lightweight model queries). Authority differs? NONE. Proceed to Phase A with exactly one fresh canonical episode. Failure taxonomy on any later mismatch: FAIL_A (authority), FAIL_B (interface/transport), FAIL_C (physics reproducibility), FAIL_D (scorer identity), FAIL_E (hard gate). No tuning/repair authorized.
