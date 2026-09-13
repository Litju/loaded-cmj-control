# SCIENTIFIC_CLAIM_CEILING — Successor Loaded-CMJ Contract

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **FROZEN**
SUPERSEDES for successor work: any broader reading of the historical V2/V2.1 claim
language. Adds no new permission to the existing project claim boundary; narrows the
successor claim to what the successor contract can actually support.

V&V basis: ASME V&V 40-2018 (credibility commensurate with context of use and
decision consequence), FDA 2023 CM&S credibility guidance, and the project
`VVUQ_AND_QUALIFICATION_TAXONOMY.md`. `verification != validation`;
`determinism != biomechanical validity`; `event completion != task success`;
`visual plausibility != experimental validation`.

---

## 1. The single allowed PASS claim

A successor candidate that satisfies the complete successor contract, under the
qualified model, runtime, and environment, may state:

> **"A deterministic MuJoCo rigid-body nominal 20 kg loaded countermovement-jump
> simulation satisfies the declared successor task, landing, balance, and recovery
> contract under the qualified model and environment."**

This claim is scoped to the computational system (CS). It says what the simulation
does under its declared gates.

## 2. What a PASS additionally supports (with evidence)

| Statement | Supported when |
|---|---|
| The canonical runtime executed to completion with valid model state (L0/L1) | always, for a qualifying PASS |
| The event sequence E1–E12 occurred with the declared physical semantics (L2) | always |
| The declared performance floor was met (L3) | after OD-01 is resolved |
| The landing was a whole-body admissible capture, not a vertical arrest (L4) | always |
| The balance capture and recovery satisfied the declared physical bounds (L5) | after OD-04/OD-05 and RES-87 calibration |
| Negative controls NC-01..NC-10 fail as required | after RES-89/RES-91 implement them |
| The result is reproducible from the recorded state | after RES-88 provenance work |

## 3. Explicit non-claims (hard ceiling)

A successor PASS does **not** establish, and must never be used to assert:

1. human predictive validity of the model;
2. subject-specific biomechanics;
3. population norms or percentiles;
4. muscle force, activation, or fatigue predictions (no muscle model exists);
5. neural control, motor learning, or reflex behaviour;
6. optimal human technique;
7. injury-risk prediction (ACL, ankle, spine, or any other);
8. clinical diagnosis, screening, return-to-sport, or load prescription;
9. elite-performance benchmarking;
10. experimental predictive validity unless separately validated against an
    independent referent with predeclared metric and tolerance;
11. medio-lateral or multi-planar validity beyond the declared sagittal-reduced
    representation;
12. validity under COU changes (different athlete, load, task, or representation)
    without a new COU.

## 4. Claims that require future validation evidence

| Claim | Required evidence |
|---|---|
| "The simulation reproduces human loaded-CMJ COM/GRF/kinematics within tolerance X" | independent referent dataset (force plate + motion capture, documented load, calibration, sync), predeclared metric and tolerance, statistical summary |
| "The landing strategy is representative of human loaded-CMJ landing" | referent-based validation of the compared quantities, not internal gates |
| "The recovery resembles human post-landing stabilization" | referent-based TTS/DPSI-equivalent comparison under a declared mapping |
| "The performance floor corresponds to a population stratum" | population study with a declared sampling frame; the floor is currently a task plausibility bound only |

## 5. Mapping from contract layers to claim classes

| Layer | Claim class | Notes |
|---|---|---|
| L0 EXECUTION_VALIDITY | code/solution verification | not validation |
| L1 MODEL_STATE_VALIDITY | code/physics verification | not validation |
| L2 EVENT_SEQUENCE_VALIDITY | event-contract conformance | not validation; never sufficient alone |
| L3 TASK_PERFORMANCE_VALIDITY | task-contract conformance | the floor is a task definition |
| L4 LANDING_CAPTURE_VALIDITY | task-contract conformance (physical + human-plausibility gates) | not injury prediction |
| L5 RECOVERY_VALIDITY | task-contract conformance | not clinical stability |
| L6 CANDIDATE_CREDIBILITY | provenance/determinism/test/review | owned by later RES |

## 6. Prohibited language

- "validated model" without a referent.
- "the model predicts ..." for any human outcome.
- "safe landing" / "injury-free landing".
- "optimal technique".
- "biomechanically valid" as a synonym for "satisfied the contract".
- "12/12" (or any event count) presented as success.

## 7. Amendment

Raising this ceiling requires: (a) a named independent referent dataset with
provenance; (b) a predeclared validation metric and acceptance tolerance; (c) an
independent reconciliation; (d) a sealed validation evidence bundle; and (e) an owner
decision recorded as a new version of this document. Prose alone never raises the
ceiling.
