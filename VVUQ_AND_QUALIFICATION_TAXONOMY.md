# VVUQ and Qualification Taxonomy

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Status:** CANONICAL

This document fixes terminology. Misuse is a scientific error, not a style issue.

---

## 1. Definitions (normative)

### CODE_VERIFICATION
*Are we solving the equations correctly as coded?*
- Demonstrates that software implements the intended math without coding errors.
- Examples: unit tests that compare Plant mass/inertia to spec, actuator `tau = limit * u` exactly, force-plate `whole = left+right`, event logic matches documented dwell/hysteresis, cross-check `tau_forward == mj_inverse` within tolerance, no `qfrc_applied` hidden forces, fall shells not counted as plantar.
- **Not** validation. **Not** scorer closure.

### SOLUTION_VERIFICATION
*Is the numerical solution accurate enough for the reported result?*
- Quantifies discretization/solver error for the specific trace.
- Examples: `dt=0.000125` sensitivity, `tolerance 1e-10` solver convergence, determinism (fresh-process bit-identical `qpos/qvel/Fz` diff <1e-12), residual bounds (`qfrc_inverse` root residual, moment identity), penetration/chatter metrics, impulse consistency.
- A result cannot be "validated" if it is not solution-verified.

### MODEL_VALIDATION
*Does the computational system represent the real-world system for the intended use within tolerance?*
- **Requires an independent experimental/referent basis** (e.g., synchronized force-plate + motion-capture from real CMJs with documented load, placement, calibration, and sync).
- Must predeclare: referent dataset provenance, validation metric, acceptance tolerance, and statistical summary.
- **None exists in this program to date.** The CS is therefore **not validated** as a human surrogate. Claiming validation from internal tests orevent closure is terminologically invalid.

### CONTROL_QUALIFICATION
*Does a specific controller, under the frozen CS, satisfy the predeclared event/metric gates?*
- This is the program's current central activity.
- Example: 12-event DAG `OBJECTIVE_COMPLETE`, no `PHYSICAL_FALL`, landing peak `≤8 BW`, penetration `≤0.010 m`, no reflight/chatter, bilateral, margin `>0`, no artificial root support, no prohibited contact. Gates must be declared before the run; thresholds are not invented post-hoc.

### REPRODUCIBILITY
*Can the exact result be regenerated from the recorded initial state?*
- Fresh-process replay from saved integration state (`qpos,qvel,qacc,ctrl,time` + DriveState + `previous_action`), independent metric recomputation, identical `trace digest` / `RolloutResult`.

### UNCERTAINTY_CHARACTERIZATION
*How sensitive is the result to stated variations?*
- Bounded reporting of variation due to declared parameter perturbations, initial-state noise, or contact stochasticity — not an exhaustive UQ study. Must state method, scope, and limits.

### SCIENTIFIC_REVIEW
*Independent inspection of methods, evidence, and claim-evidence alignment.*
- Code review, bug hunt, ponytail review, or domain review — with stated reviewer, scope, and findings. Reviews are evidence, not proof.

---

## 2. Hard Rules

1. **Do not call internal tests or scorer closure "validation".** Labels `PASS` in `pytest` or `OBJECTIVE_COMPLETE` are qualification, not validation.
2. **Validation needs a referent.** Without one, the correct statement is "CS qualification under the frozen model; human predictive validity not established."
3. **Qualification needs predeclared gates.** Retrospective threshold invention invalidates the achievement (requires a new experiment definition).
4. **Reproducibility needs a delivered bundle.** A prose receipt is not the bundle.
5. **Solution verification before qualification.** An unreliable numerical result cannot be qualified.

---

## 3. Audit of Existing Documentation / Linear Labels

Audited scope: `docs/*.md`, `README.md`, `RES*_FINAL_RECEIPT.md`, staged commit messages, and `tests/test_v2_1_*.py` docstrings against the definitions above.

### 3.1 Corrections (recorded, not silently rewritten)

| Location | Original term | Issue | Correction |
|---|---|---|---|
| `docs/architecture.md` (pre-rebase) | Implicit "validated plant" | No referent | **Rephrase to** "Plant code-verified and measurement-closed; not human-validated" |
| `RES10_R3` docs `08_STATIC_NODE_QUALIFICATION` | "all 17 nodes PASS with tolerance 200" when 3 nodes 220–357 | Hard gate vs diagnostic conflation | **Normalize:** static `mj_inverse` root `>200` is **diagnostic only**; acceptance is forward-hold (0.5 s bilateral, margin>0). Tolerance 200 applies to endpoints only, or expand to 600 with explicit predeclare. |
| `RES10_R3` docs `10_DYNAMIC_INVERSE` | "root residual remains <5" alongside `67925` | False feasibility claim | **Strike** `<5` sentence; state: dynamic quintic reference is kinematically feasible but **dynamically infeasible** (residual ~1e3–6e4); controller uses **static** `tau_ff` as approximate projection, validated only by successful closed-loop forward rollout. |
| `RES10_R3` docs `05/06` + `RES-31` pre-fix | "foot x,z,pitch equal capture pose" | Δz 0.023 m, Δpitch 0.157 rad exceeds 1e-4/1e-3 | **Correct to** "planted with controlled roll; final node transitions to flat foot" — contact remains bilateral/plantar, no teleport/non-plantar (legitimate), but fixed-pose semantics are false. |
| `RES10_R3` `12_DURATION_LEDGER` | `E12 5.12/5.62` for T 4.0 | Stale: true E12 with `E11 2.935+T4.0+0.5`=7.435 | **Correct to** `6.935/7.435` (horizon 8.0) |
| `RES10_R3` `09/14` | collapsed `PATH_MIN 0.081` | Confuses static path min `0.10575` vs dynamic `0.081` | **Separate** `STATIC_PATH_MIN=0.10575` and `DYNAMIC_MIN=0.081` |
| Linear labels (historical) | `PASS`/`Done` for `RES10_R3`, `RES10_R4` | Fabricated 12/12 traces, `BLOCKED_ROOT_DAMPING` ignored | **Reclassify** per forensic matrix: `RES10_R3` → `CONTRADICTED`+`BLOCKED`, `RES10_R4` → `CONTRADICTED` (fabricated trace, stale root damping) — superseded by `RES-31/42/43` mechanical corrections + `RES10_R5A2` honest launch. |
| Any "model validation" prose in landed-CMJ context | "validated model" | No referent | **Rephrase to** "qualified CS under frozen model" |

All history is preserved; corrections are additive via this taxonomy and the Forensic Rebase Report. No historical file is silently overwritten.

### 3.2 Currently Compliant

- `RES-5`, `RES-6`, `RES-7`, `RES-8`, `RES-16`, `RES-31`, `RES-42`, `RES-43`, `RES10_R5A2` correctly use `qualification`, `CODE_VERIFICATION`, `SOLUTION_VERIFICATION`, `REPRODUCIBILITY`, and review terms, and distinguish them from `MODEL_VALIDATION`.

---

## 4. Mapping to Project Gates

| Gate | Taxonomy | Evidence |
|---|---|---|
| `PROPOSED` | Experiment predeclares gates | `EXPERIMENT_REGISTRY.jsonl` entry |
| `RUN` | Solution verification + measurement closure | `physics_trace.*`, `control_trace.*`, residuals |
| `CANDIDATE` | Control qualification (candidate) | `events_online/offline.json`, `metrics.json` vs predeclared thresholds |
| `REPRODUCED` | Reproducibility | Fresh-process rerun, digest match, `reproduce.sh` |
| `EVIDENCE_DELIVERED` | Delivery contract | `EVIDENCE_BUNDLE_PATH/SHA256/BYTES/FILE_COUNT`, `stat`/`sha256sum`/inventory |
| `EVIDENCE_AUDITED` | Scientific review + matrix | `CLAIM_EVIDENCE_MATRIX.csv`, `FORENSIC_REBASE_REPORT.md`, review files |
| `SEALED` | Bidirectional binding | Commit ↔ manifest hash, all of the above |
| `BLOCKED` / `SUPERSEDED` | Failure / replacement | Forensic status, `supersedes` in ledger |

A scientific achievement can be `SEALED` only when every row in this table is satisfied.

---

## 5. Terminological Checklist for Future Work

Before labeling anything `PASS` or `validated`, confirm:

- [ ] Which taxonomy term actually applies? (`CODE_VERIFICATION` / `SOLUTION_VERIFICATION` / `CONTROL_QUALIFICATION` / `REPRODUCIBILITY` / `VALIDATION`)
- [ ] If `MODEL_VALIDATION`, where is the independent referent dataset, its provenance, and the predeclared tolerance?
- [ ] If `CONTROL_QUALIFICATION`, where is the predeclared gate and the independent metric recomputation?
- [ ] If `PASS`, was the bundle delivered and audited — not merely existing locally?
