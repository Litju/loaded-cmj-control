# PROJECT SCIENTIFIC CONSTITUTION — Loaded CMJ MuJoCo

**Project:** Loaded CMJ MuJoCo  
**Parent Program:** Loaded CMJ — MuJoCo V2.1 Ship  
**Constitution Version:** 1.0.0  
**Date:** 2026-09-03  
**Entry Head (at rebase):** `2a5967d359f34562a9e356c3b138062cffdf4d51`  
**Entry Tree:** `b7bec500e9060d1b59e243ca0a929cffe47b162a`  
**Status:** CANONICAL — governs all future evidence, claims, and Linear work

---

## 1. Purpose and Primacy

This Constitution is the single root of scientific authority for the Loaded CMJ MuJoCo repository. Every model, experiment, qualification claim, Linear issue, and evidence bundle derives its legitimacy from this document and the authorities it references. Where any other document — README, ADR, receipt, Linear label, or inline comment — conflicts with this Constitution, this Constitution prevails until formally amended.

The project objective is **not** to produce another controller result. The objective is to establish a trustworthy, independently inspectable, reproducible, and adjudicable record for a deterministic MuJoCo loaded countermovement-jump simulation and its controller qualification under a frozen computational model.

---

## 2. Systems Under Study — Terminological Firewall

### 2.1 Real-World System (RWS)

*Human loaded countermovement jump:* a 75 kg athlete with ~20 kg external load (barbell/high-bar), performing a sagittal-dominant CMJ on a force-plate instrumented floor, with neuromuscular, tendinous, and whole-body dynamics, variable anthropometry, fatigue, and coaching.

*The RWS is NOT modeled as validated human biomechanics in this project.* No claim about human predictive validity, injury, or optimal technique is justified by the current computational system alone (see §4).

### 2.2 Computational System (CS)

*Reduced sagittal-dominant MuJoCo rigid-body simulation* with:

- 10 bodies (pelvis, torso+arms, 20 kg cylindrical load, 2× thigh/shank/foot), 10 joints (3 honestly floating sagittal root + 7 actuated sagittal hinges), 16 geoms (8 contact + 6 fall-shell + floor + shells), 7 physical torque actuators, MuJoCo 3.8.0, `implicitfast` + `Newton`, `dt=0.000125 s`, 40 substeps per 5 ms control, gravity 9.81 m/s², 8.0 s horizon.
- Transparent bounded torque: `tau = limit * u`, `u ∈ [-1,1]`, per-joint limits 250/250/300/200 Nm, no hidden activation, no power clipping, no `qfrc_applied`.
- Bilateral 6-axis virtual force plates (left/right `foot_box` ↔ floor, condim 3, solref `0.016 1` calibrated, solimp `0.99 0.99 ...`, friction `0.9 0.005 0.0001`), separate fall-shell contacts (contype 4, not plantar), explicit COP validity, whole-wrench = sum of all contacts.
- Causal rollout: `policy → PolicyWorker → 15-D validation → DriveState → Plant/MuJoCo → BiomechanicalSample → V2EventDetector → RolloutResult`; policy receives only public observations (16 fields), never MuJoCo handles, event state, or termination.

### 2.3 Claim-Ceiling Consequence

All qualified achievements are **CS-anchored**. They demonstrate that a specific controller, starting from a specific initial integration state, drives the CS through a mechanically specified event sequence under the frozen Plant/solver/measurement/event contracts. They do not, without independent experimental referent data, validate that the RWS behaves identically.

---

## 3. Scientific Authority Hierarchy

```
Constitution (this document)
  └─ Intended Use & Claim Boundary
  └─ VVUQ & Qualification Taxonomy
  └─ Authority Ledger (versioned plant/mechanics/solver/actuator/measurement/event/controller/experiment/qualification/evidence/rendering)
  └─ Experiment Protocol + Registry
  └─ Evidence Contract (bundle schema, manifest binding)
  └─ Claim-Evidence Matrix + Forensic Rebase Report
  └─ Project Rebase Checkpoint
```

No achievement is SEALED unless:

1. acceptance criteria were **predeclared** in the Experiment Registry;
2. the run was **reproduced** from the recorded initial integration state in a fresh process;
3. an **evidence bundle** exists with complete traces, manifest, and checksums;
4. the bundle was **delivered** (path/hash/bytes manifest shown, `stat` + `sha256sum` + inventory);
5. the bundle was **audited** (claims match artifacts, negative controls pass where applicable);
6. the commit contains/references the **evidence manifest hash**, and the manifest contains `COMMIT_SHA` + `COMMIT_TREE`.

Loose "PASS because evidence exists somewhere locally" is explicitly forbidden.

---

## 4. Governing Principles

### 4.1 Reproducibility First

One qualified achievement = one code commit = one immutable evidence manifest = one evidence-archive hash (bidirectional binding). Giant raw traces remain in the evidence repository; the source repo stores only the manifest hash reference. Raw traces are never silently regenerated and labeled original.

### 4.2 Evidence Before Synthesis

No receipt is evidence until its referenced artifacts are physically located and `sha256sum`-verified. Retrospective threshold invention, changing objectives after observing candidates, and undeclared random search are prohibited. Every experiment declares hypothesis, frozen/varied variables, search method, candidate order/budget, metrics, hard gates, stopping rule, and diagnostic-vs-qualification status before execution.

### 4.3 Terminological Rigor

`Verification`, `validation`, `qualification`, `reproducibility`, `uncertainty`, and `review` are distinct (see VVUQ taxonomy). Internal tests or scorer closure are **not** "validation". Model validation requires an independent experimental/referent basis.

### 4.4 No Controller Tuning Inside This Mission

RES-10 controller work is PAUSED until this scientific rebase completes, its evidence pipeline is proven by fresh-process replay, and its forensic audit is sealed. The harmless replay used to prove the pipeline must not alter controller authority.

### 4.5 Single-Commit Foundation

This rebase itself is one independently qualified project-foundation achievement and must appear as exactly one commit:

```
Project: establish scientific evidence authority
```

only after: forensic audit complete → evidence pipeline tested → evidence package delivered → Linear synchronized → reviews PASS → worktree reconciled → reproduction post-commit.

---

## 5. Roles and Boundaries

- **Plant / Mechanics / Solver authorities** own mass, inertia, body topology, MJCF XML, integrator, solver, timestep, substeps, tolerances — never the controller.
- **Actuator contract** owns `tau = limit * u` and fail-closed validation — never hidden state.
- **Measurement contract** owns force-plate partition, wrench reconstruction, COP, transport, whole-wrench sums — never policy observations.
- **Event/Scorer contract** owns the 12-gate monotone DAG (`supported_start` → `stable_recovery`), dwell, hysteresis, fall semantics — never an objective score.
- **Controller** owns the public 7-D action and its gains/modes — never the Plant or solver.
- **Experiment / Qualification** owns predeclared gates and budgets — never retrospective scoring.
- **Evidence pipeline** owns deterministic recording, manifest, checksums, `reproduce.sh`, and the final mechanically generated receipt.
- **Visualization** owns scored-replay identity, state-to-frame mapping, camera, and deterministic encoding — never physics.

Each authority carries `version`, `source_paths`, `git commit/tree`, `content SHA256`, `dependencies`, `supersedes/superseded-by`, and `known caveats`.

---

## 6. Amendment

Amendment requires a new version of this Constitution, a recorded supersession entry in the Authority Ledger, and a sealed evidence bundle that demonstrates the amendment's scope does not silently rewrite history. History is corrected by additive forensic records, not by deletion.

---

## 7. Provenance

- MuJoCo 3.8.0 (pinned in `pyproject.toml` / `uv.lock` `a80b951e5898e0a7f4984cf5a9a6fbc7d50a4ce9633d1be46bb5433d1be46bb5434dedb6c5a4`)
- Python 3.13.13 (rebase host 3.14.4, qualified runtime 3.13.13), NumPy 2.5.1, x86_64 Linux 6.6 WSL2
- Model `loaded-cmj-20kg-athlete-v2`, scenario `loaded_jump_fixed_20kg_v2`
- Evidence repository `/home/litju/Projects/loaded-cmj-control-evidence` (non-Git, content-addressed by bundle hash)
- Linear project "Loaded CMJ — MuJoCo V2.1 Ship" (rebased per Checkpoint; see `PROJECT_REBASE_CHECKPOINT.md`)
