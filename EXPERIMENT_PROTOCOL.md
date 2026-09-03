# Experiment Protocol

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Status:** CANONICAL

---

## 1. Scope

Every experiment that can influence a qualification decision must be **predeclared** in `EXPERIMENT_REGISTRY.jsonl` before execution. Ad-hoc runs are allowed only as strictly **diagnostic** (not gating) and must be labeled as such.

---

## 2. Predeclaration Schema

Each entry (one JSON object per line) must specify:

```json
{
  "experiment_id": "EXP-20260903-STANDING-REPLAY-001",
  "hypothesis": "A deterministic standing replay from the recorded initial integration state reproduces bit-identically in a fresh process.",
  "authority_commit": "2a5967d359f34562a9e356c3b138062cffdf4d51",
  "authority_tree": "b7bec500e9060d1b59e243ca0a929cffe47b162a",
  "variables_allowed_to_change": ["initial_state_restored_from_npy"],
  "variables_frozen": ["Plant", "solver", "actuator_contract", "measurement_contract", "event_scorer_contract", "controller", "dt", "horizon"],
  "search_method": "deterministic_single_candidate",
  "candidate_order": ["standing_hold_Kp400_Kd10_T2.0"],
  "candidate_budget": 1,
  "metrics": ["trace_digest_match", "qpos_max_abs_error", "qvel_max_abs_error", "Fz_max_abs_error", "event_identity", "margin_min"],
  "hard_gates": ["trace_digest == original", "qpos_err < 1e-12", "events_online == events_offline"],
  "stopping_rule": "stop_after_budget_or_first_gate_failure",
  "is_diagnostic": false,
  "is_qualification": true,
  "predeclared_at": "2026-09-03T00:00:00Z"
}
```

### 2.1 Field Notes

- `hypothesis` — falsifiable, single sentence.
- `authority_commit` / `authority_tree` — frozen commit identity the experiment is valid against; experiments are invalid if the tree drifted.
- `variables_allowed_to_change` / `variables_frozen` — explicit partition; anything not listed as allowed is frozen.
- `search_method` — `grid` | `ladder` | `hill_climbing` | `SLSQP+hill_climbing` | `deterministic_single_candidate` | `manual_diagnostic` etc.; must name the algorithm.
- `candidate_order` — deterministic enumeration; no implicit random order.
- `candidate_budget` — integer upper bound; execution must not exceed it.
- `metrics` — names that will appear in `metrics.json` / `events_*.json`.
- `hard_gates` — predicates evaluated before any performance comparison; invention of new thresholds after observing candidates is prohibited.
- `stopping_rule` — `stop_after_budget_or_first_gate_pass` | `exhaust_budget` | etc.
- `is_diagnostic` vs `is_qualification` — exactly one `true`; diagnostic runs cannot seal achievements.

---

## 3. Hard Rules

1. **No undeclared random search.** Randomized methods must be seeded, with seed and generator stated, and still predeclare the budget.
2. **No retrospective threshold invention.** Gates are frozen before the first candidate executes. Adding a gate after seeing results requires opening a **new** experiment definition.
3. **No changing the objective after observing candidates without opening a new experiment definition.** The objective/metric is frozen.
4. **Diagnostic ≠ qualification.** A diagnostic run may inform the next predeclared experiment but never directly gate `SEALED`.
5. **Search budget is a bound, not a target.** Stopping early per `stopping_rule` is allowed; exceeding the budget is not.

---

## 4. Execution

- The recorder (`tools/evidence_recorder.py`) refuses to run if the matching `EXPERIMENT_REGISTRY.jsonl` entry is not found or if `authority_commit` does not match `git rev-parse HEAD`.
- At run completion, the manifest's `experiment_id` must equal the predeclared entry, and `experiments.jsonl` inside the bundle must contain the exact predeclared JSON plus the observed `candidate_traces[]` and `gate_results{}`.
- Any deviation (extra candidates, changed frozen variable, invented metric) marks the bundle `BLOCKED_EXPERIMENT_CONTRACT_VIOLATION` and prevents sealing.

---

## 5. Budgets Used by Current Authority (reference)

| Mission | Method | Budget |
|---|---|---|
| RES-6 solref ladder | offline `geom_solref` ladder `0.004→0.020` | 6 |
| RES-6 selection bound | canonical | — |
| RES-8 capture gains | `Kd=2ζ√(Kp·Ieff)` analytic + `500` max | 500 |
| RES-10 R2 recovery | deterministic ladder `T 1.0-2.5` | 4 + 9 manifold nodes |
| R5A2 launch residual | `SLSQP+hill_climbing` 16 params 4 knots | 2 solver runs + `16` params |
| R5A2 landing blocked | patch `0 UNKNOWN` classified | 0 additional |
| Head capture | standing `0` with `IMPACT 190/Kd20` | 7 vars local search |

All budgets were declared in their respective receipts and are grandfathered as predeclared for this rebase's forensic purpose.

---

## 6. Relation to Qualification State Machine

```
PROPOSED  — registry entry written, not yet run
RUN       — candidate(s) executed, traces recorded
CANDIDATE — metrics computed, gates evaluated
REPRODUCED — fresh-process replay digest matches
EVIDENCE_DELIVERED — bundle path/hash/bytes/inventory shown + attached
EVIDENCE_AUDITED — matrix + reviews PASS
SEALED    — commit ↔ manifest binding complete
BLOCKED / SUPERSEDED — explicit terminal/nonterminal
```

Can be `SEALED` only when every gate in this chain and in `VVUQ_AND_QUALIFICATION_TAXONOMY.md` is satisfied.

---

## 7. Amending the Protocol

Version the protocol, append a ledger entry, do not mutate past registry lines. Past experiments remain bound to the protocol version they were declared under.

