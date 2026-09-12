# PROVENANCE_REVIEW (RES-80, independent pass)

Scope: canonical runtime authority/identity, external evidence dependencies, packaging, absolute paths,
scorer/controller separation, result schema. Read-only. Wheel built to /tmp only.

## 1. Frozen candidate authority: verified and limited

Verified at baseline:
- HEAD `8f26736...` / tree `855fe3b...` match the mission freeze.
- `PLANT_SHA256 5f224414...` matches `v2_plant.xml`; `SCORER_SHA256 286ef328...` matches `events.py`;
  `CANDIDATE_SPEC_SHA256 b02c74b8...` matches the spec JSON; `RUNTIME_CONTRACT_SHA256 cafb91fe...`
  matches the contract; `CONTROLLER_COMPOSITION_SHA256 6f56ffa1...` recomputes exactly over the five
  declared files in order.
- R001 `TRACE_SHA256 4d047879...` is present and identical in the RES-12 result, the RES-79 extraction
  report, the RIS-12A sidecar and the RES-79 replay `ivec` history hash.

## 2. The freeze is partial (CRITICAL)

The composition hash covers `res72_integration.py`, `balance_capture.py`, `stable_recovery.py`,
`terminal_capture.py`, `full_closure.py`. But:

- `full_closure.py` is imported by no production module (tests only); the actual regime switching,
  E10/RR predicates, dwells and termination live inline in **unhashed** `canonical_runtime.py`.
- Executed-but-unhashed behavior files: `canonical_runtime.py`, `constants.py`, `plant.py`,
  `measurement.py`, `drive.py`, `support_continuity.py`, and `tools/res52/core52.py`,
  `tools/res52/soft_contact.py`, `tools/evid_trace_v2.py`.
- `run_canonical_episode()` performs zero hash verification; the only identity check is the `--candidate`
  string (canonical_runtime.py:623-625). Any tree with the five files intact can emit
  `CANDIDATE_ID=V2.1-R001`.
- Entry-head provenance is fragmented: identity says `20e7488...`, spec/contract say `9ca6a8b...`,
  RES-79 smoke says `395c194...`, actual HEAD is `8f26736...` (MED-015). `tests/test_res78...:80`
  enforces the spec's value while the identity file disagrees.

## 3. External evidence authorities are unbound (CRITICAL)

`_load_authorities()` (canonical_runtime.py:74-85) reads five JSONs from absolute evidence paths with no
hash check: `recovery_ready_spec.json`, `RECOVERY_MANIFOLD_AUTHORITY.json`,
`stand_handoff_ready_spec.json`, `RECOVERY_TIME_SCALING_AUTHORITY.json`, RES-52
`experiment_spec.json` (CONTROLLER_CONSTANTS). Mutating any of these changes RR gating, the recovery
manifold/T_RISE, handoff thresholds or the terminal BVLS constants while every declared hash stays
valid. `support_continuity.load_spec()` verifies a SPEC_SHA256 but is never called on this path.

## 4. Clean-execution audit (deterministic)

- `uv build --wheel` produced `loaded_cmj_control-0.1.0-py3-none-any.whl`: 64 entries; `tools/` absent;
  `v2_plant.xml` present; no scipy in `Requires-Dist`.
- Production imports `core52`, `soft_contact`, `evid_trace_v2` (canonical_runtime.py:53-55) and `scipy`
  (`balance_capture.py:26`, `stable_recovery.py:34`, `soft_contact.py:35`). `scipy` is absent from
  `uv.lock` and from `pyproject.toml`.
- Therefore a clean install cannot import `loaded_cmj.v2.canonical_runtime`, and even a source checkout
  cannot run it without the external evidence tree (absolute `/home/litju/...` paths).
- 35 hardcoded `/home/litju` references exist across `src/` and `tools/`; production modules inject
  `sys.path`; `evid_trace_v2` is imported under two distinct module identities
  (`evid_trace_v2` and `tools.evid_trace_v2`).

## 5. Scorer/controller separation

The runtime ingests scorer state observationally (`watcher.update`, checkpoints, termination) - category A.
But it also calls the scorer's private predicate `watcher._is_true_standing_neighborhood(...)` at every
control step and passes `env0` into `StableRecoveryController.step`, where it gates SETTLE->HANDOFF -
category B (a pure physical predicate implemented inside scorer code, consumed by control). It is not
event circularity, but it does mean modifying `events.py` (or the unhashed `constants.py` it reads)
changes the applied action stream. The authority claims "No detector read for control" /
"Scorer runs observationally only" are false as written.

## 6. Output schema

`CANONICAL_V2_RUNTIME_CONTRACT.md:107-115` requires `ENTRY_HEAD/TREE`, `PLANT_SHA256`,
`CONTROLLER_COMPOSITION_SHA256`, `SCORER_SHA256`, `CONTROL_DT_SEMANTICS` and `CTRL_MODES` in the result.
The emitted result contains none of them. Scorer metrics (`APEX_COM_Z`, `FLIGHT_RISE_FROM_TAKEOFF`,
`TAKEOFF_VZ`, `FLIGHT_DURATION`, `COUNTERMOVEMENT_DEPTH`, landing peaks) are computed and discarded at
serialization; a reader of the canonical result cannot distinguish a 3.6 cm hop from a functional CMJ.
`WALL_S` additionally makes the JSON non-byte-deterministic.

## 7. Ranked provenance findings

| Rank | ID | Finding | Severity |
|---|---|---|---|
| 1 | CRIT-004 | Candidate identity does not freeze the executing system | CRITICAL |
| 2 | CRIT-005 | External runtime authorities unbound | CRITICAL |
| 3 | CRIT-006 | Package not self-contained; scipy undeclared; clean install fails | CRITICAL |
| 4 | HIGH-007 | Scorer predicate gates control; separation claim false | HIGH |
| 5 | HIGH-013 | Output omits performance metrics and required schema fields | HIGH |
| 6 | MED-015 | Entry-head fragmentation / stale corrupted ledger | MEDIUM |
| 7 | MED-016 | Absolute paths, sys.path injection, duplicate module identities | MEDIUM |
| 8 | MED-014/018 | Horizon and constant authority duplication/drift | MEDIUM |
| 9 | LOW-006 | WALL_S breaks byte determinism | LOW |

**Verdict:** computational reproducibility of R001 in the author's tree is real, but the frozen-candidate
provenance claim is not: identity is a partial fingerprint over a system that depends on unhashed source
and mutable external evidence, and the shipped package cannot reproduce the run.
