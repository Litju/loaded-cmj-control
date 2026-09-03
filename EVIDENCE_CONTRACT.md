# Evidence Delivery Contract

**Version:** 2.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Ledger:** `AUTHORITY_LEDGER.json`  
**Trace schema:** `TRACE_SCHEMA_V2.md` (`LCMJ_TRACE_SCHEMA_VERSION=2`)  
**Status:** CANONICAL — hard gate for `EVIDENCE_DELIVERED`

Changelog vs 1.0.0 (R0.1 normalization):
- Ambiguous `manifest_sha256` replaced by explicit `MANIFEST_FILE_SHA256` /
  `MANIFEST_CANONICAL_SHA256` with frozen canonical algorithm (§5) and
  documented self-reference rule (FILE hash lives outside `manifest.json`).
- `experiment_spec.json` / `run_record.json` / `result_assessment.json`
  separation with `SPEC_EXECUTION_MATCH` binding (§6).
- Sealed reproduction evidence required inside the bundle (§7).
- Full `mjSTATE_INTEGRATION` certificate required (§8).
- Authority-safe `reproduce.sh` with hard refusal (§9).
- Trace schema v2 with sample-count rule (§10).
- Complete `claim_evidence.csv` binding (§11).

---

## 1. Principle

A mission cannot return `PASS` merely because evidence exists somewhere locally. Qualification requires a single **owner-deliverable evidence archive** whose path, hash, byte count, manifest, and inventory are shown by executed `stat`, `sha256sum`, and archive listing. A textual receipt is never a substitute for the bundle.

If the bundle cannot be delivered or its concrete path/hash/inventory cannot be shown:

```
STATUS=BLOCKED_EVIDENCE_NOT_DELIVERED
```

Do **not** return `PASS`.

---

## 2. Bundle Layout (required)

Every future experiment influencing a decision must generate a run directory that, before archiving, contains at minimum:

```
<evidence_root>/<mission_id>/
  manifest.json                 # machine-generated manifest (no hand-entered hashes)
  experiment_spec.json          # PRE-EXECUTION: immutable predeclaration (sealed before run)
  run_record.json               # EXECUTION: what actually executed (carries EXPERIMENT_SPEC_SHA256)
  result_assessment.json        # POST-EXECUTION: interpretation only (never a new claim)
  environment.json              # MuJoCo/Python/arch/OS/uv.lock/dependency hashes
  initial_integration_state.npz # full mjSTATE_INTEGRATION vector + readability mirrors
  initial_integration_state.json# state_spec/state_size/sha + MUJOCO/ARCH/MODEL/COMMIT identity
  physics_trace.npz             # schema v2 per-physics-step samples
  control_trace.npz             # schema v2 per-control-step actions
  branch_control_sequence.npz   # exact replay authority (recorded continuation actions)
  events_online.json            # V2EventDetector online records (ordered)
  events_offline.json           # offline recomputation (must match online)
  metrics.json                  # independently recomputed from physics_trace
  experiments.jsonl             # legacy one-line mirror (experiment_id + spec sha)
  claim_evidence.csv            # complete claim binding (§11)
  reproduction.json             # REPRODUCTION: sealed fresh-process verdict
  reproduction_stdout.txt       # REPRODUCTION: captured stdout of the replay process
  reproduction_stderr.txt       # REPRODUCTION: captured stderr of the replay process
  tests/
    <qualification checks>.txt
  reviews/
    CODE_REVIEW.md
    BUG_HUNT.md
    SCIENTIFIC_EVIDENCE_REVIEW.md
  reproduce.sh                  # authority-safe single-command replay (hard refusal)
  checksums.sha256              # sha256sum of every file above (excluding itself)
  FINAL_RECEIPT.md              # mechanically generated from manifest+reproduction (no hand hashes)
```

### 2.1 Lifecycle partition

- **PRE-EXECUTION:** `experiment_spec.json`, `experiments.jsonl` (mirror).
- **EXECUTION:** `run_record.json`, `environment.json`,
  `initial_integration_state.*`, `physics_trace.*`, `control_trace.*`,
  `branch_control_sequence.npz`, `events_online.json`, `events_offline.json`,
  `metrics.json`.
- **POST-EXECUTION:** `result_assessment.json`, `claim_evidence.csv`
  (preliminary statuses; reproduction-bound rows sealed later).
- **REPRODUCTION:** `reproduction.json`, `reproduction_stdout.txt`,
  `reproduction_stderr.txt` (written only by a fresh process via
  `reproduce.sh`; sealed into manifest by `tools/evid_finalize.py`).
- **AUDIT:** `tests/`, `reviews/`, `reproduce.sh`, `checksums.sha256`,
  `manifest.json`, `FINAL_RECEIPT.md`.

The evidence system prevents post-hoc modification of the immutable
experiment specification without changing its hash and experiment version:
the spec is sealed (`EXPERIMENT_SPEC_SHA256`) before execution, the run
record carries that sha, and any content change fails `SPEC_EXECUTION_MATCH`.

---

## 3. Delivery Fields (hard-required final output)

For every qualification/audit run, the mission final return must create and display the actual outputs of:

```
EVIDENCE_BUNDLE_PATH=<absolute real path>
EVIDENCE_BUNDLE_SHA256=<64 lowercase hex>
EVIDENCE_BUNDLE_BYTES=<integer>
EVIDENCE_MANIFEST_PATH=<absolute path>
MANIFEST_FILE_SHA256=<64 lowercase hex>
MANIFEST_CANONICAL_SHA256=<64 lowercase hex>
EVIDENCE_FILE_COUNT=<integer>
EXPERIMENT_SPEC_PATH=<absolute path>
EXPERIMENT_SPEC_SHA256=<64 lowercase hex>
STATE_VECTOR_SHA256=<64 lowercase hex>
REPRODUCTION_RESULT_PATH=<absolute path>
```

Verification commands (executed, not described):

```bash
stat "$EVIDENCE_BUNDLE_PATH"
sha256sum "$EVIDENCE_BUNDLE_PATH"
sha256sum "$EVIDENCE_MANIFEST_PATH"
find "$EVIDENCE_BUNDLE_PATH" -type f | wc -l   # or unzip -l / tar -tzf | wc -l for the archive
tar -tzf "$EVIDENCE_BUNDLE_PATH" | sort         # archive inventory verification
```

The values are **returned** in the final mission output, not merely logged. If the interface permits attachment, **ATTACH THE EVIDENCE BUNDLE**.

A mission that cannot show all fields plus the command outputs is `BLOCKED_EVIDENCE_NOT_DELIVERED`.

---

## 4. Deterministic Recorder — Implementation

The reference implementation is `tools/evid_*.py` (R0.1) superseding
`tools/evidence_recorder.py` (v1, retained for provenance). It:

- snapshots `git rev-parse HEAD` / `HEAD^{tree}` at call time (no argument);
- hashes `src/loaded_cmj/v2/assets/v2_plant.xml`, `constants.py`, `plant.py`,
  `controller.py`, `events.py`, `drive.py`, `uv.lock`;
- captures `mujoco.__version__`, `numpy.__version__`, `platform.machine()`/`platform.system()`, `sys.version`;
- seals `experiment_spec.json` before execution and binds it via `run_record.json`;
- captures the branch point with `mj_stateSize` / `mj_getState` (§8);
- streams schema-v2 `physics_trace.npz` / `control_trace.npz` as the rollout proceeds;
- calls `V2EventDetector` both online and offline and asserts identity;
- independently recomputes `metrics.json` from the trace (not from detector comments);
- emits `checksums.sha256` covering every other file;
- refuses to seal `FINAL_RECEIPT.md` reproduction claims until `reproduction.json`
  exists with `EXIT_CODE==0` and `IDENTICAL==true`;
- fails closed on any nonfinite, non-bounded, or missing artifact.

Recorder output is byte-for-byte reproducible for the same code commit and
initial state. The v1 recorder's historical bundles remain valid under the
v1 contract; new bundles must satisfy this v2 contract.

---

## 5. Hash Contract

For every artifact in an evidence bundle:

```
FILE_SHA256 = SHA256(raw bytes)
```

`checksums.sha256` must list every immutable evidence artifact except
`checksums.sha256` itself (documented self-reference rule: a checksum file
cannot contain its own hash).

For `manifest.json` specifically, two hashes with never-shared names:

```
MANIFEST_FILE_SHA256      = SHA256(raw manifest.json bytes on disk)
MANIFEST_CANONICAL_SHA256 = SHA256(canonical serialization of the manifest
                            content with only the documented self-hash
                            fields removed)
```

Self-hash removal set (frozen): `MANIFEST_FILE_SHA256`,
`MANIFEST_CANONICAL_SHA256`.

Frozen canonical JSON algorithm:

- UTF-8;
- `json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)`;
- no trailing newline in the hashed bytes;
- deterministic key ordering and separators;
- floats via CPython repr (deterministic for float64);
- no nondeterministic timestamps inside canonicalized content unless the
  timestamp itself is part of the intended manifest content.

Self-reference rule (frozen): `manifest.json` stores
`MANIFEST_CANONICAL_SHA256` inside itself (verifiable by stripping both
self fields and re-canonicalizing). It does NOT store
`MANIFEST_FILE_SHA256` inside itself — no file can contain its own raw-byte
hash. The FILE hash is recorded externally in `checksums.sha256` (the
`manifest.json` line), `FINAL_RECEIPT.md`, delivery fields, and the sealing
commit message.

A unit test proves identical logical manifests generate identical
`MANIFEST_CANONICAL_SHA256` (`tests/test_r01_manifest_hash.py`). No
hand-entered values are accepted (hex-format + recomputation checks).

---

## 6. Experiment Spec Contract

`experiment_spec.json` is immutable before execution. Required fields:

```
EXPERIMENT_ID, EXPERIMENT_VERSION, MISSION,
AUTHORITY_COMMIT_SHA, AUTHORITY_COMMIT_TREE,
HYPOTHESIS, START_STATE_AUTHORITY,
ALLOWED_VARIABLES, FROZEN_VARIABLES,
SEARCH_METHOD, CANDIDATE_ORDERING,
BUDGET_DEFINITION, HARD_GATES,
OBJECTIVE_HIERARCHY, STOPPING_RULE,
QUALIFICATION_OR_DIAGNOSTIC, EXPECTED_OUTPUTS,
SPEC_CREATED_AT, EXPERIMENT_SPEC_SHA256
```

`BUDGET_DEFINITION` must carry explicit counters (§7 of
`EXPERIMENT_PROTOCOL.md`); bare `BUDGET=12` is forbidden.

The run recorder carries `EXPERIMENT_SPEC_SHA256` in `run_record.json` and
proves it executed the declared experiment over the bound fields
(ID/version/authority/horizon/branch/control-law/dts/continuation-steps).
If execution parameters differ: `SPEC_EXECUTION_MATCH=FAIL`. The result is
preserved as exploratory evidence but cannot claim qualification under the
mismatched spec.

---

## 7. Reproduction Contract

Future bundles must include actual reproduction evidence (written by a fresh
process, captured by `reproduce.sh`):

```
reproduction.json, reproduction_stdout.txt, reproduction_stderr.txt
```

`reproduction.json` minimum fields:

```
COMMAND, START_TIME, END_TIME, EXIT_CODE,
SOURCE_TRACE_SHA256, REPLAY_TRACE_SHA256, IDENTICAL,
MAX_QPOS_ERROR, MAX_QVEL_ERROR,
ONLINE_OFFLINE_EVENT_IDENTITY, ENVIRONMENT_MATCH, COMMIT_MATCH
```

The final receipt derives reproduction claims from these files. Sealing
requires `EXIT_CODE==0` and `IDENTICAL==true` with zero qpos/qvel error.

---

## 8. Initial State Contract

The state archive is a canonical full MuJoCo integration-state certificate
using `mj_stateSize(model, mjSTATE_INTEGRATION)` / `mj_getState(...)` /
`mj_setState(...)`. Persisted:

```
state_spec = mjSTATE_INTEGRATION, state_size, state_vector,
state_vector_sha256
```

plus metadata proving `MUJOCO_VERSION`, `ARCHITECTURE`, `MODEL_HASH`,
`COMMIT_SHA`, `COMMIT_TREE` (`initial_integration_state.json`).

For this Plant the vector is 108 float64
`[time(1), qpos(10), qvel(10), qacc_warmstart(10), ctrl(7),
qfrc_applied(10), xfrc_applied(60)]`. Replay from an arbitrary branch point
must use `mj_setState` with this vector — never manual `qpos/qvel` copying,
which drops the warmstart/applied-force tails. Controller internal state
needed to reproduce decisions is recorded separately in `control_trace.npz`
(`phase`, `internal_state`) and `branch_control_sequence.npz`.

---

## 9. reproduce.sh Authority Safety

`reproduce.sh` must verify before replaying:

```
CURRENT_COMMIT_SHA == MANIFEST_COMMIT_SHA
CURRENT_TREE == MANIFEST_COMMIT_TREE
MUJOCO_VERSION == MANIFEST_MUJOCO_VERSION
MODEL_HASH == MANIFEST_MODEL_HASH
```

On any mismatch it exits non-zero with an explicit `AUTHORITY_MISMATCH`
message. It never rewrites history or checks out another revision. Simple
hard refusal is the contract.

---

## 10. Trace Contract

Trace schema v2 (`TRACE_SCHEMA_V2.md`, `LCMJ_TRACE_SCHEMA_VERSION=2`)
records at physics rate at least: time; qpos/qvel/qacc; ctrl;
qfrc_actuator/qfrc_passive/qfrc_constraint; root position/velocity;
COM position/velocity; centroidal `H` (at minimum `H_y`); trunk orientation
and angular rate; left/right `Fz`, full contact wrenches where available,
CoP; contact geom/body identities, distance/penetration, count;
constraint-row data (`efc_type/efc_id/efc_force`) auditing root/joint/contact
constraints; support polygon/margin; actuator torque/utilization; controller
phase and internal state; event detector state and records; fall,
prohibited-contact, and reflight/contact-state flags.

Derived values may be omitted only when a clearly documented deterministic
derivation exists — but every scientific claim must be traceable. Sample
counts obey `EXPECTED_PHYSICS_STEPS = round(T/physics_dt)` and
`EXPECTED_CONTROL_STEPS = round(T/control_dt)` with the endpoint rule in
`TRACE_SCHEMA_V2.md` (prevents 4 s / 64,000-step-class inconsistencies).
Schema changes increment the version.

---

## 11. Claim/Evidence Binding

`claim_evidence.csv` columns (minimum):

```
CLAIM_ID, CLAIM, CLAIM_TYPE, STATUS, SOURCE_ARTIFACT,
SOURCE_FIELD_OR_RANGE, DERIVATION, ACCEPTANCE_CRITERION, RESULT, NOTES
```

The bundle contains the complete relevant claim matrix for its experiment
(no sparse headline subset). No claim such as `NO_ROOT_SUPPORT`,
`NO_PROHIBITED_CONTACT`, `DETERMINISM`, `EVENT_IDENTITY`, or `E12_PASS` may
appear in `FINAL_RECEIPT.md` without a claim-evidence mapping row (an
explicit `NOT_CLAIMED` row satisfies the rule for out-of-scope claims such
as `E12_PASS` in an infrastructure bundle).

---

## 12. Self-Verification

Every evidence pipeline qualification includes a **harmless branch replay**
solely to prove the pipeline (not to tune the controller):

1. run a harmless deterministic trajectory for a short period;
2. choose a non-initial branch time after dynamics evolved;
3. capture full `mjSTATE_INTEGRATION` + state hash;
4. continue the original run for N control periods (recording the control sequence);
5. start a fresh process; verify commit/tree/environment;
6. restore the exact saved state with `mj_setState`;
7. replay the same control sequence;
8. compare continuation (qpos/qvel/physics-trace/control/event/contact identity).

Do not use this replay to change controller authority.

---

## 13. Non-Goals

- The evidence repository is **non-Git**; its authority is bundle hash, not Git history. Git history remains the source-repo authority.
- Giant traces are not stored in the source repo unless deliberately chosen; the separate evidence repository is acceptable and expected.
- No replacement historical evidence is ever manufactured.

---

## 14. Checklist Before Commit

- [ ] `manifest.json` contains `COMMIT_SHA` + `COMMIT_TREE` matching `git rev-parse HEAD` / `HEAD^{tree}`
- [ ] `MANIFEST_CANONICAL_SHA256` verifies by strip-and-recanonicalize; `MANIFEST_FILE_SHA256` in receipt/checksums matches raw bytes
- [ ] `experiment_spec.json` sealed before execution; `run_record.json` carries its sha; `SPEC_EXECUTION_MATCH==PASS`
- [ ] `initial_integration_state.*` is full `mjSTATE_INTEGRATION` (108 floats) with `mj_setState` restore proven from a nonzero branch
- [ ] `events_online.json == events_offline.json`
- [ ] `metrics.json` is independently recomputed from `physics_trace.*`
- [ ] sample counts equal `T/dt`, `T/control_dt` exactly
- [ ] `claim_evidence.csv` binds every receipt claim
- [ ] `reproduction.json` + stdout/stderr present with `IDENTICAL==true`, zero errors
- [ ] `FINAL_RECEIPT.md` is mechanically generated (no hand hashes)
- [ ] `checksums.sha256` lists every file
- [ ] `reproduce.sh` succeeds from the bound checkout and refuses a mismatched checkout
- [ ] Bundle path/hash/bytes/manifest/file-count displayed via `stat`/`sha256sum`/inventory and attached if possible
