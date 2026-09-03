# Evidence Delivery Contract

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Ledger:** `AUTHORITY_LEDGER.json`  
**Status:** CANONICAL — hard gate for `EVIDENCE_DELIVERED`

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
  environment.json              # MuJoCo/Python/arch/OS/uv.lock/dependency hashes
  initial_integration_state.npz # full MjData required for exact replay (qpos qvel qacc ctrl time + DriveState + previous_action)
  physics_trace.npz             # per-physics-substep samples (float64, contiguous, time-ordered)
  control_trace.npz             # per-control-step actions/observations/targets
  events_online.json            # V2EventDetector online records (ordered)
  events_offline.json           # offline recomputation (must match online)
  metrics.json                  # force-time, CoP, margins, residuals, utilization — independently recomputed
  experiments.jsonl             # predeclared experiment definitions (one JSON per line)
  tests/
    <qualification tests>.py / .log
  reviews/
    CODE_REVIEW.md
    BUG_HUNT.md
    PONYTAIL_REVIEW.md (or explicit waiver with reason)
  claim_evidence.csv            # subset of CLAIM_EVIDENCE_MATRIX.csv for this run
  reproduce.sh                  # single-command fresh-process replay that restores from initial_integration_state.*
  checksums.sha256              # sha256sum of every file above (excluding itself)
  FINAL_RECEIPT.md              # mechanically generated from manifest (no hand-entered hashes)
```

### 2.1 Manifest fields (exact)

`manifest.json` must contain, as strings/numbers with provenance:

```json
{
  "mission": "LCMJ-R0-SCIENTIFIC-REBASE-20260903",
  "evidence_version": "1.0.0",
  "commit_sha": "2a5967d... (from git rev-parse HEAD)",
  "commit_tree": "b7bec500... (from git rev-parse HEAD^{tree})",
  "mujoco_version": "3.8.0",
  "python_version": "3.13.13",
  "numpy_version": "2.5.1",
  "architecture": "x86_64",
  "os": "Linux 6.6.114.1-microsoft-standard-WSL2",
  "uv_lock_hash": "a80b951e...",
  "model_xml_hash": "5f224414...",
  "plant_hash": "86b22181...",
  "controller_hash": "870bab26...",
  "scorer_hash": "286ef328... / 8b90eea8...",
  "solver": "Newton",
  "integrator": "implicitfast",
  "timestep": 0.000125,
  "substeps_per_control": 40,
  "horizon_s": 8.0,
  "physics_steps": 64000,
  "manifest_sha256": "self-hash computed after fill but before FINAL_RECEIPT",
  "trace_sha256": "canonical bytes of physics_trace.npz",
  "initial_state_sha256": "bytes of initial_integration_state.npz"
}
```

**Rule:** `FINAL_RECEIPT.md` is rendered from this manifest. No hash in the receipt is typed by hand.

### 2.2 Integration state — full disclosure

`initial_integration_state.npz` must store exactly the bytes needed for bit-identical replay: `qpos (10)`, `qvel (10)`, `qacc (10)`, `ctrl (7)`, `time`, `qacc_warmstart (21 if used)`, `DriveState (a_plus,a_minus,tau_prev,previous_command,override_flags,reversal_phase)` where applicable, and `previous_action (7)`. Speculative or lossy states are not acceptable. The file must be loadable by `reproduce.sh` without recomputing hidden state.

---

## 3. Delivery Fields (hard-required final output)

For every qualification/audit run, the mission final return must create and display the actual outputs of:

```
EVIDENCE_BUNDLE_PATH=<absolute real path>
EVIDENCE_BUNDLE_SHA256=<64 lowercase hex>
EVIDENCE_BUNDLE_BYTES=<integer>
EVIDENCE_MANIFEST_PATH=<absolute path>
EVIDENCE_MANIFEST_SHA256=<64 lowercase hex>
EVIDENCE_FILE_COUNT=<integer>
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

A mission that cannot show all six fields plus the three command outputs is `BLOCKED_EVIDENCE_NOT_DELIVERED`.

---

## 4. Deterministic Recorder — Implementation

The reference recorder is `tools/evidence_recorder.py` (part of this rebase). It:

- snapshots `git rev-parse HEAD` / `HEAD^{tree}` at call time (no argument);
- hashes `src/loaded_cmj/v2/assets/v2_plant.xml`, `constants.py`, `controller.py`, `events.py`, `uv.lock`;
- captures `mujoco.__version__`, `numpy.__version__`, `platform.machine()`/`platform.system()`, `sys.version`;
- writes `initial_integration_state.npz` via `Plant.make_data()` exact fields;
- streams `physics_trace.npz` / `control_trace.npz` as the rollout proceeds;
- calls `V2EventDetector` both online and offline and asserts identity;
- independently recomputes `metrics.json` from the trace (not from detector comments);
- emits `checksums.sha256` and refuses to emit `FINAL_RECEIPT.md` until `checksums.sha256` covers every other file;
- fails closed on any nonfinite, non-bounded, or missing artifact.

Recorder output is byte-for-byte reproducible for the same code commit and initial seed (the project's seed handling remains `DETERMINISTIC_SEED 0, SEED_IS_CONSUMED False`).

---

## 5. Self-Verification

Every evidence pipeline qualification includes a **harmless standing replay** solely to prove the pipeline (not to tune the controller):

1. produce a bundle;
2. destroy in-memory state;
3. start a fresh Python process;
4. restore from `initial_integration_state.npz` + `environment.json`;
5. rerun via `reproduce.sh`;
6. independently recompute metrics;
7. verify identical/declared reproducibility and that every `FINAL_RECEIPT.md` value traces to a raw artifact.

Do not use this replay to change controller authority.

---

## 6. Non-Goals

- The evidence repository is **non-Git**; its authority is bundle hash, not Git history. Git history remains the source-repo authority.
- Giant traces are not stored in the source repo unless deliberately chosen; the separate evidence repository is acceptable and expected.
- No replacement historical evidence is ever manufactured.

---

## 7. Checklist Before Commit

- [ ] `manifest.json` contains `COMMIT_SHA` + `COMMIT_TREE` matching `git rev-parse HEAD` / `HEAD^{tree}`
- [ ] `manifest.json` contains every version/hash in §2.1
- [ ] `initial_integration_state.npz` reproduces bit-identically in a fresh process (`diff <1e-12` not sufficient for trace digest — digest must match)
- [ ] `events_online.json == events_offline.json`
- [ ] `metrics.json` is independently recomputed from `physics_trace.*`
- [ ] `FINAL_RECEIPT.md` is mechanically generated (no hand hashes)
- [ ] `checksums.sha256` lists every file
- [ ] `reproduce.sh` succeeds from a clean checkout
- [ ] Bundle path/hash/bytes/manifest/file-count displayed via `stat`/`sha256sum`/inventory and attached if possible
