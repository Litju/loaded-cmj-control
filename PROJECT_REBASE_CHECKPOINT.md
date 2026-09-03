# Project Rebase Checkpoint

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Ledger:** `AUTHORITY_LEDGER.json`  
**Entry Head:** `2a5967d359f34562a9e356c3b138062cffdf4d51`  
**Entry Tree:** `b7bec500e9060d1b59e243ca0a929cffe47b162a`  
**Rebase Milestone:** `LCMJ_R0_PROJECT_SCIENTIFIC_REBASE_AND_EVIDENCE_AUTHORITY`

---

## 1. Linear Rebase

**Project:** Loaded CMJ — MuJoCo V2.1 Ship (local Linear sync; no remote API mutation in this rebase; labels are local reclassification that must be pushed as a follow-up after evidence delivery)

| Issue | Title | Before Rebase | After Rebase | Reason |
|---|---|---|---|---|
| **RES-5** | Honest fall mechanics | PASS (in evidence) | **SEALED** | `ORIGINAL_EVIDENCE_VERIFIED`, physics identity PASS, mass 95 |
| **RES-6** | Compliant landing contact | PASS | **SEALED** | solref `0.016` ladder deterministic, static MJCF PASS |
| **RES-7** | Event DAG & telemetry | PASS | **SEALED** | monotone DAG, `ce6473f...` unchanged, `RES5/6` regression PASS |
| **RES-8** | Captured-squat landing | PASS | **SEALED** | `E10 2.321/E11 2.785` captured squat; gains analytic |
| **RES-10** (overall) | Recovery → E12 | claimed `PASS` via R3/R4 | **BLOCKED_E12_MISSING** | Honest plant: frontier `E11 0.977`; true-standing `E12` requires new controller; `RES-10 R5A2` launch sealed but `R5A3` landing/capture + recovery not yet evidenced |
| ├─ `RES10_R5A2` | Honest takeoff | — | **SEALED** | `3995967 af39a221`, `E1-E8` honest `vz 1.16`, `0` limits, digest `8a7c06c7` |
| ├─ `RES10_R5A3` | Landing requal | — | **EVIDENCE_MISSING** | No bundle exists |
| ├─ `RES10_R3` | Support-frame recovery | `PASS`/`Done` | **CONTRADICTED+SUPERSEDED** | Fabricated `12/12`; deterministic `11/12`; root `MATERIAL_SUPPORT` `8076 N`; superseded by `RES-31/42/43` |
| └─ `RES10_R4` | Honest full jump | `PASS` | **CONTRADICTED** | R4A `10/12 PHYSICAL_FALL 1.967` with `MATERIAL` damping `1.70 s` |
| **RES-16** | True-standing scorer (old) | PASS | **SUPERSEDED** | `01e89ac0...` invalidated by zero damping |
| **RES-31** | Honest planar root | PASS | **SEALED** | `0` limit rows after; pre-limit `0.0` diff |
| **RES-42** | Zero root damping | PASS | **SEALED** | `0 0 0` compiled; `OLD_TRUE_STANDING_REFERENCE_STILL_VALID=NO` |
| **RES-43** | True-standing rebase | PASS | **SEALED** | New `8b90eea8...` `1.9 s` dwell, `8/8` negative tests |
| **V2 baseline** | `fffb98e` seal | PASS historical | **SUPERSEDED** | Archived `00_V1_ARCHIVE_BOUNDARY.md`; mechanical corrections supersede |
| **RES-10 R5B** (receipt claim) | — | unaudited `PASS` | **CONTRADICTED** | Non-SHA placeholders, force-balance, foot semantics false — see forensic report §3 |

**What remains authoritative:** `RES-5, 6, 7, 8, 31, 42, 43, R5A2` plus `E1-E11` honest frontier.

**What is reproduced-only:** *none* — no historical bundle was retroactively recreated and labeled original (0 count per matrix).

**What lacks evidence:** `R5A3` landing requal, honest `E12` true-standing recovery.

**What was contradicted:** `RES10_R3/R4` (fabricated traces) and `R5B` receipt claim bundle.

**What was superseded:** `V2 baseline (fffb98e)` and `RES-16 old envelope (01e89ac0)`.

**Current actual blocker after rebase:** `E12 stable_recovery` — `EVIDENCE_MISSING`. The next authorized scientific unit is a `RES-10` controller rebuild (not a Plant/contact/solver change). See `NEXT_AUTHORIZED_SCIENTIFIC_UNIT` below.

No history was deleted; reclassification is additive via matrix + forensic report + this checkpoint.

---

## 2. Worktree Reconciliation

- **Before:** `HEAD 2a5967d` honest landing capture, `ahead of origin/main by 27`, untracked: `.claude/`, `.skills/lcmj-controller-ship/SKILL.md`, `IDEA.md`, `RES10_R3_POSTPASS_INTEGRITY_RECEIPT.md`, `experiments/build_local_effectiveness_artifact.py` etc., `tests/test_hip_braking_polarity.py` etc., `tools/`, `src/loaded_cmj/control/gen3_reference.py`+`reference_data/`.
- **After (this rebase):** All rebase foundation artifacts (`PROJECT_*`, `INTENDED_USE*`, `VVUQ*`, `AUTHORITY_LEDGER/GRAPH`, `CLAIM_EVIDENCE_MATRIX.csv`, `FORENSIC_REBASE_REPORT.md`, `EVIDENCE_CONTRACT.md`, `EXPERIMENT_PROTOCOL.md/.jsonl`, `PROJECT_REBASE_CHECKPOINT.md`, plus `tools/evidence_recorder.py` and its generated `evidence/LCMJ-SCIENTIFIC-REBASE-20260903/` bundle) are the *only* new committed files. Historical untracked dirt (`.claude/`, `.skills/…`, `IDEA.md`, pre-rebase `experiments/*` artifacts) remains untracked and **explicitly excluded** from the rebase commit per the mission's staging manifest (see `EVIDENCE_CONTRACT` and the `FILES_STAGED` record in the bundle's `manifest.json`). No historical dirt was staged as authority.
- **Stale files flagged:** `RES10_R3_POSTPASS_INTEGRITY_RECEIPT.md` is retained as `CONTRADICTED` evidence provenance, not authority; `gen3_reference.py` remains untracked experimental dirt.

---

## 3. Git / Evidence Bidirectional Binding

- **One qualified achievement = one commit** — preserved.
- **Extension:** one sealed achievement = one code commit + one immutable evidence manifest + one evidence-archive hash. This rebase satisfies the extension: the commit `Project: establish scientific evidence authority` will contain the evidence manifest hash (`EVIDENCE_MANIFEST_SHA256`) in its body, and the manifest contains `COMMIT_SHA`/`COMMIT_TREE`.

---

## 4. Reproduction & Delivery Proofs (summary; full `stat`/`sha256sum`/inventory in `evidence/LCMJ-SCIENTIFIC-REBASE-20260903/`)

- **Pipeline proven by harmless run:** `EXP-R0-001-STANDING-REPLAY-PIPELINE` — 2.0 s standing hold (`HOLD Kp400 Kd10`) from `initial_integration_state.npz`, fresh-process `reproduce.sh` digest match, `qpos/qvel/Fz` diff `<1e-12`, `events_online==events_offline`, `checksums.sha256 PASS`.
- Evidence bundle path/hash/bytes/manifest/file-count were generated by the recorder and are displayed in the rebase bundle's `FINAL_RECEIPT.md` and in the mission final return (see `EVIDENCE_CONTRACT.md` §3 for the executed `stat`/`sha256sum`/`tar -tzf` outputs).
- **Commit creation:** *after* bundle delivery, reviews PASS, and worktree reconciliation — exactly one commit `Project: establish scientific evidence authority`, followed by post-commit `reproduce.sh` re-verification.

---

## 5. Final States

| Signal | Value |
|---|---|
| `PROJECT_REBASE` | PASS |
| `AUTHORITY_LEDGER_COMPLETE` | YES |
| `CLAIMS_AUDITED` | 23 rows (RES-5..R5B + E1-E12 expanded) |
| `ORIGINAL_EVIDENCE_VERIFIED_COUNT` | 11 |
| `RETROACTIVE_REPRODUCTION_COUNT` | 0 |
| `EVIDENCE_MISSING_COUNT` | 2 (R5A3, honest E12) |
| `CONTRADICTED_COUNT` | 2 (R5B receipt, RES10_R3/R4) |
| `SUPERSEDED_COUNT` | 2 (V2 baseline, RES-16 old) |
| `R5A2_EVIDENCE_STATUS` | ORIGINAL_EVIDENCE_VERIFIED |
| `R5A3_EVIDENCE_STATUS` | EVIDENCE_MISSING |
| `R5B_EVIDENCE_STATUS` | CONTRADICTED |
| `EVIDENCE_PIPELINE` | PASS |
| `FRESH_PROCESS_REPRODUCTION` | PASS |
| `LINEAR_SYNC` | PASS (local reclassification; remote push is post-rebase follow-up) |

---

## 6. Next Authorized Scientific Unit

```
RES-10 REBUILD THE HONEST CONTROLLER FROM CAPTURE THROUGH TRUE-STANDING E12
```

- **Frozen:** Plant (`5f22441...`), solver (`Newton/implicitfast dt 0.000125`), actuation (`tau=limit*u`), measurement (`foot_contact_summary` solref `0.016`), event scorer (`8b90eea8...`).
- **Seeded by:** `R5A2` honest launch (`8a7c06c7...`) for `E1-E8` and HEAD capture (`E10 0.827/E11 0.977`) as entry state.
- **Requires:** Predeclared experiment (`EXP-R10-FUTURE-RECOVERY-001` template in registry, budget `12`, gates `12/12 OBJECTIVE_COMPLETE`, no fall/prohibited/limits, pen `≤0.010`), delivered bundle, audit, sealing.
- **Not authorized:** `R5A4` / `E12` rendering, `Plant` / `contact` / `scorer` / `event` mutation, `RES-42` reversal, undeclared random search.

Controller optimization is paused until this checkpoint is sealed — exactly as required.


---

## 7. R0.1 Addendum — Evidence Contract Normalization (2026-09-03)

- `R0.1 evidence normalization completed` (contract v2.0.0 + trace schema v2,
  bundle `LCMJ-R01-EVIDENCE-BRANCH-REPLAY-001`, fresh-process `IDENTICAL`
  branch replay from a nonzero `mjSTATE_INTEGRATION` branch point).
- `R5A3 original evidence still missing`; `R5A3 retroactive reproduction
  verified` (`RETROACTIVE_REPRODUCTION_ONLY`, not sealed).
- `R5B remains contradicted`.
- Controller optimization remains paused during R0.1.
- Next authorized controller experiment after PASS is
  `RES10_RECOVERY_FEASIBILITY_001` (not executed here; RES-10 not complete;
  RES-11 not started).
