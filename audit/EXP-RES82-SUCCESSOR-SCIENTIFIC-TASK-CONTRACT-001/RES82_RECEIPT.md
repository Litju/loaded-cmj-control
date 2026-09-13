# RES82_RECEIPT — Successor Loaded-CMJ Scientific Task Contract

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
LINEAR_ISSUE: `RES-82`
STATUS: **COMPLETE — STRUCTURAL CONTRACT SEALED; NUMERIC CLOSURE VIA OWNER DECISIONS**

---

## 1. Entry authority

| Item | Value |
|---|---|
| ENTRY_HEAD | `12ea42029489ddc2839389beafcf0811b7b45271` |
| ENTRY_TREE | `79f183724486ff95a1269a87cdb21d078b1893a5` |
| COMMIT_MESSAGE | `V2: seal post-RES80 authority reset and errata` |
| BRANCH | `main` |
| ORIGIN/MAIN at entry | `12ea42029489ddc2839389beafcf0811b7b45271` (verified equal) |
| RES80_AUDIT_HEAD | `8c1cbb49c91ba9e6d504a1df63fba242efcf7c64` |
| RES80_EVIDENCE_SEAL | `c921e6de29a263be1bdcee78c9fa62c22b968388991cda50afa34bf3260a9341` |
| RES81_EVIDENCE_SEAL | `a0ed5304e6835fa9a367e3dfb11d36d652c80fbc99e0eca60fd1d99027a5eeca` |
| R001_TRACE | `4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561` |

R001 remains `HISTORICAL_REPRODUCIBILITY_ONLY`, physical-visual credibility `FAIL`,
ship `BLOCKED`. No R001 authority was modified.

## 2. Deliverables

Tracked under `audit/EXP-RES82-SUCCESSOR-SCIENTIFIC-TASK-CONTRACT-001/`:

| Artifact | Purpose |
|---|---|
| `SUCCESSOR_CONTEXT_OF_USE.md` | frozen COU, model risk, claim permission |
| `SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.csv/.md` | 37-source, 17-field provenance table |
| `SCIENTIFIC_CLAIM_CEILING.md` | hard claim ceiling and V&V position |
| `SUCCESSOR_TASK_CONTRACT.md/.json` | layers, composition, terminology freeze |
| `EVENT_CONTRACT_E1_E12.md/.json` | normative E1–E12 semantics, blockers, NC anchors |
| `PERFORMANCE_METRIC_CONTRACT.md/.json` | jump-height adjudication, primary gate, consistency gates |
| `LANDING_BALANCE_RECOVERY_CONTRACT.md/.json` | L4/L5 whole-body landing/capture/recovery |
| `DWELL_SEMANTICS.md/.json` | MED-001 closure and per-event dwell declarations |
| `NEGATIVE_CONTROL_CONTRACT.md/.json` | NC-01..NC-12 designs |
| `CANONICAL_OUTPUT_SCHEMA.json` | mandatory result/metric schema with cross-field rules |
| `MODEL_DEPENDENCY_MATRIX.json` | Plant-independent vs deferred requirements |
| `OWNER_DECISIONS_REQUIRED.md/.json` | OD-01..OD-13 with evidence and recommendations |
| `SOURCE_REVIEW_COVERAGE.json` | search coverage and gaps |
| `citation_resolution.json` | Europe PMC resolution of 23/23 external sources |
| `CONTRACT_CONSISTENCY_REPORT.md/.json` | 14 mechanical gates + R001 retrospective |
| `INDEPENDENT_REVIEW_PASSES.md` | nine review passes and closures |
| `validate_contract.py`, `validate_schema.py`, `check_evidence_links.py` | read-only validators |
| `build_evidence_table.py`, `build_citation_resolution.py` | read-only generators |
| `RES82_RECEIPT.md` | this receipt |

## 3. Key adjudications

- **Primary canonical jump height:** `COM_RISE_TAKEOFF_TO_APEX` (direct model-truth
  displacement). Cross-check: ballistic height from takeoff `vz`; comparability:
  flight-time height; context: apex above initial standing. Bare "jump height" is
  prohibited.
- **One primary magnitude gate:** `COM_RISE_TAKEOFF_TO_APEX >= H_MIN`; `H_MIN` is an
  owner decision (OD-01) with candidate PF-1 = 0.150 m recommended, anchored to
  published 20 kg CMJ performance and disclosed as a cross-family plausibility bound.
- **Takeoff:** `PHYSICAL_TAKEOFF` (contact-set empty + positive clearance both feet +
  upward COM + no substituted support). `FORCE_THRESHOLD_TAKEOFF` is measurement-only
  (HIGH-010).
- **Genuine flight:** sustained physical no-contact + clearance for the declared dwell
  (candidate 0.050 s; OD-03); no unexecuted geometric claim remains.
- **Apex:** deterministic zero-crossing inside physical flight; **no dwell**
  (`APEX_DWELL_REQUIRED=NO`; HIGH-011).
- **E10:** whole-body landing admissibility; vertical arrest alone is insufficient,
  and the R001 forward lunge is excluded by the behavioral `L4-T8` window plus
  `L4-T9`/`L4-T10`.
- **E11:** `com_speed_sagittal = sqrt(vx²+vz²)`, `Hy`, support margin, CoP validity,
  bilateral support (MED-002).
- **E12:** robust physical standing envelope calibrated by RES-87; one-trace/ULP
  equality prohibited; physical-state event with controller handoff reported
  separately (HIGH-012, MED-013).
- **Dwell:** PHYSICAL_TIME integer-index convention; the one-sample-early historical
  convention is retired (MED-001); REFLIGHT is an explicit SAMPLE_COUNT rule
  preserving RES-57 detection.
- **Support continuity:** scoped `QUALIFIED`/`NOT_QUALIFIED`/`NOT_EVALUABLE`;
  `FULL_EPISODE` required for success (MED-012).
- **E1:** requires no fall contact and COM quiescence (MED-020, ADV-7).
- **Task success:** strict conjunction of eight independent components; event-chain
  completion can never imply success (`EVENT_CHAIN_COMPLETION != TASK_SUCCESS`).

## 4. Consistency and verification

| Gate | Result |
|---|---|
| `validate_contract.py` (14 gates) | 14/14 PASS |
| `validate_schema.py` | PASS; meta-validation + 4 executed instance fixtures (1 valid, 3 must-reject all rejected) |
| `check_evidence_links.py` | PASS; 37 rows, 0 unresolved references |
| Independent review passes | 9/9 executed; adversarial FAIL closed; final state PASS_WITH_FINDINGS with only owner-decision-limited residuals |
| Production source changes | none (tracked worktree clean; denylisted paths untouched) |

## 5. R001 retrospective

`R001_SUCCESSOR_CONTRACT_RESULT = FAIL`, with multiple independent causes:
performance floor (PF-1 candidate), physical takeoff/flight transition, apex flight
guard not demonstrated, takeoff whip (−9.6837 rad/s), whole-body landing state
(Hy 9.88; root pitch 0.292 rad; forward-accelerating `com_vx` 0.211 → 0.342 m/s),
balance capture under corrected COM speed, recovery envelope/regime, full-episode
support continuity (`NOT_QUALIFIED`, chatter 188, reflight `[4, 8, 1815]`), and the
anatomical model state owned by RES-83. Unknown R001 quantities are marked
`UNKNOWN_NOT_EVALUABLE`.

## 6. Owner decisions

13 decisions registered (OD-01..OD-13), all open or acknowledgment-required, with
options, evidence rows, tradeoffs, and recommendations. No unresolved numeric choice
is hidden; no candidate may be qualified while an affected decision is open.

## 7. Evidence and commit

- Evidence bundle: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES82-SUCCESSOR-SCIENTIFIC-TASK-CONTRACT-001/`
- Seal convention: `checksums.sha256` excludes itself, `SEAL.json`, and
  `POSTCOMMIT_SIDECAR*`; `EVIDENCE_SEAL_SHA256 = sha256(checksums.sha256 bytes)`.
- Commit message: `V2: seal successor loaded-CMJ scientific task contract`.
- Final HEAD/tree and remote-sync proof: recorded in `POSTCOMMIT_SIDECAR.json`
  (excluded from checksums, per convention) and in the mission return.

## 8. Next authorized action

`RES-83_HUMAN_VALID_PLANT_REBUILD` — unless the owner elects to close OD-01/OD-03/
OD-13 before Plant work; no other successor work was started by this mission.
