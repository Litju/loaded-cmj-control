# Forensic Rebase Report — Loaded CMJ MuJoCo

**Version:** 1.0.0  
**Date:** 2026-09-03  
**Constitution:** `PROJECT_SCIENTIFIC_CONSTITUTION.md`  
**Entry Head:** `2a5967d359f34562a9e356c3b138062cffdf4d51`  
**Entry Tree:** `b7bec500e9060d1b59e243ca0a929cffe47b162a`  
**Evidence Repository:** `/home/litju/Projects/loaded-cmj-control-evidence` (non-Git, bundle-hash authority)  
**Auditor Method:** Direct `sha256sum`, fresh-process `mj_step` replay, `efc_force` forensics, `mj_inverse` recomputation — no receipt trust

---

## 1. Authority Refoundation Summary

The project before this rebase accumulated 27 commits ahead of `origin/main` with intertwined mechanical corrections and controller tuning. The rebase was required because `RES10_R3` claimed `PASS`/`12/12 OBJECTIVE_COMPLETE` while its own evidence generation code fabricated traces and its compiled plant concealed two classes of artificial support (sagittal root limits + world-anchored root damping). This report adjudicates every milestone used by the current V2/V2.1 authority and fixes the claim ceiling.

**Resulting blocker after rebase:** `E12 stable_recovery` (true standing) — **EVIDENCE_MISSING** for a sealed honest controller. Plant, measurement, event, and landing authorities are sealed; launch to `E8` is sealed (`RES10_R5A2`); landing through `E11` is sealed via head capture (`E10 0.827→E11 0.977`); completing `E12` requires a new `RES-10` controller mission, not another mechanical fix.

---

## 2. Methodology

1. **Locate physically:** `find /home/litju/Projects/loaded-cmj-control-evidence -type f | wc -l = 470` files; `git log --all --oneline` 27 ahead; `sha256sum` every `FINAL_RECEIPT*` and every `src/loaded_cmj/v2/*`.
2. **Recompute independently:** For each claim requiring a trace, launch a fresh `V2Plant` + `V2EventDetector` + committed `controller.py` replay (`timestep 0.000125`, 40 substeps/control, 8.0 s horizon), compare `event_records`, `whole_Fz`, `qpos/qvel` to receipt, and compute `efc_force`/`Jᵀλ` for root limits and `qfrc_inverse[0:3]` for inverse residuals.
3. **Apply statuses:** `ORIGINAL_EVIDENCE_VERIFIED` iff bundle exists, hash matches, and fresh replay reproduces within declared tolerance; `CONTRADICTED` iff recomputation disagrees and the document's claim is materially false; `SUPERSEDED` iff correctly replaced by a later sealed authority; `EVIDENCE_MISSING` iff no bundle exists; `RETROACTIVE_REPRODUCTION_ONLY` never used — no historical bundle was recreated and labeled original.

---

## 3. R5B Receipt Adjudication (Unaudited Claim Document)

The supplied R5B receipt was treated strictly as a claim document. No raw trace bundle exists at the claimed path. Internal inconsistencies explicitly adjudicated:

### 3.1 Non-SHA strings stored in SHA256 fields
- **Finding:** Several `*_SHA256` fields contained truncated, uppercase, or non-hex placeholders (e.g., `71a0d045...` without full 64-lower-hex, `a1b2c3` placeholder in `LCMJ-V2-PLANT-CONTROLLER-CODESIGN-20260828` 02_V2_BODY_MODEL).
- **Adjudication:** `CONTRADICTED`. Correct SHA256 is 64 lowercase hex. All ledger entries in `AUTHORITY_LEDGER.json` now contain mechanically computed `sha256sum` values (see table §4). No placeholder is accepted as a hash.

### 3.2 4 s step-count consistency
- **Claim ledger:** `EPISODE_HORIZON_S 4.0` with `EPISODE_CONTROL_STEPS 800` implies `32000` physics steps, but some receipts report `64000` steps or `1200` controls without updating `constants.py` or horizon.
- **Adjudication:** `CONTRADICTED`. Honest consistent count is `horizon 8.0 / 0.000125 = 64000` steps, `1600` controls. Horizon extensions (`4.0→6.0→8.0`) were transparently recorded in `constants.py` (RES-10 R2, HEAD `V2_EPISODE_HORIZON_S=8.0`) — where not recorded, the claim is stale.

### 3.3 Static force balance of the proposed capture equilibrium
- **Claim:** Capture equilibrium "unique" with `MAX_UNACTUATED_INVERSE_RESIDUAL 181.48` and `qacc=0` static balance.
- **Recomputation:** `V2Plant.mj_inverse` at `qvel=0 qacc=0` along the 17-node quintic path: `root_norm` peaks at `357.72` (alpha 0.9375), not `181.48` (capture-only). Tolerance `200` is violated at 3 interior nodes (`220, 267, 357`). Stand is `0.15` — correct.
- **Adjudication:** `CONTRADICTED` if presented as a hard gate. Correct statement (adopted in ledger): static `mj_inverse` is **diagnostic only**; acceptance is 0.5 s forward bilateral hold (margin>0, finite, no reflight/fall). Tolerance `200` is endpoint-only; intermediate nodes require predeclared `600` if they are to be gated.

### 3.4 Root residual tolerance provenance
- **Finding:** Documents state `tolerance 200` but claim `all 17 PASS` while 3 nodes exceed it; `FINAL_RECEIPT` mentions expanded `600` without predeclare.
- **Adjudication:** `CONTRADICTED` until normalized. `RES10_R3` `08_STATIC_NODE_QUALIFICATION` cannot be `PASS` under its own stated tolerance. Normalized: diagnostic-only, forward-hold is authority. See `VVUQ_AND_QUALIFICATION_TAXONOMY.md` §3.1.

### 3.5 Random / undeclared experiments
- **Finding:** Some ladder/sweep receipts (e.g., `RES-6` solref ladder) were described as "random search" but are deterministic `0.004→0.020` `T1..T6` with bounded budget; other controller hunks mixed `SUPPORTED`/`FLIGHT`/`LANDING_PREP`/`IMPACT`/`CAPTURE` without clean labeling.
- **Adjudication:** `RES-6` is **not** random — it is declared deterministic offline `geom_solref` mutation with `OFFICIAL_MJENBL_OVERRIDE_USED=NO` and static MJCF authority, so `ORIGINAL_EVIDENCE_VERIFIED`. Undeclared random search in controller tuning (had it occurred) would be `CONTRADICTED`, but the sealed controllers document their budgets (`5` canonical candidates, `4` recovery durations, `16` supported params, `500` max) — therefore not random per `EXPERIMENT_PROTOCOL.md`.

### 3.6 COM-vs-root velocity provenance
- **Finding:** Some pro forma reports conflate `com_velocity_mps[2]` with `root_tz` qvel. `TAKEOFF_VZ` and `propulsion` dwell are defined by COM vz (Plant-owned), not by `qpos[root_tz]`.
- **Adjudication:** Correct provenance is **Plant `center_of_mass_velocity`** (as in `RES10_R3` integrity forensic). Where `R5B` reports "root velocity" for a COM gate, adjudicate `CONTRADICTED` and re-derive from `Plant.center_of_mass_velocity`. The recomputed `COM vz` at takeoff `1.164` (R5A2 honest) differs materially from root-tz rate.

### 3.7 Actuator-utilization consistency
- **Finding:** Utilization fields mix `static hold 0.374` with `landing transient hip 1.0` and previously `0.42` legacy prose.
- **Adjudication:** `CONTRADICTED` if collapsed. Normalized: `STATIC_CAPTURE 0.374` (hold), `LANDING_TRANSIENT 1.0` (impact). Legacy `0.42` superseded by committed `Kd=2ζ√(Kp·Ieff)` values (`Ieff 7.75/1.81/...`, IMPACT `Kd 99.61/50.42/24.81/2.87`, CAPTURE `109.57/55.46/27.29/3.16`). `LIMITS [250,250,250,300,300,200,200]` are the authority.

### 3.8 "Unique equilibrium" claim basis
- **Finding:** "Unique equilibrium" asserted without set verification (no exhaustive search, no basin proof).
- **Adjudication:** `CONTRADICTED`. At most "selected capture among qualified family via high-damped settle with bilateral/margin/prohibited checks, maximized support margin`0.104` for standing`0,0,0,0,0,0,0` — not a uniqueness proof." Unique-equilibrium language is struck.

---

## 4. Per-Milestone Adjudication

| Milestone | Commitment | Evidence Path | Reproducible? | Status | Caveat |
|---|---|---|---|---|---|
| **RES-5** | `6f03dad` honest fall | `.../RES5-HONEST-FALL...` (9 files) | YES (`0.0` diff before 3.117 s) | **ORIGINAL_EVIDENCE_VERIFIED** | Root limit forensics prove `ARTIFICIAL_LIMIT_SUPPORT_CONFIRMED` before, `0` rows after; mass 95 preserved |
| **RES-6** | `818985a` compliant contact | `.../RES6-COMPLIANT...` (10 files) | YES (static MJCF replay identical) | **ORIGINAL_EVIDENCE_VERIFIED** | Ladder `0.004→0.016` deterministic, `PRIMARY_PEAK 5.05 BW`, impulse residual `-0.5 N·s` |
| **RES-7** | `3542d49` event DAG | `.../RES7-EVENT-DAG...` (12 files) | YES (physics identity PASS) | **ORIGINAL_EVIDENCE_VERIFIED** | Manifest `ce6473f...` unchanged; monotone DAG proven |
| **RES-8** | `63d18e0` captured squat | `.../RES8-CAPTURED...` (11 files) | YES (`18/18`) | **ORIGINAL_EVIDENCE_VERIFIED** | Gains via `Kd=2ζ√(Kp·Ieff)`; `E10 2.321/E11 2.785` before recovery |
| **RES-16** | `91b79a5` true standing (old) | `.../RES16-TRUE-STANDING...` (14 files) | YES (15200/15200) | **ORIGINAL_EVIDENCE_VERIFIED** but **SUPERSEDED** by RES-43 | `01e89ac0...` invalidated by zero damping |
| **RES-31** | `fa2a8f6` honest planar root | `.../RES31-HONEST-PLANAR...` (15 files) | YES (`0` limit rows after) | **ORIGINAL_EVIDENCE_VERIFIED** | `11/12 INCOMPLETE_HORIZON` deterministic after; capture pose stale flagged |
| **RES-42** | `8708829` zero root damping | `.../RES42-ZERO-ROOT-DAMPING...` (12 files) | YES (`damping 0` compiled) | **ORIGINAL_EVIDENCE_VERIFIED** | `STANDING_PHYSICAL_SUPPORT PASS`; `OLD_TRUE_STANDING_REFERENCE_STILL_VALID=NO` |
| **RES-43** | `9d97bc5` true standing rebase | `.../RES43-TRUE-STANDING-REBASE...` (14 files) | YES (`1.9 s` dwell) | **ORIGINAL_EVIDENCE_VERIFIED** | New `8b90eea8...` is current E12 contract |
| **R5A2** | `3995967` honest takeoff | `.../RES10-R5A2-HONEST-TAKEOFF...` (`FINAL_RECEIPT.json` + patch) | YES (`RUN1==RUN2==POSTCOMMIT 8a7c06c7`) | **ORIGINAL_EVIDENCE_VERIFIED** | `16`-param `SUPPORTED` residual only; blocked landing patch correctly excluded (`0 UNKNOWN`) |
| **R5A3** | *expected* landing requal | `.../RES10-R5A3*` | NO directory | **EVIDENCE_MISSING** | No bundle exists |
| **R5B** (supplied receipt) | *claim only* | fabricated `RES10_R3`/`R4` 12/12 traces | NO (11/12 & 10/12 deterministic fails) | **CONTRADICTED** | See §3; superseded by RES-31/42/43 + R5A2 |
| **E1-E11** | `2a5967d` HEAD (and ancestors) | per-RES receipts | YES (within `1e-12` until `0.648 s`) | **ORIGINAL_EVIDENCE_VERIFIED** (E1-E11) | E12 is **EVIDENCE_MISSING** for honest sealed recovery |

Counts: `ORIGINAL_EVIDENCE_VERIFIED=11` (RES-5/6/7/8/16-superseded/31/42/43/R5A2 + E1-E11 + V2 baseline archived), `RETROACTIVE_REPRODUCTION_ONLY=0`, `EVIDENCE_MISSING=2` (R5A3, honest E12), `CONTRADICTED=2` (R5B claim bundle, RES10_R3/R4 fabricated 12/12), `SUPERSEDED=2` (V2 baseline, RES-16 old envelope). Totals are consistent with the matrix CSV (row-level counts differ because E1-E11 are expanded per-event there).

---

## 5. Currently Claimed E1-E11 Authority — Disposition

All 11 pre-recovery events are `ORIGINAL_EVIDENCE_VERIFIED` under the current honest plant (zero limits, zero damping) through the honest frontier (`E5` before first root-limit, `E11 0.977` after head capture tuning). Specifics:

- `E1 0.00025`, `E2 0.162874`, `E3 0.381124`, `E4 0.54175`, `E5 0.551625`, `E6 0.616`, `E7 0.626`, `E8 0.734`, `E9 0.779/2.007`, `E10 0.827/2.321`, `E11 0.977/2.785` — all reproduced across RES-7 through R5A2 within MuJoCo `3.8.0` tolerance and pre-limit equivalence.
- `E12 stable_recovery` with true-standing predicate (`8b90eea8...`) is **not yet verified** for a sealed honest controller; frozen controllers produce `10/12` or `11/12 INCOMPLETE_HORIZON` (falls at `1.70` / `1.92` / `3.213` s depending on path). This is the correct `BLOCKED` after the rebase, not a missed bug.

---

## 6. Reclassification Actions

- **RES10_R3** (`ba5708c`) `RES10_R3_SUPPORT_FRAME_WHOLE_BODY_RECOVERY` **PASS** → **CONTRADICTED + SUPERSEDED**. `11/12` deterministic; `MATERIAL_SUPPORT` present; inverse/foot/duration/margin claims false. Preserved historically but not current authority.
- **RES10_R4** (`1df985b`/`2a5967d` lineage) **PASS** → **CONTRADICTED** (R4A shows 10/12 `PHYSICAL_FALL` with material root damping, chatter `YES`, `moment residual 452`).
- **V2 baseline** (`fffb98e`) → **SUPERSEDED** (archived per `00_V1_ARCHIVE_BOUNDARY.md`).
- **RES-16 old envelope** (`01e89ac0`) → **SUPERSEDED** by `8b90eea8`.
- All other `PASS` milestones remain `ORIGINAL_EVIDENCE_VERIFIED` and are the current authority.

---

## 7. No Manufacture Declaration

No historical evidence was recreated. All "reproduction" traces cited are fresh deterministic replays written to `/tmp/repro_*` and are not persisted as `evidence/repository` historical bundles. The only new evidence produced in this mission is the pipeline qualification bundle `LCMJ-SCIENTIFIC-REBASE-20260903` (see `EVIDENCE_CONTRACT.md`), whose receipt is mechanically generated and whose bundle is delivered with `stat`/`sha256sum`/inventory proofs.

---

## 8. Next Authorized Scientific Unit

`RES-10 REBUILD THE HONEST CONTROLLER FROM CAPTURE THROUGH TRUE-STANDING E12` on the frozen honest plant (`5f22441...`), using the sealed `RES-43` E12 envelope (`8b90eea8`) and the sealed `R5A2` launch (`8a7c06c7...`) as entry — with a predeclared experiment (budget/metrics/hard gates, see `EXPERIMENT_REGISTRY.jsonl`). No mechanical Plant/contact/solver change is authorized. No `R5A4`/`E12` rendering/push until `EVIDENCE_DELIVERED` + `EVIDENCE_AUDITED` + `SEALED`.


---

## 9. R0.1 Addendum — R5A3 Historical vs Reproduction Split (2026-09-03)

`LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION` corrects the R5A3 evidence
classification without manufacturing history:

- `R5A3_ORIGINAL_EVIDENCE_STATUS=EVIDENCE_MISSING` — the original historical
  R5A3 bundle was not recovered (unchanged from §4).
- `R5A3_CURRENT_REPRODUCTION_STATUS=RETROACTIVE_REPRODUCTION_ONLY` — the
  scientific-rebase primary bundle reproduced the R5A3 trajectory from commit
  `2a5967d359f34562a9e356c3b138062cffdf4d51`; this is present
  reproducibility only (see `CLAIM_EVIDENCE_MATRIX.csv`
  `RES10_R5A3_RETROACTIVE_REPRODUCTION`).
- R5A3 is NOT upgraded to `ORIGINAL_EVIDENCE_VERIFIED` / `SEALED`.
- `R5B` remains `CONTRADICTED`; no causal conclusions revived.

No controller, Plant, contact, actuator, solver, or scorer semantics changed
in this addendum.
