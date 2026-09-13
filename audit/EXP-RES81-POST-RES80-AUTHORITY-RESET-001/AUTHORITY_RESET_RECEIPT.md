# AUTHORITY RESET RECEIPT — RES-81

MISSION: `LCMJ_POST_RES80_REMEDIATION_PROGRAM_001`
LINEAR_ISSUE: `RES-81`
MODE: `AUTHORITY_RECONCILIATION_ERRATA_AND_ROADMAP_CLOSURE`

> **THE ORIGINAL RES-80 BUNDLE IS IMMUTABLE.** Nothing in this achievement edits, amends,
> squashes, or rewrites the sealed RES-80 audit commit or its evidence bundle. The errata
> are additive and supersede only the listed numeric/causal wording.

## Achievement

Post-RES80 authority reset: local Git reconciled with GitHub, the sealed RES-80 audit
commit published exactly, an additive RES-80 errata authority sealed, one durable
post-audit baseline recorded, all 53 RES-80 defects and all 50 successor requirements
mapped to roadmap owners, and the Linear dependency graph validated.

No Plant, controller, event/scorer, measurement, test, or packaging code was modified.
No successor candidate was created or tuned.

## Reconciled authorities

| Authority | Value |
|---|---|
| Historical R001 HEAD | `8f26736db1231042cbedc61f61ca4862b7be871c` |
| Historical R001 TREE | `855fe3b418c09b5028396adeaed3ba8cda51e89f` |
| RES-80 audit HEAD | `8c1cbb49c91ba9e6d504a1df63fba242efcf7c64` |
| RES-80 audit TREE | `79544558e6fff05d7cd22e3692d580127730a410` |
| RES-80 audit parent | `8f26736db1231042cbedc61f61ca4862b7be871c` |
| RES-80 evidence seal | `c921e6de29a263be1bdcee78c9fa62c22b968388991cda50afa34bf3260a9341` |
| R001 trace SHA-256 | `4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561` |

The RES-80 audit commit exists locally with an exact SHA/tree/parent/message match and
contains only `audit/EXP-RES80-.../` files. It was fast-forwarded to `origin/main` with
no merge, rebase, squash, or force push. The RES-80 bundle recomputed 43/43 checksums
with 0 mismatches and the recomputed seal equals the declared seal.

## Additive errata (RES80_ERRATA_001)

| ID | Subject | Severity before | Severity after | R001 effect |
|---|---|---|---|---|
| ERR-001 | Jump-height terminology | N/A | N/A | None |
| ERR-002 | Takeoff-definition ambiguity | N/A | N/A | None |
| ERR-003 | Action-difference timestamp/indexing | CRITICAL (CRIT-007) | CRITICAL (CRIT-007) | None |
| ERR-004 | Transition lead-time wording | N/A | N/A | None |
| ERR-005 | HIGH-003 one-sided balance causal wording | HIGH | HIGH | None (severity unchanged) |

Key recomputed values (full evidence in `RES81_RECOMPUTE.json`):

- `APEX_ABOVE_INITIAL_STANDING_COM = 0.03626536191815721 m`
- `TAKEOFF_TO_APEX_COM_RISE_E6 = 0.07515774782276163 m`
- `E6_COM_VZ = 1.2144603063933925 m/s`
- `BALLISTIC_HEIGHT_FROM_E6_VZ = 0.07517399774745835 m`
- `E6_TO_E9_DURATION = 0.22687500000007577 s`
- `MAX_DU = 0.9439067825713727` applied at `POST_ACTION_TIME = 0.6450000000000072 s` (was mis-timestamped 0.640 s)
- `FLIGHT_COMMAND_TO_SUSTAINED_TAKEOFF_MS = 1.75`, `FLIGHT_COMMAND_TO_E6_MS = 3.125` (old 8.125 ms wording retired)
- `SUSTAINED_TAKEOFF_START = 0.6467500000000078 s`; `LAST_FORCE_BEARING_RECONTACT = 0.6462500000000077 s`; `LAST_PHYSICAL_CONTACT_REGISTRATION_BEFORE_FLIGHT = 0.6477500000000082-0.6480000000000082 s`; `CANONICAL_E6 = 0.6481250000000083 s`
- HIGH-003 source proof: `FX_BRAKE_MIN=-120.0 < 0`, `FX_BRAKE_MAX=0.0`, `Fx_raw=-px/T_rem`; R001 vx 0.18142 -> 0.34183 peak (t=1.0385, raw demand -120.27 N) -> zero crossing 1.601125 -> min -0.058181 (t=2.117) -> final -0.001276 m/s

R001 disposition is unchanged: `HISTORICAL_REPRODUCIBILITY=PASS`,
`PHYSICAL_VISUAL_CREDIBILITY=FAIL`, `SHIP_STATUS=BLOCKED`. Nothing in this achievement
rescues the R001 success claim.

## Roadmap coverage

- Defects: 53/53 mapped, 0 orphans, 0 duplicate primary ownership
  (CRITICAL 7 / HIGH 19 / MEDIUM 20 / LOW 7).
- Primary owner histogram: RES-82=8, RES-83=4, RES-84=7, RES-85=3, RES-86=4,
  RES-87=5, RES-88=9, RES-89=5, RES-94=8.
- Successor requirements: 50/50 mapped, 0 orphans.
- Overlapping Linear "primary" claims adjudicated to exactly one primary owner for
  MED-011 (RES-84), HIGH-012 (RES-87), MED-013 (RES-82); rationale recorded in
  `RES80_DEFECT_TO_ROADMAP_MATRIX.json` (`OWNERSHIP_RESOLUTIONS`).
- Linear dependency graph RES-81..RES-94 validated against the intended graph: PASS.
  No relation edits were required.

## Deliverables

| File | Purpose |
|---|---|
| `RES80_ERRATA_001.md` / `.json` | Additive errata authority (5 records) |
| `POST_RES80_AUTHORITY_BASELINE.json` | Durable post-audit baseline bindings |
| `RES80_DEFECT_TO_ROADMAP_MATRIX.json` / `.md` | 53/53 defect coverage |
| `ROADMAP_REQUIREMENT_COVERAGE.json` | 50/50 requirement coverage |
| `RES81_RECOMPUTE.json` | Independent recomputation evidence |
| `recompute_errata.py` / `build_matrix.py` | Deterministic reproduction scripts |
| `AUTHORITY_RESET_RECEIPT.md` | This receipt |
| `GIT_RECONCILIATION.txt` / `REMOTE_SYNC_PROOF.txt` | Raw Git/remote proof (bundle) |
| `checksums.sha256` / `SEAL.json` | Bundle seal (bundle) |

## Commit gate

The RES-80 audit commit `8c1cbb49...` is a pre-existing achievement and was published,
not recreated. The new RES-81 achievement is sealed by exactly one dedicated commit
containing only `audit/EXP-RES81-POST-RES80-AUTHORITY-RESET-001/`. Its final HEAD/tree
are recorded in the postcommit sidecar; this file is not modified to embed its own SHA.

## Next authorized action

`RES-82_SCIENTIFIC_TASK_CONTRACT`

Do not start RES-82 from this achievement.
