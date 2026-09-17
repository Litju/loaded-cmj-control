# RES85A_AUTHORITY_RECEIPT — RES-85 Scientific Control Authority Freeze

MISSION: `RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001`
LINEAR ISSUE: RES-85
ACHIEVEMENT: **A — FREEZE RES-85 SCIENTIFIC CONTROL AUTHORITY**
STATUS: `PASS` (`AUTHORITY_CHECK_REPORT.json`, `AUTHORITY_REDTEAM_REPORT` content in the
same report)

## 1. Immutable inputs reconciled at mission start

| Input | Value | Reconciliation |
|---|---|---|
| ENTRY_HEAD | `b0eccb8a6eeda40950665b3be517854f8b48baab` | matches mission checkpoint |
| ENTRY_TREE | `62b5d5d7a20f4ac50090ab7602723a6cd373561c` | matches mission checkpoint |
| origin/main | `b0eccb8a6eeda40950665b3be517854f8b48baab` | equal to HEAD |
| Modified tracked files | none | verified; pre-existing untracked owner files preserved |
| RES-83 Plant XML SHA256 | `eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d` | exact match |
| RES-84 evidence seal | `223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda` | exact match (external bundle digest-verified) |

The sealed V3 Plant (`src/loaded_cmj/v3/assets/v3_plant.xml`) and the RES-84
measurement/contact authority (`src/loaded_cmj/v3/measurement.py`) are immutable
inputs to RES-85. No Plant anatomy, inertial parameter, contact geometry,
measurement semantic, PHYSICAL_TIME dwell semantic, takeoff occurrence,
clearance guard, CoP/support semantic or diagnostic comparator was modified.

## 2. Frozen authority artifacts

| Artifact | Content |
|---|---|
| `METHOD_COMPARATOR_PANEL.json` | H2 method separation: direct simulator SYSTEM_COM (primary), force-platform impulse-momentum, force-platform flight-time, bar/LVT displacement/velocity; `ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED`; no cross-method conversion; `0.150 m` retained only as the labelled `HISTORICAL_ANTI_TRIVIALITY_NEGATIVE_CONTROL_ONLY` magnitude |
| `ACTUATION_AUTHORITY.json` | Nine channels (trunk_pelvis, left/right hip, knee, ankle, MTP) with moment, power and torque-rate ceilings; frozen enforcement order; saturation semantics with per-stage recording; bilateral symmetry enforcement; provenance class and sensitivity band per channel |
| `MTP_ENERGY_AUTHORITY.json` | Active/passive MTP moment definitions, separate power/work ledgers, total identity, per-foot budgets, supported-phase authority, phase gate, late-phase restriction, zero-passive sensitivity rule, anti-energy-injection rule `AEI-1` |
| `PHASE_MACHINE_AUTHORITY.json` | Seven-state causal machine `STAND → COUNTERMOVEMENT → BRAKING → PROPULSION → TAKEOFF_CONFIRM → FLIGHT → LANDING_PREP` with per-state observable inputs, exit predicates, hysteresis/debounce, failure/reversion and wall-clock role |

## 3. H2 authority

```
H2 = SYSTEM_COM_z(APEX) − SYSTEM_COM_z(TAKEOFF_OCCURRENCE)
```

* Origin event and apex definition are RES-84 primitives.
* `0.150 m` is **not** an elite-performance target, band or floor. It remains a
  historical anti-triviality / negative-control magnitude only.
* No method-matched conversion to a single elite +20 kg H2 target exists; the
  hard gate is withheld as `NOT_ESTABLISHED` and reporting is method-explicit.

## 4. Actuation authority summary (engineering nominal, sensitivity-banded)

| Channel | Moment ceiling (N·m) | Power ceiling (W) | Rate ceiling (N·m/s) |
|---|---|---|---|
| trunk_pelvis | 220 | 600 | 3000 |
| hip (each) | 330 | 1600 | 6000 |
| knee (each) | 380 | 1700 | 8000 |
| ankle (each) | 260 | 1100 | 6000 |
| MTP (each) | 45 | 120 | 1500 |

Every scalar is `ENGINEERING_NOMINAL_WITH_SENSITIVITY` with a declared sweep.
Dynamic jump/loading evidence is the primary class; isokinetic maxima are
`ISOKINETIC_CONTEXT_ONLY` and are explicitly prohibited as dynamic constants.
No V2 or R001 value is imported.

## 5. Red-team findings (all clean)

| Red-team question | Result |
|---|---|
| Copied R001 values | none (literal scan over all authority files) |
| Hidden height targets | none; `0.150` appears only as the labelled anti-triviality reference and its prohibition |
| Method mismatch | refused by construction: `NOT_ESTABLISHED`, single-target conversion `PROHIBITED` |
| Double-counted MTP energy | two separate ledgers with exact identities `P_total = P_active + P_passive`, `W_total = W_active + W_passive` |
| Asymmetric nominal control | symmetry projected and recorded for all four mirrored pairs in every RES-85 phase |
| Duplicated event definitions | prohibited; measurement primitives are consumed from `loaded_cmj.v3.measurement` only |
| Time-programmed primary transitions | none; wall-clock is debounce-only and five post-BRAKING states declare `wall_clock_role = none` |

## 6. Deterministic verification

`build_authority_checks.py` re-executes every check above and writes
`AUTHORITY_CHECK_REPORT.json`, `SOURCE_PROVENANCE.json` and `HASH_MANIFEST.json`.
`tests/test_res85_authority_freeze.py` asserts the same computation in pytest.
`SOURCE_PROVENANCE.json` records the hashes of the Plant XML, the measurement
module and the authority files, and pins the RES-83 XML hash and the RES-84
evidence seal as declared input facts.

## 7. Claim ceiling

This receipt freezes *authority*, not performance. It establishes no loaded-CMJ
jump height, no elite comparison and no Plant capability claim. Achievement B
implements and qualifies the causal controller under these limits; RES-86 owns
landing capture and RES-87 owns recovery.

---

## RES-85C erratum (appended; not part of the Achievement-A record)

The RES-85C Blocker-B correction supersedes the H2-floor framing used in
section 3 of this Achievement-A record.  The current authority in this bundle
(`METHOD_COMPARATOR_PANEL.json`) declares:

* `H_ANTI_TRIVIALITY_FLOOR = 0.150 m` with role
  `HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY`, i.e. a hard
  minimum functional success condition (not an elite norm and not an
  optimization target);
* `ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` (unchanged);
* `ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED` (restored).

Everything else in this Achievement-A record is unchanged history.  See
`RES85C_CORRECTION_RECEIPT.md` for the full correction.
