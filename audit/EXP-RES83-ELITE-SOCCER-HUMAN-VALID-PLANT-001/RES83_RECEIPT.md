# RES83_RECEIPT — Elite-Soccer Human-Valid Loaded-CMJ Plant (V3)

MISSION: `RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001`
LINEAR_ISSUE: `RES-83` (parent of RES-95; milestone M6)
STATUS: **COMPLETE — V3 PLANT IMPLEMENTED, ALL DETERMINISTIC GATES PASS**

---

## 0. Resumption adjudication (RES83_RECOVER_ADJUDICATE_AND_SEAL_V3_PLANT_001)

An interrupted implementation session was recovered in the working tree,
forensically preserved outside the repository, independently reproduced and
then sealed.  Preservation snapshot (created before any mutation):

| Item | Value |
|---|---|
| RECOVERY_ARCHIVE | `/home/litju/Projects/loaded-cmj-control-evidence/RES83_INTERRUPTED_RECOVERY_20260915T022645Z.tar.gz` |
| RECOVERY_ARCHIVE_SHA256 | `7d73815ef325f1db696ec62c306a0273d73b6dfae758d6d98c8d1511cd756a2d` |
| Contents | `src/loaded_cmj/v3/`, `tests/test_res83_v3_plant.py`, `audit/EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001/` |

Independent adjudication (does not use this receipt's own claims):

- 276 independent checks (authority–model conformance, de Leva/Matsumoto
  inertia recomputation, independent HAT recomputation from `AUTHORITY_INPUTS`,
  composite-bar recomputation, HAT+bar parallel-axis recomputation, JC-09 FK
  sign probes, root passivity/ballistic flight, plantar identity/sole plane,
  prohibited-floor probes, compiled exclusion decode, pose legality, MTP
  topology, import boundary, stale-token and claim scan): **276/276 PASS**.
- Recovered evidence JSON is byte-identical to a fresh regeneration from the
  recovered implementation; two consecutive regenerations are byte-identical
  (no scientific nondeterminism).
- One genuine defect was found and repaired (section 8).

---

## 1. Entry authority (reconciled before any change)

| Item | Value |
|---|---|
| ENTRY_HEAD | `3e5e1e4075c469e7eb1164090c8df55f2a5cc598` |
| ENTRY_TREE | `75352cfedfa130718719cf1b686d59cde6fadfb8` |
| BRANCH | `main` |
| ORIGIN/MAIN at entry | `3e5e1e4075c469e7eb1164090c8df55f2a5cc598` (verified equal) |
| TRACKED WORKTREE at entry | clean (0 modified tracked files) |
| Pre-existing untracked paths | 26 entries (43 individual files) preserved untouched |
| RES-95 model authority | `LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1`, status `FROZEN_FOR_RES83_IMPLEMENTATION`, in-repo bundle `audit/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001/` |
| RES-95 external seal | `EVIDENCE_SEAL_SHA256 = c02374e4cd31777d14a65f6b78cebc8075097812be265c98690aada61d8d3952` (reconciled) |
| RES-95 bundle hash manifest | recomputed at run time: every listed file matches (see `AUTHORITY_BUNDLE_INTEGRITY.json`, 37 checks PASS) |

No V1 or V2 artifact, controller, scorer, test or experiment file was modified.
The only changed/added paths are the V3 surface, the RES-83 test module and the
RES-83 evidence directory.

## 2. Deliverables (tracked)

### Implementation surface (new, distinct successor — V1/V2 untouched)

| Path | Content |
|---|---|
| `src/loaded_cmj/v3/__init__.py` | V3 package surface |
| `src/loaded_cmj/v3/constants.py` | RES-95 authority transcription + provisional-numerical registry |
| `src/loaded_cmj/v3/plant.py` | runtime surface: identity assertion, pose library, plantar patch geometry, contact classification, MTP passive control |
| `src/loaded_cmj/v3/assets/v3_plant.xml` | sealed Plant: 14 bodies, 12 joints, 12 qpos/dof, 9 actuators, 17 geoms |

### Tests (new)

`tests/test_res83_v3_plant.py` — all 25 mandatory RES-83 tests plus 4
supplementary gates; every test asserts the same computation that is archived
in the evidence bundle.

### Evidence (new)

`audit/EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001/`

| Artifact | Content |
|---|---|
| `PLANT_IMPLEMENTATION_SPEC.json` | model identity, frames, joint order, collision classes, provisional-numerical register, pose library, explicit omissions |
| `MODEL_INTROSPECTION.json` | compiled identity and full body/joint/geom/actuator tables |
| `MASS_INERTIA_AUDIT.json` | per-body mass/COM/inertia against AB/UB/LB/FM authority; 79/99 kg closure |
| `HAT_BAR_REALIZATION_AUDIT.json` | HAT full tensor (Ixz preserved), bar composite surrogate, bar offset, HAT+bar combined realization |
| `JOINT_FK_SIGN_AUDIT.json` | joint address/order/axis/sign/range audit + 8 deterministic JC-09 FK probes with exact rigid predictions |
| `ROM_REACHABILITY_AUDIT.json` | structural envelope reachability, hard-bound activation, reverse-knee impossibility |
| `REPRESENTATIVE_POSE_AUDIT.json` | 10 deterministic configurations (standing → flight → landed/recovered), contacts, penetration, COM-over-support |
| `COLLISION_MATRIX.json` | declared + empirically probed body-pair matrix, bit classes, exact exclusions with rationale, prohibited-floor reachability probes |
| `ROOT_PASSIVITY_AUDIT.json` | zero root passives, no constraints/tendons/mocap, ballistic and pitched flight probes |
| `PLANTAR_REGION_IDENTITY_AUDIT.json` | heel/forefoot/toe identity, dimensions, sole plane, contact registration |
| `AUTHORITY_BUNDLE_INTEGRITY.json` | RES-95 bundle SHA-256 manifest re-verification |
| `IMPORT_BOUNDARY_AUDIT.json` | AST scan (no V1/V2/R001 controller or trajectory import/identifier) plus a fresh-interpreter proof that the Plant instantiates with only `loaded_cmj.v3` modules; pre-existing eager root `__init__` side effect disclosed, not a V3 dependency |
| `FINAL_CONSTANT_AUDIT.json` | no RES-84/RES-85 final constants; provisional registry; zero-passive variant |
| `AUTHORITY_CONFORMANCE_MATRIX.json` | decision-by-decision conformance (PT/AB/UB/LB/FM/JC/CC/SL/RA/DF) |
| `PLANT_VALIDATION_REPORT.json` | aggregate gate report |
| `HASH_MANIFEST.json` | SHA-256 of every evidence artifact, V3 source, XML and test |
| `build_evidence.py`, `seal_evidence.py` | reproducible build/seal scripts |
| `RES83_RECEIPT.md` | this receipt |

## 3. Key realized numbers

| Quantity | Realized value |
|---|---|
| MODEL_REVISION / MODEL_ID | `loaded-cmj-model-3` / `loaded-cmj-20kg-elite-soccer-v3` |
| Topology | NBODY=14, NJNT=12, NQ=NV=12, NU=9, NEQ=0, NA=0, NGEOM=17 |
| Athlete / bar / system mass | 79.0 / 20.0 / 99.0 kg |
| HAT mass / COM / inertia | 38.7969 kg; (−0.015964566, 0, 0.449673876) m; Ixx=1.692352492, Iyy=1.475358908, Izz=0.850425056, Ixz=−0.020779449 kg m² |
| HAT+bar system Iyy sensitivity range | [1.7316734138069028, 2.0243773264768694] kg m² (RES-95 `hat_sensitivity`, recomputed from the authority's own cases) |
| Bar composite | shaft 1.370 m × 0.028 m (6.821547881 kg) + 2 sleeves 0.415 m × 0.050 m (6.589226060 kg each); ρ_eff=8086.42235 kg/m³; I_transverse=11.755856968, I_axis=0.004786778 kg m² |
| Bar placement in HAT frame | (−0.095, 0, 0.600) m, identity orientation, welded (no bar DOF) |
| HAT+bar system | 58.7969 kg; COM (−0.0428488, 0, 0.5008079) m; Iyy=1.860804824 kg m² |
| Limb geometry | thigh 0.2425H=0.4449875 m; shank 0.2493H=0.4574655 m; foot 0.275 m; ankle height 0.071565 m; MTP at 0.20625 m |
| Standing geometry | root_tz = 0.974018 m (AB-13 closure overshoot absorbed, lengths never rescaled) |
| ROM (rad) | trunk ±0.610865; hip [−0.349066, 2.268928]; knee [0, 2.443461]; ankle [−0.959931, 0.785398]; MTP [−0.523599, 1.570796] |
| Joint axes | trunk +y; hip −y; knee +y; ankle −y; MTP −y (JC-08) |
| MTP passive prior | k=25 N·m/rad, c=2 N·m·s/rad, neutral 0.0 (FM-09), zero-passive variant representable with the active channel present |
| Actuator channels | 9 net-moment motors, gear=1, no torque/rate/power limits frozen (RES-85 owns) |
| Collision classes | floor(1/6), plantar support(2/7), prohibited(4/7); 12 explicit exclusions + 5 weld-aware parent filters; 61 enabled dynamic pairs |
| Contact classification | plantar-floor condim=4; other condim=3; sliding friction nominal 0.9 (sweep [0.5, 1.5]); torsional/rolling deferred (RES-84) |

## 4. Verification executed before commit

| Gate | Result |
|---|---|
| `PLANT_VALIDATION_REPORT.json` | 14 artifacts, 0 failed checks, status PASS |
| `pytest tests/test_res83_v3_plant.py` | 31 passed (25 mandatory + 6 supplementary) |
| `pytest tests/` (full suite, regression) | see `POSTCOMMIT_SIDECAR` note in section 6; no existing test touched or regressed |
| `ruff check src/loaded_cmj/v3 tests/test_res83_v3_plant.py` | clean |
| RES-95 bundle integrity | 37/37 checks PASS (hash manifest + authority id) |
| Independent verification | see section 5 |
| Tracked worktree | only the RES-83 additions are staged/committed |

## 5. Independent verification (validator independence)

The implementation unit executed the deterministic gates above; an independent
verification pass was run afterwards by a separate read-only LCMJ
`lcmj-independent-verifier` agent with no implementation edit rights. The
verifier receipt (identity, scope and verdict) is recorded in the RES-83
Linear closure comment and in the external `POSTCOMMIT_SIDECAR.json`; it is
not embedded here so that the in-repo manifest cannot be silently rewritten by
the implementer after verification.

## 6. Evidence bundle and seal convention

- Repository bundle: `audit/EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001/`
- External evidence root: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES83-ELITE-SOCCER-HUMAN-VALID-PLANT-001/`
  - `checksums.sha256` excludes `checksums.sha256`, `SEAL.json`, `POSTCOMMIT_SIDECAR*`;
  - `EVIDENCE_SEAL_SHA256 = sha256(checksums.sha256 bytes)` recorded in `SEAL.json`;
  - `POSTCOMMIT_SIDECAR.json` records `FINAL_HEAD`, `FINAL_TREE`, remote sync and worktree state after the commit.
- `HASH_MANIFEST.json` in this directory covers every evidence artifact, the V3
  source files, the XML and the test module; it is regenerated by
  `python3 build_evidence.py`.

## 7. Commit

Commit message: `V3: implement elite-soccer human-valid loaded-CMJ Plant`

`FINAL_HEAD`, `FINAL_TREE`, `REMOTE_SYNC` and `TRACKED_WORKTREE_CLEAN` are recorded in the
external `POSTCOMMIT_SIDECAR.json`, per the RES-82/RES-95 convention (the in-repo
receipt cannot contain post-commit hashes without invalidating its own manifest entry).

## 8. Resumption repair record (RES83_RECOVER_ADJUDICATE_AND_SEAL_V3_PLANT_001)

One real defect was found by the resumption adjudication and repaired:

| Item | Detail |
|---|---|
| Defect | `V3_SYSTEM_IYY_SENSITIVITY_RANGE_KG_M2` upper bound was transcribed as `2.0243773266743828`; sealed RES-95 `DERIVED_QUANTITIES.hat_sensitivity.system_iyy_max` is `2.0243773264768694` |
| Class | authority transcription error in a declared sensitivity quantity (no physics/compiled-model effect) |
| Authority ruling | RES-95 wins; correction unambiguously implied by `DERIVED_QUANTITIES.json` |
| Repair | constant corrected; `build_evidence.py` now anchors the realized HAT inertia, bar inertia and the declared sensitivity range directly to `DERIVED_QUANTITIES.json` (`hat_full_inertia_matches_res95_derived`, `bar_inertia_matches_res95_derived`, `system_iyy_sensitivity_range_matches_res95`), recomputing the range from the authority's own sensitivity cases |
| Regression test | `test_system_iyy_sensitivity_range_matches_res95`, `test_hat_bar_inertia_matches_res95_derived` |
| Re-verification | full RES-83 qualification re-run after repair: 31/31 tests, builder PASS, 276/276 independent checks, two byte-identical regenerations |

The resumption also hardened the import-boundary claim: the pre-existing
repository-root `src/loaded_cmj/__init__.py` eagerly imports
`loaded_cmj.runtime.engine`, so a plain `import loaded_cmj.v3.plant` loads V1/V2
runtime packages as a package-initialization side effect.  That file is tracked
before RES-83 and unmodified; it is disclosed in
`IMPORT_BOUNDARY_AUDIT.json.package_init_disclosure` and the V3 Plant is proven
in a fresh interpreter (stub root package) to load only `loaded_cmj.v3`
modules with zero controller/runtime dependencies.

All other numeric literals in `constants.py` (143) and `v3_plant.xml` (163)
were machine-swept against the sealed RES-95 bundle; the remaining
non-verbatim literals are verified body-frame derivations or comment
roundings, not stale authority.  No RES-84/RES-85/controller/trajectory value
is present in the RES-83 surface.

## 9. Claim ceiling

This receipt establishes the sealed RES-95 Plant topology/geometry/inertia
realization and its deterministic structural gates only. It establishes no
contact calibration (RES-84), no actuator/controller science, no elite H2
mapping or performance gate (RES-85/RES-91), no candidate (RES-90), no full
CMJ execution, no human predictive validity and no population norm.

`NEXT_AUTHORIZED_ACTION=RES84_MEASUREMENT_CONTACT_AUTHORITY`
