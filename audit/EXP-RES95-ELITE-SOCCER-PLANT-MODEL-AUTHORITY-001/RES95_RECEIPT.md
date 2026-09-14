# RES95_RECEIPT — Elite-Soccer Successor Plant Model Authority

MISSION: `RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001`
LINEAR_ISSUE: `RES-95` (parent RES-83, milestone M6)
STATUS: **COMPLETE — MODEL AUTHORITY FROZEN, BUNDLE SEALED, DEDICATED COMMIT CREATED**

---

## 1. Entry authority (reconciled before any change)

| Item | Value |
|---|---|
| ENTRY_HEAD | `4ce32eef873c5b07d9130663865528932ce4e3b9` |
| ENTRY_TREE | `e81027a4c6cd23ff263b5a104f4e8310de2ed327` |
| BRANCH | `main` |
| ORIGIN/MAIN at entry | `4ce32eef873c5b07d9130663865528932ce4e3b9` (verified equal) |
| TRACKED WORKTREE at entry | clean (0 modified tracked files; 26 pre-existing untracked paths excluded from this achievement) |
| RES-82 authority commit | `4ce32eef873c5b07d9130663865528932ce4e3b9` (= ENTRY_HEAD; RES-82 task contract sealed) |
| RES-82 receipt SHA-256 | `bcd8f7a76b9e20657b4450ac63297a72a29383c84e474cf6279ea8bdb38ec0be` |
| RES-82 evidence seal | `487b0c813a01b6541f89e340ce5515211d49edaf4c3da0e3284858a90c33ecc2` (external `SEAL.json` SHA-256 `c2cc809b4f56a5bf41eecd2f46901397237766974792636bf1ad9836fff24729`) |
| Controlling upstream authority | Linear document `LCMJ_ELITE_SOCCER_SYSTEM_AUTHORITY_V1` (fetched 2026-09-14, including the adversarial corrections) and the latest RES-95 Linear comment `NEXT AUTHORIZED UNIT — RES-95 repository closure after adversarial re-audit` |
| Active Linear unit | RES-95 (In Progress) — confirmed |

No R001 or RES-80/81/82 artifact was modified.

## 2. Deliverables (tracked bundle)

`audit/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001/`

| Artifact | Content |
|---|---|
| `SPORT_CONTEXT_AUTHORITY.md/.json` | frozen context of use, population, task, evidence hierarchy, claim boundary |
| `REFERENCE_ATHLETE_SPEC.md/.json` | synthetic nominal athlete (1.835 m / 79.0 kg), 20 kg load, 99.0 kg system, RES-82 BW reconciliation |
| `ANTHROPOMETRY_BSIP_AUTHORITY.md/.json` | de Leva-adjust Zatsiorsky-Seluyanov BSIP, masses, geometry, closure delta, foot cross-check |
| `UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY.md/.json` | reduced HAT decomposition, frozen bar-hold posture, numeric HAT and HAT+bar inertia |
| `LOAD_BAR_AUTHORITY.md/.json` | IWF-style 20 kg composite bar geometry, inertial surrogate, collision embodiment, placement |
| `JOINT_COORDINATE_ROM_AUTHORITY.md/.json` | world/joint conventions, planar root, ROM envelope, analytic FK sign probes |
| `FOOT_MTP_MODEL_AUTHORITY.md/.json` | multi-segment foot, plantar patches, sole plane, foot inertia, passive/active MTP contract |
| `STANCE_LAB_CONTEXT_AUTHORITY.md/.json` | leg-plane geometry (not a stance-width claim), force-plate chain |
| `COLLISION_CONTACT_POLICY.md/.json` | legal/fall contact boundary, MuJoCo `condim`/friction/margin/gap/solref/solimp/penetration classification |
| `PLANT_TOPOLOGY_AUTHORITY.md/.json` | NBODY=14, NQ=NV=12, NU=9, bilateral symmetry and out-of-plane reaction audit |
| `EVENT_MEASUREMENT_BOUNDARY.md/.json` | TAKEOFF_OCCURRENCE vs CONFIRMATION, H2 anchoring, flight/apex/touchdown, comparators |
| `PERFORMANCE_AUTHORITY_BOUNDARY.md/.json` | anti-triviality floor 0.150 m; elite targets NOT_FROZEN; method-matched mapping deferred |
| `DEFERRED_NUMERICAL_CALIBRATIONS.md/.json` | every deferred numeric decision with owner and qualification domain |
| `AUTHORITY_INPUTS.json` | machine-readable inputs (de Leva Table 4 rows, posture, bar, foot, topology) |
| `DERIVED_QUANTITIES.json` | deterministic derived numbers and the 15-case HAT/bar sensitivity block |
| `MODEL_AUTHORITY_EVIDENCE_TABLE.csv/.md` | 22 sources + 136 classified decisions, provenance and sensitivity per row |
| `MODEL_AUTHORITY_REVIEW.md` | adversarial review, findings C1–N3 and dispositions, red-team scan, residual risks |
| `MODEL_AUTHORITY_HASH_MANIFEST.json` | SHA-256 of every bundle artifact (excluding manifest/validation report) |
| `build_authority_numbers.py`, `render_authority_bundle.py`, `seal_bundle.py`, `validate_authority.py` | reproducible build/seal/validation scripts |
| `AUTHORITY_VALIDATION_REPORT.json` | generated gate report (G1–G9) |

## 3. Key frozen numbers (reproducible from `AUTHORITY_INPUTS.json`)

| Quantity | Value |
|---|---|
| Reference athlete / load / system | 1.835 m; 79.000 kg; 20.000 kg; 99.000 kg; 971.19 N; ratio 0.25316456 |
| Thigh / shank (normative) | 0.2425·H = 0.4449875 m; 0.2493·H = 0.4574655 m |
| HAT mass / COM / inertia (Ixx, Iyy, Izz, Ixz) | 38.7969 kg; (−0.0159646, 0, 0.4496739) m; (1.692352492, 1.475358908, 0.850425056, −0.020779449) kg m² |
| HAT principal moments / principal rotation | (1.692865034, 1.475358908, 0.849912514) kg m²; −1.41296° about y |
| Bar composite (rho_eff, shaft, sleeve) | 8086.42235 kg/m³; 6.821547881 kg; 6.589226060 kg |
| Bar inertia (transverse, axial) | 11.755856968 kg m²; 0.004786778 kg m² |
| Bar local placement in HAT frame | (−0.095, 0, 0.600) m, identity orientation |
| HAT+bar system mass / COM / inertia (Ixx, Iyy, Izz, Ixz) | 58.7969 kg; (−0.0428488, 0, 0.5008079) m; (13.746432717, 1.860804824, 12.688717904, 0.136014355) kg m² |
| Bar-placement sensitivity (system Iyy range) | 1.731673414 … 2.024377326 kg m² |
| Foot segment masses / MTP / toe length | 0.4675536 / 0.4588952 / 0.1558512 kg; 0.20625 m; 0.06875 m |
| Ankle height / sole plane | 0.071565 m; z = −0.071565 m in foot frame |
| Segment chain closure delta | +0.030905995 m (declared, not silently repaired) |
| Topology | NBODY=14, NQ=NV=12, NU=9 (trunk-pelvis + bilateral hip/knee/ankle/MTP) |
| Event semantics | TAKEOFF_OCCURRENCE = final legal plantar contact loss; confirmation must not shift it; flight ≥ 0.050 s; apex interpolated |
| Performance | H_ANTI_TRIVIALITY_FLOOR = 0.150 m; ELITE_SOCCER_PLUS20_DIRECT_H2_TARGET = NOT_FROZEN; HARD_GATE = DEFERRED_PENDING_METHOD_MATCHED_MAPPING |

## 4. Verification executed before commit

| Gate | Result |
|---|---|
| Independent numeric replication (separate agent, own implementation, 55 quantities + 15 sensitivity cases) | `REPLICATED`; max relative difference 1.3e-16 |
| `validate_authority.py` gates G1–G7, G9 | PASS (239 checks; see `AUTHORITY_VALIDATION_REPORT.json`) |
| `validate_authority.py` gate G8 (hash manifest) | PASS after sealing |
| Red-team stale scan (10 mission classes) | 0 unmarked occurrences; all hits supersession-marked |
| Independent adversarial audit | all CRITICAL/HIGH/MEDIUM/LOW/NIT findings closed (`MODEL_AUTHORITY_REVIEW.md` §3) |
| Production/controller/scorer/test/experiment files | unchanged (`git status` tracked clean; no denylisted path touched) |

## 5. Evidence bundle and seal convention

- Repository bundle: `audit/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001/`
- External evidence root: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES95-ELITE-SOCCER-PLANT-MODEL-AUTHORITY-001/`
  - `checksums.sha256` excludes `checksums.sha256`, `SEAL.json`, `POSTCOMMIT_SIDECAR*`;
  - `EVIDENCE_SEAL_SHA256 = sha256(checksums.sha256 bytes)` recorded in `SEAL.json`;
  - `POSTCOMMIT_SIDECAR.json` records `FINAL_HEAD`, `FINAL_TREE`, remote sync and worktree state after the commit.

## 6. Commit

Commit message: `V3: freeze elite-soccer successor Plant model authority`

`FINAL_HEAD`, `FINAL_TREE`, `REMOTE_SYNC` and `TRACKED_WORKTREE_CLEAN` are recorded in the external
`POSTCOMMIT_SIDECAR.json` and in the RES-95 Linear closure comment, per the RES-82 convention
(the in-repo receipt cannot contain post-commit hashes without invalidating its own manifest entry).

## 7. Linear status note (M6)

The controlling Linear document currently carries the pre-closure status label
`ADVERSARIAL RESEARCH CORRECTION OPEN — NOT YET SEALED FOR IMPLEMENTATION`. Per the latest
RES-95 comment, the repository-tracked bundle, hash manifest, evidence table, receipt and
dedicated commit are the closure gate for that label. After this commit is verified and recorded,
RES-95 is moved to **Done** and the next authorized action is
`NEXT_AUTHORIZED_ACTION=RES-83_HUMAN_VALID_PLANT_IMPLEMENTATION`.

## 8. Claim ceiling

This receipt freezes specification only. It establishes no Plant implementation, no controller
result, no candidate, no human predictive validity and no population norm. Plant implementation,
controller work, contact calibration, candidate authorization and qualification remain owned by
RES-83/84/85/86/87/88/89/90/91 as declared in `DEFERRED_NUMERICAL_CALIBRATIONS.md`.
