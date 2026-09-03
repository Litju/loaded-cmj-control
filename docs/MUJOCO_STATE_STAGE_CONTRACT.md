# MuJoCo State-Stage Contract — Loaded CMJ V2.1

**Status:** NORMATIVE for all future project measurements (sealed by
`EXP-RES10-REC01A-SYNC-DYNAMICS-001`, mission
`RES10_REC01A_SYNCHRONIZED_DYNAMICS_AUTHORITY`).
**Scope:** MuJoCo 3.8.0, V2 Plant (`v2_plant.xml`), `implicitfast` integrator,
Newton solver (100/50, tol 1e-10), `dt=0.000125`, 40 substeps/control.
Verified on x86-64; re-verify on any version/arch/solver change.

---

## 1. Rule

A measurement record labeled with sample time `T` MUST contain quantities that
all belong to the dynamical instant `T`. The only sanctioned pipeline point is:

> **SYNCHRONIZED POINT:** full `mjSTATE_INTEGRATION` restored via `mj_setState`
> (or equivalently the live state immediately after reset/control-write),
> followed by `mj_forward`, with all derived quantities read AFTER that call.

Reading derived quantities from `mjData` immediately after `mj_step` and
labeling them with the incremented time `t+h` is **NOT sanctioned**: those
quantities belong to the pre-integration instant `t` (proven §3).

## 2. Field classification (MuJoCo 3.8.0, this Plant)

| Field | Class | After `mj_step`, belongs to |
|---|---|---|
| `time` | INTEGRATION STATE | `t+h` (current) |
| `qpos` (10) | INTEGRATION STATE | `t+h` (current) |
| `qvel` (10) | INTEGRATION STATE | `t+h` (current) |
| `qacc_warmstart` (10) | INTEGRATION STATE | solver warmstart for next solve (current) |
| `ctrl` (7) | INTEGRATION STATE (held input) | as written (current) |
| `qfrc_applied` (10), `xfrc_applied` (60) | INTEGRATION STATE | as written (current; always 0 in this Plant) |
| `qacc` (10) | FORWARD-DYNAMICS OUTPUT | **`t` (STALE by one step)** |
| `qfrc_bias` | DERIVED VELOCITY/FORCE QUANTITY | **`t` (STALE)** |
| `qfrc_passive` | DERIVED FORCE QUANTITY | **`t` (STALE)** |
| `qfrc_constraint` (`Jᵀλ`) | CONSTRAINT OUTPUT | **`t` (STALE)** |
| `qfrc_actuator` | DERIVED ACTUATION QUANTITY | recomputed from `ctrl` (equals current; safe) |
| `xpos`/`xmat`/`xipos`/`ximat` | DERIVED POSITION QUANTITIES | **`t` (STALE; ~7e-05 drift/step)** |
| `subtree_com` | DERIVED POSITION QUANTITY | **`t` (STALE)** |
| `cvel`/`cdof`/`cinert` | DERIVED VELOCITY QUANTITIES | **`t` (STALE; cvel ~0.07/step)** |
| `contact[]`, `ncon` | CONTACT OUTPUT (collision at `t`) | **`t` (STALE; misses transitions)** |
| `nefc`, `efc_type/id/force`, `efc_J` | CONSTRAINT OUTPUT | **`t` (STALE)** |
| `solver_fwdinv` | DIAGNOSTIC (internal fwd/inv compare) | current step (valid) |
| `solver_niter/nnz` | DIAGNOSTIC | current step (valid) |
| `qfrc_inverse` | INVERSE-DYNAMICS OUTPUT | defined only by `mj_inverse` call contract §4 |

INTEGRATION STATE = the 108-float `mjSTATE_INTEGRATION` vector
`[time(1), qpos(10), qvel(10), qacc_warmstart(10), ctrl(7),
qfrc_applied(10), xfrc_applied(60)]`. Everything else is DERIVED and MUST be
recomputed by `mj_forward` at the declared instant before being recorded.

## 3. Measured evidence (this mission)

Single-step forensics at the E9 contact transition (`t=0.779`, `0→4` contacts):

- `qacc` immediately after `mj_step` is **bit-identical** (`0.0` diff) to the
  pre-integration forward solve `qacc_fwd(t)`, and differs by `8.96` from the
  synchronized solve `qacc_fwd(t+h)`.
- `mj_forward` on the same data changes: `qacc` by `7.44`, `qfrc_constraint`
  by `5.45`, `efc_force` by `1.36`, `cvel` by `0.068`, `xpos` by `6.7e-05`,
  COM by `5.4e-05`; `qpos/qvel/ctrl/time` unchanged (`0.0`).
- At the same transition, post-step `ncon=0` while synchronized `ncon=4`:
  contact detection lags one full step.

Production impact (canonical prefix, current vs synchronized):

- Every event E1–E11 fires exactly **one physics sample late** (`−0.000125 s`
  synchronized-minus-current uniformly): systematic stage lag, not noise.
- Max deltas: COM position `1.47e-04 m`, COM velocity `1.32e-02 m/s`,
  whole `Fz` `5439 N`, CoP `0.16 m`, trunk tilt `4.1e-04 rad`,
  `qfrc_constraint` `5439 N`, `qacc` `14181`.
- Hence `PRODUCTION_SAMPLING_AUTHORITY=FAIL`; production event/scorer code was
  NOT modified in this mission (independent correction unit required).

## 4. Inverse-dynamics call contract

`mj_inverse(m, d)` requires `(qpos, qvel, qacc)` of ONE instant and computes

> `qfrc_inverse = M(q)·qacc + qfrc_bias(q,qvel) − qfrc_passive(q,qvel) − qfrc_constraint(q,qvel,qacc)`

(proven empirically §§5–6 of the mission evidence: free-flight identity at
`1e-12`, contact identity via `M·qacc+bias−passive−constraint` at `6e-17`).
After the call, `d.qfrc_actuator` is zeroed (not meaningful); the valid test is

> `SYNC_FORCE_ERROR = max|qfrc_inverse − (qfrc_actuator_fwd + qfrc_applied)|`,
> `SYNC_ROOT_ERROR = ‖qfrc_inverse[0:3]‖₂` (root unactuated, applied zero),

evaluated ONLY on synchronized data. Calling `mj_inverse` on post-`mj_step`
data mixes `qpos/qvel(t+h)` with `qacc(t)` and is labeled
`MIXED_STAGE_DIAGNOSTIC_ONLY`; it reproduced REC-01 V1 bit-exactly
(`59.645378 N` / `66.471013`) and is never a validity test.

## 5. Forward/inverse consistency authority

With `mjENBL_FWDINV` enabled on a diagnostic model copy (canonical source
untouched), `solver_fwdinv` stays ≤ `9.4e-08` across all contact modes
including transitions, and enabling the flag leaves the canonical `qpos/qvel`
trajectory bit-identical (`0.0`). Synchronized manual identity holds to
≤ `6.1e-08` (solver-tolerance floor, corroborated by the built-in diagnostic).
REC-01 V1's large residuals were a mixed-stage artifact; its replay,
mapping, and support findings stand, its inverse interpretation is superseded.

## 6. Forward obligations

- All future physics-rate traces MUST be sampled at the SYNCHRONIZED POINT.
- Every sample SHOULD carry `STATE_TIME`, `STATE_SHA256`, `QACC_TIME`,
  `CONTACT_TIME`, `REPORT_TIME`, `SYNCHRONIZED=true`.
- No claim of forward/inverse (in)consistency is admissible unless all
  quantities share one declared instant.
- This contract is void outside its scope line; any Plant/solver/version/arch
  change requires re-verification before measurement claims resume.
