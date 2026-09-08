# SUPPORT_SEMANTICS_CALLGRAPH — RES-57 Phase A

`MISSION=RES10_BILATERAL_SUPPORT_CONTINUITY_AUTHORITY_001`
Authority HEAD `e85d9c9492dd648089350206c6a71d632e342929` / TREE `850c718228d628fde19de9c38b861b01bc8a596a`.
No Plant/contact/controller/scorer edits. Read-only audit.

## 1. Canonical per-foot force threshold

- `src/loaded_cmj/v2/constants.py:180`
  `V2_CONTACT_FZ_THRESHOLD_N = 10.0` — "per foot low threshold for bilateral detection; separate from event hysteresis".
- `tools/res52/spec.py:74`
  `CONTACT_ACTIVE_THRESHOLD_N = 10.0` — RES-52/55 inner-layer active threshold (same numeric value, separate owner).
- `src/loaded_cmj/v2/constants.py:195`
  `V2_EVENT_THRESHOLDS["BILATERAL_TAKEOFF_FZ_N"] = 10.0` — scorer/event threshold.
- `src/loaded_cmj/v2/events.py:27`
  `BILATERAL_FZ_THRESHOLD_N = float(V2_EVENT_THRESHOLDS["BILATERAL_TAKEOFF_FZ_N"])` — derived alias, no new number.

All three owners agree numerically at **10.0 N per foot**. No hysteresis band exists anywhere:
active is strict `> 10.0`, inactive is `<= 10.0`. There is no separate release/engage
threshold, no debounce, no time hysteresis at sample level. Dwell/hysteresis enters only
through run-length / episode / dwell counters (see §5).

## 2. Per-foot contact-active predicate (two parallel implementations, same threshold)

### 2a. Production / scorer path — force-summary active flags

- `src/loaded_cmj/v2/plant.py:187-331` `V2Plant.foot_contact_summary(data)`
  - Sums world-frame plantar force per foot from `mj_contactForce` rows where
    `(geom == floor) and (geom == left_foot_box or right_foot_box)` (lines 196-244).
  - World conversion: `world_force = frame.T @ (sign * wrench[:3])` with
    `sign = +1 if foot_geom == con.geom[1] else -1` (lines 227-235, V1 convention preserved).
  - Returns `left_Fz = left_F[2]`, `right_Fz`, `whole_Fz` (313-315).
  - Active flags (301-304):
    `active_left = left_Fz > V2_CONTACT_FZ_THRESHOLD_N`
    `active_right = right_Fz > V2_CONTACT_FZ_THRESHOLD_N`.
- `src/loaded_cmj/v2/measurement.py:207-218` `SynchronizedPhysicsSample.from_live_state`
  reads `sm["active_left/right"]` from the **shadow** `meas` (forwarded copy) into
  `support_active[2]`. Live is never forwarded.
- `tools/res52/run_cell.py:162-163`, archived RES-56 `run_capture.py` equivalent:
  `actL = (arr["fzl"] > 10.0)`, `actR = (arr["fzr"] > 10.0)` where `fzl/fzr` are the
  synchronized per-foot Fz traces. Literal `10.0` duplicates the constant (same value).

### 2b. Inner-layer / geometry path — contact-row existence

- `tools/res52/core52.py:146-165` `foot_floor_contacts(model, data)`
  per foot: all `data.contact` rows with `(floor, foot_box)` pair; `deepest_row` = argmin `dist`.
- `tools/res52/core52.py:186-223` `soft_contact_state(model, data, gap_half_height)`:
  - foot has row → `active_row=True`, `dist=con.dist`, `nvel=efc_vel[efc_address]`;
  - foot has no row → `active_row=False`, `dist=geom_xpos[fg][2]-gap_half_height` (geometric gap,
    positive), `nvel=true_foot_point_velocity(...)[2]`.
- `tools/res52/soft_contact.py:73-105` `SoftContactPolicy.y_of(pd)` — identical row/deepest
  selection on probe copies; `contact_flags` (107-109) returns force-summary active flags.
- `tools/res52/s1_branch_states.py:52-83` — branch authority records both `ACTIVE_CONTACT_ROWS`,
  `CONTACT_DISTANCE_L/R`, `CONTACT_ACTIVE_ROW_L/R` (row existence) and `CONTACT_ACTIVE_L/R`
  (force-summary `support_active`).

Key distinction preserved: **row existence** (`active_row`, geometric engagement proxy) vs
**force active** (`Fz > 10 N`, compressive-support proxy). They usually agree but need not:
a penetrating row can carry ~0 force for one solver step (the RES-56 flicker pattern), and a
separating foot has no row by construction.

## 3. Contact distance / penetration

- `tools/res52/core52.py:10-14,186-223`:
  `contact_distance_i` = signed MuJoCo `contact.dist` of deepest floor row for foot i
  (negative = penetrating). No-row → lowest-corner gap (positive).
  `penetration_i = max(0, -contact_distance_i)` [m].
- `src/loaded_cmj/v2/plant.py:245-246,326`: `penetration = -dist if dist<0 else 0`,
  `max_penetration` = max over floor-foot rows; `SynchronizedPhysicsSample.max_penetration_m`.
- `tools/res52/soft_contact.py:86-90`: same deepest-dist selection on probes.
- Model: `geom_size[left_foot_box][2]` half-height via `foot_gap_half_height`
  (`tools/res52/core52.py:226-232`); frozen model geometry, never tuned.
- MuJoCo soft contact: `V2_CONTACT_SOLREF=(0.016,1.0)`, `V2_CONTACT_SOLIMP=(0.99,0.99,0.001,0.5,2.0)`,
  friction `(0.9,0.005,0.0001)` (`src/loaded_cmj/v2/constants.py:123-128`). Penetration is
  expected (compliant contact); `dist<0` means geometric overlap, not guaranteed load.

## 4. Normal velocity (RES-55 corrected authority)

- Contract: `TRUE_FOOT_POINT_VELOCITY_CONTRACT.md` (root, committed `e85d9c9`).
- Implementations (identical):
  `tools/res52/core52.py:168-183` and `tools/res52/soft_contact.py:43-53`
  `true_foot_point_velocity(model, data, body_id, point_world)`:
  `v = J_point(q) @ qvel` via `mj_jac`, non-mutating same-state. Prohibited:
  `mj_objectVelocity.linear + omega x (p - xpos)`.
- Active row: `nvel = data.efc_vel[con.efc_address]` = MuJoCo `d(dist)/dt` exactly.
- Inactive: `nvel = true_foot_point_velocity(... p_low)[2]`, `p_low = geom_xpos[fg]-[0,0,hh]`.
- Sign: world `+z`; `foot_normal_velocity = d(dist)/dt`; POSITIVE = SEPARATING,
  NEGATIVE = APPROACHING (`core52.py:15-21`, `soft_contact.py:27-29`).
- Consumers: `TraceAcc` (`core52.py:272-281,310-315`), `SoftContactPolicy.y_of` outputs
  `NVEL_L/R` (`soft_contact.py:101-105`), local map `G[5,6]`, `solve_du` `W_NVEL` regulation
  toward 0, validation `NVEL_ABS_MPS=0.05` (`spec.py:36`), qualified domain `[-0.35,0.35]`
  (`spec.py:70-71`).

## 5. Episode / dwell / counting semantics (frozen, must not be re-tuned)

All counts operate on **physics-rate** synchronized traces with `PHYSICS_DT=0.000125 s`.

- Walk (transient) window: `WALK_WINDOW_S = 0.030` s = 240 physics steps
  (`tools/res52/spec.py:75`). Samples with `t_rel <= 0.030` are excluded from
  `POST_WALK_LOSS_EPISODES`; they are still counted in fraction/run statistics.
- Per-foot inactive fraction: `1 - mean(actL)` over full horizon; gate `<= 0.005`
  (`spec.py:77`, `run_cell.py:224-225`). For 1200-step S50 horizon this allows ≤ 6 inactive
  samples per foot in total.
- Per-foot max inactive run: `max(_runs(~actL))`; gate `<= 2` physics steps = 0.25 ms
  (`spec.py:78`, `run_cell.py:226-227`). A single-step dropout (run length 1) passes this gate.
- Whole-support chatter: `below = (FZw < 10.0)`; `trans = sum(diff(below) != 0)` = number of
  whole-Fz threshold transitions; gate `<= 8` (`spec.py:81`, `run_cell.py:165,230`).
  This is the "primary chatter" / `no_primary_chatter` gate. S50 observed 0.
- Reflight (canonical): `refl_runs = _runs(below)`; `reflight_episodes = [r for r in refl_runs
  if r >= REFLIGHT_MIN_DURATION_PHYSICS_STEPS=4]` (`spec.py:79`, `run_cell.py:166-167`).
  Canonical reflight = whole-support loss (`whole Fz < 10 N`) sustained ≥ 4 physics steps
  = **0.5 ms**. S50/S75 observed `[]`.
- POST_WALK loss episodes (strict unilateral-count gate):
  `either = ~actL | ~actR`; runs of `either`; `post_walk_loss_episodes = #{runs with
  start_time - t0 > WALK_WINDOW_S}` (`run_cell.py:168-175`). Gate `<= 1`
  (`spec.py:80`, `run_cell.py:229`). RES-56 S50 observed **3** (the three flickers) → FAIL.
- RES-56 runner uses byte-identical logic with `WALK_S=0.030`, `GATE_PEN=0.010`,
  `GATE_PEAK_BW=8.0` (archived `run_capture.py:280-310, ~360-412`).
  `bilateral_contact` gate additionally requires `not REFLIGHT_EPISODES` and
  `POST_WALK_LOSS <= 1`; `no_chatter` requires `trans <= 8`.

Helpers `_runs` / `_run_starts` (`run_cell.py:300-326`, identical in archived runner):
maximal-True-run lengths / start indices. Deterministic, no tolerance.

## 6. V2EventDetector contact predicates (scorer, unchanged)

File: `src/loaded_cmj/v2/events.py`. Physics DT `0.000125` (line 21). Threshold alias line 27.
Dwell map `DWELL_S` (48-61) → sample windows `WIN` (64-70, `round(dwell/DT)`).

| Event | Guard (contact part) | Dwell | Window |
|---|---|---|---|
| supported_start (126-141) | whole>0.30 BW AND L>10 AND R>10 AND margin/tilt/prohibited/\|vz\|<0.05 | 0.10 s | 800 |
| countermovement_onset | vz + whole>0.30 BW | 0.03 s | 240 |
| valid_countermovement | depth + whole>0.30 BW | 0.0 | 1 |
| upward_reversal | vz crossings | 0.010 s | 80 |
| vertical_propulsion | vz>0 AND whole>1.05 BW | 0.050 s | 400 |
| bilateral_takeoff onset (193-202) | L<10 AND R<10 AND vz≥0.60 | 0.010 s | 80 |
| bilateral_takeoff sustain (204-209) | L<10 AND R<10 | — | — |
| genuine_flight (211-216) | L<10 AND R<10 | 0.08 s | 640 |
| apex (408-439) | vz +→− crossing in flight | 0.0 | 1 |
| descending_landing onset (218-230) | vz<−0.10 AND (L>10 OR R>10) | 0.010 s | 80 |
| descending_landing sustain (232-239) | L>10 OR R>10 | — | — |
| impact_absorption (241-245) | \|vz\|<0.05 | 0.020 s | 160 |
| balance_capture (247-257) | L>10 AND R>10 AND \|vz\|<0.30 | 0.150 s | 1200 |
| stable_recovery (392-406) | tilt + \|vz\|<0.05 + whole>0.5 BW + true-standing neighborhood | 0.500 s | 4000 |

True-standing neighborhood (259-390): 7 joints + COM z + root z + trunk pitch inside
`V2_TRUE_STANDING_ENVELOPE` plus `L>10 AND R>10`, no fall, no prohibited. Reuses 10 N.

Note: detector uses **10 N per-foot** force predicates only — no geometry, no normal
velocity, no duration below 5-10 ms except apex/instant events. It cannot by itself
distinguish a 0.125-ms force flicker with retained geometry from genuine liftoff; that is
why RES-57 needs a separate support-continuity contract built on synchronized geometry +
velocity + duration.

## 7. RES-52/55 contact-retention semantics (carried, unchanged)

- Sealed spec: `tools/res52/spec.py:20-97` (`CONTROLLER_CONSTANTS`), SHA
  `a8087f02fc78aa7977aedf6d5d819963bb10c355818c57154b3cfb3df2ec42ef`.
- `PEN_MIN_RETAIN_M=0.0005`, `PEN_MAX_M=0.0090`, `PEN_SAFETY_LIMIT_M=0.010` (60-62,76).
- One-sided distance rows in `SoftContactPolicy._dist_rows` (`soft_contact.py:196-217`):
  separated (`d0>0`) → drive to 0; over-penetrated (`d0<−PEN_MAX`) → cap; shallow
  (`d0>−PEN_MIN_RETAIN`) → retain compression. Weights `W_DIST_ENGAGE/MAX/RETAIN = 4/4/2`.
- `W_NVEL=1.0` regulates foot normal velocity toward 0 (approach/separation damping).
- Scales: `FZ 100 N`, `NVEL 0.05 m/s`, `DIST 0.0005 m`, `VZ 0.02`, `QDOT 0.3` (53-59).
- Validation: `FZ_WHOLE 20 N`, `FZ_L/R 15 N`, `DIST 0.0005 m`, `NVEL 0.05 m/s`,
  `COM_VZ 0.01`, `QDOT 0.05` + `TRUST_REL 0.10` (32-39); `RHO0 0.25`, shrinks ≤3, fallback
  holds previous action (28-31,66-69). Trust fallbacks gate ≤5.
- P1a S50 125-ms cell is the arrest-equivalent reference: early S50 RES-56 arrest is
  bit-identical to qualified P1a peak/penetration per RES-56 receipt.

## 8. Required six-way distinction (definitions for Phase B/E)

1. `GEOMETRIC_ENGAGEMENT_SAMPLE` — foot has ≥1 floor contact row at this physics sample
   (`active_row=True`, deepest `dist` defined). Pure geometry-existence fact.
2. `COMPRESSIVE_SUPPORT_SAMPLE` — synchronized per-foot `Fz > 10.0 N`. Load-bearing fact.
3. `UNILATERAL_FORCE_DROPOUT_SAMPLE` — exactly one foot `Fz <= 10.0 N` while the other
   foot `Fz > 10.0 N` at this sample (whole support may remain high). Force-pattern fact.
4. `GEOMETRIC_LIFTOFF_SAMPLE` — foot has **zero** floor rows AND lowest-corner gap `> 0`
   (equivalently no `active_row` and `dist > 0`). Physical-separation fact. Must be
   reported as such whenever observed; never relabeled by duration filtering.
5. `CONTROL_RELEVANT_SUPPORT_LOSS_EPISODE` — finite-duration episode meeting the
   prospectively frozen Phase-B contract (force threshold + separation evidence +
   duration). Only this level can QUALIFY/NOT_QUALIFY support continuity.
6. Canonical `REFLIGHT` — whole `Fz < 10 N` for ≥4 consecutive physics steps (0.5 ms).
   Stronger, bilateral, force-only event. Unchanged.

Existing gates conflate (2) with (1) at sample level (force threshold as contact proxy)
and count (3) without (4). Phase B must separate them prospectively.
