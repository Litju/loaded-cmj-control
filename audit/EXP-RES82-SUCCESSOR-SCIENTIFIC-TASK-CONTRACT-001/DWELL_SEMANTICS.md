# DWELL_SEMANTICS — Successor Event Dwell Convention (MED-001 closure)

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
DEFECT CLOSED: `MED-001` (systematic dwell off-by-one; every dwell event confirmed one
physics sample — 0.125 ms — before its declared duration).
STATUS: **FROZEN**.

The successor contract fixes **one** dwell convention. Event semantics in
`EVENT_CONTRACT_E1_E12.md` reference this document; no event may redefine dwell
arithmetic locally.

---

## 1. Grid and notation

| Symbol | Meaning | Value |
|---|---|---|
| `dt` | physics timestep | 0.000125 s (125 µs) |
| `i` | sample index of the first sample at which the predicate is true (`ONSET_SAMPLE`) | integer ≥ 0 |
| `j` | current sample index while the predicate remains continuously true | integer ≥ i |
| `K` | dwell sample distance `K = j − i` | integer ≥ 0 |
| `D` | `REQUIRED_DURATION` (seconds) | declared per event |
| `K_D` | required sample distance `K_D = ceil(D / dt)` | integer ≥ 0 |
| `N_true` | count of consecutive true samples `N_true = j − i + 1` | integer ≥ 1 |

Predicates are evaluated on **every physics sample** (`dt` grid). A run is
**continuous** iff the predicate is true at every sample from `i` through `j` with no
gap. A single false sample ends the run; a later true sample starts a **fresh**
candidate (`ONSET_SAMPLE` resets, no carryover).

## 2. Normative convention — PHYSICAL_TIME

The successor default is **PHYSICAL_TIME**, defined by the elapsed time between the
first and last true sample of the run:

```
elapsed_time(i, j) = t[j] − t[i]  ==  K · dt        (exact sample-index arithmetic)
CONFIRMED  iff  K ≥ K_D   where   K_D = ceil(D / dt)
```

- `FIRST_TRUE_TIME  = t[i]`
- `CONFIRMATION_SAMPLE = i + K_D`
- `CONFIRMATION_TIME = t[i + K_D]` — numerically identical to `t[i] + K_D · dt` by the
  fixed grid, and computed from the sample index (not by accumulating floating sums).
- Boundary inclusivity: the predicate must be true **at every sample from `i` to
  `i + K_D` inclusive**; the onset sample counts as part of the run but contributes
  zero elapsed time.

The comparison `K ≥ K_D` is an **integer** comparison. Floating-point time is reported
for human readability only and must never be the gate comparison. This removes the
historical off-by-one caused by comparing a sample **count** to a sample **distance**
and by floating accumulation. `K_D` is declared as an integer constant derived from the
exact ratio (`K_D = D_µs / 125` (equivalently `D_ns / 125000`) for the exact multiples used here) and must satisfy the
invariant `(K_D − 1)·dt < D ≤ K_D·dt`; implementations must use integer constants, not
runtime `ceil(D/dt)` on floating values.

## 3. Retired erroneous conventions (explicit)

The following historical behaviours are **invalid** under the successor contract:

1. **Sample-inclusive count**: `N_true = j − i + 1; confirmed iff N_true ≥ K_D`.
   For `D = 0.10 s`, this confirms at `K = 799` (`0.099875 s`), one sample
   (`0.000125 s`) early. This is the R001/MED-001 defect.
2. **Distance-minus-one**: `confirmed iff K ≥ K_D − 1`. Identical to (1); retired.
3. **Floating accumulation without a declared tolerance**: comparing an accumulated
   `t[j] − t[i]` with `D` where numerical noise can flip the last sample. Retired in
   favour of exact `K · dt` sample arithmetic.
4. **Post-hoc dwell reconstruction with a different grid** (control-rate samples used
   for a physics-rate dwell). Retired: dwells are physics-sample dwells.

## 4. DWELL_TYPE taxonomy

| DWELL_TYPE | Meaning | Confirmation |
|---|---|---|
| `NONE` | Instantaneous latch; no sustained condition. | At the first sample the predicate is true; `ONSET_SAMPLE = CONFIRMATION_SAMPLE`, `FIRST_TRUE_TIME = CONFIRMATION_TIME`. |
| `PHYSICAL_TIME` | Sustained predicate with elapsed-time requirement `D`. | Section 2. |
| `SAMPLE_COUNT` | Sustained predicate with a declared count of consecutive true samples `N`. | Confirmed at `j = i + N − 1`; `elapsed_time = (N − 1) · dt`. Must be **explicitly declared per event**; never inferred from a duration. |

No successor E1–E12 event uses `SAMPLE_COUNT`; the declaration is retained so that a
future measurement observable (e.g., a force-threshold comparability dwell) can use it
without re-opening the convention.

## 5. Two-phase events (hysteresis)

Some events separate an `ONSET_PREDICATE` (stricter) from a `SUSTAIN_PREDICATE`
(looser). The convention is:

- The run starts at the first sample satisfying `ONSET_PREDICATE`.
- Continuation requires `SUSTAIN_PREDICATE` at every subsequent sample.
- An `ONSET_PREDICATE` sample must also satisfy `SUSTAIN_PREDICATE` (checked in the
  contract for each two-phase event; the onset predicate is defined strictly stronger).
- Dwell arithmetic is unchanged and applies to the continuous run.

## 6. Worked boundary examples at dt = 0.000125 s

| `D` (s) | `K_D = ceil(D/dt)` | First true sample `i` | `FIRST_TRUE_TIME` | `CONFIRMATION_SAMPLE` | `CONFIRMATION_TIME` | `elapsed` |
|---|---|---|---|---|---|---|
| 0.005 | 40 | 100 | 0.012500 s | 140 | 0.017500 s | 0.005000 s |
| 0.010 | 80 | 1000 | 0.125000 s | 1080 | 0.135000 s | 0.010000 s |
| 0.020 | 160 | 2000 | 0.250000 s | 2160 | 0.270000 s | 0.020000 s |
| 0.030 | 240 | 2000 | 0.250000 s | 2240 | 0.280000 s | 0.030000 s |
| 0.050 | 400 | 4000 | 0.500000 s | 4400 | 0.550000 s | 0.050000 s |
| 0.080 | 640 | 4000 | 0.500000 s | 4640 | 0.580000 s | 0.080000 s |
| 0.100 | 800 | 1000 | 0.125000 s | 1800 | 0.225000 s | 0.100000 s |
| 0.150 | 1200 | 5000 | 0.625000 s | 6200 | 0.775000 s | 0.150000 s |
| 0.500 | 4000 | 1000 | 0.125000 s | 5000 | 0.625000 s | 0.500000 s |

Contrast with the retired convention: for `D = 0.100 s`, `i = 1000`, the retired
convention confirmed at sample `1799` (`0.224875 s`, elapsed `0.099875 s`), which is
exactly `0.000125 s` early. The successor confirms at sample `1800` (`0.225000 s`).

## 7. Per-event dwell declaration (normative summary)

| EVENT_ID | NAME | DWELL_TYPE | REQUIRED_DURATION | Notes |
|---|---|---|---|---|
| E1 | supported_start | PHYSICAL_TIME | 0.100 s | Initial-standing validity run |
| E2 | countermovement_onset | PHYSICAL_TIME | 0.030 s | Two-phase (onset < −0.08 m/s; sustain < −0.03 m/s) |
| E3 | valid_countermovement | NONE | — | Latches when depth and support conditions first hold |
| E4 | upward_reversal | PHYSICAL_TIME | 0.010 s | Two-phase (onset > +0.02 m/s; sustain > 0) |
| E5 | vertical_propulsion | PHYSICAL_TIME | 0.050 s | Net-positive upward propulsion under bilateral support |
| E6 | physical_takeoff | NONE | — | Instantaneous physical-state latch; sustain is owned by E7 |
| E7 | genuine_flight | PHYSICAL_TIME | 0.050 s (candidate; OD-03) | Physical no-contact + clearance condition |
| E8 | apex | NONE | — | Deterministic zero-crossing, interpolated |
| E9 | descending_landing | PHYSICAL_TIME | 0.010 s | Physical contact re-establishment with descending COM |
| E10 | impact_absorption | PHYSICAL_TIME | 0.020 s | Post-impact admissible local state |
| E11 | balance_capture | PHYSICAL_TIME | 0.150 s | Captured-state run (whole-body state variables) |
| E12 | stable_recovery | PHYSICAL_TIME | 0.500 s | Robust standing-envelope run |

All numeric `REQUIRED_DURATION` values are either frozen task decisions or owner
decisions; provenance per event in `EVENT_CONTRACT_E1_E12.md`.

## 8. Reporting requirements

Every event record MUST expose:

- `ONSET_SAMPLE`, `FIRST_TRUE_TIME`;
- `CONFIRMATION_SAMPLE`, `CONFIRMATION_TIME`;
- `DWELL_TYPE`, `REQUIRED_DURATION`, `K_D`;
- `vDWELL` (diagnostic) = actual confirmed elapsed time.

`occurred_at := FIRST_TRUE_TIME`; `confirmed_at := CONFIRMATION_TIME`. No consumer may
report `confirmed_at − occurred_at < REQUIRED_DURATION` for a confirmed event.

## 9. Failure-blocker scope semantics (normative)

`FAILURE_BLOCKERS` lists are evaluated with this scope rule; no other interpretation is
permitted:

1. **Default rule (all blockers):** a blocker fails the candidate run if its condition
   holds at **any** sample during the continuous candidate run (onset through the
   current sample). A single-sample occurrence resets the candidate exactly like a
   sustain failure. Named templates below are specializations; any blocker name that
   does not match a template is evaluated by this default rule.
2. A blocker named `<condition>_before_confirmation` or `<condition>_during_run` is
   evaluated over `[ONSET_SAMPLE, CONFIRMATION_SAMPLE]` inclusive.
3. A blocker named `<condition>_at_confirmation` is evaluated at `CONFIRMATION_SAMPLE`
   only.
4. A blocker named `<condition>_never_met` is a whole-episode condition evaluated at
   the episode horizon (or at terminal fall).
5. A blocker that occurs **after** an event is confirmed never retroactively unlatches
   that event; it is handled by the layer gates (L1–L5) and by terminal-fall
   adjudication. Events are physical-history facts, not conditionally valid labels.
6. Blocker templates not listed here (e.g., `support_margin_le_0`, `clearance_violation`,
   `reflight_within_run`, `envelope_exit`, `layer_landing_gate_failure`) are evaluated
   by the default rule unless the event contract states otherwise; `layer_*` blockers
   are layer-owned and never unlatch an event.

## 10. Non-event dechatter spans (reflight/chatter)

The non-event project dechatter rules use the same integer sample-distance convention:

- `REFLIGHT`: whole `Fz < 10.0 N` for `N_true ≥ 4` consecutive samples — an explicit
  **SAMPLE_COUNT** rule preserving the frozen RES-57 detection (`CANONICAL_REFLIGHT_MIN_STEPS=4`).
  Span = 3 intervals = 0.375 ms. The historical "4 steps = 0.5 ms" wording counted 4
  intervals and is recorded as an erratum; the frozen detection behaviour is not
  weakened or strengthened by the successor contract.
- `CHATTER_TRANSITION`: whole-Fz crossing of the 10.0 N threshold; maximum 8.

This is the only declared SAMPLE_COUNT dwell in the successor contract; all E1–E12
events remain PHYSICAL_TIME or NONE.

## 11. Consistency demands on future implementations (RES-89/RES-91)

1. A boundary test must verify confirmation at exactly `K_D` and rejection at `K_D − 1`.
2. A test must verify that a single false sample inside a candidate run resets the
   candidate to the next true sample.
3. A test must verify `confirmed_at − occurred_at ≥ REQUIRED_DURATION` for every event.
4. Dwell constants must be declared once and bound to this document by hash; no
   duplicate literal in runtime, mirror, or external JSON.
