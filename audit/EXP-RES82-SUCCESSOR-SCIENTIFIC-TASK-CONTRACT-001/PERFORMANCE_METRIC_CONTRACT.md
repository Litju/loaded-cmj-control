# PERFORMANCE_METRIC_CONTRACT — Successor Loaded-CMJ Metrics

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **STRUCTURALLY FROZEN**; numeric floors are explicitly owner decisions.
DEFECTS CLOSED HERE: `CRIT-003` (no meaningful task-performance requirement),
`MED-002` (COM-speed name/implementation mismatch), `MED-011` (metric definition
ambiguity as it affects performance reporting), and the "one Boolean 12/12"
architecture.

This contract defines **which quantity is the method-qualified canonical jump height**,
**which quantities are cross-checks**, and **which single magnitude floor gates task
performance**. It does not
triple-gate ballistically related quantities.

---

## 1. Jump-height quantity adjudication

Literature: four force-platform jump-height equations exist in two families — defined
from takeoff and defined from standing — and can differ by up to ~15 cm on the same
jump; the choice must follow the jump modality, the reason for testing, and the
definition of jump height (Eythorsdottir 2024, PMID 39425876). The impulse–momentum
(takeoff-to-apex) method is the preferred method when the height from takeoff to apex is
wanted; double integration is preferred for standing-to-apex (Xu 2023, PMID 36940054).
Processing choices alone can change the estimate by >25 % (Eythorsdottir 2026,
PMID 41672931).

In this simulation the **exact whole-body COM state is available at every physics
sample**. Therefore:

| ID | Quantity | Definition | Status | Rationale |
|---|---|---|---|---|
| **A** | `COM_RISE_TAKEOFF_TO_APEX` (`h_direct`) | `COM_Z(apex) − COM_Z(physical_takeoff)` | **PRIMARY_CANONICAL_JUMP_HEIGHT** | Direct model-truth displacement; no force threshold, no integration drift, no flight-time assumption; matches the "takeoff-to-apex" definition family with the least processing sensitivity |
| B | `BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ` (`h_tov`) | `vz(physical_takeoff)² / (2g)` | **SECONDARY_CROSS_CHECK** (physics consistency) | Impulse–momentum-equivalent estimate; differs from A only by internal segment motion during flight |
| C | `FLIGHT_TIME_HEIGHT` (`h_ft`) | `g · T_flight_physical² / 8`, `T_flight_physical = LANDING_FIRST_CONTACT_TIME − PHYSICAL_TAKEOFF_TIME`; `g = 9.81 m/s²` (project constant) | **EXPERIMENTAL_COMPARABILITY_METRIC** (report-only) | Assumes equal COM heights at takeoff and landing and no posture asymmetry; a loaded landing violates these assumptions; retained only for comparison with flight-time-method studies |
| D | `COM_APEX_ABOVE_INITIAL_STANDING` (`h_standing`) | `COM_Z(apex) − COM_Z(initial_standing)` | **SECONDARY_CONTEXT_METRIC** | Standing-to-apex definition family; never interchangeable with A; difference = `TAKEOFF_COM_Z − INITIAL_STANDING_COM_Z`, sign-indefinite |
| E | `FORCE_INTEGRATED_COM_DISPLACEMENT` | double integration of `(Fz_whole − m·g)/m` from initial standing to apex | **EXPERIMENTAL_COMPARABILITY_METRIC** (report-only) | The experimental double-integration equivalent; in simulation it must agree with the model-truth trajectory within numerical bounds (verification diagnostic) |

**Normative naming rule.** The bare term "jump height" is prohibited in successor
outputs. Every value is written with its method qualifier:
`COM_RISE_TAKEOFF_TO_APEX`, `BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ`,
`FLIGHT_TIME_HEIGHT`, `COM_APEX_ABOVE_INITIAL_STANDING`,
`FORCE_INTEGRATED_COM_DISPLACEMENT`. The historical conflation of `0.0363 m` and
`0.0752 m` under one "jump height" label is explicitly retired (ERR-001).

**Reference points are explicit.** `COM_Z(apex)` is an absolute world coordinate and is
never called a height. `TAKEOFF_COM_Z` is the COM world coordinate at physical takeoff.

## 2. Primary performance gate

The successor contract contains exactly **one** primary performance magnitude gate:

```
PRIMARY_PERFORMANCE_GATE:
    COM_RISE_TAKEOFF_TO_APEX >= H_MIN
```

`H_MIN` status: **OWNER_DECISION_REQUIRED (OD-01)**. Candidate options in §2.1. Until
OD-01 is resolved, the structure of the gate is frozen and no candidate may be
classified `TASK_SUCCESS = true`.

### 2.1 Candidate floors with evidence

Population/task context for all candidates: adult male athletes performing a bilateral
CMJ while carrying a 20 kg external load; measurement by force plate.

| Candidate | Value | Basis | False pass prevented | Tradeoff |
|---|---|---|---|---|
| **PF-1 (recommended)** | `h_direct ≥ 0.150 m` | Kraska 2009 (PMID 20029097): collegiate athletes' **strong-group** 20 kg CMJ height 27.57 ± 8.64 cm (mean − SD ≈ 18.9 cm); 0 kg CMJ group means 28.3–33.5 cm. The weak-stratum 20 kg mean is **not directly reported**; applying the weak group's larger SJ percent decrease (≈30.4 % vs 17.8 %) to its 0 kg mean gives a derived estimate ≈ 19–20 cm (derivation, not a reported value). 0.150 m is ≈ 20–25 % below that derived weak-stratum estimate and ≈ 2× R001's 0.0752 m. **Cross-family limitation:** the human values are flight-time jump heights, the gate is direct COM displacement; the comparison is a plausibility band, not a calibration (S01/S02). | Trivial hop; force-threshold artifact; a 3.6–7.5 cm "jump" | Conservative as a *task-plausibility floor*, not a performance target |
| PF-2 | `h_direct ≥ 0.200 m` | Near the derived weak-stratum 20 kg estimate (≈19–20 cm) | Above = "genuinely loaded-jump-scale" | May exceed the rebuilt Plant/controller capability, converting a task floor into a de-facto performance target |
| PF-3 | `h_direct ≥ 0.100 m` | Structural false-pass floor only | Excludes R001 (0.075 m) but not a 12–15 cm hop | Weak scientific statement; a 12 cm hop would qualify |
| PF-4 | no numeric floor frozen now | Structural gate frozen; numeric deferred to owner after RES-83 capability is known | None until resolved | `TASK_PERFORMANCE_VALID` stays false/unresolved; the contract remains honest but cannot qualify a candidate |

**Recommendation (engineering-scientific inference, not an owner decision): PF-1.**
Rationale: it is anchored to the published 20 kg CMJ performance scale in adult male
athletes, sits ≈ 20–25 % below a clearly labelled *derived* weak-stratum estimate, and
excludes the trivial-hop false pass by a factor of two. It does not encode a
performance target. The human–simulation cross-family comparison is disclosed and
limits the claim to task plausibility.

**No candidate may be adopted silently.** OD-01 records the decision, value, rationale,
and date. After RES-83 lands, the owner must confirm that the adopted floor remains a
*task* floor and not a *model capability* assertion; if the rebuilt model cannot reach
it, that is a Plant/controller finding, not a contract edit.

### 2.2 What the primary gate does NOT claim

- It is not a normative population percentile.
- It is not an injury/technique judgement.
- It does not imply the model reproduces a human loaded CMJ; it bounds the task
  interpretation of the simulation only.

## 3. Physics consistency gates

Ballistically related quantities are cross-checked, not re-gated as magnitude floors.

| ID | Gate | Definition | Class | Provenance |
|---|---|---|---|---|
| `CG-01` | `FLIGHT_ACCELERATION_CONSISTENCY` | In the physical flight interval, the whole-body COM vertical acceleration estimate `Δcom_vz/Δt` satisfies `abs(mean + g) <= ε_acc` except for samples adjacent to contact; no external vertical force acts | CONSISTENCY_GATE | PHYSICS_IDENTITY + NUMERICAL_TOLERANCE (`ε_acc` from solver/finite-difference limits; RES-89) |
| `CG-02` | `BALLISTIC_HEIGHT_RESIDUAL` | `residual = h_direct − h_tov` over the certified physical flight interval; the only external force in flight is gravity, so the whole-body COM is ballistic and the residual is numerical | CONSISTENCY_GATE | PHYSICS_IDENTITY + NUMERICAL_TOLERANCE; candidate `ε_ball = 0.001` relative, tightened with RES-89 solver data |
| `CG-03` | `TAKEOFF_TRANSITION_WHIP` | `max abs(root_pitch_rate)` over `[E5_onset, E6 + 0.050 s]` ≤ `R_WHIP`; also `max abs(trunk_pitch_rate)` ≤ `R_WHIP_TRUNK` | HARD_GATE (consistency class) | Partial project history + owner decision OD-06; candidate `R_WHIP = 5.0 rad/s` (R001 was 9.68 rad/s) |
| `CG-04` | `EXTENSION_STATE_AT_TAKEOFF` | At physical takeoff, hip/knee/ankle configuration lies inside the RES-83-declared robust extension set (concept frozen; numeric deferred) | HARD_GATE | MODEL_DEPENDENT_DEFERRED (RES-83), prevents the R001 reverse-knee/hip-extension takeoff state |
| `CG-05` | `FORCE_PHYSICAL_TAKEOFF_ORDER` | `FORCE_TAKEOFF_LEAD = t_physical_takeoff − t_force_threshold_takeoff`; reported; a large positive lead indicates force chatter decoupled from geometry | DIAGNOSTIC | MEASUREMENT (Smith 2024; Pérez-Castilla 2022); no fence in either direction |
| `CG-06` | `FLIGHT_TIME_HEIGHT_DIFFERENCE` | `h_ft − h_direct`; reported | DIAGNOSTIC | Measurement comparability only; never gates |

`CG-03` and `CG-04` are the anti-whip/anti-anatomy hard gates that the historical
system lacked. They are **not** magnitude gates and do not multiply the performance
floor.

## 4. Minimum physical-flight validity

- `E7` (genuine flight) must confirm with the declared dwell (OD-03).
- The apex must lie strictly inside the physical-flight interval (E8).
- The physical flight interval must be bounded by E6 and E9 and must contain no
  contact rows for either foot between those events (checked per sample).
- No force-threshold quantity participates in any of these conditions (HIGH-010).

## 5. Metric classes

Every canonical metric carries exactly one class:

| Class | Meaning |
|---|---|
| `HARD_GATE` | A false value prevents `TASK_SUCCESS`; an `INVALID_OUTCOME_PREVENTED` statement is mandatory |
| `CONSISTENCY_GATE` | Internal physical/numerical consistency; failure blocks success |
| `DIAGNOSTIC` | Reported and used for review; does not gate |
| `REPORT_ONLY` | Raw data and context; never gates |

### 5.1 Hard gates and the invalid outcomes they prevent

| Hard gate | Variable | Definition | Units | Threshold/status | Provenance | Invalid outcome prevented |
|---|---|---|---|---|---|---|
| `FINITE_STATE` | all state values | every declared value finite at every sample | boolean | true / FROZEN | PHYSICS_IDENTITY | non-finite/NaN state posing as a trajectory |
| `EXECUTION_COMPLETED` | runtime status | episode reaches declared horizon without fault | boolean | true / FROZEN | NUMERICAL_TOLERANCE | truncated run posing as a completed task |
| `HORIZON_SUFFICIENT` | episode horizon | horizon covers E12 confirmation plus the post-E12 observation margin | s | OD-13 | NUMERICAL_TOLERANCE | success declared before the post-recovery observation could elapse |
| `NO_FALL_CONTACT` | fall_contact | no fall shell loads the floor | boolean | false / FROZEN | PROJECT_HARD_RULE | standing/landing on a fall shell |
| `NO_PROHIBITED_CONTACT` | prohibited_contact | no non-plantar non-fall support | boolean | false / FROZEN | PROJECT_HARD_RULE | support substitutes (load, torso, limb shells) |
| `PENETRATION_MAX` | MAX_PENETRATION | max floor penetration | m | 0.010 / OD-04 re-approval | PROJECT_HISTORICAL_AUTHORITY | sinking through the floor beyond tolerance |
| `NO_ARTIFICIAL_SUPPORT` | artificial_support (forces and state/integration overrides) | no external support force and no state/integration override outside the Plant | boolean | false / FROZEN | PROJECT_HARD_RULE | external support or state manipulation not in the Plant (freeze exploits) |
| `JOINT_LIMITS_RESPECTED` | joint configuration | inside RES-83 anatomical ranges | rad | DEFERRED | MODEL_DEPENDENT_DEFERRED | anatomically invalid joint configurations |
| `ACTUATOR_BOUNDS_RESPECTED` | actuator command | inside declared limits | norm/Nm | DEFERRED | MODEL_DEPENDENT_DEFERRED | commands outside declared actuator limits |
| `PHYSICAL_TAKEOFF_DEFINED` | E6 predicate | contact-free + clearance + upward COM | boolean | structure frozen; OD-02 | PHYSICS_IDENTITY + OWNER_TASK_DECISION | force-threshold artifact declared as takeoff |
| `GENUINE_FLIGHT_DWELL` | E7 dwell | sustained no-contact + clearance | s | OD-03 | OWNER_TASK_DECISION | single-sample force dropout declared as flight |
| `APEX_IN_FLIGHT` | E8 crossing | vz zero-crossing inside physical flight | boolean | true / FROZEN | PHYSICS_IDENTITY | contact-phase zero-crossing declared as apex |
| `TAKEOFF_TRANSITION_WHIP` | max abs(root_pitch_rate) | peak root/trunk pitch rate over transition | rad/s | OD-06 | MODEL_DEPENDENT_DEFERRED + OWNER_TASK_DECISION | pitched/whipping takeoff |
| `EXTENSION_STATE_AT_TAKEOFF` | takeoff joint configuration | inside RES-83 robust extension set | rad | DEFERRED | MODEL_DEPENDENT_DEFERRED | anatomically invalid takeoff configuration |
| `PRIMARY_PERFORMANCE_GATE` | COM_RISE_TAKEOFF_TO_APEX | direct COM rise physical-takeoff→apex | m | OD-01 | LITERATURE_INFORMED + OWNER_TASK_DECISION | trivial hop qualifying as a loaded CMJ |
| `BILATERAL_LANDING` | establishment time | both feet loaded within dwell | s | OD-10 | OWNER_TASK_DECISION | single-foot or foot-avoidance landing |
| `LANDING_BOUNDS` | landing state vector | L4-T1..T8 incl. behavioral momentum capture | mixed | OD-04 | OWNER_TASK_DECISION | forward lunge / pitched landing arrested only vertically |
| `NO_POST_LANDING_REFLIGHT` | reflight, chatter | no reflight beyond approved chatter | boolean/count | reflight=false, chatter≤8 / FROZEN | PROJECT_HISTORICAL_AUTHORITY | support loss after landing beyond approved chatter |
| `BALANCE_CAPTURE_BOUNDS` | speed/Hy/margin | E11 capture bounds | m/s, kg·m²/s, m | OD-05 | OWNER_TASK_DECISION | "vertically stopped" declared as captured |
| `STABLE_STANDING_ENVELOPE` | envelope membership | robust physical standing predicate | boolean | RES-87 calibration | MODEL_DEPENDENT_DEFERRED | micron-overfit or brittle standing |
| `STABLE_HOLD_DURATION` | E12 dwell | sustained envelope membership | s | 0.500 / FROZEN | OWNER_TASK_DECISION | momentary standing instant declared as recovery |

### 5.2 Provenance-class mapping (project authority labels)

The six declared provenance classes are the only classes a **numeric threshold** may
carry. Project-authority labels used in the gate catalog are recorded as source labels
and map onto the six as follows:

| Source label used | Mapped class |
|---|---|
| `PROJECT_HARD_RULE` | OWNER_TASK_DECISION |
| `PROJECT_HISTORICAL_AUTHORITY` / `PROJECT_HISTORICAL_HARD_RULE_CANDIDATE_OD04` | OWNER_TASK_DECISION (subject to OD-04 re-approval) |
| `PROJECT_AUTHORITY_SUPPORT_CONTINUITY` | OWNER_TASK_DECISION (frozen RES-57 values) |
| `TASK_OD10` | OWNER_TASK_DECISION (OD-10) |
| `ENGINEERING_INFERENCE` | LITERATURE_INFORMED or OWNER_TASK_DECISION |
| `MEASUREMENT` (diagnostics without thresholds, e.g. CG-05) | n/a (no numeric threshold) |
| `MODEL_DEPENDENT_DEFERRED` | MODEL_DEPENDENT_DEFERRED |
| `PHYSICS_IDENTITY`, `NUMERICAL_TOLERANCE` | as declared |

### 5.3 Consistency gates

`CG-01` flight acceleration; `CG-02` ballistic height residual (physics identity + numerical tolerance); `CG-03` takeoff whip (hard; candidate `R_WHIP = R_WHIP_TRUNK = 5.0 rad/s`, OD-06); `CG-04` extension state (hard, model-dependent); support-continuity consistency per scope (see `EVENT_CONTRACT_E1_E12.md` §4).

### 5.4 Diagnostics (never gate)

Force-threshold takeoff/landing times; force-defined flight duration; force/physical
takeoff lead; flight-time height; ballistic height; force-integrated displacement;
raw peak landing `Fz`; loading rate; `Fz` chatter counts; reflight run list; CoP
excursion; maximum foot clearance (left/right); `COM_X` excursion after landing;
`COM_VX` peak after touchdown; `Hy` timeline; root/trunk pitch timeline; regime
timeline; `CONTROLLER_HANDOFF_TIME`.

`LANDING_PEAK_FZ_BW` is **not** a diagnostic: it is the hard gate input for `L4-G6`
(`<= 8.0 BW`). The raw `LANDING_PEAK_FZ` remains a diagnostic.

## 6. Quantities never again conflated (naming audits)

| Retired usage | Successor rule |
|---|---|
| `apex_height` = absolute `COM_Z` | `APEX_COM_Z` (absolute, report-only) vs `COM_RISE_TAKEOFF_TO_APEX` (height, method-qualified) |
| "jump height" without qualifier | prohibited; use the method-qualified names |
| `BALANCE_CAPTURE_COM_SPEED_MPS` applied to `com_vz` | `com_speed_sagittal = sqrt(vx²+vz²)`; never call a single component "COM speed" |
| "support continuity PASS" on a slice | `QUALIFIED`/`NOT_QUALIFIED` with `SUPPORT_CONTINUITY_SCOPE_ID` |
| loading rate = peak/time-to-peak | `LOADING_RATE_MAX_SLOPE` (max slope of `Fz(t)`) diagnostic |
| unqualified "takeoff" | `PHYSICAL_TAKEOFF` or `FORCE_THRESHOLD_TAKEOFF` |

## 7. Owner decisions referenced

| OD | Item |
|---|---|
| OD-01 | `H_MIN` primary performance floor (PF-1/PF-2/PF-3/PF-4) |
| OD-02 | `C_GEOM` geometric clearance tolerance |
| OD-03 | `GENUINE_FLIGHT` dwell |
| OD-04 | L4 landing numeric bounds (`com_vx`, `com_vz`, `Hy`, root/trunk pitch, rates) |
| OD-05 | E11 numeric bounds (`C_11`, `HY_11`, `M_11`) |
| OD-06 | Takeoff-whip rate bounds (`R_WHIP`, `R_WHIP_TRUNK`) |
| OD-07 | `DEPTH_MIN` after RES-83 |
| OD-09 | `F_thr` comparability threshold and sensitivity set |
| OD-10 | Landing asymmetry allowance and bilateral-landing dwell |
