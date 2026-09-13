# EVENT_CONTRACT_E1_E12 — Successor Event Definition

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **STRUCTURALLY FROZEN** (numeric items flagged `OWNER_DECISION_REQUIRED` or
`MODEL_DEPENDENT_DEFERRED` per `OWNER_DECISIONS_REQUIRED.md` and
`MODEL_DEPENDENCY_MATRIX.json`).
DEFECTS CLOSED HERE: `HIGH-010`, `HIGH-011`, `MED-001` (via `DWELL_SEMANTICS.md`),
`MED-002`, `MED-012`, `MED-013`, `MED-020`, and the conceptual defect
`EVENT_CHAIN_COMPLETION != TASK_SUCCESS`.

The twelve events form a **monotone DAG**: each event has exactly one predecessor and
cannot latch before its predecessor. **Event-chain completion is necessary but not
sufficient for task success** (`SUCCESSOR_TASK_CONTRACT.md`).

---

## 0. Shared vocabulary (normative)

Every helper below is a declared measurement on a physics sample `s`. No event may use
an undeclared helper.

| Helper | Definition |
|---|---|
| `fall_contact(s)` | any fall-shell collision geom has a floor contact row or carries normal load in `s` |
| `prohibited_contact(s)` | any non-plantar, non-fall geom (load, torso, pelvis, limb shells) contacts the floor or supports load in `s`; includes any support substitute |
| `plantar_contact(s,foot)` | the foot's plantar box has ≥1 floor contact row in `s` |
| `foot_clearance(s,foot)` | signed minimum distance of the lowest foot geometry point to the floor plane; `> 0` = separated (correct rotated-foot geometry; MED-005/HIGH-009 class) |
| `Fz(s,foot)` | virtual-plate normal force for that foot in `s` |
| `Fz_whole(s)` | `Fz(left)+Fz(right)` plus any other measured vertical load; support-substitute contact is a blocker regardless |
| `BW` | `SYSTEM_WEIGHT = m_system · g = 95.0 kg · 9.81 m/s² = 931.95 N` — the single normative body-weight quantity (total system weight, athlete + load); never athlete-only weight |
| `g` | 9.81 m/s² (project V2 constant, identical to the Plant gravity) |
| `F_thr` | declared per-foot comparability force threshold (candidate 10 N; OD-09); NEVER defines physical takeoff/flight |
| `bilateral_loaded(s)` | `Fz(s,left) > F_thr` and `Fz(s,right) > F_thr`; used only where declared |
| `bilateral_plantar(s)` | `plantar_contact(s,left)` and `plantar_contact(s,right)` |
| `no_nonplantar_support(s)` | `not prohibited_contact(s)`; no fall-shell loading; no artificial support force and no state/integration override outside the Plant |
| `com_z(s)`, `com_vz(s)`, `com_vx(s)` | whole-body COM state from the Plant (model truth) |
| `com_speed_sagittal(s)` | `sqrt(com_vx² + com_vz²)` — the ONLY quantity that may be called COM speed in this sagittal model |
| `Hy(s)` | whole-body centroidal angular momentum about the sagittal `y` axis |
| `support_margin(s)` | signed distance from COM ground projection to the boundary of the **active** support hull (≥1 active contact); positive inside; must be ≤ 0 during true flight (HIGH-008 class) |
| `cop_valid(s)` | CoP valid under the declared per-foot validity threshold; per-foot validities and the resultant CoP are reported; frame per RES-84 (PD-05) |
| `root_pitch(s)`, `root_pitch_rate(s)` | pelvis/root sagittal pitch/rate (signed) |
| `trunk_pitch(s)`, `trunk_pitch_rate(s)` | trunk sagittal pitch/rate (signed; MED-004 class) |
| `reflight(s)` | `CANONICAL_REFLIGHT` (force-only, RES-57): whole `Fz < 10.0 N` for `N_true ≥ 4` consecutive physics samples (**declared SAMPLE_COUNT semantics**; span 0.375 ms = 3 intervals; the historical 0.5 ms wording counted 4 intervals and is recorded as an erratum). No separation evidence required; window-bound after E9 confirmation. Chatter transitions are a separate count |
| `finite(s)` | all declared state and measurement values finite |
| `standing_envelope(s)` | physical robust-standing predicate calibrated by RES-87 over perturbed holds (never one deterministic trace, HIGH-012 class) |
| `C_GEOM` | declared geometric clearance tolerance derived from contact/numerical tolerances (OD-02; NEVER a human-performance number) |
| `V_DESC` | vertical descent-rate threshold on `abs(com_vz)` at first landing contact (candidate 0.10 m/s; OD-08) |
| `V_ABS_TAIL` | residual vertical speed `abs(com_vz)` defining impact arrest (candidate 0.05 m/s; OD-08) |
| `E11_ONSET` | `ONSET_SAMPLE` of the **confirmed** E11 run; must satisfy `E11_onset ≥ E10_confirmation`; if E11 never confirms, the L4-T8 window is `NOT_EVALUABLE` and `LANDING_VALID=false` (closing bound of the L4-T8 behavioral window) |

**Prohibited constructions** (mechanically checked by `validate_contract.py`):
`"com speed"` may not be used for a single component; `"takeoff"` may not appear
unqualified where force/physical ambiguity exists; `"jump height"` may not appear
without a method qualifier.

## 1. Event table (normative)

### E1 — `supported_start`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The athlete is in a valid quiet bilateral standing state on the floor and has not fallen. |
| PREDECESSOR | none |
| ONSET_PREDICATE | `finite` AND `bilateral_loaded` AND `Fz_whole > 0.30·BW` AND `support_margin > 0.02 m` AND `abs(trunk_pitch) < 0.1745 rad` AND `abs(com_vz) < 0.05 m/s` AND `abs(com_vx) < 0.05 m/s` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | continuous run with `DWELL_TYPE=PHYSICAL_TIME`, `D=0.100 s` (`K_D=800`) per `DWELL_SEMANTICS.md` |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.100 s |
| REQUIRED_MEASUREMENTS | `Fz(left/right)`, `Fz_whole`, `com_z`, `com_vx`, `com_vz`, `support_margin`, `trunk_pitch`, `fall_contact`, `prohibited_contact`, finiteness |
| FAILURE_BLOCKERS | `fall_contact` true at any sample (MED-020 closure); `prohibited_contact` true; `support_margin ≤ 0`; non-finite state; no bilateral plantar support; internal-quiescence violation (joint-rate/CoP placeholder; numeric RES-84) |
| NEGATIVE_CONTROLS | NC-04, NC-11 (initial horizontal momentum) |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The trajectory began from a physically supported, quiescent standing state, not from a fall shell." |
| NUMERIC | `0.30·BW`, `0.02 m`, `0.1745 rad`, `0.05 m/s` (`com_vz` and `com_vx`) — OWNER_TASK_DECISION, **OD-13** |

### E2 — `countermovement_onset`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The athlete begins the deliberate downward (eccentric) motion of the countermovement while supported. |
| PREDECESSOR | E1 |
| ONSET_PREDICATE | `com_vz < −0.08 m/s` AND `Fz_whole > 0.30·BW` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | `com_vz < −0.03 m/s` (strictly weaker than onset; onset implies sustain) |
| CONFIRMATION_PREDICATE | run with `D=0.030 s` (`K_D=240`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.030 s |
| REQUIRED_MEASUREMENTS | `com_vz`, `Fz_whole`, fall/prohibited flags |
| FAILURE_BLOCKERS | support loss before confirmation; fall; prohibited contact |
| NEGATIVE_CONTROLS | NC-01 (a squat jump / static impulse never has a supported negative COM-velocity run after standing) |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "A countermovement was initiated from supported standing." |

### E3 — `valid_countermovement`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The countermovement reached a meaningful depth while supported: a genuine eccentric phase, not a token dip. |
| PREDECESSOR | E2 |
| ONSET_PREDICATE | `(com_z(E1 reference) − com_z) > DEPTH_MIN` AND `Fz_whole > 0.30·BW` AND `bilateral_plantar` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | not applicable (instantaneous latch) |
| CONFIRMATION_PREDICATE | first sample satisfying ONSET_PREDICATE |
| DWELL_TYPE / DWELL_VALUE | NONE / — |
| REQUIRED_MEASUREMENTS | `com_z` relative to E1 standing reference (`INITIAL_STANDING_COM_Z`), `Fz_whole`, plantar contacts, fall/prohibited flags |
| FAILURE_BLOCKERS | depth never reached before reversal; fall; prohibited contact |
| NEGATIVE_CONTROLS | NC-01 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The movement contained a genuine countermovement depth, not a hop or a threshold artifact." |
| NUMERIC | `DEPTH_MIN` candidate 0.10 m; status `MODEL_DEPENDENT_DEFERRED` (OD-07). Depth concept is Plant-independent; its numeric realization depends on the corrected hip/knee/ankle ranges of RES-83. |

### E4 — `upward_reversal`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The COM vertical velocity reverses from negative to positive under support (eccentric→concentric transition). |
| PREDECESSOR | E3 |
| ONSET_PREDICATE | `com_vz > +0.02 m/s` after a confirmed negative-`com_vz` countermovement run |
| SUSTAIN_PREDICATE | `com_vz > 0` |
| CONFIRMATION_PREDICATE | run with `D=0.010 s` (`K_D=80`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.010 s |
| REQUIRED_MEASUREMENTS | `com_vz`, support state |
| FAILURE_BLOCKERS | support loss; fall; non-finite |
| NEGATIVE_CONTROLS | NC-01 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "A true reversal existed between eccentric and concentric phases." |

### E5 — `vertical_propulsion`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The athlete produces sustained net-positive upward propulsion while bilaterally supported. |
| PREDECESSOR | E4 |
| ONSET_PREDICATE | `com_vz > 0` AND `Fz_whole > 1.05·BW` AND `bilateral_plantar` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | run with `D=0.050 s` (`K_D=400`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.050 s |
| REQUIRED_MEASUREMENTS | `com_vz`, `Fz_whole`, plantar contacts, prohibited flag |
| FAILURE_BLOCKERS | unilateral support during the run; fall; prohibited contact; premature physical takeoff before the dwell |
| NEGATIVE_CONTROLS | NC-02 (force dropout without physical separation is still not flight, and can end propulsion) |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "A sustained propulsive phase existed under bilateral support." |

### E6 — `physical_takeoff`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The instant the system physically leaves the floor: all foot-floor contacts have ended with positive geometric clearance and upward COM velocity, and no other support exists. |
| PREDECESSOR | E5 |
| ONSET_PREDICATE | `(not plantar_contact(s,left))` AND `(not plantar_contact(s,right))` AND `foot_clearance(s,left) > C_GEOM` AND `foot_clearance(s,right) > C_GEOM` AND `com_vz > 0` AND `not fall_contact` AND `no_nonplantar_support` |
| SUSTAIN_PREDICATE | not applicable (instantaneous physical-state latch) |
| CONFIRMATION_PREDICATE | first sample satisfying ONSET_PREDICATE (`DWELL_TYPE=NONE`) |
| DWELL_TYPE / DWELL_VALUE | NONE / — |
| REQUIRED_MEASUREMENTS | physical contact rows both feet, `foot_clearance` both feet, `com_vz`, fall/prohibited flags |
| FAILURE_BLOCKERS | fall contact as substitute support; prohibited contact; clearance condition never met |
| NEGATIVE_CONTROLS | NC-02 (force dropout with retained contact is NOT physical takeoff) |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "Physical flight began here; this is not a force-threshold artifact." |
| NOTES | `FORCE_THRESHOLD_TAKEOFF` (measurement observable, `D=0.010 s` at `F_thr`) is reported separately and NEVER defines E6 (HIGH-010 closure). Sustain of the no-contact condition is owned by E7 so no predicate depends on future events. |

### E7 — `genuine_flight`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | Sustained physical flight: both feet are physically separated from the floor continuously for the declared dwell — the executable definition of genuine flight. |
| PREDECESSOR | E6 |
| ONSET_PREDICATE | `(not plantar_contact(s,left))` AND `(not plantar_contact(s,right))` AND `foot_clearance(s,left) > C_GEOM` AND `foot_clearance(s,right) > C_GEOM` AND `no_nonplantar_support` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | run with `D=0.050 s` (`K_D=400`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.050 s (candidate; OD-03) |
| REQUIRED_MEASUREMENTS | contact rows, `foot_clearance` (both feet), fall/prohibited flags |
| FAILURE_BLOCKERS | any re-contact inside the dwell; fall; prohibited; clearance violation |
| NEGATIVE_CONTROLS | NC-02, NC-08 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The feet were genuinely off the floor for a sustained interval; no geometry is declared that is not executed." |
| NOTES | The historical `GENUINE_FLIGHT_GAP_M=0.010 m` force-only declaration is retired. The geometric condition is executable and separated from the force-threshold observable. |

### E8 — `apex`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The deterministic instant of maximum COM height during physical flight, where `com_vz` crosses from positive to non-positive. |
| PREDECESSOR | E7 |
| ONSET_PREDICATE | sample pair `(s_prev, s_cur)` in physical flight (`no plantar contact` AND `foot_clearance > C_GEOM` both feet at both samples) with `com_vz(s_prev) > 0` AND `com_vz(s_cur) ≤ 0` |
| SUSTAIN_PREDICATE | not applicable |
| CONFIRMATION_PREDICATE | `APEX_TIME = t_prev + (com_vz(s_prev)/(com_vz(s_prev) − com_vz(s_cur)))·(t_cur − t_prev)`; `APEX_COM_Z` from the same linear interpolation of `com_z` over the bracketing pair; the crossing must lie strictly inside the physical-flight interval |
| DWELL_TYPE / DWELL_VALUE | NONE / — |
| REQUIRED_MEASUREMENTS | `com_vz`, `com_z`, contact rows, clearance, time |
| FAILURE_BLOCKERS | crossing detected while any contact exists (NC-03); crossing outside the certified flight interval; fall during flight |
| NEGATIVE_CONTROLS | NC-03 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The apex is a deterministic ballistic zero-crossing inside physical flight." |
| NOTES | `APEX_DWELL_REQUIRED = NO`. The historical `APEX_DWELL_S=0.005` declaration was never implemented; a mathematical apex is an instant and no dwell is scientifically required. HIGH-011 is closed by correct zero-crossing semantics plus the flight guard, not by inventing a dwell. |

### E9 — `descending_landing`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | First physical re-contact with the floor after flight, with the COM still descending: the landing begins. |
| PREDECESSOR | E8 |
| ONSET_PREDICATE | `(plantar_contact(s,left) or plantar_contact(s,right))` AND `com_vz < −V_DESC` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | `plantar_contact(s,left) or plantar_contact(s,right)` (contact retained) |
| CONFIRMATION_PREDICATE | run with `D=0.010 s` (`K_D=80`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.010 s |
| REQUIRED_MEASUREMENTS | plantar contacts (both feet), `com_vz`, fall/prohibited flags, geometric first-contact time |
| FAILURE_BLOCKERS | fall-shell landing; prohibited contact; no contact ever established after flight |
| NEGATIVE_CONTROLS | NC-03 (contact-phase zero-crossing is not apex), NC-08 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The flight ended in a physical bilateral-capable foot landing." |
| NOTES | `LANDING_FIRST_CONTACT` (geometric) and `BILATERAL_LANDING_ESTABLISHED` are separate metrics (`PERFORMANCE_METRIC_CONTRACT.md`). `V_DESC` is a vertical descent-rate threshold on `abs(com_vz)` (OD-08). |

### E10 — `impact_absorption`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | After first landing contact the athlete establishes bilateral support and arrests the downward COM motion, producing a whole-body post-impact admissible state. |
| PREDECESSOR | E9 |
| ONSET_PREDICATE | `bilateral_plantar` AND `bilateral_loaded` AND `abs(com_vz) < V_ABS_TAIL` AND `not reflight` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | run with `D=0.020 s` (`K_D=160`); at confirmation the point-in-time L4 state (L4-T1..T7) is evaluated. The behavioral L4-T8 window is `[E10_confirmation, E11_onset]` and is evaluated by the L4 layer, not by the E10 event latch. |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.020 s |
| REQUIRED_MEASUREMENTS | `com_vz`, bilateral force/contact, `reflight`, fall/prohibited flags, plus the full L4 landing state (`com_vx`, `Hy`, root/trunk pitch and rates) at confirmation |
| FAILURE_BLOCKERS | reflight within the run; fall/prohibited; unilateral-only support at confirmation; layer landing-gate failure (layer-owned; does not unlatch E10) |
| NEGATIVE_CONTROLS | NC-05 (pitched/lunging landing), NC-08 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The impact was absorbed into an admissible whole-body landing state." |
| NOTES | The local predicate is deliberately minimal; the whole-body criteria live in the L4 layer so that a vertical-only `abs(com_vz)` can never certify impact absorption (HIGH-001 closure). |

### E11 — `balance_capture`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The whole-body state is dynamically captured: sagittal COM speed small, centroidal angular momentum bounded, COM inside the active support hull with margin, CoP valid, bilateral support. |
| PREDECESSOR | E10 |
| ONSET_PREDICATE | `bilateral_plantar` AND `bilateral_loaded` AND `com_speed_sagittal ≤ C_11` AND `abs(Hy) ≤ HY_11` AND `support_margin > M_11` AND `cop_valid` AND `not reflight` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | run with `D=0.150 s` (`K_D=1200`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.150 s |
| REQUIRED_MEASUREMENTS | `com_vx`, `com_vz`, `com_speed_sagittal`, `Hy`, `support_margin`, `cop_valid`, bilateral support, reflight/fall/prohibited |
| FAILURE_BLOCKERS | `com_vx` outside bound while `com_vz` small (NC-06); `Hy` outside bound; no support margin; CoP invalid; reflight; fall |
| NEGATIVE_CONTROLS | NC-06 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The landing momentum was captured; the athlete is not merely vertically stopped." |
| NOTES | `BALANCE_CAPTURE_COM_SPEED_MPS` is retired as a name/implementation mismatch (MED-002). The successor quantity is explicitly `com_speed_sagittal = sqrt(vx²+vz²)`; numeric bounds OD-05, informed by TTS/DPSI structure but NOT imported as clinical thresholds (Chen 2026; Ross 2005; Wikstrom 2007; Hof 2005). `E11_onset` closes the L4-T8 behavioral window; no separate latency bound is needed because L4-T8 bounds the momentum over the entire inter-window interval. |

### E12 — `stable_recovery`

| Field | Value |
|---|---|
| SCIENTIFIC_MEANING | The athlete is in the robust physical standing envelope continuously for the declared dwell: quiet bilateral support with no fall and no reflight. |
| PREDECESSOR | E11 |
| ONSET_PREDICATE | `standing_envelope(s)` AND `not reflight` AND `not fall_contact` AND `not prohibited_contact` |
| SUSTAIN_PREDICATE | identical to ONSET_PREDICATE |
| CONFIRMATION_PREDICATE | run with `D=0.500 s` (`K_D=4000`) |
| DWELL_TYPE / DWELL_VALUE | PHYSICAL_TIME / 0.500 s |
| REQUIRED_MEASUREMENTS | posture vector, `com_z`, `com_vx/vz`, `Hy`, joint rates, bilateral support forces, `support_margin`, `cop_valid`, fall/reflight/prohibited flags, dwell |
| FAILURE_BLOCKERS | envelope exit; fall; reflight; prohibited contact; CoP invalid |
| NEGATIVE_CONTROLS | NC-07 (brittle standing), NC-08 |
| PRIMARY_OR_DIAGNOSTIC | PRIMARY |
| CLAIM_SUPPORTED | "The athlete recovered to a robust physical standing envelope — a standing state, not an equality to one historical trace." |
| NOTES | `standing_envelope` must be calibrated by RES-87 from multiple perturbed holds with a declared robustness target; the historical micron-scale one-trajectory/ULP envelope and any equality to a single nominal trace are retired (HIGH-012). E12 is a **physical-state** event, not a controller-mode event: it may confirm in any controller regime whose physical state satisfies the predicate; the controller handoff time is reported separately (`CONTROLLER_HANDOFF_TIME`) and the final-regime holding requirement is a candidate-credibility item (L6), never retroactively excusing a bad L4/L5 state (MED-013 closure, Option B). |

## 2. Monotone DAG and non-retroactivity

```
E1 → E2 → E3 → E4 → E5 → E6 → E7 → E8 → E9 → E10 → E11 → E12
```

- Events latch in this order only; no event may latch before its predecessor is
  confirmed.
- Latching a later event **never** repairs an earlier layer failure: layer results
  (`SUCCESSOR_TASK_CONTRACT.md`) are evaluated on their own windows and cannot be
  overridden by `EVENT_CHAIN_VALID=true`.
- Dwells are physics-sample dwells per `DWELL_SEMANTICS.md`.
- E6/E7/E8/E9 use physical contact geometry, never the force-threshold observable.

## 3. Force-threshold observables (measurement only)

The following are **reported** for experimental comparability and cross-checking; none
may be used as a hard truth or as a physical-state predicate:

| Observable | Definition |
|---|---|
| `FORCE_THRESHOLD_TAKEOFF_TIME` | first sample with `Fz(left) < F_thr` AND `Fz(right) < F_thr` sustained `0.010 s` (`PHYSICAL_TIME`) |
| `FORCE_DEFINED_FLIGHT_DURATION` | duration between `FORCE_THRESHOLD_TAKEOFF_TIME` and `FORCE_THRESHOLD_LANDING_TIME` |
| `FORCE_THRESHOLD_LANDING_TIME` | first sample after takeoff with `Fz(left) > F_thr` OR `Fz(right) > F_thr` sustained `0.010 s` |
| `FIRST_TRANSIENT_FORCE_DROPOUT_TIME` | first sample with transient bilateral force below `F_thr` regardless of contact (R001 defect evidence) |
| `SUSTAINED_FORCE_OFF_TIME` | first sample of the final sustained bilateral below-`F_thr` run that is not interrupted by above-threshold force before landing |

`F_thr` candidate 10 N (OD-09). Sensitivity across {1, 5, 10, 20 N, noise-based PkRes}
must be reported (Smith 2024; Pérez-Castilla 2022); the choice is documented as a
comparability convention, and threshold-induced changes in force-derived jump-height
estimates are treated as a data-processing sensitivity, not as physics (Eythorsdottir
2026).

## 4. Support-continuity scopes (MED-012 closure)

Every reported support-continuity result MUST carry `SUPPORT_CONTINUITY_SCOPE_ID` and
window `[t0, t1)`:

| SCOPE_ID | Window |
|---|---|
| `SUPPORTED_PHASE` | `[E1_confirmation, E6)` |
| `PROPULSION` | `[E5_onset, E6)` |
| `TAKEOFF_TRANSITION` | `[E5_confirmation, E7_confirmation]` |
| `FLIGHT` | `[E6, E9)` |
| `LANDING_CAPTURE` | `[E9, E11_confirmation)` |
| `RECOVERY` | `[E11_confirmation, E12_confirmation]` |
| `FULL_EPISODE` | `[t_0, E12_confirmation]` or `[t_0, horizon]` if E12 does not confirm |

Rules:

1. A result phrase "support continuity PASS" is **invalid**; results are
   `QUALIFIED` / `NOT_QUALIFIED` / `NOT_EVALUABLE` per scope.
2. `TASK_SUCCESS` requires the `FULL_EPISODE` scope to be `QUALIFIED`. `RECOVERY_VALID`
   is false unless `FULL_EPISODE` is `QUALIFIED`.
3. The top-level `FULL_EPISODE` field must agree with the `FULL_EPISODE` entry in
   `SUPPORT_CONTINUITY_SCOPE_RESULTS`.
4. If a closing event does not confirm, the window closes at the episode horizon and
   the scope result is `NOT_EVALUABLE` (never silently `QUALIFIED`); `t_0` is the
   episode start sample.
5. Post-landing slice results may be reported only as `LANDING_CAPTURE`/`RECOVERY`
   diagnostics and never as full-episode qualification.
6. The RES-57 `BILATERAL_SUPPORT_CONTINUITY_CONTRACT` sample/episode definitions
   remain the measurement layer; this contract supplies the scope binding it lacked.

## 5. Event numeric provenance register

Every numeric constant appearing in an event predicate carries exactly one provenance
class. This register is normative and mechanically checked.

| Event | Constant | Provenance class |
|---|---|---|
| E1 | `0.30·BW`, `0.02 m`, `0.1745 rad`, `0.05 m/s` (`com_vz`), `0.05 m/s` (`com_vx`) | OWNER_TASK_DECISION (OD-13; historical project values retained; `com_vx` bound added to close ADV-7) |
| E2 | `−0.08 m/s` onset, `−0.03 m/s` sustain, `0.30·BW` | OWNER_TASK_DECISION (historical project values retained) |
| E3 | `0.30·BW`; `DEPTH_MIN` | OWNER_TASK_DECISION (as E1); MODEL_DEPENDENT_DEFERRED (OD-07) |
| E4 | `+0.02 m/s` onset; `0 m/s` sustain | OWNER_TASK_DECISION (historical); PHYSICS_IDENTITY (sign reversal) |
| E5 | `1.05·BW` | PHYSICS_IDENTITY (net positive upward impulse) with OWNER_TASK_DECISION margin |
| E6 | `C_GEOM` | NUMERICAL_TOLERANCE (OD-02) |
| E7 | `C_GEOM`; `0.050 s` | NUMERICAL_TOLERANCE (OD-02); OWNER_TASK_DECISION (OD-03) |
| E8 | zero crossing | PHYSICS_IDENTITY (no numeric constant) |
| E9 | `V_DESC = 0.10 m/s` | OWNER_TASK_DECISION (OD-08) |
| E10 | `V_ABS_TAIL = 0.05 m/s` | OWNER_TASK_DECISION (OD-08) |
| E11 | `C_11`, `HY_11`, `M_11` | OWNER_TASK_DECISION (OD-05) |
| E12 | `0.500 s` dwell; envelope numerics | OWNER_TASK_DECISION (structural); MODEL_DEPENDENT_DEFERRED (OD-12) |

## 6. Non-event dechatter rules (project authority, unchanged)

`REFLIGHT` and `CHATTER_TRANSITION` are force-only project dechatter rules retained
from RES-57; they are not physical-state definitions and do not conflict with OD-11:

- `REFLIGHT` = whole `Fz < 10.0 N` for `N_true ≥ 4` consecutive samples (declared
  SAMPLE_COUNT semantics; span 0.375 ms), force-only, no separation evidence
  required; the historical 0.5 ms wording is an erratum (see `DWELL_SEMANTICS.md` §10).
- `CHATTER_TRANSITION` = whole-Fz crossing of the 10.0 N threshold; maximum 8.
- The successor dwell arithmetic (integer sample distance) is used where a PHYSICAL_TIME
  span is declared; REFLIGHT is explicitly a SAMPLE_COUNT rule, and the historical
  "4 steps = 0.5 ms" wording is recorded as an erratum rather than silently rewritten.

## 7. Regime/handoff semantics (MED-013 closure)

- Scientific events describe **physical state**, not controller mode (Option B,
  adjudicated).
- `CONTROLLER_HANDOFF_TIME` and `CONTROLLER_REGIME_AT_E12_ONSET` are mandatory reported
  metadata.
- A candidate may claim L5 recovery only on the physical predicate; the L6 candidate
  credibility layer separately requires the declared final standing regime to hold the
  standing state (or the claim is reduced to "physical standing under mixed regimes").
- No event time is allowed to acquire meaning from the internal mode timeline.

## 8. Dwell summary

See `DWELL_SEMANTICS.md` §7. `MED-001` is closed by the PHYSICAL_TIME integer-index
convention; the retired one-sample-early convention is explicitly listed as invalid.

## 9. Negative-control anchors

| Event | Negative controls |
|---|---|
| E1 | NC-04 |
| E2 | NC-01 |
| E3 | NC-01 |
| E4 | NC-01 |
| E5 | NC-02 |
| E6 | NC-02 |
| E7 | NC-02, NC-08 |
| E8 | NC-03 |
| E9 | NC-03, NC-08 |
| E10 | NC-05, NC-08 |
| E11 | NC-06 |
| E12 | NC-07, NC-08 |

## 10. R001 failure demonstrations (this contract, without modifying R001)

| Event | R001 evidence | Successor verdict |
|---|---|---|
| E6/E7 | force chatter 0.640–0.64775 s with contact registrations; last contact registration inside the pre-flight window | PHYSICAL_TAKEOFF/FLIGHT semantics not demonstrated by the historical force-only E6/E7 |
| E8 | apex fallback path could latch without the flight guard | not independently auditable as guarded; negative control NC-03 required |
| E10 | `Hy=9.88 kg·m²/s`, root pitch `0.292 rad` at E10; trunk forward `25.01°` is an **E11** value (E10 trunk pitch `UNKNOWN_NOT_EVALUABLE`); peak forward `com_vx 0.342 m/s` after touchdown | FAIL (L4 whole-body bounds; candidate OD-04) |
| E11 | historical guard used `abs(com_vz)`; actual sagittal COM speed included `vx≈0.279 m/s`; `com_vx` still accelerating at E11 | FAIL / not demonstrated (MED-002) |
| E12 | one-trajectory ULP envelope; 13.15 s recovery from a 24° forward lean; E12 dwell began under `SETTLE` | FAIL (HIGH-012, MED-013) |
| FULL_EPISODE | support adjudication historical `NOT_QUALIFIED`; chatter 188 transitions; reflight runs `[4, 8, 1815]` | FAIL (MED-012) |

Exact retrospective accounting is in `CONTRACT_CONSISTENCY_REPORT.md` §R001 and in the
mission receipt; unknown historical quantities are marked `UNKNOWN_NOT_EVALUABLE`
rather than assumed.
