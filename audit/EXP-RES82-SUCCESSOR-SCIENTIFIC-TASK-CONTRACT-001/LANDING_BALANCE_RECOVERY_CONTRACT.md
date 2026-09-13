# LANDING, BALANCE CAPTURE, AND RECOVERY CONTRACT

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **STRUCTURALLY FROZEN**; numeric bounds are owner decisions or
RES-87-deferred calibrations.
DEFECTS CLOSED HERE (contract side): `HIGH-001`, `HIGH-002` (bounds), `HIGH-004`
(entry-state structure), `HIGH-012` (contract side), `MED-002`, `MED-012`, `MED-013`.

This artifact defines the whole-body state requirements for the landing (E10 /
L4), balance capture (E11 / L5), recovery-ready, and stable standing (E12 / L5). It
separates **hard safety/physical gates** from **human-plausibility/task gates** and
prohibits a vertical-only criterion from certifying capture.

---

## 1. Landing event structure

| Concept | Definition | Observable |
|---|---|---|
| `LANDING_FIRST_CONTACT` | first sample after physical takeoff with any foot-floor contact row AND `foot_clearance <= 0` for that foot | geometry |
| `BILATERAL_LANDING_ESTABLISHED` | first time after `LANDING_FIRST_CONTACT` at which both feet have plantar contact rows AND both feet carry `Fz > F_thr`, sustained `D_BL` (OD-10; candidate 0.050 s) | geometry + force |
| `LANDING_ASYMMETRY` | `abs(t_first_contact_left − t_first_contact_right)` and the force-share asymmetry over `[LANDING_FIRST_CONTACT, BILATERAL_LANDING_ESTABLISHED]` | geometry + force |
| `LANDING_PEAK_FZ`, `LANDING_PEAK_FZ_BW` | peak `Fz_whole` in `[LANDING_FIRST_CONTACT, E10_confirmation]` and normalized to body weight | force |
| `LOADING_RATE_MAX_SLOPE` | maximum positive slope of `Fz_whole(t)` in the landing window | force |
| `MAX_PENETRATION` | maximum floor penetration depth of any foot point | geometry |

Nominal symmetric CMJ: slight left-right landing asynchrony is **allowed** within a
declared bound `D_BL` and asymmetry bound (OD-10). Microsecond equality is not
required and is not claimed; no evidence supports exact simultaneity, and the
measurement system cannot resolve it meaningfully. `D_BL` is a task tolerance, not a
human-performance finding.

## 2. Layer L4 — landing capture validity (E10)

L4 is a **whole-body post-impact admissible state**. `abs(com_vz) < V_ABS_TAIL` alone
is explicitly insufficient (HIGH-001).

### 2.1 Hard gates (must all hold for `LANDING_VALID`; category tags per §4)

| Gate | Variable | Requirement / definition | Units | Threshold/status | Provenance | Invalid outcome prevented |
|---|---|---|---|---|---|---|
| `L4-G1` | prior flight + descending COM | prior certified physical flight (E7) and descending COM at first contact | boolean | true / FROZEN | PHYSICS_EVENT_CONTRACT | landing scored without a preceding genuine flight |
| `L4-G2` | fall/prohibited flags | no fall contact and no prohibited contact in `[E9, E10_confirmation]` | boolean | false / FROZEN | PROJECT HARD RULE | landing on a fall shell or support substitute |
| `L4-G3` | bilateral establishment time | `t_bilateral_load − t_LANDING_FIRST_CONTACT ≤ D_EST` (OD-10 candidate 0.050 s), then sustained for `D_BL` | s | OD-10 | TASK (OD-10) | single-foot or delayed-second-foot landing strategy |
| `L4-G4` | `MAX_PENETRATION` | maximum floor penetration in the landing window | m | 0.010 / OD-04 re-approval | PROJECT HISTORICAL AUTHORITY | contact solver beyond its admissible regime |
| `L4-G5` | reflight / chatter | no post-landing reflight beyond approved chatter | boolean/count | reflight=false, chatter≤8 / FROZEN | PROJECT AUTHORITY (`BILATERAL_SUPPORT_CONTINUITY_CONTRACT`) | lost landing hidden by a later nominal frame |
| `L4-G6` | `LANDING_PEAK_FZ_BW` | peak landing whole-body `Fz` normalized to body weight | BW | 8.0 / OD-04 re-approval | PROJECT HISTORICAL HARD RULE | impact beyond the declared landing-force envelope |

### 2.2 Whole-body task gates at E10 confirmation (must all hold)

| Gate | Variable | Definition | Candidate bound | Status |
|---|---|---|---|---|
| `L4-T1` | `COM_VX_AT_E10` | sagittal COM horizontal velocity at E10 confirmation | `abs <= 0.30 m/s` | OD-04 |
| `L4-T2` | `COM_VZ_AT_E10` | RETIRED_REDUNDANT: E10's own predicate already requires `abs(com_vz) < V_ABS_TAIL` continuously through confirmation; the transient operand is `MAX_ABS_COM_VZ_LANDING_WINDOW` (diagnostic) | — | OD-04 |
| `L4-T3` | `HY_AT_E10` | `abs(Hy)` at E10 confirmation | candidate ≤ 5 kg·m²/s | OD-04 (no literature value; engineering bound) |
| `L4-T4` | `ROOT_PITCH_AT_E10` | signed root/pelvis sagittal pitch | candidate `abs <= 0.35 rad` | OD-04 |
| `L4-T5` | `TRUNK_PITCH_AT_E10` | signed trunk sagittal pitch | candidate `abs <= 0.45 rad` | OD-04 (trunk flexion at landing is mechanically permissible and even protective (Blackburn & Padua 2008, 2009); the defect is the un-captured forward lunge, not flexion per se) |
| `L4-T6` | `ROOT_PITCH_RATE_AT_E10` | root pitch rate | candidate `abs <= 3.0 rad/s` | OD-04 |
| `L4-T7` | `TRUNK_PITCH_RATE_AT_E10` | trunk pitch rate | candidate `abs <= 3.0 rad/s` | OD-04 |
| `L4-T8` | `POST_LANDING_MOMENTUM_CAPTURE` | behavioral gate over `[E10_confirmation, E11_onset]`: `max(abs(com_vx(t))) <= max(abs(COM_VX_AT_E10), VX_TAIL_CAP)` AND `COM_X_MAX_AFTER_LANDING := max over [LANDING_FIRST_CONTACT, E11_onset] of (com_x(t) − com_x(LANDING_FIRST_CONTACT)) <= X_FWD_MAX` (one-sided forward); if E11 never onsets the gate is `NOT_EVALUABLE` and `LANDING_VALID=false` | candidates `VX_TAIL_CAP = 0.15 m/s`, `X_FWD_MAX = 0.10 m` | OD-04 |
| `L4-T9` | `MAX_ABS_COM_VX_FIRST_CONTACT_TO_E10` | `max(abs(com_vx))` over `[LANDING_FIRST_CONTACT, E10_confirmation]` | candidate `0.40 m/s` | OD-04 (ADV-7 closure: bounds horizontal momentum during impact absorption) |
| `L4-T10` | `LANDING_TRANSIENT_WINDOW_MAXIMA` | over `[LANDING_FIRST_CONTACT, E10_confirmation]`: `max abs(root_pitch_rate) ≤ 5.0`, `max abs(trunk_pitch_rate) ≤ 5.0`, `max abs(Hy) ≤ 5.0`, plus mid-window posture envelope `max root_pitch ≤ 0.45 rad`, `max trunk_pitch ≤ 0.55 rad` | OD-04 (ADV-4 closure: bound, not a proof; NC-05 gains a decaying-whip fixture; `HY_LAND_W` aligned with `HY_AT_E10`, no evidence supports a larger transient) |

**Rationale for L4-T8.** R001 was vertically arrested but continued accelerating
forward after touchdown (`com_vx` 0.211 → 0.342 m/s) — the "forward lunge". A
point-in-time posture/velocity bound alone can be satisfied by a state that is still
diverging, and a post-confirmation window that is never explicitly declared cannot
exclude it. The behavioral capture gate is therefore defined over the explicit,
non-empty window `[E10_confirmation, E11_onset]` and is evaluated by the L4 layer; the
point-in-time bounds prevent a bad initial landing state. `L4-T9` closes the impact
interval, and `L4-T10` closes the transient-whip gap. All three are task gates, not
clinical claims.

**R001 landing-state evidence.** `Hy=9.88 kg·m²/s`, `root_pitch=0.292 rad`,
`trunk_forward=25.01°` (E11 value; E10 trunk pitch `UNKNOWN_NOT_EVALUABLE` in the sealed summary), `com_vx` peak `0.342 m/s` after touchdown, `peak Fz=4.78 BW`,
`max penetration=0.0096 m`. Under this contract's candidate bounds R001 fails
`L4-T3` and `L4-T8`; under some permissive posture candidates it may pass `L4-T4/T5`,
which is why the behavioral gate exists. The layer is failed by independent causes —
the contract is not tuned to make R001 fail on every line.

## 3. Layer L5 — balance capture (E11) and recovery

### 3.1 `balance_capture` state variables

| Variable | Symbol | Units | Why it is required |
|---|---|---|---|
| sagittal COM speed | `com_speed_sagittal = sqrt(vx² + vz²)` | m/s | the correct scalar (MED-002); a vertically-stopped body with residual `vx` is not captured |
| centroidal angular momentum | `Hy` | kg·m²/s | rotational capture; a body can translate-capture while still rotating |
| support hull margin | `support_margin` | m | COM must be inside the active support hull with positive margin, not at its edge |
| CoP state | `cop_valid`, CoP position | m | the measured control point must exist and lie in the support region |
| bilateral support | `bilateral_plantar`, `bilateral_loaded` | — | capture is a bilateral standing task |
| reflight/fall/prohibited | flags | — | capture cannot be claimed while support is being lost |
| posture/rates (entry) | root/trunk pitch and rates, joint rates | rad, rad/s | recovery-readiness; prevents a captured-velocity state with an untenable posture |

### 3.2 `BALANCE_CAPTURE` gate (E11)

Sustained `D=0.150 s` with:

```
bilateral_plantar AND bilateral_loaded
AND com_speed_sagittal <= C_11
AND abs(Hy) <= HY_11
AND support_margin > M_11
AND cop_valid
AND NOT reflight AND NOT fall_contact AND NOT prohibited_contact
```

| Bound | Candidate | Status | Provenance |
|---|---|---|---|
| `C_11` | 0.20 m/s | OD-05 | Structure informed by TTS/DPSI stabilization logic (Chen 2026; Ross 2005; Wikstrom 2007) and by capture-point reasoning (Hof 2005). Clinical thresholds are NOT imported: those populations/tasks (single-leg hop stabilization in ankle/ACL patients) are not this task. `C_11` is a simulation task bound. |
| `C_11` (alt) | 0.30 m/s | OD-05 | retains the historical numeric (now applied to the correct scalar) |
| `HY_11` | 2 kg·m²/s | OD-05 | no literature threshold exists; engineering bound; must be owner-approved |
| `M_11` | 0.02 m | OD-05 | candidate; margin must be positive; numeric tied to support-hull measurement (RES-84) |

**Research note on dynamic postural stability.** TTS (time for the vertical GRF to
return to and remain within a band, e.g., 5 % body weight) and DPSI (normalized GRF
variance over a 3 s post-impact window) are the established clinical methods
(Chen 2026 systematic review/meta-analysis; Ross & Guskiewicz 2005; Wikstrom 2007).
They support the *structure* used here: (i) capture is a sustained property over a
window, not an instant; (ii) both AP and vertical dimensions matter; (iii) stability
is defined relative to the subject's quiet stance variability. They do **not** supply
numerical thresholds transferable to a deterministic sagittal simulator, and the
anterior-cruciate-ligament patient populations are not this COU. `C_11`, `HY_11`,
`M_11` are therefore marked owner decisions, not literature imports.

### 3.3 `RECOVERY_READY` (state between capture and stable standing)

`RECOVERY_READY(s)` is a reported state, not a separate DAG event (avoiding gate
proliferation):

```
E11 predicate holds
AND posture within the recovery-entry set (RES-87-calibrated; concept frozen here)
AND posture rates within the recovery-entry rate bounds (RES-87)
AND CoP inside the recovery-entry hull fraction (RES-87)
AND no penetration issue during [E9, s]
```

- It answers: *is the system in a dynamically admissible state from which standing
  recovery can begin without an extreme corrective maneuver?*
- `RECOVERY_READY_TIME` is a mandatory reported metric.
- L5 requires `RECOVERY_READY_TIME <= E12_onset`; a trajectory that never becomes
  recovery-ready cannot be recovered by later good behavior.
- **Non-retroactivity:** an eventual E12 confirmation cannot validate a bad E10/E11
  state; L4 and L5 are evaluated on their own windows.
- **Recovery maintenance:** over `[E11_confirmation, E12_confirmation]` the E11 capture
  bounds must continue to hold at every sample; a capture→excursion→re-stabilize
  sequence fails `RECOVERY_VALID` (closes the F6 scope gap).
- **Post-recovery observation:** `RECOVERY_VALID` additionally requires that, after
  E12 confirmation, the episode continues for at least `POST_E12_OBSERVATION_MARGIN`
  (candidate 0.250 s; OD-13) with no fall, no reflight, and continued envelope
  membership. A horizon ending at E12 confirmation cannot certify recovery
  (ADV-5 closure), and state freezes/overrides fail `NO_ARTIFICIAL_SUPPORT`
  (NC-12).

### 3.4 `STABLE_RECOVERY` / stable standing (E12)

`standing_envelope(s)` is a **physical robust-standing predicate**, calibrated by
RES-87, with these mandatory components:

| Component | Requirement | Deferred detail |
|---|---|---|
| POSTURE | joint configuration inside the standing posture set derived from multiple perturbed holds | numeric intervals (RES-87) |
| VELOCITY | `com_speed_sagittal` and joint rates below declared quiet-standing bounds | numeric bounds (RES-87) |
| ANGULAR MOMENTUM | `abs(Hy)` below declared bound | numeric (RES-87) |
| SUPPORT | bilateral plantar support; `Fz_whole` within a declared band around body weight | band (RES-87) |
| CoP | valid and inside the standing hull fraction | fraction (RES-87) |
| MARGIN | `support_margin > 0` with a declared minimum | numeric (RES-87) |
| NO_FALL / NO_REFLIGHT / NO_PROHIBITED | flags false for the entire dwell | — |
| DWELL | `PHYSICAL_TIME 0.500 s` | — |

**Calibration requirement (RES-87).** The envelope must be derived from a family of
perturbed initial standing states with a declared robustness target: the system must
remain in (or return to) the envelope after the approved perturbations. The following
are **prohibited definitions**:

1. equality to one historical deterministic standing trace;
2. micron-scale windows derived from a single hold;
3. ULP-adjacent bound expansion presented as robustness;
4. a dwell that begins under one controller regime and is certified under another
   without separate reporting (`CONTROLLER_HANDOFF_TIME`,
   `CONTROLLER_REGIME_AT_E12_ONSET` are mandatory).

**R001 failure.** R001's E12 used a one-trajectory ULP envelope (HIGH-012), began
under `SETTLE` 50.8 ms before the declared handoff (MED-013), required a 13.15 s
recovery from a 24° forward lean, and its full-episode support adjudication was
`NOT_QUALIFIED` (MED-012). Under this contract R001 fails L5 on envelope robustness
and on full-episode support continuity — independent causes.

## 4. Hard safety vs human-plausibility separation (normative)

| Category | Examples | Meaning of failure |
|---|---|---|
| HARD SAFETY / PHYSICAL | fall, prohibited contact, penetration, reflight, non-finite, joint/actuator limits | physically invalid or unsupported simulation; cannot be a task success |
| HUMAN-PLAUSIBILITY / TASK | landing `vx`/`Hy`/pitch bounds, capture speed, recovery-entry posture, standing envelope | the simulation is physically consistent but does not represent an admissible loaded-CMJ task execution |

The distinction is reported explicitly so that no human-plausibility gate is
presented as a physical impossibility claim, and no physical failure is hidden as a
"style" issue.

## 5. Layer outputs and non-retroactivity

| Layer | Window | Result |
|---|---|---|
| `LANDING_VALID` (L4) | point-in-time at `E10_confirmation` (L4-T1..T7) + `[LANDING_FIRST_CONTACT, E10_confirmation]` (L4-T9/T10) + `[E10_confirmation, E11_onset]` (L4-T8) + hard-gate windows | boolean; `L4-T8` `NOT_EVALUABLE` if E11 never onsets ⇒ false |
| `BALANCE_VALID` (L5) | `[E10_confirmation, E11_confirmation]` | boolean from §3.2 |
| `RECOVERY_VALID` (L5) | `[E11_confirmation, E12_confirmation or horizon]` AND `FULL_EPISODE` `QUALIFIED` AND the post-E12 observation margin | boolean from §3.3–3.4 |

`TASK_SUCCESS` requires all three. `EVENT_CHAIN_VALID` alone never implies
`LANDING_VALID`/`BALANCE_VALID`/`RECOVERY_VALID` or `TASK_SUCCESS`.
