# OWNER_DECISIONS_REQUIRED — Successor Loaded-CMJ Task Contract

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **OPEN ITEMS** (12 decisions). The structural contract is frozen without them;
no candidate may be classified `TASK_SUCCESS=true` while an affected decision is open.
Each decision names its evidence row(s) in `SCIENTIFIC_CONTRACT_EVIDENCE_TABLE.md`.

Decision classes (per `SUCCESSOR_TASK_CONTRACT.md` §9): `LITERATURE_DIRECT`,
`LITERATURE_INFORMED`, `PHYSICS_IDENTITY`, `NUMERICAL_TOLERANCE`,
`OWNER_TASK_DECISION`, `MODEL_DEPENDENT_DEFERRED`. Every open decision below is either
`OWNER_TASK_DECISION` or `MODEL_DEPENDENT_DEFERRED` awaiting owner authorization.

---

## OD-01 — Primary performance floor `H_MIN`

- **Variable:** `COM_RISE_TAKEOFF_TO_APEX`
- **Options:** PF-1 = 0.150 m (recommended); PF-2 = 0.200 m; PF-3 = 0.100 m;
  PF-4 = no numeric floor (structural gate only).
- **Evidence:** S08 (Kraska 2009: 20 kg CMJ ≈ 27.6 ± 8.6 cm in the strongest group,
  lower in weaker strata; 0 kg CMJ 28–34 cm), S10 (CMJ 0.38–0.73 m unloaded range),
  S11 (CMJ 37.8 ± 4.1 cm young male athletes), S09 (load reduces performance but
  preserves pattern at 10–40 % BM).
- **Tradeoff:** PF-1 excludes a trivial hop by ~2× while staying below the weakest
  plausible loaded stratum; PF-2 risks becoming a capability target; PF-3 keeps a
  weak claim.
- **Recommendation:** PF-1 (0.150 m), anchored to the published 20 kg CMJ scale; the
  weak-stratum 20 kg value is *derived* (not directly reported) by applying the weak
  group's SJ percent decrease to its 0 kg mean; the human flight-time vs direct-COM
  cross-family limitation is disclosed. After RES-83 lands, confirm the adopted floor
  is still a *task* floor; a rebuilt model that cannot reach it is a Plant/controller
  finding, not a contract edit.
- **Affected gates:** `PRIMARY_PERFORMANCE_GATE`, `TASK_PERFORMANCE_VALID`, NC-01.

## OD-02 — Geometric clearance tolerance `C_GEOM`

- **Variable:** `foot_clearance` floor used by E6/E7.
- **Options:** (a) contact/numerical-tolerance-derived (recommended), e.g., a declared
  multiple of the contact-solver penetration/length scale established by RES-84;
  (b) absolute 0.001 m; (c) historical 0.010 m.
- **Evidence:** S24 (credibility commensurate with use); project contact model
  (`solref 0.016 1.0`, `solimp ...`), `BILATERAL_SUPPORT_CONTINUITY_CONTRACT` (force
  zero is not liftoff; geometry decides).
- **Tradeoff:** a human-performance-derived gap is not supportable; a
  numerical-tolerance-derived threshold is executable and honest; 0.010 m is the
  obsolete unexecuted declaration.
- **Recommendation:** (a), with the numeric value fixed by RES-84 and recorded here.
- **Affected gates:** E6, E7, `PHYSICAL_TAKEOFF_DEFINED`, `GENUINE_FLIGHT_DWELL`,
  NC-02.

## OD-03 — Genuine-flight dwell

- **Variable:** `E7` `REQUIRED_DURATION`.
- **Options:** 0.005 s (one control interval; pure dechatter), 0.050 s (recommended),
  0.150 s (qualitative flight statement).
- **Evidence:** no literature absolute minimum exists for a simulator; the dwell is a
  dechatter/verification requirement. The primary performance floor, once set,
  implies a much longer ballistic flight (≈0.35 s at 0.15 m), so the dwell selection
  is not doing performance work.
- **Tradeoff:** 0.005 s is minimal but may admit numerically marginal flights;
  0.150 s risks conflating dwell with performance.
- **Recommendation:** 0.050 s.
- **Affected gates:** E7, `GENUINE_FLIGHT_DWELL`, NC-02, NC-08.

## OD-04 — Landing (L4) numeric bounds

- **Variables:** L4-T1 `COM_VX_AT_E10`, L4-T2 (RETIRED_REDUNDANT), L4-T3 `HY_AT_E10`,
  L4-T4/T5 root/trunk pitch, L4-T6/T7 rates, L4-T8 momentum capture and the
  `COM_X_MAX` window `[LANDING_FIRST_CONTACT, E11_onset]`, L4-T9
  `MAX_ABS_COM_VX_FIRST_CONTACT_TO_E10`, L4-T10 landing transient maxima and
  mid-window posture envelope, plus re-approval of `MAX_PENETRATION <= 0.010 m` and
  `LANDING_PEAK_FZ_BW <= 8.0`.
- **Options:** strict / medium / permissive sets; contract candidates are the medium
  set (§2.2 of `LANDING_BALANCE_RECOVERY_CONTRACT.md`).
- **Evidence:** S16 (trunk–pelvis–limb coupling), S17/S18 (trunk flexion is
  permissible/beneficial; the defect is the uncaptured lunge), S15 (landing force
  scale), P02 (project 8 BW / 0.010 m historical hard rules), P05 (R001 lunge
  evidence: `com_vx` 0.211→0.342 m/s post-touchdown; `Hy` 9.88; root pitch
  0.292 rad; trunk 25°).
- **Tradeoff:** posture bounds alone cannot exclude the R001 lunge; the behavioral
  `L4-T8` momentum-capture gate does the primary work. Over-tight posture bounds
  would criminalize normal trunk flexion.
- **Recommendation:** adopt the medium candidate set and treat `L4-T8` as the
  primary anti-lunge gate; re-approve 0.010 m / 8 BW explicitly.
- **Affected gates:** `LANDING_VALID`, `LANDING_BOUNDS`, NC-05, NC-09.

## OD-05 — Balance-capture (E11) numeric bounds

- **Variables:** `C_11` (sagittal COM speed), `HY_11`, `M_11` (support margin).
- **Options:** `C_11` = 0.20 m/s (recommended) or 0.30 m/s (historical numeric on the
  corrected scalar); `HY_11` = 2 or 5 kg·m²/s; `M_11` = 0.02 or 0.05 m.
- **Evidence:** S19/S20/S21 establish TTS/DPSI *structure* only; S23 capture-point
  logic; no transferable threshold exists. P05: R001 E11 `com_vx` ≈ 0.279 m/s with
  the historical guard only checking `com_vz` (MED-002).
- **Tradeoff:** permissive values risk certifying "vertically stopped" as captured;
  strict values may exceed near-term controller authority. If the rebuilt controller
  cannot satisfy the adopted bounds, that is a control finding, not a contract edit.
- **Recommendation:** `C_11 = 0.20 m/s`, `HY_11 = 2 kg·m²/s`, `M_11 = 0.02 m` as the
  initial task bounds, revisited with RES-84 support-hull and RES-87 envelope data.
- **Affected gates:** E11, `BALANCE_VALID`, NC-06.

## OD-06 — Takeoff transition whip bounds

- **Variables:** `R_WHIP` (root/pelvis pitch rate), `R_WHIP_TRUNK`.
- **Options:** 5.0 rad/s (recommended), 8.0 rad/s, or a model-derived bound from the
  RES-83/RES-85 actuator-rate declaration.
- **Evidence:** P05 (R001 pelvis pitch rate −9.68 rad/s at takeoff; non-human whip);
  no literature bound for pelvis pitch rate in a loaded CMJ exists; P04-style
  engineering reasoning.
- **Tradeoff:** depends on the rebuilt actuation model; a bound tighter than the
  model's attainable pre-takeoff motion is over-constraining.
- **Recommendation:** 5.0 rad/s initially; finalize with RES-85.
- **Affected gates:** `CG-03`, `TAKEOFF_TRANSITION_WHIP`.

## OD-07 — Countermovement depth `DEPTH_MIN`

- **Variable:** E3 depth threshold.
- **Options:** 0.100 m (historical), 0.150 m, 0.200 m, or a Plant-relative depth.
- **Evidence:** S12 (human countermovement depth ≈ 32.5 ± 7.1 cm), S08/S09
  (loaded CMJ pattern); P05 (R001 countermovement was shallow and hip-range-limited);
  CRIT-001/CRIT-002 (current Plant cannot realize human depth).
- **Tradeoff:** human depth values are not achievable until RES-83; setting the
  numeric now would either encode the broken Plant or pre-constrain the rebuild.
- **Recommendation:** freeze the *concept* now (done) and set the numeric after
  RES-83 reports the achievable anatomical countermovement depth; jointly record the
  decision as `MODEL_DEPENDENT_DEFERRED` → resolved.
- **Affected gates:** E3, `VALID_COUNTERMOVEMENT`, NC-01.

## OD-08 — Landing-phase vertical-rate thresholds

- **Variables:** `V_DESC` (E9 descending-COM condition), `V_ABS_TAIL` (E10 impact
  arrest).
- **Options:** `V_DESC` = 0.10 m/s (recommended), 0.20 m/s; `V_ABS_TAIL` = 0.05 m/s
  (recommended), 0.10 m/s.
- **Evidence:** P06/P05 (historical values existed: E9 −0.10, E10 0.05); S19/S20
  (stabilization bands are sustained, not instantaneous). These are dechatter/task
  conditions, not performance gates.
- **Tradeoff:** too loose lets chatter latch E9/E10 early; too tight delays latch
  into the capture window. Both remain subordinate to L4/L5 gates.
- **Recommendation:** retain 0.10 / 0.05.
- **Affected gates:** E9, E10.

## OD-09 — Force-threshold comparability value and sensitivity set

- **Variables:** `F_thr`; sensitivity set.
- **Options:** retain 10 N (recommended) with sensitivity across {1, 5, 10, 20 N,
  noise-based PkRes/5SD}; or adopt a noise-derived threshold only.
- **Evidence:** S04 (thresholds 20/10/5/1 N, 5SD, PkRes: reliable but statistically
  different, practically trivial), S06 (loaded CMJ thresholds 10 N / 50 N / 1 %SW /
  10 %SW / 5SDSW affect reliability and magnitude), S02 (processing sensitivity).
- **Tradeoff:** simulation has no electrical noise; a fixed declared threshold is a
  comparability convention; noise-derived thresholds are meaningful for hardware
  plates.
- **Recommendation:** retain 10 N as the declared comparability threshold AND report
  the sensitivity set. Force thresholds must never enter a **physical-state definition**
  (OD-11); the canonical reflight/chatter project gates remain frozen force-only
  de-chatter rules (RES-57: whole-Fz 10 N, `K ≥ 4` span, chatter ≤ 8) and are not
  physical-state definitions.
- **Affected gates:** `FORCE_THRESHOLD_TAKEOFF_TIME`, `FORCE_TAKEOFF_LEAD`,
  `FORCE_DEFINED_FLIGHT_DURATION` (all diagnostic-only).

## OD-10 — Landing asynchrony allowance and bilateral-establishment dwell

- **Variables:** `D_EST` (establishment latency from first contact), `D_BL` (bilateral
  landing sustain dwell), `LANDING_ASYMMETRY_S` bound.
- **Options:** `D_BL` = 0.020 / 0.050 (recommended) / 0.100 s; asymmetry bound = none
  (report-only) or a declared numeric.
- **Evidence:** no evidence supports microsecond simultaneity; bilateral landing is a
  task requirement, asynchrony is a tolerance question. S12 (force-share asymmetry
  reliability is moderate/poor — do not over-gate a noisy quantity).
- **Tradeoff:** too tight a dwell forces artificial symmetry; report-only asymmetry
  is honest; a loose dwell could hide a single-leg landing strategy.
- **Recommendation:** allow asynchrony; require establishment (`D_EST = 0.050 s`) and
  sustained bilateral loading (`D_BL = 0.050 s`) as separate variables; report
  asymmetry; gate only on establishment and the no-reflight/no-single-support
  conditions.
- **Affected gates:** `BILATERAL_LANDING`, `L4-G3`, NC-05.

## OD-11 — Force threshold participation in hard gates (adjudication acknowledgment)

- **Question:** may any force-threshold observable participate in `PHYSICAL_TAKEOFF`,
  `GENUINE_FLIGHT`, `APEX`, or `LANDING_FIRST_CONTACT`?
- **Recommendation (adjudicated in contract; owner acknowledgment requested):** **NO.**
  Exact contact geometry and clearance are available; force thresholds are
  comparability observables only (HIGH-010 closure). The owner must either accept
  this or explicitly authorize a reduced-observation variant.
- **Affected gates:** E6/E7/E8/E9, NC-02, NC-03.

## OD-12 — Standing-envelope tolerance class and robustness target (RES-87)

- **Variables:** posture/velocity/Hy/margin/CoP tolerance classes for
  `standing_envelope`; the perturbation family and the acceptance criterion.
- **Options:** (a) envelope derived from N perturbed holds with the requirement that
  each perturbed system remains in or returns to the envelope within a declared
  settling time; (b) a physically motivated joint-limit-adjacent standing set;
  (c) combined (recommended).
- **Evidence:** S22 (quiet-standing steadiness measures), S23 (dynamic stability
  condition), P05/HIGH-012 (the historical ULP one-trace envelope is invalid).
- **Tradeoff:** robustness studies cost compute; a purely analytic envelope may not
  capture controller/model interaction.
- **Recommendation:** (c): calibration from perturbed holds (RES-87) with a declared
  perturbation family and settling criterion, cross-checked against the
  physical standing manifold; record the numeric tolerances as RES-87 output and
  bind them into this contract by version.
- **Affected gates:** E12, `RECOVERY_VALID`, NC-07.

---

## OD-13 — Initial quiescence and post-recovery observation bounds

- **Variables:** E1 `abs(com_vx)`; E1 joint-rate/CoP quiescence (candidate);
  `POST_E12_OBSERVATION_MARGIN`.
- **Options:** `|com_vx| ≤ 0.05 m/s` and post-E12 margin 0.250 s (recommended);
  `|com_vx| ≤ 0.10 m/s` and margin 0.500 s; no horizontal quiescence (rejected by
  the adversarial review — opens the moving-start hybrid ADV-7).
- **Evidence:** P05 (historical E1 had only a `com_vz` bound), P02 (rotation/HAX
  framework).
- **Tradeoff:** quiescence bounds prevent a standing-broad-jump hybrid; too tight a
  bound may reject a legitimate slow-drift start. The post-E12 margin prevents
  certifying recovery with a horizon that ends at confirmation.
- **Recommendation:** accept `|com_vx| ≤ 0.05 m/s` and a 0.250 s post-E12 observation
  margin; joint-rate/CoP quiescence numerics deferred to RES-84 with the structural
  requirement frozen.
- **Affected gates:** E1, NC-11, `HORIZON_SUFFICIENT`, `RECOVERY_VALID`, NC-12.

---

## Decision status summary

| OD | Item | Status | Class |
|---|---|---|---|
| OD-01 | Primary performance floor | OPEN | OWNER_TASK_DECISION |
| OD-02 | Geometric clearance tolerance | OPEN | MODEL_DEPENDENT_DEFERRED (RES-84) |
| OD-03 | Genuine-flight dwell | OPEN | OWNER_TASK_DECISION |
| OD-04 | Landing numeric bounds | OPEN | OWNER_TASK_DECISION + project re-approval |
| OD-05 | Balance-capture numeric bounds | OPEN | OWNER_TASK_DECISION |
| OD-06 | Takeoff whip bounds | OPEN | MODEL_DEPENDENT_DEFERRED (RES-83/85) |
| OD-07 | Countermovement depth | OPEN | MODEL_DEPENDENT_DEFERRED (RES-83) |
| OD-08 | Landing vertical-rate thresholds | OPEN | OWNER_TASK_DECISION |
| OD-09 | Force-threshold comparability value | OPEN | OWNER_TASK_DECISION |
| OD-10 | Landing asynchrony/dwell | OPEN | OWNER_TASK_DECISION |
| OD-11 | Force threshold in hard gates | ACKNOWLEDGMENT REQUESTED | OWNER_TASK_DECISION |
| OD-12 | Standing envelope robustness target | OPEN | MODEL_DEPENDENT_DEFERRED (RES-87) |
| OD-13 | Initial quiescence and post-recovery observation | OPEN | OWNER_TASK_DECISION |

Until an affected decision is closed, the corresponding component of
`TASK_SUCCESS_COMPONENTS` is reported `false`/unresolved and can never be inferred
from event-chain completion.
