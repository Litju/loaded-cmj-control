# CONTROL_REVIEW (RES-80, independent pass)

Scope: `res72_integration.py`, `balance_capture.py`, `stable_recovery.py`, `terminal_capture.py`,
`full_closure.py`, `canonical_runtime.py`, legacy `controller.py`, and `tools/res52/soft_contact.py`
as used by production. Read-only. Numeric evidence from the sealed R001 replay/action schedule.

## 1. Production phase/state transition table

| Transition | Guard (file:line) | Dwell/hysteresis | Action law | Max observed/derived step |
|---|---|---|---|---|
| HOLD -> SUPPORTED | total_Fz>0.30W and weak>10, t-t0>0.10 (res72:152-155) | 0.10 s phase-age | zero-pose PD KP1000/KD20 (188-190) | n/a |
| FLEX -> EXTEND | `t_in<0.30` time switch (res72:59-60,193) | none | torque program switch | **0.568 at t=0.400** |
| EXTEND -> zeros | `t_in<0.60` time switch | none | zero torque | small |
| SUPPORTED -> FLIGHT | `maxf < 10`, single control sample (res72:156-159) | **none**; only t-t0>0.30 | zero-pose PD KP80/KD12 (201-203) | **0.944 at t=0.640** |
| FLIGHT -> LANDING_PREP | `cvz < -0.005`, single sample (160-164) | none | PD to PREP_TARGET | ~0.19 at 0.7700->0.7750 |
| LANDING_PREP -> IMPACT | per-foot `maxf>=20` control sample (166-169) | none | PD to IMPACT_TARGET KP114/KD20 | controlled |
| IMPACT -> TERMINAL | `t - td_physics >= 0.050` at control boundary (170-177) | 50 ms TD-relative | RES-58 outer + RES-52 inner | ~0.17 |
| PRELANDING -> BALANCE | |vz|<0.05, bilateral Fz>10, no fall, no prohib; 160 physics samples (canonical:336-346) | 20 ms streak, truncate | carried u_prev | ~0.03 at 0.9935 |
| BALANCE -> RECOVERY | RR thresholds + margin/Fz/pen; 0.10 s dwell (canonical:352-361) | 0.10 s | RISE law; **held reset to zeros** | **0.545 at t=1.337** |
| RISE -> SETTLE | `s>=1` (stable_recovery:575-578) | none | gains x8, QDOT 0.2->0.8, Q rows removed, rho cap 0.25->0.008 (404-408,431-434,532) | small-signal |
| SETTLE -> HANDOFF | rates/forces + scorer `in_envelope`; 0.05 s sustain (stable_recovery:580-586,612-634) | 0.05 s | RES-43 PD Kp400/Kd10 (47-49,593-597) | near-zero signals |

## 2. Hard steps, resets, jumps

1. **SUPPORTED -> FLIGHT at t=0.640, |Δu|=0.944** (hip ~236 Nm, knee ~270 Nm per 5 ms). The switch
   precedes true bilateral support loss (t=0.648125) by 8.125 ms and precedes sealed E6 by the same
   amount. The flight law commands the leg toward zero pose while q_hip=-0.32 / q_knee=+0.54, i.e. it
   acts as a violent extension/braking command. Pelvis pitch rate reaches -9.68 rad/s at t=0.649875.
2. **FLEX -> EXTEND at t=0.400, |Δu|=0.568** (hip +95 Nm, knee +170 Nm step) on a time schedule,
   independent of state.
3. **BALANCE -> RECOVERY at t=1.337, |Δu|=0.545** with the held action reset to zeros
   (canonical_runtime.py:253; stable_recovery.py:127).
4. **RISE -> SETTLE** controller stiffness/trust jumps (8x servo, 4x QDOT weight, trust cap reduction).
5. Truncated control intervals (28/28/33-substep events) mean the first action after a dwell completion
   is applied over a shortened interval, outside the nominal 5 ms validation window.

## 3. Adjudications

- **(a) Zero-pose flight PD is not a credible takeoff command.** At the switch the command is dominated
  by the damping term against the on-going rotation; it contains no COM/ballistic authority. Observed
  consequence: backward pitch whip and the low jump.
- **(b) The SUPPORTED->FLIGHT guard has no dwell/hysteresis/direction.** A single noisy sample can
  trigger it; the sealed RES-57 control-relevance dwell (0.005 s) is not reused. Confirmed defect
  (CONTROL/SCORER mismatch).
- **(c) Action-history reset is material.** 0.545 step at the switch; it also violates the
  PREVIOUS_APPLIED_ACTION continuity nominal used by the soft-contact inner layer.
- **(d) Terminal capture is vertical-only.** No horizontal/CAM/CoP authority; the takeoff pitch and
  impact Hy (3.33 -> 9.88 kg m²/s) are structurally outside its reach.
- **(e) Balance clamps are one-sided** (`FX in [-120,0]`, `HDOT in [-60,0]`), so forward CoM motion
  cannot be commanded; the post-touchdown forward acceleration of the CoM (0.181 -> 0.342 m/s) is
  uncorrectable in principle by this layer.
- **(f) Handoff gating uses a scorer predicate.** SETTLE->HANDOFF requires
  `in_envelope = watcher._is_true_standing_neighborhood(...)`, contradicting the "scorer observational
  only" claim and coupling control to unhashed `constants.py`.
- **(g) CoP projection is not exactly feasible after final clamps.** The final Fx re-clip can leave the
  commanded (Fx, Hdot) off the feasible CoP line; bounds are static and unrelated to actual foot pose.
  Bounded, latent.

## 4. Soft-contact inner layer (tools/res52/soft_contact.py)

- **Active-set crossings are counted but not handled** in `build_G` (182-192): a two-sided secant is
  still formed across a contact-mode discontinuity. Later modules (balance/recovery) added one-sided
  fallbacks — evidence the gap is real.
- **FZ_MIN rows disappear on refinement** (proved with an instrumented synthetic solve): pass 0 has the
  3 FZ_MIN rows, passes 1-2 do not (rows rebuilt as `base_rows + dist rows`, 262-263). Validation does
  not test the min-force margin.
- Trust region/fallback semantics are otherwise deterministic; on fallback the returned diagnostics
  (`Y_VAL`, `du`) still describe the rejected trial action while `u` is the held action (telemetry
  mismatch).

## 5. Ranked control defects

| Rank | ID | Finding | Severity |
|---|---|---|---|
| 1 | CRIT-007 | Premature single-sample SUPPORTED->FLIGHT + zero-pose flight PD; 0.944 step; -9.68 rad/s pitch whip | CRITICAL |
| 2 | HIGH-002 | Terminal capture vertical-only; no horizontal/CAM/CoP authority | HIGH |
| 3 | HIGH-003 | Balance braking-only authority cannot stop a forward lunge | HIGH |
| 4 | HIGH-005 | BALANCE->RECOVERY action-history reset (0.545) | HIGH |
| 5 | HIGH-014 | FZ_MIN row loss + unhandled active-set crossings | HIGH |
| 6 | HIGH-006 | No activation dynamics / action slew with 200 Hz direct torque | HIGH |
| 7 | HIGH-001/004 | E10 vertical-only predicate; RR has no absolute posture gate | HIGH |
| 8 | MED-008 | Time-programmed FLEX->EXTEND step | MEDIUM |
| 9 | MED-009 | CoP post-projection clamp residual; static CoP bounds | MEDIUM |
| 10 | MED-017 | Swallowed diagnostics / fallback telemetry mismatch | MEDIUM |

**Verdict:** the controller is an internally consistent hybrid program that reproduces bit-exactly, but
its takeoff phasing, authority sets and nonsmooth program switches are not a credible model of human
loaded-CMJ control, and several guarantees (CoP feasibility, min-force rows, action continuity) are
weaker than their documentation.
