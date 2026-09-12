# BUG_HUNT (RES-80, adversarial pass)

Independent hunt for defects not already in the provisional list. Every item below is confirmed by
source/measurement evidence; no fix was attempted.

## BUG-01 - `controller.py` is dead in production but is what most "V2.1" tests exercise
Files: `tests/test_v2_1_res10_honest_full_jump.py`, others (see TEST_SUITE_FORENSIC_REPORT §5).
No production module imports `loaded_cmj.v2.controller`; the canonical runtime re-implements its
HOLD/SUPPORTED/FLIGHT prefix in `res72_integration.py`. Nine V2.1-named suites test the legacy module's
globals. Register: MED-019 (with HIGH-015/017).

## BUG-02 - The 20 kg external load has no collision geometry and no fall shell
`v2_plant.xml:80-83`: `load_shell` has class `shell` (`contype=0 conaffinity=0`). In a fall the 1.5 m bar
passes through the floor/body without any contact or fall flag; `fall_contact` only scans the six fall
shells (measurement.py:163-183). Register: MED-007.

## BUG-03 - `max_penetration` ignores fall-shell contacts
`plant.py:196-246` accumulates penetration only for floor-foot pairs; fall-shell floor penetration is
never reported. Register: MED-006/007.

## BUG-04 - `E1` supported-start does not check `fall_contact`
`events.py:126-141` checks the structurally-dead `prohibited` flag but not `fall_contact`; a fall shell
on the floor at t~0 does not block E1. Register: MED-020.

## BUG-05 - Apex can latch outside flight
`events.py:430-436` fallback: after the two-sided flight check fails, the function still returns a
crossing time if `vz_prev>0 and vz_cur<=0`. A contact-phase crossing after E7 would latch E8.
Register: HIGH-011.

## BUG-06 - `assert` statements as production control flow
`res72_integration.py:213-214` asserts terminal policy/sample bindings inside `act()`; with `python -O`
these vanish and the code would dereference `None`. `canonical_runtime.py:223,225` likewise assert the
time-identity and held-action contract. Under optimization, fail-closed checks disappear.
(New; register-adjacent to MED-017.)

## BUG-07 - RR dwell loop can arm on stale state across truncation
`canonical_runtime.py:352-378`: `rr_since` is a physics-rate streak evaluated inside the sub-step loop;
when a truncation breaks the interval (`break` at line 447), the pending streak persists into the next
control interval. If the condition was marginal at truncation, `rr_conf_t` can be recorded at a sample
whose control interval was cut short (the 28-substep E10/RR truncations). The effect is bounded to one
control interval but is not documented.
(New; register-adjacent to MED-017.)

## BUG-08 - E10 streak can count across regime switch incorrectly
`canonical_runtime.py:336-350`: `e10_streak` is reset when `prelanding.phase != 7`, but it is evaluated
after `prelanding.act` has already advanced the phase for the current control step. Boundary behavior is
one sample ambiguous. (New, low.)

## BUG-09 - Hardcoded `WALL_S` and exception-swallowing produce non-reproducible evidence
`canonical_runtime.py:388-389,586`. Register: LOW-006/MED-017.

## BUG-10 - `times` and event sample `time_s` disagree under truncation
`canonical_runtime.py:321-324` sets `ev1["time_s"]=sim_t`, while `sync1.event_sample()` returned the
shadow time (identical here). The re-write is harmless now but means two time authorities are used.
(New, low.)

## BUG-11 - Unvalidated first action after every truncation
After E10/RR/handoff truncations, the next control interval starts with a shortened execution; the
balance/recovery inner validation ran for 40 substeps but the plant executes 28/33/40. The trust-region
validation therefore models an interval the plant does not execute at those three boundaries.
(New; part of HIGH-014/CONTROL_REVIEW.)

## BUG-12 - `full_closure.rr_predicate_ok` parameter is named `root_ry` but receives root pitch rate
`full_closure.py:62-69` compares the argument to `ROOT_PITCH_RATE_ABS_MAX`; the runtime passes
`sync1.qvel[2]`. The hashed mirror is dead but its naming could mislead a successor into passing angle.
(New, low; register-adjacent to MED-018.)

## BUG-13 - `_is_true_standing_neighborhood` qpos fallback comments are wrong (no behavioral impact)
`events.py:280-283` comment maps qpos 3/4/7/5/8/6/9 correctly, but the preceding comment block says
"last 7 are actuated"; harmless. (New, low/documentation.)

## BUG-14 - `support_continuity.unilateral_dropout_samples` returns a boolean array under a scalar API
`support_continuity.py:97-105`: `bool(drop_l is not None) and (drop_l ^ drop_r)` returns the numpy array
(never used). Register: LOW-005.

## BUG-15 - Action schedule append in the RES-79 replay tool silently pads/truncates
`tools/res79_smoke_replay.py` builds per-physics action arrays by repeating control actions; the
audit's independent rebuild matched length exactly, but the tool has no fail-closed check that the
schedule length equals `N_PHYS` (it gates on counts elsewhere). Low risk, noted for successor tooling.

## Adjacent-latent items explicitly REJECTED as active bugs

- Contact-frame transform (correct; comments wrong).
- COM velocity authority (correct).
- Root damping/catch (absent).
- `prohibited` geometry ordering sign (handled in both orders).

**Verdict:** the bug hunt found four new behavioral latents (BUG-06/07/11/12) plus collision/fall gaps
already captured. None changed the R001 outcome; all are relevant to a successor qualification.
