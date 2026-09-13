# NEGATIVE-CONTROL CONTRACT — Required Successor Falsification Fixtures

MISSION: `RES82_SUCCESSOR_LOADED_CMJ_SCIENTIFIC_TASK_CONTRACT_001`
STATUS: **FROZEN** (design); executable tests are later obligations (RES-89 / RES-91).
DEFECTS ADDRESSED: `CRIT-003`, `HIGH-001`, `HIGH-010`, `HIGH-011`, `HIGH-012`,
`MED-002`, `MED-020`, and the conceptual defect `EVENT_CHAIN_COMPLETION != TASK_SUCCESS`.

Each negative control defines a trajectory class that must **FAIL** the successor
contract. A contract that passes any of these is not accepted. "Expected" fields are
the mandatory verdicts of the corresponding future executable tests.

---

## NC-01 — `TRIVIAL_HOP`

- **Construction**: a trajectory that accumulates event-like ordering (supported
  start, small countermovement, takeoff, contact, landing, standing) but whose
  `COM_RISE_TAKEOFF_TO_APEX` is below the approved performance floor (e.g., the R001
  scale, ~0.075 m, or any sub-floor value).
- **Expected**: `TASK_PERFORMANCE_VALID = false`; `TASK_SUCCESS = false`.
- **Prevents**: the CRIT-003 false pass where a 3.6 cm hop satisfied every executable
  gate.
- **Assertion anchor**: primary performance gate (OD-01).

## NC-02 — `FORCE_DROPOUT_WITHOUT_PHYSICAL_FLIGHT`

- **Construction**: bilateral `Fz` falls below `F_thr` transiently or for a short run
  while geometric floor contact/recontact persists (penetrating or load-redistributed
  state with retained contact rows).
- **Expected**: `PHYSICAL_TAKEOFF = FAIL`; `GENUINE_PHYSICAL_FLIGHT = FAIL`;
  force-threshold observables may show a dropout (that is the point of the control).
- **Prevents**: HIGH-010 — declaring flight from force alone with no executable
  geometric clearance.
- **Assertion anchor**: E6/E7 predicates use contact rows and `foot_clearance`.

## NC-03 — `CONTACT_PHASE_APEX_ZERO_CROSSING`

- **Construction**: `com_vz` crosses from positive to non-positive while at least one
  foot still has floor contact (e.g., a slow post-landing settle or an ascending-phase
  artifact).
- **Expected**: `APEX = FAIL`.
- **Prevents**: HIGH-011 — an apex latch outside physical flight, including the
  historical unguarded fallback path.
- **Assertion anchor**: E8 requires both bracketing samples to be in physical flight.

## NC-04 — `FALL_AT_INITIAL_SUPPORT`

- **Construction**: the plant has adequate bilateral plantar `Fz` but a fall-shell
  collision is contacting/loading the floor at the start (athlete "standing" on a fall
  shell).
- **Expected**: `E1 = FAIL` (`supported_start` not confirmed).
- **Prevents**: MED-020 — a supported-start guard that ignores the fall flag.
- **Assertion anchor**: E1 requires `NOT fall_contact` and `NOT prohibited_contact`.

## NC-05 — `PITCHING_LANDING`

- **Construction**: vertical COM velocity is arrested at landing (`abs(com_vz)` small)
  but horizontal momentum / `Hy` / pitch exceed the L4 bounds, or the forward momentum
  continues to grow after E10 (R001-like forward lunge), or a landing-window transient
  whip decays before the E10 point-in-time sample (L4-T10 fixture).
- **Expected**: `E10 = FAIL` (`LANDING_VALID = false`); `TASK_SUCCESS = false`.
- **Prevents**: HIGH-001 — a vertical-only impact-absorption predicate.
- **Assertion anchor**: `L4-T1..T8`, especially the behavioral `L4-T8`.

## NC-06 — `FALSE_BALANCE`

- **Construction**: `abs(com_vz)` small but `abs(com_vx)` or `abs(Hy)` remains outside
  the E11 bounds (motion is still translating/rotating laterally in the sagittal
  plane).
- **Expected**: `E11 = FAIL` (`BALANCE_VALID = false`).
- **Prevents**: MED-002 — calling a single vertical component "COM speed" and
  declaring capture.
- **Assertion anchor**: E11 uses `com_speed_sagittal` and `Hy`.

## NC-07 — `BRITTLE_STANDING`

- **Construction**: the exact nominal standing trace lies inside the envelope, but
  small approved perturbations of the standing initial state cannot remain in / return
  to the envelope (one-trajectory overfit, as in R001's ULP envelope).
- **Expected**: `E12_QUALIFICATION = FAIL`.
- **Prevents**: HIGH-012 — a standing envelope that only certifies the trace it was
  derived from.
- **Assertion anchor**: `standing_envelope` calibration robustness requirement
  (RES-87); the executable control runs perturbed holds.

## NC-08 — `POST_LANDING_REFLIGHT`

- **Construction**: landing occurs, then support is lost again beyond the approved
  chatter/reflight tolerance during `[E9, E12_confirmation or horizon]`.
- **Expected**: `TASK_SUCCESS = false`; L4/L5 support gates fail.
- **Prevents**: hiding a lost landing behind a later nominal standing frame; R001's
  full-episode support `NOT_QUALIFIED` with chatter 188 and reflight runs
  `[4, 8, 1815]`.
- **Assertion anchor**: `NO_POST_LANDING_REFLIGHT`; `FULL_EPISODE` support scope.

## NC-09 — `EXCESSIVE_PENETRATION`

- **Construction**: feet/body penetrate the floor beyond the declared penetration
  tolerance at any time (contact solver pushed out of its admissible regime).
- **Expected**: `TASK_SUCCESS = false`; hard gate `PENETRATION_MAX` fails.
- **Prevents**: scoring a state that is not physically admissible as a successful
  landing.
- **Assertion anchor**: `MAX_PENETRATION <= 0.010 m` (OD-04 re-approval).

## NC-10 — `EVENT_CHAIN_WITHOUT_TASK_SUCCESS`

- **Construction**: all twelve event labels latched (or event-like ordering achieved)
  while the performance gate fails (NC-01) and/or the landing/capture/recovery layers
  fail.
- **Expected**: `EVENT_CHAIN_VALID = true` is permitted; `TASK_SUCCESS = false`
  mandatory.
- **Prevents**: the historical conceptual defect that 12/12 event latching equals
  success — exactly the R001 disposition.
- **Assertion anchor**: `SUCCESSOR_TASK_CONTRACT.json` result rule; the top-level
  result must be the conjunction of independent layers.

## NC-11 — `MOVING_START_CMJ_HYBRID`

- **Construction**: E1 begins with non-quiescent horizontal motion (e.g., `com_vx` above
  the E1 quiescence bound), or horizontal momentum is carried through the
  countermovement/flight and then scrubbed during landing inside the force/penetration
  envelope (a standing-broad-jump hybrid; ADV-7).
- **Expected**: `E1 = FAIL` when the start is non-quiescent; the impact-interval
  horizontal momentum gate (`L4-T9`) fails when momentum is smuggled through the
  impact interval.
- **Prevents**: a CMJ contract satisfied by a horizontal-momentum hybrid.
- **Assertion anchor**: E1 `abs(com_vx) < 0.05 m/s` (OD-13); `MAX_ABS_COM_VX_PRE_FIRST_CONTACT_TO_E10` bound (OD-04).

## NC-12 — `FREEZE_OR_STATE_OVERRIDE_RECOVERY`

- **Construction**: the state is artificially frozen or overridden (velocity clamp,
  position injection, integration bypass) so that a physically unstable or divergent
  recovery appears to hold the standing envelope; or the episode terminates
  immediately after E12 confirmation before post-recovery instability can appear.
- **Expected**: `TASK_SUCCESS = false`; `NO_ARTIFICIAL_SUPPORT` fails for state
  overrides; `HORIZON_SUFFICIENT` fails without the declared post-E12 observation
  margin; `RECOVERY_VALID = false`.
- **Prevents**: "recovery by freezing" and "recovery certified by a short horizon".
- **Assertion anchor**: `NO_ARTIFICIAL_SUPPORT` (state/integration override scope);
  `HORIZON_SUFFICIENT` with post-E12 observation margin (OD-13); `RECOVERY_VALID`.

---

## Adversarial false-pass mandate

The adversarial review pass (RES-82 review gate; later RES-89) must attempt to
construct a physically bad trajectory that satisfies the contract as written. If it
succeeds, the contract does **not** pass and the gap must be closed before sealing.

## Mapping to future executable tests

| Control | Expected verdict | Test owner |
|---|---|---|
| NC-01 | TASK_SUCCESS=false | RES-89/RES-91 |
| NC-02 | PHYSICAL_TAKEOFF/FLIGHT=false | RES-89/RES-91 |
| NC-03 | APEX=false | RES-89/RES-91 |
| NC-04 | E1=false | RES-89/RES-91 |
| NC-05 | E10/LANDING_VALID=false | RES-89/RES-91 |
| NC-06 | E11/BALANCE_VALID=false | RES-89/RES-91 |
| NC-07 | E12_QUALIFICATION=false | RES-87/RES-89 |
| NC-08 | TASK_SUCCESS=false | RES-89/RES-91 |
| NC-09 | TASK_SUCCESS=false | RES-89/RES-91 |
| NC-10 | EVENT_CHAIN_VALID=true, TASK_SUCCESS=false | RES-89/RES-91 |
| NC-11 | E1=false or L4-T9=false | RES-89/RES-91 |
| NC-12 | TASK_SUCCESS=false (override/horizon) | RES-88/RES-89 |
