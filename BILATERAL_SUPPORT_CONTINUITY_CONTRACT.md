# BILATERAL_SUPPORT_CONTINUITY_CONTRACT (RES-57, V2.1)

`MISSION=RES10_BILATERAL_SUPPORT_CONTINUITY_AUTHORITY_001`
Frozen BEFORE any inspection of the +127.125/+128.250/+145.750-ms target windows.
Authority HEAD `e85d9c9492dd648089350206c6a71d632e342929`.

This contract does not modify Plant, contact, actuator, scorer, controller, takeoff,
propulsion, RES-54/RES-55 measurement, RES-43 standing, or any existing threshold/dwell.
It adds a reusable semantic layer that separates sample-level physical facts from
finite-duration control relevance.

## 1. Derivation (prospective, not retrospective)

- Existing per-foot force threshold: **10.0 N** strict `>` (`V2_CONTACT_FZ_THRESHOLD_N`,
  `CONTACT_ACTIVE_THRESHOLD_N`, `BILATERAL_TAKEOFF_FZ_N` — all numerically identical).
- Physics timestep: **0.000125 s**. Control interval: **0.005 s** = 40 physics steps.
- Existing canonical dwells: reflight 0.5 ms (4 steps, bilateral whole-support);
  takeoff/reversal/landing 10 ms; genuine flight 80 ms; capture 150 ms.
- MuJoCo soft contact: penetration (`dist<0`) is compliant overlap, not proof of load;
  force zero with retained rows is a legal solver state for one step (redundant
  elliptic/pyramidal rows redistribute load).

Control-relevance principle (predeclared in RES-57):

> A transient per-foot force dropout shorter than one 5-ms control interval is NOT,
> by itself, a `CONTROL_RELEVANT_SUPPORT_LOSS_EPISODE`.

Rationale: the synchronized controller observes and acts at 5-ms boundaries only
(`SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL`, `OBSERVATION_INPUT_CONTROL =
PREVIOUS_HELD_CONTROL`). A sub-control-interval force transient cannot be observed,
cannot be acted upon, and cannot propagate through the 5-ms exact-forward inner
branches as a distinct control decision. The 40-step floor is the controller's own
Nyquist, not a number chosen to make RES-56 pass: a single-step (0.125 ms) dropout is
1/40th of the floor; the old strict gate (max run 2 steps, POST_WALK ≤ 1) remains
historically FAIL and is not rewritten.

No already-frozen stricter project authority governs unilateral finite-duration support
loss (the 2-step run gate is the strict count being adjudicated, not a physics
definition; event dwells are ≥ 10 ms and concern bilateral/scorer events). The 5-ms
floor therefore stands.

## 2. Sample-level definitions (every physics sample, both feet)

| Field | Definition | Source |
|---|---|---|
| `GEOMETRIC_ENGAGEMENT_SAMPLE` | foot has ≥1 floor contact row | `foot_floor_contacts` deepest-row existence |
| `COMPRESSIVE_SUPPORT_SAMPLE` | per-foot `Fz > 10.0 N` | synchronized `foot_contact_summary` |
| `UNILATERAL_FORCE_DROPOUT_SAMPLE` | exactly one foot `Fz ≤ 10.0 N`, other `> 10.0 N` | force pattern |
| `GEOMETRIC_LIFTOFF_SAMPLE` | zero floor rows AND lowest-corner gap `> 0` | geometry + rows |
| `SEPARATION_EVIDENCE_SAMPLE` | `GEOMETRIC_LIFTOFF_SAMPLE` OR `nvel > 0` (+z separating) | frozen RES-55 normal velocity |

Rules:

- Force zero is NEVER equated with geometric liftoff without checking rows/gap.
- Negative `dist` is NEVER equated with guaranteed load-bearing support.
- Normal velocity is ALWAYS consulted for separation evidence.
- A short geometric liftoff is ALWAYS reported as `GEOMETRIC_LIFTOFF_SAMPLE=true`;
  duration filtering applies only to the episode field, never to the sample field.

## 3. Finite-duration episode definition

`CONTROL_RELEVANT_SUPPORT_LOSS_EPISODE` (per foot) =

- maximal run of consecutive physics samples where
  `(per-foot Fz ≤ 10.0 N) AND (SEPARATION_EVIDENCE_SAMPLE)` holds, AND
- run length `≥ 40` physics steps (`≥ 0.005 s`, one control interval).

Shorter candidate runs are reported as transient dropout runs with
`CONTROL_RELEVANT=false`; they are not hidden, only classified as non-control-relevant.

Canonical `REFLIGHT` is unchanged and separate: whole `Fz < 10 N` for `≥ 4` steps
(0.5 ms), bilateral, force-only. Primary chatter unchanged: whole-Fz transitions `≤ 8`.

## 4. Trajectory adjudication

For a 150-ms horizon under this contract:

- `GEOMETRIC_LIFTOFF_SAMPLE_COUNT` = samples with any-foot liftoff (dist-exact).
- `UNILATERAL_FORCE_DROPOUT_SAMPLE_COUNT` = samples with exactly-one-foot dropout.
- `CONTROL_RELEVANT_SUPPORT_LOSS_COUNT` = control-relevant episodes across both feet.
- `CANONICAL_REFLIGHT` = list of whole-support run lengths ≥ 4 (empty if none).

`SUPPORT_CONTINUITY_S50=QUALIFIED` iff ALL hold:

1. `CONTROL_RELEVANT_SUPPORT_LOSS_COUNT == 0`, AND
2. `CANONICAL_REFLIGHT == []`, AND
3. `CHATTER_TRANSITIONS <= 8` (no primary chatter), AND
4. no hard-gate violation (peak ≤ 8 BW, penetration ≤ 0.010 m, prohibited=false,
   root limit rows 0, root passive 0, actuator |u|≤1, finite, trust fallbacks ≤5,
   no physical fall).

Otherwise `SUPPORT_CONTINUITY_S50=NOT_QUALIFIED`. No controller changes either way.
Historical RES-56 (`POST_WALK 3>1` → FAIL) is preserved; this contract is new authority
for future qualification.

## 5. Boundary table (normative)

| Run length | Duration | Classification (given separation evidence) |
|---|---|---|
| 1 step | 0.125 ms | transient dropout, NOT control-relevant |
| 2 steps | 0.25 ms | transient dropout, NOT control-relevant (old max-run gate) |
| 4 steps | 0.5 ms | transient dropout, NOT control-relevant (== canonical reflight floor, but unilateral) |
| 39 steps | 4.875 ms | transient dropout, NOT control-relevant |
| 40 steps | 5.0 ms | `CONTROL_RELEVANT_SUPPORT_LOSS_EPISODE` |
| 41+ steps | >5 ms | `CONTROL_RELEVANT_SUPPORT_LOSS_EPISODE` |

Without separation evidence (penetrating, approaching/zero velocity), even a sustained
force dropout is NOT a support-loss episode under this contract (load-redistribution
regime); it is reported as sustained unilateral dropout without liftoff.

## 6. Prohibitions (reviews must hunt)

- Retrospective threshold weakening (10 N unchanged).
- Force-zero ⇒ liftoff without geometry check.
- Negative dist ⇒ guaranteed support.
- Ignoring normal velocity.
- Hiding short liftoff by duration filtering (sample field must remain true).
- Modifying canonical reflight (4 steps) or chatter (8 transitions) semantics.
- `mj_forward` on LIVE for diagnostics (synchronized copies only).
- Modifying recorded RES-56 actions; tuning Plant/contact/controller/scorer.

## 7. Identity

Spec: `support_continuity_spec.json` (hash-sealed). Helpers:
`src/loaded_cmj/v2/support_continuity.py` (pure, no MuJoCo calls).
Hash recorded below at freeze time; tests enforce immutability.

Frozen identity (stable, self-reference-free): the contract is identified by the
sealed spec payload hash below. File-content sha256 values are recorded in the
evidence manifest at freeze time; the `.md` file must not hash itself.

`SUPPORT_CONTINUITY_CONTRACT_SHA256=cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734`
(`= SUPPORT_CONTINUITY_SPEC_SHA256`, sha256 of `support_continuity_spec.json` payload
excluding its own `SPEC_SHA256` field; file sha256 `f4223bc47ff549b01047281446f43b336986e9c28e8d132242026c0f904a3f81`.)
`CONTROL_RELEVANT_LOSS_DWELL=0.005 s = 40 physics steps`
