# CLAIM_REVIEW (RES-80)

Question: what statements about V2.1-R001 remain scientifically defensible after this audit?

## 1. R001 disposition (frozen, not rewritten)

```text
COMPUTATIONAL_REPRODUCIBILITY=PASS
DECLARED_EVENT_CHAIN=PASS
DECLARED_NUMERICAL_GATES=PASS
OWNER_PHYSICAL_VISUAL_CREDIBILITY=FAIL
SHIP_STATUS=BLOCKED
```

This audit confirms each line and adds no reinterpretation.

## 2. Defensible claims

- The frozen V2.1-R001 episode reproduces deterministically in the pinned environment
  (Python 3.13.13, MuJoCo 3.8.0, NumPy 2.5.1, SciPy 1.18.1, dt=0.000125 s, control dt=0.005 s):
  TRACE `4d047879...`, 3,048 control intervals, 121,889 physics samples, T_END 15.236125 s.
- The sealed action schedule, when replayed on the frozen Plant, reproduces the sealed trajectory
  (RES-79 replay gates, independently re-read in this audit).
- The declared 12-event chain latches in order with the sealed occurrence/confirmation times.
- The declared numerical gates (root damping zero, no root limits, finite states, no fall, no
  accidental fall-shell contact, post-landing support adjudication within its slice) hold.
- The render is non-invasive: RES-79's replay identity gates show the renderer consumed the exact
  accepted arrays; physical defects are not rendering artifacts (P45 confirmed).
- The candidate's own event/checkpoint/trace hashes are internally consistent.

## 3. Claims that are NOT supported

- **"Successful loaded countermovement jump."** The accepted motion is a reverse-knee hop:
  knee hinge direction is anatomically inverted (CRIT-001), hip range excludes human flexion (CRIT-002),
  net jump height is 0.0363 m above standing (0.0768 m from takeoff), max foot clearance 0.0496 m,
  takeoff at 31-degree knee flexion with a flat foot, and a -9.68 rad/s pelvis pitch whip at takeoff.
- **"Human-like / credible loaded-CMJ mechanics."** No MTP/arch, no heel rise, no activation dynamics,
  and non-anatomical joint kinematics.
- **"Impact absorption / balance capture / stable recovery"** in the strong sense: E10 is vertical-only,
  E11 uses vertical speed, RES-58 has no horizontal/angular authority, RR has no posture gate, and E12
  is certified on a one-trajectory micron envelope.
- **"Genuine flight"** as declared: the geometric gap constant is unused.
- **"Frozen candidate V2.1-R001."** Identity is a five-file fingerprint over a larger executing system
  and unverified external authorities; no runtime hash enforcement.
- **"Qualified by the test suite."** The suite cannot collect as a whole, contains 11 placeholders,
  7 neutralized assertions, and does not execute the canonical composition.
- **Deployable/portable**: clean install fails (tools excluded, scipy undeclared); absolute evidence
  paths required.
- **Population generality, human-technique optimality, experimental validation**: never claimed by
  admissible evidence in this repository and explicitly not demonstrated.

## 4. Narrow valid claim

The strongest defensible statement is:

> V2.1-R001 is a bit-exactly reproducible, physics-simulated, event-chain-complete loaded-jump
> trajectory whose event and numerical gates pass under the frozen V2.1 definitions, but whose
> kinematics and performance are non-anatomical and trivial relative to a loaded countermovement jump,
> so it does not constitute successful task performance and must not be shipped as one.

## 5. Recommended claim-status changes (successor work, not this mission)

1. Rename/retire the phrase "successful loaded countermovement jump" for V2.1-R001.
2. Require any future candidate claim to be expressed in the `CLAIM_CEILING.md` tiers and supported by
   the canonical output schema plus independent V&V.
3. Treat historical R001 evidence as reproducibility-only (see `AUTHORITY_INVALIDATION_MATRIX.md`).
