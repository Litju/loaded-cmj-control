# CLAIM_CEILING (RES-80)

Exact scientific claim ceiling for V2.1-R001 after the post-render full-system forensic audit.
No repair, no reinterpretation of the frozen disposition.

## PROVEN (supported by reproducible evidence)

- Deterministic simulation reproducibility of the R001 episode in the pinned environment
  (TRACE `4d047879...`, N_CTRL 3048, N_PHYS 121889, T_END 15.236125 s, substep histogram 3045x40/2x28/1x33).
- Exact action-schedule execution as sealed (schedule replay reproduces the trace; online/offline
  detector identity PASS).
- Declared 12-event predicate chain completion with the sealed occurrence/confirmation times.
- Declared numerical hard gates (finite states, no fall, no root limit rows, zero root passive force,
  post-landing support slice, peak utilization within bounds).
- Render/replay identity: the RES-79 render consumed the exact accepted arrays (non-invasive).
- Correctness of the physics-facing primitives independently re-verified in this audit: contact-force
  transform, COM velocity authority, state staging/ctrl inclusion, zero root damping.

## NOT PROVEN (no admissible evidence in the repository)

- Biomechanical human validity or human-like technique.
- Loaded-CMJ realism (no meaningful jump magnitude, flat-foot push-off, reverse-knee kinematics).
- Realistic foot mechanics (no MTP/toe/arch, no heel rise).
- Any quantitative performance claim (jump height 0.0363 m; rise 0.0768 m; flight 0.2268 s;
  clearance 0.0496 m; takeoff vz 1.2274 m/s - measured here, not gated by the system).
- Realistic landing strategy (vertical-only capture, one-sided balance, RR without posture).
- Realistic recovery strategy (one-trajectory micron envelope, 13.15 s recovery from a 24-degree lean).
- Population generality or perturbation robustness.
- Experimental validation.
- True frozen-candidate identity and clean-install deployment.

## FAILED

- Owner physical/visual credibility (frozen disposition; CONFIRMED by this audit with quantified causes:
  reverse-knee kinematics, premature takeoff switch with a -9.68 rad/s pitch whip, 3.6 cm hop, forward
  landing lunge, absent foot mechanics).

## Justification of the phrase "successful loaded countermovement jump"

Not justified. Under the executable semantics, the phrase reduces to "event-chain and numerical-gate
completion with a deterministic render"; biomechanics and performance are invalid or unmeasured, the
candidate identity is not enforceable, and the test suite cannot substantiate the claim. The task
contract itself lacks any minimum performance requirement, which is a specification defect rather than
a property of R001.

## Narrow valid claim

> V2.1-R001 is a bit-exactly reproducible deterministic simulated jump-episode whose declared V2.1 event
> predicates and numerical gates complete, but whose kinematics are anatomically inverted and whose
> jump magnitude is trivial; it is a computational/reproducibility artifact, not a successful loaded
> countermovement jump.

## Ceiling after future repair (for planning only - not authorized here)

A successor may claim "successful loaded countermovement jump" only when: (a) the ankle/knee/hip
anatomy is human-valid; (b) a separately approved task contract defines minimum performance, landing and
recovery criteria; (c) the canonical result exposes those metrics; (d) independent V&V plus owner visual
review pass; and (e) candidate identity freezes the full executing system. Until then, the ceiling
remains the narrow claim above.
