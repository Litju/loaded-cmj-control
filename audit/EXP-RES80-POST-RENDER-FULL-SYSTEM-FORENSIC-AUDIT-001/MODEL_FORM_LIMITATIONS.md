# MODEL_FORM_LIMITATIONS (RES-80)

Classification: **ACCEPTABLE_CURRENT_SIMPLIFICATION** (does not block a reduced-model claim if declared),
**REQUIRES_SUCCESSOR_MODEL** (material to the intended loaded-CMJ task), **UNKNOWN_REQUIRES_VALIDATION**
(acceptability cannot be judged from this repository's evidence).

| # | Limitation | Class | Why / Evidence |
|---|---|---|---|
| 1 | Sagittal reduction (x, z, pitch only; lateral/roll/yaw welded) | ACCEPTABLE for the symmetric bilateral task; UNKNOWN_REQUIRES_VALIDATION for asymmetric landings | `constants.py:31`; R001 has zero unilateral support samples; lateral strategies unrepresentable, and lateral stability metrics are vacuous (y-margin ~0.155 m by construction). |
| 2 | Aggregated torso/head/arms (single rigid body + one lumbar hinge) | ACCEPTABLE_CURRENT_SIMPLIFICATION, with claim caveat | 40.2 kg torso aggregate; no spine articulation or arm swing (a real CMJ momentum contributor). Trunk pitch program exists, so the coarse motion is representable; internal trunk dynamics are not. |
| 3 | Rigid 20 kg external load (1.5 m bar) | ACCEPTABLE_CURRENT_SIMPLIFICATION | Inertia only; no strap/sling compliance; no collision (MED-007). Appropriate for a barbell-style load until fall/contact states matter. |
| 4 | Rigid single-segment foot; no MTP, no toe, no arch, no plantar fascia | **REQUIRES_SUCCESSOR_MODEL** for human push-off claims | `v2_plant.xml:105-109`; R001 foot pitch <= 0.004 rad during stance and 0.0043 rad at takeoff - no heel rise/forefoot rocker; CoP cannot migrate across a forefoot (HIGH-019). |
| 5 | Muscle/activation abstraction: direct bounded torque, no activation state | **REQUIRES_SUCCESSOR_MODEL** for human dynamics claims | `drive.py:25-36`; command steps up to 0.944 per 5 ms; no force rise times (HIGH-006). |
| 6 | Joint anatomy conventions (knee reversed, hip extension-biased, ankle sign label inverted) | **REQUIRES_SUCCESSOR_MODEL** (critical) | CRIT-001/002; LOW-007. Human knee flexion is outside the joint range. |
| 7 | Passive joint terms (stiffness 15, damping 5-8, armature 0.008-0.02) | ACCEPTABLE_CURRENT_SIMPLIFICATION | Bounded, explicit, identical bilaterally; not fitted to human passive properties but not tuned to fake results either. |
| 8 | Contact compliance `solref=(0.016,1)`, friction 0.9, elliptic cone, peak penetration 9.6 mm | ACCEPTABLE under the declared compliant-contact model; wording-limited | Physically consistent with a compliant mat; must not be described as rigid-floor contact (CONTACT_REVIEW §2.7). |
| 9 | Collision exclusions: no self-collision, no load collider, adjacent-segment exclusions | UNKNOWN_REQUIRES_VALIDATION (fall states) | MED-007; not exercised by R001; ~6 mm foot-shank clearance is a standing fragility in the exclusion. |
| 10 | Anthropometric generality: one 95 kg subject, exact bilateral symmetry, controller enforces symmetry | UNKNOWN_REQUIRES_VALIDATION | No population or perturbation study; recovery manifold and envelope are single-trajectory artifacts (HIGH-012). |
| 11 | Support polygon represented by a full-foot AABB over both feet | **REQUIRES_SUCCESSOR_MODEL** (measurement authority) | HIGH-008; gates trivially satisfied; positive in flight. |
| 12 | CoP measured relative to the moving ankle and never fed back | REQUIRES_SUCCESSOR_MODEL (measurement/control) | MED-005; balance/recovery use static analytic CoP bounds, not measured CoP. |
| 13 | Fixed control rate 200 Hz, no sensor noise, no latency, perfect state | ACCEPTABLE for a deterministic reference study; UNKNOWN for deployment | Deterministic trace demonstrates it, but nothing in R001 tests robustness to noise/delay. |
| 14 | Free root with no artificial support | ACCEPTABLE (positive finding) | Root damping/stiffness/armature zero; no limit rows in R001 (`MAXROOTPASSIVE=0.0`, `ROOT_ROWS=0`). |
| 15 | No toe/heel collision shells; fall detection only via six shell capsules | ACCEPTABLE for R001; UNKNOWN for falls | Fall detection functional for shell-floor contact only; load and self-collision unmonitored (MED-007). |

## Materiality summary

Of the fifteen, three are **task-invalidating for human claims if left as-is** (4, 5, 6), two are
**measurement authorities that must be rebuilt** (11, 12), and the rest are either acceptable reduced-model
choices or unvalidated assumptions that must be declared. No simplification in the list is classified as
a software bug solely because it is a simplification; each is tied to a concrete claim it limits.
