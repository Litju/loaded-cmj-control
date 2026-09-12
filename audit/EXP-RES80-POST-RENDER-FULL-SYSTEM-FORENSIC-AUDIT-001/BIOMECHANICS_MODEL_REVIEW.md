# BIOMECHANICS_MODEL_REVIEW (RES-80, independent pass)

Scope: `v2_plant.xml`, `constants.py`, `plant.py`, `measurement.py`, and the sealed R001 replay arrays.
Read-only; forward-kinematics probes in `probes/probe_geometry.py`, `probes/probe_skeleton.py`.

## 1. Joint-direction verification (the central model-form finding)

Exact planar chain from the frozen XML (thigh L=0.43, shank L=0.43, hinges about ±y):

- **Knee (axis `0 -1 0`, range [0, 2.40])**: for positive q_k the knee->ankle displacement is
  `(+0.43 sin q_k, -0.43 cos q_k)`. Probe values: q=0.4 -> ankle_x - knee_x = +0.167 m;
  q=0.8 -> +0.309 m; q=1.2 -> +0.401 m. **Positive q moves the ankle anteriorly**. Human knee flexion
  moves the ankle posteriorly (heel toward buttocks); it requires q_k < 0, which the range excludes.
  The comment "0 extended, flexion positive" is false for the implemented axis.
- **Hip (axis `0 1 0`, range [-0.50, 1.80])**: positive q_h moves the knee posteriorly (extension).
  Only 0.50 rad of hip flexion is available; a human loaded-CMJ crouch requires roughly 0.8-1.5 rad.

Accepted R001 confirms the consequence: at t=0.4875 s (deepest countermovement) knee q=+1.269 rad and
the knee is 29 cm posterior to the ankle plane; at E6 takeoff the knee is still +0.54 rad in the same
reversed direction; during landing (E9-E11) the knee is 12-16 cm posterior to the ankle. The motion is a
**reverse-knee ("bird-leg") countermovement** - an anatomical mirror of a human CMJ.

Verdict: **CRITICAL** (MODEL_FORM_DEFECT). No biomechanical interpretation of this trajectory is valid.

## 2. Anthropometry

- Segment lengths: thigh 0.43 m, shank 0.43 m (hip-to-ankle 0.86 m), foot 0.30 m - plausible for a
  ~1.8 m athlete; foot length is at the high end.
- Masses: athlete 75 kg (pelvis 10.65, torso/head/arms aggregate 40.2, thigh 7.5 x2, shank 3.4875 x2,
  foot 1.0875 x2) + 20 kg load. Pelvis COM at +0.070 m; torso COM at +0.260 m - plausible.
- Inertia: pelvis (0.0870, 0.0401, 0.0923); torso (1.7219, 1.4807, 0.6271); load (3.7531, 0.00625,
  3.7531) corresponding to a 20 kg, 1.5 m bar. Bilateral symmetry is exact.
- External load: modeled as a rigid 1.5 m cylindrical bar (radius 0.025) attached to the torso at
  +0.42 m, i.e. a back-squat-style barbell. Reasonable embodiment of "20 kg load"; no compliance or
  strap dynamics.

## 3. Model-form limitation classification

| Limitation | Class | Why |
|---|---|---|
| Sagittal reduction (x,z,pitch only; no lateral/roll/yaw) | ACCEPTABLE_CURRENT_SIMPLIFICATION for a bilateral symmetric task; UNKNOWN_REQUIRES_VALIDATION for asymmetric landings | R001 has zero unilateral samples; lateral strategies are unrepresentable |
| Aggregated torso/head/arms (single rigid body with one lumbar hinge) | ACCEPTABLE_CURRENT_SIMPLIFICATION with caveat | No spine/arm swing; trunk CoM tuning still allows a plausible pitch profile, but arm-swing momentum (a real CMJ contributor) is absent |
| Rigid 20 kg load | ACCEPTABLE_CURRENT_SIMPLIFICATION | Barbell treated as rigid inertia |
| Rigid single-segment foot, no MTP/toe/arch (flat push-off) | REQUIRES_SUCCESSOR_MODEL for human push-off fidelity | R001 never rolls the foot (max stance pitch 0.004 rad); claim must exclude foot mechanics |
| Joint direction/range conventions (knee reversed; hip extension-biased) | REQUIRES_SUCCESSOR_MODEL (CRITICAL defect) | Kinematically excludes human knee flexion |
| Direct torque actuation, no muscle activation | REQUIRES_SUCCESSOR_MODEL or explicit claim exclusion | 0.5-0.94 action steps at 200 Hz are not physiological; no force rise dynamics |
| Passive joint stiffness/damping (15/8/5 Nm family) | ACCEPTABLE_CURRENT_SIMPLIFICATION | Transparent constants; not tuned to human passive properties but bounded |
| Contact compliance (solref 0.016) and 9.6 mm peak penetration | ACCEPTABLE under declared compliant-contact model | Must not be described as rigid-floor contact |
| Collision exclusions, no self-collision, load has no collider | UNKNOWN_REQUIRES_VALIDATION (fall states) | Never exercised in R001 |
| Anthropometric generality (one 95 kg subject, symmetric) | UNKNOWN_REQUIRES_VALIDATION | No population robustness |
| Bilateral symmetric action enforcement (controller duplicates channels; recovery adds symmetry rows) | ACCEPTABLE_CURRENT_SIMPLIFICATION with robustness caveat | Single-support/asymmetric strategies are not exercised |

## 4. Owner-observation mapping

| Owner observation | Mechanism (evidence) |
|---|---|
| Non-human knee/leg backward-forward flick during squat/extension | Reversed knee hinge (knee apex posterior); knee q rises to +1.269 then falls while the hip reverses from +0.59 to -0.32 (CRIT-001/002) |
| Whole-body backward motion + violent transition at push-off/takeoff | Premature flight switch + zero-pose flight PD; pelvis pitch rate -9.68 rad/s at t=0.65; pelvis x travels +0.073 -> -0.028 m (CRIT-007) |
| Extremely little visible flight / low jump | Net jump height 0.0363 m above standing; 0.0768 m rise; max foot clearance 0.0496 m; no minimum-height gate (CRIT-003) |
| Non-human forward lunge/bounce on landing | Inherited vx 0.181 m/s plus impact-added Hy (3.33 -> 9.88 kg m²/s); braking-only balance authority; E10 vertical-only predicate; RR posture-free (HIGH-001/002/003/004) |
| Inadequate foot/toe-off mechanics | No MTP/toe; flat-foot push-off (foot pitch 0.0043 rad at takeoff) (HIGH-019) |
| Numerical gates did not reject | Event gates are force/dwell latches; no performance or posture gates (CRIT-003, P43/P44) |

## 5. Verdict

The accepted motion is a deterministic but anatomically mirrored loaded jump with a trivial hop
magnitude, a control-induced takeoff whip and an insufficient landing strategy. The biomechanical claim
"successful loaded countermovement jump" is not defensible. The model can be repaired toward human
kinematics only with a joint-anatomy correction (knee sign/range, hip flexion range) and full downstream
sign/controller/event requalification.
