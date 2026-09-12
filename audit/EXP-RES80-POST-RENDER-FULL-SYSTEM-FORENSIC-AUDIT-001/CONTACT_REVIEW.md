# CONTACT_REVIEW (RES-80, independent pass)

Scope: `v2_plant.xml`, `plant.py` (force/CoP/margin/fall), `constants.py`, `measurement.py`,
`tools/res52/core52.py`, `tools/res52/soft_contact.py`, and the contact/support contracts.
Read-only; numeric probes are in `probes/`.

## 1. Foot and collision model inventory

| Item | Evidence | Verdict |
|---|---|---|
| Single rigid box foot 0.30 x 0.12 x 0.02 m (center +0.045 x, -0.030 z of ankle; bottom 0.040 below ankle) | v2_plant.xml:105-109,133-137 | verified |
| One ankle hinge per foot; no MTP/toe/arch | v2_plant.xml:106,134 | verified |
| Floor `contype=1 conaffinity=6`; feet `2/1`; body shells `0/0`; fall shells `4/0` | v2_plant.xml:38-55,64-83,143-153 | verified |
| Contact excludes: pelvis-torso, pelvis-load, pelvis-thighs, torso-load, thigh-shank, shank-foot | v2_plant.xml:143-153 | verified |
| Contact params: `solref=(0.016,1)`, `solimp=(0.99,0.99,0.001,0.5,2)`, friction 0.9, condim 3, elliptic, impratio 10 | v2_plant.xml:19-41,108-109 | verified |
| XML header claims ngeom=14; actual 16 | v2_plant.xml:12 vs constants.py:39 | stale doc (LOW-001) |

## 2. Adjudicated statements

### 2.1 Support margin overstates the active support polygon - CONFIRMED
`plant.py:285-300` builds an axis-aligned box spanning BOTH foot geom centers with fixed half-extents
(0.150, 0.060), ignores foot rotation, and includes inactive/airborne feet. Recomputation of the
production formula over all 121,889 R001 samples: minimum margin 0.08385 m; during true flight
0.09660-0.12852 m; example single-support sample at t=1.06525 (fzR=0 N) reports 0.1463 m. The y-term is
structurally ~0.155 m because the model has no lateral DOF. Gates (E1 0.02 m; RR/tube/handoff 0.05 m)
are trivially satisfied for any state. For flat, symmetric, bilateral feet the box happens to equal the
convex hull, so R001's grounded gates are not wrong for the flat case - but the metric has no general
meaning and is positive with an empty support set.

### 2.2 Inactive-foot lowest point wrong under rotation - CONFIRMED
`core52.py:203-218,226-232` and copies use `dist = geom_xpos[2] - size_z_half (0.010)` and
`p_low = geom_xpos - [0,0,0.010]`. True lowest corner offset is `hx|sin psi| + hz|cos psi|`.
Probe: at ankle q=-0.70 the true lowest z = -0.0582 m vs code +0.0360 m (error 0.0943 m); at +-0.35 rad,
0.0508 m. The z-velocity point is also wrong by up to ~0.15 m per 1 rad/s of foot pitch rate.
Accepted R001 keeps feet essentially flat (max |foot pitch| 0.1384 rad at E9; <=0.004 rad during
stance), so the defect is latent for R001 but codified in the frozen RES-55/RES-52 contracts.

### 2.3 Foot push-off has no heel rise - CONFIRMED as a model-form limitation
R001 foot pitch is 0.0043 rad at E6 takeoff and never rolls; the push-off is a flat-foot knee/hip
extension. A rigid box could pivot on its toe edge, but with no MTP/arch there is no forefoot rocker and
the CoP migration path is discontinuous at the edge. This is task-material if human push-off technique is
claimed; acceptable only under an explicitly reduced-model claim.

### 2.4 CoP origin/frame misdeclared - CONFIRMED
`plant.py:263-277` computes CoP relative to the moving ankle xy projection, not the plate/box center,
and uses a hardcoded `Fz>20 N` validity threshold while the only named per-foot threshold is 10 N
(constants.py:180; TRACE_SCHEMA_V2.md:51 declares "plate frame"). No production controller consumes the
measured CoP (analytic static clamps are used). The world-force transform itself is correct:
`frame.T @ (sign*wrench)` matched `qfrc_constraint` on the root DOFs at tested states, including
non-identity contact frames, and the sign logic handles both geom orderings (P31 REJECTED as an active
defect; the surrounding comments are wrong -> LOW-003).

### 2.5 prohibited_contact is structurally dead - CONFIRMED
The flag scans only shell geoms with `contype=0 conaffinity=0` (plant.py:252-261), which can never
contact. `PROHIB=false` in the canonical result is vacuous; the meaningful fall signal is
`fall_contact` (fall shells `4` vs floor `6`, measurement.py:163-183). `max_penetration` also only
scans floor-foot contacts, so fall-shell penetration is not reported.

### 2.6 Collision topology - LIMITATION (latent)
No body self-collision exists; the 20 kg load has no collision geometry and no fall shell; adjacent
segments are excluded. The foot-shank exclusion had a computed minimum clearance of ~6 mm at the ankle
limit - safe today, fragile under any geometry change. R001 (flat, symmetric, no fall) is not affected;
fall states can show impossible intersections undetected.

### 2.7 Penetration - ACCEPTABLE under the declared compliant-contact model
Standing penetration ~7.9e-5 m; R001 max penetration 0.00964 m = 48% of the 0.02 m box thickness,
within the declared 0.010 m cap but soft-mat-like. Consistent with `solref=(0.016,1)`; inconsistent with
any "rigid floor" wording.

### 2.8 Contact geom ordering/sign - REJECTED as an active bug
The transform is correct for MuJoCo 3.8.0; only fragility/comment issues remain (LOW-003).

## 3. Findings linked to register

| Finding | Register ID | Severity |
|---|---|---|
| Support margin not the active hull; gates trivial; positive in flight | HIGH-008 | HIGH |
| Rotated-foot gap/velocity geometry wrong (up to 0.094 m) | HIGH-009 | HIGH (latent) |
| No MTP/toe/arch; flat-foot push-off, no heel rise | HIGH-019 | HIGH (claim scope) |
| CoP frame/origin misdeclared; 20 N vs 10 N | MED-005 | MEDIUM |
| prohibited_contact dead; MAXPEN incomplete | MED-006 | MEDIUM |
| No self-collision; load no collider; foot-shank ~6 mm | MED-007 | MEDIUM |
| Stale comments / duplicated support literals | LOW-003 | LOW |
| XML ngeom header stale | LOW-001 | LOW |

**Verdict:** no CRITICAL contact defect is active in R001 as run. Two HIGH latent defects (support-margin
semantics; rotated-foot gap/velocity) become critical for any successor that relies on the margin or
uses foot pitch during swing/landing, and both are sealed into project contracts, so they require
authority-level correction rather than local patching.
