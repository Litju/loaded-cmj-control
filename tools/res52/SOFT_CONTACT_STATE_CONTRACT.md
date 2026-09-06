# SOFT_CONTACT_STATE_CONTRACT — RES-52

EXPERIMENT_ID = EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001
MISSION = RES10_SYNC_SOFT_CONTACT_FORCE_REALIZATION_AUTHORITY_001

## Scope

Per-foot soft-contact state variables derived ONLY from synchronized
same-state MuJoCo data (shadow `mj_forward` of the exact live
`mjSTATE_INTEGRATION` vector). No quantity is estimated from stale data,
integrated open-loop, or read from the live data after mutation.

## Per-foot variables (i ∈ {L, R})

1. `contact_distance_i` — signed distance [m].
   - Foot in floor contact: the `con.dist` of the deepest (most negative)
     floor contact row of that foot's box geom. Negative = penetrating.
   - Foot not in floor contact: geometric gap of the lowest foot point above
     the floor, `foot_box_z − half_height` (positive). The half-height is
     read from the frozen model (`geom_size` z extent), never tuned.
2. `penetration_i = max(0, −contact_distance_i)` [m].
3. `foot_normal_velocity_i` [m/s], signed **POSITIVE = SEPARATING**
   (distance increasing), **NEGATIVE = APPROACHING** (penetration growing).
   - Active contact: `efc_vel[contact.efc_address]` of the corresponding
     constraint row — MuJoCo's own d(dist)/dt for that contact row.
   - Inactive foot: world-z velocity of the lowest foot point computed from
     the synchronized body spatial velocity (`mj_objectVelocity`) projected
     on the floor normal. Positive = moving away from the floor.
4. `actual_Fz_i` — synchronized per-foot plantar normal force [N] from
   `foot_contact_summary` (contact-frame force rotated to world, z component
   summed over that foot's floor contacts). This is the MEASURED MuJoCo
   contact force, never a commanded/desired force.
5. `contact_active_i` — `actual_Fz_i > V2_CONTACT_FZ_THRESHOLD_N (=10 N)`.

## Standing reference (observed, not invented)

From `S_STAND` (RES-43 true-standing authority: HOLD Kp400 Kd10, q_ref=0, on
the zero-root-damping Plant, captured at t=1.0 s under the synchronized
sample-before-update convention):

| quantity | value |
|---|---|
| PEN_EQ_L | 7.9208e-05 m (0.0792 mm) |
| PEN_EQ_R | 7.9208e-05 m (0.0792 mm) |
| FZ_EQ_L  | 465.975 N |
| FZ_EQ_R  | 465.975 N |
| NORMAL_VEL_EQ | 0 (by definition of the equilibrium) |
| FZ_EQ_WHOLE | 931.950 N = 1.000 body weight (M=95.0 kg, g=9.81) |

These are OBSERVED equilibrium values. No penetration preload target is
invented anywhere in this mission.

## Sign-verification evidence

- At S50 the seed branch is unloading: contact force decays and
  `foot_normal_velocity = +0.212 m/s` (separating), penetration 8.18 mm —
  sign convention confirmed against the known unloading trajectory.
- During free approach (S100 capture window) negative values occur while the
  penetration recovers — approaching motion.
- `efc_vel` cross-checked against finite differences of `con.dist` over one
  physics step (test suite).

## Controller usage (Phase C)

The realization layer regulates exactly these variables through the local
exact-forward map: whole/L/R `actual_Fz`, per-foot signed
`contact_distance`, per-foot `foot_normal_velocity`, COM vz, joint rates,
and interval-minimum Fz. Penetration is never commanded directly; the
one-sided retention rows act on signed distance only:
- re-engage when separated (`dist > 0` → target 0);
- penetration cap `dist ≥ −9.0 mm` (hard 10 mm limit margin);
- compression-retention floor `dist ≤ −0.5 mm` when a sub-body-weight
  support demand is active (contact-retention margin, NOT a force preload;
  the force target remains the predeclared profile).

Never demanded: tensile ground force. Never assumed: desired Fz = actual Fz.
