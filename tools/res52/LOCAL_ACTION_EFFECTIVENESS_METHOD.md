# LOCAL_ACTION_EFFECTIVENESS_METHOD — RES-52

EXPERIMENT_ID = EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001

## Principle

MuJoCo's contact authority is compliant and unilateral. A rigid-contact
lambda from an external QP is NOT the physical contact force. This layer
therefore uses a **local exact-forward action-effectiveness model**: every
sensitivity is measured by running the actual frozen Plant forward for the
full control interval on independent copies of the exact synchronized state.
No analytical contact model, no approximate Jacobian, no derivative across a
contact switch is used as if globally valid.

## Per-control-update procedure (every 5 ms)

1. **Capture**: read the exact live `mjSTATE_INTEGRATION` vector `x_k`
   (synchronized sample-before-update authority; the sample SHA256 is
   asserted against the live state every update).
2. **Nominal branch**: restore `x_k` into an independent MjData copy, apply
   the nominal action `u_prev` (the previously applied action — continuity
   walk), advance EXACTLY 40 physics substeps (5 ms) with `mj_step`, then
   `mj_forward` and read the terminal outputs `y_nom`.
3. **Perturbation probes**: for each of the 7 actuator channels j, apply
   `u_prev ± ρ·e_j` (two-sided; ρ = current trust radius = 0.25) on
   independent copies; 14 branches. Terminal outputs `y_j±`. If a bound
   clips one side, the surviving side gives a one-sided slope.
4. **Local map**: `G = ∂y/∂u`, columns
   `(y_j+ − y_j−)/(Δ+ + Δ−)`. Outputs
   `y = [Fz_whole, Fz_L, Fz_R, dist_L, dist_R, nvel_L, nvel_R, COM_vz,
   qdot_0..6, Fzmin_whole, Fzmin_L, Fzmin_R]` (18 outputs).
   `Fzmin_*` are the interval-minimum instantaneous plantar forces recorded
   over the 40 substeps — the intra-interval contact-loss preview.
5. **Active-set detection**: the contact-active flags (per-foot Fz > 10 N)
   of every probe are compared with the nominal branch. Any difference is
   recorded as an active-set crossing (counter + per-update flags). A
   validation branch that changes the contact set against the nominal is
   REJECTED (trust failure → shrink).
6. **Bounded solve**: deterministic bounded-variable least squares (BVLS)
   for `Δu ∈ R^7` on the weighted residual stack:
   - whole-Fz tracking (target = predeclared profile at interval end),
   - per-foot Fz (symmetric half split),
   - foot-normal-velocity → 0 (separation damping),
   - one-sided contact-distance rows (re-engage / 9 mm cap / 0.5 mm
     retention floor),
   - COM vz → profile integral from cell start,
   - joint-rate damping (target 0),
   - one-sided interval-minimum-Fz rows (dip suppression),
   - action regularization.
   Bounds: `|u| ≤ 1`, `|Δu| ≤ ρ` (trust region as action rate).
   Two-pass one-sided row refinement (predeclared, ≤3 passes).
7. **Validation branch**: the chosen action `u = clip(u_prev + Δu)` runs on
   one more independent copy over the same 5 ms. The exact result must agree
   with the linear prediction inside the predeclared per-channel tolerance
   (floor + 10 % of the predicted change) AND preserve the nominal contact
   set.
8. **Trust-region handling**: on failure, ρ ← ρ/2 (floor 0.0025) and the
   map+solve are recomputed at the shrunken radius (max 3 shrinks per
   update). If still failing, the applied action falls back to `u_prev`
   (exact branch by construction). Between successful updates ρ recovers by
   2^(1/4) toward 0.25. **No outer parameter search ever occurs.**
9. **Apply**: only the validated (or exact-fallback) action is written to
   LIVE (`plant.apply_action`); LIVE advances ONLY with exact `mj_step`.

## Honesty properties (audited)

- `FULL_7DOF_ACTION_SPACE_USED = true` — all 7 channels are perturbed and
  solved every update; the effect of every channel is visible in the sealed
  per-cell `G` matrices (`trace_*.npz`, `G` array 18×7 per update).
- `OLD_EXT_DIR_USED = false` — no fixed direction vector exists anywhere in
  the layer; the audit test greps the layer source and the run diagnostics.
- No direct qpos/qvel/root/contact-force writes; no applied-force channels;
  probes and validation run only on copies; LIVE is never forwarded for
  reporting (shadow authority only).
- Desired Fz is never treated as actual Fz: feedback uses only synchronized
  measured forces (`y_nom` read from the shadow, validated per update).

## Cost

17 branches × 40 substeps per update. Full six-cell matrix ≈ 20 s wall time.
