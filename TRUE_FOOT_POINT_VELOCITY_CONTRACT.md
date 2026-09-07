# TRUE_FOOT_POINT_VELOCITY_CONTRACT (RES-55, V2.1)

## Authority

`true_foot_point_velocity(model, data, body_id, point_world)` returns the true
material foot-point linear velocity in world (m/s) computed as MuJoCo-native
`mj_jac(model, data, Jp, Jr, point, body) @ qvel` on the SAME synchronized
state passed in. No history, no finite difference, no `cvel`, no
`objectVelocity` transport. No `mj_forward` on live solely for measurement
refresh.

Two identical implementations (proven equivalent by construction):
- `tools/res52/core52.py:true_foot_point_velocity`
- `tools/res52/soft_contact.py:true_foot_point_velocity`

## Point selection (preserved)

Per foot L/R, from synchronized same-state `MjData`:
- If deepest floor contact row exists: `dist=con.dist`, `nvel=efc_vel[efc_address]`
  (MuJoCo `d(dist)/dt`, already exact point-velocity authority, retained).
- Else (separated): `p_low=geom_xpos[foot_box]-[0,0,gap_half_height]`
  (lowest foot-box corner, world), `dist=geom_xpos[fg][2]-gap_half_height`,
  `nvel=true_foot_point_velocity(...fb,p_low)[2]`.

Deepest (min `dist`) row wins per foot. L/R independent. No averaging.
`gap_half_height=model.geom_size[left_foot_box][2]` (frozen model, never tuned).

## Normal definition (preserved)

World `+z`. `foot_normal_velocity=d(dist)/dt`.
POSITIVE=SEPARATING (distance increasing), NEGATIVE=APPROACHING.
Verified at S50 seed unloading `+0.2129` separating at 8.15 mm penetration;
`efc_vel==J(contact)@qvel` to `0.00e+00`; standing `~0`.

## Previous defect (retired)

Old inactive code:
`v_body=mj_objectVelocity(BODY).linear; v_pt=v_body+omega x (p-xpos)`.
`mj_objectVelocity.linear` already equals velocity at `xipos` (proven
`jacBodyCom@qvel` identity `1e-19..7e-16`; `obj-Jxpos` `6e-08..3.7e-02`),
so rotation was double-counted by `omega x (xipos-xpos)`:
S40 `-0.0204`, S50 `+0.0167`, S75 `+0.0201`, S100 `+0.0339` m/s normal
(33–68% of `0.05` threshold); P3 inactive `0.35` RMS, `0.40` MAX.
`soft_contact.y_of` additionally used geom center vs corner (z-identical,
now unified to `p_low`).

## Correct transport (proven equivalent)

`M1=J_point@qvel` agrees with `M2=v_xipos+omega x (p-xipos)` to
`1e-19..6e-16` (tol `1e-09/1e-08` PASS) and with `M3=centered FD(p_low(t))`
inside derived physics-rate tolerance (smooth `5e-04/2e-03`, impact
`5e-03/2e-02`; P1b marginal `5.13e-04` vs `5.00e-04` noted, MAX PASS).
Runtime uses `M1` (non-mutating, exact on both shadow-forwarded and
live post-step paths).

## Use

All `tools/res52` foot-normal paths (`soft_contact_state`, `SoftContactPolicy.y_of`,
`TraceAcc`, branch authority, viability map, local-model `G[5,6]`, `solve_du`
`W_NVEL`, validation `NVEL_ABS 0.05`, `QUALIFIED_NVEL_DOMAIN [-0.35,0.35]`)
consume this authority with unchanged point/normal/aggregation semantics.
Corrected `RES-54` whole-body COM velocity untouched. No threshold, gain,
profile, contact-parameter change.

## Prohibition

Any manual transport from `mj_objectVelocity(BODY).linear` using `(p-xpos)`
is prohibited in `tools/res52`. Regression test greps for the pattern.
