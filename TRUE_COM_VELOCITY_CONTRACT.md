# TRUE_COM_VELOCITY_CONTRACT (RES-54, V2.1)

## Authority

`V2Plant.center_of_mass_velocity(data)` returns the true whole-body COM
linear velocity in world (m/s) computed as MuJoCo-native
`mj_jacSubtreeCom(model, data, J, pelvis) @ qvel` on the SAME state passed in.
No history, no finite difference, no `cvel`, no `objectVelocity` transport.

## Root selection

- Proven dynamic subtree root: body 1 `pelvis` (`body_subtreemass == 95.0 kg`
  total athlete+load). World body 0 also spans 95.0 kg but is static; all
  dynamic bodies descend from pelvis. Constructor asserts root mass == total.
- `center_of_mass` (mass-weighted `xipos`) matches `subtree_com[pelvis]` to
  1e-15; velocity authority matches `mj_subtreeVel`-after-`mj_forward` and
  `mj_jacSubtreeCom@qvel` to 1e-16.

## Why Jacobian form

Raw `data.subtree_linvel` without a preceding `mj_forward` lags one physics
step on live post-step data (proven `g*DT` lag); calling `mj_forward` inside
would mutate live `qacc`/contact staging. The Jacobian form is exact on both
the synchronized shadow path (forwarded) and the live trace path (post-step)
without mutating the state.

## Previous defect (retired)

Old code mass-weighted `mj_objectVelocity.linear + omega x (xipos-xpos)`.
`mj_objectVelocity.linear` already equals the `xipos`-point velocity (proven
`jacBodyCom@qvel` identity to 1e-16), so rotation was double-counted:
error `sum(m*omega x offset)/M`, e.g. S50 `[+0.1111, 0, -0.0229]` m/s.

## Use

All paths (`SynchronizedPhysicsSample`, `public_observation`, trace,
`event_sample`, `controller_observation`) call this function; corrected value
flows to every `com_vx`/`com_vz`/momentum/apex/reversal consumer with no
threshold, dwell, Plant, contact, solver, actuator, or controller-parameter
change.
