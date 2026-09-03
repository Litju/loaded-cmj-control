# Trace Schema V2 — RES-10 Control Experiments

**LCMJ_TRACE_SCHEMA_VERSION=2**
**Status:** CANONICAL for all future RES-10 control experiments (R0.1).
Any future schema change increments the version. Silent field changes are forbidden.

---

## 1. Sampling

| Stream | Rate | Rule | Count for horizon T |
|---|---|---|---|
| physics | 8000 Hz (`PHYSICS_DT=0.000125`) | one sample AFTER every `mj_step`, `t=dt..T` (no `t=0` sample) | `EXPECTED_PHYSICS_STEPS = round(T / 0.000125)` |
| control | 200 Hz (`CONTROL_DT=0.005`, `SUBSTEPS_PER_CONTROL=40`) | one entry per control step `k=0..M-1` at `t=k*control_dt`, action applied then 40 substeps | `EXPECTED_CONTROL_STEPS = round(T / 0.005)` |

Physics/control alignment: control entry `k` precedes physics samples
`(k*40 .. k*40+39)`. `physics_trace.npz:ctrl[i]` is `d.ctrl` AFTER step `i`;
`control_trace.npz:action[k]` is the commanded `u` for interval `k`.

Example: continuation `T=0.5 s` → `N=4000` physics, `M=100` control.

---

## 2. Arrays, shapes, units, frames

Physics (`physics_trace.npz`), `N` = continuation physics steps:

| Array | Shape | Unit | Frame / convention |
|---|---|---|---|
| `time` | `(N,)` | s | world clock, `t=branch+dt..end` |
| `qpos` | `(N,10)` | m/rad | joint order `V2_JOINT_NAMES` (root_tx, root_tz, root_ry, lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle) |
| `qvel` | `(N,10)` | m/s, rad/s | same order |
| `qacc` | `(N,10)` | m/s², rad/s² | `d.qacc` after step |
| `ctrl` | `(N,7)` | unitless `u∈[-1,1]` | actuator order `V2_MJ_ACTUATOR_NAMES` |
| `qfrc_actuator` | `(N,10)` | N / Nm | generalized force, MuJoCo `d.qfrc_actuator` |
| `qfrc_passive` | `(N,10)` | N / Nm | `d.qfrc_passive` (gravity+spring+damper; root must be ~0 stiffness/damping) |
| `qfrc_constraint` | `(N,10)` | N / Nm | `d.qfrc_constraint` (`Jᵀλ`, audits root/joint/contact support) |
| `root_pos` | `(N,3)` | m | pelvis body `xpos`, world |
| `root_vel` | `(N,3)` | m/s | pelvis linear velocity, world (`mj_objectVelocity`) |
| `root_angvel` | `(N,3)` | rad/s | pelvis angular velocity, world |
| `com` | `(N,3)` | m | whole-system COM from `xipos` weighting, world |
| `com_vel` | `(N,3)` | m/s | whole-system COM velocity (mass-weighted body COM velocities), world |
| `H` | `(N,3)` | kg m²/s | centroidal angular momentum about COM, world (derivation §4) |
| `Hy` | `(N,)` | kg m²/s | sagittal component `H[1]` (minimum required) |
| `trunk_tilt` | `(N,)` | rad | angle between torso `+z` and world `+z` |
| `trunk_angvel` | `(N,3)` | rad/s | torso angular velocity, world |
| `torso_xquat` | `(N,4)` | unit quat `wxyz` | torso orientation, world |
| `left_Fz` / `right_Fz` | `(N,)` | N | plantar normal force per foot |
| `left_force` / `right_force` | `(N,3)` | N | plantar force, world |
| `left_moment` / `right_moment` | `(N,3)` | Nm | plantar moment about world origin |
| `left_cop` / `right_cop` | `(N,2)` | m | CoP in plate frame (valid iff `Fz>20`) |
| `cop_valid` | `(N,2)` | bool | per-foot CoP validity |
| `ncon` / `contact_count` | `(N,)` | count | `d.ncon` |
| `contact_geom1/geom2` | `(N,16)` | id | geom ids, `-1` pad |
| `contact_dist` | `(N,16)` | m | `con.dist` (negative = penetration), `NaN` pad |
| `contact_pos` | `(N,16,3)` | m | contact position, world, `NaN` pad |
| `nefc` | `(N,)` | count | `d.nefc` |
| `efc_type` | `(N,128)` | enum | `mjtConstraint` int (`3`=joint limit, `7`=elliptic contact), `-1` pad |
| `efc_id` | `(N,128)` | id | joint/contact id per row, `-1` pad |
| `efc_force` | `(N,128)` | N/Nm | `d.efc_force`, `NaN` pad |
| `support_polygon` | `(N,4)` | m | `[x_min,x_max,y_min,y_max]` from foot geom centers ±`(0.150,0.060)`, world |
| `support_margin` | `(N,)` | m | min distance from COM-xy to polygon edge |
| `actuator_torque` | `(N,7)` | Nm | `limit*u` |
| `actuator_util` | `(N,7)` | fraction | `|u|` |
| `controller_phase` | `(N,)` | enum | `100=HARMLESS_HOLD` (this experiment) |
| `fall_flag` | `(N,)` | bool | fall-shell vs floor contact |
| `prohibited_flag` | `(N,)` | bool | shell-geom vs floor contact |
| `contact_state` | `(N,2)` | bool | `[left_active,right_active]` (`Fz>10 N`) |
| `reflight_flag` | `(N,)` | bool | contact loss after touchdown (false for harmless hold) |

Control (`control_trace.npz`), `M` = continuation control steps:

| Array | Shape | Unit |
|---|---|---|
| `time` | `(M,)` | s |
| `action` | `(M,7)` | `u∈[-1,1]` |
| `phase` | `(M,)` | enum (`100`) |

`branch_control_sequence.npz` duplicates `action` as the exact replay authority.

---

## 3. NaN policy

- Dense physics arrays (`time,qpos,qvel,qacc,ctrl,qfrc_*,root_*,com_*,H,Hy,trunk_*,Fz,forces,moments,margin,torque,util`) must be finite; any nonfinite fails closed.
- Padded tables use `-1` for integer ids/types and `NaN` for floats (`contact_dist,pos,efc_force`) on absent rows.
- `contact_count/ncon/nefc` give the valid prefix length per row.

---

## 4. Derivation formulas (deterministic, no duplication ambiguity)

- **COM:** `com = Σ m_b xipos_b / Σ m_b` over non-world bodies.
- **COM velocity:** `com_vel = Σ m_b v_com,b / Σ m_b`, `v_com,b = v_body,b + ω_b × (xipos_b − xpos_b)`, `v/ω` from `mj_objectVelocity(..., flg_local=0)`.
- **H:** `H = Σ [ Iw_b ω_b + (xipos_b − com) × m_b v_com,b ]`, `Iw_b = R_b diag(body_inertia_b) R_bᵀ`, `R_b` from `ximat`.
- **Trunk tilt:** `acos(clip(R_torso[2,2],−1,1))`.
- **Plantar wrench:** per-contact `wrench` from `mj_contactForce`, world force `F = R_gcᵀ (sign·raw[:3])` with `sign=+1` iff foot geom is `contact.geom[1]` (V2Plant convention); summed per foot; moment about world origin `Σ p × F`.
- **Support polygon:** foot geom centers `(geom_xpos)` ± half-sizes `(hx,hy)=(0.150,0.060)`; margin = min COM-xy distance to edge.
- **Actuator:** `tau = limit·u`, `util = |u|`, limits `250/250/250/300/300/200/200`.
- **Constraint audit:** root-limit rows = `efc_type==3 (mjCNSTR_LIMIT_JOINT)` with joint name in `{root_tx,root_tz,root_ry}` via `mj_id2name`; contact rows = `efc_type==7`.

---

## 5. Contact / event indexing

- Contact indexing: `contact_geom1[i,k],contact_geom2[i,k]` for physics sample `i`, slot `k<contact_count[i]`; geom names via `mj_id2name(mjOBJ_GEOM, id)`.
- Event indexing: `events_online/offline.json:event_records.<name>.{sample_index,confirmed_sample_index}` index into physics samples `0..N-1` of the CONTINUATION (branch-relative, not full-run).
- Event detector state per sample is recoverable from `fall_flag/prohibited_flag/com_* /Fz/margin/tilt` inputs stored above.

---

## 6. Versioning

`LCMJ_TRACE_SCHEMA_VERSION=2` is frozen by this mission. Any added/removed/renamed field, unit/frame change, rate change, or padding change requires `VERSION=3` plus a migration note. Tests assert exact field presence and sample-count consistency.
