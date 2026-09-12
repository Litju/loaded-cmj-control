#!/usr/bin/env python3
"""RES-80 audit probe A: Plant geometry, joint sign convention, foot geometry.

Read-only. Does not import production modules except the XML via mujoco directly.
"""
import numpy as np
import mujoco

XML = "/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m = mujoco.MjModel.from_xml_string(open(XML).read())
d = mujoco.MjData(m)

def reset(qpos):
    mujoco.mj_resetData(m, d)
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)

# qpos order: root_tx, root_tz, root_ry, lumbar, lhip, lknee, lank, rhip, rknee, rank
base = np.array([0, 0.9, 0, 0, 0, 0, 0, 0, 0, 0], float)

print("=== PROBE A1: knee angle sign convention ===")
print("left_knee axis in XML: 0 -1 0; range 0..2.40 (declared '0 extended, flexion positive')")
nm = lambda o, i: mujoco.mj_id2name(m, o, i)
for knee in [0.0, 0.4, 0.8, 1.2]:
    q = base.copy(); q[5] = knee  # left_knee qpos index
    reset(q)
    hip = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_thigh")]
    kne = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_shank")]
    ank = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_foot")]
    # knee joint location = shank body origin; ankle = foot body origin
    print(f"knee_q={knee:.2f}  knee_pos(x={kne[0]:+.4f},z={kne[2]:.4f})  ankle_pos(x={ank[0]:+.4f},z={ank[2]:.4f})  ankle_x-knee_x={ank[0]-kne[0]:+.4f}")

print()
print("=== PROBE A2: foot lowest point vs rotation (true rotated box corners) ===")
# foot box: in foot body frame center (0.045, 0, -0.030), half sizes (.15,.06,.01)
# geom xmat available via d.geom_xmat; pos via d.geom_xpos
lg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "left_foot_box")
half = np.array([0.150, 0.060, 0.010])
for ankle in [-0.7, -0.35, 0.0, 0.35, 0.5]:
    q = base.copy(); q[6] = ankle
    reset(q)
    R = d.geom_xmat[lg].reshape(3, 3)
    c = d.geom_xpos[lg]
    corners = np.array([c + R @ (half * np.array(s)) for s in
                        [(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]])
    lowest = corners[:, 2].min()
    # plant's "lowest foot material point" assumption: geom center z - halfheight?
    naive = c[2] - half[2]
    print(f"ankle_q={ankle:+.2f}  geom_center_z={c[2]:.4f}  true_lowest_z={lowest:.4f}  naive_center_minus_halfz={naive:.4f}  err={lowest-naive:+.5f}")

print()
print("=== PROBE A3: contact frame convention ===")
# Hold the model in steady stance and read the floor contact frame
reset(base)
mujoco.mj_step(m, d)
for i in range(d.ncon):
    c = d.contact[i]
    g1, g2 = c.geom1, c.geom2
    n1 = nm(mujoco.mjtObj.mjOBJ_GEOM, g1)
    n2 = nm(mujoco.mjtObj.mjOBJ_GEOM, g2)
    if "floor" in (n1, n2) and "foot" in (n1 + n2):
        R = c.frame.reshape(3, 3)
        print(f"contact {n1} x {n2}: frame=\n{R}\n dist={c.dist:.6f} pos={c.pos}")
        w = np.zeros(6); mujoco.mj_contactForce(m, d, i, w)
        print(f"  contact force (contact frame) = {w[:3]}")
        print(f"  frame @ f   = {R @ w[:3]}")
        print(f"  frame.T @ f = {R.T @ w[:3]}")
        # world force on geom2 (foot if g2 is foot)
        print(f"  geom1={n1}, geom2={n2}")
        break
