#!/usr/bin/env python3
"""RES-80 audit probe B: deterministic R001 trajectory quantification.

Loads the sealed RES-79 replay arrays (qpos/qvel/time) and recomputes, from the
frozen Plant XML only, all jump/landing/recovery metrics. Read-only.
Outputs /tmp/opencode/res80/traj_metrics.npz + JSON summary.
"""
import json
import numpy as np
import mujoco

EVID = "/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
XML = "/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"

m = mujoco.MjModel.from_xml_string(open(XML).read())
d = mujoco.MjData(m)
T = np.load(EVID + "replay_time.npy")
Q = np.load(EVID + "replay_qpos.npy")
V = np.load(EVID + "replay_qvel.npy")

bid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in
       ["pelvis", "torso_head_arms", "external_load", "left_thigh", "left_shank", "left_foot",
        "right_thigh", "right_shank", "right_foot"]}
gid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in
       ["floor", "left_foot_box", "right_foot_box"]}
qadr = {n: int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in
        ["root_tx", "root_tz", "root_ry", "lumbar", "left_hip", "right_hip",
         "left_knee", "right_knee", "left_ankle", "right_ankle"]}
vadr = {n: int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in
        ["root_tx", "root_tz", "root_ry", "lumbar", "left_hip", "right_hip",
         "left_knee", "right_knee", "left_ankle", "right_ankle"]}

half = np.array([0.150, 0.060, 0.010])
corners_sign = np.array([(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)

n = len(T)
com = np.zeros((n, 3)); cv = np.zeros((n, 3))
fzL = np.zeros(n); fzR = np.zeros(n); fxL = np.zeros(n); fxR = np.zeros(n)
lowL = np.zeros(n); lowR = np.zeros(n)
nvL = np.zeros(n); nvR = np.zeros(n)
conL = np.zeros(n, int); conR = np.zeros(n, int)
pitch = np.zeros(n); pelvis_z = np.zeros(n)
comdot_fd = np.zeros((n, 3))
mass = m.body_mass

uf = np.zeros((3, m.nv))
for i in range(n):
    d.qpos[:] = Q[i]
    d.qvel[:] = V[i]
    mujoco.mj_forward(m, d)
    com[i] = (mass[:, None] * d.xipos).sum(0) / mass.sum()
    mujoco.mj_jacSubtreeCom(m, d, uf, bid["pelvis"])
    cv[i] = uf @ V[i]
    # per-foot force + lowest corner
    for side, fgeom, body, fzarr, fxarr, lowarr, nvarr, conarr in (
            ("L", gid["left_foot_box"], bid["left_foot"], fzL, fxL, lowL, nvL, conL),
            ("R", gid["right_foot_box"], bid["right_foot"], fzR, fxR, lowR, nvR, conR)):
        F = np.zeros(3)
        cnt = 0
        for ci in range(d.ncon):
            c = d.contact[ci]
            g1, g2 = int(c.geom1), int(c.geom2)
            other = g2 if g1 == gid["floor"] else (g1 if g2 == gid["floor"] else -1)
            if other != fgeom:
                continue
            cnt += 1
            w = np.zeros(6)
            mujoco.mj_contactForce(m, d, ci, w)
            sign = 1.0 if fgeom == g2 else -1.0
            F += c.frame.reshape(3, 3).T @ (sign * w[:3])
        fzarr[i] = F[2]; fxarr[i] = F[0]; conarr[i] = cnt
        R = d.geom_xmat[fgeom].reshape(3, 3)
        cpos = d.geom_xpos[fgeom]
        pts = cpos[None, :] + corners_sign @ (R * half).T
        lowarr[i] = pts[:, 2].min()
        Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv))
        idxlow = int(np.argmin(pts[:, 2]))
        mujoco.mj_jac(m, d, Jp, Jr, pts[idxlow], body)
        nvarr[i] = (Jp @ V[i])[2]
    pitch[i] = Q[i, qadr["root_ry"]]
    pelvis_z[i] = d.xpos[bid["pelvis"]][2]

# central-difference COM velocity check (interior)
comdot_fd[1:-1] = (com[2:] - com[:-2]) / (T[2:] - T[:-2])[:, None]
err_fd = np.abs(comdot_fd[1:-1] - cv[1:-1]).max()

res = dict(T=T, Q=Q, V=V, com=com, cv=cv, fzL=fzL, fzR=fzR, fxL=fxL, fxR=fxR,
           lowL=lowL, lowR=lowR, nvL=nvL, nvR=nvR, conL=conL, conR=conR,
           pitch=pitch, pelvis_z=pelvis_z)
np.savez_compressed("/tmp/opencode/res80/traj_metrics.npz", **res)

# event times from sealed extraction
EV = json.load(open(EVID + "extraction_report.json"))["recomputed_events"]
def occ(name): return float(EV[name]["occurred_at"])
def conf(name): return float(EV[name]["confirmed_at"])
def idx_at(t): return int(np.argmin(np.abs(T - t)))

out = {}
out["com_velocity_fd_max_err"] = float(err_fd)
com_z = com[:, 2]
out["com_z_initial"] = float(com_z[0])
out["com_z_max"] = float(com_z.max())
out["com_z_at_E6_occ"] = float(com_z[idx_at(occ("bilateral_takeoff"))])
out["com_z_at_E6_conf"] = float(com_z[idx_at(conf("bilateral_takeoff"))])
out["com_vz_at_E6_occ"] = float(cv[idx_at(occ("bilateral_takeoff")), 2])
out["com_vz_at_E6_conf"] = float(cv[idx_at(conf("bilateral_takeoff")), 2])
apex_i = int(np.argmax(com_z))
out["apex_i"] = apex_i
out["apex_t"] = float(T[apex_i])
out["com_rise_from_E6occ_to_apex"] = float(com_z.max() - com_z[idx_at(occ("bilateral_takeoff"))])
out["com_rise_from_E6conf_to_apex"] = float(com_z.max() - com_z[idx_at(conf("bilateral_takeoff"))])
out["com_drop_E6conf_to_E9occ"] = float(com_z[idx_at(occ("descending_landing"))] - com_z.max())
out["com_z_at_E9_occ"] = float(com_z[idx_at(occ("descending_landing"))])
out["com_z_at_E9_conf"] = float(com_z[idx_at(conf("descending_landing"))])
out["vz_at_E9_occ"] = float(cv[idx_at(occ("descending_landing")), 2])
out["E6_E9_flight_time"] = float(occ("descending_landing") - occ("bilateral_takeoff"))
out["E7_conf_E9_E9occ"] = float(occ("descending_landing") - conf("genuine_flight"))
out["E6_conf_to_E9_conf"] = float(conf("descending_landing") - conf("bilateral_takeoff"))

# physical flight: total normal force below 10 N both feet
both_off = (fzL < 10) & (fzR < 10)
# contiguous segment containing E6 occ
def seg_around(mask, i0):
    a = i0
    while a > 0 and mask[a - 1]:
        a -= 1
    b = i0
    while b < n - 1 and mask[b + 1]:
        b += 1
    return a, b
i6 = idx_at(occ("bilateral_takeoff"))
if both_off[i6]:
    a, b = seg_around(both_off, i6)
    out["true_support_off_start_t"] = float(T[a])
    out["true_support_off_end_t"] = float(T[b])
    out["true_flight_duration_fz"] = float(T[b] - T[a])
    out["true_flight_start_idx"] = int(a); out["true_flight_end_idx"] = int(b)
    out["true_flight_first_recontact_fzmax"] = float(max(fzL[b + 1] if b + 1 < n else 0, fzR[b + 1] if b + 1 < n else 0))
# geometric clearance: lowest corner > 0
clear_ok = (lowL > 0.0005) & (lowR > 0.0005)
i_apex = apex_i
if clear_ok[i_apex]:
    a2, b2 = seg_around(clear_ok, i_apex)
    out["true_geometric_clearance_flight_t"] = float(T[b2] - T[a2])
    out["clear_start_idx"] = int(a2); out["clear_end_idx"] = int(b2)
out["lowL_at_E6occ"] = float(lowL[i6]); out["lowR_at_E6occ"] = float(lowR[i6])
out["max_lowL_flight_E6_E9"] = float(lowL[i6:idx_at(occ("descending_landing")) + 1].max())
out["max_lowR_flight_E6_E9"] = float(lowR[i6:idx_at(occ("descending_landing")) + 1].max())
# over the true off segment
if both_off[i6]:
    seg = slice(a, b + 1)
    out["max_lowL_true_off"] = float(lowL[seg].max())
    out["max_lowR_true_off"] = float(lowR[seg].max())
    out["min_lowL_true_off"] = float(lowL[seg].min())
    out["min_lowR_true_off"] = float(lowR[seg].min())
# ballistic check on true flight
if both_off[i6]:
    vz_a = cv[a, 2]
    dt_f = T[b] - T[a]
    out["ballistic_vz_at_off_start"] = float(vz_a)
    out["ballistic_time_from_vz"] = float(2 * abs(vz_a) / 9.81)
    out["jump_height_from_vz"] = float(vz_a ** 2 / (2 * 9.81))
    out["jump_height_from_time"] = float(9.81 * dt_f ** 2 / 8)
    # apex within true flight
    seg2 = slice(a, b + 1)
    ia = a + int(np.argmax(com_z[seg2]))
    out["true_flight_apex_t"] = float(T[ia])
    out["true_flight_apex_rise"] = float(com_z[ia] - com_z[a])
    out["takeoff_to_apex_time"] = float(T[ia] - T[a])
    out["apex_to_touchdown_time"] = float(T[b] - T[ia])

# root / pelvis rise
out["pelvis_z_initial"] = float(pelvis_z[0])
out["pelvis_z_max_flight"] = float(pelvis_z[a:b + 1].max()) if both_off[i6] else None
out["pelvis_rise_flight"] = float(pelvis_z[a:b + 1].max() - pelvis_z[a]) if both_off[i6] else None

# landing metrics
i9 = idx_at(occ("descending_landing"))
out["landing_vz_E9occ"] = float(cv[i9, 2])
out["landing_vx_E9occ"] = float(cv[i9, 0])
out["landing_pitch_E9occ"] = float(pitch[i9])
out["landing_pitch_rate_E9occ"] = float(V[i9, vadr["root_ry"]])
out["landing_comx_E9occ"] = float(com[i9, 0])
out["landing_comx_initial"] = float(com[0, 0])

# knee angle trace at key events
kneeL = Q[:, qadr["left_knee"]]; kneeR = Q[:, qadr["right_knee"]]
hipL = Q[:, qadr["left_hip"]]; hipR = Q[:, qadr["right_hip"]]
ankL = Q[:, qadr["left_ankle"]]; ankR = Q[:, qadr["right_ankle"]]
lum = Q[:, qadr["lumbar"]]
out["knee_at_E2occ"] = float(kneeL[idx_at(occ("countermovement_onset"))])
out["knee_at_E3"] = float(kneeL[idx_at(occ("valid_countermovement"))])
out["knee_at_E4occ"] = float(kneeL[idx_at(occ("upward_reversal"))])
out["knee_at_E5occ"] = float(kneeL[idx_at(occ("vertical_propulsion"))])
out["knee_at_E6occ"] = float(kneeL[i6])
out["knee_max"] = float(kneeL.max()); out["knee_max_t"] = float(T[int(np.argmax(kneeL))])
out["knee_min"] = float(kneeL.min())
out["knee_at_apex"] = float(kneeL[i_apex])
out["knee_at_E9occ"] = float(kneeL[i9])
out["hip_at_E3"] = float(hipL[idx_at(occ("valid_countermovement"))])
out["hip_at_E6occ"] = float(hipL[i6])
out["ankle_at_E3"] = float(ankL[idx_at(occ("valid_countermovement"))])
out["ankle_at_E6occ"] = float(ankL[i6])
out["lumbar_at_E3"] = float(lum[idx_at(occ("valid_countermovement"))])
out["lumbar_at_E6occ"] = float(lum[i6])

# knee angle relative ankle-knee x offset uses FK; capture the x-offsets
kne_x = np.zeros(n); ank_x = np.zeros(n)
for i in [idx_at(occ("valid_countermovement")), i6, i_apex, i9]:
    d.qpos[:] = Q[i]; d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    kne_x[i] = d.xpos[bid["left_shank"], 0]
    ank_x[i] = d.xpos[bid["left_foot"], 0]
out["ankle_minus_knee_x_at_E3"] = float(ank_x[idx_at(occ("valid_countermovement"))] - kne_x[idx_at(occ("valid_countermovement"))])
out["ankle_minus_knee_x_at_E6"] = float(ank_x[i6] - kne_x[i6])
out["ankle_minus_knee_x_at_apex"] = float(ank_x[i_apex] - kne_x[i_apex])
out["ankle_minus_knee_x_at_E9"] = float(ank_x[i9] - kne_x[i9])

# support margin behaviour vs active feet (production formula recomputed)
margin = np.zeros(n)
for i in range(n):
    d.qpos[:] = Q[i]; d.qvel[:] = V[i]; mujoco.mj_forward(m, d)
    lc = d.geom_xpos[gid["left_foot_box"]]; rc = d.geom_xpos[gid["right_foot_box"]]
    x_min = min(lc[0] - 0.150, rc[0] - 0.150); x_max = max(lc[0] + 0.150, rc[0] + 0.150)
    y_min = min(lc[1] - 0.060, rc[1] - 0.060); y_max = max(lc[1] + 0.060, rc[1] + 0.060)
    margin[i] = min(com[i, 0] - x_min, x_max - com[i, 0], com[i, 1] - y_min, y_max - com[i, 1])
out["margin_min"] = float(margin.min()); out["margin_at_E6occ"] = float(margin[i6])
out["margin_at_E9occ"] = float(margin[i9])
out["margin_single_support_example_t"] = None
# find a sample where one foot inactive but margin still large through full-box hull
inactive_one = ((fzL < 10) ^ (fzR < 10))
if inactive_one.any():
    ii = int(np.argmax(inactive_one.astype(int)))
    out["margin_single_support_example"] = dict(t=float(T[ii]), margin=float(margin[ii]),
                                                fzL=float(fzL[ii]), fzR=float(fzR[ii]),
                                                lowL=float(lowL[ii]), lowR=float(lowR[ii]))
out["margin_from_inactive_foot_hull_example"] = bool(inactive_one.any())

# prohibited / fall check: no body shell can ever contact by construction
out["ncon_max"] = None

# max penetration
pen = np.maximum(0, -np.minimum(lowL, lowR))
out["max_penetration_recomputed"] = float(pen.max())
out["peak_fzL"] = float(fzL.max()); out["peak_fzR"] = float(fzR.max())
out["peak_total_Fz_BW"] = float((fzL + fzR).max() / (95 * 9.81))
out["min_total_Fz"] = float((fzL + fzR).min())

# impact: after first recontact
if both_off[i6]:
    rc = b
    win = slice(rc, min(n, rc + int(0.1 / 0.000125)))
    out["impact_peak_fz_100ms"] = float((fzL[win] + fzR[win]).max())
    out["impact_peak_fz_100ms_BW"] = float((fzL[win] + fzR[win]).max() / (95 * 9.81))

# COM horizontal excursion
out["com_x_min"] = float(com[:, 0].min()); out["com_x_max"] = float(com[:, 0].max())
out["com_x_at_E10occ"] = float(com[idx_at(occ("impact_absorption")), 0])
out["com_x_at_E11occ"] = float(com[idx_at(occ("balance_capture")), 0])
out["com_x_at_RR"] = None
out["com_x_final"] = float(com[-1, 0])
out["com_vx_max_abs"] = float(np.abs(cv[:, 0]).max())
out["com_vx_at_E9"] = float(cv[i9, 0])
out["com_vx_at_E10occ"] = float(cv[idx_at(occ("impact_absorption")), 0])
out["com_vx_at_E11occ"] = float(cv[idx_at(occ("balance_capture")), 0])
out["com_vx_final"] = float(cv[-1, 0])
out["com_vx_reversal_at_t"] = float(T[int(np.argmin(cv[:, 0]))])

with open("/tmp/opencode/res80/traj_summary.json", "w") as f:
    json.dump(out, f, indent=2, default=str)
print(json.dumps(out, indent=2, default=str))
