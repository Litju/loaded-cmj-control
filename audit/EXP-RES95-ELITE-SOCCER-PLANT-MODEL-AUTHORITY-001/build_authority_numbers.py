#!/usr/bin/env python3
"""RES-95 authority number builder.

Reads AUTHORITY_INPUTS.json and writes DERIVED_QUANTITIES.json deterministically.
All model inertia computations are analytic rigid-body sums (parallel-axis theorem).

Read-only with respect to every non-bundle repository path.
Run:  python3 build_authority_numbers.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
INPUTS = json.loads((HERE / "AUTHORITY_INPUTS.json").read_text())

H = INPUTS["reference_athlete"]["ATHLETE_STATURE_M"]
M_ATH = INPUTS["reference_athlete"]["ATHLETE_MASS_KG"]
M_BAR = INPUTS["reference_athlete"]["EXTERNAL_LOAD_MASS_KG"]
G = INPUTS["constants"]["g_m_s2"]
DL = INPUTS["de_leva_1996_male"]
S = H / DL["sample_stature_m"]
ROWS = DL["table4_male_rows"]


def L_of(key: str) -> float:
    return ROWS[key]["L_mm"] / 1000.0 * S


def m_of(key: str) -> float:
    return ROWS[key]["mass_frac"] * M_ATH


def tensor_about_point(m, p, u, rg, Lseg, com):
    """Full 3x3 inertia tensor of a segment about arbitrary point `com` (HAT frame)."""
    u = np.asarray(u, dtype=float)
    u = u / np.linalg.norm(u)
    rs, rt, rl = rg
    y = np.array([0.0, 1.0, 0.0])
    v = y - np.dot(y, u) * u
    if np.linalg.norm(v) < 1e-12:
        v = np.array([1.0, 0.0, 0.0])
    v = v / np.linalg.norm(v)
    w = np.cross(u, v)
    w = w / np.linalg.norm(w)
    I = (m * (rl * Lseg) ** 2 * np.outer(u, u)
         + m * (rs * Lseg) ** 2 * np.outer(v, v)
         + m * (rt * Lseg) ** 2 * np.outer(w, w))
    d = np.asarray(p, dtype=float) - np.asarray(com, dtype=float)
    return I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))


def build_hat(posture):
    z_lpt_top = L_of("lpt")
    z_mpt_top = z_lpt_top + L_of("mpt")
    z_upt_top = z_mpt_top + L_of("upt")

    head_pitch = math.radians(posture["head_pitch_deg"])
    head_dir = np.array([math.sin(head_pitch), 0.0, math.cos(head_pitch)])
    cerv = np.array([0.0, 0.0, z_upt_top])
    head_com = cerv + head_dir * (L_of("head") * (1.0 - ROWS["head"]["com_frac_from_origin"]))

    r_upt = np.array([0.0, 0.0, z_upt_top - L_of("upt") * ROWS["upt"]["com_frac_from_origin"]])
    r_mpt = np.array([0.0, 0.0, z_mpt_top - L_of("mpt") * ROWS["mpt"]["com_frac_from_origin"]])

    S_pt = np.array([posture["shoulder_joint_x_m"], posture["shoulder_joint_y_m"],
                     L_of("mid_shoulder")])
    W_pt = np.array([
        INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"] + 0.0,
        posture["hand_grip_half_width_y_m"],
        INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"] + posture["wrist_dz_m"]])
    a, b = L_of("upper_arm"), L_of("forearm")
    d = W_pt - S_pt
    D = np.linalg.norm(d)
    x0 = (a * a - b * b + D * D) / (2.0 * D)
    h = math.sqrt(max(a * a - x0 * x0, 0.0))
    u = d / D
    down = np.array([0.0, 0.0, -1.0])
    n = down - np.dot(down, u) * u
    n = n / np.linalg.norm(n)
    base = S_pt + x0 * u
    E_lo = base + h * n
    E_hi = base - h * n
    E = E_lo if E_lo[2] < E_hi[2] else E_hi

    r_ua = S_pt + ROWS["upper_arm"]["com_frac_from_origin"] * (E - S_pt)
    r_fa = E + ROWS["forearm"]["com_frac_from_origin"] * (W_pt - E)
    r_hand = W_pt + np.array([0.0, 1.0, 0.0]) * (L_of("hand") * ROWS["hand"]["com_frac_from_origin"])

    segs = [
        ("head", m_of("head"), head_com, head_dir, ("rg_sagittal", "rg_transverse", "rg_longitudinal"), L_of("head"), ROWS["head"]),
        ("upt", m_of("upt"), r_upt, np.array([0.0, 0.0, 1.0]), ("rg_sagittal", "rg_transverse", "rg_longitudinal"), L_of("upt"), ROWS["upt"]),
        ("mpt", m_of("mpt"), r_mpt, np.array([0.0, 0.0, 1.0]), ("rg_sagittal", "rg_transverse", "rg_longitudinal"), L_of("mpt"), ROWS["mpt"]),
    ]
    for side in (+1.0, -1.0):
        ys = np.array([1.0, side, 1.0])
        S2, E2, W2 = S_pt * ys, E * ys, W_pt * ys
        segs.append(("upper_arm", m_of("upper_arm"), r_ua * ys, E2 - S2,
                     ("rg_sagittal", "rg_transverse", "rg_longitudinal"), a, ROWS["upper_arm"]))
        segs.append(("forearm", m_of("forearm"), r_fa * ys, W2 - E2,
                     ("rg_sagittal", "rg_transverse", "rg_longitudinal"), b, ROWS["forearm"]))
        segs.append(("hand", m_of("hand"), r_hand * ys, np.array([0.0, side, 0.0]),
                     ("rg_sagittal", "rg_transverse", "rg_longitudinal"), L_of("hand"), ROWS["hand"]))

    M = sum(m for _, m, *_ in segs)
    com = sum(m * p for _, m, p, *_ in segs) / M
    I = np.zeros((3, 3))
    for _, m, p, uu, rg_keys, Ls, row in segs:
        rg = tuple(row[k] for k in rg_keys)
        I += tensor_about_point(m, p, uu, rg, Ls, com)
    return {
        "S": S_pt, "E": E, "W": W_pt, "mass_kg": M, "com_m": com, "inertia_about_com": I,
        "r_upper_arm": r_ua, "r_forearm": r_fa, "r_hand": r_hand,
        "head_com": head_com, "r_upt": r_upt, "r_mpt": r_mpt,
        "arm_interior_angle_deg": math.degrees(math.acos(
            float(np.clip(np.dot((E - S_pt) / a, (W_pt - E) / b), -1.0, 1.0)))),
    }


def bar_numbers():
    b = INPUTS["bar"]
    L_shaft = b["BAR_SHAFT_LENGTH_M"]
    d_g, d_s = b["BAR_GRIP_DIAMETER_M"], b["BAR_SLEEVE_DIAMETER_M"]
    L_s = b["BAR_SLEEVE_LENGTH_M"]
    V = math.pi / 4 * (d_g ** 2 * L_shaft + 2 * d_s ** 2 * L_s)
    rho = M_BAR / V
    m_sh = rho * math.pi / 4 * d_g ** 2 * L_shaft
    m_sl = rho * math.pi / 4 * d_s ** 2 * L_s
    I_ax = 0.5 * m_sh * (d_g / 2) ** 2 + 2 * 0.5 * m_sl * (d_s / 2) ** 2
    I_tr = (m_sh * (3 * (d_g / 2) ** 2 + L_shaft ** 2) / 12.0
            + 2.0 * (m_sl * (3 * (d_s / 2) ** 2 + L_s ** 2) / 12.0
                     + m_sl * ((L_shaft / 2.0) + (L_s / 2.0)) ** 2))
    return {"rho_eff_kg_m3": rho, "m_shaft_kg": m_sh, "m_sleeve_each_kg": m_sl,
            "volume_m3": V, "I_axis_kg_m2": I_ax, "I_transverse_kg_m2": I_tr,
            "com_in_bar_frame_m": [0.0, 0.0, 0.0]}


def rot_y(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def system_hat_bar(hat, bar):
    x_b = INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"]
    z_b = INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"]
    I_bar = np.diag([bar["I_transverse_kg_m2"], bar["I_axis_kg_m2"], bar["I_transverse_kg_m2"]])
    M = hat["mass_kg"] + M_BAR
    com = (hat["mass_kg"] * hat["com_m"] + M_BAR * np.array([x_b, 0.0, z_b])) / M
    d1 = hat["com_m"] - com
    d2 = np.array([x_b, 0.0, z_b]) - com
    I = (hat["inertia_about_com"] + hat["mass_kg"] * (np.dot(d1, d1) * np.eye(3) - np.outer(d1, d1))
         + I_bar + M_BAR * (np.dot(d2, d2) * np.eye(3) - np.outer(d2, d2)))
    return {"mass_kg": M, "com_m": com, "inertia_about_com": I}


def main():
    posture = {
        "head_pitch_deg": INPUTS["hat_posture"]["head_pitch_deg"],
        "shoulder_joint_x_m": INPUTS["hat_posture"]["shoulder_joint_x_m"],
        "shoulder_joint_y_m": INPUTS["hat_posture"]["shoulder_joint_y_m"],
        "wrist_dz_m": INPUTS["hat_posture"]["wrist_dz_m"],
        "hand_grip_half_width_y_m": INPUTS["hat_posture"]["hand_grip_half_width_y_m"],
    }
    hat = build_hat(posture)
    bar = bar_numbers()
    system = system_hat_bar(hat, bar)

    derived = {
        "schema_version": "1.0.0",
        "file_role": "derived_quantities",
        "authority_id": INPUTS["authority_id"],
        "generated_from": "AUTHORITY_INPUTS.json",
        "scale_factor_s": S,
        "segment_lengths_m": {
            "head": L_of("head"), "upt": L_of("upt"), "mpt": L_of("mpt"), "lpt": L_of("lpt"),
            "upper_arm": L_of("upper_arm"), "forearm": L_of("forearm"), "hand": L_of("hand"),
            "thigh": INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H,
            "shank": INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H,
            "whole_foot_deleva": L_of("whole_foot"),
            "mid_shoulder": L_of("mid_shoulder"),
            "thigh_deleva_scaled_crosscheck_m": L_of("thigh"),
            "shank_deleva_scaled_crosscheck_m": L_of("shank"),
            "thigh_crosscheck_delta_m": L_of("thigh") - INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H,
            "shank_crosscheck_delta_m": L_of("shank") - INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H,
            "normative_rule": "thigh and shank use the frozen stature ratios 0.2425H and 0.2493H as the normative lengths; the de Leva-scaled values are declared cross-checks only",
        },
        "segment_masses_kg": {
            "head": m_of("head"), "upt": m_of("upt"), "mpt": m_of("mpt"), "lpt_pelvis": m_of("lpt"),
            "upper_arm": m_of("upper_arm"), "forearm": m_of("forearm"), "hand": m_of("hand"),
            "thigh": m_of("thigh"), "shank": m_of("shank"), "whole_foot": m_of("whole_foot"),
        },
        "mass_closure_kg": {
            "hat": hat["mass_kg"],
            "pelvis": m_of("lpt"),
            "two_thighs": 2 * m_of("thigh"), "two_shanks": 2 * m_of("shank"),
            "two_feet": 2 * m_of("whole_foot"),
            "athlete_total": (hat["mass_kg"] + m_of("lpt") + 2 * m_of("thigh")
                              + 2 * m_of("shank") + 2 * m_of("whole_foot")),
            "bar": M_BAR,
            "system_total": (hat["mass_kg"] + m_of("lpt") + 2 * m_of("thigh") + 2 * m_of("shank")
                             + 2 * m_of("whole_foot") + M_BAR),
        },
        "system_weight_N": (M_ATH + M_BAR) * G,
        "athlete_weight_N": M_ATH * G,
        "load_to_athlete_mass_ratio": M_BAR / M_ATH,
        "hat": {
            "frame": INPUTS["hat_posture"]["frame"],
            "shoulder_joint_m": hat["S"].tolist(),
            "elbow_joint_m": hat["E"].tolist(),
            "wrist_joint_m": hat["W"].tolist(),
            "mass_kg": hat["mass_kg"],
            "com_m": hat["com_m"].tolist(),
            "inertia_about_com_kg_m2": hat["inertia_about_com"].tolist(),
            "principal_moments_kg_m2": sorted(np.linalg.eigvalsh(hat["inertia_about_com"]).tolist(), reverse=True),
            "principal_rotation_about_y_deg": math.degrees(0.5 * math.atan2(
                2.0 * float(hat["inertia_about_com"][0, 2]),
                float(hat["inertia_about_com"][0, 0] - hat["inertia_about_com"][2, 2]))),
            "elbow_flexion_deg": hat["arm_interior_angle_deg"],
        "elbow_interior_angle_deg": 180.0 - hat["arm_interior_angle_deg"],
            "head_com_m": hat["head_com"].tolist(),
            "upt_com_m": hat["r_upt"].tolist(),
            "mpt_com_m": hat["r_mpt"].tolist(),
            "upper_arm_com_m": hat["r_upper_arm"].tolist(),
            "forearm_com_m": hat["r_forearm"].tolist(),
            "hand_com_m": hat["r_hand"].tolist(),
            "trunk_chain_cerv_z_m": float(L_of("lpt") + L_of("mpt") + L_of("upt")),
            "vertex_z_m": float(L_of("lpt") + L_of("mpt") + L_of("upt") + L_of("head")),
        },
        "bar": {
            **bar,
            "mass_kg": M_BAR,
            "length_m": INPUTS["bar"]["BAR_LENGTH_M"],
            "com_in_hat_frame_m": [INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"], 0.0,
                                   INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"]],
            "sleeve_span_m": [INPUTS["bar"]["BAR_GRIP_SECTION_LENGTH_M"] / 2.0 + 0.03,
                              INPUTS["bar"]["BAR_LENGTH_M"] / 2.0],
        },
        "hat_bar_system": {
            "mass_kg": system["mass_kg"],
            "com_m": system["com_m"].tolist(),
            "inertia_about_com_kg_m2": system["inertia_about_com"].tolist(),
        },
        "pelvis": {
            "mass_kg": m_of("lpt"),
            "com_z_above_midh_m": float(L_of("lpt") * (1.0 - ROWS["lpt"]["com_frac_from_origin"])),
            "inertia_about_com_kg_m2": tensor_about_point(
                m_of("lpt"), np.array([0, 0, L_of("lpt") * (1 - ROWS["lpt"]["com_frac_from_origin"])]),
                np.array([0, 0, 1.0]),
                (ROWS["lpt"]["rg_sagittal"], ROWS["lpt"]["rg_transverse"], ROWS["lpt"]["rg_longitudinal"]),
                L_of("lpt"),
                np.array([0, 0, L_of("lpt") * (1 - ROWS["lpt"]["com_frac_from_origin"])])).tolist(),
        },
        "thigh": {
            "mass_kg": m_of("thigh"),
            "length_m": INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H,
            "com_below_hjc_m": float(INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H * ROWS["thigh"]["com_frac_from_origin"]),
            "inertia_about_com_kg_m2": tensor_about_point(
                m_of("thigh"), np.array([0, 0, -INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H * ROWS["thigh"]["com_frac_from_origin"]]),
                np.array([0, 0, 1.0]),
                (ROWS["thigh"]["rg_sagittal"], ROWS["thigh"]["rg_transverse"], ROWS["thigh"]["rg_longitudinal"]),
                INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H,
                np.array([0, 0, -INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H * ROWS["thigh"]["com_frac_from_origin"]])).tolist(),
        },
        "shank": {
            "mass_kg": m_of("shank"),
            "length_m": INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H,
            "com_below_kjc_m": float(INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H * ROWS["shank"]["com_frac_from_origin"]),
            "inertia_about_com_kg_m2": tensor_about_point(
                m_of("shank"), np.array([0, 0, -INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H * ROWS["shank"]["com_frac_from_origin"]]),
                np.array([0, 0, 1.0]),
                (ROWS["shank"]["rg_sagittal"], ROWS["shank"]["rg_transverse"], ROWS["shank"]["rg_longitudinal"]),
                INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H,
                np.array([0, 0, -INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H * ROWS["shank"]["com_frac_from_origin"]])).tolist(),
        },
        "whole_foot_deleva_reference": {
            "mass_kg": m_of("whole_foot"),
            "length_frozen_m": INPUTS["frozen_geometry"]["FOOT_LENGTH_M"],
            "com_from_heel_m": INPUTS["frozen_geometry"]["FOOT_LENGTH_M"] * ROWS["whole_foot"]["com_frac_from_origin"],
            "inertia_about_sagittal_axis_through_com_kg_m2": m_of("whole_foot") * (
                ROWS["whole_foot"]["rg_sagittal"] * INPUTS["frozen_geometry"]["FOOT_LENGTH_M"]) ** 2,
        },
        "foot": foot_numbers(),
        "segment_chain_closure": segment_chain_closure(),
        "hat_sensitivity": hat_sensitivity(posture, hat, bar),
        "bar_uniform_rod_surrogate": uniform_rod_surrogate(),
    }

    out = HERE / "DERIVED_QUANTITIES.json"
    out.write_text(json.dumps(derived, indent=2, sort_keys=False) + "\n")
    print("wrote", out.name)
    print(json.dumps({
        "hat_mass_kg": hat["mass_kg"],
        "hat_com_m": hat["com_m"].tolist(),
        "hat_inertia": hat["inertia_about_com"].round(9).tolist(),
        "system_mass_kg": system["mass_kg"],
        "system_com_m": system["com_m"].tolist(),
        "system_inertia": system["inertia_about_com"].round(9).tolist(),
        "closure_delta_m": derived["segment_chain_closure"]["delta_m"],
    }, indent=2))


def foot_numbers():
    fg = INPUTS["frozen_geometry"]
    fs = INPUTS["foot_segments"]
    m_foot = m_of("whole_foot")
    L = fg["FOOT_LENGTH_M"]
    h_ankle = fg["ANKLE_JOINT_HEIGHT_RATIO_OF_STATURE"] * H
    x_ankle = fg["ANKLE_X_FROM_HEEL_M"]
    x_mid = fg["MIDTARSAL_X_FROM_HEEL_M"]
    x_mtp = fg["MTP_X_FROM_HEEL_M"]
    x_toe_tip = L

    hind_L = x_mid - 0.0
    fore_L = x_mtp - x_mid
    toe_L = x_toe_tip - x_mtp
    m_h = fs["relative_mass"]["hindfoot"] * m_foot
    m_f = fs["relative_mass"]["forefoot"] * m_foot
    m_t = fs["relative_mass"]["phalanx"] * m_foot
    # segment proximal joint centers in ankle frame
    mid_pt = np.array([x_mid - x_ankle, 0.0, -h_ankle + 0.021565])
    mtp_pt = np.array([x_mtp - x_ankle, 0.0, -h_ankle + 0.031565])
    tip_pt = np.array([x_toe_tip - x_ankle, 0.0, -h_ankle + 0.031565])
    ankle_pt = np.array([0.0, 0.0, 0.0])
    com_h = ankle_pt + fs["relative_com_from_segment_proximal_joint_center"]["hindfoot"] * (mid_pt - ankle_pt)
    com_f = mid_pt + fs["relative_com_from_segment_proximal_joint_center"]["forefoot"] * (mtp_pt - mid_pt)
    com_t = mtp_pt + fs["relative_com_from_segment_proximal_joint_center"]["phalanx"] * (tip_pt - mtp_pt)

    def foot_I(mm, key, com):
        rel = fs["relative_inertia_tensor_around_com"][key]
        scale = mm ** (5.0 / 3.0)
        return [rel["Ixx"] * scale, rel["Iyy"] * scale, rel["Izz"] * scale]

    I_h, I_f, I_t = foot_I(m_h, "hindfoot", com_h), foot_I(m_f, "forefoot", com_f), foot_I(m_t, "phalanx", com_t)
    com_all = (m_h * com_h + m_f * com_f + m_t * com_t) / (m_h + m_f + m_t)
    return {
        "segment_masses_kg": {"hindfoot": m_h, "forefoot": m_f, "phalanx": m_t},
        "segment_lengths_m": {"hindfoot": hind_L, "forefoot": fore_L, "phalanx": toe_L},
        "ankle_joint_height_m": h_ankle,
        "ankle_x_from_heel_m": x_ankle,
        "mid_tarsal_x_from_heel_m": x_mid,
        "mtp_x_from_heel_m": x_mtp,
        "toe_tip_x_from_heel_m": x_toe_tip,
        "segment_com_ankle_frame_m": {"hindfoot": com_h.tolist(), "forefoot": com_f.tolist(),
                                      "phalanx": com_t.tolist()},
        "assembled_foot_com_ankle_frame_m": com_all.tolist(),
        "assembled_foot_com_x_from_heel_m": float(com_all[0] + x_ankle),
        "deleva_foot_com_x_from_heel_m": float(fg["FOOT_LENGTH_M"] * ROWS["whole_foot"]["com_frac_from_origin"]),
        "com_x_delta_vs_deleva_m": float(com_all[0] + x_ankle - fg["FOOT_LENGTH_M"] * ROWS["whole_foot"]["com_frac_from_origin"]),
        "segment_inertia_about_com_kg_m2": {"hindfoot": I_h, "forefoot": I_f, "phalanx": I_t},
        "contact_regions": fg["contact_regions"],
        "sole_plane_z_m": float(-h_ankle),
        "no_subsole_geometry_rule": "no collision geom may extend below the sole plane, and all three contact patches are planar on the sole plane",
    }


def segment_chain_closure():
    leg = (INPUTS["frozen_geometry"]["ANKLE_JOINT_HEIGHT_RATIO_OF_STATURE"] * H
           + INPUTS["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H
           + INPUTS["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H)
    torso_head = L_of("lpt") + L_of("mpt") + L_of("upt") + L_of("head")
    return {"hip_height_m": float(leg), "midh_to_vertex_m": float(torso_head),
            "sum_m": float(leg + torso_head), "stature_m": H,
            "delta_m": float(leg + torso_head - H),
            "delta_fraction_of_stature": float((leg + torso_head - H) / H)}


def hat_sensitivity(posture, hat_nominal, bar):
    cases = []
    def add(name, **kw):
        p = dict(posture)
        p.update(kw)
        h = build_hat(p)
        s = system_hat_bar(h, bar)
        cases.append({"case": name, "hat_com_m": h["com_m"].tolist(),
                      "hat_inertia_kg_m2": h["inertia_about_com"].tolist(),
                      "system_com_m": s["com_m"].tolist(),
                      "system_inertia_kg_m2": s["inertia_about_com"].tolist()})
        return h
    dx = INPUTS["bar"]["placement_sensitivity_m"][0]
    dz = INPUTS["bar"]["placement_sensitivity_m"][1]
    orig_x = INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"]
    orig_z = INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"]
    try:
        for sx in (-dx, dx):
            for sz in (-dz, dz):
                INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"] = orig_x + sx
                INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"] = orig_z + sz
                add(f"bar_placement_dx_{sx:+.3f}_dz_{sz:+.3f}")
    finally:
        INPUTS["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"] = orig_x
        INPUTS["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"] = orig_z
    for hp in INPUTS["hat_posture"]["head_pitch_sensitivity_deg"]:
        add(f"head_pitch_{hp:+.0f}deg", head_pitch_deg=hp)
    for sx in INPUTS["hat_posture"]["shoulder_joint_x_sensitivity_m"]:
        add(f"shoulder_x_{sx:+.3f}", shoulder_joint_x_m=sx)
    for sy in INPUTS["hat_posture"]["shoulder_joint_y_sensitivity_m"]:
        add(f"shoulder_y_{sy:+.3f}", shoulder_joint_y_m=sy)
    for wz in INPUTS["hat_posture"]["wrist_dz_sensitivity_m"]:
        add(f"wrist_dz_{wz:+.3f}", wrist_dz_m=wz)
    for gy in INPUTS["hat_posture"]["hand_grip_half_width_y_sensitivity_m"]:
        add(f"grip_y_{gy:+.3f}", hand_grip_half_width_y_m=gy)
    iyy = [c["system_inertia_kg_m2"][1][1] for c in cases]
    return {"cases": cases, "system_iyy_min": min(iyy), "system_iyy_max": max(iyy),
            "declared_perturbations": {
                "bar_placement_m": [dx, dz],
                "head_pitch_deg": INPUTS["hat_posture"]["head_pitch_sensitivity_deg"],
                "shoulder_joint_x_m": INPUTS["hat_posture"]["shoulder_joint_x_sensitivity_m"],
                "shoulder_joint_y_m": INPUTS["hat_posture"]["shoulder_joint_y_sensitivity_m"],
                "wrist_dz_m": INPUTS["hat_posture"]["wrist_dz_sensitivity_m"],
                "hand_grip_half_width_y_m": INPUTS["hat_posture"]["hand_grip_half_width_y_sensitivity_m"]}}


def uniform_rod_surrogate():
    L, d = INPUTS["bar"]["BAR_LENGTH_M"], INPUTS["bar"]["BAR_GRIP_DIAMETER_M"]
    r = d / 2
    return {"note": "NON-IWF surrogate retained only as a declared sensitivity case",
            "I_transverse_kg_m2": M_BAR * (3 * r * r + L * L) / 12.0,
            "I_axis_kg_m2": 0.5 * M_BAR * r * r}


if __name__ == "__main__":
    main()
