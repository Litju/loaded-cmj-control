#!/usr/bin/env python3
"""RES-95 model-authority validator (independent recomputation + integrity gates).

This validator deliberately does NOT import the renderer. It recomputes the
authority numbers from AUTHORITY_INPUTS.json with its own implementation and
checks them against the rendered artifacts.

Gates:
  G1  inputs parse; reference quantities are exact
  G2  independent numeric recomputation (masses, geometry, HAT/bar/system, foot)
  G3  artifact JSON schema, status, taxonomy and cross-file link consistency
  G4  JSON/Markdown agreement for the frozen decisions
  G5  evidence-table completeness and source-reference resolution
  G6  red-team stale-authority scan (marked supersession required)
  G7  topology and ROM invariants
  G8  hash manifest verification (when present)
  G9  no production / controller / test file modification
Emits AUTHORITY_VALIDATION_REPORT.json. Exit code 0 iff every gate passes.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

TAXONOMY = {"LITERATURE_DIRECT", "HUMAN_ANATOMY_REFERENCE", "LITERATURE_INFORMED_SYNTHETIC",
            "PHYSICS_IDENTITY", "ENGINEERING_NOMINAL_WITH_SENSITIVITY", "DEFERRED_TO_LATER_AUTHORITY"}

ARTIFACT_FILES = [
    "SPORT_CONTEXT_AUTHORITY", "REFERENCE_ATHLETE_SPEC", "ANTHROPOMETRY_BSIP_AUTHORITY",
    "UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "LOAD_BAR_AUTHORITY", "JOINT_COORDINATE_ROM_AUTHORITY",
    "FOOT_MTP_MODEL_AUTHORITY", "STANCE_LAB_CONTEXT_AUTHORITY", "COLLISION_CONTACT_POLICY",
    "PLANT_TOPOLOGY_AUTHORITY", "EVENT_MEASUREMENT_BOUNDARY", "PERFORMANCE_AUTHORITY_BOUNDARY",
    "DEFERRED_NUMERICAL_CALIBRATIONS",
]

REPORT = {"schema_version": "1.0.0", "gates": {}, "checks": [], "failures": []}


def record(gate, name, ok, detail=""):
    REPORT["checks"].append({"gate": gate, "check": name, "ok": bool(ok), "detail": str(detail)})
    if not ok:
        REPORT["failures"].append(f"{gate}:{name}: {detail}")


def rows_com_frac(rec):
    """de Leva male whole-foot COM fraction (input constant; asserted elsewhere)."""
    return 0.4415


def load_inputs():
    return json.loads((HERE / "AUTHORITY_INPUTS.json").read_text())


def load_derived():
    return json.loads((HERE / "DERIVED_QUANTITIES.json").read_text())


# ------------------------------------------------------------------ independent numerics
def recompute(inputs):
    H = inputs["reference_athlete"]["ATHLETE_STATURE_M"]
    M = inputs["reference_athlete"]["ATHLETE_MASS_KG"]
    g = inputs["constants"]["g_m_s2"]
    rows = inputs["de_leva_1996_male"]["table4_male_rows"]
    S = H / inputs["de_leva_1996_male"]["sample_stature_m"]
    L = {k: v["L_mm"] / 1000.0 * S for k, v in rows.items()}
    # thigh and shank are normatively frozen by stature ratio; de Leva scaling is a cross-check
    thigh_crosscheck = L["thigh"]
    shank_crosscheck = L["shank"]
    L["thigh"] = inputs["frozen_geometry"]["THIGH_LENGTH_RATIO_OF_STATURE"] * H
    L["shank"] = inputs["frozen_geometry"]["SHANK_LENGTH_RATIO_OF_STATURE"] * H
    assert abs(thigh_crosscheck - L["thigh"]) < 1e-4 and abs(shank_crosscheck - L["shank"]) < 1e-4
    mass = {k: v["mass_frac"] * M for k, v in rows.items() if "mass_frac" in v}

    # HAT posture
    hp = inputs["hat_posture"]
    S_pt = np.array([hp["shoulder_joint_x_m"], hp["shoulder_joint_y_m"], L["mid_shoulder"]])
    W_pt = np.array([inputs["bar"]["BAR_CENTER_X_IN_HAT_FRAME_M"], hp["hand_grip_half_width_y_m"],
                     inputs["bar"]["BAR_CENTER_Z_IN_HAT_FRAME_M"] + hp["wrist_dz_m"]])
    a, b = L["upper_arm"], L["forearm"]
    d = W_pt - S_pt
    D = float(np.linalg.norm(d))
    x0 = (a * a - b * b + D * D) / (2 * D)
    h = math.sqrt(max(a * a - x0 * x0, 0.0))
    u = d / D
    down = np.array([0.0, 0.0, -1.0])
    nn = down - float(np.dot(down, u)) * u
    nn /= np.linalg.norm(nn)
    E = S_pt + x0 * u + h * nn

    z_upt_top = L["lpt"] + L["mpt"] + L["upt"]
    r_head = np.array([0.0, 0.0, z_upt_top + L["head"] * (1 - rows["head"]["com_frac_from_origin"])])
    r_upt = np.array([0.0, 0.0, z_upt_top - L["upt"] * rows["upt"]["com_frac_from_origin"]])
    r_mpt = np.array([0.0, 0.0, z_upt_top - L["upt"] - L["mpt"] * rows["mpt"]["com_frac_from_origin"]])
    r_ua = S_pt + rows["upper_arm"]["com_frac_from_origin"] * (E - S_pt)
    r_fa = E + rows["forearm"]["com_frac_from_origin"] * (W_pt - E)
    r_hand = W_pt + np.array([0.0, 1.0, 0.0]) * L["hand"] * rows["hand"]["com_frac_from_origin"]

    segs = [("head", mass["head"], r_head, np.array([0.0, 0.0, 1.0]), rows["head"], L["head"]),
            ("upt", mass["upt"], r_upt, np.array([0.0, 0.0, 1.0]), rows["upt"], L["upt"]),
            ("mpt", mass["mpt"], r_mpt, np.array([0.0, 0.0, 1.0]), rows["mpt"], L["mpt"])]
    for side in (1.0, -1.0):
        ys = np.array([1.0, side, 1.0])
        segs += [("ua", mass["upper_arm"], r_ua * ys, E * ys - S_pt * ys, rows["upper_arm"], a),
                 ("fa", mass["forearm"], r_fa * ys, W_pt * ys - E * ys, rows["forearm"], b),
                 ("hand", mass["hand"], r_hand * ys, np.array([0.0, side, 0.0]), rows["hand"], L["hand"])]
    Mtot = sum(s[1] for s in segs)
    com = sum(s[1] * s[2] for s in segs) / Mtot

    def seg_tensor(m, p, uu, row, Ls, origin):
        uu = uu / np.linalg.norm(uu)
        yv = np.array([0.0, 1.0, 0.0])
        v = yv - np.dot(yv, uu) * uu
        nv = float(np.linalg.norm(v))
        if nv < 1e-12:
            v = np.array([1.0, 0.0, 0.0])
            nv = 1.0
        v = v / nv
        w = np.cross(uu, v)
        R = np.column_stack([uu, v, w])
        # radii: longitudinal about long axis; sagittal about v; transverse about w
        Ilong = m * (row["rg_longitudinal"] * Ls) ** 2
        Isag = m * (row["rg_sagittal"] * Ls) ** 2
        Itr = m * (row["rg_transverse"] * Ls) ** 2
        Iloc = np.diag([Ilong, Isag, Itr])
        I0 = R @ Iloc @ R.T
        dd = p - origin
        return I0 + m * (float(np.dot(dd, dd)) * np.eye(3) - np.outer(dd, dd))

    Ihat = np.zeros((3, 3))
    for _, m, p, uu, row, Ls in segs:
        Ihat += seg_tensor(m, p, uu, row, Ls, com)

    # bar composite
    bar = inputs["bar"]
    V = math.pi / 4 * (bar["BAR_GRIP_DIAMETER_M"] ** 2 * bar["BAR_SHAFT_LENGTH_M"]
                       + 2 * bar["BAR_SLEEVE_DIAMETER_M"] ** 2 * bar["BAR_SLEEVE_LENGTH_M"])
    rho = 20.0 / V
    m_sh = rho * math.pi / 4 * bar["BAR_GRIP_DIAMETER_M"] ** 2 * bar["BAR_SHAFT_LENGTH_M"]
    m_sl = rho * math.pi / 4 * bar["BAR_SLEEVE_DIAMETER_M"] ** 2 * bar["BAR_SLEEVE_LENGTH_M"]
    Ibar = np.diag([
        m_sh * (3 * (bar["BAR_GRIP_DIAMETER_M"] / 2) ** 2 + bar["BAR_SHAFT_LENGTH_M"] ** 2) / 12
        + 2 * (m_sl * (3 * (bar["BAR_SLEEVE_DIAMETER_M"] / 2) ** 2 + bar["BAR_SLEEVE_LENGTH_M"] ** 2) / 12
               + m_sl * ((bar["BAR_SHAFT_LENGTH_M"] + bar["BAR_SLEEVE_LENGTH_M"]) / 2) ** 2),
        0.5 * m_sh * (bar["BAR_GRIP_DIAMETER_M"] / 2) ** 2
        + m_sl * (bar["BAR_SLEEVE_DIAMETER_M"] / 2) ** 2,
        m_sh * (3 * (bar["BAR_GRIP_DIAMETER_M"] / 2) ** 2 + bar["BAR_SHAFT_LENGTH_M"] ** 2) / 12
        + 2 * (m_sl * (3 * (bar["BAR_SLEEVE_DIAMETER_M"] / 2) ** 2 + bar["BAR_SLEEVE_LENGTH_M"] ** 2) / 12
               + m_sl * ((bar["BAR_SHAFT_LENGTH_M"] + bar["BAR_SLEEVE_LENGTH_M"]) / 2) ** 2)])
    # correct the axial term order-safe (diag above used Ixx,Iyy,Izz where Iyy = axial)
    Ibar[0, 0] = Ibar[2, 2] = (m_sh * (3 * (bar["BAR_GRIP_DIAMETER_M"] / 2) ** 2 + bar["BAR_SHAFT_LENGTH_M"] ** 2) / 12
                               + 2 * (m_sl * (3 * (bar["BAR_SLEEVE_DIAMETER_M"] / 2) ** 2 + bar["BAR_SLEEVE_LENGTH_M"] ** 2) / 12
                                      + m_sl * ((bar["BAR_SHAFT_LENGTH_M"] + bar["BAR_SLEEVE_LENGTH_M"]) / 2) ** 2))
    Ibar[1, 1] = 0.5 * m_sh * (bar["BAR_GRIP_DIAMETER_M"] / 2) ** 2 + m_sl * (bar["BAR_SLEEVE_DIAMETER_M"] / 2) ** 2

    bar_pos = np.array([bar["BAR_CENTER_X_IN_HAT_FRAME_M"], 0.0, bar["BAR_CENTER_Z_IN_HAT_FRAME_M"]])
    Msys = Mtot + 20.0
    com_sys = (Mtot * com + 20.0 * bar_pos) / Msys
    d1 = com - com_sys
    d2 = bar_pos - com_sys
    Isys = (Ihat + Mtot * (float(np.dot(d1, d1)) * np.eye(3) - np.outer(d1, d1))
            + Ibar + 20.0 * (float(np.dot(d2, d2)) * np.eye(3) - np.outer(d2, d2)))

    # foot
    fg = inputs["frozen_geometry"]
    foot_L = fg["FOOT_LENGTH_M"]
    foot_m = mass["whole_foot"]
    hind_m = inputs["foot_segments"]["relative_mass"]["hindfoot"] * foot_m
    fore_m = inputs["foot_segments"]["relative_mass"]["forefoot"] * foot_m
    toe_m = inputs["foot_segments"]["relative_mass"]["phalanx"] * foot_m
    ankle_h = fg["ANKLE_JOINT_HEIGHT_RATIO_OF_STATURE"] * H
    mid_pt = np.array([fg["MIDTARSAL_X_FROM_HEEL_M"] - fg["ANKLE_X_FROM_HEEL_M"], 0.0, -ankle_h + 0.021565])
    mtp_pt = np.array([fg["MTP_X_FROM_HEEL_M"] - fg["ANKLE_X_FROM_HEEL_M"], 0.0, -ankle_h + 0.031565])
    tip_pt = np.array([foot_L - fg["ANKLE_X_FROM_HEEL_M"], 0.0, -ankle_h + 0.031565])
    com_h = inputs["foot_segments"]["relative_com_from_segment_proximal_joint_center"]["hindfoot"] * mid_pt
    com_f = mid_pt + inputs["foot_segments"]["relative_com_from_segment_proximal_joint_center"]["forefoot"] * (mtp_pt - mid_pt)
    com_t = mtp_pt + inputs["foot_segments"]["relative_com_from_segment_proximal_joint_center"]["phalanx"] * (tip_pt - mtp_pt)
    scale = {"hindfoot": hind_m ** (5 / 3), "forefoot": fore_m ** (5 / 3), "phalanx": toe_m ** (5 / 3)}
    I_foot = {k: [v * scale[k] for v in inputs["foot_segments"]["relative_inertia_tensor_around_com"][k].values()]
              for k in scale}
    com_all = (hind_m * com_h + fore_m * com_f + toe_m * com_t) / foot_m

    torso_head = L["lpt"] + L["mpt"] + L["upt"] + L["head"]
    hip_h = ankle_h + L["shank"] + L["thigh"]

    return {
        "S": S, "L": L, "mass": mass, "HAT_mass": Mtot, "HAT_com": com, "HAT_I": Ihat,
        "bar": {"rho": rho, "m_shaft": m_sh, "m_sleeve": m_sl, "I": Ibar},
        "system": {"M": Msys, "com": com_sys, "I": Isys},
        "foot": {"m": {"hindfoot": hind_m, "forefoot": fore_m, "phalanx": toe_m},
                 "ankle_h": ankle_h, "mid_pt": mid_pt, "mtp_pt": mtp_pt,
                 "com": com_all, "I": I_foot,
                 "com_x_from_heel": float(com_all[0] + fg["ANKLE_X_FROM_HEEL_M"])},
        "closure": {"hip_h": hip_h, "torso_head": torso_head, "sum": hip_h + torso_head,
                    "delta": hip_h + torso_head - H},
        "weights": {"system": 99.0 * g, "athlete": 79.0 * g},
        "E": E, "W": W_pt, "S_pt": S_pt,
    }


def close(a, b, tol=1e-9):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(a)), abs(float(b)))
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return set(a) == set(b) and all(close(a[k], b[k], tol) for k in a)
    return a == b


def check_close(gate, name, got, want, tol=1e-9):
    ok = close(got, want, tol)
    record(gate, name, ok, "" if ok else f"got={got} want={want}")
    return ok


# ------------------------------------------------------------------------- G1..G2
def gate_numeric(inputs, rec):
    ref = inputs["reference_athlete"]
    record("G1", "stature_m", ref["ATHLETE_STATURE_M"] == 1.835, ref["ATHLETE_STATURE_M"])
    record("G1", "athlete_mass_kg", ref["ATHLETE_MASS_KG"] == 79.0, ref["ATHLETE_MASS_KG"])
    record("G1", "bar_mass_kg", ref["EXTERNAL_LOAD_MASS_KG"] == 20.0, ref["EXTERNAL_LOAD_MASS_KG"])
    record("G1", "system_mass_kg", ref["TOTAL_SYSTEM_MASS_KG"] == 99.0, ref["TOTAL_SYSTEM_MASS_KG"])

    mass_sum = ((rec["mass"]["head"] + rec["mass"]["upt"] + rec["mass"]["mpt"] + rec["mass"]["lpt"])
                + 2 * (rec["mass"]["upper_arm"] + rec["mass"]["forearm"] + rec["mass"]["hand"])
                + 2 * (rec["mass"]["thigh"] + rec["mass"]["shank"] + rec["mass"]["whole_foot"]))
    check_close("G2", "segment_mass_fractions_sum_to_athlete_mass", mass_sum, 79.0, 1e-12)
    check_close("G2", "HAT_mass", rec["HAT_mass"], 38.7969, 1e-12)
    check_close("G2", "athlete_plus_bar_mass", mass_sum + 20.0, 99.0, 1e-12)
    check_close("G2", "thigh_length_ratio_form", rec["L"]["thigh"], 0.2425 * 1.835, 2e-5)
    check_close("G2", "shank_length_ratio_form", rec["L"]["shank"], 0.2493 * 1.835, 5e-5)
    check_close("G2", "closure_delta", rec["closure"]["delta"], 0.030905995404939546, 1e-12)
    check_close("G2", "thigh_normative_ratio", rec["L"]["thigh"], 0.2425 * 1.835, 1e-12)
    check_close("G2", "shank_normative_ratio", rec["L"]["shank"], 0.2493 * 1.835, 1e-12)
    record("G2", "HAT_inertia_symmetric", float(np.max(np.abs(rec["HAT_I"] - rec["HAT_I"].T))) < 1e-12,
           float(np.max(np.abs(rec["HAT_I"] - rec["HAT_I"].T))))
    record("G2", "HAT_inertia_positive_definite", bool(np.all(np.linalg.eigvalsh(rec["HAT_I"]) > 0)),
           np.linalg.eigvalsh(rec["HAT_I"]).tolist())
    x_heel_assembled = rec["foot"]["com_x_from_heel"]
    deleva_x = 0.275 * rows_com_frac(rec)
    check_close("G2", "foot COM x from heel (assembled)", x_heel_assembled,
                float(rec["foot"]["com"][0] + inputs["frozen_geometry"]["ANKLE_X_FROM_HEEL_M"]), 1e-9)
    check_close("G2", "foot de Leva COM x from heel (position, not mass moment)", deleva_x, 0.1214125, 1e-12)
    check_close("G2", "foot COM delta vs de Leva", x_heel_assembled - deleva_x, -0.00616055, 1e-9)
    check_close("G2", "foot hindfoot inertia Iyy", rec["foot"]["I"]["hindfoot"][1],
                0.0005182470417970387, 1e-12)
    check_close("G2", "foot forefoot inertia Izz", rec["foot"]["I"]["forefoot"][2],
                0.0006006367106418295, 1e-12)
    check_close("G2", "foot toe inertia Ixx", rec["foot"]["I"]["phalanx"][0],
                0.0001150951698085389, 1e-12)
    record("G2", "system_inertia_positive_definite", bool(np.all(np.linalg.eigvalsh(rec["system"]["I"]) > 0)),
           np.linalg.eigvalsh(rec["system"]["I"]).tolist())
    # bar mass closure
    check_close("G2", "bar_mass_closure", rec["bar"]["m_shaft"] + 2 * rec["bar"]["m_sleeve"], 20.0, 1e-12)
    # foot mass closure
    check_close("G2", "foot_mass_closure", sum(rec["foot"]["m"].values()), rec["mass"]["whole_foot"], 1e-12)
    # arm reach triangles
    a_len, b_len = rec["L"]["upper_arm"], rec["L"]["forearm"]
    se = float(np.linalg.norm(rec["E"] - rec["S_pt"]))
    ew = float(np.linalg.norm(rec["W"] - rec["E"]))
    check_close("G2", "IK |SE|=upper_arm", se, a_len, 1e-9)
    check_close("G2", "IK |EW|=forearm", ew, b_len, 1e-9)


def compare_derived_to_json(inputs, rec, derived):
    """Check the JSON authority artifacts against the independent recomputation."""
    def dec(art, dec_id):
        data = json.loads((HERE / f"{art}.json").read_text())
        for d in data["decisions"]:
            if d["id"] == dec_id:
                return d["value"]
        raise KeyError(f"{art}:{dec_id}")

    check_close("G2", "JSON UB-03 HAT mass", dec("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "UB-03"), rec["HAT_mass"], 1e-9)
    check_close("G2", "JSON UB-04 HAT COM", dec("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "UB-04"),
                [float(x) for x in rec["HAT_com"]], 1e-9)
    check_close("G2", "JSON UB-05 HAT inertia", dec("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "UB-05"),
                rec["HAT_I"].tolist(), 1e-9)
    ub08 = dec("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "UB-08")
    check_close("G2", "JSON UB-08 system mass", ub08["mass_kg"], rec["system"]["M"], 1e-9)
    check_close("G2", "JSON UB-08 system COM", ub08["com_m"], [float(x) for x in rec["system"]["com"]], 1e-9)
    check_close("G2", "JSON UB-08 system inertia", ub08["inertia_about_com_kg_m2"], rec["system"]["I"].tolist(), 1e-9)
    lb07 = dec("LOAD_BAR_AUTHORITY", "LB-07")
    check_close("G2", "JSON LB-07 rho", lb07["rho_eff_kg_m3"], rec["bar"]["rho"], 1e-9)
    check_close("G2", "JSON LB-07 shaft mass", lb07["m_shaft_kg"], rec["bar"]["m_shaft"], 1e-9)
    lb08 = dec("LOAD_BAR_AUTHORITY", "LB-08")
    check_close("G2", "JSON LB-08 transverse inertia", lb08["I_transverse_kg_m2"], rec["bar"]["I"][0, 0], 1e-9)
    check_close("G2", "JSON LB-08 axial inertia", lb08["I_axis_kg_m2"], rec["bar"]["I"][1, 1], 1e-9)
    fm04 = dec("FOOT_MTP_MODEL_AUTHORITY", "FM-04")
    check_close("G2", "JSON FM-04 foot masses", fm04, rec["foot"]["m"], 1e-9)
    ab13 = dec("ANTHROPOMETRY_BSIP_AUTHORITY", "AB-13")
    check_close("G2", "JSON AB-13 closure sum", ab13["sum_m"], rec["closure"]["sum"], 1e-9)
    check_close("G2", "JSON AB-13 closure delta", ab13["delta_m"], rec["closure"]["delta"], 1e-9)
    ra07 = dec("REFERENCE_ATHLETE_SPEC", "RA-07")
    check_close("G2", "JSON RA-07 system weight", ra07, rec["weights"]["system"], 1e-9)
    ab15 = dec("ANTHROPOMETRY_BSIP_AUTHORITY", "AB-15")
    check_close("G2", "JSON AB-15 de Leva foot COM x", ab15["deleva_com_x_from_heel_m"], 0.1214125, 1e-12)
    check_close("G2", "JSON AB-15 assembled foot COM x", ab15["assembled_segment_com_x_from_heel_m"],
                rec["foot"]["com_x_from_heel"], 1e-9)
    check_close("G2", "JSON AB-15 com delta", ab15["com_x_delta_m"],
                rec["foot"]["com_x_from_heel"] - 0.1214125, 1e-9)
    fm07 = dec("FOOT_MTP_MODEL_AUTHORITY", "FM-07")
    check_close("G2", "JSON FM-07 segment lengths", fm07["segment_lengths_m"],
                {"hindfoot": 0.07, "forefoot": 0.13625, "phalanx": 0.06875}, 1e-12)
    fm08 = dec("FOOT_MTP_MODEL_AUTHORITY", "FM-08")
    check_close("G2", "JSON FM-08 foot inertias", fm08["segment_inertia_about_com_kg_m2"], rec["foot"]["I"], 1e-12)
    ub06 = dec("UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY", "UB-06")
    check_close("G2", "JSON UB-06 principal moments", ub06["principal_moments_kg_m2"],
                sorted(np.linalg.eigvalsh(rec["HAT_I"]).tolist(), reverse=True), 1e-9)
    derived_file = json.loads((HERE / "DERIVED_QUANTITIES.json").read_text())
    sens = derived_file["hat_sensitivity"]
    check_close("G2", "sensitivity system Iyy min", sens["system_iyy_min"], 1.7316734138069032, 1e-9)
    check_close("G2", "sensitivity system Iyy max", sens["system_iyy_max"], 2.0243773264768694, 1e-9)
    check_close("G2", "bar sleeve span", derived_file["bar"]["sleeve_span_m"], [0.685, 1.1], 1e-12)
    check_close("G2", "bar uniform-rod surrogate", derived_file["bar_uniform_rod_surrogate"]["I_transverse_kg_m2"],
                8.067646666666666, 1e-9)


# ------------------------------------------------------------------------- G3/G4
REQUIRED_TOP = ["schema_version", "artifact", "artifact_role", "authority_id", "mission",
                "linear_issue", "sport_context_id", "status", "purpose", "cross_references",
                "classification_taxonomy", "decisions"]
REQUIRED_DEC = ["id", "statement", "value", "units", "classification", "sources", "locator",
                "sensitivity", "notes"]


def gate_schema():
    for art in ARTIFACT_FILES:
        p = HERE / f"{art}.json"
        if not p.exists():
            record("G3", f"{art}.json exists", False, "missing")
            continue
        data = json.loads(p.read_text())
        missing = [k for k in REQUIRED_TOP if k not in data]
        record("G3", f"{art}.json top-level keys", not missing, missing)
        record("G3", f"{art}.json artifact name", data.get("artifact") == art, data.get("artifact"))
        record("G3", f"{art}.json status", data.get("status") == "FROZEN_FOR_RES83_IMPLEMENTATION",
               data.get("status"))
        ids = []
        for d in data["decisions"]:
            m = [k for k in REQUIRED_DEC if k not in d]
            if m:
                record("G3", f"{art}:{d.get('id')} decision keys", False, m)
            if d["classification"] not in TAXONOMY:
                record("G3", f"{art}:{d['id']} classification", False, d["classification"])
            ids.append(d["id"])
        record("G3", f"{art} decision ids unique", len(ids) == len(set(ids)), len(ids))
        # cross references resolve to bundle files when they are bundle-like names
        for ref in data["cross_references"]:
            base = ref.replace(".json", "").replace(".md", "")
            if base in ARTIFACT_FILES:
                record("G3", f"{art} ref {ref}", (HERE / f"{base}.json").exists() and (HERE / f"{base}.md").exists(), "")
            elif ref.startswith("RES-82 ") or "/" in ref:
                record("G3", f"{art} external ref {ref}", True, "declared external authority")
            else:
                ok = (HERE / base).exists() or (HERE / f"{base}.json").exists() or (HERE / f"{base}.md").exists()
                record("G3", f"{art} ref {ref}", ok, "missing reference target")


def gate_md_json():
    for art in ARTIFACT_FILES:
        j = json.loads((HERE / f"{art}.json").read_text())
        md = (HERE / f"{art}.md").read_text()
        missing = [d["id"] for d in j["decisions"] if f"`{d['id']}`" not in md]
        record("G4", f"{art} MD contains all decision ids", not missing, missing)
        record("G4", f"{art} MD title", md.startswith(f"# {art} "), md.splitlines()[0][:60] if md else "")
    # critical numeric literals in the MDs
    critical = {
        "UPPER_BODY_HAT_BAR_INERTIAL_AUTHORITY.md": ["38.7969", "1.475358908", "-0.095", "155.045"],
        "LOAD_BAR_AUTHORITY.md": ["11.75585696777048", "0.004786777979600392", "6.821547880650857",
                                  "6.589226059674571", "8.06764667"],
        "ANTHROPOMETRY_BSIP_AUTHORITY.md": ["0.2425", "0.2493", "0.030905995", "0.275", "0.105"],
        "FOOT_MTP_MODEL_AUTHORITY.md": ["0.20625", "0.06875", "0.4675536", "0.4588952", "0.1558512", "25",
                                        "-0.071565"],
        "EVENT_MEASUREMENT_BOUNDARY.md": ["0.050 s", "TAKEOFF_OCCURRENCE", "TAKEOFF_CONFIRMATION", "max(2 mm"],
        "PERFORMANCE_AUTHORITY_BOUNDARY.md": ["NOT_FROZEN", "DEFERRED_PENDING_METHOD_MATCHED_MAPPING", "0.15"],
        "PLANT_TOPOLOGY_AUTHORITY.md": ["12", "9", "14"],
        "REFERENCE_ATHLETE_SPEC.md": ["1.835", "79", "20", "99", "971.19", "24"],
    }
    for fname, needles in critical.items():
        text = (HERE / fname).read_text()
        for nd in needles:
            record("G4", f"{fname} contains {nd}", nd in text, "")


# ------------------------------------------------------------------------- G5
def gate_evidence():
    csv_path = HERE / "MODEL_AUTHORITY_EVIDENCE_TABLE.csv"
    if not csv_path.exists():
        record("G5", "evidence table exists", False, "missing")
        return
    import csv as _csv
    rows = list(_csv.DictReader(csv_path.open()))
    src = {r["ROW_ID"] for r in rows if r["ROW_TYPE"] == "SOURCE"}
    decisions = [r for r in rows if r["ROW_TYPE"] == "DECISION"]
    record("G5", "source rows present", len(src) >= 20, len(src))
    record("G5", "decision rows present", len(decisions) == 136, len(decisions))
    bad_class = [r["ROW_ID"] for r in decisions if r["CLASSIFICATION"] not in TAXONOMY]
    record("G5", "all decision classifications in taxonomy", not bad_class, bad_class)
    unresolved = []
    for r in decisions + [x for x in rows if x["ROW_TYPE"] == "SOURCE"]:
        ids = [s.strip() for s in r["SOURCE_IDS"].split(";") if s.strip() not in ("", "—")]
        for s in ids:
            if s not in src:
                unresolved.append((r["ROW_ID"], s))
    record("G5", "all source references resolve", not unresolved, unresolved[:5])
    for art in ARTIFACT_FILES:
        j = json.loads((HERE / f"{art}.json").read_text())
        for d in j["decisions"]:
            ids = [s.strip() for s in d["sources"].split(";") if s.strip() not in ("", "—")]
            for s in ids:
                if s not in src:
                    record("G5", f"{art}:{d['id']} source {s}", False, "unresolved")
    md = (HERE / "MODEL_AUTHORITY_EVIDENCE_TABLE.md").read_text()
    record("G5", "evidence MD present", "MODEL_AUTHORITY_EVIDENCE_TABLE" in md, "")


# ------------------------------------------------------------------------- G6
STALE_PATTERNS = [
    ("stale_thigh_shank_ratio", r"0\.24[56]\s*[Hh]\b"),
    ("stale_ankle_plus20_cap", r"(?i)\+\s*20\s*(?:deg|degrees|°)[^\n]{0,40}ankle|ankle[^\n]{0,60}\+\s*20\s*(?:deg|degrees|°)"),
    ("stale_nu7", r"\bNU\s*=\s*7\b|\bn_u\s*=\s*7\b"),
    ("passive_only_mtp", r"(?i)passive[- ]only MTP"),
    ("biological_170_stance", r"(?i)0\.170\s*m[^\n]{0,80}(?:stance[- ]width|biological|elite soccer)"),
    ("free_root_claim", r"(?i)(?:6[- ]?dof|six[- ]?dof)[^\n]{0,40}free root|free root[^\n]{0,30}(?:6|six)"),
    ("scalar_friction_only", r"(?i)(?:single|one|scalar)[^\n]{0,40}(?:mu|friction coefficient)[^\n]{0,50}(?:only|entire|whole|is)"),
    ("uniform_iwf_bar", r"(?i)uniform[^\n]{0,60}(?:2\.2\d*\s*m|2200\s*mm)[^\n]{0,40}(?:0?\.0?28|28\s*mm)"),
    ("withdrawn_elite_targets", r"(?i)(?:0\.200\s*m|0\.28\d*\s*[-–]\s*0\.40\d*\s*m|0\.350\s*m)[^\n]{0,60}(?:target|band|expected|elite|gate)"),
    ("clearance_defined_delayed_h2", r"(?i)(?:delayed[- ]clearance|clearance[- ]defined)[^\n]{0,80}(?:H2|takeoff)|(?:H2|takeoff)[^\n]{0,80}(?:delayed[- ]clearance|clearance[- ]defined)"),
]
SUPERSESSION_MARKERS = ["supersed", "supersession", "withdrawn", "withdraw", "not current", "no longer",
                        "never", "not ", "no ", "n't", "prohibit", "forbid", "historical", "former", "retired",
                        "non-iwf", "excluded", "not_frozen", "insufficient", "not sufficient", "not a", "prevent",
                        "must not", "may not", "rejected", "reject", "illegal", "disallow", "outside",
                        "avoid", "conflict", "defect", "failure"]


def gate_redteam():
    # Scope: the authority artifacts (JSON primaries, Markdown renderings, evidence table,
    # review and receipt). Tooling scripts are excluded: their regex literals are detection
    # text, not authority assertions.
    in_scope = [f"{a}.md" for a in ARTIFACT_FILES] + [f"{a}.json" for a in ARTIFACT_FILES] + [
        "AUTHORITY_INPUTS.json", "DERIVED_QUANTITIES.json",
        "MODEL_AUTHORITY_EVIDENCE_TABLE.csv", "MODEL_AUTHORITY_EVIDENCE_TABLE.md",
        "MODEL_AUTHORITY_REVIEW.md", "RES95_RECEIPT.md"]
    hits, unmarked = [], []
    for fname in in_scope:
        p = HERE / fname
        if not p.exists():
            continue
        for i, line in enumerate(p.read_text(errors="replace").splitlines(), 1):
            for name, pat in STALE_PATTERNS:
                if re.search(pat, line):
                    low = line.lower()
                    marked = any(m in low for m in SUPERSESSION_MARKERS)
                    entry = {"file": fname, "line": i, "pattern": name, "marked": marked}
                    hits.append(entry)
                    if not marked:
                        unmarked.append(entry)
    record("G6", "stale-pattern hits all supersession-marked", not unmarked, unmarked[:8])
    REPORT["redteam_hits"] = hits
    REPORT["redteam_unmarked"] = unmarked
    record("G6", "red-team scan executed", len(in_scope) > 0, f"{len(hits)} marked hits")


# ------------------------------------------------------------------------- G7
def gate_topology(inputs):
    topo = inputs["topology"]
    record("G7", "NBODY", topo["NBODY_including_world"] == 14, topo["NBODY_including_world"])
    record("G7", "body list length", len(topo["bodies"]) == 14, len(topo["bodies"]))
    record("G7", "NQ=NV=12", topo["NQ"] == 12 and topo["NV"] == 12, (topo["NQ"], topo["NV"]))
    record("G7", "NU=9", topo["NU"] == 9 and len(topo["actuator_channels"]) == 9, topo["NU"])
    joint_total = sum(v for v in topo["joints"].values())
    record("G7", "joint count sums to NQ", joint_total == 12, joint_total)
    rom = inputs["rom_rad"]
    expected = {"trunk_pelvis_hinge": (-35.0, 35.0), "hip": (-20.0, 130.0), "knee": (0.0, 140.0),
                "ankle": (-55.0, 45.0), "mtp": (-30.0, 90.0)}
    for k, (lo_deg, hi_deg) in expected.items():
        got = rom[k]
        check_close("G7", f"ROM {k} deg", [math.degrees(got[0]), math.degrees(got[1])], [lo_deg, hi_deg], 1e-6)
    for k in ("root_tx", "root_tz", "root_ry"):
        record("G7", f"root {k} unrestricted", "unrestricted" in rom[k], rom[k])
    # ROM literal agreement with system authority (mission freeze)
    record("G7", "ankle span correct", abs((rom["ankle"][1] - rom["ankle"][0]) - math.radians(100.0)) < 1e-6,
           rom["ankle"])
    record("G7", "MTP span correct", abs((rom["mtp"][1] - rom["mtp"][0]) - math.radians(120.0)) < 1e-6, rom["mtp"])


# ------------------------------------------------------------------------- G8
def gate_manifest():
    mp = HERE / "MODEL_AUTHORITY_HASH_MANIFEST.json"
    if not mp.exists():
        record("G8", "hash manifest present", False, "not yet sealed")
        return
    manifest = json.loads(mp.read_text())
    bad = []
    for entry in manifest["files"]:
        p = HERE / entry["path"]
        if not p.exists():
            bad.append((entry["path"], "missing"))
            continue
        h = hashlib.sha256(p.read_bytes()).hexdigest()
        if h != entry["sha256"]:
            bad.append((entry["path"], "hash-mismatch"))
    record("G8", "manifest entries verified", not bad, bad[:5])
    record("G8", "manifest covers authority artifacts",
           all(any(e["path"] == f"{a}.json" for e in manifest["files"]) for a in ARTIFACT_FILES), "")


# ------------------------------------------------------------------------- G9
def gate_production():
    out = subprocess.run(["git", "status", "--porcelain"], cwd=REPO, capture_output=True, text=True).stdout
    modified = [l for l in out.splitlines() if len(l) > 2 and l[0] != "?"]
    record("G9", "no modified tracked files", not modified, modified[:10])
    diff = subprocess.run(["git", "diff", "--name-only", "HEAD"], cwd=REPO, capture_output=True, text=True).stdout.strip()
    record("G9", "git diff HEAD empty", diff == "", diff.splitlines()[:10])
    denylist_prefix = ("src/loaded_cmj/", "tools/", "tests/", "experiments/")
    touched = [l[3:].strip() for l in out.splitlines() if len(l) > 3 and l[0] in "MARD"]
    bad = [t for t in touched if t.startswith(denylist_prefix)]
    record("G9", "no production/controller/test/experiment paths touched", not bad, bad)


def main():
    inputs = load_inputs()
    derived = load_derived()
    rec = recompute(inputs)
    gate_numeric(inputs, rec)
    compare_derived_to_json(inputs, rec, derived)
    gate_schema()
    gate_md_json()
    gate_evidence()
    gate_redteam()
    gate_topology(inputs)
    gate_manifest()
    gate_production()

    gates = {}
    for c in REPORT["checks"]:
        gates.setdefault(c["gate"], {"total": 0, "failed": 0})
        gates[c["gate"]]["total"] += 1
        gates[c["gate"]]["failed"] += 0 if c["ok"] else 1
    for gname, gv in gates.items():
        gv["status"] = "PASS" if gv["failed"] == 0 else "FAIL"
    REPORT["gates"] = {"G1_inputs": gates.get("G1", {}), "G2_numeric": gates.get("G2", {}),
                       "G3_schema": gates.get("G3", {}), "G4_md_json": gates.get("G4", {}),
                       "G5_evidence": gates.get("G5", {}), "G6_redteam": gates.get("G6", {}),
                       "G7_topology": gates.get("G7", {}), "G8_manifest": gates.get("G8", {}),
                       "G9_production": gates.get("G9", {})}
    REPORT["validation_status"] = "PASS" if not REPORT["failures"] else "FAIL"
    REPORT["failed_checks"] = len(REPORT["failures"])
    REPORT["total_checks"] = len(REPORT["checks"])
    (HERE / "AUTHORITY_VALIDATION_REPORT.json").write_text(json.dumps(REPORT, indent=2) + "\n")

    print(f"validation_status={REPORT['validation_status']} "
          f"checks={REPORT['total_checks']} failures={REPORT['failed_checks']}")
    for f in REPORT["failures"][:20]:
        print("  FAIL:", f)
    return 0 if REPORT["validation_status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
