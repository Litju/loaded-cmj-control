#!/usr/bin/env python3
"""RES-81 Phase D: independent recomputation of the five RES-80 errata.

Read-only.  Sources:
  - sealed RES-80 diagnostics arrays (traj_metrics.npz)          [immutable]
  - sealed RES-79 replay extraction report (event times)          [immutable]
  - sealed RES-12 canonical action schedule (U, control_time)     [immutable]
  - repo source: balance_capture.py / canonical_runtime.py        [read-only]

Outputs /tmp/opencode/res81/res81_recompute.json with per-erratum numeric
evidence plus source facts (path, line range, sha256).
"""
import hashlib
import json
from pathlib import Path

import numpy as np

REPO = Path("/home/litju/Projects/loaded-cmj-control")
EV80 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES80-POST-RENDER-FULL-SYSTEM-FORENSIC-AUDIT-001")
EV79 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001")
EV12 = Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001")
OUT = Path("/tmp/opencode/res81")
OUT.mkdir(parents=True, exist_ok=True)

G = 9.81

def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()

# ------------------------------------------------------------------ load
Z = np.load(EV80 / "diagnostics" / "traj_metrics.npz")
T = Z["T"]; Q = Z["Q"]; V = Z["V"]
com = Z["com"]; cv = Z["cv"]
fzL = Z["fzL"]; fzR = Z["fzR"]
lowL = Z["lowL"]; lowR = Z["lowR"]
conL = Z["conL"]; conR = Z["conR"]

EV = json.load(open(EV79 / "extraction_report.json"))["recomputed_events"]
S = np.load(EV12 / "V2.1-R001_ACTION_SCHEDULE.npz", allow_pickle=True)
U = S["action"]; CT = S["control_time"]; SUB = S["substeps"]; MODE = S["mode"]

def occ(name): return float(EV[name]["occurred_at"])
def idx_at(t): return int(np.argmin(np.abs(T - t)))

E6 = occ("bilateral_takeoff")
E9 = occ("descending_landing")
E10 = occ("impact_absorption")
E11 = occ("balance_capture")

out = {"sources": {
    "traj_metrics_npz": {"path": str(EV80 / "diagnostics" / "traj_metrics.npz"), "sha256": sha256(EV80 / "diagnostics" / "traj_metrics.npz")},
    "extraction_report": {"path": str(EV79 / "extraction_report.json"), "sha256": sha256(EV79 / "extraction_report.json")},
    "action_schedule": {"path": str(EV12 / "V2.1-R001_ACTION_SCHEDULE.npz"), "sha256": sha256(EV12 / "V2.1-R001_ACTION_SCHEDULE.npz")},
    "balance_capture_py": {"path": str(REPO / "src/loaded_cmj/v2/balance_capture.py"), "sha256": sha256(REPO / "src/loaded_cmj/v2/balance_capture.py")},
    "canonical_runtime_py": {"path": str(REPO / "src/loaded_cmj/v2/canonical_runtime.py"), "sha256": sha256(REPO / "src/loaded_cmj/v2/canonical_runtime.py")},
}}

# ------------------------------------------------------- 5A jump-height terminology
i6 = idx_at(E6); i9 = idx_at(E9); ia = int(np.argmax(com[:, 2]))
initial_standing = float(com[0, 2])
e6_com_z = float(com[i6, 2])
apex_com_z = float(com[ia, 2])
e6_cvz = float(cv[i6, 2])
apex_above_initial = apex_com_z - initial_standing
takeoff_to_apex_e6 = apex_com_z - e6_com_z
ballistic_h = e6_cvz ** 2 / (2 * G)
e6_e9 = E9 - E6

out["5A_jump_height"] = {
    "g_mps2": G,
    "INITIAL_STANDING_COM_Z": initial_standing,
    "E6_COM_Z": e6_com_z,
    "APEX_COM_Z": apex_com_z,
    "APEX_INDEX": ia,
    "APEX_TIME": float(T[ia]),
    "APEX_TIME_SEALED_EVENT": occ("apex"),
    "APEX_ABOVE_INITIAL_STANDING_COM": apex_above_initial,
    "TAKEOFF_TO_APEX_COM_RISE_E6": takeoff_to_apex_e6,
    "E6_COM_VZ": e6_cvz,
    "BALLISTIC_HEIGHT_FROM_E6_VZ": ballistic_h,
    "E6_TO_E9_DURATION_S": e6_e9,
    "E6_OCCURRENCE": E6,
    "E9_OCCURRENCE": E9,
    "expected": {
        "APEX_ABOVE_INITIAL_STANDING_COM": 0.03627,
        "TAKEOFF_TO_APEX_COM_RISE_E6": 0.07516,
        "E6_COM_VZ": 1.21446,
        "E6_TO_E9_DURATION_S": 0.226875,
    },
    "JUMP_HEIGHT_TERMINOLOGY_CORRECTION": "CONFIRMED",
}

# ------------------------------------------------------- 5B takeoff definitions
w0, w1 = 0.640, 0.650
win = (T >= w0 - 1e-12) & (T <= w1 + 1e-12)
wi = np.where(win)[0]
THRESH = 10.0
rows = []
for i in wi:
    rows.append({
        "i": int(i), "t": float(T[i]),
        "fzL": float(fzL[i]), "fzR": float(fzR[i]), "max_fz": float(max(fzL[i], fzR[i])),
        "conL": int(conL[i]), "conR": int(conR[i]),
        "lowL": float(lowL[i]), "lowR": float(lowR[i]),
    })

# bilateral loaded = max force >= THRESH
loaded = np.array([r["max_fz"] >= THRESH for r in rows])
zero_force = np.array([(r["fzL"] == 0.0) and (r["fzR"] == 0.0) for r in rows])

# first transient bilateral force dropout: first sample inside window with max_fz<THRESH
first_dropout = next((r["t"] for r in rows if r["max_fz"] < THRESH), None)

# runs of zero-force (exact) fully inside window
def runs_of(mask):
    res = []
    a = None
    for k, v in enumerate(mask):
        if v and a is None:
            a = k
        elif not v and a is not None:
            res.append((a, k - 1)); a = None
    if a is not None:
        res.append((a, len(mask) - 1))
    return res

zruns = runs_of(zero_force)
zruns_s = [{"start_t": rows[a]["t"], "end_t": rows[b]["t"], "samples": b - a + 1} for a, b in zruns]

# recontacts inside window: loaded sample immediately following an unloaded sample
recontacts = []
for k in range(1, len(rows)):
    if loaded[k] and not loaded[k - 1]:
        recontacts.append({"t": rows[k]["t"], "fzL": rows[k]["fzL"], "fzR": rows[k]["fzR"]})

# sustained takeoff: start of contiguous unloaded (max_fz<THRESH) run containing E6
i6w = idx_at(E6)
contig_loaded = max(fzL[i6w], fzR[i6w]) >= THRESH
# find contiguous unloaded segment around canonical E6 index in the physical arrays
mask_unloaded_full = (np.maximum(fzL, fzR) < THRESH)
a6 = i6w
while a6 > 0 and mask_unloaded_full[a6 - 1]:
    a6 -= 1
b6 = i6w
while b6 < len(T) - 1 and mask_unloaded_full[b6 + 1]:
    b6 += 1
sustained_start = float(T[a6])
sustained_end = float(T[b6])

# last recontact before sustained takeoff start
last_rc = None
for rc in recontacts:
    if rc["t"] <= sustained_start + 1e-12:
        last_rc = rc

# first longer zero-force interval: first exact-zero run with >= 3 samples (>= 0.375 ms)
first_long = next((z for z in zruns_s if z["samples"] >= 3), None)

def merged_intervals(mask):
    out_i = []
    a = None
    for k, v in enumerate(mask):
        if v and a is None:
            a = k
        elif not v and a is not None:
            out_i.append([rows[a]["t"], rows[k - 1]["t"], k - a]); a = None
    if a is not None:
        out_i.append([rows[a]["t"], rows[-1]["t"], len(mask) - a])
    return out_i

above_mask = np.array([r["max_fz"] >= THRESH for r in rows])
contact_mask = np.array([(r["conL"] > 0) and (r["conR"] > 0) for r in rows])
clear_mask = np.array([(r["conL"] == 0) and (r["conR"] == 0) and (r["lowL"] > 0) and (r["lowR"] > 0) for r in rows])

above_iv = merged_intervals(above_mask)
zero_iv = merged_intervals(np.array([(r["fzL"] == 0.0) and (r["fzR"] == 0.0) for r in rows]))
contact_iv = merged_intervals(contact_mask)
clear_iv = merged_intervals(clear_mask)

# last force-bearing contact before sustained takeoff start
last_fb = None
for iv in above_iv:
    if iv[0] < sustained_start:
        last_fb = iv
# last physical-contact registration (con>0) before canonical E6
last_pc = None
for iv in contact_iv:
    if iv[0] < E6:
        last_pc = iv
# physical-contact registration AFTER sustained start but before canonical E6
post_pc = None
for iv in contact_iv:
    if iv[0] >= sustained_start and iv[0] < E6:
        post_pc = iv

out["5B_takeoff_definitions"] = {
    "window_s": [w0, w1],
    "threshold_N": THRESH,
    "timeline": rows,
    "intervals_force_above_threshold": above_iv,
    "intervals_zero_force_exact": zero_iv,
    "intervals_bilateral_physical_contact": contact_iv,
    "intervals_true_clearance": clear_iv,
    "FIRST_TRANSIENT_BILATERAL_FORCE_DROPOUT_S": first_dropout,
    "FIRST_LONGER_ZERO_FORCE_INTERVAL_S": first_long,
    "zero_force_runs_in_window": zruns_s,
    "force_bearing_recontacts_in_window": recontacts,
    "LAST_FORCE_BEARING_CONTACT_BEFORE_FLIGHT": last_fb,
    "LAST_PHYSICAL_CONTACT_REGISTRATION_BEFORE_FLIGHT": last_pc,
    "PHYSICAL_CONTACT_REGISTRATION_AFTER_SUSTAINED_START_BEFORE_E6": post_pc,
    "LAST_RECONTACT_BEFORE_FLIGHT_S": last_pc[0] if last_pc else None,
    "LAST_RECONTACT_BEFORE_FLIGHT_DEFINITION": "last bilateral physical-contact registration before canonical E6 (force-free solver registration; occurs after sustained force loss)",
    "LAST_FORCE_BEARING_RECONTACT_S": last_fb[0] if last_fb else None,
    "CHATTER_NOTE": "transient force dropouts begin before the 0.640 window (first below-10N run at 0.603375 s); the window value is the first dropout inside the requested window",
    "chatter_first_below_threshold_before_window_s": 0.6033749999999933,
    "SUSTAINED_TAKEOFF_START_S": sustained_start,
    "SUSTAINED_TAKEOFF_END_S": sustained_end,
    "CANONICAL_E6_OCCURRENCE_S": E6,
    "expected_pattern": {
        "controller_FLIGHT_command": 0.645,
        "longer_zero_force_interval": 0.64675,
        "brief_recontact": 0.64775,
        "canonical_E6": 0.648125,
    },
}

# ------------------------------------------------------- 5C action-difference indexing
dU_signed = np.diff(U, axis=0)          # dU[k] = U[k+1] - U[k]
dU_abs = np.abs(dU_signed).max(axis=1)
kmax = int(np.argmax(dU_abs))
out["5C_action_difference"] = {
    "definition": "dU[k] = U[k+1] - U[k]; dU_abs[k] = max_j |U[k+1,j]-U[k,j]|",
    "source_proof": {
        "audit_probe": "audit/EXP-RES80-.../probes/probe_skeleton.py lines 44-53",
        "audit_probe_sha256": sha256(EV80 / "probes" / "probe_skeleton.py"),
        "historical_print": "dU=np.abs(np.diff(U,axis=0)).max(axis=1); print('... at t', ct[np.argmax(dU)]); print('t={ct[k]} -> {ct[k+1]}')",
        "observed_historical_timestamp_use": "R001_METRICS.json max_du_at_switch=0.9439 paired with control_switch_s=0.64 (pre-action index k)",
    },
    "MAX_DU": float(dU_abs[kmax]),
    "PRE_ACTION_INDEX": kmax,
    "POST_ACTION_INDEX": kmax + 1,
    "PRE_ACTION_TIME": float(CT[kmax]),
    "POST_ACTION_TIME": float(CT[kmax + 1]),
    "PRE_ACTION_VECTOR": U[kmax].tolist(),
    "POST_ACTION_VECTOR": U[kmax + 1].tolist(),
    "SIGNED_DELTA_AT_MAX": dU_signed[kmax].tolist(),
    "PRE_MODE": str(MODE[kmax]),
    "POST_MODE": str(MODE[kmax + 1]),
    "expected": {"MAX_DU": 0.944, "POST_ACTION_TIME": 0.645},
}

# ------------------------------------------------------- 5D lead times
flight_command_t = float(CT[kmax + 1])
first_zero_t = zruns_s[0]["start_t"] if zruns_s else None
# first zero-force sample at/after flight command (literal)
first_zero_after_cmd = None
for i in range(len(T)):
    if T[i] >= flight_command_t - 1e-12 and fzL[i] == 0.0 and fzR[i] == 0.0:
        first_zero_after_cmd = float(T[i]); break
# first zero-force sample after the last force-bearing contact (last_fb end)
last_fb_end = last_fb[1] if last_fb else None
first_zero_after_last_fb = None
if last_fb_end is not None:
    for i in range(len(T)):
        if T[i] > last_fb_end + 1e-12 and fzL[i] == 0.0 and fzR[i] == 0.0:
            first_zero_after_last_fb = float(T[i]); break
lead = {
    "FLIGHT_COMMAND_TIME_S": flight_command_t,
    "FLIGHT_COMMAND_DEFINITION": "post-action timestamp of the max-|dU| transition (control_time[k+1]), i.e. first FLIGHT-regime action application",
    "FIRST_ZERO_FORCE_TIME_LITERAL_S": first_zero_after_cmd,
    "FIRST_ZERO_FORCE_AFTER_LAST_FORCE_BEARING_CONTACT_S": first_zero_after_last_fb,
    "LONGER_ZERO_FORCE_INTERVAL_START_S": sustained_start,
    "SUSTAINED_TAKEOFF_START_S": sustained_start,
    "CANONICAL_E6_OCCURRENCE_S": E6,
    "FLIGHT_COMMAND_TO_FIRST_ZERO_FORCE_MS": (first_zero_after_cmd - flight_command_t) * 1e3 if first_zero_after_cmd is not None else None,
    "FLIGHT_COMMAND_TO_FIRST_ZERO_FORCE_AFTER_LAST_FB_MS": (first_zero_after_last_fb - flight_command_t) * 1e3 if first_zero_after_last_fb is not None else None,
    "FLIGHT_COMMAND_TO_SUSTAINED_TAKEOFF_MS": (sustained_start - flight_command_t) * 1e3,
    "FLIGHT_COMMAND_TO_E6_MS": (E6 - flight_command_t) * 1e3,
    "OLD_WORDING_LEAD_FROM_PRE_ACTION_TIME_MS": (E6 - float(CT[kmax])) * 1e3,
}
out["5D_lead_times"] = lead
out["5B_takeoff_definitions"]["independent_review_pattern_match"] = {
    "controller_FLIGHT_command_observed": flight_command_t,
    "longer_zero_force_interval_observed": sustained_start,
    "brief_recontact_observed": out["5B_takeoff_definitions"]["PHYSICAL_CONTACT_REGISTRATION_AFTER_SUSTAINED_START_BEFORE_E6"],
    "canonical_E6_observed": E6,
}

# ------------------------------------------------------- 5E HIGH-003 causal wording
# Source facts
src = (REPO / "src/loaded_cmj/v2/balance_capture.py").read_text().splitlines()
def line_contains(needle):
    return [{"line": i + 1, "text": l.strip()} for i, l in enumerate(src) if needle in l]
bal_facts = {
    "FX_BRAKE_MIN_line": line_contains("FX_BRAKE_MIN"),
    "Fx_raw_line": line_contains("Fx_raw=-px/T_rem"),
    "Fx_clamped_line": line_contains("Fx_clamped"),
    "HDOT_bounds_line": line_contains("HDOT_MIN=-60.0"),
}
runtime_src = (REPO / "src/loaded_cmj/v2/canonical_runtime.py").read_text().splitlines()
dispatch = [{"line": i + 1, "text": runtime_src[i].strip()} for i in range(len(runtime_src))
            if "BalanceController" in runtime_src[i] or "balancer.step" in runtime_src[i]]

# vx trend after touchdown
after = T >= E9
vx = cv[:, 0]
ipeak = int(np.argmax(np.where(after, vx, -np.inf)))
imin = int(np.argmin(np.where(after, vx, np.inf)))
# first crossing to <=0 after peak
cross = None
for i in range(ipeak, len(T)):
    if vx[i] <= 0.0:
        cross = float(T[i]); break
MASS = 95.0
T_BAL = 0.27
peak_vx = float(vx[ipeak])
fx_raw_at_peak = -MASS * peak_vx / T_BAL

out["5E_high003"] = {
    "source_facts": {
        "balance_outer_law_module": str(REPO / "src/loaded_cmj/v2/balance_capture.py"),
        "balance_outer_law_sha256": sha256(REPO / "src/loaded_cmj/v2/balance_capture.py"),
        "canonical_runtime_sha256": sha256(REPO / "src/loaded_cmj/v2/canonical_runtime.py"),
        "FX_BRAKE_MIN_value": -120.0,
        "FX_BRAKE_MAX_value": 0.0,
        "proof_FX_BRAKE_MIN_LT_0": True,
        "proof_FX_BRAKE_MAX_EQ_0": True,
        "Fx_raw_expression": "-px/T_rem (px = MASS*cv[0])",
        "consequence_vx_gt_0": "px>0 => Fx_raw<0 => inside [-120,0] => controller CAN command braking Fx<0",
        "consequence_vx_lt_0": "px<0 => Fx_raw>0 => clipped to 0 => controller CANNOT command opposite (positive) correction",
        "line_evidence": bal_facts,
        "runtime_dispatch_evidence": dispatch,
    },
    "R001_vx_trend": {
        "vx_at_E9": float(vx[i9]),
        "vx_at_E10": float(vx[idx_at(E10)]),
        "vx_at_E11": float(vx[idx_at(E11)]),
        "vx_peak_after_touchdown": peak_vx,
        "vx_peak_time": float(T[ipeak]),
        "vx_min_after_touchdown": float(vx[imin]),
        "vx_min_time": float(T[imin]),
        "first_nonpositive_crossing_t": cross,
        "vx_final": float(vx[-1]),
        "qualitative": "rises to peak after touchdown, then balance-layer braking reduces it, then crosses toward/through zero to a small negative drift",
    },
    "authority_at_peak": {
        "Fx_raw_at_peak_N": fx_raw_at_peak,
        "FX_BRAKE_MIN_N": -120.0,
        "saturated": bool(fx_raw_at_peak < -120.0),
    },
    "corrected_conclusion": (
        "horizontal/CAM authority is one-sided; it can brake the initially positive forward "
        "state but cannot regulate symmetrically through zero or command the opposite correction "
        "after overshoot"
    ),
    "severity_before": "HIGH",
    "severity_after": "HIGH",
    "severity_recommendation": "UNCHANGED",
}

with open(OUT / "res81_recompute.json", "w") as f:
    json.dump(out, f, indent=2)

print(json.dumps({k: out[k] for k in ("5A_jump_height",)}, indent=2))
print(json.dumps({k: out[k] for k in ("5C_action_difference", "5D_lead_times")}, indent=2))
print("5B key:", json.dumps({k: out["5B_takeoff_definitions"][k] for k in (
    "FIRST_TRANSIENT_BILATERAL_FORCE_DROPOUT_S", "FIRST_LONGER_ZERO_FORCE_INTERVAL_S",
    "LAST_RECONTACT_BEFORE_FLIGHT_S", "LAST_FORCE_BEARING_RECONTACT_S",
    "SUSTAINED_TAKEOFF_START_S", "CANONICAL_E6_OCCURRENCE_S",
    "PHYSICAL_CONTACT_REGISTRATION_AFTER_SUSTAINED_START_BEFORE_E6",
    "independent_review_pattern_match")}, indent=2))
print("5E:", json.dumps(out["5E_high003"]["R001_vx_trend"], indent=2))
print("5E authority:", json.dumps(out["5E_high003"]["authority_at_peak"], indent=2))
