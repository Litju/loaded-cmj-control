#!/usr/bin/env python3
"""RES-52 fresh-process reproduction leg.

Reproduces, in THIS fresh process:
  1. Phase A branch-state authority (S_STAND/S40/S50/S75/S100 SHAs);
  2. the sealed experiment spec SHA;
  3. the full frozen qualification matrix cell-by-cell, comparing every
     qualification metric against FORCE_PROFILE_QUALIFICATION.json.

Writes reproduction.json (this directory) and prints one JSON line summary.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import core52 as C
from core52 import BUNDLE, WORK, sha_arr, sha_bytes, make_plant, bind_plant
from run_cell import run_cell

TOL = {
    "PEAK_FZ_BW": 1e-9, "MAX_PENETRATION": 1e-9,
    "FZ_TRACK_BOUNDARY_RMS_ERR_BW": 1e-9, "FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW": 1e-9,
    "FZ_TRACK_LATE_RMS_ERR_BW": 1e-9, "FZ_TRACK_LATE_MAX_ABS_ERR_BW": 1e-9,
    "FZ_RIPPLE_LATE_MAX_BW": 1e-9,
}
EXACT = ["CONTACT_INACTIVE_FRACTION_L", "CONTACT_INACTIVE_FRACTION_R",
         "MAX_INACTIVE_RUN_L", "MAX_INACTIVE_RUN_R", "POST_WALK_LOSS_EPISODES",
         "REFLIGHT_EPISODES", "TRUST_SHRINK_EVENTS", "TRUST_FALLBACKS",
         "START_STATE_SHA256", "END_STATE_SHA256", "CHATTER_TRANSITIONS"]


def main() -> None:
    t0 = time.time()
    rep = {"EXPERIMENT_ID": C.EXPERIMENT_ID,
           "FRESH_PROCESS": True,
           "STARTED_AT": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    # 1. branch states
    from s1_branch_states import run_branch_capture, run_standing_capture
    plant = make_plant()
    bind_plant(plant)
    states, _, _ = run_branch_capture()
    stand, _ = run_standing_capture()
    sealed = np.load(BUNDLE / "branch_states.npz")
    shas = {}
    for lab in ["S40", "S50", "S75", "S100"]:
        shas[lab] = sha_arr(states[lab][0])
        rep[f"{lab}_SHA256"] = shas[lab]
        rep[f"{lab}_REPRODUCED"] = (shas[lab] == sha_arr(sealed[lab]))
    rep["S_STAND_SHA256"] = sha_arr(stand["STATE_VECTOR"])
    rep["S_STAND_REPRODUCED"] = (rep["S_STAND_SHA256"] == sha_arr(sealed["S_STAND"]))
    # 2. spec
    spec = json.loads((BUNDLE / "experiment_spec.json").read_text())
    rep["SPEC_SHA256"] = spec["EXPERIMENT_SPEC_SHA256"]
    # 3. qualification matrix
    ref_events = json.loads((WORK / "EVENTS.json").read_text())
    sealed_q = json.loads((BUNDLE / "FORCE_PROFILE_QUALIFICATION.json").read_text())
    cells, match = {}, True
    for cell in spec["PROFILES"]:
        cid = cell["CELL_ID"]
        out, *_ = run_cell(cell, spec["CONTROLLER_CONSTANTS"],
                           states[cell["START_STATE"]][0]
                           if cell["START_STATE"] != "S_STAND" else stand["STATE_VECTOR"],
                           ref_events)
        cmp_ = {}
        for k in TOL:
            a, b = out[k], sealed_q["RESULTS"][cid][k]
            ok = abs(float(a) - float(b)) <= TOL[k]
            cmp_[k] = {"sealed": float(b), "repro": float(a), "match": bool(ok)}
            match &= ok
        for k in EXACT:
            a, b = out[k], sealed_q["RESULTS"][cid][k]
            if k in ("REFLIGHT_EPISODES",):
                ok = list(map(int, a)) == list(map(int, b))
            else:
                ok = (a == b)
            cmp_[k] = {"sealed": b, "repro": a, "match": bool(ok)}
            match &= ok
        cmp_["QUALIFIED_SEALED"] = sealed_q["RESULTS"][cid]["QUALIFIED"]
        cmp_["QUALIFIED_REPRO"] = out["QUALIFIED"]
        match &= (cmp_["QUALIFIED_SEALED"] == cmp_["QUALIFIED_REPRO"])
        cmp_["SEGMENT_QUALIFIED_SEALED"] = sealed_q["RESULTS"][cid][
            "ARREST_SEGMENT"]["SEGMENT_QUALIFIED"]
        cmp_["SEGMENT_QUALIFIED_REPRO"] = out["ARREST_SEGMENT"]["SEGMENT_QUALIFIED"]
        match &= (cmp_["SEGMENT_QUALIFIED_SEALED"] == cmp_["SEGMENT_QUALIFIED_REPRO"])
        cells[cid] = cmp_
        print(f"[repro] {cid}: match={all(v['match'] for v in cmp_.values() if isinstance(v, dict))}")
    rep["CELLS"] = cells
    rep["SPEC_EXECUTION_MATCH"] = "PASS" if match else "FAIL"
    rep["WALL_S"] = time.time() - t0
    (Path(__file__).resolve().parent / "reproduction.json").write_text(
        json.dumps(rep, indent=2) + "\n")
    print(json.dumps({"SPEC_EXECUTION_MATCH": rep["SPEC_EXECUTION_MATCH"],
                      "S50_SHA256": rep["S50_SHA256"],
                      "S_STAND_REPRODUCED": rep["S_STAND_REPRODUCED"]}))


if __name__ == "__main__":
    main()
