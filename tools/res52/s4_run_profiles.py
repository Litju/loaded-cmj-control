#!/usr/bin/env python3
"""RES-52: execute the frozen qualification matrix (P0a..P3), one cell each,
retaining all traces and all failed cells as evidence. Writes
FORCE_PROFILE_QUALIFICATION.json and CONTACT_VIABILITY_MAP.json."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import core52 as C
from core52 import BUNDLE, WORK
from run_cell import run_cell


def main() -> None:
    t0 = time.time()
    spec = json.loads((BUNDLE / "experiment_spec.json").read_text())
    states = np.load(BUNDLE / "branch_states.npz")
    ref_events = json.loads((WORK / "EVENTS.json").read_text())
    constants = spec["CONTROLLER_CONSTANTS"]
    auth = json.loads((BUNDLE / "BRANCH_STATE_AUTHORITY.json").read_text())

    results = {}
    traces_ok = True
    for cell in spec["PROFILES"]:
        cid = cell["CELL_ID"]
        sv = states[cell["START_STATE"]]
        t0c = time.time()
        out, acc, arr, matrices, infos, policy = run_cell(
            cell, constants, sv, ref_events)
        out["WALL_S"] = time.time() - t0c
        out["SPEC_SHA256"] = spec["EXPERIMENT_SPEC_SHA256"]
        results[cid] = out
        np.savez_compressed(BUNDLE / f"trace_{cid}.npz", **arr,
                            c_t=acc.control_arrays()["c_t"],
                            c_phase=acc.control_arrays()["c_phase"],
                            G=np.asarray(matrices, dtype=float))
        diag = []
        for info in infos:
            diag.append({
                "u": info["u"].tolist(), "u_nom": info["u_nom"].tolist(),
                "du": info["du"].tolist(), "rho": info["rho"],
                "shrinks": info["shrinks"], "VALIDATED": info["VALIDATED"],
                "FALLBACK": info["FALLBACK"],
                "Y_NOM": info["Y_NOM"].tolist(), "Y_VAL": info["Y_VAL"].tolist(),
                "Y_PRED": info["Y_PRED"].tolist(), "VAL_ERR": info["VAL_ERR"].tolist(),
                "FLAGS_NOM": list(info["FLAGS_NOM"]), "FLAGS_VAL": list(info["FLAGS_VAL"]),
                "RHO_AFTER": info["RHO_AFTER"],
                "CROSSINGS": [bool(mm.get("active_set_crossing", False)) for mm in info["META"]],
            })
        (BUNDLE / f"modeldiag_{cid}.json").write_text(json.dumps(diag, indent=1) + "\n")
        status = "QUALIFIED" if out["QUALIFIED"] else (
            "SEGMENT_QUALIFIED" if out.get("ARREST_SEGMENT", {}).get("SEGMENT_QUALIFIED")
            else "FAILED")
        print(f"[cell] {cid}: {status} bndRMS={out['FZ_TRACK_BOUNDARY_RMS_ERR_BW']:.4f} "
              f"bndMAX={out['FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW']:.4f} "
              f"peakBW={out['PEAK_FZ_BW']:.3f} pen={out['MAX_PENETRATION']*1e3:.3f}mm "
              f"shr={out['TRUST_SHRINK_EVENTS']} fb={out['TRUST_FALLBACKS']} "
              f"({out['WALL_S']:.0f}s)")
        for gk, gv in out["GATES"].items():
            if not gv:
                print(f"    GATE_FAIL {gk}")
        if not out.get("QUALIFIED") and out.get("ARREST_SEGMENT"):
            for gk, gv in out["ARREST_SEGMENT"]["SEGMENT_GATES"].items():
                if not gv:
                    print(f"    SEG_GATE_FAIL {gk}")

    # ---- contact viability map ----
    def _state_auth(name: str) -> dict:
        return auth["STATES"][name] if name in auth["STATES"] else auth["S_STAND"]

    viability = []
    for cell in spec["PROFILES"]:
        cid = cell["CELL_ID"]
        r = results[cid]
        seg = r.get("ARREST_SEGMENT", {})
        sa = _state_auth(cell["START_STATE"])
        viability.append({
            "CELL_ID": cid,
            "START_STATE": cell["START_STATE"],
            "START_STATE_TIME_S": r["START_STATE_TIME"],
            "START_STATE_SHA256": r["START_STATE_SHA256"],
            "FZ_NODES_BW": cell["FZ_NODES_BW"],
            "HORIZON_S": cell["HORIZON_S"],
            "ARREST_SEGMENT_S": seg.get("ARREST_SEGMENT_S"),
            "START_WHOLE_FZ_N": sa["FZ_WHOLE"],
            "START_PEN_M": max(sa["PENETRATION_L"], sa["PENETRATION_R"]),
            "START_FOOT_NVEL_MPS": max(abs(sa["FOOT_NORMAL_VEL_L"]),
                                       abs(sa["FOOT_NORMAL_VEL_R"])),
            "START_COM_VZ": sa["COM_VZ"],
            "QUALIFIED_FULL_HORIZON": bool(r["QUALIFIED"]),
            "QUALIFIED_ARREST_SEGMENT": bool(seg.get("SEGMENT_QUALIFIED", False)),
            "PEAK_FZ_BW": r["PEAK_FZ_BW"],
            "MAX_PENETRATION_M": r["MAX_PENETRATION"],
            "BOUNDARY_RMS_ERR_BW": r["FZ_TRACK_BOUNDARY_RMS_ERR_BW"],
            "BOUNDARY_MAX_ERR_BW": r["FZ_TRACK_BOUNDARY_MAX_ABS_ERR_BW"],
            "RIPPLE_LATE_MAX_BW": r["FZ_RIPPLE_LATE_MAX_BW"],
            "POST_WALK_LOSS_EPISODES": r["POST_WALK_LOSS_EPISODES"],
            "REFLIGHT_EPISODES": r["REFLIGHT_EPISODES"],
            "TRUST_FALLBACKS": r["TRUST_FALLBACKS"],
            "TRUST_SHRINK_EVENTS": r["TRUST_SHRINK_EVENTS"],
            "FAILED_GATES": [g for g, v in r["GATES"].items() if not v],
        })
    vm = {
        "EXPERIMENT_ID": C.EXPERIMENT_ID,
        "SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"],
        "MAP": viability,
        "QUALIFIED_DOMAIN": {
            "FULL_HORIZON_CELLS": [v["CELL_ID"] for v in viability if v["QUALIFIED_FULL_HORIZON"]],
            "ARREST_SEGMENT_CELLS": [v["CELL_ID"] for v in viability if v["QUALIFIED_ARREST_SEGMENT"]],
            "H1_CONTACT_REALIZATION": None,  # sealed by the assessment step
        },
    }
    (BUNDLE / "CONTACT_VIABILITY_MAP.json").write_text(json.dumps(vm, indent=2) + "\n")

    summary = {
        "EXPERIMENT_ID": C.EXPERIMENT_ID,
        "EXECUTED_AT": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "SPEC_SHA256": spec["EXPERIMENT_SPEC_SHA256"],
        "RESULTS": results,
        "ALL_QUALIFIED": all(r["QUALIFIED"] for r in results.values()),
        "FULLY_QUALIFIED_CELLS": [cid for cid, r in results.items() if r["QUALIFIED"]],
        "SEGMENT_QUALIFIED_CELLS": [cid for cid, r in results.items()
                                    if not r["QUALIFIED"]
                                    and r.get("ARREST_SEGMENT", {}).get("SEGMENT_QUALIFIED")],
        "WALL_S": time.time() - t0,
    }
    (BUNDLE / "FORCE_PROFILE_QUALIFICATION.json").write_text(
        json.dumps(summary, indent=2) + "\n")
    print("[s4] ALL_QUALIFIED =", summary["ALL_QUALIFIED"],
          "| SEGMENT:", summary["SEGMENT_QUALIFIED_CELLS"])


if __name__ == "__main__":
    main()
