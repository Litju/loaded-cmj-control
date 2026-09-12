#!/usr/bin/env python3
"""RES-79: deterministic replay extraction + forensic smoke render of V2.1-R001.

MISSION=RES13A_ACCEPTED_TRAJECTORY_VISUAL_SMOKE_001
LINEAR_ISSUE=RES-79
CANDIDATE_ID=V2.1-R001

DIAGNOSTIC RENDERING ONLY. This module performs no controller development,
no tuning, no optimization, no model/scorer/event redesign, and no aesthetic
re-simulation. It replays the ACCEPTED action schedule through the FROZEN
plant and renders the resulting states with a SEPARATE visualization data
structure. Rendering cannot feed back into physics by construction (see
RENDER NON-INVASIVENESS section below).

READ-ONLY posture toward science:
  * Imports ONLY frozen plant + measurement + events (observational use).
    Controller modules (res72_integration, balance_capture, stable_recovery,
    terminal_capture, full_closure, canonical_runtime) are NEVER imported here,
    so this path cannot mutate controller state even by accident.
  * Never writes to src/, to the canonical evidence bundle, or to any
    tracked scientific file. All outputs go to --out-dir (RES-79 evidence).
  * The frozen plant XML file is never modified (SHA re-verified). The only
    render-side adjustment is an in-memory override of the visualization-only
    `vis.global offwidth/offheight` framebuffer size on a SEPARATE render
    model instance that is never stepped (mj_fwdPosition/render only).

RENDER NON-INVASIVENESS (structural):
  * Physics replay uses its own V2Plant + live MjData (replay objects).
  * Telemetry sampling uses a dedicated scratch MjData + shadow MjData.
  * Frame rendering uses a dedicated viz MjData on a separate render model.
  * The viz path calls ONLY mj_fwdPosition (position kinematics, no solver,
    no dynamics) + read-only observers + Renderer.render. mj_step is never
    called on viz/scratch data.
  * Extracted history arrays are marked read-only before any render stage.
  * Empirical proof (hashes before/after) is recorded in the integrity report.

EXTRACTION IDENTITY GATES (fail-closed, all must pass):
  TRACE_SHA256 exact match, 12/12 events exact match (detector recomputed
  observationally on replay states), 7/7 canonical checkpoints exact match,
  substep schedule exact match, N_CTRL/N_PHYS/T_END exact match,
  termination + outcome exact match.

Frame map: 60 fps, frame j at t=j/60, source = nearest physics sample
(t_i=(i+1)*DT). Every rendered frame records its source sample index.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

REPO = Path(__file__).resolve().parents[1]
if str(REPO / "src") not in sys.path:
    sys.path.insert(0, str(REPO / "src"))

import numpy as np

import mujoco

from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.measurement import (
    create_measurement_data,
    SynchronizedPhysicsSample,
)
from loaded_cmj.v2.events import V2EventDetector

# --------------------------------------------------------------------------
# Frozen mission authority (literals; cross-checked against bundle on load).
# --------------------------------------------------------------------------

CANDIDATE_ID = "V2.1-R001"
TRACE_SHA256 = "4d0478793dbdc000cd84b26392e611b8d665c3b5f1996f3663b8241470f97561"
PLANT_SHA256 = "5f2244149c6b8831a1bfe6fa29a8ce33be0e41100fbdd61bfd8d9555222be191"
CONTROLLER_COMPOSITION_SHA256 = "6f56ffa180b12c128d67ea13fd9c08413554a9d964d175615f16a8e572185d12"
SCORER_SHA256 = "286ef328e4b334a15775df01ba6aad971cf8f808ddbcb028fcda4032164f2deb"
CANDIDATE_SPEC_SHA256 = "b02c74b8af547692b6544790b93c87b16a86c0ac04c76abeac47aaf2b9972e0c"
RUNTIME_CONTRACT_SHA256 = "cafb91fe838b68c8110157659d35f6fae561295e98409fc0b6bc94375d933c1e"
ENTRY_HEAD = "395c19426448ea7a3c1b7e612aff1a64e086972f"
ENTRY_TREE = "532dfb4ba638dab3cd604c4f05fefe4cbb8de6fc"

PHYS_DT = 0.000125
N_CTRL = 3048
N_PHYS = 121889
T_END = 15.236125000027243
SUBSTEP_HISTOGRAM = {40: 3045, 28: 2, 33: 1}
MASS_KG = 95.0
BW_N = MASS_KG * 9.81
OUTCOME = "PASS_E12_POST_HOLD"
TERMINATION = "OBJECTIVE_COMPLETE"
MAXUTIL = 0.8588783322803201

FPS = 60
WIDTH = 1280
HEIGHT = 720

EVENT_ORDER = [
    "supported_start",
    "countermovement_onset",
    "valid_countermovement",
    "upward_reversal",
    "vertical_propulsion",
    "bilateral_takeoff",
    "genuine_flight",
    "apex",
    "descending_landing",
    "impact_absorption",
    "balance_capture",
    "stable_recovery",
]
EVENT_TAG = {n: f"E{i+1}" for i, n in enumerate(EVENT_ORDER)}

CANONICAL_CHECKPOINTS = {
    "S_APEX": "97ed110f5ad2bb5cd8e0e91a91a503e9326814a2bfcc68ce73237900be8351c2",
    "S_E10": "7931fd8b2715362cfd766a75f46553681ce872e541daaf84a95b5dfd50da8ec0",
    "S_E11": "31c7f7d11ead4277484fa1d1b93245f5dd06f256f003365edc99b328daf7039d",
    "S_RR": "846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710",
    "S_RR_CONFIRMED": "02559b91904d7103177bae29761f65b48d7b4e9db69cd3e6c886805422766a06",
    "S_STAND_HANDOFF": "2141e801ab403b32d5fc83f94a046997041815ef70c44919796534907a329679",
    "S_E12": "363a5d89298cab15c194229a64838e561126e72285a41b48665d18bdbbe88146",
}

FAIL = "RES79_FAIL"


# --------------------------------------------------------------------------
# Small pure helpers (unit-tested).
# --------------------------------------------------------------------------

def sha_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def sha_file(p: Path) -> str:
    return sha_bytes(Path(p).read_bytes())


def vec_sha(v: np.ndarray) -> str:
    return sha_bytes(np.ascontiguousarray(np.asarray(v, float)).tobytes())


def fetch_integration_vector(model, data) -> np.ndarray:
    """Byte-identical copy of canonical_runtime._fetch_vec (mjSTATE_INTEGRATION)."""
    n = int(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
    vv = np.zeros(n, float)
    mujoco.mj_getState(model, data, vv, mujoco.mjtState.mjSTATE_INTEGRATION)
    return np.ascontiguousarray(vv)


def physics_index_for_time(t: float, dt: float = PHYS_DT, n: int = N_PHYS) -> int:
    """Nearest physics sample index for wall time t (sample times (i+1)*dt)."""
    i = int(round(float(t) / dt)) - 1
    return max(0, min(n - 1, i))


def build_frame_map(fps: int = FPS, t_end: float = T_END,
                    dt: float = PHYS_DT, n_phys: int = N_PHYS) -> list[dict]:
    """Deterministic nearest-sample frame map for the full episode.

    Frame j renders at t_frame=j/fps; source is the nearest physics sample.
    Last frame j=floor(t_end*fps). Returns per-frame provenance records.
    """
    j_last = int(math_floor(t_end * fps))
    out = []
    for j in range(j_last + 1):
        t_f = j / fps
        i = physics_index_for_time(t_f, dt, n_phys)
        t_s = (i + 1) * dt
        out.append({"frame": j, "t_frame": t_f, "sample": i,
                    "t_sample": t_s, "err_s": t_s - t_f})
    return out


def math_floor(x: float) -> int:
    import math as _m
    return _m.floor(x)


def control_starts(substeps: np.ndarray) -> np.ndarray:
    """Start physics index of each control interval (starts[0]=0)."""
    s = np.zeros(len(substeps) + 1, dtype=np.int64)
    s[1:] = np.cumsum(np.asarray(substeps, dtype=np.int64))
    return s[:-1]


def control_index_for_sample(i: int, starts: np.ndarray) -> int:
    return int(bisect.bisect_right(list(starts), int(i)) - 1)


def boundary_state_index(control_k: int, substeps: np.ndarray) -> int:
    """Post-step physics index holding the control-k boundary state (-1=reset)."""
    if control_k <= 0:
        return -1
    return int(np.sum(np.asarray(substeps[:control_k], dtype=np.int64))) - 1


def ass_escape(s: str) -> str:
    return s.replace("\\", r"\\").replace("{", r"\{").replace("}", r"\}")


def ass_timestamp(t: float) -> str:
    t = max(0.0, float(t))
    h = int(t // 3600)
    m = int((t % 3600) // 60)
    s = t % 60
    return f"{h}:{m:02d}:{s:05.2f}"


def check_gate(name: str, ok: bool, detail: str = "") -> dict:
    return {"gate": name, "pass": bool(ok), "detail": str(detail)}


def fail_closed(msg: str) -> "NoReturn":
    from typing import NoReturn  # local import to keep module import-light
    print(f"{FAIL}: {msg}", file=sys.stderr)
    raise SystemExit(2)


# --------------------------------------------------------------------------
# Authority loading.
# --------------------------------------------------------------------------

def load_authority(bundle_dir: Path) -> dict:
    """Load accepted bundle artifacts + cross-check mission literals."""
    bundle_dir = Path(bundle_dir)
    result = json.loads((bundle_dir / "V2.1-R001_CANONICAL_RESULT.json").read_text())
    if result.get("CANDIDATE_ID") != CANDIDATE_ID:
        fail_closed(f"bundle candidate {result.get('CANDIDATE_ID')!r} != {CANDIDATE_ID!r}")
    if result.get("TRACE_SHA256") != TRACE_SHA256:
        fail_closed("bundle TRACE_SHA256 disagrees with mission literal")
    if result.get("CANONICAL_CHECKPOINTS") != CANONICAL_CHECKPOINTS:
        fail_closed("bundle CANONICAL_CHECKPOINTS disagree with mission literals")
    for k, v in (("N_CTRL", N_CTRL), ("N_PHYS", N_PHYS), ("T_END", T_END),
                 ("OUTCOME", OUTCOME), ("TERMINATION", TERMINATION)):
        if result.get(k) != v:
            fail_closed(f"bundle {k}={result.get(k)!r} != mission {v!r}")
    sched = np.load(bundle_dir / "V2.1-R001_ACTION_SCHEDULE.npz")
    U = np.asarray(sched["action"], float)
    ctrl_t = np.asarray(sched["control_time"], float)
    substeps = np.asarray(sched["substeps"], dtype=np.int64)
    modes = [str(m) for m in list(sched["mode"])]
    if U.shape != (N_CTRL, 7):
        fail_closed(f"action schedule shape {U.shape} != ({N_CTRL}, 7)")
    if len(ctrl_t) != N_CTRL or len(substeps) != N_CTRL or len(modes) != N_CTRL:
        fail_closed("control schedule length mismatch "
                    f"t={len(ctrl_t)} sub={len(substeps)} mode={len(modes)}")
    if int(np.sum(substeps)) != N_PHYS:
        fail_closed(f"substep total {int(np.sum(substeps))} != {N_PHYS}")
    if not bool(np.all((U >= -1.0 - 1e-12) & (U <= 1.0 + 1e-12))):
        fail_closed("action schedule outside [-1, 1]")
    return {"result": result, "U": U, "ctrl_t": ctrl_t,
            "substeps": substeps, "modes": modes}


# --------------------------------------------------------------------------
# Deterministic replay extraction (frozen plant + accepted action schedule).
# --------------------------------------------------------------------------

def replay_extract(auth: dict, verbose: bool = False) -> dict:
    """Replay accepted actions through the frozen plant; capture full history.

    Applies the accepted action schedule with the accepted substep counts
    through a fresh canonical-reset V2Plant. Captures per-step qpos/qvel,
    time, control index, and the FULL mjSTATE_INTEGRATION vector (the TRACE
    and checkpoint hashes cover warmstart/ctrl bytes, which cannot be
    re-derived from qpos/qvel alone). No controller, no detector, no search.
    """
    U = auth["U"]
    substeps = auth["substeps"]
    plant = V2Plant()
    live = plant.make_data()
    plant.reset(live)
    mujoco.mj_forward(plant.model, live)
    reset_vec = fetch_integration_vector(plant.model, live)

    qpos_hist = np.zeros((N_PHYS, plant.model.nq), float)
    qvel_hist = np.zeros((N_PHYS, plant.model.nv), float)
    time_hist = np.zeros((N_PHYS,), float)
    cidx_hist = np.zeros((N_PHYS,), np.int32)
    # Full integration vectors (incl. solver warmstart/ctrl) are stored
    # because checkpoint/TRACE identity hashes cover them; re-deriving from
    # (qpos,qvel) alone cannot reproduce warmstart bytes.
    nvec = int(mujoco.mj_stateSize(
        plant.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    ivec_hist = np.zeros((N_PHYS, nvec), float)

    trace = hashlib.sha256()
    sim_t = 0.0
    w = 0
    for k in range(N_CTRL):
        plant.apply_action(live, U[k])
        for _ in range(int(substeps[k])):
            mujoco.mj_step(plant.model, live)
            sim_t += PHYS_DT
            ivec = fetch_integration_vector(plant.model, live)
            trace.update(np.ascontiguousarray(ivec).tobytes())
            ivec_hist[w] = ivec
            qpos_hist[w] = live.qpos
            qvel_hist[w] = live.qvel
            time_hist[w] = sim_t
            cidx_hist[w] = k
            w += 1
            if verbose and (w % 30000 == 0):
                print(f"[res79] replay {w}/{N_PHYS} t={sim_t:.4f}", flush=True)
    if w != N_PHYS:
        fail_closed(f"replay produced {w} physics steps != {N_PHYS}")
    return {"plant": plant, "qpos": qpos_hist, "qvel": qvel_hist,
            "time": time_hist, "cidx": cidx_hist, "ivec": ivec_hist,
            "reset_vec": reset_vec,
            "trace_sha": trace.hexdigest(), "sim_t": float(sim_t)}


def find_index_by_hash(ivec_hist: np.ndarray, reset_vec: np.ndarray,
                       target_sha: str, guess: int, window: int = 5) -> int:
    """Locate the exact sample index whose integration vector hashes to target."""
    if guess == -1:
        if vec_sha(reset_vec) == target_sha:
            return -1
        fail_closed(f"reset vector hash mismatch for {target_sha[:12]}")
    for i in range(max(0, guess - window), min(N_PHYS, guess + window + 1)):
        if vec_sha(ivec_hist[i]) == target_sha:
            return i
    fail_closed(f"checkpoint {target_sha[:16]}... not found near guess {guess}")


def verify_identity_gates(auth: dict, rep: dict) -> tuple[list[dict], dict]:
    """Verify TRACE/schedule/counts/termination/outcome + 7/7 checkpoints."""
    result = auth["result"]
    gates: list[dict] = []
    gates.append(check_gate("TRACE_SHA256",
                            rep["trace_sha"] == TRACE_SHA256, rep["trace_sha"]))
    gates.append(check_gate("N_CTRL", len(auth["substeps"]) == N_CTRL
                            and auth["U"].shape[0] == N_CTRL,
                            f"intervals={len(auth['substeps'])} actions={auth['U'].shape[0]}"))
    gates.append(check_gate("N_PHYS", int(np.sum(auth["substeps"])) == N_PHYS,
                            f"substep_total={int(np.sum(auth['substeps']))}"))
    gates.append(check_gate("T_END", rep["sim_t"] == T_END,
                            f"replay={rep['sim_t']!r} accepted={T_END!r}"))
    import collections as _co
    hist = dict(_co.Counter(int(x) for x in auth["substeps"]))
    gates.append(check_gate("SUBSTEP_SCHEDULE", hist == SUBSTEP_HISTOGRAM,
                            f"replay={hist} accepted={SUBSTEP_HISTOGRAM}"))
    gates.append(check_gate("OUTCOME", result.get("OUTCOME") == OUTCOME,
                            str(result.get("OUTCOME"))))
    gates.append(check_gate("TERMINATION", result.get("TERMINATION") == TERMINATION,
                            str(result.get("TERMINATION"))))
    umax = float(np.max(np.abs(auth["U"])))
    gates.append(check_gate("MAXUTIL", umax == MAXUTIL,
                            f"schedule_max={umax!r} accepted={MAXUTIL!r}"))

    ivec_hist = np.ascontiguousarray(rep["ivec"])
    ev = result["EVENTS"]
    ck_index: dict[str, int] = {}
    # S_APEX <- S_DET_apex confirmation sample.
    apex_idx = int(ev["apex"]["confirmed_sample_index"])
    ck_index["S_APEX"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_APEX"], apex_idx)
    gates.append(check_gate("CHECKPOINT_S_APEX", ck_index["S_APEX"] == apex_idx,
                            f"idx={ck_index['S_APEX']}"))
    # S_E10 <- E10 ctrl vector == impact_absorption confirmation sample.
    e10_idx = int(ev["impact_absorption"]["confirmed_sample_index"])
    ck_index["S_E10"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_E10"], e10_idx)
    gates.append(check_gate("CHECKPOINT_S_E10", ck_index["S_E10"] == e10_idx,
                            f"idx={ck_index['S_E10']}"))
    # S_E11 <- balance_capture confirmation sample.
    e11_idx = int(ev["balance_capture"]["confirmed_sample_index"])
    ck_index["S_E11"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_E11"], e11_idx)
    gates.append(check_gate("CHECKPOINT_S_E11", ck_index["S_E11"] == e11_idx,
                            f"idx={ck_index['S_E11']}"))
    # S_RR <- RR entry time; S_RR_CONFIRMED <- RR conf time.
    rr_entry_t = float(result["RR_ENTRY_T"])
    rr_conf_t = float(result["RR_CONF_T"])
    rr_guess = physics_index_for_time(rr_entry_t)
    rrc_guess = physics_index_for_time(rr_conf_t)
    ck_index["S_RR"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_RR"], rr_guess)
    ck_index["S_RR_CONFIRMED"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_RR_CONFIRMED"],
        rrc_guess)
    gates.append(check_gate("CHECKPOINT_S_RR",
                            ck_index["S_RR"] == rr_guess,
                            f"idx={ck_index['S_RR']} guess={rr_guess}"))
    gates.append(check_gate("CHECKPOINT_S_RR_CONFIRMED",
                            ck_index["S_RR_CONFIRMED"] == rrc_guess,
                            f"idx={ck_index['S_RR_CONFIRMED']} guess={rrc_guess}"))
    # S_STAND_HANDOFF <- control boundary state at HANDOFF_T.
    handoff_t = float(result["HANDOFF_T"])
    k_hand = int(np.argmin(np.abs(auth["ctrl_t"] - handoff_t)))
    hand_guess = boundary_state_index(k_hand, auth["substeps"])
    ck_index["S_STAND_HANDOFF"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_STAND_HANDOFF"],
        hand_guess)
    gates.append(check_gate(
        "CHECKPOINT_S_STAND_HANDOFF", ck_index["S_STAND_HANDOFF"] == hand_guess,
        f"idx={ck_index['S_STAND_HANDOFF']} guess={hand_guess} ctrl_k={k_hand} "
        f"dt_ctrl={abs(float(auth['ctrl_t'][k_hand]) - handoff_t):.3e}"))
    # S_E12 <- stable_recovery confirmation sample.
    e12_idx = int(ev["stable_recovery"]["confirmed_sample_index"])
    ck_index["S_E12"] = find_index_by_hash(
        ivec_hist, rep["reset_vec"], CANONICAL_CHECKPOINTS["S_E12"], e12_idx)
    gates.append(check_gate("CHECKPOINT_S_E12", ck_index["S_E12"] == e12_idx,
                            f"idx={ck_index['S_E12']}"))
    return gates, ck_index


def recompute_events(auth: dict, rep: dict, verbose: bool = False) -> tuple[list[dict], dict]:
    """Recompute the 12-gate event chain observationally on replay states.

    Uses ONLY frozen measurement + frozen detector (no controller). Mirrors the
    canonical runtime's per-step sampling (event_sample with time_s=sim_t).
    """
    plant = V2Plant()
    ivec_hist = np.ascontiguousarray(rep["ivec"])
    scratch = plant.make_data()
    shadow = create_measurement_data(plant)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    det = V2EventDetector()
    det.reset()
    if verbose:
        print("[res79] detector recompute start", flush=True)
    for i in range(N_PHYS):
        # Bit-exact restore (incl. solver warmstart/ctrl) so the shadow
        # forward matches the canonical runtime's shadow forward exactly.
        mujoco.mj_setState(plant.model, scratch, np.ascontiguousarray(ivec_hist[i]), spec)
        sample = SynchronizedPhysicsSample.from_live_state(plant, scratch, shadow)
        ev1 = sample.event_sample()
        ev1 = dict(ev1)
        ev1["time_s"] = float(rep["time"][i])
        det.update(ev1)
        if verbose and ((i + 1) % 40000 == 0):
            print(f"[res79] detector {i + 1}/{N_PHYS}", flush=True)
    fin = det.finalize()
    got = {k: {"occurred_at": float(v.occurred_at),
               "confirmed_at": float(v.confirmed_at),
               "sample_index": int(v.sample_index),
               "confirmed_sample_index": int(v.confirmed_sample_index)}
           for k, v in fin.event_records.items()}
    want = auth["result"]["EVENTS"]
    gates: list[dict] = []
    all12 = set(got) == set(want) == set(EVENT_ORDER)
    gates.append(check_gate("EVENTS_12OF12", all12,
                            f"recomputed={sorted(got)} accepted={sorted(want)}"))
    exact = bool(got == want)
    gates.append(check_gate("EVENTS_EXACT", exact,
                            "all occ/conf times + sample indices identical"
                            if exact else "MISMATCH (see report)"))
    gates.append(check_gate("EVENT_TERMINATION",
                            str(fin.termination) == TERMINATION,
                            str(fin.termination)))
    return gates, got


# --------------------------------------------------------------------------
# Telemetry sampling (official synchronized path, read-only over history).
# --------------------------------------------------------------------------

def sample_telemetry(plant, auth: dict, rep: dict, indices: list[int]) -> dict[int, dict]:
    """Official SynchronizedPhysicsSample telemetry for selected samples.

    Uses a dedicated scratch MjData + shadow MjData. Never touches replay
    arrays except by read (callers mark them read-only first).
    """
    scratch = plant.make_data()
    shadow = create_measurement_data(plant)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    ivec_hist = np.ascontiguousarray(rep["ivec"])
    out: dict[int, dict] = {}
    for i in indices:
        k = int(rep["cidx"][i])
        mujoco.mj_setState(plant.model, scratch, np.ascontiguousarray(ivec_hist[i]), spec)
        s = SynchronizedPhysicsSample.from_live_state(plant, scratch, shadow)
        fz = float(s.whole_Fz_N)
        load_id = int(plant.idx.body["external_load"])
        torso_id = int(plant.idx.torso_body)
        load_xyz = np.asarray(shadow.xpos[load_id], float)
        torso_xyz = np.asarray(shadow.xpos[torso_id], float)
        out[i] = {
            "t": float(rep["time"][i]),
            "ctrl_k": k,
            "mode": str(auth["modes"][k]),
            "u_max": float(np.max(np.abs(auth["U"][k]))),
            "com_z": float(s.com_position_m[2]),
            "com_x": float(s.com_position_m[0]),
            "com_vz": float(s.com_velocity_mps[2]),
            "com_vx": float(s.com_velocity_mps[0]),
            "fz_N": fz,
            "fz_bw": fz / BW_N,
            "fzL_N": float(s.left_Fz_N),
            "fzR_N": float(s.right_Fz_N),
            "supL": bool(s.support_active[0]),
            "supR": bool(s.support_active[1]),
            "pen_m": float(s.max_penetration_m),
            "tilt_rad": float(s.trunk_tilt_rad),
            "pelvis_z": float(s.pelvis_position_world_m[2]),
            "pelvis_x": float(s.pelvis_position_world_m[0]),
            "load_z": float(load_xyz[2]),
            "load_x": float(load_xyz[0]),
            # Rotation-invariant weld check: torso->load distance must be
            # constant (XML welds external_load at 0.420 m; world-frame
            # offsets legitimately rotate with torso pitch).
            "torso_load_dist": float(np.linalg.norm(load_xyz - torso_xyz)),
        }
    return out


def last_confirmed_event(auth: dict, t: float) -> tuple[str, str, float]:
    """Latest canonical event confirmed at/before time t -> (tag, name, conf_t)."""
    ev = auth["result"]["EVENTS"]
    best = ("--", "none", float("nan"))
    for name in EVENT_ORDER:
        c = float(ev[name]["confirmed_at"])
        if c <= t + 1e-12:
            best = (EVENT_TAG[name], name, c)
    return best


def occurrence_at_sample(auth: dict, i: int) -> str | None:
    """Event tag occurring exactly at physics sample i (or None)."""
    ev = auth["result"]["EVENTS"]
    for name in EVENT_ORDER:
        if int(ev[name]["sample_index"]) == int(i):
            return EVENT_TAG[name] + " " + name
    return None


def telemetry_lines(auth: dict, i: int, tel: dict) -> list[str]:
    tag, name, conf_t = last_confirmed_event(auth, tel["t"])
    occ = occurrence_at_sample(auth, i)
    if np.isfinite(conf_t):
        last_line = f"LAST_EVENT: {tag} {name} (conf {conf_t:.6f} s)"
    else:
        last_line = "LAST_EVENT: none yet"
    lines = [
        f"t={tel['t']:.6f} s   {CANDIDATE_ID}   MODE={tel['mode']}",
        last_line,
        f"COM z={tel['com_z']:.4f} m  vz={tel['com_vz']:.4f} m/s",
        f"Fz={tel['fz_N']:.1f} N = {tel['fz_bw']:.4f} BW "
        f"(L={tel['fzL_N']:.1f} N R={tel['fzR_N']:.1f} N)",
        f"SUPPORT(inst,Fz>10N): L={'ACTIVE' if tel['supL'] else '-----'} "
        f"R={'ACTIVE' if tel['supR'] else '-----'}",
        f"|u|max={tel['u_max']:.4f} (session max {MAXUTIL:.4f})",
    ]
    if occ is not None:
        lines.append(f">>> {occ} OCCURS <<<")
    return lines


def _ass_head(title: str) -> str:
    return ("[Script Info]\nTitle: " + title + "\nScriptType: v4.00+\n"
            f"PlayResX: {WIDTH}\nPlayResY: {HEIGHT}\nScaledBorderAndShadow: yes\n\n"
            "[V4+ Styles]\nFormat: Name, Fontname, Fontsize, PrimaryColour, "
            "BackColour, Bold, Alignment, BorderStyle, Outline, Shadow, "
            "MarginL, MarginV\n"
            "Style: Telem,DejaVu Sans,18,&H00FFFFFF,&HCC000000,1,7,3,1,0,24,24\n\n"
            "[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, "
            "MarginR, MarginV, Effect, Text\n")


def build_ass(path: Path, auth: dict, frame_map: list[dict], telem: dict[int, dict]) -> None:
    """Write an ASS subtitle file with one telemetry block per video frame."""
    head = _ass_head("RES-79 smoke telemetry")
    rows = []
    n = len(frame_map)
    for j, fr in enumerate(frame_map):
        i = int(fr["sample"])
        tel = telem[i]
        t0 = j / FPS
        t1 = (j + 1) / FPS if j + 1 < n else t0 + 1.0 / FPS
        block = "\\N".join(ass_escape(s) for s in telemetry_lines(auth, i, tel))
        rows.append(f"Dialogue: 0,{ass_timestamp(t0)},{ass_timestamp(t1)},Telem,,0,0,0,,{block}")
    path.write_text(head + "\n".join(rows) + "\n")


# --------------------------------------------------------------------------
# Render model + cameras (visualization-only; physics file untouched).
# --------------------------------------------------------------------------

def setup_render_model(xml_rel: str, width: int, height: int) -> tuple:
    """Load a SEPARATE render model; override ONLY vis framebuffer size.

    The XML file on disk is never modified (SHA re-verified by caller).
    The returned model is never stepped -- mj_fwdPosition/render only.
    """
    xml_path = REPO / xml_rel
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    model.vis.global_.offwidth = int(width)
    model.vis.global_.offheight = int(height)
    return model, str(xml_path)


def make_camera(lookat, distance: float, azimuth: float, elevation: float):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.fixedcamid = -1
    cam.trackbodyid = -1
    cam.lookat[:] = [float(lookat[0]), float(lookat[1]), float(lookat[2])]
    cam.distance = float(distance)
    cam.azimuth = float(azimuth)
    cam.elevation = float(elevation)
    return cam


def fit_side_camera(auth: dict, rep: dict, telem: dict[int, dict],
                    frame_map: list[dict], aspect: float) -> dict:
    """Deterministic fixed side-camera fit from accepted-state extents."""
    xs = np.array([telem[int(fr["sample"])]["pelvis_x"] for fr in frame_map])
    zs = np.array([telem[int(fr["sample"])]["load_z"] for fr in frame_map])
    xmin, xmax = float(xs.min()), float(xs.max())
    zmax = float(zs.max())
    xmid = (xmin + xmax) / 2.0
    margin = 0.35
    fovy_half = np.deg2rad(22.5)
    zmid = (0.0 + zmax + margin) / 2.0
    half_h = max(zmid - 0.0, (zmax + margin) - zmid)
    d_h = half_h / np.tan(fovy_half)
    half_w = (xmax - xmin) / 2.0 + margin
    d_w = half_w / (np.tan(fovy_half) * aspect)
    dist = max(d_h, d_w) * 1.15
    return {"lookat": [xmid, 0.0, zmid], "distance": float(dist),
            "azimuth": 90.0, "elevation": 0.0,
            "x_range": [xmin, xmax], "zmax_load": zmax}


def make_renderer(render_model, width: int, height: int):
    """Create one persistent EGL renderer (callers reuse; close when done).

    NOTE: MuJoCo 3.8's classic Renderer exposes only scene-flag indices 0..10,
    so contact-point markers (mjVIS_CONTACTPOINT=14) are unavailable on this
    path. The forensic renders therefore use the default scene flags
    untouched; contact geometry itself (feet boxes, floor) is inspected
    directly, which is the mission-required content.
    """
    return mujoco.Renderer(render_model, height, width)


def render_with(renderer, render_model, camera, rep: dict,
                samples: list[int]) -> list[np.ndarray]:
    """Render selected physics samples through an existing renderer+camera."""
    viz = mujoco.MjData(render_model)
    frames = []
    for i in samples:
        viz.qpos[:] = np.asarray(rep["qpos"][i], float)
        mujoco.mj_fwdPosition(render_model, viz)
        renderer.update_scene(viz, camera=camera)
        frames.append(np.asarray(renderer.render()).copy())
    return frames


def render_samples_to_frames(render_model, width: int, height: int, cam,
                             rep: dict, samples: list[int]) -> list[np.ndarray]:
    """Render selected physics samples with a dedicated viz MjData.

    Viz data receives ONLY copies of recorded qpos (kinematics via
    mj_fwdPosition -- position-level only, no solver, no dynamics).
    No mj_step, no ctrl writes, no feedback into any physics object.
    Convenience wrapper (creates/closes its own renderer); batch callers
    should prefer make_renderer + render_with to limit EGL contexts.
    """
    renderer = make_renderer(render_model, width, height)
    try:
        return render_with(renderer, render_model, cam, rep, samples)
    finally:
        renderer.close()


def frame_containment(frame: np.ndarray, border: int = 3) -> dict:
    """Heuristic forensic check: athlete present + not clipped by L/R/top.

    The frozen scene has a pure-black clear color and a neutral-grey floor
    whose brightness falls off with distance, so absolute color references
    are unreliable. Instead, foreground = pixels that are neither near-black
    nor neutral-grey (athlete shells are bluish, bar is orange, feet green).
    """
    px = frame.astype(np.int16)
    mx = px.max(axis=2)
    mn = px.min(axis=2)
    near_black = mx < 24
    neutral = ((mx - mn) <= 18) & (mx >= 24)
    fg = ~(near_black | neutral)
    h, w = fg.shape
    bot = fg  # bottom handled by caller via touch_bottom (warn-only)
    _ = (h, w, bot)
    return {
        "fg_frac": float(fg.mean()),
        "touch_left": bool(fg[:, :border].any()),
        "touch_right": bool(fg[:, -border:].any()),
        "touch_top": bool(fg[:border, :].any()),
        "touch_bottom": bool(fg[-border:, :].any()),
    }


def floor_presence(frame: np.ndarray) -> float:
    """Fraction of neutral floor-like pixels in the bottom third of frame."""
    px = frame.astype(np.int16)
    bot = px[int(px.shape[0] * 2 / 3):, :, :]
    mx = bot.max(axis=2)
    mn = bot.min(axis=2)
    neutral = ((mx - mn) <= 22) & (mx >= 20)
    return float(neutral.mean())


def build_ass_single(path: Path, lines: list[str]) -> None:
    """One-dialogue ASS overlay for a forensic still (same style as video)."""
    block = "\\N".join(ass_escape(s) for s in lines)
    head = _ass_head("RES-79 still telemetry")
    path.write_text(head + f"Dialogue: 0,0:00:00.00,0:00:05.00,Telem,,0,0,0,,{block}\n")


def write_still(frame: np.ndarray, width: int, height: int, out_png: Path,
                overlay_lines: list[str] | None) -> None:
    # Overlay via single-frame ASS burn (same DejaVu/box style as the video
    # subtitles). Rationale: ffmpeg drawtext renders textfile newlines as
    # missing-glyph boxes; ASS \\N separators are clean. The human-readable
    # telemetry text is ALSO kept as a sibling .txt audit record.
    if overlay_lines:
        txt = out_png.with_suffix(".txt")
        txt.write_text("\n".join(overlay_lines) + "\n")
        ass = out_png.with_suffix(".ass")
        build_ass_single(ass, overlay_lines)
        vf = f"subtitles='{str(ass)}'"
    else:
        vf = None
    cmd = ["ffmpeg", "-y", "-v", "error",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
           "-i", "pipe:0"]
    if vf:
        cmd += ["-vf", vf]
    cmd += ["-frames:v", "1", str(out_png)]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert p.stdin is not None
    p.stdin.write(np.ascontiguousarray(frame, dtype=np.uint8).tobytes())
    _, err = p.communicate()
    if p.returncode != 0:
        fail_closed(f"ffmpeg still failed for {out_png.name}: {err.decode()[-2000:]}")


# --------------------------------------------------------------------------
# Pipeline orchestration.
# --------------------------------------------------------------------------

SCIENCE_FILES = [
    "src/loaded_cmj/v2/canonical_runtime.py",
    "src/loaded_cmj/v2/res72_integration.py",
    "src/loaded_cmj/v2/balance_capture.py",
    "src/loaded_cmj/v2/stable_recovery.py",
    "src/loaded_cmj/v2/terminal_capture.py",
    "src/loaded_cmj/v2/full_closure.py",
    "src/loaded_cmj/v2/plant.py",
    "src/loaded_cmj/v2/measurement.py",
    "src/loaded_cmj/v2/events.py",
    "src/loaded_cmj/v2/support_continuity.py",
    "src/loaded_cmj/v2/assets/v2_plant.xml",
    "CANONICAL_V2_CANDIDATE_SPEC.json",
    "CANONICAL_V2_RUNTIME_CONTRACT.md",
]
BUNDLE_FILES = [
    "V2.1-R001_CANONICAL_RESULT.json",
    "V2.1-R001_ACTION_SCHEDULE.npz",
    "V2.1-R001_ctrl_t.npy",
    "V2.1-R001_ctrl_u.npy",
    "V2.1-R001_ctrl_mode.npy",
    "V2.1-R001_ctrl_substeps.npy",
]


def snapshot_hashes(paths: list[Path]) -> dict[str, str]:
    return {str(p): sha_file(p) for p in paths}


def science_snapshot() -> dict[str, str]:
    return snapshot_hashes([REPO / f for f in SCIENCE_FILES])


def bundle_snapshot(bundle_dir: Path) -> dict[str, str]:
    return snapshot_hashes([Path(bundle_dir) / f for f in BUNDLE_FILES])


def history_hashes(rep: dict) -> dict[str, str]:
    return {k: sha_bytes(np.ascontiguousarray(rep[k]).tobytes())
            for k in ("qpos", "qvel", "time", "cidx", "ivec")}


def cmd_extract(bundle_dir: Path, out_dir: Path, verbose: bool = False) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pre_science = science_snapshot()
    pre_bundle = bundle_snapshot(bundle_dir)
    auth = load_authority(bundle_dir)
    rep = replay_extract(auth, verbose=verbose)
    gates_id, ck_index = verify_identity_gates(auth, rep)
    gates_ev, recomputed = recompute_events(auth, rep, verbose=verbose)
    gates = gates_id + gates_ev
    failed = [g for g in gates if not g["pass"]]
    hist_files = {}
    for k in ("qpos", "qvel", "time", "ivec"):
        p = out_dir / f"replay_{k}.npy"
        np.save(p, np.ascontiguousarray(rep[k]))
        hist_files[k] = str(p)
    p = out_dir / "replay_cidx.npy"
    np.save(p, np.ascontiguousarray(rep["cidx"]))
    hist_files["cidx"] = str(p)
    report = {
        "mission": "RES13A_ACCEPTED_TRAJECTORY_VISUAL_SMOKE_001",
        "candidate": CANDIDATE_ID,
        "trace_sha": rep["trace_sha"],
        "sim_t": rep["sim_t"],
        "gates": gates,
        "all_pass": not failed,
        "ck_index": ck_index,
        "recomputed_events": recomputed,
        "history_hashes": history_hashes(rep),
        "history_files": hist_files,
        "pre_science": pre_science,
        "pre_bundle": pre_bundle,
        "reset_vec_sha": vec_sha(rep["reset_vec"]),
    }
    (out_dir / "extraction_report.json").write_text(json.dumps(report, indent=2))
    if failed:
        fail_closed("extraction identity gates FAILED: "
                    + json.dumps(failed, indent=2))
    print(json.dumps({"extract": "PASS", "trace_sha": rep["trace_sha"],
                      "gates": len(gates)}, indent=2))
    return report


def load_extraction(out_dir: Path) -> tuple[dict, dict]:
    out_dir = Path(out_dir)
    report = json.loads((out_dir / "extraction_report.json").read_text())
    if not report.get("all_pass"):
        fail_closed("extraction report is not PASS; refusing to render")
    rep = {k: np.load(out_dir / f"replay_{k}.npy")
           for k in ("qpos", "qvel", "time", "cidx", "ivec")}
    for arr in rep.values():
        arr.flags.writeable = False
    return report, rep


def exact_still_samples(auth: dict, rep_extra: dict) -> dict[str, int]:
    """Exact authoritative physics samples for forensic stills (no approximation)."""
    ev = auth["result"]["EVENTS"]
    out = {f"{EVENT_TAG[n]}_{n}_occurrence": int(ev[n]["sample_index"])
           for n in EVENT_ORDER}
    out.update({
        "E9_descending_landing_confirmation": int(ev["descending_landing"]["confirmed_sample_index"]),
        "E10_impact_absorption_confirmation": int(ev["impact_absorption"]["confirmed_sample_index"]),
        "E11_balance_capture_confirmation": int(ev["balance_capture"]["confirmed_sample_index"]),
        "STAND_HANDOFF": int(rep_extra["ck_index"]["S_STAND_HANDOFF"]),
        "E12_stable_recovery_confirmation": int(ev["stable_recovery"]["confirmed_sample_index"]),
        "FINAL_POSTHOLD": N_PHYS - 1,
    })
    rr_entry_t = auth["result"].get("RR_ENTRY_T")
    if rr_entry_t is not None:
        out["RECOVERY_READY_ENTRY"] = physics_index_for_time(float(rr_entry_t))
    for k, v in out.items():
        if not (0 <= v < N_PHYS):
            fail_closed(f"still sample out of bounds: {k}={v}")
    return out


def peak_penetration_in_window(plant, auth: dict, rep: dict,
                               t0: float = 0.6, t1: float = 1.5) -> tuple[int, float]:
    i0 = physics_index_for_time(t0)
    i1 = physics_index_for_time(t1)
    tel = sample_telemetry(plant, auth, rep, list(range(i0, i1 + 1)))
    best = max(tel, key=lambda i: tel[i]["pen_m"])
    return best, float(tel[best]["pen_m"])


def ffprobe_stream(path: Path) -> dict:
    r = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                        "-show_entries",
                        "stream=width,height,avg_frame_rate,duration,nb_frames",
                        "-of", "json", str(path)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        fail_closed(f"ffprobe failed for {path.name}: {r.stderr[-1000:]}")
    streams = json.loads(r.stdout).get("streams", [])
    if not streams:
        fail_closed(f"ffprobe found no video stream in {path.name}")
    return streams[0]


def ffprobe_frame_count(path: Path) -> int:
    r = subprocess.run(["ffprobe", "-v", "error", "-count_frames",
                        "-select_streams", "v:0", "-show_entries",
                        "stream=nb_read_frames", "-of", "csv=p=0", str(path)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        fail_closed(f"ffprobe count_frames failed for {path.name}")
    try:
        return int(r.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        fail_closed(f"ffprobe unparseable frame count for {path.name}: {r.stdout[-500:]}")
    return -1


def cmd_render(bundle_dir: Path, out_dir: Path, report: dict, rep: dict,
               frozen_rerun_json: Path | None = None,
               verbose: bool = False) -> dict:
    out_dir = Path(out_dir)
    auth = load_authority(bundle_dir)
    plant = V2Plant()
    if verbose:
        print("[res79] history hashes re-verified pre-render", flush=True)
    if history_hashes(rep) != report["history_hashes"]:
        fail_closed("replay history mutated between extract and render")

    frame_map = build_frame_map()
    n_frames = len(frame_map)
    if verbose:
        print(f"[res79] frame map: {n_frames} frames @ {FPS}fps", flush=True)
    stills = exact_still_samples(auth, report)
    peak_idx, peak_pen = peak_penetration_in_window(plant, auth, rep)
    stills["PEAK_PENETRATION"] = peak_idx
    if verbose:
        print(f"[res79] peak penetration sample={peak_idx} pen={peak_pen:.6f} m",
              flush=True)
    telem_idx = sorted(set([int(fr["sample"]) for fr in frame_map]
                           + list(stills.values())))
    telem = sample_telemetry(plant, auth, rep, telem_idx)
    for i, t in telem.items():
        for k in ("com_z", "com_vz", "fz_N", "fz_bw", "fzL_N", "fzR_N",
                  "pen_m", "tilt_rad", "pelvis_z", "load_z"):
            if not np.isfinite(t[k]):
                fail_closed(f"non-finite telemetry {k} at sample {i}")

    # Bar/load coupling: torso<->load DISTANCE must be constant (weld per XML
    # at 0.420 m). World-frame offsets are NOT constant (torso pitches).
    dd = np.array([telem[i]["torso_load_dist"] for i in sorted(telem)])
    bar_couple = {"dist_mean_m": float(dd.mean()),
                  "dist_std_m": float(dd.std()),
                  "dist_min_m": float(dd.min()),
                  "dist_max_m": float(dd.max())}

    # Cameras (fixed; deterministic fit from accepted-state extents).
    aspect = WIDTH / HEIGHT
    side = fit_side_camera(auth, rep, telem, frame_map, aspect)
    secondary = dict(side)
    secondary.update({"azimuth": 140.0, "elevation": 10.0})
    land_tel = telem[physics_index_for_time(1.0)]
    closeup = {"lookat": [float(land_tel["pelvis_x"]), 0.0, 0.14],
               "distance": 1.15, "azimuth": 90.0, "elevation": 0.0}
    cameras = {"side": side, "secondary": secondary, "closeup": closeup}

    # Render model: separate instance, vis-only framebuffer override.
    render_model, xml_used = setup_render_model(
        "src/loaded_cmj/v2/assets/v2_plant.xml", WIDTH, HEIGHT)
    if sha_file(xml_used) != PLANT_SHA256:
        fail_closed("render XML file hash changed; refusing to render")
    geom_names = set()
    for gi in range(render_model.ngeom):
        nm = mujoco.mj_id2name(render_model, mujoco.mjtObj.mjOBJ_GEOM, gi)
        geom_names.add(nm or f"#{gi}")
    sci_model = plant.model
    if render_model.ngeom != sci_model.ngeom:
        fail_closed("render/scientific geom inventory mismatch")

    # Reference colors are derived inside frame_containment/floor_presence
    # (neutral-grey rule); probe one frame to fail fast on render errors.
    cam_side = make_camera(**{k: side[k] for k in
                              ("lookat", "distance", "azimuth", "elevation")})
    _probe = render_samples_to_frames(render_model, WIDTH, HEIGHT, cam_side,
                                      rep, [0])[0]
    del _probe

    side_samples = [int(fr["sample"]) for fr in frame_map]
    sec_samples = list(side_samples)
    closeup_frames_map = [fr for fr in frame_map
                          if 0.85 <= fr["t_frame"] <= 1.15 + 1e-12]
    closeup_samples = [int(fr["sample"]) for fr in closeup_frames_map]

    ass_path = out_dir / "smoke_telemetry.ass"
    build_ass(ass_path, auth, frame_map, telem)
    n_close = len(closeup_frames_map)
    ass_close = out_dir / "smoke_telemetry_closeup.ass"
    build_ass_subset(ass_close, auth, closeup_frames_map, telem, n_close)

    media: dict[str, str] = {}
    # Two persistent EGL renderers shared by every video and still below,
    # to limit GL context churn. Cameras vary per update_scene call;
    # renderers are closed at the end of cmd_render.
    renderer_plain = make_renderer(render_model, WIDTH, HEIGHT)
    renderer_contact = make_renderer(render_model, WIDTH, HEIGHT)

    def render_encode(tag: str, renderer, cam: dict, samples: list[int],
                      ass: Path | None, out_name: str,
                      containment: bool) -> tuple[Path, dict]:
        if verbose:
            print(f"[res79] render+encode {tag}: {len(samples)} frames",
                  flush=True)
        c = make_camera(**{k: cam[k] for k in
                           ("lookat", "distance", "azimuth", "elevation")})
        viz = mujoco.MjData(render_model)
        out_mp4 = out_dir / out_name
        vf = f"subtitles='{str(ass)}'" if ass is not None else None
        cmd = ["ffmpeg", "-y", "-v", "error",
               "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{WIDTH}x{HEIGHT}", "-framerate", str(FPS),
               "-i", "pipe:0"]
        if vf:
            cmd += ["-vf", vf]
        cmd += ["-c:v", "libx264", "-crf", "18", "-preset", "medium",
                "-pix_fmt", "yuv420p", "-r", str(FPS), str(out_mp4)]
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert p.stdin is not None
        stats = {"n": 0, "fg_min": 1.0, "fg_med": 0.0, "fg_vals": [],
                 "clip_lr_top": 0, "clip_bottom": 0, "floor_min": 1.0}
        for i in samples:
            viz.qpos[:] = np.asarray(rep["qpos"][i], float)
            mujoco.mj_fwdPosition(render_model, viz)
            renderer.update_scene(viz, camera=c)
            fr = np.asarray(renderer.render()).copy()
            if not bool(np.isfinite(fr).all()):
                p.kill()
                fail_closed(f"non-finite rendered frame at sample {i}")
            if containment:
                cc = frame_containment(fr)
                stats["n"] += 1
                stats["fg_vals"].append(cc["fg_frac"])
                stats["fg_min"] = min(stats["fg_min"], cc["fg_frac"])
                if cc["touch_left"] or cc["touch_right"] or cc["touch_top"]:
                    stats["clip_lr_top"] += 1
                if cc["touch_bottom"]:
                    stats["clip_bottom"] += 1
                stats["floor_min"] = min(stats["floor_min"],
                                         floor_presence(fr))
            try:
                p.stdin.write(np.ascontiguousarray(fr, dtype=np.uint8).tobytes())
            except BrokenPipeError:
                _, err = p.communicate()
                fail_closed(f"ffmpeg pipe broke for {out_name}: "
                            f"{err.decode()[-2000:]}")
        _, err = p.communicate()
        if p.returncode != 0:
            fail_closed(f"ffmpeg encode failed for {out_name}: "
                        f"{err.decode()[-2000:]}")
        return out_mp4, stats

    try:
        side_mp4, side_stats = render_encode(
            "side", renderer_plain, side, side_samples, ass_path,
            "FULL_SMOKE_SIDE.mp4", containment=True)
        media["FULL_SMOKE_SIDE.mp4"] = str(side_mp4)
        sec_mp4, sec_stats = render_encode(
            "secondary", renderer_plain, secondary, sec_samples, ass_path,
            "FULL_SMOKE_SECONDARY.mp4", containment=True)
        media["FULL_SMOKE_SECONDARY.mp4"] = str(sec_mp4)
        close_mp4, _ = render_encode(
            "closeup", renderer_contact, closeup, closeup_samples, ass_close,
            "LANDING_CLOSEUP.mp4", containment=False)
        media["LANDING_CLOSEUP.mp4"] = str(close_mp4)

        # Event + landing stills at EXACT samples (side camera).
        still_cam = make_camera(**{k: side[k] for k in
                                   ("lookat", "distance", "azimuth",
                                    "elevation")})
        still_dir = out_dir / "EVENT_STILLS"
        still_dir.mkdir(exist_ok=True)
        land_dir = out_dir / "LANDING_CONTACT_STILLS"
        land_dir.mkdir(exist_ok=True)
        event_still_paths: dict[str, str] = {}
        for n in EVENT_ORDER:
            key = f"{EVENT_TAG[n]}_{n}_occurrence"
            i = stills[key]
            fr = render_with(renderer_contact, render_model, still_cam,
                             rep, [i])[0]
            name = f"{EVENT_TAG[n]}_{n}.png"
            write_still(fr, WIDTH, HEIGHT, still_dir / name,
                        telemetry_lines(auth, i, telem[i]))
            event_still_paths[n] = str(still_dir / name)
        landing_spec = [
            ("PRE_LANDING.png", stills["E9_descending_landing_occurrence"] - 40),
            ("LANDING_OCCURRENCE.png", stills["E9_descending_landing_occurrence"]),
            ("E9_CONFIRMATION.png", stills["E9_descending_landing_confirmation"]),
            ("PEAK_PENETRATION.png", stills["PEAK_PENETRATION"]),
            ("E10_OCCURRENCE.png", stills["E10_impact_absorption_occurrence"]),
            ("E10_CONFIRMATION.png", stills["E10_impact_absorption_confirmation"]),
            ("E11_OCCURRENCE.png", stills["E11_balance_capture_occurrence"]),
            ("E11_CONFIRMATION.png", stills["E11_balance_capture_confirmation"]),
        ]
        landing_still_paths: dict[str, str] = {}
        landing_still_samples: dict[str, int] = {}
        for name, i in landing_spec:
            if not (0 <= i < N_PHYS):
                fail_closed(f"landing still sample out of bounds: {name}={i}")
            if i not in telem:
                extra = sample_telemetry(plant, auth, rep, [i])
                telem.update(extra)
            fr = render_with(renderer_contact, render_model, still_cam,
                             rep, [i])[0]
            write_still(fr, WIDTH, HEIGHT, land_dir / name,
                        telemetry_lines(auth, i, telem[i]))
            landing_still_paths[name] = str(land_dir / name)
            landing_still_samples[name] = int(i)
    finally:
        renderer_plain.close()
        renderer_contact.close()

    # Post-render: history + science must be unchanged.
    post_hist = history_hashes(rep)
    post_science = science_snapshot()
    render_context = {
        "frame_map": frame_map,
        "n_frames": n_frames,
        "stills": stills,
        "peak_penetration": {"sample": peak_idx, "pen_m": peak_pen},
        "cameras": cameras,
        "media": media,
        "event_still_paths": event_still_paths,
        "landing_still_paths": landing_still_paths,
        "landing_still_samples": landing_still_samples,
        "ass_path": str(ass_path),
        "ass_closeup_path": str(ass_close),
        "bar_coupling": bar_couple,
        "side_containment": {k: (sorted(v) if k == "fg_vals" else v)
                             for k, v in side_stats.items()},
        "secondary_containment": {k: (sorted(v) if k == "fg_vals" else v)
                                  for k, v in sec_stats.items()},
        "geom_names": sorted(geom_names),
        "post_history_hashes": post_hist,
        "post_science": post_science,
        "history_intact": post_hist == report["history_hashes"],
        "science_unchanged": post_science == report["pre_science"],
        "closeup_frame_map": closeup_frames_map,
    }
    if not render_context["history_intact"]:
        fail_closed("replay history mutated during render")
    if not render_context["science_unchanged"]:
        fail_closed("tracked science files changed during render")
    return {"auth": auth, "telem": telem, "render": render_context}


def build_ass_subset(path: Path, auth: dict, frames: list[dict],
                     telem: dict[int, dict], n: int) -> None:
    head = _ass_head("RES-79 landing closeup telemetry")
    rows = []
    for j, fr in enumerate(frames):
        i = int(fr["sample"])
        t0 = j / FPS
        t1 = (j + 1) / FPS
        block = "\\N".join(ass_escape(s)
                           for s in telemetry_lines(auth, i, telem[i]))
        rows.append(f"Dialogue: 0,{ass_timestamp(t0)},{ass_timestamp(t1)},"
                    f"Telem,,0,0,0,,{block}")
    path.write_text(head + "\n".join(rows) + "\n")


# --------------------------------------------------------------------------
# Provenance writers.

def git_head_tree() -> tuple[str, str]:
    head = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                          text=True, cwd=str(REPO))
    tree = subprocess.run(["git", "rev-parse", "HEAD^{tree}"],
                          capture_output=True, text=True, cwd=str(REPO))
    if head.returncode != 0 or tree.returncode != 0:
        fail_closed("git HEAD/tree query failed")
    return head.stdout.strip(), tree.stdout.strip()


def env_versions() -> dict:
    import platform
    import scipy
    return {"python": platform.python_version(),
            "mujoco": mujoco.__version__,
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "arch": f"{platform.machine()} {platform.system()}",
            "physics_dt": PHYS_DT}


def ffmpeg_version() -> str:
    r = subprocess.run(["ffmpeg", "-hide_banner", "-version"],
                       capture_output=True, text=True)
    return (r.stdout.strip().splitlines() or ["unknown"])[0]


def write_event_frame_index(out_dir: Path, auth: dict, render: dict) -> Path:
    ev = auth["result"]["EVENTS"]
    frame_map = render["frame_map"]
    frame_samples = [int(fr["sample"]) for fr in frame_map]

    def nearest_frame(sample_idx: int) -> dict:
        j = int(np.argmin([abs(s - sample_idx) for s in frame_samples]))
        fr = frame_map[j]
        occ_t = (sample_idx + 1) * PHYS_DT
        return {"video_frame_index": j,
                "video_source_physics_index": int(fr["sample"]),
                "video_source_time_s": float(fr["t_sample"]),
                "video_time_error_s": float(fr["t_sample"] - occ_t)}

    events = {}
    for n in EVENT_ORDER:
        e = ev[n]
        occ = int(e["sample_index"])
        entry = {"event_name": n,
                 "event_tag": EVENT_TAG[n],
                 "occurred_time_s": float(e["occurred_at"]),
                 "occurred_physics_index": occ,
                 "confirmed_time_s": float(e["confirmed_at"]),
                 "confirmed_physics_index": int(e["confirmed_sample_index"])}
        entry.update(nearest_frame(occ))
        entry["exact_still_path"] = render["event_still_paths"][n]
        entry["exact_still_sample"] = occ
        events[n] = entry
    stills = render["stills"]
    special = {
        "APEX": {"sample": stills["E8_apex_occurrence"]},
        "LANDING": {"sample": stills["E9_descending_landing_occurrence"]},
        "PEAK_PENETRATION": {"sample": stills["PEAK_PENETRATION"],
                             "pen_m": render["peak_penetration"]["pen_m"]},
        "E10": {"occurrence_sample": stills["E10_impact_absorption_occurrence"],
                "confirmation_sample": stills["E10_impact_absorption_confirmation"]},
        "E11": {"occurrence_sample": stills["E11_balance_capture_occurrence"],
                "confirmation_sample": stills["E11_balance_capture_confirmation"]},
        "RECOVERY_READY": {"sample": stills.get("RECOVERY_READY_ENTRY")},
        "STAND_HANDOFF": {"sample": stills["STAND_HANDOFF"],
                          "handoff_t": float(auth["result"]["HANDOFF_T"])},
        "E12": {"occurrence_sample": stills["E12_stable_recovery_occurrence"],
                "confirmation_sample": stills["E12_stable_recovery_confirmation"]},
        "FINAL_POSTHOLD": {"sample": stills["FINAL_POSTHOLD"],
                           "t_end": T_END},
        "LANDING_STILLS": dict(render["landing_still_samples"]),
    }
    for _k, v in special.items():
        for sk, sv in list(v.items()):
            if sk == "sample" and sv is not None:
                v["video_frame"] = nearest_frame(int(sv))
    doc = {"candidate": CANDIDATE_ID, "fps": FPS,
           "frame_selection_rule":
               "frame j at t=j/fps; source=nearest physics sample "
               "(sample times (i+1)*DT); no interpolation",
           "n_frames": render["n_frames"],
           "full_frame_map": render["frame_map"],
           "events": events,
           "special": special}
    p = out_dir / "EVENT_FRAME_INDEX.json"
    p.write_text(json.dumps(doc, indent=2))
    return p


def media_file_list(render: dict) -> list[Path]:
    files = [Path(render["media"][k]) for k in
             ("FULL_SMOKE_SIDE.mp4", "FULL_SMOKE_SECONDARY.mp4",
              "LANDING_CLOSEUP.mp4")]
    files += [Path(p) for p in render["event_still_paths"].values()]
    files += [Path(p) for p in render["landing_still_paths"].values()]
    files += [Path(render["ass_path"]), Path(render["ass_closeup_path"])]
    return files


def run_media_checks(out_dir: Path, render: dict) -> list[dict]:
    checks: list[dict] = []
    specs = [("FULL_SMOKE_SIDE.mp4", render["n_frames"]),
             ("FULL_SMOKE_SECONDARY.mp4", render["n_frames"]),
             ("LANDING_CLOSEUP.mp4", len(render["closeup_frame_map"]))]
    for name, want_frames in specs:
        p = Path(render["media"][name])
        checks.append(check_gate(f"MEDIA_EXISTS_{name}", p.exists()
                                 and p.stat().st_size > 0,
                                 f"bytes={p.stat().st_size if p.exists() else -1}"))
        st = ffprobe_stream(p)
        try:
            num, den = st.get("avg_frame_rate", "0/1").split("/")
            fps = float(num) / float(den) if float(den) else 0.0
        except (ValueError, ZeroDivisionError):
            fps = 0.0
        checks.append(check_gate(f"MEDIA_RES_{name}",
                                 int(st.get("width", -1)) == WIDTH
                                 and int(st.get("height", -1)) == HEIGHT,
                                 f"{st.get('width')}x{st.get('height')}"))
        checks.append(check_gate(f"MEDIA_FPS_{name}", abs(fps - FPS) < 0.01,
                                 f"avg_frame_rate={st.get('avg_frame_rate')}"))
        nread = ffprobe_frame_count(p)
        checks.append(check_gate(f"MEDIA_FRAMES_{name}", nread == want_frames,
                                 f"decoded={nread} declared={want_frames}"))
        try:
            dur = float(st.get("duration", "nan"))
        except (TypeError, ValueError):
            dur = float("nan")
        if name == "LANDING_CLOSEUP.mp4":
            want_dur = len(render["closeup_frame_map"]) / FPS
        else:
            want_dur = T_END
        checks.append(check_gate(
            f"MEDIA_DURATION_{name}",
            bool(np.isfinite(dur)) and abs(dur - want_dur) < 0.15,
            f"duration={dur:.4f}s expected~{want_dur:.4f}s"))
    for n, p in render["event_still_paths"].items():
        pp = Path(p)
        ok = pp.exists() and pp.stat().st_size > 0
        if ok:
            r = subprocess.run(["ffmpeg", "-v", "error", "-i", str(pp),
                                "-f", "null", "-"],
                               capture_output=True, text=True)
            ok = r.returncode == 0
        checks.append(check_gate(f"STILL_EVENT_{n}", ok,
                                 f"bytes={pp.stat().st_size if pp.exists() else -1}"))
    for name, p in render["landing_still_paths"].items():
        pp = Path(p)
        ok = pp.exists() and pp.stat().st_size > 0
        if ok:
            r = subprocess.run(["ffmpeg", "-v", "error", "-i", str(pp),
                                "-f", "null", "-"],
                               capture_output=True, text=True)
            ok = r.returncode == 0
        checks.append(check_gate(f"STILL_LANDING_{name}", ok,
                                 f"bytes={pp.stat().st_size if pp.exists() else -1}"))
    fm = render["frame_map"]
    seq = [int(fr["sample"]) for fr in fm]
    checks.append(check_gate(
        "FRAME_INDEX_BOUNDS",
        all(0 <= s < N_PHYS for s in seq)
        and all(b >= a for a, b in zip(seq, seq[1:])),
        f"n={len(fm)} monotonic_non_decreasing in-bounds"))
    sc = render["side_containment"]
    fg_vals = sorted(sc.get("fg_vals", []))
    med = float(np.median(fg_vals)) if fg_vals else 0.0
    checks.append(check_gate("ATHLETE_VISIBLE_SIDE",
                             med > 0.0005 and sc.get("clip_lr_top", 1) == 0,
                             f"median_fg_frac={med:.4f} "
                             f"clipped_lr_top_frames={sc.get('clip_lr_top')} "
                             f"bottom_touch_frames={sc.get('clip_bottom')} "
                             f"floor_frac_min={sc.get('floor_min', 0.0):.3f}"))
    sc2 = render.get("secondary_containment", {})
    fg2 = sorted(sc2.get("fg_vals", []))
    med2 = float(np.median(fg2)) if fg2 else 0.0
    checks.append(check_gate("ATHLETE_VISIBLE_SECONDARY",
                             med2 > 0.0005 and sc2.get("clip_lr_top", 1) == 0,
                             f"median_fg_frac={med2:.4f} "
                             f"clipped_lr_top_frames={sc2.get('clip_lr_top')} "
                             f"bottom_touch_frames={sc2.get('clip_bottom')} "
                             f"floor_frac_min={sc2.get('floor_min', 0.0):.3f}"))
    checks.append(check_gate(
        "BAR_GEOMETRY_PRESENT",
        "load_shell" in set(render["geom_names"]),
        f"ngeom_render={len(render['geom_names'])}"))
    checks.append(check_gate(
        "BAR_COUPLING_RIGID",
        abs(render["bar_coupling"]["dist_mean_m"] - 0.42) < 1e-9
        and render["bar_coupling"]["dist_std_m"] < 1e-9,
        f"dist_mean={render['bar_coupling']['dist_mean_m']:.6f} m "
        f"dist_std={render['bar_coupling']['dist_std_m']:.3e} m "
        f"(XML weld offset 0.420 m; rotation-invariant)"))
    return checks


def write_metadata(out_dir: Path, auth: dict, report: dict, render: dict,
                   media_checks: list[dict], event_index_path: Path,
                   frozen_gates: list[dict]) -> tuple[Path, dict[str, str]]:
    head, tree = git_head_tree()
    media_hashes = {Path(p).name: sha_file(p)
                    for p in media_file_list(render)}
    meta = {
        "mission": "RES13A_ACCEPTED_TRAJECTORY_VISUAL_SMOKE_001",
        "linear_issue": "RES-79",
        "entry_head": ENTRY_HEAD, "entry_tree": ENTRY_TREE,
        "final_head_at_render": head, "final_tree_at_render": tree,
        "candidate_id": CANDIDATE_ID,
        "trace_sha256": TRACE_SHA256,
        "plant_sha256": PLANT_SHA256,
        "controller_composition_sha256": CONTROLLER_COMPOSITION_SHA256,
        "scorer_sha256": SCORER_SHA256,
        "candidate_spec_sha256": CANDIDATE_SPEC_SHA256,
        "runtime_contract_sha256": RUNTIME_CONTRACT_SHA256,
        "environment": env_versions(),
        "render_backend": "mujoco.Renderer (classic) with MUJOCO_GL=egl",
        "ffmpeg_version": ffmpeg_version(),
        "video_codec": "libx264 crf18 preset=medium pix_fmt=yuv420p",
        "fps": FPS, "resolution": [WIDTH, HEIGHT],
        "frame_count": render["n_frames"],
        "frame_selection_rule":
            "frame j at t=j/fps; source=nearest physics sample "
            "((i+1)*DT); documented per-frame in EVENT_FRAME_INDEX.json; "
            "no interpolation of qpos/qvel",
        "cameras": render["cameras"],
        "framebuffer_override":
            "frozen XML file byte-identical (PLANT_SHA256 re-verified); "
            "in-memory vis.global offwidth/offheight set to "
            f"{WIDTH}x{HEIGHT} on a SEPARATE never-stepped render MjModel; "
            "physics replay used an unmodified model instance",
        "accepted_schedule_hashes": dict(report.get("pre_bundle", {})),
        "extraction_gates": report["gates"],
        "frozen_rerun_gates": frozen_gates,
        "media_checks": media_checks,
        "render_influence_on_physics": False,
        "media": {k: {"path": v, "sha256": media_hashes.get(Path(v).name)}
                  for k, v in render["media"].items()},
        "event_stills": {k: {"path": v, "sha256": media_hashes.get(Path(v).name)}
                         for k, v in render["event_still_paths"].items()},
        "landing_stills": {k: {"path": v, "sha256": media_hashes.get(Path(v).name)}
                           for k, v in render["landing_still_paths"].items()},
        "event_frame_index": str(event_index_path),
        "bar_coupling": render["bar_coupling"],
        "peak_penetration": render["peak_penetration"],
        "history_hashes": report["history_hashes"],
        "post_history_hashes": render["post_history_hashes"],
        "science_unchanged": render["science_unchanged"],
    }
    meta_path = out_dir / "SMOKE_RENDER_METADATA.json"
    meta_path.write_text(json.dumps(meta, indent=2))
    media_hashes["SMOKE_RENDER_METADATA.json"] = sha_file(meta_path)
    media_hashes[event_index_path.name] = sha_file(event_index_path)
    return meta_path, media_hashes


def write_media_sha(out_dir: Path, media_hashes: dict[str, str],
                    extra_paths: list[Path]) -> Path:
    lines = []
    for name in sorted(media_hashes):
        lines.append(f"{media_hashes[name]}  {name}")
    for p in extra_paths:
        lines.append(f"{sha_file(p)}  {p.name}")
    sp = out_dir / "MEDIA_SHA256.txt"
    sp.write_text("\n".join(lines) + "\n")
    return sp


def write_integrity_report(out_dir: Path, report: dict, render: dict,
                           media_checks: list[dict],
                           frozen_gates: list[dict]) -> Path:
    def fmt_gates(gs: list[dict]) -> str:
        return "\n".join(f"- [{'PASS' if g['pass'] else 'FAIL'}] {g['gate']}: "
                         f"{g['detail']}" for g in gs)

    def verdict(gs: list[dict], pred) -> str:
        sel = [g for g in gs if pred(g)]
        return "PASS" if sel and all(g["pass"] for g in sel) else "FAIL"

    p = out_dir / "RENDER_INTEGRITY_REPORT.md"
    p.write_text(
        "# RES-79 render integrity report\n"
        "\n"
        "MISSION=RES13A_ACCEPTED_TRAJECTORY_VISUAL_SMOKE_001\n"
        f"CANDIDATE_ID={CANDIDATE_ID}\n"
        "\n"
        "## Identity verdicts\n"
        "\n"
        f"TRACE_IDENTITY={verdict(report['gates'], lambda g: 'TRACE' in g['gate'])}\n"
        "EVENT_IDENTITY="
        f"{verdict(report['gates'], lambda g: g['gate'].startswith('EVENT'))}\n"
        "CHECKPOINT_IDENTITY="
        f"{verdict(report['gates'], lambda g: g['gate'].startswith('CHECKPOINT'))}\n"
        "SCHEDULE_IDENTITY="
        f"{verdict(report['gates'], lambda g: 'SUBSTEP' in g['gate'] or g['gate'] in ('N_CTRL', 'N_PHYS', 'T_END', 'OUTCOME', 'TERMINATION', 'MAXUTIL'))}\n"
        "RENDER_INFLUENCE_ON_PHYSICS=false\n"
        "\n"
        "## Extraction identity gates (accepted-action replay vs RES-12 authority)\n"
        "\n"
        f"{fmt_gates(report['gates'])}\n"
        "\n"
        "## Frozen rerun cross-check gates\n"
        "\n"
        f"{fmt_gates(frozen_gates) if frozen_gates else 'not supplied (see Linear comment for FROZEN_RERUN_STATUS)'}\n"
        "\n"
        "## Automated media checks (render-only, no biomechanics thresholds)\n"
        "\n"
        f"{fmt_gates(media_checks)}\n"
        "\n"
        "## Scientific/render separation\n"
        "\n"
        "- SCIENTIFIC_MODEL: frozen `src/loaded_cmj/v2/assets/v2_plant.xml`\n"
        f"  (PLANT_SHA256={PLANT_SHA256}), canonical reset, accepted action\n"
        "  schedule, exact `plant.apply_action` + `mj_step` integration. Never\n"
        "  resized, never re-lit, never re-camera'd.\n"
        "- RENDER_MODEL: separately loaded from the same frozen file; the file\n"
        "  on disk is byte-identical (re-verified before render). Only the\n"
        "  in-memory `vis.global offwidth/offheight` was overridden to\n"
        f"  {WIDTH}x{HEIGHT} because the frozen XML declares a 640x480\n"
        "  offscreen framebuffer. The render model is never stepped: viz data\n"
        "  receives copied qpos only, then `mj_fwdPosition` (position kinematics,\n"
        "  no solver, no dynamics) + `Renderer.render`.\n"
        "  There is no code path from renderer/render model back to scientific\n"
        "  replay objects.\n"
        "- Controller modules are never imported by the smoke harness, so\n"
        "  controller state cannot be reached, let alone mutated.\n"
        "- Telemetry uses the official `SynchronizedPhysicsSample.from_live_state`\n"
        "  observational path on dedicated scratch/shadow data; results flow\n"
        "  only into subtitles/stills/provenance, never into replay.\n"
        "- Replay history arrays are marked read-only after extraction; hashes\n"
        "  before render, after render, and of tracked science files are\n"
        "  recorded in SMOKE_RENDER_METADATA.json and match.\n"
        "\n"
        "## Frame timing policy\n"
        "\n"
        f"- {FPS} fps; frame j renders at t=j/{FPS}; source = nearest physics\n"
        "  sample (sample i covers ((i)*DT,(i+1)*DT], timestamped (i+1)*DT).\n"
        f"- Full episode: frames 0..{render['n_frames'] - 1} "
        f"({render['n_frames']} frames).\n"
        "- Landing closeup: frames with t in [0.85, 1.15] s "
        f"({len(render['closeup_frame_map'])} frames), zoomed fixed side camera.\n"
        "  (Contact-point markers are unavailable via the MuJoCo 3.8 classic\n"
        "  Renderer scene-flag range; foot/floor geometry is inspected directly.)\n"
        "- Event/landing stills use EXACT authoritative physics samples (see\n"
        "  EVENT_FRAME_INDEX.json), not nearest video frames.\n"
        "\n"
        "## Code review / bug hunt\n"
        "\n"
        "Recorded by the operator in the Linear RES-79 comment and final receipt\n"
        "(CODE_REVIEW= / BUG_HUNT= fields). No HIGH/CRITICAL harness issue may\n"
        "remain.\n"
    )
    return p


def write_review_packet(out_dir: Path, auth: dict, render: dict,
                        telem: dict[int, dict]) -> Path:
    idx = sorted(telem)
    comz = [telem[i]["com_z"] for i in idx]
    apex_i = render["stills"]["E8_apex_occurrence"]
    p = out_dir / "VISUAL_REVIEW_PACKET.md"
    p.write_text(
        "# RES-79 visual review packet (owner adjudication required)\n"
        "\n"
        "MISSION=RES13A_ACCEPTED_TRAJECTORY_VISUAL_SMOKE_001\n"
        f"CANDIDATE_ID={CANDIDATE_ID}\n"
        f"TRACE_SHA256={TRACE_SHA256}\n"
        "\n"
        "OWNER_VISUAL_REVIEW_REQUIRED=true\n"
        "\n"
        "This packet covers the FIRST forensic render of the accepted physical\n"
        "trajectory. It is raw MuJoCo physics geometry, not MakeHuman, not RES-14.\n"
        "\n"
        "## Media inventory\n"
        "\n"
        f"- FULL_SMOKE_SIDE.mp4 — full episode, fixed side camera, {FPS} fps,\n"
        f"  {WIDTH}x{HEIGHT}, {render['n_frames']} frames, telemetry subtitles burned in.\n"
        "- FULL_SMOKE_SECONDARY.mp4 — same episode, fixed 3/4 camera.\n"
        "- LANDING_CLOSEUP.mp4 — t in [0.85, 1.15] s, zoomed side camera on\n"
        "  feet/floor region.\n"
        "- EVENT_STILLS/ — E1..E12 at exact occurrence samples.\n"
        "- LANDING_CONTACT_STILLS/ — pre-landing, occurrence, E9 confirmation, peak\n"
        "  penetration, E10 occurrence/confirmation, E11 occurrence/confirmation.\n"
        "- EVENT_FRAME_INDEX.json — per-frame and per-event provenance.\n"
        "- SMOKE_RENDER_METADATA.json — full authority binding + media hashes.\n"
        "\n"
        "## Measured facts from accepted authority (not visual claims)\n"
        "\n"
        f"- COM z range over rendered frames: {min(comz):.4f}..{max(comz):.4f} m.\n"
        f"- COM z at apex sample ({apex_i}): {telem[apex_i]['com_z']:.4f} m, "
        f"vz={telem[apex_i]['com_vz']:.4f} m/s.\n"
        f"- Peak total Fz authority: {auth['result']['PEAK_BW']:.4f} BW.\n"
        f"- Max penetration authority: {auth['result']['MAXPEN_M']:.6f} m; "
        "landing-window\n"
        f"  replay peak at sample {render['peak_penetration']['sample']} = "
        f"{render['peak_penetration']['pen_m']:.6f} m.\n"
        f"- Session max |u| = {MAXUTIL:.4f}.\n"
        "\n"
        "## Owner inspection checklist\n"
        "\n"
        "1. initial standing\n"
        "2. bar placement\n"
        "3. bar-body coupling\n"
        "4. countermovement depth\n"
        "5. countermovement timing\n"
        "6. hip coordination\n"
        "7. knee coordination\n"
        "8. ankle coordination\n"
        "9. lumbar/torso behavior\n"
        "10. upward reversal\n"
        "11. propulsion\n"
        "12. bilateral takeoff\n"
        "13. actual visible floor clearance\n"
        "14. flight posture\n"
        "15. apex\n"
        "16. descent\n"
        "17. landing orientation\n"
        "18. foot-ground interaction\n"
        "19. penetration\n"
        "20. impact absorption\n"
        "21. balance capture\n"
        "22. long recovery phase\n"
        "23. stand handoff\n"
        "24. E12 posture\n"
        "25. joint flips\n"
        "26. hyperextension\n"
        "27. body intersections\n"
        "28. bar intersections\n"
        "29. topology errors\n"
        "30. any grossly non-human motion\n"
        "\n"
        "## Visual observations (operator)\n"
        "\n"
        "Filled after still inspection; each classified RENDER_ONLY / RETARGET_ONLY /\n"
        "TELEMETRY_ONLY / PHYSICS_OR_MODEL_DEFECT. Aesthetics alone (small jump, slow\n"
        "recovery, unattractive posture) are NOT defects. Do not tune from this packet.\n"
    )
    return p


def cross_check_frozen_rerun(path: Path | None) -> list[dict]:
    if path is None or not Path(path).exists():
        return []
    fresh = json.loads(Path(path).read_text())
    gates = [
        check_gate("FROZEN_TRACE", fresh.get("TRACE_SHA256") == TRACE_SHA256,
                   str(fresh.get("TRACE_SHA256"))),
        check_gate("FROZEN_N_CTRL", fresh.get("N_CTRL") == N_CTRL,
                   str(fresh.get("N_CTRL"))),
        check_gate("FROZEN_N_PHYS", fresh.get("N_PHYS") == N_PHYS,
                   str(fresh.get("N_PHYS"))),
        check_gate("FROZEN_T_END", fresh.get("T_END") == T_END,
                   str(fresh.get("T_END"))),
        check_gate("FROZEN_OUTCOME", fresh.get("OUTCOME") == OUTCOME,
                   str(fresh.get("OUTCOME"))),
        check_gate("FROZEN_TERMINATION",
                   fresh.get("TERMINATION") == TERMINATION,
                   str(fresh.get("TERMINATION"))),
        check_gate("FROZEN_ONLINE_OFFLINE",
                   fresh.get("ONLINE_OFFLINE_IDENTITY") == "PASS",
                   str(fresh.get("ONLINE_OFFLINE_IDENTITY"))),
        check_gate("FROZEN_EVENTS_12OF12",
                   fresh.get("EVENTS") == _accepted_events(),
                   "fresh EVENTS dict identical to RES-12 authority"
                   if fresh.get("EVENTS") == _accepted_events() else "MISMATCH"),
        check_gate("FROZEN_CHECKPOINTS_7OF7",
                   fresh.get("CANONICAL_CHECKPOINTS") == CANONICAL_CHECKPOINTS,
                   "fresh 7-checkpoint map identical to authority"
                   if fresh.get("CANONICAL_CHECKPOINTS") == CANONICAL_CHECKPOINTS
                   else "MISMATCH"),
    ]
    return gates


def _accepted_events() -> dict:
    bundle = Path(
        "/home/litju/Projects/loaded-cmj-control-evidence/"
        "EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001/"
        "V2.1-R001_CANONICAL_RESULT.json")
    return json.loads(bundle.read_text())["EVENTS"]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="RES-79 accepted-trajectory smoke replay")
    ap.add_argument("--bundle", required=True,
                    help="RES-12 canonical evidence bundle directory")
    ap.add_argument("--out-dir", required=True,
                    help="RES-79 evidence output directory")
    ap.add_argument("--stage", default="all",
                    choices=["extract", "render", "all"])
    ap.add_argument("--frozen-rerun-json", default="",
                    help="Optional fresh canonical result JSON for cross-check")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)
    bundle = Path(args.bundle)
    out = Path(args.out_dir)
    frozen = Path(args.frozen_rerun_json) if args.frozen_rerun_json else None
    if args.stage == "render":
        report, rep = load_extraction(out)
    else:
        report = cmd_extract(bundle, out, verbose=args.verbose)
        rep = None
    if args.stage in ("render", "all"):
        if rep is None:
            report, rep = load_extraction(out)
        frozen_gates = cross_check_frozen_rerun(frozen)
        ctx = cmd_render(bundle, out, report, rep, verbose=args.verbose)
        auth, telem, render = ctx["auth"], ctx["telem"], ctx["render"]
        media_checks = run_media_checks(out, render)
        failed = [g for g in media_checks if not g["pass"]]
        event_index_path = write_event_frame_index(out, auth, render)
        meta_path, media_hashes = write_metadata(out, auth, report, render,
                                                 media_checks, event_index_path,
                                                 frozen_gates)
        integ_path = write_integrity_report(out, report, render, media_checks,
                                            frozen_gates)
        packet_path = write_review_packet(out, auth, render, telem)
        sha_path = write_media_sha(out, media_hashes, [integ_path, packet_path])
        print(json.dumps({
            "render": "COMPLETE" if not failed else "CHECKS_FAILED",
            "media": render["media"],
            "event_frame_index": str(event_index_path),
            "metadata": str(meta_path),
            "integrity_report": str(integ_path),
            "review_packet": str(packet_path),
            "media_sha": str(sha_path),
            "failed_checks": failed}, indent=2))
        if failed:
            return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

