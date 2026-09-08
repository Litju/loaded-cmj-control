"""V2.1 bilateral support-continuity semantics (RES-57, reusable authority).

Pure classification helpers over SYNCHRONIZED physics-rate traces. No MuJoCo
calls, no Plant/contact/controller/scorer mutation, no mj_forward, no tuning.

Frozen contract (see BILATERAL_SUPPORT_CONTINUITY_CONTRACT.md and
support_continuity_spec.json):
  per-foot force threshold  = 10.0 N (existing authority, strict >)
  control-relevance dwell   = one 5-ms control interval = 40 physics steps
  separation evidence       = loss of geometric engagement (zero floor rows
                              and/or dist > 0) OR separating normal motion
                              (foot normal velocity > 0, +z separating)
  canonical reflight        = whole Fz < 10 N for >= 4 consecutive steps (unchanged)
  primary chatter           = whole-Fz threshold transitions > 8 (unchanged)

Sample-level facts and finite-duration episodes are separate fields and must
never be collapsed into one boolean.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

PHYSICS_DT_S = 0.000125
CONTROL_DT_S = 0.005
STEPS_PER_CONTROL_INTERVAL = int(round(CONTROL_DT_S / PHYSICS_DT_S))  # 40
PER_FOOT_FZ_THRESHOLD_N = 10.0
WHOLE_FZ_THRESHOLD_N = 10.0
WALK_WINDOW_S = 0.030
REFLIGHT_MIN_STEPS = 4
CHATTER_TRANSITIONS_MAX = 8
CONTROL_RELEVANT_LOSS_STEPS = STEPS_PER_CONTROL_INTERVAL  # 40
CONTROL_RELEVANT_LOSS_DWELL_S = CONTROL_DT_S  # 0.005

SPEC_FILENAME = "support_continuity_spec.json"


def _runs(mask) -> list:
    runs = []
    run = 0
    for b in mask:
        if b:
            run += 1
        elif run:
            runs.append(run)
            run = 0
    if run:
        runs.append(run)
    return runs


def _run_starts(mask) -> list:
    starts = []
    run = 0
    for i, b in enumerate(mask):
        if b:
            if run == 0:
                starts.append(i)
            run += 1
        else:
            run = 0
    return starts


def classify_sample(*, has_row: bool, dist: float, fz: float, nvel: float) -> dict:
    """Classify one foot at one physics sample (frozen semantics).

    has_row: True if foot has >=1 floor contact row (geometric engagement).
    dist: deepest signed contact dist (negative=penetrating) or geometric gap
        (positive) when no row. Must be consistent with has_row.
    fz: synchronized per-foot plantar normal force [N].
    nvel: foot normal velocity d(dist)/dt [m/s], +z separating.
    """
    geometric_engagement = bool(has_row)
    compressive_support = bool(float(fz) > PER_FOOT_FZ_THRESHOLD_N)
    force_dropout = bool(float(fz) <= PER_FOOT_FZ_THRESHOLD_N)
    # Geometric liftoff: no rows AND strictly separated gap. A penetrating or
    # exactly-touching (dist<=0) foot with no enumerated row is NOT liftoff
    # (numerical edge); report engagement False but liftoff False. In practice
    # MuJoCo enumerates a row for any dist below margin, so this edge is rare.
    if (not has_row) and (float(dist) > 0.0):
        geometric_liftoff = True
    else:
        geometric_liftoff = False
    separation_evidence = bool(geometric_liftoff or (float(nvel) > 0.0))
    return {
        "GEOMETRIC_ENGAGEMENT_SAMPLE": geometric_engagement,
        "COMPRESSIVE_SUPPORT_SAMPLE": compressive_support,
        "UNILATERAL_FORCE_DROPOUT_SAMPLE": force_dropout,  # per-foot part; caller ANDs bilaterally
        "GEOMETRIC_LIFTOFF_SAMPLE": geometric_liftoff,
        "SEPARATION_EVIDENCE_SAMPLE": separation_evidence,
    }


def unilateral_dropout_samples(fz_l, fz_r):
    """Boolean arrays: exactly one foot dropped out at each sample."""
    import numpy as np

    fzl = np.asarray(fz_l, dtype=float)
    fzr = np.asarray(fz_r, dtype=float)
    drop_l = fzl <= PER_FOOT_FZ_THRESHOLD_N
    drop_r = fzr <= PER_FOOT_FZ_THRESHOLD_N
    return bool(drop_l is not None) and (drop_l ^ drop_r)


def control_relevant_episodes(*, fz: list, has_row: list, nvel: list) -> list:
    """Maximal runs where force dropout AND separation evidence hold.

    Returns list of dicts {start, length, duration_s}. Control-relevant iff
    length >= 40 physics steps (5 ms). Shorter runs are returned too with
    CONTROL_RELEVANT=False so callers can distinguish transient from episode.
    """
    import numpy as np

    fz_a = np.asarray(fz, dtype=float)
    hr_a = np.asarray(has_row, dtype=bool)
    nv_a = np.asarray(nvel, dtype=float)
    # geometric liftoff per sample for separation evidence
    # NOTE: dist not passed here; has_row False is treated as disengagement.
    # Full sample classification (with dist) is done by classify_sample; for
    # episode detection the conservative proxy (no row => disengaged) is used
    # together with nvel>0. Dist-exact episodes are verified in the audit with
    # full contact rows. This proxy can only lengthen candidate runs when a
    # rowless-but-touching edge occurs, never shorten a true liftoff run.
    sep = (~hr_a) | (nv_a > 0.0)
    cand = (fz_a <= PER_FOOT_FZ_THRESHOLD_N) & sep
    starts = _run_starts(cand)
    lens = _runs(cand)
    out = []
    for st, ln in zip(starts, lens):
        out.append({
            "start": int(st),
            "length": int(ln),
            "duration_s": float(ln * PHYSICS_DT_S),
            "CONTROL_RELEVANT": bool(ln >= CONTROL_RELEVANT_LOSS_STEPS),
        })
    return out


def canonical_reflight(whole_fz) -> list:
    """Whole-support runs with Fz<10N sustained >=4 steps (unchanged)."""
    import numpy as np

    below = np.asarray(whole_fz, dtype=float) < WHOLE_FZ_THRESHOLD_N
    return [int(r) for r in _runs(below) if r >= REFLIGHT_MIN_STEPS]


def chatter_transitions(whole_fz) -> int:
    """Whole-Fz threshold transitions (unchanged primary-chatter predicate)."""
    import numpy as np

    below = (np.asarray(whole_fz, dtype=float) < WHOLE_FZ_THRESHOLD_N).astype(np.int8)
    if len(below) < 2:
        return 0
    return int(np.sum(np.diff(below) != 0))


def adjudicate_trajectory(*, fz_l, fz_r, whole_fz, has_row_l, has_row_r,
                          nvel_l, nvel_r, dist_l=None, dist_r=None) -> dict:
    """Adjudicate a full trajectory under the frozen contract.

    Returns sample counts plus per-foot control-relevant episodes, canonical
    reflight, chatter, and SUPPORT_CONTINUITY verdict (without hard gates;
    caller conjoins hard-gate status).
    """
    import numpy as np

    fzl = np.asarray(fz_l, dtype=float)
    fzr = np.asarray(fz_r, dtype=float)
    n = len(fzl)
    # sample-level unilateral dropout
    uni = ((fzl <= PER_FOOT_FZ_THRESHOLD_N) ^ (fzr <= PER_FOOT_FZ_THRESHOLD_N))
    # geometric liftoff counts (dist-exact when provided, else row proxy)
    if dist_l is not None and dist_r is not None:
        dl = np.asarray(dist_l, dtype=float)
        dr = np.asarray(dist_r, dtype=float)
        hrl = np.asarray(has_row_l, dtype=bool)
        hrr = np.asarray(has_row_r, dtype=bool)
        liftoff = ((~hrl) & (dl > 0.0)) | ((~hrr) & (dr > 0.0))
        liftoff_count = int(np.sum(liftoff))
    else:
        liftoff_count = 0
    ep_l = control_relevant_episodes(fz=fzl, has_row=has_row_l, nvel=nvel_l)
    ep_r = control_relevant_episodes(fz=fzr, has_row=has_row_r, nvel=nvel_r)
    n_control = sum(1 for e in ep_l + ep_r if e["CONTROL_RELEVANT"])
    refl = canonical_reflight(whole_fz)
    chat = chatter_transitions(whole_fz)
    support_ok = bool(n_control == 0 and len(refl) == 0 and chat <= CHATTER_TRANSITIONS_MAX)
    return {
        "N_SAMPLES": int(n),
        "UNILATERAL_FORCE_DROPOUT_SAMPLE_COUNT": int(np.sum(uni)),
        "GEOMETRIC_LIFTOFF_SAMPLE_COUNT": int(liftoff_count),
        "CONTROL_RELEVANT_EPISODES_L": ep_l,
        "CONTROL_RELEVANT_EPISODES_R": ep_r,
        "CONTROL_RELEVANT_SUPPORT_LOSS_COUNT": int(n_control),
        "CANONICAL_REFLIGHT": [int(r) for r in refl],
        "CHATTER_TRANSITIONS": int(chat),
        "SUPPORT_CONTINUITY": "QUALIFIED" if support_ok else "NOT_QUALIFIED",
    }


def spec_sha256(spec: dict) -> str:
    """Hash of spec excluding its own SPEC_SHA256 field (evidence-contract style)."""
    payload = {k: v for k, v in spec.items() if k != "SPEC_SHA256"}
    blob = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    return hashlib.sha256(blob.encode()).hexdigest()


def load_spec(path: str | Path) -> dict:
    """Load spec and verify hash immutability."""
    spec = json.loads(Path(path).read_text())
    expect = spec.get("SPEC_SHA256")
    if expect != spec_sha256(spec):
        raise ValueError(f"support-continuity spec hash mismatch: {expect} != recomputed")
    return spec
