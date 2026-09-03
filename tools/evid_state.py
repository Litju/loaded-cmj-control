#!/usr/bin/env python3
"""Full MuJoCo integration-state certificate (R0.1 §2D + §9).

Uses mj_stateSize / mj_getState / mj_setState with state_spec =
mjSTATE_INTEGRATION. Persists state_spec, state_size, state_vector,
state_vector_sha256 plus the metadata needed to prove environment identity:

  MUJOCO_VERSION, ARCHITECTURE, MODEL_HASH, COMMIT_SHA, COMMIT_TREE

The certificate supports replay from an arbitrary branch point (E8/E9/E10/E11
or any t>0) without manual reconstruction. It does NOT rely only on
qpos/qvel/qacc/ctrl/time: mjSTATE_INTEGRATION for this Plant is 108 floats
[time(1), qpos(10), qvel(10), qacc_warmstart(10), ctrl(7),
 qfrc_applied(10), xfrc_applied(60)] and the warmstart/applied-force tails
are required for exact continuation under contacts.
"""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
from pathlib import Path

import mujoco
import numpy as np

STATE_SPEC_INT = int(mujoco.mjtState.mjSTATE_INTEGRATION)
STATE_SPEC_NAME = "mjSTATE_INTEGRATION"


def git_rev(kind: str, root: Path) -> str:
    return subprocess.check_output(["git", "rev-parse", kind], cwd=str(root)).decode().strip()


def capture_state_vector(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    size = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)
    vec = np.zeros(size, dtype=np.float64)
    mujoco.mj_getState(model, data, vec, mujoco.mjtState.mjSTATE_INTEGRATION)
    return vec.copy()


def restore_state_vector(model: mujoco.MjModel, data: mujoco.MjData, vec: np.ndarray) -> None:
    vec = np.asarray(vec, dtype=np.float64).reshape(-1)
    expected = mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)
    if vec.shape[0] != expected:
        raise ValueError(f"state vector size {vec.shape[0]} != expected {expected}")
    mujoco.mj_setState(model, data, vec, mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)


def write_state_certificate(
    bundle_dir: Path,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    model_hash: str,
    root: Path,
    stem: str = "initial_integration_state",
) -> dict:
    """Capture full integration state + controller-agnostic metadata.

    Writes <stem>.npz (state_vector, qpos, qvel, ctrl, time for readability)
    and <stem>.json (spec/size/sha + environment identity). Returns the JSON
    dict (also written to disk).
    """
    from loaded_cmj.v2.plant import V2Plant  # noqa: F401 (ensures Plant import path)

    vec = capture_state_vector(model, data)
    sha = hashlib.sha256(vec.tobytes()).hexdigest()
    commit_sha = git_rev("HEAD", root)
    commit_tree = git_rev("HEAD^{tree}", root)
    cert = {
        "state_spec": STATE_SPEC_NAME,
        "state_spec_int": STATE_SPEC_INT,
        "state_size": int(vec.shape[0]),
        "state_vector_sha256": sha,
        "MUJOCO_VERSION": mujoco.__version__,
        "ARCHITECTURE": platform.machine(),
        "MODEL_HASH": model_hash,
        "COMMIT_SHA": commit_sha,
        "COMMIT_TREE": commit_tree,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "na": int(model.na),
        "nmocap": int(model.nmocap),
        "nuserdata": int(model.nuserdata),
        "neq": int(model.neq),
    }
    npz_path = bundle_dir / f"{stem}.npz"
    np.savez_compressed(
        npz_path,
        state_vector=np.asarray(vec, dtype=np.float64),
        state_spec=np.array([STATE_SPEC_INT], dtype=np.int64),
        state_size=np.array([int(vec.shape[0])], dtype=np.int64),
        # readability mirrors (NOT authority for replay; state_vector is)
        qpos=np.asarray(data.qpos, dtype=np.float64).copy(),
        qvel=np.asarray(data.qvel, dtype=np.float64).copy(),
        ctrl=np.asarray(data.ctrl, dtype=np.float64).copy(),
        time=np.array([float(data.time)], dtype=np.float64),
    )
    json_path = bundle_dir / f"{stem}.json"
    json_path.write_text(json.dumps(cert, indent=2, sort_keys=True) + "\n")
    cert["npz_path"] = str(npz_path)
    cert["json_path"] = str(json_path)
    return cert


def load_state_certificate(bundle_dir: Path, stem: str = "initial_integration_state") -> tuple[np.ndarray, dict]:
    meta = json.loads((bundle_dir / f"{stem}.json").read_text())
    arr = np.load(bundle_dir / f"{stem}.npz")
    vec = np.asarray(arr["state_vector"], dtype=np.float64).reshape(-1)
    sha = hashlib.sha256(vec.tobytes()).hexdigest()
    if sha != meta["state_vector_sha256"]:
        raise ValueError(f"state_vector sha mismatch: {sha} != {meta['state_vector_sha256']}")
    if int(vec.shape[0]) != int(meta["state_size"]):
        raise ValueError("state_size mismatch")
    if meta.get("state_spec") != STATE_SPEC_NAME:
        raise ValueError(f"state_spec mismatch: {meta.get('state_spec')}")
    return vec, meta
