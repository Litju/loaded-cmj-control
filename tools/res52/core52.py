"""RES-52 mission core: soft-contact state authority + synchronized rollout.

MISSION=RES10_SYNC_SOFT_CONTACT_FORCE_REALIZATION_AUTHORITY_001

Inherited frozen conventions (RES-51 verified lineage, no retired control plane):
- Branch restore: exact mjSTATE_INTEGRATION vectors (E8 SHA 401b4512...).
- All controller observations from SynchronizedPhysicsSample (shadow only).
- Exact forward MuJoCo mj_step is the sole physical authority.

Soft-contact per-foot state (this mission, Phase B):
- contact_distance_i : signed MuJoCo contact dist of the deepest floor contact
  for foot i (negative = penetrating). If the foot has no floor contact the
  geometric gap of the lowest foot corner above the floor is used (positive).
- penetration_i = max(0, -contact_distance_i)   [m]
- foot_normal_velocity_i : rate of change of the foot-floor normal distance at
  the contact point, m/s. Signed POSITIVE = SEPARATING (distance increasing),
  NEGATIVE = APPROACHING (penetration increasing). When active this is read
  from the synchronized efc_velocity of the corresponding contact row
  (MuJoCo computes exactly d(dist)/dt). When inactive it is computed from the
  synchronized foot body spatial velocity projected on the world -z normal at
  the lowest foot corner.
- actual_Fz_i : synchronized per-foot plantar normal force (foot_contact_summary).
- contact_active_i : synchronized actual_Fz_i > V2_CONTACT_FZ_THRESHOLD_N.
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path("/home/litju/Projects/loaded-cmj-control")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO))

from loaded_cmj.v2.plant import V2Plant  # noqa: E402
from loaded_cmj.v2.measurement import (  # noqa: E402
    SynchronizedPhysicsSample,
    create_measurement_data,
)
from loaded_cmj.v2.events import V2EventDetector, V2EventRecord  # noqa: E402
from loaded_cmj.v2.drive import LIMITS as DRIVE_LIMITS  # noqa: E402

PHYSICS_DT = 0.000125
CONTROL_DT = 0.005
SUB = 40
HORIZON = 4.0
MASS_KG = 95.0
G = 9.81
WEIGHT_N = MASS_KG * G  # 931.95 N body weight
BW_N = WEIGHT_N

E8_SHA = "401b45128033fc6eb1b59a5c8bbe6a3644b31cd06a367e89def1e74cff7eb0e5"
E8_TIME = 0.7692500000000487
STEP_AT_E8 = 153
REMAINING_SUBSTEPS = 6
TD_TIME = 0.8753750000000842          # C00 authority (verified fresh reproduction)
TD_COM_VZ = -1.0081503367597662
PRE_SEED = ["supported_start", "countermovement_onset", "valid_countermovement",
            "upward_reversal", "vertical_propulsion", "bilateral_takeoff",
            "genuine_flight", "apex"]
ROOT_JOINT_NAMES = ("root_tx", "root_tz", "root_ry")

WORK = Path("/tmp/opencode/res52/work")
EVIDENCE_ROOT = Path("/home/litju/Projects/loaded-cmj-control-evidence")
EXPERIMENT_ID = "EXP-RES10-SOFT-CONTACT-FORCE-REALIZATION-001"
BUNDLE = EVIDENCE_ROOT / EXPERIMENT_ID

# deterministic post-touchdown capture offsets (ms) -> physics times
S_OFFSETS_S = {"S40": 0.040, "S50": 0.050, "S75": 0.075, "S100": 0.100}
S_TIMES = {k: TD_TIME + v for k, v in S_OFFSETS_S.items()}
STAND_CAPTURE_TIME = 1.0  # 5-ms control boundary inside RES-43 qualified window [0.100125, 2.0]

ACTUATOR_NAMES = ["m_lumbar", "m_left_hip", "m_right_hip", "m_left_knee",
                  "m_right_knee", "m_left_ankle", "m_right_ankle"]
JOINT_NAMES = ["lumbar", "left_hip", "right_hip", "left_knee",
               "right_knee", "left_ankle", "right_ankle"]


def sha_arr(a) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.asarray(a, dtype=np.float64)).tobytes()).hexdigest()


def sha_bytes(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def seed_detector(ref_events: dict) -> V2EventDetector:
    det = V2EventDetector()
    det.reset()
    for name in PRE_SEED:
        r = ref_events[name]
        det.event_records[name] = V2EventRecord(
            name=name, occurred_at=float(r["occurred_at"]),
            confirmed_at=float(r["confirmed_at"]),
            sample_index=int(r["sample_index"]),
            confirmed_sample_index=int(r["confirmed_sample_index"]))
        det.events[name] = float(r["occurred_at"])
        det.valid[name] = True
    return det


def make_plant() -> V2Plant:
    return V2Plant()


def restore_state(model, data, vec: np.ndarray):
    mujoco.mj_setState(model, data, np.ascontiguousarray(vec, dtype=np.float64),
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)


def get_state_vector(model, data) -> np.ndarray:
    vec = np.zeros(int(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION)))
    mujoco.mj_getState(model, data, vec, mujoco.mjtState.mjSTATE_INTEGRATION)
    return vec


def static_inverse_u(plant, x_vec: np.ndarray, baseline_data) -> np.ndarray:
    """Static inverse-dynamics hold baseline at the current synchronized state.

    Computes the actuator torques that hold qacc=0 under the current gravity +
    contact loading (mj_inverse on an independent copy), mapped through the
    bounded torque drive. This baseline statically CARRIES the current load
    instead of relaxing under it (which a position-PD toward the cell-start
    posture cannot do at deep-flexion landing postures).
    """
    m = plant.model
    vadr = [plant.idx.vadr[n] for n in JOINT_NAMES]
    mujoco.mj_setState(m, baseline_data, np.ascontiguousarray(x_vec),
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(m, baseline_data)
    baseline_data.qacc[:] = 0.0
    mujoco.mj_inverse(m, baseline_data)
    tau = np.asarray(baseline_data.qfrc_inverse[vadr], dtype=float)
    return np.clip(tau / DRIVE_LIMITS, -1.0, 1.0)


def foot_floor_contacts(model, data) -> dict:
    """Per-foot floor contact rows for foot geoms, from the CURRENT MjData.

    Returns dict with per foot: ncon, deepest dist row index (or None).
    """
    plant_idx = _IDX  # bound by bind_plant
    out = {}
    for side, fg in (("L", plant_idx.left_foot_geom), ("R", plant_idx.right_foot_geom)):
        rows = []
        for i in range(data.ncon):
            con = data.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            if (g1 == plant_idx.floor_geom or g2 == plant_idx.floor_geom) and (g1 == fg or g2 == fg):
                rows.append(i)
        deepest = None
        if rows:
            dists = [float(data.contact[i].dist) for i in rows]
            deepest = rows[int(np.argmin(dists))]
        out[side] = {"rows": rows, "deepest_row": deepest, "ncon": len(rows)}
    return out


def true_foot_point_velocity(model, data, body_id: int, point_world: np.ndarray) -> np.ndarray:
    """RES-55 true foot-point velocity authority (non-mutating, same-state).

    v_point = J_point(q) @ qvel via MuJoCo mj_jac at the exact material point.
    Proven equivalent to v_xipos + omega x (p - xipos) to 1e-16 (M1≈M2).
    Must NOT use mj_objectVelocity(BODY).linear + omega x (p - xpos): that
    linear term already equals the xipos-point velocity (RES-54 proof via
    jacBodyCom identity), so (p - xpos) transport double-counts rotation.
    No mj_forward on live solely for measurement refresh; caller passes the
    already-synchronized same-state MjData.
    """
    Jp = np.zeros((3, model.nv), dtype=np.float64)
    Jr = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jac(model, data, Jp, Jr, np.asarray(point_world, dtype=np.float64),
                  int(body_id))
    return (Jp @ np.asarray(data.qvel, dtype=np.float64)).copy()


def soft_contact_state(model, data, *, gap_half_height: float) -> dict:
    """Phase B per-foot soft-contact state from the CURRENT synchronized MjData.

    gap_half_height: vertical half-height of the foot box above its origin,
    used only for the inactive-foot geometric gap.
    """
    fc = foot_floor_contacts(model, data)
    st = {}
    for side in ("L", "R"):
        info = fc[side]
        if info["deepest_row"] is not None:
            i = info["deepest_row"]
            con = data.contact[i]
            dist = float(con.dist)
            nvel = float(data.efc_vel[con.efc_address])
            active_row = True
        else:
            # geometric gap of lowest foot corner above the floor (positive)
            fg = _IDX.left_foot_geom if side == "L" else _IDX.right_foot_geom
            fb = _IDX.left_foot_body if side == "L" else _IDX.right_foot_body
            fz = float(data.geom_xpos[fg][2])
            dist = float(fz) - gap_half_height  # distance of lowest point to floor
            # separating velocity of the lowest foot point along world +z normal
            # (RES-55 authority): J_point @ qvel at the exact material corner.
            # Prohibited: mj_objectVelocity.linear + omega x (p - xpos).
            p_low = np.asarray(data.geom_xpos[fg], float).copy()
            p_low[2] -= gap_half_height
            v_pt = true_foot_point_velocity(model, data, fb, p_low)
            nvel = float(v_pt[2])  # +z motion = separating from floor
            active_row = False
        pen = max(0.0, -dist)
        st[side] = {
            "contact_distance": dist,
            "penetration": pen,
            "foot_normal_velocity": nvel,
            "active_row": active_row,
        }
    return st


def foot_gap_half_height(model) -> float:
    """Vertical half-size of the foot box geoms (model authority, not tuned)."""
    for side in ("left_foot_box", "right_foot_box"):
        gid = _IDX.geom[side]
        size = np.asarray(model.geom_size[gid], float)
        # box: half extents; assume z half-extent is size[2]
    return float(np.asarray(model.geom_size[_IDX.geom["left_foot_box"]], float)[2])


_IDX = None


def bind_plant(plant: V2Plant):
    global _IDX
    _IDX = plant.idx


# ----------------------------------------------------------------- traces ---
class TraceAcc:
    """Physics-rate synchronized trace accumulator for soft-contact cells."""

    def __init__(self):
        self.t = []
        self.qpos = []
        self.qvel = []
        self.qacc = []
        self.ctrl = []
        self.qfrc_actuator = []
        self.qfrc_passive = []
        self.com = []
        self.cv = []
        self.hy = []
        self.fzl = []
        self.fzr = []
        self.fxl = []
        self.fxr = []
        self.cop = []
        self.copv = []
        self.ncon = []
        self.prohib = []
        self.fall = []
        self.tilt = []
        self.jp = []
        self.jv = []
        self.root_rate = []
        self.margin = []
        # per-foot soft contact state (Phase B)
        self.distL = []
        self.distR = []
        self.penL = []
        self.penR = []
        self.nvelL = []
        self.nvelR = []
        self.actL = []
        self.actR = []
        self.phase = []
        self.c_t = []
        self.c_phase = []
        self.c_info = []

    def add_physics(self, s: SynchronizedPhysicsSample, cstate: dict, hy: float, phase: int):
        self.t.append(float(s.time))
        self.qpos.append(np.asarray(s.qpos, float))
        self.qvel.append(np.asarray(s.qvel, float))
        self.qacc.append(np.asarray(s.qacc, float))
        self.ctrl.append(np.asarray(s.ctrl, float))
        self.qfrc_actuator.append(np.asarray(s.qfrc_actuator, float))
        self.qfrc_passive.append(np.asarray(s.qfrc_passive, float))
        self.com.append(np.asarray(s.com_position_m, float))
        self.cv.append(np.asarray(s.com_velocity_mps, float))
        self.fzl.append(float(s.left_Fz_N))
        self.fzr.append(float(s.right_Fz_N))
        self.fxl.append(float(s.left_force_world_N[0]))
        self.fxr.append(float(s.right_force_world_N[0]))
        self.cop.append(np.asarray(s.plantar_cop_xy_m, float))
        self.copv.append(np.asarray(s.plantar_cop_valid, bool))
        self.ncon.append(int(s.ncon))
        self.prohib.append(bool(s.prohibited_contact))
        self.fall.append(bool(s.fall_contact))
        self.tilt.append(float(s.trunk_tilt_rad))
        self.jp.append(np.asarray(s.joint_position_rad, float))
        self.jv.append(np.asarray(s.joint_velocity_radps, float))
        self.root_rate.append([float(s.qvel[0]), float(s.qvel[1]), float(s.qvel[2])])
        self.margin.append(float(s.support_margin_m))
        for side, dist_key, pen_key, nv_key, act_key in (
                ("L", "distL", "penL", "nvelL", "actL"), ("R", "distR", "penR", "nvelR", "actR")):
            getattr(self, dist_key).append(float(cstate[side]["contact_distance"]))
            getattr(self, pen_key).append(float(cstate[side]["penetration"]))
            getattr(self, nv_key).append(float(cstate[side]["foot_normal_velocity"]))
            getattr(self, act_key).append(bool(cstate[side]["active_row"]))
        self.hy.append(float(hy))
        self.phase.append(int(phase))

    def add_control(self, t: float, phase: int, info: dict):
        self.c_t.append(float(t))
        self.c_phase.append(int(phase))
        self.c_info.append(dict(info))

    def arrays(self) -> dict:
        out = {}
        for key in ["t", "distL", "distR", "penL", "penR", "nvelL", "nvelR",
                    "margin", "tilt", "fzl", "fzr", "fxl", "fxr", "hy"]:
            out[key] = np.asarray(getattr(self, key), dtype=float)
        for key in ["actL", "actR", "prohib", "fall", "copv"]:
            out[key] = np.asarray(getattr(self, key), dtype=bool)
        for key in ["qpos", "qvel", "qacc", "com", "cv", "jp", "jv", "ctrl",
                    "qfrc_actuator", "qfrc_passive"]:
            v = getattr(self, key)
            if v:
                out[key] = np.stack(v)
            else:
                w = 10 if key in ("qpos", "qvel", "qacc", "qfrc_actuator", "qfrc_passive") else 3 if key in ("com", "cv") else 7
                out[key] = np.zeros((0, w))
        out["cop"] = np.stack(self.cop) if self.cop else np.zeros((0, 2, 2))
        out["root_rate"] = np.asarray(self.root_rate, dtype=float) if self.root_rate else np.zeros((0, 3))
        out["ncon"] = np.asarray(self.ncon, dtype=np.int64)
        out["phase"] = np.asarray(self.phase, dtype=np.int64)
        return out

    def control_arrays(self) -> dict:
        return {"c_t": np.asarray(self.c_t, dtype=float),
                "c_phase": np.asarray(self.c_phase, dtype=np.int64)}
