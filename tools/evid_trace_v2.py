#!/usr/bin/env python3
"""RES-10 trace schema v2 (R0.1 §2F + §8).

LCMJ_TRACE_SCHEMA_VERSION=2.

Physics-rate sampling (dt=0.000125 s): one sample AFTER every mj_step,
t = dt..T (N samples, no t=0 sample). Control-rate trace (5 ms): one entry
per control step k=0..M-1 at t=k*control_dt (M samples).

Documented derivation policy: every scientific claim is traceable either to
a directly recorded array or to a clearly documented deterministic
derivation listed in TRACE_SCHEMA_V2.md. No duplicated derived values are
stored when the derivation is exact and documented.

Units: SI (m, m/s, rad, rad/s, N, Nm, s). Frames: world for positions,
forces, momenta; joint-local for qpos/qvel ordering per V2_JOINT_NAMES.
NaN policy: fixed-size padded contact/efc tables use sentinel -1 (ids/types)
and NaN (floats) for absent rows; all dense physics arrays must be finite
(fail-closed otherwise).
"""

from __future__ import annotations

import mujoco
import numpy as np

LCMJ_TRACE_SCHEMA_VERSION = 2
PHYSICS_DT = 0.000125
CONTROL_DT = 0.005
SUBSTEPS_PER_CONTROL = 40
MAX_CONTACTS_PAD = 16
MAX_EFC_PAD = 128

PHYSICS_FIELDS = [
    "time", "qpos", "qvel", "qacc", "ctrl",
    "qfrc_actuator", "qfrc_passive", "qfrc_constraint",
    "root_pos", "root_vel", "root_angvel",
    "com", "com_vel", "H", "Hy",
    "trunk_tilt", "trunk_angvel", "torso_xquat",
    "left_Fz", "right_Fz", "left_force", "right_force",
    "left_moment", "right_moment", "left_cop", "right_cop",
    "cop_valid", "ncon", "contact_geom1", "contact_geom2",
    "contact_dist", "contact_pos", "contact_count",
    "nefc", "efc_type", "efc_id", "efc_force",
    "support_polygon", "support_margin",
    "actuator_torque", "actuator_util",
    "controller_phase", "fall_flag", "prohibited_flag",
    "contact_state", "reflight_flag",
]

CONTROL_FIELDS = ["time", "action", "phase", "phase_name", "internal_state"]


def centroidal_H_world(model: mujoco.MjModel, data: mujoco.MjData, com: np.ndarray) -> np.ndarray:
    """Whole-body centroidal angular momentum about COM (world, kg m^2/s).

    Derivation (exact, deterministic):
      H = sum_bodies [ Iw_b @ omega_b + (xipos_b - com) x (m_b * v_b) ]
    where omega_b/v_b come from mj_objectVelocity (world), Iw_b =
    R_b @ diag(body_inertia_b) @ R_b^T with R_b from data.ximat, and the sum
    covers every non-world body including the jointless 20 kg external load.
    """
    center = np.asarray(com, dtype=np.float64).reshape(3)
    h = np.zeros(3, dtype=np.float64)
    vel = np.zeros(6, dtype=np.float64)
    for bid in range(1, model.nbody):
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, int(bid), vel, 0)
        rot = np.asarray(data.ximat[bid], dtype=np.float64).reshape(3, 3)
        ib = np.diag(np.asarray(model.body_inertia[bid], dtype=np.float64))
        iw = rot @ ib @ rot.T
        mass = float(model.body_mass[bid])
        h += iw @ vel[:3]
        h += np.cross(np.asarray(data.xipos[bid], dtype=np.float64) - center, mass * vel[3:6])
    return h


def body_linear_angular_vel(model: mujoco.MjModel, data: mujoco.MjData, body_id: int):
    vel = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, int(body_id), vel, 0)
    return vel[3:6].copy(), vel[:3].copy()


class TraceV2Collector:
    """Accumulates physics-rate + control-rate samples for schema v2."""

    def __init__(self, plant) -> None:
        self.plant = plant
        self.m = plant.model
        self.p = {}
        self.c = {}
        self._reset()

    def _reset(self):
        P: dict = {k: [] for k in PHYSICS_FIELDS}
        C: dict = {k: [] for k in CONTROL_FIELDS}
        self.p, self.c = P, C

    # -- control entry (called once per control step, before substeps) --
    def push_control(self, t: float, action: np.ndarray, phase: int, phase_name: str, internal: dict):
        self.c["time"].append(float(t))
        self.c["action"].append(np.asarray(action, dtype=np.float64).reshape(-1).copy())
        self.c["phase"].append(int(phase))
        self.c["phase_name"].append(str(phase_name))
        self.c["internal_state"].append(dict(internal))

    # -- physics entry (called after every mj_step) --
    def push_physics(self, t: float, phase: int, fall_flag: bool, reflight_flag: bool = False):
        from loaded_cmj.v2.constants import V2_TORQUE_LIMITS_NM, V2_MJ_ACTUATOR_NAMES

        plant, m, d_holder = self.plant, self.m, None
        # caller passes data via self._data set before stepping loop
        d = self._data
        com = plant.center_of_mass(d)
        com_vel = plant.center_of_mass_velocity(d)
        summary = plant.foot_contact_summary(d)
        H = centroidal_H_world(m, d, com)
        # root = pelvis body world
        root_pos = np.asarray(d.xpos[plant.idx.pelvis_body], dtype=np.float64).copy()
        lin, ang = body_linear_angular_vel(m, d, plant.idx.pelvis_body)
        _, trunk_angvel = body_linear_angular_vel(m, d, plant.idx.torso_body)
        torso_quat = np.asarray(d.xquat[plant.idx.torso_body], dtype=np.float64).copy()
        # contacts padded
        g1 = np.full((MAX_CONTACTS_PAD,), -1, dtype=np.int64)
        g2 = np.full((MAX_CONTACTS_PAD,), -1, dtype=np.int64)
        dist = np.full((MAX_CONTACTS_PAD,), np.nan, dtype=np.float64)
        pos = np.full((MAX_CONTACTS_PAD, 3), np.nan, dtype=np.float64)
        ncon = int(d.ncon)
        for i in range(min(ncon, MAX_CONTACTS_PAD)):
            con = d.contact[i]
            g1[i] = int(con.geom1)
            g2[i] = int(con.geom2)
            dist[i] = float(con.dist)
            pos[i, :] = np.asarray(con.pos, dtype=np.float64)
        # efc padded
        nefc = int(d.nefc)
        et = np.full((MAX_EFC_PAD,), -1, dtype=np.int64)
        eid = np.full((MAX_EFC_PAD,), -1, dtype=np.int64)
        ef = np.full((MAX_EFC_PAD,), np.nan, dtype=np.float64)
        nn = min(nefc, MAX_EFC_PAD)
        if nn > 0:
            et[:nn] = np.asarray(d.efc_type[:nn], dtype=np.int64)
            eid[:nn] = np.asarray(d.efc_id[:nn], dtype=np.int64)
            ef[:nn] = np.asarray(d.efc_force[:nn], dtype=np.float64)
        # support polygon from foot geom centers (documented in TRACE_SCHEMA_V2.md)
        lg, rg = plant.idx.left_foot_geom, plant.idx.right_foot_geom
        lcx, lcy = float(d.geom_xpos[lg][0]), float(d.geom_xpos[lg][1])
        rcx, rcy = float(d.geom_xpos[rg][0]), float(d.geom_xpos[rg][1])
        hx, hy = 0.150, 0.060
        poly = np.array(
            [min(lcx - hx, rcx - hx), max(lcx + hx, rcx + hx),
             min(lcy - hy, rcy - hy), max(lcy + hy, rcy + hy)],
            dtype=np.float64,
        )
        # actuator torque/util
        u = np.asarray(d.ctrl, dtype=np.float64).reshape(-1)
        limits = np.array([V2_TORQUE_LIMITS_NM[n] for n in
                           ["lumbar", "left_hip", "right_hip", "left_knee",
                            "right_knee", "left_ankle", "right_ankle"]], dtype=np.float64)
        tau = limits * u
        util = np.abs(u)
        P = self.p
        P["time"].append(float(t))
        P["qpos"].append(np.asarray(d.qpos, dtype=np.float64).copy())
        P["qvel"].append(np.asarray(d.qvel, dtype=np.float64).copy())
        P["qacc"].append(np.asarray(d.qacc, dtype=np.float64).copy())
        P["ctrl"].append(u.copy())
        P["qfrc_actuator"].append(np.asarray(d.qfrc_actuator, dtype=np.float64).copy())
        P["qfrc_passive"].append(np.asarray(d.qfrc_passive, dtype=np.float64).copy())
        P["qfrc_constraint"].append(np.asarray(d.qfrc_constraint, dtype=np.float64).copy())
        P["root_pos"].append(root_pos)
        P["root_vel"].append(np.asarray(lin, dtype=np.float64))
        P["root_angvel"].append(np.asarray(ang, dtype=np.float64))
        P["com"].append(np.asarray(com, dtype=np.float64).copy())
        P["com_vel"].append(np.asarray(com_vel, dtype=np.float64).copy())
        P["H"].append(np.asarray(H, dtype=np.float64))
        P["Hy"].append(float(H[1]))
        P["trunk_tilt"].append(float(plant.trunk_tilt(d)))
        P["trunk_angvel"].append(np.asarray(trunk_angvel, dtype=np.float64))
        P["torso_xquat"].append(torso_quat)
        P["left_Fz"].append(float(summary["left_Fz"]))
        P["right_Fz"].append(float(summary["right_Fz"]))
        P["left_force"].append(np.asarray(summary["left_force"], dtype=np.float64))
        P["right_force"].append(np.asarray(summary["right_force"], dtype=np.float64))
        P["left_moment"].append(np.asarray(summary["left_moment_world_origin"], dtype=np.float64))
        P["right_moment"].append(np.asarray(summary["right_moment_world_origin"], dtype=np.float64))
        P["left_cop"].append(np.asarray(summary["left_cop"], dtype=np.float64))
        P["right_cop"].append(np.asarray(summary["right_cop"], dtype=np.float64))
        P["cop_valid"].append(np.asarray(
            [bool(summary["left_cop_valid"]), bool(summary["right_cop_valid"])], dtype=np.bool_))
        P["ncon"].append(ncon)
        P["contact_geom1"].append(g1)
        P["contact_geom2"].append(g2)
        P["contact_dist"].append(dist)
        P["contact_pos"].append(pos)
        P["contact_count"].append(ncon)
        P["nefc"].append(nefc)
        P["efc_type"].append(et)
        P["efc_id"].append(eid)
        P["efc_force"].append(ef)
        P["support_polygon"].append(poly)
        P["support_margin"].append(float(summary["support_margin"]))
        P["actuator_torque"].append(tau)
        P["actuator_util"].append(util)
        P["controller_phase"].append(int(phase))
        P["fall_flag"].append(bool(fall_flag))
        P["prohibited_flag"].append(bool(summary["prohibited_contact"]))
        P["contact_state"].append(np.asarray(
            [bool(summary["active_left"]), bool(summary["active_right"])], dtype=np.bool_))
        P["reflight_flag"].append(bool(reflight_flag))

    def finalize(self) -> tuple[dict, dict]:
        P = {
            "time": np.asarray(self.p["time"], dtype=np.float64),
            "qpos": np.stack(self.p["qpos"]).astype(np.float64) if self.p["qpos"] else np.zeros((0, 10)),
            "qvel": np.stack(self.p["qvel"]).astype(np.float64) if self.p["qvel"] else np.zeros((0, 10)),
            "qacc": np.stack(self.p["qacc"]).astype(np.float64) if self.p["qacc"] else np.zeros((0, 10)),
            "ctrl": np.stack(self.p["ctrl"]).astype(np.float64) if self.p["ctrl"] else np.zeros((0, 7)),
            "qfrc_actuator": np.stack(self.p["qfrc_actuator"]).astype(np.float64) if self.p["qfrc_actuator"] else np.zeros((0, 10)),
            "qfrc_passive": np.stack(self.p["qfrc_passive"]).astype(np.float64) if self.p["qfrc_passive"] else np.zeros((0, 10)),
            "qfrc_constraint": np.stack(self.p["qfrc_constraint"]).astype(np.float64) if self.p["qfrc_constraint"] else np.zeros((0, 10)),
            "root_pos": np.stack(self.p["root_pos"]).astype(np.float64) if self.p["root_pos"] else np.zeros((0, 3)),
            "root_vel": np.stack(self.p["root_vel"]).astype(np.float64) if self.p["root_vel"] else np.zeros((0, 3)),
            "root_angvel": np.stack(self.p["root_angvel"]).astype(np.float64) if self.p["root_angvel"] else np.zeros((0, 3)),
            "com": np.stack(self.p["com"]).astype(np.float64) if self.p["com"] else np.zeros((0, 3)),
            "com_vel": np.stack(self.p["com_vel"]).astype(np.float64) if self.p["com_vel"] else np.zeros((0, 3)),
            "H": np.stack(self.p["H"]).astype(np.float64) if self.p["H"] else np.zeros((0, 3)),
            "Hy": np.asarray(self.p["Hy"], dtype=np.float64),
            "trunk_tilt": np.asarray(self.p["trunk_tilt"], dtype=np.float64),
            "trunk_angvel": np.stack(self.p["trunk_angvel"]).astype(np.float64) if self.p["trunk_angvel"] else np.zeros((0, 3)),
            "torso_xquat": np.stack(self.p["torso_xquat"]).astype(np.float64) if self.p["torso_xquat"] else np.zeros((0, 4)),
            "left_Fz": np.asarray(self.p["left_Fz"], dtype=np.float64),
            "right_Fz": np.asarray(self.p["right_Fz"], dtype=np.float64),
            "left_force": np.stack(self.p["left_force"]).astype(np.float64) if self.p["left_force"] else np.zeros((0, 3)),
            "right_force": np.stack(self.p["right_force"]).astype(np.float64) if self.p["right_force"] else np.zeros((0, 3)),
            "left_moment": np.stack(self.p["left_moment"]).astype(np.float64) if self.p["left_moment"] else np.zeros((0, 3)),
            "right_moment": np.stack(self.p["right_moment"]).astype(np.float64) if self.p["right_moment"] else np.zeros((0, 3)),
            "left_cop": np.stack(self.p["left_cop"]).astype(np.float64) if self.p["left_cop"] else np.zeros((0, 2)),
            "right_cop": np.stack(self.p["right_cop"]).astype(np.float64) if self.p["right_cop"] else np.zeros((0, 2)),
            "cop_valid": np.stack(self.p["cop_valid"]) if self.p["cop_valid"] else np.zeros((0, 2), dtype=np.bool_),
            "ncon": np.asarray(self.p["ncon"], dtype=np.int64),
            "contact_geom1": np.stack(self.p["contact_geom1"]) if self.p["contact_geom1"] else np.zeros((0, MAX_CONTACTS_PAD), dtype=np.int64),
            "contact_geom2": np.stack(self.p["contact_geom2"]) if self.p["contact_geom2"] else np.zeros((0, MAX_CONTACTS_PAD), dtype=np.int64),
            "contact_dist": np.stack(self.p["contact_dist"]) if self.p["contact_dist"] else np.zeros((0, MAX_CONTACTS_PAD)),
            "contact_pos": np.stack(self.p["contact_pos"]) if self.p["contact_pos"] else np.zeros((0, MAX_CONTACTS_PAD, 3)),
            "contact_count": np.asarray(self.p["contact_count"], dtype=np.int64),
            "nefc": np.asarray(self.p["nefc"], dtype=np.int64),
            "efc_type": np.stack(self.p["efc_type"]) if self.p["efc_type"] else np.zeros((0, MAX_EFC_PAD), dtype=np.int64),
            "efc_id": np.stack(self.p["efc_id"]) if self.p["efc_id"] else np.zeros((0, MAX_EFC_PAD), dtype=np.int64),
            "efc_force": np.stack(self.p["efc_force"]) if self.p["efc_force"] else np.zeros((0, MAX_EFC_PAD)),
            "support_polygon": np.stack(self.p["support_polygon"]).astype(np.float64) if self.p["support_polygon"] else np.zeros((0, 4)),
            "support_margin": np.asarray(self.p["support_margin"], dtype=np.float64),
            "actuator_torque": np.stack(self.p["actuator_torque"]).astype(np.float64) if self.p["actuator_torque"] else np.zeros((0, 7)),
            "actuator_util": np.stack(self.p["actuator_util"]).astype(np.float64) if self.p["actuator_util"] else np.zeros((0, 7)),
            "controller_phase": np.asarray(self.p["controller_phase"], dtype=np.int64),
            "fall_flag": np.asarray(self.p["fall_flag"], dtype=np.bool_),
            "prohibited_flag": np.asarray(self.p["prohibited_flag"], dtype=np.bool_),
            "contact_state": np.stack(self.p["contact_state"]) if self.p["contact_state"] else np.zeros((0, 2), dtype=np.bool_),
            "reflight_flag": np.asarray(self.p["reflight_flag"], dtype=np.bool_),
        }
        C = {
            "time": np.asarray(self.c["time"], dtype=np.float64),
            "action": np.stack(self.c["action"]).astype(np.float64) if self.c["action"] else np.zeros((0, 7)),
            "phase": np.asarray(self.c["phase"], dtype=np.int64),
            "phase_name": np.asarray(self.c["phase_name"]),
            "internal_state_keys": np.asarray([sorted(x.keys()) for x in self.c["internal_state"]], dtype=object)
            if self.c["internal_state"] else np.zeros((0,), dtype=object),
        }
        return P, C


def expected_counts(horizon_s: float) -> tuple[int, int]:
    """Exact endpoint-inclusion rule: N=T/dt physics samples (t=dt..T),
    M=T/control_dt control samples (t=0..T-control_dt)."""
    n = int(round(float(horizon_s) / PHYSICS_DT))
    m = int(round(float(horizon_s) / CONTROL_DT))
    return n, m
