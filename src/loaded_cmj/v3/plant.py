"""V3 Plant runtime surface — elite-soccer human-valid loaded-CMJ Plant.

Authority: LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1.

This module loads ``assets/v3_plant.xml``, asserts the frozen compiled
identity, and exposes deterministic Plant primitives (pose construction,
plantar patch geometry, contact classification, MTP passive control) used by
the RES-83 audits and tests.

It contains no controller, no trajectory, no event/scorer logic and no
RES-84/RES-85 final calibration.  V1/V2 code is never imported.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from importlib import resources
from typing import Iterable, Mapping

import mujoco
import numpy as np

from loaded_cmj.v3.constants import (
    V3_ACTUATOR_NAMES,
    V3_ATHLETE_MASS_KG,
    V3_BODY_NAMES,
    V3_COMPILED_NA,
    V3_COMPILED_NBODY,
    V3_COMPILED_NEQ,
    V3_COMPILED_NGEOM,
    V3_COMPILED_NJNT,
    V3_COMPILED_NQ,
    V3_COMPILED_NU,
    V3_COMPILED_NV,
    V3_CONTYPE_FLOOR,
    V3_CONTYPE_PROHIBITED,
    V3_CONTYPE_SUPPORT,
    V3_FLOOR_GEOM,
    V3_GEOM_NAMES,
    V3_JOINT_NAMES,
    V3_JOINT_RANGES_RAD,
    V3_LOAD_MASS_KG,
    V3_MTP_JOINT_NAMES,
    V3_PLANTAR_SUPPORT_GEOMS,
    V3_PROHIBITED_FLOOR_GEOMS,
    V3_SIDES,
    V3_SUPPORT_GEOM_BY_FOOT_REGION,
    V3_SUPPORT_REGIONS,
    V3_SYSTEM_MASS_KG,
    V3_XML_FILENAME,
)

_ASSETS_PACKAGE = "loaded_cmj.v3.assets"

# geom name -> (side, region) for the legal plantar support patches
SUPPORT_GEOM_TO_SIDE_REGION = {
    V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]: (side, region)
    for side in V3_SIDES
    for region in V3_SUPPORT_REGIONS
}


class V3PlantError(RuntimeError):
    """Raised when the V3 Plant does not match the frozen RES-95 identity."""


@dataclass(frozen=True)
class V3Indices:
    body: Mapping[str, int]
    joint: Mapping[str, int]
    actuator: Mapping[str, int]
    geom: Mapping[str, int]
    qadr: Mapping[str, int]
    vadr: Mapping[str, int]


def model_xml() -> str:
    """Return the tracked V3 Plant XML exactly as sealed."""
    return resources.files(_ASSETS_PACKAGE).joinpath(V3_XML_FILENAME).read_text()


def build_model() -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_string(model_xml())
    return model


def build_zero_passive_model() -> mujoco.MjModel:
    """Instantiate the FM-09 zero-passive MTP sensitivity case.

    The zero-passive case must remain representable with the active MTP
    channels present and zeroed (AUTHORITY_INPUTS.materials.zero_passive_case_required).
    """
    spec = mujoco.MjSpec.from_string(model_xml())
    zeros = np.zeros(3, dtype=np.float64)
    for name in V3_MTP_JOINT_NAMES:
        joint = spec.joint(name)
        joint.stiffness = zeros
        joint.damping = zeros
    return spec.compile()


def resolve_indices(model: mujoco.MjModel) -> V3Indices:
    def _id(objtype: mujoco.mjtObj, name: str) -> int:
        idx = mujoco.mj_name2id(model, objtype, name)
        if idx < 0:
            raise V3PlantError(f"missing {objtype} named {name!r}")
        return int(idx)

    body = {n: _id(mujoco.mjtObj.mjOBJ_BODY, n) for n in V3_BODY_NAMES}
    joint = {n: _id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in V3_JOINT_NAMES}
    actuator = {n: _id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in V3_ACTUATOR_NAMES}
    geom = {n: _id(mujoco.mjtObj.mjOBJ_GEOM, n) for n in V3_GEOM_NAMES}
    return V3Indices(
        body=body,
        joint=joint,
        actuator=actuator,
        geom=geom,
        qadr={n: int(model.jnt_qposadr[joint[n]]) for n in V3_JOINT_NAMES},
        vadr={n: int(model.jnt_dofadr[joint[n]]) for n in V3_JOINT_NAMES},
    )


def body_inertia_full(model: mujoco.MjModel, body_id: int) -> np.ndarray:
    """Reconstruct the 3x3 inertia tensor about the body-frame COM."""
    quat = np.asarray(model.body_iquat[body_id], dtype=np.float64)
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    rot = rot.reshape(3, 3)
    return rot @ np.diag(np.asarray(model.body_inertia[body_id], dtype=np.float64)) @ rot.T


class V3Plant:
    """Deterministic runtime surface for the frozen V3 Plant."""

    def __init__(self, model: mujoco.MjModel | None = None) -> None:
        self.model = build_model() if model is None else model
        self.idx = resolve_indices(self.model)
        self._assert_identity()

    # ------------------------------------------------------------------
    # identity
    # ------------------------------------------------------------------
    def _assert_identity(self) -> None:
        m = self.model
        checks = {
            "nbody": V3_COMPILED_NBODY,
            "njnt": V3_COMPILED_NJNT,
            "nq": V3_COMPILED_NQ,
            "nv": V3_COMPILED_NV,
            "nu": V3_COMPILED_NU,
            "neq": V3_COMPILED_NEQ,
            "na": V3_COMPILED_NA,
            "ngeom": V3_COMPILED_NGEOM,
        }
        for key, want in checks.items():
            got = int(getattr(m, key))
            if got != want:
                raise V3PlantError(f"compiled {key}={got} != authority {want}")
        if int(m.ntendon) != 0:
            raise V3PlantError(f"compiled ntendon={m.ntendon} != authority 0")
        total = float(m.body_mass.sum())
        if abs(total - V3_SYSTEM_MASS_KG) > 1e-9:
            raise V3PlantError(f"compiled total mass {total} != {V3_SYSTEM_MASS_KG}")
        athlete = total - float(m.body_mass[self.idx.body["bar"]])
        if abs(athlete - V3_ATHLETE_MASS_KG) > 1e-9:
            raise V3PlantError(f"compiled athlete mass {athlete} != {V3_ATHLETE_MASS_KG}")
        bar = float(m.body_mass[self.idx.body["bar"]])
        if abs(bar - V3_LOAD_MASS_KG) > 1e-9:
            raise V3PlantError(f"compiled bar mass {bar} != {V3_LOAD_MASS_KG}")

    # ------------------------------------------------------------------
    # data lifecycle
    # ------------------------------------------------------------------
    def make_data(self) -> mujoco.MjData:
        return mujoco.MjData(self.model)

    def reset(self, data: mujoco.MjData, qpos: Iterable[float] | None = None) -> None:
        mujoco.mj_resetData(self.model, data)
        if qpos is not None:
            data.qpos[:] = np.asarray(qpos, dtype=np.float64)
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, data)

    def set_joint_angles(self, data: mujoco.MjData, angles: Mapping[str, float]) -> None:
        for name, value in angles.items():
            if name not in self.idx.qadr:
                raise V3PlantError(f"unknown joint {name!r}")
            data.qpos[self.idx.qadr[name]] = float(value)
        mujoco.mj_forward(self.model, data)

    def set_mtp_passive(self, stiffness: float, damping: float) -> None:
        """Switch the MTP passive prior (e.g. the FM-09 zero-passive case)."""
        for name in V3_MTP_JOINT_NAMES:
            jid = self.idx.joint[name]
            dof = int(self.model.jnt_dofadr[jid])
            self.model.jnt_stiffness[jid] = float(stiffness)
            self.model.dof_damping[dof] = float(damping)

    # ------------------------------------------------------------------
    # geometry helpers
    # ------------------------------------------------------------------
    def _box_corners(self, data: mujoco.MjData, geom_id: int) -> np.ndarray:
        size = np.asarray(self.model.geom_size[geom_id], dtype=np.float64)
        rot = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
        pos = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
        signs = np.array(
            [(sx, sy, sz) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)],
            dtype=np.float64,
        )
        offsets = (signs * size) @ rot.T
        return pos + offsets

    def support_patch_corners(self, data: mujoco.MjData) -> dict[tuple[str, str], np.ndarray]:
        """World corners of every legal plantar support patch."""
        corners: dict[tuple[str, str], np.ndarray] = {}
        for side in V3_SIDES:
            for region in V3_SUPPORT_REGIONS:
                gid = self.idx.geom[V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]
                corners[(side, region)] = self._box_corners(data, gid)
        return corners

    def support_min_z(self, data: mujoco.MjData) -> float:
        corners = self.support_patch_corners(data)
        return float(min(c[:, 2].min() for c in corners.values()))

    def drop_to_floor(self, data: mujoco.MjData, clearance_m: float = 0.0) -> float:
        """Translate root_tz so the lowest legal patch corner is at ``clearance_m``."""
        mujoco.mj_forward(self.model, data)
        delta = clearance_m - self.support_min_z(data)
        data.qpos[self.idx.qadr["root_tz"]] += delta
        mujoco.mj_forward(self.model, data)
        return float(delta)

    def press_into_floor(self, data: mujoco.MjData, depth_m: float) -> None:
        """Translate root_tz down by ``depth_m`` to force floor contact registration.

        Used only by the deterministic contact-region probes; the depth is an
        explicit probe parameter (PROVISIONAL_NUMERICAL, not a calibration).
        """
        if depth_m <= 0.0:
            raise V3PlantError("press depth must be positive")
        data.qpos[self.idx.qadr["root_tz"]] -= float(depth_m)
        mujoco.mj_forward(self.model, data)

    def balance_root_x(self, data: mujoco.MjData, tol_m: float = 0.01) -> float:
        """Translate root_tx so the 99 kg system COM sits over the active support centre."""
        mujoco.mj_forward(self.model, data)
        com = np.asarray(data.subtree_com[self.idx.body["pelvis"]], dtype=np.float64)
        corners = self.support_patch_corners(data)
        all_corners = np.vstack(list(corners.values()))
        min_z = float(all_corners[:, 2].min())
        active = all_corners[all_corners[:, 2] <= min_z + tol_m]
        support_cx = float(active[:, 0].mean())
        delta = support_cx - float(com[0])
        data.qpos[self.idx.qadr["root_tx"]] += delta
        mujoco.mj_forward(self.model, data)
        return float(delta)

    # ------------------------------------------------------------------
    # contact classification
    # ------------------------------------------------------------------
    def geom_name(self, geom_id: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""

    def body_name(self, body_id: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(body_id)) or ""

    def contacts(self, data: mujoco.MjData) -> list[dict[str, object]]:
        """Deterministic classified contact list."""
        out: list[dict[str, object]] = []
        for i in range(data.ncon):
            c = data.contact[i]
            g1 = self.geom_name(c.geom1)
            g2 = self.geom_name(c.geom2)
            b1 = self.body_name(self.model.geom_bodyid[c.geom1])
            b2 = self.body_name(self.model.geom_bodyid[c.geom2])
            if g1 == V3_FLOOR_GEOM or g2 == V3_FLOOR_GEOM:
                other = g2 if g1 == V3_FLOOR_GEOM else g1
                if other in V3_PLANTAR_SUPPORT_GEOMS:
                    kind = "legal_plantar_floor"
                    side, region = SUPPORT_GEOM_TO_SIDE_REGION[other]
                elif other in V3_PROHIBITED_FLOOR_GEOMS:
                    kind = "prohibited_floor"
                    side, region = None, None
                else:
                    kind = "other_floor"
                    side, region = None, None
                out.append({
                    "kind": kind,
                    "geom1": g1,
                    "geom2": g2,
                    "body1": b1,
                    "body2": b2,
                    "side": side,
                    "region": region,
                    "dist_m": float(c.dist),
                    "penetration_m": max(0.0, -float(c.dist)),
                })
            else:
                out.append({
                    "kind": "self",
                    "geom1": g1,
                    "geom2": g2,
                    "body1": b1,
                    "body2": b2,
                    "side": None,
                    "region": None,
                    "dist_m": float(c.dist),
                    "penetration_m": max(0.0, -float(c.dist)),
                })
        return out

    def legal_support_contacts(self, data: mujoco.MjData) -> list[dict[str, object]]:
        return [c for c in self.contacts(data) if c["kind"] == "legal_plantar_floor"]

    def prohibited_contacts(self, data: mujoco.MjData) -> list[dict[str, object]]:
        return [c for c in self.contacts(data) if c["kind"] == "prohibited_floor"]

    def self_contacts(self, data: mujoco.MjData) -> list[dict[str, object]]:
        return [c for c in self.contacts(data) if c["kind"] == "self"]

    # ------------------------------------------------------------------
    # collision policy introspection
    # ------------------------------------------------------------------
    def geom_collision_class(self, geom_id: int) -> str:
        contype = int(self.model.geom_contype[geom_id])
        if contype == V3_CONTYPE_FLOOR:
            return "floor"
        if contype == V3_CONTYPE_SUPPORT:
            return "support"
        if contype == V3_CONTYPE_PROHIBITED:
            return "prohibited"
        return f"unknown:{contype}"


# ---------------------------------------------------------------------------
# Representative pose library (RES-83 configuration reachability)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class V3PoseSpec:
    name: str
    trunk_deg: float
    hip_deg: float
    knee_deg: float
    ankle_deg: float | None = None
    foot_pitch_deg: float | None = 0.0
    mtp_deg: float | None = None
    root_ry_deg: float = 0.0
    clearance_m: float = 0.0
    balance: bool = True
    note: str = field(default="")

    def ankle_rad(self) -> float:
        if self.ankle_deg is not None:
            return math.radians(self.ankle_deg)
        pitch = math.radians(self.foot_pitch_deg or 0.0)
        return math.radians(self.root_ry_deg - self.hip_deg + self.knee_deg) - pitch

    def mtp_rad(self) -> float:
        if self.mtp_deg is not None:
            return math.radians(self.mtp_deg)
        return math.radians(self.foot_pitch_deg or 0.0)


def representative_poses() -> tuple[V3PoseSpec, ...]:
    """Deterministic legal/representative configurations used by the audits."""
    return (
        V3PoseSpec("legal_standing", 0.0, 0.0, 0.0, foot_pitch_deg=0.0,
                   note="flat-foot standing, both feet plantar support, zero penetration"),
        V3PoseSpec("deep_legal_countermovement", 25.0, 85.0, 105.0, foot_pitch_deg=0.0,
                   note="deep sagittal countermovement within structural ROM, feet flat"),
        V3PoseSpec("braking", 15.0, 45.0, 70.0, foot_pitch_deg=0.0,
                   note="braking configuration, heels down, knees over feet"),
        V3PoseSpec("propulsion", 10.0, 15.0, 25.0, foot_pitch_deg=25.0,
                   note="propulsion, ankle negative (plantarflexion), heel rising, MTP dorsiflexed"),
        V3PoseSpec("heel_rise_toe_rocker", 5.0, 5.0, 20.0, foot_pitch_deg=40.0,
                   note="heel-rise toe-rocker, forefoot/toe support only"),
        V3PoseSpec("toe_off", 0.0, 0.0, 10.0, foot_pitch_deg=50.0,
                   note="toe-off, strongly plantarflexed ankle and dorsiflexed MTP"),
        V3PoseSpec("flight", 5.0, 40.0, 60.0, ankle_deg=-25.0, foot_pitch_deg=None,
                   mtp_deg=10.0, clearance_m=0.08, balance=False,
                   note="flight, all plantar patches clear of the floor"),
        V3PoseSpec("toe_forefoot_first_touchdown", 10.0, 20.0, 30.0, foot_pitch_deg=25.0,
                   note="toe/forefoot-first touchdown, heel still above the floor"),
        V3PoseSpec("landing_absorption", 20.0, 60.0, 95.0, foot_pitch_deg=0.0,
                   note="landing absorption, feet flat, deep knee flexion"),
        V3PoseSpec("recovered_standing", 0.0, 0.0, 0.0, foot_pitch_deg=0.0,
                   note="recovered standing configuration"),
    )


def build_pose(plant: V3Plant, data: mujoco.MjData, spec: V3PoseSpec) -> dict[str, float]:
    """Materialize a pose deterministically and return the resulting joint angles (rad)."""
    plant.reset(data)
    ry = math.radians(spec.root_ry_deg)
    hip = math.radians(spec.hip_deg)
    knee = math.radians(spec.knee_deg)
    angles: dict[str, float] = {
        "root_ry": ry,
        "trunk_pelvis": math.radians(spec.trunk_deg),
        "left_hip": hip,
        "right_hip": hip,
        "left_knee": knee,
        "right_knee": knee,
        "left_ankle": spec.ankle_rad(),
        "right_ankle": spec.ankle_rad(),
        "left_mtp": spec.mtp_rad(),
        "right_mtp": spec.mtp_rad(),
    }
    plant.set_joint_angles(data, angles)
    plant.drop_to_floor(data, clearance_m=spec.clearance_m)
    if spec.balance:
        plant.balance_root_x(data)
    angles["root_tx"] = float(data.qpos[plant.idx.qadr["root_tx"]])
    angles["root_tz"] = float(data.qpos[plant.idx.qadr["root_tz"]])
    return angles


def pose_violations(plant: V3Plant, data: mujoco.MjData, spec: V3PoseSpec) -> list[str]:
    """Return the list of structural violations of a materialized pose (empty = legal)."""
    violations: list[str] = []
    for name in V3_JOINT_NAMES:
        value = float(data.qpos[plant.idx.qadr[name]])
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        if value < rng[0] - 1e-9 or value > rng[1] + 1e-9:
            violations.append(f"joint_limit:{name}={value}")
    if spec.clearance_m == 0.0 and spec.balance:
        if plant.support_min_z(data) < -1e-6:
            violations.append("support_penetration")
    if plant.prohibited_contacts(data):
        violations.append("prohibited_floor_contact")
    if plant.self_contacts(data):
        violations.append("self_intersection")
    return violations
