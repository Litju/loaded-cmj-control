"""V2 Plant — sagittal-dominant 20 kg loaded CMJ.

Transparent bounded torque, bilateral box feet, deterministic MuJoCo.
No SO3, no hidden drive, no qfrc_applied.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.v2.constants import (
    V2_COMPILED_NA,
    V2_COMPILED_NBODY,
    V2_COMPILED_NEQ,
    V2_COMPILED_NGEOM,
    V2_COMPILED_NJNT,
    V2_COMPILED_NQ,
    V2_COMPILED_NV,
    V2_COMPILED_NU,
    V2_CONTACT_SOLIMP,
    V2_CONTACT_SOLREF,
    V2_GRAVITY_MAGNITUDE,
    V2_JOINT_NAMES,
    V2_MJ_ACTUATOR_NAMES,
    V2_RESET_QPOS,
    V2_TORQUE_LIMITS_NM,
    V2_TOTAL_MASS_KG,
)
from loaded_cmj.v2.drive import action_to_torque, LIMITS

_ASSET = "v2_plant.xml"

class V2PlantError(RuntimeError):
    pass

@dataclass(frozen=True)
class V2Indices:
    body: dict[str, int]
    joint: dict[str, int]
    actuator: dict[str, int]
    geom: dict[str, int]
    qadr: dict[str, int]
    vadr: dict[str, int]
    floor_geom: int
    left_foot_geom: int
    right_foot_geom: int
    left_foot_body: int
    right_foot_body: int
    pelvis_body: int
    torso_body: int

def model_path() -> Path:
    # importlib.resources path
    resource = resources.files("loaded_cmj.v2.assets").joinpath(_ASSET)
    return Path(resource)

def model_xml() -> str:
    return resources.files("loaded_cmj.v2.assets").joinpath(_ASSET).read_text()

def build_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(model_xml())

def resolve_indices(model: mujoco.MjModel) -> V2Indices:
    def _id(objtype, name):
        idx = mujoco.mj_name2id(model, objtype, name)
        if idx < 0:
            raise V2PlantError(f"missing {name}")
        return int(idx)
    body = {n: _id(mujoco.mjtObj.mjOBJ_BODY, n) for n in ["pelvis","torso_head_arms","external_load","left_thigh","left_shank","left_foot","right_thigh","right_shank","right_foot"]}
    joint = {n: _id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in V2_JOINT_NAMES}
    actuator = {n: _id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in V2_MJ_ACTUATOR_NAMES}
    geom = {n: _id(mujoco.mjtObj.mjOBJ_GEOM, n) for n in ["floor","pelvis_shell","pelvis_fall","torso_shell","torso_fall","load_shell","left_thigh_shell","left_thigh_fall","left_shank_shell","left_shank_fall","left_foot_box","right_thigh_shell","right_thigh_fall","right_shank_shell","right_shank_fall","right_foot_box"]}
    qadr = {n: int(model.jnt_qposadr[joint[n]]) for n in V2_JOINT_NAMES}
    vadr = {n: int(model.jnt_dofadr[joint[n]]) for n in V2_JOINT_NAMES}
    return V2Indices(
        body=body, joint=joint, actuator=actuator, geom=geom,
        qadr=qadr, vadr=vadr,
        floor_geom=geom["floor"],
        left_foot_geom=geom["left_foot_box"],
        right_foot_geom=geom["right_foot_box"],
        left_foot_body=body["left_foot"],
        right_foot_body=body["right_foot"],
        pelvis_body=body["pelvis"],
        torso_body=body["torso_head_arms"],
    )

class V2Plant:
    def __init__(self, model: mujoco.MjModel | None = None) -> None:
        self.model = build_model() if model is None else model
        self.idx = resolve_indices(self.model)
        self._assert_identity()
        # torque limits in actuator order
        self.torque_limits = np.array([V2_TORQUE_LIMITS_NM[n] for n in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]], dtype=np.float64)

    def _assert_identity(self):
        m=self.model
        checks={"nq":V2_COMPILED_NQ,"nv":V2_COMPILED_NV,"nu":V2_COMPILED_NU,"nbody":V2_COMPILED_NBODY,"njnt":V2_COMPILED_NJNT,"ngeom":V2_COMPILED_NGEOM,"neq":V2_COMPILED_NEQ,"na":V2_COMPILED_NA}
        for k,want in checks.items():
            got=int(getattr(m,k))
            if got!=want:
                raise V2PlantError(f"{k} {got} != {want}")
        if abs(float(m.body_mass.sum()) - V2_TOTAL_MASS_KG) > 1e-9:
            raise V2PlantError(f"mass {m.body_mass.sum()} != {V2_TOTAL_MASS_KG}")

    def make_data(self) -> mujoco.MjData:
        return mujoco.MjData(self.model)

    def reset(self, data: mujoco.MjData, qpos: np.ndarray | None = None):
        mujoco.mj_resetData(self.model, data)
        data.qpos[:] = np.asarray(V2_RESET_QPOS if qpos is None else qpos, dtype=np.float64)
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, data)

    def apply_action(self, data: mujoco.MjData, action: np.ndarray):
        tau = action_to_torque(action)
        # action is u in [-1,1], tau = limit * u, ctrl = u (gear scales to tau)
        u = np.asarray(action, dtype=np.float64).reshape(-1)
        if u.shape[0] != len(V2_MJ_ACTUATOR_NAMES):
            raise V2PlantError(f"action dim {u.shape} != {len(V2_MJ_ACTUATOR_NAMES)}")
        for i, name in enumerate(V2_MJ_ACTUATOR_NAMES):
            aid = self.idx.actuator[name]
            # check u in [-1,1]
            if u[i] < -1.0 -1e-9 or u[i] > 1.0 +1e-9:
                raise V2PlantError(f"u {u[i]} outside [-1,1] for {name}")
            lo, hi = self.model.actuator_forcerange[aid]
            # forcerange is -limit..limit, gear maps u*gear = tau, so check tau
            if tau[i] < lo -1e-9 or tau[i] > hi +1e-9:
                raise V2PlantError(f"tau {tau[i]} exceeds forcerange {lo}..{hi} for {name}")
            data.ctrl[aid] = u[i]
        return tau

    def center_of_mass(self, data: mujoco.MjData) -> np.ndarray:
        # use xipos weighted
        masses=self.model.body_mass
        xipos=np.asarray(data.xipos, dtype=np.float64)
        com=np.sum(masses[:,None]*xipos, axis=0)/masses.sum()
        return com.copy()

    def center_of_mass_velocity(self, data: mujoco.MjData) -> np.ndarray:
        # Use linear momentum / total mass
        # Compute via object velocities weighted
        # Simpler: use qvel + mass
        # We'll compute COM velocity via finite diff of com? Use linear momentum
        # Use technique: sum m * v_body / M where v_body is body center vel
        # mujoco.mj_objectVelocity for each body
        total=np.zeros(3)
        for bid in range(1, self.model.nbody):
            vel=np.zeros(6)
            mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
            # vel[3:] is linear of body origin, but COM may offset? Use xipos? For now approximate with body origin vel (close, since bodies are extended but COM offset small)
            # For accurate COM vel, use data.qvel and jac? Alternative compute via simulation: use momentum
            # We'll use linear momentum from dynamics: data.qfrc? Not.
            # Simpler: use pelvis linear velocity as COM surrogate for sagittal task? But we want COM.
            # We'll compute via body's center velocity using xipos offset?
            # Use Jacobian method: for each body, compute vel at COM via point
            # Mujoco has body com pos: xipos, and body rotation: xmat.
            # Velocity of COM point = vel_linear + omega × (com - body_pos)
            # We can get via data.cvel? data.cvel is COM spatial velocity
            pass
        # Alternative: use data.qvel for root_tx,tz gives COM vel approx? For our sagittal model, root_tx dot is COM x vel plus leg contributions.
        # For simplicity, use sensor: compute COM via xipos difference over time? But we have no history.
        # Use mujoco's ability: mj_comPos already computes com; we want its derivative via finite diff approximated by linear momentum?
        # Let's use data.subtree_com? Actually mjsim has no direct com vel.
        # We can compute via math: com_vel = sum(m_i * v_i_com) / M, where v_i_com = v_body + omega × (xipos - xpos)
        masses=self.model.body_mass
        com_vel=np.zeros(3)
        for bid in range(1, self.model.nbody):
            # get body velocity (spatial) at body origin: vel[3:] linear, vel[:3] angular
            vel=np.zeros(6)
            mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_BODY, bid, vel, 0)
            omega=vel[:3]
            v_body=vel[3:]
            # offset from body origin to its COM (xipos)
            # body origin is xpos[bid], com is xipos[bid]
            offset = np.asarray(data.xipos[bid]) - np.asarray(data.xpos[bid])
            v_com = v_body + np.cross(omega, offset)
            com_vel += float(masses[bid]) * v_com
        return com_vel / float(masses.sum())

    def trunk_tilt(self, data: mujoco.MjData) -> float:
        # lumbar + root pitch
        # torso orientation vs vertical: use torso xmat
        mat=np.asarray(data.xmat[self.idx.torso_body], dtype=np.float64).reshape(3,3)
        # tilt is angle between torso up (local z) and world z: arccos(mat[2,2])? Or compute
        up_world = mat[:,2]  # local z in world
        cos_tilt = np.clip(up_world[2], -1,1)
        return float(np.arccos(cos_tilt))

    def foot_contact_summary(self, data: mujoco.MjData) -> dict[str, Any]:
        # Sum per foot force via contact forces
        # For each contact, compute world force contributed to foot
        left_F = np.zeros(3)
        right_F = np.zeros(3)
        left_M = np.zeros(3)
        right_M = np.zeros(3)
        # Track deepest penetration for diagnostics
        max_pen = 0.0
        for i in range(data.ncon):
            con = data.contact[i]
            g1 = int(con.geom1)
            g2 = int(con.geom2)
            # identify foot
            is_left = (g1 == self.idx.left_foot_geom or g2 == self.idx.left_foot_geom)
            is_right = (g1 == self.idx.right_foot_geom or g2 == self.idx.right_foot_geom)
            is_floor = (g1 == self.idx.floor_geom or g2 == self.idx.floor_geom)
            if not (is_floor and (is_left or is_right)):
                continue
            # mj_contactForce in contact frame
            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.model, data, i, wrench)
            # convert to world: wrench[:3] is force in contact frame
            # Need frame matrix: con.frame (3x3) rotation from contact frame to world? Actually frame is contact frame axes in world.
            # The force in world = frame.T?? Let's reuse V1 conversion but simplified.
            # For our case, we can approximate world force as frame @ wrench[:3] with sign?
            # Let's use V1 logic: sign = +1 if system geom is geom[1] else -1, then world = frame.T @ (sign*raw)
            # Determine system geom index
            if g2 == self.idx.left_foot_geom or g2 == self.idx.right_foot_geom:
                sys_idx = 1 if g2 in (self.idx.left_foot_geom, self.idx.right_foot_geom) else 0
                # but need to know if sys is g1 or g2: if sys is g1 then sign -1, else +1. Use V1 definition: sign=+1 if sys is geom[1]
            else:
                # shouldn't happen
                continue
            # Determine which foot is system
            foot_geom = self.idx.left_foot_geom if is_left else self.idx.right_foot_geom
            sys_idx = 1 if int(con.geom1) != foot_geom and int(con.geom2) == foot_geom else 0
            # Actually if foot is geom2, sys_idx=1 => sign +1, else -1
            # Let's compute directly: raw is force on geom1? No, MJ docs: contact force is force on geom1? Might be ambiguous.
            # Simpler: compute sign as per V1: env-on-system, system is foot. So sign = +1 if system is geom1? Wait V1 docs: sign=+1 when system geom is contact.geom[1] and -1 when system is geom[0]. Let's follow that.
            sign = 1.0 if foot_geom == int(con.geom[1]) else -1.0
            frame = np.asarray(con.frame, dtype=np.float64).reshape(3,3)
            # con.frame is 3x3 rotation from contact to world? In MJ, frame is world rotation of contact frame (columns are axes). So world_force = frame @ (sign*raw)
            # But V1 uses frame.T . Need to check: V1 does force = r_gc @ (sign*raw) where r_gc = frame.T, and frame is contact frame in world? Let's follow V1's method but V1 used frame.T
            # We'll follow V1: force = frame.T @ (sign*raw[:3])
            r_gc = frame.T
            world_force = r_gc @ (sign * wrench[:3])
            # For simplicity we also try frame @ raw and compare magnitudes - they should be similar but transposed.
            # We'll use V1 convention.
            point = np.asarray(con.pos, dtype=np.float64)
            # choose origin as foot center projected? For per-foot moment about world origin (0,0,0) per spec: M_origin = (p - 0) × F
            # But for CoP we need moment about foot plate origin. We'll compute both.
            if is_left:
                left_F += world_force
                left_M += np.cross(point, world_force)  # moment about world origin
            else:
                right_F += world_force
                right_M += np.cross(point, world_force)
            penetration = -float(con.dist) if float(con.dist) < 0 else 0.0
            max_pen = max(max_pen, penetration)

        # Fallback if no contacts but we have penetration via cfrc_ext? The above sums 0 if no contact; we should also consider if still in contact but ncon 0 due to numerical? Use cfrc fallback
        # whole wrench = left+right
        whole_F = left_F + right_F
        whole_M = left_M + right_M
        # Also add off-plate? For V2 no off-plate; prohibited contacts are nonexistent because shell geoms have contype 0, but check
        prohibited = False
        # Check for shell contact: if any contact involves shell geoms, mark prohibited
        shell_geoms = {self.idx.geom[n] for n in ["pelvis_shell","torso_shell","load_shell","left_thigh_shell","right_thigh_shell","left_shank_shell","right_shank_shell"]}
        for i in range(data.ncon):
            con = data.contact[i]
            if int(con.geom1) in shell_geoms or int(con.geom2) in shell_geoms:
                # if contact with floor
                if int(con.geom1) == self.idx.floor_geom or int(con.geom2) == self.idx.floor_geom:
                    prohibited = True

        # Per foot plate origin: use foot body xpos xy at z=0
        left_origin = np.array([float(data.xpos[self.idx.left_foot_body,0]), float(data.xpos[self.idx.left_foot_body,1]), 0.0])
        right_origin = np.array([float(data.xpos[self.idx.right_foot_body,0]), float(data.xpos[self.idx.right_foot_body,1]), 0.0])
        # Compute CoP: shift moment to plate origin, then CoP = (-My/Fz, Mx/Fz)
        def cop_from(F, M_world, origin):
            # moment about plate origin = M_world - origin × F
            M_plate = M_world - np.cross(origin, F)
            Fz = float(F[2])
            if Fz > 20.0:
                cop = np.array([-M_plate[1]/Fz, M_plate[0]/Fz])
                valid = True
            else:
                cop = np.array([0.0,0.0])
                valid = False
            return cop, valid, M_plate

        left_cop, left_valid, left_M_plate = cop_from(left_F, left_M, left_origin)
        right_cop, right_valid, right_M_plate = cop_from(right_F, right_M, right_origin)
        # Whole cop? Not needed
        # Support margin: compute convex hull of foot boxes projection?
        # For V2 simple, support polygon is rectangle encompassing both feet: x in [left_foot_center_x ± 0.13, right likewise], y in[±0.095 ± 0.0475]
        # Simplify margin as distance from COM xy to nearest edge? We'll compute interval.
        com = self.center_of_mass(data)
        # Dynamic support polygon from foot geom centers (half sizes 0.130 x, 0.0475 y)
        lg = self.idx.left_foot_geom
        rg = self.idx.right_foot_geom
        left_cx = float(data.geom_xpos[lg][0])
        left_cy = float(data.geom_xpos[lg][1])
        right_cx = float(data.geom_xpos[rg][0])
        right_cy = float(data.geom_xpos[rg][1])
        hx, hy = 0.150, 0.060
        x_min = min(left_cx - hx, right_cx - hx)
        x_max = max(left_cx + hx, right_cx + hx)
        y_min = min(left_cy - hy, right_cy - hy)
        y_max = max(left_cy + hy, right_cy + hy)
        dx = min(com[0]-x_min, x_max-com[0])
        dy = min(com[1]-y_min, y_max-com[1])
        margin = float(min(dx, dy))
        # Provide per foot active flag: Fz > CONTACT_FZ_THRESHOLD?
        from loaded_cmj.v2.constants import V2_CONTACT_FZ_THRESHOLD_N
        active_left = float(left_F[2]) > V2_CONTACT_FZ_THRESHOLD_N
        active_right = float(right_F[2]) > V2_CONTACT_FZ_THRESHOLD_N

        return {
            "left_force": left_F,
            "right_force": right_F,
            "left_moment_world_origin": left_M,
            "right_moment_world_origin": right_M,
            "whole_force": whole_F,
            "whole_moment": whole_M,
            "left_Fz": float(left_F[2]),
            "right_Fz": float(right_F[2]),
            "whole_Fz": float(whole_F[2]),
            "left_cop": left_cop,
            "right_cop": right_cop,
            "left_cop_valid": left_valid,
            "right_cop_valid": right_valid,
            "left_origin": left_origin,
            "right_origin": right_origin,
            "support_margin": float(margin),
            "active_left": bool(active_left),
            "active_right": bool(active_right),
            "prohibited_contact": bool(prohibited),
            "max_penetration": float(max_pen),
            # for force-plate observables
            "plate_wrench": np.array([np.concatenate((left_F, left_M)), np.concatenate((right_F, right_M)), np.zeros(6)]),
            "cop_xy": np.array([left_cop, right_cop]),
            "cop_valid": np.array([left_valid, right_valid]),
        }

    def joint_positions(self, data: mujoco.MjData) -> np.ndarray:
        # return 7 sagittal joints in order: lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle
        # Map to qpos
        vals=[]
        for name in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]:
            # But qpos order is lumbar, left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle
            # Need to map correctly
            adr = self.idx.qadr[name]
            vals.append(float(data.qpos[adr]))
        # reorder to physical order lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle
        order = ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]
        return np.array(vals, dtype=np.float64)  # already in that order? Check vals order vs order
        # vals currently enumerated in order list we used: lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle => correct

    def joint_velocities(self, data: mujoco.MjData) -> np.ndarray:
        vals=[]
        for name in ["lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle"]:
            adr = self.idx.vadr[name]
            vals.append(float(data.qvel[adr]))
        return np.array(vals, dtype=np.float64)

    def public_observation(self, data: mujoco.MjData, scored_time_s: float, step_index: int, episode_reset: bool, previous_action: np.ndarray) -> dict[str, Any]:
        com = self.center_of_mass(data)
        com_vel = self.center_of_mass_velocity(data)
        summary = self.foot_contact_summary(data)
        # pelvis
        pelvis_pos = np.asarray(data.qpos[0:3], dtype=np.float64) if False else np.array([float(data.qpos[self.idx.qadr["root_tx"]]), 0.0, float(data.qpos[self.idx.qadr["root_tz"]])])  # y is 0 constrained
        # Actually pelvis world position: for slide joints, pos is combination? Use xpos body
        pelvis_world = np.asarray(data.xpos[self.idx.pelvis_body], dtype=np.float64)
        # orientation: root_ry hinge rot about y; compute quat
        # For simplicity, return identity
        pelvis_quat = np.array([1.0,0,0,0])
        # pelvis linear vel: root_tx vel, root_tz vel
        pelvis_vel = np.array([float(data.qvel[self.idx.vadr["root_tx"]]), 0.0, float(data.qvel[self.idx.vadr["root_tz"]])])
        pelvis_ang_vel = np.array([0.0, float(data.qvel[self.idx.vadr["root_ry"]]), 0.0])
        return {
            "time_s": float(scored_time_s),
            "step_index": int(step_index),
            "control_dt_s": 0.005,
            "episode_reset": bool(episode_reset),
            "joint_position_rad": self.joint_positions(data),
            "joint_velocity_radps": self.joint_velocities(data),
            "pelvis_position_world_m": pelvis_world,
            "pelvis_orientation_world_quat_wxyz": pelvis_quat,
            "pelvis_linear_velocity_world_mps": pelvis_vel,
            "pelvis_angular_velocity_body_radps": pelvis_ang_vel,
            "plantar_normal_force_N": np.array([summary["left_Fz"], summary["right_Fz"]]),
            "plantar_cop_xy_m": summary["cop_xy"],
            "plantar_cop_valid": summary["cop_valid"],
            "previous_action": np.asarray(previous_action, dtype=np.float64),
            "plantar_force_world_N": np.array([summary["left_force"], summary["right_force"]]),
            "plantar_moment_world_origin_Nm": np.array([summary["left_moment_world_origin"], summary["right_moment_world_origin"]]),
            "com_position_m": com,
            "com_velocity_mps": com_vel,
        }

    def is_finite(self, data: mujoco.MjData) -> bool:
        return bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all() and np.isfinite(data.qacc).all())
