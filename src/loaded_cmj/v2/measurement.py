"""V2 synchronized physics-sample measurement authority (RES-10 sync correction).

Normative contracts:
  docs/MUJOCO_STATE_STAGE_CONTRACT.md
  CONTROLLER_OBSERVATION_CONTRACT.md (this mission)

A sample labeled T must contain INTEGRATION state at T plus DERIVED
quantities recomputed at T via mj_forward on a shadow MjData.

Live MjData is NEVER forwarded for reporting; only the shadow is.
Live qpos/qvel/qacc_warmstart/ctrl/applied/solver/future trajectory are
never mutated by measurement.

Centralized API: SynchronizedPhysicsSample.from_live_state(...)
Use this everywhere instead of ad-hoc mj_forward calls.

CONTROL_SAMPLE_CONVENTION=SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL

At each control boundary t_k:
  x_k = exact live mjSTATE_INTEGRATION at t_k
  u_prev = control held over [t_{k-1}, t_k), still present in live.ctrl
  shadow <- x_k (mj_setState) ; mj_forward(shadow)  [forces under u_prev]
  obs_sync = controller_observation(sample, ...)  [all physical from shadow]
  u_k = controller(obs_sync)
  live.apply(u_k) ; step 40x to t_{k+1}

OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_CONTROL
NEW_CONTROL_EFFECTIVE_INTERVAL=[t_k, t_{k+1})
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np


STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION

CONTROL_SAMPLE_CONVENTION = "SAMPLE_SYNCHRONIZED_STATE_THEN_UPDATE_CONTROL"
OBSERVATION_INPUT_CONTROL = "PREVIOUS_HELD_CONTROL"

# Non-Plant metadata keys: explicitly external bookkeeping, never physics.
NON_PLANT_METADATA_KEYS = ("step_index", "episode_reset", "previous_action")


def state_size(model: mujoco.MjModel) -> int:
    return int(mujoco.mj_stateSize(model, STATE_SPEC))


def get_integration_vector(model: mujoco.MjModel, live: mujoco.MjData) -> np.ndarray:
    n = state_size(model)
    vec = np.zeros(n, dtype=np.float64)
    mujoco.mj_getState(model, live, vec, STATE_SPEC)
    return vec


def create_measurement_data(plant) -> mujoco.MjData:
    """Persistent shadow MjData for measurement only (never stepped)."""
    return plant.make_data()


def synchronize_measurement(
    model: mujoco.MjModel, live: mujoco.MjData, meas: mujoco.MjData
) -> mujoco.MjData:
    """Copy live mjSTATE_INTEGRATION into meas and mj_forward(meas).

    Does not mutate live in any way (read-only mj_getState on live).
    Never calls mj_forward on live. Returns meas for chaining.
    """
    n = state_size(model)
    vec = np.zeros(n, dtype=np.float64)
    mujoco.mj_getState(model, live, vec, STATE_SPEC)
    mujoco.mj_setState(model, meas, np.ascontiguousarray(vec), STATE_SPEC)
    mujoco.mj_forward(model, meas)
    return meas


def live_integration_hash(model: mujoco.MjModel, live: mujoco.MjData) -> str:
    import hashlib

    vec = get_integration_vector(model, live)
    return str(hashlib.sha256(np.ascontiguousarray(vec).tobytes()).hexdigest())


@dataclass(frozen=True)
class SynchronizedPhysicsSample:
    """One synchronized production sample at instant T.

    Single owner for controller, trace, event, scorer, forceplate.
    Every physical field comes from the SAME shadow mj_forward at T,
    except explicitly labeled non-Plant metadata (step_index,
    episode_reset, previous_action) supplied by the rollout loop.
    """

    STATE_TIME: float
    QACC_TIME: float
    CONTACT_TIME: float
    REPORT_TIME: float
    SYNCHRONIZED: bool
    time: float
    qpos: np.ndarray
    qvel: np.ndarray
    qacc: np.ndarray
    ctrl: np.ndarray
    qfrc_actuator: np.ndarray
    qfrc_passive: np.ndarray
    qfrc_constraint: np.ndarray
    ncon: int
    nefc: int
    state_vector_sha256: str
    # Extended full-sample authority (section 5)
    com_position_m: np.ndarray
    com_velocity_mps: np.ndarray
    trunk_tilt_rad: float
    joint_position_rad: np.ndarray
    joint_velocity_radps: np.ndarray
    pelvis_position_world_m: np.ndarray
    left_force_world_N: np.ndarray
    right_force_world_N: np.ndarray
    left_moment_world_origin_Nm: np.ndarray
    right_moment_world_origin_Nm: np.ndarray
    left_Fz_N: float
    right_Fz_N: float
    whole_Fz_N: float
    plantar_cop_xy_m: np.ndarray
    plantar_cop_valid: np.ndarray
    support_margin_m: float
    support_active: np.ndarray
    prohibited_contact: bool
    fall_contact: bool
    max_penetration_m: float

    @classmethod
    def from_live_state(cls, plant, live: mujoco.MjData, meas: mujoco.MjData) -> "SynchronizedPhysicsSample":
        """Build a synchronized sample without mutating live.

        Steps (mission section 4):
          1. leave LIVE untouched;
          2. capture live mjSTATE_INTEGRATION;
          3. restore into MEASUREMENT (shadow);
          4. mj_forward(measurement);
          5. read ALL derived from measurement;
          6. label with measurement.time.
        """
        import hashlib

        m = plant.model
        n = state_size(m)
        vec = np.zeros(n, dtype=np.float64)
        mujoco.mj_getState(m, live, vec, STATE_SPEC)
        vec_c = np.ascontiguousarray(vec)
        sha = str(hashlib.sha256(vec_c.tobytes()).hexdigest())
        mujoco.mj_setState(m, meas, vec_c, STATE_SPEC)
        mujoco.mj_forward(m, meas)
        t = float(meas.time)
        com = plant.center_of_mass(meas)
        cv = plant.center_of_mass_velocity(meas)
        sm = plant.foot_contact_summary(meas)
        # fall shells vs floor on synchronized contact table
        fall_geoms = {
            plant.idx.geom[nm]
            for nm in [
                "pelvis_fall",
                "torso_fall",
                "left_thigh_fall",
                "right_thigh_fall",
                "left_shank_fall",
                "right_shank_fall",
            ]
        }
        floor = plant.idx.floor_geom
        fall = False
        for i in range(meas.ncon):
            con = meas.contact[i]
            g1i, g2i = int(con.geom1), int(con.geom2)
            if g1i == floor or g2i == floor:
                other = g2i if g1i == floor else g1i
                if other in fall_geoms:
                    fall = True
                    break
        return cls(
            STATE_TIME=t,
            QACC_TIME=t,
            CONTACT_TIME=t,
            REPORT_TIME=t,
            SYNCHRONIZED=True,
            time=t,
            qpos=np.asarray(meas.qpos, dtype=np.float64).copy(),
            qvel=np.asarray(meas.qvel, dtype=np.float64).copy(),
            qacc=np.asarray(meas.qacc, dtype=np.float64).copy(),
            ctrl=np.asarray(meas.ctrl, dtype=np.float64).copy(),
            qfrc_actuator=np.asarray(meas.qfrc_actuator, dtype=np.float64).copy(),
            qfrc_passive=np.asarray(meas.qfrc_passive, dtype=np.float64).copy(),
            qfrc_constraint=np.asarray(meas.qfrc_constraint, dtype=np.float64).copy(),
            ncon=int(meas.ncon),
            nefc=int(meas.nefc),
            state_vector_sha256=sha,
            com_position_m=np.asarray(com, dtype=np.float64).copy(),
            com_velocity_mps=np.asarray(cv, dtype=np.float64).copy(),
            trunk_tilt_rad=float(plant.trunk_tilt(meas)),
            joint_position_rad=np.asarray(plant.joint_positions(meas), dtype=np.float64).copy(),
            joint_velocity_radps=np.asarray(plant.joint_velocities(meas), dtype=np.float64).copy(),
            pelvis_position_world_m=np.asarray(meas.xpos[plant.idx.pelvis_body], dtype=np.float64).copy(),
            left_force_world_N=np.asarray(sm["left_force"], dtype=np.float64).copy(),
            right_force_world_N=np.asarray(sm["right_force"], dtype=np.float64).copy(),
            left_moment_world_origin_Nm=np.asarray(sm["left_moment_world_origin"], dtype=np.float64).copy(),
            right_moment_world_origin_Nm=np.asarray(sm["right_moment_world_origin"], dtype=np.float64).copy(),
            left_Fz_N=float(sm["left_Fz"]),
            right_Fz_N=float(sm["right_Fz"]),
            whole_Fz_N=float(sm["whole_Fz"]),
            plantar_cop_xy_m=np.asarray(sm["cop_xy"], dtype=np.float64).copy(),
            plantar_cop_valid=np.asarray(sm["cop_valid"], dtype=bool).copy(),
            support_margin_m=float(sm["support_margin"]),
            support_active=np.asarray(
                [bool(sm["active_left"]), bool(sm["active_right"])], dtype=bool
            ).copy(),
            prohibited_contact=bool(sm["prohibited_contact"]),
            fall_contact=bool(fall),
            max_penetration_m=float(sm["max_penetration"]),
        )

    def check_time_identity(self) -> bool:
        return bool(
            self.SYNCHRONIZED
            and self.STATE_TIME == self.QACC_TIME == self.CONTACT_TIME == self.REPORT_TIME == self.time
        )

    def controller_observation(
        self, *, step_index: int, episode_reset: bool, previous_action: np.ndarray
    ) -> dict[str, Any]:
        """Complete public controller observation from THIS sample only.

        All physical fields come from the synchronized sample. The only
        non-Plant metadata (clearly labeled) are step_index, episode_reset,
        previous_action. previous_action must equal the held control u_prev
        whose forces are reported (caller's responsibility; checked by tests).
        """
        return {
            "time_s": float(self.time),
            "step_index": int(step_index),
            "control_dt_s": 0.005,
            "episode_reset": bool(episode_reset),
            "joint_position_rad": np.asarray(self.joint_position_rad, dtype=np.float64).copy(),
            "joint_velocity_radps": np.asarray(self.joint_velocity_radps, dtype=np.float64).copy(),
            "pelvis_position_world_m": np.asarray(self.pelvis_position_world_m, dtype=np.float64).copy(),
            "pelvis_orientation_world_quat_wxyz": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            "pelvis_linear_velocity_world_mps": np.asarray(
                [float(self.qvel[0]), 0.0, float(self.qvel[1])], dtype=np.float64
            ),
            "pelvis_angular_velocity_body_radps": np.asarray(
                [0.0, float(self.qvel[2]), 0.0], dtype=np.float64
            ),
            "plantar_normal_force_N": np.asarray(
                [self.left_Fz_N, self.right_Fz_N], dtype=np.float64
            ),
            "plantar_cop_xy_m": np.asarray(self.plantar_cop_xy_m, dtype=np.float64).copy(),
            "plantar_cop_valid": np.asarray(self.plantar_cop_valid, dtype=bool).copy(),
            "previous_action": np.asarray(previous_action, dtype=np.float64).copy(),
            "plantar_force_world_N": np.stack(
                [np.asarray(self.left_force_world_N), np.asarray(self.right_force_world_N)]
            ).astype(np.float64),
            "plantar_moment_world_origin_Nm": np.stack(
                [
                    np.asarray(self.left_moment_world_origin_Nm),
                    np.asarray(self.right_moment_world_origin_Nm),
                ]
            ).astype(np.float64),
            "com_position_m": np.asarray(self.com_position_m, dtype=np.float64).copy(),
            "com_velocity_mps": np.asarray(self.com_velocity_mps, dtype=np.float64).copy(),
            # observation authority certificate (one state -> one observation)
            "observation_state_sha256": str(self.state_vector_sha256),
            "observation_time_s": float(self.time),
            "OBSERVATION_INPUT_CONTROL": str(OBSERVATION_INPUT_CONTROL),
        }

    def event_sample(self) -> dict[str, Any]:
        """V2EventDetector input dict strictly from THIS synchronized sample."""
        return {
            "time_s": float(self.time),
            "com_z": float(self.com_position_m[2]),
            "com_vz": float(self.com_velocity_mps[2]),
            "com_x": float(self.com_position_m[0]),
            "whole_Fz": float(self.whole_Fz_N),
            "left_Fz": float(self.left_Fz_N),
            "right_Fz": float(self.right_Fz_N),
            "trunk_tilt": float(self.trunk_tilt_rad),
            "com_margin": float(self.support_margin_m),
            "prohibited": bool(self.prohibited_contact),
            "fall_contact": bool(self.fall_contact),
            "joint_position_rad": np.asarray(self.joint_position_rad, dtype=np.float64).tolist(),
            "qpos": np.asarray(self.qpos, dtype=np.float64).tolist(),
            "pelvis_position_world_m": np.asarray(
                self.pelvis_position_world_m, dtype=np.float64
            ).tolist(),
            "com_position_m": np.asarray(self.com_position_m, dtype=np.float64).tolist(),
            "STATE_TIME": float(self.time),
            "QACC_TIME": float(self.time),
            "CONTACT_TIME": float(self.time),
            "REPORT_TIME": float(self.time),
            "SYNCHRONIZED": True,
            "physics_sample_state_sha256": str(self.state_vector_sha256),
        }


def synchronized_observation_from_live(
    plant, live: mujoco.MjData, meas: mujoco.MjData, *, scored_time_s: float, step_index: int, episode_reset: bool, previous_action: np.ndarray
) -> dict[str, Any]:
    """Public observation evaluated at synchronized instant (shadow).

    Builds ONE SynchronizedPhysicsSample from live, then derives the
    controller observation from that same sample. All physical fields come
    from the shadow; scored_time_s must equal the sample time at a control
    boundary (asserted by callers via SHA identity tests).
    """
    sample = SynchronizedPhysicsSample.from_live_state(plant, live, meas)
    return sample.controller_observation(
        step_index=step_index, episode_reset=episode_reset, previous_action=previous_action
    )


def event_sample_from_measurement(plant, meas: mujoco.MjData, *, time_s: float) -> dict[str, Any]:
    """Build V2EventDetector input dict strictly from synchronized meas at T."""
    com = plant.center_of_mass(meas)
    cv = plant.center_of_mass_velocity(meas)
    sm = plant.foot_contact_summary(meas)
    # fall shells vs floor on synchronized contact table
    fall_geoms = {
        plant.idx.geom[n]
        for n in [
            "pelvis_fall",
            "torso_fall",
            "left_thigh_fall",
            "right_thigh_fall",
            "left_shank_fall",
            "right_shank_fall",
        ]
    }
    floor = plant.idx.floor_geom
    fall = False
    for i in range(meas.ncon):
        con = meas.contact[i]
        g1i, g2i = int(con.geom1), int(con.geom2)
        if g1i == floor or g2i == floor:
            other = g2i if g1i == floor else g1i
            if other in fall_geoms:
                fall = True
                break
    return {
        "time_s": float(time_s),
        "com_z": float(com[2]),
        "com_vz": float(cv[2]),
        "com_x": float(com[0]),
        "whole_Fz": float(sm["whole_Fz"]),
        "left_Fz": float(sm["left_Fz"]),
        "right_Fz": float(sm["right_Fz"]),
        "trunk_tilt": float(plant.trunk_tilt(meas)),
        "com_margin": float(sm["support_margin"]),
        "prohibited": bool(sm["prohibited_contact"]),
        "fall_contact": bool(fall),
        "joint_position_rad": plant.joint_positions(meas).tolist(),
        "qpos": np.asarray(meas.qpos).tolist(),
        "pelvis_position_world_m": np.asarray(meas.xpos[plant.idx.pelvis_body]).tolist(),
        "com_position_m": com.tolist(),
        # sample-semantics certificate
        "STATE_TIME": float(time_s),
        "QACC_TIME": float(time_s),
        "CONTACT_TIME": float(time_s),
        "REPORT_TIME": float(time_s),
        "SYNCHRONIZED": True,
    }


def legacy_event_sample_from_live(plant, live: mujoco.MjData, *, time_s: float) -> dict[str, Any]:
    """Legacy stale production sample (pre-correction) for old-vs-new comparison only."""
    com = plant.center_of_mass(live)
    cv = plant.center_of_mass_velocity(live)
    sm = plant.foot_contact_summary(live)
    fall_geoms = {
        plant.idx.geom[n]
        for n in [
            "pelvis_fall",
            "torso_fall",
            "left_thigh_fall",
            "right_thigh_fall",
            "left_shank_fall",
            "right_shank_fall",
        ]
    }
    floor = plant.idx.floor_geom
    fall = False
    for i in range(live.ncon):
        con = live.contact[i]
        g1i, g2i = int(con.geom1), int(con.geom2)
        if g1i == floor or g2i == floor:
            other = g2i if g1i == floor else g1i
            if other in fall_geoms:
                fall = True
                break
    return {
        "time_s": float(time_s),
        "com_z": float(com[2]),
        "com_vz": float(cv[2]),
        "com_x": float(com[0]),
        "whole_Fz": float(sm["whole_Fz"]),
        "left_Fz": float(sm["left_Fz"]),
        "right_Fz": float(sm["right_Fz"]),
        "trunk_tilt": float(plant.trunk_tilt(live)),
        "com_margin": float(sm["support_margin"]),
        "prohibited": bool(sm["prohibited_contact"]),
        "fall_contact": bool(fall),
        "joint_position_rad": plant.joint_positions(live).tolist(),
        "qpos": np.asarray(live.qpos).tolist(),
        "pelvis_position_world_m": np.asarray(live.xpos[plant.idx.pelvis_body]).tolist(),
        "com_position_m": com.tolist(),
    }
