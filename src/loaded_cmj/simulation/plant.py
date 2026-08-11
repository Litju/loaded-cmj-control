"""Compose and expose the Loaded CMJ Control plant.

The whole physics model is public and deterministic. The control problem is
defined by the plant and observation contracts rather than hidden dynamics.

This module writes ``data.ctrl`` and nothing else.  It never writes
``qfrc_applied`` or ``xfrc_applied``.  Every index is resolved by name once, in
``Plant.__init__``, and cached; there are no per-step name lookups and no
positional ``qpos``/``ctrl`` constants outside the single startup assertion.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from loaded_cmj.simulation import drive as drive_model, so3 as so3_map
from loaded_cmj.simulation.constants import (
    ACTION_CHANNELS,
    ACTION_DIM,
    ADMISSIBLE_CONTACT_GEOMS,
    ATHLETE_BODY_NAMES,
    BALL_ADMISSIBLE_BOX_RAD,
    BALL_JOINT_CHANNELS,
    BALL_JOINT_MJ_ACTUATORS,
    BALL_JOINT_NAMES,
    BODY_NAMES,
    BODY_WEIGHT_N,
    COMPILED_NA,
    COMPILED_NBODY,
    COMPILED_NEQ,
    COMPILED_NGEOM,
    COMPILED_NJNT,
    COMPILED_NQ,
    COMPILED_NU,
    COMPILED_NV,
    CONTACT_F_ACTIVE_N,
    CONTACT_F_COP_MIN_N,
    CONTACT_F_OFF_N,
    CONTACT_F_ON_N,
    CONTACT_GAP_MAX_M,
    CONTACT_N_OFF,
    CONTACT_N_ON,
    CONTACT_T_OFF_S,
    CONTACT_T_ON_S,
    CONTROL_PERIOD_S,
    FIXED_HOLD_SUBSTEPS,
    FLOOR_GEOM_NAME,
    GEOM_DISTANCE_CUTOFF_M,
    GRAVITY_MAGNITUDE,
    HINGE_CHANNEL_MJ_ACTUATOR,
    HINGE_JOINT_NAMES,
    HINGE_RANGES_RAD,
    HOLD_ACTION,
    HOLD_TAU_PREV_INITIAL,
    JOINT_NAMES,
    LEFT_PAD_GEOM_NAMES,
    LOAD_BODY_NAME,
    MJ_ACTUATOR_NAMES,
    MODEL_XML_FILENAME,
    OBSERVATION_FIELDS,
    PHYSICS_TIMESTEP_S,
    RESET_QPOS,
    RIGHT_PAD_GEOM_NAMES,
    ROOT_JOINT_NAME,
    SHELL_GEOM_NAMES,
    SUBSTEPS_PER_CONTROL,
    TOTAL_MASS_KG,
)

_ASSET_NAME = MODEL_XML_FILENAME

# Expected (qpos offset, qvel offset) per joint, in declaration order.  This is
# the normative target of the single startup assertion; trusted code resolves
# every offset by name and never slices positionally.
_EXPECTED_JOINT_OFFSETS: tuple[tuple[str, int, int], ...] = (
    ("root", 0, 0),
    ("lumbar", 7, 6),
    ("left_hip", 11, 9),
    ("left_knee_flexion", 15, 12),
    ("left_ankle_dorsiflexion", 16, 13),
    ("left_ankle_eversion", 17, 14),
    ("right_hip", 18, 15),
    ("right_knee_flexion", 22, 18),
    ("right_ankle_dorsiflexion", 23, 19),
    ("right_ankle_eversion", 24, 20),
)

_HINGE_CHANNEL_JOINT: tuple[tuple[int, str], ...] = (
    (9, "left_knee_flexion"),
    (10, "right_knee_flexion"),
    (11, "left_ankle_dorsiflexion"),
    (12, "right_ankle_dorsiflexion"),
    (13, "left_ankle_eversion"),
    (14, "right_ankle_eversion"),
)

# Public channel labels. The plant binds the final wire names here while
# preserving the positional action interface.
_AUTHORITY_ACTION_CHANNELS: tuple[str, ...] = (
    "lumbar_flexion", "lumbar_lateral", "lumbar_axial",
    "left_hip_flexion", "left_hip_abduction", "left_hip_rotation",
    "right_hip_flexion", "right_hip_abduction", "right_hip_rotation",
    "left_knee_flexion", "right_knee_flexion",
    "left_ankle_sagittal", "right_ankle_sagittal",
    "left_ankle_frontal", "right_ankle_frontal",
)

# Hinge channels retain the already-qualified MuJoCo joint signs.  These
# explicit values make the signed scalar map inspectable instead of allowing a
# later caller to infer it from positional order.
_HINGE_CHANNEL_SIGNS: dict[str, float] = {
    "left_knee_flexion": -1.0,
    "right_knee_flexion": -1.0,
    "left_ankle_sagittal": 1.0,
    "right_ankle_sagittal": 1.0,
    "left_ankle_frontal": 1.0,
    "right_ankle_frontal": -1.0,
}

# Deterministic equilibrium torque selection in the final anatomical basis.
# The public fixed-hold action and its previous realized torque are one source
# pair.  The normalized command is derived from the current final capacity
# envelope; it is not a free hard-coded controller action.
_EQUILIBRIUM_TAU = np.asarray(HOLD_TAU_PREV_INITIAL, dtype=np.float64).copy()

# Qualification-bound passive region parameters. The region is expressed
# in the public anatomical coordinate basis and does not alter the compiled
# Plant identity.  Values are deliberately kept in this module so the
# potential, gradient, taper and ledgers share one immutable declaration.
_REGION_RADIUS = np.asarray((
    2.50, 2.50, 2.50,
    2.50, 2.50, 2.50,
    2.50, 2.50, 2.50,
    2.50, 2.50, 2.50, 2.50, 2.50, 2.50,
), dtype=np.float64)
_REGION_CENTER = np.zeros(ACTION_DIM, dtype=np.float64)
_REGION_EXPONENT = 4
_REGION_LIMIT_K = 25.0
_REGION_TAPER_DELTA = 0.10
_DAMPING_COEFFICIENT = np.asarray((
    6.0, 6.0, 6.0,
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0,
    2.0, 2.0, 1.5, 1.5, 1.0, 1.0,
), dtype=np.float64)


def contact_wrench_from_raw(
    frame: np.ndarray,
    raw_wrench: np.ndarray,
    *,
    system_geom_index: int,
    contact_point: np.ndarray,
    plate_origin: np.ndarray,
) -> np.ndarray:
    """Convert one MuJoCo contact wrench to environment-on-system world form.

    ``mj_contactForce`` is expressed in the contact frame with the frozen
    geom-order convention.  The environment-on-system sign is ``+1`` when the
    system geom is ``contact.geom[1]`` and ``-1`` when it is
    ``contact.geom[0]``.  The moment is transported from the contact point to
    the requested plate origin.
    """
    f = np.asarray(frame, dtype=np.float64).reshape(3, 3)
    raw = np.asarray(raw_wrench, dtype=np.float64).reshape(6)
    point = np.asarray(contact_point, dtype=np.float64).reshape(3)
    origin = np.asarray(plate_origin, dtype=np.float64).reshape(3)
    if system_geom_index not in (0, 1):
        raise PlantIntegrityError("system_geom_index must be 0 or 1")
    sign = 1.0 if int(system_geom_index) == 1 else -1.0
    r_gc = f.T
    force = r_gc @ (sign * raw[:3])
    moment_contact = r_gc @ (sign * raw[3:])
    moment_origin = moment_contact + np.cross(point - origin, force)
    wrench = np.concatenate((force, moment_origin))
    if not np.isfinite(wrench).all():
        raise PlantIntegrityError("contact wrench conversion evaluated non-finite")
    return wrench


class PlantIntegrityError(RuntimeError):
    """The compiled Plant does not match its frozen contract identity."""


class ResetAdmissibilityError(RuntimeError):
    """A supported-reset admissibility condition R1-R10 failed."""


@dataclass(frozen=True)
class FixedHoldResetState:
    """Deterministic trusted state handed to the first participant call."""

    qpos: np.ndarray
    qvel: np.ndarray
    drive_state: np.ndarray
    previous_torque: np.ndarray
    previous_action: np.ndarray
    contact_force_N: np.ndarray
    contact_cop_xy_m: np.ndarray
    contact_cop_valid: np.ndarray
    first_observation: dict[str, Any]
    reset_metadata: dict[str, Any]
    # ``drive_state`` remains the signed aggregate compatibility view.  These
    # fields retain the qualified dual-drive state for trusted diagnostics and
    # reset-equivalence checks without exposing it to the participant.
    a_plus: np.ndarray | None = None
    a_minus: np.ndarray | None = None
    override_flags: dict[str, np.ndarray] | None = None
    previous_command: np.ndarray | None = None
    reversal_phase: np.ndarray | None = None


# --------------------------------------------------------------------------
# Model construction
# --------------------------------------------------------------------------
def model_path() -> Path:
    source_path = Path(__file__).resolve().parents[3] / "assets" / _ASSET_NAME
    if source_path.is_file():
        return source_path
    resource = resources.files("loaded_cmj.assets").joinpath(_ASSET_NAME)
    return Path(resource)


def model_xml() -> str:
    """Read the packaged authoritative MJCF without a filesystem assumption."""
    return resources.files("loaded_cmj.assets").joinpath(_ASSET_NAME).read_text()


def build_spec() -> Any:
    """Return the parsed ``MjSpec`` for the authoritative MJCF."""
    return mujoco.MjSpec.from_string(model_xml())


def build_model() -> mujoco.MjModel:
    """Compile the single authoritative MJCF.

    The same model path is used by experiments and deterministic replay tools.
    """
    return mujoco.MjModel.from_xml_string(model_xml())


def observation_spec() -> tuple[str, ...]:
    """The sixteen public observation field names, in publication order."""
    return OBSERVATION_FIELDS


@dataclass(frozen=True)
class PlantIndices:
    """Name-resolved indices, resolved once and never recomputed."""

    body: dict[str, int]
    joint: dict[str, int]
    actuator: dict[str, int]
    geom: dict[str, int]
    qadr: dict[str, int]
    vadr: dict[str, int]
    athlete_bodies: tuple[int, ...]
    load_body: int
    all_bodies: tuple[int, ...]
    floor_geom: int
    left_pads: tuple[int, ...]
    right_pads: tuple[int, ...]
    shell_geoms: tuple[int, ...]
    pelvis_body: int
    torso_body: int
    left_foot_body: int
    right_foot_body: int


def resolve_indices(model: mujoco.MjModel) -> PlantIndices:
    """Resolve every name the trusted path needs, or raise immediately."""

    def _id(objtype: Any, name: str) -> int:
        idx = mujoco.mj_name2id(model, objtype, name)
        if idx < 0:
            raise PlantIntegrityError(f"model has no object named {name!r}")
        return int(idx)

    body = {n: _id(mujoco.mjtObj.mjOBJ_BODY, n) for n in BODY_NAMES}
    joint = {n: _id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES}
    actuator = {n: _id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in MJ_ACTUATOR_NAMES}
    geom_names = (FLOOR_GEOM_NAME,) + ADMISSIBLE_CONTACT_GEOMS + SHELL_GEOM_NAMES
    geom = {n: _id(mujoco.mjtObj.mjOBJ_GEOM, n) for n in geom_names}

    return PlantIndices(
        body=body,
        joint=joint,
        actuator=actuator,
        geom=geom,
        qadr={n: int(model.jnt_qposadr[joint[n]]) for n in JOINT_NAMES},
        vadr={n: int(model.jnt_dofadr[joint[n]]) for n in JOINT_NAMES},
        athlete_bodies=tuple(body[n] for n in ATHLETE_BODY_NAMES),
        load_body=body[LOAD_BODY_NAME],
        all_bodies=tuple(body[n] for n in BODY_NAMES),
        floor_geom=geom[FLOOR_GEOM_NAME],
        left_pads=tuple(geom[n] for n in LEFT_PAD_GEOM_NAMES),
        right_pads=tuple(geom[n] for n in RIGHT_PAD_GEOM_NAMES),
        shell_geoms=tuple(geom[n] for n in SHELL_GEOM_NAMES),
        pelvis_body=body["pelvis"],
        torso_body=body["torso_head_arms"],
        left_foot_body=body["left_foot"],
        right_foot_body=body["right_foot"],
    )


# --------------------------------------------------------------------------
# The Plant
# --------------------------------------------------------------------------
class Plant:
    """Trusted wrapper over the compiled model.

    Owns name resolution, anatomical coordinates, the anatomical-to-generalized
    force map, the deterministic supported reset, and the public observation
    builder. It does not own the rollout loop or the event detector.
    """

    def __init__(self, model: mujoco.MjModel | None = None) -> None:
        self.model = build_model() if model is None else model
        self.idx = resolve_indices(self.model)
        self._assert_compiled_identity()
        self._force_range = np.array(
            [self.model.actuator_forcerange[self.idx.actuator[n]] for n in MJ_ACTUATOR_NAMES],
            dtype=np.float64,
        )
        self._ball_channel0 = {j: BALL_JOINT_CHANNELS[j][0] for j in BALL_JOINT_NAMES}
        self._ball_actuator_ids = {
            j: tuple(self.idx.actuator[a] for a in BALL_JOINT_MJ_ACTUATORS[j])
            for j in BALL_JOINT_NAMES
        }
        self._hinge_actuator_ids = {
            ch: self.idx.actuator[name] for ch, name in HINGE_CHANNEL_MJ_ACTUATOR.items()
        }
        self._pad_ids = list(self.idx.left_pads) + list(self.idx.right_pads)
        self._shell_ids = frozenset(self.idx.shell_geoms)
        self._left_pad_set = frozenset(self.idx.left_pads)
        self._right_pad_set = frozenset(self.idx.right_pads)

        # Freeze the neutral relative rotations from the actual compiled reset
        # geometry, not from an assumed identity chart.  The reference is a
        # Plant identity datum and is never mutated during a rollout.
        neutral_data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, neutral_data)
        neutral_data.qpos[:] = np.asarray(RESET_QPOS, dtype=np.float64)
        neutral_data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, neutral_data)
        self._neutral_reference_quat = {
            joint: self._relative_rotation_quaternion(neutral_data, joint)
            for joint in BALL_JOINT_NAMES
        }
        neutral_s = self.anatomical_coordinates(neutral_data)
        neutral_sd = np.zeros(ACTION_DIM, dtype=np.float64)
        tau_min, tau_max = drive_model.capacity_envelope(neutral_s, neutral_sd)
        tau_capacity = np.where(_EQUILIBRIUM_TAU >= 0.0, tau_max, -tau_min)
        if np.any(tau_capacity <= 0.0) or not np.isfinite(tau_capacity).all():
            raise PlantIntegrityError("equilibrium capacity is invalid")
        self._tau_eq = _EQUILIBRIUM_TAU.copy()
        self._u_eq = self._tau_eq / tau_capacity
        if not np.isfinite(self._u_eq).all() or np.any(np.abs(self._u_eq) > 1.0):
            raise PlantIntegrityError("equilibrium command is outside [-1, 1]")

    @property
    def action_channel_order(self) -> tuple[str, ...]:
        """Final authority channel names in serialized 15-channel order."""
        return _AUTHORITY_ACTION_CHANNELS

    @property
    def hinge_channel_signs(self) -> dict[str, float]:
        """Signed scalar hinge maps, returned as a defensive copy."""
        return dict(_HINGE_CHANNEL_SIGNS)

    @property
    def u_eq(self) -> np.ndarray:
        """Canonical normalized equilibrium command for the fixed hold."""
        return self._u_eq.copy()

    @property
    def tau_eq(self) -> np.ndarray:
        """Realized anatomical equilibrium torque used to seed the reset."""
        return self._tau_eq.copy()

    def _relative_rotation_matrix(
        self, data: mujoco.MjData, joint: str
    ) -> np.ndarray:
        jid = self.idx.joint[joint]
        child = int(self.model.jnt_bodyid[jid])
        parent = int(self.model.body_parentid[child])
        if parent <= 0:
            raise PlantIntegrityError(f"ball joint {joint!r} has no body parent")
        r_gp = np.asarray(data.xmat[parent], dtype=np.float64).reshape(3, 3)
        r_gc = np.asarray(data.xmat[child], dtype=np.float64).reshape(3, 3)
        return r_gp.T @ r_gc

    def _relative_rotation_quaternion(
        self, data: mujoco.MjData, joint: str
    ) -> np.ndarray:
        return so3_map.matrix_to_quaternion(self._relative_rotation_matrix(data, joint))

    def relative_rotation_quaternion(
        self, data: mujoco.MjData, joint: str
    ) -> np.ndarray:
        """Current ``q_delta`` for ``R_ref.T @ R_PC`` at a named ball joint."""
        if joint not in BALL_JOINT_NAMES:
            raise PlantIntegrityError(f"{joint!r} is not a ball joint")
        return self._relative_rotation_quaternion(data, joint)

    def neutral_reference_quaternion(self, joint: str) -> np.ndarray:
        """Frozen ``q_ref`` corresponding to the supported reset pose."""
        try:
            return self._neutral_reference_quat[joint].copy()
        except KeyError as exc:
            raise PlantIntegrityError(f"{joint!r} is not a ball joint") from exc

    def load_carriage_transform(self, data: mujoco.MjData) -> dict[str, np.ndarray]:
        """Return the trunk-to-load rigid transform in the trunk body frame."""
        parent = self.idx.torso_body
        child = self.idx.load_body
        r_gp = np.asarray(data.xmat[parent], dtype=np.float64).reshape(3, 3)
        r_gc = np.asarray(data.xmat[child], dtype=np.float64).reshape(3, 3)
        p_gp = np.asarray(data.xpos[parent], dtype=np.float64)
        p_gc = np.asarray(data.xpos[child], dtype=np.float64)
        return {
            "rotation": r_gp.T @ r_gc,
            "translation": r_gp.T @ (p_gc - p_gp),
        }

    # -- startup assertion (STATE_AND_COORDINATE_CONTRACT.md section 6 rule 5)
    def _assert_compiled_identity(self) -> None:
        m = self.model
        expected = {
            "nq": COMPILED_NQ, "nv": COMPILED_NV, "nu": COMPILED_NU,
            "nbody": COMPILED_NBODY, "njnt": COMPILED_NJNT,
            "ngeom": COMPILED_NGEOM, "neq": COMPILED_NEQ, "na": COMPILED_NA,
        }
        for name, want in expected.items():
            got = int(getattr(m, name))
            if got != want:
                raise PlantIntegrityError(f"compiled {name} = {got}, contract requires {want}")
        for name, q_want, v_want in _EXPECTED_JOINT_OFFSETS:
            if self.idx.qadr[name] != q_want or self.idx.vadr[name] != v_want:
                raise PlantIntegrityError(
                    f"joint {name!r} resolved to qpos {self.idx.qadr[name]} / "
                    f"qvel {self.idx.vadr[name]}, contract requires {q_want} / {v_want}"
                )
        for name in JOINT_NAMES:
            jt = int(m.jnt_type[self.idx.joint[name]])
            if name == ROOT_JOINT_NAME:
                want_type = int(mujoco.mjtJoint.mjJNT_FREE)
            elif name in BALL_JOINT_NAMES:
                want_type = int(mujoco.mjtJoint.mjJNT_BALL)
            else:
                want_type = int(mujoco.mjtJoint.mjJNT_HINGE)
            if jt != want_type:
                raise PlantIntegrityError(f"joint {name!r} has the wrong type")
        if abs(float(m.body_mass.sum()) - TOTAL_MASS_KG) > 1e-9:
            raise PlantIntegrityError("total body mass does not close to 95.0 kg")
        if float(np.abs(m.body_gravcomp).max()) != 0.0:
            raise PlantIntegrityError("body_gravcomp must be identically zero")
        # Only `pelvis` may be a child of `world`, and only through `root`.
        for bid in range(1, m.nbody):
            if int(m.body_parentid[bid]) == 0 and bid != self.idx.pelvis_body:
                raise PlantIntegrityError("a body other than pelvis is a child of world")
        # No actuator may transmit to the root free joint.
        root_jid = self.idx.joint[ROOT_JOINT_NAME]
        for a in range(m.nu):
            if (int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                    and int(m.actuator_trnid[a, 0]) == root_jid):
                raise PlantIntegrityError("an actuator transmits to the root free joint")
        # Root passive mechanics are exactly zero (PASSIVE_MECHANICS section 3.1).
        if float(m.jnt_stiffness[root_jid]) != 0.0:
            raise PlantIntegrityError("root joint stiffness must be exactly 0.0")
        if float(np.abs(m.dof_damping[0:6]).max()) != 0.0:
            raise PlantIntegrityError("root dof damping must be exactly 0.0")
        if float(np.abs(m.dof_armature[0:6]).max()) != 0.0:
            raise PlantIntegrityError("root dof armature must be exactly 0.0")
        # The external load carries no joint of its own.
        if int(m.body_jntnum[self.idx.load_body]) != 0:
            raise PlantIntegrityError("external_load must be a rigid jointless child")

    # ---------------------------------------------------------------- state
    def make_data(self) -> mujoco.MjData:
        return mujoco.MjData(self.model)

    def anatomical_coordinates(self, data: mujoco.MjData) -> np.ndarray:
        """The fifteen anatomical coordinates ``s``, in canonical channel order."""
        s = np.empty(ACTION_DIM, dtype=np.float64)
        for joint in BALL_JOINT_NAMES:
            c0 = self._ball_channel0[joint]
            current = self._relative_rotation_quaternion(data, joint)
            s[c0:c0 + 3] = so3_map.anatomical_coordinate(
                joint,
                current,
                reference_quat=self._neutral_reference_quat[joint],
            )
        for ch, joint in _HINGE_CHANNEL_JOINT:
            authority_name = _AUTHORITY_ACTION_CHANNELS[ch]
            sign = _HINGE_CHANNEL_SIGNS[authority_name]
            s[ch] = sign * float(data.qpos[self.idx.qadr[joint]])
        return s

    def anatomical_rates(self, data: mujoco.MjData) -> np.ndarray:
        """The fifteen anatomical rates ``s_dot``, in canonical channel order."""
        sd = np.empty(ACTION_DIM, dtype=np.float64)
        for joint in BALL_JOINT_NAMES:
            vadr = self.idx.vadr[joint]
            c0 = self._ball_channel0[joint]
            current = self._relative_rotation_quaternion(data, joint)
            sd[c0:c0 + 3] = so3_map.anatomical_rate(
                joint,
                current,
                data.qvel[vadr:vadr + 3],
                reference_quat=self._neutral_reference_quat[joint],
            )
        for ch, joint in _HINGE_CHANNEL_JOINT:
            authority_name = _AUTHORITY_ACTION_CHANNELS[ch]
            sign = _HINGE_CHANNEL_SIGNS[authority_name]
            sd[ch] = sign * float(data.qvel[self.idx.vadr[joint]])
        return sd

    # --------------------------------------------------------------- passive
    def region_radius_vector(self) -> np.ndarray:
        """Return the immutable anatomical-region radii ``r_i``."""
        return _REGION_RADIUS.copy()

    def region_phi(self, s: np.ndarray) -> float:
        """Evaluate ``Phi = sum_i |(s_i-s0_i)/r_i|^4``."""
        value = np.asarray(s, dtype=np.float64).reshape(ACTION_DIM)
        if not np.isfinite(value).all():
            raise PlantIntegrityError("region coordinate is not finite")
        normalized = (value - _REGION_CENTER) / _REGION_RADIUS
        phi = float(np.sum(np.abs(normalized) ** _REGION_EXPONENT))
        if not np.isfinite(phi):
            raise PlantIntegrityError("region Phi evaluated non-finite")
        return phi

    def passive_potential(self, s: np.ndarray) -> float:
        """Anatomical passive-limit potential ``V_lim`` in joules."""
        phi = self.region_phi(s)
        value = _REGION_LIMIT_K * max(phi - 1.0, 0.0) ** 4
        if not np.isfinite(value):
            raise PlantIntegrityError("passive potential evaluated non-finite")
        return float(value)

    def passive_potential_gradient(self, s: np.ndarray) -> np.ndarray:
        """Exact gradient of ``passive_potential`` in anatomical coordinates."""
        value = np.asarray(s, dtype=np.float64).reshape(ACTION_DIM)
        if not np.isfinite(value).all():
            raise PlantIntegrityError("passive-gradient coordinate is not finite")
        normalized = (value - _REGION_CENTER) / _REGION_RADIUS
        phi = float(np.sum(np.abs(normalized) ** _REGION_EXPONENT))
        if phi <= 1.0:
            return np.zeros(ACTION_DIM, dtype=np.float64)
        grad_phi = _REGION_EXPONENT * (value - _REGION_CENTER) ** 3 / (_REGION_RADIUS ** 4)
        gradient = _REGION_EXPONENT * _REGION_LIMIT_K * (phi - 1.0) ** (_REGION_EXPONENT - 1) * grad_phi
        if not np.isfinite(gradient).all():
            raise PlantIntegrityError("passive potential gradient evaluated non-finite")
        return gradient

    def outward_capacity_taper(self, s: np.ndarray, channel: int, sigma: float = 1.0) -> float:
        """Smooth outward capacity multiplier for one anatomical channel."""
        index = int(channel)
        if not 0 <= index < ACTION_DIM:
            raise PlantIntegrityError("outward taper channel is outside action dimension")
        value = np.asarray(s, dtype=np.float64).reshape(ACTION_DIM)
        phi = self.region_phi(value)
        if phi <= 1.0 - _REGION_TAPER_DELTA:
            return 1.0
        # The directional derivative is the exact first-order region test in
        # the scalar anatomical basis; negative motion is inward and retains
        # ordinary capacity.
        gradient = self.passive_potential_gradient(value) / max(
            _REGION_LIMIT_K * _REGION_EXPONENT * max(phi - 1.0, 0.0) ** 3, 1e-300
        ) if phi > 1.0 else (
            _REGION_EXPONENT * (value - _REGION_CENTER) ** 3 / (_REGION_RADIUS ** 4)
        )
        outward = float(sigma) * float(gradient[index])
        if outward <= 0.0:
            return 1.0
        if phi >= 1.0:
            return 0.0
        r = float(np.clip((phi - (1.0 - _REGION_TAPER_DELTA)) / _REGION_TAPER_DELTA, 0.0, 1.0))
        smooth = 6.0 * r**5 - 15.0 * r**4 + 10.0 * r**3
        return float(np.clip(1.0 - smooth, 0.0, 1.0))

    def passive_elastic_torque(self, s: np.ndarray) -> np.ndarray:
        """Signed anatomical elastic torque ``-gradient(V_lim)``."""
        return -self.passive_potential_gradient(s)

    def passive_damping_torque(self, s_dot: np.ndarray) -> np.ndarray:
        """Positive-semidefinite anatomical viscous damping torque."""
        rate = np.asarray(s_dot, dtype=np.float64).reshape(ACTION_DIM)
        if not np.isfinite(rate).all():
            raise PlantIntegrityError("passive damping rate is not finite")
        tau = -_DAMPING_COEFFICIENT * rate
        if not np.isfinite(tau).all():
            raise PlantIntegrityError("passive damping evaluated non-finite")
        return tau

    def passive_force_components(self, data: mujoco.MjData) -> dict[str, Any]:
        """Return source-separated passive generalized-force diagnostics."""
        elastic = self.passive_elastic_torque(self.anatomical_coordinates(data))
        damping = self.passive_damping_torque(self.anatomical_rates(data))
        def map_torque(torque: np.ndarray) -> np.ndarray:
            mapped_qfrc = np.zeros(self.model.nv, dtype=np.float64)
            for joint in BALL_JOINT_NAMES:
                c0 = self._ball_channel0[joint]
                current = self._relative_rotation_quaternion(data, joint)
                aids = self._ball_actuator_ids[joint]
                lo = float(self._force_range[aids[0], 0])
                hi = float(self._force_range[aids[0], 1])
                mapped = so3_map.dual_torque(
                    joint, current, torque[c0:c0 + 3],
                    reference_quat=self._neutral_reference_quat[joint],
                    force_range=(lo, hi),
                )
                mapped_qfrc[np.asarray([self.idx.vadr[joint] + k for k in range(3)], dtype=int)] = mapped
            for ch, joint in _HINGE_CHANNEL_JOINT:
                mapped_qfrc[self.idx.vadr[joint]] = _HINGE_CHANNEL_SIGNS[_AUTHORITY_ACTION_CHANNELS[ch]] * torque[ch]
            return mapped_qfrc

        qfrc_elastic = map_torque(elastic)
        qfrc_damping = map_torque(damping)
        qfrc = qfrc_elastic + qfrc_damping
        return {
            "qfrc_elastic": qfrc_elastic,
            "qfrc_damping": qfrc_damping,
            "qfrc_total": qfrc.copy(),
            "root_direct_force": False,
            "load_force": np.zeros(6, dtype=np.float64),
            "elastic_torque": elastic.copy(),
            "damping_torque": damping.copy(),
        }

    def passive_work_ledgers(
        self, s0: np.ndarray, s1: np.ndarray, sd0: np.ndarray, sd1: np.ndarray, dt: float
    ) -> dict[str, float]:
        """Source-separated passive work over one diagnostic step."""
        q0 = self.passive_potential(s0)
        q1 = self.passive_potential(s1)
        rate_mid = 0.5 * (np.asarray(sd0, dtype=np.float64) + np.asarray(sd1, dtype=np.float64))
        damping_mid = self.passive_damping_torque(rate_mid)
        damping_work = float(damping_mid @ rate_mid * float(dt))
        return {
            # The retained plant has no separate anatomical tendon/spring
            # potential.  Its conservative anatomical potential is the
            # explicit limit potential below; do not count it twice.
            "elastic_work": 0.0,
            "damping_work": damping_work,
            "active_work": 0.0,
            "limit_work": float(-(q1 - q0)),
            "potential_delta": float(q1 - q0),
        }

    def realized_power_components(
        self, data: mujoco.MjData, realized_anatomical_torque: np.ndarray
    ) -> dict[str, float]:
        """Return post-step powers from realized torque and matching rates.

        The caller supplies the torque returned by the qualified drive step;
        this method never reconstructs it from the requested action.  All
        values are instantaneous powers in watts.  The existing passive work
        ledger remains a separate J-valued diagnostic and is not consumed here.
        """
        tau = np.asarray(realized_anatomical_torque, dtype=np.float64).reshape(ACTION_DIM)
        rate = self.anatomical_rates(data)
        if not np.isfinite(tau).all() or not np.isfinite(rate).all():
            raise PlantIntegrityError("realized power input is not finite")
        passive = self.passive_force_components(data)
        active_power = float(tau @ rate)
        damping_power = float(np.asarray(passive["damping_torque"]) @ rate)
        limit_power = float(np.asarray(passive["elastic_torque"]) @ rate)
        values = {
            "active_power_signed_W": active_power,
            "active_power_positive_W": max(active_power, 0.0),
            "active_power_negative_W": min(active_power, 0.0),
            "passive_power_W": damping_power + limit_power,
            "damping_power_W": damping_power,
            "limit_power_W": limit_power,
        }
        if not all(np.isfinite(value) for value in values.values()):
            raise PlantIntegrityError("realized power evaluated non-finite")
        return values

    def total_system_energy(self, data: mujoco.MjData) -> float:
        """Whole-system mechanical energy plus the anatomical limit potential."""
        energy = 0.0
        velocity = np.zeros(6, dtype=np.float64)
        for body_id in self.idx.all_bodies:
            mujoco.mj_objectVelocity(
                self.model, data, int(mujoco.mjtObj.mjOBJ_BODY), int(body_id), velocity, 0
            )
            mass = float(self.model.body_mass[body_id])
            omega = velocity[:3]
            linear = velocity[3:]
            rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
            inertia = rot @ np.diag(np.asarray(self.model.body_inertia[body_id], dtype=float)) @ rot.T
            energy += 0.5 * mass * float(linear @ linear)
            energy += 0.5 * float(omega @ inertia @ omega)
            energy += mass * GRAVITY_MAGNITUDE * float(data.xipos[body_id, 2])
        energy += self.passive_potential(self.anatomical_coordinates(data))
        if not np.isfinite(energy):
            raise PlantIntegrityError("total system energy evaluated non-finite")
        return float(energy)

    def energy_work_residual(self, data: mujoco.MjData, energy_delta: float, declared_work: float = 0.0) -> float:
        """Return the discrete source-accounting residual for a supplied step."""
        residual = float(energy_delta) - float(declared_work)
        if not np.isfinite(residual):
            raise PlantIntegrityError("energy/work residual evaluated non-finite")
        return residual

    # ---------------------------------------------------------------- force
    def apply_anatomical_torque(self, data: mujoco.MjData, tau_eta: np.ndarray) -> np.ndarray:
        """Write ``tau_eta`` into ``data.ctrl``; return the generalized torques.

        Hinge channels are the identity.  Ball channels go through the
        state-dependent dual map ``T_j(q)^T``.  Root rows receive nothing: no
        actuator exists there, so the contribution is structurally zero.
        """
        tau = np.asarray(tau_eta, dtype=np.float64).reshape(ACTION_DIM)
        if not np.isfinite(tau).all():
            raise PlantIntegrityError("anatomical torque is not finite")
        ctrl = np.zeros(self.model.nu, dtype=np.float64)
        for joint in BALL_JOINT_NAMES:
            c0 = self._ball_channel0[joint]
            aids = self._ball_actuator_ids[joint]
            lo = float(self._force_range[aids[0], 0])
            hi = float(self._force_range[aids[0], 1])
            current = self._relative_rotation_quaternion(data, joint)
            tau_mj = so3_map.dual_torque(
                joint,
                current,
                tau[c0:c0 + 3],
                reference_quat=self._neutral_reference_quat[joint],
                force_range=(lo, hi),
            )
            for k, aid in enumerate(aids):
                ctrl[aid] = tau_mj[k]
        for ch, aid in self._hinge_actuator_ids.items():
            authority_name = _AUTHORITY_ACTION_CHANNELS[ch]
            value = _HINGE_CHANNEL_SIGNS[authority_name] * float(tau[ch])
            lo = float(self._force_range[aid, 0])
            hi = float(self._force_range[aid, 1])
            if value < lo or value > hi:
                raise PlantIntegrityError(
                    f"channel {ACTION_CHANNELS[ch]} torque {value} leaves forcerange"
                )
            ctrl[aid] = value
        data.ctrl[:] = ctrl
        return ctrl

    # ---------------------------------------------------------------- reset
    def reset_supported(self, data: mujoco.MjData) -> None:
        """Steps 1-3 and 10 of RESET_CONTRACT.md section 1.

        These writes to ``qpos``/``qvel`` are the ONLY direct writes to physical
        state in the entire system.  No root servo, no world support, no
        post-step correction, no ``mj_inverse``.
        """
        mujoco.mj_resetData(self.model, data)
        data.qpos[:] = np.asarray(RESET_QPOS, dtype=np.float64)
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, data)

    def reset(self, *, seed: int = 0, metadata: Any = None) -> FixedHoldResetState:
        """Return the canonical deterministic equilibrium reset state.

        The accepted reset identity is ``seed=0`` and no metadata. The
        method returns the exact supported equilibrium tuple.  The separate
        ``reset_fixed_hold`` diagnostic may be run afterward for the bounded
        neutral dwell, but its settling trace is not substituted for the exact
        reset velocity.
        """
        if int(seed) != 0:
            raise ResetAdmissibilityError("reset accepts only seed=0")
        if metadata is not None:
            raise ResetAdmissibilityError("reset accepts metadata=None only")
        data = self.make_data()
        self.reset_supported(data)
        self.assert_reset_admissible(data)
        fz, cop, cop_valid = self.foot_contact_summary(data)
        return FixedHoldResetState(
            qpos=np.asarray(data.qpos, dtype=np.float64).copy(),
            qvel=np.asarray(data.qvel, dtype=np.float64).copy(),
            drive_state=self.u_eq,
            previous_torque=self.tau_eq,
            previous_action=self.u_eq,
            contact_force_N=fz.copy(),
            contact_cop_xy_m=cop.copy(),
            contact_cop_valid=cop_valid.copy(),
            first_observation=self.public_observation(
                data,
                scored_time_s=0.0,
                step_index=0,
                episode_reset=True,
                previous_action=self.u_eq,
            ),
            reset_metadata={
                "protocol": "equilibrium_reset",
                "scored_time_s": 0.0,
                "episode_reset": True,
            },
            a_plus=np.maximum(self.u_eq, 0.0),
            a_minus=np.maximum(-self.u_eq, 0.0),
            override_flags={},
            previous_command=self.u_eq.copy(),
            reversal_phase=np.zeros(ACTION_DIM, dtype=np.int8),
        )

    def _external_wrench_generalized(self, data: mujoco.MjData) -> np.ndarray:
        """Map trusted world-frame COM wrenches to generalized coordinates."""
        q_xfrc = np.zeros(self.model.nv, dtype=np.float64)
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        for body_id in range(1, self.model.nbody):
            wrench = np.asarray(data.xfrc_applied[body_id], dtype=np.float64)
            if not np.any(wrench):
                continue
            jacp.fill(0.0)
            jacr.fill(0.0)
            mujoco.mj_jacBody(self.model, data, jacp, jacr, body_id)
            q_xfrc += jacp.T @ wrench[0:3] + jacr.T @ wrench[3:6]
        return q_xfrc

    def dynamics_residual(self, data: mujoco.MjData) -> dict[str, Any]:
        """Evaluate the final continuous generalized force balance.

        ``qfrc_constraint`` is retained as one classified right-hand-side
        source; contact/plate decompositions are downstream diagnostics and
        are never added again here.
        """
        if not all(np.isfinite(x).all() for x in (
            data.qpos, data.qvel, data.qacc, data.qfrc_bias,
            data.qfrc_passive, data.qfrc_actuator, data.qfrc_applied,
            data.qfrc_constraint, data.xfrc_applied,
        )):
            raise PlantIntegrityError("dynamics residual inputs are not finite")
        mass_acc = np.zeros(self.model.nv, dtype=np.float64)
        mujoco.mj_mulM(self.model, data, mass_acc, data.qacc)
        q_xfrc = self._external_wrench_generalized(data)
        residual = (
            mass_acc + np.asarray(data.qfrc_bias)
            - np.asarray(data.qfrc_passive)
            - np.asarray(data.qfrc_actuator)
            - np.asarray(data.qfrc_applied)
            - q_xfrc
            - np.asarray(data.qfrc_constraint)
        )
        return {
            "mass_acc": mass_acc.copy(),
            "qfrc_bias": np.asarray(data.qfrc_bias, dtype=np.float64).copy(),
            "qfrc_passive": np.asarray(data.qfrc_passive, dtype=np.float64).copy(),
            "qfrc_actuator": np.asarray(data.qfrc_actuator, dtype=np.float64).copy(),
            "qfrc_applied": np.asarray(data.qfrc_applied, dtype=np.float64).copy(),
            "q_xfrc": q_xfrc,
            "qfrc_constraint": np.asarray(data.qfrc_constraint, dtype=np.float64).copy(),
            "residual": residual.copy(),
            "mechanics_residual_trans_N": float(np.abs(residual[:3]).max()),
            "mechanics_residual_rot_Nm": float(np.abs(residual[3:]).max()),
        }

    def reset_fixed_hold(self, data: mujoco.MjData) -> FixedHoldResetState:
        """Execute the disclosed open-loop fixed-hold reset protocol.

        The hold uses the normal drive, torque, SO(3), actuator, and MuJoCo
        paths.  It has no feedback target, participant call, external force,
        or state write after the initial supported reset.
        """
        self.reset_supported(data)
        self.assert_reset_admissible(data)
        action = np.asarray(HOLD_ACTION, dtype=np.float64).copy()
        state = drive_model.DriveState(
            a_plus=np.maximum(action, 0.0),
            a_minus=np.maximum(-action, 0.0),
            tau_prev=self.tau_eq,
            previous_command=action.copy(),
        )
        tau_prev = state.tau_prev.copy()
        com0 = self.center_of_mass(data).copy()
        qvel_hist: list[float] = []
        qacc_hist: list[float] = []
        grf_hist: list[float] = []
        max_penetration = 0.0
        max_applied = 0.0
        all_finite = True
        no_limit_contact = True
        latch = np.zeros(2, dtype=bool)
        on_steps = np.zeros(2, dtype=np.int64)
        off_steps = np.zeros(2, dtype=np.int64)
        on_required = round(CONTACT_T_ON_S / PHYSICS_TIMESTEP_S)
        off_required = round(CONTACT_T_OFF_S / PHYSICS_TIMESTEP_S)
        latch_after_50ms: list[bool] = []
        limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        for step in range(FIXED_HOLD_SUBSTEPS):
            result = drive_model.drive_state_step(
                action,
                state,
                self.anatomical_coordinates(data),
                self.anatomical_rates(data),
                PHYSICS_TIMESTEP_S,
            )
            tau_prev = np.asarray(result["tau"], dtype=np.float64).copy()
            self.apply_anatomical_torque(data, tau_prev)
            mujoco.mj_step(self.model, data)
            qvel_hist.append(float(np.abs(data.qvel).max()))
            qacc_hist.append(float(np.abs(data.qacc).max()))
            fz_step = self.foot_contact_summary(data)[0]
            grf_hist.append(float(fz_step.sum()))
            max_penetration = max(max_penetration, self.max_pad_penetration(data))
            max_applied = max(max_applied, float(np.abs(data.qfrc_applied).max()),
                              float(np.abs(data.xfrc_applied).max()))
            gaps = self.pad_gaps(data).reshape(2, 4).min(axis=1)
            for foot in range(2):
                if not latch[foot]:
                    on_steps[foot] = (on_steps[foot] + 1
                                      if fz_step[foot] >= CONTACT_F_ON_N
                                      and gaps[foot] <= CONTACT_GAP_MAX_M else 0)
                    if on_steps[foot] >= on_required:
                        latch[foot] = True
                        off_steps[foot] = 0
                else:
                    off_steps[foot] = (off_steps[foot] + 1
                                       if fz_step[foot] <= CONTACT_F_OFF_N else 0)
                    if off_steps[foot] >= off_required:
                        latch[foot] = False
                        on_steps[foot] = 0
            if (step + 1) * PHYSICS_TIMESTEP_S >= 0.050:
                latch_after_50ms.append(bool(latch.all()))
            if data.nefc:
                no_limit_contact = no_limit_contact and not bool(np.any(
                    np.asarray(data.efc_type[:data.nefc], dtype=np.int32) == limit_type
                ))
            all_finite = all_finite and all(np.isfinite(x).all() for x in (
                data.qpos, data.qvel, data.qacc, data.ctrl, data.qfrc_applied,
                data.xfrc_applied, state.a_plus, state.a_minus,
                state.tau_prev, state.previous_command, tau_prev, fz_step,
            ))
        com_end = self.center_of_mass(data)
        dwell_values = {
            "D1": float(np.linalg.norm(com_end[0:2] - com0[0:2])),
            "D2": float(abs(com_end[2] - com0[2])),
            "D3": float(max(qvel_hist[-100:])),
            "D4": float(max(qacc_hist[-100:])),
            "D5": bool(latch_after_50ms and all(latch_after_50ms)),
            "D6": float(np.mean(grf_hist[-100:])),
            "D7": bool(no_limit_contact),
            "D8": float(max_penetration),
            "D9": bool(all_finite),
            "D10": float(max_applied),
        }
        dwell_pass = {
            "D1": dwell_values["D1"] <= 0.005,
            "D2": dwell_values["D2"] <= 0.004,
            "D3": dwell_values["D3"] <= 0.02,
            "D4": dwell_values["D4"] <= 1.0,
            "D5": dwell_values["D5"],
            "D6": abs(dwell_values["D6"] - BODY_WEIGHT_N) / BODY_WEIGHT_N <= 0.02,
            "D7": dwell_values["D7"],
            "D8": dwell_values["D8"] <= 0.001,
            "D9": dwell_values["D9"],
            "D10": dwell_values["D10"] == 0.0,
        }
        if not all(dwell_pass.values()):
            failed = ", ".join(k for k, passed in dwell_pass.items() if not passed)
            raise ResetAdmissibilityError(f"fixed-hold dwell failed {failed}")
        fz, cop, cop_valid = self.foot_contact_summary(data)
        first_observation = self.public_observation(
            data,
            scored_time_s=0.0,
            step_index=0,
            episode_reset=True,
            previous_action=action,
        )
        return FixedHoldResetState(
            qpos=np.asarray(data.qpos, dtype=np.float64).copy(),
            qvel=np.asarray(data.qvel, dtype=np.float64).copy(),
            drive_state=state.z.copy(),
            previous_torque=tau_prev.copy(),
            previous_action=action.copy(),
            contact_force_N=fz.copy(),
            contact_cop_xy_m=cop.copy(),
            contact_cop_valid=cop_valid.copy(),
            first_observation=first_observation,
            reset_metadata={
                "protocol": "fixed_hold_reset",
                "reset_action_source": "HOLD_ACTION",
                "reset_action": action.tolist(),
                "dwell_substeps": FIXED_HOLD_SUBSTEPS,
                "physics_timestep_s": PHYSICS_TIMESTEP_S,
                "physics_steps_per_control": SUBSTEPS_PER_CONTROL,
                "reset_is_control_step_count": False,
                "dwell_duration_s": FIXED_HOLD_SUBSTEPS * PHYSICS_TIMESTEP_S,
                "policy_calls_during_dwell": 0,
                "reset_scored": False,
                "reset_samples_scored": False,
                "dwell_values": dwell_values,
                "dwell_pass": dwell_pass,
                "scored_time_s": 0.0,
                "episode_reset": True,
            },
            a_plus=state.a_plus.copy(),
            a_minus=state.a_minus.copy(),
            override_flags={
                key: value.copy() for key, value in state.override_flags.items()
            },
            previous_command=state.previous_command.copy(),
            reversal_phase=state.reversal_phase.copy(),
        )

    def assert_reset_admissible(self, data: mujoco.MjData) -> dict[str, Any]:
        """Assert R1-R10 of RESET_CONTRACT.md section 3 at the declared pose."""
        report: dict[str, Any] = {}
        gaps = self.pad_gaps(data)
        report["R1_max_abs_gap_m"] = float(np.abs(gaps).max())
        if report["R1_max_abs_gap_m"] > 0.0005:
            raise ResetAdmissibilityError(f"R1 pad gap {report['R1_max_abs_gap_m']}")

        report["R2_shell_contact"] = bool(self.shell_contact(data))
        if report["R2_shell_contact"]:
            raise ResetAdmissibilityError("R2 forbidden shell contact at reset")

        left_ok = int(np.sum(gaps[0:4] <= 0.0005))
        right_ok = int(np.sum(gaps[4:8] <= 0.0005))
        report["R3_left_pads_down"] = left_ok
        report["R3_right_pads_down"] = right_ok
        if left_ok < 3 or right_ok < 3:
            raise ResetAdmissibilityError("R3 bilateral support requires >= 3 pads per foot")

        margins = {}
        for joint in HINGE_JOINT_NAMES:
            q = float(data.qpos[self.idx.qadr[joint]])
            lo, hi = HINGE_RANGES_RAD[joint]
            margins[joint] = min(q - lo, hi - q)
        report["R4_min_hinge_margin_rad"] = float(min(margins.values()))
        if report["R4_min_hinge_margin_rad"] < 0.05:
            raise ResetAdmissibilityError("R4 hinge limit margin below 0.05 rad")

        s = self.anatomical_coordinates(data)
        ball_margin = float("inf")
        for joint in BALL_JOINT_NAMES:
            c0 = self._ball_channel0[joint]
            for axis, (lo, hi) in enumerate(BALL_ADMISSIBLE_BOX_RAD[joint]):
                v = float(s[c0 + axis])
                ball_margin = min(ball_margin, v - lo, hi - v)
        report["R5_min_ball_margin_rad"] = ball_margin
        if ball_margin < 0.05:
            raise ResetAdmissibilityError("R5 ball admissible-region margin below 0.05 rad")

        com = self.center_of_mass(data)
        report["R6_com_margin_m"] = float(self.support_margin(data, com, (True, True)))
        if report["R6_com_margin_m"] < 0.020:
            raise ResetAdmissibilityError("R6 COM support margin below 0.020 m")

        report["R7_qvel_inf"] = float(np.abs(data.qvel).max())
        if report["R7_qvel_inf"] != 0.0:
            raise ResetAdmissibilityError("R7 reset velocity must be exactly zero")

        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise ResetAdmissibilityError("R8 non-finite reset state")
        root_q = self.idx.qadr[ROOT_JOINT_NAME]
        qroot = data.qpos[root_q + 3:root_q + 7]
        report["R8_root_quat_norm_err"] = float(abs(np.linalg.norm(qroot) - 1.0))
        if report["R8_root_quat_norm_err"] > 1e-9:
            raise ResetAdmissibilityError("R8 root quaternion is not unit")

        report["R9_neq"] = int(self.model.neq)
        if self.model.neq != 0:
            raise ResetAdmissibilityError("R9 world support / equality constraint present")

        conds = {}
        for joint in BALL_JOINT_NAMES:
            current = self._relative_rotation_quaternion(data, joint)
            tmat = so3_map.tangent_map(
                joint,
                current,
                reference_quat=self._neutral_reference_quat[joint],
                check=False,
            )
            conds[joint] = float(so3_map.condition_number(tmat))
        report["R10_cond"] = conds
        if max(conds.values()) > so3_map.CONDITION_GUARD:
            raise ResetAdmissibilityError("R10 ball tangent map is ill-conditioned at reset")
        return report

    # ------------------------------------------------------------------ COM
    def center_of_mass(self, data: mujoco.MjData) -> np.ndarray:
        """Whole-system COM, mass-weighted over the nine non-world bodies."""
        ids = list(self.idx.all_bodies)
        mass = self.model.body_mass[ids]
        return np.asarray(mass @ data.xipos[ids] / mass.sum(), dtype=np.float64)

    def center_of_mass_velocity(self, data: mujoco.MjData) -> np.ndarray:
        """Whole-system COM velocity, mass-weighted body linear velocities (world)."""
        acc = np.zeros(3, dtype=np.float64)
        total = 0.0
        vel = np.zeros(6, dtype=np.float64)
        for bid in self.idx.all_bodies:
            mujoco.mj_objectVelocity(
                self.model, data, int(mujoco.mjtObj.mjOBJ_BODY), int(bid), vel, 0
            )
            m = float(self.model.body_mass[bid])
            acc += m * vel[3:6]
            total += m
        return acc / total

    def center_of_mass_acceleration(
        self,
        data: mujoco.MjData,
        *,
        support_wrench: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return whole-system COM acceleration in world SI coordinates.

        The production measurement uses the complete environment-on-system
        contact force reconstructed by :meth:`contact_wrench_summary`.  The
        only other external force in the frozen V1 Plant is uniform gravity,
        which is added explicitly here.  ``support_wrench`` is expressed at
        world origin as ``[Fx,Fy,Fz,Mx,My,Mz]`` and includes the off-plate
        diagnostic partition; it is never replaced by the two plate rows.
        """
        if support_wrench is None:
            support_wrench = self.contact_wrench_summary(data)["whole_wrench"]
        wrench = np.asarray(support_wrench, dtype=np.float64).reshape(6)
        if not np.isfinite(wrench).all():
            raise PlantIntegrityError("support wrench is non-finite")
        total_mass = float(np.sum(self.model.body_mass[list(self.idx.all_bodies)]))
        acceleration = wrench[:3] / total_mass
        acceleration = acceleration + np.asarray(
            (0.0, 0.0, -GRAVITY_MAGNITUDE), dtype=np.float64
        )
        if not np.isfinite(acceleration).all():
            raise PlantIntegrityError("COM acceleration is non-finite")
        return acceleration

    def linear_momentum(self, data: mujoco.MjData) -> np.ndarray:
        """Return ``p = M * v_COM`` in world kg m s⁻¹ coordinates."""
        total_mass = float(np.sum(self.model.body_mass[list(self.idx.all_bodies)]))
        momentum = total_mass * self.center_of_mass_velocity(data)
        if not np.isfinite(momentum).all():
            raise PlantIntegrityError("linear momentum is non-finite")
        return np.asarray(momentum, dtype=np.float64)

    def centroidal_angular_momentum(
        self,
        data: mujoco.MjData,
        com: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return the full world-frame centroidal angular momentum vector.

        MuJoCo's body velocity query with ``flg_local=0`` returns angular and
        linear velocity in world coordinates.  ``model.body_inertia`` is
        diagonal in the body inertial frame; ``data.ximat`` rotates that
        tensor into world coordinates before it is applied to world angular
        velocity.  The sum includes every non-world body, including the
        jointless 20 kg external load and the floating-base pelvis.
        """
        if com is None:
            com = self.center_of_mass(data)
        center = np.asarray(com, dtype=np.float64).reshape(3)
        if not np.isfinite(center).all():
            raise PlantIntegrityError("COM reference is non-finite")
        h = np.zeros(3, dtype=np.float64)
        velocity = np.zeros(6, dtype=np.float64)
        for body_id in self.idx.all_bodies:
            mujoco.mj_objectVelocity(
                self.model,
                data,
                int(mujoco.mjtObj.mjOBJ_BODY),
                int(body_id),
                velocity,
                0,
            )
            rotation = np.asarray(data.ximat[body_id], dtype=np.float64).reshape(3, 3)
            inertia_body = np.diag(np.asarray(self.model.body_inertia[body_id], dtype=np.float64))
            inertia_world = rotation @ inertia_body @ rotation.T
            mass = float(self.model.body_mass[body_id])
            h += inertia_world @ velocity[:3]
            h += np.cross(np.asarray(data.xipos[body_id], dtype=np.float64) - center, mass * velocity[3:6])
        if not np.isfinite(h).all():
            raise PlantIntegrityError("centroidal angular momentum is non-finite")
        return h

    def centroidal_hdot_from_external_wrench(
        self,
        data: mujoco.MjData,
        com: np.ndarray | None = None,
        *,
        support_wrench: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return ``Hdot`` from the complete external wrench about the COM.

        The trusted wrench is reconstructed at world origin.  Its moment is
        shifted to the instantaneous COM by ``M_COM = M_origin - c x F``.
        Uniform gravity acts through the complete-system COM, so its moment
        about that reference is zero; its force is already handled by the
        linear COM acceleration path.  No contact filtering or event state is
        used here.
        """
        if com is None:
            com = self.center_of_mass(data)
        center = np.asarray(com, dtype=np.float64).reshape(3)
        if support_wrench is None:
            support_wrench = self.contact_wrench_summary(data)["whole_wrench"]
        wrench = np.asarray(support_wrench, dtype=np.float64).reshape(6)
        if not np.isfinite(center).all() or not np.isfinite(wrench).all():
            raise PlantIntegrityError("centroidal external wrench is non-finite")
        hdot = wrench[3:] - np.cross(center, wrench[:3])
        if not np.isfinite(hdot).all():
            raise PlantIntegrityError("centroidal angular momentum rate is non-finite")
        return np.asarray(hdot, dtype=np.float64)

    def trunk_tilt_rad(self, data: mujoco.MjData) -> float:
        """Angle between the torso body ``+z`` axis and world ``+z``."""
        rot = np.asarray(data.xmat[self.idx.torso_body], dtype=np.float64).reshape(3, 3)
        return float(np.arccos(float(np.clip(rot[2, 2], -1.0, 1.0))))

    # -------------------------------------------------------------- contact
    def pad_world_positions(self, data: mujoco.MjData) -> np.ndarray:
        """World positions of the eight foot pads, left four then right four."""
        return np.asarray(data.geom_xpos[self._pad_ids], dtype=np.float64)

    def pad_gaps(self, data: mujoco.MjData) -> np.ndarray:
        """Per-pad signed distance to the floor, left four then right four.

        Secondary ON gate only.  ``mj_geomDistance`` is clipped at ``cutoff``
        and is not monotone once geoms interpenetrate, so force -- never gap --
        is the primary contact signal in both directions.
        """
        out = np.empty(8, dtype=np.float64)
        for k, gid in enumerate(self._pad_ids):
            out[k] = mujoco.mj_geomDistance(
                self.model, data, self.idx.floor_geom, int(gid), GEOM_DISTANCE_CUTOFF_M, None
            )
        return out

    def shell_contact(self, data: mujoco.MjData) -> bool:
        """True when any ``*_shell`` geom is in contact with the floor."""
        floor = self.idx.floor_geom
        for c in range(data.ncon):
            con = data.contact[c]
            g1, g2 = int(con.geom1), int(con.geom2)
            if (g1 == floor and g2 in self._shell_ids) or (
                g2 == floor and g1 in self._shell_ids
            ):
                return True
        return False

    def max_pad_penetration(self, data: mujoco.MjData) -> float:
        """Deepest floor-vs-pad interpenetration in metres (non-negative)."""
        floor = self.idx.floor_geom
        pads = self._left_pad_set | self._right_pad_set
        worst = 0.0
        for c in range(data.ncon):
            con = data.contact[c]
            g1, g2 = int(con.geom1), int(con.geom2)
            if (g1 == floor and g2 in pads) or (g2 == floor and g1 in pads):
                worst = max(worst, -float(con.dist))
        return worst

    @staticmethod
    def classify_contact_point(point: np.ndarray) -> str:
        """Apply the half-open virtual left/right plate partition."""
        x, y = np.asarray(point, dtype=np.float64).reshape(3)[:2]
        inside_x = -0.30 <= float(x) < 0.30
        if inside_x and 0.0 <= float(y) < 0.30:
            return "left_plate"
        if inside_x and -0.30 < float(y) < 0.0:
            return "right_plate"
        return "off_plate"

    @staticmethod
    def _contact_row_for_region(region: str, foot: int | None) -> tuple[int, bool]:
        """Assign one physical contact after geometry, then foot compatibility."""
        designated = foot is not None and region == (
            "left_plate" if foot == 0 else "right_plate"
        )
        return (int(foot), False) if designated else (2, True)

    def _contact_point_speed(self, data: mujoco.MjData, geom_id: int) -> float:
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacGeom(self.model, data, jacp, jacr, int(geom_id))
        velocity = jacp @ np.asarray(data.qvel, dtype=np.float64)
        return float(np.linalg.norm(velocity[:2])) if np.isfinite(velocity).all() else float("inf")

    def contact_wrench_summary(self, data: mujoco.MjData) -> dict[str, Any]:
        """Live all-contact wrench, plate partition, COP and stability diagnostics."""
        floor = int(self.idx.floor_geom)
        pad_sets = (self._left_pad_set, self._right_pad_set)
        origins = np.zeros((2, 3), dtype=np.float64)
        pads = self.pad_world_positions(data)
        origins[0] = np.mean(pads[:4], axis=0)
        origins[1] = np.mean(pads[4:], axis=0)
        whole = np.zeros(6, dtype=np.float64)
        # Rows are left virtual plate, right virtual plate, and off-plate
        # diagnostic partition.  The latter is never silently discarded from
        # the whole support wrench.
        plate = np.zeros((3, 6), dtype=np.float64)
        normal = np.zeros(2, dtype=np.float64)
        tangential = np.zeros(2, dtype=np.float64)
        contacts: list[dict[str, Any]] = []
        max_penetration = 0.0
        max_slip = 0.0
        friction_ok = True
        prohibited_contact = False
        assignment_count = 0
        wrench_raw = np.zeros(6, dtype=np.float64)
        for ci in range(int(data.ncon)):
            con = data.contact[ci]
            g1, g2 = int(con.geom1), int(con.geom2)
            if g1 == floor:
                system_geom = g2
                system_index = 1
            elif g2 == floor:
                system_geom = g1
                system_index = 0
            else:
                continue
            mujoco.mj_contactForce(self.model, data, ci, wrench_raw)
            point = np.asarray(con.pos, dtype=np.float64).copy()
            world_wrench = contact_wrench_from_raw(
                np.asarray(con.frame, dtype=np.float64), wrench_raw,
                system_geom_index=system_index, contact_point=point,
                plate_origin=np.zeros(3, dtype=np.float64),
            )
            whole += world_wrench
            region = self.classify_contact_point(point)
            foot = 0 if system_geom in pad_sets[0] else 1 if system_geom in pad_sets[1] else None
            row, prohibited = self._contact_row_for_region(region, foot)
            designated = not prohibited
            prohibited_contact = prohibited_contact or prohibited
            assignment_count += 1
            contact_friction_ok = True
            if designated:
                plate_wrench = contact_wrench_from_raw(
                    np.asarray(con.frame, dtype=np.float64), wrench_raw,
                    system_geom_index=system_index, contact_point=point,
                    plate_origin=origins[foot],
                )
                plate[foot] += plate_wrench
                normal_value = max(float(plate_wrench[2]), 0.0)
                normal[foot] += normal_value
                tangential[foot] += float(np.linalg.norm(plate_wrench[:2]))
                mu = float(self.model.geom_friction[system_geom, 0])
                contact_friction_ok = float(np.linalg.norm(plate_wrench[:2])) <= mu * normal_value + 1e-8
                friction_ok = friction_ok and contact_friction_ok
            else:
                plate[2] += world_wrench
            slip = self._contact_point_speed(data, system_geom)
            penetration = max(-float(con.dist), 0.0)
            max_penetration = max(max_penetration, penetration)
            max_slip = max(max_slip, slip)
            contacts.append({
                "index": ci,
                "geom1": g1,
                "geom2": g2,
                "system_geom": system_geom,
                "system_geom_index": system_index,
                "region": region,
                "geometric_region": region,
                "designated_foot": foot if designated else None,
                "row": int(row),
                "assignment_count": 1,
                "prohibited_contact": bool(prohibited),
                "wrench": world_wrench.copy(),
                "normal_force_N": float(max(world_wrench[2], 0.0)),
                "tangential_force_N": float(np.linalg.norm(world_wrench[:2])),
                "penetration_m": penetration,
                "slip_speed_mps": slip,
                "friction_feasible": bool(contact_friction_ok),
            })
        cop = np.zeros((2, 2), dtype=np.float64)
        cop_valid = normal > CONTACT_F_COP_MIN_N
        for foot in (0, 1):
            if cop_valid[foot]:
                local_offset = np.asarray(
                    (-plate[foot, 4] / plate[foot, 2], plate[foot, 3] / plate[foot, 2]),
                    dtype=np.float64,
                )
                cop[foot] = origins[foot, :2] + local_offset
        active = normal > CONTACT_F_ACTIVE_N
        return {
            "contacts": contacts,
            "whole_wrench": whole,
            "plate_wrench": plate,
            "normal_force": normal,
            "tangential_force": tangential,
            "cop_xy": cop.reshape(4),
            "cop_world_xy": cop.copy(),
            "cop_frame": "world",
            "plate_origin_world_m": origins.copy(),
            "cop_origin_world_xy": origins[:, :2].copy(),
            "cop_valid": cop_valid,
            "contact_active": bool(np.any(active)),
            "active_by_foot": active,
            "contact_region": [entry["region"] for entry in contacts],
            "prohibited_contact": bool(prohibited_contact),
            "contact_assignment_count": int(assignment_count),
            "friction_feasible": bool(friction_ok),
            "penetration_m": float(max_penetration),
            "slip_speed_mps": float(max_slip),
        }

    def update_support_latch(
        self,
        latch: np.ndarray,
        on_steps: np.ndarray,
        off_steps: np.ndarray,
        normal_force: np.ndarray,
        dt: float,
    ) -> dict[str, Any]:
        """Update hysteretic support latches with frozen debounce counts."""
        latched = np.asarray(latch, dtype=bool).reshape(2).copy()
        on = np.asarray(on_steps, dtype=np.int64).reshape(2).copy()
        off = np.asarray(off_steps, dtype=np.int64).reshape(2).copy()
        force = np.asarray(normal_force, dtype=np.float64).reshape(2)
        n_on = max(int(CONTACT_N_ON), int(np.ceil(CONTACT_T_ON_S / float(dt) - 1e-12)))
        n_off = max(int(CONTACT_N_OFF), int(np.ceil(CONTACT_T_OFF_S / float(dt) - 1e-12)))
        transitions: list[dict[str, Any]] = []
        for foot in (0, 1):
            if not latched[foot]:
                on[foot] = on[foot] + 1 if force[foot] >= CONTACT_F_ON_N else 0
                off[foot] = 0
                if on[foot] >= n_on:
                    latched[foot] = True
                    on[foot] = n_on
                    transitions.append({"foot": foot, "event": "on"})
            else:
                off[foot] = off[foot] + 1 if force[foot] <= CONTACT_F_OFF_N else 0
                on[foot] = 0
                if off[foot] >= n_off:
                    latched[foot] = False
                    off[foot] = n_off
                    transitions.append({"foot": foot, "event": "off"})
        return {"latch": latched, "on_steps": on, "off_steps": off, "transitions": transitions}

    def foot_contact_summary(
        self, data: mujoco.MjData
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(normal_force[2], cop_xy[4], cop_valid[2])`` for [left, right].

        Vertical contact force per foot, summed over that foot's four pads via
        ``mj_contactForce``, and the world-frame centre of pressure of that
        foot's contact wrench.  Non-negative by construction.
        """
        summary = self.contact_wrench_summary(data)
        return (
            np.asarray(summary["normal_force"], dtype=np.float64).copy(),
            np.asarray(summary["cop_xy"], dtype=np.float64).copy(),
            np.asarray(summary["cop_valid"], dtype=bool).copy(),
        )

    def support_polygon(self, data: mujoco.MjData, latched: tuple[bool, bool]) -> np.ndarray:
        """Pad centres of every currently latched foot, projected to ``z = 0``."""
        pads = self.pad_world_positions(data)[:, 0:2]
        rows = []
        if latched[0]:
            rows.append(pads[0:4])
        if latched[1]:
            rows.append(pads[4:8])
        if not rows:
            return np.empty((0, 2), dtype=np.float64)
        return np.concatenate(rows, axis=0)

    def support_margin(
        self, data: mujoco.MjData, com: np.ndarray, latched: tuple[bool, bool]
    ) -> float:
        """Signed distance from the COM ground projection to the support edge."""
        pts = self.support_polygon(data, latched)
        if pts.shape[0] == 0:
            return -float("inf")
        return _point_in_hull_margin(np.asarray(com[0:2], dtype=np.float64), pts)

    # ---------------------------------------------------------- observation
    def public_observation(
        self,
        data: mujoco.MjData,
        *,
        scored_time_s: float,
        step_index: int,
        episode_reset: bool,
        previous_action: np.ndarray,
    ) -> dict[str, Any]:
        """Build the sixteen-field public observation.

        Every array is a fresh ``.copy()``; no ``MjData`` view, memoryview,
        model handle, callable, or arbitrary object is ever placed in the dict.
        """
        root_q = self.idx.qadr[ROOT_JOINT_NAME]
        root_v = self.idx.vadr[ROOT_JOINT_NAME]
        torso_vel = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            self.model, data, int(mujoco.mjtObj.mjOBJ_BODY), int(self.idx.torso_body),
            torso_vel, 1,
        )
        fz, cop, cop_valid = self.foot_contact_summary(data)
        return {
            "time_s": float(scored_time_s),
            "step_index": int(step_index),
            "control_dt_s": float(CONTROL_PERIOD_S),
            "episode_reset": bool(episode_reset),
            "joint_position_rad": self.anatomical_coordinates(data),
            "joint_velocity_radps": self.anatomical_rates(data),
            "pelvis_position_world_m": np.asarray(
                data.qpos[root_q:root_q + 3], dtype=np.float64).copy(),
            "pelvis_orientation_world_quat_wxyz": _canonical_quat(
                data.qpos[root_q + 3:root_q + 7]),
            "pelvis_linear_velocity_world_mps": np.asarray(
                data.qvel[root_v:root_v + 3], dtype=np.float64).copy(),
            "pelvis_angular_velocity_body_radps": np.asarray(
                data.qvel[root_v + 3:root_v + 6], dtype=np.float64).copy(),
            "trunk_load_orientation_world_quat_wxyz": _canonical_quat(
                data.xquat[self.idx.torso_body]),
            "trunk_load_angular_velocity_body_radps": np.asarray(
                torso_vel[0:3], dtype=np.float64).copy(),
            "plantar_normal_force_N": np.asarray(fz, dtype=np.float64).copy(),
            "plantar_cop_xy_m": np.asarray(cop, dtype=np.float64).copy(),
            "plantar_cop_valid": np.asarray(cop_valid, dtype=bool).copy(),
            "previous_action": np.asarray(
                previous_action, dtype=np.float64).reshape(ACTION_DIM).copy(),
        }


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def _canonical_quat(q: np.ndarray) -> np.ndarray:
    """Unit, scalar-first, canonicalized to the ``w >= 0`` hemisphere."""
    arr = np.asarray(q, dtype=np.float64).reshape(4).copy()
    n = float(np.linalg.norm(arr))
    if n <= 0.0 or not np.isfinite(n):
        raise PlantIntegrityError("quaternion has zero or non-finite norm")
    arr /= n
    if arr[0] < 0.0:
        arr = -arr
    return arr


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Monotone-chain convex hull of a 2-D point set, counter-clockwise."""
    pts = np.unique(np.asarray(points, dtype=np.float64).reshape(-1, 2), axis=0)
    if pts.shape[0] <= 2:
        return pts

    def _build(seq: np.ndarray) -> list[np.ndarray]:
        out: list[np.ndarray] = []
        for p in seq:
            while len(out) >= 2:
                a, b = out[-2], out[-1]
                cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
                if cross <= 0.0:
                    out.pop()
                else:
                    break
            out.append(p)
        return out

    lower = _build(pts)
    upper = _build(pts[::-1])
    return np.asarray(lower[:-1] + upper[:-1], dtype=np.float64)


def _point_in_hull_margin(point: np.ndarray, points: np.ndarray) -> float:
    """Signed distance to the hull boundary: positive inside, negative outside."""
    hull = _convex_hull(points)
    n = hull.shape[0]
    if n < 3:
        return -float(np.linalg.norm(point - np.asarray(points).mean(axis=0)))
    inside = True
    best_inside = float("inf")
    dist_outside = float("inf")
    for k in range(n):
        a = hull[k]
        b = hull[(k + 1) % n]
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length <= 0.0:
            continue
        normal = np.array([-edge[1], edge[0]], dtype=np.float64) / length
        signed = float(normal @ (point - a))
        if signed < 0.0:
            inside = False
        best_inside = min(best_inside, signed)
        t = float(np.clip(float(edge @ (point - a)) / (length * length), 0.0, 1.0))
        dist_outside = min(dist_outside, float(np.linalg.norm(point - (a + t * edge))))
    return best_inside if inside else -dist_outside


__all__ = [
    "Plant",
    "PlantIndices",
    "PlantIntegrityError",
    "ResetAdmissibilityError",
    "build_model",
    "build_spec",
    "model_path",
    "observation_spec",
    "resolve_indices",
]
