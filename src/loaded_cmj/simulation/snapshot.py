"""Exact same-Plant macro-state capture and restore for ML-238.

The snapshot owns only restart state.  It does not advance the plant, own
action projection, or serialize runtime/evaluator state; callers continue to
use :func:`loaded_cmj.simulation.transition.step_5ms` as the sole transition
owner.
"""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.simulation import drive
from loaded_cmj.simulation.constants import (
    ACTION_DIM,
    BALL_JOINT_NAMES,
    COMPILED_NA,
    COMPILED_NQ,
    COMPILED_NU,
    COMPILED_NV,
    COMPILED_NEQ,
    MODEL_ID,
    MODEL_REVISION,
    PHYSICS_TIMESTEP_S,
)
from loaded_cmj.simulation.plant import Plant, model_path


SNAPSHOT_SCHEMA_VERSION = "LCMJ-V1-MACRO-SNAPSHOT-1.0.0"
"""Raw restart schema; the optimizer state is specified separately."""

_STATE_MASK = int(
    mujoco.mjtState.mjSTATE_TIME
    | mujoco.mjtState.mjSTATE_QPOS
    | mujoco.mjtState.mjSTATE_QVEL
    | mujoco.mjtState.mjSTATE_WARMSTART
    | mujoco.mjtState.mjSTATE_CTRL
)
_EXPECTED_DIMENSIONS = (
    COMPILED_NQ,
    COMPILED_NV,
    COMPILED_NU,
    COMPILED_NA,
    COMPILED_NEQ,
    0,  # nmocap
    0,  # nuserdata
    0,  # nplugin
)


class MacroSnapshotError(ValueError):
    """Raised when a snapshot is non-finite, incompatible, or violates V1."""


def _readonly_copy(value: Any, *, dtype: np.dtype, shape: tuple[int, ...], name: str) -> np.ndarray:
    arr = np.asarray(value)
    if arr.shape != shape:
        raise MacroSnapshotError(f"{name} shape {arr.shape} != {shape}")
    if arr.dtype != dtype:
        raise MacroSnapshotError(f"{name} dtype {arr.dtype} != {dtype}")
    out = np.array(arr, dtype=dtype, copy=True, order="C")
    out.setflags(write=False)
    return out


def _finite(name: str, value: np.ndarray) -> None:
    if not np.isfinite(value).all():
        raise MacroSnapshotError(f"{name} contains NaN or Inf")


def _xml_sha256() -> str:
    return sha256(model_path().read_bytes()).hexdigest()


def _dimensions(model: mujoco.MjModel) -> tuple[int, ...]:
    return (
        int(model.nq),
        int(model.nv),
        int(model.nu),
        int(model.na),
        int(model.neq),
        int(model.nmocap),
        int(model.nuserdata),
        int(model.nplugin),
    )


def _kinematic_body_ids(model: mujoco.MjModel) -> tuple[int, ...]:
    """Return the body rows read by the shared transition's SO(3) map."""
    body_ids: list[int] = []
    for joint in BALL_JOINT_NAMES:
        jid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint))
        if jid < 0:
            raise MacroSnapshotError(f"qualified ball joint {joint!r} is missing")
        child = int(model.jnt_bodyid[jid])
        parent = int(model.body_parentid[child])
        if parent <= 0:
            raise MacroSnapshotError(f"ball joint {joint!r} has no body parent")
        for body_id in (parent, child):
            if body_id not in body_ids:
                body_ids.append(body_id)
    return tuple(body_ids)


def _validate_model(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[int, ...]:
    dimensions = _dimensions(model)
    if dimensions != _EXPECTED_DIMENSIONS:
        raise MacroSnapshotError(
            "ML-238 V1 snapshot only supports the qualified model dimensions; "
            f"got {dimensions}, expected {_EXPECTED_DIMENSIONS}"
        )
    if float(model.opt.timestep) != PHYSICS_TIMESTEP_S:
        raise MacroSnapshotError("MuJoCo timestep does not match the frozen V1 timestep")
    if data.qpos.shape != (model.nq,) or data.qvel.shape != (model.nv,):
        raise MacroSnapshotError("MjData qpos/qvel dimensions do not match its model")
    if data.qacc_warmstart.shape != (model.nv,) or data.ctrl.shape != (model.nu,):
        raise MacroSnapshotError("MjData warm-start/control dimensions do not match its model")
    if data.xmat.shape != (model.nbody, 9):
        raise MacroSnapshotError("MjData body-rotation dimensions do not match its model")
    return dimensions


def _copy_flags(flags: dict[str, np.ndarray]) -> tuple[tuple[str, np.ndarray], ...]:
    copied: list[tuple[str, np.ndarray]] = []
    for key in sorted(flags):
        copied.append(
            (
                str(key),
                _readonly_copy(
                    flags[key], dtype=np.dtype(np.bool_), shape=(ACTION_DIM,), name=f"override_flags[{key}]"
                ),
            )
        )
    return tuple(copied)


@dataclass(frozen=True, slots=True)
class MacroSnapshot:
    """Immutable, copy-safe raw restart state for one qualified Plant/MjData."""

    qpos: np.ndarray
    qvel: np.ndarray
    time: float
    qacc_warmstart: np.ndarray
    ctrl: np.ndarray
    kinematic_body_ids: tuple[int, ...]
    kinematic_xmat: np.ndarray
    a_plus: np.ndarray
    a_minus: np.ndarray
    tau_prev: np.ndarray
    previous_command: np.ndarray
    reversal_phase: np.ndarray
    override_flags: tuple[tuple[str, np.ndarray], ...]
    previous_accepted_action: np.ndarray
    schema_version: str
    model_id: str
    model_revision: str
    mujoco_version: str
    model_xml_sha256: str
    dimensions: tuple[int, ...]
    mujoco_state_mask: int
    mujoco_state_size: int
    integrator: int
    solver: int
    solver_iterations: int
    solver_ls_iterations: int

    def __post_init__(self) -> None:
        dimensions = tuple(int(x) for x in self.dimensions)
        object.__setattr__(self, "dimensions", dimensions)
        if dimensions != _EXPECTED_DIMENSIONS:
            raise MacroSnapshotError(f"snapshot dimensions {dimensions} are not the V1 schema")

        nq, nv, nu, *_ = dimensions
        arrays = (
            ("qpos", self.qpos, np.dtype(np.float64), (nq,)),
            ("qvel", self.qvel, np.dtype(np.float64), (nv,)),
            ("qacc_warmstart", self.qacc_warmstart, np.dtype(np.float64), (nv,)),
            ("ctrl", self.ctrl, np.dtype(np.float64), (nu,)),
            (
                "kinematic_xmat",
                self.kinematic_xmat,
                np.dtype(np.float64),
                (len(self.kinematic_body_ids), 9),
            ),
            ("a_plus", self.a_plus, np.dtype(np.float64), (ACTION_DIM,)),
            ("a_minus", self.a_minus, np.dtype(np.float64), (ACTION_DIM,)),
            ("tau_prev", self.tau_prev, np.dtype(np.float64), (ACTION_DIM,)),
            ("previous_command", self.previous_command, np.dtype(np.float64), (ACTION_DIM,)),
            ("reversal_phase", self.reversal_phase, np.dtype(np.int8), (ACTION_DIM,)),
            ("previous_accepted_action", self.previous_accepted_action, np.dtype(np.float64), (ACTION_DIM,)),
        )
        for name, value, dtype, shape in arrays:
            copied = _readonly_copy(value, dtype=dtype, shape=shape, name=name)
            object.__setattr__(self, name, copied)
            if np.issubdtype(dtype, np.floating):
                _finite(name, copied)

        body_ids = tuple(int(x) for x in self.kinematic_body_ids)
        if not body_ids or len(set(body_ids)) != len(body_ids):
            raise MacroSnapshotError("kinematic_body_ids must be a nonempty unique tuple")
        object.__setattr__(self, "kinematic_body_ids", body_ids)

        flags = []
        for key, value in self.override_flags:
            flags.append(
                (
                    str(key),
                    _readonly_copy(
                        value,
                        dtype=np.dtype(np.bool_),
                        shape=(ACTION_DIM,),
                        name=f"override_flags[{key}]",
                    ),
                )
            )
        if len({key for key, _ in flags}) != len(flags):
            raise MacroSnapshotError("override_flags contains duplicate keys")
        if tuple(key for key, _ in flags) != tuple(sorted(key for key, _ in flags)):
            raise MacroSnapshotError("override_flags must be stored in sorted key order")
        object.__setattr__(self, "override_flags", tuple(flags))

        if not np.isfinite(float(self.time)):
            raise MacroSnapshotError("time contains NaN or Inf")
        object.__setattr__(self, "time", float(self.time))
        if self.schema_version != SNAPSHOT_SCHEMA_VERSION:
            raise MacroSnapshotError(f"unsupported snapshot schema {self.schema_version!r}")
        if int(self.mujoco_state_mask) != _STATE_MASK:
            raise MacroSnapshotError("snapshot MuJoCo state mask does not match the frozen V1 mask")

    @classmethod
    def capture(
        cls,
        *,
        plant: Plant,
        data: mujoco.MjData,
        drive_state: drive.DriveState,
        previous_accepted_action: np.ndarray,
    ) -> "MacroSnapshot":
        """Capture selected state without retaining any live NumPy view."""
        dimensions = _validate_model(plant.model, data)
        if not isinstance(drive_state, drive.DriveState):
            raise MacroSnapshotError("capture requires DriveState")
        drive_state._validate()
        if np.any(data.qfrc_applied != 0.0) or np.any(data.xfrc_applied != 0.0):
            raise MacroSnapshotError("V1 forbids nonzero hidden applied-force buffers")

        previous = np.asarray(previous_accepted_action)
        if previous.shape != (ACTION_DIM,) or previous.dtype != np.dtype(np.float64):
            raise MacroSnapshotError("previous accepted action must be float64 with shape (15,)")
        previous = np.array(previous, dtype=np.float64, copy=True)
        _finite("previous_accepted_action", previous)
        if np.any(np.abs(previous) > 1.0 + 1e-12):
            raise MacroSnapshotError("previous accepted action leaves the public [-1, 1] contract")

        state_size = int(mujoco.mj_stateSize(plant.model, _STATE_MASK))
        body_ids = _kinematic_body_ids(plant.model)
        return cls(
            qpos=np.asarray(data.qpos, dtype=np.float64),
            qvel=np.asarray(data.qvel, dtype=np.float64),
            time=float(data.time),
            qacc_warmstart=np.asarray(data.qacc_warmstart, dtype=np.float64),
            ctrl=np.asarray(data.ctrl, dtype=np.float64),
            kinematic_body_ids=body_ids,
            kinematic_xmat=np.asarray(data.xmat[list(body_ids)], dtype=np.float64),
            a_plus=drive_state.a_plus,
            a_minus=drive_state.a_minus,
            tau_prev=drive_state.tau_prev,
            previous_command=drive_state.previous_command,
            reversal_phase=drive_state.reversal_phase,
            override_flags=_copy_flags(drive_state.override_flags),
            previous_accepted_action=previous,
            schema_version=SNAPSHOT_SCHEMA_VERSION,
            model_id=MODEL_ID,
            model_revision=MODEL_REVISION,
            mujoco_version=mujoco.__version__,
            model_xml_sha256=_xml_sha256(),
            dimensions=dimensions,
            mujoco_state_mask=_STATE_MASK,
            mujoco_state_size=state_size,
            integrator=int(plant.model.opt.integrator),
            solver=int(plant.model.opt.solver),
            solver_iterations=int(plant.model.opt.iterations),
            solver_ls_iterations=int(plant.model.opt.ls_iterations),
        )

    def restore(self, *, plant: Plant, data: mujoco.MjData, drive_state: drive.DriveState) -> np.ndarray:
        """Restore raw state and return the sole macro previous-action value.

        This intentionally does not call ``mj_forward``.  The shared
        transition reads the selected kinematic ``xmat`` rows before the first
        ``mj_step``; forwarding would replace the captured post-step cache and
        change the qualified continuation.  MuJoCo refreshes the remaining
        derived contact/solver quantities as part of the step.  Applied forces
        are canonicalized to zero rather than preserved as hidden state.
        """
        dimensions = _validate_model(plant.model, data)
        if (
            self.model_id != MODEL_ID
            or self.model_revision != MODEL_REVISION
            or self.mujoco_version != mujoco.__version__
            or self.model_xml_sha256 != _xml_sha256()
            or self.dimensions != dimensions
            or self.integrator != int(plant.model.opt.integrator)
            or self.solver != int(plant.model.opt.solver)
            or self.solver_iterations != int(plant.model.opt.iterations)
            or self.solver_ls_iterations != int(plant.model.opt.ls_iterations)
            or self.mujoco_state_size
            != int(mujoco.mj_stateSize(plant.model, _STATE_MASK))
            or self.kinematic_body_ids != _kinematic_body_ids(plant.model)
        ):
            raise MacroSnapshotError("snapshot model/runtime identity does not match the live Plant")
        if not isinstance(drive_state, drive.DriveState):
            raise MacroSnapshotError("restore requires DriveState")

        data.qpos[:] = self.qpos
        data.qvel[:] = self.qvel
        data.time = self.time
        data.qacc_warmstart[:] = self.qacc_warmstart
        data.ctrl[:] = self.ctrl
        data.xmat[list(self.kinematic_body_ids), :] = self.kinematic_xmat
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0

        drive_state.a_plus[:] = self.a_plus
        drive_state.a_minus[:] = self.a_minus
        drive_state.tau_prev[:] = self.tau_prev
        drive_state.previous_command[:] = self.previous_command
        drive_state.reversal_phase[:] = self.reversal_phase
        drive_state.override_flags = {key: value.copy() for key, value in self.override_flags}
        drive_state._validate()
        return self.previous_accepted_action.copy()


__all__ = ["MacroSnapshot", "MacroSnapshotError", "SNAPSHOT_SCHEMA_VERSION"]
