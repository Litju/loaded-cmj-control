"""One-episode causal rollout engine for Loaded CMJ Control.

This module owns the parent episode clock and the handoff order between the
isolated policy worker, Plant, BiomechanicalSample, CMJEventDetector, and
immutable RolloutResult.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from loaded_cmj.simulation import drive as drive_model  # noqa: E402
from loaded_cmj.biomechanics.events import (  # noqa: E402
    CMJEventDetector,
    BiomechanicalSample,
    TerminationClass,
)
from loaded_cmj.simulation.plant import Plant, PlantIntegrityError, ResetAdmissibilityError, build_model  # noqa: E402
from loaded_cmj.simulation.transition import step_5ms  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    ACTION_DIM,
    MODEL_REVISION,
    CONTROL_PERIOD_S,
    EPISODE_CONTROL_STEPS,
    EPISODE_HORIZON_S,
    EPISODE_PHYSICS_SUBSTEPS,
    FIXED_HOLD_SUBSTEPS,
    HOLD_ACTION,
    PHYSICS_TIMESTEP_S,
    POLICY_PROTOCOL_VERSION,
    SUBSTEPS_PER_CONTROL,
    WORKER_FIRST_CALL_TIMEOUT_S,
    WORKER_MAX_ADDRESS_SPACE_BYTES,
    WORKER_MAX_OPEN_FILES,
    WORKER_MAX_PROCESSES,
    WORKER_MAX_REQUEST_BYTES,
    WORKER_MAX_RESPONSE_BYTES,
    WORKER_MAX_STDERR_CHARS,
    WORKER_MAX_CPU_SECONDS,
    WORKER_STEP_TIMEOUT_S,
)
from loaded_cmj.runtime.errors import (  # noqa: E402
    InternalEvaluationError as RuntimeEvaluationError,
    InvalidActionError,
    InvalidPolicyError,
    PolicyProtocolError,
    PolicyTimeoutError,
)
from loaded_cmj.runtime.observations import validate_action, validate_observation  # noqa: E402
from loaded_cmj.runtime.policy_worker import (  # noqa: E402
    PolicyWorker,
    PolicyWorkerBootstrapError,
    PolicyWorkerConfig,
    PolicyWorkerError,
)
from loaded_cmj.runtime.results import (  # noqa: E402
    EvaluationOutcome,
    RolloutResult,
    TerminationReason,
    canonical_trace_digest,
)
from loaded_cmj.runtime.policy_spec import PolicySpec  # noqa: E402


RUNTIME_REVISION = "loaded-cmj-rollout-1"


def _hash_parts(*parts: bytes) -> str:
    digest = hashlib.sha256()
    for part in parts:
        digest.update(len(part).to_bytes(8, "little"))
        digest.update(part)
    return digest.hexdigest()


def _action_digest(action: np.ndarray) -> str:
    array = np.asarray(action, dtype="<f8").reshape(ACTION_DIM)
    return _hash_parts(b"accepted_action_v1", array.tobytes(order="C"))


def _state_digest(data: mujoco.MjData) -> str:
    parts = [b"post_step_state_v1"]
    for name in ("qpos", "qvel", "qacc", "ctrl"):
        array = np.asarray(getattr(data, name), dtype="<f8")
        parts.extend((name.encode("ascii"), array.tobytes(order="C")))
    parts.extend((b"time", np.asarray([float(data.time)], dtype="<f8").tobytes()))
    return _hash_parts(*parts)


def _trace_digest(
    attempt_id: str,
    samples: tuple[BiomechanicalSample, ...],
) -> str:
    return canonical_trace_digest(attempt_id, samples)


def _json_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _fault_reason(exc: BaseException) -> str:
    if isinstance(exc, PolicyTimeoutError):
        return "POLICY_TIMEOUT"
    if isinstance(exc, (InvalidActionError, PolicyWorkerError)):
        message = str(exc).lower()
        if "exited" in message or "closed" in message:
            return "POLICY_EXITED"
        if "shape" in message:
            return "INVALID_ACTION_SHAPE"
        if "dtype" in message or "type" in message:
            return "INVALID_ACTION_DTYPE"
        if "nan" in message or "infinity" in message:
            return "INVALID_ACTION_NONFINITE"
        if "minimum" in message or "maximum" in message or "bound" in message:
            return "INVALID_ACTION_OUT_OF_BOUNDS"
        if isinstance(exc, PolicyWorkerError):
            return "POLICY_EXCEPTION"
        return "INVALID_ACTION"
    if isinstance(exc, PolicyProtocolError):
        return "POLICY_PROTOCOL_ERROR"
    if isinstance(exc, PolicyWorkerBootstrapError):
        return "POLICY_BOOTSTRAP_ERROR"
    if isinstance(exc, InvalidPolicyError):
        return "POLICY_EXCEPTION"
    return "POLICY_WORKER_ERROR"


def _termination_reason(termination: TerminationClass) -> TerminationReason:
    return {
        TerminationClass.OBJECTIVE_COMPLETE: TerminationReason.OBJECTIVE_COMPLETE,
        TerminationClass.PHYSICAL_FALL: TerminationReason.PHYSICAL_FALL,
        TerminationClass.PHYSICS_NONFINITE_FAULT: TerminationReason.PHYSICS_NONFINITE_FAULT,
        TerminationClass.INCOMPLETE_HORIZON: TerminationReason.INCOMPLETE_HORIZON,
        TerminationClass.AGENT_FAULT: TerminationReason.AGENT_FAULT,
        TerminationClass.INTERNAL_EVALUATION_ERROR: TerminationReason.INTERNAL_EVALUATION_ERROR,
    }[termination]


def _events_projection(result: Any) -> tuple[dict[str, Any], ...]:
    return tuple(
        {
            "name": name,
            "time_s": float(time_s),
            "valid": bool(result.event_valid.get(name, False)),
            "evaluator_revision": MODEL_REVISION,
        }
        for name, time_s in result.events.items()
    )


class RolloutEngine:
    """One fresh-worker, fresh-Plant episode attempt."""

    def __init__(
        self,
        policy_path: Path,
        *,
        attempt_id: str,
        policy_spec_path: Path,
        drop_privileges: bool = True,
    ) -> None:
        # Make the worker cwd absolute without resolving a submitted symlink;
        # PolicyWorker.start() must retain ownership of the regular-file check.
        self.policy_path = Path(policy_path).absolute()
        self.attempt_id = str(attempt_id)
        self.policy_spec_path = Path(policy_spec_path).resolve()
        self.drop_privileges = bool(drop_privileges)
        self.completed_control_steps = 0
        self.completed_physics_steps = 0

    def run(self) -> RolloutResult:
        """Execute exactly one fresh-worker, fresh-Plant causal attempt."""
        model: mujoco.MjModel | None = None
        plant: Plant | None = None
        data: mujoco.MjData | None = None
        evaluator: CMJEventDetector | None = None
        worker: PolicyWorker | None = None
        fault: dict[str, Any] | None = None
        event_result: Any | None = None
        previous_action = np.asarray(HOLD_ACTION, dtype=np.float64).copy()
        drive_state: drive_model.DriveState | None = None
        physics_index = 0
        try:
            spec = PolicySpec.from_json_file(self.policy_spec_path)
            if spec.protocol_version != POLICY_PROTOCOL_VERSION:
                raise RuntimeEvaluationError("policy protocol revision mismatch")
            if spec.entrypoint != "act":
                raise RuntimeEvaluationError("policy entrypoint must be act")
            if spec.action.bounds_behavior != "reject":
                raise RuntimeEvaluationError("policy action bounds behavior must be reject")
            if tuple(spec.action.value.shape or ()) != (ACTION_DIM,):
                raise RuntimeEvaluationError("policy action shape is not the frozen 15-D contract")
            worker_config = PolicyWorkerConfig(
                step_timeout_s=WORKER_STEP_TIMEOUT_S,
                first_call_timeout_s=WORKER_FIRST_CALL_TIMEOUT_S,
                max_request_bytes=WORKER_MAX_REQUEST_BYTES,
                max_response_bytes=WORKER_MAX_RESPONSE_BYTES,
                max_stderr_chars=WORKER_MAX_STDERR_CHARS,
                max_address_space_bytes=WORKER_MAX_ADDRESS_SPACE_BYTES,
                max_processes=WORKER_MAX_PROCESSES,
                max_cpu_seconds=WORKER_MAX_CPU_SECONDS,
                max_open_files=WORKER_MAX_OPEN_FILES,
            )
            evaluator = CMJEventDetector()
            evaluator.reset()
            # One fresh child is created before the task reset and is never
            # reused for another attempt.
            worker = PolicyWorker(
                self.policy_path,
                config=worker_config,
                policy_spec=spec,
                cwd=self.policy_path.parent,
                drop_privileges=self.drop_privileges,
                prepare_policy_access=self.drop_privileges,
            )
            worker.start()

            model = build_model()
            plant = Plant(model)
            data = plant.make_data()
            reset_state = plant.reset_fixed_hold(data)
            if reset_state.reset_metadata["dwell_substeps"] != FIXED_HOLD_SUBSTEPS:
                raise RuntimeEvaluationError("reset dwell metadata is inconsistent")
            if (
                reset_state.reset_metadata.get("policy_calls_during_dwell") != 0
                or reset_state.reset_metadata.get("reset_scored") is not False
                or reset_state.reset_metadata.get("reset_samples_scored") is not False
            ):
                raise RuntimeEvaluationError("reset dwell is not unscored and policy-free")
            if not np.array_equal(reset_state.previous_action, HOLD_ACTION):
                raise RuntimeEvaluationError("reset previous action is not HOLD_ACTION")
            drive_state = drive_model.DriveState(
                a_plus=reset_state.a_plus,
                a_minus=reset_state.a_minus,
                tau_prev=reset_state.previous_torque,
                previous_command=reset_state.previous_command,
                override_flags=reset_state.override_flags,
                reversal_phase=reset_state.reversal_phase,
            )
            observation = validate_observation(reset_state.first_observation, spec.observation)
            if not (
                observation["time_s"] == 0.0
                and observation["step_index"] == 0
                and observation["episode_reset"] is True
                and np.array_equal(observation["previous_action"], HOLD_ACTION)
            ):
                raise RuntimeEvaluationError("first public observation violates the scored t=0 contract")

            for control_index in range(EPISODE_CONTROL_STEPS):
                try:
                    candidate = worker.act(observation)
                    raw_action = np.asarray(
                        validate_action(candidate, spec.action), dtype=np.float64
                    ).reshape(ACTION_DIM).copy()
                except (PolicyTimeoutError, PolicyWorkerError, PolicyProtocolError, InvalidActionError, InvalidPolicyError) as exc:
                    fault = self._policy_fault(
                        exc,
                        control_index=control_index,
                        physics_index=physics_index,
                    )
                    break
                try:
                    controller_observability = worker.debug_state()
                except (PolicyTimeoutError, PolicyWorkerError, PolicyProtocolError) as exc:
                    fault = self._policy_fault(
                        exc,
                        control_index=control_index,
                        physics_index=physics_index,
                    )
                    break

                def on_substep(
                    substep: int,
                    accepted_action: np.ndarray,
                    drive_result: dict[str, Any],
                    realized_tau: np.ndarray,
                    interval_start_power: dict[str, float],
                ) -> bool:
                    nonlocal event_result, fault, physics_index
                    assert model is not None and plant is not None and data is not None
                    try:
                        physics_index += 1
                        self._assert_finite_post_step(
                            model,
                            data,
                            physics_index,
                            base_time_s=FIXED_HOLD_SUBSTEPS * PHYSICS_TIMESTEP_S,
                        )
                        state_digest = _state_digest(data)
                        support_margin = self._support_margin(plant, data)
                        sample = BiomechanicalSample.from_plant(
                            plant,
                            data,
                            time_s=physics_index * PHYSICS_TIMESTEP_S,
                            support_polygon_margin_m=(support_margin, support_margin),
                            realized_anatomical_torque=realized_tau,
                            interval_start_power_components=interval_start_power,
                            accepted_action=accepted_action,
                            controller_observability=controller_observability,
                            actuator_override_flags=drive_result["override_flags"],
                            actuator_capacity_lower_Nm=drive_result["capacity_lower"],
                            actuator_capacity_upper_Nm=drive_result["capacity_upper"],
                            attempt_id=self.attempt_id,
                            control_index=control_index,
                            physics_index=physics_index,
                            accepted_action_digest=_action_digest(accepted_action),
                            post_step_state_digest=state_digest,
                            model_revision=MODEL_REVISION,
                            runtime_revision=RUNTIME_REVISION,
                            extraction_status="LIVE_POST_MJ_STEP",
                        )
                    except Exception as exc:  # noqa: BLE001 - physical boundary fails closed
                        fault = self._physics_fault(
                            exc,
                            control_index=control_index,
                            substep=substep,
                            physics_index=physics_index,
                        )
                        return False

                    self.completed_physics_steps = physics_index
                    try:
                        candidate_event = evaluator.update(sample)
                    except Exception as exc:  # noqa: BLE001 - one evaluator owner
                        fault = self._internal_fault(
                            exc,
                            control_index=control_index,
                            substep=substep,
                            physics_index=physics_index,
                        )
                        return False
                    if candidate_event is not None:
                        event_result = evaluator.finalize(horizon_s=sample.time_s)
                        return False
                    return True

                assert plant is not None and data is not None and drive_state is not None
                try:
                    transition = step_5ms(
                        plant=plant,
                        data=data,
                        drive_state=drive_state,
                        previous_accepted_action=previous_action,
                        raw_action=raw_action,
                        on_substep=on_substep,
                    )
                except Exception as exc:  # noqa: BLE001 - physical boundary fails closed
                    fault = self._physics_fault(
                        exc,
                        control_index=control_index,
                        substep=None,
                        physics_index=physics_index,
                    )
                    break
                if fault is not None or event_result is not None:
                    break
                if transition.substeps_executed != SUBSTEPS_PER_CONTROL:
                    fault = self._internal_fault(
                        RuntimeEvaluationError("shared transition did not complete 40 substeps"),
                        control_index=control_index,
                        substep=None,
                        physics_index=physics_index,
                    )
                    break
                # The action becomes the held command only after the worker
                # response has passed validation and the shared transition has
                # produced its single accepted-action projection.
                previous_action = transition.accepted_action.copy()
                self.completed_control_steps = control_index + 1
                if control_index + 1 < EPISODE_CONTROL_STEPS:
                    assert plant is not None and data is not None
                    observation = validate_observation(
                        plant.public_observation(
                            data,
                            scored_time_s=(control_index + 1) * CONTROL_PERIOD_S,
                            step_index=control_index + 1,
                            episode_reset=False,
                            previous_action=previous_action,
                        ),
                        spec.observation,
                    )

            if event_result is None and evaluator.live_trace:
                event_result = evaluator.finalize(horizon_s=EPISODE_HORIZON_S)
            elif event_result is None:
                event_result = None
        except (PolicyWorkerBootstrapError, RuntimeEvaluationError) as exc:
            fault = self._internal_fault(
                exc,
                control_index=self.completed_control_steps,
                substep=None,
                physics_index=physics_index,
            )
        except (
            PolicyTimeoutError,
            PolicyWorkerError,
            PolicyProtocolError,
            InvalidActionError,
            InvalidPolicyError,
        ) as exc:
            fault = self._policy_fault(
                exc,
                control_index=self.completed_control_steps,
                physics_index=physics_index,
            )
        except (ResetAdmissibilityError, PlantIntegrityError) as exc:
            fault = self._physics_fault(
                exc,
                control_index=self.completed_control_steps,
                substep=None,
                physics_index=physics_index,
            )
        except Exception as exc:  # noqa: BLE001 - trusted runtime fails closed
            fault = self._internal_fault(
                exc,
                control_index=self.completed_control_steps,
                substep=None,
                physics_index=physics_index,
            )
        finally:
            if worker is not None:
                try:
                    worker.close()
                except Exception as exc:  # noqa: BLE001 - cleanup is part of the rollout boundary
                    fault = self._policy_fault(
                        exc,
                        control_index=self.completed_control_steps,
                        physics_index=physics_index,
                        cleanup=True,
                    )

        if evaluator is not None and evaluator.live_trace:
            if event_result is None:
                event_result = evaluator.finalize(horizon_s=min(
                    EPISODE_HORIZON_S,
                    evaluator.live_trace[-1].time_s,
                ))
            trace = evaluator.live_trace
        else:
            trace = ()
        return self._build_result(
            trace=trace,
            event_result=event_result,
            fault=fault,
            physics_index=physics_index,
        )

    @staticmethod
    def _assert_finite_post_step(
        model: mujoco.MjModel,
        data: mujoco.MjData,
        physics_index: int,
        *,
        base_time_s: float,
    ) -> None:
        values = (
            data.qpos,
            data.qvel,
            data.qacc,
            data.ctrl,
            data.qfrc_bias,
            data.qfrc_passive,
            data.qfrc_actuator,
            data.qfrc_constraint,
        )
        if not all(np.isfinite(np.asarray(value)).all() for value in values):
            raise PlantIntegrityError(f"non-finite MuJoCo state after physics index {physics_index}")
        if not np.isfinite(float(data.time)):
            raise PlantIntegrityError(f"non-finite MuJoCo time after physics index {physics_index}")
        expected = float(base_time_s) + physics_index * PHYSICS_TIMESTEP_S
        if abs(float(data.time) - expected) > 1e-12:
            raise PlantIntegrityError(
                f"MuJoCo time {data.time!r} disagrees with integer physics index {physics_index}"
            )
        if int(model.nv) <= 0:
            raise PlantIntegrityError("compiled Plant has no generalized velocities")

    @staticmethod
    def _support_margin(plant: Plant, data: mujoco.MjData) -> float:
        summary = plant.contact_wrench_summary(data)
        active = tuple(bool(item) for item in np.asarray(summary["active_by_foot"], dtype=bool))
        margin = plant.support_margin(data, plant.center_of_mass(data), active)
        return float(margin) if np.isfinite(margin) else -1.0

    def _policy_fault(
        self,
        exc: BaseException,
        *,
        control_index: int,
        physics_index: int,
        cleanup: bool = False,
    ) -> dict[str, Any]:
        reason = "WORKER_CLEANUP_FAILURE" if cleanup else _fault_reason(exc)
        return {
            "fault_class": "POLICY_WORKER_FAULT",
            "reason": reason,
            "exception_type": type(exc).__name__,
            "attempt_id": self.attempt_id,
            "control_index": int(control_index),
            "physics_index": int(physics_index),
            "request_id": f"{self.attempt_id}:control:{int(control_index):04d}",
            "child_cleanup_required": True,
        }

    def _physics_fault(
        self,
        exc: BaseException,
        *,
        control_index: int,
        substep: int | None,
        physics_index: int,
    ) -> dict[str, Any]:
        return {
            "fault_class": "PHYSICS_NONFINITE_FAULT",
            "reason": "PHYSICS_OR_EXTRACTION_FAILURE",
            "exception_type": type(exc).__name__,
            "attempt_id": self.attempt_id,
            "control_index": int(control_index),
            "substep": None if substep is None else int(substep),
            "physics_index": int(physics_index),
            "sample_constructed": False,
        }

    def _internal_fault(
        self,
        exc: BaseException,
        *,
        control_index: int,
        substep: int | None,
        physics_index: int,
    ) -> dict[str, Any]:
        return {
            "fault_class": "INTERNAL_EVALUATION_ERROR",
            "reason": "TRUSTED_RUNTIME_FAILURE",
            "exception_type": type(exc).__name__,
            "attempt_id": self.attempt_id,
            "control_index": int(control_index),
            "substep": None if substep is None else int(substep),
            "physics_index": int(physics_index),
        }

    def _build_result(
        self,
        *,
        trace: tuple[BiomechanicalSample, ...],
        event_result: Any | None,
        fault: dict[str, Any] | None,
        physics_index: int,
    ) -> RolloutResult:
        if fault is not None:
            if fault["fault_class"] == "INTERNAL_EVALUATION_ERROR":
                termination = TerminationReason.INTERNAL_EVALUATION_ERROR
                outcome = EvaluationOutcome.INTERNAL_ERROR
                evaluation_outcome = EvaluationOutcome.INTERNAL_ERROR.value
            elif fault["fault_class"] == "PHYSICS_NONFINITE_FAULT":
                termination = TerminationReason.PHYSICS_NONFINITE_FAULT
                outcome = EvaluationOutcome.INTERNAL_ERROR
                evaluation_outcome = EvaluationOutcome.INTERNAL_ERROR.value
            else:
                termination = TerminationReason.AGENT_FAULT
                outcome = EvaluationOutcome.INVALID_POLICY
                evaluation_outcome = EvaluationOutcome.AGENT_FAULT.value
            result_events: tuple[dict[str, Any], ...] = (
                _events_projection(event_result) if event_result is not None else ()
            )
            physical_metrics = event_result.raw_metrics if event_result is not None else {}
            episode_terminated = True
            rollout_valid = False
            complete_rollout = False
        elif event_result is not None:
            termination = _termination_reason(event_result.termination_class)
            result_events = _events_projection(event_result)
            physical_metrics = event_result.raw_metrics
            if event_result.termination_class is TerminationClass.OBJECTIVE_COMPLETE:
                outcome = EvaluationOutcome.OK
                evaluation_outcome = EvaluationOutcome.OBJECTIVE_SUCCESS.value
            elif event_result.termination_class is TerminationClass.PHYSICAL_FALL:
                outcome = EvaluationOutcome.OK
                evaluation_outcome = EvaluationOutcome.PHYSICAL_FAILURE.value
            elif event_result.termination_class is TerminationClass.INCOMPLETE_HORIZON:
                outcome = EvaluationOutcome.OK
                evaluation_outcome = EvaluationOutcome.INCOMPLETE.value
            else:
                outcome = EvaluationOutcome.INTERNAL_ERROR
                evaluation_outcome = EvaluationOutcome.INTERNAL_ERROR.value
            episode_terminated = True
            rollout_valid = event_result.termination_class in {
                TerminationClass.OBJECTIVE_COMPLETE,
                TerminationClass.PHYSICAL_FALL,
                TerminationClass.INCOMPLETE_HORIZON,
            }
            complete_rollout = rollout_valid
        else:
            termination = TerminationReason.INTERNAL_EVALUATION_ERROR
            outcome = EvaluationOutcome.INTERNAL_ERROR
            evaluation_outcome = EvaluationOutcome.INTERNAL_ERROR.value
            result_events = ()
            physical_metrics = {}
            episode_terminated = True
            rollout_valid = False
            complete_rollout = False

        first_time = trace[0].time_s if trace else None
        final_time = trace[-1].time_s if trace else None
        trace_identity = _trace_digest(self.attempt_id, trace)
        metadata = {
            "attempt_id": self.attempt_id,
            "termination_reason": termination.value,
            "sample_count": len(trace),
            "completed_control_steps": self.completed_control_steps,
            "completed_physics_steps": physics_index,
            "trace_identity": trace_identity,
            "fault": fault,
        }
        evidence_identity = _json_digest(metadata)
        return RolloutResult(
            outcome=outcome,
            termination_reason=termination,
            completed_steps=self.completed_control_steps,
            objective_completed=termination is TerminationReason.OBJECTIVE_COMPLETE,
            metrics={
                "sample_count": float(len(trace)),
                "completed_control_steps": float(self.completed_control_steps),
                "completed_physics_steps": float(physics_index),
            },
            attempt_id=self.attempt_id,
            model_revision=MODEL_REVISION,
            runtime_revision=RUNTIME_REVISION,
            evaluation_outcome=evaluation_outcome,
            physical_metrics=physical_metrics,
            events=result_events,
            fault_data=fault or {},
            trace=trace,
            trace_identity=trace_identity,
            sample_count=len(trace),
            first_sample_time_s=first_time,
            final_sample_time_s=final_time,
            completed_control_steps=self.completed_control_steps,
            completed_physics_steps=physics_index,
            simulated_duration_s=float(final_time or 0.0),
            rollout_valid=rollout_valid,
            episode_terminated=episode_terminated,
            complete_rollout=complete_rollout,
            evidence_identity=evidence_identity,
        )


def run_rollout(
    policy_path: Path,
    *,
    attempt_id: str = "qualification-000001",
    policy_spec_path: Path,
    drop_privileges: bool = True,
) -> RolloutResult:
    """Run one deterministic fresh-worker rollout."""
    return RolloutEngine(
        policy_path,
        attempt_id=attempt_id,
        policy_spec_path=policy_spec_path,
        drop_privileges=drop_privileges,
    ).run()


__all__ = [
    "RUNTIME_REVISION",
    "RolloutEngine",
    "run_rollout",
]
