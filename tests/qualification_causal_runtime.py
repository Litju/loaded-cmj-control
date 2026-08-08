#!/usr/bin/env python3
"""Focused qualification for the trusted causal runtime.

All generated policies in this module are qualification sentinels. They are
not controllers.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_TASK_ROOT = Path(__file__).resolve().parents[1]
_PUBLIC_SRC = _TASK_ROOT / "src"
if str(_PUBLIC_SRC) not in sys.path:
    sys.path.insert(0, str(_PUBLIC_SRC))

from loaded_cmj.simulation import drive as drive_model  # noqa: E402
from loaded_cmj.biomechanics.events import CMJEventDetector, BiomechanicalSample, TerminationClass  # noqa: E402
from loaded_cmj.simulation.constants import (  # noqa: E402
    ACTION_DIM,
    CONTROL_PERIOD_S,
    EPISODE_CONTROL_STEPS,
    EPISODE_HORIZON_S,
    EPISODE_PHYSICS_SUBSTEPS,
    FIXED_HOLD_SUBSTEPS,
    HOLD_ACTION,
    PHYSICS_TIMESTEP_S,
    SUBSTEPS_PER_CONTROL,
)
from loaded_cmj.runtime.policy_worker import PolicyWorker  # noqa: E402
from loaded_cmj.runtime.errors import (  # noqa: E402
    InvalidActionError,
    PolicyProtocolError,
    PolicyTimeoutError,
)
from loaded_cmj.runtime.policy_worker import PolicyWorkerError  # noqa: E402
from loaded_cmj.runtime.results import (  # noqa: E402
    RolloutResult,
)
from loaded_cmj.runtime.policy_spec import PolicySpec  # noqa: E402
from loaded_cmj.runtime.engine import run_rollout  # noqa: E402
from tests.qualification_cmj_events import valid_trace  # noqa: E402


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _write_policy(directory: Path, name: str, source: str) -> Path:
    path = directory / f"{name}.py"
    path.write_text(source, encoding="utf-8")
    return path


def _legal_source() -> str:
    return "def act(obs):\n    del obs\n    return [0.0] * 15\n"


def _runtime_previous_action_source() -> str:
    return """\
calls = 0

def reset():
    raise RuntimeError("reset() must never be called")

def act(obs):
    global calls
    calls += 1
    if calls == 1:
        return [0.0] * 15
    return obs["previous_action"]
"""


def _task_observation() -> tuple[PolicySpec, dict[str, object]]:
    from loaded_cmj.simulation.plant import Plant, build_model

    spec = PolicySpec.from_json_file(_TASK_ROOT / "src" / "loaded_cmj" / "control" / "policy_spec.json")
    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    _assert(reset.reset_metadata["dwell_substeps"] == FIXED_HOLD_SUBSTEPS, "reset dwell is not 2400 steps")
    _assert(np.array_equal(reset.first_observation["previous_action"], np.asarray(HOLD_ACTION)), "first previous_action is not HOLD_ACTION")
    return spec, reset.first_observation


def _worker_call(
    policy_path: Path,
    spec: PolicySpec,
    observation: dict[str, object],
    **kwargs: Any,
) -> Any:
    worker = PolicyWorker(
        policy_path,
        policy_spec=spec,
        cwd=policy_path.parent,
        drop_privileges=False,
        **kwargs,
    )
    try:
        worker.start()
        return worker.act(observation)
    finally:
        worker.close()


def _worker_binding_checks(directory: Path) -> dict[str, object]:
    spec, observation = _task_observation()
    checks: dict[str, object] = {}

    legal = _write_policy(directory, "valid", _legal_source())
    action = _worker_call(legal, spec, observation)
    _assert(np.asarray(action).shape == (ACTION_DIM,), "valid worker action did not have 15 channels")
    _assert(np.isfinite(action).all(), "valid worker action was not finite")
    checks["valid_action"] = "PASS"

    symlink_path = directory / "policy_symlink.py"
    symlink_path.symlink_to(legal)
    try:
        _worker_call(symlink_path, spec, observation)
    except PolicyWorkerError:
        pass
    else:
        raise AssertionError("worker followed a submitted policy symlink")
    checks["regular_file_artifact_boundary"] = "PASS"

    invalid_sources = {
        "wrong_shape": "def act(obs):\n    del obs\n    return [0.0] * 14\n",
        "nan": "def act(obs):\n    del obs\n    return [float('nan')] * 15\n",
        "inf": "def act(obs):\n    del obs\n    return [float('inf')] * 15\n",
        "out_of_range": "def act(obs):\n    del obs\n    return [2.0] * 15\n",
    }
    for name, source in invalid_sources.items():
        path = _write_policy(directory, f"invalid_{name}", source)
        try:
            _worker_call(path, spec, observation)
        except (InvalidActionError, PolicyWorkerError) as exc:
            message = str(exc).lower()
            if name == "wrong_shape":
                _assert("shape" in message, "wrong-shape action did not identify shape")
            elif name in {"nan", "inf"}:
                _assert("nan" in message or "infinity" in message, f"{name} was not rejected as nonfinite")
            else:
                _assert("maximum" in message or "bound" in message, "out-of-range action did not identify bounds")
        else:
            raise AssertionError(f"{name} action was accepted")
        checks[f"invalid_{name}"] = "PASS"

    timeout_source = """\
import time
calls = 0
def act(obs):
    global calls
    calls += 1
    if calls > 1:
        time.sleep(1.0)
    return [0.0] * 15
"""
    timeout_path = _write_policy(directory, "timeout", timeout_source)
    worker = PolicyWorker(
        timeout_path,
        policy_spec=spec,
        cwd=timeout_path.parent,
        drop_privileges=False,
        timeout_s=0.05,
        first_call_timeout_s=2.0,
    )
    try:
        worker.start()
        worker.act(observation)
        try:
            worker.act(observation)
        except PolicyTimeoutError:
            pass
        else:
            raise AssertionError("timeout sentinel returned instead of timing out")
    finally:
        worker.close()
    checks["timeout"] = "PASS"

    crash_path = _write_policy(
        directory,
        "crash",
        "def act(obs):\n    del obs\n    raise RuntimeError('qualification crash')\n",
    )
    try:
        _worker_call(crash_path, spec, observation)
    except PolicyWorkerError:
        pass
    else:
        raise AssertionError("crash sentinel returned instead of failing")
    checks["crash"] = "PASS"

    protocol_source = """\
import inspect
import json
import os

def act(obs):
    del obs
    frame = inspect.currentframe()
    request_id = "stale"
    while frame is not None:
        if frame.f_locals.get("request_id"):
            request_id = str(frame.f_locals["request_id"]) + "-stale"
            break
        frame = frame.f_back
    payload = json.dumps({
        "protocol_version": 2,
        "request_id": request_id,
        "ok": True,
        "result": [0.0] * 15,
    }) + "\\n"
    for fd in range(3, 64):
        try:
            os.write(fd, payload.encode())
        except OSError:
            pass
    return [0.0] * 15
"""
    protocol_path = _write_policy(directory, "protocol_mismatch", protocol_source)
    try:
        _worker_call(protocol_path, spec, observation)
    except PolicyProtocolError:
        pass
    else:
        raise AssertionError("stale response was accepted")
    checks["protocol_correlation_mismatch"] = "PASS"

    fresh_source = """\
calls = 0
def act(obs):
    global calls
    calls += 1
    return [0.25 if calls == 1 else 0.0] + [0.0] * 14
"""
    fresh_path = _write_policy(directory, "fresh_process", fresh_source)
    workers: list[tuple[PolicyWorker, int, list[float]]] = []
    for _ in range(2):
        worker = PolicyWorker(
            fresh_path,
            policy_spec=spec,
            cwd=fresh_path.parent,
            drop_privileges=False,
        )
        worker.start()
        first = worker.act(observation)
        pid = int(worker._proc.pid) if worker._proc is not None else -1
        workers.append((worker, pid, first))
    try:
        _assert(np.array_equal(workers[0][2], workers[1][2]), "fresh workers shared module state")
        _assert(workers[0][1] != workers[1][1], "fresh attempts reused one child process")
    finally:
        for worker, _pid, _first in workers:
            worker.close()
            _assert(worker._proc is None, "worker cleanup did not clear child handle")
    checks["fresh_process_isolation_and_cleanup"] = "PASS"
    return checks


def _runtime_checks(directory: Path) -> tuple[dict[str, object], RolloutResult]:
    policy = _write_policy(directory, "runtime_previous_action", _runtime_previous_action_source())
    result = run_rollout(
        policy,
        attempt_id="causal-runtime-qualification",
        policy_spec_path=_TASK_ROOT / "src" / "loaded_cmj" / "control" / "policy_spec.json",
        drop_privileges=False,
    )
    checks: dict[str, object] = {}
    _assert(PHYSICS_TIMESTEP_S == 0.000125, "physics timestep contract changed")
    _assert(SUBSTEPS_PER_CONTROL == 40, "physics steps per control contract changed")
    _assert(CONTROL_PERIOD_S == 0.005, "control period contract changed")
    _assert(EPISODE_CONTROL_STEPS == 800, "control horizon contract changed")
    _assert(EPISODE_PHYSICS_SUBSTEPS == 32000, "physics horizon contract changed")
    _assert(EPISODE_HORIZON_S == 4.0, "episode horizon contract changed")
    checks["runtime_frozen_clock_contract"] = "PASS"
    _assert(result.rollout_valid and result.complete_rollout, "finite physical terminal was not a complete rollout")
    _assert(result.termination_reason.value in {"PHYSICAL_FALL", "OBJECTIVE_COMPLETE", "INCOMPLETE_HORIZON"}, "unexpected physical terminal")
    _assert(result.sample_count == len(result.trace) > 40, "runtime did not emit a usable post-step trace")
    _assert(result.first_sample_time_s == PHYSICS_TIMESTEP_S, "first scored sample was not h")
    _assert(all(sample.time_s > 0.0 for sample in result.trace), "trace contains a scored t=0 sample")
    _assert(all(result.trace[index].time_s < result.trace[index + 1].time_s for index in range(len(result.trace) - 1)), "trace timestamps are not strict")
    _assert(all(sample.physics_index == index + 1 for index, sample in enumerate(result.trace)), "physics indices are not one-to-one")
    _assert(all(np.isclose(sample.time_s, sample.physics_index * PHYSICS_TIMESTEP_S, atol=0.0, rtol=0.0) for sample in result.trace), "timestamps are not integer-derived")
    _assert(all(sample.control_index == (sample.physics_index - 1) // SUBSTEPS_PER_CONTROL for sample in result.trace), "control/physics ownership is misaligned")
    _assert(all(sample.attempt_id == result.attempt_id for sample in result.trace), "attempt provenance missing")
    _assert(all(sample.accepted_action_digest and sample.post_step_state_digest for sample in result.trace), "action/state provenance missing")
    _assert(all(sample.model_revision and sample.runtime_revision for sample in result.trace), "revision provenance missing")
    _assert(all(sample.extraction_status == "LIVE_POST_MJ_STEP" for sample in result.trace), "live extraction status is not post-mj_step")

    first_digest = result.trace[0].accepted_action_digest
    first_block = [sample for sample in result.trace if sample.control_index == 0]
    _assert(len(first_block) == SUBSTEPS_PER_CONTROL, "accepted action was not held for exactly 40 substeps")
    _assert(all(sample.accepted_action_digest == first_digest for sample in first_block), "held action identity changed within one control block")
    second_block = [sample for sample in result.trace if sample.control_index == 1]
    if second_block:
        _assert(all(sample.accepted_action_digest == first_digest for sample in second_block), "previous_action was not the last accepted action")
    checks["runtime_exact_clock_action_hold_and_previous_action"] = "PASS"
    checks["runtime_live_provenance"] = "PASS"

    _assert(result.sample_count == len(result.trace), "result trace/sample count diverged")
    _assert(result.complete_rollout and result.rollout_valid, "valid physical terminal classification is wrong")
    checks["result_classification"] = "PASS"

    try:
        result.trace[0].com_position_m[0] = 99.0
    except ValueError:
        pass
    else:
        raise AssertionError("sealed causal sample array remained writable")
    checks["result_trace_immutability"] = "PASS"

    invalid = _write_policy(directory, "runtime_invalid", "def act(obs):\n    del obs\n    return [0.0] * 14\n")
    invalid_result = run_rollout(
        invalid,
        attempt_id="causal-runtime-invalid",
        policy_spec_path=_TASK_ROOT / "src" / "loaded_cmj" / "control" / "policy_spec.json",
        drop_privileges=False,
    )
    _assert(not invalid_result.rollout_valid and not invalid_result.complete_rollout, "invalid action was classified as valid")
    _assert(invalid_result.completed_physics_steps == 0 and invalid_result.sample_count == 0, "invalid action mutated physics or trace")
    _assert(invalid_result.fault_data["reason"] == "INVALID_ACTION_SHAPE", "invalid action fault metadata was not deterministic")
    checks["invalid_action_before_mutation"] = "PASS"
    return checks, result


def _same_state_physical_sample_check() -> dict[str, object]:
    from loaded_cmj.simulation.plant import Plant, build_model

    model = build_model()
    plant = Plant(model)
    data = plant.make_data()
    reset = plant.reset_fixed_hold(data)
    drive_state = drive_model.DriveState(
        a_plus=reset.a_plus,
        a_minus=reset.a_minus,
        tau_prev=reset.previous_torque,
        previous_command=reset.previous_command,
        override_flags=reset.override_flags,
        reversal_phase=reset.reversal_phase,
    )
    action = np.zeros(ACTION_DIM, dtype=np.float64)
    drive_result = drive_model.drive_state_step(
        action,
        drive_state,
        plant.anatomical_coordinates(data),
        plant.anatomical_rates(data),
        PHYSICS_TIMESTEP_S,
    )
    torque = np.asarray(drive_result["tau"], dtype=np.float64).copy()
    interval_power = plant.realized_power_components(data, torque)
    plant.apply_anatomical_torque(data, torque)
    mujoco.mj_step(model, data)
    sample = BiomechanicalSample.from_plant(
        plant,
        data,
        time_s=PHYSICS_TIMESTEP_S,
        realized_anatomical_torque=torque,
        interval_start_power_components=interval_power,
        attempt_id="causal-sample-identity",
        control_index=0,
        physics_index=1,
        accepted_action_digest="action",
        post_step_state_digest="state",
        model_revision="task",
        runtime_revision="runtime",
    )
    summary = plant.contact_wrench_summary(data)
    _assert(np.array_equal(sample.com_position_m, plant.center_of_mass(data)), "sample COM was not read from post-step data")
    _assert(np.array_equal(sample.com_velocity_mps, plant.center_of_mass_velocity(data)), "sample COM velocity was not read from post-step data")
    _assert(np.array_equal(sample.plate_wrench_N_Nm, np.asarray(summary["plate_wrench"])), "sample plate wrench was not read from post-step data")
    before = sample.com_position_m.copy()
    data.qpos[0] += 1.0e-3
    _assert(np.array_equal(sample.com_position_m, before), "sample array aliased mutable Plant state")
    return {"same_state_physical_sample": "PASS"}


def _streaming_event_checks() -> dict[str, object]:
    samples = valid_trace()
    live = CMJEventDetector()
    calls = 0
    original = live._evaluate_validated

    def counted(trace: list[BiomechanicalSample], *, horizon_s: float | None) -> Any:
        nonlocal calls
        calls += 1
        return original(trace, horizon_s=horizon_s)

    live._evaluate_validated = counted  # type: ignore[method-assign]
    prefix: list[BiomechanicalSample] = []
    live_result = None
    for sample in samples:
        prefix.append(sample)
        live_result = live.update(sample)
        if live_result is not None:
            break
    _assert(live_result is not None, "live event detector did not detect the deterministic terminal")
    live_result = live.finalize(horizon_s=prefix[-1].time_s)
    sealed_result = CMJEventDetector().evaluate(prefix, horizon_s=prefix[-1].time_s)
    _assert(calls <= 2, "live event detector used repeated growing-prefix evaluation")
    _assert(live_result.to_jsonable() == sealed_result.to_jsonable(), "live detector differs from sealed detector")
    _assert(live_result.termination_class is TerminationClass.OBJECTIVE_COMPLETE, "live event-detector terminal class changed")
    _assert(len(live.live_trace) == len(prefix), "live trace length changed after sealing")
    try:
        live.update(samples[len(prefix)])
    except Exception:
        pass
    else:
        raise AssertionError("sealed live event accepted a post-terminal sample")
    return {
        "streaming_event_equivalence": "PASS",
        "terminal_sample_inclusion": "PASS",
        "sealed_evaluator_calls": calls,
        "terminal_sample_time_s": prefix[-1].time_s,
        "terminal_sample_count": len(prefix),
    }


def _work_checks() -> dict[str, object]:
    power_W = 7.0
    rows: dict[str, float] = {}
    for count in (1, 2, 40):
        samples = []
        for index in range(1, count + 1):
            samples.append(
                BiomechanicalSample(
                    time_s=index * PHYSICS_TIMESTEP_S,
                    com_position_m=(0.0, 0.0, 1.0),
                    com_velocity_mps=(0.0, 0.0, 0.0),
                    plate_wrench_N_Nm=np.zeros((3, 6)),
                    active_power_signed_W=power_W,
                    active_power_positive_W=power_W,
                    active_power_negative_W=0.0,
                    interval_start_active_power_signed_W=power_W,
                    interval_start_active_power_positive_W=power_W,
                    interval_start_active_power_negative_W=0.0,
                    interval_start_passive_power_W=0.0,
                    interval_start_damping_power_W=0.0,
                    interval_start_limit_power_W=0.0,
                )
            )
        result = CMJEventDetector().evaluate(samples, horizon_s=samples[-1].time_s)
        measured = float(result.raw_metrics["work"]["active_work_cumulative_J"])
        expected = power_W * count * PHYSICS_TIMESTEP_S
        _assert(np.isclose(measured, expected, atol=1e-15, rtol=0.0), f"first interval work mismatch for N={count}")
        rows[str(count)] = measured
    return {"first_interval_work": "PASS", "measured_J": rows}


def run() -> dict[str, object]:
    with tempfile.TemporaryDirectory(prefix="loaded-cmj-causal-") as temporary:
        directory = Path(temporary)
        worker = _worker_binding_checks(directory)
        runtime, runtime_result = _runtime_checks(directory)
        same_state = _same_state_physical_sample_check()
    streaming = _streaming_event_checks()
    work = _work_checks()
    checks = {**worker, **runtime, **same_state, **streaming, **work}
    return {
        "result": "PASS",
        "checks": checks,
        "focused_tests": len(checks),
        "focused_passed": len(checks),
        "runtime": {
            "termination": runtime_result.termination_reason.value,
            "sample_count": runtime_result.sample_count,
            "completed_physics_steps": runtime_result.completed_physics_steps,
            "first_sample_time_s": runtime_result.first_sample_time_s,
            "final_sample_time_s": runtime_result.final_sample_time_s,
            "rollout_valid": runtime_result.rollout_valid,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path)
    args = parser.parse_args()
    report = run()
    if args.evidence_root is not None:
        args.evidence_root.mkdir(parents=True, exist_ok=True)
        (args.evidence_root / "causal_runtime.json").write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
        )
    print(f"FOCUSED tests={report['focused_tests']} passed={report['focused_passed']}")
    print(f"RESULT={report['result']} CAUSAL-RUNTIME")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
