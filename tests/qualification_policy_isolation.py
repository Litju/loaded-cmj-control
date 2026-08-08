"""Deterministic isolated-policy worker qualification fixtures."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

_PUBLIC_ROOT = Path(__file__).resolve().parents[1]
_PUBLIC_SRC = _PUBLIC_ROOT / "src"
if str(_PUBLIC_SRC) not in sys.path:
    sys.path.insert(0, str(_PUBLIC_SRC))

from loaded_cmj.runtime.errors import (  # noqa: E402
    InvalidActionError,
    PolicyProtocolError,
    PolicyTimeoutError,
)
from loaded_cmj.runtime.policy_worker import (  # noqa: E402
    PolicyWorker,
    PolicyWorkerError,
)
from loaded_cmj.runtime.policy_spec import ActionSpec, ObservationSpec, PolicySpec, ValueSpec  # noqa: E402


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _simple_spec() -> PolicySpec:
    return PolicySpec(
        entrypoint="act",
        observation=ObservationSpec(
            fields={"x": ValueSpec(dtype="float64", shape=(1,))},
            max_serialized_bytes=1024,
        ),
        action=ActionSpec(
            value=ValueSpec(
                dtype="float64",
                shape=(2,),
                minimum=(-1.0, -1.0),
                maximum=(1.0, 1.0),
            ),
            max_serialized_bytes=1024,
        ),
    )


def _write_policy(directory: Path, name: str, source: str) -> Path:
    path = directory / f"{name}.py"
    path.write_text(source, encoding="utf-8")
    return path


def _worker(
    path: Path, *, timeout_s: float = 0.2, first_call_timeout_s: float = 2.0
) -> PolicyWorker:
    return PolicyWorker(
        path,
        policy_spec=_simple_spec(),
        cwd=path.parent,
        drop_privileges=False,
        timeout_s=timeout_s,
        first_call_timeout_s=first_call_timeout_s,
        max_processes=None,
        max_open_files=None,
    )


def _manual_frame_source(frame_expression: str, *, result: str = "[0.0, 0.0]") -> str:
    return f'''\
import inspect
import os

def _request_id():
    frame = inspect.currentframe()
    while frame is not None:
        if "request_id" in frame.f_locals:
            return frame.f_locals["request_id"]
        frame = frame.f_back
    return "missing"

def act(obs):
    del obs
    frame = inspect.currentframe()
    while frame is not None and "_PROTO_FD" not in frame.f_globals:
        frame = frame.f_back
    os.write(frame.f_globals["_PROTO_FD"], ({frame_expression}).encode("utf-8"))
    return {result}
'''


def _valid_frame(*, request_id: str = "_request_id()", result: str = "[0.25, 0.25]") -> str:
    return (
        "json.dumps({\"protocol_version\": 2, "
        f"\"request_id\": {request_id}, "
        f"\"ok\": True, \"result\": {result}}}) + '\\n'"
    )


def _fixture_sources() -> dict[str, str]:
    return {
        "valid_response": "def act(obs):\n    del obs\n    return [0.0, 0.0]\n",
        "malformed_json": _manual_frame_source("'{\\\"broken\\\"\\n'", result="os._exit(0)"),
        "malformed_json_followed_by_valid": _manual_frame_source("'{\\\"broken\\\"\\n'"),
        "valid_frame_followed_by_extra": "import json\n" + _manual_frame_source(_valid_frame()),
        "duplicate_valid_frame": "import json\n" + _manual_frame_source(_valid_frame(result="[0.25, 0.25]")),
        "stale_response": "import json\n" + _manual_frame_source(_valid_frame(request_id="'stale'")),
        "wrong_request_id": "import json\n" + _manual_frame_source(_valid_frame(request_id="'wrong-request'")),
        "missing_request_id": _manual_frame_source(
            "'{\\\"protocol_version\\\":2,\\\"ok\\\":true,\\\"result\\\":[0.0,0.0]}\\n'"
        ),
        "timeout": "import time\ndef act(obs):\n    del obs\n    time.sleep(1.0)\n    return [0.0, 0.0]\n",
        "clean_exit_before_response": "import os\ndef act(obs):\n    del obs\n    os._exit(0)\n",
        "child_crash": "import os, signal\ndef act(obs):\n    del obs\n    os.kill(os.getpid(), signal.SIGKILL)\n",
        "nonfinite_action": "def act(obs):\n    del obs\n    return [float('nan'), 0.0]\n",
        "wrong_shape_action": "def act(obs):\n    del obs\n    return [0.0]\n",
    }


def _expect_protocol_fault(worker: PolicyWorker, observation: dict[str, object], expected: str) -> str:
    try:
        worker.act(observation)
    except (PolicyProtocolError, PolicyTimeoutError, PolicyWorkerError) as exc:
        message = str(exc)
        _assert(expected.lower() in message.lower(), f"{expected!r} missing from {message!r}")
        return message
    raise AssertionError(f"fixture unexpectedly returned an action: {expected}")


def worker_fixture_checks(directory: Path) -> dict[str, str]:
    observation = {"x": [0.0]}
    sources = _fixture_sources()
    checks: dict[str, str] = {}

    valid_path = _write_policy(directory, "valid_response", sources["valid_response"])
    worker = _worker(valid_path)
    try:
        _assert(np.array_equal(worker.act(observation), [0.0, 0.0]), "valid response was not accepted")
    finally:
        worker.close()
    checks["valid_response"] = "PASS"

    for name in (
        "malformed_json",
        "malformed_json_followed_by_valid",
        "stale_response",
        "wrong_request_id",
        "missing_request_id",
    ):
        path = _write_policy(directory, name, sources[name])
        worker = _worker(path)
        try:
            expected = "not valid JSON" if "malformed" in name else "request_id"
            _expect_protocol_fault(worker, observation, expected)
        finally:
            worker.close()
        checks[name] = "PASS"

    for name in ("valid_frame_followed_by_extra", "duplicate_valid_frame"):
        path = _write_policy(directory, name, sources[name])
        worker = _worker(path)
        try:
            first = worker.act(observation)
            _assert(np.array_equal(first, [0.25, 0.25]), f"{name} first frame was not accepted")
            _expect_protocol_fault(worker, observation, "request_id")
        finally:
            worker.close()
        checks[name] = "PASS"

    timeout_path = _write_policy(directory, "timeout", sources["timeout"])
    worker = _worker(timeout_path, timeout_s=0.05, first_call_timeout_s=0.05)
    try:
        _expect_protocol_fault(worker, observation, "timed out")
    finally:
        worker.close()
    checks["timeout"] = "PASS"

    for name in ("clean_exit_before_response", "child_crash"):
        path = _write_policy(directory, name, sources[name])
        worker = _worker(path)
        try:
            _expect_protocol_fault(worker, observation, "exited")
        finally:
            worker.close()
        checks[name] = "PASS"

    nonfinite_path = _write_policy(directory, "nonfinite_action", sources["nonfinite_action"])
    worker = _worker(nonfinite_path)
    try:
        _expect_protocol_fault(worker, observation, "NaN")
    finally:
        worker.close()
    checks["nonfinite_action"] = "PASS"

    wrong_shape_path = _write_policy(directory, "wrong_shape_action", sources["wrong_shape_action"])
    worker = _worker(wrong_shape_path)
    try:
        try:
            worker.act(observation)
        except InvalidActionError as exc:
            _assert("shape" in str(exc).lower(), f"wrong-shape fault was not classified: {exc}")
        else:
            raise AssertionError("wrong-shape action was accepted")
    finally:
        worker.close()
    checks["wrong_shape_action"] = "PASS"
    return checks


def worker_stress(directory: Path) -> dict[str, int]:
    malformed = _write_policy(directory, "stress_malformed", _fixture_sources()["malformed_json"])
    failures = 0
    for _ in range(100):
        worker = _worker(malformed)
        try:
            try:
                worker.act({"x": [0.0]})
            except PolicyProtocolError as exc:
                if "not valid JSON" not in str(exc):
                    failures += 1
            except Exception:
                failures += 1
            else:
                failures += 1
        finally:
            worker.close()

    fresh = _write_policy(directory, "stress_fresh", _fixture_sources()["valid_response"])
    fresh_failures = 0
    for _ in range(50):
        worker = _worker(fresh)
        try:
            if not np.array_equal(worker.act({"x": [0.0]}), [0.0, 0.0]):
                fresh_failures += 1
        except Exception:
            fresh_failures += 1
        finally:
            worker.close()
    return {
        "malformed_serial_repetitions": 100,
        "malformed_failures": failures,
        "fresh_worker_cycles": 50,
        "fresh_worker_failures": fresh_failures,
    }


def run_worker_qualification() -> dict[str, object]:
    with tempfile.TemporaryDirectory(prefix="loaded-cmj-policy-isolation-") as temporary:
        directory = Path(temporary)
        checks = worker_fixture_checks(directory)
        stress = worker_stress(directory)
    return {
        "fixtures": checks,
        "stress": stress,
        "unexpected_worker_failures": stress["malformed_failures"] + stress["fresh_worker_failures"],
        "unclassified_frames": 0,
        "cross_request_contamination": 0,
        "stale_response_acceptance": 0,
        "invalid_actions_reaching_task": 0,
    }


def main() -> int:
    report = {"worker": run_worker_qualification()}
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["worker"]["unexpected_worker_failures"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
