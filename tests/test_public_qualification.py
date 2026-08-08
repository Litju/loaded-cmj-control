"""Run the owner-authored scientific qualification suites in the public tree."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest


PUBLIC_ROOT = Path(__file__).resolve().parents[1]
QUALIFICATION_SUITES = (
    "qualification_mechanics.py",
    "qualification_actuator_dynamics.py",
    "qualification_passive_energetics.py",
    "qualification_contact_force_plates.py",
    "qualification_numerical_mechanics.py",
    "qualification_determinism.py",
    "qualification_cmj_events.py",
    "qualification_semantic_contracts.py",
    "qualification_policy_isolation.py",
    "qualification_causal_runtime.py",
)


@pytest.mark.parametrize("suite", QUALIFICATION_SUITES)
def test_qualification_suite_passes(suite: str, tmp_path: Path) -> None:
    evidence_root = tmp_path / Path(suite).stem
    env = os.environ.copy()
    source_path = str(PUBLIC_ROOT / "src")
    env["PYTHONPATH"] = os.pathsep.join(
        item for item in (source_path, str(PUBLIC_ROOT), env.get("PYTHONPATH", "")) if item
    )
    command = [sys.executable, str(PUBLIC_ROOT / "tests" / suite), "--evidence-root", str(evidence_root)]
    completed = subprocess.run(
        command,
        cwd=PUBLIC_ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, (
        f"{suite} failed with exit {completed.returncode}\n"
        f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
    )
    assert "PASS" in completed.stdout or suite == "qualification_policy_isolation.py"
