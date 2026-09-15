#!/usr/bin/env python3
"""Seal the RES-83 evidence bundle into the external evidence root.

Follows the RES-82 / RES-95 convention:
  checksums.sha256 covers every copied bundle file except checksums.sha256,
  SEAL.json and POSTCOMMIT_SIDECAR*;
  EVIDENCE_SEAL_SHA256 = sha256(checksums.sha256 bytes).

Modes:
  python3 seal_evidence.py --external [--postcommit]
  python3 seal_evidence.py --verify

--postcommit additionally records FINAL_HEAD/FINAL_TREE/REMOTE_SYNC/
TRACKED_WORKTREE_CLEAN (read from git) in POSTCOMMIT_SIDECAR.json.
"""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
EXTERNAL_ROOT = Path("/home/litju/Projects/loaded-cmj-control-evidence") / HERE.name
EXCLUDE_NAMES = {"checksums.sha256", "SEAL.json"}
EXCLUDE_PREFIX = ("POSTCOMMIT_SIDECAR",)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True).stdout.strip()


def seal_external() -> dict:
    if EXTERNAL_ROOT.exists():
        for child in EXTERNAL_ROOT.iterdir():
            if child.is_file() or child.is_symlink():
                child.unlink()
            elif child.is_dir():
                shutil.rmtree(child)
    else:
        EXTERNAL_ROOT.mkdir(parents=True)
    copied: list[Path] = []
    for path in sorted(HERE.rglob("*")):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        if path.name in EXCLUDE_NAMES or path.name.startswith(EXCLUDE_PREFIX):
            continue
        target = EXTERNAL_ROOT / path.relative_to(HERE)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        copied.append(target)
    lines = [f"{sha256(t)}  {t.relative_to(EXTERNAL_ROOT)}" for t in sorted(copied)]
    (EXTERNAL_ROOT / "checksums.sha256").write_text("\n".join(lines) + "\n")
    seal_hash = sha256(EXTERNAL_ROOT / "checksums.sha256")
    seal = {
        "schema_version": "1.0.0",
        "MISSION": "RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001",
        "LINEAR_ISSUE": "RES-83",
        "ACHIEVEMENT": "V3 elite-soccer human-valid loaded-CMJ Plant implemented and sealed",
        "BUNDLE_PATH": str(EXTERNAL_ROOT),
        "CHECKSUMS_FILE": "checksums.sha256",
        "CHECKSUMS_SCOPE": "all bundle files except checksums.sha256, SEAL.json, POSTCOMMIT_SIDECAR*",
        "EVIDENCE_SEAL_SHA256": seal_hash,
        "file_count": len(lines),
        "sealed_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    (EXTERNAL_ROOT / "SEAL.json").write_text(json.dumps(seal, indent=2) + "\n")
    print("external bundle:", EXTERNAL_ROOT)
    print("files sealed:", len(lines))
    print("EVIDENCE_SEAL_SHA256:", seal_hash)
    return seal


def verify() -> int:
    seal = json.loads((EXTERNAL_ROOT / "SEAL.json").read_text())
    actual = sha256(EXTERNAL_ROOT / "checksums.sha256")
    ok = actual == seal["EVIDENCE_SEAL_SHA256"]
    print("seal hash match:", ok)
    digest_ok = True
    for line in (EXTERNAL_ROOT / "checksums.sha256").read_text().splitlines():
        digest, rel = line.split("  ", 1)
        file_ok = sha256(EXTERNAL_ROOT / rel) == digest
        digest_ok &= file_ok
        if not file_ok:
            print("MISMATCH:", rel)
    print("all file digests match:", digest_ok)
    return 0 if (ok and digest_ok) else 1


def postcommit() -> None:
    head = git("rev-parse", "HEAD")
    tree = git("rev-parse", "HEAD^{tree}")
    origin = git("rev-parse", "origin/main")
    status = git("status", "--porcelain")
    sidecar = {
        "schema_version": "1.0.0",
        "MISSION": "RES83_IMPLEMENT_ELITE_SOCCER_HUMAN_VALID_PLANT_001",
        "LINEAR_ISSUE": "RES-83",
        "FINAL_HEAD": head,
        "FINAL_TREE": tree,
        "ORIGIN_MAIN": origin,
        "REMOTE_SYNC": head == origin,
        "TRACKED_WORKTREE_CLEAN": status == "",
        "untracked_entries": [line for line in status.splitlines() if line.startswith("??")],
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    (EXTERNAL_ROOT / "POSTCOMMIT_SIDECAR.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    print(json.dumps(sidecar, indent=2))


if __name__ == "__main__":
    if "--verify" in sys.argv:
        sys.exit(verify())
    if "--external" in sys.argv:
        seal_external()
        if "--postcommit" in sys.argv:
            postcommit()
    else:
        print(__doc__)
        sys.exit(2)
