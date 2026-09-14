#!/usr/bin/env python3
"""Seal the RES-95 bundle: in-repo hash manifest and external evidence root.

Modes:
  python3 seal_bundle.py            # write MODEL_AUTHORITY_HASH_MANIFEST.json in place
  python3 seal_bundle.py --external # copy bundle to the external evidence root and seal it

The external seal follows the RES-82 convention:
  checksums.sha256 covers every copied file except checksums.sha256, SEAL.json and POSTCOMMIT_SIDECAR*;
  EVIDENCE_SEAL_SHA256 = sha256(checksums.sha256 bytes).
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

EXCLUDE_MANIFEST = {"MODEL_AUTHORITY_HASH_MANIFEST.json", "AUTHORITY_VALIDATION_REPORT.json",
                    "checksums.sha256", "SEAL.json"}
EXCLUDE_PREFIX = ("POSTCOMMIT_SIDECAR",)


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def bundle_files():
    files = []
    for p in sorted(HERE.rglob("*")):
        if not p.is_file():
            continue
        if p.name in EXCLUDE_MANIFEST or p.name.startswith(EXCLUDE_PREFIX):
            continue
        if "__pycache__" in p.parts:
            continue
        files.append(p)
    return files


def write_manifest():
    files = bundle_files()
    entry_head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
                                text=True).stdout.strip()
    entry_tree = subprocess.run(["git", "rev-parse", "HEAD^{tree}"], cwd=REPO, capture_output=True,
                                text=True).stdout.strip()
    manifest = {
        "schema_version": "1.0.0",
        "artifact": "MODEL_AUTHORITY_HASH_MANIFEST",
        "authority_id": "LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1",
        "mission": "RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001",
        "linear_issue": "RES-95",
        "status": "FROZEN_FOR_RES83_IMPLEMENTATION",
        "entry_head": entry_head,
        "entry_tree": entry_tree,
        "hash_algorithm": "sha256",
        "excluded_from_manifest": sorted(EXCLUDE_MANIFEST),
        "files": [{"path": str(p.relative_to(HERE)), "sha256": sha256(p),
                   "bytes": p.stat().st_size} for p in files],
        "sealed_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    (HERE / "MODEL_AUTHORITY_HASH_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"manifest written: {len(manifest['files'])} files")
    print("entry HEAD:", entry_head)
    print("entry TREE:", entry_tree)


def seal_external():
    if EXTERNAL_ROOT.exists():
        for child in EXTERNAL_ROOT.iterdir():
            if child.is_file():
                child.unlink()
    else:
        EXTERNAL_ROOT.mkdir(parents=True)
    copied = []
    for p in sorted(HERE.rglob("*")):
        if not p.is_file() or "__pycache__" in p.parts:
            continue
        if p.name in {"checksums.sha256", "SEAL.json"} or p.name.startswith(EXCLUDE_PREFIX):
            continue
        target = EXTERNAL_ROOT / p.relative_to(HERE)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)
        copied.append(target)
    lines = []
    for t in sorted(copied):
        if t.name == "checksums.sha256" or t.name == "SEAL.json" or t.name.startswith(EXCLUDE_PREFIX):
            continue
        lines.append(f"{sha256(t)}  {t.relative_to(EXTERNAL_ROOT)}")
    (EXTERNAL_ROOT / "checksums.sha256").write_text("\n".join(lines) + "\n")
    seal_hash = sha256(EXTERNAL_ROOT / "checksums.sha256")
    seal = {
        "schema_version": "1.0.0",
        "MISSION": "RES95_CLOSE_CORRECTED_MODEL_AUTHORITY_AND_SEAL_REPOSITORY_BUNDLE_001",
        "LINEAR_ISSUE": "RES-95",
        "ACHIEVEMENT": "elite-soccer successor Plant model authority sealed",
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


if __name__ == "__main__":
    if "--external" in sys.argv:
        seal_external()
    else:
        write_manifest()
