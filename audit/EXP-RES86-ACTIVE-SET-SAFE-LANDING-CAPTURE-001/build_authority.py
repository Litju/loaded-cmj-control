#!/usr/bin/env python3
"""Generate the tracked RES-86 landing authority artifacts.

MISSION: RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001

Writes (mechanically, from ``src/loaded_cmj/v3/landing_authority.py``):

* ``V3_LANDING_ACCEPTANCE_AUTHORITY.json`` - the compact machine-readable
  V3 landing authority implementing the downstream RES-82 decision record;
* ``V3_LANDING_ACCEPTANCE_AUTHORITY_DIGEST.json`` - canonical digest binding;
* ``checksums.sha256`` - file hashes for this audit directory.

Run:  .venv/bin/python audit/EXP-RES86-.../build_authority.py
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from loaded_cmj.v3.landing_authority import (  # noqa: E402
    authority_canonical_bytes,
    authority_sha256,
    landing_acceptance_authority,
    validate_authority,
)

AUDIT_DIR = Path(__file__).resolve().parent
AUTHORITY_PATH = AUDIT_DIR / "V3_LANDING_ACCEPTANCE_AUTHORITY.json"
DIGEST_PATH = AUDIT_DIR / "V3_LANDING_ACCEPTANCE_AUTHORITY_DIGEST.json"
MODULE_PATH = SRC / "loaded_cmj" / "v3" / "landing_authority.py"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    authority = landing_acceptance_authority()
    failures = validate_authority(authority)
    if failures:
        raise SystemExit("invalid authority: " + "; ".join(failures))
    AUDIT_DIR.mkdir(parents=True, exist_ok=True)
    AUTHORITY_PATH.write_bytes(authority_canonical_bytes(authority) + b"\n")
    on_disk = json.loads(AUTHORITY_PATH.read_text())
    if authority_sha256(on_disk) != authority_sha256(authority):
        raise SystemExit("canonical digest unstable across serialization")
    digest = {
        "authority_id": authority["authority_id"],
        "authority_version": authority["authority_version"],
        "canonical_sha256": authority_sha256(authority),
        "source_module": str(MODULE_PATH.relative_to(ROOT)),
        "source_module_sha256": sha256_file(MODULE_PATH),
        "classification": authority["classification"]["class"],
        "false_closure_prohibition": authority["handoff"]["false_closure_prohibition"],
    }
    DIGEST_PATH.write_text(json.dumps(digest, indent=2, sort_keys=True) + "\n")
    checksum_path = AUDIT_DIR / "checksums.sha256"
    files = sorted(p for p in AUDIT_DIR.rglob("*")
                   if p.is_file() and p.name != "checksums.sha256")
    checksum_path.write_text("".join(
        f"{sha256_file(path)}  {path.relative_to(AUDIT_DIR)}\n" for path in files))
    print(json.dumps(digest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
