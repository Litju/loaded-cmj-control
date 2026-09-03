#!/usr/bin/env python3
"""Seal reproduction evidence into the bundle manifest (R0.1 §2C).

Mechanical post-reproduction finalizer (no hand-entered values):
  1. reads reproduction.json + stdout/stderr from the bundle dir
  2. flips PENDING reproduction-bound claim rows to their sealed STATUS/RESULT
  3. binds REPRODUCTION_* hashes + verdicts into manifest.json, recomputing
     MANIFEST_CANONICAL_SHA256 (self-fields stripped) and MANIFEST_FILE_SHA256
  4. regenerates checksums.sha256 + FINAL_RECEIPT.md from sealed artifacts

Fails closed unless reproduction EXIT_CODE==0 and IDENTICAL==true.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.evid_canonical import canonical_sha256, file_sha256, is_valid_sha256_hex  # noqa: E402


def finalize(bundle_dir: Path) -> dict:
    bundle_dir = bundle_dir.resolve()
    repro = json.loads((bundle_dir / "reproduction.json").read_text())
    if int(repro["EXIT_CODE"]) != 0 or not bool(repro["IDENTICAL"]):
        raise RuntimeError(f"refusing to seal failed reproduction: {repro}")
    manifest = json.loads((bundle_dir / "manifest.json").read_text())

    manifest["REPRODUCTION_JSON_SHA256"] = file_sha256(bundle_dir / "reproduction.json")
    manifest["REPRODUCTION_STDOUT_SHA256"] = file_sha256(bundle_dir / "reproduction_stdout.txt")
    manifest["REPRODUCTION_STDERR_SHA256"] = file_sha256(bundle_dir / "reproduction_stderr.txt")
    manifest["REPLAY_TRACE_SHA256"] = repro["REPLAY_TRACE_SHA256"]
    manifest["REPRODUCTION_EXIT_CODE"] = int(repro["EXIT_CODE"])
    manifest["REPRODUCTION_IDENTICAL"] = bool(repro["IDENTICAL"])
    manifest["REPRODUCTION_MAX_QPOS_ERROR"] = float(repro["MAX_QPOS_ERROR"])
    manifest["REPRODUCTION_MAX_QVEL_ERROR"] = float(repro["MAX_QVEL_ERROR"])
    manifest["REPRODUCTION_EVENT_IDENTITY"] = bool(repro["ONLINE_OFFLINE_EVENT_IDENTITY"])
    manifest["REPRODUCTION_ENVIRONMENT_MATCH"] = bool(repro["ENVIRONMENT_MATCH"])
    manifest["REPRODUCTION_COMMIT_MATCH"] = bool(repro["COMMIT_MATCH"])

    # re-seal claim rows bound to reproduction
    rows: list[list[str]] = []
    with open(bundle_dir / "claim_evidence.csv") as f:
        rdr = csv.DictReader(f)
        cols = rdr.fieldnames or []
        for r in rdr:
            if r["CLAIM_ID"] in ("BRANCH_REPLAY_IDENTITY", "DETERMINISM") and repro["IDENTICAL"]:
                r["STATUS"] = "PASS"
                r["RESULT"] = "PASS"
            if r["CLAIM_ID"] == "ENVIRONMENT_MATCH" and repro["ENVIRONMENT_MATCH"]:
                r["STATUS"] = "PASS"
                r["RESULT"] = "PASS"
            if r["CLAIM_ID"] == "COMMIT_MATCH" and repro["COMMIT_MATCH"]:
                r["STATUS"] = "PASS"
                r["RESULT"] = "PASS"
            rows.append([r[c] for c in cols])
    with open(bundle_dir / "claim_evidence.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    manifest["CLAIM_EVIDENCE_SHA256"] = file_sha256(bundle_dir / "claim_evidence.csv")

    manifest["MANIFEST_CANONICAL_SHA256"] = canonical_sha256(manifest)
    # Self-reference rule: FILE hash lives outside manifest.json (checksums /
    # receipt / delivery), never inside it.
    manifest.pop("MANIFEST_FILE_SHA256", None)
    (bundle_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    file_sha = file_sha256(bundle_dir / "manifest.json")
    assert canonical_sha256(json.loads((bundle_dir / "manifest.json").read_text())) == \
        manifest["MANIFEST_CANONICAL_SHA256"]
    assert is_valid_sha256_hex(file_sha)
    assert file_sha != manifest["MANIFEST_CANONICAL_SHA256"]

    with open(bundle_dir / "checksums.sha256", "w") as cf:
        for p in sorted(bundle_dir.rglob("*")):
            if p.is_file() and p.name != "checksums.sha256":
                cf.write(f"{file_sha256(p)}  {p.relative_to(bundle_dir)}\n")

    receipt = (
        "# FINAL RECEIPT — LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION\n\n"
        f"MISSION=LCMJ_R0_1_EVIDENCE_CONTRACT_NORMALIZATION\nSTATUS=PASS\n\n"
        f"EXPERIMENT_ID={manifest['EXPERIMENT_ID']}\n"
        f"EXPERIMENT_SPEC_SHA256={manifest['EXPERIMENT_SPEC_SHA256']}\n"
        f"SPEC_EXECUTION_MATCH={manifest['SPEC_EXECUTION_MATCH']}\n\n"
        f"COMMIT_SHA={manifest['COMMIT_SHA']}\nCOMMIT_TREE={manifest['COMMIT_TREE']}\n"
        f"MUJOCO_VERSION={manifest['MUJOCO_VERSION']}\nMODEL_HASH={manifest['MODEL_HASH']}\n"
        f"TRACE_SCHEMA_VERSION={manifest['TRACE_SCHEMA_VERSION']}\n\n"
        f"BRANCH_TIME_S={manifest['BRANCH_TIME_S']:.6f}\n"
        f"STATE_SPEC={manifest['STATE_SPEC']}\nSTATE_SIZE={manifest['STATE_SIZE']}\n"
        f"STATE_VECTOR_SHA256={manifest['STATE_VECTOR_SHA256']}\n\n"
        f"SOURCE_TRACE_SHA256={manifest['SOURCE_TRACE_SHA256']}\n"
        f"REPLAY_TRACE_SHA256={manifest['REPLAY_TRACE_SHA256']}\n"
        f"CONTROL_TRACE_SHA256={manifest['CONTROL_TRACE_SHA256']}\n"
        f"MANIFEST_FILE_SHA256={file_sha}\n"
        f"MANIFEST_CANONICAL_SHA256={manifest['MANIFEST_CANONICAL_SHA256']}\n\n"
        f"REPRODUCTION_EXIT_CODE={manifest['REPRODUCTION_EXIT_CODE']}\n"
        f"REPRODUCTION_IDENTICAL={'true' if manifest['REPRODUCTION_IDENTICAL'] else 'false'}\n"
        f"MAX_QPOS_ERROR={manifest['REPRODUCTION_MAX_QPOS_ERROR']:.3e}\n"
        f"MAX_QVEL_ERROR={manifest['REPRODUCTION_MAX_QVEL_ERROR']:.3e}\n"
        f"ONLINE_OFFLINE_EVENT_IDENTITY={'PASS' if manifest['REPRODUCTION_EVENT_IDENTITY'] else 'FAIL'}\n"
        f"ENVIRONMENT_MATCH={'PASS' if manifest['REPRODUCTION_ENVIRONMENT_MATCH'] else 'FAIL'}\n"
        f"COMMIT_MATCH={'PASS' if manifest['REPRODUCTION_COMMIT_MATCH'] else 'FAIL'}\n\n"
        f"EVIDENCE_ROOT={bundle_dir}\n"
    )
    (bundle_dir / "FINAL_RECEIPT.md").write_text(receipt)
    # checksums must cover the final receipt + manifest + claim map updates
    with open(bundle_dir / "checksums.sha256", "w") as cf:
        for p in sorted(bundle_dir.rglob("*")):
            if p.is_file() and p.name != "checksums.sha256":
                cf.write(f"{file_sha256(p)}  {p.relative_to(bundle_dir)}\n")
    print(json.dumps({"sealed": True,
                      "MANIFEST_FILE_SHA256": file_sha,
                      "MANIFEST_CANONICAL_SHA256": manifest["MANIFEST_CANONICAL_SHA256"]}, indent=2))
    return manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle-dir", type=Path, required=True)
    args = ap.parse_args()
    finalize(args.bundle_dir)


if __name__ == "__main__":
    main()
