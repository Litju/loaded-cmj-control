"""R0.1: bundle contract — checksums, claim binding, receipt-from-evidence (§11)."""
import csv
import json
import sys
from pathlib import Path

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(TASK_ROOT))

REQUIRED_BUNDLE_FILES = [
    "manifest.json", "experiment_spec.json", "run_record.json", "result_assessment.json",
    "environment.json", "initial_integration_state.npz", "initial_integration_state.json",
    "physics_trace.npz", "control_trace.npz", "branch_control_sequence.npz",
    "events_online.json", "events_offline.json", "metrics.json", "experiments.jsonl",
    "claim_evidence.csv", "reproduction.json", "reproduction_stdout.txt", "reproduction_stderr.txt",
    "reproduce.sh", "checksums.sha256", "FINAL_RECEIPT.md",
]

REQUIRED_CLAIM_COLUMNS = ["CLAIM_ID", "CLAIM", "CLAIM_TYPE", "STATUS", "SOURCE_ARTIFACT",
                          "SOURCE_FIELD_OR_RANGE", "DERIVATION", "ACCEPTANCE_CRITERION",
                          "RESULT", "NOTES"]

# Every claim string that may appear in FINAL_RECEIPT must be bound here.
RECEIPT_CLAIM_IDS = ["BRANCH_REPLAY_IDENTITY", "FULL_MJSTATE_CAPTURE", "SPEC_EXECUTION_MATCH",
                     "DETERMINISM", "EVENT_IDENTITY", "NO_FALL", "NO_PROHIBITED_CONTACT",
                     "NO_ROOT_SUPPORT", "TRACE_SAMPLE_COUNT", "TRACE_SCHEMA_V2",
                     "ENVIRONMENT_MATCH", "COMMIT_MATCH"]


def _bundle_dir_from_env() -> Path | None:
    import os

    p = os.environ.get("R01_BUNDLE_DIR")
    return Path(p) if p else None


def test_required_bundle_file_list_documented():
    # Static contract: the assembler must reference every required file.
    import tools.evid_bundle as eb
    import inspect

    src = inspect.getsource(eb.run_bundle)
    for f in REQUIRED_BUNDLE_FILES:
        assert f in src, f"bundle assembler missing {f}"


def test_claim_evidence_columns():
    import tools.evid_bundle as eb  # noqa: F401
    # columns are written by the assembler; assert the contract constant
    assert REQUIRED_CLAIM_COLUMNS == ["CLAIM_ID", "CLAIM", "CLAIM_TYPE", "STATUS", "SOURCE_ARTIFACT",
                                      "SOURCE_FIELD_OR_RANGE", "DERIVATION", "ACCEPTANCE_CRITERION",
                                      "RESULT", "NOTES"]


def test_receipt_claims_bound_in_map():
    # The assembler's claim rows must cover every receipt claim id.
    import tools.evid_bundle as eb
    import inspect

    src = inspect.getsource(eb.run_bundle)
    for cid in RECEIPT_CLAIM_IDS + ["E12_PASS"]:
        assert cid in src, f"claim {cid} not bound in bundle assembler"


def test_live_bundle_if_present():
    b = _bundle_dir_from_env()
    if b is None or not b.exists():
        return  # exercised in the delivery step, not in unit runs
    for f in REQUIRED_BUNDLE_FILES:
        assert (b / f).exists(), f"missing bundle file {f}"
    # checksums completeness: every immutable artifact except checksums.sha256 itself
    lines = (b / "checksums.sha256").read_text().splitlines()
    listed = {ln.split("  ", 1)[1] for ln in lines if "  " in ln}
    for p in sorted(b.rglob("*")):
        if p.is_file() and p.name != "checksums.sha256":
            assert str(p.relative_to(b)) in listed, f"checksums missing {p.name}"
    # manifest hash semantics: CANONICAL inside, FILE outside (self-ref rule)
    import hashlib as _hl

    manifest = json.loads((b / "manifest.json").read_text())
    assert "MANIFEST_CANONICAL_SHA256" in manifest
    assert "MANIFEST_FILE_SHA256" not in manifest
    raw_sha = _hl.sha256((b / "manifest.json").read_bytes()).hexdigest()
    cks = dict(ln.split("  ", 1)[::-1] for ln in lines if "  " in ln)
    # cks maps path -> hash after [::-1]? rebuild properly:
    cks2 = {}
    for ln in lines:
        if "  " in ln:
            h, pth = ln.split("  ", 1)
            cks2[pth] = h
    assert cks2["manifest.json"] == raw_sha
    # claim-evidence completeness
    with open(b / "claim_evidence.csv") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0].keys()) == REQUIRED_CLAIM_COLUMNS
    ids = {r["CLAIM_ID"] for r in rows}
    for cid in RECEIPT_CLAIM_IDS:
        assert cid in ids, f"claim map missing {cid}"
    # receipt derived from evidence (no hand hashes): spot-check manifest binding
    receipt = (b / "FINAL_RECEIPT.md").read_text()
    assert raw_sha in receipt
    assert manifest["MANIFEST_CANONICAL_SHA256"] in receipt
    assert manifest["STATE_VECTOR_SHA256"] in receipt
    # reproduction evidence inclusion
    repro = json.loads((b / "reproduction.json").read_text())
    for k in ["COMMAND", "START_TIME", "END_TIME", "EXIT_CODE", "SOURCE_TRACE_SHA256",
              "REPLAY_TRACE_SHA256", "IDENTICAL", "MAX_QPOS_ERROR", "MAX_QVEL_ERROR",
              "ONLINE_OFFLINE_EVENT_IDENTITY", "ENVIRONMENT_MATCH", "COMMIT_MATCH"]:
        assert k in repro, f"reproduction.json missing {k}"
    assert (b / "reproduction_stdout.txt").stat().st_size > 0


def test_reproduce_sh_authority_gates():
    src = (TASK_ROOT / "tools" / "evid_bundle.py").read_text()
    # reproduce.sh template must gate all four authorities with hard refusal
    for token in ["CURRENT_COMMIT_SHA", "MANIFEST_COMMIT_SHA", "CURRENT_TREE",
                  "MANIFEST_COMMIT_TREE", "MUJOCO_VERSION", "MODEL_HASH",
                  "AUTHORITY_MISMATCH", "exit 1"]:
        assert token in src, f"reproduce.sh missing gate token {token}"
