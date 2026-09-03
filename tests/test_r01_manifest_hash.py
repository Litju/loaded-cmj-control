"""R0.1: manifest file/canonical hash semantics + determinism (§5, §11)."""
import json
import sys
from pathlib import Path

TASK_ROOT = Path(__file__).resolve().parents[1]
if str(TASK_ROOT) not in sys.path:
    sys.path.insert(0, str(TASK_ROOT))

from tools.evid_canonical import (
    SELF_HASH_FIELDS,
    canonical_bytes,
    canonical_sha256,
    is_valid_sha256_hex,
)


def test_self_hash_fields_frozen():
    assert SELF_HASH_FIELDS == frozenset({"MANIFEST_FILE_SHA256", "MANIFEST_CANONICAL_SHA256"})


def test_canonical_determinism_identical_logical_manifests():
    a = {"b": 1, "a": [1.5, 2.0], "nested": {"z": "x", "y": 0.1}}
    # same logical content, different key order
    b = {"nested": {"y": 0.1, "z": "x"}, "a": [1.5, 2.0], "b": 1}
    assert canonical_bytes(a) == canonical_bytes(b)
    assert canonical_sha256(a) == canonical_sha256(b)
    assert is_valid_sha256_hex(canonical_sha256(a))


def test_canonical_strips_only_self_hashes():
    base = {"k": "v", "n": 3}
    m1 = dict(base, MANIFEST_FILE_SHA256="a" * 64, MANIFEST_CANONICAL_SHA256="b" * 64)
    m2 = dict(base, MANIFEST_FILE_SHA256="c" * 64, MANIFEST_CANONICAL_SHA256="d" * 64)
    assert canonical_sha256(m1) == canonical_sha256(m2) == canonical_sha256(base)
    # non-self fields DO change the hash
    m3 = dict(base, extra=1)
    assert canonical_sha256(m3) != canonical_sha256(base)


def test_canonical_algorithm_exactness():
    obj = {"a": 1, "b": [1, 2]}
    raw = canonical_bytes(obj)
    # deterministic separators, sorted keys, utf-8, no trailing newline
    assert raw == b'{"a":1,"b":[1,2]}'
    assert not raw.endswith(b"\n")


def test_no_ambiguous_manifest_sha_field():
    # New manifests must not use the ambiguous legacy 'manifest_sha256' name.
    import tools.evid_bundle as eb  # noqa: F401
    import inspect

    src = inspect.getsource(eb.run_bundle)
    assert "manifest_sha256" not in src.replace("MANIFEST_FILE_SHA256", "").replace(
        "MANIFEST_CANONICAL_SHA256", "").replace("EXPERIMENT_SPEC_SHA256", "").replace(
        "STATE_VECTOR_SHA256", "").replace("SOURCE_TRACE_SHA256", "")
