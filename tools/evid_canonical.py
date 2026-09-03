#!/usr/bin/env python3
"""Canonical JSON + manifest hash semantics (R0.1 §5).

Defines the single frozen canonical serialization algorithm and the two
explicit manifest hashes:

  MANIFEST_FILE_SHA256      = SHA256(raw manifest.json bytes on disk)
  MANIFEST_CANONICAL_SHA256 = SHA256(canonical bytes of manifest content with
                              only the documented self-hash fields removed)

Self-hash removal set (frozen):
  {"MANIFEST_FILE_SHA256", "MANIFEST_CANONICAL_SHA256"}

Self-reference rule (frozen, avoids impossible fixpoint):
  manifest.json stores MANIFEST_CANONICAL_SHA256 inside itself (verifiable by
  stripping both self fields and re-canonicalizing). manifest.json does NOT
  store MANIFEST_FILE_SHA256 inside itself, because no file can contain its
  own raw-byte hash. MANIFEST_FILE_SHA256 is defined as SHA256(raw
  manifest.json bytes) and is recorded EXTERNALLY in checksums.sha256 (the
  manifest.json line), FINAL_RECEIPT.md, delivery fields, and the sealing
  commit message. Both values are mechanically generated.

Canonical algorithm (frozen):
  - UTF-8 encoding
  - json.dumps(obj, sort_keys=True, separators=(",", ":"),
               ensure_ascii=False) with no trailing newline in the hashed
    bytes (file on disk MAY end with exactly one "\\n"; the canonical hash
    never includes it)
  - deterministic key ordering via sort_keys
  - deterministic separators ("," and ":")
  - floats serialized by CPython repr (deterministic for float64)
  - no nondeterministic timestamps inside canonicalized content unless the
    timestamp is part of the intended manifest content (timestamps that are
    content hash identically; wall-clock "recorded_at" lives in
    environment.json, never in the canonical manifest core)

Both values are mechanically generated; hand-entered values are rejected by
tests (hex format + recomputation checks).
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

SELF_HASH_FIELDS = frozenset({"MANIFEST_FILE_SHA256", "MANIFEST_CANONICAL_SHA256"})


def canonical_bytes(obj: dict) -> bytes:
    """Frozen canonical serialization → UTF-8 bytes (no trailing newline)."""
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")


def canonical_sha256(obj: dict, strip_self_hashes: bool = True) -> str:
    """SHA256 of canonical bytes, optionally stripping self-hash fields."""
    if strip_self_hashes:
        obj = {k: v for k, v in obj.items() if k not in SELF_HASH_FIELDS}
    return hashlib.sha256(canonical_bytes(obj)).hexdigest()


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def manifest_hashes(manifest_path: Path) -> tuple[str, str]:
    """Return (MANIFEST_FILE_SHA256, MANIFEST_CANONICAL_SHA256) for a file."""
    raw = Path(manifest_path).read_bytes()
    file_sha = hashlib.sha256(raw).hexdigest()
    obj = json.loads(raw.decode("utf-8"))
    canon_sha = canonical_sha256(obj, strip_self_hashes=True)
    return file_sha, canon_sha


def is_valid_sha256_hex(s: str) -> bool:
    if not isinstance(s, str) or len(s) != 64:
        return False
    try:
        int(s, 16)
    except ValueError:
        return False
    return s == s.lower() and all(c in "0123456789abcdef" for c in s)
