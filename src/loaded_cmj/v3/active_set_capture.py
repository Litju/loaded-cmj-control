"""V3 lossless contact / EFC active-set capture (RES-86A).

Authority: ``LCMJ_RES86_V3_ACTIVE_SET_CAPTURE_V1``.

The RES-85E fixed-pad trace (16 contact columns, 128 EFC columns) is forbidden
for RES-86 qualification: the observed predecessor maxima are ``ncon = 24``
and ``nefc = 144``.  This module records the runtime contact and constraint
buffers *losslessly* with a variable-length CSR/offset encoding:

* ``contact_offsets[N + 1]`` and flat per-row contact arrays including the
  decoded RES-84 classification / legal-support role;
* ``efc_offsets[N + 1]`` and flat per-row constraint arrays including the row
  Jacobian ``efc_J`` so derivative evidence can be reconstructed exactly;
* for every sample ``diff(contact_offsets) == ncon`` and
  ``diff(efc_offsets) == nefc``.

No truncation, no silent overflow, no fixed arbitrary cap.  MuJoCo 3.8
multi-point (multi-CCD style) contacts are handled naturally: every
``mjContact`` row carries its own ``efc_address``/``dim`` and the per-contact
EFC row span is derived and validated from the runtime buffers.  The recorder
fails closed (``V3ActiveSetCaptureError``) on any row-count, coverage or
finiteness inconsistency.

The active-set signature canonicalises the row ordering before hashing, so two
equivalent active sets recorded in different buffer order produce the same
signature, while any material change to a contact, a prohibited contact, a
constraint row type/id/state/force or a Jacobian row changes it.

Two identities are deliberately separated:

``RES86_ACTIVE_SET_SIGNATURE_V1`` (exact numerical evidence fingerprint)
    Backward-compatible exact evidence identity.  It hashes continuously
    varying quantities (contact distance/position/frame/force, EFC
    pos/vel/force/margin/KBIP/D/aref/b and the Jacobian rows) together with
    the sample/interval identity.  It answers "is this the same numerical
    evidence?", never "is this the same contact mode?".

``RES86_CONTACT_MODE_SIGNATURE_V1`` (contact mode identity)
    Discrete contact/constraint mode identity for local derivative
    eligibility.  It encodes only discrete structure: the normalized contact
    geom pair, contact class, side, plantar region, contact dimension, the
    active/legal-plantar/prohibited booleans, the per-side legal support and
    prohibited-present support descriptor, and the EFC type/state with a
    stable semantic constraint identity (canonical semantic contact identity
    for contact constraints, never the raw contact-buffer index) plus row
    multiplicities.  Sample index, time, branch/interval identity, distances,
    positions, frames, forces and every numerical Jacobian/EFC value are
    excluded.

Frozen future derivative rule (declared here, not implemented in RES-86A1;
RES-86B owns deterministic epsilon shrink/refinement):

* a CENTRAL difference is allowed only when
  ``MODE(nominal) == MODE(+epsilon) == MODE(-epsilon)``;
* when only one side matches the nominal mode, CENTRAL is FORBIDDEN and the
  one-sided same-mode difference is the candidate;
* when neither side matches, the derivative is
  ``UNAVAILABLE_AT_CURRENT_EPSILON``;
* incompatible modes are never averaged.
"""

from __future__ import annotations

import hashlib
import struct
from collections import Counter
from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import measurement as M
from loaded_cmj.v3.plant import model_xml

V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID = "LCMJ_RES86_V3_ACTIVE_SET_CAPTURE_V1"
V3_ACTIVE_SET_SIGNATURE_VERSION = "RES86_ACTIVE_SET_SIGNATURE_V1"
V3_EXACT_EVIDENCE_FINGERPRINT_VERSION = V3_ACTIVE_SET_SIGNATURE_VERSION
V3_CONTACT_MODE_SIGNATURE_VERSION = "RES86_CONTACT_MODE_SIGNATURE_V1"

# Frozen future derivative rule (RES-86B implements the epsilon refinement).
DERIVATIVE_CENTRAL_ALLOWED = "CENTRAL_ALLOWED_SAME_MODE_BOTH_SIDES"
DERIVATIVE_ONE_SIDED_SAME_MODE = "ONE_SIDED_SAME_MODE_CANDIDATE"
DERIVATIVE_UNAVAILABLE = "DERIVATIVE_UNAVAILABLE_AT_CURRENT_EPSILON"


def derivative_eligibility(mode_nominal: str, mode_plus: str, mode_minus: str) -> str:
    """Frozen mode-compatibility rule for a local derivative candidate.

    Returns one of :data:`DERIVATIVE_CENTRAL_ALLOWED`,
    :data:`DERIVATIVE_ONE_SIDED_SAME_MODE` or :data:`DERIVATIVE_UNAVAILABLE`.
    A central difference is never taken across incompatible contact modes.
    """
    plus_same = mode_plus == mode_nominal
    minus_same = mode_minus == mode_nominal
    if plus_same and minus_same:
        return DERIVATIVE_CENTRAL_ALLOWED
    if plus_same or minus_same:
        return DERIVATIVE_ONE_SIDED_SAME_MODE
    return DERIVATIVE_UNAVAILABLE

CONTACT_CONSTRAINT_TYPES: tuple[int, ...] = (
    int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS),
    int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL),
    int(mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC),
)

_CONTACT_CLASS_CODE = {
    M.V3ContactClass.LEGAL_PLANTAR_FLOOR: 0,
    M.V3ContactClass.PROHIBITED_FLOOR: 1,
    M.V3ContactClass.OTHER_FLOOR: 2,
    M.V3ContactClass.SELF: 3,
}
_SIDE_CODE = {"left": 0, "right": 1, None: -1}
_REGION_CODE = {"heel": 0, "forefoot": 1, "toe": 2, None: -1}


class V3ActiveSetCaptureError(RuntimeError):
    """Explicit fail-closed recorder state; never a silent truncation."""


def contact_class_code(record: M.V3ContactRecord) -> int:
    return int(_CONTACT_CLASS_CODE[record.contact_class])


@dataclass(frozen=True)
class V3ActiveSetTables:
    """Lossless variable-length active-set capture over a native stream.

    Every flat array is indexed by the CSR offsets; ``contact_offsets[s]`` is
    the first row of sample ``s`` and ``contact_offsets[s + 1]`` one past its
    last row (and likewise for the EFC tables).
    """

    sample_index: np.ndarray
    time_s: np.ndarray
    ncon: np.ndarray
    nefc: np.ndarray

    contact_offsets: np.ndarray
    contact_id: np.ndarray
    contact_geom0_id: np.ndarray
    contact_geom1_id: np.ndarray
    contact_class: np.ndarray
    contact_side: np.ndarray
    contact_region: np.ndarray
    contact_dist: np.ndarray
    contact_pos: np.ndarray
    contact_frame: np.ndarray
    contact_efc_address: np.ndarray
    contact_dim: np.ndarray
    contact_efc_nrows: np.ndarray
    contact_force6: np.ndarray
    contact_normal_force: np.ndarray
    contact_active: np.ndarray
    contact_legal_active: np.ndarray
    contact_prohibited: np.ndarray

    efc_offsets: np.ndarray
    efc_type: np.ndarray
    efc_id: np.ndarray
    efc_state: np.ndarray
    efc_pos: np.ndarray
    efc_vel: np.ndarray
    efc_force: np.ndarray
    efc_margin: np.ndarray
    efc_aref: np.ndarray
    efc_b: np.ndarray
    efc_d: np.ndarray
    efc_kbip: np.ndarray
    efc_frictionloss: np.ndarray
    efc_jacobian: np.ndarray

    model_sha256: str
    nq: int
    nv: int
    nu: int
    na: int
    authority_id: str = V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID

    # ------------------------------------------------------------------
    # counts / slicing
    # ------------------------------------------------------------------
    @property
    def sample_count(self) -> int:
        return int(self.sample_index.shape[0])

    def contact_count(self, sample: int) -> int:
        return int(self.ncon[sample])

    def efc_count(self, sample: int) -> int:
        return int(self.nefc[sample])

    def contact_rows(self, sample: int) -> dict[str, np.ndarray]:
        lo, hi = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
        return {
            "contact_id": self.contact_id[lo:hi],
            "geom0_id": self.contact_geom0_id[lo:hi],
            "geom1_id": self.contact_geom1_id[lo:hi],
            "class": self.contact_class[lo:hi],
            "side": self.contact_side[lo:hi],
            "region": self.contact_region[lo:hi],
            "dist": self.contact_dist[lo:hi],
            "pos": self.contact_pos[lo:hi],
            "frame": self.contact_frame[lo:hi],
            "efc_address": self.contact_efc_address[lo:hi],
            "dim": self.contact_dim[lo:hi],
            "efc_nrows": self.contact_efc_nrows[lo:hi],
            "force6": self.contact_force6[lo:hi],
            "normal_force": self.contact_normal_force[lo:hi],
            "active": self.contact_active[lo:hi],
            "legal_active": self.contact_legal_active[lo:hi],
            "prohibited": self.contact_prohibited[lo:hi],
        }

    def efc_rows(self, sample: int) -> dict[str, np.ndarray]:
        lo, hi = int(self.efc_offsets[sample]), int(self.efc_offsets[sample + 1])
        return {
            "type": self.efc_type[lo:hi],
            "id": self.efc_id[lo:hi],
            "state": self.efc_state[lo:hi],
            "pos": self.efc_pos[lo:hi],
            "vel": self.efc_vel[lo:hi],
            "force": self.efc_force[lo:hi],
            "margin": self.efc_margin[lo:hi],
            "aref": self.efc_aref[lo:hi],
            "b": self.efc_b[lo:hi],
            "d": self.efc_d[lo:hi],
            "kbip": self.efc_kbip[lo:hi],
            "frictionloss": self.efc_frictionloss[lo:hi],
            "jacobian": self.efc_jacobian[lo:hi],
        }

    # ------------------------------------------------------------------
    # validation
    # ------------------------------------------------------------------
    def validate(self) -> list[str]:
        """Fail-closed structural validation; returns an empty list when valid."""
        failures: list[str] = []
        n = self.sample_count
        if n != int(self.time_s.shape[0]) or n != int(self.ncon.shape[0]) or n != int(self.nefc.shape[0]):
            failures.append("SAMPLE_ARRAY_LENGTH_MISMATCH")
            return failures
        for label, offsets, counts, flat_len in (
            ("contact", self.contact_offsets, self.ncon, int(self.contact_id.shape[0])),
            ("efc", self.efc_offsets, self.nefc, int(self.efc_type.shape[0])),
        ):
            if offsets.shape[0] != n + 1:
                failures.append(f"{label.upper()}_OFFSETS_LENGTH_NOT_N_PLUS_1")
                continue
            if int(offsets[0]) != 0:
                failures.append(f"{label.upper()}_OFFSETS_DO_NOT_START_AT_ZERO")
            if int(offsets[-1]) != flat_len:
                failures.append(f"{label.upper()}_OFFSETS_END_DOES_NOT_MATCH_FLAT_LENGTH")
            if np.any(np.diff(offsets) < 0):
                failures.append(f"{label.upper()}_OFFSETS_NOT_MONOTONE")
            if not np.array_equal(np.diff(offsets), counts):
                failures.append(f"{label.upper()}_OFFSET_DIFF_DOES_NOT_EQUAL_COUNT")
        if failures:
            return failures
        contact_len = int(self.contact_offsets[-1])
        efc_len = int(self.efc_offsets[-1])
        for name in ("contact_id", "contact_geom0_id", "contact_geom1_id", "contact_class",
                     "contact_side", "contact_region", "contact_dist", "contact_pos",
                     "contact_frame", "contact_efc_address", "contact_dim", "contact_efc_nrows",
                     "contact_force6", "contact_normal_force", "contact_active",
                     "contact_legal_active", "contact_prohibited"):
            if int(getattr(self, name).shape[0]) != contact_len:
                failures.append(f"CONTACT_ARRAY_LENGTH_MISMATCH:{name}")
        for name in ("efc_type", "efc_id", "efc_state", "efc_pos", "efc_vel", "efc_force",
                     "efc_margin", "efc_aref", "efc_b", "efc_d", "efc_kbip",
                     "efc_frictionloss", "efc_jacobian"):
            if int(getattr(self, name).shape[0]) != efc_len:
                failures.append(f"EFC_ARRAY_LENGTH_MISMATCH:{name}")
        if failures:
            return failures
        # every contact-type EFC row is covered exactly once by the contact spans
        covered = np.zeros(int(self.efc_type.shape[0]), dtype=bool)
        for sample in range(n):
            lo, hi = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
            base = int(self.efc_offsets[sample])
            sample_rows = int(self.efc_count(sample))
            contact_rows = int(np.count_nonzero(
                np.isin(self.efc_type[base:base + sample_rows], CONTACT_CONSTRAINT_TYPES)))
            rows = 0
            for row in range(lo, hi):
                address = int(self.contact_efc_address[row])
                nrows = int(self.contact_efc_nrows[row])
                if address < 0:
                    if nrows != 0:
                        failures.append("NEGATIVE_EFC_ADDRESS_WITH_NONZERO_ROW_COUNT")
                    continue
                if address + nrows > sample_rows:
                    failures.append("CONTACT_EFC_SPAN_OUT_OF_RANGE")
                    continue
                if covered[base + address:base + address + nrows].any():
                    failures.append("CONTACT_EFC_SPANS_OVERLAP")
                covered[base + address:base + address + nrows] = True
                rows += nrows
            if rows != contact_rows:
                failures.append("CONTACT_EFC_ROW_TOTAL_DOES_NOT_EQUAL_CONTACT_ROW_COUNT")
        is_contact = np.isin(self.efc_type, CONTACT_CONSTRAINT_TYPES)
        if not np.array_equal(covered, is_contact):
            failures.append("CONTACT_TYPE_ROW_COVERAGE_MISMATCH")
        return sorted(set(failures))

    def require_valid(self) -> None:
        failures = self.validate()
        if failures:
            raise V3ActiveSetCaptureError("; ".join(failures))

    def canonical_digest(self) -> str:
        digest = hashlib.sha256()
        for name in sorted(self.__dataclass_fields__):
            value = getattr(self, name)
            digest.update(name.encode("utf-8"))
            if isinstance(value, np.ndarray):
                digest.update(str(value.dtype).encode("utf-8"))
                digest.update(str(value.shape).encode("utf-8"))
                digest.update(np.ascontiguousarray(value).tobytes())
            else:
                digest.update(repr(value).encode("utf-8"))
        return digest.hexdigest()

    # ------------------------------------------------------------------
    # deterministic active-set signature
    # ------------------------------------------------------------------
    def _contact_order_key(self, row: int) -> tuple:
        return (
            int(self.contact_geom0_id[row]),
            int(self.contact_geom1_id[row]),
            int(self.contact_class[row]),
            int(self.contact_side[row]),
            int(self.contact_region[row]),
            self.contact_dist[row].tobytes(),
            self.contact_pos[row].tobytes(),
            self.contact_id[row],
        )

    def _efc_order_key(self, row: int) -> tuple:
        return (
            int(self.efc_type[row]),
            int(self.efc_id[row]),
            int(self.efc_state[row]),
            self.efc_pos[row].tobytes(),
            self.efc_vel[row].tobytes(),
            self.efc_force[row].tobytes(),
        )

    def sample_signature(self, sample: int) -> str:
        """Canonical per-sample active-set signature (hex sha256)."""
        lo_c, hi_c = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
        lo_e, hi_e = int(self.efc_offsets[sample]), int(self.efc_offsets[sample + 1])
        digest = hashlib.sha256()
        digest.update(V3_ACTIVE_SET_SIGNATURE_VERSION.encode("utf-8"))
        digest.update(struct.pack("<qq", int(self.sample_index[sample]), int(sample)))
        digest.update(struct.pack("<d", float(self.time_s[sample])))
        digest.update(b"|LEGAL|")
        for row in range(lo_c, hi_c):
            if int(self.contact_legal_active[row]):
                digest.update(struct.pack("<q", int(self.contact_id[row])))
                digest.update(struct.pack("<i", int(self.contact_geom0_id[row])))
                digest.update(struct.pack("<i", int(self.contact_geom1_id[row])))
                digest.update(struct.pack("<i", int(self.contact_side[row])))
        digest.update(b"|PROHIBITED|")
        for row in range(lo_c, hi_c):
            if int(self.contact_prohibited[row]):
                digest.update(struct.pack("<q", int(self.contact_id[row])))
                digest.update(struct.pack("<i", int(self.contact_geom0_id[row])))
                digest.update(struct.pack("<i", int(self.contact_geom1_id[row])))
        digest.update(b"|SUPPORT_MODE|")
        for side_code in (0, 1):
            active = any(int(self.contact_legal_active[row]) and int(self.contact_side[row]) == side_code
                         for row in range(lo_c, hi_c))
            digest.update(b"1" if active else b"0")
        digest.update(b"|CONTACTS|")
        for ordinal, row in enumerate(sorted(range(lo_c, hi_c), key=self._contact_order_key)):
            digest.update(struct.pack("<q", ordinal))
            for array in (self.contact_id, self.contact_geom0_id, self.contact_geom1_id,
                          self.contact_class, self.contact_side, self.contact_region):
                digest.update(np.ascontiguousarray(array[row:row + 1]).tobytes())
            digest.update(self.contact_dist[row].tobytes())
            digest.update(self.contact_pos[row].tobytes())
            digest.update(self.contact_frame[row].tobytes())
            digest.update(np.ascontiguousarray(self.contact_efc_address[row:row + 1]).tobytes())
            digest.update(np.ascontiguousarray(self.contact_dim[row:row + 1]).tobytes())
            digest.update(np.ascontiguousarray(self.contact_efc_nrows[row:row + 1]).tobytes())
            digest.update(self.contact_force6[row].tobytes())
            digest.update(self.contact_normal_force[row].tobytes())
            for array in (self.contact_active, self.contact_legal_active, self.contact_prohibited):
                digest.update(np.ascontiguousarray(array[row:row + 1]).tobytes())
        digest.update(b"|EFC|")
        for ordinal, row in enumerate(sorted(range(lo_e, hi_e), key=self._efc_order_key)):
            digest.update(struct.pack("<q", ordinal))
            for array in (self.efc_type, self.efc_id, self.efc_state):
                digest.update(np.ascontiguousarray(array[row:row + 1]).tobytes())
            for array in (self.efc_pos, self.efc_vel, self.efc_force, self.efc_margin,
                          self.efc_aref, self.efc_b, self.efc_d, self.efc_kbip,
                          self.efc_frictionloss):
                digest.update(array[row].tobytes())
            digest.update(self.efc_jacobian[row].tobytes())
        return digest.hexdigest()

    def branch_signature(self, start_sample: int, end_sample: int, *,
                         branch_id: str, executed_interval_id: str) -> str:
        """Exact numerical evidence fingerprint over ``[start, end]``.

        ``RES86_ACTIVE_SET_SIGNATURE_V1`` (backward-compatible).  Includes the
        sample domain identity (branch id, executed interval id, start/end
        sample and time, authority/model identity) plus every native sample's
        canonical active-set signature, so changing one contact, one
        constraint row or the executed interval changes the fingerprint.
        """
        if not 0 <= start_sample <= end_sample < self.sample_count:
            raise V3ActiveSetCaptureError("signature interval outside the captured stream")
        digest = hashlib.sha256()
        digest.update(V3_ACTIVE_SET_SIGNATURE_VERSION.encode("utf-8"))
        digest.update(self.authority_id.encode("utf-8"))
        digest.update(self.model_sha256.encode("utf-8"))
        digest.update(struct.pack("<q", int(self.nq)))
        digest.update(struct.pack("<q", int(self.nv)))
        digest.update(struct.pack("<q", int(self.nu)))
        digest.update(struct.pack("<q", int(self.na)))
        digest.update(branch_id.encode("utf-8"))
        digest.update(executed_interval_id.encode("utf-8"))
        digest.update(struct.pack("<qq", int(start_sample), int(end_sample)))
        digest.update(struct.pack("<dd", float(self.time_s[start_sample]), float(self.time_s[end_sample])))
        digest.update(struct.pack("<q", int(end_sample - start_sample + 1)))
        for sample in range(int(start_sample), int(end_sample) + 1):
            digest.update(struct.pack("<q", int(self.sample_index[sample])))
            digest.update(bytes.fromhex(self.sample_signature(sample)))
        return digest.hexdigest()

    # ------------------------------------------------------------------
    # contact-mode identity (discrete structure only)
    # ------------------------------------------------------------------
    def _contact_mode_descriptor(self, row: int) -> tuple:
        """Discrete semantic descriptor of one contact row (no numerics)."""
        geom = (int(self.contact_geom0_id[row]), int(self.contact_geom1_id[row]))
        return (
            tuple(sorted(geom)),
            int(self.contact_class[row]),
            int(self.contact_side[row]),
            int(self.contact_region[row]),
            int(self.contact_dim[row]),
            bool(self.contact_active[row]),
            bool(self.contact_legal_active[row]),
            bool(self.contact_prohibited[row]),
        )

    def contact_mode_descriptors(self, sample: int) -> list[tuple]:
        lo, hi = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
        return [self._contact_mode_descriptor(row) for row in range(lo, hi)]

    def support_mode_descriptor(self, sample: int) -> tuple[bool, bool, bool]:
        """(left legal support active, right legal support active, prohibited)."""
        lo, hi = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
        left = right = prohibited = False
        for row in range(lo, hi):
            if int(self.contact_legal_active[row]):
                side = int(self.contact_side[row])
                if side == 0:
                    left = True
                elif side == 1:
                    right = True
            if int(self.contact_prohibited[row]):
                prohibited = True
        return (left, right, prohibited)

    def constraint_mode_descriptors(self, sample: int) -> list[tuple]:
        """Discrete descriptor of every EFC row of one sample.

        Contact-constraint rows are associated with their canonical semantic
        contact identity (descriptor plus within-contact row ordinal), never
        with the raw contact-buffer index.  Non-contact rows use the stable
        ``(type, id)`` constraint identity.
        """
        lo_c, hi_c = int(self.contact_offsets[sample]), int(self.contact_offsets[sample + 1])
        lo_e, hi_e = int(self.efc_offsets[sample]), int(self.efc_offsets[sample + 1])
        contact_identity: dict[int, tuple] = {}
        for row in range(lo_c, hi_c):
            address = int(self.contact_efc_address[row])
            nrows = int(self.contact_efc_nrows[row])
            if address < 0 or nrows <= 0:
                continue
            descriptor = self._contact_mode_descriptor(row)
            for ordinal in range(nrows):
                contact_identity[address + ordinal] = (descriptor, ordinal)
        descriptors: list[tuple] = []
        for row in range(lo_e, hi_e):
            efc_type = int(self.efc_type[row])
            state = int(self.efc_state[row])
            identity = contact_identity.get(row - lo_e)
            if efc_type not in CONTACT_CONSTRAINT_TYPES or identity is None:
                identity = ("CONSTRAINT", efc_type, int(self.efc_id[row]))
            descriptors.append((efc_type, state, identity))
        return descriptors

    def sample_mode_signature(self, sample: int) -> str:
        """Discrete contact/constraint mode identity of one native sample.

        Encodes only discrete structure (contact descriptors with
        multiplicity, the support-mode descriptor and constraint descriptors
        with multiplicity).  Sample index, time, distances, positions, frames,
        forces and every numerical EFC/Jacobian value are excluded.
        """
        digest = hashlib.sha256()
        digest.update(V3_CONTACT_MODE_SIGNATURE_VERSION.encode("utf-8"))
        digest.update(b"|SUPPORT|")
        for flag in self.support_mode_descriptor(sample):
            digest.update(b"1" if flag else b"0")
        digest.update(b"|CONTACTS|")
        contacts = Counter(self.contact_mode_descriptors(sample))
        for descriptor, count in sorted(contacts.items(), key=lambda item: repr(item[0])):
            digest.update(repr((descriptor, int(count))).encode("utf-8"))
        digest.update(b"|CONSTRAINTS|")
        constraints = Counter(self.constraint_mode_descriptors(sample))
        for descriptor, count in sorted(constraints.items(), key=lambda item: repr(item[0])):
            digest.update(repr((descriptor, int(count))).encode("utf-8"))
        return digest.hexdigest()

    def branch_mode_signature(self, start_sample: int, end_sample: int) -> str:
        """Contact-mode identity over an inclusive native sample interval.

        The mode identity deliberately excludes the sample index, time and the
        branch/interval identity: two intervals occupy the same mode when
        their ordered per-sample mode signatures agree.
        """
        if not 0 <= start_sample <= end_sample < self.sample_count:
            raise V3ActiveSetCaptureError("mode signature interval outside the captured stream")
        digest = hashlib.sha256()
        digest.update(V3_CONTACT_MODE_SIGNATURE_VERSION.encode("utf-8"))
        for sample in range(int(start_sample), int(end_sample) + 1):
            digest.update(bytes.fromhex(self.sample_mode_signature(sample)))
        return digest.hexdigest()


class ActiveSetRecorder:
    """Append-only lossless recorder for the runtime contact / EFC buffers."""

    def __init__(self, plant) -> None:
        self.plant = plant
        self.model_sha256 = hashlib.sha256(model_xml().encode("utf-8")).hexdigest()
        self._columns: dict[str, list] = {}

    def _push(self, name: str, value) -> None:
        self._columns.setdefault(name, []).append(value)

    def append(self, data: mujoco.MjData, *, sample_index: int, time_s: float,
               records: Sequence[M.V3ContactRecord] | None = None) -> None:
        """Record one native sample; fails closed on any inconsistency."""
        if records is None:
            records = M.contact_records(self.plant, data)
        if len(records) != int(data.ncon):
            raise V3ActiveSetCaptureError(
                f"contact row count mismatch: decoded {len(records)} != runtime ncon {int(data.ncon)}")
        nefc = int(data.nefc)
        if nefc < 0:
            raise V3ActiveSetCaptureError("negative nefc")
        addresses = [int(r.efc_address) for r in records]
        for address in addresses:
            if address >= nefc:
                raise V3ActiveSetCaptureError(
                    f"contact efc_address {address} outside nefc {nefc}")

        self._push("sample_index", np.int64(sample_index))
        self._push("time_s", np.float64(time_s))
        self._push("ncon", np.int64(len(records)))
        self._push("nefc", np.int64(nefc))

        covered = np.zeros(nefc, dtype=bool)
        for position, record in enumerate(records):
            address = int(record.efc_address)
            nrows = 0
            if address >= 0:
                candidates = [a for a in addresses[position + 1:] if a >= 0]
                next_address = int(candidates[0]) if candidates else nefc
                nrows = int(next_address) - address
                if nrows < 0:
                    raise V3ActiveSetCaptureError("contact EFC addresses are not monotone")
                if address + nrows > nefc:
                    raise V3ActiveSetCaptureError("contact EFC span exceeds nefc")
                if covered[address:address + nrows].any():
                    raise V3ActiveSetCaptureError("contact EFC spans overlap")
                covered[address:address + nrows] = True
                for row in range(address, address + nrows):
                    if int(data.efc_type[row]) not in CONTACT_CONSTRAINT_TYPES:
                        raise V3ActiveSetCaptureError(
                            "contact EFC span contains a non-contact constraint row")
                    if int(data.efc_id[row]) != int(record.contact_id):
                        raise V3ActiveSetCaptureError(
                            "contact EFC row id does not match the contact identity")
            elif int(record.dim) == 0:
                nrows = 0
            self._push("contact_id", np.int64(record.contact_id))
            self._push("contact_geom0_id", np.int32(record.geom0_id))
            self._push("contact_geom1_id", np.int32(record.geom1_id))
            self._push("contact_class", np.int8(contact_class_code(record)))
            self._push("contact_side", np.int8(_SIDE_CODE[record.side]))
            self._push("contact_region", np.int8(_REGION_CODE[record.region]))
            self._push("contact_dist", np.float64(record.dist_m))
            self._push("contact_pos", np.asarray(record.position_world_m, dtype=np.float64))
            self._push("contact_frame", np.asarray(record.frame, dtype=np.float64))
            self._push("contact_efc_address", np.int32(address))
            self._push("contact_dim", np.int32(record.dim))
            self._push("contact_efc_nrows", np.int32(nrows))
            self._push("contact_force6", np.asarray(
                [*record.force_contact_frame_n, *record.torque_contact_frame_nm], dtype=np.float64))
            self._push("contact_normal_force", np.float64(record.normal_force_n))
            self._push("contact_active", np.bool_(record.active_constraint))
            self._push("contact_legal_active", np.bool_(record.active_legal_plantar))
            self._push("contact_prohibited", np.bool_(record.prohibited))

        is_contact = np.isin(np.asarray(data.efc_type[:nefc]), CONTACT_CONSTRAINT_TYPES)
        if not np.array_equal(covered, is_contact):
            raise V3ActiveSetCaptureError(
                "contact-type EFC row coverage mismatch (uncovered or foreign rows)")

        for row in range(nefc):
            self._push("efc_type", np.int8(data.efc_type[row]))
            self._push("efc_id", np.int32(data.efc_id[row]))
            self._push("efc_state", np.int8(data.efc_state[row]))
            self._push("efc_pos", np.float64(data.efc_pos[row]))
            self._push("efc_vel", np.float64(data.efc_vel[row]))
            self._push("efc_force", np.float64(data.efc_force[row]))
            self._push("efc_margin", np.float64(data.efc_margin[row]))
            self._push("efc_aref", np.float64(data.efc_aref[row]))
            self._push("efc_b", np.float64(data.efc_b[row]))
            self._push("efc_d", np.float64(data.efc_D[row]))
            self._push("efc_kbip", np.asarray(data.efc_KBIP[row], dtype=np.float64).copy())
            self._push("efc_frictionloss", np.float64(data.efc_frictionloss[row]))
            self._push("efc_jacobian", np.asarray(data.efc_J[row * self.plant.model.nv:
                                                              (row + 1) * self.plant.model.nv],
                                                  dtype=np.float64).copy())

    def finalize(self) -> V3ActiveSetTables:
        """Materialize CSR tables; validates counts and finiteness."""
        if not self._columns:
            raise V3ActiveSetCaptureError("empty capture")
        sample_index = np.asarray(self._columns["sample_index"], dtype=np.int64)
        time_s = np.asarray(self._columns["time_s"], dtype=np.float64)
        ncon = np.asarray(self._columns["ncon"], dtype=np.int64)
        nefc = np.asarray(self._columns["nefc"], dtype=np.int64)
        n = int(sample_index.shape[0])

        contact_offsets = np.zeros(n + 1, dtype=np.int64)
        contact_offsets[1:] = np.cumsum(ncon)
        efc_offsets = np.zeros(n + 1, dtype=np.int64)
        efc_offsets[1:] = np.cumsum(nefc)

        tables = V3ActiveSetTables(
            sample_index=sample_index,
            time_s=time_s,
            ncon=ncon,
            nefc=nefc,
            contact_offsets=contact_offsets,
            contact_id=np.asarray(self._columns["contact_id"], dtype=np.int64),
            contact_geom0_id=np.asarray(self._columns["contact_geom0_id"], dtype=np.int32),
            contact_geom1_id=np.asarray(self._columns["contact_geom1_id"], dtype=np.int32),
            contact_class=np.asarray(self._columns["contact_class"], dtype=np.int8),
            contact_side=np.asarray(self._columns["contact_side"], dtype=np.int8),
            contact_region=np.asarray(self._columns["contact_region"], dtype=np.int8),
            contact_dist=np.asarray(self._columns["contact_dist"], dtype=np.float64),
            contact_pos=np.asarray(self._columns["contact_pos"], dtype=np.float64).reshape(-1, 3),
            contact_frame=np.asarray(self._columns["contact_frame"], dtype=np.float64).reshape(-1, 9),
            contact_efc_address=np.asarray(self._columns["contact_efc_address"], dtype=np.int32),
            contact_dim=np.asarray(self._columns["contact_dim"], dtype=np.int32),
            contact_efc_nrows=np.asarray(self._columns["contact_efc_nrows"], dtype=np.int32),
            contact_force6=np.asarray(self._columns["contact_force6"], dtype=np.float64).reshape(-1, 6),
            contact_normal_force=np.asarray(self._columns["contact_normal_force"], dtype=np.float64),
            contact_active=np.asarray(self._columns["contact_active"], dtype=np.bool_),
            contact_legal_active=np.asarray(self._columns["contact_legal_active"], dtype=np.bool_),
            contact_prohibited=np.asarray(self._columns["contact_prohibited"], dtype=np.bool_),
            efc_offsets=efc_offsets,
            efc_type=np.asarray(self._columns["efc_type"], dtype=np.int8),
            efc_id=np.asarray(self._columns["efc_id"], dtype=np.int32),
            efc_state=np.asarray(self._columns["efc_state"], dtype=np.int8),
            efc_pos=np.asarray(self._columns["efc_pos"], dtype=np.float64),
            efc_vel=np.asarray(self._columns["efc_vel"], dtype=np.float64),
            efc_force=np.asarray(self._columns["efc_force"], dtype=np.float64),
            efc_margin=np.asarray(self._columns["efc_margin"], dtype=np.float64),
            efc_aref=np.asarray(self._columns["efc_aref"], dtype=np.float64),
            efc_b=np.asarray(self._columns["efc_b"], dtype=np.float64),
            efc_d=np.asarray(self._columns["efc_d"], dtype=np.float64),
            efc_kbip=np.asarray(self._columns["efc_kbip"], dtype=np.float64).reshape(-1, 4),
            efc_frictionloss=np.asarray(self._columns["efc_frictionloss"], dtype=np.float64),
            efc_jacobian=np.asarray(self._columns["efc_jacobian"], dtype=np.float64).reshape(
                -1, int(self.plant.model.nv)),
            model_sha256=self.model_sha256,
            nq=int(self.plant.model.nq),
            nv=int(self.plant.model.nv),
            nu=int(self.plant.model.nu),
            na=int(self.plant.model.na),
        )
        failures = tables.validate()
        if failures:
            raise V3ActiveSetCaptureError("; ".join(failures))
        for name in ("contact_dist", "contact_pos", "contact_frame", "contact_force6",
                     "contact_normal_force", "efc_pos", "efc_vel", "efc_force", "efc_margin",
                     "efc_aref", "efc_b", "efc_d", "efc_kbip",
                     "efc_frictionloss", "efc_jacobian"):
            array = getattr(tables, name)
            if array.size and not np.all(np.isfinite(array)):
                raise V3ActiveSetCaptureError(f"non-finite value in {name}")
        return tables


def capture_active_set(plant, data: mujoco.MjData, *, sample_index: int, time_s: float,
                       records: Sequence[M.V3ContactRecord] | None = None) -> V3ActiveSetTables:
    """Convenience single-sample capture (used by the contract tests)."""
    recorder = ActiveSetRecorder(plant)
    recorder.append(data, sample_index=sample_index, time_s=time_s, records=records)
    return recorder.finalize()


def decode_tables_to_arrays(tables: V3ActiveSetTables, sample: int) -> dict[str, np.ndarray]:
    """Reconstruct every ``ncon``/``nefc`` row of one sample as flat arrays.

    This is the lossless decoding path used to prove that the CSR encoding can
    rebuild the original runtime buffers row for row.
    """
    return {"contacts": tables.contact_rows(sample), "efc": tables.efc_rows(sample)}


__all__ = [
    "ActiveSetRecorder",
    "CONTACT_CONSTRAINT_TYPES",
    "DERIVATIVE_CENTRAL_ALLOWED",
    "DERIVATIVE_ONE_SIDED_SAME_MODE",
    "DERIVATIVE_UNAVAILABLE",
    "V3_ACTIVE_SET_CAPTURE_AUTHORITY_ID",
    "V3_ACTIVE_SET_SIGNATURE_VERSION",
    "V3_CONTACT_MODE_SIGNATURE_VERSION",
    "V3_EXACT_EVIDENCE_FINGERPRINT_VERSION",
    "V3ActiveSetCaptureError",
    "V3ActiveSetTables",
    "capture_active_set",
    "contact_class_code",
    "decode_tables_to_arrays",
    "derivative_eligibility",
]
