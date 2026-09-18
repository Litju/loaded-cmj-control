"""RES-86A lossless contact / EFC active-set recorder contract tests.

MISSION: `RES86A_LANDING_AUTHORITY_BRANCH_AND_RECORDER_FOUNDATION_001`
LINEAR ISSUE: RES-86

Every assertion exercises the variable-length CSR encoding against the live
MuJoCo runtime buffers of the sealed V3 Plant: row-count identity, lossless
reconstruction, fail-closed behaviour, multi-point contact handling and the
deterministic active-set signature.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
SRC = TASK_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.active_set_capture import (  # noqa: E402
    CONTACT_CONSTRAINT_TYPES,
    ActiveSetRecorder,
    V3ActiveSetCaptureError,
    capture_active_set,
)
from loaded_cmj.v3.launch_runtime import settle_standing_stance  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402

SAMPLE_INDEX = 5
SAMPLE_TIME_S = 0.010
BRANCH_ID = "RES86_TEST_BRANCH"
INTERVAL_ID = "RES86_TEST_INTERVAL"


@pytest.fixture(scope="session")
def pressed_state():
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    data.qpos[plant.idx.qadr["root_tz"]] -= 0.01
    mujoco.mj_forward(plant.model, data)
    return plant, data


@pytest.fixture(scope="session")
def tables(pressed_state):
    plant, data = pressed_state
    return capture_active_set(plant, data, sample_index=SAMPLE_INDEX, time_s=SAMPLE_TIME_S)


def test_ragged_offsets_match_runtime_counts(pressed_state, tables):
    plant, data = pressed_state
    assert tables.validate() == []
    assert tables.contact_count(0) == int(data.ncon)
    assert tables.efc_count(0) == int(data.nefc)
    assert np.array_equal(np.diff(tables.contact_offsets), tables.ncon)
    assert np.array_equal(np.diff(tables.efc_offsets), tables.nefc)
    assert int(tables.contact_offsets[-1]) == sum(tables.contact_count(i) for i in range(tables.sample_count))


def test_contact_encoding_reconstructs_every_row(pressed_state, tables):
    plant, data = pressed_state
    records = M.contact_records(plant, data)
    rows = tables.contact_rows(0)
    assert rows["dist"].shape[0] == len(records)
    for position, record in enumerate(records):
        assert rows["contact_id"][position] == record.contact_id
        assert rows["geom0_id"][position] == record.geom0_id
        assert rows["geom1_id"][position] == record.geom1_id
        assert rows["dist"][position] == record.dist_m
        assert np.array_equal(rows["pos"][position], np.asarray(record.position_world_m))
        assert np.array_equal(rows["frame"][position], np.asarray(record.frame))
        assert rows["efc_address"][position] == record.efc_address
        assert rows["dim"][position] == record.dim
        assert np.array_equal(rows["force6"][position],
                              np.asarray([*record.force_contact_frame_n,
                                          *record.torque_contact_frame_nm]))
        assert rows["normal_force"][position] == record.normal_force_n
        assert bool(rows["active"][position]) == record.active_constraint
        assert bool(rows["legal_active"][position]) == record.active_legal_plantar
        assert bool(rows["prohibited"][position]) == record.prohibited


def test_efc_encoding_reconstructs_every_row(pressed_state, tables):
    plant, data = pressed_state
    rows = tables.efc_rows(0)
    nefc = int(data.nefc)
    assert rows["type"].shape[0] == nefc
    for name, source in (("type", data.efc_type), ("id", data.efc_id),
                         ("state", data.efc_state), ("pos", data.efc_pos),
                         ("vel", data.efc_vel), ("force", data.efc_force),
                         ("margin", data.efc_margin), ("aref", data.efc_aref),
                         ("b", data.efc_b), ("d", data.efc_D),
                         ("frictionloss", data.efc_frictionloss)):
        assert np.array_equal(rows[name], np.asarray(source[:nefc])), name
    assert np.array_equal(rows["kbip"], np.asarray(data.efc_KBIP[:nefc]))
    assert np.array_equal(rows["jacobian"],
                          np.asarray(data.efc_J[:nefc * plant.model.nv]).reshape(nefc, plant.model.nv))


def test_multi_point_contacts_are_preserved(pressed_state, tables):
    plant, data = pressed_state
    records = M.contact_records(plant, data)
    pairs = [(r.geom0_id, r.geom1_id) for r in records]
    assert len(pairs) > len(set(pairs)), "expected multi-point (multi-CCD style) duplicate geom pairs"
    rows = tables.contact_rows(0)
    assert rows["geom0_id"].shape[0] == len(pairs)
    for position, pair in enumerate(pairs):
        assert (rows["geom0_id"][position], rows["geom1_id"][position]) == pair
    for position in range(len(records)):
        nrows = int(rows["efc_nrows"][position])
        if rows["efc_address"][position] >= 0:
            assert nrows > 0
            assert int(rows["dim"][position]) > 0
        start = int(rows["efc_address"][position])
        for row in range(start, start + nrows):
            assert int(tables.efc_type[row]) in CONTACT_CONSTRAINT_TYPES
            assert int(tables.efc_id[row]) == int(rows["contact_id"][position])


def test_recorder_fails_closed_on_contact_row_mismatch(tables):
    truncated = dataclasses.replace(tables, contact_id=tables.contact_id[:-1])
    with pytest.raises(V3ActiveSetCaptureError):
        truncated.require_valid()
    broken_offsets = tables.contact_offsets.copy()
    broken_offsets[-1] += 1
    mismatched = dataclasses.replace(tables, contact_offsets=broken_offsets)
    with pytest.raises(V3ActiveSetCaptureError):
        mismatched.require_valid()


def test_recorder_fails_closed_on_efc_row_mismatch(tables):
    truncated = dataclasses.replace(tables, efc_force=tables.efc_force[:-1])
    with pytest.raises(V3ActiveSetCaptureError):
        truncated.require_valid()
    broken_offsets = tables.efc_offsets.copy()
    broken_offsets[-1] -= 1
    mismatched = dataclasses.replace(tables, efc_offsets=broken_offsets)
    with pytest.raises(V3ActiveSetCaptureError):
        mismatched.require_valid()


def test_signature_is_deterministic(pressed_state):
    plant, data = pressed_state
    first = capture_active_set(plant, data, sample_index=SAMPLE_INDEX, time_s=SAMPLE_TIME_S)
    second = capture_active_set(plant, data, sample_index=SAMPLE_INDEX, time_s=SAMPLE_TIME_S)
    assert first.sample_signature(0) == first.sample_signature(0)
    assert first.sample_signature(0) == second.sample_signature(0)
    assert first.canonical_digest() == second.canonical_digest()
    assert first.branch_signature(0, 0, branch_id=BRANCH_ID,
                                  executed_interval_id=INTERVAL_ID) == \
        second.branch_signature(0, 0, branch_id=BRANCH_ID, executed_interval_id=INTERVAL_ID)


def test_changing_one_contact_changes_the_signature(tables):
    baseline = tables.branch_signature(0, 0, branch_id=BRANCH_ID, executed_interval_id=INTERVAL_ID)
    dist = tables.contact_dist.copy()
    dist[0] = dist[0] + 1.0e-6
    mutated = dataclasses.replace(tables, contact_dist=dist)
    assert mutated.branch_signature(0, 0, branch_id=BRANCH_ID,
                                    executed_interval_id=INTERVAL_ID) != baseline
    geom = tables.contact_geom1_id.copy()
    geom[0] = geom[0] + 1
    mutated = dataclasses.replace(tables, contact_geom1_id=geom)
    assert mutated.branch_signature(0, 0, branch_id=BRANCH_ID,
                                    executed_interval_id=INTERVAL_ID) != baseline


def test_prohibited_contact_changes_the_signature(tables):
    baseline = tables.branch_signature(0, 0, branch_id=BRANCH_ID, executed_interval_id=INTERVAL_ID)
    contact_class = tables.contact_class.copy()
    contact_class[0] = 1  # PROHIBITED_FLOOR
    mutated = dataclasses.replace(tables, contact_class=contact_class)
    assert mutated.branch_signature(0, 0, branch_id=BRANCH_ID,
                                    executed_interval_id=INTERVAL_ID) != baseline


def test_branch_interval_identity_changes_the_signature(tables):
    baseline = tables.branch_signature(0, 0, branch_id=BRANCH_ID, executed_interval_id=INTERVAL_ID)
    assert tables.branch_signature(0, 0, branch_id=BRANCH_ID + "_X",
                                   executed_interval_id=INTERVAL_ID) != baseline
    assert tables.branch_signature(0, 0, branch_id=BRANCH_ID,
                                   executed_interval_id=INTERVAL_ID + "_X") != baseline


def test_no_fixed_pad_truncation_at_predecessor_maxima(pressed_state, tables):
    plant, data = pressed_state
    assert int(data.ncon) == 24
    assert int(data.nefc) == 144
    assert tables.contact_count(0) == 24
    assert tables.efc_count(0) == 144
    assert tables.contact_count(0) > 16
    assert tables.efc_count(0) > 128


def test_multi_sample_stream_offsets_are_per_sample(pressed_state):
    plant, data = pressed_state
    recorder = ActiveSetRecorder(plant)
    for sample in range(3):
        recorder.append(data, sample_index=sample, time_s=sample * 0.002)
    tables = recorder.finalize()
    assert tables.validate() == []
    assert tables.sample_count == 3
    assert np.array_equal(np.diff(tables.contact_offsets), tables.ncon)
    assert np.array_equal(np.diff(tables.efc_offsets), tables.nefc)
    for sample in range(3):
        assert tables.contact_count(sample) == int(data.ncon)
        assert tables.efc_count(sample) == int(data.nefc)
        assert tables.sample_signature(sample) == tables.sample_signature(sample)


def test_non_contact_constraint_rows_are_preserved(pressed_state):
    plant, data = pressed_state
    data.qpos[plant.idx.qadr["trunk_pelvis"]] = 0.7  # beyond the frozen +-0.610865 ROM
    mujoco.mj_forward(plant.model, data)
    assert int(data.nefc) > int(np.count_nonzero(np.isin(
        np.asarray(data.efc_type[:data.nefc]), CONTACT_CONSTRAINT_TYPES)))
    tables = capture_active_set(plant, data, sample_index=0, time_s=0.0)
    assert tables.validate() == []
    assert tables.efc_count(0) == int(data.nefc)
    rows = tables.efc_rows(0)
    assert np.array_equal(rows["type"], np.asarray(data.efc_type[:data.nefc]))
    assert np.any(rows["type"] == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))


def test_stream_capture_is_not_a_live_memory_view(pressed_state):
    plant, data = pressed_state
    recorder = ActiveSetRecorder(plant)
    first_j = np.asarray(data.efc_J[:data.nefc * plant.model.nv], dtype=np.float64).copy()
    first_kbip = np.asarray(data.efc_KBIP[:data.nefc], dtype=np.float64).copy()
    recorder.append(data, sample_index=0, time_s=0.0)
    data.qpos[plant.idx.qadr["root_tz"]] -= 0.002
    mujoco.mj_forward(plant.model, data)
    recorder.append(data, sample_index=1, time_s=0.002)
    tables = recorder.finalize()
    assert np.array_equal(tables.efc_rows(0)["jacobian"].ravel(), first_j)
    assert np.array_equal(tables.efc_rows(0)["kbip"], first_kbip)
    assert not np.array_equal(tables.efc_rows(0)["jacobian"], tables.efc_rows(1)["jacobian"])


# ---------------------------------------------------------------------------
# A2: exact evidence fingerprint vs contact-mode identity
# ---------------------------------------------------------------------------
from loaded_cmj.v3.active_set_capture import (  # noqa: E402
    DERIVATIVE_CENTRAL_ALLOWED,
    DERIVATIVE_ONE_SIDED_SAME_MODE,
    DERIVATIVE_UNAVAILABLE,
    V3_ACTIVE_SET_SIGNATURE_VERSION,
    V3_CONTACT_MODE_SIGNATURE_VERSION,
    V3_EXACT_EVIDENCE_FINGERPRINT_VERSION,
    derivative_eligibility,
)

HISTORICAL_EXACT_BRANCH_FINGERPRINT = \
    "cea0ff7c92c7f10f0b168e67ba1758b97720557ce8a8eda2eb1710391a4a4cf1"


def _exact(tables) -> str:
    return tables.branch_signature(0, 0, branch_id=BRANCH_ID, executed_interval_id=INTERVAL_ID)


def _mode(tables) -> str:
    return tables.sample_mode_signature(0)


def test_fingerprint_and_mode_versions_are_separate():
    assert V3_EXACT_EVIDENCE_FINGERPRINT_VERSION == V3_ACTIVE_SET_SIGNATURE_VERSION == \
        "RES86_ACTIVE_SET_SIGNATURE_V1"
    assert V3_CONTACT_MODE_SIGNATURE_VERSION == "RES86_CONTACT_MODE_SIGNATURE_V1"
    assert V3_EXACT_EVIDENCE_FINGERPRINT_VERSION != V3_CONTACT_MODE_SIGNATURE_VERSION


def test_identical_state_has_same_exact_and_same_mode(tables):
    assert _exact(tables) == _exact(tables)
    assert _mode(tables) == _mode(tables)


def test_tiny_contact_distance_perturbation_exact_different_mode_same(tables):
    dist = tables.contact_dist.copy()
    dist[0] += 1.0e-12
    mutated = dataclasses.replace(tables, contact_dist=dist)
    assert _exact(mutated) != _exact(tables)
    assert _mode(mutated) == _mode(tables)


def test_tiny_force_perturbation_exact_different_mode_same(tables):
    force = tables.contact_force6.copy()
    force[0, 2] += 1.0e-12
    mutated = dataclasses.replace(tables, contact_force6=force)
    assert _exact(mutated) != _exact(tables)
    assert _mode(mutated) == _mode(tables)


def test_tiny_efc_jacobian_perturbation_exact_different_mode_same(tables):
    jac = tables.efc_jacobian.copy()
    jac[0, 0] += 1.0e-12
    mutated = dataclasses.replace(tables, efc_jacobian=jac)
    assert _exact(mutated) != _exact(tables)
    assert _mode(mutated) == _mode(tables)


def test_contact_geom_identity_change_mode_different(tables):
    geom = tables.contact_geom1_id.copy()
    geom[0] += 1
    mutated = dataclasses.replace(tables, contact_geom1_id=geom)
    assert _mode(mutated) != _mode(tables)


def test_legal_to_prohibited_mode_different(tables):
    contact_class = tables.contact_class.copy()
    prohibited = tables.contact_prohibited.copy()
    legal = tables.contact_legal_active.copy()
    contact_class[0] = 1
    prohibited[0] = True
    legal[0] = False
    mutated = dataclasses.replace(tables, contact_class=contact_class,
                                  contact_prohibited=prohibited,
                                  contact_legal_active=legal)
    assert _mode(mutated) != _mode(tables)


def test_support_side_loss_mode_different(tables):
    legal = tables.contact_legal_active.copy()
    side = tables.contact_side.copy()
    target = int(np.nonzero(side == 1)[0][0])
    legal[target] = False
    mutated = dataclasses.replace(tables, contact_legal_active=legal)
    assert _mode(mutated) != _mode(tables)


def test_efc_type_change_mode_different(tables):
    efc_type = tables.efc_type.copy()
    efc_type[0] = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    mutated = dataclasses.replace(tables, efc_type=efc_type)
    assert _mode(mutated) != _mode(tables)


def test_efc_state_change_mode_different(tables):
    efc_state = tables.efc_state.copy()
    efc_state[0] = 0
    mutated = dataclasses.replace(tables, efc_state=efc_state)
    assert _mode(mutated) != _mode(tables)


def test_contact_multiplicity_change_mode_different(tables):
    contact_offsets = tables.contact_offsets.copy()
    ncon = tables.ncon.copy()
    fields = {name: getattr(tables, name) for name in tables.__dataclass_fields__}
    for name in ("contact_id", "contact_geom0_id", "contact_geom1_id", "contact_class",
                 "contact_side", "contact_region", "contact_dist", "contact_pos",
                 "contact_frame", "contact_efc_address", "contact_dim", "contact_efc_nrows",
                 "contact_force6", "contact_normal_force", "contact_active",
                 "contact_legal_active", "contact_prohibited"):
        array = np.asarray(getattr(tables, name))
        fields[name] = np.concatenate([array, array[-1:]])
    contact_offsets[1] = contact_offsets[1] + 1
    ncon[0] = ncon[0] + 1
    fields["contact_offsets"] = contact_offsets
    fields["ncon"] = ncon
    mutated = dataclasses.replace(tables, **fields)
    assert _mode(mutated) != _mode(tables)


def test_equivalent_contact_row_permutation_mode_same(tables):
    lo, hi = int(tables.contact_offsets[0]), int(tables.contact_offsets[1])
    order = np.arange(hi - lo)
    rng = np.random.default_rng(0)
    rng.shuffle(order)
    fields = {name: getattr(tables, name) for name in tables.__dataclass_fields__}
    for name in ("contact_id", "contact_geom0_id", "contact_geom1_id", "contact_class",
                 "contact_side", "contact_region", "contact_dist", "contact_pos",
                 "contact_frame", "contact_efc_address", "contact_dim", "contact_efc_nrows",
                 "contact_force6", "contact_normal_force", "contact_active",
                 "contact_legal_active", "contact_prohibited"):
        array = np.asarray(getattr(tables, name)).copy()
        array[:hi - lo] = array[:hi - lo][order]
        fields[name] = array
    mutated = dataclasses.replace(tables, **fields)
    assert _mode(mutated) == _mode(tables)


def test_equivalent_efc_row_permutation_mode_same(tables):
    lo, hi = int(tables.efc_offsets[0]), int(tables.efc_offsets[1])
    order = np.arange(hi - lo)
    rng = np.random.default_rng(1)
    rng.shuffle(order)
    fields = {name: getattr(tables, name) for name in tables.__dataclass_fields__}
    for name in ("efc_type", "efc_id", "efc_state", "efc_pos", "efc_vel", "efc_force",
                 "efc_margin", "efc_aref", "efc_b", "efc_d", "efc_kbip",
                 "efc_frictionloss", "efc_jacobian"):
        array = np.asarray(getattr(tables, name)).copy()
        array[:hi - lo] = array[:hi - lo][order]
        fields[name] = array
    mutated = dataclasses.replace(tables, **fields)
    assert _mode(mutated) == _mode(tables)


def test_historical_exact_branch_fingerprint_unchanged():
    from loaded_cmj.v3.active_set_capture import V3ActiveSetTables
    bundle = (TASK_ROOT.parent / "loaded-cmj-control-evidence" /
              "EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001")
    tables_npz = bundle / "branch" / "contact_efc_tables.npz"
    if not tables_npz.exists():
        pytest.skip("sealed RES-86A tables bundle not present")
    import json

    arrays = {name: np.load(tables_npz)[name] for name in np.load(tables_npz).files}
    manifest = json.loads((bundle / "branch" / "contact_efc_tables_manifest.json").read_text())
    tables = V3ActiveSetTables(model_sha256=manifest["model_sha256"],
                               nq=12, nv=12, nu=9, na=0, **arrays)
    assert tables.canonical_digest() == \
        "49229f6c4b2b0b2b8fda562eec088be2800fccfeb37431452fe66d5a0f895833"
    assert tables.branch_signature(
        790, 1499, branch_id="RES86_BRANCH_PRE_TOUCHDOWN_SAMPLE_790",
        executed_interval_id="NATIVE_SAMPLES_790_1499") == \
        HISTORICAL_EXACT_BRANCH_FINGERPRINT


def test_derivative_eligibility_rule_is_frozen():
    assert derivative_eligibility("M", "M", "M") == DERIVATIVE_CENTRAL_ALLOWED
    assert derivative_eligibility("M", "M", "X") == DERIVATIVE_ONE_SIDED_SAME_MODE
    assert derivative_eligibility("M", "X", "M") == DERIVATIVE_ONE_SIDED_SAME_MODE
    assert derivative_eligibility("M", "X", "X") == DERIVATIVE_UNAVAILABLE
