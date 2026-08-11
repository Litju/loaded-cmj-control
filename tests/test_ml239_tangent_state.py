from __future__ import annotations

from dataclasses import fields, replace
import inspect

import mujoco
import numpy as np
import pytest

from loaded_cmj.simulation.snapshot import MacroSnapshot
from loaded_cmj.simulation.so3 import log_so3, matrix_to_quaternion, quaternion_multiply
from loaded_cmj.simulation.tangent import (
    CACHE_SO3_SLICE,
    CONFIGURATION_SLICE,
    DRIVESTATE_SLICE,
    PREVIOUS_ACTION_SLICE,
    QVEL_SLICE,
    SO3BranchError,
    SO3_BRANCH_MAX_RAD,
    TANGENT_DIMENSION,
    WARMSTART_SLICE,
    boxminus,
    boxplus,
)
from loaded_cmj.simulation.transition import step_5ms
from tests.test_ml238_macro_state import fixtures as _ml238_fixtures


@pytest.fixture(scope="module")
def qualified_fixtures():
    return _ml238_fixtures.__wrapped__()


def _model(qualified_fixtures):
    return next(iter(qualified_fixtures.values())).plant.model


def _relative_cache(snapshot: MacroSnapshot) -> tuple[np.ndarray, ...]:
    rows = {
        int(body_id): np.asarray(row, dtype=np.float64).reshape(3, 3)
        for body_id, row in zip(snapshot.kinematic_body_ids, snapshot.kinematic_xmat)
    }
    return (
        rows[1].T @ rows[2],
        rows[1].T @ rows[4],
        rows[1].T @ rows[7],
    )


def _known_exp_matrix(phi: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(phi))
    if theta == 0.0:
        return np.eye(3, dtype=np.float64)
    quaternion = np.empty(4, dtype=np.float64)
    quaternion[0] = np.cos(theta / 2.0)
    quaternion[1:] = np.sin(theta / 2.0) * phi / theta
    raw = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(raw, quaternion)
    return raw.reshape(3, 3)


def _snapshot_with_cache_perturbation(
    snapshot: MacroSnapshot, phi: np.ndarray, block: int
) -> MacroSnapshot:
    rows = {
        int(body_id): np.asarray(row, dtype=np.float64).reshape(3, 3).copy()
        for body_id, row in zip(snapshot.kinematic_body_ids, snapshot.kinematic_xmat)
    }
    parent, child = ((1, 2), (1, 4), (1, 7))[block]
    relatives = _relative_cache(snapshot)
    rows[child] = rows[parent] @ relatives[block] @ _known_exp_matrix(phi)
    xmat = snapshot.kinematic_xmat.copy()
    for row_index, body_id in enumerate(snapshot.kinematic_body_ids):
        xmat[row_index] = rows[body_id].reshape(9)
    return replace(snapshot, kinematic_xmat=xmat)


def _assert_qpos_equivalent(model, qpos_a, qpos_b, atol=2e-12):
    tangent = np.empty(model.nv, dtype=np.float64)
    mujoco.mj_differentiatePos(model, tangent, 1.0, qpos_a, qpos_b)
    assert np.max(np.abs(tangent)) <= atol


def _assert_cache_valid(snapshot: MacroSnapshot, atol=2e-10):
    identity = np.eye(3)
    for row in snapshot.kinematic_xmat:
        rotation = row.reshape(3, 3)
        assert np.max(np.abs(rotation.T @ rotation - identity)) <= atol
        assert abs(float(np.linalg.det(rotation)) - 1.0) <= atol


def test_layout_dimension_and_slices():
    slices = (
        CONFIGURATION_SLICE,
        CACHE_SO3_SLICE,
        QVEL_SLICE,
        DRIVESTATE_SLICE,
        PREVIOUS_ACTION_SLICE,
        WARMSTART_SLICE,
    )
    assert tuple((item.start, item.stop) for item in slices) == (
        (0, 21),
        (21, 30),
        (30, 51),
        (51, 96),
        (96, 111),
        (111, 132),
    )
    assert sum(item.stop - item.start for item in slices) == TANGENT_DIMENSION == 132


def test_zero_identity_and_snapshot_schema(qualified_fixtures):
    model = _model(qualified_fixtures)
    expected_fields = {
        "qpos",
        "qvel",
        "time",
        "qacc_warmstart",
        "ctrl",
        "kinematic_body_ids",
        "kinematic_xmat",
        "a_plus",
        "a_minus",
        "tau_prev",
        "previous_command",
        "reversal_phase",
        "override_flags",
        "previous_accepted_action",
        "schema_version",
        "model_id",
        "model_revision",
        "mujoco_version",
        "model_xml_sha256",
        "dimensions",
        "mujoco_state_mask",
        "mujoco_state_size",
        "integrator",
        "solver",
        "solver_iterations",
        "solver_ls_iterations",
    }
    assert {item.name for item in fields(MacroSnapshot)} == expected_fields
    for fixture in qualified_fixtures.values():
        snapshot = fixture.snapshot
        result = boxplus(snapshot, np.zeros(TANGENT_DIMENSION), model=model)
        assert result is not snapshot
        assert result.schema_version == "LCMJ-V1-MACRO-SNAPSHOT-1.0.0"
        assert result.dimensions == snapshot.dimensions
        assert np.array_equal(result.qpos, snapshot.qpos)
        assert np.array_equal(result.qvel, snapshot.qvel)
        assert np.array_equal(result.kinematic_xmat, snapshot.kinematic_xmat)
        assert np.array_equal(result.a_plus, snapshot.a_plus)
        assert np.array_equal(result.a_minus, snapshot.a_minus)
        assert np.array_equal(result.tau_prev, snapshot.tau_prev)
        assert np.array_equal(result.previous_accepted_action, snapshot.previous_accepted_action)
        assert np.array_equal(result.qacc_warmstart, snapshot.qacc_warmstart)
        assert np.allclose(boxminus(snapshot, snapshot, model=model), np.zeros(132), atol=2e-12)
        _assert_cache_valid(result)


@pytest.mark.parametrize("tangent_index", (0, 1, 2, 3, 4, 5, 6, 9, 12, 15, 18, 20))
def test_configuration_native_signs(tangent_index, qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S0"].snapshot
    epsilon = 1e-4
    qpos = source.qpos.copy()
    if tangent_index < 3:
        qpos[tangent_index] += epsilon
    elif tangent_index < 6:
        perturbation = np.zeros(4)
        perturbation[0] = np.cos(epsilon / 2.0)
        perturbation[1 + tangent_index - 3] = np.sin(epsilon / 2.0)
        qpos[3:7] = quaternion_multiply(qpos[3:7], perturbation)
    elif tangent_index < 9:
        perturbation = np.zeros(4)
        perturbation[0] = np.cos(epsilon / 2.0)
        perturbation[1 + tangent_index - 6] = np.sin(epsilon / 2.0)
        qpos[7:11] = quaternion_multiply(qpos[7:11], perturbation)
    elif tangent_index < 12:
        perturbation = np.zeros(4)
        perturbation[0] = np.cos(epsilon / 2.0)
        perturbation[1 + tangent_index - 9] = np.sin(epsilon / 2.0)
        qpos[11:15] = quaternion_multiply(qpos[11:15], perturbation)
    elif tangent_index < 15:
        qpos[15 + tangent_index - 12] += epsilon
    elif tangent_index < 18:
        perturbation = np.zeros(4)
        perturbation[0] = np.cos(epsilon / 2.0)
        perturbation[1 + tangent_index - 15] = np.sin(epsilon / 2.0)
        qpos[18:22] = quaternion_multiply(qpos[18:22], perturbation)
    else:
        qpos[22 + tangent_index - 18] += epsilon
    target = replace(source, qpos=qpos)
    delta = boxminus(target, source, model=model)
    expected = np.zeros(21)
    expected[tangent_index] = epsilon
    assert np.allclose(delta[CONFIGURATION_SLICE], expected, atol=2e-12, rtol=2e-9)
    reconstructed = boxplus(source, delta, model=model)
    _assert_qpos_equivalent(model, reconstructed.qpos, target.qpos)


def test_configuration_random_local_roundtrip(qualified_fixtures):
    model = _model(qualified_fixtures)
    rng = np.random.default_rng(239042)
    for name in ("S0", "S1", "S4"):
        source = qualified_fixtures[name].snapshot
        for _ in range(8):
            delta = np.zeros(132)
            delta[CONFIGURATION_SLICE] = rng.normal(0.0, 1e-4, 21)
            target = boxplus(source, delta, model=model)
            recovered = boxminus(target, source, model=model)
            assert np.allclose(recovered[CONFIGURATION_SLICE], delta[CONFIGURATION_SLICE], atol=2e-11, rtol=2e-8)
            _assert_qpos_equivalent(model, boxplus(source, recovered, model=model).qpos, target.qpos)


def test_quaternion_sign_equivalence(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S0"].snapshot
    for start in (3, 7, 11, 18):
        qpos = source.qpos.copy()
        qpos[start : start + 4] *= -1.0
        sign_flipped = replace(source, qpos=qpos)
        delta = boxminus(sign_flipped, source, model=model)
        assert np.max(np.abs(delta[CONFIGURATION_SLICE])) <= 2e-12
        _assert_qpos_equivalent(model, sign_flipped.qpos, source.qpos)


@pytest.mark.parametrize("axis", (0, 1, 2))
def test_cache_known_right_rotation_sign(axis, qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S2"].snapshot
    epsilon = 1e-4
    phi = np.zeros(3)
    phi[axis] = epsilon
    delta = np.zeros(132)
    delta[21 + axis] = epsilon
    target = boxplus(source, delta, model=model)
    before = _relative_cache(source)[0]
    after = _relative_cache(target)[0]
    expected = _known_exp_matrix(phi)
    assert np.allclose(before.T @ after, expected, atol=2e-12, rtol=2e-10)
    recovered = boxminus(target, source, model=model)
    assert np.allclose(recovered[21:24], phi, atol=2e-12, rtol=2e-9)


def test_cache_exp_log_and_reconstruction_validity(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S4"].snapshot
    phi = np.asarray((0.20, -0.10, 0.15), dtype=np.float64)
    delta = np.zeros(132)
    delta[CACHE_SO3_SLICE] = np.tile(phi, 3)
    target = boxplus(source, delta, model=model)
    recovered = boxminus(target, source, model=model)
    assert np.allclose(recovered[CACHE_SO3_SLICE], delta[CACHE_SO3_SLICE], atol=3e-12, rtol=2e-9)
    _assert_cache_valid(target)
    for relative in _relative_cache(target):
        assert np.allclose(relative.T @ relative, np.eye(3), atol=2e-10)
        assert abs(float(np.linalg.det(relative)) - 1.0) <= 2e-10
    direct = _known_exp_matrix(phi)
    assert np.allclose(
        log_so3(matrix_to_quaternion(direct)), phi, atol=2e-12, rtol=2e-9
    )


def test_so3_branch_guard_accepts_local_and_rejects_nonlocal(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S0"].snapshot
    near = np.zeros(132)
    near[21] = SO3_BRANCH_MAX_RAD - 1e-6
    boxplus(source, near, model=model)
    at_guard = np.zeros(132)
    at_guard[21] = SO3_BRANCH_MAX_RAD
    with pytest.raises(SO3BranchError):
        boxplus(source, at_guard, model=model)
    beyond = np.zeros(132)
    beyond[21] = SO3_BRANCH_MAX_RAD + 1e-3
    with pytest.raises(SO3BranchError):
        boxplus(source, beyond, model=model)
    nonlocal_pair = _snapshot_with_cache_perturbation(
        source, np.asarray((SO3_BRANCH_MAX_RAD + 1e-3, 0.0, 0.0)), 0
    )
    with pytest.raises(SO3BranchError):
        boxminus(nonlocal_pair, source, model=model)
    near_pi_pair = _snapshot_with_cache_perturbation(
        source, np.asarray((np.pi - 1e-6, 0.0, 0.0)), 0
    )
    with pytest.raises(SO3BranchError):
        boxminus(near_pi_pair, source, model=model)


def test_full_basis_roundtrips_and_composite_roundtrip(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S3"].snapshot
    for index in range(TANGENT_DIMENSION):
        delta = np.zeros(TANGENT_DIMENSION)
        delta[index] = 1e-6
        recovered = boxminus(boxplus(source, delta, model=model), source, model=model)
        assert np.allclose(recovered, delta, atol=3e-11, rtol=3e-8)

    rng = np.random.default_rng(239239)
    composite = rng.normal(0.0, 1.0, TANGENT_DIMENSION)
    composite[CONFIGURATION_SLICE] *= 1e-4
    composite[CACHE_SO3_SLICE] *= 2e-2
    composite[QVEL_SLICE] *= 1e-3
    composite[DRIVESTATE_SLICE] *= 1e-3
    composite[PREVIOUS_ACTION_SLICE] *= 1e-3
    composite[WARMSTART_SLICE] *= 1e-3
    target = boxplus(source, composite, model=model)
    recovered = boxminus(target, source, model=model)
    assert np.allclose(recovered, composite, atol=3e-10, rtol=3e-8)


def test_boxplus_boxminus_reconstructs_nearby_raw_state_pairs(qualified_fixtures):
    model = _model(qualified_fixtures)
    for first, second in (("S0", "S1"), ("S1", "S2"), ("S2", "S3"), ("S4", "S5")):
        source = qualified_fixtures[first].snapshot
        target = qualified_fixtures[second].snapshot
        delta = boxminus(target, source, model=model)
        reconstructed = boxplus(source, delta, model=model)
        _assert_qpos_equivalent(model, reconstructed.qpos, target.qpos, atol=3e-10)
        for reconstructed_relative, target_relative in zip(
            _relative_cache(reconstructed), _relative_cache(target)
        ):
            assert np.allclose(reconstructed_relative, target_relative, atol=3e-10, rtol=2e-9)
        _assert_cache_valid(reconstructed)
        assert np.allclose(reconstructed.qvel, target.qvel, atol=2e-12)
        assert np.allclose(reconstructed.a_plus, target.a_plus, atol=2e-12)
        assert np.allclose(reconstructed.a_minus, target.a_minus, atol=2e-12)
        assert np.allclose(reconstructed.tau_prev, target.tau_prev, atol=2e-12)
        assert np.allclose(reconstructed.previous_accepted_action, target.previous_accepted_action, atol=2e-12)
        assert np.allclose(reconstructed.qacc_warmstart, target.qacc_warmstart, atol=2e-12)


def test_tangent_slice_isolation_and_no_slew(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S2"].snapshot
    cases = (
        (CONFIGURATION_SLICE, 0, 1e-4),
        (CACHE_SO3_SLICE, 0, 1e-2),
        (QVEL_SLICE, 0, 1e-3),
        (DRIVESTATE_SLICE, 0, 1e-3),
        (PREVIOUS_ACTION_SLICE, 0, 0.25),
        (WARMSTART_SLICE, 0, 1e-3),
    )
    for block, offset, amount in cases:
        delta = np.zeros(132)
        delta[block.start + offset] = amount
        target = boxplus(source, delta, model=model)
        if block is not CONFIGURATION_SLICE:
            assert np.array_equal(target.qpos, source.qpos)
        if block is not CACHE_SO3_SLICE:
            assert np.array_equal(target.kinematic_xmat, source.kinematic_xmat)
        if block is not QVEL_SLICE:
            assert np.array_equal(target.qvel, source.qvel)
        if block is not DRIVESTATE_SLICE:
            assert np.array_equal(target.a_plus, source.a_plus)
            assert np.array_equal(target.a_minus, source.a_minus)
            assert np.array_equal(target.tau_prev, source.tau_prev)
        if block is not PREVIOUS_ACTION_SLICE:
            assert np.array_equal(target.previous_accepted_action, source.previous_accepted_action)
        if block is not WARMSTART_SLICE:
            assert np.array_equal(target.qacc_warmstart, source.qacc_warmstart)
    cache_delta = np.zeros(132)
    cache_delta[21] = 1e-2
    cache_target = boxplus(source, cache_delta, model=model)
    assert np.array_equal(cache_target.kinematic_xmat[0], source.kinematic_xmat[0])
    assert not np.array_equal(cache_target.kinematic_xmat[1], source.kinematic_xmat[1])
    assert np.array_equal(cache_target.kinematic_xmat[2:], source.kinematic_xmat[2:])
    action_delta = np.zeros(132)
    action_delta[PREVIOUS_ACTION_SLICE.start] = 0.25
    action_target = boxplus(source, action_delta, model=model)
    assert action_target.previous_accepted_action[0] == source.previous_accepted_action[0] + 0.25
    assert np.array_equal(action_target.previous_command, source.previous_command)


def test_boxplus_copy_safe_and_does_not_mutate_inputs(qualified_fixtures):
    model = _model(qualified_fixtures)
    source = qualified_fixtures["S1"].snapshot
    source_qvel = source.qvel.copy()
    delta = np.zeros(132)
    delta[QVEL_SLICE.start] = 1e-3
    target = boxplus(source, delta, model=model)
    delta[QVEL_SLICE.start] = 5.0
    assert np.array_equal(source.qvel, source_qvel)
    assert target.qvel[0] == source_qvel[0] + 1e-3
    assert target.qvel.flags.writeable is False
    with pytest.raises(ValueError):
        target.qvel[0] = 0.0


@pytest.mark.parametrize(
    "delta",
    (
        np.zeros(131),
        np.zeros((132, 1)),
        np.full(132, np.nan),
        np.full(132, np.inf),
        np.full(132, -np.inf),
    ),
)
def test_invalid_delta_rejected(delta, qualified_fixtures):
    model = _model(qualified_fixtures)
    with pytest.raises(ValueError):
        boxplus(qualified_fixtures["S0"].snapshot, delta, model=model)


def test_transition_owner_has_no_tangent_scope_leakage():
    source = inspect.getsource(step_5ms)
    assert "boxplus" not in source
    assert "boxminus" not in source
    assert "mj_differentiatePos" not in source
    assert "mj_integratePos" not in source
