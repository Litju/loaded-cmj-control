"""Regression tests for the bounded ML-243 WITNESS initializer rule."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from loaded_cmj.oracle.nlp_binding import (
    E3E4NLPBinding,
    WITNESS_A_MINUS_10_STATE_INDEX,
    WITNESS_A_MINUS_9_STATE_INDEX,
    WITNESS_MAX_INITIALIZER_DISPLACEMENT,
    WITNESS_RIGHT_ANKLE_EVERSION_BIAS,
    WITNESS_RIGHT_ANKLE_EVERSION_STATE_INDEX,
)
from loaded_cmj.simulation.tangent import boxminus, boxplus

sys.path.insert(0, str(Path(__file__).parent))
from test_ml238_macro_state import fixtures as macro_fixtures


@pytest.fixture(scope="module")
def supported_fixture():
    return macro_fixtures.__wrapped__()["S0"]


def _binding_for_fixture(fixture):
    binding = object.__new__(E3E4NLPBinding)
    binding.problem = SimpleNamespace(
        horizon=1,
        plant=fixture.plant,
        reference_snapshots=(fixture.snapshot, fixture.snapshot),
    )
    return binding


def test_witness_initializer_rule_is_deterministic_and_tangent_packaged(supported_fixture) -> None:
    binding = _binding_for_fixture(supported_fixture)
    reference = supported_fixture.snapshot

    first = binding._witness_initializer_state_delta(1)
    second = binding._witness_initializer_state_delta(1)

    assert np.array_equal(first, second)
    assert np.isfinite(first).all()
    assert np.max(np.abs(first)) <= WITNESS_MAX_INITIALIZER_DISPLACEMENT
    changed = set(np.flatnonzero(first != 0.0).tolist())
    assert changed == {
        2,
        WITNESS_RIGHT_ANKLE_EVERSION_STATE_INDEX,
        WITNESS_A_MINUS_9_STATE_INDEX,
        WITNESS_A_MINUS_10_STATE_INDEX,
    }
    assert np.isclose(
        first[WITNESS_RIGHT_ANKLE_EVERSION_STATE_INDEX],
        WITNESS_RIGHT_ANKLE_EVERSION_BIAS,
        rtol=0.0,
        atol=1.0e-12,
    )

    reconstructed = boxplus(reference, first, model=supported_fixture.plant.model)
    repackaged = boxminus(reconstructed, reference, model=supported_fixture.plant.model)
    np.testing.assert_allclose(repackaged, first, rtol=0.0, atol=1.0e-12)
