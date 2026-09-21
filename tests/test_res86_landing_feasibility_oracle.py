"""RES-86 controller-independent landing feasibility oracle tests.

MISSION: RES86_TOE_PHASE_FEASIBILITY_AND_RESOLUTION_001
LINEAR ISSUE: RES-86

The oracle is the exact-branch feasibility instrument used to separate a
controller/map defect (CASE A) from a hybrid contact-progression defect
(CASE B).  These tests bind its identity, its fast-vs-full measurement
equivalence, the profile parameterization and the deterministic non-stochastic
search surface.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

TASK_ROOT = Path(__file__).resolve().parents[1]
for _path in (str(TASK_ROOT), str(TASK_ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.actuation import MOMENT_CEILING_NM  # noqa: E402
from tools.res86.landing_feasibility_oracle import (  # noqa: E402
    BW_N,
    E8_STATE_SHA256,
    N_ORACLE_COORDS,
    PRE_TOUCHDOWN_STATE_SHA256,
    BranchStabilizer,
    COORDINATE_CEILING,
    LandingFeasibilityOracle,
    capture_canonical_branch,
    default_seed_ordering,
)

HORIZON_STEPS = 150


@pytest.fixture(scope="session")
def branch():
    return capture_canonical_branch()


@pytest.fixture(scope="session")
def oracle_legal(branch):
    return LandingFeasibilityOracle(branch, class_name="legal")


@pytest.fixture(scope="session")
def oracle_toe(branch):
    return LandingFeasibilityOracle(branch, class_name="toe")


def test_branch_certificates_match_sealed_identity(branch):
    assert branch.pre_touchdown.sample_index == 790
    assert branch.pre_touchdown.time_s == 1.580
    assert branch.pre_touchdown.state_sha256 == PRE_TOUCHDOWN_STATE_SHA256
    assert branch.e8.sample_index == 791
    assert branch.e8.time_s == 1.582
    assert branch.e8.state_sha256 == E8_STATE_SHA256
    assert branch.pre_touchdown.state_vector.shape == branch.e8.state_vector.shape
    assert branch.e8.actuation.previous_applied_nm
    # The flight phase gate zeroes the applied active MTP moment transiently; it
    # must not latch the energy ledger, so the landing continues from the
    # unchanged cumulative budget.
    for entry in branch.pre_touchdown.actuation.mtp_ledger:
        assert entry.active_gated is False
        assert entry.late_phase is True
        assert entry.active_positive_work_j > 0.0
        assert entry.active_positive_work_j < 0.5 * 25.0


def test_profile_mapping_is_piecewise_linear_and_symmetric(oracle_legal):
    knots = np.zeros((3, N_ORACLE_COORDS), dtype=np.float64)
    knots[0] = 0.0
    knots[1] = COORDINATE_CEILING
    knots[2] = -np.asarray(COORDINATE_CEILING)
    profile = oracle_legal.profile_from_knots(knots, 4, 3)
    assert profile.shape == (4, 9)
    # bilateral channels of each pair carry the same value
    for _coordinate, channels in (("hip_pair", (1, 2)), ("knee_pair", (3, 4)),
                                  ("ankle_pair", (5, 6))):
        assert np.array_equal(profile[:, channels[0]], profile[:, channels[1]])
    # the MTP channel is not a decision variable and stays zero
    assert np.all(profile[:, 7] == 0.0)
    assert np.all(profile[:, 8] == 0.0)
    # piecewise-linear interpolation at the declared sample midpoints
    times = oracle_legal.knot_times(4, 3)
    sample_times = (np.arange(4, dtype=np.float64) + 0.5) * oracle_legal.dt
    expected = np.interp(sample_times, times, knots[:, 1])
    assert np.allclose(profile[:, 1], expected, rtol=0, atol=1e-12)
    # all knots are clipped inside the frozen moment ceiling
    assert np.all(np.abs(profile) <= np.asarray(MOMENT_CEILING_NM) + 1e-9)


def test_fast_path_matches_full_authority_replay(oracle_legal):
    knots = np.zeros((4, N_ORACLE_COORDS), dtype=np.float64)
    result = oracle_legal.evaluate(knots, 60, collect_samples=False)
    verification = oracle_legal.verify(result.desired_nm, 60)
    assert verification["status"] == "PASS"
    assert verification["checks"]["terminal_vz_match"]
    assert verification["checks"]["penetration_match"]
    assert verification["checks"]["rom_match"]
    assert verification["checks"]["net_impulse_match"]
    assert verification["checks"]["prohibited_match"]


def test_evaluation_is_deterministic(oracle_legal):
    knots = np.zeros((5, N_ORACLE_COORDS), dtype=np.float64)
    first = oracle_legal.evaluate(knots, 40, collect_samples=False)
    second = oracle_legal.evaluate(knots, 40, collect_samples=False)
    assert first.terminal_state_sha256 == second.terminal_state_sha256
    assert first.terminal_com_vz_m_s == second.terminal_com_vz_m_s
    assert first.max_penetration_m == second.max_penetration_m


def test_initial_contact_digs_beyond_limit_without_control(oracle_legal):
    """The frozen launch hands over an extending leg; the free toe phase digs."""
    knots = np.zeros((3, N_ORACLE_COORDS), dtype=np.float64)
    result = oracle_legal.evaluate(knots, 60, collect_samples=False)
    assert result.max_penetration_m > 0.010
    assert "MAX_PENETRATION" in result.hard_gate_failures
    assert result.prohibited_any is False


def test_static_support_seed_is_inside_actuation_authority(oracle_legal):
    seeds = default_seed_ordering(oracle_legal, 5)
    assert len(seeds) >= 2
    for seed in seeds:
        assert seed.shape == (5, N_ORACLE_COORDS)
        for index, ceiling in enumerate(COORDINATE_CEILING):
            assert np.all(np.abs(seed[:, index]) <= ceiling + 1e-9)
    assert np.any(seeds[1] != 0.0)


def test_stabilizer_reduces_structural_rom_violation(oracle_legal):
    """The declared stabilizer term improves the branch inside the same authority."""
    knots = np.zeros((5, N_ORACLE_COORDS), dtype=np.float64)
    plain = oracle_legal.evaluate(knots, 75, collect_samples=False)
    stabilized = oracle_legal.evaluate(knots, 75, collect_samples=False,
                                       stabilizer=BranchStabilizer(), record_actions=True)
    assert stabilized.min_rom_margin_rad > plain.min_rom_margin_rad
    assert stabilized.max_moment_ratio <= 1.0 + 1e-9
    assert stabilized.max_power_ratio <= 1.0 + 1e-9
    assert stabilized.mtp_authority_ok is True
    assert stabilized.peak_total_fz_n / BW_N <= 8.0
    assert stabilized.prohibited_any is False


def test_toe_class_rejects_forefoot_progression(oracle_toe):
    """A branch that loads the forefoot is rejected by the toe-restricted class."""
    knots = np.zeros((3, N_ORACLE_COORDS), dtype=np.float64)
    result = oracle_toe.evaluate(knots, 40, collect_samples=False)
    assert result.toe_mode_escaped is True
    assert "TOE_MODE_ESCAPED" in result.hard_gate_failures
    assert result.admissible is False


def test_hard_gate_catalog_and_cost_ordering(oracle_legal):
    knots = np.zeros((3, N_ORACLE_COORDS), dtype=np.float64)
    result = oracle_legal.evaluate(knots, 60, collect_samples=False)
    failures = result.hard_gate_failures
    assert "MAX_PENETRATION" in failures
    cost = oracle_legal.cost(result)
    assert cost > 0.0
