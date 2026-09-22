"""RES-86 E10-aware optimization and validation semantics.

MISSION: RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR ISSUE: RES-86

These tests bind the E10-awareness introduced for the production E10 closure:

* the optimization (feasibility oracle) ranks candidates with the frozen E10
  point gates and the first-contact-to-E10 transient envelopes from the start,
  with a graded reach pressure toward the D_BL dwell;
* the controller's objective targets are clipped to the frozen E10 point gates
  and the executed live trace latches the first envelope violation so a landing
  that left the frozen window can never confirm E10;
* the declared limits are exactly the frozen landing-acceptance authority
  values and the structured witness bounds cover every declared parameter.
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

import mujoco  # noqa: E402

from loaded_cmj.v3 import measurement as M  # noqa: E402
from loaded_cmj.v3.actuation import V3ActuationAuthority  # noqa: E402
from loaded_cmj.v3.landing_authority import landing_acceptance_authority  # noqa: E402
from loaded_cmj.v3.landing_control import (  # noqa: E402
    E10_POINT_LIMITS,
    E10_WINDOW_LIMITS,
    OUTPUT_INDEX,
    V3LandingConfig,
    V3LandingController,
)
from loaded_cmj.v3.plant import V3Plant  # noqa: E402
from loaded_cmj.v3.launch_runtime import settle_standing_stance  # noqa: E402
from tools.res86.landing_feasibility_oracle import (  # noqa: E402
    E10_REACH_PROGRESS_CREDIT,
    N_ORACLE_COORDS,
    PENALTY_NO_E10,
    STRUCTURED_PARAMETER_BOUNDS,
    STRUCTURED_STARTS,
    LandingFeasibilityOracle,
    StructuredWitnessParameters,
    capture_canonical_branch,
)


@pytest.fixture(scope="session")
def oracle_e10():
    return LandingFeasibilityOracle(capture_canonical_branch(), class_name="legal",
                                    e10_aware=True)


def test_declared_limits_match_frozen_authority():
    authority = landing_acceptance_authority()
    point = authority["E10_point_in_time"]
    window = authority["first_contact_to_E10_window"]
    assert E10_POINT_LIMITS == {
        "com_vx": point["max_abs_com_vx_m_s"],
        "hy": point["max_abs_Hy_kg_m2_s"],
        "root_pitch": point["max_abs_root_pitch_rad"],
        "trunk_pitch": point["max_abs_trunk_pitch_rad"],
        "root_pitch_rate": point["max_abs_root_pitch_rate_rad_s"],
        "trunk_pitch_rate": point["max_abs_trunk_pitch_rate_rad_s"],
    }
    assert E10_WINDOW_LIMITS == {
        "com_vx": window["max_abs_com_vx_m_s"],
        "hy": window["max_abs_Hy_kg_m2_s"],
        "root_pitch": window["root_pitch_envelope_rad"],
        "trunk_pitch": window["trunk_pitch_envelope_rad"],
        "root_pitch_rate": window["max_abs_root_pitch_rate_rad_s"],
        "trunk_pitch_rate": window["max_abs_trunk_pitch_rate_rad_s"],
    }


def _track(samples: int = 60, *, tail_start: int = 11, tail_stop: int | None = None,
           window_hy: float = 1.0, hy_spike_offset: int | None = None):
    entries = []
    for j in range(1, samples + 1):
        in_tail = j >= tail_start and (tail_stop is None or j <= tail_stop)
        entries.append({
            "offset": j,
            "time_s": 1.580 + j * 0.002,
            "left_legal": True, "right_legal": True,
            "left_fz": 500.0, "right_fz": 500.0,
            "vx": 0.0, "vz": -1.0 if not in_tail else 0.0,
            "hy": window_hy, "root_pitch": 0.0, "trunk_pitch": 0.0,
            "root_rate": 0.0, "trunk_rate": 0.0,
        })
    if hy_spike_offset is not None:
        entries[hy_spike_offset - 1]["hy"] = 6.0
    return entries


def test_e10_summary_confirms_at_25_intervals(oracle_e10):
    summary = oracle_e10._e10_summary(_track())
    assert summary["reached"] is True
    assert summary["first_contact_offset"] == 1
    assert summary["sustain_start_offset"] == 11
    assert summary["e10_offset"] == 36
    assert summary["window_gates_ok"] is True
    assert summary["point_gates_ok"] is True


def test_e10_summary_24_intervals_must_not_confirm(oracle_e10):
    summary = oracle_e10._e10_summary(_track(tail_stop=35))
    assert summary["reached"] is False
    assert summary["sustain_max_run_s"] == pytest.approx(0.048)
    assert summary["sustain_progress"] < 1.0


def test_e10_summary_window_envelope_and_progress(oracle_e10):
    summary = oracle_e10._e10_summary(_track(hy_spike_offset=5))
    assert summary["window_gates_ok"] is False
    assert summary["window_exceedance"] > 0.0
    assert summary["window_values"]["max_abs_hy_kg_m2_s"] == pytest.approx(6.0)
    # the reach pressure is graded by the sustained-predicate run, never a flat
    # cliff, and the tail vertical-velocity deficit is reported
    assert 0.0 <= summary["sustain_progress"] <= 1.0
    assert summary["tail_min_abs_vz_m_s"] == pytest.approx(0.0)


def test_e10_aware_cost_prices_the_reach_and_the_window(oracle_e10):
    zeros = np.zeros((4, N_ORACLE_COORDS), dtype=np.float64)
    result = oracle_e10.evaluate(zeros, 60, collect_samples=False)
    assert result.e10_aware is True
    assert result.e10_reached is False
    cost = oracle_e10.cost(result)
    assert cost >= PENALTY_NO_E10 * (1.0 - E10_REACH_PROGRESS_CREDIT)
    # a non-E10-aware oracle keeps the original cost (no E10 terms)
    plain = LandingFeasibilityOracle(oracle_e10.branch, class_name="legal")
    plain_result = plain.evaluate(zeros, 60, collect_samples=False)
    assert plain_result.e10_aware is False
    assert plain.cost(plain_result) < PENALTY_NO_E10 * (1.0 - E10_REACH_PROGRESS_CREDIT)


def test_structured_witness_bounds_cover_every_declared_parameter():
    fields = list(StructuredWitnessParameters.__dataclass_fields__)
    assert len(STRUCTURED_PARAMETER_BOUNDS) == len(fields)
    for lower, upper in STRUCTURED_PARAMETER_BOUNDS:
        assert lower <= upper
    for start in STRUCTURED_STARTS:
        for (lower, upper), name in zip(STRUCTURED_PARAMETER_BOUNDS, fields):
            value = float(getattr(start, name))
            assert lower <= value <= upper, (name, value)


def _flat_stance() -> tuple[V3Plant, object, V3ActuationAuthority]:
    plant = V3Plant()
    data = plant.make_data()
    settle_standing_stance(plant, data)
    authority = V3ActuationAuthority(float(plant.model.opt.timestep))
    return plant, data, authority


def test_objective_targets_are_clipped_to_the_point_gates():
    plant, data, authority = _flat_stance()
    controller = V3LandingController(plant, data, config=V3LandingConfig(),
                                     actuation=authority)
    base = controller.engine.capture_state(data, sample_index=0, time_s=0.0,
                                           authority=authority)
    outcome = controller.engine.evaluate(base, np.zeros(9), phase="LANDING_CAPTURE")
    target = controller._objective_targets(outcome)
    for name, limit in (("com_vx_terminal", E10_POINT_LIMITS["com_vx"]),
                        ("hy_terminal", E10_POINT_LIMITS["hy"]),
                        ("root_pitch_terminal", E10_POINT_LIMITS["root_pitch"]),
                        ("trunk_pitch_terminal", E10_POINT_LIMITS["trunk_pitch"]),
                        ("root_pitch_rate_terminal", E10_POINT_LIMITS["root_pitch_rate"]),
                        ("trunk_pitch_rate_terminal", E10_POINT_LIMITS["trunk_pitch_rate"])):
        assert abs(float(target[OUTPUT_INDEX[name]])) <= limit + 1.0e-12, name


def test_live_window_violation_latches_and_blocks_e10_confirmation():
    plant, data, authority = _flat_stance()
    controller = V3LandingController(plant, data, config=V3LandingConfig(),
                                     actuation=authority)
    frame = M.native_frame(plant, data, 0, 0.0)
    # before physical first contact the window is not active
    controller._first_contact_time_s = None
    controller._update_phase(frame, data)
    assert controller.e10_window_violated_time_s is None
    # a root pitch outside the frozen window envelope is latched with its cause
    data.qpos[plant.idx.qadr["root_ry"]] = 0.60
    mujoco.mj_forward(plant.model, data)
    controller._first_contact_time_s = 0.0
    controller._update_phase(frame, data)
    assert controller.e10_window_violated_time_s == 0.0
    assert "E10_WINDOW_ROOT_PITCH" in controller.e10_window_violation_reason
    # once violated, no later satisfying sample can confirm E10
    data.qpos[plant.idx.qadr["root_ry"]] = 0.0
    mujoco.mj_forward(plant.model, data)
    controller._sustain_start_time_s = -1.0
    controller._update_phase(frame, data)
    assert controller.e10_confirmed_time_s is None
    assert controller._sustain_start_time_s is None
