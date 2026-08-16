from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from loaded_cmj.oracle import derivatives
from loaded_cmj.oracle.derivatives import ActiveSetFingerprint
from loaded_cmj.simulation.plant import (
    Plant,
    SupportMarginBranchCertificate,
    build_model,
)


class _SyntheticPlant(Plant):
    def __init__(self, points: object) -> None:
        self._synthetic_points = np.asarray(points, dtype=np.float64).reshape(-1, 2)

    def support_polygon(self, data: object, latched: tuple[bool, bool]) -> np.ndarray:
        del data, latched
        return self._synthetic_points.copy()


SQUARE = np.asarray(
    [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    dtype=np.float64,
)


def _diagnostic(
    points: object,
    point: object,
    active: tuple[bool, bool] = (True, True),
):
    plant = _SyntheticPlant(points)
    return plant.support_margin_diagnostics(
        data=None,
        com=np.asarray((*np.asarray(point, dtype=np.float64), 0.0)),
        latched=active,
    )


@pytest.fixture
def real_plant_data() -> tuple[Plant, object]:
    plant = Plant(build_model())
    data = plant.make_data()
    plant.reset_supported(data)
    return plant, data


def test_scalar_public_behavior_and_diagnostic_margin_are_exact(real_plant_data):
    plant, data = real_plant_data
    com = plant.center_of_mass(data)
    for active in ((False, False), (True, False), (False, True), (True, True)):
        scalar = plant.support_margin(data, com, active)
        diagnostic = plant.support_margin_diagnostics(data, com, active)
        assert diagnostic.margin == scalar


def test_diagnostic_margin_matches_support_margin_for_real_support(real_plant_data):
    plant, data = real_plant_data
    com = plant.center_of_mass(data)
    diagnostic = plant.support_margin_diagnostics(data, com, (True, True))
    assert diagnostic.margin == plant.support_margin(data, com, (True, True))
    assert diagnostic.support_active_set == (True, True)
    assert diagnostic.support_point_count == 8


def test_unique_smooth_inside_edge_branch():
    diagnostic = _diagnostic(SQUARE, [0.25, 0.5])
    assert diagnostic.branch_family == "POLYGON_INSIDE"
    assert not diagnostic.exact_tie
    assert not diagnostic.norm_zero_kink
    assert diagnostic.unique_active_minimizer is not None
    assert diagnostic.projection_regimes == ()


def test_inside_active_edge_switch_is_visible():
    left = _diagnostic(SQUARE, [0.25, 0.5])
    right = _diagnostic(SQUARE, [0.75, 0.5])
    assert left.branch_family == right.branch_family == "POLYGON_INSIDE"
    assert left.active_minimizer_set != right.active_minimizer_set


def test_exact_inside_tied_minimum_is_visible():
    diagnostic = _diagnostic(SQUARE, [0.5, 0.5])
    assert diagnostic.branch_family == "POLYGON_INSIDE"
    assert diagnostic.exact_tie
    assert len(diagnostic.active_minimizer_set) == 4


def test_smooth_outside_segment_branch_is_visible():
    lower = _diagnostic(SQUARE, [0.5, 2.0])
    farther = _diagnostic(SQUARE, [0.5, 3.0])
    assert lower.branch_family == farther.branch_family == "POLYGON_OUTSIDE"
    assert not lower.exact_tie
    assert lower.active_minimizer_set == farther.active_minimizer_set
    assert lower.projection_regimes == farther.projection_regimes
    assert lower.margin != farther.margin
    assert {label for _, label in lower.projection_regimes} == {"INTERIOR"}


def test_outside_active_segment_switch_is_visible():
    top = _diagnostic(SQUARE, [0.5, 2.0])
    right = _diagnostic(SQUARE, [2.0, 0.5])
    assert top.active_minimizer_set != right.active_minimizer_set


def test_segment_projection_endpoint_a_and_b_regimes_are_exposed():
    corner = _diagnostic(SQUARE, [0.0, 2.0])
    assert corner.branch_family == "POLYGON_OUTSIDE"
    assert corner.exact_tie
    assert {label for _, label in corner.projection_regimes} == {
        "ENDPOINT_A",
        "ENDPOINT_B",
    }


def test_segment_projection_regime_change_is_visible():
    interior = _diagnostic(SQUARE, [0.5, 2.0])
    endpoint = _diagnostic(SQUARE, [0.0, 2.0])
    assert interior.projection_regimes != endpoint.projection_regimes


def test_hull_topology_switch_is_visible_without_support_set_change():
    interior_point = _diagnostic(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.5, 0.5]],
        [0.25, 0.5],
    )
    hull_vertex = _diagnostic(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.5, 1.2]],
        [0.25, 0.5],
    )
    assert interior_point.support_point_count == hull_vertex.support_point_count == 5
    assert interior_point.canonical_hull_vertex_order != hull_vertex.canonical_hull_vertex_order


def test_support_active_set_switch_is_visible():
    left = _diagnostic(SQUARE, [0.25, 0.5], active=(True, False))
    bilateral = _diagnostic(SQUARE, [0.25, 0.5], active=(True, True))
    assert left.support_active_set != bilateral.support_active_set
    assert left.derivative_branch_certificate != bilateral.derivative_branch_certificate


def test_degenerate_support_branch_and_norm_zero_kink():
    zero = _diagnostic([[0.0, 0.0]], [0.0, 0.0])
    off_zero = _diagnostic([[-1.0, 0.0], [1.0, 0.0]], [0.0, 1.0])
    assert zero.branch_family == off_zero.branch_family == "DEGENERATE"
    assert zero.norm_zero_kink
    assert not off_zero.norm_zero_kink


def test_empty_support_and_nonfinite_support_are_nonfinite():
    empty = _diagnostic([], [0.0, 0.0], active=(False, False))
    nonfinite = _diagnostic(
        [[0.0, 0.0], [np.nan, 0.0], [1.0, 1.0], [0.0, 1.0]],
        [0.5, 0.5],
    )
    assert empty.branch_family == "EMPTY"
    assert empty.margin == -float("inf")
    assert not empty.finite
    assert nonfinite.branch_family == "NONFINITE"
    assert not nonfinite.finite


def test_same_branch_continuous_value_change_has_same_certificate():
    first = _diagnostic(SQUARE, [0.25, 0.5])
    second = _diagnostic(SQUARE, [0.35, 0.5])
    assert first.margin != second.margin
    assert first.derivative_branch_certificate == second.derivative_branch_certificate


def _certificate(**overrides) -> SupportMarginBranchCertificate:
    values = dict(
        support_active_set=(True, True),
        branch_family="POLYGON_INSIDE",
        support_point_count=4,
        canonical_hull_vertex_order=(0, 1, 2, 3),
        active_minimizer_set=((0, 1),),
        unique_active_minimizer=(0, 1),
        projection_regimes=(),
        exact_tie=False,
        norm_zero_kink=False,
        finite=True,
    )
    values.update(overrides)
    return SupportMarginBranchCertificate(**values)


def _evaluation(
    certificate: SupportMarginBranchCertificate,
    *,
    contact_steps: tuple = (),
    support_active_steps: tuple = (),
):
    active_set = ActiveSetFingerprint(
        contact_steps=contact_steps,
        prohibited_contact_steps=(),
        cop_valid_steps=(),
        support_active_steps=support_active_steps,
        friction_steps=(),
        native_joint_limit_steps=(),
        drive_flag_steps=(),
        action_branches=("INTERIOR",) * 15,
    )
    return SimpleNamespace(
        active_set=active_set,
        support_margin_branch_steps=(certificate,),
    )


def _plan(classification: str = derivatives.CENTRAL_INTERIOR):
    return SimpleNamespace(
        classification=classification,
        native_domain_legal=True,
        reason="",
    )


@pytest.mark.parametrize(
    "classification",
    [
        derivatives.CENTRAL_INTERIOR,
        derivatives.FORWARD_FEASIBLE_SIDE,
        derivatives.BACKWARD_FEASIBLE_SIDE,
    ],
)
def test_support_owner_certificate_checks_every_declared_stencil_sample(classification):
    base = _evaluation(_certificate())
    samples = [
        _evaluation(_certificate()),
        _evaluation(_certificate(active_minimizer_set=((1, 2),), unique_active_minimizer=(1, 2))),
    ]
    report = derivatives._column_result(
        axis="OWNER_STATE",
        index=0,
        step=1.0,
        base=base,
        plus=samples[0],
        minus=samples[1],
        samples=samples,
        block="test",
        plan=_plan(classification),
        support_geometry=True,
        support_margin_owner=True,
    )
    assert not report.valid
    assert report.reason == derivatives.SUPPORT_HULL_KINK_INVALID


def test_unique_same_branch_is_accepted_by_support_certificate():
    base = _evaluation(_certificate())
    sample = _evaluation(_certificate())
    assert derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )


def test_redundant_canonical_hull_change_is_accepted_by_support_certificate():
    base = _evaluation(_certificate())
    sample = _evaluation(
        _certificate(canonical_hull_vertex_order=(0, 1, 3, 2))
    )
    assert derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )
    assert derivatives._support_margin_branch_rejection_reason(base, sample) == ""


def test_support_margin_owner_accepts_raw_contact_switch_with_invariant_support():
    base = _evaluation(_certificate())
    sample = _evaluation(_certificate(), contact_steps=((0, 1),))
    report = derivatives._column_result(
        axis="OWNER_STATE",
        index=0,
        step=1.0,
        base=base,
        plus=sample,
        minus=sample,
        samples=(sample, sample),
        block="test",
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )
    assert report.valid
    assert report.reason == "SMOOTH_FIXED_ACTIVE_SET"
    assert derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )


def test_raw_contact_switch_remains_rejected_without_support_margin_opt_in():
    base = _evaluation(_certificate())
    sample = _evaluation(_certificate(), contact_steps=((0, 1),))
    assert not derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=False,
    )
    assert derivatives._branch_rejection_reason(
        base,
        [sample],
        support_geometry=True,
        support_margin_owner=False,
    ) == derivatives.PHYSICAL_CONTACT_SWITCH_INVALID


def test_support_margin_owner_rejects_active_foot_switch_even_when_contact_changes_are_opted_in():
    base = _evaluation(_certificate(), support_active_steps=((True, True),))
    sample = _evaluation(
        _certificate(),
        contact_steps=((0, 1),),
        support_active_steps=((True, False),),
    )
    assert not derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )
    assert derivatives._branch_rejection_reason(
        base,
        [sample],
        support_geometry=True,
        support_margin_owner=True,
    ) == derivatives.PHYSICAL_CONTACT_SWITCH_INVALID


@pytest.mark.parametrize(
    ("changed", "expected_reason"),
    [
        (
            dict(active_minimizer_set=((1, 2),), unique_active_minimizer=(1, 2)),
            derivatives.SUPPORT_HULL_KINK_INVALID,
        ),
        (
            dict(exact_tie=True),
            derivatives.SUPPORT_HULL_KINK_INVALID,
        ),
        (
            dict(projection_regimes=(((0, 1), "ENDPOINT_A"),)),
            derivatives.SUPPORT_HULL_PROJECTION_BRANCH_SWITCH_INVALID,
        ),
        (
            dict(canonical_hull_vertex_order=(0, 1, 2, 3, 4), support_point_count=5),
            derivatives.SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID,
        ),
        (
            dict(support_point_count=5),
            derivatives.SUPPORT_HULL_TOPOLOGY_SWITCH_INVALID,
        ),
        (
            dict(support_active_set=(False, True)),
            derivatives.SUPPORT_ACTIVE_SET_SWITCH_INVALID,
        ),
        (
            dict(finite=False, branch_family="NONFINITE"),
            derivatives.NONFINITE_EVALUATION,
        ),
        (
            dict(norm_zero_kink=True),
            derivatives.SUPPORT_HULL_KINK_INVALID,
        ),
    ],
)
def test_support_certificate_negative_controls_reject(changed, expected_reason):
    base = _evaluation(_certificate())
    sample = _evaluation(_certificate(**changed))
    assert not derivatives._same_certificate_for_column(
        base,
        sample,
        axis="OWNER_STATE",
        index=0,
        plan=_plan(),
        support_geometry=True,
        support_margin_owner=True,
    )
    assert derivatives._branch_rejection_reason(
        base,
        [sample],
        support_geometry=True,
        support_margin_owner=True,
    ) == expected_reason


def test_support_certificate_owner_coverage_excludes_reversal():
    assert "SUPPORT_MARGIN" in derivatives._SUPPORT_GEOMETRY_OWNER_IDS
    assert "E3_E4_HORIZONTAL" in derivatives._SUPPORT_GEOMETRY_OWNER_IDS
    assert "E3_E4_REVERSAL" not in derivatives._SUPPORT_GEOMETRY_OWNER_IDS


@pytest.mark.parametrize(
    ("owner_id", "valid", "reason"),
    [
        ("SUPPORT_MARGIN", False, derivatives.SUPPORT_HULL_KINK_INVALID),
        ("E3_E4_HORIZONTAL", False, derivatives.SUPPORT_HULL_KINK_INVALID),
        ("E3_E4_REVERSAL", True, "SMOOTH_FIXED_ACTIVE_SET"),
    ],
)
def test_owner_callback_certificate_selection(monkeypatch, owner_id, valid, reason):
    base_certificate = _certificate()
    changed_certificate = _certificate(
        active_minimizer_set=((1, 2),),
        unique_active_minimizer=(1, 2),
    )
    active_set = ActiveSetFingerprint(
        contact_steps=(),
        prohibited_contact_steps=(),
        cop_valid_steps=(),
        support_active_steps=(),
        friction_steps=(),
        native_joint_limit_steps=(),
        drive_flag_steps=(),
        action_branches=("INTERIOR",) * 15,
    )

    def evaluation(certificate, value):
        return SimpleNamespace(
            active_set=active_set,
            owner_outputs={owner_id: np.asarray([value], dtype=np.float64)},
            support_margin_branch_steps=(certificate,),
            snapshot_digest="snapshot",
            next_snapshot_digest="next-snapshot",
        )

    evaluations = iter(
        [
            evaluation(base_certificate, 1.0),
            evaluation(base_certificate, 1.1),
            evaluation(changed_certificate, 1.2),
        ]
    )
    monkeypatch.setattr(derivatives, "_workspace", lambda plant: object())
    monkeypatch.setattr(
        derivatives,
        "evaluate_wrapped_step_5ms",
        lambda **kwargs: next(evaluations),
    )
    snapshot = SimpleNamespace(previous_accepted_action=np.zeros(15, dtype=np.float64))
    result = derivatives.differentiate_owner_output(
        plant=SimpleNamespace(model=object()),
        base_snapshot=snapshot,
        raw_action=np.full(15, 0.5, dtype=np.float64),
        owner_id=owner_id,
        state_steps={key: 1.0 for key in derivatives.STATE_STEP_BLOCKS},
        action_steps=0.1,
        state_columns=[],
        action_columns=[0],
        allow_nonsmooth=True,
    )
    report = result.action_columns[0]
    assert report.valid is valid
    assert report.reason == reason
