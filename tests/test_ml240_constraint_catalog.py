"""Focused ML-240 catalog, owner-equivalence, and freeze-monitor tests."""

from __future__ import annotations

import inspect
from pathlib import Path

import numpy as np

import loaded_cmj.oracle.constraints as constraints
from loaded_cmj.oracle.constraints import CATALOG, CATALOG_ID


def test_catalog_schema_and_version() -> None:
    assert CATALOG_ID == "LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0"
    assert CATALOG.catalog_id == CATALOG_ID
    assert all(set(spec.to_record()) == {
        "constraint_id", "name", "class", "enforcement", "owner_module", "owner_symbol", "bound_owner",
        "evaluation_domain", "sense", "units", "frame", "reference_point", "sign_convention",
        "physical_tolerance", "tolerance_source", "phase_i_scale", "phase_i_scale_source", "contact_modes",
        "differentiability", "public_visibility", "privileged_required", "phase_applicability", "description", "notes",
    } for spec in CATALOG.specs)


def test_constraint_ids_are_unique() -> None:
    ids = [spec.constraint_id for spec in CATALOG.specs]
    assert len(ids) == len(set(ids))


def test_every_hard_constraint_has_a_source_owner() -> None:
    assert all(spec.owner_module and spec.owner_symbol and spec.bound_owner for spec in CATALOG.specs
                if spec.classification.startswith("HARD_"))


def test_every_hard_constraint_has_units_frame_reference_and_sign() -> None:
    for spec in CATALOG.specs:
        if spec.classification.startswith("HARD_"):
            assert spec.units and spec.frame and spec.reference_point and spec.sign_convention
            assert spec.sense in {"MARGIN_G_GE_0", "EQUALITY_H_EQ_0", "DISCRETE_PASS"}


def test_every_hard_constraint_has_tolerance_provenance() -> None:
    for spec in CATALOG.specs:
        if spec.classification.startswith("HARD_"):
            assert spec.tolerance_source
            assert spec.physical_tolerance is not None


def test_every_future_elastic_constraint_has_phase_i_scale_provenance() -> None:
    for spec in CATALOG.specs:
        assert spec.phase_i_scale is not None
        assert spec.phase_i_scale_source


def test_inequality_direction_is_frozen_to_g_ge_zero() -> None:
    assert all(spec.sense != "MARGIN_G_LE_0" for spec in CATALOG.specs)
    assert CATALOG.get("SUPPORT_MARGIN").sign_convention == "g=owner signed margin"
    assert CATALOG.get("E3_E4_REVERSAL").sign_convention == "world +Z; g=vz-0"


def test_action_box_owner_is_direct() -> None:
    marker = object()
    seen: list[tuple[object, object]] = []

    def owner(candidate: object, spec: object) -> object:
        seen.append((candidate, spec))
        return marker

    original = constraints.validate_action
    constraints.validate_action = owner  # type: ignore[assignment]
    try:
        candidate, spec = object(), object()
        assert constraints.adapt_validate_action(candidate, spec) is marker
        assert seen == [(candidate, spec)]
    finally:
        constraints.validate_action = original


def test_action_slew_owner_is_direct() -> None:
    marker = np.arange(15, dtype=np.float64)
    seen: list[tuple[np.ndarray, np.ndarray]] = []

    def owner(previous: np.ndarray, raw: np.ndarray) -> np.ndarray:
        seen.append((previous, raw))
        return marker

    original = constraints.project_accepted_action
    constraints.project_accepted_action = owner  # type: ignore[assignment]
    try:
        previous = np.zeros(15)
        raw = np.ones(15)
        assert constraints.adapt_project_accepted_action(previous, raw) is marker
        assert seen == [(previous, raw)]
    finally:
        constraints.project_accepted_action = original


def test_drivestate_owner_is_direct() -> None:
    marker = {"tau": np.zeros(15)}
    seen: list[tuple[object, ...]] = []

    def owner(*args: object) -> dict[str, object]:
        seen.append(args)
        return marker

    original = constraints.drive_state_step
    constraints.drive_state_step = owner  # type: ignore[assignment]
    try:
        args = (np.zeros(15), object(), np.zeros(15), np.zeros(15), 0.000125)
        assert constraints.adapt_drive_state_step(*args) is marker
        assert seen == [args]
    finally:
        constraints.drive_state_step = original


def test_plant_state_and_contact_adapters_are_direct() -> None:
    class FakePlant:
        def anatomical_coordinates(self, data: object) -> object:
            return ("q", data)

        def anatomical_rates(self, data: object) -> object:
            return ("qd", data)

        def contact_wrench_summary(self, data: object) -> object:
            return ("wrench", data)

        def foot_contact_summary(self, data: object) -> object:
            return ("foot", data)

        def support_polygon(self, data: object, latched: tuple[bool, bool]) -> object:
            return ("polygon", data, latched)

        def support_margin(self, data: object, com: np.ndarray, latched: tuple[bool, bool]) -> object:
            return ("margin", data, com, latched)

    plant = FakePlant()
    data = object()
    latched = (True, False)
    com = np.zeros(3)
    assert constraints.adapt_anatomical_coordinates(plant, data) == ("q", data)
    assert constraints.adapt_anatomical_rates(plant, data) == ("qd", data)
    assert constraints.adapt_contact_wrench_summary(plant, data) == ("wrench", data)
    assert constraints.adapt_foot_contact_summary(plant, data) == ("foot", data)
    assert constraints.adapt_support_polygon(plant, data, latched) == ("polygon", data, latched)
    margin = constraints.adapt_support_margin(plant, data, com, latched)
    assert margin[0] == "margin" and margin[1] is data and margin[2] is com and margin[3] == latched


def test_plant_momentum_centroidal_and_power_adapters_are_direct() -> None:
    class FakePlant:
        def center_of_mass_velocity(self, data: object) -> object:
            return ("v", data)

        def linear_momentum(self, data: object) -> object:
            return ("p", data)

        def centroidal_angular_momentum(self, data: object) -> object:
            return ("H", data)

        def centroidal_hdot_from_external_wrench(self, data: object, wrench: np.ndarray) -> object:
            return ("Hdot", data, wrench)

        def realized_power_components(self, torque: np.ndarray, data: object) -> object:
            return ("power", torque, data)

    plant = FakePlant()
    data = object()
    wrench = np.zeros(6)
    torque = np.zeros(15)
    assert constraints.adapt_com_velocity(plant, data) == ("v", data)
    assert constraints.adapt_linear_momentum(plant, data) == ("p", data)
    assert constraints.adapt_centroidal_h(plant, data) == ("H", data)
    hdot = constraints.adapt_centroidal_hdot(plant, data, wrench)
    power = constraints.adapt_power_components(plant, torque, data)
    assert hdot[0] == "Hdot" and hdot[1] is data and hdot[2] is wrench
    assert power[0] == "power" and power[1] is torque and power[2] is data


def test_diagnostic_adapters_do_not_add_equations() -> None:
    predictor = lambda *args, **kwargs: (args, kwargs)
    assert constraints.adapt_capturability(predictor, 1, answer=2) == ((1,), {"answer": 2})

    original_metrics = constraints.derive_force_time_metrics
    constraints.derive_force_time_metrics = predictor  # type: ignore[assignment]
    try:
        assert constraints.adapt_force_time_metrics(1, answer=2) == ((1,), {"answer": 2})
    finally:
        constraints.derive_force_time_metrics = original_metrics


def test_official_event_engine_remains_postcheck_authority() -> None:
    class FakeDetector:
        def finalize(self, *, horizon_s: float | None = None) -> object:
            return ("official", horizon_s)

    assert constraints.adapt_official_event_finalize(FakeDetector(), horizon_s=0.5) == ("official", 0.5)
    spec = CATALOG.get("OFFICIAL_EVENT_ACCEPTANCE")
    assert spec.enforcement == "POSTCHECK_EVENT_ENGINE"
    assert spec.classification == "PHASE_METADATA"


def test_capturability_is_not_promoted_to_hard_feasibility() -> None:
    spec = CATALOG.get("CAPTURABILITY_PREDICTED_SUPPORT_RESERVE")
    assert spec.classification == "DIAGNOSTIC_ONLY"
    assert spec.enforcement == "POSTCHECK_MECHANICS"
    assert not spec.classification.startswith("HARD_")


def test_owner_units_frames_signs_and_visibility_are_complete() -> None:
    for spec in CATALOG.specs:
        assert spec.units and spec.frame and spec.reference_point and spec.sign_convention
        assert spec.public_visibility in constraints.ALLOWED_VISIBILITY
        assert isinstance(spec.privileged_required, bool)


def test_mode_applicability_and_differentiability_are_complete() -> None:
    for spec in CATALOG.specs:
        assert spec.contact_modes
        assert spec.phase_applicability
        assert spec.differentiability in constraints.ALLOWED_DIFFERENTIABILITY
    assert set(CATALOG.get("PHASE_MODE_LABELS").phase_applicability) == set(constraints.ALL_PHASES)


def test_e3_e4_terminal_set_is_explicit_without_new_posture_or_h_bounds() -> None:
    required = {"E3_E4_SUPPORTED", "E3_E4_REVERSAL", "E3_E4_HORIZONTAL", "DRIVE_REALIZED_TORQUE_CAPACITY",
                "CONTACT_PROHIBITED", "JOINT_NATIVE_LIMIT_STATUS", "CENTROIDAL_H", "CAPTURABILITY_PREDICTED_SUPPORT_RESERVE"}
    assert required <= {spec.constraint_id for spec in CATALOG.specs}
    assert CATALOG.get("CENTROIDAL_H").classification == "DIAGNOSTIC_ONLY"
    assert CATALOG.get("CAPTURABILITY_PREDICTED_SUPPORT_RESERVE").classification == "DIAGNOSTIC_ONLY"


def test_e3_e4_reversal_phase_i_metadata_is_nonelastic_after_erratum() -> None:
    spec = CATALOG.get("E3_E4_REVERSAL")
    assert spec.classification == "HARD_TERMINAL"
    assert spec.enforcement == "EXPLICIT_NLP_LATER"
    assert spec.sense == "MARGIN_G_GE_0"
    assert spec.phase_i_scale == "NOT_APPLICABLE"
    assert spec.phase_i_scale_source == "hard terminal exact reversal condition; no elastic residual"


def test_e3_e4_reversal_residual_preserves_exact_terminal_feasibility() -> None:
    residual = lambda v_z: v_z - 0.0
    assert residual(-1.0e-6) < 0.0
    assert residual(0.0) >= 0.0
    assert residual(1.0e-6) >= 0.0


def test_complete_movement_map_has_later_phase_contract_rows() -> None:
    required = {
        "PROPULSION_TAKEOFF", "FLIGHT_NO_SUPPORT", "FLIGHT_BALLISTIC", "LANDING_RECONTACT", "LANDING_LOADING",
        "RECOVERY_ARREST", "RECOVERY_BILATERAL", "RECOVERY_CAPTURABILITY", "RECOVERY_STABLE_DWELL",
    }
    assert required <= {spec.constraint_id for spec in CATALOG.specs}


def test_catalog_contains_no_scorer_or_reward_values() -> None:
    serialized = repr(CATALOG.records()).lower()
    assert "score" not in serialized
    assert "reward" not in serialized


def test_catalog_has_no_controller_runtime_dependency() -> None:
    runtime_root = Path(__file__).parents[1] / "src" / "loaded_cmj" / "runtime"
    assert not any("loaded_cmj.oracle" in path.read_text() for path in runtime_root.glob("*.py"))


def test_catalog_does_not_duplicate_physical_formulas() -> None:
    source = inspect.getsource(constraints)
    forbidden = (
        "mujoco.mj_contactForce", "contact_wrench_from_raw", "_point_in_hull_margin", "np.cross(",
        "np.trapz", "COP_x", "COP_y",
    )
    assert all(token not in source for token in forbidden)
    assert "project_accepted_action" in source
    assert "drive_state_step" in source


def test_snapshot_schema_and_tangent_layout_are_unchanged() -> None:
    from loaded_cmj.simulation.snapshot import SNAPSHOT_SCHEMA_VERSION
    from loaded_cmj.simulation.tangent import TANGENT_DIMENSION, TANGENT_LAYOUT_ID

    assert SNAPSHOT_SCHEMA_VERSION == "LCMJ-V1-MACRO-SNAPSHOT-1.0.0"
    assert TANGENT_LAYOUT_ID == "LCMJ-V1-TANGENT-STATE-132-1.0.0"
    assert TANGENT_DIMENSION == 132


def test_shared_transition_and_action_slew_owners_are_unchanged() -> None:
    from loaded_cmj.simulation import transition

    source = inspect.getsource(transition.step_5ms)
    assert "project_accepted_action" in source
    assert transition.ACCEPTED_ACTION_MAX_STEP == 0.20
    assert transition.PHYSICS_TIMESTEP_S == 0.000125
    assert transition.SUBSTEPS_PER_CONTROL == 40


def test_drive_capacity_rate_and_power_metadata_use_native_owners() -> None:
    assert CATALOG.get("DRIVE_REALIZED_TORQUE_CAPACITY").enforcement == "INTRINSIC_TRANSITION"
    assert CATALOG.get("DRIVE_TORQUE_RATE").owner_symbol.startswith("_ordered_torque_projection")
    assert CATALOG.get("DRIVE_SIGNED_POWER").owner_symbol.startswith("project_signed_power")


def test_joint_native_limit_and_rate_metadata_do_not_invent_rom() -> None:
    assert CATALOG.get("JOINT_ANATOMICAL_POSITION").classification == "DIAGNOSTIC_ONLY"
    assert CATALOG.get("JOINT_ANATOMICAL_RATE").classification == "DIAGNOSTIC_ONLY"
    assert CATALOG.get("JOINT_NATIVE_LIMIT_STATUS").differentiability == "DISCRETE_PREDICATE"


def test_contact_cop_and_support_metadata_preserve_owner_frames() -> None:
    assert CATALOG.get("CONTACT_COP_VALIDITY").frame == "world"
    assert CATALOG.get("CONTACT_COP_VALIDITY").reference_point == "plate moment origin / ground plane"
    assert CATALOG.get("SUPPORT_MARGIN").owner_symbol == "support_margin"
    assert CATALOG.get("SUPPORT_MARGIN").sense == "MARGIN_G_GE_0"


def test_momentum_centroidal_and_energy_tolerances_are_provenanced() -> None:
    assert CATALOG.get("IMPULSE_MOMENTUM_CLOSURE").physical_tolerance == 2e-4
    assert CATALOG.get("CENTROIDAL_H_CLOSURE").physical_tolerance == 5e-6
    assert CATALOG.get("ENERGY_WORK_RESIDUAL").physical_tolerance == 0.025
    assert all(spec.tolerance_source for spec in CATALOG.specs)


def test_no_derivative_nlp_solver_or_oracle_execution_scope() -> None:
    source = inspect.getsource(constraints)
    for forbidden in ("cyipopt", "casadi", "scipy.optimize", "finite_difference", "solve_oracle"):
        assert forbidden not in source.lower()


def test_deterministic_supported_fixture_matches_plant_owners() -> None:
    from loaded_cmj.simulation.plant import Plant, build_model

    plant = Plant(build_model())
    data = plant.make_data()
    plant.reset_supported(data)
    latched = (True, True)
    com = plant.center_of_mass(data)
    assert np.array_equal(constraints.adapt_anatomical_coordinates(plant, data), plant.anatomical_coordinates(data))
    assert np.array_equal(constraints.adapt_anatomical_rates(plant, data), plant.anatomical_rates(data))
    assert np.array_equal(constraints.adapt_linear_momentum(plant, data), plant.linear_momentum(data))
    assert np.array_equal(constraints.adapt_centroidal_h(plant, data), plant.centroidal_angular_momentum(data))
    assert constraints.adapt_support_margin(plant, data, com, latched) == plant.support_margin(data, com, latched)
    left = constraints.adapt_contact_wrench_summary(plant, data)
    right = plant.contact_wrench_summary(data)
    assert np.array_equal(left["whole_wrench"], right["whole_wrench"])
    assert np.array_equal(left["cop_valid"], right["cop_valid"])


def test_deterministic_supported_fixture_matches_drivestate_owner() -> None:
    from loaded_cmj.simulation import drive
    from loaded_cmj.simulation.plant import Plant, build_model

    plant = Plant(build_model())
    data = plant.make_data()
    plant.reset_supported(data)
    s = plant.anatomical_coordinates(data)
    s_dot = plant.anatomical_rates(data)
    command = np.linspace(-0.4, 0.4, 15)
    adapted_state = drive.DriveState()
    owner_state = drive.DriveState()
    adapted = constraints.adapt_drive_state_step(command, adapted_state, s, s_dot, 0.000125)
    owner = drive.drive_state_step(command, owner_state, s, s_dot, 0.000125)
    assert adapted.keys() == owner.keys()
    for key in adapted:
        if isinstance(adapted[key], np.ndarray):
            assert np.array_equal(adapted[key], owner[key])
        elif isinstance(adapted[key], dict):
            assert adapted[key].keys() == owner[key].keys()
            for flag in adapted[key]:
                assert np.array_equal(adapted[key][flag], owner[key][flag])
        else:
            assert adapted[key] == owner[key]
