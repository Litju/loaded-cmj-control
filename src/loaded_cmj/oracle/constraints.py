"""The narrow LCMJ V1 oracle constraint catalog.

This module is metadata plus transparent owner adapters.  It is deliberately
not a dynamics engine, event engine, symbolic expression system, or optimizer
interface.  Every executable adapter below calls an existing qualified owner
directly; it does not reconstruct the owner's physical quantity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from loaded_cmj.biomechanics.events import CMJEventDetector
from loaded_cmj.biomechanics.metrics import derive_force_time_metrics
from loaded_cmj.runtime.observations import validate_action
from loaded_cmj.simulation.drive import drive_state_step
from loaded_cmj.simulation.plant import Plant
from loaded_cmj.simulation.transition import project_accepted_action

CATALOG_ID = "LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0"

ALLOWED_CLASSES = frozenset(
    {
        "HARD_PATH",
        "HARD_GUARD",
        "HARD_TERMINAL",
        "DIAGNOSTIC_ONLY",
        "PHASE_METADATA",
    }
)
ALLOWED_ENFORCEMENTS = frozenset(
    {
        "INTRINSIC_TRANSITION",
        "EXPLICIT_NLP_LATER",
        "AGGREGATE_NLP_LATER",
        "POSTCHECK_EVENT_ENGINE",
        "POSTCHECK_MECHANICS",
        "METADATA_ONLY",
    }
)
ALLOWED_DIFFERENTIABILITY = frozenset(
    {
        "SMOOTH_MODE_LOCAL",
        "PIECEWISE_SMOOTH",
        "NONSMOOTH_CONTACT",
        "DISCRETE_PREDICATE",
        "INTERVAL_AGGREGATE",
        "POSTTRACE_ONLY",
        "NOT_APPLICABLE",
    }
)
ALLOWED_VISIBILITY = frozenset(
    {"PUBLIC_COMPATIBLE", "PRIVILEGED_ORACLE_ONLY", "POSTTRACE_EVALUATOR_ONLY"}
)
ALLOWED_SENSES = frozenset(
    {"MARGIN_G_GE_0", "EQUALITY_H_EQ_0", "DISCRETE_PASS", "NONE"}
)
ALL_PHASES = (
    "SUPPORTED",
    "COUNTERMOVEMENT",
    "BRAKING_REVERSAL",
    "PROPULSION",
    "FLIGHT",
    "LANDING_PREPARATION",
    "LANDING_CONTACT",
    "ABSORPTION",
    "RECOVERY",
)
ALL_CONTACT_MODES = ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION")


@dataclass(frozen=True)
class ConstraintSpec:
    """One source-owned catalog row; no executable expression is stored."""

    constraint_id: str
    name: str
    classification: str
    enforcement: str
    owner_module: str
    owner_symbol: str
    bound_owner: str
    evaluation_domain: str
    sense: str
    units: str
    frame: str
    reference_point: str
    sign_convention: str
    physical_tolerance: float | str | None
    tolerance_source: str
    phase_i_scale: float | str | None
    phase_i_scale_source: str
    contact_modes: tuple[str, ...]
    differentiability: str
    public_visibility: str
    privileged_required: bool
    phase_applicability: tuple[str, ...]
    description: str
    notes: str = ""

    def to_record(self) -> dict[str, Any]:
        """Return the stable machine-readable spelling used by the evidence."""

        return {
            "constraint_id": self.constraint_id,
            "name": self.name,
            "class": self.classification,
            "enforcement": self.enforcement,
            "owner_module": self.owner_module,
            "owner_symbol": self.owner_symbol,
            "bound_owner": self.bound_owner,
            "evaluation_domain": self.evaluation_domain,
            "sense": self.sense,
            "units": self.units,
            "frame": self.frame,
            "reference_point": self.reference_point,
            "sign_convention": self.sign_convention,
            "physical_tolerance": self.physical_tolerance,
            "tolerance_source": self.tolerance_source,
            "phase_i_scale": self.phase_i_scale,
            "phase_i_scale_source": self.phase_i_scale_source,
            "contact_modes": list(self.contact_modes),
            "differentiability": self.differentiability,
            "public_visibility": self.public_visibility,
            "privileged_required": self.privileged_required,
            "phase_applicability": list(self.phase_applicability),
            "description": self.description,
            "notes": self.notes,
        }


@dataclass(frozen=True)
class ConstraintValue:
    """Minimal result semantics for a direct owner adapter."""

    constraint_id: str
    feasible: bool | None
    margin_or_residual: Any = None
    raw_value: Any = None
    tolerance: float | str | None = None


class ConstraintCatalog:
    """Immutable indexed view of the one catalog; it has no plugin surface."""

    def __init__(self, specs: tuple[ConstraintSpec, ...], *, catalog_id: str = CATALOG_ID) -> None:
        if not specs:
            raise ValueError("constraint catalog cannot be empty")
        ids = [spec.constraint_id for spec in specs]
        if len(ids) != len(set(ids)):
            raise ValueError("constraint IDs must be unique")
        for spec in specs:
            if spec.classification not in ALLOWED_CLASSES:
                raise ValueError(f"invalid constraint class: {spec.classification}")
            if spec.enforcement not in ALLOWED_ENFORCEMENTS:
                raise ValueError(f"invalid enforcement: {spec.enforcement}")
            if spec.differentiability not in ALLOWED_DIFFERENTIABILITY:
                raise ValueError(f"invalid differentiability for {spec.constraint_id}: {spec.differentiability}")
            if spec.public_visibility not in ALLOWED_VISIBILITY:
                raise ValueError(f"invalid public visibility: {spec.public_visibility}")
            if spec.sense not in ALLOWED_SENSES:
                raise ValueError(f"invalid residual sense: {spec.sense}")
            if not spec.owner_module or not spec.owner_symbol or not spec.bound_owner:
                raise ValueError(f"source owner is incomplete: {spec.constraint_id}")
            if not spec.units or not spec.frame or not spec.reference_point:
                raise ValueError(f"units/frame/reference point are incomplete: {spec.constraint_id}")
            if not spec.sign_convention or not spec.tolerance_source:
                raise ValueError(f"sign/tolerance provenance is incomplete: {spec.constraint_id}")
            if spec.phase_i_scale is None or not spec.phase_i_scale_source:
                raise ValueError(f"Phase-I scale provenance is incomplete: {spec.constraint_id}")
            if not spec.contact_modes or not spec.phase_applicability:
                raise ValueError(f"mode applicability is incomplete: {spec.constraint_id}")
        self.catalog_id = catalog_id
        self.specs = tuple(specs)
        self._by_id = {spec.constraint_id: spec for spec in self.specs}

    def get(self, constraint_id: str) -> ConstraintSpec:
        return self._by_id[constraint_id]

    def records(self) -> list[dict[str, Any]]:
        return [spec.to_record() for spec in self.specs]


def _spec(
    constraint_id: str,
    name: str,
    classification: str,
    enforcement: str,
    owner_module: str,
    owner_symbol: str,
    bound_owner: str,
    evaluation_domain: str,
    sense: str,
    units: str,
    frame: str,
    reference_point: str,
    sign_convention: str,
    physical_tolerance: float | str | None,
    tolerance_source: str,
    phase_i_scale: float | str | None,
    phase_i_scale_source: str,
    contact_modes: tuple[str, ...],
    differentiability: str,
    public_visibility: str,
    privileged_required: bool,
    phase_applicability: tuple[str, ...],
    description: str,
    notes: str = "",
) -> ConstraintSpec:
    return ConstraintSpec(
        constraint_id=constraint_id,
        name=name,
        classification=classification,
        enforcement=enforcement,
        owner_module=owner_module,
        owner_symbol=owner_symbol,
        bound_owner=bound_owner,
        evaluation_domain=evaluation_domain,
        sense=sense,
        units=units,
        frame=frame,
        reference_point=reference_point,
        sign_convention=sign_convention,
        physical_tolerance=physical_tolerance,
        tolerance_source=tolerance_source,
        phase_i_scale=phase_i_scale,
        phase_i_scale_source=phase_i_scale_source,
        contact_modes=contact_modes,
        differentiability=differentiability,
        public_visibility=public_visibility,
        privileged_required=privileged_required,
        phase_applicability=phase_applicability,
        description=description,
        notes=notes,
    )


_POLICY = "src/loaded_cmj/runtime/policy_spec.py; src/loaded_cmj/runtime/observations.py"
_TRANSITION = "src/loaded_cmj/simulation/transition.py"
_DRIVE = "src/loaded_cmj/simulation/drive.py"
_PLANT = "src/loaded_cmj/simulation/plant.py"
_EVENTS = "src/loaded_cmj/biomechanics/events.py"
_METRICS = "src/loaded_cmj/biomechanics/metrics.py"
_RESULTS = "src/loaded_cmj/runtime/results.py"

_SPECS = (
    _spec(
        "ACTION_RAW_BOX", "raw/public action box", "HARD_PATH", "INTRINSIC_TRANSITION", _POLICY,
        "PolicySpec.action + validate_action", "policy_spec.json action ValueSpec [-1, 1]", "state",
        "MARGIN_G_GE_0", "1", "normalized action coordinates", "none", "upper-value/lower-value",
        "EXACT", "policy specification bounds_behavior=reject", 2.0, "policy action range width",
        ("SUPPORTED_CONTACT", "FLIGHT"), "SMOOTH_MODE_LOCAL", "PUBLIC_COMPATIBLE", False, ALL_PHASES,
        "Validated raw action is within the public normalized action box.",
    ),
    _spec(
        "ACTION_ACCEPTED_SLEW", "accepted-action slew", "HARD_PATH", "INTRINSIC_TRANSITION", _TRANSITION,
        "project_accepted_action / ACCEPTED_ACTION_MAX_STEP", "ACCEPTED_ACTION_MAX_STEP=0.20 per control interval",
        "transition", "MARGIN_G_GE_0", "1", "normalized action coordinates", "none", "upper-absolute delta",
        "EXACT", "shared transition owner and ML-237 amendment", 0.20, "ACCEPTED_ACTION_MAX_STEP",
        ("SUPPORTED_CONTACT", "FLIGHT"), "SMOOTH_MODE_LOCAL", "PUBLIC_COMPATIBLE", False, ALL_PHASES,
        "Accepted action is the exact transition projection of raw action against the previous accepted action.",
        "No catalog slew equation is allowed.",
    ),
    _spec(
        "DRIVE_STATE_VALID", "finite valid hidden drive state", "HARD_PATH", "INTRINSIC_TRANSITION", _DRIVE,
        "DriveState._validate / drive_state_step", "DriveState validation", "transition", "DISCRETE_PASS", "1",
        "anatomical channels", "none", "pass=True", "EXACT", "DriveState._validate", "NOT_APPLICABLE",
        "discrete predicate; no elastic residual", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY",
        True, ALL_PHASES, "Hidden DriveState remains finite and internally valid.",
    ),
    _spec(
        "DRIVE_ACTIVATION_RANGE", "activation state range", "HARD_PATH", "INTRINSIC_TRANSITION", _DRIVE,
        "activation_update / DriveState._validate", "DriveState validation", "transition", "MARGIN_G_GE_0", "1",
        "anatomical channels", "none", "value-lower/upper-value", "EXACT", "DriveState._validate [0,1]",
        1.0, "activation native range", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY", True,
        ALL_PHASES, "Drive activation remains in the native state range.",
    ),
    _spec(
        "DRIVE_REALIZED_TORQUE_CAPACITY", "realized torque capacity", "HARD_PATH", "INTRINSIC_TRANSITION", _DRIVE,
        "capacity_envelope / drive_state_step capacity_lower/upper", "TAU_BAR_POS/NEG and F3 DriveState qualification",
        "transition", "MARGIN_G_GE_0", "N m", "anatomical joint axes", "joint actuator reference", "upper-value/lower-value",
        "EXACT_OWNER_PROJECTION", "drive capacity envelope and ordered projection", "owner-derived",
        "TAU_BAR_MAX from drive.py", ALL_CONTACT_MODES, "PIECEWISE_SMOOTH", "PRIVILEGED_ORACLE_ONLY", True,
        ALL_PHASES, "Realized torque is the production DriveState output inside its signed capacity envelope.",
    ),
    _spec(
        "DRIVE_TORQUE_RATE", "realized torque rate", "HARD_PATH", "INTRINSIC_TRANSITION", _DRIVE,
        "_ordered_torque_projection / drive_state_step", "RATE_MAX and PHYSICS_TIMESTEP_S", "transition", "MARGIN_G_GE_0",
        "N m/s", "anatomical joint axes", "joint actuator reference", "upper-absolute rate delta",
        "EXACT_OWNER_PROJECTION", "ordered DriveState projection", "owner-derived", "RATE_MAX from drive.py",
        ALL_CONTACT_MODES, "PIECEWISE_SMOOTH", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Torque rate behavior is intrinsic to the substep projection.",
    ),
    _spec(
        "DRIVE_SIGNED_POWER", "signed actuator power limit", "HARD_PATH", "INTRINSIC_TRANSITION", _DRIVE,
        "project_signed_power / drive_state_step", "POWER_POS/NEG", "transition", "MARGIN_G_GE_0", "W",
        "anatomical joint axes", "joint actuator reference", "owner signed-power projection", "EXACT_OWNER_PROJECTION",
        "DriveState signed power projection", "owner-derived", "POWER_POS/NEG from drive.py", ALL_CONTACT_MODES,
        "PIECEWISE_SMOOTH", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Realized torque obeys the native signed power projection.",
    ),
    _spec(
        "DRIVE_OVERRIDE_FLAGS", "actuator projection flags", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _DRIVE,
        "drive_state_step override_flags", "DriveState projection flags", "transition", "DISCRETE_PASS", "1",
        "anatomical joint axes", "joint actuator reference", "pass=True", "EXACT", "DriveState flags",
        "NOT_APPLICABLE", "flag diagnostic; no elastic residual", ALL_CONTACT_MODES, "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES, "Projection-stage flags are diagnostics, not violations.",
    ),
    _spec(
        "DRIVE_ACTIVE_WORK", "active positive/negative work", "DIAGNOSTIC_ONLY", "AGGREGATE_NLP_LATER", _PLANT,
        "realized_power_components", "F3 energy/work contract", "interval", "NONE", "J", "world energy scalar",
        "whole system", "signed owner work", "0.025 J only for closure, not work objective", "F3 54_MECHANICS_TOLERANCE_CONTRACT.md",
        "owner-derived", "realized power ledger", ALL_CONTACT_MODES, "INTERVAL_AGGREGATE", "PRIVILEGED_ORACLE_ONLY", True,
        ALL_PHASES, "Active work ledger, separated into positive and negative components.",
    ),
    _spec(
        "DRIVE_PASSIVE_WORK", "passive damping/limit work", "DIAGNOSTIC_ONLY", "AGGREGATE_NLP_LATER", _PLANT,
        "realized_power_components", "F3 energy/work contract", "interval", "NONE", "J", "world energy scalar",
        "whole system", "signed owner work", "0.025 J only for closure, not work objective", "F3 energy/work contract",
        "owner-derived", "realized power ledger", ALL_CONTACT_MODES, "INTERVAL_AGGREGATE", "PRIVILEGED_ORACLE_ONLY", True,
        ALL_PHASES, "Passive damping and native-limit work ledger.",
    ),
    _spec(
        "DRIVE_NATIVE_LIMIT_WORK", "native-limit work", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "BiomechanicalSample native-limit work", "F3 energy/work contract", "post_trace", "NONE", "J",
        "world energy scalar", "whole system", "signed owner work", "0.025 J closure only", "F3 energy/work contract",
        "NOT_APPLICABLE", "diagnostic; no frozen dominance scale", ALL_CONTACT_MODES, "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Native-limit work is reported but cannot become a primary actuator.",
    ),
    _spec(
        "JOINT_ANATOMICAL_POSITION", "anatomical joint position", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "anatomical_coordinates", "compiled MJCF joint ranges where applicable", "state", "NONE", "rad",
        "anatomical joint axes", "model joint origin", "owner coordinate convention", "EXACT_OWNER_VALUE",
        "Plant anatomical coordinate owner; MJCF range source", "owner-derived", "loaded_jump_athlete.xml",
        ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PUBLIC_COMPATIBLE", False, ALL_PHASES,
        "Canonical anatomical coordinate output; no inferred human-ROM bound is added.",
    ),
    _spec(
        "JOINT_ANATOMICAL_RATE", "anatomical joint rate", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "anatomical_rates", "compiled model/native authority", "state", "NONE", "rad/s", "anatomical joint axes",
        "model joint origin", "owner coordinate convention", "EXACT_OWNER_VALUE", "Plant anatomical rate owner",
        "owner-derived", "compiled model authority", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PUBLIC_COMPATIBLE", False,
        ALL_PHASES, "Canonical anatomical rate output; no approximate rate limit is invented.",
    ),
    _spec(
        "JOINT_NATIVE_LIMIT_STATUS", "native joint-limit status", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "compiled MJCF + trusted data.efc_type", "MJCF model authority", "post_trace", "DISCRETE_PASS", "1",
        "native constraint coordinates", "native joint frames", "pass=True", "EXACT", "MJCF and MuJoCo constraint state",
        "NOT_APPLICABLE", "discrete predicate; no elastic residual", ALL_CONTACT_MODES, "DISCRETE_PREDICATE",
        "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES, "Native constraint activity is audited, not used as an actuator loophole.",
    ),
    _spec(
        "CONTACT_PERMITTED_SUPPORT", "permitted designated support contact", "HARD_GUARD", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary / foot_contact_summary", "CONTACT_F_ON_N and contact latch constants", "transition",
        "DISCRETE_PASS", "1", "world", "plate origins", "pass=True", "EXACT", "contact owner and F3 latch contract",
        "NOT_APPLICABLE", "discrete contact predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "Designated support contact must be valid in supported modes.",
    ),
    _spec(
        "CONTACT_PROHIBITED", "prohibited contact absent", "HARD_GUARD", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary prohibited_contact", "contact geometry classification", "transition", "DISCRETE_PASS", "1",
        "world", "contact point", "pass=True", "EXACT", "Plant contact owner", "NOT_APPLICABLE",
        "discrete contact predicate", ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE",
        "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES, "Prohibited contact is a hard hybrid predicate.",
    ),
    _spec(
        "CONTACT_OFF_PLATFORM", "off-platform contact absent", "HARD_GUARD", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary contact region/plate row", "Plant contact geometry", "transition", "DISCRETE_PASS", "1",
        "world", "contact point", "pass=True", "EXACT", "Plant contact owner", "NOT_APPLICABLE",
        "discrete contact predicate", ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE",
        "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES, "Off-platform contact remains visible and is not discarded from whole wrench.",
    ),
    _spec(
        "CONTACT_NORMAL_FORCE_VALIDITY", "normal force validity", "HARD_PATH", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary normal_force", "CONTACT_F_ACTIVE_N / COP threshold", "transition", "MARGIN_G_GE_0", "N",
        "world", "plate moment origin", "+Z is plate/world normal", "EXACT_OWNER_VALUE", "Plant contact owner",
        "owner-derived", "bodyweight reference 931.95 N", ALL_CONTACT_MODES, "NONSMOOTH_CONTACT", "PRIVILEGED_ORACLE_ONLY", True,
        ALL_PHASES, "Normal force validity is read from the bilateral plate owner.",
    ),
    _spec(
        "CONTACT_FRICTION", "contact friction feasibility", "HARD_PATH", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary friction_feasible", "compiled geom friction", "transition", "DISCRETE_PASS", "1",
        "world", "contact point", "pass=True", "EXACT", "Plant friction owner and compiled MJCF", "NOT_APPLICABLE",
        "discrete contact predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "Friction feasibility is the existing contact owner result.",
    ),
    _spec(
        "CONTACT_COP_VALIDITY", "COP validity", "HARD_PATH", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary cop_valid", "CONTACT_F_COP_MIN_N=20 N", "transition", "DISCRETE_PASS", "1",
        "world", "plate moment origin / ground plane", "+Fz; owner COP signs", "EXACT", "F3 COP validation and source constant",
        "NOT_APPLICABLE", "force threshold predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "COP is valid only for a sufficiently loaded designated plate.",
    ),
    _spec(
        "CONTACT_COP_SUPPORT_GEOMETRY", "COP support geometry", "HARD_PATH", "EXPLICIT_NLP_LATER", _PLANT,
        "cop_world_xy + support_margin", "Plant support polygon/margin owner", "transition", "MARGIN_G_GE_0", "m",
        "world", "plate origin / ground plane", "support margin g>=0", "EXACT_OWNER_VALUE", "Plant support/COP owners",
        "owner-derived", "support polygon extent from Plant", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "COP/support geometry uses the existing support-margin owner.",
    ),
    _spec(
        "SUPPORT_POLYGON", "latched support polygon", "HARD_PATH", "EXPLICIT_NLP_LATER", _PLANT,
        "support_polygon", "compiled pad geometry and support latch", "state", "MARGIN_G_GE_0", "m",
        "world ground XY", "ground plane", "inside owner hull", "EXACT_OWNER_VALUE", "Plant support polygon owner",
        "owner-derived", "pad geometry extent from Plant", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "Support polygon is Plant-owned and latch-aware.",
    ),
    _spec(
        "SUPPORT_MARGIN", "signed support margin", "HARD_PATH", "EXPLICIT_NLP_LATER", _PLANT,
        "support_margin", "Plant support polygon and geometry", "state", "MARGIN_G_GE_0", "m", "world ground XY",
        "COM vertical projection", "g=owner signed margin", "EXACT_OWNER_VALUE", "Plant support_margin owner",
        "owner-derived", "support polygon extent from Plant", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ("SUPPORTED", "BRAKING_REVERSAL", "LANDING_CONTACT", "ABSORPTION", "RECOVERY"),
        "COM support reserve is the canonical continuous support residual.",
    ),
    _spec(
        "CAPTURABILITY_PREDICTED_SUPPORT_RESERVE", "predicted support reserve diagnostic", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS",
        "src/loaded_cmj/biomechanics/capturability.py", "predict_support_reserve", "F3 capturability structural contract",
        "interval", "NONE", "m", "world sagittal X / vertical Z", "COM/support geometry", "signed diagnostic report",
        "NO_SCALAR_TOLERANCE", "F3 capturability evidence: structural/finite checks only", "NOT_APPLICABLE",
        "diagnostic; no elastic feasibility residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ("COUNTERMOVEMENT", "BRAKING_REVERSAL", "PROPULSION", "RECOVERY"),
        "Approximate finite-horizon support reserve; never a hard reachability restriction.",
    ),
    _spec(
        "COM_VELOCITY", "whole-system COM velocity", "DIAGNOSTIC_ONLY", "EXPLICIT_NLP_LATER", _PLANT,
        "center_of_mass_velocity", "Plant body mass ownership", "state", "NONE", "m/s", "world", "whole-system COM",
        "world +Z vertical", "EXACT_OWNER_VALUE", "Plant COM owner", "owner-derived", "bodyweight/mass model reference",
        ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Trusted COM velocity value; phase guards may reference its vertical component.",
    ),
    _spec(
        "LINEAR_MOMENTUM", "whole-system linear momentum", "DIAGNOSTIC_ONLY", "EXPLICIT_NLP_LATER", _PLANT,
        "linear_momentum", "Plant total mass and F3 momentum contract", "state", "NONE", "kg m/s", "world",
        "whole-system COM", "world axes", "EXACT_OWNER_VALUE", "Plant linear_momentum owner", "owner-derived",
        "bodyweight/mass model reference", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Trusted linear momentum; no arbitrary magnitude bound is introduced.",
    ),
    _spec(
        "FORCE_TIME_METRICS", "force-time metrics", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _METRICS,
        "derive_force_time_metrics", "F3 event interval and metric contract", "post_trace", "NONE", "N, N s, N/s",
        "world", "plate origins", "owner metric signs", "EXACT_OWNER_VALUE", "canonical metrics owner", "NOT_APPLICABLE",
        "interval aggregate; no elastic residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ("PROPULSION", "LANDING_CONTACT", "ABSORPTION"),
        "Canonical propulsion/landing metric projection; no second impulse integrator.",
    ),
    _spec(
        "IMPULSE_MOMENTUM_CLOSURE", "impulse-momentum closure", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "linear_momentum + qualified external-wrench trace", "F3 tolerance 2e-4 kg m/s", "post_trace", "EQUALITY_H_EQ_0",
        "kg m/s", "world", "whole system / owner wrench origin", "h=delta p-integral impulse", 2e-4,
        "54_MECHANICS_TOLERANCE_CONTRACT.md linear impulse closure", 2e-4, "accepted physical closure tolerance",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE", "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES,
        "Qualified trace closure diagnostic with an explicit F3 tolerance.",
    ),
    _spec(
        "JZ_TOGO", "vertical impulse-to-go diagnostic", "DIAGNOSTIC_ONLY", "AGGREGATE_NLP_LATER",
        _PLANT, "frozen J_z,togo relationship over existing Plant/trace owners",
        "architecture mathematical contract", "interval", "NONE", "N s", "world", "whole-system COM", "world +Z signed impulse",
        "NO_HARD_BOUND", "architecture contract; no threshold", "NOT_APPLICABLE", "diagnostic only; no elastic residual",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE", "PRIVILEGED_ORACLE_ONLY", True, ("PROPULSION",),
        "Architecture-level impulse-to-go diagnostic; not a second plant or hard feasibility margin.",
    ),
    _spec(
        "JZ_REACH", "vertical reachable impulse diagnostic", "DIAGNOSTIC_ONLY", "AGGREGATE_NLP_LATER",
        f"{_DRIVE}; {_PLANT}", "frozen J_z,reach relationship over existing DriveState/Plant owners",
        "architecture mathematical contract", "interval", "NONE", "N s", "world", "whole-system COM", "world +Z signed impulse",
        "NO_HARD_BOUND", "architecture contract; no threshold", "NOT_APPLICABLE", "diagnostic only; no elastic residual",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE", "PRIVILEGED_ORACLE_ONLY", True, ("PROPULSION",),
        "Architecture-level reachable impulse diagnostic; no optimizer solve is performed.",
    ),
    _spec(
        "M_JZ", "vertical impulse gap diagnostic", "DIAGNOSTIC_ONLY", "AGGREGATE_NLP_LATER",
        f"{_DRIVE}; {_PLANT}", "frozen M_Jz relationship from existing J_z,togo/J_z,reach owners",
        "architecture mathematical contract", "interval", "NONE", "N s", "world", "whole-system COM", "signed gap report",
        "NO_HARD_BOUND", "architecture contract; no threshold", "NOT_APPLICABLE", "diagnostic only; no elastic residual",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE", "PRIVILEGED_ORACLE_ONLY", True, ("PROPULSION",),
        "Impulse gap diagnostic only.",
    ),
    _spec(
        "CENTROIDAL_H", "centroidal angular momentum", "DIAGNOSTIC_ONLY", "EXPLICIT_NLP_LATER", _PLANT,
        "centroidal_angular_momentum", "F3 centroidal wrench contract", "state", "NONE", "kg m2/s", "world",
        "instantaneous whole-system COM", "world axes", "EXACT_OWNER_VALUE", "Plant centroidal H owner", "owner-derived",
        "mass/inertia model reference", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Trusted H about the instantaneous whole-system COM; no arbitrary magnitude cap.",
    ),
    _spec(
        "CENTROIDAL_HDOT", "external-wrench centroidal Hdot", "DIAGNOSTIC_ONLY", "EXPLICIT_NLP_LATER", _PLANT,
        "centroidal_hdot_from_external_wrench", "F3 centroidal wrench contract", "state", "NONE", "N m", "world",
        "instantaneous whole-system COM", "world cross-product sign", "EXACT_OWNER_VALUE", "Plant Hdot owner", "owner-derived",
        "mass/inertia model reference", ALL_CONTACT_MODES, "SMOOTH_MODE_LOCAL", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Trusted external-wrench Hdot about COM; no arbitrary magnitude cap.",
    ),
    _spec(
        "CENTROIDAL_H_CLOSURE", "centroidal H closure", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "centroidal H/Hdot + qualified trace", "F3 5e-6 kg m2/s non-impact closure", "post_trace", "EQUALITY_H_EQ_0",
        "kg m2/s", "world", "whole-system COM", "h=delta H-integral Hdot", 5e-6,
        "54_MECHANICS_TOLERANCE_CONTRACT.md centroidal closure", 5e-6, "accepted H closure tolerance",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE", "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES,
        "Qualified centroidal closure check; contact-transition window uses its separate contract.",
    ),
    _spec(
        "ENERGY_WORK_RESIDUAL", "energy/work closure residual", "HARD_TERMINAL", "POSTCHECK_MECHANICS", _PLANT,
        "BiomechanicalSample energy ledger + qualified trace", "F3 tolerance 0.025 J", "post_trace", "EQUALITY_H_EQ_0", "J",
        "world", "whole system", "h=energy residual", 0.025, "54_MECHANICS_TOLERANCE_CONTRACT.md energy/work closure",
        0.025, "accepted physical energy tolerance", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Qualified energy/work closure acceptance; not an objective.",
    ),
    _spec(
        "ENERGY_FINITE", "finite energy/work ledger", "HARD_PATH", "POSTCHECK_MECHANICS", _PLANT,
        "BiomechanicalSample energy fields", "F3 energy/work contract", "post_trace", "DISCRETE_PASS", "J/W",
        "world", "whole system", "pass=True", "EXACT", "F3 energy qualification", "NOT_APPLICABLE",
        "discrete finite predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Nonfinite energy/work invalidates a trusted trace.",
    ),
    _spec(
        "ACTIVE_POWER", "realized active power", "DIAGNOSTIC_ONLY", "EXPLICIT_NLP_LATER", _PLANT,
        "realized_power_components", "DriveState signed power is intrinsic", "transition", "NONE", "W",
        "anatomical joint axes", "joint actuator reference", "owner signed value", "EXACT_OWNER_VALUE",
        "Plant realized power owner", "owner-derived", "DriveState POWER_POS/NEG", ALL_CONTACT_MODES, "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES, "Active power is reported from realized torque and Plant-owned rates.",
    ),
    _spec(
        "ACTIVE_WORK_POS", "positive active work", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "realized_power_components + sample ledger", "F3 energy/work contract", "post_trace", "NONE", "J", "world",
        "whole system", "positive work report", "EXACT_OWNER_VALUE", "Plant/sample energy owner", "NOT_APPLICABLE",
        "diagnostic interval; no elastic residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Positive active work is a diagnostic, not a low-energy feasibility objective.",
    ),
    _spec(
        "ACTIVE_WORK_NEG", "negative active work", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "realized_power_components + sample ledger", "F3 energy/work contract", "post_trace", "NONE", "J", "world",
        "whole system", "negative work report", "EXACT_OWNER_VALUE", "Plant/sample energy owner", "NOT_APPLICABLE",
        "diagnostic interval; no elastic residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Negative active work is a diagnostic.",
    ),
    _spec(
        "PASSIVE_WORK", "passive damping work", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "realized_power_components + sample ledger", "F3 energy/work contract", "post_trace", "NONE", "J", "world",
        "whole system", "passive work report", "EXACT_OWNER_VALUE", "Plant/sample energy owner", "NOT_APPLICABLE",
        "diagnostic interval; no elastic residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Passive work is reported without a fabricated threshold.",
    ),
    _spec(
        "NATIVE_LIMIT_WORK", "native-limit work dominance", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "BiomechanicalSample native-limit work", "F3 energy/work contract", "post_trace", "NONE", "J", "world",
        "whole system", "native-limit work report", "EXACT_OWNER_VALUE", "Plant/sample energy owner", "NOT_APPLICABLE",
        "diagnostic; no dominance threshold", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Native-limit work is audited; no ad-hoc non-dominance number is added.",
    ),
    _spec(
        "OFFICIAL_EVENT_ACCEPTANCE", "official event acceptance", "PHASE_METADATA", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector.finalize", "EventThresholds and F3 event DAG", "post_trace", "NONE", "1", "world sample semantics",
        "trace samples", "pass=True", "EXACT", "CMJEventDetector is sole event truth", "NOT_APPLICABLE",
        "post-trace event predicate", ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION"), "POSTTRACE_ONLY",
        "POSTTRACE_EVALUATOR_ONLY", True, ALL_PHASES, "Official dwell/order acceptance remains outside oracle Markov state.",
    ),
    _spec(
        "E3_E4_SUPPORTED", "E3->E4 supported contact", "HARD_TERMINAL", "POSTCHECK_MECHANICS", _PLANT,
        "contact_wrench_summary + support_margin", "F3 contact latch and support owner", "post_trace", "DISCRETE_PASS", "1",
        "world", "plate/support geometry", "pass=True", "EXACT", "Plant contact/support owners", "NOT_APPLICABLE",
        "discrete terminal predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT",
        "PRIVILEGED_ORACLE_ONLY", True, ("COUNTERMOVEMENT", "BRAKING_REVERSAL"),
        "E3->E4 target requires valid supported geometry; official upward_reversal remains evaluator truth.",
    ),
    _spec(
        "E3_E4_REVERSAL", "E3->E4 nonnegative COM vertical velocity", "HARD_TERMINAL", "EXPLICIT_NLP_LATER", _PLANT,
        "center_of_mass_velocity", "event reversal contract", "transition", "MARGIN_G_GE_0", "m/s", "world",
        "whole-system COM", "world +Z; g=vz-0", "EXACT", "event contract; crossing remains CMJEventDetector",
        "NOT_APPLICABLE", "hard terminal exact reversal condition; no elastic residual", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "PIECEWISE_SMOOTH",
        "PRIVILEGED_ORACLE_ONLY", True, ("BRAKING_REVERSAL",),
        "Continuous reversal relationship, not a copied event/dwell detector.",
    ),
    _spec(
        "E3_E4_HORIZONTAL", "E3->E4 horizontal support admissibility", "HARD_TERMINAL", "EXPLICIT_NLP_LATER", _PLANT,
        "support_margin + COM state", "Plant support polygon owner; no frozen capturability scalar bound", "transition",
        "MARGIN_G_GE_0", "m", "world ground XY", "COM projection", "owner support margin g>=0", "EXACT_OWNER_VALUE",
        "Plant support margin; predicted reserve stays diagnostic", "owner-derived", "support polygon extent from Plant",
        ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "PIECEWISE_SMOOTH", "PRIVILEGED_ORACLE_ONLY", True,
        ("BRAKING_REVERSAL",), "Exact support geometry is terminal; approximate stopping reserve is not.",
    ),
    _spec(
        "PROPULSION_TAKEOFF", "propulsion/takeoff guard", "HARD_GUARD", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector + Plant.center_of_mass_velocity", "EventThresholds/contact latch", "transition", "DISCRETE_PASS", "1",
        "world", "COM and plates", "pass=True; vz sign owner", "EXACT", "official takeoff event contract", "NOT_APPLICABLE",
        "event predicate", ("SUPPORTED_CONTACT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT", "PRIVILEGED_ORACLE_ONLY", True,
        ("PROPULSION",), "Positive COM vertical velocity and legitimate support release use event-owner semantics.",
    ),
    _spec(
        "FLIGHT_NO_SUPPORT", "flight has no support contact", "HARD_PATH", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector + Plant.contact_wrench_summary", "contact latch exact predicate", "transition", "DISCRETE_PASS", "1",
        "world", "plate/contact geometry", "pass=True", "EXACT", "contact/event owner", "NOT_APPLICABLE",
        "discrete contact predicate", ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION"), "NONSMOOTH_CONTACT",
        "PRIVILEGED_ORACLE_ONLY", True, ("FLIGHT",), "Flight must not contain hidden support contact.",
    ),
    _spec(
        "FLIGHT_BALLISTIC", "flight ballistic COM consistency", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _PLANT,
        "COM acceleration + qualified trace", "F3 ballistic acceleration tolerance 5e-3 m/s2", "post_trace",
        "EQUALITY_H_EQ_0", "m/s2", "world", "whole-system COM", "h=acceleration+gravity residual", 5e-3,
        "54_MECHANICS_TOLERANCE_CONTRACT.md ballistic COM", 5e-3, "accepted ballistic residual tolerance",
        ("FLIGHT",), "INTERVAL_AGGREGATE", "POSTTRACE_EVALUATOR_ONLY", True, ("FLIGHT",),
        "Flight ballistic consistency is a qualified diagnostic/postcheck.",
    ),
    _spec(
        "LANDING_RECONTACT", "descending permitted recontact", "HARD_GUARD", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector official landing event", "EventThresholds and contact latch", "transition", "DISCRETE_PASS", "1",
        "world", "plate/contact geometry", "pass=True", "EXACT", "CMJEventDetector event owner", "NOT_APPLICABLE",
        "discrete event predicate", ("FLIGHT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE", "POSTTRACE_EVALUATOR_ONLY", True,
        ("LANDING_PREPARATION", "LANDING_CONTACT"), "Landing is accepted only through the official descending recontact event.",
    ),
    _spec(
        "LANDING_LOADING", "finite landing loading", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS", _METRICS,
        "derive_force_time_metrics + event intervals", "F3 force impulse/peak convergence", "post_trace", "NONE", "N/Ns",
        "world", "plate origins", "owner metric sign", "EXACT_OWNER_VALUE", "metrics/event owners", "NOT_APPLICABLE",
        "interval aggregate; no invented loading threshold", ("CONTACT_TRANSITION",), "INTERVAL_AGGREGATE",
        "POSTTRACE_EVALUATOR_ONLY", True, ("LANDING_CONTACT", "ABSORPTION"), "Finite loading and window semantics stay post-trace.",
    ),
    _spec(
        "RECOVERY_ARREST", "recovery COM arrest", "DIAGNOSTIC_ONLY", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector recovery result + Plant COM", "no frozen scalar arrest bound", "post_trace", "NONE", "m/s",
        "world", "whole-system COM", "event-owned near-zero condition", "NO_FROZEN_THRESHOLD", "no scalar threshold in authority",
        "NOT_APPLICABLE", "post-trace diagnostic", ("SUPPORTED_CONTACT",), "POSTTRACE_ONLY", "POSTTRACE_EVALUATOR_ONLY", True,
        ("RECOVERY",), "No ad-hoc near-zero arrest threshold is added.",
    ),
    _spec(
        "RECOVERY_BILATERAL", "recovery bilateral support", "DIAGNOSTIC_ONLY", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector recovery result + Plant support", "contact latch exact predicate", "post_trace", "DISCRETE_PASS", "1",
        "world", "plate origins", "pass=True", "EXACT", "event/contact owners", "NOT_APPLICABLE", "discrete predicate",
        ("SUPPORTED_CONTACT",), "NONSMOOTH_CONTACT", "POSTTRACE_EVALUATOR_ONLY", True, ("RECOVERY",),
        "Bilateral recovery is official event acceptance metadata, not duplicated runtime logic.",
    ),
    _spec(
        "RECOVERY_BODYWEIGHT", "recovery bodyweight support", "DIAGNOSTIC_ONLY", "POSTCHECK_EVENT_ENGINE", _PLANT,
        "normal_force + CMJEventDetector result", "no frozen scalar threshold beyond event contract", "post_trace", "NONE", "N",
        "world", "plate origins", "owner force report", "NO_FROZEN_THRESHOLD", "no scalar threshold in authority", "NOT_APPLICABLE",
        "post-trace diagnostic", ("SUPPORTED_CONTACT",), "POSTTRACE_ONLY", "POSTTRACE_EVALUATOR_ONLY", True, ("RECOVERY",),
        "Bodyweight support is reported only if the event authority supplies it.",
    ),
    _spec(
        "RECOVERY_CAPTURABILITY", "recovery support/capturability", "DIAGNOSTIC_ONLY", "POSTCHECK_MECHANICS",
        "src/loaded_cmj/biomechanics/capturability.py", "predict_support_reserve + Plant.support_margin",
        "F3 capturability structural contract", "post_trace", "NONE", "m", "world sagittal/support plane",
        "COM/support geometry", "signed diagnostic", "NO_SCALAR_TOLERANCE", "F3 capturability evidence", "NOT_APPLICABLE",
        "diagnostic; no elastic residual", ("SUPPORTED_CONTACT",), "POSTTRACE_ONLY", "PRIVILEGED_ORACLE_ONLY", True, ("RECOVERY",),
        "Approximate reserve does not become exact same-Plant viability proof.",
    ),
    _spec(
        "RECOVERY_STABLE_DWELL", "recovery stable dwell", "PHASE_METADATA", "POSTCHECK_EVENT_ENGINE", _EVENTS,
        "CMJEventDetector recovery dwell logic", "EventThresholds/dwell contract", "post_trace", "NONE", "1", "world trace samples",
        "trace interval", "pass=True", "EXACT", "CMJEventDetector sole event authority", "NOT_APPLICABLE",
        "post-trace event predicate", ("SUPPORTED_CONTACT",), "POSTTRACE_ONLY", "POSTTRACE_EVALUATOR_ONLY", True, ("RECOVERY",),
        "Stable dwell and event ordering are phase metadata only.",
    ),
    _spec(
        "TERMINATION_RESULT", "canonical termination result", "PHASE_METADATA", "POSTCHECK_EVENT_ENGINE", _RESULTS,
        "RolloutResult / TerminationReason", "runtime result contract", "post_trace", "NONE", "enum", "not applicable",
        "sealed result", "exact enum", "EXACT", "runtime.results owner", "NOT_APPLICABLE", "result metadata; no residual",
        ("SUPPORTED_CONTACT", "FLIGHT", "CONTACT_TRANSITION"), "DISCRETE_PREDICATE", "POSTTRACE_EVALUATOR_ONLY", True,
        ALL_PHASES, "Canonical evaluator termination remains outside oracle dynamics.",
    ),
    _spec(
        "PHASE_MODE_LABELS", "oracle phase applicability labels", "PHASE_METADATA", "METADATA_ONLY", "ML-240 catalog",
        "ConstraintSpec.phase_applicability", "architecture phase-label contract", "metadata", "NONE", "1", "not applicable",
        "not applicable", "no residual", "EXACT", "ML-240 phase applicability contract", "NOT_APPLICABLE",
        "metadata only; no elastic residual", ALL_CONTACT_MODES, "NOT_APPLICABLE", "PRIVILEGED_ORACLE_ONLY", True, ALL_PHASES,
        "Labels distinguish transcription applicability without creating an event or phase state machine.",
    ),
)


CATALOG = ConstraintCatalog(_SPECS)


def adapt_validate_action(candidate: Any, action_spec: Any) -> Any:
    """Call the policy validation owner directly."""

    return validate_action(candidate, action_spec)


def adapt_project_accepted_action(previous_accepted_action: np.ndarray, raw_action: np.ndarray) -> np.ndarray:
    """Call the shared accepted-action slew owner directly."""

    return project_accepted_action(previous_accepted_action, raw_action)


def adapt_drive_state_step(
    command: np.ndarray,
    state: Any,
    anatomical_coordinates: np.ndarray,
    anatomical_rates: np.ndarray,
    h: float,
) -> dict[str, Any]:
    """Call the production DriveState substep owner directly."""

    return drive_state_step(command, state, anatomical_coordinates, anatomical_rates, h)


def adapt_anatomical_coordinates(plant: Plant, data: Any) -> np.ndarray:
    """Call the Plant anatomical-coordinate owner directly."""

    return plant.anatomical_coordinates(data)


def adapt_anatomical_rates(plant: Plant, data: Any) -> np.ndarray:
    """Call the Plant anatomical-rate owner directly."""

    return plant.anatomical_rates(data)


def adapt_contact_wrench_summary(plant: Plant, data: Any) -> dict[str, Any]:
    """Call the Plant bilateral wrench/COP/contact owner directly."""

    return plant.contact_wrench_summary(data)


def adapt_foot_contact_summary(plant: Plant, data: Any) -> dict[str, Any]:
    """Call the Plant foot-summary projection directly."""

    return plant.foot_contact_summary(data)


def adapt_support_polygon(plant: Plant, data: Any, latched: tuple[bool, bool]) -> np.ndarray:
    """Call the Plant support polygon owner directly."""

    return plant.support_polygon(data, latched)


def adapt_support_margin(plant: Plant, data: Any, com: np.ndarray, latched: tuple[bool, bool]) -> float:
    """Call the Plant support-margin owner directly."""

    return plant.support_margin(data, com, latched)


def adapt_com_velocity(plant: Plant, data: Any) -> np.ndarray:
    """Call the Plant COM velocity owner directly."""

    return plant.center_of_mass_velocity(data)


def adapt_linear_momentum(plant: Plant, data: Any) -> np.ndarray:
    """Call the Plant linear-momentum owner directly."""

    return plant.linear_momentum(data)


def adapt_centroidal_h(plant: Plant, data: Any) -> np.ndarray:
    """Call the Plant centroidal angular-momentum owner directly."""

    return plant.centroidal_angular_momentum(data)


def adapt_centroidal_hdot(plant: Plant, data: Any, external_wrench: np.ndarray) -> np.ndarray:
    """Call the Plant centroidal Hdot owner directly."""

    return plant.centroidal_hdot_from_external_wrench(data, external_wrench)


def adapt_power_components(plant: Plant, realized_torque: np.ndarray, data: Any) -> dict[str, float]:
    """Call the Plant realized-power ledger owner directly."""

    return plant.realized_power_components(realized_torque, data)


def adapt_capturability(predictor: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
    """Call the already-qualified capturability diagnostic owner directly."""

    return predictor(*args, **kwargs)


def adapt_force_time_metrics(*args: Any, **kwargs: Any) -> Any:
    """Call the canonical force-time metrics owner directly."""

    return derive_force_time_metrics(*args, **kwargs)


def adapt_official_event_finalize(detector: CMJEventDetector, *, horizon_s: float | None = None) -> Any:
    """Call the official event engine; no event state is stored in this catalog."""

    return detector.finalize(horizon_s=horizon_s)


def constraint_value(
    constraint_id: str,
    feasible: bool | None,
    *,
    margin_or_residual: Any = None,
    raw_value: Any = None,
    tolerance: float | str | None = None,
) -> ConstraintValue:
    """Construct the small standardized adapter result without adding a DSL."""

    CATALOG.get(constraint_id)
    return ConstraintValue(
        constraint_id=constraint_id,
        feasible=feasible,
        margin_or_residual=margin_or_residual,
        raw_value=raw_value,
        tolerance=tolerance,
    )


__all__ = [
    "ALLOWED_CLASSES",
    "ALLOWED_DIFFERENTIABILITY",
    "ALLOWED_ENFORCEMENTS",
    "ALLOWED_SENSES",
    "ALLOWED_VISIBILITY",
    "ALL_PHASES",
    "CATALOG",
    "CATALOG_ID",
    "ConstraintCatalog",
    "ConstraintSpec",
    "ConstraintValue",
    "adapt_anatomical_coordinates",
    "adapt_anatomical_rates",
    "adapt_capturability",
    "adapt_centroidal_h",
    "adapt_centroidal_hdot",
    "adapt_com_velocity",
    "adapt_contact_wrench_summary",
    "adapt_drive_state_step",
    "adapt_force_time_metrics",
    "adapt_foot_contact_summary",
    "adapt_linear_momentum",
    "adapt_official_event_finalize",
    "adapt_power_components",
    "adapt_project_accepted_action",
    "adapt_support_margin",
    "adapt_support_polygon",
    "adapt_validate_action",
    "constraint_value",
]
