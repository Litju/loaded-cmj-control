"""Exact runtime contact realization (RES-86 solution verification).

Authority: ``LCMJ_RES86_V3_CONTACT_REALIZATION_V1``.

The RES-84 contact-parameter authority seals the *geom-level* Plant inventory
(``loaded_cmj.v3.measurement.contact_parameter_inventory``) and the RES-86
landing acceptance authority seals the nominal contact realization used by the
landing gates (``solref`` [0.02, 1.0], ``solimp`` [0.9, 0.95, 0.001, 0.5, 2.0],
plantar ``condim`` 4, sliding friction 0.9, torsional/rolling declared 0.0).

Neither of those is what MuJoCo actually solves.  The solver consumes the
*effective* ``mjContact`` row after the engine's geom-pair mixing: condim is
resolved, ``solref``/``solimp`` are mixed through ``solmix``/priority, friction
is mixed elementwise with the engine's minimum clamp, and the torsional/rolling
terms are realized at ``mjMINMU`` even when the declared placeholder is 0.0.

This module reads the exact runtime rows and verifies them against the declared
nominal, so a landing witness can never claim a contact realization the engine
did not actually use.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np

from loaded_cmj.v3 import constants as C
from loaded_cmj.v3.plant import V3Plant, model_xml

V3_CONTACT_REALIZATION_AUTHORITY_ID = "LCMJ_RES86_V3_CONTACT_REALIZATION_V1"

DECLARED_SOLREF = (0.02, 1.0)
DECLARED_SOLIMP = (0.9, 0.95, 0.001, 0.5, 2.0)
DECLARED_SLIDING_FRICTION = 0.9
DECLARED_PLANTAR_CONDIM = 4
DECLARED_TORSIONAL_ROLLING = 0.0
ENGINE_MINIMUM_FRICTION = float(mujoco.mjMINMU)
SOLREF_TOLERANCE = 1.0e-12
SOLIMP_TOLERANCE = 1.0e-12

# ---------------------------------------------------------------------------
# RES-86 contact candidate (solution verification, CC-10/CC-11)
# ---------------------------------------------------------------------------
# The RES-84 provisional numerical baseline is the declared geom-level
# ``solref`` (0.02, 1.0).  RES-86 owns the final solution verification of the
# provisional contact numerics.  The smallest stiffening declared by the RES-86
# audit lattice is the plantar support-geom solref time constant 0.02 -> 0.01 s
# (the other lattice entries additionally stiffen the impedance scaling and are
# therefore *not* smaller).  The geom-level declaration is not what the solver
# consumes: MuJoCo mixes the two geoms' solref through ``solmix`` (both 1.0,
# equal priority 0), so the realized contact consumes the arithmetic mean of the
# declared plantar value and the floor's compiled default (0.02, 1.0):
# 0.01 -> 0.015 and 0.02 -> 0.02.  The realized value is what every
# qualification claim is stated against.
RES86_CONTACT_CANDIDATE_LABEL = "RES86_CONTACT_CANDIDATE_SOLREF_TAU_0_01"
RES86_CANDIDATE_DECLARED_SOLREF = (0.01, 1.0)
RES86_NOMINAL_DECLARED_SOLREF = DECLARED_SOLREF
RES86_CANDIDATE_REALIZED_SOLREF = (0.015, 1.0)
RES86_NOMINAL_REALIZED_SOLREF = DECLARED_SOLREF
MUJOCO_COMPILED_DEFAULT_SOLREF = (0.02, 1.0)
# MuJoCo resolves a soft contact against a reference-acceleration safety
# mechanism (``refsafe``); the RES-86 solution verification never disables it
# and never changes the integrator/solver/cone semantics.
ENGINE_REQUIRED_DISABLED_FLAGS = 0
# A declared contact time constant is numerically resolved when the engine's
# stiffness/constraint update is not asked to represent a time constant shorter
# than two integration steps.  MuJoCo's own compiled default check uses the same
# factor for the reference acceleration path.
TIMECONST_DT_RESOLUTION_FACTOR = 2.0


def mixed_solref(declared_solref: Sequence[float], *,
                 other_solref: Sequence[float] = MUJOCO_COMPILED_DEFAULT_SOLREF,
                 declared_solmix: float = 1.0,
                 other_solmix: float = 1.0,
                 declared_priority: int = 0,
                 other_priority: int = 0) -> tuple[float, float]:
    """Engine geom-pair mixing rule for ``solref`` (equal-priority solmix mean).

    MuJoCo takes the higher-priority geom's value when the priorities differ;
    with equal priority the two values are combined with the ``solmix``
    weights.  This helper is the *expected* value only: every qualification
    claim is verified against the engine's realized ``mjContact`` row.
    """
    if int(declared_priority) > int(other_priority):
        return (float(declared_solref[0]), float(declared_solref[1]))
    if int(other_priority) > int(declared_priority):
        return (float(other_solref[0]), float(other_solref[1]))
    total = float(declared_solmix) + float(other_solmix)
    if total <= 0.0:
        raise ValueError("solmix weights must sum to a positive value")
    return tuple(
        (float(declared_solmix) * float(a) + float(other_solmix) * float(b)) / total
        for a, b in zip(declared_solref, other_solref))


def build_contact_realization_plant(declared_solref: Sequence[float] | None = None,
                                    dt_s: float | None = None) -> V3Plant:
    """In-memory Plant whose support geoms declare ``declared_solref``.

    Only the legal plantar support geoms' ``solref`` is set; ``condim``,
    ``solimp`` and ``friction`` are untouched, so the realization is a single
    declared contact-compliance change on an unchanged Plant topology.  The
    sealed XML on disk is never mutated.  ``dt_s`` overrides the integration
    step (the declared timestep sensitivity lattice) without touching the sealed
    Plant declaration.
    """
    if declared_solref is None and dt_s is None:
        return V3Plant()
    spec = mujoco.MjSpec.from_string(model_xml())
    if dt_s is not None:
        spec.option.timestep = float(dt_s)
    if declared_solref is not None:
        support_names = set(C.V3_PLANTAR_SUPPORT_GEOMS)
        values = np.asarray(declared_solref, dtype=np.float64)
        if values.shape != (2,):
            raise ValueError("declared_solref must have two entries")
        for geom in spec.geoms:
            if geom.name in support_names:
                geom.solref = values
    return V3Plant(model=spec.compile())


def engine_solver_semantics(model: mujoco.MjModel) -> dict:
    """Exact solver semantics of a compiled model (never changed by RES-86)."""
    disableflags = int(model.opt.disableflags)
    return {
        "timestep_s": float(model.opt.timestep),
        "integrator": int(model.opt.integrator),
        "cone": int(model.opt.cone),
        "solver": int(model.opt.solver),
        "iterations": int(model.opt.iterations),
        "tolerance": float(model.opt.tolerance),
        "ls_iterations": int(model.opt.ls_iterations),
        "disableflags": disableflags,
        "refsafe_enabled": bool(
            not (disableflags & int(mujoco.mjtDisableBit.mjDSBL_REFSAFE))),
        "contact_disabled": bool(
            disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)),
    }


def timeconst_resolution(declared_solref: Sequence[float], dt_s: float, *,
                         other_solref: Sequence[float] = MUJOCO_COMPILED_DEFAULT_SOLREF
                         ) -> dict:
    """Realized time constant and its resolution against the integration step."""
    realized = mixed_solref(declared_solref, other_solref=other_solref)
    ratio = float(realized[0]) / float(dt_s) if dt_s > 0.0 else float("inf")
    return {
        "declared_solref": [float(v) for v in declared_solref],
        "realized_solref": [float(v) for v in realized],
        "dt_s": float(dt_s),
        "timeconst_over_dt": ratio,
        "resolved": bool(ratio >= TIMECONST_DT_RESOLUTION_FACTOR),
        "resolution_factor": TIMECONST_DT_RESOLUTION_FACTOR,
    }


@dataclass(frozen=True)
class V3EffectiveContact:
    """One runtime ``mjContact`` row involving the floor."""

    index: int
    geom1: str
    geom2: str
    other_geom: str
    collision_class: str
    side: str | None
    region: str | None
    dim: int
    solref: tuple[float, float]
    solimp: tuple[float, float, float, float, float]
    friction: tuple[float, float, float, float, float]
    includemargin_m: float
    dist_m: float
    geom1_priority: int
    geom2_priority: int
    geom1_solmix: float
    geom2_solmix: float

    @property
    def is_plantar_floor(self) -> bool:
        return self.side is not None


def _geom_name(plant: V3Plant, gid: int) -> str:
    try:
        return plant.geom_name(int(gid))
    except (KeyError, ValueError):
        return f"geom_{int(gid)}"


def effective_floor_contacts(plant: V3Plant,
                             data: mujoco.MjData) -> tuple[V3EffectiveContact, ...]:
    """Every active floor contact with its exact engine-mixed parameters."""
    model = plant.model
    floor_gid = int(plant.idx.geom[C.V3_FLOOR_GEOM])
    legal = {int(plant.idx.geom[C.V3_SUPPORT_GEOM_BY_FOOT_REGION[side][region]]): (side, region)
             for side in C.V3_SIDES for region in C.V3_SUPPORT_REGIONS}
    prohibited = {int(plant.idx.geom[name]) for name in C.V3_PROHIBITED_FLOOR_GEOMS}
    rows: list[V3EffectiveContact] = []
    for contact_id in range(int(data.ncon)):
        contact = data.contact[contact_id]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if g1 == floor_gid:
            other = g2
        elif g2 == floor_gid:
            other = g1
        else:
            continue
        side, region = legal.get(other, (None, None))
        collision_class = ("LEGAL_PLANTAR_FLOOR" if side is not None
                           else "PROHIBITED_FLOOR" if other in prohibited
                           else "OTHER")
        rows.append(V3EffectiveContact(
            index=contact_id,
            geom1=_geom_name(plant, g1),
            geom2=_geom_name(plant, g2),
            other_geom=_geom_name(plant, other),
            collision_class=collision_class,
            side=side,
            region=region,
            dim=int(contact.dim),
            solref=(float(contact.solref[0]), float(contact.solref[1])),
            solimp=tuple(float(v) for v in np.asarray(contact.solimp,
                                                      dtype=np.float64).reshape(-1)),
            friction=tuple(float(v) for v in np.asarray(contact.friction,
                                                        dtype=np.float64).reshape(-1)),
            includemargin_m=float(contact.includemargin),
            dist_m=float(contact.dist),
            geom1_priority=int(model.geom_priority[g1]),
            geom2_priority=int(model.geom_priority[g2]),
            geom1_solmix=float(model.geom_solmix[g1]),
            geom2_solmix=float(model.geom_solmix[g2]),
        ))
    return tuple(rows)


def verify_effective_contact_realization(plant: V3Plant, data: mujoco.MjData,
                                         *, require_plantar: bool = True,
                                         expected_solref: Sequence[float] = DECLARED_SOLREF,
                                         expected_solimp: Sequence[float] = DECLARED_SOLIMP,
                                         expected_dim: int = DECLARED_PLANTAR_CONDIM,
                                         expected_sliding_friction: float = DECLARED_SLIDING_FRICTION,
                                         expected_includemargin_m: float | None = None,
                                         ) -> dict:
    """Verify the runtime rows against an expected (mixed) realization.

    Every legal plantar floor contact must have the *realized* ``dim``,
    ``solref``, ``solimp``, sliding friction and (when declared)
    ``includemargin``; the declared torsional/rolling placeholder (0.0) is
    realized exactly at the engine's minimum friction clamp ``mjMINMU`` (the
    engine never realizes a zero torsional/rolling term).  ``expected_solref``
    is the value after the engine's geom-pair mixing, never the raw geom-level
    declaration.
    """
    expected_solref = tuple(float(v) for v in expected_solref)
    expected_solimp = tuple(float(v) for v in expected_solimp)
    rows = effective_floor_contacts(plant, data)
    plantar = [row for row in rows if row.is_plantar_floor]
    failures: list[str] = []
    if require_plantar and not plantar:
        failures.append("NO_ACTIVE_PLANTAR_CONTACT")
    for row in plantar:
        if row.dim != int(expected_dim):
            failures.append(f"CONDIM:{row.other_geom}:{row.dim}")
        if (abs(row.solref[0] - expected_solref[0]) > SOLREF_TOLERANCE
                or abs(row.solref[1] - expected_solref[1]) > SOLREF_TOLERANCE):
            failures.append(f"SOLREF:{row.other_geom}:{row.solref}")
        for actual, declared in zip(row.solimp, expected_solimp):
            if abs(actual - declared) > SOLIMP_TOLERANCE:
                failures.append(f"SOLIMP:{row.other_geom}:{row.solimp}")
                break
        if (abs(row.friction[0] - float(expected_sliding_friction)) > SOLREF_TOLERANCE
                or abs(row.friction[1] - float(expected_sliding_friction)) > SOLREF_TOLERANCE):
            failures.append(f"SLIDING_FRICTION:{row.other_geom}:{row.friction}")
        if (abs(row.friction[2] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE
                or abs(row.friction[3] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE
                or abs(row.friction[4] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE):
            failures.append(f"TOR_ROLL_CLAMP:{row.other_geom}:{row.friction}")
        if expected_includemargin_m is not None and abs(
                row.includemargin_m - float(expected_includemargin_m)) > SOLREF_TOLERANCE:
            failures.append(f"INCLUDEMARGIN:{row.other_geom}:{row.includemargin_m}")
    return {
        "authority_id": V3_CONTACT_REALIZATION_AUTHORITY_ID,
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "declared": {
            "solref": list(expected_solref),
            "solimp": list(expected_solimp),
            "sliding_friction": float(expected_sliding_friction),
            "plantar_condim": int(expected_dim),
            "torsional_rolling_declared": DECLARED_TORSIONAL_ROLLING,
            "torsional_rolling_realized": ENGINE_MINIMUM_FRICTION,
            "includemargin_m": expected_includemargin_m,
            "realization_note": (
                "the declared 0.0 torsional/rolling placeholder is realized at "
                "the engine minimum friction clamp mjMINMU; the engine never "
                "solves a zero torsional/rolling term"),
        },
        "solver_semantics": engine_solver_semantics(plant.model),
        "plantar_rows": [
            {
                "geom": row.other_geom,
                "side": row.side,
                "region": row.region,
                "dim": row.dim,
                "solref": list(row.solref),
                "solimp": list(row.solimp),
                "friction": list(row.friction),
                "includemargin_m": row.includemargin_m,
                "dist_m": row.dist_m,
                "geom_priorities": [row.geom1_priority, row.geom2_priority],
                "geom_solmix": [row.geom1_solmix, row.geom2_solmix],
            }
            for row in plantar
        ],
        "floor_row_count": len(rows),
        "prohibited_row_count": sum(1 for row in rows if row.collision_class == "PROHIBITED_FLOOR"),
    }


def contact_realization_fingerprint(plant: V3Plant, data: mujoco.MjData) -> str:
    """Stable digest of the runtime contact realization (evidence binding)."""
    import hashlib
    rows = effective_floor_contacts(plant, data)
    digest = hashlib.sha256()
    for row in rows:
        digest.update(f"{row.other_geom}|{row.dim}|".encode("utf-8"))
        digest.update(np.asarray(row.solref, dtype=np.float64).tobytes())
        digest.update(np.asarray(row.solimp, dtype=np.float64).tobytes())
        digest.update(np.asarray(row.friction, dtype=np.float64).tobytes())
    return digest.hexdigest()


__all__ = [
    "DECLARED_PLANTAR_CONDIM",
    "DECLARED_SLIDING_FRICTION",
    "DECLARED_SOLIMP",
    "DECLARED_SOLREF",
    "DECLARED_TORSIONAL_ROLLING",
    "ENGINE_MINIMUM_FRICTION",
    "ENGINE_REQUIRED_DISABLED_FLAGS",
    "MUJOCO_COMPILED_DEFAULT_SOLREF",
    "RES86_CANDIDATE_DECLARED_SOLREF",
    "RES86_CANDIDATE_REALIZED_SOLREF",
    "RES86_CONTACT_CANDIDATE_LABEL",
    "RES86_NOMINAL_DECLARED_SOLREF",
    "RES86_NOMINAL_REALIZED_SOLREF",
    "TIMECONST_DT_RESOLUTION_FACTOR",
    "V3EffectiveContact",
    "V3_CONTACT_REALIZATION_AUTHORITY_ID",
    "build_contact_realization_plant",
    "contact_realization_fingerprint",
    "effective_floor_contacts",
    "engine_solver_semantics",
    "mixed_solref",
    "timeconst_resolution",
    "verify_effective_contact_realization",
]
