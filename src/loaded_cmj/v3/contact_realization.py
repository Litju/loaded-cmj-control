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
from loaded_cmj.v3.plant import V3Plant

V3_CONTACT_REALIZATION_AUTHORITY_ID = "LCMJ_RES86_V3_CONTACT_REALIZATION_V1"

DECLARED_SOLREF = (0.02, 1.0)
DECLARED_SOLIMP = (0.9, 0.95, 0.001, 0.5, 2.0)
DECLARED_SLIDING_FRICTION = 0.9
DECLARED_PLANTAR_CONDIM = 4
DECLARED_TORSIONAL_ROLLING = 0.0
ENGINE_MINIMUM_FRICTION = float(mujoco.mjMINMU)
SOLREF_TOLERANCE = 1.0e-12
SOLIMP_TOLERANCE = 1.0e-12


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
                                         *, require_plantar: bool = True) -> dict:
    """Verify the runtime rows against the declared nominal realization.

    Every legal plantar floor contact must have the declared ``dim``,
    ``solref``, ``solimp`` and sliding friction; the declared torsional/rolling
    placeholder (0.0) must be realized exactly at the engine's minimum friction
    clamp ``mjMINMU`` (the engine never realizes a zero torsional/rolling term).
    """
    rows = effective_floor_contacts(plant, data)
    plantar = [row for row in rows if row.is_plantar_floor]
    failures: list[str] = []
    if require_plantar and not plantar:
        failures.append("NO_ACTIVE_PLANTAR_CONTACT")
    for row in plantar:
        if row.dim != DECLARED_PLANTAR_CONDIM:
            failures.append(f"CONDIM:{row.other_geom}:{row.dim}")
        if (abs(row.solref[0] - DECLARED_SOLREF[0]) > SOLREF_TOLERANCE
                or abs(row.solref[1] - DECLARED_SOLREF[1]) > SOLREF_TOLERANCE):
            failures.append(f"SOLREF:{row.other_geom}:{row.solref}")
        for actual, declared in zip(row.solimp, DECLARED_SOLIMP):
            if abs(actual - declared) > SOLIMP_TOLERANCE:
                failures.append(f"SOLIMP:{row.other_geom}:{row.solimp}")
                break
        if (abs(row.friction[0] - DECLARED_SLIDING_FRICTION) > SOLREF_TOLERANCE
                or abs(row.friction[1] - DECLARED_SLIDING_FRICTION) > SOLREF_TOLERANCE):
            failures.append(f"SLIDING_FRICTION:{row.other_geom}:{row.friction}")
        if (abs(row.friction[2] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE
                or abs(row.friction[3] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE
                or abs(row.friction[4] - ENGINE_MINIMUM_FRICTION) > SOLREF_TOLERANCE):
            failures.append(f"TOR_ROLL_CLAMP:{row.other_geom}:{row.friction}")
    return {
        "authority_id": V3_CONTACT_REALIZATION_AUTHORITY_ID,
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "declared": {
            "solref": list(DECLARED_SOLREF),
            "solimp": list(DECLARED_SOLIMP),
            "sliding_friction": DECLARED_SLIDING_FRICTION,
            "plantar_condim": DECLARED_PLANTAR_CONDIM,
            "torsional_rolling_declared": DECLARED_TORSIONAL_ROLLING,
            "torsional_rolling_realized": ENGINE_MINIMUM_FRICTION,
            "realization_note": (
                "the declared 0.0 torsional/rolling placeholder is realized at "
                "the engine minimum friction clamp mjMINMU; the engine never "
                "solves a zero torsional/rolling term"),
        },
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
    "V3EffectiveContact",
    "V3_CONTACT_REALIZATION_AUTHORITY_ID",
    "contact_realization_fingerprint",
    "effective_floor_contacts",
    "verify_effective_contact_realization",
]
