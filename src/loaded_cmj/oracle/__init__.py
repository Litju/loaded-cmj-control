"""Privileged, source-referencing oracle metadata for LCMJ V1."""

from loaded_cmj.oracle.composition import (
    E3E4_OWNER_IDS,
    E3E4_PHYSICAL_ROW_COUNT,
    E3E4CompositionError,
    E3E4PhysicalConstraintComposer,
    PhysicalConstraintDerivative,
    PhysicalConstraintOwnerReceipt,
)
from loaded_cmj.oracle.constraints import (
    CATALOG,
    CATALOG_ID,
    ConstraintCatalog,
    ConstraintSpec,
    ConstraintValue,
)
from loaded_cmj.oracle.transcription import (
    TRANSCRIPTION_SCHEMA_ID,
    DirectMultipleShootingProblem,
)

__all__ = [
    "CATALOG",
    "CATALOG_ID",
    "ConstraintCatalog",
    "ConstraintSpec",
    "ConstraintValue",
    "DirectMultipleShootingProblem",
    "E3E4CompositionError",
    "E3E4PhysicalConstraintComposer",
    "E3E4_OWNER_IDS",
    "E3E4_PHYSICAL_ROW_COUNT",
    "PhysicalConstraintDerivative",
    "PhysicalConstraintOwnerReceipt",
    "TRANSCRIPTION_SCHEMA_ID",
]
