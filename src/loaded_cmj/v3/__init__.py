"""LCMJ Plant V3 — elite-soccer human-valid loaded-CMJ Plant package.

Authority: LCMJ_RES95_ELITE_SOCCER_PLANT_MODEL_AUTHORITY_V1.

V3 is a distinct successor implementation surface.  V1 and V2 remain
historical implementations in their own packages and are not imported here.
"""

from loaded_cmj.v3.constants import (
    V3_MODEL_ID,
    V3_MODEL_REVISION,
)
from loaded_cmj.v3.plant import (
    V3Plant,
    V3PlantError,
    V3PoseSpec,
    build_model,
    build_pose,
    model_xml,
    representative_poses,
)

__all__ = [
    "V3_MODEL_ID",
    "V3_MODEL_REVISION",
    "V3Plant",
    "V3PlantError",
    "V3PoseSpec",
    "build_model",
    "build_pose",
    "model_xml",
    "representative_poses",
]
