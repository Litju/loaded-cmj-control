"""RES-85C deterministic Plant joint-ROM soft-limit probe.

MISSION: RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001

The sealed V3 Plant declares ``limited="true"`` hinge joints with MuJoCo 3.8.0
compiler-default limit softness, so the ROM is enforced by a soft constraint and
a driven joint may cross its limit by a bounded numerical amount.  This script
measures that Plant property per actuated channel and per direction with a
deterministic probe (no controller, no measurement layer):

  start at ``limit -/+ 0.15 rad`` (inside the ROM), zero velocity, apply the
  channel's frozen moment ceiling towards the limit for 0.8 s at the native
  timestep, and record the maximum overshoot beyond the limit.

The result is the declared per-channel soft-limit compliance envelope used by
the RES-85C joint-ROM feasibility check; it is a Plant property, not a
controller budget.

Run:  python3 probe_joint_rom_soft_limits.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SRC = REPO / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import mujoco  # noqa: E402

from loaded_cmj.v3.actuation import CHANNELS, MOMENT_CEILING_NM  # noqa: E402
from loaded_cmj.v3.constants import V3_JOINT_RANGES_RAD  # noqa: E402
from loaded_cmj.v3.plant import V3Plant  # noqa: E402

START_OFFSET_RAD = 0.15
STEPS = 400


def _run_direction(channel: str, direction: int, ceiling_nm: float) -> dict[str, Any]:
    plant = V3Plant()
    data = plant.make_data()
    lo, hi = V3_JOINT_RANGES_RAD[channel]
    limit = hi if direction > 0 else lo
    start = limit - direction * START_OFFSET_RAD
    data.qpos[plant.idx.qadr["root_tz"]] = 2.0
    for name in CHANNELS:
        data.qpos[plant.idx.qadr[name]] = 0.0
    data.qpos[plant.idx.qadr[channel]] = start
    data.qvel[:] = 0.0
    extremum = start
    for _ in range(STEPS):
        data.ctrl[:] = 0.0
        data.ctrl[CHANNELS.index(channel)] = direction * ceiling_nm
        mujoco.mj_step(plant.model, data)
        value = float(data.qpos[plant.idx.qadr[channel]])
        extremum = max(extremum, value) if direction > 0 else min(extremum, value)
    return {
        "direction": "upper" if direction > 0 else "lower",
        "limit_rad": float(limit),
        "start_rad": float(start),
        "applied_moment_nm": float(direction * ceiling_nm),
        "extremum_rad": float(extremum),
        "overshoot_rad": float(direction * (extremum - limit)),
    }


def build() -> dict[str, Any]:
    channels: dict[str, Any] = {}
    worst = 0.0
    for name in CHANNELS:
        rng = V3_JOINT_RANGES_RAD[name]
        if rng is None:
            continue
        ceiling = float(MOMENT_CEILING_NM[CHANNELS.index(name)])
        upper = _run_direction(name, +1, ceiling)
        lower = _run_direction(name, -1, ceiling)
        envelope = max(upper["overshoot_rad"], lower["overshoot_rad"])
        worst = max(worst, envelope)
        channels[name] = {
            "moment_ceiling_nm": ceiling,
            "upper": upper,
            "lower": lower,
            "soft_limit_compliance_rad": float(envelope),
        }
    return {
        "schema_version": "1.0.0",
        "mission": "RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001",
        "linear_issue": "RES-85",
        "purpose": ("declared Plant soft-limit compliance envelope used by the RES-85C "
                    "launch joint-ROM feasibility check"),
        "probe": {
            "start_offset_rad": START_OFFSET_RAD,
            "steps": STEPS,
            "moment": "channel frozen moment ceiling (ACTUATION_AUTHORITY.json)",
            "integrator": "sealed Plant (MuJoCo 3.8.0 compiler defaults)",
            "measurement_layer_used": False,
        },
        "channels": channels,
        "worst_case_soft_limit_compliance_rad": float(worst),
        "interpretation": (
            "a driven joint may cross its declared ROM by at most this measured amount; "
            "the RES-85C feasibility check requires the launch measured overshoot to stay "
            "inside the per-channel envelope AND the controller posture reference to stay "
            "inside the declared ROM"),
    }


def main() -> int:
    payload = build()
    (HERE / "JOINT_ROM_SOFT_LIMIT_PROBE.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n")
    for name, value in payload["channels"].items():
        print(f"{name:14s} ceiling={value['moment_ceiling_nm']:6.1f} "
              f"upper={value['upper']['overshoot_rad']:+.4f} "
              f"lower={value['lower']['overshoot_rad']:+.4f} "
              f"envelope={value['soft_limit_compliance_rad']:.4f}")
    print("worst case:", payload["worst_case_soft_limit_compliance_rad"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
