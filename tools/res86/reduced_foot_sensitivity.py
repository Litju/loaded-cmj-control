#!/usr/bin/env python3
"""RES-86 audit-only reduced-foot model-form sensitivity.

MISSION: RES86_TOUCHDOWN_CONFIGURATION_AND_ABSORPTION_RESOLUTION_001
LINEAR ISSUE: RES-86

Mission step 6.  The controller cannot close the frozen ankle structural ROM
while it sustains the absorption stroke (the best exact-branch traces reach the
frozen 0.785398 rad ankle limit and the next validated interval needs more ROM
than the gate allows).  This instrument is an **audit only**: it builds an
in-memory Plant realization that releases the sealed locked midfoot
(FM-01/FM-02/FM-03: "no midtarsal DOF, no unapproved arch DOF") into a bounded
midtarsal hinge at the declared midtarsal point, and measures whether that
bounded freedom changes the causal bottleneck.

It never mutates the sealed Plant XML on disk, never changes the frozen
authority, and never proposes a Plant change: the output is a declared
sensitivity measurement for the owner, with the nominal realization as the
paired control.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3 import constants as C  # noqa: E402
from loaded_cmj.v3.landing_control import V3LandingConfig  # noqa: E402
from loaded_cmj.v3.landing_runtime import run_landing_episode  # noqa: E402
from loaded_cmj.v3.plant import V3Plant, model_xml  # noqa: E402

V3_REDUCED_FOOT_AUDIT_AUTHORITY_ID = "LCMJ_RES86_REDUCED_FOOT_AUDIT_V1"

MIDFOOT_JOINT_NAMES = ("left_midfoot", "right_midfoot")
MIDFOOT_BODY_NAMES = ("left_forefoot", "right_forefoot")


@dataclass(frozen=True)
class ReducedFootRealization:
    label: str
    midfoot_range_rad: float
    midfoot_stiffness_nm_per_rad: float
    midfoot_damping_nms_per_rad: float
    declaration: str


NOMINAL = ReducedFootRealization(
    label="nominal_locked_midfoot", midfoot_range_rad=0.0,
    midfoot_stiffness_nm_per_rad=0.0, midfoot_damping_nms_per_rad=0.0,
    declaration="sealed RES-95 Plant: forefoot welded at the midtarsal point")

LATTICE: tuple[ReducedFootRealization, ...] = (
    NOMINAL,
    ReducedFootRealization(
        "midfoot_pm_0.05", 0.05, 25.0, 2.0,
        "bounded midtarsal freedom (FM-09-style passive prior)"),
    ReducedFootRealization(
        "midfoot_pm_0.10", 0.10, 25.0, 2.0,
        "bounded midtarsal freedom (FM-09-style passive prior)"),
    ReducedFootRealization(
        "midfoot_pm_0.20", 0.20, 25.0, 2.0,
        "bounded midtarsal freedom (FM-09-style passive prior)"),
)


class _AuditPlant(V3Plant):
    """Audit-only Plant surface: skips the frozen-identity assertion.

    The identity assertion exists to protect the frozen Plant.  This audit
    realization is deliberately *not* the frozen Plant, so the assertion is
    skipped here and the realization is recorded as audit-only.  Nothing in
    production can construct this class.
    """

    def _assert_identity(self) -> None:  # noqa: D102 - audit override
        return


def build_reduced_foot_plant(realization: ReducedFootRealization) -> V3Plant:
    """Audit-only in-memory Plant with a bounded midtarsal hinge."""
    xml = model_xml()
    if realization.midfoot_range_rad <= 0.0:
        return V3Plant(model=mujoco.MjSpec.from_string(xml).compile())
    spec = mujoco.MjSpec.from_string(xml)
    for name in MIDFOOT_BODY_NAMES:
        body = spec.body(name)
        joint = body.add_joint()
        joint.name = "left_midfoot" if name.startswith("left") else "right_midfoot"
        joint.type = mujoco.mjtJoint.mjJNT_HINGE
        joint.axis = np.asarray([0.0, -1.0, 0.0], dtype=np.float64)
        joint.range = np.asarray([-realization.midfoot_range_rad,
                                  realization.midfoot_range_rad], dtype=np.float64)
        joint.limited = True
        joint.stiffness = np.asarray(
            [[realization.midfoot_stiffness_nm_per_rad], [0.0], [0.0]],
            dtype=np.float64)
        joint.damping = np.asarray(
            [[realization.midfoot_damping_nms_per_rad], [0.0], [0.0]],
            dtype=np.float64)
        joint.armature = 0.0
    return _AuditPlant(model=spec.compile())


def transfer_state(nominal: V3Plant, target: V3Plant,
                   vector: np.ndarray) -> np.ndarray:
    """Transfer a nominal integration state onto the audit realization by name.

    The audit realization adds the bounded midtarsal DOF; every sealed joint is
    transferred exactly and the new DOF starts at its neutral zero.
    """
    src = nominal.make_data()
    mujoco.mj_setState(nominal.model, src, np.asarray(vector, dtype=np.float64),
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    dst = target.make_data()
    mujoco.mj_resetData(target.model, dst)
    for name in C.V3_JOINT_NAMES:
        dst.qpos[target.idx.qadr[name]] = src.qpos[nominal.idx.qadr[name]]
        dst.qvel[target.idx.vadr[name]] = src.qvel[nominal.idx.vadr[name]]
    for index, name in enumerate(C.V3_ACTUATOR_NAMES):
        dst.ctrl[index] = src.ctrl[index]
    for name in C.V3_JOINT_NAMES:
        dst.qacc_warmstart[target.idx.vadr[name]] = \
            src.qacc_warmstart[nominal.idx.vadr[name]]
    dst.time = float(src.time)
    mujoco.mj_forward(target.model, dst)
    out = np.zeros(int(mujoco.mj_stateSize(target.model,
                                           mujoco.mjtState.mjSTATE_INTEGRATION)))
    mujoco.mj_getState(target.model, dst, out, mujoco.mjtState.mjSTATE_INTEGRATION)
    return out


def run_realization(realization: ReducedFootRealization, *, horizon_s: float,
                    landing_config: V3LandingConfig) -> dict:
    plant = build_reduced_foot_plant(realization)
    t0 = time.time()
    certificate = None
    if True:  # always run the paired nominal-launch -> transfer -> landing path
        from loaded_cmj.v3.controller import V3ControllerConfig
        from loaded_cmj.v3.landing_runtime import (
            POST_APEX_SAMPLE,
            V3HandoffCertificate,
            _run_launch_to_post_apex,
        )
        nominal = V3Plant()
        nominal_data = nominal.make_data()
        _launch, nominal_cert, _events, _rec = _run_launch_to_post_apex(
            nominal, nominal_data,
            n_samples=int(round(horizon_s / 0.002)),
            config=V3ControllerConfig(dt_s=0.002))
        state = transfer_state(nominal, plant, nominal_cert.state_vector)
        certificate = V3HandoffCertificate(
            sample_index=int(nominal_cert.sample_index),
            time_s=float(nominal_cert.time_s),
            state_vector=state,
            ctrl_nm=np.array(nominal_cert.ctrl_nm, dtype=np.float64, copy=True),
            qacc_warmstart=np.zeros(int(plant.model.nv), dtype=np.float64),
            actuation=nominal_cert.actuation,
            state_sha256="",
        )
        import hashlib
        object.__setattr__(certificate, "state_sha256",
                           hashlib.sha256(state.tobytes()).hexdigest())
    episode = run_landing_episode(horizon_s=horizon_s, landing_config=landing_config,
                                  plant=plant, handoff_certificate=certificate)
    events = episode.events
    telemetry = episode.telemetry
    ankle_upper = float(C.V3_JOINT_RANGES_RAD["left_ankle"][1])
    qpos = np.asarray(telemetry.qpos)
    ankle = np.asarray([float(qpos[i, plant.idx.qadr["left_ankle"]])
                        for i in range(qpos.shape[0])])
    midfoot = None
    if realization.midfoot_range_rad > 0.0:
        mid_joint = int(mujoco.mj_name2id(
            plant.model, mujoco.mjtObj.mjOBJ_JOINT, "left_midfoot"))
        mid_adr = int(plant.model.jnt_qposadr[mid_joint])
        midfoot = np.asarray([float(qpos[i, mid_adr]) for i in range(qpos.shape[0])])
    return {
        "label": realization.label,
        "declared": {
            "midfoot_range_rad": realization.midfoot_range_rad,
            "midfoot_stiffness_nm_per_rad": realization.midfoot_stiffness_nm_per_rad,
            "midfoot_damping_nms_per_rad": realization.midfoot_damping_nms_per_rad,
            "declaration": realization.declaration,
        },
        "status": episode.status,
        "fault": episode.fault,
        "landing_samples": int(episode.diagnostics["landing_samples"]),
        "e10_confirmed_time_s": episode.diagnostics["e10_confirmed_time_s"],
        "hard_gates": (events.get("hard_gate_audit") or {}).get("gates"),
        "hard_gate_values": {
            k: v for k, v in (events.get("hard_gate_audit") or {}).items()
            if k in ("max_penetration_m", "peak_total_floor_fz_bw",
                     "chatter_transitions", "material_reflight_intervals",
                     "min_rom_margin_rad", "max_abs_moment_ratio",
                     "max_abs_power_ratio")},
        "e10_event": events.get("e10"),
        "point_gates": ((events.get("e10") or {}).get("point") or {}).get("gates"),
        "window_gates": (events.get("first_contact_to_e10_window") or {}).get("gates"),
        "ankle_upper_rad": ankle_upper,
        "ankle_max_rad": float(np.max(ankle)) if ankle.size else None,
        "ankle_rom_margin_min_rad": float(ankle_upper - np.max(ankle)) if ankle.size else None,
        "midfoot_max_abs_rad": (float(np.max(np.abs(midfoot)))
                                if midfoot is not None and midfoot.size else None),
        "wall_s": time.time() - t0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--horizon-s", type=float, default=3.0)
    parser.add_argument("--only", type=str, default=None,
                        help="comma-separated realization labels")
    parser.add_argument("--prep-flexion", type=float, default=-0.5)
    parser.add_argument("--prep-gain-scale", type=float, default=2.0)
    parser.add_argument("--prep-ankle-offset", type=float, default=0.2)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    xml_before = model_xml()
    config = V3LandingConfig()
    config.prep_flexion_target_rad = args.prep_flexion
    config.prep_position_gain_scale = args.prep_gain_scale
    config.prep_ankle_offset_rad = args.prep_ankle_offset
    only = set(args.only.split(",")) if args.only else None
    report = {
        "authority_id": V3_REDUCED_FOOT_AUDIT_AUTHORITY_ID,
        "mission": "RES86_TOUCHDOWN_CONFIGURATION_AND_ABSORPTION_RESOLUTION_001",
        "audit_only": True,
        "plant_change_proposed": False,
        "sealed_xml_unmutated": None,
        "landing_config": {
            "prep_flexion_target_rad": config.prep_flexion_target_rad,
            "prep_position_gain_scale": config.prep_position_gain_scale,
            "prep_ankle_offset_rad": config.prep_ankle_offset_rad,
        },
        "realizations": [],
    }
    for realization in LATTICE:
        if only is not None and realization.label not in only:
            continue
        print(f"realization {realization.label} ...", flush=True)
        row = run_realization(realization, horizon_s=args.horizon_s,
                              landing_config=config)
        report["realizations"].append(row)
        print(json.dumps({k: row[k] for k in (
            "label", "status", "fault", "landing_samples", "e10_confirmed_time_s",
            "ankle_max_rad", "ankle_rom_margin_min_rad", "midfoot_max_abs_rad",
            "hard_gate_values")}, indent=2, default=str), flush=True)
    report["sealed_xml_unmutated"] = bool(model_xml() == xml_before)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
