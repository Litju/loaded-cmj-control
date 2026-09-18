#!/usr/bin/env python3
"""RES-86 landing development rollout harness (development evidence only).

Builds the exact RES-85 -> RES-86 handoff certificate once, then runs the
landing controller from that certificate and prints the engineering metrics
needed to decide the next deterministic change.  This tool is a development
instrument: final external qualification evidence is produced separately.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT / "src"), str(ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from loaded_cmj.v3.actuation import V3ActuationState, V3MtpLedgerEntry  # noqa: E402
from loaded_cmj.v3.landing_control import (  # noqa: E402
    V3LandingConfig,
    CONTROL_COORDINATES,
    CONTROL_INTERVAL_NATIVE_STEPS,
)
from loaded_cmj.v3.landing_runtime import (  # noqa: E402
    E8_STATE_SHA256,
    V3HandoffCertificate,
    build_handoff_certificate,
    run_landing_episode,
)

DEFAULT_CERT = Path("/tmp/opencode/res86/handoff_791.npz")


def save_certificate(path: Path, certificate: V3HandoffCertificate) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        state_vector=certificate.state_vector,
        ctrl_nm=certificate.ctrl_nm,
        qacc_warmstart=certificate.qacc_warmstart,
        previous_applied_nm=np.asarray(certificate.actuation.previous_applied_nm),
        mtp_active_left=np.array([certificate.actuation.mtp_ledger[0].active_positive_work_j]),
        mtp_active_right=np.array([certificate.actuation.mtp_ledger[1].active_positive_work_j]),
        mtp_total_left=np.array([certificate.actuation.mtp_ledger[0].total_positive_work_j]),
        mtp_total_right=np.array([certificate.actuation.mtp_ledger[1].total_positive_work_j]),
        step_index=np.array([certificate.actuation.step_index]),
        sample_index=np.array([certificate.sample_index]),
        time_s=np.array([certificate.time_s]),
    )


def load_certificate(path: Path) -> V3HandoffCertificate:
    data = np.load(path)
    ledger = (
        V3MtpLedgerEntry(active_positive_work_j=float(data["mtp_active_left"][0]),
                         total_positive_work_j=float(data["mtp_total_left"][0])),
        V3MtpLedgerEntry(active_positive_work_j=float(data["mtp_active_right"][0]),
                         total_positive_work_j=float(data["mtp_total_right"][0])),
    )
    state = V3ActuationState(
        previous_applied_nm=tuple(float(v) for v in data["previous_applied_nm"]),
        mtp_ledger=ledger,
        step_index=int(data["step_index"][0]),
    )
    certificate = V3HandoffCertificate(
        sample_index=int(data["sample_index"][0]),
        time_s=float(data["time_s"][0]),
        state_vector=np.asarray(data["state_vector"], dtype=np.float64),
        ctrl_nm=np.asarray(data["ctrl_nm"], dtype=np.float64),
        qacc_warmstart=np.asarray(data["qacc_warmstart"], dtype=np.float64),
        actuation=state,
        state_sha256="",
    )
    import hashlib
    object.__setattr__(certificate, "state_sha256",
                       hashlib.sha256(certificate.state_vector.tobytes()).hexdigest())
    return certificate


def landing_config_from_args(args) -> V3LandingConfig:
    config = V3LandingConfig()
    for name in ("t_stop_s", "t_capture_x_s", "t_capture_h_s", "t_attitude_s",
                 "a_z_max_m_s2", "fx_fraction_max", "linear_prediction_relative_tolerance",
                 "max_refinements", "solver_iterations", "cop_hull_margin_m"):
        value = getattr(args, name, None)
        if value is not None:
            setattr(config, name, value)
    if args.trust is not None:
        config.trust_region_coordinates_nm = tuple(float(v) for v in args.trust)
    if args.no_mtp:
        config.include_mtp_coordinate = False
    if args.config_json is not None:
        overrides = json.loads(Path(args.config_json).read_text())
        for name, value in overrides.items():
            if not hasattr(config, name):
                raise SystemExit(f"unknown V3LandingConfig field: {name}")
            setattr(config, name, value)
    return config


def print_report(episode) -> None:
    events = episode.events
    diagnostics = episode.diagnostics
    telemetry = episode.telemetry
    print(json.dumps({
        "status": episode.status,
        "fault": episode.fault,
        "handoff_state_sha256": episode.handoff.state_sha256,
        "e8_identity_ok": episode.handoff.state_sha256 == E8_STATE_SHA256,
        "launch_events": episode.launch_events,
        "d_est": events.get("d_est"),
        "e9_bilateral": events.get("e9_bilateral_established"),
        "e10": events.get("e10"),
        "window": {
            k: v for k, v in (events.get("first_contact_to_e10_window") or {}).items()
            if k != "gates"
        },
        "window_gates": (events.get("first_contact_to_e10_window") or {}).get("gates"),
        "hard_gates": (events.get("hard_gate_audit") or {}).get("gates"),
        "hard_gate_values": {
            k: v for k, v in (events.get("hard_gate_audit") or {}).items()
            if k in ("max_penetration_m", "peak_total_floor_fz_bw", "chatter_transitions",
                     "material_reflight_intervals", "min_rom_margin_rad",
                     "max_abs_moment_ratio", "max_abs_power_ratio")
        },
        "diagnostics": diagnostics,
        "telemetry": {
            "samples": int(telemetry.index.shape[0]),
            "fallbacks": int(np.count_nonzero(telemetry.fallback)),
            "failed_gate_events": sorted(set(
                str(v) for v in telemetry.failed_gate if v and str(v) != "")),
            "phases": sorted(set(str(v) for v in telemetry.phase_name)),
        },
    }, indent=2, default=str))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-handoff", action="store_true")
    parser.add_argument("--certificate", type=Path, default=DEFAULT_CERT)
    parser.add_argument("--horizon-s", type=float, default=3.5)
    parser.add_argument("--dump", type=Path, default=None)
    parser.add_argument("--t-stop-s", type=float, default=None)
    parser.add_argument("--t-capture-x-s", type=float, default=None)
    parser.add_argument("--t-capture-h-s", type=float, default=None)
    parser.add_argument("--t-attitude-s", type=float, default=None)
    parser.add_argument("--a-z-max-m-s2", type=float, default=None)
    parser.add_argument("--fx-fraction-max", type=float, default=None)
    parser.add_argument("--cop-hull-margin-m", type=float, default=None)
    parser.add_argument("--linear-prediction-relative-tolerance", type=float, default=None)
    parser.add_argument("--max-refinements", type=int, default=None)
    parser.add_argument("--solver-iterations", type=int, default=None)
    parser.add_argument("--trust", type=float, nargs=len(CONTROL_COORDINATES), default=None)
    parser.add_argument("--no-mtp", action="store_true")
    parser.add_argument("--config-json", type=Path, default=None)
    args = parser.parse_args()

    if args.build_handoff:
        t0 = time.time()
        certificate, events = build_handoff_certificate()
        save_certificate(args.certificate, certificate)
        print(json.dumps({"built": str(args.certificate),
                          "handoff_sample": certificate.sample_index,
                          "handoff_time_s": certificate.time_s,
                          "handoff_state_sha256": certificate.state_sha256,
                          "identity_ok": certificate.state_sha256 == E8_STATE_SHA256,
                          "events": events,
                          "wall_s": time.time() - t0}, indent=2, default=str))
        return

    certificate = load_certificate(args.certificate)
    config = landing_config_from_args(args)
    t0 = time.time()
    episode = run_landing_episode(horizon_s=args.horizon_s,
                                  handoff_certificate=certificate,
                                  landing_config=config)
    print_report(episode)
    print(f"wall_s={time.time() - t0:.2f}")
    if args.dump is not None:
        args.dump.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.dump, **{
            name: np.asarray(array, dtype=object)
            for name, array in episode.telemetry.arrays().items()})


if __name__ == "__main__":
    main()
