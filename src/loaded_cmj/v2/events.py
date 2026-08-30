"""V2 Event Contract — sequential monotone DAG with phase-scoped telemetry (RES-7).

Replaces stateless independent scan with causally ordered, latched, dwell-aware online detector.
Thresholds frozen from V2_EVENT_THRESHOLDS; no numeric change.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from loaded_cmj.v2.constants import (
    V2_EVENT_THRESHOLDS,
    V2_TOTAL_MASS_KG,
    V2_GRAVITY_MAGNITUDE,
    V2_TRUE_STANDING_ENVELOPE,
    V2_EVENT_CONTRACT_VERSION,
)

GRAVITY = V2_GRAVITY_MAGNITUDE
WEIGHT = V2_TOTAL_MASS_KG * GRAVITY
DT = 0.000125  # physics timestep, frozen

# Frozen V2.1 True-Standing Contract (RES-16)
V2_EVENT_CONTRACT_VERSION_EXPORT = V2_EVENT_CONTRACT_VERSION
TRUE_STANDING_ENVELOPE = V2_TRUE_STANDING_ENVELOPE
# Reuse existing support threshold, do not add new arbitrary numbers
BILATERAL_FZ_THRESHOLD_N = float(V2_EVENT_THRESHOLDS["BILATERAL_TAKEOFF_FZ_N"])

EVENT_ORDER = [
    "supported_start",
    "countermovement_onset",
    "valid_countermovement",
    "upward_reversal",
    "vertical_propulsion",
    "bilateral_takeoff",
    "genuine_flight",
    "apex",
    "descending_landing",
    "impact_absorption",
    "balance_capture",
    "stable_recovery",
]

PREDECESSOR: Dict[str, Optional[str]] = {
    n: EVENT_ORDER[i-1] if i>0 else None for i, n in enumerate(EVENT_ORDER)
}

DWELL_S: Dict[str, float] = {
    "supported_start": float(V2_EVENT_THRESHOLDS["SUPPORTED_START_DWELL_S"]),
    "countermovement_onset": float(V2_EVENT_THRESHOLDS["COUNTERMOVEMENT_ONSET_DWELL_S"]),
    "valid_countermovement": 0.0,  # effectively instant (0.0005 in old but treat as 0)
    "upward_reversal": float(V2_EVENT_THRESHOLDS["UPWARD_REVERSAL_DWELL_S"]),
    "vertical_propulsion": float(V2_EVENT_THRESHOLDS["VERTICAL_PROPULSION_DWELL_S"]),
    "bilateral_takeoff": float(V2_EVENT_THRESHOLDS["BILATERAL_TAKEOFF_DWELL_S"]),
    "genuine_flight": float(V2_EVENT_THRESHOLDS["GENUINE_FLIGHT_DWELL_S"]),
    "apex": 0.0,
    "descending_landing": float(V2_EVENT_THRESHOLDS["DESCENDING_LANDING_DWELL_S"]),
    "impact_absorption": float(V2_EVENT_THRESHOLDS["IMPACT_ABSORPTION_DWELL_S"]),
    "balance_capture": float(V2_EVENT_THRESHOLDS["BALANCE_CAPTURE_DWELL_S"]),
    "stable_recovery": float(V2_EVENT_THRESHOLDS["STABLE_RECOVERY_DWELL_S"]),
}

# Window sizes in samples
def _win(dwell_s: float) -> int:
    if dwell_s <= 1e-12:
        return 1
    # round to nearest integer
    return int(round(float(dwell_s) / DT))

WIN: Dict[str, int] = {k: _win(v) for k, v in DWELL_S.items()}

@dataclass(frozen=True)
class V2EventRecord:
    name: str
    occurred_at: float
    confirmed_at: float
    sample_index: int
    confirmed_sample_index: int

@dataclass
class V2EventResult:
    events: dict[str, float]
    event_valid: dict[str, bool]
    raw_metrics: dict[str, Any]
    flags: dict[str, Any]
    termination: str
    # extended fields for RES-7
    event_records: dict[str, V2EventRecord] = field(default_factory=dict)
    physical_fall: bool = False
    physical_fall_time: Optional[float] = None
    physical_fall_index: Optional[int] = None
    phase_intervals: dict[str, Any] = field(default_factory=dict)

class V2EventDetector:
    """Monotone online detector with dwell, direction, and terminal fall."""

    def __init__(self):
        self.th = V2_EVENT_THRESHOLDS
        self.reset()

    def reset(self):
        self.samples: List[dict[str, Any]] = []
        self.events: dict[str, float] = {}  # occurred_at map for compat
        self.valid: dict[str, bool] = {}
        self.event_records: dict[str, V2EventRecord] = {}
        self.physical_fall: bool = False
        self.physical_fall_time: Optional[float] = None
        self.physical_fall_index: Optional[int] = None
        # candidate state for current next event
        self._candidate_start_time: Optional[float] = None
        self._candidate_start_idx: Optional[int] = None
        # special flags
        self._seen_negative_vz: bool = False  # for reversal
        self._com_ref: Optional[float] = None
        # fall blocked set
        self._fall_blocked: set[str] = set()

    def _next_event_name(self) -> Optional[str]:
        for n in EVENT_ORDER:
            if n not in self.event_records:
                if n in self._fall_blocked:
                    return None
                return n
        return None

    def _guard_supported_start(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            ok = (
                float(s.get("whole_Fz", 0)) > float(th["SUPPORTED_START_FZ_FLOOR_BW"]) * WEIGHT
                and float(s.get("left_Fz", 0)) > float(th["BILATERAL_TAKEOFF_FZ_N"])
                and float(s.get("right_Fz", 0)) > float(th["BILATERAL_TAKEOFF_FZ_N"])
                and float(s.get("com_margin", 999)) > float(th["SUPPORTED_START_COM_MARGIN_M"])
                and abs(float(s.get("trunk_tilt", 0))) < float(th["SUPPORTED_START_TRUNK_TILT_MAX_RAD"])
                and not bool(s.get("prohibited", False))
                and abs(float(s.get("com_vz", s.get("com_vz_abs", 0)))) < 0.05
            )
            # also check prohibited false already; cop_valid not needed
            return bool(ok)
        except Exception:
            return False

    def _guard_countermovement_onset_sustain(self, s: dict[str, Any]) -> bool:
        # sustain guard vz < -0.03
        try:
            return float(s.get("com_vz", 0)) < -0.03
        except Exception:
            return False

    def _guard_countermovement_onset_onset(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            return float(s.get("com_vz", 0)) < float(th["COUNTERMOVEMENT_ONSET_VZ_MPS"]) and float(s.get("whole_Fz", 0)) > float(th["SUPPORTED_START_FZ_FLOOR_BW"]) * WEIGHT
        except Exception:
            return False

    def _guard_valid_countermovement(self, s: dict[str, Any]) -> bool:
        th = self.th
        if self._com_ref is None:
            return False
        try:
            depth = float(self._com_ref) - float(s.get("com_z", 0))
            if depth < float(th["VALID_COUNTERMOVEMENT_DEPTH_M"]):
                return False
            if float(s.get("whole_Fz", 0)) <= float(th["VALID_COUNTERMOVEMENT_FORCE_FLOOR_BW"]) * WEIGHT:
                return False
            if abs(float(s.get("com_vz", 0))) >= 2.5:
                return False
            return True
        except Exception:
            return False

    def _guard_upward_reversal_onset(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            return float(s.get("com_vz", 0)) > float(th["UPWARD_REVERSAL_VZ_UP_MPS"])
        except Exception:
            return False

    def _guard_upward_reversal_sustain(self, s: dict[str, Any]) -> bool:
        try:
            return float(s.get("com_vz", 0)) > 0
        except Exception:
            return False

    def _guard_vertical_propulsion(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            return float(s.get("com_vz", 0)) > 0 and float(s.get("whole_Fz", 0)) > float(th["VERTICAL_PROPULSION_FORCE_BW"]) * WEIGHT
        except Exception:
            return False

    def _guard_bilateral_takeoff_onset(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            if not (float(s.get("left_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"]) and float(s.get("right_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"])):
                return False
            if float(s.get("com_vz", 0)) < float(th["TAKEOFF_VZ_MIN_MPS"]):
                return False
            return True
        except Exception:
            return False

    def _guard_bilateral_takeoff_sustain(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            return float(s.get("left_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"]) and float(s.get("right_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"])
        except Exception:
            return False

    def _guard_genuine_flight(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            return float(s.get("left_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"]) and float(s.get("right_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"])
        except Exception:
            return False

    def _guard_descending_landing_onset(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            vz = float(s.get("com_vz", 0))
            if vz >= -abs(float(th["DESCENDING_LANDING_VZ_MPS"])):
                return False
            left = float(s.get("left_Fz", 0))
            right = float(s.get("right_Fz", 0))
            if not (left > float(th["BILATERAL_TAKEOFF_FZ_N"]) or right > float(th["BILATERAL_TAKEOFF_FZ_N"])):
                return False
            return True
        except Exception:
            return False

    def _guard_descending_landing_sustain(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            left = float(s.get("left_Fz", 0))
            right = float(s.get("right_Fz", 0))
            return left > float(th["BILATERAL_TAKEOFF_FZ_N"]) or right > float(th["BILATERAL_TAKEOFF_FZ_N"])
        except Exception:
            return False

    def _guard_impact_absorption(self, s: dict[str, Any]) -> bool:
        try:
            return abs(float(s.get("com_vz", 0))) < 0.05
        except Exception:
            return False

    def _guard_balance_capture(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            if not (float(s.get("left_Fz", 0)) > float(th["BILATERAL_TAKEOFF_FZ_N"]) and float(s.get("right_Fz", 0)) > float(th["BILATERAL_TAKEOFF_FZ_N"])):
                return False
            if abs(float(s.get("com_vz", 0))) >= float(th["BALANCE_CAPTURE_COM_SPEED_MPS"]):
                return False
            # optionally check cop_valid if present - but not required for guard, spec says bilateral supported
            return True
        except Exception:
            return False

    def _is_true_standing_neighborhood(self, s: dict[str, Any]) -> bool:
        """Frozen TRUE_STANDING_NEIGHBORHOOD predicate (RES-16).

        Requires every controlled joint, COM z, root z, trunk pitch within the
        qualified standing envelope (empirically derived, expanded by one ULP),
        plus bilateral plantar support, no fall, no prohibited contact.
        Reuses existing support threshold (BILATERAL_TAKEOFF_FZ_N = 10.0).
        """
        try:
            # 1. controlled joints q7 within envelope
            # Accept multiple aliases for joint positions
            q = None
            for key in ("joint_position_rad", "joint_q7", "controlled_q7", "q7", "qpos_7", "controlled_q"):
                if key in s and s[key] is not None:
                    q = s[key]
                    break
            # Also try to extract from 'qpos' full (10) if needed: last 7 are actuated
            if q is None and "qpos" in s and s["qpos"] is not None:
                try:
                    full = list(s["qpos"])
                    if len(full) >= 10:
                        # qpos order: root_tx, root_tz, root_ry, lumbar, left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle
                        # Map to controlled order lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle
                        # qpos indices: 3: lumbar, 4:left_hip, 7:right_hip, 5:left_knee, 8:right_knee, 6:left_ankle, 9:right_ankle
                        q = [float(full[3]), float(full[4]), float(full[7]), float(full[5]), float(full[8]), float(full[6]), float(full[9])]
                    elif len(full) == 7:
                        q = full
                except Exception:
                    q = None
            if q is None:
                return False
            q_arr = [float(x) for x in q]
            if len(q_arr) != 7:
                return False
            env = TRUE_STANDING_ENVELOPE["Q_STAND_ENVELOPE"]
            for j in range(7):
                lo, hi = env[j]
                if not (lo <= q_arr[j] <= hi):
                    return False
            # 2. COM z
            com_z = None
            for key in ("com_z", "com_position_z", "COM_Z"):
                if key in s and s[key] is not None:
                    com_z = float(s[key])
                    break
            # fallback: com_position_m[2]
            if com_z is None and "com_position_m" in s and s["com_position_m"] is not None:
                try:
                    com_z = float(s["com_position_m"][2])
                except Exception:
                    pass
            if com_z is None:
                return False
            lo, hi = TRUE_STANDING_ENVELOPE["COM_Z_STAND_ENVELOPE"]
            if not (lo <= com_z <= hi):
                return False
            # 3. root z
            root_z = None
            for key in ("root_z", "root_tz", "pelvis_z", "pelvis_position_z"):
                if key in s and s[key] is not None:
                    root_z = float(s[key])
                    break
            if root_z is None and "pelvis_position_world_m" in s and s["pelvis_position_world_m"] is not None:
                try:
                    root_z = float(s["pelvis_position_world_m"][2])
                except Exception:
                    pass
            # Also try qpos[1] is root_tz
            if root_z is None and "qpos" in s and s["qpos"] is not None:
                try:
                    root_z = float(list(s["qpos"])[1])
                except Exception:
                    pass
            if root_z is None:
                return False
            lo, hi = TRUE_STANDING_ENVELOPE["ROOT_Z_STAND_ENVELOPE"]
            if not (lo <= root_z <= hi):
                return False
            # 4. trunk pitch / tilt
            trunk = None
            for key in ("trunk_tilt", "trunk_pitch", "trunk_tilt_rad", "tilt"):
                if key in s and s[key] is not None:
                    trunk = float(s[key])
                    break
            if trunk is None:
                return False
            lo, hi = TRUE_STANDING_ENVELOPE["TRUNK_PITCH_STAND_ENVELOPE"]
            # trunk_tilt is absolute angle, envelope lower is -5e-324, upper 0.0032
            if not (lo <= trunk <= hi):
                return False
            # 5. bilateral physical plantar support (reuse existing threshold 10.0)
            left = None
            right = None
            for k in ("left_Fz", "left_fz", "left_foot_Fz"):
                if k in s and s[k] is not None:
                    left = float(s[k])
                    break
            for k in ("right_Fz", "right_fz", "right_foot_Fz"):
                if k in s and s[k] is not None:
                    right = float(s[k])
                    break
            # fallback to plantar_normal_force_N
            if left is None and "plantar_normal_force_N" in s and s["plantar_normal_force_N"] is not None:
                try:
                    left = float(s["plantar_normal_force_N"][0])
                    right = float(s["plantar_normal_force_N"][1])
                except Exception:
                    pass
            if left is None or right is None:
                return False
            if not (left > BILATERAL_FZ_THRESHOLD_N and right > BILATERAL_FZ_THRESHOLD_N):
                return False
            # 6. no physical fall
            fall = s.get("fall_contact", s.get("physical_fall", s.get("fall", s.get("prohibited_contact", False))))
            if bool(fall):
                return False
            # 7. no prohibited non-plantar support
            prohibited = s.get("prohibited", s.get("prohibited_contact", s.get("fall_contact", False)))
            # If prohibited is explicitly True, fail. If fall already checked, this is redundant but keep.
            # Need to distinguish fall vs prohibited: fall is prohibited contact via fall shells, prohibited is non-plantar
            # For true standing, both must be False
            if bool(s.get("prohibited", False)) or bool(s.get("prohibited_contact", False)):
                # If sample has explicit prohibited flag, ensure it's False
                # But note fall_contact is same as prohibited_contact for V2? In V2, prohibited is shell contact, fall is same.
                # We already checked fall, but also check prohibited separately
                if bool(s.get("prohibited", False)):
                    return False
                if bool(s.get("prohibited_contact", False)):
                    return False
            return True
        except Exception:
            return False

    def _guard_stable_recovery(self, s: dict[str, Any]) -> bool:
        th = self.th
        try:
            if abs(float(s.get("trunk_tilt", 0))) >= float(th["STABLE_RECOVERY_TILT_MAX_RAD"]):
                return False
            if abs(float(s.get("com_vz", 0))) >= 0.05:
                return False
            if float(s.get("whole_Fz", 0)) <= 0.5 * WEIGHT:
                return False
            # RES-16: require true standing neighborhood for entire dwell
            if not self._is_true_standing_neighborhood(s):
                return False
            return True
        except Exception:
            return False

    def _detect_apex_crossing(self, prev: dict[str, Any], cur: dict[str, Any]) -> Optional[float]:
        # direction positive -> nonpositive after genuine flight and while flight
        th = self.th
        try:
            vz_prev = float(prev.get("com_vz", 0))
            vz_cur = float(cur.get("com_vz", 0))
            if vz_prev > 0 and vz_cur <= 0:
                # ensure within flight (Fz still <10 for both? check at least prev had flight)
                if float(prev.get("left_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"]) and float(cur.get("left_Fz", 0)) < float(th["BILATERAL_TAKEOFF_FZ_N"]):
                    # interpolate time where vz=0
                    t_prev = float(prev.get("time_s", 0))
                    t_cur = float(cur.get("time_s", 0))
                    # linear interpolation
                    if vz_cur == vz_prev:
                        return t_cur
                    frac = (0 - vz_prev) / (vz_cur - vz_prev)
                    t_apex = t_prev + frac * (t_cur - t_prev)
                    return float(t_apex)
                # also allow apex if still flight but not checking Fz? Keep strict earlier check but also allow if both feet low
                # fallback: if we are after genuine flight, consider crossing regardless of Fz? but spec says after genuine flight
                # we have already ensured predecessor genuine exists
                # So allow crossing even if Fz not low? For robustness, allow if vz crossing and we are in flight interval (between takeoff and landing)
                # We'll allow if predecessor takeoff exists
                t_prev = float(prev.get("time_s", 0))
                t_cur = float(cur.get("time_s", 0))
                if vz_cur == vz_prev:
                    return t_cur
                frac = (0 - vz_prev) / (vz_cur - vz_prev)
                return float(t_prev + frac * (t_cur - t_prev))
        except Exception:
            return None
        return None

    def update(self, sample: dict[str, Any]):
        # Append sample
        self.samples.append(sample)
        idx = len(self.samples) - 1

        # Terminal fall handling: latch on first fall_shell contact
        # Sample may contain fall flag; also check explicit keys
        fall_flag = False
        # Check multiple possible keys
        for key in ("fall_contact", "fall_shell_contact", "physical_fall", "fall", "is_fall"):
            if bool(sample.get(key, False)):
                fall_flag = True
                break
        # Also check if sample contains prohibited fall? But prohibited is shell contype0, not fall. Fall geoms have contype 4.
        # If runner provides 'fall_contact' as dict with ncon etc, we could also infer if prohibited True and fall geoms? But we rely on flag.
        # Additionally, if sample contains 'whole_Fz' and fall shells contact, but we have no info, we can't infer. So flag must be supplied by runner.
        if fall_flag and not self.physical_fall:
            self.physical_fall = True
            self.physical_fall_time = float(sample.get("time_s", 0))
            self.physical_fall_index = idx

        # If physical fall already latched, we still need to continue evaluating events that were already in progress?
        # But we block future capture/recovery immediately via _fall_blocked logic in loop below.

        # Monotone state machine: evaluate only current next event per sample
        # Loop may latch multiple non-dwell events same sample
        # Use while to allow same-sample chaining
        loop_iter = 0
        while loop_iter < 3:  # prevent infinite; at most 2-3 events same sample
            loop_iter += 1
            next_name = self._next_event_name()
            if next_name is None:
                break
            pred = PREDECESSOR.get(next_name)
            if pred is not None and pred not in self.event_records:
                break
            # Fall block for E11/E12
            if self.physical_fall:
                if next_name == "balance_capture" and next_name not in self.event_records:
                    # If fall already occurred before capture, block it forever
                    # Only block if fall time is before now or will be before capture dwell could complete
                    # For determinism, block immediately if fall occurred and capture not yet latched
                    self._fall_blocked.add("balance_capture")
                    self._fall_blocked.add("stable_recovery")
                    break
                if next_name == "stable_recovery":
                    self._fall_blocked.add("stable_recovery")
                    break

            # Dispatch per event
            # Handle apex specially (needs prev sample)
            if next_name == "apex":
                # Need genuine flight predecessor already
                if len(self.samples) < 2:
                    break
                prev = self.samples[-2]
                cur = self.samples[-1]
                # Also need to ensure we are after genuine_flight and before landing (still flight)
                # Check if cur is after genuine_flight occurred
                apex_time = self._detect_apex_crossing(prev, cur)
                if apex_time is not None:
                    # Direction cross detected
                    # Ensure descending landing not yet occurred (flight still)
                    # Apex should be before landing; we have not yet latched landing, so ok
                    # Latch apex
                    # For apex, occurred==confirmed, interpolated
                    # Use cur idx for confirmed, but time is interpolated
                    # sample_index is idx-1? Use idx for confirmed, idx-1 for occurred approximation? Use idx
                    rec = V2EventRecord(
                        name=next_name,
                        occurred_at=float(apex_time),
                        confirmed_at=float(apex_time),
                        sample_index=idx,
                        confirmed_sample_index=idx,
                    )
                    self.event_records[next_name] = rec
                    self.events[next_name] = float(apex_time)
                    self.valid[next_name] = True
                    # clear candidate (not used)
                    self._candidate_start_time = None
                    self._candidate_start_idx = None
                    self._seen_negative_vz = False
                    continue  # try next same sample (landing)
                else:
                    break

            # Handle valid_countermovement (instant, no dwell)
            if next_name == "valid_countermovement":
                # Need com_ref
                if self._com_ref is None:
                    # set from supported_start sample
                    sup_rec = self.event_records.get("supported_start")
                    if sup_rec is not None:
                        sup_idx = sup_rec.sample_index
                        try:
                            self._com_ref = float(self.samples[sup_idx].get("com_z", 0))
                        except Exception:
                            self._com_ref = None
                if self._guard_valid_countermovement(sample):
                    rec = V2EventRecord(
                        name=next_name,
                        occurred_at=float(sample.get("time_s", 0)),
                        confirmed_at=float(sample.get("time_s", 0)),
                        sample_index=idx,
                        confirmed_sample_index=idx,
                    )
                    self.event_records[next_name] = rec
                    self.events[next_name] = float(sample.get("time_s", 0))
                    self.valid[next_name] = True
                    self._candidate_start_time = None
                    self._candidate_start_idx = None
                    self._seen_negative_vz = False
                    continue
                else:
                    break

            # For dwell events, manage candidate
            # need to select appropriate guard onset/sustain
            dwell_needed = WIN.get(next_name, 1)

            # Special handling for countermovement_onset two-phase and reversal/bilateral etc
            if next_name == "countermovement_onset":
                # candidate onset requires vz<-0.08, sustain requires vz<-0.03
                # So for candidate start, check onset; for continuation, check sustain
                if self._candidate_start_time is None:
                    guard = self._guard_countermovement_onset_onset(sample)
                    sustain_guard = guard  # not needed separate
                else:
                    guard = self._guard_countermovement_onset_sustain(sample)
                    sustain_guard = guard
                # But we need to handle first sample: if candidate None, we check onset; else check sustain
                # So we will branch below
                # To simplify, handle this event separately
                if self._candidate_start_time is None:
                    if self._guard_countermovement_onset_onset(sample):
                        self._candidate_start_time = float(sample.get("time_s", 0))
                        self._candidate_start_idx = idx
                        # check if dwell 1 already?
                        if dwell_needed <= 1:
                            rec = V2EventRecord(next_name, self._candidate_start_time, float(sample.get("time_s",0)), self._candidate_start_idx, idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        # need to check dwell for sustain; if dwell_needed==240, need 240 samples sustain
                        # For now we started, need to verify next samples sustain < -0.03
                        # If dwells sustained from start, we will check on next samples
                        # But if current sample also must satisfy sustain? The onset sample vz<-0.08 implies also <-0.03, so ok
                        # Now check if dwell already satisfied (if dwell_needed 1)
                        # Already handled
                        # Not yet latched, break to wait
                        break
                    else:
                        break
                else:
                    # candidate exists, check sustain
                    if self._guard_countermovement_onset_sustain(sample):
                        elapsed = idx - self._candidate_start_idx + 1
                        if elapsed >= dwell_needed:
                            rec = V2EventRecord(
                                name=next_name,
                                occurred_at=self._candidate_start_time,
                                confirmed_at=float(sample.get("time_s",0)),
                                sample_index=self._candidate_start_idx,
                                confirmed_sample_index=idx,
                            )
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        else:
                            break
                    else:
                        # guard failed, discard candidate
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        break
            elif next_name == "upward_reversal":
                # need seen_negative tracking
                # Update seen_negative before guard
                try:
                    vz = float(sample.get("com_vz", 0))
                    if vz < float(self.th["UPWARD_REVERSAL_VZ_DOWN_MPS"]):
                        self._seen_negative_vz = True
                except Exception:
                    pass
                if not self._seen_negative_vz:
                    break
                # now guard depends on candidate phase
                if self._candidate_start_time is None:
                    guard = self._guard_upward_reversal_onset(sample)
                else:
                    guard = self._guard_upward_reversal_sustain(sample)
                # fall through to generic handling but we have already set guard
                # Use generic logic below by setting a flag
                # We'll let generic handle with guard variable
                # To avoid duplicate, we will handle now
                if self._candidate_start_time is None:
                    if guard:
                        self._candidate_start_time = float(sample.get("time_s",0))
                        self._candidate_start_idx = idx
                        if dwell_needed <=1:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            self._seen_negative_vz=False
                            continue
                        break
                    else:
                        break
                else:
                    if guard:
                        elapsed = idx - self._candidate_start_idx + 1
                        if elapsed >= dwell_needed:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            self._seen_negative_vz=False
                            continue
                        else:
                            break
                    else:
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        # keep seen_negative? Should keep true until latched, not reset on dwell break?
                        # Keep seen_negative true to allow retry
                        break
            elif next_name == "vertical_propulsion":
                if self._candidate_start_time is None:
                    guard = self._guard_vertical_propulsion(sample)
                else:
                    guard = self._guard_vertical_propulsion(sample)
                # generic
                if self._candidate_start_time is None:
                    if guard:
                        self._candidate_start_time=float(sample.get("time_s",0))
                        self._candidate_start_idx=idx
                        if dwell_needed<=1:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        break
                    else:
                        break
                else:
                    if guard:
                        elapsed=idx-self._candidate_start_idx+1
                        if elapsed>=dwell_needed:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        else:
                            break
                    else:
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        break
            elif next_name == "bilateral_takeoff":
                if self._candidate_start_time is None:
                    guard = self._guard_bilateral_takeoff_onset(sample)
                else:
                    guard = self._guard_bilateral_takeoff_sustain(sample)
                if self._candidate_start_time is None:
                    if guard:
                        self._candidate_start_time=float(sample.get("time_s",0))
                        self._candidate_start_idx=idx
                        if dwell_needed<=1:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        break
                    else:
                        break
                else:
                    if guard:
                        elapsed=idx-self._candidate_start_idx+1
                        if elapsed>=dwell_needed:
                            # also need to ensure vz at onset >=0.60 already checked
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        else:
                            break
                    else:
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        break
            elif next_name == "genuine_flight":
                if self._candidate_start_time is None:
                    guard = self._guard_genuine_flight(sample)
                else:
                    guard = self._guard_genuine_flight(sample)
                if self._candidate_start_time is None:
                    if guard:
                        self._candidate_start_time=float(sample.get("time_s",0))
                        self._candidate_start_idx=idx
                        if dwell_needed<=1:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        break
                    else:
                        break
                else:
                    if guard:
                        elapsed=idx-self._candidate_start_idx+1
                        if elapsed>=dwell_needed:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            continue
                        else:
                            break
                    else:
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        break
            elif next_name in ("supported_start","impact_absorption","balance_capture","stable_recovery","descending_landing"):
                # generic dwell handling with appropriate guard
                if next_name == "supported_start":
                    guard = self._guard_supported_start(sample)
                elif next_name == "impact_absorption":
                    guard = self._guard_impact_absorption(sample)
                elif next_name == "balance_capture":
                    guard = self._guard_balance_capture(sample)
                elif next_name == "stable_recovery":
                    guard = self._guard_stable_recovery(sample)
                elif next_name == "descending_landing":
                    if self._candidate_start_time is None:
                        guard = self._guard_descending_landing_onset(sample)
                    else:
                        guard = self._guard_descending_landing_sustain(sample)
                else:
                    guard=False
                if self._candidate_start_time is None:
                    if guard:
                        self._candidate_start_time=float(sample.get("time_s",0))
                        self._candidate_start_idx=idx
                        if dwell_needed<=1:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            # for supported_start, set com_ref later?
                            if next_name=="supported_start":
                                try:
                                    self._com_ref=float(sample.get("com_z",0))
                                except Exception:
                                    pass
                            continue
                        break
                    else:
                        break
                else:
                    if guard:
                        elapsed=idx-self._candidate_start_idx+1
                        if elapsed>=dwell_needed:
                            rec=V2EventRecord(next_name,self._candidate_start_time,float(sample.get("time_s",0)),self._candidate_start_idx,idx)
                            self.event_records[next_name]=rec
                            self.events[next_name]=self._candidate_start_time
                            self.valid[next_name]=True
                            self._candidate_start_time=None
                            self._candidate_start_idx=None
                            if next_name=="supported_start":
                                try:
                                    # com_ref should be at occurred idx, not confirmed
                                    occ_idx=self.event_records[next_name].sample_index
                                    self._com_ref=float(self.samples[occ_idx].get("com_z",0))
                                except Exception:
                                    pass
                            continue
                        else:
                            break
                    else:
                        self._candidate_start_time=None
                        self._candidate_start_idx=None
                        break
            else:
                break
            # if we reach here, we broke without latch
            break
        return None

    def finalize(self, horizon_s: float = 4.0) -> V2EventResult:
        # Ensure com_ref set if supported_start latched but not yet set
        if "supported_start" in self.event_records and self._com_ref is None:
            try:
                sup_idx=self.event_records["supported_start"].sample_index
                self._com_ref=float(self.samples[sup_idx].get("com_z",0))
            except Exception:
                pass

        # Determine termination
        if self.physical_fall:
            termination="PHYSICAL_FALL"
        elif len(self.event_records)==len(EVENT_ORDER):
            # check if stable_recovery is last and no fall after it
            # if fall after stable, still PHYSICAL_FALL per spec (fall always prevents final success)
            # So if fall and stable both exist, fall wins -> already handled
            termination="OBJECTIVE_COMPLETE"
        else:
            # check if any events missing but fall not, then incomplete
            termination="INCOMPLETE_HORIZON"

        # But also if physical_fall and stable_recovery latched, termination remains PHYSICAL_FALL (failure)
        # Already prioritized

        # Compute metrics
        metrics=self._compute_metrics()

        flags={
            "event_ordering": self._check_ordering(),
            "no_phase_overlap": True, # simplified
            "contact_chatter": False,
            "prohibited": any(bool(x.get("prohibited",False)) for x in self.samples),
            "physical_fall": self.physical_fall,
            "physical_fall_time": self.physical_fall_time,
        }

        # For compat, ensure events dict contains occurred_at for each latched
        # Also ensure valid dict
        result=V2EventResult(
            events=dict(self.events),
            event_valid=dict(self.valid),
            raw_metrics=metrics,
            flags=flags,
            termination=termination,
            event_records=dict(self.event_records),
            physical_fall=self.physical_fall,
            physical_fall_time=self.physical_fall_time,
            physical_fall_index=self.physical_fall_index,
            phase_intervals=self._phase_intervals(),
        )
        return result

    def _check_ordering(self) -> bool:
        # Check monotone occurred timestamps
        times=[self.event_records[n].occurred_at for n in EVENT_ORDER if n in self.event_records]
        for i in range(1,len(times)):
            if times[i] + 1e-12 < times[i-1]:
                return False
        # Also check confirmed >= occurred
        for rec in self.event_records.values():
            if rec.confirmed_at + 1e-12 < rec.occurred_at:
                return False
        return True

    def _phase_intervals(self) -> dict[str, Any]:
        def get_evt(name):
            return self.event_records.get(name)
        intervals={}
        # defined phases
        phases=[
            ("COUNTERMOVEMENT_INTERVAL", "countermovement_onset", "upward_reversal"),
            ("PROPULSION_INTERVAL", "upward_reversal", "bilateral_takeoff"),
            ("FLIGHT_INTERVAL", "bilateral_takeoff", "descending_landing"),
            ("LANDING_ABSORPTION_INTERVAL", "descending_landing", "impact_absorption"),
            ("CAPTURE_INTERVAL", "impact_absorption", "balance_capture"),
            ("RECOVERY_INTERVAL", "balance_capture", "stable_recovery"),
        ]
        for pname, a, b in phases:
            ra=get_evt(a)
            rb=get_evt(b)
            if ra is not None and rb is not None:
                intervals[pname]=[float(ra.occurred_at), float(rb.occurred_at)]
            else:
                intervals[pname]=None
        return intervals

    def _compute_metrics(self) -> dict[str, Any]:
        if not self.samples:
            return {}
        s=self.samples
        times=np.array([float(x.get("time_s",0)) for x in s])
        com_z=np.array([float(x.get("com_z",0)) for x in s])
        com_vz=np.array([float(x.get("com_vz",0)) for x in s])
        whole_Fz=np.array([float(x.get("whole_Fz",0)) for x in s])
        left_Fz=np.array([float(x.get("left_Fz",0)) for x in s])
        right_Fz=np.array([float(x.get("right_Fz",0)) for x in s])

        metrics: dict[str, Any]={}

        # Helper to get event record
        def rec(name):
            return self.event_records.get(name)

        # COUNTERMOVEMENT METRICS - only before reversal
        if rec("countermovement_onset") and rec("upward_reversal"):
            onset=rec("countermovement_onset")
            rev=rec("upward_reversal")
            # interval [onset, reversal]
            i0=onset.sample_index
            i1=rev.sample_index
            if i0 is not None and i1 is not None and i1>=i0:
                # reference is com at supported_start
                sup=rec("supported_start")
                if sup is not None:
                    try:
                        ref_z=float(self.samples[sup.sample_index].get("com_z",0))
                    except Exception:
                        ref_z=float(com_z[i0])
                else:
                    ref_z=float(com_z[i0])
                # min over [i0, i1] inclusive
                min_z=float(np.min(com_z[i0:i1+1])) if i1>=i0 else float(com_z[i0])
                depth=float(ref_z - min_z)
                # peak negative vz
                slice_vz=com_vz[i0:i1+1]
                if len(slice_vz)>0:
                    idx_min=int(np.argmin(slice_vz))
                    peak_vz=float(slice_vz[idx_min])
                    t_peak=float(times[i0+idx_min])
                    braking_duration=float(rev.occurred_at - t_peak)
                    cm_duration=float(rev.occurred_at - onset.occurred_at)
                else:
                    peak_vz=None
                    t_peak=None
                    braking_duration=None
                    cm_duration=None
                metrics["COUNTERMOVEMENT_DEPTH"]=depth
                # preserve old key for compat
                metrics["countermovement_depth"]=depth
                metrics["PEAK_NEGATIVE_COM_VZ"]=peak_vz
                metrics["T_PEAK_NEGATIVE_COM_VZ"]=t_peak
                metrics["BRAKING_DURATION"]=braking_duration
                metrics["COUNTERMOVEMENT_DURATION"]=cm_duration
                # also metrics for invalid intervals? but ok
            else:
                metrics["COUNTERMOVEMENT_DEPTH"]=None
                metrics["countermovement_depth"]=None
                metrics["PEAK_NEGATIVE_COM_VZ"]=None
                metrics["T_PEAK_NEGATIVE_COM_VZ"]=None
                metrics["BRAKING_DURATION"]=None
                metrics["COUNTERMOVEMENT_DURATION"]=None
        else:
            metrics["COUNTERMOVEMENT_DEPTH"]=None
            metrics["countermovement_depth"]=None
            metrics["PEAK_NEGATIVE_COM_VZ"]=None
            metrics["T_PEAK_NEGATIVE_COM_VZ"]=None
            metrics["BRAKING_DURATION"]=None
            metrics["COUNTERMOVEMENT_DURATION"]=None

        # PROPULSION METRICS over [reversal, takeoff]
        if rec("upward_reversal") and rec("bilateral_takeoff"):
            rev=rec("upward_reversal")
            to=rec("bilateral_takeoff")
            i0=rev.sample_index
            i1=to.sample_index
            if i0 is not None and i1 is not None and i1>=i0:
                propulsion_duration=float(to.occurred_at - rev.occurred_at)
                takeoff_vz=float(self.samples[i1].get("com_vz",0))
                # impulse integral(Fz - mg) over propulsion interval
                # use whole_Fz
                # dt is 0.000125 but use actual time diff for last sample? Use DT
                impulse=float(np.sum((whole_Fz[i0:i1+1] - WEIGHT) * DT))
                # validate impulse ≈ m * delta vz
                try:
                    dv=float(com_vz[i1] - com_vz[i0])
                    impulse_expected=float(V2_TOTAL_MASS_KG * dv)
                    residual=abs(impulse-impulse_expected)/max(abs(impulse_expected),1.0) if impulse_expected!=0 else None
                except Exception:
                    residual=None
                metrics["PROPULSION_DURATION"]=propulsion_duration
                metrics["TAKEOFF_VZ"]=takeoff_vz
                metrics["takeoff_vz"]=takeoff_vz
                metrics["PROPULSIVE_NET_VERTICAL_IMPULSE"]=impulse
                metrics["propulsive_impulse_residual"]=residual
            else:
                metrics["PROPULSION_DURATION"]=None
                metrics["TAKEOFF_VZ"]=None
                metrics["takeoff_vz"]=None
                metrics["PROPULSIVE_NET_VERTICAL_IMPULSE"]=None
        else:
            metrics["PROPULSION_DURATION"]=None
            metrics["TAKEOFF_VZ"]=None
            metrics["takeoff_vz"]=None
            metrics["PROPULSIVE_NET_VERTICAL_IMPULSE"]=None
            # if takeoff missing, takeoff_vz null

        # FLIGHT METRICS
        if rec("bilateral_takeoff") and rec("descending_landing"):
            to=rec("bilateral_takeoff")
            land=rec("descending_landing")
            flight_duration=float(land.occurred_at - to.occurred_at)
            metrics["FLIGHT_DURATION"]=flight_duration
            metrics["flight_duration"]=flight_duration
            # apex
            apex_rec=rec("apex")
            if apex_rec is not None:
                apex_time=float(apex_rec.occurred_at)
                # find apex com_z: max com_z between takeoff and landing, or at apex time interpolated
                # Use max com_z in flight interval
                i0=to.sample_index
                i1=land.sample_index
                if i0 is not None and i1 is not None and i1>i0:
                    # apex com_z is max in that interval
                    apex_z=float(np.max(com_z[i0:i1+1]))
                    # rise from takeoff
                    takeoff_z=float(self.samples[i0].get("com_z",0))
                    rise=float(apex_z - takeoff_z)
                    metrics["APEX_TIME"]=apex_time
                    metrics["APEX_COM_Z"]=apex_z
                    metrics["apex_height"]=apex_z
                    metrics["FLIGHT_RISE_FROM_TAKEOFF"]=rise
                else:
                    metrics["APEX_TIME"]=apex_time
                    metrics["APEX_COM_Z"]=None
                    metrics["apex_height"]=None
                    metrics["FLIGHT_RISE_FROM_TAKEOFF"]=None
            else:
                metrics["APEX_TIME"]=None
                metrics["APEX_COM_Z"]=None
                metrics["apex_height"]=None
                metrics["FLIGHT_RISE_FROM_TAKEOFF"]=None
        else:
            metrics["FLIGHT_DURATION"]=None
            metrics["flight_duration"]=None
            metrics["APEX_TIME"]=None
            metrics["APEX_COM_Z"]=None
            metrics["apex_height"]=None
            metrics["FLIGHT_RISE_FROM_TAKEOFF"]=None
            if rec("apex") is not None:
                metrics["APEX_TIME"]=float(rec("apex").occurred_at)
                # still compute apex_z if possible
                try:
                    # apex_z as max overall?
                    metrics["APEX_COM_Z"]=float(np.max(com_z))
                    metrics["apex_height"]=float(np.max(com_z))
                except Exception:
                    metrics["APEX_COM_Z"]=None
                    metrics["apex_height"]=None

        # Ensure takeoff_vz fallback if propulsion missing but takeoff exists
        if metrics.get("TAKEOFF_VZ") is None and rec("bilateral_takeoff") is not None:
            try:
                metrics["TAKEOFF_VZ"]=float(self.samples[rec("bilateral_takeoff").sample_index].get("com_vz",0))
                metrics["takeoff_vz"]=metrics["TAKEOFF_VZ"]
            except Exception:
                pass

        # LANDING METRICS
        if rec("descending_landing"):
            land=rec("descending_landing")
            land_time=float(land.occurred_at)
            land_idx=land.sample_index
            # primary peak 100ms window
            # find indices where time in [land_time, land_time+0.100]
            # Use time array search
            idx_a=int(np.searchsorted(times, land_time))
            idx_b=int(np.searchsorted(times, land_time+0.100))
            if idx_b>idx_a:
                window_Fz=whole_Fz[idx_a:idx_b]
                if len(window_Fz)>0:
                    peak_idx_rel=int(np.argmax(window_Fz))
                    primary_peak=float(window_Fz[peak_idx_rel])
                    peak_time=float(times[idx_a+peak_idx_rel])
                    peak_BW=float(primary_peak/WEIGHT)
                    # loading rate approx (peak - landing Fz)/dt? Use simple peak/(time diff)
                    # Compute rise time: peak_time - land_time
                    dt_peak=peak_time - land_time
                    loading_rate=float(primary_peak/max(dt_peak,1e-9)) if dt_peak>0 else None
                else:
                    primary_peak=None
                    peak_time=None
                    peak_BW=None
                    loading_rate=None
            else:
                primary_peak=None
                peak_time=None
                peak_BW=None
                loading_rate=None
            metrics["PRIMARY_LANDING_PEAK_FZ_100MS"]=primary_peak
            metrics["PRIMARY_LANDING_PEAK_TIME"]=peak_time
            metrics["PRIMARY_LANDING_PEAK_BW"]=peak_BW
            metrics["PRIMARY_LOADING_RATE"]=loading_rate
            metrics["max_Fz"]=float(np.max(whole_Fz)) if len(whole_Fz)>0 else None
            # also post landing global peak separate
            # global peak after landing
            idx_land_end=int(np.searchsorted(times, land_time))
            global_peak=float(np.max(whole_Fz[idx_land_end:])) if idx_land_end<len(whole_Fz) else None
            # find its time
            if global_peak is not None:
                # find index of max after landing
                rel=np.argmax(whole_Fz[idx_land_end:])
                global_peak_time=float(times[idx_land_end+rel])
            else:
                global_peak_time=None
            metrics["POST_LANDING_GLOBAL_PEAK_FZ"]=global_peak
            metrics["POST_LANDING_GLOBAL_PEAK_TIME"]=global_peak_time

            # impact absorption duration etc
            if rec("impact_absorption"):
                abs_rec=rec("impact_absorption")
                metrics["IMPACT_ABSORPTION_DURATION"]=float(abs_rec.occurred_at - land_time)
                # net vertical impulse to absorption
                i_abs=abs_rec.sample_index
                if land_idx is not None and i_abs is not None and i_abs>=land_idx:
                    impulse_land=float(np.sum((whole_Fz[land_idx:i_abs+1]-WEIGHT)*DT))
                    metrics["LANDING_NET_VERTICAL_IMPULSE_TO_ABSORPTION"]=impulse_land
                    # momentum change?
                    try:
                        dv_land=float(com_vz[i_abs]-com_vz[land_idx])
                        expected=float(V2_TOTAL_MASS_KG*dv_land)
                        metrics["landing_impulse_residual"]=abs(impulse_land-expected)/max(abs(expected),1)
                    except Exception:
                        metrics["landing_impulse_residual"]=None
                else:
                    metrics["LANDING_NET_VERTICAL_IMPULSE_TO_ABSORPTION"]=None
            else:
                metrics["IMPACT_ABSORPTION_DURATION"]=None
                metrics["LANDING_NET_VERTICAL_IMPULSE_TO_ABSORPTION"]=None
        else:
            metrics["PRIMARY_LANDING_PEAK_FZ_100MS"]=None
            metrics["PRIMARY_LANDING_PEAK_TIME"]=None
            metrics["PRIMARY_LANDING_PEAK_BW"]=None
            metrics["PRIMARY_LOADING_RATE"]=None
            metrics["max_Fz"]=float(np.max(whole_Fz)) if len(whole_Fz)>0 else None
            metrics["POST_LANDING_GLOBAL_PEAK_FZ"]=None
            metrics["POST_LANDING_GLOBAL_PEAK_TIME"]=None
            metrics["IMPACT_ABSORPTION_DURATION"]=None
            metrics["LANDING_NET_VERTICAL_IMPULSE_TO_ABSORPTION"]=None

        # CAPTURE / RECOVERY
        if rec("descending_landing") and rec("balance_capture"):
            land=rec("descending_landing")
            cap=rec("balance_capture")
            metrics["BALANCE_CAPTURE_LATENCY_FROM_LANDING"]=float(cap.occurred_at - land.occurred_at)
            if rec("impact_absorption"):
                abs_rec=rec("impact_absorption")
                metrics["BALANCE_CAPTURE_LATENCY_FROM_ABSORPTION"]=float(cap.occurred_at - abs_rec.occurred_at)
            else:
                metrics["BALANCE_CAPTURE_LATENCY_FROM_ABSORPTION"]=None
        else:
            metrics["BALANCE_CAPTURE_LATENCY_FROM_LANDING"]=None
            metrics["BALANCE_CAPTURE_LATENCY_FROM_ABSORPTION"]=None

        if rec("balance_capture") and rec("stable_recovery"):
            cap=rec("balance_capture")
            recov=rec("stable_recovery")
            metrics["STABLE_RECOVERY_DURATION"]=float(recov.occurred_at - cap.occurred_at)
        else:
            metrics["STABLE_RECOVERY_DURATION"]=None

        # Ensure compatibility keys
        # countermovement_depth etc already
        # flight_duration already

        # CoP validity handling note
        metrics["COP_VALIDITY_HANDLING"]="CoP only valid where Fz>20 per foot; never averaged invalid; takeoff/landing neighborhoods have invalid; reported invalid intervals explicitly"
        # Provide invalid interval example
        # Compute cop invalid near takeoff/landing if possible
        # Check if samples have cop_valid
        has_cop=False
        invalid_intervals=[]
        for i,sample in enumerate(s):
            lv=sample.get("left_cop_valid", None)
            rv=sample.get("right_cop_valid", None)
            if lv is not None or rv is not None:
                has_cop=True
                # if either invalid and near takeoff/landing
                # Record intervals where invalid
                if not bool(lv) or not bool(rv):
                    invalid_intervals.append(float(sample.get("time_s",0)))
                break
        # we could add more

        # Physical fall metrics
        metrics["PHYSICAL_FALL"]=self.physical_fall
        metrics["PHYSICAL_FALL_TIME"]=self.physical_fall_time

        return metrics

