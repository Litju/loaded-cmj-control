"""V2 Event Contract — sagittal CMJ with new thresholds, frozen before controller.

Events (physically meaningful, monotone DAG):
supported_start, countermovement_onset, valid_countermovement, upward_reversal,
vertical_propulsion, bilateral_takeoff, genuine_flight, apex, descending_landing,
impact_absorption, balance_capture, stable_recovery
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np
from loaded_cmj.v2.constants import V2_EVENT_THRESHOLDS, V2_TOTAL_MASS_KG, V2_GRAVITY_MAGNITUDE

GRAVITY = V2_GRAVITY_MAGNITUDE
WEIGHT = V2_TOTAL_MASS_KG * GRAVITY

@dataclass
class V2EventResult:
    events: dict[str, float]
    event_valid: dict[str, bool]
    raw_metrics: dict[str, Any]
    flags: dict[str, Any]
    termination: str

class V2EventDetector:
    """Simple deterministic detector for V2 (physics-substep samples)."""
    def __init__(self):
        self.th = V2_EVENT_THRESHOLDS
        self.reset()

    def reset(self):
        self.samples: list[dict[str, Any]] = []
        self.events: dict[str, float] = {}
        self.valid: dict[str, bool] = {}

    def update(self, sample: dict[str, Any]):
        # sample must contain: time_s, com_z, com_vz, left_Fz, right_Fz, whole_Fz, com_margin, trunk_tilt, prohibited, qpos, qvel
        self.samples.append(sample)
        # we evaluate all events monotonically at end via finalize; live update just stores
        return None

    def finalize(self, horizon_s: float = 4.0) -> V2EventResult:
        s = self.samples
        if not s:
            return V2EventResult({}, {}, {}, {}, "INCOMPLETE_HORIZON")
        # Helpers
        def has_event(name, time):
            self.events[name]=float(time)
            self.valid[name]=True
        # 1 supported_start: stable bilateral support for 0.10 dwell while near standing (Fz>0.30*W, margin>0.02, tilt<0.1745, speed<0.05? We'll check Fz and margin and tilt)
        # Use whole_Fz >0.30*W and individual >10 and margin>0.02 and tilt<0.1745 for 0.10
        # Find first interval of 0.10 where condition holds
        th = self.th
        # Build time arrays
        times=np.array([x['time_s'] for x in s])
        # Find supported_start
        supported_start=None
        # Simple scan for 0.10 window where all samples in window satisfy
        win=int(th["SUPPORTED_START_DWELL_S"]/0.000125)
        for i in range(len(s)-win):
            ok=True
            for j in range(i,i+win):
                samp=s[j]
                if not (samp['whole_Fz'] > th["SUPPORTED_START_FZ_FLOOR_BW"]*WEIGHT and samp['left_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] and samp['right_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] and samp['com_margin']>th["SUPPORTED_START_COM_MARGIN_M"] and abs(samp['trunk_tilt'])<th["SUPPORTED_START_TRUNK_TILT_MAX_RAD"] and not samp['prohibited'] and samp['com_vz_abs']<0.05):
                    ok=False
                    break
            if ok:
                supported_start=times[i]
                has_event("supported_start", supported_start)
                break
        if supported_start is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {"prohibited": any(x['prohibited'] for x in s)}, "INCOMPLETE_HORIZON")
        # 2 countermovement_onset: sustained downward com vz < -0.08 for 0.03 after supported_start
        onset=None
        win_on=int(th["COUNTERMOVEMENT_ONSET_DWELL_S"]/0.000125)
        start_idx=np.searchsorted(times, supported_start)
        for i in range(start_idx, len(s)-win_on):
            if s[i]['com_vz'] < th["COUNTERMOVEMENT_ONSET_VZ_MPS"]:
                # check dwell: next win samples all < -0.03? Actually off threshold -0.03?
                ok=True
                for j in range(i,i+win_on):
                    if s[j]['com_vz'] > -0.03:
                        ok=False
                        break
                if ok and s[i]['whole_Fz']>th["SUPPORTED_START_FZ_FLOOR_BW"]*WEIGHT:
                    onset=times[i]
                    has_event("countermovement_onset", onset)
                    break
        if onset is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 3 valid_countermovement: COM descends >=0.10 from supported reference while bilateral support remains
        # Find COM at supported_start
        idx_supported=np.searchsorted(times, supported_start)
        com_ref=s[idx_supported]['com_z']
        valid_cm=None
        # Need dwell 0.0005 basically one sample where depth >=0.10 and force>0.30*W
        for i in range(idx_supported, len(s)):
            depth=com_ref - s[i]['com_z']
            if depth >= th["VALID_COUNTERMOVEMENT_DEPTH_M"] and s[i]['whole_Fz']>th["VALID_COUNTERMOVEMENT_FORCE_FLOOR_BW"]*WEIGHT:
                # check not too fast? max descent speed 2.5
                if abs(s[i]['com_vz']) < 2.5:
                    valid_cm=times[i]
                    has_event("valid_countermovement", valid_cm)
                    break
        if valid_cm is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 4 upward_reversal: com vz crosses -0.02 to +0.02 after valid_cm with dwell 0.01
        reversal=None
        idx_cm=np.searchsorted(times, valid_cm)
        # Find where vz goes from < -0.02 to > +0.02
        for i in range(idx_cm, len(s)-int(th["UPWARD_REVERSAL_DWELL_S"]/0.000125)):
            if s[i]['com_vz'] < th["UPWARD_REVERSAL_VZ_DOWN_MPS"]:
                # look ahead for up
                for k in range(i, min(len(s), i+int(0.5/0.000125))):
                    if s[k]['com_vz'] > th["UPWARD_REVERSAL_VZ_UP_MPS"]:
                        # check dwell of up
                        win=int(th["UPWARD_REVERSAL_DWELL_S"]/0.000125)
                        if k+win < len(s) and all(s[j]['com_vz']>0 for j in range(k,k+win)):
                            reversal=times[k]
                            has_event("upward_reversal", reversal)
                            break
                if reversal:
                    break
        if reversal is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 5 vertical_propulsion: positive com vz with Fz >1.05*W for 0.05
        propulsion=None
        idx_rev=np.searchsorted(times, reversal)
        win_prop=int(th["VERTICAL_PROPULSION_DWELL_S"]/0.000125)
        for i in range(idx_rev, len(s)-win_prop):
            if s[i]['com_vz']>0 and s[i]['whole_Fz']>th["VERTICAL_PROPULSION_FORCE_BW"]*WEIGHT:
                if all(s[j]['com_vz']>0 and s[j]['whole_Fz']>th["VERTICAL_PROPULSION_FORCE_BW"]*WEIGHT for j in range(i,i+win_prop)):
                    propulsion=times[i]
                    has_event("vertical_propulsion", propulsion)
                    break
        if propulsion is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 6 bilateral_takeoff: both Fz <10 for 0.01 and vz>=0.60
        takeoff=None
        takeoff_vz=None
        win_to=int(th["BILATERAL_TAKEOFF_DWELL_S"]/0.000125)
        for i in range(idx_rev, len(s)-win_to):
            if all(s[j]['left_Fz']<th["BILATERAL_TAKEOFF_FZ_N"] and s[j]['right_Fz']<th["BILATERAL_TAKEOFF_FZ_N"] for j in range(i,i+win_to)):
                # check vz at that window
                vz_here=s[i]['com_vz']
                if vz_here >= th["TAKEOFF_VZ_MIN_MPS"]:
                    takeoff=times[i]
                    takeoff_vz=vz_here
                    has_event("bilateral_takeoff", takeoff)
                    break
        if takeoff is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        idx_to=np.searchsorted(times, takeoff)
        # 7 genuine_flight: bilateral no-contact for 0.08
        flight=None
        win_f=int(th["GENUINE_FLIGHT_DWELL_S"]/0.000125)
        for i in range(idx_to, len(s)-win_f):
            if all(s[j]['left_Fz']<th["BILATERAL_TAKEOFF_FZ_N"] and s[j]['right_Fz']<th["BILATERAL_TAKEOFF_FZ_N"] for j in range(i,i+win_f)):
                flight=times[i]
                has_event("genuine_flight", flight)
                break
        if flight is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 8 apex: com vz crosses + to - during flight
        apex=None
        idx_f=np.searchsorted(times, flight)
        # find max com_z during flight
        # flight window is takeoff to landing, but we can just find where vz crosses 0
        for i in range(idx_to, len(s)-1):
            if s[i]['com_vz']>0 and s[i+1]['com_vz']<0:
                # ensure within flight (Fz still <10)
                if s[i]['left_Fz']<th["BILATERAL_TAKEOFF_FZ_N"]:
                    apex=times[i] + (0 - s[i]['com_vz'])/(s[i+1]['com_vz']-s[i]['com_vz'])*0.000125
                    has_event("apex", apex)
                    break
        if apex is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 9 descending_landing: plantar recontact while com descending (vz < -0.10) for 0.01
        landing=None
        win_l=int(th["DESCENDING_LANDING_DWELL_S"]/0.000125)
        idx_apex=np.searchsorted(times, apex)
        for i in range(idx_apex, len(s)-win_l):
            if s[i]['com_vz'] < -abs(th["DESCENDING_LANDING_VZ_MPS"]) and s[i]['left_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] or s[i]['right_Fz']>th["BILATERAL_TAKEOFF_FZ_N"]:
                # check dwell where at least one foot contact
                if all(s[j]['left_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] or s[j]['right_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] for j in range(i,i+win_l)):
                    landing=times[i]
                    has_event("descending_landing", landing)
                    break
        if landing is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 10 impact_absorption: landing impulse dissipated with dwell 0.02 where |vz|<0.05?
        absorption=None
        idx_land=np.searchsorted(times, landing)
        win_a=int(th["IMPACT_ABSORPTION_DWELL_S"]/0.000125)
        for i in range(idx_land, len(s)-win_a):
            if all(abs(s[j]['com_vz'])<0.05 for j in range(i,i+win_a)):
                absorption=times[i]
                has_event("impact_absorption", absorption)
                break
        if absorption is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 11 balance_capture: bilateral supported vertical motion near zero (com speed <0.30) for 0.15
        capture=None
        win_c=int(th["BALANCE_CAPTURE_DWELL_S"]/0.000125)
        for i in range(idx_land, len(s)-win_c):
            if all(abs(s[j]['com_vz'])<th["BALANCE_CAPTURE_COM_SPEED_MPS"] and s[j]['left_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] and s[j]['right_Fz']>th["BILATERAL_TAKEOFF_FZ_N"] for j in range(i,i+win_c)):
                capture=times[i]
                has_event("balance_capture", capture)
                break
        if capture is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        # 12 stable_recovery: standing neighborhood held for 0.50
        recovery=None
        win_r=int(th["STABLE_RECOVERY_DWELL_S"]/0.000125)
        for i in range(idx_land, len(s)-win_r):
            # check tilt<0.2618, qvel small, com margin>0.02, Fz near weight, etc. Simplified to tilt<0.2618 and |vz|<0.05 and Fz>0.5*W
            if all(abs(s[j]['trunk_tilt'])<th["STABLE_RECOVERY_TILT_MAX_RAD"] and abs(s[j]['com_vz'])<0.05 and s[j]['whole_Fz']>0.5*WEIGHT for j in range(i,i+win_r)):
                recovery=times[i]
                has_event("stable_recovery", recovery)
                break
        if recovery is None:
            return V2EventResult(self.events, self.valid, self._metrics(), {}, "INCOMPLETE_HORIZON")
        return V2EventResult(self.events, self.valid, self._metrics(), {}, "OBJECTIVE_COMPLETE")

    def _metrics(self):
        if not self.samples:
            return {}
        s=self.samples
        # compute metrics
        com_z=[x['com_z'] for x in s]
        times=[x['time_s'] for x in s]
        # countermovement depth = max - min after supported_start
        if "supported_start" in self.events:
            idx=np.searchsorted(times, self.events["supported_start"])
            com_ref=s[idx]['com_z']
            min_z=min(x['com_z'] for x in s[idx:])
            depth=com_ref - min_z
        else:
            depth=0
        # takeoff vz
        takeoff_vz=None
        if "bilateral_takeoff" in self.events:
            idx=np.searchsorted(times, self.events["bilateral_takeoff"])
            takeoff_vz=s[idx]['com_vz']
        # apex height
        apex_h=None
        if "apex" in self.events:
            # find max com_z near apex
            apex_h=max(com_z)
        return {
            "countermovement_depth": float(depth),
            "takeoff_vz": float(takeoff_vz) if takeoff_vz is not None else None,
            "apex_height": float(apex_h) if apex_h is not None else None,
            "max_Fz": float(max(x['whole_Fz'] for x in s)),
            "flight_duration": float(self.events.get("genuine_flight",0) - self.events.get("bilateral_takeoff",0)) if "genuine_flight" in self.events else None,
        }
