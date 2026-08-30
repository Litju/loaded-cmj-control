"""V2 RES-8 Captured-Squat Landing Controller — mechanically coherent landing absorption.

HOLD: high PD to standing (0,0,0)
SUPPORTED_JUMP: time-based open-loop torque (flex then extend) — FROZEN from RES-7 authority
FLIGHT: PD to small flight posture
LANDING_PREP: moderate impedance toward captured-squat target during descending flight
IMPACT_TRANSITION: moderate stiffness + high damping impedance to dissipate impact
CAPTURED_SQUAT: static feedforward + moderate stiffness + overdamped hold — no recovery to standing

Design principles:
- Prepare hip/knee flexion BEFORE contact
- Do not command immediate standing after touchdown
- Allow controlled flexion during impact
- Impact uses moderate Kp + high Kd
- Capture uses feedforward + moderate Kp + near-critical damping
- Negative joint work visible during absorption
- No force integral, no bounce, no re-flight
- E11 ends RES-8; no E12 recovery
"""

import math
import numpy as np

LIMITS = np.array([250,250,250,300,300,200,200], dtype=float)
HOLD_Q = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
WEIGHT=931.95

# Frozen HOLD gains
KP_HOLD = np.array([1000,1000,1000,1000,1000,1000,1000], dtype=float)
KD_HOLD = np.array([20,20,20,20,20,20,20], dtype=float)

# Frozen SUPPORTED_JUMP open-loop torques — DO NOT MODIFY (pre-takeoff identity)
FLEX_TAU = np.array([0, 20, 20, 20, 20, 7, 7], dtype=float)
EXTEND_TAU = np.array([0, -72, -72, -92, -92, -46, -46], dtype=float)

# Flight target (small)
FLIGHT_TARGET = np.array([0.0, 0.05,0.05, 0.10,0.10, 0.05,0.05], dtype=float)
KP_FLIGHT = np.array([50,50,50,50,50,50,50], dtype=float)
KD_FLIGHT = np.array([10,10,10,10,10,10,10], dtype=float)

# ---------------------------------------------------------------------------
# CAPTURE TARGET — derived from deepest valid pre-reversal state
# Source: COUNTERMOVEMENT_INTERVAL [countermovement_onset, upward_reversal]
# T_DEEPEST_VALID_PRE_REVERSAL = 0.54175 s (upward_reversal.occurred_at)
# This is the physically demonstrated bilateral-supported non-fallen 95kg state.
# Extracted from sealed RES-7 trace via V2Plant forward extraction.
# ---------------------------------------------------------------------------
# Deepest valid joints: [lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle]
# Provenance exact from RES-7 trace at 0.54175 s
CAPTURE_TARGET = np.array([-0.64031276, 0.45635633, 0.45635633, 1.40332997, 1.40332997, 0.53130472, 0.53130472], dtype=float)
# Full Q at that time (including root): [0.07602242020686999, 0.6972841161071415, 0.3648535663167031, -0.64031276, 0.45635633, 1.40332997, 0.53130472, 0.45635633, 1.40332997, 0.53130472]
# QDOT at that time: lumbar 0.471, hips -0.133, knees 0.170, ankles -0.056 (full qvel includes root)
# COM_Z_DEEPEST_VALID = 0.8865105 m, LEFT_FZ 749.17 N, RIGHT_FZ 749.17 N, COP valid, trunk pitch 0.2755 rad

# Capture feedforward — static gravity + passive compensation from MuJoCo inverse at exact capture pose
# Computed via -(qfrc_bias + qfrc_passive) at exact capture qpos with zero vel, mapped to actuator order
# qfrc_bias at capture: [0, 931.95, 61.88, 50.30, 17.63, 6.51, -0.49, 17.63, 6.51, -0.49]
# qfrc_passive: [0,0,0, 9.60, -6.84, -21.05, -7.97, -6.84, -21.05, -7.97]
# needed = -(bias+passive) => lumbar -59.90, hips -10.78, knees +14.538, ankles +8.459
CAPTURE_FEEDFORWARD = np.array([-59.90579853, -10.78667909, -10.78667909, 14.53889135, 14.53889135, 8.45986569, 8.45986569], dtype=float)

# ---------------------------------------------------------------------------
# Inertia-derived gain construction
# At capture pose, generalized inertia diagonal (M(q_capture)) from MuJoCo mj_fullM:
# Ieff per actuated DoF (actuator order): [7.752, 1.816, 1.816, 0.439, 0.439, 0.0172, 0.0172]
# Kd = 2*zeta*sqrt(Kp*Ieff)
# ---------------------------------------------------------------------------
# Effective inertias at capture (from mj_fullM at exact pose)
I_EFF = np.array([7.75247, 1.81608997, 1.81608997, 0.4397633, 0.4397633, 0.01720719, 0.01720719], dtype=float)

# LANDING_PREP: moderate stiffness, moderate damping (zeta 0.8)
KP_PREP = np.array([180, 180, 180, 150, 150, 80, 80], dtype=float)
# Kd_prep = 2*0.8*sqrt(Kp*I)
KD_PREP = 2*0.80*np.sqrt(KP_PREP*I_EFF)

# IMPACT_TRANSITION: moderate stiffness + high damping (zeta 1.0 critical)
KP_IMPACT = np.array([320, 350, 350, 350, 350, 120, 120], dtype=float)
KD_IMPACT = 2*1.00*np.sqrt(KP_IMPACT*I_EFF)

# CAPTURED_SQUAT: moderate stiffness + overdamped (zeta 1.1)
KP_CAPTURE = np.array([320, 350, 350, 350, 350, 120, 120], dtype=float)
KD_CAPTURE = 2*1.10*np.sqrt(KP_CAPTURE*I_EFF)

# Also keep LANDING legacy for reference but not used
LANDING_TARGET = CAPTURE_TARGET  # alias for legacy
KP_LAND = KP_IMPACT
KD_LAND = KD_IMPACT

# Recovery (not used in RES-8, kept for interface but never entered)
KP_REC = KP_HOLD*0.8
KD_REC = KD_HOLD*1.2
RECOVERY_TARGET = HOLD_Q

# Phase IDs — extended to preserve old landing until takeoff for identity
_HOLD=0
_SUPPORTED_JUMP=1
_FLIGHT=2
_OLD_LANDING=3
_OLD_RECOVERY=4
_LANDING_PREP=5
_IMPACT=6
_CAPTURED=7

PHASE_NAMES=("HOLD","SUPPORTED_JUMP","FLIGHT","OLD_LANDING","OLD_RECOVERY","LANDING_PREP","IMPACT_TRANSITION","CAPTURED_SQUAT")

# Old landing constants for pre-takeoff identity (frozen RES-7)
OLD_LANDING_TARGET = np.array([0.05, 0.15,0.15, 0.30,0.30, 0.10,0.10], dtype=float)
OLD_KP_LAND = np.array([800,800,800,800,800,800,800], dtype=float)
OLD_KD_LAND = np.array([40,40,40,50,50,40,40], dtype=float)
OLD_KP_REC = KP_HOLD*0.8
OLD_KD_REC = KD_HOLD*1.2
OLD_RECOVERY_TARGET = HOLD_Q

# Takeoff time authority — switch to new landing only after true takeoff
TAKEOFF_SWITCH_S = 1.560125
CONTROLLER_ARCHITECTURE_ID="LCMJ-V2-5MODE-RES8"
CONTROLLER_REVISION="V2-R008-captured-squat"
ARCHITECTURE_GENERATION=2

# State
_phase=_HOLD
_phase_started=0.0
_last_time=None
_prev_action=HOLD_Q.copy()
_has_taken_off=False
_flight_started=None
_apex_passed=False
_prev_com_vz=0.0
_impact_start=None
_capture_dwell=0.0
_balance_dwell=0.0
_switched_to_new=False

def reset(time_s):
    global _phase,_phase_started,_last_time,_prev_action,_has_taken_off,_flight_started,_apex_passed,_prev_com_vz,_impact_start,_capture_dwell,_balance_dwell,_switched_to_new
    _phase=_HOLD
    _phase_started=time_s
    _last_time=time_s
    _prev_action=HOLD_Q.copy()
    _has_taken_off=False
    _flight_started=None
    _apex_passed=False
    _prev_com_vz=0.0
    _impact_start=None
    _capture_dwell=0.0
    _balance_dwell=0.0
    _switched_to_new=False

def act(obs):
    global _phase,_phase_started,_last_time,_prev_action,_has_taken_off,_flight_started,_apex_passed,_prev_com_vz,_impact_start,_capture_dwell,_balance_dwell,_switched_to_new
    try:
        time_s=float(obs["time_s"])
        s=np.array([float(x) for x in obs["joint_position_rad"]], dtype=float)
        sd=np.array([float(x) for x in obs["joint_velocity_radps"]], dtype=float)
        try:
            forces=np.array(obs["plantar_normal_force_N"], dtype=float)
            total_fz=float(forces[0]+forces[1])
            max_fz=float(max(forces[0],forces[1]))
            weak_fz=float(min(forces[0],forces[1]))
            cop_valid=obs["plantar_cop_valid"]
        except:
            total_fz=WEIGHT
            max_fz=WEIGHT
            weak_fz=WEIGHT
            cop_valid=[True,True]
        try:
            com_vz=float(obs["com_velocity_mps"][2])
        except:
            try:
                com_vz=float(obs["pelvis_linear_velocity_world_mps"][2])
            except:
                com_vz=0.0
        try:
            dt=float(obs["control_dt_s"])
        except:
            dt=0.005
        prev=np.array([float(x) for x in obs.get("previous_action", HOLD_Q)], dtype=float)
        _prev_action=prev.copy()
        if bool(obs.get("episode_reset", False)):
            reset(time_s)
        elif _last_time is not None and time_s +1e-12 < _last_time:
            reset(time_s)
        _last_time=time_s

        # Track apex: detect crossing from positive to negative after flight
        if _has_taken_off and _prev_com_vz>0 and com_vz<=0:
            _apex_passed=True
        _prev_com_vz=com_vz

        # Transitions — FROZEN supported jump logic preserved until TAKEOFF_SWITCH_S
        # For t < TAKEOFF_SWITCH_S, use old high-stiffness landing to preserve identity
        if time_s < TAKEOFF_SWITCH_S:
            # Old branching (pre-takeoff identity)
            if _phase==_HOLD:
                if total_fz>0.30*WEIGHT and weak_fz>10 and time_s - _phase_started > 0.10:
                    _phase=_SUPPORTED_JUMP
                    _phase_started=time_s
            elif _phase==_SUPPORTED_JUMP:
                if max_fz < 10.0 and time_s - _phase_started > 0.30:
                    _phase=_FLIGHT
                    _phase_started=time_s
                    _flight_started=time_s
                    _has_taken_off=True
                    _apex_passed=False
            elif _phase==_FLIGHT:
                if max_fz >= 20.0:
                    _phase=_OLD_LANDING
                    _phase_started=time_s
            elif _phase==_OLD_LANDING:
                bilateral= weak_fz>=10 and bool(cop_valid[0]) and bool(cop_valid[1])
                captured= bilateral and abs(com_vz)<=0.15 and 0.50*WEIGHT <= total_fz <=2.00*WEIGHT
                if (captured and time_s - _phase_started > 0.10) or (time_s - _phase_started > 0.40):
                    _phase=_OLD_RECOVERY
                    _phase_started=time_s
            elif _phase==_OLD_RECOVERY:
                if max_fz<10 or abs(com_vz)>0.08:
                    if time_s - _phase_started >0.05:
                        _phase=_OLD_LANDING
                        _phase_started=time_s
            # Note: after takeoff switch, old landing will be overridden
            # If we have just crossed switch time while in old phase, migrate to new
            if time_s >= TAKEOFF_SWITCH_S and _phase in (_OLD_LANDING, _OLD_RECOVERY):
                # If already in flight (Fz 0), transition to FLIGHT for new handling
                if max_fz < 10:
                    _phase=_FLIGHT
                    _phase_started=time_s
                    _has_taken_off=True
                else:
                    # else stay as is until flight confirmed; will be handled in new branch next tick
                    pass
        else:
            # New landing logic after true takeoff
            if not _switched_to_new:
                # First tick after switch: ensure we are in FLIGHT if airborne, else remain
                if max_fz < 10 and _has_taken_off:
                    _phase=_FLIGHT
                    _phase_started=time_s
                _switched_to_new=True
            if _phase==_HOLD:
                if total_fz>0.30*WEIGHT and weak_fz>10 and time_s - _phase_started > 0.10:
                    _phase=_SUPPORTED_JUMP
                    _phase_started=time_s
            elif _phase==_SUPPORTED_JUMP:
                if max_fz < 10.0 and time_s - _phase_started > 0.30:
                    _phase=_FLIGHT
                    _phase_started=time_s
                    _flight_started=time_s
                    _has_taken_off=True
                    _apex_passed=False
            elif _phase==_FLIGHT:
                # Check for landing preparation: passed apex, descending, both feet unloaded
                if _has_taken_off and com_vz < -0.10 and total_fz < 20.0 and max_fz < 10.0:
                    if _apex_passed or (com_vz < -0.15):
                        _phase=_LANDING_PREP
                        _phase_started=time_s
                # Also immediate impact if contact appears before prep (late detection)
                if max_fz >= 20.0 and com_vz < 0 and _has_taken_off:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
            elif _phase==_OLD_LANDING:
                # Migrate old landing to new handling after switch
                bilateral= weak_fz>=10 and bool(cop_valid[0]) and bool(cop_valid[1])
                captured= bilateral and abs(com_vz)<=0.15 and 0.50*WEIGHT <= total_fz <=2.00*WEIGHT
                # Instead of going to old recovery, go to new prep/impact if flight was genuine
                if max_fz < 10 and com_vz < -0.10:
                    _phase=_FLIGHT
                    _phase_started=time_s
                elif max_fz >= 20 and _has_taken_off:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
                elif (captured and time_s - _phase_started > 0.10) or (time_s - _phase_started > 0.40):
                    # Instead of old recovery, go to capture if already post-landing
                    _phase=_LANDING_PREP
                    _phase_started=time_s
            elif _phase==_OLD_RECOVERY:
                # Migrate to new
                if max_fz<10:
                    _phase=_FLIGHT
                    _phase_started=time_s
                else:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
            elif _phase==_LANDING_PREP:
                if max_fz >= 20.0 and com_vz < 0 and _has_taken_off:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
            elif _phase==_IMPACT:
                if abs(com_vz) < 0.05:
                    _capture_dwell += dt
                else:
                    _capture_dwell = 0.0
                if _capture_dwell >= 0.020:
                    _phase=_CAPTURED
                    _phase_started=time_s
                    _balance_dwell=0.0
            elif _phase==_CAPTURED:
                bilateral = weak_fz >= 10 and bool(cop_valid[0]) and bool(cop_valid[1])
                if bilateral and abs(com_vz) < 0.30:
                    _balance_dwell += dt
                else:
                    _balance_dwell = 0.0

        # Action computation — transparent torque-space impedance
        if _phase==_HOLD:
            err=HOLD_Q - s
            raw=KP_HOLD*err + KD_HOLD*(-sd)
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_SUPPORTED_JUMP:
            t_in_phase=time_s - _phase_started
            if t_in_phase < 0.30:
                tau=FLEX_TAU
            elif t_in_phase < 0.60:
                tau=EXTEND_TAU
            else:
                tau=np.zeros(7)
            u=np.clip(tau/LIMITS, -1,1)
        elif _phase==_FLIGHT:
            err=FLIGHT_TARGET - s
            raw=KP_FLIGHT*err + KD_FLIGHT*(-sd)
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_OLD_LANDING:
            err=OLD_LANDING_TARGET - s
            raw=OLD_KP_LAND*err + OLD_KD_LAND*(-sd)
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_OLD_RECOVERY:
            blend=min(1.0, max(0.0, (time_s - _phase_started)/0.30))
            kp=(1-blend)*OLD_KP_LAND + blend*OLD_KP_REC
            kd=(1-blend)*OLD_KD_LAND + blend*OLD_KD_REC
            target=(1-blend)*OLD_LANDING_TARGET + blend*OLD_RECOVERY_TARGET
            err=target - s
            raw=kp*err + kd*(-sd)
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_LANDING_PREP:
            err=CAPTURE_TARGET - s
            raw=CAPTURE_FEEDFORWARD*0.0 + KP_PREP*err + KD_PREP*(-sd)
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_IMPACT:
            err=CAPTURE_TARGET - s
            raw=CAPTURE_FEEDFORWARD + KP_IMPACT*err - KD_IMPACT*sd
            u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_CAPTURED:
            err=CAPTURE_TARGET - s
            raw=CAPTURE_FEEDFORWARD + KP_CAPTURE*err - KD_CAPTURE*sd
            u=np.clip(raw/LIMITS, -1,1)
        else:
            u=np.zeros(7)
        u=np.clip(u, -1,1)
        if not np.isfinite(u).all():
            return HOLD_Q.tolist()
        return [float(x) for x in u]
    except Exception:
        try:
            return [float(x) for x in obs.get("previous_action", HOLD_Q)]
        except:
            return HOLD_Q.tolist()

def debug_state():
    return {
        "controller_architecture_id": CONTROLLER_ARCHITECTURE_ID,
        "controller_revision": CONTROLLER_REVISION,
        "phase": PHASE_NAMES[_phase],
        "phase_started": float(_phase_started),
        "has_taken_off": bool(_has_taken_off),
        "apex_passed": bool(_apex_passed),
        "capture_dwell": float(_capture_dwell),
        "balance_dwell": float(_balance_dwell),
    }
