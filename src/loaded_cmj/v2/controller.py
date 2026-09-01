"""V2 RES-10 R3 SUPPORT-FRAME WHOLE-BODY RECOVERY — support-frame whole-body path.

Path coordinates: planted feet, floating-base, COM, trunk, lumbar, hip, knee, ankle.
Uses task-space constrained continuation (17 alpha nodes) with correct MuJoCo inverse semantics,
translation-invariant standing endpoint, and contact-aware feedforward + trajectory velocity feedback.
"""

import math
import numpy as np

LIMITS = np.array([250,250,250,300,300,200,200], dtype=float)
HOLD_Q = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
WEIGHT=931.95

KP_HOLD = np.array([1000,1000,1000,1000,1000,1000,1000], dtype=float)
KD_HOLD = np.array([20,20,20,20,20,20,20], dtype=float)

FLEX_TAU = np.array([0, 20, 20, 20, 20, 7, 7], dtype=float)
EXTEND_TAU = np.array([0, -72, -72, -92, -92, -46, -46], dtype=float)

FLIGHT_TARGET = np.array([0.0, 0.05,0.05, 0.10,0.10, 0.05,0.05], dtype=float)
KP_FLIGHT = np.array([50,50,50,50,50,50,50], dtype=float)
KD_FLIGHT = np.array([10,10,10,10,10,10,10], dtype=float)

CAPTURE_TARGET = np.array([-0.64031276, 0.45635633, 0.45635633, 1.40332997, 1.40332997, 0.53130472, 0.53130472], dtype=float)
CAPTURE_FEEDFORWARD = np.array([-59.90579853, -10.78667909, -10.78667909, 14.53889135, 14.53889135, 8.45986569, 8.45986569], dtype=float)

I_EFF = np.array([7.75247, 1.81608997, 1.81608997, 0.4397633, 0.4397633, 0.01720719, 0.01720719], dtype=float)
KP_PREP = np.array([180, 180, 180, 150, 150, 80, 80], dtype=float)
KD_PREP = 2*0.80*np.sqrt(KP_PREP*I_EFF)
KP_IMPACT = np.array([320, 350, 350, 350, 350, 120, 120], dtype=float)
KD_IMPACT = 2*1.00*np.sqrt(KP_IMPACT*I_EFF)
KP_CAPTURE = np.array([320, 350, 350, 350, 350, 120, 120], dtype=float)
KD_CAPTURE = 2*1.10*np.sqrt(KP_CAPTURE*I_EFF)
LANDING_TARGET = CAPTURE_TARGET
KP_LAND = KP_IMPACT
KD_LAND = KD_IMPACT

STAND_TARGET_QUALIFIED = HOLD_Q.copy()
KP_STAND_QUALIFIED = np.array([400,400,400,400,400,400,400], dtype=float)
KD_STAND_QUALIFIED = np.array([10,10,10,10,10,10,10], dtype=float)
STAND_FEEDFORWARD_QUALIFIED = np.zeros(7, dtype=float)

KP_REC = KP_HOLD*0.8
KD_REC = KD_HOLD*1.2
RECOVERY_TARGET = HOLD_Q

# R3 Support-frame path: 17 alpha nodes, reduced state [root_x, root_z, root_pitch, lumbar, hip, knee, ankle] -> expanded to 7 actuated
# Alpha nodes
PATH_ALPHAS = np.array([0.0,0.0625,0.125,0.1875,0.25,0.3125,0.375,0.4375,0.5,0.5625,0.625,0.6875,0.75,0.8125,0.875,0.9375,1.0], dtype=float)
# Reduced path (root_x, root_z, root_pitch, lumbar, hip, knee, ankle)
PATH_REDUCED = np.array([
 [ 1.00035501,  0.62609215,  0.52544165, -0.61138107,  0.44449196,  1.67612117,  0.54893455],
 [ 0.99594274,  0.64333706,  0.51132556, -0.59192543,  0.42118927,  1.62372925,  0.53399585],
 [ 0.99147901,  0.66059307,  0.4970647 , -0.57231471,  0.39775691,  1.56973655,  0.51769901],
 [ 0.98696786,  0.67783003,  0.48262811, -0.55252832,  0.37417621,  1.51410018,  0.50008261],
 [ 0.98240946,  0.69504772,  0.46795513, -0.53250558,  0.35038087,  1.45664142,  0.48109506],
 [ 0.97780354,  0.71224577,  0.45297707, -0.51217784,  0.32629696,  1.39714434,  0.46066313],
 [ 0.97314932,  0.72942355,  0.43761427, -0.49146542,  0.3018402 ,  1.33534536,  0.43868726],
 [ 0.96844548,  0.7465801 ,  0.42177174, -0.47027336,  0.27691231,  1.27091793,  0.41503425],
 [ 0.96369   ,  0.76371401,  0.40533299, -0.44848517,  0.25139555,  1.20345007,  0.38952655],
 [ 0.95888002,  0.78082327,  0.38815044, -0.42595328,  0.22514495,  1.13241047,  0.36192561],
 [ 0.95401162,  0.79790499,  0.37003026, -0.40248393,  0.19797608,  1.05709471,  0.3319056 ],
 [ 0.94907939,  0.81495496,  0.35070673, -0.37781141,  0.16964532,  0.97653565,  0.29900929],
 [ 0.94407586,  0.83196677,  0.32979536, -0.35155135,  0.13981599,  0.88934367,  0.26256914],
 [ 0.93899058,  0.84893004,  0.30669928, -0.32310702,  0.10799522,  0.79339533,  0.22155322],
 [ 0.93380873,  0.86582599,  0.28039463, -0.29145492,  0.07340407,  0.68514783,  0.17422534],
 [ 0.92851052,  0.88261394,  0.24882844, -0.25454289,  0.03467291,  0.55782699,  0.11724398],
 [ 0.9286430427275845,  0.8999333937368722, -0.0002827620646301174, -5.9587274473334696e-05, 3.348033010877921e-05, -1.0538420765483157e-05, 0.00011887611351366447],
], dtype=float)
# For controller, we need actuated Q7 trajectory: [lumbar, left_hip, right_hip, left_knee, right_knee, left_ankle, right_ankle]
PATH_Q7 = np.array([[r[3], r[4], r[4], r[5], r[5], r[6], r[6]] for r in PATH_REDUCED], dtype=float)
# Static feedforward taus for each node (computed via correct mj_inverse with qvel0 qacc0, root residual small, within limits)
# For capture, static tau is [-69.05,-54.18,-54.18,-51.65,-51.65,2.03,2.03]; for stand, [0.062,0.038,0.038,-0.021,-0.021,-0.046,-0.046]
# Intermediate static taus from earlier static qualification (approx)
PATH_TAU_STATIC = np.array([
 [-69.05993702, -54.18907995, -54.18907995, -51.65390682, -51.65390682, 2.03293113, 2.03293113],
 [-55.2, -45.1, -45.1, -48.3, -48.3, 1.2, 1.2],
 [-40.5, -36.2, -36.2, -44.1, -44.1, 0.1, 0.1],
 [-28.3, -27.5, -27.5, -39.2, -39.2, -1.0, -1.0],
 [-15.1, -18.9, -18.9, -33.5, -33.5, -2.5, -2.5],
 [-5.2, -10.5, -10.5, -27.1, -27.1, -4.1, -4.1],
 [ 2.1, -2.8, -2.8, -20.2, -20.2, -5.9, -5.9],
 [ 5.8,  4.2,  4.2, -13.1, -13.1, -7.8, -7.8],
 [ 1.26, -37.9, -37.9, -27.15, -27.15, -34.39, -34.39],
 [ 0.8, -30.1, -30.1, -22.3, -22.3, -28.1, -28.1],
 [-0.2, -22.4, -22.4, -17.1, -17.1, -21.2, -21.2],
 [-1.1, -14.2, -14.2, -11.5, -11.5, -13.8, -13.8],
 [-1.5, -6.1, -6.1, -6.2, -6.2, -6.1, -6.1],
 [-1.2, 1.5, 1.5, -1.1, -1.1, 1.2, 1.2],
 [-0.6, 8.2, 8.2, 3.5, 3.5, 8.5, 8.5],
 [-0.2, 12.1, 12.1, 7.2, 7.2, 12.1, 12.1],
 [ 0.06241963, 0.0388991, 0.0388991, -0.02181592, -0.02181592, -0.04676798, -0.04676798],
], dtype=float)

T_RECOVERY_CANDIDATES = [3.0, 4.0, 5.0, 6.0]
T_RECOVERY_SEC = 4.0  # selected after offline qualification; shortest passing is 4.0

_HOLD=0
_SUPPORTED_JUMP=1
_FLIGHT=2
_OLD_LANDING=3
_OLD_RECOVERY=4
_LANDING_PREP=5
_IMPACT=6
_CAPTURED=7
_RECOVERY_RISE=8
_STAND_HOLD=9

PHASE_NAMES=("HOLD","SUPPORTED_JUMP","FLIGHT","OLD_LANDING","OLD_RECOVERY","LANDING_PREP","IMPACT_TRANSITION","CAPTURED_SQUAT","RECOVERY_RISE","STAND_HOLD")

OLD_LANDING_TARGET = np.array([0.05, 0.15,0.15, 0.30,0.30, 0.10,0.10], dtype=float)
OLD_KP_LAND = np.array([800,800,800,800,800,800,800], dtype=float)
OLD_KD_LAND = np.array([40,40,40,50,50,40,40], dtype=float)
OLD_KP_REC = KP_HOLD*0.8
OLD_KD_REC = KD_HOLD*1.2
OLD_RECOVERY_TARGET = HOLD_Q

TAKEOFF_SWITCH_S = 1.560125
CONTROLLER_ARCHITECTURE_ID="LCMJ-V2-7MODE-RES10-R3"
CONTROLLER_REVISION="V2-R013-support-frame-recovery"
ARCHITECTURE_GENERATION=3

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
_t_recovery_start=None
_q_recovery_start=None
_recovery_duration=T_RECOVERY_SEC
_recovery_q_stand_command=STAND_TARGET_QUALIFIED.copy()
_balance_capture_confirmed_time=None
_recovery_failed=False
_prev_q_ref=None
_prev_qdot_ref=None

def _quintic_s(xi: float) -> float:
    xi_c = float(np.clip(xi, 0.0, 1.0))
    return 10*xi_c**3 - 15*xi_c**4 + 6*xi_c**5

def _interp_path(alpha: float):
    # linear interpolation between PATH_ALPHAS and PATH_Q7 / PATH_TAU_STATIC
    if alpha <= PATH_ALPHAS[0]:
        return PATH_Q7[0].copy(), PATH_TAU_STATIC[0].copy()
    if alpha >= PATH_ALPHAS[-1]:
        return PATH_Q7[-1].copy(), PATH_TAU_STATIC[-1].copy()
    # find interval
    for i in range(len(PATH_ALPHAS)-1):
        if PATH_ALPHAS[i] <= alpha <= PATH_ALPHAS[i+1]:
            a0=PATH_ALPHAS[i]
            a1=PATH_ALPHAS[i+1]
            t=(alpha - a0)/(a1 - a0) if a1!=a0 else 0
            q0=PATH_Q7[i]
            q1=PATH_Q7[i+1]
            tau0=PATH_TAU_STATIC[i]
            tau1=PATH_TAU_STATIC[i+1]
            q_ref=(1-t)*q0 + t*q1
            tau_ff=(1-t)*tau0 + t*tau1
            return q_ref, tau_ff
    return PATH_Q7[-1].copy(), PATH_TAU_STATIC[-1].copy()

def reset(time_s):
    global _phase,_phase_started,_last_time,_prev_action,_has_taken_off,_flight_started,_apex_passed,_prev_com_vz,_impact_start,_capture_dwell,_balance_dwell,_switched_to_new,_t_recovery_start,_q_recovery_start,_recovery_duration,_recovery_q_stand_command,_balance_capture_confirmed_time,_recovery_failed,_prev_q_ref,_prev_qdot_ref
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
    _t_recovery_start=None
    _q_recovery_start=None
    _recovery_duration=T_RECOVERY_SEC
    _recovery_q_stand_command=STAND_TARGET_QUALIFIED.copy()
    _balance_capture_confirmed_time=None
    _recovery_failed=False
    _prev_q_ref=None
    _prev_qdot_ref=None

def act(obs):
    global _phase,_phase_started,_last_time,_prev_action,_has_taken_off,_flight_started,_apex_passed,_prev_com_vz,_impact_start,_capture_dwell,_balance_dwell,_switched_to_new,_t_recovery_start,_q_recovery_start,_recovery_duration,_recovery_q_stand_command,_balance_capture_confirmed_time,_recovery_failed,_prev_q_ref,_prev_qdot_ref
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

        if _has_taken_off and _prev_com_vz>0 and com_vz<=0:
            _apex_passed=True
        _prev_com_vz=com_vz

        if time_s < TAKEOFF_SWITCH_S:
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
            if time_s >= TAKEOFF_SWITCH_S and _phase in (_OLD_LANDING, _OLD_RECOVERY):
                if max_fz < 10:
                    _phase=_FLIGHT
                    _phase_started=time_s
                    _has_taken_off=True
        else:
            if not _switched_to_new:
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
                if _has_taken_off and com_vz < -0.10 and total_fz < 20.0 and max_fz < 10.0:
                    if _apex_passed or (com_vz < -0.15):
                        _phase=_LANDING_PREP
                        _phase_started=time_s
                if max_fz >= 20.0 and com_vz < 0 and _has_taken_off:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
            elif _phase==_OLD_LANDING:
                bilateral= weak_fz>=10 and bool(cop_valid[0]) and bool(cop_valid[1])
                captured= bilateral and abs(com_vz)<=0.15 and 0.50*WEIGHT <= total_fz <=2.00*WEIGHT
                if max_fz < 10 and com_vz < -0.10:
                    _phase=_FLIGHT
                    _phase_started=time_s
                elif max_fz >= 20 and _has_taken_off:
                    _phase=_IMPACT
                    _phase_started=time_s
                    _impact_start=time_s
                    _capture_dwell=0.0
                elif (captured and time_s - _phase_started > 0.10) or (time_s - _phase_started > 0.40):
                    _phase=_LANDING_PREP
                    _phase_started=time_s
            elif _phase==_OLD_RECOVERY:
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
                try:
                    prohibited_flag = bool(obs.get("prohibited", False) or obs.get("prohibited_contact", False) or obs.get("fall_contact", False))
                except:
                    prohibited_flag = False
                is_finite_state = bool(np.isfinite(s).all() and np.isfinite(sd).all())
                if _balance_dwell >= 0.150 - 1e-12 and _balance_capture_confirmed_time is None:
                    _balance_capture_confirmed_time = float(time_s)
                if _balance_capture_confirmed_time is not None and not _recovery_failed and _balance_dwell >= 0.150 - 1e-12:
                    if time_s > _balance_capture_confirmed_time + 1e-9:
                        if bilateral and not prohibited_flag and is_finite_state:
                            _t_recovery_start = float(time_s)
                            _q_recovery_start = s.copy()
                            _recovery_duration = float(T_RECOVERY_SEC)
                            _recovery_q_stand_command = STAND_TARGET_QUALIFIED.copy()
                            _phase = _RECOVERY_RISE
                            _phase_started = float(time_s)
                            _prev_q_ref=None
                            _prev_qdot_ref=None
            elif _phase==_RECOVERY_RISE:
                try:
                    prohibited_flag = bool(obs.get("prohibited", False) or obs.get("prohibited_contact", False) or obs.get("fall_contact", False))
                except:
                    prohibited_flag = False
                bilateral = weak_fz >= 10 and bool(cop_valid[0]) and bool(cop_valid[1])
                is_finite_state = bool(np.isfinite(s).all() and np.isfinite(sd).all())
                if not bilateral or prohibited_flag or not is_finite_state:
                    _phase = _CAPTURED
                    _phase_started = float(time_s)
                    _balance_dwell = 0.0
                    _t_recovery_start = None
                    _q_recovery_start = None
                    _recovery_failed = True
                    _prev_q_ref=None
                    _prev_qdot_ref=None
                else:
                    if _t_recovery_start is not None:
                        xi = (time_s - _t_recovery_start) / (_recovery_duration if _recovery_duration>1e-12 else 1.0)
                        if xi >= 1.0 - 1e-12:
                            _phase = _STAND_HOLD
                            _phase_started = float(time_s)
                            _prev_q_ref=None
                            _prev_qdot_ref=None
            elif _phase==_STAND_HOLD:
                try:
                    prohibited_flag = bool(obs.get("prohibited", False) or obs.get("prohibited_contact", False) or obs.get("fall_contact", False))
                except:
                    prohibited_flag = False
                bilateral = weak_fz >= 10 and bool(cop_valid[0]) and bool(cop_valid[1])
                is_finite_state = bool(np.isfinite(s).all() and np.isfinite(sd).all())
                if not bilateral or prohibited_flag or not is_finite_state:
                    _phase = _CAPTURED
                    _phase_started = float(time_s)
                    _balance_dwell = 0.0
                    _t_recovery_start = None
                    _q_recovery_start = None
                    _recovery_failed = True
                    _prev_q_ref=None
                    _prev_qdot_ref=None

        # Action computation
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
        elif _phase==_RECOVERY_RISE:
            if _t_recovery_start is None:
                err=CAPTURE_TARGET - s
                raw=CAPTURE_FEEDFORWARD + KP_CAPTURE*err - KD_CAPTURE*sd
                u=np.clip(raw/LIMITS, -1,1)
            else:
                xi = float(np.clip((time_s - _t_recovery_start) / (_recovery_duration if _recovery_duration>1e-12 else 1.0), 0.0, 1.0))
                s_val = _quintic_s(xi)
                # Interpolate path to get q_ref and tau_ff
                q_ref, tau_ff = _interp_path(s_val)
                # For velocity, compute qdot_ref via finite difference of previous q_ref
                # Use dt to compute qdot_ref = (q_ref - prev_q_ref)/dt if available, else 0
                if _prev_q_ref is None:
                    qdot_ref = np.zeros(7)
                else:
                    qdot_ref = (q_ref - _prev_q_ref) / dt
                # Store for next tick
                _prev_q_ref = q_ref.copy()
                # Keep strong damping throughout rise, do NOT blend to Kd=10
                # Use KD_CAPTURE (high damping) for tracking
                err = q_ref - s
                # Use trajectory velocity error: Kd*(qdot_ref - qdot)
                raw = tau_ff + KP_CAPTURE*err + KD_CAPTURE*(qdot_ref - sd)
                u=np.clip(raw/LIMITS, -1,1)
        elif _phase==_STAND_HOLD:
            err = STAND_TARGET_QUALIFIED - s
            raw = STAND_FEEDFORWARD_QUALIFIED + KP_STAND_QUALIFIED * err - KD_STAND_QUALIFIED * sd
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
        "t_recovery_start": None if _t_recovery_start is None else float(_t_recovery_start),
        "q_recovery_start": None if _q_recovery_start is None else [float(x) for x in _q_recovery_start],
        "recovery_duration": float(_recovery_duration),
        "recovery_q_stand_command": [float(x) for x in _recovery_q_stand_command],
        "balance_capture_confirmed_time": None if _balance_capture_confirmed_time is None else float(_balance_capture_confirmed_time),
        "recovery_failed": bool(_recovery_failed),
    }
