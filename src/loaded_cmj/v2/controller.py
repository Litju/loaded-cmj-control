import numpy as np
LIMITS=np.array([250,250,250,300,300,200,200], dtype=float)
HOLD_Q=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
WEIGHT=931.95
KP_HOLD=np.array([1000,1000,1000,1000,1000,1000,1000], dtype=float)
KD_HOLD=np.array([20,20,20,20,20,20,20], dtype=float)
FLEX_TAU=np.array([0,20,20,20,20,7,7], dtype=float)
EXTEND_TAU=np.array([0,-75,-75,-150,-150,-20,-20], dtype=float)
FLIGHT_TARGET=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
KP_FLIGHT=np.array([80,80,80,80,80,40,40], dtype=float)
KD_FLIGHT=np.array([12,12,12,12,12,6,6], dtype=float)
CAPTURE_TARGET=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
CAPTURE_FEEDFORWARD=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0], dtype=float)
I_EFF=np.array([7.75247,1.81608997,1.81608997,0.4397633,0.4397633,0.01720719,0.01720719], dtype=float)
KP_IMPACT=np.array([80,80,80,80,80,40,40], dtype=float)
KD_IMPACT=np.array([12,12,12,12,12,6,6], dtype=float)
KP_CAPTURE=np.array([80,80,80,80,80,40,40], dtype=float)
KD_CAPTURE=np.array([12,12,12,12,12,6,6], dtype=float)
# phases
_HOLD=0
_SUPPORTED=1
_FLIGHT=2
_IMPACT=3
_CAPTURED=4
_STAND=5
_phase=_HOLD
_phase_started=0
_prev_action=np.zeros(7)
# Optimized launch residual (RES10_R5A2, 16 params, 4 per group)
# Groups: lumbar, hip, knee, ankle; knots [0,0.2,0.4,0.6]
_OPT_PARAMS=np.array([0.0, -0.02, -0.01, -0.01, 0.0, 0.0, -0.03, -0.02, 0.0, 0.0, -0.05, -0.04, 0.0, 0.0, 0.0, 0.0], dtype=float)
_OPT_KNOTS=np.array([0.0,0.2,0.4,0.6], dtype=float)
def _delta_at(t_in, coeffs):
    if t_in <= _OPT_KNOTS[0]:
        return float(coeffs[0])
    if t_in >= _OPT_KNOTS[-1]:
        return 0.0 if t_in > _OPT_KNOTS[-1]+1e-9 else float(coeffs[-1])
    for i in range(len(coeffs)-1):
        if _OPT_KNOTS[i] <= t_in < _OPT_KNOTS[i+1]:
            frac=(t_in-_OPT_KNOTS[i])/(_OPT_KNOTS[i+1]-_OPT_KNOTS[i])
            return float(coeffs[i]*(1-frac)+coeffs[i+1]*frac)
    return float(coeffs[-1])

def reset(t):
    global _phase, _phase_started, _prev_action
    _phase=_HOLD
    _phase_started=t
    _prev_action=np.zeros(7)

def act(obs):
    global _phase, _phase_started, _prev_action
    try:
        t=float(obs["time_s"])
        s=np.array([float(x) for x in obs["joint_position_rad"]], dtype=float)
        sd=np.array([float(x) for x in obs["joint_velocity_radps"]], dtype=float)
        com=np.array(obs["com_position_m"], dtype=float) if "com_position_m" in obs else np.zeros(3)
        com_vel=np.array(obs["com_velocity_mps"], dtype=float) if "com_velocity_mps" in obs else np.zeros(3)
        left_fz=float(obs["plantar_normal_force_N"][0]) if "plantar_normal_force_N" in obs else 0
        right_fz=float(obs["plantar_normal_force_N"][1]) if "plantar_normal_force_N" in obs else 0
        total_fz=left_fz+right_fz
        maxf=max(left_fz, right_fz)
        weak=min(left_fz, right_fz)
        if _phase==_HOLD:
            if total_fz>0.30*WEIGHT and weak>10 and t-_phase_started>0.10:
                _phase=_SUPPORTED
                _phase_started=t
        elif _phase==_SUPPORTED:
            if maxf<10 and t-_phase_started>0.30:
                _phase=_FLIGHT
                _phase_started=t
        elif _phase==_FLIGHT:
            if maxf>=20:
                _phase=_IMPACT
                _phase_started=t
        elif _phase==_IMPACT:
            if abs(float(com_vel[2]))<0.05 and t-_phase_started>0.02:
                _phase=_CAPTURED
                _phase_started=t
        elif _phase==_CAPTURED:
            if weak>10 and abs(float(com_vel[2]))<0.30 and t-_phase_started>0.15:
                _phase=_STAND
                _phase_started=t
        if _phase==_HOLD:
            err=HOLD_Q-s
            raw=KP_HOLD*err + KD_HOLD*(-sd)
            u=np.clip(raw/LIMITS,-1,1)
        elif _phase==_SUPPORTED:
            t_in=t-_phase_started
            tau=FLEX_TAU if t_in<0.30 else (EXTEND_TAU if t_in<0.60 else np.zeros(7))
            u_seed=tau/LIMITS
            # residual delta
            d_lum=_delta_at(t_in, _OPT_PARAMS[0:4])
            d_hip=_delta_at(t_in, _OPT_PARAMS[4:8])
            d_knee=_delta_at(t_in, _OPT_PARAMS[8:12])
            d_ank=_delta_at(t_in, _OPT_PARAMS[12:16])
            delta_vec=np.array([d_lum, d_hip, d_hip, d_knee, d_knee, d_ank, d_ank], dtype=float)
            u=np.clip(u_seed+delta_vec,-1,1)
        elif _phase==_FLIGHT:
            err=FLIGHT_TARGET-s
            raw=KP_FLIGHT*err + KD_FLIGHT*(-sd)
            u=np.clip(raw/LIMITS,-1,1)
        elif _phase==_IMPACT:
            err=CAPTURE_TARGET-s
            raw=CAPTURE_FEEDFORWARD + KP_IMPACT*err - KD_IMPACT*sd
            u=np.clip(raw/LIMITS,-1,1)
        elif _phase==_CAPTURED:
            err=CAPTURE_TARGET-s
            raw=CAPTURE_FEEDFORWARD + KP_CAPTURE*err - KD_CAPTURE*sd
            u=np.clip(raw/LIMITS,-1,1)
        elif _phase==_STAND:
            err=np.zeros(7)-s
            raw=np.array([600,600,600,600,600,300,300],dtype=float)*err - np.array([30,30,30,30,30,15,15],dtype=float)*sd
            u=np.clip(raw/LIMITS,-1,1)
        else:
            u=np.zeros(7)
        _prev_action=u.copy()
        return u.tolist()
    except:
        return [0]*7
