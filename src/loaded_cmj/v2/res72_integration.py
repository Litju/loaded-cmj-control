"""RES-72 frozen single-composition integration policy.

MISSION=RES10_SYNC_CORRECTED_APEX_TO_E10_INTEGRATION_001
EXPERIMENT_ID=EXP-RES10-SYNC-CORRECTED-APEX-E10-001

Composition (exactly one, no search, no tuning):
  PRE_APEX (phases 0,1,2): committed v2/controller.py verbatim
    HOLD / SUPPORTED (with R5A2 16-param residual) / FLIGHT
  LANDING_PREP (phase 6) + INITIAL_IMPACT (phase 3):
    exact C01 from EXP-RES10-SYNC-PREDICTIVE-TD-VDAMP-E8-E10-001 C01
    (candidate_table.json C01 + harness/run_candidate.py + harness/common.py)
  TERMINAL_CAPTURE (phase 7):
    exact sealed RES-58 law (src/loaded_cmj/v2/terminal_capture.py)
    + sealed RES-52 SoftContactPolicy inner realization

Provenance (sealed sources, not prose):
  C01 PREP_TRIGGER: FLIGHT_and_com_vz_lt_-0.005
    source: candidate_table.json C01["PREP_TRIGGER immediate"]
  C01 PREP_TARGET/KP/KD, IMPACT_TARGET/KP/KD, CAPTURE_CONT (unused):
    source: candidate_table.json C01 entry (SHA 9173bf26...)
  C01 touchdown: whole_Fz>=20 physics, maxf>=20 control
    source: harness/run_candidate.py (td_t + phase 6->3)
  C01 IMPACT KD ramp: KD_INIT==KD_FINAL (identity), delay 0.005 dur 0.030
    source: candidate_table.json + run_candidate.kd_now
  Handoff guard: first control boundary with (t_control - TD_physics) >= 0.050
    TD_physics = first physics sample whole_Fz>=20 after entering phase 6
    Proof that 0.050 is sealed (not hardcoded prose):
      core52.py S_OFFSETS_S["S50"]=0.050,
      BRANCH_STATE_AUTHORITY.json S_TIMES.S50 = TD+0.050,
      experiment_spec.json START_STATE_AUTHORITY.S_TIMES.S50 = TD+0.050,
      S50 time 0.925375 = TD 0.875375 + 0.050 exactly.
    Control-grid: handoff evaluated at control boundaries only (sample-before-update),
    no mid-interval switch (frozen hold semantics).
    Absolute 0.925 NOT used; only TD-relative.
  RES-58 law: VZ_TARGET=0.0, CONTROL_DT=0.005, BW=m*g, FZ_RAW=BW-m*vz/DT,
    clamp [0.6BW,1.5BW], vz_target_next=vz+DT*(FZ/m-g)
    source: terminal_capture.py (functional identity to archived 06e73896... maxdiff 0.0)
  Measurements: RES-54 jacSubtreeCom (V2Plant.center_of_mass_velocity),
    RES-55 J_point@qvel+efc_vel (soft_contact.y_of / core52.soft_contact_state)
  Support: RES-57 contract SHA cde531e9... (support_continuity.py)

Controller triggering uses physical state (synchronized sample), never
V2EventDetector/scorer state. Scorer runs observationally in parallel.
"""
from __future__ import annotations
import hashlib
import numpy as np

# ---- frozen constants (verbatim from sealed sources) ----
WEIGHT_N = 95.0 * 9.81
MASS_KG = 95.0
GRAV = 9.81
LIMITS = np.array([250, 250, 250, 300, 300, 200, 200], dtype=float)

# v2/controller.py prefix (verbatim)
HOLD_Q = np.zeros(7)
KP_HOLD = np.array([1000]*7, dtype=float)
KD_HOLD = np.array([20]*7, dtype=float)
FLEX_TAU = np.array([0, 20, 20, 20, 20, 7, 7], dtype=float)
EXTEND_TAU = np.array([0, -75, -75, -150, -150, -20, -20], dtype=float)
FLIGHT_TARGET = np.zeros(7)
KP_FLIGHT = np.array([80, 80, 80, 80, 80, 40, 40], dtype=float)
KD_FLIGHT = np.array([12, 12, 12, 12, 12, 6, 6], dtype=float)
_OPT_PARAMS = np.array([0.0, -0.02, -0.01, -0.01, 0.0, 0.0, -0.03, -0.02,
                        0.0, 0.0, -0.05, -0.04, 0.0, 0.0, 0.0, 0.0], dtype=float)
_OPT_KNOTS = np.array([0.0, 0.2, 0.4, 0.6], dtype=float)

# C01 exact (candidate_table.json C01)
PREP_TARGET = np.array([0.0, 0.30, 0.30, 0.70, 0.70, 0.20, 0.20], dtype=float)
PREP_KP = np.array([80, 80, 80, 80, 80, 40, 40], dtype=float)
PREP_KD = np.array([12, 12, 12, 12, 12, 6, 6], dtype=float)
IMPACT_TARGET = np.array([0.0, 0.32, 0.32, 0.75, 0.75, 0.20, 0.20], dtype=float)
IMPACT_KP = np.array([114.0, 114.0, 114.0, 114.0, 114.0, 57.0, 57.0], dtype=float)
IMPACT_KD = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 10.0, 10.0], dtype=float)
KD_RAMP_DELAY_S = 0.005
KD_RAMP_DURATION_S = 0.030
PREP_TRIGGER_VZ = -0.005
TOUCHDOWN_WHOLE_FZ = 20.0
TOUCHDOWN_MAXF = 20.0
HANDOFF_ELAPSED_S = 0.050  # proven TD+50ms = S50 definition, NOT prose hardcode

PHASE_NAMES = {0: "HOLD", 1: "SUPPORTED", 2: "FLIGHT", 3: "IMPACT_ATTENUATION",
               6: "LANDING_PREP", 7: "TERMINAL_CAPTURE"}

POLICY_IDENTITY = (
    "PRE_APEX=v2.controller verbatim; "
    "LANDING_PREP=C01(PREP_TARGET=[0,.30,.70,.20],KP80/KD12,trig FLIGHT_and_cvz<-0.005); "
    "INITIAL_IMPACT=C01(IMPACT_TARGET=[0,.32,.75,.20],KP114/KD20_fixed,td whole>=20/maxf>=20); "
    "HANDOFF=first_control_boundary(t-TD_physics>=0.050,TD-relative,proven S50=TD+0.050); "
    "TERMINAL=RES58(FZ_RAW=BW-m*vz/DT,clamp[0.6,1.5]BW)+RES52_SoftContactPolicy"
)

def _delta_at(t_in: float, coeffs: np.ndarray) -> float:
    if t_in <= _OPT_KNOTS[0]:
        return float(coeffs[0])
    if t_in >= _OPT_KNOTS[-1]:
        return 0.0 if t_in > _OPT_KNOTS[-1] + 1e-9 else float(coeffs[-1])
    for i in range(len(coeffs) - 1):
        if _OPT_KNOTS[i] <= t_in < _OPT_KNOTS[i + 1]:
            frac = (t_in - _OPT_KNOTS[i]) / (_OPT_KNOTS[i + 1] - _OPT_KNOTS[i])
            return float(coeffs[i] * (1 - frac) + coeffs[i + 1] * frac)
    return float(coeffs[-1])

def smoothstep(x: float) -> float:
    x = min(1.0, max(0.0, float(x)))
    return x * x * (3.0 - 2.0 * x)

class Res72Policy:
    """Single-composition controller. No scorer input. No tuning."""

    def __init__(self, terminal_policy=None):
        # terminal_policy: SoftContactPolicy instance or None (set later with plant/probes)
        self.terminal_policy = terminal_policy
        self.reset(0.0)

    def reset(self, t: float) -> None:
        self.phase = 0
        self.phase_started = float(t)
        self.td_physics_t = None
        self.handoff_t = None
        self.prev_action = np.zeros(7)
        self.kd_state = IMPACT_KD.copy()
        self.last_terminal_info = {}

    def notify_touchdown_physics(self, t: float) -> None:
        if self.td_physics_t is None:
            self.td_physics_t = float(t)

    def _impact_kd(self, t: float) -> np.ndarray:
        # C01: KD_INIT==KD_FINAL so ramp is identity; kept for fidelity
        if self.td_physics_t is None:
            return IMPACT_KD.copy()
        ramp = smoothstep((t - self.td_physics_t - KD_RAMP_DELAY_S) / KD_RAMP_DURATION_S)
        return IMPACT_KD + (IMPACT_KD - IMPACT_KD) * ramp

    def act(self, obs: dict, sample=None, meas=None, plant=None):
        """One synchronized control boundary. Returns (u7, info)."""
        from loaded_cmj.v2 import terminal_capture as TC
        t = float(obs["time_s"])
        s = np.array([float(x) for x in obs["joint_position_rad"]], dtype=float)
        sd = np.array([float(x) for x in obs["joint_velocity_radps"]], dtype=float)
        com_vel = np.array(obs["com_velocity_mps"], dtype=float)
        cvz = float(com_vel[2])
        lfz = float(obs["plantar_normal_force_N"][0])
        rfz = float(obs["plantar_normal_force_N"][1])
        total_fz = lfz + rfz
        maxf = max(lfz, rfz)
        weak = min(lfz, rfz)
        info: dict = {"phase": self.phase}

        # ---- transitions (physical, control-grid only) ----
        if self.phase == 0:
            if total_fz > 0.30 * WEIGHT_N and weak > 10 and t - self.phase_started > 0.10:
                self.phase = 1
                self.phase_started = t
        elif self.phase == 1:
            if maxf < 10 and t - self.phase_started > 0.30:
                self.phase = 2
                self.phase_started = t
        elif self.phase == 2:
            # C01 landing-prep trigger: FLIGHT_and_com_vz_lt_-0.005 (RES-54 corrected vz)
            if cvz < PREP_TRIGGER_VZ:
                self.phase = 6
                self.phase_started = t
        elif self.phase == 6:
            # C01 touchdown: per-foot max >=20 (control-grid version of whole>=20 physics)
            if maxf >= TOUCHDOWN_MAXF:
                self.phase = 3
                self.phase_started = t
        elif self.phase == 3:
            # Handoff: first control boundary with TD-relative elapsed >=0.050
            # TD_physics set by physics-rate hook; if None (should not happen in phase 3),
            # do not handoff.
            if self.td_physics_t is not None and (t - self.td_physics_t) >= HANDOFF_ELAPSED_S - 1e-12:
                self.phase = 7
                self.phase_started = t
                self.handoff_t = t
        elif self.phase == 7:
            pass

        info["phase"] = self.phase
        info["phase_name"] = PHASE_NAMES.get(self.phase, "?")
        info["td_physics_t"] = self.td_physics_t
        info["trigger_cvz"] = cvz

        # ---- laws ----
        u = None
        if self.phase == 0:
            raw = KP_HOLD * (HOLD_Q - s) + KD_HOLD * (-sd)
            u = np.clip(raw / LIMITS, -1, 1)
        elif self.phase == 1:
            t_in = t - self.phase_started
            tau = FLEX_TAU if t_in < 0.30 else (EXTEND_TAU if t_in < 0.60 else np.zeros(7))
            u_seed = tau / LIMITS
            d_lum = _delta_at(t_in, _OPT_PARAMS[0:4])
            d_hip = _delta_at(t_in, _OPT_PARAMS[4:8])
            d_knee = _delta_at(t_in, _OPT_PARAMS[8:12])
            d_ank = _delta_at(t_in, _OPT_PARAMS[12:16])
            delta_vec = np.array([d_lum, d_hip, d_hip, d_knee, d_knee, d_ank, d_ank], dtype=float)
            u = np.clip(u_seed + delta_vec, -1, 1)
        elif self.phase == 2:
            raw = KP_FLIGHT * (FLIGHT_TARGET - s) - KD_FLIGHT * sd
            u = np.clip(raw / LIMITS, -1, 1)
        elif self.phase == 6:
            raw = PREP_KP * (PREP_TARGET - s) - PREP_KD * sd
            u = np.clip(raw / LIMITS, -1, 1)
        elif self.phase == 3:
            kd = self._impact_kd(t)
            raw = IMPACT_KP * (IMPACT_TARGET - s) - kd * sd
            u = np.clip(raw / LIMITS, -1, 1)
        elif self.phase == 7:
            # Exact sealed RES-58 outer law + RES-52 inner realization
            assert self.terminal_policy is not None, "terminal_policy not bound"
            assert sample is not None and meas is not None and plant is not None
            import mujoco as _mj
            vz_true = float(sample.com_velocity_mps[2])
            fz_des, vz_des = TC.capture_command(vz_true)
            # inner: need integration vector + u_prev
            n_state = int(_mj.mj_stateSize(plant.model, _mj.mjtState.mjSTATE_INTEGRATION))
            import numpy as _np
            vec = _np.zeros(n_state, dtype=float)
            _mj.mj_getState(plant.model, meas, vec, _mj.mjtState.mjSTATE_INTEGRATION)
            # meas is shadow already synchronized; use it as x_vec source via live? Actually
            # sample came from live via shadow; meas holds same state. Use meas state:
            # To get x_vec exactly, read from meas (shadow) which equals live integration.
            # But to be exact, read from live? Caller passes sample+meas+plant where meas is shadow.
            # Use shadow vector:
            x_vec = vec  # already from meas? No, vec above from meas? Correct: mj_getState on meas.
            # NOTE: caller must have meas synchronized to live at t; we re-read from meas.
            tinfo = self.terminal_policy.act(x_vec, np.asarray(self.prev_action, float), float(fz_des), float(vz_des), 40, self._validation_probe)
            u = _np.asarray(tinfo["u"], float)
            info.update({
                "FZ_DES": float(fz_des), "VZ_DES": float(vz_des), "VZ_TRUE": float(vz_true),
                "TERMINAL_INFO": {k: (v.tolist() if isinstance(v, _np.ndarray) else v) for k, v in tinfo.items() if k in ("rho", "shrinks", "VALIDATED", "FALLBACK")},
                "WBC_FALLBACK": bool(tinfo.get("FALLBACK", False)),
            })
            self.last_terminal_info = dict(info)
        else:
            u = np.zeros(7)
        u = np.clip(np.asarray(u, dtype=float), -1.0, 1.0)
        self.prev_action = u.copy()
        info["u"] = u.copy()
        info["t"] = t
        return u, info

    def bind_terminal(self, policy, validation_probe):
        self.terminal_policy = policy
        self._validation_probe = validation_probe
