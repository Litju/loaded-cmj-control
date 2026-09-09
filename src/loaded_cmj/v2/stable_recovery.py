#!/usr/bin/env python3
"""V2.1 frozen stable-recovery controller (RES-74, single architecture).

MISSION=RES10_SYNC_STABLE_RECOVERY_E11_TO_E12_001
EXPERIMENT_ID=EXP-RES10-SYNC-STABLE-RECOVERY-E11-E12-001

Outer: path-guided centroidal planner. State [COM x, px, COM z, pz, Hy]
  plus posture program q7t(s)/qd7t(s) from the frozen Phase-C manifold.
  Desired net wrench (Fx/Fz/Hdot) via feasible CoP projection (never
  independent infeasible Fx/Hdot), vertical via RES-58 structure with path
  vz target. Posture is lowest priority (P5); balance/support first.
Inner: sealed full-7DOF exact-forward RES-52 architecture + RES-54 COM vel
  + RES-55 foot vel + RES-57 support semantics. Vertical P2 > horizontal.
Modes: RISE (s advances iff inside tube) -> SETTLE (static lam=1 target,
  active dissipation until HANDOFF_READY sustained) -> HANDOFF (exact RES43).
No ladder, no random/RL/NOMAD, no gain search, no hidden pose-PD fallback,
no scorer input, no scalar direction channel, no direct writes (branches on copies only).
Sealed lineage: RES-52 SoftContactPolicy, RES-54 jacSubtreeCom, RES-55
  J_point, RES-57 continuity, RES-58 vertical structure, RES-73 CoP
  projection + validation/shrink machinery, RES-43 400/10 hold.
"""
import sys
from pathlib import Path
REPO = Path("/home/litju/Projects/loaded-cmj-control")
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "tools" / "res52"))
sys.path.insert(0, str(REPO / "tools"))
import numpy as np
import mujoco
from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
import core52 as C52
from evid_trace_v2 import centroidal_H_world
from scipy.optimize import lsq_linear

# Frozen constants (predeclared, no tuning during run)
MASS = 95.0
WEIGHT = 95 * 9.81
BW = WEIGHT
CONTROL_DT = 0.005
SUB = 40
RHO0 = 0.25
RHO_MIN = 0.0025
SHRINK = 0.5
MAX_SHRINK = 3
# Frozen PD core (RES43-identical) for the HANDOFF law. Named KP/KD/LIMITS.
KP = np.full(7, 400.0)
KD = np.full(7, 10.0)
LIMITS = np.array([250, 250, 250, 300, 300, 200, 200], dtype=float)
# Outer path-tracking (deterministic, declared)
TAU_X = 0.25       # COM-x velocity tracking time constant
TAU_H = 0.25       # Hy decay time constant
KZ_POS = 1.0       # vertical position correction gain (1/s)
KQ_SET = 15.0      # SETTLE joint position-restoring cascade gain (1/s)
KQ_RISE = 15.0     # RISE joint position-restoring cascade gain (1/s, =KQ_SET):
# RISE QDOT-only tracking lets position wind up (diagnosed lumbar -0.06 to
# +0.6 over 6 s: velocity errors integrate without position feedback). Gentle
# KQ closes the loop without fighting path dynamics. No integral in RISE
# (would wind on tracking lag).
KSKEW = 15.0       # skew (L/R pair-difference) restoring cascade gain (1/s,
# BOTH MODES): the symmetric task admits no skew; pair differences can only
# grow via numerical/contact feedback rectification (diagnosed 0.87 rad split
# passing all gates). Restores skew position through healthy QDOT rows.
KI_SET = 1.0       # SETTLE integral gain on joint position error (1/s)
LEAK_SET = 0.1     # SETTLE integrator leak (1/s, anti-windup with clamp)
I_CLAMP = 0.005    # SETTLE integrator clamp (rad/s velocity bias)
RHO_CAP_SET = 0.008  # SETTLE trust-box cap (probe/action steps stay inside
# contact stick (elastic, smooth G); larger steps break stick every probe,
# corrupting finite-diff G and strangling micro-correction via validation)
TOLF_SET = 0.06     # SETTLE validation strictness factor for FX/HDOT/FZ/VZ
TOLF_QD_SET = 0.12  # SETTLE validation FLOOR factor for QDOT rows (see _check:
# signal-relative tfq blends to 1.0 for macro arrival transients). Fixed
# micro-TOL froze macro corrections at SETTLE entry (diagnosed fall at s=1).
FX_LIM = 150.0     # symmetric small-force authority (rise is quasi-static)
HDOT_LIM = 60.0
FZ_MIN = 0.6 * BW
FZ_MAX = 1.5 * BW
# CoP support (world x, same as RES-73): interior margin 0.02
COP_X_MIN = -0.023
COP_X_MAX = 0.237
# Inner LS weights/scales (P2 vertical > P3/P4 balance > P5 posture).
# W_SYM enforces bilateral symmetry of the action step (weight 50, still far
# below DIST effective weight so per-foot contact safety is never outvoted;
# with symmetrized Q targets there is no sustained differential drive, so
# SYM only damps transient skew-rate): the task/plant is sagittal-symmetric
# pair action is pure antagonism (verified ±0.35 opposing ankles for a 2N
# task in diagnostics) that only excites contact stick-slip. Pair-diff rows
# are orthogonal to symmetric Du, so a hard symmetry weight preserves all
# symmetric wrench/posture authority exactly while crushing the nullspace
# runaway (verified: antagonism grew unbounded with W_SYM=0.5).
SCALES = {"FZ": 100.0, "FX": 50.0, "HDOT": 20.0, "VZ": 0.02, "DIST": 0.0005,
          "NVEL": 0.05, "QDOT": 0.3, "Q": 0.05}
WEIGHTS = {"W_FZ_WHOLE": 1.5, "W_FX": 0.8, "W_HDOT": 0.8, "W_VZ": 1.0,
           "W_NVEL": 0.5, "W_QDOT": 0.2, "W_Q": 0.15, "W_REG": 0.001,
           "W_SYM": 5.0,
           "W_DIST_ENGAGE": 4.0, "W_DIST_MAX": 4.0, "W_DIST_RETAIN": 2.0}
TOL = {"FZ_WHOLE": 20.0, "FX": 20.0, "HDOT": 15.0, "VZ": 0.01, "DIST": 0.0005,
       "NVEL": 0.05, "QDOT": 0.05, "Q": 0.01}
TRUST_REL = 0.1
# Viability tube (transit): pause progress, never jump
TUBE_MARGIN_MIN = 0.02
TUBE_PEN_MAX = 0.005
TUBE_FZ_PEAK_BW = 5.0
QNAMES = ["lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]


class StableRecoveryController:
    def __init__(self, plant, manifold, handoff_spec, t_rise, probes, validation_probe, gap_hh):
        self.plant = plant
        self.m = plant.model
        self.probes = probes
        self.vprobe = validation_probe
        self.gap_hh = gap_hh
        self.T_RISE = float(t_rise)
        order = list(manifold["order"])
        self.lams = np.array([float(k) for k in order], float)
        # rows in QADR order [root_tx,tz,ry, lumbar,lhip,rhip,lknee,rknee,lank,rank]
        self.qpos_nodes = np.array(
            [list(manifold["nodes"][str(k)]["x"]) for k in order], float)
        self.qadr10 = [int(plant.idx.qadr[n]) for n in (["root_tx", "root_tz", "root_ry"] + QNAMES)]
        self.jvadr = [int(plant.idx.vadr[n]) for n in QNAMES]
        self.spec = dict(handoff_spec)
        th = handoff_spec.get("THRESHOLDS", handoff_spec)
        self.th = dict(th)
        self.s = 0.0
        self.mode = "RISE"
        self.u_prev = np.zeros(7)
        self.handoff_ok_since = None
        self.rho_eff = RHO0
        self.validation_fails = 0
        self.shrinks = 0
        self.fallbacks = 0
        self.max_err = 0.0
        self.active_cross = 0
        self.pauses = 0
        self.validations = 0
        self._int7 = np.zeros(7)  # SETTLE-only leaky integrator (bias rejection)

    # ---- path ----
    def target_at(self, s):
        s = float(np.clip(s, 0.0, 1.0))
        lam = self.lams
        q = np.array([float(np.interp(s, lam, self.qpos_nodes[:, c])) for c in range(10)], float)
        dsdt = (1.0 / self.T_RISE) if self.mode == "RISE" else 0.0
        eps = 1e-4
        qd = np.array([
            (float(np.interp(min(1.0, s + eps), lam, self.qpos_nodes[:, c]))
             - float(np.interp(max(0.0, s - eps), lam, self.qpos_nodes[:, c]))) / (2 * eps) * dsdt
            for c in range(10)], float)
        return q, qd

    # ---- inner branch machinery (RES-52/73 sealed pattern, +q rows) ----
    def branch_run(self, pd, u, x_vec, n_sub, com0, cv0, H0):
        self.plant.apply_action(pd, u)
        Fz_sum = 0.0
        Fx_sum = 0.0
        for _ in range(n_sub):
            mujoco.mj_step(self.m, pd)
            sm_live = self.plant.foot_contact_summary(pd)
            Fx_sum += float(np.asarray(sm_live["whole_force"], float)[0])
            Fz_sum += float(sm_live["whole_Fz"])
        mujoco.mj_forward(self.m, pd)
        com1 = self.plant.center_of_mass(pd)
        cv1 = self.plant.center_of_mass_velocity(pd)
        H1 = centroidal_H_world(self.m, pd, com1)
        sm1 = self.plant.foot_contact_summary(pd)
        st = {}
        for side, fg, fb in (("L", self.plant.idx.left_foot_geom, self.plant.idx.left_foot_body),
                             ("R", self.plant.idx.right_foot_geom, self.plant.idx.right_foot_body)):
            rows = [i for i in range(pd.ncon)
                    if (int(pd.contact[i].geom1) == self.plant.idx.floor_geom
                        or int(pd.contact[i].geom2) == self.plant.idx.floor_geom)
                    and (int(pd.contact[i].geom1) == fg or int(pd.contact[i].geom2) == fg)]
            if rows:
                i = rows[int(np.argmin([float(pd.contact[j].dist) for j in rows]))]
                dist = float(pd.contact[i].dist)
                nvel = float(pd.efc_vel[pd.contact[i].efc_address])
            else:
                dist = float(pd.geom_xpos[fg][2]) - self.gap_hh
                p_low = np.asarray(pd.geom_xpos[fg], float).copy()
                p_low[2] -= self.gap_hh
                Jp = np.zeros((3, self.m.nv))
                Jr = np.zeros((3, self.m.nv))
                mujoco.mj_jac(self.m, pd, Jp, Jr, np.asarray(p_low, float), int(fb))
                nvel = float((Jp @ np.asarray(pd.qvel, float))[2])
            st[side] = (dist, nvel)
        dpx = MASS * (cv1[0] - cv0[0])
        dHy = float(H1[1] - H0[1])
        qd = np.asarray(pd.qvel, float)[[self.plant.idx.vadr[n] for n in
            ("lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")]]
        q7 = np.asarray(pd.qpos, float)[[self.plant.idx.qadr[n] for n in
            ("lumbar", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle")]]
        y = np.array([dpx / CONTROL_DT, float(dHy / CONTROL_DT), Fz_sum / n_sub,
                      st["L"][0], st["R"][0], st["L"][1], st["R"][1], float(cv1[2]),
                      qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6],
                      q7[0], q7[1], q7[2], q7[3], q7[4], q7[5], q7[6]], dtype=float)
        flags = (bool(sm1["active_left"]), bool(sm1["active_right"]))
        return y, flags, {}

    def restore(self, pd, x_vec):
        mujoco.mj_setState(self.m, pd, np.ascontiguousarray(x_vec), mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.m, pd)

    # ---- outer: path-guided centroidal + posture ----
    def outer_desired(self, com, cv, H, q7_meas):
        # Mode-appropriate outer authority (frozen): RISE tracks the moving
        # path briskly; SETTLE station-keeps quietly (sub-breakaway actions so
        # contact stick holds and no limit-cycle buzz is excited). Corrective
        # for SETTLE excitation observed in diagnostics; RISE untouched (scaling
        # runs, which stop at s=1, are bit-identical under this change).
        if self.mode == "SETTLE":
            POS_G, TX, TH, KZ = 0.0, 0.6, 0.6, 0.3
        else:
            POS_G, TX, TH, KZ = 1.0, TAU_X, TAU_H, KZ_POS
        # path targets at current s (kinematic COM/trunk from node table via probe forward)
        q_path, qd_path = self.target_at(self.s)
        # path COM/trunk: evaluate kinematically on validation probe (disposable;
        # validation restores it before use, so sequential reuse is safe)
        pd = self.vprobe
        mujoco.mj_setState(self.m, pd, np.ascontiguousarray(self._last_x_vec),
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.m, pd)
        # set path qpos onto probe
        for c, nm in enumerate(["root_tx", "root_tz", "root_ry"] + QNAMES):
            pd.qpos[int(self.plant.idx.qadr[nm])] = float(q_path[c])
        mujoco.mj_forward(self.m, pd)
        com_t = self.plant.center_of_mass(pd)
        trunk_t = float(pd.qpos[int(self.plant.idx.qadr["root_ry"])]
                        + pd.qpos[int(self.plant.idx.qadr["lumbar"])])
        # path COM velocity (finite diff of node COMs is complex; use qd-mapped approx:
        # use target velocity from node interpolation of COM via two extra evals)
        q_p, _ = self.target_at(min(1.0, self.s + 1e-3))
        q_m, _ = self.target_at(max(0.0, self.s - 1e-3))
        for c, nm in enumerate(["root_tx", "root_tz", "root_ry"] + QNAMES):
            pd.qpos[int(self.plant.idx.qadr[nm])] = float(q_p[c])
        mujoco.mj_forward(self.m, pd)
        com_p = self.plant.center_of_mass(pd)
        for c, nm in enumerate(["root_tx", "root_tz", "root_ry"] + QNAMES):
            pd.qpos[int(self.plant.idx.qadr[nm])] = float(q_m[c])
        mujoco.mj_forward(self.m, pd)
        com_m = self.plant.center_of_mass(pd)
        dsdt = (1.0 / self.T_RISE) if self.mode == "RISE" else 0.0
        cv_t = (np.asarray(com_p, float) - np.asarray(com_m, float)) / 2e-3 * dsdt
        q7t = np.asarray(q_path[3:10], float)
        # Symmetrize pair targets (hips/knees/ankles): the task, path intent,
        # and init state are sagittal-symmetric; raw node tables carry
        # micro-asymmetric solver noise (~1e-4) that Q rows would otherwise
        # track into a skew random walk (diagnosed 0.87 rad lunge passing all
        # support/force gates). Symmetric targets restore skew position.
        for (a, b) in ((1, 2), (3, 4), (5, 6)):
            m_ = 0.5 * (float(q7t[a]) + float(q7t[b]))
            q7t[a] = m_
            q7t[b] = m_
        # qd7 program: RISE tracks path velocity plus gentle position cascade;
        # SETTLE adds stronger position-restoring cascade KQ*(q7t-q7) (frozen)
        # because branch end-q sensitivity is second-order-weak while qd
        # response is first-order: position converges through velocity
        # tracking. Without it ankles random-walk out of the E12 envelope
        # (diagnosed 1.5e-4/s drift).
        qd_prog = np.asarray(qd_path[3:10], float)
        for (a, b) in ((1, 2), (3, 4), (5, 6)):
            m_ = 0.5 * (float(qd_prog[a]) + float(qd_prog[b]))
            qd_prog[a] = m_
            qd_prog[b] = m_
        # RISE position cascade (gentle): qd = path + KQ_RISE*(q7t - q7m).
        # Prevents position windup from velocity-tracking residuals.
        # SETTLE uses stronger KQ_SET plus leaky integral (bias rejection).
        q7m_now = np.asarray(q7_meas, float)
        if self.mode == "RISE":
            qd7t = qd_prog + KQ_RISE * (np.asarray(q7t, float) - q7m_now)
            self._int7[:] = 0.0
        else:
            q7m = np.asarray(q7_meas, float)
            err7 = np.asarray(q7t, float) - q7m
            # Leaky integral (bias rejection): winds until position error is
            # eliminated; leak+clamp prevent windup. Update at control rate.
            self._int7 = np.clip(self._int7 + (KI_SET * err7 - LEAK_SET * self._int7) * CONTROL_DT,
                                 -I_CLAMP, I_CLAMP)
            qd7t = qd_prog + KQ_SET * err7 + self._int7
        # Skew restoring (both modes, after program/cascade): drive each
        # pair-difference to zero through QDOT rows. Symmetric tasks admit
        # no skew; this overpowers rectified-feedback skew growth.
        qd7t = np.asarray(qd7t, float).copy()
        for (a, b) in ((1, 2), (3, 4), (5, 6)):
            sk = float(q7m_now[a]) - float(q7m_now[b])
            qd7t[a] -= 0.5 * KSKEW * sk
            qd7t[b] += 0.5 * KSKEW * sk
        # desired wrench
        px = MASS * float(cv[0])
        Hy = float(H[1])
        cx = float(com[0])
        cz = float(com[2])
        vx_des = float(cv_t[0]) + POS_G * (float(com_t[0]) - cx)
        Fx_raw = MASS * (vx_des - float(cv[0])) / TX
        Hd_raw = -Hy / TH
        Fx_cl = float(np.clip(Fx_raw, -FX_LIM, FX_LIM))
        Hd_cl = float(np.clip(Hd_raw, -HDOT_LIM, HDOT_LIM))
        vz_des = float(cv_t[2]) + KZ * (float(com_t[2]) - float(com[2]))
        fz_raw = WEIGHT + MASS * (vz_des - float(cv[2])) / CONTROL_DT
        fz_des = float(np.clip(fz_raw, FZ_MIN, FZ_MAX))
        vz_tgt_next = float(cv[2]) + CONTROL_DT * (fz_des / MASS - 9.81)
        # CoP projection (RES-73 sealed pattern)
        Fz_proj = float(fz_des)
        offset = (-cz * Fx_cl - Hd_cl) / Fz_proj if Fz_proj > 1e-9 else 0.0
        x_cop = cx + offset
        x_cop_cl = float(np.clip(x_cop, COP_X_MIN, COP_X_MAX))
        if abs(x_cop - x_cop_cl) > 1e-9:
            a = cz
            b = 1.0
            c = (x_cop_cl - cx) * Fz_proj
            wFx = 1 / (50 ** 2)
            wHd = 1 / (20 ** 2)
            r = a * Fx_cl + b * Hd_cl + c
            denom = (a * a / wFx + b * b / wHd)
            lam_ = r / denom if denom != 0 else 0.0
            Fx_proj = float(np.clip(Fx_cl - lam_ * a / wFx, -FX_LIM, FX_LIM))
            Hd_line = -cz * Fx_proj - (x_cop_cl - cx) * Fz_proj
            Hd_proj = float(np.clip(Hd_line, -HDOT_LIM, HDOT_LIM))
            if abs(Hd_proj - Hd_line) > 1e-9:
                Fx_from = (-Hd_proj - (x_cop_cl - cx) * Fz_proj) / cz if abs(cz) > 1e-9 else Fx_proj
                Fx_proj = float(np.clip(Fx_from, -FX_LIM, FX_LIM))
                Hd_proj = float(Hd_proj)
            Fx_des, Hd_des = Fx_proj, Hd_proj
        else:
            Fx_des, Hd_des = Fx_cl, Hd_cl
        return Fx_des, Hd_des, float(fz_des), float(vz_tgt_next), np.asarray(q7t, float), np.asarray(qd7t, float)

    # ---- LS solve with validation/shrink (RES-73 pattern + Q rows) ----
    def _build_G(self, x_vec, u_prev, com0, cv0, H0):
        pd0 = self.probes[0]
        self.restore(pd0, x_vec)
        com0b = self.plant.center_of_mass(pd0)
        cv0b = self.plant.center_of_mass_velocity(pd0)
        H0b = centroidal_H_world(self.m, pd0, com0b)
        y_nom, flags_nom, _ = self.branch_run(pd0, np.asarray(u_prev, float), x_vec, SUB, com0b, cv0b, H0b)
        G = np.zeros((22, 7))
        for j in range(7):
            e = np.zeros(7)
            e[j] = self.rho_eff
            up = np.clip(np.asarray(u_prev, float) + e, -1, 1)
            um = np.clip(np.asarray(u_prev, float) - e, -1, 1)
            dp = float(up[j] - u_prev[j])
            dm = float(u_prev[j] - um[j])
            if dp <= 0 and dm <= 0:
                continue
            pdp = self.probes[1 + 2 * j]
            pdm = self.probes[2 + 2 * j]
            self.restore(pdp, x_vec)
            self.restore(pdm, x_vec)
            yp, fp, _ = self.branch_run(pdp, up, x_vec, SUB, com0b, cv0b, H0b)
            ym, fm, _ = self.branch_run(pdm, um, x_vec, SUB, com0b, cv0b, H0b)
            cross = (fp != flags_nom or fm != flags_nom)
            if cross:
                self.active_cross += 1
            if cross:
                if fp == flags_nom and dp > 0:
                    G[:, j] = (yp - y_nom) / dp
                elif fm == flags_nom and dm > 0:
                    G[:, j] = (y_nom - ym) / dm
                else:
                    G[:, j] = 0
            else:
                if dp > 0 and dm > 0:
                    G[:, j] = (yp - ym) / (dp + dm)
                elif dp > 0:
                    G[:, j] = (yp - y_nom) / dp
                else:
                    G[:, j] = (y_nom - ym) / dm
        # Contact-measurement symmetrization (both modes): average L/R dist
        # and normal-velocity nominals. Per-foot penetration/velocity diffs at
        # 0.1um scale feed back differentially (8000x effective DIST weight)
        # into skew growth (diagnosed x2/step snap); the symmetric task admits
        # no sustained differential. State rows (q/qd) stay real so skew
        # position remains observable for Kskew correction. Per-foot tube and
        # support gates (unaveraged) retain safety granularity.
        y_nom = np.asarray(y_nom, float).copy()
        y_nom[3] = y_nom[4] = 0.5 * (y_nom[3] + y_nom[4])
        y_nom[5] = y_nom[6] = 0.5 * (y_nom[5] + y_nom[6])
        # Effectiveness symmetrization (both modes): average L/R pair COLUMNS
        # of G (hips/knees/ankles). Finite-diff G across nonsmooth contact
        # carries pair-asymmetric discretization noise; with symmetric demands
        # it produces asymmetric Du that pumps roll rocking (diagnosed:
        # FzL/FzR 547/547 -> 318/615 with symmetric actions, then skew snap).
        # Averaged columns force exactly symmetric Du for symmetric demands
        # (symmetric tracking bit-identical in exact arithmetic, more accurate
        # on average). Per-foot safety granularity retained in gates.
        for (i, j) in ((1, 2), (3, 4), (5, 6)):
            Gm = 0.5 * (G[:, i] + G[:, j])
            G[:, i] = Gm
            G[:, j] = Gm
        return G, y_nom, flags_nom, (com0b, cv0b, H0b)

    def _solve_ls(self, G, y_nom, u_prev, Fx_des, Hd_des, Fz_des, Vz_des, q7t, qd7t):
        # Mode-appropriate servo stiffness (frozen): SETTLE station-keeps with
        # 8x tracking stiffness (same relative priorities, all tracking rows
        # scaled equally) because micro-signal tracking (sub-Newton) needs
        # stiffer servo than RISE path tracking; RISE factor is 1.0 exactly
        # (scaling runs, which stop at s=1, are bit-identical under this change).
        # SETTLE QDOT base is raised 0.2->0.8 (tie with FX at 6.4 effective):
        # KQ-cascade position correction must outvote wrench-loop residuals to
        # unwind RISE arrival posture (diagnosed lumbar +0.28 parked: KQ
        # correction perpetually outvoted at 0.2). P-order tie (not inversion);
        # vertical stays supreme (12.0). RISE QDOT stays 0.2 (proven).
        stiff = 8.0 if self.mode == "SETTLE" else 1.0
        wqdot_base = 0.8 if self.mode == "SETTLE" else WEIGHTS["W_QDOT"]
        WF = {"W_FX": WEIGHTS["W_FX"] * stiff, "W_HDOT": WEIGHTS["W_HDOT"] * stiff,
              "W_FZ_WHOLE": WEIGHTS["W_FZ_WHOLE"] * stiff, "W_VZ": WEIGHTS["W_VZ"] * stiff,
              "W_QDOT": wqdot_base * stiff}
        rows = []
        tgt = []
        rows.append(WF["W_FX"] * G[0] / SCALES["FX"])
        tgt.append(WF["W_FX"] * (Fx_des - y_nom[0]) / SCALES["FX"])
        rows.append(WF["W_HDOT"] * G[1] / SCALES["HDOT"])
        tgt.append(WF["W_HDOT"] * (Hd_des - y_nom[1]) / SCALES["HDOT"])
        rows.append(WF["W_FZ_WHOLE"] * G[2] / SCALES["FZ"])
        tgt.append(WF["W_FZ_WHOLE"] * (Fz_des - y_nom[2]) / SCALES["FZ"])
        rows.append(WEIGHTS["W_NVEL"] * G[5] / SCALES["NVEL"])
        tgt.append(WEIGHTS["W_NVEL"] * (0.0 - y_nom[5]) / SCALES["NVEL"])
        rows.append(WEIGHTS["W_NVEL"] * G[6] / SCALES["NVEL"])
        tgt.append(WEIGHTS["W_NVEL"] * (0.0 - y_nom[6]) / SCALES["NVEL"])
        rows.append(WF["W_VZ"] * G[7] / SCALES["VZ"])
        tgt.append(WF["W_VZ"] * (Vz_des - y_nom[7]) / SCALES["VZ"])
        for k in range(7):
            rows.append(WF["W_QDOT"] * G[8 + k] / SCALES["QDOT"])
            tgt.append(WF["W_QDOT"] * (float(qd7t[k]) - y_nom[8 + k]) / SCALES["QDOT"])
        # Q (position) rows: RISE ONLY. RISE needs position feedback to prevent
        # posture drift over the multi-second track (scaling-verified); SETTLE
        # omits them because 5-ms end-q sensitivity is second-order-weak and
        # saturated Q rows chatter the micro-hold (diagnosed). SETTLE posture
        # converges via KQ-cascaded QDOT tracking instead.
        if self.mode == "RISE":
            for k in range(7):
                rows.append(WEIGHTS["W_Q"] * G[15 + k] / SCALES["Q"])
                tgt.append(WEIGHTS["W_Q"] * (float(q7t[k]) - y_nom[15 + k]) / SCALES["Q"])
        for idx, pen_max, pen_min in [(3, 0.009, 0.0005), (4, 0.009, 0.0005)]:
            d0v = y_nom[idx]
            if d0v > 0:
                rows.append(WEIGHTS["W_DIST_ENGAGE"] * G[idx] / SCALES["DIST"])
                tgt.append(WEIGHTS["W_DIST_ENGAGE"] * (0.0 - d0v) / SCALES["DIST"])
            elif d0v < -pen_max:
                rows.append(WEIGHTS["W_DIST_MAX"] * G[idx] / SCALES["DIST"])
                tgt.append(WEIGHTS["W_DIST_MAX"] * (-pen_max - d0v) / SCALES["DIST"])
            elif d0v > -pen_min:
                # RETAIN: RISE uses sealed deepen target (load-bearing for
                # engagement; hold-current stalls rise at s=0.3, diagnosed);
                # SETTLE holds current (deepening biases ankles plantarward
                # out of the envelope in quiet standing, diagnosed).
                if self.mode == "SETTLE":
                    rows.append(WEIGHTS["W_DIST_RETAIN"] * G[idx] / SCALES["DIST"])
                    tgt.append(0.0)
                else:
                    rows.append(WEIGHTS["W_DIST_RETAIN"] * G[idx] / SCALES["DIST"])
                    tgt.append(WEIGHTS["W_DIST_RETAIN"] * (-pen_min - d0v) / SCALES["DIST"])
        A = np.vstack([np.asarray(rows, float), WEIGHTS["W_REG"] * np.eye(7)])
        b = np.concatenate([np.asarray(tgt, float), np.zeros(7)])
        # Bilateral symmetry prior (BOTH MODES): the task/path/init state are
        # sagittal-symmetric, so the desired solution is symmetric.
        # Antisymmetric pair action is pure antagonism (diagnosed ±0.35
        # opposing ankles exciting stick-slip buzz in the micro-hold) and,
        # worse, the skew nullspace (lunge: one leg bent, one straight, net
        # wrench preserved) lets RISE drift into a split posture that passes
        # support/force gates but cannot settle (diagnosed 400mrad split at
        # s=1 with clean gates). Symmetric rows preserve all symmetric
        # wrench/posture authority exactly (pair-diff nullspace only).
        for (i, j) in ((1, 2), (3, 4), (5, 6)):
            e = np.zeros(7)
            e[i] = 1.0
            e[j] = -1.0
            A = np.vstack([A, WEIGHTS["W_SYM"] * e.reshape(1, -1)])
            b = np.concatenate([b, np.zeros(1)])
        lo = np.maximum(-1.0 - np.asarray(u_prev, float), -self.rho_eff)
        hi = np.minimum(1.0 - np.asarray(u_prev, float), self.rho_eff)
        res = lsq_linear(A, b, bounds=(lo, hi), method="bvls", tol=1e-10, max_iter=200)
        return np.asarray(res.x, float)

    def _check(self, y_val, y_pred, y_nom, flags_val, flags_nom):
        err = np.abs(y_val - y_pred)
        pred = np.abs(y_pred - y_nom)
        self.max_err = max(self.max_err, float(np.max(err)))
        # Validation strictness follows servo stiffness (SETTLE tracks
        # micro-signals, so prediction must be micro-accurate). QDOT factor is
        # SIGNAL-RELATIVE (frozen): macro arrival transients validate loosely
        # so corrections flow; micro-hold validates strictly so chatter coasts.
        tf = TOLF_SET if self.mode == "SETTLE" else 1.0
        if self.mode == "SETTLE":
            qdscale = 0.0
            if hasattr(self, "_last_qd7t"):
                qdscale = max(qdscale, float(np.max(np.abs(self._last_qd7t))))
            if hasattr(self, "_last_qd7m"):
                qdscale = max(qdscale, float(np.max(np.abs(self._last_qd7m))))
            tfq = min(1.0, max(TOLF_QD_SET, qdscale / 0.02))
        else:
            tfq = 1.0
        checks = [(0, "FX"), (1, "HDOT"), (2, "FZ_WHOLE"), (3, "DIST"), (4, "DIST"),
                  (5, "NVEL"), (6, "NVEL"), (7, "VZ")]
        for idx, nm in checks:
            tol = TOL[nm] * (tf if nm in ("FX", "HDOT", "FZ_WHOLE", "VZ") else 1.0)
            if not (err[idx] <= tol + TRUST_REL * pred[idx]):
                return False
        for k in range(7):
            if not (err[8 + k] <= TOL["QDOT"] * tfq + TRUST_REL * pred[8 + k]):
                return False
        if self.mode == "RISE":
            for k in range(7):
                if not (err[15 + k] <= TOL["Q"] + TRUST_REL * pred[15 + k]):
                    return False
        return bool(flags_val == flags_nom)

    def step(self, x_vec, u_prev, t, com, cv, H, sample, sm, trunkW, in_envelope):
        self._last_x_vec = np.ascontiguousarray(x_vec).copy()
        self._trunkW = float(trunkW)
        info = {"mode": self.mode, "s": float(self.s)}
        if self.mode == "HANDOFF":
            u = self._res43_hold(sample)
            self.u_prev = np.asarray(u, float).copy()
            info["u"] = self.u_prev.copy()
            return self.u_prev.copy(), info
        Fx_des, Hd_des, Fz_des, Vz_des, q7t, qd7t = self.outer_desired(
            com, cv, H, np.asarray(sample.joint_position_rad, float))
        self._last_qd7t = np.asarray(qd7t, float).copy()
        self._last_qd7m = np.asarray(sample.joint_velocity_radps, float).copy()
        info.update({"Fx_des": float(Fx_des), "Hd_des": float(Hd_des), "Fz_des": float(Fz_des)})
        G, y_nom, flags_nom, base = self._build_G(x_vec, u_prev, com, cv, H)
        Du = self._solve_ls(G, y_nom, u_prev, Fx_des, Hd_des, Fz_des, Vz_des, q7t, qd7t)
        u_try = np.clip(np.asarray(u_prev, float) + Du, -1, 1)
        self.restore(self.vprobe, x_vec)
        com0b, cv0b, H0b = base
        y_val, flags_val, _ = self.branch_run(self.vprobe, u_try, x_vec, SUB, com0b, cv0b, H0b)
        y_pred = y_nom + G @ Du
        self.validations += 1
        if self._check(y_val, y_pred, y_nom, flags_val, flags_nom):
            cap = RHO_CAP_SET if self.mode == "SETTLE" else RHO0
            self.rho_eff = min(cap, self.rho_eff * 1.189207115002721)
            u = u_try
            validated = True
        else:
            # deterministic shrink/re-solve (same pattern as RES-73, +Q rows tolerant)
            validated = False
            u = np.asarray(u_prev, float).copy()
            rho = self.rho_eff
            for _ in range(MAX_SHRINK):
                rho = max(RHO_MIN, rho * SHRINK)
                self.shrinks += 1
                self.rho_eff = rho
                G2, y_nom2, flags_nom2, base2 = self._build_G(x_vec, u_prev, com, cv, H)
                Du2 = self._solve_ls(G2, y_nom2, u_prev, Fx_des, Hd_des, Fz_des, Vz_des, q7t, qd7t)
                u_try2 = np.clip(np.asarray(u_prev, float) + Du2, -1, 1)
                self.restore(self.vprobe, x_vec)
                c0, v0, Hh = base2
                y_val2, flags_val2, _ = self.branch_run(self.vprobe, u_try2, x_vec, SUB, c0, v0, Hh)
                y_pred2 = y_nom2 + G2 @ Du2
                if self._check(y_val2, y_pred2, y_nom2, flags_val2, flags_nom2):
                    cap2 = RHO_CAP_SET if self.mode == "SETTLE" else RHO0
                    self.rho_eff = min(cap2, rho * 1.189207115002721)
                    u = u_try2
                    validated = True
                    break
            if not validated:
                self.fallbacks += 1
                self.validation_fails += 1
                # Unvalidated action: hold previous AND pause (never advance on
                # unvalidated control while tracking a moving target).
                self.pauses += 1
                info["paused"] = True
                info["fallback_hold"] = True
                self.u_prev = np.asarray(u, float).copy()
                info["u"] = self.u_prev.copy()
                return self.u_prev.copy(), info
        info["validated"] = bool(validated)
        # tube gates progress (P1-P3 live)
        if not self._tube_ok(sample, sm):
            self.pauses += 1
            info["paused"] = True
        else:
            if self.mode == "RISE":
                self.s = min(1.0, self.s + CONTROL_DT / self.T_RISE)
                if self.s >= 1.0:
                    self.mode = "SETTLE"
            elif self.mode == "SETTLE":
                if self._handoff_ready(sample, sm, H) and bool(in_envelope):
                    if self.handoff_ok_since is None:
                        self.handoff_ok_since = float(sample.time)
                    sustain = float(self.spec.get("HANDOFF_SUSTAIN_S", 0.05))
                    if float(sample.time) - self.handoff_ok_since >= sustain - 1e-9:
                        self.mode = "HANDOFF"
                        info["handoff"] = True
                else:
                    self.handoff_ok_since = None
        self.u_prev = np.asarray(u, float).copy()
        info["u"] = self.u_prev.copy()
        return self.u_prev.copy(), info

    def _res43_hold(self, sample):
        q7 = np.asarray(sample.joint_position_rad, float)
        qd7 = np.asarray(sample.joint_velocity_radps, float)
        raw = KP * (np.zeros(7) - q7) - KD * qd7
        return np.clip(raw / LIMITS, -1.0, 1.0)

    def _tube_ok(self, sample, sm):
        if not (float(sm["left_Fz"]) > 10.0 and float(sm["right_Fz"]) > 10.0):
            return False
        if float(sample.support_margin_m) < TUBE_MARGIN_MIN:
            return False
        if float(sample.max_penetration_m) > TUBE_PEN_MAX:
            return False
        if float(sm["whole_Fz"]) > TUBE_FZ_PEAK_BW * WEIGHT:
            return False
        if bool(sample.fall_contact) or bool(sample.prohibited_contact):
            return False
        return True

    def _handoff_ready(self, sample, sm, H):
        th = self.th
        cv = np.asarray(sample.com_velocity_mps, float)
        qd = np.asarray(sample.joint_velocity_radps, float)
        if abs(float(cv[0])) > float(th["COM_VX_ABS_MAX"]):
            return False
        if abs(float(cv[2])) > float(th["COM_VZ_ABS_MAX"]):
            return False
        if abs(float(H[1])) > float(th["HY_ABS_MAX"]):
            return False
        if abs(float(sample.qvel[2])) > float(th["ROOT_RY_RATE_ABS_MAX"]):
            return False
        if abs(float(self._trunkW)) > float(th["WORLD_TRUNK_TILT_RATE_ABS_MAX"]):
            return False
        if float(np.max(np.abs(qd))) > float(th["JOINT_QDOT_ABS_MAX"]):
            return False
        if float(sample.support_margin_m) < float(th.get("SUPPORT_MARGIN_MIN", 0.05)):
            return False
        if not (float(sm["left_Fz"]) > 10.0 and float(sm["right_Fz"]) > 10.0):
            return False
        if float(sample.max_penetration_m) > 0.010:
            return False
        return True
