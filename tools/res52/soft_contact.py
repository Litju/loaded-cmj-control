"""RES-52 Phase C: full 7-DOF local exact-forward soft-contact realization.

At every 5-ms control update:
  1. read the exact synchronized integration state (sample-before-update);
  2. restore that state into independent branch MjData copies;
  3. evaluate the PREVIOUS APPLIED action (continuity nominal) over exactly
     one 5-ms control interval with exact MuJoCo forward dynamics;
  4. perturb each of the 7 actuator commands two-sidedly by the current trust
     radius (deterministic) and run exact 5-ms branches on the copies;
  5. build the local map Dy = G(x) Du with outputs
     y = [Fz_whole, Fz_L, Fz_R, dist_L, dist_R, nvel_L, nvel_R, COM vz];
     dist is the signed contact distance (negative = penetrating; positive
     geometric gap when the foot has no floor contact);
  6. solve a bounded deterministic least-squares for Du (bvls active set) with
     one-sided contact-distance retention (re-engage when separated, never
     exceed PEN_MAX penetration) and COM-vz profile-integral regulation;
     bounds: |u_k - u_{k-1}| <= rho (trust region as action rate), |u| <= 1;
  7. validate the chosen action with one independent exact 5-ms branch;
     trust tolerance is per-channel floor + relative share of the predicted
     change (predeclared);
  8. on trust failure deterministically shrink the trust region and resolve
     (no outer search); after the allowed shrinks fall back to the previous
     applied action (exact branch, no prediction involved);
  9. apply only the validated bounded action to LIVE and advance with exact
     mj_step.

No direct qpos/qvel/force/contact writes. No EXT_DIR. No tensile demand.
Sign conventions: penetration = max(0, -contact_distance); foot normal
velocity positive = SEPARATING (contact distance increasing).
"""
from __future__ import annotations

import numpy as np
import mujoco
from scipy.optimize import lsq_linear

Y_NAMES = ["FZ_WHOLE", "FZ_L", "FZ_R", "DIST_L", "DIST_R", "NVEL_L", "NVEL_R", "COM_VZ",
           "QD_0", "QD_1", "QD_2", "QD_3", "QD_4", "QD_5", "QD_6",
           "FZMIN_WHOLE", "FZMIN_L", "FZMIN_R"]
NY = len(Y_NAMES)


def true_foot_point_velocity(model, data, body_id: int, point_world: np.ndarray) -> np.ndarray:
    """RES-55 true foot-point velocity authority (identical to core52).

    v_point = J_point(q) @ qvel, non-mutating same-state. Prohibited:
    mj_objectVelocity.linear + omega x (p - xpos).
    """
    Jp = np.zeros((3, model.nv), dtype=np.float64)
    Jr = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jac(model, data, Jp, Jr, np.asarray(point_world, dtype=np.float64),
                  int(body_id))
    return (Jp @ np.asarray(data.qvel, dtype=np.float64)).copy()


class SoftContactPolicy:
    def __init__(self, constants: dict, plant, probes: list):
        self.k = constants
        self.plant = plant
        self.m = plant.model
        self.rho_eff = float(constants["RHO0_U"])
        self.probes = probes  # >= 17 independent MjData copies
        self.gap_hh = None
        self.vadr = [plant.idx.vadr[n] for n in
                     ("lumbar", "left_hip", "right_hip", "left_knee",
                      "right_knee", "left_ankle", "right_ankle")]
        self.active_set_crossings = 0
        self.trust_shrink_events = 0
        self.trust_fallbacks = 0
        self.max_validation_error = 0.0

    # ---------------- outputs from a probe copy ----------------
    def y_of(self, pd) -> np.ndarray:
        p = self.plant
        sm = p.foot_contact_summary(pd)
        gap_hh = self.gap_hh
        st = {}
        for side, fg, fb in (("L", p.idx.left_foot_geom, p.idx.left_foot_body),
                             ("R", p.idx.right_foot_geom, p.idx.right_foot_body)):
            rows = [i for i in range(pd.ncon)
                    if (int(pd.contact[i].geom1) == p.idx.floor_geom
                        or int(pd.contact[i].geom2) == p.idx.floor_geom)
                    and (int(pd.contact[i].geom1) == fg
                         or int(pd.contact[i].geom2) == fg)]
            if rows:
                i = rows[int(np.argmin([float(pd.contact[j].dist) for j in rows]))]
                dist = float(pd.contact[i].dist)
                nvel = float(pd.efc_vel[pd.contact[i].efc_address])
            else:
                dist = float(pd.geom_xpos[fg][2]) - gap_hh
                # RES-55 authority: J_point @ qvel at the exact lowest corner.
                # Prohibited: mj_objectVelocity.linear + omega x (p - xpos).
                # Note: previous code used geom center for velocity; z identical
                # (vertical offset contributes zero to z), now unified to p_low.
                p_low = np.asarray(pd.geom_xpos[fg], float).copy()
                p_low[2] -= gap_hh
                nvel = float(true_foot_point_velocity(self.m, pd, fb, p_low)[2])
            st[side] = (dist, nvel)
        cv = p.center_of_mass_velocity(pd)
        qd = np.asarray(pd.qvel, dtype=float)[self.vadr]
        return np.array([
            float(sm["whole_Fz"]), float(sm["left_Fz"]), float(sm["right_Fz"]),
            st["L"][0], st["R"][0], st["L"][1], st["R"][1], float(cv[2]),
            qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qd[6],
        ], dtype=float)

    def contact_flags(self, pd) -> tuple[bool, bool]:
        sm = self.plant.foot_contact_summary(pd)
        return bool(sm["active_left"]), bool(sm["active_right"])

    def branch_run(self, pd, u: np.ndarray, n_sub: int) -> np.ndarray:
        self.plant.apply_action(pd, u)
        fz_min = np.array([np.inf, np.inf, np.inf])
        for _ in range(n_sub):
            mujoco.mj_step(self.m, pd)
            # instantaneous forces from the integrator's own forward pass
            fzw, fwl, fwr = self._inst_fz_parts(pd)
            fz_min[0] = min(fz_min[0], fzw)
            fz_min[1] = min(fz_min[1], fwl)
            fz_min[2] = min(fz_min[2], fwr)
        mujoco.mj_forward(self.m, pd)
        y = self.y_of(pd)
        return np.concatenate([y, np.where(np.isfinite(fz_min), fz_min, 0.0)])

    def _inst_fz_parts(self, pd):
        """Lean per-substep plantar Fz (contact z-force sum only)."""
        p = self.plant
        fl = fr = 0.0
        for i in range(pd.ncon):
            con = pd.contact[i]
            g1, g2 = int(con.geom1), int(con.geom2)
            floor = p.idx.floor_geom
            if g1 != floor and g2 != floor:
                continue
            lf, rf = p.idx.left_foot_geom, p.idx.right_foot_geom
            if g1 != lf and g2 != lf and g1 != rf and g2 != rf:
                continue
            foot = lf if (g1 == lf or g2 == lf) else rf
            wrench = np.zeros(6)
            mujoco.mj_contactForce(self.m, pd, i, wrench)
            frame = np.asarray(con.frame, dtype=np.float64).reshape(3, 3)
            sign = 1.0 if foot == int(con.geom[1]) else -1.0
            fz = float((frame.T @ (sign * wrench[:3]))[2])
            if foot == lf:
                fl += fz
            else:
                fr += fz
        return fl + fr, fl, fr

    def restore(self, pd, x_vec: np.ndarray) -> None:
        mujoco.mj_setState(self.m, pd, np.ascontiguousarray(x_vec),
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.m, pd)

    # ---------------- local map ----------------
    def build_G(self, x_vec: np.ndarray, u_nom: np.ndarray, n_sub: int):
        rho = self.rho_eff
        pd = self.probes[0]
        self.restore(pd, x_vec)
        y_nom = self.branch_run(pd, u_nom, n_sub)
        flags_nom = self.contact_flags(pd)
        G = np.zeros((NY, 7))
        meta = []
        for j in range(7):
            e = np.zeros(7)
            e[j] = rho
            up = np.clip(u_nom + e, -1.0, 1.0)
            um = np.clip(u_nom - e, -1.0, 1.0)
            dp = up[j] - u_nom[j]
            dm = u_nom[j] - um[j]
            if dp <= 0.0 and dm <= 0.0:
                meta.append({"j": j, "clipped_both": True,
                             "active_set_crossing": False})
                continue
            pdp, pdm = self.probes[1 + 2 * j], self.probes[2 + 2 * j]
            self.restore(pdp, x_vec)
            self.restore(pdm, x_vec)
            yp = self.branch_run(pdp, up, n_sub)
            ym = self.branch_run(pdm, um, n_sub)
            flags_p = self.contact_flags(pdp)
            flags_m = self.contact_flags(pdm)
            crossing = flags_p != flags_nom or flags_m != flags_nom
            if crossing:
                self.active_set_crossings += 1
            meta.append({"j": j, "dp": dp, "dm": dm,
                         "active_set_crossing": bool(crossing)})
            if dp > 0.0 and dm > 0.0:
                G[:, j] = (yp - ym) / (dp + dm)
            elif dp > 0.0:
                G[:, j] = (yp - y_nom) / dp
            else:
                G[:, j] = (y_nom - ym) / dm
        return y_nom, G, flags_nom, meta

    # ---------------- bounded LS solve ----------------
    def _dist_rows(self, y_ref, G):
        """One-sided contact-distance rows (predeclared): re-engage when
        separated; cap penetration at PEN_MAX; retain compression above
        PEN_MIN_RETAIN when a sub-body-weight support demand is active."""
        ks = self.k["SCALES"]
        w = self.k["LS_WEIGHTS"]
        rows, tgt = [], []
        for dist_idx in (3, 4):
            d0 = y_ref[dist_idx]
            if d0 > 0.0:
                # separated: re-engage (drive signed distance to 0)
                rows.append(w["W_DIST_ENGAGE"] * G[dist_idx] / ks["DIST_SCALE_M"])
                tgt.append(w["W_DIST_ENGAGE"] * (0.0 - d0) / ks["DIST_SCALE_M"])
            elif d0 < -self.k["PEN_MAX_M"]:
                rows.append(w["W_DIST_MAX"] * G[dist_idx] / ks["DIST_SCALE_M"])
                tgt.append(w["W_DIST_MAX"]
                           * (-self.k["PEN_MAX_M"] - d0) / ks["DIST_SCALE_M"])
            elif d0 > -self.k["PEN_MIN_RETAIN_M"]:
                rows.append(w["W_DIST_RETAIN"] * G[dist_idx] / ks["DIST_SCALE_M"])
                tgt.append(w["W_DIST_RETAIN"]
                           * (-self.k["PEN_MIN_RETAIN_M"] - d0) / ks["DIST_SCALE_M"])
        return rows, tgt

    def solve_du(self, y_nom, G, fz_des, vz_des, u_nom, rho) -> np.ndarray:
        ks = self.k["SCALES"]
        w = self.k["LS_WEIGHTS"]
        base_rows = [
            w["W_FZ_WHOLE"] * G[0] / ks["FZ_SCALE_N"],
            w["W_FZ_L"] * G[1] / ks["FZ_SCALE_N"],
            w["W_FZ_R"] * G[2] / ks["FZ_SCALE_N"],
            w["W_NVEL"] * G[5] / ks["NVEL_SCALE_MPS"],
            w["W_NVEL"] * G[6] / ks["NVEL_SCALE_MPS"],
            w["W_VZ"] * G[7] / ks["VZ_SCALE_MPS"],
        ] + [w["W_QDOT"] * G[8 + j] / ks["QDOT_SCALE_RADPS"] for j in range(7)]
        base_tgt = [
            w["W_FZ_WHOLE"] * (fz_des - y_nom[0]) / ks["FZ_SCALE_N"],
            w["W_FZ_L"] * (fz_des / 2.0 - y_nom[1]) / ks["FZ_SCALE_N"],
            w["W_FZ_R"] * (fz_des / 2.0 - y_nom[2]) / ks["FZ_SCALE_N"],
            w["W_NVEL"] * (0.0 - y_nom[5]) / ks["NVEL_SCALE_MPS"],
            w["W_NVEL"] * (0.0 - y_nom[6]) / ks["NVEL_SCALE_MPS"],
            w["W_VZ"] * (vz_des - y_nom[7]) / ks["VZ_SCALE_MPS"],
        ] + [w["W_QDOT"] * (0.0 - y_nom[8 + j]) / ks["QDOT_SCALE_RADPS"] for j in range(7)]
        rows, tgt = list(base_rows), list(base_tgt)
        for j, mi in enumerate((15, 16, 17)):
            fmin = y_nom[mi]
            ftarget = fz_des if j == 0 else fz_des / 2.0
            if fmin < ftarget - self.k["FZ_MIN_MARGIN_N"]:
                rows.append(self.k["LS_WEIGHTS"]["W_FZ_MIN"] * G[mi] / self.k["SCALES"]["FZ_SCALE_N"])
                tgt.append(self.k["LS_WEIGHTS"]["W_FZ_MIN"]
                           * (ftarget - fmin) / self.k["SCALES"]["FZ_SCALE_M" if False else "FZ_SCALE_N"])
        dr, dt_ = self._dist_rows(y_nom, G)
        rows += dr
        tgt += dt_
        du = None
        for _pass in range(3):  # predeclared 2-pass one-sided refinement
            A = np.vstack([np.asarray(rows, dtype=float), w["W_REG"] * np.eye(7)])
            b = np.concatenate([np.asarray(tgt, dtype=float), np.zeros(7)])
            lo = np.maximum(-1.0 - u_nom, -rho)
            hi = np.minimum(1.0 - u_nom, rho)
            res = lsq_linear(A, b, bounds=(lo, hi), method="bvls", tol=1e-10,
                             max_iter=200)
            du = np.asarray(res.x, dtype=float)
            y_pred = y_nom + G @ du
            dr, dt_ = self._dist_rows(y_pred, G)
            if not dr:
                break
            rows = base_rows + dr
            tgt = base_tgt + dt_
        return du

    # ---------------- one control update ----------------
    def act(self, x_vec, u_prev, fz_des, vz_des, n_sub, validation_probe):
        """Returns (info dict). All branches run on copies only.
        u_prev is the previously applied action; the trust region bounds
        |u_k - u_prev| so the layer walks the action continuously."""
        rho = self.rho_eff
        y_nom, G, flags_nom, meta = self.build_G(x_vec, u_prev, n_sub)
        du = self.solve_du(y_nom, G, fz_des, vz_des, u_prev, rho)
        u_try = np.clip(u_prev + du, -1.0, 1.0)
        accepted = False
        val_err = None
        shrink_used = 0
        flags_val = flags_nom
        y_val = y_nom
        while True:
            pdv = validation_probe
            self.restore(pdv, x_vec)
            y_val = self.branch_run(pdv, u_try, n_sub)
            flags_val = self.contact_flags(pdv)
            y_pred = y_nom + G @ (u_try - u_prev)
            val_err = np.abs(y_val - y_pred)
            tol = self.k["VALIDATION_TOL"]
            rel = self.k.get("TRUST_REL", 0.0)
            pred = np.abs(y_pred - y_nom)
            ok = (val_err[0] <= tol["FZ_WHOLE_ABS_N"] + rel * pred[0]
                  and val_err[1] <= tol["FZ_L_R_ABS_N"] + rel * pred[1]
                  and val_err[2] <= tol["FZ_L_R_ABS_N"] + rel * pred[2]
                  and val_err[3] <= tol["DIST_ABS_M"] + rel * pred[3]
                  and val_err[4] <= tol["DIST_ABS_M"] + rel * pred[4]
                  and val_err[5] <= tol["NVEL_ABS_MPS"] + rel * pred[5]
                  and val_err[6] <= tol["NVEL_ABS_MPS"] + rel * pred[6]
                  and val_err[7] <= tol["COM_VZ_ABS_MPS"] + rel * pred[7]
                  and all(val_err[8 + j] <= tol["QDOT_ABS_RADPS"] + rel * pred[8 + j]
                          for j in range(7))
                  and val_err[15] <= tol["FZ_WHOLE_ABS_N"] + rel * pred[15]
                  and val_err[16] <= tol["FZ_L_R_ABS_N"] + rel * pred[16]
                  and val_err[17] <= tol["FZ_L_R_ABS_N"] + rel * pred[17]
                  and flags_val == flags_nom)
            self.max_validation_error = max(
                self.max_validation_error, float(np.max(val_err)))
            if ok:
                accepted = True
                break
            if shrink_used >= self.k["MAX_TRUST_SHRINKS_PER_UPDATE"]:
                break
            self.rho_eff = max(self.k["RHO_MIN_U"],
                               self.rho_eff * self.k["RHO_SHRINK_FACTOR"])
            rho = self.rho_eff
            shrink_used += 1
            self.trust_shrink_events += 1
            # re-estimate the local map at the shrunken trust region (no search)
            y_nom, G, flags_nom, _ = self.build_G(x_vec, u_prev, n_sub)
            du = self.solve_du(y_nom, G, fz_des, vz_des, u_prev, rho)
            u_try = np.clip(u_prev + du, -1.0, 1.0)
        if accepted:
            self.rho_eff = min(self.k["RHO0_U"],
                               self.rho_eff * self.k["RHO_RECOVERY_FACTOR_PER_UPDATE"])
        else:
            # deterministic fallback: hold the previous applied action (exact branch)
            self.trust_fallbacks += 1
            u_try = np.asarray(u_prev, float).copy()
        return {
            "u_nom": np.asarray(u_prev, float).copy(),
            "du": np.asarray(u_try - u_prev, float),
            "u": np.asarray(u_try, float).copy(),
            "rho": float(rho),
            "shrinks": int(shrink_used),
            "VALIDATED": bool(accepted),
            "Y_NOM": np.asarray(y_nom, float).copy(),
            "Y_VAL": np.asarray(y_val, float).copy(),
            "Y_PRED": np.asarray(y_nom, float) + G @ (u_try - u_prev),
            "VAL_ERR": np.asarray(val_err, float),
            "G": G.copy(),
            "FLAGS_NOM": flags_nom,
            "FLAGS_VAL": flags_val,
            "META": meta,
            "RHO_AFTER": float(self.rho_eff),
            "FALLBACK": not accepted,
        }
