#!/usr/bin/env python3
"""V2.1 frozen centroidal balance-capture controller (RES-73, single architecture).

MISSION=RES10_SYNC_BALANCE_CAPTURE_E10_TO_E11_001
EXPERIMENT_ID=EXP-RES10-SYNC-BALANCE-CAPTURE-E10-E11-001

Outer: finite-horizon centroidal planner, state [COM x, px, COM z, pz, Hy],
  desired net wrench via feasible CoP projection (never independent infeasible Fx/Hdot).
Inner: sealed full-7DOF exact-forward RES-52 architecture + RES-54 COM vel
  + RES-55 foot vel + RES-57 support semantics. Vertical P2 > horizontal P4.
No ladder, no random/RL/NOMAD, no gain search, no hidden pose-PD fallback.
Frozen constants below (predeclared Phase B/C, no tuning during run).
Sealed lineage: RES-52 SoftContactPolicy, RES-54 jacSubtreeCom, RES-55 J_point,
  RES-57 continuity, RES-58 vertical law (outer Fz), RES-72 E10 authority.
"""
import sys
from pathlib import Path
REPO=Path("/home/litju/Projects/loaded-cmj-control")
sys.path.insert(0,str(REPO/"src")); sys.path.insert(0,str(REPO/"tools"/"res52"))
import numpy as np, mujoco
from loaded_cmj.v2.plant import V2Plant
from loaded_cmj.v2.measurement import SynchronizedPhysicsSample, create_measurement_data
from loaded_cmj.v2 import terminal_capture as TC
import core52 as C52
from tools.evid_trace_v2 import centroidal_H_world
from scipy.optimize import lsq_linear

# Frozen constants (predeclared)
MASS=95.0; WEIGHT=95*9.81; BW=WEIGHT
CONTROL_DT=0.005; SUB=40; RHO0=0.25; RHO_MIN=0.0025; SHRINK=0.5; MAX_SHRINK=3
T_BAL=0.27
# Outer limits (from Phase B coupled set, conservative)
FX_BRAKE_MIN=-120.0; FX_BRAKE_MAX=0.0  # braking only (no forward demand)
HDOT_MIN=-60.0; HDOT_MAX=0.0  # reduction only
# CoP support (world x): foot centers 0.1066, hx 0.15 => [-0.043,0.257]; require interior margin 0.02 => [-0.023,0.237]
COP_X_MIN=-0.023; COP_X_MAX=0.237
# Inner LS weights/scales (vertical higher priority P2 > P4)
SCALES={"FZ":100.0,"FX":50.0,"HDOT":20.0,"VZ":0.02,"DIST":0.0005,"NVEL":0.05,"QDOT":0.3}
WEIGHTS={"W_FZ_WHOLE":1.5,"W_FZ_LR":0.5,"W_FX":0.8,"W_HDOT":0.8,"W_VZ":1.0,"W_NVEL":0.5,"W_QDOT":0.2,"W_REG":0.001,"W_DIST_ENGAGE":4.0,"W_DIST_MAX":4.0,"W_DIST_RETAIN":2.0}
TOL={"FZ_WHOLE":20.0,"FZ_LR":15.0,"FX":20.0,"HDOT":15.0,"VZ":0.01,"DIST":0.0005,"NVEL":0.05,"QDOT":0.05}
TRUST_REL=0.1

class BalanceController:
    def __init__(self,plant,probes,validation_probe,gap_hh):
        self.plant=plant; self.m=plant.model; self.probes=probes; self.vprobe=validation_probe; self.gap_hh=gap_hh
        self.rho_eff=RHO0
        self.u_prev=None
        self.t0=None
        self.validation_fails=0; self.shrinks=0; self.fallbacks=0; self.max_err=0.0
        self.active_cross=0

    def y_of(self,pd):
        p=self.plant
        sm=p.foot_contact_summary(pd)
        # per-foot dist/nvel
        st={}
        for side,fg,fb in (("L",p.idx.left_foot_geom,p.idx.left_foot_body),("R",p.idx.right_foot_geom,p.idx.right_foot_body)):
            rows=[i for i in range(pd.ncon) if (int(pd.contact[i].geom1)==p.idx.floor_geom or int(pd.contact[i].geom2)==p.idx.floor_geom) and (int(pd.contact[i].geom1)==fg or int(pd.contact[i].geom2)==fg)]
            if rows:
                i=rows[int(np.argmin([float(pd.contact[j].dist) for j in rows]))]
                dist=float(pd.contact[i].dist); nvel=float(pd.efc_vel[pd.contact[i].efc_address])
            else:
                dist=float(pd.geom_xpos[fg][2])-self.gap_hh
                p_low=np.asarray(pd.geom_xpos[fg],float).copy(); p_low[2]-=self.gap_hh
                Jp=np.zeros((3,self.m.nv)); Jr=np.zeros((3,self.m.nv))
                mujoco.mj_jac(self.m,pd,Jp,Jr,np.asarray(p_low,float),int(fb))
                nvel=float((Jp@np.asarray(pd.qvel,float))[2])
            st[side]=(dist,nvel)
        cv=p.center_of_mass_velocity(pd)
        # Fx/Hdot via momentum deltas? For y_of (instantaneous end-state), we need mean over branch, not instantaneous.
        # Instead y_of returns end-state quantities; mean Fx/Hdot computed in branch_run via deltas.
        # For G building, we use branch_run that returns mean Fx/Hdot/Fz + end dist/nvel/vz/qdot
        # This helper is for end-state only; branch_run wraps it.
        return sm,st,cv

    def branch_run(self,pd,u,x_vec,n_sub,com0,cv0,H0):
        self.plant.apply_action(pd,u)
        Fz_sum=0; Fx_sum=0; minFz=np.inf; maxpen=0
        for _ in range(n_sub):
            mujoco.mj_step(self.m,pd)
            sh=self.probes[0]  # reuse? No, need independent? For mean, use live summary via shadow? Use direct live summary for speed (forces okay)
            # Use shadow for accuracy every step? Use live for speed (Fz from live after step is okay, qacc stale but Fz is contact, not qacc)
            # To be exact, use shadow: copy live to shadow and forward
            # For performance, use live summary (contact forces from live are exact post-step, no lag for forces? Forces depend on qpos/qvel, not qacc, so live okay)
            # Use live to save 40 forwards per branch (15 branches *40 =600 forwards saved)
            sm_live=self.plant.foot_contact_summary(pd)
            Fx_sum+=float(np.asarray(sm_live["whole_force"],float)[0]); Fz_sum+=float(sm_live["whole_Fz"])
            minFz=min(minFz,float(sm_live["whole_Fz"])); maxpen=max(maxpen,float(sm_live["max_penetration"]))
        mujoco.mj_forward(self.m,pd)
        # end synchronized measurement (need shadow for COM/Hy/qdot/dist/nvel)
        # Use pd itself as synchronized? pd after steps has live qacc/contact; need shadow forward for derived? For end-state, pd after mj_forward is synchronized (qacc/contact at T)
        # So use pd directly for end-state (no extra shadow needed, since we just forwarded)
        # But to match SynchronizedPhysicsSample semantics (shadow), we should create shadow from pd integration state and forward.
        # Simpler: use pd's current qpos/qvel with mj_forward already done, compute COM/Hy via same functions (which use xipos/xmat, not qacc, so live okay)
        com1=self.plant.center_of_mass(pd); cv1=self.plant.center_of_mass_velocity(pd)
        H1=centroidal_H_world(self.m,pd,com1)
        sm1=self.plant.foot_contact_summary(pd)
        # dist/nvel end
        st={}
        for side,fg,fb in (("L",self.plant.idx.left_foot_geom,self.plant.idx.left_foot_body),("R",self.plant.idx.right_foot_geom,self.plant.idx.right_foot_body)):
            rows=[i for i in range(pd.ncon) if (int(pd.contact[i].geom1)==self.plant.idx.floor_geom or int(pd.contact[i].geom2)==self.plant.idx.floor_geom) and (int(pd.contact[i].geom1)==fg or int(pd.contact[i].geom2)==fg)]
            if rows:
                i=rows[int(np.argmin([float(pd.contact[j].dist) for j in rows]))]
                dist=float(pd.contact[i].dist); nvel=float(pd.efc_vel[pd.contact[i].efc_address])
            else:
                dist=float(pd.geom_xpos[fg][2])-self.gap_hh
                p_low=np.asarray(pd.geom_xpos[fg],float).copy(); p_low[2]-=self.gap_hh
                Jp=np.zeros((3,self.m.nv)); Jr=np.zeros((3,self.m.nv))
                mujoco.mj_jac(self.m,pd,Jp,Jr,np.asarray(p_low,float),int(fb))
                nvel=float((Jp@np.asarray(pd.qvel,float))[2])
            st[side]=(dist,nvel)
        dpx=MASS*(cv1[0]-cv0[0]); dHy=float(H1[1]-H0[1])
        qd=np.asarray(pd.qvel,float)[[self.plant.idx.vadr[n] for n in ("lumbar","left_hip","right_hip","left_knee","right_knee","left_ankle","right_ankle")]]
        # vector for validation (Fx,Hdot,Fz,distL,distR,nvelL,nvelR,vz,qdot7)
        y=np.array([dpx/CONTROL_DT,float(dHy/CONTROL_DT),Fz_sum/n_sub,st["L"][0],st["R"][0],st["L"][1],st["R"][1],float(cv1[2]),qd[0],qd[1],qd[2],qd[3],qd[4],qd[5],qd[6]],dtype=float)
        flags=(bool(sm1["active_left"]),bool(sm1["active_right"]))
        return y,flags,{"Fz_end":float(sm1["whole_Fz"]),"FzL":float(sm1["left_Fz"]),"FzR":float(sm1["right_Fz"]),"maxpen":float(maxpen),"minFz":float(minFz),"com1":com1,"cv1":cv1,"H1":H1,"distL":st["L"][0],"distR":st["R"][0]}

    def restore(self,pd,x_vec):
        mujoco.mj_setState(self.m,pd,np.ascontiguousarray(x_vec),mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(self.m,pd)

    def outer_desired(self,com,cv,H,t):
        # Outer centroidal planner with feasible CoP projection
        px=MASS*float(cv[0]); pz=MASS*float(cv[2]); Hy=float(H[1])
        cx=float(com[0]); cz=float(com[2])
        elapsed=float(t-self.t0) if self.t0 is not None else 0.0
        T_rem=max(0.05,T_BAL-elapsed)
        # raw required means to zero in T_rem
        Fx_raw=-px/T_rem if T_rem>1e-9 else 0.0
        Hd_raw=-Hy/T_rem if T_rem>1e-9 else 0.0
        # clamp to braking/reduction only (no forward/increase demand) + feasible bounds
        Fx_clamped=float(np.clip(Fx_raw,FX_BRAKE_MIN,FX_BRAKE_MAX))
        Hd_clamped=float(np.clip(Hd_raw,HDOT_MIN,HDOT_MAX))
        # vertical high priority via RES-58 law
        fz_des,vz_des=TC.capture_command(float(cv[2]))
        # CoP projection: offset = (-z*Fx - Hdot)/Fz must be inside [COP_MIN-COMx, COP_MAX-COMx]
        # Compute offset for clamped pair
        Fz_for_proj=float(fz_des)
        offset=(-cz*Fx_clamped - Hd_clamped)/Fz_for_proj if Fz_for_proj>1e-9 else 0.0
        x_cop=cx+offset
        # clamp CoP to interior
        x_cop_clamped=float(np.clip(x_cop,COP_X_MIN,COP_X_MAX))
        # If clamped, project (Fx,Hdot) onto feasible line Hdot = -z*Fx - (x_cop_clamped-cx)*Fz
        # Choose to preserve Hdot (Hy priority? Or px? Spec P4 jointly reduce; preserve both via closest projection)
        # Closest projection: minimize (Fx-Fx_clamped)^2/50^2 + (Hdot-Hd_clamped)^2/20^2 subject to line
        # Solve analytically: line a*Fx + b*Hdot + c =0 with a=z, b=1, c=(x_cop_clamped-cx)*Fz
        # Weighted projection
        if abs(x_cop-x_cop_clamped)>1e-9:
            # need projection
            a=cz; b=1.0; c=(x_cop_clamped-cx)*Fz_for_proj
            # residual r = a*Fx_clamped + b*Hd_clamped + c should be 0 for feasible; if not, project
            # Weighted: wFx=1/50^2, wHd=1/20^2
            wFx=1/(50**2); wHd=1/(20**2)
            # Lagrange: Fx = Fx0 - lam*a/wFx? Actually minimize wFx*(dFx)^2 + wHd*(dHd)^2 s.t. a*dFx + b*dHd + r =0
            r=a*Fx_clamped + b*Hd_clamped + c
            denom=(a*a/wFx + b*b/wHd) if (wFx>0 and wHd>0) else 1.0
            lam=r/denom if denom!=0 else 0.0
            Fx_proj=Fx_clamped - lam*a/wFx
            Hd_proj=Hd_clamped - lam*b/wHd
            # re-clamp to bounds (may go outside due to projection; clamp again and recompute other via line to preserve feasibility? For simplicity, clamp and recompute Hdot from Fx via line to keep CoP feasible)
            Fx_proj=float(np.clip(Fx_proj,FX_BRAKE_MIN,FX_BRAKE_MAX))
            # recompute Hdot from line to keep CoP exactly clamped (preserve Fx priority? Or preserve Hdot? Spec says jointly, so keep projection as is, but ensure Hdot within bounds)
            # Use line to compute Hdot from Fx_proj (keeps CoP feasible exactly)
            Hd_from_line=-cz*Fx_proj - (x_cop_clamped-cx)*Fz_for_proj
            # Blend? Use Hd_from_line clamped
            Hd_proj2=float(np.clip(Hd_from_line,HDOT_MIN,HDOT_MAX))
            # If Hd clamped, recompute Fx from line to keep feasible? Iterate once
            if abs(Hd_proj2-Hd_from_line)>1e-9:
                Fx_from_line=(-Hd_proj2 - (x_cop_clamped-cx)*Fz_for_proj)/cz if abs(cz)>1e-9 else Fx_proj
                Fx_proj=float(np.clip(Fx_from_line,FX_BRAKE_MIN,FX_BRAKE_MAX))
                Hd_proj=Hd_proj2
            else:
                Hd_proj=float(Hd_proj2)
            Fx_des=Fx_proj; Hd_des=Hd_proj; x_cop_final=x_cop_clamped
        else:
            Fx_des=Fx_clamped; Hd_des=Hd_clamped; x_cop_final=x_cop
        return Fx_des,Hd_des,float(fz_des),float(vz_des),{"px":float(px),"pz":float(pz),"Hy":float(Hy),"T_rem":float(T_rem),"Fx_raw":float(Fx_raw),"Hd_raw":float(Hd_raw),"x_cop":float(x_cop_final),"offset":float(x_cop_final-cx)}

    def step(self,x_vec,u_prev,t,com,cv,H):
        if self.t0 is None:
            self.t0=float(t)
        Fx_des,Hd_des,Fz_des,Vz_des,oinfo=self.outer_desired(com,cv,H,t)
        # Build G via exact branches (nominal = u_prev)
        # Need com0/cv0/H0 for deltas (from current state)
        # Get nominal branch
        pd0=self.probes[0]
        self.restore(pd0,x_vec)
        # com0/cv0/H0 from current state (use live pd0 after restore+forward)
        com0=self.plant.center_of_mass(pd0); cv0=self.plant.center_of_mass_velocity(pd0); H0=centroidal_H_world(self.m,pd0,com0)
        y_nom,flags_nom,_=self.branch_run(pd0,np.asarray(u_prev,float),x_vec,40,com0,cv0,H0)
        # y layout: [Fx,Hdot,Fz_mean,distL,distR,nvelL,nvelR,vz,qd0..6] (15)
        G=np.zeros((15,7))
        for j in range(7):
            e=np.zeros(7); e[j]=self.rho_eff
            up=np.clip(np.asarray(u_prev,float)+e,-1,1); um=np.clip(np.asarray(u_prev,float)-e,-1,1)
            dp=float(up[j]-u_prev[j]); dm=float(u_prev[j]-um[j])
            if dp<=0 and dm<=0: continue
            pdp=self.probes[1+2*j]; pdm=self.probes[2+2*j]
            self.restore(pdp,x_vec); self.restore(pdm,x_vec)
            yp,fp,_=self.branch_run(pdp,up,x_vec,40,com0,cv0,H0)
            ym,fm,_=self.branch_run(pdm,um,x_vec,40,com0,cv0,H0)
            cross=(fp!=flags_nom or fm!=flags_nom)
            if cross: self.active_cross+=1
            # Never globalize across switching: if crossing, use one-sided within mode if possible, else zero column
            if cross:
                if fp==flags_nom and dp>0:
                    G[:,j]=(yp-y_nom)/dp
                elif fm==flags_nom and dm>0:
                    G[:,j]=(y_nom-ym)/dm
                else:
                    G[:,j]=0
            else:
                if dp>0 and dm>0:
                    G[:,j]=(yp-ym)/(dp+dm)
                elif dp>0:
                    G[:,j]=(yp-y_nom)/dp
                else:
                    G[:,j]=(y_nom-ym)/dm
        # Solve LS: rows for Fz,Fx,Hdot,VZ,NVEL,QDOT,DIST (one-sided dist)
        # Build A/b with scales/weights
        # Base rows: Fx,Hdot,Fz_whole,Fz_L? For simplicity track whole Fx/Hdot/Fz + vz + nvel + qdot (as in RES-52) + dist one-sided
        # y_nom indices: 0 Fx,1 Hdot,2 Fz_mean,3 distL,4 distR,5 nvelL,6 nvelR,7 vz,8-14 qdot
        # Desired: Fx_des,Hd_des,Fz_des, dist retain/engage, nvel 0, vz_des, qdot 0
        # For Fz_L/R, we don't have separate G? Use whole only + symmetric split? For balance, L/R distribution matters for CoP! Need per-foot Fz G? Our y doesn't include Fz_L/R means. For CoP control via Fx/Hdot, whole is enough (since Hdot encodes CoP via wrench relation). Keep whole.
        rows=[]; tgt=[]
        # Fx
        rows.append(WEIGHTS["W_FX"]*G[0]/SCALES["FX"]); tgt.append(WEIGHTS["W_FX"]*(Fx_des-y_nom[0])/SCALES["FX"])
        # Hdot
        rows.append(WEIGHTS["W_HDOT"]*G[1]/SCALES["HDOT"]); tgt.append(WEIGHTS["W_HDOT"]*(Hd_des-y_nom[1])/SCALES["HDOT"])
        # Fz whole (high priority)
        rows.append(WEIGHTS["W_FZ_WHOLE"]*G[2]/SCALES["FZ"]); tgt.append(WEIGHTS["W_FZ_WHOLE"]*(Fz_des-y_nom[2])/SCALES["FZ"])
        # NVEL
        rows.append(WEIGHTS["W_NVEL"]*G[5]/SCALES["NVEL"]); tgt.append(WEIGHTS["W_NVEL"]*(0.0-y_nom[5])/SCALES["NVEL"])
        rows.append(WEIGHTS["W_NVEL"]*G[6]/SCALES["NVEL"]); tgt.append(WEIGHTS["W_NVEL"]*(0.0-y_nom[6])/SCALES["NVEL"])
        # VZ (high priority)
        rows.append(WEIGHTS["W_VZ"]*G[7]/SCALES["VZ"]); tgt.append(WEIGHTS["W_VZ"]*(Vz_des-y_nom[7])/SCALES["VZ"])
        # QDOT
        for k in range(7):
            rows.append(WEIGHTS["W_QDOT"]*G[8+k]/SCALES["QDOT"]); tgt.append(WEIGHTS["W_QDOT"]*(0.0-y_nom[8+k])/SCALES["QDOT"])
        # DIST one-sided (retain/engage/max)
        # Use y_nom dist for logic (like RES-52)
        for idx,pen_max,pen_min in [(3,0.009,0.0005),(4,0.009,0.0005)]:
            d0=y_nom[idx]
            if d0>0:
                rows.append(WEIGHTS["W_DIST_ENGAGE"]*G[idx]/SCALES["DIST"]); tgt.append(WEIGHTS["W_DIST_ENGAGE"]*(0.0-d0)/SCALES["DIST"])
            elif d0<-pen_max:
                rows.append(WEIGHTS["W_DIST_MAX"]*G[idx]/SCALES["DIST"]); tgt.append(WEIGHTS["W_DIST_MAX"]*(-pen_max-d0)/SCALES["DIST"])
            elif d0>-pen_min:
                rows.append(WEIGHTS["W_DIST_RETAIN"]*G[idx]/SCALES["DIST"]); tgt.append(WEIGHTS["W_DIST_RETAIN"]*(-pen_min-d0)/SCALES["DIST"])
        A=np.vstack([np.asarray(rows,float),WEIGHTS["W_REG"]*np.eye(7)])
        b=np.concatenate([np.asarray(tgt,float),np.zeros(7)])
        lo=np.maximum(-1.0-np.asarray(u_prev,float),-self.rho_eff)
        hi=np.minimum(1.0-np.asarray(u_prev,float),self.rho_eff)
        res=lsq_linear(A,b,bounds=(lo,hi),method="bvls",tol=1e-10,max_iter=200)
        Du=np.asarray(res.x,float)
        u_try=np.clip(np.asarray(u_prev,float)+Du,-1,1)
        # Validate with exact branch
        self.restore(self.vprobe,x_vec)
        y_val,flags_val,_=self.branch_run(self.vprobe,u_try,x_vec,40,com0,cv0,H0)
        y_pred=y_nom+G@Du
        err=np.abs(y_val-y_pred)
        pred=np.abs(y_pred-y_nom)
        ok=(err[0]<=TOL["FX"]+TRUST_REL*pred[0] and err[1]<=TOL["HDOT"]+TRUST_REL*pred[1] and err[2]<=TOL["FZ_WHOLE"]+TRUST_REL*pred[2] and err[3]<=TOL["DIST"]+TRUST_REL*pred[3] and err[4]<=TOL["DIST"]+TRUST_REL*pred[4] and err[5]<=TOL["NVEL"]+TRUST_REL*pred[5] and err[6]<=TOL["NVEL"]+TRUST_REL*pred[6] and err[7]<=TOL["VZ"]+TRUST_REL*pred[7] and all(err[8+k]<=TOL["QDOT"]+TRUST_REL*pred[8+k] for k in range(7)) and flags_val==flags_nom)
        self.max_err=max(self.max_err,float(np.max(err)))
        if ok:
            # recover rho slowly
            self.rho_eff=min(RHO0,self.rho_eff*1.189207115002721)
            return u_try,{"VALIDATED":True,"Fx_des":Fx_des,"Hd_des":Hd_des,"Fz_des":Fz_des,"oinfo":oinfo,"G":G,"y_nom":y_nom,"y_val":y_val,"shrinks":0}
        else:
            # shrink and retry (up to MAX_SHRINK)
            shrinks=0
            rho=self.rho_eff
            while shrinks<MAX_SHRINK:
                rho=max(RHO_MIN,rho*SHRINK); shrinks+=1; self.shrinks+=1
                self.rho_eff=rho
                # rebuild G at smaller rho? For determinism, rebuild (no search)
                # Rebuild quickly with new rho
                G2=np.zeros((15,7))
                for j in range(7):
                    e2=np.zeros(7); e2[j]=rho
                    up2=np.clip(np.asarray(u_prev,float)+e2,-1,1); um2=np.clip(np.asarray(u_prev,float)-e2,-1,1)
                    dp2=float(up2[j]-u_prev[j]); dm2=float(u_prev[j]-um2[j])
                    if dp2<=0 and dm2<=0: continue
                    pdp2=self.probes[1+2*j]; pdm2=self.probes[2+2*j]
                    self.restore(pdp2,x_vec); self.restore(pdm2,x_vec)
                    yp2,fp2,_=self.branch_run(pdp2,up2,x_vec,40,com0,cv0,H0)
                    ym2,fm2,_=self.branch_run(pdm2,um2,x_vec,40,com0,cv0,H0)
                    if fp2!=flags_nom or fm2!=flags_nom:
                        if fp2==flags_nom and dp2>0: G2[:,j]=(yp2-y_nom)/dp2
                        elif fm2==flags_nom and dm2>0: G2[:,j]=(y_nom-ym2)/dm2
                        else: G2[:,j]=0
                    else:
                        if dp2>0 and dm2>0: G2[:,j]=(yp2-ym2)/(dp2+dm2)
                        elif dp2>0: G2[:,j]=(yp2-y_nom)/dp2
                        else: G2[:,j]=(y_nom-ym2)/dm2
                # resolve
                rows2=[]; tgt2=[]
                rows2.append(WEIGHTS["W_FX"]*G2[0]/SCALES["FX"]); tgt2.append(WEIGHTS["W_FX"]*(Fx_des-y_nom[0])/SCALES["FX"])
                rows2.append(WEIGHTS["W_HDOT"]*G2[1]/SCALES["HDOT"]); tgt2.append(WEIGHTS["W_HDOT"]*(Hd_des-y_nom[1])/SCALES["HDOT"])
                rows2.append(WEIGHTS["W_FZ_WHOLE"]*G2[2]/SCALES["FZ"]); tgt2.append(WEIGHTS["W_FZ_WHOLE"]*(Fz_des-y_nom[2])/SCALES["FZ"])
                rows2.append(WEIGHTS["W_NVEL"]*G2[5]/SCALES["NVEL"]); tgt2.append(WEIGHTS["W_NVEL"]*(0.0-y_nom[5])/SCALES["NVEL"])
                rows2.append(WEIGHTS["W_NVEL"]*G2[6]/SCALES["NVEL"]); tgt2.append(WEIGHTS["W_NVEL"]*(0.0-y_nom[6])/SCALES["NVEL"])
                rows2.append(WEIGHTS["W_VZ"]*G2[7]/SCALES["VZ"]); tgt2.append(WEIGHTS["W_VZ"]*(Vz_des-y_nom[7])/SCALES["VZ"])
                for k in range(7):
                    rows2.append(WEIGHTS["W_QDOT"]*G2[8+k]/SCALES["QDOT"]); tgt2.append(WEIGHTS["W_QDOT"]*(0.0-y_nom[8+k])/SCALES["QDOT"])
                for idx2,pen_max2,pen_min2 in [(3,0.009,0.0005),(4,0.009,0.0005)]:
                    d02=y_nom[idx2]
                    if d02>0: rows2.append(WEIGHTS["W_DIST_ENGAGE"]*G2[idx2]/SCALES["DIST"]); tgt2.append(WEIGHTS["W_DIST_ENGAGE"]*(0.0-d02)/SCALES["DIST"])
                    elif d02<-pen_max2: rows2.append(WEIGHTS["W_DIST_MAX"]*G2[idx2]/SCALES["DIST"]); tgt2.append(WEIGHTS["W_DIST_MAX"]*(-pen_max2-d02)/SCALES["DIST"])
                    elif d02>-pen_min2: rows2.append(WEIGHTS["W_DIST_RETAIN"]*G2[idx2]/SCALES["DIST"]); tgt2.append(WEIGHTS["W_DIST_RETAIN"]*(-pen_min2-d02)/SCALES["DIST"])
                A2=np.vstack([np.asarray(rows2,float),WEIGHTS["W_REG"]*np.eye(7)])
                b2=np.concatenate([np.asarray(tgt2,float),np.zeros(7)])
                lo2=np.maximum(-1.0-np.asarray(u_prev,float),-rho)
                hi2=np.minimum(1.0-np.asarray(u_prev,float),rho)
                res2=lsq_linear(A2,b2,bounds=(lo2,hi2),method="bvls",tol=1e-10,max_iter=200)
                Du2=np.asarray(res2.x,float); u_try2=np.clip(np.asarray(u_prev,float)+Du2,-1,1)
                self.restore(self.vprobe,x_vec)
                y_val2,flags_val2,_=self.branch_run(self.vprobe,u_try2,x_vec,40,com0,cv0,H0)
                y_pred2=y_nom+G2@Du2
                err2=np.abs(y_val2-y_pred2); pred2=np.abs(y_pred2-y_nom)
                ok2=(err2[0]<=TOL["FX"]+TRUST_REL*pred2[0] and err2[1]<=TOL["HDOT"]+TRUST_REL*pred2[1] and err2[2]<=TOL["FZ_WHOLE"]+TRUST_REL*pred2[2] and err2[3]<=TOL["DIST"]+TRUST_REL*pred2[3] and err2[4]<=TOL["DIST"]+TRUST_REL*pred2[4] and err2[5]<=TOL["NVEL"]+TRUST_REL*pred2[5] and err2[6]<=TOL["NVEL"]+TRUST_REL*pred2[6] and err2[7]<=TOL["VZ"]+TRUST_REL*pred2[7] and all(err2[8+k]<=TOL["QDOT"]+TRUST_REL*pred2[8+k] for k in range(7)) and flags_val2==flags_nom)
                self.max_err=max(self.max_err,float(np.max(err2)))
                if ok2:
                    self.rho_eff=min(RHO0,rho*1.189207115002721)
                    return u_try2,{"VALIDATED":True,"Fx_des":Fx_des,"Hd_des":Hd_des,"Fz_des":Fz_des,"oinfo":oinfo,"G":G2,"y_nom":y_nom,"y_val":y_val2,"shrinks":shrinks}
            self.fallbacks+=1
            self.validation_fails+=1
            return np.asarray(u_prev,float).copy(),{"VALIDATED":False,"Fx_des":Fx_des,"Hd_des":Hd_des,"Fz_des":Fz_des,"oinfo":oinfo,"G":G,"y_nom":y_nom,"y_val":y_nom,"shrinks":shrinks}
