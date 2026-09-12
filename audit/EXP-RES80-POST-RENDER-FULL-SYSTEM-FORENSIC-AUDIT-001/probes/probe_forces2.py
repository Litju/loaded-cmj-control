#!/usr/bin/env python3
"""Probe F: corrected Fz-based metrics using the sealed action schedule (ctrl-aware)."""
import json
import numpy as np, mujoco

EVID="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
EVID12="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001/"
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read()); d=mujoco.MjData(m)
T=np.load(EVID+"replay_time.npy"); Q=np.load(EVID+"replay_qpos.npy"); V=np.load(EVID+"replay_qvel.npy")
S=np.load(EVID12+"V2.1-R001_ACTION_SCHEDULE.npz",allow_pickle=True)
U=S["action"]; sub=S["substeps"]; ct=S["control_time"]
acts=np.zeros((len(T),7)); k=0
for kk in range(len(ct)):
    n=int(sub[kk])
    acts[k:k+n]=U[kk]; k+=n
floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"floor")
lg=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"left_foot_box"); rg=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"right_foot_box")
n=len(T)
fzL=np.zeros(n); fzR=np.zeros(n); fxL=np.zeros(n); fxR=np.zeros(n)
# runtime semantics: at sample i, live.ctrl = action of interval i-1
for i in range(1,n):
    d.qpos[:]=Q[i]; d.qvel[:]=V[i]; d.ctrl[:]=acts[i-1]
    mujoco.mj_forward(m,d)
    FL=np.zeros(3); FR=np.zeros(3)
    for ci in range(d.ncon):
        c=d.contact[ci]; g1,g2=int(c.geom1),int(c.geom2)
        if g1!=floor and g2!=floor: continue
        w=np.zeros(6); mujoco.mj_contactForce(m,d,ci,w)
        F=c.frame.reshape(3,3).T@w[:3]
        if g1==lg or g2==lg: FL+=F
        if g1==rg or g2==rg: FR+=F
    fzL[i]=FL[2]; fzR[i]=FR[2]; fxL[i]=FL[0]; fxR[i]=FR[0]
np.savez_compressed("/tmp/opencode/res80/forces.npz", fzL=fzL,fzR=fzR,fxL=fxL,fxR=fxR)

EV=json.load(open(EVID+"extraction_report.json"))["recomputed_events"]
def occ(nm): return float(EV[nm]["occurred_at"])
def conf(nm): return float(EV[nm]["confirmed_at"])
def idx(t): return int(np.argmin(np.abs(T-t)))
tot=fzL+fzR
both_off=(fzL<10)&(fzR<10)
i6=idx(occ("bilateral_takeoff"))
a=i6
while a>0 and both_off[a-1]: a-=1
b=i6
while b<n-1 and both_off[b+1]: b+=1
out={
"true_off_start_t":float(T[a]),"true_off_end_t":float(T[b]),"true_off_dur":float(T[b]-T[a]),
"E6_occ_vs_true_off_ms":(occ("bilateral_takeoff")-float(T[a]))*1000,
"E6_conf_vs_true_off_ms":(conf("bilateral_takeoff")-float(T[a]))*1000,
"E9_occ_vs_recontact_ms":(occ("descending_landing")-float(T[b]))*1000,
"fz_at_E6_occ_LR":[float(fzL[idx(occ('bilateral_takeoff'))]),float(fzR[idx(occ('bilateral_takeoff'))])],
"peak_total_Fz_BW":float(tot.max()/931.95),"peak_total_Fz_t":float(T[int(np.argmax(tot))]),
"peak_at_E9_100ms_BW":float(tot[idx(occ('descending_landing')):idx(occ('descending_landing'))+800].max()/931.95),
"Fz_at_E10_occ":[float(fzL[idx(occ('impact_absorption'))]),float(fzR[idx(occ('impact_absorption'))])],
"Fz_at_E11_occ":[float(fzL[idx(occ('balance_capture'))]),float(fzR[idx(occ('balance_capture'))])],
"Fz_total_at_E12":float(tot[idx(occ('stable_recovery'))]),
"min_whole_Fz_post_landing":float(tot[idx(occ('descending_landing')):].min()),
"unilateral_samples":int(np.sum((fzL<10)^(fzR<10))),
"flight_fz_max_left":float(fzL[a:b+1].max()),"flight_fz_max_right":float(fzR[a:b+1].max()),
"E6_occ_in_true_off":bool(both_off[idx(occ('bilateral_takeoff'))]),
"E6_conf_in_true_off":bool(both_off[idx(conf('bilateral_takeoff'))]),
"E9_occ_after_recontact":bool(idx(occ('descending_landing'))>b),
}
print(json.dumps(out,indent=2))
# support margin formula with ctrl-aware com? com is kinematic so unchanged; recompute margin quickly
com=np.load("/tmp/opencode/res80/traj_metrics.npz")["com"]
margin=np.zeros(n)
for i in range(n):
    d.qpos[:]=Q[i]; d.qvel[:]=0; mujoco.mj_forward(m,d)
    lc=d.geom_xpos[lg]; rc=d.geom_xpos[rg]
    xmin=min(lc[0]-.15,rc[0]-.15); xmax=max(lc[0]+.15,rc[0]+.15)
    ymin=min(lc[1]-.06,rc[1]-.06); ymax=max(lc[1]+.06,rc[1]+.06)
    margin[i]=min(com[i,0]-xmin,xmax-com[i,0],com[i,1]-ymin,ymax-com[i,1])
print("margin min overall:",margin.min(),"at t",T[int(np.argmin(margin))])
print("margin during flight (a..b):",margin[a:b+1].min(),margin[a:b+1].max())
print("margin at E10:",margin[idx(occ('impact_absorption'))],"at E11:",margin[idx(occ('balance_capture'))],"at RR:",margin[idx(1.337)])
