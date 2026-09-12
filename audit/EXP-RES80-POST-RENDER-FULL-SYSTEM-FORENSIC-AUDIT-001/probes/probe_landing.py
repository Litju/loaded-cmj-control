#!/usr/bin/env python3
"""Probe D: landing/balance/recovery forensics from sealed R001 replay."""
import json
import numpy as np, mujoco

EVID79="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read()); d=mujoco.MjData(m)
T=np.load(EVID79+"replay_time.npy"); Q=np.load(EVID79+"replay_qpos.npy"); V=np.load(EVID79+"replay_qvel.npy")
Z=np.load("/tmp/opencode/res80/traj_metrics.npz")
com=Z["com"]; cv=Z["cv"]; fzL=Z["fzL"]; fzR=Z["fzR"]; lowL=Z["lowL"]; lowR=Z["lowR"]; margin=np.load("/tmp/opencode/res80/traj_metrics.npz")["pelvis_z"]
EV=json.load(open(EVID79+"extraction_report.json"))["recomputed_events"]
def idx(t): return int(np.argmin(np.abs(T-float(t))))
bid=lambda n: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)

def H(model,data,com_):
    h=np.zeros(3); vel=np.zeros(6)
    for b in range(1,model.nbody):
        mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,b,vel,0)
        R=np.asarray(data.ximat[b]).reshape(3,3); I=R@np.diag(np.asarray(model.body_inertia[b]))@R.T
        h+=I@vel[:3]; h+=np.cross(np.asarray(data.xipos[b])-com_, model.body_mass[b]*vel[3:6])
    return h

def sample(i):
    d.qpos[:]=Q[i]; d.qvel[:]=V[i]; mujoco.mj_forward(m,d)
    return H(m,d,com[i])

trefs={
 "E9_occ":EV["descending_landing"]["occurred_at"],"E9_conf":EV["descending_landing"]["confirmed_at"],
 "E10_occ":EV["impact_absorption"]["occurred_at"],"E10_conf":EV["impact_absorption"]["confirmed_at"],
 "E11_occ":EV["balance_capture"]["occurred_at"],"E11_conf":EV["balance_capture"]["confirmed_at"],
 "RR_entry":1.2369999999999943,"RR_conf":1.336999999999939,
 "E12_occ":EV["stable_recovery"]["occurred_at"],"E12_conf":EV["stable_recovery"]["confirmed_at"],
 "handoff":14.487000000023667,"end":15.236125000027243}
print(f"{'ref':10s} {'t':>9s} {'comx':>8s} {'comz':>8s} {'vx':>8s} {'vz':>8s} {'Hy':>8s} {'rootpitch':>9s} {'trunk_tilt':>10s} {'kneeL':>7s} {'hipL':>7s} {'ankL':>7s} {'fzL':>7s} {'fzR':>7s} {'margin':>7s} {'pen':>8s}")
for name,t in trefs.items():
    i=idx(t)
    h=sample(i)
    tilt=float(np.degrees(np.arccos(np.clip(np.asarray(d.xmat[bid('torso_head_arms')]).reshape(3,3)[2,2],-1,1))))
    pen=max(0,-min(lowL[i],lowR[i]))
    print(f"{name:10s} {T[i]:9.5f} {com[i,0]:8.4f} {com[i,2]:8.4f} {cv[i,0]:8.4f} {cv[i,2]:8.4f} {h[1]:8.3f} {Q[i,2]:9.4f} {tilt:10.2f} {Q[i,5]:7.3f} {Q[i,4]:7.3f} {Q[i,6]:7.3f} {fzL[i]:7.1f} {fzR[i]:7.1f} {Z['pelvis_z'][i]:7.4f} {pen:8.5f}")

print()
print("=== E11 guard as implemented vs horizontal-speed variant (threshold 0.30) ===")
fw=slice(idx(0.99), idx(1.25))
speed=np.hypot(cv[fw,0],cv[fw,2])
vz=np.abs(cv[fw,2])
t=T[fw]
first_vz=np.argmax(vz<0.30); first_sp=np.argmax(speed<0.30)
print("first sample |vz|<0.30 at t=",t[first_vz],"| horizontal speed<0.30 at t=",t[first_sp])
print("speed at E11 occ:",float(np.hypot(cv[idx(EV['balance_capture']['occurred_at']),0],cv[idx(EV['balance_capture']['occurred_at']),2])))
print("vz at E11 occ:",float(abs(cv[idx(EV['balance_capture']['occurred_at']),2])))

print()
print("=== backward motion around takeoff (pelvis x / com x / vx) ===")
for t in [0.40,0.45,0.4875,0.55,0.60,0.63,0.64675,0.658,0.70,0.7719,0.80,0.875]:
    i=idx(t)
    d.qpos[:]=Q[i]; d.qvel[:]=0; mujoco.mj_forward(m,d)
    print(f"t={T[i]:.5f} pelvis_x={d.xpos[bid('pelvis')][0]:+.4f} com_x={com[i,0]:+.4f} com_vx={cv[i,0]:+.4f} root_pitch={Q[i,2]:+.4f}")

print()
print("=== forward lunge after landing ===")
for t in [0.875,0.95,0.9935,1.10,1.237,1.337,1.5,2.0,2.117,3.0,6.0,10.0,14.4,15.2]:
    i=idx(t)
    print(f"t={T[i]:.5f} com_x={com[i,0]:+.4f} com_vx={cv[i,0]:+.4f} com_vz={cv[i,2]:+.4f} root_pitch={Q[i,2]:+.4f} kneeL={Q[i,5]:+.4f} hipL={Q[i,4]:+.4f} ankL={Q[i,6]:+.4f} fzL={fzL[i]:.1f} fzR={fzR[i]:.1f}")

# peak root pitch rate in takeoff window
pr = np.gradient(Q[:,2], T)
i0,i1=idx(0.45),idx(0.70)
print()
print("max root pitch rate takeoff window:", float(pr[i0:i1].max()), "at t", float(T[i0+int(np.argmax(pr[i0:i1]))]))
print("min root pitch rate takeoff window:", float(pr[i0:i1].min()), "at t", float(T[i0+int(np.argmin(pr[i0:i1]))]))
i0,i1=idx(0.85),idx(1.20)
print("max root pitch rate landing window:", float(pr[i0:i1].max()), "at t", float(T[i0+int(np.argmax(pr[i0:i1]))]))
print("min root pitch rate landing window:", float(pr[i0:i1].min()), "at t", float(T[i0+int(np.argmin(pr[i0:i1]))]))
print()
print("max |com_vx| overall:", float(np.abs(cv[:,0]).max()), "at t", float(T[int(np.argmax(np.abs(cv[:,0])))]))
print("backward peak vx:", float(cv[:,0].min()), "at t", float(T[int(np.argmin(cv[:,0]))]))
print("com_x at t=0:", float(com[0,0]), "com_x max:", float(com[:,0].max()), "at t", float(T[int(np.argmax(com[:,0]))]))
