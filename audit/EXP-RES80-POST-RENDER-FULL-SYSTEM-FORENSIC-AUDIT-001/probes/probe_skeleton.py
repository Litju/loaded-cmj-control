#!/usr/bin/env python3
"""Probe C: skeleton geometry at key events + control-action discontinuities."""
import json
import numpy as np
import mujoco, hashlib

EVID79="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
EVID12="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001/"
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read()); d=mujoco.MjData(m)
bid=lambda n: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)
gid=lambda n: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,n)
T=np.load(EVID79+"replay_time.npy"); Q=np.load(EVID79+"replay_qpos.npy")
EV=json.load(open(EVID79+"extraction_report.json"))["recomputed_events"]
def idx(t): return int(np.argmin(np.abs(T-t)))
names=["pelvis","torso_head_arms","external_load","left_thigh","left_shank","left_foot","right_thigh","right_shank","right_foot"]
key=[("E1_start",EV["supported_start"]["occurred_at"]),("E2_cm_onset",EV["countermovement_onset"]["occurred_at"]),
     ("E3_valid_cm",EV["valid_countermovement"]["occurred_at"]),("E4_reversal",EV["upward_reversal"]["occurred_at"]),
     ("E5_propulsion",EV["vertical_propulsion"]["occurred_at"]),("E6_takeoff",EV["bilateral_takeoff"]["occurred_at"]),
     ("E8_apex",EV["apex"]["occurred_at"]),("E9_landing",EV["descending_landing"]["occurred_at"]),
     ("E10_abs",EV["impact_absorption"]["occurred_at"]),("E11_bal",EV["balance_capture"]["occurred_at"]),
     ("E12_rec",EV["stable_recovery"]["occurred_at"])]
print("body x,z world positions (m). Facing direction: +x is model anterior (toe at +x)")
print(f"{'event':14s} {'pelvis_x':>9s} {'pelvis_z':>9s} {'knee_x':>8s} {'ankle_x':>8s} {'toe_x':>8s} {'heel_x':>8s} {'shldr_x':>8s} {'shldr_z':>8s} {'load_x':>8s} {'load_z':>8s}")
for nm,t in key:
    i=idx(t); d.qpos[:]=Q[i]; d.qvel[:]=0; mujoco.mj_forward(m,d)
    P={n:d.xpos[bid(n)].copy() for n in names}
    R=d.geom_xmat[gid("left_foot_box")].reshape(3,3); c=d.geom_xpos[gid("left_foot_box")]
    half=np.array([.15,.06,.01]); cs=np.array([(a,b,cc) for a in(-1,1) for b in(-1,1) for cc in(-1,1)],float)
    pts=c[None,:]+cs@(R*half).T
    toe=pts[np.argmax(pts[:,0])]; heel=pts[np.argmin(pts[:,0])]
    print(f"{nm:14s} {P['pelvis'][0]:9.4f} {P['pelvis'][2]:9.4f} {P['left_shank'][0]:8.4f} {P['left_foot'][0]:8.4f} {toe[0]:8.4f} {heel[0]:8.4f} {P['external_load'][0]:8.4f} {P['external_load'][2]:8.4f} {P['torso_head_arms'][0]:8.4f} {P['torso_head_arms'][2]:8.4f}")

print()
print("=== root pitch / pelvis pitch angle (root_ry, rad; + = ?) ===")
for nm,t in key:
    i=idx(t)
    print(f"{nm:14s} root_ry={Q[i,2]:+.4f} lumbar={Q[i,3]:+.4f} lhip={Q[i,4]:+.4f} lknee={Q[i,5]:+.4f} lank={Q[i,6]:+.4f}")

print()
print("=== control action discontinuities (sealed RES-12 schedule) ===")
S=np.load(EVID12+"V2.1-R001_ACTION_SCHEDULE.npz", allow_pickle=True)
U=S["action"]; ct=S["control_time"]; mode=S["mode"]; sub=S["substeps"]
dU=np.abs(np.diff(U,axis=0)).max(axis=1)
print("n_ctrl",len(U),"max|du| overall",dU.max(),"at t",ct[np.argmax(dU)])
# per control step max delta on each channel
jump=np.argmax(dU)
print("jump step",jump,"u_prev",U[jump],"u_new",U[jump+1])
# top 15 jumps
order=np.argsort(dU)[::-1][:20]
print("top 20 action jumps:")
for k in order:
    print(f"  t={ct[k]:.4f} -> {ct[k+1]:.4f}  |du|max={dU[k]:.4f}  mode_prev={mode[k]} mode_new={mode[k+1]}")
# regime change points
mm=mode.astype(str)
trans=[i for i in range(1,len(mm)) if mm[i].split('|')[0]!=mm[i-1].split('|')[0] or mm[i].split('|')[1].split('_')[0]!=mm[i-1].split('|')[1].split('_')[0]]
print("regime/phase transitions:")
for i in trans:
    print(f"  t={ct[i]:.4f}  {mm[i-1]} -> {mm[i]}  |du|max={abs(U[i]-U[i-1]).max():.4f}")
