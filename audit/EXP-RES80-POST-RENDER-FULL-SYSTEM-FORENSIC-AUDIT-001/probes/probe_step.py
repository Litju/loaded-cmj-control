import numpy as np, mujoco
EVID="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
EVID12="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES12-CANONICAL-12OF12-QUALIFICATION-001/"
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read()); d=mujoco.MjData(m)
T=np.load(EVID+"replay_time.npy"); Q=np.load(EVID+"replay_qpos.npy"); V=np.load(EVID+"replay_qvel.npy")
S=np.load(EVID12+"V2.1-R001_ACTION_SCHEDULE.npz",allow_pickle=True)
U=S["action"]; ct=S["control_time"]; sub=S["substeps"]
# build per-physics action array
acts=np.zeros((len(T),7))
k=0
for kk in range(len(ct)):
    for s in range(int(sub[kk])):
        if k<len(T): acts[k]=U[kk]; k+=1
print("built acts",k,"of",len(T))
for t in [0.45,0.55]:
    i=int(np.argmin(np.abs(T-t)))
    # state at T[i] is Q[i]; recorded action for interval i->i+1 is acts[i]
    d.qpos[:]=Q[i]; d.qvel[:]=V[i]; d.ctrl[:]=acts[i]
    mujoco.mj_forward(m,d)
    F=np.zeros(3)
    floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"floor")
    for ci in range(d.ncon):
        c=d.contact[ci]
        if int(c.geom1)!=floor and int(c.geom2)!=floor: continue
        w=np.zeros(6); mujoco.mj_contactForce(m,d,ci,w)
        F+=c.frame.reshape(3,3).T@w[:3]
    qacc_fwd=d.qacc.copy()
    # snapshot and step
    d.qpos[:]=Q[i]; d.qvel[:]=V[i]; d.ctrl[:]=acts[i]
    mujoco.mj_step(m,d)
    dvz=V[i+1][1]-V[i][1]
    # qvimv: implied COM acceleration from qacc (jacobian projection)
    pel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"pelvis")
    J=np.zeros((3,m.nv)); d.qpos[:]=Q[i]; d.qvel[:]=V[i]; mujoco.mj_forward(m,d)
    mujoco.mj_jacSubtreeCom(m,d,J,pel)
    acom_qacc=float((J@qacc_fwd)[2])
    # Jdot term via finite difference of J along actual next state
    d.qpos[:]=Q[i+1]; d.qvel[:]=V[i+1]; mujoco.mj_forward(m,d)
    J2=np.zeros((3,m.nv)); mujoco.mj_jacSubtreeCom(m,d,J2,pel)
    Jdotv=float(((J2-J)@np.asarray(V[i])/0.000125)[2])
    print(f"t={T[i]:.5f} Fz={F[2]:8.2f} rootqacc={qacc_fwd[1]:9.2f} acom_qacc={acom_qacc:9.2f} Jdotv={Jdotv:9.2f} sum={acom_qacc+Jdotv:9.2f} actual_dvz/dt={dvz/0.000125:9.2f} (Fz-mg)/M={((F[2]-95*9.81)/95):9.2f}")
