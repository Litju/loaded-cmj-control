import numpy as np, mujoco
EVID="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read()); d=mujoco.MjData(m)
T=np.load(EVID+"replay_time.npy"); Q=np.load(EVID+"replay_qpos.npy"); V=np.load(EVID+"replay_qvel.npy")
mv=m.body_mass; M=float(mv.sum()); mg=M*9.81
floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"floor")
pel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"pelvis")
vadr=lambda n:int(m.jnt_dofadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)])
# compute all needed on window
i0,i1=2400,5400
vz=np.zeros(i1-i0+1); Fz=np.zeros(i1-i0+1)
for k,i in enumerate(range(i0,i1+1)):
    d.qpos[:]=Q[i]; d.qvel[:]=V[i]; mujoco.mj_forward(m,d)
    J=np.zeros((3,m.nv)); mujoco.mj_jacSubtreeCom(m,d,J,pel); vz[k]=(J@V[i])[2]
    F=np.zeros(3)
    for ci in range(d.ncon):
        c=d.contact[ci]
        if int(c.geom1)!=floor and int(c.geom2)!=floor: continue
        w=np.zeros(6); mujoco.mj_contactForce(m,d,ci,w)
        F+=c.frame.reshape(3,3).T@w[:3]
    Fz[k]=F[2]
# local windows of 40 samples: d(m*vz) over [k,k+40] vs sum F-kernel
DT=0.000125
maxres=0; worst=None
for k in range(0,len(vz)-41,40):
    dmom=M*(vz[k+40]-vz[k])
    imp=(np.sum(Fz[k:k+40])-mg*40)*DT
    r=dmom-imp
    if abs(r)>maxres: maxres=abs(r); worst=(i0+k,T[i0+k],dmom,imp,r)
print("max local (40-sample) impulse residual:",maxres,"at",worst)
# and check single-step: change in vz between consecutive samples vs Fz
print("single-step sample residuals:")
for i in [2400,2800,3200,3600,4000,4400,4800,5200]:
    k=i-i0
    dmom=M*(vz[k+1]-vz[k]); imp=(Fz[k]-mg)*DT
    print(f"t={T[i]:.5f} Fz={Fz[k]:8.1f} dmom={dmom:8.3f} imp={imp:8.3f} res={dmom-imp:8.3f}")
