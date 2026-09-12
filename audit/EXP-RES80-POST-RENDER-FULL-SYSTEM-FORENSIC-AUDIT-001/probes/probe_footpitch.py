import numpy as np, mujoco
EVID="/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES79-ACCEPTED-TRAJECTORY-VISUAL-SMOKE-001/"
m=mujoco.MjModel.from_xml_string(open("/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml").read()); d=mujoco.MjData(m)
T=np.load(EVID+"replay_time.npy"); Q=np.load(EVID+"replay_qpos.npy")
lg=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"left_foot_box")
pitch=np.zeros(len(T))
for i in range(len(T)):
    d.qpos[:]=Q[i]; mujoco.mj_forward(m,d)
    R=d.geom_xmat[lg].reshape(3,3)
    # foot box local x axis vs world x; pitch about y
    pitch[i]=np.arctan2(-R[2,0],R[2,2])
i=np.argmax(np.abs(pitch))
print("max |foot pitch| =",abs(pitch[i]),"rad at t",T[i],"; signed max",pitch.max(),"min",pitch.min())
print("pitch at key times: E3=%.4f E6=%.4f apex=%.4f E9=%.4f"%(pitch[int(np.argmin(np.abs(T-0.38)))],pitch[int(np.argmin(np.abs(T-0.648)))],pitch[int(np.argmin(np.abs(T-0.772)))],pitch[int(np.argmin(np.abs(T-0.875)))]))
print("max |pitch| during stance (t<0.648):",np.abs(pitch[T<0.648]).max())
print("max |pitch| during landing (0.875..1.34):",np.abs(pitch[(T>=0.875)&(T<=1.34)]).max())
