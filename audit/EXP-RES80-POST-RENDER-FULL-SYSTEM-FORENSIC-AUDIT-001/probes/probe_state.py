import numpy as np, mujoco
m=mujoco.MjModel.from_xml_string(open("/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml").read())
flags={
 "TIME":mujoco.mjtState.mjSTATE_TIME,"QPOS":mujoco.mjtState.mjSTATE_QPOS,"QVEL":mujoco.mjtState.mjSTATE_QVEL,
 "ACT":mujoco.mjtState.mjSTATE_ACT,"WARMSTART":mujoco.mjtState.mjSTATE_WARMSTART,"CTRL":mujoco.mjtState.mjSTATE_CTRL,
 "QFRC_APPLIED":mujoco.mjtState.mjSTATE_QFRC_APPLIED,"XFRC_APPLIED":mujoco.mjtState.mjSTATE_XFRC_APPLIED,
 "EQ_ACTIVE":mujoco.mjtState.mjSTATE_EQ_ACTIVE,"MOCAP_POS":mujoco.mjtState.mjSTATE_MOCAP_POS,
 "MOCAP_QUAT":mujoco.mjtState.mjSTATE_MOCAP_QUAT,"USERDATA":mujoco.mjtState.mjSTATE_USERDATA,
 "PLUGIN":mujoco.mjtState.mjSTATE_PLUGIN,"INTEGRATION":mujoco.mjtState.mjSTATE_INTEGRATION}
for k,v in flags.items():
    try: print(f"{k:14s} size={int(mujoco.mj_stateSize(m,v))}")
    except Exception as e: print(k,"err",e)
# empirical: does setState(INTEGRATION) copy ctrl?
d1=mujoco.MjData(m); d2=mujoco.MjData(m)
d1.ctrl[:]=np.arange(m.nu)+0.1
d1.qpos[:]=0.5; d1.time=1.25
vec=np.zeros(int(mujoco.mj_stateSize(m,mujoco.mjtState.mjSTATE_INTEGRATION))); mujoco.mj_getState(m,d1,vec,mujoco.mjtState.mjSTATE_INTEGRATION)
mujoco.mj_setState(m,d2,vec,mujoco.mjtState.mjSTATE_INTEGRATION)
print("shadow ctrl after setState(INTEGRATION):",d2.ctrl)
print("shadow time:",d2.time,"qpos[0]",d2.qpos[0])
