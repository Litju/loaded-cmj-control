import numpy as np, mujoco
XML="/home/litju/Projects/loaded-cmj-control/src/loaded_cmj/v2/assets/v2_plant.xml"
m=mujoco.MjModel.from_xml_string(open(XML).read())
d=mujoco.MjData(m)
def reset(q):
    mujoco.mj_resetData(m,d); d.qpos[:]=q; mujoco.mj_forward(m,d)
nm=lambda o,i: mujoco.mj_id2name(m,o,i)
# penetrate slightly: root z 0.8998
base=np.array([0,0.8998,0,0,0,0,0,0,0,0],float)
reset(base); mujoco.mj_step(m,d)
print("ncon", d.ncon)
for i in range(d.ncon):
    c=d.contact[i]; n1=nm(mujoco.mjtObj.mjOBJ_GEOM,c.geom1); n2=nm(mujoco.mjtObj.mjOBJ_GEOM,c.geom2)
    R=c.frame.reshape(3,3); w=np.zeros(6); mujoco.mj_contactForce(m,d,i,w)
    print(f"{n1} x {n2} dist={c.dist:+.6f} pos={c.pos}")
    print(" frame rows:", np.array2string(R, precision=3))
    print(" f_local:", w[:3], " frame@f:", np.round(R@w[:3],4), " frame.T@f:", np.round(R.T@w[:3],4))
print()
# tilted foot: rotate ankle to +0.35 with root adjusted so forefoot edge penetrates
for ank in (-0.35, 0.0, 0.35):
    q=base.copy(); q[6]=ank; q[9]=ank
    # find root z that makes true lowest point -0.0002
    for _ in range(60):
        reset(q); R=d.geom_xmat[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'left_foot_box')].reshape(3,3)
        c0=d.geom_xpos[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'left_foot_box')]
        sh=np.array([.15,.06,.01])
        low=min((c0+R@(sh*np.array(s)))[2] for s in [(i,j,k) for i in (-1,1) for j in (-1,1) for k in (-1,1)])
        q[1]-= (low+0.0002)
    mujoco.mj_step(m,d)
    tot=np.zeros(3)
    for i in range(d.ncon):
        c=d.contact[i]; n1=nm(mujoco.mjtObj.mjOBJ_GEOM,c.geom1); n2=nm(mujoco.mjtObj.mjOBJ_GEOM,c.geom2)
        if 'foot' not in n1+n2 or 'floor' not in n1+n2: continue
        R=c.frame.reshape(3,3); w=np.zeros(6); mujoco.mj_contactForce(m,d,i,w)
        foot_is_2 = 'foot' in n2
        sign = 1.0 if foot_is_2 else -1.0
        tot += R.T@(sign*w[:3])
    print(f"ankle={ank:+.2f} rootz={q[1]:.5f} ncon={d.ncon} sum(frame.T@f)={np.round(tot,2)}  |F|={np.linalg.norm(tot):.1f} weight=931.95 z={tot[2]:.1f}")
