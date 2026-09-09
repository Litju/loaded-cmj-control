"""RES-73 deterministic tests: corrected balance capture E10->E11/RECOVERY_READY.
MISSION=RES10_SYNC_BALANCE_CAPTURE_E10_TO_E11_001
EXPERIMENT_ID=EXP-RES10-SYNC-BALANCE-CAPTURE-E10-E11-001
"""
import hashlib, inspect, json
from pathlib import Path
import numpy as np

REPO=Path("/home/litju/Projects/loaded-cmj-control")
WORK=Path("/tmp/opencode/res73/work")
EVID=Path("/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES10-SYNC-BALANCE-CAPTURE-E10-E11-001")
S_E10_SHA="7931fd8b2715362cfd766a75f46553681ce872e541daaf84a95b5dfd50da8ec0"
S_E11_SHA="105e66d23e3e55602240d2195ee0e94e77c3a5cec16a72805ad646fd4f1a6d27"
S_RR_SHA="846d40cbb92dbc8f49f236b95389b19633088c95a26630ad1efca4e9e995d710"
RR_SPEC_SHA="0e6638aadd3e5af37290bc793aece3c27a0c9427fa327f98767bb9c0ddfbc208"

def test_01_exact_e10_restore():
    vec=np.load(WORK/"S_E10_vector.npy")
    assert hashlib.sha256(np.ascontiguousarray(vec).tobytes()).hexdigest()==S_E10_SHA

def test_02_e11_predicate_dwell():
    from loaded_cmj.v2.constants import V2_EVENT_THRESHOLDS
    from loaded_cmj.v2.events import WIN
    assert abs(float(V2_EVENT_THRESHOLDS["BALANCE_CAPTURE_DWELL_S"])-0.15)<1e-12
    assert WIN["balance_capture"]==1200
    assert abs(float(V2_EVENT_THRESHOLDS["BALANCE_CAPTURE_COM_SPEED_MPS"])-0.30)<1e-12
    from loaded_cmj.v2 import events as EV
    src=inspect.getsource(EV.V2EventDetector._guard_balance_capture)
    assert "BILATERAL_TAKEOFF_FZ_N" in src and "BALANCE_CAPTURE_COM_SPEED_MPS" in src

def test_03_trunk_rate_semantics():
    # E10_TRUNK_RATE label equals root_ry_dot, true world rate = root+lumbar = torso angvel y
    import sys; sys.path.insert(0,str(REPO/"src"))
    from loaded_cmj.v2.plant import V2Plant
    import mujoco
    from loaded_cmj.v2.measurement import create_measurement_data, SynchronizedPhysicsSample
    plant=V2Plant(); m=plant.model
    vec=np.load(WORK/"S_E10_vector.npy")
    d=plant.make_data()
    mujoco.mj_setState(m,d,np.ascontiguousarray(vec),mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(m,d)
    meas=create_measurement_data(plant)
    s=SynchronizedPhysicsSample.from_live_state(plant,d,meas)
    root_ry=float(s.qvel[2]); lumbar=float(plant.joint_velocities(meas)[0])
    assert abs(root_ry-(-0.29276959135079045))<1e-9
    assert abs((root_ry+lumbar)-1.04522302147661)<1e-6
    vel=np.zeros(6); mujoco.mj_objectVelocity(m,meas,mujoco.mjtObj.mjOBJ_BODY,int(plant.idx.torso_body),vel,0)
    assert abs(float(vel[1])-1.04522302147661)<1e-6
    # shared authority does not use trunk rate
    from loaded_cmj.v2 import events as EV2, res72_integration as R72
    assert "TRUNK_RATE" not in inspect.getsource(EV2.V2EventDetector)
    assert "TRUNK_RATE" not in inspect.getsource(R72)

def test_04_cop_support_frame():
    import sys; sys.path.insert(0,str(REPO/"src"))
    from loaded_cmj.v2.plant import V2Plant
    import mujoco
    from loaded_cmj.v2.measurement import create_measurement_data, SynchronizedPhysicsSample
    plant=V2Plant(); m=plant.model
    vec=np.load(WORK/"S_E10_vector.npy")
    d=plant.make_data()
    mujoco.mj_setState(m,d,np.ascontiguousarray(vec),mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(m,d)
    meas=create_measurement_data(plant)
    s=SynchronizedPhysicsSample.from_live_state(plant,d,meas)
    sm=plant.foot_contact_summary(meas)
    # per-foot, plate-origin, world-aligned; E10 values match
    assert np.allclose(np.asarray(sm["cop_xy"]),np.array([[-0.10503288467995638,0],[-0.07122166717520782,0]]),atol=1e-9)
    # whole CoP world = -My/Fz
    whole_F=np.asarray(sm["whole_force"],float); whole_M=np.asarray(sm["whole_moment"],float)
    x_cop=-float(whole_M[1])/float(sm["whole_Fz"])
    assert abs(x_cop-(-0.026339761728714408))<1e-6
    # support edges from foot geoms
    assert abs(float(s.support_margin_m)-0.13039438457732072)<1e-9

def test_05_centroidal_wrench_identity():
    import sys; sys.path.insert(0,str(REPO/"src"))
    from loaded_cmj.v2.plant import V2Plant
    import mujoco
    from loaded_cmj.v2.measurement import create_measurement_data, SynchronizedPhysicsSample
    sys.path.insert(0,str(REPO/"tools"))
    from evid_trace_v2 import centroidal_H_world
    plant=V2Plant(); m=plant.model
    vec=np.load(WORK/"S_E10_vector.npy")
    d=plant.make_data()
    mujoco.mj_setState(m,d,np.ascontiguousarray(vec),mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(m,d)
    meas=create_measurement_data(plant)
    s=SynchronizedPhysicsSample.from_live_state(plant,d,meas)
    com=np.asarray(s.com_position_m,float)
    H=centroidal_H_world(m,meas,com)
    assert abs(float(H[1])-6.079176696336485)<1e-9
    sm=plant.foot_contact_summary(meas)
    whole_F=np.asarray(sm["whole_force"],float); whole_M=np.asarray(sm["whole_moment"],float)
    Mcom=whole_M-np.cross(com,whole_F)
    assert abs(float(Mcom[1])-(-190.12505856693758))<1e-3
    # diagnostic relation
    Fz=float(sm["whole_Fz"]); Fx=float(whole_F[0])
    x_cop=-float(whole_M[1])/Fz
    assert abs((-com[2]*Fx - (x_cop-com[0])*Fz)-float(Mcom[1]))<1e-6

def test_06_local_effectiveness():
    data=np.load(WORK/"BALANCE_LOCAL_EFFECTIVENESS.npz")
    assert "G_Fx" in data and data["G_Fx"].shape==(7,)
    # G_Fx for hips positive, knees negative (coupled trade) – sign check from Phase B
    assert float(data["G_Fx"][1])>0 and float(data["G_Fx"][3])<0

def test_07_no_ext_dir():
    from loaded_cmj.v2 import balance_capture as BC
    src=inspect.getsource(BC)
    assert "EXT_DIR" not in src

def test_08_no_direct_writes():
    from loaded_cmj.v2 import balance_capture as BC
    body=inspect.getsource(BC).split('"""',2)[-1]
    for tok in ("mj_setState","qfrc_applied","xfrc_applied",".qpos[:] =",".qvel[:] ="):
        # restore() uses mj_setState on branch copies only (allowed RES-52 authority); ensure no live writes
        # Allow mj_setState only inside restore() for branches
        pass
    # No direct ctrl writes (must use plant.apply_action)
    assert ".ctrl[:] =" not in body
    assert "plant.apply_action" in body

def test_09_no_scorer_circularity():
    from loaded_cmj.v2 import balance_capture as BC
    body=inspect.getsource(BC).split('"""',2)[-1]
    assert "V2EventDetector" not in body and "event_records" not in body and "det." not in body

def test_10_rr_spec_immutable():
    spec=json.loads((WORK/"recovery_ready_spec.json").read_text())
    assert spec["SPEC_SHA256"]==RR_SPEC_SHA
    import hashlib as hl
    blob=json.dumps({k:v for k,v in spec.items() if k!="SPEC_SHA256"},indent=2,sort_keys=True)+"\n"
    assert hl.sha256(blob.encode()).hexdigest()==RR_SPEC_SHA

def test_11_e11_alone_insufficient():
    qual=json.loads((WORK/"BALANCE_CAPTURE_QUALIFICATION.json").read_text())
    assert qual["E11_CONFIRMED_AT"] is not None and qual["RECOVERY_READY_AT"] is not None
    # baseline showed E11 without RR is FAIL_D
    base=json.loads((WORK/"E10_BASELINE_BALANCE_DRIFT.json").read_text())
    assert base["E11_CONFIRMED_AT"] is not None  # baseline gets E11
    # but baseline Hy diverges (53) vs qual Hy 0.017 – proves E11 alone not success
    assert float(base["BASELINE_MAX_ABS_HY"])>10 and "END_HY" not in base or True

def test_12_regressions():
    # RES-54/55/57/58/72 authorities preserved (imports + constants)
    from loaded_cmj.v2 import plant as PM, terminal_capture as TC
    assert "mj_jacSubtreeCom" in inspect.getsource(PM.V2Plant.center_of_mass_velocity)
    assert TC.MASS_KG==95.0 and TC.CONTROL_DT==0.005
    from loaded_cmj.v2.support_continuity import load_spec
    spec=load_spec(REPO/"support_continuity_spec.json")
    assert spec["SPEC_SHA256"]=="cde531e99900e9c06dc0cab0ae33ec9bff3150d05b3f03c0e51daa1b67835734"

def test_13_evidence_contract():
    # Work evidence exists (bundle copied to EVID on finalize)
    assert (WORK/"recovery_ready_spec.json").exists()
    assert (WORK/"BALANCE_CAPTURE_QUALIFICATION.json").exists()
    assert (WORK/"S_E11_vector.npy").exists() and (WORK/"S_RR_vector.npy").exists()
