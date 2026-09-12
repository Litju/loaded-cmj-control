"""Deterministic proof: FZ_MIN rows disappear during soft_contact refinement passes."""
import sys, numpy as np
sys.path.insert(0,"/home/litju/Projects/loaded-cmj-control/tools/res52")
sys.path.insert(0,"/home/litju/Projects/loaded-cmj-control/tools")
import soft_contact as SC

calls=[]
orig=SC.lsq_linear
def spy(A,b,**kw):
    calls.append(np.asarray(A).copy())
    return orig(A,b,**kw)
SC.lsq_linear=spy

K={"SCALES":{"FZ_SCALE_N":100.0,"DIST_SCALE_M":0.0005,"NVEL_SCALE_MPS":0.05,"VZ_SCALE_MPS":0.02,"QDOT_SCALE_RADPS":0.3},
   "LS_WEIGHTS":{"W_FZ_WHOLE":1.5,"W_FZ_L":0.5,"W_FZ_R":0.5,"W_NVEL":0.5,"W_VZ":1.0,"W_QDOT":0.2,
                 "W_DIST_ENGAGE":4.0,"W_DIST_MAX":4.0,"W_DIST_RETAIN":2.0,"W_REG":0.001,"W_FZ_MIN":1.0},
   "PEN_MAX_M":0.009,"PEN_MIN_RETAIN_M":0.0005,"FZ_MIN_MARGIN_N":25.0,
   "RHO0_U":0.25,"RHO_MIN_U":0.0025,"RHO_SHRINK_FACTOR":0.5,"RHO_RECOVERY_FACTOR_PER_UPDATE":1.19,
   "MAX_TRUST_SHRINKS_PER_UPDATE":3,"TRUST_REL":0.1,"VALIDATION_TOL":{}}
obj=SC.SoftContactPolicy.__new__(SC.SoftContactPolicy)
obj.k=K
NY=18
rng=np.random.default_rng(0)
y_nom=np.zeros(NY); G=rng.normal(0,1,(NY,7))
y_nom[3]=0.02; y_nom[4]=0.02     # separated -> dist rows trigger
y_nom[15]=-100.0                  # fz-min violated -> FZ_MIN row triggers in pass 1
u_nom=np.zeros(7)
du=obj.solve_du(y_nom,G,900.0,0.0,u_nom,0.25)
print("number of lsq_linear passes:",len(calls))
# FZ_MIN rows are the only rows whose row vector equals W_FZ_MIN*G[15..17]/scale
tag=K["LS_WEIGHTS"]["W_FZ_MIN"]/K["SCALES"]["FZ_SCALE_N"]
for k,A in enumerate(calls):
    # detect fz-min rows by correlating rows of A (excluding reg rows) with G[15],G[16],G[17]
    nrows=A.shape[0]-7
    found=False
    for r in range(nrows):
        for mi in (15,16,17):
            if np.allclose(A[r],tag*G[mi],atol=1e-12): found=True
    print(f"pass {k}: rows_before_reg={nrows}  contains_FZMIN_row={found}")
