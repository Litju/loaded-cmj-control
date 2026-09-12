# CODE_REVIEW (RES-80, independent pass)

Review of implementation quality/correctness on the V2.1 execution path. No fixes applied.
Findings not already itemized in the defect register are noted here; most map to register IDs.

## 1. Positive verifications (things that are correct)

- **Contact force transform**: `frame.T @ (sign*wrench)` matched `qfrc_constraint` on the root DOFs
  exactly at flat, rotated and single-contact states; both geom orderings handled. (P31 rejected.)
- **State staging**: `mjSTATE_INTEGRATION` (108 values) includes `ctrl`; the shadow measurement therefore
  evaluates forces under the previously held action, matching the documented
  `OBSERVATION_INPUT_CONTROL=PREVIOUS_HELD_ACTION` convention. The canonical runtime's
  `assert np.array_equal(sync0.ctrl, held)` is valid.
- **COM velocity authority**: `mj_jacSubtreeCom(pelvis) @ qvel` agrees with central differences of the
  subtree COM and with `mj_subtreeVel` after `mj_forward` to 1e-5 m/s over the trajectory.
- **Root physics**: zero root damping/stiffness/armature; no root limit rows in R001; `MAXROOTPASSIVE=0.0`;
  `ROOT_ROWS=0`. The "honestly floating root" property holds.
- **Determinism**: replay of the sealed action schedule reproduces the trace SHA, event times and
  checkpoints; the offline detector recomputation matches online.

## 2. Implementation-quality findings

1. **Duplicated constants with divergent authority** (MED-018): E10 dwell and RR thresholds exist as
   runtime literals, in the hashed-but-dead `full_closure.py`, and in external JSON; no binding.
2. **Dead code and misleading names**: `BalanceController.y_of` unused; `soft_contact._dist_rows`
   docstring vs behavior; dead conditional `"FZ_SCALE_M" if False else`; `_impact_kd` identity ramp;
   `unilateral_dropout_samples` returns an array under a Boolean-array docstring and is never called.
3. **Monolithic magic numbers**: contact/support literals (0.150/0.060/0.010) are duplicated in the
   plant and the trace tool rather than read from the model; the adjacent comment disagrees.
4. **Error handling**: legacy `controller.py` has a bare `except: return [0]*7`; canonical runtime
   swallows checkpoint exceptions and converts adjudication errors into sentinel values that no gate
   rejects.
5. **Import hygiene (MED-016)**: library modules mutate `sys.path` and hardcode developer paths;
   `evid_trace_v2` is imported under two module identities.
6. **Test/implementation drift**: tests import the legacy controller and re-implement production
   formulas locally (`test_res55` threshold lambda, `test_res51` planner re-implementation), so the
   tests can pass while production diverges.
7. **Observation placeholders**: hardcoded identity pelvis quaternion; unsigned trunk tilt; CoP validity
   threshold duplicated at 20 N.
8. **File organization**: production depends on `tools/` modules without being packaged with them; the
   canonical orchestrator duplicates the transition logic of the dead `full_closure` mirror.

## 3. Severity mapping

No new independent defects were discovered in this pass beyond the register. The quality issues above
are captured by MED-003/004/005/010/011/016/017/018, LOW-003/004/005/006/007 and CRIT-004/006.

**Verdict:** the V2.1 code is competently constructed for bit-exact deterministic execution and the
physics-facing primitives tested here are correct, but it carries substantial dead/duplicated authority,
weak error surfacing, and a production import story that contradicts its packaging. The most
consequential quality issue is the provenance/identity gap (CRIT-004/005/006), not code style.
