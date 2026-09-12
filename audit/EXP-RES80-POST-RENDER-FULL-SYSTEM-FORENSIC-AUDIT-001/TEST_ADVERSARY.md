# TEST_ADVERSARY (RES-80)

Adversarial demonstration that the frozen test suite can PASS while the intended scientific requirement
is false. Every construction cites file:line. No test was modified.

## Construction 1 - "E12 reached" with only E9 latched
`tests/test_v2_1_res10_honest_full_jump.py:164-176`: `test_e12`, `test_event_count` and `test_no_fall`
assert `len(res.event_records) >= 9`. A trajectory that latches supported_start..descending_landing
(9 events) and then falls or stalls passes all three. The file's own comments document the intent
("we have 10, so pass as 12"). **Claim that fails: stable recovery / no-fall.**

## Construction 2 - Broken external-moment bookkeeping passes
`:98-100` `test_external_moment` is `assert True`. Any Hy/gravity-moment error passes. The sibling
`test_centroidal_identity` only checks `|Hy|<1e-6` at rest. **Claim that fails: centroidal momentum
correctness.**

## Construction 3 - Arbitrary penetration passes
`:153-154` `test_penetration` is `assert True`. A 5 cm penetration would pass. **Claim that fails:
penetration <= declared cap.**

## Construction 4 - Reflight after landing passes
`:156-157` `test_no_reflight` is `assert True`, while the canonical result's POST_LANDING_REFLIGHT gate
is the only real check. **Claim that fails: no post-landing reflight.**

## Construction 5 - Scorer circularity hidden behind a rename passes
`tests/test_res76_full_closure.py:47-55`: the only assertions are source-text scans for the strings
`"V2EventDetector"`/`"event_records"`, and the final check is terminated by `or True`. A controller
reading scorer state via an alias, wrapper, or `__getattr__` passes. The canonical runtime already
violates the spirit by calling the private scorer predicate for control (HIGH-007). **Claim that fails:
scorer/controller separation.**

## Construction 6 - Root contact-parameter regression passes
`tests/test_v2_1_res31_honest_planar_root.py:197`: the only check that root `jnt_solref` is unchanged is
neutralized by `or True`. **Claim that fails: root contact parameters frozen.**

## Construction 7 - "E11 alone insufficient" cannot fail
`tests/test_res73_balance_capture.py:143`: `assert float(base["BASELINE_MAX_ABS_HY"])>10 and "END_HY" not in base or True`
parses as `(A and B) or True`. A baseline bundle violating both conditions passes. **Claim that fails:
E11-alone-insufficient negative control.**

## Construction 8 - Threshold test tests a local lambda
`tests/test_res55_true_foot_point_velocity.py:136-149`: `def cls(v): return "INSIDE" if abs(v)<0.05 else "OUTSIDE"`
is defined inside the test; the production threshold can change without failing. **Claim that fails:
production threshold behavior.**

## Construction 9 - Stale canonical result passes 13/13
`tests/test_res78_canonical_runtime.py:52-56,179-223` reads stored JSON from `/tmp/opencode/res78/work`
or the RES-12A bundle. `run_canonical_episode` is only checked with `callable()` (line 87). A result
generated from older source bytes (same schema) passes. **Claim that fails: current-source
qualification.**

## Construction 10 - Production `assert` checks disappear under `python -O`
`res72_integration.py:213-214` and `canonical_runtime.py:223,225` use `assert` for runtime contracts.
Run with optimization and the "held action" / "bound policy" checks vanish (BUG-06). **Claim that
fails: fail-closed contract enforcement.**

## Construction 11 - Always-true reproducibility check
`tests/test_rec01a_synchronized_dynamics.py:208-234`: `assert max(deltas) >= 0.0` can never fail.
**Claim that fails: synchronized-dynamics reproducibility.**

## Construction 12 - Unconditional PASS qualification scripts
`tests/qualification_causal_runtime.py:470-497` writes `result: PASS`, sets `focused_passed=len(checks)`,
returns 0; it is excluded from `test_public_qualification.py`'s suite list. A broken causal runtime still
"passes" its qualification script. **Claim that fails: causal-runtime qualification.**

## Severity summary

The two highest-impact adversary results are Construction 1/9 (the files a reader would cite for
"12/12" and "canonical qualification" cannot detect their named failures). Together they mean the
repository's tests would not have caught the owner's physical credibility failure even if every test
passed. This is consistent with the mission premise and is the basis of TEST_DEFECT findings
HIGH-015..019.
