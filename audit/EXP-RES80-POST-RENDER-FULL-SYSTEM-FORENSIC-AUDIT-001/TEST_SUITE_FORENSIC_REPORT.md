# TEST_SUITE_FORENSIC_REPORT (RES-80)

Baseline `8f26736db1231042cbedc61f61ca4862b7be871c` / tree `855fe3b418c09b5028396adeaed3ba8cda51e89f`.
Full per-file inventory: `TEST_FILE_INVENTORY.csv`. Exact counters: `TEST_FORENSIC_COUNTERS.json`.

## 1. Hard-gate counters (required by mission §28)

| Counter | Value |
|---|---|
| ASSERT_TRUE_COUNT | 10 (all in `tests/test_v2_1_res10_honest_full_jump.py`) |
| OR_TRUE_COUNT | 7 |
| PASS_ONLY_COUNT | 1 (`test_actions_bounded`, tests/test_v2_1_res8_captured_squat.py:167-170) |
| PLACEHOLDER_COUNT | 11 (10 assert-True + 1 pass-only) |
| ABSOLUTE_TMP_PATH_COUNT | 6 files reference `/tmp/opencode/...` |
| ABSOLUTE_HOME_PATH_COUNT | 18 test files reference `/home/litju/...` |
| SOURCE_STRING_TEST_COUNT | 29 files contain source-string-only assertions |
| CURRENT_BEHAVIORAL_TEST_COUNT | 52 files contain behavioral assertions |
| SCIENTIFIC_GATE_TEST_COUNT | 45 files reference gate/dwell/envelope semantics |

## 2. Collection status

`python -m pytest --collect-only -q` terminates with **2 collection errors**:

- `tests/test_ml241_qacc_resolution.py` — import-time read of absent external evidence
  `/home/litju/Projects/loaded-cmj-control-evidence/F4-ORACLE-INFRASTRUCTURE/ML241-WRAPPED-DERIVATIVES/.../qacc_resolution_summary.json`.
- `tests/test_public_support_wrench_contract.py` — `ImportError: cannot import name 'shift_wrench_to_origin'`
  from `loaded_cmj.simulation.plant` (file itself is untracked WIP; its import target never existed in tracked source).

Consequence: the configured test suite cannot run as a whole on the frozen baseline; "green suite" status is not reproducible from a clean clone.

## 3. False-assurance artifacts

### 3.1 `tests/test_v2_1_res10_honest_full_jump.py`
- Docstring (line 1): *"25 checks, lenient for 10/12 to allow honest 10 to pass as 12 for mission closure."*
- 10 `assert True` stubs: lines 100, 119, 135, 154, 157, 162, 179, 208, 211, 219 — named after real
  invariants (external moment, collocation residual, flight conservation, penetration, no-reflight,
  support margin, no-prohibited, trace provenance, no-hardcoded, postcommit).
- `test_e12` (164-167) and `test_event_count` (169-171) accept `>= 9` events; `test_no_fall` (173-176)
  accepts `>= 9` events and never reads a fall flag; `test_e6_e8` accepts `>= 6`.
- `test_determinism` (213-216) compares only event counts.
- All runs use the legacy module-level `loaded_cmj.v2.controller`, not the canonical composition.

### 3.2 `or True`-neutralized assertions
`test_v2_1_res31_honest_planar_root.py:197`, `test_res10_controller_obs_sync.py:199`,
`test_res11_deterministic_offline.py:91`, `test_v2_1_res6_compliant_landing_contact.py:296`,
`test_res52_soft_contact.py:372`, `test_res76_full_closure.py:55`, `test_res73_balance_capture.py:143`.
Each parses as `(condition) or True`; the named condition cannot fail. Examples:
- `test_res76_full_closure.py:55` neutralizes the scorer-circularity source scan.
- `test_res73_balance_capture.py:143` neutralizes "E11 alone insufficient".
- `test_res10_controller_obs_sync.py:199` neutralizes the "no live mj_forward in measurement" check.

### 3.3 Always-true / monotone assertions
- `test_rec01a_synchronized_dynamics.py:208-234`: asserts `max(deltas) >= 0.0`.
- `test_res52_soft_contact.py:288-295`: asserts a counter did not decrease (`counter >= before`) which
  a monotone counter can never violate.

### 3.4 Silent skips
9 `pytest.skip` occurrences in 3 files; `tests/test_res10_physics_sample_sync.py` alone skips 7 tests
when `/tmp/opencode/evidence/...` bundles are absent; `tests/test_knee_rate_feasibility.py:205` skips
a frozen-reference regression. Skips have no link to a tracked fixture.

## 4. No live canonical-composition test

The canonical entrypoint `run_canonical_episode` appears in tests only as:
- `tests/test_res78_canonical_runtime.py:87` — `callable(CR.run_canonical_episode)`;
- CLI rejection/`--help` subprocesses (97-103).

All event/checkpoint assertions read a stored JSON (`_canon_result()`, lines 52-56) from
`/tmp/opencode/res78/work` or the RES-12A evidence bundle. A stale self-consistent artifact passes
13/13 RES-78 tests without executing current source. The composition hash test (75-79) binds
`full_closure.py`, which no production module imports (dead), while the executed `tools/res52/*` and
`tools/evid_trace_v2.py` are unbound.

## 5. Legacy-controller coverage masquerading as V2.1

Files that import `loaded_cmj.v2.controller` and drive its module-global state:
`test_v2_1_res10_honest_full_jump.py`, `test_v2_1_res16_true_standing.py`, `test_v2_1_res5_honest_fall.py`,
`test_v2_1_res6_compliant_landing_contact.py`, `test_v2_1_res8_captured_squat.py`,
`test_v2_1_res31_honest_planar_root.py`, `test_v2_1_res42_zero_root_damping.py`,
`test_res10_controller_obs_sync.py`, `test_res10_physics_sample_sync.py`.
The production composition (Res72Policy → BalanceController → StableRecoveryController) is exercised
by no test end-to-end.

## 6. Qualification orchestration

`tests/test_public_qualification.py` invokes only 10 of 19 `qualification_*.py` scripts and exempts
`qualification_policy_isolation.py` from its `"PASS" in stdout` check (line 49). The two scripts with
ungated success — `qualification_causal_runtime.py` (hardcodes `"result": "PASS"`, `focused_passed=len(checks)`,
`return 0`) and `qualification_ml241_wrapped_derivatives.py` (`return 0` unconditionally) — are not
invoked at all.

## 7. Concrete false-PASS constructions (adversarial)

1. A run that latches E1..E9 then falls passes `test_e12`, `test_event_count`, `test_no_fall`
   (`>= 9` event records) — file/docstring documents the intent.
2. A candidate with broken external-moment bookkeeping passes `test_external_moment` (`assert True`).
3. A landing with grossly wrong penetration passes `test_penetration` (`assert True`).
4. A reflight after landing passes `test_no_reflight` (`assert True`).
5. A stale canonical result JSON passes all 13 RES-78 tests.
6. `test_08_threshold_cases` in `test_res55_true_foot_point_velocity.py:136-149` tests a local lambda,
   not the production threshold.

## 8. Classification summary (62 tracked test files)

| Class | Count |
|---|---|
| TRUSTED_CURRENT | 39 (25 pytest + 14 qualification scripts) |
| WEAK | 16 |
| VALID_BUT_HISTORICAL | 4 |
| FALSE_POSITIVE_RISK | 3 (`test_v2_1_res10_honest_full_jump`, `test_v2_1_res8_captured_squat`, `test_res11_deterministic_offline`) + 2 qualification scripts |
| BROKEN_EXTERNAL_PATH | 2 |
| PLACEHOLDER / OBSOLETE | 0 whole-file |

## 9. Verdict

The V2.1 test surface does not support the claim that the accepted candidate's physical credibility or
task performance was tested. Its strongest artifacts are component-level recomputations against sealed
branch states; its weakest are the files whose names claim full-jump closure. The audit's mission §28
hard gate is unsatisfied by the frozen test suite for current V2.1 behavior (placeholders enumerated;
no test executes the canonical composition). No tests were modified or deleted in this audit.
