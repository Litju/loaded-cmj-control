# Corrected Apex-to-E10 Integration Authority (RES-72, V2.1)

`MISSION=RES10_SYNC_CORRECTED_APEX_TO_E10_INTEGRATION_001`
`EXPERIMENT_ID=EXP-RES10-SYNC-CORRECTED-APEX-E10-001`

Single frozen composition (no search, no tuning):

```text
fresh reset
→ committed v2.controller (HOLD/SUPPORTED/FLIGHT, R5A2 residual)
→ C01 LANDING_PREP (PREP_TARGET [0,.30,.70,.20], Kp80/Kd12, trigger FLIGHT_and_cvz<-0.005)
→ C01 INITIAL_IMPACT (IMPACT_TARGET [0,.32,.75,.20], Kp114/Kd20 fixed)
→ physical touchdown (whole>=20, max>=20)
→ TD+0.050 handoff at first control boundary (proven S50=TD+0.050, not absolute 0.925)
→ exact sealed RES-58 terminal law (VZ_TARGET 0.0, DT 0.005, BW=m*g, clamp [0.6,1.5]BW)
  + sealed RES-52 SoftContactPolicy inner
→ canonical E10 occurred+confirmed → STOP
```

## Provenance

- C01: `EXP-RES10-SYNC-PREDICTIVE-TD-VDAMP-E8-E10-001` C01 (`9173bf26`), harness `533d90/a0cac8`.
- Apex: corrected `0.7719764018454335`, SHA `97ed110f...`; old E8 `401b45...` superseded (historical only).
- S50: `TD+0.050` proven in `core52.S_OFFSETS_S`, `BRANCH_STATE_AUTHORITY.S_TIMES`, sealed spec.
- RES-58: `06e73896...` functional identity 0.0, constants frozen.
- Measurements: RES-54 `jacSubtreeCom`, RES-55 `J_point@qvel+efc_vel`.
- Support: RES-57 `cde531e9...`, dwell 5ms = one control interval (not Nyquist).

## Qualification (fresh reset, corrected)

- TD 0.875, handoff 0.925 (elapsed 0.050), E10 0.973625/0.9935.
- Peak 4.812 BW, pen 9.638mm, dropout 0, liftoff 0, support loss 0, reflight [], chatter 0.
- Prohibited false, root rows 0, passive 0, util 0.903, finite true, no fall.
- E10 vz 0.010 (capture), vx 0.279/Hy 6.07 energetic → class E SUCCESS (future E10→E11 work).
- Online/offline PASS, determinism bit-identical (qual vs repro).

Reusable logic: `src/loaded_cmj/v2/res72_integration.py`.
Tests: `tests/test_res72_corrected_apex_to_e10.py`.
Evidence: `EXP-RES10-SYNC-CORRECTED-APEX-E10-001` (Evidence Contract v2, SPEC_EXECUTION_MATCH PASS).

No E11/E12. No takeoff/Plant/contact/scorer/terminal-law changes.
Next authorized unit (do not execute here): `RES10_SYNC_BALANCE_CAPTURE_E10_TO_E11_001`.
