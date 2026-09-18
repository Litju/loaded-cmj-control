# RES85D_STRICT_ROM_RECEIPT

MISSION: `RES85D_STRICT_ROM_RECONCILIATION_AND_FINAL_RESEAL_001`
LINEAR ISSUE: RES-85
ENTRY_HEAD: `ffc98526bd2b0da76dbef50891415ebdb345a1fd`
ENTRY_TREE: `612b0b8a667b32ef8f217e4aed569e5c2832ff84`
RES85C_HEAD: `ffc98526bd2b0da76dbef50891415ebdb345a1fd`
RES83_PLANT_XML_SHA256: `eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d`
RES84_EVIDENCE_SEAL_SHA256: `223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda`
PREVIOUS_RES85_EVIDENCE_SEAL_SHA256: `235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e`
TELEMETRY_BLOB_SHA256: `2944bdf78888e694905f9bdbafc60f393a0a304f7d6dc6fa3aa1e1e512fb670b`

## Previous evidence attribution (RES-85C is NOT silently rewritten)

The RES-85C canonical episode remains attributable in git history at
`ffc98526bd2b0da76dbef50891415ebdb345a1fd` and is audited here from its immutable native telemetry
(`RES85C_PREDECESSOR_STRICT_ROM.json`).  The RES-85C receipts are preserved
byte-for-byte in this bundle.

| RES-85C canonical quantity | Value | Verdict |
|---|---|---|
| H2 (direct SYSTEM_COM) | `0.18867197309831352` m | historical |
| trunk_pelvis measured max | `0.6734678378904122` rad | **STRICT STRUCTURAL ROM FAIL** |
| frozen trunk_pelvis upper | `0.610865` rad | exceeded by `0.06260283789041221` rad |
| global min structural ROM margin | `-0.13469037187594915` rad | below the 1e-9 tolerance |
| previous accepted occurrence | sample `666` | historical |
| previous confirmation | sample `691` | historical |

The predecessor audit is re-executed from the native arrays with the same
strict checker used for the RES-85D qualification; its verdict is FAIL on
`trunk_pelvis:upper` (see `RES85C_PREDECESSOR_STRICT_ROM.json`).

## New canonical result (RES-85D)

| Quantity | Value |
|---|---|
| H2 (direct SYSTEM_COM) | `0.16021398534364972` m |
| takeoff vz | `1.7867932492654341` m/s |
| accepted occurrence | sample `612` / t=`1.224` s |
| takeoff confirmation | sample `637` / t=`1.274` s |
| RES85 claim end (first legal plantar recontact) | sample `791` / t=`1.582` s |
| ballistic cross-check residual | `-0.0025092621395576` m |
| functional floor classification | `ABOVE_FLOOR` (closure `True`) |

`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` and
`ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED` (unchanged).  The
`H_ANTI_TRIVIALITY_FLOOR = 0.150 m` is a hard functional non-triviality
boundary, not an elite norm and not an optimization target.

## Strict structural ROM (frozen human envelope)

* domain: `FIRST_NATIVE_SAMPLE_THROUGH_RES85_CLAIM_END` (claim end sample `791`)
* criterion: measured q_j(t) stays within [lower_j - tolerance, upper_j + tolerance] for every bounded actuated joint and every native sample in the RES-85 claim domain; the tolerance is floating-point equality only, never added anatomical ROM
* tolerance: `1e-09` rad (`FLOATING_POINT_EQUALITY_ONLY_NOT_ANATOMICAL_ROM`)
* `GLOBAL_MIN_STRUCTURAL_ROM_MARGIN_RAD = 0.0`
* audit status: `PASS`
* solver soft-limit probe role: `NUMERICAL_SOLVER_DIAGNOSTIC_ONLY_NOT_STRUCTURAL_ACCEPTANCE_AUTHORITY`

| joint | frozen lower | frozen upper | min measured | max measured | min margin | status |
|---|---|---|---|---|---|---|
| `left_ankle` | `-0.959931` | `0.785398` | `-0.05984371392172286` | `0.5638063820269908` | `0.2215916179730092` | `PASS` |
| `left_hip` | `-0.349066` | `2.268928` | `0.0` | `0.6872899218875758` | `0.349066` | `PASS` |
| `left_knee` | `0.0` | `2.443461` | `0.0` | `1.4185266741631963` | `0.0` | `PASS` |
| `left_mtp` | `-0.523599` | `1.570796` | `-0.0017059553197663773` | `0.07869167394701632` | `0.5218930446802337` | `PASS` |
| `right_ankle` | `-0.959931` | `0.785398` | `-0.059843713921714406` | `0.5638063820269948` | `0.2215916179730052` | `PASS` |
| `right_hip` | `-0.349066` | `2.268928` | `0.0` | `0.687289921887574` | `0.349066` | `PASS` |
| `right_knee` | `0.0` | `2.443461` | `0.0` | `1.418526674163196` | `0.0` | `PASS` |
| `right_mtp` | `-0.523599` | `1.570796` | `-0.0017059553197663906` | `0.07869167394700863` | `0.5218930446802337` | `PASS` |
| `trunk_pelvis` | `-0.610865` | `0.610865` | `-0.03587557739353446` | `0.5422164233170099` | `0.06864857668299007` | `PASS` |

### trunk_pelvis (the known binding channel)

* frozen upper bound: `0.610865` rad
* measured maximum: `0.5422164233170099` rad
* minimum ROM margin: `0.06864857668299007` rad (worst side `upper`, sample `566`, phase `PROPULSION`)
* at the worst sample: qdot `0.16768783578481739` rad/s, applied `-170.78923326944275` N*m, reference `0.4662876820516547` rad

## Hard actuation conformance

* torque-rate role: `NOMINAL_SLEW_BOUND`
* hard safety bounds: `['moment_ceiling', 'joint_power_ceiling', 'mtp_energy_gate']`
* max applied moment (N*m): `[220.0, 258.1906155230471, 258.1906155230471, 380.0, 380.0, 250.05510912602026, 250.05510912603694, 15.0, 15.0]`
* max joint power (W): `[600.0, 1002.0766113028352, 1002.0770204872505, 1700.0000000000002, 1700.0000000000002, 1100.0000000000002, 1100.0, 98.44859731287245, 98.44859731287207]`
* max bilateral asymmetry (N*m): `1.3073986337985843e-12` (RES-85 claim domain; the post-claim landing window is RES-86 scope)
* conformance `PASS`; safety override audit `PASS` with `54` declared override sample rows and no undeclared slew exceedance

## MTP non-compensation

* MTP energy report status: `PASS`
* matrix conclusion: `the accepted launch does not depend on replacing removed passive MTP mechanics with large active toe work`
* zero-passive + zero-active case confirmed: `True` (H2 = `0.12816367328967782` m; reported honestly as a sensitivity observation, below the canonical functional floor)
* active MTP positive work ratio (nominal): `0.0014469610529035438`

## Negative controls

| Control | Result |
|---|---|
| `NC-01_single_sample_dropout_refuses_permanent_flight` | PASS |
| `NC-01b_dropout_truncated_stream_refuses_flight` | PASS |
| `NC-02_five_true_comparator_samples_reject` | PASS |
| `NC-02b_six_true_samples_confirm_comparator_only` | PASS |
| `NC-02c_comparator_never_defines_takeoff_or_flight` | PASS |
| `NC-03_recontact_before_dwell_rejects_confirmation` | PASS |
| `NC-04_one_foot_contact_is_not_physical_takeoff` | PASS |
| `NC-05_insufficient_clearance_never_confirms_flight` | PASS |
| `NC-06_prohibited_contact_rejects_confirmation` | PASS |
| `NC-07_nonpositive_occurrence_vz_rejects` | PASS |
| `NC-08_authority_rate_and_moment_limits_hold` | PASS |
| `NC-09_previous_applied_is_sole_history` | PASS |
| `NC-10_bilateral_symmetry_projection_holds` | PASS |
| `zero_passive_active_work_within_budget` | PASS |
| `zero_passive_unbounded_command_is_budget_gated` | PASS |
| `zero_passive_episode_closes_or_reports_degradation` | PASS |
| `NC-12_startup_support_dropout_as_accepted_occurrence_fails` | PASS |
| `NC-13_top_level_occurrence_differs_from_h2_origin_fails` | PASS |
| `NC-14_comparator_offset_uses_rejected_occurrence_fails` | PASS |
| `NC-15_h2_just_below_functional_floor_fails` | PASS |
| `NC-16_zero_passive_zero_active_case_not_replaced_by_active_command` | PASS |
| `NC-17_undeclared_slew_exceedance_fails` | PASS |
| `NC-18_declared_safety_override_without_binding_bound_fails` | PASS |
| `NC-19_mtp_active_work_is_negligible_fraction_of_joint_work` | PASS |
| `NC-20_canonical_identity_audit_passes` | PASS |
| `NC-21_safety_override_audit_passes` | PASS |
| `NC-22C_strict_structural_rom_audit_passes` | PASS |
| `NC-22_measured_trunk_beyond_upper_bound_fails` | PASS |
| `NC-23_reference_inside_but_measured_outside_fails` | PASS |
| `NC-24_measured_outside_rom_inside_probe_envelope_fails` | PASS |
| `NC-25_rom_guard_never_drives_farther_outside` | PASS |
| `NC-26_strict_rom_correction_below_functional_floor_fails_closure` | PASS |
| `NC-27_boundary_within_tolerance_passes` | PASS |

## Test and suite results

* full repository suite (one run, candidate worktree at ENTRY_HEAD plus the RES-85D changes): collected `912`, passed `845`, failed `60`, skipped `7`, errors `0`; excluded collection-error modules `2` (`tests/test_ml241_qacc_resolution.py`, `tests/test_public_support_wrench_contract.py`)
* RES-85D-owned failures: `0`; RES-85C-owned: `0`; RES-85-owned: `0`; RES-83/RES-84: `0`; every recorded failure is classified `PRE_EXISTING_*`
* targeted qualification: `{'tests/test_res83_v3_plant.py': 'PASS', 'tests/test_res84_v3_measurement_contact.py': 'PASS', 'tests/test_res85_authority_freeze.py': 'PASS', 'tests/test_res85_v3_causal_launch.py': 'PASS', 'tests/test_res85c_correction.py': 'PASS', 'tests/test_res85d_strict_rom.py': 'PASS'}`; collected `222`, passed `222`, failed `0`, skipped `0`
* entry-head reproduction: `60` of `60` candidate failures reproduced node-for-node at ENTRY_HEAD with the same controlling cause; no failing test file imports the RES-85D change surface `loaded_cmj.v3`

## Determinism

* the bundle is built twice and compared over the declared identity domain;
  the two-build byte-identity result and per-artifact sha256 live in
  `DETERMINISM_REPORT.json`; the telemetry blob and canonical digest live in
  `TELEMETRY_MANIFEST.json`.
* telemetry canonical digest: `37e83c46241121d2d4bd7ba9b904a539b2a578c2fe64f4b8597c597d7c8365d3`
* the declared search (if any) is run twice and compared over its own
  canonical payload (`STRICT_ROM_SEARCH.json`).
* evidence seal: `HASH_MANIFEST.json` covers every sealed artifact of this
  bundle and the telemetry blob; the seal is written by the same deterministic
  build that produces this receipt.

## Authority amendments (RES-85 history, unchanged)

* `AMD-01` (ACTUATION_AUTHORITY.json): The enforcement chain is declared as a projection onto the intersection of four intervals (torque-rate around the previous applied moment, net-moment ceiling, joint-power ceiling, MTP budget cap) instead of a sequential cascade, and the declared precedence for an empty intersection (moment/power/MTP limit over smoothness, recorded as power_emergency or moment_emergency) is added.
* `AMD-02` (ACTUATION_AUTHORITY.json): The applied-moment symmetry tolerance is stated as 1e-9 N*m (was 1e-12 N*m), because the per-channel constraint projection may introduce a bounded numerical asymmetry after the symmetric-command projection.
* verdict: AMENDMENTS ARE ENFORCEMENT-SEMANTICS AND TOLERANCE-STATEMENT CORRECTIONS; NO CEILING, BUDGET, EVENT OR PHASE-PREDICATE WAS RELAXED

## Final claim ceiling

RES-85D closes the causal loaded-CMJ launch and flight (takeoff, confirmation,
genuine 50 ms physical-time flight, apex/H2) against the frozen RES-83 Plant,
RES-84 measurement authority and RES-85 control authority, and additionally
requires every measured bounded joint coordinate to remain inside the frozen
human structural envelope from the first native sample through the RES-85 claim
end.  It claims no landing capture, no recovery, no elite performance norm and
no successor candidate identity; the functional floor is a non-triviality
boundary, not a performance claim.

`NEXT_AUTHORIZED_ACTION=RES86_ACTIVE_SET_SAFE_LANDING_CAPTURE`
(successor work is not started by RES-85D).
