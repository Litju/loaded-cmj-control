# RES85_RECEIPT — V3 causal loaded-CMJ launch and flight control (RES-85C)

MISSION: `RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001`
LINEAR ISSUE: RES-85
STATUS: **PASS**

## Episode

* status: `COMPLETED`
* phases: `STAND -> COUNTERMOVEMENT -> BRAKING -> PROPULSION -> TAKEOFF_CONFIRM -> PROPULSION -> TAKEOFF_CONFIRM -> FLIGHT -> LANDING_PREP`
* accepted occurrence: sample `666` / t=`1.332` s
* takeoff confirmation: `True` at sample `691`
* takeoff vz: `1.9503826924187873` m/s
* H2 (DIRECT_SIMULATOR_SYSTEM_COM): `0.18867197309831352` m
* H_ANTI_TRIVIALITY_FLOOR classification: `ABOVE_FLOOR`
* launch joint ROM audit (STAND -> occurrence): `PASS`

## Method-explicit comparator reporting

| Method | Status | Value |
|---|---|---|
| DIRECT_SIMULATOR_SYSTEM_COM | PRIMARY_CANONICAL | 0.18867197309831352 m |
| BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ | SECONDARY_CROSS_CHECK | 0.19388341727251565 m |
| FORCE_PLATFORM_IMPULSE_MOMENTUM | CROSS_CHECK | see impulse_cross_check |
| FORCE_PLATFORM_FLIGHT_TIME | REPORT_ONLY | see flight time entry |
| BAR_LVT_DISPLACEMENT_VELOCITY | NOT_APPLICABLE_NATIVE | no tether instrument |

`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` and
`ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED`.  The declared
`H_ANTI_TRIVIALITY_FLOOR = 0.150 m` is a hard functional
non-triviality boundary: it is not an elite norm, not an expected
value and not an optimization target, but it is a minimum functional
success condition (diagnostic scale cross-check: minimum ballistic
`vz = sqrt(2 g 0.150) ~= 1.716 m/s`).

## Mandatory negative controls (deterministic)

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
| `NC-22_launch_joint_rom_audit_passes` | PASS |

## Actuation and MTP authority conformance

| Check | Result |
|---|---|
| `hard_moment_ceiling_never_exceeded` | PASS |
| `hard_power_ceiling_never_exceeded` | PASS |
| `nominal_slew_exceedance_zero_unless_safety_override` | PASS |
| `bilateral_applied_symmetry_within_tolerance` | PASS |
| `phase_handoffs_do_not_create_hidden_slew_violations` | PASS |
| `mtp_ledger_identity_exact` | PASS |
| `mtp_work_budgets_respected` | PASS |
| `AEI-1e_active_mtp_zero_after_takeoff_confirmation` | PASS |
| `AEI-1f_ankle_positive_work_dominates_active_mtp` | PASS |
| `safety_override_audit` | PASS |

## Repository suite classification

* collected `890`, passed `823`, failed `60`, skipped `7`, collection-error modules excluded `2`
* RES-85C-owned failures: `0`; RES-85-owned failures: `0`; RES-83/RES-84 failures: `0`
* pre-existing failures are classified and reproduced at ENTRY_HEAD; see `FULL_SUITE_CLASSIFICATION.json`

## Authority amendments during Achievement B (RES-85 history)

* `AMD-01` (ACTUATION_AUTHORITY.json): The enforcement chain is declared as a projection onto the intersection of four intervals (torque-rate around the previous applied moment, net-moment ceiling, joint-power ceiling, MTP budget cap) instead of a sequential cascade, and the declared precedence for an empty intersection (moment/power/MTP limit over smoothness, recorded as power_emergency or moment_emergency) is added.
* `AMD-02` (ACTUATION_AUTHORITY.json): The applied-moment symmetry tolerance is stated as 1e-9 N*m (was 1e-12 N*m), because the per-channel constraint projection may introduce a bounded numerical asymmetry after the symmetric-command projection.
* verdict: AMENDMENTS ARE ENFORCEMENT-SEMANTICS AND TOLERANCE-STATEMENT CORRECTIONS; NO CEILING, BUDGET, EVENT OR PHASE-PREDICATE WAS RELAXED

## Scope

RES-85 implements and qualifies causal launch/flight control only. No landing
optimisation (RES-86), no recovery (RES-87), no successor candidate identity and no
elite H2 target are claimed or created here.
