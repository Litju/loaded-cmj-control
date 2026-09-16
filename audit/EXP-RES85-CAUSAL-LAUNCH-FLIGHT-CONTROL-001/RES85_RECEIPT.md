# RES85_RECEIPT — V3 causal loaded-CMJ launch and flight control

MISSION: `RES85_REBUILD_CAUSAL_LOADED_CMJ_LAUNCH_FLIGHT_CONTROL_001`
LINEAR ISSUE: RES-85
STATUS: **PASS**

## Episode

* status: `COMPLETED`
* phases: `STAND -> COUNTERMOVEMENT -> BRAKING -> PROPULSION -> TAKEOFF_CONFIRM -> PROPULSION -> TAKEOFF_CONFIRM -> PROPULSION -> TAKEOFF_CONFIRM -> PROPULSION -> TAKEOFF_CONFIRM -> FLIGHT -> LANDING_PREP`
* takeoff confirmation: `True`
* takeoff vz: `0.782225093881675` m/s
* H2 (DIRECT_SIMULATOR_SYSTEM_COM): `0.030166215950979014` m

## Method-explicit comparator reporting

| Method | Status | Value |
|---|---|---|
| DIRECT_SIMULATOR_SYSTEM_COM | PRIMARY_CANONICAL | 0.030166215950979014 m |
| BALLISTIC_HEIGHT_FROM_TAKEOFF_VZ | SECONDARY_CROSS_CHECK | 0.031186345438236257 m |
| FORCE_PLATFORM_IMPULSE_MOMENTUM | CROSS_CHECK | see impulse_cross_check |
| FORCE_PLATFORM_FLIGHT_TIME | REPORT_ONLY | see flight time entry |
| BAR_LVT_DISPLACEMENT_VELOCITY | NOT_APPLICABLE_NATIVE | no tether instrument |

`ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED`. The historical 0.150 m value is
retained only as a labelled anti-triviality negative-control magnitude and is never
used as an elite target, band or performance floor.

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

## Actuation and MTP authority conformance

| Check | Result |
|---|---|
| `moment_ceiling_never_exceeded` | PASS |
| `torque_rate_ceiling_respected_except_declared_emergencies` | PASS |
| `joint_power_ceiling_never_exceeded` | PASS |
| `bilateral_applied_symmetry_within_tolerance` | PASS |
| `phase_handoffs_respect_authority` | PASS |
| `mtp_ledger_identity_exact` | PASS |
| `mtp_work_budgets_respected` | PASS |
| `AEI-1e_active_mtp_zero_after_takeoff_confirmation` | PASS |
| `AEI-1f_ankle_positive_work_dominates_active_mtp` | PASS |

## Repository suite classification

* collected `859`, passed `834`, failed `25`, collection-error modules `2`
* RES-85-owned failures: `0`; RES-83/RES-84 failures: `0`
* every recorded failure reproduces at ENTRY_HEAD (`b0eccb8a6eeda40950665b3be517854f8b48baab`); see `FULL_SUITE_CLASSIFICATION.json`

## Authority amendments during Achievement B

* `AMD-01` (ACTUATION_AUTHORITY.json): The enforcement chain is declared as a projection onto the intersection of four intervals (torque-rate around the previous applied moment, net-moment ceiling, joint-power ceiling, MTP budget cap) instead of a sequential cascade, and the declared precedence for an empty intersection (moment/power/MTP limit over smoothness, recorded as power_emergency or moment_emergency) is added.
* `AMD-02` (ACTUATION_AUTHORITY.json): The applied-moment symmetry tolerance is stated as 1e-9 N*m (was 1e-12 N*m), because the per-channel constraint projection may introduce a bounded numerical asymmetry after the symmetric-command projection.
* verdict: AMENDMENTS ARE ENFORCEMENT-SEMANTICS AND TOLERANCE-STATEMENT CORRECTIONS; NO CEILING, BUDGET, EVENT OR PHASE-PREDICATE WAS RELAXED

## Scope

RES-85 implements and qualifies causal launch/flight control only. No landing
optimisation (RES-86), no recovery (RES-87), no successor candidate identity and no
elite H2 target are claimed or created here.
