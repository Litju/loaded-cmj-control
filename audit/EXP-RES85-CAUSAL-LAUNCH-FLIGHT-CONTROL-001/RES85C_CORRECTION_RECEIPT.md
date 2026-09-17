# RES85C_CORRECTION_RECEIPT

MISSION: `RES85C_NARROW_POSTSEAL_CORRECTION_AND_LAUNCH_REQUALIFICATION_001`
LINEAR ISSUE: RES-85
ENTRY_HEAD: `e487369f6861d9c9bc27f9f3d92b981fb3684293`
ENTRY_TREE: `5a88417d3a352ec87fee498e8a548575f19d3e0f`
PREVIOUS_RES85_EVIDENCE_SEAL_SHA256: `235a40e7a2a2b07ea483f2352b0765315b7f43b2343b3297461a0754b024b50e`
RES84_EVIDENCE_SEAL_SHA256: `223b13fe5bc3b884c15750da6badd24c834cfec54a24a23338dfde25d1bcbeda`
RES83_PLANT_XML_SHA256: `eca5760fbd5d93e7e99ae657287e94560a996e8b888f9e65d6155cb2c6d91e2d`

The previous RES-85 (e487369) evidence remains attributable and is NOT
silently rewritten: its bundle files are the git blobs at that commit and
its external seal is recorded above.  Its episode is reproduced here as
historical pre-correction evidence with the honest classification
`PHYSICALLY_VALID_CAUSAL_TAKEOFF_AND_FLIGHT_BELOW_FUNCTIONAL_TASK_FLOOR` (H2 = `0.030166215950979014` m,
takeoff vz = `0.782225093881675` m/s).

## Entry reconciliation

* `HEAD` == `ENTRY_HEAD`, `HEAD^{tree}` == `ENTRY_TREE` and `HEAD` == `origin/main`
  were verified before any write; the RES-83 Plant XML hash and the RES-84 and
  previous RES-85 external seals were re-verified.
* the worktree carried the interrupted prior execution artifacts of this same
  RES-85C mission.  They were audited, completed and re-verified against the
  frozen authorities; this bundle is delivered by the single RES-85C correction
  commit.  No foreign or out-of-mission change was present.

## Blocker corrections

### BLOCKER A — accepted takeoff identity

* before: the exported top-level TAKEOFF_OCCURRENCE was the first support-to-zero
  transition (startup contact chatter, sample 3 / t=0.006 s) while the launch
  actually used sample 767 / t=1.534 s (confirmation 792 / t=1.584 s).
* correction: `extract_events` now exports the unique controller-confirmed
  occurrence as the top-level TAKEOFF_OCCURRENCE; the confirmation, the H2 origin,
  the ballistic cross-check, the impulse cross-check and the diagnostic-comparator
  offsets all reference that same occurrence; every candidate remains in
  `takeoff_candidate_history` with source phase, disposition, rejection reason and
  confirmation result.
* canonical accepted occurrence: sample `666` /
  t=`1.332` s; candidate history entries: `5`
  (dispositions: `{'NOT_EVALUATED': 3, 'REJECTED': 1, 'ACCEPTED': 1}`).
* identity audit status: `PASS`.

### BLOCKER B — functional floor authority

* `ELITE_SOCCER_PLUS20_H2_HARD_GATE = NOT_ESTABLISHED` (unchanged)
* `ELITE_SOCCER_PLUS20_H2_TARGET = NOT_ESTABLISHED` (restored)
* `H_ANTI_TRIVIALITY_FLOOR = 0.15` m with role
  `HARD_FUNCTIONAL_NONTRIVIALITY_NEGATIVE_CONTROL_BOUNDARY`; explicit statements:
  0.150 m is NOT an elite-performance norm, NOT an optimization target,
  and IS a minimum functional success condition.
* diagnostic scale cross-check only: minimum ballistic vz =
  `1.715517` m/s (never a replacement for direct SYSTEM_COM H2).
* corrected episode classification: `ABOVE_FLOOR`
  at H2 = `0.18867197309831352` m.
* pre-correction episode: `PHYSICALLY_VALID_CAUSAL_TAKEOFF_AND_FLIGHT_BELOW_FUNCTIONAL_TASK_FLOOR` (not relabelled PASS).

### BLOCKER C — actuation-rate contract

* torque-rate limit is declared `NOMINAL_SLEW_BOUND` (it is not, and is no longer
  reported as, a hard ceiling).
* moment / joint-power / MTP energy gates are the `HARD_SAFETY_BOUNDS`: they are
  enforced every sample and never exceeded.
* when a changing hard safety bound empties the slew-feasible interval the hard
  bound wins and the sample is recorded as an explicit SAFETY_OVERRIDE naming the
  single binding hard constraint and the violated nominal slew margin.
* canonical episode: undeclared slew exceedances = 0; declared override
  samples = `17` sample rows
  (`22` channel samples);
  every override is attributed to exactly one binding hard safety bound;
  audit status `PASS`.
* no numeric moment, power or MTP work budget was increased.

### BLOCKER D — MTP non-compensation

* deterministic matrix (M0..M3 + declared sweeps) in `MTP_NONCOMPENSATION_MATRIX.json`.
* M0 (nominal passive + nominal active): H2 = `0.18867197309831352` m,
  active MTP positive work = `0.5438341837352594` J,
  ratio to total positive joint work = `0.0010326298249116917`.
* M1 (zero passive + zero active): H2 = `0.1770700421898903` m, active MTP work =
  `0.0` J, confirmation =
  `True`.
* M3 (zero passive + nominal active): H2 = `0.18057090654359564` m.
* conclusion: the accepted launch does not depend on replacing removed passive MTP mechanics with large active toe work.
* active MTP positive work remains a negligible fraction of total positive joint
  work; the ankle remains the dominant contributor.

## Canonical result

| Quantity | Value |
|---|---|
| H2 (direct SYSTEM_COM) | `0.18867197309831352` m |
| takeoff vz | `1.9503826924187873` m/s |
| accepted occurrence | sample `666` / t=`1.332` s |
| confirmation | sample `691` / t=`1.3820000000000001` s |
| ballistic cross-check residual | `-0.0052114441742021345` m |
| functional floor classification | `ABOVE_FLOOR` |
| propulsion stroke | `0.19115047876042235` m of `0.21908418869200108` m budget |
| launch joint ROM audit | `PASS` (reference `True`, drive `True`, envelope `True`) |

## Accepted occurrence identity

| candidate | t (s) | source phase | disposition | rejection reason | confirmation sample | confirmed |
|---|---|---|---|---|---|---|
| `5` | `0.01` | `None` | `NOT_EVALUATED` | `NEVER_ESCALATED_TO_TAKEOFF_CONFIRM` | `30` | `False` |
| `658` | `1.316` | `PROPULSION` | `REJECTED` | `LEGAL_RECONTACT_BEFORE_CONFIRMATION` | `683` | `False` |
| `666` | `1.332` | `PROPULSION` | `ACCEPTED` | `None` | `691` | `True` |
| `964` | `1.928` | `None` | `NOT_EVALUATED` | `NEVER_ESCALATED_TO_TAKEOFF_CONFIRM` | `989` | `False` |
| `969` | `1.938` | `None` | `NOT_EVALUATED` | `NEVER_ESCALATED_TO_TAKEOFF_CONFIRM` | `994` | `False` |

## MTP non-compensation sensitivity matrix

| case | zero-passive | active budget (J) | MTP moment (N*m) | takeoff | H2 (m) | active MTP (J) | ratio active/total | budget binds | qualitative success |
|---|---|---|---|---|---|---|---|---|---|
| `M0_nominal_passive_nominal_active` | `False` | `None` | `None` | `True` | `0.18867197309831352` | `0.5438341837352594` | `0.0010326298249116917` | `True` | `True` |
| `M1_zero_passive_zero_active` | `True` | `0.0` | `None` | `True` | `0.1770700421898903` | `0.0` | `0.0` | `True` | `True` |
| `M2_zero_passive_tight_active` | `True` | `2.5` | `None` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `M3_zero_passive_nominal_active` | `True` | `None` | `None` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_active_budget_0.271917J` | `True` | `0.27191709186763124` | `None` | `True` | `0.18781836251514972` | `0.5438341837352625` | `0.001028258207082893` | `True` | `True` |
| `SWEEP_active_budget_0J` | `True` | `0.0` | `None` | `True` | `0.1770700421898903` | `0.0` | `0.0` | `True` | `True` |
| `SWEEP_active_budget_12.5J` | `True` | `12.5` | `None` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_active_budget_2.5J` | `True` | `2.5` | `None` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_active_budget_25J` | `True` | `25.0` | `None` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_moment_normal_passive_0Nm` | `False` | `None` | `0.0` | `True` | `0.19901148878984198` | `0.0` | `0.0` | `True` | `True` |
| `SWEEP_moment_normal_passive_22.5Nm` | `False` | `None` | `22.5` | `True` | `0.18867197309831352` | `0.5438341837352594` | `0.0010326298249116917` | `True` | `True` |
| `SWEEP_moment_normal_passive_45Nm` | `False` | `None` | `45.0` | `True` | `0.18867197309831352` | `0.5438341837352594` | `0.0010326298249116917` | `True` | `True` |
| `SWEEP_moment_normal_passive_60Nm` | `False` | `None` | `60.0` | `True` | `0.18867197309831352` | `0.5438341837352594` | `0.0010326298249116917` | `True` | `True` |
| `SWEEP_moment_zero_passive_0Nm` | `True` | `None` | `0.0` | `True` | `0.1770700421898903` | `0.0` | `0.0` | `True` | `True` |
| `SWEEP_moment_zero_passive_22.5Nm` | `True` | `None` | `22.5` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_moment_zero_passive_45Nm` | `True` | `None` | `45.0` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |
| `SWEEP_moment_zero_passive_60Nm` | `True` | `None` | `60.0` | `True` | `0.18057090654359564` | `3.2357833435465206` | `0.0054819312889644424` | `True` | `True` |

## Test and suite results

* targeted RES-85/RES-85C/Plant/measurement: `{'tests/test_res83_v3_plant.py': 'PASS', 'tests/test_res84_v3_measurement_contact.py': 'PASS', 'tests/test_res85_authority_freeze.py': 'PASS', 'tests/test_res85_v3_causal_launch.py': 'PASS', 'tests/test_res85c_correction.py': 'PASS'}`
* full repository suite: `{'collected': 890, 'collection_error_modules_excluded': 2, 'failed': 60, 'passed': 823, 'res83_res84_failures': 0, 'res85_owned_failures': 0, 'res85c_owned_failures': 0, 'skipped': 7}`
* pre-existing failure reproduction: `{'primary_checkout_head': 'e487369f6861d9c9bc27f9f3d92b981fb3684293', 'recorded_pre_res85_reproduction': {'path_dependent_explanation': 'the test resolves the external evidence root relative to the repository parent; it fails in the temporary worktree and passes in the primary checkout where the evidence root exists', 'path_dependent_test': 'tests/test_v2_1_res16_true_standing.py::test_01_captured_satisfies_old_but_fails_new', 'same_failures_reproduced': 24, 'worktree_head': 'b0eccb8a6eeda40950665b3be517854f8b48baab'}, 'res85c_reproduction': {'failures_reproduced_node_for_node': 60, 'note': 'no failing test imports the RES-85C change surface loaded_cmj.v3', 'of_which_previously_recorded': 25, 'of_which_untracked_owner_or_absent_dependency': 35, 'untracked_owner_test_files_copied_verbatim': ['tests/test_progressive_braking_controller.py', 'tests/test_hip_braking_polarity.py', 'tests/test_knee_rate_feasibility.py', 'tests/test_res51_centroidal_landing.py', 'tests/test_res52_soft_contact.py']}, 'worktree_head': 'e487369f6861d9c9bc27f9f3d92b981fb3684293'}`
* failure categories: `{'PRE_EXISTING_EXTERNAL_EVIDENCE_CONTRACT_ABSENT': 2, 'PRE_EXISTING_EXTERNAL_SESSION_ARTIFACT_ABSENT': 17, 'PRE_EXISTING_EXTERNAL_SOLVER_BACKEND_ABSENT': 3, 'PRE_EXISTING_LEGACY_ORACLE_TRANSCRIPTION_IDENTITY': 3, 'PRE_EXISTING_LEGACY_V1_QUALIFICATION': 1, 'PRE_EXISTING_LEGACY_V2_TRAJECTORY_IDENTITY': 9, 'PRE_EXISTING_OWNER_CONTROLLER_INTERFACE_MISMATCH': 25}`

## Controller engineering variables (bounded declared search)

`PROPULSION_SEARCH.json` records the predeclared staged coordinate-ascent
grid (`a_thrust_m_s2` x `thrust_az_max_m_s2`, `extension_rate_ff_gain` x
`contact_preload_m`, `trunk_lean_frac` x `trunk_extend_frac`, then the
`a_thrust_m_s2` x `thrust_az_max_m_s2` refinement stage), the fixed declared
ROM/trunk setup, the declared ordering, the evaluation function, the 33
evaluation cap and the two-run byte-identity result.  Selected configuration:

```json
{
  "a_brake_m_s2": 2.5,
  "a_thrust_m_s2": 17.0,
  "braking_trigger_margin_m": 0.02,
  "contact_preload_m": 0.005,
  "dt_s": 0.002,
  "extension_rate_ff_gain": 0.5,
  "joint_rom_barrier_gain": 800.0,
  "joint_rom_margin_rad": 0.08,
  "mtp_active_budget_j": null,
  "mtp_moment_ceiling_nm": null,
  "quiet_stand_samples": 125,
  "reference_lag_max_m": 0.05,
  "thrust_az_max_m_s2": 21.0,
  "trunk_extend_frac": 0.4,
  "trunk_kd": 30.0,
  "trunk_kp": 240.0,
  "trunk_lean_frac": 0.35,
  "trunk_rom_barrier_gain": 2500.0,
  "v_brake_trigger_m_s": -0.55,
  "v_countermovement_cmd_m_s": -0.35
}
```

## Actuation semantics

* role of the rate limit: `NOMINAL_SLEW_BOUND`
* hard safety bounds: `['moment_ceiling', 'joint_power_ceiling', 'mtp_energy_gate']`
* max applied moment (N*m): `[220.0, 142.42206771371013, 142.42206771371013, 353.3330706569493, 353.3330706569493, 92.36577584021637, 92.36577584021637, 15.0, 15.0]`
* max joint power (W): `[600.0, 878.7597684593119, 878.759768459416, 1700.0, 1700.0000000000002, 1100.0, 1100.0, 98.44859731287245, 98.44859731287207]`
* final status: conformance `PASS`, override audit `PASS`, negative controls `PASS`

## Evidence

* artifacts regenerated deterministically; `DETERMINISM_REPORT.json` records the
  declared identity domain and the two-build byte-identity result.
* `HASH_MANIFEST.json` covers every artifact and the telemetry blob.

## Final claim ceiling

RES-85C closes the causal loaded-CMJ launch and flight (takeoff, confirmation,
genuine 50 ms physical-time flight, apex/H2) against the frozen RES-83 Plant,
RES-84 measurement authority and RES-85 control authority.  It claims no landing
capture, no recovery, no elite performance norm and no successor candidate
identity; the functional floor is a non-triviality boundary, not a performance
claim.
