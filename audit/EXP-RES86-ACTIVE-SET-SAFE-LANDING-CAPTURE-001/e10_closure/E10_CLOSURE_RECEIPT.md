# RES-86 PRODUCTION E10 CLOSURE RECEIPT

MISSION=RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR_ISSUE=RES-86
EXPERIMENT_ID=EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001
STATUS=E10 NOT PASSED (bounded qualification failure; single controlling causal
blocker reported and work stopped per the mission failure clause)

## Entry and contact realization

- ENTRY_HEAD=81afd5ec91d5065702030f975c83bf2022191deb
  ENTRY_TREE=2b9752af78da9acb47cac5ec4f4fe604b85eb951
- The already-qualified contact-candidate commit `86b2752` (runtime `mjContact`
  realization: declared plantar `solref` (0.01, 1.0), realized (0.015, 1.0);
  `condim` 4, `solimp` (0.9, 0.95, 0.001, 0.5, 2.0), sliding friction 0.9,
  `includemargin` 0.0; refsafe enabled, solver semantics untouched) was pushed
  unchanged before any new work; `HEAD == origin/main` at that point.
- Every E10 attempt below runs on that qualified candidate realization through
  the production runtime (`run_landing_episode`, candidate default), with the
  post-apex LANDING_PREP handoff, the active MTP channel inside the frozen
  budget and exact branch validation.

## What was executed

1. **E10-aware optimization and validation** (commit `14d35fb`): the frozen
   E10 point gates and first-contact-to-E10 transient envelopes drive the
   exact-branch oracle ranking and the executed-trace validation from the
   start; the controller clips its contraction targets to the point gates and
   latches the first live envelope violation so E10 can never be confirmed from
   a landing that left the frozen window.  118 RES-86 tests pass.
2. **Declared preparation search** (development instrument, cached sealed
   handoff): full posture gains and a flat-ankle target move the touchdown from
   native sample 791 / com vz -1.729 m/s to sample 797 / com vz -1.847 m/s with
   a flat foot, and the declared independent knee reach moves the contact
   toward under the COM (sample 777 / com vz -1.455 m/s, contact x - COM x
   -0.049 m) without decoupling the impact angular impulse.
3. **E10-aware bounded oracle searches** on two preparation-shaped branches
   (structured witness, extended with declared Hy CoP/ankle feedback; budgets
   400/1200/1500).  The best search result sustains the bilateral-loaded
   `abs(com_vz) < 0.05` predicate for 0.048 s of the required 0.050 s, arrests
   the descent (terminal |vz| 3.5e-4), keeps both feet loaded (min fz 267 N)
   and penetrates 9.39 mm — but only with window max |Hy| 20.27 vs the frozen
   5.0 envelope, and with root-pitch/rate and vx exceedances.
4. **Canonical qualification** (`qualify_e10_closure.py`, production runtime,
   2 declared preparation candidates x 2 executions each, bit-identical
   telemetry digests):

| candidate | E8 contact | window max abs Hy | first envelope violation | fault | E10 |
|---|---|---|---|---|---|
| best_effort_touchdown | t=1.594, vz -1.847 | **4.908** (native 797..819) | E10_WINDOW_HY at t=1.640 | SUPPORT_RETENTION | not reached |
| forward_reach_sensitivity | t=1.554, vz -1.455 | 9.987 (native 777..789) | E10_WINDOW_HY at t=1.562 | MAX_PENETRATION | not reached |

   The best-effort run holds every frozen window envelope (max |Hy| 4.908 < 5.0,
   max |vx| 0.136 < 0.40, root pitch 0.176 < 0.45, rates within limits,
   penetration 7.00 mm < 10 mm, peak Fz 2.64 BW < 8 BW) for the executed trace
   and then leaves the Hy envelope at the following native sample, where the
   same solve has no support-retaining validated action (the fallback itself
   fails `SUPPORT_RETENTION`) and the controller fails closed.

## Single controlling causal blocker

**The frozen first-contact-to-E10 envelope `abs(Hy) <= 5 kg m^2/s` is the
first-binding gate of the E10 window, and it is not simultaneously satisfiable
with the frozen penetration and support-retention gates under the sealed
RES-85 flight handoff and the one preparation law.**

Mechanism, from the recorded evidence:

- Hy at physical first contact is 2.947 and is fixed by the sealed flight
  handoff: free-flight centroidal angular momentum is conserved and no internal
  joint action can change it.  The entire impact may therefore add at most
  ~2.05 units of angular impulse about the SYSTEM_COM.
- The vertical arrest requires ~150-175 N*s of vertical impulse inside the
  frozen 10 mm penetration envelope.  Absorbing it with the support polygon
  behind the COM (every admissible touchdown lands with contact x - COM x
  between -0.155 m and -0.007 m) puts a positive (backward-pitching) lever on
  that impulse; a toes-down foot moves the contact under the COM but its
  angled normal injects a horizontal impulse (~0.4-0.7 of Fz in the first
  samples; the sealed branch's contact normal is ~39 deg from vertical).
- Within that support polygon the friction force is one-directional: a rearward
  force would move the CoP forward of the support, so the controller cannot
  cancel the injected angular impulse without unloading the feet — which the
  frozen `SUPPORT_RETENTION` gate rejects (the best-effort canonical run fails
  exactly there, at the same sample where Hy leaves the envelope).
- Bounded evidence: from the sealed E8 state every contact-maintaining constant
  action over a declared knee/hip/ankle grid gives 5-sample peak |Hy| >= 6.18;
  the E10-aware bounded searches reach the dwell and the vertical arrest only
  with window max |Hy| 18.9-20.3.

Never promoted: a bounded search is **not** a global infeasibility proof.  This
receipt reports a bounded qualification failure with the first-binding envelope
and its causal mechanism identified; it does not claim that no trajectory
exists, and it does not weaken any frozen gate to manufacture a pass.

## Boundaries preserved

- No Plant topology, contact-realization, solver, ROM or authority change.
- MuJoCo time constant remains numerically resolved (realized 0.015 s at
  dt = 0.001/0.002/0.004); `refsafe` and every solver semantic untouched.
- No vertical-arrest witness is accepted as a landing witness: the vertical
  arrest found by the optimizer is explicitly not E10 (bad CAM/posture).
- No RES-87 work.  No E11/E12 claims.
- Rollouts: 8 of the 20 new canonical runtime landing episodes used; 83
  development evaluations reused one cached sealed post-apex handoff.

## Artifacts (sha256)

- `E10_CLOSURE_JOURNAL.json` `687f56315e6f54c996a851efebff863d4af41c08e92284a237513f93592b54c7`
- `E10_CLOSURE_QUALIFICATION.json` `3a6f4158a78d853f8885b7cf3cb93b7a666428cd8cfbafc67ecc3992e2d74447`
- `ORACLE_E10_SEARCH_SHAPE_A_400.json` `c24531264e3a4f6017e231d8b63a0636e2a99b11c0215ad342c9511c8a4f7006`
- `ORACLE_E10_SEARCH_SHAPE_A_1200.json` `27c02960c1a87229655e3a7049edefbb276408e16cb31eea50bec5b61175f1ec`
- `ORACLE_E10_SEARCH_SHAPE_A_HYFEEDBACK_1500.json` `190f0065764de73a76202f5b79a1ddd8b93ac760aaf2f3af806c9ee3c86a23a9`
- `tools/res86/qualify_e10_closure.py` `99a7c01cc453d353024d16898f9e40735c33405d728355d017ca74506cc3a97d`
- `src/loaded_cmj/v3/landing_control.py` `883a594e529da95d33eb00fdc5eaf53ed6bd3e8e223ee7e619ce0e2eb7a7fff4`
- `src/loaded_cmj/v3/landing_runtime.py` `1ce9f3e80d6004c55c1a4986f40fc26219a4c3cf6cbc71387dd661dd93b8f070`
- `tools/res86/landing_feasibility_oracle.py` `9bfff4e8f0fe9321d89c170cad6b0fcaf5a2f052dda2bee84fa039c8995c3c39`
- `tests/test_res86_e10_awareness.py` `3437701b582aa98594b1d6e3033c37da5b0dcca5342e905c94b398c21748a0a5`

Reproduce: `python tools/res86/qualify_e10_closure.py --out <path>` (two
bit-identical executions per candidate).

NEXT_AUTHORIZED_ACTION=NONE.  The mission failure clause applies: the single
controlling causal blocker is reported above and the mission stops.  RES-87 is
not started.
