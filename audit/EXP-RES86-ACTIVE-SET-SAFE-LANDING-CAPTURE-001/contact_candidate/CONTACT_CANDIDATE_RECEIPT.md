# RES-86 CONTACT CANDIDATE QUALIFICATION RECEIPT

MISSION=RES86_CONTACT_REALIZATION_QUALIFICATION_AND_E10_CLOSURE_001
LINEAR_ISSUE=RES-86
EXPERIMENT_ID=EXP-RES86-ACTIVE-SET-SAFE-LANDING-CAPTURE-001
STATUS=PASS (contact-candidate realization qualification; E10 continues in the same mission)

## Candidate identity (labels are not the claim)

Chosen by the declared rule "smallest declared stiffening supported by the
existing RES-86 sensitivity": the plantar support-geom `solref` time constant
0.02 -> 0.01 s.  The other lattice entries additionally stiffen the impedance
scaling and are therefore larger changes.  `condim`, `solimp` and `friction`
are unchanged; this is not a Plant topology change.

The engine does not consume the geom-level declaration.  With equal priorities
(0/0) and equal `solmix` (1.0/1.0), MuJoCo mixes the plantar geom `solref`
with the floor's compiled default `(0.02, 1.0)`; the runtime `mjContact` row is:

| realization | declared geom solref | realized `mjContact.solref` | dim | solimp | friction | includemargin |
|---|---|---|---|---|---|---|
| nominal | (0.02, 1.0) | (0.02, 1.0) | 4 | (0.9, 0.95, 0.001, 0.5, 2.0) | (0.9, 0.9, 1e-05, 1e-05, 1e-05) | 0.0 |
| candidate | (0.01, 1.0) | **(0.015, 1.0)** | 4 | (0.9, 0.95, 0.001, 0.5, 2.0) | (0.9, 0.9, 1e-05, 1e-05, 1e-05) | 0.0 |

Frozen solver semantics are untouched: Euler integrator, Newton solver,
pyramidal cone, 100 iterations, `disableflags` 0 with `refsafe` enabled and the
contact path enabled at every dt.  The realized time constant is numerically
resolved against every declared step: 0.015/0.001 = 15.0, 0.015/0.002 = 7.5,
0.015/0.004 = 3.75.

## Qualification battery

Fixed-action short-horizon (40 ms) contact response from the same branch state,
prior nominal bounded-search witness:

| dt (s) | nominal penetration | candidate penetration | nominal failures | candidate failures |
|---|---|---|---|---|
| 0.001 | 10.8514 mm | **8.1511 mm** | MAX_PENETRATION | none |
| 0.002 | 10.2115 mm | **8.0908 mm** | MAX_PENETRATION | none |
| 0.004 | 31.9444 mm | 26.0661 mm | MAX_PENETRATION | MAX_PENETRATION (out of domain) |

Full-horizon prior-witness reproduction (bit-exact, repeated-replay identical):

| cell | penetration | peak Fz | ROM margin | verdict |
|---|---|---|---|---|
| nominal witness @ nominal, dt 0.002 | 10.2115 mm | 2.2635 BW | +0.0161 rad | MAX_PENETRATION only |
| candidate witness @ candidate, dt 0.002 | **9.9102 mm** | 2.8522 BW | +0.0441 rad | **hard-safety admissible** |

Material causality: the same fixed actions produce materially different
trajectories under the two realizations (nominal witness: 10.2115 mm ->
20.7999 mm; candidate witness: 45.8188 mm -> 9.9102 mm), so contact compliance
is proven *materially causal*, not monotone.

## Criteria (all PASS)

V1 realization identity; V2 engine semantics unchanged; V3 time-constant
resolved at every dt; V4 repeated-replay determinism (all cells);
V5 bit-exact prior-witness reproduction; V6 candidate hard-safety admissible at
the production dt while the nominal boundary fails only `MAX_PENETRATION`;
V7 material causality; V8 in-domain dt comparison (candidate penetration <
nominal and failure classes subset at dt 0.001/0.002); V9 dt = 0.004 recorded
as out-of-domain stress (it moves 8 mm per sample at the sealed ~2 m/s
descent, comparable to the 10 mm budget); V10 candidate hard-authority hygiene
(moment/power/MTP/ROM/peak-Fz/chatter within the frozen limits).

## Adjudication boundaries (never promoted)

- The nominal 10.2115 mm from the bounded search is a **bounded-search
  result, never a global infeasibility proof**.
- Contact compliance is proven **materially causal**; it is never claimed to be
  monotone across trajectories.
- The existing candidate witness is a **vertical-arrest / hard-safety witness
  only** (terminal Hy 27.59 kg m^2/s and posture far outside the E10 point
  gates).  It is **not E10** and no E10 claim is derived from it.

## Artifacts

- `CONTACT_CANDIDATE_QUALIFICATION.json` sha256
  `a257bfd8cce7ad1e3f8e5fb07ba9fb3ac78ebcc1ad6dfdab3e5e5e0557004594`
- `REFERENCE_WITNESSES.json` sha256
  `c2c45fd82c1c1d9153269e2a5ecbc5b5feb03670c007498f5d682a3a1f9c89c`
- `src/loaded_cmj/v3/contact_realization.py` sha256
  `d474083177fcac165f5b4a1728ea62fb96de2865399f6816312e64357b5498ad`
- `tools/res86/qualify_contact_candidate.py` sha256
  `80fcaa33eb499b1729601435a91a87c168044c8b874515292eab2bf22d26b887`
- `tests/test_res86_contact_candidate.py` sha256
  `b1f8369ad82c03edb913e87841fba10a56be116a0144eeab950b6ad8d7f52671`

Reproduce: `python tools/res86/qualify_contact_candidate.py --out <path>`;
focused tests: `pytest tests/test_res86_contact_candidate.py
tests/test_res86_runtime_contact_realization.py`.

NEXT: production E10 closure continues in the same mission under the qualified
realization (active MTP within the frozen budget, legal toe/forefoot
progression, post-apex LANDING_PREP, two-stage absorption, exact branch
validation, E10-aware optimization/validator).
