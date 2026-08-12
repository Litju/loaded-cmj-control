# ML212C — first-witness action sign-sector owner closure

**Status:** PASS — owner decision frozen; implementation remains a separate gate.

**Scope:** This is a normative authority record for the first smooth feasibility
witness. It documents the sign-sector decision only. It does not implement the
sector bounds, authorize an Ipopt call, or change the Plant, Drive, MJCF,
transition physics, or controller.

## 1. Authority chain

The current authority is the repository
`/home/litju/Projects/loaded-cmj-control` and its evidence root
`/home/litju/Projects/loaded-cmj-control-evidence`. The legacy Alignerr
repository is historical-only and is not an authority for this closure.

The closure chain is:

1. **ML212B owner feasibility closure:** PASS, commit
   `8ea23739bd1ae214eb221642786a6d91096fb9f1`,
   `docs/ML212B_E3_E4_FEASIBILITY_OWNER_CLOSURE.md`.
2. **ML242-G2 structural NLP repair:** PASS, commit
   `8fbd4e2dc74efab4b1ee19bb300a487453c02edc`.
3. **ML241-G3 derivative repair:** PASS, commit
   `53fb65a602e094a0898ed043a94ebd127c8c7e71`.
4. **ML212C:** this document freezes the first-witness sign-sector owner
   decision. Its implementation is deferred to ML242B-G3.5.

The authoritative E3 anchor is:

```text
E3_ID=E3_R024_VALID_COUNTERMOVEMENT_GRID_0770000
E3_TIME_S=0.770000
E3_STATE_HASH=2cdd3cda6c6b91ba211dfec696e2348a5446992f32fdb65d53f51951eafac208
E3_COM_VZ_MPS=-0.2747849764551471
E3_PLANT_HASH=1d1e1dc753786cd5f070e115210d8fcc5c4ee4b0b58753a2eeadeb5aa81a2dbd
E3_MJCF_HASH=cc122cee734faa2a72d2b1bb829f824c7e865ba3365695887468fc7844dfc615
```

## 2. G3 blocker and independent receipt check

ML241-G3 established that the seven free witness controls are locally smooth
for the E3 request, while the raw witness action domains still contain the
directional-drive kink at zero:

```text
WITNESS_FREE_DOMAIN_BEFORE_CLOSURE=[-1,1]^7
WITNESS_FREE_DOMAIN_CONTAINS_ZERO_KINK=YES
PRE_SOLVER_SIGN_SECTOR_DECISION_REQUIRED=YES
```

The G3 receipt was independently checked against the current E3 provenance and
the read-only current-session action-domain/derivative ledger. The check
confirmed that:

- the free witness indices are `0,3,6,9,10,11,12`;
- each proposed seed is strictly inside its proposed closed sector;
- the fixed indices `1,2,4,5,7,8,13,14` are the explicit true-zero-kink
  negative controls and remain structurally absent from the witness decision;
- the G3 `A[2]` negative control is a physical contact-switch rejection; and
- no solver call or new rollout was used for this owner decision.

The E3 retained previous accepted action is a state/provenance field. It is not
to be conflated with the first-witness raw seed: in particular, its channel 0
value is `1.0`, whereas the witness seed below is `0.900000`. This distinction
does not alter the E3 anchor or the sign decision.

## 3. Authoritative E3 witness action signs

The first-witness seed reproduced at the authoritative E3 is:

| action index | channel | seed action | sign |
|---:|---|---:|---|
| 0 | `lumbar_flexion` | `+0.900000` | positive |
| 3 | `left_hip_flexion` | `+0.16806307252533137` | positive |
| 6 | `right_hip_flexion` | `+0.16806307252528851` | positive |
| 9 | `left_knee_flexion` | `+0.11091999487016607` | positive |
| 10 | `right_knee_flexion` | `+0.11091999487016607` | positive |
| 11 | `left_ankle_dorsiflexion` | `-0.016134519367529535` | negative |
| 12 | `right_ankle_dorsiflexion` | `-0.01613451936758624` | negative |

Therefore:

```text
AUTHORITATIVE_E3_SEED_INTERIOR_TO_SECTORS=YES
```

The strict interior statement is about the witness seed, not about the zero
boundary itself.

## 4. Owner decision: first-witness sign sectors

Freeze the following bounds for the first smooth feasibility witness:

```text
POSITIVE_SECTOR_ACTION_INDICES=0,3,6,9,10
POSITIVE_SECTOR_BOUNDS=[0,1]

NEGATIVE_SECTOR_ACTION_INDICES=11,12
NEGATIVE_SECTOR_BOUNDS=[-1,0]

SIGN_SECTOR_EPSILON_MARGIN=NONE
```

Equivalently, for the seven free witness controls:

```text
u[0],u[3],u[6],u[9],u[10] ∈ [0,1]
u[11],u[12]                ∈ [-1,0]
```

The eight fixed controls remain exactly zero and structurally absent from the
witness decision vector:

```text
WITNESS_FIXED_ACTION_INDICES=1,2,4,5,7,8,13,14
WITNESS_FIXED_ACTION_VALUES=0
```

No epsilon-shifted bounds are authorized. In particular, do not replace the
closed sectors with `[epsilon,1]` or `[-1,-epsilon]`.

## 5. Zero-boundary semantics

The frozen physical Plant remains:

```text
u_i ∈ [-1,1]  for all 15 channels
```

The directional Drive separates positive and negative activation from the raw
action. Consequently:

```text
RAW_ACTION_ZERO_CLASS=TRUE_DIRECTIONAL_DRIVE_KINK
```

Including zero as a closed sector endpoint is an exact bound decision; it does
not make the full Plant differentiable at zero and does not weaken ML241's
kink rejection. Existing callback classification tolerances are not an
epsilon margin and must not be converted into shifted sector bounds without a
new authority decision.

The sign restriction applies only to the first smooth feasibility witness. It
is not a new Plant limit, physiological restriction, permanent controller
restriction, full-task action restriction, or claim that sign reversal is
physically impossible.

The following remain qualification dependencies:

```text
IPOPT_BOUND_RELAX_FACTOR=0.0
IPOPT_BOUND_PUSH_STATUS=QUALIFICATION_REQUIRED
IPOPT_BOUND_FRAC_STATUS=QUALIFICATION_REQUIRED
SIGN_SECTOR_CALLBACK_ZERO_BEHAVIOR=QUALIFICATION_REQUIRED
```

No real Ipopt call is authorized before those qualifications. Full-horizon
derivative qualification also remains deferred.

## 6. Set inclusion and result interpretation

Let `U_FULL_15D_PLANT` be the physical 15-channel action domain, let
`U_7_CONTROL_WITNESS` impose the eight exact-zero structural witness controls,
and let `U_SECTOR_WITNESS` additionally impose the seven sign-sector bounds.
Then:

```text
U_SECTOR_WITNESS
⊂
U_7_CONTROL_WITNESS
⊂
U_FULL_15D_PLANT
```

The inclusions follow directly because the sector witness adds sign bounds to
the seven-control witness, and the seven-control witness fixes eight channels
to zero inside the full Plant domain. Therefore:

```text
SECTOR_WITNESS_PASS
=> FULL_PLANT_EXISTENCE_ESTABLISHED

SECTOR_WITNESS_FAIL
!= FULL_PLANT_INFEASIBILITY
SECTOR_WITNESS_FAIL
!= SEVEN_CONTROL_WITNESS_INFEASIBILITY
```

A sector-witness failure is a failure of this restricted witness attempt and
requires bounded escalation review. It is not a global infeasibility result.

## 7. Gate status and next gate

This document records an owner PASS for the sign-sector decision. It does not
claim that the bounds are already implemented:

```text
SIGN_SECTOR_BOUNDS_IMPLEMENTED=NO
SOLVER_RUN_PERFORMED=NO
NEW_ROLLOUT_PERFORMED=NO
```

The next gate is:

```text
NEXT_GATE=ML242B_G3_5_WITNESS_SIGN_SECTOR_BOUND_IMPLEMENTATION
```

That gate must implement and test only the frozen witness bounds, preserve the
fixed-control structural layout and full physical Plant domain, and complete
the required zero-boundary/Ipopt qualification before any solver execution.
