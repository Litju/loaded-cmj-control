# RES85E EXTERNAL EVIDENCE BINDING — RES-85

MISSION: `RES85E_EVIDENCE_CONTRACT_V2_REQUALIFICATION_AND_DELIVERY_001`
LINEAR ISSUE: RES-85
STATUS: `EVIDENCE_DELIVERED` (v2.0.0 bundle independently verified; reviewer addenda recorded)
FROZEN_SCIENCE_HEAD: `b46735b9e0ee23532b988ddcb1169a67518e9a6a`
FROZEN_SCIENCE_TREE: `ca6b7daf0f635b8719ba06e87330c61dc21209d4`

## External deliverable

* archive: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001.tar.gz`
* archive sha256: `fe54d7f1eff49ed2d3b01dc4137962beb91e88382c910e29ddfd148067b21ac5`
* archive bytes: `3046578`
* bundle directory: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001`
* bundle file count: `73`
* manifest: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001/manifest.json`
* MANIFEST_FILE_SHA256: `8dbe009d11b4294a1da95bea9c9bb18a262ed04b1d1c001811d9c173c8eb8512`
* MANIFEST_CANONICAL_SHA256: `678fd09ab5823b776461e2ce6246e60113caae1343865645ef21c49a4f25fcf8`
* experiment spec: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001/experiment_spec.json`
* EXPERIMENT_SPEC_SHA256: `5505c47af9c686f7aaca82cc89920a1a915edd82b29fd0e32a59d571904e9c31`
* initial state vector SHA256: `2a129a97e94061fe040a560b39f8f4297b9ac36265bae2dd053df0715b5d6812`
* reproduction verdict: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001/reproduction.json` (IDENTICAL=True)
* external verification log: `/home/litju/Projects/loaded-cmj-control-evidence/EXP-RES85E-RES85D-V2-EVIDENCE-DELIVERY-001.VERIFICATION.json` (status=PASS, 26 checks)

The heavy raw traces (`physics_trace.npz`, `control_trace.npz`, `initial_integration_state.npz`,
`branch_control_sequence.npz`, `self_verification/*`) live only in the external evidence root.

## Qualification results (recomputed from the delivered traces)

| Quantity | Value |
|---|---|
| canonical H2 (direct SYSTEM_COM) | `0.16021398534364972` m (ABOVE_FLOOR) |
| takeoff occurrence | sample `612` |
| takeoff confirmation | sample `637` |
| RES-85 claim end | sample `791` |
| trunk max | `0.5422164233170099` rad |
| global min structural ROM margin | `0.0` rad |
| targeted tests | `{'collected': 222, 'failed': 0, 'passed': 222, 'skipped': 0}` |
| fresh-process reproduction | IDENTICAL=true, zero qpos/qvel/action error |
| nonzero-branch pipeline self-verification | IDENTICAL=true at branch 0.2 s |

## Claim ceiling (unchanged)

CS-anchored control qualification of the frozen RES-85D launch/flight only. No landing/recovery
claim, no elite performance norm, no human predictive validity. The 0.150 m floor is a functional
non-triviality boundary only. The zero-passive/zero-active model-form sensitivity (H2 = 0.12816367328967782 m)
remains below that floor and is disclosed as a later-authority item.

`NEXT_AUTHORIZED_ACTION=RES86_ACTIVE_SET_SAFE_LANDING_CAPTURE` is unchanged.
