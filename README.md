# Loaded CMJ Control

Loaded CMJ Control is a deterministic MuJoCo simulation and hybrid-feedback
control example for a 75 kg modeled athlete carrying a rigid 20 kg external
load. It includes the qualified multibody plant, actuator and passive dynamics,
bilateral six-axis virtual force plates, live biomechanical samples, CMJ event
detection, isolated policy execution, immutable rollout results, and a minimal
replay boundary.

The fixed experiment uses an 8 kHz physics loop (`0.000125 s`), 40 physics
steps per 200 Hz controller update (`0.005 s`), and a 0.300 s policy-free
equilibrium settling period. The public observation contains 16 fields and the
controller action has 15 bounded anatomical channels.

Generation 1 is the frozen PFIP baseline. Its qualified reference behavior is
intentional: the rollout reaches `supported_start`, then terminates with
`PHYSICAL_FALL` after premature bilateral support-force collapse during the
countermovement. No Generation 2 controller changes are included.

## Run the baseline

```bash
uv sync --extra test
uv run python experiments/run_gen1.py --output /tmp/loaded-cmj-gen1
```

The command writes a summary, immutable result projection, and raw live trace.
The example defaults to local execution without an account switch. Pass
`--drop-privileges` on a host with a configured unprivileged worker account;
the policy still runs in a fresh isolated process with bounded protocol,
timeout, and action validation.

## V2.1 canonical qualification runtime (V2.1-R001)

V2.1-R001 is the single frozen V2.1 candidate: 95 kg athlete+load, NQ/NV/NU
10/10/7, ACTION_DIM 7, physics_dt 0.000125 s, nominal control_dt 0.005 s with
committed 28/33/40 hybrid truncations, horizon 20.0 s through E12 confirmation
plus 0.30 s posthold. It runs the exact committed production composition
(Res72Policy C01/RES58 → BalanceController RES-73 → StableRecoveryController
RES-74 incl RES43 hold) from canonical reset with synchronized
sample-before-update observations and observational scorer only.

Execute exactly:

```bash
python -m loaded_cmj.v2.canonical_runtime --candidate V2.1-R001
```

Contract: `CANONICAL_V2_RUNTIME_CONTRACT.md`. Spec: `CANONICAL_V2_CANDIDATE_SPEC.json`.
Module: `src/loaded_cmj/v2/canonical_runtime.py`.

PASS requires 12/12 exact event identity, 7/7 checkpoint identity
(S_APEX/S_E10/S_E11/S_RR/S_RR_CONFIRMED/S_STAND_HANDOFF/S_E12), exact
28/33/40 schedule (3045×40, 2×28, 1×33), same modes/termination/hard gates,
post-landing loss 0 reflight [] chatter 0, online/offline PASS, and fresh-process
identical reproduction. Results/evidence go to
`EXP-RES12A-CANONICAL-RUNTIME-AUTHORITY-001` (Evidence Contract v2).
RES-12 consumes this runtime without redefining authority.

## Scope

This repository is scientific simulation and control software, not a medical
or clinical model. The public license remains owner-decision pending.
