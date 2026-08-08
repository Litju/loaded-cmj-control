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

## Scope

This repository is scientific simulation and control software, not a medical
or clinical model. The public license remains owner-decision pending.
