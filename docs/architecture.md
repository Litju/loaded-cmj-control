# Architecture

The causal path is deliberately single-owner:

```text
public observation
    -> isolated policy worker
    -> validated 15-D action
    -> drive state
    -> MuJoCo step
    -> live BiomechanicalSample
    -> CMJEventDetector
    -> immutable RolloutResult
```

The policy process never receives MuJoCo handles, event results, termination
reasons, private state, or the event detector. The parent owns the episode
clock, the 8 kHz integration loop, the single event engine, and the result.

The virtual force plates retain designated support and prohibited/off-platform
contact separately while contributing all measured forces to the whole-support
wrench. COP validity is explicit. COM, inertial-center velocity, mechanics
residuals, realized actuator power, passive/limit work, and event state are
extracted from the same live post-step sample.

The replay module accepts only a sealed `RolloutResult` trace and its identity.
It does not create a second simulation or event path.
