# Fixed contracts

- Athlete mass: `75 kg`.
- Rigid external load: `20 kg`.
- Physics step: `0.000125 s` (`8 kHz`).
- Controller period: `40` physics steps (`0.005 s`, `200 Hz`).
- Maximum experiment horizon: `4.0 s` after settling.
- First physical/scored sample: `0.000125 s`; experiment time zero is the first controller observation.
- Settling: `0.300 s`, `2400` physics steps, fixed hold action, no policy calls or scored samples.
- Action: 15 finite values in `[-1, 1]`; rejected values never reach the plant.
- Event engine: one stateful `CMJEventDetector` over the live sample path.
- Work: one trapezoidal pre-step-to-post-step integration owner.

These values are copied from the frozen Generation-1 plant and runtime and are
not controller tuning parameters.
