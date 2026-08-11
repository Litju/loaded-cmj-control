# ADR — ML-240 same-Plant oracle constraint catalog

Status: accepted for the ML-240 boundary only
Model: `LCMJ-V1-20KG-HIGHBAR@1.0.0`
Architecture: `LCMJ-V1-CONTROL-SYSTEM-ARCH-1.0.0`

## Decision

The catalog ID/schema is `LCMJ-V1-ORACLE-CONSTRAINT-CATALOG-1.0.0`. It is a
metadata and thin-adapter seam over existing qualified owners. The catalog
does not own dynamics, event state, measurements, public observations,
scoring, reward, derivatives, sparse transcription, or solver execution.

The canonical residual convention is `g >= 0` feasible and `h = 0` for
equalities. Hard bounds and residuals require explicit source/F3/model/event
tolerance provenance. Missing thresholds remain diagnostic or postcheck rows;
they are not filled from optimizer output or generic biomechanics assumptions.

The classification taxonomy is `HARD_PATH`, `HARD_GUARD`, `HARD_TERMINAL`,
`DIAGNOSTIC_ONLY`, and `PHASE_METADATA`. Enforcement metadata is
`INTRINSIC_TRANSITION`, `EXPLICIT_NLP_LATER`, `AGGREGATE_NLP_LATER`,
`POSTCHECK_EVENT_ENGINE`, `POSTCHECK_MECHANICS`, or `METADATA_ONLY`.

Differentiability metadata distinguishes smooth fixed-mode values, piecewise
smooth owners, nonsmooth contact, discrete predicates, interval aggregates,
post-trace values, and non-applicable metadata. Contact/event discontinuities
are not smoothed for future derivatives.

Official event truth remains `CMJEventDetector`; its mutable state is evaluator
postcheck data, never oracle Markov state. `predict_support_reserve` remains a
diagnostic because its stopping-horizon approximation is not an exact
same-Plant reachability certificate. No scorer/reward quantity is a
feasibility constraint, and no privileged catalog value is routed to
`PolicyWorker`.
