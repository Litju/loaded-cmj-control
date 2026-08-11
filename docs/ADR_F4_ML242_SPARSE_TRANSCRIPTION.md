# ADR: ML-242 exact-discrete sparse transcription

Status: accepted for ML-243 handoff

## Decision

ML-242 represents each knot with an immutable `MacroSnapshot` reference and a
132-dimensional local tangent offset. The actual state is reconstructed only
as `boxplus(reference_snapshot, delta_x)`. The decision order is

```text
delta_x[0], u[0], delta_x[1], u[1], ..., delta_x[N-1], u[N-1], delta_x[N], slack suffix
```

where `u` is the raw 15-dimensional public command. The first tangent knot is
retained and fixed to zero by variable-bound metadata when requested.

The exact interval residual is

```text
d[k] = boxminus(x[k+1], Phi_5ms(x[k], u[k]))
```

with `step_5ms` as the only physical advancement owner. No raw snapshot
flattening, alternate plant, direct torque path, or copied action-slew logic
is introduced.

## Jacobian contract

Each interval has only three structural blocks: the current 132-state knot,
the current 15-action knot, and the next 132-state knot. The current-state and
action blocks consume the qualified ML-241 `A_k` and `B_k` maps. The endpoint
maps of `boxminus` are finite-differenced in tangent coordinates only, giving

```text
J_xk    = G_pred A_k
J_uk    = G_pred B_k
J_xnext = G_next
```

At zero defect, `G_pred` and `G_next` qualify as `-I` and `+I`; the endpoint
maps are still evaluated at every nonzero-defect point. Invalid active-set or
piecewise-action derivative metadata rejects ordinary local assembly.

The COO row/column pattern is deterministic and contains dense local blocks;
qualified-zero incoming qacc columns remain in the pattern. The 21 qacc
coordinates and nine cache-SO3 coordinates remain ordinary next-knot state
coordinates and are constrained by `G_next`.

## Constraint and slack boundary

ML-240 remains the sole constraint catalog. Intrinsic transition enforcement,
explicit smooth row metadata, aggregate sequence metadata, event postchecks,
mechanics postchecks, and metadata-only entries retain their catalog
dispositions. Capturability stays diagnostic-only. E3→E4 rows are represented
as explicit smooth terminal metadata only where ML-240 permits it; discrete and
postcheck terminal predicates remain guard metadata.

The approved ML-240 Phase-I elastic allowlist is represented as nonnegative
slack metadata with its original scale token and provenance. No unresolved or
owner-derived scale is replaced by an invented number, and no feasibility or
performance objective is created.

## Solver separation

The owner exposes deterministic packing, bounds, residuals, COO structure,
Jacobian values, and JVP access through NumPy/simple immutable structures. It
does not import SciPy, cyipopt, Ipopt, or any solver adapter. ML-243 owns the
future thin solver integration.
