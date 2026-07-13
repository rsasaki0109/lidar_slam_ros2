# HILTI degeneracy-aware solve Phase 2 (2026-07-13)

## Decision

Keep the new RKO-LIO solve opt-in and default off. The frozen candidate
improves the HILTI exp07 corridor trajectory without regressing the untouched
exp01 and exp04 holdouts, but it has not yet passed the repository-wide
multi-dataset runtime, memory, geometry, and map-quality adoption gate.

The opt-in candidate uses:

- `degeneracy_aware_solve=true`
- intervention contribution threshold `1.0e-6`
- non-observable multiplicity gap `1.0e-8`
- isolated-degenerate prior weight `0.25`

The intervention threshold is intentionally stricter than the Phase 1
diagnostic threshold `1.5e-5`. Diagnostics continue to report the broader
weak-direction population; pose remapping is limited to the near-null tail.

## Frozen solve contract

The ROS-free solver works in the sorted Hessian eigenbasis:

- well-conditioned direction: retain the Gauss--Newton component;
- isolated degenerate direction: blend the geometric component with the
  constant-velocity/IMU initial-guess correction;
- repeated non-observable eigenspace: apply zero update, because individual
  eigenvectors inside that subspace are basis-dependent.

Synthetic tests require a fully observable box to reproduce the legacy
Gauss--Newton update, the corridor's single null axis to recover the requested
prior fraction, and the single-plane three-axis null space to remain fixed.
The in-repository reference implementation and the RKO-LIO integration must
also agree numerically on the same system.

## Tuning and holdout protocol

Only HILTI 2022 exp07 was used for parameter selection. Prior weights 1.0,
0.25, and 0.1 at the Phase 1 threshold `1.5e-5` all regressed. The final
candidate lowered only the intervention threshold to `1.0e-6` and retained
weight 0.25. The exp01 and exp04 results below were then evaluated once,
without further tuning.

| sequence/profile | control points | APE RMSE (m) | decision |
|---|---:|---:|---|
| exp07 default-off baseline | 6 | 0.318107 | baseline |
| exp07 ratio 1.5e-5, weight 1.0 | 6 | 0.512671 | reject |
| exp07 ratio 1.5e-5, weight 0.25 | 6 | 0.344379 | reject |
| exp07 ratio 1.5e-5, weight 0.1 | 6 | 0.389695 | reject |
| exp07 ratio 1.0e-6, weight 0.25 | 6 | **0.259018** | tuning pass |
| exp04 holdout, frozen candidate | 7 | 0.071560 | tied, pass |
| exp01 holdout, frozen candidate | 13 | 0.065559 | within 0.066 gate, pass |

Artifacts are rooted at:

- `/media/sasaki/aiueo/benchmarks/hilti_exp07_degeneracy_solve_20260713`
- `/media/sasaki/aiueo/benchmarks/hilti_exp04_degeneracy_solve_holdout_20260713`
- `/media/sasaki/aiueo/benchmarks/hilti_exp01_degeneracy_solve_holdout_20260713`

After rebuilding with the frozen opt-in defaults, the exp07 candidate repeated
the exact APE above and produced the same trajectory SHA-256 on both runs:
`86b591aff93ffc4f9c1cf6b40ab61a2d150803f25ec9d1f80826355db835b013`.
Two default-off runs likewise matched at
`8ca65f324835c24b5f90006eba2767d01d21c42e126ce60d1cb132a795985e26`
and repeated the 0.318107 m APE baseline. The rebuilt offline-node SHA-256 was
`1158b8da6eebbc3874e900e980df211d473e11357149605cb72a6949b50a551f`.

The cache-controlled repeat pair measured 57.78 s / 472,680 KiB for the
candidate and 57.57 s / 472,624 KiB for default-off: +0.36% wall time and
+0.012% peak RSS. Accuracy and the local resource gate pass this phase;
repository-wide public-suite and map-quality adoption remain pending.
