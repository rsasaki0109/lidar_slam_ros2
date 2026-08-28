# Persistent weak-direction gate study (2026-07-14)

## Decision

Keep the persistent weak-direction gate as an opt-in research profile. Do not
enable it by default. It removes most of the cross-sensor failure caused by the
Phase 2 stateless solve, but the remaining MID-360 trajectory change is still
too large for adoption.

This work changes only incremental scan registration during new-map creation.
Saved-map loading, localization mode, and relocalization are outside its scope.

## Implemented contract

The RKO-LIO solve intervenes only when all of the following hold:

1. `degeneracy_aware_solve` and `degeneracy_persistence_gate` are both enabled;
2. an isolated, translation-dominant weak Hessian direction is observed;
3. its world-frame axis matches for at least three consecutive scans;
4. the current ICP iteration still contains a near-null isolated direction
   matching that confirmed axis.

Until confirmation, the solver returns the legacy `H.ldlt().solve(-b)` result
exactly. Directions other than the confirmed isolated axis, including repeated
`NON_OBSERVABLE` eigenspaces, also retain the legacy result. This matters
because the Phase 2 MID-360 failure was dominated by freezing a repeated weak
subspace.

Tracking uses the Phase 1 diagnostic threshold `1.5e-5`; intervention remains
at the stricter frozen Phase 2 threshold `1.0e-6`. Each opt-in run writes
`degeneracy_persistence.csv` with timestamp, candidate/confirmed state, streak,
axis cosine, six-axis vector, and intervention count.

## Cross-sensor results

| candidate | HILTI exp07 APE RMSE (m) | MID-360 delta vs unchanged baseline (m) | verdict |
|---|---:|---:|---|
| default-off baseline | 0.318107 | 0.000000 | reference |
| Phase 2 stateless solve | **0.259018** | 5.891015 | cross-sensor hard fail |
| persistence, tracking threshold `1e-6` | 0.332002 | not run | HILTI regression |
| persistence, tracking threshold `1.5e-5` | **0.316617** | **0.530182** | improved safety, not adoptable |
| above + weak-axis/motion alignment `>=0.10` | 0.374941 | not run | rejected and removed |

The accepted research profile reduces the old MID-360 deviation by 91.0%,
while slightly improving HILTI exp07 by 0.47% relative to default-off. That is
useful evidence that persistent-axis gating addresses a real part of the
failure, but 0.530 m RMSE / 1.273 m maximum change over 2,772 timestamp-matched
MID-360 poses is not safe enough to ship.

Persistence by itself does not distinguish the sensors: the maximum confirmed
streak was 165 scans on HILTI exp07 and 931 scans on MID-360. The original
hypothesis that the harmful MID-360 axes would be short-lived is therefore
rejected.

## Diagnostics

| dataset | samples | candidate scans | confirmed scans | intervened scans | intervention iterations | max streak |
|---|---:|---:|---:|---:|---:|---:|
| HILTI exp07 | 1,320 | 654 | 538 | 202 | 5,727 | 165 |
| MID-360 | 2,771 | 1,959 | 1,950 | 944 | 13,023 | 931 |

Artifacts:

- HILTI accepted research profile:
  `/media/sasaki/aiueo/benchmarks/hilti_exp07_persistent_degeneracy_20260714_v2`
- HILTI strict-tracking rejection:
  `/media/sasaki/aiueo/benchmarks/hilti_exp07_persistent_degeneracy_20260714_v1`
- HILTI motion-alignment rejection:
  `/media/sasaki/aiueo/benchmarks/hilti_exp07_persistent_degeneracy_20260714_v3`
- MID-360 accepted-profile safety run:
  `/media/sasaki/aiueo/benchmarks/mid360_public/persistent_degeneracy_20260714_v1`

## Runtime and memory

Cache-warm standalone exp07 runs with the same rebuilt binary and `/usr/bin/time
-v` produced:

| profile | wall time (s) | peak RSS (KiB) | ICP mean (ms) |
|---|---:|---:|---:|
| default off | 39.34 | 461,068 | 16.65 |
| persistent gate | 38.26 | 460,724 | 15.82 |

The differences are within ordinary single-run host noise. The correct
conclusion is no measured runtime or memory regression, not a performance
improvement. Raw reports are `resource_baseline_time.txt` and
`resource_candidate_time.txt` under the accepted HILTI artifact directory.

## Verification

The ROS-free reference and RKO-LIO fork implementations are cross-checked on
the same synthetic corridor system. Tests also cover confirmation after the
required streak, eigenvector sign invariance, axis-jump reset, reset on a
well-conditioned scan, rejection of repeated non-observable spaces, rejection
of rotation-dominant candidates, exact legacy fallback before confirmation,
and rejection of an unmatched confirmed axis.

The next credible intervention needs information richer than Hessian-axis
persistence alone, such as correspondence-level contribution classes or an
independently validated uncertainty-aware prior. Another scalar streak or
angle threshold is not supported by this evidence.
