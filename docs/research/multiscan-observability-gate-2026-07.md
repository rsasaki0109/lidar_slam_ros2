# Multi-scan Hessian observability gate study (2026-07-15)

## Decision

Reject the normalized-Hessian window as an intervention gate. Keep the code
default-off only as a reproducible negative result; do not enable either
research profile in a shipped configuration.

This study changes only incremental scan registration while creating a new
map. Saved-map loading, localization mode, and relocalization remain excluded.

## Candidate

The existing persistent weak-direction tracker was extended with a fixed
window of per-scan Hessians, each normalized to unit trace in the common
world-frame left-perturbation coordinates. Prior blending was allowed only
when the tracked axis was weak both instantaneously and in the accumulated
window. Synthetic tests cover a stationary weak axis, a slowly rotating weak
axis that becomes observable over the window, and exact legacy persistence
behavior when the new gate is disabled.

## Results

| profile | HILTI exp07 APE RMSE (m) | MID-360 delta RMSE (m) | verdict |
|---|---:|---:|---|
| default-off | 0.318107 | 0.000000 | reference |
| persistent gate | 0.316617 | 0.530182 | prior research candidate |
| window ratio `1e-6` | 0.378146 | not run | HILTI regression |
| window ratio `1e-5` | 0.359920 | 0.577689 | both gates worse |

At `1e-5`, HILTI had 195 intervened scans / 5,270 intervention iterations;
MID-360 had 942 intervened scans / 12,921 iterations. The gate therefore did
not separate the beneficial and harmful cases. The first `v1` HILTI artifact
used a stale installed RKO-LIO binary, is bit-identical to the old persistent
candidate, and is explicitly excluded from the table.

Artifacts:

- `/media/sasaki/aiueo/benchmarks/hilti_exp07_multiscan_observability_20260715_v2`
- `/media/sasaki/aiueo/benchmarks/hilti_exp07_multiscan_observability_20260715_v3`
- `/media/sasaki/aiueo/benchmarks/mid360_public/multiscan_observability_20260715_v1`

## What the failed follow-up established

A proposed 3x3 translation Schur-complement diagnostic was removed after its
origin-invariance test failed. For this point-to-point ICP Jacobian the fixed
correspondence translation block is always identity; the corridor ambiguity
comes primarily from correspondence changes and local surface geometry, not
from a single fixed-correspondence Hessian. The next candidate must therefore
use correspondence-level local surface information (for example multi-scan
voxel covariance / point-to-plane constraints), rather than another scalar
derived from the same 6x6 Hessian.

## Local-surface follow-up

A cached voxel-covariance prototype then applied anisotropic residual weights
(plane-normal weight `1.0`, tangent weight `0.1`) using only map points already
stored in the matched voxel. On the allowed exp01 development sequence it did
not improve accuracy:

| exp01 profile | raw APE RMSE (m) | matched GT points | processing RTF |
|---|---:|---:|---:|
| same-build default | 0.066713 | 10/13 | 2.1369 |
| cached local surface | 0.066757 | 11/13 | 1.8759 |

The candidate was removed rather than retained behind a flag: it missed the
3% accuracy gate, remained slower than real time, and its per-voxel cached
statistics would have increased memory even when the feature was disabled.
Artifacts are
`/media/sasaki/aiueo/benchmarks/hilti_exp01_local_surface_20260715_default`
and
`/media/sasaki/aiueo/benchmarks/hilti_exp01_local_surface_20260715_candidate_v1`.

## Runtime follow-ups

Two selective-compute ideas were also rejected. A fixed 10-iteration cap made
exp01 real-time (`RTF 0.934`) and improved exp04 on seven common checkpoints,
but regressed the exp07 corridor from `0.318107` to `0.419805` m. Extending the
budget after weak-Hessian observations did not restore corridor accuracy:
hold windows of 0, 5, 20, and 50 scans produced exp07 APE `0.399267`,
`0.389377`, `0.342768`, and `0.382299` m respectively. A fixed coarse ICP
keypoint multiplier also failed: `2.0x` and `2.5x` produced `0.351691` and
`0.382267` m. Their development configs were removed.

One exact nearest-neighbor optimization was retained. Voxel candidates are now
compared with squared distances and `sqrt` is evaluated only once for the
winning point. On complete exp01 runs, trajectory APE remained exactly
`0.0655590872` m with all 13 checkpoints while processing RTF improved from
`3.0761` to `2.0612` (33.0%). The optimized artifact is
`/media/sasaki/aiueo/benchmarks/hilti_exp01_squared_nn_20260715_v1`.
