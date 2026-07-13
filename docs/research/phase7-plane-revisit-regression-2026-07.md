# Phase 7 plane-revisit candidate regression

Date: 2026-07-14

## Decision

The current initial-residual-gated plane-revisit candidate **passes the Phase 7
candidate gate**. It improves MID-360 global and local trajectory accuracy plus
map geometry, while the HILTI position-only negative holdout remains unchanged.
The factor remains opt-in until another positive real-data sequence confirms
the gain independently.

The machine-readable verdict is stored outside the repository at:

```text
/media/sasaki/aiueo/benchmarks/phase7/plane_revisit_fresh_20260714/candidate_regression.json
```

Both dataset pairs use complete `public_suite_v1` manifests. The gate verifies
the benchmark profile, dataset contract, GT hash, raw backend-bag hash and
offline-runner executable hash before comparing metrics.

## MID-360 positive sequence

The same 341.25 s backend-input bag, dense frontend trajectory and verified
loop edge were replayed in both arms. Each arm produced byte-identical loop
edges and optimized trajectories across three runs. Map-quality reports were
also byte-identical across three evaluations per arm.

The GLIM trajectory is an independent cross-validation reference rather than
survey ground truth. Graph corrections were propagated from 640 anchors onto
2,772 dense frontend poses; 576 reference timestamps were scored.

| Metric | OFF | ON | Change | Gate |
| --- | ---: | ---: | ---: | :---: |
| dense ATE RMSE (m) | 1.214342 | 0.874364 | -28.00% | pass |
| translational RPE (%) | 0.194319 | 0.179518 | -7.62% | pass |
| planar thickness mean (m) | 0.043482 | 0.043393 | -0.20% | pass |
| planar coverage | 0.446315 | 0.446632 | +0.07% | pass |
| processing/sensor time | 0.094300 | 0.102398 | +8.59% | pass |

The ON graph uses 95 of 2,536 candidate constraints after rejecting 2,441 by
initial residual. Its sparse cross-validation APE is 0.751511 m versus
1.107267 m OFF; the denser contract above also confirms that the gain does not
come at the cost of local RPE.

## HILTI exp04 negative holdout

HILTI exp04 uses sparse position-only control points, so rotational RPE is not
claimed. The 44-anchor graph was propagated onto 1,258 dense frontend poses and
all seven GT checkpoints were associated. No plane-revisit constraint was
inserted, and OFF/ON trajectory and map artifacts were identical.

| Metric | OFF | ON | Change | Gate |
| --- | ---: | ---: | ---: | :---: |
| position-only ATE (m) | 0.071560 | 0.071560 | 0.00% | pass |
| planar thickness mean (m) | 0.061161 | 0.061161 | 0.00% | pass |
| planar coverage | 0.176490 | 0.176490 | 0.00% | pass |
| processing/sensor time | 0.011454 | 0.012881 | +12.46% | pass |

The runtime stage is only 3.2--3.6 seconds per run. The profile therefore
combines its relative 10% budget with a frozen absolute realtime-factor budget
of 0.005 (1.40 s for this 280 s bag); the measured 0.00143 increase is inside
that absolute budget.

## Setup provenance failure found during validation

An initial run selected a stale repository-local `install/` instead of the
fresh parent colcon workspace. Its old plane association inserted 230 factors
and severely regressed RPE and map geometry. The runner now prefers the parent
workspace setup, passes that exact setup into map-quality evaluation, prints
the selected executable SHA-256 and freezes the executable itself as a hashed
manifest input. Stale and current binaries can no longer be compared silently.

## Reproduction

Use `scripts/run_plane_revisit_candidate_benchmark.sh` once for
`mid360_public` and once for `hilti_exp04`. Pass the dense frontend trajectory
with `--dense-raw-tum` whenever the backend bag stores only submap-rate poses.
Then combine both manifest pairs with:

```bash
python3 scripts/evaluate_slam_candidate_regression.py \
  --baseline <mid360>/off/manifest/cross_repo_benchmark.json \
  --baseline <hilti>/off/manifest/cross_repo_benchmark.json \
  --candidate <mid360>/on/manifest/cross_repo_benchmark.json \
  --candidate <hilti>/on/manifest/cross_repo_benchmark.json \
  --output <suite>/candidate_regression.json --require-pass
```
