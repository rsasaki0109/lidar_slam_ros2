# Local surface-support map refinement gate (2026-07)

## Scope

This is a map-export-only refinement. It does not change scan matching,
pose-graph optimization, trajectory output, map loading, localization mode, or
re-localization. Candidate selection used HILTI 2022 `exp04`; the frozen
holdout sequences `exp02`, `exp03`, and `exp21` were not run.

The implementation evaluates covariance in the 27 neighboring voxels after
the configured map leaf downsample. A point is retained when its local
smallest-eigenvalue ratio has surface support. If the supported points fall
below `planar_map_filter_min_retained_ratio`, the original cloud is returned
unchanged. The manifest records the enable state and every filter parameter.

## Rejected candidates

- The temporal dynamic-object filter retained only 78,217 points on `exp04`
  and reduced planar coverage to 0.02388. It remains disabled.
- Strict plane thresholds at 0.3--0.5 m voxels supported only 4--9% of the
  saved points and therefore triggered the completeness fallback.
- The 70.84%-retained candidate (`0.1 m`, 6 neighbors, smallest ratio `0.20`)
  improved p95 thickness and coverage, but regressed mean thickness by 2.9%; it
  was rejected by the all-geometry-metrics non-regression rule.

## Selected development candidate

Parameters:

```yaml
use_planar_map_filter: true
planar_map_filter_voxel_size: 0.1
planar_map_filter_min_neighbors: 3
planar_map_filter_max_small_eigenvalue_ratio: 0.24
planar_map_filter_min_middle_eigenvalue_ratio: 0.0
planar_map_filter_min_retained_ratio: 0.90
```

Input artifact:
`/media/sasaki/aiueo/benchmarks/competitive_ours/exp04_map_leaf_0p1_20260715_v1/map.pcd`

Candidate artifact:
`/media/sasaki/aiueo/benchmarks/competitive_ours/exp04_planar_map_filter_0p1_n3_s0p24_20260715_v1`

The candidate retained 1,556,077 / 1,714,401 points (90.765%). All three
common-evaluator runs produced byte-identical reports
(`a07b9c80bf83a5e1377e97420a004597`).

| Metric | leaf=0.1 baseline | candidate | change | GLIM `exp04` |
|---|---:|---:|---:|---:|
| mean thickness (m) | 0.047656880 | 0.044526717 | -6.57% | 0.07911 |
| p95 thickness (m) | 0.108102794 | 0.096662150 | -10.58% | 0.12169 |
| planar coverage | 0.358716842 | 0.465592852 | +29.79% | 0.43030 |

This passes the development-sequence geometry gate and exceeds GLIM on all
three reported geometry metrics. It does not yet beat FAST-LIVO2 map geometry
(mean 0.03094 m, p95 0.08988 m, coverage 0.64464), and it is not a final
holdout verdict.

## Exact paired export ablation

The selected filter was then applied to the exact same integrated `exp04`
export, avoiding callback-scheduling differences between two SLAM replays.
The unfiltered cloud contained 1,796,042 points. The filtered cloud retained
1,625,322 points (90.4947%). All three evaluator reports for each side were
byte-identical.

| Metric | exact unfiltered export | exact filtered export | change |
|---|---:|---:|---:|
| mean thickness (m) | 0.047403070 | 0.043285999 | -8.69% |
| p95 thickness (m) | 0.109156499 | 0.095776113 | -12.26% |
| planar coverage | 0.344204447 | 0.449779220 | +30.67% |

An additional filter-enabled integrated replay completed map-bundle and
Autoware-compatible export verification. It produced 41 submaps, whereas the
unfiltered replay produced 40 because backend callback scheduling differed.
That replay is retained as integration evidence, not used as the paired
quality ablation. The exact-cloud comparison above is the selection evidence.

## Verification

- `graph_based_slam` Release component build: passed.
- `test_planar_map_filter`: 3/3 passed.
- `test_dynamic_object_filter`: 4/4 passed.
- `test_map_saver`: 17/17 passed.
- Map-bundle verifier: 5/5 passed.
- Common map evaluator: 3/3 byte-identical reports.

The integrated save path and exact paired ablation passed, so the candidate was
frozen before the untouched holdout suite. Holdout results are reported in the
competitive-SLAM plan rather than used to retune this filter.

## 2026-07-23 retained-ratio floor recalibration (0.90 -> 0.80)

### No-op discovery

Re-running the frozen filter parameters (`0.1 m` voxel, 3 neighbors, small
ratio `0.24`, middle ratio `0.0`) against two additional classic-map exports
at the 0.90 floor showed the completeness circuit breaker was silently
defeating the filter on most development geometry, not just the strict-plane
candidates already rejected above:

- `exp01` classic leaf-0.1 map retained 86.6% of points under the planar
  support test -- below the 0.90 floor, so `fallback_to_input` fired and the
  filter was a no-op.
- `exp07` classic leaf-0.1 map retained 87.5% -- likewise below 0.90, also a
  silent no-op.
- `exp04` (the original selection sequence) retained 90.77% -- it barely
  cleared the floor, which is why it was the only sequence where the filter
  had ever been observed to act.

In other words, the 0.90 floor was calibrated against a single sequence that
happened to sit just above it, and it suppressed the filter everywhere else
it was tried.

### Cross-sequence evidence at floor 0.80

The filter was re-run with `min_retained_ratio = 0.80` (all other parameters
unchanged) on three sequences, followed by
`bash scripts/run_map_quality_check.sh --downsample 0.1 --runs 1` on each
baseline and filtered cloud:

| Sequence | Retained (removed) | mean baseline -> filtered (m) | p95 baseline -> filtered (m) | coverage baseline -> filtered |
|---|---:|---:|---:|---:|
| exp01 (classic leaf 0.1, fresh) | 86.58% (13.42% removed) | 0.054380 -> 0.050412 | 0.115435 -> 0.106237 | 0.247473 -> 0.341905 |
| exp04 (frozen candidate map) | 90.77% (9.23% removed) | 0.047657 -> 0.044527 | 0.108103 -> 0.096662 | 0.358717 -> 0.465593 (exceeds GLIM `exp04` 0.430299) |
| exp07 (classic leaf 0.1, fresh) | 87.54% (12.46% removed) | 0.037872 -> 0.036087 | 0.094670 -> 0.085718 | 0.644345 -> 0.701942 |

All three metrics (mean thickness, p95 thickness, planar coverage) improved
on all three sequences. Removal stayed in a narrow 9.2-13.4% band -- the
filter is acting, but not aggressively.

### Decision

`min_retained_ratio` default changed from `0.90` to `0.80` everywhere it is
declared: `PlanarMapFilterConfig::min_retained_ratio` (`planar_map_filter.hpp`),
the component's `planar_map_filter_min_retained_ratio_` member and its
`declare_parameter` default (`graph_based_slam_component.h/.cpp`), the
`BundleManifest::planar_map_filter_min_retained_ratio` struct default
(`map_saver.hpp`), and the explicit value in `lidarslam/param/lidarslam.yaml`.
At 0.80 the circuit breaker still guards against pathological over-deletion
(it remains active for any sequence whose planar-supported fraction drops
below 80%), while no longer treating 86-88%-retained geometry as a reason to
skip the filter entirely.

A focused gtest was added to `test_planar_map_filter.cpp` asserting (a) the
default-constructed `PlanarMapFilterConfig::min_retained_ratio` equals `0.80`,
and (b) the fallback still trips at the new default when a synthetic cloud's
planar-supported fraction (~66.7%) is below it.

Verification: `graph_based_slam` Release rebuild succeeded; `ctest -R planar`
(`test_planar_map_filter`, `test_planar_map_consolidation`) and
`test_map_saver` all passed; `ament_uncrustify` reported no divergence on the
edited files; re-running the rebuilt `planar_map_filter` binary with default
arguments against the frozen `exp04` map reproduced the exact frozen result
(1,556,077 / 1,714,401 retained, 90.765%) and the map-quality evaluator
reproduced the byte-identical report hash `a07b9c80bf83a5e1377e97420a004597`
(mean 0.044526717 m, p95 0.096662150 m, coverage 0.465592852).

### Caveat

This is dev/regression evidence only, gathered from classic-map exports and
the already-frozen `exp04` candidate map. The frozen holdout sequences
`exp02`, `exp03`, and `exp21` were not run and remain untouched by this
change; no holdout artifact or frozen manifest yaml was modified.
