# Point-count-preserving planar map consolidation (2026-07)

## Decision

Keep the local-plane consolidation candidate default-off and advance it to
cross-sequence validation. On the exp04 development map it improved mean and
p95 plane thickness while preserving every point and slightly increasing
planar coverage. The gain is real but small, so this is not yet sufficient for
default-on adoption.

This stage changes only exported map point positions. It does not change scan
matching, trajectory output, loop closure, pose-graph optimization, saved-map
loading, localization, or relocalization.

## Algorithm contract

After the normal map leaf downsample and optional planar support filter:

1. accumulate deterministic point moments in a fixed 27-voxel neighborhood;
2. accept only neighborhoods with enough support and a strict planar
   eigenvalue signature;
3. reject points farther than 0.10 m from the fitted plane;
4. move accepted points halfway toward the plane, capped at 0.02 m;
5. preserve point count, point order, intensity, and all unsupported points;
6. return the input exactly when fewer than 10% of map points have safe planar
   support.

The implementation and standalone CLI are
`planar_map_consolidation.hpp` and `planar_map_consolidation`.

## exp04 paired result

Input:
`exp04_planar_map_filter_0p1_n3_s0p24_20260715_v1/map.pcd`, containing
1,556,077 points. Both sides used the same `map_quality_report` defaults.

Selected parameters:

```text
voxel_size=0.3
min_neighbors=12
max_small_eigenvalue_ratio=0.05
min_middle_eigenvalue_ratio=0.05
max_plane_distance_m=0.10
projection_gain=0.5
max_displacement_m=0.02
min_supported_ratio=0.10
```

The stage projected 254,002 points (16.323%), with 0.00807 m mean and 0.020 m
maximum displacement. It retained all 1,556,077 points.

| metric | baseline | consolidated | change |
|---|---:|---:|---:|
| mean plane thickness (m) | 0.031611919 | 0.031308483 | **-0.96%** |
| p95 plane thickness (m) | 0.063591811 | 0.063388258 | **-0.32%** |
| planar coverage | 0.822590399 | 0.823267101 | **+0.08%** |
| point count | 1,556,077 | 1,556,077 | unchanged |

A more permissive plane signature (`small<=0.10`, `middle>=0.02`) projected
476,552 points but improved mean thickness by only 0.18%; it was rejected.
Broader projection is therefore not supported by the development evidence.

## Verification and next gate

Synthetic tests verify thickness reduction, displacement bounds, point and
intensity preservation, insufficient-support fallback, and byte-identical
output. The component and standalone CLI build, and map-bundle parameters are
recorded deterministically.

Next run the selected frozen parameters without tuning on Construction Seq2,
HILTI exp07, and one outdoor/vegetation-heavy map. Adoption requires all three
geometry metrics to be no worse, exact point-count preservation, no fallback
misclassification, and an NDT localization residual check on at least one
sequence. Until then `use_planar_map_consolidation` remains false.
