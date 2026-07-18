# Camera-coloured map release readiness (2026-07)

## Decision

The camera-coloured map work is ready to ship as an opt-in, quality-gated
pipeline. The selected public example remains the K3 Construction Seq1 map.
Spatiotemporal calibration, dynamic masks, calibration-uncertainty propagation,
and dataset-specific geometry margins are available without changing legacy
defaults. Geometry boundary margins are not promoted for Construction Seq1.

This distinction matters: the architecture and diagnostics are production
ready, but a new guard is enabled by default only after paired full-density
validation passes the existing quality profile. The release does not exchange
surface consistency for a better aggregate colour score.

## Selected map and media

The K3 map contains 4,840,318 points. Of those, 3,618,693 receive camera colour
for coverage 0.747615. It uses overlap RGB balancing, view confidence, a
120-pixel image margin, vignette gain limit 2.5, and at least three camera
observations. The report-only Construction Seq1 profile has 11 checks and no
violations:

- held-out RGB L2 median 41.17 (limit 42.0) and inlier-20 fraction 0.2863
  (minimum 0.28);
- global roughness median/p90 5.43/20.67 (limits 6.0/22.0);
- planar roughness median/p90 7.23/23.89 (limits 7.6/25.0);
- chroma retention 1.0016 and appearance coverage 0.7476.

The README animation is rendered from K3 with the CPU surface-splat and
cinematic-camera path. Against the previous README animation, occupied-pixel
fraction improves from 0.78820 to 0.82135 and temporal-flicker p90 falls from
0.00800 to 0.00668. Five points across the encoded loop were visually checked.

| asset | dimensions | rate | frames | duration |
| --- | ---: | ---: | ---: | ---: |
| `map_flythrough_rtkslam.mp4` | 600x450 | 30 fps | 240 | 8 s |
| `map_flythrough_rtkslam.gif` | 480x360 | 10 fps | 80 | 8 s |
| `map_flythrough_rtkslam.webp` | 600x450 | 15 fps | 120 | 8 s |

WebP is the README primary; MP4 and GIF are linked fallbacks. The source
sequence is RTK-SLAM Construction Hall 1, distributed under CC-BY 4.0.

## Architecture and promotion evidence

1. The 7DoF spatiotemporal calibration recomposes camera poses from the dense
   trajectory and accepts a correction only when training and held-out edge
   loss improve, bounds and observability pass, and covariance is valid. The
   Seq1 smoke improved training/held-out mean loss by 9.20%/7.85%.
2. Geometry-aware fusion applies z-buffer silhouette, depth-edge, dynamic-mask,
   and uncertainty-aware guards before robust colour selection. Diagnostics
   retain separate rejection causes.
3. Full-density candidate N improved coverage and held-out colour relative to
   its corrected-pose baseline O, but worsened planar roughness to 8.67/28.02.
   It was rejected against the unchanged 7.6/25.0 limits. Boundary margins
   therefore remain zero by default.
4. Edge-aware sampling no longer materializes an `Mx4x3` corner stack. The
   one-million-coordinate kernel is 63.0% faster with 11.6% lower process RSS;
   a paired 484k-point/260-image screen is 25.0% faster with an identical
   report and byte-identical PLY SHA-256.

Detailed evidence:

- [spatiotemporal calibration](colored-map-spatiotemporal-calibration-2026-07.md)
- [geometry-aware fusion and paired A/B](colored-map-geometry-aware-fusion-2026-07.md)
- [performance and output equivalence](colored-map-fusion-performance-2026-07.md)

## Compatibility and safe defaults

All new correction and rejection behavior is opt-in. Without the corresponding
flags, pipeline staging, output selection, sampling behavior, and public CLI
defaults remain compatible. The geometry path requires a one-pixel z-buffer;
dynamic exclusion requires a complete, dimension-checked mask set; uncertainty
propagation requires an accepted calibration with valid seven-parameter
uncertainty and strictly increasing timestamps.

The following are not release claims:

- automatic semantic segmentation (the core consumes external PNG masks);
- universal geometry-margin values across cameras, rigs, or datasets;
- independent Seq2 or cross-rig calibration promotion;
- a completed world-map quality result from the camera-only realtime bag,
  which did not contain a finished map topic.

## Verification

- focused coloured-map Python suite: 134 passed, 4 skipped;
- ament flake8: passed;
- paired output-equivalence benchmark: passed;
- GitHub CI: Humble, Jazzy, docs/release metadata, release readiness, and the
  negative threshold guard passed on run `29662475090`.
