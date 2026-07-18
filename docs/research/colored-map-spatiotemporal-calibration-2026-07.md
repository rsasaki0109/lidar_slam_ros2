# Coloured-map spatiotemporal calibration (2026-07)

## Motivation

Camera-coloured point clouds turn small timing and extrinsic errors into colour
halos at walls, pillars, and object boundaries. The previous `--time-offset
auto` aligns camera and LiDAR clock domains, while the alignment evaluator's
optional correction searched only a 6DoF camera pose. Neither recomposed every
camera pose from the continuous SLAM trajectory while jointly searching the
residual clock offset.

## Architecture

`evaluate_lidar_camera_alignment.py --optimize-spatiotemporal` now optimizes
seven bounded parameters:

1. residual camera-to-trajectory time offset;
2. local optical-frame x/y/z extrinsic translation;
3. local optical-frame roll/pitch/yaw extrinsic rotation.

For every candidate it interpolates `world <- body` from the dense TUM
trajectory and composes it with the refined `body <- camera` transform. The
objective is the coverage-guarded distance from projected LiDAR depth
discontinuities to strong image gradients. Image edge masks are cached across
the deterministic coarse-to-fine coordinate search.

The user-facing pipeline is deliberately staged as:

```text
posed images -> uncoloured calibration geometry -> 7DoF calibration
             -> corrected camera poses -> final robust RGB map -> quality gates
```

The feature is opt-in. Without `--refine-spatiotemporal-calibration`, command
composition and output paths are unchanged.

## Safety and validation gates

- Every Nth selected view is held out before optimization.
- Training loss must decrease and held-out loss must decrease independently.
- A configurable minimum number of LiDAR depth-edge pixels is required in both
  partitions.
- A nearly static trajectory is rejected because time offset is unobservable.
- Time, translation, and rotation corrections have independent hard bounds.
- Source camera poses must reproduce one static extrinsic within 1 mm and 0.05
  degrees; otherwise their timestamps/conventions are treated as inconsistent.
- Parameters that land on a search bound are listed in `boundary_axes` for
  promotion review.
- Rejected candidates export the original camera poses, never an unvalidated
  correction, and record the rejection reason in JSON.
- Frame timestamps and original images are preserved in the corrected
  `transforms_spatiotemporal.json` dataset.

## RTK-SLAM Construction Seq1 smoke result

The first CPU smoke run used the existing K3 4.84 M-point map, deterministically
sampled to 200,000 points, 13 of 260 camera views, two search rounds, and tight
bounds of 40 ms / 4 cm / 0.5 degrees. Ten views were training data and three
were held out.

| metric | initial | refined |
| --- | ---: | ---: |
| training mean edge distance | 6.444 px | 6.004 px |
| held-out mean edge distance | 6.887 px | 6.375 px |
| held-out median edge distance | 6.325 px | 5.385 px |
| held-out out-of-range fraction | 0.285 | 0.251 |

The held-out mean improved by about 7.4%, so the candidate was accepted. The
solution reached a translation bound on two axes; this establishes end-to-end
functionality but is not a production calibration claim. Promotion requires a
wider multi-scale search followed by rebuilding the map and running held-out
RGB, appearance, trajectory, and geometry gates.

The regression suite covers deterministic search, hard bounds, continuous-time
pose recomposition, static-motion degeneracy, independent held-out rejection,
pipeline staging, cache reuse, and unchanged default command composition.
