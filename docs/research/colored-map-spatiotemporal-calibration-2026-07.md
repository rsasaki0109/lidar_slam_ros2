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
a deterministic image pyramid. Each coordinate sweep tests both one- and
two-step moves so pixel-quantisation plateaus do not trap the search. Bounds
that are closer than the next search step are expanded and searched again.

The user-facing pipeline is deliberately staged as:

```text
posed images -> uncoloured calibration geometry -> 7DoF calibration
             -> corrected camera poses -> final robust RGB map -> quality gates
```

The feature is opt-in. Without `--refine-spatiotemporal-calibration`, command
composition and output paths are unchanged.

## Safety and validation gates

- Held-out views are selected deterministically across travelled-distance
  segments and static/translation/rotation motion strata.
- Training loss must decrease and held-out loss must decrease independently.
- A configurable minimum number of LiDAR depth-edge pixels is required in both
  partitions.
- A nearly static trajectory is rejected because time offset is unobservable.
- Time, translation, and rotation corrections have independent hard bounds.
- Source camera poses must reproduce one static extrinsic within 1 mm and 0.05
  degrees; otherwise their timestamps/conventions are treated as inconsistent.
- Parameters closer to a search bound than the next search step are listed in
  `boundary_axes`; a production candidate cannot be promoted while any remain.
- The observability audit runs at the finest pyramid resolution. Five samples
  per axis provide one- and two-step centred curvatures; both must be positive
  and the estimated stationary point must remain local.
- Three 3x3 quadratic clock/translation surfaces must be positive definite.
  The normalized Hessian condition number and time/translation correlations
  are bounded, and covariance is emitted only for a positive-definite Hessian.
- Rejected candidates export the original camera poses, never an unvalidated
  correction, and record the rejection reason in JSON.
- Frame timestamps and original images are preserved in the corrected
  `transforms_spatiotemporal.json` dataset.

## RTK-SLAM Construction Seq1 production smoke result

The production CPU smoke used the existing K3 4.84 M-point map,
deterministically sampled to 200,000 points, 13 of 260 camera views, pyramid
scales 0.25/0.5/1.0, and two search rounds per scale. Nine views were training
data and four were held out by the travelled-distance/motion split. Initial
bounds were 40 ms / 4 cm / 0.5 degrees and expanded only for axes within one
search step of a bound.

| metric | initial | refined |
| --- | ---: | ---: |
| training mean edge distance | 6.539 px | 5.937 px |
| training out-of-range fraction | 0.250 | 0.217 |
| held-out mean edge distance | 6.522 px | 6.010 px |
| held-out median edge distance | 6.000 px | 5.000 px |
| held-out out-of-range fraction | 0.241 | 0.216 |

Training and held-out mean loss improved by 9.20% and 7.85%, respectively. No
axis remained near its final bound. All seven normalized Hessian eigenvalues
were positive (0.0369 to 0.0827), the condition number was 2.24, and maximum
absolute clock/translation correlation was 0.105. The correction therefore
passed the production acceptance gate and was exported. This validates the
calibration stage on Seq1; independent Seq2 and cross-rig promotion remain a
separate dataset-level gate.

The regression suite covers deterministic pattern search, step-aware bounds,
multi-scale curvature and undefined-correlation handling, stratified splitting,
continuous-time pose recomposition, static-motion degeneracy, independent
held-out rejection, pipeline staging, cache reuse, and unchanged opt-in command
composition.

## K5 residual diagnostics

An aggregate edge-distance score can hide whether a poor map is caused by one
constant camera correction or by view-dependent timing and pose errors. The
alignment evaluator therefore has an optional diagnostic output:

```bash
python3 scripts/evaluate_lidar_camera_alignment.py \
  --pointcloud coloured.ply --transforms posed/transforms.json \
  --out alignment.json --diagnostics-dir alignment_diagnostics \
  --worst-views 10
```

Each selected view records the median signed x/y displacement from projected
LiDAR depth edges to their nearest strong image edges. The diagnostic directory
contains JSON, the worst-view overlays, and a contact sheet. Image edges are
green; LiDAR edges progress from cyan through yellow to red as residual grows,
and unmatched edges are magenta. A stable signed direction across views points
to a static extrinsic error. Large changes between views instead point to clock,
motion distortion, rolling shutter, or trajectory error. These images are
diagnostic evidence, not a replacement for independent held-out acceptance.

The first full-resolution K4 audit used all 4,906,133 geometry points and 26
views. The ten worst views had 36.5% to 54.2% unmatched depth edges, despite a
12 px search radius. Across all matched edges, the weighted direction was only
(-0.027, -0.126) px and direction coherence was 0.032. The overlays show broad,
scene-dependent residuals on shelves, ceilings, and object boundaries rather
than one consistent translation. This rules out treating K4 as a simple static
extrinsic nudge. Timing, motion distortion, and trajectory-conditioned residuals
must therefore be tested independently.

### Surface-supported edge ablation

The evaluator also provides an opt-in same-surface support filter. A projected
depth-edge pixel is retained only when nearby finite depths agree within an
absolute and range-relative tolerance. Reports always include the raw edge
count and retained fraction so filtering cannot improve a score merely by
discarding difficult observations. Calibration additionally supports a minimum
retained-fraction rejection gate; the pipeline uses 25% when this filter is
enabled.

The full-density K4 `radius=2, min_neighbors=4` ablation retained 51.42% of raw
edges. Median residual improved only from 7.759 to 7.234 px, 2 px inliers from
22.31% to 23.12%, and out-of-range residuals from 34.96% to 32.79%; p90 remained
saturated at 13 px. With the production 300,000-point calibration subsample,
only 7.62% survived. Its apparently lower 4.59 px median is selection bias and
fails the 25% retention gate. This filter is useful for visual diagnosis but is
not a K5 calibration candidate. The next objective needs correspondence support
that remains meaningful under sparse geometry rather than image-plane density
alone.

### Fixed full-density 3D contours

The next candidate extracts visible depth-edge winner IDs from the complete
4.91 M-point geometry before applying `--max-points`. Each view retains a
deterministically image-distributed cap of those world-space points. During
7DoF search the same 3D points are reprojected for every candidate, so neither
the 300,000-point calibration subsample nor candidate-dependent edge detection
can change the objective's population. Pipeline support is opt-in through
`--calibration-fixed-contours` and `--alignment-fixed-contours`.

Image-associated contour selection is diagnostic-only. An ablation selecting
points initially within 12 px of an image edge made zero correction the exact
optimum: both training and held-out loss changed by 0%. The CLI now rejects
that mode during optimization. Production calibration requires geometry-only
selection (`--contour-association-distance 0`).

A geometry-only smoke used 13 stratified views and 20,000 fixed contour points
per view. All 260,000 points survived the 300,000-point calibration condition.
The candidate reduced training loss by 2.25%, but held-out loss by only 0.76%
(7.3723 to 7.3160 px), below the required 2%. The observability audit also
rejected a stationary point outside the local neighbourhood. No corrected pose
was adopted. This establishes density-independent evidence and safe rejection,
but does not yet improve K4 colour registration; fixed nearest-image-edge
distance remains too weak and ambiguous in the cluttered warehouse.

### Per-pixel orientation-aware correspondence

Fixed contours can optionally carry the unit normal of their originating depth
discontinuity. `--orientation-max-angle-deg` then requires the unoriented depth
normal and image-gradient normal to agree as well as satisfying pixel distance.
Angle-rejected contours remain saturated residuals rather than disappearing
from the population. The option is default-off and requires fixed contours.

The 30-degree production smoke retained all 260,000 fixed contour points but
left 61.24% without a valid match. Training loss improved only 1.09% and
held-out loss 0.52%, both worse than distance-only contours. The solution also
reached search bounds and failed observability because of insufficient and
unstable curvature, an unobservable time/translation pair, an out-of-local
stationary point, and ill-conditioning. It was rejected and original poses were
exported.

Pixel normals from a sparse depth raster are not stable enough around shelves,
corners, and thin structures. The next candidate should group contour pixels
into supported line segments and estimate one robust tangent per segment,
rather than loosening this per-pixel gate until it becomes distance-only again.

### Objective power check (2026-10)

Before building segment tangents, the nearest-edge objective itself was tested.
The setup reused the K4 geometry, the 26 views at stride 10, and fixed geometry-only
contours capped at 50,000 per view. Each view's pose was perturbed by a known amount,
and the unchanged metric was recomputed against the image edges at the 95th
gradient percentile.

| Pose given to the metric | median | 2 px inliers | unmatched (>12 px) |
| --- | --- | --- | --- |
| current calibration | 7.62 px | 22.0% | 36.0% |
| camera yaw +0.5 deg | 7.62 px | 22.0% | 36.1% |
| camera yaw +1 deg | 7.81 px | 21.9% | 36.2% |
| camera yaw +3 deg (~44 px at f=849) | 8.00 px | 21.7% | 36.7% |
| camera x +10 cm | 7.62 px | 22.0% | 36.1% |
| contours scored against a different view's image | 8.94 px | 20.3% | 40.8% |

- **Chance agreement:** a 3 deg error moves every point by tens of pixels, yet the
  median changes by 0.4 px. Scoring against an unrelated image keeps 20.3% of 2 px
  inliers, so only ~1.7 points of the 22.0% reflect real alignment. In this cluttered
  warehouse the 95th-percentile image edges are dense enough that almost any projected
  point finds one within a few pixels.
- **Not caused by see-through edges:** a 3D test split the contour points by their
  full-density neighbourhood (largest tangent-plane angular gap >= 120 deg within
  0.10 m). Boundary points (median 7.0 px, 34% unmatched) and surface-interior points
  (7.8 px, 36%) scored alike.
- **Not caused by uncorrected lens distortion:** matched offsets show no radial bias.
  50.0-50.8% point outward in every radius band, with a mean radial offset of
  -0.01 to -0.12 px.

Consequence: the flat loss surfaces, failed observability checks and sub-1% held-out
gains above are what this metric produces at any pose. Contour-side refinements
(support filters, fixed contours, orientation, segment tangents) cannot add the
missing information.

Cross-view colour agreement does not help with a constant extrinsic error either. The
check sampled 200k points visible in at least three of 52 views (stride 5), kept the
textured ones (image gradient at or above the 80th percentile, 131k), and measured the
per-point RGB standard deviation across views:

| Pose | median RGB std | p90 |
| --- | --- | --- |
| current calibration | 58.8 | 102.4 |
| camera yaw +3 deg | 58.7 | 103.5 |
| camera yaw +10 deg (~150 px) | 60.9 | 105.2 |
| chance (colours shuffled within each view) | 89.8 | 114.0 |

The current poses agree far better than chance, yet even 10 deg barely changes the
score. A constant camera-frame rotation shifts every view's sample by the same angle,
so the views keep agreeing with each other while all being wrong. Held-out colour error
of a recoloured map has the same blind spot. Such signals can detect errors that differ
between views (time offset under motion, per-frame pose error), not a constant
extrinsic rotation.

A constant extrinsic needs a signal tied to the LiDAR geometry itself.

### LiDAR-intensity mutual information (2026-10)

The K4 PLY stores no intensity. For each view, the raw `/livox/points` scans within
±0.3 s of the image stamp were deskewed with the same trajectory as
`build_lidar_init.py`, which gives about 67k points per view. They were z-buffered into
the view with a 140 px vignette margin, and the (intensity, grey) pairs were scored by
mutual information (32×32 bins).

**The point population must be fixed.** A pose change moves points across the
margin and changes z-buffer winners, so MI over "whatever projects" mixes alignment with
population. A first table without that control suggested a +2 deg pitch optimum, about
+5% on both search and confirmation views. Scoring only the points that win the
z-buffer under both poses, and pooling them over the views, changes the picture.
Search views are at stride 10 from view 0; confirmation views are at stride 10 from
view 5.

| Perturbation | search views | confirmation views |
| --- | --- | --- |
| pitch -1 deg | -12.2% | -9.0% |
| yaw +1 deg | -8.0% | -10.1% |
| pitch +1 deg | +5.3% | +0.7% |
| pitch +2 deg | +4.4% | -1.6% |
| camera y -5 cm | +8.0% | +2.3% |

- **MI is a pose-sensitive signal.** Wrong poses cost 8-12% per degree, unlike the
  edge and colour signals above.
- **The +2 deg optimum does not survive.** On the confirmation views it gives -1.6%.
- **The recoloured map agrees.** Recolouring the K4 geometry with the configuration-I
  options (`recolor_pointcloud.py --image-margin 120 --vignette-gain-limit 2.5
  --min-samples 3`) under the current poses and under pitch +2 deg, then scoring map
  luminance against LiDAR intensity on about 1.4 M scan points matched within 2 cm,
  gives 0.0449 vs 0.0426 (search) and 0.0444 vs 0.0419 (confirmation). The corrected
  map is worse.
- **Appearance roughness does not discriminate.** Median 4.98 vs 4.95; planar median
  6.10 vs 7.09.
- **Conclusion:** the current calibration is within about ±1 deg (or a few cm)
  vertically. The remaining colour blur is not explained by a constant extrinsic error,
  and no correction was adopted. Artefacts are in
  `benchmarks/rtkslam_seq1_colored_map_20260718/k5_mi_check/`.

Any future K5 objective must hold the scored population fixed across candidate poses,
and must pass the perturbation table on held-out views before it is optimized.

### Time offset and per-view refinement (2026-10)

- **Time offset is fine.** Each camera pose was re-interpolated at stamp + dt with the
  inferred body-to-camera transform, and fixed-population MI was scored. The optimum
  lies within ±10 ms on both view sets, and ±40-80 ms costs 2-11%. At a median camera
  speed of 0.6 m/s, 10 ms is about 6 mm.
- **Per-view MI optima look real.** A pitch/yaw search over ±1.5 deg on every one of the
  260 views (0.5 deg grid, then 0.25 deg refinement) found none of them at the current
  pose. Split-half point sets agreed within one 0.5 deg step on 92% of the views, and
  the corrections are temporally correlated (lag-1: pitch 0.50, yaw 0.35; median
  |correction|: pitch 1.0 deg, yaw 0.5 deg).
- **They make the colours worse.** Held-out RGB was evaluated independently of LiDAR
  intensity, with `evaluate_heldout_point_colors.py` fold 0. The training fold was
  recoloured with the configuration-I fusion options and the held-out views scored with
  a 140 px margin:

| Poses | RGB L2 median | p90 | inlier 20 |
| --- | --- | --- | --- |
| current | **36.3** | **148.5** | **33.3%** |
| per-view MI correction | 39.5 | 152.6 | 30.1% |
| same, temporally median-smoothed (5 views) | 39.2 | 152.7 | 30.6% |

The per-view MI maxima are reproducible but biased. Intensity depends on range and
incidence, and image brightness on lighting and vignetting, so the MI peak shifts with
scene structure. Split halves see the same structure and reproduce the bias. MI is
useful for detecting errors of a degree or more. It is not accurate enough to refine
these poses, which every signal tested here leaves within about 1 deg and 10 ms.

K5 stops here with the current K4 poses. The remaining colour blur should be looked
for in fusion and rendering rather than in camera registration. Artefacts are in
`benchmarks/rtkslam_seq1_colored_map_20260718/k5_mi_check/`.
