# Selective visual fusion development gate (2026-07-15)

## Scope and inputs

This work uses only HILTI exp01 for development. Frozen holdouts exp02, exp03,
and exp21 remain unopened. Saved-map loading, localization mode, and
relocalization are out of scope.

Cam0 images were extracted with the official `0.0019071204 s` camera/IMU time
offset and official body/camera extrinsic. The fixed exp01 set has 911 images.
Deterministic Shi-Tomasi + forward/backward LK + Essential-RANSAC extraction
accepted 420 of 910 adjacent pairs (46.15%). The extractor hashes its transform
file and every image and records all rejection reasons. Five synthetic/unit
tests cover coordinate conversion, multi-E candidate handling, and the gate.

Artifact:
`/media/sasaki/aiueo/benchmarks/competitive_ours/exp01_visual_development_20260715/selective_visual_constraints.json`.

## Rejected offline correction

A bounded linear pose-correction prototype was evaluated only as a development
probe and removed from the repository. The adjacent-pair grid improved exp01
APE from `0.0655590872` to only `0.0655497868 m` (0.014%). Longer baselines of
0.5 and 1.0 seconds reached `0.0653469895 m` inside the 0.5 m / 3 degree
correction cap, only 0.32% better. More aggressive conditions exceeded the cap.
This misses the 3% development gate and cannot justify holdout evaluation.

Grid artifacts are retained under
`exp01_visual_development_20260715/refinement_grid_v1`,
`refinement_grid_v2`, and `long_baseline_grid_v1`.

## Online weak-direction scaffold

RKO-LIO now has a default-off core that:

- rechecks tracks, inliers, ratio, rotation agreement, translation agreement,
  and baseline;
- clamps translation and rotation independently;
- adds visual information only in low-information eigendirections of the
  LiDAR/IMU scan Hessian;
- consumes each timestamp-gated prior once and exactly preserves the legacy
  linear system when disabled or rejected.

Four C++ tests cover exact fallback, confidence rejection, weak-axis-only
fusion, and independent clamps. An exp01-only artifact adapter exists to test
the fusion law before implementing the final live cam0 frontend.

With normalized weak-direction threshold `1e-5`, the complete-prefix probe saw
347 time-matched priors but fused zero scans and zero directions. Its 1,749-pose
prefix was byte-identical to the retained squared-nearest-neighbor baseline.
The runner stopped after its 20-second no-new-pose guard, so this artifact is
not a completed accuracy result. It establishes instead that the point-to-point
ICP Hessian does not expose a sufficiently weak direction at this threshold;
raising the threshold blindly would no longer be degeneracy-selective.

Artifact:
`/media/sasaki/aiueo/benchmarks/competitive_ours/exp01_selective_visual_online_20260715_v2`.

## Live metric direct frontend

A default-off live cam0 frontend now projects deskewed LiDAR depth into the
official equidistant fisheye model and minimizes sparse 3x3 photometric patches
with affine exposure, Huber weighting, current-depth occlusion checks, and a
bounded metric SE(3) correction around the LIO prediction. The z-buffer keeps
the winning camera-frame 3D point and its subpixel projection; it does not
reconstruct rays from rounded depth pixels. Synthetic tests cover fisheye
round-trip, metric warp/exposure recovery, IMU-interval camera prediction, and
early rejection of a poor initial photometric fit.

The following exp01 development probes were rejected:

- raw single-scan depth at 0.25 s keyframe spacing: only 1 confidence-gate
  pass in 758 attempts;
- deskewed single-scan depth at 0.25 s spacing: 4 confidence passes in 566
  attempts, with moving-frame median RMSE `73.05` and inlier ratio `0.075`;
- projected local voxel map: worse moving-frame median RMSE `83.95` and inlier
  ratio `0.0266`, despite a verified world/camera transform direction.

Two time-consistency fixes materially changed the result. Reducing keyframe
spacing from 0.25 s to the 10 Hz LiDAR cadence reduced median moving baseline
from about 0.39 m to 0.134 m. More importantly, the camera initial pose now
uses the current interval's mean IMU angular velocity and acceleration, exactly
matching the LIO ICP motion model instead of reusing the previous scan's
velocity. On the complete v8 run this produced 102 solver-valid moving pairs
and 82 confidence passes out of 1,345 moving attempts. For valid moving pairs,
the direct estimate agreed with LIO at 0.176 degrees median rotation difference
and 0.988 median translation cosine.

At weak-information ratio `1e-4` and relative visual information weight `0.1`,
57 scans fused 91 weak directions without failure. Full exp01 APE changed from
`0.0655591` to `0.0654416 m` (0.18% improvement); all 13 control points were
matched. This is safe but below the 3% development gate. The diagnostic
frontend is also not real-time yet (`RTF 2.25`) because it attempts a direct
solve at every LiDAR frame. The 20-second runner quiescence fallback was shown
to kill a still-progressing process near scan 1,700; diagnostic runs therefore
use a longer fallback, while the frozen final RTF/completion gate is unchanged.

Complete artifact:
`/media/sasaki/aiueo/benchmarks/competitive_ours/exp01_direct_visual_live_20260715_v8`.

## Cross-sequence directional-observability gate

The v8 fusion law did not generalize without an additional visual
observability check. On `exp04`, four fused scans regressed APE by 0.82%. On
`exp07`, 277 fused scans / 553 directions regressed APE from `0.318106945` to
`0.342651 m` (7.7%). A weight-1 probe and a previous-Hessian activation probe
were also rejected on `exp01`.

The live direct solver now Schur-marginalizes affine exposure from its final
8x8 Hessian, transforms the 6x6 camera-pose information into the world left
tangent, and checks visual information independently along every LiDAR-weak
axis. At minimum visual directional-information ratio `1e-3`:

- `exp07`: all 753 weak directions rejected, reproducing baseline APE
  `0.318106945`;
- `exp01`: all 94 weak directions rejected, reproducing baseline APE
  `0.0655590872`.

The ratio distributions overlap (`exp01` maximum `6.21e-4`, `exp07` maximum
`7.19e-4`), so no scalar threshold preserves the small `exp01` gain while
safely excluding the `exp07` regression. Further threshold tuning was stopped.

## Decision

Reject trajectory-level Essential-pose correction and raw/local-map depth
variants. Retain the default-off, time-calibrated deskewed direct frontend and
directional-observability diagnostics, but do not promote visual corrections:
the only cross-sequence-safe setting falls back exactly to LiDAR--IMU output,
while ungated fusion violates the non-regression rule. No holdout sequence is
spent on this visual candidate.
