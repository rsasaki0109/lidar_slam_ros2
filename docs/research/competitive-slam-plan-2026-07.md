# GLIM / FAST-LIVO2 competitive SLAM plan (2026-07-14)

## Objective

Beat GLIM on a LiDAR-IMU SLAM track and FAST-LIVO2 on a LiDAR-IMU-visual
track without mixing their sensor contracts. Saved-map loading, localization
mode, and relocalization remain out of scope.

The machine-readable contract is
`configs/slam_benchmark_profiles/competitive_slam_v1.yaml`. It freezes three
repetitions, identical inputs and calibration, a 3% primary-accuracy margin,
RTF at most 1.0, peak RSS at most 1.2 times the rival, map-quality
non-regression, zero verified false loops, and three held-out wins.

The benchmark host fingerprint is
`f5605cb93280749c7837ecffddebc622685e2155ad58ec83877c8a3653727c80`
(x86_64, Intel i5-1145G7, 8 logical CPUs, 32,461,332 kB MemTotal). Its
non-secret evidence artifact is
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/machine_fingerprint.json`
(SHA-256 `f9e496aba9a1f5f80f724a07ad4384546eb6857d65fcc1b09da3b5e85175da5f`).
Competition runners now record the same fingerprint directly in every new
run's provenance; raw OS/DMI identifiers contribute only to the hash and are
never written to artifacts.

## Rival freeze

- GLIM core v1.2.2 commit: `faa264a1bce1bda406f73457e35511f56cdc2eaa`
  (annotated tag object: `2044c47825cff7b508e5ea96c1d295e023b0e36c`)
- GLIM ROS 2: `4a9e7a4cb084967c8525a1be529ad3ba2a118ae7`
- FAST-LIVO2: `0d2c0346107b75b59934975adec9a6eeeb913c64`

GLIM uses direct multi-scan registration factors and supports CPU/GPU
estimation modules. FAST-LIVO2 is a ROS 1 LiDAR-inertial-visual system whose
official build targets Ubuntu 18.04--20.04. It is therefore isolated in a
pinned container rather than mixed into the ROS 2 workspace.

## Dataset policy

Construction Seq2 is suitable for runner bring-up because it contains Livox,
IMU, monocular images, and total-station checkpoints, but it has already been
used for loop-gate development and cannot count as a holdout. HILTI exp01 is a
development substrate; exp04, exp07, and the public MID-360 capture are
regression-only. Before inspecting results or tuning, the three holdout slots
were assigned on 2026-07-14 to HILTI exp02 (multilevel), exp03 (stairs), and
exp21 (outside). Their official URLs and expected byte sizes are frozen in the
machine-readable contract. Exp03 and exp21 are now `frozen`; exp02 remains
`assigned_inputs_pending_hash` until its downloaded input, calibration, and
ground-truth file has been hashed. No algorithm tuning may begin before all
three have completed that transition and both official rival baselines exist.

`scripts/freeze_hilti2022_holdout_inputs.py` rejects partial raw bags, missing
LiDAR/IMU topics, invalid raw-hash sidecars, or absent converted bags, then
records raw ROS1, canonical ROS2 tree, ground-truth, and calibration hashes.
All three competition runners accept `--input-manifest` and refuse a raw or
converted bag whose representation-specific hash differs from that frozen
manifest. FAST can mount an external dataset bag read-only, so no untracked
copy is needed.

GLIM cross-validation trajectories are not ground truth and cannot support an
absolute-accuracy claim.

## Development sequence

1. Build official-rival runners that emit the existing common benchmark JSON.
2. Measure three untouched baselines and produce a per-interval error gap map.
3. Improve fixed-lag multi-scan LIO and persistent weak-direction control for
   the GLIM track.
4. Add time-calibrated, selective direct visual updates for the FAST-LIVO2
   track, activating visual work most strongly in LiDAR-degenerate directions.
5. Integrate verified loops and deterministic map refinement.
6. Freeze code and run the three assigned holdouts without retuning.

The current host has Docker but no GPU visible through `nvidia-smi`. CPU GLIM
is the first formal track; CUDA GLIM remains a separate conditional track.

## Measured development priorities

The exp04 regression measurements define the implementation order without
using any holdout result:

| Track | Gate evidence | Current state | Development priority |
|---|---|---|---|
| GLIM trajectory | APE 0.05654 vs 0.08662 m | pass by 34.7% | preserve |
| GLIM runtime/RSS | RTF 0.993; RSS ratio 0.850 | pass | preserve, remove cold-run jitter |
| GLIM map | mean pass; p95 +2.8%; coverage -58.3% | fail | dense pose-refined fusion first |
| FAST trajectory | APE 0.07146 vs 0.05298 m | fail by 34.9% | selective direct visual update |
| FAST map/colour | coverage -20.2%; colour -2.7%/-3.3% | fail | visual confidence + denser fusion |
| FAST processing RTF | conservative upper bound 0.99949 | pass | preserve |

Accordingly, fixed-lag multi-scan work must preserve the already-winning GLIM
trajectory while increasing planar support and tightening the p95 tail. Visual
fusion must first close trajectory error; colour-map polishing alone cannot
meet the primary accuracy gate.

`scripts/compare_checkpoint_trajectory_errors.py` applies the same
interpolation and independent SE(3) alignment to both systems, then emits the
per-checkpoint gap. On the seven-point FAST track, lidarslam wins 2/7 and loses
5/7; its largest deficit is +0.0633 m at the first checkpoint, followed by
+0.0217 m near 41 seconds and +0.0164 m near the end. On the common six-point
GLIM track, lidarslam wins 5/6; its only deficit is +0.0501 m at the second
checkpoint. This localizes visual work to initialization and accumulated drift,
while GLIM work should prioritize map coverage and one local weak interval.

## Frozen implementation work packages

Algorithm work remains disabled until all three rival holdout baselines and
input manifests are complete. Once that phase gate opens, changes are staged
behind default-off flags and tuned only on exp01 plus the regression set:

1. **Fixed-lag multi-scan LIO.** Retain deskewed keypoints, map-update points,
   timestamps, and linearization poses for a bounded recent window. Rebuild the
   recent voxel layer after pose corrections instead of permanently baking
   early pose error into one map. Marginalize the oldest pose into a prior;
   publish the newest pose/covariance through the unchanged odometry contract.
2. **World-consistent weak-direction control.** Transport Hessian eigendirections
   into one frame before persistence matching, handle repeated weak eigenspaces
   as subspaces rather than arbitrary individual eigenvectors, and blend only
   the confirmed weak component with the IMU prediction. Observable components
   remain geometry-driven.
3. **Selective direct visual update.** Subscribe only to the frozen cam0 stream,
   use the official camera/IMU/LiDAR calibration, estimate one bounded time
   offset on development data, and hold it fixed for evaluation. Apply robust
   photometric residuals only to projected LiDAR points with valid depth,
   gradient, exposure, and occlusion checks. Allocate visual weight primarily
   to LiDAR-weak directions; fall back exactly to LIO when confidence is low.
4. **Pose-refined dense map fusion.** Keep mapping points separate from sparse
   registration keypoints, reproject recent scans after fixed-lag changes, and
   rebuild final tiles from optimized poses after verified loop closure. This
   directly targets coverage while preserving the existing thickness advantage.

Each package needs deterministic synthetic tests, an exp01 ablation, exp04/07
regression checks, RTF/RSS accounting, and a rollback trigger. No code path may
load a saved map, enter localization mode, or attempt relocalization.

### 2026-07-15 LiDAR--IMU checkpoint

The multi-scan observability and selective-compute experiments are recorded in
[`multiscan-observability-gate-2026-07.md`](multiscan-observability-gate-2026-07.md).
Normalized Hessian windows, cached local-surface weighting, fixed/adaptive
iteration caps, and fixed coarse ICP keypoints each failed an exp01 or exp07
accuracy gate. None was evaluated on frozen exp02/03/21.

One exact optimization was retained: voxel nearest-neighbor candidates are
compared using squared distances and `sqrt` is evaluated only for the winner.
Complete exp01 APE stayed exactly `0.0655590872` m (13/13 checkpoints) while
processing RTF improved from `3.0761` to `2.0612`, a 33.0% reduction. This is a
safe speed improvement, but dense exp01 is not real-time yet. Selective visual
fusion is now the active work package; our final holdout runs remain unopened.
The first deterministic cam0 study is documented in
[`selective-visual-fusion-gate-2026-07.md`](selective-visual-fusion-gate-2026-07.md):
420/910 exp01 pairs passed the confidence gate, but bounded adjacent and
long-baseline Essential-pose correction improved APE by only 0.014% and 0.32%,
respectively. Both miss the 3% gate. A default-off online weak-direction
scaffold is now unit-tested; at a `1e-5` normalized Hessian threshold, 347
time-matched priors produced zero eligible directions and an exact baseline
trajectory prefix. The next visual candidate is therefore a live photometric
patch residual with correspondence/surface-level observability, not a looser
threshold on the same point-to-point 6x6 Hessian.

## FAST-LIVO2 exp04 runner validation

The official FAST-LIVO2 revision was replayed three times on HILTI exp04 in
the pinned `fast-livo2-benchmark:noetic` image. This is regression evidence,
not a holdout win. The frozen artifacts are under
`/media/sasaki/aiueo/benchmarks/fast_livo2/results/competitive_v1_exp04_3x_20260714`.

- bag SHA-256: `1ea856f82330d8258c6bdcdd92677a6736767df341a16075df087de77766aa8f`
- reference SHA-256: `38cf516e51113254e4ae0207c790f740b19dee08665063e0d8df7bd277040c20`
- all three runs: 1,255 poses, complete trajectory, process exit 0
- APE RMSE: 0.052982 / 0.053073 / 0.052980 m; median 0.052982 m
- maximum peak RSS: 4,719.63 MB
- one-times playback wall RTF median: 1.0090 (diagnostic only)

FAST consumes ROS1 while the other systems consume the converted ROS2 bag, so
container-file hashes alone cannot prove equal input. The audit tool
`scripts/compare_rosbag_semantic_inputs.py` deserializes both bags and hashes
each record timestamp and every message field, including complete point-cloud
and image payloads. Only ROS1's removed transport field `std_msgs/Header.seq`
is excluded. On exp04 all 1,258 LiDAR, 50,198 IMU, and 5,029 camera messages
had identical per-topic aggregate SHA-256 values across ROS1 and ROS2. The
proof artifact is `semantic_input_equivalence.json` beside the three-run FAST
results.

The GLIM track uses a six-checkpoint subset because GLIM's three-second
initialization cannot cover the first exp04 checkpoint. That restriction does
not apply to the FAST track. Rescoring the untouched three-run lidarslam
trajectories against the same full seven-checkpoint reference used by FAST
gives a repeat-identical APE RMSE of 0.071463 m, versus FAST's 0.052982 m
median. Lidarslam is therefore 34.9% higher-error on this regression sequence;
the earlier six-point GLIM score must not be reused for the FAST comparison.

The existing even-view/odd-view colour audit also shows smaller but real FAST
advantages: held-out RGB median 35.43 versus lidarslam 36.37 (ours 2.7% worse),
RGB-L2<=20 inlier fraction 0.3655 versus 0.3536 (ours 3.3% worse), and common
planar coverage 0.6446 versus the compared lidarslam candidate's 0.5141 (ours
20.2% worse). Lidarslam's plane thickness was better in that candidate. These
gaps motivate selective visual updates and denser, pose-refined map fusion;
colour-only post-processing cannot close the trajectory deficit.

Processing throughput was separated from one-times player pacing with a
validated accelerated replay. A 2.0x probe dropped poses and a 1.25x probe
retained pose count but changed APE to 1.568 m; neither is admissible evidence.
At 1.05x, three runs retained exactly 1,255 poses, completed cleanly, and had
only 0.286% median APE drift from the 1.0x baseline. Including the entire fixed
five-second post-replay drain interval gives conservative processing RTF upper
bounds 0.99949 / 0.99901 / 0.99949. The validator checks repetition count,
pose count, accuracy drift, completion, and every RTF bound; its PASS artifact
is `processing_rtf_validation.json` under
`competitive_v1_exp04_rate1p05_3x_20260714`.

The old ad-hoc invocation ended with signal 11 while shutting the mapping
binary down directly. The controlled runner records health after bag
completion and then shuts down through `roslaunch`; all three controlled
runs exited cleanly. The gate uses processing wall time divided by sensor
duration. Pacing overhead from a one-times `rosbag play` is retained as a
separate diagnostic and must not be mislabeled as processing RTF.

Runner and scorer:

```bash
python3 scripts/run_fast_livo2_benchmark.py \
  --asset-root /media/sasaki/aiueo/benchmarks/fast_livo2 \
  --bag /media/sasaki/aiueo/benchmarks/fast_livo2/hilti_exp04_ros1.bag \
  --output <new-output-directory>
python3 scripts/summarize_fast_livo2_benchmark.py \
  --benchmark-dir <new-output-directory> \
  --reference-tum <ground-truth.tum>
```

## GLIM CPU exp04 baseline

The pinned source-built CPU image is
`glim-cpu-benchmark:competitive-v1` (`sha256:3e495e0b85d5...`). Its base image
and the source revisions of GTSAM 4.3a1, gtsam_points v1.2.2, GLIM v1.2.2,
and glim_ros2 are fixed in `docker/glim_cpu_benchmark.Dockerfile`. CUDA,
`march=native`, and the viewer are disabled.

GLIM starts its published trajectory after its three-second IMU initialization.
The first of exp04's seven surveyed checkpoints is earlier than that and was
excluded rather than clamped or extrapolated. The resulting six-checkpoint
file is generated from the intersection of all trajectory ranges and has
SHA-256 `537b7e0f13f223a07f8329cc2b7080e33dc2765a13293d67a7eb3312090db1b8`.
It is applied unchanged to both systems.

Three-run evidence:

- artifacts: `/media/sasaki/aiueo/benchmarks/glim/results/competitive_v1_exp04_3x_20260714`
- bag SHA-256: `d1117a4c6e4c3626a3039e48719ec6e39af34b0a95f5a9807163bc717229c8ee`
- all runs: 1,228 poses, final gap 0.1173 s, process exit 0
- APE RMSE median: 0.086624 m (range 0.084876--0.086702 m)
- processing RTF median: 0.2437
- peak RSS maximum: 690.88 MB

## Same-input lidarslam-ros2 exp04 baseline

The baseline uses the same ROS2 bag bytes, HILTI LiDAR--IMU calibration, and
the same six surveyed checkpoints. A completion-runner defect was found during
bring-up: its generic 120-second end margin could classify a temporary output
pause as completion on this short bag. The harness now accepts a configurable
margin; this profile uses 0.25 seconds and every formal run ended within
0.0173 seconds of the bag end.

- artifacts: `/media/sasaki/aiueo/benchmarks/competitive_ours/exp04_3x_20260714`
- all runs: 1,258 poses, process exit 0, identical APE
- APE RMSE median: 0.056536 m, 34.7% lower than GLIM
- processing RTF: 1.094 / 0.993 / 0.977; median 0.993
- peak RSS maximum: 586.83 MB, 85.0% of GLIM's maximum

The three-repeat contract aggregates trajectory accuracy and processing RTF
by median, peak RSS by maximum, and any completion/failure condition by the
worst run. On this regression sequence the trajectory, RTF, RSS, and completion
parts of the GLIM gate pass. Map-geometry non-regression remains to be scored,
and exp04 is not one of the three new holdouts required for a victory claim.

The common map-quality evaluator was then applied at 0.1 m downsampling. GLIM's
compact submaps were exported to world-frame PCD with
`scripts/export_glim_dump_map.py`; the exporter applies each submap's recorded
`T_world_origin` rather than concatenating local coordinates.

| exp04 map metric | lidarslam-ros2 | GLIM CPU | result |
|---|---:|---:|---|
| mean plane thickness (m, lower) | 0.06081 | 0.07911 | ours better |
| p95 plane thickness (m, lower) | 0.12504 | 0.12169 | ours 2.8% worse |
| planar coverage (higher) | 0.17949 | 0.43030 | ours worse |

All three repeated evaluator reports were byte-identical for each map. The
lidarslam row now comes from a fresh same-bag, same-six-checkpoint map-save run
at `/media/sasaki/aiueo/benchmarks/competitive_ours/exp04_map_fresh_20260714`;
its Autoware bundle verification passed 8/8 checks. The coverage deficit is
therefore a reproduced design gap rather than a stale-artifact uncertainty.
Trajectory accuracy is already ahead, but the GLIM map non-regression gate is
not yet met.

FAST-LIVO2's Open3D map stores XYZ as float64 in binary PLY. Feeding that file
directly to the PCL-based evaluator produced a one-point misread, so that
result is invalid. `scripts/convert_ply_xyz_to_pcd.py` now validates all XYZ
values and emits an explicit float32 XYZ binary PCD. The corrected conversion
preserved all 442,009 points (PCD SHA-256
`ef5bed7170c0e76a12f52c848a32b44c9fd73829defb4b162494ca9be7275d12`).
At 0.1 m, three byte-identical common-evaluator runs reported mean plane
thickness 0.03094 m, p95 0.08988 m, and planar coverage 0.64464. This confirms
that map geometry and coverage, not only trajectory APE, are material gaps on
the FAST-LIVO2 track. As with the earlier exp04 comparison, this is regression
evidence and not a holdout victory.

The official runner now also has a separate `--save-map` evidence mode. It
leaves the pinned FAST source clean, applies a runner-owned launch overlay, and
mounts FAST's `Log` directory into the run artifact. It is deliberately not
used for runtime/RSS gating because retaining every mapping point increases
memory. The exp04 probe retained 15,340,609 raw points and emitted a 442,007
point world-frame coloured PCD (SHA-256
`561e2065c5a1ecd54a2b74b2e520dc6925afbf77e9115305fd8383b4be98aedd`),
while preserving APE at 0.053029 m. Three byte-identical common evaluations
reported mean thickness 0.03097 m, p95 0.09031 m, and coverage 0.64490,
confirming the earlier converted-PLY values from a newly reproducible official
map export.

## Frozen exp02 input

HILTI exp02 construction multilevel is the third frozen untouched holdout. The
raw ROS1 bag is exactly 35,358,555,915 bytes with SHA-256
`e2c05e0dac389686363755370ead3fe727080195ad0b7976a7515fde9a054750`;
the canonical ROS2 tree SHA-256 is
`0cf22390317bf2841103d30623a98eb4da17cba757eb2e5a1c367cc24906bf56`.
The converted bag contains 262,178 messages over 430.286 seconds. The frozen
manifest is
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/manifests/exp02_inputs.json`
(SHA-256 `790aa4f4fb374983940301b24858885a803c598a8fa805f852dc9008efb9b683`).
The ROS1/ROS2 semantic audit exactly matched all 4,302 LiDAR, 171,801 IMU,
and 17,211 cam0 messages; its artifact SHA-256 is
`5340fcd1b305adf16f3b9cedef3293e95425b846d4b718e896aa878d6114a0c4`.
All three holdout slots now have immutable raw, canonical, manifest, semantic,
ground-truth, and calibration hashes.

## Frozen exp02 rival baselines

GLIM CPU completed all three runs cleanly with 4,271 poses and a 0.189 s end
gap in every repetition. On the 21 common checkpoints (one pre-initialization
checkpoint excluded), APE RMSE was 0.27142 / 0.16092 / 0.16633 m, giving a
0.16633 m median. Processing RTF was 0.30094 / 0.29037 / 0.29601 and maximum
peak RSS was 1,924.88 MB. Its three map exports contained 5.73M / 5.58M /
5.59M points. Conservative map quality was 0.08654 m mean thickness,
0.12389 m p95, and 0.18307 coverage.

FAST-LIVO2 also completed all three normal runs and shutdowns cleanly, with
3,387 / 3,353 / 3,440 poses and sub-0.009 s end gaps. Despite completing, it
drifted substantially on this multilevel sequence: APE RMSE was 16.47314 /
16.74360 / 16.39814 m (median 16.47314 m). Maximum normal-run peak RSS was
17,468.93 MB. The 1.0-times replay wall RTF median was 1.00361 and remains
diagnostic rather than processing-time evidence.

Three independent official FAST map exports all passed the common evaluator.
Conservative map quality was 0.03820 m mean thickness, 0.09359 m p95, and
0.49997 coverage. For visual evaluation, 860 undistorted cam0 images were
fixed once, then the identical image timestamps were re-posed with each
independent trajectory. Each run scored 86 held-out views and more than 7.3M
visible map points. RGB L2 medians were 117.18220 / 117.33772 / 114.33266 and
RGB-L2-at-most-20 inlier rates were 0.11412 / 0.11195 / 0.11532. Conservative
visual values are therefore 117.33772 and 0.11195. The hash-complete visual
summary SHA-256 is
`e2e928d3f589a4a9854935b0c5e802f751c9f9ae76f30a0f0a885ca4bbaf206d`.

## Frozen exp03 input

HILTI exp03 construction stairs is frozen before any algorithm tuning. The raw
ROS1 bag is exactly 21,915,037,983 bytes with SHA-256
`205d392d8ceab44af061905c33686ed07a8d225c3964da9b7b32c024ff866163`;
the canonical ROS2 tree SHA-256 is
`21e09d96b5374de856e41cb5a1e67bfeca9128c8227a95093486acfe230eeceb`.
The converted bag contains 188,563 messages over 309.518 seconds. The frozen
manifest is
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/manifests/exp03_inputs.json`
(SHA-256 `b18db47c61ce3085b9dc1ea224ec83c0a22983d5d1a37a0660b45b1f6cdefc32`).
The ROS1/ROS2 semantic audit exactly matched all 3,095 LiDAR, 123,557 IMU,
and 12,379 cam0 messages; its artifact SHA-256 is
`65c4a2f7692929613e955f7047eebfd674c8795611db3c80c8fec46f182a0626`.

## Frozen exp03 rival baselines

GLIM CPU artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp03/glim_cpu_3x`.
All three runs completed cleanly. The common reference contains 16 surveyed
checkpoints; the first checkpoint precedes GLIM initialization and was excluded
for every run. APE RMSE was 2.56673 / 2.20587 / 1.27193 m (median 2.20587 m),
processing RTF was 0.32586 / 0.28418 / 0.30518, and maximum peak RSS was
1,501.64 MB. The large run-to-run spread is retained as official evidence.

All three GLIM dumps produced world-frame maps, containing approximately
4.19M / 3.47M / 1.81M input points. Their mean plane thicknesses were
0.07841 / 0.08400 / 0.07524 m, p95 values were 0.12831 / 0.12965 / 0.11946 m,
and planar coverages were 0.04540 / 0.08057 / 0.13094. Run 1 fell below the
evaluator's 0.05 meaningful-coverage threshold, so the strict repeated-map
aggregator rejected the set. This is a rival map failure result, not a missing
map or evaluator error. The machine-readable `map_quality_summary.json` keeps
all three report/map hashes and rows, records two meaningful repetitions, sets
`aggregation_valid: false`, and deliberately has no aggregate value.

FAST-LIVO2 runtime artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp03/fast_livo2_1x_3x`.
All three runs produced exactly 3,092 poses and shut down cleanly. On all 17
checkpoints APE RMSE was 0.76402 / 0.79627 / 0.65214 m (median 0.76402 m),
and maximum peak RSS was 4,775.18 MB. One-times player wall RTF was 1.00382
median and remains diagnostic only.

A 1.025-times processing probe retained exactly 3,092 poses in every run,
completed cleanly, and gave a conservative maximum RTF upper bound of 0.99584.
It was nevertheless rejected because median APE changed from 0.76402 m to
0.64033 m, an absolute drift of 16.19%. Improvement direction does not make a
timing-altered run equivalent; the failed validation artifact is retained at
`fast_livo2_rate1p025_3x/processing_rtf_validation.json`.

FAST map-only artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp03/fast_livo2_map_3x`.
Three independent official exports passed the common 0.1 m evaluator. The
conservative worst values are 0.03440 m mean plane thickness, 0.10031 m p95,
and 0.57593 planar coverage.

The FAST-LIVO2 visual baseline uses each independent run's official packed-RGB
PCD and matching trajectory. Cam0 images are extracted from the semantically
identical ROS2 bag with the official Kalibr intrinsics/extrinsic, automatic
camera-to-LiDAR clock verification, undistortion, and a fixed image stride of
20. Evaluation uses two folds, holds out fold 1, and scores every fifth held-out
view without recolouring the official map. All three runs scored 62 held-out
views and more than 3.43 million visible points. RGB L2 medians were 65.24049 /
65.81876 / 64.97393 and RGB-L2-at-most-20 inlier rates were 0.23473 / 0.23414 /
0.23489. The conservative aggregate is therefore 65.81876 median error and
0.23414 inlier rate. Reports and their SHA-256 hashes are retained under
`fast_livo2_visual_3x/visual_quality_summary_with_inputs.json` (SHA-256
`1a32c0c9d67d5391db8ea80729cb11a4cd613789b8abd6aee62ac9cfea18cbd2`);
that artifact also hashes each map, trajectory, transforms file, and both
calibration files. The final-profile gate artifact is
`fast_livo2_final_result.json`.

## Frozen exp21 rival baselines

The first untouched holdout is HILTI exp21 outside building. Its raw ROS1 bag
SHA-256 is
`dd96adebdce2868d37875dafaf7fbca37bf1fa1800de0bc49ec622c6a771bf32` and
its canonical ROS2 tree SHA-256 is
`bcca9769ebdd87929dbf10af77ae58570ef4e2d67f59458e2f12779748e8cba6`.
The semantic bridge audit exactly matched all 1,528 LiDAR, 61,012 IMU, and
6,112 cam0 messages. The frozen input manifest is
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/manifests/exp21_inputs.json`.

GLIM CPU artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp21/glim_cpu_3x`.
All three runs completed on the same five surveyed checkpoints. APE RMSE was
0.167418 / 0.261510 / 0.365701 m (median 0.261510 m), processing RTF was
0.29487 / 0.29611 / 0.32489, and maximum peak RSS was 735.55 MB. Independent
map exports gave conservative worst values of 0.07614 m mean plane thickness,
0.12308 m p95, and 0.66079 planar coverage.

FAST-LIVO2 runtime artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp21/fast_livo2_1x_3x`.
All three runs completed and shut down cleanly. APE RMSE was 8.45399 / 8.36165 /
8.81196 m (median 8.45399 m), with five of five reference checkpoints used;
maximum peak RSS was 7,727.26 MB. The large error is repeatable and is not a
missing-checkpoint or completion artifact.

One-times player pacing gave a diagnostic median wall RTF of 1.00683. A formal
1.05-times validation achieved a conservative processing RTF upper bound of
0.99210 but was rejected: trajectory counts fell from 1,300 / 1,276 / 1,236
to 1,220 / 1,176 / 1,191 and median APE drifted by 18.51%. A 1.04-times probe
retained 1,238 poses and scored 8.83482 m, but its conservative RTF upper bound
was 1.00146 and it also failed the accuracy tolerance. Exp21 therefore has no
admissible accelerated-replay proof of FAST-LIVO2 processing RTF at most 1.0;
the failed validation artifact is retained rather than substituting replay
pacing as processing time.

FAST map-only artifacts are under
`/media/sasaki/aiueo/benchmarks/competitive_holdouts/exp21/fast_livo2_map_3x`.
Three independent official exports passed the common 0.1 m evaluator. The
conservative worst values are 0.04418 m mean plane thickness, 0.09761 m p95,
and 0.85672 planar coverage. Map-export resource use remains excluded from the
runtime gate, as required by the frozen protocol.

The fixed-image visual protocol retained 305 cam0 frames and scored 31
held-out views per repetition. RGB L2 medians were 93.49753 / 94.10812 /
92.27405 and RGB-L2-at-most-20 inlier rates were 0.13859 / 0.13862 / 0.14082.
Conservative values are 94.10812 and 0.13859. The visual summary, including
all report/map/trajectory/transforms/calibration hashes, has SHA-256
`e4087ba50a5e8407e2978708a15f214c199dbba00226d9f55c79cbb3b5756ec6`.

## Frozen candidate and initial holdout results

The algorithm candidate was frozen with complete dirty-tree provenance,
including tracked, untracked, and submodule files. The algorithm configuration
hashes are `befb0a723d67c77f950ac249c3a2ecff242400154b238a1a545165ab0345de95`
for the RKO-LIO YAML and
`5d01b9ef1943a12e57ff666abf8fda10caf92ca76e61a9716fcb4c6356a14e69`
for the graph-SLAM YAML. They remained byte-identical through the runner-only
timeout revisions. The current runner-v3 manifest has source-tree SHA-256
`3e46ca3795fb6f9a8292a7d7f9ae92a3eed0b083d92ca0b724c9ec07009acd23`
and manifest SHA-256
`4bf85b8baed968e036ddfae884663369e632f02ae36957f15b8a314cadd7ce30`.

On `exp03`, all three candidate runs completed. Raw APE was exactly
0.808793 m in every repetition. Dense graph-corrected APE was 0.770089 /
1.005015 / 0.869156 m, giving a 0.869156 m median. Median processing RTF was
0.692662 and maximum peak RSS was 532.09 MB. Conservative map quality was
0.043136 m mean thickness, 0.088255 m p95, and 0.354744 coverage. Each
repetition accepted one different loop edge, and graph correction was not
repeatably beneficial. The corrected trajectory beats GLIM accuracy by 60.6%
but loses to FAST-LIVO2 by 13.8%; its map also fails non-regression against
FAST-LIVO2.

The candidate's fixed-image visual evaluation re-posed the identical 618
cam0 frames with each dense graph-corrected trajectory, recoloured only from
the 309 training-fold views, and scored the same 62 held-out views. RGB L2
medians were 16.25014 / 15.97470 / 16.04314 and inlier-at-20 rates were
0.56831 / 0.57290 / 0.57159. Conservative values are therefore 16.25014 and
0.56831, both substantially better than FAST-LIVO2's 65.81876 and 0.23414.
The visual gate passes, but the complete FAST track fails trajectory accuracy,
map non-regression, and the harmful-loop rule.

On `exp21`, all three candidate runs completed. Raw APE was exactly
0.232750 m. Dense graph-corrected APE was 0.620892 / 1.050863 / 0.532720 m,
with a 0.620892 m median. Median processing RTF was 2.02062 and maximum peak
RSS was 675.06 MB. Conservative map quality was 0.054454 m mean thickness,
0.105852 m p95, and 0.461555 coverage. Each repetition again accepted one
loop edge and every graph correction degraded the raw trajectory. Although the
raw frontend beats GLIM trajectory accuracy by about 11%, the canonical
corrected result, real-time gate, and map non-regression gate fail.

The corresponding `exp21` visual runs used the same 305 fixed cam0 frames and
31 scored holdout views. RGB L2 medians were 33.25418 / 34.22253 / 32.98538;
inlier-at-20 rates were 0.38504 / 0.37887 / 0.38771. Conservative values are
34.22253 and 0.37887, again passing both visual comparisons against
FAST-LIVO2's 94.10812 and 0.13859. The complete FAST track still fails map,
RTF, and harmful-loop checks.

For a conservative zero-false-loop audit, a sole accepted edge is marked
verified harmful when the dense graph-corrected APE regresses by more than 2%
against the same run's raw trajectory on frozen surveyed checkpoints. Under
that policy, `exp03` has two harmful edges in three runs and `exp21` has three
in three. The hash-complete reports are
`docs/research/artifacts/competitive-loop-exp03.json` and
`docs/research/artifacts/competitive-loop-exp21.json`.

The first two `exp02` attempts are explicitly rejected harness artifacts. A
60-second total save budget was initially too short. After increasing the
total budget, run 3 exposed a separate fixed 15-second per-call timeout while
the 7.76-million-point planar filter required about 18 seconds; two repetitions
completed and the third repeatedly saved the map after its client timed out.
The per-call timeout now uses the remaining total save budget. A subsequent
restart was rejected before completion after detecting three unrelated GNSS
jobs at approximately one CPU core each plus another EKF/bag replay on the
machine. No timing result from that contaminated run is admissible.

## Primary sources

- https://github.com/koide3/glim
- https://github.com/koide3/glim_ros2
- https://github.com/hku-mars/FAST-LIVO2
- https://arxiv.org/abs/2408.14035
