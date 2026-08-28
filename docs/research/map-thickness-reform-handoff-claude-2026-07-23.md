# Map-thickness reform handoff for Claude (2026-07-23)

## Mission and acceptance gates

Continue the active long-horizon goal for `lidarslam_ros2`:

1. retain deterministic attribution of thickness to within-scan,
   between-scan, between-submap, and revisit effects;
2. develop dense pose-refined rebuild, deskew/time synchronization,
   fixed-lag multi-scan frontend, probabilistic surfel fusion, dynamic removal,
   and global surface BA based on measured evidence;
3. on multiple untuned datasets demonstrate at least 30% mean-thickness and
   25% p95-thickness improvement, no planar-coverage regression, no more than
   2% APE/RPE regression, at least 10% NDT localization-residual improvement,
   deterministic offline artifacts, RSS below 8 GB, and RTF below 5;
4. finish with a documented default-on/default-off decision.

The goal is still active. Do not mark it complete yet: the leading algorithms
have passed the aggregate numerical gates, the standalone NDT target is
productionized, and optimized selective-map repeat determinism passes. The
current release decision is explicitly default-off. The exp02 multilevel
geometry/resources and localization gates now pass with a deterministic 0.03
SE(3) pose-update gain. The same pre-frozen profile also passes the untouched
exp03 adoption run and its repeat determinism checks. exp01 still misses the
individual mean-NDT gate, so the release remains default-off pending an
explicit every-sequence versus macro acceptance decision.

The second untouched post-freeze sequence, exp21, passes mean thickness,
coverage, NDT, APE/RPE, and resources but misses p95 thickness (-19.9869%
versus the required -25%). Do not promote default-on under an every-sequence
policy and do not tune on exp21.

## Repository and safety boundary

- Repository: `/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2`
- Parent ROS workspace: `/home/sasaki/workspace/old_~2026/lidarslam_ws`
- ROS: Jazzy
- Source before builds/runs:

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ../install/setup.bash
  ```

- The worktree contains many unrelated user edits and untracked files. Do not
  reset, clean, or overwrite them. The relevant files are listed below.
- Builds in this session were usually invoked from the repository root, so the
  latest binaries are under `build/graph_based_slam/`. Older parent-workspace
  binaries are under `../build/graph_based_slam/`; do not accidentally test
  those.
- `/usr/bin/time -f 'elapsed=%e rss_kib=%M'` was used for resource evidence.
- `rtk` is unavailable.

## Leading geometry candidate

The leading visualization/geometry map is:

- 0.10 m output voxels;
- hierarchical probabilistic surfel support at 0.5 m, fallback-only 0.3 m,
  then fallback-only 0.4 m;
- all eight half-voxel XYZ support-grid phases;
- phase zero owns supported outputs; shifted phases only fill unsupported
  outputs and never overwrite phase zero;
- 5 cm tail-selective consolidation: start from the occupancy-preserving
  fused map and release the fine-voxel clamp only when the raw plane projection
  moves the baseline centroid by at least 0.05 m;
- all features remain default-off.

Relevant runner parameters:

```text
save_probabilistic_surfel_map:=true
save_surface_consolidated_map:=true
probabilistic_surfel_scan_voxel_size:=0.1
probabilistic_surfel_output_voxel_size:=0.1
probabilistic_surfel_support_voxel_size:=0.5
probabilistic_surfel_secondary_support_voxel_size:=0.3
probabilistic_surfel_tertiary_support_voxel_size:=0.4
probabilistic_surfel_support_grid_phases:=8
probabilistic_surfel_support_phases_fallback_only:=true
surface_consolidation_min_projection_distance_m:=0.05
```

### Frozen three-sequence geometry result

All comparisons use the same-run centroid map as baseline.

| dataset | mean thickness | p95 thickness | planar coverage |
|---|---:|---:|---:|
| exp04 development | -40.6581% | -20.9237% | +216.1229% |
| exp07 unseen | -34.2843% | -32.7517% | +358.7856% |
| exp01 unseen | -38.8962% | -33.1259% | +264.7444% |
| macro | **-37.9462%** | **-28.9338%** | **+279.8843%** |

Thus the aggregate mean, p95, and coverage gates pass. This is a post-pose map
representation, so it does not modify trajectory poses. Baseline and fused
exp01 PCDs remained byte-identical across the memory optimization.

Key artifacts:

- exp04 5 cm:
  `/tmp/lidarslam-surfel-exp04-fallback-phases8-selective5cm-20260723-v1`
- exp07 5 cm:
  `/tmp/lidarslam-surfel-exp07-fallback-phases8-selective5cm-20260723-v1`
- exp01 initial 5 cm:
  `/tmp/lidarslam-surfel-exp01-fallback-phases8-selective5cm-20260723-v1`
- exp01 memory-optimized 5 cm:
  `/tmp/lidarslam-surfel-exp01-fallback-phases8-selective5cm-memory-v2-20260723`

The optimized exp01 candidate quality was:

```text
mean thickness  0.034320949 m
p95 thickness   0.077490640 m
coverage        0.144929161
```

The early-float projection buffer changed only eight 5 cm threshold decisions
relative to the pre-optimization build; the geometry metrics changed by less
than 0.001%.

### Resource result

The first selective exp01 build peaked at 8,167,992 KiB. The implementation was
changed to hold support-stage raw projections as `Vector3f`, defer the final
double output map, and release 41,076,537 keyed observations before selective
re-voxelization. The repeat then produced:

```text
elapsed  823.65 s
RSS      7,719,268 KiB
RTF      approximately 1.19 for the roughly 693 s sequence
```

This passes both the strict 8,000,000 KiB interpretation and RTF 5.

### Final optimized exp04 repeat

Two fresh full-input runs of the memory-optimized implementation used the
frozen profile and wrote separate output directories:

- `/tmp/lidarslam-surfel-exp04-fallback-phases8-selective5cm-memory-v2-repeat-20260723`
- `/tmp/lidarslam-surfel-exp04-fallback-phases8-selective5cm-memory-v2-repeat2-20260723`

All nine output files were byte-identical. Important hashes are:

```text
selective PCD  1660011b552a962c4a1fee945b7d88289c4659aa4fb213c6fd6c3c67b7278309
surfel report  f7b397eef6b955702df51683f82058d14528cb492f8a479d228dc9d35919715c
baseline PCD   18a796e32ede9b7185f53fabbd585adcba72bc3f87565d90e8bc9bc01fa69b5f
fused PCD      9f1ff18e55ff4d83ac3c6551813ec102f8b111de75b4d108b3c1353761a547e4
```

Run 1 used 249.11 s / 2,768,492 KiB; run 2 used 244.25 s /
2,768,496 KiB. The optimized selective artifact re-evaluated at mean
0.035434750 m, p95 0.094883292 m, and coverage 0.232424851. Against the
same-run centroid baseline (0.059712989 m, 0.119989591 m, 0.073522897), this
is -40.6582%, -20.9237%, and +216.1258%, respectively.

The pre-optimization selective PCD differs because early-float projection
changes one point at the exact 5 cm selection boundary. Its baseline/fused
PCDs, all three trajectories, and loop edge are nevertheless byte-identical
to both optimized runs. Same-code optimized determinism is exact.

## Leading NDT localization target

Using the thin/selective map directly worsens NDT residual because it removes
target density. Simple centroid+fused/selective unions, repeated point weights,
and small tangent samples were measured and rejected. The working representation
uses the occupancy-preserving eight-phase fused map as input, partitions it into
1.0 m NDT voxels, accepts locally planar voxels with eigenvalue gates
`lambda0/lambda2 <= 0.10` and `lambda1/lambda2 >= 0.05`, and adds symmetric
in-plane samples:

- radius 0.50 m, never larger than half the NDT voxel width;
- axis samples `+/-t1`, `+/-t2`;
- four diagonal samples at radius `0.50/sqrt(2)`;
- no normal-direction samples, so the localization target is not thickened;
- the original point is retained;
- minimum six points in a voxel.

This profile was tuned only on exp04, frozen, then applied unchanged to exp07
and exp01 even-map/odd-query splits.

### Frozen cross-fit NDT result

| dataset | fitness mean | fitness p95 | pose translation mean | pose translation p95 |
|---|---:|---:|---:|---:|
| exp04 development | -10.8325% | -16.4684% | +5.4232% | -1.0469% |
| exp07 unseen | -13.6183% | -15.5802% | -5.4709% | -3.3299% |
| exp01 unseen | -8.1221% | -3.3962% | -2.0751% | +1.1258% |
| macro | **-10.8577%** | **-11.8149%** | **-0.7076%** | **-1.0837%** |

The aggregate NDT 10% gate passes for both mean and p95, and aggregate pose
recovery also improves. exp01 individually does not reach -10%, so state the
acceptance policy explicitly before default-on promotion; current evidence is
an aggregate/macro pass, not an every-sequence pass.

Cross-fit artifacts:

- exp04 even map and reports:
  `/tmp/lidarslam-surfel-exp04-cv-even-selective5cm-memory-v2-20260723`
- exp07 even map and reports:
  `/tmp/lidarslam-surfel-exp07-cv-even-fallback-phases8-memory-v2-20260723`
- exp01 even map and reports:
  `/tmp/lidarslam-surfel-exp01-cv-even-fallback-phases8-memory-v2-20260723`

The exact exp04 passing report is
`ndt_query_odd_fused_tangent50cm_diagonal.yaml`. It used 4,008,867 original
fused points, added 1,725,192 tangent samples, evaluated 63 unseen odd scans,
ran in 9.04 s, and used 511,932 KiB RSS.

The exact exp07 passing report is `ndt_query_odd_tangent.yaml`; it evaluated 67
odd scans. The exp01 report with the same name evaluated 114 odd scans.

## Current implementation state

### Implemented and built/tested before the final handoff request

- deterministic four-level map-thickness attribution and CSV/report adapter;
- dense pose-refined rebuild;
- gyro deskew/time-sync research path;
- fixed-lag backend path;
- probabilistic surfel fusion with hierarchical fallback support;
- eight fallback-only support phases with phase-zero ownership;
- connected-field and dynamic-removal negative probes;
- scan-rate/cross-fit plane refiner negative probe;
- absolute-prior global surface BA and negative/safe profiles;
- supported/fallback partition maps;
- full and 5 cm selective consolidation;
- memory optimization described above;
- NDT evaluator support for secondary/tertiary maps, repeated/strided secondary
  maps, tangent sampling, per-scan CSV, covariance columns, and deterministic
  registered-pose TUM output;
- sparse-checkpoint translation RPE reporting;
- explicit exp02/exp03/exp21 capture support in
  `run_hilti_overlap_crossval.sh` (legacy `all` remains exp01+exp07).

After formatting, the standalone generator, evaluator, and seven focused test
targets rebuilt successfully. A fresh 2026-07-23 validation passed 7/7 CTest
targets (56 individual gtests): `test_probabilistic_surfel_map` 18,
`test_probabilistic_surfel_fusion` 5, `test_scan_surface_refiner` 6,
`test_map_thickness_attribution` 6, `test_dense_pose_correction` 4,
`test_plane_ba` 8, and `test_ndt_localization_target` 9. The five NDT-related
edited/new C++ files pass `ament_uncrustify` with no divergence. The two
trajectory-I/O NDT tests cover deterministic canonical TUM serialization and
non-finite pose rejection; `test_sparse_checkpoint_errors.py` also passes 2/2
with translation RPE coverage. `ament_flake8` on the sparse-error script and
test, plus `git diff --check`, pass as well.

The exp02 development probes add three focused cases for optional inner-ring,
angular-midpoint, and partial-midpoint-pair sampling; the NDT target test now
contains eight cases. All new sampling features are default-off and preserve
the original eight-direction target when omitted.

### Standalone tangent-target productionization completed

The shared implementation is in
`graph_based_slam/include/graph_based_slam/ndt_localization_target.hpp`.
`map_ndt_residual_report` calls it for on-load augmentation and the installed
`ndt_localization_target` executable writes a reusable binary PCD plus YAML
provenance. CMake integration and `test_ndt_localization_target` are present;
its six target-generation cases cover sample counts/symmetry, unchanged
normal thickness, non-planar rejection, invalid radius, and byte determinism.

Two exp04 generations were byte-identical:

```text
PCD SHA-256               bd3d967c4537e58d7d3f9c5402dcb22b820bdfb76861de37ffdbbcbccb7f2870
normalized report SHA-256 ff382ece87b4522f91b860311b16a109b78a1fa8f6f8aaf69efe91c64233961d
input / sampled / output  4,008,867 / 1,725,192 / 5,734,059 points
elapsed / max RSS         0.75 s / 507,312 KiB (worst of two)
```

Generated-PCD evaluation reproduced the on-load numbers exactly for exp04 and
exp01. exp07 mean fitness differed by only `6.1e-10 m^2` after float PCD
serialization and its p95 was exact. Generation statistics were:

| dataset/map | input | sampled | output | elapsed | max RSS |
|---|---:|---:|---:|---:|---:|
| exp04 even | 4,008,867 | 1,725,192 | 5,734,059 | 0.75 s | 507,312 KiB |
| exp07 even | 2,096,482 | 942,592 | 3,039,074 | 0.36 s | 268,684 KiB |
| exp01 even | 11,772,320 | 2,015,912 | 13,788,232 | 1.93 s | 1,246,424 KiB |
| exp01 full | 19,691,414 | 2,328,240 | 22,019,654 | 3.50 s | 1,990,468 KiB |

The helper still accumulates voxel moments in deterministic input order. The
upstream fused PCD is deterministic. Changing accumulation or sorting now
would invalidate the frozen cross-fit evidence and requires rerunning it.

### Direct registered-pose trajectory result

`map_ndt_residual_report` now accepts `--trajectory REGISTERED.tum`. It writes
one final registered pose for every sampled scan using the shared deterministic
serializer in `graph_based_slam/ndt_trajectory_io.hpp`. Every odd scan was
evaluated against the even-scan map (629 exp04, 661 exp07, 1,138 exp01), and all
poses converged. Sparse HILTI checkpoint results are:

| dataset | APE RMSE | APE mean | RPE RMSE | RPE mean | NDT mean | NDT p95 |
|---|---:|---:|---:|---:|---:|---:|
| exp04 development | -8.8570% | -3.0679% | -11.4621% | -5.6925% | -10.5170% | -7.9597% |
| exp07 unseen | -3.0419% | -1.8007% | -4.5434% | -3.7839% | -13.8012% | -11.2164% |
| exp01 unseen | -9.5024% | -8.2134% | -8.9657% | -8.4957% | -7.9055% | -6.7143% |
| macro | **-7.1338%** | **-4.3607%** | **-8.3237%** | **-5.9907%** | **-10.7413%** | **-8.6301%** |

The APE/RPE non-regression gate passes on every sequence. The full odd-scan
mean NDT gate passes in macro, while exp01 still misses per-sequence. Two exp04
tangent evaluations produced byte-identical TUM and CSV files; TUM SHA-256 is
`4db1d7131d2d20d1c8a93477262fda7109849b1a1c2dd4e64f25fad615cde426`.
Reports and analysis artifacts are under `/tmp/ndt-exp{04,07,01}-trajectory-*`
and `/tmp/ndt-exp{04,07,01}-trajectory-ape-rpe.json`.

### exp02 multilevel audit (latest state)

Frozen input and all evidence are under
`/media/sasaki/aiueo/benchmarks/map_thickness_exp02_holdout_20260723/exp02`.
The input has 4,302 exact pose/cloud pairs over 752.13 s. The even-scan frozen
map used 2,151 scans, 174 submaps, and 270.2 m travel. It completed in
1,456.25 s (RTF 1.94) with 6,620,184 KiB RSS.

| exp02 geometry | baseline | selective | change |
|---|---:|---:|---:|
| mean | 0.058157903 m | 0.034778160 m | -40.2005% |
| p95 | 0.118127485 m | 0.084016766 m | -28.8762% |
| coverage | 0.057246789 | 0.210034211 | +266.8926% |

The original single-ring target improved full odd-scan NDT mean by only
8.9369%, although APE RMSE was within gate at +0.6650% and RPE RMSE improved
8.2654%. After observing that failure, exp02 became a development sequence.
Optional planar-only density probes produced:

| probe | NDT mean | APE RMSE | RPE RMSE |
|---|---:|---:|---:|
| original 8 directions | -8.9369% | +0.6650% | -8.2654% |
| +0.25 m inner ring | -9.6461% | +1.9968% | -5.7120% |
| 16 outer directions | -10.0942% | +2.5813% | -5.2055% |
| inner ring + two midpoint pairs | -10.1077% | +3.6991% | -4.6847% |

No density-only probe passes both the -10% NDT and <=2% APE gates. Inner
0.10/0.40 m and five-iteration probes were also negative.

The evaluator now supports deterministic SE(3) update regularization through
`--pose-update-gain`. Translation is linearly interpolated and rotation uses
quaternion slerp; guarded-pose fitness is recomputed from the transformed query
cloud. Gain 1.0 is the unchanged default. Gain 0.99 was rejected because the
fair same-gain baseline comparison still regressed APE by 2.6046%. Gain 0.03
passes the full exp02 comparison:

| exp02 guard 0.03 | baseline | 16 directions | change |
|---|---:|---:|---:|
| NDT mean | 0.062138551 | 0.055563116 | -10.5819% |
| NDT p95 | 0.151733801 | 0.136065770 | -10.3260% |
| APE RMSE | 0.089760653 m | 0.091244191 m | +1.6528% |
| RPE RMSE | 0.125954137 m | 0.127669685 m | +1.3620% |

The candidate ran in 425.59 s with 2,733,332 KiB RSS. Its repeat TUM and CSV
are byte-identical; their SHA-256 values are respectively
`61c3863d1f9600d8eef3a5da5354ada4db7e526b09d03be0e285bd10e791fa9b`
and `0bd525dd1d0491cc7ce08a83cd0f3fb75f61436c4511b2aead924c45023db543`.

The same profile was rerun fairly on the earlier sequences:

| sequence | NDT mean | NDT p95 | APE RMSE | RPE RMSE |
|---|---:|---:|---:|---:|
| exp04 | -12.5419% | -12.2776% | +0.4849% | +0.2756% |
| exp07 | -16.0055% | -13.2546% | -0.0846% | -0.1231% |
| exp01 | -8.4185% | -10.1285% | +0.1185% | +0.1314% |
| exp02 | -10.5819% | -10.3260% | +1.6528% | +1.3620% |
| macro | -11.8870% | -11.4967% | +0.5429% | +0.4115% |

The frozen manifest is
`docs/research/artifacts/map-thickness-ndt-guard03-candidate-2026-07-23.yaml`.
That was the pre-holdout state; the completed exp03 result is recorded below.
exp03 must not be converted into a development/tuning sequence. The release
remains default-off because exp01 still misses the individual mean-NDT
threshold and only one strictly untouched post-freeze dataset has been run.

### exp03 untouched adoption result

The pre-holdout candidate manifest SHA-256 is
`d1d76b9dbd7c3c2d3fb5d276b44d371fe62bf4f73ce0daf46a4c5afcfb645b5f`.
No parameter was changed after observing exp03. The frozen input has 3,095
exact pose/cloud pairs over 160.537 s. The even map used 1,548 scans, 133
submaps, 207.1 m travel, and two loop edges. It ran in 212.09 s (RTF 1.32) at
2,904,580 KiB RSS.

| exp03 gate | baseline | candidate | change |
|---|---:|---:|---:|
| mean thickness | 0.052662440 m | 0.035356915 m | -32.8612% |
| p95 thickness | 0.106008994 m | 0.077071068 m | -27.2976% |
| planar coverage | 0.094449920 | 0.311300294 | +229.5930% |
| NDT mean | 0.100425079 | 0.086414311 | -13.9515% |
| NDT p95 | 0.317252726 | 0.266671698 | -15.9434% |
| APE RMSE | 0.937860163 m | 0.938092093 m | +0.0247% |
| RPE RMSE | 0.749940879 m | 0.750155336 m | +0.0286% |

All 1,547 odd scans converged. The candidate NDT run took 76.37 s and
1,215,512 KiB RSS. A complete second map build made nine primary artifacts
byte-identical; regenerated target PCD and repeat candidate TUM/CSV were also
byte-identical. The result artifact is
`docs/research/artifacts/map-thickness-exp03-holdout-result-2026-07-23.yaml`.
The five-sequence macro is NDT mean/p95 -12.2999/-12.3860%, APE/RPE
+0.4393/+0.3349%.

### exp21 second untouched holdout result

The same pre-holdout manifest was applied without changes. exp21 has 1,528
exact pairs over 340.445 s. Its even map used 764 scans, 80 submaps, 123.7 m
travel, and one loop edge; it ran in 168.13 s (RTF 0.49) at 3,401,168 KiB RSS.

| exp21 gate | baseline | candidate | change |
|---|---:|---:|---:|
| mean thickness | 0.058783005 m | 0.038111589 m | -35.1656% |
| p95 thickness | 0.118644469 m | 0.094931111 m | **-19.9869% (fail)** |
| planar coverage | 0.092257431 | 0.346325609 | +275.3905% |
| NDT mean | 0.060477898 | 0.051064368 | -15.5652% |
| NDT p95 | 0.150775282 | 0.129194013 | -14.3135% |
| APE RMSE | 0.237103548 m | 0.229894340 m | -3.0405% |
| RPE RMSE | 0.252519588 m | 0.246718852 m | -2.2971% |

The result is frozen in
`docs/research/artifacts/map-thickness-exp21-holdout-result-2026-07-23.yaml`.
The six-sequence macro passes all numerical gates: geometry mean/p95
-37.0110/-27.1604%, coverage +268.5882%, NDT mean/p95 -12.8441/-12.7073%,
APE/RPE -0.1407/-0.1038%. Nevertheless, exp21 blocks an every-sequence
default-on decision. It must not be used for parameter selection.

## Relevant modified/new source files

- `graph_based_slam/include/graph_based_slam/probabilistic_surfel_map.hpp`
- `graph_based_slam/include/graph_based_slam/scan_surface_refiner.hpp`
- `graph_based_slam/include/graph_based_slam/ndt_localization_target.hpp`
- `graph_based_slam/include/graph_based_slam/ndt_trajectory_io.hpp`
- `graph_based_slam/src/graph_slam_offline_runner.cpp`
- `graph_based_slam/src/map_ndt_residual_report_main.cpp`
- `graph_based_slam/src/ndt_localization_target_main.cpp`
- `graph_based_slam/test/test_ndt_localization_target.cpp`
- `scripts/analyze_sparse_checkpoint_errors.py`
- `graph_based_slam/test/test_sparse_checkpoint_errors.py`
- `scripts/run_hilti_overlap_crossval.sh`
- `graph_based_slam/test/test_probabilistic_surfel_map.cpp`
- `graph_based_slam/test/test_scan_surface_refiner.cpp`
- `graph_based_slam/CMakeLists.txt`
- `docs/research/map-thickness-attribution-foundation-2026-07.md`
- this handoff file.

Plane-BA and other earlier goal files are also modified; inspect `git status`
and never assume all dirty files belong to this subtask.

## Reproduction commands

### Geometry quality

```bash
bash scripts/run_map_quality_check.sh \
  --input /tmp/lidarslam-surfel-exp01-fallback-phases8-selective5cm-memory-v2-20260723/map_surfel_surface_consolidated.pcd \
  --output-dir /tmp/recheck-exp01-selective-quality \
  --runs 1 --downsample 0.1 \
  --setup /home/sasaki/workspace/old_~2026/lidarslam_ws/install/setup.bash
```

### Standalone NDT target and evaluator profile

```bash
build/graph_based_slam/ndt_localization_target \
  --input /tmp/lidarslam-surfel-exp04-cv-even-selective5cm-memory-v2-20260723/map_surfel_fused.pcd \
  --output /tmp/ndt-localization-exp04-generated.pcd \
  --report /tmp/ndt-localization-exp04-generated.yaml \
  --resolution 1.0 --radius 0.50 --diagonals true

build/graph_based_slam/map_ndt_residual_report \
  --map /tmp/ndt-localization-exp04-generated.pcd \
  --bag /media/sasaki/aiueo/benchmarks/hilti_exp04_backend_fixed_20260713/backend_input \
  --output /tmp/recheck-exp04-ndt.yaml \
  --stride 20 --offset 1 --resolution 1.0 \
  --source-voxel 0.5 --max-correspondence 2.0 --max-iterations 10
```

### Even-map split parameters

Add these to the geometry runner command:

```text
probabilistic_surfel_input_scan_stride:=2
probabilistic_surfel_input_scan_offset:=0
```

The NDT query uses `--stride 20 --offset 1`, so all sampled query indices are
odd and excluded from the even map.

## Negative results that should not be repeated

- 8-phase blending regressed or was neutral; fallback-only phase ownership is
  the useful form.
- connected 26-neighbor surfel smoothing changed too few cells; fallback
  propagation either regressed geometry or was neutral.
- all-supported consolidation passed mean but missed p95 and worsened NDT.
- 2 cm selective consolidation was worse than 5 cm on all exp04 geometry
  metrics.
- centroid+fused/selective union targets improved NDT only about 3--5% mean;
  triple union reached -4.71% mean and -9.04% p95 but worsened pose recovery.
- repeated secondary weights and half-density secondary targets did not reach
  the NDT gate.
- tangent radii 0.05, 0.10, 0.20, and 0.40 m were below the final gate; 0.50 m
  with axes only passed p95 but reached only -8.62% mean. The symmetric
  0.50 m axis+diagonal profile is the first pass.
- scan-rate surface refinement and safe global surface BA were externally
  neutral; weaker priors violated trajectory gates.
- visibility/dynamic filtering was negative.

## Documentation and final validation still required

1. Decide and document whether adoption requires every sequence or a frozen
   multi-sequence macro. Under every-sequence policy, exp01 misses mean NDT and
   exp21 misses p95 thickness. Under macro policy, all numerical gates pass.
2. For an every-sequence release, develop tail-p95 robustness on data other
   than exp03/exp21, then validate on a new untouched non-HILTI or future
   dataset. Do not tune either post-freeze holdout.
3. If touching files outside the frozen NDT implementation, run the broader
   package formatting/lint suite. The five NDT files already pass
   `ament_uncrustify`; the sparse-error Python files pass `ament_flake8`; and
   the seven focused test targets pass.
4. Keep the recorded default-off decision until the acceptance policy is
   resolved or a new frozen candidate passes a new untouched dataset.
5. Do not claim final completion until additional held-out evidence supports
   the final adoption decision.

## Suggested immediate next command

Do not retune exp02/exp03/exp21. Start by reviewing the three frozen artifacts
and deciding the acceptance policy explicitly:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
for name in (
    'map-thickness-ndt-guard03-candidate-2026-07-23.yaml',
    'map-thickness-exp03-holdout-result-2026-07-23.yaml',
    'map-thickness-exp21-holdout-result-2026-07-23.yaml'):
    yaml.safe_load((Path('docs/research/artifacts') / name).read_text())
    print('valid:', name)
PY
```

Never change a parameter in response to exp03 or exp21. They are evidence, not
development sequences.
