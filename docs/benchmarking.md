# Benchmarking And Release Gate

This page describes the recommended benchmark path and the release/readiness
gate used for the default permissive workflow.

## Recommended Benchmark

The standard benchmark path for this repository is:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

### Degenerate-LIO SOTA track

The preregistered public degeneracy track is defined in
`configs/slam_benchmark_profiles/degenerate_lio_sota_v1.yaml`. It begins with
ENWIDE TunnelS/TunnelD and forbids radar, wheel odometry, GNSS, cameras,
per-sequence tuning, and scale alignment. Download exact official inputs with:

```bash
bash scripts/download_enwide.sh \
  --sequence tunnel_d \
  --dest datasets/enwide \
  --convert
```

The profile remains report-only until all ENWIDE and GEODE degenerate
sequences, pinned rivals, and the hidden holdout are complete. See
`docs/research/enwide-sota-benchmark-plan-2026-07.md` for the claim policy.
Use `scripts/run_enwide_sota_benchmark.sh` for the fixed three-repetition
candidate run; sensor and scoring choices are deliberately not command-line
options.

### Radar-less tunnel frontend A/B

The radar-less tunnel research track has a frontend-only control/candidate
runner. It uses isolated DDS domains, stops `offline_node` with `SIGINT` after
odometry becomes quiet, and records the exact parameter layers, Git SHAs, TUM
trajectories, and comparison metrics:

```bash
bash scripts/run_radarless_tunnel_ab.sh \
  --sequence tunnel \
  --output-root /media/<ssd>/benchmarks/radarless_tunnel_adaptive_v1
```

Use `--candidate-param name:=value` for a focused override and `--dry-run` to
freeze the commands without starting ROS. Run `--sequence fog` as the first
negative check. HILTI exp07 and MID-360 are also supported with explicit
`--bag`, topic, base-parameter, and optional reference arguments. The runner
intentionally rejects exp02, exp03, and exp21 because they are reserved final
holdouts.

`comparison.json` includes endpoint/path metrics, time-aligned reach-ratio
quantiles after the first 10 m of reference motion, and an SE(3)-aligned
translation delta. It also records candidate overrides and the velocity-blend
diagnostic summary when present. For an existing trajectory, the same evaluator
can be run directly:

```bash
python3 scripts/evaluate_degeneracy_trajectory.py <candidate.tum> \
  --reference-trajectory <dense-reference.tum>
```

## FAST-LIVO2 head-to-head

Use the exact same bag, sensor messages, calibration, trajectory reference, and
evaluation alignment for both systems. Record each result in this compact JSON
shape (unknown metrics should be omitted, never estimated):

```json
{
  "system": "lidarslam_ros2",
  "dataset": "hilti2022_exp04",
  "trajectory": {"ape_rmse_m": 0.07146},
  "geometry": {
    "plane_thickness_mean_m": 0.0599,
    "planar_coverage": 0.5355
  },
  "colour": {
    "heldout_rgb_l2_median": 36.37,
    "heldout_rgb_inlier_20": 0.3536
  }
}
```

Add RPE and runtime keys only after measuring them. Create a second manifest
with `"system": "FAST-LIVO2"` and run:

```bash
python3 scripts/compare_fast_livo2.py \
  --ours output/head_to_head/lidarslam_ros2.json \
  --fast-livo2 output/head_to_head/fast_livo2.json \
  --out output/head_to_head/comparison.json
```

The command also writes `comparison.md`. It scores APE, RPE, real-time factor,
peak memory, plane thickness, planar coverage, held-out RGB error, and held-out
RGB inlier rate. A metric only counts when both systems provide it; values
within 1% are ties. This prevents an attractive map image or a single trajectory
number from being presented as an overall win.

## SLAM candidate regression

Run plane-revisit OFF/ON with the same backend input and reference:

```bash
bash scripts/run_plane_revisit_candidate_benchmark.sh \
  --dataset mid360_public --bag <backend-input-bag> \
  --reference-tum <reference.tum> --fixed-loop-edges <verified.csv> \
  --output-dir /media/<ssd>/benchmarks/phase7/mid360
```

Repeat with `hilti_exp04` and `rtkslam_construction_seq2`. Construction Seq2 is
the required second positive sequence; its total-station checkpoints contain
positions but no surveyed orientations, so rotational RPE is forbidden.
`--dry-run` prints the pipeline without starting ROS. For submap-rate backend
poses, supply the matching frontend trajectory with `--dense-raw-tum`.

To create a deterministic backend input from the original sensor bag:

```bash
ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=1 \
bash scripts/record_backend_input.sh --output-dir <work>/backend_input -- \
  bash scripts/run_rko_lio_graph_benchmark.sh \
    --bag <construction_seq2> --lidar-topic /livox/points \
    --imu-topic /livox/imu --skip-reference-gen \
    --reference-tum <construction_seq2_gt.tum> \
    --reference-meta <construction_seq2_reference.json> \
    --rko-param configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml \
    --lidarslam-param lidarslam/param/lidarslam.yaml \
    --output-dir <work>/source_run --offline-timeout-secs 5400
```

The recorder refuses overwrite, flushes MCAP on exit, and requires non-empty
`/rko_lio/odometry` and `/rko_lio/frame` topics. When `--dense-raw-tum` is
supplied to the candidate runner, its timestamp span is used as the runtime
denominator; backend bags use processing time, not original sensor time.
Compare the three dataset pairs:

```bash
python3 scripts/evaluate_slam_candidate_regression.py \
  --baseline <mid360-off>/cross_repo_benchmark.json \
  --baseline <hilti-off>/cross_repo_benchmark.json \
  --baseline <construction-seq2-off>/cross_repo_benchmark.json \
  --candidate <mid360-on>/cross_repo_benchmark.json \
  --candidate <hilti-on>/cross_repo_benchmark.json \
  --candidate <construction-seq2-on>/cross_repo_benchmark.json \
  --output output/phase7/candidate_regression.json --require-pass
```

Promotion requires complete reports, matching inputs, bounded runtime and map
quality, and independently improved MID-360 and Construction Seq2 trajectories.

When aggregate ATE hides which surveyed positions changed, generate a
checkpoint-level JSON and Markdown report:

```bash
python3 scripts/analyze_sparse_checkpoint_errors.py \
  --reference-tum <surveyed-positions.tum> \
  --reference-csv <checkpoint-labels.csv> \
  --estimate raw=<raw.tum> --estimate baseline=<off-dense.tum> \
  --estimate candidate=<on-dense.tum> --baseline-label baseline \
  --output <suite>/checkpoint_errors.json
```

Each trajectory is independently SE(3)-aligned without scale before per-point
errors are compared, matching the position-only public-suite ATE semantics.

That wrapper:

- uses the bundled NTU VIRAL `rosbag2`
- runs `RKO-LIO + graph_based_slam`
- saves raw and corrected trajectories
- computes APE against the Leica prism reference
- verifies the Autoware map bundle when present
- writes `metrics.json` for the reporting pipeline

### Cross-repository suite (Localization Zoo)

`public_suite_v1.yaml` connects Localization Zoo trajectories to trajectory,
geometry, real-RGB, runtime, and memory gates:

```bash
python3 scripts/run_cross_repo_slam_benchmark.py \
  --localization-zoo ../loc_zoo_ws/localization_zoo \
  --dataset <profile> --gt-tum <gt.tum> --raw-tum <raw.tum> \
  --corrected-tum <graph.tum> --runtime-report <runtime.json> \
  --out-dir <benchmark-dir>
```

Candidate promotion compares frozen OFF/ON manifests from MID-360, the HILTI
position-only holdout, and RTK-SLAM Construction Seq2 surveyed checkpoints.
The gate never invents rotational RPE for position-only references. The
initial result is recorded in the
[Phase 7 regression note](research/phase7-plane-revisit-regression-2026-07.md);
the second-positive rejection is in the
[Phase 8 RTK-SLAM note](research/phase8-rtkslam-plane-revisit-2026-07.md).

## KITTI / LiDAR-Only Evaluation

The public default benchmark remains `RKO-LIO + graph_based_slam`. For KITTI
Odometry, use the separate LiDAR-only path because the Velodyne dataset does
not provide IMU messages.

```bash
bash scripts/download_kitti_odometry.sh --velodyne
export KITTI_ODOMETRY_ROOT="$PWD/datasets/KITTI_odometry"
bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare
```

For frontend tuning, run the sweep wrapper:

```bash
bash scripts/sweep_kitti_small_gicp.sh \
  --dataset "$KITTI_ODOMETRY_ROOT" \
  --sequences "00 05 07"
```

The LO and `small_gicp` wrappers generate a rosbag2 QoS override so PointCloud2
playback uses `best_effort`, matching the frontend sensor-data subscriptions.

## Optional 3D-BBS Verification

`graph_based_slam` can build MIT-licensed 3D-BBS support from
`Thirdparty/3d_bbs`. This is an optional verifier for Scan Context loop
candidates, not part of the default public benchmark path.

Build behavior:

- enabled at build time when `GRAPH_BASED_SLAM_ENABLE_3D_BBS=ON` and the vendor
  headers are present
- disabled at runtime unless `use_3d_bbs_for_scan_context: true` is set
- force-disabled with
  `colcon build --symlink-install --cmake-args -DGRAPH_BASED_SLAM_ENABLE_3D_BBS=OFF`

MID-360 wrapper example (research track, `report_only_until: v0.4` in
`scripts/release_profiles.yaml`):

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --use-3d-bbs-for-scan-context true
```

Typical outputs are written under:

- `output/bench_rko_lio_ntu_viral_<name>/traj_raw_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/traj_corrected_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/ape_raw_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/ape_corrected_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/metrics.json`

## Loop Cloud-Overlap Gate

After registration, the backend can require a fraction of aligned source
points to have target-cloud support. `loop_min_overlap_ratio: 0.0` keeps the
gate disabled for backward compatibility; `loop_overlap_max_distance_m`
defines the nearest-neighbor support radius. The cheap fitness and correction
gates run first, so rejected registrations do not pay the KD-tree cost.

Construction Seq2 validated the following dataset-specific candidate:

```yaml
loop_min_overlap_ratio: 0.76
loop_overlap_max_distance_m: 0.5
```

The ratio rejected the harmful `57 -> 123` revisit and its adjacent
substitutes while retaining five beneficial loop edges. Do not promote this
threshold to a general default until it passes the other release datasets.

For a cross-sensor candidate, the source-overlap threshold can be explicitly
relaxed only when registration applies a large translation correction:

```yaml
loop_min_overlap_ratio: 0.76
loop_min_overlap_ratio_large_correction: 0.70
loop_overlap_large_correction_translation_m: 1.0
loop_overlap_max_distance_m: 0.5
```

The effective threshold is 0.76 below 1.0 m correction and 0.70 at or above
it. Leaving either large-correction parameter at zero disables the override.
The candidate preserved the established MID-360 loop, rejected the HILTI
exp04 false loop, retained Construction Seq2's five verified edges, and
preserved KITTI 00's `28 -> 176` loop (source overlap 0.864002) with
byte-identical edge and trajectory artifacts. The generic YAML default remains
disabled while broader release validation is pending. Reverse and harmonic
overlap are emitted in debug logs for diagnosis, but are not acceptance gates
because target aggregation extent biases them.

Accepted candidates and debug attempts also report `support_rmse_m` and
`support_p90_m`. These are nearest-neighbour distances for source points that
fall within `loop_overlap_max_distance_m`; they reuse the overlap KD-tree and
do not launch another search. Treat them as diagnostics, not gates: repeated
geometry can produce low support residuals at the wrong longitudinal offset.
The p90 calculation uses linear-time selection rather than sorting all
supported points.

HILTI exp01/exp07 can be frozen and compared end to end with one command. Raw
bags and generated backend MCAPs stay on the external SSD by default:

```bash
bash scripts/run_hilti_overlap_crossval.sh --sequence all --runs 2
```

Use `--dry-run` to inspect every command, `--record-only` to stop after input
capture, or `--offline-only --resume` to reuse an existing capture. Each
sequence writes `comparison.json` and `comparison.md` beside `gate_off/` and
`gate_adaptive/`. The capture stage generates a parameter snapshot that keeps
submap publication active but disables expensive live loop registration; both
offline variants then consume the exact same odometry/cloud pairs.

## Summaries And HTML Report

To summarize all collected runs:

```bash
python3 scripts/benchmark_summary.py \
  --root output \
  --write-md output/benchmark_summary.md \
  --write-csv output/benchmark_summary.csv
```

To generate the static HTML report:

```bash
python3 scripts/generate_html_report.py \
  --root output \
  --out output/latest_report.html
```

To generate a short public-beta readiness report from the current local
artifacts:

```bash
python3 scripts/generate_v2_beta_readiness_report.py
```

By default this writes:

- `output/v2_beta_readiness_<YYYYMMDD>.md`

To generate a short public-facing map-authoring positioning report from the
tracked benchmark, GNSS, dynamic-filter, and classic-path artifacts:

```bash
python3 scripts/generate_map_authoring_report.py \
  --out output/map_authoring_report_$(date +%Y%m%d).md \
  --write-json output/map_authoring_report_$(date +%Y%m%d).json
```

To stage a reusable submission-style bundle from an existing run directory:

```bash
bash scripts/create_map_authoring_submission_bundle.sh \
  output/bench_rko_lio_ntu_viral_fresh_20260324 \
  output/submission_bundle_ntu_viral_fresh \
  --report output/map_authoring_report_$(date +%Y%m%d).md \
  --verify-map
```

That bundle standardizes:

- `pointcloud_map/`
- `map_projector_info.yaml`
- `metrics.json` when present
- trajectories and key logs when present
- focused reports under `reports/`, with sibling `json/svg` copied automatically when present
- `map_qa_summary.md`
- `manifest.json`

To generate a separate stress-validation report that distinguishes the current
default path from older long-loop and hard-dataset evidence:

```bash
python3 scripts/generate_stress_validation_report.py
```

By default this writes:

- `output/stress_validation_report_<YYYYMMDD>.md`

To summarize dynamic-object-filter behavior across the tracked Leo Drive
save-time benchmarks:

```bash
python3 scripts/generate_dynamic_object_filter_validation_report.py \
  --out output/dynamic_object_filter_validation_report_$(date +%Y%m%d).md \
  --write-json output/dynamic_object_filter_validation_report_$(date +%Y%m%d).json \
  --write-svg output/dynamic_object_filter_validation_report_$(date +%Y%m%d).svg
```

The default report compares the tracked `bag1` and `bag6` dynamic-filter
benchmarks, so point reduction and voxel-removal behavior can be discussed as
cross-dataset evidence rather than a single-case anecdote. It also reports
coarse tile-footprint preservation via shared metadata tiles, tile jaccard,
and filtered-tile overlap ratio.

To promote an already-recorded aligned cross-validation run such as the MID360
long-loop check into `metrics.json` so it appears in `benchmark_summary.md` and
`latest_report.html`:

```bash
python3 scripts/write_aligned_trajectory_metrics.py \
  --out-dir output/bench_rko_lio_mid360_v3 \
  --bag demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01 \
  --reference-tum output/glim_mid360_reference.tum \
  --corrected-tum output/bench_rko_lio_mid360_v3/traj_corrected.tum \
  --raw-tum output/bench_rko_lio_mid360_v3/traj_raw.tum \
  --graph-log output/bench_rko_lio_mid360_v3/graph_slam.log \
  --reference-source glim_mid360_reference \
  --reference-kind cross_validation \
  --reference-label GLIM \
  --points-topic /livox/lidar \
  --points-frame livox_frame \
  --robot-frame livox_frame
```

The summary/report pipeline now exposes the reference kind, so `ground_truth`
and `cross_validation` runs do not appear as if they were the same type of APE.

For a public-facing snapshot built on top of these artifacts, see
`docs/comparison.md` and `docs/releases/v0.2.2.md`.

To rerun the current MID360 cross-validation benchmark end-to-end:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

This MID360 wrapper defaults to a tuned `RKO-LIO + graph_based_slam` profile
with `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
`loop_edge_dedup_index_window=20`, and `loop_edge_info_weight=200`.

To benchmark the real open-data Leo Drive `driving_30_kmh` bag with mixed
RTK/non-RTK GNSS quality:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That wrapper writes a local `Applanix_GSOF49` reference trajectory,
`traj_raw.tum`, `traj_corrected.tum`, and `metrics.json` so the run appears in
`benchmark_summary.md` and `latest_report.html`.

When the main bag already contains native `sensor_msgs/msg/NavSatFix` or
`sensor_msgs/msg/Imu`, the same wrapper now prefers those real topics before it
falls back to Applanix sidecar generation.

Current Leo Drive packet-path evidence is:

- `driving_30_kmh`, GNSS-only classic path: `APE RMSE 195.285 m`
- `bag1_front`, default GNSS-only path: `APE RMSE 0.139 m`
- `bag1_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.365 m`

The important result is that packet IMU deskew is usable on the native
`all-sensors` bags, but only when the benchmark is replayed conservatively.
The benchmark now defaults to `rate=1.0` for every configuration and
deterministically prefers a `/front/` packet topic when a bag contains several
Velodyne streams. The exact-revision
[bag1 evidence](evidence/leo-drive-packet-benchmark-2026-07-30.md) records the
input, software, and output hashes. The earlier `20m+` regressions were
runtime-sensitivity and sensor-selection artifacts, not proof that the deskew
math itself was fundamentally broken. To reproduce the current experimental
IMU result on the driving bag:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-imu true \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --verify-map
```

To compare the same packet path on `all-sensors-bag6` while isolating IMU
deskew from GNSS:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-gnss false \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/left/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --imu-rotation-use-orientation false \
  --verify-map
```

To summarize the current cross-dataset odom-prior validation evidence after the
classic-path runs have been recorded:

```bash
python3 scripts/generate_odom_prior_validation_report.py \
  --out output/odom_prior_validation_report_$(date +%Y%m%d).md \
  --write-json output/odom_prior_validation_report_$(date +%Y%m%d).json \
  --write-svg output/odom_prior_validation_report_$(date +%Y%m%d).svg
```

This report intentionally compares `driving_30_kmh` and `bag6_front` side by
side, because the current velocity-based prior helps the fallback classic path
on one dataset and hurts or helps differently on another.

To validate packet IMU deskew as a repeatable matrix on real open data, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

That matrix compares `no_imu` and native-IMU runs for the default `bag1_front`
and `bag6_front` cases at `rate=1.0` and emits:

- `packet_imu_deskew_validation.md`
- `packet_imu_deskew_validation.json`

The report is generated by `generate_packet_imu_deskew_validation_report.py`
and fails if any case violates the configured path-coverage, RMSE-regression,
or matched-pose thresholds.

The same bag also exposes native `/gnss/fix`. The backend now falls back to
receive time when the NavSatFix header stamp is far from ROS time
(`gnss_header_stamp_max_skew_sec`, default `30 s`), which lets the graph attach
GNSS edges on `all-sensors-bag6`. In practice that native `/gnss/fix` still
disagrees with the `GSOF49` reference enough to degrade the cross-validation
APE, so `all-sensors-bag6` is useful for georeferenced smoke tests but not a
clean GNSS benchmark source.

To compare place-recognition behavior on MID360, rerun the same benchmark with
and without an optional descriptor family and then render the short report:

```bash
bash scripts/run_place_recognition_benchmark.sh
```

To compare the current experimental BEV-assisted distance rerank instead:

```bash
bash scripts/run_place_recognition_benchmark.sh --candidate-mode bev_rerank
```

The report shows:

- runtime `use_scan_context`
- accepted/attempted loop counts
- accepted loop source counts
- observed `ScanContext loop candidate` count
- observed `BEV rerank hint` count
- observed `SOLiD rerank candidate` count
- `APE RMSE` delta between the two runs
- optional JSON summary via `--write-json`
- optional SVG summary via `--write-svg`

The report is generated by `generate_place_recognition_report.py`.

Current checked-in evidence is:

- fair current-code baseline rerun:
  `output/bench_rko_lio_mid360_current_default_rerun_20260326/metrics.json`
  (`APE RMSE 4.096 m`)
- current best checked-in Scan Context candidate with DB/index fix,
  aggregated descriptor/registration cloud, and `scan_context_threshold=0.55`:
  `output/bench_rko_lio_mid360_sc055_yawguess_scagg_screg_20260326/metrics.json`
  (`APE RMSE 3.568 m`)
- current experimental BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202840/metrics.json`
  (`APE RMSE 3.607 m`)
- best observed BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202119/metrics.json`
  (`APE RMSE 3.533 m`)
- short comparison report:
  `output/place_recognition_report_20260326.md`

That candidate currently beats both the fair rerun baseline and the published
`3.641 m` default artifact, but the accepted loop still comes from the
distance-based path. Treat `use_scan_context=true` as an opt-in tuning path
rather than the repository default.

The BEV path is now more useful as a sensor-agnostic distance-candidate rerank
than as a standalone loop source. It has shown better-than-baseline runs, but
its rerun variance is still too large for a default-on setting.

To summarize the current stop/go decisions for place recognition and the
classic fallback path in one short report:

```bash
python3 scripts/generate_exploration_closeout_report.py \
  --out output/exploration_closeout_report_$(date +%Y%m%d).md \
  --write-json output/exploration_closeout_report_$(date +%Y%m%d).json
```

A local snapshot can be written to:

- `output/exploration_closeout_report_20260327.md`

That report fixes the current repository position in one place:

- public default place recognition remains the distance-based path
- `Scan Context` stays opt-in
- `BEV-assisted rerank` stays experimental
- `SOLiD` stays experimental/off by default
- the classic path remains a fallback workflow rather than the main public path

## Dynamic Object Filter Benchmark

The dynamic-object filter is save-time only. It does not change live odometry
or loop closure, so the right comparison is the saved map output with the same
bag and the same backend settings.

Run the paired comparison on the open-data bag6 smoke path:

```bash
bash scripts/run_dynamic_object_filter_benchmark.sh
```

That wrapper:

- runs `run_open_data_gnss_smoke.sh` twice on the same bag
- saves `no_filter/` and `dynamic_filter/` outputs under one root
- renders `dynamic_object_filter_report.md`,
  `dynamic_object_filter_report.json`, and `dynamic_object_filter_report.svg`

The report is generated by `generate_dynamic_object_filter_report.py` and
tracks:

- Autoware map verify result for both runs
- projector type
- saved grid cell count
- metadata tile count
- total saved point count
- filter candidate/kept/removed voxel counts
- saved-point reduction ratio

The current checked-in evidence is:

- baseline smoke:
  `output/open_data_gnss_smoke_bag6_autodetect_throttled_20260325`
- filtered smoke:
  `output/open_data_gnss_smoke_bag6_dynamic_filter_20260326`
- benchmark report bundle:
  `output/dynamic_object_filter_benchmark_bag6_20260326`

In that checked run, the saved map went from `138732` to `87861` points while
keeping `verify_autoware_map.py` at `PASS`.

## Leo Drive Classic Path Benchmark

To compare the current classic `scanmatcher + graph_based_slam` path on the
mixed-quality Leo Drive `driving_30_kmh` open-data bag, run:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_classic_path_benchmark_suite.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

This wrapper emits:

- `classic_path_report.md`
- `classic_path_report.json`
- `classic_path_report.svg`

The report is generated by `generate_classic_path_report.py`.

The checked-in snapshot is:

- `output/classic_path_report_20260327.md`

Current evidence is:

- `no GNSS`: `APE RMSE 313.695 m`
- `GNSS only`: `APE RMSE 195.285 m`
- `GNSS + velocity-based planar odom prior`: `APE RMSE 175.732 m`
- `GNSS + IMU`: `APE RMSE 271.144 m`

So the classic path still needs work, but the direction is clearer now:
backend GNSS helps substantially, and a velocity-based planar odom prior helps
further on `driving_30_kmh`, while the current packet IMU path is still not a
default recommendation.

## Release/Readiness Gate

To run the local readiness gate in one command:

```bash
bash scripts/run_release_readiness_checks.sh --fail-on-profiles
```

That wrapper can run:

- default build and package tests
- benchmark summary generation
- HTML report generation
- optional public MID-360 segment-reset completion gate
- standalone public MID-360 continuous kidnap-relocalization gate
- optional Autoware dogfood

The release command uses the per-dataset profile thresholds. With
`--fail-on-profiles`, it exits non-zero when a blocking profile exceeds its
threshold or has no matching run. A passing synthetic fixture therefore
checks reporting mechanics but cannot count as release evidence.

For a one-off uniform threshold check, `--ape-threshold <metres>` is also
hard:

- it exits non-zero if `--benchmark-root` contains no `metrics.json` evidence
- it exits non-zero if any selected run is missing APE
- it exits non-zero if any selected run exceeds the threshold
- by default `run_release_readiness_checks.sh` applies that hard gate only to
  `ground_truth` runs; `cross_validation` runs stay visible in reports without
  blocking release

`--fail-on-profiles` requires an active, existing release-profile YAML.
Profiles marked `report_only_until` remain non-blocking even when their data
is absent. Neither hard benchmark gate can be combined with
`--skip-benchmark-summary`. Without `--ape-threshold` or
`--fail-on-profiles`, an empty benchmark root remains report-only and the
wrapper records that benchmark reporting was skipped.

For the public MID-360 segment-reset completion evidence, add:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --public-mid360-completion
```

That hook runs `scripts/run_mid360_robot_public_completion_gate.py` as a hard
gate and writes its JSON/Markdown under the release-readiness output directory.

For the continuous RKO-LIO kidnap-relocalization evidence, run:

```bash
python3 scripts/run_mid360_robot_public_continuous_relocalization_gate.py
```

That gate checks the merged public `outdoor_kidnap_a+b` run for full-duration
RKO output, at least one global relocalization event, loop-alignment PASS,
public loop endpoint closure at the GT start/end stamps, Autoware map verify
PASS, offline completion, and tracked kidnap recovery config matching the run
config. The endpoint closure check prevents a local revisit from being counted
as continuous kidnap relocalization.

## CI Coverage

CI exercises the reporting path in two ways:

- a passing synthetic benchmark fixture must generate summary and HTML report
- a failing synthetic benchmark fixture must trip the threshold gate with
  exit code `2`

The fixture generator is:

```bash
python3 scripts/generate_sample_benchmark_metrics.py \
  --root /tmp/ci_fixture \
  --profile passing
```

Use `--profile failing` to create a negative-path fixture.

## Recommended Artifacts To Publish

If you want benchmark results to be easy to consume, publish:

- `metrics.json`
- `benchmark_summary.md`
- `benchmark_summary.csv`
- `latest_report.html`
- the exact param file used for the run
- `docs/comparison.md` when publishing the current positioning of the repo
- `docs/releases/v0.2.2.md` when publishing the current public beta scope
- `v2_beta_readiness_<YYYYMMDD>.md` when preparing a public beta snapshot
- `stress_validation_report_<YYYYMMDD>.md` when discussing long-loop or
  aggressive-motion evidence

## Related Commands

- Autoware quickstart: `docs/autoware-quickstart.md`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- public comparison page: `docs/comparison.md`
- end-to-end dogfood: `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20`
