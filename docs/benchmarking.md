# Benchmarking And Release Gate

This page describes the recommended benchmark path and the release/readiness
gate used for the default permissive workflow.

## Recommended Benchmark

The standard benchmark path for this repository is:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

That wrapper:

- uses the bundled NTU VIRAL `rosbag2`
- runs `RKO-LIO + graph_based_slam`
- saves raw and corrected trajectories
- computes APE against the Leica prism reference
- verifies the Autoware map bundle when present
- writes `metrics.json` for the reporting pipeline

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

MID360 wrapper example:

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
- `bag1_front`, `no_imu`: `APE RMSE 0.248 m`
- `bag1_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.365 m`

The important result is that packet IMU deskew is usable on the native
`all-sensors` bags, but only when the benchmark is replayed conservatively.
The wrapper now auto-selects `rate=1.0` whenever `--use-imu=true` and `--rate`
is omitted. The earlier `20m+` regressions were runtime-sensitivity artifacts,
not a proof that the deskew math itself was fundamentally broken. To reproduce
the current experimental IMU result on the driving bag:

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
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

That wrapper can run:

- default build and package tests
- benchmark summary generation
- HTML report generation
- optional Autoware dogfood

With `--ape-threshold`, the gate is hard:

- it exits non-zero if any selected run is missing APE
- it exits non-zero if any selected run exceeds the threshold
- by default `run_release_readiness_checks.sh` applies that hard gate only to
  `ground_truth` runs; `cross_validation` runs stay visible in reports without
  blocking release

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
