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

To generate a separate stress-validation report that distinguishes the current
default path from older long-loop and hard-dataset evidence:

```bash
python3 scripts/generate_stress_validation_report.py
```

By default this writes:

- `output/stress_validation_report_<YYYYMMDD>.md`

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
`docs/comparison.md` and `docs/releases/v0.2.0.md`.

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
and without Scan Context and then render the short report:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --output-dir output/bench_rko_lio_mid360_pr_distance \
  --use-scan-context false
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --output-dir output/bench_rko_lio_mid360_pr_scan_context \
  --use-scan-context true
python3 scripts/generate_place_recognition_report.py \
  --baseline-metrics output/bench_rko_lio_mid360_pr_distance/metrics.json \
  --candidate-metrics output/bench_rko_lio_mid360_pr_scan_context/metrics.json \
  --out output/place_recognition_report_$(date +%Y%m%d).md
```

The report shows:

- runtime `use_scan_context`
- accepted/attempted loop counts
- accepted loop source counts
- observed `ScanContext loop candidate` count
- `APE RMSE` delta between the two runs

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
- `docs/releases/v0.2.0.md` when publishing the current public beta scope
- `v2_beta_readiness_<YYYYMMDD>.md` when preparing a public beta snapshot
- `stress_validation_report_<YYYYMMDD>.md` when discussing long-loop or
  aggressive-motion evidence

## Related Commands

- Autoware quickstart: `docs/autoware-quickstart.md`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- public comparison page: `docs/comparison.md`
- end-to-end dogfood: `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20`
