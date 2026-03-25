# v2 Beta Readiness Report

Date: 2026-03-24

## Verdict

`lidarslam_ros2` is in a reasonable state for a public `v2 beta` release.

The current default path is:

- `RKO-LIO + graph_based_slam`
- Autoware-compatible pointcloud map export
- pointcloud-only Autoware dogfood

## Evidence

### Benchmark Snapshot

- default-path fresh run APE RMSE: `0.952 m`
- current best run APE RMSE: `0.870 m`
- fresh run wall time: `79.04 s`
- fresh run RTF: `0.136`
- fresh run map verify: `PASS`
- fresh run total map points: `265,551`

Benchmark summary artifact:

- `/media/sasaki/aiueo/ai_coding_ws/ros2/output/benchmark_summary.md`

### Pointcloud Map Snapshot

- dogfood output dir: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/dogfood_rko_lio_autoware_20260324_190734`
- `map_projector_info.yaml`: `projector_type: Local`
- pointcloud tiles: `16`
- map grid resolution: `20.0 m x 20.0 m`
- host `rviz2` subscribed to `/map/pointcloud_map`: `yes`

Relevant dogfood artifacts:

- `/media/sasaki/aiueo/ai_coding_ws/ros2/output/dogfood_rko_lio_autoware_20260324_190734/map_projector_info.yaml`
- `/media/sasaki/aiueo/ai_coding_ws/ros2/output/dogfood_rko_lio_autoware_20260324_190734/pointcloud_map/pointcloud_map_metadata.yaml`
- `/media/sasaki/aiueo/ai_coding_ws/ros2/output/dogfood_rko_lio_autoware_20260324_190734/slam.launch.log`

### Existing Reports

- markdown summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/benchmark_summary.md`
- HTML report: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/latest_report.html`

## Why This Looks Good Enough

- all current benchmark runs in `output/benchmark_summary.md` are under the
  local `1.0 m` APE gate
- the default fresh benchmark is no longer the obviously bad outlier it was
  before the launch/config fixes
- the generated pointcloud map is structured as an Autoware bundle and has a
  current local verification pass
- the Autoware dogfood run saved a multi-tile pointcloud map and `rviz2`
  subscribed to `/map/pointcloud_map`

## Remaining Caveats

- pointcloud-map flow is validated; lanelet generation is still out of scope
- the dogfood sample above is `projector_type: Local` because GNSS was not used
- current evidence is strong enough for a public beta, not for claiming
  production maturity
- remote GitHub Actions still need to be run on the final release branch/commit

## Source Artifacts

- fresh metrics: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_ntu_viral_fresh_20260324/metrics.json`
- best metrics: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_ntu_viral_loopgate_20260324/metrics.json`
- benchmark summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/benchmark_summary.md`
- latest HTML report: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/latest_report.html`
