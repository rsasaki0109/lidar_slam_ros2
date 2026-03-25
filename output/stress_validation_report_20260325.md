# Stress Validation Report

Date: 2026-03-25

## Verdict

`lidarslam_ros2` has usable long-loop evidence and a solid current default-path
benchmark on NTU VIRAL, but aggressive-motion validation for the exact `v2`
default path is still incomplete.

This is strong enough for a public `beta`, but not strong enough to claim that
the current default path is already the fully stress-validated standard across
aggressive motion and long-range loop closures.

## Current Default-Path Evidence

- dataset: `NTU VIRAL tnp_01`
- fresh default-path APE RMSE: `0.952 m`
- current best APE RMSE: `0.870 m`
- fresh map verify: `PASS`
- fresh pointcloud-map total points: `265,551`
- benchmark summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/benchmark_summary.md`

## Long-Loop Evidence

### MID360 Cross-Validation

- matched poses: `579`
- aligned path length: `1010.284 m`
- current-path cross-validation APE RMSE: `3.641 m`
- current-path raw APE RMSE: `10.220 m`
- loop-search distance seen in log: `936.1 m`
- loop closure example from log: `14 -> 581`

This is useful evidence that the graph backend has been exercised on a path of
roughly one kilometer with at least one long-range loop closure event recorded
in the log. It is also a clear sign that the current path still needs more work
on this dataset, because the current-path cross-validation error is much worse
than the NTU VIRAL result.

### MID360 Legacy Sample Context

- older sample-summary APE RMSE: `0.457 m`

This older artifact is still useful context, but it does not override the
current-path MID360 metrics above.

### Newer College math-hard

- dataset note from benchmark README: `320m (loop closure available)`
- reference path length: `320.564 m`
- legacy `lidarslam` RMSE: `12.164 m`

This artifact is useful as a hard-dataset reminder, but it is not evidence for
the current `RKO-LIO + graph_based_slam` default path. It should not be cited
as the current release-quality benchmark.

## Aggressive-Motion / Hard-Data Evidence

- legacy NTU VIRAL summary RMSE: `0.216 m`
- legacy NTU VIRAL prism summary RMSE: `0.117 m`

These NTU VIRAL summaries show that this repository has already been exercised
on a harder real-world sequence, but they are older report artifacts and do not
fully replace a fresh stress benchmark on the current `v2` default path.

## Release Interpretation

- safe claim: the current default path is benchmarked, dogfooded into Autoware,
  and backed by additional long-loop evidence
- unsafe claim: the current default path is already fully validated against
  aggressive motion and long-loop stress on multiple fresh benchmark datasets
- next benchmark to close the gap: rerun the current `RKO-LIO + graph_based_slam`
  path on `Newer College math-hard` and/or promote `MID360` into the same
  `metrics.json` reporting pipeline used by `NTU VIRAL`

## Source Artifacts

- fresh metrics: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_ntu_viral_fresh_20260324/metrics.json`
- best metrics: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_ntu_viral_loopgate_20260324/metrics.json`
- benchmark summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/benchmark_summary.md`
- MID360 current metrics: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_mid360_current_default_20260325/metrics.json`
- MID360 graph log: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/bench_rko_lio_mid360_current_default_20260325/slam.launch.log`
- MID360 legacy summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/mid360_compare_summary.json`
- benchmark README: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/BENCHMARK_README.md`
- Newer College summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/newer_college_mathhard_report_summary.json`
- NTU VIRAL legacy summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/ntu_viral_tnp01_report_summary.json`
- NTU VIRAL prism summary: `/media/sasaki/aiueo/ai_coding_ws/ros2/output/ntu_viral_tnp01_report_threads1_prism_summary.json`
