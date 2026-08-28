# SLAM-only memory reduction (2026-07-14)

## Scope

This change targets ordinary map-building SLAM only. Saved-map loading,
localization mode, and relocalization are outside the development scope.

RKO-LIO previously transformed every accepted scan into the world frame and
inserted it into an unpruned global recovery map even when
`enable_kidnap_relocalization` was false. The recovery map is now populated
only when that opt-in feature is enabled. The sliding odometry map and all
SLAM outputs are unchanged.

## Reproducible A/B

Dataset: HILTI 2022 `exp07_ros2`, PandarXT-32 and Alphasense IMU. Both runs
used the same built binary and `--skip-map-save`; the only configuration
difference was `enable_kidnap_relocalization`.

Artifacts:

- ON: `/media/sasaki/aiueo/benchmarks/hilti_exp07_reloc_map_on_ab_20260714_v1`
- OFF: `/media/sasaki/aiueo/benchmarks/hilti_exp07_reloc_map_off_ab_20260714_v1`

| Metric | Recovery map ON | SLAM-only OFF | Change |
|---|---:|---:|---:|
| Maximum RSS | 484,284 KiB | 398,924 KiB | -17.6% |
| Pipeline wall time | 43.064 s | 42.924 s | -0.3% |
| Raw trajectory rows | 1,322 | 1,322 | identical |
| Raw APE RMSE | 0.318106944727 m | 0.318106944727 m | identical |

`traj_raw.tum` is byte-identical between the two runs. The timing result is a
tie within host-load noise; the accepted quantitative gain is the 17.6%
maximum-RSS reduction with no accuracy or output regression.
