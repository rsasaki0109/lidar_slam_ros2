# RKO-LIO profile inventory (2026-10)

Phase 1 input for the [SOTA v7 Plan](../roadmap/sota-v7-plan-2026-10.md). Each column is
the profile that the current comparison or benchmark report quotes for one dataset family;
each row is an algorithm key whose value differs from the RKO-LIO default (vendored
`Thirdparty/rko_lio` at `ec61a3e`). Rig keys (topics, frames, extrinsics, units, clock
offsets, beam model, output pacing) are left out; `--json` lists them.

Generated with `python3 scripts/rko_profile_inventory.py`. The profile paths are in
`FAMILY_PROFILES` in that script.

| key | default | mapshd_humanoid_mid360 | oxford_spires_hesai | hilti2022_pandar_exp04 | hilti2022_competitive_v2 | ntu_viral_competitive_v2 | enwide_os0_v10 | geode_alpha_handheld_v18 | geode_alpha_v18 | geode_beta_ugv_v19 | geode_gamma_ugv_v19 | rtk_slam_mid360 | koide_mid360_handheld_outdoor | mid360_product_default | mid360_walking | kitti_odometry | families |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---:|
| `bump_image_max_rotation_correction_deg` | `0.0` |  |  |  |  |  | `2.0` | `2.0` | `2.0` | `2.0` | `2.0` |  |  |  |  |  | 5 |
| `bump_image_registration` | `False` |  |  |  |  |  | `True` | `True` | `True` | `True` | `True` |  |  |  |  |  | 5 |
| `bump_image_rotation_fallback_min_count` | `1` |  |  |  |  |  | `3` | `3` | `3` | `3` | `3` |  |  |  |  |  | 5 |
| `bump_image_rotation_fallback_window` | `1` |  |  |  |  |  | `10` | `10` | `10` | `10` | `10` |  |  |  |  |  | 5 |
| `convergence_criterion` | `1e-05` | `3e-05` | `3e-05` | `3e-05` | `3e-05` |  |  |  |  |  |  |  |  |  | `3e-05` |  | 5 |
| `deskew` | `True` |  |  |  |  |  |  |  |  |  |  | `False` |  |  |  | `False` | 2 |
| `double_downsample` | `True` |  |  |  |  |  |  |  |  |  |  | `False` |  |  |  |  | 1 |
| `gait_correction_weak_ratio` | `0.0` |  |  |  |  |  |  | `0.05` |  |  |  |  |  |  |  |  | 1 |
| `gait_hold_without_cadence` | `False` |  |  |  |  |  |  | `True` |  |  |  |  |  |  |  |  | 1 |
| `gait_max_correction_m` | `0.0` |  |  |  |  |  |  | `0.02` |  |  |  |  |  |  |  |  | 1 |
| `gait_reversal_lookback_sec` | `4.0` |  |  |  |  |  |  | `0.0` |  |  |  |  |  |  |  |  | 1 |
| `gait_sets_speed` | `False` |  |  |  |  |  |  | `True` |  |  |  |  |  |  |  |  | 1 |
| `gait_speed_prior` | `False` |  |  |  |  |  |  | `True` |  |  |  |  |  |  |  |  | 1 |
| `gravity_alignment_relevel_map` | `False` |  |  |  |  |  |  |  | `True` | `True` | `True` |  | `True` |  |  |  | 4 |
| `gravity_window_alignment` | `False` |  |  |  |  |  |  |  | `True` | `True` | `True` |  | `True` |  |  |  | 4 |
| `gravity_window_sec` | `20.0` |  |  |  |  |  |  |  |  |  |  |  | `40.0` |  |  |  | 1 |
| `icp_keypoint_voxel_multiplier` | `1.5` | `2.25` | `2.25` | `2.25` | `2.25` | `2.25` |  |  |  |  |  |  |  |  | `2.25` |  | 6 |
| `icp_levenberg_marquardt` | `False` |  |  |  |  |  |  | `True` | `True` | `True` | `True` |  |  |  |  |  | 4 |
| `initialization_max_gyro_rad_s` | `0.0` | `0.02` | `0.02` | `0.02` |  |  |  |  | `0.02` | `0.02` | `0.02` |  |  |  | `0.02` |  | 7 |
| `initialization_phase` | `False` | `True` | `True` | `True` | `True` | `True` | `True` | `True` | `True` | `True` | `True` |  | `True` | `True` | `True` |  | 13 |
| `localizability_boost` | `30.0` | `10.0` |  |  |  |  |  |  |  |  |  |  |  |  | `10.0` |  | 2 |
| `localizability_normal_only` | `False` | `True` |  |  |  |  |  |  |  |  |  |  |  |  | `True` |  | 2 |
| `localizability_weighting` | `False` | `True` |  |  |  |  |  |  |  |  |  |  |  |  | `True` |  | 2 |
| `max_correspondence_distance` | `0.5` | `0.25` |  |  |  |  |  |  |  |  |  |  |  |  | `0.25` |  | 2 |
| `max_expected_jerk` | `3` | `1000.0` | `1000.0` | `1000.0` |  |  |  |  |  |  |  |  |  |  | `1000.0` |  | 4 |
| `max_iterations` | `100` | `50` | `50` | `50` | `50` |  |  |  |  |  |  |  |  |  | `50` |  | 5 |
| `max_scan_delta_sec` | `1.0` |  |  |  |  |  |  |  |  |  |  |  | `3.0` |  |  |  | 1 |
| `max_tilt_correction_deg` | `0.0` |  |  |  |  |  |  |  | `0.02` |  |  |  |  |  |  |  | 1 |
| `min_beta` | `200` |  |  |  |  |  |  | `1000000000.0` | `1000000000.0` | `1000000000.0` | `1000000000.0` |  |  |  |  |  | 4 |
| `min_icp_keypoints` | `10` |  |  |  |  |  |  |  |  |  |  |  | `100` |  |  |  | 1 |
| `min_range` | `1.0` | `0.7` | `0.7` | `0.7` | `0.7` |  | `0.7` | `0.7` | `0.7` | `0.7` | `0.7` |  |  |  | `0.7` |  | 10 |
| `photometric` | `False` |  |  |  |  |  | `True` |  |  |  |  |  |  |  |  |  | 1 |
| `skip_registration_after_gap_sec` | `0.0` |  |  |  |  |  | `0.15` | `0.15` | `0.15` | `0.15` | `0.15` |  |  |  |  |  | 5 |
| `vehicle_stall_guard` | `False` |  |  |  |  |  |  |  | `True` | `True` | `True` |  |  |  |  |  | 3 |
| `velocity_window_sec` | `0.0` |  |  |  |  |  | `0.3` | `0.3` | `0.3` | `0.3` | `0.3` |  |  |  |  |  | 5 |
| `voxel_size` | `1.0` | `0.25` | `0.25` | `0.25` | `0.25` |  | `0.25` | `0.25` | `0.25` | `0.25` | `0.25` | `0.5` | `0.5` |  | `0.25` |  | 12 |
