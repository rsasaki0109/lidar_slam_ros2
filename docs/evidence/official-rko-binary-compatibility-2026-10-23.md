# Official RKO-LIO binary compatibility — 2026-10-23

Status: **PASS**

The release-shaped product was built without the RKO-LIO source submodule
and run against the official `0.4.2-1` binaries on both supported ROS
distributions. Each job used the pinned public MID-360 dataset and the
maintained `rko_lio_graph_mid360_preset` golden path.

| ROS distro | Official Debian build | E2E checks | Runtime | Raw / corrected poses | Tiles |
| --- | --- | ---: | ---: | ---: | ---: |
| Humble | `0.4.2-1jammy.20261013.225901` | 18 / 18 | 81.292 s | 2772 / 637 | 368 |
| Jazzy | `0.4.2-1noble.20260930.090637` | 18 / 18 | 96.964 s | 2772 / 637 | 368 |

The gate proved that `offline_node` resolved from `/opt/ros/<distro>`,
recorded the executable SHA-256, reached terminal success, passed the
Autoware map diagnosis with 8 passes and 0 failures, and met all trajectory
and pointcloud evidence thresholds. No map geometry, trajectories, or point
clouds were uploaded.

Permanent machine-readable evidence is stored in
[`official-rko-binary-compatibility-2026-10-23.json`](official-rko-binary-compatibility-2026-10-23.json).
The originating public workflow is
[run 39892845149](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/39892845149).

## What changed since the 2026-07-29 record

- Official `rko_lio` 0.4.1 renamed the deskewed scan topic from
  `rko_lio/frame` to `rko_lio/deskewed_scan` and publishes the scan in the
  base frame instead of the LiDAR frame. Against `0.4.2-1` the previous
  product received no scans and failed with `initial map is not received`
  ([run 39872328678](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/39872328678)).
  The RKO launch now pins `deskewed_scan_topic`, and graph_based_slam applies
  the LiDAR-to-base extrinsic only to scans that are not stamped with the
  odometry's base frame
  ([#561](https://github.com/rsasaki0109/lidar_slam_ros2/pull/561)).
- The MID-360 contract's pointcloud floor is 20 MB instead of 50 MB. The
  official `0.4.2-1` map stays below the 32-bit voxel-index limit of the
  0.2 m map downsampling and is downsampled as intended (about 27 MB); the
  0.3.2 maps were saved undownsampled (about 64 MB). See
  [real-data E2E](../real-data-e2e.md).

`lidarslam` keeps its `rko_lio >= 0.3.2` dependency. On 2026-10-23 ROS main
and testing both carry `rko_lio 0.4.2-1` for Humble and Jazzy. This closes
official-binary compatibility for the maintained MID-360 golden path only.
Fork-only research profiles remain evaluation scope.
