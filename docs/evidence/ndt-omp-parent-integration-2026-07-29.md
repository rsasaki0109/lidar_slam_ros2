# ndt_omp_ros2 parent integration — 2026-07-29

## Result

The `ndt_omp_ros2` release-quality source was merged by fast-forward into its
public `humble` branch at
`8b77fa5a6cdcad45bf35918361c892b6d94a287e`. The `lidar_slam_ros2` source
checkout now pins that public commit.

A clean external build on ROS 2 Jazzy verified that the new installed library
and headers remain source-compatible with the parent scan-matching frontend.
The check used no build, install, or log directories inside the repository
worktree.

## Scope

| Item | Recorded value |
| --- | --- |
| ROS environment | Jazzy |
| Source dependency | `ndt_omp_ros2` `8b77fa5a6cdcad45bf35918361c892b6d94a287e` |
| Parent branch | `product/rosdistro-boundary` |
| Build type | `RelWithDebInfo` |
| Packages built | `ndt_omp_ros2`, `lidarslam_msgs`, `scanmatcher` |
| Build result | 3 packages finished |
| Test result | 101 tests, 0 errors, 0 failures, 0 skipped |

The build compiled and linked the parent `scanmatcher_component`,
`scanmatcher_node`, and `scan_matcher_offline_runner` against the installed
`ndt_omp_ros2` package. This covers the custom rotation-prior,
translation-prior, and correspondence-distance API used by the product
frontend, in addition to the dependency repository's own public API test.

The only emitted diagnostics were existing PCL/CMake developer deprecation
warnings. They did not affect configure, compile, link, install, or test
results.

## Publication boundary

This closes the parent source-integration risk. It does not make
`ros-humble-ndt-omp-ros2` or `ros-jazzy-ndt-omp-ros2` available from apt.
The remaining external sequence is:

1. tag the merged dependency source as `0.1.0`;
2. create or configure the Bloom release repository;
3. submit and merge Humble and Jazzy rosdistro release changes;
4. verify the installed dependency from the ROS testing repository;
5. release the four `lidar_slam_ros2` packages and run installed first-map
   acceptance.
