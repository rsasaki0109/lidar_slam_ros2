# ndt_omp_ros2 release-quality acceptance — 2026-07-28

## Result

The proposed `ndt_omp_ros2` release source passed clean archive build,
install, package-test and downstream-consumer gates on ROS 2 Humble and
Jazzy. This closes the source/buildfarm-quality risk. It does not make the
package available from apt; merge, tag, bloom and rosdistro review are still
required.

## Boundary

| Item | Recorded value |
| --- | --- |
| Repository | `https://github.com/rsasaki0109/ndt_omp_ros2` |
| Branch | `release/buildfarm-quality` |
| Source revision | `f0326f6a021e0151c0f5a0c926d98408475a56e0` |
| Package version | `0.1.0` |
| Source input | `git archive` of the recorded revision |
| Build type | `RelWithDebInfo` |
| Humble environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061` |
| Jazzy environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` |

The image entrypoint and pre-existing workspace prefixes were bypassed.
Only `/opt/ros/<distro>` was sourced before the archive was built in a new
`/work` workspace.

## Changes under test

- The library is installed and exported as
  `ndt_omp_ros2::ndt_omp`.
- The benchmark remains available as
  `ros2 run ndt_omp_ros2 align`.
- Caller/buildfarm `CMAKE_BUILD_TYPE` is no longer overwritten.
- OpenMP uses the imported `OpenMP::OpenMP_CXX` target rather than global
  compiler flags.
- The unused `std_msgs` dependency was removed.
- A public API test and an independent installed-consumer fixture were added.

## Acceptance results

| Gate | Humble | Jazzy |
| --- | --- | --- |
| Archive configure/build | PASS | PASS |
| Install `lib/libndt_omp.a` | PASS | PASS |
| Install `lib/ndt_omp_ros2/align` | PASS | PASS |
| Public API package test | PASS | PASS |
| Colcon result | 2 tests, 0 errors/failures/skips | 2 tests, 0 errors/failures/skips |
| Fresh CMake consumer configure/link/run | PASS | PASS |

The downstream fixture uses only:

```cmake
find_package(ndt_omp_ros2 REQUIRED)
target_link_libraries(consumer ndt_omp_ros2::ndt_omp)
```

## Remaining publication sequence

1. Review and merge the release-quality branch.
2. Re-run this gate at the merge commit and create the `0.1.0` tag.
3. Create `rsasaki0109/ndt_omp_ros2-release`.
4. Run bloom for Humble and Jazzy and merge the rosdistro PRs.
5. Release the four `lidarslam_ros2` packages and run installed first-map
   acceptance on both distributions.
