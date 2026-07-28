# ndt_omp_ros2 release-quality acceptance — 2026-07-28

## Result

The proposed `ndt_omp_ros2` release source passed clean archive build,
install, package-test and downstream-consumer gates on ROS 2 Humble and
Jazzy. This closes the source/buildfarm-quality risk. It does not make the
package available from apt; merge, tag, bloom and rosdistro review are still
required.

The same gate is now enforced by repository CI. Its
[first GitHub Actions run](https://github.com/rsasaki0109/ndt_omp_ros2/actions/runs/30358865118)
passed both matrix jobs.

## Boundary

| Item | Recorded value |
| --- | --- |
| Repository | `https://github.com/rsasaki0109/ndt_omp_ros2` |
| Branch | `release/buildfarm-quality` |
| Source revision | `a9b30d1e10effe5a794e5e29c402a064ff5f0278` |
| Packaging implementation revision | `f0326f6a021e0151c0f5a0c926d98408475a56e0` |
| Package version | `0.1.0` |
| Automated source input | GitHub checkout of the recorded revision |
| Initial source input | `git archive` of the packaging implementation revision |
| Build type | `RelWithDebInfo` |
| Humble CI environment | `ros@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16` |
| Jazzy CI environment | `ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f` |
| CI run | `30358865118` — success |

The initial archive gate also ran with the pinned product build images while
bypassing their entrypoints and pre-existing workspace prefixes. The
automated gate is stricter at dependency resolution: it starts from official
ROS base images and installs only dependencies declared through rosdep.

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
| GitHub Actions job | PASS (2m42s) | PASS (2m39s) |

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
