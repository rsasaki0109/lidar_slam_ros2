# Classic scanmatcher VoxelGrid overflow safety — 2026-08-11

> Status: **LOCAL_IMPLEMENTATION_PASS_PUBLICATION_PENDING**
>
> Implementation commit:
> `a2368c486fc35c0edcac6d9dbf2f9cb89475c820`
>
> Public issue: [#69 — scanmatcher_node-1 process has died](https://github.com/rsasaki0109/lidar_slam_ros2/issues/69)
>
> Remote changes made by this work: **none**

## Decision

The classic scanmatcher must never use PCL's unfiltered-copy fallback as its
overflow behavior. Every explicit PCL `VoxelGrid` call is now behind one
fail-closed wrapper. Unsafe input clears the candidate output, emits a stable
reason and recovery action, and returns control without replacing the last
valid map or registration target. Valid input still uses PCL with the same
effective float leaf size; the bounded parity test produces exactly equal
XYZ/intensity points.

This closes the local code hazard. It does not close public issue #69 because
the implementation commit is not yet publicly resolvable, the historical
private rosbag was not retained or replayed, and no reviewed public integration
or release carries the fix.

## Upstream behavior being contained

The official PCL
[1.12.1 implementation](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/filters/include/pcl/filters/impl/voxel_grid.hpp)
and
[1.14.1 implementation](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.14.1/filters/include/pcl/filters/impl/voxel_grid.hpp)
both calculate three grid dimensions, compare their product with signed
32-bit maximum, warn on overflow, assign the input cloud to the output, and
return. The warning is therefore not a filtered or rejected result.

Issue #69 records that warning immediately before scanmatcher exited with
`-8` (`SIGFPE`). The original parameters included `vg_size_for_map: 0.1` and a
`200 m` scan range; increasing the leaf size mitigated at least one later user
report but did not provide a safe runtime contract.

The wrapper is deliberately more conservative than the upstream span-only
check. It also rejects:

- a requested leaf that is non-finite, non-positive, not representable as a
  positive finite float, or has a non-finite float inverse;
- a cloud marked `is_dense=true` that contains non-finite XYZ;
- an absolute `floor(coordinate / leaf)` outside signed 32-bit range;
- the exact floored voxel-division product above `2,147,483,647`.

All multiplication is bounded before it occurs, so the preflight does not
introduce another signed-overflow path.

## Call-site behavior

Commit `a2368c4` removes every direct `pcl::VoxelGrid` instance from
`scanmatcher_component.cpp` and routes all five stages through
`filterVoxelGridSafely`.

| Stage | Leaf parameter | Rejection behavior | Preserved state |
| --- | --- | --- | --- |
| `initial_map` | `vg_size_for_map` | return `false`; wait for another usable scan | node, configuration, and publishers remain active |
| `input_scan` | `vg_size_for_input` | skip the scan before registration | current pose, path, map, and registration target |
| `map_update` | `vg_size_for_map` | return from synchronous or asynchronous update | existing map array and registration target |
| `registration_target` | `vg_size_for_input` | do not install the unsafe newly built target | previously installed registration target |
| `recovery_target` | `vg_size_for_input` | return `false` from target refresh | existing target and recovery state |

Recurring runtime warnings are throttled to one per five seconds at each macro
site. Initial-map refusal is not throttled because it is the immediate reason
mapping cannot initialize. No path automatically increases a leaf size or
clips coordinates; both would silently change map resolution or data.

## Privacy-safe bounded reproducer

The regression fixture contains only these two synthetic points:

```text
[-200.0, -200.0, -10.0]
[ 200.0,  200.0,  10.0]
```

With an effective `0.1 m` leaf, the conservative divisions are
`4001 x 4001 x 201`, or `3,217,608,201` cells. This exceeds PCL's signed
32-bit `2,147,483,647` limit and produces
`VOXEL_GRID_LAYOUT_OVERFLOW`. The wrapper empties a pre-populated output cloud
instead of copying these input points through.

The test is a failure-class reproducer, not a claim that these were the exact
bounds in the unavailable historical bag. No bag, map geometry, issue author,
comment author, local path, or private sensor data was copied into the
repository.

## Automated coverage

The new `test_voxel_grid_safety` suite has 11 cases:

1. valid output parity with direct PCL, including intensity;
2. the bounded issue #69 overflow class and empty rejected output;
3. the largest cubic integer layout below the PCL limit;
4. the first adjacent cubic layout above the limit;
5. negative voxel indices;
6. absolute signed-32-bit index overflow;
7. zero, negative, NaN, infinite, and float-underflow leaf sizes;
8. inconsistent dense/non-finite input;
9. valid non-dense input with PCL dropping non-finite XYZ;
10. empty, null, and all-non-finite input;
11. actionable diagnostic content and stable reason code.

Every classic component call site uses the tested wrapper. The component no
longer contains a direct `pcl::VoxelGrid` construction.

## Supported-distribution execution

| Environment | Exact substrate | Build | New suite | Complete scanmatcher CTest |
| --- | --- | --- | --- | --- |
| Humble | immutable local image `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447`; PCL `1.12.1+dfsg-3build1`; GCC `11.4.0`; installed `lidarslam_msgs 0.9.0` and `ndt_omp_ros2 0.1.0` underlay | PASS, read-only source mount and clean `/tmp` build/install | 11 / 11 PASS | 9 / 9 PASS |
| Jazzy | Ubuntu 24.04 host; PCL `1.14.0+dfsg-1`; GCC `13.3.0`; installed `lidarslam_msgs 0.6.0` and `ndt_omp_ros2 0.0.0` underlay | PASS, clean temporary build/install | 11 / 11 PASS | 9 / 9 PASS |

The complete CTest set includes lidar undistortion, math utilities, odometry
prior, pose prediction, pose acceptance, IMU processing, map-update policy,
point colorization, and the new safety suite. Humble emitted only the existing
PCL CMake policy warning. Jazzy additionally emitted the existing PCL 1.14
deprecated-Boost-header notice; neither build emitted a new-code diagnostic.

Formatting and documentation checks also passed:

- `ament_uncrustify` on the new header and test;
- `ament_cpplint --filters=-legal/copyright` on the new header and test, matching
  the package's existing no-header-banner convention;
- `mkdocs build --strict`;
- `git diff --check`.

## Remaining public gate

Issue #69 should remain open until all of these are true:

1. `a2368c4` or a reviewed descendant is publicly resolvable;
2. CI reproduces both supported build/test rows from that public revision;
3. a bounded component or supported-route execution proves the process stays
   alive after an unsafe frame and can accept a later safe frame;
4. the public issue response explains the two leaf parameters and reason codes
   without claiming the unavailable historical bag was exactly reproduced;
5. the fix is included in a named release or the issue explicitly states the
   first release expected to contain it.

Until then the honest state is local implementation PASS, public resolution
pending. No issue label, comment, state, branch, pull request, image, or release
was changed during this work.
