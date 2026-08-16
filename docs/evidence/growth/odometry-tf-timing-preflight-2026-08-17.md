# Odometry TF timing preflight — 2026-08-17

## Decision

Extend the existing read-only `lidarslam-map doctor <rosbag2_dir>` contract
with replay-order timing evidence. Do not change scan matching, suppress TF
warnings, increase a lookup timeout, or substitute a stale transform.

This increment addresses the diagnosis burden in still-open
[Issue #64](https://github.com/rsasaki0109/lidar_slam_ros2/issues/64). The
reported example requested a transform at 20.364 s while the latest transform
was at 20.346 s. A later reporter eliminated the warning and path drift only
after raising the *actual* Odometry/TF update rate. The issue and its comments
were read only; no GitHub issue state was changed.

## Preflight v6 contract

Preflight schema v6 keeps v1–v5 immutable. The existing bounded connectivity
scan now publishes the selected path edges and whether each is dynamic. When a
PointCloud2 topic and a usable dynamic Odometry path both exist, doctor makes
one additional sequential bag pass, bounded to 100,000 records per selected
topic. It:

- preserves rosbag reader order as the playback-order sample basis;
- tracks the greatest valid `header.stamp` observed so far for every required
  dynamic path edge;
- counts clouds seen before all dynamic edges as startup gaps;
- counts every cloud stamp newer than the limiting latest TF stamp as a future
  extrapolation gap, without an arbitrary time threshold; and
- reports the first affected cloud, largest gap, limiting edge, bounded record
  counts, and completeness.

The Issue #64 regression fixes the values at 20.346 s and 20.364 s and requires
an exact maximum gap of 18,000,000 ns. Multi-hop coverage verifies that the
oldest required dynamic edge limits the result; static path edges are ignored
for this timing comparison.

Stable findings keep a compatible maintained mapping recommendation visible:

- `odometry-tf-startup-gap`
- `odometry-tf-future-gap`
- `odometry-tf-timing-invalid`
- `odometry-tf-timing-inspection-unavailable`

## Claim boundary

This is recorded-bag, reader-order, header-stamp evidence. A clean result says
that no sampled cloud was newer than the latest required dynamic TF stamp seen
earlier in that bag pass. It does not prove live executor order, DDS latency,
clock alignment, TF buffer duration, past-side availability, interpolation,
publisher rate, calibration, mapping success, trajectory accuracy, or support
for a new robot. Operators must still rerun `tf2_monitor` on the live system.

The command remains read-only: it does not modify a bag, message, parameter,
publisher, transform, launch file, issue, or external service.

## Local verification

The initial focused preflight run passed 29 tests with two environment-specific
serialized-reader cases skipped where their ROS/rosbags dependencies were not
available. Exact implementation commit, full package gates, strict docs,
release-bundle, publication-plan, and public CI evidence are recorded only
after the final candidate is committed and validated.
