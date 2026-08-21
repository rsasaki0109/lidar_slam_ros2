# ADR: versioned registration plugin API

- Status: accepted; Phase 0 interface, Phase 1 NDT/GICP/optional-SMALL adapters
  and static characterization paths, Phase 2 shell-loader/offline injection,
  and the ODR-safe hybrid resolver implemented; optional SMALL pluginlib DSO
  discovery/replay is characterized, while independent ODR isolation, GICP
  backend integration, and production loader promotion remain No-Go/pending
- Date: 2026-08-21
- Scope: scan-to-map registration used by `scanmatcher` and loop-candidate
  registration used by `graph_based_slam`
- Compatibility target: C++14 source API, ROS 2 Humble and Jazzy

This ADR records the first replaceable algorithm boundary in the extensible
architecture roadmap. Phase 0 includes the installed, header-only
`lidarslam_plugin_interfaces` package and contract tests. Phase 1 now also
includes the ROS-free `lidarslam_default_plugins::NdtOmpRegistration` adapter,
its direct-pclomp characterization tests, and static integration for the
scanmatcher's legacy `registration_method=NDT` path. The legacy parameter
names remain the public configuration surface and are converted to a typed
map by a ROS-free helper. The shell-side pluginlib loader and the ODR-safe
host/pluginlib hybrid resolver are now implemented as offline opt-in paths.
The typed GICP adapter and host-resident GICP selector are characterization
only; backend integration and production loader promotion remain pending. The
Phase 1 gates below remain authoritative.

A 2026-08-20 MID-360 full-sequence replay processed all 2,772 scans three
times through both the pre-migration baseline at `0c08b58f` and the integrated
adapter. Every baseline and adapter run produced the same trajectory MD5
(`9f318ac2a91e45f49d415e33b077b892`), submap-summary MD5
(`99dd1243b5e7ed4ede625969c699cef8`), 2,772 poses, and 420 submaps. Adapter
median wall time was 171.37 s versus 169.08 s (+1.35%, within the +5% gate),
and maximum peak RSS was 1,015,500 KiB versus 1,010,240 KiB (+0.52%, within
the +10% gate). The bag duration is 277.17 s, giving median processing RTF
0.618 versus 0.610. This dataset has no paired ground truth, so it does not
close the APE or map-quality gates.

The gate also exposed a build-sensitive constraint: placing pclomp's template
implementation in a separate translation unit changed multi-thread floating-
point accumulation enough to diverge after roughly 230 scans. Static linkage
did not repair it. The built-in compatibility path therefore includes the
adapter implementation in the legacy scanmatcher translation unit; the
separate shared library remains available for external-loader characterization
but is not linked into the production shell. An external pluginlib path must
pass the same replay gate independently and must not inherit this result by
claim.

The HILTI exp04 frontend precision and map-artifact gate was run on 2026-08-20 with the same
`0c08b58f` baseline and adapter working tree, the same
`lidarslam_competitive_v2.yaml`, and the local 125.814128037 s bag (1,258
LiDAR scans, `/hesai/pandar`, `/alphasense/imu`). Three sequential runs per
side produced byte-identical `trajectory_frontend.tum` (MD5
`59f87bc57455e69b23ce05c07e47b3b2`) and `submaps_frontend.csv` (MD5
`16743fe8d14fa35a623e921d9da37930`), with 1,258 poses and 10 frontend
submaps. Under the historical `ape_from_tum.py --interpolate
--max-time-diff 3.0` convention, all runs scored 7/7 sparse GT points and
APE translation RMSE `10.439764002361335 m` with maximum time gap
`0.10001611709594727 s` on both sides. Sequential `/usr/bin/time -v`
measurements were baseline 48.95 s median / 112,400 KiB maximum RSS and
adapter 48.49 s median / 112,508 KiB maximum RSS (-0.94% wall, +0.10% RSS;
processing RTF 0.3891 vs 0.3854). This is a scanmatcher-only gate and is not
the README RKO-LIO/GLIM comparison (0.0565 m). With the explicit `--save-map`
evidence option, both sides also emitted byte-identical world-frame `map.pcd`
(MD5 `16454b536af73b5486af2460c0920507`, SHA-256
`bbf742a1d223c0d21b9a85be705282bde71e73cae0e941f20a85049b472550fd`). The
no-profile geometric map-quality report was byte-identical on both sides
(MD5 `4825eec68fb4442de70cc39d42b0b53a`) with mean entropy -1.858672907,
thickness mean/p95 0.044694491/0.105756330 m, and planar coverage 0.369340696.
This closes the adapter map-artifact non-regression gate. The optional
`indoor_construction` absolute profile retains the same baseline violation
(`mme_valid_fraction=0.788852316 < 0.90`) on both sides; the colored HILTI
profile is not applicable to this uncolored frontend PCD. Trajectory/submap
hashes are therefore supplemented by the real PCD and geometric report, not
used as a map-quality proxy. The full command receipt and raw `/tmp` output
locations are recorded in
[`registration-plugin-hilti-exp04-gate-2026-08.md`](registration-plugin-hilti-exp04-gate-2026-08.md).

## Phase 2 runtime-slot slice (2026-08-21)

The scanmatcher plugin state is now one typed runtime slot rather than
parallel NDT and GICP fields:

```text
registration_plugin_session_       optional shell/session + loader lifetime
registration_plugin_                configured RegistrationPlugin object
registration_alignment_result_     result for the current scan only
cached capabilities / target policy / correspondence metric
```

The session is stored before the plugin pointer and is retained for the whole
component lifetime. On an accepted re-injection, the new plugin is held in a
local shared pointer, then the old plugin is released before the old session;
only after that is the new session followed by the new plugin stored. This
prevents a ClassLoader from unloading an external library while its old object
is being destroyed. A rejected injection leaves the existing slot untouched.

The normal production defaults are unchanged. `registration_method=NDT`
constructs `NdtOmpRegistration` in the legacy `scanmatcher_component.cpp`
translation unit and exposes that same object through the typed slot. Normal
`GICP`, `FAST_*`, and `SMALL_*` methods continue to use the existing
`registration_` PCL path. The slot replacement is used by the deferred,
offline opt-in injection path; it does not enable pluginlib or change the live
default constructor.

Hot-path decisions use only the cached capability contract, never the selected
method name or plugin class ID:

| Contract field | Runtime decision |
| --- | --- |
| `TargetPolicy::kRequiresRawTarget` | pass the full targeted cloud on map refresh/recovery |
| `TargetPolicy::kAcceptHostPrepared` | voxelize refresh/recovery targets with the existing `vg_size_for_input` policy; the initial transformed map is already prepared |
| `Capability::kRotationPrior` / `kTranslationPrior` | add the corresponding typed prior only when the capability is present |
| `CorrespondenceMetric::kMeanDistance` | use the returned mean correspondence diagnostic |
| `CorrespondenceMetric::kSquareRootFitnessProxy` | use `sqrt(fitness_score)` |
| `Capability::kMaximumCorrespondenceDistance` | permit the host adaptive-distance request |
| `Capability::kAlignedSource` | require and consume the aligned cloud, then clear the result-held pointer after copying it to the scan output |

The selected metric is validated on every successful plugin alignment, even
when adaptive correspondence is disabled. This keeps pose diagnostics and a
later adaptive-policy enablement deterministic and prevents a plugin that
advertises a metric but returns an incomplete result from entering pose
acceptance. Failure, target, and mismatch paths remain hard failures; there is
no implicit fallback to `registration_` after a plugin has been selected.

The ROS-free policy helpers are covered by target-policy, metric-validity, and
metric-failure tests. The injection suite also covers NDT/GICP selector
mismatch and replacing a pluginlib-backed session with a same-TU host session.
The scanmatcher build and all 13 package CTest targets pass in the normal
dependency-absent build; the optional Small selectors are covered by the
fail-closed tests described below.

As a short MID-360 compatibility receipt (100 scans, two runs per side,
`/livox/lidar` and `/livox/imu`), legacy same-TU and host-resolver paths were
byte-identical:

| Method | Trajectory MD5 | Submap MD5 | Poses / submaps |
| --- | --- | --- | --- |
| NDT | `9c22c7bf0fbf7fe5c42815e82be0a4c5` | `2465fb002710a3fac094c5c2fd9e7a74` | 100 / 12 |
| GICP | `ed2f59fb958b8a8c1c9fc0a7f5bdc00c` | `ca57c937f7fdc4b1a997be23972d690e` | 100 / 1 |

Each side was deterministic across its two runs and `cmp`-equal to the other
side. The GICP run is a dispatch/serialization compatibility smoke only: the
existing profile reports non-convergence/rejection for the post-map scans, so
it is not an accuracy result or a production-promotion gate. FAST_GICP,
backend injection, and live default promotion remain pending; the conditional
SMALL adapter is covered by the optional-dependency gate below but has no
full-replay promotion evidence.

## FAST_GICP / FAST_VGICP optional-dependency gate (2026-08-21)

The current Jazzy environment does not provide `fast_gicp`; CMake reports
`fast_gicp_FOUND=FALSE`. The existing scanmatcher CMake design keeps this
dependency `QUIET`, compiles the legacy FAST branches only under
`HAS_FAST_GICP`, and leaves the package manifest dependency optional. No
unverified FAST adapter DSO, plugin XML class, or host same-TU factory is
installed in this environment, so the resolver cannot advertise a class that
does not exist.

The pre-migration FAST behavior is nevertheless fixed here for the future
adapter contract:

| Method | Legacy construction | Registration parameters | Target / adaptive behavior |
| --- | --- | --- | --- |
| `FAST_GICP` | `fast_gicp::FastGICP<PointXYZI, PointXYZI>` | `gicp_corr_dist_threshold`; hard-coded transformation epsilon `1e-6`; `ndt_max_iterations`; `ndt_num_threads` when positive | host-prepared target; `vg_size_for_input` voxelization on map refresh/recovery; `sqrt(getFitnessScore())` metric; reset max distance to `DBL_MAX` after adaptive alignment |
| `FAST_VGICP` | `fast_gicp::FastVGICP<PointXYZI, PointXYZI>` | same as FAST_GICP plus `ndt_resolution` as voxel resolution | same target and adaptive policy |

Both methods use the existing generic `registration_` path: the initial
transformed map target is passed after the map voxel stage, refreshed/recovery
targets are voxelized with `vg_size_for_input`, and each source is filtered by
the existing input voxel stage. They do not populate the unified
`registration_plugin_*` slot, and no class-ID branch is added to its hot path.

Selection is now explicit and fail-closed. The ROS-free availability helper
reports both FAST selectors as unavailable when the optional package is
missing, and constructing a component with either selector exits before sensor
processing with a diagnostic naming `fast_gicp` and stating that no fallback is
allowed. Unit coverage checks the availability state and constructor death
tests cover both `FAST_GICP` and `FAST_VGICP`; NDT/GICP are never selected as a
substitute. The current gate is therefore **No-Go** for adapter/resolver
promotion, not a claim that FAST is unsupported in a dependency-enabled build.

The **Go** criteria for a dependency-enabled follow-up are: add the two typed
adapters only inside the `fast_gicp_FOUND` CMake branch; expose capabilities as
host-prepared target plus square-root-fitness metric; construct the host
compatibility factory in the scanmatcher translation unit; register external
classes only with a plugin manifest; and pass direct-fast-gicp fixture equality
(transform, fitness, aligned cloud, convergence) plus the MID-360/HILTI
replay gates before enabling any selector or changing the production default.
Those adapter/factory/fixture results are not available in this dependency-
absent run and remain explicitly unverified.

## SMALL_GICP / SMALL_VGICP optional-dependency gate (2026-08-21)

The Jazzy host used for the normal workspace does not install `small_gicp`.
`find_package(small_gicp QUIET)` is therefore false in the ordinary build:
`HAS_SMALL_GICP` is not defined, the default-plugin shared object has no
`small_gicp` dependency, and the installed manifest advertises only NDT and
GICP.  The scanmatcher availability helper reports both `SMALL_GICP` and
`SMALL_VGICP` as unavailable.  Selecting either method exits at construction
with a diagnostic naming the missing optional dependency; neither selector
falls back to NDT, GICP, or another method.  This absence path is covered by
the ROS-free selector test and the optional loader discovery test (the two
Small IDs must be absent together).

The dependency was evaluated without changing the system installation.  A
read-only extraction of the apt candidate
`ros-jazzy-small-gicp-vendor` (`2.1.0-1noble.20260309.122135`) under `/tmp`
provided the headers, `libsmall_gicp.so`, and the
`small_gicp::small_gicp` CMake target.  With that prefix prepended to
`CMAKE_PREFIX_PATH`, an isolated workspace configured `small_gicp_FOUND=TRUE`
and built the optional adapter path.  This proves that the optional branch is
buildable when the vendor package is supplied; it does not turn the package
into a required dependency.

The dependency-enabled adapter preserves the current scanmatcher
construction and target semantics:

| Selector | `RegistrationPCL` type | Typed settings | Target and result policy |
| --- | --- | --- | --- |
| `SMALL_GICP` | `PointXYZI, PointXYZI`, registration type `GICP` | `gicp_corr_dist_threshold` → `maximum_correspondence_distance`; fixed epsilon `1e-6`; `ndt_max_iterations`; positive `ndt_num_threads` | host-prepared target; `sqrt(fitness)` correspondence metric |
| `SMALL_VGICP` | same type, registration type `VGICP` | the same settings plus `ndt_resolution` → `voxel_resolution` | host-prepared target; `sqrt(fitness)` correspondence metric |

The host keeps the existing `vg_size_for_input` source and target
preprocessing for initial-map, refresh, and recovery paths.  Adaptive calls
start with the configured maximum correspondence distance and leave
`DBL_MAX` in the underlying registration object after every call, including
the first call before an EMA value exists.  A per-call override has the same
clear/reset guarantee.  `num_threads=0` retains the library default and never
claims determinism.  The adapter advertises `kDeterministic` only after
configuration with exactly one thread; the direct fixture repeats transform,
fitness, convergence, and aligned-cloud comparisons byte-for-byte for both
selectors.  The fixture also covers disabled NaN initial guesses, typed
variant-specific configuration rejection, and adaptive first/second-call
reset semantics.

When `small_gicp` is found, `lidarslam_default_plugins` conditionally builds
the two typed adapters, exports a separate manifest containing
`lidarslam_default_plugins/SmallGicpPcl` and
`lidarslam_default_plugins/SmallVGicpPcl`, and exposes same-translation-unit
host factories for the offline resolver.  The normal manifest and DSO contain
neither class when the dependency is absent.  The unified runtime slot remains
capability-driven; no Small class ID is read in the alignment hot path.  At
the injection boundary the selected legacy method is checked against the
canonical class variant (including host aliases): GICP cannot receive VGICP
and VGICP cannot receive GICP.  Either mismatch is an actionable hard failure
before sensor processing, with no fallback.

This slice remains **offline characterization-only for the production
selector**.  The temporary vendor-prefix build and direct legacy comparison
establish wiring and numerical behavior for the supplied library.  A short
HILTI exp04 compatibility smoke (first 100 scans, `ndt_num_threads=1`) also
compared the legacy path with the host-resident resolver; each pair was
`cmp`-equal:

| Selector | Trajectory MD5 | Submap MD5 | Wall legacy / host (s) | Peak RSS legacy / host (KiB) |
| --- | --- | --- | --- | --- |
| `SMALL_GICP` | `829b23ed9e307f431af558546d27d000` | `21ee5902fc9208fab0b4502e32095b60` | 8.07 / 5.71 | 108792 / 107316 |
| `SMALL_VGICP` | `ecc6e402c4d67e23154a67914a2bd3d3` | `0344cb056b47e03ec19a9a71566dd48d` | 4.57 / 4.10 | 112116 / 110928 |

The wall/RSS values are single smoke measurements and are not a performance
claim.  The receipt is preserved outside the repository at
`/tmp/small-gicp-hilti-100.nLtM36`.  The full HILTI exp04 gate is now recorded
in the [SMALL HILTI exp04 receipt](small-gicp-hilti-exp04-gate-2026-08.md):
both selectors passed two-run legacy/host byte equality, historical APE
equality, and three-run geometric map-quality report equality.  This closes
the HILTI compatibility gate; the separate MID-360 compatibility gate below
is also complete.  The ordinary external-DSO replay and its symbol-binding
result are recorded in the [external DSO/ODR receipt](small-gicp-external-dso-odr-gate-2026-08.md).

The full MID-360 compatibility gate is now recorded in the [SMALL MID-360
receipt](small-gicp-mid360-gate-2026-08.md).  With the pinned 277.17 s bag and
`ndt_num_threads=1`, both Small selectors passed two-run legacy/host trajectory,
submap, and PCD byte equality plus three-run geometric map-quality equality.
The dataset has no paired GT, so APE remains unevaluated; this closes the
compatibility gate but not production promotion or the independent external-DSO
ODR gate.  Representative runs also logged 538 GICP and 332 VGICP pose
rejections, which are identical across the legacy/host paths but prevent hash
equality from being interpreted as an absolute tracking-quality pass.

The ordinary combined external DSO was then split so that the optional Small
classes live in `liblidarslam_small_gicp_plugins.so`, with a separate
`registration_plugins_small.xml`; the NDT/GICP DSO no longer links the Small
implementation.  A Small-only `-Wl,-Bsymbolic-functions` build was evaluated
under `/tmp/small-gicp-bsymbolic-build.5FLKtQ`.  For HILTI and MID-360, both
Small selectors passed first-100 and full trajectory/submap/map comparisons
against the same legacy artifacts.  `LD_DEBUG=bindings` showed no request by
the Small DSO for `SmallGicpRegistration::align` or
`small_gicp::RegistrationPCL::computeTransformation` from
`libscanmatcher_component.so`; `nm`/`readelf` confirmed the definitions in
the dedicated DSO.  This closes the **scoped independent Small DSO/ODR gate**
for the pinned Jazzy/vendor/toolchain receipt, but does not promote any live
default or claim absolute accuracy.  Full paths, hashes, timing, RSS, and
the exact binding grep are recorded in the [SMALL external DSO/ODR gate
receipt](small-gicp-external-dso-odr-gate-2026-08.md).  The production
default and README claims remain unchanged.

## Decision summary

`RegistrationPlugin` is a small, ROS-free, clock-free C++14 interface. A host
creates one configured plugin session at startup, validates its metadata and
capabilities, and injects the session into the frontend or backend core. The
processing hot path calls the already-created object; it never performs
pluginlib lookup or reads ROS parameters.

The ROS node and offline runner are shells. They own parameter resolution,
`pluginlib::ClassLoader`, manifest generation, dependency/license checks, and
failure reporting. `BackendCore` and the future ROS-free scan-matching core
know only the interface and typed request/result objects. They must not include
pluginlib, `rclcpp`, plugin XML, or a concrete NDT/GICP header.

The built-in NDT and GICP implementations are migrated first through adapters
that preserve the current PCL behavior. FAST_GICP and FAST_VGICP remain
dependency-gated; SMALL_GICP and SMALL_VGICP have conditional adapters and
host factories, but remain characterization-only until their replay gates
pass. The
separate `small_gicp_odom_node` remains a frontend/odometry path and is not
silently made equivalent to scan-to-map registration by this ADR.

## Current implementation inventory

The following inventory is the Phase 0 source of truth. Line references point
to the current tree and are deliberately concrete so a migration review can
check every branch.

### Live scanmatcher frontend

[`ScanMatcherComponent`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/include/scanmatcher/scanmatcher_component.h)
stores a single
`boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>>` in
`registration_`. The constructor reads flat ROS parameters and constructs the
object in
[`scanmatcher_component.cpp`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L121-L496):

| `registration_method` | Concrete object and current construction | Parameters actually used |
| --- | --- | --- |
| `NDT` | `pclomp::NormalDistributionsTransform<PointXYZI, PointXYZI>`; `DIRECT7`; optional `setNumThreads` when `ndt_num_threads > 0` | `ndt_resolution`, `ndt_transformation_epsilon`, `ndt_max_iterations`, `ndt_step_size`, `ndt_outlier_ratio`, `ndt_num_threads` |
| `GICP` | `pclomp::GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI>` | `gicp_corr_dist_threshold`; transformation epsilon is hard-coded to `1e-8` |
| `FAST_GICP` | `fast_gicp::FastGICP<PointXYZI, PointXYZI>` when `HAS_FAST_GICP` is compiled | `gicp_corr_dist_threshold`, `ndt_transformation_epsilon` is not used; `ndt_max_iterations`, `ndt_num_threads` |
| `FAST_VGICP` | `fast_gicp::FastVGICP<PointXYZI, PointXYZI>` when `HAS_FAST_GICP` is compiled | `gicp_corr_dist_threshold`, `ndt_max_iterations`, `ndt_num_threads`, `ndt_resolution` as voxel resolution |
| `SMALL_GICP` | `small_gicp::RegistrationPCL<PointXYZI, PointXYZI>` with registration type `GICP` when `HAS_SMALL_GICP` is compiled | `gicp_corr_dist_threshold`, `ndt_max_iterations`, `ndt_num_threads`; `ndt_resolution` is not used |
| `SMALL_VGICP` | the same PCL wrapper with type `VGICP` | `gicp_corr_dist_threshold`, `ndt_max_iterations`, `ndt_num_threads`, `ndt_resolution` as voxel resolution |

The method string is case-sensitive. An unknown method, or a method whose
optional dependency was not found at build time, logs an error and calls
`exit(1)` during construction. There is no fallback to NDT.

The registration object is used by these live call sites:

1. `initializeMap()` sets the first transformed, map-voxelized cloud as the
   target at
   [`scanmatcher_component.cpp:858`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L858).
2. After an asynchronous map update, the target is refreshed at
   [`:905-916`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L905). NDT gets
   the full targeted cloud; every other method gets a second
   `vg_size_for_input` voxel-filtered target. This is a semantic difference,
   not just an optimization.
3. Each accepted input scan is voxel-filtered with `vg_size_for_input`, set as
   the source at
   [`:928-938`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L928), and
   aligned at [`:1063`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L1063)
   with the IMU/constant-velocity/odometry-derived initial transform.
4. `getFinalTransformation()`, `hasConverged()`, and `getFitnessScore()` feed
   pose acceptance and map-update policy at
   [`:1103-1174`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L1103).
5. Recovery target refresh repeats the NDT-versus-other target branch at
   [`:1439-1450`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L1439).

The remaining NDT-only behavior is at
[`scanmatcher_component.cpp:1010-1099`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scanmatcher_component.cpp#L1010):

- `imu_ndt_prior_enable` asks the concrete NDT object to receive an IMU-derived
  rotation prior (`setRotationPrior`) and clears it after alignment;
- `imu_z_prior_enable` sets and clears a z-only translation prior through
  `setTranslationPrior`;
- adaptive correspondence computes a distance from the EMA and calls the NDT
  concrete setter, while non-NDT methods use the base PCL setter;
- after alignment NDT reports
  `getLastMeanCorrespondenceDistance()` and resets its maximum distance; GICP,
  FAST, and SMALL use `sqrt(getFitnessScore())` as the existing proxy and reset
  to `numeric_limits<double>::max()`.

The host-side parameters must not be confused with plugin parameters. The
following are frontend policy or preprocessing and stay outside the plugin
configuration: `vg_size_for_input`, `vg_size_for_map`, `min_points_for_scan`,
range filtering, `trans_for_mapupdate`, target selection
(`num_targeted_cloud`, spatial map, voxel hash map), IMU deskew/prediction,
odometry priors, pose acceptance/recovery gates, and asynchronous map update.
The following are current registration knobs and are migrated into a
plugin-namespaced configuration: `ndt_resolution`, `ndt_step_size`,
`ndt_num_threads`, `ndt_transformation_epsilon`, `ndt_max_iterations`,
`ndt_outlier_ratio`, and `gicp_corr_dist_threshold`.

`ndt_step_size` and `ndt_outlier_ratio` are constructor-local values; they are
printed but not stored as component members. The `ndt_*` spelling is retained
only by the legacy adapter and is not used in the new public API.

### Live graph backend and `BackendCore`

The graph component has another independent `registration_` pointer of the
same PCL base type. It reads `registration_method`, `ndt_resolution`, and
`ndt_num_threads` at
[`graph_based_slam_component.cpp:69-81`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/graph_based_slam_component.cpp#L69)
and constructs it through
[`registration_factory.hpp:54-80`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/include/graph_based_slam/registration_factory.hpp#L54):

| Method | Current factory settings |
| --- | --- |
| `NDT` | `pclomp::NormalDistributionsTransform<PointXYZI, PointXYZI>`, 100 iterations, `ndt_resolution`, epsilon `0.01`, `DIRECT7`, optional `ndt_num_threads` |
| `GICP` | `pclomp::GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI>`, maximum correspondence `30`, 100 iterations, transformation epsilon `1e-8`, Euclidean fitness epsilon `1e-6`, RANSAC iterations `0`; `ndt_num_threads` is ignored |

The backend factory has no FAST or SMALL branch. Its `ndt_resolution` and
`ndt_num_threads` names are historical; the factory does not read frontend
parameters such as `ndt_step_size`, `ndt_outlier_ratio`, or
`gicp_corr_dist_threshold`. Backend defaults therefore must remain a separate
role profile even when both roles use the same plugin class.

`BackendCore::searchLoopForSubmap()` is intentionally already close to the
desired injection boundary. Its current PCL contract is visible in
[`backend_core.hpp:441-446`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/include/graph_based_slam/backend_core.hpp#L441)
the caller supplies the registration object, voxel grid, raw cloud provider,
and 3D-BBS verifier. Within
[`backend_core.hpp:526-826`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/include/graph_based_slam/backend_core.hpp#L526),
the core:

- builds filtered source and target aggregates;
- chooses a ScanContext-specific cloud or the normal cloud;
- optionally replaces the initial guess with 3D-BBS;
- calls `align(output, initial_guess)` for BBS/ScanContext paths and
  `align(output)` otherwise;
- rejects a candidate when `hasConverged()` is false;
- consumes fitness, final transform, and the aligned output cloud for overlap
  metrics and translation/rotation gates;
- creates a loop proposal only after core gates pass.

The live call site at
[`graph_based_slam_component.cpp:1697-1704`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/graph_based_slam_component.cpp#L1697)
passes the injected registration into this function. Registration is never
constructed by `BackendCore`.

The backend's `voxel_leaf_size` is host-side target/source preprocessing and is
not a registration resolution. The `loop_*` score, correction, overlap,
candidate ordering, and edge de-duplication settings are core safety policy,
not plugin settings.

### Offline and auxiliary paths

| Path | Registration ownership and call site | Phase 1 decision |
| --- | --- | --- |
| `scan_matcher_offline_runner` | Creates the real `ScanMatcherComponent` and feeds it one bag message at a time through intra-process ROS pub/sub. It has no separate registration construction. `async_map_update=false` and fixed thread count are required for the deterministic contract; see [`scan_matcher_offline_runner.cpp`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/scan_matcher_offline_runner.cpp). | The runner shell resolves and loads the same plugin as live scanmatcher, then injects it into the component/core. It must not silently let the component load a different plugin. |
| `graph_slam_offline_runner` | Reads `registration_method`, `ndt_resolution`, and `ndt_num_threads` and calls the same `makeLoopRegistration()` factory at [`graph_slam_offline_runner.cpp:307-315`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/graph_slam_offline_runner.cpp#L307) and [`:888-895`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/graph_slam_offline_runner.cpp#L888). It then feeds the object to `BackendCore` at [`:959-960`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/graph_slam_offline_runner.cpp#L959). | The runner owns `pluginlib::ClassLoader`, manifest, and plugin lifetime. The offline and live backend must use the same resolved config and plugin class. |
| `small_gicp_odom_node` | A separate stateful scan-to-incremental-voxel-map node directly constructs `small_gicp::Registration<ICPFactor,...>` or `Registration<GICPFactor,...>` at [`small_gicp_odom_node.cpp:126-140`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/scanmatcher/src/small_gicp_odom_node.cpp#L126). Its `use_gicp=false` means ICP, not the legacy `registration_method`. | Keep as a separate frontend/odometry implementation in Phase 1. A future `FrontendPlugin` may wrap it; do not claim that it implements this scan-to-map `RegistrationPlugin`. |
| `map_ndt_residual_report` | Analysis-only executable constructs `pclomp::NormalDistributionsTransform<PointXYZ, PointXYZ>` at [`map_ndt_residual_report_main.cpp:295-345`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/src/map_ndt_residual_report_main.cpp#L295). It fixes one thread and `DIRECT7`, takes CLI resolution/source voxel/max-correspondence/max-iterations, and uses NDT for residual reporting and optional pose regularization. | It is not a live estimator or loop verifier. Keep its analysis semantics in Phase 1; a generic replay/diagnostic adapter can be considered after the registration contract is stable. |
| `ndt_localization_target` | Builds tangent-sampled target geometry only; despite its name it does not construct or run a registration object. | Out of this API's scope. |
| tests | [`test_registration_determinism.cpp`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/test/test_registration_determinism.cpp) constructs NDT directly; [`test_backend_core.cpp:385-405`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/test/test_backend_core.cpp#L385) uses a direct NDT characterization harness. | Keep these as the pre-migration oracle, then add plugin-adapter equivalents before deleting direct construction. |

The offline frontend's use of the live component is useful for behavioral
coverage but does not satisfy the loader ownership rule by itself. Phase 1
must make the plugin object an explicit dependency of the component's pure
runtime/core so a runner cannot accidentally exercise a second construction
path.

### Dependencies and build constraints

- `ndt_omp_ros2` is required by both `scanmatcher` and `graph_based_slam`.
  The implementation is template/header based; the current tests include the
  `*_impl.hpp` headers directly.
- `PCL` and Eigen are required by the existing registration and core code.
- `fast_gicp` and `small_gicp` are CMake `QUIET` optional dependencies in
  `scanmatcher/CMakeLists.txt`; their package.xml dependency entries are
  commented out. Missing packages compile out their method branches.
- `OpenMP` is optional at CMake level. NDT/FAST/SMALL thread behavior must be
  recorded in capabilities and the run manifest, not inferred from the method
  name.
- `graph_based_slam` requires g2o for graph optimization and does not currently
  depend on FAST_GICP or small_gicp.
- `scanmatcher` defaults to C++17 because of small_gicp; `graph_based_slam`
  defaults to C++14. The shared interface is C++14 and may not use C++17-only
  language/library types. No C++ binary ABI promise is made across Humble,
  Jazzy, Ubuntu, compilers, PCL, or Eigen versions; plugins are built for the
  host distribution.
- `pluginlib` is a new Phase 1 shell dependency. It belongs on the live node
  and offline-runner targets, not on `BackendCore` or the interface-only
  package.

## API boundary

The dependency direction is:

```text
ROS live node / offline runner
  ├─ resolve canonical or legacy config
  ├─ pluginlib ClassLoader + metadata/capability checks
  ├─ manifest + configuration hash
  └─ inject one configured RegistrationPlugin session
                │
                ▼
ROS-free frontend/backend core ── RegistrationPlugin reference
                │
                └─ typed point cloud, request, result, failure only
```

The installed source of truth is
[`lidarslam_plugin_interfaces/registration.hpp`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam_plugin_interfaces/include/lidarslam_plugin_interfaces/registration.hpp).
It must not include `rclcpp`, ROS messages, pluginlib, PCL registration
classes, filesystem APIs, or wall-clock calls.

API v1 uses `Eigen::Matrix4f` and
`pcl::PointCloud<pcl::PointXYZI>::ConstPtr`. This deliberately accepts a PCL
cloud dependency to avoid a conversion in the registration hot path while
still hiding the `pcl::Registration` implementation hierarchy. Source
compatibility is promised within the API major; cross-distribution PCL C++ ABI
compatibility is not. A non-PCL `PointCloudView` is reserved for a measured API
v2 proposal rather than being simulated by hidden copies in v1.

## C++14 API design vocabulary

The following block records the Phase 1 design vocabulary and ownership rules;
it is informative pseudocode, not a second header definition. Exact namespaces,
signatures, capability bits, and defaults are normative only in the installed
header linked above. The header must compile with `-std=c++14`.

```cpp
namespace lidarslam { namespace plugins { namespace registration {

struct PluginApiVersion {
  std::uint16_t major;
  std::uint16_t minor;
};

static const PluginApiVersion kRegistrationApiVersion = {1u, 0u};

enum class FailureCode {
  kNone,
  kInvalidConfiguration,
  kInvalidRequest,
  kMissingPointField,
  kEmptySource,
  kEmptyTarget,
  kInsufficientCorrespondences,
  kNotConverged,
  kNumericalFailure,
  kTimeout,
  kPluginException,
  kUnsupportedCapability,
  kInternalError
};

struct Failure {
  FailureCode code;
  std::string message;
  bool recoverable;
};

struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
};

struct PointCloudView {
  const PointXYZI * data;
  std::size_t size;
  bool has_intensity;
};

struct RotationPrior {
  bool enabled;
  Eigen::Quaternionf orientation;
  double weight;
  bool constrain_roll_pitch_only;
};

struct TranslationPrior {
  bool enabled;
  Eigen::Vector3f translation;
  Eigen::Vector3f axis_weights;
};

struct CorrespondenceRequest {
  bool max_distance_enabled;
  double max_distance_m;
};

struct RegistrationRequest {
  PointCloudView source;
  Eigen::Matrix4f initial_guess;
  bool initial_guess_enabled;
  RotationPrior rotation_prior;
  TranslationPrior translation_prior;
  CorrespondenceRequest correspondence;
  std::uint64_t sequence_id;
  double stamp_sec;
};

struct RegistrationDiagnostics {
  std::uint32_t iteration_count;
  double elapsed_msec;
  bool mean_correspondence_distance_valid;
  double mean_correspondence_distance_m;
  std::vector<std::pair<std::string, std::string>> fields;
};

struct RegistrationResult {
  bool valid;                 // finite, contract-valid output
  bool converged;             // algorithm convergence flag, kept separate
  Eigen::Matrix4f transform;
  double fitness_score;
  std::shared_ptr<const std::vector<PointXYZI>> aligned_source;
  bool covariance_valid;
  std::array<double, 36> covariance;
  RegistrationDiagnostics diagnostics;
  Failure failure;
};

enum class TargetPolicy {
  kAcceptHostPrepared,
  kRequiresRawTarget,
  kPluginPreprocessesTarget
};

enum class CorrespondenceMetric {
  kMeanDistance,
  kSqrtFitnessProxy,
  kUnavailable
};

enum class ThreadModel {
  kSerial,
  kSerializedOwner,
  kReentrant
};

struct RegistrationCapabilities {
  bool supports_initial_guess;
  bool supports_rotation_prior;
  bool supports_translation_prior;
  bool supports_max_correspondence_distance;
  bool returns_aligned_source;
  bool returns_covariance;
  bool deterministic_single_thread;
  bool deterministic_fixed_thread_count;
  bool supports_reset;
  bool requires_intensity;
  TargetPolicy target_policy;
  CorrespondenceMetric correspondence_metric;
  ThreadModel thread_model;
};

struct PluginMetadata {
  std::string id;
  std::string implementation_version;
  std::string license_spdx;
  std::string upstream;
  std::string build_id;
  PluginApiVersion api_version;
};

struct Parameter {
  std::string key;
  std::string canonical_value;
};

struct PluginConfiguration {
  std::vector<Parameter> parameters;  // sorted by key before configure()
};

class RegistrationPlugin {
public:
  virtual ~RegistrationPlugin() {}
  virtual PluginApiVersion apiVersion() const = 0;
  virtual PluginMetadata metadata() const = 0;
  virtual RegistrationCapabilities capabilities() const = 0;
  virtual Failure configure(const PluginConfiguration & config) = 0;
  virtual Failure setTarget(const PointCloudView & target) = 0;
  virtual RegistrationResult align(const RegistrationRequest & request) = 0;
  virtual Failure reset() = 0;
};

}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam
```

Additional rules for this API are normative:

1. `PointCloudView` is borrowed for one call. A plugin must copy it if it
   retains data after `align()` returns. A target remains owned by the plugin
   until the next successful `setTarget()`, `reset()`, or destruction.
2. `initial_guess_enabled=false` means the host requests the implementation's
   historical no-explicit-guess overload. The built-in PCL adapter must retain
   that distinction; it must not unconditionally replace it with a host-side
   previous pose.
3. A request with an enabled prior or adaptive maximum distance is legal only
   when the corresponding capability is declared. Capability negotiation
   rejects it before subscriptions start; it is never silently ignored.
4. `valid` means the transform and required diagnostics are finite and the
   plugin completed its contract. `converged` remains a separate algorithm
   result because the current frontend has a configurable non-convergence
   policy while the backend always rejects a non-converged loop candidate.
5. `aligned_source` is required for the loop host because the current overlap
   gate uses the aligned cloud. A frontend-only host may accept a plugin with
   `returns_aligned_source=false` if it never requests that diagnostic.
6. The result's covariance is optional and does not become a graph constraint
   automatically. A future covariance-aware backend feature must opt in and
   version its own semantics.
7. `configure()` is called exactly once before `setTarget()`; target changes
   and `align()` calls are serialized by the owner unless the plugin declares
   `kReentrant`. `reset()` clears target, priors, correspondence limits, and
   per-sequence state.
8. No API function reads ROS parameters or wall time. `stamp_sec` is data from
   the input stream and is not permission to query a clock.
9. A C++ exception crossing the interface is a contract violation. The shell
   catches it at the plugin boundary, records `kPluginException`, marks the
   session failed, and does not continue with partially mutated state.

### Capability meanings for current implementations

The built-in adapters declare the following initial capability matrix. A
capability is about a semantic guarantee, not merely whether a concrete PCL
setter happens to exist.

| Adapter | Guess | Rotation / translation prior | Max distance / metric | Target policy | Determinism |
| --- | --- | --- | --- | --- | --- |
| `NdtOmp` | yes | yes / yes | yes / mean distance | raw target | single-thread bitwise; fixed multi-thread characterized, not promised across thread counts |
| `GicpOmp` | yes | no / no | yes / `sqrt(fitness)` proxy | host-prepared | no deterministic bit claimed; serialized owner |
| `FastGicp` | yes | no / no | yes / `sqrt(fitness)` proxy | host-prepared | fixed-thread result must be characterized before enabling deterministic mode |
| `FastVGicp` | yes | no / no | yes / `sqrt(fitness)` proxy | host-prepared | fixed-thread result must be characterized before enabling deterministic mode |
| `SmallGicpPcl` | yes | no / no | yes / `sqrt(fitness)` proxy | host-prepared | fixed-thread result must be characterized before enabling deterministic mode |
| `SmallVGicpPcl` | yes | no / no | yes / `sqrt(fitness)` proxy | host-prepared | fixed-thread result must be characterized before enabling deterministic mode |

The capability matrix must be checked against the actual optional library
version at build time. An adapter may advertise fewer capabilities than the
table while a library is being characterized; it must never advertise an
untested guarantee.

## Configuration and legacy compatibility

### Canonical shape

The logical configuration for each host node is:

```yaml
registration:
  plugin: lidarslam_default_plugins/NdtOmp
  parameters:
    resolution: 1.0
    maximum_iterations: 35
    num_threads: 1
```

ROS 2 parameter files may encode the same values as the node-local flattened
keys `registration.plugin` and `registration.parameters.<key>` if nested
parameter maps are unavailable in the target distribution. The manifest
always records the logical canonical form, never the spelling used in YAML.
The frontend and loop backend are separate ROS nodes and therefore have
separate node-local registration configurations; a YAML file containing both
`scan_matcher` and `graph_based_slam` sections does not create a cross-role
collision.

The canonical `parameters` map is plugin-namespaced. Host policy stays beside
it in a distinct namespace, for example `registration.adaptive` and
`registration.priors`; plugin parameters cannot change pose acceptance, loop
gates, candidate ordering, map update scheduling, or graph safety policy.

### Legacy method mapping

For two minor application releases, the shell accepts the current flat
`registration_method` spelling and maps it to the built-in class ID:

| Legacy value | Canonical built-in ID | Availability |
| --- | --- | --- |
| `NDT` | `lidarslam_default_plugins/NdtOmp` | required |
| `GICP` | `lidarslam_default_plugins/GicpOmp` (or host `lidarslam_builtin/GicpOmp`) | required |
| `FAST_GICP` | `lidarslam_default_plugins/FastGicp` | only when `fast_gicp` is installed |
| `FAST_VGICP` | `lidarslam_default_plugins/FastVGicp` | only when `fast_gicp` is installed |
| `SMALL_GICP` | `lidarslam_default_plugins/SmallGicpPcl` | only when `small_gicp` is installed |
| `SMALL_VGICP` | `lidarslam_default_plugins/SmallVGicpPcl` | only when `small_gicp` is installed |

The backend role exposes only the first two mappings until optional backend
adapters are deliberately added. A missing optional class is a startup error
that names the absent package and the available class IDs; it never falls back
to NDT.

Legacy parameter mapping is role-specific:

| Legacy key | Frontend canonical key | Loop-backend canonical key | Notes |
| --- | --- | --- | --- |
| `ndt_resolution` | `registration.parameters.resolution` | same | In FAST/SMALL VGICP it retains its current voxel-resolution meaning. |
| `ndt_num_threads` | `registration.parameters.num_threads` | same | Backend GICP historically ignores it; the new GICP adapter must preserve that default unless explicitly configured. |
| `ndt_step_size` | `registration.parameters.step_size` | not declared | NDT-only frontend setting. |
| `ndt_transformation_epsilon` | `registration.parameters.transformation_epsilon` | backend factory default `0.01` | The old backend had no parameter for this value. |
| `ndt_max_iterations` | `registration.parameters.maximum_iterations` | backend factory default `100` | Frontend GICP historically did not consume this key; FAST/SMALL did. |
| `ndt_outlier_ratio` | `registration.parameters.outlier_ratio` | not declared | NDT-only frontend setting. |
| `gicp_corr_dist_threshold` | `registration.parameters.maximum_correspondence_distance` | not declared; legacy GICP factory value is `30` | The backend must not accidentally inherit the frontend value `5.0`. |
| `adaptive_correspondence_threshold` / `adaptive_corr_dist_multiplier` | `registration.parameters.adaptive_correspondence_threshold` plus `registration.adaptive.*` host policy | not used by loop search | The adapter resets GICP distance to `DBL_MAX` after every adaptive call, including the first call before the EMA exists. The host owns the EMA policy. |
| `imu_ndt_prior_enable`, `imu_ndt_prior_weight`, `imu_ndt_prior_roll_pitch_only`, `imu_z_prior_enable`, `imu_z_prior_weight` | `registration.priors.*` host policy | not used | Canonical requests require declared prior capabilities. |

### Collision and precedence rules

There is no implicit precedence between new and old settings. The following
rules are enforced by the shell before loading a plugin:

1. If only `registration.plugin` is present, it is authoritative and only
   namespaced plugin parameters are used.
2. If only legacy `registration_method` is present, it is mapped to the
   built-in ID and a single deprecation warning is emitted. Legacy parameter
   aliases are translated into the canonical map.
3. If both plugin selectors are present, the shell resolves the legacy value
   first. It accepts the configuration only when the resulting class ID is
   exactly equal to `registration.plugin`. A mismatch is a hard startup error
   showing both values and their resolved IDs. Neither value wins silently.
4. If both a canonical parameter and its legacy alias are present, their
   parsed, type-normalized values must be equal. A mismatch is a hard startup
   error naming the key pair. Equal values are accepted with one deprecation
   warning; the canonical value is recorded in the manifest.
5. A legacy key that has no meaning for the selected plugin is handled as
   follows: in legacy-only mode the historical behavior is retained for the
   two-release window and a warning identifies the ignored key; in canonical
   mode it is an unknown plugin parameter and fails startup. This prevents a
   typo in a new plugin configuration from being silently ignored.
6. A canonical prior or adaptive request that the selected plugin cannot
   satisfy is always a hard startup error, including when a legacy alias
   enabled it. The only historical non-NDT no-op is therefore not carried into
   canonical mode.
7. The frontend's and backend's node-local settings are independent. If a
   single YAML file gives `scan_matcher.registration.plugin` and
   `graph_based_slam.registration.plugin` different values, both are valid and
   intentional. A launcher override applies to the addressed node only. The
   host must print the resolved role (`frontend_scan_to_map` or
   `backend_loop`) in its startup diagnostics and manifest.
8. If the canonical and legacy source selectors are present through different
   ROS parameter override layers, presence is determined before defaults are
   applied. A default `NDT` is not treated as an explicit legacy selector and
   therefore cannot create a false collision.

The compatibility window ends only after the second minor release that
supports the adapter. The removal is a major application/configuration change
and must be announced in the changelog; an unknown `registration_method` after
the window remains a hard error.

## Loader, metadata, and run manifest

The shell uses a standard pluginlib class export. A plugin package provides a
description equivalent to:

```xml
<library path="lidarslam_default_plugins">
  <class
    name="lidarslam_default_plugins/NdtOmp"
    type="lidarslam_default_plugins::NdtOmp"
    base_class_type="lidarslam::plugins::registration::RegistrationPlugin">
    <description>pclomp NDT registration adapter</description>
  </class>
</library>
```

The exact library path follows the package's installed target name. A loader
must keep the `ClassLoader` alive for the entire plugin session; destroying the
loader while retaining a plugin object is invalid.

The shell sequence is:

1. Resolve canonical/legacy settings and role-specific defaults.
2. Construct `pluginlib::ClassLoader<RegistrationPlugin>` and resolve the
   class ID. No lookup is permitted after this step.
3. Instantiate the class, query `metadata()` and `capabilities()`, and validate
   `metadata().api_version`, license policy, required point
   fields, target policy, priors, adaptive mode, thread model, and aligned
   output requirements.
4. Sort plugin parameters by key, call `configure()`, and call `setTarget()`
   only after configuration succeeds.
5. Store the resolved metadata/configuration in the run manifest before the
   first source cloud is processed.
6. Inject the configured object into the ROS-free core. The core receives no
   class loader and cannot discover or replace it.

Every live and offline run records a `registration` manifest object with at
least:

```yaml
registration:
  schema_version: 1
  role: frontend_scan_to_map  # or backend_loop
  plugin_id: lidarslam_default_plugins/NdtOmp
  class_id: lidarslam_default_plugins/NdtOmp
  implementation_version: 1.0.0
  api_version: {major: 1, minor: 0}
  plugin_library: liblidarslam_default_plugins.so
  plugin_xml_sha256: "..."
  license_spdx: BSD-2-Clause
  upstream: "..."
  build_id: "..."
  capabilities:
    target_policy: raw_target
    supports_initial_guess: true
    supports_rotation_prior: true
    supports_translation_prior: true
    correspondence_metric: mean_distance
    thread_model: serialized_owner
    deterministic_mode: single_thread_bitwise
  resolved_parameters:
    resolution: 2.0
    maximum_iterations: 35
    num_threads: 1
  configuration_hash: "sha256:..."
  legacy_aliases_used: []
  compatibility_warnings: []
```

The configuration hash is computed over a canonical UTF-8 serialization of the
resolved role, API version, plugin ID, sorted typed parameters, and relevant
host registration policy. YAML order, alias spelling, and warning text do not
change it. The manifest also records optional dependency versions and the
compiler/distribution build identity so a bitwise claim is not detached from
the binary that produced it.

### Phase 2 shell-loader slice receipt (2026-08-20)

The first loader slice is implemented in the shell-only package
`lidarslam_registration_loader`. It owns
`pluginlib::ClassLoader<RegistrationPlugin>` and returns a
`RegistrationPluginSession` whose shared loader is retained until the plugin
object is destroyed. The ROS-free interface package remains free of
`pluginlib`, `rclcpp`, plugin XML, and filesystem APIs; the loader shell uses
C++17 on Jazzy because that distribution's pluginlib/ament-index headers use
`std::filesystem` and `std::optional`, while the installed interface remains
C++14 source-compatible.

`lidarslam_default_plugins` now exports
`registration_plugins.xml` through the `lidarslam_plugin_interfaces` pluginlib
category and registers `lidarslam_default_plugins/NdtOmp` with
`PLUGINLIB_EXPORT_CLASS`. The separate
`lidarslam_fake_registration_plugins` package is an external-style contract
fixture. Its installed XML includes a valid identity plugin, a missing-library
class, and negative API/metadata/capability/configuration cases.

The loader validates the requested class against the installed ament-index
class list, catches malformed manifests/library/constructor failures, checks
exact API major and non-newer minor compatibility, class identity,
implementation version, permissive SPDX license, configured capabilities, and
the plugin's typed `configure()` result. Diagnostics include the requested
class and available classes where applicable; no legacy or NDT fallback is
performed. A configured capability query is repeated after `configure()` so
configuration-dependent claims such as NDT's fixed-thread deterministic bit
are evaluated against the resolved parameters. The session also exposes the
resolved library and plugin-manifest paths for the later run-manifest slice.

Install-space proof (Jazzy, with the workspace install space sourced) is the
`test_registration_plugin_loader` contract test. It discovered both the
external fixture classes and `lidarslam_default_plugins/NdtOmp`, successfully
created/configured the default NDT class, and passed the loader-lifetime,
unknown-class, missing-library, API-major, metadata/license,
capability-mismatch, and invalid-configuration cases. The tested command was:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select \
  lidarslam_plugin_interfaces lidarslam_default_plugins \
  lidarslam_fake_registration_plugins lidarslam_registration_loader
```

This slice intentionally does not change the live default or switch the
production `registration_method=NDT` path away from the same-translation-unit
built-in adapter. The next slice below adds only an offline, explicit
characterization injection; the plugin DSO remains opt-in and promotion is a
separate gate.

### Phase 2 offline injection and DSO characterization receipt (2026-08-20)

`scan_matcher_offline_runner` now accepts two runner-only parameters:

| parameter | default | contract |
| --- | --- | --- |
| `registration_plugin_enable` | `false` | must be explicitly true to select a plugin session |
| `registration_plugin_class` | empty | must be set with the enable flag; `lidarslam_builtin/NdtOmp` selects the host-resident same-TU factory, any non-reserved ID selects pluginlib |

The live `ScanMatcherComponent` does not resolve pluginlib. The offline shell
reads the component's resolved legacy NDT values, converts them with the same
ROS-free `makeNdtParameterMap()` helper, asks
`RegistrationResolver` for the explicit host or external class, validates the
target/capability contract, and calls the explicit
`ScanMatcherComponent::setRegistrationPluginSession()` boundary before adding
the executor or publishing a sensor message. The component retains the
`RegistrationPluginSession` before its plugin pointer, so the loader remains
alive until the plugin is destroyed. Once injected, target refresh and align
use the plugin and do not fall back to the built-in adapter. The offline shell
selects the typed `RegistrationConstruction::kDeferredPluginInjection`
constructor only after choosing the opt-in path; the default
`ScanMatcherComponent(NodeOptions)` constructor always builds the same-TU
adapter, and no ROS parameter can put a live node into a deferred state.

Every successful opt-in run writes a canonical, sorted
`registration_plugin_receipt.yaml` containing backend kind, class ID, API
version, implementation/license, capability bits and policies, resolved
library and manifest paths (empty for host-built-ins), capability
requirements, and typed NDT parameters. The paths are provenance fields, not
trajectory/submap/map hash inputs; receipts from different install roots must
not be compared byte-for-byte without normalizing those absolute paths.
Unknown class, namespace/collision, class/config mismatch, loader failure,
capability mismatch, injection failure, or receipt failure returns nonzero
before sensor processing. The determinism script creates `.complete` only
after a zero exit; an interrupted partial bag is also nonzero and receives no
completion marker.

The install-space smoke/injection tests and the following replay receipts were
run with the same Jazzy workspace and `lidarslam/param/lidarslam.yaml` or
`configs/hilti2022/lidarslam_competitive_v2.yaml` values. The ordinary pluginlib
DSO run is useful as a wiring proof but is not an independent numerical proof:
`LD_DEBUG=bindings` at `/tmp/lddebug-normal.agWweS`
shows the DSO's weak pclomp `setInputTarget` and
`computeTransformation` resolving to `libscanmatcher_component.so`, which
already contains the legacy same-TU instantiation.

| input / path | result | receipt |
| --- | --- | --- |
| HILTI exp04, ordinary pluginlib DSO, 1 full run | 1,258 poses / 10 submaps; trajectory MD5 `59f87bc57455e69b23ce05c07e47b3b2`, submaps MD5 `16743fe8d14fa35a623e921d9da37930`, map MD5 `16454b536af73b5486af2460c0920507`; historical interpolated APE RMSE `10.439764002361335 m`; map-quality 3-run report MD5 `4825eec68fb4442de70cc39d42b0b53a` | `/tmp/hilti-exp04-plugin-1run.Kcurti`, `/tmp/hilti-exp04-quality-plugin.hsd5QW` |
| MID-360, ordinary pluginlib DSO, 1 full run | 2,772 poses / 420 submaps; trajectory MD5 `9f318ac2a91e45f49d415e33b077b892`, submaps MD5 `99dd1243b5e7ed4ede625969c699cef8`, map MD5 `577d3fbca063e493c35a09ed361b454d`; legacy same-TU one-run map artifact is byte-identical; map-quality 3-run report MD5 `717123e45f017e0f2eeab9760fa774b2` on both inputs | `/tmp/mid360-plugin-full-1run.Kd8fWM`, `/tmp/mid360-legacy-map-1run.qWX6m4`, `/tmp/mid360-quality-plugin.VrI8kT`, `/tmp/mid360-quality-legacy.yENRdh` |

The ordinary-DSO equality is therefore recorded as interposition-contaminated
wiring evidence only. For an independent characterization, a workspace-outside
copy of `lidarslam_default_plugins` was linked with
`-Wl,-Bsymbolic` (overlay `/tmp/lidarslam-plugin-bsymbolic.9M8XGx`); `readelf -d` reports
`SYMBOLIC`, preventing the component's weak pclomp definitions from satisfying
the plugin's references. The result is a hard No-Go for production promotion:

| independent DSO input | first divergence against legacy same-TU | observed stop/resource |
| --- | --- | --- |
| MID-360, cap 300 characterization | trajectory line 3 (third pose; translation differs at approximately `1e-9`) | after 39 cloud messages / 38 poses, wall `2:17.97`, peak RSS `94,236 KiB`; stopped because the divergent DSO path became computationally impractical |
| HILTI exp04, cap 100 characterization | trajectory line 2 (second pose; translation differs at approximately `3e-9`) | after 59 cloud messages / 58 poses, wall `1:45.19`, peak RSS `95,728 KiB`; stopped for the same reason |

The interrupted artifacts are intentionally not APE or map-quality evidence;
their receipt paths are `/tmp/mid360-plugin-bsymbolic-300-1run.74QrWA` and
`/tmp/hilti-plugin-bsymbolic-100-1run.Ruynlh`.
`nm -C` shows both the component and ordinary DSO exporting weak pclomp
templates, while the normal binding trace resolves the ordinary DSO to the
component. The independent first-pose divergence is sufficient to close the
promotion gate: do not switch the default NDT route, do not claim plugin DSO
APE/map non-regression, and do not proceed to live/pluginlib production
injection until the translation-unit/ODR boundary is redesigned and the full
HILTI/MID-360 replay is rerun in a genuinely isolated process.

### Phase 2 ODR-safe hybrid resolver slice (2026-08-21)

The isolated-DSO No-Go does not require giving up external extensibility. The
shell now has a RegistrationResolver with two disjoint, explicit namespaces:

| selector form | backend kind | construction and provenance |
| --- | --- | --- |
| lidarslam_builtin/<name> | host_builtin | a factory registered by the host; for NDT the factory body is in scanmatcher_component.cpp, next to the same-TU pclomp instantiation |
| <external-package>/<name> (for example, lidarslam_default_plugins/NdtOmp) | pluginlib | pluginlib class loader and manifest; the session retains the loader until the plugin is destroyed |

The resolver returns the same RegistrationPluginSession shape for either
kind. Its provenance() contains backend_kind, the canonical selected class
ID, metadata, configured capabilities, the sorted typed parameter map, and
library/manifest paths. The latter two fields are intentionally empty for a
host-built-in session. The offline receipt records backend_kind and the
canonicalized metadata class ID, so host and external runs cannot be confused
by a shared implementation name.

lidarslam_builtin/ is reserved for host factories. An external plugin
manifest that declares any class under that prefix is a namespace violation;
the pluginlib loader also refuses a reserved ID directly. A duplicate host
registration or an exact host/pluginlib class collision is a hard resolver
failure. An empty, unknown, or otherwise invalid selector never falls through
to NDT or to another backend. If pluginlib initialization fails, an explicit
host-built-in can still be resolved; an external selector reports the loader
diagnostic instead.

The runner-only opt-in is unchanged:

~~~
ros2 run scanmatcher scan_matcher_offline_runner --ros-args \
  -p registration_plugin_enable:=true \
  -p registration_plugin_class:=lidarslam_builtin/NdtOmp
~~~

The default ScanMatcherComponent(NodeOptions) constructor and every live
registration_method=NDT path remain same-TU built-in construction. Only the
offline runner selects RegistrationConstruction::kDeferredPluginInjection,
registers the host factory, resolves and validates the session, writes the
receipt, and injects before sensor processing. No production default switch
is made in this slice.

The clean-install resolver tests cover both lifetime models: a host session
continues to align after its resolver is destroyed without a class loader, and
an installed fake external session continues after the resolver is destroyed
with its pluginlib loader retained. They also cover duplicate host IDs,
reserved namespace requests, invalid host configuration, and external
discovery/configuration failures. With the workspace install space sourced,
the short same-TU characterization prefixes matched the pre-migration
baseline exactly:

| input | host-resolver prefix | baseline prefix | result |
| --- | --- | --- | --- |
| MID-360, first 300 poses / 44 submaps | trajectory MD5 1cc32283cc8b4eff5c4caed9c155c6ca, submap MD5 c14ce769c607fd85cffc3848ac42cadf | same MD5s | exact; /tmp/hybrid-mid-host-300 |
| HILTI exp04, first 100 poses / 1 submap | trajectory MD5 822739ea4919a44a096f848d427d06ef, submap MD5 3ec9bc06ca9abfc9623e767775b30f84 | same MD5s | exact; /tmp/hybrid-hilti-host-100 |

These are host-resident compatibility receipts, not an independent external
DSO proof and not a production promotion gate. The NDT external-DSO replay
remains blocked by the independent -Bsymbolic divergence above; the optional
SMALL ordinary-DSO replay is separately recorded as compatibility wiring only
in the receipt linked below.

### Phase 2 GICP characterization slice (2026-08-21)

The frontend's legacy `registration_method=GICP` construction is still the
direct pclomp path for the normal `ScanMatcherComponent(NodeOptions)` and is
not changed by this slice. The new
`lidarslam_default_plugins::GicpOmpRegistration` adapter is available through
the same-TU host factory `lidarslam_builtin/GicpOmp` and through the
pluginlib-facing `lidarslam_default_plugins/GicpOmp` class for offline
characterization only.

The characterization contract is deliberately narrow and records the exact
legacy behavior:

| item | frozen behavior |
| --- | --- |
| construction | `GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI>`; `setMaxCorrespondenceDistance(gicp_corr_dist_threshold)` followed by `setTransformationEpsilon(1e-8)`; no new iteration/thread setters |
| target | `kAcceptHostPrepared`; map-update and recovery refreshes apply the existing `vg_size_for_input` voxel filter before `setInputTarget`; initial transformed map target follows the existing `vg_size_for_map` path |
| source/guess | host voxel-filtered source; initial guess is enabled for the same `sim_trans`; the adapter uses the no-guess overload when the typed request disables it and ignores a non-finite disabled guess |
| priors | rotation and translation priors are not advertised or sent, matching legacy GICP |
| correspondence | adaptive requests may override the max distance; result diagnostics expose `sqrt(fitness)` as the correspondence metric |
| adaptive reset | when adaptive mode is configured, max correspondence is reset to `std::numeric_limits<double>::max()` after every call, including the first call with EMA zero; an explicit per-call override is also cleared on success or exception |
| lifetime/failure | target, per-call distance, and configuration are cleared by `reset()`; configure, target, and align failures return typed failures and no exception crosses the API |

The ROS-free `makeGicpParameterMap()` maps
`gicp_corr_dist_threshold`, fixed epsilon `1e-8`, and the explicit adaptive
boolean into the typed map. Resolver requests require the GICP target policy
and `kSquareRootFitnessProxy` metric. Selecting the NDT host class for a GICP
request, or the GICP host class for an NDT request, therefore fails before the
first sensor message through capability mismatch; class-name fallback is not
used. The adapter tests include direct pclomp fixture comparison, adaptive
first/second-call reset, disabled-guess NaN handling, reset/reconfigure, and
typed configuration rejection. The shared DSO remains a useful external
contract fixture, but its pclomp template symbols are not evidence of
production-equivalent replay; the host-resident factory is the only precision
compatibility path in this slice.

A short real-bag smoke receipt compared the unchanged legacy direct GICP path
with the host-resident resolver path on the MID-360 bag (`/livox/lidar`,
`/livox/imu`, first 100 clouds, the same `lidarslam.yaml`). Both produced 100
poses and one submap: trajectory MD5
`ed2f59fb958b8a8c1c9fc0a7f5bdc00c` and submap-summary MD5
`ca57c937f7fdc4b1a997be23972d690e` on each side. The host receipt is in
`/tmp/gicp-replay-host-100/run1/registration_plugin_receipt.yaml`. This is a
short host compatibility smoke only; it is not a full-sequence accuracy,
APE, map-quality, or external-DSO production gate. In this cap, after the
initial map, all 99 alignment updates on each side reported `converged=false`
and the existing pose-acceptance policy rejected them; the matching hashes
therefore prove dispatch/serialization compatibility only, not GICP accuracy.

#### Minimal external plugin authoring example

An external package owns a namespaced class and implements only the installed
C++14 interface:

~~~
class RobustRegistration final
  : public lidarslam::plugins::registration::RegistrationPlugin
{
  // metadata(), capabilities(), configure(), setInputTarget(), align(),
  // and reset() implement the typed contract and never throw across calls.
};

PLUGINLIB_EXPORT_CLASS(
  my_registration_plugins::RobustRegistration,
  lidarslam::plugins::registration::RegistrationPlugin)
~~~

Its installed XML must use the same external namespace and exact base class:

~~~xml
<library path="my_registration_plugins">
  <class
    name="my_registration_plugins/Robust"
    type="my_registration_plugins::RobustRegistration"
    base_class_type="lidarslam::plugins::registration::RegistrationPlugin">
    <description>External robust registration adapter.</description>
  </class>
</library>
~~~

The implementation's metadata().class_id must equal the XML class ID. A host
alias is only allowed when the host explicitly registers that alias and
canonicalizes the resolved session; external plugins cannot claim the
reserved lidarslam_builtin/ prefix.

#### ABI and ODR rules

- lidarslam_plugin_interfaces remains C++14 source-compatible and free of
  ROS/pluginlib. The shell and pluginlib packages may use C++17, but every
  plugin is rebuilt for the target ROS distribution and compiler/toolchain.
- External shared libraries hide ordinary symbols. Only the pluginlib export
  entry points and intentional public interface symbols are visible.
- A plugin DSO must not export the same pclomp/Eigen template specialization
  used by the host. Concrete template implementation headers stay in one
  owning translation unit, or the implementation uses a process/ABI boundary
  designed for that purpose.
- The host NDT factory is defined in the legacy scanmatcher translation unit;
  moving NdtOmpRegistration construction into an offline runner or a
  separately linked DSO invalidates the byte-equivalence claim.
- Metadata, capabilities, typed configuration, and failure semantics are
  validated before the first cloud. No selector may trigger an implicit
  fallback or late plugin replacement.

### Phase 2 optional SMALL external DSO gate (2026-08-21)

The isolated `small_gicp` vendor build was used to load the real external
classes `lidarslam_default_plugins/SmallGicpPcl` and
`lidarslam_default_plugins/SmallVGicpPcl` through pluginlib.  First-100 and
full HILTI exp04 and MID-360 runs produced exact trajectory, submap, and map
artifacts against the legacy same-translation-unit paths.  Receipts identify
`backend_kind: pluginlib`, the dedicated Small DSO, and
`registration_plugins_small.xml`; host aliases were not selected.

The first ordinary combined-DSO result was not an independent proof:
`nm -D -C` and `readelf -Ws` showed default-visible weak
`small_gicp::RegistrationPCL` and pclomp template symbols in that DSO, and
`LD_DEBUG=bindings` showed the DSO's `SmallGicpRegistration::align` and
`RegistrationPCL::computeTransformation` resolving to
`libscanmatcher_component.so`.  The component's same-TU instantiations
therefore satisfied the ordinary DSO.  The optional Small implementation is
now in a dedicated DSO built with scoped `-Wl,-Bsymbolic-functions`; the
binding trace and full replay evidence show DSO-local resolution for the two
algorithm symbols.  The scoped independent Small ODR gate is **Go** for the
pinned Jazzy/vendor/toolchain build.  Live/default promotion and absolute
accuracy remain **No-Go** until the broader toolchain/ground-truth policy is
closed.

The full command paths, class receipts, hashes, binding trace, and candidate
fixes are recorded in the
[SMALL external DSO/ODR gate receipt](small-gicp-external-dso-odr-gate-2026-08.md).
The production default and README claims remain unchanged.

### API version policy

- API major `1` is the first public contract. A different major is a hard
  startup failure.
- An older minor is accepted when the major matches.
- A newer plugin minor is rejected. The current compatibility predicate is
  exact-major and `plugin.minor <= host.minor`; any future minor negotiation
  requires a separately tested contract change.
- Adding or changing a pure virtual function, changing ownership/lifetime, or
  changing the meaning of an existing field requires API major `2`.
- Minor additions use capability bits, optional diagnostics, or a new
  extension interface; they do not add a new pure virtual method to the major
  interface. Source compatibility is promised within a major, not C++ ABI
  compatibility between ROS distributions.

## Failure semantics

### Startup failures

The host fails before creating subscriptions or publishing mapping artifacts
for any of these conditions:

- unknown class ID, unavailable optional dependency, or malformed plugin XML;
- API major mismatch or unsafe newer minor;
- missing/invalid metadata, disallowed license, or missing build provenance;
- conflicting canonical/legacy selectors or parameter values;
- invalid plugin parameters or a `configure()` failure;
- missing required point fields or an unsupported requested capability;
- failure to create the initial target or to satisfy the host's target policy.

The error includes role, requested ID, resolved ID, API versions, missing
capability/field, and an actionable list of available classes. There is no
implicit fallback to another registration method.

### Per-call failures

`align()` always returns a fully initialized `RegistrationResult` or the shell
converts an exception to `kPluginException`; an invalid result is never passed
as an accepted pose or loop edge. The host handles codes as follows:

| Failure | Frontend behavior | Backend behavior |
| --- | --- | --- |
| empty source/target or insufficient correspondences | retain previous accepted pose; do not update the map; enter the existing acceptance/recovery policy | skip the candidate; do not create a loop edge |
| `kNotConverged` with finite output | expose `valid=true`, `converged=false`; the existing frontend `reject_nonconverged_pose_update` policy decides; | reject the candidate unconditionally, matching `BackendCore` |
| numerical failure, non-finite transform/fitness, invalid request | reject the current update and mark the session degraded; repeated/fatal policy is owned by the host | reject the candidate and log a typed diagnostic |
| timeout | reject the current update/candidate; record elapsed budget and plugin diagnostics | no loop edge; core safety gates remain unchanged |
| `kPluginException` or internal error | stop the mapping session cleanly; no partial artifact is marked complete | stop the replay/session; offline runner exits nonzero and writes no completion marker |

There is no runtime fallback from a failed plugin to NDT or another plugin.
Fallback, if ever desired, is an explicit host-level multi-stage policy with
its own manifest and characterization; it is not part of `RegistrationPlugin`.
The plugin may retain a valid target after a recoverable empty-input result,
but after an exception the session is discarded rather than reused.

## Characterization fixtures and contracts

Phase 0 freezes the current behavior before changing a call site. Existing
tests remain the oracle and new interface tests run the same inputs through the
adapter. Expected values are recorded as binary/hash outputs where the current
test already requires bit identity, and as the stated tolerances where it is a
characterization only.

### Fixture R1: standalone NDT determinism

Source: [`test_registration_determinism.cpp`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/test/test_registration_determinism.cpp).

- Generate the structured cloud with `std::mt19937(42)` and jitter
  `[-0.02, 0.02]`.
- Include the 20 m ground plane, walls at `x=10` and `y=-10`, each 3 m tall,
  and the asymmetric pillar around `(3,4)`.
- Transform the source by translation `(0.4, 0.2, 0.05)` and 8 degree yaw.
- Use NDT `DIRECT7`, resolution `2.0`, maximum iterations `35`, epsilon
  `0.01`, and initial identity guess.
- Run fresh plugin objects five times at one thread. The matrix bytes and
  fitness must be identical and converged must be true.
- Run five times at 2 and 4 threads. Preserve the existing report bounds and
  record the observed maximum matrix/fitness deviation; these runs are not a
  cross-thread byte-identity promise.
- Compare one-thread to 2/4-thread output with the existing `<5e-3 m` and
  `<0.1 degree` bounds.

The adapter test must additionally assert that the request/result fields map to
the legacy PCL values and that `reset()` removes the target and per-call
correspondence override.

### Fixture R2: backend revisit and gate semantics

Source: [`test_backend_core.cpp:318-575`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/main/graph_based_slam/test/test_backend_core.cpp#L318).

- Generate the 20-by-20 structured intensity cloud with seed 42, jitter
  `[-0.05, 0.05]`, and the deterministic z pattern in `makeStructuredCloud()`.
- Build 11 submaps. Submaps 0 and 10 share the cloud; submap 10 is translated
  by `0.3 m`; intermediate submaps have empty clouds and poses around
  `y=1000+i`.
- Use search submap count 1, distance closure 20 m, search range 10 m,
  generic fitness threshold 10, translation cap 10 m, rotation cap 180
  degrees, voxel leaf 0.2 m.
- Use one-thread NDT, `DIRECT7`, resolution 2 m, 35 iterations, epsilon 0.01.
- The proposal must be `(0,10)`, fitness `<10`, and the historical log lines
  must remain ordered.
- Two fresh sessions must produce bitwise-identical relative pose, fitness,
  and logs. A strict translation cap must still reject the candidate while
  retaining the best-attempt diagnostic.
- One-at-a-time and batched event-driven arrivals must produce the same output
  stream. Empty source/target must produce no proposal and no logs.

This fixture verifies that moving from a PCL reference to a plugin does not
move candidate aggregation, initial-guess policy, overlap metrics, or gate
policy into the plugin.

### Fixture R3: frontend target policy and priors

Add a deterministic synthetic scan sequence before the first live frontend
call-site change:

- use the R1 geometry with `PointXYZI` intensity and a source that has at least
  `min_points_for_scan` points;
- initialize the map through the same `vg_size_for_map` path;
- perform one target refresh with NDT and one with GICP using the same
  `vg_size_for_input`;
- assert that the resolved target policy selects the raw NDT target and the
  host-voxelized non-NDT target, without a plugin-ID `if` branch;
- enable the IMU rotation prior and z translation prior for the NDT adapter,
  compare the final transform and clear-after-call state to the current
  concrete-cast path;
- enable adaptive correspondence and assert NDT mean-distance versus GICP
  `sqrt(fitness)` proxy selection and reset behavior;
- test empty/recovery targets and non-convergence through the existing pose
  acceptance policy.

The fixture records the target point counts, request flags, result transform,
fitness, convergence, diagnostics, and target/reset sequence in a stable JSON
or binary receipt. It does not assert that different algorithms produce the
same pose.

### Fixture R4: live/offline replay identity

Run the existing lockstep scripts with the resolved plugin manifest:

- `scripts/run_frontend_determinism_check.sh` with
  `async_map_update=false` and a fixed registration thread count; trajectory
  and submap streams remain byte-identical across runs for the default NDT
  adapter;
- `scripts/run_offline_determinism_check.sh` over the backend-input bag;
  `loop_edges.csv`, proposal transforms, and manifest-resolved settings remain
  identical across runs;
- compare live and offline resolution of the same canonical configuration and
  plugin class ID before comparing output hashes.

No dataset identity or bag path is passed into a plugin. The runner's input
order and `sequence_id` are the only ordering information.

### Negative fixtures

The contract test kit must include a separate example plugin for each failure:
unknown class, API major mismatch, unsupported rotation prior, missing
intensity, invalid parameter, empty target, non-converged result, non-finite
transform, and throwing `align()`. Verify startup failure, typed diagnostics,
absence of silent fallback, and absence of a false `.complete` marker in
offline output.

## Phase 1 migration order

The order is part of this ADR because changing the order would create two
uncharacterized registration semantics at once.

1. **Freeze R1/R2 and inventory hashes.** Run the current direct tests and
   record compiler, dependency, thread count, and output receipts. Add R3 and
   negative test skeletons without changing production construction.
2. **Publish the C++14 interface.** Add version, typed request/result/failure,
   capability definitions, and a test-only fake. Verify the header builds from
   a C++14 consumer with no ROS or pluginlib include.
3. **Build a PCL bridge.** Wrap the existing concrete PCL object in a
   `PclRegistrationAdapter` and prove R1/R2 equivalence. The bridge is an
   internal migration aid, not the final public plugin API.
4. **Migrate frontend NDT.** Create the built-in `NdtOmp` plugin and preserve
   constructor defaults, raw-target policy, explicit/no-explicit initial guess,
   IMU priors, adaptive correspondence, clear-after-call, and result
   acceptance. Replace only the frontend construction/casts after R3 passes.
5. **Move loader ownership to live frontend shell.** Resolve canonical/legacy
   parameters, check capabilities, and inject the configured plugin into the
   frontend runtime. Keep `ScanMatcherComponent(NodeOptions)` as the ROS
   composition entry point, but make its registration runtime dependency
   explicit rather than constructing a concrete PCL object in the hot path.
6. **Migrate backend NDT.** Replace `makeLoopRegistration()` with the same
   `NdtOmp` adapter under a separate `backend_loop` role profile. Change
   `BackendCore` from a PCL registration reference to the typed plugin only
   after R2 proves identical aligned output, convergence, fitness, logs, and
   gate behavior.
7. **Migrate offline runners.** `scan_matcher_offline_runner` and
   `graph_slam_offline_runner` each resolve and load the same plugin class and
   configuration as their live counterpart. The live/offline manifest and R4
   gate become mandatory before more methods are added.
8. **Migrate built-in GICP backend.** The frontend `GicpOmp` adapter and
   host-resident characterization selector are frozen above; next preserve
   the backend factory's distinct 30 m and 100-iteration defaults and remove
   the backend's direct factory branch only after backend R2 and current GICP
   accuracy gates pass.
9. **Migrate optional scanmatcher adapters.** Add FAST_GICP and FAST_VGICP only
   when their dependency is present. The conditional SMALL_GICP and
   SMALL_VGICP adapters now have direct fixtures and host factories, but both
   families remain opt-in until target, thread, result, deterministic, and
   replay claims are complete. An absent optional package must still produce a
   clear class-availability error for a requested legacy method.
10. **Delete concrete branches.** Remove `registration_method`-driven target,
    prior, and adaptive casts from `scanmatcher_component.cpp`, delete the
    backend factory after all call sites use the loader, and remove direct
    registration includes from the cores. Keep direct NDT analysis tools and
    pre-migration tests until their replacement receipts are archived.
11. **External consumer proof.** Build a separate workspace containing one
    minimal registration plugin and one consumer against the installed C++14
    interface on Humble and Jazzy. The core repository must not be edited by
    that consumer.

## Phase 1 hard gates

Phase 1 is complete only when all of these are true:

- default NDT R1 and R2 output is byte-identical wherever the current contract
  promises byte identity;
- frontend and backend live/offline runs use the same class ID, API version,
  resolved parameters, and configuration hash;
- all currently supported registration methods retain their existing accuracy
  and map-quality gates;
- plugin dispatch adds less than 0.5% processing overhead and end-to-end RTF
  stays within 5% of the corresponding pre-plugin baseline;
- no false loop, map, or pose acceptance gate is bypassed by a plugin result;
- unknown plugins, API mismatch, missing capabilities, invalid configs,
  missing optional dependencies, non-finite results, and throwing plugins pass
  negative tests;
- installed consumers build on Humble and Jazzy with C++14 interface headers;
- `BackendCore` and the ROS-free frontend core have no pluginlib/rclcpp
  dependency and no registration-method string branch;
- manifests contain plugin provenance, capabilities, optional dependency
  versions, role, thread/determinism mode, and configuration hash for every
  live and offline run.

If any hard gate fails, the legacy/default implementation remains the release
path and the plugin path is not advertised as behaviorally equivalent.

## Consequences

Positive consequences:

- an external registration package can be installed and selected without
  editing central ROS components;
- live and deterministic offline paths share one typed algorithm contract;
- NDT-specific priors and target preparation become explicit capabilities
  instead of concrete casts and silent method branches;
- provenance, license, dependency, and configuration identity are reproducible
  in every benchmark manifest;
- the C++14 boundary can be installed independently of ROS message APIs.

Costs and constraints:

- each ROS distribution/ABI still needs a plugin build; this ADR does not
  promise binary portability;
- adapters initially copy canonical clouds into PCL data structures;
- configuration resolution and manifest validation add startup code;
- plugin authors must implement typed failure and capability declarations,
  rather than only exposing an `align()` function;
- the separate stateful `small_gicp_odom_node`, analysis tools, and future
  frontend seam remain distinct until their own contracts are characterized.

## Rejected alternatives

### Put `pluginlib::ClassLoader` in `BackendCore`

Rejected. It would make the deterministic, ROS-free core depend on ROS plugin
discovery and wall-clock/executor concerns, and would make offline and live
construction semantics diverge.

### Expose `pcl::Registration` as the public API

Rejected. It would freeze a PCL ABI/type hierarchy, cannot represent typed
prior/capability/failure semantics, and would force external plugins to inherit
implementation-specific behavior.

### Keep a larger `if (registration_method == ...)` compatibility switch

Rejected as the end state. A temporary legacy adapter is allowed during the
two-release window, but every new NDT-specific branch increases the number of
live/offline paths that must be characterized and makes external extension
impossible.

### Automatically fall back to NDT

Rejected. A missing optional dependency or a runtime plugin failure must be
visible in the manifest and error path; silently changing the estimator would
invalidate accuracy and reproducibility claims.

### Make `small_gicp_odom_node` implement this interface by aliasing names

Rejected for Phase 1. It is a stateful incremental voxel-map odometry frontend
with ICP/GICP factors and a different request/result lifecycle. It belongs at
the later `FrontendPlugin` seam unless a second implementation demonstrates a
stable shared contract.
