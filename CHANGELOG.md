# Changelog

## Unreleased

### Added

- Map review (`lidarslam.map_edits`): a person or an agent keeps or removes what
  a long-term map update changed. `python -m lidarslam store candidates` lists
  the changes as object-sized clusters (old points removed, new points held
  back, structure added); `store edit` keeps or removes candidates or boxes;
  `store review-pack` writes the file the new browser map editor
  (`docs/map-editor/`, on the docs site) opens, and `store apply-edit` applies
  the edit it saves. Edits are append-only store-frame voxel decisions with
  their author and reason, applied after the change evidence;
  `store rollback-edit` undoes the latest. Without edits the export is
  unchanged. `python -m lidarslam mcp STORE` (`pip install "lidarslam[agent]"`)
  is an MCP server for an agent such as Claude Code to do the same review:
  candidates with numbers and a top/side picture, keep / remove decisions
  committed as one `agent:<name>` edit, or with `--propose-only` a proposal
  file for a person to apply.
- Browser demo ([Try It in Your Browser](docs/browser-demo.md)): RKO-LIO odometry
  (`rko_lio::driver::OfflineDriver`) compiled to WebAssembly maps a 60 s clip of
  the public Livox MID-360 recording in a web worker and draws the map with
  three.js. On the clip the WebAssembly build gives the native build's 599
  poses (`web/replay_node.js` against `web/demo_reference.tum`; a CI job checks
  it). `web/` holds the build (`build_wasm.sh`, a serial oneTBB stand-in) and
  the clip writer. The RKO-LIO submodule copies two IMU priors into a lambda
  explicitly so Emscripten's clang 15 compiles it; outputs are unchanged.
- Colab notebook (`examples/colab/first_map.ipynb`): `pip install lidarslam`,
  download the public MID-360 recording, `lidarslam-map start --engine core`,
  then a top view and an interactive 3-D view of the verified map and a
  download of the bundle. It ran end to end in a fresh Python 3.12 venv with
  no ROS 2 (VERIFIED, 2,233,762 map points, about 4.5 minutes).
- macOS arm64 wheels (Apple Silicon, macOS 15+, Python 3.10-3.13): the wheels
  workflow builds the same C++ dependencies from source on a macOS 15 runner
  with Homebrew's libomp, bundles them with delocate, and installs and tests
  the wheel on macOS 15 and the latest macOS runner. A release tag publishes
  them to PyPI with the manylinux wheels. The map runner reserves its emergency evidence space by
  writing zeros where `os.posix_fallocate` does not exist (macOS).
- Long-term mapping plan ([roadmap](docs/roadmap/long-term-mapping-2026-11.md)):
  update a stored map live while driving, removing stale points automatically,
  with a preregistered evaluation on the public LUH long-term indoor dataset
  (three layouts of one hall, total-station positions, a scan of the empty
  hall). `scripts/convert_luh_longterm_bag.py` streams one of its ROS 1 bags
  into a compact ROS 2 bag (Ouster OS1 and IMU). First look on development
  sequence S3: RKO-LIO odometry 0.033 m position RMSE over 44.5 m.
- Live mapping without ROS 2 (`lidarslam.live`): a `Session` fed IMU and LiDAR
  messages one at a time hands out every scan's live pose (the backend's
  causal pose, through the new `OfflinePipeline::setLivePoseListener`), and
  `python -m lidarslam live <log> --realtime` plays a log at the sensor's
  rate. On LUH S3 it keeps up with the data (118.7 s in 118.9 s, pose 0.028 s
  after its scan at the median, 0.67 s at most). Outputs of the other paths are
  unchanged.
- Long-term map store (`lidarslam.store`): a directory of mapping sessions with
  `store.json` as the committed list, each session's submaps (poses and the
  backend's compressed scans) and an Autoware bundle exported from them.
  `python -m lidarslam live ... --store STORE` adds a live session;
  `python -m lidarslam store info|export|rollback STORE`. For one session the
  export is byte-identical to the backend's bundle (CI, and LUH S3). The map
  bundle writing moved into `map_export::writeBundle`; outputs are unchanged.

## 0.13.0 - 2026-11-18

The first release from the renamed repository `rsasaki0109/lidar_slam`. The
`newer_college_math_hard` release profile was not measured for this release
(a third one-time exception; see the release notes).

### Added

- GNSS anchors without ROS 2: with `use_gnss` (e.g. `lidarslam_rtk_gnss.yaml`)
  `graph_slam_offline_runner`, `lidarslam.backend` and the core engine read
  the input's `gnss_topic` and anchor the final pose graph as the live node
  does (yaw alignment, gravity prior, GNSS projector origin). The live node's
  GNSS handling moved into the shared ROS-free `gnss_anchor_store.hpp`
  unchanged. With `use_gnss` false every output is byte-identical. On RTK-SLAM
  Stadtgarten 2 and 1 the core engine gives 0.100 m and 0.067 m absolute
  (the live ROS 2 path: 0.100 m and 0.068 m).
- Velodyne packets without ROS 2: the `lidarslam` package decodes
  `velodyne_msgs/msg/VelodyneScan` with velodyne_pointcloud's decoder
  (ros-drivers/velodyne 56fc178d, BSD-3-Clause, vendored), so
  `python -m lidarslam odometry|map` and `lidarslam.io.SensorLog` read packet
  bags directly (`velodyne.*` parameters; the model calibrations ship with the
  package). On Leo Drive all-sensors-bag1 every scan of the three streams is
  byte-identical to velodyne_transform_node's cloud (1,093 of 1,093).
  `lidarslam-map run|start --engine core` maps a VelodyneScan + Imu bag with
  the RKO-LIO profile on the packets (Leo Drive bag1: verifier PASS,
  `start` VERIFIED, trajectories byte-identical to `lidarslam map`), and
  `lidar_timestamps.force_relative` is set for packet input unless the
  parameters say otherwise. The ROS 2 engine keeps its profiles; the classic
  `packet_applanix_smoke` profile still needs ROS 2.

- Offline backend (`graph_slam_offline_runner`, `lidarslam.backend` and the
  core engine): `submap_cloud_max_error_m` (default 0.0, lossless as before)
  keeps each submap's scan with x, y and z within that many metres, in
  0.28-0.44x the bytes (step quantization + zstd; SZ3 was measured and stored
  more). On eight recorded runs 0.005 m moves the pose-graph trajectory by at
  most 2.3 mm and keeps the loop edges; the plane refinement can amplify the
  change (up to 92 mm on HILTI exp04), and on GrandTour CON-1 it moves by
  0.77 m even for a 1 um change
  (bench_report/data/sz3_cloud_store_2026-11-17).

### Changed

- Map refinement (offline runner, `lidarslam.backend`, core engine): a
  window whose result changes the relative transform of two consecutive
  submaps by more than 0.10 m or 1 deg against the pose graph is now
  discarded (`refine_window_max_adjacent_change_m` / `_deg`; 0 restores the
  old behaviour byte for byte). On GrandTour CON-1 and CON-2 the ungated
  refinement moved adjacent submaps apart by up to 1 m, left the trajectory
  worse than the pose graph (0.450 / 0.352 m against 0.384 / 0.291 m) and
  moved by up to 1.6 m for a 1 um input change; gated it keeps the pose graph
  there. On MapsHD calibration-room 1 and Stadtgarten 2 no window is gated
  and the output is unchanged (bench_report/data/refine_stability_2026-11-17).
  The opt-in `refine_relative_translation_sigma` /
  `refine_relative_rotation_sigma_rad` add factors between consecutive
  submaps instead.
- The repository is now `rsasaki0109/lidar_slam` (was `lidar_slam_ros2`;
  GitHub redirects the old URLs) and the docs live at
  https://rsasaki0109.github.io/lidar_slam/. Release images from v0.13.0 on
  are `ghcr.io/rsasaki0109/lidar_slam:vX.Y.Z-<distro>`; the v0.12.x and older
  images keep `ghcr.io/rsasaki0109/lidar_slam_ros2`. The release checker and
  the `lidarslam-map-docker` launcher pick the name from the version, and the
  product schemas accept both repository and docs URLs (their `$id`s are
  unchanged).

### Fixed

- `extract_rosbag2_sqlite_topics.py`: rows of a Jazzy bag's
  `message_definitions` table are no longer taken for topics.

## 0.12.0 - 2026-11-17

The first release with the ROS-free `lidarslam` Python package. The
`newer_college_math_hard` release profile was not measured for this release
(a second one-time exception; see the release notes).

### Added

- Mapping without ROS 2 (preview): `pip install ./lidarslam_core` builds the
  `lidarslam` Python package with the RKO-LIO odometry and the graph_based_slam
  offline backend, and `lidarslam-map run|start --engine core` maps a bag with
  it (rko_lio_graph profiles; `--engine auto`, the default, still uses ROS 2 when
  the `ros2` command is found). The odometry and the offline backend give the
  same bytes as the ROS 2 offline node and `graph_slam_offline_runner`; the map
  bundle is written by the live node's map saving, moved into the ROS-free
  `map_export` library. See docs/roadmap/ros-free-core-2026-11.md.
  The package also installs the `lidarslam-map` command (the product scripts,
  schemas and parameter files of the ROS 2 install), so `lidarslam-map start`
  runs in a plain Python environment; `lidarslam-map doctor` reports ready with
  the core engine there, and the diagnosis's map tilt check reads the IMU
  without ROS 2.
  CI builds manylinux wheels of the package (`wheels.yml`) that bundle PCL,
  g2o, FLANN, oneTBB and Boost built from Ubuntu 24.04's sources, so the
  wheel installs with no apt packages and maps HILTI exp14 to the same bytes.
  The wheels cover Python 3.10-3.13 on x86_64 and aarch64, and a release tag
  publishes them to PyPI as `lidarslam` (`pip install lidarslam`) through
  Trusted Publishing; the package version is the repository `VERSION`.
- The multi-session join without ROS 2: `lidarslam-map merge
  --place-recognition` and `start --join-map` run on the `lidarslam` package
  when ROS 2 is missing (`merge --engine auto|ros2|core`; `python -m lidarslam
  join` takes the parameters of `graph_slam_multisession_join`). The join is
  one ROS-free library under both, and the package writes the same files as
  the ROS 2 executable. A core-engine run's `core_params.json` serves as the
  join's parameter file.
- `graph_slam_multisession_join`: a GNSS gate, a candidate off by default
  (`multisession_gnss_gate`). When both sessions carry RTK GNSS, a set of
  place matches is accepted if, at its places, its transform agrees with
  the transform GNSS gives there (median within 1.0 m and 2 deg). It
  replaces the whole-map check, which refuses right outdoor joins. With the
  gate off, every output is byte-identical. It is confirmed only on the
  data it was developed on (report 56.3/56.4).
- `lidarslam-map merge --place-recognition`: outputs recorded with `--editable`
  are placed by `graph_slam_multisession_join` (place recognition, no prior,
  with its refusals) before the merge's alignment and overlap gates. A refused
  session stops the merge with the join's reason. Each run's self-similarity
  result is kept in a user cache (`--join-cache-dir`), so a later merge with the
  same run skips that check (456 s to 7 s on Stadtgarten 2).
- `lidarslam-map start --join-map <output>`: after the run, join it to a stored
  editable map with `merge --place-recognition` into `map_joined/` (implies
  `--editable`). A refused join keeps the run's own map and prints the reason.
- An editable run of a bag with a GNSS (NavSatFix) topic keeps it in
  `backend_input/` (`record_backend_input.sh --gnss-topic`), for later checks
  of a session against GNSS. The map and the loop replay do not use it: on
  Stadtgarten 2 the backend topics and every replay output are byte-identical
  with and without it, and all 8,768 fixes of the bag are kept.
- **RTK GNSS anchoring that reaches the published baselines** — on RTK-SLAM
  Stadtgarten 2 the GNSS-anchored map scores 0.100 m absolute checkpoint RMSE
  without alignment (FAST-LIO-SAM 0.099 m, OKVIS-LVIG 0.092 m), from 7.6 m
  before. `lidarslam/param/lidarslam_rtk_gnss.yaml` holds the settings; new
  graph_based_slam parameters `gnss_interpolation_max_gap_sec`,
  `gnss_antenna_offset_xyz`, `gnss_robust_kernel_delta`, `gravity_prior_info`
  (an absolute roll/pitch prior per submap) and `gnss_vertical_weight_scale`
  default to the previous behaviour. Tuned on that one sequence; see
  `docs/research/gnss-rtk-slam-absolute-accuracy.md`.
- `rko_lio_slam.launch.py` `bag_gnss:=true` (runner `--bag-gnss`): the RKO-LIO
  offline node passes the bag's NavSatFix topic to graph_based_slam. The
  offline path never fed GNSS before.

### Fixed

- A map run no longer ends while RKO-LIO is still reading the bag. Map outputs
  unchanged for 15 s (a stretch without loops) were taken for the end of offline
  processing; on RTK-SLAM Stadtgarten 1 the run stopped at 40 % of the bag.
  While the frontend wrapper runs, only its completion or failure ends the wait.
- The bag preflight accepts the Livox ROS driver's per-point `offset_time`
  (uint32 ns), which RKO-LIO already reads for deskewing. Such bags (RTK-SLAM
  Stadtgarten) were offered only the GNSS smoke path, not the RKO-LIO profiles.
- `graph_slam_offline_runner` writes its registration receipt for GICP and
  SMALL_GICP loop registration too, and the offline replay
  (`run_offline_determinism_check.sh`, used by `lidarslam-map edit`) accepts
  it. Since the walking preset switched to SMALL_GICP, replays of walking runs
  stopped on the missing receipt.
- RKO-LIO reads the Livox `offset_time` per-point time, so deskew works on
  Livox PointCloud2 bags (with `deskew: true` it dropped every frame with an
  error before, so the RTK-SLAM presets ran with deskew off).
- The GNSS stamp check compares the fix header with the latest odometry stamp
  instead of the node clock, so a recorded bag replayed offline keeps its fix
  stamps; the GNSS subscription keeps 1000 messages instead of 5.

### Removed

- **Two-system comparison tooling** — the competitive benchmark stack
  (evidence bundles, GT-blind holdouts, execution selection, rival source
  closure), the M6a10 runtime phase contract, the GLIM clean-room adapter,
  the GLIM and FAST-LIVO2 comparison runners and the `voxel_slam_dev` patches
  are gone, with their tests and documentation (482 files). No release gate
  used them. The README and comparison results stay; their reproduction now
  starts from the `v0.11.0` tag.
- **Orphaned research files** — the `configs/sota_v6/` audits and
  `scripts/audit_v44g_raw_shadow_failure_profile.py`, the
  `configs/voxel_slam_v33`–`v41` run configurations whose patches went with
  the comparison tooling, and five scripts nothing called
  (`run_ape_cycle.sh`, `check_ours_trajectory_hash_equivalence.py`,
  `score_repeated_trajectory_benchmark.py`, `run_isuzu_3dgs_viewcount.sh`,
  `docker/ouster_sensor_sync_replay.Dockerfile`). The research records that
  name them point at the `v0.11.0` tag.
- **Duplicate and unused media** — the Stadtgarten and RTK-SLAM flythrough
  GIFs (their WebP and MP4 copies stay), the koide 3DGS MP4 (its GIF stays),
  and eight images nothing references; about 29 MB off the checkout.

### Changed

- graph_based_slam vendors small_gicp (koide3/small_gicp `v1.0.1-6-gfa0cfc9`,
  MIT) in `graph_based_slam/third_party/small_gicp`, so loop registration
  with `registration_method: SMALL_GICP` and `graph_slam_multisession_join`
  are in every build (source, Docker, binary release). Before, both were
  built only when CMake found an installed small_gicp. The scanmatcher and
  plugin small_gicp methods stay optional.
- The walking backend preset (`lidarslam_mid360_rko_graph_walking.yaml`,
  `--motion walking`) registers loops with small_gicp (correspondences within
  0.5 m) instead of NDT 2 m. Decided by a plan fixed before the runs
  (bench_report chapters 50 and 51): on RTK-SLAM Stadtgarten 1 it scores
  0.482 m SE(3)-aligned checkpoint RMSE against NDT 2 m's 0.510 m and the
  frontend's 0.498 m, runs are byte-identical when repeated, and its loop
  search costs at most 1.34 times NDT's.
- graph_based_slam's small_gicp loop registration keeps the KD trees and
  covariances of its last four targets, with byte-identical results: its bag
  pass on Stadtgarten 2 drops from 15.7 to 11.9 s.
- The RTK-SLAM construction RKO-LIO preset (release gates) turns deskew and
  the static gravity initialisation on: construction_seq2 0.165 → 0.062 m,
  construction_seq1 0.280 → 0.205 m checkpoint RMSE, map thickness
  0.082 → 0.049 and 0.080 → 0.051 m.
- The RTK-SLAM Stadtgarten 2 RKO-LIO preset turns deskew and the static
  gravity initialisation on: 0.296 m checkpoint RMSE (1.594 m with both off on
  the same commit), map thickness 0.072 → 0.044 m with the GNSS preset. The
  coloured map recipe on that trajectory: camera poses 4.20 → 1.09 px held-out
  epipolar error before refinement, held-out colour error 47.6 → 45.1.
- `run_rko_lio_graph_benchmark.sh` keeps its ordinary completion path only.
  `--skip-map-save` still suppresses the graph's own map writes, now through
  `LIDARSLAM_BENCHMARK_NO_MAP_ARTIFACTS`.
- The default CI gate and the product Python test runner no longer exclude
  the removed tests.
- The README, Getting Started (EN/JA) and the map authoring, index,
  comparison and external-validation pages point at the published
  `v0.11.0-humble` and `v0.11.0-jazzy` images.

## 0.11.0 - 2026-11-02

The first published release since v0.10.0. The 0.10.1 release candidate below
was never tagged; its changes ship in 0.11.0. The `newer_college_math_hard`
release profile was not measured for this release (a one-time exception; see
the release notes).

### Added

- **Multi-session join** — `graph_slam_multisession_join` joins several runs
  of one site into one pose graph, with no prior on how they relate. It finds
  Scan Context candidates, registers them with small_gicp, keeps the largest
  set of agreeing transforms and checks the whole map before it joins. It
  refuses ambiguous sets and self-similar places. Each session joins the map
  built so far, the map is re-optimized after each join, and each session's
  self-similarity is stored for later joins. Built only when small_gicp is
  found. See `docs/architecture/multisession-join.md`.
- **`SMALL_GICP` loop registration** — `registration_method: SMALL_GICP`
  registers loops with small_gicp's GICP when the backend is built with it,
  with `gicp_max_correspondence_distance` and
  `gicp_transformation_epsilon`. The default registration is unchanged.
- **Opt-in loop safeguards** — a drift gate (`loop_max_drift_ratio`,
  `loop_max_drift_offset_m`) rejects a loop whose correction exceeds the
  offset plus a share of the frontend travel between its two submaps, and
  `loop_edge_info_constant_trans` / `loop_edge_info_constant_rot` give loop
  edges a fixed information instead of one derived from the fitness. Both
  are off by default.
- **Causal trajectory output** — `graph_slam_offline_runner` can also write
  each pose as a live system would have published it
  (`causal_trajectory_output`).
- **Backend parameter reference** — `docs/graph-slam-parameters.md`, generated
  from the configuration header's comments
  (`scripts/generate_graph_slam_parameter_reference.py`), describes the
  parameters the presets set.

### Changed

- **Loop search on a worker thread** — with loop closure on, the backend used
  to drop input while it searched: 28–29 of 40 submaps on HILTI exp04, 548–551
  of 639 on a MID-360 drive, and a different result on every run. It now
  builds every submap and repeats its output. Runs with loop closure therefore
  differ from v0.10.0.
- **LiDAR-to-base extrinsic for the backend's input cloud** — RKO-LIO
  publishes its scan in the LiDAR frame and its pose in the base frame;
  `extrinsic_lidar2base_quat_xyzw_xyz` places the scan correctly, and the
  RKO-LIO launch passes RKO-LIO's own extrinsic. With a non-identity
  extrinsic (HILTI, for example) backend outputs change: exp04 end to end
  with loop closure went from 105.0 to 49.0 s. With an identity extrinsic
  outputs are byte-identical.
- **Default runs repeat** — the offline node waits for graph_based_slam
  before it publishes (`wait_for_backend`, default `true`), so the backend no
  longer misses the first scans by a different amount on every run.
- **Lower memory, same outputs** — graph snapshots share the submap clouds;
  stored scans are packed and compressed with zstd (lossless, through
  `zstd_vendor`; uncompressed without zstd); a map too large for one pass is
  saved in z slabs without assembling it (`map_save_points_per_pass`, default
  2,000,000; smaller maps are saved as before); and the offline runner keeps
  its submap clouds compressed. Sum of per-process peaks on Oxford Spires
  Keble 05: 3,028 → 1,849 MiB; offline runner peak on KITTI 05: 4.83 →
  1.74 GB. Outputs are byte-identical.
- **Faster backend, same outputs** — the final map refinement is 2.4 times
  faster with bitwise-identical results, and the loop search builds its
  source cloud only when a query has candidates.
- **Thirdparty/rko_lio** moves to `7e91814`, which carries the frontend keys
  the `configs/sota_v7` configurations set. Every added key is opt-in; the
  shipped settings produce byte-identical trajectories.
- **`lidarslam-map` usability** — `doctor` reports progress while it reads a
  large bag; `start` checks the output's free space before writing anything;
  a workspace sourced from elsewhere is accepted, and the source launcher
  notes when the installed runtime differs from the checkout; copy-ready
  commands work when pasted from a source checkout; the top-level help is
  aligned.

### Fixed

- **`lidarslam-map` on maps made by `start`** — `edit` refused every such map
  because of the `.ros_log/latest` symlink, and `view` and `inspect` reported
  a session directory as an incomplete map. `inspect`'s next steps are now
  product commands.
- **First start after a boot** — RKO-LIO's startup timeout is 120 s instead of
  30 s, the concise runner keeps the child's stderr in `map_workflow.log`, and
  the diagnosis names a startup timeout.
- **Launch** — `wait_for_output_subscribers:=true` with the default
  `subscriber_wait_timeout_sec` (`30.0`) stopped the offline node from
  starting, because the barrier accepts whole seconds only; the timeout is
  now rounded up. `ros2 launch --launch-prefix` now reaches the offline node.

### Evidence

- **First independent first-map validation** — an outside user completed
  Docker First Map with the published `v0.10.0-humble` image and attached a
  PASS receipt ([#576](https://github.com/rsasaki0109/lidar_slam_ros2/issues/576)).
  It is the first entry in `docs/evidence/external-first-map-validations.json`,
  so the v1.0 external-adoption gate stands at 1/3. Its one finding, a slow
  download of the 517 MB Zenodo bag, is already documented in Docker First Map.

## 0.10.1 - 2026-10-14 (candidate)

The next release candidate after v0.10.0. It is not tagged; v0.10.0 is the
latest published release.

### Added

- **`--motion walking` for the MID-360 preset** — `lidarslam-map start` and
  `setup` accept `--motion {wheeled,walking}` (default `wheeled`, unchanged).
  `walking` pins `rko_lio_mid360_walking.yaml` and
  `lidarslam_mid360_rko_graph_walking.yaml`, the frontend and loop settings
  adopted on the MapsHD HUMANOID-LIO-DATASET (bench_report chapters 14 to 25:
  APE geometric mean over 8 MapsHD trials, Oxford Keble 05 and HILTI exp04
  0.0908 to 0.0358 m). `sensor_setup.json`
  records `profile.motion`. The localizability and IMU options need the
  maintained RKO-LIO fork; the official binary ignores them.

### Fixed

- **Large maps saved undownsampled** — when a map's bounding box held more
  `map_leaf_size` voxels than PCL VoxelGrid's 32-bit index (about 2.1e9), the
  map was saved with every point, because VoxelGrid returns its input
  unchanged there. On the public MID-360 bag the fork's map just crosses that
  limit and was saved at 5.6 M points (68 MB) instead of 2.1 M (25 MB);
  whether a map near the limit was downsampled depended on its exact extent.
  Such a map is now split at leaf-aligned planes until each part fits and
  downsampled part by part, which keeps every point in the same voxel as one
  VoxelGrid pass. Maps within the limit are saved exactly as before.
- **Official RKO-LIO 0.4.1 and later** — official `rko_lio` 0.4.1 renamed the
  deskewed scan topic from `rko_lio/frame` to `rko_lio/deskewed_scan` and
  publishes the scan in the base frame instead of the LiDAR frame. With the
  official 0.4.2 binary that ROS apt now ships for Humble and Jazzy, the RKO
  profiles saved no map ("initial map is not received"). The RKO launch now
  pins `deskewed_scan_topic` to `/rko_lio/frame`, and graph_based_slam applies
  the LiDAR-to-base extrinsic only to scans that are not already stamped with
  the odometry's base frame, so a non-identity extrinsic is not applied twice.
  Output with the maintained RKO-LIO fork is unchanged.
- **Release workflow** — the release bundle is built before the standalone
  Docker launcher, which writes into the checkout the bundle requires to be
  clean. The v0.10.0 run failed there and was published by a re-run.
- **Documentation** — Getting Started (EN/JA), the README, map authoring and
  external validation pin the published `v0.10.0` images, which carry the
  `lidarslam-map report` handoff and the own-bag `start` contract.
- **GEODE comparison** — v0.10.0 reported RKO-LIO v16's GEODE Hard median (8.5 m,
  against 78.7 m for BIEVR-LIO). The GEODE configurations were developed on those
  sequences, while the rivals ran their own configurations, so these numbers do not
  compare the methods on unseen data. On GEODE's held-out Metro Tunneling bags RKO-LIO
  has no failure, but BIEVR-LIO's median ATE is lower. See `docs/comparison.md`.

### Known limitations

- RTK-SLAM Construction Hall 2 scores 0.165 m against 0.086 m in v0.9.0. It
  comes from RKO-LIO's direction-only gravity residual (rko_lio #14). The
  vector form restores 0.083 m but raises Construction Hall 1 from 0.280 m to
  0.383 m and leaves the four Koide MID-360 handheld sequences within 1 %, so
  it is kept.

## 0.10.0 - 2026-10-13

The first published release since v0.9.0. The 0.9.1 release candidate below
was never tagged; its changes ship in 0.10.0. This release also restores the
default workflow after the sota-v6 recovery merge, and adds camera colouring,
MID-360 presets and the degenerate LiDAR-inertial odometry (tunnel) track.

### Highlights

- **Degenerate LiDAR-inertial odometry on ENWIDE** — radar-less Ouster
  configurations v7–v10 and the `degenerate_lio_sota_v3`/`_v4` profiles. The
  recorded `degenerate_lio_sota_v4` run of v10 completed all 30 runs (ten
  sequences, three identical repetitions) with a median ATE of 0.29 m, the
  lowest of the five methods reproduced on the same bags. The
  `photometric_bump_no_radar` preset ships the v10 options. This is not a SOTA
  claim: the profile also requires a hidden tunnel holdout and one parameter
  set across all sequences.
- **GEODE Hard evaluation** — RKO-LIO device configurations (v16) and the
  profile rivals on all 18 Hard sequences. The Livox device is evaluated on
  its internal IMU, after its Xsens IMU was found 7.5–80 ms off the LiDAR
  clock. RKO-LIO v16 has the lowest median ATE (8.5 m, against 78.7 m for
  BIEVR-LIO); BIEVR-LIO is better on the Livox shield tunnels. See
  `docs/research/geode-hard-evaluation-2026-10.md`.
- **Camera colouring** — refine each camera pose against the LiDAR map before
  colouring; reject sky-coloured samples and refill points seen only against
  the sky (both opt-in); carve the camera bearer's corridor out of the map;
  and score map colours against an independent TLS reference (Oxford Spires).
  Bag images, clouds and CameraInfo now pair by sensor timestamps.
- **MID-360** — a handheld outdoor RKO-LIO profile (scan gaps up to 3 s,
  gravity re-levelling, occluded-scan skipping), an online odometry launch
  with MID-360 defaults (IMU acceleration converted from g), preset maps
  levelled with RKO-LIO's gravity initialization, and a map tilt check in
  diagnosis.
- **Reliability** — the scanmatcher and RKO-LIO TF listeners no longer hang
  when destroyed on Humble, RKO-LIO waits for its own output subscribers,
  the barrier no longer treats a discovery miss as zero subscribers, and the
  source quickstart builds the registration plugin packages it needs.

### Fixed

- **NTU VIRAL accuracy** — the NTU profile deskews with the interval's mean
  angular velocity again. The samplewise gyro deskew that RKO-LIO adopted for
  Go2 had raised the tnp_01 APE from 0.936 to 1.14 m, over its 1.00 m release
  profile; it is now the `samplewise_gyro_deskew` option and stays on by
  default.
- **Release-gate benchmark harness** — `run_rko_lio_graph_benchmark.sh` writes
  provenance again, so the blocking release profiles can pass, and scores
  each run with the reference offset of its base frame and the reference's
  time tolerance. The sota-v6 merge had replaced it with the experimental
  harness, and NTU VIRAL and RTK-SLAM were scored with the wrong lever arms.
- **Product Python suite** — `run_product_python_tests.sh` skips the
  experimental competitive contracts that the default CI gate excludes.
- **Humble CI** — RKO-LIO creates its TF listener only when the extrinsics
  come from TF; a listener destroyed right after construction could hang.
- **Default workflow builds and tests again** — repaired the broken merge in
  `graph_based_slam` (adopt the pimpl component implementation that matches the
  merged header, declare the missing `lidarslam_plugin_interfaces` /
  `lidarslam_default_plugins` / `lidarslam_registration_loader` dependencies,
  bridge the legacy loop registration through `PclRegistrationAdapter`), and
  fixed the latent PCL 1.12/1.14 `fromPCLPointCloud2` and `PYTHONPATH` issues
  so Humble and Jazzy both build and test.
- **Dependency pin** — restored the `rko_lio` 0.3.2 submodule revision (the
  merge had regressed it to an experimental 0.2.0 revision) together with the
  radar/intensity tests.
- **Canonical NDT convergence** — rebased the checked-in parent transition
  patch on the current consumer manifests and refreshed the contract, evidence,
  and recorded SHA-256.
- **Verified workflows** — `docs-site`, `real-data-e2e`, `official RKO-LIO`,
  `install upgrade`, and `bounded filesystem exhaustion` pass; the
  `release-readiness` jobs pass again.

### Changed

- **Lint** — cleared the merged lint debt (missing copyright headers, flake8,
  pep257, lint_cmake) and re-enabled those linters in the default gate. The
  `uncrustify` check stays excluded because the Humble and Jazzy uncrustify
  versions cannot agree on one style for the same source.
- **Repository hygiene** — neutralized the hardcoded `/home/sasaki` and
  `/media/sasaki` absolute paths to `/home/user` and `/media/user`, and rebound
  the fingerprint profile hashes that referenced them.

### Known limitations

- The experimental competitive/benchmark contracts are excluded from the
  default gate until their preregistration receipts are re-frozen with a
  maintained tool.
- No method, this one included, tracks every GEODE Hard sequence; the tunnel
  track makes no SOTA claim.
- The v1.0 gates listed under 0.9.1 remain open.

## 0.9.1 - 2026-08-12

A reliability and first-map usability release candidate. It keeps the v0.9.0
publication immutable while making the supported Docker and source journeys
more guided, recoverable, and reviewable.

### Highlights

- **One coherent beginner surface** — the no-argument `lidarslam-map` home,
  bag-optional doctor, guided `start`, Japanese quickstart, and fixed MID-360
  demo share the same profiles, preflight, verification, and recovery contract.
- **Crash-safe point-cloud handling** — classic scanmatcher VoxelGrid paths and
  graph point-cloud conversions fail closed on unsafe dimensions, overflow,
  invalid leaves, or inconsistent dense clouds while preserving the process and
  its last valid map.
- **Recoverable map lifecycle** — atomic outputs, finalization-only resume,
  session history and comparison, source-preserving edit/merge plans, local 3D
  preview, and privacy-bounded support/first-map receipts keep provenance and
  next actions visible.
- **Fresh-terminal onboarding** — Docker and the exact six-package Humble/Jazzy
  source route now converge on one verified first-map outcome, with installed
  resource checks and public-readiness probes.
- **One-command candidate trial session** — a dedicated observer can start from
  one exact Actions run URL and retain the authenticated handoff, selected
  Docker/source execution, and a hash-bound session receipt under one atomic
  directory. Docker rows bootstrap a content-addressed, recipe-labelled local
  observer image before timing instead of requiring a copied build command.
- **Guided trial readiness** — the same session command now checks the exact
  destination, Ubuntu/ROS row, x86_64 host, measured filesystem, Docker/source
  runtime, source RX counter, human measurements, and isolation confirmation
  without network access or writes. Stable findings distinguish a blocked host
  from runnable non-comparable evidence and print one shell-safe next command.
- **Release identity hardening** — release-bundle CI checks out the exact PR
  head, fetches immutable tags, and refuses historical version reuse. Candidate
  metadata is aligned at 0.9.1 without moving or recreating `v0.9.0`.
- **Reviewable candidate operations** — the GET-only protected-environment
  audit and G0 card now provide a status-specific, copy-ready administrator
  handoff while keeping settings writes, independent approval, and E2 dispatch
  outside the tool.
- **Parse-safe GLIM comparison** — restore the Docker timeout branch in
  `compare_with_glim.sh` and make every tracked shell script pass `bash -n` in
  the maintained docs/product gate.
- **Content-verified GLIM fallback** — replace the path/topic-only trajectory
  cache with a schema-backed identity over bag, effective config, runtime,
  options, and harness bytes; malformed, modified, legacy, or contradictory
  entries now fail closed instead of becoming cross-validation evidence.
- **Exact public-docs identity** — bind the deployed Getting Started bytes,
  canonical route fragments, product version, and Pages workflow to one source
  commit, and keep first-time cohorts closed when that manifest is absent,
  stale, or tampered.
- **Copy-safe clean-host identity** — derive the source commit and both Docker
  digests from one validated published-release report, then recheck those exact
  live identities before a comparable onboarding row can start.
- **Honest adoption evidence** — onboarding and GLIM usability comparisons are
  schema-backed, environment-matched, and fail closed when rows are missing;
  no overall winner is inferred from incomplete evidence.

### Remaining v1.0 gates

- complete the reviewed NDT ownership/rosdistro and main-channel package path;
- capture complete Humble/Jazzy Docker/source onboarding measurements; and
- accept three independent first-map validations. The tracked count remains
  0/3.

## 0.9.0 - 2026-07-30

The stable product-foundation release candidate. It keeps the v0.7 golden
path and adds fail-closed, machine-readable gates for the remaining v1.0
distribution, publication, and external-adoption work.

### Highlights

- **Cross-phase v1 readiness audit** — one schema-validated contract reports
  all ten product gates, verifies evidence paths, release version/tag state,
  and the independent-user ledger, and refuses to describe incomplete work
  as ready.
- **Release-ready binary dependencies** — the official PRBonn RKO-LIO
  `0.3.2-1` binaries passed the pinned Humble/Jazzy installed golden-path E2E,
  and the product now requires `rko_lio >= 0.3.2`. The pinned
  `ndt_omp_ros2 0.1.0` candidate has a read-only preflight that distinguishes
  local readiness, ready-to-tag, partial publication, and completed
  Humble/Jazzy rosdistro state.
- **Package-manager evidence gate** — a manual Humble/Jazzy workflow installs
  exact ROS apt versions, validates every installed CLI/runtime resource,
  detects stale paths across main-to-testing upgrades, and runs the pinned
  real MID-360 map contract. It remains evidence-open until public
  lidarslam packages exist. The live readiness audit now verifies the exact
  immutable-tag/main-channel run identity and both named-distro jobs, so a
  workflow definition or hand-edited gate cannot stand in for execution.
- **Operationally honest distribution** — docs distinguish ROS testing from
  the normal main repository, preserve exact dependency and candidate state,
  and withhold the beginner apt command until main-channel installation
  evidence passes. A post-publication audit validates the stable release,
  tag commit, all image/rollback/promotion records, and every archived bundle
  file hash before publication evidence is accepted.
- **Pre-tag bundle proof** — main CI builds the curated release bundle twice
  from the clean candidate commit, reuses the complete publication verifier
  against both archives, requires byte-identical output, and retains the
  verified rehearsal artifact before an immutable release tag can be created.
- **Actionable benchmark blockers** — every blocking release profile carries
  a tracked remediation command that is printed for `NO_DATA`/`FAIL`. The
  Newer College row now names the official form-gated Maths-Hard sequence and
  its ICP-to-survey-map ground truth instead of incorrectly describing it as
  prism ground truth.
- **Idempotent publication guidance** — the live `ndt_omp_ros2` release audit
  now points at the already-generated Humble/Jazzy rosdistro PRs and explicitly
  forbids recreating the source tag or rerunning Bloom while those PRs remain
  current.
- **Complete default pytest coverage** — every Python test module in
  `graph_based_slam` and `lidarslam` is now registered with CTest, and CI rejects
  future modules that are added without a matching registration.
- **Extensible degeneracy research boundary** — the oriented appearance
  matcher exposes separate intensity/height diagnostics behind default-off
  adapters. Fog and tunnel holdouts did not justify automatic channel
  selection, so no experimental policy became a product default.

### Remaining v1.0 gates

- publish `ndt_omp_ros2 0.1.0`, then the four lidarslam packages, into Humble
  and Jazzy rosdistro;
- wait for required dependencies and lidarslam packages to reach the normal
  ROS apt repository, then capture package-manager install/upgrade evidence;
- publish and verify the first immutable release carrying rollback assets;
- accept three independent first-map validations. The tracked count remains
  0/3.

## 0.7.0 - 2026-07-29

The productization release candidate. It turns the research-oriented workspace
into a documented offline rosbag2-to-Autoware-map product while retaining the
deterministic and map-quality foundations delivered after v0.6.

### Highlights

- **One installed product CLI** — `lidarslam-map doctor`, `run`, `inspect`,
  and optional `view` now drive the same source and installed workflow.
  Machine-readable option contracts, Bash completion, stable exit behavior,
  preflight-v3, diagnosis-v1, and resumable run-manifest-v2 records are tested
  on Humble and Jazzy. Path and duration metavars use one consistent
  vocabulary, completion must exactly match the option contract, and invalid
  names, values, or combinations consistently exit with status 2. Doctor,
  runner help, parser choices, completion, and release bundles share one
  installed maintained-profile registry.
- **Fail-closed map authoring** — output collision, low free space, real
  bounded-filesystem exhaustion, timestamp reversal, process termination, and
  partial-run recovery preserve terminal manifests and actionable diagnosis
  instead of claiming success.
- **Reproducible distribution** — clean-prefix fresh/upgrade parity, versioned
  Humble/Jazzy image automation, SBOM, BuildKit and GitHub provenance,
  digest-only rollback planning, and immutable two-image release promotion
  are release-gated.
- **Real-data operations evidence** — the pinned public MID-360 E2E gate,
  one-hour and eight-hour named-hardware soaks, periodic resource telemetry,
  bounded iteration timeouts, and a real 32 MiB ENOSPC gate publish
  schema-validated evidence.
- **Independent adoption contract** — a structured first-map issue form,
  schema-validated public ledger, and strict readiness command reject
  duplicate reporters, failed runs, and unreviewed evidence while preserving
  an honest 0/3 status before v1.0. Product runs now emit a privacy-bounded
  receipt that binds the final manifest, diagnosis, and map-verifier log
  without publishing geometry or private paths. Receipt parsing accepts the
  verifier's canonical explanatory `RESULT` line and rejects malformed status
  lookalikes.
- **Map-quality and architecture work** — deterministic offline frontend and
  backend runners, event-driven loop search as the only backend path,
  map-quality metrics, offline refinement, stronger Graph SLAM ownership
  boundaries, and opt-in degeneracy/colouring research remain available
  without expanding the beginner surface.

### Current boundaries

- ROS buildfarm packages are not yet released because `ndt_omp_ros2` remains
  outside rosdistro. Official RKO-LIO release entries and binaries now exist
  for Humble and Jazzy, but testing/main versions differ and clean
  installed-E2E compatibility with the maintained fork remains a release
  gate.
- amd64 is the tested image platform; arm64/Jetson remains evaluation tier.
- This is a prerelease candidate. The v1.0 external-adoption gate still
  requires three independent users to complete first-map validation.

## 0.6.0 - 2026-06-12

The deterministic core / ROS shell refactor (roadmap `docs/roadmap/v0.6.md`,
PRs #237–#262). Same bag + same config now provably produces the same map:
the offline runners replay recorded inputs through the same cores the live
nodes use and the release gate enforces byte-identical results across runs.

### Highlights

- **Deterministic offline mapping (backend)** — `graph_slam_offline_runner`
  replays a recorded odometry bag through submap creation, event-driven
  BackendCore loop search and pose-graph optimization with no executor and
  no wall clock; 3 runs are *byte-identical* (loop edges + optimized
  trajectory) on both gate substrates (MID-360: APE 7.262851264566504 to the
  last digit; NTU VIRAL: 9 loop edges, Leica-GT APE 0.8460511516520233).
  Before the refactor the same input produced different loop edges every run.
- **Deterministic offline mapping (frontend)** — `scan_matcher_offline_runner`
  drives the real `ScanMatcherComponent` in lockstep over a raw sensor bag
  (intra-process, drained executor, synchronous map update); 3 runs over the
  full NTU tnp_01 bag are byte-identical (5795 poses, 133 submaps) — the
  first determinism coverage for the scanmatcher (NDT) frontend.
- **Event-driven loop search by default** — the backend now searches loops on
  submap arrival instead of a wall-clock timer; the v0.4
  `deterministic_loop_scheduling` experiment is retired (its negative result
  motivated this architecture). The legacy timer path stays available behind
  `event_driven_loop_search: false` for one release.
- **Release gate hard-enforces determinism** — opt-in
  `--offline-determinism-bag` / `--frontend-determinism-bag` stages in
  `run_release_readiness_checks.sh` fail the gate on any byte-level mismatch.
- **Real bug fixes surfaced by the refactor** — silent ~6% input drop under
  load (QoS KeepLast(1) + best-effort), IMU/GNSS pose-graph edges weighting
  the wrong error blocks (GNSS georeferencing never actually pulled), missing
  tie-breaks in candidate ranking, GNSS yaw alignment with gauge release
  (first georeferenced maps).
- **God objects decomposed with characterization tests** —
  `graph_based_slam_component.cpp` 3421 → 2265 lines behind 19 tested pure
  headers; `scanmatcher_component.cpp` 2140 → 1694 lines with 4 new tested
  pure headers (pose prediction, pose acceptance incl. the tracking state
  machine, IMU processing, map-update policy). Package test count grew to
  1100+.

## 0.5.0 - 2026-06-11

First version aligned across `VERSION`, the four core `package.xml` files and
the release docs, prepared for the first rosdistro (bloom) binary release.
Covers the v0.4 and v0.5 roadmap milestones on top of 0.3.0.

### Highlights

- **Real ground truth on Livox MID-360 (v0.5 milestone)** — all four RTK-SLAM
  sequences measured against total-station checkpoints; the two indoor
  Construction Hall profiles now block releases (pass 0.30 m / 0.55 m), the
  outdoor Stadtgarten pair soaks as report-only with a dedicated outdoor
  preset (`double_downsample: false`: 3.9 m → 0.835 m), and the
  `mid360_vs_glim` cross-validation gate is demoted to a report-only canary.
- **Deterministic loop scheduling (v0.4 milestone)** — opt-in
  `deterministic_loop_scheduling` parameter (default off) that deterministically
  catches up unqueried submaps instead of latest-only wall-clock scheduling;
  the research-track release profiles graduated to blocking gates.
- **One-command everything** — ghcr Docker demo image
  (`docker run ... ghcr.io/rsasaki0109/lidar_slam_ros2:humble` downloads a
  public MID-360 bag and produces the map headless), the beginner bag-to-map
  runner, and the Autoware bundle now emits `lanelet2_map.osm` alongside
  `pointcloud_map/` + `map_projector_info.yaml`.
- **Photoreal map deliverables** — camera-colored point-cloud map flythrough
  (README hero GIF) with occlusion-aware robust colorization, plus the
  documented optional 3DGS pipeline (gsplat, Apache-2.0).
- **rosdistro release prep** — per-package `CHANGELOG.rst`, SPDX
  `BSD-2-Clause` license tags, versions aligned at 0.5.0, and a bloom runbook
  (`docs/rosdistro-release.md`).

## 0.3.0 - 2026-05-25

Public `v3` release. Carries forward the non-GPL default workflow from
`v0.2.x` and adds the Livox MID-360 operator
toolkit, an opt-in STD/BTC-style triangle descriptor research stack with
variance-validated defaults, dogfood wrapper measurement plumbing, and a
user-friendly README.

### Highlights

- **Livox MID-360 operator toolkit** — Jetson-class robot-side recording →
  SLAM → Autoware map workflow with host preflight, bag stamp rewriter,
  recording / production-candidate sessions, public-dataset map runner, and
  dashboards
- **Opt-in STD/BTC-style triangle descriptor stack** — BSD-2 implementation
  with `edge_3d` keypoint extractor for narrow-FOV / indoor, fine-grained ROS
  params, a diagnostic `triangle_descriptor_skip_ransac` flag, and an
  empirically validated MID-360 default of `max_pairs: 16`; default
  `use_triangle_descriptor: false` on every preset
- **Dogfood wrapper measurement plumbing** — frame overrides,
  quiescence-based offline completion, graph-drain wait, corrected-path
  capture, and reference-TUM APE evaluation
- **User-friendly README** — 5-minute "try the public default" path, badges,
  grouped feature lists, progressive disclosure of research tracks

### Research closeout

- 3-dataset variance evidence (NTU / MID-360 / Newer College) shows the
  triangle stack does not yet provide a reproducible APE win — it is opt-in
  research only
- single-run APE claims on MID-360 triangle ablations are unreliable
  (variance can exceed observed |Δ| by 8x); all reports in this release ran
  ≥3 runs with mean ± std + `|Δ|/σ`
- MID-360-specific `+1 m` APE drift was traced to RANSAC compute cost, not
  the act of enabling the pipeline; `max_pairs: 32 → 16` on the MID-360 yaml
  eliminates it (U-shaped sweep; `=16` is the empirical sweet spot)
- full narrative: `docs/research/triangle-stack-2026-05-summary.md`

### Notes

- the recommended public workflow remains `RKO-LIO + graph_based_slam` with
  distance-based loop closure
- the default release path remains non-GPL and focused on pointcloud-map
  generation for Autoware-compatible workflows
- The triangle stack is additive — it does not change the default
  public path

## 0.2.2 - 2026-03-28

Public `v2 beta` patch release focused on release stability and cross-distro CI
consistency.

### Highlights

- fixed Humble/Jazzy style and include-path mismatches that appeared after the
  `0.2.1` release-prep refresh
- kept the public `RKO-LIO + graph_based_slam` workflow, reports, and release
  metadata aligned on `develop`
- validated the patched release scope with green `docs`, `humble`, `jazzy`,
  `release readiness`, and threshold-guard workflows

### Notes

- this is a patch release over `0.2.1`, not a scope expansion
- public defaults and known limits remain unchanged from `0.2.1`

## 0.2.1 - 2026-03-28

Public `v2 beta` refresh focused on map-authoring workflow hardening and
clearer fallback-path positioning.

### Highlights

- GNSS-aware graph optimization now uses covariance-based weighting and has
  real open-data validation for both direct `NavSatFix` bags and Applanix
  sidecar conversion
- packet-path IMU deskew was hardened around `PointCloud2.time` handling and
  validated on real open data with a repeatable matrix report
- save-time dynamic-object filtering now has cross-dataset validation on Leo
  Drive `bag1` and `bag6`, with roughly `50%` saved-point reduction while
  keeping verification `PASS`
- classic-path fallback benchmarking now includes GNSS-only, IMU, and
  velocity-based odom-prior comparisons, with a tracked validation report that
  keeps dataset-specific knobs out of the public default
- exploratory place-recognition work is now explicitly closed out: distance
  remains the public default, while Scan Context, BEV rerank, and SOLiD stay
  opt-in or experimental
- map-authoring reporting and submission-bundle tooling were extended so maps,
  metrics, logs, and focused reports can be packaged and compared more
  repeatably

### Notes

- the recommended public workflow is still `RKO-LIO + graph_based_slam`
- the default release path remains non-GPL and focused on pointcloud-map
  generation rather than full production autonomy stacks

## 0.2.0 - 2026-03-25

Public beta candidate for the `v2` release line.

### Highlights

- recommended default workflow narrowed to permissive-license components
- `RKO-LIO + graph_based_slam` established as the dogfooded default path
- graph backend improved with better adjacent edges, loop dedup, robust kernels,
  multi-candidate validation, and safer state handling
- Autoware-compatible pointcloud-map export hardened with
  `map_projector_info.yaml` and bundle verification
- end-to-end Autoware dogfood flow added:
  `rosbag2 -> SLAM -> map save -> Autoware map loaders -> rviz2`
- benchmark reporting, HTML report generation, and release/readiness gate added
- CI expanded with default workflow checks and release-readiness fixture jobs
- contribution guide, Autoware quickstart, benchmarking guide, and issue
  templates added for external reports
- fixed public Autoware entrypoint added: `scripts/run_autoware_quickstart.sh`
- comparison page and checked-in release notes added for public `v2 beta`
- MID360 current default tuned to `voxel_size=0.5`, `max_range=80.0`,
  `search_submap_num=5`, `loop_edge_dedup_index_window=20`,
  `loop_edge_info_weight=200`

### Notes

- this release is suitable for public beta / developer preview distribution
- the default workflow remains focused on pointcloud-map generation for
  Autoware; lanelet generation is intentionally out of scope
