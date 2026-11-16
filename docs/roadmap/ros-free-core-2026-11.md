# ROS-free core (2026-11)

Goal: map a sensor log without a ROS 2 install, installed with `pip install`. ROS 2 stays one
option (live nodes, rosbag replay, RViz), not a requirement. The default offline mapping must give
the same bytes with or without ROS 2.

## Where ROS 2 is today

| Part | ROS-free already | Still needs ROS 2 |
| --- | --- | --- |
| Backend engine (`GraphSlamApplication`, pose graph, loop search, map refinement) | yes, by contract (`docs/graph-slam-architecture.md`) | build (ament), `cloud_compression` / `graph_state_store` keep `sensor_msgs/PointCloud2` |
| Offline backend runner (`graph_slam_offline_runner`) | yes since M4: its stages are `graphslam::offline::OfflinePipeline` | the ROS 2 shell only (`rosbag2_cpp` input, node parameters, `rclcpp` logging) |
| Frontend core (`rko_lio::core`) | yes (`RKO_LIO_BUILD_ROS=OFF`) | none |
| Frontend driver (`rko_lio` `offline_node`, `base_node`) | no | `rosbag2_cpp`, `rclcpp` parameters, message conversion |
| Product CLI (`lidarslam-map`) | Python | `ros2 launch`, `ros2 bag record`, `rosbag2_py` |

## Milestones

Each milestone is one reviewable change. Outputs of the ROS 2 path stay byte-identical.

1. **M1, plain-CMake core build.** `lidarslam_core/` builds the backend core libraries, NDT-OMP,
   small_gicp, 3D-BBS and `rko_lio::core` with Ubuntu packages only, and runs the 49 ROS-free
   backend unit tests. A separate workflow (`ros-free-core.yml`) builds it on a runner without
   ROS. `lidarslam_core/test` keeps the plain build in step with `graph_based_slam/CMakeLists.txt`.
2. **M2, Python package and sensor-log input without ROS.** A `pip install`-able package
   (scikit-build-core + pybind11 over `lidarslam_core`). Bags are read with `rosbags` (PyPI, pure
   Python: rosbag2 MCAP/SQLite3 and ROS 1 bags) and plain files (point clouds + IMU CSV). Check:
   decoded clouds and IMU samples equal what the ROS path reads from the MID-360 demo bag.
   *Status (2026-11-16):* `pip install ./lidarslam_core` builds `lidarslam._core` (message types)
   and `lidarslam.io.SensorLog` with no ROS environment; `python -m lidarslam inspect <log>` lists
   topics. `lidarslam_core/test/compare_with_rosbag2.py` found 0 differences against `rosbag2_py`
   on HILTI exp14 (MCAP, 740 PointCloud2 + 29,539 Imu) and a GNSS log (16,028 NavSatFix, MCAP and
   SQLite3). The demo bag was not on this machine; HILTI exp14 stood in for it.
3. **M3, frontend driver without ROS.** Move the offline driver logic of `base_node` (parameters,
   message buffering, extrinsics, outputs) behind plain types so the ROS node and a ROS-free
   executable share it. Check: frontend trajectory byte-identical to the ROS `offline_node`.
   *Status (2026-11-16):* the fork has a ROS-free layer `rko_lio::driver` (rko_lio ec2cee5):
   `declare_driver_config` reads all parameters the node read (moved verbatim, now behind a
   `Parameters` interface), `point_cloud.hpp` holds the PointCloud2 readers, and `OfflineDriver`
   registers frames in ThreadedNode's order. The ROS 2 node calls the same functions.
   `python -m lidarslam odometry <log> --params <ROS 2 parameter file>` runs it from the pip
   package. On HILTI exp14 with the G22 parameters, the dump_results trajectory is byte-identical
   (md5 `dba74867…`) for the unmodified node (a6559cd), the refactored node and the pip package,
   and `config.json` is identical for the two nodes. The pip run took 8.3 s and 102 MB peak RSS
   (node: 9.3 s, 144 MB). Visual and radar fusion stay ROS-only (the driver refuses them).
4. **M4, backend runner without ROS.** Replace `sensor_msgs::msg::PointCloud2` inside the backend
   with a plain packed-cloud type (same bytes), read parameters from YAML, and feed the runner from
   the M2 input or directly from the M3 frontend. Check: trajectories, loop edges, g2o, map PCD
   byte-identical to `graph_slam_offline_runner`.
   *Status (2026-11-16):* the backend helpers (cloud conversion, extrinsic, compression) are
   templates over the message type, with ROS-free `graphslam::msg` cloud and odometry types
   (`cloud_message.hpp`; `fromCloudMsg` is `pcl::fromROSMsg`, `poseToAffine` is `tf2::fromMsg`).
   The runner's stages moved into `offline_pipeline.cpp` (`graphslam::offline::OfflinePipeline`,
   no ROS include): parameters are `graphslam::offline::Parameters` (typed values read with
   rclcpp's type rules), messages go to a `Log`, and the input is pushed in recorded order.
   `graph_slam_offline_runner` is now its ROS 2 shell. `lidarslam_core` builds the same pipeline
   and the unchanged registration loader against a pluginlib stand-in (`compat/pluginlib`: the
   host built-in registrations NDT, GICP and SMALL_GICP; no plugin DSOs without ROS), and
   `python -m lidarslam backend <backend_input> --params <graph_based_slam file> --out DIR` runs it
   from the pip package. `lidarslam.params` types parameter files like rcl; on the 51 file/node
   pairs of `lidarslam/param` that rcl parses, names, types and values equal rclpy's.
   Check: every output file is byte-identical between the unmodified develop runner (48e3e055),
   the refactored ROS 2 runner and the pip package (no ROS environment, Ubuntu's libg2o 20230806
   against ROS's 2020.5.29) on MapsHD calibration-room 2 (walking preset: 90 deg extrinsic, 6
   files; with loops, map PCDs, dense, degeneracy and causal outputs, 16 files; NDT 2 m, 8 files)
   and underground-garage 1 (116 submaps, 2 loops, 9 files). The pip run took 9.0 s and 974 MB
   peak RSS (ROS 2 runner: 9.0 s, 959 MB). `trajectory_causal.tum` models the loop worker's delay
   with the measured compute time, so it can differ between any two runs; the other files are
   deterministic with one registration thread.
5. **M5, product path.** `pip install lidarslam` gives `lidarslam-map start <bag>` without ROS 2;
   the ROS 2 packages call the same core; docs say "ROS 2 optional".
   *Step 1 (2026-11-16):* `python -m lidarslam map <log> --odometry-params P --backend-params P
   --out DIR` runs RKO-LIO and the backend in one pass: each registered scan's deskewed cloud and
   odometry go in memory to `OfflinePipeline` as the offline node's `/rko_lio/frame` (x, y, z
   float32, 16-byte points, LiDAR frame) and `/rko_lio/odometry` (odom -> base, covariance,
   twist), and the backend takes the LiDAR extrinsic from the odometry file as the launch file
   does. On HILTI exp14 (G22 odometry, `lidarslam.yaml` backend, refine maps on) the 8 backend
   files and the odometry trajectory are byte-identical to the ROS 2 path (offline node -> `ros2
   bag record` of 739 + 739 messages -> `graph_slam_offline_runner`); the pip run took 12.8 s and
   160 MB peak RSS (ROS 2: 40 s frontend with recording + 7 s backend).
   *Step 2 (2026-11-16):* the live node's map saving (voxel downsampling, planar and dynamic
   filters, grid cells, metadata, `map.pcd`, `map_projector_info.yaml`, `map_bundle.yaml`) moved
   out of the ROS adapter into `map_export.{hpp,cpp}` (no ROS); `map_save` calls it. The offline
   pipeline writes the same bundle and `pose_graph.g2o` from its optimized poses with
   `offline_map_bundle:=true` (default false: its other outputs are unchanged, checked on
   calibration-room 2), and `lidarslam map` turns it on. On HILTI exp14 the 19 files of the
   bundle run (bundle, tiles, trajectories, refinement maps) are byte-identical between the ROS 2
   runner and the pip package, and `verify_autoware_map.py` passes. The live node's own map could
   not be byte-compared: two runs of the unmodified node fed by `ros2 bag play` already differ;
   `test_map_export` pins the moved code (streamed and assembled saves, projector info,
   manifest).
   *Step 3 (2026-11-16):* `lidarslam-map run|start --engine {auto,ros2,core}`. `core` runs
   `scripts/run_rko_lio_graph_core.py` instead of the ROS 2 launch script: the same inputs, the
   odometry parameters assembled as `rko_lio_slam.launch.py` assembles them (launch defaults and
   frames, then the RKO-LIO file), `lidarslam.mapping` with the map bundle, `traj_raw.tum` /
   `traj_corrected.tum`, `lanelet2_map.osm`, and with `--editable` the backend input written as
   rosbag2 (MCAP). `auto` (default) keeps ROS 2 when `ros2` is on PATH. The rest of the product
   (verification, diagnosis, receipt, session, browser preview) is plain Python and unchanged. On
   HILTI exp14: `run --engine core` finished with diagnosis `success`, verify PASS and a PASS
   receipt; `start --engine core` gave a VERIFIED session. Its odometry trajectory is
   byte-identical to the ROS 2 node's, its backend files to the ROS 2 offline runner's, and the
   backend input it records replays in the ROS 2 offline runner to the same 19 files as the
   recorded ROS 2 input. Not covered by the core engine: the GNSS and packet profiles, the bag's
   GNSS in the backend input, and `--join-map` (the multi-session join needs ROS 2). The live
   `start` runs the online graph node, so its maps are not expected to equal these bytes.
6. **M6, `lidarslam-map` from pip, then binary wheels.** First releases are source installs that
   build against Ubuntu's PCL and g2o (`apt install libpcl-dev libg2o-dev`, then `pip install`).
   Prebuilt manylinux wheels need the core to stop depending on system PCL (Boost, FLANN); g2o can
   be built inside the wheel. Scope this by measuring which PCL parts the default pipeline really
   uses.
   *Step 1 (2026-11-16):* `pip install ./lidarslam_core` installs the `lidarslam-map` command. The
   package holds the tree the ROS 2 package installs under `share/lidarslam` (the 57 product
   scripts, the 19 product schemas, `VERSION`, build provenance, bash completion and `param/`),
   from the same file lists, so the product scripts run unchanged; `lidarslam_benchmark_tools`
   carries the product scripts for the browser preview. `lidarslam-map doctor` reports ready with
   the core engine when the `ros2` command is missing (new optional `engine` object in the doctor
   report), and the diagnosis's map tilt check reads the IMU through `lidarslam.io` when
   `rosbag2_py` is missing (same 25,306 samples and 3.76 deg as `rosbag2_py` on HILTI exp14). On
   HILTI exp14, in a Python environment with no ROS 2 installed or sourced, run from outside the
   checkout: `lidarslam-map run` (31.7 s, 158 MB peak RSS) wrote 30 files, 24 of them (map bundle,
   tiles, trajectories, lanelet2, backend input) byte-identical to step 3's run, the other 6 being
   reports that differ in paths, times and provenance only; `lidarslam-map start` gave a VERIFIED
   session whose 29 map and data files equal step 3's session. (Step 3's runs used the checkout's
   launcher, which sources the ROS 2 workspace for the report tools; these did not.)
   *Step 2, measurement (2026-11-16):* with Ubuntu's PCL the built `_core` module loads 84
   shared libraries (VTK 9.1, OpenNI, OpenGL, libusb, pcap, Boost, ...), but it imports only 37
   PCL symbols: PCD read/write and `lzfCompress` (io), `KdTreeFLANN` and `search::KdTree` (kdtree,
   search), `VoxelGrid` and `getMinMax3D` (filters), `IterativeClosestPoint` (registration),
   `PCLBase` and `console::print` (common). VTK and OpenNI come only from how Ubuntu builds
   `libpcl_io`. PCL 1.14.0 from Ubuntu's source package (`pcl_1.14.0+dfsg-1`), built with Debian's
   flags (`-O2`, `PCL_ENABLE_SSE=OFF`, no AVX or `-march`) and only the modules the core needs
   (common, octree, kdtree, search, sample_consensus, filters, features, 2d, io, registration; no
   VTK, OpenNI, pcap, PNG, Qt), took about 20 minutes on 4 cores and 101 MB installed. Built against
   it, `_core` loads 31 libraries; the remaining OpenGL ones come from Ubuntu's libg2o
   (`g2o_opengl_helper`), which a g2o built for the wheel would drop. `lidarslam-map run` on HILTI
   exp14 with it wrote the same 24 data files as step 3's run (map bundle, tiles, trajectories,
   pose graph, lanelet2, backend input), and its diagnosis and verify log equal step 3's after
   path normalization (23.3 s, 139 MB peak RSS). So a manylinux wheel can build PCL (this module
   set), FLANN, g2o (no OpenGL) and the Boost parts PCL io needs, and keep the map bytes. Next:
   a wheel build in CI (cibuildwheel, manylinux_2_28) with those built from source, repaired with
   auditwheel, then the same exp14 byte check on the installed wheel.

## Notes

- Sophus has no Ubuntu package; the plain build fetches it with RKO-LIO's own recipe.
- g2o: the plain build uses `libg2o-dev` (Ubuntu 24.04: 20230806); the ROS build uses
  `ros-<distro>-libg2o` (2020-05-29). M4's checks above gave the same bytes with the two.
- The installed ROS 2 workspace used for the running blind evaluation is not rebuilt by this work.
