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
6. **M6, binary wheels.** First releases are source installs that build against Ubuntu's PCL and
   g2o (`apt install libpcl-dev libg2o-dev`, then `pip install`). Prebuilt manylinux wheels need the
   core to stop depending on system PCL (Boost, FLANN); g2o can be built inside the wheel. Scope this
   after M4 by measuring which PCL parts the default pipeline really uses.

## Notes

- Sophus has no Ubuntu package; the plain build fetches it with RKO-LIO's own recipe.
- g2o: the plain build uses `libg2o-dev` (Ubuntu 24.04: 20230806); the ROS build uses
  `ros-<distro>-libg2o` (2020-05-29). M4's checks above gave the same bytes with the two.
- The installed ROS 2 workspace used for the running blind evaluation is not rebuilt by this work.
