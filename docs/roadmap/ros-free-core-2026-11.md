# ROS-free core (2026-11)

Goal: map a sensor log without a ROS 2 install. ROS 2 stays one option (live nodes, rosbag
replay, RViz), not a requirement. The default offline mapping must give the same bytes with or
without ROS 2.

## Where ROS 2 is today

| Part | ROS-free already | Still needs ROS 2 |
| --- | --- | --- |
| Backend engine (`GraphSlamApplication`, pose graph, loop search, map refinement) | yes, by contract (`docs/graph-slam-architecture.md`) | build (ament), `cloud_compression` / `graph_state_store` keep `sensor_msgs/PointCloud2` |
| Offline backend runner (`graph_slam_offline_runner`) | no | `rosbag2_cpp` input, `rclcpp` parameters, `sensor_msgs` clouds inside |
| Frontend core (`rko_lio::core`) | yes (`RKO_LIO_BUILD_ROS=OFF`) | none |
| Frontend driver (`rko_lio` `offline_node`, `base_node`) | no | `rosbag2_cpp`, `rclcpp` parameters, message conversion |
| Product CLI (`lidarslam-map`) | Python | `ros2 launch`, `ros2 bag record`, `rosbag2_py` |

## Milestones

Each milestone is one reviewable change. Outputs of the ROS 2 path stay byte-identical.

1. **M1, plain-CMake core build.** `lidarslam_core/` builds the backend core libraries, NDT-OMP,
   small_gicp, 3D-BBS and `rko_lio::core` with Ubuntu packages only, and runs the 49 ROS-free
   backend unit tests. A separate workflow (`ros-free-core.yml`) builds it on a runner without
   ROS. `lidarslam_core/test` keeps the plain build in step with `graph_based_slam/CMakeLists.txt`.
2. **M2, sensor-log input without ROS.** A reader for rosbag2 (MCAP and SQLite3) that decodes
   `sensor_msgs/PointCloud2`, `Imu` and `NavSatFix` from CDR without ROS libraries, behind an input
   interface that also takes plain files (point clouds + IMU CSV). Check: decoded messages equal
   `rosbag2_cpp` output on the MID-360 demo bag.
3. **M3, frontend driver without ROS.** Move the offline driver logic of `base_node` (parameters,
   message buffering, extrinsics, outputs) behind plain types so the ROS node and a ROS-free
   executable share it. Check: frontend trajectory byte-identical to the ROS `offline_node`.
4. **M4, backend runner without ROS.** Replace `sensor_msgs::msg::PointCloud2` inside the backend
   with a plain packed-cloud type (same bytes), read parameters from YAML, and feed the runner from
   the M2 input or directly from the M3 frontend. Check: trajectories, loop edges, g2o, map PCD
   byte-identical to `graph_slam_offline_runner`.
5. **M5, product path.** `lidarslam-map start` runs the ROS-free pipeline when ROS 2 is absent;
   a Docker image without ROS; docs say "ROS 2 optional".

## Notes

- Sophus has no Ubuntu package; the plain build fetches it with RKO-LIO's own recipe.
- g2o: the plain build uses `libg2o-dev` (Ubuntu 24.04: 20230806); the ROS build uses
  `ros-<distro>-libg2o` (2020-05-29). M4's byte-identity check must use one g2o for both sides, or
  report the difference.
- The installed ROS 2 workspace used for the running blind evaluation is not rebuilt by this work.
