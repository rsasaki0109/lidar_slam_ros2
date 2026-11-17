# velodyne_pointcloud decoder (vendored)

From [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne) at
`56fc178d2dad4b6d38c6a69aeb2435ff75503e52` (BSD-3-Clause, see `LICENSE`), the revision of the
Leo Drive packet evidence (docs/evidence/leo-drive-packet-benchmark-2026-07-30.md):

- `src/rawdata.cpp`, `include/velodyne_pointcloud/rawdata.hpp`: `velodyne_pointcloud/src/lib` and
  `include`, arithmetic unchanged. Changes: the ROS 2 includes are replaced by
  `ros_free.hpp`, `RawData` takes a `CalibrationDocument` instead of a file name, and the timing
  table is no longer printed to stdout.
- `src/calibration.cpp`, `include/velodyne_pointcloud/calibration.hpp`: the same reads in the same
  order, from scalar strings converted as yaml-cpp 0.8 converts them, instead of a `YAML::Node`.
- `params/*.yaml`: the model calibrations of `velodyne_pointcloud/params`, unchanged.
- `include/velodyne_pointcloud/ros_free.hpp`, `datacontainerbase.hpp`: written for lidarslam (the
  parts of rclcpp, velodyne_msgs, angles and DataContainerBase the decoder uses).

The container that fills the cloud (`PointcloudXYZIRT` without frames) is in
`lidarslam_core/src/python_velodyne.cpp`. Checked against velodyne_transform_node in
bench_report/data/ros_free_packets_2026-11-17.
