# M7 step c results (plan.txt, preregistered 2026-11-17)

Carrier: the M7c commits on develop fa78814c. The `lidarslam` package was built with pip and run
with no ROS environment. The reference used ROS 2 Jazzy with the velodyne_pointcloud overlay
built from ros-drivers/velodyne 56fc178d (diagnostics 81b53fad, angles a96224f9), the revision
of the July Leo Drive evidence.

Input: Leo Drive `all-sensors-bag1` (Autoware's public recording, 36.5 s), the same SQLite file
as the July evidence (SHA-256 `74e59157…ac22`, 2,506,907,648 bytes), unpacked from the published
`.db3.zstd`.

## Checks without ground truth

| Check | Result |
| --- | --- |
| P1 decoder identity, front / left / right VelodyneScan (VLP16, VLP16db.yaml, min_range 0.9, max_range 200, organize_cloud false) | **1,093 of 1,093 scans identical** to velodyne_transform_node's PointCloud2: 364 / 364 / 365 clouds, 5,275,327 / 14,423,710 / 14,522,077 points; header stamp and frame_id, height, width, fields, is_bigendian, point_step, row_step, is_dense and data bytes |
| P2 odometry identity, RKO-LIO in the package on the front packets vs on the node's clouds (same IMU messages and log times) | `odometry_tum.txt` and `odometry.csv` byte-identical (364 poses) |
| P3 PointCloud2 input unchanged | 84 of 84 output files byte-identical, and the 728 recorded backend-input messages identical, between the package Python files before (fa78814c) and after the change (see the deviation below) |

**ACCEPT.**

Reference clouds: the three transform nodes ran on the bag played at 0.25× with the packet
topics overridden to reliable QoS (as the Leo Drive benchmark does). Every scan produced a cloud;
the recorded counts equal the VelodyneScan counts (`tools/ref_record.sh`, `tools/p1_compare.py`).
The package read and decoded the three streams in 4.5 s.

P2 input: bag1's IMU messages and front packet log times, each packet scan replaced by the
node's cloud for it (`tools/p2_build_input.py`). RKO-LIO parameters: `rko_lio_kitti_odometry.yaml`
with the extrinsics from the bag's `/tf_static` (`tools/leo_rko.yaml`). velodyne_pointcloud's
`time` field holds seconds from the scan stamp, 19-77 ms on this bag, so RKO-LIO needs
`lidar_timestamps.force_relative: true`; its automatic check stops at a 1 ms offset.

### Deviation from the plan: P3

The plan named the HILTI exp14 core run. That bag is no longer on this host (disk), so P3 was run
on what the change touches instead: `lidarslam map` (`tools/p3_map.py`) on a PointCloud2 + IMU
log (the P2 node-cloud input), once with the package's Python files from fa78814c and once with
the new ones, with the same `_core` module. The C++ change adds a class and touches no existing
code. The two backend-input MCAP files differ only in 6 bytes, the file name the rosbag2
metadata record carries (`p3_old_input.mcap` / `p3_new_input.mcap`).

## Report only

`python -m lidarslam map` on bag1 (front packets + `/sensing/imu/imu_data`, `tools/leo_rko.yaml`,
`lidarslam.yaml`), no ROS 2: 364 frames, 4.8 s, 217 MB peak memory. Against the Applanix GSOF49
trajectory of the same recording (cross-validation, `write_aligned_trajectory_metrics.py`'s SE3
alignment and matching):

| Trajectory | APE RMSE | pairs |
| --- | ---: | ---: |
| graph (`trajectory_optimized.tum`) | 0.285 m | 586 |
| RKO-LIO odometry (`odometry_tum.txt`) | 0.396 m | 3,639 |
| v0.12.0 release gate, classic packet profile (NDT scanmatcher, ROS 2) | 0.142 m | 571 |

The RKO-LIO parameters are KITTI's, not tuned for this rig.

Not done in this step: `lidarslam-map run|start --engine core` on a packet bag. The product's
preflight and setup wizard take a VelodyneScan bag only for the classic `packet_applanix_smoke`
profile, which needs ROS 2; the RKO-LIO profiles require PointCloud2. The packet path works
through `python -m lidarslam odometry|map` and the Python API (`velodyne.*` parameters).
