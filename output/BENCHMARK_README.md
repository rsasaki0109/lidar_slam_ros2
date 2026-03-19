# LiDAR Odometry / SLAM Benchmark

Comprehensive benchmark of LiDAR odometry and SLAM systems on the **Newer College math-hard** dataset.

## Dataset

| Item | Value |
|------|-------|
| Dataset | Newer College "math-hard" |
| Path length | 320m (loop closure available) |
| LiDAR | Ouster OS0-128 @ 10Hz |
| IMU | Ouster IMU @ 100Hz |
| Duration | 193 seconds |
| Evaluation | evo_ape with SE(3) Umeyama alignment |

## Results

### LIO (LiDAR-Inertial Odometry)

| Method | License | RMSE (m) | Poses | Notes |
|--------|---------|----------|-------|-------|
| **DLIO** | MIT | **0.070** | 1896 | NanoGICP + jerk IMU model + per-point deskew |
| **RKO-LIO** | MIT | **0.081** | 1930 | KISS-ICP + IMU tight coupling |

### LO (LiDAR-Only)

| Method | License | RMSE (m) | Poses | Notes |
|--------|---------|----------|-------|-------|
| **GenZ-ICP** | MIT | **0.146** | 1841 | planarity_threshold=0.5, point-to-plane adaptive |
| **KISS-ICP** | MIT | **0.440** | 1913 | VoxelHashMap + adaptive ICP |
| **KISS-SLAM** | MIT | **0.434** | 1930 | KISS-ICP + MapClosures |
| small_gicp odom | MIT | 4.977 | 1762 | IncrementalVoxelMap + GICP |
| lidarslam + FAST_GICP + VHM | BSD/MIT | 11.522 | 1199 | VoxelHashMap + constant velocity |
| lidarslam + NDT (baseline) | - | 24.286 | 1887 | Original baseline |

### Cross-validation (no GT, qualitative)

| Dataset | Method | Poses | Status |
|---------|--------|-------|--------|
| MID-360 | KISS-ICP | 2760 | Normal operation |
| MID-360 | GenZ-ICP | 2566 | Normal operation |
| MID-360 | RKO-LIO | 2020 | Normal operation |
| NTU-VIRAL tnp_01 | KISS-ICP | 936 | Normal operation |

## Key Findings

1. **IMU fusion is critical**: LIO methods (0.07-0.08m) outperform LO methods (0.15-0.44m) by 3-6x
2. **KISS-ICP dominates LO**: VoxelHashMap + adaptive threshold + robust kernel = unbeatable simplicity
3. **GenZ-ICP needs tuning**: Default planarity=0.2 fails outdoors. Setting to 0.5 makes it competitive (0.146m)
4. **Processing speed matters**: Methods that drop scans (due to slow processing) perform poorly regardless of algorithm quality

## Tested Methods

### Available (ROS2 + MIT/BSD)

| Method | Type | License | ROS2 | Repo |
|--------|------|---------|------|------|
| KISS-ICP | LO | MIT | Native | PRBonn/kiss-icp |
| KISS-SLAM | LO+LC | MIT | pip | PRBonn/kiss-slam |
| GenZ-ICP | LO | MIT | Native | cocel-postech/genz-icp |
| small_gicp | Registration | MIT | Native | koide3/small_gicp |
| DLIO | LIO | MIT | Native | vectr-ucla/direct_lidar_inertial_odometry |
| RKO-LIO | LIO | MIT | Native | PRBonn/rko_lio |

### License NG (GPL)

| Method | License | Notes |
|--------|---------|-------|
| FAST-LIO2 | GPLv2 | HKU-MARS |
| Faster-LIO | GPLv2 | FAST-LIO derivative |
| LiLi-OM | GPLv3 | KIT |
| MULLS | GPL-3.0 | |

### ROS2 Not Available

| Method | License | Notes |
|--------|---------|-------|
| MAD-ICP | BSD-3 | Active, ROS2 in TODO |
| CT-ICP | MIT | Abandoned (2022) |
| DLO | MIT | ROS1 only, predecessor of DLIO |

## Modifications Made

### lidarslam-ros2
- Non-monotonic timestamp skip in scanmatcher
- VoxelHashMap for spatial local map management
- Adaptive correspondence threshold for all registration methods
- FAST_GICP / FAST_VGICP / SMALL_GICP / SMALL_VGICP integration
- `cloud_queue_depth` parameter
- `small_gicp_odom_node`: Standalone odometry using IncrementalVoxelMap

### graph_based_slam
- Direct Odometry + PointCloud2 input mode (`use_odom_input` parameter)
- Cloud-driven submap generation for LIO frontend sync
- Multi-submap source aggregation for loop detection
- Scan Context loop detection (`use_scan_context` parameter, GPL-free implementation)
- PCD disk cache for memory-efficient submap storage (`use_pcd_cache` parameter)

### Third-party modifications
- GenZ-ICP: Library name collision fix (`libodometry_component.so` → `libgenz_odometry_component.so`)
- RKO-LIO: Added `publish_odom_tf` parameter for TF broadcast control
- DLIO: Added `publish_tf` parameter for TF broadcast control

## How to Run

### KISS-ICP (best LO)
```bash
ros2 launch kiss_icp odometry.launch.py topic:=/os_cloud_node/points use_sim_time:=true
```

### GenZ-ICP (tuned)
```bash
ros2 launch genz_icp odometry.launch.py topic:=/os_cloud_node/points \
  use_sim_time:=true planarity_threshold:=0.5 voxel_size:=0.6 \
  max_points_per_voxel:=3 desired_num_voxelized_points:=3000
```

### RKO-LIO (best LIO with graph_based_slam compatibility)
```bash
# Requires static TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 os_sensor os_imu &
ros2 run rko_lio offline_node --ros-args \
  -p bag_path:=/path/to/bag \
  -p imu_topic:=/os_cloud_node/imu \
  -p lidar_topic:=/os_cloud_node/points \
  -p base_frame:=os_sensor \
  -p publish_odom_tf:=false  # Set false when running with graph_based_slam
```

### DLIO (best accuracy, standalone only)
```bash
ros2 run tf2_ros static_transform_publisher --ros-args -p use_sim_time:=true -- 0 0 0 0 0 0 os_imu imu &
ros2 run tf2_ros static_transform_publisher --ros-args -p use_sim_time:=true -- 0 0 0 0 0 0 os_sensor lidar &
ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
  pointcloud_topic:=/os_cloud_node/points imu_topic:=/os_cloud_node/imu rviz:=false
# WARNING: DLIO MUST run standalone. Even TF publishers + bag recorder cause CPU competition,
# reducing LiDAR reception from 10Hz to ~2Hz, which causes IMU drift and trajectory explosion.
# graph_based_slam integration is not possible with DLIO.
```

### graph_based_slam with LIO frontend
```bash
# RKO-LIO + graph_based_slam with Scan Context loop detection
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 os_sensor os_imu &

ros2 run graph_based_slam graph_based_slam_node --ros-args \
  -p use_odom_input:=true \
  -p submap_distance_threshold:=1.5 \
  -r odom_input:=/rko_lio/odometry \
  -r cloud_input:=/os_cloud_node/points \
  -p use_scan_context:=true \
  -p scan_context_threshold:=0.4 \
  -p use_pcd_cache:=true \
  -p threshold_loop_closure_score:=3.0 \
  -p voxel_leaf_size:=0.5

ros2 run rko_lio offline_node --ros-args \
  -p bag_path:=/path/to/bag \
  -p base_frame:=os_sensor \
  -p publish_odom_tf:=false
```

### Evaluation
```bash
evo_ape tum gt.csv trajectory.tum -a --t_max_diff 0.1
```
