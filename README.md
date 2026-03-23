lidarslam_ros2
====
ROS 2 SLAM package with a scan-matching frontend and a graph-based SLAM backend with loop closure.

## Features

- **Multiple registration methods**: NDT, GICP, FAST_GICP, SMALL_GICP
- **LIO frontend support**: RKO-LIO and DLIO odometry can feed into graph_based_slam for loop closure
- **GPL-free Scan Context loop detection**: built-in Scan Context descriptor for place recognition without GPL dependencies
- **PCD disk cache**: memory-efficient submap storage that pages point clouds to disk
- **Adaptive correspondence threshold**: automatically adjusts the registration correspondence distance based on an exponential moving average of fitness scores
- **GNSS constraints**: optional NavSatFix integration for georeferenced mapping with pose graph optimization
- **Autoware-compatible map output**: grid-divided PCD maps with metadata for `pointcloud_map_loader`, plus `map_projector_info.yaml` for georeferencing

## Benchmark Results

Newer College math-hard dataset (APE RMSE, meters):

| Method | RMSE |
|---|---|
| RKO-LIO + graph_based_slam loop closure (info=1000) | **0.078 m** |
| RKO-LIO raw | 0.082 m |
| KISS-ICP | 0.440 m |
| lidarslam NDT baseline | 24.286 m |

## RKO-LIO Frontend with Loop Closure

RKO-LIO can be used as a LIO frontend, with `graph_based_slam` providing loop closure on its odometry output.

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Key `graph_based_slam` parameters for this workflow:

| Name | Type | Default | Description |
|---|---|---|---|
| adjacent_edge_info_weight | double | 1000.0 | Information weight for adjacent edges in the pose graph. Higher values trust the LIO odometry more. |
| threshold_loop_closure_score | double | 1.0 | NDT fitness score threshold for accepting a loop closure |
| use_scan_context | bool | false | Enable Scan Context descriptors for loop detection (GPL-free) |
| use_pcd_cache | bool | false | Cache submaps to PCD files on disk to reduce memory usage |

## Creating Maps for Autoware

lidarslam_ros2 can generate point cloud maps compatible with Autoware's `pointcloud_map_loader`.

When map saving is triggered (via loop closure or `ros2 service call /map_save std_srvs/Empty`), the map is automatically divided into grid cells and saved with metadata.

Output structure:
```
pointcloud_map/
  pointcloud_map_metadata.yaml   # Grid cell metadata for Autoware
  0_0.pcd                        # Grid cell PCD files (binary compressed)
  0_20.pcd
  20_0.pcd
  ...
map.pcd                          # Full map (single file, for visualization)
```

Key parameters for map output:

| Name | Type | Default | Description |
|---|---|---|---|
| map_save_dir | string | "." | Output directory for map files |
| map_grid_size_x | double | 20.0 | Grid cell width [m] |
| map_grid_size_y | double | 20.0 | Grid cell height [m] |
| map_leaf_size | double | 0.2 | Voxel downsampling resolution [m] |

When `use_gnss:=true`, GNSS position constraints are added to the pose graph and the map origin is saved as `map_projector_info.yaml` for Autoware's `map_projection_loader`.

To use the map in Autoware, copy the `pointcloud_map/` directory and `map_projector_info.yaml` to your Autoware map directory.

## requirement to build
You need  [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) for scan-matcher

clone
(If you forget to add the --recursive option when you do a git clone, run `git submodule update --init --recursive` in the lidarslam_ros2 directory)
```
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y
```
build
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## io

### frontend(scan-matcher) 
- input  
/input_cloud  (sensor_msgs/PointCloud2)  
/tf(from "base_link" to LiDAR's frame)  
/initial_pose  (geometry_msgs/PoseStamed)(optional)  
/imu  (sensor_msgs/Imu)(optional)  
/tf(from "odom" to "base_link")(Odometry)(optional)  

- output  
/current_pose (geometry_msgs/PoseStamped)  
/map  (sensor_msgs/PointCloud2)  
/path  (nav_msgs/Path)  
/tf(from "map" to "base_link")  
/map_array(lidarslam_msgs/MapArray)

### backend(graph-based-slam)
- input  
/map_array(lidarslam_msgs/MapArray)
- output  
/modified_path  (nav_msgs/Path)   
/modified_map  (sensor_msgs/PointCloud2)  

- srv  
/map_save  (std_srvs/Empty)　　

## how to save the map

`pose_graph.g2o` and `map.pcd` are saved in loop closing or using the following service call.

```
ros2 service call /map_save std_srvs/Empty
```

## params

- frontend(scan-matcher) 

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT"|"NDT", "GICP", "FAST_GICP", or "SMALL_GICP"|
|ndt_resolution|double|5.0|resolution size of voxel[m]|
|ndt_num_threads|int|0|threads using ndt(if `0` is set, maximum alloawble threads are used.)(The higher the number, the better, but reduce it if the CPU processing is too large to estimate its own position.)|
|gicp_corr_dist_threshold|double|5.0|the distance threshold between the two corresponding points of the source and target[m]|
|adaptive_correspondence_threshold|bool|false|automatically adjust correspondence distance using an EMA of fitness scores|
|trans_for_mapupdate|double|1.5|moving distance of map update[m]|
|vg_size_for_input|double|0.2|down sample size of input cloud[m]|
|vg_size_for_map|double|0.05|down sample size of map cloud[m]|
|use_min_max_filter|bool|false|whether or not to use minmax filter|
|scan_max_range|double|100.0|max range of input cloud[m]|
|scan_min_range|double|1.0|min range of input cloud[m]|
|scan_period|double|0.1|scan period of input cloud[sec](If you want to compound imu, you need to change this parameter.)|
|map_publish_period|double|15.0|period of map publish[sec]|
|num_targeted_cloud|int|10|number of targeted cloud in registration(The higher this number,  the less distortion.)|
|set_initial_pose|bool|false|whether or not to set the default pose value in the param file|
|initial_pose_x|double|0.0|x-coordinate of the initial pose value[m]|
|initial_pose_y|double|0.0|y-coordinate of the initial pose value[m]|
|initial_pose_z|double|0.0|z-coordinate of the initial pose value[m]|
|initial_pose_qx|double|0.0|Quaternion x of the initial pose value|
|initial_pose_qy|double|0.0|Quaternion y of the initial pose value|
|initial_pose_qz|double|0.0|Quaternion z of the initial pose value|
|initial_pose_qw|double|1.0|Quaternion w of the initial pose value|
|publish_tf|bool|true|Whether or not to publish tf from global frame to robot frame|
|use_odom|bool|false|whether odom is used or not for initial attitude in point cloud registration|
|use_imu|bool|false|whether 9-axis imu(Angular velocity, acceleration and orientation must be included.) is used or not for point cloud distortion correction.(Note that you must also set the `scan_period`.)|
|debug_flag|bool|false|Whether or not to display the registration information|


- backend(graph-based-slam)

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT"|"NDT", "GICP", "FAST_GICP", or "SMALL_GICP"|
|ndt_resolution|double|5.0|resolution size of voxel[m]|
|ndt_num_threads|int|0|threads using ndt(if `0` is set, maximum alloawble threads are used.)|
|voxel_leaf_size|double|0.2|down sample size of input cloud[m]|
|loop_detection_period|int|1000|period of searching loop detection[ms]|
|threshold_loop_closure_score|double|1.0| fitness score of ndt for loop closure|
|distance_loop_closure|double|20.0| distance far from revisit candidates for loop closure[m]|
|range_of_searching_loop_closure|double|20.0|search radius for candidate points from the present for loop closure[m]|
|search_submap_num|int|2|the number of submap points before and after the revisit point used for registration|
|num_adjacent_pose_cnstraints|int|5|the number of constraints between successive nodes in a pose graph over time|
|adjacent_edge_info_weight|double|1000.0|information matrix weight for adjacent edges (higher = trust odometry more)|
|use_scan_context|bool|false|enable Scan Context loop detection (GPL-free)|
|use_pcd_cache|bool|false|cache submaps to PCD files on disk to reduce memory|
|use_save_map_in_loop|bool|true|Whether to save the map when loop close(If the map saving process in loop close is too heavy and the self-position estimation fails, set this to `false`.)|
|map_save_dir|string|"."|output directory for map files|
|map_grid_size_x|double|20.0|grid cell width for Autoware-compatible map division [m]|
|map_grid_size_y|double|20.0|grid cell height for Autoware-compatible map division [m]|
|map_leaf_size|double|0.2|voxel downsampling resolution for saved map [m]|
|use_gnss|bool|false|enable GNSS position constraints in pose graph (subscribes to /gnss/fix)|
|gnss_info_weight|double|1.0|information weight for GNSS position constraints|

## demo
### GLIM MID360 sample

The recommended sample dataset for this branch is the official GLIM MID360 rosbag:

- download: `https://doi.org/10.5281/zenodo.14841855`
- file: `rosbag2_2024_04_16-14_17_01.zip`
- after extraction, use the extracted rosbag directory as `<bag_dir>`
- points topic: `/livox/lidar`
- imu topic: `/livox/imu`
- default sample path on this branch: MID360 tuned frontend with `use_imu: false` and `graph_based_slam` disabled

Run:

```bash
bash scripts/run_bag_demo.sh \
  --bag <bag_dir> \
  --points-topic /livox/lidar \
  --imu-topic /livox/imu \
  --robot-frame-id livox_frame \
  --points-frame-id livox_frame
```

Compare against GLIM:

```bash
bash scripts/compare_with_glim.sh \
  --bag <bag_dir> \
  --points-topic /livox/lidar \
  --imu-topic /livox/imu
```

Current sample result:

- GLIM path length: `1077.12 m`
- lidarslam path length: `1077.58 m`
- aligned comparison: `APE RMSE = 0.457 m`, `APE median = 0.395 m`, `APE max = 1.078 m`

<img src="./lidarslam/images/mid360_glim_compare_xy.svg" width="960px">

<img src="./lidarslam/images/mid360_glim_compare_error.svg" width="960px">

<img src="./lidarslam/images/mid360_glim_attitude_compare.png" width="960px">

<img src="./lidarslam/images/mid360_glim_map_compare.png" width="960px">

## Used Libraries 

- Eigen
- PCL(BSD3)
- g2o(BSD2 except a part)
- [ndt_omp](https://github.com/koide3/ndt_omp) (BSD2)

## Related packages 

- [li_slam_ros2](https://github.com/rsasaki0109/li_slam_ros2) (BSD2)
