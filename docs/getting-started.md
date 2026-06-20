# Getting Started

Use this page when you are new to `lidarslam_ros2` and want the shortest path
to a working map.

## Choose A Path

| You have | Run |
| --- | --- |
| Docker only, no ROS 2 workspace | `docker run --rm -v "$PWD/lidarslam_output:/lidarslam_ws/output" ghcr.io/rsasaki0109/lidar_slam_ros2:humble` |
| A rosbag2 directory and a built workspace | `bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2` |
| A bag, but you are not sure which topics it has | `python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2` |
| You want the fixed public demo dataset | `bash scripts/download_ntu_viral_tnp01.sh && bash scripts/run_autoware_quickstart.sh` |

## 1. Build The Workspace

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If the repository was cloned without submodules:

```bash
git -C src/lidar_slam_ros2 submodule update --init --recursive
```

## 2. Run Your First Bag

```bash
cd ~/ros2_ws/src/lidar_slam_ros2
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --dry-run
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
```

The dry run prints the selected public workflow before anything starts. The real
run writes the map under `output/` by default.

## 3. Check The Result

Successful runs should leave these files:

- `pointcloud_map/`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `map_projector_info.yaml`
- `verify_autoware_map.log`
- `autoware_map_diagnosis.md`

Open the saved map in a browser viewer:

```bash
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --foxglove
```

Or inspect an existing output directory:

```bash
python3 scripts/diagnose_autoware_map_run.py output/<run_dir> --write
python3 scripts/verify_autoware_map.py output/<run_dir>/pointcloud_map
```

## Common First-Run Problems

| Symptom | Next check |
| --- | --- |
| `metadata.yaml not found` | Pass the rosbag2 directory, not a `.db3` file. |
| No compatible path is recommended | Run `python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2` and check for `PointCloud2` plus `Imu`, or `VelodyneScan` plus Applanix topics. |
| Map verification fails | Open `verify_autoware_map.log` and `autoware_map_diagnosis.md` in the output directory. |
| Viewer starts but no map appears | Confirm the run produced `pointcloud_map/` and try the Foxglove path before the full Autoware viewer. |

For the full operator reference, continue with
[Autoware Quickstart](autoware-quickstart.md) and
[Operator Workflows](workflows.md).
