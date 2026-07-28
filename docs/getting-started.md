# Getting Started

Use this page when you are new to `lidarslam_ros2` and want the shortest path
to a working map.

## Choose A Path

| You have | Run |
| --- | --- |
| Docker only, no ROS 2 workspace | Follow [Docker First Map](#docker-first-map-no-ros-2-workspace) below |
| A rosbag2 directory and a built workspace | `lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map"` |
| A bag, but you are not sure which topics it has | `lidarslam-map doctor /path/to/rosbag2` |
| You want the fixed public demo dataset | `bash scripts/download_ntu_viral_tnp01.sh && bash scripts/run_autoware_quickstart.sh` |

## Docker First Map (No ROS 2 Workspace)

```bash
mkdir -p "$PWD/lidarslam_output"
docker run --rm \
  -e LIDARSLAM_HOST_UID="$(id -u)" \
  -e LIDARSLAM_HOST_GID="$(id -g)" \
  -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:humble
```

The first run downloads the tracked 517 MB MID-360 bag and prints periodic
byte, percentage and transfer-rate updates. The map is written to
`lidarslam_output/mid360_demo`. On Linux, the two ownership variables make the
container return the output directory to your user even if the run fails.
Omit them on platforms where Docker already maps bind-mount ownership.

## 1. Build The Workspace

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If the repository was cloned without submodules:

```bash
git -C src/lidar_slam_ros2 submodule update --init --recursive
```

## 2. Run Your First Bag

```bash
mkdir -p "$PWD/output"
lidarslam-map run /path/to/rosbag2 \
  --output-dir "$PWD/output/my_map" \
  --dry-run
lidarslam-map run /path/to/rosbag2 \
  --output-dir "$PWD/output/my_map"
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

Map generation and viewing have separate exit codes. After a successful run,
open the browser viewer explicitly:

```bash
lidarslam-map view "$PWD/output/my_map" --viewer foxglove
```

Or inspect an existing output directory:

```bash
lidarslam-map inspect output/<run_dir> --write
```

## Common First-Run Problems

| Symptom | Next check |
| --- | --- |
| `metadata.yaml not found` | Pass the rosbag2 directory, not a `.db3` file. |
| No compatible path is recommended | Run `lidarslam-map doctor /path/to/rosbag2` and check for `PointCloud2` plus `Imu`, or `VelodyneScan` plus Applanix topics. |
| Map verification fails | Open `verify_autoware_map.log` and `autoware_map_diagnosis.md` in the output directory. |
| Viewer starts but no map appears | Confirm the run produced `pointcloud_map/` and try the Foxglove path before the full Autoware viewer. |

For the full operator reference, continue with
[Distribution and installed CLI](distribution.md),
[Autoware Quickstart](autoware-quickstart.md) and
[Operator Workflows](workflows.md).
