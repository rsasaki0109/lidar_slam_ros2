# Getting Started

Use this page when you are new to `lidarslam_ros2` and want the shortest path
to a working map.

## Choose A Path

| You have | Run |
| --- | --- |
| Docker only, no ROS 2 workspace | Follow [Docker First Map](#docker-first-map-no-ros-2-workspace) below |
| A bag and want an explanation before a long run | `lidarslam-map run /path/to/rosbag2 --guided` |
| A rosbag2 directory and a built workspace | `lidarslam-map run /path/to/rosbag2 --output-dir "$PWD/output/my_map"` |
| A bag, but you are not sure which topics it has | `lidarslam-map doctor /path/to/rosbag2` |
| You want the fixed public first-map demo | `bash scripts/run_first_map_demo.sh` |

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

The Docker image invokes the same `scripts/run_first_map_demo.sh` implementation
used by a sourced source workspace. Both paths use the fixed MID-360 dataset,
the `rko_lio_graph_mid360_preset`, and the same manifest, verifier, diagnosis,
and first-map receipt artifacts.
On Ubuntu 24.04, replace `:humble` with `:jazzy`; the entrypoint and first-map
contract are unchanged.

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

## 2. Run the Fixed First-Map Demo

After the build and `source install/setup.bash`, run one command from the
repository root:

```bash
cd ~/ros2_ws/src/lidar_slam_ros2
bash scripts/run_first_map_demo.sh
```

This downloads the same fixed 517 MB MID-360 bag used by Docker and writes
`output/mid360_demo/`. The command calls `lidarslam-map run` with
`rko_lio_graph_mid360_preset` and prints the paths of the versioned manifest,
verifier log, diagnosis, and privacy-bounded first-map receipt. The source and
Docker first-map paths therefore have one fixed dataset and one output contract.

## 3. Run Your Own Bag

For a human-operated first run, use the guided path. It checks the bag first,
shows the detected LiDAR/IMU topics, the selected preset, the output location,
and waits for confirmation:

```bash
lidarslam-map run /path/to/rosbag2 --guided
```

Use `--yes` when launching from another tool, or use the lower-level `run`
command directly for scripts.

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

## 4. Check The Result

Successful runs should leave these files:

- `pointcloud_map/`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `map_projector_info.yaml`
- `verify_autoware_map.log`
- `run_manifest.json`
- `autoware_map_diagnosis.json`
- `autoware_map_diagnosis.md`
- `first_map_validation_receipt.json`
- `first_map_validation_receipt.md`

The first-map receipt contains a copy-ready verification summary without map
geometry or private paths. At the end of a run, the CLI prints the reviewable
JSON receipt path and a direct link to the Independent First-map Validation
issue form. Both passing and failing reports improve the onboarding path; see
[Independent First-map Validation](external-first-map-validation.md) for the
privacy and acceptance rules.

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
