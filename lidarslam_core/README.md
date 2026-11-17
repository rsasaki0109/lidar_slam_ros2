# lidarslam (ROS-free core)

The mapping core of [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) without a
ROS 2 install. ROS 2 stays one way to run it; this directory is the other.

Status: preview (`docs/roadmap/ros-free-core-2026-11.md`). The Python package reads sensor logs
(rosbag2 MCAP/SQLite3, ROS 1 bags), runs the RKO-LIO odometry and the graph_based_slam offline
backend, and installs the `lidarslam-map` product command. Maps of an RKO-LIO profile are
byte-identical to the ROS 2 offline path. Install from source, or from a manylinux wheel that
bundles every C++ library: `.github/workflows/wheels.yml` builds them for Python 3.10-3.13 on
x86_64 and aarch64 (glibc 2.28+) as workflow artifacts (not published). The x86_64 wheel gives
the same map bytes as the source install; aarch64 maps differ in the last bits (GCC fuses
multiply-adds there) and are checked by the test suite.

## Python package

```bash
pip install ./lidarslam_core          # from a clone of the repository (apt packages below)
lidarslam-map doctor                  # map engine: core, no ROS 2 needed
lidarslam-map start /path/to/rosbag2  # verified Autoware map bundle
lidarslam-map start /path/to/day2 --join-map ./output/day1   # join a repeat visit to a stored map
python -m lidarslam inspect /path/to/rosbag2
```

The multi-session join (`lidarslam-map merge --place-recognition`, `start --join-map`) runs on
the package too: `python -m lidarslam backend` replays each run's recorded backend input and
`python -m lidarslam join` takes the parameters of the ROS 2 `graph_slam_multisession_join`
(`--params FILE -p name:=value`) and writes the same files.

`lidarslam-map` is the product command of the ROS 2 install: the package carries its scripts,
schemas and parameter files under `lidarslam/share/lidarslam`, in the tree the ROS 2 package
installs, from the same file lists (`lidarslam/product-runtime-files.txt`,
`lidarslam/product-schema-files.txt`).

```python
from lidarslam.io import SensorLog

with SensorLog('/path/to/rosbag2') as log:
    for record in log.records(['/points', '/imu']):
        print(record.topic, record.log_time_ns, type(record.message).__name__)
```

## C++ core (CMake, no ROS)

```bash
sudo apt install build-essential cmake libeigen3-dev libpcl-dev libg2o-dev libzstd-dev \
  libtbb-dev robin-map-dev libgtest-dev
cmake -S lidarslam_core -B build-core -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j && ctest --test-dir build-core
```
