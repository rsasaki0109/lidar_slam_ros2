# lidarslam (ROS-free core)

The mapping core of [lidar_slam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2) without a
ROS 2 install. ROS 2 stays one way to run it; this directory is the other.

Status: in development (`docs/roadmap/ros-free-core-2026-11.md`). Today the Python package reads
sensor logs (rosbag2 MCAP/SQLite3, ROS 1 bags) into the core's message types; the frontend and
backend drivers follow.

## Python package

```bash
pip install ./lidarslam_core          # from a clone of the repository
python -m lidarslam inspect /path/to/rosbag2
```

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
