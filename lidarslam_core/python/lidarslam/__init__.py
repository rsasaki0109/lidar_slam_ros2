"""lidarslam: LiDAR-inertial SLAM without a ROS 2 install.

ROS 2 is one way to run lidar_slam_ros2; this package is the other. See
docs/roadmap/ros-free-core-2026-11.md for what is ready and what follows.
"""

from lidarslam._core import (  # noqa: F401
    Header,
    ImuSample,
    NavSatFix,
    Odometry,
    PackedCloud,
    PointField,
    Stamp,
)

try:
    from importlib.metadata import PackageNotFoundError, version as _version

    __version__ = _version('lidarslam')  # the repository VERSION file, set at build time
except PackageNotFoundError:  # imported from a source tree that is not installed
    __version__ = 'unknown'
