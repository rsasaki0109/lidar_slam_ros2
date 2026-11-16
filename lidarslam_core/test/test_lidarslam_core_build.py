"""Keep the ROS-free core build (lidarslam_core/CMakeLists.txt) in step with the ROS packages.

The plain-CMake build mirrors target definitions from graph_based_slam/CMakeLists.txt and
Thirdparty/ndt_omp_ros2/CMakeLists.txt. These checks fail when a core source or a ROS-free unit
test is added on one side only, and when ROS creeps into the plain build.
"""

from __future__ import annotations

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[2]
CORE_CMAKE = REPO_ROOT / 'lidarslam_core' / 'CMakeLists.txt'
GBS_CMAKE = REPO_ROOT / 'graph_based_slam' / 'CMakeLists.txt'
NDT_CMAKE = REPO_ROOT / 'Thirdparty' / 'ndt_omp_ros2' / 'CMakeLists.txt'

# gtests of graph_based_slam that need ROS messages or the ROS client library. They stay in the
# ament build only; every other gtest must also be built by lidarslam_core.
ROS_ONLY_GTESTS = {
    'test_graph_slam_config',  # rclcpp parameters
    'test_gnss_weighting',  # sensor_msgs/NavSatFix
    'test_cloud_input_extrinsic',  # sensor_msgs/PointCloud2
    'test_graph_state_store',  # lidarslam_msgs
    'test_cloud_compression',  # sensor_msgs/PointCloud2
    'test_backend_core',  # sensor_msgs + ROS adapters
    'test_pointcloud_conversion',  # pcl_conversions
    'test_graph_slam_composition',  # links the ROS component
}

CORE_LIBRARIES = (
    'graph_based_slam_small_gicp',
    'graph_slam_application',
    'graph_slam_map_point',
    'graph_slam_io_adapters',
    'graph_slam_cloud_compression',
    'graph_slam_map_export',
    'graph_slam_offline_pipeline',
)
LOADER_CMAKE = REPO_ROOT / 'lidarslam_registration_loader' / 'CMakeLists.txt'
OFFLINE_PIPELINE = REPO_ROOT / 'graph_based_slam' / 'src' / 'offline_pipeline.cpp'
ROS_HEADER = re.compile(
    r'#include\s*[<"](rclcpp|rcl|sensor_msgs|geometry_msgs|nav_msgs|std_msgs|lidarslam_msgs'
    r'|tf2|tf2_ros|tf2_eigen|rosbag2|rosbag2_cpp|pcl_conversions|message_filters|pluginlib)/')


def _strip_comments(text: str) -> str:
    return '\n'.join(line.split('#', 1)[0] for line in text.splitlines())


def _library_sources(cmake: str, name: str) -> list[str]:
    match = re.search(r'add_library\(\s*' + re.escape(name) + r'\s+STATIC\s+([^)]*)\)', cmake)
    assert match, f'add_library({name} STATIC ...) not found'
    return [token.replace('${GBS_ROOT}/', '') for token in match.group(1).split()]


def _ros_gtests() -> set[str]:
    cmake = _strip_comments(GBS_CMAKE.read_text(encoding='utf-8'))
    return set(re.findall(r'ament_add_gtest\(\s*(\w+)', cmake))


def _core_gtests() -> list[str]:
    cmake = _strip_comments(CORE_CMAKE.read_text(encoding='utf-8'))
    match = re.search(r'set\(LIDARSLAM_CORE_GTESTS\s+([^)]*)\)', cmake)
    assert match, 'LIDARSLAM_CORE_GTESTS list not found'
    return match.group(1).split()


def test_every_ros_free_gtest_is_built_without_ros():
    ros = _ros_gtests()
    core = _core_gtests()
    assert len(core) == len(set(core)), 'duplicate entries in LIDARSLAM_CORE_GTESTS'
    assert ROS_ONLY_GTESTS <= ros, 'a ROS-only gtest no longer exists; update ROS_ONLY_GTESTS'
    assert set(core) == ros - ROS_ONLY_GTESTS


def test_core_gtests_include_no_ros_headers():
    offenders = [
        name for name in _core_gtests()
        if ROS_HEADER.search(
            (REPO_ROOT / 'graph_based_slam' / 'test' / f'{name}.cpp').read_text(encoding='utf-8'))
    ]
    assert offenders == []


def test_core_library_sources_match_the_ros_package():
    ros = _strip_comments(GBS_CMAKE.read_text(encoding='utf-8'))
    core = _strip_comments(CORE_CMAKE.read_text(encoding='utf-8'))
    for name in CORE_LIBRARIES:
        assert _library_sources(core, name) == _library_sources(ros, name), name


def test_ndt_omp_sources_match_the_submodule():
    ndt = _strip_comments(NDT_CMAKE.read_text(encoding='utf-8'))
    core = _strip_comments(CORE_CMAKE.read_text(encoding='utf-8'))
    ros_match = re.search(r'add_library\(\s*ndt_omp\s+([^)]*)\)', ndt)
    core_match = re.search(r'add_library\(\s*ndt_omp\s+STATIC\s+([^)]*)\)', core)
    assert ros_match and core_match
    ros_sources = [token for token in ros_match.group(1).split()]
    core_sources = [token.replace('${NDT_OMP_ROOT}/', '') for token in core_match.group(1).split()]
    assert core_sources == ros_sources


def test_plain_build_needs_no_ros():
    core = _strip_comments(CORE_CMAKE.read_text(encoding='utf-8'))
    for forbidden in ('ament', 'rclcpp', 'rosidl', 'sensor_msgs', 'rosbag2', 'catkin',
                      'pcl_conversions', '/opt/ros'):
        assert forbidden not in core, forbidden
    assert (REPO_ROOT / 'lidarslam_core' / 'COLCON_IGNORE').is_file()


def test_registration_loader_sources_match_the_ros_package():
    ros = _strip_comments(LOADER_CMAKE.read_text(encoding='utf-8'))
    core = _strip_comments(CORE_CMAKE.read_text(encoding='utf-8'))
    ros_match = re.search(r'add_library\(\s*\$\{PROJECT_NAME\}\s+SHARED\s+([^)]*)\)', ros)
    core_match = re.search(
        r'add_library\(\s*lidarslam_registration_loader\s+STATIC\s+([^)]*)\)', core)
    assert ros_match and core_match
    prefix = '${LIDARSLAM_REPO_ROOT}/lidarslam_registration_loader/'
    assert [t.replace(prefix, '') for t in core_match.group(1).split()] == \
        ros_match.group(1).split()


def test_offline_pipeline_includes_no_ros_headers():
    # graph_slam_offline_runner's stages; the ROS 2 shell is graph_slam_offline_runner.cpp.
    assert not ROS_HEADER.search(OFFLINE_PIPELINE.read_text(encoding='utf-8'))
