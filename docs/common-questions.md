---
title: LiDAR SLAM inputs, ROS 2 requirements, maps, and accuracy
description: Answers about lidar_slam's ROS-free and ROS 2 workflows, supported sensor-log inputs, Autoware map outputs, and the scope of published benchmark comparisons.
---

# Common questions about lidar_slam

`lidar_slam` builds pointcloud maps from recorded sensor data using RKO-LIO
and graph_based_slam. Choose the installation and engine for your input, and
check the output before using it in another system.

## Can I use lidar_slam without installing ROS 2?

**Yes: the fixed Docker demo needs no host ROS 2 workspace, and the ROS-free
core preview supports offline RKO-LIO mapping through `lidarslam-map`.**
These are different installation paths: running a published Docker image does
not install the native Python package on the host.

For the native preview, follow the
[wheel and source-install instructions](https://github.com/rsasaki0109/lidar_slam#without-ros-2-pip-preview).
That section records how to obtain a wheel, its platform requirements, and
the current PyPI publication boundary. Do not assume a wheel is available
from PyPI solely because the repository contains the package.

The default `--engine auto` uses ROS 2 when the `ros2` command is found and
the core package otherwise. Use `--engine core` when explicitly choosing the
core engine. Visual/radar fusion, pluginlib-loaded registration plugins, and
the classic GNSS/Applanix packet profiles still require ROS 2. Built-in core
registrations do not require pluginlib.

Sources: [Docker first map](getting-started.md#docker-first-map-no-ros-2-workspace),
[ROS-free core implementation and verification record](roadmap/ros-free-core-2026-11.md),
and the [input and support contract](product-contract.md#input-contract).

## What sensor data and bag formats can I use?

**The primary RKO-LIO path needs LiDAR point clouds and IMU data, with the
correct timestamps, per-point time fields, and LiDAR/IMU/base calibration.**
A matching message type alone does not establish compatibility.

The documented ROS 2 path accepts a rosbag2 directory with `metadata.yaml`.
The ROS-free package reads rosbag2 MCAP/SQLite3 and ROS 1 bags. Choose the
engine as well as the file format: these formats do not make every sensor
profile interchangeable.

For Velodyne packet input, the core path supports `VelodyneScan + Imu` with
`--engine core`; the separate classic Applanix packet profile still requires
ROS 2. The core RKO-LIO path can also use recorded RTK GNSS anchors when the
selected graph parameter file enables GNSS. Neither feature removes the need
for the bag's calibration and matching parameters.

Start with `lidarslam-map doctor /path/to/rosbag2` for the documented rosbag2
preflight. It reads the input without network access or writes, then reports
compatibility findings and a next action. A bounded clean timestamp sample
is not proof of full-bag monotonicity or correct calibration.

Sources: [input contract and preflight limits](product-contract.md#input-contract),
[required input topics](workflows.md#required-input-topics), and
[ROS-free GNSS and packet verification](roadmap/ros-free-core-2026-11.md).

## What does an Autoware-compatible map include?

**The product path writes `pointcloud_map/` and `map_projector_info.yaml`,
alongside map metadata, trajectories, and verification reports.**
`lanelet2_map.osm` is produced when lanelet generation is enabled and succeeds.

`verify_autoware_map.py` must report `PASS` before the pointcloud bundle is
treated as valid. Generated lanelets are an authoring starting point, not
surveyed road semantics. Review them before use. Bundle verification does
not certify localization reliability or autonomous-driving safety.

Sources: [supported output and validation contract](product-contract.md#supported-outcome)
and [Autoware-compatible map authoring](autoware-map-authoring.md).

## Is lidar_slam more accurate than other LiDAR SLAM systems?

**The published comparisons support conclusions for their stated datasets,
configurations, references, and metrics; they do not establish a universal
winner.** Use the row and experiment record that match your intended input.

A ground-truth accuracy measurement and agreement with another SLAM estimate
are different evidence. Check the reference kind, alignment, trajectory
coverage, solver configuration, runtime conditions, and repetition count
before comparing numbers. Report-only results and release-blocking gates
also have different roles.

The [comparison page](comparison.md) identifies the release snapshot and the
scope of each experiment. The [benchmarking guide](benchmarking.md) explains
how to reproduce the evidence. These recorded results are not a performance
promise for an untested bag or sensor mounting.
