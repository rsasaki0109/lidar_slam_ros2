# lidarslam_ros2 Docs

`lidarslam_ros2` is a ROS 2 LiDAR SLAM repository focused on:

- pointcloud-map authoring
- benchmarked map workflows
- Autoware-compatible map loading

The supported public path is:

- frontend: `RKO-LIO`
- backend: `graph_based_slam`
- output: `pointcloud_map/` and `map_projector_info.yaml`

## Start Here

- [Autoware-Compatible Map Authoring](autoware-map-authoring.md)
- [Autoware Quickstart](autoware-quickstart.md)
- [Autoware Foxglove](autoware-foxglove.md)

## Operations

- [Operator Workflows](workflows.md)
- [Benchmarking And Release Gate](benchmarking.md)
- [Comparison](comparison.md)

## Releases

- [v0.2.2](releases/v0.2.2.md)
- [v0.2.1](releases/v0.2.1.md)
- [v0.2.0](releases/v0.2.0.md)

## Local Preview

Build the docs:

```bash
python3 -m mkdocs build --strict
```

Serve them locally:

```bash
python3 -m mkdocs serve
```
