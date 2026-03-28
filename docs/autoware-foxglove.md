# Autoware Foxglove Viewer

This page documents the optional web viewer path for Autoware-compatible
`pointcloud_map/` bundles using `foxglove_bridge`.

The goal is simple:

- run Autoware's map loaders against a staged map bundle
- expose `/map/pointcloud_map` and `/map/map_projector_info`
- open the result from a browser or Foxglove Desktop

## Fastest Path

Prepare a user-writable Foxglove Bridge prefix once:

```bash
bash scripts/prepare_foxglove_bridge_prefix.sh
```

Then stage an existing `graph_based_slam` output and launch the bridge:

```bash
bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh \
  output/dogfood_rko_lio_autoware_20260324_190734 \
  --foxglove-prefix /tmp/foxglove_bridge_jazzy
```

The script prints a websocket endpoint such as:

```text
ws://127.0.0.1:8765
```

Open Foxglove and connect to that websocket.

## What The Script Checks

Before printing the websocket endpoint, the script waits for:

- `/map/map_projector_info` inside the Dockerized Autoware loaders
- `/map/pointcloud_map` inside the Dockerized Autoware loaders
- `/map/pointcloud_map` on the host ROS graph
- the Foxglove Bridge TCP port to open

## Main Entrypoints

- prepare local bridge prefix:
  `bash scripts/prepare_foxglove_bridge_prefix.sh`
- existing map bundle:
  `bash scripts/run_autoware_pointcloud_map_foxglove.sh /path/to/autoware_map_bundle --foxglove-prefix /tmp/foxglove_bridge_jazzy`
- graph-based SLAM output:
  `bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh /path/to/output_dir --foxglove-prefix /tmp/foxglove_bridge_jazzy`

## Notes

- This is an optional viewer path. The main public quickstart is still
  `bash scripts/run_autoware_quickstart.sh`.
- The bridge path is useful when you want browser-based proof or a shareable
  web visualization, without relying on a local `rviz2` screenshot.
