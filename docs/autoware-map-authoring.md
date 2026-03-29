# Autoware-Compatible Map Authoring

This page is the shortest product-level summary of how `lidarslam_ros2` is used
to produce `pointcloud_map/` artifacts for Autoware-compatible workflows.

The supported public path is:

- frontend: `RKO-LIO`
- backend: `graph_based_slam`
- output: `pointcloud_map/` plus `map_projector_info.yaml`

## Why Use This Repo For Map Authoring

- non-GPL default path
- pointcloud-map generation is a first-class workflow
- saved-map verification is part of the documented public flow
- optional GNSS georeferencing writes `LocalCartesian` map metadata
- optional save-time dynamic-object cleanup improves map compactness
- benchmark, report, and release-readiness artifacts are tracked in-repo

## Fastest Supported Path

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_autoware_quickstart.sh
```

This is the shortest maintained path from `lidarslam_ros2` to a verified map
bundle opened through Autoware's map loaders.

## Beginner One-Command Path

If you already have a rosbag2 directory and just want the repo to choose the
path for you:

```bash
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
```

Use `--foxglove` to open the saved map in the browser path after the run.

## Preflight An Arbitrary Bag

Before picking a launch path, inspect the bag once:

```bash
python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2
```

The preflight reads `metadata.yaml`, lists the key sensor topics, and prints the
shortest supported next command for the bag.

It also prints a beginner-friendly copy-paste command that uses:

```bash
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
```

If you want the repo to pick and execute the shortest supported path for you,
use the one-shot runner:

```bash
python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2
```

It uses the same preflight decision, runs the recommended public workflow,
verifies the saved `pointcloud_map/`, and writes a diagnosis report next to the
saved map outputs.

For Livox/MID360-style bags, the runner automatically prefers the tracked
MID360 preset instead of the generic public YAMLs.

## What You Get

- `pointcloud_map/` tiles
- `pointcloud_map_metadata.yaml`
- `map_projector_info.yaml`
- `PASS` / `FAIL` map verification via `scripts/verify_autoware_map.py`
- benchmark/report artifacts for the same workflow family

When GNSS is disabled, `map_projector_info.yaml` stays valid with:

```yaml
projector_type: Local
```

When GNSS is enabled and a stable origin is available, the same output becomes:

```yaml
projector_type: LocalCartesian
map_origin:
  latitude: ...
  longitude: ...
```

## Recommended Entrypoints

- bag preflight: `python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2`
- beginner one-command path: `bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2`
- one-shot runner: `python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2`
- quickstart: `bash scripts/run_autoware_quickstart.sh`
- benchmark path: `bash scripts/run_rko_lio_graph_benchmark.sh`
- release gate: `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`
- map cleanup benchmark: `bash scripts/run_dynamic_object_filter_benchmark.sh`
- pointcloud-map verify: `python3 scripts/verify_autoware_map.py <pointcloud_map_dir>`
- map-run diagnosis: `python3 scripts/diagnose_autoware_map_run.py <output_dir>`

## Current Public Position

The current public position of this repository is:

- map authoring for Autoware-compatible pointcloud-map workflows
- tracked benchmark evidence on `NTU VIRAL`, `MID360`, and `Leo Drive`
- save-time cleanup as an optional map-quality / map-size tool
- place-recognition exploration kept opt-in or experimental unless it clearly
  beats the default path

## Related Docs

- [Autoware Quickstart](autoware-quickstart.md)
- [Operator Workflows](workflows.md)
- [Benchmarking And Release Gate](benchmarking.md)
- [Comparison](comparison.md)
- [v0.2.2 Release Notes](releases/v0.2.2.md)
