# Golden-path CLI

Phase 1 introduces one repo-local command surface over the existing,
well-tested map-authoring tools:

```bash
./scripts/lidarslam doctor <rosbag2_dir>
./scripts/lidarslam run <rosbag2_dir> --output-dir output/my_map
./scripts/lidarslam inspect output/my_map
```

This is initially a source-checkout interface. It delegates to
`preflight_autoware_map_bag.py`, `run_autoware_map_from_bag.py`, and
`diagnose_autoware_map_run.py`, so the established profile selection and map
verification behavior remain authoritative.

## Commands

`doctor` checks rosbag2 metadata, reports detected topic capabilities, and
selects a compatible maintained profile. Add `--json` for automation.

`run` prints the selected profile and deterministic command before execution.
Use `--dry-run` to inspect the plan without creating the output directory.
The existing runner continues to verify and diagnose completed outputs.

`inspect` classifies an output as `success`, `map_saved`, `verify_failed`,
`runtime_failed`, or `incomplete`. Add `--write` to create the Markdown and
JSON diagnosis artifacts in the output directory.

Each subcommand accepts the same options as its delegated tool:

```bash
./scripts/lidarslam doctor --help
./scripts/lidarslam run --help
./scripts/lidarslam inspect --help
```

## Exit-code contract

| Code | Meaning |
| --- | --- |
| `0` | Command completed successfully |
| `2` | Invalid usage, input, profile, or output path |
| `70` | Internal/tooling error prevented the command from starting |
| other non-zero | Map-workflow exit code, propagated unchanged |

## Installation-name decision

The ROS package already installs a C++ node named `lidarslam`. Replacing that
binary silently would break `ros2 run lidarslam lidarslam` users. Until Phase 2
resolves the installed command and compatibility shim together, the supported
Phase 1 spelling is explicitly `./scripts/lidarslam`.

This staged decision does not add a fourth beginner workflow: the CLI is a
single command surface over the existing own-bag product path.
