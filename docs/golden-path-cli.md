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
The runner writes into `<output>.partial` and atomically renames that directory
to the requested output name after the workflow stops. It refuses to start
when either name already exists, so a new run never overwrites prior evidence.
Completed, failed, and interrupted workflows retain their artifacts and
terminal state for diagnosis.

Every non-dry run produces `run_manifest.json`. It records:

- SHA-256 identities for `metadata.yaml`, every referenced rosbag storage file,
  and every output artifact;
- product, core package, Git commit/dirty-state, and ROS distribution identity;
- the selected profile and exact argument vector;
- UTC start/finish timestamps, exit code, terminal status, and diagnosis
  status.

Hashing large bags adds a sequential input read before execution. This is
deliberate: the manifest identifies the data that was actually processed,
rather than only the path where it happened to be stored.

`succeeded` means the workflow exited successfully. When verification is
enabled (the default), a non-`success` diagnosis changes the manifest to
`failed`. With `--no-verify-map`, inspect `output.diagnosis_status` separately;
`succeeded` does not claim that map verification ran.

`inspect` classifies an output as `success`, `map_saved`, `verify_failed`,
`runtime_failed`, or `incomplete`. Add `--write` to create the Markdown and
JSON diagnosis artifacts in the output directory.

## Versioned JSON contracts

Automation should select the schema using `schema_version` and `schema_uri`;
it must not infer compatibility from the repository version.

- [Preflight schema v1](schemas/preflight-v1.schema.json)
- [Diagnosis schema v1](schemas/diagnosis-v1.schema.json)
- [Run manifest schema v1](schemas/run-manifest-v1.schema.json)

Top-level fields are closed within a published schema. A field addition,
removal, type change, or semantic break requires a new schema file and
migration guidance.

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
| `130` | Run was interrupted by the operator |
| other non-zero | Map-workflow exit code, propagated unchanged |

## Installation-name decision

The ROS package already installs a C++ node named `lidarslam`. Replacing that
binary silently would break `ros2 run lidarslam lidarslam` users. Until Phase 2
resolves the installed command and compatibility shim together, the supported
Phase 1 spelling is explicitly `./scripts/lidarslam`.

This staged decision does not add a fourth beginner workflow: the CLI is a
single command surface over the existing own-bag product path.
