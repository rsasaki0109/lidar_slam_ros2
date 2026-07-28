# Golden-path CLI

The product exposes one command surface over the existing, well-tested
map-authoring tools. After a source build and `source install/setup.bash`, use:

```bash
lidarslam-map doctor <rosbag2_dir>
lidarslam-map run <rosbag2_dir> --output-dir output/my_map
lidarslam-map inspect output/my_map
```

The equivalent repo-local spelling is:

```bash
./scripts/lidarslam doctor <rosbag2_dir>
./scripts/lidarslam run <rosbag2_dir> --output-dir output/my_map
./scripts/lidarslam inspect output/my_map
```

It delegates to
`preflight_autoware_map_bag.py`, `run_autoware_map_from_bag.py`, and
`diagnose_autoware_map_run.py`, so the established profile selection and map
verification behavior remain authoritative.

## Commands

`doctor` checks rosbag2 metadata, reports detected topic capabilities, and
selects a compatible maintained profile. For RKO-LIO profiles it also reads
the first selected `PointCloud2` record and requires FLOAT32 XYZ fields plus a
supported per-point timestamp field (`t`, `timestamp`, `time`, or `stamps`).
If the record cannot be inspected or does not satisfy that layout, no RKO-LIO
profile is recommended. Add `--json` for automation. This field-layout check
does not certify timestamp units, monotonicity, calibration, or TF.

`run` prints the selected profile and deterministic command before execution.
Use `--dry-run` to inspect the plan without creating the output directory.
The runner writes into `<output>.partial` and atomically renames that directory
to the requested output name after the workflow stops. It refuses to start
when either name already exists, so a new run never overwrites prior evidence.
It also refuses to launch when the output filesystem has less than 5 GiB free.
Use `--min-free-space-gib <GiB>` to set a larger reserve after sizing the
expected map; the configured reserve is a refusal boundary, not an output-size
prediction.
Completed, failed, and interrupted workflows retain their artifacts and
terminal state for diagnosis. The runner isolates the workflow in its own
process group. `Ctrl-C` (`SIGINT`) and service/container termination
(`SIGTERM`) are forwarded to that group, bounded by a ten-second graceful
shutdown before forced cleanup. The manifest is then completed with
`interrupted` status, exit `130` or `143`, and the terminating signal in
`lifecycle.last_error`.

If the workflow has stopped but verification, finalization, diagnosis, or
checksum generation was interrupted, resume only those terminal
post-processing stages:

```bash
./scripts/lidarslam run <rosbag2_dir> \
  --output-dir output/my_map \
  --resume
```

Resume requires the original bag, software checkout, ROS distribution,
profile, options, and output path. Their checksums and identities must match
the manifest exactly. It accepts either `output/my_map.partial` before the
atomic rename or `output/my_map` after the rename, but never both. A manifest
at `initialized` or `workflow_running` is refused because the original process
may still be active. An advisory lock also prevents two post-processing
processes from owning the same output.

Resume never starts the SLAM workflow again.

Every non-dry run produces a schema-v2 `run_manifest.json`. It records:

- SHA-256 identities for `metadata.yaml`, every referenced rosbag storage file,
  and every output artifact;
- product, core package, Git commit/dirty-state, and ROS distribution identity;
- the selected profile and exact argument vector;
- UTC start/finish timestamps, exit code, terminal status, and diagnosis
  status;
- the durable lifecycle stage, resume count, runner exit code, and last
  post-processing error.

Hashing large bags adds a sequential input read before execution. This is
deliberate: the manifest identifies the data that was actually processed,
rather than only the path where it happened to be stored.

`execution.exit_code` is always the map-workflow exit code.
`lifecycle.runner_exit_code` is the overall runner result, including
verification and post-processing. `succeeded` means both the workflow and
enabled verification completed successfully. When verification is enabled
(the default), a non-`success` diagnosis changes the manifest to `failed`.
With `--no-verify-map`, inspect `output.diagnosis_status` separately;
`succeeded` does not claim that map verification ran.

`inspect` classifies an output as `success`, `map_saved`, `verify_failed`,
`runtime_failed`, or `incomplete`. Add `--write` to create the Markdown and
JSON diagnosis artifacts in the output directory.

## Option tiers

The command help groups options by operator intent. Existing option names
remain compatible; the tiers make the stable beginner surface distinct from
viewer plumbing and safety overrides.

| Tier | Options | Use |
| --- | --- | --- |
| Doctor output | `doctor --json` | Emit the versioned preflight contract for automation |
| Map selection and output | `run --profile`, `run --output-dir` | Select a maintained profile or an explicit artifact directory |
| Safety and lifecycle | `run --min-free-space-gib`, `run --dry-run`, `run --resume` | Refuse unsafe starts, inspect a plan, or finish terminal post-processing |
| Viewer | `run --viewer {none,autoware,foxglove}` | Open an optional viewer after a successful run |
| Advanced viewer | `run --autoware-core-dir`, `run --work-dir`, `run --viewer-run-dir`, `run --viewer-rebuild`, `run --auto-exit-secs` | Control viewer build/runtime details; these require a non-`none` viewer |
| Advanced safety override | `run --no-verify-map` | Diagnostic-only run without required map verification; success does not claim verification |
| Inspection context/output | `inspect --bag`, `inspect --json`, `inspect --write` | Add source-bag context, choose machine output, or persist diagnosis files |

Viewer-specific options that would otherwise be ignored are rejected with exit
code `2`. In particular, `--autoware-core-dir` requires
`--viewer autoware`; the other advanced viewer options require either
`--viewer autoware` or `--viewer foxglove`.

## Versioned JSON contracts

Automation should select the schema using `schema_version` and `schema_uri`;
it must not infer compatibility from the repository version.

- [Preflight schema v3](schemas/preflight-v3.schema.json) — current; adds
  full sqlite write-order record timestamp inspection
- [Preflight schema v2](schemas/preflight-v2.schema.json) — adds record-level
  PointCloud2 field inspection
- [Preflight schema v1](schemas/preflight-v1.schema.json)
- [Diagnosis schema v1](schemas/diagnosis-v1.schema.json)
- [Run manifest schema v1](schemas/run-manifest-v1.schema.json)
- [Run manifest schema v2](schemas/run-manifest-v2.schema.json) — current;
  adds resumable lifecycle state

Top-level fields are closed within a published schema. A field addition,
removal, type change, or semantic break requires a new schema file and
migration guidance.

Preflight v1/v2 and run manifest v1 remain published for existing artifacts.
Preflight v1 only reports metadata-level topic compatibility. Run manifest v1
predates durable lifecycle stages, so it can be inspected but cannot be
resumed safely.

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
| `130` | Run was interrupted by the operator (`SIGINT`) |
| `143` | Run was terminated by a service or container manager (`SIGTERM`) |
| other non-zero | Map-workflow exit code, propagated unchanged |

## Installed names and compatibility

The ROS package's historical C++ node remains
`ros2 run lidarslam lidarslam`. It is not replaced by the product CLI.

The installed product command is `lidarslam-map`. ROS tooling can invoke the
same command surface through
`ros2 run lidarslam lidarslam-cli <command> ...`. The repo-local
`./scripts/lidarslam` spelling remains useful before installation.

These are aliases for the same own-bag entrypoint, not additional beginner
workflows. See [Distribution and installed CLI](distribution.md) for package
contents, platform support, and the binary-release boundary.
