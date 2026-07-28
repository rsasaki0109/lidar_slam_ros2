# CLI compatibility and option policy

`lidarslam-map` is the supported product command. This page defines which
parts of its command line are intended to remain compatible through v1 and
how provisional options will be moved without surprising existing users.
The machine-readable inventory is
[`contracts/cli-v1.json`](contracts/cli-v1.json).

## Product surface

The beginner workflow remains three commands:

```bash
lidarslam-map doctor <rosbag2_dir>
lidarslam-map run <rosbag2_dir> --output-dir <dir>
lidarslam-map inspect <output_dir>
```

Research scripts, benchmark runners, ROS launch arguments, and the historical
`ros2 run lidarslam lidarslam` node are outside this CLI contract.

## Stability labels

| Label | Promise |
| --- | --- |
| Stable | The name, accepted value shape, and meaning are v1 compatibility commitments. Additive choices are allowed when they do not change an existing invocation. |
| Provisional | The option works and is tested, but its location or spelling may change before v1. A replacement and compatibility alias must land before removal. |

During v0.9, a stable option cannot be silently renamed, removed, or assigned
a different default meaning. If a security or data-integrity defect requires
a behavior change, the release notes and command output must identify it.

After v1.0, removal of a stable option requires a major release. A deprecated
spelling must continue to work for at least one minor release, emit one
actionable warning to stderr, and preserve its previous exit-code behavior.
Automation must not parse warnings from stdout.

JSON artifact compatibility is governed by the published JSON schemas, not by
this option policy or the repository version.

## Current inventory

| Command | Stable operator options | Provisional options |
| --- | --- | --- |
| `doctor` | `--json` | None |
| `run` | `--profile`, `--output-dir`, `--min-free-space-gib`, `--dry-run`, `--resume` | `--viewer`, advanced viewer plumbing, `--no-verify-map` |
| `inspect` | `--bag`, `--json`, `--write` | None |

`-h`/`--help` is stable for every command. Top-level `--version` is also
stable.

The positional names describe directories deliberately:

- `rosbag2_dir` is the directory containing `metadata.yaml`, never an
  individual `.db3` or `.mcap` storage file;
- `output_dir` is a map-run directory or its terminal bundle.

## Option tiers

The `run` options are ordered by operator intent:

1. **Core:** choose a profile and output directory.
2. **Lifecycle:** plan, reserve storage, or resume post-processing.
3. **Viewer:** open a successful result.
4. **Advanced viewer:** control viewer-specific build and workspace details.
5. **Break glass:** disable an integrity check for diagnosis.

Viewer construction is not map construction. Before v1, viewer behavior
should move to a dedicated `view` command. Existing `run --viewer ...`
invocations must remain as warning-emitting compatibility aliases during the
published deprecation window.

`--no-verify-map` is a diagnostic escape hatch, not a normal performance
option. Before v1 it should gain an explicit replacement that communicates
the verification mode. The old name must remain an alias during the same
deprecation window, and an unverified run must never be described as a
verified success.

## Naming rules for new options

- Use lowercase kebab case.
- End directory paths with `-dir`.
- Include units in numeric names, such as `-secs` or `-gib`.
- Prefer positive behavior and safe defaults. Negative flags are reserved for
  explicit break-glass behavior.
- Reject invalid combinations with exit code `2`; do not silently ignore an
  option.
- Add automation output only through a versioned JSON contract.
- Do not add a beginner-facing command when an existing command owns the
  lifecycle.

Every public addition must update `contracts/cli-v1.json`, command help,
documentation, and tests in the same change. CI compares the manifest with
the flags rendered by each command, so an undocumented option fails the
contract test.

## Planned migration sequence

1. Introduce `lidarslam-map view <output_dir>` using the existing viewer
   implementation.
2. Route `run --viewer ...` through `view` and emit a deprecation warning
   without changing the completed map result.
3. Introduce an explicit verification-mode option and retain
   `--no-verify-map` as its compatibility alias.
4. Freeze help snapshots, exit codes, JSON schemas, and shell completion in
   the Humble and Jazzy installed-CLI checks.

No provisional option is removed merely because the replacement exists.
Removal follows the compatibility window above.
