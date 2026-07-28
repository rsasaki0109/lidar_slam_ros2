# Operational reliability

This page defines how the golden-path runner behaves when an input, workflow,
or post-processing stage fails. It is the Phase 3 evidence ledger; a row is
only marked covered when an automated test exercises the public behavior.

## Failure and recovery matrix

| Failure | Public behavior | Preserved state | Operator action | Coverage |
| --- | --- | --- | --- | --- |
| Missing or corrupt `metadata.yaml`, missing referenced storage file | Exit `2` before workflow launch | Existing output is untouched; a new output is not created | Correct or replace the bag, then rerun `lidarslam-map doctor` | Automated |
| Incompatible topic/profile or PointCloud2 field layout | Exit `2` with available profiles and missing requirements before workflow launch; RKO-LIO requires FLOAT32 XYZ and a supported per-point timestamp field | Existing output is untouched | Choose a compatible profile or fix the input fields | Automated |
| Final or `.partial` output collision | Exit `2`; no overwrite | Both existing paths remain immutable | Inspect the existing run or choose a new output name | Automated |
| Output filesystem below the configured reserve | Exit `2` before workflow launch with required and observed GiB | Existing output is untouched; a new output is not created | Free storage, choose another output filesystem, or set a deliberately sized `--min-free-space-gib` budget | Automated |
| Workflow exits non-zero | Propagate the workflow exit code | Final output, diagnosis, checksums, and schema-v2 manifest with `failed` status | Inspect the diagnosis and launch log; rerun into a new directory after fixing the cause | Automated |
| Storage exhaustion during a manifest or map write | Preserve the previous atomic manifest when possible; return the workflow or runner failure code; diagnose `ENOSPC`, quota, and “No space left on device” signatures | Final or `.partial` evidence that was already durable; incomplete manifest temp files are removed | Preserve evidence, free storage, and rerun into a new output directory | Automated failure injection |
| Operator `Ctrl-C` (`SIGINT`) | Forward `SIGINT` to the isolated workflow process group; force cleanup after ten seconds; exit `130` | Final output and manifest with `interrupted` status and signal reason | Inspect preserved evidence; rerun into a new directory if a map is still required | Automated |
| Service/container stop (`SIGTERM`) | Forward `SIGTERM` to the isolated workflow process group; force cleanup after ten seconds; exit `143` | Final output, diagnosis, checksums, and manifest with `interrupted`, `143`, and `SIGTERM` | Inspect preserved evidence; rerun into a new directory if a map is still required | Automated failure injection |
| Termination after the workflow result is durable but before post-processing completes | Leave the last durable schema-v2 lifecycle stage in the final or `.partial` directory | Original input/software/command identity and map artifacts | Run the same command and output path with `--resume`; SLAM is not rerun | Automated |
| Ambiguous or unsafe resume state | Exit `2`; refuse concurrent or pre-terminal post-processing | Both candidate directories and manifests remain unchanged | Verify no original process is active; resolve the ambiguity manually before retrying | Automated |
| Missing TF connectivity observed in ROS logs | Diagnosis is `runtime_failed` with a TF connectivity hint | Launch log and diagnosis artifacts | Correct calibration/frame configuration and rerun | Automated diagnosis fixture |
| Pinned public MID-360 bag → verified map | Nightly Jazzy workflow runs the installed CLI with exact archive/bag identity and bounded output thresholds | Non-geometry evidence report, manifests, diagnosis, verification, and logs | Inspect the failed assertion and retained evidence; do not move the contract without review | Automated real-data E2E |

`run_manifest.json` is authoritative for terminal workflow status. Diagnosis
uses its `failed` or `interrupted` state even when the workflow stopped before
writing a recognizable ROS log signature.

## Output storage boundary

`lidarslam-map run` requires at least 5 GiB free on the filesystem that will
contain the output directory. It checks the nearest existing parent before
workflow launch and prints the probe path, required GiB, and observed GiB.
This reserve is a deterministic disk-pressure refusal boundary, not an
output-size prediction: large bags and dense maps can require substantially
more.

Override the reserve only after sizing the expected map:

```bash
lidarslam-map run /path/to/rosbag2 \
  --output-dir /data/maps/session-01 \
  --min-free-space-gib 20
```

The value must be finite and greater than zero. When capacity is below the
budget, no output or `.partial` directory is created. Atomic manifest updates
remove an incomplete temporary file on write failure, so the previous durable
manifest is not replaced by truncated JSON.

## Termination boundary

The runner starts the delegated workflow in a separate POSIX process group.
For `SIGINT` and `SIGTERM`, it forwards the same signal to the entire group,
waits up to ten seconds for the group leader, then sends `SIGKILL` to any
remaining group members even if the leader exited first. The leader is always
reaped before terminal post-processing begins.

An external `SIGKILL`, kernel panic, power loss, or storage device loss cannot
be converted into a terminal manifest by user-space code. Such a run may
remain at `workflow_running`; `--resume` deliberately refuses it because an
original process could still own the artifacts. Confirm that no workflow
process is alive, retain the directory as evidence, and start a new output.

## Soak profiles

The operator-only soak harness exercises the same `lidarslam-map run` contract
as the golden path. It is deliberately not a fourth beginner entrypoint.
Choose a fixed one-hour or eight-hour profile and supply budgets tied to a
named machine. The `lidarslam` package declares GNU `time` as a runtime
dependency because it supplies the peak-RSS evidence:

```bash
python3 scripts/run_map_soak.py /data/bags/mid360 \
  --output-root /data/soak/one-hour-20260727 \
  --soak-profile one-hour \
  --hardware-label 'lab-amd64-7950x-64GiB' \
  --map-profile mid360_livox_smoke \
  --max-peak-rss-mib 4096 \
  --max-output-gib 40 \
  --max-dropped-inputs 0 \
  --max-iteration-secs 300 \
  --min-free-space-gib 100 \
  --telemetry-interval-secs 30
```

The harness repeats complete, collision-safe map runs until the profile
duration is reached. Each iteration records its command, exit code, GNU time
wall duration and peak RSS, output size, remaining free space, console log and
raw GNU time report. `--max-iteration-secs` is a required positive wall-time
budget for one complete map run. While an iteration is still running, the
harness samples free space and cumulative output size every 30 seconds by
default. The interval must be positive and cannot exceed 60 seconds.

Every sample is appended to `telemetry_samples` and atomically checkpoints
`soak_report.json`. If free space falls below `--min-free-space-gib` or output
growth exceeds `--max-output-gib`, the harness stops the entire timed process
group with `SIGTERM` and a bounded `SIGKILL` fallback, then attempts to
finalize a failed report. The last successful sample remains available even
when the iteration never produces normal GNU time evidence.

- [`soak-report-v4.schema.json`](schemas/soak-report-v4.schema.json) — current;
  adds the required per-iteration wall-time threshold, maximum observed
  duration, and `iteration_duration_within_budget` terminal check;
- [`soak-report-v3.schema.json`](schemas/soak-report-v3.schema.json)
  — published compatibility schema containing a non-secret machine
  fingerprint, harness revision and checksums, and the exact input/software
  identity copied from each successful product run;
- [`soak-report-v2.schema.json`](schemas/soak-report-v2.schema.json)
  — published compatibility schema for periodic telemetry reports;
- [`soak-report-v1.schema.json`](schemas/soak-report-v1.schema.json)
  — published compatibility schema for iteration-only reports.

Schemas v1, v2 and v3 remain published and immutable, so archived reports
retain a resolvable contract.

A passing v4 report requires both `provenance_recorded` and
`iteration_duration_within_budget`. The first successful
iteration fixes the input and software identities for the whole soak; any
later iteration with a different identity terminates the run as failed. The
machine ID is a SHA-256 fingerprint: private machine identifiers contribute
to stability but are never written to the report. Hostnames, usernames and
network addresses are not collected.

Operator `Ctrl-C` and service `SIGTERM` use the same process-group cleanup and
return `130` and `143`, respectively. Their terminal v4 report has
`interrupted` status. If the filesystem refuses the final atomic update, the
last successfully written running-state sample remains as recovery evidence.

When an iteration reaches `--max-iteration-secs`, the harness records a final
sample, terminates the whole timed process group, reaps its leader, and writes
a terminal v4 report with `failed` status. This converts a live player or
mapper stall into bounded, machine-readable evidence instead of allowing a
named one-hour or eight-hour profile to hang indefinitely.

The drop counter is a conservative count of documented console signatures for
message-filter drops, scan drops and queue/buffer overflow. One line can match
more than one signature, so the total can over-count; a zero only proves those
signatures were absent from the captured logs. A failed iteration, unreadable
telemetry, or exceeded RSS/output/drop budget stops the profile immediately
and preserves failed evidence. The duration gate passes only after the full
3,600 or 28,800 seconds have elapsed.

## Named-hardware execution evidence

The one-hour and eight-hour profiles were executed consecutively on
2026-07-27/28 using one clean merged revision, one fixed rosbag2 identity and
the named Jazzy machine
`sasaki-laptop-i5-1145G7-32GiB-jazzy-native`. Both v4 reports passed all eight
terminal checks.

| Profile | Iterations | Wall time | Longest iteration | Peak RSS | Output | Drops |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| one-hour | 86 / 86 | 3,638.645 s | 43.164 s / 300 s | 209.320 MiB / 1,024 MiB | 1,889,888,063 B / 5 GiB | 0 / 0 |
| eight-hour | 671 / 671 | 28,834.115 s | 45.732 s / 300 s | 210.871 MiB / 1,024 MiB | 14,553,359,036 B / 30 GiB | 0 / 0 |

The [auditable evidence ledger](evidence/real-data-soak-2026-07-28.md)
records the exact commit, input and report hashes, thresholds, free-space
minimums, reproduction commands and scope limitations. This closes the named
one-hour/eight-hour execution row for this hardware/input/profile combination;
it does not establish a universal performance or map-quality guarantee.

## Pre-launch record timestamp gate

Preflight schema v3 streams every record on maintained map-authoring input
topics directly from each sqlite3 storage file. Records are checked per topic
in sqlite message-row insertion order, including across split files. Equal
timestamps are accepted; timestamp reversal detection fails closed on the
first smaller timestamp and reports the
topic, both row ids, both timestamps, and storage-file names. The implementation
uses constant memory and runs before workflow planning or output creation:

```bash
./scripts/lidarslam preflight /path/to/rosbag2
./scripts/lidarslam run /path/to/rosbag2 --dry-run
```

This gate deliberately rejects non-sqlite3 storage until an equivalent
write-order inspection is implemented. It checks rosbag record timestamps, not
message header stamps or per-point timestamp units.

## Open Phase 3 gates

The following readiness rows remain incomplete and must not be inferred from
the termination coverage:

- a bounded-filesystem live exhaustion test in the scheduled real-data
  environment;
- output migration tooling and last-known-good rollback instructions.

See the [pinned real-data E2E contract](real-data-e2e.md) and the
[v0.9 roadmap](roadmap/v0.9.md) for the remaining Phase 3 and v1.0 gates.
