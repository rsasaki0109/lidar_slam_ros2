# Operational reliability

This page defines how the golden-path runner behaves when an input, workflow,
or post-processing stage fails. It is the Phase 3 evidence ledger; a row is
only marked covered when an automated test exercises the public behavior.

## Failure and recovery matrix

| Failure | Public behavior | Preserved state | Operator action | Coverage |
| --- | --- | --- | --- | --- |
| Missing or corrupt `metadata.yaml`, missing referenced storage file | Exit `2` before workflow launch | Existing output is untouched; a new output is not created | Correct or replace the bag, then rerun `lidarslam-map doctor` | Automated |
| Incompatible topic/profile selection | Exit `2` with available profiles and missing requirements | Existing output is untouched | Choose a compatible profile or fix the input | Automated |
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
waits up to ten seconds, then sends `SIGKILL` if the group leader has not
exited. The leader is always reaped before terminal post-processing begins.

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
  --telemetry-interval-secs 30
```

The harness repeats complete, collision-safe map runs until the profile
duration is reached. Each iteration records its command, exit code, GNU time
wall duration and peak RSS, output size, remaining free space, console log and
raw GNU time report. While an iteration is still running, it samples free
space and cumulative output size every 30 seconds by default. The interval
must be positive and cannot exceed 60 seconds.

Every sample is appended to `telemetry_samples` and atomically checkpoints
`soak_report.json`. If free space falls below `--min-free-space-gib` or output
growth exceeds `--max-output-gib`, the harness stops the entire timed process
group with `SIGTERM` and a bounded `SIGKILL` fallback, then attempts to
finalize a failed report. The last successful sample remains available even
when the iteration never produces normal GNU time evidence. Schema v2 defines
the periodic evidence; schema v1 remains published for existing reports:

- [`soak-report-v2.schema.json`](schemas/soak-report-v2.schema.json)
- [`soak-report-v1.schema.json`](schemas/soak-report-v1.schema.json)

Operator `Ctrl-C` and service `SIGTERM` use the same process-group cleanup and
return `130` and `143`, respectively. Their terminal v2 report has
`interrupted` status. If the filesystem refuses the final atomic update, the
last successfully written running-state sample remains as recovery evidence.

The drop counter is a conservative count of documented console signatures for
message-filter drops, scan drops and queue/buffer overflow. One line can match
more than one signature, so the total can over-count; a zero only proves those
signatures were absent from the captured logs. A failed iteration, unreadable
telemetry, or exceeded RSS/output/drop budget stops the profile immediately
and preserves failed evidence. The duration gate passes only after the full
3,600 or 28,800 seconds have elapsed.

## Open Phase 3 gates

The following readiness rows remain incomplete and must not be inferred from
the termination coverage:

- timestamp reversal detection against real rosbag records before launch;
- execute and archive the one-hour and eight-hour soak profiles on named
  release hardware; the harness and machine-readable thresholds are automated,
  but real-duration evidence is not yet recorded;
- a bounded-filesystem live exhaustion test in the scheduled real-data
  environment;
- output migration tooling and last-known-good rollback instructions.

See the [pinned real-data E2E contract](real-data-e2e.md) and the
[v0.9 roadmap](roadmap/v0.9.md) for the remaining Phase 3 and v1.0 gates.
