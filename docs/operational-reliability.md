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

## Open Phase 3 gates

The following readiness rows remain incomplete and must not be inferred from
the termination coverage:

- timestamp reversal detection against real rosbag records before launch;
- long-running free-space telemetry and a bounded-filesystem live exhaustion
  test in the scheduled real-data environment;
- one-hour and eight-hour soak profiles with RSS, wall-time, output-size, and
  dropped-input counters;
- output migration tooling and last-known-good rollback instructions.

See the [pinned real-data E2E contract](real-data-e2e.md) and the
[v0.9 roadmap](roadmap/v0.9.md) for the remaining Phase 3 and v1.0 gates.
