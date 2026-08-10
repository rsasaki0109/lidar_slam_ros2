# Comparable onboarding trials

An onboarding trial measures how much work a new operator must do before the
product produces a verified first map. It complements the generated first-map
receipt: the receipt proves the result, while the trial record captures time,
commands, transfer size, disk use, and any undocumented intervention.

Use this contract for the fixed Docker and source quickstarts before changing
the landing page or promoting a release. Never reconstruct a missing value
from memory. Store it as `null`; the checker will retain the valid record but
mark its measurements `INCOMPLETE`.

## Fixed G0 trial matrix

| Trial | Clean starting point | Canonical documentation | Fixed input |
| --- | --- | --- | --- |
| Docker Humble, x86_64 | Ubuntu 22.04 with Docker installed; no project image, dataset, or output cache | [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) | MID-360 public demo |
| Docker Jazzy, x86_64 | Ubuntu 24.04 with Docker installed; no project image, dataset, or output cache | Docker first-map candidate for Jazzy | MID-360 public demo |
| Source Humble, x86_64 | Ubuntu 22.04 with ROS 2 Humble installed; no checkout, build, dataset, or output | [source quickstart](getting-started.md#1-build-the-workspace) | documented source demo |
| Source Jazzy, x86_64 | Ubuntu 24.04 with ROS 2 Jazzy installed; no checkout, build, dataset, or output | [source quickstart](getting-started.md#1-build-the-workspace) | documented source demo |

A row with no runnable documented path is a product finding, not a skipped
success. Record it as `FAIL` at the earliest applicable stage. Docker and
source currently use different fixed datasets, so runtime and output-size
values are compared across releases within the same row. Cross-path values
measure onboarding burden only; they are not algorithm-performance claims.

## Measurement sheet

Record every field in
[`onboarding-trial-v1.schema.json`](schemas/onboarding-trial-v1.schema.json).
The following rules keep separate operators and releases comparable.

| Field | Measurement rule |
| --- | --- |
| `environment.clean_start` | `true` only when the documented prerequisites are present but the project checkout/image, dataset, build, install, and output are absent. Do not pre-pull an image or warm a package, Git, or dataset cache. |
| `environment.revision` | Resolve source to a 40-character Git commit or an image to a `sha256:` digest before acceptance. A release tag may document a trial, but it is not immutable enough for comparison. |
| `input.download_bytes` | Count the bytes newly transferred for the selected dataset from a cold dataset cache. Use the archive's recorded payload size when the downloader exposes it. This field excludes unrelated host traffic and prerequisite installation. |
| `measurements.wall_time_sec` | Start immediately before entering the first command on the selected documentation path. Stop when the receipt is written or the attempt reaches its terminal failure. Include downloads and unattended processing. |
| `measurements.active_operator_time_sec` | Accumulate only hands-on time spent entering commands, answering prompts, reading required output, and following documented next actions. Pause while a command runs unattended. Do not include note taking for the study. |
| `measurements.command_count` | Count each command submitted by the operator. A copied multiline shell block is one command; commands invoked internally by a script do not count. Recovery commands count, even when documented. |
| `measurements.peak_disk_bytes` | Measure the largest increase above the clean baseline in the trial's dedicated filesystem scope. Include project images or checkout, dataset, build/install tree, temporary output, and final output. Use the same scope for every row. |
| `measurements.output_bytes` | Measure allocated bytes for the finalized run directory after success, or the retained partial run directory after failure. Do not include the source dataset. |
| `outcome.undocumented_manual_steps` | Count every intervention needed for progress that the selected page did not instruct. A passing comparable baseline requires zero. |
| `outcome.finding_codes` | Use short stable codes such as `image-tag-missing`, `dependency-resolution`, or `mapping-timeout`. Every failed trial needs at least one code and a failure stage. |

Byte values are integer counts, not rounded MB/GB display values. Wall and
active time may contain fractional seconds. Keep the raw stopwatch and disk
observations outside Git if they reveal local details; the committed record
contains only the bounded aggregate fields.

## PASS and comparability gates

A `PASS` record must have all of these outcomes:

- runner exit code `0`;
- succeeded manifest and successful diagnosis;
- verifier and privacy-bounded receipt both report `PASS`;
- no undocumented manual steps and no failure stage;
- SHA-256 values for the manifest and receipt.

A valid record is a **comparable baseline** only when it also starts clean,
uses an immutable revision, has all six measurements, and passes. Failed and
incomplete trials remain valuable evidence; they must not be silently removed
from the study.

Validate a record with:

```bash
python3 scripts/check_onboarding_trial.py trial.json \
  --json \
  --require-comparable
```

Exit code `0` means comparable, `1` means the JSON is valid but does not meet
the comparison gate, and `2` means the record violates the contract. Omit
`--require-comparable` when auditing an expected failure.

## Privacy boundary

The record must not contain operator identity, private filesystem paths, exact
commands, map geometry, bag metadata, hostnames, IP addresses, or raw logs.
Use bounded slugs for trial and dataset identifiers. Review the manifest and
receipt separately before sharing them; only their SHA-256 values belong in
this record.

Summarize completed rows in the
[weekly growth scorecard](growth-scorecard.md), then fix the largest repeated
activation blocker before expanding discovery work.
