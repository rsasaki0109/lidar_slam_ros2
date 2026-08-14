# GLIM usability scorecard

This scorecard measures whether a first-time operator can complete the same
overlapping LiDAR-SLAM jobs with `lidarslam_ros2` and GLIM. It does not infer an
overall winner and does not treat a longer feature list as usability evidence.

Current checked-in status:

```bash
python3 scripts/check_usability_scorecard.py --json
```

The expected result is `NOT_READY` until one public, exact-version record exists
for each product. Missing evidence never counts as success.

## Fixed six tasks

| Task ID | Operator question | Required evidence |
| --- | --- | --- |
| `discover-supported-path` | How long until the correct supported command is identified? | wall/active time, command and failure count, supported command identified |
| `run-fixed-demo` | Can the same fixed public input reach a verifiable result? | commands, download, wall/active time, peak disk, failures, output bytes |
| `inspect-own-bag` | Are topics, frames, timestamps, and workflow choice explained before mapping? | all four explanation checks plus time, commands, and failures |
| `produce-downstream-artifact` | Can the result become a verified downstream artifact without hidden assembly? | produced/verified checks, time, commands, failures, output bytes |
| `understand-failure` | Does one public error expose a stable code and one safe recovery action? | both recovery checks plus time, commands, and failures |
| `repeat-or-upgrade` | Does the documented command and output contract survive a supported upgrade? | command/output checks, time, download, commands, and failures |

Each task remains independent. A fast demo cannot compensate for an
unrecoverable error, and a strong map artifact cannot erase an undocumented
installation step.

## Neutral paired protocol

Use one external operator who has not previously run either product. Record
which product was attempted first; the paired records must contain one `first`
and one `second` order. For a stronger public claim, repeat the scorecard with a
second external operator in the opposite order and publish both scorecards.

Before timing:

1. select exact publicly resolvable product versions and documentation URLs;
2. allocate clean Humble or Jazzy hosts with the same OS, architecture, and
   declared hardware class, supported by each product's selected public docs;
3. assign one comparison pair ID and anonymous operator cohort ID;
4. choose the same public input ID for each overlapping task;
5. start from each product's public landing page, not a maintainer shortcut;
6. prepare a neutral observer who does not provide undocumented help; and
7. record a transcript hash without publishing operator identity or private
   filesystem paths.

The two hosts may have different machine fingerprints, but their declared
hardware class and supported software environment must match. A mismatch blocks
every task instead of being explained away after the run.

## Timer and command rules

- Start wall and active time when the operator opens the task's public entry
  page or submits its first documented command, whichever comes first.
- Active time includes reading required output, entering commands, answering
  prompts, and following displayed recovery actions. Pause it during unattended
  downloads, builds, and mapping.
- Count every operator-submitted shell command. Commands copied as one shell
  submission count once; hidden observer commands do not enter the product
  score but belong in the private study notes. Preserve retries as repeated
  entries in the exact command sequence, whose length must equal the recorded
  command count; a documentation-only task may record zero commands.
- Count a failure whenever the documented route reaches a non-success terminal
  state or the operator must abandon a command.
- Record workflow download and peak-disk measurements from the same isolation
  boundary used by the onboarding trial contract.
- Do not repair a result from memory. Any undocumented manual step is recorded
  and makes that task non-comparable.

## Record and validate evidence

Each product record must validate against
[`usability-scorecard-trial-v1.schema.json`](schemas/usability-scorecard-trial-v1.schema.json).
The checked-in evidence index is
[`glim-usability-scorecard-evidence-v1.json`](contracts/glim-usability-scorecard-evidence-v1.json).
It names only reviewed records under `docs/evidence/usability/` and keeps absent
rows as `null`.

Create a safe worksheet instead of hand-writing the six-task JSON:

```bash
python3 scripts/prepare_usability_scorecard.py \
  --product lidarslam_ros2 \
  --version 0.9.1 \
  --revision-kind git-commit \
  --revision <40-lowercase-hex-commit> \
  --documentation-url https://<public-docs-host>/<product-path> \
  --cohort-id external-paired-operator-a \
  --comparison-pair-id paired-jazzy-machine-class-a \
  --input-id fixed-demo-v1 \
  --product-order first \
  --ros-distro jazzy \
  --os-family ubuntu-24.04 \
  --architecture x86_64 \
  --hardware-class eight-core-32gib-x86_64 \
  --machine-fingerprint-sha256 <64-lowercase-hex-fingerprint> \
  --output /tmp/lidarslam-usability-trial.json
```

The generator writes once and refuses to overwrite. It leaves commands,
measurements, transcripts, and task checks empty or negative, defaults the
public-identity and clean-host claims to false, and marks every task
`not-recorded`. Add `--publicly-resolvable` and `--clean-start` only when those
prerequisites have actually been checked. The result is a worksheet, not
evidence; do not add it to the reviewed index until the observed trial is
complete and the checker passes.

For a paired run, prepare both records from one shared set of cohort, input,
and environment arguments. The command assigns opposite product order and
preflights both output names before writing either worksheet:

```bash
python3 scripts/prepare_usability_scorecard_pair.py \
  --lidarslam-version 0.9.1 \
  --lidarslam-revision-kind git-commit \
  --lidarslam-revision <40-lowercase-hex-commit> \
  --lidarslam-documentation-url https://<public-docs-host>/lidarslam \
  --lidarslam-trial-id lidarslam-pair-operator-a \
  --glim-version <glim-version> \
  --glim-revision-kind <git-commit-or-release-tag-or-image-digest> \
  --glim-revision <exact-glim-revision> \
  --glim-documentation-url https://<public-docs-host>/glim \
  --glim-trial-id glim-pair-operator-a \
  --cohort-id external-paired-operator-a \
  --comparison-pair-id paired-jazzy-machine-class-a \
  --input-id fixed-demo-v1 \
  --ros-distro jazzy \
  --os-family ubuntu-24.04 \
  --architecture x86_64 \
  --hardware-class eight-core-32gib-x86_64 \
  --machine-fingerprint-sha256 <64-lowercase-hex-fingerprint> \
  --output-dir /tmp/usability-pair-operator-a
```

The common fingerprint is convenient for a clean sequential pair. When the
products use separate hosts, replace it with
`--lidarslam-machine-fingerprint-sha256` and
`--glim-machine-fingerprint-sha256`. Use
`--lidarslam-order second` for the opposite order. Product-publicity flags
(`--lidarslam-publicly-resolvable` and `--glim-publicly-resolvable`) and
`--clean-start` remain opt-in claims. The pair command refuses to overwrite
either existing worksheet and emits a local-only, `PREPARED_INCOMPLETE`
manifest; it does not add records to the reviewed index or perform a remote
mutation.

Validate two records before adding them to the index:

```bash
python3 scripts/check_usability_scorecard.py \
  --record /path/to/lidarslam-record.json \
  --record /path/to/glim-record.json \
  --json
```

The checker enforces task/check order, task-specific non-null measurements,
same input and paired environment, public product identities, clean hosts,
single-line commands, transcript hashes, no undocumented steps, and an external
first-attempt pair.

## Status meanings

| Status | Meaning |
| --- | --- |
| `NOT_READY` | No task is comparable, including when either product record is missing. |
| `PARTIAL` | At least one task is comparable, or all tasks were measured by a maintainer/non-first-time pair. |
| `READY` | All six tasks are comparable for a public, external first-attempt pair. |

`READY` means the scorecard is publishable; it does not mean
`lidarslam_ros2` is globally better than GLIM. Publish exact task values,
versions, commands, limitations, product order, and intentional job differences.

## Privacy and authority

Records contain no operator identity, secret, or private path. Commands use
public placeholders. The evidence index authorizes no upload, GitHub mutation,
benchmark claim, or winner statement. Publication remains a separate maintainer
decision after the exact records and candidate revision are public.
