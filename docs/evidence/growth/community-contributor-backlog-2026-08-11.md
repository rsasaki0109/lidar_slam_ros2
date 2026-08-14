# Community contributor backlog — 2026-08-11

> Status: **PREPARED_NOT_PUBLISHED**
>
> Repository snapshot: **837 Stars, 29 open issues, 0 open pull requests**
>
> Publication authority: **not granted**

This is the bounded community work queue for the G0 phase of the
[1,000 Stars roadmap](../../roadmap/1000-stars.md). It converts recurring
support demand into five tasks that a new contributor should be able to finish
in 30 minutes or less. It is not a record of created GitHub issues: no issue,
label, assignment, comment, or repository setting was changed during this
read-only audit.

The snapshot was taken from the public GitHub repository on 2026-08-11. It
retains only aggregate counts and public issue numbers. Author identities and
comment bodies are not copied into this evidence record.

On 2026-08-12, when the live aggregate had moved to 30 open issues, this prose
backlog was promoted to the machine-readable
`docs/contracts/contributor-starter-queue-v1.json` contract. A read-only
GitHub connector audit found one open pull request, #427, and zero matching
open pull requests for each of C1–C5. The local checker validates exact file
scope, a fixed command allowlist, 30-minute estimates, known-gap drift, and the
no-write boundary. It can render copy-ready task bodies, but it does not create
issues, labels, comments, branches, or pull requests. The duplicate audit must
be rerun immediately before any future publication.

```bash
python3 scripts/contributor_starter_queue.py --json
python3 scripts/contributor_starter_queue.py --list
python3 scripts/contributor_starter_queue.py --task starter-C5
```

## Operationalization validation — 2026-08-12

The checked-in queue reports `QUEUE_READY_LOCAL_ONLY`: all five tasks remain
locally present, their known gaps are still observable, and none has a matching
open pull request in the recorded read-only audit. The default command and the
list/detail views perform no network or workspace write. Focused verification
uses built-in profiles rather than executing commands supplied by JSON; docs
output goes to a temporary directory and Python cache writes are disabled.

| Check | Result |
| --- | --- |
| queue/schema/drift/authority regressions | 17 passed |
| `--verify starter-C1` | strict MkDocs profile passed; no workspace artifact |
| `--verify starter-C5` | strict MkDocs profile passed; no workspace artifact |
| contributor runner style | full flake8 passed for runner and test |
| complete maintained Python gate | graph 1,428 passed / 13 skipped; lidar_slam 687 passed; 2,115 total |
| documentation | strict MkDocs build passed with pre-existing notices |
| authority | no issue, label, comment, branch, PR, or other remote mutation |

These checks prove that the queue is bounded and usable by maintainers. They do
not prove a 30-minute external completion and do not authorize publication.
That evidence begins only after a separately approved issue is claimed and a
non-maintainer reports prepared-environment timing.

## What the backlog says

Only issue
[#422](https://github.com/rsasaki0109/lidar_slam_ros2/issues/422) currently has
the `good first issue` label. Sixteen of the 29 open issues have no label, and
28 were opened before 2026. The old backlog is therefore a stronger source of
beginner work than speculative feature expansion.

The 29 open issues fall into these mutually exclusive planning groups. The
grouping is an operating aid, not a final disposition of any issue.

| Demand group | Count | Public issue numbers |
| --- | ---: | --- |
| First-map validation and community | 1 | #422 |
| Install and dependency setup | 3 | #108, #110, #122 |
| TF, input, and missing output | 6 | #64, #93, #102, #103, #106, #112 |
| Sensor and robot onboarding | 9 | #83, #89, #95, #96, #98, #100, #105, #111, #115 |
| Mapping quality, reliability, and tuning | 7 | #30, #53, #69, #92, #94, #104, #124 |
| Advanced capability or algorithm scope | 3 | #101, #116, #118 |

The first publication batch should target setup, diagnosis, and documentation.
Relocalization, loop-closure redesign, and broad algorithm work are not starter
tasks because they cannot be bounded honestly to one fixture and one focused
check.

## Starter-task contract

Every published starter issue must contain all of the following:

- one operator-visible outcome;
- an estimate of at most 30 minutes for a prepared contributor environment;
- exact files that may change;
- explicit non-goals;
- acceptance criteria that can be checked without private data or hardware;
- one focused command that normally completes in under five minutes;
- `good first issue`, `help wanted`, and one domain label;
- a maintainer confirmation that no open pull request already implements it.

The estimate begins after the repository and documented development
dependencies are available. Review latency, ROS installation, and first-time
tool downloads are reported separately rather than hidden inside the estimate.

## Candidate C1 — current g2o setup card

Suggested issue title:

> Docs: explain the supported g2o package path and EOL boundary

Why this task exists:

- issues #108 and #122 report `libg2o` resolution or API-version failures;
- issue #110 includes a Humble container installation path;
- the current docs state that Humble and Jazzy packages resolve `libg2o`, but
  the beginner page does not turn the historical errors into one recovery card.

Scope:

- update `docs/getting-started.md` and, only if needed,
  `docs/rosdistro-release.md`;
- distinguish a missing rosdep key from an incompatible source-built g2o;
- show `rosdep resolve libg2o` and the supported binary-package check;
- state that Foxy is outside the maintained product contract;
- recommend a pinned supported package path, not an unpinned source clone.

Acceptance:

- a reader can identify whether the failure is dependency resolution or a C++
  API mismatch;
- Humble and Jazzy are the only maintained distributions claimed;
- the card links to the product support boundary;
- no hardware support or build-success claim is added without evidence.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **20 minutes**. Non-goals: changing CMake, vendoring g2o, or reviving
an EOL ROS distribution.

## Candidate C2 — no-map three-check card

Suggested issue title:

> Docs: add a three-check recovery card for an empty `/map` or viewer

Why this task exists:

- issues #93, #102, #103, and #106 all describe a missing map or blank viewer;
- the beginner page currently jumps from the symptom to the output directory
  without separating input, frame, runtime, and viewer failures.

Scope:

- extend the `Common First-Run Problems` section in
  `docs/getting-started.md`;
- provide one check each for a live `PointCloud2` input, a non-empty sampled
  `frame_id`, and a connected TF path;
- distinguish “no map messages were produced” from “a map exists but the
  viewer fixed frame or selected topic is wrong”;
- route own-bag users back to `lidarslam-map doctor` and the generated diagnosis.

Acceptance:

- each command includes the expected observation and one next action;
- placeholder topic and frame names are visibly marked for replacement;
- the card never asks a user to upload a map, bag, or location-bearing log;
- the fixed public demo remains the first control experiment.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **25 minutes**. Non-goals: changing ROS nodes, promising support for
an untested sensor, or diagnosing an individual historical bag.

## Candidate C3 — Odometry message versus TF

Suggested issue title:

> Docs: explain why an Odometry topic does not guarantee `odom -> base_link` TF

Why this task exists:

- issue #112 has an Odometry message whose frame names exist in the message but
  not in the TF tree;
- issue #64 reports future-extrapolation warnings after TF frequency lag;
- these are different failures and should lead to different next actions.

Scope:

- add a short card to `docs/workflows.md`;
- explain that a `nav_msgs/msg/Odometry` publisher does not necessarily
  broadcast the corresponding transform;
- show how to check the Odometry header, `odom -> base_link` TF availability,
  and transform freshness;
- separate a missing transform from an extrapolation/timestamp problem.

Acceptance:

- the card contains no copied third-party broadcaster implementation;
- frame direction and placeholder names are explicit;
- both failures end in a safe, testable next action;
- the text does not recommend suppressing TF warnings as a fix.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **25 minutes**. Non-goals: supplying a robot-specific broadcaster,
changing scanmatcher timing, or tuning a controller.

## Candidate C4 — custom PointCloud2 sensor checklist

Suggested issue title:

> Docs: add the minimal checklist for adapting another PointCloud2 LiDAR

Why this task exists:

- issues #83, #89, #95, #96, #98, #100, #105, #111, and #115 ask how to adapt
  a new sensor or platform;
- most requests need the same input contract before vendor-specific tuning is
  meaningful.

Scope:

- add a compact checklist to `docs/workflows.md`;
- cover the `PointCloud2` topic, non-empty frame, static extrinsic, timestamps,
  scan period, and min/max range;
- show the public launch arguments used to remap the topic and frames;
- link to the sensor-support issue form for evidence that does not fit the
  checklist.

Acceptance:

- every checklist item has one observation command or configuration field;
- vendor names are examples only and are not added to the supported matrix;
- success means “ready for a controlled first run,” not “accuracy validated”;
- unsafe advice such as guessing an extrinsic is explicitly excluded.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: adding a driver, selecting universal
tuning values, or claiming support for hardware that has not passed a recipe.

## Completed C5 — Japanese empty-frame recovery card

Suggested issue title:

> Docs: explain empty `frame_id` recovery in the Japanese first-run card

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now states that a sampled `frame_id` must be
non-empty and tells the operator to repair the publisher's
`header.frame_id`, repeat the check, and avoid guessing a viewer frame. The
queue's drift probe deliberately retires this task after the marker appears.

The original reason for the task was:

- issue #102 shows the operator-visible empty-frame symptom;
- the English first-run card now explains that an empty sampled `frame_id` is
  a publisher problem, while the Japanese card still stops after the sample
  command and does not state the repair action;
- the fail-closed preflight implementation is already covered by the public
  candidate, so the next bounded contribution should close the language-path
  gap rather than duplicate the implementation.

Scope:

- extend the three-check recovery card in `docs/getting-started-ja.md`;
- state that a sampled `frame_id` must be non-empty;
- tell the operator to repair the publisher's `header.frame_id`, repeat the
  check, and avoid guessing a viewer frame;
- keep the existing topic and TF commands and the privacy boundary unchanged.

Acceptance:

- the Japanese three-check card states that a sampled `frame_id` must be
  non-empty;
- an empty or timed-out sample tells the operator to repair the publisher
  `header.frame_id` and repeat the check;
- the card tells the operator not to guess a viewer frame and keeps the
  existing topic and TF commands intact;
- no rosbag, hardware, network, or private log is needed by the change.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the preflight implementation,
translating the entire getting-started guide, or claiming support for an
unvalidated sensor.

## Completed C5 — Japanese TF frame substitution

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now connects the non-empty frame sampled in
check 2 to `<POINTCLOUD_FRAME>`, identifies the runtime/viewer target frame,
and tells the operator to reuse the same actual frame names when the TF check
fails. The queue's drift probe retires this task after the marker appears.
The preceding C5 topic-selection increment is also retained in the candidate:
it shows `ros2 topic list -t`, selects the
`sensor_msgs/msg/PointCloud2` row, and tells the reader to copy only its topic
name.

## Completed C5 — Japanese headless preview recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now explains that the browser preview is a
self-contained local HTML artifact, shows the exact `HTML:` path emitted by
`view`, and gives copy-ready `--no-open` plus `--preview-dir` commands for
headless or browser-failure recovery. It also keeps the map data privacy
boundary visible by directing users to the sanitized support report instead of
uploading the preview, map, bag, or raw log.

The queue's drift probe retires this task after the `--no-open` marker appears.

Suggested issue title:

> Docs: add headless preview recovery to the Japanese first-run guide

Acceptance:

- explain that a browser not opening or a headless machine should use the
  printed self-contained HTML path;
- show `--no-open` and `--preview-dir` as safe preview options;
- keep the existing map verification, diagnosis, TF, topic-selection,
  empty-frame, and privacy guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the browser, viewer, or preview
implementation, translating the entire guide, or claiming support for an
unvalidated sensor.

## Completed C5 — Japanese session history and recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now explains `lidarslam-map sessions`, the
`--status action_required` filter, `--viewer none`, and read-only `--json`
inspection. It tells the operator to read retained `Details:` and `Next:`
values, follow the exact next command, and check `map_verify` and diagnosis
findings before changing viewer settings. Session history and recovery remain
local-only and preserve the no-private-upload boundary.

The queue's drift probe retires this task after the session marker appears.

Suggested issue title:

> Docs: explain Japanese session history and recovery

Acceptance:

- explain how to list saved sessions with `lidarslam-map sessions`;
- show `--status action_required`, `--viewer none`, and `--json` for focused or
  headless recovery;
- tell the reader to follow the retained `Next` action and keep preview,
  diagnosis, and no-private-upload guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing session storage, recovery, or
viewer implementation, translating the entire guide, or claiming support for
an unvalidated sensor.

## Completed C5 — Japanese privacy-first support handoff

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides the read-only
`lidarslam-map support /path/to/session_bundle --json` inspection path, explains
that `--first-map` revalidates a receipt-bound PASS without creating a ZIP or
contacting GitHub, and names the three generated files that must be reviewed
before any selective sharing. It keeps maps, bags, raw logs, parameters,
private paths, credential-like command values, and the `--first-map --json`
handoff JSON outside the public attachment path.

The queue's drift probe retires this task after the support command marker
appears. The next task narrows the handoff from privacy review to independent
validation form use; it does not provide live troubleshooting.

Suggested issue title:

> Docs: explain Japanese privacy-first support handoff

Acceptance:

- show `lidarslam-map support /path/to/session_bundle --json` for read-only
  inspection;
- explain that `--first-map` is a read-only verified-first-map handoff and
  never uploads to GitHub;
- tell the reader to review `README.txt`, `issue-body.md`, and
  `support-report.json` before sharing, while preserving session, preview,
  diagnosis, and no-private-upload guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing support-bundle generation, issue
templates, or session implementation, translating the entire guide, or
claiming support for an unvalidated sensor.

## Completed C5 — Japanese independent first-map validation handoff

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that `--first-map` prints the
canonical independent-validation issue form alongside the verification summary
and safe environment hints. It tells the operator to fill the form from their
own run, redact private paths from the command, and attach only the reviewed
first-map receipt. It explicitly excludes the local handoff JSON, receipt path,
map, bag, preview, raw log, trajectory, parameter, and screenshot attachments,
and preserves the no-live-guidance rule for independent validation.

The queue's drift probe retires this task after the independent-validation
marker appears. The next task moves one step earlier in the evidence chain:
recording the exact installed product identity before support or validation.

Suggested issue title:

> Docs: explain Japanese independent first-map validation handoff

Outcome:

A Japanese first-run user can fill the independent-validation form from their
own run and attach only a reviewed first-map receipt, without exposing a map,
bag, preview, raw log, local receipt path, or handoff JSON.

Acceptance:

- explain that `--first-map` prints the verification summary, safe environment
  hints, and the canonical independent-validation issue form;
- tell the operator to fill the form from their own run and attach only a
  reviewed first-map receipt;
- state that the handoff JSON and local receipt path are not public
  attachments, and that the command never uploads to GitHub;
- keep the existing support, session, preview, diagnosis, and no-private-upload
  guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing support-bundle generation, issue
templates, or first-map validation schemas, translating the entire guide,
providing live guidance that would invalidate independent validation, or
claiming support for an unvalidated sensor.

## Completed C5 — Japanese version identity and support-boundary check

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows the read-only `lidarslam-map
--version` command, distinguishes the published v0.9.0 Docker images from the
unpublished v0.9.1 source candidate, and tells the operator to transfer the
observed version/revision rather than guessing a release identity. The existing
support, privacy, and independent-validation boundaries remain intact.

The queue's drift probe retires this task after the version marker appears. The
next task turns the Japanese failure path into stable-code triage so a user can
choose the correct next action before requesting support.

Suggested issue title:

> Docs: add a Japanese version identity and support-boundary check

Outcome:

A Japanese first-run user can record the installed product identity, match it
to the documented stable or candidate path, and avoid presenting an unpublished
candidate or moving tag as supported release evidence.

Acceptance:

- show `lidarslam-map --version` and tell the reader to record its output before
  support or validation handoff;
- distinguish the immutable published v0.9.0 Docker images from the unpublished
  v0.9.1 source candidate path;
- tell the reader not to use a moving `develop` tag or guess a release identity
  when reporting evidence;
- preserve the existing support, session, preview, diagnosis, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing release metadata, publishing an
image, changing version semantics, translating the entire guide, claiming that
an unpublished candidate is a stable release, or validating an untested sensor.

## Completed C5 — Japanese reason-code and Next-action triage

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows the read-only
`lidarslam-map doctor /path/to/rosbag2 --json` diagnosis path, distinguishes the
`findings[].code` field in the doctor report from `reason.code` and
`findings[].code` in start/session recovery JSON, and tells the reader to use
stable codes instead of viewer symptoms or English message text. It also tells
the reader to follow retained `next_action`, `Next:`, or `next_command` values,
separates safe `--resume` post-processing from returning to doctor, and keeps raw
JSON with local paths out of public support attachments.

The queue's drift probe retires this task after the next Japanese dry-run card
marker appears. The next task moves one step earlier in the write boundary:
showing how to inspect an own-bag plan before a session or map is created.

Suggested issue title:

> Docs: explain the Japanese dry-run and write boundary

Outcome:

A Japanese first-run user can inspect an own-bag plan before writes, understand
when a session or map may be created, and choose a controlled next action without
rerunning an unknown input blindly.

Acceptance:

- show `lidarslam-map start /path/to/rosbag2 --yes --dry-run --json` as a
  no-write preflight;
- explain that dry-run creates no session or map output and that confirmation is
  required before mapping writes;
- keep `--viewer none` as the safe choice for headless execution and name the
  retained next command after inspection;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the start or session
implementation, translating the entire guide, asking for a private bag/map/raw-
log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese dry-run and write boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows
`lidarslam-map start /path/to/rosbag2 --yes --dry-run --json`, explains that the
dry-run returns a `status: dry_run` plan without leaving a session bundle or map,
and identifies `run.command_shell` as the retained command to review. It also
separates the no-maintained-profile `reason.code`/`findings[].code` path, keeps
the doctor `next_command` actionable, and states that `--viewer none` is the
headless route after the plan is accepted.

The queue's drift probe retires this task after the next Japanese fresh-retry
card marker appears. The next task protects failed-run provenance by making the
fresh output-directory rule visible in the Japanese recovery path.

Suggested issue title:

> Docs: explain Japanese fresh-output retry without overwrite

Outcome:

A Japanese first-run user can retry a failed or incomplete map in a fresh output
directory, preserve the retained setup and evidence, and avoid overwriting an
earlier run.

Acceptance:

- explain that an existing output directory is not overwritten and that a retry
  uses a fresh `--output-dir`;
- distinguish the retained pinned setup and evidence from a new map output
  directory;
- tell the reader to use the retained retry or next command rather than
  reconstructing a command from a viewer symptom;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the start, session, or recovery
implementation, translating the entire guide, asking for a private bag/map/raw-
log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese fresh-output retry without overwrite

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that an existing `--output-dir` is
never overwritten, distinguishes the retained `setup_bundle` and evidence from
the fresh `retry.output_dir`, and tells the reader to use the generated
`retry.command` or `--resume` `next_command` instead of reconstructing a command
from a viewer symptom. The local-only and no-private-upload boundary remains
explicit.

The queue's drift probe retires this task after the next Japanese verification
boundary marker appears. The next task makes the trust boundary visible between
a map that displays and a map whose receipt-backed verification actually passed.

Suggested issue title:

> Docs: explain the Japanese verified-result boundary

Outcome:

A Japanese first-run user can distinguish a displayed map from a verified result,
read the retained receipt status, and avoid sharing or relying on an unverified
output as trusted evidence.

Acceptance:

- distinguish `map_verify: PASS` from a map that merely displays in a viewer;
- name the retained `first_map_validation_receipt.json` and explain `NOT VERIFIED`
  or `UNAVAILABLE` without guessing;
- direct the reader to the retained diagnosis or inspect command before support
  or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation or
quality thresholds, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese verified-result boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes a viewer-rendered map from a
trusted result: the same run must show `map_verify: PASS` and retain a
`first_map_validation_receipt.json` whose status is `PASS`. It explains
`NOT VERIFIED` and `UNAVAILABLE` without guessing, directs the reader through
the retained diagnosis and `inspect` command, and keeps receipt review and the
no-private-upload boundary before support or independent validation.

The queue's drift probe retires this task after the next Japanese receipt/session
boundary marker appears. The next task confirms that a PASS receipt belongs to
the same session and output being discussed.

Suggested issue title:

> Docs: explain the Japanese verified-result boundary

Outcome:

A Japanese first-run user can distinguish a displayed map from a verified result,
read the retained receipt status, and avoid sharing or relying on an unverified
output as trusted evidence.

Acceptance:

- distinguish `map_verify: PASS` from a map that merely displays in a viewer;
- name the retained `first_map_validation_receipt.json` and explain `NOT VERIFIED`
  or `UNAVAILABLE` without guessing;
- direct the reader to the retained diagnosis or inspect command before support
  or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation or
quality thresholds, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese receipt/session match boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now binds a receipt to the same session and map
output, explains the `run_id` and `manifest_sha256` provenance fields, and makes
`support --first-map` the read-only revalidation gate. A receipt is a sharing
candidate only after the handoff reports `READY FOR REVIEW`; copied, stale,
mismatched, or failed evidence remains untrusted and the existing privacy
boundary stays in place.

The queue's drift probe retires this task after the next Japanese failed-
revalidation recovery marker appears. The next task explains how to preserve old
evidence and recover safely when that gate rejects a receipt.

Suggested issue title:

> Docs: explain the Japanese receipt/session match boundary

Outcome:

A Japanese first-run user can confirm that a validation receipt belongs to the
same session and output, and can reject copied, stale, mismatched, or failed
evidence as untrusted.

Acceptance:

- tell the reader to match `first_map_validation_receipt.json` to the same
  session/output and read its top-level status;
- treat `status: FAIL`, a missing or malformed receipt, or failed receipt
  revalidation as untrusted rather than inferring PASS from a viewer;
- direct the reader to the retained diagnosis, manifest, verification log, or
  `inspect` command before support or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation,
receipt schemas, or quality thresholds, translating the entire guide, asking for
a private bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese failed receipt revalidation recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that a rejected read-only
`support --first-map` handoff leaves the original map, session, receipt, and
manifest intact. It directs the operator through local-only session and
`map_session_recovery.json` inspection, retained `Details:`/`Next:` values,
`--resume`, the pinned fresh-output `retry.command`, and a separate
verification-required run when neither recovery path is available. The guide
keeps old and new evidence separate, forbids editing or copying receipts, and
requires a new `READY FOR REVIEW` handoff before a reviewed receipt becomes a
sharing candidate.

The queue's drift probe retires this task after the failed-revalidation marker
appears. The next task turns the completed recovery path into a short final
check before any public sharing.

Suggested issue title:

> Docs: explain the Japanese failed receipt revalidation recovery

Outcome:

A Japanese first-run user can respond to a rejected first-map handoff by
preserving the old evidence and using a fresh verification-enabled output
without editing receipts or claiming support.

Acceptance:

- explain that a rejected `support --first-map` revalidation does not destroy
  the old map or justify editing its receipt, manifest, or session;
- direct the reader from a rejected handoff to retained `Details:`, `Next:`,
  `retry.command`, or a fresh verification-enabled output command;
- preserve the non-overwrite, retained-evidence, local-only, and no-private-
  upload boundaries while describing recovery;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing receipt validation, recovery, or
verification implementation, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese pre-share verification checklist

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides a copy-ready five-item gate for
product version/revision, same-session and output identity, read-only
`READY FOR REVIEW` revalidation, receipt privacy, and the single permitted
public attachment. It explicitly separates the local-only handoff JSON and
paths from the reviewed receipt, requires private-path redaction in the
operator-supplied command, and excludes maps, bags, logs, previews, and the
session bundle from public sharing.

The queue's drift probe retires this task after the five-item checklist marker
appears. The next task turns those confirmed fields into a Japanese public
report template that a validator can fill without copying local evidence.

## Completed C5 — Japanese reviewed-receipt public share template

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now mirrors the public validation form with a
copy-ready block for the documentation path, immutable release/commit/image
digest, environment, private-path-redacted command, PASS verification summary,
findings, and the reviewed receipt attachment. It states that handoff JSON,
receipt paths, session evidence, maps, bags, logs, previews, and other run
artifacts stay local or unshared, while only the reviewed PASS receipt is a
public attachment candidate.

The queue's drift probe retires this task after the public-share-template
marker appears. The next task explains the distinction between a local handoff,
a public report, maintainer review, and accepted ledger evidence.

## Completed C5 — Japanese validation report review status

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes local `READY FOR REVIEW`,
public report submission, maintainer review, accepted ledger evidence, and
unresolved or rejected reports. It explicitly states that a local handoff or
public receipt is not accepted validation until public review and ledger
requirements pass, keeps one report/receipt pair, forbids evidence editing or
duplication, and points contributors away from live step-by-step validation
help.

The queue's drift probe retires this task after the validation-report review
status marker appears. The next task supplies a path-free instructional example
without turning an example into accepted evidence.

## Completed C5 — Japanese privacy-safe validation report example

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now includes one path-free, clearly illustrative
report example with an immutable-identity placeholder, redacted command,
receipt-derived verification fields, operator-supplied public fields, and an
explicit `not submitted` / `not maintainer-reviewed` / `not accepted` status.
It warns contributors not to copy example hashes or treat the example as real
evidence, and keeps paths, maps, bags, logs, previews, and session bundles out
of the public example.

The queue's drift probe retires this task after the privacy-safe report-example
marker appears. The next task helps a user recover safely when switching
between the supported first-map routes.

## Completed C5 — Japanese Docker/source route chooser

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now has one route-choice card with a copy-ready
Docker fixed-demo command, a source `--dry-run` command, one stop/check boundary
for each, the published v0.9.0 versus unpublished v0.9.1 identity boundary, and
the unsupported PPA/package-manager boundary. It tells beginners to choose one
route, use fresh output when changing routes, and never mix Docker/source
receipts or session artifacts.

The queue's drift probe retires this task after the Docker/source route-choice
marker appears. The next task separates sanitized support diagnostics from
independent-validation evidence.

## Completed C5 — Japanese fresh-output route-switch recovery card

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now separates same-session `--resume`,
same-pinned-setup `retry.command`, and a changed Docker/source route. It requires
fresh output for a route switch, preserves the old map/session/receipt/manifest,
keeps v0.9.0 Docker and v0.9.1 source identity separate, and directs contributors
to retained `Details:`/`Next:` instructions without rebuilding commands from
viewer appearance.

The queue's drift probe retires this task after the fresh-output route-switch
marker appears. The next task separates sanitized support diagnostics from
independent-validation evidence.

## Completed C5 — Japanese support report versus validation report

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives one decision card for ordinary
sanitized support diagnostics and one for an independent validation report. It
separates the `support --json` report, the local-only `--first-map` handoff, the
operator-authored public validation report, and the reviewed receipt; support
diagnostics are explicitly not accepted validation evidence.

The card retains the no-private-upload boundary for local paths, recovery JSON,
maps, bags, logs, previews, and session bundles. It also keeps the published
v0.9.0 Docker identity separate from the unpublished v0.9.1 source candidate and
requires the reporter to record the exact identity rather than infer it from a
viewer.

The queue's drift probe retires this task after the support-versus-validation
marker appears. The next task makes the provenance of every public validation
report field explicit so contributors do not invent a hash, status, or identity.

## Completed C5 — Japanese public validation report field provenance

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now has a field-provenance table that separates
operator-supplied public fields, same-run receipt-derived validation fields, and
review or acceptance status. It binds identity, result, verification summary,
manifest hash, and receipt attachment to the same session and tells the reader
to stop on missing, unavailable, example-only, viewer-only, or mismatched values.

The card retains the no-private-upload boundary for local paths, recovery JSON,
maps, bags, logs, previews, trajectories, parameters, screenshots, and session
bundles. It keeps the published v0.9.0 Docker identity separate from the
unpublished v0.9.1 source candidate and preserves the existing support,
independent-validation, receipt, and review-status guidance.

The queue's drift probe retires this task after the field-provenance marker
appears. The next task makes an operator's public `findings` field actionable
without allowing it to rewrite receipt-derived evidence or disclose private data.

## Completed C5 — Japanese validation-report findings without private data

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives a four-field `step`/`expected`/
`observed`/`impact` pattern for one operator observation, distinguishes that
observation from receipt-derived evidence, forbids private artifacts, root-cause
claims, and acceptance claims, and routes maintainer-live-guidance-only
observations to the sanitized support report.

The queue's drift probe retires this task after the findings marker appears. The
next task routes unresolved findings to support or a safe retry without
duplicating evidence or treating follow-up as accepted validation.

## Completed C5 — Japanese validation-report finding follow-up

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides a two-route decision table for an
unresolved finding: a sanitized support follow-up using saved `Details:`,
`Next:`, and unedited `retry.command` instructions with fresh output, or a new
independent validation using a fresh run. It keeps the original report, receipt,
manifest hash, and review status unchanged, forbids duplicate issue/session
artifacts, and stops when identity or same-session data is missing.

The card preserves the v0.9.0 Docker versus v0.9.1 source-candidate boundary,
the no-private-upload rule, and the distinction between `READY FOR REVIEW`,
unresolved follow-up, and accepted ledger evidence. The queue's drift probe
retires this task after the finding-follow-up marker appears.

The next task makes the original report/receipt pair and a follow-up summary
easy to audit without creating duplicate evidence.

## Completed C5 — Japanese validation-report follow-up evidence pairing

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes the original
`report + reviewed receipt` pair, a follow-up note, and a new
independent-validation report. It gives a copy-ready audit block with route,
`reason.code`, sanitized `Details:`/`Next:`, fresh-output facts, review status,
and a duplicate-artifact check, while keeping the original evidence immutable
and local artifacts private.

The queue's drift probe retires this task after the follow-up-evidence marker
appears. The next task makes the sanitized follow-up summary itself easy to
audit without turning it into a new validation result.

## Successor C5 — Auditable Japanese validation-report follow-up summaries

The next prepared C5 task keeps the Japanese language-path scope and explains
how a maintainer can audit a sanitized follow-up summary against the original
evidence. It should preserve the distinction between a note, a new report, and
accepted ledger evidence without exposing private paths or receipt artifacts.

Suggested issue title:

> Docs: explain auditable Japanese validation-report follow-up summaries

Outcome:

A Japanese maintainer can audit an original report/receipt pair and a follow-up
summary without treating notes as new evidence or exposing private artifacts.

Acceptance:

- distinguish the original report/receipt pair, a follow-up note, and a new
  independent-validation report;
- give a copy-ready audit block with route, `reason.code`, sanitized
  `Details:`/`Next:`, fresh-output facts, and review status without private
  paths;
- keep the original report, receipt, and hash immutable, permit one
  report/receipt pair per run, and forbid duplicate issue or session artifacts;
- preserve the v0.9.0 Docker versus v0.9.1 source-candidate identity boundary
  and stop when identity or session data is missing;
- preserve the existing support, session, privacy, independent-validation, and
  review-status guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the issue template, review
ledger, support or verification implementation, changing the Docker image,
source helper, recovery implementation, or release identity, translating the
entire guide, asking for a private bag/map/raw-log upload, or claiming support
for an unvalidated sensor.

## Publication and review sequence

1. Recheck each candidate against the then-current public `develop` revision.
2. Confirm that no open issue or pull request already implements the exact
   acceptance criteria.
3. Obtain explicit authorization before creating or editing GitHub issues.
4. Publish C1 and the current C5 successor first; they exercise setup and
   diagnosis documentation.
5. Publish C2 and C3 after the first pair's review burden is known.
6. Publish C4 only after the common sensor checklist has a named maintainer
   reviewer; do not let vendor-specific discussion expand its scope.
7. Keep #422 open independently until three accepted external first-map
   validations exist.

The old source issues are not automatically closed when a starter task is
published or merged. Each old issue still needs a supported-version check, a
public resolution or support-boundary explanation, and an explicit disposition.
Those issue-specific decisions are recorded in the
[complete read-only triage proposal](open-issue-triage-proposal-2026-08-11.md).

## Success and stop rules

The first five tasks are successful when:

- at least three are completed by non-maintainers;
- median prepared-environment completion time is at most 30 minutes;
- every task is accepted using only its listed focused checks;
- at least two reusable support answers move into public documentation; and
- no task expands into an unbounded hardware or algorithm project.

If two consecutive starter contributions exceed the estimate by more than
15 minutes because of repository setup or test cost, stop publishing more
starter issues and repair the contributor path. If review capacity cannot keep
the published tasks moving, reduce the visible ready queue instead of inviting
more contributors into a stalled path.
