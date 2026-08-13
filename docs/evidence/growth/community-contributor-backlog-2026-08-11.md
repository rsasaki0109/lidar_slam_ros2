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

## Candidate C5 — Japanese empty-frame recovery card

Suggested issue title:

> Docs: explain empty `frame_id` recovery in the Japanese first-run card

Why this task exists:

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

## Publication and review sequence

1. Recheck each candidate against the then-current public `develop` revision.
2. Confirm that no open issue or pull request already implements the exact
   acceptance criteria.
3. Obtain explicit authorization before creating or editing GitHub issues.
4. Publish C1 and C5 first; they exercise setup and diagnosis documentation.
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
