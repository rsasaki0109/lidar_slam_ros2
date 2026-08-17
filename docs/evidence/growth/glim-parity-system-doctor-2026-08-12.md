# GLIM parity: bag-optional system doctor — 2026-08-12

> Decision: **LOCAL_UX_INCREMENT_PASS / PUBLIC_COMPARISON_PENDING**
>
> Candidate base: public Draft PR `#427` head `3f4dd70`
>
> Network or files written by `lidarslam-map doctor`: **none**
>
> Remote mutations performed: **none**

## Why this increment

The current GLIM documentation presents four adoption advantages that matter to
a new user: PPA binary packages for Humble/Jazzy, prebuilt Docker images, a
direct rosbag executor, and an offline viewer that supports correction, export,
object removal, and session merging. Its setup documentation also separates
sensor/topic configuration from normal execution.

`lidar_slam_ros2` now overlaps the direct bag, sensor setup, local 3D review,
editing, and session-merge tasks through `start`, `setup`, `view`, `edit`, and
`merge`. GLIM's PPA remains the largest installation advantage. That cannot be
closed honestly by adding another source script: this project's package-manager
path still depends on reviewed NDT ownership and rosdistro publication.

The largest unblocked adoption gap was therefore the moment immediately after
installation. Previously, `doctor` required a bag, so a user could not ask
whether the product surface, ROS environment, and demo storage were ready
before locating data. The no-argument terminal home also had no safe route for
someone whose intent was simply “check this installation.”

Primary comparison sources inspected on 2026-08-12:

- [GLIM installation](https://koide3.github.io/glim/installation.html)
- [GLIM getting started](https://koide3.github.io/glim/quickstart.html)
- [GLIM Docker images](https://koide3.github.io/glim/docker.html)
- [GLIM README](https://github.com/koide3/glim/blob/master/README.md)

## Product change

`lidarslam-map doctor` now has two explicit modes:

```bash
lidarslam-map doctor
lidarslam-map doctor /path/to/rosbag2
```

Without a bag it checks, read-only:

1. the curated runtime-file inventory and current product version;
2. whether a source checkout has a matching installed prefix;
3. Humble/Jazzy activation and `ros2` availability;
4. `rosbag2_py`, required for safe input inspection; and
5. free space for the fixed demo, defaulting to the existing 8 GiB floor.

The report is governed by `system-doctor-v1.schema.json`. It returns `ready` or
`action_required`, stable finding codes, and one copy-ready `next_action` per
finding. JSON intentionally omits checkout, home, install-prefix, and
demo-directory paths. `--demo-dir` chooses another filesystem and
`--min-free-space-gib` can raise the floor. A successful diagnosis exits zero
even when action is required; automation keys on `status` and finding codes.

With a bag, the dispatcher delegates to the existing
`preflight_autoware_map_bag.py` implementation. Topic, PointCloud2 field,
timestamp, and maintained-profile behavior is not forked. System-only storage
options are rejected in bag mode rather than ignored.

The interactive no-argument home adds **Check this installation** before full
help. It prints the exact `lidarslam-map doctor` command and runs immediately
without confirmation because the contract proves there is no network or write.
Demo confirmation and own-bag calibration review remain unchanged.

## Verification

| Check | Result |
| --- | --- |
| source/installed ready reports and schema invariants | PASS |
| missing build, runtime file, ROS, CLI, bag reader, and storage findings | stable-code regressions PASS |
| privacy-bounded JSON | local path exclusion PASS |
| bag-mode exact delegation and option separation | PASS |
| TTY home doctor route and unchanged automation behavior | PASS |
| CLI option/help and machine contract | PASS |
| graph product CLI and documentation contracts | PASS |
| focused lidar_slam tests | 37 passed |
| focused graph tests | 22 passed |
| non-symlinked Jazzy install | build/install PASS; helper, manifest, and schema installed |
| fresh-environment absolute installed launcher | `ready`; 53/53 helpers; Jazzy, `ros2`, and `rosbag2_py` ready |
| non-symlinked Humble overlay install | network-isolated immutable image build/install PASS; helper, manifest, and schema installed |
| Humble installed launcher and complete installed-product gate | `ready`; 53/53 helpers; Humble, `ros2`, and `rosbag2_py` ready; PASS |
| Humble report schema | Draft 2020-12 validation PASS; no local path disclosure |
| installed bytecode state | zero cache artifacts before and after doctor |
| complete maintained Python gate | graph: 1,428 passed / 13 skipped / 11 existing warnings; lidar_slam: 670 passed; 2,098 total |
| Python style/docstrings/copyright | `ament_flake8` 7 files; `ament_pep257` and `ament_copyright` 2 files; PASS |
| documentation | `mkdocs build --strict`: PASS with pre-existing Material/navigation notices |
| machine formats and shell | 89 versioned candidate JSON files parse; shell syntax and `git diff --check` PASS |

The system doctor now has non-symlinked installed proofs on both Jazzy and
Humble. The Humble overlay was built with network access disabled from the
immutable Humble image digest recorded by the distribution evidence, and its
installed-product gate exercised both doctor modes. The complete public CI
matrix still must run on the exact future candidate before it can be proposed
publicly.

## Honest boundary

This increment improves diagnosis and first-command confidence; it does not
create a PPA, Debian package, release, public benchmark, or independent first
map. It also does not prove parity from a feature list. After source publication,
the scorecard must measure command discovery, clean installation, fixed-demo
completion, failure recovery, and active operator time on equivalent Humble and
Jazzy hosts.

## Low-storage recovery follow-up — 2026-08-16

> Decision: **LOCAL_ACTIVATION_REPAIR_PASS / PUBLIC_OBSERVATION_PENDING**
>
> Implementation tip:
> `d01652080485bc68354f354043e4b2e732439223`
>
> Safety floor changed: **no; remains 8 GiB by default**
>
> Network, GitHub, release, or community mutations: **none**

An actual fixed-demo preflight on the Jazzy source candidate reproduced the
remaining recovery gap. With about 6.24 GiB free, system doctor identified low
storage but formerly returned a `<dir>` placeholder; demo dry-run only said to
free space or choose another directory. The operator therefore had to compute
the shortage and reconstruct a command before retrying.

At the implementation tip, both versioned reports expose exact
`additional_bytes_required`. Human output rounds the shortage upward to the
next 0.01 GiB, so it never understates what must be freed. System-doctor JSON
continues to omit the selected local path and returns the placeholder-free
`lidarslam-map doctor` retry. Demo JSON already permits its selected paths and
now retains the complete shell-quoted demo command, including paths with
spaces and all effective storage/viewer options.

The same host then reported:

```text
Demo storage: 6.2 GiB free; 8.0 GiB required; free 1.76 GiB more
additional_bytes_required: 1884504064
Next: Free at least 1.76 GiB ... then run: <complete command>
```

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| doctor/demo focused and schema tests | 28 passed |
| CLI, installed-contract, home, and completion focus | 66 passed |
| complete sourced `lidarslam/test` | 992 passed |
| complete sourced `graph_based_slam/test` | 1,442 passed, 13 skipped, 11 pre-existing ImageIO warnings |
| registered `lidarslam` CTest, including lint | 93 / 93 passed |
| registered `graph_based_slam` CTest, including lint | 232 / 232 passed |
| strict MkDocs and JSON parsing | PASS; only pre-existing Material/navigation notices |
| patch hygiene | `git diff --check` PASS |

This closes one locally observed activation failure. It is not a clean-host
timing result, an independent first map, a paired GLIM scorecard, or evidence
that the unpublished v0.9.1 distribution paths are ready.

## Single recovery action follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_ACTION_RECOVERY_PASS / EXTERNAL_FIRST_ATTEMPT_PENDING**
>
> Implementation tip:
> `a83bbfeaea8196a19513c7a26772d500fe8419b8`
>
> Network, files, GitHub, release, or community mutations performed by the
> observed doctor run: **none**

A real invocation from an unconfigured source-checkout shell retained five
valid findings: missing source install, ROS environment, `ros2`, `rosbag2_py`,
and fixed-demo storage. The previous human card presented a recovery beside
every finding, leaving a beginner to infer dependency order.

The system report now exposes one required top-level `next_action`. In an
`action_required` report it copies the first dependency-ordered finding into
schema-bound `code`, `reason`, and `action` fields; in a `ready` report it is
exactly `null`. The human card renders that selection once under **Do this
now**, retains every remaining stable finding code as a visible follow-up, and
asks the operator to rerun doctor so the remaining state is reprioritized. The
per-finding JSON recovery text remains intact for automation and detailed
inspection.

At the exact implementation tip, the observed five-finding report selected
`source-build-required` and the existing copy-ready
`source_quickstart.sh --build-only` action. Its JSON SHA-256 was
`08c74e4867d5e6848587fa7c5a69c3a72452a7ef461fe8ee4f776e4601d3b4bd`;
the human card SHA-256 was
`4c4071c3882b32076a499847e34c012124a3d5d9d0b3a49bef9541d7e1fe849d`.
Both reported `network_accessed: false` and `writes_performed: false`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| doctor, home, option, completion, and installed-CLI focus | 51 passed |
| exact S6 graph docs/product command | 35 passed |
| exact S6 integrated product/growth command | 321 passed |
| G0 dashboard regressions | 21 passed |
| schema status/action coupling and first-finding selection | PASS |
| Jazzy `ament_flake8` and `ament_pep257` | PASS |
| strict MkDocs and patch hygiene | PASS |

This removes one locally reproduced decision burden from the existing doctor;
it does not add another diagnosis surface, perform the selected build, prove a
clean-host completion time, create a paired GLIM observation, or claim parity.

## One-action own-bag handoff follow-up — 2026-08-17

> Decision: **LOCAL_ONE_ACTION_BAG_HANDOFF_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `387a002dc7826be267fe600db906f80460e6f270`
>
> Mapping, network, file, GitHub, release, or community mutations performed by
> the observed doctor command: **none**

The product-dispatched bag report formerly exposed the internal beginner
script, browser variant, raw launch command, and every compatible path even
after selecting a primary profile. A first-time operator therefore still had
to choose which mapping command should follow a successful diagnosis.

At the implementation tip, a ready `lidarslam-map doctor <bag>` report keeps
the selected profile and its reasons, but renders exactly one shell-safe
exact-input `lidarslam-map start <bag>` under **Do this now**. If any finding
remains, the report withholds start, points to the first finding, and renders
the exact-input doctor retry. Direct use of the preflight script retains the
detailed developer commands and alternatives; machine-readable preflight and
path-free public-evidence contracts are unchanged. The dispatcher and renderer
round-trip command and bag paths with shell quoting instead of concatenating
untrusted path text.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| bag preflight and shell-safe ready/finding handoffs | 32 passed / 2 dependency skips |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, and patch hygiene | PASS |

This closes one local command-selection gap without running mapping or hiding
expert evidence. It is not a clean-host timing result, a paired external GLIM
observation, an independent first map, a package-manager release, or a parity
or superiority claim.

## Compact own-bag readiness card follow-up — 2026-08-17

> Decision: **LOCAL_COMPACT_BAG_CARD_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `fc87cf86cabba5f55fec47316c6a9a3a4e4cb90f`
>
> Mapping, network, file, GitHub, release, or community mutations performed by
> the observed doctor command: **none**

The preceding one-action change removed command choice but left the product
report shaped like an expert preflight: nine possible input categories, topic
names, per-check reasons, profile reasons, every finding message/action, and
advisory commands appeared before the operator reached support. That evidence
is useful locally, but it makes the default success path harder to scan.

The product-dispatched card is now bounded to status, bag duration and message
count, detected input types without topic/frame names, selected profile, four
check statuses, and one action. A ready fixture is regression-limited to at
most 26 lines. A finding card prints the first message/action, reduces the
remainder to stable codes, withholds start, and gives the exact-input retry.
The card exposes one shell-safe private `doctor <bag> --json` command for full
local detail and the separate path-free public-evidence command. Direct
preflight continues to render every topic, reason, alternative, finding, and
advisory; no machine schema changed.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| compact/direct bag preflight boundaries | 32 passed / 2 dependency skips |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, and patch hygiene | PASS |

This lowers default reading cost without deleting expert diagnostics or
changing automation. It is not an external first-time observation, a paired
GLIM scorecard result, an independent first map, or a parity/superiority claim.

## Single-prompt own-bag start follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_PROMPT_START_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `90c508eef4c6ce6868582bda80e684f14223ea4a`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The compact doctor card now hands a ready bag to `start`, but the next RKO
screen previously displayed an exact second `--yes` command immediately before
the same process asked for confirmation. That made a first-time operator choose
between copying the displayed command and answering the prompt.

The interactive control flow now renders the profile extrinsics once, tells the
operator to review them and answer the fail-closed prompt immediately below,
and omits the redundant `--yes` command. Non-interactive `start`, `setup`, and
dry-run retain the exact reviewed rerun command for automation and copy-paste
use. Decline, EOF, and unreviewed calibration still start no mapping.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 35 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This removes one misleading copy-paste detour without weakening calibration
review or automation. It is not a real first map, clean-host timing result,
paired external GLIM observation, or parity/superiority claim.

## Direct-to-progress confirmed-start follow-up — 2026-08-17

> Decision: **LOCAL_DIRECT_PROGRESS_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `2d0bb84a447e29b940adda4bd432e3d5725c9cc0`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

After the single calibration prompt, confirmed live `start` still rendered the
complete READY setup card and then a second start/progress card. Topics,
transforms, the delegated command, and the setup destination were therefore
repeated after the operator had already made the safety decision.

The confirmed path now skips that repeated review and enters the existing
start/progress card directly. Setup-only and dry-run output retain full selected
inputs, calibration, and delegated command detail because no execution follows.
An unconfirmed non-RKO path also retains that detail before its fail-closed
prompt. The durable `sensor_setup.json`, `session.json`, live progress,
verification, and recovery contracts are unchanged.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 36 passed |
| exact S3 lifecycle command | 71 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This removes duplicate terminal reading without hiding a pending decision or
changing automation. It is not a real first map, clean-host timing result,
paired external GLIM observation, or parity/superiority claim.

## Single-card map-completion follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_COMPLETION_CARD_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `8a620e54a121f5ac45913791b40b5239a59f5885`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

Successful `start` previously printed a completion block with setup, map, and
`Reopen`, then wrote session paths, then printed a second summary whose `Next`
usually repeated the reopen command. A viewer failure added another `Reopen
later` line. The operator therefore had to reconcile two or three terminal
handoffs after the map had already finished.

Terminal success is now one `Map session: VERIFIED` or `UNVERIFIED` card. It
contains the map output, evidence-backed verification state, viewer state,
session index/page, run manifest, first-map receipt, and exactly one recommended
`Next`. A verified card retains the privacy-safe `Share` action. Viewer failure
makes the single `Next` the view retry, emits one warning, and does not replace
verified map success. If the derived session index cannot be written, one
completed fallback card still preserves the map path and view command.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 37 passed |
| exact S3 lifecycle command | 72 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This turns map completion into one decision point without changing structured
session, progress, verification, or recovery evidence. It is not a real first
map, clean-host timing result, paired external GLIM observation, or
parity/superiority claim.

## One-action failed-map recovery follow-up — 2026-08-17

> Decision: **LOCAL_ONE_ACTION_RECOVERY_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `14081ea101744b868b80d900bb5a1c42b4ad5046`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The previous ACTION REQUIRED terminal card printed every finding message and
action, the primary `next_command` again, a safe retry, an inspect alternative,
and multiple evidence paths. Those are useful recovery records, but they made a
failed first map begin with several competing commands.

The default card is now bounded to the first stable reason, remaining finding
codes without secondary prose or actions, exactly one safe `Next`, and one
`Details` path. The detail preference is the human session page, then canonical
recovery JSON, session JSON, and finally the preserved setup bundle. Every
finding/action, retry, inspect command, evidence path, resume condition, and
fresh-output rule remains unchanged in `map_session_recovery.json` and the
derived session handoff.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 38 passed |
| exact S3 lifecycle command | 73 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This gives failed mapping one immediate repair step without deleting expert or
machine recovery evidence. It is not a real failure recovery, clean-host timing
result, paired external GLIM observation, or parity/superiority claim.

## Bounded long-stage heartbeat follow-up — 2026-08-17

> Decision: **LOCAL_BOUNDED_HEARTBEAT_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `e2043c0f324ba8fb855b3a723cb670acd40cb2ad`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

Previously, a delegated stage that legitimately ran for several minutes stayed
silent until its durable run-manifest stage changed. That preserved evidence
correctness but could make a healthy first map look stuck.

An unchanged non-complete stage now emits at most one terminal heartbeat every
30 seconds. It reuses the durable stage label and reports monotonic elapsed time
only. A heartbeat writes neither `session.json` nor `session.html`, and it emits
no percentage, ETA, or assertion that the delegated workflow advanced. Durable
stage transitions remain the sole progress-artifact update boundary.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 39 passed |
| exact S3 lifecycle command | 74 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This keeps long mapping visibly active without inventing progress or increasing
artifact churn. The test uses a mocked monitor clock and stage source; it is not
a real long mapping run, clean-host timing result, paired external GLIM
observation, or parity/superiority claim.

## Safe operator interruption follow-up — 2026-08-17

> Decision: **LOCAL_SAFE_INTERRUPTION_PASS / REAL_MAPPING_TRIAL_PENDING**
>
> Implementation tip:
> `6d1249e45a2c91d3b5794f2c6f65eebf19336299`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The lower map runner already isolated its ROS workflow, forwarded SIGINT and
SIGTERM, reaped that process group, and wrote an interrupted terminal manifest.
The product-level `start` still delegated through `subprocess.run`, so its own
KeyboardInterrupt could escape before that lower cleanup and evidence sealing
finished.

`start` now supervises the delegated runner in a separate session. One Ctrl-C
is forwarded as SIGINT, then the product waits up to 20 seconds for bounded
cleanup and terminal evidence. If needed, it requests termination, waits 10
more seconds, and force-reaps the delegated runner. SIGTERM enters the same
supervision boundary. The resulting non-zero status flows through the existing
`workflow-interrupted` recovery receipt, session page, one `Next`, and one
`Details` path; no verified success or Python traceback is synthesized.

A real process-level regression starts a synthetic 60-second delegated child,
sends SIGINT only to the product supervisor, and verifies exit 130 plus an
absent child PID. This exercises signal forwarding and reap behavior without
running ROS mapping.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 42 passed |
| exact S3 lifecycle command | 77 passed |
| exact S3 edit/merge command | 15 passed |
| complete lower map-runner regressions | 48 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This turns one stop request into a bounded evidence-preserving handoff. It is
not a real interrupted mapping trial, clean-host timing result, paired external
GLIM observation, or parity/superiority claim.

## Real ROS interruption correction and trial — 2026-08-17

> Decision: **REAL_ROS_INTERRUPTION_PASS / DEFAULT_STORAGE_AND_CLEAN_HOST_PENDING**
>
> Corrected implementation tip:
> `0301a0d269db4f45b3c8471d3cbf6622e9124337`
>
> Original synthetic-only implementation tip:
> `6d1249e45a2c91d3b5794f2c6f65eebf19336299`
>
> Local ROS mapping performed: **yes, interruption trial only**
>
> Network, GitHub, release, or community mutation performed: **none**

The first real ROS attempt at clean local tip `a24879c…` found a gap that the
synthetic child probe did not cover. Terminal Ctrl-C reached the outer stable
CLI and its `subprocess.run` dispatcher killed the `start` helper. The inner
runner process then received SIGINT without its complete descendant group, so
the ROS workflow survived to a successful map while `session.json` remained
`running`. The terminal contained a Python `KeyboardInterrupt` traceback and no
recovery receipt. The retained red-run hashes bind stdout `63a4f808…`, stderr
`ffb2e720…`, stale session `ac9d2681…`, and successful lower manifest
`f7cb081f…`; that map is test output, not trusted interruption evidence.

Correction `0301a0d…` replaces the stable dispatcher's `subprocess.run` with a
wait-and-forward supervisor. The outermost CLI owns an isolated command group;
nested CLI dispatch waits for the already-signalled group instead of sending a
duplicate signal. The `start` helper now sends SIGINT, SIGTERM, and final
SIGKILL to the complete delegated group rather than only its leader. A new real
dispatcher regression sends SIGINT only to the supervisor and verifies that
both the delegated process and its descendant are reaped without traceback.

The green trial used ROS 2 Jazzy, the same public 50-second MID360 fixture ZIP
(`20e51517…`, 98,873,952 bytes), bag metadata `d866804b…`, and sqlite storage
`0a38fbcc…`. The `lidarslam` copy install was clean exact `0301a0d…`; the
graph/scanmatcher overlay was exact ancestor `d8d2eab…` with no runtime-source
change through the tested tip, and the RKO-LIO source was unchanged from base
`3f4dd70…`. This controlled overlay is not a clean-host package-manager result.

After `workflow_running` and the live ROS process tree were observed, one
terminal Ctrl-C returned 130 in 1.5 seconds. The durable run covers 11.807
seconds from start to sealed manifest and ends `interrupted / complete`, with
`map workflow interrupted by SIGINT`, required verification not completed, and
no completed `map.pcd` or Lanelet2 geometry. All trial descendants were absent.
The final terminal projection contains exactly one `ACTION REQUIRED`, one
`Next`, one `Details`, and one stop request, with no traceback, VERIFIED, or
UNVERIFIED card.

The run manifest, recovery receipt, session index, and first-map receipt each
validate against their tracked schemas. All 17 manifest-bound artifact sizes
and SHA-256 values revalidate. The final stdout, stderr, session, manifest, and
recovery hashes are respectively `ff635663…`, `46d24440…`, `ef17c7b…`,
`8beb0907…`, and `657660d5…`.

Verification on the corrected tip:

| Check | Result |
| --- | --- |
| CLI home plus complete sensor-setup regressions | 51 passed |
| exact S3 lifecycle command | 77 passed |
| complete lower map-runner regressions | 48 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and 17 artifact checksums | PASS |
| changed-code Jazzy `ament_flake8`, JSON, bytecode, and patch hygiene | PASS |

The host had only 1.58 GiB free, so this bounded trial explicitly used a 0.5
GiB floor. The unchanged default storage check correctly remained unsatisfied;
this result is not permission to lower that default. It also is not a complete
map, map-quality or accuracy result, clean-host timing result, independent
first map, paired GLIM observation, or parity/superiority claim.
