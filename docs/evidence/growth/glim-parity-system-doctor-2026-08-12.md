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
